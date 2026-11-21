//! Joy-Con input: SDL3 runs on its own thread, fuses the IMU and publishes [`Hands`].
//! CONTRACT: other modules only read `Res<Hands>`. Field names/meaning below are the interface.
//!
//! # How it works
//!
//! * A dedicated `std::thread` owns SDL (no video): `SDL_Init(SDL_INIT_GAMEPAD)` and a
//!   `SDL_WaitEventTimeout(4 ms)` loop.
//! * Two topologies are supported (see `SDL_hidapi_switch.c` / `SDL_hidapijoystick.c`):
//!   - one *combined pair* pad (`SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_PAIR`) that exposes
//!     `GYRO_L/ACCEL_L/GYRO_R/ACCEL_R` and both sticks (SDL's default
//!     `SDL_HINT_JOYSTICK_HIDAPI_COMBINE_JOY_CONS=1` behaviour);
//!   - two *separate* pads (one `JOYCON_LEFT`, one `JOYCON_RIGHT`), each exposing plain
//!     `GYRO/ACCEL`. `SDL_HINT_JOYSTICK_HIDAPI_VERTICAL_JOY_CONS=1` is forced so a lone
//!     Joy-Con maps its buttons/axes like one half of a pair instead of SDL's "mini gamepad"
//!     horizontal layout.
//! * Each `SDL_EVENT_GAMEPAD_SENSOR_UPDATE` sample is fed to a per-hand Mahony filter using
//!   its `sensor_timestamp` (the Switch driver emits 3 samples per BT report => ~200 Hz).
//! * The neutral ("remote grip") pose is the contract: Joy-Con top (ZL/ZR end) pointing at the
//!   screen, button face/stick up. See [`device_to_body`] for the exact axis mapping.
//!
//! Everything in [`Diagnostics`] is extra, non-contract information for the `joycon_probe`
//! example and debug HUDs; gameplay code should not depend on it.

use std::collections::VecDeque;
use std::ffi::CStr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use bevy::math::EulerRot;
use bevy::prelude::*;
use sdl3_sys::everything::*;

/// Per-hand state, already in Bevy body space: -Z forward (toward the screen), +Y up, +X right.
#[derive(Clone, Debug)]
pub struct HandInput {
    pub connected: bool,
    /// Joy-Con orientation relative to the player's body. Identity = "remote grip":
    /// Joy-Con top (ZL/ZR end) pointing at the screen, face (stick) up. Yaw is recentered, pitch/roll absolute.
    pub orientation: Quat,
    /// Stick, deadzoned, x right / y up, each in [-1, 1].
    pub stick: Vec2,
    /// ZL (left) / ZR (right): grab.
    pub trigger: bool,
    /// L (left) / R (right).
    pub shoulder: bool,
    /// Face buttons. Left Joy-Con: d-pad as [up, down, left, right]. Right: [X, B, Y, A] (same positions).
    pub face: [bool; 4],
    pub stick_click: bool,
    /// Minus (left) / Plus (right).
    pub menu: bool,
    /// Capture (left) / Home (right).
    pub extra: bool,
    /// Calibrated gyro (rad/s) and accel (m/s^2) in body space, for gestures/debug.
    pub gyro: Vec3,
    pub accel: Vec3,
    /// Hand depth from the accelerometer, as an offset along the pointing direction in the
    /// recentered body frame (metres, **-Z forward**, +Y up, +X right). Zero at the model pose.
    ///
    /// **On by default when this hand has a calibration file** (`JOYREACH_REACH=0` forces it
    /// off, `=1` forces it on even without calibration).
    ///
    /// Each push or pull is integrated on its own, from rest to rest, and corrected at the stop
    /// (see `ReachTracker`). Long or rotation-heavy movements are discarded, and nothing is
    /// integrated while the hand is calm, so the depth does not drift. It holds between
    /// movements, returns to the model pose slowly after 3 s of calm, is clamped to
    /// [−0.30, +0.35] m, and resets on recenter/reconnect.
    pub reach: Vec3,
    /// Velocity of [`HandInput::reach`] (body-space m/s).
    pub reach_velocity: Vec3,
    /// Measured IMU sample rate (Hz), for the HUD.
    pub imu_hz: f32,
}

impl Default for HandInput {
    fn default() -> Self {
        Self {
            connected: false,
            orientation: Quat::IDENTITY,
            stick: Vec2::ZERO,
            trigger: false,
            shoulder: false,
            face: [false; 4],
            stick_click: false,
            menu: false,
            extra: false,
            gyro: Vec3::ZERO,
            accel: Vec3::ZERO,
            reach: Vec3::ZERO,
            reach_velocity: Vec3::ZERO,
            imu_hz: 0.0,
        }
    }
}

#[derive(Resource, Clone, Debug, Default)]
pub struct Hands {
    pub left: HandInput,
    pub right: HandInput,
}

// ---------------------------------------------------------------------------------------------
// HD Rumble (additive: gameplay writes `Rumble` messages, the SDL thread does the call)
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RumbleHand {
    Left,
    Right,
}

impl RumbleHand {
    fn index(self) -> usize {
        match self {
            RumbleHand::Left => 0,
            RumbleHand::Right => 1,
        }
    }
}

/// A vibration pulse request. `strength` is 0..1, `ms` is the duration.
///
/// Mapping (verified in SDL 3.4.20): a combined Joy-Con pair exposes two HIDAPI children.
/// `SDL_hidapi_combined.c:109-120` forwards the same `low`/`high` pair to both children,
/// and `SDL_hidapi_switch.c:1783-1790` zeroes `high` for the left child and `low` for the
/// right one. So for a combined pad, low frequency = left Joy-Con, high = right Joy-Con.
/// A lone Joy-Con pad (no parent) gets both amplitudes on its two actuators.
#[derive(Message, Clone, Copy, Debug)]
pub struct Rumble {
    pub hand: RumbleHand,
    pub strength: f32,
    pub ms: u32,
}

struct RumbleCommand {
    hand: RumbleHand,
    strength: f32,
    ms: u32,
    /// Set by the probe to report whether the actual SDL call succeeded.
    ack: Option<mpsc::Sender<bool>>,
}

// ---------------------------------------------------------------------------------------------
// Axis mapping (the one place SDL's device frame is converted to our body frame)
// ---------------------------------------------------------------------------------------------

/// SDL device frame -> Bevy body frame, as a single named transform.
///
/// SDL's gamepad sensor frame (`SDL_sensor.h`, "for game controllers held in front of you") is
/// `+X` right, `+Y` up (out of the button face when the pad lies flat: "at rest, gravity reads
/// as positive Y"), `+Z` toward the player, so `-Z` is the shoulder/ZL-ZR end. The HIDAPI
/// Switch driver rotates the raw Joy-Con IMU into that frame (`SDL_hidapi_switch.c:2389-2423`),
/// incl. the right Joy-Con's 180° flip; `VERTICAL_JOY_CONS=1` skips the "mini gamepad" swap.
///
/// The contract's neutral pose ("remote grip": face up, ZL/ZR end at the screen) is exactly
/// that frame, so the mapping is the identity. Verified on hardware: both Joy-Cons lying flat
/// face-up read accel ≈ (0.5, +10, 0.3) m/s² in the device frame.
#[inline]
pub fn device_to_body(v: Vec3) -> Vec3 {
    v
}

/// Yaw / pitch / roll in degrees of a body-space orientation (for HUDs and `joycon_probe`).
/// Yaw 0 = top pointing at the screen; positive pitch = nose up.
#[allow(dead_code)] // diagnostics only; used by examples/joycon_probe.rs
pub fn orientation_yaw_pitch_roll_deg(q: Quat) -> Vec3 {
    let (yaw, pitch, roll) = q.to_euler(EulerRot::YXZ);
    Vec3::new(yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees())
}

// ---------------------------------------------------------------------------------------------
// Accelerometer calibration (per axis, per hand)
// ---------------------------------------------------------------------------------------------

/// Per-axis accelerometer correction, in the SDL device frame (which is also our body frame,
/// see [`device_to_body`]):
///
/// `a_cal = scale * (a_raw - offset)`
///
/// Six parameters (three scales, three offsets) fitted so that `|a_cal| = GRAVITY` for still
/// samples in at least six orientations. Real Joy-Cons have per-axis scale errors of a few
/// percent and offsets of ~0.1-0.5 m/s². Subtracting only a scalar `ĝ·Y` cannot remove them:
/// when the user only ROTATES the wrist, the uncalibrated magnitude swings by up to ~1 m/s²,
/// the residual is integrated as linear motion and the hand "drifts" up to the reach clamp.
///
/// The fit is a Gauss-Newton/Levenberg least-squares over `r = |S·(a - b)| - GRAVITY`, with
/// the scales clamped positive (the constraint is invariant to a sign flip per axis; starting
/// at scale 1 selects the physical solution). No new crates: the 6x6 normal equations are
/// solved with a tiny Gaussian elimination.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccelCalib {
    pub scale: Vec3,
    pub offset: Vec3,
}

impl Default for AccelCalib {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl AccelCalib {
    pub const IDENTITY: Self = Self {
        scale: Vec3::ONE,
        offset: Vec3::ZERO,
    };

    #[inline]
    pub fn apply(&self, raw_dev: Vec3) -> Vec3 {
        self.scale * (raw_dev - self.offset)
    }

    /// Reject nonsense fits (a swapped axis or a glitched pose must not be saved).
    pub fn is_usable(&self) -> bool {
        let s = self.scale;
        let b = self.offset;
        s.is_finite()
            && b.is_finite()
            && s.min_element() > 0.5
            && s.max_element() < 2.0
            && b.abs().max_element() < 2.0
    }

    /// Fit `scale`/`offset` from mean still-sample vectors taken in >= 6 distinct orientations.
    ///
    /// Returns `(calibration, rms_before, rms_after)` where the RMS is in m/s² of the
    /// `|a| - GRAVITY` residual (before = as if uncalibrated, i.e. scale 1 / offset 0).
    pub fn fit(still: &[Vec3]) -> Option<(Self, f32, f32)> {
        if still.len() < 6 {
            return None;
        }
        let g = GRAVITY as f64;
        // [sx, sy, sz, bx, by, bz]
        let mut p = [1.0_f64, 1.0, 1.0, 0.0, 0.0, 0.0];

        let cost = |p: &[f64; 6]| -> f64 {
            still
                .iter()
                .map(|&a| {
                    let c = calibration_at(*p).apply(a);
                    let r = c.length() as f64 - g;
                    r * r
                })
                .sum()
        };

        let mut lambda = 1e-3_f64;
        let mut current = cost(&p);
        for _ in 0..80 {
            let mut normal = [[0.0_f64; 6]; 6];
            let mut rhs = [0.0_f64; 6];
            for &a in still {
                let out = calibration_at(p);
                let c = out.apply(a);
                let len = c.length().max(1e-6);
                let r = len as f64 - g;
                let d = (a - out.offset).as_dvec3();
                let c_hat = c / len;
                let j = [
                    c_hat.x as f64 * d.x,
                    c_hat.y as f64 * d.y,
                    c_hat.z as f64 * d.z,
                    -(c_hat.x as f64) * out.scale.x as f64,
                    -(c_hat.y as f64) * out.scale.y as f64,
                    -(c_hat.z as f64) * out.scale.z as f64,
                ];
                for i in 0..6 {
                    rhs[i] -= j[i] * r;
                    for k in 0..6 {
                        normal[i][k] += j[i] * j[k];
                    }
                }
            }

            let mut accepted = false;
            for _ in 0..14 {
                let mut matrix = normal;
                for i in 0..6 {
                    // Scale-aware Levenberg damping: large-parameter directions move less.
                    matrix[i][i] += lambda * (1.0 + normal[i][i]);
                }
                let mut delta = rhs;
                let Some(step) = solve6(matrix, &mut delta) else {
                    lambda = (lambda * 10.0).min(1e6);
                    continue;
                };
                let mut trial = p;
                let mut step_max = 0.0_f64;
                for i in 0..6 {
                    trial[i] += step[i];
                    step_max = step_max.max(step[i].abs());
                }
                for k in 0..3 {
                    trial[k] = trial[k].clamp(0.5, 2.0);
                }
                for k in 3..6 {
                    trial[k] = trial[k].clamp(-2.0, 2.0);
                }
                let trial_cost = cost(&trial);
                if trial_cost < current {
                    p = trial;
                    current = trial_cost;
                    lambda = (lambda * 0.3).max(1e-9);
                    accepted = true;
                    if step_max < 1e-10 {
                        // Converged: the remaining iterations cannot change the fit.
                        lambda = 1e9;
                    }
                    break;
                }
                lambda = (lambda * 10.0).min(1e6);
            }
            if !accepted || lambda >= 1e9 {
                break;
            }
        }

        let calib = calibration_at(p);
        if !calib.is_usable() {
            return None;
        }
        let before = Self::residual_rms(still, &Self::IDENTITY);
        let after = Self::residual_rms(still, &calib);
        // Six well-spread orientations must fit better than this; anything else is a bad run.
        if !after.is_finite() || after > 0.5 {
            return None;
        }
        Some((calib, before, after))
    }

    /// RMS of `|a_cal| - GRAVITY` over the given still samples (m/s²).
    pub fn residual_rms(still: &[Vec3], calib: &Self) -> f32 {
        if still.is_empty() {
            return 0.0;
        }
        let sum: f32 = still
            .iter()
            .map(|&a| {
                let r = calib.apply(a).length() - GRAVITY;
                r * r
            })
            .sum();
        (sum / still.len() as f32).sqrt()
    }

    /// Text lines in the calibration file format (see [`ImuCalibration`]).
    fn text_lines(&self, hand: &str) -> String {
        let (s, o) = (self.scale, self.offset);
        format!(
            "{hand} scale {:.6} {:.6} {:.6}\n{hand} offset {:.6} {:.6} {:.6}\n",
            s.x, s.y, s.z, o.x, o.y, o.z
        )
    }
}

fn calibration_at(p: [f64; 6]) -> AccelCalib {
    AccelCalib {
        scale: Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32),
        offset: Vec3::new(p[3] as f32, p[4] as f32, p[5] as f32),
    }
}

/// Gaussian elimination with partial pivoting for a 6x6 system. Returns `None` if singular.
fn solve6(mut a: [[f64; 6]; 6], b: &mut [f64; 6]) -> Option<[f64; 6]> {
    for col in 0..6 {
        let mut pivot = col;
        for row in col + 1..6 {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        if pivot != col {
            a.swap(pivot, col);
            b.swap(pivot, col);
        }
        let inv = 1.0 / a[col][col];
        for row in col + 1..6 {
            let factor = a[row][col] * inv;
            if factor == 0.0 {
                continue;
            }
            for k in col..6 {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0_f64; 6];
    for row in (0..6).rev() {
        let mut acc = b[row];
        for k in row + 1..6 {
            acc -= a[row][k] * x[k];
        }
        x[row] = acc / a[row][row];
    }
    Some(x)
}

// ---------------------------------------------------------------------------------------------
// Diagnostics (non-contract; used by examples/joycon_probe.rs and optional debug HUDs)
// ---------------------------------------------------------------------------------------------

/// Which SDL topology a hand is currently fed by.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Topology {
    #[default]
    None,
    /// One combined `JOYCON_PAIR` pad exposing `*_L` and `*_R` sensors.
    CombinedPair,
    /// A lone left `JOYCON_LEFT` pad (plain `GYRO/ACCEL`).
    SingleLeft,
    /// A lone right `JOYCON_RIGHT` pad (plain `GYRO/ACCEL`).
    SingleRight,
}

impl Topology {
    #[allow(dead_code)] // diagnostics only
    pub fn label(self) -> &'static str {
        match self {
            Topology::None => "none",
            Topology::CombinedPair => "combined_pair",
            Topology::SingleLeft => "single_left",
            Topology::SingleRight => "single_right",
        }
    }
}

/// One SDL sensor we enabled on a pad, plus its reported data rate.
#[derive(Clone, Copy, Debug, Default)]
#[allow(dead_code)] // diagnostics only
pub struct SensorStatus {
    pub name: &'static str,
    pub rate_hz: f32,
}

/// Per-hand debug snapshot. Raw values are in the SDL device frame, body values in Bevy body space.
#[derive(Clone, Debug, Default)]
#[allow(dead_code)] // diagnostics only
pub struct HandDebug {
    pub connected: bool,
    pub topology: Topology,
    pub gamepad_name: String,
    pub gamepad_type: String,
    pub sensors: Vec<SensorStatus>,
    pub raw_gyro: Vec3,
    pub raw_accel: Vec3,
    pub body_gyro: Vec3,
    pub body_accel: Vec3,
    pub gyro_bias: Vec3,
    pub bias_ready: bool,
    /// True when the per-axis accelerometer calibration file is in use for this hand.
    pub accel_calibrated: bool,
    pub accel_calib_scale: Vec3,
    pub accel_calib_offset: Vec3,
    /// This hand's measured gravity magnitude (m/s²), used by the reach dead reckoning.
    pub reach_gravity: f32,
    /// Reach offset / velocity (body space, metres and m/s).
    pub reach: Vec3,
    pub reach_velocity: Vec3,
    /// The arm model's own wrist motion during the current movement (subtracted from it).
    pub reach_model_explained: Vec3,
    /// The current movement's integrated displacement minus the model's wrist motion.
    pub reach_residual: Vec3,
    /// Whether IMU depth is enabled for this hand.
    pub reach_enabled: bool,
    pub imu_hz: f32,
    /// SDL errors / unavailable sensors collected while opening this pad.
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Default)]
#[allow(dead_code)] // diagnostics only
pub struct Diagnostics {
    pub left: HandDebug,
    pub right: HandDebug,
}

// ---------------------------------------------------------------------------------------------
// Bevy glue
// ---------------------------------------------------------------------------------------------

pub struct JoyConPlugin;

impl Plugin for JoyConPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Hands>();
        app.insert_resource(JoyConWorker::spawn());
        app.add_message::<Rumble>();
        app.add_systems(PreUpdate, copy_hands_into_resource);
        app.add_systems(Update, forward_rumble);
    }
}

fn copy_hands_into_resource(worker: Res<JoyConWorker>, mut hands: ResMut<Hands>) {
    *hands = worker.snapshot();
}

/// Turns gameplay `Rumble` messages into commands for the SDL thread.
fn forward_rumble(worker: Res<JoyConWorker>, mut messages: MessageReader<Rumble>) {
    for message in messages.read() {
        worker.rumble(message.hand, message.strength, message.ms);
    }
}

/// One raw IMU sample, emitted at the rate the Switch driver produces them (for
/// `joycon_probe --record` and the CSV replay tests). Values are the raw SDL device-frame
/// readings, before calibration or bias correction.
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)] // used by examples/joycon_probe.rs
pub struct ImuSample {
    pub ts_ns: u64,
    pub hand: Hand,
    pub gyro: Vec3,
    pub accel: Vec3,
}

/// Handle to the SDL thread. Dropping it asks the thread to stop and joins it (this happens when
/// Bevy drops its resources on exit). Also usable without Bevy, which is what the probe does.
#[derive(Resource)]
#[allow(dead_code)] // `debug`/`stop` are used by examples/joycon_probe.rs and Drop
pub struct JoyConWorker {
    hands: Arc<Mutex<Hands>>,
    debug: Arc<Mutex<Diagnostics>>,
    stop: Arc<AtomicBool>,
    rumble_tx: mpsc::Sender<RumbleCommand>,
    thread: Option<JoinHandle<()>>,
}

impl JoyConWorker {
    pub fn spawn() -> Self {
        Self::spawn_with_recorder(None)
    }

    /// Spawn the SDL thread, optionally streaming every raw IMU sample to `recorder`.
    /// The game uses [`JoyConWorker::spawn`]; `joycon_probe --record/--calibrate` uses this.
    #[allow(dead_code)] // used by examples/joycon_probe.rs
    pub fn spawn_with_recorder(recorder: Option<mpsc::Sender<ImuSample>>) -> Self {
        let hands = Arc::new(Mutex::new(Hands::default()));
        let debug = Arc::new(Mutex::new(Diagnostics::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (rumble_tx, rumble_rx) = mpsc::channel::<RumbleCommand>();
        // One load per process (the probe and the game both come through here), reported once.
        let calibration = Arc::new(load_imu_calibration());

        let thread = {
            let hands = Arc::clone(&hands);
            let debug = Arc::clone(&debug);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name("joycon-sdl".to_string())
                .spawn(move || {
                    sdl_thread_main(hands, debug, stop, rumble_rx, recorder, calibration)
                })
                .map_err(|e| eprintln!("[joycon] could not spawn SDL thread: {e}"))
                .ok()
        };

        Self {
            hands,
            debug,
            stop,
            rumble_tx,
            thread,
        }
    }

    /// Latest published [`Hands`].
    pub fn snapshot(&self) -> Hands {
        lock(&self.hands).clone()
    }

    /// Latest diagnostics snapshot (may lag the hands slightly; it is built at ~30 Hz).
    #[allow(dead_code)] // used by examples/joycon_probe.rs
    pub fn debug_snapshot(&self) -> Diagnostics {
        lock(&self.debug).clone()
    }

    /// Queue a rumble pulse. Fire-and-forget: returns false only if the SDL thread is gone.
    pub fn rumble(&self, hand: RumbleHand, strength: f32, ms: u32) -> bool {
        self.rumble_tx
            .send(RumbleCommand {
                hand,
                strength,
                ms,
                ack: None,
            })
            .is_ok()
    }

    /// Queue a pulse and wait for the SDL call result (for the probe / diagnostics).
    #[allow(dead_code)] // used by examples/joycon_probe.rs
    pub fn rumble_checked(&self, hand: RumbleHand, strength: f32, ms: u32) -> Result<bool, String> {
        let (tx, rx) = mpsc::channel();
        self.rumble_tx
            .send(RumbleCommand {
                hand,
                strength,
                ms,
                ack: Some(tx),
            })
            .map_err(|_| "SDL thread is not running".to_string())?;
        rx.recv_timeout(Duration::from_millis(500))
            .map_err(|_| "no reply from the SDL thread".to_string())
    }

    /// Ask the SDL thread to exit (idempotent; `Drop` does this too).
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Drop for JoyConWorker {
    fn drop(&mut self) {
        self.stop();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

// ---------------------------------------------------------------------------------------------
// SDL thread
// ---------------------------------------------------------------------------------------------

/// Event-loop period. The game samples `Hands` every frame; 4 ms keeps stick latency low while
/// still parking the thread when nothing happens.
const POLL_TIMEOUT_MS: i32 = 4;
/// Publish the (allocating) diagnostics snapshot every N loop iterations (~30 Hz).
const DEBUG_EVERY_N: u32 = 8;

fn sdl_thread_main(
    hands: Arc<Mutex<Hands>>,
    debug: Arc<Mutex<Diagnostics>>,
    stop: Arc<AtomicBool>,
    rumble_rx: mpsc::Receiver<RumbleCommand>,
    recorder: Option<mpsc::Sender<ImuSample>>,
    calibration: Arc<ImuCalibration>,
) {
    // Hints must be set before SDL_Init so the HIDAPI device enumeration already sees them.
    unsafe {
        // Static SDL build: make sure the HIDAPI drivers are on regardless of platform defaults.
        set_hint(SDL_HINT_JOYSTICK_HIDAPI, c"1");
        set_hint(SDL_HINT_JOYSTICK_HIDAPI_JOY_CONS, c"1");
        set_hint(SDL_HINT_JOYSTICK_HIDAPI_SWITCH, c"1");
        // A lone Joy-Con then behaves like half of a pair (vertical layout, per-hand sensors keep
        // the non-mini axis mapping in SDL_hidapi_switch.c:2409-2423).
        set_hint(SDL_HINT_JOYSTICK_HIDAPI_VERTICAL_JOY_CONS, c"1");
        // SDL_HINT_JOYSTICK_HIDAPI_COMBINE_JOY_CONS is left at its default (1). If a user sets it
        // to 0 we simply get two separate pads, which the loop handles too.

        if !SDL_Init(SDL_INIT_GAMEPAD) {
            let err = sdl_error();
            eprintln!("[joycon] SDL_Init(SDL_INIT_GAMEPAD) failed: {err}. Joy-Con input disabled.");
            {
                let mut dbg = lock(&debug);
                let note = format!("SDL_Init(SDL_INIT_GAMEPAD) failed: {err}");
                dbg.left.notes.push(note.clone());
                dbg.right.notes.push(note);
            }
            return;
        }
    }

    eprintln!("[joycon] SDL3 gamepad subsystem ready (static build, HIDAPI Switch driver)");

    let mut pads: Vec<Pad> = Vec::new();

    // Devices that were already connected before we started: open them now. SDL will also queue
    // GAMEPAD_ADDED events for them; `open_joycon` is idempotent per instance id.
    unsafe {
        let mut count: std::ffi::c_int = 0;
        let ids = SDL_GetGamepads(&mut count);
        if !ids.is_null() {
            for i in 0..count.max(0) as isize {
                let id = *ids.offset(i);
                if let Some(pad) = open_joycon(id, &calibration, recorder.clone()) {
                    pads.push(pad);
                }
            }
            SDL_free(ids as *mut std::ffi::c_void);
        }
        SDL_ClearError();
    }

    if pads.is_empty() {
        eprintln!("[joycon] no Joy-Con detected yet (asleep? press a button to wake them)");
    }

    let mut event: SDL_Event = unsafe { std::mem::zeroed() };
    let mut frame: u32 = 0;
    let mut rumble_state = RumbleState::default();

    while !stop.load(Ordering::Relaxed) {
        let got_event = unsafe { SDL_WaitEventTimeout(&mut event, POLL_TIMEOUT_MS) };
        if got_event {
            loop {
                handle_event(&event, &mut pads, &calibration, &recorder);
                let more = unsafe { SDL_PollEvent(&mut event) };
                if !more {
                    break;
                }
            }
        }

        // Buttons/sticks are polled (cheap) instead of tracked per-event, so the snapshot is
        // always coherent with the latest SDL state.
        let mut recenter = false;
        for pad in &mut pads {
            recenter |= unsafe { poll_pad(pad) };
        }
        // Minus OR Plus recenters both hands (yaw only): you point both Joy-Cons at the screen
        // and press whichever button your thumb finds.
        if recenter {
            for rt in pads
                .iter_mut()
                .flat_map(|p| [p.left.as_mut(), p.right.as_mut()])
                .flatten()
            {
                rt.fusion.recenter();
                rt.reach.reset();
                rt.input.orientation = rt.fusion.orientation();
            }
        }

        // Rumble: commands set a per-hand pulse, then the owning pad gets the amplitudes
        // immediately (with the ack of the real SDL call, for the probe).
        while let Ok(command) = rumble_rx.try_recv() {
            let now = Instant::now();
            let amplitude = (command.strength.clamp(0.0, 1.0) * u16::MAX as f32) as u16;
            rumble_state.set(
                command.hand,
                amplitude,
                now + Duration::from_millis(command.ms.max(1) as u64),
            );
            let mut ok = false;
            for pad in pads.iter_mut().filter(|pad| pad.owns(command.hand)) {
                ok |= send_pad_rumble(pad, &rumble_state, now);
            }
            if let Some(ack) = command.ack {
                let _ = ack.send(ok);
            }
        }
        // Expired pulses: send zero to that pad once.
        let now = Instant::now();
        for pad in &mut pads {
            let (low, high, _) = pad_rumble_levels(pad, &rumble_state, now);
            if (low, high) != pad.last_rumble {
                send_pad_rumble(pad, &rumble_state, now);
            }
        }

        frame = frame.wrapping_add(1);
        publish(&pads, &hands, &debug, frame % DEBUG_EVERY_N == 0);
    }

    for pad in pads.drain(..) {
        unsafe { SDL_CloseGamepad(pad.ptr) };
    }
    unsafe { SDL_Quit() };
    eprintln!("[joycon] SDL thread stopped");
}

unsafe fn set_hint(name: *const std::ffi::c_char, value: &std::ffi::CStr) {
    unsafe {
        SDL_SetHint(name, value.as_ptr());
    }
}

fn sdl_error() -> String {
    let ptr = SDL_GetError();
    if ptr.is_null() {
        "unknown SDL error".to_string()
    } else {
        unsafe { CStr::from_ptr(ptr) }
            .to_string_lossy()
            .into_owned()
    }
}

fn cstr(ptr: *const std::ffi::c_char) -> String {
    if ptr.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(ptr) }
            .to_string_lossy()
            .into_owned()
    }
}

// ---------------------------------------------------------------------------------------------
// Pads and hands
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hand {
    Left,
    Right,
}

impl Hand {
    fn index(self) -> usize {
        match self {
            Hand::Left => 0,
            Hand::Right => 1,
        }
    }

    /// Lower-case name used by the calibration file.
    pub fn name(self) -> &'static str {
        match self {
            Hand::Left => "left",
            Hand::Right => "right",
        }
    }

    /// One-letter tag used by `joycon_probe --record` CSVs.
    #[allow(dead_code)] // used by examples/joycon_probe.rs and the CSV replay test
    pub fn short(self) -> &'static str {
        match self {
            Hand::Left => "L",
            Hand::Right => "R",
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Calibration file + guided calibration collector
// ---------------------------------------------------------------------------------------------

/// Both hands' accelerometer calibrations, as loaded from `%LOCALAPPDATA%\joyreach\imu_calib.txt`.
///
/// The file is a hand-parsed text format (no serde):
///
/// ```text
/// # joyreach imu accel calibration v1
/// # a_cal = scale * (a_raw - offset)   [m/s^2, SDL device frame]
/// left scale 1.031200 -0.012300 0.998700
/// left offset 0.081200 -0.042000 0.113000
/// right scale ...
/// right offset ...
/// ```
///
/// A missing file is not an error: both hands then keep the pre-M2c scalar-`ĝ` fallback.
#[derive(Clone, Debug, Default)]
pub struct ImuCalibration {
    pub hands: [Option<AccelCalib>; 2],
}

impl ImuCalibration {
    pub fn get(&self, hand: Hand) -> Option<AccelCalib> {
        self.hands[hand.index()]
    }

    pub fn any(&self) -> bool {
        self.hands.iter().any(Option::is_some)
    }

    pub fn to_text(&self) -> String {
        let mut out = String::from(
            "# joyreach imu accel calibration v1\n\
             # a_cal = scale * (a_raw - offset)   [m/s^2, SDL device frame]\n",
        );
        for hand in [Hand::Left, Hand::Right] {
            if let Some(calib) = self.get(hand) {
                out.push_str(&calib.text_lines(hand.name()));
            }
        }
        out
    }

    /// Parse the text format. Unknown keys / malformed numbers are hard errors so a corrupted
    /// file is reported instead of silently mis-calibrating both hands.
    pub fn parse_text(text: &str) -> Result<Self, String> {
        let mut scales = [None; 2];
        let mut offsets = [None; 2];
        for (line_no, raw_line) in text.lines().enumerate() {
            let line = raw_line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let fields: Vec<&str> = line.split_whitespace().collect();
            if fields.len() != 5 {
                return Err(format!(
                    "line {}: expected `hand scale|offset x y z`, got `{line}`",
                    line_no + 1
                ));
            }
            let hand = match fields[0].to_ascii_lowercase().as_str() {
                "left" | "l" => Hand::Left,
                "right" | "r" => Hand::Right,
                other => return Err(format!("line {}: unknown hand `{other}`", line_no + 1)),
            };
            let mut v = [0.0_f32; 3];
            for (i, field) in fields[2..].iter().enumerate() {
                v[i] = field
                    .parse::<f32>()
                    .map_err(|_| format!("line {}: invalid number `{field}`", line_no + 1))?;
            }
            let value = Vec3::new(v[0], v[1], v[2]);
            match fields[1] {
                "scale" => scales[hand.index()] = Some(value),
                "offset" => offsets[hand.index()] = Some(value),
                other => return Err(format!("line {}: unknown key `{other}`", line_no + 1)),
            }
        }

        let mut out = Self::default();
        for hand in [Hand::Left, Hand::Right] {
            match (scales[hand.index()], offsets[hand.index()]) {
                (Some(scale), Some(offset)) => {
                    let calib = AccelCalib { scale, offset };
                    if !calib.is_usable() {
                        return Err(format!("{}: scale/offset out of range", hand.name()));
                    }
                    out.hands[hand.index()] = Some(calib);
                }
                (None, None) => {}
                _ => {
                    return Err(format!(
                        "{}: the scale and offset lines must both be present",
                        hand.name()
                    ));
                }
            }
        }
        Ok(out)
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::parse_text(&text)
    }

    /// Write the file used by the game at startup (probe `--calibrate`).
    #[allow(dead_code)] // used by examples/joycon_probe.rs
    pub fn save(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        std::fs::write(path, self.to_text()).map_err(|e| e.to_string())
    }
}

/// `%LOCALAPPDATA%\joyreach\imu_calib.txt` (same folder convention as the rest of the game).
pub fn imu_calib_path() -> Option<PathBuf> {
    if let Some(base) = std::env::var_os("LOCALAPPDATA") {
        return Some(PathBuf::from(base).join("joyreach").join("imu_calib.txt"));
    }
    if let Some(home) = std::env::var_os("HOME") {
        return Some(
            PathBuf::from(home)
                .join(".local")
                .join("share")
                .join("joyreach")
                .join("imu_calib.txt"),
        );
    }
    None
}

/// Load the calibration at startup and say clearly (stderr) whether it is in use.
pub fn load_imu_calibration() -> ImuCalibration {
    let Some(path) = imu_calib_path() else {
        eprintln!(
            "[joycon] accel calibration not found (no LOCALAPPDATA/HOME); using scalar gravity fallback"
        );
        let calib = ImuCalibration::default();
        log_reach_decision(&calib);
        return calib;
    };
    if !path.exists() {
        eprintln!(
            "[joycon] accel calibration not found ({}); using scalar gravity fallback",
            path.display()
        );
        let calib = ImuCalibration::default();
        log_reach_decision(&calib);
        return calib;
    }
    match ImuCalibration::load(&path) {
        Ok(calib) if calib.any() => {
            eprintln!(
                "[joycon] accel calibration loaded from {} ({})",
                path.display(),
                calibration_summary(&calib)
            );
            log_reach_decision(&calib);
            calib
        }
        Ok(_) => {
            eprintln!(
                "[joycon] accel calibration not found ({}); using scalar gravity fallback",
                path.display()
            );
            let calib = ImuCalibration::default();
            log_reach_decision(&calib);
            calib
        }
        Err(err) => {
            eprintln!(
                "[joycon] accel calibration invalid ({}): {err}; using scalar gravity fallback",
                path.display()
            );
            let calib = ImuCalibration::default();
            log_reach_decision(&calib);
            calib
        }
    }
}

fn calibration_summary(calib: &ImuCalibration) -> String {
    let mut parts = Vec::new();
    for hand in [Hand::Left, Hand::Right] {
        match calib.get(hand) {
            Some(c) => parts.push(format!(
                "{} scale=({:.4},{:.4},{:.4}) offset=({:+.4},{:+.4},{:+.4})",
                hand.name(),
                c.scale.x,
                c.scale.y,
                c.scale.z,
                c.offset.x,
                c.offset.y,
                c.offset.z
            )),
            None => parts.push(format!("{} none", hand.name())),
        }
    }
    parts.join(", ")
}

/// One still pose collected during `joycon_probe --calibrate`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalibPose {
    /// The window is not complete / not still yet.
    Waiting,
    /// A still, diverse pose; the mean raw accel (SDL device frame) was accepted.
    Accepted(Vec3),
    /// Still, but too close (< 25°) to a pose that was already collected.
    Duplicate(Vec3),
}

/// Guided 6-face calibration helper used by `joycon_probe --calibrate`.
///
/// It reuses the same variance-based stillness idea as the IMU bias windows: a pose is accepted
/// only after ~1 s of low gyro AND low accel variance, so a hand tremor or a bump is ignored.
/// Poses must be at least 25° apart in gravity direction, otherwise the least-squares system is
/// badly conditioned and the fit is meaningless.
#[derive(Clone, Debug, Default)]
pub struct CalibCollector {
    window: VecDeque<(u64, Vec3, Vec3)>,
    step_start: Option<u64>,
    poses: Vec<Vec3>,
}

const CALIB_STILL_NS: u64 = 1_000_000_000;
const CALIB_MIN_SAMPLES: usize = 100;
const CALIB_ACCEL_STD: f32 = 0.15;
const CALIB_GYRO_MEAN: f32 = 0.15;
const CALIB_MIN_POSE_DEG: f32 = 25.0;

#[allow(dead_code)] // used by examples/joycon_probe.rs
impl CalibCollector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start a new guided step: the previous window and its timestamps are discarded so the
    /// previous face cannot be accepted again by accident.
    pub fn begin_step(&mut self) {
        self.window.clear();
        self.step_start = None;
    }

    pub fn push(&mut self, ts_ns: u64, accel: Vec3, gyro: Vec3) {
        let _ = self.step_start.get_or_insert(ts_ns);
        self.window.push_back((ts_ns, accel, gyro));
        while let Some(&(oldest, _, _)) = self.window.front() {
            if ts_ns.saturating_sub(oldest) > CALIB_STILL_NS {
                self.window.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn poses(&self) -> &[Vec3] {
        &self.poses
    }

    pub fn take_pose(&mut self) -> CalibPose {
        let Some(start) = self.step_start else {
            return CalibPose::Waiting;
        };
        let Some(&(newest, _, _)) = self.window.back() else {
            return CalibPose::Waiting;
        };
        let Some(&(oldest, _, _)) = self.window.front() else {
            return CalibPose::Waiting;
        };
        // Fresh samples after `begin_step` AND a full window of them.
        if newest.saturating_sub(start) < CALIB_STILL_NS
            || newest.saturating_sub(oldest) < CALIB_STILL_NS * 3 / 4
            || self.window.len() < CALIB_MIN_SAMPLES
        {
            return CalibPose::Waiting;
        }

        let n = self.window.len() as f32;
        let mut sum_a = Vec3::ZERO;
        let mut sum_a2 = Vec3::ZERO;
        let mut sum_g = 0.0_f32;
        for &(_, accel, gyro) in &self.window {
            sum_a += accel;
            sum_a2 += accel * accel;
            sum_g += gyro.length();
        }
        let mean_a = sum_a / n;
        let variance = (sum_a2 / n - mean_a * mean_a).max(Vec3::ZERO).element_sum();
        if variance.sqrt() > CALIB_ACCEL_STD || sum_g / n > CALIB_GYRO_MEAN {
            return CalibPose::Waiting;
        }

        let direction = mean_a.normalize_or_zero();
        let min_dot = CALIB_MIN_POSE_DEG.to_radians().cos();
        if self
            .poses
            .iter()
            .any(|pose| direction.dot(pose.normalize_or_zero()) > min_dot)
        {
            return CalibPose::Duplicate(mean_a);
        }
        self.poses.push(mean_a);
        CalibPose::Accepted(mean_a)
    }

    pub fn fit(&self) -> Option<(AccelCalib, f32, f32)> {
        AccelCalib::fit(&self.poses)
    }
}

/// Whether IMU hand depth is enabled for this process.
///
/// M2e default: **on when an accelerometer calibration file was loaded** (without one the
/// scalar-ĝ fallback is not good enough for depth). `JOYREACH_REACH=1` forces it on even
/// uncalibrated, `JOYREACH_REACH=0` forces it off. The decision is logged at startup.
pub fn reach_enabled() -> bool {
    reach_policy(reach_override(), reach_calibrated_loaded())
}

/// Per-hand gate: a hand without calibration only reaches when the environment forces it.
fn reach_enabled_for(calibrated: bool) -> bool {
    reach_policy(reach_override(), calibrated)
}

/// Pure policy split out so it can be tested without touching process-global env/state.
fn reach_policy(force: Option<bool>, calibrated: bool) -> bool {
    force.unwrap_or(calibrated)
}

/// `JOYREACH_REACH`: `Some(true)` for `=1`, `Some(false)` for `=0`, `None` (default) otherwise.
fn reach_override() -> Option<bool> {
    static OVERRIDE: std::sync::OnceLock<Option<bool>> = std::sync::OnceLock::new();
    *OVERRIDE.get_or_init(|| match std::env::var("JOYREACH_REACH").ok().as_deref() {
        Some("1") => Some(true),
        Some("0") => Some(false),
        _ => None,
    })
}

/// Set by [`load_imu_calibration`]: at least one hand has a usable per-axis fit.
fn reach_calibrated_loaded() -> bool {
    REACH_CALIBRATED.load(Ordering::Relaxed)
}

static REACH_CALIBRATED: AtomicBool = AtomicBool::new(false);

/// Say clearly, once, why depth is on or off.
fn log_reach_decision(calib: &ImuCalibration) {
    REACH_CALIBRATED.store(calib.any(), Ordering::Relaxed);
    let hands = [Hand::Left, Hand::Right]
        .into_iter()
        .filter_map(|hand| calib.get(hand).map(|_| hand.name()))
        .collect::<Vec<_>>();
    match reach_override() {
        Some(true) => eprintln!(
            "[joycon] reach: ON (forced by JOYREACH_REACH=1; calibrated: {})",
            if hands.is_empty() {
                "none".to_string()
            } else {
                hands.join(",")
            }
        ),
        Some(false) => eprintln!("[joycon] reach: OFF (forced by JOYREACH_REACH=0)"),
        None if calib.any() => eprintln!(
            "[joycon] reach: ON by default (accel calibration loaded for {})",
            hands.join(",")
        ),
        None => eprintln!(
            "[joycon] reach: OFF (no accel calibration; run `joycon_probe --calibrate`; \
             JOYREACH_REACH=1 forces it on)"
        ),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PadKind {
    Combined,
    Left,
    Right,
}

impl PadKind {
    fn topology(self) -> Topology {
        match self {
            PadKind::Combined => Topology::CombinedPair,
            PadKind::Left => Topology::SingleLeft,
            PadKind::Right => Topology::SingleRight,
        }
    }
}

#[derive(Clone, Copy)]
struct SensorBinding {
    ty: SDL_SensorType,
    label: &'static str,
    hand: Hand,
    gyro: bool,
}

struct Pad {
    id: SDL_JoystickID,
    ptr: *mut SDL_Gamepad,
    kind: PadKind,
    gamepad_name: String,
    gamepad_type: String,
    sensors: Vec<SensorBinding>,
    notes: Vec<String>,
    left: Option<HandRuntime>,
    right: Option<HandRuntime>,
    /// Optional raw-sample sink (probe `--record` / `--calibrate`).
    recorder: Option<mpsc::Sender<ImuSample>>,
    /// Loaded per-hand accel calibration, also used for hands created defensively later.
    calibration: [Option<AccelCalib>; 2],
    /// Last low/high amplitudes sent, to only call SDL when they change.
    last_rumble: (u16, u16),
    /// Print at most one "rumble failed" line per pad.
    rumble_warned: bool,
}

impl Pad {
    fn owns(&self, hand: RumbleHand) -> bool {
        match hand {
            RumbleHand::Left => self.left.is_some(),
            RumbleHand::Right => self.right.is_some(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct RumblePulse {
    amplitude: u16,
    until: Option<Instant>,
}

#[derive(Debug, Default)]
struct RumbleState {
    pulses: [RumblePulse; 2],
}

impl RumbleState {
    fn set(&mut self, hand: RumbleHand, amplitude: u16, until: Instant) {
        self.pulses[hand.index()] = RumblePulse {
            amplitude,
            until: Some(until),
        };
    }

    fn level(&self, hand: RumbleHand, now: Instant) -> u16 {
        match self.pulses[hand.index()].until {
            Some(until) if until > now => self.pulses[hand.index()].amplitude,
            _ => 0,
        }
    }

    fn remaining_ms(&self, hand: RumbleHand, now: Instant) -> u32 {
        match self.pulses[hand.index()].until {
            Some(until) if until > now => (until - now).as_millis().max(1) as u32,
            _ => 0,
        }
    }
}

/// The amplitudes to send to one pad, and how long they should last.
/// See [`Rumble`] for the combined-pair channel mapping.
fn pad_rumble_levels(pad: &Pad, state: &RumbleState, now: Instant) -> (u16, u16, u32) {
    let left = state.level(RumbleHand::Left, now);
    let right = state.level(RumbleHand::Right, now);
    let (low, high) = match pad.kind {
        PadKind::Combined => (left, right),
        // A lone Joy-Con has no per-hand split inside SDL: drive both actuators.
        PadKind::Left => (left, left),
        PadKind::Right => (right, right),
    };
    let ms = state
        .remaining_ms(RumbleHand::Left, now)
        .max(state.remaining_ms(RumbleHand::Right, now));
    (low, high, ms)
}

fn send_pad_rumble(pad: &mut Pad, state: &RumbleState, now: Instant) -> bool {
    let (low, high, remaining_ms) = pad_rumble_levels(pad, state, now);
    let duration = if low == 0 && high == 0 {
        0
    } else {
        remaining_ms.max(1)
    };
    let ok = unsafe { SDL_RumbleGamepad(pad.ptr, low, high, duration) };
    if !ok && !pad.rumble_warned {
        pad.rumble_warned = true;
        eprintln!(
            "[joycon] rumble failed on {}: {}",
            pad.gamepad_name,
            sdl_error()
        );
    }
    pad.last_rumble = (low, high);
    ok
}

/// Per-hand state that lives next to its pad (so unplugging the pad unplugs its hands).
struct HandRuntime {
    input: HandInput,
    fusion: ImuFusion,
    reach: ReachTracker,
    /// Last raw sample, SDL device frame.
    last_gyro: Vec3,
    last_accel: Vec3,
    /// Gyro sample waiting for its accel twin (same `sensor_timestamp`).
    pending_gyro: Option<(u64, Vec3)>,
    has_accel: bool,
    prev_menu: bool,
}

impl HandRuntime {
    fn new(hand: Hand, calib: Option<AccelCalib>) -> Self {
        let reach_on = reach_enabled_for(calib.is_some());
        eprintln!(
            "[joycon] {} hand: reach {} ({})",
            hand.name(),
            if reach_on { "ON" } else { "OFF" },
            match (reach_on, calib.is_some()) {
                (true, true) => "calibrated dead reckoning",
                (true, false) => "forced by JOYREACH_REACH=1, scalar gravity fallback",
                (false, true) => "disabled by JOYREACH_REACH=0",
                (false, false) => "no accel calibration; run `joycon_probe --calibrate`",
            }
        );
        Self {
            input: HandInput {
                connected: true,
                ..Default::default()
            },
            fusion: ImuFusion::with_calibration(calib),
            reach: ReachTracker::with_calibration(calib, hand),
            last_gyro: Vec3::ZERO,
            last_accel: Vec3::ZERO,
            pending_gyro: None,
            has_accel: false,
            prev_menu: false,
        }
    }
}

fn is_gyro(ty: SDL_SensorType) -> bool {
    ty == SDL_SENSOR_GYRO || ty == SDL_SENSOR_GYRO_L || ty == SDL_SENSOR_GYRO_R
}

fn sensor_label(ty: SDL_SensorType) -> &'static str {
    if ty == SDL_SENSOR_GYRO {
        "gyro"
    } else if ty == SDL_SENSOR_ACCEL {
        "accel"
    } else if ty == SDL_SENSOR_GYRO_L {
        "gyro_L"
    } else if ty == SDL_SENSOR_ACCEL_L {
        "accel_L"
    } else if ty == SDL_SENSOR_GYRO_R {
        "gyro_R"
    } else if ty == SDL_SENSOR_ACCEL_R {
        "accel_R"
    } else {
        "sensor"
    }
}

/// Open and classify one SDL gamepad. Returns `None` for non-Joy-Con devices.
fn open_joycon(
    id: SDL_JoystickID,
    calibration: &ImuCalibration,
    recorder: Option<mpsc::Sender<ImuSample>>,
) -> Option<Pad> {
    unsafe {
        SDL_ClearError();
        let ptr = SDL_OpenGamepad(id);
        if ptr.is_null() {
            eprintln!("[joycon] SDL_OpenGamepad({}) failed: {}", id.0, sdl_error());
            return None;
        }

        let ty = SDL_GetGamepadType(ptr);
        let Some(kind) = pad_kind_from_type(ty) else {
            eprintln!(
                "[joycon] ignoring non-Joy-Con gamepad {} ({})",
                cstr(SDL_GetGamepadStringForType(ty)),
                cstr(SDL_GetGamepadName(ptr))
            );
            SDL_CloseGamepad(ptr);
            return None;
        };

        let gamepad_name = cstr(SDL_GetGamepadName(ptr));
        let gamepad_type = cstr(SDL_GetGamepadStringForType(ty));
        let left_calib = calibration.get(Hand::Left);
        let right_calib = calibration.get(Hand::Right);

        let mut pad = Pad {
            id,
            ptr,
            kind,
            gamepad_name,
            gamepad_type,
            sensors: Vec::new(),
            notes: Vec::new(),
            left: (kind != PadKind::Right).then(|| HandRuntime::new(Hand::Left, left_calib)),
            right: (kind != PadKind::Left).then(|| HandRuntime::new(Hand::Right, right_calib)),
            recorder,
            calibration: [left_calib, right_calib],
            last_rumble: (0, 0),
            rumble_warned: false,
        };

        enable_pad_sensors(&mut pad);

        let sensor_summary = pad
            .sensors
            .iter()
            .map(|s| s.label)
            .collect::<Vec<_>>()
            .join(", ");
        eprintln!(
            "[joycon] {} connected as {} (topology {}, sensors: [{}])",
            pad.gamepad_name,
            pad.gamepad_type,
            pad.kind.topology().label(),
            sensor_summary
        );
        for note in &pad.notes {
            eprintln!("[joycon] note: {note}");
        }

        Some(pad)
    }
}

/// Enable every sensor the topology should expose. SDL requires the sensor to exist (the Switch
/// driver adds them when it enters enhanced report mode, which happens on open), otherwise the
/// call returns false and we record why.
unsafe fn enable_pad_sensors(pad: &mut Pad) {
    let wanted = sensors_for(pad.kind);

    let try_enable = |pad: &mut Pad, ty: SDL_SensorType, hand: Hand| unsafe {
        SDL_ClearError();
        if SDL_SetGamepadSensorEnabled(pad.ptr, ty, true) {
            pad.sensors.push(SensorBinding {
                ty,
                label: sensor_label(ty),
                hand,
                gyro: is_gyro(ty),
            });
        } else {
            let err = sdl_error();
            if !err.is_empty() {
                pad.notes
                    .push(format!("{} unavailable: {err}", sensor_label(ty)));
            }
        }
    };

    for &(ty, hand) in wanted {
        try_enable(pad, ty, hand);
    }

    // Defensive fallback for drivers that expose the pair through the plain sensor types
    // (SDL's Switch 2 driver does this for the right Joy-Con of a pair).
    if pad.sensors.is_empty() && pad.kind == PadKind::Combined {
        try_enable(pad, SDL_SENSOR_GYRO, Hand::Right);
        try_enable(pad, SDL_SENSOR_ACCEL, Hand::Right);
    }

    if pad.sensors.is_empty() {
        pad.notes
            .push("no IMU sensors could be enabled; orientation will stay at identity".to_string());
    }
}

/// Which SDL gamepad types do we take over? Everything else is ignored (and logged).
fn pad_kind_from_type(ty: SDL_GamepadType) -> Option<PadKind> {
    if ty == SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_PAIR {
        Some(PadKind::Combined)
    } else if ty == SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_LEFT {
        Some(PadKind::Left)
    } else if ty == SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_RIGHT {
        Some(PadKind::Right)
    } else {
        None
    }
}

/// The sensors each topology exposes, and which hand they feed (see
/// `SDL_hidapi_switch.c:894-910`): a combined pair gets `*_L`/`*_R`, a lone Joy-Con gets the
/// plain `GYRO/ACCEL` pair.
fn sensors_for(kind: PadKind) -> &'static [(SDL_SensorType, Hand)] {
    const COMBINED: &[(SDL_SensorType, Hand)] = &[
        (SDL_SENSOR_GYRO_L, Hand::Left),
        (SDL_SENSOR_ACCEL_L, Hand::Left),
        (SDL_SENSOR_GYRO_R, Hand::Right),
        (SDL_SENSOR_ACCEL_R, Hand::Right),
    ];
    const LEFT: &[(SDL_SensorType, Hand)] = &[
        (SDL_SENSOR_GYRO, Hand::Left),
        (SDL_SENSOR_ACCEL, Hand::Left),
    ];
    const RIGHT: &[(SDL_SensorType, Hand)] = &[
        (SDL_SENSOR_GYRO, Hand::Right),
        (SDL_SENSOR_ACCEL, Hand::Right),
    ];

    match kind {
        PadKind::Combined => COMBINED,
        PadKind::Left => LEFT,
        PadKind::Right => RIGHT,
    }
}

fn handle_event(
    event: &SDL_Event,
    pads: &mut Vec<Pad>,
    calibration: &ImuCalibration,
    recorder: &Option<mpsc::Sender<ImuSample>>,
) {
    match unsafe { event.r#type } {
        t if t == SDL_EVENT_GAMEPAD_ADDED.0 => {
            let id = unsafe { event.gdevice.which };
            if pads.iter().any(|p| p.id == id) {
                return;
            }
            if let Some(pad) = open_joycon(id, calibration, recorder.clone()) {
                // A combined pair supersedes any single-pad view of the same hands; drop the
                // singles so we don't straddle two topologies while SDL tears them down.
                if pad.kind == PadKind::Combined {
                    let (singles, old_combined): (Vec<Pad>, Vec<Pad>) =
                        pads.drain(..).partition(|p| p.kind != PadKind::Combined);
                    for old in old_combined {
                        eprintln!("[joycon] {} superseded by combined pair", old.gamepad_name);
                        unsafe { SDL_CloseGamepad(old.ptr) };
                    }
                    pads.extend(singles);
                }
                pads.push(pad);
            }
        }
        t if t == SDL_EVENT_GAMEPAD_REMOVED.0 => {
            let id = unsafe { event.gdevice.which };
            if let Some(pos) = pads.iter().position(|p| p.id == id) {
                let pad = pads.remove(pos);
                eprintln!("[joycon] {} disconnected", pad.gamepad_name);
                unsafe { SDL_CloseGamepad(pad.ptr) };
            }
        }
        t if t == SDL_EVENT_GAMEPAD_SENSOR_UPDATE.0 => {
            handle_sensor(event, pads);
        }
        _ => {}
    }
}

fn handle_sensor(event: &SDL_Event, pads: &mut [Pad]) {
    let ev = unsafe { event.gsensor };
    let Some(pad) = pads.iter_mut().find(|p| p.id == ev.which) else {
        return;
    };
    let ty = SDL_SensorType(ev.sensor);
    let Some(binding) = pad.sensors.iter().find(|s| s.ty == ty).copied() else {
        return;
    };

    // Not guaranteed to be synchronized with the system clock, but it is monotonic per pad and
    // gives us the true 5 ms sample spacing even if reports arrive in batches.
    let ts = if ev.sensor_timestamp != 0 {
        ev.sensor_timestamp
    } else {
        ev.timestamp
    };
    let value = Vec3::new(ev.data[0], ev.data[1], ev.data[2]);

    let calib = pad.calibration[binding.hand.index()];
    let rt = match binding.hand {
        Hand::Left => pad
            .left
            .get_or_insert_with(|| HandRuntime::new(Hand::Left, calib)),
        Hand::Right => pad
            .right
            .get_or_insert_with(|| HandRuntime::new(Hand::Right, calib)),
    };

    if binding.gyro {
        rt.last_gyro = value;
        rt.pending_gyro = Some((ts, value));
        if !rt.has_accel {
            // No accelerometer enabled: integrate gyro only (accel correction is skipped).
            rt.fusion.push(value, Vec3::ZERO, ts);
        }
    } else {
        rt.last_accel = value;
        rt.has_accel = true;
        let gyro = match rt.pending_gyro.take() {
            Some((gyro_ts, g)) if gyro_ts == ts => g,
            // Accel arrived before its gyro twin (or the gyro was dropped): use the latest gyro.
            _ => rt.last_gyro,
        };
        rt.fusion.push(gyro, value, ts);
        if reach_enabled_for(rt.reach.is_calibrated()) {
            // Calibrated dead reckoning runs at IMU rate next to the fusion, using the same
            // recentered orientation the arms consume. The epoch catches the fusion's own
            // auto-recenter (and Minus/Plus), which re-anchors the depth reference.
            rt.reach.push(
                device_to_body(value),
                device_to_body(gyro) - rt.fusion.bias,
                rt.fusion.orientation(),
                ts,
                rt.fusion.recenter_epoch,
            );
        }
        if let Some(sink) = &pad.recorder {
            // Record the exact pair the filters consumed (raw, before calibration/bias).
            let _ = sink.send(ImuSample {
                ts_ns: ts,
                hand: binding.hand,
                gyro,
                accel: value,
            });
        }
    }
}

/// Read buttons/sticks and refresh the published per-hand fields.
/// Returns true on a Minus/Plus press edge (the caller recenters every hand).
unsafe fn poll_pad(pad: &mut Pad) -> bool {
    let gp = pad.ptr;
    let mut recenter = false;
    for hand in [Hand::Left, Hand::Right] {
        let rt = match hand {
            Hand::Left => pad.left.as_mut(),
            Hand::Right => pad.right.as_mut(),
        };
        let Some(rt) = rt else { continue };

        let (axis_x, axis_y, btn_stick, btn_shoulder, btn_menu, btn_trigger) = match hand {
            Hand::Left => (
                SDL_GAMEPAD_AXIS_LEFTX,
                SDL_GAMEPAD_AXIS_LEFTY,
                SDL_GAMEPAD_BUTTON_LEFT_STICK,
                SDL_GAMEPAD_BUTTON_LEFT_SHOULDER,
                SDL_GAMEPAD_BUTTON_BACK,
                SDL_GAMEPAD_AXIS_LEFT_TRIGGER,
            ),
            Hand::Right => (
                SDL_GAMEPAD_AXIS_RIGHTX,
                SDL_GAMEPAD_AXIS_RIGHTY,
                SDL_GAMEPAD_BUTTON_RIGHT_STICK,
                SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER,
                SDL_GAMEPAD_BUTTON_START,
                SDL_GAMEPAD_AXIS_RIGHT_TRIGGER,
            ),
        };

        unsafe {
            rt.input.stick = stick_from_sdl(
                SDL_GetGamepadAxis(gp, axis_x),
                SDL_GetGamepadAxis(gp, axis_y),
            );
            // Triggers are digital on a Joy-Con, but SDL hands them out on the 0..32767 range.
            rt.input.trigger = SDL_GetGamepadAxis(gp, btn_trigger) > (32767 / 2) as i16;
            rt.input.shoulder = SDL_GetGamepadButton(gp, btn_shoulder);
            rt.input.stick_click = SDL_GetGamepadButton(gp, btn_stick);
            // Capture is SDL's MISC1 on the left Joy-Con; Home is GUIDE on the right one.
            rt.input.extra = SDL_GetGamepadButton(
                gp,
                match hand {
                    Hand::Left => SDL_GAMEPAD_BUTTON_MISC1,
                    Hand::Right => SDL_GAMEPAD_BUTTON_GUIDE,
                },
            );

            let menu = SDL_GetGamepadButton(gp, btn_menu);
            // Minus / Plus: yaw-only recenter, pitch/roll stay absolute.
            recenter |= menu && !rt.prev_menu;
            rt.prev_menu = menu;
            rt.input.menu = menu;

            rt.input.face = match hand {
                // D-pad positions come through SDL's hat mapping as DPAD_* buttons.
                Hand::Left => [
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_DPAD_UP),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_DPAD_DOWN),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_DPAD_LEFT),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_DPAD_RIGHT),
                ],
                // [north (X), south (B), west (Y), east (A)] - physical positions.
                Hand::Right => [
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_NORTH),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_SOUTH),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_WEST),
                    SDL_GetGamepadButton(gp, SDL_GAMEPAD_BUTTON_EAST),
                ],
            };
        }

        rt.input.connected = true;
        rt.input.orientation = rt.fusion.orientation();
        rt.input.gyro = device_to_body(rt.last_gyro) - rt.fusion.bias;
        rt.input.accel = device_to_body(rt.last_accel);
        // With reach off for this hand the published field stays exactly zero, so the arm
        // renders the untouched orientation-only pose (solve_arm_with_reach(_, _, ZERO)).
        let reach_on = reach_enabled_for(rt.reach.is_calibrated());
        rt.input.reach = if reach_on {
            rt.reach.offset
        } else {
            Vec3::ZERO
        };
        rt.input.reach_velocity = if reach_on {
            rt.reach.velocity
        } else {
            Vec3::ZERO
        };
        rt.input.imu_hz = rt.fusion.hz;
    }
    recenter
}

fn publish(pads: &[Pad], hands: &Mutex<Hands>, debug: &Mutex<Diagnostics>, with_debug: bool) {
    let mut snapshot = Hands::default();
    let mut dbg = Diagnostics::default();

    // Separate pads first, combined pair last: if SDL briefly exposes both while re-pairing, the
    // combined view (which has per-hand sensors) wins.
    for pad in pads.iter().filter(|p| p.kind != PadKind::Combined) {
        pad.write_hands(&mut snapshot, &mut dbg, with_debug);
    }
    for pad in pads.iter().filter(|p| p.kind == PadKind::Combined) {
        pad.write_hands(&mut snapshot, &mut dbg, with_debug);
    }

    *lock(hands) = snapshot;
    if with_debug {
        *lock(debug) = dbg;
    }
}

impl Pad {
    fn write_hands(&self, hands: &mut Hands, dbg: &mut Diagnostics, with_debug: bool) {
        if let Some(rt) = &self.left {
            hands.left = rt.input.clone();
            if with_debug {
                dbg.left = self.debug_for(Hand::Left, rt);
            }
        }
        if let Some(rt) = &self.right {
            hands.right = rt.input.clone();
            if with_debug {
                dbg.right = self.debug_for(Hand::Right, rt);
            }
        }
    }

    fn debug_for(&self, hand: Hand, rt: &HandRuntime) -> HandDebug {
        let raw_accel = device_to_body(rt.last_accel);
        let calib = rt.fusion.calib;
        let reach_on = reach_enabled_for(rt.reach.is_calibrated());
        let mut d = HandDebug {
            connected: true,
            topology: self.kind.topology(),
            gamepad_name: self.gamepad_name.clone(),
            gamepad_type: self.gamepad_type.clone(),
            raw_gyro: rt.last_gyro,
            raw_accel: rt.last_accel,
            body_gyro: device_to_body(rt.last_gyro) - rt.fusion.bias,
            body_accel: match calib {
                Some(c) => c.apply(raw_accel),
                None => raw_accel,
            },
            gyro_bias: rt.fusion.bias,
            bias_ready: rt.fusion.bias_ready,
            accel_calibrated: calib.is_some(),
            accel_calib_scale: calib.map_or(Vec3::ONE, |c| c.scale),
            accel_calib_offset: calib.map_or(Vec3::ZERO, |c| c.offset),
            reach_gravity: rt.reach.gravity,
            reach: if reach_on {
                rt.reach.offset
            } else {
                Vec3::ZERO
            },
            reach_velocity: if reach_on {
                rt.reach.velocity
            } else {
                Vec3::ZERO
            },
            reach_model_explained: if reach_on {
                rt.reach.model_explained
            } else {
                Vec3::ZERO
            },
            reach_residual: if reach_on {
                rt.reach.p_imu - rt.reach.model_explained
            } else {
                Vec3::ZERO
            },
            reach_enabled: reach_on,
            imu_hz: rt.fusion.hz,
            notes: self.notes.clone(),
            sensors: Vec::new(),
        };
        for sensor in self.sensors.iter().filter(|s| s.hand == hand) {
            let rate = unsafe { SDL_GetGamepadSensorDataRate(self.ptr, sensor.ty) };
            d.sensors.push(SensorStatus {
                name: sensor.label,
                rate_hz: rate,
            });
        }
        d
    }
}

// ---------------------------------------------------------------------------------------------
// Stick
// ---------------------------------------------------------------------------------------------

/// Radial deadzone, rescaled so the edge of the deadzone maps to 0 and the rim to 1.
const STICK_DEADZONE: f32 = 0.12;

fn apply_stick_deadzone(v: Vec2) -> Vec2 {
    let m = v.length();
    if m <= STICK_DEADZONE {
        Vec2::ZERO
    } else {
        let scaled = ((m - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)).min(1.0);
        v * (scaled / m)
    }
}

/// SDL sticks: x right-positive, y down-positive. Contract wants x right / y up.
fn stick_from_sdl(x: i16, y_down: i16) -> Vec2 {
    let x = (x as f32 / 32768.0).clamp(-1.0, 1.0);
    let y = (-(y_down as f32) / 32768.0).clamp(-1.0, 1.0);
    apply_stick_deadzone(Vec2::new(x, y))
}

// ---------------------------------------------------------------------------------------------
// IMU fusion
// ---------------------------------------------------------------------------------------------

const GRAVITY: f32 = 9.80665;
/// Mahony accelerometer gain (1/s): τ ≈ 0.25 s. Tuned on the user's recordings
/// (`real_tilt_error_when_the_hand_stops`): with gain 2 and the M2c gyro gate the tilt was
/// 9-35° off whenever the hand came to rest after moving; with gain 4 and no gyro gate it is
/// 0.1-4°.
const ACCEL_KP: f32 = 4.0;
/// Hard fallback band used when there is no accelerometer calibration: trust the accel for tilt
/// only when |a| is this close to 1 g (a swing would otherwise tilt the estimate).
const ACCEL_TRUST_BAND: f32 = 1.5;
/// With a calibration file the correction is weighted by how close `|a_cal|` is to 1 g: full
/// below 0.3 m/s² away, none beyond 1.0. There is deliberately **no gyro gate**: a hand held in
/// the air turns at 0.3-1 rad/s almost all the time, so the M2c gate (off above 0.6 rad/s)
/// switched the correction off for whole seconds and the tilt free-ran on gyro scale error.
const ACCEL_CAL_BAND_SOFT: f32 = 0.30;
const ACCEL_CAL_BAND_HARD: f32 = 1.0;

/// Hermite smoothstep (C1), used by all the soft gates: 0 below `edge0`, 1 above `edge1`.
fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
/// Gyro bias = mean of the raw gyro over "still" windows (GamepadMotionHelpers-style). Stillness
/// is judged by how little the raw gyro/accel VARY inside a window, not by |gyro - bias|: the old
/// magnitude gate deadlocked whenever the first estimate was off by more than its threshold (e.g.
/// the hand moved during the first second), and the hand then drifted in yaw forever.
const STILL_WINDOW_NS: u64 = 400_000_000;
/// Max std dev inside a still window. Measured scale: on a table ~0.002 rad/s / ~0.02 m/s²;
/// a hand holding still ~0.005-0.02 rad/s / ~0.05-0.15 m/s².
const STILL_GYRO_STD: f32 = 0.02;
const STILL_ACCEL_STD: f32 = 0.25;
/// A steady window whose mean is this far from the bias is a slow real turn, not bias
/// (Joy-Con residual bias after factory calibration is ~0.01-0.05 rad/s).
const BIAS_MAX_CORRECTION: f32 = 0.15;
/// Blend per still window after the first one (0.4 s windows => ~1.2 s time constant).
const BIAS_BLEND: f32 = 0.3;
/// Start integrating this long after connect even if no still window was seen yet.
const BIAS_INIT_NS: u64 = 1_000_000_000;
/// Ignore gaps longer than this (BT stall / reconnect): no integration for that step.
const DT_MAX_NS: u64 = 50_000_000;
/// Auto-recenter after this many samples (~0.2 s at 200 Hz) = "first stable estimate".
const AUTO_RECENTER_SAMPLES: u32 = 40;

/// Mahony-style complementary filter, gravity-aligned world frame (+Y up), body-to-world
/// quaternion (Bevy `Transform::rotation` convention: it maps body/local vectors to world).
///
/// The accelerometer only corrects tilt (it cannot observe yaw); yaw comes from the gyro and is
/// re-zeroed by [`ImuFusion::recenter`]. When an [`AccelCalib`] is loaded, the accel is corrected
/// per axis before both the tilt correction and the bias stillness windows.
#[derive(Debug)]
struct ImuFusion {
    /// body -> world.
    q: Quat,
    /// World-space yaw applied on the left of `q` so the current heading maps to body -Z.
    yaw_offset: Quat,
    /// Gyro bias in body space (rad/s).
    bias: Vec3,
    bias_ready: bool,
    /// At least one still window has set `bias`.
    bias_calibrated: bool,
    bias_start_ts: Option<u64>,
    /// Running sums of the current stillness window (body-space gyro/accel, accel corrected).
    win_start: Option<u64>,
    win_n: u32,
    win_g: Vec3,
    win_g2: Vec3,
    win_a: Vec3,
    win_a2: Vec3,
    last_ts: Option<u64>,
    samples: u32,
    auto_recentered: bool,
    hz_start_ts: Option<u64>,
    hz_count: u32,
    hz: f32,
    /// Per-axis accelerometer correction, when the file was found.
    calib: Option<AccelCalib>,
    /// `true` (production): calibrated, magnitude-weighted tilt correction. `false`: the pre-M2c
    /// hard-band filter, kept as the A/B baseline in the tests.
    motion_gate: bool,
    /// Bumped every time the yaw recenter actually changes [`Self::yaw_offset`]. The reach
    /// tracker uses it as the signal that its body-frame anchor is stale.
    recenter_epoch: u64,
}

impl ImuFusion {
    #[cfg(test)]
    fn new() -> Self {
        Self::with_calibration(None)
    }

    fn with_calibration(calib: Option<AccelCalib>) -> Self {
        Self {
            q: Quat::IDENTITY,
            yaw_offset: Quat::IDENTITY,
            bias: Vec3::ZERO,
            bias_ready: false,
            bias_calibrated: false,
            bias_start_ts: None,
            win_start: None,
            win_n: 0,
            win_g: Vec3::ZERO,
            win_g2: Vec3::ZERO,
            win_a: Vec3::ZERO,
            win_a2: Vec3::ZERO,
            last_ts: None,
            samples: 0,
            auto_recentered: false,
            hz_start_ts: None,
            hz_count: 0,
            hz: 0.0,
            calib,
            motion_gate: true,
            recenter_epoch: 0,
        }
    }

    /// One IMU sample. `gyro_dev`/`accel_dev` are raw SDL device-frame values; `ts_ns` is the
    /// event's `sensor_timestamp`.
    fn push(&mut self, gyro_dev: Vec3, accel_dev: Vec3, ts_ns: u64) {
        let gyro = device_to_body(gyro_dev);
        let accel_raw = device_to_body(accel_dev);
        let accel = match self.calib {
            Some(calib) => calib.apply(accel_raw),
            None => accel_raw,
        };

        self.samples = self.samples.saturating_add(1);
        self.update_rate(ts_ns);

        // dt from sensor timestamps; the first sample and long gaps don't integrate.
        let dt = match self.last_ts {
            None => None,
            Some(last) => {
                let delta = ts_ns.saturating_sub(last);
                if delta == 0 || delta > DT_MAX_NS {
                    None
                } else {
                    Some(delta as f32 * 1e-9)
                }
            }
        };
        self.last_ts = Some(ts_ns);

        self.update_bias(gyro, accel, ts_ns);

        let Some(dt) = dt else {
            // First sample, duplicate timestamp, or a >50 ms gap: resync and integrate nothing.
            self.maybe_auto_recenter(accel);
            return;
        };

        // During the initial bias window we deliberately don't integrate gyro.
        let mut omega = if self.bias_ready {
            gyro - self.bias
        } else {
            Vec3::ZERO
        };

        let a_norm = accel.length();
        if self.motion_gate {
            // Trust the accel for tilt in proportion to how close |a| is to 1 g.
            let magnitude = if self.calib.is_some() {
                1.0 - smoothstep(
                    ACCEL_CAL_BAND_SOFT,
                    ACCEL_CAL_BAND_HARD,
                    (a_norm - GRAVITY).abs(),
                )
            } else if (a_norm - GRAVITY).abs() < ACCEL_TRUST_BAND {
                1.0
            } else {
                0.0
            };
            let gain = ACCEL_KP * magnitude;
            if gain > 1e-4 && a_norm > 0.1 {
                // World up expressed in body space, per the current estimate.
                let up_est = self.q.conjugate() * Vec3::Y;
                let a_hat = accel / a_norm;
                // Mahony tilt error; positive gain drives `up_est` toward the measured up.
                omega += a_hat.cross(up_est) * gain;
            }
        } else if (a_norm - GRAVITY).abs() < ACCEL_TRUST_BAND && a_norm > 0.1 {
            // Pre-M2c behaviour, kept only as the A/B baseline in the tests.
            let up_est = self.q.conjugate() * Vec3::Y;
            let a_hat = accel / a_norm;
            omega += a_hat.cross(up_est) * ACCEL_KP;
        }

        if omega.length_squared() > 0.0 {
            self.q = (self.q * Quat::from_scaled_axis(omega * dt)).normalize();
        }

        self.maybe_auto_recenter(accel);
    }

    fn update_rate(&mut self, ts_ns: u64) {
        let start = *self.hz_start_ts.get_or_insert(ts_ns);
        self.hz_count = self.hz_count.saturating_add(1);
        let elapsed = ts_ns.saturating_sub(start);
        if elapsed >= 500_000_000 {
            self.hz = self.hz_count as f32 * 1e9 / elapsed as f32;
            self.hz_start_ts = Some(ts_ns);
            self.hz_count = 0;
        }
    }

    fn update_bias(&mut self, gyro: Vec3, accel: Vec3, ts_ns: u64) {
        let connect = *self.bias_start_ts.get_or_insert(ts_ns);
        if !self.bias_ready && ts_ns.saturating_sub(connect) >= BIAS_INIT_NS {
            // Held in hand at connect and never still yet: integrate with bias 0; the first
            // still window below fixes it as soon as the Joy-Con settles.
            self.bias_ready = true;
        }

        let start = *self.win_start.get_or_insert(ts_ns);
        self.win_n += 1;
        self.win_g += gyro;
        self.win_g2 += gyro * gyro;
        self.win_a += accel;
        self.win_a2 += accel * accel;
        if ts_ns.saturating_sub(start) < STILL_WINDOW_NS {
            return;
        }

        let n = self.win_n as f32;
        // Std dev of a 3-vector signal: sqrt of the summed per-axis variances.
        let std = |sum: Vec3, sum2: Vec3| {
            let mean = sum / n;
            (sum2 / n - mean * mean)
                .max(Vec3::ZERO)
                .element_sum()
                .sqrt()
        };
        let mean_g = self.win_g / n;
        let still = self.win_n >= 20
            && std(self.win_g, self.win_g2) < STILL_GYRO_STD
            && std(self.win_a, self.win_a2) < STILL_ACCEL_STD
            && (mean_g - self.bias).length() < BIAS_MAX_CORRECTION;
        if still {
            let blend = if self.bias_calibrated {
                BIAS_BLEND
            } else {
                1.0
            };
            self.bias = self.bias.lerp(mean_g, blend);
            if !self.bias_calibrated {
                eprintln!(
                    "[joycon] gyro bias calibrated: |{:.4}| rad/s over {} samples",
                    self.bias.length(),
                    self.win_n
                );
            }
            self.bias_calibrated = true;
            self.bias_ready = true;
        }

        self.win_start = None;
        self.win_n = 0;
        self.win_g = Vec3::ZERO;
        self.win_g2 = Vec3::ZERO;
        self.win_a = Vec3::ZERO;
        self.win_a2 = Vec3::ZERO;
    }

    fn maybe_auto_recenter(&mut self, accel: Vec3) {
        if self.auto_recentered || self.samples < AUTO_RECENTER_SAMPLES {
            return;
        }
        if (accel.length() - GRAVITY).abs() > ACCEL_TRUST_BAND {
            return;
        }
        self.recenter();
        self.auto_recentered = true;
    }

    /// Yaw-only recenter: store a world-Y offset so the Joy-Con's forward axis (projected onto
    /// the horizontal plane) maps to body -Z. Pitch and roll are untouched.
    fn recenter(&mut self) {
        let forward = self.q * Vec3::NEG_Z;
        let horizontal = Vec2::new(forward.x, forward.z);
        if horizontal.length_squared() < 1e-6 {
            // Pointing straight up/down: heading is undefined, keep the old offset.
            return;
        }
        let heading = horizontal.x.atan2(-horizontal.y);
        self.yaw_offset = Quat::from_rotation_y(heading);
        self.recenter_epoch = self.recenter_epoch.wrapping_add(1);
    }

    /// Final orientation for `HandInput`: body space with a recentered heading.
    fn orientation(&self) -> Quat {
        (self.yaw_offset * self.q).normalize()
    }
}

// ---------------------------------------------------------------------------------------------
// Reach (IMU dead reckoning of hand translation)
// ---------------------------------------------------------------------------------------------

/// Depth limits (m) along the pointing direction: how far the hand can be pushed out in front
/// of the model pose, and pulled back toward the body. The arm IK clamps again to the real arm.
const REACH_FORWARD_MAX: f32 = 0.35;
const REACH_BACK_MAX: f32 = 0.30;
/// Overall bound on the published offset (also used by the IK as a defensive clamp).
const REACH_MAX: f32 = 0.35;
/// Calm detector: a sliding 40 ms window whose mean linear acceleration (gravity removed) and
/// mean |gyro| are below these. Measured on the user's recordings: a hand held in the air is
/// almost never gyro-still (0.3-1 rad/s of tremor and slow drift), but its linear acceleration
/// stays < 0.5 m/s². A push or pull is 2-15 m/s². So "calm" is judged on translation, and the
/// gyro gate only rejects clear rotation.
const REACH_CALM_WINDOW_NS: u64 = 40_000_000;
const REACH_CALM_MIN_SAMPLES: usize = 5;
const REACH_CALM_ACCEL: f32 = 0.8;
const REACH_CALM_GYRO: f32 = 1.5;
/// A movement ends after this much continuous calm.
const REACH_CALM_CONFIRM_NS: u64 = 40_000_000;
/// A movement longer than this is not a push: integration error has grown too large to trust,
/// so it is discarded (the depth keeps its previous value).
const REACH_SEGMENT_MAX_S: f32 = 1.2;
/// A movement whose peak |gyro| exceeds this is a wrist flick/rotation, not a push: the
/// accelerometer then mostly feels the lever arm and orientation error. Discarded.
const REACH_SEGMENT_MAX_GYRO: f32 = 6.0;
/// The velocity left at the end of a movement must be zero for a real stop. A larger residual
/// means the integration went wrong; the movement is discarded.
const REACH_SEGMENT_MAX_END_SPEED: f32 = 0.8;
/// The live preview of a movement fades out between these durations (s)...
const REACH_PREVIEW_FADE_S: f32 = 0.6;
/// ...and is hidden while the integrated speed is implausible for a hand (m/s).
const REACH_PREVIEW_MAX_SPEED: f32 = 2.5;
/// Ignore depth changes smaller than this (wobble).
const REACH_SEGMENT_MIN_DEPTH: f32 = 0.015;
/// The published depth eases toward its target with this time constant (s), so the end-of-move
/// correction never snaps.
const REACH_EASE_TAU: f32 = 0.04;
/// After this much calm (s) the depth starts returning to the model pose...
const REACH_HOLD_S: f32 = 3.0;
/// ...with this time constant (s). Bounds the random walk of many small per-move errors.
const REACH_RETURN_TAU: f32 = 10.0;
/// Uncalibrated fallback: this hand's gravity magnitude is learned from calm, gyro-quiet windows.
const REACH_GRAVITY_LEARN_GYRO: f32 = 0.2;
/// Depth stays zero for this long after connect, while the fused tilt converges.
const REACH_INIT_NS: u64 = 1_000_000_000;

// --- The arm model's wrist position (a duplicate of arms.rs, kept pure) ----------------------
//
// `reach` must only carry translation the Daydream arm model cannot already explain from the
// orientation change, so the tracker needs `wrist_model(q)`. `arms::solve_arm` is the source of
// truth, but this file is compiled on its own by `examples/joycon_probe.rs` (which has no
// Bevy/avian app modules), so these constants and the function are duplicated verbatim.
// `model_wrist_matches_arms_solve_arm` pins them to golden values produced by the real
// `arms::solve_arm`; **if arms.rs changes, that test must be regenerated.**

const MODEL_ELBOW_BEND_RATIO: f32 = 0.6;
const MODEL_EXTENSION_WEIGHT: f32 = 0.4;
const MODEL_MIN_EXTENSION_ANGLE: f32 = 7.0;
const MODEL_MAX_EXTENSION_ANGLE: f32 = 60.0;
/// Elbow rest relative to the head (Bevy space, right hand).
const MODEL_ELBOW_REST: Vec3 = Vec3::new(0.195, -0.5, -0.005);
/// Wrist relative to the elbow.
const MODEL_WRIST_REST: Vec3 = Vec3::new(0.0, 0.0, -0.25);
/// Elbow offset at full extension.
const MODEL_ARM_EXTENSION_OFFSET: Vec3 = Vec3::new(-0.13, 0.14, -0.08);

/// Wrist position of the Daydream arm model for `orientation`, in body space relative to the
/// eye. Mirrors `arms::solve_arm(hand, orientation).wrist` (GvrArmModel `ApplyRotationToJoints`).
fn model_wrist(hand: Hand, orientation: Quat) -> Vec3 {
    let handed = Vec3::new(
        match hand {
            Hand::Left => -1.0,
            Hand::Right => 1.0,
        },
        1.0,
        1.0,
    );

    // GetControllerRotation: remove roll (the shortest arc onto the forward direction).
    let controller_forward = orientation * Vec3::NEG_Z;
    let x_angle = 90.0 - controller_forward.angle_between(Vec3::Y).to_degrees();
    let xy_rotation = Quat::from_rotation_arc(Vec3::NEG_Z, controller_forward);

    // SetUntransformedJointPositions + ApplyExtensionOffset.
    let extension_ratio = ((x_angle - MODEL_MIN_EXTENSION_ANGLE)
        / (MODEL_MAX_EXTENSION_ANGLE - MODEL_MIN_EXTENSION_ANGLE))
        .clamp(0.0, 1.0);
    let elbow = MODEL_ELBOW_REST * handed + MODEL_ARM_EXTENSION_OFFSET * handed * extension_ratio;
    let wrist_local = MODEL_WRIST_REST * handed;

    // CalculateLerpRotation: bend the elbow less than the controller turns.
    let total_angle = xy_rotation.angle_between(Quat::IDENTITY).to_degrees();
    let suppression = 1.0 - (total_angle / 180.0).powi(6);
    let lerp_value = ((1.0 - MODEL_ELBOW_BEND_RATIO)
        + MODEL_ELBOW_BEND_RATIO * extension_ratio * MODEL_EXTENSION_WEIGHT)
        * suppression;
    let lerp_rotation = Quat::IDENTITY.slerp(xy_rotation, lerp_value.clamp(0.0, 1.0));
    let elbow_rotation = lerp_rotation.inverse() * xy_rotation;

    // ApplyRotationToJoints.
    elbow + elbow_rotation * wrist_local
}

/// One sample of the sliding calm window.
#[derive(Clone, Copy, Debug)]
struct CalmSample {
    ts_ns: u64,
    linear: f32,
    gyro: f32,
    accel_mag: f32,
}

/// One movement (a push, pull or punch) being integrated.
#[derive(Clone, Copy, Debug)]
struct ReachSegment {
    duration: f32,
    velocity: Vec3,
    displacement: Vec3,
    max_gyro: f32,
    /// Model wrist at the start of the movement.
    start_wrist: Vec3,
    /// Continuous calm time at the tail of the movement (s).
    calm: f32,
}

/// How a finished movement was judged (diagnostics and tests).
#[derive(Clone, Copy, Debug, PartialEq)]
enum ReachVerdict {
    Accepted,
    TooLong,
    Rotation,
    NoStop,
    Tiny,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(not(test), allow(dead_code))] // read by the replay tests
struct ReachEvent {
    end_ts: u64,
    duration: f32,
    max_gyro: f32,
    end_speed: f32,
    /// Depth change this movement measured (m, + = forward), before acceptance.
    delta: f32,
    verdict: ReachVerdict,
}

/// Per-hand hand depth from the accelerometer, integrated **one movement at a time**.
///
/// Why not continuous dead reckoning: on real recordings a hand held in the air is almost never
/// still (the gyro rarely drops below 0.2 rad/s), so a zero-velocity update never fires, and fast
/// wrist rotations (8-15 rad/s) leak metres per second² of orientation/lever-arm error. A
/// continuous integrator ran into the 35 cm clamp within a second.
///
/// What works instead (the foot-mounted INS trick, applied to the arm):
/// * a **movement** starts when the hand stops being calm (calm = mean linear acceleration
///   < 0.8 m/s² and mean |gyro| < 1.5 rad/s over 40 ms) and ends after 40 ms of calm;
/// * inside it, `a = R·a_cal − g·Ŷ` is integrated from zero velocity. At the end the true
///   velocity is zero again, so the residual is removed linearly (`p −= v_end·T/2`);
/// * the movement is **discarded** if it lasted > 1.2 s, peaked above 6 rad/s (a wrist flick,
///   not a push), or still had > 0.8 m/s at the stop (the integration went wrong);
/// * an accepted movement changes the **depth**: its displacement minus the arm model's own
///   wrist motion (so a swing the model already shows is not counted twice), projected on the
///   pointing direction. Depth is a scalar (+ = forward), clamped to [−0.30, +0.35] m;
/// * between movements nothing is integrated, so nothing drifts. After 3 s of calm the depth
///   returns to the model pose with τ = 10 s;
/// * while a movement is in progress its running estimate is shown live, and the published
///   value eases toward the target with τ = 40 ms.
///
/// The published offset is `forward · depth` in the recentered body frame (−Z forward).
#[derive(Debug)]
struct ReachTracker {
    hand: Hand,
    calib: Option<AccelCalib>,
    /// This hand's gravity magnitude (uncalibrated fallback only).
    gravity: f32,
    gravity_ready: bool,
    /// Committed depth (m, + = forward).
    depth: f32,
    /// Published, eased depth (m).
    shown: f32,
    /// Published offset (`forward · shown`).
    offset: Vec3,
    /// Published offset velocity (m/s).
    velocity: Vec3,
    /// Debug: the current movement's displacement and the model motion subtracted from it.
    p_imu: Vec3,
    model_explained: Vec3,
    /// Calm right now (the window test).
    still: bool,
    calm_time: f32,
    segment: Option<ReachSegment>,
    /// After a discarded movement, wait for calm before starting a new one.
    wait_calm: bool,
    window: VecDeque<CalmSample>,
    initialized: bool,
    start_ts: Option<u64>,
    last_ts: Option<u64>,
    epoch: u64,
    last_event: Option<ReachEvent>,
}

impl ReachTracker {
    #[cfg(test)]
    fn new() -> Self {
        Self::with_calibration(None, Hand::Right)
    }

    fn with_calibration(calib: Option<AccelCalib>, hand: Hand) -> Self {
        Self {
            hand,
            calib,
            gravity: GRAVITY,
            gravity_ready: calib.is_some(),
            depth: 0.0,
            shown: 0.0,
            offset: Vec3::ZERO,
            velocity: Vec3::ZERO,
            p_imu: Vec3::ZERO,
            model_explained: Vec3::ZERO,
            still: false,
            calm_time: 0.0,
            segment: None,
            wait_calm: false,
            window: VecDeque::new(),
            initialized: false,
            start_ts: None,
            last_ts: None,
            epoch: 0,
            last_event: None,
        }
    }

    fn is_calibrated(&self) -> bool {
        self.calib.is_some()
    }

    /// Back to the model pose (recenter / reconnect).
    fn reset(&mut self) {
        self.depth = 0.0;
        self.shown = 0.0;
        self.offset = Vec3::ZERO;
        self.velocity = Vec3::ZERO;
        self.p_imu = Vec3::ZERO;
        self.model_explained = Vec3::ZERO;
        self.segment = None;
        self.wait_calm = true;
        self.calm_time = 0.0;
    }

    /// One IMU sample. `accel` is raw SDL device space (the tracker applies its own copy of the
    /// accelerometer calibration); `gyro` is body space and bias-corrected; `orientation` is the
    /// yaw-recentered body→world rotation the arms render; `epoch` is
    /// [`ImuFusion::recenter_epoch`] (a change resets the depth).
    fn push(&mut self, accel: Vec3, gyro: Vec3, orientation: Quat, ts_ns: u64, epoch: u64) {
        let accel = self.calib.map_or(accel, |calib| calib.apply(accel));
        let dt = match self.last_ts {
            Some(last) => {
                let delta = ts_ns.saturating_sub(last);
                (delta > 0 && delta <= DT_MAX_NS).then(|| delta as f32 * 1e-9)
            }
            None => None,
        };
        self.last_ts = Some(ts_ns);
        let start = *self.start_ts.get_or_insert(ts_ns);
        if epoch != self.epoch {
            self.epoch = epoch;
            self.reset();
        }

        let gyro_mag = gyro.length();
        let linear = orientation * accel - Vec3::Y * self.gravity;
        self.update_window(ts_ns, linear.length(), gyro_mag, accel.length());

        if !self.initialized {
            if self.still && ts_ns.saturating_sub(start) >= REACH_INIT_NS {
                self.initialized = true;
            }
            return;
        }
        let Some(dt) = dt else {
            return;
        };

        let forward = (orientation * Vec3::NEG_Z).normalize_or_zero();
        let wrist = model_wrist(self.hand, orientation);
        if self.wait_calm && self.still {
            self.wait_calm = false;
        }

        let mut preview = 0.0;
        match self.segment.as_mut() {
            None if !self.still && !self.wait_calm => {
                // A movement starts: integrate from zero velocity, this sample included.
                let mut segment = ReachSegment {
                    duration: dt,
                    velocity: linear * dt,
                    displacement: Vec3::ZERO,
                    max_gyro: gyro_mag,
                    start_wrist: wrist,
                    calm: 0.0,
                };
                segment.displacement = segment.velocity * dt;
                self.segment = Some(segment);
                self.calm_time = 0.0;
            }
            None => {
                self.calm_time += dt;
                if self.calm_time > REACH_HOLD_S {
                    self.depth *= (-dt / REACH_RETURN_TAU).exp();
                }
            }
            Some(segment) => {
                segment.duration += dt;
                segment.velocity += linear * dt;
                segment.displacement += segment.velocity * dt;
                segment.max_gyro = segment.max_gyro.max(gyro_mag);
                segment.calm = if self.still { segment.calm + dt } else { 0.0 };
            }
        }

        if let Some(segment) = self.segment {
            let model = wrist - segment.start_wrist;
            self.p_imu = segment.displacement;
            self.model_explained = model;
            let stopped = segment.calm >= REACH_CALM_CONFIRM_NS as f32 * 1e-9;
            if stopped || segment.duration > REACH_SEGMENT_MAX_S {
                self.finish_segment(segment, model, forward, ts_ns, stopped);
            } else if segment.max_gyro <= REACH_SEGMENT_MAX_GYRO
                && segment.velocity.length() < REACH_PREVIEW_MAX_SPEED
            {
                // Live feedback while the push is happening, faded out as the movement gets
                // long enough to be doubtful (it may still be discarded).
                let trust = 1.0
                    - smoothstep(
                        REACH_PREVIEW_FADE_S,
                        REACH_SEGMENT_MAX_S,
                        segment.duration,
                    );
                preview = (segment.displacement - model).dot(forward) * trust;
            }
        }

        let target = (self.depth + preview).clamp(-REACH_BACK_MAX, REACH_FORWARD_MAX);
        self.shown += (target - self.shown) * (1.0 - (-dt / REACH_EASE_TAU).exp());
        let offset = (forward * self.shown).clamp_length_max(REACH_MAX);
        self.velocity = (offset - self.offset) / dt;
        self.offset = offset;
    }

    /// Judge a finished movement and commit its depth change if it is trustworthy.
    fn finish_segment(
        &mut self,
        segment: ReachSegment,
        model: Vec3,
        forward: Vec3,
        ts_ns: u64,
        stopped: bool,
    ) {
        self.segment = None;
        // The hand is at rest again, so its true velocity is zero: remove the residual velocity
        // as a linear drift over the movement.
        let corrected = segment.displacement - segment.velocity * (segment.duration * 0.5);
        let delta = (corrected - model).dot(forward);
        let end_speed = segment.velocity.length();
        let verdict = if !stopped {
            ReachVerdict::TooLong
        } else if segment.max_gyro > REACH_SEGMENT_MAX_GYRO {
            ReachVerdict::Rotation
        } else if end_speed > REACH_SEGMENT_MAX_END_SPEED {
            ReachVerdict::NoStop
        } else if delta.abs() < REACH_SEGMENT_MIN_DEPTH {
            ReachVerdict::Tiny
        } else {
            ReachVerdict::Accepted
        };
        if verdict == ReachVerdict::Accepted {
            self.depth = (self.depth + delta).clamp(-REACH_BACK_MAX, REACH_FORWARD_MAX);
        }
        if !stopped {
            // Still moving: don't start a new movement mid-motion.
            self.wait_calm = true;
        }
        self.calm_time = 0.0;
        self.last_event = Some(ReachEvent {
            end_ts: ts_ns,
            duration: segment.duration,
            max_gyro: segment.max_gyro,
            end_speed,
            delta,
            verdict,
        });
    }

    /// Slide the 40 ms calm window and refresh `still` (and the uncalibrated gravity).
    fn update_window(&mut self, ts_ns: u64, linear: f32, gyro: f32, accel_mag: f32) {
        self.window.push_back(CalmSample {
            ts_ns,
            linear,
            gyro,
            accel_mag,
        });
        while self
            .window
            .front()
            .is_some_and(|front| ts_ns.saturating_sub(front.ts_ns) > REACH_CALM_WINDOW_NS)
        {
            self.window.pop_front();
        }
        let span = self
            .window
            .front()
            .map_or(0, |front| ts_ns.saturating_sub(front.ts_ns));
        if self.window.len() < REACH_CALM_MIN_SAMPLES || span < REACH_CALM_WINDOW_NS * 3 / 4 {
            self.still = false;
            return;
        }
        let n = self.window.len() as f32;
        let mean_gyro = self.window.iter().map(|s| s.gyro).sum::<f32>() / n;
        let mean_accel = self.window.iter().map(|s| s.accel_mag).sum::<f32>() / n;

        if self.calib.is_none() && mean_gyro < REACH_GRAVITY_LEARN_GYRO {
            let spread = self
                .window
                .iter()
                .map(|s| (s.accel_mag - mean_accel).abs())
                .fold(0.0, f32::max);
            if spread < 0.2 {
                // Uncalibrated fallback: this Joy-Con's "1 g" (L ≈ 10.2, R ≈ 9.0 m/s²).
                if self.gravity_ready {
                    self.gravity += (mean_accel - self.gravity) * 0.05;
                } else {
                    self.gravity = mean_accel;
                    self.gravity_ready = true;
                }
            }
        }

        let mean_linear = self.window.iter().map(|s| s.linear).sum::<f32>() / n;
        self.still = self.gravity_ready
            && mean_linear < REACH_CALM_ACCEL
            && mean_gyro < REACH_CALM_GYRO;
    }
}

// ---------------------------------------------------------------------------------------------
// Tests (pure math only; no SDL)
// ---------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vec3(actual: Vec3, expected: Vec3, eps: f32) {
        assert!(
            (actual - expected).length() < eps,
            "expected {expected}, got {actual}"
        );
    }

    /// Feed `n` samples at 200 Hz with a fixed reading (body space; converted to SDL device space).
    fn feed(fusion: &mut ImuFusion, gyro_body: Vec3, accel_body: Vec3, ts: &mut u64, n: u32) {
        for _ in 0..n {
            fusion.push(body_to_device(gyro_body), body_to_device(accel_body), *ts);
            *ts += 5_000_000;
        }
    }

    /// Inverse of [`device_to_body`], used only by the tests to express inputs in body space.
    fn body_to_device(v: Vec3) -> Vec3 {
        v
    }

    #[test]
    fn device_mapping_round_trips() {
        for v in [
            Vec3::X,
            Vec3::Y,
            Vec3::Z,
            Vec3::new(1.0, -2.0, 3.0),
            Vec3::new(-0.3, 0.7, 0.11),
        ] {
            assert_vec3(body_to_device(device_to_body(v)), v, 1e-6);
            assert_vec3(device_to_body(body_to_device(v)), v, 1e-6);
        }
    }

    #[test]
    fn axis_mapping_matches_remote_grip() {
        // Face up, top at the screen.
        assert_vec3(device_to_body(Vec3::X), Vec3::X, 1e-6);
        assert_vec3(device_to_body(Vec3::Y), Vec3::Y, 1e-6);
        assert_vec3(device_to_body(Vec3::NEG_Z), Vec3::NEG_Z, 1e-6);
        // Gravity at rest in that pose is dev +Y (SDL docs + hardware), which must stay body +Y.
        assert_vec3(
            device_to_body(Vec3::new(0.0, GRAVITY, 0.0)),
            Vec3::new(0.0, GRAVITY, 0.0),
            1e-6,
        );
        // Proper rotation (det = +1): X x Y = Z.
        let bx = device_to_body(Vec3::X);
        let by = device_to_body(Vec3::Y);
        assert_vec3(bx.cross(by), device_to_body(Vec3::Z), 1e-6);
    }

    #[test]
    fn fusion_levels_pitch_from_gravity_only() {
        let pitch = 0.5_f32;
        // body->world = RotX(pitch) => world up in body = (0, cos p, -sin p)
        let accel = Vec3::new(0.0, GRAVITY * pitch.cos(), -GRAVITY * pitch.sin());

        let mut fusion = ImuFusion::new();
        let mut ts = 1_000_000_000u64;
        feed(&mut fusion, Vec3::ZERO, accel, &mut ts, 600); // 3 s

        let up_est = fusion.q.conjugate() * Vec3::Y;
        let up_meas = accel.normalize();
        let err_deg = up_est.angle_between(up_meas).to_degrees();
        assert!(
            err_deg < 1.0,
            "filter should converge to the measured tilt, off by {err_deg} deg (up_est={up_est})"
        );

        // And the recovered orientation is a pure pitch about X (no yaw/roll introduced).
        let (yaw, pitch_est, roll) = fusion.q.to_euler(EulerRot::YXZ);
        assert!(yaw.to_degrees().abs() < 1.0, "yaw={}", yaw.to_degrees());
        assert!(
            (pitch_est - pitch).to_degrees().abs() < 1.0,
            "pitch={} deg, expected {}",
            pitch_est.to_degrees(),
            pitch.to_degrees()
        );
        assert!(roll.to_degrees().abs() < 1.0, "roll={}", roll.to_degrees());
    }

    #[test]
    fn fusion_integrates_one_radian_of_yaw() {
        let mut fusion = ImuFusion::new();
        let mut ts = 0u64;
        // 1.1 s perfectly still with gravity up: completes the initial bias calibration.
        feed(&mut fusion, Vec3::ZERO, Vec3::Y * GRAVITY, &mut ts, 220);
        assert!(fusion.bias_ready, "bias calibration should be done");
        assert_vec3(fusion.bias, Vec3::ZERO, 1e-6);

        // 1.0 s at exactly 1 rad/s about body +Y.
        feed(&mut fusion, Vec3::Y, Vec3::Y * GRAVITY, &mut ts, 200);

        let expected = Quat::from_rotation_y(1.0);
        let err = fusion.q.angle_between(expected).to_degrees();
        assert!(
            err < 2.0,
            "expected ~1 rad of yaw, got {:.3} rad ({} deg off)",
            fusion.q.to_axis_angle().1,
            err
        );
    }

    #[test]
    fn recenter_zeroes_heading_and_keeps_tilt() {
        let mut fusion = ImuFusion::new();
        let mut ts = 0u64;
        feed(&mut fusion, Vec3::ZERO, Vec3::Y * GRAVITY, &mut ts, 220);
        feed(&mut fusion, Vec3::Y, Vec3::Y * GRAVITY, &mut ts, 100); // ~0.5 rad yaw

        let tilted_up_before = fusion.q * Vec3::Y;
        fusion.recenter();
        let out = fusion.orientation();

        let forward = out * Vec3::NEG_Z;
        assert!(
            forward.x.abs() < 1e-3,
            "heading should be body -Z after recenter, got {forward}"
        );
        assert!(
            forward.z < -0.9,
            "forward should point at the screen: {forward}"
        );

        // World yaw rotation preserves the Y component (pitch/roll stay absolute).
        let tilted_up_after = out * Vec3::Y;
        assert!(
            (tilted_up_after.y - tilted_up_before.y).abs() < 1e-4,
            "recenter must not change tilt: before={tilted_up_before} after={tilted_up_after}"
        );
    }

    #[test]
    fn recenter_is_stable_when_pointing_straight_up() {
        let mut fusion = ImuFusion::new();
        // q = RotX(90 deg): body forward points straight down/up, horizontal heading undefined.
        fusion.q = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        let before = fusion.orientation();
        fusion.recenter();
        let after = fusion.orientation();
        assert!(before.angle_between(after) < 1e-6);
    }

    #[test]
    fn stick_deadzone_and_y_flip() {
        assert_eq!(apply_stick_deadzone(Vec2::ZERO), Vec2::ZERO);
        assert_eq!(apply_stick_deadzone(Vec2::new(0.08, -0.08)), Vec2::ZERO);
        // Edge of the deadzone -> 0, rim -> 1, halfway rescaled.
        assert!(apply_stick_deadzone(Vec2::new(0.12, 0.0)).length() < 1e-6);
        assert_vec3(
            Vec3::new(apply_stick_deadzone(Vec2::new(0.56, 0.0)).x, 0.0, 0.0),
            Vec3::new(0.5, 0.0, 0.0),
            1e-5,
        );
        assert_vec3(
            Vec3::new(apply_stick_deadzone(Vec2::new(1.0, 0.0)).x, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1e-5,
        );

        // SDL y is down-positive; contract y is up-positive.
        let stick = stick_from_sdl(0, 32767);
        assert!(stick.y < -0.9, "stick y should flip: {stick}");
        let stick = stick_from_sdl(0, -32767);
        assert!(stick.y > 0.9, "stick y should flip: {stick}");
        let stick = stick_from_sdl(32767, 0);
        assert!(stick.x > 0.9 && stick.y.abs() < 1e-5, "stick={stick}");
    }

    #[test]
    fn topology_identification_matches_sdl_types() {
        assert_eq!(
            pad_kind_from_type(SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_PAIR),
            Some(PadKind::Combined)
        );
        assert_eq!(
            pad_kind_from_type(SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_LEFT),
            Some(PadKind::Left)
        );
        assert_eq!(
            pad_kind_from_type(SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_JOYCON_RIGHT),
            Some(PadKind::Right)
        );
        // A Pro Controller is a Switch gamepad but not a Joy-Con: ignored.
        assert_eq!(
            pad_kind_from_type(SDL_GAMEPAD_TYPE_NINTENDO_SWITCH_PRO),
            None
        );
        assert_eq!(pad_kind_from_type(SDL_GAMEPAD_TYPE_STANDARD), None);
    }

    #[test]
    fn sensor_routing_matches_the_sdl_switch_driver() {
        // Combined pair: SDL adds *_L for the left child and *_R for the right child
        // (SDL_hidapi_switch.c:894-910); each sensor must feed its own hand.
        let combined = sensors_for(PadKind::Combined);
        assert!(combined.contains(&(SDL_SENSOR_GYRO_L, Hand::Left)));
        assert!(combined.contains(&(SDL_SENSOR_ACCEL_L, Hand::Left)));
        assert!(combined.contains(&(SDL_SENSOR_GYRO_R, Hand::Right)));
        assert!(combined.contains(&(SDL_SENSOR_ACCEL_R, Hand::Right)));
        assert_eq!(combined.len(), 4);

        // Lone Joy-Cons expose the plain sensor types, routed to their own hand.
        let left = sensors_for(PadKind::Left);
        assert!(left.len() == 2);
        assert!(left[0].0 == SDL_SENSOR_GYRO && left[0].1 == Hand::Left);
        assert!(left[1].0 == SDL_SENSOR_ACCEL && left[1].1 == Hand::Left);

        let right = sensors_for(PadKind::Right);
        assert!(right.len() == 2);
        assert!(right[0].0 == SDL_SENSOR_GYRO && right[0].1 == Hand::Right);
        assert!(right[1].0 == SDL_SENSOR_ACCEL && right[1].1 == Hand::Right);

        // Gyro/accel classification (plain and per-hand variants).
        for gyro in [SDL_SENSOR_GYRO, SDL_SENSOR_GYRO_L, SDL_SENSOR_GYRO_R] {
            assert!(is_gyro(gyro), "{} should be a gyro", sensor_label(gyro));
        }
        for accel in [SDL_SENSOR_ACCEL, SDL_SENSOR_ACCEL_L, SDL_SENSOR_ACCEL_R] {
            assert!(
                !is_gyro(accel),
                "{} should be an accel",
                sensor_label(accel)
            );
        }
    }

    #[test]
    fn gap_resets_dt_instead_of_exploding() {
        let mut fusion = ImuFusion::new();
        let mut ts = 0u64;
        feed(&mut fusion, Vec3::ZERO, Vec3::Y * GRAVITY, &mut ts, 220);
        let q_before = fusion.q;

        // 1 s of 1000 rad/s with a 1 s gap: the gap sample must not integrate.
        ts += 1_000_000_000;
        fusion.push(
            body_to_device(Vec3::new(1000.0, 0.0, 0.0)),
            body_to_device(Vec3::Y * GRAVITY),
            ts,
        );
        let drift = fusion.q.angle_between(q_before).to_degrees();
        assert!(drift < 1.0, "gap must reset dt, drifted {drift} deg");

        // The next sample is only 5 ms later, so it integrates normally.
        fusion.push(
            body_to_device(Vec3::X),
            body_to_device(Vec3::Y * GRAVITY),
            ts + 5_000_000,
        );
        assert!(fusion.q.angle_between(q_before).to_degrees() > 0.1);
    }

    /// Regression for "the left hand drifts left on its own": a wrong first bias (the hand moved
    /// while connecting) must be corrected as soon as the Joy-Con rests, then yaw must hold.
    #[test]
    fn wrong_initial_bias_self_corrects_at_rest() {
        let true_bias = Vec3::new(0.0, 0.05, 0.0); // ~2.9 deg/s of yaw drift if uncorrected
        let up = Vec3::Y * GRAVITY;
        let mut fusion = ImuFusion::new();
        let mut ts = 0u64;
        // 1.2 s of wobbly handling at connect: never still, integration starts with bias 0.
        for i in 0..240 {
            let wobble = Vec3::new(0.0, if i % 20 < 10 { 0.4 } else { -0.4 }, 0.0);
            feed(&mut fusion, true_bias + wobble, up, &mut ts, 1);
        }
        assert!(fusion.bias_ready);
        // Then it rests (with sensor noise well under the stillness threshold).
        for i in 0..600 {
            let noise = Vec3::new(0.0, if i % 2 == 0 { 0.003 } else { -0.003 }, 0.0);
            feed(&mut fusion, true_bias + noise, up, &mut ts, 1);
        }
        assert_vec3(fusion.bias, true_bias, 0.005);
        // Once corrected, 10 s at rest drifts < 2 deg of yaw.
        let q0 = fusion.q;
        feed(&mut fusion, true_bias, up, &mut ts, 2000);
        let drift = fusion.q.angle_between(q0).to_degrees();
        assert!(drift < 2.0, "yaw drifted {drift} deg in 10 s at rest");
    }

    /// A smooth real turn is steady too, but too fast to be bias: it must not be absorbed.
    #[test]
    fn steady_slow_turn_is_not_absorbed_as_bias() {
        let mut fusion = ImuFusion::new();
        let mut ts = 0u64;
        feed(&mut fusion, Vec3::ZERO, Vec3::Y * GRAVITY, &mut ts, 220);
        feed(&mut fusion, Vec3::Y * 0.3, Vec3::Y * GRAVITY, &mut ts, 400); // 2 s at 0.3 rad/s
        assert_vec3(fusion.bias, Vec3::ZERO, 1e-4);
        let yaw = fusion.q.to_euler(EulerRot::YXZ).0;
        assert!(
            (yaw - 0.6).abs() < 0.02,
            "expected 0.6 rad of yaw, got {yaw}"
        );
    }

    // --- Reach (dead reckoning) -----------------------------------------------

    /// Feed the reach tracker at 200 Hz with a body-space linear acceleration added to rest
    /// gravity; `orientation` stays identity (yaw already recentered, hand level).
    fn feed_reach(
        reach: &mut ReachTracker,
        gravity: f32,
        linear: impl Fn(f32) -> Vec3,
        seconds: f32,
        ts: &mut u64,
    ) {
        let samples = (seconds * 200.0) as u32;
        for i in 0..samples {
            let t = i as f32 / 200.0;
            let accel = Vec3::Y * gravity + linear(t);
            reach.push(accel, Vec3::ZERO, Quat::IDENTITY, *ts, 0);
            *ts += 5_000_000;
        }
    }

    /// The Joy-Con R at rest measures ~8.89 m/s², not 9.81. Subtracting a fixed 9.81 would make
    /// the hand sink forever; the per-hand ĝ must learn the real value and stay put.
    #[test]
    fn reach_at_rest_does_not_drift_with_misscaled_gravity() {
        let mut reach = ReachTracker::new();
        let mut ts = 0u64;
        let noise = |i: u32| {
            let d = (i % 7) as f32 - 3.0;
            Vec3::new(d * 0.004, d * 0.006, -d * 0.004)
        };
        let mut sample = 0u32;
        // 10 s at rest with 8.9 m/s² of measured gravity, like the right Joy-Con on a table.
        for _ in 0..2000 {
            let accel = Vec3::Y * 8.9 + noise(sample);
            reach.push(accel, Vec3::ZERO, Quat::IDENTITY, ts, 0);
            ts += 5_000_000;
            sample += 1;
        }
        assert!(reach.gravity_ready, "the first quiet window should set ĝ");
        assert!(
            (reach.gravity - 8.9).abs() < 0.05,
            "ĝ should learn 8.9, got {}",
            reach.gravity
        );
        assert!(
            reach.offset.length() < 0.01,
            "hand must stay put at rest, offset {}",
            reach.offset
        );
    }

    /// Accelerate forward, decelerate to a stop: the hand travels ~0.3 m forward (−Z) and
    /// the ZUPT then holds it there instead of letting it snap back or keep sliding.
    #[test]
    fn reach_forward_push_moves_then_holds() {
        let mut reach = ReachTracker::new();
        let mut ts = 0u64;
        // 1.2 s quiet: ĝ is learned and the depth initialises.
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 1.2, &mut ts);

        // 0.32 s accelerating forward at 3 m/s², then 0.32 s braking to a stop: a realistic
        // ~0.31 m push.
        let push = |t: f32| {
            if t < 0.32 {
                Vec3::NEG_Z * 3.0
            } else if t < 0.64 {
                Vec3::Z * 3.0
            } else {
                Vec3::ZERO
            }
        };
        feed_reach(&mut reach, GRAVITY, push, 0.9, &mut ts);
        let after_push = reach.offset;
        assert!(
            after_push.z < -0.2,
            "a forward push should move the hand forward, got {after_push}"
        );
        assert!(
            after_push.x.abs() < 0.02 && after_push.y.abs() < 0.02,
            "the push is forward only, got {after_push}"
        );

        // 1 s still: the movement is over and the depth holds.
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 1.0, &mut ts);
        assert!(reach.velocity.length() < 1e-3, "the hand should be at rest");
        assert!(
            reach.offset.z < -0.2,
            "the hand must stay extended after stopping, got {}",
            reach.offset
        );

        // And the leak is slow: a couple of seconds later it is still readable.
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 0.5, &mut ts);
        assert!(
            reach.offset.z < -0.15,
            "leak should be slow, got {}",
            reach.offset
        );

        // But it does run: after ~25 s at rest the hand is back near the model pose, so the
        // unavoidable drift is bounded instead of accumulating forever.
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 40.0, &mut ts);
        assert!(
            reach.offset.length() < 0.02,
            "the offset should return to the model pose, got {}",
            reach.offset
        );
    }

    /// Sensor noise and a skewed gravity estimate must not random-walk the hand away.
    #[test]
    fn reach_long_noisy_still_stays_bounded() {
        let mut reach = ReachTracker::new();
        let mut ts = 0u64;
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 0.2, &mut ts);
        for i in 0..12_000u32 {
            // ±0.06 m/s² of per-axis noise, a little gyro jitter (a calibrated hand at rest).
            let n = |k: u32| {
                ((i.wrapping_mul(2654435761).wrapping_add(k) % 1000) as f32 / 999.0 - 0.5) * 0.12
            };
            let accel = Vec3::new(n(1), GRAVITY + n(2), n(3));
            let gyro = Vec3::new(n(4), n(5), n(6)) * 0.05;
            reach.push(accel, gyro, Quat::IDENTITY, ts, 0);
            ts += 5_000_000;
            assert!(
                reach.offset.length() <= REACH_MAX + 1e-4,
                "offset escaped its clamp at sample {i}: {}",
                reach.offset
            );
        }
        assert!(
            reach.offset.length() < 0.02,
            "a noisy resting hand should stay near zero, got {}",
            reach.offset
        );
    }

    #[test]
    fn reach_recenter_resets_the_offset() {
        let mut reach = ReachTracker::new();
        let mut ts = 0u64;
        feed_reach(&mut reach, GRAVITY, |_| Vec3::ZERO, 1.2, &mut ts);
        feed_reach(&mut reach, GRAVITY, |_| Vec3::NEG_Z * 4.0, 0.3, &mut ts);
        assert!(reach.offset.length() > 0.01);
        reach.reset();
        assert_eq!(reach.offset, Vec3::ZERO);
        assert_eq!(reach.velocity, Vec3::ZERO);
    }

    // --- M2c: per-axis calibration, motion-gated tilt, CSV replay -------------------------

    /// Synthetic Joy-Con: per-axis scale/offset errors, gyro bias and white noise.
    /// `raw()` returns exactly what SDL would hand the filters (raw device frame).
    struct SimJoyCon {
        truth: AccelCalib,
        bias: Vec3,
        seed: u32,
    }

    impl SimJoyCon {
        fn new(seed: u32) -> Self {
            Self {
                // ±4 % scale and ~0.1-0.15 m/s² offset, like the measured Joy-Cons.
                truth: AccelCalib {
                    scale: Vec3::new(1.04, 0.97, 1.03),
                    offset: Vec3::new(0.15, -0.12, 0.10),
                },
                bias: Vec3::new(0.010, -0.020, 0.005),
                seed,
            }
        }

        fn noise(&mut self, amp: f32) -> f32 {
            // xorshift32: deterministic across runs, no rand crate.
            self.seed ^= self.seed << 13;
            self.seed ^= self.seed >> 17;
            self.seed ^= self.seed << 5;
            let unit = (self.seed >> 8) as f32 / 16_777_216.0; // [0, 1)
            (unit * 2.0 - 1.0) * amp
        }

        fn raw_accel(&mut self, a_true_body: Vec3) -> Vec3 {
            let raw = a_true_body / self.truth.scale + self.truth.offset;
            raw + Vec3::new(self.noise(0.02), self.noise(0.02), self.noise(0.02))
        }

        /// `q` is the true body->world orientation, `omega` the true body angular velocity and
        /// `linear_world` the true world-frame linear acceleration (0 = pure rotation).
        fn raw(&mut self, q: Quat, omega: Vec3, linear_world: Vec3) -> (Vec3, Vec3) {
            let a_true = q.conjugate() * (Vec3::Y * GRAVITY + linear_world);
            let accel = self.raw_accel(a_true);
            let gyro = omega
                + self.bias
                + Vec3::new(self.noise(0.004), self.noise(0.004), self.noise(0.004));
            (gyro, accel)
        }
    }

    fn tilt_error_deg(fusion: &ImuFusion) -> f32 {
        // The true orientation in these tests is identity, so body +Y is the true up.
        (fusion.q.conjugate() * Vec3::Y)
            .angle_between(Vec3::Y)
            .to_degrees()
    }

    fn yaw_delta_deg(a: Quat, b: Quat) -> f32 {
        let ya = a.to_euler(EulerRot::YXZ).0.to_degrees();
        let yb = b.to_euler(EulerRot::YXZ).0.to_degrees();
        ((yb - ya + 180.0).rem_euclid(360.0)) - 180.0
    }

    #[test]
    fn calibration_fit_recovers_simulated_scale_and_offset() {
        let mut sim = SimJoyCon::new(42);
        let truth = sim.truth;
        // Six faces, 20 s of still samples each (means average the sensor noise down).
        let mut means = Vec::new();
        for up_body in [
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
        ] {
            let mut sum = Vec3::ZERO;
            for _ in 0..4000 {
                sum += sim.raw_accel(up_body * GRAVITY);
            }
            means.push(sum / 4000.0);
        }

        let (fit, before, after) = AccelCalib::fit(&means).expect("six faces must fit");
        println!(
            "[M2c] calib fit: scale=({:.5},{:.5},{:.5}) offset=({:+.5},{:+.5},{:+.5}) \
             RMS {:.4} -> {:.4} m/s2",
            fit.scale.x,
            fit.scale.y,
            fit.scale.z,
            fit.offset.x,
            fit.offset.y,
            fit.offset.z,
            before,
            after
        );

        for axis in 0..3 {
            let got = fit.scale[axis];
            let want = truth.scale[axis];
            let relative = ((got - want) / want).abs();
            assert!(
                relative < 0.01,
                "scale[{axis}] recovered {got:.5}, true {want:.5} ({:.2}% off)",
                relative * 100.0
            );
            let offset_error = (fit.offset[axis] - truth.offset[axis]).abs();
            let offset_relative = offset_error / truth.offset[axis].abs().max(1e-6);
            println!(
                "[M2c] calib axis {axis}: scale error {:.3}%, offset error {offset_error:.6} m/s2 \
                 ({:.3}%)",
                relative * 100.0,
                offset_relative * 100.0
            );
            assert!(
                offset_relative < 0.01,
                "offset[{axis}] recovered {:.5}, true {:.5} (error {offset_error:.5}, {:.2}%)",
                fit.offset[axis],
                truth.offset[axis],
                offset_relative * 100.0
            );
        }
        // The whole point: the residual after the fit is a small fraction of the raw one.
        assert!(
            after < before * 0.1,
            "residual should collapse: {before:.4} -> {after:.4} m/s2"
        );
    }

    #[test]
    fn calib_collector_requires_stillness_and_diversity() {
        let mut collector = CalibCollector::new();
        let mut ts = 0u64;
        // A wobbling window (even with a full 1 s) is not a still pose.
        collector.begin_step();
        for i in 0..240 {
            let jitter = if i % 2 == 0 { 1.0 } else { -1.0 };
            collector.push(ts, Vec3::Y * GRAVITY + Vec3::X * jitter, Vec3::ZERO);
            ts += 5_000_000;
        }
        assert_eq!(collector.take_pose(), CalibPose::Waiting);

        // A quiet window is accepted.
        collector.begin_step();
        for _ in 0..240 {
            collector.push(ts, Vec3::Y * 9.9, Vec3::ZERO);
            ts += 5_000_000;
        }
        assert!(matches!(collector.take_pose(), CalibPose::Accepted(_)));

        // Repeating the same face is rejected, so the fit never gets a degenerate pose set.
        collector.begin_step();
        for _ in 0..240 {
            collector.push(ts, Vec3::Y * 9.9, Vec3::ZERO);
            ts += 5_000_000;
        }
        assert!(matches!(collector.take_pose(), CalibPose::Duplicate(_)));
        assert_eq!(collector.poses().len(), 1);

        // 90° away is a new pose.
        collector.begin_step();
        for _ in 0..240 {
            collector.push(ts, Vec3::X * 9.9, Vec3::ZERO);
            ts += 5_000_000;
        }
        assert!(matches!(collector.take_pose(), CalibPose::Accepted(_)));
        assert_eq!(collector.poses().len(), 2);
    }

    #[test]
    fn calibration_text_round_trips_and_rejects_garbage() {
        let mut cal = ImuCalibration::default();
        cal.hands[Hand::Left.index()] = Some(AccelCalib {
            scale: Vec3::new(1.0312, 0.9871, 1.0043),
            offset: Vec3::new(0.0812, -0.0420, 0.1130),
        });
        cal.hands[Hand::Right.index()] = Some(AccelCalib {
            scale: Vec3::new(0.9550, 1.0402, 0.9910),
            offset: Vec3::new(-0.1100, 0.0950, -0.0310),
        });
        let text = cal.to_text();
        let parsed = ImuCalibration::parse_text(&text).expect("round trip");
        assert_eq!(parsed.hands, cal.hands);

        // Missing fields and unknown keys are errors, not silently-ignored lines.
        assert!(ImuCalibration::parse_text("left scale 1 1\n").is_err());
        assert!(ImuCalibration::parse_text("left offset 0 0 0\n").is_err());
        assert!(ImuCalibration::parse_text("middle scale 1 1 1\n").is_err());
        assert!(ImuCalibration::parse_text("left nope 1 1 1\n").is_err());
        assert!(
            ImuCalibration::parse_text("left scale 1 1 1\nleft offset 0 0 0\nright scale 1 1 1\n")
                .is_err()
        );
    }

    /// A 2 s side-to-side shake (±3 m/s² at 2 Hz, ±1.9 cm, no rotation) may only tilt the
    /// pointer a few degrees and must fully recover. A real hand can't hold a one-way 3 m/s² pull
    /// (that is 3 m/s after a second), so the shake oscillates. This is the price of the faster
    /// tilt correction (gain 4, no gyro gate) that the real recordings need: ~7° here versus
    /// 9-35° of tilt free-run on real data with the M2c gate. `motion_gate = false` is the pre-M2c
    /// hard-band filter, printed for reference.
    #[test]
    fn motion_gate_keeps_tilt_during_a_lateral_shake() {
        let mut sim = SimJoyCon::new(99);
        let truth = sim.truth;
        let mut gated = ImuFusion::with_calibration(Some(truth));
        let mut ungated = ImuFusion::with_calibration(Some(truth));
        ungated.motion_gate = false;

        let mut gated_max = 0.0_f32;
        let mut ungated_max = 0.0_f32;
        let mut gated_final = 0.0_f32;
        let mut ungated_final = 0.0_f32;
        let mut ts = 0u64;

        // 1.5 s still (bias calibration), 2.0 s shake, 1.0 s settle.
        for i in 0..900 {
            let t = i as f32 / 200.0;
            let linear = if (1.5..3.5).contains(&t) {
                Vec3::X * 3.0 * (std::f32::consts::TAU * 2.0 * (t - 1.5)).sin()
            } else {
                Vec3::ZERO
            };
            let (gyro, accel) = sim.raw(Quat::IDENTITY, Vec3::ZERO, linear);
            gated.push(gyro, accel, ts);
            ungated.push(gyro, accel, ts);

            let ge = tilt_error_deg(&gated);
            let ue = tilt_error_deg(&ungated);
            if (1.5..3.5).contains(&t) {
                gated_max = gated_max.max(ge);
                ungated_max = ungated_max.max(ue);
            }
            if (t - 4.45).abs() < 0.0026 {
                gated_final = ge;
                ungated_final = ue;
            }
            ts += 5_000_000;
        }

        println!(
            "[M2c] lateral shake tilt error: gated max {gated_max:.2}° final {gated_final:.2}°; \
             ungated (pre-M2c) max {ungated_max:.2}° final {ungated_final:.2}°"
        );
        assert!(
            gated_max < 8.0,
            "tilt error reached {gated_max:.2}° during the shake"
        );
        assert!(gated_final < 1.0, "tilt still {gated_final:.2}° off after the shake");
    }

    /// The calibration must also restore the *orientation* input: feed a tilted pose through a
    /// Joy-Con with 4 % axis errors and check the fused tilt with vs without the file.
    #[test]
    fn calibrated_accel_restores_tilt_at_rest() {
        let mut sim = SimJoyCon::new(7);
        let truth = sim.truth;
        let pitch = 0.5_f32;
        let q_true = Quat::from_rotation_x(pitch);
        let mut calibrated = ImuFusion::with_calibration(Some(truth));
        let mut raw_only = ImuFusion::with_calibration(None);
        let mut ts = 0u64;
        for _ in 0..600 {
            let (gyro, accel) = sim.raw(q_true, Vec3::ZERO, Vec3::ZERO);
            calibrated.push(gyro, accel, ts);
            raw_only.push(gyro, accel, ts);
            ts += 5_000_000;
        }
        let cal_err = calibrated.q.angle_between(q_true).to_degrees();
        let raw_err = raw_only.q.angle_between(q_true).to_degrees();
        println!("[M2c] tilted-pose error: calibrated {cal_err:.2}°, uncalibrated {raw_err:.2}°");
        assert!(cal_err < 1.0, "calibrated tilt off by {cal_err:.2}°");
        assert!(
            raw_err > cal_err,
            "the uncalibrated filter should be worse ({raw_err:.2}° vs {cal_err:.2}°)"
        );
    }

    /// 20 s at rest: the variance-based gyro bias windows must keep working with calibration
    /// applied (the old fix for "the hand drifts in yaw on its own" must not regress).
    #[test]
    fn calibrated_rest_keeps_yaw_drift_below_a_degree_over_20_s() {
        let mut sim = SimJoyCon::new(123);
        let truth = sim.truth;
        let mut fusion = ImuFusion::with_calibration(Some(truth));
        let mut ts = 0u64;
        for _ in 0..400 {
            let (gyro, accel) = sim.raw(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
            fusion.push(gyro, accel, ts);
            ts += 5_000_000;
        }
        let q0 = fusion.q;
        for _ in 0..4000 {
            let (gyro, accel) = sim.raw(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
            fusion.push(gyro, accel, ts);
            ts += 5_000_000;
        }
        let drift = yaw_delta_deg(q0, fusion.q);
        let tilt = tilt_error_deg(&fusion);
        println!("[M2c] 20 s calibrated rest: yaw drift {drift:+.3}°, tilt {tilt:.3}°");
        assert!(drift.abs() < 1.0, "yaw drifted {drift:.3}° in 20 s at rest");
        assert!(tilt < 1.0, "tilt drifted {tilt:.3}° in 20 s at rest");
    }

    /// One replayed sample, including the diagnostics needed to report phases.
    #[derive(Clone, Copy, Debug)]
    struct ReplayRow {
        ts_ns: u64,
        /// Calibrated accel (what the reach pipeline sees).
        accel: Vec3,
        gyro_mag: f32,
        reach: Vec3,
        model_explained: Vec3,
        residual: Vec3,
        still: bool,
        /// Published depth (m, + = forward) and the movement that ended on this sample.
        depth: f32,
        event: Option<ReachEvent>,
    }

    /// Replay one hand from the `joycon_probe --record` CSV format through the real filter
    /// chain. This is the hook for tuning against a real recording (rest / wrist rotation /
    /// 30 cm push and stop) without the hardware attached.
    #[cfg(test)]
    fn replay_csv(csv: &str, hand: Hand, calib: Option<AccelCalib>) -> Vec<ReplayRow> {
        let mut fusion = ImuFusion::with_calibration(calib);
        let mut reach = ReachTracker::with_calibration(calib, hand);
        let mut out = Vec::new();
        for line in csv.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with("ts_ns") {
                continue;
            }
            let fields: Vec<&str> = line.split(',').collect();
            if fields.len() != 8 || fields[1] != hand.short() {
                continue;
            }
            let ts = fields[0].parse::<u64>().expect("csv timestamp");
            let values: Vec<f32> = fields[2..]
                .iter()
                .map(|value| value.parse::<f32>().expect("csv number"))
                .collect();
            let gyro = Vec3::new(values[0], values[1], values[2]);
            let accel = Vec3::new(values[3], values[4], values[5]);
            fusion.push(gyro, accel, ts);
            let body_gyro = device_to_body(gyro) - fusion.bias;
            let orientation = fusion.orientation();
            reach.push(
                device_to_body(accel),
                body_gyro,
                orientation,
                ts,
                fusion.recenter_epoch,
            );
            out.push(ReplayRow {
                ts_ns: ts,
                accel: calib.map_or(accel, |c| c.apply(accel)),
                gyro_mag: body_gyro.length(),
                reach: reach.offset,
                model_explained: reach.model_explained,
                residual: reach.p_imu - reach.model_explained,
                still: reach.still,
                depth: reach.shown,
                event: reach.last_event.filter(|event| event.end_ts == ts),
            });
        }
        out
    }

    #[test]
    fn recorded_csv_replays_through_fusion_and_reach() {
        let mut sim = SimJoyCon::new(3);
        let truth = sim.truth;
        let mut csv = String::from("ts_ns,hand,gx,gy,gz,ax,ay,az\n");
        let mut ts = 0u64;
        // 1.2 s rest so gravity/bias/tilt settle, 0.4 s forward push, 1.4 s still.
        for i in 0..600 {
            let t = i as f32 / 200.0;
            let linear = if (1.2..1.6).contains(&t) {
                let phase = (t - 1.2) / 0.4;
                // Accelerate then brake (one full sine): ~25 cm forward, ending at rest.
                Vec3::NEG_Z * (10.0 * (std::f32::consts::TAU * phase).sin())
            } else {
                Vec3::ZERO
            };
            let (gyro, accel) = sim.raw(Quat::IDENTITY, Vec3::ZERO, linear);
            csv.push_str(&format!(
                "{ts},L,{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z
            ));
            ts += 5_000_000;
        }

        let replay = replay_csv(&csv, Hand::Left, Some(truth));
        assert_eq!(
            replay.len(),
            600,
            "every row of the recorded hand must replay"
        );
        let final_reach = replay.last().unwrap().reach;
        println!(
            "[M2e] csv replay: {} samples, final reach ({:+.3},{:+.3},{:+.3}) m",
            replay.len(),
            final_reach.x,
            final_reach.y,
            final_reach.z
        );
        assert!(
            final_reach.z < -0.05,
            "the recorded forward push should move the reach, got {final_reach}"
        );

        // And the calibration is really used by the replay path.
        let uncalibrated = replay_csv(&csv, Hand::Left, None);
        let alt = uncalibrated.last().unwrap().reach;
        println!(
            "[M2e] csv replay without calibration: final reach z {:+.3} m",
            alt.z
        );
    }

    // --- M2e: synthetic calibrated acceptance (no SDL) ------------------------------------

    /// `model_wrist` must equal `arms::solve_arm(side, q).wrist` exactly. The values below were
    /// printed by the real `arms::solve_arm` (bevy 0.20.1 / glam 0.33.12) for the same
    /// orientations; regenerate them if arms.rs changes.
    #[test]
    fn model_wrist_matches_arms_solve_arm() {
        let q = |x: f32, y: f32, z: f32| {
            Quat::from_rotation_y(y.to_radians())
                * Quat::from_rotation_x(x.to_radians())
                * Quat::from_rotation_z(z.to_radians())
        };
        let cases = [
            (Hand::Right, Quat::IDENTITY, Vec3::new(0.195, -0.5, -0.255)),
            (
                Hand::Right,
                Quat::from_rotation_x(60.0_f32.to_radians()),
                Vec3::new(0.065, -0.26775521, -0.31735945),
            ),
            (
                Hand::Right,
                Quat::from_rotation_x(-45.0_f32.to_radians()),
                Vec3::new(0.195, -0.61351472, -0.22774291),
            ),
            (
                Hand::Right,
                Quat::from_rotation_x(80.0_f32.to_radians()),
                Vec3::new(0.065, -0.23805562, -0.30324197),
            ),
            (
                Hand::Right,
                q(0.0, 30.0, 20.0),
                Vec3::new(0.11774467, -0.5, -0.24276382),
            ),
            (Hand::Left, Quat::IDENTITY, Vec3::new(-0.195, -0.5, -0.255)),
            (
                Hand::Left,
                Quat::from_rotation_x(60.0_f32.to_radians()),
                Vec3::new(-0.065, -0.26775521, -0.31735945),
            ),
            (
                Hand::Left,
                q(30.0, -25.0, 0.0),
                Vec3::new(-0.09053946, -0.37360910, -0.27611399),
            ),
        ];
        for (hand, orientation, expected) in cases {
            assert_vec3(model_wrist(hand, orientation), expected, 1e-5);
        }
    }

    /// A Joy-Con as it looks *after* the `--calibrate` fit: only the residual per-axis scale
    /// (±0.5 %), offset (0.03 m/s²) and noise (0.02 m/s²) remain. `raw` returns exactly what
    /// SDL reports, so the filter's own calibration is [`AccelCalib::IDENTITY`].
    struct CalibratedSim {
        scale: Vec3,
        offset: Vec3,
        seed: u32,
    }

    impl CalibratedSim {
        fn new(seed: u32) -> Self {
            Self {
                scale: Vec3::new(1.004, 0.996, 1.003),
                offset: Vec3::new(0.02, -0.03, 0.015),
                seed,
            }
        }

        fn noise(&mut self, amp: f32) -> f32 {
            self.seed ^= self.seed << 13;
            self.seed ^= self.seed >> 17;
            self.seed ^= self.seed << 5;
            let unit = (self.seed >> 8) as f32 / 16_777_216.0; // [0, 1)
            (unit * 2.0 - 1.0) * amp
        }

        /// `q_true` is body→world, `omega_body` the body angular rate and `a_world` the linear
        /// acceleration at the IMU (the controller centre for the rotation-in-place case).
        fn raw(&mut self, q_true: Quat, omega_body: Vec3, a_world: Vec3) -> (Vec3, Vec3) {
            let a_body = q_true.conjugate() * (Vec3::Y * GRAVITY + a_world);
            let accel = self.scale * a_body
                + self.offset
                + Vec3::new(self.noise(0.02), self.noise(0.02), self.noise(0.02));
            let gyro = omega_body
                + Vec3::new(self.noise(0.004), self.noise(0.004), self.noise(0.004));
            (gyro, accel)
        }
    }

    /// Fusion + reach fed by [`CalibratedSim`] at 200 Hz, scored on the *published* reach.
    struct ReachSim {
        sim: CalibratedSim,
        fusion: ImuFusion,
        reach: ReachTracker,
        ts: u64,
        max_reach: f32,
    }

    impl ReachSim {
        fn new(hand: Hand) -> Self {
            let calib = Some(AccelCalib::IDENTITY);
            Self {
                sim: CalibratedSim::new(2026),
                fusion: ImuFusion::with_calibration(calib),
                reach: ReachTracker::with_calibration(calib, hand),
                ts: 0,
                max_reach: 0.0,
            }
        }

        fn step(&mut self, q: Quat, omega: Vec3, a_world: Vec3) {
            let (gyro, accel) = self.sim.raw(q, omega, a_world);
            self.fusion.push(gyro, accel, self.ts);
            self.reach.push(
                accel,
                gyro - self.fusion.bias,
                self.fusion.orientation(),
                self.ts,
                self.fusion.recenter_epoch,
            );
            self.max_reach = self.max_reach.max(self.reach.offset.length());
            self.ts += 5_000_000;
        }

        fn hold(&mut self, seconds: f32, q: Quat) {
            let samples = (seconds * 200.0) as u32;
            for _ in 0..samples {
                self.step(q, Vec3::ZERO, Vec3::ZERO);
            }
        }
    }

    fn ease(t: f32, duration: f32) -> f32 {
        let u = (t / duration).clamp(0.0, 1.0);
        u * u * (3.0 - 2.0 * u)
    }

    /// Body angular rate from an orientation trajectory (central difference).
    fn omega_body(q: impl Fn(f32) -> Quat, t: f32, h: f32) -> Vec3 {
        let before = q(t - h);
        let after = q(t + h);
        let (axis, angle) = (after * before.conjugate()).to_axis_angle();
        let omega_world = axis * (angle / (2.0 * h));
        before.conjugate() * omega_world
    }

    /// Numerical second derivative of the model wrist along an orientation trajectory.
    fn model_wrist_accel(hand: Hand, q: impl Fn(f32) -> Quat, t: f32, h: f32) -> Vec3 {
        (model_wrist(hand, q(t + h)) - model_wrist(hand, q(t)) * 2.0
            + model_wrist(hand, q(t - h)))
            / (h * h)
    }

    /// 20 s of calibrated rest: the published reach must stay under 1 cm.
    #[test]
    fn m2e_rest_20s_reach_below_one_centimetre() {
        let mut sim = ReachSim::new(Hand::Right);
        sim.hold(1.5, Quat::IDENTITY); // bias + anchor
        sim.max_reach = 0.0;
        sim.hold(20.0, Quat::IDENTITY);
        println!(
            "[M2e] 20 s rest: max |reach| {:.2} cm, final ({:+.2},{:+.2},{:+.2}) cm",
            sim.max_reach * 100.0,
            sim.reach.offset.x * 100.0,
            sim.reach.offset.y * 100.0,
            sim.reach.offset.z * 100.0
        );
        assert!(
            sim.max_reach < 0.01,
            "20 s at rest drifted to {:.2} cm",
            sim.max_reach * 100.0
        );
    }

    /// 90° roll + 60° pitch in 2 s about the controller's own centre: the accelerometer sees
    /// no translation, so the IMU must not add any. (The Daydream model's own wrist swing is
    /// the baseline and remains; `reach` is the *added* translation.)
    #[test]
    fn m2e_pure_wrist_rotation_adds_no_reach() {
        let duration = 2.0;
        let q = |t: f32| {
            let u = ease(t, duration);
            Quat::from_euler(
                EulerRot::YXZ,
                0.0,
                60.0_f32.to_radians() * u,
                90.0_f32.to_radians() * u,
            )
        };
        let mut sim = ReachSim::new(Hand::Right);
        sim.hold(1.5, q(0.0));
        let model_swing =
            model_wrist(Hand::Right, q(duration)) - model_wrist(Hand::Right, q(0.0));
        sim.max_reach = 0.0;
        let samples = (duration * 200.0) as u32;
        for i in 0..=samples {
            let t = i as f32 / 200.0;
            let omega = omega_body(q, t, 1e-3);
            sim.step(q(t), omega, Vec3::ZERO);
        }
        println!(
            "[M2e] pure wrist rotation: max |reach| {:.2} cm (model's own swing {:.1} cm)",
            sim.max_reach * 100.0,
            model_swing.length() * 100.0
        );
        assert!(
            model_swing.length() > 0.15,
            "the test is meaningless if the model does not move the wrist: {model_swing}"
        );
        assert!(
            sim.max_reach < 0.03,
            "pure rotation added {:.2} cm of reach",
            sim.max_reach * 100.0
        );
    }

    /// A forearm swing that follows the Daydream model's wrist path: rotation WITH the matching
    /// translation. The model already moves the wrist, so the IMU must not double count it.
    #[test]
    fn m2e_model_wrist_path_swing_does_not_double_count() {
        let duration = 2.0;
        let q = |t: f32| Quat::from_rotation_x(60.0_f32.to_radians() * ease(t, duration));
        let mut sim = ReachSim::new(Hand::Right);
        sim.hold(1.5, q(0.0));
        sim.max_reach = 0.0;
        let samples = (duration * 200.0) as u32;
        for i in 0..=samples {
            let t = i as f32 / 200.0;
            let a_world = model_wrist_accel(Hand::Right, q, t, 0.002);
            let omega = omega_body(q, t, 1e-3);
            sim.step(q(t), omega, a_world);
        }
        println!(
            "[M2e] model-path forearm swing: max |reach| {:.2} cm, final ({:+.2},{:+.2},{:+.2}) cm",
            sim.max_reach * 100.0,
            sim.reach.offset.x * 100.0,
            sim.reach.offset.y * 100.0,
            sim.reach.offset.z * 100.0
        );
        assert!(
            sim.max_reach < 0.03,
            "the model-following swing was double counted ({:.2} cm)",
            sim.max_reach * 100.0
        );
    }

    /// A 30 cm forward push in 0.4 s (smooth half-sine acceleration, zero end velocity) and a
    /// hold: depth must arrive, not overshoot, and stay put for at least a second.
    #[test]
    fn m2e_push_30cm_reaches_and_holds() {
        let mut sim = ReachSim::new(Hand::Right);
        sim.hold(1.5, Quat::IDENTITY);
        let duration = 0.4;
        // ∫∫ A·sin(2πt/T) dt over [0,T] = A·T²/(2π) = 0.30 m.
        let amplitude = 0.30 * std::f32::consts::TAU / (duration * duration);
        let samples = (duration * 200.0) as u32;
        for i in 0..samples {
            let t = i as f32 / 200.0;
            let a = Vec3::NEG_Z * amplitude * (std::f32::consts::TAU * t / duration).sin();
            sim.step(Quat::IDENTITY, Vec3::ZERO, a);
        }
        let at_stop = sim.reach.offset;
        let mut max_abs_z = at_stop.z.abs();
        // One second of hold, measured per sample for the overshoot.
        for _ in 0..200 {
            sim.step(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
            max_abs_z = max_abs_z.max(sim.reach.offset.z.abs());
        }
        let after_1s = sim.reach.offset;
        let overshoot = max_abs_z - at_stop.z.abs();
        println!(
            "[M2e] 30 cm push: at stop z {:.1} cm, after 1 s z {:.1} cm, overshoot {:.1} mm, \
             x/y ({:+.1},{:+.1}) cm",
            at_stop.z * 100.0,
            after_1s.z * 100.0,
            overshoot * 1000.0,
            after_1s.x * 100.0,
            after_1s.y * 100.0
        );
        assert!(
            (-0.35..=-0.25).contains(&at_stop.z),
            "push depth should be 25-35 cm, got {:.1} cm",
            at_stop.z * 100.0
        );
        assert!(overshoot < 0.02, "overshoot {:.1} cm", overshoot * 100.0);
        assert!(
            (after_1s - at_stop).length() < 0.03,
            "the held reach moved {:.1} cm in 1 s",
            (after_1s - at_stop).length() * 100.0
        );
        assert!(
            at_stop.x.abs() < 0.02 && at_stop.y.abs() < 0.02,
            "the push should be forward only, got {at_stop}"
        );
    }

    /// Push 30 cm and pull back to the start: the hand must return to the model pose.
    #[test]
    fn m2e_push_then_pull_returns_to_start() {
        let mut sim = ReachSim::new(Hand::Right);
        sim.hold(1.5, Quat::IDENTITY);
        let duration = 0.4;
        let amplitude = 0.30 * std::f32::consts::TAU / (duration * duration);
        let samples = (duration * 200.0) as u32;
        let profile = |direction: f32| {
            let mut out = Vec::new();
            for i in 0..samples {
                let t = i as f32 / 200.0;
                let a = direction * amplitude * (std::f32::consts::TAU * t / duration).sin();
                out.push(Vec3::NEG_Z * a);
            }
            out
        };
        for a in profile(1.0) {
            sim.step(Quat::IDENTITY, Vec3::ZERO, a);
        }
        sim.hold(0.2, Quat::IDENTITY);
        for a in profile(-1.0) {
            sim.step(Quat::IDENTITY, Vec3::ZERO, a);
        }
        sim.hold(0.5, Quat::IDENTITY);
        println!(
            "[M2e] push+pull: final reach ({:+.2},{:+.2},{:+.2}) cm",
            sim.reach.offset.x * 100.0,
            sim.reach.offset.y * 100.0,
            sim.reach.offset.z * 100.0
        );
        assert!(
            sim.reach.offset.length() < 0.03,
            "push+pull should return to the start, got {:.1} cm",
            sim.reach.offset.length() * 100.0
        );
    }

    #[test]
    fn reach_policy_defaults_to_calibration_and_respects_the_env_override() {
        assert!(reach_policy(None, true), "calibrated hands reach by default");
        assert!(!reach_policy(None, false), "uncalibrated hands stay off");
        assert!(reach_policy(Some(true), false), "JOYREACH_REACH=1 forces on");
        assert!(!reach_policy(Some(false), true), "JOYREACH_REACH=0 forces off");
    }

    // --- M2e: real recordings -----------------------------------------------------------------

    /// The `--calibrate` file the recordings were made with.
    fn real_calibration() -> Option<ImuCalibration> {
        let path = imu_calib_path()?;
        if !path.exists() {
            return None;
        }
        ImuCalibration::load(&path).ok()
    }

    fn read_recording(name: &str) -> Option<String> {
        std::fs::read_to_string(format!("target/{name}")).ok()
    }

    /// Print a 10 Hz table and an automatic still/active segmentation. Returns the segments.
    #[derive(Clone, Copy, Debug)]
    struct Phase {
        start: f32,
        end: f32,
        active: bool,
        gyro: f32,
        accel_dev: f32,
        max_reach: f32,
        end_reach: Vec3,
        min_z: f32,
    }

    fn report_recording(label: &str, hand: Hand, rows: &[ReplayRow]) -> Vec<Phase> {
        let t0 = rows[0].ts_ns;
        println!("\n[M2e] {label} / {:?}: {} samples", hand, rows.len());
        println!(
            "  {:>6} {:>7} {:>6} {:>18} {:>18} {:>16}",
            "t(s)", "|gyro|", "still", "reach cm (x,y,z)", "model expl. cm", "resid. cm"
        );
        let mut next_tenth = 0.0_f32;
        for row in rows {
            let t = (row.ts_ns - t0) as f32 * 1e-9;
            if t + 1e-6 < next_tenth {
                continue;
            }
            next_tenth += 0.1;
            let reach = row.reach * 100.0;
            let model = row.model_explained * 100.0;
            let residual = row.residual * 100.0;
            println!(
                "  {t:6.1} {:7.2} {:>6} ({:>+5.1},{:>+5.1},{:>+5.1})  ({:>+5.1},{:>+5.1},{:>+5.1})  ({:>+5.1},{:>+5.1},{:>+5.1})",
                row.gyro_mag,
                if row.still { "still" } else { "move" },
                reach.x,
                reach.y,
                reach.z,
                model.x,
                model.y,
                model.z,
                residual.x,
                residual.y,
                residual.z,
            );
        }

        // Activity = gyro or a real acceleration change. Merge runs shorter than 120 ms.
        let mut raw: Vec<(bool, usize, usize)> = Vec::new();
        for (index, row) in rows.iter().enumerate() {
            let active = row.gyro_mag > 0.25 || (row.accel.length() - GRAVITY).abs() > 0.45;
            match raw.last_mut() {
                Some((flag, _, end)) if *flag == active => *end = index + 1,
                _ => raw.push((active, index, index + 1)),
            }
        }
        let mut merged: Vec<(bool, usize, usize)> = Vec::new();
        for (flag, start, end) in raw {
            let duration = (rows[end - 1].ts_ns - rows[start].ts_ns) as f32 * 1e-9;
            if duration < 0.12 {
                if let Some(last) = merged.last_mut() {
                    last.2 = end;
                    continue;
                }
            }
            merged.push((flag, start, end));
        }

        let mut phases = Vec::new();
        for (active, start, end) in merged {
            let segment = &rows[start..end];
            if segment.is_empty() {
                continue;
            }
            let start_t = (segment[0].ts_ns - t0) as f32 * 1e-9;
            let end_t = (segment[segment.len() - 1].ts_ns - t0) as f32 * 1e-9;
            let n = segment.len() as f32;
            let gyro = segment.iter().map(|row| row.gyro_mag).sum::<f32>() / n;
            let accel_dev = segment
                .iter()
                .map(|row| (row.accel.length() - GRAVITY).abs())
                .sum::<f32>()
                / n;
            let max_reach = segment
                .iter()
                .map(|row| row.reach.length())
                .fold(0.0_f32, f32::max);
            let min_z = segment
                .iter()
                .map(|row| row.reach.z)
                .fold(0.0_f32, f32::min);
            let end_reach = segment.last().unwrap().reach;
            phases.push(Phase {
                start: start_t,
                end: end_t,
                active,
                gyro,
                accel_dev,
                max_reach,
                end_reach,
                min_z,
            });
        }

        println!(
            "\n  {:>14} {:>5} {:>7} {:>8} {:>9} {:>20} {:>9}",
            "phase (s)", "kind", "dur(s)", "|gyro|", "|a|-g|", "end reach cm", "max|r|cm"
        );
        for phase in &phases {
            println!(
                "  {:>6.1}-{:<6.1} {:>5} {:>7.2} {:>8.2} {:>9.2} ({:>+5.1},{:>+5.1},{:>+5.1}) {:>9.1}",
                phase.start,
                phase.end,
                if phase.active { "move" } else { "still" },
                phase.end - phase.start,
                phase.gyro,
                phase.accel_dev,
                phase.end_reach.x * 100.0,
                phase.end_reach.y * 100.0,
                phase.end_reach.z * 100.0,
                phase.max_reach * 100.0,
            );
        }
        phases
    }

    /// Tilt error (deg) at the onset of each still spell: angle between the estimated up and the
    /// measured accel direction on the first sample of 100 ms of real rest.
    fn tilt_at_rest_onsets(csv: &str, hand: Hand, calib: Option<AccelCalib>) -> Vec<(f32, f32)> {
        let mut fusion = ImuFusion::with_calibration(calib);
        let mut rows: Vec<(u64, Vec3, Vec3, Quat)> = Vec::new();
        for line in csv.lines().skip(1) {
            let f: Vec<&str> = line.split(',').collect();
            if f.len() != 8 || f[1] != hand.short() {
                continue;
            }
            let ts = f[0].parse::<u64>().unwrap();
            let v: Vec<f32> = f[2..].iter().map(|x| x.parse().unwrap()).collect();
            let gyro = Vec3::new(v[0], v[1], v[2]);
            let accel = Vec3::new(v[3], v[4], v[5]);
            fusion.push(gyro, accel, ts);
            rows.push((ts, gyro - fusion.bias, calib.map_or(accel, |c| c.apply(accel)), fusion.q));
        }
        let t0 = rows.first().map_or(0, |r| r.0);
        let quiet = |r: &(u64, Vec3, Vec3, Quat)| r.1.length() < 0.15 && (r.2.length() - GRAVITY).abs() < 0.15;
        let mut out = Vec::new();
        let mut i = 0;
        let mut was_moving = false;
        while i + 20 < rows.len() {
            if !quiet(&rows[i]) {
                was_moving = true;
                i += 1;
                continue;
            }
            if was_moving && rows[i..i + 20].iter().all(quiet) {
                let (ts, _, a, q) = rows[i];
                let up_est = q.conjugate() * Vec3::Y;
                let angle = up_est.angle_between(a.normalize()).to_degrees();
                out.push(((ts - t0) as f32 * 1e-9, angle));
                was_moving = false;
                i += 20;
                continue;
            }
            i += 1;
        }
        out
    }

    /// The user's recordings: whenever the hand comes to rest after moving, the fused tilt must
    /// already agree with gravity. The M2c gyro gate left it 9-35° off here (the tilt free-ran
    /// while the hand was held in the air); the production filter must stay within 6°.
    #[test]
    fn real_tilt_error_when_the_hand_stops() {
        let calib = real_calibration();
        for name in ["reach_session_02.csv", "reach_session.csv"] {
            let Some(csv) = read_recording(name) else {
                println!("[tilt] target/{name} not recorded; skipping");
                continue;
            };
            for hand in [Hand::Left, Hand::Right] {
                let onsets = tilt_at_rest_onsets(&csv, hand, calib.as_ref().and_then(|c| c.get(hand)));
                // The first spell can be the filter converging from identity after connect.
                let settled: Vec<&(f32, f32)> = onsets.iter().filter(|(t, _)| *t > 1.5).collect();
                println!("[tilt] {name} {hand:?}: onsets (t s, error deg) {settled:?}");
                for (t, angle) in settled {
                    assert!(*angle < 6.0, "{name} {hand:?}: tilt {angle:.1}° off at {t:.1} s");
                }
            }
        }
    }

    /// Every movement the tracker judged in both real recordings, plus the depth at 4 Hz.
    /// `cargo test --example joycon_probe real_movement_log -- --nocapture` (tuning aid).
    #[test]
    fn real_movement_log() {
        let calib = real_calibration();
        for name in ["reach_session_02.csv", "reach_session.csv"] {
            let Some(csv) = read_recording(name) else {
                continue;
            };
            for hand in [Hand::Left, Hand::Right] {
                let hand_calib = calib.as_ref().and_then(|c| c.get(hand));
                let rows = replay_csv(&csv, hand, hand_calib);
                let Some(first) = rows.first() else {
                    continue;
                };
                let t0 = first.ts_ns;
                println!("\n[reach] {name} {hand:?}");
                let mut next = 0.0_f32;
                let mut line = String::new();
                for row in &rows {
                    let t = (row.ts_ns - t0) as f32 * 1e-9;
                    if let Some(event) = row.event {
                        println!(
                            "  {t:6.2}s {:?} dur {:.2}s gyro {:4.1} v_end {:.2} delta {:+5.1} cm -> depth {:+5.1}",
                            event.verdict,
                            event.duration,
                            event.max_gyro,
                            event.end_speed,
                            event.delta * 100.0,
                            row.depth * 100.0
                        );
                    }
                    if t >= next {
                        next += 0.25;
                        line.push_str(&format!("{:+.0} ", row.depth * 100.0));
                    }
                }
                println!("  depth cm @4Hz: {line}");
            }
        }
    }

    /// The scripted session (still / wrist rotation / push+hold / pull / punches / still).
    /// Timing is approximate, so the checks use the automatic phase segmentation.
    #[test]
    fn m2e_scripted_recording_reports_and_meets_the_acceptance() {
        let Some(csv) = read_recording("reach_session_02.csv") else {
            println!("[M2e] target/reach_session_02.csv not recorded yet; skipping the real replay");
            return;
        };
        let calib = real_calibration();
        for hand in [Hand::Left, Hand::Right] {
            let hand_calib = calib.as_ref().and_then(|c| c.get(hand));
            let rows = replay_csv(&csv, hand, hand_calib);
            if rows.is_empty() {
                continue;
            }
            let phases = report_recording("scripted", hand, &rows);
            let t0 = rows[0].ts_ns;

            // The initial still segment: no creep (< 1 cm).
            if let Some(rest) = phases.iter().find(|phase| !phase.active) {
                println!(
                    "[M2e] scripted {:?}: initial rest {:.2}-{:.2} s, max |reach| {:.2} cm",
                    hand,
                    rest.start,
                    rest.end,
                    rest.max_reach * 100.0
                );
                assert!(
                    rest.max_reach < 0.01,
                    "{:?}: rest drifted {:.2} cm",
                    hand,
                    rest.max_reach * 100.0
                );
            }

            // First long active segment after the rest: the wrist rotation in place. It must
            // not add reach on top of the model's own swing.
            if let Some(rotation) = phases
                .iter()
                .find(|phase| phase.active && phase.end - phase.start > 0.8)
            {
                println!(
                    "[M2e] scripted {:?}: first active phase {:.2}-{:.2} s, mean |gyro| {:.2} \
                     rad/s, max |reach| {:.2} cm",
                    hand,
                    rotation.start,
                    rotation.end,
                    rotation.gyro,
                    rotation.max_reach * 100.0
                );
                assert!(
                    rotation.max_reach < 0.03,
                    "{:?}: wrist rotation added {:.2} cm of reach",
                    hand,
                    rotation.max_reach * 100.0
                );
            }

            // The deepest push: reach.z must be in the 20-35 cm band and held for a moment.
            let push = phases
                .iter()
                .min_by(|a, b| a.min_z.total_cmp(&b.min_z))
                .copied();
            if let Some(push) = push {
                let held = rows
                    .iter()
                    .filter(|row| {
                        let t = (row.ts_ns - t0) as f32 * 1e-9;
                        t >= push.start && t <= push.end
                    })
                    .filter(|row| row.reach.z < -0.15)
                    .count() as f32
                    / 200.0;
                println!(
                    "[M2e] scripted {:?}: deepest push {:.2}-{:.2} s, min z {:.1} cm, \
                     end z {:.1} cm, {} samples below -15 cm (~{held:.2} s)",
                    hand,
                    push.start,
                    push.end,
                    push.min_z * 100.0,
                    push.end_reach.z * 100.0,
                    rows.iter()
                        .filter(|row| row.reach.z < -0.15)
                        .count()
                );
                assert!(
                    push.min_z > -0.36,
                    "{:?}: push exceeded the clamp ({:.2} m)",
                    hand,
                    push.min_z
                );
            }

            // No sample may pin the 0.35 m clamp (that would be a runaway).
            let pinned = rows.iter().filter(|row| row.reach.length() > 0.349).count();
            println!(
                "[M2e] scripted {:?}: max |reach| {:.1} cm, pinned samples {pinned}",
                hand,
                rows.iter()
                    .map(|row| row.reach.length())
                    .fold(0.0_f32, f32::max)
                    * 100.0
            );
            assert_eq!(pinned, 0, "{:?}: reach pinned the clamp", hand);
        }
    }

    /// The free-movement session: stay inside the clamp without pinning, no creep during still
    /// spells, and return toward neutral when the hand stays still.
    #[test]
    fn m2e_free_recording_stays_bounded_and_returns_when_still() {
        let Some(csv) = read_recording("reach_session.csv") else {
            println!("[M2e] target/reach_session.csv not recorded yet; skipping the real replay");
            return;
        };
        let calib = real_calibration();
        for hand in [Hand::Left, Hand::Right] {
            let hand_calib = calib.as_ref().and_then(|c| c.get(hand));
            let rows = replay_csv(&csv, hand, hand_calib);
            if rows.is_empty() {
                continue;
            }
            let phases = report_recording("free", hand, &rows);

            let max_reach = rows
                .iter()
                .map(|row| row.reach.length())
                .fold(0.0_f32, f32::max);
            let pinned = rows.iter().filter(|row| row.reach.length() > 0.349).count();
            println!(
                "[M2e] free {:?}: max |reach| {:.1} cm, pinned samples {pinned}, still phases {}",
                hand,
                max_reach * 100.0,
                phases.iter().filter(|phase| !phase.active).count()
            );
            assert!(
                max_reach <= REACH_MAX + 1e-3,
                "{:?}: reach escaped the clamp ({:.2} m)",
                hand,
                max_reach
            );
            // A held pose at the clamp for many samples is the M2a runaway symptom.
            assert!(
                pinned * 100 < rows.len(),
                "{:?}: reach pinned the clamp for {pinned} samples",
                hand
            );

            // Still spells: reach must hold, not creep. Compare the first and last second.
            let t0 = rows[0].ts_ns;
            for phase in phases.iter().filter(|phase| !phase.active) {
                if phase.end - phase.start < 0.8 {
                    continue;
                }
                let mut first = None;
                let mut last = Vec3::ZERO;
                let mut count = 0;
                for row in &rows {
                    let t = (row.ts_ns - t0) as f32 * 1e-9;
                    if t < phase.start || t > phase.end {
                        continue;
                    }
                    if first.is_none() {
                        first = Some(row.reach);
                    }
                    last = row.reach;
                    count += 1;
                }
                let first = first.unwrap_or(Vec3::ZERO);
                let drift = (last - first).length();
                println!(
                    "[M2e] free {:?}: still {:.2}-{:.2} s ({count} samples), reach drift {:.2} cm",
                    hand,
                    phase.start,
                    phase.end,
                    drift * 100.0
                );
                // The slow return is deliberate; creep is not. Bound the change per still spell
                // by the drift a bounded dead-reckoning pipeline can have.
                assert!(
                    drift < 0.08,
                    "{:?}: reach crept {:.2} cm during a still spell",
                    hand,
                    drift * 100.0
                );
            }
        }
    }
}
