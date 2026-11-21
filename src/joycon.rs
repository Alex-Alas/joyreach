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

use std::ffi::CStr;
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
    /// Calibrated gyro (rad/s) and accel (m/s^2) in body space, for gestures/debug.
    pub gyro: Vec3,
    pub accel: Vec3,
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
            gyro: Vec3::ZERO,
            accel: Vec3::ZERO,
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
        let hands = Arc::new(Mutex::new(Hands::default()));
        let debug = Arc::new(Mutex::new(Diagnostics::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (rumble_tx, rumble_rx) = mpsc::channel::<RumbleCommand>();

        let thread = {
            let hands = Arc::clone(&hands);
            let debug = Arc::clone(&debug);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name("joycon-sdl".to_string())
                .spawn(move || sdl_thread_main(hands, debug, stop, rumble_rx))
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
                if let Some(pad) = open_joycon(id) {
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
                handle_event(&event, &mut pads);
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
            for rt in pads.iter_mut().flat_map(|p| [p.left.as_mut(), p.right.as_mut()]).flatten() {
                rt.fusion.recenter();
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
        unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
    }
}

fn cstr(ptr: *const std::ffi::c_char) -> String {
    if ptr.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
    }
}

// ---------------------------------------------------------------------------------------------
// Pads and hands
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Hand {
    Left,
    Right,
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
    /// Last raw sample, SDL device frame.
    last_gyro: Vec3,
    last_accel: Vec3,
    /// Gyro sample waiting for its accel twin (same `sensor_timestamp`).
    pending_gyro: Option<(u64, Vec3)>,
    has_accel: bool,
    prev_menu: bool,
}

impl HandRuntime {
    fn new() -> Self {
        Self {
            input: HandInput {
                connected: true,
                ..Default::default()
            },
            fusion: ImuFusion::new(),
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
fn open_joycon(id: SDL_JoystickID) -> Option<Pad> {
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

        let mut pad = Pad {
            id,
            ptr,
            kind,
            gamepad_name,
            gamepad_type,
            sensors: Vec::new(),
            notes: Vec::new(),
            left: (kind != PadKind::Right).then(HandRuntime::new),
            right: (kind != PadKind::Left).then(HandRuntime::new),
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
                pad.notes.push(format!("{} unavailable: {err}", sensor_label(ty)));
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
    const LEFT: &[(SDL_SensorType, Hand)] =
        &[(SDL_SENSOR_GYRO, Hand::Left), (SDL_SENSOR_ACCEL, Hand::Left)];
    const RIGHT: &[(SDL_SensorType, Hand)] =
        &[(SDL_SENSOR_GYRO, Hand::Right), (SDL_SENSOR_ACCEL, Hand::Right)];

    match kind {
        PadKind::Combined => COMBINED,
        PadKind::Left => LEFT,
        PadKind::Right => RIGHT,
    }
}

fn handle_event(event: &SDL_Event, pads: &mut Vec<Pad>) {
    match unsafe { event.r#type } {
        t if t == SDL_EVENT_GAMEPAD_ADDED.0 => {
            let id = unsafe { event.gdevice.which };
            if pads.iter().any(|p| p.id == id) {
                return;
            }
            if let Some(pad) = open_joycon(id) {
                // A combined pair supersedes any single-pad view of the same hands; drop the
                // singles so we don't straddle two topologies while SDL tears them down.
                if pad.kind == PadKind::Combined {
                    let (singles, old_combined): (Vec<Pad>, Vec<Pad>) = pads
                        .drain(..)
                        .partition(|p| p.kind != PadKind::Combined);
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

    let rt = match binding.hand {
        Hand::Left => pad.left.get_or_insert_with(HandRuntime::new),
        Hand::Right => pad.right.get_or_insert_with(HandRuntime::new),
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
        let mut d = HandDebug {
            connected: true,
            topology: self.kind.topology(),
            gamepad_name: self.gamepad_name.clone(),
            gamepad_type: self.gamepad_type.clone(),
            raw_gyro: rt.last_gyro,
            raw_accel: rt.last_accel,
            body_gyro: device_to_body(rt.last_gyro) - rt.fusion.bias,
            body_accel: device_to_body(rt.last_accel),
            gyro_bias: rt.fusion.bias,
            bias_ready: rt.fusion.bias_ready,
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
/// Mahony accelerometer gain (1/s). ~0.5 s time constant at 200 Hz.
const ACCEL_KP: f32 = 2.0;
/// Only trust the accelerometer for tilt when |a| is within this band of 1 g (linear acceleration
/// during a swing would otherwise tilt the estimate).
const ACCEL_TRUST_BAND: f32 = 1.5;
/// "Still" threshold for slow gyro-bias re-estimation.
const GYRO_STILL_EPS: f32 = 0.03;
const GYRO_STILL_NS: u64 = 500_000_000;
/// Samples above this while calibrating are considered motion and excluded from the average.
const BIAS_INIT_MAX_GYRO: f32 = 0.5;
const BIAS_INIT_NS: u64 = 1_000_000_000;
/// Ignore gaps longer than this (BT stall / reconnect): no integration for that step.
const DT_MAX_NS: u64 = 50_000_000;
/// Auto-recenter after this many samples (~0.2 s at 200 Hz) = "first stable estimate".
const AUTO_RECENTER_SAMPLES: u32 = 40;

/// Mahony-style complementary filter, gravity-aligned world frame (+Y up), body-to-world
/// quaternion (Bevy `Transform::rotation` convention: it maps body/local vectors to world).
///
/// The accelerometer only corrects tilt (it cannot observe yaw); yaw comes from the gyro and is
/// re-zeroed by [`ImuFusion::recenter`].
#[derive(Debug)]
struct ImuFusion {
    /// body -> world.
    q: Quat,
    /// World-space yaw applied on the left of `q` so the current heading maps to body -Z.
    yaw_offset: Quat,
    /// Gyro bias in body space (rad/s).
    bias: Vec3,
    bias_ready: bool,
    bias_start_ts: Option<u64>,
    bias_sum: Vec3,
    bias_n: u32,
    still_since: Option<u64>,
    still_sum: Vec3,
    still_n: u32,
    last_ts: Option<u64>,
    samples: u32,
    auto_recentered: bool,
    hz_start_ts: Option<u64>,
    hz_count: u32,
    hz: f32,
}

impl ImuFusion {
    fn new() -> Self {
        Self {
            q: Quat::IDENTITY,
            yaw_offset: Quat::IDENTITY,
            bias: Vec3::ZERO,
            bias_ready: false,
            bias_start_ts: None,
            bias_sum: Vec3::ZERO,
            bias_n: 0,
            still_since: None,
            still_sum: Vec3::ZERO,
            still_n: 0,
            last_ts: None,
            samples: 0,
            auto_recentered: false,
            hz_start_ts: None,
            hz_count: 0,
            hz: 0.0,
        }
    }

    /// One IMU sample. `gyro_dev`/`accel_dev` are raw SDL device-frame values; `ts_ns` is the
    /// event's `sensor_timestamp`.
    fn push(&mut self, gyro_dev: Vec3, accel_dev: Vec3, ts_ns: u64) {
        let gyro = device_to_body(gyro_dev);
        let accel = device_to_body(accel_dev);

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

        self.update_bias(gyro, ts_ns);

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
        if (a_norm - GRAVITY).abs() < ACCEL_TRUST_BAND && a_norm > 0.1 {
            // World up expressed in body space, per the current estimate.
            let up_est = self.q.conjugate() * Vec3::Y;
            let a_hat = accel / a_norm;
            // Mahony tilt error; positive gain drives `up_est` toward the measured up.
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

    fn update_bias(&mut self, gyro: Vec3, ts_ns: u64) {
        if !self.bias_ready {
            let start = *self.bias_start_ts.get_or_insert(ts_ns);
            if gyro.length() < BIAS_INIT_MAX_GYRO {
                self.bias_sum += gyro;
                self.bias_n += 1;
            }
            if ts_ns.saturating_sub(start) >= BIAS_INIT_NS {
                if self.bias_n >= 10 {
                    self.bias = self.bias_sum / self.bias_n as f32;
                    eprintln!(
                        "[joycon] gyro bias calibrated: |{:.4}| rad/s over {} samples",
                        self.bias.length(),
                        self.bias_n
                    );
                }
                // Even if the user was moving, start integrating: the still-based re-estimation
                // below fixes the bias as soon as the Joy-Con settles.
                self.bias_ready = true;
            }
            return;
        }

        // Slow, quiet-only re-estimation: after 0.5 s under the still threshold, blend the
        // window average into the bias by 5% (per window).
        if (gyro - self.bias).length() < GYRO_STILL_EPS {
            let still_since = *self.still_since.get_or_insert(ts_ns);
            self.still_sum += gyro;
            self.still_n += 1;
            if ts_ns.saturating_sub(still_since) >= GYRO_STILL_NS && self.still_n >= 20 {
                let avg = self.still_sum / self.still_n as f32;
                self.bias = self.bias * 0.95 + avg * 0.05;
                self.still_since = Some(ts_ns);
                self.still_sum = Vec3::ZERO;
                self.still_n = 0;
            }
        } else {
            self.still_since = None;
            self.still_sum = Vec3::ZERO;
            self.still_n = 0;
        }
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
    }

    /// Final orientation for `HandInput`: body space with a recentered heading.
    fn orientation(&self) -> Quat {
        (self.yaw_offset * self.q).normalize()
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
            fusion.push(
                body_to_device(gyro_body),
                body_to_device(accel_body),
                *ts,
            );
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
        feed(
            &mut fusion,
            Vec3::Y,
            Vec3::Y * GRAVITY,
            &mut ts,
            200,
        );

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
        assert!(forward.z < -0.9, "forward should point at the screen: {forward}");

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
        assert_eq!(
            apply_stick_deadzone(Vec2::new(0.08, -0.08)),
            Vec2::ZERO
        );
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
            assert!(!is_gyro(accel), "{} should be an accel", sensor_label(accel));
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
}
