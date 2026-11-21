//! joyreach — M0 "brazos vivos".
//!
//! Dev flags:
//! - `--screenshot <path>`: let the scene settle for ~4 s, save the primary window, exit.
//! - `JOYREACH_FAKE=1`: drive both hands with animated fake input (no hardware).
//! - `JOYREACH_REACH=1`: opt in to IMU hand depth (off by default; orientation-only otherwise).

mod arms;
mod grab;
mod handray;
mod hud;
mod joycon;
mod player;
mod theme;
mod world;

use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
use bevy::window::{PresentMode, WindowLevel, WindowPlugin, WindowResolution};
use std::env;

use joycon::Hands;

fn main() {
    let mut args = env::args().skip(1);
    let mut screenshot_path = None;
    while let Some(arg) = args.next() {
        if arg == "--screenshot" {
            screenshot_path = args.next();
        }
    }
    let fake = env::var("JOYREACH_FAKE").is_ok_and(|value| value == "1");
    let screenshot = screenshot_path.is_some();

    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "joyreach".into(),
                resolution: WindowResolution::new(1600, 900),
                // Low input latency matters more than tearing here.
                present_mode: PresentMode::AutoNoVsync,
                // Screenshots only: a background/occluded window can lose its 3D pass
                // (the HUD still draws), so keep the capture window above the terminal.
                window_level: if screenshot {
                    WindowLevel::AlwaysOnTop
                } else {
                    WindowLevel::Normal
                },
                focused: screenshot,
                ..default()
            }),
            ..default()
        }),
        world::WorldPlugin,
        player::PlayerPlugin,
        arms::ArmsPlugin,
        handray::HandRayPlugin,
        grab::GrabPlugin,
        hud::HudPlugin,
        joycon::JoyConPlugin,
    ))
    // Fixed 120 Hz: the hands are kinematic bodies driven by velocity, so a
    // high fixed rate keeps pushes crisp and the hand from tunnelling.
    .insert_resource(Time::<Fixed>::from_hz(120.0));

    if fake {
        // Runs at the start of `Update`, before every gameplay consumer. The joycon
        // plugin copies device state in `PreUpdate`, so this replaces it for the frame.
        app.insert_resource(FakeHands(Hands::default()))
            .add_systems(Update, fake_hands.in_set(player::InputSet));
    }

    if let Some(path) = screenshot_path {
        app.insert_resource(ScreenshotRequest {
            path,
            timer: 0.0,
            requested: false,
        })
        .add_systems(Update, take_screenshot);
    }

    app.run();
}

/// Animated stand-in for the Joy-Con so the arms can be checked without hardware.
/// Also mirrored into [`FakeHands`] because the fixed-step physics reads input
/// before `Update` runs and would otherwise never see the fake state.
///
/// Script (so `--screenshot` at ~4 s catches a beam grab at distance):
/// 0–0.6 s the fake player walks back (left stick) so the bench props are beyond the
/// direct-grab radius; 0–1.5 s slow wave; 1.5–20 s both hands settle and aim at the bench,
/// the right one beam-grabs a prop at 2.2 s, pushes the beam target out to ~2.3 m, and
/// twists its wrist from 3.1 s so the held prop rotates for the shot.
fn fake_hands(mut hands: ResMut<Hands>, mut fake: ResMut<FakeHands>, time: Res<Time>) {
    let t = time.elapsed_secs();
    for index in 0..2 {
        let hand = if index == 0 {
            &mut hands.left
        } else {
            &mut hands.right
        };
        let phase = index as f32 * 1.7;

        hand.connected = true;
        hand.imu_hz = 200.0;

        // Slow, readable waves: pitch −20°..+50°, yaw ±25°, roll ±20°.
        let pitch = 15.0_f32.to_radians()
            + 35.0_f32.to_radians() * (t * 0.8 + phase).sin();
        let yaw = 25.0_f32.to_radians() * (t * 0.55 + phase * 0.7).sin();
        let roll = 20.0_f32.to_radians() * (t * 0.9 + phase * 1.3).sin();
        hand.orientation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

        // Trigger stays open during the wave phase so the demo prop is untouched.
        hand.trigger = false;

        // Defaults first: the bench script below opts back into the buttons it needs.
        hand.shoulder = false;
        hand.face = [false; 4];
        hand.stick = Vec2::ZERO;
        hand.stick_click = false;
        hand.menu = false;

        // Walk backwards for the first 0.6 s: at the spawn the palm hovers on the bench
        // props, so only from ~1.5 m away does the beam (not the direct grab) reach them.
        if index == 0 && t < 0.6 {
            hand.stick = Vec2::new(0.0, -1.0);
        }

        if (1.5..20.0).contains(&t) {
            // Bench demo: hands turned in over the props. The right hand aims at the bench
            // from afar and closes at 2.2 s; X then pushes the beam target out to ~2.3 m.
            let micro = ((t * 1.3 + phase).sin() * 0.8).to_radians();
            // The right palm sits ~1.15 m high and ~1.7 m from the props (tops ~1.05 m):
            // −6° lands the ray on them instead of skimming over the bench.
            // After the grab it tilts up to lift the prop clear of the bench for the shot.
            let lift = ((t - 2.4) / 0.6).clamp(0.0, 1.0);
            let pitch: f32 = if index == 0 { 5.0 } else { -6.0 + 16.0 * lift };
            let roll = if index == 1 && t > 3.1 {
                35.0_f32.to_radians() * ((t - 3.1) * 2.0).sin()
            } else {
                0.0
            };
            hand.orientation =
                Quat::from_euler(EulerRot::YXZ, micro, pitch.to_radians(), roll);
            hand.trigger = index == 1 && t > 2.2;
            if index == 1 {
                hand.face[0] = (2.45..3.1).contains(&t);
            }
        } else if t >= 20.0 {
            // Release (throw tick) after the demo and keep waving.
            hand.trigger = ((t - 20.0 + index as f32 * 0.75) / 1.5) as i64 % 2 == 0;
        }

        hand.gyro = Vec3::ZERO;
        hand.accel = Vec3::new(0.0, 9.81, 0.0);
        // M2c: depth is opt-in. The fake demo must obey the same switch, otherwise the
        // screenshot would show a reach that the real input path never publishes.
        if joycon::reach_enabled() {
            let reach_wave = (t * 0.75 - index as f32 * 0.9).sin();
            hand.reach = Vec3::new(0.0, 0.02 * reach_wave, -(0.18 + 0.17 * reach_wave));
            hand.reach_velocity = Vec3::new(
                0.0,
                0.0,
                -0.17 * 0.75 * (t * 0.75 - index as f32 * 0.9).cos(),
            );
        } else {
            hand.reach = Vec3::ZERO;
            hand.reach_velocity = Vec3::ZERO;
        }
    }
    fake.0 = hands.clone();
}

/// Latest fake input, for systems that run outside `Update` (the physics step).
#[derive(Resource)]
pub struct FakeHands(pub Hands);

#[derive(Resource)]
struct ScreenshotRequest {
    path: String,
    timer: f32,
    requested: bool,
}

/// Wait for the sky/SSAO/TAA to settle, capture, then quit once the file is written.
fn take_screenshot(
    time: Res<Time>,
    mut request: ResMut<ScreenshotRequest>,
    mut commands: Commands,
) {
    if request.requested {
        return;
    }
    request.timer += time.delta_secs();
    if request.timer < 4.0 {
        return;
    }
    request.requested = true;

    let path = request.path.clone();
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path))
        .observe(|_: On<ScreenshotCaptured>, mut commands: Commands| {
            commands.write_message(AppExit::Success);
        });
}
