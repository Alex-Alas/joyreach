//! joyreach — M0 "brazos vivos".
//!
//! Dev flags:
//! - `--screenshot <path>`: let the scene settle for ~4 s, save the primary window, exit.
//! - `JOYREACH_FAKE=1`: drive both hands with animated fake input (no hardware).

mod arms;
mod hud;
mod joycon;
mod player;
mod theme;
mod world;

use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, ScreenshotCaptured, save_to_disk};
use bevy::window::{PresentMode, WindowPlugin, WindowResolution};
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

    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "joyreach".into(),
                resolution: WindowResolution::new(1600, 900),
                // Low input latency matters more than tearing here.
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }),
        world::WorldPlugin,
        player::PlayerPlugin,
        arms::ArmsPlugin,
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

        // Triggers toggle every 1.5 s, offset per hand, to check the finger curl.
        hand.trigger = ((t + index as f32 * 0.75) / 1.5) as i64 % 2 == 0;
        hand.shoulder = false;
        hand.face = [false; 4];
        hand.stick = Vec2::ZERO;
        hand.stick_click = false;
        hand.menu = false;
        hand.gyro = Vec3::ZERO;
        hand.accel = Vec3::new(0.0, 9.81, 0.0);
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
