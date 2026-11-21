//! M0 player: no physics body yet. A yaw-only body, an eye at 1.60 m and a
//! pitch camera (FOV 80°). Left stick moves, right stick looks (quadratic), with
//! WASD + grabbed-mouse as a development fallback.

use bevy::anti_alias::taa::TemporalAntiAliasing;
use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::AtmosphereEnvironmentMapLight;
use bevy::pbr::FogFalloff;
use bevy::pbr::{AtmosphereSettings, ScreenSpaceAmbientOcclusion};
use bevy::input::mouse::MouseMotion;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions};

use crate::joycon::Hands;

pub const EYE_HEIGHT: f32 = 1.6;
/// Initial pitch, about −28°: the workbench and both hands sit in the lower frame.
pub const START_PITCH: f32 = -0.524;
const MOVE_SPEED: f32 = 3.0;
const STICK_LOOK_YAW: f32 = 150.0_f32.to_radians();
const STICK_LOOK_PITCH: f32 = 110.0_f32.to_radians();
const MOUSE_SENSITIVITY: f32 = 0.0022;
const WALK_LIMIT: f32 = crate::world::TERRACE_HALF - 0.7;

/// The whole player state, in body space. `position` is on the deck (y = 0).
#[derive(Resource, Debug)]
pub struct PlayerRig {
    pub position: Vec3,
    pub yaw: f32,
    /// Positive looks up. Clamped to ±85°.
    pub pitch: f32,
    pub velocity: Vec3,
}

impl Default for PlayerRig {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            yaw: 0.0,
            pitch: START_PITCH,
            velocity: Vec3::ZERO,
        }
    }
}

/// Root of the player. Yaw only: arms are children and must not follow camera pitch.
#[derive(Component)]
pub struct PlayerBody;

#[derive(Component)]
pub struct PlayerCamera;

/// All player systems run in this set, so the arms can read a fresh body transform.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PlayerSet;

/// Optional input override (fake Joy-Con) runs before every gameplay consumer.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct InputSet;

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PlayerRig>()
            .add_systems(Startup, spawn_player)
            .configure_sets(Update, PlayerSet.after(InputSet))
            .add_systems(
                Update,
                (
                    grab_cursor,
                    look_with_hands,
                    look_with_mouse,
                    move_player,
                    write_player_transform,
                )
                    .chain()
                    .in_set(PlayerSet),
            );
    }
}

pub fn spawn_player(mut commands: Commands, rig: Res<PlayerRig>) {
    commands
        .spawn((
            PlayerBody,
            Transform::from_translation(rig.position)
                .with_rotation(Quat::from_rotation_y(rig.yaw)),
            Visibility::default(),
        ))
        .with_children(|parent| {
            parent.spawn((
                PlayerCamera,
                Camera3d::default(),
                Projection::Perspective(PerspectiveProjection {
                    fov: 80.0_f32.to_radians(),
                    near: 0.05,
                    ..default()
                }),
                Transform::from_xyz(0.0, EYE_HEIGHT, 0.0)
                    .with_rotation(Quat::from_rotation_x(rig.pitch)),
                // Rendering: HDR + AgX, a whisper of bloom for the emissive cursors,
                // contact occlusion and temporal AA (MSAA must be off for both).
                Hdr,
                Msaa::Off,
                // AgX washed the neon sleeves to salmon/pastel; Tony keeps the hands saturated.
                Tonemapping::TonyMcMapface,
                Bloom {
                    intensity: 0.045,
                    low_frequency_boost: 0.2,
                    ..default()
                },
                Exposure { ev100: 13.6 },
                AtmosphereSettings::default(),
                AtmosphereEnvironmentMapLight::default(),
                ScreenSpaceAmbientOcclusion::default(),
                TemporalAntiAliasing::default(),
                // Haze only far away; the table stays crisp.
                DistanceFog {
                    color: Color::srgba(0.68, 0.75, 0.84, 1.0),
                    directional_light_color: Color::srgba(1.0, 0.94, 0.85, 0.4),
                    directional_light_exponent: 30.0,
                    falloff: FogFalloff::Linear {
                        start: 18.0,
                        end: 110.0,
                    },
                },
                IsDefaultUiCamera,
            ));
        });
}

fn grab_cursor(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut cursor: Single<&mut CursorOptions>,
) {
    if mouse.just_pressed(MouseButton::Left) && cursor.grab_mode == CursorGrabMode::None {
        cursor.visible = false;
        cursor.grab_mode = CursorGrabMode::Locked;
    }
    if keys.just_pressed(KeyCode::Escape) {
        cursor.visible = true;
        cursor.grab_mode = CursorGrabMode::None;
    }
}

fn look_with_hands(hands: Res<Hands>, time: Res<Time>, mut rig: ResMut<PlayerRig>) {
    let stick = hands.right.stick;
    // Quadratic response: small deflections stay precise, full tilt turns fast.
    let x = stick.x * stick.x.abs();
    let y = stick.y * stick.y.abs();
    let dt = time.delta_secs();

    rig.yaw -= x * STICK_LOOK_YAW * dt;
    rig.pitch = (rig.pitch + y * STICK_LOOK_PITCH * dt).clamp(-1.483, 1.483);
}

fn look_with_mouse(
    mut motion: MessageReader<MouseMotion>,
    cursor: Single<&CursorOptions>,
    mut rig: ResMut<PlayerRig>,
) {
    if cursor.grab_mode == CursorGrabMode::None {
        return;
    }
    for event in motion.read() {
        rig.yaw -= event.delta.x * MOUSE_SENSITIVITY;
        rig.pitch = (rig.pitch - event.delta.y * MOUSE_SENSITIVITY).clamp(-1.483, 1.483);
    }
}

fn move_player(
    hands: Res<Hands>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut rig: ResMut<PlayerRig>,
) {
    let stick = hands.left.stick;
    let mut input = Vec2::new(stick.x, stick.y);

    let mut keyboard = Vec2::ZERO;
    if keys.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
        keyboard.y += 1.0;
    }
    if keys.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
        keyboard.y -= 1.0;
    }
    if keys.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
        keyboard.x += 1.0;
    }
    if keys.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
        keyboard.x -= 1.0;
    }
    input += keyboard;
    let input = if input.length_squared() > 1.0 {
        input.normalize()
    } else {
        input
    };

    // Stick +Y is forward, which is -Z in body space.
    let direction = Quat::from_rotation_y(rig.yaw) * Vec3::new(input.x, 0.0, -input.y);
    let target = direction * MOVE_SPEED;

    // Quick acceleration, a touch slower when releasing: responsive but not snappy.
    let tau = if target.length_squared() > 0.01 { 0.055 } else { 0.09 };
    let blend = 1.0 - (-time.delta_secs() / tau).exp();
    rig.velocity = rig.velocity.lerp(target, blend);
    if rig.velocity.length_squared() > MOVE_SPEED * MOVE_SPEED {
        rig.velocity = rig.velocity.normalize() * MOVE_SPEED;
    }

    let velocity = rig.velocity;
    rig.position += velocity * time.delta_secs();
    rig.position.x = rig.position.x.clamp(-WALK_LIMIT, WALK_LIMIT);
    rig.position.z = rig.position.z.clamp(-WALK_LIMIT, WALK_LIMIT);
    rig.position.y = 0.0;
}

fn write_player_transform(
    rig: Res<PlayerRig>,
    mut body: Single<&mut Transform, (With<PlayerBody>, Without<PlayerCamera>)>,
    mut camera: Single<&mut Transform, (With<PlayerCamera>, Without<PlayerBody>)>,
) {
    body.translation = rig.position;
    body.rotation = Quat::from_rotation_y(rig.yaw);

    camera.translation = Vec3::new(0.0, EYE_HEIGHT, 0.0);
    camera.rotation = Quat::from_rotation_x(rig.pitch);
}
