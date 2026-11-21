//! M2b player: a kinematic capsule character with gravity, ground detection and
//! coyote-time jump, plus a stance that eases the eye height between standing and
//! crouched — automatically as the camera looks down, or on a manual toggle
//! (left stick click / C / Ctrl).
//!
//! Movement, gravity and collision response follow avian's `kinematic_character_3d`
//! example (collide-and-slide through [`MoveAndSlide`], with a downward shape cast
//! for ground detection).
//!
//! The body root is a *rig origin*, not the feet: it sits at
//! `feet + (eye_height − EYE_HEIGHT)`, so the camera and the whole arm chain
//! (anchored at `EYE_HEIGHT` in `arms.rs`) come down together when crouching
//! without touching the arm solver.

use avian3d::math::*;
use avian3d::prelude::*;
use bevy::anti_alias::taa::TemporalAntiAliasing;
use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::MouseMotion;
use bevy::light::AtmosphereEnvironmentMapLight;
use bevy::pbr::FogFalloff;
use bevy::pbr::{AtmosphereSettings, ScreenSpaceAmbientOcclusion};
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions};

use crate::joycon::Hands;

/// Standing eye height. Also the arm chain anchor in `arms.rs`.
pub const EYE_HEIGHT: f32 = 1.6;
/// Initial pitch, about −28°: the workbench and both hands sit in the lower frame.
pub const START_PITCH: f32 = -0.524;

/// Eye height with the manual crouch toggle on.
pub const CROUCH_EYE_HEIGHT: f32 = 0.95;
/// Eye height at the bottom of the automatic look-down crouch.
pub const AUTO_CROUCH_EYE_HEIGHT: f32 = 0.90;
/// Looking down past this pitch starts lowering the eyes (DESIGN §6, brief M2).
pub const AUTO_CROUCH_START: f32 = 35.0_f32.to_radians();
/// …and this is where they reach [`AUTO_CROUCH_EYE_HEIGHT`].
pub const AUTO_CROUCH_END: f32 = 80.0_f32.to_radians();
/// Eye-height easing: ~95 % of the way in 150 ms, no head-bob.
const EYE_TAU: f32 = 0.05;

/// Capsule radius. The workbench top and the parapets are the things it must not
/// pass through.
pub const PLAYER_RADIUS: f32 = 0.25;
/// Capsule total height standing (eyes 0.15 m below the crown).
pub const STAND_HEIGHT: f32 = 1.75;
/// Capsule total height crouched (fits the 0.95 m manual crouch).
pub const CROUCH_HEIGHT: f32 = 1.10;

const MOVE_SPEED: f32 = 3.0;
const CROUCH_SPEED: f32 = 1.4;
/// Jump apex, metres; [`jump_speed`] derives the launch speed from it.
const JUMP_HEIGHT: f32 = 0.6;
/// A little heavier than Earth so the jump does not feel floaty on a flat screen.
const GRAVITY: f32 = 12.0;
const TERMINAL_VELOCITY: f32 = 20.0;
const COYOTE_TIME: f32 = 0.1;
/// Slopes up to this angle count as ground (the deck and the parapet tops).
const MAX_SLOPE: f32 = 30.0_f32.to_radians();

const STICK_LOOK_YAW: f32 = 150.0_f32.to_radians();
const STICK_LOOK_PITCH: f32 = 110.0_f32.to_radians();
const MOUSE_SENSITIVITY: f32 = 0.0022;
const WALK_LIMIT: f32 = crate::world::TERRACE_HALF - 0.7;

/// The whole player state, in body space. `position` is the capsule's feet on the deck.
#[derive(Resource, Debug)]
pub struct PlayerRig {
    pub position: Vec3,
    pub yaw: f32,
    /// Positive looks up. Clamped to ±85°.
    pub pitch: f32,
    /// Full velocity; the vertical part carries jumps and gravity.
    pub velocity: Vec3,
    /// Current eye height above the feet, eased towards [`target_eye_height`].
    pub eye_height: f32,
    /// Height of the kinematic capsule (standing or crouched).
    pub stance_height: f32,
    /// Manual crouch toggle.
    pub crouched: bool,
    pub grounded: bool,
    /// Seconds of coyote time left.
    pub coyote: f32,
    /// One-shot jump request, consumed by the next move step.
    pub jump_queued: bool,
    /// Level state of the edge-triggered buttons, for press detection.
    prev_stick_click: bool,
    prev_jump_button: bool,
}

impl Default for PlayerRig {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            yaw: 0.0,
            pitch: START_PITCH,
            velocity: Vec3::ZERO,
            eye_height: EYE_HEIGHT,
            stance_height: STAND_HEIGHT,
            crouched: false,
            grounded: true,
            coyote: COYOTE_TIME,
            jump_queued: false,
            prev_stick_click: false,
            prev_jump_button: false,
        }
    }
}

/// Root of the player visuals. Yaw only: arms are children and must not follow
/// camera pitch. Its translation is a rig origin, not the feet (see module docs).
#[derive(Component)]
pub struct PlayerBody;

#[derive(Component)]
pub struct PlayerCamera;

/// The kinematic capsule that moves and collides. Separate from [`PlayerBody`] so
/// the visuals and the body of the character can be offset independently.
#[derive(Component)]
pub struct PlayerCapsule;

/// All player systems run in this set, so the arms can read a fresh body transform.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PlayerSet;

/// Optional input override (fake Joy-Con) runs before every gameplay consumer.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct InputSet;

// --- Pure rules (unit tested) -------------------------------------------------

/// Eye height from camera pitch alone: standing until −35°, then a smoothstep down
/// to 0.90 m at −80° and below. Monotonic: looking further down never raises the eyes.
pub fn auto_crouch_eye_height(pitch: f32) -> f32 {
    let t = ((-pitch) - AUTO_CROUCH_START) / (AUTO_CROUCH_END - AUTO_CROUCH_START);
    let t = t.clamp(0.0, 1.0);
    let eased = t * t * (3.0 - 2.0 * t);
    EYE_HEIGHT + (AUTO_CROUCH_EYE_HEIGHT - EYE_HEIGHT) * eased
}

/// Target eye height for the current pitch and manual crouch (the lower of the two).
pub fn target_eye_height(pitch: f32, crouched: bool) -> f32 {
    let auto = auto_crouch_eye_height(pitch);
    if crouched {
        auto.min(CROUCH_EYE_HEIGHT)
    } else {
        auto
    }
}

/// Crouch is a toggle: only a fresh press flips it.
pub fn toggle_crouch(crouched: bool, pressed: bool) -> bool {
    if pressed { !crouched } else { crouched }
}

/// Jump is allowed while grounded or during the coyote window after leaving ground.
pub fn jump_allowed(grounded: bool, coyote: f32) -> bool {
    grounded || coyote > 0.0
}

/// Walk speed for the current stance.
pub fn walk_speed(crouched: bool) -> f32 {
    if crouched { CROUCH_SPEED } else { MOVE_SPEED }
}

/// Launch speed that gives a [`JUMP_HEIGHT`] apex at our gravity (`v = √(2gh)`).
pub fn jump_speed() -> f32 {
    (2.0 * GRAVITY * JUMP_HEIGHT).sqrt()
}

/// Capsule for a given total height, centred on the entity origin.
fn player_capsule(height: f32) -> Collider {
    Collider::capsule(PLAYER_RADIUS, (height - PLAYER_RADIUS * 2.0).max(0.05))
}

// --- Plugin -------------------------------------------------------------------

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        let mut rig = PlayerRig::default();
        // Dev only: `JOYREACH_PITCH=<deg>` starts the camera at that pitch, so
        // `--screenshot` can capture the look-down crouch.
        if let Some(deg) = std::env::var("JOYREACH_PITCH")
            .ok()
            .and_then(|value| value.parse::<f32>().ok())
        {
            rig.pitch = deg.to_radians().clamp(-1.483, 1.483);
        }
        rig.eye_height = target_eye_height(rig.pitch, rig.crouched);

        app.insert_resource(rig)
            .add_systems(Startup, spawn_player)
            .configure_sets(Update, PlayerSet.after(InputSet))
            .add_systems(
                Update,
                (
                    grab_cursor,
                    look_with_hands,
                    look_with_mouse,
                    stance_input,
                    check_headroom,
                    update_stance,
                    ease_eye_height,
                    move_player,
                    write_player_transform,
                )
                    .chain()
                    .in_set(PlayerSet),
            );
    }
}

pub fn spawn_player(mut commands: Commands, rig: Res<PlayerRig>) {
    let eye_height = rig.eye_height;
    commands
        .spawn((
            PlayerBody,
            Transform::from_translation(rig.position + Vec3::Y * (eye_height - EYE_HEIGHT))
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

    // The character body: a kinematic capsule that is moved by hand (move-and-slide)
    // instead of by the integrator. `CustomPositionIntegration` keeps avian from
    // applying its velocity to the position a second time.
    let height = rig.stance_height;
    commands.spawn((
        PlayerCapsule,
        Transform::from_translation(rig.position + Vec3::Y * height * 0.5),
        RigidBody::Kinematic,
        CustomPositionIntegration,
        player_capsule(height),
        // Hands are on their own layers; the player's filter ignores them so the
        // palms cannot shove the body around while reaching.
        CollisionLayers::new(LayerMask::DEFAULT, LayerMask::DEFAULT),
        LinearVelocity::default(),
    ));
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

/// Crouch toggle (left stick click, C, Ctrl) and one-shot jump requests (B, Space).
fn stance_input(
    hands: Res<Hands>,
    keys: Res<ButtonInput<KeyCode>>,
    mut rig: ResMut<PlayerRig>,
) {
    let click = hands.left.stick_click;
    let click_edge = click && !rig.prev_stick_click;
    rig.prev_stick_click = click;
    let key_edge = keys.just_pressed(KeyCode::KeyC)
        || keys.just_pressed(KeyCode::ControlLeft)
        || keys.just_pressed(KeyCode::ControlRight);
    rig.crouched = toggle_crouch(rig.crouched, click_edge || key_edge);

    // Right Joy-Con face buttons are [X, B, Y, A]; B is south.
    let jump_button = hands.right.face[1] || keys.pressed(KeyCode::Space);
    if jump_button && !rig.prev_jump_button {
        rig.jump_queued = true;
    }
    rig.prev_jump_button = jump_button;
}

/// While a stand-up is pending, checks that the standing capsule fits. If something is
/// above (today there is nothing to crouch under; later, desks and windows), the crouch
/// is kept — with its lower eyes and slower walk — instead of popping up through it.
fn check_headroom(
    spatial_query: SpatialQuery,
    mut rig: ResMut<PlayerRig>,
    capsule: Query<Entity, With<PlayerCapsule>>,
) {
    if rig.crouched || rig.stance_height >= STAND_HEIGHT - 1e-4 {
        return;
    }
    let Ok(entity) = capsule.single() else {
        return;
    };
    // Where the standing capsule would be, minus a hair so the deck itself does not
    // count as an obstacle.
    let shape = player_capsule(STAND_HEIGHT - 0.06);
    let position = (rig.position + Vec3::Y * (STAND_HEIGHT * 0.5 + 0.02)).real();
    let filter = SpatialQueryFilter::from_mask(LayerMask::DEFAULT).with_excluded_entities([entity]);
    if !spatial_query
        .shape_intersections(&shape, position, Quat::IDENTITY, &filter)
        .is_empty()
    {
        rig.crouched = true;
    }
}

/// Grows or shrinks the capsule with the crouch state.
fn update_stance(
    mut rig: ResMut<PlayerRig>,
    mut capsule: Query<(&mut Collider, &mut Transform), With<PlayerCapsule>>,
) {
    let Ok((mut collider, mut transform)) = capsule.single_mut() else {
        return;
    };
    let target = if rig.crouched {
        CROUCH_HEIGHT
    } else {
        STAND_HEIGHT
    };
    if (target - rig.stance_height).abs() < 1e-4 {
        return;
    }

    rig.stance_height = target;
    *collider = player_capsule(target);
    transform.translation.y = rig.position.y + target * 0.5;
}

fn ease_eye_height(time: Res<Time>, mut rig: ResMut<PlayerRig>) {
    let target = target_eye_height(rig.pitch, rig.crouched);
    let blend = 1.0 - (-time.delta_secs() / EYE_TAU).exp();
    rig.eye_height += (target - rig.eye_height) * blend;
}

/// Stick / WASD movement, gravity, jump, and the collide-and-slide step.
fn move_player(
    time: Res<Time>,
    hands: Res<Hands>,
    keys: Res<ButtonInput<KeyCode>>,
    spatial_query: SpatialQuery,
    move_and_slide: MoveAndSlide,
    mut rig: ResMut<PlayerRig>,
    mut capsule: Query<
        (Entity, &mut Transform, &Collider, &mut LinearVelocity),
        With<PlayerCapsule>,
    >,
    mut prop_bodies: Query<(&RigidBody, Forces), Without<PlayerCapsule>>,
    collider_of: Query<&ColliderOf>,
    masses: Query<&ComputedMass>,
) {
    let Ok((entity, mut transform, collider, mut linear)) = capsule.single_mut() else {
        return;
    };
    let dt = time.delta_secs();
    let up = *transform.up();

    // Ground check from the current stance: a slightly slimmer capsule cast down.
    let cast = player_capsule(rig.stance_height - 0.02);
    let center = (rig.position + Vec3::Y * rig.stance_height * 0.5).real();
    let filter = SpatialQueryFilter::from_mask(LayerMask::DEFAULT)
        .with_excluded_entities([entity]);
    let ground_hit = spatial_query.cast_shape(
        &cast,
        center,
        Quat::IDENTITY,
        Dir3::NEG_Y,
        &ShapeCastConfig::from_max_distance(0.08),
        &filter,
    );
    let grounded = rig.velocity.y <= 0.05
        && ground_hit.is_some_and(|hit| up.angle_between(hit.normal1) <= MAX_SLOPE);
    rig.grounded = grounded;
    rig.coyote = if grounded {
        COYOTE_TIME
    } else {
        (rig.coyote - dt).max(0.0)
    };

    // --- Horizontal intent ---------------------------------------------------
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
    let speed = walk_speed(rig.crouched);
    let direction = Quat::from_rotation_y(rig.yaw) * Vec3::new(input.x, 0.0, -input.y);
    let target = direction * speed;

    // Quick acceleration, a touch slower when releasing: responsive but not snappy.
    let tau = if target.length_squared() > 0.01 {
        0.055
    } else {
        0.09
    };
    let blend = 1.0 - (-dt / tau).exp();
    let horizontal = Vec3::new(rig.velocity.x, 0.0, rig.velocity.z).lerp(target, blend);
    let horizontal = if horizontal.length_squared() > speed * speed {
        horizontal.normalize() * speed
    } else {
        horizontal
    };
    rig.velocity.x = horizontal.x;
    rig.velocity.z = horizontal.z;

    // --- Jump and gravity ----------------------------------------------------
    if rig.jump_queued {
        if jump_allowed(grounded, rig.coyote) {
            rig.velocity.y = jump_speed();
            rig.coyote = 0.0;
            rig.grounded = false;
        }
        rig.jump_queued = false;
    }
    rig.velocity.y = (rig.velocity.y - GRAVITY * dt).max(-TERMINAL_VELOCITY);

    // --- Collide and slide ---------------------------------------------------
    // Same structure as avian's `kinematic_character_3d` example: accept every hit,
    // kill the velocity component that drives into ground/ceiling, and collect the
    // contacts so the props we bump into get a shove.
    let mut touched_ground = false;
    let mut pushed: Vec<(Entity, Vec3, Vec3)> = Vec::new();

    let output = move_and_slide.move_and_slide(
        collider,
        transform.translation.real(),
        transform.rotation,
        rig.velocity,
        time.delta(),
        &MoveAndSlideConfig::default(),
        &filter,
        |hit| {
            let normal = **hit.normal;
            let angle = up.angle_between(normal);
            if angle <= MAX_SLOPE {
                touched_ground = true;
                if hit.velocity.y < 0.0 {
                    hit.velocity.y = 0.0;
                }
            } else if angle >= core::f32::consts::PI - MAX_SLOPE && hit.velocity.y > 0.0 {
                // Ceiling: stop rising instead of sticking to it.
                hit.velocity.y = 0.0;
            }
            pushed.push((hit.entity, normal, *hit.velocity));
            MoveAndSlideHitResponse::Accept
        },
    );

    transform.translation = output.position.f32();
    rig.velocity = output.projected_velocity;
    // Landing/ceiling contact already killed the vertical speed in the callback; keep
    // gravity from re-accumulating into the deck while we rest on it.
    if touched_ground && rig.velocity.y < 0.0 {
        rig.velocity.y = 0.0;
    }

    rig.position = transform.translation - Vec3::Y * rig.stance_height * 0.5;
    rig.position.x = rig.position.x.clamp(-WALK_LIMIT, WALK_LIMIT);
    rig.position.z = rig.position.z.clamp(-WALK_LIMIT, WALK_LIMIT);
    transform.translation = rig.position + Vec3::Y * rig.stance_height * 0.5;
    linear.0 = rig.velocity;

    // --- Shove the dynamic props we walk into, without being launched by them --
    for (collider_entity, normal, character_velocity) in pushed {
        let Ok(collider_of) = collider_of.get(collider_entity) else {
            continue;
        };
        let Ok((rigid_body, mut forces)) = prop_bodies.get_mut(collider_of.body) else {
            continue;
        };
        if !rigid_body.is_dynamic() {
            continue;
        }
        let Ok(mass) = masses.get(collider_of.body) else {
            continue;
        };
        // Closing speed along the contact normal; capped so a light prop leaves at
        // about walking pace instead of being shot across the terrace.
        let approach = -normal.dot(character_velocity - forces.linear_velocity());
        if approach <= 0.05 {
            continue;
        }
        let delta_v = (approach * 0.8).min(2.0);
        forces.apply_linear_impulse(-normal * delta_v * mass.value());
    }
}

fn write_player_transform(
    rig: Res<PlayerRig>,
    mut body: Single<&mut Transform, (With<PlayerBody>, Without<PlayerCamera>)>,
    mut camera: Single<&mut Transform, (With<PlayerCamera>, Without<PlayerBody>)>,
) {
    // The rig origin slides down with the eyes so the arms anchored at EYE_HEIGHT
    // in `arms.rs` follow the crouch without their own offset.
    body.translation = rig.position + Vec3::Y * (rig.eye_height - EYE_HEIGHT);
    body.rotation = Quat::from_rotation_y(rig.yaw);

    camera.translation = Vec3::new(0.0, EYE_HEIGHT, 0.0);
    camera.rotation = Quat::from_rotation_x(rig.pitch);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: f32, b: f32, eps: f32) {
        assert!((a - b).abs() < eps, "{a} != {b}");
    }

    #[test]
    fn eyes_stay_standing_until_the_look_down_threshold() {
        for pitch_deg in [0.0_f32, -10.0, -28.0, -35.0] {
            assert_close(
                auto_crouch_eye_height(pitch_deg.to_radians()),
                EYE_HEIGHT,
                1e-5,
            );
        }
        // And never rise above standing, however far up you look.
        assert_close(auto_crouch_eye_height(60.0_f32.to_radians()), EYE_HEIGHT, 1e-5);
    }

    #[test]
    fn eyes_reach_the_auto_crouch_floor_at_minus_eighty() {
        assert_close(
            auto_crouch_eye_height(-80.0_f32.to_radians()),
            AUTO_CROUCH_EYE_HEIGHT,
            1e-5,
        );
        assert_close(
            auto_crouch_eye_height(-85.0_f32.to_radians()),
            AUTO_CROUCH_EYE_HEIGHT,
            1e-5,
        );
    }

    #[test]
    fn eye_height_is_monotonic_as_the_camera_looks_down() {
        let mut previous = f32::INFINITY;
        for step in 0..=100 {
            let pitch = -(step as f32) * (85.0 / 100.0);
            let eye = auto_crouch_eye_height(pitch.to_radians());
            assert!(eye <= previous + 1e-6, "eye height rose at {pitch}°");
            previous = eye;
        }
        // And it actually moves through the band, not just at the endpoints.
        assert!(auto_crouch_eye_height(-50.0_f32.to_radians()) < EYE_HEIGHT - 0.05);
        assert!(auto_crouch_eye_height(-50.0_f32.to_radians()) > AUTO_CROUCH_EYE_HEIGHT);
    }

    #[test]
    fn manual_crouch_wins_when_it_is_lower_than_the_auto_crouch() {
        assert_close(target_eye_height(0.0, true), CROUCH_EYE_HEIGHT, 1e-5);
        assert_close(
            target_eye_height(-80.0_f32.to_radians(), true),
            AUTO_CROUCH_EYE_HEIGHT,
            1e-5,
        );
        assert_close(
            target_eye_height(-80.0_f32.to_radians(), false),
            AUTO_CROUCH_EYE_HEIGHT,
            1e-5,
        );
    }

    #[test]
    fn crouch_toggles_only_on_a_fresh_press() {
        assert!(toggle_crouch(false, true));
        assert!(!toggle_crouch(true, true));
        assert!(!toggle_crouch(false, false));
        assert!(toggle_crouch(true, false));
    }

    #[test]
    fn jump_needs_ground_or_the_coyote_window() {
        assert!(jump_allowed(true, 0.0));
        assert!(jump_allowed(false, 0.05));
        assert!(!jump_allowed(false, 0.0));
        assert!(!jump_allowed(false, -0.1));
    }

    #[test]
    fn crouching_slows_the_walk() {
        assert!(walk_speed(true) < walk_speed(false));
    }

    #[test]
    fn the_jump_launch_speed_gives_the_brief_apex() {
        // v² = 2 g h → the launch speed and gravity must agree with JUMP_HEIGHT.
        let apex = jump_speed() * jump_speed() / (2.0 * GRAVITY);
        assert_close(apex, JUMP_HEIGHT, 1e-4);
        // A ~0.63 s hop: quick enough to feel responsive on a flat screen.
        assert!(jump_speed() > 3.0 && jump_speed() < 5.0);
    }
}
