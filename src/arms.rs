//! M0 arms: a faithful port of Google Daydream's `GvrArmModel` (Apache-2.0),
//! plus the flat-screen presentation (cylinders + gloves) and the kinematic
//! hand colliders that push the physics props.
//!
//! `GvrArmModel` source: googlevr/gvr-unity-sdk, `GvrArmModel.cs`.
//! Unity is left-handed (+Z forward), Bevy is right-handed (−Z forward):
//! positions convert as `(x, y, -z)` and quaternions as `(x, y, z, w) → (-x, -y, z, w)`.
//! The left arm mirrors X (`handedMultiplier = (-1, 1, 1)`), exactly like the original.
//!
//! The model is solved in **body space** (torso yaw only, never camera pitch).
//! `solve_arm` is pure and unit tested; the ECS systems only apply its output.

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::joycon::{HandInput, Hands};
use crate::player::{EYE_HEIGHT, PlayerBody, PlayerSet};
use crate::theme;

// --- GvrArmModel constants (converted to Bevy space) -------------------------
const ELBOW_BEND_RATIO: f32 = 0.6;
const EXTENSION_WEIGHT: f32 = 0.4;
const MIN_EXTENSION_ANGLE: f32 = 7.0;
const MAX_EXTENSION_ANGLE: f32 = 60.0;

/// Shoulder relative to the head. Unity `(0.17, -0.2, -0.03)`.
const SHOULDER: Vec3 = Vec3::new(0.17, -0.2, 0.03);
/// Elbow rest relative to the head. Unity `(0.195, -0.5, 0.005)`.
const ELBOW_REST: Vec3 = Vec3::new(0.195, -0.5, -0.005);
/// Wrist relative to the elbow. Unity `(0, 0, 0.25)`.
const WRIST_REST: Vec3 = Vec3::new(0.0, 0.0, -0.25);
/// Controller (palm) relative to the wrist. Unity `(0, 0, 0.05)`.
const CONTROLLER_REST: Vec3 = Vec3::new(0.0, 0.0, -0.05);
/// Elbow offset at full extension. Unity `(-0.13, 0.14, 0.08)`.
const ARM_EXTENSION_OFFSET: Vec3 = Vec3::new(-0.13, 0.14, -0.08);

/// Flat-screen adaptation: the Daydream model puts the hands at chest height,
/// which falls just below an 80° FOV. Lift the whole chain a little so the
/// forearms and gloves read on a monitor. The model itself is untouched.
/// Kept nearly in the eye plane: pushing the chain forward would put the
/// shoulders in front of the camera, which looks enormous and wrong.
pub const ARM_ANCHOR_OFFSET: Vec3 = Vec3::new(0.0, 0.07, -0.04);

const UPPER_RADIUS: f32 = 0.052;
const FOREARM_RADIUS: f32 = 0.036;
const ELBOW_RADIUS: f32 = 0.052;
const WRIST_RADIUS: f32 = 0.033;
/// Total travel of the finger curl, in radians (≈ 83°).
const FINGER_CURL: f32 = 1.45;

/// Which physical hand a chain belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandSide {
    Left,
    Right,
}

impl HandSide {
    pub fn index(self) -> usize {
        match self {
            HandSide::Left => 0,
            HandSide::Right => 1,
        }
    }

    pub fn sign_x(self) -> f32 {
        match self {
            HandSide::Left => -1.0,
            HandSide::Right => 1.0,
        }
    }

    pub fn color(self) -> Color {
        match self {
            HandSide::Left => theme::JOY_LEFT,
            HandSide::Right => theme::JOY_RIGHT,
        }
    }

    fn input<'a>(self, hands: &'a Hands) -> &'a HandInput {
        match self {
            HandSide::Left => &hands.left,
            HandSide::Right => &hands.right,
        }
    }
}

/// The solved arm chain in body space, relative to the eye (the neck is at 0).
#[derive(Clone, Copy, Debug)]
pub struct ArmPose {
    pub shoulder: Vec3,
    pub elbow: Vec3,
    pub wrist: Vec3,
    /// Palm / controller position.
    pub hand: Vec3,
    pub elbow_rotation: Quat,
    pub wrist_rotation: Quat,
    /// Full controller orientation (includes roll).
    pub hand_rotation: Quat,
}

/// Port of `GvrArmModel.ApplyArmModel` / `GetControllerRotation`.
///
/// `orientation` is the Joy-Con orientation in body space, identity = pointing
/// at the screen with the stick face up. The torso rotation is identity because
/// this is solved in the torso frame (the arm root follows body yaw).
pub fn solve_arm(side: HandSide, orientation: Quat) -> ArmPose {
    let handed = Vec3::new(side.sign_x(), 1.0, 1.0);

    // --- GetControllerRotation -------------------------------------------------
    // Controller forward in Bevy space is -Z (Unity +Z).
    let controller_forward = orientation * Vec3::NEG_Z;
    // Unity: 90 - angle(forward, up). Same geometry with Bevy's -Z forward.
    let x_angle = 90.0 - controller_forward.angle_between(Vec3::Y).to_degrees();
    // Remove the roll: the shortest arc that takes forward onto the controller.
    let xy_rotation = Quat::from_rotation_arc(Vec3::NEG_Z, controller_forward);

    // --- SetUntransformedJointPositions + ApplyExtensionOffset -----------------
    let extension_ratio = ((x_angle - MIN_EXTENSION_ANGLE)
        / (MAX_EXTENSION_ANGLE - MIN_EXTENSION_ANGLE))
        .clamp(0.0, 1.0);
    let elbow = ELBOW_REST * handed + ARM_EXTENSION_OFFSET * handed * extension_ratio;
    let wrist_local = WRIST_REST * handed;
    let controller_local = CONTROLLER_REST * handed;

    // --- CalculateLerpRotation -------------------------------------------------
    let total_angle = xy_rotation.angle_between(Quat::IDENTITY).to_degrees();
    let suppression = 1.0 - (total_angle / 180.0).powi(6);
    let lerp_value = ((1.0 - ELBOW_BEND_RATIO)
        + ELBOW_BEND_RATIO * extension_ratio * EXTENSION_WEIGHT)
        * suppression;
    let lerp_rotation = Quat::IDENTITY.slerp(xy_rotation, lerp_value.clamp(0.0, 1.0));

    // --- CalculateFinalJointRotations -----------------------------------------
    // torsoRotation is identity in this frame.
    let elbow_rotation = lerp_rotation.inverse() * xy_rotation;
    let wrist_rotation = elbow_rotation * lerp_rotation;

    // --- ApplyRotationToJoints -------------------------------------------------
    let wrist = elbow + elbow_rotation * wrist_local;
    let hand = wrist + wrist_rotation * controller_local;

    ArmPose {
        shoulder: SHOULDER * handed,
        elbow,
        wrist,
        hand,
        elbow_rotation,
        wrist_rotation,
        hand_rotation: orientation,
    }
}

// --- ECS ---------------------------------------------------------------------

pub struct ArmsPlugin;

impl Plugin for ArmsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_arms.after(crate::player::spawn_player))
            .add_systems(Update, update_arms.after(PlayerSet))
            .add_systems(FixedUpdate, drive_hand_bodies);
    }
}

#[derive(Component)]
struct ArmRoot {
    side: HandSide,
}

/// Smoothed trigger state, one per hand.
#[derive(Component, Default)]
struct ArmAnim {
    curl: f32,
}

#[derive(Component, Clone, Copy)]
enum ArmPart {
    Upper(HandSide),
    Elbow(HandSide),
    Fore(HandSide),
    Wrist(HandSide),
    /// Pivot at the palm centre: position, orientation and near-camera fade.
    Palm(HandSide),
    /// Rounded palm mesh; constant scale so the fingers are not distorted.
    PalmMesh,
    Finger {
        side: HandSide,
        base: Quat,
        amount: f32,
    },
}

/// A sleeve mesh that swaps to a desaturated material while its Joy-Con is off.
#[derive(Component)]
struct SleevePart {
    side: HandSide,
    normal: Handle<StandardMaterial>,
    muted: Handle<StandardMaterial>,
}

#[derive(Component)]
struct HandBody {
    side: HandSide,
}

#[derive(Clone, Copy, Default)]
struct HandState {
    pose: Option<ArmPose>,
    curl: f32,
    connected: bool,
}

fn sleeve_material(color: Color) -> StandardMaterial {
    StandardMaterial {
        base_color: color,
        perceptual_roughness: 0.6,
        reflectance: 0.18,
        ..default()
    }
}

fn setup_arms(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    body: Single<Entity, With<PlayerBody>>,
) {
    let cylinder = meshes.add(Cylinder::new(0.5, 1.0).mesh().resolution(24));
    let ball = meshes.add(Sphere::new(1.0).mesh().uv(24, 16));
    let finger_mesh = meshes.add(Capsule3d::new(0.0095, 0.062));
    let thumb_mesh = meshes.add(Capsule3d::new(0.0110, 0.050));

    let glove = materials.add(StandardMaterial {
        base_color: theme::GLOVE,
        perceptual_roughness: 0.72,
        reflectance: 0.2,
        ..default()
    });

    let sleeve_handles: [(Handle<StandardMaterial>, Handle<StandardMaterial>); 2] = [
        (
            materials.add(sleeve_material(theme::JOY_LEFT)),
            materials.add(sleeve_material(theme::desaturate(theme::JOY_LEFT, 0.5))),
        ),
        (
            materials.add(sleeve_material(theme::JOY_RIGHT)),
            materials.add(sleeve_material(theme::desaturate(theme::JOY_RIGHT, 0.5))),
        ),
    ];

    let root_transform = Transform::from_translation(Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET);

    commands.entity(*body).with_children(|parent| {
        for side in [HandSide::Left, HandSide::Right] {
            let (sleeve, muted) = sleeve_handles[side.index()].clone();
            let sign = side.sign_x();

            parent
                .spawn((
                    ArmRoot { side },
                    ArmAnim::default(),
                    root_transform,
                    Visibility::default(),
                ))
                .with_children(|arm| {
                    // Upper arm sleeve (shoulder → elbow).
                    arm.spawn((
                        ArmPart::Upper(side),
                        Mesh3d(cylinder.clone()),
                        MeshMaterial3d(sleeve.clone()),
                        SleevePart {
                            side,
                            normal: sleeve.clone(),
                            muted: muted.clone(),
                        },
                        Transform::default(),
                    ));
                    // Elbow ball hides the joint seam.
                    arm.spawn((
                        ArmPart::Elbow(side),
                        Mesh3d(ball.clone()),
                        MeshMaterial3d(sleeve.clone()),
                        SleevePart {
                            side,
                            normal: sleeve.clone(),
                            muted: muted.clone(),
                        },
                        Transform::default(),
                    ));
                    // Forearm sleeve, slightly thinner than the upper arm.
                    arm.spawn((
                        ArmPart::Fore(side),
                        Mesh3d(cylinder.clone()),
                        MeshMaterial3d(sleeve.clone()),
                        SleevePart {
                            side,
                            normal: sleeve.clone(),
                            muted: muted.clone(),
                        },
                        Transform::default(),
                    ));
                    // Wrist cuff.
                    arm.spawn((
                        ArmPart::Wrist(side),
                        Mesh3d(ball.clone()),
                        MeshMaterial3d(sleeve.clone()),
                        SleevePart {
                            side,
                            normal: sleeve,
                            muted,
                        },
                        Transform::default(),
                    ));

                    // Glove: rounded palm + four fingers + thumb.
                    arm.spawn((
                        ArmPart::Palm(side),
                        Transform::default(),
                        Visibility::default(),
                    ))
                    .with_children(|palm| {
                        palm.spawn((
                            ArmPart::PalmMesh,
                            Mesh3d(ball.clone()),
                            MeshMaterial3d(glove.clone()),
                            Transform::from_scale(Vec3::new(0.043, 0.019, 0.048)),
                        ));
                        for (i, x) in [-0.030_f32, -0.010, 0.010, 0.030].into_iter().enumerate() {
                            palm.spawn((
                                ArmPart::Finger {
                                    side,
                                    base: Quat::IDENTITY,
                                    amount: 1.0 - i as f32 * 0.06,
                                },
                                Transform::from_xyz(x, 0.0, -0.040),
                                Visibility::default(),
                            ))
                            .with_child((
                                Mesh3d(finger_mesh.clone()),
                                MeshMaterial3d(glove.clone()),
                                Transform::from_xyz(0.0, 0.0, -0.0405)
                                    .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
                            ));
                        }
                        // Thumb sits on the inner side of the hand.
                        palm.spawn((
                            ArmPart::Finger {
                                side,
                                base: Quat::from_rotation_y(-sign * 0.55)
                                    * Quat::from_rotation_z(sign * 0.35),
                                amount: 0.7,
                            },
                            Transform::from_xyz(-0.042 * sign, 0.0, -0.012),
                            Visibility::default(),
                        ))
                        .with_child((
                            Mesh3d(thumb_mesh.clone()),
                            MeshMaterial3d(glove.clone()),
                            Transform::from_xyz(0.0, 0.0, -0.0365)
                                .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
                        ));
                    });
                });
        }
    });

    // Kinematic physical hands live at the root of the world (their Position is
    // global), independent of the visual chain.
    for side in [HandSide::Left, HandSide::Right] {
        let rest = solve_arm(side, Quat::IDENTITY);
        let position = Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + rest.hand;
        commands.spawn((
            HandBody { side },
            Transform::from_translation(position).with_rotation(rest.hand_rotation),
            Position(position),
            Rotation(rest.hand_rotation),
            RigidBody::Kinematic,
            Collider::cuboid(0.10, 0.045, 0.12),
            Friction::new(0.9),
            Restitution::new(0.0),
            SleepingDisabled,
            LinearVelocity::default(),
            AngularVelocity::default(),
        ));
    }
}

fn update_arms(
    time: Res<Time>,
    hands: Res<Hands>,
    mut roots: Query<(&ArmRoot, &mut ArmAnim)>,
    mut parts: Query<(&ArmPart, &mut Transform)>,
    mut sleeves: Query<(&SleevePart, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let dt = time.delta_secs();
    let mut states = [HandState::default(); 2];

    for (root, mut anim) in &mut roots {
        let input = root.side.input(&hands);
        let target = if input.connected && input.trigger {
            1.0
        } else {
            0.0
        };
        // Close in ~80 ms, open in ~120 ms, exponential ease-out (DESIGN §7).
        let tau = if target > anim.curl { 0.08 } else { 0.12 };
        anim.curl += (target - anim.curl) * (1.0 - (-dt / tau).exp());

        let orientation = if input.connected {
            input.orientation
        } else {
            Quat::IDENTITY
        };
        states[root.side.index()] = HandState {
            pose: Some(solve_arm(root.side, orientation)),
            curl: anim.curl,
            connected: input.connected,
        };
    }

    for (part, mut transform) in &mut parts {
        match *part {
            ArmPart::Upper(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                segment(&mut transform, pose.shoulder, pose.elbow, UPPER_RADIUS);
            }
            ArmPart::Fore(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                segment(&mut transform, pose.elbow, pose.wrist, FOREARM_RADIUS);
            }
            ArmPart::Elbow(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                transform.translation = pose.elbow;
                transform.rotation = pose.elbow_rotation;
                transform.scale = Vec3::splat(ELBOW_RADIUS);
            }
            ArmPart::Wrist(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                transform.translation = pose.wrist;
                transform.rotation = pose.wrist_rotation;
                transform.scale = Vec3::splat(WRIST_RADIUS);
            }
            ArmPart::Palm(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                // Fade the glove out when it gets too close to the eye instead of
                // clipping through the camera (DESIGN §7, "cerca de la cámara").
                let distance = (ARM_ANCHOR_OFFSET + pose.hand).length();
                let fade = ((distance - 0.16) / (0.30 - 0.16)).clamp(0.0, 1.0);
                let fade = fade * fade * (3.0 - 2.0 * fade);

                transform.translation = pose.hand;
                transform.rotation = pose.hand_rotation;
                // Scaling the pivot fades the whole glove (mesh + fingers) without
                // ever distorting the finger capsules at normal distance.
                transform.scale = Vec3::splat(fade);
            }
            ArmPart::PalmMesh => {}
            ArmPart::Finger {
                side,
                base,
                amount,
            } => {
                let curl = states[side.index()].curl;
                transform.rotation = base * Quat::from_rotation_x(-FINGER_CURL * amount * curl);
            }
        }
    }

    for (sleeve, mut material) in &mut sleeves {
        let connected = states[sleeve.side.index()].connected;
        let wanted = if connected {
            &sleeve.normal
        } else {
            &sleeve.muted
        };
        if material.0 != *wanted {
            material.0 = wanted.clone();
        }
    }
}

/// Place a unit-height cylinder (radius 0.5, height 1) between two points.
fn segment(transform: &mut Transform, from: Vec3, to: Vec3, radius: f32) {
    let delta = to - from;
    let length = delta.length();
    if length < 1e-5 {
        transform.scale = Vec3::ZERO;
        return;
    }
    transform.translation = (from + to) * 0.5;
    transform.rotation = Quat::from_rotation_arc(Vec3::Y, delta / length);
    transform.scale = Vec3::new(radius * 2.0, length, radius * 2.0);
}

fn drive_hand_bodies(
    time: Res<Time<Fixed>>,
    hands: Res<Hands>,
    fake: Option<Res<crate::FakeHands>>,
    body: Single<&Transform, With<PlayerBody>>,
    mut hands_query: Query<(
        &HandBody,
        &Position,
        &Rotation,
        &mut LinearVelocity,
        &mut AngularVelocity,
    )>,
) {
    let dt = time.delta_secs().max(1e-5);
    // The fixed step runs before `Update`, so in fake mode it must read the
    // mirrored snapshot instead of the device state.
    let hands: &Hands = match fake.as_ref() {
        Some(fake) => &fake.0,
        None => &hands,
    };

    for (hand, position, rotation, mut linear, mut angular) in &mut hands_query {
        let input = hand.side.input(&hands);
        let orientation = if input.connected {
            input.orientation
        } else {
            Quat::IDENTITY
        };
        let pose = solve_arm(hand.side, orientation);

        let target_position =
            body.transform_point(Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + pose.hand);
        let target_rotation = body.rotation * pose.hand_rotation;

        // Drive by velocity, never by teleporting: props receive real momentum.
        let velocity = ((target_position - position.0) / dt).clamp_length_max(10.0);
        linear.0 = velocity;

        let mut delta = target_rotation * rotation.0.inverse();
        if delta.w < 0.0 {
            delta = -delta;
        }
        let (axis, angle) = delta.to_axis_angle();
        angular.0 = if angle > 1e-5 {
            axis * (angle / dt).min(30.0)
        } else {
            Vec3::ZERO
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: f32, b: f32, eps: f32) {
        assert!((a - b).abs() < eps, "{a} != {b}");
    }

    #[test]
    fn identity_hand_is_ahead_and_below_the_shoulder() {
        let pose = solve_arm(HandSide::Right, Quat::IDENTITY);
        assert!(
            pose.hand.z < -0.2,
            "hand should be in front of the eye, got {:?}",
            pose.hand
        );
        assert!(
            pose.hand.y < pose.shoulder.y,
            "hand should hang below the shoulder, got hand {:?} shoulder {:?}",
            pose.hand,
            pose.shoulder
        );
        // The well-known Daydream neutral pose: forward ≈ 0.305 m, down ≈ 0.50 m.
        assert_close(pose.hand.z, -0.305, 0.01);
        assert_close(pose.hand.y, -0.50, 0.01);
    }

    #[test]
    fn pitching_up_lifts_and_extends_the_hand() {
        let neutral = solve_arm(HandSide::Right, Quat::IDENTITY);
        let up = solve_arm(
            HandSide::Right,
            Quat::from_rotation_x(60.0_f32.to_radians()),
        );
        assert!(
            up.hand.y > neutral.hand.y + 0.15,
            "pitch up should raise the hand: {:?} vs {:?}",
            up.hand,
            neutral.hand
        );
        assert!(
            up.hand.z < neutral.hand.z - 0.02,
            "pitch up should extend the arm: {:?} vs {:?}",
            up.hand,
            neutral.hand
        );
    }

    #[test]
    fn left_arm_mirrors_right() {
        let right = solve_arm(HandSide::Right, Quat::IDENTITY);
        let left = solve_arm(HandSide::Left, Quat::IDENTITY);
        assert_close(left.hand.x, -right.hand.x, 1e-5);
        assert_close(left.hand.y, right.hand.y, 1e-5);
        assert_close(left.hand.z, right.hand.z, 1e-5);
        assert_close(left.elbow.x, -right.elbow.x, 1e-5);
        assert_close(left.shoulder.x, -right.shoulder.x, 1e-5);
    }

    #[test]
    fn extension_ratio_is_clamped() {
        let down = solve_arm(
            HandSide::Right,
            Quat::from_rotation_x(-80.0_f32.to_radians()),
        );
        let up = solve_arm(
            HandSide::Right,
            Quat::from_rotation_x(85.0_f32.to_radians()),
        );
        assert_close(down.elbow.y, -0.50, 1e-5);
        assert_close(up.elbow.y, -0.36, 0.001);
        assert_close(up.elbow.z, -0.085, 0.001);
    }

    /// Body-space hand position (feet at y = 0).
    fn hand_world_space(pose: ArmPose) -> Vec3 {
        Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + pose.hand
    }

    #[test]
    fn neutral_hand_hovers_over_the_bench_front_half() {
        use crate::world::{
            BENCH_BACK_Z, BENCH_FRONT_Z, BENCH_HALF_WIDTH, BENCH_TOP_Y,
        };

        let hand = hand_world_space(solve_arm(HandSide::Right, Quat::IDENTITY));
        assert!(
            hand.z < BENCH_FRONT_Z,
            "neutral hand must be past the front edge: {hand:?}"
        );
        assert!(
            hand.z > BENCH_BACK_Z,
            "neutral hand must not overshoot the bench: {hand:?}"
        );
        assert!(hand.x.abs() < BENCH_HALF_WIDTH);

        let clearance = hand.y - BENCH_TOP_Y;
        assert!(
            (0.08..0.30).contains(&clearance),
            "neutral hand should float 8–30 cm above the top, got {clearance} m"
        );
    }

    #[test]
    fn pitching_down_puts_the_palm_within_six_centimetres_of_the_bench() {
        use crate::world::BENCH_TOP_Y;

        let pose = solve_arm(
            HandSide::Right,
            Quat::from_rotation_x(-45.0_f32.to_radians()),
        );
        let hand = hand_world_space(pose);
        // The palm ellipsoid is 4 cm thick, so its underside is 2 cm below centre.
        let palm_bottom = hand.y - 0.020;
        let clearance = palm_bottom - BENCH_TOP_Y;
        assert!(
            (0.0..0.06).contains(&clearance),
            "pitching -45° should bring the palm to within ~6 cm of the top, got {clearance} m"
        );
    }
}
