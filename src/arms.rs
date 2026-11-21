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
use bevy::world_serialization::WorldInstanceReady;

use crate::joycon::{HandInput, Hands};
use crate::player::{EYE_HEIGHT, PlayerBody, PlayerRig, PlayerSet};
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

/// Torso lean: looking down past this angle starts moving the arm roots forward and
/// down, so the hands reach where the camera looks (DESIGN §7, M1 reach).
pub const LEAN_START: f32 = 20.0_f32.to_radians();
pub const LEAN_RANGE: f32 = 40.0_f32.to_radians();
/// Offset of the whole arm chain at full lean (forward is −Z).
pub const LEAN_OFFSET: Vec3 = Vec3::new(0.0, -0.35, -0.25);
/// Smoothing time constant for the lean.
pub const LEAN_TAU: f32 = 0.1;

/// Collision layer bits for the hands, used by grab to stop a held prop colliding
/// with the hand that holds it.
pub const HAND_LAYER_LEFT: u32 = 1 << 1;
pub const HAND_LAYER_RIGHT: u32 = 1 << 2;

const UPPER_RADIUS: f32 = 0.052;
const FOREARM_RADIUS: f32 = 0.036;
const ELBOW_RADIUS: f32 = 0.052;
const WRIST_RADIUS: f32 = 0.033;

/// Upper arm is hidden while its shoulder is this close to the eye: at 80° FOV a 10 cm
/// sleeve 20 cm from the camera covers a whole bottom corner of the frame (M2a fix).
const UPPER_NEAR_HIDE: f32 = 0.30;
/// Fully visible once the upper arm clears this distance from the eye.
const UPPER_NEAR_SHOW: f32 = 0.52;

/// Defensive clamp for the reach offset fed into the IK (the input already clamps it).
const REACH_IK_MAX: f32 = 0.35;

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

/// How far the torso leans for a camera pitch (radians, negative = looking down).
/// 0 while looking level or up; grows linearly from −20° to full lean at −60°.
pub fn lean_ratio(pitch: f32) -> f32 {
    ((-pitch - LEAN_START) / LEAN_RANGE).clamp(0.0, 1.0)
}

/// Analytic two-bone IK from the shoulder to a desired wrist position.
///
/// Returns `(wrist, elbow)`. The wrist is clamped to the real segment lengths (the arm can
/// neither stretch nor collapse), and the elbow bends in the plane of the pole hint, which
/// points down and outward so elbows hang like real ones instead of poking into the chest.
fn two_bone_ik(side: HandSide, shoulder: Vec3, target: Vec3) -> (Vec3, Vec3) {
    // GvrArmModel segment lengths, measured from the model itself (metres).
    let upper = (ELBOW_REST - SHOULDER).length();
    let fore = WRIST_REST.length();
    let max_reach = upper + fore - 1e-3;
    let min_reach = (upper - fore).abs() + 1e-3;

    let delta = target - shoulder;
    let distance = delta.length();
    let direction = if distance > 1e-5 {
        delta / distance
    } else {
        Vec3::NEG_Z
    };
    let distance = distance.clamp(min_reach, max_reach);
    let wrist = shoulder + direction * distance;

    // Law of cosines: distance from the shoulder to the elbow's projection on the bone.
    let along = (upper * upper - fore * fore + distance * distance) / (2.0 * distance);
    let height = (upper * upper - along * along).max(0.0).sqrt();

    let hint = Vec3::new(0.55 * side.sign_x(), -1.0, 0.20);
    let mut pole = hint - direction * hint.dot(direction);
    if pole.length_squared() < 1e-6 {
        // Straight up/down target: fall back to the body's outward axis.
        pole = Vec3::X * side.sign_x();
        pole -= direction * pole.dot(direction);
    }
    let pole = pole.normalize_or_zero();
    let elbow = shoulder + direction * along + pole * height;

    (wrist, elbow)
}

/// [`solve_arm`] plus the IMU reach offset: the wrist is displaced and the elbow re-solved
/// with two-bone IK, so a real push/pull moves the whole arm instead of only the hand.
pub fn solve_arm_with_reach(side: HandSide, orientation: Quat, reach: Vec3) -> ArmPose {
    let base = solve_arm(side, orientation);
    let reach = reach.clamp_length_max(REACH_IK_MAX);
    if reach.length_squared() < 1e-8 {
        return base;
    }

    let handed = Vec3::new(side.sign_x(), 1.0, 1.0);
    let (wrist, elbow) = two_bone_ik(side, base.shoulder, base.wrist + reach);
    let hand = wrist + base.hand_rotation * (CONTROLLER_REST * handed);

    let rest_upper = (ELBOW_REST - SHOULDER).normalize();
    let rest_fore = WRIST_REST.normalize();
    let elbow_rotation = Quat::from_rotation_arc(rest_upper, (elbow - base.shoulder).normalize());
    let wrist_rotation = Quat::from_rotation_arc(rest_fore, (wrist - elbow).normalize());

    ArmPose {
        shoulder: base.shoulder,
        elbow,
        wrist,
        hand,
        elbow_rotation,
        wrist_rotation,
        hand_rotation: base.hand_rotation,
    }
}

/// Body-space offset of the arm-chain anchor for a camera pitch.
pub fn lean_offset(pitch: f32) -> Vec3 {
    LEAN_OFFSET * lean_ratio(pitch)
}

/// Smoothed lean, shared by the visual arms (`Update`) and the hand bodies (`FixedUpdate`).
#[derive(Resource, Default)]
pub struct ArmLean {
    pub offset: Vec3,
}

// --- ECS ---------------------------------------------------------------------

pub struct ArmsPlugin;

impl Plugin for ArmsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ArmLean>()
            .init_resource::<HandMotion>()
            .init_resource::<HandCurl>()
            .add_systems(Startup, setup_arms.after(crate::player::spawn_player))
            .add_systems(
                Update,
                (update_lean, update_arms, pose_hand_models)
                    .chain()
                    .after(PlayerSet),
            )
            .add_systems(FixedUpdate, drive_hand_bodies.in_set(HandBodiesSet));
    }
}

/// Hand bodies are driven before anything that follows the hands (grab).
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct HandBodiesSet;

/// Latest hand velocities (from the fixed step), for throw and impact rumble.
#[derive(Resource, Default)]
pub struct HandMotion {
    pub velocity: [Vec3; 2],
    pub angular_velocity: [Vec3; 2],
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
}

/// The matte glove material, also used to override the hand models' grey material.
#[derive(Resource, Clone)]
struct GloveMaterial(Handle<StandardMaterial>);

/// Root of a skinned hand scene. The fingers are posed by [`pose_hand_models`].
#[derive(Component)]
struct HandModel {
    side: HandSide,
}

/// One posed joint of a finger, with its bind pose (Armature space) captured on load.
struct Joint {
    entity: Entity,
    bind: Transform,
    /// Relaxed-hand rotation about local X, radians.
    base_angle: f32,
    /// Extra rotation about local X at a full fist, radians.
    max_angle: f32,
}

/// One finger: metacarpal → proximal → intermediate → distal → tip as present.
struct FingerChain {
    joints: Vec<Joint>,
}

#[derive(Component)]
struct HandRig {
    side: HandSide,
    fingers: Vec<FingerChain>,
}

/// Smoothed finger curl per hand (0 = open, 1 = fist), set by `update_arms`.
#[derive(Resource, Default)]
pub struct HandCurl {
    pub value: [f32; 2],
}

/// A sleeve mesh that swaps to a desaturated material while its Joy-Con is off.
#[derive(Component)]
struct SleevePart {
    side: HandSide,
    normal: Handle<StandardMaterial>,
    muted: Handle<StandardMaterial>,
}

/// A hand's kinematic collider / palm point. Public so grab can query the palms.
#[derive(Component)]
pub struct HandBody {
    pub side: HandSide,
}

#[derive(Clone, Copy, Default)]
struct HandState {
    pose: Option<ArmPose>,
    connected: bool,
    /// How visible the upper arm is (0 = shoulder next to the camera, 1 = clear of it).
    upper_fade: f32,
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
    asset_server: Res<AssetServer>,
    body: Single<Entity, With<PlayerBody>>,
) {
    let cylinder = meshes.add(Cylinder::new(0.5, 1.0).mesh().resolution(24));
    let ball = meshes.add(Sphere::new(1.0).mesh().uv(24, 16));

    let glove = materials.add(StandardMaterial {
        base_color: theme::GLOVE,
        perceptual_roughness: 0.72,
        reflectance: 0.2,
        ..default()
    });
    commands.insert_resource(GloveMaterial(glove.clone()));

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

                    // Glove: a skinned WebXR generic-hand model (W3C, see
                    // assets/models/hands/LICENSE.md). Aligned and posed once loaded.
                    arm.spawn((
                        ArmPart::Palm(side),
                        Transform::default(),
                        Visibility::default(),
                    ))
                    .with_children(|palm| {
                        let path = match side {
                            HandSide::Left => "models/hands/left.glb",
                            HandSide::Right => "models/hands/right.glb",
                        };
                        palm.spawn((
                            HandModel { side },
                            Transform::default(),
                            Visibility::default(),
                            WorldAssetRoot(
                                asset_server.load(GltfAssetLabel::Scene(0).from_asset(path)),
                            ),
                        ))
                        .observe(hand_model_ready);
                    });
                });
        }
    });

    // Kinematic physical hands live at the root of the world (their Position is
    // global), independent of the visual chain.
    for side in [HandSide::Left, HandSide::Right] {
        let rest = solve_arm(side, Quat::IDENTITY);
        let position = Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + rest.hand;
        let layer = match side {
            HandSide::Left => HAND_LAYER_LEFT,
            HandSide::Right => HAND_LAYER_RIGHT,
        };
        commands.spawn((
            HandBody { side },
            // M2d: the palm ray every hand always carries (grab now, M3 pointer later).
            crate::handray::HandRay::default(),
            Transform::from_translation(position).with_rotation(rest.hand_rotation),
            Position(position),
            Rotation(rest.hand_rotation),
            RigidBody::Kinematic,
            Collider::cuboid(0.10, 0.045, 0.12),
            // Own layer so a held prop can ignore exactly this hand.
            CollisionLayers::new(LayerMask(layer), LayerMask::ALL),
            // Collision-start events feed the impact rumble.
            CollisionEventsEnabled,
            Friction::new(0.9),
            Restitution::new(0.0),
            SleepingDisabled,
            LinearVelocity::default(),
            AngularVelocity::default(),
        ));
    }
}

fn update_lean(
    time: Res<Time>,
    rig: Res<PlayerRig>,
    mut lean: ResMut<ArmLean>,
    mut roots: Query<&mut Transform, With<ArmRoot>>,
) {
    let target = lean_offset(rig.pitch);
    let blend = 1.0 - (-time.delta_secs() / LEAN_TAU).exp();
    lean.offset = lean.offset.lerp(target, blend);

    // Torso lean moves the whole chain (shoulders included) with the camera pitch.
    let translation = Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + lean.offset;
    for mut transform in &mut roots {
        transform.translation = translation;
    }
}

/// How visible the upper arm should be: hidden while its shoulder is right next to the camera,
/// full strength once it clears [`UPPER_NEAR_SHOW`]. Smoothstepped.
///
/// `lean` is the current torso lean: the arm root sits at `eye + ARM_ANCHOR_OFFSET + lean`, so
/// the camera is at `-(ARM_ANCHOR_OFFSET + lean)` in the pose's space.
fn upper_fade(pose: &ArmPose, lean: Vec3) -> f32 {
    let eye = -(ARM_ANCHOR_OFFSET + lean);
    let clearance = point_segment_distance(eye, pose.shoulder, pose.elbow);
    let t = ((clearance - UPPER_NEAR_HIDE) / (UPPER_NEAR_SHOW - UPPER_NEAR_HIDE)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn update_arms(
    time: Res<Time>,
    hands: Res<Hands>,
    lean: Res<ArmLean>,
    mut curl: ResMut<HandCurl>,
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
        curl.value[root.side.index()] = anim.curl;

        let orientation = if input.connected {
            input.orientation
        } else {
            Quat::IDENTITY
        };
        let reach = if input.connected && crate::joycon::reach_enabled() {
            input.reach
        } else {
            Vec3::ZERO
        };
        let pose = solve_arm_with_reach(root.side, orientation, reach);
        states[root.side.index()] = HandState {
            pose: Some(pose),
            connected: input.connected,
            upper_fade: upper_fade(&pose, lean.offset),
        };
    }

    for (part, mut transform) in &mut parts {
        match *part {
            ArmPart::Upper(side) => {
                let Some(pose) = states[side.index()].pose else {
                    continue;
                };
                // The shoulder sits right next to the camera (0.2 m at rest), where a 10 cm
                // sleeve is a huge blob in the bottom corner. Fade it out while it is that
                // close: the elbow and forearm already read as coming from below the frame,
                // and leaning forward / reaching (when the shoulder clears ~0.5 m) brings it
                // back.
                let fade = states[side.index()].upper_fade;
                if fade <= 1e-4 {
                    transform.scale = Vec3::ZERO;
                } else {
                    segment(&mut transform, pose.shoulder, pose.elbow, UPPER_RADIUS * fade);
                    transform.scale.y *= fade;
                }
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
                // A big joint ball only makes sense when the upper arm is there to hide the
                // seam. With the upper arm hidden, match the forearm radius so the elbow is not
                // a red/blue blob bigger than the arm itself.
                let fade = states[side.index()].upper_fade;
                let radius = FOREARM_RADIUS + (ELBOW_RADIUS - FOREARM_RADIUS) * fade;
                transform.translation = pose.elbow;
                transform.rotation = pose.elbow_rotation;
                transform.scale = Vec3::splat(radius);
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
                // Scaling the pivot fades the whole glove (scene + fingers) without
                // ever distorting the model at normal distance.
                transform.scale = Vec3::splat(fade);
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

/// The glTF wrist frame is aligned to the palm pivot; the palm centre sits this far
/// in front of the wrist (metres), so the hand model is shifted back by it.
const PALM_OFFSET_Z: f32 = 0.048;

/// Relaxed-hand base curl and full-fist extra curl, per finger phalanx
/// (proximal, intermediate, distal), in degrees.
const FINGER_BASE_CURL: [f32; 3] = [12.0, 8.0, 6.0];
const FINGER_MAX_CURL: [f32; 3] = [70.0, 90.0, 60.0];
/// The thumb curls less and its metacarpal turns toward the palm.
const THUMB_BASE_CURL: [f32; 3] = [0.0, 8.0, 6.0];
const THUMB_MAX_CURL: [f32; 3] = [18.0, 40.0, 40.0];

/// Rotation about the joint's local X that closes the hand toward the palm.
/// Verified visually; the left model is a mirror, so it can need its own sign.
fn curl_sign(side: HandSide) -> f32 {
    match side {
        HandSide::Left => 1.0,
        HandSide::Right => 1.0,
    }
}

/// On the scene's ready event: align the wrist to the palm pivot, swap the grey
/// material for the glove and cache the finger joints (WebXR hand joint names).
fn hand_model_ready(
    ready: On<WorldInstanceReady>,
    model: Query<&HandModel>,
    mut commands: Commands,
    children: Query<&Children>,
    parents: Query<&ChildOf>,
    names: Query<&Name>,
    transforms: Query<&Transform>,
    mut materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
    glove: Res<GloveMaterial>,
) {
    let root = ready.entity;
    let Ok(model) = model.get(root) else {
        return;
    };

    let mut wrist = None;
    let mut named: Vec<(String, Entity)> = Vec::new();
    for descendant in children.iter_descendants(root) {
        if let Ok(name) = names.get(descendant) {
            let name = name.as_str().to_string();
            if name == "wrist" {
                wrist = Some(descendant);
            }
            named.push((name, descendant));
        }
        if let Ok(mut material) = materials.get_mut(descendant) {
            material.0 = glove.0.clone();
        }
    }
    let Some(wrist) = wrist else {
        warn!("hand model for {:?} has no wrist joint", model.side);
        return;
    };
    let Some(wrist_to_root) = transform_to(root, wrist, &parents, &transforms) else {
        warn!("hand model for {:?} is not parented to its root", model.side);
        return;
    };

    // Put the wrist joint frame 4.8 cm behind the palm pivot: fingers along −Z,
    // back of the hand +Y, exactly the pivot's frame.
    let palm = Transform::from_translation(Vec3::new(0.0, 0.0, PALM_OFFSET_Z));
    commands
        .entity(root)
        .insert(palm.mul_transform(inverse_transform(wrist_to_root)));

    let find = |name: &str| {
        named
            .iter()
            .find_map(|(candidate, entity)| (candidate == name).then_some(*entity))
    };
    let joint = |name: &str, base_deg: f32, max_deg: f32| {
        find(name).and_then(|entity| {
            transforms.get(entity).ok().map(|bind| Joint {
                entity,
                bind: *bind,
                base_angle: base_deg.to_radians(),
                max_angle: max_deg.to_radians(),
            })
        })
    };

    let mut fingers = Vec::new();
    for finger in ["index", "middle", "ring", "pinky"] {
        // The metacarpal stays fixed for the four fingers: a real hand curls from
        // the knuckles out, and the bind pose already has them in place.
        let mut joints = Vec::new();
        if let Some(j) = joint(&format!("{finger}-finger-metacarpal"), 0.0, 0.0) {
            joints.push(j);
        }
        for (index, part) in ["proximal", "intermediate", "distal"].iter().enumerate() {
            let name = format!("{finger}-finger-phalanx-{part}");
            if let Some(j) = joint(&name, FINGER_BASE_CURL[index], FINGER_MAX_CURL[index]) {
                joints.push(j);
            }
        }
        if let Some(j) = joint(&format!("{finger}-finger-tip"), 0.0, 0.0) {
            joints.push(j);
        }
        if joints.len() > 2 {
            fingers.push(FingerChain { joints });
        }
    }
    let mut thumb = Vec::new();
    if let Some(j) = joint("thumb-metacarpal", THUMB_BASE_CURL[0], THUMB_MAX_CURL[0]) {
        thumb.push(j);
    }
    for (index, part) in ["proximal", "distal"].iter().enumerate() {
        let name = format!("thumb-phalanx-{part}");
        if let Some(j) = joint(&name, THUMB_BASE_CURL[index + 1], THUMB_MAX_CURL[index + 1]) {
            thumb.push(j);
        }
    }
    if let Some(j) = joint("thumb-tip", 0.0, 0.0) {
        thumb.push(j);
    }
    if thumb.len() > 2 {
        fingers.push(FingerChain { joints: thumb });
    }

    if fingers.len() != 5 {
        warn!(
            "hand model for {:?}: found {} finger chains, expected 5",
            model.side,
            fingers.len()
        );
    }
    commands.entity(root).insert(HandRig {
        side: model.side,
        fingers,
    });
}

/// Apply the smoothed curl to every cached finger joint.
///
/// The WebXR generic-hand rig stores the joints as flat siblings in Armature space
/// (absolute bind transforms, not a parent→child chain), so each joint is posed by
/// composing the parent's bind→posed delta: `p' = p_parent' + Δ_parent·(p − p_parent)`.
fn pose_hand_models(
    curl: Res<HandCurl>,
    rigs: Query<&HandRig>,
    mut transforms: Query<&mut Transform>,
) {
    for rig in &rigs {
        let curl = curl.value[rig.side.index()];
        let sign = curl_sign(rig.side);
        for finger in &rig.fingers {
            let mut parent_posed: Option<Vec3> = None;
            let mut parent_delta = Quat::IDENTITY;
            let mut previous_bind = Vec3::ZERO;
            for joint in &finger.joints {
                let angle = joint.base_angle + joint.max_angle * curl;
                let bend = Quat::from_rotation_x(-sign * angle);
                let rotation = parent_delta * joint.bind.rotation * bend;
                let translation = match parent_posed {
                    None => joint.bind.translation,
                    Some(parent) => parent + parent_delta * (joint.bind.translation - previous_bind),
                };
                if let Ok(mut transform) = transforms.get_mut(joint.entity) {
                    transform.translation = translation;
                    transform.rotation = rotation;
                }
                parent_delta = rotation * joint.bind.rotation.inverse();
                parent_posed = Some(translation);
                previous_bind = joint.bind.translation;
            }
        }
    }
}

/// `entity`'s transform expressed in `root`'s space (excluding `root` itself).
fn transform_to(
    root: Entity,
    entity: Entity,
    parents: &Query<&ChildOf>,
    transforms: &Query<&Transform>,
) -> Option<Transform> {
    let mut accumulated = *transforms.get(entity).ok()?;
    let mut current = entity;
    while let Ok(child_of) = parents.get(current) {
        let parent = child_of.parent();
        if parent == root {
            return Some(accumulated);
        }
        accumulated = transforms.get(parent).ok()?.mul_transform(accumulated);
        current = parent;
    }
    None
}

/// Inverse of a TRS transform (no shear; glTF node transforms qualify).
fn inverse_transform(transform: Transform) -> Transform {
    let rotation = transform.rotation.inverse();
    let scale = transform.scale.recip();
    Transform {
        translation: -(scale * (rotation * transform.translation)),
        rotation,
        scale,
    }
}

/// Distance from `point` to the segment `from`→`to`.
fn point_segment_distance(point: Vec3, from: Vec3, to: Vec3) -> f32 {
    let delta = to - from;
    let length_squared = delta.length_squared();
    if length_squared < 1e-9 {
        return (point - from).length();
    }
    let t = ((point - from).dot(delta) / length_squared).clamp(0.0, 1.0);
    (point - (from + delta * t)).length()
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
    lean: Res<ArmLean>,
    fake: Option<Res<crate::FakeHands>>,
    mut motion: ResMut<HandMotion>,
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
        let reach = if input.connected && crate::joycon::reach_enabled() {
            input.reach
        } else {
            Vec3::ZERO
        };
        let pose = solve_arm_with_reach(hand.side, orientation, reach);

        let target_position =
            body.transform_point(Vec3::Y * EYE_HEIGHT + ARM_ANCHOR_OFFSET + lean.offset + pose.hand);
        let target_rotation = body.rotation * pose.hand_rotation;

        // Drive by velocity, never by teleporting: props receive real momentum.
        let velocity = ((target_position - position.0) / dt).clamp_length_max(10.0);
        linear.0 = velocity;

        let mut delta = target_rotation * rotation.0.inverse();
        if delta.w < 0.0 {
            delta = -delta;
        }
        let (axis, angle) = delta.to_axis_angle();
        let angular_velocity = if angle > 1e-5 {
            axis * (angle / dt).min(30.0)
        } else {
            Vec3::ZERO
        };
        angular.0 = angular_velocity;

        motion.velocity[hand.side.index()] = velocity;
        motion.angular_velocity[hand.side.index()] = angular_velocity;
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

    #[test]
    fn lean_starts_at_twenty_degrees_below_and_is_zero_at_level() {
        assert_eq!(lean_offset(0.0), Vec3::ZERO);
        assert_eq!(lean_offset(20.0_f32.to_radians()), Vec3::ZERO);
        assert!(lean_offset(-10.0_f32.to_radians()).length() < 1e-6);
        assert!((lean_offset(-60.0_f32.to_radians()) - LEAN_OFFSET).length() < 1e-5);
        // Clamped past −60°.
        assert_eq!(lean_ratio(-89.0_f32.to_radians()), 1.0);
        assert!((lean_offset(-30.0_f32.to_radians()) - LEAN_OFFSET * 0.25).length() < 1e-5);
    }


    #[test]
    fn leaning_down_puts_the_neutral_palm_below_the_bench_top() {
        use crate::world::BENCH_TOP_Y;

        let hand = hand_world_space(solve_arm(HandSide::Right, Quat::IDENTITY))
            + lean_offset(-60.0_f32.to_radians());
        assert!(
            hand.y < BENCH_TOP_Y,
            "full lean should reach below the bench top: y = {}",
            hand.y
        );
        // And further forward than the neutral hover, so low floor props are reachable.
        let neutral = hand_world_space(solve_arm(HandSide::Right, Quat::IDENTITY));
        assert!(hand.z < neutral.z - 0.2);
    }

    // --- Reach offset + two-bone IK -------------------------------------------

    fn segment_lengths() -> (f32, f32) {
        ((ELBOW_REST - SHOULDER).length(), WRIST_REST.length())
    }

    #[test]
    fn reach_within_the_arm_reaches_the_displaced_wrist() {
        let base = solve_arm(HandSide::Right, Quat::IDENTITY);
        let reach = Vec3::new(0.0, 0.0, -0.06);
        let pose = solve_arm_with_reach(HandSide::Right, Quat::IDENTITY, reach);

        // The wrist follows the target exactly while it is within reach...
        assert!(
            (pose.wrist - (base.wrist + reach)).length() < 1e-3,
            "wrist {:?} should be {:?}",
            pose.wrist,
            base.wrist + reach
        );
        // ...the bones keep their lengths...
        let (upper, fore) = segment_lengths();
        assert!(((pose.elbow - pose.shoulder).length() - upper).abs() < 1e-3);
        assert!(((pose.wrist - pose.elbow).length() - fore).abs() < 1e-3);
        // ...and the palm stays 5 cm past the wrist, in the hand's local -Z.
        assert!(
            (pose.hand - (pose.wrist + pose.hand_rotation * CONTROLLER_REST)).length() < 1e-5,
            "hand {:?} should hang off wrist {:?}",
            pose.hand,
            pose.wrist
        );
    }

    #[test]
    fn reach_beyond_the_arm_is_clamped_to_the_real_reach() {
        let (upper, fore) = segment_lengths();
        let max_reach = upper + fore;
        let pose = solve_arm_with_reach(
            HandSide::Right,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, -REACH_IK_MAX),
        );
        let distance = (pose.wrist - pose.shoulder).length();
        assert!(
            distance <= max_reach + 1e-3,
            "the arm stretched to {distance} m (max {max_reach})"
        );
        assert!(
            distance > max_reach - 0.01,
            "a far target should fully extend the arm, got {distance}"
        );
        // Still a valid two-bone solution.
        assert!(((pose.wrist - pose.elbow).length() - fore).abs() < 1e-3);
    }

    #[test]
    fn ik_elbow_hangs_below_the_shoulder_wrist_line() {
        let pose = solve_arm_with_reach(
            HandSide::Right,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, -0.15),
        );
        let line = pose.wrist - pose.shoulder;
        let t = (pose.elbow - pose.shoulder).dot(line) / line.length_squared();
        let line_point = pose.shoulder + line * t;
        assert!(
            pose.elbow.y < line_point.y - 1e-3,
            "elbow {:?} should hang below the line point {:?}",
            pose.elbow,
            line_point
        );
        // The pole also pushes it outward, like a real elbow (never across the chest).
        assert!(
            pose.elbow.x > pose.shoulder.x - 0.02,
            "right elbow should stay on its own side: elbow {:?}, shoulder {:?}",
            pose.elbow,
            pose.shoulder
        );
    }

    #[test]
    fn zero_reach_keeps_the_daydream_pose_untouched() {
        let base = solve_arm(HandSide::Right, Quat::IDENTITY);
        let pose = solve_arm_with_reach(HandSide::Right, Quat::IDENTITY, Vec3::ZERO);
        assert!((pose.wrist - base.wrist).length() < 1e-6);
        assert!((pose.hand - base.hand).length() < 1e-6);
    }

    /// M2c measurement (no fix): how much of the 0.35 m reach the Daydream neutral pose can
    /// actually follow forward vs backward. The neutral wrist already sits 0.415 m from the
    /// shoulder of a 0.553 m arm, so a forward target hits the elbow's max extension early
    /// while backward travel is not limited. Reported in the M2c notes and used to decide
    /// whether the opt-in reach ever needs a shoulder-lean fix.
    #[test]
    fn forward_ik_travel_at_neutral_is_measured() {
        let base = solve_arm(HandSide::Right, Quat::IDENTITY);
        let forward = solve_arm_with_reach(
            HandSide::Right,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, -REACH_IK_MAX),
        );
        let backward = solve_arm_with_reach(
            HandSide::Right,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, REACH_IK_MAX),
        );
        let forward_hand = base.hand.z - forward.hand.z;
        let backward_hand = backward.hand.z - base.hand.z;
        let forward_wrist = base.wrist.z - forward.wrist.z;
        let backward_wrist = backward.wrist.z - base.wrist.z;
        println!(
            "[M2c] neutral IK travel forward: hand {forward_hand:.3} m / wrist {forward_wrist:.3} m; \
             backward: hand {backward_hand:.3} m / wrist {backward_wrist:.3} m"
        );
        // Both directions move, and the forward range is the ~0.21 m measured on 2026-11-21.
        assert!(forward_hand > 0.20, "forward travel dropped to {forward_hand}");
        assert!(backward_hand > 0.30, "backward travel dropped to {backward_hand}");
    }

    #[test]
    fn upper_arm_is_hidden_at_rest_and_back_when_leaning() {
        let neutral = solve_arm(HandSide::Right, Quat::IDENTITY);
        assert!(
            upper_fade(&neutral, Vec3::ZERO) < 0.01,
            "at rest the neutral shoulder is next to the eye: the sleeve must hide"
        );

        // Full torso lean (looking down 60°) moves the shoulders 35 cm down / 25 cm forward,
        // clear of the camera, so the upper arm can come back.
        assert!(
            upper_fade(&neutral, LEAN_OFFSET) > 0.99,
            "leaning should bring the upper arm back (fade {})",
            upper_fade(&neutral, LEAN_OFFSET)
        );
    }
}
