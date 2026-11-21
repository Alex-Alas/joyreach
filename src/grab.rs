//! M1 grab & throw: ZL/ZR closes a hand on the nearest prop, the prop is driven by a
//! force-limited velocity tracker so the table and other props can block it, and the
//! release throws it with the hand's recent velocity.
//!
//! M2d adds the palm beam: if nothing is inside the direct-grab radius but the hand's
//! [`HandRay`](crate::handray::HandRay) hits a dynamic prop, the hand grabs it at that
//! distance physics-gun style. The prop is pulled to the beam point by the same force-limited
//! tracker, its rotation follows the wrist (delta since grab), and the D-pad / X-Y push and
//! pull it between 0.35 m and 8 m. A thin hand-coloured beam and a hit dot show the aim
//! (DESIGN §7.1), and the existing inverted-hull outline marks the hover target.
//!
//! Also here: the hand-coloured grab outline (DESIGN §8) and the HD Rumble triggers
//! (DESIGN §9). The outline is an inverted hull (back faces of a slightly larger copy
//! of the prop mesh, unlit) — one child per hand so each hand owns its colour.
//!
//! The pure decision/kinematics helpers at the top are unit tested; the ECS systems
//! below only feed them.

use avian3d::prelude::*;
use bevy::light::NotShadowCaster;
use bevy::prelude::*;
use bevy::render::render_resource::Face;

use crate::arms::{HAND_LAYER_LEFT, HAND_LAYER_RIGHT, HandBody, HandMotion, HandSide};
use crate::handray::{HAND_RAY_MAX, HandRay};
use crate::joycon::{Hands, Rumble, RumbleHand};
use crate::theme;

// --- Tuning (all distances in metres, forces in newtons) ----------------------

/// A prop is a candidate when its collider surface is this close to the palm point.
pub const GRAB_RADIUS: f32 = 0.12;
/// Cap on the force the hand may use to hold a prop. Acceleration = force / mass, so
/// a heavier prop lags behind the hand while a light one tracks it tightly.
/// Realistic masses (0.2–0.8 kg props, ~3 kg crates) mean 60 N is snappy on a prop but
/// visibly lags on a crate — and still beats the crate's ~29 N of weight.
pub const MAX_GRAB_FORCE: f32 = 60.0;
/// Cap on the torque used to twist a held prop (N·m).
pub const MAX_GRAB_TORQUE: f32 = 8.0;
/// Throw speed = average hand velocity over the last [`THROW_SAMPLES`] fixed steps × gain.
/// Since M2a the hand velocity includes the real IMU reach, so it needs much less help.
pub const THROW_GAIN: f32 = 1.1;
/// How fast a grabbed prop is pulled into the palm (s). ~80 ms settle, DESIGN §7.
pub const GRIP_SNAP_TAU: f32 = 0.03;
pub const THROW_MAX_SPEED: f32 = 12.0;
pub const THROW_MAX_SPIN: f32 = 12.0;
pub const THROW_SAMPLES: usize = 5;
/// Extra outline shell thickness, in world metres, used to pick each prop's scale.
pub const OUTLINE_THICKNESS: f32 = 0.004;
/// Outline fade after release (DESIGN §8: 150 ms).
pub const OUTLINE_FADE: f32 = 0.15;

/// Beam hold distance limits (DESIGN §7.1).
pub const BEAM_MIN_DISTANCE: f32 = 0.35;
pub const BEAM_MAX_DISTANCE: f32 = HAND_RAY_MAX;
/// Push/pull rate while a beam holds a prop: D-pad ↑/↓ (left hand) or X/Y (right hand).
pub const BEAM_PUSH_PULL_SPEED: f32 = 1.5;
/// Idle beam/dot size and glow, and the brighter/thicker held state (DESIGN §7.1).
const BEAM_RADIUS_IDLE: f32 = 0.0035;
const BEAM_RADIUS_HOLD: f32 = 0.007;
const BEAM_DOT_RADIUS_IDLE: f32 = 0.013;
const BEAM_DOT_RADIUS_HOLD: f32 = 0.020;
const BEAM_GLOW_IDLE: f32 = 2.4;
const BEAM_GLOW_HOLD: f32 = 6.0;
const BEAM_ALPHA_IDLE: f32 = 0.38;

/// HD Rumble feel. Strength is 0..1.
const RUMBLE_GRAB: (f32, u32) = (0.9, 40);
const RUMBLE_THROW: (f32, u32) = (0.55, 15);
/// Beam lock: a short tick, shorter than the direct grab pulse.
const RUMBLE_BEAM: (f32, u32) = (0.5, 22);
/// Impact pulses: 10–40 ms, proportional to the relative speed.
const RUMBLE_TOUCH_MIN_MS: u32 = 10;
const RUMBLE_TOUCH_MAX_MS: u32 = 40;
const RUMBLE_TOUCH_SPEED: f32 = 3.0;
const RUMBLE_TOUCH_COOLDOWN: f32 = 0.12;

// --- Pure helpers -------------------------------------------------------------

/// Nearest candidate within `radius`. Ties keep the first, so the caller's order is
/// the deterministic tie-breaker. `None` when everything is out of reach.
pub fn nearest_candidate<T: Copy>(
    radius: f32,
    candidates: impl IntoIterator<Item = (T, f32)>,
) -> Option<(T, f32)> {
    let mut best: Option<(T, f32)> = None;
    for (id, distance) in candidates {
        if distance > radius {
            continue;
        }
        match best {
            Some((_, best_distance)) if best_distance <= distance => {}
            _ => best = Some((id, distance)),
        }
    }
    best
}

/// One prop per hand: if both hands reach the same prop, the closer hand keeps it and
/// the other hand gets nothing this frame.
pub fn resolve_claims<T: Copy + PartialEq>(
    left: Option<(T, f32)>,
    right: Option<(T, f32)>,
) -> (Option<T>, Option<T>) {
    match (left, right) {
        (Some((left_id, left_distance)), Some((right_id, right_distance)))
            if left_id == right_id =>
        {
            if left_distance <= right_distance {
                (Some(left_id), None)
            } else {
                (None, Some(right_id))
            }
        }
        (left, right) => (left.map(|(id, _)| id), right.map(|(id, _)| id)),
    }
}

/// Move a velocity toward `desired` by at most `max_delta` (the force limit).
pub fn limited_velocity_step(current: Vec3, desired: Vec3, max_delta: f32) -> Vec3 {
    current + (desired - current).clamp_length_max(max_delta)
}

/// Which grab wins when a hand can both touch and aim at a prop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GrabKind {
    /// The prop is inside [`GRAB_RADIUS`] of the palm: M1 grip snap, unchanged.
    Direct,
    /// The palm ray hit a dynamic prop further out: physics-gun hold.
    Beam,
}

/// Direct wins when the palm is actually touching something; otherwise the beam wins when the
/// ray reached a prop; otherwise the hand grabs nothing. Pure so the precedence is tested.
pub fn select_grab(
    direct_distance: Option<f32>,
    beam_distance: Option<f32>,
    radius: f32,
) -> Option<GrabKind> {
    if direct_distance.is_some_and(|distance| distance <= radius) {
        Some(GrabKind::Direct)
    } else if beam_distance.is_some_and(|distance| distance >= 0.0) {
        Some(GrabKind::Beam)
    } else {
        None
    }
}

/// Point on the beam at `hold_distance` from the palm. The prop's grabbed local point is
/// driven to this; the prop centre then hangs off it by the grab offset.
pub fn beam_target(origin: Vec3, direction: Vec3, hold_distance: f32) -> Vec3 {
    origin + direction.normalize_or_zero() * hold_distance.max(0.0)
}

/// Push/pull one fixed step. Positive `axis` moves the target away from the palm.
pub fn adjust_hold_distance(distance: f32, axis: f32, dt: f32) -> f32 {
    let step = axis.clamp(-1.0, 1.0) * BEAM_PUSH_PULL_SPEED * dt.max(0.0);
    (distance + step).clamp(BEAM_MIN_DISTANCE, BEAM_MAX_DISTANCE)
}

/// Prop rotation while beam-held: the hand's rotation since grab, applied to the prop's
/// rotation at grab. A wrist twist therefore rotates the object 1:1.
pub fn beam_rotation(hand_now: Quat, hand_at_grab: Quat, prop_at_grab: Quat) -> Quat {
    ((hand_now * hand_at_grab.inverse()) * prop_at_grab).normalize()
}

/// World-space point of the prop that was grabbed, from its current centre and rotation.
pub fn held_point(center: Vec3, rotation: Quat, local_offset: Vec3) -> Vec3 {
    center + rotation * local_offset
}

/// Push/pull intent from the face buttons: left D-pad up/down, right X/Y.
/// Returns +1 away, −1 closer, 0 when neither/both are pressed.
pub fn push_pull_axis(side: HandSide, face: [bool; 4]) -> f32 {
    let (away, closer) = match side {
        // Left face order is [up, down, left, right].
        HandSide::Left => (face[0], face[1]),
        // Right face order is [X, B, Y, A]: B is the jump button, so X and Y are used.
        HandSide::Right => (face[0], face[2]),
    };
    let mut axis = 0.0;
    if away {
        axis += 1.0;
    }
    if closer {
        axis -= 1.0;
    }
    axis
}

/// Average of the hand velocity history × gain, clamped to `max_speed`.
pub fn throw_velocity(samples: &[Vec3], gain: f32, max_speed: f32) -> Vec3 {
    if samples.is_empty() {
        return Vec3::ZERO;
    }
    let sum: Vec3 = samples.iter().copied().sum();
    let average = sum / samples.len() as f32;
    (average * gain).clamp_length_max(max_speed)
}

/// Outline scale for a prop of the given half extent: a constant shell thickness in
/// world space, so a 10 cm cube and a 52 cm ball get the same visual outline weight.
pub fn outline_scale(half_extent: f32) -> f32 {
    1.0 + OUTLINE_THICKNESS / half_extent.max(0.01)
}

/// Distance from `point` to a prop's collider surface (0 when inside).
pub fn palm_to_prop(point: Vec3, translation: Vec3, rotation: Quat, collider: &Collider) -> f32 {
    collider.distance_to_point(translation, rotation, point, true)
}

/// Half the prop's size along `direction` (world space): the distance from its centre to the
/// surface on that side. Cast from the centre outward; a ray that starts inside a non-solid
/// shape stops at the boundary, which is exactly the half extent (sphere radius, cuboid half
/// side along that axis, ...).
pub fn half_extent_along(
    collider: &Collider,
    translation: Vec3,
    rotation: Quat,
    direction: Vec3,
) -> f32 {
    let direction = direction.normalize_or_zero();
    if direction == Vec3::ZERO {
        return 0.05;
    }
    collider
        .cast_ray(translation, rotation, translation, direction, 1.0, false)
        .map(|(toi, _)| toi)
        .filter(|toi| toi.is_finite() && *toi > 1e-4)
        .unwrap_or(0.05)
}

/// Hand-space offset that puts the prop's centre on the palm surface, right under the palm
/// centre: the palm faces local −Y (same frame as the glove, whose back is +Y).
pub fn grip_offset(half_size: f32) -> Vec3 {
    Vec3::NEG_Y * half_size.max(0.01)
}

// --- Outline material / children ---------------------------------------------

/// Inverted-hull outline: unlit, back faces only, one per hand colour.
pub fn outline_material(color: Color) -> StandardMaterial {
    StandardMaterial {
        base_color: color,
        unlit: true,
        cull_mode: Some(Face::Front),
        alpha_mode: AlphaMode::Blend,
        fog_enabled: false,
        ..default()
    }
}

/// Marks the inverted-hull child of a prop. `side` decides which hand it belongs to.
#[derive(Component)]
pub struct GrabOutline {
    pub side: HandSide,
}

/// Spawn the two outline shells under a freshly spawned prop.
pub fn spawn_outlines(
    commands: &mut Commands,
    prop: Entity,
    mesh: Handle<Mesh>,
    half_extent: f32,
    materials: &OutlineMaterials,
) {
    let scale = outline_scale(half_extent);
    commands.entity(prop).with_children(|children| {
        for side in [HandSide::Left, HandSide::Right] {
            children.spawn((
                GrabOutline { side },
                Mesh3d(mesh.clone()),
                MeshMaterial3d(materials.handle(side)),
                Transform::from_scale(Vec3::splat(scale)),
                Visibility::Hidden,
                bevy::light::NotShadowCaster,
            ));
        }
    });
}

/// The two shared outline materials (one per hand colour, see DESIGN §3/§8).
#[derive(Resource, Clone)]
pub struct OutlineMaterials {
    left: Handle<StandardMaterial>,
    right: Handle<StandardMaterial>,
    colors: [Color; 2],
}

impl OutlineMaterials {
    pub fn new(materials: &mut Assets<StandardMaterial>) -> Self {
        Self {
            left: materials.add(outline_material(theme::JOY_LEFT)),
            right: materials.add(outline_material(theme::JOY_RIGHT)),
            colors: [theme::JOY_LEFT, theme::JOY_RIGHT],
        }
    }

    pub fn handle(&self, side: HandSide) -> Handle<StandardMaterial> {
        match side {
            HandSide::Left => self.left.clone(),
            HandSide::Right => self.right.clone(),
        }
    }

    /// Set the shared material's alpha. Only one outline per hand is ever visible,
    /// so a single material per colour is enough to fade the release.
    fn set_alpha(&self, materials: &mut Assets<StandardMaterial>, side: HandSide, alpha: f32) {
        let color = self.colors[side.index()];
        if let Some(mut material) = materials.get_mut(self.handle(side).id()) {
            material.base_color = Color::from(Srgba::from(color).with_alpha(alpha.clamp(0.0, 1.0)));
        }
    }
}

// --- Grab state ---------------------------------------------------------------

/// Marks a dynamic prop that a hand can grab.
#[derive(Component)]
pub struct Grabbable;

#[derive(Clone, Copy, Debug)]
enum HoldKind {
    /// M1: the prop is snapped into the palm and follows the hand frame.
    Direct {
        /// Prop centre in hand space: starts where the hand found it and eases into `grip`.
        offset: Vec3,
        /// Where the prop should sit once the grab settles (in the palm).
        grip: Vec3,
        /// Prop orientation in hand space, captured at grab time.
        rotation_offset: Quat,
    },
    /// M2d: the prop hangs off the palm ray at the distance it was grabbed.
    Beam {
        /// Distance from the palm to the grabbed point, along the ray.
        distance: f32,
        /// Grabbed point in the prop's local frame (no jump at capture, and it rotates with it).
        local_offset: Vec3,
        /// Hand orientation at grab; the wrist delta since then drives the prop.
        hand_rotation: Quat,
        /// Prop orientation at grab.
        prop_rotation: Quat,
        /// Extra world-space rotation from L/R + stick, applied on top of the wrist delta.
        spin: Quat,
        /// Physics the prop gets when released (cycled with → / A while held).
        mode: PropMode,
    },
}

/// What a prop does after the beam lets go of it (physics-gun modes).
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PropMode {
    /// Gravity and the usual contacts.
    #[default]
    Normal,
    /// Hangs where it was released (no gravity, heavy damping); still solid and pushable.
    Frozen,
    /// No gravity, no damping: keeps drifting and spinning after a throw.
    ZeroG,
}

impl PropMode {
    pub fn next(self) -> Self {
        match self {
            PropMode::Normal => PropMode::Frozen,
            PropMode::Frozen => PropMode::ZeroG,
            PropMode::ZeroG => PropMode::Normal,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            PropMode::Normal => "Normal",
            PropMode::Frozen => "Congelado",
            PropMode::ZeroG => "Gravedad cero",
        }
    }

    /// (gravity scale, linear damping, angular damping) applied at release.
    pub fn physics(self) -> (f32, f32, f32) {
        match self {
            PropMode::Normal => (1.0, 0.0, 0.0),
            PropMode::Frozen => (0.0, 12.0, 12.0),
            PropMode::ZeroG => (0.0, 0.02, 0.02),
        }
    }
}

/// Per-hand beam status for the HUD and the stick routing (L/R + stick rotates the prop
/// instead of moving / looking).
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct BeamStatus {
    pub hands: [BeamHandStatus; 2],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BeamHandStatus {
    /// The hand is holding a prop with the beam.
    pub holding: bool,
    /// L/R is held: this hand's stick rotates the prop.
    pub rotating: bool,
    pub mode: PropMode,
}

/// L/R + stick rotation speed of a beam-held prop (rad/s at full tilt).
pub const BEAM_ROTATE_SPEED: f32 = 2.5;

/// Apply one stick step of rotation: yaw around world up, pitch around the beam's right axis.
pub fn beam_spin_step(spin: Quat, stick: Vec2, beam_direction: Vec3, dt: f32) -> Quat {
    let right = beam_direction.cross(Vec3::Y).normalize_or(Vec3::X);
    let yaw = Quat::from_rotation_y(stick.x * BEAM_ROTATE_SPEED * dt);
    let pitch = Quat::from_axis_angle(right, -stick.y * BEAM_ROTATE_SPEED * dt);
    (yaw * pitch * spin).normalize()
}

#[derive(Clone, Copy, Debug)]
struct HeldProp {
    entity: Entity,
    hold: HoldKind,
}

#[derive(Clone, Copy, Debug)]
struct HandGrab {
    held: Option<HeldProp>,
    prev_trigger: bool,
    /// → (left) / A (right) last step, for the physics-mode edge.
    prev_mode_button: bool,
    /// Outline of the prop released moments ago (entity, seconds left).
    fade: Option<(Entity, f32)>,
    /// Hand velocity history for the throw (ring buffer).
    linear_history: [Vec3; THROW_SAMPLES],
    angular_history: [Vec3; THROW_SAMPLES],
    history_head: usize,
    history_len: usize,
    impact_cooldown: f32,
}

impl Default for HandGrab {
    fn default() -> Self {
        Self {
            held: None,
            prev_trigger: false,
            prev_mode_button: false,
            fade: None,
            linear_history: [Vec3::ZERO; THROW_SAMPLES],
            angular_history: [Vec3::ZERO; THROW_SAMPLES],
            history_head: 0,
            history_len: 0,
            impact_cooldown: 0.0,
        }
    }
}

impl HandGrab {
    fn push_velocity(&mut self, linear: Vec3, angular: Vec3) {
        self.linear_history[self.history_head] = linear;
        self.angular_history[self.history_head] = angular;
        self.history_head = (self.history_head + 1) % THROW_SAMPLES;
        self.history_len = (self.history_len + 1).min(THROW_SAMPLES);
    }

    /// Samples oldest → newest, so the average is the hand's last ~5 fixed steps.
    fn take_linear(&self) -> Vec<Vec3> {
        self.collect(&self.linear_history)
    }

    fn take_angular(&self) -> Vec<Vec3> {
        self.collect(&self.angular_history)
    }

    fn collect(&self, ring: &[Vec3; THROW_SAMPLES]) -> Vec<Vec3> {
        (0..self.history_len)
            .map(|i| {
                let index =
                    (self.history_head + THROW_SAMPLES - self.history_len + i) % THROW_SAMPLES;
                ring[index]
            })
            .collect()
    }
}

#[derive(Resource, Default)]
pub struct GrabState {
    hands: [HandGrab; 2],
}

impl GrabState {
    fn hand(&self, side: HandSide) -> &HandGrab {
        &self.hands[side.index()]
    }

    fn hand_mut(&mut self, side: HandSide) -> &mut HandGrab {
        &mut self.hands[side.index()]
    }
}

// --- Beam visuals (DESIGN §7.1) ----------------------------------------------

/// One of the two meshes that draw a hand's beam: the thin cylinder or the hit dot.
#[derive(Component, Clone, Copy)]
struct BeamVisual {
    side: HandSide,
    dot: bool,
}

/// Shared beam material per hand. Only one beam per hand is ever visible, so mutating the
/// single material's alpha/emissive per frame is enough (same trick as the outline).
#[derive(Resource, Clone)]
struct BeamMaterials {
    handles: [Handle<StandardMaterial>; 2],
    colors: [Color; 2],
}

/// The per-hand beam pose for this frame, computed once and shared by beam + dot.
#[derive(Clone, Copy, Debug, Default)]
struct BeamPose {
    visible: bool,
    origin: Vec3,
    end: Vec3,
    holding: bool,
}

fn beam_material(color: Color) -> StandardMaterial {
    // Lit (black base) rather than unlit: Bevy's unlit path ignores `emissive`, and the beam
    // leans on emissive for bloom. A black, fully rough surface contributes no lighting of
    // its own, so only the hand colour remains.
    let linear = color.to_linear();
    StandardMaterial {
        base_color: Color::srgba(0.0, 0.0, 0.0, BEAM_ALPHA_IDLE),
        emissive: LinearRgba::rgb(
            linear.red * BEAM_GLOW_IDLE,
            linear.green * BEAM_GLOW_IDLE,
            linear.blue * BEAM_GLOW_IDLE,
        ),
        perceptual_roughness: 1.0,
        metallic: 0.0,
        reflectance: 0.0,
        alpha_mode: AlphaMode::Blend,
        fog_enabled: false,
        ..default()
    }
}

fn setup_beams(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // A thin cylinder (radius 0.5 × scale) stretched between palm and target.
    let beam_mesh = meshes.add(Cylinder::new(0.5, 1.0).mesh().resolution(8));
    let dot_mesh = meshes.add(Sphere::new(0.5).mesh().uv(10, 6));

    let colors = [theme::JOY_LEFT, theme::JOY_RIGHT];
    let handles = [
        materials.add(beam_material(colors[0])),
        materials.add(beam_material(colors[1])),
    ];

    for side in [HandSide::Left, HandSide::Right] {
        for dot in [false, true] {
            commands.spawn((
                BeamVisual { side, dot },
                Mesh3d(if dot {
                    dot_mesh.clone()
                } else {
                    beam_mesh.clone()
                }),
                MeshMaterial3d(handles[side.index()].clone()),
                Transform::default(),
                Visibility::Hidden,
                NotShadowCaster,
            ));
        }
    }

    commands.insert_resource(BeamMaterials { handles, colors });
}

// --- ECS ----------------------------------------------------------------------

pub struct GrabPlugin;

impl Plugin for GrabPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GrabState>()
            .init_resource::<BeamStatus>()
            .add_systems(Startup, setup_beams)
            .add_systems(
                Update,
                (
                    update_outlines,
                    update_beams,
                    rumble_on_impact,
                    tick_impact_cooldowns,
                )
                    .chain(),
            )
            .add_systems(
                FixedUpdate,
                drive_grabs
                    .after(crate::arms::HandBodiesSet)
                    .after(crate::handray::HandRaySet),
            );
    }
}

/// Snapshot of a grabbable prop: entity, centre, rotation, collider, dynamic.
type PropSnapshot = Vec<(Entity, Vec3, Quat, Collider, bool)>;

/// Snapshot the props so both hands test the same list in one pass.
fn prop_snapshot(
    props: &Query<(Entity, &Position, &Rotation, &Collider, &RigidBody), With<Grabbable>>,
) -> PropSnapshot {
    props
        .iter()
        .map(|(entity, position, rotation, collider, body)| {
            (
                entity,
                position.0,
                rotation.0,
                collider.clone(),
                body.is_dynamic(),
            )
        })
        .collect()
}

/// Best candidate for one hand, shared by the outline (Update) and the grab (fixed).
fn candidate_for(palm: Vec3, props: &[(Entity, Vec3, Quat, Collider, bool)]) -> Option<(Entity, f32)> {
    nearest_candidate(
        GRAB_RADIUS,
        props.iter().map(|(entity, position, rotation, collider, _)| {
            (*entity, palm_to_prop(palm, *position, *rotation, collider))
        }),
    )
}

/// Beam candidate: the ray must end on a dynamic, grabbable prop. Static geometry (bench,
/// deck, parapets) blocks the beam but is not a target, so aiming at the table gives nothing.
fn beam_candidate(ray: &HandRay, props: &[(Entity, Vec3, Quat, Collider, bool)]) -> Option<(Entity, f32)> {
    let hit = ray.hit?;
    if hit.distance > BEAM_MAX_DISTANCE {
        return None;
    }
    let (_, _, _, _, dynamic) = props.iter().find(|(entity, ..)| *entity == hit.entity)?;
    dynamic.then_some((hit.entity, hit.distance))
}

/// Per fixed step: input edges, force-limited tracking of the held prop, throw.
#[allow(clippy::too_many_arguments)]
fn drive_grabs(
    mut commands: Commands,
    time: Res<Time<Fixed>>,
    hands: Res<Hands>,
    fake: Option<Res<crate::FakeHands>>,
    motion: Res<HandMotion>,
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<GrabState>,
    palms: Query<(&HandBody, &Position, &Rotation, &HandRay), Without<Grabbable>>,
    mut props: Query<
        (
            &Position,
            &Rotation,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &ComputedMass,
            &ComputedAngularInertia,
            &Collider,
            &RigidBody,
        ),
        With<Grabbable>,
    >,
    prop_ids: Query<(Entity, &Position, &Rotation, &Collider, &RigidBody), With<Grabbable>>,
    prop_modes: Query<&PropMode>,
    mut status: ResMut<BeamStatus>,
    mut rumble: MessageWriter<Rumble>,
) {
    let dt = time.delta_secs().max(1e-5);
    // The fixed step runs before `Update`, so fake mode must read the mirrored snapshot.
    let hands: &Hands = match fake.as_ref() {
        Some(fake) => &fake.0,
        None => &hands,
    };

    let mut palm_poses: [Option<(Vec3, Quat, HandRay)>; 2] = [None; 2];
    for (body, position, rotation, ray) in &palms {
        palm_poses[body.side.index()] = Some((position.0, rotation.0, *ray));
    }

    // --- Candidate selection (one prop per hand, nearest hand wins) ----------
    let prop_list = prop_snapshot(&prop_ids);
    let mut candidates = [None; 2];
    let mut kinds = [GrabKind::Direct; 2];
    for side in [HandSide::Left, HandSide::Right] {
        let Some((palm, _, ray)) = palm_poses[side.index()] else {
            continue;
        };
        let direct = candidate_for(palm, &prop_list);
        let beam = beam_candidate(&ray, &prop_list);
        match select_grab(
            direct.map(|(_, distance)| distance),
            beam.map(|(_, distance)| distance),
            GRAB_RADIUS,
        ) {
            Some(GrabKind::Direct) => {
                candidates[side.index()] = direct;
                kinds[side.index()] = GrabKind::Direct;
            }
            Some(GrabKind::Beam) => {
                candidates[side.index()] = beam;
                kinds[side.index()] = GrabKind::Beam;
            }
            None => {}
        }
    }
    let (mut left_candidate, mut right_candidate) =
        resolve_claims(candidates[0], candidates[1]);
    // A prop already in the other hand cannot be grabbed again.
    let held_left = state.hands[0].held.map(|held| held.entity);
    let held_right = state.hands[1].held.map(|held| held.entity);
    if left_candidate == held_right {
        left_candidate = None;
    }
    if right_candidate == held_left {
        right_candidate = None;
    }

    // --- Edges and held-prop tracking ---------------------------------------
    for side in [HandSide::Left, HandSide::Right] {
        let input = match side {
            HandSide::Left => &hands.left,
            HandSide::Right => &hands.right,
        };
        let trigger = input.connected && input.trigger;
        let just_pressed = trigger && !state.hand(side).prev_trigger;
        let just_released = !trigger && state.hand(side).prev_trigger;
        let hand_velocity = motion.velocity[side.index()];
        let hand_spin = motion.angular_velocity[side.index()];
        let hand = state.hand_mut(side);
        hand.prev_trigger = trigger;
        hand.push_velocity(hand_velocity, hand_spin);
        // → (left d-pad) / A (right): cycle what the beam-held prop does when released.
        let mode_button = input.connected && input.face[3];
        let mode_pressed = mode_button && !hand.prev_mode_button;
        hand.prev_mode_button = mode_button;
        status.hands[side.index()] = BeamHandStatus::default();

        let Some((palm, palm_rotation, ray)) = palm_poses[side.index()] else {
            continue;
        };

        if just_pressed && hand.held.is_none() {
            let candidate = match side {
                HandSide::Left => left_candidate,
                HandSide::Right => right_candidate,
            };
            if let Some(entity) = candidate {
                if let Ok((position, rotation, _, _, _, _, collider, _)) = props.get(entity) {
                    let hold = match kinds[side.index()] {
                        GrabKind::Direct => {
                            // Pull the prop into the palm over ~80 ms: its centre goes to the
                            // palm surface on the palm side, so it reads as held instead of
                            // hovering 12–14 cm away (DESIGN §7).
                            let palm_normal = palm_rotation * Vec3::NEG_Y;
                            let half_size =
                                half_extent_along(collider, position.0, rotation.0, palm_normal);
                            HoldKind::Direct {
                                offset: palm_rotation.inverse() * (position.0 - palm),
                                grip: grip_offset(half_size),
                                rotation_offset: palm_rotation.inverse() * rotation.0,
                            }
                        }
                        GrabKind::Beam => {
                            // No positional jump: remember where on the prop the ray landed,
                            // in the prop's own frame, and how far the beam point is.
                            let Some(hit) = ray.hit.filter(|hit| hit.entity == entity) else {
                                continue;
                            };
                            HoldKind::Beam {
                                distance: hit.distance,
                                local_offset: rotation.0.inverse() * (hit.point - position.0),
                                hand_rotation: palm_rotation,
                                prop_rotation: rotation.0,
                                spin: Quat::IDENTITY,
                                mode: prop_modes.get(entity).copied().unwrap_or_default(),
                            }
                        }
                    };
                    hand.held = Some(HeldProp { entity, hold });
                    commands
                        .entity(entity)
                        .insert(SleepingDisabled)
                        .insert(held_collision_layers(side));
                    let (strength, ms) = match kinds[side.index()] {
                        GrabKind::Direct => RUMBLE_GRAB,
                        GrabKind::Beam => RUMBLE_BEAM,
                    };
                    rumble.write(Rumble {
                        hand: rumble_hand(side),
                        strength,
                        ms,
                    });
                    match kinds[side.index()] {
                        GrabKind::Direct => info!(
                            "grab {:?}: prop {entity:?} at {:.3} m",
                            side,
                            (position.0 - palm).length()
                        ),
                        GrabKind::Beam => info!(
                            "beam grab {:?}: prop {entity:?} at {:.3} m",
                            side,
                            ray.hit.map_or(0.0, |hit| hit.distance)
                        ),
                    }
                }
            }
        }

        if hand.held.is_none() {
            continue;
        }
        // Ease the capture offset into the palm (grip snap, direct only).
        if let Some(held) = hand.held.as_mut() {
            if let HoldKind::Direct { offset, grip, .. } = &mut held.hold {
                let blend = 1.0 - (-dt / GRIP_SNAP_TAU).exp();
                *offset = offset.lerp(*grip, blend);
            }
        }
        let Some(mut held) = hand.held else {
            continue;
        };

        // Push/pull a beam hold: left D-pad ↑/↓, right X/Y, or Q/E on the keyboard.
        if let HoldKind::Beam { distance, .. } = &mut held.hold {
            let mut axis = push_pull_axis(side, input.face);
            if keys.pressed(KeyCode::KeyE) {
                axis += 1.0; // away
            }
            if keys.pressed(KeyCode::KeyQ) {
                axis -= 1.0; // closer
            }
            let axis = axis.clamp(-1.0, 1.0);
            if axis != 0.0 {
                *distance = adjust_hold_distance(*distance, axis, dt);
            }
        }
        // L/R + this hand's stick rotates the prop; → / A cycles its release physics.
        if let HoldKind::Beam { spin, mode, .. } = &mut held.hold {
            let rotating = input.connected && input.shoulder;
            if rotating {
                *spin = beam_spin_step(*spin, input.stick, ray.direction, dt);
            }
            if mode_pressed {
                *mode = mode.next();
                rumble.write(Rumble {
                    hand: rumble_hand(side),
                    strength: 0.45,
                    ms: 18,
                });
                info!("beam mode {:?}: {}", side, mode.label());
            }
            status.hands[side.index()] = BeamHandStatus {
                holding: true,
                rotating,
                mode: *mode,
            };
        }
        hand.held = Some(held);

        if just_released {
            let mut speed = 0.0;
            if let Ok((_, _, mut prop_linear, mut prop_angular, ..)) = props.get_mut(held.entity) {
                match held.hold {
                    HoldKind::Direct { .. } => {
                        // Throw: the hand's recent velocity, gained and clamped.
                        prop_linear.0 =
                            throw_velocity(&hand.take_linear(), THROW_GAIN, THROW_MAX_SPEED);
                        prop_angular.0 =
                            throw_velocity(&hand.take_angular(), THROW_GAIN, THROW_MAX_SPIN);
                    }
                    HoldKind::Beam { .. } => {
                        // Keep the velocity the beam built up (a wrist flick throws it), with
                        // the same gain and clamps as a direct throw.
                        prop_linear.0 =
                            (prop_linear.0 * THROW_GAIN).clamp_length_max(THROW_MAX_SPEED);
                        prop_angular.0 =
                            (prop_angular.0 * THROW_GAIN).clamp_length_max(THROW_MAX_SPIN);
                    }
                }
                speed = prop_linear.0.length();
            }
            commands
                .entity(held.entity)
                .remove::<SleepingDisabled>()
                .insert(CollisionLayers::DEFAULT);
            if let HoldKind::Beam { mode, .. } = held.hold {
                let (gravity, linear_damping, angular_damping) = mode.physics();
                commands.entity(held.entity).insert((
                    mode,
                    GravityScale(gravity),
                    LinearDamping(linear_damping),
                    AngularDamping(angular_damping),
                ));
            }
            hand.held = None;
            hand.fade = Some((held.entity, OUTLINE_FADE));
            let (strength, ms) = RUMBLE_THROW;
            rumble.write(Rumble {
                hand: rumble_hand(side),
                strength,
                ms,
            });
            info!("throw {:?}: {speed:.2} m/s", side);
            continue;
        }

        // --- Force-limited velocity tracking --------------------------------
        let Ok((position, rotation, mut linear, mut angular, mass, inertia, _, _)) =
            props.get_mut(held.entity)
        else {
            // The prop disappeared: drop the grab.
            hand.held = None;
            continue;
        };

        let (target_position, target_rotation, target_velocity) = match held.hold {
            HoldKind::Direct {
                offset,
                rotation_offset,
                ..
            } => (
                palm + palm_rotation * offset,
                (palm_rotation * rotation_offset).normalize(),
                hand_velocity,
            ),
            HoldKind::Beam {
                distance,
                local_offset,
                hand_rotation,
                prop_rotation,
                spin,
                ..
            } => {
                let anchor = beam_target(ray.origin, ray.direction, distance);
                let target_position = anchor + rotation.0 * local_offset;
                // Sweeping the hand moves the point at the end of a long ray much faster than
                // the palm: feed that velocity forward so the spring tracks the sweep.
                let target_velocity = hand_velocity + hand_spin.cross(anchor - ray.origin);
                (
                    target_position,
                    (spin * beam_rotation(palm_rotation, hand_rotation, prop_rotation)).normalize(),
                    target_velocity,
                )
            }
        };

        let desired_linear = (target_position - position.0) / dt + target_velocity;
        let mass = mass.value().max(1e-6);
        let max_delta_v = MAX_GRAB_FORCE / mass * dt;
        linear.0 = limited_velocity_step(linear.0, desired_linear, max_delta_v);

        let mut delta = target_rotation * rotation.0.inverse();
        if delta.w < 0.0 {
            delta = -delta;
        }
        let (axis, angle) = delta.to_axis_angle();
        let desired_angular = if angle > 1e-5 {
            axis * (angle / dt)
        } else {
            Vec3::ZERO
        };
        let (principal, _) = inertia.principal_angular_inertia_with_local_frame();
        let inertia_scalar = principal.max_element().max(1e-6);
        let max_delta_w = MAX_GRAB_TORQUE / inertia_scalar * dt;
        angular.0 = limited_velocity_step(angular.0, desired_angular, max_delta_w);
    }
}

/// Collision layers that stop the prop touching the holding hand while staying solid
/// for the rest of the world (the table must still block it).
fn held_collision_layers(side: HandSide) -> CollisionLayers {
    let hand_bit = match side {
        HandSide::Left => HAND_LAYER_LEFT,
        HandSide::Right => HAND_LAYER_RIGHT,
    };
    CollisionLayers::new(
        LayerMask::DEFAULT,
        LayerMask(LayerMask::ALL.0 & !hand_bit),
    )
}

fn rumble_hand(side: HandSide) -> RumbleHand {
    match side {
        HandSide::Left => RumbleHand::Left,
        HandSide::Right => RumbleHand::Right,
    }
}

/// Outline + fade. The candidate prop gets its hand-coloured shell; a held prop keeps
/// it solid; after release it fades out over [`OUTLINE_FADE`].
fn update_outlines(
    time: Res<Time>,
    mut state: ResMut<GrabState>,
    materials: Res<OutlineMaterials>,
    mut assets: ResMut<Assets<StandardMaterial>>,
    palms: Query<(&HandBody, &Position, &HandRay), Without<Grabbable>>,
    props: Query<(Entity, &Position, &Rotation, &Collider, &RigidBody), With<Grabbable>>,
    mut outlines: Query<(&GrabOutline, &ChildOf, &mut Visibility)>,
) {
    let dt = time.delta_secs();
    let mut palm_poses = [None; 2];
    for (body, position, ray) in &palms {
        palm_poses[body.side.index()] = Some((position.0, *ray));
    }

    let prop_list = prop_snapshot(&props);
    let mut candidates = [None; 2];
    for side in [HandSide::Left, HandSide::Right] {
        let Some((palm, ray)) = palm_poses[side.index()] else {
            continue;
        };
        let hand = state.hand(side);
        if hand.held.is_some() || hand.fade.is_some() {
            continue;
        }
        // Direct hover wins; otherwise the outline marks what the beam is aimed at.
        candidates[side.index()] = candidate_for(palm, &prop_list)
            .or_else(|| beam_candidate(&ray, &prop_list));
    }
    let (mut left_candidate, mut right_candidate) =
        resolve_claims(candidates[0], candidates[1]);
    let held_left = state.hands[0].held.map(|held| held.entity);
    let held_right = state.hands[1].held.map(|held| held.entity);
    if left_candidate == held_right {
        left_candidate = None;
    }
    if right_candidate == held_left {
        right_candidate = None;
    }

    let mut wanted = [None; 2];
    let mut alpha = [1.0_f32; 2];
    for side in [HandSide::Left, HandSide::Right] {
        let hand = state.hand_mut(side);
        if let Some(held) = hand.held {
            wanted[side.index()] = Some(held.entity);
            continue;
        }
        if let Some((entity, remaining)) = hand.fade {
            let remaining = remaining - dt;
            if remaining > 0.0 {
                hand.fade = Some((entity, remaining));
                wanted[side.index()] = Some(entity);
                alpha[side.index()] = remaining / OUTLINE_FADE;
                continue;
            }
            hand.fade = None;
        }
        wanted[side.index()] = match side {
            HandSide::Left => left_candidate,
            HandSide::Right => right_candidate,
        };
    }

    for side in [HandSide::Left, HandSide::Right] {
        materials.set_alpha(&mut assets, side, alpha[side.index()]);
    }

    for (outline, child_of, mut visibility) in &mut outlines {
        let visible = wanted[outline.side.index()] == Some(child_of.parent());
        *visibility = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Place a unit-height cylinder (radius 0.5, height 1) between two points.
fn segment_transform(transform: &mut Transform, from: Vec3, to: Vec3, radius: f32) -> bool {
    let delta = to - from;
    let length = delta.length();
    if length < 1e-4 {
        return false;
    }
    transform.translation = (from + to) * 0.5;
    transform.rotation = Quat::from_rotation_arc(Vec3::Y, delta / length);
    transform.scale = Vec3::new(radius * 2.0, length, radius * 2.0);
    true
}

/// Draw each hand's beam (DESIGN §7.1): faint and thin while aiming at a grabbable prop,
/// brighter and thicker while holding one, plus a dot on the hit point. The beam is the
/// hand extended, so it uses that hand's colour and never becomes world decoration.
fn update_beams(
    state: Res<GrabState>,
    materials: Res<BeamMaterials>,
    mut assets: ResMut<Assets<StandardMaterial>>,
    hands: Query<(&HandBody, &Position, &Rotation, &HandRay)>,
    props: Query<(&Position, &Rotation), With<Grabbable>>,
    mut visuals: Query<(&BeamVisual, &mut Transform, &mut Visibility)>,
) {
    let mut beams = [BeamPose::default(); 2];
    for (body, position, _, ray) in &hands {
        let side = body.side;
        let beam = &mut beams[side.index()];
        beam.origin = position.0;

        if let Some(held) = state.hand(side).held {
            // A direct grab keeps the M1 look; only a beam hold draws the beam.
            if let HoldKind::Beam { local_offset, .. } = held.hold {
                if let Ok((prop_position, prop_rotation)) = props.get(held.entity) {
                    beam.visible = true;
                    beam.holding = true;
                    beam.end = held_point(prop_position.0, prop_rotation.0, local_offset);
                }
            }
            continue;
        }

        let Some(hit) = ray.hit else {
            continue;
        };
        // Idle aim: only when the ray lands on a grabbable prop the other hand isn't using.
        if props.get(hit.entity).is_err() {
            continue;
        }
        let other = match side {
            HandSide::Left => HandSide::Right,
            HandSide::Right => HandSide::Left,
        };
        if state
            .hand(other)
            .held
            .is_some_and(|held| held.entity == hit.entity)
        {
            continue;
        }
        beam.visible = true;
        beam.end = hit.point;
    }

    for side in [HandSide::Left, HandSide::Right] {
        let beam = beams[side.index()];
        let (alpha, glow) = if !beam.visible {
            (0.0, 0.0)
        } else if beam.holding {
            (1.0, BEAM_GLOW_HOLD)
        } else {
            (BEAM_ALPHA_IDLE, BEAM_GLOW_IDLE)
        };
        let linear = materials.colors[side.index()].to_linear();
        if let Some(mut material) = assets.get_mut(materials.handles[side.index()].id()) {
            material.base_color = Color::srgba(0.0, 0.0, 0.0, alpha);
            material.emissive = LinearRgba::rgb(
                linear.red * glow,
                linear.green * glow,
                linear.blue * glow,
            );
        }
    }

    for (visual, mut transform, mut visibility) in &mut visuals {
        let beam = beams[visual.side.index()];
        if !beam.visible {
            *visibility = Visibility::Hidden;
            continue;
        }
        let (radius, dot_radius) = if beam.holding {
            (BEAM_RADIUS_HOLD, BEAM_DOT_RADIUS_HOLD)
        } else {
            (BEAM_RADIUS_IDLE, BEAM_DOT_RADIUS_IDLE)
        };
        if visual.dot {
            transform.translation = beam.end;
            transform.rotation = Quat::IDENTITY;
            transform.scale = Vec3::splat(dot_radius * 2.0);
            *visibility = Visibility::Visible;
        } else if segment_transform(&mut transform, beam.origin, beam.end, radius) {
            *visibility = Visibility::Visible;
        } else {
            *visibility = Visibility::Hidden;
        }
    }
}

/// Impact pulses: a hand that starts touching a prop buzzes with a short pulse. Rate
/// limited per hand so a prop sliding along the palm doesn't buzz continuously.
fn rumble_on_impact(
    mut state: ResMut<GrabState>,
    mut collisions: MessageReader<CollisionStart>,
    hands: Query<&HandBody>,
    props: Query<&LinearVelocity, With<Grabbable>>,
    motion: Res<HandMotion>,
    mut rumble: MessageWriter<Rumble>,
) {
    for event in collisions.read() {
        // Events are only emitted for colliders with `CollisionEventsEnabled` (hands).
        let Ok(hand) = hands.get(event.collider1) else {
            continue;
        };
        let Ok(prop_velocity) = props.get(event.collider2) else {
            continue;
        };
        let grab = state.hand_mut(hand.side);
        if grab.impact_cooldown > 0.0 {
            continue;
        }
        let speed = (motion.velocity[hand.side.index()] - prop_velocity.0).length();
        if speed < 0.25 {
            continue;
        }
        let t = ((speed - 0.25) / RUMBLE_TOUCH_SPEED).clamp(0.0, 1.0);
        let ms = RUMBLE_TOUCH_MIN_MS
            + ((RUMBLE_TOUCH_MAX_MS - RUMBLE_TOUCH_MIN_MS) as f32 * t) as u32;
        grab.impact_cooldown = RUMBLE_TOUCH_COOLDOWN;
        rumble.write(Rumble {
            hand: rumble_hand(hand.side),
            strength: 0.35 + 0.5 * t,
            ms,
        });
    }
}

fn tick_impact_cooldowns(time: Res<Time>, mut state: ResMut<GrabState>) {
    for hand in &mut state.hands {
        hand.impact_cooldown = (hand.impact_cooldown - time.delta_secs()).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_candidate_picks_closest_within_radius() {
        let candidates = [(1u8, 0.30), (2, 0.08), (3, 0.20)];
        assert_eq!(nearest_candidate(GRAB_RADIUS, candidates), Some((2, 0.08)));
    }

    #[test]
    fn nearest_candidate_ignores_everything_out_of_reach() {
        let candidates = [(1u8, 0.13), (2, 0.40)];
        assert_eq!(nearest_candidate(GRAB_RADIUS, candidates), None);
        assert_eq!(
            nearest_candidate(GRAB_RADIUS, [(1u8, GRAB_RADIUS)]),
            Some((1, GRAB_RADIUS))
        );
    }

    #[test]
    fn shared_prop_goes_to_the_nearer_hand() {
        assert_eq!(
            resolve_claims(Some((7u8, 0.04)), Some((7, 0.09))),
            (Some(7), None)
        );
        assert_eq!(
            resolve_claims(Some((7u8, 0.10)), Some((7, 0.03))),
            (None, Some(7))
        );
        assert_eq!(
            resolve_claims(Some((1u8, 0.04)), Some((2, 0.09))),
            (Some(1), Some(2))
        );
        assert_eq!(resolve_claims(None, Some((2u8, 0.09))), (None, Some(2)));
        assert_eq!(resolve_claims::<u8>(None, None), (None, None));
    }

    #[test]
    fn force_limit_clamps_the_velocity_change() {
        let step = limited_velocity_step(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), 0.5);
        assert!((step - Vec3::new(0.5, 0.0, 0.0)).length() < 1e-6);
        // Already close: no clamp.
        let step = limited_velocity_step(Vec3::ZERO, Vec3::new(0.2, -0.1, 0.0), 0.5);
        assert!((step - Vec3::new(0.2, -0.1, 0.0)).length() < 1e-6);
    }

    #[test]
    fn force_limit_converges_when_unconstrained() {
        let desired = Vec3::new(3.0, -1.0, 0.5);
        let mut velocity = Vec3::ZERO;
        for _ in 0..200 {
            velocity = limited_velocity_step(velocity, desired, 0.05);
        }
        assert!(
            (velocity - desired).length() < 1e-4,
            "should converge to the target, got {velocity}"
        );
    }

    #[test]
    fn throw_averages_samples_scales_and_clamps() {
        let samples = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0)];
        // Average 2 m/s × THROW_GAIN.
        assert!(
            (throw_velocity(&samples, THROW_GAIN, THROW_MAX_SPEED)
                - Vec3::new(2.0 * THROW_GAIN, 0.0, 0.0))
            .length()
                < 1e-5
        );
        let fast = [Vec3::new(20.0, 0.0, 0.0); 5];
        assert!(
            (throw_velocity(&fast, THROW_GAIN, THROW_MAX_SPEED).length() - THROW_MAX_SPEED).abs()
                < 1e-5
        );
        assert_eq!(throw_velocity(&[], THROW_GAIN, THROW_MAX_SPEED), Vec3::ZERO);
    }

    #[test]
    fn grip_half_extent_is_the_surface_distance_from_the_centre() {
        let sphere = Collider::sphere(0.06);
        let half = half_extent_along(&sphere, Vec3::ZERO, Quat::IDENTITY, Vec3::Y);
        assert!((half - 0.06).abs() < 1e-3, "sphere half extent {half}");

        // A 10 cm cube: 5 cm to the face whatever the rotation, along a face normal.
        let cube = Collider::cuboid(0.10, 0.10, 0.10);
        let half = half_extent_along(&cube, Vec3::ZERO, Quat::IDENTITY, Vec3::Y);
        assert!((half - 0.05).abs() < 1e-3, "cube half extent {half}");
        let half = half_extent_along(&cube, Vec3::ZERO, Quat::IDENTITY, Vec3::X);
        assert!((half - 0.05).abs() < 1e-3, "cube half extent along X {half}");

        // And the grip sits under the palm centre, on the palm side (−Y of the hand).
        let grip = grip_offset(0.05);
        assert!((grip - Vec3::new(0.0, -0.05, 0.0)).length() < 1e-6);
    }

    #[test]
    fn outline_scale_keeps_a_constant_shell_thickness() {
        for half_extent in [0.025_f32, 0.05, 0.17, 0.26] {
            let scale = outline_scale(half_extent);
            let thickness = (scale - 1.0) * half_extent;
            assert!(
                (thickness - OUTLINE_THICKNESS).abs() < 1e-5,
                "half extent {half_extent} got {thickness} m"
            );
        }
    }

    // --- M2d: palm beam ------------------------------------------------------

    #[test]
    fn direct_grab_wins_inside_the_palm_radius() {
        assert_eq!(
            select_grab(Some(0.05), Some(2.0), GRAB_RADIUS),
            Some(GrabKind::Direct)
        );
        assert_eq!(
            select_grab(Some(GRAB_RADIUS), Some(1.0), GRAB_RADIUS),
            Some(GrabKind::Direct)
        );
    }

    #[test]
    fn beam_takes_over_outside_the_palm_radius() {
        // Touching something just outside the radius is not a direct grab.
        assert_eq!(
            select_grab(Some(0.30), Some(2.0), GRAB_RADIUS),
            Some(GrabKind::Beam)
        );
        assert_eq!(
            select_grab(None, Some(2.0), GRAB_RADIUS),
            Some(GrabKind::Beam)
        );
        assert_eq!(select_grab(Some(0.30), None, GRAB_RADIUS), None);
        assert_eq!(select_grab(None, None, GRAB_RADIUS), None);
    }

    #[test]
    fn beam_target_is_the_palm_plus_the_ray() {
        let target = beam_target(Vec3::ZERO, Vec3::NEG_Z, 2.0);
        assert!((target - Vec3::new(0.0, 0.0, -2.0)).length() < 1e-6);

        // A non-unit direction must not stretch the hold distance.
        let target = beam_target(Vec3::ZERO, Vec3::NEG_Z * 5.0, 2.0);
        assert!((target - Vec3::new(0.0, 0.0, -2.0)).length() < 1e-6);

        // Zero direction is harmless.
        assert_eq!(beam_target(Vec3::ONE, Vec3::ZERO, 2.0), Vec3::ONE);
    }

    #[test]
    fn push_pull_moves_at_the_tuned_speed() {
        let away = adjust_hold_distance(2.0, 1.0, 1.0);
        assert!((away - (2.0 + BEAM_PUSH_PULL_SPEED)).abs() < 1e-6);
        let closer = adjust_hold_distance(2.0, -1.0, 0.5);
        assert!((closer - (2.0 - BEAM_PUSH_PULL_SPEED * 0.5)).abs() < 1e-6);
        // Diagonal/no input is not allowed to move it.
        assert_eq!(adjust_hold_distance(2.0, 0.0, 0.5), 2.0);
    }

    #[test]
    fn push_pull_clamps_to_the_beam_range() {
        assert_eq!(
            adjust_hold_distance(BEAM_MIN_DISTANCE, -1.0, 100.0),
            BEAM_MIN_DISTANCE
        );
        assert_eq!(
            adjust_hold_distance(BEAM_MAX_DISTANCE, 1.0, 100.0),
            BEAM_MAX_DISTANCE
        );
        assert_eq!(
            adjust_hold_distance(0.4, -5.0, 1.0),
            BEAM_MIN_DISTANCE,
            "the axis is clamped to [-1, 1] too"
        );
    }

    #[test]
    fn beam_rotation_is_the_wrist_delta_applied_to_the_prop() {
        let hand_at_grab = Quat::from_rotation_x(0.2);
        let prop_at_grab = Quat::from_rotation_z(-0.3);
        let hand_now = Quat::from_rotation_y(0.7) * hand_at_grab;

        let target = beam_rotation(hand_now, hand_at_grab, prop_at_grab);
        let expected = hand_now * hand_at_grab.inverse() * prop_at_grab;
        assert!(
            target.dot(expected).abs() > 1.0 - 1e-5,
            "delta rotation mismatch: {target:?} vs {expected:?}"
        );

        // A 90° wrist roll turns the prop exactly 90° around the same axis.
        let target = beam_rotation(
            Quat::from_rotation_z(core::f32::consts::FRAC_PI_2),
            Quat::IDENTITY,
            Quat::IDENTITY,
        );
        let rotated = target * Vec3::X;
        assert!(
            (rotated - Vec3::Y).length() < 1e-5,
            "object X should end up on Y, got {rotated:?}"
        );
    }

    #[test]
    fn prop_modes_cycle_and_freeze_has_no_gravity() {
        let mut mode = PropMode::Normal;
        for _ in 0..3 {
            mode = mode.next();
        }
        assert_eq!(mode, PropMode::Normal);
        assert_eq!(PropMode::Frozen.physics().0, 0.0);
        assert!(PropMode::Frozen.physics().1 > 5.0, "frozen props must stop quickly");
        assert_eq!(PropMode::Normal.physics(), (1.0, 0.0, 0.0));
    }

    #[test]
    fn stick_spin_yaws_around_world_up() {
        // Full right stick for 1 s at 2.5 rad/s: 2.5 rad of yaw, no pitch.
        let mut spin = Quat::IDENTITY;
        for _ in 0..100 {
            spin = beam_spin_step(spin, Vec2::X, Vec3::NEG_Z, 0.01);
        }
        let (axis, angle) = spin.to_axis_angle();
        assert!((angle - BEAM_ROTATE_SPEED).abs() < 1e-3, "angle {angle}");
        assert!(axis.abs().dot(Vec3::Y) > 0.999, "axis {axis}");
    }

    #[test]
    fn grabbed_point_follows_the_prop_rotation() {
        let point = held_point(
            Vec3::new(0.0, 1.0, 0.0),
            Quat::from_rotation_y(core::f32::consts::FRAC_PI_2),
            Vec3::new(1.0, 0.0, 0.0),
        );
        assert!((point - Vec3::new(0.0, 1.0, -1.0)).length() < 1e-5);
    }

    #[test]
    fn push_pull_uses_free_face_buttons() {
        use HandSide::{Left, Right};

        assert_eq!(push_pull_axis(Left, [true, false, false, false]), 1.0);
        assert_eq!(push_pull_axis(Left, [false, true, false, false]), -1.0);
        assert_eq!(push_pull_axis(Right, [true, false, false, false]), 1.0);
        assert_eq!(push_pull_axis(Right, [false, false, true, false]), -1.0);

        // Right face[1] is B (jump) and must never move the beam.
        assert_eq!(push_pull_axis(Right, [false, true, false, false]), 0.0);
        // Both directions cancel out.
        assert_eq!(push_pull_axis(Left, [true, true, false, false]), 0.0);
    }

    #[test]
    fn beam_mesh_spans_its_two_endpoints() {
        let mut transform = Transform::default();
        assert!(segment_transform(
            &mut transform,
            Vec3::ZERO,
            Vec3::new(0.0, 2.0, 0.0),
            0.01
        ));
        assert!((transform.translation - Vec3::Y).length() < 1e-6);
        assert!((transform.scale.y - 2.0).abs() < 1e-6);
        assert!((transform.scale.x - 0.02).abs() < 1e-6);
        assert!(!segment_transform(
            &mut transform,
            Vec3::ONE,
            Vec3::ONE,
            0.01
        ));
    }
}
