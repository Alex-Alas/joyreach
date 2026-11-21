//! M1 grab & throw: ZL/ZR closes a hand on the nearest prop, the prop is driven by a
//! force-limited velocity tracker so the table and other props can block it, and the
//! release throws it with the hand's recent velocity.
//!
//! Also here: the hand-coloured grab outline (DESIGN §8) and the HD Rumble triggers
//! (DESIGN §9). The outline is an inverted hull (back faces of a slightly larger copy
//! of the prop mesh, unlit) — one child per hand so each hand owns its colour.
//!
//! The pure decision/kinematics helpers at the top are unit tested; the ECS systems
//! below only feed them.

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::render::render_resource::Face;

use crate::arms::{HAND_LAYER_LEFT, HAND_LAYER_RIGHT, HandBody, HandMotion, HandSide};
use crate::joycon::{Hands, Rumble, RumbleHand};
use crate::theme;

// --- Tuning (all distances in metres, forces in newtons) ----------------------

/// A prop is a candidate when its collider surface is this close to the palm point.
pub const GRAB_RADIUS: f32 = 0.12;
/// Cap on the force the hand may use to hold a prop. Acceleration = force / mass, so
/// a heavier prop lags behind the hand while a light one tracks it tightly.
pub const MAX_GRAB_FORCE: f32 = 400.0;
/// Cap on the torque used to twist a held prop (N·m).
pub const MAX_GRAB_TORQUE: f32 = 8.0;
/// Throw speed = average hand velocity over the last [`THROW_SAMPLES`] fixed steps × gain.
/// The 3DoF arm model moves the hand far less than a real one, hence the gain.
pub const THROW_GAIN: f32 = 1.6;
pub const THROW_MAX_SPEED: f32 = 12.0;
pub const THROW_MAX_SPIN: f32 = 12.0;
pub const THROW_SAMPLES: usize = 5;
/// Extra outline shell thickness, in world metres, used to pick each prop's scale.
pub const OUTLINE_THICKNESS: f32 = 0.004;
/// Outline fade after release (DESIGN §8: 150 ms).
pub const OUTLINE_FADE: f32 = 0.15;

/// HD Rumble feel. Strength is 0..1.
const RUMBLE_GRAB: (f32, u32) = (0.9, 40);
const RUMBLE_THROW: (f32, u32) = (0.55, 15);
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
struct HeldProp {
    entity: Entity,
    /// Prop centre in hand space, captured at grab time.
    offset: Vec3,
    /// Prop orientation in hand space, captured at grab time.
    rotation_offset: Quat,
}

#[derive(Clone, Copy, Debug)]
struct HandGrab {
    held: Option<HeldProp>,
    prev_trigger: bool,
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

// --- ECS ----------------------------------------------------------------------

pub struct GrabPlugin;

impl Plugin for GrabPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GrabState>()
            .add_systems(
                Update,
                (update_outlines, rumble_on_impact, tick_impact_cooldowns).chain(),
            )
            .add_systems(FixedUpdate, drive_grabs.after(crate::arms::HandBodiesSet));
    }
}

/// Snapshot the props so both hands test the same list in one pass.
fn prop_snapshot(
    props: &Query<(Entity, &Position, &Rotation, &Collider), With<Grabbable>>,
) -> Vec<(Entity, Vec3, Quat, Collider)> {
    props
        .iter()
        .map(|(entity, position, rotation, collider)| {
            (entity, position.0, rotation.0, collider.clone())
        })
        .collect()
}

/// Best candidate for one hand, shared by the outline (Update) and the grab (fixed).
fn candidate_for(palm: Vec3, props: &[(Entity, Vec3, Quat, Collider)]) -> Option<(Entity, f32)> {
    nearest_candidate(
        GRAB_RADIUS,
        props.iter().map(|(entity, position, rotation, collider)| {
            (*entity, palm_to_prop(palm, *position, *rotation, collider))
        }),
    )
}

/// Per fixed step: input edges, force-limited tracking of the held prop, throw.
fn drive_grabs(
    mut commands: Commands,
    time: Res<Time<Fixed>>,
    hands: Res<Hands>,
    fake: Option<Res<crate::FakeHands>>,
    motion: Res<HandMotion>,
    mut state: ResMut<GrabState>,
    palms: Query<(&HandBody, &Position, &Rotation), Without<Grabbable>>,
    mut props: Query<
        (
            &Position,
            &Rotation,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &ComputedMass,
            &ComputedAngularInertia,
            &Collider,
        ),
        With<Grabbable>,
    >,
    prop_ids: Query<(Entity, &Position, &Rotation, &Collider), With<Grabbable>>,
    mut rumble: MessageWriter<Rumble>,
) {
    let dt = time.delta_secs().max(1e-5);
    // The fixed step runs before `Update`, so fake mode must read the mirrored snapshot.
    let hands: &Hands = match fake.as_ref() {
        Some(fake) => &fake.0,
        None => &hands,
    };

    let mut palm_poses = [None; 2];
    for (body, position, rotation) in &palms {
        palm_poses[body.side.index()] = Some((position.0, rotation.0));
    }

    // --- Candidate selection (one prop per hand, nearest hand wins) ----------
    let prop_list = prop_snapshot(&prop_ids);
    let mut candidates = [None; 2];
    for side in [HandSide::Left, HandSide::Right] {
        let Some((palm, _)) = palm_poses[side.index()] else {
            continue;
        };
        candidates[side.index()] = candidate_for(palm, &prop_list);
    }
    let (left_candidate, right_candidate) = resolve_claims(candidates[0], candidates[1]);

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

        let Some((palm, palm_rotation)) = palm_poses[side.index()] else {
            continue;
        };

        if just_pressed && hand.held.is_none() {
            let candidate = match side {
                HandSide::Left => left_candidate,
                HandSide::Right => right_candidate,
            };
            if let Some(entity) = candidate {
                if let Ok((position, rotation, ..)) = props.get(entity) {
                    hand.held = Some(HeldProp {
                        entity,
                        offset: palm_rotation.inverse() * (position.0 - palm),
                        rotation_offset: palm_rotation.inverse() * rotation.0,
                    });
                    commands
                        .entity(entity)
                        .insert(SleepingDisabled)
                        .insert(held_collision_layers(side));
                    let (strength, ms) = RUMBLE_GRAB;
                    rumble.write(Rumble {
                        hand: rumble_hand(side),
                        strength,
                        ms,
                    });
                    info!(
                        "grab {:?}: prop {entity:?} at {:.3} m",
                        side,
                        (position.0 - palm).length()
                    );
                }
            }
        }

        let Some(held) = hand.held else {
            continue;
        };

        if just_released {
            // Throw: the hand's recent velocity, gained and clamped.
            let linear = throw_velocity(&hand.take_linear(), THROW_GAIN, THROW_MAX_SPEED);
            let angular = throw_velocity(&hand.take_angular(), THROW_GAIN, THROW_MAX_SPIN);
            if let Ok((_, _, mut prop_linear, mut prop_angular, ..)) = props.get_mut(held.entity) {
                prop_linear.0 = linear;
                prop_angular.0 = angular;
            }
            commands
                .entity(held.entity)
                .remove::<SleepingDisabled>()
                .insert(CollisionLayers::DEFAULT);
            hand.held = None;
            hand.fade = Some((held.entity, OUTLINE_FADE));
            let (strength, ms) = RUMBLE_THROW;
            rumble.write(Rumble {
                hand: rumble_hand(side),
                strength,
                ms,
            });
            info!("throw {:?}: {:.2} m/s", side, linear.length());
            continue;
        }

        // --- Force-limited velocity tracking --------------------------------
        let Ok((position, rotation, mut linear, mut angular, mass, inertia, _)) =
            props.get_mut(held.entity)
        else {
            // The prop disappeared: drop the grab.
            hand.held = None;
            continue;
        };

        let target_position = palm + palm_rotation * held.offset;
        let target_rotation = (palm_rotation * held.rotation_offset).normalize();

        let desired_linear = (target_position - position.0) / dt + hand_velocity;
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
    palms: Query<(&HandBody, &Position), Without<Grabbable>>,
    props: Query<(Entity, &Position, &Rotation, &Collider), With<Grabbable>>,
    mut outlines: Query<(&GrabOutline, &ChildOf, &mut Visibility)>,
) {
    let dt = time.delta_secs();
    let mut palm_poses = [None; 2];
    for (body, position) in &palms {
        palm_poses[body.side.index()] = Some(position.0);
    }

    let prop_list = prop_snapshot(&props);
    let mut candidates = [None; 2];
    for side in [HandSide::Left, HandSide::Right] {
        let Some(palm) = palm_poses[side.index()] else {
            continue;
        };
        let hand = state.hand(side);
        if hand.held.is_some() || hand.fade.is_some() {
            continue;
        }
        candidates[side.index()] = candidate_for(palm, &prop_list);
    }
    let (left_candidate, right_candidate) = resolve_claims(candidates[0], candidates[1]);

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
        // Average 2 m/s × 1.6 = 3.2 m/s.
        assert!(
            (throw_velocity(&samples, THROW_GAIN, THROW_MAX_SPEED)
                - Vec3::new(3.2, 0.0, 0.0))
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
}
