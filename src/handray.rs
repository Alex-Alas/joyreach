//! M2d palm ray: a generic orientation-driven ray from each palm.
//!
//! The ray is a component on the kinematic hand body, so anything that already follows a hand
//! (`grab`, and later the M3 desktop-window pointer) can query it next to the palm pose.
//! `trace_hand_rays` refreshes it in the fixed step, after the hand bodies have been driven,
//! so the physics-gun grab sees exactly the palm frame the direct-grab radius test uses.
//!
//! The ray casts against the default collision layer (props, bench, deck) and excludes the
//! player capsule and both hand colliders. It stores the first hit entity, point, distance and
//! normal; [`HandRay::hit`] is `None` when the 8 m ray reaches nothing.

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::arms::HandBody;
use crate::grab::Grabbable;
use crate::player::PlayerCapsule;

/// Palm-ray range (DESIGN §7: the beam is clamped to 8 m).
pub const HAND_RAY_MAX: f32 = 8.0;
/// Aim assist: when the exact ray misses every grabbable prop, a prop whose centre lies within
/// this cone around the ray (and in clear line of sight) is taken instead. Joy-Con aim at 2-3 m
/// wobbles by a couple of degrees, and the props are 5-15 cm.
pub const AIM_ASSIST_DEG: f32 = 8.0;
/// ...or within this perpendicular distance of the ray (m), whichever is larger, so close props
/// are not missed by a few centimetres.
pub const AIM_ASSIST_RADIUS: f32 = 0.10;

/// Aim-assist score of a target centre for a ray: `Some(angle)` (radians) if it is in the cone
/// (or within the radius of the ray) and in front of the palm, else `None`.
pub fn assist_angle(origin: Vec3, direction: Vec3, center: Vec3, max_distance: f32) -> Option<f32> {
    let to_center = center - origin;
    let along = to_center.dot(direction);
    if along <= 0.0 || along > max_distance {
        return None;
    }
    let perpendicular = (to_center - direction * along).length();
    let angle = perpendicular.atan2(along);
    (angle <= AIM_ASSIST_DEG.to_radians() || perpendicular <= AIM_ASSIST_RADIUS).then_some(angle)
}

/// The first collider a [`HandRay`] reached.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandRayHit {
    /// Collider entity that was hit.
    pub entity: Entity,
    /// World-space intersection point.
    pub point: Vec3,
    /// Distance from the palm to [`HandRayHit::point`], metres.
    pub distance: f32,
    /// World-space surface normal at the hit.
    pub normal: Vec3,
}

/// One hand's pointing ray. `direction` is unit length and points out of the fingers
/// (the Joy-Con top, local −Z of the hand), so it is also the M3 window pointer.
#[derive(Component, Clone, Copy, Debug)]
pub struct HandRay {
    pub origin: Vec3,
    pub direction: Vec3,
    pub max_distance: f32,
    pub hit: Option<HandRayHit>,
}

impl HandRay {
    pub fn new(max_distance: f32) -> Self {
        Self {
            origin: Vec3::ZERO,
            direction: Vec3::NEG_Z,
            max_distance,
            hit: None,
        }
    }

    /// World-space point the ray ended at, if it hit anything.
    #[allow(dead_code)] // M3's window pointer reads the point without unpacking the hit.
    pub fn point(&self) -> Option<Vec3> {
        self.hit.map(|hit| hit.point)
    }

    /// Entity the ray ended at, if it hit anything.
    #[allow(dead_code)] // M3's window pointer reads the entity without unpacking the hit.
    pub fn hit_entity(&self) -> Option<Entity> {
        self.hit.map(|hit| hit.entity)
    }
}

impl Default for HandRay {
    fn default() -> Self {
        Self::new(HAND_RAY_MAX)
    }
}

/// The hand's pointing direction in world space. Identity orientation points at the screen
/// (−Z), exactly like the neutral remote grip.
pub fn pointing_direction(hand_rotation: Quat) -> Vec3 {
    (hand_rotation * Vec3::NEG_Z).normalize_or_zero()
}

/// Ray refresh runs here, after [`crate::arms::HandBodiesSet`].
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct HandRaySet;

pub struct HandRayPlugin;

impl Plugin for HandRayPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            trace_hand_rays
                .in_set(HandRaySet)
                .after(crate::arms::HandBodiesSet),
        );
    }
}

fn trace_hand_rays(
    spatial_query: SpatialQuery,
    player: Single<Entity, With<PlayerCapsule>>,
    mut hands: Query<(Entity, &HandBody, &Position, &Rotation, &mut HandRay)>,
    grabbables: Query<(Entity, &Position), (With<Grabbable>, Without<HandBody>)>,
) {
    let mut excluded = Vec::with_capacity(3);
    excluded.push(*player);
    for (entity, ..) in &hands {
        excluded.push(entity);
    }
    let filter = SpatialQueryFilter::from_mask(LayerMask::DEFAULT)
        .with_excluded_entities(excluded.iter().copied());

    for (_, _, position, rotation, mut ray) in &mut hands {
        ray.origin = position.0;
        ray.direction = pointing_direction(rotation.0);
        ray.hit = None;

        let Ok(direction) = Dir3::new(ray.direction) else {
            continue;
        };
        if let Some(hit) = spatial_query.cast_ray(
            ray.origin,
            direction,
            ray.max_distance,
            true,
            &filter,
        ) {
            ray.hit = Some(HandRayHit {
                entity: hit.entity,
                point: ray.origin + ray.direction * hit.distance,
                distance: hit.distance,
                normal: hit.normal,
            });
        }

        // Aim assist: the exact ray landed on something that is not a prop (or on nothing).
        if ray.hit.is_some_and(|hit| grabbables.contains(hit.entity)) {
            continue;
        }
        let reach = ray.hit.map_or(ray.max_distance, |hit| hit.distance + 0.5);
        let best = grabbables
            .iter()
            .filter_map(|(entity, center)| {
                assist_angle(ray.origin, ray.direction, center.0, reach)
                    .map(|angle| (entity, center.0, angle))
            })
            .min_by(|a, b| a.2.total_cmp(&b.2));
        let Some((entity, center, _)) = best else {
            continue;
        };
        // Line of sight: the first thing toward the prop's centre must be the prop itself.
        let Ok(toward) = Dir3::new(center - ray.origin) else {
            continue;
        };
        if let Some(hit) =
            spatial_query.cast_ray(ray.origin, toward, ray.max_distance, true, &filter)
        {
            if hit.entity == entity {
                ray.hit = Some(HandRayHit {
                    entity,
                    point: ray.origin + toward * hit.distance,
                    distance: hit.distance,
                    normal: hit.normal,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_hand_points_at_the_screen() {
        assert!((pointing_direction(Quat::IDENTITY) - Vec3::NEG_Z).length() < 1e-6);
    }

    #[test]
    fn pointing_direction_follows_yaw() {
        // Yawing the Joy-Con 90° left (positive Y) turns the fingers toward −X.
        let direction = pointing_direction(Quat::from_rotation_y(90.0_f32.to_radians()));
        assert!(
            (direction - Vec3::NEG_X).length() < 1e-5,
            "got {direction:?}"
        );
    }

    #[test]
    fn pointing_direction_survives_roll() {
        // Roll is a twist around the pointing axis: the beam direction must not move.
        let neutral = pointing_direction(Quat::IDENTITY);
        let rolled = pointing_direction(Quat::from_rotation_z(70.0_f32.to_radians()));
        assert!((neutral - rolled).length() < 1e-5);
    }

    #[test]
    fn aim_assist_takes_props_near_the_ray_only() {
        let origin = Vec3::ZERO;
        let forward = Vec3::NEG_Z;
        // 2 m ahead, 20 cm off the ray: ~5.7°, inside the 8° cone.
        assert!(assist_angle(origin, forward, Vec3::new(0.2, 0.0, -2.0), 8.0).is_some());
        // 2 m ahead, 40 cm off: ~11°, outside.
        assert!(assist_angle(origin, forward, Vec3::new(0.4, 0.0, -2.0), 8.0).is_none());
        // Close: 30 cm ahead, 8 cm off is ~15° but within the 10 cm radius.
        assert!(assist_angle(origin, forward, Vec3::new(0.08, 0.0, -0.3), 8.0).is_some());
        // Behind the palm or beyond the range: never.
        assert!(assist_angle(origin, forward, Vec3::new(0.0, 0.0, 1.0), 8.0).is_none());
        assert!(assist_angle(origin, forward, Vec3::new(0.0, 0.0, -9.0), 8.0).is_none());
    }

    #[test]
    fn new_ray_is_empty_and_eight_metres_long() {
        let ray = HandRay::default();
        assert_eq!(ray.max_distance, HAND_RAY_MAX);
        assert_eq!(ray.hit, None);
        assert_eq!(ray.point(), None);
        assert_eq!(ray.hit_entity(), None);
    }
}
