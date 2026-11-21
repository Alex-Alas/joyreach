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
use crate::player::PlayerCapsule;

/// Palm-ray range (DESIGN §7: the beam is clamped to 8 m).
pub const HAND_RAY_MAX: f32 = 8.0;

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
    fn new_ray_is_empty_and_eight_metres_long() {
        let ray = HandRay::default();
        assert_eq!(ray.max_distance, HAND_RAY_MAX);
        assert_eq!(ray.hit, None);
        assert_eq!(ray.point(), None);
        assert_eq!(ray.hit_entity(), None);
    }
}
