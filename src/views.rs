//! Points of view to watch the avatar's arms move.
//!
//! The camera cycles **1ª persona → detrás → frente → lado** (`V` or Home on the Joy-Con R;
//! `JOYREACH_VIEW=behind|front|side` picks the starting view, handy for screenshots). The outside
//! views show a simple body (head + torso) and full-length upper arms; both live on render
//! layer 1, which the first-person camera does not draw, so they never block the first-person
//! view.
//!
//! A picture-in-picture second camera was tried and dropped: in Bevy 0.20 with this scene
//! (atmosphere sky, TAA, SSAO), adding any second 3D camera stopped both from drawing geometry.

use bevy::camera::visibility::RenderLayers;
use bevy::light::NotShadowCaster;
use bevy::prelude::*;

use crate::joycon::Hands;
use crate::player::{EYE_HEIGHT, PlayerBody, PlayerCamera};
use crate::theme;

/// Render layer of the outside-view-only meshes (avatar body, full upper arms).
pub const AVATAR_LAYER: usize = 1;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ViewMode {
    #[default]
    FirstPerson,
    Behind,
    Front,
    Side,
}

impl ViewMode {
    pub fn next(self) -> Self {
        match self {
            ViewMode::FirstPerson => ViewMode::Behind,
            ViewMode::Behind => ViewMode::Front,
            ViewMode::Front => ViewMode::Side,
            ViewMode::Side => ViewMode::FirstPerson,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            ViewMode::FirstPerson => "1ª persona",
            ViewMode::Behind => "Detrás",
            ViewMode::Front => "Frente",
            ViewMode::Side => "Lado",
        }
    }

    fn from_env(value: &str) -> Option<Self> {
        match value {
            "behind" | "detras" => Some(ViewMode::Behind),
            "front" | "frente" => Some(ViewMode::Front),
            "side" | "lado" => Some(ViewMode::Side),
            "first" | "1p" => Some(ViewMode::FirstPerson),
            _ => None,
        }
    }
}

#[derive(Resource, Debug, Default)]
pub struct Views {
    pub mode: ViewMode,
    prev_home: bool,
}

/// Camera transform in the player-body frame for a view mode and camera pitch.
pub fn camera_local(mode: ViewMode, pitch: f32) -> Transform {
    let head = Vec3::Y * EYE_HEIGHT;
    let look = Quat::from_rotation_x(pitch);
    match mode {
        ViewMode::FirstPerson => Transform::from_translation(head).with_rotation(look),
        // Over the right shoulder, looking where the player looks.
        ViewMode::Behind => {
            Transform::from_translation(head + look * Vec3::new(0.35, 0.25, 2.1)).with_rotation(look)
        }
        // Facing the player from the front-left, framing head, torso and both arms.
        ViewMode::Front => Transform::from_translation(head + Vec3::new(-0.6, 0.1, -1.9))
            .looking_at(head + Vec3::new(0.0, -0.35, -0.25), Vec3::Y),
        ViewMode::Side => Transform::from_translation(head + Vec3::new(1.8, 0.05, -0.45))
            .looking_at(head + Vec3::new(0.0, -0.3, -0.35), Vec3::Y),
    }
}

pub struct ViewsPlugin;

impl Plugin for ViewsPlugin {
    fn build(&self, app: &mut App) {
        let mode = std::env::var("JOYREACH_VIEW")
            .ok()
            .and_then(|value| ViewMode::from_env(&value))
            .unwrap_or_default();
        app.insert_resource(Views {
            mode,
            prev_home: false,
        })
        .add_systems(Startup, spawn_avatar.after(crate::player::spawn_player))
        .add_systems(
            Update,
            (view_input, apply_view_layers)
                .chain()
                .before(crate::player::PlayerSet),
        );
    }
}

fn spawn_avatar(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    body: Single<Entity, With<PlayerBody>>,
) {
    let shirt = materials.add(StandardMaterial {
        base_color: theme::CHARCOAL,
        perceptual_roughness: 0.85,
        ..default()
    });
    let skin = materials.add(StandardMaterial {
        base_color: theme::GLOVE,
        perceptual_roughness: 0.7,
        ..default()
    });
    let layer = RenderLayers::layer(AVATAR_LAYER);
    let head = Vec3::Y * EYE_HEIGHT;

    commands.entity(*body).with_children(|parent| {
        // Head, centred just behind the eyes so the first-person camera is inside it.
        parent.spawn((
            Mesh3d(meshes.add(Sphere::new(0.11).mesh().uv(24, 16))),
            MeshMaterial3d(skin.clone()),
            Transform::from_translation(head + Vec3::new(0.0, 0.02, 0.06)),
            layer.clone(),
            NotShadowCaster,
        ));
        // Neck.
        parent.spawn((
            Mesh3d(meshes.add(Capsule3d::new(0.045, 0.08))),
            MeshMaterial3d(skin),
            Transform::from_translation(head + Vec3::new(0.0, -0.14, 0.07)),
            layer.clone(),
            NotShadowCaster,
        ));
        // Torso: shoulders at the arm model's shoulder height (~0.12 m below the eyes).
        parent.spawn((
            Mesh3d(meshes.add(Capsule3d::new(0.17, 0.36))),
            MeshMaterial3d(shirt),
            Transform::from_translation(head + Vec3::new(0.0, -0.45, 0.08))
                .with_scale(Vec3::new(1.15, 1.0, 0.7)),
            layer,
            NotShadowCaster,
        ));
    });
}

fn view_input(keys: Res<ButtonInput<KeyCode>>, hands: Res<Hands>, mut views: ResMut<Views>) {
    let home = hands.right.connected && hands.right.extra;
    let home_pressed = home && !views.prev_home;
    views.prev_home = home;
    if keys.just_pressed(KeyCode::KeyV) || home_pressed {
        views.mode = views.mode.next();
        info!("view: {}", views.mode.label());
    }
}

/// First person draws only the world layer; outside views also draw the avatar layer.
fn apply_view_layers(
    mut commands: Commands,
    views: Res<Views>,
    main: Single<(Entity, Option<&RenderLayers>), With<PlayerCamera>>,
) {
    let (main_entity, main_layers) = *main;
    let wanted = if views.mode == ViewMode::FirstPerson {
        RenderLayers::layer(0)
    } else {
        RenderLayers::from_layers(&[0, AVATAR_LAYER])
    };
    if main_layers != Some(&wanted) {
        commands.entity(main_entity).insert(wanted);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn view_modes_cycle_back_to_first_person() {
        let mut mode = ViewMode::FirstPerson;
        for _ in 0..4 {
            mode = mode.next();
        }
        assert_eq!(mode, ViewMode::FirstPerson);
    }

    #[test]
    fn first_person_camera_sits_at_the_eyes() {
        let t = camera_local(ViewMode::FirstPerson, 0.3);
        assert!((t.translation - Vec3::Y * EYE_HEIGHT).length() < 1e-6);
    }

    #[test]
    fn outside_views_look_at_the_player() {
        for mode in [ViewMode::Behind, ViewMode::Front, ViewMode::Side] {
            let t = camera_local(mode, 0.0);
            let to_head = (Vec3::Y * EYE_HEIGHT - t.translation).normalize();
            // The head is within ~45° of the camera's forward axis.
            assert!(t.forward().dot(to_head) > 0.7, "{mode:?} does not frame the player");
        }
    }

    #[test]
    fn env_names_pick_views() {
        assert_eq!(ViewMode::from_env("front"), Some(ViewMode::Front));
        assert_eq!(ViewMode::from_env("lado"), Some(ViewMode::Side));
        assert_eq!(ViewMode::from_env("nope"), None);
    }
}
