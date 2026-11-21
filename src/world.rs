//! M0 world: a calm wooden rooftop terrace, the work table and its physics props.
//!
//! Layout (metres, spawn at the origin looking toward -Z):
//! - deck 30×30 m, top surface at y = 0, low celadon parapets on the four edges
//! - walnut table 0.9 m in front, top at 0.75 m
//! - ten small dynamic props on the table, a crate stack to the right,
//!   three large light balls on the deck
//! - bevy `Atmosphere` sky + one warm sun at ~55° elevation

use avian3d::prelude::*;
use bevy::light::atmosphere::ScatteringMedium;
use bevy::light::{Atmosphere, CascadeShadowConfigBuilder, SunDisk, light_consts::lux};
use bevy::prelude::*;

use crate::grab::{Grabbable, OutlineMaterials, spawn_outlines};
use crate::theme;

/// Half extent of the walkable deck. The player is clamped a little inside this.
pub const TERRACE_HALF: f32 = 15.0;
/// Height of the workbench top surface (standing height, so the neutral hands
/// hover over its front half).
pub const BENCH_TOP_Y: f32 = 0.95;
/// Near edge of the bench (closest to the spawn point).
pub const BENCH_FRONT_Z: f32 = -0.25;
/// Far edge of the bench.
pub const BENCH_BACK_Z: f32 = -0.85;
/// Half width of the bench.
pub const BENCH_HALF_WIDTH: f32 = 0.85;
/// Bench centre on Z.
const BENCH_Z: f32 = (BENCH_FRONT_Z + BENCH_BACK_Z) * 0.5;
/// Bench depth (positive; front edge is the larger Z).
const BENCH_DEPTH: f32 = BENCH_FRONT_Z - BENCH_BACK_Z;

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GlobalAmbientLight::NONE)
            .insert_resource(ClearColor(Color::srgb(0.66, 0.75, 0.84)))
            .add_plugins(PhysicsPlugins::default())
            .add_systems(Startup, setup_world);
    }
}

fn wood_material(base: Color, roughness: f32) -> StandardMaterial {
    StandardMaterial {
        base_color: base,
        perceptual_roughness: roughness,
        reflectance: 0.12,
        ..default()
    }
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scattering_mediums: ResMut<Assets<ScatteringMedium>>,
) {
    // --- Sky -----------------------------------------------------------------
    // The Atmosphere entity positions itself one planet-radius below the origin,
    // so its normal is +Y around our scene. Do not give it a Transform.
    let earth = scattering_mediums.add(ScatteringMedium::earth(256, 256));
    commands.spawn(Atmosphere::earth(earth));

    // --- Sun: 55° elevation, warm, coming from behind-right so shadows fall
    // away from the camera and read depth on the bench and deck. --------------
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.91, 0.78),
            illuminance: lux::RAW_SUNLIGHT * 0.85,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-0.38, -0.82, -0.42), Vec3::Y),
        CascadeShadowConfigBuilder {
            first_cascade_far_bound: 2.5,
            maximum_distance: 55.0,
            ..default()
        }
        .build(),
        SunDisk::EARTH,
    ));

    // --- Materials -----------------------------------------------------------
    let deck = materials.add(wood_material(theme::DECK, 0.82));
    let deck_light = materials.add(wood_material(theme::DECK_LIGHT, 0.8));
    let deck_dark = materials.add(wood_material(theme::DECK_DARK, 0.85));
    let walnut = materials.add(StandardMaterial {
        base_color: theme::WALNUT,
        perceptual_roughness: 0.55,
        reflectance: 0.2,
        ..default()
    });
    let celadon = materials.add(StandardMaterial {
        base_color: theme::CELADON,
        perceptual_roughness: 0.9,
        ..default()
    });
    let crate_wood = materials.add(wood_material(Color::srgb_u8(0x9A, 0x74, 0x4E), 0.75));

    let mustard = materials.add(StandardMaterial {
        base_color: theme::MUSTARD,
        perceptual_roughness: 0.62,
        ..default()
    });
    let sage = materials.add(StandardMaterial {
        base_color: theme::SAGE,
        perceptual_roughness: 0.7,
        ..default()
    });
    let plum = materials.add(StandardMaterial {
        base_color: theme::PLUM,
        perceptual_roughness: 0.66,
        ..default()
    });
    let stone = materials.add(StandardMaterial {
        base_color: theme::STONE,
        perceptual_roughness: 0.72,
        ..default()
    });
    let charcoal = materials.add(StandardMaterial {
        base_color: theme::CHARCOAL,
        perceptual_roughness: 0.5,
        ..default()
    });

    // Grab outlines: one shared unlit material per hand colour (DESIGN §8).
    let outlines = OutlineMaterials::new(&mut materials);
    commands.insert_resource(outlines.clone());

    // --- Deck: a thin slab plus 30 m long planks with 4 cm seams ---------------
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(30.0, 0.33, 30.0))),
        MeshMaterial3d(deck_dark.clone()),
        Transform::from_xyz(0.0, -0.175, 0.0),
        RigidBody::Static,
        Collider::cuboid(30.0, 0.35, 30.0),
        Friction::new(0.95),
    ));

    let plank_mesh = meshes.add(Cuboid::new(30.0, 0.02, 0.56));
    for i in 0..50 {
        let z = -14.7 + i as f32 * 0.6;
        let material = match i % 6 {
            0 => deck_dark.clone(),
            3 => deck_light.clone(),
            _ => deck.clone(),
        };
        commands.spawn((
            Mesh3d(plank_mesh.clone()),
            MeshMaterial3d(material),
            Transform::from_xyz(0.0, -0.01, z),
        ));
    }

    // --- Parapets: low mint walls, the edges double as the walkable boundary ---
    let wall_x_mesh = meshes.add(Cuboid::new(30.0, 0.55, 0.24));
    let wall_z_mesh = meshes.add(Cuboid::new(0.24, 0.55, 30.0));
    for (mesh, collider, pos) in [
        (
            wall_x_mesh.clone(),
            Collider::cuboid(30.0, 0.55, 0.24),
            Vec3::new(0.0, 0.275, -14.88),
        ),
        (
            wall_x_mesh.clone(),
            Collider::cuboid(30.0, 0.55, 0.24),
            Vec3::new(0.0, 0.275, 14.88),
        ),
        (
            wall_z_mesh.clone(),
            Collider::cuboid(0.24, 0.55, 30.0),
            Vec3::new(-14.88, 0.275, 0.0),
        ),
        (
            wall_z_mesh.clone(),
            Collider::cuboid(0.24, 0.55, 30.0),
            Vec3::new(14.88, 0.275, 0.0),
        ),
    ] {
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(celadon.clone()),
            Transform::from_translation(pos),
            RigidBody::Static,
            collider,
            Friction::new(0.8),
        ));
    }

    // --- Workbench: standing height, near edge 0.25 m in front of the spawn --
    let bench_top = meshes.add(Cuboid::new(
        BENCH_HALF_WIDTH * 2.0,
        0.07,
        BENCH_DEPTH,
    ));
    commands.spawn((
        Mesh3d(bench_top),
        MeshMaterial3d(walnut.clone()),
        Transform::from_xyz(0.0, BENCH_TOP_Y - 0.035, BENCH_Z),
        RigidBody::Static,
        Collider::cuboid(BENCH_HALF_WIDTH * 2.0, 0.07, BENCH_DEPTH),
        Friction::new(0.7),
    ));
    let leg_mesh = meshes.add(Cuboid::new(0.07, 0.88, 0.07));
    for x in [-0.78, 0.78] {
        for z in [BENCH_FRONT_Z - 0.09, BENCH_BACK_Z + 0.09] {
            commands.spawn((
                Mesh3d(leg_mesh.clone()),
                MeshMaterial3d(walnut.clone()),
                Transform::from_xyz(x, 0.44, z),
                RigidBody::Static,
                Collider::cuboid(0.07, 0.88, 0.07),
            ));
        }
    }
    // A slim stretcher keeps the bench from looking like four sticks.
    let stretcher = meshes.add(Cuboid::new(1.5, 0.05, 0.05));
    for z in [BENCH_FRONT_Z - 0.09, BENCH_BACK_Z + 0.09] {
        commands.spawn((
            Mesh3d(stretcher.clone()),
            MeshMaterial3d(walnut.clone()),
            Transform::from_xyz(0.0, 0.36, z),
        ));
    }

    // --- Ten small props on the bench (8–15 cm, hand sized) -------------------
    spawn_bench_props(
        &mut commands,
        &mut meshes,
        [
            mustard.clone(),
            sage.clone(),
            plum.clone(),
            stone.clone(),
            charcoal.clone(),
        ],
        &outlines,
    );

    // --- Crate stack to the right of the bench -------------------------------
    let crate_mesh = meshes.add(Cuboid::new(0.34, 0.34, 0.34));
    for (pos, yaw) in [
        (Vec3::new(1.05, 0.172, -1.02), 0.0),
        (Vec3::new(1.05, 0.512, -1.02), 0.04),
        (Vec3::new(1.42, 0.172, -1.24), -0.05),
    ] {
        let crate_entity = commands
            .spawn((
                Mesh3d(crate_mesh.clone()),
                MeshMaterial3d(crate_wood.clone()),
                Transform::from_translation(pos).with_rotation(Quat::from_rotation_y(yaw)),
                RigidBody::Dynamic,
                Collider::cuboid(0.34, 0.34, 0.34),
                Friction::new(0.85),
                Restitution::new(0.05),
                Grabbable,
            ))
            .id();
        spawn_outlines(&mut commands, crate_entity, crate_mesh.clone(), 0.17, &outlines);
    }

    // --- Three large light balls on the deck ---------------------------------
    let ball_mesh = meshes.add(Sphere::new(0.26).mesh().uv(28, 18));
    for (pos, material) in [
        (Vec3::new(-1.02, 0.262, -1.55), stone.clone()),
        (Vec3::new(0.88, 0.262, -1.80), stone.clone()),
        (Vec3::new(0.1, 0.262, -2.42), mustard.clone()),
    ] {
        let ball_entity = commands
            .spawn((
                Mesh3d(ball_mesh.clone()),
                MeshMaterial3d(material),
                Transform::from_translation(pos),
                RigidBody::Dynamic,
                Collider::sphere(0.26),
                Friction::new(0.6),
                Restitution::new(0.25),
                Grabbable,
            ))
            .id();
        spawn_outlines(&mut commands, ball_entity, ball_mesh.clone(), 0.26, &outlines);
    }

    // --- Planters with clipped shrubs, far enough to give the horizon scale ---
    let planter_mesh = meshes.add(Cuboid::new(1.8, 0.45, 0.7));
    let shrub_mesh = meshes.add(Sphere::new(0.5).mesh().uv(20, 12));
    for (x, z) in [(-6.4, -10.5), (6.6, -9.6)] {
        commands.spawn((
            Mesh3d(planter_mesh.clone()),
            MeshMaterial3d(celadon.clone()),
            Transform::from_xyz(x, 0.225, z),
            RigidBody::Static,
            Collider::cuboid(1.8, 0.45, 0.7),
        ));
        for (dx, dz, s) in [(-0.45, 0.0, 1.0), (0.15, 0.08, 1.15), (0.55, -0.05, 0.85)] {
            commands.spawn((
                Mesh3d(shrub_mesh.clone()),
                MeshMaterial3d(sage.clone()),
                Transform::from_xyz(x + dx, 0.72, z + dz)
                    .with_scale(Vec3::new(0.85 * s, 0.75 * s, 0.85 * s)),
            ));
        }
    }
}

/// Ten dynamic props on the bench, packed within ±0.40 m of the spawn axis and
/// 0.30–0.60 m in front so both hands have targets at rest.
fn spawn_bench_props(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    [mustard, sage, plum, stone, charcoal]: [Handle<StandardMaterial>; 5],
    outlines: &OutlineMaterials,
) {
    let surface = BENCH_TOP_Y + 0.004;

    type Prop = (
        f32,
        f32,
        f32,
        Handle<Mesh>,
        Collider,
        Handle<StandardMaterial>,
    );
    let mut props: Vec<Prop> = Vec::new();

    let cube_mesh = |meshes: &mut Assets<Mesh>, s: f32| meshes.add(Cuboid::new(s, s, s));
    let cube_collider = |s: f32| Collider::cuboid(s, s, s);
    let sphere_mesh =
        |meshes: &mut Assets<Mesh>, r: f32| meshes.add(Sphere::new(r).mesh().uv(24, 16));
    let sphere_collider = |r: f32| Collider::sphere(r);
    let cylinder_mesh = |meshes: &mut Assets<Mesh>, r: f32, h: f32| {
        meshes.add(Cylinder::new(r, h).mesh().resolution(24))
    };
    let cylinder_collider = |r: f32, h: f32| Collider::cylinder(r, h);

    // Front row, closest to the hands (z ≈ −0.34).
    let s = 0.10;
    props.push((
        -0.30,
        -0.34,
        s * 0.5,
        cube_mesh(meshes, s),
        cube_collider(s),
        mustard.clone(),
    ));
    let r = 0.06;
    props.push((
        -0.12,
        -0.35,
        r,
        sphere_mesh(meshes, r),
        sphere_collider(r),
        plum.clone(),
    ));
    let s = 0.09;
    props.push((
        0.06,
        -0.34,
        s * 0.5,
        cube_mesh(meshes, s),
        cube_collider(s),
        sage.clone(),
    ));
    let (r, h) = (0.05, 0.11);
    props.push((
        0.24,
        -0.35,
        h * 0.5,
        cylinder_mesh(meshes, r, h),
        cylinder_collider(r, h),
        stone.clone(),
    ));
    // Right edge and back row.
    let s = 0.08;
    props.push((
        0.36,
        -0.44,
        s * 0.5,
        cube_mesh(meshes, s),
        cube_collider(s),
        charcoal.clone(),
    ));
    let s = 0.12;
    props.push((
        -0.33,
        -0.50,
        s * 0.5,
        cube_mesh(meshes, s),
        cube_collider(s),
        sage.clone(),
    ));
    let r = 0.07;
    props.push((
        -0.15,
        -0.52,
        r,
        sphere_mesh(meshes, r),
        sphere_collider(r),
        mustard.clone(),
    ));
    let (r, h) = (0.055, 0.10);
    props.push((
        0.04,
        -0.54,
        h * 0.5,
        cylinder_mesh(meshes, r, h),
        cylinder_collider(r, h),
        plum.clone(),
    ));
    let s = 0.11;
    props.push((
        0.22,
        -0.53,
        s * 0.5,
        cube_mesh(meshes, s),
        cube_collider(s),
        stone.clone(),
    ));
    let r = 0.05;
    props.push((
        0.0,
        -0.44,
        r,
        sphere_mesh(meshes, r),
        sphere_collider(r),
        charcoal.clone(),
    ));

    for (x, z, half_h, mesh, collider, material) in props {
        // A deterministic tilt keeps the props from reading as a dev grid.
        let yaw = (x * 7.3 + z * 5.1).sin() * 0.7;
        let entity = commands
            .spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material),
                Transform::from_xyz(x, surface + half_h, z)
                    .with_rotation(Quat::from_rotation_y(yaw)),
                RigidBody::Dynamic,
                collider,
                Friction::new(0.65),
                Restitution::new(0.18),
                Grabbable,
            ))
            .id();
        spawn_outlines(commands, entity, mesh, half_h, outlines);
    }
}
