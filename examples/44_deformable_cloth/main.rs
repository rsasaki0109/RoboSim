//! Runs and optionally renders deterministic XPBD cloth draping over a box.

use rne_assets::load_and_spawn_scene;
use rne_deformable::{
    step_deformable_world, DeformableBody, DeformableSolverConfig, DeformableVisual,
};
use rne_ecs::World;
use rne_math::Vec3;
use rne_render::{hash_rgba8, Camera, RenderBackend, RenderScene, TriangleMesh, Visual};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::world_transform_of;
use std::path::PathBuf;

fn main() {
    let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/deformable_cloth.rne.scene.toml");
    let mut world = World::new();
    let spawned = load_and_spawn_scene(&mut world, &scene_path).expect("load cloth scene");
    let cloth_entity = spawned.deformables[0];
    for _ in 0..300 {
        step_deformable_world(
            &mut world,
            Vec3::new(0.0, -9.81, 0.0),
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect("step cloth");
    }
    let cloth = world
        .get::<DeformableBody>(cloth_entity)
        .expect("spawned cloth");
    let surface = cloth.cloth_surface_mesh().expect("cloth surface");
    assert!(cloth
        .particles
        .iter()
        .all(|particle| particle.position_m.is_finite()));
    assert!(surface
        .normals
        .iter()
        .all(|normal| normal.iter().all(|value| value.is_finite())));
    println!(
        "XPBD cloth draped: particles={} triangles={} hash={:#018x}",
        cloth.particles.len(),
        surface.indices.len() / 3,
        cloth.stable_state_hash()
    );

    if std::env::args().any(|argument| argument == "--render") {
        render(&world, cloth_entity);
    }
}

fn render(world: &World, cloth_entity: rne_ecs::Entity) {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let mut scene = RenderScene::new();
    let mut visuals = world
        .iter_entities()
        .filter_map(|entity| {
            let visual = world.get::<Visual>(entity.id())?.clone();
            Some((entity.id().to_bits(), entity.id(), visual))
        })
        .collect::<Vec<_>>();
    visuals.sort_by_key(|(bits, _, _)| *bits);
    for (_, entity, visual) in visuals {
        scene.items.push(RenderScene::item_from_visual(
            world_transform_of(world, entity),
            visual.shape,
            visual.color_rgba,
            visual.local_offset,
        ));
    }
    let cloth = world.get::<DeformableBody>(cloth_entity).expect("cloth");
    let surface = cloth.cloth_surface_mesh().expect("cloth surface");
    let color = world
        .get::<DeformableVisual>(cloth_entity)
        .expect("cloth visual")
        .color_rgba;
    scene.items.push(RenderScene::item_from_dynamic_mesh(
        TriangleMesh {
            positions: surface.positions,
            normals: surface.normals,
            indices: surface.indices,
        },
        color,
    ));
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(640, 480, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 0.48, 0.0),
        yaw_rad: 0.72,
        pitch_rad: 0.92,
        distance_m: 1.75,
    };
    let output = backend
        .render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.03, 0.05, 0.08, 1.0],
        )
        .expect("render cloth");
    println!(
        "wgpu cloth color_hash={:#018x}",
        hash_rgba8(&output.color.rgba8)
    );
}
