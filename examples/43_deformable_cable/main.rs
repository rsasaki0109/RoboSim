//! Runs and optionally renders a deterministic XPBD cable draped over a sphere.

use rne_assets::load_and_spawn_scene;
use rne_deformable::{
    step_deformable_world, DeformableBody, DeformableSolverConfig, DeformableVisual,
};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_render::{hash_rgba8, Camera, RenderBackend, RenderScene, Visual};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::{world_transform_of, Transform3};
use std::path::PathBuf;

fn main() {
    let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/deformable_cable.rne.scene.toml");
    let mut world = World::new();
    let spawned = load_and_spawn_scene(&mut world, &scene_path).expect("load cable scene");
    let cable_entity = spawned.deformables[0];
    for _ in 0..240 {
        step_deformable_world(
            &mut world,
            Vec3::new(0.0, -9.81, 0.0),
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect("step cable");
    }
    let cable = world
        .get::<DeformableBody>(cable_entity)
        .expect("spawned cable");
    println!(
        "XPBD cable settled: particles={} hash={:#018x} midpoint={:?}",
        cable.particles.len(),
        cable.stable_state_hash(),
        cable.particles[cable.particles.len() / 2].position_m
    );
    assert!(cable
        .particles
        .iter()
        .all(|particle| particle.position_m.is_finite()));

    if std::env::args().any(|argument| argument == "--render") {
        render(&world, cable_entity);
    }
}

fn render(world: &World, cable_entity: rne_ecs::Entity) {
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
    let cable = world.get::<DeformableBody>(cable_entity).expect("cable");
    let color = world
        .get::<DeformableVisual>(cable_entity)
        .expect("cable visual")
        .color_rgba;
    for segment in cable.cable_segments() {
        let delta = segment.end_m - segment.start_m;
        let length_m = delta.length();
        let rotation = Quat::from_rotation_arc(Vec3::Z, delta / length_m);
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                (segment.start_m + segment.end_m) * 0.5,
                rotation,
            ),
            rne_render::VisualShape::Cylinder {
                radius_m: segment.radius_m,
                length_m,
            },
            color,
            Transform3::IDENTITY,
        ));
    }
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(640, 480, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 0.62, 0.0),
        yaw_rad: 0.72,
        pitch_rad: 1.05,
        distance_m: 1.65,
    };
    let output = backend
        .render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.03, 0.05, 0.08, 1.0],
        )
        .expect("render cable");
    println!(
        "wgpu cable color_hash={:#018x}",
        hash_rgba8(&output.color.rgba8)
    );
}
