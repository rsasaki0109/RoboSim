//! Renders URDF visuals with wgpu color and depth outputs.

use rne_ecs::World;
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{hash_depth_f32, hash_rgba8, Camera, RenderBackend, RenderScene, Visual};
use rne_render_wgpu::WgpuRenderBackend;
use rne_urdf_import::{parse_urdf, spawn_urdf_robot};
use rne_world::Transform3 as WorldTransform3;

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping wgpu example");
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };

    let xml =
        include_str!("../../adapters/ros2/rne_urdf_import/tests/fixtures/minimal_diff_drive.urdf");
    let urdf = parse_urdf(xml).expect("parse URDF");
    let mut world = World::new();
    let spawned = spawn_urdf_robot(&mut world, &urdf).expect("spawn URDF");

    let mut scene = RenderScene::new();
    for entity in spawned.links.values() {
        let Some(visual) = world.get::<Visual>(*entity).cloned() else {
            continue;
        };
        let world_transform = world
            .get::<WorldTransform3>(*entity)
            .copied()
            .unwrap_or_default();
        scene.items.push(RenderScene::item_from_visual(
            world_transform,
            visual.shape,
            visual.color_rgba,
            visual.local_offset,
        ));
    }

    let camera = Camera::new(128, 96, std::f64::consts::FRAC_PI_4);
    let view = Transform3::from_translation_rotation(Vec3::new(0.0, 1.5, 4.0), Quat::IDENTITY);

    let output = backend
        .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
        .expect("render scene");

    let center_depth = output.depth.depth_m
        [(output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize];

    println!(
        "rendered {} primitives: color_hash={:#018x} depth_hash={:#018x} center_depth={:.2} m",
        scene.items.len(),
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        center_depth
    );

    if scene.items.is_empty() || center_depth >= camera.far_m as f32 {
        std::process::exit(1);
    }
}
