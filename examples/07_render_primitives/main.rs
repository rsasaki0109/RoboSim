//! Renders URDF visuals with wgpu color and depth outputs.

use rne_ecs::World;
use rne_math::Vec3;
use rne_render::{hash_depth_f32, hash_rgba8, Camera, RenderBackend, RenderScene, Visual};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot};
use rne_world::world_transform_of;

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
        scene.items.push(RenderScene::item_from_visual(
            world_transform_of(&world, *entity),
            visual.shape,
            visual.color_rgba,
            visual.local_offset,
        ));
    }

    let camera = Camera::new(128, 96, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 0.0, 0.0),
        ..CameraOrbit::default()
    };

    let output = backend
        .render_scene_camera(&camera, &orbit.camera_transform(), &scene, [0.05, 0.08, 0.12, 1.0])
        .expect("render scene");

    let center_depth = output.depth.depth_m
        [(output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize];
    let min_depth = output
        .depth
        .depth_m
        .iter()
        .copied()
        .fold(f32::INFINITY, f32::min);

    println!(
        "rendered {} primitives: color_hash={:#018x} depth_hash={:#018x} center_depth={:.2} m min_depth={:.2} m",
        scene.items.len(),
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        center_depth,
        min_depth
    );

    if scene.items.is_empty() || min_depth >= camera.far_m as f32 {
        std::process::exit(1);
    }
}
