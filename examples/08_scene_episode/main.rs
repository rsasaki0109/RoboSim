//! End-to-end demo: scene asset → episode loop → optional wgpu render.

use rne_ai::{ConstantVelocityPolicy, DiffDriveEpisode, DiffDriveEpisodeConfig, Episode, Policy};
use rne_ecs::World;
use rne_math::{Quat, Transform3, Vec3};
use rne_physics::Collider;
use rne_render::{
    hash_depth_f32, hash_rgba8, Camera, RenderBackend, RenderScene, Visual, VisualShape,
};
use rne_render_wgpu::WgpuRenderBackend;
use rne_robot::DiffDriveSpawned;
use rne_world::Transform3 as WorldTransform3;
use std::path::PathBuf;

fn main() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = repo_root.join("assets/scenes/episode_diff_drive.rne.scene.toml");

    let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
        goal_x_m: 2.0,
        max_steps: 300,
        record_log: true,
        scene_path: Some(scene_path),
        ..DiffDriveEpisodeConfig::default()
    });
    let mut policy = ConstantVelocityPolicy::new(6.0);

    let mut step = env.reset();
    println!(
        "scene loaded: seed={}, reset base_x={:.2} m",
        env.world_seed(),
        step.observation.base_x_m
    );

    while !step.is_done() {
        step = env.step(policy.act(&step.observation));
    }

    println!(
        "episode done: base_x={:.2} m, reward={:.3}, terminated={}, truncated={}, total_reward={:.3}",
        step.observation.base_x_m,
        step.reward,
        step.terminated,
        step.truncated,
        env.total_reward()
    );
    println!("recorded commands = {}", env.log().records().len());

    if !step.terminated {
        std::process::exit(1);
    }

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping final render");
        return;
    }

    match render_final_pose(env.simulation()) {
        Ok(summary) => println!("{summary}"),
        Err(error) => eprintln!("render skipped: {error}"),
    }
}

fn render_final_pose(sim: &rne_ai::DiffDriveSim) -> Result<String, String> {
    let mut backend = WgpuRenderBackend::new().map_err(|error| error.to_string())?;
    let scene = build_diff_drive_render_scene(sim.world(), sim.robot());
    let camera = Camera::new(160, 120, std::f64::consts::FRAC_PI_4);
    let view = Transform3::from_translation_rotation(Vec3::new(0.0, 1.5, 4.0), Quat::IDENTITY);

    let output = backend
        .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
        .map_err(|error| error.to_string())?;

    let center_depth = output.depth.depth_m
        [(output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize];

    Ok(format!(
        "final render: items={} color_hash={:#018x} depth_hash={:#018x} center_depth={:.2} m",
        scene.items.len(),
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        center_depth
    ))
}

fn build_diff_drive_render_scene(world: &World, robot: &DiffDriveSpawned) -> RenderScene {
    let drive = robot.drive;
    let mut scene = RenderScene::new();

    scene.items.push(render_item(
        world,
        robot.base_link,
        VisualShape::Box {
            size_m: base_size_m(world, robot.base_link),
        },
        [0.35, 0.55, 0.95, 1.0],
    ));

    for wheel in [robot.left_wheel, robot.right_wheel] {
        if wheel == robot.base_link {
            continue;
        }
        scene.items.push(render_item(
            world,
            wheel,
            VisualShape::Cylinder {
                radius_m: drive.wheel_radius_m,
                length_m: drive.wheel_radius_m * 0.6,
            },
            [0.2, 0.2, 0.2, 1.0],
        ));
    }

    scene.items.push(RenderScene::item_from_visual(
        WorldTransform3::from_translation_rotation(Vec3::new(0.0, -0.01, 0.0), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(40.0, 0.02, 40.0),
        },
        [0.25, 0.28, 0.32, 1.0],
        WorldTransform3::IDENTITY,
    ));

    scene
}

fn render_item(
    world: &World,
    entity: rne_ecs::Entity,
    shape: VisualShape,
    color_rgba: [f32; 4],
) -> rne_render::RenderSceneItem {
    let world_transform = world
        .get::<WorldTransform3>(entity)
        .copied()
        .unwrap_or_default();
    let local_offset = world
        .get::<Visual>(entity)
        .map(|visual| visual.local_offset)
        .unwrap_or_default();
    RenderScene::item_from_visual(world_transform, shape, color_rgba, local_offset)
}

fn base_size_m(world: &World, base_link: rne_ecs::Entity) -> Vec3 {
    world
        .get::<Collider>(base_link)
        .and_then(|collider| match collider.shape {
            rne_physics::ColliderShape::Cuboid { half_extents_m } => Some(half_extents_m * 2.0),
            _ => None,
        })
        .unwrap_or_else(|| Vec3::new(0.5, 0.3, 0.4))
}
