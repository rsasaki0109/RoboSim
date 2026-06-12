//! Loads `.rne.scene.toml` / `.rne.robot.toml` assets and spawns the ECS world.

use rne_assets::{load_and_spawn_scene, load_robot_asset, RobotKind};
use rne_core::{SimDuration, SimTime};
use rne_ecs::World;
use rne_math::Hertz;
use rne_physics::{PhysicsBackend, PhysicsWorldDesc};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, ActuatorCommand, ActuatorCommandBuffer,
    DiffDriveComponent,
};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot};
use rne_world::Transform3;
use std::path::PathBuf;

fn main() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = repo_root.join("assets/scenes/episode_diff_drive.rne.scene.toml");

    let mut world = World::new();
    let spawned = load_and_spawn_scene(&mut world, &scene_path).expect("load scene");
    let (model_name, robot) = spawned.robots.first().expect("robot");

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("physics sync");

    let drive = world
        .get::<DiffDriveComponent>(robot.robot)
        .expect("diff drive")
        .0;
    let mut command_buffer = ActuatorCommandBuffer::new();
    let dt = SimDuration::from_hertz(Hertz::new(60.0));

    for _ in 0..60 {
        command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: drive.left_actuator,
                velocity_rad_s: 6.0,
            },
            SimTime::ZERO,
        );
        command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: drive.right_actuator,
                velocity_rad_s: 6.0,
            },
            SimTime::ZERO,
        );
        apply_actuator_commands(&mut world, &mut command_buffer);
        differential_drive_kinematics(&mut world, &[drive], dt);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("physics step");
    }

    let pose = world
        .get::<Transform3>(robot.base_link)
        .expect("base pose")
        .translation;

    println!(
        "scene model={model_name} robots={} base_x={:.2} m seed={}",
        spawned.robots.len(),
        pose.x,
        world
            .get::<rne_world::WorldEntity>(spawned.world)
            .expect("world")
            .seed
    );

    let urdf_robot_path = repo_root.join("assets/robots/diff_drive_urdf.rne.robot.toml");
    let urdf_asset = load_robot_asset(&urdf_robot_path).expect("urdf robot asset");
    assert_eq!(urdf_asset.kind, RobotKind::Urdf);
    let urdf_path = urdf_asset
        .urdf
        .expect("urdf section")
        .resolve_path(urdf_robot_path.parent().expect("robot dir"));
    let xml = std::fs::read_to_string(&urdf_path).expect("read urdf");
    let urdf = parse_urdf(&xml).expect("parse urdf");
    let mut urdf_world = World::new();
    let imported = spawn_urdf_robot(&mut urdf_world, &urdf).expect("spawn urdf");
    println!(
        "urdf adapter: links={} colliders={}",
        imported.links.len(),
        imported.collider_count
    );

    if pose.x <= 0.5 {
        std::process::exit(1);
    }
}
