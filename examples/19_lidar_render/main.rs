//! Renders a diff-drive LiDAR scan as sphere markers in a wgpu scene pass.

use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, InMemoryDataBus, PointCloud, StreamId};
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{
    hash_depth_f32, hash_rgba8, Camera, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, DiffDriveConfig, DiffDriveSpawned,
};
use rne_sensor::{
    sample_sensors, ImuSpec, LidarSpec, Sensor, SensorKind, SensorSampleContext, SensorState,
};
use rne_world::Transform3;

const LIDAR_STREAM: StreamId = StreamId::new(101);
const SIM_STEPS: usize = 120;
const MIN_LIDAR_HITS: usize = 8;

fn spawn_ground(world: &mut World) {
    let ground = spawn_named(world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(20.0, 0.5, 20.0),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
}

fn spawn_wall(world: &mut World) {
    let wall = spawn_named(world, "wall_north");
    world.entity_mut(wall).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(8.0, 1.0, 0.25),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, 1.0, 8.0), Quat::IDENTITY),
    ));
}

fn attach_lidar(world: &mut World, robot: &DiffDriveSpawned) -> rne_ecs::Entity {
    world.entity_mut(robot.base_link).insert((
        Sensor {
            kind: SensorKind::Imu(ImuSpec::default()),
            update_rate_hz: 100.0,
            latency_ticks: 0,
            frame_id: 10,
            enabled: true,
            stream_id: StreamId::new(100),
        },
        SensorState::default(),
    ));

    let lidar = spawn_named(world, "lidar");
    world.entity_mut(lidar).insert((
        Sensor {
            kind: SensorKind::Lidar(LidarSpec {
                ray_count: 120,
                max_range_m: 15.0,
                ..LidarSpec::default()
            }),
            update_rate_hz: 10.0,
            latency_ticks: 0,
            frame_id: 11,
            enabled: true,
            stream_id: LIDAR_STREAM,
        },
        SensorState::default(),
        Transform3::IDENTITY,
    ));
    lidar
}

fn sync_lidar_mount(world: &mut World, base_link: rne_ecs::Entity, lidar: rne_ecs::Entity) {
    let Some(base) = world.get::<Transform3>(base_link).copied() else {
        return;
    };
    if let Some(mut lidar_tf) = world.get_mut::<Transform3>(lidar) {
        lidar_tf.translation = base.translation + base.rotation * Vec3::new(0.0, 0.2, 0.0);
        lidar_tf.rotation = base.rotation;
    }
}

fn append_box(scene: &mut RenderScene, center: Vec3, size_m: Vec3, color_rgba: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: rne_math::Transform3 {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m },
        color_rgba,
        mesh: None,
    });
}

fn build_scene(base: Vec3, lidar_mount: Vec3, cloud: &PointCloud) -> RenderScene {
    let mut scene = RenderScene::new();
    append_box(
        &mut scene,
        Vec3::new(0.0, -0.01, 0.0),
        Vec3::new(40.0, 0.02, 40.0),
        [0.25, 0.28, 0.32, 1.0],
    );
    append_box(
        &mut scene,
        Vec3::new(0.0, 1.0, 8.0),
        Vec3::new(16.0, 2.0, 0.5),
        [0.45, 0.48, 0.52, 1.0],
    );
    append_box(
        &mut scene,
        base,
        Vec3::new(0.5, 0.3, 0.4),
        [0.35, 0.55, 0.95, 1.0],
    );
    scene.append_lidar_points_sized(
        std::slice::from_ref(&lidar_mount),
        0.06,
        [0.95, 0.35, 0.25, 1.0],
    );
    scene.append_lidar_points(&cloud.points_m, [0.15, 0.95, 0.35, 1.0]);
    scene
}

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping LiDAR render example");
        return;
    }

    let mut world = World::new();
    spawn_ground(&mut world);
    spawn_wall(&mut world);
    let robot = spawn_diff_drive_robot(
        &mut world,
        &DiffDriveConfig {
            initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
            ..DiffDriveConfig::default()
        },
    );
    let lidar_entity = attach_lidar(&mut world, &robot);

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    backend.sync_from_ecs(&mut world, physics_world).unwrap();

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut command_buffer = ActuatorCommandBuffer::new();
    let mut data_bus = InMemoryDataBus::new();
    let mut sim_time = SimTime::ZERO;
    command_buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: robot.left_actuator,
            velocity_rad_s: 6.0,
        },
        sim_time,
    );
    command_buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: robot.right_actuator,
            velocity_rad_s: 6.0,
        },
        sim_time,
    );

    let drive = world
        .get::<DiffDriveComponent>(robot.robot)
        .expect("drive component")
        .0;

    for _ in 0..SIM_STEPS {
        apply_actuator_commands(&mut world, &mut command_buffer);
        differential_drive_kinematics(&mut world, &[drive], dt);
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        sync_lidar_mount(&mut world, robot.base_link, lidar_entity);
        sample_sensors(
            &mut SensorSampleContext {
                world: &mut world,
                sim_time,
                physics: &backend,
                physics_world,
                render: None,
            },
            &mut data_bus,
        );
        sim_time = sim_time + dt;
    }

    let cloud = data_bus
        .latest::<PointCloud>(LIDAR_STREAM)
        .expect("lidar frame")
        .payload
        .clone();
    assert!(
        cloud.points_m.len() >= MIN_LIDAR_HITS,
        "expected at least {MIN_LIDAR_HITS} lidar hits, got {}",
        cloud.points_m.len()
    );

    let base = world
        .get::<Transform3>(robot.base_link)
        .expect("base transform")
        .translation;
    let lidar_mount = world
        .get::<Transform3>(lidar_entity)
        .expect("lidar transform")
        .translation;
    let scene = build_scene(base, lidar_mount, &cloud);

    let mut render_backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };

    let camera = Camera::new(640, 360, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: base + Vec3::new(0.0, 0.0, 4.0),
        yaw_rad: 0.35,
        pitch_rad: 0.62,
        distance_m: 10.0,
    };
    let output = render_backend
        .render_scene_camera(
            &camera,
            &orbit.camera_transform(),
            &scene,
            [0.05, 0.08, 0.12, 1.0],
        )
        .expect("render lidar scene");

    let min_depth = output
        .depth
        .depth_m
        .iter()
        .copied()
        .fold(f32::INFINITY, f32::min);

    println!(
        "rendered lidar scene: hits={} scene_items={} color_hash={:#018x} depth_hash={:#018x} min_depth={:.2} m base_x={:.2} m",
        cloud.points_m.len(),
        scene.items.len(),
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        min_depth,
        base.x
    );

    if min_depth >= camera.far_m as f32 {
        std::process::exit(1);
    }
}
