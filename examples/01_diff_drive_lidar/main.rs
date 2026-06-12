//! Differential drive robot with IMU, wheel encoders, and LiDAR on a simple floor.

use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, InMemoryDataBus, StreamId};
use rne_ecs::{spawn_named, World};
use rne_log::{frame_header, SimulationLog};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, DiffDriveConfig, DiffDriveSpawned,
};
use rne_sensor::{
    sample_sensors, ImuSpec, LidarSpec, Sensor, SensorKind, SensorSampleContext, SensorState,
    WheelEncoderSpec,
};
use rne_world::Transform3;

struct MountedSensors {
    imu: rne_ecs::Entity,
    lidar: rne_ecs::Entity,
}

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

fn spawn_wall(world: &mut World, name: &str, position: Vec3, half_extents: Vec3) {
    let wall = spawn_named(world, name);
    world.entity_mut(wall).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: half_extents,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(position, Quat::IDENTITY),
    ));
}

fn attach_sensors(world: &mut World, robot: &DiffDriveSpawned) -> MountedSensors {
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
                ray_count: 180,
                max_range_m: 15.0,
                ..LidarSpec::default()
            }),
            update_rate_hz: 10.0,
            latency_ticks: 0,
            frame_id: 11,
            enabled: true,
            stream_id: StreamId::new(101),
        },
        SensorState::default(),
        Transform3::default(),
    ));

    for (entity, stream, actuator) in [
        (
            spawn_named(world, "left_encoder"),
            StreamId::new(102),
            robot.left_actuator,
        ),
        (
            spawn_named(world, "right_encoder"),
            StreamId::new(103),
            robot.right_actuator,
        ),
    ] {
        world.entity_mut(entity).insert((
            Sensor {
                kind: SensorKind::WheelEncoder(WheelEncoderSpec { actuator }),
                update_rate_hz: 50.0,
                latency_ticks: 0,
                frame_id: 12,
                enabled: true,
                stream_id: stream,
            },
            SensorState::default(),
        ));
    }

    MountedSensors {
        imu: robot.base_link,
        lidar,
    }
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

fn main() {
    let mut world = World::new();
    spawn_ground(&mut world);
    spawn_wall(
        &mut world,
        "wall_north",
        Vec3::new(0.0, 1.0, 8.0),
        Vec3::new(8.0, 1.0, 0.25),
    );

    let robot = spawn_diff_drive_robot(
        &mut world,
        &DiffDriveConfig {
            initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
            ..DiffDriveConfig::default()
        },
    );
    let sensors = attach_sensors(&mut world, &robot);

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    backend.sync_from_ecs(&mut world, physics_world).unwrap();

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut command_buffer = ActuatorCommandBuffer::new();
    let mut data_bus = InMemoryDataBus::new();
    let mut log = SimulationLog::new();
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

    for step in 0..180 {
        apply_actuator_commands(&mut world, &mut command_buffer);
        differential_drive_kinematics(&mut world, &[drive], dt);
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        sync_lidar_mount(&mut world, robot.base_link, sensors.lidar);

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

        if step % 60 == 59 {
            let pose = world
                .get::<Transform3>(robot.base_link)
                .unwrap()
                .translation;
            let lidar = data_bus
                .latest::<rne_data::PointCloud>(StreamId::new(101))
                .unwrap();
            let imu = data_bus
                .latest::<rne_data::ImuSample>(StreamId::new(100))
                .unwrap();
            println!(
                "step {}: base=({:.2}, {:.2}, {:.2}) m, lidar points={}, imu ay={:.2} m/s²",
                step + 1,
                pose.x,
                pose.y,
                pose.z,
                lidar.payload.points_m.len(),
                imu.payload.linear_acceleration_m_s2.y
            );

            log.record_lidar(
                frame_header(
                    StreamId::new(101),
                    lidar.entity.index(),
                    lidar.sequence,
                    lidar.capture_time,
                    lidar.available_time,
                ),
                lidar.payload,
            );
        }

        sim_time = sim_time + dt;
    }

    let final_x = world
        .get::<Transform3>(robot.base_link)
        .unwrap()
        .translation
        .x;
    println!("final forward travel = {final_x:.2} m");
    println!("imu frames = {}", data_bus.frame_count(StreamId::new(100)));
    println!(
        "lidar frames = {}",
        data_bus.frame_count(StreamId::new(101))
    );
    println!(
        "wheel encoder frames = {}",
        data_bus.frame_count(StreamId::new(102))
    );
    let _ = sensors.imu;
}
