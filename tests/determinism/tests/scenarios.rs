//! Determinism regression tests for Robot Native Engine scenarios.

use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, DiffDriveConfig,
};
use rne_world::Transform3;

fn run_diff_drive(steps: u32) -> u64 {
    let mut world = World::new();
    let robot = spawn_diff_drive_robot(&mut world, &DiffDriveConfig::default());
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    backend.sync_from_ecs(&mut world, physics_world).unwrap();

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut buffer = ActuatorCommandBuffer::new();
    let mut sim_time = SimTime::ZERO;
    buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: robot.left_actuator,
            velocity_rad_s: 6.0,
        },
        sim_time,
    );
    buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: robot.right_actuator,
            velocity_rad_s: 6.0,
        },
        sim_time,
    );

    let drive = world
        .get::<DiffDriveComponent>(robot.robot)
        .expect("drive")
        .0;

    for _ in 0..steps {
        apply_actuator_commands(&mut world, &mut buffer);
        differential_drive_kinematics(&mut world, &[drive], dt);
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        sim_time = sim_time + dt;
    }

    hash_physics_state(&world)
}

#[test]
fn diff_drive_hash_is_repeatable() {
    let first = run_diff_drive(120);
    let second = run_diff_drive(120);
    assert_eq!(first, second);
    assert_ne!(first, 0);
}

fn run_falling_cube(steps: u32) -> u64 {
    let mut world = World::new();
    let ground = spawn_named(&mut world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(10.0, 0.5, 10.0),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));

    let cube = spawn_named(&mut world, "cube");
    world.entity_mut(cube).insert((
        RigidBody::default(),
        Collider::cuboid(Vec3::splat(0.5)),
        Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
    ));

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    backend.sync_from_ecs(&mut world, physics_world).unwrap();

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    for _ in 0..steps {
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
    }

    hash_physics_state(&world)
}

#[test]
fn falling_cube_hash_is_repeatable() {
    let first = run_falling_cube(180);
    let second = run_falling_cube(180);
    assert_eq!(first, second);
    assert_ne!(first, 0);
}
