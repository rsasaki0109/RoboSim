#![cfg(feature = "mujoco")]

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, JointActuation, JointState, PhysicsBackend, PhysicsError, PhysicsWorldDesc,
    PrismaticJointDesc, RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_physics_mujoco::{MuJoCoBackend, MuJoCoError};
use rne_world::Transform3;

fn spawn_body(
    world: &mut World,
    name: &str,
    body_type: RigidBodyType,
    collider: Collider,
    position: Vec3,
) -> Entity {
    let entity = spawn_named(world, name);
    world.entity_mut(entity).insert((
        RigidBody {
            body_type,
            mass_kg: 2.0,
            ..RigidBody::default()
        },
        collider,
        Transform3::from_translation_rotation(position, Quat::IDENTITY),
    ));
    entity
}

#[test]
fn compiles_and_syncs_multiple_rigid_bodies() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let fixed = spawn_body(
        &mut world,
        "fixed",
        RigidBodyType::Fixed,
        Collider::cuboid(Vec3::splat(0.5)),
        Vec3::new(20.0, 0.0, 0.0),
    );
    let sphere = spawn_body(
        &mut world,
        "sphere",
        RigidBodyType::Dynamic,
        Collider::sphere(0.05),
        Vec3::new(0.0, 5.0, 0.0),
    );
    let cube = spawn_body(
        &mut world,
        "cube",
        RigidBodyType::Dynamic,
        Collider::cuboid(Vec3::splat(0.1)),
        Vec3::new(2.0, 8.0, 0.0),
    );
    world
        .get_mut::<RigidBody>(cube)
        .unwrap()
        .linear_velocity_m_s
        .x = 1.0;

    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("compile and upload ECS state");
    backend.step(physics_world, dt).expect("fixed step");
    backend
        .sync_to_ecs(&mut world, physics_world)
        .expect("download native state");

    let sphere_transform = world.get::<Transform3>(sphere).unwrap();
    let sphere_body = world.get::<RigidBody>(sphere).unwrap();
    let cube_transform = world.get::<Transform3>(cube).unwrap();
    let cube_body = world.get::<RigidBody>(cube).unwrap();
    assert!(sphere_transform.translation.y < 5.0);
    assert!(cube_transform.translation.y < 8.0);
    assert!(sphere_body.linear_velocity_m_s.y < 0.0);
    assert!(cube_body.linear_velocity_m_s.y < 0.0);
    assert!(cube_transform.translation.x > 2.0);
    assert_eq!(
        world.get::<Transform3>(fixed).unwrap().translation,
        Vec3::new(20.0, 0.0, 0.0)
    );
}

#[test]
fn preflight_accepts_supported_articulation_before_native_model_creation() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let mut world = World::new();
    let parent = spawn_body(
        &mut world,
        "parent",
        RigidBodyType::Fixed,
        Collider::sphere(0.1),
        Vec3::ZERO,
    );
    let child = spawn_body(
        &mut world,
        "child",
        RigidBodyType::Dynamic,
        Collider::sphere(0.1),
        -Vec3::Y,
    );
    world.entity_mut(child).insert(RevoluteJointDesc {
        parent,
        axis: Vec3::Z,
        anchor_parent_m: Vec3::ZERO,
        anchor_child_m: Vec3::Y,
        lower_rad: None,
        upper_rad: None,
    });

    backend
        .preflight_world(&world)
        .expect("supported revolute topology passes preflight");
}

#[test]
fn kinematic_body_fails_with_capability_error_before_model_creation() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let entity = spawn_body(
        &mut world,
        "kinematic",
        RigidBodyType::Kinematic,
        Collider::sphere(0.1),
        Vec3::Y,
    );

    assert_eq!(
        backend.preflight_world(&world),
        Err(MuJoCoError::MissingCapability {
            capability: rne_physics::PhysicsCapability::KinematicBody,
            entity_index: entity.index(),
        })
    );
    assert_eq!(
        backend.sync_from_ecs(&mut world, physics_world),
        Err(PhysicsError::MissingCapabilities {
            missing: vec![rne_physics::PhysicsCapability::KinematicBody],
        })
    );

    world.get_mut::<RigidBody>(entity).unwrap().body_type = RigidBodyType::Dynamic;
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("failed preflight did not create or lock a native model");
}

fn run_revolute(command: JointActuation) -> JointState {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 16,
        })
        .expect("physics world");
    let mut world = World::new();
    let parent = spawn_body(
        &mut world,
        "parent",
        RigidBodyType::Fixed,
        Collider::sphere(0.05),
        Vec3::ZERO,
    );
    let child = spawn_body(
        &mut world,
        "child",
        RigidBodyType::Dynamic,
        Collider::sphere(0.05),
        -Vec3::Y,
    );
    world.entity_mut(child).insert((
        RevoluteJointDesc {
            parent,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::Y,
            lower_rad: Some(-1.0),
            upper_rad: Some(1.0),
        },
        command,
    ));
    for _ in 0..30 {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("upload joint state and command");
        backend.step(physics_world, dt).expect("fixed step");
        backend
            .sync_to_ecs(&mut world, physics_world)
            .expect("download joint state");
    }
    *world.get::<JointState>(child).expect("joint state")
}

#[test]
fn revolute_position_velocity_and_effort_modes_move_the_joint() {
    let position = run_revolute(JointActuation::RevolutePosition {
        target_position_rad: 0.4,
        stiffness_nm_per_rad: 40.0,
        damping_nm_s_per_rad: 4.0,
        max_effort_nm: 20.0,
    });
    let velocity = run_revolute(JointActuation::RevoluteVelocity {
        target_velocity_rad_s: 1.0,
        gain_nm_s_per_rad: 4.0,
        max_effort_nm: 20.0,
    });
    let effort = run_revolute(JointActuation::RevoluteEffort {
        effort_nm: 2.0,
        max_effort_nm: 2.0,
    });
    assert!(position.position_rad().unwrap() > 0.1);
    assert!(velocity.position_rad().unwrap() > 0.1);
    assert!(effort.position_rad().unwrap() > 0.01);
}

fn run_prismatic(command: JointActuation) -> JointState {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 16,
        })
        .expect("physics world");
    let mut world = World::new();
    let parent = spawn_body(
        &mut world,
        "parent",
        RigidBodyType::Fixed,
        Collider::sphere(0.05),
        Vec3::ZERO,
    );
    let child = spawn_body(
        &mut world,
        "child",
        RigidBodyType::Dynamic,
        Collider::sphere(0.05),
        -Vec3::Y,
    );
    world.entity_mut(child).insert((
        PrismaticJointDesc {
            parent,
            axis: Vec3::X,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::Y,
            lower_m: Some(-0.25),
            upper_m: Some(0.25),
        },
        command,
    ));
    for _ in 0..30 {
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, dt).unwrap();
        backend.sync_to_ecs(&mut world, physics_world).unwrap();
    }
    *world.get::<JointState>(child).expect("joint state")
}

#[test]
fn prismatic_position_velocity_and_effort_modes_move_the_joint() {
    let position = run_prismatic(JointActuation::PrismaticPosition {
        target_position_m: 0.15,
        stiffness_n_per_m: 80.0,
        damping_n_s_per_m: 8.0,
        max_force_n: 30.0,
    });
    let velocity = run_prismatic(JointActuation::PrismaticVelocity {
        target_velocity_m_s: 0.4,
        gain_n_s_per_m: 10.0,
        max_force_n: 30.0,
    });
    let effort = run_prismatic(JointActuation::PrismaticEffort {
        force_n: 2.0,
        max_force_n: 2.0,
    });
    assert!(position.position_m().unwrap() > 0.05);
    assert!(velocity.position_m().unwrap() > 0.05);
    assert!(effort.position_m().unwrap() > 0.01);
}

#[test]
fn invalid_actuation_returns_precise_pre_step_error() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let parent = spawn_body(
        &mut world,
        "parent",
        RigidBodyType::Fixed,
        Collider::sphere(0.05),
        Vec3::ZERO,
    );
    let child = spawn_body(
        &mut world,
        "child",
        RigidBodyType::Dynamic,
        Collider::sphere(0.05),
        -Vec3::Y,
    );
    world.entity_mut(child).insert((
        RevoluteJointDesc {
            parent,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::Y,
            lower_rad: None,
            upper_rad: None,
        },
        JointActuation::PrismaticEffort {
            force_n: 1.0,
            max_force_n: 2.0,
        },
    ));
    assert!(matches!(
        backend.preflight_world(&world),
        Err(MuJoCoError::InvalidActuation { .. })
    ));
    assert!(matches!(
        backend.sync_from_ecs(&mut world, physics_world),
        Err(PhysicsError::InvalidActuation { .. })
    ));
}

#[test]
fn rejects_wrong_step_and_post_compile_topology_change() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("MuJoCo runtime");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    spawn_body(
        &mut world,
        "body",
        RigidBodyType::Dynamic,
        Collider::sphere(0.1),
        Vec3::Y,
    );
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("compile topology");
    assert_eq!(
        backend.step(physics_world, SimDuration::from_hertz(Hertz::new(120.0))),
        Err(PhysicsError::InitializationFailed)
    );

    spawn_body(
        &mut world,
        "late_body",
        RigidBodyType::Dynamic,
        Collider::sphere(0.1),
        Vec3::new(2.0, 1.0, 0.0),
    );
    assert_eq!(
        backend.sync_from_ecs(&mut world, physics_world),
        Err(PhysicsError::InitializationFailed)
    );
}
