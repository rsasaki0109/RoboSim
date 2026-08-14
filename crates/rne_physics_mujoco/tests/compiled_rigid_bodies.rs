#![cfg(feature = "mujoco")]

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc, RevoluteJointDesc,
    RigidBody, RigidBodyType,
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
fn preflight_rejects_articulation_before_native_model_creation() {
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
        Vec3::Y,
    );
    world.entity_mut(child).insert(RevoluteJointDesc {
        parent,
        axis: Vec3::Z,
        anchor_parent_m: Vec3::ZERO,
        anchor_child_m: Vec3::ZERO,
        lower_rad: None,
        upper_rad: None,
    });

    assert_eq!(
        backend.preflight_world(&world),
        Err(MuJoCoError::MissingCapability {
            capability: PhysicsCapability::Articulation,
        })
    );
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
