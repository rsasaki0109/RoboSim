//! Analytic physics backend integration tests.

use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    capture_physics_snapshot, require_capabilities, PhysicsBackend, PhysicsCapability,
    PhysicsError, PhysicsWorldDesc, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_analytic::AnalyticBackend;
use rne_world::Transform3;

fn falling_world() -> (AnalyticBackend, PhysicsWorldId, World, rne_ecs::Entity) {
    let mut backend = AnalyticBackend::new();
    let world_id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let entity = spawn_named(&mut world, "cube");
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Dynamic,
            linear_velocity_m_s: Vec3::ZERO,
            ..RigidBody::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
    ));
    backend.sync_from_ecs(&mut world, world_id).expect("sync");
    (backend, world_id, world, entity)
}

#[test]
fn dynamic_body_free_falls_under_gravity() {
    let (mut backend, world_id, mut world, entity) = falling_world();
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    for _ in 0..60 {
        rne_physics_analytic::step_physics(&mut backend, &mut world, world_id, dt).expect("step");
    }

    let transform = world.get::<Transform3>(entity).expect("transform");
    // Analytic expectation for 1 s of free fall from 5 m: 5 - 0.5 * 9.81.
    let expected_y = 5.0 - 0.5 * 9.81;
    assert!(
        (transform.translation.y - expected_y).abs() < 0.2,
        "free-fall height drifted too far: {} vs {}",
        transform.translation.y,
        expected_y
    );
    assert!(
        transform.translation.y < expected_y,
        "explicit Euler drifts below the continuous value"
    );
    assert!(
        (transform.translation.x.abs() + transform.translation.z.abs()) < 1e-12,
        "horizontal drift must be zero"
    );
    let rigid_body = world.get::<RigidBody>(entity).expect("rigid body");
    assert!(
        (rigid_body.linear_velocity_m_s.y + 9.81).abs() < 1.0e-6,
        "integrated velocity must be written back to ECS: {:?}",
        rigid_body.linear_velocity_m_s
    );
}

#[test]
fn free_fall_is_bit_deterministic() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let hashes = (0..2)
        .map(|_| {
            let (mut backend, world_id, mut world, _entity) = falling_world();
            for _ in 0..120 {
                rne_physics_analytic::step_physics(&mut backend, &mut world, world_id, dt)
                    .expect("step");
            }
            capture_physics_snapshot(&world, &[], 120, dt.ticks() * 120)
                .expect("canonical snapshot")
                .stable_hash()
        })
        .collect::<Vec<_>>();
    assert_eq!(
        hashes[0], hashes[1],
        "analytic free fall must be deterministic"
    );
}

#[test]
fn kinematic_and_fixed_bodies_do_not_move() {
    let mut backend = AnalyticBackend::new();
    let world_id = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("world");
    let mut world = World::new();
    let dynamic = spawn_named(&mut world, "dynamic");
    let fixed = spawn_named(&mut world, "fixed");
    world.entity_mut(dynamic).insert((
        RigidBody::default(),
        Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
    ));
    world.entity_mut(fixed).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.0), Quat::IDENTITY),
    ));
    backend.sync_from_ecs(&mut world, world_id).expect("sync");

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    for _ in 0..30 {
        rne_physics_analytic::step_physics(&mut backend, &mut world, world_id, dt).expect("step");
    }

    let fixed_y = world
        .get::<Transform3>(fixed)
        .expect("transform")
        .translation
        .y;
    assert!(
        fixed_y.abs() < 1e-12,
        "fixed body must not move, got {fixed_y}"
    );
    let dynamic_y = world
        .get::<Transform3>(dynamic)
        .expect("transform")
        .translation
        .y;
    assert!(dynamic_y < 5.0, "dynamic body must fall");
}

#[test]
fn declares_only_rigid_body_and_deterministic_step() {
    let backend = AnalyticBackend::new();
    assert_eq!(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::DeterministicStep
        ]
    );
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::DeterministicStep,
        ],
    )
    .expect("declared capabilities are accepted");

    let error = require_capabilities(backend.capabilities(), &[PhysicsCapability::Articulation])
        .expect_err("articulation is not supported");
    assert!(matches!(error, PhysicsError::MissingCapabilities { .. }));
}

#[test]
fn contacts_and_raycasts_are_empty() {
    let (backend, world_id, _world, _entity) = falling_world();
    assert!(backend.contacts(world_id).expect("contacts").is_empty());
    assert!(backend
        .raycast(
            world_id,
            rne_physics::RaycastQuery::downward(Vec3::ZERO, 10.0)
        )
        .expect("raycast")
        .is_empty());
}
