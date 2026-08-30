#![cfg(feature = "mujoco")]

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, PhysicsBackend, PhysicsCapability, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_mujoco::{MuJoCoBackend, MuJoCoError, EXPECTED_MUJOCO_VERSION_PREFIX};
use rne_world::Transform3;

const FREE_FALL_MJCF: &str = r#"
<mujoco model="rne-free-fall">
  <compiler angle="radian"/>
  <option timestep="0.016666666" gravity="0 -9.81 0"/>
  <worldbody>
    <body name="rne_free_fall_body" pos="0 5 0">
      <freejoint name="rne_free_fall_joint"/>
      <geom name="rne_free_fall_sphere" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
"#;

#[test]
fn runtime_line_is_explicit() {
    let result = MuJoCoBackend::runtime_version();
    match result {
        Ok(version) => assert!(version.starts_with(EXPECTED_MUJOCO_VERSION_PREFIX)),
        Err(MuJoCoError::RuntimeVersionMismatch { .. }) => panic!("unexpected runtime ABI"),
        Err(error) => panic!("MuJoCo runtime unavailable: {error}"),
    }
}

#[test]
fn fixture_loads_only_when_the_runtime_is_installed() {
    let backend = MuJoCoBackend::from_mjcf(FREE_FALL_MJCF).expect("fixture should load");
    assert_eq!(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::Articulation,
            PhysicsCapability::ContactForce,
            PhysicsCapability::RaycastBatch,
            PhysicsCapability::JointEffortMeasurement,
            PhysicsCapability::ExternalBodyWrench,
        ]
    );
}

fn fixture_world() -> (World, Entity) {
    let mut world = World::new();
    let entity = spawn_named(&mut world, "free-fall");
    world.entity_mut(entity).insert((
        RigidBody {
            mass_kg: 1.0,
            linear_velocity_m_s: Vec3::ZERO,
            ..RigidBody::default()
        },
        Collider::sphere(0.05),
        Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
    ));
    (world, entity)
}

fn run_free_fall() -> (f64, f64, u64) {
    let mut backend = MuJoCoBackend::from_mjcf(FREE_FALL_MJCF).expect("fixture should load");
    let (mut world, entity) = fixture_world();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("MuJoCo world should be created");
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    for _ in 0..60 {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("ECS state should synchronize into MuJoCo");
        backend
            .step(physics_world, dt)
            .expect("MuJoCo should accept its fixture timestep");
        backend
            .sync_to_ecs(&mut world, physics_world)
            .expect("MuJoCo state should synchronize into ECS");
    }
    let transform = world.get::<Transform3>(entity).expect("transform");
    let body = world.get::<RigidBody>(entity).expect("rigid body");
    assert!(transform.translation.y < 5.0);
    assert!(body.linear_velocity_m_s.y < 0.0);
    assert!(transform.translation.y.is_finite());
    assert!(body.linear_velocity_m_s.y.is_finite());
    (
        transform.translation.y,
        body.linear_velocity_m_s.y,
        transform.translation.y.to_bits() ^ body.linear_velocity_m_s.y.to_bits(),
    )
}

#[test]
fn free_fall_is_repeatable_on_the_same_runtime() {
    let first = run_free_fall();
    let second = run_free_fall();
    assert_eq!(first, second);
}

#[test]
fn compiled_world_reports_canonical_accumulated_contact_impulse() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("compiled backend");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let ground = spawn_named(&mut world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::new(2.0, 0.5, 2.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let cube = spawn_named(&mut world, "cube");
    world.entity_mut(cube).insert((
        RigidBody {
            mass_kg: 2.0,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::splat(0.25)),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY),
    ));

    let mut settled_impulses = Vec::new();
    for step in 0..180 {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("sync into MuJoCo");
        backend.step(physics_world, dt).expect("fixed step");
        backend
            .sync_to_ecs(&mut world, physics_world)
            .expect("sync from MuJoCo");
        if step >= 120 {
            let contact = backend
                .contacts(physics_world)
                .expect("contact evidence")
                .iter()
                .find(|contact| contact.entity_a == ground && contact.entity_b == cube)
                .expect("canonical ground/cube pair");
            assert!(contact.normal.y > 0.99);
            assert_eq!(contact.normal.x, 0.0);
            assert_eq!(contact.normal.z, 0.0);
            settled_impulses.push(contact.impulse as f64);
        }
    }

    let mean_impulse = settled_impulses.iter().sum::<f64>() / settled_impulses.len() as f64;
    let expected_impulse = 2.0 * 9.81 * dt.as_seconds().value();
    assert!((mean_impulse - expected_impulse).abs() < 0.02);
}

#[test]
fn compiled_sensor_overlap_is_zero_impulse_contact() {
    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    let mut backend = MuJoCoBackend::new(dt).expect("compiled backend");
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let sensor_entity = spawn_named(&mut world, "sensor");
    let mut sensor = Collider::cuboid(Vec3::splat(0.5));
    sensor.sensor = true;
    world.entity_mut(sensor_entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        sensor,
        Transform3::default(),
    ));
    let solid_entity = spawn_named(&mut world, "solid");
    world.entity_mut(solid_entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::splat(0.5)),
        Transform3::from_translation_rotation(Vec3::new(0.25, 0.0, 0.0), Quat::IDENTITY),
    ));

    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("sync into MuJoCo");
    backend.step(physics_world, dt).expect("fixed step");
    let contacts = backend.contacts(physics_world).expect("sensor evidence");
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].entity_a, sensor_entity);
    assert_eq!(contacts[0].entity_b, solid_entity);
    assert_eq!(contacts[0].normal, Vec3::ZERO);
    assert_eq!(contacts[0].impulse, 0.0);
}
