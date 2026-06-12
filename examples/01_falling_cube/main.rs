//! Falling cube under gravity with Rapier physics.

use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;

fn main() {
    let mut backend = RapierBackend::new();
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
        Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
    ));

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    backend.sync_from_ecs(&mut world, physics_world).unwrap();

    for step in 0..120 {
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        if step % 30 == 29 {
            let y = world.get::<Transform3>(cube).unwrap().translation.y;
            println!("step {}: cube y = {y:.3} m", step + 1);
        }
    }

    let hits = backend
        .raycast(
            physics_world,
            rne_physics::RaycastQuery::downward(Vec3::new(0.0, 5.0, 0.0), 20.0),
        )
        .unwrap();
    println!("raycast hits = {}", hits.len());
    println!("state hash = {:016x}", hash_physics_state(&world));
}
