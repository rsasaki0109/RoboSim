//! Deterministic collision bake and a physics probe against the baked collider.
//!
//! An L-shaped prism (a concave mesh) is decomposed into a compound of convex
//! boxes, written as an `.rne.collision.json` sidecar, then used as a static
//! Rapier collider while a sphere is dropped onto it.

use rne_collision_bake::{bake_voxel_decomposition, save_bake, VoxelBakeConfig};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{Collider, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;

/// L-shaped polygon extruded along Z, in the mesh's local frame.
fn l_prism() -> (Vec<[f32; 3]>, Vec<u32>) {
    let positions = vec![
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 2.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 1.0],
        [2.0, 0.0, 1.0],
        [2.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 2.0, 1.0],
        [0.0, 2.0, 1.0],
    ];
    let indices = vec![
        0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, // bottom
        6, 8, 7, 6, 9, 8, 6, 10, 9, 6, 11, 10, // top
        0, 1, 7, 0, 7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, // sides
        3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5, 6, 11,
    ];
    (positions, indices)
}

fn main() {
    let (positions, indices) = l_prism();
    let config = VoxelBakeConfig {
        max_cells_per_axis: 12,
        max_parts: 2048,
    };
    let bake = bake_voxel_decomposition(&positions, &indices, config).expect("bake");
    assert!(
        bake.part_count >= 2,
        "a concave mesh must decompose into more than one convex part"
    );
    println!(
        "baked {} convex boxes from {} triangles",
        bake.part_count, bake.source_triangle_count
    );

    let out = std::path::Path::new("artifacts/collision-bake/l_prism.rne.collision.json");
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent).expect("create artifact directory");
    }
    save_bake(out, &bake).expect("write collision sidecar");
    println!("wrote {}", out.display());

    // Drop a sphere onto the baked static collider near the L's notch.
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();

    let ground = spawn_named(&mut world, "baked_static");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: bake.shape.clone(),
            ..Collider::default()
        },
        Transform3::IDENTITY,
    ));

    let sphere = spawn_named(&mut world, "probe_sphere");
    world.entity_mut(sphere).insert((
        RigidBody::default(),
        Collider::sphere(0.1),
        Transform3::from_translation_rotation(Vec3::new(1.5, 2.0, 0.5), Quat::IDENTITY),
    ));

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    backend.sync_from_ecs(&mut world, physics_world).unwrap();
    for _ in 0..180 {
        step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
    }

    let rest_y = world
        .get::<Transform3>(sphere)
        .expect("sphere transform")
        .translation
        .y;
    println!("sphere rest y = {rest_y:.3} m on the baked collider");
}
