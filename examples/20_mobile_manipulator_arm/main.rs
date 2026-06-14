//! Minimal URDF arm articulation under Rapier (Phase A mobile manipulator).

use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{Collider, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_urdf_import::{
    attach_urdf_articulation, parse_urdf, spawn_urdf_robot_with_config, UrdfArticulationConfig,
    UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};

const URDF: &str = include_str!("../../crates/rne_urdf_import/tests/fixtures/mm_minimal_arm.urdf");
const MIN_FOREARM_DISPLACEMENT_M: f64 = 0.01;

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut world = World::new();
    let urdf = parse_urdf(URDF).expect("parse mm_minimal arm URDF");
    let spawned = spawn_urdf_robot_with_config(
        &mut world,
        &urdf,
        UrdfSpawnConfig {
            base_body_type: RigidBodyType::Fixed,
            ..UrdfSpawnConfig::default()
        },
    )
    .expect("spawn URDF arm");

    let attached = attach_urdf_articulation(
        &mut world,
        &urdf,
        &spawned,
        UrdfArticulationConfig::default(),
    )
    .expect("attach articulation");
    assert_eq!(attached.revolute_joints, 4);

    world
        .entity_mut(spawned.base_link)
        .insert(Transform3::from_translation_rotation(
            Vec3::new(0.0, 0.3, 0.0),
            Quat::IDENTITY,
        ));

    let ground = spawn_named(&mut world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::new(10.0, 0.05, 10.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.05, 0.0), Quat::IDENTITY),
    ));

    let upper_arm = spawned.links["upper_arm_link"];
    let forearm = spawned.links["forearm_link"];
    let initial_forearm = world_transform_of(&world, forearm).translation;

    world
        .get_mut::<rne_physics::JointMotor>(upper_arm)
        .expect("shoulder motor")
        .velocity_rad_s = 3.0;

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let dt = SimDuration::from_hertz(Hertz::new(60.0));

    for _ in 0..480 {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("physics step");
    }

    let displacement = (world_transform_of(&world, forearm).translation - initial_forearm).length();

    if smoke {
        if displacement < MIN_FOREARM_DISPLACEMENT_M {
            eprintln!("smoke failed: forearm displacement={displacement:.4} m");
            std::process::exit(1);
        }
        println!("smoke ok: forearm displacement={displacement:.4} m");
        return;
    }

    println!("revolute joints wired = {}", attached.revolute_joints);
    println!("forearm displacement = {displacement:.4} m");
}
