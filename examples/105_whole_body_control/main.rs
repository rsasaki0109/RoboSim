//! Runs the whole-body inverse-dynamics controller on the floating-base 12-DoF
//! Unitree Go2 with four point contacts at the feet.
//!
//! The first solve holds the center of mass and posture against gravity; the
//! second requests a forward center-of-mass acceleration to show the controller
//! using the leg redundancy. Everything is headless and deterministic.
//!
//! Run with `cargo run -p whole_body_control --example 105_whole_body_control`.

use rne_dynamics::{center_of_mass, ArticulatedModel};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};
use rne_wbc::{ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const TORQUE_LIMIT_NM: f64 = 23.7;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);

fn main() {
    let document = parse_urdf_document(GO2_URDF).expect("parse Go2 URDF");
    let mut world = World::new();
    let config = UrdfSpawnConfig {
        attach_colliders: false,
        attach_mesh_colliders: false,
        self_collisions: false,
        use_declared_inertial_masses: true,
        ..UrdfSpawnConfig::default()
    };
    let spawned =
        spawn_urdf_document_with_config(&mut world, &document, config).expect("spawn Go2");
    let base_link = world
        .get::<Robot>(spawned.robot)
        .expect("robot component")
        .base_link;
    // The vendored Go2 URDF is Z-up; rotate the base so the model's up axis
    // matches RNE's Y-up world before enabling the floating base.
    world.entity_mut(base_link).insert((
        rne_world::Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        ),
        FloatingBase,
    ));

    let model = ArticulatedModel::from_robot(&world, spawned.robot).expect("articulated model");
    let nv = model.nv();
    let nj = nv - model.base_dof();
    let q = vec![0.0; nv];
    let qd = vec![0.0; nv];

    let contacts: Vec<ContactPoint> = FOOT_LINKS
        .iter()
        .filter_map(|name| {
            model
                .kinematic()
                .link_entity_by_name(name)
                .map(|link| ContactPoint::new(link, SOLE_OFFSET_LOCAL_M, 0.6))
        })
        .collect();
    assert_eq!(contacts.len(), 4, "expected four foot contacts");

    let total_mass: f64 = (0..model.link_count())
        .filter_map(|index| model.link_inertia(index))
        .map(|inertia| inertia.mass_kg)
        .sum();
    let com = center_of_mass(&model, &q).expect("center of mass");

    let controller = WholeBodyController::new(WholeBodyConfig {
        torque_limits_nm: Some(vec![TORQUE_LIMIT_NM; nj]),
        ..WholeBodyConfig::default()
    });
    let posture = PostureTask {
        desired_joint_positions: q[model.base_dof()..].to_vec(),
        desired_joint_velocities: None,
        desired_joint_accelerations: None,
        position_gain_s_inv2: 100.0,
        velocity_gain_s_inv: 20.0,
    };

    let hold = ComTask::hold(com);
    let standing = controller
        .solve(
            &model,
            &q,
            &qd,
            &contacts,
            Some(&hold),
            None,
            Some(&posture),
        )
        .expect("standing solve");
    let vertical_force: f64 = standing
        .contact_forces_world_n
        .iter()
        .map(|force| force.y)
        .sum();
    let base_residual = norm6(&standing.base_wrench_residual);
    let contact_residual = standing
        .contact_acceleration_residual_m_s2
        .iter()
        .fold(0.0_f64, |maximum, value| maximum.max(*value));
    let max_torque = standing
        .joint_torque_nm
        .iter()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    println!(
        "standing: mass={total_mass:.2}kg vertical_force={vertical_force:.2}N (weight={:.2}N)",
        total_mass * 9.81
    );
    println!(
        "standing: base_residual={base_residual:.3e} contact_residual={contact_residual:.3e} max_torque={max_torque:.2}Nm saturated={}",
        standing.torque_saturated
    );

    let mut accelerate = ComTask::hold(com);
    accelerate.desired_acceleration_m_s2 = Vec3::new(1.0, 0.0, 0.0);
    let moving = controller
        .solve(
            &model,
            &q,
            &qd,
            &contacts,
            Some(&accelerate),
            None,
            Some(&posture),
        )
        .expect("accelerating solve");
    println!(
        "com task: requested_x=1.000 achieved=({:.3}, {:.3}, {:.3}) m/s²",
        moving.com_acceleration_m_s2.x,
        moving.com_acceleration_m_s2.y,
        moving.com_acceleration_m_s2.z
    );

    let again = controller
        .solve(
            &model,
            &q,
            &qd,
            &contacts,
            Some(&accelerate),
            None,
            Some(&posture),
        )
        .expect("accelerating solve");
    let deterministic = moving == again;

    let standing_ok = (vertical_force - total_mass * 9.81).abs() < 1.0
        && base_residual < 1.0
        && contact_residual < 1.0e-3
        && max_torque <= TORQUE_LIMIT_NM + 1.0e-9;
    let task_ok = moving.com_acceleration_m_s2.x > 0.5;
    println!("deterministic={deterministic} standing_ok={standing_ok} task_ok={task_ok}");
    if !(standing_ok && task_ok && deterministic) {
        eprintln!("whole-body control diagnostics failed");
        std::process::exit(1);
    }
    println!("whole-body control diagnostics: ok");
}

fn norm6(values: &[f64; 6]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}
