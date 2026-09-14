//! Exercises the native articulated dynamics layer on the 12-DoF Unitree Go2.
//!
//! The example spawns the RNE-adapted Go2 URDF with its declared inertial
//! properties, enables the floating base, builds a backend-neutral
//! [`rne_dynamics::ArticulatedModel`], and evaluates the mass matrix, gravity
//! force, center of mass, and a free-fall forward-dynamics step. Everything is
//! headless and deterministic.
//!
//! Run with `cargo run -p dynamics_diagnostics --example 103_dynamics_diagnostics`.

use rne_dynamics::{
    center_of_mass, forward_dynamics, gravity_torque, mass_matrix, ArticulatedModel,
};
use rne_ecs::World;
use rne_robot::{FloatingBase, Robot};
use rne_urdf_import::{parse_urdf_document, spawn_urdf_document_with_config, UrdfSpawnConfig};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");

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
    world.entity_mut(base_link).insert(FloatingBase);

    let model = ArticulatedModel::from_robot(&world, spawned.robot).expect("articulated model");
    let nv = model.nv();
    println!(
        "go2: links={} nv={} base_dof={}",
        model.link_count(),
        nv,
        model.base_dof()
    );

    let q = vec![0.0; nv];
    let qd = vec![0.0; nv];
    let matrix = mass_matrix(&model, &q).expect("mass matrix");
    let mut symmetric = true;
    let mut positive_diagonal = true;
    for row in 0..nv {
        if matrix.get(row, row) <= 0.0 {
            positive_diagonal = false;
        }
        for col in 0..nv {
            if (matrix.get(row, col) - matrix.get(col, row)).abs() > 1.0e-9 {
                symmetric = false;
            }
        }
    }

    let gravity = gravity_torque(&model, &q).expect("gravity");
    let gravity_norm = gravity
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    let com = center_of_mass(&model, &q).expect("center of mass");

    let tau = vec![0.0; nv];
    let qdd = forward_dynamics(&model, &q, &qd, &tau).expect("forward dynamics");

    // Consistency of the equation of motion: M qdd + C qd + g = tau = 0.
    let residual: f64 = {
        let inertia_term = matrix.mul_vec(&qdd);
        inertia_term
            .iter()
            .zip(gravity.iter())
            .map(|(left, right)| (left + right).abs())
            .fold(0.0_f64, f64::max)
    };

    println!("mass matrix {nv}x{nv}: symmetric={symmetric} positive_diagonal={positive_diagonal}");
    println!("gravity force norm = {gravity_norm:.3} N / N·m");
    println!(
        "center of mass = ({:.4}, {:.4}, {:.4}) m",
        com.x, com.y, com.z
    );
    println!(
        "free-fall base acceleration = ({:.3}, {:.3}, {:.3}) m/s²",
        qdd[0], qdd[1], qdd[2]
    );
    println!("equation-of-motion residual = {residual:.3e}");

    let finite = qdd.iter().all(|value| value.is_finite()) && com.is_finite();
    if !(symmetric && positive_diagonal && finite && residual < 1.0e-9) {
        eprintln!("dynamics diagnostics failed");
        std::process::exit(1);
    }
    println!("dynamics diagnostics: ok");
}
