//! Compare Rapier stepped trajectories against Pinocchio golden references.

use approx::assert_relative_eq;
use rne_dynamics_tests::{
    planar_angle_from_vertical_rad, spawn_pendulum, spawn_two_link_planar, PhysicsHarness,
};
use rne_math::Vec3;
use rne_physics::PhysicsWorldDesc;
use serde::Deserialize;
use std::path::PathBuf;

/// Maximum |tip_sim − tip_ref| for single pendulum (m).
const PINOCCHIO_SINGLE_TIP_EPS_M: f64 = 0.12;

/// Maximum |tip_sim − tip_ref| for multi-link planar chains (m).
const PINOCCHIO_MULTI_TIP_EPS_M: f64 = 0.18;

/// Relative tolerance on tip distance from origin (guards scale drift).
const PINOCCHIO_TIP_REL_EPS: f64 = 0.08;

/// Joint-angle slack for revolute DOFs (rad).
///
/// Constraint stabilization biases angle by a few milliradians per step; 0.15 rad
/// (~8.6°) is loose but still rejects gross integrator or axis sign errors.
const PINOCCHIO_JOINT_ANGLE_EPS_RAD: f64 = 0.15;

#[derive(Debug, Deserialize)]
struct GoldenTrajectory {
    hz: f64,
    #[allow(dead_code)]
    duration_s: f64,
    parameters: serde_json::Value,
    samples: Vec<GoldenSample>,
}

#[derive(Debug, Deserialize)]
struct GoldenSample {
    step: u32,
    #[allow(dead_code)]
    t_s: f64,
    joint_q_rad: Vec<f64>,
    tip_m: [f64; 3],
}

fn golden_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/dynamics")
        .join(format!("pinocchio_{name}.json"))
}

fn load_golden(name: &str) -> GoldenTrajectory {
    let path = golden_path(name);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("read golden {}: {error}", path.display()));
    serde_json::from_str(&text).unwrap_or_else(|error| {
        panic!("parse golden {}: {error}", path.display());
    })
}

fn assert_tip_near(reference: [f64; 3], simulated: Vec3, label: &str, eps_m: f64) {
    let ref_vec = Vec3::new(reference[0], reference[1], reference[2]);
    let err = (simulated - ref_vec).length();
    assert!(
        err < eps_m,
        "{label}: tip error {err:.4} m exceeds {eps_m} m (sim={simulated:?} ref={ref_vec:?})"
    );
    let ref_len = ref_vec.length();
    if ref_len > 1e-6 {
        assert_relative_eq!(
            simulated.length(),
            ref_len,
            max_relative = PINOCCHIO_TIP_REL_EPS
        );
    }
}

fn assert_angles_near(reference: &[f64], simulated: &[f64], label: &str, eps_rad: f64) {
    assert_eq!(reference.len(), simulated.len());
    for (i, (r, s)) in reference.iter().zip(simulated.iter()).enumerate() {
        let err = (s - r).abs();
        assert!(
            err < eps_rad,
            "{label} joint[{i}]: angle error {err:.4} rad exceeds {eps_rad} rad (sim={s:.4} ref={r:.4})"
        );
    }
}

#[test]
fn single_pendulum_tracks_pinocchio_golden() {
    let golden = load_golden("single_pendulum");
    let length_m = golden.parameters["length_m"].as_f64().expect("length_m");
    let mass_kg = golden.parameters["mass_kg"].as_f64().expect("mass_kg");
    let radius_m = golden.parameters["radius_m"].as_f64().expect("radius_m");
    let initial_angle_rad = golden.parameters["initial_angle_rad"]
        .as_f64()
        .expect("initial_angle_rad");

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    let (_pivot, bob) =
        spawn_pendulum(&mut harness, length_m, initial_angle_rad, mass_kg, radius_m);

    let hz = golden.hz;
    for sample in &golden.samples {
        while harness.steps < sample.step {
            harness.step_hz(hz, 1);
        }
        let bob_pos = harness.translation(bob);
        let angle = planar_angle_from_vertical_rad(Vec3::ZERO, bob_pos);
        assert_angles_near(
            &sample.joint_q_rad,
            &[angle],
            "single_pendulum",
            PINOCCHIO_JOINT_ANGLE_EPS_RAD,
        );
        assert_tip_near(
            sample.tip_m,
            bob_pos,
            "single_pendulum",
            PINOCCHIO_SINGLE_TIP_EPS_M,
        );
    }
}

#[test]
fn two_link_planar_tracks_pinocchio_golden() {
    let golden = load_golden("two_link_planar");
    let l1 = golden.parameters["link1_length_m"]
        .as_f64()
        .expect("link1_length_m");
    let l2 = golden.parameters["link2_length_m"]
        .as_f64()
        .expect("link2_length_m");
    let m1 = golden.parameters["link1_mass_kg"]
        .as_f64()
        .expect("link1_mass_kg");
    let m2 = golden.parameters["link2_mass_kg"]
        .as_f64()
        .expect("link2_mass_kg");
    let r1 = golden.parameters["link1_radius_m"]
        .as_f64()
        .expect("link1_radius_m");
    let q0 = golden.parameters["initial_q_rad"]
        .as_array()
        .expect("initial_q_rad");
    let q1 = q0[0].as_f64().expect("q1");
    let q2 = q0[1].as_f64().expect("q2");

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    let (_base, _link1, link2) = spawn_two_link_planar(&mut harness, l1, l2, m1, m2, r1, q1, q2);

    let hz = golden.hz;
    for sample in &golden.samples {
        while harness.steps < sample.step {
            harness.step_hz(hz, 1);
        }
        let tip = harness.translation(link2);
        assert_tip_near(
            sample.tip_m,
            tip,
            "two_link_planar",
            PINOCCHIO_MULTI_TIP_EPS_M,
        );
    }
}
