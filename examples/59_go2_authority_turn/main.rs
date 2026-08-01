//! Raises the commanded turn's authority above the chaos floor.
//!
//! The ±8 N·m commanded policy (`58_go2_steered_turn`) obeys its yaw-rate
//! command, but only *differentially*: its ~0.1 rad windows sit inside the
//! ±0.3 rad spread that cross-OS libm orbit differences produce, so absolute
//! per-window obedience does not transfer between platforms. This example
//! attacks the margin with two levers: the feed-forward clamp rises to
//! ±12 N·m, and the feature vector gains an *integral* of the yaw-rate error
//! (a learned PI structure) in place of the least useful lean-rate component.
//! The search is the same worse-of-both-commanded-directions CEM,
//! warm-started from the ±8 winner. `--train` reproduces it (seed 42); the
//! default mode replays the pinned winner under both commands headlessly.

use std::fs;
use std::path::{Path, PathBuf};

use rne_ai::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand,
    UnitreeGo2TorquePolicy, UrdfJointTorqueTarget, UrdfSceneSim, UNITREE_GO2_POLICY_FEATURES,
};
use rne_math::Vec3;

const ROLLOUT_STEPS: u64 = 1440;
const WINDOW_START_STEP: u64 = 480;
const WINDOW_SPLIT_STEP: u64 = 960;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 40.0;
const KD: f64 = 0.5;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;
/// Feed-forward clamp for this search: the authority lever.
const POLICY_LIMIT_NM: f64 = 12.0;
/// Commanded yaw-rate magnitude the policy is trained and verified against.
const YAW_RATE_REF_RAD_S: f64 = 0.25;
/// Clamp on the integral feature so it cannot wind up without bound.
const INTEGRAL_CLAMP: f64 = 0.5;
const DIM: usize = 12 * UNITREE_GO2_POLICY_FEATURES;
const POPULATION: usize = 64;
const ELITE: usize = 16;
const ITERATIONS: usize = 30;

fn walk_command() -> UnitreeGo2GaitCommand {
    UnitreeGo2GaitCommand {
        stride_rad: 0.24,
        cycle_steps: 45,
        ..UnitreeGo2GaitCommand::default()
    }
}

struct RolloutOutcome {
    window_a_yaw_rad: f64,
    window_b_yaw_rad: f64,
    forward_m: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
}

/// Assembles the feature vector. Relative to example 58, feature 3 (the
/// body-Z lean rate, the least informative component there) becomes the
/// clamped *integral* of the yaw-rate error — the term that lets a linear
/// policy hold a persistent turning torque against a persistent error.
fn features(
    sim: &UrdfSceneSim,
    step: u64,
    cycle: u64,
    yaw_rate_ref_rad_s: f64,
    error_integral: f64,
) -> [f64; UNITREE_GO2_POLICY_FEATURES] {
    let pose = sim.named_transform("base").expect("base pose");
    let inverse = pose.rotation.inverse();
    let up_body = (inverse * Vec3::Y).normalize_or_zero();
    let observed = sim.observe();
    let omega_world = Vec3::new(
        observed.base_angular_velocity_x_rad_s,
        observed.base_angular_velocity_y_rad_s,
        observed.base_angular_velocity_z_rad_s,
    );
    let omega_body = inverse * omega_world;
    let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
    let (phase_sin, phase_cos) = (2.0 * std::f64::consts::PI * two_cycle_phase).sin_cos();
    [
        up_body.x,
        up_body.z,
        omega_body.x,
        error_integral,
        yaw_rate_ref_rad_s - omega_world.y,
        phase_sin,
        phase_cos,
        1.0,
    ]
}

fn rollout(policy: &UnitreeGo2TorquePolicy, yaw_rate_ref_rad_s: f64, steps: u64) -> RolloutOutcome {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(180.0, 18.0, TORQUE_LIMIT_NM);
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            ..walk_command()
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let up_body_reference = {
        let pose = sim.named_transform("base").expect("base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let true_tilt = |sim: &UrdfSceneSim| {
        let pose = sim.named_transform("base").expect("base pose");
        let up = (pose.rotation * up_body_reference).normalize_or_zero();
        up.y.clamp(-1.0, 1.0).acos()
    };
    let start = sim.observe();
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut window_a_yaw_rad = 0.0;
    let mut window_b_yaw_rad = 0.0;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad = 0.0_f64;
    let mut error_integral = 0.0_f64;
    let cycle = walk_command().cycle_steps;
    let dt = 1.0 / 60.0;
    for step in 0..steps {
        let targets = unitree_go2_trot_targets(step, walk_command());
        let feature_vector = features(&sim, step, cycle, yaw_rate_ref_rad_s, error_integral);
        let feed_forward = policy.torques_nm_with_limit(&feature_vector, POLICY_LIMIT_NM);
        let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
            .iter()
            .zip(feed_forward.iter())
            .map(|(target, extra)| {
                let q = sim
                    .named_joint_position(target.link_name)
                    .expect("joint position");
                let qd = sim
                    .named_joint_velocity(target.link_name)
                    .expect("joint velocity");
                UrdfJointTorqueTarget {
                    link_name: target.link_name,
                    torque_nm: (KP * (target.position - q) - KD * qd + extra)
                        .clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                    max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                }
            })
            .collect();
        sim.step_joint_torques(&torques);
        let observed = sim.observe();
        error_integral = (error_integral
            + (yaw_rate_ref_rad_s - observed.base_angular_velocity_y_rad_s) * dt)
            .clamp(-INTEGRAL_CLAMP, INTEGRAL_CLAMP);
        let mut delta = observed.base_relative_yaw_rad - previous_yaw;
        while delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        if (WINDOW_START_STEP..WINDOW_SPLIT_STEP).contains(&step) {
            window_a_yaw_rad += delta;
        } else if step >= WINDOW_SPLIT_STEP {
            window_b_yaw_rad += delta;
        }
        previous_yaw = observed.base_relative_yaw_rad;
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(true_tilt(&sim));
    }
    let end = sim.observe();
    let forward_m = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
    RolloutOutcome {
        window_a_yaw_rad,
        window_b_yaw_rad,
        forward_m,
        min_height_m,
        max_tilt_rad,
    }
}

fn policy_from(params: &[f64; DIM]) -> UnitreeGo2TorquePolicy {
    let mut weights = [[0.0; UNITREE_GO2_POLICY_FEATURES]; 12];
    for joint in 0..12 {
        for feature in 0..UNITREE_GO2_POLICY_FEATURES {
            weights[joint][feature] =
                params[joint * UNITREE_GO2_POLICY_FEATURES + feature].clamp(-8.0, 8.0);
        }
    }
    UnitreeGo2TorquePolicy { weights }
}

/// Warm-start mean: the ±8 winner's weights with the integral column zeroed
/// (its slot held the body-Z lean rate there).
fn warm_start_mean() -> [f64; DIM] {
    let mut mean = [0.0; DIM];
    for joint in 0..12 {
        for feature in 0..UNITREE_GO2_POLICY_FEATURES {
            let source = UnitreeGo2TorquePolicy::LEARNED_COMMANDED_TURN.weights[joint][feature];
            mean[joint * UNITREE_GO2_POLICY_FEATURES + feature] =
                if feature == 3 { 0.0 } else { source };
        }
    }
    mean
}

/// Score of one commanded direction: sign-corrected minimum window yaw with
/// the fall and crouch penalties of the previous searches.
fn direction_score(outcome: &RolloutOutcome, reference_sign: f64) -> f64 {
    2.0 * (reference_sign * outcome.window_a_yaw_rad).min(reference_sign * outcome.window_b_yaw_rad)
        - if outcome.max_tilt_rad > 0.8 { 5.0 } else { 0.0 }
        - 20.0 * (0.15 - outcome.min_height_m).max(0.0)
}

/// Commanded score: the worse of the two commanded directions.
fn commanded_score(params: &[f64; DIM]) -> f64 {
    let policy = policy_from(params);
    let positive = rollout(&policy, YAW_RATE_REF_RAD_S, ROLLOUT_STEPS);
    let negative = rollout(&policy, -YAW_RATE_REF_RAD_S, ROLLOUT_STEPS);
    direction_score(&positive, 1.0).min(direction_score(&negative, -1.0))
}

fn gaussian(rng: &mut rne_ai::DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

const ITERATIONS_PER_RUN: usize = 3;
const PARALLEL_ROLLOUTS: usize = 16;

type TrainState = (usize, [f64; DIM], [f64; DIM], (f64, [f64; DIM]));

fn state_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/go2_authority_cem_state.txt")
}

fn load_state(path: &Path) -> Option<TrainState> {
    let text = fs::read_to_string(path).ok()?;
    let values: Vec<f64> = text
        .split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    if values.len() != 2 + 3 * DIM {
        return None;
    }
    let mut mean = [0.0; DIM];
    let mut sigma = [0.0; DIM];
    let mut best_params = [0.0; DIM];
    mean.copy_from_slice(&values[1..1 + DIM]);
    sigma.copy_from_slice(&values[1 + DIM..1 + 2 * DIM]);
    best_params.copy_from_slice(&values[2 + 2 * DIM..2 + 3 * DIM]);
    Some((
        values[0] as usize,
        mean,
        sigma,
        (values[1 + 2 * DIM], best_params),
    ))
}

fn save_state(path: &Path, state: &TrainState) {
    let mut text = format!("{}\n", state.0);
    for value in state.1.iter().chain(state.2.iter()) {
        text.push_str(&format!("{value:.12}\n"));
    }
    text.push_str(&format!("{:.12}\n", state.3 .0));
    for value in state.3 .1.iter() {
        text.push_str(&format!("{value:.12}\n"));
    }
    fs::write(path, text).expect("write CEM state");
}

fn train() {
    let path = state_path();
    let (start_iteration, mut mean, mut sigma, mut best) = load_state(&path).unwrap_or((
        0,
        warm_start_mean(),
        [0.5; DIM],
        (f64::MIN, warm_start_mean()),
    ));
    let end_iteration = (start_iteration + ITERATIONS_PER_RUN).min(ITERATIONS);
    for iteration in start_iteration..end_iteration {
        // Sequential sampling from a per-iteration seed keeps the search
        // deterministic and resumable; only the physics rollouts parallelize.
        let mut rng = rne_ai::DeterministicRng::new(42 + iteration as u64);
        let population: Vec<[f64; DIM]> = (0..POPULATION)
            .map(|_| {
                let mut params = [0.0_f64; DIM];
                for (value, (m, s)) in params.iter_mut().zip(mean.iter().zip(sigma.iter())) {
                    *value = m + s * gaussian(&mut rng);
                }
                params
            })
            .collect();
        let mut scored: Vec<(f64, [f64; DIM])> = Vec::with_capacity(POPULATION);
        for chunk in population.chunks(PARALLEL_ROLLOUTS) {
            let scores = std::thread::scope(|scope| {
                let handles: Vec<_> = chunk
                    .iter()
                    .map(|params| scope.spawn(move || commanded_score(params)))
                    .collect();
                handles
                    .into_iter()
                    .map(|handle| handle.join().expect("rollout thread"))
                    .collect::<Vec<_>>()
            });
            for (score, params) in scores.into_iter().zip(chunk.iter()) {
                scored.push((score, *params));
            }
        }
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).expect("finite scores"));
        if scored[0].0 > best.0 {
            best = scored[0];
        }
        for dimension in 0..DIM {
            let elite_mean = scored[..ELITE]
                .iter()
                .map(|(_, params)| params[dimension])
                .sum::<f64>()
                / ELITE as f64;
            let elite_variance = scored[..ELITE]
                .iter()
                .map(|(_, params)| (params[dimension] - elite_mean).powi(2))
                .sum::<f64>()
                / ELITE as f64;
            mean[dimension] = elite_mean;
            sigma[dimension] = elite_variance.sqrt().max(0.1);
        }
        let policy = policy_from(&scored[0].1);
        let positive = rollout(&policy, YAW_RATE_REF_RAD_S, ROLLOUT_STEPS);
        let negative = rollout(&policy, -YAW_RATE_REF_RAD_S, ROLLOUT_STEPS);
        println!(
            "iter {iteration:2}: best commanded score {:.3} (+ref {:+.3}/{:+.3} fwd {:.2} | -ref {:+.3}/{:+.3} fwd {:.2})",
            scored[0].0,
            positive.window_a_yaw_rad,
            positive.window_b_yaw_rad,
            positive.forward_m,
            negative.window_a_yaw_rad,
            negative.window_b_yaw_rad,
            negative.forward_m
        );
        save_state(&path, &(iteration + 1, mean, sigma, best));
    }
    if end_iteration < ITERATIONS {
        println!("checkpointed at iteration {end_iteration}/{ITERATIONS}; run again to continue");
        return;
    }
    let policy = policy_from(&best.1);
    for reference in [YAW_RATE_REF_RAD_S, -YAW_RATE_REF_RAD_S] {
        let outcome = rollout(&policy, reference, ROLLOUT_STEPS);
        println!(
            "ref {reference:+.2}: windows {:+.3}/{:+.3} fwd {:.2} tilt {:.2}",
            outcome.window_a_yaw_rad,
            outcome.window_b_yaw_rad,
            outcome.forward_m,
            outcome.max_tilt_rad
        );
    }
    println!("final best: commanded score {:.3}", best.0);
    // Full 12-decimal precision: contact-gated rollouts diverge under
    // 6-decimal rounding, so the pinned constant must reproduce these digits.
    println!("weights: [");
    for row in policy.weights {
        let cells: Vec<String> = row.iter().map(|value| format!("{value:.12}")).collect();
        println!("    [{}],", cells.join(", "));
    }
    println!("],");
}

fn main() {
    if std::env::args().any(|argument| argument == "--train") {
        train();
        return;
    }

    // Default: replay the pinned NEGATIVE result. The authority hypothesis —
    // that a ±12 N·m clamp plus an integral error term lifts the commanded
    // turn above the ±0.3 rad chaos floor — is refuted: the winner's
    // sign-corrected windows stay far below the floor.
    let policy = UnitreeGo2TorquePolicy::LEARNED_AUTHORITY_TURN;
    let mut worst_obedient_window = f64::MAX;
    for reference in [YAW_RATE_REF_RAD_S, -YAW_RATE_REF_RAD_S] {
        let outcome = rollout(&policy, reference, ROLLOUT_STEPS);
        let sign = reference.signum();
        worst_obedient_window = worst_obedient_window
            .min(sign * outcome.window_a_yaw_rad)
            .min(sign * outcome.window_b_yaw_rad);
        assert!(
            outcome.max_tilt_rad < 0.8 && outcome.min_height_m > 0.1,
            "authority winner must stay up: tilt {:.2} height {:.3}",
            outcome.max_tilt_rad,
            outcome.min_height_m
        );
        println!(
            "authority ref {reference:+.2}: windows {:+.3}/{:+.3}, fwd {:.2} m, tilt {:.2}",
            outcome.window_a_yaw_rad,
            outcome.window_b_yaw_rad,
            outcome.forward_m,
            outcome.max_tilt_rad
        );
    }
    assert!(
        worst_obedient_window < 0.25,
        "the refutation must hold: authority did not clear the chaos floor, got {worst_obedient_window:+.3}"
    );
    println!("worst obedient window: {worst_obedient_window:+.3} rad (floor is ~0.3)");
}
