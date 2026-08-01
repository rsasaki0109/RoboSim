//! Learns a state-feedback torque policy for the Go2 turn.
//!
//! Every prior searched controller on this platform is a *clock*: phase-indexed
//! joint offsets or torques replayed open-loop. This example closes the loop —
//! a linear [`UnitreeGo2TorquePolicy`] maps the measured body state
//! (yaw-invariant up-vector lean components, body-frame lean rates, world yaw
//! rate, two-cycle gait phase, bias) to per-joint feed-forward torques on the
//! torque-PD walk. The search is the ensemble-median CEM the chaos-horizon
//! measurements demanded: each candidate is scored by the median of three
//! ulp-perturbed replays, so knife-edge trajectories cannot win.
//! `--train` reproduces the search (seed 42); the default mode replays the
//! pinned winner headlessly; `--ensemble` prints its perturbation spread.

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
    score: f64,
}

/// Assembles the policy's feature vector from the live simulation state.
fn features(sim: &UrdfSceneSim, step: u64, cycle: u64) -> [f64; UNITREE_GO2_POLICY_FEATURES] {
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
        omega_body.z,
        omega_world.y,
        phase_sin,
        phase_cos,
        1.0,
    ]
}

fn rollout(policy: &UnitreeGo2TorquePolicy, steps: u64) -> RolloutOutcome {
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
    let cycle = walk_command().cycle_steps;
    for step in 0..steps {
        let targets = unitree_go2_trot_targets(step, walk_command());
        let feed_forward = policy.torques_nm(&features(&sim, step, cycle));
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
    let score = 2.0 * window_a_yaw_rad.min(window_b_yaw_rad)
        - if max_tilt_rad > 0.8 { 5.0 } else { 0.0 }
        - 20.0 * (0.15 - min_height_m).max(0.0);
    RolloutOutcome {
        window_a_yaw_rad,
        window_b_yaw_rad,
        forward_m,
        min_height_m,
        max_tilt_rad,
        score,
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

/// Ensemble score: the median min-window score of three replays whose first
/// weight differs by one part in 1e9 — the objective the chaos horizon
/// demands (a knife-edge trajectory scores what its perturbed neighbors do).
fn ensemble_score(params: &[f64; DIM]) -> f64 {
    let mut scores: Vec<f64> = (0..3)
        .map(|k| {
            let mut policy = policy_from(params);
            policy.weights[0][0] += k as f64 * 1.0e-9;
            rollout(&policy, ROLLOUT_STEPS).score
        })
        .collect();
    scores.sort_by(|a, b| a.partial_cmp(b).expect("finite scores"));
    scores[1]
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
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/go2_policy_cem_state.txt")
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
    let (start_iteration, mut mean, mut sigma, mut best) =
        load_state(&path).unwrap_or((0, [0.0; DIM], [1.0; DIM], (f64::MIN, [0.0; DIM])));
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
                    .map(|params| scope.spawn(move || ensemble_score(params)))
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
        let probe = rollout(&policy_from(&scored[0].1), ROLLOUT_STEPS);
        println!(
            "iter {iteration:2}: best median score {:.3} (windows {:+.3}/{:+.3} fwd {:.2} minH {:.3} maxTilt {:.2})",
            scored[0].0,
            probe.window_a_yaw_rad,
            probe.window_b_yaw_rad,
            probe.forward_m,
            probe.min_height_m,
            probe.max_tilt_rad
        );
        save_state(&path, &(iteration + 1, mean, sigma, best));
    }
    if end_iteration < ITERATIONS {
        println!("checkpointed at iteration {end_iteration}/{ITERATIONS}; run again to continue");
        return;
    }
    for k in 0..3 {
        let mut policy = policy_from(&best.1);
        policy.weights[0][0] += k as f64 * 1.0e-9;
        let outcome = rollout(&policy, ROLLOUT_STEPS);
        println!(
            "ensemble member {k}: windows {:+.3}/{:+.3} fwd {:.2} tilt {:.2}",
            outcome.window_a_yaw_rad,
            outcome.window_b_yaw_rad,
            outcome.forward_m,
            outcome.max_tilt_rad
        );
    }
    let final_outcome = rollout(&policy_from(&best.1), ROLLOUT_STEPS);
    println!(
        "final best: median score {:.3} windows {:+.3}/{:+.3} rad per 8 s",
        best.0, final_outcome.window_a_yaw_rad, final_outcome.window_b_yaw_rad
    );
    // Full 12-decimal precision: contact-gated rollouts diverge under
    // 6-decimal rounding, so the pinned constant must reproduce these digits.
    let policy = policy_from(&best.1);
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
    if std::env::args().any(|argument| argument == "--ensemble") {
        for perturbation in [0.0, 1.0e-9, 3.0e-9, 1.0e-6] {
            let mut policy = UnitreeGo2TorquePolicy::LEARNED_TURN;
            policy.weights[0][0] += perturbation;
            let outcome = rollout(&policy, ROLLOUT_STEPS);
            println!(
                "policy perturb {perturbation:.0e}: windows {:+.3}/{:+.3} fwd {:.2} tilt {:.2}",
                outcome.window_a_yaw_rad,
                outcome.window_b_yaw_rad,
                outcome.forward_m,
                outcome.max_tilt_rad
            );
        }
        return;
    }

    // Default: replay the pinned winner headlessly. The distinguishing claim
    // of the closed-loop policy is the operating point — a sustained turn
    // *while walking* — so both windows and forward progress are asserted.
    // Cross-platform, the exact rates are platform-local (persistent libm ulp
    // differences settle onto nearby orbits); the sustained turn is the bar.
    let outcome = rollout(&UnitreeGo2TorquePolicy::LEARNED_TURN, ROLLOUT_STEPS);
    assert!(
        outcome.window_a_yaw_rad > 0.08 && outcome.window_b_yaw_rad > 0.08,
        "policy turn must sustain both windows, got {:+.3}/{:+.3}",
        outcome.window_a_yaw_rad,
        outcome.window_b_yaw_rad
    );
    assert!(
        outcome.forward_m > 1.5,
        "policy turn must keep walking, got {:.2} m",
        outcome.forward_m
    );
    assert!(
        outcome.max_tilt_rad < 0.8 && outcome.min_height_m > 0.1,
        "policy turn must stay up: tilt {:.2} height {:.3}",
        outcome.max_tilt_rad,
        outcome.min_height_m
    );
    println!(
        "policy turn verified: windows {:+.3}/{:+.3} rad per 8 s, fwd {:.2} m, maxTilt {:.2}",
        outcome.window_a_yaw_rad, outcome.window_b_yaw_rad, outcome.forward_m, outcome.max_tilt_rad
    );
}
