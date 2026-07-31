//! Searches torque space for the Go2 turn that joint-space control cannot find.
//!
//! `docs/GO2_LOCOMOTION.md` measures nine hand-designed steering mechanisms
//! across two actuation regimes — six in position space, three at force level —
//! and none sustains a turn on this 3-DoF-per-leg platform; three learned
//! position-space searches plateau at ~0.02 rad/s. This example runs the same
//! deterministic, resumable, parallel cross-entropy search in *torque space*:
//! contact-gated Fourier feed-forward torques
//! ([`UnitreeGo2TorqueOverlay`], 72 coefficients) on top of the low-bandwidth
//! torque-PD walk (kp 40 / kd 0.5, the gains the 60 Hz discrete stability
//! bound allows), with the anti-cheat minimum-of-two-late-windows yaw
//! objective. `--train` reproduces the search (seed 42); the default mode
//! replays the pinned winner headlessly and verifies the measurement.

use std::fs;
use std::path::{Path, PathBuf};

use rne_ai::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand,
    UnitreeGo2TorqueOverlay, UrdfJointTorqueTarget, UrdfSceneSim,
};

const ROLLOUT_STEPS: u64 = 1440;
const WINDOW_START_STEP: u64 = 480;
const WINDOW_SPLIT_STEP: u64 = 960;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 40.0;
const KD: f64 = 0.5;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;
const DIM: usize = 72;
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

fn rollout(overlay: &UnitreeGo2TorqueOverlay, steps: u64) -> RolloutOutcome {
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
    // Yaw-invariant tilt: the Euler observations wrap between branches once the
    // body yaws, so tilt is the angle between the settled body-up direction and
    // world up.
    let up_body = {
        let pose = sim.named_transform("base").expect("base pose");
        (pose.rotation.inverse() * rne_math::Vec3::Y).normalize_or_zero()
    };
    let true_tilt = |sim: &UrdfSceneSim| {
        let pose = sim.named_transform("base").expect("base pose");
        let up = (pose.rotation * up_body).normalize_or_zero();
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
        let stance = [
            sim.link_contact_impulse_ns("FL_foot") > 0.0,
            sim.link_contact_impulse_ns("FR_foot") > 0.0,
            sim.link_contact_impulse_ns("RL_foot") > 0.0,
            sim.link_contact_impulse_ns("RR_foot") > 0.0,
        ];
        let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
        let feed_forward = overlay.torques_nm(two_cycle_phase, stance);
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
    // The anti-cheat objective from the position-space searches, with the
    // crouch threshold adapted to the torque walk's natural ride height
    // (baseline min height 0.178 versus the servo walk's 0.19+).
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

fn overlay_from(params: &[f64; DIM]) -> UnitreeGo2TorqueOverlay {
    let mut coefficients = [[0.0; 6]; 12];
    for joint in 0..12 {
        coefficients[joint][0] = params[joint * 6].clamp(-6.0, 6.0);
        for term in 1..6 {
            coefficients[joint][term] = params[joint * 6 + term].clamp(-4.0, 4.0);
        }
    }
    UnitreeGo2TorqueOverlay { coefficients }
}

fn gaussian(rng: &mut rne_ai::DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

const ITERATIONS_PER_RUN: usize = 6;
const PARALLEL_ROLLOUTS: usize = 16;

type TrainState = (usize, [f64; DIM], [f64; DIM], (f64, [f64; DIM]));

fn state_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/go2_torque_cem_state.txt")
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
                    .map(|params| {
                        scope.spawn(move || rollout(&overlay_from(params), ROLLOUT_STEPS).score)
                    })
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
        let probe = rollout(&overlay_from(&scored[0].1), ROLLOUT_STEPS);
        println!(
            "iter {iteration:2}: best score {:.3} (windows {:+.3}/{:+.3} fwd {:.2} minH {:.3} maxTilt {:.2})",
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
        println!("checkpointed at iteration {end_iteration}/{ITERATIONS}; run --train again");
        return;
    }
    let final_outcome = rollout(&overlay_from(&best.1), ROLLOUT_STEPS);
    println!(
        "final best: score {:.3} windows {:+.3}/{:+.3} rad per 8 s ({:.3} rad/s sustained)",
        best.0,
        final_outcome.window_a_yaw_rad,
        final_outcome.window_b_yaw_rad,
        final_outcome
            .window_a_yaw_rad
            .min(final_outcome.window_b_yaw_rad)
            / 8.0
    );
    // Print at the state file's full 12-decimal precision: the contact-gated
    // rollout is chaotic enough that 6-decimal rounding lands on a different
    // trajectory, so the pinned constant must reproduce these digits exactly.
    let overlay = overlay_from(&best.1);
    println!("coefficients: [");
    for coefficient in overlay.coefficients {
        println!(
            "    [{:.12}, {:.12}, {:.12}, {:.12}, {:.12}, {:.12}],",
            coefficient[0],
            coefficient[1],
            coefficient[2],
            coefficient[3],
            coefficient[4],
            coefficient[5]
        );
    }
    println!("],");
}

fn main() {
    if std::env::args().any(|argument| argument == "--train") {
        train();
        return;
    }

    // Default: replay the pinned winner headlessly and verify the plateau
    // break — sustained yaw above the position-space searches' honest bar in
    // both late windows, still walking, still upright.
    let outcome = rollout(&UnitreeGo2TorqueOverlay::LEARNED_TURN, ROLLOUT_STEPS);
    assert!(
        outcome.window_a_yaw_rad > 0.2 && outcome.window_b_yaw_rad > 0.2,
        "learned torque turn must sustain the plateau break, windows {:+.3}/{:+.3}",
        outcome.window_a_yaw_rad,
        outcome.window_b_yaw_rad
    );
    assert!(
        outcome.max_tilt_rad < 0.8 && outcome.min_height_m > 0.1,
        "learned torque turn must stay up: tilt {:.2} height {:.3}",
        outcome.max_tilt_rad,
        outcome.min_height_m
    );
    println!(
        "learned torque turn verified: windows {:+.3}/{:+.3} rad per 8 s ({:.3} rad/s sustained), fwd {:.2} m, maxTilt {:.2}",
        outcome.window_a_yaw_rad,
        outcome.window_b_yaw_rad,
        outcome
            .window_a_yaw_rad
            .min(outcome.window_b_yaw_rad)
            / 8.0,
        outcome.forward_m,
        outcome.max_tilt_rad
    );
}
