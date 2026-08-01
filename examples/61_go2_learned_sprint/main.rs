//! Learns a Go2 walk that beats the hand-scripted trot at its own job.
//!
//! This opens the learned-locomotion chapter: the steering campaign proved
//! the torque pathway, the ensemble-median objective, and the chaos-floor
//! discipline; this example points them at *forward speed* for the first
//! time. The search space is the proven contact-gated torque overlay
//! ([`UnitreeGo2TorqueOverlay`]) on the torque-PD walk; the objective is the
//! **minimum forward displacement over two disjoint late windows** (an
//! anti-cheat structure inherited from the yaw campaign: a dive or stumble
//! scores its bad window), with penalties for falling, crouching, and yaw
//! drift so the winner must *walk straight*, not tumble forward. Each
//! candidate is scored by the median of three ulp-perturbed replays so
//! knife-edge trajectories cannot win. `--train` reproduces the search
//! (seed 42); the default mode replays the pinned winner headlessly against
//! the zero-overlay baseline.

use std::fs;
use std::path::{Path, PathBuf};

use rne_ai::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand,
    UnitreeGo2TorqueOverlay, UrdfJointTorqueTarget, UrdfSceneSim,
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
    window_a_forward_m: f64,
    window_b_forward_m: f64,
    total_forward_m: f64,
    total_yaw_rad: f64,
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
    // Transport is scored as straight-line displacement between window
    // boundaries: lateral shimmy scores nothing, only genuinely covered
    // ground counts.
    let mut window_start_position = [start.base_x_m, start.base_z_m];
    let mut window_a_forward_m = 0.0;
    let mut window_b_forward_m = 0.0;
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut total_yaw_rad = 0.0;
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
        let mut yaw_delta = observed.base_relative_yaw_rad - previous_yaw;
        while yaw_delta > std::f64::consts::PI {
            yaw_delta -= 2.0 * std::f64::consts::PI;
        }
        while yaw_delta < -std::f64::consts::PI {
            yaw_delta += 2.0 * std::f64::consts::PI;
        }
        total_yaw_rad += yaw_delta;
        previous_yaw = observed.base_relative_yaw_rad;
        if step + 1 == WINDOW_START_STEP {
            window_start_position = [observed.base_x_m, observed.base_z_m];
        } else if step + 1 == WINDOW_SPLIT_STEP {
            window_a_forward_m = (observed.base_x_m - window_start_position[0])
                .hypot(observed.base_z_m - window_start_position[1]);
            window_start_position = [observed.base_x_m, observed.base_z_m];
        } else if step + 1 == steps {
            window_b_forward_m = (observed.base_x_m - window_start_position[0])
                .hypot(observed.base_z_m - window_start_position[1]);
        }
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(true_tilt(&sim));
    }
    let end = sim.observe();
    let total_forward_m = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
    // Anti-cheat speed score: the minimum straight-line displacement over two
    // disjoint late windows (a dive or stumble scores its bad window, shimmy
    // scores nothing), with the campaign's fall and crouch penalties plus a
    // straightness penalty so the winner walks, not spirals.
    let score = 2.0 * window_a_forward_m.min(window_b_forward_m)
        - if max_tilt_rad > 0.8 { 5.0 } else { 0.0 }
        - 20.0 * (0.15 - min_height_m).max(0.0)
        - 0.5 * total_yaw_rad.abs();
    RolloutOutcome {
        window_a_forward_m,
        window_b_forward_m,
        total_forward_m,
        total_yaw_rad,
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

/// Ensemble score: the median score of three ulp-perturbed replays — the
/// chaos-floor discipline from the steering campaign.
fn ensemble_score(params: &[f64; DIM]) -> f64 {
    let mut scores: Vec<f64> = (0..3)
        .map(|k| {
            let mut overlay = overlay_from(params);
            overlay.coefficients[0][0] += k as f64 * 1.0e-9;
            rollout(&overlay, ROLLOUT_STEPS).score
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
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/go2_sprint_cem_state.txt")
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
        let probe = rollout(&overlay_from(&scored[0].1), ROLLOUT_STEPS);
        println!(
            "iter {iteration:2}: best median score {:.3} (windows {:.2}/{:.2} m total {:.2} m yaw {:+.2} minH {:.3} maxTilt {:.2})",
            scored[0].0,
            probe.window_a_forward_m,
            probe.window_b_forward_m,
            probe.total_forward_m,
            probe.total_yaw_rad,
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
        let mut overlay = overlay_from(&best.1);
        overlay.coefficients[0][0] += k as f64 * 1.0e-9;
        let outcome = rollout(&overlay, ROLLOUT_STEPS);
        println!(
            "ensemble member {k}: windows {:.2}/{:.2} m total {:.2} m yaw {:+.2} tilt {:.2}",
            outcome.window_a_forward_m,
            outcome.window_b_forward_m,
            outcome.total_forward_m,
            outcome.total_yaw_rad,
            outcome.max_tilt_rad
        );
    }
    let final_outcome = rollout(&overlay_from(&best.1), ROLLOUT_STEPS);
    println!(
        "final best: median score {:.3} total {:.2} m over 24 s ({:.3} m/s)",
        best.0,
        final_outcome.total_forward_m,
        final_outcome.total_forward_m / 24.0
    );
    // Full 12-decimal precision: contact-gated rollouts diverge under
    // 6-decimal rounding, so the pinned constant must reproduce these digits.
    let overlay = overlay_from(&best.1);
    println!("coefficients: [");
    for row in overlay.coefficients {
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

    // Default: the comparison verify — the learned overlay must out-walk the
    // zero-overlay baseline decisively while staying straight and upright.
    let baseline = rollout(&UnitreeGo2TorqueOverlay::ZERO, ROLLOUT_STEPS);
    let sprint = rollout(&UnitreeGo2TorqueOverlay::LEARNED_SPRINT, ROLLOUT_STEPS);
    println!(
        "baseline: windows {:.2}/{:.2} m total {:.2} m ({:.3} m/s)",
        baseline.window_a_forward_m,
        baseline.window_b_forward_m,
        baseline.total_forward_m,
        baseline.total_forward_m / 24.0,
    );
    println!(
        "sprint:   windows {:.2}/{:.2} m total {:.2} m ({:.3} m/s), yaw {:+.2}, minH {:.3}, tilt {:.2}",
        sprint.window_a_forward_m,
        sprint.window_b_forward_m,
        sprint.total_forward_m,
        sprint.total_forward_m / 24.0,
        sprint.total_yaw_rad,
        sprint.min_height_m,
        sprint.max_tilt_rad
    );
    let baseline_min = baseline.window_a_forward_m.min(baseline.window_b_forward_m);
    let sprint_min = sprint.window_a_forward_m.min(sprint.window_b_forward_m);
    assert!(
        sprint_min > 1.4 * baseline_min && sprint_min > 2.8,
        "the learned overlay must out-walk the trot: {sprint_min:.2} m vs baseline {baseline_min:.2} m"
    );
    assert!(
        sprint.total_yaw_rad.abs() < 0.4,
        "the sprint must stay straight, yaw {:+.2}",
        sprint.total_yaw_rad
    );
    assert!(
        sprint.max_tilt_rad < 0.8 && sprint.min_height_m > 0.15,
        "the sprint must stay up: tilt {:.2} height {:.3}",
        sprint.max_tilt_rad,
        sprint.min_height_m
    );
}
