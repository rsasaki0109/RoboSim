//! Replays and optionally learns a faster G1 walk that actually covers ground.
//!
//! The scripted G1 gait is a near-stationary stepper across its entire
//! stable envelope (measured: ≤0.1 m of transport per 12 s at any stride
//! that stands; 0.15 rad falls). This example points the Go2 transport
//! search at the humanoid: a contact-gated Fourier torque overlay
//! ([`UnitreeG1TorqueOverlay`], 48 coefficients on the eight proximal
//! joints) rides the hybrid walking tick — ankles and arms servo-follow the
//! stepper, hips and knees track it through torque PD (kp 300 / kd 10, the
//! humanoid-scale gains) plus the learned feed-forward. The objective is
//! the anti-cheat minimum window displacement with fall, tilt, and
//! straightness penalties, each candidate scored by the median of three
//! ulp-perturbed replays. `--train` reproduces the search (seed 42); the
//! default mode replays the pinned speed-up winner against the stepper baseline.

use std::fs;
use std::path::{Path, PathBuf};

use rne_ai::{
    unitree_g1_dynamic_scene_path, unitree_g1_gait_targets, UnitreeG1GaitCommand,
    UnitreeG1TorqueOverlay, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::Vec3;

const ROLLOUT_STEPS: u64 = 1440;
const WINDOW_START_STEP: u64 = 480;
const WINDOW_SPLIT_STEP: u64 = 960;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 300.0;
const KD: f64 = 10.0;
const TORQUE_LIMIT_NM: f64 = 88.0;
const SPEED_LIMIT_RAD_S: f64 = 30.0;
const WALK_STRIDE_RAD: f64 = 0.065;
const WALK_FOOT_LIFT_RAD: f64 = 0.12;
const WALK_CYCLE_STEPS: u64 = 100;
const SPEEDUP_MIN_WINDOW_M: f64 = 0.20;
const DIM: usize = 48;
const POPULATION: usize = 64;
const ELITE: usize = 16;
const ITERATIONS: usize = 30;

const TORQUE_LINKS: [&str; 8] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
];

fn walk_command() -> UnitreeG1GaitCommand {
    UnitreeG1GaitCommand {
        stride_rad: WALK_STRIDE_RAD,
        foot_lift_rad: WALK_FOOT_LIFT_RAD,
        cycle_steps: WALK_CYCLE_STEPS,
    }
}

#[derive(Clone, Copy, Debug)]
struct RolloutOutcome {
    window_a_forward_m: f64,
    window_b_forward_m: f64,
    total_forward_m: f64,
    total_yaw_rad: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
    score: f64,
}

fn settle(sim: &mut UrdfSceneSim, cycle_steps: u64) {
    sim.configure_position_motors(220.0, 24.0, TORQUE_LIMIT_NM);
    let stand = unitree_g1_gait_targets(
        0,
        UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps,
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
}

fn rollout(
    command: UnitreeG1GaitCommand,
    overlay: &UnitreeG1TorqueOverlay,
    steps: u64,
) -> RolloutOutcome {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("load dynamic G1");
    settle(&mut sim, command.cycle_steps);
    let up_body_reference = {
        let pose = sim.named_transform("pelvis").expect("pelvis pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let true_tilt = |sim: &UrdfSceneSim| {
        let pose = sim.named_transform("pelvis").expect("pelvis pose");
        let up = (pose.rotation * up_body_reference).normalize_or_zero();
        up.y.clamp(-1.0, 1.0).acos()
    };
    let start = sim.observe();
    let mut window_start_position = [start.base_x_m, start.base_z_m];
    let mut window_a_forward_m = 0.0;
    let mut window_b_forward_m = 0.0;
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut total_yaw_rad = 0.0;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad = 0.0_f64;
    let cycle = command.cycle_steps;
    for step in 0..steps {
        let targets = unitree_g1_gait_targets(step, command);
        let servo: Vec<UrdfJointPositionTarget<'_>> = targets
            .iter()
            .filter(|target| !TORQUE_LINKS.contains(&target.link_name))
            .copied()
            .collect();
        sim.set_joint_position_targets(&servo);
        let stance = [
            sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0,
            sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0,
        ];
        let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
        let feed_forward = overlay.torques_nm(two_cycle_phase, stance);
        let torques: Vec<UrdfJointTorqueTarget<'_>> = TORQUE_LINKS
            .iter()
            .enumerate()
            .map(|(index, link_name)| {
                let target_position = targets
                    .iter()
                    .find(|target| target.link_name == *link_name)
                    .expect("torque link in gait targets")
                    .position;
                let q = sim.named_joint_position(link_name).expect("joint position");
                let qd = sim.named_joint_velocity(link_name).expect("joint velocity");
                UrdfJointTorqueTarget {
                    link_name,
                    torque_nm: (KP * (target_position - q) - KD * qd + feed_forward[index])
                        .clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                    max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                }
            })
            .collect();
        sim.step_joint_torques(&torques);
        let observed = sim.observe();
        if !observed.base_y_m.is_finite() {
            // A solver blow-up is an automatic worst score.
            return RolloutOutcome {
                window_a_forward_m: 0.0,
                window_b_forward_m: 0.0,
                total_forward_m: 0.0,
                total_yaw_rad: 0.0,
                min_height_m: -1.0,
                max_tilt_rad: 10.0,
                score: -100.0,
            };
        }
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
    // Anti-cheat transport score: minimum straight-line window displacement,
    // with fall/crouch penalties tuned to the humanoid's 0.80 m stance and a
    // straightness penalty.
    let score = 2.0 * window_a_forward_m.min(window_b_forward_m)
        - if max_tilt_rad > 0.5 { 5.0 } else { 0.0 }
        - 20.0 * (0.60 - min_height_m).max(0.0)
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

fn overlay_from(params: &[f64; DIM]) -> UnitreeG1TorqueOverlay {
    let mut coefficients = [[0.0; 6]; 8];
    for joint in 0..8 {
        coefficients[joint][0] = params[joint * 6].clamp(-30.0, 30.0);
        for term in 1..6 {
            coefficients[joint][term] = params[joint * 6 + term].clamp(-20.0, 20.0);
        }
    }
    UnitreeG1TorqueOverlay { coefficients }
}

/// Verification ensemble: three ulp-perturbed replays — the chaos-floor
/// discipline from the Go2 campaign.
fn ensemble_outcomes(
    command: UnitreeG1GaitCommand,
    overlay: UnitreeG1TorqueOverlay,
) -> Vec<Option<RolloutOutcome>> {
    [0.0_f64, 1.0e-9, 3.0e-9]
        .iter()
        .map(|perturbation| {
            let mut member = overlay;
            member.coefficients[0][0] += *perturbation;
            // Wild candidates can blow the solver up inside a step (the
            // humanoid explosion mode); a panicking rollout deterministically
            // becomes a fall instead of killing the search.
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                rollout(command, &member, ROLLOUT_STEPS)
            }))
            .ok()
        })
        .collect()
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(|a, b| a.partial_cmp(b).expect("finite ensemble metric"));
    values[values.len() / 2]
}

fn ensemble_score_for(command: UnitreeG1GaitCommand, overlay: UnitreeG1TorqueOverlay) -> f64 {
    // Keep the original resumable CEM stream (0/1/2e-9) stable. The
    // verification and command-sweep ensemble above uses the pinned
    // cross-platform bar (0/1/3e-9).
    let mut scores: Vec<f64> = (0..3)
        .map(|k| {
            let mut member = overlay;
            member.coefficients[0][0] += k as f64 * 1.0e-9;
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                rollout(command, &member, ROLLOUT_STEPS).score
            }))
            .unwrap_or(-100.0)
        })
        .collect();
    scores.sort_by(|a, b| a.partial_cmp(b).expect("finite scores"));
    scores[1]
}

fn ensemble_score(params: &[f64; DIM]) -> f64 {
    ensemble_score_for(walk_command(), overlay_from(params))
}

/// Ensemble summary used by the deterministic gait-command speed sweep.
#[derive(Clone, Copy, Debug)]
struct SweepSummary {
    command: UnitreeG1GaitCommand,
    median_score: f64,
    median_min_window_m: f64,
    median_total_forward_m: f64,
    median_min_height_m: f64,
    median_abs_yaw_rad: f64,
    panics: usize,
}

fn summarize_sweep_candidate(
    command: UnitreeG1GaitCommand,
    overlay: UnitreeG1TorqueOverlay,
) -> SweepSummary {
    let outcomes = ensemble_outcomes(command, overlay);
    let panics = outcomes.iter().filter(|outcome| outcome.is_none()).count();
    let scores = outcomes
        .iter()
        .map(|outcome| outcome.map_or(-100.0, |value| value.score))
        .collect();
    let min_windows = outcomes
        .iter()
        .map(|outcome| {
            outcome.map_or(0.0, |value| {
                value.window_a_forward_m.min(value.window_b_forward_m)
            })
        })
        .collect();
    let total_forward = outcomes
        .iter()
        .map(|outcome| outcome.map_or(0.0, |value| value.total_forward_m))
        .collect();
    let min_heights = outcomes
        .iter()
        .map(|outcome| outcome.map_or(-1.0, |value| value.min_height_m))
        .collect();
    let abs_yaw = outcomes
        .iter()
        .map(|outcome| outcome.map_or(10.0, |value| value.total_yaw_rad.abs()))
        .collect();
    SweepSummary {
        command,
        median_score: median(scores),
        median_min_window_m: median(min_windows),
        median_total_forward_m: median(total_forward),
        median_min_height_m: median(min_heights),
        median_abs_yaw_rad: median(abs_yaw),
        panics,
    }
}

fn speed_sweep_commands() -> Vec<UnitreeG1GaitCommand> {
    let mut commands = Vec::new();
    for stride_rad in [0.060, 0.065, 0.070, 0.075, 0.080, 0.085] {
        for foot_lift_rad in [0.08, 0.10, 0.12] {
            for cycle_steps in [90, 100, 110] {
                commands.push(UnitreeG1GaitCommand {
                    stride_rad,
                    foot_lift_rad,
                    cycle_steps,
                });
            }
        }
    }
    commands
}

fn speed_sweep() {
    // Solver panics from candidates outside the contact basin are expected
    // and scored as falls, not printed as a wall of panic diagnostics.
    std::panic::set_hook(Box::new(|_| {}));
    let commands = speed_sweep_commands();
    let overlay = UnitreeG1TorqueOverlay::LEARNED_STRIDE;
    let mut summaries = Vec::with_capacity(commands.len());
    for chunk in commands.chunks(PARALLEL_ROLLOUTS) {
        let chunk_summaries = std::thread::scope(|scope| {
            let handles: Vec<_> = chunk
                .iter()
                .map(|command| {
                    let command = *command;
                    scope.spawn(move || summarize_sweep_candidate(command, overlay))
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| handle.join().expect("speed sweep rollout thread"))
                .collect::<Vec<_>>()
        });
        summaries.extend(chunk_summaries);
    }
    summaries.sort_by(|a, b| {
        b.median_score
            .partial_cmp(&a.median_score)
            .expect("finite sweep scores")
            .then_with(|| {
                b.median_min_window_m
                    .partial_cmp(&a.median_min_window_m)
                    .expect("finite sweep windows")
            })
    });
    println!(
        "G1 speed sweep: {} commands, fixed overlay, ranking by ensemble median score",
        summaries.len()
    );
    for summary in summaries.iter().take(12) {
        println!(
            "stride={:.3} lift={:.2} cycle={:3} score={:+.3} windows={:.3} m total={:.3} m speed={:.4} m/s minH={:.3} |yaw|={:.3} panics={}",
            summary.command.stride_rad,
            summary.command.foot_lift_rad,
            summary.command.cycle_steps,
            summary.median_score,
            summary.median_min_window_m,
            summary.median_total_forward_m,
            summary.median_total_forward_m / 24.0,
            summary.median_min_height_m,
            summary.median_abs_yaw_rad,
            summary.panics,
        );
    }
    let fastest_valid = summaries
        .iter()
        .filter(|summary| {
            summary.panics <= 1
                && summary.median_min_height_m > 0.7
                && summary.median_abs_yaw_rad < 0.3
        })
        .max_by(|a, b| {
            a.median_total_forward_m
                .partial_cmp(&b.median_total_forward_m)
                .expect("finite sweep transport")
        })
        .expect("speed sweep must find a valid candidate");
    println!(
        "fastest valid: stride={:.3} lift={:.2} cycle={} median windows={:.3} m total={:.3} m ({:.4} m/s)",
        fastest_valid.command.stride_rad,
        fastest_valid.command.foot_lift_rad,
        fastest_valid.command.cycle_steps,
        fastest_valid.median_min_window_m,
        fastest_valid.median_total_forward_m,
        fastest_valid.median_total_forward_m / 24.0,
    );
}

fn gaussian(rng: &mut rne_ai::DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

const ITERATIONS_PER_RUN: usize = 2;
const PARALLEL_ROLLOUTS: usize = 16;

type TrainState = (usize, [f64; DIM], [f64; DIM], (f64, [f64; DIM]));

fn state_path() -> PathBuf {
    // The speed-up command is a different plant trajectory from the first
    // stride search, so its resumable CEM state must not be mixed with it.
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/g1_stride_speedup_cem_state.txt")
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
    // Solver panics from wild candidates are expected and scored, not fatal;
    // silence the default hook so the log stays readable.
    std::panic::set_hook(Box::new(|_| {}));
    let path = state_path();
    let (start_iteration, mut mean, mut sigma, mut best) =
        load_state(&path).unwrap_or((0, [0.0; DIM], [5.0; DIM], (f64::MIN, [0.0; DIM])));
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
            sigma[dimension] = elite_variance.sqrt().max(0.5);
        }
        let probe = rollout(walk_command(), &overlay_from(&scored[0].1), ROLLOUT_STEPS);
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
        let outcome = rollout(walk_command(), &overlay, ROLLOUT_STEPS);
        println!(
            "ensemble member {k}: windows {:.2}/{:.2} m total {:.2} m minH {:.3} tilt {:.2}",
            outcome.window_a_forward_m,
            outcome.window_b_forward_m,
            outcome.total_forward_m,
            outcome.min_height_m,
            outcome.max_tilt_rad
        );
    }
    let final_outcome = rollout(walk_command(), &overlay_from(&best.1), ROLLOUT_STEPS);
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
    if std::env::args().any(|argument| argument == "--sweep") {
        speed_sweep();
        return;
    }
    if std::env::args().any(|argument| argument == "--train") {
        train();
        return;
    }

    // Default: the comparison verify at its cross-platform bar. A degraded
    // humanoid orbit can blow the solver up mid-step (the ulp-shifted orbit
    // on another OS did exactly that in CI), so each replay runs under
    // catch_unwind - a panic is a fall - and the claim is the MEDIAN of
    // three ulp-perturbed replays.
    let baseline = rollout(walk_command(), &UnitreeG1TorqueOverlay::ZERO, ROLLOUT_STEPS);
    let members = ensemble_outcomes(walk_command(), UnitreeG1TorqueOverlay::LEARNED_STRIDE);
    println!(
        "stepper baseline: windows {:.2}/{:.2} m total {:.2} m",
        baseline.window_a_forward_m, baseline.window_b_forward_m, baseline.total_forward_m,
    );
    for (index, member) in members.iter().enumerate() {
        match member {
            Some(outcome) => println!(
                "member {index}: windows {:.2}/{:.2} m total {:.2} m minH {:.3} yaw {:+.2}",
                outcome.window_a_forward_m,
                outcome.window_b_forward_m,
                outcome.total_forward_m,
                outcome.min_height_m,
                outcome.total_yaw_rad
            ),
            None => println!("member {index}: SOLVER PANIC (scored as fall)"),
        }
    }
    let mut min_windows: Vec<f64> = members
        .iter()
        .map(|member| {
            member
                .as_ref()
                .map_or(0.0, |o| o.window_a_forward_m.min(o.window_b_forward_m))
        })
        .collect();
    min_windows.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let median = min_windows[1];
    let baseline_min = baseline.window_a_forward_m.min(baseline.window_b_forward_m);
    assert!(
        median > 2.0 * baseline_min && median > SPEEDUP_MIN_WINDOW_M,
        "the median replay must hit the speed-up envelope: {median:.2} m vs stepper {baseline_min:.2} m"
    );
}
