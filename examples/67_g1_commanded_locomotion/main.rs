//! Evaluates the v0.1 command-conditioned official G1 locomotion policy.
//!
//! The same headless harness runs forward, stop, and differential left/right
//! steering commands. `--train` runs a small deterministic CEM over a
//! contact-gated differential torque overlay; `--smoke` uses short rollouts
//! and performs no renderer or network work.

use rne_ai::{
    run_unitree_g1_commanded_gait_with_policy, UnitreeG1CommandedGaitConfig,
    UnitreeG1CommandedGaitOutcome, UnitreeG1CommandedTorquePolicy, UnitreeG1TorqueOverlay,
    UnitreeG1VelocityCommand,
};

const FORWARD_M_S: f64 = 0.0276;
const TURN_RATE_RAD_S: f64 = 0.05;

#[derive(Clone, Copy, Debug)]
struct Candidate {
    yaw_overlay_scale: f64,
    yaw_overlay_joint_gains: [f64; 8],
    yaw_overlay_coefficients: [[f64; 6]; 8],
}

impl Candidate {
    fn validated_steering() -> Self {
        Self {
            yaw_overlay_scale: 1.0,
            yaw_overlay_coefficients: UnitreeG1TorqueOverlay::LEARNED_DIFFERENTIAL_STEERING
                .coefficients,
            ..Self::default()
        }
    }

    fn policy(self) -> UnitreeG1CommandedTorquePolicy {
        let yaw_overlay = if self.yaw_overlay_coefficients == [[0.0; 6]; 8] {
            UnitreeG1TorqueOverlay::LEARNED_STRIDE
        } else {
            UnitreeG1TorqueOverlay {
                coefficients: self.yaw_overlay_coefficients,
            }
        };
        UnitreeG1CommandedTorquePolicy {
            forward_velocity_feedback_gain: 0.0,
            yaw_overlay,
            yaw_overlay_gain: self.yaw_overlay_scale,
            yaw_overlay_joint_gains: self.yaw_overlay_joint_gains,
            ..UnitreeG1CommandedTorquePolicy::default()
        }
    }
}

impl Default for Candidate {
    fn default() -> Self {
        Self {
            yaw_overlay_scale: 0.0,
            yaw_overlay_joint_gains: [1.0; 8],
            yaw_overlay_coefficients: [[0.0; 6]; 8],
        }
    }
}

fn config(command: UnitreeG1VelocityCommand, steps: u64) -> UnitreeG1CommandedGaitConfig {
    UnitreeG1CommandedGaitConfig {
        command,
        rollout_steps: steps,
        ..UnitreeG1CommandedGaitConfig::default()
    }
}

fn run(
    command: UnitreeG1VelocityCommand,
    steps: u64,
    candidate: Candidate,
) -> UnitreeG1CommandedGaitOutcome {
    let config = config(command, steps);
    run_unitree_g1_commanded_gait_with_policy(config, candidate.policy())
        .expect("load and evaluate dynamic G1")
}

fn print_outcome(label: &str, outcome: UnitreeG1CommandedGaitOutcome) {
    println!(
        "{label:>7}: cmd=({:+.4} m/s,{:+.3} rad/s) x={:+.3} z={:+.3} path={:.3} yaw={:+.3} mean=({:+.4},{:+.4}) minH={:.3} tilt={:.3} tau={:.2} fell={} digest=0x{:016x}",
        outcome.command.forward_m_s,
        outcome.command.yaw_rate_rad_s,
        outcome.base_x_displacement_m,
        outcome.base_z_displacement_m,
        outcome.total_displacement_m,
        outcome.total_yaw_rad,
        outcome.mean_forward_velocity_m_s,
        outcome.mean_yaw_rate_rad_s,
        outcome.min_height_m,
        outcome.max_tilt_rad,
        outcome.max_command_nm,
        outcome.fell,
        outcome.replay_digest,
    );
}

fn evaluate_candidate(candidate: Candidate, steps: u64) -> [UnitreeG1CommandedGaitOutcome; 4] {
    [
        run(
            UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: 0.0,
            },
            steps,
            candidate,
        ),
        run(UnitreeG1VelocityCommand::default(), steps, candidate),
        run(
            UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: TURN_RATE_RAD_S,
            },
            steps,
            candidate,
        ),
        run(
            UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: -TURN_RATE_RAD_S,
            },
            steps,
            candidate,
        ),
    ]
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    if std::env::args().any(|argument| argument == "--train" || argument == "--train-overlay") {
        train_turn_overlay(smoke);
        return;
    }

    let steps = if smoke { 120 } else { 1440 };
    let outcomes = evaluate_candidate(Candidate::validated_steering(), steps);
    for (label, outcome) in [
        ("forward", outcomes[0]),
        ("stop", outcomes[1]),
        ("left", outcomes[2]),
        ("right", outcomes[3]),
    ] {
        print_outcome(label, outcome);
        assert!(outcome.max_height_maybe_finite());
    }

    if !smoke {
        assert!(!outcomes[0].fell, "forward command fell");
        assert!(!outcomes[1].fell, "stop command fell");
        assert!(!outcomes[2].fell, "left command fell");
        assert!(!outcomes[3].fell, "right command fell");
        assert!(outcomes[0].total_displacement_m > 0.20);
        assert!(outcomes[1].total_displacement_m < 0.22);
        assert!(outcomes[2].base_z_displacement_m > 0.15);
        assert!(outcomes[3].base_z_displacement_m < -0.15);
        assert!(outcomes[2].total_yaw_rad.abs() < 0.10);
        assert!(outcomes[3].total_yaw_rad.abs() < 0.10);
        assert!(outcomes[0].max_tilt_rad < 0.50);
        assert!(outcomes[0].total_yaw_rad.abs() < 0.10);

        let mut disturbance_config = config(
            UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: 0.0,
            },
            720,
        );
        disturbance_config.disturbance_step = Some(360);
        disturbance_config.disturbance_axis_angle_rad = [0.0, 0.0, 0.02];
        let disturbance = run_unitree_g1_commanded_gait_with_policy(
            disturbance_config,
            Candidate::validated_steering().policy(),
        )
        .expect("evaluate disturbed G1");
        print_outcome("push", disturbance);
        assert!(disturbance.disturbance_applied);
        assert!(!disturbance.fell);
        assert!(disturbance.min_height_m > 0.70);
        assert!(disturbance.max_tilt_rad < 0.50);
    }
}

trait FiniteOutcome {
    fn max_height_maybe_finite(self) -> bool;
}

impl FiniteOutcome for UnitreeG1CommandedGaitOutcome {
    fn max_height_maybe_finite(self) -> bool {
        [
            self.base_x_displacement_m,
            self.base_z_displacement_m,
            self.total_displacement_m,
            self.mean_forward_velocity_m_s,
            self.mean_yaw_rate_rad_s,
            self.total_yaw_rad,
            self.min_height_m,
            self.max_tilt_rad,
            self.max_command_nm,
        ]
        .iter()
        .all(|value| value.is_finite())
    }
}

const TURN_OVERLAY_DIM: usize = 24;

fn turn_overlay_from(params: &[f64; TURN_OVERLAY_DIM]) -> UnitreeG1TorqueOverlay {
    let mut coefficients = [[0.0; 6]; 8];
    for joint in 0..8 {
        coefficients[joint][0] = params[joint * 3].clamp(-16.0, 16.0);
        coefficients[joint][1] = params[joint * 3 + 1].clamp(-12.0, 12.0);
        coefficients[joint][2] = params[joint * 3 + 2].clamp(-12.0, 12.0);
    }
    UnitreeG1TorqueOverlay { coefficients }
}

fn turn_overlay_candidate(params: &[f64; TURN_OVERLAY_DIM]) -> Candidate {
    Candidate {
        yaw_overlay_scale: 1.0,
        yaw_overlay_coefficients: turn_overlay_from(params).coefficients,
        ..Candidate::default()
    }
}

fn turn_overlay_score(params: &[f64; TURN_OVERLAY_DIM], steps: u64) -> f64 {
    let score = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut scores = Vec::new();
        for perturbation in [0.0_f64, 1.0e-9, -1.0e-9] {
            let mut member = *params;
            member[0] += perturbation;
            let candidate = turn_overlay_candidate(&member);
            let left = run(
                UnitreeG1VelocityCommand {
                    forward_m_s: FORWARD_M_S,
                    yaw_rate_rad_s: TURN_RATE_RAD_S,
                },
                steps,
                candidate,
            );
            let right = run(
                UnitreeG1VelocityCommand {
                    forward_m_s: FORWARD_M_S,
                    yaw_rate_rad_s: -TURN_RATE_RAD_S,
                },
                steps,
                candidate,
            );
            if left.fell || right.fell {
                scores.push(-10.0);
            } else {
                let signed_turn = left.base_z_displacement_m.min(-right.base_z_displacement_m);
                let forward = (left.total_displacement_m + right.total_displacement_m) * 0.25;
                scores
                    .push(4.0 * signed_turn + forward - left.max_tilt_rad.max(right.max_tilt_rad));
            }
        }
        scores.sort_by(|a, b| a.partial_cmp(b).expect("finite overlay scores"));
        scores[1]
    }));
    score.unwrap_or(-10.0)
}

fn train_turn_overlay(smoke: bool) {
    std::panic::set_hook(Box::new(|_| {}));
    let steps = if smoke { 120 } else { 240 };
    let population = if smoke { 4 } else { 6 };
    let iterations = if smoke { 1 } else { 2 };
    let mut mean = [0.0; TURN_OVERLAY_DIM];
    let mut sigma = [4.0; TURN_OVERLAY_DIM];
    let mut rng = 0x6701_u64;
    for iteration in 0..iterations {
        let mut scored = Vec::with_capacity(population);
        for _ in 0..population {
            let mut params = [0.0; TURN_OVERLAY_DIM];
            for (value, (average, spread)) in params.iter_mut().zip(mean.iter().zip(sigma.iter())) {
                *value = bounded_sample(&mut rng, *average, *spread, -16.0, 16.0);
            }
            scored.push((turn_overlay_score(&params, steps), params));
        }
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).expect("finite overlay score"));
        let elite = &scored[..(population / 2).max(1)];
        for dimension in 0..TURN_OVERLAY_DIM {
            mean[dimension] = elite
                .iter()
                .map(|(_, params)| params[dimension])
                .sum::<f64>()
                / elite.len() as f64;
            sigma[dimension] = (elite
                .iter()
                .map(|(_, params)| (params[dimension] - mean[dimension]).powi(2))
                .sum::<f64>()
                / elite.len() as f64)
                .sqrt()
                .max(0.25);
        }
        println!(
            "turn-overlay CEM iter={iteration} best={:+.4} elite0={:+.3} elite1={:+.3} elite2={:+.3}",
            scored[0].0, mean[0], mean[1], mean[2]
        );
    }
    let candidate = turn_overlay_candidate(&mean);
    let left = run(
        UnitreeG1VelocityCommand {
            forward_m_s: FORWARD_M_S,
            yaw_rate_rad_s: TURN_RATE_RAD_S,
        },
        if smoke { 120 } else { 720 },
        candidate,
    );
    let right = run(
        UnitreeG1VelocityCommand {
            forward_m_s: FORWARD_M_S,
            yaw_rate_rad_s: -TURN_RATE_RAD_S,
        },
        if smoke { 120 } else { 720 },
        candidate,
    );
    println!(
        "turn-overlay candidate lateral=({:+.4},{:+.4}) yaw=({:+.4},{:+.4}) fell=({}, {})",
        left.base_z_displacement_m,
        right.base_z_displacement_m,
        left.total_yaw_rad,
        right.total_yaw_rad,
        left.fell,
        right.fell
    );
    if !smoke {
        let signed_turn = left.base_z_displacement_m.min(-right.base_z_displacement_m);
        assert!(!left.fell && !right.fell, "CEM turn candidate fell");
        assert!(signed_turn > 0.02, "CEM found no signed steering path");
    }
    if !smoke {
        println!(
            "turn-overlay coefficients: {:?}",
            turn_overlay_from(&mean).coefficients
        );
    }
}

fn bounded_sample(rng: &mut u64, mean: f64, sigma: f64, minimum: f64, maximum: f64) -> f64 {
    *rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
    let unit = ((*rng >> 11) as f64) / ((1_u64 << 53) as f64);
    let normal = (unit * 2.0 - 1.0) * 1.7320508075688772;
    (mean + sigma * normal).clamp(minimum, maximum)
}
