//! Evaluates and searches a command-conditioned G1 heading-yaw candidate.
//!
//! The v0.2 harness preserves the v0.1 learned stride and its hybrid actuator
//! boundary: eight proximal joints use torque-PD while ankles, arms, and waist
//! remain position-servoed. The yaw channel is contact-aware through the
//! proximal overlay's measured stance gates. A deterministic CEM searches the
//! command-scaled yaw overlay against both commanded directions and scores a
//! three-member one-ULP replay ensemble.

use rne_ai::{
    run_unitree_g1_commanded_gait_with_policy, UnitreeG1CommandedGaitConfig,
    UnitreeG1CommandedGaitOutcome, UnitreeG1CommandedTorquePolicy, UnitreeG1TorqueOverlay,
    UnitreeG1VelocityCommand,
};

const FORWARD_M_S: f64 = 0.0276;
const TURN_RATE_RAD_S: f64 = 0.05;
const EVAL_STEPS: u64 = 240;
const CEM_DIM: usize = 48;

#[derive(Clone, Copy, Debug)]
struct Candidate {
    yaw_overlay: UnitreeG1TorqueOverlay,
}

impl Candidate {
    fn validated_heading() -> Self {
        Self {
            yaw_overlay: UnitreeG1TorqueOverlay::ZERO,
        }
    }

    fn policy(self) -> UnitreeG1CommandedTorquePolicy {
        UnitreeG1CommandedTorquePolicy {
            yaw_rate_kp_nm_per_rad_s: 32.0,
            max_yaw_torque_nm: 16.0,
            negative_yaw_rate_gain_scale: 0.5,
            yaw_overlay: self.yaw_overlay,
            yaw_overlay_gain: 8.0,
            mirror_yaw_overlay_negative: false,
            ..UnitreeG1CommandedTorquePolicy::default()
        }
    }
}

fn config(command: UnitreeG1VelocityCommand, steps: u64) -> UnitreeG1CommandedGaitConfig {
    UnitreeG1CommandedGaitConfig {
        command,
        settle_steps: 60,
        rollout_steps: steps,
        // The nominal gait keeps its v0.1 learned-stride mirror. The v0.2
        // command targets use the explicit signed hip-yaw channel instead.
        mirror_negative_yaw: false,
        yaw_hip_yaw_right_sign: -1.0,
        yaw_hip_yaw_target_rad_per_rad_s: 0.0,
        // A bounded target makes this first heading contract a stable turn
        // reference instead of letting a small-rate plant error integrate
        // into an unbounded pose request.
        heading_target_clamp_rad: 0.08,
        ..UnitreeG1CommandedGaitConfig::default()
    }
}

fn run(
    command: UnitreeG1VelocityCommand,
    steps: u64,
    candidate: Candidate,
) -> UnitreeG1CommandedGaitOutcome {
    run_unitree_g1_commanded_gait_with_policy(config(command, steps), candidate.policy())
        .expect("load and evaluate dynamic G1")
}

fn print_outcome(label: &str, outcome: UnitreeG1CommandedGaitOutcome) {
    let radius = outcome
        .turn_radius_m
        .map_or_else(|| "none".to_owned(), |value| format!("{value:.3} m"));
    println!(
        "{label:>7}: cmd=({:+.4} m/s,{:+.3} rad/s) target={:+.3} yaw={:+.3} err={:+.3} rateErr={:.3} radius={radius:>8} x={:+.3} z={:+.3} minH={:.3} tilt={:.3} tau={:.2} fell={} digest=0x{:016x}",
        outcome.command.forward_m_s,
        outcome.command.yaw_rate_rad_s,
        outcome.target_heading_rad,
        outcome.total_yaw_rad,
        outcome.heading_error_rad,
        outcome.mean_abs_yaw_rate_error_rad_s,
        outcome.base_x_displacement_m,
        outcome.base_z_displacement_m,
        outcome.min_height_m,
        outcome.max_tilt_rad,
        outcome.max_command_nm,
        outcome.fell,
        outcome.replay_digest,
    );
}

fn finite(outcome: UnitreeG1CommandedGaitOutcome) -> bool {
    let scalar_metrics = [
        outcome.base_x_displacement_m,
        outcome.base_z_displacement_m,
        outcome.total_displacement_m,
        outcome.mean_forward_velocity_m_s,
        outcome.mean_yaw_rate_rad_s,
        outcome.total_yaw_rad,
        outcome.target_heading_rad,
        outcome.heading_error_rad,
        outcome.mean_abs_heading_error_rad,
        outcome.mean_abs_yaw_rate_error_rad_s,
        outcome.min_height_m,
        outcome.max_tilt_rad,
        outcome.max_command_nm,
    ];
    scalar_metrics.iter().all(|value| value.is_finite())
        && outcome.turn_radius_m.is_none_or(f64::is_finite)
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

fn overlay_from(params: &[f64; CEM_DIM]) -> UnitreeG1TorqueOverlay {
    let mut coefficients = [[0.0; 6]; 8];
    for joint in 0..8 {
        for term in 0..6 {
            coefficients[joint][term] = params[joint * 6 + term].clamp(-8.0, 8.0);
        }
    }
    UnitreeG1TorqueOverlay { coefficients }
}

fn candidate_from(params: &[f64; CEM_DIM]) -> Candidate {
    Candidate {
        yaw_overlay: overlay_from(params),
    }
}

fn direction_score(
    left: UnitreeG1CommandedGaitOutcome,
    right: UnitreeG1CommandedGaitOutcome,
) -> f64 {
    if left.fell
        || right.fell
        || left.min_height_m <= 0.75
        || right.min_height_m <= 0.75
        || left.total_yaw_rad <= 0.01
        || right.total_yaw_rad >= -0.001
        || !finite(left)
        || !finite(right)
    {
        return -10.0;
    }
    let signed_yaw = left.total_yaw_rad.min(-right.total_yaw_rad);
    let rate_error = left.mean_abs_yaw_rate_error_rad_s + right.mean_abs_yaw_rate_error_rad_s;
    let heading_error = left.heading_error_rad.abs() + right.heading_error_rad.abs();
    5.0 * signed_yaw - rate_error - heading_error - left.max_tilt_rad.max(right.max_tilt_rad)
}

fn score(params: &[f64; CEM_DIM], steps: u64) -> f64 {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut members = Vec::with_capacity(3);
        for direction in [-1_i8, 0, 1] {
            let mut member = *params;
            member[0] = ulp_offset(member[0], direction);
            let outcomes = evaluate_candidate(candidate_from(&member), steps);
            members.push(direction_score(outcomes[2], outcomes[3]));
        }
        members.sort_by(|a, b| a.partial_cmp(b).expect("finite CEM score"));
        members[1]
    }));
    result.unwrap_or(-10.0)
}

fn ulp_offset(value: f64, direction: i8) -> f64 {
    if direction == 0 || !value.is_finite() {
        return value;
    }
    if value == 0.0 {
        return f64::from_bits(if direction > 0 { 1 } else { (1_u64 << 63) | 1 });
    }
    let bits = value.to_bits();
    let next_bits = if direction > 0 {
        if value.is_sign_positive() {
            bits + 1
        } else {
            bits - 1
        }
    } else if value.is_sign_positive() {
        bits - 1
    } else {
        bits + 1
    };
    f64::from_bits(next_bits)
}

fn bounded_sample(rng: &mut u64, mean: f64, sigma: f64) -> f64 {
    *rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
    let unit = ((*rng >> 11) as f64) / ((1_u64 << 53) as f64);
    let normal = (unit * 2.0 - 1.0) * 1.7320508075688772;
    (mean + sigma * normal).clamp(-8.0, 8.0)
}

fn train_cem(smoke: bool) {
    std::panic::set_hook(Box::new(|_| {}));
    let steps = if smoke { 120 } else { EVAL_STEPS };
    let population = if smoke { 4 } else { 8 };
    let iterations = if smoke { 1 } else { 3 };
    let mut mean = [0.0; CEM_DIM];
    let mut sigma = [1.0; CEM_DIM];
    let mut rng = 0x6802_u64;
    let mut best = (score(&mean, steps), mean);

    for iteration in 0..iterations {
        let mut scored = Vec::with_capacity(population);
        for _ in 0..population {
            let mut params = [0.0; CEM_DIM];
            for (value, (average, spread)) in params.iter_mut().zip(mean.iter().zip(sigma.iter())) {
                *value = bounded_sample(&mut rng, *average, *spread);
            }
            scored.push((score(&params, steps), params));
        }
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).expect("finite CEM score"));
        if scored[0].0 > best.0 {
            best = scored[0];
        }
        let elite_count = (population / 2).max(1);
        for dimension in 0..CEM_DIM {
            mean[dimension] = scored[..elite_count]
                .iter()
                .map(|(_, params)| params[dimension])
                .sum::<f64>()
                / elite_count as f64;
            sigma[dimension] = (scored[..elite_count]
                .iter()
                .map(|(_, params)| (params[dimension] - mean[dimension]).powi(2))
                .sum::<f64>()
                / elite_count as f64)
                .sqrt()
                .max(0.25);
        }
        println!(
            "heading-yaw CEM iter={iteration} best={:+.4} mean0={:+.3} mean1={:+.3} mean2={:+.3}",
            scored[0].0, mean[0], mean[1], mean[2]
        );
    }

    let baseline_score = score(&[0.0; CEM_DIM], steps);
    let chosen = if best.0 >= baseline_score {
        candidate_from(&best.1)
    } else {
        Candidate::validated_heading()
    };
    let outcomes = evaluate_candidate(chosen, steps);
    print_outcome("left", outcomes[2]);
    print_outcome("right", outcomes[3]);
    println!(
        "heading-yaw CEM score={:+.4} baseline={:+.4} coefficients={:?}",
        best.0, baseline_score, chosen.yaw_overlay.coefficients
    );
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    if std::env::args().any(|argument| argument == "--train") {
        train_cem(smoke);
        return;
    }

    let steps = if smoke { 120 } else { EVAL_STEPS };
    let outcomes = evaluate_candidate(Candidate::validated_heading(), steps);
    for (label, outcome) in [
        ("forward", outcomes[0]),
        ("stop", outcomes[1]),
        ("left", outcomes[2]),
        ("right", outcomes[3]),
    ] {
        print_outcome(label, outcome);
        assert!(finite(outcome));
    }

    if !smoke {
        for outcome in outcomes {
            assert!(!outcome.fell, "v0.2 G1 command fell");
            assert!(
                outcome.min_height_m > 0.75,
                "G1 dropped below the heading envelope"
            );
        }
        assert!(outcomes[0].total_displacement_m > 0.02);
        assert!(
            outcomes[2].total_yaw_rad > 0.01,
            "left body yaw lost its sign"
        );
        assert!(
            outcomes[3].total_yaw_rad < -0.001,
            "right body yaw lost its sign"
        );
        assert!(outcomes[2].target_heading_rad > 0.0);
        assert!(outcomes[3].target_heading_rad < 0.0);
        assert!(outcomes[2].turn_radius_m.is_some());
        assert!(outcomes[3].turn_radius_m.is_some());

        let replay = run(
            UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: TURN_RATE_RAD_S,
            },
            EVAL_STEPS,
            Candidate::validated_heading(),
        );
        assert_eq!(
            outcomes[2], replay,
            "heading replay must be bit deterministic"
        );
    }
}
