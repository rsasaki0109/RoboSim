//! LIPM + ZMP-preview walking for the official 23-DoF Unitree G1.
//!
//! Plans footsteps and a center-of-mass trajectory with `rne_legged` (ZMP
//! preview control), places the swing foot on each foothold, and converts the
//! plan into joint targets with a planar two-link leg inverse kinematics — the
//! model-based biped pipeline the open-source walking controllers share.
//!
//! Run with `cargo run --release -p g1_lipm_walk --example 110_g1_lipm_walk`.

use rne_ai::{run_unitree_g1_lipm_walk, UnitreeG1LipmWalkConfig};

fn main() {
    let steps = std::env::args()
        .position(|argument| argument == "--steps")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(8);
    let step_length_m = std::env::args()
        .position(|argument| argument == "--step-length")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.18);
    let trace = std::env::args().any(|argument| argument == "--trace");
    let limit = std::env::args()
        .position(|argument| argument == "--limit")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(88.0);
    let parse = |flag: &str, default: f64| {
        std::env::args()
            .position(|argument| argument == flag)
            .and_then(|index| std::env::args().nth(index + 1))
            .and_then(|value| value.parse::<f64>().ok())
            .unwrap_or(default)
    };
    let single_support = parse("--single-support", 0.5);
    let width = parse("--width", 0.04);
    let com_height = parse("--com-height", 0.60);
    let foot_gain = std::env::args()
        .position(|argument| argument == "--foot-gain")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(1.0);
    let stiffness = std::env::args()
        .position(|argument| argument == "--stiffness")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(220.0);
    let gain = std::env::args()
        .position(|argument| argument == "--gain")
        .and_then(|index| std::env::args().nth(index + 1))
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.3);
    let config = UnitreeG1LipmWalkConfig {
        trace,
        com_feedback_gain: gain,
        dcm_foot_placement_gain: foot_gain,
        position_stiffness: stiffness,
        position_damping: (stiffness * 0.11).max(24.0),
        torque_limit_nm: limit,
        steps,
        single_support_s: single_support,
        step_width_m: width,
        com_height_m: com_height,
        step_length_m,
        rollout_steps: (0.4 / (1.0 / 60.0)) as usize + steps * 36 + (0.4 / (1.0 / 60.0)) as usize,
        ..UnitreeG1LipmWalkConfig::default()
    };
    let outcome = run_unitree_g1_lipm_walk(config).expect("LIPM walk");
    println!(
        "pattern={:.2}s planned_com=({:.3},{:.3}) measured=({:.3},{:.3}) forward={:.3} minH={:.3} tilt={:.3} track_err={:.3} fell={} digest=0x{:016x}",
        outcome.pattern_duration_s,
        outcome.planned_com_m.x_m,
        outcome.planned_com_m.z_m,
        outcome.measured_pelvis_m.x_m,
        outcome.measured_pelvis_m.z_m,
        outcome.forward_distance_m,
        outcome.min_height_m,
        outcome.max_tilt_rad,
        outcome.max_tracking_error_m,
        outcome.fell,
        outcome.digest,
    );
}
