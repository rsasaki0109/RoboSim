//! Headless forward-walk regression for the official Unitree G1.
//!
//! Drives the validated commanded gait with a pure forward velocity and asserts
//! the measured boundary: no fall, an upright body, bounded torque, and ground
//! covered. The same plant is captured as a hero GIF by example 92; this example
//! is the fast CLI and CI gate. Pass `--smoke` for the short rollout.
//!
//! Run with `cargo run -p g1_walk --example 111_g1_walk`.

use rne_ai::{
    run_unitree_g1_commanded_gait, UnitreeG1CommandedGaitConfig, UnitreeG1VelocityCommand,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let (settle_steps, rollout_steps) = if smoke { (60, 480) } else { (240, 1440) };

    let config = UnitreeG1CommandedGaitConfig {
        settle_steps,
        rollout_steps,
        command: UnitreeG1VelocityCommand {
            forward_m_s: 0.0276,
            yaw_rate_rad_s: 0.0,
        },
        ..UnitreeG1CommandedGaitConfig::default()
    };

    let first = run_unitree_g1_commanded_gait(config.clone()).expect("G1 forward walk");
    let second = run_unitree_g1_commanded_gait(config).expect("G1 forward walk replay");
    assert_eq!(
        first, second,
        "G1 forward walk must replay deterministically"
    );

    let min_displacement_m = if smoke { 0.05 } else { 0.2 };
    assert!(!first.fell, "G1 forward walk fell: {first:?}");
    assert!(
        first.min_height_m > 0.75,
        "G1 forward walk dropped too low: {first:?}"
    );
    assert!(
        first.max_tilt_rad < 0.30,
        "G1 forward walk tilted too far: {first:?}"
    );
    assert!(
        first.max_command_nm <= 88.0,
        "G1 forward walk exceeded the torque limit: {first:?}"
    );
    assert!(
        first.total_displacement_m > min_displacement_m,
        "G1 forward walk must cover ground: {first:?}"
    );

    println!(
        "g1 walk passed: steps={rollout_steps} displacement={:.3} m minH={:.3} m tilt={:.3} rad",
        first.total_displacement_m, first.min_height_m, first.max_tilt_rad,
    );
}
