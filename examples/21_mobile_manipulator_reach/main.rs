//! Open-loop shoulder motion toward +X using `MobileManipulatorSim`.

use rne_ai::{MobileManipulatorAction, MobileManipulatorSim};

const MIN_EE_DISPLACEMENT_M: f64 = 0.025;
const MIN_SHOULDER_DELTA_RAD: f64 = 0.15;

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut sim = MobileManipulatorSim::new_mm_minimal();
    let initial = sim.observe();

    for _ in 0..360 {
        sim.step(MobileManipulatorAction {
            left_wheel_velocity_rad_s: 0.0,
            right_wheel_velocity_rad_s: 0.0,
            shoulder_velocity_rad_s: 3.0,
            elbow_velocity_rad_s: 0.0,
        });
    }

    let final_obs = sim.observe();
    let displacement = ((final_obs.ee_x_m - initial.ee_x_m).powi(2)
        + (final_obs.ee_y_m - initial.ee_y_m).powi(2)
        + (final_obs.ee_z_m - initial.ee_z_m).powi(2))
    .sqrt();
    let shoulder_delta = (final_obs.shoulder_position_rad - initial.shoulder_position_rad).abs();

    if smoke {
        if displacement < MIN_EE_DISPLACEMENT_M && shoulder_delta < MIN_SHOULDER_DELTA_RAD {
            eprintln!(
                "smoke failed: ee displacement={displacement:.4} m shoulder_delta={shoulder_delta:.4} rad"
            );
            std::process::exit(1);
        }
        println!(
            "smoke ok: ee displacement={displacement:.4} m (joint_state={})",
            final_obs.joint_state_count
        );
        return;
    }

    println!(
        "initial ee = ({:.3}, {:.3}, {:.3})",
        initial.ee_x_m, initial.ee_y_m, initial.ee_z_m
    );
    println!(
        "final ee   = ({:.3}, {:.3}, {:.3})",
        final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m
    );
    println!(
        "shoulder = {:.3} rad, elbow = {:.3} rad",
        final_obs.shoulder_position_rad, final_obs.elbow_position_rad
    );
}
