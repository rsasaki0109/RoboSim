//! Open-loop shoulder reach toward a calibrated world-frame target.

use rne_ai::{
    ee_distance_to_target_m, mm_minimal_scene_path, MobileManipulatorAction, MobileManipulatorSim,
    ReachTarget,
};

/// Calibrated EE pose after open-loop shoulder motion on `mm_minimal` (with gripper).
const POSE_TARGET: ReachTarget = ReachTarget {
    x_m: 0.456,
    y_m: 0.562,
    z_m: 0.204,
};
const REACH_SUCCESS_M: f64 = 0.05;
const SHOULDER_VELOCITY_RAD_S: f64 = 3.0;
const ALT_SHOULDER_VELOCITIES_RAD_S: [f64; 3] = [3.0, -3.0, 6.0];
const REACH_STEPS: usize = 720;
const MAX_SMOKE_ATTEMPTS: usize = 4;

fn run_reach(sim: &mut MobileManipulatorSim, shoulder_velocity_rad_s: f64) -> (f64, f64) {
    let initial = sim.observe();
    let initial_error = ee_distance_to_target_m(&initial, POSE_TARGET);

    for _ in 0..REACH_STEPS {
        sim.step(MobileManipulatorAction {
            left_wheel_velocity_rad_s: 0.0,
            right_wheel_velocity_rad_s: 0.0,
            shoulder_velocity_rad_s,
            elbow_velocity_rad_s: 0.0,
            gripper_velocity_rad_s: 0.0,
        });
    }

    let final_error = ee_distance_to_target_m(&sim.observe(), POSE_TARGET);
    (initial_error, final_error)
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    if smoke {
        for attempt in 1..=MAX_SMOKE_ATTEMPTS {
            let mut sim = MobileManipulatorSim::from_scene_path(&mm_minimal_scene_path())
                .expect("load mm_minimal scene");
            for shoulder_velocity_rad_s in ALT_SHOULDER_VELOCITIES_RAD_S {
                let (initial_error, final_error) = run_reach(&mut sim, shoulder_velocity_rad_s);
                if final_error < REACH_SUCCESS_M {
                    println!(
                        "smoke ok: ee error={final_error:.4} m (initial={initial_error:.4} m, attempt={attempt}, shoulder={shoulder_velocity_rad_s:.1} rad/s, joint_state={})",
                        sim.observe().joint_state_count
                    );
                    return;
                }
                let _ = sim.reset();
            }
        }
        eprintln!(
            "smoke failed: could not reach within {REACH_SUCCESS_M} m after {MAX_SMOKE_ATTEMPTS} attempts"
        );
        std::process::exit(1);
    }

    let mut sim = MobileManipulatorSim::from_scene_path(&mm_minimal_scene_path())
        .expect("load mm_minimal scene");
    let initial = sim.observe();
    let initial_error = ee_distance_to_target_m(&initial, POSE_TARGET);
    let (_, final_error) = run_reach(&mut sim, SHOULDER_VELOCITY_RAD_S);
    let final_obs = sim.observe();

    println!(
        "target ee = ({:.3}, {:.3}, {:.3})",
        POSE_TARGET.x_m, POSE_TARGET.y_m, POSE_TARGET.z_m
    );
    println!(
        "initial ee = ({:.3}, {:.3}, {:.3}) error={initial_error:.4} m",
        initial.ee_x_m, initial.ee_y_m, initial.ee_z_m
    );
    println!(
        "final ee   = ({:.3}, {:.3}, {:.3}) error={final_error:.4} m",
        final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m
    );
    println!(
        "shoulder = {:.3} rad, elbow = {:.3} rad",
        final_obs.shoulder_position_rad, final_obs.elbow_position_rad
    );
}
