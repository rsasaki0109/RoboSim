//! Headless dynamic push-off probe for the Unitree Go2.
//!
//! The body crouches, the knees extend into a saturated torque push-off, and
//! the probe measures how far the stance extends, how upright the body stays,
//! and whether the feet actually leave the ground. It is a measurement, not a
//! demo: the open-loop profile currently extends the stance while the feet stay
//! planted, which matches the boundary recorded in
//! `docs/PLAN_LEGGED_LOCOMOTION_FRONTIER.md`. `--scan` and `--trace` expose the
//! torque sweep and per-step telemetry used to reach that conclusion.
//!
//! Run with `cargo run -p go2_jump --example 106_go2_jump [--scan] [--trace]`.

use rne_ai::{
    unitree_go2_dynamic_scene_path, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::Vec3;

const SETTLE_STEPS: u64 = 240;
const CROUCH_STEPS: u64 = 45;
const LAUNCH_STEPS: u64 = 36;
const FLIGHT_STEPS: u64 = 150;
const LAND_STEPS: u64 = 120;

const POSITION_STIFFNESS: f64 = 420.0;
const POSITION_DAMPING: f64 = 28.0;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;

const STAND_THIGH_RAD: f64 = 0.8;
const STAND_CALF_RAD: f64 = -1.5;
const CROUCH_THIGH_RAD: f64 = 1.25;
const CROUCH_CALF_RAD: f64 = -2.25;
const TUCK_THIGH_RAD: f64 = 0.95;
const TUCK_CALF_RAD: f64 = -1.7;

const PREFIXES: [&str; 4] = ["FL", "FR", "RL", "RR"];

/// One outcome of a headless hop rollout.
#[derive(Clone, Copy, Debug)]
struct JumpOutcome {
    baseline_y_m: f64,
    crouch_y_m: f64,
    apex_y_m: f64,
    final_y_m: f64,
    final_tilt_rad: f64,
    max_tilt_rad: f64,
    airborne_steps: u64,
    landed: bool,
}

fn targets(thigh_rad: f64, calf_rad: f64) -> [UrdfJointPositionTarget<'static>; 12] {
    let mut out = [UrdfJointPositionTarget {
        link_name: "",
        position: 0.0,
    }; 12];
    for (leg, prefix) in PREFIXES.iter().enumerate() {
        let base = leg * 3;
        let (hip, thigh, calf) = match *prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf"),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf"),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf"),
            _ => ("RR_hip", "RR_thigh", "RR_calf"),
        };
        out[base] = UrdfJointPositionTarget {
            link_name: hip,
            position: 0.0,
        };
        out[base + 1] = UrdfJointPositionTarget {
            link_name: thigh,
            position: thigh_rad,
        };
        out[base + 2] = UrdfJointPositionTarget {
            link_name: calf,
            position: calf_rad,
        };
    }
    out
}

fn tilt_rad(sim: &UrdfSceneSim, up_reference: Vec3) -> f64 {
    let pose = sim.named_transform("base").expect("base pose");
    let up = (pose.rotation * up_reference).normalize_or_zero();
    up.y.clamp(-1.0, 1.0).acos()
}

/// Signed fore-aft pitch of the body, in radians (positive is nose-up).
fn pitch_error_rad(sim: &UrdfSceneSim) -> f64 {
    let pose = sim.named_transform("base").expect("base pose");
    let forward = pose.rotation * Vec3::X;
    forward.y.atan2(forward.x)
}

/// Lowest foot-link height in the world, in meters.
fn min_foot_height_m(sim: &UrdfSceneSim) -> f64 {
    PREFIXES
        .iter()
        .filter_map(|prefix| sim.named_transform(&format!("{prefix}_foot")))
        .map(|transform| transform.translation.y)
        .fold(f64::INFINITY, f64::min)
}

const AIRBORNE_FOOT_HEIGHT_M: f64 = 0.04;

fn run_jump(
    crouch_thigh: f64,
    crouch_calf: f64,
    calf_torque_nm: f64,
    thigh_torque_nm: f64,
    pitch_gain: f64,
) -> JumpOutcome {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);

    let stand = targets(STAND_THIGH_RAD, STAND_CALF_RAD);
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let baseline_y_m = sim.observe().base_y_m;
    let up_reference = {
        let pose = sim.named_transform("base").expect("base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };

    let crouch = targets(crouch_thigh, crouch_calf);
    for _ in 0..CROUCH_STEPS {
        sim.step_joint_position_targets(&crouch);
    }
    let crouch_y_m = sim.observe().base_y_m;

    let mut launch_torques: Vec<UrdfJointTorqueTarget<'static>> = Vec::new();
    for prefix in PREFIXES {
        let (hip, thigh, calf) = match prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf"),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf"),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf"),
            _ => ("RR_hip", "RR_thigh", "RR_calf"),
        };
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: hip,
            torque_nm: 0.0,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: thigh,
            torque_nm: thigh_torque_nm,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: calf,
            torque_nm: calf_torque_nm,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
    }

    let mut apex_y_m = f64::MIN;
    let mut airborne_steps = 0_u64;
    let mut landed = false;
    let mut max_tilt_rad = 0.0_f64;
    for _ in 0..LAUNCH_STEPS {
        let correction = pitch_gain * pitch_error_rad(&sim);
        let mut torques = launch_torques.clone();
        torques[1].torque_nm += correction;
        torques[4].torque_nm += correction;
        torques[7].torque_nm -= correction;
        torques[10].torque_nm -= correction;
        sim.step_joint_torques(&torques);
        apex_y_m = apex_y_m.max(sim.observe().base_y_m);
        max_tilt_rad = max_tilt_rad.max(tilt_rad(&sim, up_reference));
        if min_foot_height_m(&sim) > AIRBORNE_FOOT_HEIGHT_M {
            airborne_steps += 1;
        }
    }

    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let tuck = targets(TUCK_THIGH_RAD, TUCK_CALF_RAD);
    for _ in 0..FLIGHT_STEPS {
        sim.step_joint_position_targets(&tuck);
        apex_y_m = apex_y_m.max(sim.observe().base_y_m);
        max_tilt_rad = max_tilt_rad.max(tilt_rad(&sim, up_reference));
        if min_foot_height_m(&sim) > AIRBORNE_FOOT_HEIGHT_M {
            airborne_steps += 1;
        } else if airborne_steps >= 3 {
            landed = true;
            break;
        }
    }

    let stand_again = targets(STAND_THIGH_RAD, STAND_CALF_RAD);
    for _ in 0..LAND_STEPS {
        sim.step_joint_position_targets(&stand_again);
    }
    let final_obs = sim.observe();

    JumpOutcome {
        baseline_y_m,
        crouch_y_m,
        apex_y_m,
        final_y_m: final_obs.base_y_m,
        final_tilt_rad: tilt_rad(&sim, up_reference),
        max_tilt_rad,
        airborne_steps,
        landed,
    }
}

fn trace_jump(calf_torque_nm: f64, thigh_torque_nm: f64) {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand = targets(STAND_THIGH_RAD, STAND_CALF_RAD);
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let up_reference = {
        let pose = sim.named_transform("base").expect("base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let crouch = targets(CROUCH_THIGH_RAD, CROUCH_CALF_RAD);
    for _ in 0..CROUCH_STEPS {
        sim.step_joint_position_targets(&crouch);
    }
    let mut launch_torques: Vec<UrdfJointTorqueTarget<'static>> = Vec::new();
    for prefix in PREFIXES {
        let (hip, thigh, calf) = match prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf"),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf"),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf"),
            _ => ("RR_hip", "RR_thigh", "RR_calf"),
        };
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: hip,
            torque_nm: 0.0,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: thigh,
            torque_nm: thigh_torque_nm,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
        launch_torques.push(UrdfJointTorqueTarget {
            link_name: calf,
            torque_nm: calf_torque_nm,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        });
    }
    for step in 0..LAUNCH_STEPS {
        sim.step_joint_torques(&launch_torques);
        let obs = sim.observe();
        println!(
            "launch {step:03}: y={:.4} foot={:.4} tilt={:.3}",
            obs.base_y_m,
            min_foot_height_m(&sim),
            tilt_rad(&sim, up_reference)
        );
    }
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let tuck = targets(TUCK_THIGH_RAD, TUCK_CALF_RAD);
    for step in 0..FLIGHT_STEPS {
        sim.step_joint_position_targets(&tuck);
        let obs = sim.observe();
        println!(
            "flight {step:03}: y={:.4} foot={:.4} tilt={:.3}",
            obs.base_y_m,
            min_foot_height_m(&sim),
            tilt_rad(&sim, up_reference)
        );
    }
}

fn main() {
    let scan = std::env::args().any(|argument| argument == "--scan");
    let trace = std::env::args().any(|argument| argument == "--trace");
    if trace {
        trace_jump(TORQUE_LIMIT_NM, 0.0);
        return;
    }
    if scan {
        for (crouch_thigh, crouch_calf) in
            [(1.25_f64, -2.25_f64), (1.10, -2.00), (0.95, -1.75)]
        {
            for pitch_gain in [0.0_f64, -40.0, 40.0, -80.0, 80.0] {
                let outcome = run_jump(crouch_thigh, crouch_calf, 23.7, 0.0, pitch_gain);
                println!(
                    "scan crouch=({crouch_thigh:+.2},{crouch_calf:+.2}) pitch_gain={pitch_gain:+.0}: height={:+.3} apex={:.3} airborne={} landed={} final_y={:.3} max_tilt={:.3}",
                    outcome.apex_y_m - outcome.baseline_y_m,
                    outcome.apex_y_m,
                    outcome.airborne_steps,
                    outcome.landed,
                    outcome.final_y_m,
                    outcome.max_tilt_rad,
                );
            }
        }
        return;
    }

    let outcome = run_jump(CROUCH_THIGH_RAD, CROUCH_CALF_RAD, TORQUE_LIMIT_NM, 0.0, 0.0);
    let extension_m = outcome.apex_y_m - outcome.crouch_y_m;
    println!(
        "go2 push-off probe: baseline={:.3} crouch={:.3} apex={:.3} extension={:.3} final={:.3} final_tilt={:.3} rad max_tilt={:.3} rad airborne_steps={}",
        outcome.baseline_y_m,
        outcome.crouch_y_m,
        outcome.apex_y_m,
        extension_m,
        outcome.final_y_m,
        outcome.final_tilt_rad,
        outcome.max_tilt_rad,
        outcome.airborne_steps,
    );
    // Boundary probe, not a passing gate. The open-loop saturated knee push
    // extends the stance by this much with the feet planted, but does not
    // achieve liftoff; this matches docs/PLAN_LEGGED_LOCOMOTION_FRONTIER.md.
    if outcome.airborne_steps > 0 {
        println!(
            "liftoff achieved: {} airborne steps",
            outcome.airborne_steps
        );
    } else {
        println!("no liftoff: feet stay planted through the push-off");
    }
    let recovered = outcome.final_y_m > 0.18 && outcome.final_tilt_rad < 0.5;
    println!("recovered={recovered}");
    if !recovered {
        eprintln!("go2 push-off probe failed to recover");
        std::process::exit(1);
    }
    println!("go2 push-off probe: ok");
}
