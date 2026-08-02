//! Verifies a Go2 phase-conditioned policy that emits pure joint torques.
//!
//! The startup stand uses position motors only to put the model in a repeatable
//! initial state. Once locomotion starts, all twelve joints are driven by
//! [`UnitreeGo2PureTorquePolicy`]: there is no scripted trot target and no
//! position-PD torque assembled in this example.

use rne_ai::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand,
    UnitreeGo2PureTorquePolicy, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::Vec3;

const JOINTS: [&str; 12] = [
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh",
    "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
];
const CYCLE_STEPS: u64 = 45;
const SETTLE_STEPS: u64 = 240;
const ROLLOUT_STEPS: u64 = 1440;
const POSITION_STIFFNESS: f64 = 180.0;
const POSITION_DAMPING: f64 = 18.0;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;

#[derive(Clone, Copy, Debug)]
struct RolloutOutcome {
    window_a_displacement_m: f64,
    window_b_displacement_m: f64,
    total_displacement_m: f64,
    total_yaw_rad: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
    max_command_nm: f64,
}

fn startup_stand(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: CYCLE_STEPS,
            ..UnitreeGo2GaitCommand::default()
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
}

fn joint_state(sim: &UrdfSceneSim) -> ([f64; 12], [f64; 12]) {
    let positions = JOINTS.map(|link| sim.named_joint_position(link).expect("Go2 joint position"));
    let velocities = JOINTS.map(|link| sim.named_joint_velocity(link).expect("Go2 joint velocity"));
    (positions, velocities)
}

fn rollout(policy: &UnitreeGo2PureTorquePolicy, steps: u64) -> RolloutOutcome {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    startup_stand(&mut sim);

    let up_body_reference = {
        let pose = sim.named_transform("base").expect("Go2 base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let true_tilt = |sim: &UrdfSceneSim| {
        let pose = sim.named_transform("base").expect("Go2 base pose");
        let up = (pose.rotation * up_body_reference).normalize_or_zero();
        up.y.clamp(-1.0, 1.0).acos()
    };

    let start = sim.observe();
    let mut window_start = [start.base_x_m, start.base_z_m];
    let mut window_a_displacement_m = 0.0;
    let mut window_b_displacement_m = 0.0;
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut total_yaw_rad = 0.0;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad = 0.0_f64;
    let mut max_command_nm = 0.0_f64;

    for step in 0..steps {
        let (positions, velocities) = joint_state(&sim);
        let phase = (step % CYCLE_STEPS) as f64 / CYCLE_STEPS as f64;
        let commands = policy.torques_nm(phase, &positions, &velocities, TORQUE_LIMIT_NM);
        max_command_nm = max_command_nm.max(
            commands
                .iter()
                .map(|command| command.abs())
                .fold(0.0_f64, f64::max),
        );
        let torque_targets: Vec<UrdfJointTorqueTarget<'_>> = JOINTS
            .iter()
            .zip(commands.iter())
            .map(|(link_name, torque_nm)| UrdfJointTorqueTarget {
                link_name,
                torque_nm: *torque_nm,
                max_velocity_rad_s: SPEED_LIMIT_RAD_S,
            })
            .collect();
        sim.step_joint_torques(&torque_targets);

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
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(true_tilt(&sim));

        if step + 1 == 480 {
            window_start = [observed.base_x_m, observed.base_z_m];
        } else if step + 1 == 960 {
            window_a_displacement_m =
                (observed.base_x_m - window_start[0]).hypot(observed.base_z_m - window_start[1]);
            window_start = [observed.base_x_m, observed.base_z_m];
        } else if step + 1 == steps {
            window_b_displacement_m =
                (observed.base_x_m - window_start[0]).hypot(observed.base_z_m - window_start[1]);
        }
    }

    let end = sim.observe();
    RolloutOutcome {
        window_a_displacement_m,
        window_b_displacement_m,
        total_displacement_m: (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m),
        total_yaw_rad,
        min_height_m,
        max_tilt_rad,
        max_command_nm,
    }
}

fn main() {
    let policy = UnitreeGo2PureTorquePolicy::LEARNED_WALK;
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let steps = if smoke { 360 } else { ROLLOUT_STEPS };
    let outcome = rollout(&policy, steps);

    if smoke {
        assert!(
            outcome.min_height_m > 0.15,
            "pure torque smoke must remain standing, min height {:.3} m",
            outcome.min_height_m
        );
        println!(
            "Go2 pure torque smoke ok: minH {:.3} tilt {:.3} maxTau {:.2}",
            outcome.min_height_m, outcome.max_tilt_rad, outcome.max_command_nm
        );
        return;
    }

    let minimum_window_m = outcome
        .window_a_displacement_m
        .min(outcome.window_b_displacement_m);
    assert!(
        minimum_window_m > 0.5,
        "pure torque policy must transport in both windows: {:.3}/{:.3} m",
        outcome.window_a_displacement_m,
        outcome.window_b_displacement_m
    );
    assert!(
        outcome.min_height_m > 0.14 && outcome.max_tilt_rad < 1.5,
        "pure torque policy must stay upright: minH {:.3} tilt {:.3}",
        outcome.min_height_m,
        outcome.max_tilt_rad
    );
    assert!(
        outcome.max_command_nm <= TORQUE_LIMIT_NM + 1.0e-9,
        "policy command exceeded actuator limit: {:.3} N*m",
        outcome.max_command_nm
    );
    println!(
        "Go2 pure torque policy verified: windows {:.3}/{:.3} m, total {:.3} m, yaw {:+.3}, minH {:.3}, tilt {:.3}, maxTau {:.2}",
        outcome.window_a_displacement_m,
        outcome.window_b_displacement_m,
        outcome.total_displacement_m,
        outcome.total_yaw_rad,
        outcome.min_height_m,
        outcome.max_tilt_rad,
        outcome.max_command_nm
    );
}
