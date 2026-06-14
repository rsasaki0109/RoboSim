//! Reach helpers for mobile manipulator end-effector goals.

use crate::action::MobileManipulatorAction;
use crate::observation::MobileManipulatorObservation;

/// World-frame reach target for the end effector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReachTarget {
    /// Target X in meters.
    pub x_m: f64,
    /// Target Y in meters.
    pub y_m: f64,
    /// Target Z in meters.
    pub z_m: f64,
}

impl ReachTarget {
    /// Creates a world-frame reach target.
    pub fn new(x_m: f64, y_m: f64, z_m: f64) -> Self {
        Self { x_m, y_m, z_m }
    }
}

/// Euclidean distance from the observation EE pose to a reach target.
pub fn ee_distance_to_target_m(obs: &MobileManipulatorObservation, target: ReachTarget) -> f64 {
    let dx = obs.ee_x_m - target.x_m;
    let dy = obs.ee_y_m - target.y_m;
    let dz = obs.ee_z_m - target.z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Joint-space reach target for a 2-DOF arm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointReachTarget {
    /// Target shoulder joint angle in radians.
    pub shoulder_rad: f64,
    /// Target elbow joint angle in radians.
    pub elbow_rad: f64,
}

impl JointReachTarget {
    /// Creates a joint-space reach target.
    pub fn new(shoulder_rad: f64, elbow_rad: f64) -> Self {
        Self {
            shoulder_rad,
            elbow_rad,
        }
    }
}

/// Proportional joint velocities toward a joint-space target.
pub fn reach_action_joint_proportional(
    obs: &MobileManipulatorObservation,
    target: JointReachTarget,
    max_joint_velocity_rad_s: f64,
) -> MobileManipulatorAction {
    let shoulder_error_rad = target.shoulder_rad - obs.shoulder_position_rad;
    let elbow_error_rad = target.elbow_rad - obs.elbow_position_rad;

    MobileManipulatorAction {
        left_wheel_velocity_rad_s: 0.0,
        right_wheel_velocity_rad_s: 0.0,
        shoulder_velocity_rad_s: clamp_joint_velocity(
            4.0 * shoulder_error_rad,
            max_joint_velocity_rad_s,
        ),
        elbow_velocity_rad_s: clamp_joint_velocity(4.0 * elbow_error_rad, max_joint_velocity_rad_s),
        gripper_velocity_rad_s: 0.0,
    }
}

/// Proportional joint velocities that drive the EE toward a world-frame target.
pub fn reach_action_proportional(
    obs: &MobileManipulatorObservation,
    target: ReachTarget,
    max_joint_velocity_rad_s: f64,
) -> MobileManipulatorAction {
    let dx = target.x_m - obs.ee_x_m;
    let dy = target.y_m - obs.ee_y_m;
    let dz = target.z_m - obs.ee_z_m;

    let shoulder_velocity_rad_s =
        clamp_joint_velocity(2.5 * dx - 0.5 * dy, max_joint_velocity_rad_s);
    let elbow_velocity_rad_s = clamp_joint_velocity(1.5 * dx + 3.0 * dz, max_joint_velocity_rad_s);

    MobileManipulatorAction {
        left_wheel_velocity_rad_s: 0.0,
        right_wheel_velocity_rad_s: 0.0,
        shoulder_velocity_rad_s,
        elbow_velocity_rad_s,
        gripper_velocity_rad_s: 0.0,
    }
}

fn clamp_joint_velocity(velocity_rad_s: f64, max_abs_rad_s: f64) -> f64 {
    velocity_rad_s.clamp(-max_abs_rad_s, max_abs_rad_s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MobileManipulatorAction, MobileManipulatorSim};

    const MM_MINIMAL_REACH_TARGET: ReachTarget = ReachTarget {
        x_m: 0.456,
        y_m: 0.562,
        z_m: 0.204,
    };

    const REACH_SUCCESS_M: f64 = 0.05;

    #[test]
    fn open_loop_shoulder_reach_within_five_cm() {
        let target = MM_MINIMAL_REACH_TARGET;

        for _ in 0..12 {
            let mut sim = MobileManipulatorSim::new_mm_minimal();
            for shoulder_velocity_rad_s in [3.0, -3.0, 6.0] {
                for _ in 0..720 {
                    sim.step(MobileManipulatorAction {
                        left_wheel_velocity_rad_s: 0.0,
                        right_wheel_velocity_rad_s: 0.0,
                        shoulder_velocity_rad_s,
                        elbow_velocity_rad_s: 0.0,
                        gripper_velocity_rad_s: 0.0,
                    });
                }
                let final_error = ee_distance_to_target_m(&sim.observe(), target);
                if final_error < REACH_SUCCESS_M {
                    return;
                }
                let _ = sim.reset();
            }
        }

        panic!("expected reach error < {REACH_SUCCESS_M} m within retry budget");
    }
}
