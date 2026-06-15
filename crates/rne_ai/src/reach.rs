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

/// Axis-aligned region from which a reach target is sampled each episode.
///
/// Used for goal-conditioned reach: the per-episode target is drawn uniformly from this
/// box so the policy must generalize across targets (the goal is exposed in the
/// observation as `target_d{x,y,z}_m`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReachRandomization {
    /// Minimum-corner target.
    pub min: ReachTarget,
    /// Maximum-corner target.
    pub max: ReachTarget,
    /// Success distance threshold in meters.
    pub success_m: f64,
}

impl ReachRandomization {
    /// Samples a uniform reach target within the region.
    pub fn sample(&self, rng: &mut crate::rng::DeterministicRng) -> ReachTarget {
        ReachTarget::new(
            rng.uniform_f64(self.min.x_m, self.max.x_m),
            rng.uniform_f64(self.min.y_m, self.max.y_m),
            rng.uniform_f64(self.min.z_m, self.max.z_m),
        )
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
    use crate::observation::MobileManipulatorObservation;

    const MM_MINIMAL_JOINT_REACH_TARGET: JointReachTarget = JointReachTarget {
        shoulder_rad: -0.50,
        elbow_rad: 0.05,
    };

    #[test]
    fn reach_action_joint_proportional_points_toward_target() {
        let obs = MobileManipulatorObservation {
            shoulder_position_rad: 0.0,
            elbow_position_rad: 0.0,
            ..MobileManipulatorObservation::default()
        };
        let action = reach_action_joint_proportional(&obs, MM_MINIMAL_JOINT_REACH_TARGET, 6.0);
        assert!(action.shoulder_velocity_rad_s < 0.0);
        assert!(action.elbow_velocity_rad_s > 0.0);
    }

    #[test]
    fn reach_action_proportional_moves_toward_world_target() {
        let obs = MobileManipulatorObservation {
            ee_x_m: 0.40,
            ee_y_m: 0.50,
            ee_z_m: 0.10,
            ..MobileManipulatorObservation::default()
        };
        let target = ReachTarget::new(0.50, 0.60, 0.20);
        let action = reach_action_proportional(&obs, target, 6.0);
        assert!(action.shoulder_velocity_rad_s > 0.0);
        assert!(action.elbow_velocity_rad_s > 0.0);
    }
}
