//! Joint-space trajectory generation and velocity tracking for position motors.

use crate::action::MobileManipulatorAction;
use crate::mm_lift_kinematics::MmLiftJointTarget;
use crate::observation::MobileManipulatorObservation;

/// Linear joint-space trajectory between two configurations over a fixed step count.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTrajectory {
    start: MmLiftJointTarget,
    end: MmLiftJointTarget,
    steps: u64,
    step: u64,
}

impl JointTrajectory {
    /// Creates a trajectory that reaches `end` in `steps` simulation ticks.
    pub fn new(start: MmLiftJointTarget, end: MmLiftJointTarget, steps: u64) -> Self {
        Self {
            start,
            end,
            steps: steps.max(1),
            step: 0,
        }
    }

    /// Returns true once the trajectory has finished.
    pub fn is_complete(&self) -> bool {
        self.step >= self.steps
    }

    /// Advances one tick and returns the interpolated joint target for this step.
    pub fn advance(&mut self) -> MmLiftJointTarget {
        self.step = (self.step + 1).min(self.steps);
        let t = self.step as f64 / self.steps as f64;
        lerp_joint_target(self.start, self.end, t)
    }

    /// Current waypoint without advancing.
    pub fn sample(&self) -> MmLiftJointTarget {
        let t = self.step as f64 / self.steps as f64;
        lerp_joint_target(self.start, self.end, t)
    }
}

/// Velocity limits used when tracking a joint-space target each simulation tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTrackingLimits {
    /// Maximum lift speed in meters per second.
    pub max_lift_velocity_m_s: f64,
    /// Maximum revolute joint speed in radians per second.
    pub max_joint_velocity_rad_s: f64,
}

impl Default for JointTrackingLimits {
    fn default() -> Self {
        Self {
            max_lift_velocity_m_s: 0.3,
            max_joint_velocity_rad_s: 1.2,
        }
    }
}

/// Proportional joint velocities that drive position motors toward a joint target.
pub fn joint_tracking_action(
    obs: &MobileManipulatorObservation,
    lift_position_m: f64,
    target: MmLiftJointTarget,
    limits: JointTrackingLimits,
) -> MobileManipulatorAction {
    MobileManipulatorAction {
        lift_velocity_m_s: clamp(
            8.0 * (target.lift_m - lift_position_m),
            limits.max_lift_velocity_m_s,
        ),
        shoulder_velocity_rad_s: clamp(
            12.0 * (target.shoulder_rad - obs.shoulder_position_rad),
            limits.max_joint_velocity_rad_s,
        ),
        elbow_velocity_rad_s: clamp(
            12.0 * (target.elbow_rad - obs.elbow_position_rad),
            limits.max_joint_velocity_rad_s,
        ),
        ..MobileManipulatorAction::default()
    }
}

/// Holds absolute lift-arm joint targets while optionally driving the gripper.
pub fn hold_lift_joint_action(
    target: MmLiftJointTarget,
    gripper_velocity_rad_s: f64,
) -> MobileManipulatorAction {
    MobileManipulatorAction {
        gripper_velocity_rad_s,
        lift_joint_target: Some(target),
        ..MobileManipulatorAction::default()
    }
}

fn lerp_joint_target(
    start: MmLiftJointTarget,
    end: MmLiftJointTarget,
    t: f64,
) -> MmLiftJointTarget {
    MmLiftJointTarget {
        lift_m: start.lift_m + (end.lift_m - start.lift_m) * t,
        shoulder_rad: start.shoulder_rad + (end.shoulder_rad - start.shoulder_rad) * t,
        elbow_rad: start.elbow_rad + (end.elbow_rad - start.elbow_rad) * t,
    }
}

fn clamp(value: f64, max_abs: f64) -> f64 {
    value.clamp(-max_abs, max_abs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn trajectory_reaches_end() {
        let start = MmLiftJointTarget {
            lift_m: 0.0,
            shoulder_rad: 0.0,
            elbow_rad: 0.0,
        };
        let end = MmLiftJointTarget {
            lift_m: 0.2,
            shoulder_rad: 0.5,
            elbow_rad: -0.3,
        };
        let mut traj = JointTrajectory::new(start, end, 10);
        let mut last = start;
        while !traj.is_complete() {
            last = traj.advance();
        }
        assert_relative_eq!(last.lift_m, end.lift_m, epsilon = 1e-9);
        assert_relative_eq!(last.shoulder_rad, end.shoulder_rad, epsilon = 1e-9);
        assert_relative_eq!(last.elbow_rad, end.elbow_rad, epsilon = 1e-9);
    }
}
