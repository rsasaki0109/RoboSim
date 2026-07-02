//! Policy traits for controlling episodes.

use crate::episode::Episode;
use crate::mm_lift_kinematics::{MmLiftGripperTarget, MmLiftJointTarget, MmLiftKinematics};
use crate::observation::MobileManipulatorObservation;

/// Maps observations to actions for a specific episode type.
pub trait Policy<E: Episode> {
    /// Chooses the next action from the latest observation.
    fn act(&mut self, observation: &E::Observation) -> E::Action;
}

/// Drives both wheels at a fixed velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstantVelocityPolicy {
    velocity_rad_s: f64,
}

impl ConstantVelocityPolicy {
    /// Creates a policy that commands equal wheel speeds.
    pub fn new(velocity_rad_s: f64) -> Self {
        Self { velocity_rad_s }
    }
}

impl Policy<crate::env::DiffDriveEpisode> for ConstantVelocityPolicy {
    fn act(&mut self, _observation: &crate::DiffDriveObservation) -> crate::DiffDriveAction {
        crate::DiffDriveAction::forward(self.velocity_rad_s)
    }
}

const LOWER_TO_PICK: u64 = 200;
const GRASP: u64 = LOWER_TO_PICK + 120;
const LIFT: u64 = GRASP + 150;
const SETTLE_AFTER_SWING: u64 = 150;
const LOWER_TO_PLACE: u64 = 200;
const RELEASE: u64 = 120;
const DEFAULT_CARRY_Y_M: f64 = 0.35;
const CARRY_JOINT_RATE_RAD_S: f64 = 0.8;

/// Scripted pick-and-place policy for the `mm_lift` robot: a fixed-timing state machine
/// that lowers the top-down claw over the cube, grasps it, lifts it, swings the arm to a
/// new spot, lowers it, and opens to release. Drives the same trajectory used by the
/// `lift_pick_place` episode and example 31, so they share one implementation.
///
/// The carry swing uses a fixed shoulder rate; longer [`Self::with_swing_steps`] values
/// rotate further and place the cube farther around the column. Prefer
/// [`IkLiftPickPlacePolicy`] when targeting arbitrary place poses via analytic IK.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftPickPlacePolicy {
    step: u64,
    swing_steps: u64,
}

impl Default for LiftPickPlacePolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl LiftPickPlacePolicy {
    /// Default number of swing steps, carrying the cube ~1.1 m to one side.
    pub const DEFAULT_SWING_STEPS: u64 = 90;

    /// Creates a policy at the start of the pick-and-place sequence.
    pub fn new() -> Self {
        Self::with_swing_steps(Self::DEFAULT_SWING_STEPS)
    }

    /// Creates a policy whose carry swing lasts `swing_steps` steps — more steps rotate
    /// the arm further, placing the cube farther around the column.
    pub fn with_swing_steps(swing_steps: u64) -> Self {
        Self {
            step: 0,
            swing_steps,
        }
    }

    /// Total number of steps the sequence runs (after which it commands no motion).
    pub fn total_steps(&self) -> u64 {
        pick_place_total_steps(self.swing_steps)
    }

    /// Returns the action for the current step and advances the state machine.
    pub fn next_action(
        &mut self,
        _observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;
        let swing = LIFT + self.swing_steps;
        let settle = swing + SETTLE_AFTER_SWING;
        let lower_to_place = settle + LOWER_TO_PLACE;
        let release = lower_to_place + RELEASE;

        let s = self.step;
        self.step += 1;
        if s < LOWER_TO_PICK {
            MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            }
        } else if s < GRASP {
            MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            }
        } else if s < LIFT {
            MobileManipulatorAction {
                lift_velocity_m_s: 0.3,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < swing {
            MobileManipulatorAction {
                shoulder_velocity_rad_s: CARRY_JOINT_RATE_RAD_S,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < settle {
            MobileManipulatorAction {
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < lower_to_place {
            MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < release {
            MobileManipulatorAction {
                gripper_velocity_rad_s: 3.0,
                ..MobileManipulatorAction::default()
            }
        } else {
            MobileManipulatorAction::default()
        }
    }

    /// Analytic kinematics for the `mm_lift` arm (used by IK-based policies).
    pub fn kinematics() -> MmLiftKinematics {
        MmLiftKinematics::mm_lift()
    }

    /// World-frame place target used by the default `lift_pick_place` episode.
    pub fn default_place_target() -> MmLiftGripperTarget {
        default_place_target()
    }
}

impl Policy<crate::MobileManipulatorEpisode> for LiftPickPlacePolicy {
    fn act(
        &mut self,
        observation: &crate::MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        self.next_action(observation)
    }
}

/// Pick-and-place policy that solves carry targets with [`MmLiftKinematics`] and drives
/// the arm toward the IK joint solution at a fixed joint rate during the swing phase.
#[derive(Clone, Debug, PartialEq)]
pub struct IkLiftPickPlacePolicy {
    step: u64,
    swing_steps: u64,
    kin: MmLiftKinematics,
    carry_target: MmLiftGripperTarget,
    carry_hold: Option<MmLiftJointTarget>,
}

impl Default for IkLiftPickPlacePolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl IkLiftPickPlacePolicy {
    /// Default number of swing steps tracking the IK carry pose.
    pub const DEFAULT_SWING_STEPS: u64 = LiftPickPlacePolicy::DEFAULT_SWING_STEPS;

    /// Creates a policy at the start of the pick-and-place sequence.
    pub fn new() -> Self {
        Self::with_swing_steps(Self::DEFAULT_SWING_STEPS)
    }

    /// Creates a policy whose carry swing lasts `swing_steps` steps.
    pub fn with_swing_steps(swing_steps: u64) -> Self {
        Self {
            step: 0,
            swing_steps,
            kin: MmLiftKinematics::mm_lift(),
            carry_target: carry_target_for_swing(swing_steps),
            carry_hold: None,
        }
    }

    /// Overrides the world-frame gripper-base target used during the carry swing.
    pub fn with_carry_target(mut self, target: MmLiftGripperTarget) -> Self {
        self.carry_target = target;
        self
    }

    /// Total number of steps the sequence runs (after which it commands no motion).
    pub fn total_steps(&self) -> u64 {
        pick_place_total_steps(self.swing_steps)
    }

    /// Returns the action for the current step and advances the state machine.
    pub fn next_action(
        &mut self,
        observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;
        let swing = LIFT + self.swing_steps;
        let settle = swing + SETTLE_AFTER_SWING;
        let lower_to_place = settle + LOWER_TO_PLACE;
        let release = lower_to_place + RELEASE;

        let s = self.step;
        self.step += 1;
        if s < LOWER_TO_PICK {
            MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            }
        } else if s < GRASP {
            MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            }
        } else if s < LIFT {
            MobileManipulatorAction {
                lift_velocity_m_s: 0.3,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < swing {
            self.carry_action(observation, s == LIFT)
        } else if s < settle {
            MobileManipulatorAction {
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < lower_to_place {
            MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            }
        } else if s < release {
            MobileManipulatorAction {
                gripper_velocity_rad_s: 3.0,
                ..MobileManipulatorAction::default()
            }
        } else {
            MobileManipulatorAction::default()
        }
    }

    fn carry_action(
        &mut self,
        observation: &MobileManipulatorObservation,
        start_swing: bool,
    ) -> crate::MobileManipulatorAction {
        if start_swing && self.carry_hold.is_none() {
            self.carry_hold = Some(
                self.kin
                    .inverse_kinematics_at_lift(
                        observation.lift_position_m,
                        self.carry_target.x_m,
                        self.carry_target.z_m,
                    )
                    .expect("carry target must be reachable"),
            );
        }
        let target = self
            .carry_hold
            .expect("carry hold target must be initialized");
        let mut action = joint_rate_toward_target(observation, target, CARRY_JOINT_RATE_RAD_S);
        action.gripper_velocity_rad_s = -2.0;
        action
    }
}

impl Policy<crate::MobileManipulatorEpisode> for IkLiftPickPlacePolicy {
    fn act(
        &mut self,
        observation: &crate::MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        self.next_action(observation)
    }
}

fn pick_place_total_steps(swing_steps: u64) -> u64 {
    LIFT + swing_steps + SETTLE_AFTER_SWING + LOWER_TO_PLACE + RELEASE
}

fn default_place_target() -> MmLiftGripperTarget {
    MmLiftGripperTarget::new(0.55, 0.03, -0.87)
}

fn carry_target_for_swing(swing_steps: u64) -> MmLiftGripperTarget {
    let kin = MmLiftKinematics::mm_lift();
    let (shoulder_x, _) = kin.shoulder_xz_m(0.0);
    let place = default_place_target();
    let dx = place.x_m - shoulder_x;
    let place_angle = place.z_m.atan2(dx);
    let sweep = (swing_steps as f64 / LiftPickPlacePolicy::DEFAULT_SWING_STEPS as f64).min(1.0);
    let reach_m = 0.86 * sweep;
    MmLiftGripperTarget::new(
        shoulder_x + reach_m * place_angle.cos(),
        DEFAULT_CARRY_Y_M,
        reach_m * place_angle.sin(),
    )
}

fn joint_rate_toward_target(
    observation: &MobileManipulatorObservation,
    target: MmLiftJointTarget,
    max_rate_rad_s: f64,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    MobileManipulatorAction {
        lift_velocity_m_s: signed_rate_toward(
            observation.lift_position_m,
            target.lift_m,
            0.3,
            0.02,
        ),
        shoulder_velocity_rad_s: signed_rate_toward(
            observation.shoulder_position_rad,
            target.shoulder_rad,
            max_rate_rad_s,
            0.05,
        ),
        elbow_velocity_rad_s: signed_rate_toward(
            observation.elbow_position_rad,
            target.elbow_rad,
            max_rate_rad_s,
            0.05,
        ),
        ..MobileManipulatorAction::default()
    }
}

fn signed_rate_toward(current: f64, target: f64, max_rate: f64, tolerance: f64) -> f64 {
    let error = target - current;
    if error.abs() < tolerance {
        0.0
    } else {
        error.signum() * max_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn ik_carry_target_points_at_default_place() {
        let target = carry_target_for_swing(IkLiftPickPlacePolicy::DEFAULT_SWING_STEPS);
        let place = default_place_target();
        assert_relative_eq!(target.x_m, place.x_m, epsilon = 0.05);
        assert!(target.z_m < 0.0);
        assert_relative_eq!(target.z_m, place.z_m, epsilon = 0.05);
    }

    #[test]
    fn ik_swing_targets_differ_by_step_count() {
        let near = carry_target_for_swing(60);
        let far = carry_target_for_swing(120);
        let separation = ((far.x_m - near.x_m).powi(2) + (far.z_m - near.z_m).powi(2)).sqrt();
        assert!(
            separation > 0.15,
            "swing step count should scale carry reach: near={near:?}, far={far:?}, separation={separation:.2} m"
        );
    }
}
