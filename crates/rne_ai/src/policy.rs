//! Policy traits for controlling episodes.

use crate::episode::Episode;

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

/// Scripted pick-and-place policy for the `mm_lift` robot: a fixed-timing state machine
/// that lowers the top-down claw over the cube, grasps it, lifts it, swings the arm to a
/// new spot, lowers it, and opens to release. Drives the same trajectory used by the
/// `lift_pick_place` episode and example 31, so they share one implementation.
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
        Self {
            step: 0,
            swing_steps: Self::DEFAULT_SWING_STEPS,
        }
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
        940 + self.swing_steps
    }

    /// Returns the action for the current step and advances the state machine.
    pub fn next_action(&mut self) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;
        // Cumulative phase boundaries (steps); the swing phase length is configurable.
        const LOWER_TO_PICK: u64 = 200;
        const GRASP: u64 = LOWER_TO_PICK + 120;
        const LIFT: u64 = GRASP + 150;
        let swing: u64 = LIFT + self.swing_steps;
        let settle: u64 = swing + 150;
        let lower_to_place: u64 = settle + 200;
        let release: u64 = lower_to_place + 120;

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
                shoulder_velocity_rad_s: 0.8,
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
}

impl Policy<crate::MobileManipulatorEpisode> for LiftPickPlacePolicy {
    fn act(
        &mut self,
        _observation: &crate::MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        self.next_action()
    }
}
