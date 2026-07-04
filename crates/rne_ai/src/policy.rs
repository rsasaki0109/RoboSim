//! Policy traits for controlling episodes.

use crate::episode::Episode;
use crate::mm_lift_kinematics::{MmLiftGripperTarget, MmLiftJointTarget, MmLiftKinematics};
use crate::mm_minimal_kinematics::{
    MmMinimalGripperTarget, MmMinimalJointTarget, MmMinimalKinematics,
};
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

const CLUTTER_SETTLE_STEPS: u64 = 20;
const CLUTTER_APPROACH_STEPS: u64 = 360;
const CLUTTER_IK_CARRY_STEPS: u64 = 340;
const CLUTTER_HOLD_STEPS: u64 = 80;
const CLUTTER_RELEASE_STEPS: u64 = 150;
const CLUTTER_CARRY_SHOULDER_RAD_S: f64 = -0.50;
const CLUTTER_CARRY_ELBOW_RAD_S: f64 = -0.69;
const CLUTTER_CARRY_GRIPPER_RAD_S: f64 = -2.5;

const MOBILE_CLUTTER_SETTLE_STEPS: u64 = 80;
const MOBILE_CLUTTER_DRIVE_STEPS: u64 = 480;
const MOBILE_CLUTTER_MIN_DRIVE_STEPS: u64 = 40;
const MOBILE_CLUTTER_ARM_SETTLE_STEPS: u64 = 120;
const MOBILE_CLUTTER_MIN_GRIPPER_DX_M: f64 = 0.04;
const MOBILE_CLUTTER_IK_CARRY_STEPS: u64 = CLUTTER_IK_CARRY_STEPS;
const MOBILE_CLUTTER_HOLD_STEPS: u64 = CLUTTER_HOLD_STEPS;
const MOBILE_CLUTTER_RELEASE_STEPS: u64 = CLUTTER_RELEASE_STEPS;

/// IK-assisted navigate → pick → place for `mm_mobile` clutter episodes.
///
/// Drives the diff-drive base toward the pick target, then reuses the fixed-base clutter
/// arm phases (approach → tuned carry → hold → release) toward
/// [`mm_mobile_clutter_place_target`](crate::mm_mobile_clutter_place_target).
#[derive(Clone, Debug, PartialEq)]
pub struct IkMobileClutterPickPlacePolicy {
    step: u64,
    kin: MmMinimalKinematics,
}

impl Default for IkMobileClutterPickPlacePolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl IkMobileClutterPickPlacePolicy {
    /// Creates a policy for the default mobile clutter place target.
    pub fn new() -> Self {
        Self {
            step: 0,
            kin: MmMinimalKinematics::mm_mobile(),
        }
    }

    /// Total scripted steps (settle → drive → arm settle → approach → carry → hold → release).
    pub fn total_steps(&self) -> u64 {
        MOBILE_CLUTTER_SETTLE_STEPS
            + MOBILE_CLUTTER_DRIVE_STEPS
            + MOBILE_CLUTTER_ARM_SETTLE_STEPS
            + CLUTTER_APPROACH_STEPS
            + MOBILE_CLUTTER_IK_CARRY_STEPS
            + MOBILE_CLUTTER_HOLD_STEPS
            + MOBILE_CLUTTER_RELEASE_STEPS
    }

    /// Step index where the arm settle / approach phases begin (after base drive).
    pub fn arm_start_step(&self) -> u64 {
        self.mobile_arm_start_step()
    }

    /// Overrides the internal step counter (for composing drive + arm phases in tests).
    pub fn set_step(&mut self, step: u64) {
        self.step = step;
    }

    /// Returns the internal step counter.
    pub fn current_step(&self) -> u64 {
        self.step
    }

    fn mobile_arm_start_step(&self) -> u64 {
        MOBILE_CLUTTER_SETTLE_STEPS + MOBILE_CLUTTER_DRIVE_STEPS + MOBILE_CLUTTER_ARM_SETTLE_STEPS
    }

    /// Returns the action for the current step and advances the internal counter.
    pub fn next_action(
        &mut self,
        observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;

        let arm_start = self.mobile_arm_start_step();
        let approach_end = arm_start + CLUTTER_APPROACH_STEPS;
        let carry_end = approach_end + MOBILE_CLUTTER_IK_CARRY_STEPS;
        let hold_end = carry_end + MOBILE_CLUTTER_HOLD_STEPS;
        let release_end = hold_end + MOBILE_CLUTTER_RELEASE_STEPS;

        let s = self.step;
        self.step += 1;

        if s < MOBILE_CLUTTER_SETTLE_STEPS {
            MobileManipulatorAction::default()
        } else if s < MOBILE_CLUTTER_SETTLE_STEPS + MOBILE_CLUTTER_DRIVE_STEPS {
            let action = mobile_clutter_drive_action(observation);
            let driven = s - MOBILE_CLUTTER_SETTLE_STEPS + 1;
            if driven >= MOBILE_CLUTTER_MIN_DRIVE_STEPS && mobile_drive_stop_for_arm_m(observation)
            {
                self.step = MOBILE_CLUTTER_SETTLE_STEPS + MOBILE_CLUTTER_DRIVE_STEPS;
            }
            action
        } else if s < arm_start {
            mobile_clutter_backup_action(observation)
        } else if s < approach_end {
            mobile_clutter_approach_action(observation, &self.kin)
        } else if s < carry_end {
            clutter_carry_action(observation)
        } else if s < hold_end {
            MobileManipulatorAction {
                gripper_velocity_rad_s: CLUTTER_CARRY_GRIPPER_RAD_S,
                ..MobileManipulatorAction::default()
            }
        } else if s < release_end {
            MobileManipulatorAction {
                gripper_velocity_rad_s: 3.0,
                ..MobileManipulatorAction::default()
            }
        } else {
            MobileManipulatorAction::default()
        }
    }
}

impl Policy<crate::MobileManipulatorEpisode> for IkMobileClutterPickPlacePolicy {
    fn act(
        &mut self,
        observation: &crate::MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        self.next_action(observation)
    }
}

/// IK-assisted pick-and-place for the fixed-base `mm_minimal` clutter episodes.
///
/// Uses analytic IK during approach, then a tuned fixed-velocity carry that tracks
/// object-to-place deltas in simulation before release on the ground target.
#[derive(Clone, Debug, PartialEq)]
pub struct IkClutterPickPlacePolicy {
    step: u64,
    kin: MmMinimalKinematics,
}

impl Default for IkClutterPickPlacePolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl IkClutterPickPlacePolicy {
    /// Creates a policy for the default clutter place target.
    pub fn new() -> Self {
        Self {
            step: 0,
            kin: MmMinimalKinematics::mm_minimal(),
        }
    }

    /// Total scripted steps (settle → approach → carry → hold → release).
    pub fn total_steps(&self) -> u64 {
        CLUTTER_SETTLE_STEPS
            + CLUTTER_APPROACH_STEPS
            + CLUTTER_IK_CARRY_STEPS
            + CLUTTER_HOLD_STEPS
            + CLUTTER_RELEASE_STEPS
    }

    /// Returns the action for the current step and advances the internal counter.
    pub fn next_action(
        &mut self,
        observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;
        let approach_end = CLUTTER_SETTLE_STEPS + CLUTTER_APPROACH_STEPS;
        let carry_end = approach_end + CLUTTER_IK_CARRY_STEPS;
        let hold_end = carry_end + CLUTTER_HOLD_STEPS;
        let release_end = hold_end + CLUTTER_RELEASE_STEPS;

        let s = self.step;
        self.step += 1;

        if s < CLUTTER_SETTLE_STEPS {
            MobileManipulatorAction::default()
        } else if s < approach_end {
            clutter_approach_action(observation, &self.kin)
        } else if s < carry_end {
            clutter_carry_action(observation)
        } else if s < hold_end {
            MobileManipulatorAction {
                gripper_velocity_rad_s: CLUTTER_CARRY_GRIPPER_RAD_S,
                ..MobileManipulatorAction::default()
            }
        } else if s < release_end {
            MobileManipulatorAction {
                gripper_velocity_rad_s: 3.0,
                ..MobileManipulatorAction::default()
            }
        } else {
            MobileManipulatorAction::default()
        }
    }
}

impl Policy<crate::MobileManipulatorEpisode> for IkClutterPickPlacePolicy {
    fn act(
        &mut self,
        observation: &crate::MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        self.next_action(observation)
    }
}

fn clutter_approach_action(
    observation: &MobileManipulatorObservation,
    kin: &MmMinimalKinematics,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    let object_x_m = if observation.pick_object_y_m != 0.0 {
        observation.pick_object_x_m
    } else {
        observation.ee_x_m + observation.target_dx_m
    };
    let object_z_m = if observation.pick_object_y_m != 0.0 {
        observation.pick_object_z_m
    } else {
        observation.ee_z_m + observation.target_dz_m
    };
    let target = MmMinimalGripperTarget::new(object_x_m, kin.shoulder_y_m(), object_z_m);
    if let Ok(joints) = kin.inverse_kinematics(target) {
        let mut action = joint_rate_toward_minimal(observation, joints, CARRY_JOINT_RATE_RAD_S);
        action.gripper_velocity_rad_s = -2.5;
        action
    } else {
        MobileManipulatorAction {
            shoulder_velocity_rad_s: (4.0 * observation.target_dx_m).clamp(-6.0, 6.0),
            elbow_velocity_rad_s: (4.0 * observation.target_dz_m).clamp(-6.0, 6.0),
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        }
    }
}

fn clutter_carry_action(
    _observation: &MobileManipulatorObservation,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    MobileManipulatorAction {
        gripper_velocity_rad_s: CLUTTER_CARRY_GRIPPER_RAD_S,
        shoulder_velocity_rad_s: CLUTTER_CARRY_SHOULDER_RAD_S,
        elbow_velocity_rad_s: CLUTTER_CARRY_ELBOW_RAD_S,
        ..MobileManipulatorAction::default()
    }
}

fn mobile_clutter_approach_action(
    observation: &MobileManipulatorObservation,
    kin: &MmMinimalKinematics,
) -> crate::MobileManipulatorAction {
    let (object_x_m, object_z_m) = mobile_pick_object_xz(observation);
    let target = MmMinimalGripperTarget::new(
        object_x_m,
        kin.shoulder_y_at_base(observation.base_y_m),
        object_z_m,
    );
    if let Ok(joints) = kin.inverse_kinematics_at_base(
        observation.base_x_m,
        observation.base_y_m,
        observation.base_z_m,
        observation.base_yaw_rad,
        target,
    ) {
        let mut action = joint_rate_toward_minimal(observation, joints, CARRY_JOINT_RATE_RAD_S);
        action.gripper_velocity_rad_s = -2.5;
        action
    } else {
        mobile_gripper_approach_action(observation)
    }
}

fn mobile_gripper_approach_action(
    observation: &MobileManipulatorObservation,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    MobileManipulatorAction {
        shoulder_velocity_rad_s: (4.0 * mobile_gripper_pick_dx(observation)).clamp(-6.0, 6.0),
        elbow_velocity_rad_s: (4.0 * mobile_gripper_pick_dz(observation)).clamp(-6.0, 6.0),
        gripper_velocity_rad_s: -2.5,
        ..MobileManipulatorAction::default()
    }
}

fn mobile_pick_object_xz(observation: &MobileManipulatorObservation) -> (f64, f64) {
    if observation.pick_object_y_m != 0.0 {
        (observation.pick_object_x_m, observation.pick_object_z_m)
    } else {
        (
            observation.ee_x_m + observation.target_dx_m,
            observation.ee_z_m + observation.target_dz_m,
        )
    }
}

fn wrap_heading_rad(angle: f64) -> f64 {
    let mut wrapped = angle.rem_euclid(std::f64::consts::TAU);
    if wrapped > std::f64::consts::PI {
        wrapped -= std::f64::consts::TAU;
    }
    wrapped
}

fn mobile_clutter_drive_action(
    observation: &MobileManipulatorObservation,
) -> crate::MobileManipulatorAction {
    use crate::mm_mobile_twist_to_wheel_velocities;
    use crate::MobileManipulatorAction;

    let (object_x_m, object_z_m) = mobile_pick_object_xz(observation);
    let dx_world = object_x_m - observation.base_x_m;
    let dz_world = object_z_m - observation.base_z_m;
    let distance_m = (dx_world * dx_world + dz_world * dz_world).sqrt();
    let heading_to_object = dz_world.atan2(dx_world);
    let heading_error = wrap_heading_rad(heading_to_object - observation.base_yaw_rad);
    let forward_m_s = if heading_error.abs() > 0.12 {
        0.0
    } else {
        (0.65 * distance_m).clamp(0.0, 0.25)
    };
    let yaw_rate_rad_s = (-2.0 * heading_error).clamp(-0.7, 0.7);
    let (left, right) = mm_mobile_twist_to_wheel_velocities(forward_m_s, yaw_rate_rad_s);
    MobileManipulatorAction {
        left_wheel_velocity_rad_s: left.clamp(-3.0, 3.0),
        right_wheel_velocity_rad_s: right.clamp(-3.0, 3.0),
        shoulder_velocity_rad_s: 0.0,
        elbow_velocity_rad_s: 0.0,
        gripper_velocity_rad_s: 0.0,
        ..MobileManipulatorAction::default()
    }
}

fn mobile_gripper_pick_dx(observation: &MobileManipulatorObservation) -> f64 {
    observation.gripper_target_dx_m
}

fn mobile_gripper_pick_dz(observation: &MobileManipulatorObservation) -> f64 {
    observation.gripper_target_dz_m
}

/// Drive stops once the base is within arm reach of the pick object or has overshot it.
fn mobile_drive_stop_for_arm_m(observation: &MobileManipulatorObservation) -> bool {
    let (object_x_m, object_z_m) = mobile_pick_object_xz(observation);
    let dx = object_x_m - observation.base_x_m;
    let dz = object_z_m - observation.base_z_m;
    let base_dist = (dx * dx + dz * dz).sqrt();
    base_dist < 0.55 || dx < 0.08
}

fn mobile_clutter_backup_action(
    observation: &MobileManipulatorObservation,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    let gripper_dx = mobile_gripper_pick_dx(observation);
    if gripper_dx >= MOBILE_CLUTTER_MIN_GRIPPER_DX_M {
        return MobileManipulatorAction::default();
    }
    if gripper_dx < 0.0 {
        return MobileManipulatorAction {
            shoulder_velocity_rad_s: (3.0 * gripper_dx).clamp(-4.0, 0.0),
            elbow_velocity_rad_s: (3.0 * mobile_gripper_pick_dz(observation)).clamp(-4.0, 4.0),
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
    }
    MobileManipulatorAction {
        shoulder_velocity_rad_s: (4.0 * gripper_dx).clamp(0.0, 4.0),
        elbow_velocity_rad_s: (3.0 * mobile_gripper_pick_dz(observation)).clamp(-4.0, 4.0),
        gripper_velocity_rad_s: -2.5,
        ..MobileManipulatorAction::default()
    }
}

fn joint_rate_toward_minimal(
    observation: &MobileManipulatorObservation,
    target: MmMinimalJointTarget,
    max_rate_rad_s: f64,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    MobileManipulatorAction {
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

/// Goal-conditioned reach policy that scales arm motion from wrist depth.
///
/// Uses the center-pixel depth from the wrist RGB-D stream to slow the arm near
/// obstacles and speed up in open space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisuomotorReachPolicy {
    /// Shoulder gain on goal-relative X error.
    pub shoulder_gain: f64,
    /// Elbow gain on goal-relative Z error.
    pub elbow_gain: f64,
    /// Depth in meters treated as a nominal stand-off distance.
    pub depth_bias_m: f64,
}

impl Default for VisuomotorReachPolicy {
    fn default() -> Self {
        Self {
            shoulder_gain: 2.5,
            elbow_gain: 3.0,
            depth_bias_m: 0.55,
        }
    }
}

impl VisuomotorReachPolicy {
    fn depth_scale(&self, observation: &MobileManipulatorObservation) -> f64 {
        if observation.wrist_depth_center_m <= 0.0 {
            return 1.0;
        }
        (self.depth_bias_m / observation.wrist_depth_center_m).clamp(0.35, 1.5)
    }
}

impl Policy<crate::env::MobileManipulatorEpisode> for VisuomotorReachPolicy {
    fn act(
        &mut self,
        observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        let scale = self.depth_scale(observation);
        crate::MobileManipulatorAction {
            shoulder_velocity_rad_s: (self.shoulder_gain * observation.target_dx_m * scale)
                .clamp(-6.0, 6.0),
            elbow_velocity_rad_s: (self.elbow_gain * observation.target_dz_m * scale)
                .clamp(-6.0, 6.0),
            ..Default::default()
        }
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

    #[test]
    fn ik_clutter_policy_total_steps_matches_phases() {
        let policy = IkClutterPickPlacePolicy::new();
        assert_eq!(policy.total_steps(), 950);
    }

    #[test]
    fn ik_mobile_clutter_policy_total_steps_matches_phases() {
        let policy = IkMobileClutterPickPlacePolicy::new();
        assert_eq!(policy.total_steps(), 1610);
    }

    #[test]
    fn visuomotor_depth_scale_clamps_and_defaults_without_depth() {
        let mut policy = VisuomotorReachPolicy::default();
        let mut obs = crate::MobileManipulatorObservation {
            target_dx_m: 1.0,
            wrist_depth_center_m: 0.0,
            ..Default::default()
        };
        let action = policy.act(&obs);
        assert_relative_eq!(action.shoulder_velocity_rad_s, 2.5, epsilon = 1e-9);

        obs.wrist_depth_center_m = 0.2;
        let action = policy.act(&obs);
        assert_relative_eq!(action.shoulder_velocity_rad_s, 3.75, epsilon = 1e-9);

        obs.wrist_depth_center_m = 5.0;
        let action = policy.act(&obs);
        assert_relative_eq!(action.shoulder_velocity_rad_s, 0.875, epsilon = 1e-9);
    }
}
