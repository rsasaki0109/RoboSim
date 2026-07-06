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

/// Joint rate for the fixed-base clutter approach's IK tracking, distinct from
/// `CARRY_JOINT_RATE_RAD_S`: the shoulder/elbow are now a position-hold servo (see
/// `configure_arm_position_motors`) with real spring lag, and bang-bang control
/// (`signed_rate_toward`'s all-or-nothing max rate) at 0.8 rad/s overshoots the IK
/// target on every approach, then reverses — a sustained limit-cycle chatter that
/// never settles inside the grasp tolerance. A slower rate keeps the overshoot (and
/// so the settle time) small enough that the servo actually reaches the target.
const CLUTTER_APPROACH_RATE_RAD_S: f64 = 0.3;
const CLUTTER_SETTLE_STEPS: u64 = 20;
const CLUTTER_APPROACH_STEPS: u64 = 360;
const CLUTTER_IK_CARRY_STEPS: u64 = 340;
const CLUTTER_HOLD_STEPS: u64 = 80;
const CLUTTER_RELEASE_STEPS: u64 = 150;
const CLUTTER_CARRY_GRIPPER_RAD_S: f64 = -2.5;

const MOBILE_CLUTTER_SETTLE_STEPS: u64 = 80;
const MOBILE_CLUTTER_PICK_DRIVE_STEPS: u64 = 480;
const MOBILE_CLUTTER_RETREAT_STEPS: u64 = 300;
const MOBILE_CLUTTER_CARRY_DRIVE_STEPS: u64 = 480;
const MOBILE_CLUTTER_RELEASE_STEPS: u64 = CLUTTER_RELEASE_STEPS;
/// Object-to-place horizontal distance that gates the gripper release (m); slightly
/// tighter than the episode's 0.12 m place tolerance to absorb the drop.
const MOBILE_CLUTTER_RELEASE_GATE_M: f64 = 0.10;
/// Forward speed cap while poking the gripper into the pick object (m/s).
const MOBILE_CLUTTER_PICK_DRIVE_SPEED_M_S: f64 = 0.15;
/// Forward speed cap while carrying the object toward the place target (m/s).
const MOBILE_CLUTTER_CARRY_DRIVE_SPEED_M_S: f64 = 0.25;
/// Wheel velocity during the post-grasp retreat (rad/s, negative = reverse).
const MOBILE_CLUTTER_RETREAT_WHEEL_RAD_S: f64 = -1.5;
/// Base-to-place-target horizontal distance at which the post-grasp retreat stops
/// (m). Far enough back that the welded object (carried ~0.95 m ahead of the base
/// center) has been dragged clear off the clutter table's near edge, so the carry
/// drive's turn toward the place target swings it in free air instead of grinding
/// it across the tabletop, and so that the subsequent straight carry line passes
/// the table on its open side.
const MOBILE_CLUTTER_RETREAT_DISTANCE_M: f64 = 1.9;

/// Scripted navigate → pick → retreat → carry → release for `mm_mobile` clutter
/// episodes.
///
/// Observation-gated phase machine: drives the base slowly into the pick object with
/// the gripper closing until the contact weld grasps it (the arm stays extended, like
/// the transport pick script), backs straight up until the welded object clears the
/// clutter table, then drives toward
/// [`mm_mobile_clutter_place_target`](crate::mm_mobile_clutter_place_target) with the
/// object carried ahead of the base and releases once the object is over the target.
/// The base does all the transport work: the kinematically pinned platform can drag
/// the payload where the position-held arm's force-limited motors cannot.
#[derive(Clone, Debug, PartialEq)]
pub struct IkMobileClutterPickPlacePolicy {
    step: u64,
}

impl Default for IkMobileClutterPickPlacePolicy {
    fn default() -> Self {
        Self::new()
    }
}

impl IkMobileClutterPickPlacePolicy {
    /// Creates a policy for the default mobile clutter place target.
    pub fn new() -> Self {
        Self { step: 0 }
    }

    /// Total scripted steps (settle → pick drive → retreat → carry drive → release).
    pub fn total_steps(&self) -> u64 {
        MOBILE_CLUTTER_SETTLE_STEPS
            + MOBILE_CLUTTER_PICK_DRIVE_STEPS
            + MOBILE_CLUTTER_RETREAT_STEPS
            + MOBILE_CLUTTER_CARRY_DRIVE_STEPS
            + MOBILE_CLUTTER_RELEASE_STEPS
    }

    /// Step index where the release phase begins (after the drive phases).
    pub fn arm_start_step(&self) -> u64 {
        MOBILE_CLUTTER_SETTLE_STEPS
            + MOBILE_CLUTTER_PICK_DRIVE_STEPS
            + MOBILE_CLUTTER_RETREAT_STEPS
            + MOBILE_CLUTTER_CARRY_DRIVE_STEPS
    }

    /// Overrides the internal step counter (for composing drive + arm phases in tests).
    pub fn set_step(&mut self, step: u64) {
        self.step = step;
    }

    /// Returns the internal step counter.
    pub fn current_step(&self) -> u64 {
        self.step
    }

    /// Returns the action for the current step and advances the internal counter.
    pub fn next_action(
        &mut self,
        observation: &MobileManipulatorObservation,
    ) -> crate::MobileManipulatorAction {
        use crate::MobileManipulatorAction;

        let settle_end = MOBILE_CLUTTER_SETTLE_STEPS;
        let pick_drive_end = settle_end + MOBILE_CLUTTER_PICK_DRIVE_STEPS;
        let retreat_end = pick_drive_end + MOBILE_CLUTTER_RETREAT_STEPS;
        let carry_drive_end = retreat_end + MOBILE_CLUTTER_CARRY_DRIVE_STEPS;
        let release_end = carry_drive_end + MOBILE_CLUTTER_RELEASE_STEPS;

        // Before a grasp the episode reports the pick object pose (nonzero Y for a
        // tabletop object); once grasped it zeroes the pick pose and switches the
        // target deltas to place-target-relative.
        let grasped = observation.pick_object_y_m == 0.0;

        // Observation-gated early phase exits.
        let mut s = self.step;
        if (settle_end..pick_drive_end).contains(&s) && grasped {
            s = pick_drive_end;
        }
        if (pick_drive_end..retreat_end).contains(&s)
            && mobile_place_base_distance_m(observation) > MOBILE_CLUTTER_RETREAT_DISTANCE_M
        {
            s = retreat_end;
        }
        if (retreat_end..carry_drive_end).contains(&s)
            && grasped
            && observation.target_dx_m.hypot(observation.target_dz_m)
                < MOBILE_CLUTTER_RELEASE_GATE_M
        {
            s = carry_drive_end;
        }
        self.step = s + 1;

        if s < settle_end {
            MobileManipulatorAction::default()
        } else if s < pick_drive_end {
            let (object_x_m, object_z_m) = mobile_pick_object_xz(observation);
            let mut action = mobile_drive_toward_action(
                observation,
                object_x_m,
                object_z_m,
                MOBILE_CLUTTER_PICK_DRIVE_SPEED_M_S,
            );
            action.gripper_velocity_rad_s = -2.5;
            action
        } else if s < retreat_end {
            // Back straight up to drag the welded object off the near table edge so
            // it hangs on the weld in free air: the grasp captures the cube pressed
            // a few millimetres into the tabletop, and that contact wedge resists
            // sideways arm motion far harder than the force-limited arm motors can
            // pull. The kinematically pinned base has no such limit.
            MobileManipulatorAction {
                left_wheel_velocity_rad_s: MOBILE_CLUTTER_RETREAT_WHEEL_RAD_S,
                right_wheel_velocity_rad_s: MOBILE_CLUTTER_RETREAT_WHEEL_RAD_S,
                ..MobileManipulatorAction::default()
            }
        } else if s < carry_drive_end {
            // Drive straight at the place target with the object carried ahead of
            // the base; the release gate above fires when the object passes over
            // the target. Gripper velocity stays zero: the contact weld holds the
            // object and only an opening command releases it; continuing to command
            // "close" leaves the limitless fingers flapping against the welded
            // object.
            mobile_drive_toward_action(
                observation,
                crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_X_M,
                crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_Z_M,
                MOBILE_CLUTTER_CARRY_DRIVE_SPEED_M_S,
            )
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
            // `pick_object_y_m` is only filled while the object is still free; once
            // the grasp weld attaches, switch straight to the carry so the approach
            // does not keep integrating toward a stale pick pose and fold the arm
            // onto its own welded payload (a self-jam the position-held servo
            // cannot push out of, since the payload moves rigidly with the arm).
            if observation.pick_object_y_m != 0.0 {
                clutter_approach_action(observation, &self.kin)
            } else {
                clutter_carry_action(observation, &self.kin)
            }
        } else if s < carry_end {
            clutter_carry_action(observation, &self.kin)
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
    // Solve IK (elbow-down branch: for the clutter table's -z pick lane it folds the
    // fingertips onto the object in a tight arc instead of leading with the forearm's
    // side face, which would bulldoze the object off the table without ever making
    // finger contact). When the object lies outside the analytic workspace, aim at
    // the nearest reachable point along the same bearing instead of falling back to
    // a naive Cartesian-error-to-joint-velocity law: against the position-held servo
    // (see `configure_arm_position_motors`), that naive law is a genuine closed-loop
    // instability (a sustained limit cycle), since it maps Cartesian axes directly
    // onto joint velocities without accounting for the arm's coupled Jacobian.
    let joints = kin
        .inverse_kinematics_elbow_down(target)
        .unwrap_or_else(|_| {
            kin.inverse_kinematics_elbow_down(kin.max_reach_toward(object_x_m, object_z_m))
                .expect("max_reach_toward scales inside the analytic workspace")
        });
    let mut action = joint_rate_toward_minimal(observation, joints, CLUTTER_APPROACH_RATE_RAD_S);
    // Keep the gripper closing for the whole approach: the grasp weld triggers on
    // the first finger contact, so the swing welds the object wherever a finger
    // bar first brushes it. That offset is not precisely controllable (this
    // gripper's finger bars extend sideways and the forearm leads any azimuthal
    // sweep), so the clutter place target is derived from where the scripted
    // policy actually lands the cube — the same convention the lift place target
    // uses (see `MobileManipulatorEpisodeConfig::lift_pick_place`).
    action.gripper_velocity_rad_s = -2.5;
    action
}

/// Carries the grasped cube toward the fixed-base clutter place target by driving
/// the joints to the IK solution at maximal reach toward it (the ground target
/// itself lies just outside the horizontal workspace; the cube is released at the
/// closest reachable point above it and falls within the place tolerance).
/// Replaces the old fixed shoulder/elbow velocity pair, which was tuned against
/// the unstable pre-fix arm dynamics and has no meaning against the stable
/// position-held servo.
fn clutter_carry_action(
    observation: &MobileManipulatorObservation,
    kin: &MmMinimalKinematics,
) -> crate::MobileManipulatorAction {
    let place = crate::mm_minimal_kinematics::mm_minimal_clutter_place_target();
    let carry_target = kin.max_reach_toward(place.x_m, place.z_m);
    // Elbow-down branch, matching the approach: keeps the carry a short in-branch
    // extension instead of a full arm reconfiguration through the +z clutter.
    let mut action = match kin.inverse_kinematics_elbow_down(carry_target) {
        Ok(joints) => joint_rate_toward_minimal(observation, joints, CLUTTER_APPROACH_RATE_RAD_S),
        Err(_) => crate::MobileManipulatorAction::default(),
    };
    action.gripper_velocity_rad_s = CLUTTER_CARRY_GRIPPER_RAD_S;
    action
}

/// Horizontal base distance to the mobile clutter place target.
fn mobile_place_base_distance_m(observation: &MobileManipulatorObservation) -> f64 {
    let dx = crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_X_M - observation.base_x_m;
    let dz = crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_Z_M - observation.base_z_m;
    dx.hypot(dz)
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

/// Heading-based diff-drive step toward a world XZ point: rotates in place until the
/// heading error is small, then drives forward with speed proportional to distance
/// (capped at `max_forward_m_s`). The arm and gripper channels are left at zero for
/// the caller to fill in.
fn mobile_drive_toward_action(
    observation: &MobileManipulatorObservation,
    target_x_m: f64,
    target_z_m: f64,
    max_forward_m_s: f64,
) -> crate::MobileManipulatorAction {
    use crate::mm_mobile_twist_to_wheel_velocities;
    use crate::MobileManipulatorAction;

    let dx_world = target_x_m - observation.base_x_m;
    let dz_world = target_z_m - observation.base_z_m;
    let distance_m = dx_world.hypot(dz_world);
    let heading_to_target = dz_world.atan2(dx_world);
    // The base's forward axis is `Quat::from_rotation_y(yaw) * X = (cos yaw, -sin yaw)`
    // in the XZ plane, so its travel-direction angle in `atan2(z, x)` terms is `-yaw`;
    // this heading error is `heading_to_target - (-yaw) = heading_to_target + yaw`.
    // A positive commanded twist yaw rate increases the observed yaw (see the
    // `mobile_twist_positive_yaw_rate_increases_observed_yaw` sim test), i.e. it
    // decreases the travel-direction angle, so the commanded rate needs a negative
    // gain to drive the error toward zero.
    let heading_error = wrap_heading_rad(heading_to_target + observation.base_yaw_rad);
    let forward_m_s = if heading_error.abs() > 0.12 {
        0.0
    } else {
        (0.65 * distance_m).clamp(0.0, max_forward_m_s)
    };
    let yaw_rate_rad_s = (-2.0 * heading_error).clamp(-0.7, 0.7);
    let (left, right) = mm_mobile_twist_to_wheel_velocities(forward_m_s, yaw_rate_rad_s);
    MobileManipulatorAction {
        left_wheel_velocity_rad_s: left.clamp(-3.0, 3.0),
        right_wheel_velocity_rad_s: right.clamp(-3.0, 3.0),
        ..MobileManipulatorAction::default()
    }
}

/// Gain for `joint_rate_toward_minimal`'s proportional tracking (rad/s per rad of error).
const CLUTTER_APPROACH_GAIN: f64 = 0.5;

fn joint_rate_toward_minimal(
    observation: &MobileManipulatorObservation,
    target: MmMinimalJointTarget,
    max_rate_rad_s: f64,
) -> crate::MobileManipulatorAction {
    use crate::MobileManipulatorAction;
    MobileManipulatorAction {
        shoulder_velocity_rad_s: proportional_rate_toward(
            observation.shoulder_position_rad,
            target.shoulder_rad,
            max_rate_rad_s,
            CLUTTER_APPROACH_GAIN,
            0.05,
        ),
        elbow_velocity_rad_s: proportional_rate_toward(
            observation.elbow_position_rad,
            target.elbow_rad,
            max_rate_rad_s,
            CLUTTER_APPROACH_GAIN,
            0.05,
        ),
        ..MobileManipulatorAction::default()
    }
}

/// Proportional (not bang-bang) rate toward `target`: scales with the error and
/// saturates at `max_rate`, instead of always commanding the full rate right up
/// until snapping to zero inside `tolerance`. `joint_rate_toward_minimal` drives
/// the fixed-base clutter approach against the shoulder/elbow's spring-lagged
/// position-hold servo (see `configure_arm_position_motors`); bang-bang control
/// against a lagged plant is a textbook relay oscillator (a sustained limit cycle
/// around the target that never settles inside tolerance), so this ramps the
/// commanded rate down near the target instead of holding it at max until the
/// last instant.
fn proportional_rate_toward(
    current: f64,
    target: f64,
    max_rate: f64,
    gain: f64,
    tolerance: f64,
) -> f64 {
    let error = target - current;
    if error.abs() < tolerance {
        0.0
    } else {
        (error * gain).clamp(-max_rate, max_rate)
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
        assert_eq!(policy.total_steps(), 1490);
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
