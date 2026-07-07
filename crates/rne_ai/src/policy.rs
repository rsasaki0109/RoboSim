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
/// Gripper-mount-to-object horizontal distance below which a clutter approach
/// starts closing the fingers (m). The grasp weld requires BOTH fingers in contact
/// simultaneously (see `MobileManipulatorSim::find_graspable_in_contact`), so the
/// old "close for the whole approach" tuning no longer works: a partially-closed
/// leading finger bar bulldozes the cube out of the finger pocket (or clean off
/// the table) before the trailing finger can reach its far side. Keeping the
/// fingers open until the cube sits between them and only then closing gives the
/// two-sided pinch the weld gate needs. A bit wider than the finger pocket
/// (0.08 m across) so the ~1 rad of closing travel completes at about the time
/// the mount settles onto the cube.
const CLUTTER_CLOSE_GRIPPER_DISTANCE_M: f64 = 0.12;
/// Inbound radial offset of the stage-3 IK target short of the pick object (m).
/// Stage 3 used to aim the gripper mount at the object center; with the
/// position-held arm's spring lag the mount routinely overshoots past the cube
/// along the inbound radial, so only the trailing finger bar contacts (never
/// both — the two-finger weld gate never fires). Stopping short by roughly the
/// finger-pad reach (0.04 m offset + 0.035 m cube half-width) keeps the open
/// pocket spanning the cube when the close command starts.
const CLUTTER_STAGE3_MOUNT_LEAD_M: f64 = 0.07;
/// Parallel-gripper opening metric (`0.5 * (left - right)`) at rest; once a
/// clutter approach has begun closing (metric below this), keep issuing close
/// even if contact nudges the mount-to-object distance back above
/// [`CLUTTER_CLOSE_GRIPPER_DISTANCE_M`] — otherwise the trailing finger never
/// reaches the far side of the cube.
const CLUTTER_GRIPPER_OPEN_REST_RAD: f64 = -0.02;
/// Radial standoff of the fixed-base clutter approach's intermediate waypoint,
/// short of the pick object along its bearing from the shoulder (m). The arm
/// first swings to this waypoint, then extends radially onto the object so the
/// open finger pocket (bars flanking the mount tangentially) leads the final
/// approach. Approaching azimuthally instead sweeps a finger bar tip — up to
/// 0.12 m ahead of the mount along the sweep — through the object first, and
/// with the two-finger weld gate that leading bar just shoves the object away
/// (or off the table) before the pocket can ever bracket it. Long enough to
/// clear the bar length (0.12 m) plus the object's width with margin.
const CLUTTER_APPROACH_STANDOFF_M: f64 = 0.22;
/// Tangential (cross-bearing) mount-to-object misalignment below which the
/// fixed-base clutter approach switches from the standoff waypoint to the radial
/// final approach (m). Must be small relative to the finger pocket's clearance
/// around a clutter cube (0.08 m pocket vs 0.07 m cube), so the cube actually
/// enters the pocket instead of being rammed radially by a misaligned bar tip.
const CLUTTER_RADIAL_ALIGN_TOLERANCE_M: f64 = 0.02;
/// Slack on the mount's radial retraction before the fixed-base clutter approach
/// starts its short-radius swing (m); loose on purpose — the swing only needs the
/// finger structure pulled well inside the clutter cubes' radial band, not an
/// exact radius hold.
const CLUTTER_RETRACT_RADIUS_TOLERANCE_M: f64 = 0.08;
/// Object radial Z above which a pick sits on the table's +z rim (`clutter_cube_c`
/// at z ≈ 0.31 vs table half-extent 0.35 m). Uses [`clutter_high_z_approach_action`]
/// instead of the generic three-stage swing so inbound motion does not shove the
/// cube past the edge. Kept above `clutter_cube_a`'s nominal z (0.20 m) plus the
/// transient +z drift the generic approach induces (~0.29 m) so mid-approach lane
/// switches do not derail the nearer cube.
const CLUTTER_HIGH_Z_PICK_RADIAL_DZ_M: f64 = 0.30;
/// West offset for the high +z standoff waypoint (m). Positions the mount on the
/// -x side of the object so inbound motion does not sweep finger bars through it.
const CLUTTER_HIGH_Z_STANDOFF_WEST_M: f64 = 0.08;
/// Fraction of [`CLUTTER_HIGH_Z_STANDOFF_WEST_M`] applied when forming the
/// southwest column X for the south-corridor waypoint (0.75 × 0.08 m).
const CLUTTER_HIGH_Z_STANDOFF_SCALE: f64 = 0.75;
/// Stage-1 south dipping waypoint Z for high +z picks (m). Below the parked arm
/// and the +z clutter cubes so the retract arc does not sweep through them.
const CLUTTER_HIGH_Z_SOUTH_DIP_Z_M: f64 = -0.12;
/// Mount X must be at or west of the southwest column (`sw_x` + this slack) before
/// the high +z approach commits to the inbound final arc (m). One-sided gate so
/// the final-stage servo cannot re-trigger the south-corridor dip.
const CLUTTER_HIGH_Z_SOUTH_CORRIDOR_X_TOLERANCE_M: f64 = 0.18;
/// Joint rate scale for the high +z south-corridor stage (×
/// [`CLUTTER_APPROACH_RATE_RAD_S`]). Slightly faster than the final stage so the
/// dip clears the +z lane with steps to spare for the inbound close.
const CLUTTER_HIGH_Z_CORRIDOR_RATE_SCALE: f64 = 1.35;
/// Inbound Z offset subtracted from the radial high +z final IK target (m). Zero
/// keeps the target on the object bearing so the elbow-down branch can lift the
/// mount into the finger pocket without stalling short.
const CLUTTER_HIGH_Z_FINAL_LEAD_Z_M: f64 = 0.0;
/// Gripper-mount-to-object distance below which the high +z final stage starts
/// closing (m). Matches [`CLUTTER_CLOSE_GRIPPER_DISTANCE_M`] plus margin for the
/// spring-lagged mount settling ~0.20 m short of the cube center on this path.
const CLUTTER_HIGH_Z_CLOSE_GRIPPER_DISTANCE_M: f64 = 0.20;
const CLUTTER_SETTLE_STEPS: u64 = 20;
/// Approach phase length for fixed-base clutter policies and tests. Long enough for
/// `clutter_cube_b` on the -z lane (object-bearing stage-1 retract plus two-finger
/// close) to finish within the phase; the center cube typically grasps earlier.
const CLUTTER_APPROACH_STEPS: u64 = 420;
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
            // Close only once the object is inside the finger pocket (see
            // `CLUTTER_CLOSE_GRIPPER_DISTANCE_M`): the two-finger weld gate needs
            // a two-sided pinch, and driving in with partially-closed fingers just
            // plows the cube ahead of the leading finger bar.
            action.gripper_velocity_rad_s = mobile_clutter_gripper_velocity_rad_s(observation);
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
            // Drive so the CARRIED OBJECT (not the base) converges on the place
            // target. The object rides on the spring-held arm nearly a meter ahead
            // of the base, and its lateral offset after the retreat's drag-and-turn
            // is contact-history dependent (it differs measurably across
            // platforms), so steering the base straight at the place point can
            // march the object past the release gate without ever crossing it.
            // `mobile_carry_object_toward_action` closed-loops the object error
            // instead; the release gate above fires when the object passes over
            // the target. Gripper velocity stays zero: the contact weld holds the
            // object and only an opening command releases it; continuing to
            // command "close" leaves the limitless fingers flapping against the
            // welded object.
            if grasped {
                mobile_carry_object_toward_action(observation)
            } else {
                mobile_drive_toward_action(
                    observation,
                    crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_X_M,
                    crate::mm_minimal_kinematics::MM_MOBILE_CLUTTER_PLACE_Z_M,
                    MOBILE_CLUTTER_CARRY_DRIVE_SPEED_M_S,
                )
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
    let (shoulder_x_m, shoulder_z_m) = kin.shoulder_xz_m();
    let radial_dz_m = object_z_m - shoulder_z_m;
    if radial_dz_m > CLUTTER_HIGH_Z_PICK_RADIAL_DZ_M {
        return clutter_high_z_approach_action(
            observation,
            kin,
            object_x_m,
            object_z_m,
            radial_dz_m,
        );
    }
    // Three-stage approach (stateless — each step re-derives its stage from the
    // observed mount and object poses): retract the mount to the standoff radius
    // at its CURRENT bearing, swing at that short radius onto the object's
    // bearing, then extend radially so the open finger pocket leads onto the
    // object (see `CLUTTER_APPROACH_STANDOFF_M` / the tolerances below for why an
    // azimuthal sweep at reach radius cannot capture against the two-finger weld
    // gate — and worse, the joint-space transient from the extended rest pose
    // straight to a folded IK pose sweeps the finger structure tangentially at
    // near-full radius, plowing through any cube near the path).
    let radial_dx_m = object_x_m - shoulder_x_m;
    let radial_dz_m = object_z_m - shoulder_z_m;
    let object_radius_m = radial_dx_m.hypot(radial_dz_m);
    let standoff_radius_m = (object_radius_m - CLUTTER_APPROACH_STANDOFF_M).max(0.2);
    // Mount position from the pre-grasp observation (object minus mount->object
    // delta); `gripper_target_d*` is zeroed post-grasp, but the approach only
    // runs pre-grasp.
    let mount_x_m = object_x_m - observation.gripper_target_dx_m;
    let mount_z_m = object_z_m - observation.gripper_target_dz_m;
    let mount_dx_m = mount_x_m - shoulder_x_m;
    let mount_dz_m = mount_z_m - shoulder_z_m;
    let mount_radius_m = mount_dx_m.hypot(mount_dz_m);
    let tangential_offset_m = if object_radius_m > 1.0e-9 {
        // Cross-bearing component of the mount->object offset: how far the mount
        // sits off the object's radial line from the shoulder.
        ((mount_x_m - shoulder_x_m) * radial_dz_m - (mount_z_m - shoulder_z_m) * radial_dx_m).abs()
            / object_radius_m
    } else {
        0.0
    };
    let aligned = tangential_offset_m <= CLUTTER_RADIAL_ALIGN_TOLERANCE_M;
    let retracted = mount_radius_m <= standoff_radius_m + CLUTTER_RETRACT_RADIUS_TOLERANCE_M;
    let target = if aligned {
        // Stage 3: radial final approach with the mount short of the object so the
        // open finger pocket (bars flanking the mount in gripper Z) leads onto the
        // cube instead of the mount overshooting past it along the inbound radial.
        let mount_radius_m = (object_radius_m - CLUTTER_STAGE3_MOUNT_LEAD_M).max(standoff_radius_m);
        let scale = mount_radius_m / object_radius_m.max(1.0e-9);
        MmMinimalGripperTarget::new(
            shoulder_x_m + radial_dx_m * scale,
            kin.shoulder_y_m(),
            shoulder_z_m + radial_dz_m * scale,
        )
    } else if retracted {
        // Stage 2: swing at the short standoff radius onto the object's bearing.
        let scale = standoff_radius_m / object_radius_m.max(1.0e-9);
        MmMinimalGripperTarget::new(
            shoulder_x_m + radial_dx_m * scale,
            kin.shoulder_y_m(),
            shoulder_z_m + radial_dz_m * scale,
        )
    } else {
        // Stage 1: retract to a fixed waypoint at the standoff radius. On the +z
        // clutter lane the arm's +x rest bearing keeps the retract servo stable
        // (retracting "along the mount's own bearing" chases the servo's own
        // transient bulge and never converges). On the -z lane (`clutter_cube_b`)
        // that +x waypoint sits above the object and the spring-lagged servo
        // bulges even further +z, so the mount never retracts enough to enter the
        // swing stage — use the object's bearing for the standoff waypoint there.
        let scale = standoff_radius_m / object_radius_m.max(1.0e-9);
        if radial_dz_m < 0.0 {
            MmMinimalGripperTarget::new(
                shoulder_x_m + radial_dx_m * scale,
                kin.shoulder_y_m(),
                shoulder_z_m + radial_dz_m * scale,
            )
        } else {
            MmMinimalGripperTarget::new(
                shoulder_x_m + standoff_radius_m,
                kin.shoulder_y_m(),
                shoulder_z_m,
            )
        }
    };
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
            kin.inverse_kinematics_elbow_down(kin.max_reach_toward(target.x_m, target.z_m))
                .expect("max_reach_toward scales inside the analytic workspace")
        });
    let mut action = joint_rate_toward_minimal(observation, joints, CLUTTER_APPROACH_RATE_RAD_S);
    // Close only once the object sits inside the finger pocket (see
    // `clutter_approach_gripper_velocity_rad_s`): the two-finger weld gate needs a
    // two-sided pinch, and closing during the sweep just bulldozes the cube with
    // the leading finger bar.
    action.gripper_velocity_rad_s = clutter_approach_gripper_velocity_rad_s(observation);
    action
}

/// Two-stage south-corridor approach for table-edge +z clutter picks.
///
/// The generic three-stage radial swing bulldozes such cubes past the table's +z
/// rim before the trailing finger can reach them. This path first dips the mount
/// south along the southwest standoff column (clearing the +z clutter lane), then
/// closes radially inward from below with the open finger pocket leading.
fn clutter_high_z_approach_action(
    observation: &MobileManipulatorObservation,
    kin: &MmMinimalKinematics,
    object_x_m: f64,
    object_z_m: f64,
    _radial_dz_m: f64,
) -> crate::MobileManipulatorAction {
    let (shoulder_x_m, shoulder_z_m) = kin.shoulder_xz_m();
    let radial_dx_m = object_x_m - shoulder_x_m;
    let radial_dz_m = object_z_m - shoulder_z_m;
    let object_radius_m = radial_dx_m.hypot(radial_dz_m);
    let mount_x_m = object_x_m - observation.gripper_target_dx_m;
    let gripper_distance_m = observation
        .gripper_target_dx_m
        .hypot(observation.gripper_target_dz_m);
    let sw_x_m = object_x_m - CLUTTER_HIGH_Z_STANDOFF_WEST_M * CLUTTER_HIGH_Z_STANDOFF_SCALE;
    let in_final = mount_x_m <= sw_x_m + CLUTTER_HIGH_Z_SOUTH_CORRIDOR_X_TOLERANCE_M;
    let target = if in_final {
        let mount_radius_m = (object_radius_m - CLUTTER_STAGE3_MOUNT_LEAD_M)
            .max(standoff_radius_for_high_z(object_radius_m));
        let scale = mount_radius_m / object_radius_m.max(1.0e-9);
        MmMinimalGripperTarget::new(
            shoulder_x_m + radial_dx_m * scale,
            kin.shoulder_y_m(),
            shoulder_z_m + radial_dz_m * scale - CLUTTER_HIGH_Z_FINAL_LEAD_Z_M,
        )
    } else {
        MmMinimalGripperTarget::new(sw_x_m, kin.shoulder_y_m(), CLUTTER_HIGH_Z_SOUTH_DIP_Z_M)
    };
    let joints = kin
        .inverse_kinematics_elbow_down(target)
        .unwrap_or_else(|_| {
            kin.inverse_kinematics_elbow_down(kin.max_reach_toward(target.x_m, target.z_m))
                .expect("max_reach_toward scales inside the analytic workspace")
        });
    let approach_rate_rad_s = if in_final && gripper_distance_m < 0.18 {
        CLUTTER_APPROACH_RATE_RAD_S * 0.5
    } else if in_final {
        CLUTTER_APPROACH_RATE_RAD_S * 1.25
    } else {
        CLUTTER_APPROACH_RATE_RAD_S * CLUTTER_HIGH_Z_CORRIDOR_RATE_SCALE
    };
    let mut action = joint_rate_toward_minimal(observation, joints, approach_rate_rad_s);
    action.gripper_velocity_rad_s = clutter_high_z_gripper_velocity_rad_s(observation, in_final);
    action
}

fn standoff_radius_for_high_z(object_radius_m: f64) -> f64 {
    (object_radius_m - CLUTTER_APPROACH_STANDOFF_M).max(0.2)
}

/// Gripper close command for high +z clutter approaches.
///
/// Fingers stay open through the south-corridor stage. During the final radial
/// stage, close once inside [`CLUTTER_HIGH_Z_CLOSE_GRIPPER_DISTANCE_M`], then
/// keep closing while still within 1.2× that distance so contact transients do
/// not reopen the gripper before the trailing finger reaches the far side.
fn clutter_high_z_gripper_velocity_rad_s(
    observation: &MobileManipulatorObservation,
    in_final_approach: bool,
) -> f64 {
    if !in_final_approach {
        return 0.0;
    }
    let gripper_distance_m = observation
        .gripper_target_dx_m
        .hypot(observation.gripper_target_dz_m);
    let started_closing = observation.gripper_position_rad < CLUTTER_GRIPPER_OPEN_REST_RAD;
    if gripper_distance_m < CLUTTER_HIGH_Z_CLOSE_GRIPPER_DISTANCE_M
        || (started_closing && gripper_distance_m < CLUTTER_HIGH_Z_CLOSE_GRIPPER_DISTANCE_M * 1.2)
    {
        -2.5
    } else {
        0.0
    }
}

/// Gripper close command for fixed-base clutter approaches.
fn clutter_approach_gripper_velocity_rad_s(observation: &MobileManipulatorObservation) -> f64 {
    let gripper_distance_m = observation
        .gripper_target_dx_m
        .hypot(observation.gripper_target_dz_m);
    let started_closing = observation.gripper_position_rad < CLUTTER_GRIPPER_OPEN_REST_RAD;
    if gripper_distance_m < CLUTTER_CLOSE_GRIPPER_DISTANCE_M || started_closing {
        -2.5
    } else {
        0.0
    }
}

/// Gripper close command for mobile clutter drive-in (distance gate only).
fn mobile_clutter_gripper_velocity_rad_s(observation: &MobileManipulatorObservation) -> f64 {
    let gripper_distance_m = observation
        .gripper_target_dx_m
        .hypot(observation.gripper_target_dz_m);
    if gripper_distance_m < CLUTTER_CLOSE_GRIPPER_DISTANCE_M {
        -2.5
    } else {
        0.0
    }
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

/// Nominal forward distance of the carried clutter object from the mobile base
/// center (m); scales the steering share of the object-error feedback below.
///
/// Re-derived (was `0.95`) after `canonical_grasp_anchor` in
/// `env::mobile_manipulator::sim` gained a forward standoff (see
/// `GRASP_FORWARD_STANDOFF_MARGIN_M`): the grasped cube now rides further ahead of
/// the gripper mount, which increases this lever arm by roughly the clutter cube's
/// half-width (0.035 m) plus that margin. Measured directly from the simulation
/// (logging the welded cube's world position against the base's during a carry
/// rollout) as ~1.00 m when the base isn't actively turning; turning induces a
/// transient lag (the weld is a spring/constraint, not an infinitely rigid rod, so
/// the cube swings wide under yaw) that made the previous, now-more-mismatched
/// constant feed an incorrect lever arm into the `w = GAIN * lateral / L` term
/// below, amplifying the lateral-channel loop gain enough to leave the
/// carry-drive's fixed step budget without a converged margin: it visibly
/// oscillated back and forth past centerline before barely settling by the last
/// step on one platform and not quite settling on another (float-rounding and
/// Rapier solver iteration order differ enough between the Windows and Linux CI
/// runners to tip a borderline convergence either way).
const MOBILE_CLUTTER_CARRY_OBJECT_LEAD_M: f64 = 1.00;
/// Proportional gain mapping carried-object position error to base twist (1/s).
const MOBILE_CLUTTER_CARRY_OBJECT_GAIN: f64 = 0.6;

/// Diff-drive step that moves the CARRIED OBJECT toward the place target.
///
/// Treats the object as a lookahead point rigidly ~`MOBILE_CLUTTER_CARRY_OBJECT_LEAD_M`
/// ahead of the base and feedback-linearizes the unicycle: for a point `P = B + L f(yaw)`,
/// `dP = v f + L w df/dyaw`, so commanding `v` from the error's heading-aligned
/// component and `w` from its lateral component moves `P` straight at the target.
/// Steering the base's own heading at a compensated aim point does not work here:
/// the aim rotates with the base (the object error vector is expressed through the
/// base yaw), so a bearing controller chases a moving target and dithers in place.
/// Requires a grasped object (`target_d*` = place target relative to the object).
fn mobile_carry_object_toward_action(
    observation: &MobileManipulatorObservation,
) -> crate::MobileManipulatorAction {
    use crate::mm_mobile_twist_to_wheel_velocities;
    use crate::MobileManipulatorAction;

    let error_x = observation.target_dx_m;
    let error_z = observation.target_dz_m;
    let yaw = observation.base_yaw_rad;
    // Base forward axis in the XZ plane is `Quat::from_rotation_y(yaw) * X`
    // (see `apply_mobile_base_planar_drive`), and its yaw-derivative is `g`.
    let forward = (yaw.cos(), -yaw.sin());
    let forward_yaw_derivative = (-yaw.sin(), -yaw.cos());
    let along_m = error_x * forward.0 + error_z * forward.1;
    let lateral_m = error_x * forward_yaw_derivative.0 + error_z * forward_yaw_derivative.1;

    let forward_m_s = (MOBILE_CLUTTER_CARRY_OBJECT_GAIN * along_m).clamp(
        -MOBILE_CLUTTER_CARRY_DRIVE_SPEED_M_S,
        MOBILE_CLUTTER_CARRY_DRIVE_SPEED_M_S,
    );
    let yaw_rate_rad_s = (MOBILE_CLUTTER_CARRY_OBJECT_GAIN * lateral_m
        / MOBILE_CLUTTER_CARRY_OBJECT_LEAD_M)
        .clamp(-0.7, 0.7);
    let (left, right) = mm_mobile_twist_to_wheel_velocities(forward_m_s, yaw_rate_rad_s);
    MobileManipulatorAction {
        left_wheel_velocity_rad_s: left.clamp(-3.0, 3.0),
        right_wheel_velocity_rad_s: right.clamp(-3.0, 3.0),
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
        assert_eq!(policy.total_steps(), 1010);
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
