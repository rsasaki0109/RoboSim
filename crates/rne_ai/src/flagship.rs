//! Portable contract shared by the installed mobile-manipulation flagship.

use crate::{
    ActionSpec, ObservationSpec, RandomDistributionSpec, RandomizationParameterSpec,
    RandomizationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType,
    TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};

use crate::{
    mm_mobile_twist_to_wheel_velocities, mm_mobile_wheel_velocities_to_twist,
    IkMobileLiftPickPlacePolicy, MmLiftJointTarget, MmLiftKinematics, MobileLiftPickPlacePhase,
    MobileManipulatorAction, MobileManipulatorObservation,
};

/// Stable TaskSpec identity used by the installed mobile-lift flagship.
pub const FLAGSHIP_MOBILE_LIFT_TASK_ID: &str = "rne.flagship.mobile_lift_shared_aisle.v1";

/// Stable controller identity used by the installed mobile-lift flagship.
pub const FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID: &str = "rne.ai.ik_mobile_lift_pick_place_policy.v1";

/// Complete portable TaskSpec identity used by controller-driven execution paths.
pub const FLAGSHIP_MOBILE_LIFT_TASK_ID_V2: &str = "rne.flagship.mobile_lift_shared_aisle.v2";

/// Controller identity whose only runtime input is the v2 flattened TaskSpec observation.
pub const FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2: &str =
    "rne.ai.portable_ik_mobile_lift_pick_place_controller.v2";

/// Flattened width of the complete v2 flagship observation.
pub const FLAGSHIP_MOBILE_LIFT_OBSERVATION_WIDTH_V2: usize = 24;

/// Flattened width of the v2 flagship action.
pub const FLAGSHIP_MOBILE_LIFT_ACTION_WIDTH_V2: usize = 7;

/// Maximum planar speed emitted by the portable v2 controller.
pub const FLAGSHIP_MOBILE_LIFT_MAX_BASE_SPEED_M_S_V2: f64 = 0.1;

/// Maximum yaw speed emitted by the portable v2 controller.
pub const FLAGSHIP_MOBILE_LIFT_MAX_YAW_SPEED_RAD_S_V2: f64 = std::f64::consts::PI / 6.0;

/// Schema version for the portable built-in controller contract artifact.
pub const FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipMobileLiftControllerContract`].
pub const FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND: &str =
    "rne_flagship_mobile_lift_controller_contract";

const FLAGSHIP_V2_OBSERVATION_ORDER: [&str; 14] = [
    "base_position_m",
    "base_yaw_rad",
    "arm_joint_position_rad",
    "lift_position_m",
    "gripper_position_m",
    "payload_position_m",
    "place_target_position_m",
    "wrist_camera_pixel_count",
    "wrist_depth_min_m",
    "traffic_actor_position_m",
    "traffic_signal_green",
    "traffic_clear",
    "grasped",
    "policy_phase",
];

const FLAGSHIP_V2_ACTION_ORDER: [&str; 4] = [
    "wheel_velocity_rad_s",
    "arm_joint_target_rad",
    "lift_target_m",
    "gripper_velocity_m_s",
];

/// Integer nanosecond period used by the installed flagship controller.
pub const FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS: u64 = 16_666_667;

/// Maximum controller decisions in one flagship episode.
pub const FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS: u64 = 8_000;

/// Maximum controller decisions in one v2 flagship episode.
pub const FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS_V2: u64 = 10_000;

/// Domain-randomization identity for traffic departure delay.
pub const FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION: &str = "traffic_departure_delay_s";

/// Domain-randomization identity for traffic speed delta.
pub const FLAGSHIP_TRAFFIC_SPEED_DIMENSION: &str = "traffic_speed_delta_m_s";

/// Builds the exact portable TaskSpec used by installed flagship proofs.
///
/// `fixed_delta_ticks` is the integer number of nanoseconds in one controller
/// decision. Keeping that integer at the API boundary prevents a hardware,
/// simulator, or recorded-stream adapter from silently choosing a different
/// rate through floating-point rounding.
pub fn flagship_mobile_lift_task_spec(fixed_delta_ticks: u64) -> TaskSpec {
    TaskSpec::new(
        FLAGSHIP_MOBILE_LIFT_TASK_ID,
        fixed_delta_ticks as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![2], "m"),
            TensorSpec::new("arm_joint_position_rad", TensorDType::F64, vec![3], "rad"),
            TensorSpec::new("lift_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("gripper_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("payload_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("wrist_camera_pixel_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("wrist_depth_min_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(0.0, f64::MAX)),
            TensorSpec::new("traffic_actor_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("traffic_signal_green", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("traffic_clear", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("grasped", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("policy_phase", TensorDType::I32, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 9.0)),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("arm_joint_target_rad", TensorDType::F64, vec![3], "rad").with_bounds(
                TensorBounds::broadcast(-std::f64::consts::PI, std::f64::consts::PI),
            ),
            TensorSpec::new("lift_target_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(-0.5, 0.5)),
            TensorSpec::new("gripper_velocity_m_s", TensorDType::F64, vec![], "m/s")
                .with_bounds(TensorBounds::broadcast(-0.1, 0.1)),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("task_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("task_completed", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new(
                    "inspection_pick_place_completed",
                    TerminationKind::Success,
                ),
                TerminationConditionSpec::new("perception_stream_lost", TerminationKind::Failure),
            ],
            Some(FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS),
        ),
        ResetSpec::splitmix64(false),
    )
    .with_randomization(RandomizationSpec::new(vec![
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION,
            "s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_SPEED_DIMENSION,
            "m/s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
    ]))
}

/// Builds the complete portable v2 TaskSpec consumed directly by the v2 controller.
///
/// Unlike v1, this contract includes full base pose and the place target. Those
/// fields are dynamic controller inputs and cannot be reconstructed honestly
/// from the v1 flattened observation.
pub fn flagship_mobile_lift_task_spec_v2(fixed_delta_ticks: u64) -> TaskSpec {
    TaskSpec::new(
        FLAGSHIP_MOBILE_LIFT_TASK_ID_V2,
        fixed_delta_ticks as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("arm_joint_position_rad", TensorDType::F64, vec![3], "rad"),
            TensorSpec::new("lift_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("gripper_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("payload_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("place_target_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("wrist_camera_pixel_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("wrist_depth_min_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(0.0, f64::MAX)),
            TensorSpec::new("traffic_actor_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("traffic_signal_green", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("traffic_clear", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("grasped", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("policy_phase", TensorDType::I32, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 9.0)),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("arm_joint_target_rad", TensorDType::F64, vec![3], "rad").with_bounds(
                TensorBounds::broadcast(-std::f64::consts::PI, std::f64::consts::PI),
            ),
            TensorSpec::new("lift_target_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(-0.5, 0.5)),
            TensorSpec::new("gripper_velocity_m_s", TensorDType::F64, vec![], "m/s")
                .with_bounds(TensorBounds::broadcast(-0.1, 0.1)),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("task_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("task_completed", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new(
                    "inspection_pick_place_completed",
                    TerminationKind::Success,
                ),
                TerminationConditionSpec::new("perception_stream_lost", TerminationKind::Failure),
            ],
            Some(FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS_V2),
        ),
        ResetSpec::splitmix64(false),
    )
    .with_randomization(RandomizationSpec::new(vec![
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION,
            "s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_SPEED_DIMENSION,
            "m/s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
    ]))
}

/// Numeric model geometry bound into the portable controller artifact.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipMobileLiftKinematicsContract {
    /// Nominal base height for the shipped mobile-lift model.
    pub nominal_base_y_m: f64,
    /// Lift anchor X offset on the base.
    pub anchor_x_m: f64,
    /// Lift anchor Y offset on the base.
    pub anchor_y_m: f64,
    /// Shoulder pivot X offset from the carriage.
    pub shoulder_offset_x_m: f64,
    /// Upper-arm length.
    pub upper_arm_m: f64,
    /// Forearm length to the gripper base.
    pub forearm_m: f64,
    /// Minimum lift target.
    pub lift_min_m: f64,
    /// Maximum lift target.
    pub lift_max_m: f64,
}

impl From<MmLiftKinematics> for FlagshipMobileLiftKinematicsContract {
    fn from(value: MmLiftKinematics) -> Self {
        Self {
            nominal_base_y_m: value.base_y_m,
            anchor_x_m: value.anchor_x_m,
            anchor_y_m: value.anchor_y_m,
            shoulder_offset_x_m: value.shoulder_offset_x_m,
            upper_arm_m: value.upper_arm_m,
            forearm_m: value.forearm_m,
            lift_min_m: value.lift_min_m,
            lift_max_m: value.lift_max_m,
        }
    }
}

/// Complete serializable configuration of the built-in v2 controller.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipMobileLiftControllerContract {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Contract artifact schema version.
    pub schema_version: u32,
    /// Exact controller implementation and configuration identity.
    pub controller_id: String,
    /// Exact TaskSpec identity accepted by the controller.
    pub task_id: String,
    /// Tensor names in exact TaskSpec observation order.
    pub observation_order: Vec<String>,
    /// Tensor names in exact TaskSpec action order.
    pub action_order: Vec<String>,
    /// Proportional twist scaling begins above this linear-speed limit.
    pub max_base_speed_m_s: f64,
    /// Proportional twist scaling begins above this yaw-speed limit.
    pub max_yaw_speed_rad_s: f64,
    /// Exact kinematic parameters used to derive geometric features.
    pub kinematics: FlagshipMobileLiftKinematicsContract,
}

impl FlagshipMobileLiftControllerContract {
    /// Returns the exact built-in v2 contract.
    pub fn built_in() -> Self {
        Self {
            kind: FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND.to_string(),
            schema_version: FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION,
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            observation_order: FLAGSHIP_V2_OBSERVATION_ORDER.map(str::to_string).to_vec(),
            action_order: FLAGSHIP_V2_ACTION_ORDER.map(str::to_string).to_vec(),
            max_base_speed_m_s: FLAGSHIP_MOBILE_LIFT_MAX_BASE_SPEED_M_S_V2,
            max_yaw_speed_rad_s: FLAGSHIP_MOBILE_LIFT_MAX_YAW_SPEED_RAD_S_V2,
            kinematics: MmLiftKinematics::mm_mobile_lift().into(),
        }
    }

    /// Rejects identity, tensor order, limit, or model-geometry drift.
    pub fn validate(&self) -> Result<(), FlagshipMobileLiftControllerError> {
        if self != &Self::built_in() {
            return Err(FlagshipMobileLiftControllerError::ControllerContractDrift);
        }
        Ok(())
    }
}

/// Stateful built-in controller whose runtime boundary is exactly the v2 TaskSpec.
#[derive(Clone, Debug, PartialEq)]
pub struct FlagshipMobileLiftControllerV2 {
    policy: IkMobileLiftPickPlacePolicy,
}

impl Default for FlagshipMobileLiftControllerV2 {
    fn default() -> Self {
        Self::new()
    }
}

impl FlagshipMobileLiftControllerV2 {
    /// Creates a controller at the first settle decision.
    pub fn new() -> Self {
        Self {
            policy: IkMobileLiftPickPlacePolicy::new(),
        }
    }

    /// Returns the complete serializable contract for this built-in controller.
    pub fn contract() -> FlagshipMobileLiftControllerContract {
        FlagshipMobileLiftControllerContract::built_in()
    }

    /// Returns the phase index required in the next observation.
    pub fn expected_policy_phase(&self) -> i32 {
        mobile_lift_phase_index(self.policy.phase())
    }

    /// Validates one v2 flattened observation and emits one v2 flattened action.
    ///
    /// Missing perception and phase disagreement fail without advancing policy
    /// state. A red or uncleared traffic source returns an exact hold action and
    /// also leaves policy state unchanged.
    pub fn next_action(
        &mut self,
        values: &[f64],
    ) -> Result<Vec<f64>, FlagshipMobileLiftControllerError> {
        let observation = parse_flagship_v2_observation(values)?;
        let actual_phase = parse_i32(values[23], "policy_phase")?;
        let expected_phase = self.expected_policy_phase();
        if actual_phase != expected_phase {
            return Err(FlagshipMobileLiftControllerError::PolicyPhase {
                expected: expected_phase,
                actual: actual_phase,
            });
        }
        let traffic_signal_green = parse_bool(values[20], "traffic_signal_green")?;
        let traffic_clear = parse_bool(values[21], "traffic_clear")?;
        let permitted = traffic_signal_green && traffic_clear;
        let mut candidate_policy = self.policy.clone();
        let action = if permitted {
            candidate_policy.next_action(&observation)
        } else {
            MobileManipulatorAction::default()
        };
        let flattened = flatten_and_bound_flagship_v2_action(action, &observation)?;
        if permitted {
            self.policy = candidate_policy;
        }
        Ok(flattened)
    }
}

fn parse_flagship_v2_observation(
    values: &[f64],
) -> Result<MobileManipulatorObservation, FlagshipMobileLiftControllerError> {
    if values.len() != FLAGSHIP_MOBILE_LIFT_OBSERVATION_WIDTH_V2 {
        return Err(FlagshipMobileLiftControllerError::ObservationWidth {
            expected: FLAGSHIP_MOBILE_LIFT_OBSERVATION_WIDTH_V2,
            actual: values.len(),
        });
    }
    if let Some((index, _)) = values
        .iter()
        .enumerate()
        .find(|(_, value)| !value.is_finite())
    {
        return Err(FlagshipMobileLiftControllerError::NonFiniteObservation { index });
    }
    let camera_pixels = parse_usize(values[15], "wrist_camera_pixel_count")?;
    if camera_pixels == 0 || values[16] <= 0.0 {
        return Err(FlagshipMobileLiftControllerError::MissingPerception);
    }
    let grasped = parse_bool(values[22], "grasped")?;
    let joints = MmLiftJointTarget {
        lift_m: values[7],
        shoulder_rad: values[4],
        elbow_rad: values[5],
    };
    let ee = MmLiftKinematics::mm_mobile_lift()
        .forward_kinematics_at_base(values[0], values[1], values[2], values[3], joints);
    Ok(MobileManipulatorObservation {
        base_x_m: values[0],
        base_y_m: values[1],
        base_z_m: values[2],
        base_yaw_rad: values[3],
        ee_x_m: ee.x_m,
        ee_y_m: ee.y_m,
        ee_z_m: ee.z_m,
        shoulder_position_rad: values[4],
        elbow_position_rad: values[5],
        wrist_yaw_position_rad: values[6],
        gripper_position_m: values[8],
        lift_position_m: values[7],
        is_grasping: grasped,
        wrist_camera_pixels: camera_pixels,
        joint_state_count: 4,
        target_dx_m: values[12] - values[9],
        target_dy_m: values[13] - values[10],
        target_dz_m: values[14] - values[11],
        wrist_depth_center_m: values[16],
        wrist_depth_min_m: values[16],
        pick_object_x_m: values[9],
        pick_object_y_m: values[10],
        pick_object_z_m: values[11],
        gripper_target_dx_m: values[9] - ee.x_m,
        gripper_target_dy_m: values[10] - ee.y_m,
        gripper_target_dz_m: values[11] - ee.z_m,
        ..MobileManipulatorObservation::default()
    })
}

fn flatten_and_bound_flagship_v2_action(
    action: MobileManipulatorAction,
    observation: &MobileManipulatorObservation,
) -> Result<Vec<f64>, FlagshipMobileLiftControllerError> {
    let (linear_m_s, yaw_rad_s) = mm_mobile_wheel_velocities_to_twist(
        action.left_wheel_velocity_rad_s,
        action.right_wheel_velocity_rad_s,
    );
    let scale = (linear_m_s.abs() / FLAGSHIP_MOBILE_LIFT_MAX_BASE_SPEED_M_S_V2)
        .max(yaw_rad_s.abs() / FLAGSHIP_MOBILE_LIFT_MAX_YAW_SPEED_RAD_S_V2)
        .max(1.0);
    let (left, right) = mm_mobile_twist_to_wheel_velocities(linear_m_s / scale, yaw_rad_s / scale);
    let target = action.lift_joint_target;
    let values = vec![
        left,
        right,
        target
            .map(|value| value.shoulder_rad)
            .unwrap_or(observation.shoulder_position_rad),
        target
            .map(|value| value.elbow_rad)
            .unwrap_or(observation.elbow_position_rad),
        action
            .wrist_yaw_target_rad
            .unwrap_or(observation.wrist_yaw_position_rad),
        target
            .map(|value| value.lift_m)
            .unwrap_or(observation.lift_position_m),
        action.gripper_velocity_m_s,
    ];
    let bounds = [
        (-10.0, 10.0),
        (-10.0, 10.0),
        (-std::f64::consts::PI, std::f64::consts::PI),
        (-std::f64::consts::PI, std::f64::consts::PI),
        (-std::f64::consts::PI, std::f64::consts::PI),
        (-0.5, 0.5),
        (-0.1, 0.1),
    ];
    for (index, (value, (minimum, maximum))) in values.iter().zip(bounds).enumerate() {
        if !value.is_finite() || !(minimum..=maximum).contains(value) {
            return Err(FlagshipMobileLiftControllerError::ActionValue {
                index,
                value: *value,
            });
        }
    }
    Ok(values)
}

fn parse_bool(value: f64, field: &'static str) -> Result<bool, FlagshipMobileLiftControllerError> {
    match value {
        0.0 => Ok(false),
        1.0 => Ok(true),
        _ => Err(FlagshipMobileLiftControllerError::InvalidDiscreteValue { field, value }),
    }
}

fn parse_i32(value: f64, field: &'static str) -> Result<i32, FlagshipMobileLiftControllerError> {
    if value.fract() != 0.0 || value < i32::MIN as f64 || value > i32::MAX as f64 {
        return Err(FlagshipMobileLiftControllerError::InvalidDiscreteValue { field, value });
    }
    Ok(value as i32)
}

fn parse_usize(
    value: f64,
    field: &'static str,
) -> Result<usize, FlagshipMobileLiftControllerError> {
    if value.fract() != 0.0 || value < 0.0 || value > i64::MAX as f64 {
        return Err(FlagshipMobileLiftControllerError::InvalidDiscreteValue { field, value });
    }
    usize::try_from(value as u64)
        .map_err(|_| FlagshipMobileLiftControllerError::InvalidDiscreteValue { field, value })
}

fn mobile_lift_phase_index(phase: MobileLiftPickPlacePhase) -> i32 {
    match phase {
        MobileLiftPickPlacePhase::Settle => 0,
        MobileLiftPickPlacePhase::Navigate => 1,
        MobileLiftPickPlacePhase::Approach => 2,
        MobileLiftPickPlacePhase::LowerToPick => 3,
        MobileLiftPickPlacePhase::Grasp => 4,
        MobileLiftPickPlacePhase::Lift => 5,
        MobileLiftPickPlacePhase::Transport => 6,
        MobileLiftPickPlacePhase::Lower => 7,
        MobileLiftPickPlacePhase::Release => 8,
        MobileLiftPickPlacePhase::Done => 9,
    }
}

/// Failure validating the portable v2 controller boundary.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FlagshipMobileLiftControllerError {
    /// A serialized built-in contract no longer matches the implementation.
    #[error("flagship v2 controller contract differs from the built-in implementation")]
    ControllerContractDrift,
    /// A controller output violated the exact TaskSpec bounds.
    #[error("flagship v2 action element {index} has invalid value {value}")]
    ActionValue {
        /// Flattened action index.
        index: usize,
        /// Rejected value.
        value: f64,
    },
    /// Flattened observation width does not match the TaskSpec.
    #[error("flagship v2 observation width must be {expected}, got {actual}")]
    ObservationWidth {
        /// Required width.
        expected: usize,
        /// Supplied width.
        actual: usize,
    },
    /// A continuous or discrete value was NaN or infinite.
    #[error("flagship v2 observation element {index} is not finite")]
    NonFiniteObservation {
        /// Flattened element index.
        index: usize,
    },
    /// An integer or boolean tensor was not represented exactly.
    #[error("flagship v2 {field} has invalid discrete value {value}")]
    InvalidDiscreteValue {
        /// Tensor name.
        field: &'static str,
        /// Rejected value.
        value: f64,
    },
    /// Camera or depth evidence required by the controller was absent.
    #[error("flagship v2 camera and positive depth observations are required")]
    MissingPerception,
    /// External task state disagreed with the controller's actual state.
    #[error("flagship v2 policy phase must be {expected}, got {actual}")]
    PolicyPhase {
        /// Controller phase before this decision.
        expected: i32,
        /// Supplied task-state phase.
        actual: i32,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v2_observation() -> Vec<f64> {
        vec![
            0.0, 0.25, 0.0, 0.0, // base pose
            0.0, 0.0, 0.0, // arm
            0.1, 0.02, // lift and gripper
            0.8, 0.45, 0.0, // payload
            1.5, 0.02, 0.0, // place target
            307_200.0, 0.4, // wrist RGB-D
            2.0, 0.0, 0.0, // traffic actor
            1.0, 1.0, 0.0, // signal, clear, grasped
            0.0, // policy phase
        ]
    }

    #[test]
    fn flagship_contract_is_valid_ordered_and_tick_exact() {
        let task = flagship_mobile_lift_task_spec(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS);
        task.validate().unwrap();

        assert_eq!(task.task_id, FLAGSHIP_MOBILE_LIFT_TASK_ID);
        assert_eq!(task.control_step_s, 0.016_666_667);
        assert_eq!(task.termination.max_episode_steps, Some(8_000));
        assert_eq!(task.observation.tensors.len(), 12);
        assert_eq!(task.action.tensors.len(), 4);
        assert_eq!(task.action.tensors[0].name, "wheel_velocity_rad_s");
        assert_eq!(task.action.tensors[1].name, "arm_joint_target_rad");
        assert_eq!(
            task.randomization.as_ref().map(|contract| {
                contract
                    .parameters
                    .iter()
                    .map(|parameter| parameter.name.as_str())
                    .collect::<Vec<_>>()
            }),
            Some(vec![
                FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION,
                FLAGSHIP_TRAFFIC_SPEED_DIMENSION,
            ])
        );
    }

    #[test]
    fn v2_contract_adds_dynamic_pose_and_place_target() {
        let task = flagship_mobile_lift_task_spec_v2(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS);
        task.validate().unwrap();
        assert_eq!(task.task_id, FLAGSHIP_MOBILE_LIFT_TASK_ID_V2);
        assert_eq!(task.termination.max_episode_steps, Some(10_000));
        assert_eq!(task.observation.tensors.len(), 14);
        let observation_width = task
            .observation
            .tensors
            .iter()
            .map(|tensor| tensor.shape.iter().product::<usize>().max(1))
            .sum::<usize>();
        let action_width = task
            .action
            .tensors
            .iter()
            .map(|tensor| tensor.shape.iter().product::<usize>().max(1))
            .sum::<usize>();
        assert_eq!(observation_width, FLAGSHIP_MOBILE_LIFT_OBSERVATION_WIDTH_V2);
        assert_eq!(action_width, FLAGSHIP_MOBILE_LIFT_ACTION_WIDTH_V2);
        assert_eq!(task.observation.tensors[0].shape, vec![3]);
        assert_eq!(task.observation.tensors[1].name, "base_yaw_rad");
        assert_eq!(task.observation.tensors[6].name, "place_target_position_m");

        let controller = FlagshipMobileLiftControllerV2::contract();
        controller.validate().unwrap();
        let bytes = serde_json::to_vec_pretty(&controller).unwrap();
        let round_tripped: FlagshipMobileLiftControllerContract =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(round_tripped, controller);
        let mut tampered = controller;
        tampered.max_base_speed_m_s += 0.01;
        assert_eq!(
            tampered.validate(),
            Err(FlagshipMobileLiftControllerError::ControllerContractDrift)
        );
    }

    #[test]
    fn v2_controller_is_taskspec_only_fail_closed_and_physically_bounded() {
        let mut controller = FlagshipMobileLiftControllerV2::new();
        let mut observation = v2_observation();

        let mut missing = observation.clone();
        missing[15] = 0.0;
        assert_eq!(
            controller.next_action(&missing),
            Err(FlagshipMobileLiftControllerError::MissingPerception)
        );
        assert_eq!(controller.expected_policy_phase(), 0);

        let mut wrong_phase = observation.clone();
        wrong_phase[23] = 1.0;
        assert_eq!(
            controller.next_action(&wrong_phase),
            Err(FlagshipMobileLiftControllerError::PolicyPhase {
                expected: 0,
                actual: 1,
            })
        );
        assert_eq!(controller.expected_policy_phase(), 0);

        for _ in 0..700 {
            observation[23] = f64::from(controller.expected_policy_phase());
            let action = controller.next_action(&observation).unwrap();
            assert_eq!(action.len(), FLAGSHIP_MOBILE_LIFT_ACTION_WIDTH_V2);
            assert!(action.iter().all(|value| value.is_finite()));
            let (linear_m_s, yaw_rad_s) = mm_mobile_wheel_velocities_to_twist(action[0], action[1]);
            assert!(linear_m_s.abs() <= FLAGSHIP_MOBILE_LIFT_MAX_BASE_SPEED_M_S_V2 + 1.0e-12);
            assert!(yaw_rad_s.abs() <= FLAGSHIP_MOBILE_LIFT_MAX_YAW_SPEED_RAD_S_V2 + 1.0e-12);
        }
        assert_eq!(controller.expected_policy_phase(), 1);
    }

    #[test]
    fn v2_controller_traffic_hold_does_not_advance() {
        let mut controller = FlagshipMobileLiftControllerV2::new();
        let mut observation = v2_observation();
        observation[20] = 0.0;
        let action = controller.next_action(&observation).unwrap();
        assert_eq!(action[0], 0.0);
        assert_eq!(action[1], 0.0);
        assert_eq!(controller.expected_policy_phase(), 0);

        observation[4] = 4.0;
        assert_eq!(
            controller.next_action(&observation),
            Err(FlagshipMobileLiftControllerError::ActionValue {
                index: 2,
                value: 4.0,
            })
        );
        assert_eq!(controller.expected_policy_phase(), 0);
    }
}
