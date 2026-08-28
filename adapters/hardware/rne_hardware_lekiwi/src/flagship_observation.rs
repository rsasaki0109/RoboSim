//! Fail-closed, parent-order observation fusion for the flagship LeKiwi path.
//!
//! LeKiwi cannot directly observe the complete flagship task. This module
//! therefore requires explicit, tick-stamped physical, localization,
//! perception, traffic, and task-state sources. Missing values are never
//! synthesized or zero-filled. Source freshness and sequence continuity are
//! checked without a wall clock before the exact 19-element parent observation
//! is emitted.

use crate::{flagship_rate::FLAGSHIP_CONTROLLER_PERIOD_TICKS, lekiwi_base_task_spec};
use rne_ai::{flagship_mobile_lift_task_spec, FLAGSHIP_MOBILE_LIFT_TASK_ID};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Schema version for flagship-to-LeKiwi observation-fusion evidence.
pub const FLAGSHIP_LEKIWI_OBSERVATION_FUSION_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipLeKiwiObservationFusion`].
pub const FLAGSHIP_LEKIWI_OBSERVATION_FUSION_KIND: &str = "rne_flagship_lekiwi_observation_fusion";

const LEKIWI_OBSERVATION_WIDTH: usize = 9;
const LEKIWI_ARM_WIDTH: usize = 5;
const FLAGSHIP_OBSERVATION_WIDTH: usize = 19;
const MAX_EXACT_F64_INTEGER: i64 = 1_i64 << 53;

/// One source sample bound to simulation ticks and a content-addressed contract.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipTimedObservation<T> {
    /// Stable source identity for this fuser session.
    pub source_id: String,
    /// Source-local monotonic sequence; repeated values may be held unchanged.
    pub source_sequence: u64,
    /// Integer nanosecond simulation tick at which the value was sampled.
    pub sample_tick: u64,
    /// Maximum permitted age at a controller decision, in integer ticks.
    pub max_age_ticks: u64,
    /// `sha256:` digest of the exact source configuration or calibration bytes.
    pub source_contract_sha256: String,
    /// Typed source payload.
    pub value: T,
}

/// TaskSpec-ordered normalized LeKiwi numeric observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiPhysicalObservation {
    /// Five arm positions, gripper percentage, planar x/y velocity, and yaw rate.
    pub values: Vec<f64>,
}

/// Metric base pose supplied by a calibrated localization source.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLocalizationObservation {
    /// Flagship base x/z position in metres.
    pub base_position_m: [f64; 2],
}

/// Metric payload and wrist-camera observation supplied by perception.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipPerceptionObservation {
    /// Payload x/y/z position in the flagship world frame, in metres.
    pub payload_position_m: [f64; 3],
    /// RGBA8 pixel count in the calibrated wrist-camera frame.
    pub wrist_camera_pixel_count: i64,
    /// Minimum finite calibrated wrist depth in metres.
    pub wrist_depth_min_m: f64,
    /// Whether the task's grasp detector currently reports a maintained grasp.
    pub grasped: bool,
}

/// Traffic state supplied by the same task-level traffic contract as simulation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipTrafficObservation {
    /// Traffic actor x/y/z position in the flagship world frame, in metres.
    pub actor_position_m: [f64; 3],
    /// Whether the bound traffic signal is green.
    pub signal_green: bool,
    /// Whether the task-level shared-aisle clearance contract is satisfied.
    pub clear: bool,
}

/// Non-LeKiwi task state required by the existing flagship controller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipTaskStateObservation {
    /// Lift displacement in metres in the flagship model convention.
    pub lift_position_m: f64,
    /// Gripper displacement in metres in the flagship model convention.
    pub gripper_position_m: f64,
    /// Stable flagship policy phase index in `0..=9`.
    pub policy_phase: i32,
}

/// Complete source set required for one parent-order observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiObservationInputs {
    /// Physical LeKiwi observation source.
    pub physical: FlagshipTimedObservation<FlagshipLeKiwiPhysicalObservation>,
    /// Metric localization source.
    pub localization: FlagshipTimedObservation<FlagshipLocalizationObservation>,
    /// Metric perception source.
    pub perception: FlagshipTimedObservation<FlagshipPerceptionObservation>,
    /// Traffic-contract source.
    pub traffic: FlagshipTimedObservation<FlagshipTrafficObservation>,
    /// Flagship task/controller state source.
    pub task_state: FlagshipTimedObservation<FlagshipTaskStateObservation>,
}

/// Affine projection from one LeKiwi arm-position element to one flagship joint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipArmChannelCalibration {
    /// Zero-based source element in LeKiwi `arm_joint_position_rad[5]`.
    pub physical_element: usize,
    /// Multiplicative `rad/rad` calibration scale.
    pub scale: f64,
    /// Additive flagship-model offset in radians.
    pub offset_rad: f64,
}

/// Explicit morphology calibration for the three flagship arm observations.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiArmCalibration {
    /// Stable calibration identity tied to the physical robot and model.
    pub calibration_id: String,
    /// Output-order channels: shoulder, elbow, and wrist yaw.
    pub channels: [FlagshipArmChannelCalibration; 3],
}

impl FlagshipLeKiwiArmCalibration {
    /// Validates identity, source indices, uniqueness, scale, and offset.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiObservationError> {
        validate_calibration(self)
    }

    /// Computes the deterministic digest embedded in fusion evidence.
    pub fn computed_sha256(&self) -> Result<String, FlagshipLeKiwiObservationError> {
        self.validate()?;
        Ok(arm_calibration_sha256(self))
    }
}

/// Source identity and freshness retained in fusion evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationSourceEvidence {
    /// Stable semantic source role.
    pub role: String,
    /// Stable source identity.
    pub source_id: String,
    /// Source-local sequence used for this decision.
    pub source_sequence: u64,
    /// Sample tick.
    pub sample_tick: u64,
    /// Computed age at the parent controller tick.
    pub age_ticks: u64,
    /// Maximum accepted age.
    pub max_age_ticks: u64,
    /// Exact source-contract digest label.
    pub source_contract_sha256: String,
}

/// One physical observation deliberately unused by the parent controller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UnusedLeKiwiObservation {
    /// LeKiwi TaskSpec tensor name.
    pub tensor_name: String,
    /// Row-major tensor element.
    pub tensor_element: usize,
    /// Declared unit.
    pub unit: String,
    /// Exact value not used by the parent observation.
    pub value: f64,
}

/// Evidence-bearing result of one complete parent-order observation fusion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiObservationFusion {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Observation-fusion schema version.
    pub schema_version: u32,
    /// Exact parent TaskSpec identity.
    pub parent_task_id: String,
    /// Exact zero-based parent controller sequence.
    pub parent_sequence: u64,
    /// Integer nanosecond simulation tick for this controller decision.
    pub parent_tick: u64,
    /// Digest of the exact arm-channel calibration values.
    pub arm_calibration_sha256: String,
    /// Ordered source and freshness evidence.
    pub sources: Vec<FlagshipObservationSourceEvidence>,
    /// Exact flattened observation in flagship TaskSpec tensor order.
    pub observation_values: Vec<f64>,
    /// SHA-256 of length-prefixed little-endian observation values.
    pub observation_sha256: String,
    /// Physical values deliberately denied influence over the parent controller.
    pub unused_physical_observations: Vec<UnusedLeKiwiObservation>,
    /// Stable success verdict for this fusion boundary only.
    pub status: String,
}

/// Stateful continuity checker and fuser for parent-order observations.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlagshipLeKiwiObservationFuser {
    expected_parent_sequence: u64,
    previous_inputs: Option<FlagshipLeKiwiObservationInputs>,
    arm_calibration_sha256: Option<String>,
}

impl FlagshipLeKiwiObservationFuser {
    /// Creates a fuser synchronized to parent sequence and tick zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the only parent sequence currently accepted by [`Self::fuse`].
    pub fn expected_parent_sequence(&self) -> u64 {
        self.expected_parent_sequence
    }

    /// Fuses one complete source set into the exact flagship observation order.
    ///
    /// Any error leaves the fuser state unchanged.
    pub fn fuse(
        &mut self,
        parent_sequence: u64,
        inputs: &FlagshipLeKiwiObservationInputs,
        arm_calibration: &FlagshipLeKiwiArmCalibration,
    ) -> Result<FlagshipLeKiwiObservationFusion, FlagshipLeKiwiObservationError> {
        if parent_sequence != self.expected_parent_sequence {
            return Err(FlagshipLeKiwiObservationError::UnexpectedParentSequence {
                expected: self.expected_parent_sequence,
                actual: parent_sequence,
            });
        }
        let next_parent_sequence = parent_sequence
            .checked_add(1)
            .ok_or(FlagshipLeKiwiObservationError::SequenceOverflow)?;
        let parent_tick = parent_sequence
            .checked_mul(FLAGSHIP_CONTROLLER_PERIOD_TICKS)
            .ok_or(FlagshipLeKiwiObservationError::TickOverflow)?;
        validate_calibration(arm_calibration)?;
        let calibration_sha256 = arm_calibration_sha256(arm_calibration);
        if self
            .arm_calibration_sha256
            .as_ref()
            .is_some_and(|previous| previous != &calibration_sha256)
        {
            return Err(FlagshipLeKiwiObservationError::ArmCalibrationChanged);
        }
        validate_source_set(parent_tick, inputs)?;
        validate_physical(&inputs.physical.value.values)?;
        validate_auxiliary(inputs)?;
        if let Some(previous) = &self.previous_inputs {
            validate_source_continuity(previous, inputs)?;
        }

        let physical = &inputs.physical.value.values;
        let arm = arm_calibration.channels.each_ref().map(|channel| {
            physical[channel.physical_element].mul_add(channel.scale, channel.offset_rad)
        });
        if let Some((element, _)) = arm.iter().enumerate().find(|(_, value)| !value.is_finite()) {
            return Err(FlagshipLeKiwiObservationError::NonFiniteFusedArm { element });
        }
        let localization = &inputs.localization.value;
        let perception = &inputs.perception.value;
        let traffic = &inputs.traffic.value;
        let task_state = &inputs.task_state.value;
        let observation_values = vec![
            localization.base_position_m[0],
            localization.base_position_m[1],
            arm[0],
            arm[1],
            arm[2],
            task_state.lift_position_m,
            task_state.gripper_position_m,
            perception.payload_position_m[0],
            perception.payload_position_m[1],
            perception.payload_position_m[2],
            perception.wrist_camera_pixel_count as f64,
            perception.wrist_depth_min_m,
            traffic.actor_position_m[0],
            traffic.actor_position_m[1],
            traffic.actor_position_m[2],
            bool_to_f64(traffic.signal_green),
            bool_to_f64(traffic.clear),
            bool_to_f64(perception.grasped),
            f64::from(task_state.policy_phase),
        ];
        validate_parent_observation(&observation_values)?;

        let mut used_arm_elements = BTreeSet::new();
        for channel in &arm_calibration.channels {
            used_arm_elements.insert(channel.physical_element);
        }
        let mut unused = (0..LEKIWI_ARM_WIDTH)
            .filter(|element| !used_arm_elements.contains(element))
            .map(|element| UnusedLeKiwiObservation {
                tensor_name: "arm_joint_position_rad".to_string(),
                tensor_element: element,
                unit: "rad".to_string(),
                value: physical[element],
            })
            .collect::<Vec<_>>();
        unused.extend([
            unused_value("gripper_position_pct", 0, "pct", physical[5]),
            unused_value("base_linear_velocity_m_s", 0, "m/s", physical[6]),
            unused_value("base_linear_velocity_m_s", 1, "m/s", physical[7]),
            unused_value("base_angular_velocity_rad_s", 0, "rad/s", physical[8]),
        ]);

        let fusion = FlagshipLeKiwiObservationFusion {
            kind: FLAGSHIP_LEKIWI_OBSERVATION_FUSION_KIND.to_string(),
            schema_version: FLAGSHIP_LEKIWI_OBSERVATION_FUSION_SCHEMA_VERSION,
            parent_task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID.to_string(),
            parent_sequence,
            parent_tick,
            arm_calibration_sha256: calibration_sha256.clone(),
            sources: source_evidence(parent_tick, inputs),
            observation_sha256: values_sha256(&observation_values),
            observation_values,
            unused_physical_observations: unused,
            status: "passed".to_string(),
        };
        self.expected_parent_sequence = next_parent_sequence;
        self.previous_inputs = Some(inputs.clone());
        self.arm_calibration_sha256 = Some(calibration_sha256);
        Ok(fusion)
    }
}

fn validate_calibration(
    calibration: &FlagshipLeKiwiArmCalibration,
) -> Result<(), FlagshipLeKiwiObservationError> {
    if calibration.calibration_id.trim().is_empty() {
        return Err(FlagshipLeKiwiObservationError::EmptyCalibrationId);
    }
    let mut elements = BTreeSet::new();
    for (output_element, channel) in calibration.channels.iter().enumerate() {
        if channel.physical_element >= LEKIWI_ARM_WIDTH {
            return Err(FlagshipLeKiwiObservationError::ArmSourceElement {
                output_element,
                physical_element: channel.physical_element,
            });
        }
        if !elements.insert(channel.physical_element) {
            return Err(FlagshipLeKiwiObservationError::DuplicateArmSourceElement {
                physical_element: channel.physical_element,
            });
        }
        if !channel.scale.is_finite() || channel.scale == 0.0 || !channel.offset_rad.is_finite() {
            return Err(FlagshipLeKiwiObservationError::InvalidArmCalibration { output_element });
        }
    }
    Ok(())
}

fn validate_source_set(
    parent_tick: u64,
    inputs: &FlagshipLeKiwiObservationInputs,
) -> Result<(), FlagshipLeKiwiObservationError> {
    validate_source("physical", parent_tick, &inputs.physical)?;
    validate_source("localization", parent_tick, &inputs.localization)?;
    validate_source("perception", parent_tick, &inputs.perception)?;
    validate_source("traffic", parent_tick, &inputs.traffic)?;
    validate_source("task_state", parent_tick, &inputs.task_state)?;
    Ok(())
}

fn validate_source<T>(
    role: &'static str,
    parent_tick: u64,
    sample: &FlagshipTimedObservation<T>,
) -> Result<(), FlagshipLeKiwiObservationError> {
    if sample.source_id.trim().is_empty() {
        return Err(FlagshipLeKiwiObservationError::EmptySourceId { role });
    }
    if !is_sha256_label(&sample.source_contract_sha256) {
        return Err(FlagshipLeKiwiObservationError::SourceContractDigest { role });
    }
    let age_ticks = parent_tick.checked_sub(sample.sample_tick).ok_or(
        FlagshipLeKiwiObservationError::FutureSourceSample {
            role,
            sample_tick: sample.sample_tick,
            parent_tick,
        },
    )?;
    if age_ticks > sample.max_age_ticks {
        return Err(FlagshipLeKiwiObservationError::StaleSourceSample {
            role,
            age_ticks,
            max_age_ticks: sample.max_age_ticks,
        });
    }
    Ok(())
}

fn validate_source_continuity(
    previous: &FlagshipLeKiwiObservationInputs,
    current: &FlagshipLeKiwiObservationInputs,
) -> Result<(), FlagshipLeKiwiObservationError> {
    validate_one_continuity("physical", &previous.physical, &current.physical)?;
    validate_one_continuity(
        "localization",
        &previous.localization,
        &current.localization,
    )?;
    validate_one_continuity("perception", &previous.perception, &current.perception)?;
    validate_one_continuity("traffic", &previous.traffic, &current.traffic)?;
    validate_one_continuity("task_state", &previous.task_state, &current.task_state)?;
    Ok(())
}

fn validate_one_continuity<T: Serialize>(
    role: &'static str,
    previous: &FlagshipTimedObservation<T>,
    current: &FlagshipTimedObservation<T>,
) -> Result<(), FlagshipLeKiwiObservationError> {
    if current.source_id != previous.source_id
        || current.source_contract_sha256 != previous.source_contract_sha256
    {
        return Err(FlagshipLeKiwiObservationError::SourceIdentityChanged { role });
    }
    if current.source_sequence < previous.source_sequence {
        return Err(FlagshipLeKiwiObservationError::SourceSequenceRegressed {
            role,
            previous: previous.source_sequence,
            actual: current.source_sequence,
        });
    }
    if current.source_sequence > previous.source_sequence
        && previous.source_sequence.checked_add(1) != Some(current.source_sequence)
    {
        return Err(FlagshipLeKiwiObservationError::SourceSequenceGap {
            role,
            expected: previous.source_sequence.saturating_add(1),
            actual: current.source_sequence,
        });
    }
    if current.source_sequence == previous.source_sequence
        && serde_json::to_vec(current)
            .map_err(|_| FlagshipLeKiwiObservationError::SourceSampleEncoding { role })?
            != serde_json::to_vec(previous)
                .map_err(|_| FlagshipLeKiwiObservationError::SourceSampleEncoding { role })?
    {
        return Err(
            FlagshipLeKiwiObservationError::SourceSequenceReusedWithDifferentSample {
                role,
                sequence: current.source_sequence,
            },
        );
    }
    if current.source_sequence > previous.source_sequence
        && current.sample_tick < previous.sample_tick
    {
        return Err(FlagshipLeKiwiObservationError::SourceTickRegressed {
            role,
            previous: previous.sample_tick,
            actual: current.sample_tick,
        });
    }
    Ok(())
}

fn validate_physical(values: &[f64]) -> Result<(), FlagshipLeKiwiObservationError> {
    let task = lekiwi_base_task_spec();
    let expected = [
        ("arm_joint_position_rad", 5),
        ("gripper_position_pct", 1),
        ("base_linear_velocity_m_s", 2),
        ("base_angular_velocity_rad_s", 1),
    ];
    if !task
        .observation
        .tensors
        .iter()
        .zip(expected)
        .all(|(tensor, (name, elements))| {
            tensor.name == name && tensor_elements(&tensor.shape) == elements
        })
        || task.observation.tensors.len() != expected.len()
    {
        return Err(FlagshipLeKiwiObservationError::PhysicalContractDrift);
    }
    if values.len() != LEKIWI_OBSERVATION_WIDTH {
        return Err(FlagshipLeKiwiObservationError::PhysicalWidth {
            actual: values.len(),
        });
    }
    for (element, value) in values.iter().copied().enumerate() {
        if !value.is_finite() {
            return Err(FlagshipLeKiwiObservationError::NonFinitePhysical { element });
        }
    }
    if !(0.0..=100.0).contains(&values[5]) {
        return Err(FlagshipLeKiwiObservationError::PhysicalGripperPercent { value: values[5] });
    }
    Ok(())
}

fn validate_auxiliary(
    inputs: &FlagshipLeKiwiObservationInputs,
) -> Result<(), FlagshipLeKiwiObservationError> {
    let localization = &inputs.localization.value;
    let perception = &inputs.perception.value;
    let traffic = &inputs.traffic.value;
    let task = &inputs.task_state.value;
    let scalars = localization
        .base_position_m
        .into_iter()
        .chain(perception.payload_position_m)
        .chain([perception.wrist_depth_min_m])
        .chain(traffic.actor_position_m)
        .chain([task.lift_position_m, task.gripper_position_m]);
    if let Some((element, _)) = scalars.enumerate().find(|(_, value)| !value.is_finite()) {
        return Err(FlagshipLeKiwiObservationError::NonFiniteAuxiliary { element });
    }
    if perception.wrist_camera_pixel_count <= 0
        || perception.wrist_camera_pixel_count > MAX_EXACT_F64_INTEGER
    {
        return Err(FlagshipLeKiwiObservationError::CameraPixelCount {
            value: perception.wrist_camera_pixel_count,
        });
    }
    if perception.wrist_depth_min_m <= 0.0 {
        return Err(FlagshipLeKiwiObservationError::WristDepth {
            value: perception.wrist_depth_min_m,
        });
    }
    if !(0..=9).contains(&task.policy_phase) {
        return Err(FlagshipLeKiwiObservationError::PolicyPhase {
            value: task.policy_phase,
        });
    }
    Ok(())
}

fn validate_parent_observation(values: &[f64]) -> Result<(), FlagshipLeKiwiObservationError> {
    let task = flagship_mobile_lift_task_spec(FLAGSHIP_CONTROLLER_PERIOD_TICKS);
    let expected = [
        ("base_position_m", 2),
        ("arm_joint_position_rad", 3),
        ("lift_position_m", 1),
        ("gripper_position_m", 1),
        ("payload_position_m", 3),
        ("wrist_camera_pixel_count", 1),
        ("wrist_depth_min_m", 1),
        ("traffic_actor_position_m", 3),
        ("traffic_signal_green", 1),
        ("traffic_clear", 1),
        ("grasped", 1),
        ("policy_phase", 1),
    ];
    if task.observation.tensors.len() != expected.len()
        || !task
            .observation
            .tensors
            .iter()
            .zip(expected)
            .all(|(tensor, (name, elements))| {
                tensor.name == name && tensor_elements(&tensor.shape) == elements
            })
        || values.len() != FLAGSHIP_OBSERVATION_WIDTH
    {
        return Err(FlagshipLeKiwiObservationError::ParentContractDrift);
    }
    if values.iter().any(|value| !value.is_finite())
        || !values[15..=17]
            .iter()
            .all(|value| *value == 0.0 || *value == 1.0)
        || values[18].fract() != 0.0
        || !(0.0..=9.0).contains(&values[18])
    {
        return Err(FlagshipLeKiwiObservationError::InvalidParentObservation);
    }
    Ok(())
}

fn tensor_elements(shape: &[usize]) -> usize {
    shape.iter().copied().product::<usize>().max(1)
}

fn bool_to_f64(value: bool) -> f64 {
    if value {
        1.0
    } else {
        0.0
    }
}

fn source_evidence(
    parent_tick: u64,
    inputs: &FlagshipLeKiwiObservationInputs,
) -> Vec<FlagshipObservationSourceEvidence> {
    vec![
        evidence("physical", parent_tick, &inputs.physical),
        evidence("localization", parent_tick, &inputs.localization),
        evidence("perception", parent_tick, &inputs.perception),
        evidence("traffic", parent_tick, &inputs.traffic),
        evidence("task_state", parent_tick, &inputs.task_state),
    ]
}

fn evidence<T>(
    role: &str,
    parent_tick: u64,
    sample: &FlagshipTimedObservation<T>,
) -> FlagshipObservationSourceEvidence {
    FlagshipObservationSourceEvidence {
        role: role.to_string(),
        source_id: sample.source_id.clone(),
        source_sequence: sample.source_sequence,
        sample_tick: sample.sample_tick,
        age_ticks: parent_tick - sample.sample_tick,
        max_age_ticks: sample.max_age_ticks,
        source_contract_sha256: sample.source_contract_sha256.clone(),
    }
}

fn unused_value(
    tensor_name: &str,
    tensor_element: usize,
    unit: &str,
    value: f64,
) -> UnusedLeKiwiObservation {
    UnusedLeKiwiObservation {
        tensor_name: tensor_name.to_string(),
        tensor_element,
        unit: unit.to_string(),
        value,
    }
}

fn arm_calibration_sha256(calibration: &FlagshipLeKiwiArmCalibration) -> String {
    let mut hasher = Sha256::new();
    hash_bytes(&mut hasher, calibration.calibration_id.as_bytes());
    for channel in &calibration.channels {
        hasher.update((channel.physical_element as u64).to_le_bytes());
        hasher.update(channel.scale.to_bits().to_le_bytes());
        hasher.update(channel.offset_rad.to_bits().to_le_bytes());
    }
    format!("{:x}", hasher.finalize())
}

fn values_sha256(values: &[f64]) -> String {
    let mut hasher = Sha256::new();
    hasher.update((values.len() as u64).to_le_bytes());
    for value in values {
        hasher.update(value.to_bits().to_le_bytes());
    }
    format!("{:x}", hasher.finalize())
}

fn hash_bytes(hasher: &mut Sha256, bytes: &[u8]) {
    hasher.update((bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn is_sha256_label(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

/// Failure validating source authority, freshness, continuity, or fusion.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FlagshipLeKiwiObservationError {
    /// Parent sequence was duplicated, missing, or reordered.
    #[error("flagship parent sequence must be {expected}, got {actual}")]
    UnexpectedParentSequence {
        /// Required next sequence.
        expected: u64,
        /// Supplied sequence.
        actual: u64,
    },
    /// Parent sequence cannot be advanced safely.
    #[error("flagship parent sequence overflow")]
    SequenceOverflow,
    /// Parent sequence cannot be converted to an integer simulation tick.
    #[error("flagship parent tick overflow")]
    TickOverflow,
    /// Arm calibration identity was empty.
    #[error("flagship arm calibration ID must not be empty")]
    EmptyCalibrationId,
    /// An arm mapping addressed a nonexistent physical element.
    #[error(
        "flagship arm element {output_element} maps invalid physical element {physical_element}"
    )]
    ArmSourceElement {
        /// Flagship arm element.
        output_element: usize,
        /// Invalid LeKiwi arm element.
        physical_element: usize,
    },
    /// Two flagship joints attempted to consume one physical joint observation.
    #[error("LeKiwi arm element {physical_element} is mapped more than once")]
    DuplicateArmSourceElement {
        /// Duplicated LeKiwi arm element.
        physical_element: usize,
    },
    /// Arm scale or offset was zero/nonfinite where prohibited.
    #[error("flagship arm calibration element {output_element} is invalid")]
    InvalidArmCalibration {
        /// Invalid flagship arm element.
        output_element: usize,
    },
    /// Arm calibration changed after the fuser session began.
    #[error("flagship arm calibration changed during one fuser session")]
    ArmCalibrationChanged,
    /// A required source had no stable identity.
    #[error("{role} source ID must not be empty")]
    EmptySourceId {
        /// Semantic source role.
        role: &'static str,
    },
    /// A source-contract digest was not a SHA-256 label.
    #[error("{role} source contract must be a sha256: label")]
    SourceContractDigest {
        /// Semantic source role.
        role: &'static str,
    },
    /// A source sample claimed a tick after the parent decision.
    #[error("{role} sample tick {sample_tick} is after parent tick {parent_tick}")]
    FutureSourceSample {
        /// Semantic source role.
        role: &'static str,
        /// Supplied source tick.
        sample_tick: u64,
        /// Parent decision tick.
        parent_tick: u64,
    },
    /// A source sample exceeded its declared age limit.
    #[error("{role} sample age {age_ticks} exceeds {max_age_ticks} ticks")]
    StaleSourceSample {
        /// Semantic source role.
        role: &'static str,
        /// Computed age.
        age_ticks: u64,
        /// Declared maximum age.
        max_age_ticks: u64,
    },
    /// A source identity or configuration changed during one fuser session.
    #[error("{role} source identity or contract changed")]
    SourceIdentityChanged {
        /// Semantic source role.
        role: &'static str,
    },
    /// A source-local sequence moved backwards.
    #[error("{role} source sequence regressed from {previous} to {actual}")]
    SourceSequenceRegressed {
        /// Semantic source role.
        role: &'static str,
        /// Previous sequence.
        previous: u64,
        /// Rejected sequence.
        actual: u64,
    },
    /// A source-local sequence skipped one or more unobserved samples.
    #[error("{role} source sequence must be {expected} or held, got {actual}")]
    SourceSequenceGap {
        /// Semantic source role.
        role: &'static str,
        /// Required next sequence when advancing.
        expected: u64,
        /// Rejected sequence.
        actual: u64,
    },
    /// One source sequence was reused with different bytes or metadata.
    #[error("{role} source sequence {sequence} was reused with a different sample")]
    SourceSequenceReusedWithDifferentSample {
        /// Semantic source role.
        role: &'static str,
        /// Reused sequence.
        sequence: u64,
    },
    /// A validated typed source sample could not be canonically encoded.
    #[error("{role} source sample could not be canonically encoded")]
    SourceSampleEncoding {
        /// Semantic source role.
        role: &'static str,
    },
    /// A newly sequenced source sample moved backwards in simulation ticks.
    #[error("{role} source tick regressed from {previous} to {actual}")]
    SourceTickRegressed {
        /// Semantic source role.
        role: &'static str,
        /// Previous sample tick.
        previous: u64,
        /// Rejected sample tick.
        actual: u64,
    },
    /// Physical observation width did not match LeKiwi TaskSpec order.
    #[error("LeKiwi physical observation width must be 9, got {actual}")]
    PhysicalWidth {
        /// Supplied width.
        actual: usize,
    },
    /// Compiled LeKiwi observation order no longer matches this v1 fusion.
    #[error("compiled LeKiwi observation contract drifted from fusion schema v1")]
    PhysicalContractDrift,
    /// One physical observation was NaN or infinite.
    #[error("LeKiwi physical observation element {element} must be finite")]
    NonFinitePhysical {
        /// Invalid flattened element.
        element: usize,
    },
    /// Physical gripper percentage violated its TaskSpec bound.
    #[error("LeKiwi physical gripper percentage {value} is outside 0..=100")]
    PhysicalGripperPercent {
        /// Rejected percentage.
        value: f64,
    },
    /// One required auxiliary metric was NaN or infinite.
    #[error("flagship auxiliary metric {element} must be finite")]
    NonFiniteAuxiliary {
        /// Invalid flattened auxiliary element.
        element: usize,
    },
    /// Camera pixel count was absent or not exactly representable in flattened evidence.
    #[error("flagship wrist camera pixel count {value} is invalid")]
    CameraPixelCount {
        /// Rejected count.
        value: i64,
    },
    /// Wrist depth was absent or nonpositive.
    #[error("flagship wrist minimum depth {value} must be positive")]
    WristDepth {
        /// Rejected depth in metres.
        value: f64,
    },
    /// Policy phase violated the stable `0..=9` contract.
    #[error("flagship policy phase {value} is outside 0..=9")]
    PolicyPhase {
        /// Rejected phase.
        value: i32,
    },
    /// Affine arm calibration produced NaN or infinity.
    #[error("flagship fused arm element {element} must be finite")]
    NonFiniteFusedArm {
        /// Invalid output arm element.
        element: usize,
    },
    /// Compiled parent observation shape no longer matches this v1 fusion.
    #[error("compiled flagship observation contract drifted from fusion schema v1")]
    ParentContractDrift,
    /// Final parent observation violated dtype or numeric invariants.
    #[error("fused flagship observation violates its dtype or numeric contract")]
    InvalidParentObservation,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: char) -> String {
        format!("sha256:{}", byte.to_string().repeat(64))
    }

    fn timed<T>(id: &str, value: T) -> FlagshipTimedObservation<T> {
        FlagshipTimedObservation {
            source_id: id.to_string(),
            source_sequence: 0,
            sample_tick: 0,
            max_age_ticks: FLAGSHIP_CONTROLLER_PERIOD_TICKS * 2,
            source_contract_sha256: digest('a'),
            value,
        }
    }

    fn inputs() -> FlagshipLeKiwiObservationInputs {
        FlagshipLeKiwiObservationInputs {
            physical: timed(
                "lekiwi",
                FlagshipLeKiwiPhysicalObservation {
                    values: vec![0.1, 0.2, 0.3, 0.4, 0.5, 25.0, 0.01, 0.02, 0.03],
                },
            ),
            localization: timed(
                "localization",
                FlagshipLocalizationObservation {
                    base_position_m: [1.0, 2.0],
                },
            ),
            perception: timed(
                "perception",
                FlagshipPerceptionObservation {
                    payload_position_m: [3.0, 4.0, 5.0],
                    wrist_camera_pixel_count: 640 * 480,
                    wrist_depth_min_m: 0.4,
                    grasped: true,
                },
            ),
            traffic: timed(
                "traffic",
                FlagshipTrafficObservation {
                    actor_position_m: [6.0, 7.0, 8.0],
                    signal_green: true,
                    clear: false,
                },
            ),
            task_state: timed(
                "task-state",
                FlagshipTaskStateObservation {
                    lift_position_m: 0.6,
                    gripper_position_m: 0.02,
                    policy_phase: 4,
                },
            ),
        }
    }

    fn calibration() -> FlagshipLeKiwiArmCalibration {
        FlagshipLeKiwiArmCalibration {
            calibration_id: "robot-1-to-flagship-v1".to_string(),
            channels: [
                FlagshipArmChannelCalibration {
                    physical_element: 1,
                    scale: -1.0,
                    offset_rad: 0.01,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 2,
                    scale: 1.0,
                    offset_rad: 0.02,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 4,
                    scale: 0.5,
                    offset_rad: 0.03,
                },
            ],
        }
    }

    #[test]
    fn fuses_exact_parent_order_and_records_unused_physical_values() {
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        let report = fuser.fuse(0, &inputs(), &calibration()).unwrap();
        assert_eq!(report.observation_values.len(), 19);
        assert_eq!(&report.observation_values[0..2], &[1.0, 2.0]);
        let expected_arm = [-0.19, 0.32, 0.28];
        for (actual, expected) in report.observation_values[2..5].iter().zip(expected_arm) {
            assert!((actual - expected).abs() < 1.0e-12);
        }
        assert_eq!(&report.observation_values[7..10], &[3.0, 4.0, 5.0]);
        assert_eq!(&report.observation_values[15..19], &[1.0, 0.0, 1.0, 4.0]);
        assert_eq!(report.sources.len(), 5);
        assert_eq!(report.unused_physical_observations.len(), 6);
        assert_eq!(report.observation_sha256.len(), 64);
        assert_eq!(fuser.expected_parent_sequence(), 1);
    }

    #[test]
    fn held_sources_are_allowed_only_when_the_complete_sample_is_identical() {
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        let first = inputs();
        fuser.fuse(0, &first, &calibration()).unwrap();
        fuser.fuse(1, &first, &calibration()).unwrap();

        let mut mutated = first;
        mutated.perception.value.wrist_depth_min_m = 0.5;
        assert!(matches!(
            fuser.fuse(2, &mutated, &calibration()),
            Err(
                FlagshipLeKiwiObservationError::SourceSequenceReusedWithDifferentSample {
                    role: "perception",
                    ..
                }
            )
        ));
        assert_eq!(fuser.expected_parent_sequence(), 2);
    }

    #[test]
    fn stale_future_missing_perception_and_bad_digest_fail_closed() {
        let mut stale = inputs();
        stale.physical.max_age_ticks = 0;
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        fuser.fuse(0, &stale, &calibration()).unwrap();
        assert!(matches!(
            fuser.fuse(1, &stale, &calibration()),
            Err(FlagshipLeKiwiObservationError::StaleSourceSample {
                role: "physical",
                ..
            })
        ));

        let mut future = inputs();
        future.localization.sample_tick = 1;
        assert!(matches!(
            FlagshipLeKiwiObservationFuser::new().fuse(0, &future, &calibration()),
            Err(FlagshipLeKiwiObservationError::FutureSourceSample {
                role: "localization",
                ..
            })
        ));

        let mut absent = inputs();
        absent.perception.value.wrist_camera_pixel_count = 0;
        assert!(matches!(
            FlagshipLeKiwiObservationFuser::new().fuse(0, &absent, &calibration()),
            Err(FlagshipLeKiwiObservationError::CameraPixelCount { .. })
        ));

        let mut bad_digest = inputs();
        bad_digest.traffic.source_contract_sha256 = "not-a-digest".to_string();
        assert!(matches!(
            FlagshipLeKiwiObservationFuser::new().fuse(0, &bad_digest, &calibration()),
            Err(FlagshipLeKiwiObservationError::SourceContractDigest { role: "traffic" })
        ));
    }

    #[test]
    fn invalid_calibration_physical_width_and_sequence_gap_do_not_advance() {
        let mut duplicate = calibration();
        duplicate.channels[1].physical_element = duplicate.channels[0].physical_element;
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        assert!(matches!(
            fuser.fuse(0, &inputs(), &duplicate),
            Err(FlagshipLeKiwiObservationError::DuplicateArmSourceElement { .. })
        ));
        assert_eq!(fuser.expected_parent_sequence(), 0);

        let mut narrow = inputs();
        narrow.physical.value.values.pop();
        assert!(matches!(
            fuser.fuse(0, &narrow, &calibration()),
            Err(FlagshipLeKiwiObservationError::PhysicalWidth { actual: 8 })
        ));
        assert_eq!(fuser.expected_parent_sequence(), 0);
        assert!(matches!(
            fuser.fuse(1, &inputs(), &calibration()),
            Err(FlagshipLeKiwiObservationError::UnexpectedParentSequence { .. })
        ));
    }

    #[test]
    fn source_gap_and_calibration_change_fail_without_advancing() {
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        let first = inputs();
        fuser.fuse(0, &first, &calibration()).unwrap();

        let mut gap = first.clone();
        gap.traffic.source_sequence = 2;
        gap.traffic.sample_tick = FLAGSHIP_CONTROLLER_PERIOD_TICKS;
        assert!(matches!(
            fuser.fuse(1, &gap, &calibration()),
            Err(FlagshipLeKiwiObservationError::SourceSequenceGap {
                role: "traffic",
                ..
            })
        ));
        assert_eq!(fuser.expected_parent_sequence(), 1);

        let mut changed = calibration();
        changed.channels[0].offset_rad += 0.1;
        assert_eq!(
            fuser.fuse(1, &first, &changed),
            Err(FlagshipLeKiwiObservationError::ArmCalibrationChanged)
        );
        assert_eq!(fuser.expected_parent_sequence(), 1);
    }

    #[test]
    fn reused_sequence_rejects_float_bit_changes_including_negative_zero() {
        let mut first = inputs();
        first.physical.value.values[6] = 0.0;
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        fuser.fuse(0, &first, &calibration()).unwrap();

        let mut changed = first;
        changed.physical.value.values[6] = -0.0;
        assert!(matches!(
            fuser.fuse(1, &changed, &calibration()),
            Err(
                FlagshipLeKiwiObservationError::SourceSequenceReusedWithDifferentSample {
                    role: "physical",
                    ..
                }
            )
        ));
        assert_eq!(fuser.expected_parent_sequence(), 1);
    }
}
