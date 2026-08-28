//! Replayable full-content shadow-run contracts for the flagship LeKiwi path.
//!
//! Filesystem hashing and path confinement are performed by `xtask`. This
//! module owns the portable manifest, typed stream artifacts, and deterministic
//! semantic replay of the action, rate, and observation boundaries.

use crate::flagship_observation::{
    FlagshipLeKiwiArmCalibration, FlagshipLeKiwiObservationError, FlagshipLeKiwiObservationFuser,
    FlagshipLeKiwiObservationFuserV2, FlagshipLeKiwiObservationFusion,
    FlagshipLeKiwiObservationFusionV2, FlagshipLeKiwiObservationInputs,
    FlagshipLeKiwiObservationInputsV2,
};
use crate::flagship_projection::{
    project_flagship_action_to_lekiwi, project_flagship_action_to_lekiwi_v2,
    FlagshipLeKiwiActionProjection, FlagshipLeKiwiProjectionError,
};
use crate::flagship_rate::{
    FlagshipLeKiwiRateDecision, FlagshipLeKiwiRateError, FlagshipLeKiwiRateScheduler,
    FlagshipLeKiwiRateSchedulerV2,
};
use crate::physical_evidence::EvidenceFileRef;
use crate::session::{LEKIWI_REFERENCE_SESSION_KIND, LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION};
use crate::{
    LEKIWI_MOCK_DEVICE_ID, LEKIWI_PHYSICAL_DEVICE_ID_PREFIX, LEKIWI_REFERENCE_PROFILE_KIND,
    LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
};
use rne_ai::{
    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND,
    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION, FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID,
    FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2, FLAGSHIP_MOBILE_LIFT_TASK_ID,
    FLAGSHIP_MOBILE_LIFT_TASK_ID_V2, TASK_SPEC_KIND, TASK_SPEC_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Schema version for the complete flagship LeKiwi shadow manifest.
pub const FLAGSHIP_LEKIWI_SHADOW_MANIFEST_SCHEMA_VERSION: u32 = 1;

/// Current shadow-manifest schema version for portable v2 controller evidence.
pub const FLAGSHIP_LEKIWI_SHADOW_MANIFEST_CURRENT_SCHEMA_VERSION: u32 = 2;

/// Stable discriminator for [`FlagshipLeKiwiShadowManifest`].
pub const FLAGSHIP_LEKIWI_SHADOW_MANIFEST_KIND: &str = "rne_flagship_lekiwi_shadow_manifest";

/// Schema version for the controller contract bound by the shadow manifest.
pub const FLAGSHIP_CONTROLLER_CONTRACT_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipControllerContract`].
pub const FLAGSHIP_CONTROLLER_CONTRACT_KIND: &str = "rne_controller_contract";

/// Schema version for the arm morphology-calibration artifact.
pub const FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipLeKiwiArmCalibrationArtifact`].
pub const FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND: &str = "rne_flagship_lekiwi_arm_calibration";

/// Schema version for an auxiliary observation-source contract.
pub const FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipObservationSourceContract`].
pub const FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND: &str =
    "rne_flagship_observation_source_contract";

/// Schema version for a replayable action-projection stream.
pub const FLAGSHIP_ACTION_PROJECTION_STREAM_SCHEMA_VERSION: u32 = 1;

/// Current action-stream schema version for portable v2 identities.
pub const FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION: u32 = 2;

/// Stable discriminator for [`FlagshipActionProjectionStream`].
pub const FLAGSHIP_ACTION_PROJECTION_STREAM_KIND: &str =
    "rne_flagship_lekiwi_action_projection_stream";

/// Schema version for a replayable rate-decision stream.
pub const FLAGSHIP_RATE_DECISION_STREAM_SCHEMA_VERSION: u32 = 1;

/// Current rate-stream schema version for portable v2 identities.
pub const FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION: u32 = 2;

/// Stable discriminator for [`FlagshipRateDecisionStream`].
pub const FLAGSHIP_RATE_DECISION_STREAM_KIND: &str = "rne_flagship_lekiwi_rate_decision_stream";

/// Schema version for a replayable observation-fusion stream.
pub const FLAGSHIP_OBSERVATION_FUSION_STREAM_SCHEMA_VERSION: u32 = 1;

/// Current observation-stream schema version for portable v2 inputs and fusion.
pub const FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION: u32 = 2;

/// Stable discriminator for [`FlagshipObservationFusionStream`].
pub const FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND: &str =
    "rne_flagship_lekiwi_observation_fusion_stream";

const FLAGSHIP_CONTROLLER_ACTION_ORDER: [&str; 7] = [
    "left_wheel_velocity_rad_s",
    "right_wheel_velocity_rad_s",
    "shoulder_target_rad",
    "elbow_target_rad",
    "wrist_yaw_target_rad",
    "lift_target_m",
    "gripper_velocity_m_s",
];

/// Exact built-in controller contract used by all flagship execution paths.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipControllerContract {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Controller-contract schema version.
    pub schema_version: u32,
    /// Exact controller identity.
    pub controller_id: String,
    /// Stable Rust policy type identity.
    pub policy: String,
    /// Stable missing-target normalization contract.
    pub normalization: String,
    /// Flattened action element order.
    pub action_order: Vec<String>,
}

impl FlagshipControllerContract {
    /// Returns the canonical built-in flagship controller contract.
    pub fn built_in() -> Self {
        Self {
            kind: FLAGSHIP_CONTROLLER_CONTRACT_KIND.to_string(),
            schema_version: FLAGSHIP_CONTROLLER_CONTRACT_SCHEMA_VERSION,
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID.to_string(),
            policy: "IkMobileLiftPickPlacePolicy".to_string(),
            normalization: "missing_joint_targets_hold_pre_step_observation_v1".to_string(),
            action_order: FLAGSHIP_CONTROLLER_ACTION_ORDER
                .into_iter()
                .map(str::to_string)
                .collect(),
        }
    }

    /// Rejects any controller identity or action-order drift.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        if self != &Self::built_in() {
            return Err(invalid(
                "controller_contract",
                "does not equal the built-in contract",
            ));
        }
        Ok(())
    }
}

/// Versioned file wrapper for the explicit arm morphology calibration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiArmCalibrationArtifact {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Calibration-artifact schema version.
    pub schema_version: u32,
    /// Exact calibration consumed by observation fusion.
    pub calibration: FlagshipLeKiwiArmCalibration,
}

impl FlagshipLeKiwiArmCalibrationArtifact {
    /// Validates kind/schema plus the embedded morphology calibration.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        if self.kind != FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND
            || self.schema_version != FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION
        {
            return Err(invalid("arm_calibration", "unsupported kind or schema"));
        }
        self.calibration
            .validate()
            .map_err(FlagshipLeKiwiShadowError::Observation)
    }
}

/// Semantic role of one auxiliary observation source.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlagshipObservationSourceRole {
    /// Metric base localization.
    Localization,
    /// Payload and wrist RGB-D perception.
    Perception,
    /// Task-level traffic state.
    Traffic,
    /// Lift, gripper, and controller phase state.
    TaskState,
}

/// Content-bound configuration contract for one auxiliary observation source.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationSourceContract {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Source-contract schema version.
    pub schema_version: u32,
    /// Required semantic role.
    pub role: FlagshipObservationSourceRole,
    /// Stable source identity copied into every timed observation.
    pub source_id: String,
    /// Exact timestamp domain; v1 requires integer RNE simulation ticks.
    pub clock_domain: String,
    /// Maximum age copied into every timed observation from this source.
    pub max_age_ticks: u64,
    /// Bounded implementation/configuration identity.
    pub implementation_id: String,
}

impl FlagshipObservationSourceContract {
    /// Validates the stable source boundary without reading its containing file.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        if self.kind != FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND
            || self.schema_version != FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION
        {
            return Err(invalid("source_contract", "unsupported kind or schema"));
        }
        validate_identifier("source_contract.source_id", &self.source_id)?;
        validate_identifier("source_contract.implementation_id", &self.implementation_id)?;
        if self.clock_domain != "rne_sim_tick_ns" {
            return Err(invalid(
                "source_contract.clock_domain",
                "must be rne_sim_tick_ns",
            ));
        }
        if self.max_age_ticks == 0 {
            return Err(invalid(
                "source_contract.max_age_ticks",
                "must be greater than zero",
            ));
        }
        Ok(())
    }
}

/// Input and recomputed output for one action-projection record.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipActionProjectionRecord {
    /// Exact seven-element parent action.
    pub parent_action: Vec<f64>,
    /// Expected deterministic projection.
    pub projection: FlagshipLeKiwiActionProjection,
}

/// Replayable ordered action-projection evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipActionProjectionStream {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Stream schema version.
    pub schema_version: u32,
    /// Exact parent TaskSpec identity.
    pub task_id: String,
    /// Exact parent controller identity.
    pub controller_id: String,
    /// Non-empty ordered records.
    pub records: Vec<FlagshipActionProjectionRecord>,
}

impl FlagshipActionProjectionStream {
    /// Replays every action projection and rejects any mismatch.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        let (task_id, controller_id) = stream_parent_identity(self.schema_version)?;
        validate_stream_header(
            &self.kind,
            self.schema_version,
            FLAGSHIP_ACTION_PROJECTION_STREAM_KIND,
            self.schema_version,
            &self.task_id,
            &self.controller_id,
            (task_id, controller_id),
        )?;
        ensure_records(&self.records)?;
        for (index, record) in self.records.iter().enumerate() {
            let actual = if self.schema_version == FLAGSHIP_ACTION_PROJECTION_STREAM_SCHEMA_VERSION
            {
                project_flagship_action_to_lekiwi(&record.parent_action)?
            } else {
                project_flagship_action_to_lekiwi_v2(&record.parent_action)?
            };
            if actual != record.projection {
                return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                    stream: "action_projection",
                    index,
                });
            }
        }
        Ok(())
    }
}

/// Input and recomputed output for one deterministic rate decision.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipRateDecisionRecord {
    /// Exact seven-element parent action.
    pub parent_action: Vec<f64>,
    /// Expected deterministic rate decision.
    pub decision: FlagshipLeKiwiRateDecision,
}

/// Replayable ordered rate-decision evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipRateDecisionStream {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Stream schema version.
    pub schema_version: u32,
    /// Exact parent TaskSpec identity.
    pub task_id: String,
    /// Exact parent controller identity.
    pub controller_id: String,
    /// Non-empty contiguous records beginning at parent sequence zero.
    pub records: Vec<FlagshipRateDecisionRecord>,
}

impl FlagshipRateDecisionStream {
    /// Replays every stateful rate decision and rejects any mismatch.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        let (task_id, controller_id) = stream_parent_identity(self.schema_version)?;
        validate_stream_header(
            &self.kind,
            self.schema_version,
            FLAGSHIP_RATE_DECISION_STREAM_KIND,
            self.schema_version,
            &self.task_id,
            &self.controller_id,
            (task_id, controller_id),
        )?;
        ensure_records(&self.records)?;
        if self.schema_version == FLAGSHIP_RATE_DECISION_STREAM_SCHEMA_VERSION {
            let mut scheduler = FlagshipLeKiwiRateScheduler::new();
            for (index, record) in self.records.iter().enumerate() {
                let sequence = u64::try_from(index)
                    .map_err(|_| invalid("rate_stream.records", "record index exceeds u64"))?;
                let actual = scheduler.ingest(sequence, &record.parent_action)?;
                if actual != record.decision {
                    return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                        stream: "rate_decision",
                        index,
                    });
                }
            }
        } else {
            let mut scheduler = FlagshipLeKiwiRateSchedulerV2::new();
            for (index, record) in self.records.iter().enumerate() {
                let sequence = u64::try_from(index)
                    .map_err(|_| invalid("rate_stream.records", "record index exceeds u64"))?;
                let actual = scheduler.ingest(sequence, &record.parent_action)?;
                if actual != record.decision {
                    return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                        stream: "rate_decision",
                        index,
                    });
                }
            }
        }
        Ok(())
    }
}

/// Input and recomputed output for one parent-order observation fusion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationFusionRecord {
    /// Exact typed source set consumed by the fuser.
    pub inputs: FlagshipLeKiwiObservationInputs,
    /// Expected deterministic fusion result.
    pub fusion: FlagshipLeKiwiObservationFusion,
}

/// Replayable ordered observation-fusion evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationFusionStream {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Stream schema version.
    pub schema_version: u32,
    /// Exact parent TaskSpec identity.
    pub task_id: String,
    /// Exact parent controller identity.
    pub controller_id: String,
    /// Non-empty contiguous records beginning at parent sequence zero.
    pub records: Vec<FlagshipObservationFusionRecord>,
}

impl FlagshipObservationFusionStream {
    /// Replays every stateful fusion against the exact arm calibration.
    pub fn validate(
        &self,
        calibration: &FlagshipLeKiwiArmCalibration,
    ) -> Result<(), FlagshipLeKiwiShadowError> {
        validate_stream_header(
            &self.kind,
            self.schema_version,
            FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND,
            FLAGSHIP_OBSERVATION_FUSION_STREAM_SCHEMA_VERSION,
            &self.task_id,
            &self.controller_id,
            (
                FLAGSHIP_MOBILE_LIFT_TASK_ID,
                FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID,
            ),
        )?;
        ensure_records(&self.records)?;
        calibration
            .validate()
            .map_err(FlagshipLeKiwiShadowError::Observation)?;
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        for (index, record) in self.records.iter().enumerate() {
            let sequence = u64::try_from(index)
                .map_err(|_| invalid("observation_stream.records", "record index exceeds u64"))?;
            let actual = fuser.fuse(sequence, &record.inputs, calibration)?;
            if actual != record.fusion {
                return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                    stream: "observation_fusion",
                    index,
                });
            }
        }
        Ok(())
    }
}

/// One complete v2 source set and its expected fusion output.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationFusionRecordV2 {
    /// Exact typed v2 source set consumed by the fuser.
    pub inputs: FlagshipLeKiwiObservationInputsV2,
    /// Expected deterministic v2 fusion result.
    pub fusion: FlagshipLeKiwiObservationFusionV2,
    /// Exact portable-controller action produced from `fusion.observation_values`.
    pub controller_action: Vec<f64>,
}

/// Replayable ordered v2 observation-fusion evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipObservationFusionStreamV2 {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Stream schema version; exactly two.
    pub schema_version: u32,
    /// Exact portable v2 TaskSpec identity.
    pub task_id: String,
    /// Exact portable v2 controller identity.
    pub controller_id: String,
    /// Non-empty contiguous records beginning at parent sequence zero.
    pub records: Vec<FlagshipObservationFusionRecordV2>,
}

impl FlagshipObservationFusionStreamV2 {
    /// Replays every v2 fusion and proves the portable controller accepts it.
    pub fn validate(
        &self,
        calibration: &FlagshipLeKiwiArmCalibration,
    ) -> Result<(), FlagshipLeKiwiShadowError> {
        validate_stream_header(
            &self.kind,
            self.schema_version,
            FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND,
            FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION,
            &self.task_id,
            &self.controller_id,
            (
                FLAGSHIP_MOBILE_LIFT_TASK_ID_V2,
                FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2,
            ),
        )?;
        ensure_records(&self.records)?;
        calibration
            .validate()
            .map_err(FlagshipLeKiwiShadowError::Observation)?;
        let mut fuser = FlagshipLeKiwiObservationFuserV2::new();
        let mut controller = rne_ai::FlagshipMobileLiftControllerV2::new();
        for (index, record) in self.records.iter().enumerate() {
            let sequence = u64::try_from(index)
                .map_err(|_| invalid("observation_stream.records", "record index exceeds u64"))?;
            let actual = fuser.fuse(sequence, &record.inputs, calibration)?;
            if actual != record.fusion {
                return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                    stream: "observation_fusion_v2",
                    index,
                });
            }
            let actual_action = controller.next_action(&actual.observation_values)?;
            if actual_action != record.controller_action {
                return Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                    stream: "controller_action_v2",
                    index,
                });
            }
        }
        Ok(())
    }
}

/// Whether a non-actuating shadow run used a mock or physical device stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlagshipLeKiwiShadowExecutionClass {
    /// Dependency-free mock device; never physical evidence.
    Mock,
    /// Physical device connected in non-actuating shadow mode.
    PhysicalShadow,
}

/// Fixed full-content artifact set for one replayable shadow run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiShadowArtifacts {
    /// Exact release TaskSpec file.
    pub task_spec: EvidenceFileRef,
    /// Exact built-in controller contract file.
    pub controller_contract: EvidenceFileRef,
    /// Exact LeKiwi reference profile file.
    pub reference_profile: EvidenceFileRef,
    /// Exact morphology-calibration file.
    pub arm_calibration: EvidenceFileRef,
    /// Metric localization source contract.
    pub localization_source: EvidenceFileRef,
    /// Payload/wrist perception source contract.
    pub perception_source: EvidenceFileRef,
    /// Traffic source contract.
    pub traffic_source: EvidenceFileRef,
    /// Lift/gripper/policy task-state source contract.
    pub task_state_source: EvidenceFileRef,
    /// Underlying non-actuating LeKiwi reference session.
    pub physical_shadow_session: EvidenceFileRef,
    /// Replayable action-projection stream.
    pub action_projection_stream: EvidenceFileRef,
    /// Replayable rate-decision stream.
    pub rate_decision_stream: EvidenceFileRef,
    /// Replayable observation-fusion stream.
    pub observation_fusion_stream: EvidenceFileRef,
}

impl FlagshipLeKiwiShadowArtifacts {
    /// Returns every required artifact in stable semantic order.
    pub fn all(&self) -> [(&'static str, &EvidenceFileRef); 12] {
        [
            ("task_spec", &self.task_spec),
            ("controller_contract", &self.controller_contract),
            ("reference_profile", &self.reference_profile),
            ("arm_calibration", &self.arm_calibration),
            ("localization_source", &self.localization_source),
            ("perception_source", &self.perception_source),
            ("traffic_source", &self.traffic_source),
            ("task_state_source", &self.task_state_source),
            ("physical_shadow_session", &self.physical_shadow_session),
            ("action_projection_stream", &self.action_projection_stream),
            ("rate_decision_stream", &self.rate_decision_stream),
            ("observation_fusion_stream", &self.observation_fusion_stream),
        ]
    }

    /// Returns mutable artifact access in stable semantic order.
    pub fn all_mut(&mut self) -> [(&'static str, &mut EvidenceFileRef); 12] {
        [
            ("task_spec", &mut self.task_spec),
            ("controller_contract", &mut self.controller_contract),
            ("reference_profile", &mut self.reference_profile),
            ("arm_calibration", &mut self.arm_calibration),
            ("localization_source", &mut self.localization_source),
            ("perception_source", &mut self.perception_source),
            ("traffic_source", &mut self.traffic_source),
            ("task_state_source", &mut self.task_state_source),
            ("physical_shadow_session", &mut self.physical_shadow_session),
            (
                "action_projection_stream",
                &mut self.action_projection_stream,
            ),
            ("rate_decision_stream", &mut self.rate_decision_stream),
            (
                "observation_fusion_stream",
                &mut self.observation_fusion_stream,
            ),
        ]
    }
}

/// Content-addressed index for one complete, non-actuating flagship shadow run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiShadowManifest {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Manifest schema version.
    pub schema_version: u32,
    /// Stable run identity.
    pub run_id: String,
    /// Full source commit used to build the host.
    pub rne_commit: String,
    /// Mock or physical-shadow classification.
    pub execution_class: FlagshipLeKiwiShadowExecutionClass,
    /// Exact Ready-handshake device identity.
    pub device_id: String,
    /// Exact parent TaskSpec identity.
    pub task_id: String,
    /// Exact parent controller identity.
    pub controller_id: String,
    /// Number of records required in every boundary stream.
    pub sample_count: usize,
    /// Must remain false for this non-actuating manifest.
    pub actuator_writes_emitted: bool,
    /// Complete full-content artifact set.
    pub artifacts: FlagshipLeKiwiShadowArtifacts,
    /// SHA-256 of compact JSON with this field empty.
    pub content_sha256: String,
}

impl FlagshipLeKiwiShadowManifest {
    /// Computes the self-excluding deterministic manifest digest.
    pub fn computed_content_sha256(&self) -> Result<String, FlagshipLeKiwiShadowError> {
        let mut canonical = self.clone();
        canonical.content_sha256.clear();
        let bytes = serde_json::to_vec(&canonical).map_err(|error| {
            FlagshipLeKiwiShadowError::Serialization {
                reason: error.to_string(),
            }
        })?;
        Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
    }

    /// Recomputes and stores the self-excluding manifest digest.
    pub fn seal(&mut self) -> Result<(), FlagshipLeKiwiShadowError> {
        self.content_sha256 = self.computed_content_sha256()?;
        Ok(())
    }

    /// Validates schema, identities, fixed roles, unique paths, and digest.
    pub fn validate(&self) -> Result<(), FlagshipLeKiwiShadowError> {
        if self.kind != FLAGSHIP_LEKIWI_SHADOW_MANIFEST_KIND
            || !matches!(self.schema_version, 1 | 2)
        {
            return Err(invalid("manifest", "unsupported kind or schema"));
        }
        validate_identifier("run_id", &self.run_id)?;
        validate_git_revision(&self.rne_commit)?;
        let (task_id, controller_id) = stream_parent_identity(self.schema_version)?;
        if self.task_id != task_id || self.controller_id != controller_id {
            return Err(invalid(
                "manifest",
                "wrong parent task or controller identity",
            ));
        }
        if self.sample_count == 0 || self.sample_count > 1_000_000 {
            return Err(invalid("sample_count", "must be in 1..=1000000"));
        }
        if self.actuator_writes_emitted {
            return Err(invalid(
                "actuator_writes_emitted",
                "must be false in shadow",
            ));
        }
        match self.execution_class {
            FlagshipLeKiwiShadowExecutionClass::Mock if self.device_id != LEKIWI_MOCK_DEVICE_ID => {
                return Err(invalid(
                    "device_id",
                    "mock class requires the mock device ID",
                ));
            }
            FlagshipLeKiwiShadowExecutionClass::PhysicalShadow
                if !self.device_id.starts_with(LEKIWI_PHYSICAL_DEVICE_ID_PREFIX)
                    || self.device_id == LEKIWI_PHYSICAL_DEVICE_ID_PREFIX =>
            {
                return Err(invalid(
                    "device_id",
                    "physical shadow requires a concrete physical device ID",
                ));
            }
            _ => {}
        }

        let (controller_kind, controller_schema, stream_schema) =
            if self.schema_version == FLAGSHIP_LEKIWI_SHADOW_MANIFEST_SCHEMA_VERSION {
                (
                    FLAGSHIP_CONTROLLER_CONTRACT_KIND,
                    FLAGSHIP_CONTROLLER_CONTRACT_SCHEMA_VERSION,
                    1,
                )
            } else {
                (
                    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND,
                    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION,
                    2,
                )
            };
        let expected = [
            ("task_spec", TASK_SPEC_KIND, TASK_SPEC_SCHEMA_VERSION),
            ("controller_contract", controller_kind, controller_schema),
            (
                "reference_profile",
                LEKIWI_REFERENCE_PROFILE_KIND,
                LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
            ),
            (
                "arm_calibration",
                FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND,
                FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION,
            ),
            (
                "localization_source",
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            ),
            (
                "perception_source",
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            ),
            (
                "traffic_source",
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            ),
            (
                "task_state_source",
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            ),
            (
                "physical_shadow_session",
                LEKIWI_REFERENCE_SESSION_KIND,
                LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
            ),
            (
                "action_projection_stream",
                FLAGSHIP_ACTION_PROJECTION_STREAM_KIND,
                stream_schema,
            ),
            (
                "rate_decision_stream",
                FLAGSHIP_RATE_DECISION_STREAM_KIND,
                stream_schema,
            ),
            (
                "observation_fusion_stream",
                FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND,
                stream_schema,
            ),
        ];
        let mut paths = BTreeSet::new();
        for ((role, artifact), (expected_role, kind, schema_version)) in
            self.artifacts.all().into_iter().zip(expected)
        {
            if role != expected_role
                || artifact.kind != kind
                || artifact.schema_version != schema_version
            {
                return Err(invalid(
                    role,
                    "artifact kind or schema does not match its role",
                ));
            }
            artifact
                .validate()
                .map_err(|error| FlagshipLeKiwiShadowError::Invalid {
                    field: role,
                    reason: error.to_string(),
                })?;
            if !paths.insert(artifact.path.as_str()) {
                return Err(invalid(role, "artifact paths must be unique"));
            }
        }
        validate_sha256("content_sha256", &self.content_sha256)?;
        if self.content_sha256 != self.computed_content_sha256()? {
            return Err(FlagshipLeKiwiShadowError::DigestMismatch);
        }
        Ok(())
    }
}

fn validate_stream_header(
    kind: &str,
    schema_version: u32,
    expected_kind: &str,
    expected_schema_version: u32,
    task_id: &str,
    controller_id: &str,
    expected_parent: (&str, &str),
) -> Result<(), FlagshipLeKiwiShadowError> {
    if kind != expected_kind || schema_version != expected_schema_version {
        return Err(invalid("stream", "unsupported kind or schema"));
    }
    if task_id != expected_parent.0 || controller_id != expected_parent.1 {
        return Err(invalid(
            "stream",
            "wrong parent task or controller identity",
        ));
    }
    Ok(())
}

fn stream_parent_identity(
    schema_version: u32,
) -> Result<(&'static str, &'static str), FlagshipLeKiwiShadowError> {
    match schema_version {
        1 => Ok((
            FLAGSHIP_MOBILE_LIFT_TASK_ID,
            FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID,
        )),
        2 => Ok((
            FLAGSHIP_MOBILE_LIFT_TASK_ID_V2,
            FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2,
        )),
        _ => Err(invalid("stream", "unsupported schema version")),
    }
}

fn ensure_records<T>(records: &[T]) -> Result<(), FlagshipLeKiwiShadowError> {
    if records.is_empty() || records.len() > 1_000_000 {
        return Err(invalid(
            "stream.records",
            "must contain 1..=1000000 records",
        ));
    }
    Ok(())
}

fn validate_identifier(field: &'static str, value: &str) -> Result<(), FlagshipLeKiwiShadowError> {
    if value.is_empty()
        || value.len() > 128
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
    {
        return Err(invalid(field, "must be a bounded ASCII identifier"));
    }
    Ok(())
}

fn validate_git_revision(value: &str) -> Result<(), FlagshipLeKiwiShadowError> {
    if value.len() != 40
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid(
            "rne_commit",
            "must be 40 lowercase hexadecimal digits",
        ));
    }
    Ok(())
}

fn validate_sha256(field: &'static str, value: &str) -> Result<(), FlagshipLeKiwiShadowError> {
    let hex = value.strip_prefix("sha256:").unwrap_or_default();
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid(field, "must be a lowercase sha256: digest"));
    }
    Ok(())
}

fn invalid(field: &'static str, reason: &'static str) -> FlagshipLeKiwiShadowError {
    FlagshipLeKiwiShadowError::Invalid {
        field,
        reason: reason.to_string(),
    }
}

/// Failure validating or replaying a flagship LeKiwi shadow contract.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FlagshipLeKiwiShadowError {
    /// A portable field violated its schema or role invariant.
    #[error("invalid flagship LeKiwi shadow field {field}: {reason}")]
    Invalid {
        /// Stable field or role.
        field: &'static str,
        /// Failure reason.
        reason: String,
    },
    /// One recomputed stream record differed from declared evidence.
    #[error("{stream} stream record {index} does not replay exactly")]
    ReplayMismatch {
        /// Stable stream role.
        stream: &'static str,
        /// Zero-based record index.
        index: usize,
    },
    /// The self-excluding manifest digest did not match.
    #[error("flagship LeKiwi shadow manifest content digest mismatch")]
    DigestMismatch,
    /// Portable manifest serialization failed.
    #[error("could not serialize flagship LeKiwi shadow manifest: {reason}")]
    Serialization {
        /// Serialization error without a foreign public error type.
        reason: String,
    },
    /// Action projection failed closed during replay.
    #[error(transparent)]
    Projection(#[from] FlagshipLeKiwiProjectionError),
    /// Rate scheduling failed closed during replay.
    #[error(transparent)]
    Rate(#[from] FlagshipLeKiwiRateError),
    /// Observation fusion failed closed during replay.
    #[error(transparent)]
    Observation(#[from] FlagshipLeKiwiObservationError),
    /// Portable v2 controller rejected a fused TaskSpec observation.
    #[error(transparent)]
    Controller(#[from] rne_ai::FlagshipMobileLiftControllerError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flagship_observation::{
        FlagshipArmChannelCalibration, FlagshipLeKiwiPhysicalObservation,
        FlagshipLocalizationObservation, FlagshipLocalizationObservationV2,
        FlagshipPerceptionObservation, FlagshipTaskStateObservation,
        FlagshipTaskStateObservationV2, FlagshipTimedObservation, FlagshipTrafficObservation,
    };
    use crate::flagship_rate::FLAGSHIP_CONTROLLER_PERIOD_TICKS;

    fn calibration() -> FlagshipLeKiwiArmCalibration {
        FlagshipLeKiwiArmCalibration {
            calibration_id: "shadow-calibration-v1".to_string(),
            channels: [
                FlagshipArmChannelCalibration {
                    physical_element: 0,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 1,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 2,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
            ],
        }
    }

    fn timed<T>(role: &str, sequence: u64, value: T) -> FlagshipTimedObservation<T> {
        FlagshipTimedObservation {
            source_id: format!("{role}-source"),
            source_sequence: sequence,
            sample_tick: sequence * FLAGSHIP_CONTROLLER_PERIOD_TICKS,
            max_age_ticks: FLAGSHIP_CONTROLLER_PERIOD_TICKS,
            source_contract_sha256: format!("sha256:{}", "a".repeat(64)),
            value,
        }
    }

    fn inputs(sequence: u64) -> FlagshipLeKiwiObservationInputs {
        FlagshipLeKiwiObservationInputs {
            physical: timed(
                "physical",
                sequence,
                FlagshipLeKiwiPhysicalObservation {
                    values: vec![0.1, 0.2, 0.3, 0.4, 0.5, 25.0, 0.0, 0.0, 0.0],
                },
            ),
            localization: timed(
                "localization",
                sequence,
                FlagshipLocalizationObservation {
                    base_position_m: [sequence as f64 * 0.01, 0.0],
                },
            ),
            perception: timed(
                "perception",
                sequence,
                FlagshipPerceptionObservation {
                    payload_position_m: [0.5, 0.2, -0.4],
                    wrist_camera_pixel_count: 640 * 480,
                    wrist_depth_min_m: 0.4,
                    grasped: false,
                },
            ),
            traffic: timed(
                "traffic",
                sequence,
                FlagshipTrafficObservation {
                    actor_position_m: [1.0, 0.0, 0.0],
                    signal_green: true,
                    clear: true,
                },
            ),
            task_state: timed(
                "task-state",
                sequence,
                FlagshipTaskStateObservation {
                    lift_position_m: 0.0,
                    gripper_position_m: 0.02,
                    policy_phase: 1,
                },
            ),
        }
    }

    fn inputs_v2(sequence: u64) -> FlagshipLeKiwiObservationInputsV2 {
        FlagshipLeKiwiObservationInputsV2 {
            physical: timed(
                "physical",
                sequence,
                FlagshipLeKiwiPhysicalObservation {
                    values: vec![0.1, 0.2, 0.3, 0.4, 0.5, 25.0, 0.0, 0.0, 0.0],
                },
            ),
            localization: timed(
                "localization",
                sequence,
                FlagshipLocalizationObservationV2 {
                    base_position_m: [0.0, 0.0, 0.0],
                    base_yaw_rad: 0.0,
                },
            ),
            perception: timed(
                "perception",
                sequence,
                FlagshipPerceptionObservation {
                    payload_position_m: [0.5, 0.2, -0.4],
                    wrist_camera_pixel_count: 640 * 480,
                    wrist_depth_min_m: 0.4,
                    grasped: false,
                },
            ),
            traffic: timed(
                "traffic",
                sequence,
                FlagshipTrafficObservation {
                    actor_position_m: [1.0, 0.0, 0.0],
                    signal_green: true,
                    clear: true,
                },
            ),
            task_state: timed(
                "task-state",
                sequence,
                FlagshipTaskStateObservationV2 {
                    lift_position_m: 0.0,
                    gripper_position_m: 0.02,
                    place_target_position_m: [0.8, 0.02, 0.0],
                    policy_phase: 0,
                },
            ),
        }
    }

    fn action(sequence: usize) -> Vec<f64> {
        let wheel = sequence as f64 * 0.01;
        vec![wheel, wheel, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

    fn artifact(kind: &str, schema_version: u32, path: &str) -> EvidenceFileRef {
        EvidenceFileRef::new(
            kind,
            schema_version,
            path,
            format!("sha256:{}", "b".repeat(64)),
        )
    }

    fn artifacts() -> FlagshipLeKiwiShadowArtifacts {
        FlagshipLeKiwiShadowArtifacts {
            task_spec: artifact(TASK_SPEC_KIND, TASK_SPEC_SCHEMA_VERSION, "task.json"),
            controller_contract: artifact(
                FLAGSHIP_CONTROLLER_CONTRACT_KIND,
                FLAGSHIP_CONTROLLER_CONTRACT_SCHEMA_VERSION,
                "controller.json",
            ),
            reference_profile: artifact(
                LEKIWI_REFERENCE_PROFILE_KIND,
                LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
                "profile.json",
            ),
            arm_calibration: artifact(
                FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND,
                FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION,
                "arm-calibration.json",
            ),
            localization_source: artifact(
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                "localization.json",
            ),
            perception_source: artifact(
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                "perception.json",
            ),
            traffic_source: artifact(
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                "traffic.json",
            ),
            task_state_source: artifact(
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                "task-state.json",
            ),
            physical_shadow_session: artifact(
                LEKIWI_REFERENCE_SESSION_KIND,
                LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
                "session.json",
            ),
            action_projection_stream: artifact(
                FLAGSHIP_ACTION_PROJECTION_STREAM_KIND,
                FLAGSHIP_ACTION_PROJECTION_STREAM_SCHEMA_VERSION,
                "actions.json",
            ),
            rate_decision_stream: artifact(
                FLAGSHIP_RATE_DECISION_STREAM_KIND,
                FLAGSHIP_RATE_DECISION_STREAM_SCHEMA_VERSION,
                "rates.json",
            ),
            observation_fusion_stream: artifact(
                FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND,
                FLAGSHIP_OBSERVATION_FUSION_STREAM_SCHEMA_VERSION,
                "observations.json",
            ),
        }
    }

    fn manifest() -> FlagshipLeKiwiShadowManifest {
        let mut manifest = FlagshipLeKiwiShadowManifest {
            kind: FLAGSHIP_LEKIWI_SHADOW_MANIFEST_KIND.to_string(),
            schema_version: FLAGSHIP_LEKIWI_SHADOW_MANIFEST_SCHEMA_VERSION,
            run_id: "flagship-lekiwi-mock-shadow-001".to_string(),
            rne_commit: "a".repeat(40),
            execution_class: FlagshipLeKiwiShadowExecutionClass::Mock,
            device_id: LEKIWI_MOCK_DEVICE_ID.to_string(),
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID.to_string(),
            sample_count: 3,
            actuator_writes_emitted: false,
            artifacts: artifacts(),
            content_sha256: String::new(),
        };
        manifest.seal().unwrap();
        manifest
    }

    fn manifest_v2() -> FlagshipLeKiwiShadowManifest {
        let mut manifest = manifest();
        manifest.schema_version = FLAGSHIP_LEKIWI_SHADOW_MANIFEST_CURRENT_SCHEMA_VERSION;
        manifest.task_id = FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string();
        manifest.controller_id = FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string();
        manifest.artifacts.controller_contract = artifact(
            FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND,
            FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION,
            "controller.json",
        );
        manifest.artifacts.action_projection_stream.schema_version =
            FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION;
        manifest.artifacts.rate_decision_stream.schema_version =
            FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION;
        manifest.artifacts.observation_fusion_stream.schema_version =
            FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION;
        manifest.seal().unwrap();
        manifest
    }

    #[test]
    fn all_three_streams_replay_and_detect_tampering() {
        let mut scheduler = FlagshipLeKiwiRateScheduler::new();
        let mut fuser = FlagshipLeKiwiObservationFuser::new();
        let mut actions = Vec::new();
        let mut rates = Vec::new();
        let mut observations = Vec::new();
        for sequence in 0..3 {
            let parent_action = action(sequence);
            actions.push(FlagshipActionProjectionRecord {
                projection: project_flagship_action_to_lekiwi(&parent_action).unwrap(),
                parent_action: parent_action.clone(),
            });
            rates.push(FlagshipRateDecisionRecord {
                decision: scheduler.ingest(sequence as u64, &parent_action).unwrap(),
                parent_action,
            });
            let source = inputs(sequence as u64);
            observations.push(FlagshipObservationFusionRecord {
                fusion: fuser
                    .fuse(sequence as u64, &source, &calibration())
                    .unwrap(),
                inputs: source,
            });
        }
        let action_stream = FlagshipActionProjectionStream {
            kind: FLAGSHIP_ACTION_PROJECTION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_ACTION_PROJECTION_STREAM_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID.to_string(),
            records: actions,
        };
        let rate_stream = FlagshipRateDecisionStream {
            kind: FLAGSHIP_RATE_DECISION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_RATE_DECISION_STREAM_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID.to_string(),
            records: rates,
        };
        let observation_stream = FlagshipObservationFusionStream {
            kind: FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_OBSERVATION_FUSION_STREAM_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID.to_string(),
            records: observations,
        };
        action_stream.validate().unwrap();
        rate_stream.validate().unwrap();
        observation_stream.validate(&calibration()).unwrap();

        let mut tampered = rate_stream;
        tampered.records[1].decision.physical_slot = 99;
        assert!(matches!(
            tampered.validate(),
            Err(FlagshipLeKiwiShadowError::ReplayMismatch { index: 1, .. })
        ));
    }

    #[test]
    fn v2_streams_replay_the_complete_observation_controller_projection_chain() {
        let mut fuser = FlagshipLeKiwiObservationFuserV2::new();
        let mut controller = rne_ai::FlagshipMobileLiftControllerV2::new();
        let mut scheduler = FlagshipLeKiwiRateSchedulerV2::new();
        let source = inputs_v2(0);
        let fusion = fuser.fuse(0, &source, &calibration()).unwrap();
        let controller_action = controller.next_action(&fusion.observation_values).unwrap();

        let action_stream = FlagshipActionProjectionStream {
            kind: FLAGSHIP_ACTION_PROJECTION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            records: vec![FlagshipActionProjectionRecord {
                projection: project_flagship_action_to_lekiwi_v2(&controller_action).unwrap(),
                parent_action: controller_action.clone(),
            }],
        };
        let rate_stream = FlagshipRateDecisionStream {
            kind: FLAGSHIP_RATE_DECISION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            records: vec![FlagshipRateDecisionRecord {
                decision: scheduler.ingest(0, &controller_action).unwrap(),
                parent_action: controller_action.clone(),
            }],
        };
        let observation_stream = FlagshipObservationFusionStreamV2 {
            kind: FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            records: vec![FlagshipObservationFusionRecordV2 {
                inputs: source,
                fusion,
                controller_action,
            }],
        };

        action_stream.validate().unwrap();
        rate_stream.validate().unwrap();
        observation_stream.validate(&calibration()).unwrap();
        manifest_v2().validate().unwrap();

        let mut tampered = observation_stream;
        tampered.records[0].controller_action[0] += 0.01;
        assert!(matches!(
            tampered.validate(&calibration()),
            Err(FlagshipLeKiwiShadowError::ReplayMismatch {
                stream: "controller_action_v2",
                index: 0,
            })
        ));
    }

    #[test]
    fn controller_and_source_contracts_are_strict() {
        FlagshipControllerContract::built_in().validate().unwrap();
        let mut controller = FlagshipControllerContract::built_in();
        controller.action_order.swap(0, 1);
        assert!(controller.validate().is_err());

        let source = FlagshipObservationSourceContract {
            kind: FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND.to_string(),
            schema_version: FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            role: FlagshipObservationSourceRole::Perception,
            source_id: "wrist-rgbd-v1".to_string(),
            clock_domain: "rne_sim_tick_ns".to_string(),
            max_age_ticks: 33_333_334,
            implementation_id: "calibrated-rgbd-pipeline-v1".to_string(),
        };
        source.validate().unwrap();
        let mut wall_clock = source;
        wall_clock.clock_domain = "unix_ns".to_string();
        assert!(wall_clock.validate().is_err());
    }

    #[test]
    fn manifest_separates_mock_from_physical_and_freezes_roles_and_digest() {
        let manifest = manifest();
        manifest.validate().unwrap();

        let mut relabelled = manifest.clone();
        relabelled.execution_class = FlagshipLeKiwiShadowExecutionClass::PhysicalShadow;
        relabelled.seal().unwrap();
        assert!(relabelled.validate().is_err());

        let mut wrong_role = manifest.clone();
        wrong_role.artifacts.task_spec.kind = FLAGSHIP_CONTROLLER_CONTRACT_KIND.to_string();
        wrong_role.seal().unwrap();
        assert!(wrong_role.validate().is_err());

        let mut tampered = manifest;
        tampered.sample_count += 1;
        assert_eq!(
            tampered.validate(),
            Err(FlagshipLeKiwiShadowError::DigestMismatch)
        );
    }
}
