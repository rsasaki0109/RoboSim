//! Cross-version compatibility fixture verification for RNE release artifacts.
//!
//! The suite consumes the installed `release/compatibility-fixtures.toml`
//! registry. Every registered JSON artifact must still pass its typed reader,
//! while deterministic mutations prove that an unsupported schema and an
//! unknown top-level field fail closed.

#![deny(missing_docs)]

use anyhow::{bail, ensure, Context};
use rne_accelerator_contract::{
    AcceleratorCapabilityReport, AcceleratorConformanceReport, AcceleratorManifest,
    AcceleratorRuntimeContract, ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION,
    ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
};
use rne_ai::{
    BehaviorReplayArtifact, Episode, EpisodeStep, MobileManipulatorSim,
    MobileManipulatorSimSnapshot, PortableBatchCheckpoint, PortableBatchOperation, TaskSpec,
    TaskSpecValidationError, VectorizedEpisode, VectorizedEpisodeCheckpoint,
    VectorizedEpisodeCheckpointError, VectorizedEpisodeConfig,
    MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION, MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
    TASK_SPEC_SCHEMA_VERSION, VECTORIZED_EPISODE_CHECKPOINT_VERSION,
};
use rne_core::control::{ControlCommand, RunnerControlState};
use rne_data::transport::{
    decode_control_command, decode_image_depth, decode_image_rgb8, decode_lidar_point_cloud,
    encode_control_command, encode_image_depth, encode_image_rgb8, encode_lidar_point_cloud,
    negotiate_transport, ClientHello, ControlAck, GapNotice, NegotiatedTransport,
    NegotiationPolicy, NegotiationReject, NegotiationRejectCode, SensorFrameMetadata, ServerHello,
    StatusMessage, TransportCapabilities, TransportFrame, TransportMessageKind,
    TRANSPORT_MAX_PAYLOAD_BYTES,
};
use rne_data::{
    decode_dataset_action, decode_dataset_annotation, decode_dataset_imu,
    decode_dataset_task_outcome, decode_dataset_transform, encode_dataset_action,
    encode_dataset_annotation, encode_dataset_imu, encode_dataset_task_outcome,
    encode_dataset_transform, DatasetActionSample, DatasetBundle, DatasetError,
    DatasetGroundTruthAnnotation, DatasetManifest, DatasetTaskOutcomeSample,
    DepthPairEvaluationReport, DepthPairMetricSpec, ImageDepth, ImageRgb8, ImuSample, PointCloud,
    PoseSample, RendererDatasetCaptureReport, StreamId, DATASET_BUNDLE_SCHEMA_VERSION,
    DATASET_PAYLOAD_SCHEMA_VERSION, RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION,
};
use rne_hardware_gateway::mock::MockConformanceReport;
use rne_hardware_gateway::wire::{HardwareSessionEvidence, HARDWARE_WIRE_SCHEMA_VERSION};
use rne_log::{
    FailureCapsule, FailureCapsuleError, ReplayArtifact, FAILURE_CAPSULE_SCHEMA_VERSION,
};
use rne_math::Vec3;
use rne_openscenario::{
    ScenarioReplayArtifact, ScenarioReplayArtifactError, SCENARIO_REPLAY_SCHEMA_VERSION,
};
use rne_physics_conformance::ExternalPhysicsBackendConformanceReport;
use rne_physics_conformance_suite::ConformanceReport;
use rne_plugin::{
    ControllerPluginConformanceReport, CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION,
};
use rne_plugin_sdk::{
    RneControllerStepResultV3, RneJointObservationV3, RneJointPosition, RneJointVelocity,
    RneJointVelocityV3, RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION,
    RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND, RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION,
    RNE_CONTROLLER_CAP_MULTI_ROBOT, RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION,
    RNE_PLUGIN_ABI_VERSION, RNE_PLUGIN_MIN_ABI_VERSION, RNE_PLUGIN_SDK_C_HEADER,
    RNE_PLUGIN_SDK_VERSION,
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

/// Stable compatibility report discriminator.
pub const COMPATIBILITY_FIXTURE_REPORT_KIND: &str = "rne_compatibility_fixture_report";
/// Current compatibility report schema.
pub const COMPATIBILITY_FIXTURE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Current registry schema.
pub const COMPATIBILITY_FIXTURE_REGISTRY_SCHEMA_VERSION: u32 = 1;
/// Provenance-bound historical migration fixture schema.
pub const HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION: u32 = 2;
/// Historical compatibility decision fixture schema.
pub const HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION: u32 = 1;

const MAX_FIXTURE_BYTES: u64 = 4 * 1024 * 1024;
const MAX_REGISTRY_BYTES: u64 = 256 * 1024;
const MAX_DETAIL_CHARS: usize = 240;
const FRONTEND_FAMILY_SESSION_ID: u64 = 0x1122_3344_5566_7788;
const FRONTEND_FAMILY_FIRST_SEQUENCE: u64 = 100;
const FRONTEND_MESSAGE_FAMILIES: [(&str, TransportMessageKind); 10] = [
    ("client_hello", TransportMessageKind::ClientHello),
    ("server_hello", TransportMessageKind::ServerHello),
    ("reject", TransportMessageKind::Reject),
    ("control_command", TransportMessageKind::ControlCommand),
    ("control_ack", TransportMessageKind::ControlAck),
    ("status", TransportMessageKind::Status),
    ("image_rgb8", TransportMessageKind::ImageRgb8),
    ("image_depth_f32", TransportMessageKind::ImageDepthF32),
    ("lidar_point_cloud", TransportMessageKind::LidarPointCloud),
    ("gap", TransportMessageKind::Gap),
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FixtureSpec {
    id: &'static str,
    contract: &'static str,
    schema_version: u32,
    version_field: &'static str,
}

const FIXTURE_SPECS: [FixtureSpec; 31] = [
    FixtureSpec {
        id: "accelerator_capability_v1",
        contract: "accelerator_capability",
        schema_version: ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "accelerator_conformance_v1",
        contract: "accelerator_conformance",
        schema_version: ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "behavior_replay_v1",
        contract: "behavior_replay",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "controller_c_abi_v3",
        contract: "controller_c_abi",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "controller_plugin_conformance_v1",
        contract: "controller_plugin_conformance",
        schema_version: CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "dataset_bundle_v1",
        contract: "dataset_bundle",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "dataset_bundle_v1_aecafb6",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "dataset_depth_evaluation_v1",
        contract: "dataset_offline_evaluation",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "dataset_payload_v1",
        contract: "dataset_payload",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "failure_capsule_v1",
        contract: "failure_capsule",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "failure_capsule_v1_61d6c81",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "generic_replay_v1",
        contract: "generic_replay",
        schema_version: 1,
        version_field: "version",
    },
    FixtureSpec {
        id: "frontend_message_families_v1",
        contract: "frontend_message_families",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "frontend_transport_v1",
        contract: "frontend_transport",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "frontend_transport_v1_be53f16",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "hardware_mock_conformance_v1",
        contract: "hardware_mock_conformance",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "hardware_session_evidence_v1",
        contract: "hardware_session_evidence",
        schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "mobile_manipulator_snapshot_v1_to_v3",
        contract: "historical_mobile_manipulator_snapshot",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "mobile_manipulator_snapshot_v1_47525b1_to_v3",
        contract: "historical_mobile_manipulator_snapshot_provenance",
        schema_version: HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "mobile_manipulator_snapshot_v2_2255cbe_to_v3",
        contract: "historical_mobile_manipulator_snapshot_provenance",
        schema_version: HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "physics_conformance_v2",
        contract: "physics_conformance",
        schema_version: 2,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "physics_external_conformance_v1",
        contract: "external_physics_conformance",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "portable_batch_checkpoint_v2",
        contract: "portable_batch_checkpoint",
        schema_version: 2,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "renderer_capture_report_v1",
        contract: "renderer_capture_report",
        schema_version: RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "scenario_replay_v2_533729d_requires_rerun",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "scenario_replay_v3_e959e3f_requires_rerun",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "scenario_replay_v4",
        contract: "scenario_replay",
        schema_version: 4,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "task_spec_sensor_goal_v1",
        contract: "task_spec",
        schema_version: TASK_SPEC_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "task_spec_v1",
        contract: "task_spec",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "task_spec_v1_70a9ff3",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "vectorized_episode_checkpoint_v1_bd4d44f",
        contract: "historical_artifact_decision",
        schema_version: HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        version_field: "schema_version",
    },
];

const CONTROLLER_C_ABI_LAYOUT_KIND: &str = "rne_controller_c_abi_layout";
const HISTORICAL_MIGRATION_KIND: &str = "rne_historical_migration_case";
const HISTORICAL_COMPATIBILITY_DECISION_KIND: &str = "rne_historical_compatibility_decision";
const HISTORICAL_MIGRATION_FLOAT_TOLERANCE: f64 = 1.0e-9;
const HISTORICAL_SOURCE_SCENE: &str = "assets/scenes/mm_minimal.rne.scene.toml";
const HISTORICAL_SOURCE_WORKSPACE_VERSION: &str = "0.8.0";
const HISTORICAL_SOURCE_GENERATION_STEPS: u64 = 7;
const HISTORICAL_V1_REVISION: &str = "47525b127a77cbffa9da27b1e0c127ee673aa641";
const HISTORICAL_V1_TREE: &str = "bb408cec26d34bd2a9b423dbf8b2a4d44cdf7013";
const HISTORICAL_V2_REVISION: &str = "2255cbefec9d1eb5040603fbb119a290ad855191";
const HISTORICAL_V2_TREE: &str = "373e5453c7ba94ee4efbeceb9985db4c97f5feff";
const HISTORICAL_VECTORIZED_V1_REVISION: &str = "bd4d44f5bd781fc41fd8305938001f0a858993a5";
const HISTORICAL_VECTORIZED_V1_TREE: &str = "23482add2c5d1de2978897d894d1ba745787bd06";
const HISTORICAL_SCENARIO_V2_REVISION: &str = "533729ddc78e53284eaa11d823afae18dcd110ab";
const HISTORICAL_SCENARIO_V2_TREE: &str = "b016841b2aed16bafc131f6a4698ee3b30cec34d";
const HISTORICAL_SCENARIO_V3_REVISION: &str = "e959e3ffe8426de3a8320d2d4c95e4e1438a50ad";
const HISTORICAL_SCENARIO_V3_TREE: &str = "17c6045624ccf2ed1271d19ea50926cb568ab337";
const HISTORICAL_TASK_SPEC_V1_REVISION: &str = "70a9ff35afbf0215803dd288103bdda79fa46891";
const HISTORICAL_TASK_SPEC_V1_TREE: &str = "94459bcb0c5090921bf6edbcf6f63246ebdd6a40";
const HISTORICAL_DATASET_V1_REVISION: &str = "aecafb62c99f432b2a76956575f4562c6047a6bc";
const HISTORICAL_DATASET_V1_TREE: &str = "0bc9d2d48185282da31dc80eb8857d84012a5928";
const HISTORICAL_FAILURE_CAPSULE_V1_REVISION: &str = "61d6c813e79d7eac6a8ab212776d620069f98905";
const HISTORICAL_FAILURE_CAPSULE_V1_TREE: &str = "5dac12166fe39da5a1207426f3e7520851e415d2";
const HISTORICAL_FRONTEND_V1_INTRO_REVISION: &str = "1a38391362ece24cc73c0e1470a51bd7f933e6fc";
const HISTORICAL_FRONTEND_V1_INTRO_TREE: &str = "3117bd4949f19c36a1fba66524b97bd4bd1af3d4";
const HISTORICAL_FRONTEND_V1_REVISION: &str = "be53f16347beb7df822850748d0e01ce41d227a0";
const HISTORICAL_FRONTEND_V1_TREE: &str = "78a68abd73fb4564793559d8e75e021ad5090129";
const HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST: u64 = 17_972_057_113_911_492_359;
const HISTORICAL_SCENARIO_STABLE_HASH: u64 = 8_877_782_128_690_619_681;
const HISTORICAL_SCENARIO_INPUT_DIGEST: u64 = 7_797_312_748_051_183_840;
const HISTORICAL_SCENARIO_NETWORK_DIGEST: u64 = 11_356_543_501_090_577_429;
const HISTORICAL_DATASET_SHARD_SHA256: &str =
    "sha256:cdd53532104064a6a971c10170fe05cfc9acb1b4e9be8183ed44a6b3175f17d6";
const HISTORICAL_DATASET_EVALUATION_SHA256: &str =
    "sha256:d09bf2d9079fe607373b6376c7fa4d0cbde3be4ce1787914702ccf4caa06aa16";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HistoricalSourceSpec {
    fixture_id: &'static str,
    schema_version: u32,
    revision: &'static str,
    tree: &'static str,
}

const HISTORICAL_SOURCE_SPECS: [HistoricalSourceSpec; 2] = [
    HistoricalSourceSpec {
        fixture_id: "mobile_manipulator_snapshot_v1_47525b1_to_v3",
        schema_version: 1,
        revision: HISTORICAL_V1_REVISION,
        tree: HISTORICAL_V1_TREE,
    },
    HistoricalSourceSpec {
        fixture_id: "mobile_manipulator_snapshot_v2_2255cbe_to_v3",
        schema_version: 2,
        revision: HISTORICAL_V2_REVISION,
        tree: HISTORICAL_V2_TREE,
    },
];

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum HistoricalCompatibilityOutcome {
    AcceptedAndRestored,
    RejectedRequiresRerun,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum HistoricalCompatibilityReason {
    SameSchemaFrontendTransport,
    SameSchemaReplayCheckpoint,
    SameSchemaValidatedArtifact,
    SameSchemaStreamingDatasetBundle,
    MissingRequiredReplayEvidence,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HistoricalCompatibilityFileSpec {
    path: &'static str,
    sha256: &'static str,
    size_bytes: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HistoricalCompatibilitySourceSpec {
    fixture_id: &'static str,
    artifact_contract: &'static str,
    source_schema_version: u32,
    current_schema_version: u32,
    revision: &'static str,
    tree: &'static str,
    workspace_version: &'static str,
    source_path: &'static str,
    schema_declaration: &'static str,
    source_golden_path: Option<&'static str>,
    source_golden_blob: Option<&'static str>,
    source_artifact_sha256: &'static str,
    source_files: &'static [HistoricalCompatibilityFileSpec],
    expected_outcome: HistoricalCompatibilityOutcome,
    reason_code: HistoricalCompatibilityReason,
    expected_replay_digest: Option<u64>,
    expected_error: Option<&'static str>,
    expected_result_sha256: Option<&'static str>,
    missing_required_fields: &'static [&'static str],
}

const SCENARIO_V2_MISSING_REQUIRED_FIELDS: &[&str] = &[
    "scenario_digest",
    "network_digest",
    "engine_version",
    "result.result_digest",
    "result.final_actors",
    "result.action_evidence",
    "result.unapplied_action_count",
    "result.minimum_observed_gap_m",
    "result.ownership",
];
const SCENARIO_V3_MISSING_REQUIRED_FIELDS: &[&str] = &[
    "result.result_digest",
    "result.final_actors",
    "result.action_evidence",
    "result.unapplied_action_count",
    "result.minimum_observed_gap_m",
    "result.ownership",
];
const HISTORICAL_DATASET_FILES: &[HistoricalCompatibilityFileSpec] =
    &[HistoricalCompatibilityFileSpec {
        path: "records.rnedata",
        sha256: HISTORICAL_DATASET_SHARD_SHA256,
        size_bytes: 736,
    }];

const HISTORICAL_COMPATIBILITY_SOURCE_SPECS: [HistoricalCompatibilitySourceSpec; 7] = [
    HistoricalCompatibilitySourceSpec {
        fixture_id: "dataset_bundle_v1_aecafb6",
        artifact_contract: "dataset_bundle",
        source_schema_version: 1,
        current_schema_version: DATASET_BUNDLE_SCHEMA_VERSION,
        revision: HISTORICAL_DATASET_V1_REVISION,
        tree: HISTORICAL_DATASET_V1_TREE,
        workspace_version: "0.1.0",
        source_path: "crates/rne_data/src/dataset.rs",
        schema_declaration: "DATASET_BUNDLE_SCHEMA_VERSION: u32 = 1",
        source_golden_path: Some("tests/golden/datasets/bundle-manifest-v1.json"),
        source_golden_blob: Some("785dac0449f20461fa96dde7917afbd06bdd753b"),
        source_artifact_sha256:
            "sha256:e995a3a56a44577cc6b848355f29367a88167570829661b32e32e76eb06f6832",
        source_files: HISTORICAL_DATASET_FILES,
        expected_outcome: HistoricalCompatibilityOutcome::AcceptedAndRestored,
        reason_code: HistoricalCompatibilityReason::SameSchemaStreamingDatasetBundle,
        expected_replay_digest: None,
        expected_error: None,
        expected_result_sha256: Some(HISTORICAL_DATASET_EVALUATION_SHA256),
        missing_required_fields: &[],
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "failure_capsule_v1_61d6c81",
        artifact_contract: "failure_capsule",
        source_schema_version: 1,
        current_schema_version: FAILURE_CAPSULE_SCHEMA_VERSION,
        revision: HISTORICAL_FAILURE_CAPSULE_V1_REVISION,
        tree: HISTORICAL_FAILURE_CAPSULE_V1_TREE,
        workspace_version: "0.1.0",
        source_path: "crates/rne_log/src/capsule.rs",
        schema_declaration: "FAILURE_CAPSULE_SCHEMA_VERSION: u32 = 1",
        source_golden_path: Some("tests/golden/evidence/failure-capsule-v1.json"),
        source_golden_blob: Some("c8f341d37b26f8699899cc62e40867a32616c89b"),
        source_artifact_sha256:
            "sha256:bf66f667d02795b1496481f2dfbc6c609070203d0fd98417494022ae7cda4caa",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::AcceptedAndRestored,
        reason_code: HistoricalCompatibilityReason::SameSchemaValidatedArtifact,
        expected_replay_digest: None,
        expected_error: None,
        expected_result_sha256: None,
        missing_required_fields: &[],
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "frontend_transport_v1_be53f16",
        artifact_contract: "frontend_transport",
        source_schema_version: 1,
        current_schema_version: 1,
        revision: HISTORICAL_FRONTEND_V1_REVISION,
        tree: HISTORICAL_FRONTEND_V1_TREE,
        workspace_version: "0.1.0",
        source_path: "crates/rne_data/src/transport.rs",
        schema_declaration: "TRANSPORT_PROTOCOL_MAJOR: u16 = 1",
        source_golden_path: Some("tests/golden/protocol/frontend-transport-v1.json"),
        source_golden_blob: Some("2eecf4edc03fa10c78dc950453f7adcde70bdb6a"),
        source_artifact_sha256:
            "sha256:998640e79945057bb755182009c397f3537996be583ec9f74c048fd1c6dcda71",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::AcceptedAndRestored,
        reason_code: HistoricalCompatibilityReason::SameSchemaFrontendTransport,
        expected_replay_digest: None,
        expected_error: None,
        expected_result_sha256: None,
        missing_required_fields: &[],
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "scenario_replay_v2_533729d_requires_rerun",
        artifact_contract: "scenario_replay",
        source_schema_version: 2,
        current_schema_version: SCENARIO_REPLAY_SCHEMA_VERSION,
        revision: HISTORICAL_SCENARIO_V2_REVISION,
        tree: HISTORICAL_SCENARIO_V2_TREE,
        workspace_version: "0.13.0",
        source_path: "crates/rne_openscenario/src/replay.rs",
        schema_declaration: "SCENARIO_REPLAY_SCHEMA_VERSION: u32 = 2",
        source_golden_path: None,
        source_golden_blob: None,
        source_artifact_sha256:
            "sha256:98897446879eb9d890bda6779e0b3199c3192ccb2cdc83d576ca9b24060e665e",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::RejectedRequiresRerun,
        reason_code: HistoricalCompatibilityReason::MissingRequiredReplayEvidence,
        expected_replay_digest: None,
        expected_error: Some("unsupported scenario replay schema version: expected 4, got 2"),
        expected_result_sha256: None,
        missing_required_fields: SCENARIO_V2_MISSING_REQUIRED_FIELDS,
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "scenario_replay_v3_e959e3f_requires_rerun",
        artifact_contract: "scenario_replay",
        source_schema_version: 3,
        current_schema_version: SCENARIO_REPLAY_SCHEMA_VERSION,
        revision: HISTORICAL_SCENARIO_V3_REVISION,
        tree: HISTORICAL_SCENARIO_V3_TREE,
        workspace_version: "0.13.0",
        source_path: "crates/rne_openscenario/src/replay.rs",
        schema_declaration: "SCENARIO_REPLAY_SCHEMA_VERSION: u32 = 3",
        source_golden_path: None,
        source_golden_blob: None,
        source_artifact_sha256:
            "sha256:98706249ab54e69778d149d99e4e358a233b0421c1594df0ef450b7a094b3353",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::RejectedRequiresRerun,
        reason_code: HistoricalCompatibilityReason::MissingRequiredReplayEvidence,
        expected_replay_digest: None,
        expected_error: Some("unsupported scenario replay schema version: expected 4, got 3"),
        expected_result_sha256: None,
        missing_required_fields: SCENARIO_V3_MISSING_REQUIRED_FIELDS,
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "task_spec_v1_70a9ff3",
        artifact_contract: "task_spec",
        source_schema_version: 1,
        current_schema_version: TASK_SPEC_SCHEMA_VERSION,
        revision: HISTORICAL_TASK_SPEC_V1_REVISION,
        tree: HISTORICAL_TASK_SPEC_V1_TREE,
        workspace_version: "0.1.0",
        source_path: "crates/rne_ai/src/task.rs",
        schema_declaration: "TASK_SPEC_SCHEMA_VERSION: u32 = 1",
        source_golden_path: Some("tests/golden/tasks/task-spec-v1.json"),
        source_golden_blob: Some("a2f1cd3c77f279d4f48eb029c452db9603a0c708"),
        source_artifact_sha256:
            "sha256:6c9c0e93b0d22e86ce87bb028f08e9005a008a9a6494f76acf52ebf78af48b2c",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::AcceptedAndRestored,
        reason_code: HistoricalCompatibilityReason::SameSchemaValidatedArtifact,
        expected_replay_digest: None,
        expected_error: None,
        expected_result_sha256: None,
        missing_required_fields: &[],
    },
    HistoricalCompatibilitySourceSpec {
        fixture_id: "vectorized_episode_checkpoint_v1_bd4d44f",
        artifact_contract: "vectorized_episode_checkpoint",
        source_schema_version: 1,
        current_schema_version: VECTORIZED_EPISODE_CHECKPOINT_VERSION,
        revision: HISTORICAL_VECTORIZED_V1_REVISION,
        tree: HISTORICAL_VECTORIZED_V1_TREE,
        workspace_version: "0.1.0",
        source_path: "crates/rne_ai/src/vectorized.rs",
        schema_declaration: "VECTORIZED_EPISODE_CHECKPOINT_VERSION: u32 = 1",
        source_golden_path: None,
        source_golden_blob: None,
        source_artifact_sha256:
            "sha256:f62660f376eb7d79b3f8bd2b95632780450e295fb480d3db01e567ee9579e0e2",
        source_files: &[],
        expected_outcome: HistoricalCompatibilityOutcome::AcceptedAndRestored,
        reason_code: HistoricalCompatibilityReason::SameSchemaReplayCheckpoint,
        expected_replay_digest: Some(HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST),
        expected_error: None,
        expected_result_sha256: None,
        missing_required_fields: &[],
    },
];

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum MigrationOutcome {
    AcceptedWithinTolerance,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct HistoricalMigrationFixture {
    kind: String,
    schema_version: u32,
    artifact_contract: String,
    source_schema_version: u32,
    current_schema_version: u32,
    expected_outcome: MigrationOutcome,
    float_tolerance: f64,
    source_snapshot: Value,
    current_snapshot_sha256: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct HistoricalMigrationProvenanceFixture {
    kind: String,
    schema_version: u32,
    artifact_contract: String,
    source_schema_version: u32,
    current_schema_version: u32,
    source_revision: String,
    source_tree: String,
    source_workspace_version: String,
    source_scene: String,
    generation_steps: u64,
    expected_outcome: MigrationOutcome,
    float_tolerance: f64,
    source_snapshot: Value,
    source_snapshot_sha256: String,
    current_snapshot_sha256: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct HistoricalCompatibilityDecisionFixture {
    kind: String,
    schema_version: u32,
    artifact_contract: String,
    source_schema_version: u32,
    current_schema_version: u32,
    source_revision: String,
    source_tree: String,
    source_workspace_version: String,
    expected_outcome: HistoricalCompatibilityOutcome,
    reason_code: HistoricalCompatibilityReason,
    missing_required_fields: Vec<String>,
    source_artifact: Value,
    source_artifact_sha256: String,
    #[serde(default)]
    source_files: Vec<HistoricalCompatibilityFile>,
    expected_replay_digest: Option<u64>,
    expected_error: Option<String>,
    #[serde(default)]
    expected_result_sha256: Option<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct HistoricalCompatibilityFile {
    path: String,
    size_bytes: usize,
    sha256: String,
    contents_hex: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct CAbiNamedValue {
    name: String,
    value: u64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct CAbiFieldLayout {
    name: String,
    c_type: String,
    offset_bytes: usize,
    size_bytes: usize,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct CAbiStructLayout {
    name: String,
    size_bytes: usize,
    align_bytes: usize,
    fields: Vec<CAbiFieldLayout>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct CAbiSymbol {
    name: String,
    since_abi: u32,
    c_signature: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct ControllerCAbiFixture {
    kind: String,
    schema_version: u32,
    sdk_version: u32,
    minimum_abi_version: u32,
    current_abi_version: u32,
    pointer_width_bits: u32,
    capability_bits: Vec<CAbiNamedValue>,
    structs: Vec<CAbiStructLayout>,
    symbols: Vec<CAbiSymbol>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontendHelloFixture {
    min_protocol_major: u16,
    max_protocol_major: u16,
    capabilities_bits: u64,
    required_capabilities_bits: u64,
    max_payload_bytes: u32,
    queue_frame_limit: u32,
    queue_byte_limit: u32,
    resume_after_sequence: Option<u64>,
}

impl FrontendHelloFixture {
    fn as_client_hello(self) -> ClientHello {
        ClientHello {
            min_protocol_major: self.min_protocol_major,
            max_protocol_major: self.max_protocol_major,
            capabilities: TransportCapabilities::from_bits(self.capabilities_bits),
            required_capabilities: TransportCapabilities::from_bits(
                self.required_capabilities_bits,
            ),
            max_payload_bytes: self.max_payload_bytes,
            queue_frame_limit: self.queue_frame_limit,
            queue_byte_limit: self.queue_byte_limit,
            resume_after_sequence: self.resume_after_sequence,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NegotiatedTransportFixture {
    protocol_major: u16,
    protocol_minor: u16,
    capabilities_bits: u64,
    max_payload_bytes: u32,
    queue_frame_limit: u32,
    queue_byte_limit: u32,
    resume_after_sequence: Option<u64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontendTransportFixture {
    schema_version: u32,
    message_kind: String,
    protocol_major: u16,
    protocol_minor: u16,
    flags: u16,
    sequence: u64,
    session_id: u64,
    frame_hex: String,
    hello: FrontendHelloFixture,
    negotiated: NegotiatedTransportFixture,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontendMessageFamiliesFixture {
    schema_version: u32,
    frames: Vec<FrontendMessageFrameFixture>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontendMessageFrameFixture {
    message_kind: String,
    protocol_major: u16,
    protocol_minor: u16,
    flags: u16,
    sequence: u64,
    session_id: u64,
    frame_hex: String,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct DatasetMetadataFixture {
    stream_id: u64,
    sensor_sequence: u64,
    capture_ticks: u64,
    available_ticks: u64,
}

impl DatasetMetadataFixture {
    fn as_metadata(self) -> SensorFrameMetadata {
        SensorFrameMetadata {
            stream_id: self.stream_id,
            sensor_sequence: self.sensor_sequence,
            capture_ticks: self.capture_ticks,
            available_ticks: self.available_ticks,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetImuFixture {
    angular_velocity_rad_s: [f64; 3],
    linear_acceleration_m_s2: [f64; 3],
    payload_hex: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetTransformFixture {
    position_m: [f64; 3],
    yaw_rad: f64,
    payload_hex: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetActionFixture {
    values: Vec<f64>,
    payload_hex: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetOutcomeFixture {
    episode_index: u64,
    step_in_episode: u64,
    reward: f64,
    cumulative_reward: f64,
    terminated: bool,
    truncated: bool,
    success: Option<bool>,
    payload_hex: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetAnnotationFixture {
    class_id: u32,
    instance_id: u64,
    values: Vec<f64>,
    payload_hex: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetPayloadFixture {
    schema_version: u32,
    metadata: DatasetMetadataFixture,
    imu: DatasetImuFixture,
    transform: DatasetTransformFixture,
    action: DatasetActionFixture,
    outcome: DatasetOutcomeFixture,
    annotation: DatasetAnnotationFixture,
}

/// Strict registry of compatibility fixtures shipped with a release.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CompatibilityFixtureRegistry {
    /// Registry schema version.
    pub schema_version: u32,
    /// RNE release that owns this registry snapshot.
    pub release_version: String,
    /// Exact ordered fixture list.
    #[serde(rename = "fixture")]
    pub fixtures: Vec<CompatibilityFixture>,
}

/// One typed JSON artifact retained as a compatibility fixture.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CompatibilityFixture {
    /// Stable fixture identity.
    pub id: String,
    /// Reader dispatch identity.
    pub contract: String,
    /// Schema accepted by this fixture.
    pub schema_version: u32,
    /// Forward-slash relative path under the corpus root.
    pub path: String,
    /// SHA-256 of canonical compact JSON, prefixed with `sha256:`.
    pub canonical_json_sha256: String,
}

/// Result for one accepted artifact and its two fail-closed mutations.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CompatibilityFixtureCheck {
    /// Stable fixture identity.
    pub id: String,
    /// Reader dispatch identity.
    pub contract: String,
    /// Fixture schema version.
    pub schema_version: u32,
    /// Corpus-relative fixture path.
    pub path: String,
    /// Verified canonical JSON digest.
    pub canonical_json_sha256: String,
    /// Whether the unmodified fixture passed its current typed reader.
    pub accepted: bool,
    /// Whether a deterministic future-schema mutation was rejected.
    pub future_schema_rejected: bool,
    /// Whether a deterministic unknown-field mutation was rejected.
    pub unknown_field_rejected: bool,
    /// Aggregate check verdict.
    pub passed: bool,
    /// Bounded stable diagnostic.
    pub detail: String,
}

/// Deterministic result of verifying the complete installed corpus.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CompatibilityFixtureReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Release version copied from the verified registry.
    pub release_version: String,
    /// SHA-256 of the canonical serialized registry.
    pub registry_sha256: String,
    /// Checks in registry order.
    pub checks: Vec<CompatibilityFixtureCheck>,
    /// True only when every fixture passes all three reader checks.
    pub passed: bool,
}

impl CompatibilityFixtureReport {
    /// Validates report identity, ordering, fields, and aggregate verdict.
    pub fn validate(&self, registry: &CompatibilityFixtureRegistry) -> anyhow::Result<()> {
        ensure!(
            self.kind == COMPATIBILITY_FIXTURE_REPORT_KIND,
            "compatibility report kind mismatch"
        );
        ensure!(
            self.schema_version == COMPATIBILITY_FIXTURE_REPORT_SCHEMA_VERSION,
            "compatibility report schema mismatch"
        );
        ensure!(
            self.release_version == registry.release_version,
            "compatibility report release mismatch"
        );
        ensure!(
            self.registry_sha256 == registry_digest(registry)?,
            "compatibility registry digest mismatch"
        );
        ensure!(
            self.checks.len() == registry.fixtures.len(),
            "compatibility report check count mismatch"
        );
        for (check, fixture) in self.checks.iter().zip(&registry.fixtures) {
            ensure!(
                check.id == fixture.id
                    && check.contract == fixture.contract
                    && check.schema_version == fixture.schema_version
                    && check.path == fixture.path
                    && check.canonical_json_sha256 == fixture.canonical_json_sha256,
                "compatibility report check identity mismatch for {}",
                fixture.id
            );
            ensure!(
                check.passed
                    == (check.accepted
                        && check.future_schema_rejected
                        && check.unknown_field_rejected),
                "compatibility report verdict mismatch for {}",
                fixture.id
            );
            ensure!(
                check.detail.chars().count() <= MAX_DETAIL_CHARS,
                "compatibility report detail is unbounded for {}",
                fixture.id
            );
        }
        ensure!(
            self.passed == self.checks.iter().all(|check| check.passed),
            "compatibility report aggregate mismatch"
        );
        Ok(())
    }
}

/// Reads and strictly validates a compatibility registry.
pub fn read_registry(path: &Path) -> anyhow::Result<CompatibilityFixtureRegistry> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect compatibility registry {}", path.display()))?;
    ensure!(
        metadata.file_type().is_file(),
        "compatibility registry must be a regular file"
    );
    ensure!(
        metadata.len() <= MAX_REGISTRY_BYTES,
        "compatibility registry exceeds {} bytes",
        MAX_REGISTRY_BYTES
    );
    let bytes = fs::read(path)
        .with_context(|| format!("read compatibility registry {}", path.display()))?;
    let text = std::str::from_utf8(&bytes).context("compatibility registry is not UTF-8")?;
    let registry: CompatibilityFixtureRegistry = toml::from_str(text)
        .with_context(|| format!("parse compatibility registry {}", path.display()))?;
    validate_registry(&registry)?;
    Ok(registry)
}

/// Runs every typed compatibility fixture and both fail-closed mutations.
pub fn run_compatibility(
    root: &Path,
    registry_path: &Path,
) -> anyhow::Result<CompatibilityFixtureReport> {
    let root = root
        .canonicalize()
        .with_context(|| format!("canonicalize corpus root {}", root.display()))?;
    ensure!(
        root.is_dir(),
        "compatibility corpus root is not a directory"
    );
    let registry = read_registry(registry_path)?;
    let checks = registry
        .fixtures
        .iter()
        .zip(FIXTURE_SPECS)
        .map(|(fixture, spec)| check_fixture(&root, fixture, spec))
        .collect::<Vec<_>>();
    let passed = checks.iter().all(|check| check.passed);
    let report = CompatibilityFixtureReport {
        kind: COMPATIBILITY_FIXTURE_REPORT_KIND.to_string(),
        schema_version: COMPATIBILITY_FIXTURE_REPORT_SCHEMA_VERSION,
        release_version: registry.release_version.clone(),
        registry_sha256: registry_digest(&registry)?,
        checks,
        passed,
    };
    report.validate(&registry)?;
    Ok(report)
}

/// Verifies that provenance-bound historical fixtures still reference the
/// exact ancestor commits and Git trees that emitted their source schemas.
///
/// This source-checkout gate is intentionally separate from
/// [`run_compatibility`], because an extracted native bundle contains the
/// content-addressed fixtures but not the repository's Git object database.
pub fn verify_historical_source_history(root: &Path) -> anyhow::Result<()> {
    for source in HISTORICAL_SOURCE_SPECS {
        git_text(
            root,
            &["cat-file", "-e", &format!("{}^{{commit}}", source.revision)],
        )?;
        let actual_tree = git_text(root, &["show", "-s", "--format=%T", source.revision])?;
        ensure!(
            actual_tree.trim() == source.tree,
            "historical source tree mismatch for {}: expected {}, got {}",
            source.fixture_id,
            source.tree,
            actual_tree.trim()
        );
        git_text(
            root,
            &["merge-base", "--is-ancestor", source.revision, "HEAD"],
        )?;

        let cargo_toml = git_text(root, &["show", &format!("{}:Cargo.toml", source.revision)])?;
        ensure!(
            cargo_toml.contains(&format!(
                "version = \"{HISTORICAL_SOURCE_WORKSPACE_VERSION}\""
            )),
            "historical source workspace version mismatch for {}",
            source.fixture_id
        );
        let sim_source = git_text(
            root,
            &[
                "show",
                &format!(
                    "{}:crates/rne_ai/src/env/mobile_manipulator/sim.rs",
                    source.revision
                ),
            ],
        )?;
        ensure!(
            sim_source.contains(&format!(
                "MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION: u32 = {}",
                source.schema_version
            )),
            "historical source schema declaration mismatch for {}",
            source.fixture_id
        );
        git_text(
            root,
            &[
                "cat-file",
                "-e",
                &format!("{}:{HISTORICAL_SOURCE_SCENE}", source.revision),
            ],
        )?;
    }

    git_text(
        root,
        &[
            "cat-file",
            "-e",
            &format!("{HISTORICAL_FRONTEND_V1_INTRO_REVISION}^{{commit}}"),
        ],
    )?;
    let frontend_intro_tree = git_text(
        root,
        &[
            "show",
            "-s",
            "--format=%T",
            HISTORICAL_FRONTEND_V1_INTRO_REVISION,
        ],
    )?;
    ensure!(
        frontend_intro_tree.trim() == HISTORICAL_FRONTEND_V1_INTRO_TREE,
        "historical frontend transport introduction tree mismatch"
    );
    git_text(
        root,
        &[
            "merge-base",
            "--is-ancestor",
            HISTORICAL_FRONTEND_V1_INTRO_REVISION,
            "HEAD",
        ],
    )?;
    let frontend_intro_manifest = git_text(
        root,
        &[
            "show",
            &format!("{HISTORICAL_FRONTEND_V1_INTRO_REVISION}:Cargo.toml"),
        ],
    )?;
    ensure!(
        frontend_intro_manifest.contains("version = \"0.14.0-rc.1\""),
        "historical frontend transport introduction workspace version mismatch"
    );
    let frontend_intro_source = git_text(
        root,
        &[
            "show",
            &format!("{HISTORICAL_FRONTEND_V1_INTRO_REVISION}:crates/rne_data/src/transport.rs"),
        ],
    )?;
    for marker in [
        "TRANSPORT_PROTOCOL_MAJOR: u16 = 1",
        "TRANSPORT_PROTOCOL_MINOR: u16 = 0",
        "frame_header_has_platform_independent_golden_bytes",
    ] {
        ensure!(
            frontend_intro_source.contains(marker),
            "historical frontend transport introduction omitted marker {marker}"
        );
    }

    for source in HISTORICAL_COMPATIBILITY_SOURCE_SPECS {
        git_text(
            root,
            &["cat-file", "-e", &format!("{}^{{commit}}", source.revision)],
        )?;
        let actual_tree = git_text(root, &["show", "-s", "--format=%T", source.revision])?;
        ensure!(
            actual_tree.trim() == source.tree,
            "historical compatibility source tree mismatch for {}: expected {}, got {}",
            source.fixture_id,
            source.tree,
            actual_tree.trim()
        );
        git_text(
            root,
            &["merge-base", "--is-ancestor", source.revision, "HEAD"],
        )?;
        let cargo_toml = git_text(root, &["show", &format!("{}:Cargo.toml", source.revision)])?;
        ensure!(
            cargo_toml.contains(&format!("version = \"{}\"", source.workspace_version)),
            "historical compatibility workspace version mismatch for {}",
            source.fixture_id
        );
        let source_text = git_text(
            root,
            &[
                "show",
                &format!("{}:{}", source.revision, source.source_path),
            ],
        )?;
        ensure!(
            source_text.contains(source.schema_declaration),
            "historical compatibility schema declaration mismatch for {}",
            source.fixture_id
        );
        if let Some(path) = source.source_golden_path {
            let expected_blob = source
                .source_golden_blob
                .context("historical source golden omitted expected blob")?;
            let source_blob =
                git_text(root, &["rev-parse", &format!("{}:{path}", source.revision)])?;
            let current_blob = git_text(root, &["rev-parse", &format!("HEAD:{path}")])?;
            ensure!(
                source_blob.trim() == expected_blob && current_blob.trim() == expected_blob,
                "historical source/current golden blob mismatch for {}",
                source.fixture_id
            );
            let source_golden = git_text(root, &["show", &format!("{}:{path}", source.revision)])?;
            let source_value: Value = serde_json::from_str(&source_golden).with_context(|| {
                format!("parse historical source golden for {}", source.fixture_id)
            })?;
            ensure!(
                canonical_json_digest(&source_value)? == source.source_artifact_sha256,
                "historical source golden digest mismatch for {}",
                source.fixture_id
            );
        }
        if source.artifact_contract == "scenario_replay" {
            for path in [
                "assets/scenarios/speed.xosc",
                "assets/traffic/corridor.rne.traffic.json",
            ] {
                git_text(
                    root,
                    &["cat-file", "-e", &format!("{}:{path}", source.revision)],
                )?;
            }
        } else if source.artifact_contract == "dataset_bundle" {
            let tests = git_text(
                root,
                &[
                    "show",
                    &format!(
                        "{}:crates/rne_data/tests/dataset_bundle.rs",
                        source.revision
                    ),
                ],
            )?;
            for marker in [
                "fn write_reference_bundle_at",
                "writer.write_gap(StreamId::new(10), 1, 1, 10, 12)",
                "evaluate_depth_pair(DepthPairMetricSpec",
            ] {
                ensure!(
                    tests.contains(marker),
                    "historical dataset generation recipe mismatch for {}",
                    source.fixture_id
                );
            }
        }
    }
    Ok(())
}

/// Writes a validated compatibility report as stable pretty JSON.
pub fn write_report(report: &CompatibilityFixtureReport, path: &Path) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create compatibility output {}", parent.display()))?;
    }
    let mut bytes = serde_json::to_vec_pretty(report)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write compatibility report {}", path.display()))
}

fn validate_registry(registry: &CompatibilityFixtureRegistry) -> anyhow::Result<()> {
    ensure!(
        registry.schema_version == COMPATIBILITY_FIXTURE_REGISTRY_SCHEMA_VERSION,
        "compatibility registry schema must be {}",
        COMPATIBILITY_FIXTURE_REGISTRY_SCHEMA_VERSION
    );
    ensure!(
        !registry.release_version.trim().is_empty(),
        "compatibility registry release_version must not be empty"
    );
    ensure!(
        registry.fixtures.len() == FIXTURE_SPECS.len(),
        "compatibility registry must contain exactly {} fixtures",
        FIXTURE_SPECS.len()
    );
    let mut paths = BTreeSet::new();
    for (fixture, spec) in registry.fixtures.iter().zip(FIXTURE_SPECS) {
        ensure!(
            fixture.id == spec.id
                && fixture.contract == spec.contract
                && fixture.schema_version == spec.schema_version,
            "compatibility fixture identity/order mismatch: expected {}",
            spec.id
        );
        validate_relative_path(&fixture.path)?;
        ensure!(
            paths.insert(fixture.path.as_str()),
            "duplicate compatibility fixture path {}",
            fixture.path
        );
        validate_sha256(&fixture.canonical_json_sha256)?;
    }
    Ok(())
}

fn check_fixture(
    root: &Path,
    fixture: &CompatibilityFixture,
    spec: FixtureSpec,
) -> CompatibilityFixtureCheck {
    match try_check_fixture(root, fixture, spec) {
        Ok((accepted, future_schema_rejected, unknown_field_rejected)) => {
            let passed = accepted && future_schema_rejected && unknown_field_rejected;
            CompatibilityFixtureCheck {
                id: fixture.id.clone(),
                contract: fixture.contract.clone(),
                schema_version: fixture.schema_version,
                path: fixture.path.clone(),
                canonical_json_sha256: fixture.canonical_json_sha256.clone(),
                accepted,
                future_schema_rejected,
                unknown_field_rejected,
                passed,
                detail: if passed {
                    if spec
                        .contract
                        .starts_with("historical_mobile_manipulator_snapshot")
                    {
                        "accepted within 1e-9; future schema and unknown field rejected".to_string()
                    } else if spec.contract == "historical_artifact_decision" {
                        "historical decision verified; future schema and unknown field rejected"
                            .to_string()
                    } else {
                        "accepted; future schema and unknown field rejected".to_string()
                    }
                } else {
                    "typed reader did not satisfy every compatibility expectation".to_string()
                },
            }
        }
        Err(error) => CompatibilityFixtureCheck {
            id: fixture.id.clone(),
            contract: fixture.contract.clone(),
            schema_version: fixture.schema_version,
            path: fixture.path.clone(),
            canonical_json_sha256: fixture.canonical_json_sha256.clone(),
            accepted: false,
            future_schema_rejected: false,
            unknown_field_rejected: false,
            passed: false,
            detail: bounded_detail(&format!("{error:#}")),
        },
    }
}

fn try_check_fixture(
    root: &Path,
    fixture: &CompatibilityFixture,
    spec: FixtureSpec,
) -> anyhow::Result<(bool, bool, bool)> {
    let path = resolve_fixture(root, &fixture.path)?;
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("inspect compatibility fixture {}", path.display()))?;
    ensure!(
        metadata.file_type().is_file(),
        "fixture is not a regular file"
    );
    ensure!(
        metadata.len() <= MAX_FIXTURE_BYTES,
        "fixture exceeds {} bytes",
        MAX_FIXTURE_BYTES
    );
    let bytes = fs::read(&path)
        .with_context(|| format!("read compatibility fixture {}", path.display()))?;
    let value: Value = serde_json::from_slice(&bytes)
        .with_context(|| format!("parse compatibility fixture {}", path.display()))?;
    let digest = canonical_json_digest(&value)?;
    ensure!(
        digest == fixture.canonical_json_sha256,
        "canonical JSON digest mismatch: expected {}, got {}",
        fixture.canonical_json_sha256,
        digest
    );

    validate_typed_input(root, spec, value.clone(), &bytes)
        .context("accepted fixture was rejected")?;

    let mut future = value.clone();
    let future_object = future
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    future_object.insert(
        spec.version_field.to_string(),
        Value::from(u64::from(spec.schema_version) + 10_000),
    );
    let future_bytes = serde_json::to_vec(&future)?;
    let future_schema_rejected = validate_typed_input(root, spec, future, &future_bytes).is_err();

    let mut unknown = value;
    let unknown_object = unknown
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    unknown_object.insert(
        "rne_unknown_compatibility_field".to_string(),
        Value::Bool(true),
    );
    let unknown_bytes = serde_json::to_vec(&unknown)?;
    let unknown_field_rejected = validate_typed_input(root, spec, unknown, &unknown_bytes).is_err();
    Ok((true, future_schema_rejected, unknown_field_rejected))
}

fn validate_typed_input(
    root: &Path,
    spec: FixtureSpec,
    value: Value,
    bytes: &[u8],
) -> anyhow::Result<()> {
    if spec.contract != "accelerator_conformance" {
        return validate_typed(root, spec, value);
    }
    let actual_schema = value
        .get(spec.version_field)
        .and_then(Value::as_u64)
        .context("fixture omitted its integer version field")?;
    ensure!(
        actual_schema == u64::from(spec.schema_version),
        "unsupported {} schema: expected {}, got {}",
        spec.contract,
        spec.schema_version,
        actual_schema
    );
    let fixture = AcceleratorConformanceReport::from_json_slice(bytes)?;
    let manifest: AcceleratorManifest = toml::from_str(&fs::read_to_string(
        root.join("adapters/mjx/accelerator.toml"),
    )?)?;
    let runtime: AcceleratorRuntimeContract =
        toml::from_str(&fs::read_to_string(root.join("adapters/mjx/runtime.toml"))?)?;
    let task: TaskSpec = serde_json::from_slice(&fs::read(
        root.join("adapters/mjx/fixtures/free-fall-task-spec-v1.json"),
    )?)?;
    let model = fs::read(root.join("adapters/mjx/fixtures/free-fall-v1.xml"))?;
    fixture.validate_against(&manifest, &runtime, &task, &model)?;
    Ok(())
}

fn validate_typed(root: &Path, spec: FixtureSpec, value: Value) -> anyhow::Result<()> {
    let actual_schema = value
        .get(spec.version_field)
        .and_then(Value::as_u64)
        .context("fixture omitted its integer version field")?;
    ensure!(
        actual_schema == u64::from(spec.schema_version),
        "unsupported {} schema: expected {}, got {}",
        spec.contract,
        spec.schema_version,
        actual_schema
    );
    match spec.contract {
        "accelerator_capability" => {
            let fixture: AcceleratorCapabilityReport = serde_json::from_value(value)?;
            let manifest: AcceleratorManifest = toml::from_str(&fs::read_to_string(
                root.join("adapters/mjx/accelerator.toml"),
            )?)?;
            let runtime: AcceleratorRuntimeContract =
                toml::from_str(&fs::read_to_string(root.join("adapters/mjx/runtime.toml"))?)?;
            let task: TaskSpec = serde_json::from_slice(&fs::read(
                root.join("adapters/mjx/fixtures/free-fall-task-spec-v1.json"),
            )?)?;
            fixture.validate_against(&manifest, &runtime, &task)?;
        }
        "behavior_replay" => {
            let fixture: BehaviorReplayArtifact = serde_json::from_value(value)?;
            fixture.validate_compatibility()?;
        }
        "controller_c_abi" => {
            let fixture: ControllerCAbiFixture = serde_json::from_value(value)?;
            validate_controller_c_abi(&fixture)?;
        }
        "controller_plugin_conformance" => {
            let fixture: ControllerPluginConformanceReport = serde_json::from_value(value)?;
            fixture.validate()?;
            ensure!(
                fixture.passed(),
                "controller plugin conformance fixture failed"
            );
        }
        "dataset_bundle" => {
            let fixture: DatasetManifest = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "dataset_offline_evaluation" => {
            let fixture: DepthPairEvaluationReport = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "dataset_payload" => {
            let fixture: DatasetPayloadFixture = serde_json::from_value(value)?;
            validate_dataset_payload(&fixture)?;
        }
        "failure_capsule" => {
            let fixture: FailureCapsule = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "generic_replay" => {
            let fixture: ReplayArtifact = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "frontend_message_families" => {
            let fixture: FrontendMessageFamiliesFixture = serde_json::from_value(value)?;
            validate_frontend_message_families(&fixture)?;
        }
        "frontend_transport" => {
            let fixture: FrontendTransportFixture = serde_json::from_value(value)?;
            validate_frontend_transport(&fixture)?;
        }
        "hardware_mock_conformance" => {
            let fixture: MockConformanceReport = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "hardware_session_evidence" => {
            let fixture: HardwareSessionEvidence = serde_json::from_value(value)?;
            let task: TaskSpec = serde_json::from_slice(&fs::read(
                root.join("assets/tasks/diff_drive_goal.task.json"),
            )?)?;
            fixture.validate_against(&task)?;
        }
        "historical_mobile_manipulator_snapshot" => {
            let fixture: HistoricalMigrationFixture = serde_json::from_value(value)?;
            validate_historical_mobile_manipulator_snapshot(root, &fixture)?;
        }
        "historical_mobile_manipulator_snapshot_provenance" => {
            let fixture: HistoricalMigrationProvenanceFixture = serde_json::from_value(value)?;
            validate_historical_mobile_manipulator_snapshot_provenance(root, spec.id, &fixture)?;
        }
        "historical_artifact_decision" => {
            let fixture: HistoricalCompatibilityDecisionFixture = serde_json::from_value(value)?;
            validate_historical_compatibility_decision(spec.id, &fixture)?;
        }
        "physics_conformance" => {
            let fixture: ConformanceReport = serde_json::from_value(value)?;
            ensure!(fixture.all_passed(), "physics conformance fixture failed");
        }
        "external_physics_conformance" => {
            let fixture: ExternalPhysicsBackendConformanceReport = serde_json::from_value(value)?;
            fixture.validate()?;
            ensure!(fixture.passed(), "external physics fixture failed");
        }
        "portable_batch_checkpoint" => {
            let fixture: PortableBatchCheckpoint<u64> = serde_json::from_value(value)?;
            validate_checkpoint(&fixture)?;
        }
        "renderer_capture_report" => {
            let fixture: RendererDatasetCaptureReport = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "scenario_replay" => {
            let fixture: ScenarioReplayArtifact = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "task_spec" => {
            let fixture: TaskSpec = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        other => bail!("unsupported compatibility contract {other}"),
    }
    Ok(())
}

fn validate_controller_c_abi(fixture: &ControllerCAbiFixture) -> anyhow::Result<()> {
    ensure!(
        fixture.kind == CONTROLLER_C_ABI_LAYOUT_KIND,
        "controller C ABI fixture kind mismatch"
    );
    ensure!(
        fixture.schema_version == RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION,
        "controller C ABI schema mismatch"
    );
    ensure!(
        fixture == &current_controller_c_abi(),
        "controller C ABI constants, layout, or symbols changed"
    );
    for symbol in &fixture.symbols {
        ensure!(
            RNE_PLUGIN_SDK_C_HEADER.contains(&symbol.name),
            "controller C header omitted symbol {}",
            symbol.name
        );
    }
    for layout in &fixture.structs {
        ensure!(
            RNE_PLUGIN_SDK_C_HEADER.contains(&layout.name),
            "controller C header omitted structure {}",
            layout.name
        );
    }
    Ok(())
}

fn validate_historical_mobile_manipulator_snapshot(
    root: &Path,
    fixture: &HistoricalMigrationFixture,
) -> anyhow::Result<()> {
    ensure!(
        fixture.kind == HISTORICAL_MIGRATION_KIND,
        "historical migration fixture kind mismatch"
    );
    ensure!(
        fixture.schema_version == 1,
        "historical migration case schema mismatch"
    );
    ensure!(
        fixture.artifact_contract == "mobile_manipulator_sim_snapshot",
        "historical migration artifact contract mismatch"
    );
    ensure!(
        fixture.source_schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION,
        "historical snapshot source schema mismatch"
    );
    ensure!(
        fixture.current_schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "historical snapshot current schema mismatch"
    );
    ensure!(
        fixture.expected_outcome == MigrationOutcome::AcceptedWithinTolerance,
        "historical snapshot outcome mismatch"
    );
    ensure!(
        fixture.float_tolerance == HISTORICAL_MIGRATION_FLOAT_TOLERANCE,
        "historical snapshot tolerance mismatch"
    );
    validate_sha256(&fixture.current_snapshot_sha256)?;

    let source_object = fixture
        .source_snapshot
        .as_object()
        .context("historical source snapshot must be an object")?;
    ensure!(
        !source_object.contains_key("wrist_depth_frame")
            && !source_object.contains_key("grasp_retarget"),
        "schema-v1 source must omit fields introduced in v2 and v3"
    );
    let snapshot: MobileManipulatorSimSnapshot =
        serde_json::from_value(fixture.source_snapshot.clone())?;
    ensure!(
        snapshot.schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION,
        "historical source payload schema mismatch"
    );

    let scene = root.join("assets/scenes/mm_minimal.rne.scene.toml");
    let mut sim = MobileManipulatorSim::from_scene_path(&scene)
        .with_context(|| format!("load historical migration scene {}", scene.display()))?;
    sim.restore_snapshot(&snapshot)
        .map_err(|error| anyhow::anyhow!("restore schema-v1 snapshot: {error:?}"))?;
    let current_snapshot = sim.snapshot();
    ensure!(
        current_snapshot.schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "restored snapshot did not normalize to the current schema"
    );
    let mut expected_snapshot = snapshot.clone();
    expected_snapshot.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION;
    let expected_current = serde_json::to_value(&expected_snapshot)?;
    let current = serde_json::to_value(&current_snapshot)?;
    let normalized_expected = canonical_state_value(&expected_current);
    let normalized_current = canonical_state_value(&current);
    ensure!(
        normalized_current == normalized_expected,
        "historical snapshot restore exceeded tolerance: {}",
        first_json_difference(&normalized_expected, &normalized_current, "snapshot")
    );
    let current_digest = normalized_state_digest(&current)?;
    ensure!(
        current_digest == fixture.current_snapshot_sha256,
        "historical snapshot expected-state digest changed: expected {}, got {}",
        fixture.current_snapshot_sha256,
        current_digest
    );

    let mut unsupported = snapshot.clone();
    unsupported.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION + 1;
    ensure!(
        sim.restore_snapshot(&unsupported).is_err(),
        "snapshot reader accepted an unsupported future schema"
    );
    let mut unknown_source = fixture.source_snapshot.clone();
    unknown_source
        .as_object_mut()
        .context("historical source snapshot must be an object")?
        .insert("unknown_future_state".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<MobileManipulatorSimSnapshot>(unknown_source).is_err(),
        "snapshot reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_mobile_manipulator_snapshot_provenance(
    root: &Path,
    fixture_id: &str,
    fixture: &HistoricalMigrationProvenanceFixture,
) -> anyhow::Result<()> {
    let source = HISTORICAL_SOURCE_SPECS
        .iter()
        .find(|source| source.fixture_id == fixture_id)
        .context("unknown provenance-bound historical migration fixture")?;
    ensure!(
        fixture.kind == HISTORICAL_MIGRATION_KIND,
        "historical migration fixture kind mismatch"
    );
    ensure!(
        fixture.schema_version == HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
        "historical migration provenance schema mismatch"
    );
    ensure!(
        fixture.artifact_contract == "mobile_manipulator_sim_snapshot",
        "historical migration artifact contract mismatch"
    );
    ensure!(
        fixture.source_schema_version == source.schema_version,
        "historical snapshot source schema mismatch"
    );
    ensure!(
        fixture.current_schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "historical snapshot current schema mismatch"
    );
    ensure!(
        fixture.source_revision == source.revision && fixture.source_tree == source.tree,
        "historical snapshot source revision/tree mismatch"
    );
    ensure!(
        fixture.source_workspace_version == HISTORICAL_SOURCE_WORKSPACE_VERSION,
        "historical snapshot source workspace version mismatch"
    );
    ensure!(
        fixture.source_scene == HISTORICAL_SOURCE_SCENE,
        "historical snapshot source scene mismatch"
    );
    ensure!(
        fixture.generation_steps == HISTORICAL_SOURCE_GENERATION_STEPS,
        "historical snapshot generation step count mismatch"
    );
    ensure!(
        fixture.expected_outcome == MigrationOutcome::AcceptedWithinTolerance,
        "historical snapshot outcome mismatch"
    );
    ensure!(
        fixture.float_tolerance == HISTORICAL_MIGRATION_FLOAT_TOLERANCE,
        "historical snapshot tolerance mismatch"
    );
    validate_sha256(&fixture.source_snapshot_sha256)?;
    validate_sha256(&fixture.current_snapshot_sha256)?;
    ensure!(
        sha256(&serde_json::to_vec(&fixture.source_snapshot)?) == fixture.source_snapshot_sha256,
        "historical source snapshot digest mismatch"
    );

    let source_object = fixture
        .source_snapshot
        .as_object()
        .context("historical source snapshot must be an object")?;
    ensure!(
        !source_object.contains_key("grasp_retarget"),
        "pre-v3 source must omit grasp_retarget"
    );
    match source.schema_version {
        1 => ensure!(
            !source_object.contains_key("wrist_depth_frame"),
            "schema-v1 source must omit wrist_depth_frame"
        ),
        2 => ensure!(
            source_object
                .get("wrist_depth_frame")
                .is_some_and(|value| !value.is_null()),
            "schema-v2 source must retain a populated wrist_depth_frame"
        ),
        other => bail!("unsupported historical source schema {other}"),
    }

    let snapshot: MobileManipulatorSimSnapshot =
        serde_json::from_value(fixture.source_snapshot.clone())?;
    ensure!(
        snapshot.schema_version == source.schema_version,
        "historical source payload schema mismatch"
    );
    ensure!(
        snapshot.step_count == HISTORICAL_SOURCE_GENERATION_STEPS
            && snapshot.sim_ticks > 0
            && snapshot.joint_state_frame.is_some()
            && snapshot.wrist_camera_frame.is_some(),
        "historical source is not the expected nonzero sensor-bearing snapshot"
    );

    let scene = root.join(HISTORICAL_SOURCE_SCENE);
    let mut sim = MobileManipulatorSim::from_scene_path(&scene)
        .with_context(|| format!("load historical migration scene {}", scene.display()))?;
    sim.restore_snapshot(&snapshot).map_err(|error| {
        anyhow::anyhow!(
            "restore schema-v{} historical snapshot: {error:?}",
            source.schema_version
        )
    })?;
    let current_snapshot = sim.snapshot();
    ensure!(
        current_snapshot.schema_version == MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "restored snapshot did not normalize to the current schema"
    );
    let mut expected_snapshot = snapshot.clone();
    expected_snapshot.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION;
    let expected_current = serde_json::to_value(&expected_snapshot)?;
    let current = serde_json::to_value(&current_snapshot)?;
    let normalized_expected = canonical_state_value(&expected_current);
    let normalized_current = canonical_state_value(&current);
    ensure!(
        normalized_current == normalized_expected,
        "historical snapshot restore exceeded tolerance: {}",
        first_json_difference(&normalized_expected, &normalized_current, "snapshot")
    );
    let current_digest = normalized_state_digest(&current)?;
    ensure!(
        current_digest == fixture.current_snapshot_sha256,
        "historical snapshot expected-state digest changed: expected {}, got {}",
        fixture.current_snapshot_sha256,
        current_digest
    );

    let mut unsupported = snapshot.clone();
    unsupported.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION + 1;
    ensure!(
        sim.restore_snapshot(&unsupported).is_err(),
        "snapshot reader accepted an unsupported future schema"
    );
    let mut unknown_source = fixture.source_snapshot.clone();
    unknown_source
        .as_object_mut()
        .context("historical source snapshot must be an object")?
        .insert("unknown_future_state".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<MobileManipulatorSimSnapshot>(unknown_source).is_err(),
        "snapshot reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_compatibility_decision(
    fixture_id: &str,
    fixture: &HistoricalCompatibilityDecisionFixture,
) -> anyhow::Result<()> {
    let source = HISTORICAL_COMPATIBILITY_SOURCE_SPECS
        .iter()
        .find(|source| source.fixture_id == fixture_id)
        .context("unknown historical compatibility decision fixture")?;
    ensure!(
        fixture.kind == HISTORICAL_COMPATIBILITY_DECISION_KIND,
        "historical compatibility decision kind mismatch"
    );
    ensure!(
        fixture.schema_version == HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        "historical compatibility decision schema mismatch"
    );
    ensure!(
        fixture.artifact_contract == source.artifact_contract
            && fixture.source_schema_version == source.source_schema_version
            && fixture.current_schema_version == source.current_schema_version,
        "historical compatibility contract/schema mismatch"
    );
    ensure!(
        fixture.source_revision == source.revision
            && fixture.source_tree == source.tree
            && fixture.source_workspace_version == source.workspace_version,
        "historical compatibility source provenance mismatch"
    );
    ensure!(
        fixture.expected_outcome == source.expected_outcome
            && fixture.reason_code == source.reason_code,
        "historical compatibility decision outcome/reason mismatch"
    );
    ensure!(
        fixture.expected_replay_digest == source.expected_replay_digest
            && fixture.expected_error.as_deref() == source.expected_error
            && fixture.expected_result_sha256.as_deref() == source.expected_result_sha256,
        "historical compatibility expected result mismatch"
    );
    let expected_missing = source
        .missing_required_fields
        .iter()
        .map(|field| (*field).to_string())
        .collect::<Vec<_>>();
    ensure!(
        fixture.missing_required_fields == expected_missing,
        "historical compatibility missing-field decision mismatch"
    );
    validate_sha256(&fixture.source_artifact_sha256)?;
    let source_digest = canonical_json_digest(&fixture.source_artifact)?;
    ensure!(
        source_digest == fixture.source_artifact_sha256
            && fixture.source_artifact_sha256 == source.source_artifact_sha256,
        "historical compatibility source artifact digest mismatch: expected {}, got {}",
        source.source_artifact_sha256,
        source_digest
    );
    ensure!(
        fixture.source_files.len() == source.source_files.len(),
        "historical compatibility source file count mismatch"
    );
    for (file, expected) in fixture.source_files.iter().zip(source.source_files) {
        validate_relative_path(&file.path)?;
        validate_sha256(&file.sha256)?;
        let contents = decode_lower_hex(&file.contents_hex)?;
        ensure!(
            file.path == expected.path
                && file.size_bytes == expected.size_bytes
                && file.sha256 == expected.sha256
                && contents.len() == expected.size_bytes
                && sha256(&contents) == expected.sha256,
            "historical compatibility source file mismatch for {}",
            file.path
        );
    }
    ensure!(
        fixture
            .source_artifact
            .get("schema_version")
            .and_then(Value::as_u64)
            == Some(u64::from(source.source_schema_version)),
        "historical compatibility source payload schema mismatch"
    );

    match source.artifact_contract {
        "dataset_bundle" => validate_historical_dataset_bundle(fixture, source),
        "failure_capsule" => validate_historical_failure_capsule(fixture, source),
        "frontend_transport" => validate_historical_frontend_transport(fixture, source),
        "task_spec" => validate_historical_task_spec(fixture, source),
        "vectorized_episode_checkpoint" => {
            validate_historical_vectorized_checkpoint(fixture, source)
        }
        "scenario_replay" => validate_historical_scenario_replay(fixture, source),
        other => bail!("unsupported historical compatibility contract {other}"),
    }
}

fn validate_historical_frontend_transport(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::AcceptedAndRestored
            && source.reason_code == HistoricalCompatibilityReason::SameSchemaFrontendTransport
            && fixture.source_files.is_empty(),
        "frontend transport decision must retain the exact same-schema wire artifact"
    );
    let transport: FrontendTransportFixture =
        serde_json::from_value(fixture.source_artifact.clone())?;
    validate_frontend_transport(&transport)?;

    let mut future = fixture.source_artifact.clone();
    future["schema_version"] = Value::from(2_u32);
    let future: FrontendTransportFixture = serde_json::from_value(future)?;
    ensure!(
        validate_frontend_transport(&future).is_err(),
        "frontend transport reader accepted an unsupported fixture schema"
    );
    let mut unknown = fixture.source_artifact.clone();
    unknown
        .as_object_mut()
        .context("historical frontend transport fixture must be an object")?
        .insert("unknown_future_frame".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<FrontendTransportFixture>(unknown).is_err(),
        "frontend transport reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_task_spec(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::AcceptedAndRestored
            && source.reason_code == HistoricalCompatibilityReason::SameSchemaValidatedArtifact
            && fixture.source_files.is_empty(),
        "TaskSpec decision must retain the exact same-schema artifact"
    );
    let task: TaskSpec = serde_json::from_value(fixture.source_artifact.clone())?;
    task.validate()?;
    ensure!(
        task.schema_version == TASK_SPEC_SCHEMA_VERSION
            && serde_json::to_value(&task)? == fixture.source_artifact,
        "historical TaskSpec did not roundtrip exactly"
    );

    let mut future = task.clone();
    future.schema_version = TASK_SPEC_SCHEMA_VERSION + 1;
    ensure!(
        matches!(
            future.validate(),
            Err(TaskSpecValidationError::UnsupportedSchemaVersion {
                expected: TASK_SPEC_SCHEMA_VERSION,
                actual
            }) if actual == TASK_SPEC_SCHEMA_VERSION + 1
        ),
        "TaskSpec reader accepted an unsupported future schema"
    );
    let mut unknown = fixture.source_artifact.clone();
    unknown
        .as_object_mut()
        .context("historical TaskSpec must be an object")?
        .insert("unknown_future_contract".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<TaskSpec>(unknown).is_err(),
        "TaskSpec reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_failure_capsule(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::AcceptedAndRestored
            && source.reason_code == HistoricalCompatibilityReason::SameSchemaValidatedArtifact
            && fixture.source_files.is_empty(),
        "Failure Capsule decision must retain the exact same-schema artifact"
    );
    let capsule: FailureCapsule = serde_json::from_value(fixture.source_artifact.clone())?;
    capsule.validate()?;
    ensure!(
        capsule.schema_version == FAILURE_CAPSULE_SCHEMA_VERSION
            && serde_json::to_value(&capsule)? == fixture.source_artifact,
        "historical Failure Capsule did not roundtrip exactly"
    );

    let mut future = capsule.clone();
    future.schema_version = FAILURE_CAPSULE_SCHEMA_VERSION + 1;
    ensure!(
        matches!(
            future.validate(),
            Err(FailureCapsuleError::UnsupportedSchemaVersion {
                expected: FAILURE_CAPSULE_SCHEMA_VERSION,
                actual
            }) if actual == FAILURE_CAPSULE_SCHEMA_VERSION + 1
        ),
        "Failure Capsule reader accepted an unsupported future schema"
    );
    let mut unknown = fixture.source_artifact.clone();
    unknown
        .as_object_mut()
        .context("historical Failure Capsule must be an object")?
        .insert("unknown_future_evidence".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<FailureCapsule>(unknown).is_err(),
        "Failure Capsule reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_dataset_bundle(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::AcceptedAndRestored
            && source.reason_code
                == HistoricalCompatibilityReason::SameSchemaStreamingDatasetBundle
            && fixture.source_files.len() == 1,
        "dataset decision must retain the complete same-schema streaming bundle"
    );
    let manifest: DatasetManifest = serde_json::from_value(fixture.source_artifact.clone())?;
    manifest.validate()?;
    ensure!(
        manifest.schema_version == DATASET_BUNDLE_SCHEMA_VERSION
            && serde_json::to_value(&manifest)? == fixture.source_artifact,
        "historical dataset manifest did not roundtrip exactly"
    );

    let bundle_temp = HistoricalDatasetTempDir::create()?;
    let bundle_root = bundle_temp.path().join("bundle");
    fs::create_dir(&bundle_root).context("create historical dataset bundle root")?;
    let mut manifest_bytes = serde_json::to_vec_pretty(&fixture.source_artifact)?;
    manifest_bytes.push(b'\n');
    fs::write(bundle_root.join("manifest.json"), manifest_bytes)
        .context("write historical dataset manifest")?;
    let shard = decode_lower_hex(&fixture.source_files[0].contents_hex)?;
    fs::write(bundle_root.join("records.rnedata"), &shard)
        .context("write historical dataset shard")?;

    let bundle = DatasetBundle::open(&bundle_root)?;
    let verification = bundle.verify()?;
    ensure!(
        verification.schema_version == DATASET_BUNDLE_SCHEMA_VERSION
            && verification.stream_count == 2
            && verification.record_count == 6
            && verification.sample_count == 4
            && verification.dropped_count == 2
            && verification.passed,
        "historical dataset verification result changed"
    );
    let evaluation = bundle.evaluate_depth_pair(DepthPairMetricSpec {
        predicted_stream: StreamId::new(10),
        ground_truth_stream: StreamId::new(11),
        tolerance_m: 0.11,
    })?;
    ensure!(
        evaluation.compared_frames == 2
            && evaluation.compared_pixels == 6
            && evaluation.dropped_pairs == 1
            && evaluation.passed,
        "historical dataset offline evaluation changed"
    );
    let evaluation_digest = canonical_json_digest(&serde_json::to_value(&evaluation)?)?;
    ensure!(
        Some(evaluation_digest.as_str()) == fixture.expected_result_sha256.as_deref(),
        "historical dataset offline evaluation digest changed"
    );

    let mut corrupted = shard;
    let last = corrupted
        .last_mut()
        .context("historical dataset shard must not be empty")?;
    *last ^= 1;
    fs::write(bundle_root.join("records.rnedata"), corrupted)
        .context("write corrupted historical dataset shard")?;
    ensure!(
        DatasetBundle::open(&bundle_root)?.verify().is_err(),
        "historical dataset reader accepted corrupted shard bytes"
    );

    let mut future = manifest.clone();
    future.schema_version = DATASET_BUNDLE_SCHEMA_VERSION + 1;
    ensure!(
        matches!(
            future.validate(),
            Err(DatasetError::InvalidField {
                field: "schema_version",
                ..
            })
        ),
        "dataset reader accepted an unsupported future schema"
    );
    let mut unknown = fixture.source_artifact.clone();
    unknown
        .as_object_mut()
        .context("historical dataset manifest must be an object")?
        .insert("wall_clock_time".to_string(), Value::from(1_u64));
    ensure!(
        serde_json::from_value::<DatasetManifest>(unknown).is_err(),
        "dataset reader accepted an unknown manifest field"
    );
    Ok(())
}

#[derive(Debug)]
struct HistoricalDatasetTempDir {
    path: PathBuf,
}

impl HistoricalDatasetTempDir {
    fn create() -> anyhow::Result<Self> {
        let temp_root = std::env::temp_dir();
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .context("system time predates Unix epoch")?
            .as_nanos();
        for attempt in 0..1_024_u32 {
            let path = temp_root.join(format!(
                "rne-historical-dataset-{}-{nonce}-{attempt}",
                std::process::id()
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Ok(Self { path }),
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(error) => {
                    return Err(error).context("create historical dataset verification root")
                }
            }
        }
        bail!("could not allocate a unique historical dataset verification root")
    }

    fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for HistoricalDatasetTempDir {
    fn drop(&mut self) {
        if fs::symlink_metadata(&self.path).is_ok_and(|metadata| {
            metadata.file_type().is_dir() && !metadata.file_type().is_symlink()
        }) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }
}

#[derive(Clone, Debug)]
struct CompatibilityToyEpisode {
    value: i32,
    step: u32,
}

impl Episode for CompatibilityToyEpisode {
    type Observation = i32;
    type Action = i32;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.value = 0;
        self.step = 0;
        EpisodeStep {
            observation: self.value,
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        self.value += action;
        self.step += 1;
        EpisodeStep {
            observation: self.value,
            reward: f64::from(self.value),
            terminated: false,
            truncated: self.step >= 4,
        }
    }

    fn episode_index(&self) -> u32 {
        0
    }

    fn step_in_episode(&self) -> u64 {
        u64::from(self.step)
    }
}

fn validate_historical_vectorized_checkpoint(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::AcceptedAndRestored
            && source.reason_code == HistoricalCompatibilityReason::SameSchemaReplayCheckpoint,
        "vectorized checkpoint decision must retain same-schema restore"
    );
    let checkpoint: VectorizedEpisodeCheckpoint<i32> =
        serde_json::from_value(fixture.source_artifact.clone())?;
    ensure!(
        checkpoint.schema_version == VECTORIZED_EPISODE_CHECKPOINT_VERSION
            && checkpoint.seed == 7
            && checkpoint.num_envs == 2
            && checkpoint.auto_reset
            && checkpoint.has_reset
            && checkpoint.actions == vec![vec![1, 2]]
            && checkpoint.replay_digest == HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST,
        "historical vectorized checkpoint generation recipe changed"
    );
    let mut batch = VectorizedEpisode::from_seeded(
        VectorizedEpisodeConfig {
            num_envs: 2,
            seed: 7,
            auto_reset: true,
        },
        |_seed| CompatibilityToyEpisode { value: 0, step: 0 },
    );
    batch
        .restore_checkpoint(&checkpoint)
        .map_err(|error| anyhow::anyhow!("restore historical vectorized checkpoint: {error:?}"))?;
    ensure!(
        batch.replay_digest() == HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST,
        "historical vectorized checkpoint replay digest changed"
    );
    let restored = batch
        .checkpoint()
        .map_err(|error| anyhow::anyhow!("capture restored vectorized checkpoint: {error:?}"))?;
    ensure!(
        restored == checkpoint,
        "historical vectorized checkpoint did not roundtrip exactly"
    );

    let mut future = checkpoint.clone();
    future.schema_version = VECTORIZED_EPISODE_CHECKPOINT_VERSION + 1;
    ensure!(
        matches!(
            batch.restore_checkpoint(&future),
            Err(VectorizedEpisodeCheckpointError::UnsupportedSchemaVersion {
                expected: VECTORIZED_EPISODE_CHECKPOINT_VERSION,
                actual
            }) if actual == VECTORIZED_EPISODE_CHECKPOINT_VERSION + 1
        ),
        "vectorized checkpoint reader accepted an unsupported future schema"
    );
    let mut unknown = fixture.source_artifact.clone();
    unknown
        .as_object_mut()
        .context("historical vectorized checkpoint must be an object")?
        .insert("unknown_future_state".to_string(), Value::Bool(true));
    ensure!(
        serde_json::from_value::<VectorizedEpisodeCheckpoint<i32>>(unknown).is_err(),
        "vectorized checkpoint reader accepted an unknown top-level field"
    );
    Ok(())
}

fn validate_historical_scenario_replay(
    fixture: &HistoricalCompatibilityDecisionFixture,
    source: &HistoricalCompatibilitySourceSpec,
) -> anyhow::Result<()> {
    ensure!(
        source.expected_outcome == HistoricalCompatibilityOutcome::RejectedRequiresRerun
            && source.reason_code == HistoricalCompatibilityReason::MissingRequiredReplayEvidence,
        "scenario replay decision must require rerun for missing evidence"
    );
    let artifact = &fixture.source_artifact;
    ensure!(
        artifact.get("kind").and_then(Value::as_str) == Some("rne-scenario-replay")
            && artifact.get("scenario_path").and_then(Value::as_str)
                == Some("assets/scenarios/speed.xosc")
            && artifact.get("network_path").and_then(Value::as_str)
                == Some("assets/traffic/corridor.rne.traffic.json")
            && artifact.get("executed_steps").and_then(Value::as_u64) == Some(300)
            && artifact.get("replayable").and_then(Value::as_bool) == Some(true)
            && artifact
                .get("control_commands")
                .and_then(Value::as_array)
                .is_some_and(Vec::is_empty)
            && json_path(artifact, "options.steps").and_then(Value::as_u64) == Some(300)
            && json_path(artifact, "options.hz").and_then(Value::as_f64) == Some(60.0)
            && json_path(artifact, "result.steps").and_then(Value::as_u64) == Some(300)
            && json_path(artifact, "result.stable_hash").and_then(Value::as_u64)
                == Some(HISTORICAL_SCENARIO_STABLE_HASH),
        "historical scenario replay generation recipe changed"
    );
    for field in source.missing_required_fields {
        ensure!(
            json_path(artifact, field).is_none(),
            "historical scenario replay unexpectedly contains required v4 field {field}"
        );
    }
    match source.source_schema_version {
        2 => ensure!(
            artifact.get("scenario_digest").is_none()
                && artifact.get("network_digest").is_none()
                && artifact.get("engine_version").is_none(),
            "scenario replay v2 unexpectedly contains v3 input provenance"
        ),
        3 => ensure!(
            artifact.get("scenario_digest").and_then(Value::as_u64)
                == Some(HISTORICAL_SCENARIO_INPUT_DIGEST)
                && artifact.get("network_digest").and_then(Value::as_u64)
                    == Some(HISTORICAL_SCENARIO_NETWORK_DIGEST)
                && artifact.get("engine_version").and_then(Value::as_str)
                    == Some(source.workspace_version),
            "scenario replay v3 input provenance changed"
        ),
        other => bail!("unsupported historical scenario replay schema {other}"),
    }

    let text = serde_json::to_string(artifact)?;
    let error = ScenarioReplayArtifact::from_json(&text)
        .expect_err("historical scenario replay must require rerun");
    ensure!(
        matches!(
            &error,
            ScenarioReplayArtifactError::UnsupportedVersion {
                expected: SCENARIO_REPLAY_SCHEMA_VERSION,
                actual
            } if *actual == source.source_schema_version
        ),
        "historical scenario replay used the wrong rejection class: {error}"
    );
    let error_text = error.to_string();
    ensure!(
        Some(error_text.as_str()) == fixture.expected_error.as_deref(),
        "historical scenario replay rejection text changed"
    );

    let mut relabeled = artifact.clone();
    relabeled
        .as_object_mut()
        .context("historical scenario replay must be an object")?
        .insert(
            "schema_version".to_string(),
            Value::from(SCENARIO_REPLAY_SCHEMA_VERSION),
        );
    ensure!(
        ScenarioReplayArtifact::from_json(&serde_json::to_string(&relabeled)?).is_err(),
        "historical scenario replay was accepted after unsafe schema relabeling"
    );
    let mut future = artifact.clone();
    future
        .as_object_mut()
        .context("historical scenario replay must be an object")?
        .insert(
            "schema_version".to_string(),
            Value::from(SCENARIO_REPLAY_SCHEMA_VERSION + 1),
        );
    ensure!(
        matches!(
            ScenarioReplayArtifact::from_json(&serde_json::to_string(&future)?),
            Err(ScenarioReplayArtifactError::UnsupportedVersion {
                expected: SCENARIO_REPLAY_SCHEMA_VERSION,
                actual
            }) if actual == SCENARIO_REPLAY_SCHEMA_VERSION + 1
        ),
        "scenario replay reader accepted an unsupported future schema"
    );
    Ok(())
}

fn json_path<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.')
        .try_fold(value, |current, key| current.get(key))
}

fn current_controller_c_abi() -> ControllerCAbiFixture {
    let pointer_size = std::mem::size_of::<*const std::ffi::c_char>();
    ControllerCAbiFixture {
        kind: CONTROLLER_C_ABI_LAYOUT_KIND.to_string(),
        schema_version: RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION,
        sdk_version: RNE_PLUGIN_SDK_VERSION,
        minimum_abi_version: RNE_PLUGIN_MIN_ABI_VERSION,
        current_abi_version: RNE_PLUGIN_ABI_VERSION,
        pointer_width_bits: usize::BITS,
        capability_bits: vec![
            c_abi_value(
                "joint_position_observation",
                RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION,
            ),
            c_abi_value(
                "joint_velocity_observation",
                RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION,
            ),
            c_abi_value(
                "joint_velocity_command",
                RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND,
            ),
            c_abi_value("multi_robot", RNE_CONTROLLER_CAP_MULTI_ROBOT),
        ],
        structs: vec![
            CAbiStructLayout {
                name: "RneJointPosition".to_string(),
                size_bytes: std::mem::size_of::<RneJointPosition>(),
                align_bytes: std::mem::align_of::<RneJointPosition>(),
                fields: vec![
                    c_abi_field(
                        "name",
                        "const char *",
                        std::mem::offset_of!(RneJointPosition, name),
                        pointer_size,
                    ),
                    c_abi_field(
                        "position_rad",
                        "double",
                        std::mem::offset_of!(RneJointPosition, position_rad),
                        std::mem::size_of::<f64>(),
                    ),
                ],
            },
            CAbiStructLayout {
                name: "RneJointVelocity".to_string(),
                size_bytes: std::mem::size_of::<RneJointVelocity>(),
                align_bytes: std::mem::align_of::<RneJointVelocity>(),
                fields: vec![
                    c_abi_field(
                        "name",
                        "const char *",
                        std::mem::offset_of!(RneJointVelocity, name),
                        pointer_size,
                    ),
                    c_abi_field(
                        "velocity_rad_s",
                        "double",
                        std::mem::offset_of!(RneJointVelocity, velocity_rad_s),
                        std::mem::size_of::<f64>(),
                    ),
                ],
            },
            CAbiStructLayout {
                name: "RneJointObservationV3".to_string(),
                size_bytes: std::mem::size_of::<RneJointObservationV3>(),
                align_bytes: std::mem::align_of::<RneJointObservationV3>(),
                fields: vec![
                    c_abi_field(
                        "robot_id",
                        "const char *",
                        std::mem::offset_of!(RneJointObservationV3, robot_id),
                        pointer_size,
                    ),
                    c_abi_field(
                        "name",
                        "const char *",
                        std::mem::offset_of!(RneJointObservationV3, name),
                        pointer_size,
                    ),
                    c_abi_field(
                        "position_rad",
                        "double",
                        std::mem::offset_of!(RneJointObservationV3, position_rad),
                        std::mem::size_of::<f64>(),
                    ),
                    c_abi_field(
                        "velocity_rad_s",
                        "double",
                        std::mem::offset_of!(RneJointObservationV3, velocity_rad_s),
                        std::mem::size_of::<f64>(),
                    ),
                    c_abi_field(
                        "has_velocity",
                        "uint8_t",
                        std::mem::offset_of!(RneJointObservationV3, has_velocity),
                        std::mem::size_of::<u8>(),
                    ),
                    c_abi_field(
                        "reserved",
                        "uint8_t[7]",
                        std::mem::offset_of!(RneJointObservationV3, reserved),
                        std::mem::size_of::<[u8; 7]>(),
                    ),
                ],
            },
            CAbiStructLayout {
                name: "RneJointVelocityV3".to_string(),
                size_bytes: std::mem::size_of::<RneJointVelocityV3>(),
                align_bytes: std::mem::align_of::<RneJointVelocityV3>(),
                fields: vec![
                    c_abi_field(
                        "robot_id",
                        "const char *",
                        std::mem::offset_of!(RneJointVelocityV3, robot_id),
                        pointer_size,
                    ),
                    c_abi_field(
                        "name",
                        "const char *",
                        std::mem::offset_of!(RneJointVelocityV3, name),
                        pointer_size,
                    ),
                    c_abi_field(
                        "velocity_rad_s",
                        "double",
                        std::mem::offset_of!(RneJointVelocityV3, velocity_rad_s),
                        std::mem::size_of::<f64>(),
                    ),
                ],
            },
            CAbiStructLayout {
                name: "RneControllerStepResultV3".to_string(),
                size_bytes: std::mem::size_of::<RneControllerStepResultV3>(),
                align_bytes: std::mem::align_of::<RneControllerStepResultV3>(),
                fields: vec![
                    c_abi_field(
                        "status",
                        "int32_t",
                        std::mem::offset_of!(RneControllerStepResultV3, status),
                        std::mem::size_of::<i32>(),
                    ),
                    c_abi_field(
                        "output_count",
                        "size_t",
                        std::mem::offset_of!(RneControllerStepResultV3, output_count),
                        std::mem::size_of::<usize>(),
                    ),
                ],
            },
        ],
        symbols: vec![
            c_abi_symbol("rne_plugin_abi_version", 2, "uint32_t(void)"),
            c_abi_symbol("rne_plugin_name", 2, "const char *(void)"),
            c_abi_symbol(
                "rne_controller_create",
                2,
                "void *(const char *, double, double, double, char *, size_t)",
            ),
            c_abi_symbol("rne_controller_destroy", 2, "void(void *)"),
            c_abi_symbol(
                "rne_controller_step",
                2,
                "size_t(const void *, const RneJointPosition *, size_t, RneJointVelocity *, size_t)",
            ),
            c_abi_symbol("rne_plugin_capabilities", 3, "uint64_t(void)"),
            c_abi_symbol(
                "rne_controller_configure_v3",
                3,
                "int32_t(void *, uint64_t, char *, size_t)",
            ),
            c_abi_symbol(
                "rne_controller_reset_v3",
                3,
                "int32_t(void *, uint64_t, uint64_t, uint64_t, uint64_t, char *, size_t)",
            ),
            c_abi_symbol(
                "rne_controller_step_v3",
                3,
                "RneControllerStepResultV3(void *, uint64_t, uint64_t, const RneJointObservationV3 *, size_t, RneJointVelocityV3 *, size_t, char *, size_t)",
            ),
            c_abi_symbol(
                "rne_controller_shutdown_v3",
                3,
                "int32_t(void *, char *, size_t)",
            ),
        ],
    }
}

fn c_abi_value(name: &str, value: u64) -> CAbiNamedValue {
    CAbiNamedValue {
        name: name.to_string(),
        value,
    }
}

fn c_abi_field(
    name: &str,
    c_type: &str,
    offset_bytes: usize,
    size_bytes: usize,
) -> CAbiFieldLayout {
    CAbiFieldLayout {
        name: name.to_string(),
        c_type: c_type.to_string(),
        offset_bytes,
        size_bytes,
    }
}

fn c_abi_symbol(name: &str, since_abi: u32, c_signature: &str) -> CAbiSymbol {
    CAbiSymbol {
        name: name.to_string(),
        since_abi,
        c_signature: c_signature.to_string(),
    }
}

fn validate_frontend_transport(fixture: &FrontendTransportFixture) -> anyhow::Result<()> {
    ensure!(
        fixture.schema_version == 1,
        "frontend transport fixture schema mismatch"
    );
    ensure!(
        fixture.message_kind == "client_hello",
        "frontend fixture message kind mismatch"
    );
    let bytes = decode_lower_hex(&fixture.frame_hex)?;
    let frame = TransportFrame::decode(&bytes, TRANSPORT_MAX_PAYLOAD_BYTES)?;
    ensure!(
        frame.protocol_major == fixture.protocol_major
            && frame.protocol_minor == fixture.protocol_minor
            && frame.kind == TransportMessageKind::ClientHello
            && frame.flags == fixture.flags
            && frame.sequence == fixture.sequence
            && frame.session_id == fixture.session_id,
        "frontend frame header mismatch"
    );
    ensure!(
        frame.encode()? == bytes,
        "frontend frame did not re-encode exactly"
    );

    let expected_hello = fixture.hello.as_client_hello();
    let hello = ClientHello::decode_payload(&frame.payload)?;
    ensure!(hello == expected_hello, "frontend hello payload mismatch");
    ensure!(
        hello.encode_payload() == frame.payload,
        "frontend hello did not re-encode exactly"
    );

    let negotiated = negotiate_transport(hello, NegotiationPolicy::default())
        .map_err(|reject| anyhow::anyhow!("frontend negotiation rejected: {:?}", reject.code))?;
    let expected = fixture.negotiated;
    ensure!(
        negotiated.protocol_major == expected.protocol_major
            && negotiated.protocol_minor == expected.protocol_minor
            && negotiated.capabilities.bits() == expected.capabilities_bits
            && negotiated.max_payload_bytes == expected.max_payload_bytes
            && negotiated.queue_frame_limit == expected.queue_frame_limit
            && negotiated.queue_byte_limit == expected.queue_byte_limit
            && negotiated.resume_after_sequence == expected.resume_after_sequence,
        "frontend negotiated limits mismatch"
    );

    let mut bad_magic = bytes.clone();
    bad_magic[0] ^= 1;
    ensure!(
        TransportFrame::decode(&bad_magic, TRANSPORT_MAX_PAYLOAD_BYTES).is_err(),
        "frontend transport accepted corrupt magic"
    );
    let mut unknown_kind = bytes.clone();
    unknown_kind[8..10].copy_from_slice(&u16::MAX.to_le_bytes());
    ensure!(
        TransportFrame::decode(&unknown_kind, TRANSPORT_MAX_PAYLOAD_BYTES).is_err(),
        "frontend transport accepted an unknown message kind"
    );
    ensure_rejects_edge_mutations(&bytes, |candidate| {
        TransportFrame::decode(candidate, TRANSPORT_MAX_PAYLOAD_BYTES).map(|_| ())
    })?;
    ensure_rejects_edge_mutations(&frame.payload, |candidate| {
        ClientHello::decode_payload(candidate).map(|_| ())
    })?;

    let incompatible = ClientHello {
        min_protocol_major: fixture.protocol_major.saturating_add(1),
        max_protocol_major: fixture.protocol_major.saturating_add(1),
        ..hello
    };
    let rejection = negotiate_transport(incompatible, NegotiationPolicy::default())
        .expect_err("incompatible frontend protocol must fail closed");
    ensure!(
        rejection.code == NegotiationRejectCode::UnsupportedVersion,
        "frontend version mismatch used the wrong rejection code"
    );
    Ok(())
}

fn validate_frontend_message_families(
    fixture: &FrontendMessageFamiliesFixture,
) -> anyhow::Result<()> {
    ensure!(
        fixture.schema_version == 1,
        "frontend message-family fixture schema mismatch"
    );
    ensure!(
        fixture.frames.len() == FRONTEND_MESSAGE_FAMILIES.len(),
        "frontend message-family fixture must cover exactly ten protocol-v1 kinds"
    );
    for (index, (frame_fixture, (name, kind))) in fixture
        .frames
        .iter()
        .zip(FRONTEND_MESSAGE_FAMILIES)
        .enumerate()
    {
        ensure!(
            frame_fixture.message_kind == name,
            "frontend message-family order mismatch at {name}"
        );
        let expected_sequence = FRONTEND_FAMILY_FIRST_SEQUENCE + index as u64;
        ensure!(
            frame_fixture.protocol_major == 1
                && frame_fixture.protocol_minor == 0
                && frame_fixture.flags == 0
                && frame_fixture.sequence == expected_sequence
                && frame_fixture.session_id == FRONTEND_FAMILY_SESSION_ID,
            "frontend {name} fixture header contract mismatch"
        );
        let bytes = decode_lower_hex(&frame_fixture.frame_hex)?;
        let frame = TransportFrame::decode(&bytes, TRANSPORT_MAX_PAYLOAD_BYTES)?;
        ensure!(
            frame.protocol_major == frame_fixture.protocol_major
                && frame.protocol_minor == frame_fixture.protocol_minor
                && frame.kind == kind
                && frame.flags == frame_fixture.flags
                && frame.sequence == frame_fixture.sequence
                && frame.session_id == frame_fixture.session_id,
            "frontend {name} frame header mismatch"
        );
        ensure!(
            frame.encode()? == bytes,
            "frontend {name} frame did not re-encode exactly"
        );
        ensure!(
            frame.payload == reference_frontend_payload(kind)?,
            "frontend {name} payload bytes changed"
        );
        validate_reference_frontend_payload(kind, &frame.payload)?;
        ensure_rejects_edge_mutations(&bytes, |candidate| {
            TransportFrame::decode(candidate, TRANSPORT_MAX_PAYLOAD_BYTES).map(|_| ())
        })?;
        ensure_rejects_edge_mutations(&frame.payload, |candidate| {
            decode_frontend_payload(kind, candidate)
        })?;
    }
    Ok(())
}

fn reference_client_hello() -> ClientHello {
    ClientHello {
        min_protocol_major: 1,
        max_protocol_major: 1,
        capabilities: TransportCapabilities::ALL_V1,
        required_capabilities: TransportCapabilities::CONTROL.union(TransportCapabilities::STATUS),
        max_payload_bytes: 1_048_576,
        queue_frame_limit: 16,
        queue_byte_limit: 2_097_152,
        resume_after_sequence: Some(41),
    }
}

fn reference_negotiated_transport() -> NegotiatedTransport {
    NegotiatedTransport {
        protocol_major: 1,
        protocol_minor: 0,
        capabilities: TransportCapabilities::ALL_V1,
        max_payload_bytes: 1_048_576,
        queue_frame_limit: 16,
        queue_byte_limit: 2_097_152,
        resume_after_sequence: Some(41),
    }
}

fn reference_sensor_metadata() -> SensorFrameMetadata {
    SensorFrameMetadata {
        stream_id: 7,
        sensor_sequence: 11,
        capture_ticks: 20,
        available_ticks: 25,
    }
}

fn reference_lidar_cloud() -> PointCloud {
    let mut cloud = PointCloud::new();
    cloud.push_return(Vec3::new(1.0, 2.0, 3.0), 0.5, 7, 1, 2, 0.01);
    cloud.push_return(Vec3::new(4.0, 5.0, 6.0), 0.75, 8, 2, 3, 0.02);
    cloud
}

fn reference_frontend_payload(kind: TransportMessageKind) -> anyhow::Result<Vec<u8>> {
    Ok(match kind {
        TransportMessageKind::ClientHello => reference_client_hello().encode_payload(),
        TransportMessageKind::ServerHello => ServerHello {
            negotiated: reference_negotiated_transport(),
            reconnect_generation: 2,
            current_sequence: 99,
            dropped_messages: 4,
        }
        .encode_payload(),
        TransportMessageKind::Reject => NegotiationReject {
            code: NegotiationRejectCode::UnsupportedVersion,
            message: "protocol v1 required".to_string(),
        }
        .encode_payload(),
        TransportMessageKind::ControlCommand => {
            encode_control_command(ControlCommand::Step { frames: 17 })
        }
        TransportMessageKind::ControlAck => ControlAck {
            command_sequence: 103,
            state: RunnerControlState::Paused,
        }
        .encode_payload(),
        TransportMessageKind::Status => StatusMessage {
            step: 3,
            sim_time_ticks: 50,
            state: RunnerControlState::Paused,
            snapshot_json: br#"{"base":[1,2,3]}"#.to_vec(),
        }
        .encode_payload()?,
        TransportMessageKind::ImageRgb8 => encode_image_rgb8(
            reference_sensor_metadata(),
            &ImageRgb8::from_rgba8(2, 1, vec![1, 2, 3, 4, 5, 6, 7, 8]),
        )?,
        TransportMessageKind::ImageDepthF32 => encode_image_depth(
            reference_sensor_metadata(),
            &ImageDepth::new(2, 1, vec![1.25, 9.5]),
        )?,
        TransportMessageKind::LidarPointCloud => {
            encode_lidar_point_cloud(reference_sensor_metadata(), &reference_lidar_cloud())?
        }
        TransportMessageKind::Gap => GapNotice {
            first_missing_sequence: 20,
            last_missing_sequence: 22,
            total_dropped_messages: 3,
        }
        .encode_payload(),
    })
}

fn validate_reference_frontend_payload(
    kind: TransportMessageKind,
    payload: &[u8],
) -> anyhow::Result<()> {
    match kind {
        TransportMessageKind::ClientHello => {
            ensure!(
                ClientHello::decode_payload(payload)? == reference_client_hello(),
                "client hello semantics changed"
            );
        }
        TransportMessageKind::ServerHello => {
            ensure!(
                ServerHello::decode_payload(payload)?
                    == ServerHello {
                        negotiated: reference_negotiated_transport(),
                        reconnect_generation: 2,
                        current_sequence: 99,
                        dropped_messages: 4,
                    },
                "server hello semantics changed"
            );
        }
        TransportMessageKind::Reject => {
            ensure!(
                NegotiationReject::decode_payload(payload)?
                    == NegotiationReject {
                        code: NegotiationRejectCode::UnsupportedVersion,
                        message: "protocol v1 required".to_string(),
                    },
                "negotiation rejection semantics changed"
            );
        }
        TransportMessageKind::ControlCommand => {
            ensure!(
                decode_control_command(payload)? == ControlCommand::Step { frames: 17 },
                "control command semantics changed"
            );
        }
        TransportMessageKind::ControlAck => {
            ensure!(
                ControlAck::decode_payload(payload)?
                    == ControlAck {
                        command_sequence: 103,
                        state: RunnerControlState::Paused,
                    },
                "control acknowledgement semantics changed"
            );
        }
        TransportMessageKind::Status => {
            ensure!(
                StatusMessage::decode_payload(payload)?
                    == StatusMessage {
                        step: 3,
                        sim_time_ticks: 50,
                        state: RunnerControlState::Paused,
                        snapshot_json: br#"{"base":[1,2,3]}"#.to_vec(),
                    },
                "status semantics changed"
            );
        }
        TransportMessageKind::ImageRgb8 => {
            ensure!(
                decode_image_rgb8(payload)?
                    == (
                        reference_sensor_metadata(),
                        ImageRgb8::from_rgba8(2, 1, vec![1, 2, 3, 4, 5, 6, 7, 8]),
                    ),
                "RGB8 semantics changed"
            );
        }
        TransportMessageKind::ImageDepthF32 => {
            ensure!(
                decode_image_depth(payload)?
                    == (
                        reference_sensor_metadata(),
                        ImageDepth::new(2, 1, vec![1.25, 9.5]),
                    ),
                "depth semantics changed"
            );
        }
        TransportMessageKind::LidarPointCloud => {
            ensure!(
                decode_lidar_point_cloud(payload)?
                    == (reference_sensor_metadata(), reference_lidar_cloud()),
                "LiDAR semantics changed"
            );
        }
        TransportMessageKind::Gap => {
            ensure!(
                GapNotice::decode_payload(payload)?
                    == GapNotice {
                        first_missing_sequence: 20,
                        last_missing_sequence: 22,
                        total_dropped_messages: 3,
                    },
                "gap semantics changed"
            );
        }
    }
    Ok(())
}

fn decode_frontend_payload(
    kind: TransportMessageKind,
    payload: &[u8],
) -> Result<(), rne_data::transport::TransportError> {
    match kind {
        TransportMessageKind::ClientHello => ClientHello::decode_payload(payload).map(|_| ()),
        TransportMessageKind::ServerHello => ServerHello::decode_payload(payload).map(|_| ()),
        TransportMessageKind::Reject => NegotiationReject::decode_payload(payload).map(|_| ()),
        TransportMessageKind::ControlCommand => decode_control_command(payload).map(|_| ()),
        TransportMessageKind::ControlAck => ControlAck::decode_payload(payload).map(|_| ()),
        TransportMessageKind::Status => StatusMessage::decode_payload(payload).map(|_| ()),
        TransportMessageKind::ImageRgb8 => decode_image_rgb8(payload).map(|_| ()),
        TransportMessageKind::ImageDepthF32 => decode_image_depth(payload).map(|_| ()),
        TransportMessageKind::LidarPointCloud => decode_lidar_point_cloud(payload).map(|_| ()),
        TransportMessageKind::Gap => GapNotice::decode_payload(payload).map(|_| ()),
    }
}

fn validate_dataset_payload(fixture: &DatasetPayloadFixture) -> anyhow::Result<()> {
    ensure!(
        fixture.schema_version == DATASET_PAYLOAD_SCHEMA_VERSION,
        "dataset payload schema mismatch"
    );
    let metadata = fixture.metadata.as_metadata();

    let imu = ImuSample {
        angular_velocity_rad_s: vec3(fixture.imu.angular_velocity_rad_s),
        linear_acceleration_m_s2: vec3(fixture.imu.linear_acceleration_m_s2),
    };
    let imu_bytes = decode_lower_hex(&fixture.imu.payload_hex)?;
    ensure!(
        decode_dataset_imu(&imu_bytes)? == (metadata, imu),
        "dataset IMU payload mismatch"
    );
    ensure!(
        encode_dataset_imu(metadata, &imu)? == imu_bytes,
        "dataset IMU payload did not re-encode exactly"
    );
    ensure_rejects_edge_mutations(&imu_bytes, |bytes| decode_dataset_imu(bytes).map(|_| ()))?;

    let transform = PoseSample {
        position_m: vec3(fixture.transform.position_m),
        yaw_rad: fixture.transform.yaw_rad,
    };
    let transform_bytes = decode_lower_hex(&fixture.transform.payload_hex)?;
    ensure!(
        decode_dataset_transform(&transform_bytes)? == (metadata, transform),
        "dataset transform payload mismatch"
    );
    ensure!(
        encode_dataset_transform(metadata, &transform)? == transform_bytes,
        "dataset transform payload did not re-encode exactly"
    );
    ensure_rejects_edge_mutations(&transform_bytes, |bytes| {
        decode_dataset_transform(bytes).map(|_| ())
    })?;

    let action = DatasetActionSample {
        values: fixture.action.values.clone(),
    };
    let action_bytes = decode_lower_hex(&fixture.action.payload_hex)?;
    ensure!(
        decode_dataset_action(&action_bytes)? == (metadata, action.clone()),
        "dataset action payload mismatch"
    );
    ensure!(
        encode_dataset_action(metadata, &action)? == action_bytes,
        "dataset action payload did not re-encode exactly"
    );
    ensure_rejects_edge_mutations(&action_bytes, |bytes| {
        decode_dataset_action(bytes).map(|_| ())
    })?;

    let outcome = DatasetTaskOutcomeSample {
        episode_index: fixture.outcome.episode_index,
        step_in_episode: fixture.outcome.step_in_episode,
        reward: fixture.outcome.reward,
        cumulative_reward: fixture.outcome.cumulative_reward,
        terminated: fixture.outcome.terminated,
        truncated: fixture.outcome.truncated,
        success: fixture.outcome.success,
    };
    let outcome_bytes = decode_lower_hex(&fixture.outcome.payload_hex)?;
    ensure!(
        decode_dataset_task_outcome(&outcome_bytes)? == (metadata, outcome),
        "dataset outcome payload mismatch"
    );
    ensure!(
        encode_dataset_task_outcome(metadata, &outcome)? == outcome_bytes,
        "dataset outcome payload did not re-encode exactly"
    );
    ensure_rejects_edge_mutations(&outcome_bytes, |bytes| {
        decode_dataset_task_outcome(bytes).map(|_| ())
    })?;

    let annotation = DatasetGroundTruthAnnotation {
        class_id: fixture.annotation.class_id,
        instance_id: fixture.annotation.instance_id,
        values: fixture.annotation.values.clone(),
    };
    let annotation_bytes = decode_lower_hex(&fixture.annotation.payload_hex)?;
    ensure!(
        decode_dataset_annotation(&annotation_bytes)? == (metadata, annotation.clone()),
        "dataset annotation payload mismatch"
    );
    ensure!(
        encode_dataset_annotation(metadata, &annotation)? == annotation_bytes,
        "dataset annotation payload did not re-encode exactly"
    );
    ensure_rejects_edge_mutations(&annotation_bytes, |bytes| {
        decode_dataset_annotation(bytes).map(|_| ())
    })?;
    Ok(())
}

fn vec3(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

fn ensure_rejects_edge_mutations<E>(
    bytes: &[u8],
    mut decode: impl FnMut(&[u8]) -> Result<(), E>,
) -> anyhow::Result<()> {
    ensure!(!bytes.is_empty(), "binary compatibility payload is empty");
    ensure!(
        decode(&bytes[..bytes.len() - 1]).is_err(),
        "binary reader accepted a truncated payload"
    );
    let mut trailing = bytes.to_vec();
    trailing.push(0);
    ensure!(
        decode(&trailing).is_err(),
        "binary reader accepted trailing bytes"
    );
    Ok(())
}

fn decode_lower_hex(text: &str) -> anyhow::Result<Vec<u8>> {
    ensure!(
        text.len().is_multiple_of(2) && !text.is_empty(),
        "binary fixture hex must contain complete bytes"
    );
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let high = lower_hex_nibble(pair[0])?;
            let low = lower_hex_nibble(pair[1])?;
            Ok((high << 4) | low)
        })
        .collect()
}

fn lower_hex_nibble(byte: u8) -> anyhow::Result<u8> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        _ => bail!("binary fixture hex must use lowercase ASCII"),
    }
}

fn validate_checkpoint(checkpoint: &PortableBatchCheckpoint<u64>) -> anyhow::Result<()> {
    ensure!(
        checkpoint.num_envs > 0,
        "checkpoint num_envs must be positive"
    );
    ensure!(
        checkpoint.lanes.len() == checkpoint.num_envs,
        "checkpoint lane count mismatch"
    );
    if let Some(task_spec) = &checkpoint.task_spec {
        task_spec.validate()?;
    }
    for (index, lane) in checkpoint.lanes.iter().enumerate() {
        ensure!(
            lane.lane_id == index as u64,
            "checkpoint lane order mismatch"
        );
        ensure!(
            lane.episode_seed.is_some() == checkpoint.seed_strategy.is_some(),
            "checkpoint seeded mode mismatch"
        );
    }
    for operation in &checkpoint.operations {
        match operation {
            PortableBatchOperation::Step { actions } => ensure!(
                actions.len() == checkpoint.num_envs,
                "checkpoint action width mismatch"
            ),
            PortableBatchOperation::ResetLanes { lane_ids } => {
                ensure!(
                    !lane_ids.is_empty(),
                    "checkpoint reset lanes must not be empty"
                );
                ensure!(
                    lane_ids.windows(2).all(|ids| ids[0] < ids[1])
                        && lane_ids
                            .last()
                            .is_some_and(|lane_id| *lane_id < checkpoint.num_envs as u64),
                    "checkpoint reset lane IDs are not canonical"
                );
            }
            _ => bail!("checkpoint contains an unsupported operation"),
        }
    }
    Ok(())
}

fn resolve_fixture(root: &Path, relative: &str) -> anyhow::Result<PathBuf> {
    validate_relative_path(relative)?;
    let path = root.join(relative);
    let canonical = path
        .canonicalize()
        .with_context(|| format!("canonicalize compatibility fixture {}", path.display()))?;
    ensure!(
        canonical.starts_with(root),
        "compatibility fixture escapes corpus root"
    );
    Ok(canonical)
}

fn validate_relative_path(path: &str) -> anyhow::Result<()> {
    ensure!(
        !path.is_empty(),
        "compatibility fixture path must not be empty"
    );
    ensure!(
        !path.contains('\\'),
        "compatibility fixture path must use forward slashes"
    );
    let path = Path::new(path);
    ensure!(
        !path.is_absolute(),
        "compatibility fixture path must be relative"
    );
    ensure!(
        path.components()
            .all(|component| matches!(component, Component::Normal(_))),
        "compatibility fixture path is not canonical"
    );
    Ok(())
}

fn validate_sha256(digest: &str) -> anyhow::Result<()> {
    let Some(hex) = digest.strip_prefix("sha256:") else {
        bail!("compatibility fixture digest must use sha256: prefix");
    };
    ensure!(
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "compatibility fixture digest must be 64 lowercase hex characters"
    );
    Ok(())
}

fn canonical_json_digest(value: &Value) -> anyhow::Result<String> {
    Ok(sha256(&serde_json::to_vec(value)?))
}

fn normalized_state_digest(value: &Value) -> anyhow::Result<String> {
    let normalized = canonical_state_value(value);
    canonical_json_digest(&normalized)
}

fn canonical_state_value(value: &Value) -> Value {
    match value {
        Value::Number(number) if number.is_f64() => {
            let value = number.as_f64().expect("JSON float");
            let rounded = (value * 1_000_000_000.0).round() / 1_000_000_000.0;
            Value::from(if rounded == 0.0 { 0.0 } else { rounded })
        }
        Value::Array(values) => Value::Array(values.iter().map(canonical_state_value).collect()),
        Value::Object(values) => {
            let mut entries = values.iter().collect::<Vec<_>>();
            entries.sort_by_key(|(key, _)| *key);
            Value::Object(
                entries
                    .into_iter()
                    .map(|(key, value)| (key.clone(), canonical_state_value(value)))
                    .collect(),
            )
        }
        _ => value.clone(),
    }
}

fn first_json_difference(expected: &Value, actual: &Value, path: &str) -> String {
    match (expected, actual) {
        (Value::Object(expected), Value::Object(actual)) => {
            if expected.keys().collect::<Vec<_>>() != actual.keys().collect::<Vec<_>>() {
                return format!("{path} keys differ");
            }
            for (key, expected_value) in expected {
                let actual_value = &actual[key];
                if expected_value != actual_value {
                    return first_json_difference(
                        expected_value,
                        actual_value,
                        &format!("{path}.{key}"),
                    );
                }
            }
            format!("{path} differs")
        }
        (Value::Array(expected), Value::Array(actual)) => {
            if expected.len() != actual.len() {
                return format!(
                    "{path} length differs: expected {}, got {}",
                    expected.len(),
                    actual.len()
                );
            }
            for (index, (expected_value, actual_value)) in expected.iter().zip(actual).enumerate() {
                if expected_value != actual_value {
                    return first_json_difference(
                        expected_value,
                        actual_value,
                        &format!("{path}[{index}]"),
                    );
                }
            }
            format!("{path} differs")
        }
        _ => format!("{path} differs: expected {expected}, got {actual}"),
    }
}

fn registry_digest(registry: &CompatibilityFixtureRegistry) -> anyhow::Result<String> {
    Ok(sha256(&serde_json::to_vec(registry)?))
}

fn git_text(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(args)
        .output()
        .with_context(|| format!("run git {}", args.join(" ")))?;
    ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        bounded_detail(&String::from_utf8_lossy(&output.stderr))
    );
    String::from_utf8(output.stdout).context("git output is not UTF-8")
}

fn sha256(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

fn bounded_detail(detail: &str) -> String {
    detail
        .replace(['\r', '\n'], " ")
        .chars()
        .take(MAX_DETAIL_CHARS)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn workspace_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(Path::parent)
            .expect("workspace root")
            .to_path_buf()
    }

    #[test]
    fn committed_corpus_passes_every_reader_and_rejection_check() {
        let root = workspace_root();
        let report = run_compatibility(&root, &root.join("release/compatibility-fixtures.toml"))
            .expect("run corpus");
        assert!(report.passed, "checks: {:#?}", report.checks);
        assert_eq!(report.checks.len(), FIXTURE_SPECS.len());
    }

    #[test]
    fn sensor_task_identity_and_hardware_session_cannot_alias_the_retained_task() {
        let root = workspace_root();
        let current: TaskSpec = serde_json::from_slice(
            &fs::read(root.join("assets/tasks/diff_drive_goal.task.json")).unwrap(),
        )
        .unwrap();
        let retained: TaskSpec = serde_json::from_slice(
            &fs::read(root.join("tests/golden/tasks/task-spec-v1.json")).unwrap(),
        )
        .unwrap();
        let session: HardwareSessionEvidence = serde_json::from_slice(
            &fs::read(
                root.join("tests/golden/hardware/gateway-process-disconnect-session-v1.json"),
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(current.task_id, "rne.diff_drive.sensor_goal.v1");
        assert_eq!(retained.task_id, "rne.diff_drive.goal.v1");
        assert_ne!(current.task_id, retained.task_id);
        assert_eq!(flattened_width(&current.observation.tensors), 9);
        assert_eq!(flattened_width(&retained.observation.tensors), 5);
        session.validate_against(&current).unwrap();
        assert!(session.validate_against(&retained).is_err());
    }

    #[test]
    fn committed_report_matches_golden_shape() {
        let root = workspace_root();
        let report = run_compatibility(&root, &root.join("release/compatibility-fixtures.toml"))
            .expect("run corpus");
        let actual = format!("{}\n", serde_json::to_string_pretty(&report).unwrap());
        let expected = fs::read_to_string(
            root.join("tests/compatibility/tests/golden/compatibility-report-v1.json"),
        )
        .expect("read golden");
        assert_eq!(actual.replace("\r\n", "\n"), expected.replace("\r\n", "\n"));
    }

    #[test]
    fn frontend_message_families_reject_reordering_and_wire_tampering() {
        let root = workspace_root();
        let bytes =
            fs::read(root.join("tests/golden/protocol/frontend-message-families-v1.json")).unwrap();
        let fixture: FrontendMessageFamiliesFixture = serde_json::from_slice(&bytes).unwrap();
        validate_frontend_message_families(&fixture).unwrap();

        let mut reordered = fixture.clone();
        reordered.frames.swap(0, 1);
        assert!(validate_frontend_message_families(&reordered).is_err());

        let mut tampered = fixture;
        let last = tampered.frames[9].frame_hex.len() - 2;
        tampered.frames[9].frame_hex.replace_range(last.., "01");
        assert!(validate_frontend_message_families(&tampered).is_err());
    }

    fn flattened_width(tensors: &[rne_ai::TensorSpec]) -> usize {
        tensors
            .iter()
            .map(|tensor| tensor.shape.iter().copied().product::<usize>())
            .sum()
    }

    #[test]
    fn strict_registry_rejects_unknown_fields_and_traversal() {
        let root = workspace_root();
        let text = fs::read_to_string(root.join("release/compatibility-fixtures.toml")).unwrap();
        let unknown = format!("unknown = true\n{text}");
        assert!(toml::from_str::<CompatibilityFixtureRegistry>(&unknown).is_err());

        let mut registry: CompatibilityFixtureRegistry = toml::from_str(&text).unwrap();
        registry.fixtures[0].path = "../escape.json".to_string();
        assert!(validate_registry(&registry).is_err());
    }

    #[test]
    fn migration_state_hash_normalizes_signed_zero_and_sub_tolerance_drift() {
        let expected = serde_json::json!({
            "position_m": [0.95, 0.0, -0.04],
            "schema_version": 3,
        });
        let reconstructed = serde_json::json!({
            "schema_version": 3,
            "position_m": [0.9500000000000001, -0.0, -0.04000000000000001],
        });
        assert_eq!(
            normalized_state_digest(&expected).unwrap(),
            normalized_state_digest(&reconstructed).unwrap()
        );
        let outside_tolerance = serde_json::json!({
            "position_m": [0.950000002, 0.0, -0.04],
            "schema_version": 3,
        });
        assert_ne!(
            normalized_state_digest(&expected).unwrap(),
            normalized_state_digest(&outside_tolerance).unwrap()
        );
    }

    #[test]
    fn historical_provenance_rejects_retargeting_and_source_tampering() {
        let root = workspace_root();
        let path =
            root.join("tests/golden/migrations/mobile-manipulator-snapshot-v2-2255cbe-to-v3.json");
        let value: Value = serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
        let fixture: HistoricalMigrationProvenanceFixture = serde_json::from_value(value).unwrap();

        let mut retargeted = fixture.clone();
        retargeted.source_revision = "0".repeat(40);
        assert!(validate_historical_mobile_manipulator_snapshot_provenance(
            &root,
            "mobile_manipulator_snapshot_v2_2255cbe_to_v3",
            &retargeted,
        )
        .is_err());

        let mut tampered = fixture;
        tampered.source_snapshot["sim_ticks"] = Value::from(1_u64);
        let error = validate_historical_mobile_manipulator_snapshot_provenance(
            &root,
            "mobile_manipulator_snapshot_v2_2255cbe_to_v3",
            &tampered,
        )
        .unwrap_err();
        assert!(format!("{error:#}").contains("source snapshot digest mismatch"));
    }

    #[test]
    fn historical_decisions_reject_retargeting_and_unsafe_relabeling() {
        let root = workspace_root();
        let scenario_path =
            root.join("tests/golden/compatibility/scenario-replay-v3-e959e3f-requires-rerun.json");
        let value: Value = serde_json::from_slice(&fs::read(scenario_path).unwrap()).unwrap();
        let fixture: HistoricalCompatibilityDecisionFixture =
            serde_json::from_value(value).unwrap();
        validate_historical_compatibility_decision(
            "scenario_replay_v3_e959e3f_requires_rerun",
            &fixture,
        )
        .unwrap();

        let mut retargeted = fixture.clone();
        retargeted.source_tree = "0".repeat(40);
        assert!(validate_historical_compatibility_decision(
            "scenario_replay_v3_e959e3f_requires_rerun",
            &retargeted,
        )
        .is_err());

        let mut relabeled = fixture;
        relabeled.source_artifact["schema_version"] = Value::from(SCENARIO_REPLAY_SCHEMA_VERSION);
        assert!(validate_historical_compatibility_decision(
            "scenario_replay_v3_e959e3f_requires_rerun",
            &relabeled,
        )
        .is_err());

        let checkpoint_path =
            root.join("tests/golden/compatibility/vectorized-episode-checkpoint-v1-bd4d44f.json");
        let checkpoint: Value =
            serde_json::from_slice(&fs::read(checkpoint_path).unwrap()).unwrap();
        let mut unknown = checkpoint["source_artifact"].clone();
        unknown["unexpected"] = Value::Bool(true);
        assert!(serde_json::from_value::<VectorizedEpisodeCheckpoint<i32>>(unknown).is_err());
    }

    #[test]
    fn historical_same_schema_families_reject_source_and_result_tampering() {
        let root = workspace_root();
        let fixture_root = root.join("tests/golden/compatibility");

        let dataset: Value = serde_json::from_slice(
            &fs::read(fixture_root.join("dataset-bundle-v1-aecafb6.json")).unwrap(),
        )
        .unwrap();
        let dataset: HistoricalCompatibilityDecisionFixture =
            serde_json::from_value(dataset).unwrap();
        validate_historical_compatibility_decision("dataset_bundle_v1_aecafb6", &dataset).unwrap();
        let mut corrupted = dataset.clone();
        let contents = &mut corrupted.source_files[0].contents_hex;
        contents.replace_range(
            ..2,
            if contents.starts_with("00") {
                "ff"
            } else {
                "00"
            },
        );
        assert!(validate_historical_compatibility_decision(
            "dataset_bundle_v1_aecafb6",
            &corrupted,
        )
        .is_err());
        let mut forged_result = dataset;
        forged_result.expected_result_sha256 = Some(format!("sha256:{}", "0".repeat(64)));
        assert!(validate_historical_compatibility_decision(
            "dataset_bundle_v1_aecafb6",
            &forged_result,
        )
        .is_err());

        for (fixture_id, file_name) in [
            ("task_spec_v1_70a9ff3", "task-spec-v1-70a9ff3.json"),
            (
                "failure_capsule_v1_61d6c81",
                "failure-capsule-v1-61d6c81.json",
            ),
            (
                "frontend_transport_v1_be53f16",
                "frontend-transport-v1-be53f16.json",
            ),
        ] {
            let value: Value =
                serde_json::from_slice(&fs::read(fixture_root.join(file_name)).unwrap()).unwrap();
            let fixture: HistoricalCompatibilityDecisionFixture =
                serde_json::from_value(value).unwrap();
            validate_historical_compatibility_decision(fixture_id, &fixture).unwrap();
            let mut retargeted = fixture;
            retargeted.source_revision = "0".repeat(40);
            assert!(validate_historical_compatibility_decision(fixture_id, &retargeted).is_err());
        }

        let frontend: Value = serde_json::from_slice(
            &fs::read(fixture_root.join("frontend-transport-v1-be53f16.json")).unwrap(),
        )
        .unwrap();
        let mut frontend: HistoricalCompatibilityDecisionFixture =
            serde_json::from_value(frontend).unwrap();
        frontend.source_artifact["frame_hex"] = Value::String("00".repeat(72));
        assert!(validate_historical_compatibility_decision(
            "frontend_transport_v1_be53f16",
            &frontend,
        )
        .is_err());
    }

    #[test]
    fn digest_tampering_produces_a_failed_bounded_report() {
        let root = workspace_root();
        let source = root.join("release/compatibility-fixtures.toml");
        let mut registry = read_registry(&source).unwrap();
        registry.fixtures[0].canonical_json_sha256 = format!("sha256:{}", "0".repeat(64));
        let temp = tempfile::tempdir().unwrap();
        let registry_path = temp.path().join("fixtures.toml");
        fs::write(&registry_path, toml::to_string(&registry).unwrap()).unwrap();
        let report = run_compatibility(&root, &registry_path).unwrap();
        assert!(!report.passed);
        assert!(!report.checks[0].passed);
        assert!(report.checks[0].detail.contains("digest mismatch"));
        assert!(report.checks[0].detail.chars().count() <= MAX_DETAIL_CHARS);
    }
}
