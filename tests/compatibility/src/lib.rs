//! Cross-version compatibility fixture verification for RNE release artifacts.
//!
//! The suite consumes the installed `release/compatibility-fixtures.toml`
//! registry. Every registered JSON artifact must still pass its typed reader,
//! while deterministic mutations prove that an unsupported schema and an
//! unknown top-level field fail closed.

#![deny(missing_docs)]

use anyhow::{bail, ensure, Context};
use rne_ai::{
    BehaviorReplayArtifact, MobileManipulatorSim, MobileManipulatorSimSnapshot,
    PortableBatchCheckpoint, PortableBatchOperation, TaskSpec,
    MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION, MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
};
use rne_data::transport::{
    negotiate_transport, ClientHello, NegotiationPolicy, NegotiationRejectCode,
    SensorFrameMetadata, TransportCapabilities, TransportFrame, TransportMessageKind,
    TRANSPORT_MAX_PAYLOAD_BYTES,
};
use rne_data::{
    decode_dataset_action, decode_dataset_annotation, decode_dataset_imu,
    decode_dataset_task_outcome, decode_dataset_transform, encode_dataset_action,
    encode_dataset_annotation, encode_dataset_imu, encode_dataset_task_outcome,
    encode_dataset_transform, DatasetActionSample, DatasetGroundTruthAnnotation, DatasetManifest,
    DatasetTaskOutcomeSample, DepthPairEvaluationReport, ImuSample, PoseSample,
    DATASET_PAYLOAD_SCHEMA_VERSION,
};
use rne_hardware_gateway::mock::MockConformanceReport;
use rne_log::{FailureCapsule, ReplayArtifact};
use rne_math::Vec3;
use rne_openscenario::ScenarioReplayArtifact;
use rne_physics_conformance::ExternalPhysicsBackendConformanceReport;
use rne_physics_conformance_suite::ConformanceReport;
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
use std::path::{Component, Path, PathBuf};

/// Stable compatibility report discriminator.
pub const COMPATIBILITY_FIXTURE_REPORT_KIND: &str = "rne_compatibility_fixture_report";
/// Current compatibility report schema.
pub const COMPATIBILITY_FIXTURE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Current registry schema.
pub const COMPATIBILITY_FIXTURE_REGISTRY_SCHEMA_VERSION: u32 = 1;

const MAX_FIXTURE_BYTES: u64 = 4 * 1024 * 1024;
const MAX_REGISTRY_BYTES: u64 = 256 * 1024;
const MAX_DETAIL_CHARS: usize = 240;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FixtureSpec {
    id: &'static str,
    contract: &'static str,
    schema_version: u32,
    version_field: &'static str,
}

const FIXTURE_SPECS: [FixtureSpec; 15] = [
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
        id: "dataset_bundle_v1",
        contract: "dataset_bundle",
        schema_version: 1,
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
        id: "generic_replay_v1",
        contract: "generic_replay",
        schema_version: 1,
        version_field: "version",
    },
    FixtureSpec {
        id: "frontend_transport_v1",
        contract: "frontend_transport",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "hardware_mock_conformance_v1",
        contract: "hardware_mock_conformance",
        schema_version: 1,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "mobile_manipulator_snapshot_v1_to_v3",
        contract: "historical_mobile_manipulator_snapshot",
        schema_version: 1,
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
        id: "scenario_replay_v4",
        contract: "scenario_replay",
        schema_version: 4,
        version_field: "schema_version",
    },
    FixtureSpec {
        id: "task_spec_v1",
        contract: "task_spec",
        schema_version: 1,
        version_field: "schema_version",
    },
];

const CONTROLLER_C_ABI_LAYOUT_KIND: &str = "rne_controller_c_abi_layout";
const HISTORICAL_MIGRATION_KIND: &str = "rne_historical_migration_case";
const HISTORICAL_MIGRATION_FLOAT_TOLERANCE: f64 = 1.0e-9;

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
                    if spec.contract == "historical_mobile_manipulator_snapshot" {
                        "accepted within 1e-9; future schema and unknown field rejected".to_string()
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

    validate_typed(root, spec, value.clone()).context("accepted fixture was rejected")?;

    let mut future = value.clone();
    let future_object = future
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    future_object.insert(
        spec.version_field.to_string(),
        Value::from(u64::from(spec.schema_version) + 10_000),
    );
    let future_schema_rejected = validate_typed(root, spec, future).is_err();

    let mut unknown = value;
    let unknown_object = unknown
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    unknown_object.insert(
        "rne_unknown_compatibility_field".to_string(),
        Value::Bool(true),
    );
    let unknown_field_rejected = validate_typed(root, spec, unknown).is_err();
    Ok((true, future_schema_rejected, unknown_field_rejected))
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
        "behavior_replay" => {
            let fixture: BehaviorReplayArtifact = serde_json::from_value(value)?;
            fixture.validate_compatibility()?;
        }
        "controller_c_abi" => {
            let fixture: ControllerCAbiFixture = serde_json::from_value(value)?;
            validate_controller_c_abi(&fixture)?;
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
        "frontend_transport" => {
            let fixture: FrontendTransportFixture = serde_json::from_value(value)?;
            validate_frontend_transport(&fixture)?;
        }
        "hardware_mock_conformance" => {
            let fixture: MockConformanceReport = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "historical_mobile_manipulator_snapshot" => {
            let fixture: HistoricalMigrationFixture = serde_json::from_value(value)?;
            validate_historical_mobile_manipulator_snapshot(root, &fixture)?;
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
