//! Cross-platform 1.0 RC bundle assembly and installed-artifact rehearsal.

use super::{
    cargo_metadata, fuzz_smoke, release_readiness, supply_chain, validate_blocker_registry,
    validate_contract_registry, validate_release_metadata, workspace_root, RELEASE_VERSION,
};
use anyhow::{bail, Context};
use rne_accelerator_contract::AcceleratorScaffoldContract;
use rne_plugin::ControllerPluginScaffoldContract;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::ffi::{OsStr, OsString};
use std::fs;
use std::io::Read;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Output};

/// Machine-readable release provenance report schema.
pub(crate) const RELEASE_REPORT_SCHEMA_VERSION: u32 = 2;
/// Machine-readable installed-bundle rehearsal report schema.
pub(crate) const INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION: u32 = 7;
/// Archive-bound independently extracted rehearsal report schema.
pub(crate) const ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION: u32 = 2;
/// Installed Python public-API contract schema.
pub(crate) const PYTHON_API_CONTRACT_SCHEMA_VERSION: u32 = 1;
/// Installed Python public-API verification report schema.
pub(crate) const PYTHON_API_REPORT_SCHEMA_VERSION: u32 = 1;
/// Bundled MuJoCo runtime provenance manifest schema.
pub(crate) const MUJOCO_RUNTIME_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Independently produced installed flagship reproduction report schema.
pub(crate) const EXTERNAL_FLAGSHIP_REPRODUCTION_REPORT_SCHEMA_VERSION: u32 = 2;
const EXTERNAL_FLAGSHIP_SUBMISSION_SCHEMA_VERSION: u32 = 2;
const EXTERNAL_FLAGSHIP_SUBMISSION_KIND: &str = "rne_external_flagship_submission_candidate";
const EXTERNAL_FLAGSHIP_CANDIDATE_STATUS: &str = "not_accepted_pending_maintainer_verification";

const RELEASE_BINARY_PACKAGES: [(&str, &str); 11] = [
    ("rne_asset_cli", "rne-asset"),
    ("rne_compatibility_suite", "rne-compatibility"),
    ("rne_accelerator_contract", "rne-accelerator-conformance"),
    ("rne_accelerator_contract", "rne-accelerator-protocol-mock"),
    ("rne_physics_conformance_suite", "rne-physics-conformance"),
    ("rne_scenario_scale", "rne-scenario-scale"),
    ("rne_hardware_gateway", "rne-hardware-conformance"),
    ("rne_hardware_gateway", "rne-hardware-mock-device"),
    ("rne_hardware_gateway", "rne-simulator-conformance"),
    ("rne_hardware_gateway", "rne-simulator-mock-adapter"),
    ("flagship_validation_workflow", "rne-flagship-proof"),
];
const RELEASE_PLUGIN_PACKAGE: &str = "rne_plugin_example_velocity_servo";
const MUJOCO_VERSION: &str = "3.9.0";
const MUJOCO_ARCHIVE_PATH_ENV: &str = "MUJOCO_ARCHIVE_PATH";
const MUJOCO_RUNTIME_ROOT_ENV: &str = "MUJOCO_RUNTIME_ROOT";
const MUJOCO_DYNAMIC_LINK_DIR_ENV: &str = "MUJOCO_DYNAMIC_LINK_DIR";
const MUJOCO_LINUX_ARCHIVE: &str = "mujoco-3.9.0-linux-x86_64.tar.gz";
const MUJOCO_LINUX_ARCHIVE_SHA256: &str =
    "d11f281540d0d1844e2923bf43b6fff5ad186ec55927a8dae0eb26b9e579eed2";
const MUJOCO_LINUX_RUNTIME_SHA256: &str =
    "526773636a795dad11e094c8655d2375984a5cd7090f254d86bb71074651b852";
const MUJOCO_LINUX_LICENSE_SHA256: &str =
    "cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30";
const MUJOCO_LINUX_NOTICES_SHA256: &str =
    "0fa07f5d8fb8d19ca6d383fabdc0af86052df48f952b0e370e89d4018c36afdf";
const MUJOCO_WINDOWS_ARCHIVE: &str = "mujoco-3.9.0-windows-x86_64.zip";
const MUJOCO_WINDOWS_ARCHIVE_SHA256: &str =
    "544f44a8a7df3e94648a7eaf41500f4456eb59f9f01df3ec2cfb03bdbf5c2bb9";
const MUJOCO_WINDOWS_RUNTIME_SHA256: &str =
    "d2119d435ef68ceb114d01bc3658eff42ac23ebcc08adca94d9b1ff0b9eb0d0e";
const MUJOCO_WINDOWS_LICENSE_SHA256: &str =
    "3ddf9be5c28fe27dad143a5dc76eea25222ad1dd68934a047064e56ed2fa40c5";
const MUJOCO_WINDOWS_NOTICES_SHA256: &str =
    "5ac8b2055ea0f52d37738cbb1086c9d44c73a5470f6c93e6ed31c47953e1cf0d";
const SHA256_MANIFEST: &str = "SHA256SUMS";
const RELEASE_REPORT: &str = "release-report.json";
const INSTALL_REPORT: &str = "install-rehearsal-report.json";
const ARCHIVE_INSTALL_REPORT: &str = "archive-install-rehearsal-report.json";
const ARCHIVE_INSTALL_REPORT_KIND: &str = "rne_archive_install_rehearsal";
const INSTALL_CHECK_IDS: [&str; 12] = [
    "robot_replay",
    "flagship_proof",
    "scenario_replay",
    "physics_conformance",
    "scenario_scale_100",
    "hardware_adapter",
    "simulator_adapter",
    "accelerator_protocol",
    "controller_plugin",
    "compatibility_corpus",
    "python_wheel",
    "python_api",
];
const EXTERNAL_FLAGSHIP_REQUIRED_PROOF_PATHS: [&str; 5] = [
    "flagship-proof/installed-proof-report.json",
    "flagship-proof/time-to-proof-report.json",
    "flagship-proof/cross-backend-report.json",
    "flagship-proof/recorded-shadow-proof.json",
    "flagship-proof/failure-capsule/capsule.json",
];
const MAX_EXTERNAL_SUBMISSION_BYTES: u64 = 128 * 1024;
const MAX_EXTERNAL_LOG_BYTES: u64 = 16 * 1024 * 1024;

const BUNDLE_FILES: [(&str, &str); 91] = [
    ("README.md", "README.md"),
    ("CHANGELOG.md", "CHANGELOG.md"),
    ("LICENSE-MIT", "LICENSE-MIT"),
    ("LICENSE-APACHE", "LICENSE-APACHE"),
    ("Cargo.lock", "Cargo.lock"),
    ("docs/COMPATIBILITY.md", "COMPATIBILITY.md"),
    ("docs/SUPPORT.md", "SUPPORT.md"),
    ("docs/RELEASE_INSTALL.md", "INSTALL.md"),
    ("docs/ONE_ZERO_READINESS.md", "ONE_ZERO_READINESS.md"),
    ("docs/EVIDENCE_QUICKSTART.md", "docs/EVIDENCE_QUICKSTART.md"),
    ("docs/FAILURE_CAPSULE.md", "docs/FAILURE_CAPSULE.md"),
    (
        "docs/EXTERNAL_EVIDENCE_INTAKE.md",
        "docs/EXTERNAL_EVIDENCE_INTAKE.md",
    ),
    (
        "docs/EXTERNAL_FLAGSHIP_REPRODUCTION.md",
        "docs/EXTERNAL_FLAGSHIP_REPRODUCTION.md",
    ),
    ("docs/PLUGIN_SDK.md", "docs/PLUGIN_SDK.md"),
    (
        "docs/EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md",
        "docs/EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md",
    ),
    (
        "docs/HARDWARE_ADAPTER_CONFORMANCE.md",
        "docs/HARDWARE_ADAPTER_CONFORMANCE.md",
    ),
    (
        "docs/EXTERNAL_SIMULATOR_ADAPTER_CONFORMANCE.md",
        "docs/EXTERNAL_SIMULATOR_ADAPTER_CONFORMANCE.md",
    ),
    (
        "docs/ACCELERATOR_PROTOCOL.md",
        "docs/ACCELERATOR_PROTOCOL.md",
    ),
    (
        "crates/rne_plugin_sdk/src/abi.rs",
        "sdk/rust/rne_plugin_sdk.rs",
    ),
    (
        "crates/rne_plugin_sdk/include/rne_plugin_sdk.h",
        "sdk/c/rne_plugin_sdk.h",
    ),
    ("release/blockers.toml", "release/blockers.toml"),
    (
        "release/one-zero-readiness.toml",
        "release/one-zero-readiness.toml",
    ),
    (
        "release/external-evidence-intake.toml",
        "release/external-evidence-intake.toml",
    ),
    (
        "release/external-flagship-submission-template.json",
        "release/external-flagship-submission-template.json",
    ),
    (
        "release/external-project-submission-template.json",
        "release/external-project-submission-template.json",
    ),
    (
        "release/external-plugin-submission-template.json",
        "release/external-plugin-submission-template.json",
    ),
    (
        "release/external-simulator-submission-template.json",
        "release/external-simulator-submission-template.json",
    ),
    (
        "release/evidence/compatibility-report-v1.json",
        "release/evidence/compatibility-report-v1.json",
    ),
    ("release/exit-matrix.toml", "release/exit-matrix.toml"),
    (
        "release/compatibility-fixtures.toml",
        "release/compatibility-fixtures.toml",
    ),
    (
        "release/rust-api-baseline.toml",
        "release/rust-api-baseline.toml",
    ),
    (
        "release/artifact-attestation.toml",
        "release/artifact-attestation.toml",
    ),
    ("release/python_wheel_smoke.py", "python-wheel-smoke.py"),
    ("release/python_api_compat.py", "python-api-compat.py"),
    (
        "release/python-api-v1.json",
        "sdk/python/rne_py-api-v1.json",
    ),
    (
        "assets/runs/mesh_diff_drive.rne.run.toml",
        "assets/runs/mesh_diff_drive.rne.run.toml",
    ),
    (
        "assets/tasks/diff_drive_goal.task.json",
        "assets/tasks/diff_drive_goal.task.json",
    ),
    (
        "adapters/hardware/rne_hardware_gateway/tests/fixtures/simulator/runtime.json",
        "adapters/simulator/reference/runtime.json",
    ),
    (
        "adapters/hardware/rne_hardware_gateway/tests/fixtures/simulator/world.sdf",
        "adapters/simulator/reference/world.sdf",
    ),
    (
        "adapters/hardware/rne_hardware_gateway/tests/fixtures/simulator/robot.urdf",
        "adapters/simulator/reference/robot.urdf",
    ),
    (
        "adapters/hardware/rne_hardware_gateway/tests/fixtures/simulator/adapter.toml",
        "adapters/simulator/reference/adapter.toml",
    ),
    (
        "adapters/mjx/accelerator.toml",
        "adapters/mjx/accelerator.toml",
    ),
    ("adapters/mjx/runtime.toml", "adapters/mjx/runtime.toml"),
    (
        "adapters/mjx/fixtures/free-fall-task-spec-v1.json",
        "adapters/mjx/fixtures/free-fall-task-spec-v1.json",
    ),
    (
        "adapters/mjx/fixtures/free-fall-v1.xml",
        "adapters/mjx/fixtures/free-fall-v1.xml",
    ),
    (
        "tests/golden/accelerators/capability-report-v1.json",
        "tests/golden/accelerators/capability-report-v1.json",
    ),
    (
        "tests/golden/accelerators/conformance-report-v1.json",
        "tests/golden/accelerators/conformance-report-v1.json",
    ),
    (
        "tests/golden/accelerators/process-conformance-report-v1.json",
        "tests/golden/accelerators/process-conformance-report-v1.json",
    ),
    (
        "tests/golden/accelerators/protocol-transcript-v1.json",
        "tests/golden/accelerators/protocol-transcript-v1.json",
    ),
    (
        "tests/golden/accelerators/scaffold-contract-v1.json",
        "tests/golden/accelerators/scaffold-contract-v1.json",
    ),
    (
        "tests/golden/accelerators/scale-report-v1.json",
        "tests/golden/accelerators/scale-report-v1.json",
    ),
    (
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
    ),
    (
        "assets/scenes/mm_minimal.rne.scene.toml",
        "assets/scenes/mm_minimal.rne.scene.toml",
    ),
    (
        "assets/scenes/mm_mobile_lift_pick_place.rne.scene.toml",
        "assets/scenes/mm_mobile_lift_pick_place.rne.scene.toml",
    ),
    (
        "assets/robots/mm_mobile_lift.rne.robot.toml",
        "assets/robots/mm_mobile_lift.rne.robot.toml",
    ),
    (
        "assets/robots/mm_mobile_lift/mm_mobile_lift.urdf",
        "assets/robots/mm_mobile_lift/mm_mobile_lift.urdf",
    ),
    (
        "assets/robots/mm_minimal.rne.robot.toml",
        "assets/robots/mm_minimal.rne.robot.toml",
    ),
    (
        "assets/robots/mm_minimal/mm_minimal.urdf",
        "assets/robots/mm_minimal/mm_minimal.urdf",
    ),
    (
        "assets/robots/mesh_diff_drive.rne.robot.toml",
        "assets/robots/mesh_diff_drive.rne.robot.toml",
    ),
    (
        "assets/robots/mesh_diff_drive/mesh_diff_drive.urdf",
        "assets/robots/mesh_diff_drive/mesh_diff_drive.urdf",
    ),
    (
        "assets/robots/mesh_diff_drive/meshes/base_link.stl",
        "assets/robots/mesh_diff_drive/meshes/base_link.stl",
    ),
    (
        "assets/runs/scenario_speed.rne.run.toml",
        "assets/runs/scenario_speed.rne.run.toml",
    ),
    (
        "tests/golden/replays/behavior-replay-v1.json",
        "tests/golden/replays/behavior-replay-v1.json",
    ),
    (
        "tests/golden/plugins/controller-c-abi-layout-v3.json",
        "tests/golden/plugins/controller-c-abi-layout-v3.json",
    ),
    (
        "tests/golden/plugins/controller-plugin-conformance-v1.json",
        "tests/golden/plugins/controller-plugin-conformance-v1.json",
    ),
    (
        "tests/golden/plugins/controller-scaffold-v1.json",
        "tests/golden/plugins/controller-scaffold-v1.json",
    ),
    (
        "tests/golden/datasets/bundle-manifest-v1.json",
        "tests/golden/datasets/bundle-manifest-v1.json",
    ),
    (
        "tests/golden/compatibility/dataset-bundle-v1-aecafb6.json",
        "tests/golden/compatibility/dataset-bundle-v1-aecafb6.json",
    ),
    (
        "tests/golden/datasets/depth-pair-evaluation-v1.json",
        "tests/golden/datasets/depth-pair-evaluation-v1.json",
    ),
    (
        "tests/golden/datasets/native-payload-v1.json",
        "tests/golden/datasets/native-payload-v1.json",
    ),
    (
        "tests/golden/evidence/failure-capsule-v1.json",
        "tests/golden/evidence/failure-capsule-v1.json",
    ),
    (
        "tests/golden/compatibility/failure-capsule-v1-61d6c81.json",
        "tests/golden/compatibility/failure-capsule-v1-61d6c81.json",
    ),
    (
        "tests/golden/replays/generic-replay-v1.json",
        "tests/golden/replays/generic-replay-v1.json",
    ),
    (
        "tests/golden/protocol/frontend-message-families-v1.json",
        "tests/golden/protocol/frontend-message-families-v1.json",
    ),
    (
        "tests/golden/protocol/frontend-transport-v1.json",
        "tests/golden/protocol/frontend-transport-v1.json",
    ),
    (
        "tests/golden/compatibility/frontend-transport-v1-be53f16.json",
        "tests/golden/compatibility/frontend-transport-v1-be53f16.json",
    ),
    (
        "tests/golden/hardware/gateway-mock-conformance-v1.json",
        "tests/golden/hardware/gateway-mock-conformance-v1.json",
    ),
    (
        "tests/golden/hardware/gateway-process-disconnect-session-v1.json",
        "tests/golden/hardware/gateway-process-disconnect-session-v1.json",
    ),
    (
        "tests/golden/migrations/mobile-manipulator-snapshot-v1-to-v3.json",
        "tests/golden/migrations/mobile-manipulator-snapshot-v1-to-v3.json",
    ),
    (
        "tests/golden/migrations/mobile-manipulator-snapshot-v1-47525b1-to-v3.json",
        "tests/golden/migrations/mobile-manipulator-snapshot-v1-47525b1-to-v3.json",
    ),
    (
        "tests/golden/migrations/mobile-manipulator-snapshot-v2-2255cbe-to-v3.json",
        "tests/golden/migrations/mobile-manipulator-snapshot-v2-2255cbe-to-v3.json",
    ),
    (
        "tests/golden/physics/conformance-report-v2.json",
        "tests/golden/physics/conformance-report-v2.json",
    ),
    (
        "crates/rne_physics_conformance/tests/golden/external-backend-conformance-v1.json",
        "crates/rne_physics_conformance/tests/golden/external-backend-conformance-v1.json",
    ),
    (
        "tests/golden/tasks/vectorized-checkpoint-v2.json",
        "tests/golden/tasks/vectorized-checkpoint-v2.json",
    ),
    (
        "tests/golden/datasets/renderer-capture-report-v1.json",
        "tests/golden/datasets/renderer-capture-report-v1.json",
    ),
    (
        "tests/golden/compatibility/scenario-replay-v2-533729d-requires-rerun.json",
        "tests/golden/compatibility/scenario-replay-v2-533729d-requires-rerun.json",
    ),
    (
        "tests/golden/compatibility/scenario-replay-v3-e959e3f-requires-rerun.json",
        "tests/golden/compatibility/scenario-replay-v3-e959e3f-requires-rerun.json",
    ),
    (
        "tests/golden/replays/scenario-replay-v4.json",
        "tests/golden/replays/scenario-replay-v4.json",
    ),
    (
        "tests/golden/tasks/task-spec-v1.json",
        "tests/golden/tasks/task-spec-v1.json",
    ),
    (
        "tests/golden/compatibility/task-spec-v1-70a9ff3.json",
        "tests/golden/compatibility/task-spec-v1-70a9ff3.json",
    ),
    (
        "tests/golden/compatibility/vectorized-episode-checkpoint-v1-bd4d44f.json",
        "tests/golden/compatibility/vectorized-episode-checkpoint-v1-bd4d44f.json",
    ),
];
const SCENARIO_FILES: [(&str, &str); 2] = [
    ("assets/scenarios/speed.xosc", "assets/scenarios/speed.xosc"),
    (
        "assets/traffic/corridor.rne.traffic.json",
        "assets/traffic/corridor.rne.traffic.json",
    ),
];

#[derive(Debug)]
struct BundleOptions {
    target: String,
    wheel: PathBuf,
    output_dir: PathBuf,
    expected_tag: Option<String>,
    python: PathBuf,
    allow_dirty: bool,
}

#[derive(Debug)]
struct InstallOptions {
    archive: PathBuf,
    bundle_dir: PathBuf,
    output_dir: PathBuf,
    python: PathBuf,
}

#[derive(Debug)]
struct ExternalFlagshipOptions {
    archive: PathBuf,
    bundle_dir: PathBuf,
    proof_dir: PathBuf,
    proof_bundle: PathBuf,
    submission: PathBuf,
    evidence_repo_dir: PathBuf,
    revision: String,
    output: PathBuf,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ExternalFlagshipSubmissionCandidate {
    kind: String,
    schema_version: u32,
    candidate_status: String,
    author_assistance: bool,
    evidence_repository: SubmissionRepository,
    measurement: SubmissionMeasurement,
    release_archive: SubmissionArtifact,
    proof_bundle: SubmissionArtifact,
    required_proof_paths: Vec<String>,
    reproduction: SubmissionReproduction,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionRepository {
    owner: String,
    url: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionMeasurement {
    measured_on: String,
    machine_label: String,
    operating_system: String,
    architecture: String,
    release_target: String,
    elapsed_ms: u64,
    target_ms: u64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionArtifact {
    url: String,
    file_name: String,
    size_bytes: u64,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionReproduction {
    commands: Vec<String>,
    exit_statuses: Vec<i32>,
    stdout_log_path: String,
    stderr_log_path: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct MemberDigest {
    path: String,
    size_bytes: u64,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct AuditVerdicts {
    cargo_deny: String,
    cargo_audit: String,
    source_policy: String,
    license_policy: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct InstallCheck {
    id: String,
    status: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct InstallRehearsalReport {
    schema_version: u32,
    release_version: String,
    target: String,
    status: String,
    checks: Vec<InstallCheck>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct InstalledFlagshipProofReport {
    kind: String,
    schema_version: u32,
    status: String,
    task_id: String,
    physics_execution_paths: Vec<String>,
    success_status: String,
    expected_failure_contract: String,
    first_violation_step: u64,
    capsule_verified: bool,
    recorded_shadow_status: Option<String>,
    recorded_shadow_case_count: usize,
    installed_bundle_verified: bool,
    bundle_verification_report: Option<MemberDigest>,
    producer_executable: MemberDigest,
    artifacts: Vec<MemberDigest>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct InstalledRecordedShadowCase {
    id: String,
    mode: rne_hardware_gateway::HardwareMode,
    expected_status: String,
    observed_status: String,
    accepted_samples: usize,
    violating_elements: usize,
    first_divergence_tensor: Option<String>,
    suppressed_actions: usize,
    actuator_writes_emitted: bool,
    session: String,
    report: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct InstalledRecordedShadowProof {
    kind: String,
    schema_version: u32,
    status: String,
    task_id: String,
    controller_id: String,
    clock_source: String,
    cases: Vec<InstalledRecordedShadowCase>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct MujocoRuntimeManifest {
    kind: String,
    schema_version: u32,
    version: String,
    source_url: String,
    archive_file: String,
    archive_sha256: String,
    runtime_members: Vec<MemberDigest>,
    license_members: Vec<MemberDigest>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct TimeToProofReport {
    kind: String,
    schema_version: u32,
    status: String,
    task_id: String,
    machine_label: String,
    operating_system: String,
    architecture: String,
    measurement_scope: String,
    elapsed_ms: u64,
    target_ms: u64,
    within_target: bool,
    installed_bundle_verification: MemberDigest,
    installed_proof_report: MemberDigest,
    failure_capsule_manifest: MemberDigest,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ArchiveSubject {
    file: String,
    size_bytes: u64,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ArchiveInstallRehearsalReport {
    kind: String,
    schema_version: u32,
    archive: ArchiveSubject,
    bundle_root: String,
    release_report: MemberDigest,
    checksum_manifest: MemberDigest,
    time_to_proof: MemberDigest,
    rehearsal: InstallRehearsalReport,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ExternalFlagshipReproductionReport {
    kind: String,
    schema_version: u32,
    status: String,
    owner: String,
    repository: String,
    revision: String,
    measured_on: String,
    author_assistance: bool,
    release_version: String,
    release_revision: String,
    release_target: String,
    machine_label: String,
    operating_system: String,
    architecture: String,
    elapsed_ms: u64,
    target_ms: u64,
    task_id: String,
    physics_execution_paths: Vec<String>,
    first_violation_step: u64,
    first_violation_sim_time_ticks: u64,
    archive: MemberDigest,
    proof_bundle: MemberDigest,
    submission_candidate: MemberDigest,
    stdout_log: MemberDigest,
    stderr_log: MemberDigest,
    release_report: MemberDigest,
    checksum_manifest: MemberDigest,
    producer_executable: MemberDigest,
    installed_proof_report: MemberDigest,
    time_to_proof_report: MemberDigest,
    cross_backend_report: MemberDigest,
    failure_capsule_manifest: MemberDigest,
}

pub(crate) struct StagedExternalFlagshipReproduction<'a> {
    pub(crate) owner: &'a str,
    pub(crate) repository: &'a str,
    pub(crate) revision: &'a str,
    pub(crate) measured_on: &'a str,
    pub(crate) release_archive: &'a Path,
    pub(crate) proof_bundle: &'a Path,
    pub(crate) submission_candidate: &'a Path,
    pub(crate) stdout_log: &'a Path,
    pub(crate) stderr_log: &'a Path,
}

impl InstallRehearsalReport {
    fn all_passed(&self) -> bool {
        self.status == "passed"
            && self
                .checks
                .iter()
                .map(|check| check.id.as_str())
                .eq(INSTALL_CHECK_IDS)
            && self.checks.iter().all(|check| check.status == "passed")
    }

    fn verdicts(&self) -> BTreeMap<String, String> {
        self.checks
            .iter()
            .map(|check| (check.id.clone(), check.status.clone()))
            .collect()
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ReleaseReport {
    schema_version: u32,
    release_version: String,
    git_commit: String,
    target: String,
    rustc_version: String,
    cargo_version: String,
    cargo_lock_sha256: String,
    clean_worktree: bool,
    expected_tag: Option<String>,
    tag_matches_commit: bool,
    reproducible: bool,
    audit: AuditVerdicts,
    fuzz_campaign_digest_sha256: String,
    contracts: serde_json::Value,
    #[serde(alias = "flagship_workflows")]
    installed_workflows: BTreeMap<String, String>,
    members: Vec<MemberDigest>,
}

pub(crate) struct ReadinessReleaseEvidence<'a> {
    pub(crate) archive_path: &'a Path,
    pub(crate) archive_sha256: &'a str,
    pub(crate) release_report_path: &'a Path,
    pub(crate) release_report_sha256: &'a str,
    pub(crate) checksum_manifest_path: &'a Path,
    pub(crate) checksum_manifest_sha256: &'a str,
    pub(crate) install_report_path: &'a Path,
    pub(crate) install_report_sha256: &'a str,
}

pub(crate) struct ReadinessReleaseIdentity<'a> {
    pub(crate) target: &'a str,
    pub(crate) commit: &'a str,
    pub(crate) tag: &'a str,
}

fn validate_archive_subject(
    subject: &ArchiveSubject,
    archive_path: &Path,
    expected_sha256: &str,
    target: &str,
) -> anyhow::Result<()> {
    let metadata = fs::symlink_metadata(archive_path)
        .with_context(|| format!("inspect release archive {}", archive_path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file(),
        "release archive must be a regular file"
    );
    let actual_file = archive_path
        .file_name()
        .and_then(OsStr::to_str)
        .context("release archive file name is not valid Unicode")?;
    let expected_file = if target.contains("windows") {
        format!("{}.zip", bundle_name(target))
    } else {
        format!("{}.tar.gz", bundle_name(target))
    };
    anyhow::ensure!(
        actual_file == expected_file && subject.file == expected_file,
        "archive-install report names the wrong release archive"
    );
    validate_prefixed_sha256("archive-install archive", &subject.sha256)?;
    let actual_sha256 = format!("sha256:{}", sha256_file_hex(archive_path)?);
    anyhow::ensure!(
        subject.size_bytes == metadata.len()
            && subject.sha256 == expected_sha256
            && subject.sha256 == actual_sha256,
        "archive-install report does not bind the exact release archive bytes"
    );
    Ok(())
}

fn read_bound_file(path: &Path, expected_sha256: &str, label: &str) -> anyhow::Result<Vec<u8>> {
    validate_prefixed_sha256(label, expected_sha256)?;
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file(),
        "{label} must be a regular file"
    );
    let bytes = fs::read(path).with_context(|| format!("read {label} {}", path.display()))?;
    anyhow::ensure!(
        format!("sha256:{}", sha256_hex(&bytes)) == expected_sha256,
        "{label} changed after evidence verification"
    );
    Ok(bytes)
}

fn validate_member_identity(
    identity: &MemberDigest,
    expected_path: &str,
    bytes: &[u8],
) -> anyhow::Result<()> {
    anyhow::ensure!(
        identity.path == expected_path,
        "archive-install subject path mismatch: expected {expected_path}, got {}",
        identity.path
    );
    validate_sha256_hex("archive-install subject", &identity.sha256)?;
    anyhow::ensure!(
        identity.size_bytes == u64::try_from(bytes.len()).unwrap_or(u64::MAX)
            && identity.sha256 == sha256_hex(bytes),
        "archive-install subject {expected_path} does not match its retained bytes"
    );
    Ok(())
}

fn validate_release_members(members: &[MemberDigest]) -> anyhow::Result<()> {
    let mut previous = None;
    for member in members {
        anyhow::ensure!(
            !member.path.contains('\\'),
            "release member path must use forward slashes: {}",
            member.path
        );
        validate_relative_member(Path::new(&member.path))?;
        validate_sha256_hex("release member", &member.sha256)?;
        if let Some(previous) = previous {
            anyhow::ensure!(
                previous < member.path.as_str(),
                "release members must be unique and sorted"
            );
        }
        anyhow::ensure!(
            member.path != RELEASE_REPORT && member.path != SHA256_MANIFEST,
            "release member list contains a self-referential report or checksum manifest"
        );
        previous = Some(member.path.as_str());
    }
    Ok(())
}

fn validate_release_checksum_chain(
    release: &ReleaseReport,
    release_bytes: &[u8],
    checksum_bytes: &[u8],
    rehearsal: &InstallRehearsalReport,
) -> anyhow::Result<()> {
    let declared = parse_sha256_manifest(checksum_bytes)?;
    let mut expected = release
        .members
        .iter()
        .map(|member| (member.path.clone(), member.sha256.clone()))
        .collect::<BTreeMap<_, _>>();
    anyhow::ensure!(
        expected
            .insert(RELEASE_REPORT.to_string(), sha256_hex(release_bytes))
            .is_none(),
        "release report unexpectedly listed itself as a payload member"
    );
    anyhow::ensure!(
        declared == expected,
        "retained SHA256SUMS does not match the release report member graph"
    );

    let inner_bytes = pretty_json_bytes(rehearsal)?;
    let inner = release
        .members
        .iter()
        .find(|member| member.path == INSTALL_REPORT)
        .context("release members omitted the staged install rehearsal")?;
    validate_member_identity(inner, INSTALL_REPORT, &inner_bytes)?;
    Ok(())
}

fn validate_sha256_hex(label: &str, digest: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        digest.len() == 64
            && digest
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "{label} SHA-256 must be 64 lowercase hexadecimal characters"
    );
    Ok(())
}

fn validate_prefixed_sha256(label: &str, digest: &str) -> anyhow::Result<()> {
    let hex = digest
        .strip_prefix("sha256:")
        .with_context(|| format!("{label} SHA-256 must use the sha256: prefix"))?;
    validate_sha256_hex(label, hex)
}

/// Validates the two reports retained for one independently reproduced release artifact.
pub(crate) fn validate_readiness_release_reports(
    evidence: ReadinessReleaseEvidence<'_>,
    expected: ReadinessReleaseIdentity<'_>,
) -> anyhow::Result<()> {
    let release_bytes = read_bound_file(
        evidence.release_report_path,
        evidence.release_report_sha256,
        "release report",
    )?;
    let release: ReleaseReport = serde_json::from_slice(&release_bytes)
        .with_context(|| format!("parse {}", evidence.release_report_path.display()))?;
    anyhow::ensure!(
        release.schema_version == RELEASE_REPORT_SCHEMA_VERSION,
        "readiness release report schema must be {RELEASE_REPORT_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        release.release_version == RELEASE_VERSION,
        "readiness release report version must be {RELEASE_VERSION}"
    );
    anyhow::ensure!(
        release.target == expected.target,
        "readiness release target mismatch: expected {}, got {}",
        expected.target,
        release.target
    );
    anyhow::ensure!(
        release.git_commit == expected.commit,
        "readiness release commit mismatch: expected {}, got {}",
        expected.commit,
        release.git_commit
    );
    anyhow::ensure!(
        release.clean_worktree
            && release.expected_tag.as_deref() == Some(expected.tag)
            && release.tag_matches_commit
            && release.reproducible,
        "readiness release report must come from a clean, matching tag and be reproducible"
    );
    anyhow::ensure!(
        [
            release.audit.cargo_deny.as_str(),
            release.audit.cargo_audit.as_str(),
            release.audit.source_policy.as_str(),
            release.audit.license_policy.as_str(),
        ]
        .into_iter()
        .all(|status| status == "passed"),
        "readiness release report supply-chain verdicts must all pass"
    );
    anyhow::ensure!(
        release.installed_workflows.len() == INSTALL_CHECK_IDS.len()
            && INSTALL_CHECK_IDS.iter().all(|id| {
                release
                    .installed_workflows
                    .get(*id)
                    .is_some_and(|status| status == "passed")
            }),
        "readiness release report must retain all twelve passing installed workflows"
    );
    anyhow::ensure!(
        !release.members.is_empty(),
        "readiness release report must bind bundle members"
    );
    validate_release_members(&release.members)?;

    let install_bytes = read_bound_file(
        evidence.install_report_path,
        evidence.install_report_sha256,
        "archive-install report",
    )?;
    let archive_install: ArchiveInstallRehearsalReport = serde_json::from_slice(&install_bytes)
        .with_context(|| format!("parse {}", evidence.install_report_path.display()))?;
    anyhow::ensure!(
        archive_install.kind == ARCHIVE_INSTALL_REPORT_KIND
            && archive_install.schema_version == ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        "readiness archive-install report identity mismatch"
    );
    anyhow::ensure!(
        archive_install.bundle_root == bundle_name(expected.target),
        "readiness archive-install bundle root mismatch"
    );
    validate_archive_subject(
        &archive_install.archive,
        evidence.archive_path,
        evidence.archive_sha256,
        expected.target,
    )?;
    validate_member_identity(
        &archive_install.release_report,
        RELEASE_REPORT,
        &release_bytes,
    )?;
    let checksum_bytes = read_bound_file(
        evidence.checksum_manifest_path,
        evidence.checksum_manifest_sha256,
        "checksum manifest",
    )?;
    validate_member_identity(
        &archive_install.checksum_manifest,
        SHA256_MANIFEST,
        &checksum_bytes,
    )?;
    anyhow::ensure!(
        archive_install.time_to_proof.path == "flagship-proof/time-to-proof-report.json"
            && archive_install.time_to_proof.size_bytes > 0,
        "readiness archive-install report omitted time-to-proof evidence"
    );
    validate_sha256_hex(
        "readiness time-to-proof report",
        &archive_install.time_to_proof.sha256,
    )?;
    let install = &archive_install.rehearsal;
    anyhow::ensure!(
        install.schema_version == INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        "readiness install report schema must be {INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        install.release_version == RELEASE_VERSION && install.target == expected.target,
        "readiness install report version or target mismatch"
    );
    anyhow::ensure!(
        install.all_passed(),
        "readiness install report must pass all twelve canonical checks"
    );
    anyhow::ensure!(
        release.installed_workflows == install.verdicts(),
        "readiness release and independently extracted workflow verdicts differ"
    );
    validate_release_checksum_chain(&release, &release_bytes, &checksum_bytes, install)?;
    Ok(())
}

/// Builds and stages one native release bundle, including wheel and provenance evidence.
pub(crate) fn release_bundle(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_bundle_options(args)?;
    release_readiness::enforce_release_promotion(&root)?;
    validate_release_workflow_contract(&root)?;
    validate_release_target(&options.target)?;
    ensure_native_target(&root, &options.target)?;

    let wheel = absolute_from(&root, &options.wheel);
    anyhow::ensure!(wheel.is_file(), "wheel does not exist: {}", wheel.display());
    anyhow::ensure!(
        wheel.extension() == Some(OsStr::new("whl")),
        "wheel must use the .whl extension: {}",
        wheel.display()
    );

    let output_root = absolute_from(&root, &options.output_dir);
    fs::create_dir_all(&output_root)
        .with_context(|| format!("create release output {}", output_root.display()))?;
    let bundle_name = bundle_name(&options.target);
    let bundle_dir = output_root.join(&bundle_name);
    reset_generated_child(&output_root, &bundle_dir, &bundle_name)?;

    let clean_worktree = git_worktree_is_clean(&root)?;
    anyhow::ensure!(
        clean_worktree || options.allow_dirty,
        "release bundle requires a clean worktree (use --allow-dirty only for local development)"
    );
    let git_commit = git_output(&root, &["rev-parse", "HEAD"])?;
    let tag_matches_commit =
        validate_expected_tag(&root, options.expected_tag.as_deref(), &git_commit)?;

    let metadata = cargo_metadata(&root)?;
    validate_release_metadata(&metadata)?;
    let blockers: toml::Value =
        toml::from_str(&fs::read_to_string(root.join("release/blockers.toml"))?)?;
    validate_blocker_registry(&blockers)?;
    let contracts: toml::Value =
        toml::from_str(&fs::read_to_string(root.join("release/contracts.toml"))?)?;
    validate_contract_registry(&contracts)?;

    build_native_artifacts(&root, &options.target)?;
    stage_static_files(&root, &bundle_dir)?;
    stage_native_artifacts(&metadata, &bundle_dir, &options.target)?;
    stage_mujoco_runtime(&bundle_dir, &options.target)?;
    copy_file(
        &wheel,
        &bundle_dir
            .join("wheels")
            .join(wheel.file_name().context("wheel path has no file name")?),
    )?;

    let evidence_dir = release_evidence_dir(&metadata, &options.target)?;
    reset_generated_directory(&evidence_dir)?;
    let mut supply_args = vec![
        "--output-dir".to_string(),
        evidence_dir.to_string_lossy().into_owned(),
    ]
    .into_iter();
    supply_chain(&mut supply_args)?;
    let mut fuzz_args = vec![
        "--output-dir".to_string(),
        evidence_dir.to_string_lossy().into_owned(),
    ]
    .into_iter();
    fuzz_smoke(&mut fuzz_args)?;
    copy_file(
        &evidence_dir.join("sbom.cargo.json"),
        &bundle_dir.join("sbom.cargo.json"),
    )?;
    copy_file(
        &evidence_dir.join("cargo-lock.sha256"),
        &bundle_dir.join("evidence/cargo-lock.sha256"),
    )?;
    copy_file(
        &evidence_dir.join("report.json"),
        &bundle_dir.join("evidence/fuzz-smoke-report.json"),
    )?;

    let rehearsal_name = format!(".rehearsal-{}", options.target);
    let rehearsal_dir = output_root.join(&rehearsal_name);
    reset_generated_directory(&rehearsal_dir)?;
    let rehearsal = run_install_rehearsal(
        &bundle_dir,
        &rehearsal_dir,
        &options.python,
        &options.target,
        false,
    )?;
    write_pretty_json(&bundle_dir.join(INSTALL_REPORT), &rehearsal)?;
    anyhow::ensure!(
        rehearsal.all_passed(),
        "installed-bundle rehearsal failed; inspect {}",
        bundle_dir.join(INSTALL_REPORT).display()
    );

    let members = collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST])?;
    let fuzz: serde_json::Value =
        serde_json::from_slice(&fs::read(evidence_dir.join("report.json"))?)?;
    let fuzz_digest = fuzz["campaign_digest_sha256"]
        .as_str()
        .context("fuzz report omitted campaign_digest_sha256")?
        .to_string();
    let lock_bytes = fs::read(root.join("Cargo.lock"))?;
    let report = ReleaseReport {
        schema_version: RELEASE_REPORT_SCHEMA_VERSION,
        release_version: RELEASE_VERSION.to_string(),
        git_commit,
        target: options.target.clone(),
        rustc_version: program_version("rustc")?,
        cargo_version: program_version("cargo")?,
        cargo_lock_sha256: sha256_hex(&lock_bytes),
        clean_worktree,
        expected_tag: options.expected_tag.clone(),
        tag_matches_commit,
        reproducible: clean_worktree && options.expected_tag.is_some() && tag_matches_commit,
        audit: AuditVerdicts {
            cargo_deny: "passed".to_string(),
            cargo_audit: "passed".to_string(),
            source_policy: "passed".to_string(),
            license_policy: "passed".to_string(),
        },
        fuzz_campaign_digest_sha256: fuzz_digest,
        contracts: serde_json::to_value(contracts)?,
        installed_workflows: rehearsal.verdicts(),
        members,
    };
    write_pretty_json(&bundle_dir.join(RELEASE_REPORT), &report)?;
    write_sha256_manifest(&bundle_dir)?;
    verify_sha256_manifest(&bundle_dir)?;

    remove_generated_child(&output_root, &rehearsal_dir, &rehearsal_name)?;
    let evidence_parent = evidence_dir
        .parent()
        .context("release evidence directory has no parent")?;
    remove_generated_child(evidence_parent, &evidence_dir, &options.target)?;

    println!(
        "release bundle ready: target={} reproducible={} path={}",
        options.target,
        report.reproducible,
        bundle_dir.display()
    );
    Ok(())
}

/// Verifies an extracted bundle and reruns every installed-artifact smoke.
pub(crate) fn release_install_smoke(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_install_options(args)?;
    let archive = absolute_from(&root, &options.archive);
    let bundle_dir = absolute_from(&root, &options.bundle_dir);
    let output_dir = absolute_from(&root, &options.output_dir);
    anyhow::ensure!(
        bundle_dir.is_dir(),
        "bundle directory missing: {}",
        bundle_dir.display()
    );
    prepare_empty_directory(&output_dir)?;
    verify_sha256_manifest(&bundle_dir)?;

    let release: ReleaseReport =
        serde_json::from_slice(&fs::read(bundle_dir.join(RELEASE_REPORT))?)?;
    anyhow::ensure!(
        release.schema_version == RELEASE_REPORT_SCHEMA_VERSION
            && release.release_version == RELEASE_VERSION,
        "bundle release report is incompatible"
    );
    anyhow::ensure!(
        bundle_dir.file_name() == Some(OsStr::new(&bundle_name(&release.target))),
        "extracted bundle root does not match its release target"
    );
    let payload_members = collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST])?;
    anyhow::ensure!(
        release.members == payload_members,
        "bundle payload does not match release-report.json"
    );
    validate_release_target(&release.target)?;
    let report = run_install_rehearsal(
        &bundle_dir,
        &output_dir,
        &options.python,
        &release.target,
        true,
    )?;
    let archive_report = build_archive_install_report(
        &archive,
        &bundle_dir,
        &release,
        &report,
        &output_dir.join("flagship-proof/time-to-proof-report.json"),
    )?;
    write_pretty_json(&output_dir.join(ARCHIVE_INSTALL_REPORT), &archive_report)?;
    anyhow::ensure!(
        report.all_passed(),
        "installed-bundle rehearsal failed; inspect {}",
        output_dir.join(ARCHIVE_INSTALL_REPORT).display()
    );
    println!(
        "installed release bundle passed: target={} report={}",
        release.target,
        output_dir.join(ARCHIVE_INSTALL_REPORT).display()
    );
    Ok(())
}

/// Verifies a third-party installed flagship run against the exact release archive.
pub(crate) fn external_flagship_check(
    args: &mut impl Iterator<Item = String>,
) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_external_flagship_options(args)?;
    let archive = absolute_from(&root, &options.archive);
    let bundle_dir = absolute_from(&root, &options.bundle_dir);
    let proof_dir = absolute_from(&root, &options.proof_dir);
    let proof_bundle_path = absolute_from(&root, &options.proof_bundle);
    let submission_path = absolute_from(&root, &options.submission);
    let evidence_repo_dir = absolute_from(&root, &options.evidence_repo_dir);
    let output = absolute_from(&root, &options.output);
    let submission_bytes = read_external_regular_file(
        &submission_path,
        "external flagship submission candidate",
        MAX_EXTERNAL_SUBMISSION_BYTES,
    )?;
    let submission: ExternalFlagshipSubmissionCandidate = serde_json::from_slice(&submission_bytes)
        .context("parse completed external flagship submission candidate")?;
    validate_external_submission_candidate(&submission)?;
    validate_external_operator(
        &submission.evidence_repository.owner,
        &submission.evidence_repository.url,
        &options.revision,
        &submission.measurement.measured_on,
    )?;
    validate_external_repository_checkout(
        &evidence_repo_dir,
        &submission.evidence_repository.url,
        &options.revision,
    )?;
    let submission_relative_path = validate_committed_external_file(
        &evidence_repo_dir,
        &submission_path,
        "submission candidate",
    )?;
    anyhow::ensure!(
        !output.exists(),
        "refusing to replace external flagship report {}",
        output.display()
    );
    for (label, directory) in [("bundle", &bundle_dir), ("proof", &proof_dir)] {
        let metadata = fs::symlink_metadata(directory)
            .with_context(|| format!("inspect external flagship {label} directory"))?;
        anyhow::ensure!(
            metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
            "external flagship {label} must be a real non-symlink directory"
        );
    }
    verify_sha256_manifest(&bundle_dir)?;

    let release_bytes = fs::read(bundle_dir.join(RELEASE_REPORT))?;
    let release: ReleaseReport = serde_json::from_slice(&release_bytes)?;
    validate_release_target(&release.target)?;
    anyhow::ensure!(
        release.schema_version == RELEASE_REPORT_SCHEMA_VERSION
            && release.release_version == RELEASE_VERSION
            && release.clean_worktree
            && release.reproducible
            && release.tag_matches_commit
            && release.expected_tag.as_deref() == Some(&format!("v{RELEASE_VERSION}")),
        "external flagship evidence requires a clean tagged reproducible release bundle"
    );
    anyhow::ensure!(
        bundle_dir.file_name() == Some(OsStr::new(&bundle_name(&release.target))),
        "external flagship bundle root does not match its target"
    );
    let payload_members = collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST])?;
    anyhow::ensure!(
        release.members == payload_members,
        "external flagship bundle payload does not match release-report.json"
    );
    validate_mujoco_runtime(&bundle_dir, &release.target)?;

    let archive_metadata = fs::symlink_metadata(&archive)
        .with_context(|| format!("inspect external release archive {}", archive.display()))?;
    anyhow::ensure!(
        archive_metadata.file_type().is_file() && !archive_metadata.file_type().is_symlink(),
        "external release archive must be a regular non-symlink file"
    );
    let archive_file = archive
        .file_name()
        .and_then(OsStr::to_str)
        .context("external release archive name is not valid Unicode")?;
    let archive_sha256 = sha256_file_hex(&archive)?;
    let archive_subject = ArchiveSubject {
        file: archive_file.to_string(),
        size_bytes: archive_metadata.len(),
        sha256: format!("sha256:{archive_sha256}"),
    };
    validate_archive_subject(
        &archive_subject,
        &archive,
        &archive_subject.sha256,
        &release.target,
    )?;
    validate_submission_artifact(&submission.release_archive, &archive, "release archive")?;
    let proof_bundle = digest_external_file(&proof_bundle_path, "proof bundle")?;
    validate_submission_artifact(&submission.proof_bundle, &proof_bundle_path, "proof bundle")?;

    let stdout_path = resolve_submission_member(
        &evidence_repo_dir,
        &submission.reproduction.stdout_log_path,
        "stdout log",
    )?;
    let stderr_path = resolve_submission_member(
        &evidence_repo_dir,
        &submission.reproduction.stderr_log_path,
        "stderr log",
    )?;
    let mut stdout_log =
        digest_external_bounded_file(&stdout_path, "stdout log", MAX_EXTERNAL_LOG_BYTES)?;
    stdout_log.path = submission.reproduction.stdout_log_path.clone();
    let mut stderr_log =
        digest_external_bounded_file(&stderr_path, "stderr log", MAX_EXTERNAL_LOG_BYTES)?;
    stderr_log.path = submission.reproduction.stderr_log_path.clone();
    validate_committed_external_file(&evidence_repo_dir, &stdout_path, "stdout log")?;
    validate_committed_external_file(&evidence_repo_dir, &stderr_path, "stderr log")?;

    let producer = bundle_dir
        .join("bin")
        .join(native_binary_name("rne-flagship-proof", &release.target));
    let proof = validate_installed_flagship_proof(&proof_dir, &producer)?;
    let timing_path = proof_dir.join("time-to-proof-report.json");
    let timing: TimeToProofReport = serde_json::from_slice(&fs::read(&timing_path)?)?;
    validate_external_machine_label(&timing.machine_label)?;
    validate_time_to_proof_report(&proof_dir, &timing.machine_label)?;
    validate_timing_platform(&timing, &release.target)?;
    validate_submission_measurement(&submission.measurement, &timing, &release.target)?;
    rne_asset_cli::failure_capsule::verify_directory(&proof_dir.join("failure-capsule"))?;
    let (first_violation_step, first_violation_sim_time_ticks) =
        validate_external_cross_backend_report(&proof_dir.join("cross-backend-report.json"))?;

    let report = ExternalFlagshipReproductionReport {
        kind: "rne_external_flagship_reproduction_report".to_string(),
        schema_version: EXTERNAL_FLAGSHIP_REPRODUCTION_REPORT_SCHEMA_VERSION,
        status: "passed".to_string(),
        owner: submission.evidence_repository.owner,
        repository: submission.evidence_repository.url,
        revision: options.revision,
        measured_on: submission.measurement.measured_on,
        author_assistance: false,
        release_version: release.release_version,
        release_revision: release.git_commit,
        release_target: release.target,
        machine_label: timing.machine_label,
        operating_system: timing.operating_system,
        architecture: timing.architecture,
        elapsed_ms: timing.elapsed_ms,
        target_ms: timing.target_ms,
        task_id: proof.task_id,
        physics_execution_paths: proof.physics_execution_paths,
        first_violation_step,
        first_violation_sim_time_ticks,
        archive: MemberDigest {
            path: archive_file.to_string(),
            size_bytes: archive_metadata.len(),
            sha256: archive_sha256,
        },
        proof_bundle,
        submission_candidate: member_digest_from_bytes(
            &submission_relative_path,
            &submission_bytes,
        ),
        stdout_log,
        stderr_log,
        release_report: member_digest_from_bytes(RELEASE_REPORT, &release_bytes),
        checksum_manifest: digest_member(&bundle_dir, SHA256_MANIFEST)?,
        producer_executable: proof.producer_executable,
        installed_proof_report: digest_member(&proof_dir, "installed-proof-report.json")?,
        time_to_proof_report: digest_member(&proof_dir, "time-to-proof-report.json")?,
        cross_backend_report: digest_member(&proof_dir, "cross-backend-report.json")?,
        failure_capsule_manifest: digest_member(&proof_dir, "failure-capsule/capsule.json")?,
    };
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    write_pretty_json(&output, &report)?;
    println!(
        "external flagship reproduction verified: owner={} machine={} elapsed_ms={} report={}",
        report.owner,
        report.machine_label,
        report.elapsed_ms,
        output.display()
    );
    Ok(())
}

/// Revalidates the retained bytes that make an accepted installed flagship run
/// eligible for the 1.0 readiness gate.
pub(crate) fn validate_staged_external_flagship_report(
    report_bytes: &[u8],
    staged: StagedExternalFlagshipReproduction<'_>,
) -> anyhow::Result<()> {
    let report: ExternalFlagshipReproductionReport = serde_json::from_slice(report_bytes)
        .context("parse staged external flagship reproduction report")?;
    validate_external_operator(
        staged.owner,
        staged.repository,
        staged.revision,
        staged.measured_on,
    )?;
    anyhow::ensure!(
        report.kind == "rne_external_flagship_reproduction_report"
            && report.schema_version == EXTERNAL_FLAGSHIP_REPRODUCTION_REPORT_SCHEMA_VERSION
            && report.status == "passed"
            && report.owner == staged.owner
            && report.repository == staged.repository
            && report.revision == staged.revision
            && report.measured_on == staged.measured_on
            && !report.author_assistance
            && report.release_version == RELEASE_VERSION
            && is_lower_git_object_id(&report.release_revision)
            && report.elapsed_ms <= report.target_ms
            && report.target_ms == 15 * 60 * 1_000
            && report.task_id == "rne.flagship.mobile_lift_shared_aisle.v1"
            && report.physics_execution_paths == ["rapier_native", "mujoco_native"]
            && report.first_violation_step > 0
            && report.first_violation_sim_time_ticks > 0,
        "staged external flagship report identity or qualifying verdict drifted"
    );
    validate_release_target(&report.release_target)?;
    validate_external_machine_label(&report.machine_label)?;
    anyhow::ensure!(
        matches!(report.operating_system.as_str(), "windows" | "linux")
            && report.architecture == "x86_64",
        "staged external flagship report platform is not qualifying"
    );

    let candidate_bytes = read_external_regular_file(
        staged.submission_candidate,
        "staged submission candidate",
        MAX_EXTERNAL_SUBMISSION_BYTES,
    )?;
    let candidate: ExternalFlagshipSubmissionCandidate =
        serde_json::from_slice(&candidate_bytes)
            .context("parse staged external flagship submission candidate")?;
    validate_external_submission_candidate(&candidate)?;
    anyhow::ensure!(
        candidate.evidence_repository.owner == staged.owner
            && candidate.evidence_repository.url == staged.repository
            && candidate.measurement.measured_on == staged.measured_on
            && candidate.measurement.machine_label == report.machine_label
            && candidate.measurement.operating_system == report.operating_system
            && candidate.measurement.architecture == report.architecture
            && candidate.measurement.release_target == report.release_target
            && candidate.measurement.elapsed_ms == report.elapsed_ms
            && candidate.measurement.target_ms == report.target_ms,
        "staged external flagship candidate does not bind the accepted report"
    );
    validate_submission_artifact(
        &candidate.release_archive,
        staged.release_archive,
        "staged release archive",
    )?;
    validate_submission_artifact(
        &candidate.proof_bundle,
        staged.proof_bundle,
        "staged proof bundle",
    )?;
    anyhow::ensure!(
        candidate.reproduction.stdout_log_path == report.stdout_log.path
            && candidate.reproduction.stderr_log_path == report.stderr_log.path,
        "staged external flagship log paths differ from the candidate"
    );

    validate_report_member(
        &report.archive,
        staged.release_archive,
        None,
        "release archive",
    )?;
    validate_report_member(
        &report.proof_bundle,
        staged.proof_bundle,
        None,
        "proof bundle",
    )?;
    validate_relative_member(Path::new(&report.submission_candidate.path))?;
    validate_report_member(
        &report.submission_candidate,
        staged.submission_candidate,
        Some(&report.submission_candidate.path),
        "submission candidate",
    )?;
    validate_report_member(
        &report.stdout_log,
        staged.stdout_log,
        Some(&candidate.reproduction.stdout_log_path),
        "stdout log",
    )?;
    validate_report_member(
        &report.stderr_log,
        staged.stderr_log,
        Some(&candidate.reproduction.stderr_log_path),
        "stderr log",
    )?;
    Ok(())
}

fn validate_report_member(
    expected: &MemberDigest,
    path: &Path,
    expected_path: Option<&str>,
    label: &str,
) -> anyhow::Result<()> {
    let actual = digest_external_file(path, label)?;
    anyhow::ensure!(
        expected.path == expected_path.unwrap_or(&actual.path)
            && expected.size_bytes == actual.size_bytes
            && expected.sha256 == actual.sha256,
        "staged external flagship {label} differs from the accepted report"
    );
    Ok(())
}

fn member_digest_from_bytes(path: &str, bytes: &[u8]) -> MemberDigest {
    MemberDigest {
        path: path.to_string(),
        size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_hex(bytes),
    }
}

fn validate_external_submission_candidate(
    submission: &ExternalFlagshipSubmissionCandidate,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        submission.kind == EXTERNAL_FLAGSHIP_SUBMISSION_KIND
            && submission.schema_version == EXTERNAL_FLAGSHIP_SUBMISSION_SCHEMA_VERSION
            && submission.candidate_status == EXTERNAL_FLAGSHIP_CANDIDATE_STATUS
            && !submission.author_assistance,
        "external flagship submission identity or non-acceptance boundary drifted"
    );
    anyhow::ensure!(
        submission.required_proof_paths
            == EXTERNAL_FLAGSHIP_REQUIRED_PROOF_PATHS.map(str::to_string),
        "external flagship submission required proof paths drifted"
    );
    validate_submission_artifact_shape(&submission.release_archive, "release archive")?;
    validate_submission_artifact_shape(&submission.proof_bundle, "proof bundle")?;
    anyhow::ensure!(
        submission.reproduction.commands.len() >= 3
            && submission.reproduction.commands.len()
                == submission.reproduction.exit_statuses.len()
            && submission
                .reproduction
                .commands
                .iter()
                .all(|command| !command.trim().is_empty() && command.len() <= 4096)
            && submission
                .reproduction
                .exit_statuses
                .iter()
                .all(|status| *status == 0),
        "external flagship reproduction must retain at least three successful commands and matching zero exit statuses"
    );
    anyhow::ensure!(
        submission.reproduction.stdout_log_path != submission.reproduction.stderr_log_path,
        "external flagship stdout and stderr logs must be distinct files"
    );
    validate_relative_member(Path::new(&submission.reproduction.stdout_log_path))?;
    validate_relative_member(Path::new(&submission.reproduction.stderr_log_path))?;
    validate_external_machine_label(&submission.measurement.machine_label)?;
    validate_release_target(&submission.measurement.release_target)?;
    anyhow::ensure!(
        matches!(
            submission.measurement.operating_system.as_str(),
            "windows" | "linux"
        ) && submission.measurement.architecture == "x86_64"
            && submission.measurement.target_ms == 15 * 60 * 1_000
            && submission.measurement.elapsed_ms <= submission.measurement.target_ms,
        "external flagship submitted timing platform or 15-minute verdict is invalid"
    );
    Ok(())
}

fn validate_submission_artifact_shape(
    artifact: &SubmissionArtifact,
    label: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        artifact.url.starts_with("https://")
            && artifact.url.is_ascii()
            && artifact.url.len() <= 2048
            && !artifact.url.contains('#')
            && !artifact.url.contains('?')
            && artifact.file_name.len() <= 255
            && !artifact.file_name.is_empty()
            && Path::new(&artifact.file_name).file_name() == Some(OsStr::new(&artifact.file_name))
            && artifact.url.ends_with(&format!("/{}", artifact.file_name))
            && artifact.size_bytes > 0
            && is_lower_sha256(&artifact.sha256),
        "external flagship {label} identity is invalid"
    );
    Ok(())
}

fn is_lower_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_lower_git_object_id(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_submission_artifact(
    submitted: &SubmissionArtifact,
    path: &Path,
    label: &str,
) -> anyhow::Result<()> {
    let actual = digest_external_file(path, label)?;
    anyhow::ensure!(
        submitted.file_name == actual.path
            && submitted.size_bytes == actual.size_bytes
            && submitted.sha256 == actual.sha256,
        "external flagship {label} bytes differ from the completed submission candidate"
    );
    Ok(())
}

fn read_external_regular_file(path: &Path, label: &str, maximum: u64) -> anyhow::Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect external flagship {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file()
            && !metadata.file_type().is_symlink()
            && metadata.len() > 0
            && metadata.len() <= maximum,
        "external flagship {label} must be a non-empty regular non-symlink file no larger than {maximum} bytes"
    );
    fs::read(path).with_context(|| format!("read external flagship {label} {}", path.display()))
}

fn digest_external_bounded_file(
    path: &Path,
    label: &str,
    maximum: u64,
) -> anyhow::Result<MemberDigest> {
    let bytes = read_external_regular_file(path, label, maximum)?;
    let name = path
        .file_name()
        .and_then(OsStr::to_str)
        .with_context(|| format!("external flagship {label} name is not valid Unicode"))?;
    Ok(member_digest_from_bytes(name, &bytes))
}

fn digest_external_file(path: &Path, label: &str) -> anyhow::Result<MemberDigest> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect external flagship {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink() && metadata.len() > 0,
        "external flagship {label} must be a non-empty regular non-symlink file"
    );
    let name = path
        .file_name()
        .and_then(OsStr::to_str)
        .with_context(|| format!("external flagship {label} name is not valid Unicode"))?;
    Ok(MemberDigest {
        path: name.to_string(),
        size_bytes: metadata.len(),
        sha256: sha256_file_hex(path)?,
    })
}

fn resolve_submission_member(root: &Path, relative: &str, label: &str) -> anyhow::Result<PathBuf> {
    let relative_path = Path::new(relative);
    validate_relative_member(relative_path)?;
    let canonical_root = fs::canonicalize(root).with_context(|| {
        format!(
            "resolve external flagship submission root {}",
            root.display()
        )
    })?;
    let path = root.join(relative_path);
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("inspect external flagship {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "external flagship {label} must be a regular non-symlink file"
    );
    let canonical = fs::canonicalize(&path)
        .with_context(|| format!("resolve external flagship {label} {}", path.display()))?;
    anyhow::ensure!(
        canonical.starts_with(&canonical_root),
        "external flagship {label} escapes the submission repository"
    );
    Ok(canonical)
}

fn validate_external_repository_checkout(
    root: &Path,
    expected_url: &str,
    expected_revision: &str,
) -> anyhow::Result<()> {
    let metadata = fs::symlink_metadata(root)
        .with_context(|| format!("inspect external evidence repository {}", root.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
        "external evidence repository must be a real non-symlink directory"
    );
    anyhow::ensure!(
        git_output(root, &["rev-parse", "HEAD"])? == expected_revision,
        "external evidence repository HEAD differs from the submitted revision"
    );
    anyhow::ensure!(
        git_output(root, &["status", "--porcelain", "--untracked-files=all"])?.is_empty(),
        "external evidence repository must be clean including untracked files"
    );
    let origin = git_output(root, &["remote", "get-url", "origin"])?;
    let normalized_origin = origin.strip_suffix(".git").unwrap_or(&origin);
    let normalized_expected = expected_url.strip_suffix(".git").unwrap_or(expected_url);
    anyhow::ensure!(
        normalized_origin == normalized_expected,
        "external evidence repository origin differs from the completed submission candidate"
    );
    Ok(())
}

fn validate_committed_external_file(
    root: &Path,
    path: &Path,
    label: &str,
) -> anyhow::Result<String> {
    let canonical_root = fs::canonicalize(root)
        .with_context(|| format!("resolve external evidence repository {}", root.display()))?;
    let canonical_path = fs::canonicalize(path)
        .with_context(|| format!("resolve committed external {label} {}", path.display()))?;
    let relative = canonical_path
        .strip_prefix(&canonical_root)
        .with_context(|| format!("external {label} is outside the evidence repository"))?;
    validate_relative_member(relative)?;
    let git_relative = relative.to_string_lossy().replace('\\', "/");
    let object = format!("HEAD:{git_relative}");
    let output = Command::new("git")
        .current_dir(&canonical_root)
        .args(["show", "--no-textconv", &object])
        .output()
        .with_context(|| format!("read committed external {label} {object}"))?;
    anyhow::ensure!(
        output.status.success(),
        "external {label} is not committed at the submitted revision: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    let working_bytes = fs::read(&canonical_path)
        .with_context(|| format!("read external {label} {}", canonical_path.display()))?;
    anyhow::ensure!(
        output.stdout == working_bytes,
        "external {label} working bytes differ from the submitted revision"
    );
    Ok(git_relative)
}

fn validate_submission_measurement(
    submitted: &SubmissionMeasurement,
    timing: &TimeToProofReport,
    release_target: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        submitted.machine_label == timing.machine_label
            && submitted.operating_system == timing.operating_system
            && submitted.architecture == timing.architecture
            && submitted.release_target == release_target
            && submitted.elapsed_ms == timing.elapsed_ms
            && submitted.target_ms == timing.target_ms,
        "external flagship measurement differs from the proof timing report or release target"
    );
    Ok(())
}

fn validate_external_operator(
    owner: &str,
    repository: &str,
    revision: &str,
    measured_on: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        !owner.eq_ignore_ascii_case("rsasaki0109")
            && !owner.is_empty()
            && owner.len() <= 39
            && owner
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            && !owner.starts_with('-')
            && !owner.ends_with('-'),
        "external flagship owner must be an independent canonical GitHub owner"
    );
    let prefix = format!("https://github.com/{owner}/");
    anyhow::ensure!(
        repository.starts_with(&prefix)
            && repository.len() > prefix.len()
            && repository.len() <= 256
            && !repository[prefix.len()..].contains('/')
            && !repository.contains('?')
            && !repository.contains('#')
            && repository.is_ascii(),
        "external flagship repository must be one public GitHub repository owned by {owner}"
    );
    anyhow::ensure!(
        revision.len() == 40
            && revision
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "external flagship revision must be 40 lowercase hexadecimal characters"
    );
    validate_iso_date(measured_on)
}

fn validate_iso_date(value: &str) -> anyhow::Result<()> {
    let bytes = value.as_bytes();
    anyhow::ensure!(
        bytes.len() == 10
            && bytes[4] == b'-'
            && bytes[7] == b'-'
            && bytes
                .iter()
                .enumerate()
                .all(|(index, byte)| matches!(index, 4 | 7) || byte.is_ascii_digit()),
        "measurement date must use YYYY-MM-DD"
    );
    let year = value[0..4].parse::<u32>()?;
    let month = value[5..7].parse::<u32>()?;
    let day = value[8..10].parse::<u32>()?;
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let maximum_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => 0,
    };
    anyhow::ensure!(
        year >= 2026 && (1..=maximum_day).contains(&day),
        "invalid measurement date"
    );
    Ok(())
}

fn validate_external_machine_label(label: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !label.starts_with("github-hosted-release-rehearsal-")
            && !label.eq_ignore_ascii_case("test-machine")
            && !label.trim().is_empty(),
        "CI or placeholder machine labels cannot qualify as external reproduction"
    );
    Ok(())
}

fn validate_timing_platform(report: &TimeToProofReport, target: &str) -> anyhow::Result<()> {
    let expected_os = if target.contains("windows") {
        "windows"
    } else {
        "linux"
    };
    anyhow::ensure!(
        report.operating_system == expected_os
            && report.architecture == "x86_64"
            && report.elapsed_ms <= report.target_ms
            && report.target_ms == 15 * 60 * 1_000,
        "external timing platform or target does not match the release archive"
    );
    Ok(())
}

fn validate_external_cross_backend_report(path: &Path) -> anyhow::Result<(u64, u64)> {
    let report: serde_json::Value = serde_json::from_slice(&fs::read(path)?)?;
    anyhow::ensure!(
        report.get("kind").and_then(serde_json::Value::as_str)
            == Some(rne_asset_cli::FLAGSHIP_CROSS_BACKEND_REPORT_KIND)
            && report
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(u64::from(
                    rne_asset_cli::FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION,
                ))
            && report.get("status").and_then(serde_json::Value::as_str) == Some("passed")
            && report.get("task_id").and_then(serde_json::Value::as_str)
                == Some("rne.flagship.mobile_lift_shared_aisle.v1")
            && report
                .get("controller_id")
                .and_then(serde_json::Value::as_str)
                == Some("rne.ai.ik_mobile_lift_pick_place_policy.v1")
            && report
                .get("controller_contract")
                .and_then(serde_json::Value::as_str)
                == Some("identical_controller_type_and_configuration_per_backend"),
        "external cross-backend report identity or status mismatch"
    );
    let backends = report
        .get("backends")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted backends")?;
    anyhow::ensure!(
        backends.len() == 2
            && backends[0]
                .get("backend_id")
                .and_then(serde_json::Value::as_str)
                == Some("rapier_native")
            && backends[1]
                .get("backend_id")
                .and_then(serde_json::Value::as_str)
                == Some("mujoco_native")
            && backends.iter().all(|backend| {
                backend.get("status").and_then(serde_json::Value::as_str) == Some("passed")
            }),
        "external cross-backend report did not pass both production backends"
    );
    let exact_outcomes = report
        .get("exact_outcomes")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted exact outcomes")?;
    let expected_outcomes = [
        "all_behavior_contracts_passed",
        "inspection_completed",
        "traffic_cleared_without_collision_or_signal_violation",
        "payload_grasped_once",
        "pick_place_completed",
        "terminated_without_truncation_or_fail_closed_abort",
    ];
    anyhow::ensure!(
        exact_outcomes.len() == expected_outcomes.len()
            && exact_outcomes
                .iter()
                .filter_map(serde_json::Value::as_str)
                .eq(expected_outcomes),
        "external cross-backend exact success outcomes are incomplete"
    );
    let tolerances = report
        .get("tolerance_checks")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted tolerance checks")?;
    let expected_tolerances = [
        ("completion_step_delta", "step", 500.0),
        ("base_planar_position_delta", "m", 0.4),
        ("payload_position_delta", "m", 0.06),
        ("payload_apex_delta", "m", 0.07),
        ("arm_joint_position_delta", "rad", 0.2),
        ("lift_position_delta", "m", 0.04),
        ("gripper_position_delta", "m", 0.04),
        ("wrist_depth_delta", "m", 0.02),
        ("total_reward_delta", "reward", 0.75),
    ];
    anyhow::ensure!(
        tolerances.len() == expected_tolerances.len()
            && tolerances.iter().zip(expected_tolerances).all(
                |(check, (id, unit, maximum_delta))| {
                    check.get("id").and_then(serde_json::Value::as_str) == Some(id)
                        && check.get("unit").and_then(serde_json::Value::as_str) == Some(unit)
                        && check
                            .get("maximum_delta")
                            .and_then(serde_json::Value::as_f64)
                            == Some(maximum_delta)
                        && check.get("status").and_then(serde_json::Value::as_str) == Some("passed")
                        && check
                            .get("observed_delta")
                            .and_then(serde_json::Value::as_f64)
                            .is_some_and(|observed| observed.is_finite() && observed >= 0.0)
                },
            ),
        "external cross-backend SI tolerance checks are incomplete or failed"
    );
    let failure_outcomes = report
        .get("failure_exact_outcomes")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted exact failure outcomes")?;
    let expected_failure_outcomes = [
        "same_seed_and_minimized_fault_dimensions",
        "same_expected_contract",
        "same_first_violation_step",
        "same_first_violation_sim_time",
        "both_failure_replays_verified",
    ];
    anyhow::ensure!(
        failure_outcomes.len() == expected_failure_outcomes.len()
            && failure_outcomes
                .iter()
                .filter_map(serde_json::Value::as_str)
                .eq(expected_failure_outcomes),
        "external cross-backend exact failure outcomes are incomplete"
    );
    let failures = report
        .get("intentional_failures")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted intentional failures")?;
    anyhow::ensure!(
        failures.len() == 2
            && failures[0]
                .get("backend_id")
                .and_then(serde_json::Value::as_str)
                == Some("rapier_native")
            && failures[1]
                .get("backend_id")
                .and_then(serde_json::Value::as_str)
                == Some("mujoco_native")
            && failures.iter().all(|failure| {
                failure.get("status").and_then(serde_json::Value::as_str) == Some("passed")
                    && failure
                        .get("expected_contract")
                        .and_then(serde_json::Value::as_str)
                        == Some("perception_stream_alive")
                    && failure
                        .get("matched_replay_frames")
                        .and_then(serde_json::Value::as_u64)
                        .is_some_and(|frames| frames > 0)
            }),
        "external intentional failure evidence is incomplete"
    );
    let first_step = failures[0]
        .get("first_violation_step")
        .and_then(serde_json::Value::as_u64)
        .context("Rapier failure omitted first violation step")?;
    let first_time = failures[0]
        .get("first_violation_sim_time_ticks")
        .and_then(serde_json::Value::as_u64)
        .context("Rapier failure omitted first violation time")?;
    anyhow::ensure!(
        first_step > 0
            && first_time > 0
            && failures[1]
                .get("first_violation_step")
                .and_then(serde_json::Value::as_u64)
                == Some(first_step)
            && failures[1]
                .get("first_violation_sim_time_ticks")
                .and_then(serde_json::Value::as_u64)
                == Some(first_time),
        "external first violation differs between Rapier and MuJoCo"
    );
    let checks = report
        .get("failure_tolerance_checks")
        .and_then(serde_json::Value::as_array)
        .context("external cross-backend report omitted failure checks")?;
    anyhow::ensure!(
        checks.len() == 2
            && checks[0].get("id").and_then(serde_json::Value::as_str)
                == Some("first_violation_step_delta")
            && checks[0].get("unit").and_then(serde_json::Value::as_str) == Some("step")
            && checks[1].get("id").and_then(serde_json::Value::as_str)
                == Some("first_violation_time_delta")
            && checks[1].get("unit").and_then(serde_json::Value::as_str) == Some("ns")
            && checks.iter().all(|check| {
                check.get("status").and_then(serde_json::Value::as_str) == Some("passed")
                    && check
                        .get("observed_delta")
                        .and_then(serde_json::Value::as_f64)
                        == Some(0.0)
                    && check
                        .get("maximum_delta")
                        .and_then(serde_json::Value::as_f64)
                        == Some(0.0)
            }),
        "external first-violation checks are not exact"
    );
    Ok((first_step, first_time))
}

fn build_archive_install_report(
    archive_path: &Path,
    bundle_dir: &Path,
    release: &ReleaseReport,
    rehearsal: &InstallRehearsalReport,
    time_to_proof_path: &Path,
) -> anyhow::Result<ArchiveInstallRehearsalReport> {
    let archive_metadata = fs::symlink_metadata(archive_path)
        .with_context(|| format!("inspect release archive {}", archive_path.display()))?;
    anyhow::ensure!(
        archive_metadata.file_type().is_file(),
        "release-install-smoke requires a regular --archive file"
    );
    let archive_file = archive_path
        .file_name()
        .and_then(OsStr::to_str)
        .context("release archive file name is not valid Unicode")?
        .to_string();
    let archive_sha256 = format!("sha256:{}", sha256_file_hex(archive_path)?);
    let archive = ArchiveSubject {
        file: archive_file,
        size_bytes: archive_metadata.len(),
        sha256: archive_sha256.clone(),
    };
    validate_archive_subject(&archive, archive_path, &archive_sha256, &release.target)?;
    let release_bytes = fs::read(bundle_dir.join(RELEASE_REPORT))?;
    let checksum_bytes = fs::read(bundle_dir.join(SHA256_MANIFEST))?;
    validate_release_members(&release.members)?;
    validate_release_checksum_chain(release, &release_bytes, &checksum_bytes, rehearsal)?;
    anyhow::ensure!(
        release.installed_workflows == rehearsal.verdicts(),
        "independent rehearsal verdicts differ from the staged release report"
    );
    let release_report = MemberDigest {
        path: RELEASE_REPORT.to_string(),
        size_bytes: u64::try_from(release_bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_hex(&release_bytes),
    };
    let checksum_manifest = MemberDigest {
        path: SHA256_MANIFEST.to_string(),
        size_bytes: u64::try_from(checksum_bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_hex(&checksum_bytes),
    };
    let time_to_proof_bytes = fs::read(time_to_proof_path)
        .with_context(|| format!("read {}", time_to_proof_path.display()))?;
    let time_to_proof = MemberDigest {
        path: "flagship-proof/time-to-proof-report.json".to_string(),
        size_bytes: u64::try_from(time_to_proof_bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_hex(&time_to_proof_bytes),
    };
    Ok(ArchiveInstallRehearsalReport {
        kind: ARCHIVE_INSTALL_REPORT_KIND.to_string(),
        schema_version: ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        archive,
        bundle_root: bundle_name(&release.target),
        release_report,
        checksum_manifest,
        time_to_proof,
        rehearsal: rehearsal.clone(),
    })
}

fn parse_bundle_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<BundleOptions> {
    let mut target = None;
    let mut wheel = None;
    let mut output_dir = PathBuf::from("artifacts/release");
    let mut expected_tag = None;
    let mut python = default_python();
    let mut allow_dirty = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--target" => target = Some(required_arg(args, "--target")?),
            "--wheel" => wheel = Some(PathBuf::from(required_arg(args, "--wheel")?)),
            "--output-dir" => output_dir = PathBuf::from(required_arg(args, "--output-dir")?),
            "--expected-tag" => expected_tag = Some(required_arg(args, "--expected-tag")?),
            "--python" => python = PathBuf::from(required_arg(args, "--python")?),
            "--allow-dirty" => allow_dirty = true,
            other => bail!("unknown release-bundle argument: {other}"),
        }
    }
    Ok(BundleOptions {
        target: target.context("release-bundle requires --target TARGET")?,
        wheel: wheel.context("release-bundle requires --wheel PATH")?,
        output_dir,
        expected_tag,
        python,
        allow_dirty,
    })
}

fn parse_install_options(
    args: &mut impl Iterator<Item = String>,
) -> anyhow::Result<InstallOptions> {
    let mut archive = None;
    let mut bundle_dir = None;
    let mut output_dir = PathBuf::from("artifacts/release-install-smoke");
    let mut python = default_python();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--archive" => {
                archive = Some(PathBuf::from(required_arg(args, "--archive")?));
            }
            "--bundle-dir" => {
                bundle_dir = Some(PathBuf::from(required_arg(args, "--bundle-dir")?));
            }
            "--output-dir" => output_dir = PathBuf::from(required_arg(args, "--output-dir")?),
            "--python" => python = PathBuf::from(required_arg(args, "--python")?),
            other => bail!("unknown release-install-smoke argument: {other}"),
        }
    }
    Ok(InstallOptions {
        archive: archive.context("release-install-smoke requires --archive PATH")?,
        bundle_dir: bundle_dir.context("release-install-smoke requires --bundle-dir PATH")?,
        output_dir,
        python,
    })
}

fn parse_external_flagship_options(
    args: &mut impl Iterator<Item = String>,
) -> anyhow::Result<ExternalFlagshipOptions> {
    let mut archive = None;
    let mut bundle_dir = None;
    let mut proof_dir = None;
    let mut proof_bundle = None;
    let mut submission = None;
    let mut evidence_repo_dir = None;
    let mut revision = None;
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--archive" => archive = Some(PathBuf::from(required_arg(args, "--archive")?)),
            "--bundle-dir" => {
                bundle_dir = Some(PathBuf::from(required_arg(args, "--bundle-dir")?));
            }
            "--proof-dir" => {
                proof_dir = Some(PathBuf::from(required_arg(args, "--proof-dir")?));
            }
            "--proof-bundle" => {
                proof_bundle = Some(PathBuf::from(required_arg(args, "--proof-bundle")?));
            }
            "--submission" => {
                submission = Some(PathBuf::from(required_arg(args, "--submission")?));
            }
            "--evidence-repo-dir" => {
                evidence_repo_dir = Some(PathBuf::from(required_arg(args, "--evidence-repo-dir")?));
            }
            "--revision" => revision = Some(required_arg(args, "--revision")?),
            "--output" => output = Some(PathBuf::from(required_arg(args, "--output")?)),
            other => bail!("unknown external-flagship-check argument: {other}"),
        }
    }
    Ok(ExternalFlagshipOptions {
        archive: archive.context("external-flagship-check requires --archive PATH")?,
        bundle_dir: bundle_dir.context("external-flagship-check requires --bundle-dir PATH")?,
        proof_dir: proof_dir.context("external-flagship-check requires --proof-dir PATH")?,
        proof_bundle: proof_bundle
            .context("external-flagship-check requires --proof-bundle PATH")?,
        submission: submission.context("external-flagship-check requires --submission PATH")?,
        evidence_repo_dir: evidence_repo_dir
            .context("external-flagship-check requires --evidence-repo-dir PATH")?,
        revision: revision.context("external-flagship-check requires --revision SHA")?,
        output: output.context("external-flagship-check requires --output PATH")?,
    })
}

fn required_arg(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn default_python() -> PathBuf {
    PathBuf::from(if cfg!(windows) { "python" } else { "python3" })
}

fn validate_release_target(target: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !target.is_empty()
            && target.len() <= 96
            && target
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.')),
        "invalid release target {target:?}"
    );
    anyhow::ensure!(
        target.contains("windows") || target.contains("linux"),
        "M6 release bundles support Linux and Windows targets only"
    );
    Ok(())
}

fn bundle_name(target: &str) -> String {
    format!("rne-{RELEASE_VERSION}-{target}")
}

fn ensure_native_target(root: &Path, target: &str) -> anyhow::Result<()> {
    let output = command_output(root, Path::new("rustc"), &[OsString::from("-vV")], &[])?;
    ensure_success("rustc -vV", &output)?;
    let text = String::from_utf8_lossy(&output.stdout);
    let host = text
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .context("rustc -vV omitted host target")?;
    anyhow::ensure!(
        host == target,
        "release rehearsal must be native so the wheel and bundle match: host={host} target={target}"
    );
    Ok(())
}

fn build_native_artifacts(root: &Path, target: &str) -> anyhow::Result<()> {
    let mut args = vec![
        OsString::from("build"),
        OsString::from("--locked"),
        OsString::from("--release"),
    ];
    for package in RELEASE_BINARY_PACKAGES
        .iter()
        .map(|(package, _)| *package)
        .filter(|package| *package != "flagship_validation_workflow")
        .collect::<BTreeSet<_>>()
    {
        args.push(OsString::from("-p"));
        args.push(OsString::from(package));
    }
    args.push(OsString::from("-p"));
    args.push(OsString::from(RELEASE_PLUGIN_PACKAGE));
    let output = command_output(root, Path::new("cargo"), &args, &[])?;
    print_output(&output);
    ensure_success("cargo build release artifacts", &output)?;

    let mut flagship_args = vec![
        OsString::from("rustc"),
        OsString::from("--locked"),
        OsString::from("--release"),
        OsString::from("-p"),
        OsString::from("flagship_validation_workflow"),
        OsString::from("--features"),
        OsString::from("mujoco"),
        OsString::from("--bin"),
        OsString::from("rne-flagship-proof"),
    ];
    if target.contains("linux") {
        flagship_args.extend([
            OsString::from("--"),
            OsString::from("-C"),
            OsString::from("link-arg=-Wl,-rpath,$ORIGIN/../lib"),
        ]);
    }
    let output = command_output(root, Path::new("cargo"), &flagship_args, &[])?;
    print_output(&output);
    ensure_success("cargo build cross-backend flagship proof", &output)
}

fn stage_static_files(root: &Path, bundle_dir: &Path) -> anyhow::Result<()> {
    for (source, destination) in BUNDLE_FILES.into_iter().chain(SCENARIO_FILES) {
        copy_file(&root.join(source), &bundle_dir.join(destination))?;
    }
    Ok(())
}

fn stage_native_artifacts(
    metadata: &serde_json::Value,
    bundle_dir: &Path,
    target: &str,
) -> anyhow::Result<()> {
    let target_dir = PathBuf::from(
        metadata["target_directory"]
            .as_str()
            .context("cargo metadata omitted target_directory")?,
    );
    let release_dir = target_dir.join("release");
    for (_, binary) in RELEASE_BINARY_PACKAGES {
        let file = native_binary_name(binary, target);
        copy_file(&release_dir.join(&file), &bundle_dir.join("bin").join(file))?;
    }
    let plugin = native_plugin_name(target);
    copy_file(
        &release_dir.join(&plugin),
        &bundle_dir.join("lib").join(plugin),
    )?;
    let root = workspace_root()?;
    copy_file(
        &root.join("crates/rne_plugin_example_velocity_servo/rne-plugin.json"),
        &bundle_dir.join("lib/rne-plugin.json"),
    )?;
    Ok(())
}

fn mujoco_runtime_root() -> anyhow::Result<PathBuf> {
    if let Some(root) = env::var_os(MUJOCO_RUNTIME_ROOT_ENV) {
        let root = PathBuf::from(root);
        anyhow::ensure!(
            root.is_dir(),
            "{MUJOCO_RUNTIME_ROOT_ENV} is not a directory: {}",
            root.display()
        );
        return Ok(root);
    }
    let link_dir = env::var_os(MUJOCO_DYNAMIC_LINK_DIR_ENV)
        .map(PathBuf::from)
        .context("release bundle requires MUJOCO_RUNTIME_ROOT or MUJOCO_DYNAMIC_LINK_DIR")?;
    let root = link_dir
        .parent()
        .context("MUJOCO_DYNAMIC_LINK_DIR has no runtime root")?;
    anyhow::ensure!(
        root.is_dir(),
        "inferred MuJoCo runtime root is not a directory: {}",
        root.display()
    );
    Ok(root.to_path_buf())
}

fn digest_member(root: &Path, relative: &str) -> anyhow::Result<MemberDigest> {
    let path = root.join(relative);
    let bytes = fs::read(&path).with_context(|| format!("read {}", path.display()))?;
    Ok(MemberDigest {
        path: relative.to_string(),
        size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_hex(&bytes),
    })
}

fn stage_mujoco_runtime(bundle_dir: &Path, target: &str) -> anyhow::Result<()> {
    let runtime_root = mujoco_runtime_root()?;
    let (archive_file, archive_sha256, runtime_sources) = mujoco_archive_contract(target);
    let archive_path = env::var_os(MUJOCO_ARCHIVE_PATH_ENV)
        .map(PathBuf::from)
        .context("release bundle requires MUJOCO_ARCHIVE_PATH for provenance verification")?;
    anyhow::ensure!(
        archive_path.file_name() == Some(OsStr::new(archive_file)),
        "MuJoCo archive must be named {archive_file}: {}",
        archive_path.display()
    );
    anyhow::ensure!(
        archive_path.is_file() && sha256_file_hex(&archive_path)? == archive_sha256,
        "MuJoCo archive failed the pinned SHA-256 check: {}",
        archive_path.display()
    );
    for (source, destination, expected_sha256) in runtime_sources {
        anyhow::ensure!(
            sha256_file_hex(&runtime_root.join(source))? == *expected_sha256,
            "MuJoCo runtime member does not match the pinned archive: {source}"
        );
        copy_file(&runtime_root.join(source), &bundle_dir.join(destination))?;
    }
    let license_sources = mujoco_license_contract(target);
    for (source, destination, expected_sha256) in license_sources {
        anyhow::ensure!(
            sha256_file_hex(&runtime_root.join(source))? == *expected_sha256,
            "MuJoCo license member does not match the pinned archive: {source}"
        );
        copy_file(&runtime_root.join(source), &bundle_dir.join(destination))?;
    }

    let runtime_members = runtime_sources
        .iter()
        .map(|(_, destination, _)| digest_member(bundle_dir, destination))
        .collect::<anyhow::Result<Vec<_>>>()?;
    let license_members = license_sources
        .iter()
        .map(|(_, destination, _)| digest_member(bundle_dir, destination))
        .collect::<anyhow::Result<Vec<_>>>()?;
    let manifest = MujocoRuntimeManifest {
        kind: "rne_mujoco_runtime".to_string(),
        schema_version: MUJOCO_RUNTIME_MANIFEST_SCHEMA_VERSION,
        version: MUJOCO_VERSION.to_string(),
        source_url: format!(
            "https://github.com/google-deepmind/mujoco/releases/download/{MUJOCO_VERSION}/{archive_file}"
        ),
        archive_file: archive_file.to_string(),
        archive_sha256: archive_sha256.to_string(),
        runtime_members,
        license_members,
    };
    write_pretty_json(
        &bundle_dir.join("third-party/mujoco/runtime-manifest.json"),
        &manifest,
    )
}

fn mujoco_archive_contract(
    target: &str,
) -> (
    &'static str,
    &'static str,
    &'static [(&'static str, &'static str, &'static str)],
) {
    if target.contains("windows") {
        (
            MUJOCO_WINDOWS_ARCHIVE,
            MUJOCO_WINDOWS_ARCHIVE_SHA256,
            &[(
                "bin/mujoco.dll",
                "bin/mujoco.dll",
                MUJOCO_WINDOWS_RUNTIME_SHA256,
            )],
        )
    } else {
        (
            MUJOCO_LINUX_ARCHIVE,
            MUJOCO_LINUX_ARCHIVE_SHA256,
            &[
                (
                    "lib/libmujoco.so",
                    "lib/libmujoco.so",
                    MUJOCO_LINUX_RUNTIME_SHA256,
                ),
                (
                    "lib/libmujoco.so",
                    "lib/libmujoco.so.3.9.0",
                    MUJOCO_LINUX_RUNTIME_SHA256,
                ),
            ],
        )
    }
}

fn mujoco_license_contract(target: &str) -> &'static [(&'static str, &'static str, &'static str)] {
    if target.contains("windows") {
        &[
            (
                "LICENSE",
                "third-party/mujoco/LICENSE",
                MUJOCO_WINDOWS_LICENSE_SHA256,
            ),
            (
                "THIRD_PARTY_NOTICES.txt",
                "third-party/mujoco/THIRD_PARTY_NOTICES.txt",
                MUJOCO_WINDOWS_NOTICES_SHA256,
            ),
        ]
    } else {
        &[
            (
                "LICENSE",
                "third-party/mujoco/LICENSE",
                MUJOCO_LINUX_LICENSE_SHA256,
            ),
            (
                "THIRD_PARTY_NOTICES",
                "third-party/mujoco/THIRD_PARTY_NOTICES.txt",
                MUJOCO_LINUX_NOTICES_SHA256,
            ),
        ]
    }
}

fn validate_mujoco_runtime(bundle_dir: &Path, target: &str) -> anyhow::Result<()> {
    let manifest_path = bundle_dir.join("third-party/mujoco/runtime-manifest.json");
    let manifest: MujocoRuntimeManifest = serde_json::from_slice(
        &fs::read(&manifest_path).with_context(|| format!("read {}", manifest_path.display()))?,
    )
    .with_context(|| format!("parse {}", manifest_path.display()))?;
    let (archive_file, archive_sha256, runtime_sources) = mujoco_archive_contract(target);
    anyhow::ensure!(
        manifest.kind == "rne_mujoco_runtime"
            && manifest.schema_version == MUJOCO_RUNTIME_MANIFEST_SCHEMA_VERSION
            && manifest.version == MUJOCO_VERSION
            && manifest.archive_file == archive_file
            && manifest.archive_sha256 == archive_sha256
            && manifest.source_url
                == format!(
                    "https://github.com/google-deepmind/mujoco/releases/download/{MUJOCO_VERSION}/{archive_file}"
                ),
        "bundled MuJoCo runtime provenance is not the pinned target contract"
    );
    let expected_runtime_paths = runtime_sources
        .iter()
        .map(|(_, destination, _)| *destination)
        .collect::<Vec<_>>();
    anyhow::ensure!(
        manifest
            .runtime_members
            .iter()
            .map(|member| member.path.as_str())
            .eq(expected_runtime_paths),
        "bundled MuJoCo runtime member set is incomplete or unordered"
    );
    let expected_license_paths = mujoco_license_contract(target)
        .iter()
        .map(|(_, destination, _)| *destination);
    anyhow::ensure!(
        manifest
            .license_members
            .iter()
            .map(|member| member.path.as_str())
            .eq(expected_license_paths),
        "bundled MuJoCo license member set is incomplete or unordered"
    );
    for member in manifest
        .runtime_members
        .iter()
        .chain(&manifest.license_members)
    {
        validate_proof_member(bundle_dir, member, &member.path)?;
    }
    Ok(())
}

fn native_binary_name(name: &str, target: &str) -> String {
    if target.contains("windows") {
        format!("{name}.exe")
    } else {
        name.to_string()
    }
}

fn native_plugin_name(target: &str) -> String {
    native_cdylib_name(RELEASE_PLUGIN_PACKAGE, target)
}

fn native_cdylib_name(name: &str, target: &str) -> String {
    if target.contains("windows") {
        format!("{name}.dll")
    } else if target.contains("darwin") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

fn release_evidence_dir(metadata: &serde_json::Value, target: &str) -> anyhow::Result<PathBuf> {
    let target_dir = PathBuf::from(
        metadata["target_directory"]
            .as_str()
            .context("cargo metadata omitted target_directory")?,
    );
    Ok(target_dir.join("release-evidence").join(target))
}

fn run_install_rehearsal(
    bundle_dir: &Path,
    output_dir: &Path,
    python: &Path,
    target: &str,
    verify_checksums: bool,
) -> anyhow::Result<InstallRehearsalReport> {
    if verify_checksums {
        verify_sha256_manifest(bundle_dir)?;
    }
    validate_mujoco_runtime(bundle_dir, target)?;
    fs::create_dir_all(output_dir)?;
    let bin_dir = bundle_dir.join("bin");
    let asset_cli = bin_dir.join(native_binary_name("rne-asset", target));
    let compatibility = bin_dir.join(native_binary_name("rne-compatibility", target));
    let physics = bin_dir.join(native_binary_name("rne-physics-conformance", target));
    let scale = bin_dir.join(native_binary_name("rne-scenario-scale", target));
    let hardware_conformance = bin_dir.join(native_binary_name("rne-hardware-conformance", target));
    let hardware_mock = bin_dir.join(native_binary_name("rne-hardware-mock-device", target));
    let simulator_conformance =
        bin_dir.join(native_binary_name("rne-simulator-conformance", target));
    let simulator_mock = bin_dir.join(native_binary_name("rne-simulator-mock-adapter", target));
    let accelerator_conformance =
        bin_dir.join(native_binary_name("rne-accelerator-conformance", target));
    let accelerator_mock =
        bin_dir.join(native_binary_name("rne-accelerator-protocol-mock", target));
    let flagship_proof = bin_dir.join(native_binary_name("rne-flagship-proof", target));

    let robot_replay = output_dir.join("robot.rne-replay");
    let robot_run = run_check_command(
        "robot replay generation",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("run"),
            bundle_dir
                .join("assets/runs/mesh_diff_drive.rne.run.toml")
                .into_os_string(),
            OsString::from("--replay-out"),
            robot_replay.clone().into_os_string(),
        ],
        &[],
    );
    let robot_verify = robot_run
        && run_check_command(
            "robot replay verification",
            bundle_dir,
            &asset_cli,
            &[
                OsString::from("replay"),
                robot_replay.clone().into_os_string(),
            ],
            &[],
        );
    let capsule_dir = output_dir.join("failure-capsule");
    let capsule_create = robot_verify
        && run_check_command(
            "installed Failure Capsule creation",
            bundle_dir,
            &asset_cli,
            &[
                OsString::from("failure-capsule"),
                OsString::from("create"),
                OsString::from("--replay"),
                bundle_dir
                    .join("tests/golden/replays/behavior-replay-v1.json")
                    .into_os_string(),
                OsString::from("--evidence"),
                bundle_dir
                    .join("assets/tasks/diff_drive_goal.task.json")
                    .into_os_string(),
                OsString::from("--output"),
                capsule_dir.clone().into_os_string(),
                OsString::from("--backend"),
                OsString::from("installed-reference"),
                OsString::from("--backend-version"),
                OsString::from(RELEASE_VERSION),
            ],
            &[],
        );
    let capsule_verify = capsule_create
        && run_check_command(
            "installed Failure Capsule verification",
            bundle_dir,
            &asset_cli,
            &[
                OsString::from("failure-capsule"),
                OsString::from("verify"),
                capsule_dir.into_os_string(),
            ],
            &[],
        );

    let flagship_output = output_dir.join("flagship-proof");
    let flagship_machine_label = format!("github-hosted-release-rehearsal-{target}");
    let flagship_passed = run_check_command(
        "installed flagship proof",
        bundle_dir,
        &flagship_proof,
        &[
            flagship_output.clone().into_os_string(),
            OsString::from("--cross-backend"),
            OsString::from("--measure-on"),
            OsString::from(&flagship_machine_label),
            OsString::from("--verify-installed-bundle"),
            bundle_dir.to_path_buf().into_os_string(),
        ],
        &[],
    ) && validate_installed_flagship_proof(&flagship_output, &flagship_proof)
        .map_err(|error| {
            eprintln!("installed flagship proof validation failed: {error:#}");
            error
        })
        .is_ok()
        && validate_time_to_proof_report(&flagship_output, &flagship_machine_label)
            .map_err(|error| {
                eprintln!("time-to-proof report validation failed: {error:#}");
                error
            })
            .is_ok();

    let scenario_replay = output_dir.join("scenario.rne-replay");
    let scenario_run = run_check_command(
        "scenario replay generation",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("run"),
            bundle_dir
                .join("assets/runs/scenario_speed.rne.run.toml")
                .into_os_string(),
            OsString::from("--replay-out"),
            scenario_replay.clone().into_os_string(),
        ],
        &[],
    );
    let scenario_verify = scenario_run
        && run_check_command(
            "scenario replay verification",
            bundle_dir,
            &asset_cli,
            &[OsString::from("replay"), scenario_replay.into_os_string()],
            &[],
        );

    let physics_report = output_dir.join("physics-conformance.json");
    let physics_passed = run_check_command(
        "physics conformance",
        bundle_dir,
        &physics,
        &[
            OsString::from("--output"),
            physics_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &physics_report,
        "all_passed",
        &serde_json::Value::Bool(true),
    );

    let scale_report = output_dir.join("scenario-scale.json");
    let scale_passed = run_check_command(
        "100-actor scenario scale",
        bundle_dir,
        &scale,
        &[
            OsString::from("--output"),
            scale_report.clone().into_os_string(),
        ],
        &[(
            OsString::from("RNE_SCENARIO_SCALE_BENCHMARK_CLASS"),
            OsString::from(format!("release-rehearsal-{target}")),
        )],
    ) && json_field_matches(
        &scale_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );

    let hardware_report = output_dir.join("hardware-adapter-conformance.json");
    let hardware_passed = run_check_command(
        "external hardware adapter conformance",
        bundle_dir,
        &hardware_conformance,
        &[
            OsString::from("--adapter"),
            hardware_mock.clone().into_os_string(),
            OsString::from("--adapter-arg"),
            OsString::from("--device-id"),
            OsString::from("--adapter-arg"),
            OsString::from("rne-release-hardware-mock-v1"),
            OsString::from("--adapter-arg"),
            OsString::from("--expected-task-id"),
            OsString::from("--adapter-arg"),
            OsString::from("rne.diff_drive.sensor_goal.v1"),
            OsString::from("--adapter-arg"),
            OsString::from("--observation-width"),
            OsString::from("--adapter-arg"),
            OsString::from("9"),
            OsString::from("--adapter-arg"),
            OsString::from("--action-width"),
            OsString::from("--adapter-arg"),
            OsString::from("2"),
            OsString::from("--task"),
            bundle_dir
                .join("assets/tasks/diff_drive_goal.task.json")
                .into_os_string(),
            OsString::from("--allow-hil"),
            OsString::from("--output"),
            hardware_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &hardware_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );

    let simulator_report = output_dir.join("simulator-adapter-conformance.json");
    let simulator_passed = run_check_command(
        "external simulator adapter conformance",
        bundle_dir,
        &simulator_conformance,
        &[
            OsString::from("--adapter"),
            simulator_mock.into_os_string(),
            OsString::from("--adapter-arg"),
            OsString::from("--simulator-id"),
            OsString::from("--adapter-arg"),
            OsString::from("gazebo_sim_fixture"),
            OsString::from("--adapter-arg"),
            OsString::from("--simulator-version"),
            OsString::from("--adapter-arg"),
            OsString::from("8.9.0"),
            OsString::from("--adapter-arg"),
            OsString::from("--task-id"),
            OsString::from("--adapter-arg"),
            OsString::from("rne.diff_drive.sensor_goal.v1"),
            OsString::from("--adapter-arg"),
            OsString::from("--task-sha256"),
            OsString::from("--adapter-arg"),
            OsString::from("532d2e76854cecbc09e5f8d985486c2f9548a3f39a17865a59f10d86dd08e3ca"),
            OsString::from("--adapter-arg"),
            OsString::from("--observation-width"),
            OsString::from("--adapter-arg"),
            OsString::from("9"),
            OsString::from("--adapter-arg"),
            OsString::from("--action-width"),
            OsString::from("--adapter-arg"),
            OsString::from("2"),
            OsString::from("--adapter-arg"),
            OsString::from("--fixed-delta-ticks"),
            OsString::from("--adapter-arg"),
            OsString::from("16666667"),
            OsString::from("--runtime-manifest"),
            bundle_dir
                .join("adapters/simulator/reference/runtime.json")
                .into_os_string(),
            OsString::from("--task"),
            bundle_dir
                .join("assets/tasks/diff_drive_goal.task.json")
                .into_os_string(),
            OsString::from("--output"),
            simulator_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &simulator_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );

    let accelerator_report = output_dir.join("accelerator-protocol-conformance.json");
    let reference_accelerator_passed = run_check_command(
        "external accelerator protocol conformance",
        bundle_dir,
        &accelerator_conformance,
        &[
            OsString::from("--adapter"),
            accelerator_mock.clone().into_os_string(),
            OsString::from("--adapter-arg"),
            OsString::from("--transcript"),
            OsString::from("--adapter-arg"),
            bundle_dir
                .join("tests/golden/accelerators/protocol-transcript-v1.json")
                .into_os_string(),
            OsString::from("--subject"),
            accelerator_mock.into_os_string(),
            OsString::from("--manifest"),
            bundle_dir
                .join("adapters/mjx/accelerator.toml")
                .into_os_string(),
            OsString::from("--runtime"),
            bundle_dir
                .join("adapters/mjx/runtime.toml")
                .into_os_string(),
            OsString::from("--task"),
            bundle_dir
                .join("adapters/mjx/fixtures/free-fall-task-spec-v1.json")
                .into_os_string(),
            OsString::from("--output"),
            accelerator_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &accelerator_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );
    let scaffold_accelerator_passed = run_accelerator_scaffold_rehearsal(
        bundle_dir,
        output_dir,
        python,
        &accelerator_conformance,
    );
    let accelerator_passed = reference_accelerator_passed && scaffold_accelerator_passed;

    let plugin_report = output_dir.join("controller-plugin-conformance.json");
    let reference_plugin_passed = run_check_command(
        "controller plugin conformance",
        bundle_dir,
        &asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("check"),
            OsString::from("--library"),
            bundle_dir
                .join("lib")
                .join(native_plugin_name(target))
                .into_os_string(),
            OsString::from("--manifest"),
            bundle_dir.join("lib/rne-plugin.json").into_os_string(),
            OsString::from("--output"),
            plugin_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &plugin_report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    );
    let scaffold_plugin_passed = run_scaffold_rehearsal(bundle_dir, output_dir, &asset_cli, target);
    let plugin_passed = reference_plugin_passed && scaffold_plugin_passed;

    let compatibility_report = output_dir.join("compatibility-fixture-report.json");
    let compatibility_passed = run_check_command(
        "installed compatibility corpus",
        bundle_dir,
        &compatibility,
        &[
            OsString::from("--root"),
            bundle_dir.to_path_buf().into_os_string(),
            OsString::from("--output"),
            compatibility_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &compatibility_report,
        "passed",
        &serde_json::Value::Bool(true),
    );

    let (wheel_passed, python_api_passed) =
        run_python_wheel_smoke(bundle_dir, output_dir, python, target);
    let checks = vec![
        check("robot_replay", robot_verify && capsule_verify),
        check("flagship_proof", flagship_passed),
        check("scenario_replay", scenario_verify),
        check("physics_conformance", physics_passed),
        check("scenario_scale_100", scale_passed),
        check("hardware_adapter", hardware_passed),
        check("simulator_adapter", simulator_passed),
        check("accelerator_protocol", accelerator_passed),
        check("controller_plugin", plugin_passed),
        check("compatibility_corpus", compatibility_passed),
        check("python_wheel", wheel_passed),
        check("python_api", python_api_passed),
    ];
    let passed = checks.iter().all(|check| check.status == "passed");
    let report = InstallRehearsalReport {
        schema_version: INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        release_version: RELEASE_VERSION.to_string(),
        target: target.to_string(),
        status: if passed { "passed" } else { "failed" }.to_string(),
        checks,
    };
    if report.all_passed() {
        cleanup_install_rehearsal_transients(output_dir)?;
    }
    Ok(report)
}

fn run_scaffold_rehearsal(
    bundle_dir: &Path,
    output_dir: &Path,
    asset_cli: &Path,
    target: &str,
) -> bool {
    const NAME: &str = "release_scaffold_controller";
    let parent = output_dir.join("controller-authoring");
    if !run_check_command(
        "scaffold controller plugin",
        bundle_dir,
        asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("new"),
            OsString::from(NAME),
            OsString::from("--dir"),
            parent.clone().into_os_string(),
            OsString::from("--schema"),
            OsString::from("1"),
        ],
        &[],
    ) {
        return false;
    }
    let crate_dir = parent.join(NAME);
    let contract_path = crate_dir.join("rne-scaffold.json");
    let contract = match fs::read(&contract_path)
        .map_err(anyhow::Error::from)
        .and_then(|bytes| {
            ControllerPluginScaffoldContract::from_json_slice(&bytes).map_err(Into::into)
        }) {
        Ok(contract) => contract,
        Err(error) => {
            eprintln!("could not validate controller scaffold contract: {error:#}");
            return false;
        }
    };
    if let Err(error) = contract.validate_directory(&crate_dir) {
        eprintln!("controller scaffold directory differs from its contract: {error}");
        return false;
    }
    let bundled_sdk = bundle_dir.join("sdk/rust/rne_plugin_sdk.rs");
    let scaffold_sdk = crate_dir.join("src/rne_plugin_sdk.rs");
    match (fs::read(&bundled_sdk), fs::read(&scaffold_sdk)) {
        (Ok(bundled), Ok(scaffolded)) if bundled == scaffolded => {}
        (Ok(_), Ok(_)) => {
            eprintln!("scaffold SDK differs from bundled SDK source");
            return false;
        }
        (Err(error), _) => {
            eprintln!(
                "could not read bundled SDK {}: {error}",
                bundled_sdk.display()
            );
            return false;
        }
        (_, Err(error)) => {
            eprintln!(
                "could not read scaffold SDK {}: {error}",
                scaffold_sdk.display()
            );
            return false;
        }
    }
    let scaffold_target = parent.join("target");
    if !run_check_command(
        "build scaffolded controller offline",
        &crate_dir,
        Path::new("cargo"),
        &[
            OsString::from("build"),
            OsString::from("--offline"),
            OsString::from("--manifest-path"),
            crate_dir.join("Cargo.toml").into_os_string(),
            OsString::from("--target-dir"),
            scaffold_target.clone().into_os_string(),
        ],
        &[(OsString::from("RUSTFLAGS"), OsString::from("-Dwarnings"))],
    ) {
        return false;
    }
    let report = output_dir.join("controller-scaffold-conformance.json");
    run_check_command(
        "scaffolded controller conformance",
        bundle_dir,
        asset_cli,
        &[
            OsString::from("plugin"),
            OsString::from("check"),
            OsString::from("--library"),
            scaffold_target
                .join("debug")
                .join(native_cdylib_name(NAME, target))
                .into_os_string(),
            OsString::from("--manifest"),
            crate_dir.join("rne-plugin.json").into_os_string(),
            OsString::from("--output"),
            report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    )
}

fn run_accelerator_scaffold_rehearsal(
    bundle_dir: &Path,
    output_dir: &Path,
    python: &Path,
    accelerator_conformance: &Path,
) -> bool {
    const NAME: &str = "release_scaffold_accelerator";
    let parent = output_dir.join("accelerator-authoring");
    if !run_check_command(
        "scaffold accelerator adapter",
        bundle_dir,
        accelerator_conformance,
        &[
            OsString::from("scaffold"),
            OsString::from(NAME),
            OsString::from("--dir"),
            parent.clone().into_os_string(),
            OsString::from("--schema"),
            OsString::from("1"),
        ],
        &[],
    ) {
        return false;
    }
    let directory = parent.join(NAME);
    let readme = match fs::read_to_string(directory.join("README.md")) {
        Ok(readme) => readme,
        Err(error) => {
            eprintln!("could not read accelerator scaffold README: {error}");
            return false;
        }
    };
    if !readme.contains("cannot qualify as independent evidence") {
        eprintln!("accelerator scaffold omitted its nonqualifying-evidence warning");
        return false;
    }
    let contract_path = directory.join("rne-scaffold.json");
    let contract = match fs::read(&contract_path)
        .map_err(anyhow::Error::from)
        .and_then(|bytes| AcceleratorScaffoldContract::from_json_slice(&bytes).map_err(Into::into))
    {
        Ok(contract) => contract,
        Err(error) => {
            eprintln!("could not validate accelerator scaffold contract: {error:#}");
            return false;
        }
    };
    if let Err(error) = contract.validate_directory(&directory) {
        eprintln!("accelerator scaffold directory differs from its contract: {error}");
        return false;
    }
    let report = output_dir.join("accelerator-scaffold-conformance.json");
    run_check_command(
        "scaffolded accelerator conformance",
        bundle_dir,
        accelerator_conformance,
        &[
            OsString::from("--adapter"),
            python.as_os_str().to_os_string(),
            OsString::from("--adapter-arg"),
            directory.join("adapter.py").into_os_string(),
            OsString::from("--subject"),
            directory.join("adapter.py").into_os_string(),
            OsString::from("--manifest"),
            directory.join("accelerator.toml").into_os_string(),
            OsString::from("--runtime"),
            directory.join("runtime.toml").into_os_string(),
            OsString::from("--task"),
            directory.join("task.json").into_os_string(),
            OsString::from("--output"),
            report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(
        &report,
        "status",
        &serde_json::Value::String("passed".to_string()),
    )
}

fn run_python_wheel_smoke(
    bundle_dir: &Path,
    output_dir: &Path,
    python: &Path,
    target: &str,
) -> (bool, bool) {
    let wheels = match files_with_extension(&bundle_dir.join("wheels"), "whl") {
        Ok(wheels) if wheels.len() == 1 => wheels,
        Ok(wheels) => {
            eprintln!("expected exactly one wheel, found {}", wheels.len());
            return (false, false);
        }
        Err(error) => {
            eprintln!("could not enumerate bundled wheel: {error:#}");
            return (false, false);
        }
    };
    let venv = output_dir.join("wheel-venv");
    if venv.exists() {
        if let Err(error) = fs::remove_dir_all(&venv) {
            eprintln!("could not reset wheel venv {}: {error}", venv.display());
            return (false, false);
        }
    }
    if !run_check_command(
        "create wheel smoke venv",
        output_dir,
        python,
        &[
            OsString::from("-m"),
            OsString::from("venv"),
            venv.clone().into_os_string(),
        ],
        &[],
    ) {
        return (false, false);
    }
    let installed_python = if target.contains("windows") {
        venv.join("Scripts/python.exe")
    } else {
        venv.join("bin/python")
    };
    if !run_check_command(
        "install bundled ABI3 wheel",
        output_dir,
        &installed_python,
        &[
            OsString::from("-m"),
            OsString::from("pip"),
            OsString::from("install"),
            OsString::from("--disable-pip-version-check"),
            OsString::from("--no-index"),
            OsString::from("--no-deps"),
            OsString::from("--force-reinstall"),
            wheels[0].clone().into_os_string(),
        ],
        &[],
    ) {
        return (false, false);
    }
    let wheel_passed = run_check_command(
        "execute ABI3 wheel smoke",
        output_dir,
        &installed_python,
        &[bundle_dir.join("python-wheel-smoke.py").into_os_string()],
        &[],
    );
    let api_report = output_dir.join("python-api-report.json");
    let api_passed = run_check_command(
        "verify installed Python API contract",
        output_dir,
        &installed_python,
        &[
            bundle_dir.join("python-api-compat.py").into_os_string(),
            OsString::from("--fixture"),
            bundle_dir
                .join("sdk/python/rne_py-api-v1.json")
                .into_os_string(),
            OsString::from("--output"),
            api_report.clone().into_os_string(),
        ],
        &[],
    ) && json_field_matches(&api_report, "passed", &serde_json::Value::Bool(true));
    (wheel_passed, api_passed)
}

fn check(id: &str, passed: bool) -> InstallCheck {
    InstallCheck {
        id: id.to_string(),
        status: if passed { "passed" } else { "failed" }.to_string(),
    }
}

fn validate_installed_flagship_proof(
    root: &Path,
    producer: &Path,
) -> anyhow::Result<InstalledFlagshipProofReport> {
    let report_path = root.join("installed-proof-report.json");
    let report: InstalledFlagshipProofReport = serde_json::from_slice(
        &fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?,
    )
    .with_context(|| format!("parse {}", report_path.display()))?;
    anyhow::ensure!(
        report.kind == rne_asset_cli::INSTALLED_FLAGSHIP_PROOF_REPORT_KIND,
        "unexpected installed flagship proof kind {}",
        report.kind
    );
    anyhow::ensure!(
        report.schema_version == rne_asset_cli::INSTALLED_FLAGSHIP_PROOF_REPORT_SCHEMA_VERSION,
        "unsupported installed flagship proof schema {}",
        report.schema_version
    );
    anyhow::ensure!(report.status == "passed", "installed flagship proof failed");
    anyhow::ensure!(
        report.task_id == "rne.flagship.mobile_lift_shared_aisle.v1",
        "unexpected installed flagship TaskSpec {}",
        report.task_id
    );
    anyhow::ensure!(
        report.physics_execution_paths == ["rapier_native", "mujoco_native"],
        "installed flagship proof must execute the packaged Rapier and MuJoCo paths"
    );
    anyhow::ensure!(
        report.success_status == "passed"
            && report.expected_failure_contract == "perception_stream_alive"
            && report.first_violation_step > 0
            && report.capsule_verified
            && report.recorded_shadow_status.as_deref() == Some("passed")
            && report.recorded_shadow_case_count == 3
            && report.installed_bundle_verified
            && report.bundle_verification_report.is_some(),
        "installed flagship proof omitted required success/failure evidence"
    );
    let producer_name = producer
        .file_name()
        .and_then(OsStr::to_str)
        .context("installed flagship producer has no Unicode file name")?;
    anyhow::ensure!(
        matches!(
            producer_name,
            "rne-flagship-proof" | "rne-flagship-proof.exe"
        ),
        "installed flagship producer has an unexpected file name"
    );
    let expected_producer_path = format!("bin/{producer_name}");
    validate_member_identity(
        &report.producer_executable,
        &expected_producer_path,
        &fs::read(producer).with_context(|| format!("read {}", producer.display()))?,
    )?;

    let expected_paths = [
        "cross-backend-report.json",
        "failure-capsule/capsule.json",
        "failure-minimized.rne-replay",
        "failure.behavior-report.json",
        "flagship.task.json",
        "installed-bundle-verification.json",
        "mujoco-failure.behavior-report.json",
        "mujoco-failure.rne-replay",
        "mujoco-success.behavior-report.json",
        "rapier-minimized-failure.behavior-report.json",
        "recorded-shadow-calibration.json",
        "recorded-shadow-controller.json",
        "recorded-shadow-disconnect.report.json",
        "recorded-shadow-disconnect.session.json",
        "recorded-shadow-mujoco.trace.json",
        "recorded-shadow-playback.report.json",
        "recorded-shadow-playback.session.json",
        "recorded-shadow-proof.json",
        "recorded-shadow-rapier.trace.json",
        "recorded-shadow-requirements.json",
        "recorded-shadow-shadow.report.json",
        "recorded-shadow-shadow.session.json",
        "replay-inspector.html",
        "success.behavior-report.json",
        "workflow-report.json",
    ];
    anyhow::ensure!(
        report
            .artifacts
            .iter()
            .map(|artifact| artifact.path.as_str())
            .eq(expected_paths),
        "installed flagship proof artifact set is incomplete or not canonically ordered"
    );
    for artifact in &report.artifacts {
        validate_proof_member(root, artifact, &artifact.path)?;
    }
    let bundle_verification = report
        .bundle_verification_report
        .as_ref()
        .context("installed flagship proof omitted bundle verification identity")?;
    let retained = report
        .artifacts
        .iter()
        .find(|artifact| artifact.path == "installed-bundle-verification.json")
        .context("installed flagship proof omitted bundle verification artifact")?;
    anyhow::ensure!(
        bundle_verification == retained,
        "installed flagship proof bundle-verification identities differ"
    );
    validate_installed_bundle_verification(root, producer)?;
    validate_installed_recorded_shadow_proof(root)?;
    Ok(report)
}

fn validate_installed_bundle_verification(root: &Path, producer: &Path) -> anyhow::Result<()> {
    let report_path = root.join("installed-bundle-verification.json");
    let retained: rne_asset_cli::installed_bundle::InstalledBundleVerificationReport =
        serde_json::from_slice(
            &fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?,
        )
        .with_context(|| format!("parse {}", report_path.display()))?;
    let bundle_root = producer
        .parent()
        .and_then(Path::parent)
        .context("installed flagship producer is not inside a bundle bin directory")?;
    let fresh = rne_asset_cli::installed_bundle::verify(bundle_root)
        .context("reverify installed release bundle from retained proof")?;
    anyhow::ensure!(
        retained == fresh,
        "retained installed-bundle verification differs from a fresh full verification"
    );
    Ok(())
}

fn validate_installed_recorded_shadow_proof(root: &Path) -> anyhow::Result<()> {
    let path = root.join("recorded-shadow-proof.json");
    let proof: InstalledRecordedShadowProof = serde_json::from_slice(
        &fs::read(&path).with_context(|| format!("read {}", path.display()))?,
    )
    .with_context(|| format!("parse {}", path.display()))?;
    anyhow::ensure!(
        proof.kind == "rne_installed_recorded_shadow_proof"
            && proof.schema_version == 1
            && proof.status == "passed"
            && proof.task_id == "rne.flagship.mobile_lift_shared_aisle.v1"
            && proof.controller_id == "rne.ai.ik_mobile_lift_pick_place_policy.v1"
            && proof.clock_source == "sim_clock_fixed_step",
        "installed recorded/shadow proof identity or status is invalid"
    );
    let expected = [
        (
            "playback",
            rne_hardware_gateway::HardwareMode::Playback,
            "passed",
            512,
            "recorded-shadow-playback.session.json",
            "recorded-shadow-playback.report.json",
        ),
        (
            "shadow",
            rne_hardware_gateway::HardwareMode::Shadow,
            "failed",
            512,
            "recorded-shadow-shadow.session.json",
            "recorded-shadow-shadow.report.json",
        ),
        (
            "disconnect",
            rne_hardware_gateway::HardwareMode::Shadow,
            "failed_as_expected",
            128,
            "recorded-shadow-disconnect.session.json",
            "recorded-shadow-disconnect.report.json",
        ),
    ];
    anyhow::ensure!(
        proof.cases.len() == expected.len(),
        "installed recorded/shadow proof must contain three cases"
    );
    for (case, (id, mode, status, samples, session, report)) in proof.cases.iter().zip(expected) {
        anyhow::ensure!(
            case.id == id
                && case.mode == mode
                && case.expected_status == status
                && case.observed_status == status
                && case.accepted_samples == samples
                && case.suppressed_actions == samples
                && !case.actuator_writes_emitted
                && case.session == session
                && case.report == report,
            "installed recorded/shadow case {id} violates its retained contract"
        );
        if id == "shadow" {
            anyhow::ensure!(
                case.violating_elements > 0 && case.first_divergence_tensor.is_some(),
                "installed shadow case omitted its measured divergence"
            );
        } else {
            anyhow::ensure!(
                case.violating_elements == 0 && case.first_divergence_tensor.is_none(),
                "installed {id} case unexpectedly contains numeric divergence"
            );
        }
    }
    Ok(())
}

fn validate_time_to_proof_report(root: &Path, expected_machine: &str) -> anyhow::Result<()> {
    let report_path = root.join("time-to-proof-report.json");
    let report: TimeToProofReport = serde_json::from_slice(
        &fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?,
    )
    .with_context(|| format!("parse {}", report_path.display()))?;
    anyhow::ensure!(
        report.kind == rne_asset_cli::TIME_TO_PROOF_REPORT_KIND
            && report.schema_version == rne_asset_cli::TIME_TO_PROOF_REPORT_SCHEMA_VERSION,
        "unexpected time-to-proof report kind or schema"
    );
    anyhow::ensure!(
        report.status == "passed"
            && report.within_target
            && report.elapsed_ms <= report.target_ms
            && report.target_ms == 15 * 60 * 1_000,
        "time-to-proof measurement exceeded its 15-minute target"
    );
    anyhow::ensure!(
        report.task_id == "rne.flagship.mobile_lift_shared_aisle.v1"
            && report.machine_label == expected_machine
            && !report.operating_system.is_empty()
            && !report.architecture.is_empty()
            && report.measurement_scope
                == "verified_installed_bundle_to_verified_capsule_and_bound_report",
        "time-to-proof measurement identity does not match the installed rehearsal"
    );
    validate_proof_member(
        root,
        &report.installed_bundle_verification,
        "installed-bundle-verification.json",
    )?;
    validate_proof_member(
        root,
        &report.installed_proof_report,
        "installed-proof-report.json",
    )?;
    validate_proof_member(
        root,
        &report.failure_capsule_manifest,
        "failure-capsule/capsule.json",
    )?;
    Ok(())
}

fn validate_proof_member(root: &Path, member: &MemberDigest, expected: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        member.path == expected && !member.path.contains('\\'),
        "installed proof artifact path does not match {expected}"
    );
    let relative = Path::new(&member.path);
    validate_relative_member(relative)?;
    validate_sha256_hex(
        &format!("installed proof artifact {}", member.path),
        &member.sha256,
    )?;
    let path = root.join(relative);
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("inspect installed proof artifact {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "installed proof artifact must be a regular file: {}",
        path.display()
    );
    anyhow::ensure!(
        metadata.len() == member.size_bytes,
        "installed proof artifact size mismatch: {}",
        member.path
    );
    anyhow::ensure!(
        sha256_file_hex(&path)? == member.sha256,
        "installed proof artifact SHA-256 mismatch: {}",
        member.path
    );
    Ok(())
}

fn json_field_matches(path: &Path, field: &str, expected: &serde_json::Value) -> bool {
    let result = fs::read(path)
        .map_err(anyhow::Error::from)
        .and_then(|bytes| serde_json::from_slice::<serde_json::Value>(&bytes).map_err(Into::into));
    match result {
        Ok(value) if value.get(field) == Some(expected) => true,
        Ok(value) => {
            eprintln!(
                "{} field {field:?} did not match {expected}: {:?}",
                path.display(),
                value.get(field)
            );
            false
        }
        Err(error) => {
            eprintln!("could not validate {}: {error:#}", path.display());
            false
        }
    }
}

fn run_check_command(
    label: &str,
    cwd: &Path,
    program: &Path,
    args: &[OsString],
    envs: &[(OsString, OsString)],
) -> bool {
    println!("$ {} ({label})", program.display());
    match command_output(cwd, program, args, envs) {
        Ok(output) => {
            print_output(&output);
            if !output.status.success() {
                eprintln!("{label} failed with status {}", output.status);
            }
            output.status.success()
        }
        Err(error) => {
            eprintln!("{label} could not start: {error:#}");
            false
        }
    }
}

fn command_output(
    cwd: &Path,
    program: &Path,
    args: &[OsString],
    envs: &[(OsString, OsString)],
) -> anyhow::Result<Output> {
    Command::new(program)
        .current_dir(cwd)
        .args(args)
        .envs(envs.iter().cloned())
        .output()
        .with_context(|| format!("run {}", program.display()))
}

fn print_output(output: &Output) {
    print!("{}", String::from_utf8_lossy(&output.stdout));
    eprint!("{}", String::from_utf8_lossy(&output.stderr));
}

fn ensure_success(label: &str, output: &Output) -> anyhow::Result<()> {
    anyhow::ensure!(
        output.status.success(),
        "{label} failed with status {}",
        output.status
    );
    Ok(())
}

fn program_version(program: &str) -> anyhow::Result<String> {
    let output = Command::new(program)
        .arg("--version")
        .output()
        .with_context(|| format!("run required release program {program} --version"))?;
    ensure_success(&format!("{program} --version"), &output)?;
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Ensures a release tag for the current version actually starts the release workflow.
pub(crate) fn validate_release_workflow_contract(root: &Path) -> anyhow::Result<()> {
    let path = root.join(".github/workflows/release.yml");
    let text = fs::read_to_string(&path)
        .with_context(|| format!("read release workflow {}", path.display()))?;
    validate_release_workflow_text(&text)
        .with_context(|| format!("validate release workflow {}", path.display()))
}

fn validate_release_workflow_text(text: &str) -> anyhow::Result<()> {
    let parts = RELEASE_VERSION.split('.').collect::<Vec<_>>();
    anyhow::ensure!(
        parts.len() == 3 && parts.iter().all(|part| !part.is_empty()),
        "release version must have exactly three non-empty components"
    );

    let expected_tag_trigger = format!("tags: [\"v{}.{}.*\"]", parts[0], parts[1]);
    let tag_triggers = text
        .lines()
        .map(str::trim)
        .filter(|line| line.starts_with("tags:"))
        .collect::<Vec<_>>();
    anyhow::ensure!(
        tag_triggers == [expected_tag_trigger.as_str()],
        "release workflow tag trigger must be exactly `{expected_tag_trigger}`, found {tag_triggers:?}"
    );

    let expected_version = format!("RELEASE_VERSION: \"{RELEASE_VERSION}\"");
    let declared_versions = text
        .lines()
        .map(str::trim)
        .filter(|line| line.starts_with("RELEASE_VERSION:"))
        .collect::<Vec<_>>();
    anyhow::ensure!(
        declared_versions == [expected_version.as_str()],
        "release workflow version must be exactly `{expected_version}`, found {declared_versions:?}"
    );
    Ok(())
}

fn git_worktree_is_clean(root: &Path) -> anyhow::Result<bool> {
    Ok(git_output(root, &["status", "--porcelain", "--untracked-files=all"])?.is_empty())
}

fn validate_expected_tag(
    root: &Path,
    expected_tag: Option<&str>,
    commit: &str,
) -> anyhow::Result<bool> {
    let Some(tag) = expected_tag else {
        return Ok(false);
    };
    anyhow::ensure!(
        tag == format!("v{RELEASE_VERSION}"),
        "expected release tag must be v{RELEASE_VERSION}, got {tag}"
    );
    let reference = format!("refs/tags/{tag}^{{commit}}");
    let tag_commit = git_output(root, &["rev-parse", &reference])?;
    anyhow::ensure!(
        tag_commit == commit,
        "release tag {tag} points to {tag_commit}, not tested commit {commit}"
    );
    Ok(true)
}

fn git_output(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn absolute_from(root: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    }
}

fn reset_generated_child(parent: &Path, path: &Path, expected_name: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        path.parent() == Some(parent) && path.file_name() == Some(OsStr::new(expected_name)),
        "refusing to reset unexpected generated path {}",
        path.display()
    );
    reset_generated_directory(path)
}

fn remove_generated_child(parent: &Path, path: &Path, expected_name: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        path.parent() == Some(parent) && path.file_name() == Some(OsStr::new(expected_name)),
        "refusing to remove unexpected generated path {}",
        path.display()
    );
    let metadata = match fs::symlink_metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(error) => {
            return Err(error)
                .with_context(|| format!("inspect generated directory {}", path.display()));
        }
    };
    anyhow::ensure!(
        metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
        "refusing to remove non-directory generated path {}",
        path.display()
    );
    fs::remove_dir_all(path)
        .with_context(|| format!("remove generated directory {}", path.display()))
}

fn cleanup_install_rehearsal_transients(output_dir: &Path) -> anyhow::Result<()> {
    for name in [
        "wheel-venv",
        "controller-authoring",
        "accelerator-authoring",
    ] {
        remove_generated_child(output_dir, &output_dir.join(name), name)?;
    }
    Ok(())
}

fn reset_generated_directory(path: &Path) -> anyhow::Result<()> {
    if path.exists() {
        fs::remove_dir_all(path)
            .with_context(|| format!("remove generated directory {}", path.display()))?;
    }
    fs::create_dir_all(path)
        .with_context(|| format!("create generated directory {}", path.display()))
}

fn prepare_empty_directory(path: &Path) -> anyhow::Result<()> {
    if path.exists() {
        anyhow::ensure!(
            fs::read_dir(path)?.next().is_none(),
            "release-install-smoke output directory must be empty: {}",
            path.display()
        );
    } else {
        fs::create_dir_all(path)?;
    }
    Ok(())
}

fn copy_file(source: &Path, destination: &Path) -> anyhow::Result<()> {
    anyhow::ensure!(
        source.is_file(),
        "bundle source missing: {}",
        source.display()
    );
    if let Some(parent) = destination.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::copy(source, destination).with_context(|| {
        format!(
            "copy bundle member {} to {}",
            source.display(),
            destination.display()
        )
    })?;
    Ok(())
}

fn files_with_extension(directory: &Path, extension: &str) -> anyhow::Result<Vec<PathBuf>> {
    let mut files = fs::read_dir(directory)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.is_file() && path.extension() == Some(OsStr::new(extension)))
        .collect::<Vec<_>>();
    files.sort();
    Ok(files)
}

fn collect_files(root: &Path) -> anyhow::Result<Vec<PathBuf>> {
    fn visit(directory: &Path, files: &mut Vec<PathBuf>) -> anyhow::Result<()> {
        let mut entries = fs::read_dir(directory)?.collect::<Result<Vec<_>, _>>()?;
        entries.sort_by_key(std::fs::DirEntry::file_name);
        for entry in entries {
            let path = entry.path();
            let file_type = entry.file_type()?;
            anyhow::ensure!(
                !file_type.is_symlink(),
                "bundle contains symbolic link {}",
                path.display()
            );
            if file_type.is_dir() {
                visit(&path, files)?;
            } else if file_type.is_file() {
                files.push(path);
            } else {
                bail!("bundle contains unsupported member {}", path.display());
            }
        }
        Ok(())
    }

    let mut files = Vec::new();
    visit(root, &mut files)?;
    Ok(files)
}

fn collect_member_digests(root: &Path, excluded: &[&str]) -> anyhow::Result<Vec<MemberDigest>> {
    let mut members = Vec::new();
    for path in collect_files(root)? {
        let relative = member_path(root, &path)?;
        if excluded.contains(&relative.as_str()) {
            continue;
        }
        let bytes = fs::read(&path)?;
        members.push(MemberDigest {
            path: relative,
            size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
            sha256: sha256_hex(&bytes),
        });
    }
    members.sort_by(|left, right| left.path.cmp(&right.path));
    Ok(members)
}

fn member_path(root: &Path, path: &Path) -> anyhow::Result<String> {
    let relative = path.strip_prefix(root)?;
    validate_relative_member(relative)?;
    Ok(relative.to_string_lossy().replace('\\', "/"))
}

fn validate_relative_member(path: &Path) -> anyhow::Result<()> {
    anyhow::ensure!(
        !path.as_os_str().is_empty()
            && !path.is_absolute()
            && path
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "invalid bundle member path {}",
        path.display()
    );
    Ok(())
}

fn write_sha256_manifest(root: &Path) -> anyhow::Result<()> {
    let members = collect_member_digests(root, &[SHA256_MANIFEST])?;
    let mut text = String::new();
    for member in members {
        text.push_str(&member.sha256);
        text.push_str("  ");
        text.push_str(&member.path);
        text.push('\n');
    }
    fs::write(root.join(SHA256_MANIFEST), text)?;
    Ok(())
}

fn verify_sha256_manifest(root: &Path) -> anyhow::Result<()> {
    let bytes = fs::read(root.join(SHA256_MANIFEST))
        .with_context(|| format!("read {SHA256_MANIFEST} from {}", root.display()))?;
    let declared = parse_sha256_manifest(&bytes)?;
    let actual = collect_member_digests(root, &[SHA256_MANIFEST])?
        .into_iter()
        .map(|member| (member.path, member.sha256))
        .collect::<BTreeMap<_, _>>();
    anyhow::ensure!(
        declared == actual,
        "bundle SHA256SUMS does not match its members"
    );
    Ok(())
}

fn parse_sha256_manifest(bytes: &[u8]) -> anyhow::Result<BTreeMap<String, String>> {
    let text = std::str::from_utf8(bytes).context("SHA256SUMS is not UTF-8")?;
    anyhow::ensure!(
        !text.is_empty() && text.ends_with('\n') && !text.contains('\r'),
        "SHA256SUMS must be non-empty canonical LF-terminated text"
    );
    let mut declared = BTreeMap::new();
    for line in text.lines() {
        let (digest, path) = line
            .split_once("  ")
            .context("SHA256SUMS entries must use `<digest>  <path>`")?;
        validate_sha256_hex(&format!("SHA256SUMS member {path}"), digest)?;
        anyhow::ensure!(
            !path.contains('\\'),
            "SHA256SUMS paths must use forward slashes"
        );
        let path_buf = PathBuf::from(path);
        validate_relative_member(&path_buf)?;
        anyhow::ensure!(
            declared
                .insert(path.to_string(), digest.to_string())
                .is_none(),
            "duplicate SHA256SUMS member {path}"
        );
    }
    Ok(declared)
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn sha256_file_hex(path: &Path) -> anyhow::Result<String> {
    let mut file = fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let count = file
            .read(&mut buffer)
            .with_context(|| format!("hash {}", path.display()))?;
        if count == 0 {
            break;
        }
        digest.update(&buffer[..count]);
    }
    Ok(format!("{:x}", digest.finalize()))
}

fn pretty_json_bytes(value: &impl Serialize) -> anyhow::Result<Vec<u8>> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    Ok(bytes)
}

fn write_pretty_json(path: &Path, value: &impl Serialize) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, pretty_json_bytes(value)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_external_submission_candidate() -> ExternalFlagshipSubmissionCandidate {
        ExternalFlagshipSubmissionCandidate {
            kind: EXTERNAL_FLAGSHIP_SUBMISSION_KIND.to_string(),
            schema_version: EXTERNAL_FLAGSHIP_SUBMISSION_SCHEMA_VERSION,
            candidate_status: EXTERNAL_FLAGSHIP_CANDIDATE_STATUS.to_string(),
            author_assistance: false,
            evidence_repository: SubmissionRepository {
                owner: "external-owner".to_string(),
                url: "https://github.com/external-owner/rne-reproduction".to_string(),
            },
            measurement: SubmissionMeasurement {
                measured_on: "2026-08-27".to_string(),
                machine_label: "community-lab-desktop-a".to_string(),
                operating_system: "windows".to_string(),
                architecture: "x86_64".to_string(),
                release_target: "x86_64-pc-windows-msvc".to_string(),
                elapsed_ms: 21_921,
                target_ms: 15 * 60 * 1_000,
            },
            release_archive: SubmissionArtifact {
                url: "https://example.invalid/rne-0.2.0-windows.zip".to_string(),
                file_name: "rne-0.2.0-windows.zip".to_string(),
                size_bytes: 7,
                sha256: sha256_hex(b"archive"),
            },
            proof_bundle: SubmissionArtifact {
                url: "https://example.invalid/proof.zip".to_string(),
                file_name: "proof.zip".to_string(),
                size_bytes: 5,
                sha256: sha256_hex(b"proof"),
            },
            required_proof_paths: EXTERNAL_FLAGSHIP_REQUIRED_PROOF_PATHS
                .map(str::to_string)
                .to_vec(),
            reproduction: SubmissionReproduction {
                commands: vec![
                    "verify archive".to_string(),
                    "extract archive".to_string(),
                    "run proof".to_string(),
                ],
                exit_statuses: vec![0, 0, 0],
                stdout_log_path: "logs/stdout.txt".to_string(),
                stderr_log_path: "logs/stderr.txt".to_string(),
            },
        }
    }

    #[test]
    fn staged_external_flagship_report_rebinds_every_retained_input() {
        let directory = tempfile::tempdir().expect("external flagship evidence");
        let archive = directory.path().join("rne-0.2.0-windows.zip");
        let proof_bundle = directory.path().join("proof.zip");
        let candidate_path = directory.path().join("candidate.json");
        let stdout = directory.path().join("stdout.txt");
        let stderr = directory.path().join("stderr.txt");
        fs::write(&archive, b"archive").expect("archive");
        fs::write(&proof_bundle, b"proof").expect("proof bundle");
        fs::write(&stdout, b"stdout").expect("stdout");
        fs::write(&stderr, b"stderr").expect("stderr");
        let candidate = valid_external_submission_candidate();
        write_pretty_json(&candidate_path, &candidate).expect("candidate");

        let mut stdout_member = digest_external_file(&stdout, "stdout").expect("stdout digest");
        stdout_member.path = candidate.reproduction.stdout_log_path.clone();
        let mut stderr_member = digest_external_file(&stderr, "stderr").expect("stderr digest");
        stderr_member.path = candidate.reproduction.stderr_log_path.clone();
        let mut submission_member =
            digest_external_file(&candidate_path, "candidate").expect("candidate digest");
        submission_member.path = "submissions/candidate.json".to_string();
        let retained_member = MemberDigest {
            path: "retained.json".to_string(),
            size_bytes: 1,
            sha256: sha256_hex(b"x"),
        };
        let report = ExternalFlagshipReproductionReport {
            kind: "rne_external_flagship_reproduction_report".to_string(),
            schema_version: EXTERNAL_FLAGSHIP_REPRODUCTION_REPORT_SCHEMA_VERSION,
            status: "passed".to_string(),
            owner: candidate.evidence_repository.owner.clone(),
            repository: candidate.evidence_repository.url.clone(),
            revision: "a".repeat(40),
            measured_on: candidate.measurement.measured_on.clone(),
            author_assistance: false,
            release_version: RELEASE_VERSION.to_string(),
            release_revision: "b".repeat(40),
            release_target: candidate.measurement.release_target.clone(),
            machine_label: candidate.measurement.machine_label.clone(),
            operating_system: candidate.measurement.operating_system.clone(),
            architecture: candidate.measurement.architecture.clone(),
            elapsed_ms: candidate.measurement.elapsed_ms,
            target_ms: candidate.measurement.target_ms,
            task_id: "rne.flagship.mobile_lift_shared_aisle.v1".to_string(),
            physics_execution_paths: vec!["rapier_native".to_string(), "mujoco_native".to_string()],
            first_violation_step: 240,
            first_violation_sim_time_ticks: 2_000_000_000,
            archive: digest_external_file(&archive, "archive").expect("archive digest"),
            proof_bundle: digest_external_file(&proof_bundle, "proof").expect("proof digest"),
            submission_candidate: submission_member,
            stdout_log: stdout_member,
            stderr_log: stderr_member,
            release_report: retained_member.clone(),
            checksum_manifest: retained_member.clone(),
            producer_executable: retained_member.clone(),
            installed_proof_report: retained_member.clone(),
            time_to_proof_report: retained_member.clone(),
            cross_backend_report: retained_member.clone(),
            failure_capsule_manifest: retained_member,
        };
        let report_bytes = pretty_json_bytes(&report).expect("report");
        let staged = || StagedExternalFlagshipReproduction {
            owner: &candidate.evidence_repository.owner,
            repository: &candidate.evidence_repository.url,
            revision: &report.revision,
            measured_on: &candidate.measurement.measured_on,
            release_archive: &archive,
            proof_bundle: &proof_bundle,
            submission_candidate: &candidate_path,
            stdout_log: &stdout,
            stderr_log: &stderr,
        };
        validate_staged_external_flagship_report(&report_bytes, staged())
            .expect("complete retained chain");

        fs::write(&stdout, b"tampered").expect("tamper stdout");
        assert!(validate_staged_external_flagship_report(&report_bytes, staged()).is_err());
    }

    #[test]
    fn release_target_is_bounded_and_cannot_escape() {
        assert!(validate_release_target("x86_64-unknown-linux-gnu").is_ok());
        assert!(validate_release_target("x86_64-pc-windows-msvc").is_ok());
        assert!(validate_release_target("../windows").is_err());
        assert!(validate_release_target("macos").is_err());
    }

    #[test]
    fn native_artifact_names_match_platform_conventions() {
        assert_eq!(
            native_binary_name("rne-asset", "x86_64-pc-windows-msvc"),
            "rne-asset.exe"
        );
        assert_eq!(
            native_plugin_name("x86_64-pc-windows-msvc"),
            "rne_plugin_example_velocity_servo.dll"
        );
        assert_eq!(
            native_plugin_name("x86_64-unknown-linux-gnu"),
            "librne_plugin_example_velocity_servo.so"
        );
        assert_eq!(
            native_cdylib_name("custom_controller", "aarch64-apple-darwin"),
            "libcustom_controller.dylib"
        );
    }

    #[test]
    fn successful_rehearsal_cleanup_is_bounded_and_preserves_reports() {
        let root = tempfile::tempdir().expect("temporary root");
        let output = root.path().join("evidence");
        fs::create_dir_all(output.join("wheel-venv/lib")).expect("wheel venv");
        fs::create_dir_all(output.join("controller-authoring/scaffold/target"))
            .expect("controller authoring");
        fs::create_dir_all(output.join("accelerator-authoring/scaffold"))
            .expect("accelerator authoring");
        fs::write(output.join("wheel-venv/lib/module.py"), b"temporary").expect("venv file");
        fs::write(
            output.join("controller-authoring/scaffold/target/plugin"),
            b"temporary",
        )
        .expect("controller build");
        fs::write(
            output.join("accelerator-authoring/scaffold/adapter.py"),
            b"temporary",
        )
        .expect("accelerator scaffold");
        fs::write(
            output.join("archive-install-rehearsal-report.json"),
            b"retained",
        )
        .expect("retained report");
        let outside = root.path().join("outside");
        fs::create_dir(&outside).expect("outside directory");
        fs::write(outside.join("keep.txt"), b"keep").expect("outside file");

        cleanup_install_rehearsal_transients(&output).expect("cleanup transients");

        assert!(!output.join("wheel-venv").exists());
        assert!(!output.join("controller-authoring").exists());
        assert!(!output.join("accelerator-authoring").exists());
        assert_eq!(
            fs::read(output.join("archive-install-rehearsal-report.json")).unwrap(),
            b"retained"
        );
        assert_eq!(fs::read(outside.join("keep.txt")).unwrap(), b"keep");
        assert!(remove_generated_child(&output, &outside, "outside").is_err());

        fs::write(output.join("wheel-venv"), b"not a directory").expect("regular file");
        assert!(cleanup_install_rehearsal_transients(&output).is_err());
        assert_eq!(
            fs::read(output.join("wheel-venv")).unwrap(),
            b"not a directory"
        );
    }

    #[cfg(unix)]
    #[test]
    fn successful_rehearsal_cleanup_refuses_symlinked_transients() {
        use std::os::unix::fs::symlink;

        let root = tempfile::tempdir().expect("temporary root");
        let output = root.path().join("evidence");
        let outside = root.path().join("outside");
        fs::create_dir_all(&output).expect("evidence directory");
        fs::create_dir_all(&outside).expect("outside directory");
        fs::write(outside.join("keep.txt"), b"keep").expect("outside file");
        symlink(&outside, output.join("wheel-venv")).expect("transient symlink");

        assert!(cleanup_install_rehearsal_transients(&output).is_err());
        assert!(fs::symlink_metadata(output.join("wheel-venv"))
            .unwrap()
            .file_type()
            .is_symlink());
        assert_eq!(fs::read(outside.join("keep.txt")).unwrap(), b"keep");
    }

    #[test]
    fn checksum_manifest_rejects_tampering_and_unlisted_files() {
        let directory = tempfile::tempdir().expect("temporary bundle");
        fs::write(directory.path().join("a.txt"), b"stable").expect("member");
        write_sha256_manifest(directory.path()).expect("manifest");
        let valid_manifest = fs::read_to_string(directory.path().join(SHA256_MANIFEST))
            .expect("read valid manifest");
        verify_sha256_manifest(directory.path()).expect("valid manifest");

        fs::write(directory.path().join("a.txt"), b"changed").expect("tamper");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::write(directory.path().join("a.txt"), b"stable").expect("restore");
        fs::write(directory.path().join("extra.txt"), b"extra").expect("extra");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::remove_file(directory.path().join("extra.txt")).expect("remove extra");

        fs::remove_file(directory.path().join("a.txt")).expect("remove member");
        assert!(verify_sha256_manifest(directory.path()).is_err());
        fs::write(directory.path().join("a.txt"), b"stable").expect("restore member");

        fs::write(
            directory.path().join(SHA256_MANIFEST),
            format!("{valid_manifest}{valid_manifest}"),
        )
        .expect("duplicate manifest entry");
        assert!(verify_sha256_manifest(directory.path()).is_err());

        let digest = sha256_hex(b"stable");
        fs::write(
            directory.path().join(SHA256_MANIFEST),
            format!("{digest}  ../a.txt\n"),
        )
        .expect("traversal manifest entry");
        assert!(verify_sha256_manifest(directory.path()).is_err());
    }

    #[test]
    fn installed_flagship_proof_rehashes_every_declared_artifact() {
        let directory = tempfile::tempdir().expect("temporary proof");
        let bundle_root = directory.path().join("rne-0.2.0-test-target");
        fs::create_dir_all(bundle_root.join("bin")).expect("bundle bin");
        fs::write(bundle_root.join("release-report.json"), b"release\n").expect("release report");
        let producer = bundle_root.join("bin").join(if cfg!(windows) {
            "rne-flagship-proof.exe"
        } else {
            "rne-flagship-proof"
        });
        fs::write(&producer, b"producer").expect("producer");
        write_sha256_manifest(&bundle_root).expect("bundle manifest");
        let verification = rne_asset_cli::installed_bundle::verify(&bundle_root)
            .expect("installed bundle verification");
        write_pretty_json(
            &directory.path().join("installed-bundle-verification.json"),
            &verification,
        )
        .expect("bundle verification report");
        let paths = [
            "cross-backend-report.json",
            "failure-capsule/capsule.json",
            "failure-minimized.rne-replay",
            "failure.behavior-report.json",
            "flagship.task.json",
            "installed-bundle-verification.json",
            "mujoco-failure.behavior-report.json",
            "mujoco-failure.rne-replay",
            "mujoco-success.behavior-report.json",
            "rapier-minimized-failure.behavior-report.json",
            "recorded-shadow-calibration.json",
            "recorded-shadow-controller.json",
            "recorded-shadow-disconnect.report.json",
            "recorded-shadow-disconnect.session.json",
            "recorded-shadow-mujoco.trace.json",
            "recorded-shadow-playback.report.json",
            "recorded-shadow-playback.session.json",
            "recorded-shadow-proof.json",
            "recorded-shadow-rapier.trace.json",
            "recorded-shadow-requirements.json",
            "recorded-shadow-shadow.report.json",
            "recorded-shadow-shadow.session.json",
            "replay-inspector.html",
            "success.behavior-report.json",
            "workflow-report.json",
        ];
        let artifacts: Vec<MemberDigest> = paths
            .into_iter()
            .map(|relative| {
                let path = directory.path().join(relative);
                fs::create_dir_all(path.parent().unwrap()).expect("artifact parent");
                let bytes = if relative == "installed-bundle-verification.json" {
                    fs::read(&path).expect("bundle verification report")
                } else if relative == "recorded-shadow-proof.json" {
                    serde_json::to_vec(&serde_json::json!({
                        "kind": "rne_installed_recorded_shadow_proof",
                        "schema_version": 1,
                        "status": "passed",
                        "task_id": "rne.flagship.mobile_lift_shared_aisle.v1",
                        "controller_id": "rne.ai.ik_mobile_lift_pick_place_policy.v1",
                        "clock_source": "sim_clock_fixed_step",
                        "cases": [
                            {
                                "id": "playback", "mode": "playback",
                                "expected_status": "passed", "observed_status": "passed",
                                "accepted_samples": 512, "violating_elements": 0,
                                "first_divergence_tensor": null, "suppressed_actions": 512,
                                "actuator_writes_emitted": false,
                                "session": "recorded-shadow-playback.session.json",
                                "report": "recorded-shadow-playback.report.json"
                            },
                            {
                                "id": "shadow", "mode": "shadow",
                                "expected_status": "failed", "observed_status": "failed",
                                "accepted_samples": 512, "violating_elements": 174,
                                "first_divergence_tensor": "lift_position_m",
                                "suppressed_actions": 512, "actuator_writes_emitted": false,
                                "session": "recorded-shadow-shadow.session.json",
                                "report": "recorded-shadow-shadow.report.json"
                            },
                            {
                                "id": "disconnect", "mode": "shadow",
                                "expected_status": "failed_as_expected",
                                "observed_status": "failed_as_expected",
                                "accepted_samples": 128, "violating_elements": 0,
                                "first_divergence_tensor": null, "suppressed_actions": 128,
                                "actuator_writes_emitted": false,
                                "session": "recorded-shadow-disconnect.session.json",
                                "report": "recorded-shadow-disconnect.report.json"
                            }
                        ]
                    }))
                    .expect("recorded/shadow proof fixture")
                } else {
                    format!("stable:{relative}").into_bytes()
                };
                fs::write(&path, &bytes).expect("artifact");
                MemberDigest {
                    path: relative.to_string(),
                    size_bytes: bytes.len() as u64,
                    sha256: sha256_hex(&bytes),
                }
            })
            .collect();
        let report = InstalledFlagshipProofReport {
            kind: rne_asset_cli::INSTALLED_FLAGSHIP_PROOF_REPORT_KIND.to_string(),
            schema_version: rne_asset_cli::INSTALLED_FLAGSHIP_PROOF_REPORT_SCHEMA_VERSION,
            status: "passed".to_string(),
            task_id: "rne.flagship.mobile_lift_shared_aisle.v1".to_string(),
            physics_execution_paths: vec!["rapier_native".to_string(), "mujoco_native".to_string()],
            success_status: "passed".to_string(),
            expected_failure_contract: "perception_stream_alive".to_string(),
            first_violation_step: 307,
            capsule_verified: true,
            recorded_shadow_status: Some("passed".to_string()),
            recorded_shadow_case_count: 3,
            installed_bundle_verified: true,
            bundle_verification_report: artifacts
                .iter()
                .find(|artifact| artifact.path == "installed-bundle-verification.json")
                .cloned(),
            producer_executable: MemberDigest {
                path: if cfg!(windows) {
                    "bin/rne-flagship-proof.exe"
                } else {
                    "bin/rne-flagship-proof"
                }
                .to_string(),
                size_bytes: 8,
                sha256: sha256_hex(b"producer"),
            },
            artifacts,
        };
        write_pretty_json(
            &directory.path().join("installed-proof-report.json"),
            &report,
        )
        .expect("proof report");
        validate_installed_flagship_proof(directory.path(), &producer).expect("valid proof");

        let recorded_proof_path = directory.path().join("recorded-shadow-proof.json");
        let recorded_proof_bytes = fs::read(&recorded_proof_path).expect("recorded proof");
        let mut recorded_proof: serde_json::Value =
            serde_json::from_slice(&recorded_proof_bytes).expect("recorded proof JSON");
        recorded_proof["cases"][0]["actuator_writes_emitted"] = serde_json::json!(true);
        write_pretty_json(&recorded_proof_path, &recorded_proof).expect("tampered recorded proof");
        assert!(validate_installed_recorded_shadow_proof(directory.path()).is_err());
        fs::write(&recorded_proof_path, recorded_proof_bytes).expect("restore recorded proof");

        let bound_member = |relative: &str| {
            let bytes = fs::read(directory.path().join(relative)).expect("bound member");
            MemberDigest {
                path: relative.to_string(),
                size_bytes: bytes.len() as u64,
                sha256: sha256_hex(&bytes),
            }
        };
        let timing = TimeToProofReport {
            kind: rne_asset_cli::TIME_TO_PROOF_REPORT_KIND.to_string(),
            schema_version: rne_asset_cli::TIME_TO_PROOF_REPORT_SCHEMA_VERSION,
            status: "passed".to_string(),
            task_id: "rne.flagship.mobile_lift_shared_aisle.v1".to_string(),
            machine_label: "test-machine".to_string(),
            operating_system: "test-os".to_string(),
            architecture: "test-arch".to_string(),
            measurement_scope: "verified_installed_bundle_to_verified_capsule_and_bound_report"
                .to_string(),
            elapsed_ms: 12_345,
            target_ms: 15 * 60 * 1_000,
            within_target: true,
            installed_bundle_verification: bound_member("installed-bundle-verification.json"),
            installed_proof_report: bound_member("installed-proof-report.json"),
            failure_capsule_manifest: bound_member("failure-capsule/capsule.json"),
        };
        write_pretty_json(&directory.path().join("time-to-proof-report.json"), &timing)
            .expect("timing report");
        validate_time_to_proof_report(directory.path(), "test-machine")
            .expect("valid timing report");

        fs::write(
            directory.path().join("installed-proof-report.json"),
            b"tampered",
        )
        .expect("tamper proof report");
        assert!(validate_time_to_proof_report(directory.path(), "test-machine").is_err());
        write_pretty_json(
            &directory.path().join("installed-proof-report.json"),
            &report,
        )
        .expect("restore proof report");

        fs::write(directory.path().join("flagship.task.json"), b"tampered")
            .expect("tamper artifact");
        assert!(validate_installed_flagship_proof(directory.path(), &producer).is_err());
    }

    #[test]
    fn external_flagship_identity_and_machine_checks_fail_closed() {
        let revision = "0123456789abcdef0123456789abcdef01234567";
        validate_external_operator(
            "external-owner",
            "https://github.com/external-owner/rne-reproduction",
            revision,
            "2026-08-23",
        )
        .expect("valid independent operator");
        for (owner, repository, tested_revision, date) in [
            (
                "rsasaki0109",
                "https://github.com/rsasaki0109/reproduction",
                revision,
                "2026-08-23",
            ),
            (
                "external-owner",
                "https://github.com/different-owner/reproduction",
                revision,
                "2026-08-23",
            ),
            (
                "external-owner",
                "https://github.com/external-owner/reproduction",
                "ABCDEF",
                "2026-08-23",
            ),
            (
                "external-owner",
                "https://github.com/external-owner/reproduction",
                revision,
                "2026-02-30",
            ),
        ] {
            assert!(validate_external_operator(owner, repository, tested_revision, date).is_err());
        }
        validate_external_machine_label("community-lab-desktop-a").expect("named external machine");
        for label in [
            "",
            "test-machine",
            "github-hosted-release-rehearsal-windows-x86_64",
        ] {
            assert!(validate_external_machine_label(label).is_err());
        }
    }

    #[test]
    fn completed_external_submission_is_acyclic_and_fail_closed() {
        let candidate = valid_external_submission_candidate();
        validate_external_submission_candidate(&candidate).expect("valid completed candidate");

        let mut embedded_revision = serde_json::to_value(&candidate).expect("candidate JSON");
        embedded_revision["evidence_repository"]["revision"] = serde_json::json!("0".repeat(40));
        assert!(
            serde_json::from_value::<ExternalFlagshipSubmissionCandidate>(embedded_revision)
                .is_err()
        );

        let mut nonzero = candidate.clone();
        nonzero.reproduction.exit_statuses[1] = 1;
        assert!(validate_external_submission_candidate(&nonzero).is_err());

        let mut self_referential_path = candidate;
        self_referential_path.reproduction.stdout_log_path = "../candidate.json".to_string();
        assert!(validate_external_submission_candidate(&self_referential_path).is_err());
    }

    #[test]
    fn external_submission_checker_requires_separate_revision_and_artifacts() {
        let revision = "0123456789abcdef0123456789abcdef01234567";
        let mut arguments = vec![
            "--archive",
            "release.zip",
            "--bundle-dir",
            "bundle",
            "--proof-dir",
            "flagship-proof",
            "--proof-bundle",
            "proof.zip",
            "--submission",
            "candidate.json",
            "--evidence-repo-dir",
            "external-repo",
            "--revision",
            revision,
            "--output",
            "accepted.json",
        ]
        .into_iter()
        .map(str::to_string);
        let options = parse_external_flagship_options(&mut arguments).expect("complete options");
        assert_eq!(options.revision, revision);
        assert_eq!(options.proof_bundle, PathBuf::from("proof.zip"));
        assert_eq!(options.submission, PathBuf::from("candidate.json"));
        assert_eq!(options.evidence_repo_dir, PathBuf::from("external-repo"));

        let mut legacy = vec!["--owner".to_string(), "external-owner".to_string()].into_iter();
        assert!(parse_external_flagship_options(&mut legacy).is_err());
    }

    #[test]
    fn external_submission_files_must_be_exact_committed_repository_bytes() {
        let directory = tempfile::tempdir().expect("temporary external repository");
        let root = directory.path();
        let run_git = |args: &[&str]| {
            let output = Command::new("git")
                .current_dir(root)
                .args(args)
                .output()
                .expect("run git");
            assert!(
                output.status.success(),
                "git {:?}: {}",
                args,
                String::from_utf8_lossy(&output.stderr)
            );
        };
        run_git(&["init"]);
        run_git(&["config", "user.name", "External Operator"]);
        run_git(&["config", "user.email", "external@example.invalid"]);
        run_git(&[
            "remote",
            "add",
            "origin",
            "https://github.com/external-owner/rne-reproduction.git",
        ]);
        fs::write(root.join("candidate.json"), b"candidate\n").expect("candidate");
        fs::create_dir(root.join("logs")).expect("logs");
        fs::write(root.join("logs/stdout.txt"), b"stdout\n").expect("stdout");
        fs::write(root.join("logs/stderr.txt"), b"stderr\n").expect("stderr");
        run_git(&["add", "."]);
        run_git(&["commit", "-m", "retain independent reproduction"]);
        let revision = git_output(root, &["rev-parse", "HEAD"]).expect("revision");
        validate_external_repository_checkout(
            root,
            "https://github.com/external-owner/rne-reproduction",
            &revision,
        )
        .expect("exact repository checkout");
        validate_committed_external_file(
            root,
            &root.join("candidate.json"),
            "submission candidate",
        )
        .expect("exact candidate bytes");

        fs::write(root.join("candidate.json"), b"tampered\n").expect("tamper candidate");
        assert!(validate_external_repository_checkout(
            root,
            "https://github.com/external-owner/rne-reproduction",
            &revision
        )
        .is_err());
        assert!(validate_committed_external_file(
            root,
            &root.join("candidate.json"),
            "submission candidate"
        )
        .is_err());
    }

    #[test]
    fn submission_artifact_and_measurement_must_match_downloaded_bytes() {
        let directory = tempfile::tempdir().expect("temporary submission");
        let proof_bundle = directory.path().join("proof.zip");
        fs::write(&proof_bundle, b"proof").expect("proof bundle");
        let candidate = valid_external_submission_candidate();
        validate_submission_artifact(&candidate.proof_bundle, &proof_bundle, "proof bundle")
            .expect("matching proof bundle");

        let mut wrong_digest = candidate.proof_bundle.clone();
        wrong_digest.sha256 = "0".repeat(64);
        assert!(
            validate_submission_artifact(&wrong_digest, &proof_bundle, "proof bundle").is_err()
        );

        let timing = TimeToProofReport {
            kind: rne_asset_cli::TIME_TO_PROOF_REPORT_KIND.to_string(),
            schema_version: rne_asset_cli::TIME_TO_PROOF_REPORT_SCHEMA_VERSION,
            status: "passed".to_string(),
            task_id: "rne.flagship.mobile_lift_shared_aisle.v1".to_string(),
            machine_label: candidate.measurement.machine_label.clone(),
            operating_system: candidate.measurement.operating_system.clone(),
            architecture: candidate.measurement.architecture.clone(),
            measurement_scope: "verified_installed_bundle_to_verified_capsule_and_bound_report"
                .to_string(),
            elapsed_ms: candidate.measurement.elapsed_ms,
            target_ms: candidate.measurement.target_ms,
            within_target: true,
            installed_bundle_verification: MemberDigest {
                path: "installed-bundle-verification.json".to_string(),
                size_bytes: 1,
                sha256: "0".repeat(64),
            },
            installed_proof_report: MemberDigest {
                path: "installed-proof-report.json".to_string(),
                size_bytes: 1,
                sha256: "0".repeat(64),
            },
            failure_capsule_manifest: MemberDigest {
                path: "failure-capsule/capsule.json".to_string(),
                size_bytes: 1,
                sha256: "0".repeat(64),
            },
        };
        validate_submission_measurement(&candidate.measurement, &timing, "x86_64-pc-windows-msvc")
            .expect("matching measurement");
        let mut wrong_timing = timing;
        wrong_timing.elapsed_ms += 1;
        assert!(validate_submission_measurement(
            &candidate.measurement,
            &wrong_timing,
            "x86_64-pc-windows-msvc"
        )
        .is_err());
    }

    #[test]
    fn external_cross_backend_report_requires_canonical_semantics_and_units() {
        let directory = tempfile::tempdir().expect("temporary report");
        let path = directory.path().join("cross-backend-report.json");
        let tolerances = [
            ("completion_step_delta", "step", 500.0),
            ("base_planar_position_delta", "m", 0.4),
            ("payload_position_delta", "m", 0.06),
            ("payload_apex_delta", "m", 0.07),
            ("arm_joint_position_delta", "rad", 0.2),
            ("lift_position_delta", "m", 0.04),
            ("gripper_position_delta", "m", 0.04),
            ("wrist_depth_delta", "m", 0.02),
            ("total_reward_delta", "reward", 0.75),
        ]
        .map(|(id, unit, maximum_delta)| {
            serde_json::json!({
                "id": id,
                "unit": unit,
                "observed_delta": 0.0,
                "maximum_delta": maximum_delta,
                "status": "passed"
            })
        });
        let failure = |backend_id| {
            serde_json::json!({
                "backend_id": backend_id,
                "status": "passed",
                "expected_contract": "perception_stream_alive",
                "first_violation_step": 307,
                "first_violation_sim_time_ticks": 5_116_666_462_u64,
                "matched_replay_frames": 308
            })
        };
        let mut report = serde_json::json!({
            "kind": rne_asset_cli::FLAGSHIP_CROSS_BACKEND_REPORT_KIND,
            "schema_version": rne_asset_cli::FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION,
            "status": "passed",
            "task_id": "rne.flagship.mobile_lift_shared_aisle.v1",
            "controller_id": "rne.ai.ik_mobile_lift_pick_place_policy.v1",
            "controller_contract": "identical_controller_type_and_configuration_per_backend",
            "exact_outcomes": [
                "all_behavior_contracts_passed",
                "inspection_completed",
                "traffic_cleared_without_collision_or_signal_violation",
                "payload_grasped_once",
                "pick_place_completed",
                "terminated_without_truncation_or_fail_closed_abort"
            ],
            "backends": [
                {"backend_id": "rapier_native", "status": "passed"},
                {"backend_id": "mujoco_native", "status": "passed"}
            ],
            "tolerance_checks": tolerances,
            "failure_exact_outcomes": [
                "same_seed_and_minimized_fault_dimensions",
                "same_expected_contract",
                "same_first_violation_step",
                "same_first_violation_sim_time",
                "both_failure_replays_verified"
            ],
            "intentional_failures": [failure("rapier_native"), failure("mujoco_native")],
            "failure_tolerance_checks": [
                {"id": "first_violation_step_delta", "unit": "step", "observed_delta": 0.0, "maximum_delta": 0.0, "status": "passed"},
                {"id": "first_violation_time_delta", "unit": "ns", "observed_delta": 0.0, "maximum_delta": 0.0, "status": "passed"}
            ]
        });
        write_pretty_json(&path, &report).expect("cross-backend report");
        assert_eq!(
            validate_external_cross_backend_report(&path).expect("valid report"),
            (307, 5_116_666_462)
        );

        report["tolerance_checks"][1]["unit"] = serde_json::json!("cm");
        write_pretty_json(&path, &report).expect("mutated report");
        assert!(validate_external_cross_backend_report(&path).is_err());
    }

    #[test]
    fn bundled_mujoco_runtime_manifest_rehashes_runtime_and_licenses() {
        let directory = tempfile::tempdir().expect("temporary bundle");
        for (relative, bytes) in [
            ("bin/mujoco.dll", b"runtime".as_slice()),
            ("third-party/mujoco/LICENSE", b"license".as_slice()),
            (
                "third-party/mujoco/THIRD_PARTY_NOTICES.txt",
                b"notices".as_slice(),
            ),
        ] {
            let path = directory.path().join(relative);
            fs::create_dir_all(path.parent().unwrap()).expect("member parent");
            fs::write(path, bytes).expect("member");
        }
        let manifest = MujocoRuntimeManifest {
            kind: "rne_mujoco_runtime".to_string(),
            schema_version: MUJOCO_RUNTIME_MANIFEST_SCHEMA_VERSION,
            version: MUJOCO_VERSION.to_string(),
            source_url: format!(
                "https://github.com/google-deepmind/mujoco/releases/download/{MUJOCO_VERSION}/{MUJOCO_WINDOWS_ARCHIVE}"
            ),
            archive_file: MUJOCO_WINDOWS_ARCHIVE.to_string(),
            archive_sha256: MUJOCO_WINDOWS_ARCHIVE_SHA256.to_string(),
            runtime_members: vec![
                digest_member(directory.path(), "bin/mujoco.dll").expect("runtime digest"),
            ],
            license_members: [
                "third-party/mujoco/LICENSE",
                "third-party/mujoco/THIRD_PARTY_NOTICES.txt",
            ]
            .map(|relative| digest_member(directory.path(), relative).expect("license digest"))
            .to_vec(),
        };
        write_pretty_json(
            &directory
                .path()
                .join("third-party/mujoco/runtime-manifest.json"),
            &manifest,
        )
        .expect("runtime manifest");

        validate_mujoco_runtime(directory.path(), "x86_64-pc-windows-msvc").expect("valid runtime");
        fs::write(directory.path().join("bin/mujoco.dll"), b"tampered").expect("tamper runtime");
        assert!(validate_mujoco_runtime(directory.path(), "x86_64-pc-windows-msvc").is_err());
    }

    #[test]
    fn install_report_requires_every_frozen_workflow() {
        let report = InstallRehearsalReport {
            schema_version: INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
            release_version: RELEASE_VERSION.to_string(),
            target: "x86_64-unknown-linux-gnu".to_string(),
            status: "passed".to_string(),
            checks: INSTALL_CHECK_IDS.map(|id| check(id, true)).to_vec(),
        };
        assert!(report.all_passed());

        let mut duplicated = report;
        duplicated.checks[7].id = "robot_replay".to_string();
        assert!(!duplicated.all_passed());
    }

    #[test]
    fn release_report_reads_legacy_workflow_name_and_writes_precise_name() {
        let legacy = serde_json::json!({
            "schema_version": 1,
            "release_version": RELEASE_VERSION,
            "git_commit": "1".repeat(40),
            "target": "x86_64-unknown-linux-gnu",
            "rustc_version": "rustc-test",
            "cargo_version": "cargo-test",
            "cargo_lock_sha256": "0".repeat(64),
            "clean_worktree": true,
            "expected_tag": "v0.1.0",
            "tag_matches_commit": true,
            "reproducible": true,
            "audit": {
                "cargo_deny": "passed",
                "cargo_audit": "passed",
                "source_policy": "passed",
                "license_policy": "passed"
            },
            "fuzz_campaign_digest_sha256": "0".repeat(64),
            "contracts": {},
            "flagship_workflows": { "robot_replay": "passed" },
            "members": []
        });

        let report: ReleaseReport = serde_json::from_value(legacy).expect("legacy report");
        assert_eq!(
            report.installed_workflows.get("robot_replay"),
            Some(&"passed".to_string())
        );
        let current = serde_json::to_value(report).expect("current report");
        assert!(current.get("installed_workflows").is_some());
        assert!(current.get("flagship_workflows").is_none());
    }

    #[test]
    fn readiness_static_contract_is_staged_in_release_bundles() {
        let root = workspace_root().expect("workspace root");
        let output = tempfile::tempdir().expect("temporary bundle");
        stage_static_files(&root, output.path()).expect("stage bundle files");
        assert_eq!(
            fs::read(output.path().join("release/one-zero-readiness.toml")).unwrap(),
            fs::read(root.join("release/one-zero-readiness.toml")).unwrap()
        );
        assert_eq!(
            fs::read(
                output
                    .path()
                    .join("release/evidence/compatibility-report-v1.json")
            )
            .unwrap(),
            fs::read(root.join("release/evidence/compatibility-report-v1.json")).unwrap()
        );
        assert_eq!(
            fs::read(output.path().join("ONE_ZERO_READINESS.md")).unwrap(),
            fs::read(root.join("docs/ONE_ZERO_READINESS.md")).unwrap()
        );
        assert_eq!(
            fs::read(output.path().join("SUPPORT.md")).unwrap(),
            fs::read(root.join("docs/SUPPORT.md")).unwrap()
        );
        assert_eq!(
            fs::read(output.path().join("release/external-evidence-intake.toml")).unwrap(),
            fs::read(root.join("release/external-evidence-intake.toml")).unwrap()
        );
        assert_eq!(
            fs::read(
                output
                    .path()
                    .join("release/external-flagship-submission-template.json")
            )
            .unwrap(),
            fs::read(root.join("release/external-flagship-submission-template.json")).unwrap()
        );
        for template in [
            "external-project-submission-template.json",
            "external-plugin-submission-template.json",
            "external-simulator-submission-template.json",
        ] {
            assert_eq!(
                fs::read(output.path().join("release").join(template)).unwrap(),
                fs::read(root.join("release").join(template)).unwrap()
            );
        }
        for guide in [
            "EXTERNAL_EVIDENCE_INTAKE.md",
            "EXTERNAL_FLAGSHIP_REPRODUCTION.md",
            "EVIDENCE_QUICKSTART.md",
            "FAILURE_CAPSULE.md",
            "PLUGIN_SDK.md",
            "EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md",
            "HARDWARE_ADAPTER_CONFORMANCE.md",
            "ACCELERATOR_PROTOCOL.md",
        ] {
            assert_eq!(
                fs::read(output.path().join("docs").join(guide)).unwrap(),
                fs::read(root.join("docs").join(guide)).unwrap()
            );
        }
        assert_eq!(
            fs::read(output.path().join("Cargo.lock")).unwrap(),
            fs::read(root.join("Cargo.lock")).unwrap()
        );
    }

    #[test]
    fn readiness_release_reports_bind_the_archive_checksum_chain_and_workflows() {
        let directory = tempfile::tempdir().expect("temporary reports");
        let target = "x86_64-pc-windows-msvc";
        let bundle_dir = directory.path().join(bundle_name(target));
        fs::create_dir(&bundle_dir).unwrap();
        fs::write(bundle_dir.join("README.md"), b"x").unwrap();
        let archive_path = directory
            .path()
            .join(format!("{}.zip", bundle_name(target)));
        fs::write(&archive_path, b"deterministic archive bytes").unwrap();
        let release_path = bundle_dir.join(RELEASE_REPORT);
        let checksum_path = bundle_dir.join(SHA256_MANIFEST);
        let install_path = directory.path().join(ARCHIVE_INSTALL_REPORT);
        let time_to_proof_path = directory.path().join("time-to-proof-report.json");
        fs::write(&time_to_proof_path, b"timing proof\n").unwrap();
        let revision = "1".repeat(40);
        let tag = "v0.1.0-rc.1";
        let workflows = INSTALL_CHECK_IDS
            .into_iter()
            .map(|id| (id.to_string(), "passed".to_string()))
            .collect();
        let install = InstallRehearsalReport {
            schema_version: INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
            release_version: RELEASE_VERSION.to_string(),
            target: target.to_string(),
            status: "passed".to_string(),
            checks: INSTALL_CHECK_IDS.map(|id| check(id, true)).to_vec(),
        };
        write_pretty_json(&bundle_dir.join(INSTALL_REPORT), &install).unwrap();
        let mut release = ReleaseReport {
            schema_version: RELEASE_REPORT_SCHEMA_VERSION,
            release_version: RELEASE_VERSION.to_string(),
            git_commit: revision.clone(),
            target: target.to_string(),
            rustc_version: "rustc-test".to_string(),
            cargo_version: "cargo-test".to_string(),
            cargo_lock_sha256: "0".repeat(64),
            clean_worktree: true,
            expected_tag: Some(tag.to_string()),
            tag_matches_commit: true,
            reproducible: true,
            audit: AuditVerdicts {
                cargo_deny: "passed".to_string(),
                cargo_audit: "passed".to_string(),
                source_policy: "passed".to_string(),
                license_policy: "passed".to_string(),
            },
            fuzz_campaign_digest_sha256: "0".repeat(64),
            contracts: serde_json::json!({}),
            installed_workflows: workflows,
            members: Vec::new(),
        };
        release.members =
            collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST]).unwrap();
        write_pretty_json(&release_path, &release).unwrap();
        write_sha256_manifest(&bundle_dir).unwrap();
        let archive_install = build_archive_install_report(
            &archive_path,
            &bundle_dir,
            &release,
            &install,
            &time_to_proof_path,
        )
        .unwrap();
        assert_eq!(
            String::from_utf8(pretty_json_bytes(&archive_install).unwrap()).unwrap(),
            include_str!("../../tests/golden/release/archive-install-rehearsal-v2.json")
        );
        let mut unknown = serde_json::to_value(&archive_install).unwrap();
        unknown
            .as_object_mut()
            .unwrap()
            .insert("unexpected".to_string(), serde_json::Value::Bool(true));
        assert!(serde_json::from_value::<ArchiveInstallRehearsalReport>(unknown).is_err());
        write_pretty_json(&install_path, &archive_install).unwrap();
        let archive_sha256 = format!("sha256:{}", sha256_file_hex(&archive_path).unwrap());
        let validate = |expected_tag| {
            let release_sha256 = format!("sha256:{}", sha256_file_hex(&release_path).unwrap());
            let checksum_sha256 = format!("sha256:{}", sha256_file_hex(&checksum_path).unwrap());
            let install_sha256 = format!("sha256:{}", sha256_file_hex(&install_path).unwrap());
            validate_readiness_release_reports(
                ReadinessReleaseEvidence {
                    archive_path: &archive_path,
                    archive_sha256: &archive_sha256,
                    release_report_path: &release_path,
                    release_report_sha256: &release_sha256,
                    checksum_manifest_path: &checksum_path,
                    checksum_manifest_sha256: &checksum_sha256,
                    install_report_path: &install_path,
                    install_report_sha256: &install_sha256,
                },
                ReadinessReleaseIdentity {
                    target,
                    commit: &revision,
                    tag: expected_tag,
                },
            )
        };
        validate(tag).unwrap();

        assert!(validate("v0.1.0-rc.2").is_err());

        let original_install_sha256 = format!("sha256:{}", sha256_file_hex(&install_path).unwrap());
        let mut swapped_archive = archive_install.clone();
        swapped_archive.archive.sha256 = format!("sha256:{}", "0".repeat(64));
        write_pretty_json(&install_path, &swapped_archive).unwrap();
        assert!(read_bound_file(
            &install_path,
            &original_install_sha256,
            "archive-install report"
        )
        .is_err());
        assert!(validate(tag).is_err());

        write_pretty_json(&install_path, &archive_install).unwrap();
        fs::write(&checksum_path, b"0").unwrap();
        assert!(validate(tag).is_err());
    }

    #[test]
    fn release_workflow_accepts_the_current_release_series() {
        let workflow = format!(
            "on:\n  push:\n    tags: [\"v0.2.*\"]\nenv:\n  RELEASE_VERSION: \"{RELEASE_VERSION}\"\n"
        );
        validate_release_workflow_text(&workflow).unwrap();
    }

    #[test]
    fn committed_release_workflow_matches_current_release() {
        let root = workspace_root().expect("workspace root");
        validate_release_workflow_contract(&root).unwrap();
    }

    #[test]
    fn release_workflow_rejects_a_stale_tag_series() {
        let workflow = format!(
            "on:\n  push:\n    tags: [\"v0.1.*\"]\nenv:\n  RELEASE_VERSION: \"{RELEASE_VERSION}\"\n"
        );
        let error = validate_release_workflow_text(&workflow).unwrap_err();
        assert!(error.to_string().contains("v0.2.*"));
    }

    #[test]
    fn release_workflow_rejects_a_stale_declared_version() {
        let workflow = "on:\n  push:\n    tags: [\"v0.2.*\"]\nenv:\n  RELEASE_VERSION: \"0.1.0\"\n";
        let error = validate_release_workflow_text(workflow).unwrap_err();
        assert!(error.to_string().contains(RELEASE_VERSION));
    }
}
