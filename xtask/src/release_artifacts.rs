//! Cross-platform 1.0 RC bundle assembly and installed-artifact rehearsal.

use super::{
    cargo_metadata, fuzz_smoke, release_readiness, supply_chain, validate_blocker_registry,
    validate_contract_registry, validate_release_metadata, workspace_root, RELEASE_VERSION,
};
use anyhow::{bail, Context};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{OsStr, OsString};
use std::fs;
use std::io::Read;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Output};

/// Machine-readable release provenance report schema.
pub(crate) const RELEASE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Machine-readable installed-bundle rehearsal report schema.
pub(crate) const INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION: u32 = 4;
/// Archive-bound independently extracted rehearsal report schema.
pub(crate) const ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION: u32 = 1;
/// Installed Python public-API contract schema.
pub(crate) const PYTHON_API_CONTRACT_SCHEMA_VERSION: u32 = 1;
/// Installed Python public-API verification report schema.
pub(crate) const PYTHON_API_REPORT_SCHEMA_VERSION: u32 = 1;

const RELEASE_BINARY_PACKAGES: [(&str, &str); 6] = [
    ("rne_asset_cli", "rne-asset"),
    ("rne_compatibility_suite", "rne-compatibility"),
    ("rne_physics_conformance_suite", "rne-physics-conformance"),
    ("rne_scenario_scale", "rne-scenario-scale"),
    ("rne_hardware_gateway", "rne-hardware-conformance"),
    ("rne_hardware_gateway", "rne-hardware-mock-device"),
];
const RELEASE_PLUGIN_PACKAGE: &str = "rne_plugin_example_velocity_servo";
const SHA256_MANIFEST: &str = "SHA256SUMS";
const RELEASE_REPORT: &str = "release-report.json";
const INSTALL_REPORT: &str = "install-rehearsal-report.json";
const ARCHIVE_INSTALL_REPORT: &str = "archive-install-rehearsal-report.json";
const ARCHIVE_INSTALL_REPORT_KIND: &str = "rne_archive_install_rehearsal";
const INSTALL_CHECK_IDS: [&str; 9] = [
    "robot_replay",
    "scenario_replay",
    "physics_conformance",
    "scenario_scale_100",
    "hardware_adapter",
    "controller_plugin",
    "compatibility_corpus",
    "python_wheel",
    "python_api",
];

const BUNDLE_FILES: [(&str, &str); 61] = [
    ("README.md", "README.md"),
    ("CHANGELOG.md", "CHANGELOG.md"),
    ("LICENSE-MIT", "LICENSE-MIT"),
    ("LICENSE-APACHE", "LICENSE-APACHE"),
    ("Cargo.lock", "Cargo.lock"),
    ("docs/COMPATIBILITY.md", "COMPATIBILITY.md"),
    ("docs/RELEASE_INSTALL.md", "INSTALL.md"),
    ("docs/ONE_ZERO_READINESS.md", "ONE_ZERO_READINESS.md"),
    ("docs/EVIDENCE_QUICKSTART.md", "docs/EVIDENCE_QUICKSTART.md"),
    ("docs/FAILURE_CAPSULE.md", "docs/FAILURE_CAPSULE.md"),
    (
        "docs/EXTERNAL_EVIDENCE_INTAKE.md",
        "docs/EXTERNAL_EVIDENCE_INTAKE.md",
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
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
        "assets/scenes/mesh_diff_drive.rne.scene.toml",
    ),
    (
        "assets/scenes/mm_minimal.rne.scene.toml",
        "assets/scenes/mm_minimal.rne.scene.toml",
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
    rehearsal: InstallRehearsalReport,
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
    flagship_workflows: BTreeMap<String, String>,
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
        release.flagship_workflows.len() == INSTALL_CHECK_IDS.len()
            && INSTALL_CHECK_IDS.iter().all(|id| {
                release
                    .flagship_workflows
                    .get(*id)
                    .is_some_and(|status| status == "passed")
            }),
        "readiness release report must retain all nine passing installed workflows"
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
        "readiness install report must pass all nine canonical checks"
    );
    anyhow::ensure!(
        release.flagship_workflows == install.verdicts(),
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
        flagship_workflows: rehearsal.verdicts(),
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
    let archive_report = build_archive_install_report(&archive, &bundle_dir, &release, &report)?;
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

fn build_archive_install_report(
    archive_path: &Path,
    bundle_dir: &Path,
    release: &ReleaseReport,
    rehearsal: &InstallRehearsalReport,
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
        release.flagship_workflows == rehearsal.verdicts(),
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
    Ok(ArchiveInstallRehearsalReport {
        kind: ARCHIVE_INSTALL_REPORT_KIND.to_string(),
        schema_version: ARCHIVE_INSTALL_REHEARSAL_REPORT_SCHEMA_VERSION,
        archive,
        bundle_root: bundle_name(&release.target),
        release_report,
        checksum_manifest,
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

fn build_native_artifacts(root: &Path, _target: &str) -> anyhow::Result<()> {
    let mut args = vec![
        OsString::from("build"),
        OsString::from("--locked"),
        OsString::from("--release"),
    ];
    for package in RELEASE_BINARY_PACKAGES
        .iter()
        .map(|(package, _)| *package)
        .collect::<BTreeSet<_>>()
    {
        args.push(OsString::from("-p"));
        args.push(OsString::from(package));
    }
    args.push(OsString::from("-p"));
    args.push(OsString::from(RELEASE_PLUGIN_PACKAGE));
    let output = command_output(root, Path::new("cargo"), &args, &[])?;
    print_output(&output);
    ensure_success("cargo build release artifacts", &output)
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
    fs::create_dir_all(output_dir)?;
    let bin_dir = bundle_dir.join("bin");
    let asset_cli = bin_dir.join(native_binary_name("rne-asset", target));
    let compatibility = bin_dir.join(native_binary_name("rne-compatibility", target));
    let physics = bin_dir.join(native_binary_name("rne-physics-conformance", target));
    let scale = bin_dir.join(native_binary_name("rne-scenario-scale", target));
    let hardware_conformance = bin_dir.join(native_binary_name("rne-hardware-conformance", target));
    let hardware_mock = bin_dir.join(native_binary_name("rne-hardware-mock-device", target));

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
            OsString::from("rne.diff_drive.goal.v1"),
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
        check("scenario_replay", scenario_verify),
        check("physics_conformance", physics_passed),
        check("scenario_scale_100", scale_passed),
        check("hardware_adapter", hardware_passed),
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
        ],
        &[],
    ) {
        return false;
    }
    let crate_dir = parent.join(NAME);
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
    let output = Command::new(program).arg("--version").output()?;
    ensure_success(&format!("{program} --version"), &output)?;
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
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
    for name in ["wheel-venv", "controller-authoring"] {
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
        fs::write(output.join("wheel-venv/lib/module.py"), b"temporary").expect("venv file");
        fs::write(
            output.join("controller-authoring/scaffold/target/plugin"),
            b"temporary",
        )
        .expect("controller build");
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
            fs::read(output.path().join("release/external-evidence-intake.toml")).unwrap(),
            fs::read(root.join("release/external-evidence-intake.toml")).unwrap()
        );
        for guide in [
            "EXTERNAL_EVIDENCE_INTAKE.md",
            "EVIDENCE_QUICKSTART.md",
            "FAILURE_CAPSULE.md",
            "PLUGIN_SDK.md",
            "EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md",
            "HARDWARE_ADAPTER_CONFORMANCE.md",
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
            flagship_workflows: workflows,
            members: Vec::new(),
        };
        release.members =
            collect_member_digests(&bundle_dir, &[RELEASE_REPORT, SHA256_MANIFEST]).unwrap();
        write_pretty_json(&release_path, &release).unwrap();
        write_sha256_manifest(&bundle_dir).unwrap();
        let archive_install =
            build_archive_install_report(&archive_path, &bundle_dir, &release, &install).unwrap();
        assert_eq!(
            pretty_json_bytes(&archive_install).unwrap(),
            include_bytes!("../../tests/golden/release/archive-install-rehearsal-v1.json")
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
}
