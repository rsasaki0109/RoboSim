//! Evidence-backed RNE 1.0 readiness audit.
//!
//! The audit never promotes the version by itself. It verifies exact external
//! evidence and emits a deterministic report for an explicitly supplied date.

use super::{
    external_project, failure_capsule, lekiwi_evidence, release_artifacts, release_exit,
    validate_blocker_registry, workspace_root, RELEASE_VERSION,
};
use anyhow::{bail, Context};
use rne_accelerator_contract::{
    AcceleratorManifest, AcceleratorProcessConformanceReport, AcceleratorRuntimeContract,
};
use rne_ai::TaskSpec;
use rne_compatibility_suite::CompatibilityFixtureReport;
use rne_hardware_gateway::{
    conformance::{
        HardwareAdapterConformanceIdentity, HardwareAdapterConformanceReport,
        HardwareAdapterConformanceSubject,
    },
    simulator::{
        conformance::{SimulatorAdapterConformanceIdentity, SimulatorAdapterConformanceReport},
        SimulatorRuntimeManifest,
    },
    GatewayConfig, HardwareGateway,
};
use rne_log::FailureCapsule;
use rne_physics_conformance::ExternalPhysicsBackendConformanceReport;
use rne_plugin::{ControllerPluginConformanceReport, PluginKind, PluginManifest};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::ffi::OsString;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

#[cfg(test)]
use rne_hardware_gateway::simulator::conformance::{
    SimulatorAdapterConformanceCheck, SimulatorAdapterConformanceSubject,
};

pub(crate) const MANIFEST_SCHEMA_VERSION: u32 = 8;
pub(crate) const REPORT_SCHEMA_VERSION: u32 = 1;
const REPORT_KIND: &str = "rne_one_zero_readiness_report";
const DEFAULT_MANIFEST: &str = "release/one-zero-readiness.toml";
const DEFAULT_OUTPUT: &str = "artifacts/release-readiness/report.json";
const DEFAULT_PROMOTION_OUTPUT: &str = "artifacts/release-readiness/promotion-report.json";
const PROMOTION_MANIFEST_ENV: &str = "RNE_ONE_ZERO_READINESS_MANIFEST";
const PROMOTION_AS_OF_ENV: &str = "RNE_ONE_ZERO_READINESS_AS_OF";
const PROMOTION_OUTPUT_ENV: &str = "RNE_ONE_ZERO_READINESS_OUTPUT";
const ATTESTATION_RECEIPT_KIND: &str = "rne_github_attestation_verification";
pub(crate) const ATTESTATION_RECEIPT_SCHEMA_VERSION: u32 = 1;
const MINIMUM_STABILITY_DAYS: u32 = 183;
const MINIMUM_EXTERNAL_PROJECTS: usize = 2;
const MINIMUM_COMPATIBILITY_CHECKS: usize = 36;
pub(crate) const MAX_EVIDENCE_BYTES: u64 = 64 * 1024 * 1024;
const MAX_ADAPTER_ARGUMENTS: usize = 128;
const MAX_ADAPTER_ARGUMENT_BYTES: usize = 4_096;
const ACCELERATOR_ADAPTER_EVIDENCE_FILES: usize = 5;
const PLATFORM_RELEASE_EVIDENCE_FILES: usize = 7;
const CHECK_IDS: [&str; 9] = [
    "stability_window",
    "external_projects",
    "third_party_plugin",
    "external_system",
    "reference_hardware",
    "release_artifacts",
    "historical_compatibility",
    "p0_p1_blockers",
    "support_commitment",
];

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReadinessManifest {
    schema_version: u32,
    release_version: String,
    project_owner: String,
    candidate: CandidateSurface,
    minimum_stability_days: u32,
    minimum_external_projects: usize,
    minimum_compatibility_checks: usize,
    unplanned_breaking_changes: u32,
    blocker_registry: String,
    required_platforms: Vec<ReleasePlatform>,
    support: SupportCommitment,
    #[serde(default)]
    external_project: Vec<ExternalProjectEvidence>,
    #[serde(default)]
    third_party_plugin: Vec<ThirdPartyPluginEvidence>,
    #[serde(default)]
    external_system: Vec<ExternalSystemEvidence>,
    accelerator_adapter: Vec<AcceleratorAdapterEvidence>,
    #[serde(default)]
    platform_release: Vec<PlatformReleaseEvidence>,
    #[serde(default)]
    reference_hardware: Option<EvidenceRef>,
    #[serde(default)]
    compatibility_report: Option<EvidenceRef>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CandidateSurface {
    revision: String,
    tree: String,
    since: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SupportCommitment {
    committed: bool,
    maintainer: String,
    support_period: String,
    policy_url: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvidenceRef {
    path: String,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExternalProjectEvidence {
    id: String,
    owner: String,
    repository: String,
    revision: String,
    first_used_on: String,
    last_verified_on: String,
    author_assistance: bool,
    release_archive: EvidenceRef,
    task_spec: EvidenceRef,
    failure_capsule: EvidenceRef,
    submission_candidate: EvidenceRef,
    stdout_log: EvidenceRef,
    stderr_log: EvidenceRef,
    submission_report: EvidenceRef,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ThirdPartyPluginEvidence {
    id: String,
    owner: String,
    repository: String,
    revision: String,
    release_archive: EvidenceRef,
    library: EvidenceRef,
    manifest: EvidenceRef,
    report: EvidenceRef,
    submission_candidate: EvidenceRef,
    stdout_log: EvidenceRef,
    stderr_log: EvidenceRef,
    submission_report: EvidenceRef,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
enum ExternalSystemKind {
    PhysicsBackend,
    HardwareAdapter,
    SimulatorAdapter,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExternalSystemEvidence {
    id: String,
    owner: String,
    repository: String,
    revision: String,
    kind: ExternalSystemKind,
    subject: EvidenceRef,
    #[serde(default)]
    task_spec: Option<EvidenceRef>,
    #[serde(default)]
    adapter_arguments: Vec<String>,
    #[serde(default)]
    runtime_manifest: Option<EvidenceRef>,
    #[serde(default)]
    runtime_artifacts: Vec<EvidenceRef>,
    report: EvidenceRef,
    #[serde(default)]
    release_archive: Option<EvidenceRef>,
    #[serde(default)]
    submission_candidate: Option<EvidenceRef>,
    #[serde(default)]
    stdout_log: Option<EvidenceRef>,
    #[serde(default)]
    stderr_log: Option<EvidenceRef>,
    #[serde(default)]
    submission_report: Option<EvidenceRef>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AcceleratorAdapterEvidence {
    id: String,
    owner: String,
    repository: String,
    revision: String,
    subject: EvidenceRef,
    task_spec: EvidenceRef,
    accelerator_manifest: EvidenceRef,
    runtime_contract: EvidenceRef,
    #[serde(default)]
    adapter_arguments: Vec<String>,
    report: EvidenceRef,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
enum ReleasePlatform {
    LinuxX86_64,
    WindowsX86_64,
}

impl ReleasePlatform {
    const fn target(self) -> &'static str {
        match self {
            Self::LinuxX86_64 => "x86_64-unknown-linux-gnu",
            Self::WindowsX86_64 => "x86_64-pc-windows-msvc",
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PlatformReleaseEvidence {
    platform: ReleasePlatform,
    revision: String,
    tag: String,
    archive: EvidenceRef,
    attestation: EvidenceRef,
    archive_attestation_verification: EvidenceRef,
    release_report: EvidenceRef,
    checksum_manifest: EvidenceRef,
    install_report: EvidenceRef,
    install_attestation_verification: EvidenceRef,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct AttestationVerificationReceipt {
    kind: String,
    schema_version: u32,
    provider: String,
    repository: String,
    certificate_identity: String,
    source_ref: String,
    source_revision: String,
    signer_revision: String,
    issuer: String,
    predicate_type: String,
    deny_self_hosted_runners: bool,
    artifact_sha256: String,
    attestation_bundle_sha256: String,
    verified_attestations: u32,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ReadinessReport {
    kind: String,
    schema_version: u32,
    manifest_sha256: String,
    release_version: String,
    as_of: String,
    candidate_revision: String,
    candidate_tree: String,
    candidate_since: String,
    calendar_stability_days: u32,
    observed_external_use_days: u32,
    eligible: bool,
    checks: Vec<ReadinessCheck>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ReadinessCheck {
    id: String,
    status: String,
    detail: String,
    evidence_sha256: Vec<String>,
}

impl ReadinessCheck {
    fn new(
        id: &str,
        passed: bool,
        detail: impl Into<String>,
        evidence_sha256: impl IntoIterator<Item = String>,
    ) -> Self {
        let mut evidence_sha256 = evidence_sha256.into_iter().collect::<Vec<_>>();
        evidence_sha256.sort();
        evidence_sha256.dedup();
        Self {
            id: id.to_string(),
            status: if passed { "passed" } else { "not_met" }.to_string(),
            detail: detail.into().chars().take(512).collect(),
            evidence_sha256,
        }
    }
}

#[derive(Debug)]
struct Options {
    manifest: PathBuf,
    output: PathBuf,
    as_of: CivilDate,
    require_eligible: bool,
}

#[derive(Debug, Eq, PartialEq)]
struct PromotionInputs {
    manifest: PathBuf,
    output: PathBuf,
    as_of: CivilDate,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct CivilDate {
    year: i32,
    month: u32,
    day: u32,
}

impl CivilDate {
    fn parse(value: &str) -> anyhow::Result<Self> {
        let bytes = value.as_bytes();
        anyhow::ensure!(
            bytes.len() == 10 && bytes[4] == b'-' && bytes[7] == b'-',
            "date must use YYYY-MM-DD, got {value:?}"
        );
        let year = parse_digits(&bytes[0..4], "year")? as i32;
        let month = parse_digits(&bytes[5..7], "month")?;
        let day = parse_digits(&bytes[8..10], "day")?;
        anyhow::ensure!((2000..=9999).contains(&year), "date year is out of range");
        anyhow::ensure!((1..=12).contains(&month), "date month is out of range");
        let max_day = days_in_month(year, month);
        anyhow::ensure!((1..=max_day).contains(&day), "date day is out of range");
        Ok(Self { year, month, day })
    }

    fn days_since_epoch(self) -> i64 {
        let adjusted_year = self.year - i32::from(self.month <= 2);
        let era = adjusted_year.div_euclid(400);
        let year_of_era = adjusted_year - era * 400;
        let shifted_month = self.month as i32 + if self.month > 2 { -3 } else { 9 };
        let day_of_year = (153 * shifted_month + 2) / 5 + self.day as i32 - 1;
        let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
        i64::from(era * 146_097 + day_of_era - 719_468)
    }

    fn days_until(self, later: Self) -> anyhow::Result<u32> {
        let days = later.days_since_epoch() - self.days_since_epoch();
        anyhow::ensure!(days >= 0, "date range runs backwards");
        u32::try_from(days).context("date range is too large")
    }
}

impl std::fmt::Display for CivilDate {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "{:04}-{:02}-{:02}",
            self.year, self.month, self.day
        )
    }
}

#[derive(Debug)]
struct VerifiedEvidence {
    path: PathBuf,
    sha256: String,
    bytes: Vec<u8>,
}

#[derive(Debug)]
struct AttestationVerificationRequest<'a> {
    artifact_path: &'a Path,
    artifact_sha256: &'a str,
    bundle_path: &'a Path,
    bundle_sha256: &'a str,
    revision: &'a str,
    tag: &'a str,
}

#[derive(Debug, Deserialize)]
struct GhAttestationVerification {
    #[serde(rename = "verificationResult")]
    verification_result: GhVerificationResult,
}

#[derive(Debug, Deserialize)]
struct GhVerificationResult {
    statement: GhStatement,
}

#[derive(Debug, Deserialize)]
struct GhStatement {
    #[serde(rename = "predicateType")]
    predicate_type: String,
    subject: Vec<GhSubject>,
}

#[derive(Debug, Deserialize)]
struct GhSubject {
    digest: GhDigest,
}

#[derive(Debug, Deserialize)]
struct GhDigest {
    sha256: Option<String>,
}

#[derive(Debug)]
struct ProjectUse {
    first: CivilDate,
    last: CivilDate,
    digests: Vec<String>,
}

/// Runs the deterministic, evidence-backed RNE 1.0 readiness audit.
pub(crate) fn release_readiness(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let options = parse_options(args, &root)?;
    let manifest = read_manifest(&options.manifest)?;
    validate_manifest_identity(&root, &manifest)?;
    let report = evaluate(&root, &options.manifest, &manifest, options.as_of)?;
    write_report(&options.output, &report)?;
    println!(
        "RNE 1.0 readiness: eligible={} checks={}/{} report={}",
        report.eligible,
        report
            .checks
            .iter()
            .filter(|check| check.status == "passed")
            .count(),
        report.checks.len(),
        options.output.display()
    );
    anyhow::ensure!(
        !options.require_eligible || report.eligible,
        "RNE 1.0 readiness requirements are not met; inspect {}",
        options.output.display()
    );
    Ok(())
}

/// Validates the committed tracker without claiming that external gates passed.
pub(crate) fn validate_committed_manifest(root: &Path) -> anyhow::Result<()> {
    let path = root.join(DEFAULT_MANIFEST);
    validate_manifest_path(root, &path)
}

/// Validates a readiness manifest's fixed identity without evaluating evidence.
pub(crate) fn validate_manifest_path(root: &Path, path: &Path) -> anyhow::Result<()> {
    let manifest = read_manifest(path)?;
    validate_manifest_identity(root, &manifest)
}

/// Enforces the external-evidence gate before any 1.x release command proceeds.
pub(crate) fn enforce_release_promotion(root: &Path) -> anyhow::Result<()> {
    if !version_requires_one_zero_promotion(RELEASE_VERSION)? {
        return Ok(());
    }
    let inputs = promotion_inputs(
        root,
        RELEASE_VERSION,
        environment_value(PROMOTION_MANIFEST_ENV)?,
        environment_value(PROMOTION_AS_OF_ENV)?,
        environment_value(PROMOTION_OUTPUT_ENV)?,
    )?
    .expect("1.x releases always require promotion inputs");
    let manifest = read_manifest(&inputs.manifest)?;
    validate_manifest_identity(root, &manifest)?;
    let report = evaluate(root, &inputs.manifest, &manifest, inputs.as_of)?;
    write_report(&inputs.output, &report)?;
    anyhow::ensure!(
        report.eligible,
        "release {RELEASE_VERSION} is blocked by the RNE 1.0 readiness gate; inspect {}",
        inputs.output.display()
    );
    println!(
        "RNE 1.0 promotion gate passed: version={RELEASE_VERSION} report={}",
        inputs.output.display()
    );
    Ok(())
}

fn promotion_inputs(
    root: &Path,
    release_version: &str,
    manifest: Option<String>,
    as_of: Option<String>,
    output: Option<String>,
) -> anyhow::Result<Option<PromotionInputs>> {
    if !version_requires_one_zero_promotion(release_version)? {
        return Ok(None);
    }
    let manifest = manifest.with_context(|| {
        format!(
            "release {release_version} requires {PROMOTION_MANIFEST_ENV}=PATH to a complete external evidence pack"
        )
    })?;
    let as_of = as_of.with_context(|| {
        format!(
            "release {release_version} requires {PROMOTION_AS_OF_ENV}=YYYY-MM-DD for deterministic readiness evaluation"
        )
    })?;
    let output = output.unwrap_or_else(|| DEFAULT_PROMOTION_OUTPUT.to_string());
    Ok(Some(PromotionInputs {
        manifest: absolute_from(root, manifest),
        output: absolute_from(root, output),
        as_of: CivilDate::parse(&as_of)?,
    }))
}

fn version_requires_one_zero_promotion(version: &str) -> anyhow::Result<bool> {
    let core = version
        .split(['-', '+'])
        .next()
        .context("release version is empty")?;
    let components = core.split('.').collect::<Vec<_>>();
    anyhow::ensure!(
        components.len() == 3
            && components.iter().all(|component| !component.is_empty()
                && component.bytes().all(|byte| byte.is_ascii_digit())),
        "release version must start with numeric MAJOR.MINOR.PATCH, got {version:?}"
    );
    let major = components[0]
        .parse::<u64>()
        .context("release major version is out of range")?;
    Ok(major >= 1)
}

fn environment_value(name: &str) -> anyhow::Result<Option<String>> {
    match std::env::var(name) {
        Ok(value) => Ok(Some(value)),
        Err(std::env::VarError::NotPresent) => Ok(None),
        Err(std::env::VarError::NotUnicode(_)) => {
            bail!("environment variable {name} is not valid Unicode")
        }
    }
}

fn parse_options(args: &mut impl Iterator<Item = String>, root: &Path) -> anyhow::Result<Options> {
    let mut manifest = root.join(DEFAULT_MANIFEST);
    let mut output = root.join(DEFAULT_OUTPUT);
    let mut as_of = None;
    let mut require_eligible = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--manifest" => manifest = absolute_from(root, next_value(args, "--manifest")?),
            "--output" => output = absolute_from(root, next_value(args, "--output")?),
            "--as-of" => as_of = Some(CivilDate::parse(&next_value(args, "--as-of")?)?),
            "--require-eligible" => require_eligible = true,
            "--help" | "-h" => {
                println!(
                    "release-readiness --as-of YYYY-MM-DD [--manifest PATH] [--output PATH] [--require-eligible]"
                );
                return Err(anyhow::anyhow!("help requested"));
            }
            other => bail!("unknown release-readiness argument: {other}"),
        }
    }
    Ok(Options {
        manifest,
        output,
        as_of: as_of.context("release-readiness requires --as-of YYYY-MM-DD")?,
        require_eligible,
    })
}

fn next_value(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn absolute_from(root: &Path, path: String) -> PathBuf {
    let path = PathBuf::from(path);
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn read_manifest(path: &Path) -> anyhow::Result<ReadinessManifest> {
    let text = fs::read_to_string(path)
        .with_context(|| format!("read readiness manifest {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("parse readiness manifest {}", path.display()))
}

fn validate_manifest_identity(root: &Path, manifest: &ReadinessManifest) -> anyhow::Result<()> {
    anyhow::ensure!(
        manifest.schema_version == MANIFEST_SCHEMA_VERSION,
        "1.0 readiness manifest schema must be {MANIFEST_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        manifest.release_version == RELEASE_VERSION,
        "1.0 readiness manifest release must remain {RELEASE_VERSION} until every gate passes"
    );
    validate_identifier("project_owner", &manifest.project_owner)?;
    anyhow::ensure!(
        manifest.minimum_stability_days == MINIMUM_STABILITY_DAYS,
        "1.0 readiness must require exactly {MINIMUM_STABILITY_DAYS} stability days"
    );
    anyhow::ensure!(
        manifest.minimum_external_projects == MINIMUM_EXTERNAL_PROJECTS,
        "1.0 readiness must require exactly {MINIMUM_EXTERNAL_PROJECTS} external projects"
    );
    anyhow::ensure!(
        manifest.minimum_compatibility_checks >= MINIMUM_COMPATIBILITY_CHECKS,
        "1.0 readiness cannot cover fewer than {MINIMUM_COMPATIBILITY_CHECKS} compatibility checks"
    );
    anyhow::ensure!(
        manifest.blocker_registry == "release/blockers.toml",
        "1.0 readiness blocker registry must remain release/blockers.toml"
    );
    anyhow::ensure!(
        manifest.required_platforms
            == [ReleasePlatform::LinuxX86_64, ReleasePlatform::WindowsX86_64],
        "1.0 readiness requires Linux x86-64 then Windows x86-64"
    );
    anyhow::ensure!(
        is_git_object_id(&manifest.candidate.revision)
            && is_git_object_id(&manifest.candidate.tree),
        "candidate revision and tree must be lowercase 40-character Git IDs"
    );
    let candidate_since = CivilDate::parse(&manifest.candidate.since)?;
    let baseline =
        fs::read_to_string(root.join("release/rust-api-baseline.toml"))?.parse::<toml::Value>()?;
    anyhow::ensure!(
        baseline
            .get("baseline_revision")
            .and_then(toml::Value::as_str)
            == Some(manifest.candidate.revision.as_str())
            && baseline.get("baseline_tree").and_then(toml::Value::as_str)
                == Some(manifest.candidate.tree.as_str()),
        "1.0 candidate must equal the immutable Rust API baseline"
    );
    let actual_tree = git_output(
        root,
        &["show", "-s", "--format=%T", &manifest.candidate.revision],
    )?;
    anyhow::ensure!(
        actual_tree == manifest.candidate.tree,
        "1.0 candidate tree does not match its revision"
    );
    let commit_date = git_output(
        root,
        &["show", "-s", "--format=%cs", &manifest.candidate.revision],
    )?;
    anyhow::ensure!(
        commit_date == candidate_since.to_string(),
        "1.0 candidate since date must equal the baseline commit date"
    );
    ensure_git_ancestor(root, &manifest.candidate.revision)?;

    validate_support_shape(&manifest.support)?;
    validate_unique_manifest_entries(manifest)?;
    Ok(())
}

fn validate_support_shape(support: &SupportCommitment) -> anyhow::Result<()> {
    for (field, value) in [
        ("support.maintainer", support.maintainer.as_str()),
        ("support.support_period", support.support_period.as_str()),
        ("support.policy_url", support.policy_url.as_str()),
    ] {
        anyhow::ensure!(
            !value.chars().any(char::is_control),
            "{field} contains control characters"
        );
    }

    if !support.committed {
        anyhow::ensure!(
            support.maintainer.is_empty()
                && support.support_period.is_empty()
                && support.policy_url.is_empty(),
            "uncommitted support must not contain maintainer, period, or policy claims"
        );
        return Ok(());
    }

    anyhow::ensure!(
        !support.maintainer.trim().is_empty()
            && support.maintainer == support.maintainer.trim()
            && support.maintainer.len() <= 128,
        "committed support maintainer must be a canonical non-empty value of at most 128 bytes"
    );
    anyhow::ensure!(
        !support.support_period.trim().is_empty()
            && support.support_period == support.support_period.trim()
            && support.support_period.len() <= 256,
        "committed support period must be a canonical non-empty value of at most 256 bytes"
    );
    anyhow::ensure!(
        support.policy_url.len() <= 2_048 && is_https_url(&support.policy_url),
        "committed support policy must be a bounded HTTPS URL"
    );
    Ok(())
}

fn validate_unique_manifest_entries(manifest: &ReadinessManifest) -> anyhow::Result<()> {
    unique_ids(
        "external project",
        manifest
            .external_project
            .iter()
            .map(|entry| entry.id.as_str()),
    )?;
    unique_ids(
        "third-party plugin",
        manifest
            .third_party_plugin
            .iter()
            .map(|entry| entry.id.as_str()),
    )?;
    unique_ids(
        "external system",
        manifest
            .external_system
            .iter()
            .map(|entry| entry.id.as_str()),
    )?;
    unique_ids(
        "accelerator adapter",
        manifest
            .accelerator_adapter
            .iter()
            .map(|entry| entry.id.as_str()),
    )?;
    let platforms = manifest
        .platform_release
        .iter()
        .map(|entry| entry.platform)
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        platforms.len() == manifest.platform_release.len(),
        "platform release evidence contains a duplicate platform"
    );
    Ok(())
}

fn unique_ids<'a>(label: &str, ids: impl IntoIterator<Item = &'a str>) -> anyhow::Result<()> {
    let mut unique = BTreeSet::new();
    for id in ids {
        validate_identifier(label, id)?;
        anyhow::ensure!(unique.insert(id), "duplicate {label} id {id:?}");
    }
    Ok(())
}

fn evaluate(
    root: &Path,
    manifest_path: &Path,
    manifest: &ReadinessManifest,
    as_of: CivilDate,
) -> anyhow::Result<ReadinessReport> {
    let evidence_root = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let manifest_sha256 = normalized_text_sha256(&fs::read_to_string(manifest_path)?)?;
    let candidate_since = CivilDate::parse(&manifest.candidate.since)?;
    let calendar_stability_days = candidate_since.days_until(as_of)?;
    let projects = verify_external_projects(evidence_root, manifest, candidate_since, as_of)?;
    let project_digests = projects
        .iter()
        .flat_map(|project| project.digests.iter().cloned())
        .collect::<Vec<_>>();
    let observed_external_use_days = observed_use_days(&projects)?;
    let stability_passed = calendar_stability_days >= manifest.minimum_stability_days
        && observed_external_use_days >= manifest.minimum_stability_days
        && manifest.unplanned_breaking_changes == 0;
    let external_projects_passed = projects.len() >= manifest.minimum_external_projects;

    let plugin_digests = verify_third_party_plugins(evidence_root, manifest)?;
    let system_digests = verify_external_systems(evidence_root, manifest)?;
    let accelerator_adapter_digests = verify_accelerator_adapters(evidence_root, manifest)?;
    let hardware_digests = verify_reference_hardware(evidence_root, manifest)?;
    let release_digests = verify_platform_releases(root, evidence_root, manifest)?;
    let compatibility_digests = verify_compatibility(root, evidence_root, manifest)?;

    let blocker_path = root.join(&manifest.blocker_registry);
    let blocker_text = fs::read_to_string(&blocker_path)?;
    let blocker_registry = blocker_text.parse::<toml::Value>()?;
    validate_blocker_registry(&blocker_registry)?;

    let support_passed = manifest.support.committed
        && !manifest.support.maintainer.trim().is_empty()
        && !manifest.support.support_period.trim().is_empty()
        && is_https_url(&manifest.support.policy_url);
    let checks = vec![
        ReadinessCheck::new(
            CHECK_IDS[0],
            stability_passed,
            format!(
                "calendar={calendar_stability_days}/{}, observed_external_use={observed_external_use_days}/{}, unplanned_breaking_changes={}",
                manifest.minimum_stability_days,
                manifest.minimum_stability_days,
                manifest.unplanned_breaking_changes
            ),
            project_digests.clone(),
        ),
        ReadinessCheck::new(
            CHECK_IDS[1],
            external_projects_passed,
            format!(
                "verified_external_projects={}/{}",
                projects.len(),
                manifest.minimum_external_projects
            ),
            project_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[2],
            !manifest.third_party_plugin.is_empty(),
            format!(
                "verified_third_party_plugins={}",
                manifest.third_party_plugin.len()
            ),
            plugin_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[3],
            !manifest.external_system.is_empty(),
            format!(
                "verified_external_systems={} audited_accelerator_adapters={}",
                manifest.external_system.len(),
                accelerator_adapter_digests.len() / ACCELERATOR_ADAPTER_EVIDENCE_FILES
            ),
            system_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[4],
            !hardware_digests.is_empty(),
            format!("verified_reference_hardware_runs={}", hardware_digests.len()),
            hardware_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[5],
            release_digests.len()
                == manifest.required_platforms.len() * PLATFORM_RELEASE_EVIDENCE_FILES,
            format!(
                "verified_platform_releases={}/{}",
                release_digests.len() / PLATFORM_RELEASE_EVIDENCE_FILES,
                manifest.required_platforms.len()
            ),
            release_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[6],
            !compatibility_digests.is_empty(),
            format!(
                "minimum_compatibility_checks={}",
                manifest.minimum_compatibility_checks
            ),
            compatibility_digests,
        ),
        ReadinessCheck::new(
            CHECK_IDS[7],
            true,
            "release blocker registry has zero open P0/P1 entries",
            [normalized_text_sha256(&blocker_text)?],
        ),
        ReadinessCheck::new(
            CHECK_IDS[8],
            support_passed,
            format!(
                "committed={} maintainer={} support_period={}",
                manifest.support.committed,
                display_or_missing(&manifest.support.maintainer),
                display_or_missing(&manifest.support.support_period)
            ),
            Vec::new(),
        ),
    ];
    anyhow::ensure!(
        checks
            .iter()
            .map(|check| check.id.as_str())
            .eq(CHECK_IDS.into_iter()),
        "readiness check registry drifted"
    );
    let eligible = checks.iter().all(|check| check.status == "passed");
    Ok(ReadinessReport {
        kind: REPORT_KIND.to_string(),
        schema_version: REPORT_SCHEMA_VERSION,
        manifest_sha256,
        release_version: RELEASE_VERSION.to_string(),
        as_of: as_of.to_string(),
        candidate_revision: manifest.candidate.revision.clone(),
        candidate_tree: manifest.candidate.tree.clone(),
        candidate_since: candidate_since.to_string(),
        calendar_stability_days,
        observed_external_use_days,
        eligible,
        checks,
    })
}

fn verify_external_projects(
    evidence_root: &Path,
    manifest: &ReadinessManifest,
    candidate_since: CivilDate,
    as_of: CivilDate,
) -> anyhow::Result<Vec<ProjectUse>> {
    let mut repositories = BTreeSet::new();
    let mut capsules = BTreeSet::new();
    let mut projects = Vec::new();
    for entry in &manifest.external_project {
        validate_identifier("external project id", &entry.id)?;
        validate_external_owner(
            &manifest.project_owner,
            &entry.owner,
            &entry.repository,
            "external project",
        )?;
        validate_external_revision("external project", &entry.id, &entry.revision)?;
        anyhow::ensure!(
            !entry.author_assistance,
            "external project {} required repository-author assistance",
            entry.id
        );
        anyhow::ensure!(
            repositories.insert(entry.repository.as_str()),
            "external project repository is duplicated: {}",
            entry.repository
        );
        let first = CivilDate::parse(&entry.first_used_on)?;
        let last = CivilDate::parse(&entry.last_verified_on)?;
        candidate_since.days_until(first)?;
        first.days_until(last)?;
        last.days_until(as_of)?;

        let release_archive = verify_evidence(evidence_root, &entry.release_archive)?;
        let task = verify_evidence(evidence_root, &entry.task_spec)?;
        let task_spec: TaskSpec = serde_json::from_slice(&task.bytes)
            .with_context(|| format!("parse external project {} TaskSpec", entry.id))?;
        task_spec
            .validate()
            .with_context(|| format!("validate external project {} TaskSpec", entry.id))?;
        let capsule = verify_evidence(evidence_root, &entry.failure_capsule)?;
        anyhow::ensure!(
            capsule
                .path
                .file_name()
                .is_some_and(|name| name == "capsule.json"),
            "external project {} Failure Capsule path must end in capsule.json",
            entry.id
        );
        let capsule_manifest: FailureCapsule = serde_json::from_slice(&capsule.bytes)
            .with_context(|| format!("parse external project {} Failure Capsule", entry.id))?;
        capsule_manifest
            .validate()
            .with_context(|| format!("validate external project {} Failure Capsule", entry.id))?;
        failure_capsule::verify_directory(
            capsule
                .path
                .parent()
                .context("Failure Capsule has no parent")?,
        )?;
        let submission_candidate = verify_evidence(evidence_root, &entry.submission_candidate)?;
        let stdout_log = verify_evidence(evidence_root, &entry.stdout_log)?;
        let stderr_log = verify_evidence(evidence_root, &entry.stderr_log)?;
        let submission_report = verify_evidence(evidence_root, &entry.submission_report)?;
        external_project::validate_staged_submission_report(
            &submission_report.bytes,
            external_project::StagedSubmission {
                owner: &entry.owner,
                repository: &entry.repository,
                revision: &entry.revision,
                first_used_on: &entry.first_used_on,
                last_verified_on: &entry.last_verified_on,
                release_archive: &release_archive.path,
                task_spec: &task.path,
                failure_capsule: &capsule.path,
                submission_candidate: &submission_candidate.path,
                stdout_log: &stdout_log.path,
                stderr_log: &stderr_log.path,
            },
        )
        .with_context(|| format!("validate external project {} submission report", entry.id))?;
        anyhow::ensure!(
            capsules.insert(capsule.sha256.clone()),
            "external projects must retain distinct Failure Capsules"
        );
        projects.push(ProjectUse {
            first,
            last,
            digests: vec![
                release_archive.sha256,
                task.sha256,
                capsule.sha256,
                submission_candidate.sha256,
                stdout_log.sha256,
                stderr_log.sha256,
                submission_report.sha256,
            ],
        });
    }
    Ok(projects)
}

fn verify_third_party_plugins(
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let mut ids = BTreeSet::new();
    let mut repositories = BTreeSet::new();
    let mut subjects = BTreeSet::<String>::new();
    let mut digests = Vec::new();
    for entry in &manifest.third_party_plugin {
        validate_identifier("third-party plugin id", &entry.id)?;
        anyhow::ensure!(
            ids.insert(entry.id.as_str()),
            "third-party plugin id is duplicated: {}",
            entry.id
        );
        validate_external_owner(
            &manifest.project_owner,
            &entry.owner,
            &entry.repository,
            "third-party plugin",
        )?;
        validate_external_revision("third-party plugin", &entry.id, &entry.revision)?;
        anyhow::ensure!(
            repositories.insert(entry.repository.as_str()),
            "third-party plugin repository is duplicated: {}",
            entry.repository
        );
        let release_archive = verify_evidence(evidence_root, &entry.release_archive)?;
        let library = verify_evidence(evidence_root, &entry.library)?;
        let manifest_evidence = verify_evidence(evidence_root, &entry.manifest)?;
        let report_evidence = verify_evidence(evidence_root, &entry.report)?;
        let submission_candidate = verify_evidence(evidence_root, &entry.submission_candidate)?;
        let stdout_log = verify_evidence(evidence_root, &entry.stdout_log)?;
        let stderr_log = verify_evidence(evidence_root, &entry.stderr_log)?;
        let submission_report = verify_evidence(evidence_root, &entry.submission_report)?;
        let plugin_manifest: PluginManifest = serde_json::from_slice(&manifest_evidence.bytes)
            .with_context(|| format!("parse third-party plugin {} manifest", entry.id))?;
        plugin_manifest
            .validate()
            .with_context(|| format!("validate third-party plugin {} manifest", entry.id))?;
        anyhow::ensure!(
            plugin_manifest.kind == PluginKind::Controller,
            "third-party plugin {} is not a controller plugin",
            entry.id
        );
        let report: ControllerPluginConformanceReport =
            serde_json::from_slice(&report_evidence.bytes)
                .with_context(|| format!("parse third-party plugin {} report", entry.id))?;
        report
            .validate()
            .with_context(|| format!("validate third-party plugin {} report", entry.id))?;
        anyhow::ensure!(
            report.passed(),
            "third-party plugin {} did not pass conformance",
            entry.id
        );
        verify_unprefixed_subject(
            "controller plugin library",
            &library,
            &report.subject.library_file,
            &report.subject.library_sha256,
            Some(report.subject.library_size_bytes),
        )?;
        verify_unprefixed_subject(
            "controller plugin manifest",
            &manifest_evidence,
            &report.subject.manifest_file,
            &report.subject.manifest_sha256,
            None,
        )?;
        let controller = report
            .controller
            .as_ref()
            .context("passing controller plugin report omitted its identity")?;
        anyhow::ensure!(
            controller.name == plugin_manifest.name,
            "third-party plugin {} manifest and negotiated names differ",
            entry.id
        );
        anyhow::ensure!(
            subjects.insert(library.sha256.clone()),
            "third-party plugin subject is duplicated: {}",
            entry.library.path
        );
        super::external_plugin::validate_staged_submission_report(
            &submission_report.bytes,
            super::external_plugin::StagedSubmission {
                owner: &entry.owner,
                repository: &entry.repository,
                revision: &entry.revision,
                release_archive: &release_archive.path,
                library: &library.path,
                manifest: &manifest_evidence.path,
                conformance_report: &report_evidence.path,
                submission_candidate: &submission_candidate.path,
                stdout_log: &stdout_log.path,
                stderr_log: &stderr_log.path,
            },
        )?;
        digests.extend([
            release_archive.sha256,
            library.sha256,
            manifest_evidence.sha256,
            report_evidence.sha256,
            submission_candidate.sha256,
            stdout_log.sha256,
            stderr_log.sha256,
            submission_report.sha256,
        ]);
    }
    Ok(digests)
}

fn verify_external_systems(
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let mut ids = BTreeSet::new();
    let mut repositories = BTreeSet::new();
    let mut subjects = BTreeSet::<String>::new();
    let mut digests = Vec::new();
    for entry in &manifest.external_system {
        validate_identifier("external system id", &entry.id)?;
        anyhow::ensure!(
            ids.insert(entry.id.as_str()),
            "external system id is duplicated: {}",
            entry.id
        );
        validate_external_owner(
            &manifest.project_owner,
            &entry.owner,
            &entry.repository,
            "external system",
        )?;
        validate_external_revision("external system", &entry.id, &entry.revision)?;
        anyhow::ensure!(
            repositories.insert(entry.repository.as_str()),
            "external system repository is duplicated: {}",
            entry.repository
        );
        let subject = verify_evidence(evidence_root, &entry.subject)?;
        let report_evidence = verify_evidence(evidence_root, &entry.report)?;
        match entry.kind {
            ExternalSystemKind::PhysicsBackend => {
                anyhow::ensure!(
                    entry.task_spec.is_none()
                        && entry.adapter_arguments.is_empty()
                        && entry.runtime_manifest.is_none()
                        && entry.runtime_artifacts.is_empty()
                        && entry.release_archive.is_none()
                        && entry.submission_candidate.is_none()
                        && entry.stdout_log.is_none()
                        && entry.stderr_log.is_none()
                        && entry.submission_report.is_none(),
                    "external physics backend {} must not declare adapter-only evidence",
                    entry.id
                );
                let report: ExternalPhysicsBackendConformanceReport =
                    serde_json::from_slice(&report_evidence.bytes).with_context(|| {
                        format!("parse external physics backend {} report", entry.id)
                    })?;
                report.validate().with_context(|| {
                    format!("validate external physics backend {} report", entry.id)
                })?;
                anyhow::ensure!(
                    report.passed(),
                    "external physics backend {} did not pass conformance",
                    entry.id
                );
                verify_prefixed_subject(
                    "external physics backend subject",
                    &subject,
                    &report.subject.label,
                    &report.subject.sha256,
                )?;
                digests.extend([subject.sha256.clone(), report_evidence.sha256.clone()]);
            }
            ExternalSystemKind::HardwareAdapter => {
                anyhow::ensure!(
                    entry.runtime_manifest.is_none()
                        && entry.runtime_artifacts.is_empty()
                        && entry.release_archive.is_none()
                        && entry.submission_candidate.is_none()
                        && entry.stdout_log.is_none()
                        && entry.stderr_log.is_none()
                        && entry.submission_report.is_none(),
                    "external hardware adapter {} must not declare simulator runtime evidence",
                    entry.id
                );
                let task_reference = entry.task_spec.as_ref().with_context(|| {
                    format!(
                        "external hardware adapter {} omitted its exact TaskSpec",
                        entry.id
                    )
                })?;
                let task = verify_evidence(evidence_root, task_reference)?;
                let task_spec: TaskSpec =
                    serde_json::from_slice(&task.bytes).with_context(|| {
                        format!("parse external hardware adapter {} TaskSpec", entry.id)
                    })?;
                task_spec.validate().with_context(|| {
                    format!("validate external hardware adapter {} TaskSpec", entry.id)
                })?;
                let report: HardwareAdapterConformanceReport =
                    serde_json::from_slice(&report_evidence.bytes).with_context(|| {
                        format!("parse external hardware adapter {} report", entry.id)
                    })?;
                report.validate().with_context(|| {
                    format!("validate external hardware adapter {} report", entry.id)
                })?;
                anyhow::ensure!(
                    report.passed(),
                    "external hardware adapter {} did not pass conformance",
                    entry.id
                );
                verify_unprefixed_subject(
                    "external hardware adapter subject",
                    &subject,
                    &report.subject.adapter_file,
                    &report.subject.adapter_sha256,
                    Some(report.subject.adapter_size_bytes),
                )?;
                verify_unprefixed_subject(
                    "external hardware adapter TaskSpec",
                    &task,
                    &report.subject.task_file,
                    &report.subject.task_sha256,
                    None,
                )?;
                verify_adapter_arguments(&entry.adapter_arguments, &report.subject)?;
                let identity = report
                    .adapter
                    .as_ref()
                    .context("passing hardware adapter report omitted its identity")?;
                verify_hardware_task_identity(&entry.id, &task_spec, identity)?;
                digests.extend([
                    subject.sha256.clone(),
                    task.sha256,
                    report_evidence.sha256.clone(),
                ]);
            }
            ExternalSystemKind::SimulatorAdapter => {
                let task_reference = entry.task_spec.as_ref().with_context(|| {
                    format!(
                        "external simulator adapter {} omitted its exact TaskSpec",
                        entry.id
                    )
                })?;
                let runtime_reference = entry.runtime_manifest.as_ref().with_context(|| {
                    format!(
                        "external simulator adapter {} omitted its runtime manifest",
                        entry.id
                    )
                })?;
                let release_reference = entry.release_archive.as_ref().with_context(|| {
                    format!(
                        "external simulator adapter {} omitted its RNE release archive",
                        entry.id
                    )
                })?;
                let candidate_reference =
                    entry.submission_candidate.as_ref().with_context(|| {
                        format!(
                            "external simulator adapter {} omitted its submission candidate",
                            entry.id
                        )
                    })?;
                let stdout_reference = entry.stdout_log.as_ref().with_context(|| {
                    format!(
                        "external simulator adapter {} omitted its stdout log",
                        entry.id
                    )
                })?;
                let stderr_reference = entry.stderr_log.as_ref().with_context(|| {
                    format!(
                        "external simulator adapter {} omitted its stderr log",
                        entry.id
                    )
                })?;
                let submission_report_reference =
                    entry.submission_report.as_ref().with_context(|| {
                        format!(
                            "external simulator adapter {} omitted its maintainer report",
                            entry.id
                        )
                    })?;
                let task = verify_evidence(evidence_root, task_reference)?;
                let runtime_evidence = verify_evidence(evidence_root, runtime_reference)?;
                let release_archive = verify_evidence(evidence_root, release_reference)?;
                let submission_candidate = verify_evidence(evidence_root, candidate_reference)?;
                let stdout_log = verify_evidence(evidence_root, stdout_reference)?;
                let stderr_log = verify_evidence(evidence_root, stderr_reference)?;
                let submission_report =
                    verify_evidence(evidence_root, submission_report_reference)?;
                let task_spec: TaskSpec =
                    serde_json::from_slice(&task.bytes).with_context(|| {
                        format!("parse external simulator adapter {} TaskSpec", entry.id)
                    })?;
                task_spec.validate().with_context(|| {
                    format!("validate external simulator adapter {} TaskSpec", entry.id)
                })?;
                let runtime: SimulatorRuntimeManifest =
                    serde_json::from_slice(&runtime_evidence.bytes).with_context(|| {
                        format!(
                            "parse external simulator adapter {} runtime manifest",
                            entry.id
                        )
                    })?;
                runtime.validate().with_context(|| {
                    format!(
                        "validate external simulator adapter {} runtime manifest",
                        entry.id
                    )
                })?;
                let report: SimulatorAdapterConformanceReport =
                    serde_json::from_slice(&report_evidence.bytes).with_context(|| {
                        format!("parse external simulator adapter {} report", entry.id)
                    })?;
                report.validate().with_context(|| {
                    format!("validate external simulator adapter {} report", entry.id)
                })?;
                anyhow::ensure!(
                    report.passed(),
                    "external simulator adapter {} did not pass conformance",
                    entry.id
                );
                verify_unprefixed_subject(
                    "external simulator adapter subject",
                    &subject,
                    &report.subject.adapter_file,
                    &report.subject.adapter_sha256,
                    Some(report.subject.adapter_size_bytes),
                )?;
                verify_unprefixed_subject(
                    "external simulator adapter TaskSpec",
                    &task,
                    &report.subject.task_file,
                    &report.subject.task_sha256,
                    None,
                )?;
                verify_unprefixed_subject(
                    "external simulator runtime manifest",
                    &runtime_evidence,
                    &report.subject.runtime_manifest_file,
                    &report.subject.runtime_manifest_sha256,
                    Some(report.subject.runtime_manifest_size_bytes),
                )?;
                verify_normalized_arguments(
                    "external simulator adapter",
                    &entry.adapter_arguments,
                    report.subject.argument_count,
                    &report.subject.arguments_sha256,
                )?;
                let identity = report
                    .adapter
                    .as_ref()
                    .context("passing simulator adapter report omitted its identity")?;
                verify_simulator_task_identity(&entry.id, &task_spec, &runtime, identity)?;
                anyhow::ensure!(
                    entry.runtime_artifacts.len() == runtime.artifacts.len(),
                    "external simulator adapter {} retained the wrong runtime artifact count",
                    entry.id
                );
                anyhow::ensure!(
                    report.subject.runtime_artifacts == runtime.artifacts,
                    "external simulator adapter {} report and runtime manifest artifacts differ",
                    entry.id
                );
                let mut retained = Vec::with_capacity(entry.runtime_artifacts.len());
                let mut retained_paths = Vec::with_capacity(entry.runtime_artifacts.len());
                for (reference, artifact) in entry.runtime_artifacts.iter().zip(&runtime.artifacts)
                {
                    let evidence = verify_evidence(evidence_root, reference)?;
                    verify_unprefixed_subject(
                        "external simulator runtime artifact",
                        &evidence,
                        &artifact.file,
                        &artifact.sha256,
                        Some(artifact.size_bytes),
                    )?;
                    retained.push(evidence.sha256);
                    retained_paths.push(evidence.path);
                }
                super::external_simulator::validate_staged_submission_report(
                    &submission_report.bytes,
                    super::external_simulator::StagedSubmission {
                        owner: &entry.owner,
                        repository: &entry.repository,
                        revision: &entry.revision,
                        release_archive: &release_archive.path,
                        adapter: &subject.path,
                        task_spec: &task.path,
                        runtime_manifest: &runtime_evidence.path,
                        runtime_artifacts: &retained_paths,
                        conformance_report: &report_evidence.path,
                        submission_candidate: &submission_candidate.path,
                        stdout_log: &stdout_log.path,
                        stderr_log: &stderr_log.path,
                        adapter_arguments: &entry.adapter_arguments,
                    },
                )?;
                digests.extend([
                    subject.sha256.clone(),
                    task.sha256,
                    runtime_evidence.sha256,
                    report_evidence.sha256.clone(),
                    release_archive.sha256,
                    submission_candidate.sha256,
                    stdout_log.sha256,
                    stderr_log.sha256,
                    submission_report.sha256,
                ]);
                digests.extend(retained);
            }
        }
        anyhow::ensure!(
            subjects.insert(subject.sha256.clone()),
            "external system subject is duplicated: {}",
            entry.subject.path
        );
    }
    Ok(digests)
}

fn verify_accelerator_adapters(
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let mut repositories = BTreeSet::new();
    let mut subjects = BTreeSet::new();
    let mut digests = Vec::new();
    for entry in &manifest.accelerator_adapter {
        validate_identifier("accelerator adapter id", &entry.id)?;
        validate_external_owner(
            &manifest.project_owner,
            &entry.owner,
            &entry.repository,
            "accelerator adapter",
        )?;
        validate_external_revision("accelerator adapter", &entry.id, &entry.revision)?;
        anyhow::ensure!(
            repositories.insert(entry.repository.as_str()),
            "accelerator adapter repository is duplicated: {}",
            entry.repository
        );

        let subject = verify_evidence(evidence_root, &entry.subject)?;
        let task = verify_evidence(evidence_root, &entry.task_spec)?;
        let manifest_evidence = verify_evidence(evidence_root, &entry.accelerator_manifest)?;
        let runtime_evidence = verify_evidence(evidence_root, &entry.runtime_contract)?;
        let report_evidence = verify_evidence(evidence_root, &entry.report)?;

        let task_spec: TaskSpec = serde_json::from_slice(&task.bytes)
            .with_context(|| format!("parse accelerator adapter {} TaskSpec", entry.id))?;
        task_spec
            .validate()
            .with_context(|| format!("validate accelerator adapter {} TaskSpec", entry.id))?;
        let accelerator_manifest: AcceleratorManifest = toml::from_str(
            std::str::from_utf8(&manifest_evidence.bytes).with_context(|| {
                format!("accelerator adapter {} manifest is not UTF-8", entry.id)
            })?,
        )
        .with_context(|| format!("parse accelerator adapter {} manifest", entry.id))?;
        accelerator_manifest
            .validate()
            .with_context(|| format!("validate accelerator adapter {} manifest", entry.id))?;
        let runtime: AcceleratorRuntimeContract = toml::from_str(
            std::str::from_utf8(&runtime_evidence.bytes).with_context(|| {
                format!(
                    "accelerator adapter {} runtime contract is not UTF-8",
                    entry.id
                )
            })?,
        )
        .with_context(|| format!("parse accelerator adapter {} runtime contract", entry.id))?;
        runtime.validate().with_context(|| {
            format!("validate accelerator adapter {} runtime contract", entry.id)
        })?;
        let report: AcceleratorProcessConformanceReport =
            serde_json::from_slice(&report_evidence.bytes).with_context(|| {
                format!("parse accelerator adapter {} process report", entry.id)
            })?;
        report
            .validate_against(&accelerator_manifest, &runtime, &task_spec)
            .with_context(|| {
                format!(
                    "bind accelerator adapter {} report to retained contracts",
                    entry.id
                )
            })?;

        verify_unprefixed_subject(
            "accelerator adapter subject",
            &subject,
            &report.subject.adapter_file,
            &report.subject.adapter_sha256,
            Some(report.subject.adapter_size_bytes),
        )?;
        verify_unprefixed_subject(
            "accelerator adapter TaskSpec",
            &task,
            &report.subject.task_file,
            &report.subject.task_sha256,
            None,
        )?;
        verify_unprefixed_subject(
            "accelerator adapter manifest",
            &manifest_evidence,
            &report.subject.manifest_file,
            &report.subject.manifest_sha256,
            None,
        )?;
        verify_unprefixed_subject(
            "accelerator adapter runtime contract",
            &runtime_evidence,
            &report.subject.runtime_file,
            &report.subject.runtime_sha256,
            None,
        )?;
        verify_normalized_arguments(
            "accelerator adapter",
            &entry.adapter_arguments,
            report.subject.argument_count,
            &report.subject.arguments_sha256,
        )?;
        anyhow::ensure!(
            subjects.insert(subject.sha256.clone()),
            "accelerator adapter subject is duplicated: {}",
            entry.subject.path
        );
        digests.extend([
            subject.sha256,
            task.sha256,
            manifest_evidence.sha256,
            runtime_evidence.sha256,
            report_evidence.sha256,
        ]);
    }
    Ok(digests)
}

fn verify_unprefixed_subject(
    label: &str,
    evidence: &VerifiedEvidence,
    expected_file: &str,
    expected_sha256: &str,
    expected_size_bytes: Option<u64>,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        expected_sha256 == evidence_sha256_hex(evidence)?,
        "{label} SHA-256 does not match the conformance report"
    );
    anyhow::ensure!(
        expected_file == evidence_file_name(evidence)?,
        "{label} file name does not match the conformance report"
    );
    if let Some(expected_size_bytes) = expected_size_bytes {
        anyhow::ensure!(
            expected_size_bytes == u64::try_from(evidence.bytes.len())?,
            "{label} size does not match the conformance report"
        );
    }
    Ok(())
}

fn verify_prefixed_subject(
    label: &str,
    evidence: &VerifiedEvidence,
    expected_file: &str,
    expected_sha256: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        expected_sha256 == evidence.sha256,
        "{label} SHA-256 does not match the conformance report"
    );
    anyhow::ensure!(
        expected_file == evidence_file_name(evidence)?,
        "{label} file name does not match the conformance report"
    );
    Ok(())
}

fn verify_adapter_arguments(
    arguments: &[String],
    subject: &HardwareAdapterConformanceSubject,
) -> anyhow::Result<()> {
    verify_normalized_arguments(
        "external hardware adapter",
        arguments,
        subject.argument_count,
        &subject.arguments_sha256,
    )
}

fn verify_normalized_arguments(
    label: &str,
    arguments: &[String],
    expected_count: usize,
    expected_sha256: &str,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        arguments.len() <= MAX_ADAPTER_ARGUMENTS,
        "{label} launch contract exceeds {MAX_ADAPTER_ARGUMENTS} arguments"
    );
    for (index, argument) in arguments.iter().enumerate() {
        anyhow::ensure!(
            argument.len() <= MAX_ADAPTER_ARGUMENT_BYTES && !argument.chars().any(char::is_control),
            "{label} argument {index} is not bounded printable text"
        );
    }
    let bytes = serde_json::to_vec(arguments).context("serialize adapter launch arguments")?;
    let digest = sha256_prefixed(&bytes);
    anyhow::ensure!(
        expected_count == arguments.len() && expected_sha256 == &digest["sha256:".len()..],
        "{label} launch arguments do not match the conformance report"
    );
    Ok(())
}

fn verify_hardware_task_identity(
    id: &str,
    task_spec: &TaskSpec,
    identity: &HardwareAdapterConformanceIdentity,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        identity.task_id == task_spec.task_id,
        "external hardware adapter {id} negotiated the wrong TaskSpec identity"
    );
    let gateway = HardwareGateway::new(task_spec.clone(), GatewayConfig::default())
        .with_context(|| format!("construct external hardware adapter {id} TaskSpec"))?;
    anyhow::ensure!(
        identity.observation_width == gateway.observation_width()
            && identity.action_width == gateway.action_width(),
        "external hardware adapter {id} negotiated TaskSpec widths that do not match the retained TaskSpec"
    );
    Ok(())
}

fn verify_simulator_task_identity(
    id: &str,
    task_spec: &TaskSpec,
    runtime: &SimulatorRuntimeManifest,
    identity: &SimulatorAdapterConformanceIdentity,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        identity.task_id == task_spec.task_id,
        "external simulator adapter {id} negotiated the wrong TaskSpec identity"
    );
    let gateway = HardwareGateway::new(task_spec.clone(), GatewayConfig::default())
        .with_context(|| format!("construct external simulator adapter {id} TaskSpec"))?;
    anyhow::ensure!(
        identity.observation_width == gateway.observation_width()
            && identity.action_width == gateway.action_width(),
        "external simulator adapter {id} negotiated TaskSpec widths that do not match the retained TaskSpec"
    );
    let fixed_delta_ticks = (task_spec.control_step_s * 1_000_000_000.0).round() as u64;
    anyhow::ensure!(
        identity.fixed_delta_ticks == fixed_delta_ticks
            && runtime.fixed_delta_ticks == fixed_delta_ticks,
        "external simulator adapter {id} fixed step differs from the retained TaskSpec"
    );
    anyhow::ensure!(
        identity.simulator_id == runtime.simulator_id
            && identity.simulator_version == runtime.simulator_version,
        "external simulator adapter {id} handshake differs from the retained runtime manifest"
    );
    Ok(())
}

fn evidence_sha256_hex(evidence: &VerifiedEvidence) -> anyhow::Result<&str> {
    evidence
        .sha256
        .strip_prefix("sha256:")
        .context("verified evidence SHA-256 is not canonical")
}

fn evidence_file_name(evidence: &VerifiedEvidence) -> anyhow::Result<&str> {
    evidence
        .path
        .file_name()
        .and_then(|name| name.to_str())
        .context("verified evidence file name is not valid Unicode")
}

fn verify_reference_hardware(
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let Some(reference) = &manifest.reference_hardware else {
        return Ok(Vec::new());
    };
    let evidence = verify_evidence(evidence_root, reference)?;
    lekiwi_evidence::verify_manifest(&evidence.path)?;
    Ok(vec![evidence.sha256])
}

fn verify_platform_releases(
    root: &Path,
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let mut platforms = BTreeSet::new();
    let mut release_identities = BTreeSet::new();
    let mut digests = Vec::new();
    for entry in &manifest.platform_release {
        anyhow::ensure!(
            platforms.insert(entry.platform),
            "duplicate release evidence for {:?}",
            entry.platform
        );
        anyhow::ensure!(
            is_git_object_id(&entry.revision),
            "release evidence revision must be a lowercase 40-character Git ID"
        );
        validate_release_tag(&entry.tag)?;
        ensure_git_ancestor(root, &entry.revision)?;
        ensure_git_tag(root, &entry.tag, &entry.revision)?;
        release_identities.insert((entry.revision.as_str(), entry.tag.as_str()));
        let archive = verify_evidence(evidence_root, &entry.archive)?;
        let attestation = verify_evidence(evidence_root, &entry.attestation)?;
        let archive_attestation_verification =
            verify_evidence(evidence_root, &entry.archive_attestation_verification)?;
        let expected_archive_receipt =
            verify_github_attestation(&AttestationVerificationRequest {
                artifact_path: &archive.path,
                artifact_sha256: &archive.sha256,
                bundle_path: &attestation.path,
                bundle_sha256: &attestation.sha256,
                revision: &entry.revision,
                tag: &entry.tag,
            })?;
        verify_attestation_receipt(
            &archive_attestation_verification.bytes,
            &expected_archive_receipt,
        )?;
        let release = verify_evidence(evidence_root, &entry.release_report)?;
        let checksum = verify_evidence(evidence_root, &entry.checksum_manifest)?;
        let install = verify_evidence(evidence_root, &entry.install_report)?;
        let install_attestation_verification =
            verify_evidence(evidence_root, &entry.install_attestation_verification)?;
        let expected_install_receipt =
            verify_github_attestation(&AttestationVerificationRequest {
                artifact_path: &install.path,
                artifact_sha256: &install.sha256,
                bundle_path: &attestation.path,
                bundle_sha256: &attestation.sha256,
                revision: &entry.revision,
                tag: &entry.tag,
            })?;
        verify_attestation_receipt(
            &install_attestation_verification.bytes,
            &expected_install_receipt,
        )?;
        release_artifacts::validate_readiness_release_reports(
            release_artifacts::ReadinessReleaseEvidence {
                archive_path: &archive.path,
                archive_sha256: &archive.sha256,
                release_report_path: &release.path,
                release_report_sha256: &release.sha256,
                checksum_manifest_path: &checksum.path,
                checksum_manifest_sha256: &checksum.sha256,
                install_report_path: &install.path,
                install_report_sha256: &install.sha256,
            },
            release_artifacts::ReadinessReleaseIdentity {
                target: entry.platform.target(),
                commit: &entry.revision,
                tag: &entry.tag,
            },
        )?;
        digests.extend([
            archive.sha256,
            attestation.sha256,
            archive_attestation_verification.sha256,
            release.sha256,
            checksum.sha256,
            install.sha256,
            install_attestation_verification.sha256,
        ]);
    }
    anyhow::ensure!(
        release_identities.len() <= 1,
        "all platform release evidence must reproduce the same tagged revision"
    );
    anyhow::ensure!(
        platforms
            .iter()
            .copied()
            .eq(manifest.required_platforms.iter().copied())
            || platforms.is_empty(),
        "release evidence platforms must exactly match the required platform registry"
    );
    Ok(digests)
}

fn verify_github_attestation(
    request: &AttestationVerificationRequest<'_>,
) -> anyhow::Result<AttestationVerificationReceipt> {
    let args = github_attestation_verify_args(request);
    let output = Command::new("gh")
        .args(&args)
        .env("GH_PROMPT_DISABLED", "1")
        .env("NO_COLOR", "1")
        .output()
        .context("run GitHub CLI attestation verifier")?;
    anyhow::ensure!(
        output.status.success(),
        "GitHub attestation verification failed for {}: {}",
        request.artifact_path.display(),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    let verified_attestations = validate_github_attestation_output(&output.stdout, request)?;
    anyhow::ensure!(
        verified_attestations == 1,
        "the retained bundle must contain exactly one verified attestation, found {verified_attestations}"
    );
    Ok(attestation_receipt(request, verified_attestations))
}

fn github_attestation_verify_args(request: &AttestationVerificationRequest<'_>) -> Vec<OsString> {
    let source_ref = format!("refs/tags/{}", request.tag);
    let certificate_identity = format!(
        "https://github.com/{}/{}@{}",
        release_exit::EXPECTED_ATTESTATION_REPOSITORY,
        release_exit::EXPECTED_ATTESTATION_WORKFLOW,
        source_ref
    );
    [
        OsString::from("attestation"),
        OsString::from("verify"),
        request.artifact_path.as_os_str().to_owned(),
        OsString::from("-R"),
        OsString::from(release_exit::EXPECTED_ATTESTATION_REPOSITORY),
        OsString::from("--bundle"),
        request.bundle_path.as_os_str().to_owned(),
        OsString::from("--cert-identity"),
        OsString::from(certificate_identity),
        OsString::from("--source-ref"),
        OsString::from(source_ref),
        OsString::from("--source-digest"),
        OsString::from(request.revision),
        OsString::from("--signer-digest"),
        OsString::from(request.revision),
        OsString::from("--cert-oidc-issuer"),
        OsString::from(release_exit::EXPECTED_ATTESTATION_ISSUER),
        OsString::from("--predicate-type"),
        OsString::from(release_exit::EXPECTED_ATTESTATION_PREDICATE),
        OsString::from("--deny-self-hosted-runners"),
        OsString::from("--format"),
        OsString::from("json"),
    ]
    .into()
}

fn validate_github_attestation_output(
    bytes: &[u8],
    request: &AttestationVerificationRequest<'_>,
) -> anyhow::Result<u32> {
    let results: Vec<GhAttestationVerification> = serde_json::from_slice(bytes)
        .context("GitHub attestation verifier did not return its JSON array")?;
    anyhow::ensure!(
        !results.is_empty(),
        "GitHub attestation verifier returned no verified attestations"
    );
    let expected_digest = request
        .artifact_sha256
        .strip_prefix("sha256:")
        .context("verified artifact digest is not canonical SHA-256")?;
    for result in &results {
        anyhow::ensure!(
            result.verification_result.statement.predicate_type
                == release_exit::EXPECTED_ATTESTATION_PREDICATE,
            "verified attestation predicate drifted"
        );
        anyhow::ensure!(
            result
                .verification_result
                .statement
                .subject
                .iter()
                .any(|subject| subject.digest.sha256.as_deref() == Some(expected_digest)),
            "verified attestation did not bind the exact archive SHA-256"
        );
    }
    u32::try_from(results.len()).context("too many verified attestations")
}

fn attestation_receipt(
    request: &AttestationVerificationRequest<'_>,
    verified_attestations: u32,
) -> AttestationVerificationReceipt {
    let source_ref = format!("refs/tags/{}", request.tag);
    AttestationVerificationReceipt {
        kind: ATTESTATION_RECEIPT_KIND.to_string(),
        schema_version: ATTESTATION_RECEIPT_SCHEMA_VERSION,
        provider: release_exit::EXPECTED_ATTESTATION_PROVIDER.to_string(),
        repository: release_exit::EXPECTED_ATTESTATION_REPOSITORY.to_string(),
        certificate_identity: format!(
            "https://github.com/{}/{}@{}",
            release_exit::EXPECTED_ATTESTATION_REPOSITORY,
            release_exit::EXPECTED_ATTESTATION_WORKFLOW,
            source_ref
        ),
        source_ref,
        source_revision: request.revision.to_string(),
        signer_revision: request.revision.to_string(),
        issuer: release_exit::EXPECTED_ATTESTATION_ISSUER.to_string(),
        predicate_type: release_exit::EXPECTED_ATTESTATION_PREDICATE.to_string(),
        deny_self_hosted_runners: true,
        artifact_sha256: request.artifact_sha256.to_string(),
        attestation_bundle_sha256: request.bundle_sha256.to_string(),
        verified_attestations,
    }
}

fn verify_attestation_receipt(
    bytes: &[u8],
    expected: &AttestationVerificationReceipt,
) -> anyhow::Result<()> {
    let actual: AttestationVerificationReceipt =
        serde_json::from_slice(bytes).context("parse strict attestation verification receipt")?;
    anyhow::ensure!(
        actual == *expected,
        "attestation verification receipt does not match fresh cryptographic verification"
    );
    Ok(())
}

fn verify_compatibility(
    root: &Path,
    evidence_root: &Path,
    manifest: &ReadinessManifest,
) -> anyhow::Result<Vec<String>> {
    let Some(reference) = &manifest.compatibility_report else {
        return Ok(Vec::new());
    };
    let evidence = verify_evidence(evidence_root, reference)?;
    let report: CompatibilityFixtureReport =
        serde_json::from_slice(&evidence.bytes).context("parse historical compatibility report")?;
    let registry =
        rne_compatibility_suite::read_registry(&root.join("release/compatibility-fixtures.toml"))?;
    report.validate(&registry)?;
    let mut ids = BTreeSet::new();
    anyhow::ensure!(
        report.checks.len() >= manifest.minimum_compatibility_checks
            && report.checks.iter().all(|check| {
                ids.insert(check.id.as_str())
                    && check.accepted
                    && check.future_schema_rejected
                    && check.unknown_field_rejected
                    && check.passed
            })
            && report.passed,
        "historical compatibility report did not pass the required corpus"
    );
    rne_compatibility_suite::verify_historical_source_history(root)
        .context("reverify historical compatibility source provenance")?;
    let replayed = rne_compatibility_suite::run_compatibility(
        root,
        &root.join("release/compatibility-fixtures.toml"),
    )
    .context("replay historical compatibility corpus with current typed readers")?;
    anyhow::ensure!(
        report == replayed,
        "retained historical compatibility report does not match a fresh typed-reader replay"
    );

    let mut digests = Vec::with_capacity(report.checks.len() + 2);
    digests.push(evidence.sha256);
    digests.push(report.registry_sha256);
    digests.extend(
        report
            .checks
            .into_iter()
            .map(|check| check.canonical_json_sha256),
    );
    Ok(digests)
}

fn verify_evidence(root: &Path, reference: &EvidenceRef) -> anyhow::Result<VerifiedEvidence> {
    anyhow::ensure!(
        is_sha256(&reference.sha256),
        "evidence SHA-256 is not canonical"
    );
    anyhow::ensure!(
        !reference.path.is_empty() && !reference.path.contains('\\'),
        "evidence path must be a non-empty forward-slash relative path"
    );
    let relative = Path::new(&reference.path);
    anyhow::ensure!(
        !relative.is_absolute()
            && relative
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "evidence path escapes its manifest directory: {}",
        reference.path
    );
    let canonical_root = fs::canonicalize(root)
        .with_context(|| format!("resolve evidence root {}", root.display()))?;
    let candidate = root.join(relative);
    let metadata = fs::symlink_metadata(&candidate)
        .with_context(|| format!("inspect evidence {}", candidate.display()))?;
    anyhow::ensure!(
        metadata.is_file() && !metadata.file_type().is_symlink(),
        "evidence must be a regular non-symlink file: {}",
        candidate.display()
    );
    anyhow::ensure!(
        metadata.len() <= MAX_EVIDENCE_BYTES,
        "evidence exceeds {MAX_EVIDENCE_BYTES} bytes: {}",
        candidate.display()
    );
    let path = fs::canonicalize(&candidate)?;
    anyhow::ensure!(
        path.starts_with(&canonical_root),
        "evidence resolved outside its manifest directory: {}",
        candidate.display()
    );
    let bytes = fs::read(&path)?;
    let actual = sha256_prefixed(&bytes);
    anyhow::ensure!(
        actual == reference.sha256,
        "evidence digest mismatch for {}: expected {}, got {actual}",
        reference.path,
        reference.sha256
    );
    Ok(VerifiedEvidence {
        path,
        sha256: actual,
        bytes,
    })
}

fn observed_use_days(projects: &[ProjectUse]) -> anyhow::Result<u32> {
    let Some(first) = projects.iter().map(|project| project.first).min() else {
        return Ok(0);
    };
    let last = projects
        .iter()
        .map(|project| project.last)
        .max()
        .expect("non-empty projects have a last date");
    first.days_until(last)
}

fn validate_external_owner(
    project_owner: &str,
    owner: &str,
    repository: &str,
    label: &str,
) -> anyhow::Result<()> {
    validate_identifier(&format!("{label} owner"), owner)?;
    anyhow::ensure!(
        !owner.eq_ignore_ascii_case(project_owner),
        "{label} owner must be independent of {project_owner}"
    );
    anyhow::ensure!(
        is_https_url(repository),
        "{label} repository must use HTTPS"
    );
    Ok(())
}

fn validate_external_revision(label: &str, id: &str, revision: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        is_git_object_id(revision),
        "{label} {id} revision must be a lowercase 40-character Git ID"
    );
    Ok(())
}

fn validate_identifier(field: &str, value: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !value.trim().is_empty() && value.len() <= 128 && !value.chars().any(char::is_control),
        "{field} must be a bounded non-empty identifier"
    );
    Ok(())
}

fn is_https_url(value: &str) -> bool {
    value.starts_with("https://")
        && value.len() > "https://".len()
        && !value.chars().any(char::is_whitespace)
}

fn validate_release_tag(value: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        value.starts_with('v')
            && value.len() <= 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'-' | b'_')),
        "release evidence tag must be a bounded v-prefixed Git tag"
    );
    Ok(())
}

fn display_or_missing(value: &str) -> &str {
    if value.trim().is_empty() {
        "<missing>"
    } else {
        value
    }
}

fn parse_digits(bytes: &[u8], field: &str) -> anyhow::Result<u32> {
    anyhow::ensure!(
        bytes.iter().all(|byte| byte.is_ascii_digit()),
        "date {field} contains a non-digit"
    );
    bytes.iter().try_fold(0_u32, |value, byte| {
        value
            .checked_mul(10)
            .and_then(|value| value.checked_add(u32::from(*byte - b'0')))
            .context("date component overflowed")
    })
}

const fn days_in_month(year: i32, month: u32) -> u32 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

const fn is_leap_year(year: i32) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

fn is_git_object_id(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_sha256(value: &str) -> bool {
    value.len() == 71
        && value.starts_with("sha256:")
        && value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn sha256_prefixed(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

fn normalized_text_sha256(text: &str) -> anyhow::Result<String> {
    let normalized = text.replace("\r\n", "\n");
    anyhow::ensure!(
        !normalized.contains('\r'),
        "readiness text evidence contains an unsupported lone carriage return"
    );
    Ok(sha256_prefixed(normalized.as_bytes()))
}

fn git_output(root: &Path, args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git").current_dir(root).args(args).output()?;
    anyhow::ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(String::from_utf8(output.stdout)?.trim().to_string())
}

fn ensure_git_ancestor(root: &Path, revision: &str) -> anyhow::Result<()> {
    let output = Command::new("git")
        .current_dir(root)
        .args(["merge-base", "--is-ancestor", revision, "HEAD"])
        .output()?;
    anyhow::ensure!(
        output.status.success(),
        "1.0 candidate revision must remain an ancestor of HEAD"
    );
    Ok(())
}

fn ensure_git_tag(root: &Path, tag: &str, revision: &str) -> anyhow::Result<()> {
    let reference = format!("refs/tags/{tag}^{{commit}}");
    let actual = git_output(root, &["rev-parse", &reference])?;
    anyhow::ensure!(
        actual == revision,
        "release evidence tag {tag} does not resolve to {revision}"
    );
    Ok(())
}

fn write_report(path: &Path, report: &ReadinessReport) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut bytes = serde_json::to_vec_pretty(report)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_dates_cover_leap_days_and_reject_invalid_dates() {
        let leap_start = CivilDate::parse("2024-02-28").unwrap();
        let march = CivilDate::parse("2024-03-01").unwrap();
        assert_eq!(leap_start.days_until(march).unwrap(), 2);
        assert!(CivilDate::parse("2026-02-29").is_err());
        assert!(CivilDate::parse("2026/08/16").is_err());
    }

    #[test]
    fn support_commitment_shape_fails_closed() {
        let empty = SupportCommitment {
            committed: false,
            maintainer: String::new(),
            support_period: String::new(),
            policy_url: String::new(),
        };
        validate_support_shape(&empty).unwrap();

        for mut ambiguous in [
            SupportCommitment {
                maintainer: "maintainer".to_string(),
                ..empty.clone()
            },
            SupportCommitment {
                support_period: "12 months".to_string(),
                ..empty.clone()
            },
            SupportCommitment {
                policy_url: "https://example.invalid/support".to_string(),
                ..empty.clone()
            },
        ] {
            ambiguous.committed = false;
            assert!(validate_support_shape(&ambiguous).is_err());
        }

        let committed = SupportCommitment {
            committed: true,
            maintainer: "RNE maintainer".to_string(),
            support_period: "12 months after each stable minor release".to_string(),
            policy_url: "https://example.invalid/support".to_string(),
        };
        validate_support_shape(&committed).unwrap();

        let mut missing_maintainer = committed.clone();
        missing_maintainer.maintainer.clear();
        assert!(validate_support_shape(&missing_maintainer).is_err());
        let mut padded_period = committed.clone();
        padded_period.support_period.push(' ');
        assert!(validate_support_shape(&padded_period).is_err());
        let mut insecure_policy = committed.clone();
        insecure_policy.policy_url = "http://example.invalid/support".to_string();
        assert!(validate_support_shape(&insecure_policy).is_err());
        let mut oversized_maintainer = committed;
        oversized_maintainer.maintainer = "m".repeat(129);
        assert!(validate_support_shape(&oversized_maintainer).is_err());
    }

    #[test]
    fn evidence_paths_and_digests_fail_closed() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("report.json");
        fs::write(&path, b"{}\n").unwrap();
        let good = EvidenceRef {
            path: "report.json".to_string(),
            sha256: sha256_prefixed(b"{}\n"),
        };
        assert_eq!(verify_evidence(temp.path(), &good).unwrap().bytes, b"{}\n");
        let bad = EvidenceRef {
            sha256: format!("sha256:{}", "0".repeat(64)),
            ..good
        };
        assert!(verify_evidence(temp.path(), &bad).is_err());
        let escape = EvidenceRef {
            path: "../report.json".to_string(),
            sha256: format!("sha256:{}", "0".repeat(64)),
        };
        assert!(verify_evidence(temp.path(), &escape).is_err());
    }

    #[test]
    fn compatibility_evidence_must_match_a_fresh_typed_reader_replay() {
        let root = workspace_root().unwrap();
        let registry_path = root.join("release/compatibility-fixtures.toml");
        let report = rne_compatibility_suite::run_compatibility(&root, &registry_path).unwrap();
        let temp = tempfile::tempdir().unwrap();
        let report_path = temp.path().join("compatibility-report.json");
        let report_bytes = serde_json::to_vec_pretty(&report).unwrap();
        fs::write(&report_path, &report_bytes).unwrap();

        let mut manifest = read_manifest(&root.join(DEFAULT_MANIFEST)).unwrap();
        manifest.compatibility_report = Some(EvidenceRef {
            path: "compatibility-report.json".to_string(),
            sha256: sha256_prefixed(&report_bytes),
        });
        let digests = verify_compatibility(&root, temp.path(), &manifest).unwrap();
        assert_eq!(digests.len(), report.checks.len() + 2);
        assert_eq!(digests[1], report.registry_sha256);

        let mut fabricated = report;
        fabricated.checks[0].detail = "fabricated passing result".to_string();
        fabricated
            .validate(&rne_compatibility_suite::read_registry(&registry_path).unwrap())
            .unwrap();
        let fabricated_bytes = serde_json::to_vec_pretty(&fabricated).unwrap();
        fs::write(&report_path, &fabricated_bytes).unwrap();
        manifest.compatibility_report.as_mut().unwrap().sha256 = sha256_prefixed(&fabricated_bytes);
        let error = verify_compatibility(&root, temp.path(), &manifest).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("does not match a fresh typed-reader replay"),
            "unexpected error: {error:#}"
        );
    }

    #[test]
    fn external_conformance_subjects_bind_exact_retained_bytes() {
        let bytes = b"independently built subject v1".to_vec();
        let sha256 = sha256_prefixed(&bytes);
        let evidence = VerifiedEvidence {
            path: PathBuf::from("external-controller.dll"),
            sha256: sha256.clone(),
            bytes,
        };
        verify_unprefixed_subject(
            "plugin",
            &evidence,
            "external-controller.dll",
            &sha256["sha256:".len()..],
            Some(evidence.bytes.len() as u64),
        )
        .unwrap();
        verify_prefixed_subject("backend", &evidence, "external-controller.dll", &sha256).unwrap();

        assert!(verify_unprefixed_subject(
            "plugin",
            &evidence,
            "swapped-controller.dll",
            &sha256["sha256:".len()..],
            Some(evidence.bytes.len() as u64),
        )
        .is_err());
        assert!(verify_unprefixed_subject(
            "plugin",
            &evidence,
            "external-controller.dll",
            &"0".repeat(64),
            Some(evidence.bytes.len() as u64),
        )
        .is_err());
        assert!(verify_unprefixed_subject(
            "plugin",
            &evidence,
            "external-controller.dll",
            &sha256["sha256:".len()..],
            Some(evidence.bytes.len() as u64 + 1),
        )
        .is_err());
    }

    #[test]
    fn external_adapter_launch_contract_is_canonical_and_bounded() {
        let arguments = vec![
            "<adapter-subject>".to_string(),
            "--device-id".to_string(),
            "third-party-v1".to_string(),
        ];
        let digest = sha256_prefixed(&serde_json::to_vec(&arguments).unwrap());
        let subject = HardwareAdapterConformanceSubject {
            adapter_file: "adapter.py".to_string(),
            adapter_sha256: "a".repeat(64),
            adapter_size_bytes: 42,
            launcher_file: "python".to_string(),
            arguments_sha256: digest["sha256:".len()..].to_string(),
            argument_count: arguments.len(),
            task_file: "task.json".to_string(),
            task_sha256: "b".repeat(64),
        };
        verify_adapter_arguments(&arguments, &subject).unwrap();

        let mut reordered = arguments.clone();
        reordered.swap(1, 2);
        assert!(verify_adapter_arguments(&reordered, &subject).is_err());
        assert!(verify_adapter_arguments(&arguments[..2], &subject).is_err());
        let control = vec!["bad\nargument".to_string()];
        assert!(verify_adapter_arguments(&control, &subject).is_err());
        let oversized = vec!["x".repeat(MAX_ADAPTER_ARGUMENT_BYTES + 1)];
        assert!(verify_adapter_arguments(&oversized, &subject).is_err());

        let task: TaskSpec =
            serde_json::from_str(include_str!("../../tests/golden/tasks/task-spec-v1.json"))
                .unwrap();
        let gateway = HardwareGateway::new(task.clone(), GatewayConfig::default()).unwrap();
        let identity = HardwareAdapterConformanceIdentity {
            device_id: "third-party-v1".to_string(),
            task_id: task.task_id.clone(),
            wire_schema_version: 1,
            observation_width: gateway.observation_width(),
            action_width: gateway.action_width(),
        };
        verify_hardware_task_identity("adapter", &task, &identity).unwrap();
        let mut wrong_task = identity.clone();
        wrong_task.task_id = "wrong.task".to_string();
        assert!(verify_hardware_task_identity("adapter", &task, &wrong_task).is_err());
        let mut wrong_width = identity;
        wrong_width.action_width += 1;
        assert!(verify_hardware_task_identity("adapter", &task, &wrong_width).is_err());
    }

    #[test]
    fn external_physics_report_is_rebound_to_its_retained_subject() {
        let temp = tempfile::tempdir().unwrap();
        let subject_name = "reference-external-backend-source.tar.zst";
        let report_name = "external-physics-report.json";
        let subject_bytes = b"reference external backend source bundle v1";
        let report_bytes = include_bytes!(
            "../../crates/rne_physics_conformance/tests/golden/external-backend-conformance-v1.json"
        );
        fs::write(temp.path().join(subject_name), subject_bytes).unwrap();
        fs::write(temp.path().join(report_name), report_bytes).unwrap();

        let manifest_text = format!(
            r#"
schema_version = 8
release_version = "0.1.0"
project_owner = "project-owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]
accelerator_adapter = []

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""

[[external_system]]
id = "external-physics"
owner = "external-owner"
repository = "https://example.invalid/physics"
revision = "1111111111111111111111111111111111111111"
kind = "physics_backend"
subject = {{ path = "{subject_name}", sha256 = "{}" }}
report = {{ path = "{report_name}", sha256 = "{}" }}
"#,
            sha256_prefixed(subject_bytes),
            sha256_prefixed(report_bytes),
        );
        let mut manifest: ReadinessManifest = toml::from_str(&manifest_text).unwrap();
        assert_eq!(
            verify_external_systems(temp.path(), &manifest).unwrap(),
            vec![
                sha256_prefixed(subject_bytes),
                sha256_prefixed(report_bytes)
            ]
        );

        let swapped = b"different backend implementation";
        fs::write(temp.path().join(subject_name), swapped).unwrap();
        manifest.external_system[0].subject.sha256 = sha256_prefixed(swapped);
        assert!(verify_external_systems(temp.path(), &manifest).is_err());
    }

    #[test]
    fn external_simulator_report_rebinds_task_runtime_and_every_artifact() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let task_source = root.join("assets/tasks/diff_drive_goal.task.json");
        let fixture_root =
            root.join("adapters/hardware/rne_hardware_gateway/tests/fixtures/simulator");
        let retained = [
            ("task.json", task_source),
            ("runtime.json", fixture_root.join("runtime.json")),
            ("world.sdf", fixture_root.join("world.sdf")),
            ("robot.urdf", fixture_root.join("robot.urdf")),
            ("adapter.toml", fixture_root.join("adapter.toml")),
        ];
        for (name, source) in &retained {
            fs::copy(source, temp.path().join(name)).unwrap();
        }
        fs::write(
            temp.path().join("gazebo-adapter.bin"),
            b"external gazebo adapter v1",
        )
        .unwrap();
        let bytes = |name: &str| fs::read(temp.path().join(name)).unwrap();
        let digest = |name: &str| sha256_prefixed(&bytes(name));
        let hex = |name: &str| digest(name)["sha256:".len()..].to_string();
        let runtime: SimulatorRuntimeManifest =
            serde_json::from_slice(&bytes("runtime.json")).unwrap();
        let arguments = vec![
            "--runtime-manifest".to_string(),
            "<runtime-manifest>".to_string(),
        ];
        let arguments_sha256 = sha256_prefixed(&serde_json::to_vec(&arguments).unwrap())
            ["sha256:".len()..]
            .to_string();
        let report = SimulatorAdapterConformanceReport {
            schema_version: 1,
            kind: "rne_simulator_adapter_conformance_report".to_string(),
            status: "passed".to_string(),
            subject: SimulatorAdapterConformanceSubject {
                adapter_file: "gazebo-adapter.bin".to_string(),
                adapter_sha256: hex("gazebo-adapter.bin"),
                adapter_size_bytes: bytes("gazebo-adapter.bin").len() as u64,
                launcher_file: "gazebo-adapter.bin".to_string(),
                arguments_sha256,
                argument_count: arguments.len(),
                task_file: "task.json".to_string(),
                task_sha256: hex("task.json"),
                runtime_manifest_file: "runtime.json".to_string(),
                runtime_manifest_sha256: hex("runtime.json"),
                runtime_manifest_size_bytes: bytes("runtime.json").len() as u64,
                runtime_artifacts: runtime.artifacts.clone(),
            },
            adapter: Some(SimulatorAdapterConformanceIdentity {
                simulator_id: runtime.simulator_id.clone(),
                simulator_version: runtime.simulator_version.clone(),
                adapter_id: "external_gazebo_adapter".to_string(),
                task_id: "rne.diff_drive.sensor_goal.v1".to_string(),
                wire_schema_version: 1,
                observation_width: 9,
                action_width: 2,
                fixed_delta_ticks: 16_666_667,
            }),
            checks: [
                "open_identity",
                "task_binding",
                "fixed_delta_binding",
                "reset_origin",
                "bounded_step",
                "fixed_step_progression",
                "deterministic_replay",
                "action_sequence_rejection",
                "session_isolation",
                "width_rejection",
            ]
            .into_iter()
            .map(|id| SimulatorAdapterConformanceCheck {
                id: id.to_string(),
                status: "passed".to_string(),
                detail: "independent fixture passed".to_string(),
            })
            .collect(),
        };
        report.validate().unwrap();
        fs::write(
            temp.path().join("simulator-report.json"),
            report.to_json_pretty().unwrap(),
        )
        .unwrap();
        let release_name = "rne-0.2.0-x86_64-unknown-linux-gnu.tar.gz";
        fs::write(temp.path().join(release_name), b"official release archive").unwrap();
        fs::write(temp.path().join("submission.json"), b"candidate bytes").unwrap();
        fs::write(temp.path().join("stdout.txt"), b"passed\n").unwrap();
        fs::write(temp.path().join("stderr.txt"), b"empty\n").unwrap();
        let member = |name: &str| {
            serde_json::json!({
                "path": name,
                "size_bytes": bytes(name).len(),
                "sha256": hex(name),
            })
        };
        let submission_report = serde_json::json!({
            "kind": "rne_external_simulator_adapter_submission_report",
            "schema_version": 1,
            "status": "passed",
            "owner": "external-owner",
            "repository": "https://github.com/external-owner/gazebo",
            "revision": "1111111111111111111111111111111111111111",
            "author_assistance": false,
            "release_tag": "v0.2.0",
            "release_target": "x86_64-unknown-linux-gnu",
            "operating_system": "linux",
            "architecture": "x86_64",
            "adapter_arguments": arguments,
            "adapter_identity": report.adapter.as_ref().unwrap(),
            "release_archive": member(release_name),
            "adapter": member("gazebo-adapter.bin"),
            "task_spec": member("task.json"),
            "runtime_manifest": member("runtime.json"),
            "runtime_artifacts": [member("world.sdf"), member("robot.urdf"), member("adapter.toml")],
            "conformance_report": member("simulator-report.json"),
            "submission_candidate": member("submission.json"),
            "stdout_log": member("stdout.txt"),
            "stderr_log": member("stderr.txt"),
        });
        fs::write(
            temp.path().join("submission-report.json"),
            serde_json::to_vec_pretty(&submission_report).unwrap(),
        )
        .unwrap();

        let manifest_text = format!(
            r#"
schema_version = 8
release_version = "0.2.0"
project_owner = "project-owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]
accelerator_adapter = []

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""

[[external_system]]
id = "external-gazebo"
owner = "external-owner"
repository = "https://github.com/external-owner/gazebo"
revision = "1111111111111111111111111111111111111111"
kind = "simulator_adapter"
subject = {{ path = "gazebo-adapter.bin", sha256 = "{}" }}
task_spec = {{ path = "task.json", sha256 = "{}" }}
adapter_arguments = ["--runtime-manifest", "<runtime-manifest>"]
runtime_manifest = {{ path = "runtime.json", sha256 = "{}" }}
runtime_artifacts = [
  {{ path = "world.sdf", sha256 = "{}" }},
  {{ path = "robot.urdf", sha256 = "{}" }},
  {{ path = "adapter.toml", sha256 = "{}" }},
]
report = {{ path = "simulator-report.json", sha256 = "{}" }}
release_archive = {{ path = "{release_name}", sha256 = "{}" }}
submission_candidate = {{ path = "submission.json", sha256 = "{}" }}
stdout_log = {{ path = "stdout.txt", sha256 = "{}" }}
stderr_log = {{ path = "stderr.txt", sha256 = "{}" }}
submission_report = {{ path = "submission-report.json", sha256 = "{}" }}
"#,
            digest("gazebo-adapter.bin"),
            digest("task.json"),
            digest("runtime.json"),
            digest("world.sdf"),
            digest("robot.urdf"),
            digest("adapter.toml"),
            digest("simulator-report.json"),
            digest(release_name),
            digest("submission.json"),
            digest("stdout.txt"),
            digest("stderr.txt"),
            digest("submission-report.json"),
        );
        let mut manifest: ReadinessManifest = toml::from_str(&manifest_text).unwrap();
        let verified = verify_external_systems(temp.path(), &manifest).unwrap();
        assert_eq!(verified.len(), 12);

        fs::write(temp.path().join("robot.urdf"), b"tampered robot").unwrap();
        manifest.external_system[0].runtime_artifacts[1].sha256 = digest("robot.urdf");
        assert!(verify_external_systems(temp.path(), &manifest).is_err());
    }

    #[test]
    fn external_accelerator_report_is_rebound_without_satisfying_external_system() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let fixtures = [
            (
                "server.py",
                root.join("adapters/mjx/rne_mjx_adapter/server.py"),
            ),
            (
                "accelerator.toml",
                root.join("adapters/mjx/accelerator.toml"),
            ),
            ("runtime.toml", root.join("adapters/mjx/runtime.toml")),
            (
                "free-fall-task-spec-v1.json",
                root.join("adapters/mjx/fixtures/free-fall-task-spec-v1.json"),
            ),
            (
                "process-conformance-report-v1.json",
                root.join("tests/golden/accelerators/process-conformance-report-v1.json"),
            ),
        ];
        for (name, source) in &fixtures {
            let text = fs::read_to_string(source).unwrap().replace("\r\n", "\n");
            assert!(!text.contains('\r'));
            fs::write(temp.path().join(name), text).unwrap();
        }
        let digest = |name: &str| sha256_prefixed(&fs::read(temp.path().join(name)).unwrap());
        let manifest_text = format!(
            r#"
schema_version = 8
release_version = "0.1.0"
project_owner = "project-owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""

[[accelerator_adapter]]
id = "external-accelerator"
owner = "external-owner"
repository = "https://example.invalid/accelerator"
revision = "1111111111111111111111111111111111111111"
subject = {{ path = "server.py", sha256 = "{}" }}
task_spec = {{ path = "free-fall-task-spec-v1.json", sha256 = "{}" }}
accelerator_manifest = {{ path = "accelerator.toml", sha256 = "{}" }}
runtime_contract = {{ path = "runtime.toml", sha256 = "{}" }}
adapter_arguments = ["-m", "rne_mjx_adapter", "--backend", "fake", "--allow-test-backend"]
report = {{ path = "process-conformance-report-v1.json", sha256 = "{}" }}
"#,
            digest("server.py"),
            digest("free-fall-task-spec-v1.json"),
            digest("accelerator.toml"),
            digest("runtime.toml"),
            digest("process-conformance-report-v1.json"),
        );
        let mut manifest: ReadinessManifest = toml::from_str(&manifest_text).unwrap();
        assert!(manifest.external_system.is_empty());
        assert_eq!(
            verify_accelerator_adapters(temp.path(), &manifest)
                .unwrap()
                .len(),
            ACCELERATOR_ADAPTER_EVIDENCE_FILES
        );

        manifest.accelerator_adapter[0].adapter_arguments.swap(0, 1);
        assert!(verify_accelerator_adapters(temp.path(), &manifest).is_err());
        manifest.accelerator_adapter[0].adapter_arguments.swap(0, 1);

        let runtime_path = temp.path().join("runtime.toml");
        let runtime_bytes = fs::read(&runtime_path).unwrap();
        let mut semantically_equivalent_runtime = runtime_bytes.clone();
        semantically_equivalent_runtime.push(b'\n');
        fs::write(&runtime_path, &semantically_equivalent_runtime).unwrap();
        manifest.accelerator_adapter[0].runtime_contract.sha256 =
            sha256_prefixed(&semantically_equivalent_runtime);
        assert!(verify_accelerator_adapters(temp.path(), &manifest).is_err());
        fs::write(&runtime_path, &runtime_bytes).unwrap();
        manifest.accelerator_adapter[0].runtime_contract.sha256 = sha256_prefixed(&runtime_bytes);

        fs::write(temp.path().join("server.py"), b"substituted adapter").unwrap();
        manifest.accelerator_adapter[0].subject.sha256 = digest("server.py");
        assert!(verify_accelerator_adapters(temp.path(), &manifest).is_err());
    }

    #[test]
    fn text_evidence_digest_is_line_ending_independent() {
        assert_eq!(
            normalized_text_sha256("first\nsecond\n").unwrap(),
            normalized_text_sha256("first\r\nsecond\r\n").unwrap()
        );
        assert!(normalized_text_sha256("first\rsecond\n").is_err());
    }

    #[test]
    fn attestation_verifier_pins_bundle_identity_tag_and_revision() {
        let artifact_sha256 = format!("sha256:{}", "a".repeat(64));
        let bundle_sha256 = format!("sha256:{}", "b".repeat(64));
        let request = AttestationVerificationRequest {
            artifact_path: Path::new("evidence/archive with spaces.zip"),
            artifact_sha256: &artifact_sha256,
            bundle_path: Path::new("evidence/bundle with spaces.json"),
            bundle_sha256: &bundle_sha256,
            revision: "804eda6fc4e6423b06dcc6d54d3e42d0a0ec23cc",
            tag: "v1.0.0",
        };
        let args = github_attestation_verify_args(&request)
            .iter()
            .map(|arg| arg.to_string_lossy().into_owned())
            .collect::<Vec<_>>();
        assert_eq!(
            args,
            [
                "attestation",
                "verify",
                "evidence/archive with spaces.zip",
                "-R",
                "rsasaki0109/RoboSim",
                "--bundle",
                "evidence/bundle with spaces.json",
                "--cert-identity",
                "https://github.com/rsasaki0109/RoboSim/.github/workflows/release.yml@refs/tags/v1.0.0",
                "--source-ref",
                "refs/tags/v1.0.0",
                "--source-digest",
                "804eda6fc4e6423b06dcc6d54d3e42d0a0ec23cc",
                "--signer-digest",
                "804eda6fc4e6423b06dcc6d54d3e42d0a0ec23cc",
                "--cert-oidc-issuer",
                "https://token.actions.githubusercontent.com",
                "--predicate-type",
                "https://slsa.dev/provenance/v1",
                "--deny-self-hosted-runners",
                "--format",
                "json",
            ]
        );
    }

    #[test]
    fn attestation_output_and_receipt_fail_closed_on_tampering() {
        let artifact_sha256 = format!("sha256:{}", "a".repeat(64));
        let bundle_sha256 = format!("sha256:{}", "b".repeat(64));
        let request = AttestationVerificationRequest {
            artifact_path: Path::new("evidence/archive.zip"),
            artifact_sha256: &artifact_sha256,
            bundle_path: Path::new("evidence/bundle.json"),
            bundle_sha256: &bundle_sha256,
            revision: "804eda6fc4e6423b06dcc6d54d3e42d0a0ec23cc",
            tag: "v1.0.0",
        };
        let output = serde_json::to_vec(&serde_json::json!([{
            "attestation": {"mediaType": "application/vnd.dev.sigstore.bundle.v0.3+json"},
            "verificationResult": {
                "statement": {
                    "predicateType": "https://slsa.dev/provenance/v1",
                    "subject": [{"name": "archive.zip", "digest": {"sha256": "a".repeat(64)}}]
                }
            }
        }]))
        .unwrap();
        assert_eq!(
            validate_github_attestation_output(&output, &request).unwrap(),
            1
        );

        let wrong_subject = String::from_utf8(output.clone())
            .unwrap()
            .replace(&"a".repeat(64), &"c".repeat(64));
        assert!(validate_github_attestation_output(wrong_subject.as_bytes(), &request).is_err());

        let expected = attestation_receipt(&request, 1);
        let receipt = serde_json::to_vec(&expected).unwrap();
        verify_attestation_receipt(&receipt, &expected).unwrap();
        let mut golden = serde_json::to_string_pretty(&expected).unwrap();
        golden.push('\n');
        assert_eq!(
            golden,
            include_str!("../../tests/golden/release/github-attestation-verification-v1.json")
        );

        let mut unknown_field = serde_json::to_value(&expected).unwrap();
        unknown_field
            .as_object_mut()
            .unwrap()
            .insert("unexpected".to_string(), serde_json::Value::Bool(true));
        assert!(verify_attestation_receipt(
            &serde_json::to_vec(&unknown_field).unwrap(),
            &expected
        )
        .is_err());

        let mut tampered = expected.clone();
        tampered.source_revision = "c".repeat(40);
        assert!(
            verify_attestation_receipt(&serde_json::to_vec(&tampered).unwrap(), &expected).is_err()
        );
    }

    #[test]
    fn one_x_promotion_inputs_are_mandatory_and_deterministic() {
        let root = Path::new("workspace");
        assert!(promotion_inputs(root, "0.99.0", None, None, None)
            .unwrap()
            .is_none());
        assert!(promotion_inputs(root, "1.0.0", None, None, None).is_err());
        assert!(promotion_inputs(
            root,
            "1.0.0",
            Some("evidence/readiness.toml".to_string()),
            None,
            None,
        )
        .is_err());
        assert!(promotion_inputs(
            root,
            "1.0.0",
            Some("evidence/readiness.toml".to_string()),
            Some("2027-02-30".to_string()),
            None,
        )
        .is_err());

        let inputs = promotion_inputs(
            root,
            "1.0.0-rc.1",
            Some("evidence/readiness.toml".to_string()),
            Some("2027-02-15".to_string()),
            Some("evidence/promotion.json".to_string()),
        )
        .unwrap()
        .unwrap();
        assert_eq!(inputs.manifest, root.join("evidence/readiness.toml"));
        assert_eq!(inputs.output, root.join("evidence/promotion.json"));
        assert_eq!(inputs.as_of.to_string(), "2027-02-15");

        assert!(version_requires_one_zero_promotion("1.0.0").unwrap());
        assert!(version_requires_one_zero_promotion("2.0.0").unwrap());
        assert!(!version_requires_one_zero_promotion("0.1.0").unwrap());
        assert!(version_requires_one_zero_promotion("v1.0.0").is_err());
    }

    #[test]
    fn current_zero_x_release_does_not_require_promotion_environment() {
        let root = workspace_root().unwrap();
        enforce_release_promotion(&root).unwrap();
    }

    #[test]
    fn committed_manifest_is_valid_but_not_yet_eligible() {
        let root = workspace_root().unwrap();
        let path = root.join(DEFAULT_MANIFEST);
        let manifest = read_manifest(&path).unwrap();
        validate_manifest_identity(&root, &manifest).unwrap();
        let report = evaluate(
            &root,
            &path,
            &manifest,
            CivilDate::parse("2026-08-20").unwrap(),
        )
        .unwrap();
        assert!(!report.eligible);
        assert_eq!(
            report
                .checks
                .iter()
                .map(|check| check.id.as_str())
                .collect::<Vec<_>>(),
            CHECK_IDS
        );
        assert_eq!(
            report
                .checks
                .iter()
                .filter(|check| check.status == "passed")
                .map(|check| check.id.as_str())
                .collect::<Vec<_>>(),
            ["historical_compatibility", "p0_p1_blockers"]
        );
    }

    #[test]
    fn committed_progress_report_matches_golden() {
        let root = workspace_root().unwrap();
        let path = root.join(DEFAULT_MANIFEST);
        let manifest = read_manifest(&path).unwrap();
        let report = evaluate(
            &root,
            &path,
            &manifest,
            CivilDate::parse("2026-08-20").unwrap(),
        )
        .unwrap();
        let mut actual = serde_json::to_string_pretty(&report).unwrap();
        actual.push('\n');
        assert_eq!(
            actual,
            include_str!("../../tests/golden/release/one-zero-readiness-v1.json")
        );
    }

    #[test]
    fn unknown_manifest_fields_are_rejected() {
        let manifest = r#"
schema_version = 8
release_version = "0.1.0"
project_owner = "owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]
unexpected = true
accelerator_adapter = []

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""
"#;
        assert!(toml::from_str::<ReadinessManifest>(manifest).is_err());
    }

    #[test]
    fn platform_release_manifest_v5_requires_the_complete_archive_chain() {
        let manifest = r#"
schema_version = 8
release_version = "0.1.0"
project_owner = "owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]
accelerator_adapter = []

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""

[[platform_release]]
platform = "windows_x86_64"
revision = "1111111111111111111111111111111111111111"
tag = "v1.0.0"
archive = { path = "release/archive.zip", sha256 = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" }
attestation = { path = "release/bundle.json", sha256 = "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" }
archive_attestation_verification = { path = "release/archive-receipt.json", sha256 = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc" }
release_report = { path = "release/release-report.json", sha256 = "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd" }
checksum_manifest = { path = "release/SHA256SUMS", sha256 = "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" }
install_report = { path = "release/archive-install-report.json", sha256 = "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff" }
install_attestation_verification = { path = "release/install-receipt.json", sha256 = "sha256:9999999999999999999999999999999999999999999999999999999999999999" }
"#;
        let parsed: ReadinessManifest = toml::from_str(manifest).unwrap();
        assert_eq!(parsed.platform_release.len(), 1);

        let relabelled_v3 = manifest.replace("accelerator_adapter = []\n", "");
        assert!(toml::from_str::<ReadinessManifest>(&relabelled_v3).is_err());

        let missing_checksum = manifest.replace(
            "checksum_manifest = { path = \"release/SHA256SUMS\", sha256 = \"sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\" }\n",
            "",
        );
        assert!(toml::from_str::<ReadinessManifest>(&missing_checksum).is_err());
        let legacy_name = manifest.replace(
            "archive_attestation_verification",
            "attestation_verification",
        );
        assert!(toml::from_str::<ReadinessManifest>(&legacy_name).is_err());
    }

    #[test]
    fn legacy_unbound_external_reports_cannot_be_relabelled_as_manifest_v5() {
        let manifest = r#"
schema_version = 8
release_version = "0.1.0"
project_owner = "project-owner"
minimum_stability_days = 183
minimum_external_projects = 2
minimum_compatibility_checks = 36
unplanned_breaking_changes = 0
blocker_registry = "release/blockers.toml"
required_platforms = ["linux_x86_64", "windows_x86_64"]
accelerator_adapter = []

[candidate]
revision = "0000000000000000000000000000000000000000"
tree = "0000000000000000000000000000000000000000"
since = "2026-08-15"

[support]
committed = false
maintainer = ""
support_period = ""
policy_url = ""

[[third_party_plugin]]
id = "unbound-plugin"
owner = "external-owner"
repository = "https://example.invalid/plugin"
report = { path = "plugin/report.json", sha256 = "sha256:0000000000000000000000000000000000000000000000000000000000000000" }
"#;
        assert!(toml::from_str::<ReadinessManifest>(manifest).is_err());
    }
}
