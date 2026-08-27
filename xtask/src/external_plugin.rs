//! Verifies a third-party controller-plugin submission without executing untrusted code.

use super::{workspace_root, RELEASE_VERSION};
use anyhow::{bail, Context};
use rne_plugin::{ControllerPluginConformanceReport, PluginKind, PluginManifest};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::ffi::OsStr;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

pub(crate) const EXTERNAL_PLUGIN_SUBMISSION_REPORT_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_KIND: &str = "rne_external_controller_plugin_submission_candidate";
const REPORT_KIND: &str = "rne_external_controller_plugin_submission_report";
const CANDIDATE_STATUS: &str = "not_accepted_pending_maintainer_verification";
const MAX_EVIDENCE_BYTES: u64 = 64 * 1024 * 1024;
const MAX_LOG_BYTES: u64 = 16 * 1024 * 1024;
const MAX_RELEASE_ARCHIVE_BYTES: u64 = 2 * 1024 * 1024 * 1024;

#[derive(Debug)]
struct Options {
    release_archive: PathBuf,
    library: PathBuf,
    manifest: PathBuf,
    report: PathBuf,
    submission: PathBuf,
    evidence_repo_dir: PathBuf,
    revision: String,
    output: PathBuf,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionCandidate {
    kind: String,
    schema_version: u32,
    candidate_status: String,
    author_assistance: bool,
    evidence_repository: Repository,
    release: ReleaseIdentity,
    platform: Platform,
    artifacts: PluginArtifacts,
    reproduction: Reproduction,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct Repository {
    owner: String,
    url: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ReleaseIdentity {
    tag: String,
    target: String,
    archive: Artifact,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct Platform {
    operating_system: String,
    architecture: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct PluginArtifacts {
    library: Artifact,
    manifest: Artifact,
    report: Artifact,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct Artifact {
    url: String,
    file_name: String,
    size_bytes: u64,
    sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct Reproduction {
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

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SubmissionReport {
    kind: String,
    schema_version: u32,
    status: String,
    owner: String,
    repository: String,
    revision: String,
    author_assistance: bool,
    release_tag: String,
    release_target: String,
    operating_system: String,
    architecture: String,
    controller_name: String,
    controller_abi_version: u32,
    controller_schema_version: u32,
    capabilities: Vec<rne_plugin::ControllerCapability>,
    release_archive: MemberDigest,
    library: MemberDigest,
    manifest: MemberDigest,
    conformance_report: MemberDigest,
    submission_candidate: MemberDigest,
    stdout_log: MemberDigest,
    stderr_log: MemberDigest,
}

/// Exact staged files and ownership expected by the 1.0 readiness audit.
pub(crate) struct StagedSubmission<'a> {
    pub(crate) owner: &'a str,
    pub(crate) repository: &'a str,
    pub(crate) revision: &'a str,
    pub(crate) release_archive: &'a Path,
    pub(crate) library: &'a Path,
    pub(crate) manifest: &'a Path,
    pub(crate) conformance_report: &'a Path,
    pub(crate) submission_candidate: &'a Path,
    pub(crate) stdout_log: &'a Path,
    pub(crate) stderr_log: &'a Path,
}

/// Revalidates a maintainer report against every file staged for readiness.
pub(crate) fn validate_staged_submission_report(
    report_bytes: &[u8],
    staged: StagedSubmission<'_>,
) -> anyhow::Result<()> {
    let report: SubmissionReport = serde_json::from_slice(report_bytes)
        .context("parse staged external controller-plugin submission report")?;
    anyhow::ensure!(
        report.kind == REPORT_KIND
            && report.schema_version == EXTERNAL_PLUGIN_SUBMISSION_REPORT_SCHEMA_VERSION
            && report.status == "passed",
        "staged external controller-plugin submission report identity drifted"
    );
    validate_external_repository(staged.owner, staged.repository, staged.revision)?;
    anyhow::ensure!(
        report.owner == staged.owner
            && report.repository == staged.repository
            && report.revision == staged.revision,
        "staged external controller-plugin ownership or revision differs from the maintainer report"
    );
    anyhow::ensure!(
        report.release_tag == format!("v{RELEASE_VERSION}"),
        "staged external controller-plugin report names the wrong RNE release"
    );
    validate_platform(
        &Platform {
            operating_system: report.operating_system.clone(),
            architecture: report.architecture.clone(),
        },
        &report.release_target,
    )?;
    validate_library_platform(&report.library.path, &report.release_target)?;
    anyhow::ensure!(
        report.release_archive.path == release_archive_name(&report.release_target)?,
        "staged external controller-plugin release archive name drifted"
    );

    let release_archive = digest_file(
        staged.release_archive,
        "staged release archive",
        MAX_RELEASE_ARCHIVE_BYTES,
    )?;
    let library = digest_file(
        staged.library,
        "staged controller library",
        MAX_EVIDENCE_BYTES,
    )?;
    let manifest = digest_file(
        staged.manifest,
        "staged plugin manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    let conformance = digest_file(
        staged.conformance_report,
        "staged conformance report",
        MAX_EVIDENCE_BYTES,
    )?;
    anyhow::ensure!(
        report.release_archive == release_archive
            && report.library == library
            && report.manifest == manifest
            && report.conformance_report == conformance,
        "staged external controller-plugin primary artifacts differ from the maintainer report"
    );
    verify_unlocated_member(
        &report.submission_candidate,
        staged.submission_candidate,
        "submission candidate",
        MAX_EVIDENCE_BYTES,
    )?;
    verify_unlocated_member(
        &report.stdout_log,
        staged.stdout_log,
        "stdout log",
        MAX_LOG_BYTES,
    )?;
    verify_unlocated_member(
        &report.stderr_log,
        staged.stderr_log,
        "stderr log",
        MAX_LOG_BYTES,
    )?;

    let plugin_manifest: PluginManifest = serde_json::from_slice(&fs::read(staged.manifest)?)?;
    plugin_manifest.validate()?;
    let conformance_report: ControllerPluginConformanceReport =
        serde_json::from_slice(&fs::read(staged.conformance_report)?)?;
    conformance_report.validate()?;
    anyhow::ensure!(
        conformance_report.passed()
            && conformance_report
                .controller
                .as_ref()
                .is_some_and(|controller| {
                    controller.name == report.controller_name
                        && controller.name == plugin_manifest.name
                        && controller.abi_version == report.controller_abi_version
                        && controller.controller_schema_version == report.controller_schema_version
                        && controller.capabilities == report.capabilities
                }),
        "staged external controller-plugin identity differs from the maintainer report"
    );
    Ok(())
}

fn verify_unlocated_member(
    expected: &MemberDigest,
    path: &Path,
    label: &str,
    maximum_bytes: u64,
) -> anyhow::Result<()> {
    validate_relative_path(&expected.path)?;
    let actual = digest_file(path, label, maximum_bytes)?;
    anyhow::ensure!(
        expected.size_bytes == actual.size_bytes && expected.sha256 == actual.sha256,
        "staged external controller-plugin {label} differs from the maintainer report"
    );
    Ok(())
}

/// Verifies exact third-party plugin submission bytes and emits a staging-ready report.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let Some(options) = parse_options(args)? else {
        return Ok(());
    };
    let release_archive = absolute_from(&root, &options.release_archive);
    let library = absolute_from(&root, &options.library);
    let manifest = absolute_from(&root, &options.manifest);
    let conformance_report = absolute_from(&root, &options.report);
    let submission_path = absolute_from(&root, &options.submission);
    let evidence_repo_dir = absolute_from(&root, &options.evidence_repo_dir);
    let output = absolute_from(&root, &options.output);

    let submission_bytes =
        read_regular_file(&submission_path, "submission candidate", MAX_EVIDENCE_BYTES)?;
    let candidate: SubmissionCandidate = serde_json::from_slice(&submission_bytes)
        .context("parse external controller-plugin submission candidate")?;
    validate_candidate(&candidate, &options.revision)?;
    validate_repository_checkout(
        &evidence_repo_dir,
        &candidate.evidence_repository.url,
        &options.revision,
    )?;
    let submission_relative =
        validate_committed_file(&evidence_repo_dir, &submission_path, "submission candidate")?;

    let archive_digest = validate_artifact(
        &candidate.release.archive,
        &release_archive,
        "release archive",
        MAX_RELEASE_ARCHIVE_BYTES,
    )?;
    let library_digest = validate_artifact(
        &candidate.artifacts.library,
        &library,
        "controller library",
        MAX_EVIDENCE_BYTES,
    )?;
    let manifest_digest = validate_artifact(
        &candidate.artifacts.manifest,
        &manifest,
        "plugin manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    let report_digest = validate_artifact(
        &candidate.artifacts.report,
        &conformance_report,
        "conformance report",
        MAX_EVIDENCE_BYTES,
    )?;

    validate_library_platform(&library_digest.path, &candidate.release.target)?;
    let plugin_manifest: PluginManifest =
        serde_json::from_slice(&fs::read(&manifest)?).context("parse submitted plugin manifest")?;
    plugin_manifest
        .validate()
        .context("validate submitted plugin manifest")?;
    anyhow::ensure!(
        plugin_manifest.kind == PluginKind::Controller,
        "submitted plugin manifest is not a controller plugin"
    );
    let typed_report: ControllerPluginConformanceReport =
        serde_json::from_slice(&fs::read(&conformance_report)?)
            .context("parse submitted controller-plugin conformance report")?;
    typed_report
        .validate()
        .context("validate submitted controller-plugin conformance report")?;
    anyhow::ensure!(
        typed_report.passed(),
        "submitted controller plugin did not pass conformance"
    );
    anyhow::ensure!(
        typed_report.subject.library_file == library_digest.path
            && typed_report.subject.library_size_bytes == library_digest.size_bytes
            && typed_report.subject.library_sha256 == library_digest.sha256
            && typed_report.subject.manifest_file == manifest_digest.path
            && typed_report.subject.manifest_sha256 == manifest_digest.sha256,
        "controller-plugin conformance report is not bound to the submitted library and manifest bytes"
    );
    let controller = typed_report
        .controller
        .context("passing controller-plugin report omitted negotiated identity")?;
    anyhow::ensure!(
        controller.name == plugin_manifest.name,
        "submitted manifest and negotiated controller names differ"
    );

    let stdout_path = resolve_repository_member(
        &evidence_repo_dir,
        &candidate.reproduction.stdout_log_path,
        "stdout log",
    )?;
    let stderr_path = resolve_repository_member(
        &evidence_repo_dir,
        &candidate.reproduction.stderr_log_path,
        "stderr log",
    )?;
    let stdout_relative = validate_committed_file(&evidence_repo_dir, &stdout_path, "stdout log")?;
    let stderr_relative = validate_committed_file(&evidence_repo_dir, &stderr_path, "stderr log")?;
    let mut stdout_log = digest_file(&stdout_path, "stdout log", MAX_LOG_BYTES)?;
    stdout_log.path = stdout_relative;
    let mut stderr_log = digest_file(&stderr_path, "stderr log", MAX_LOG_BYTES)?;
    stderr_log.path = stderr_relative;

    let submission_candidate = MemberDigest {
        path: submission_relative,
        size_bytes: u64::try_from(submission_bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_bytes(&submission_bytes),
    };
    let report = SubmissionReport {
        kind: REPORT_KIND.to_string(),
        schema_version: EXTERNAL_PLUGIN_SUBMISSION_REPORT_SCHEMA_VERSION,
        status: "passed".to_string(),
        owner: candidate.evidence_repository.owner,
        repository: candidate.evidence_repository.url,
        revision: options.revision,
        author_assistance: candidate.author_assistance,
        release_tag: candidate.release.tag,
        release_target: candidate.release.target,
        operating_system: candidate.platform.operating_system,
        architecture: candidate.platform.architecture,
        controller_name: controller.name,
        controller_abi_version: controller.abi_version,
        controller_schema_version: controller.controller_schema_version,
        capabilities: controller.capabilities,
        release_archive: archive_digest,
        library: library_digest,
        manifest: manifest_digest,
        conformance_report: report_digest,
        submission_candidate,
        stdout_log,
        stderr_log,
    };
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "external controller plugin verified: owner={} controller={} report={}",
        report.owner,
        report.controller_name,
        output.display()
    );
    Ok(())
}

fn parse_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<Option<Options>> {
    let mut release_archive = None;
    let mut library = None;
    let mut manifest = None;
    let mut report = None;
    let mut submission = None;
    let mut evidence_repo_dir = None;
    let mut revision = None;
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--release-archive" => release_archive = Some(path_arg(args, &argument)?),
            "--library" => library = Some(path_arg(args, &argument)?),
            "--manifest" => manifest = Some(path_arg(args, &argument)?),
            "--report" => report = Some(path_arg(args, &argument)?),
            "--submission" => submission = Some(path_arg(args, &argument)?),
            "--evidence-repo-dir" => evidence_repo_dir = Some(path_arg(args, &argument)?),
            "--revision" => revision = Some(string_arg(args, &argument)?),
            "--output" => output = Some(path_arg(args, &argument)?),
            "--help" | "-h" => {
                println!("external-plugin-check --release-archive PATH --library PATH --manifest PATH --report PATH --submission PATH --evidence-repo-dir PATH --revision SHA --output PATH");
                return Ok(None);
            }
            other => bail!("unknown external-plugin-check argument: {other}"),
        }
    }
    Ok(Some(Options {
        release_archive: release_archive
            .context("external-plugin-check requires --release-archive PATH")?,
        library: library.context("external-plugin-check requires --library PATH")?,
        manifest: manifest.context("external-plugin-check requires --manifest PATH")?,
        report: report.context("external-plugin-check requires --report PATH")?,
        submission: submission.context("external-plugin-check requires --submission PATH")?,
        evidence_repo_dir: evidence_repo_dir
            .context("external-plugin-check requires --evidence-repo-dir PATH")?,
        revision: revision.context("external-plugin-check requires --revision SHA")?,
        output: output.context("external-plugin-check requires --output PATH")?,
    }))
}

fn path_arg(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<PathBuf> {
    Ok(PathBuf::from(string_arg(args, option)?))
}

fn string_arg(args: &mut impl Iterator<Item = String>, option: &str) -> anyhow::Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn validate_candidate(candidate: &SubmissionCandidate, revision: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        candidate.kind == SUBMISSION_KIND
            && candidate.schema_version == SUBMISSION_SCHEMA_VERSION
            && candidate.candidate_status == CANDIDATE_STATUS,
        "external controller-plugin submission identity or non-acceptance boundary drifted"
    );
    validate_external_repository(
        &candidate.evidence_repository.owner,
        &candidate.evidence_repository.url,
        revision,
    )?;
    anyhow::ensure!(
        candidate.release.tag == format!("v{RELEASE_VERSION}"),
        "external controller-plugin submission must name release v{RELEASE_VERSION}"
    );
    validate_platform(&candidate.platform, &candidate.release.target)?;
    anyhow::ensure!(
        candidate.release.archive.file_name
            == release_archive_name(&candidate.release.target)?,
        "external controller-plugin submission release archive name does not match the official target artifact"
    );
    for (artifact, label) in [
        (&candidate.release.archive, "release archive"),
        (&candidate.artifacts.library, "controller library"),
        (&candidate.artifacts.manifest, "plugin manifest"),
        (&candidate.artifacts.report, "conformance report"),
    ] {
        validate_artifact_shape(artifact, label)?;
    }
    anyhow::ensure!(
        candidate.reproduction.commands.len() >= 2
            && candidate.reproduction.commands.len()
                == candidate.reproduction.exit_statuses.len()
            && candidate
                .reproduction
                .commands
                .iter()
                .all(|command| !command.trim().is_empty() && command.len() <= 4096)
            && candidate
                .reproduction
                .exit_statuses
                .iter()
                .all(|status| *status == 0),
        "external controller-plugin submission must retain at least two successful commands and matching zero exit statuses"
    );
    anyhow::ensure!(
        candidate.reproduction.stdout_log_path != candidate.reproduction.stderr_log_path,
        "external controller-plugin stdout and stderr logs must be distinct"
    );
    validate_relative_path(&candidate.reproduction.stdout_log_path)?;
    validate_relative_path(&candidate.reproduction.stderr_log_path)
}

fn validate_platform(platform: &Platform, target: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        platform.architecture == "x86_64"
            && matches!(
                (platform.operating_system.as_str(), target),
                ("windows", "x86_64-pc-windows-msvc") | ("linux", "x86_64-unknown-linux-gnu")
            ),
        "external controller-plugin platform and release target differ"
    );
    Ok(())
}

fn validate_library_platform(file_name: &str, target: &str) -> anyhow::Result<()> {
    let valid = match target {
        "x86_64-pc-windows-msvc" => file_name.ends_with(".dll"),
        "x86_64-unknown-linux-gnu" => file_name.ends_with(".so"),
        _ => false,
    };
    anyhow::ensure!(
        valid,
        "controller library extension does not match release target"
    );
    Ok(())
}

fn release_archive_name(target: &str) -> anyhow::Result<String> {
    let suffix = match target {
        "x86_64-pc-windows-msvc" => "zip",
        "x86_64-unknown-linux-gnu" => "tar.gz",
        _ => bail!("unsupported external controller-plugin release target {target}"),
    };
    Ok(format!("rne-{RELEASE_VERSION}-{target}.{suffix}"))
}

fn validate_external_repository(owner: &str, url: &str, revision: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !owner.eq_ignore_ascii_case("rsasaki0109")
            && !owner.is_empty()
            && owner.len() <= 39
            && owner
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-')
            && !owner.starts_with('-')
            && !owner.ends_with('-'),
        "external controller-plugin owner must be an independent canonical GitHub owner"
    );
    let prefix = format!("https://github.com/{owner}/");
    anyhow::ensure!(
        url.starts_with(&prefix)
            && url.len() > prefix.len()
            && url.len() <= 256
            && !url[prefix.len()..].contains('/')
            && !url.contains('?')
            && !url.contains('#')
            && url.is_ascii(),
        "external controller-plugin repository must be one public GitHub repository owned by {owner}"
    );
    anyhow::ensure!(
        revision.len() == 40 && revision.bytes().all(is_lower_hex),
        "external controller-plugin revision must be 40 lowercase hexadecimal characters"
    );
    Ok(())
}

fn validate_artifact_shape(artifact: &Artifact, label: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        artifact.url.starts_with("https://")
            && artifact.url.is_ascii()
            && artifact.url.len() <= 2048
            && !artifact.url.contains('?')
            && !artifact.url.contains('#')
            && !artifact.file_name.is_empty()
            && artifact.file_name.len() <= 255
            && Path::new(&artifact.file_name).file_name() == Some(OsStr::new(&artifact.file_name))
            && artifact.url.ends_with(&format!("/{}", artifact.file_name))
            && artifact.size_bytes > 0
            && artifact.sha256.len() == 64
            && artifact.sha256.bytes().all(is_lower_hex),
        "external controller-plugin {label} identity is invalid"
    );
    Ok(())
}

fn validate_artifact(
    submitted: &Artifact,
    path: &Path,
    label: &str,
    maximum_bytes: u64,
) -> anyhow::Result<MemberDigest> {
    let digest = digest_file(path, label, maximum_bytes)?;
    anyhow::ensure!(
        submitted.file_name == digest.path
            && submitted.size_bytes == digest.size_bytes
            && submitted.sha256 == digest.sha256,
        "external controller-plugin {label} bytes differ from the submission candidate"
    );
    Ok(digest)
}

fn digest_file(path: &Path, label: &str, maximum_bytes: u64) -> anyhow::Result<MemberDigest> {
    let bytes = read_regular_file(path, label, maximum_bytes)?;
    let name = path
        .file_name()
        .and_then(OsStr::to_str)
        .with_context(|| format!("external controller-plugin {label} name is not UTF-8"))?;
    Ok(MemberDigest {
        path: name.to_string(),
        size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
        sha256: sha256_bytes(&bytes),
    })
}

fn read_regular_file(path: &Path, label: &str, maximum_bytes: u64) -> anyhow::Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path).with_context(|| {
        format!(
            "inspect external controller-plugin {label} {}",
            path.display()
        )
    })?;
    anyhow::ensure!(
        metadata.file_type().is_file()
            && !metadata.file_type().is_symlink()
            && metadata.len() > 0
            && metadata.len() <= maximum_bytes,
        "external controller-plugin {label} must be a non-empty regular non-symlink file no larger than {maximum_bytes} bytes"
    );
    fs::read(path)
        .with_context(|| format!("read external controller-plugin {label} {}", path.display()))
}

fn validate_repository_checkout(
    root: &Path,
    expected_url: &str,
    expected_revision: &str,
) -> anyhow::Result<()> {
    let metadata = fs::symlink_metadata(root)
        .with_context(|| format!("inspect external plugin repository {}", root.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
        "external plugin repository must be a real non-symlink directory"
    );
    anyhow::ensure!(
        git_output(root, &["rev-parse", "HEAD"])? == expected_revision,
        "external plugin repository HEAD differs from the submitted revision"
    );
    anyhow::ensure!(
        git_output(root, &["status", "--porcelain", "--untracked-files=all"])?.is_empty(),
        "external plugin repository must be clean including untracked files"
    );
    let origin = git_output(root, &["remote", "get-url", "origin"])?;
    anyhow::ensure!(
        origin.strip_suffix(".git").unwrap_or(&origin)
            == expected_url.strip_suffix(".git").unwrap_or(expected_url),
        "external plugin repository origin differs from the submission candidate"
    );
    Ok(())
}

fn validate_committed_file(root: &Path, path: &Path, label: &str) -> anyhow::Result<String> {
    let canonical_root = fs::canonicalize(root)?;
    let canonical_path = fs::canonicalize(path)?;
    let relative = canonical_path
        .strip_prefix(&canonical_root)
        .with_context(|| format!("external plugin {label} is outside its evidence repository"))?;
    let relative_text = relative.to_string_lossy().replace('\\', "/");
    validate_relative_path(&relative_text)?;
    let object = format!("HEAD:{relative_text}");
    let output = Command::new("git")
        .current_dir(&canonical_root)
        .args(["show", "--no-textconv", &object])
        .output()?;
    anyhow::ensure!(
        output.status.success(),
        "external plugin {label} is not committed at the submitted revision"
    );
    anyhow::ensure!(
        output.stdout == fs::read(&canonical_path)?,
        "external plugin {label} working bytes differ from the submitted revision"
    );
    Ok(relative_text)
}

fn resolve_repository_member(root: &Path, relative: &str, label: &str) -> anyhow::Result<PathBuf> {
    validate_relative_path(relative)?;
    let canonical_root = fs::canonicalize(root)?;
    let path = root.join(relative);
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("inspect external plugin {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "external plugin {label} must be a regular non-symlink file"
    );
    let canonical = fs::canonicalize(&path)?;
    anyhow::ensure!(
        canonical.starts_with(canonical_root),
        "external plugin {label} escapes its evidence repository"
    );
    Ok(canonical)
}

fn validate_relative_path(path: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        !path.is_empty()
            && !path.contains('\\')
            && !path.chars().any(char::is_control)
            && !Path::new(path).is_absolute()
            && Path::new(path)
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "external plugin path must be a canonical relative path"
    );
    Ok(())
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

fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn is_lower_hex(byte: u8) -> bool {
    byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)
}

fn absolute_from(root: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn git(root: &Path, args: &[&str]) -> String {
        let output = Command::new("git")
            .current_dir(root)
            .args(args)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "git {}: {}",
            args.join(" "),
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8_lossy(&output.stdout).trim().to_string()
    }

    fn artifact(file_name: &str, bytes: &[u8]) -> Artifact {
        Artifact {
            url: format!("https://example.invalid/releases/{file_name}"),
            file_name: file_name.to_string(),
            size_bytes: u64::try_from(bytes.len()).unwrap(),
            sha256: sha256_bytes(bytes),
        }
    }

    fn candidate() -> SubmissionCandidate {
        SubmissionCandidate {
            kind: SUBMISSION_KIND.to_string(),
            schema_version: SUBMISSION_SCHEMA_VERSION,
            candidate_status: CANDIDATE_STATUS.to_string(),
            author_assistance: true,
            evidence_repository: Repository {
                owner: "external-owner".to_string(),
                url: "https://github.com/external-owner/controller".to_string(),
            },
            release: ReleaseIdentity {
                tag: format!("v{RELEASE_VERSION}"),
                target: "x86_64-pc-windows-msvc".to_string(),
                archive: artifact("rne-0.2.0-x86_64-pc-windows-msvc.zip", b"archive"),
            },
            platform: Platform {
                operating_system: "windows".to_string(),
                architecture: "x86_64".to_string(),
            },
            artifacts: PluginArtifacts {
                library: artifact("controller.dll", b"library"),
                manifest: artifact("rne-plugin.json", b"manifest"),
                report: artifact("controller-conformance.json", b"report"),
            },
            reproduction: Reproduction {
                commands: vec!["verify archive".to_string(), "plugin check".to_string()],
                exit_statuses: vec![0, 0],
                stdout_log_path: "logs/stdout.txt".to_string(),
                stderr_log_path: "logs/stderr.txt".to_string(),
            },
        }
    }

    #[test]
    fn candidate_is_acyclic_and_platform_bound() {
        let candidate = candidate();
        validate_candidate(&candidate, &"a".repeat(40)).unwrap();
        let mut value = serde_json::to_value(&candidate).unwrap();
        value["evidence_repository"]["revision"] = serde_json::json!("a".repeat(40));
        assert!(serde_json::from_value::<SubmissionCandidate>(value).is_err());
        let mut mismatched = candidate.clone();
        mismatched.platform.operating_system = "linux".to_string();
        assert!(validate_candidate(&mismatched, &"a".repeat(40)).is_err());
    }

    #[test]
    fn candidate_rejects_nonzero_status_and_mutable_url() {
        let mut nonzero = candidate();
        nonzero.reproduction.exit_statuses[1] = 1;
        assert!(validate_candidate(&nonzero, &"a".repeat(40)).is_err());
        let mut mutable = candidate();
        mutable.artifacts.library.url.push_str("?latest=true");
        assert!(validate_candidate(&mutable, &"a".repeat(40)).is_err());
    }

    #[test]
    fn options_require_every_exact_artifact() {
        let mut args = [
            "--release-archive",
            "release.zip",
            "--library",
            "controller.dll",
            "--manifest",
            "rne-plugin.json",
            "--report",
            "report.json",
            "--submission",
            "submission.json",
            "--evidence-repo-dir",
            "external-repo",
            "--revision",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--output",
            "verified.json",
        ]
        .into_iter()
        .map(str::to_string);
        let options = parse_options(&mut args).unwrap().unwrap();
        assert_eq!(options.library, PathBuf::from("controller.dll"));
        let mut legacy = ["--owner", "external-owner"]
            .into_iter()
            .map(str::to_string);
        assert!(parse_options(&mut legacy).is_err());
    }

    #[test]
    fn artifact_bytes_are_rehashed_and_bounded() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("controller.dll");
        fs::write(&path, b"library").unwrap();
        let expected = artifact("controller.dll", b"library");
        validate_artifact(&expected, &path, "library", MAX_EVIDENCE_BYTES).unwrap();
        fs::write(&path, b"tampered").unwrap();
        assert!(validate_artifact(&expected, &path, "library", MAX_EVIDENCE_BYTES).is_err());
    }

    #[test]
    fn full_submission_binds_clean_git_and_typed_plugin_subjects() {
        let directory = tempfile::tempdir().unwrap();
        let evidence_repo = directory.path().join("external-controller");
        let downloads = directory.path().join("downloads");
        fs::create_dir_all(evidence_repo.join("logs")).unwrap();
        fs::create_dir_all(&downloads).unwrap();

        let release_archive = downloads.join("rne-0.2.0-x86_64-pc-windows-msvc.zip");
        let library = downloads.join("external_controller.dll");
        let manifest = downloads.join("rne-plugin.json");
        let conformance = downloads.join("controller-conformance.json");
        fs::write(&release_archive, b"release archive").unwrap();
        fs::write(&library, b"external controller library").unwrap();
        let manifest_bytes = br#"{
  "name": "external_controller",
  "kind": "controller"
}"#;
        fs::write(&manifest, manifest_bytes).unwrap();
        let report_value = serde_json::json!({
            "schema_version": 1,
            "kind": "rne_controller_plugin_conformance_report",
            "status": "passed",
            "subject": {
                "library_file": "external_controller.dll",
                "library_sha256": sha256_bytes(b"external controller library"),
                "library_size_bytes": b"external controller library".len(),
                "manifest_file": "rne-plugin.json",
                "manifest_sha256": sha256_bytes(manifest_bytes)
            },
            "controller": {
                "name": "external_controller",
                "abi_version": 3,
                "controller_schema_version": 1,
                "capabilities": ["joint_position_observation", "joint_velocity_command"]
            },
            "checks": [
                {"id": "manifest_identity", "status": "passed", "detail": "ok"},
                {"id": "abi_symbols", "status": "passed", "detail": "ok"},
                {"id": "capability_negotiation", "status": "passed", "detail": "ok"},
                {"id": "fixed_step_schema", "status": "passed", "detail": "ok"},
                {"id": "reset_replay_exact", "status": "passed", "detail": "ok"},
                {"id": "lifecycle_shutdown", "status": "passed", "detail": "ok"}
            ]
        });
        fs::write(
            &conformance,
            serde_json::to_vec_pretty(&report_value).unwrap(),
        )
        .unwrap();

        let mut submission = candidate();
        submission.evidence_repository.url =
            "https://github.com/external-owner/controller".to_string();
        submission.release.archive = artifact(
            "rne-0.2.0-x86_64-pc-windows-msvc.zip",
            &fs::read(&release_archive).unwrap(),
        );
        submission.artifacts.library =
            artifact("external_controller.dll", &fs::read(&library).unwrap());
        submission.artifacts.manifest = artifact("rne-plugin.json", &fs::read(&manifest).unwrap());
        submission.artifacts.report = artifact(
            "controller-conformance.json",
            &fs::read(&conformance).unwrap(),
        );
        submission.reproduction.stdout_log_path = "logs/plugin-check.stdout.txt".to_string();
        submission.reproduction.stderr_log_path = "logs/plugin-check.stderr.txt".to_string();
        let submission_path = evidence_repo.join("external-plugin-submission.json");
        fs::write(
            &submission_path,
            serde_json::to_vec_pretty(&submission).unwrap(),
        )
        .unwrap();
        let stdout_log_path = evidence_repo.join("logs/plugin-check.stdout.txt");
        let stderr_log_path = evidence_repo.join("logs/plugin-check.stderr.txt");
        fs::write(&stdout_log_path, b"passed\n").unwrap();
        fs::write(&stderr_log_path, b"empty\n").unwrap();

        git(&evidence_repo, &["init"]);
        git(
            &evidence_repo,
            &["config", "user.email", "external@example.invalid"],
        );
        git(
            &evidence_repo,
            &["config", "user.name", "External Operator"],
        );
        git(
            &evidence_repo,
            &[
                "remote",
                "add",
                "origin",
                "https://github.com/external-owner/controller.git",
            ],
        );
        git(&evidence_repo, &["add", "."]);
        git(&evidence_repo, &["commit", "-m", "retain plugin evidence"]);
        let revision = git(&evidence_repo, &["rev-parse", "HEAD"]);
        let output = directory.path().join("verified.json");
        let run_once = || {
            let arguments = vec![
                "--release-archive".to_string(),
                release_archive.display().to_string(),
                "--library".to_string(),
                library.display().to_string(),
                "--manifest".to_string(),
                manifest.display().to_string(),
                "--report".to_string(),
                conformance.display().to_string(),
                "--submission".to_string(),
                submission_path.display().to_string(),
                "--evidence-repo-dir".to_string(),
                evidence_repo.display().to_string(),
                "--revision".to_string(),
                revision.clone(),
                "--output".to_string(),
                output.display().to_string(),
            ];
            run(&mut arguments.into_iter())
        };
        run_once().unwrap();
        let verified: serde_json::Value =
            serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
        assert_eq!(verified["status"], "passed");
        assert_eq!(verified["revision"], revision);
        assert_eq!(verified["controller_name"], "external_controller");
        let staged = || StagedSubmission {
            owner: "external-owner",
            repository: "https://github.com/external-owner/controller",
            revision: &revision,
            release_archive: &release_archive,
            library: &library,
            manifest: &manifest,
            conformance_report: &conformance,
            submission_candidate: &submission_path,
            stdout_log: &stdout_log_path,
            stderr_log: &stderr_log_path,
        };
        validate_staged_submission_report(&fs::read(&output).unwrap(), staged()).unwrap();

        fs::write(&library, b"tampered controller library").unwrap();
        assert!(run_once().is_err());
        assert!(validate_staged_submission_report(&fs::read(&output).unwrap(), staged()).is_err());
    }
}
