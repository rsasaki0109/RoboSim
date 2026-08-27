//! Verifies an independently owned simulator-adapter submission without executing it.

use super::external_submission::{
    absolute_from, digest_file, read_regular_file, release_archive_name, resolve_repository_member,
    sha256_bytes, validate_artifact, validate_artifact_shape, validate_committed_file,
    validate_external_repository, validate_platform, validate_relative_path,
    validate_repository_checkout, Artifact, MemberDigest, Platform, ReleaseIdentity, Repository,
    Reproduction, MAX_EVIDENCE_BYTES, MAX_LOG_BYTES, MAX_RELEASE_ARCHIVE_BYTES,
};
use super::{workspace_root, RELEASE_VERSION};
use anyhow::{bail, Context};
use rne_ai::TaskSpec;
use rne_hardware_gateway::{
    simulator::{
        conformance::{SimulatorAdapterConformanceIdentity, SimulatorAdapterConformanceReport},
        SimulatorRuntimeManifest,
    },
    GatewayConfig, HardwareGateway,
};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub(crate) const EXTERNAL_SIMULATOR_SUBMISSION_REPORT_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_KIND: &str = "rne_external_simulator_adapter_submission_candidate";
const REPORT_KIND: &str = "rne_external_simulator_adapter_submission_report";
const CANDIDATE_STATUS: &str = "not_accepted_pending_maintainer_verification";
const MAX_ADAPTER_ARGUMENTS: usize = 128;
const MAX_ADAPTER_ARGUMENT_BYTES: usize = 4096;

#[derive(Debug)]
struct Options {
    release_archive: PathBuf,
    adapter: PathBuf,
    task: PathBuf,
    runtime_manifest: PathBuf,
    runtime_artifacts: Vec<PathBuf>,
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
    artifacts: SimulatorArtifacts,
    adapter_arguments: Vec<String>,
    reproduction: Reproduction,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct SimulatorArtifacts {
    adapter: Artifact,
    task_spec: Artifact,
    runtime_manifest: Artifact,
    runtime_artifacts: Vec<Artifact>,
    report: Artifact,
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
    adapter_arguments: Vec<String>,
    adapter_identity: SimulatorAdapterConformanceIdentity,
    release_archive: MemberDigest,
    adapter: MemberDigest,
    task_spec: MemberDigest,
    runtime_manifest: MemberDigest,
    runtime_artifacts: Vec<MemberDigest>,
    conformance_report: MemberDigest,
    submission_candidate: MemberDigest,
    stdout_log: MemberDigest,
    stderr_log: MemberDigest,
}

/// Exact files and ownership staged for the release-readiness audit.
pub(crate) struct StagedSubmission<'a> {
    pub(crate) owner: &'a str,
    pub(crate) repository: &'a str,
    pub(crate) revision: &'a str,
    pub(crate) release_archive: &'a Path,
    pub(crate) adapter: &'a Path,
    pub(crate) task_spec: &'a Path,
    pub(crate) runtime_manifest: &'a Path,
    pub(crate) runtime_artifacts: &'a [PathBuf],
    pub(crate) conformance_report: &'a Path,
    pub(crate) submission_candidate: &'a Path,
    pub(crate) stdout_log: &'a Path,
    pub(crate) stderr_log: &'a Path,
    pub(crate) adapter_arguments: &'a [String],
}

/// Revalidates a maintainer report against every staged simulator artifact.
pub(crate) fn validate_staged_submission_report(
    report_bytes: &[u8],
    staged: StagedSubmission<'_>,
) -> anyhow::Result<()> {
    let report: SubmissionReport = serde_json::from_slice(report_bytes)
        .context("parse staged external simulator-adapter submission report")?;
    anyhow::ensure!(
        report.kind == REPORT_KIND
            && report.schema_version == EXTERNAL_SIMULATOR_SUBMISSION_REPORT_SCHEMA_VERSION
            && report.status == "passed",
        "staged external simulator-adapter submission report identity drifted"
    );
    validate_external_repository(staged.owner, staged.repository, staged.revision)?;
    anyhow::ensure!(
        report.owner == staged.owner
            && report.repository == staged.repository
            && report.revision == staged.revision
            && report.release_tag == format!("v{RELEASE_VERSION}"),
        "staged external simulator-adapter ownership, revision, or release differs from the maintainer report"
    );
    validate_platform(
        &Platform {
            operating_system: report.operating_system.clone(),
            architecture: report.architecture.clone(),
        },
        &report.release_target,
    )?;
    anyhow::ensure!(
        report.release_archive.path == release_archive_name(&report.release_target)?,
        "staged external simulator-adapter release archive name drifted"
    );
    verify_arguments(&report.adapter_arguments, staged.adapter_arguments, None)?;

    let primary = [
        (
            &report.release_archive,
            staged.release_archive,
            MAX_RELEASE_ARCHIVE_BYTES,
            "release archive",
        ),
        (
            &report.adapter,
            staged.adapter,
            MAX_EVIDENCE_BYTES,
            "adapter",
        ),
        (
            &report.task_spec,
            staged.task_spec,
            MAX_EVIDENCE_BYTES,
            "TaskSpec",
        ),
        (
            &report.runtime_manifest,
            staged.runtime_manifest,
            MAX_EVIDENCE_BYTES,
            "runtime manifest",
        ),
        (
            &report.conformance_report,
            staged.conformance_report,
            MAX_EVIDENCE_BYTES,
            "conformance report",
        ),
    ];
    for (expected, path, limit, label) in primary {
        verify_unlocated_member(expected, path, label, limit)?;
    }
    anyhow::ensure!(
        report.runtime_artifacts.len() == staged.runtime_artifacts.len(),
        "staged external simulator runtime artifact count drifted"
    );
    for (expected, path) in report
        .runtime_artifacts
        .iter()
        .zip(staged.runtime_artifacts)
    {
        verify_unlocated_member(expected, path, "runtime artifact", MAX_EVIDENCE_BYTES)?;
    }
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

    let typed = parse_and_bind(
        staged.adapter,
        staged.task_spec,
        staged.runtime_manifest,
        staged.runtime_artifacts,
        staged.conformance_report,
        staged.adapter_arguments,
    )?;
    anyhow::ensure!(
        typed == report.adapter_identity,
        "staged external simulator-adapter identity differs from the maintainer report"
    );
    Ok(())
}

/// Rehashes a clean external Git submission and emits a staging-ready report.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let Some(options) = parse_options(args)? else {
        return Ok(());
    };
    let release_archive = absolute_from(&root, &options.release_archive);
    let adapter = absolute_from(&root, &options.adapter);
    let task = absolute_from(&root, &options.task);
    let runtime_manifest = absolute_from(&root, &options.runtime_manifest);
    let runtime_artifacts: Vec<PathBuf> = options
        .runtime_artifacts
        .iter()
        .map(|path| absolute_from(&root, path))
        .collect();
    let conformance_report = absolute_from(&root, &options.report);
    let submission_path = absolute_from(&root, &options.submission);
    let evidence_repo_dir = absolute_from(&root, &options.evidence_repo_dir);
    let output = absolute_from(&root, &options.output);

    let submission_bytes =
        read_regular_file(&submission_path, "submission candidate", MAX_EVIDENCE_BYTES)?;
    let candidate: SubmissionCandidate = serde_json::from_slice(&submission_bytes)
        .context("parse external simulator-adapter submission candidate")?;
    validate_candidate(&candidate, &options.revision)?;
    validate_repository_checkout(
        &evidence_repo_dir,
        &candidate.evidence_repository.url,
        &options.revision,
    )?;
    let submission_relative =
        validate_committed_file(&evidence_repo_dir, &submission_path, "submission candidate")?;

    let release_digest = validate_artifact(
        &candidate.release.archive,
        &release_archive,
        "release archive",
        MAX_RELEASE_ARCHIVE_BYTES,
    )?;
    let adapter_digest = validate_artifact(
        &candidate.artifacts.adapter,
        &adapter,
        "adapter",
        MAX_EVIDENCE_BYTES,
    )?;
    let task_digest = validate_artifact(
        &candidate.artifacts.task_spec,
        &task,
        "TaskSpec",
        MAX_EVIDENCE_BYTES,
    )?;
    let runtime_digest = validate_artifact(
        &candidate.artifacts.runtime_manifest,
        &runtime_manifest,
        "runtime manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    anyhow::ensure!(
        runtime_artifacts.len() == candidate.artifacts.runtime_artifacts.len(),
        "external simulator submission runtime artifact count differs from the candidate"
    );
    let runtime_artifact_digests: Vec<MemberDigest> = candidate
        .artifacts
        .runtime_artifacts
        .iter()
        .zip(&runtime_artifacts)
        .map(|(expected, path)| {
            validate_artifact(expected, path, "runtime artifact", MAX_EVIDENCE_BYTES)
        })
        .collect::<anyhow::Result<_>>()?;
    let conformance_digest = validate_artifact(
        &candidate.artifacts.report,
        &conformance_report,
        "conformance report",
        MAX_EVIDENCE_BYTES,
    )?;

    let identity = parse_and_bind(
        &adapter,
        &task,
        &runtime_manifest,
        &runtime_artifacts,
        &conformance_report,
        &candidate.adapter_arguments,
    )?;

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

    let report = SubmissionReport {
        kind: REPORT_KIND.to_string(),
        schema_version: EXTERNAL_SIMULATOR_SUBMISSION_REPORT_SCHEMA_VERSION,
        status: "passed".to_string(),
        owner: candidate.evidence_repository.owner,
        repository: candidate.evidence_repository.url,
        revision: options.revision,
        author_assistance: candidate.author_assistance,
        release_tag: candidate.release.tag,
        release_target: candidate.release.target,
        operating_system: candidate.platform.operating_system,
        architecture: candidate.platform.architecture,
        adapter_arguments: candidate.adapter_arguments,
        adapter_identity: identity,
        release_archive: release_digest,
        adapter: adapter_digest,
        task_spec: task_digest,
        runtime_manifest: runtime_digest,
        runtime_artifacts: runtime_artifact_digests,
        conformance_report: conformance_digest,
        submission_candidate: MemberDigest {
            path: submission_relative,
            size_bytes: u64::try_from(submission_bytes.len()).unwrap_or(u64::MAX),
            sha256: sha256_bytes(&submission_bytes),
        },
        stdout_log,
        stderr_log,
    };
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "external simulator adapter verified: owner={} adapter={} simulator={} report={}",
        report.owner,
        report.adapter_identity.adapter_id,
        report.adapter_identity.simulator_id,
        output.display()
    );
    Ok(())
}

fn parse_and_bind(
    adapter_path: &Path,
    task_path: &Path,
    runtime_manifest_path: &Path,
    runtime_artifact_paths: &[PathBuf],
    report_path: &Path,
    arguments: &[String],
) -> anyhow::Result<SimulatorAdapterConformanceIdentity> {
    let adapter = digest_file(adapter_path, "adapter", MAX_EVIDENCE_BYTES)?;
    let task = digest_file(task_path, "TaskSpec", MAX_EVIDENCE_BYTES)?;
    let runtime_manifest = digest_file(
        runtime_manifest_path,
        "runtime manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    let task_spec: TaskSpec = serde_json::from_slice(&fs::read(task_path)?)?;
    task_spec.validate()?;
    let runtime: SimulatorRuntimeManifest =
        serde_json::from_slice(&fs::read(runtime_manifest_path)?)?;
    runtime.validate()?;
    let typed: SimulatorAdapterConformanceReport = serde_json::from_slice(&fs::read(report_path)?)?;
    typed.validate()?;
    anyhow::ensure!(
        typed.passed(),
        "submitted simulator adapter did not pass conformance"
    );
    anyhow::ensure!(
        typed.subject.adapter_file == adapter.path
            && typed.subject.adapter_size_bytes == adapter.size_bytes
            && typed.subject.adapter_sha256 == adapter.sha256
            && typed.subject.task_file == task.path
            && typed.subject.task_sha256 == task.sha256
            && typed.subject.runtime_manifest_file == runtime_manifest.path
            && typed.subject.runtime_manifest_size_bytes == runtime_manifest.size_bytes
            && typed.subject.runtime_manifest_sha256 == runtime_manifest.sha256,
        "simulator conformance report is not bound to the submitted adapter, TaskSpec, and runtime manifest"
    );
    verify_arguments(
        arguments,
        arguments,
        Some((
            &typed.subject.arguments_sha256,
            typed.subject.argument_count,
        )),
    )?;
    anyhow::ensure!(
        runtime_artifact_paths.len() == runtime.artifacts.len()
            && typed.subject.runtime_artifacts == runtime.artifacts,
        "simulator runtime artifact catalogs differ"
    );
    for (path, expected) in runtime_artifact_paths.iter().zip(&runtime.artifacts) {
        let actual = digest_file(path, "runtime artifact", MAX_EVIDENCE_BYTES)?;
        anyhow::ensure!(
            actual.path == expected.file
                && actual.size_bytes == expected.size_bytes
                && actual.sha256 == expected.sha256,
            "simulator runtime artifact bytes differ from the runtime manifest"
        );
    }
    let identity = typed
        .adapter
        .context("passing simulator report omitted identity")?;
    let gateway = HardwareGateway::new(task_spec.clone(), GatewayConfig::default())?;
    let fixed_delta_ticks = (task_spec.control_step_s * 1_000_000_000.0).round() as u64;
    anyhow::ensure!(
        identity.task_id == task_spec.task_id
            && identity.observation_width == gateway.observation_width()
            && identity.action_width == gateway.action_width()
            && identity.fixed_delta_ticks == fixed_delta_ticks
            && runtime.fixed_delta_ticks == fixed_delta_ticks
            && identity.simulator_id == runtime.simulator_id
            && identity.simulator_version == runtime.simulator_version,
        "simulator handshake identity differs from retained TaskSpec or runtime manifest"
    );
    Ok(identity)
}

fn verify_arguments(
    report_arguments: &[String],
    staged_arguments: &[String],
    subject: Option<(&str, usize)>,
) -> anyhow::Result<()> {
    anyhow::ensure!(
        report_arguments == staged_arguments && report_arguments.len() <= MAX_ADAPTER_ARGUMENTS,
        "external simulator launch arguments drifted or exceed the bound"
    );
    for (index, argument) in report_arguments.iter().enumerate() {
        anyhow::ensure!(
            argument.len() <= MAX_ADAPTER_ARGUMENT_BYTES && !argument.chars().any(char::is_control),
            "external simulator argument {index} is not bounded printable text"
        );
    }
    if let Some((expected_sha256, expected_count)) = subject {
        let digest = sha256_bytes(&serde_json::to_vec(report_arguments)?);
        anyhow::ensure!(
            expected_count == report_arguments.len() && expected_sha256 == digest,
            "external simulator launch arguments differ from the conformance report"
        );
    }
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
        "staged external simulator-adapter {label} differs from the maintainer report"
    );
    Ok(())
}

fn parse_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<Option<Options>> {
    let mut options = Options {
        release_archive: PathBuf::new(),
        adapter: PathBuf::new(),
        task: PathBuf::new(),
        runtime_manifest: PathBuf::new(),
        runtime_artifacts: Vec::new(),
        report: PathBuf::new(),
        submission: PathBuf::new(),
        evidence_repo_dir: PathBuf::new(),
        revision: String::new(),
        output: PathBuf::new(),
    };
    while let Some(argument) = args.next() {
        let value = |args: &mut dyn Iterator<Item = String>| {
            args.next()
                .with_context(|| format!("{argument} requires a value"))
        };
        match argument.as_str() {
            "--release-archive" => options.release_archive = value(args)?.into(),
            "--adapter" => options.adapter = value(args)?.into(),
            "--task" => options.task = value(args)?.into(),
            "--runtime-manifest" => options.runtime_manifest = value(args)?.into(),
            "--runtime-artifact" => options.runtime_artifacts.push(value(args)?.into()),
            "--report" => options.report = value(args)?.into(),
            "--submission" => options.submission = value(args)?.into(),
            "--evidence-repo-dir" => options.evidence_repo_dir = value(args)?.into(),
            "--revision" => options.revision = value(args)?,
            "--output" => options.output = value(args)?.into(),
            "--help" | "-h" => {
                println!("external-simulator-check --release-archive PATH --adapter PATH --task PATH --runtime-manifest PATH --runtime-artifact PATH (three times) --report PATH --submission PATH --evidence-repo-dir PATH --revision SHA --output PATH");
                return Ok(None);
            }
            other => bail!("unknown external-simulator-check argument: {other}"),
        }
    }
    anyhow::ensure!(
        !options.release_archive.as_os_str().is_empty()
            && !options.adapter.as_os_str().is_empty()
            && !options.task.as_os_str().is_empty()
            && !options.runtime_manifest.as_os_str().is_empty()
            && options.runtime_artifacts.len() == 3
            && !options.report.as_os_str().is_empty()
            && !options.submission.as_os_str().is_empty()
            && !options.evidence_repo_dir.as_os_str().is_empty()
            && !options.revision.is_empty()
            && !options.output.as_os_str().is_empty(),
        "external-simulator-check requires every exact artifact, exactly three --runtime-artifact values, repository revision, and output"
    );
    Ok(Some(options))
}

fn validate_candidate(candidate: &SubmissionCandidate, revision: &str) -> anyhow::Result<()> {
    anyhow::ensure!(
        candidate.kind == SUBMISSION_KIND
            && candidate.schema_version == SUBMISSION_SCHEMA_VERSION
            && candidate.candidate_status == CANDIDATE_STATUS,
        "external simulator submission identity or non-acceptance boundary drifted"
    );
    validate_external_repository(
        &candidate.evidence_repository.owner,
        &candidate.evidence_repository.url,
        revision,
    )?;
    anyhow::ensure!(
        candidate.release.tag == format!("v{RELEASE_VERSION}")
            && candidate.release.archive.file_name
                == release_archive_name(&candidate.release.target)?,
        "external simulator submission names the wrong official release artifact"
    );
    validate_platform(&candidate.platform, &candidate.release.target)?;
    anyhow::ensure!(
        candidate.artifacts.runtime_artifacts.len() == 3,
        "external simulator submission must retain exactly three runtime artifacts"
    );
    validate_artifact_shape(&candidate.release.archive, "release archive")?;
    validate_artifact_shape(&candidate.artifacts.adapter, "adapter")?;
    validate_artifact_shape(&candidate.artifacts.task_spec, "TaskSpec")?;
    validate_artifact_shape(&candidate.artifacts.runtime_manifest, "runtime manifest")?;
    for artifact in &candidate.artifacts.runtime_artifacts {
        validate_artifact_shape(artifact, "runtime artifact")?;
    }
    validate_artifact_shape(&candidate.artifacts.report, "conformance report")?;
    verify_arguments(
        &candidate.adapter_arguments,
        &candidate.adapter_arguments,
        None,
    )?;
    anyhow::ensure!(
        candidate.reproduction.commands.len() >= 2
            && candidate.reproduction.commands.len() == candidate.reproduction.exit_statuses.len()
            && candidate.reproduction.commands.iter().all(|command| {
                !command.trim().is_empty()
                    && command.len() <= 4096
                    && !command.chars().any(char::is_control)
            })
            && candidate.reproduction.exit_statuses.iter().all(|status| *status == 0)
            && candidate.reproduction.stdout_log_path != candidate.reproduction.stderr_log_path,
        "external simulator submission must retain distinct logs and at least two successful bounded commands"
    );
    validate_relative_path(&candidate.reproduction.stdout_log_path)?;
    validate_relative_path(&candidate.reproduction.stderr_log_path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(file_name: &str) -> Artifact {
        Artifact {
            url: format!("https://example.invalid/releases/{file_name}"),
            file_name: file_name.to_string(),
            size_bytes: 1,
            sha256: "a".repeat(64),
        }
    }

    fn candidate() -> SubmissionCandidate {
        SubmissionCandidate {
            kind: SUBMISSION_KIND.to_string(),
            schema_version: 1,
            candidate_status: CANDIDATE_STATUS.to_string(),
            author_assistance: false,
            evidence_repository: Repository {
                owner: "external-owner".to_string(),
                url: "https://github.com/external-owner/gazebo-adapter".to_string(),
            },
            release: ReleaseIdentity {
                tag: format!("v{RELEASE_VERSION}"),
                target: "x86_64-unknown-linux-gnu".to_string(),
                archive: artifact("rne-0.2.0-x86_64-unknown-linux-gnu.tar.gz"),
            },
            platform: Platform {
                operating_system: "linux".to_string(),
                architecture: "x86_64".to_string(),
            },
            artifacts: SimulatorArtifacts {
                adapter: artifact("adapter.py"),
                task_spec: artifact("task.json"),
                runtime_manifest: artifact("runtime.json"),
                runtime_artifacts: vec![
                    artifact("world.sdf"),
                    artifact("robot.sdf"),
                    artifact("adapter.json"),
                ],
                report: artifact("simulator-conformance.json"),
            },
            adapter_arguments: vec!["<runtime-manifest>".to_string()],
            reproduction: Reproduction {
                commands: vec!["verify release".to_string(), "run conformance".to_string()],
                exit_statuses: vec![0, 0],
                stdout_log_path: "logs/stdout.txt".to_string(),
                stderr_log_path: "logs/stderr.txt".to_string(),
            },
        }
    }

    #[test]
    fn candidate_is_acyclic_platform_bound_and_complete() {
        validate_candidate(&candidate(), &"a".repeat(40)).unwrap();
        let mut value = serde_json::to_value(candidate()).unwrap();
        value["evidence_repository"]["revision"] = serde_json::json!("a".repeat(40));
        assert!(serde_json::from_value::<SubmissionCandidate>(value).is_err());
        let mut missing = candidate();
        missing.artifacts.runtime_artifacts.pop();
        assert!(validate_candidate(&missing, &"a".repeat(40)).is_err());
    }

    #[test]
    fn options_require_three_ordered_runtime_artifacts() {
        let mut args = [
            "--release-archive",
            "rne.tar.gz",
            "--adapter",
            "adapter.py",
            "--task",
            "task.json",
            "--runtime-manifest",
            "runtime.json",
            "--runtime-artifact",
            "world.sdf",
            "--runtime-artifact",
            "robot.sdf",
            "--runtime-artifact",
            "adapter.json",
            "--report",
            "report.json",
            "--submission",
            "submission.json",
            "--evidence-repo-dir",
            "repo",
            "--revision",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--output",
            "verified.json",
        ]
        .into_iter()
        .map(str::to_string);
        assert_eq!(
            parse_options(&mut args)
                .unwrap()
                .unwrap()
                .runtime_artifacts
                .len(),
            3
        );
    }

    #[test]
    fn normalized_argument_hash_is_exact_and_bounded() {
        let arguments = vec!["<runtime-manifest>".to_string()];
        let digest = sha256_bytes(&serde_json::to_vec(&arguments).unwrap());
        verify_arguments(&arguments, &arguments, Some((&digest, 1))).unwrap();
        assert!(verify_arguments(&arguments, &[], Some((&digest, 1))).is_err());
    }
}
