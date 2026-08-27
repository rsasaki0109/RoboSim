//! Verifies an independently owned external-project submission without executing it.

use super::external_submission::{
    absolute_from, digest_file, read_regular_file, release_archive_name, resolve_repository_member,
    sha256_bytes, validate_artifact, validate_artifact_shape, validate_committed_file,
    validate_external_repository, validate_platform, validate_relative_path,
    validate_repository_checkout, MemberDigest, Platform, ReleaseIdentity, Repository,
    Reproduction, MAX_EVIDENCE_BYTES, MAX_LOG_BYTES, MAX_RELEASE_ARCHIVE_BYTES,
};
use super::{failure_capsule, workspace_root, RELEASE_VERSION};
use anyhow::{bail, Context};
use rne_ai::TaskSpec;
use rne_log::FailureCapsule;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub(crate) const EXTERNAL_PROJECT_SUBMISSION_REPORT_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_SCHEMA_VERSION: u32 = 1;
const SUBMISSION_KIND: &str = "rne_external_project_submission_candidate";
const REPORT_KIND: &str = "rne_external_project_submission_report";
const CANDIDATE_STATUS: &str = "not_accepted_pending_maintainer_verification";
const MAX_CAPSULE_ARTIFACTS: usize = 512;
const MAX_CAPSULE_TOTAL_BYTES: u64 = 2 * 1024 * 1024 * 1024;

#[derive(Debug)]
struct Options {
    release_archive: PathBuf,
    task: PathBuf,
    capsule_dir: PathBuf,
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
    usage: Usage,
    project: ProjectArtifacts,
    reproduction: Reproduction,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct Usage {
    first_used_on: String,
    last_verified_on: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct ProjectArtifacts {
    task_spec_path: String,
    failure_capsule_path: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
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
    first_used_on: String,
    last_verified_on: String,
    release_archive: MemberDigest,
    task_spec: MemberDigest,
    failure_capsule: MemberDigest,
    capsule_artifacts: Vec<MemberDigest>,
    submission_candidate: MemberDigest,
    stdout_log: MemberDigest,
    stderr_log: MemberDigest,
}

/// Exact staged files and ownership expected by the 1.0 readiness audit.
pub(crate) struct StagedSubmission<'a> {
    pub(crate) owner: &'a str,
    pub(crate) repository: &'a str,
    pub(crate) revision: &'a str,
    pub(crate) first_used_on: &'a str,
    pub(crate) last_verified_on: &'a str,
    pub(crate) release_archive: &'a Path,
    pub(crate) task_spec: &'a Path,
    pub(crate) failure_capsule: &'a Path,
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
        .context("parse staged external-project submission report")?;
    anyhow::ensure!(
        report.kind == REPORT_KIND
            && report.schema_version == EXTERNAL_PROJECT_SUBMISSION_REPORT_SCHEMA_VERSION
            && report.status == "passed"
            && !report.author_assistance,
        "staged external-project submission report identity drifted"
    );
    validate_external_repository(staged.owner, staged.repository, staged.revision)?;
    anyhow::ensure!(
        report.owner == staged.owner
            && report.repository == staged.repository
            && report.revision == staged.revision
            && report.first_used_on == staged.first_used_on
            && report.last_verified_on == staged.last_verified_on,
        "staged external-project ownership, revision, or usage dates differ from the maintainer report"
    );
    anyhow::ensure!(
        report.release_tag == format!("v{RELEASE_VERSION}"),
        "staged external-project report names the wrong RNE release"
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
        "staged external-project release archive name drifted"
    );

    let release_archive = digest_file(
        staged.release_archive,
        "staged release archive",
        MAX_RELEASE_ARCHIVE_BYTES,
    )?;
    let task_spec = digest_file(staged.task_spec, "staged TaskSpec", MAX_EVIDENCE_BYTES)?;
    let failure_capsule = digest_file(
        staged.failure_capsule,
        "staged Failure Capsule manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    anyhow::ensure!(
        report.release_archive == release_archive
            && report.task_spec == task_spec
            && report.failure_capsule == failure_capsule,
        "staged external-project primary artifacts differ from the maintainer report"
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

    validate_project_subjects(
        staged.task_spec,
        staged.failure_capsule,
        Some(&report.capsule_artifacts),
    )?;
    Ok(())
}

/// Verifies exact external-project submission bytes and emits a staging-ready report.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let Some(options) = parse_options(args)? else {
        return Ok(());
    };
    let release_archive = absolute_from(&root, &options.release_archive);
    let task = absolute_from(&root, &options.task);
    let capsule_dir = absolute_from(&root, &options.capsule_dir);
    let failure_capsule = capsule_dir.join("capsule.json");
    let submission_path = absolute_from(&root, &options.submission);
    let evidence_repo_dir = absolute_from(&root, &options.evidence_repo_dir);
    let output = absolute_from(&root, &options.output);

    let submission_bytes =
        read_regular_file(&submission_path, "submission candidate", MAX_EVIDENCE_BYTES)?;
    let candidate: SubmissionCandidate = serde_json::from_slice(&submission_bytes)
        .context("parse external-project submission candidate")?;
    validate_candidate(&candidate, &options.revision)?;
    validate_repository_checkout(
        &evidence_repo_dir,
        &candidate.evidence_repository.url,
        &options.revision,
    )?;

    let expected_task = resolve_repository_member(
        &evidence_repo_dir,
        &candidate.project.task_spec_path,
        "TaskSpec",
    )?;
    let expected_capsule = resolve_repository_member(
        &evidence_repo_dir,
        &candidate.project.failure_capsule_path,
        "Failure Capsule manifest",
    )?;
    anyhow::ensure!(
        fs::canonicalize(&task)? == expected_task,
        "external-project --task differs from the candidate repository path"
    );
    anyhow::ensure!(
        fs::canonicalize(&failure_capsule)? == expected_capsule,
        "external-project --capsule-dir differs from the candidate repository path"
    );

    let submission_relative =
        validate_committed_file(&evidence_repo_dir, &submission_path, "submission candidate")?;
    validate_committed_file(&evidence_repo_dir, &task, "TaskSpec")?;
    validate_committed_file(
        &evidence_repo_dir,
        &failure_capsule,
        "Failure Capsule manifest",
    )?;

    let archive_digest = validate_artifact(
        &candidate.release.archive,
        &release_archive,
        "release archive",
        MAX_RELEASE_ARCHIVE_BYTES,
    )?;
    let task_digest = digest_file(&task, "TaskSpec", MAX_EVIDENCE_BYTES)?;
    let capsule_digest = digest_file(
        &failure_capsule,
        "Failure Capsule manifest",
        MAX_EVIDENCE_BYTES,
    )?;
    let capsule_artifacts = validate_project_subjects(&task, &failure_capsule, None)?;
    for artifact in &capsule_artifacts {
        let path = capsule_dir.join(&artifact.path);
        validate_committed_file(&evidence_repo_dir, &path, "Failure Capsule artifact")?;
    }

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
        schema_version: EXTERNAL_PROJECT_SUBMISSION_REPORT_SCHEMA_VERSION,
        status: "passed".to_string(),
        owner: candidate.evidence_repository.owner,
        repository: candidate.evidence_repository.url,
        revision: options.revision,
        author_assistance: candidate.author_assistance,
        release_tag: candidate.release.tag,
        release_target: candidate.release.target,
        operating_system: candidate.platform.operating_system,
        architecture: candidate.platform.architecture,
        first_used_on: candidate.usage.first_used_on,
        last_verified_on: candidate.usage.last_verified_on,
        release_archive: archive_digest,
        task_spec: task_digest,
        failure_capsule: capsule_digest,
        capsule_artifacts,
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
        "external project verified: owner={} task={} capsule_artifacts={} report={}",
        report.owner,
        report.task_spec.path,
        report.capsule_artifacts.len(),
        output.display()
    );
    Ok(())
}

fn validate_project_subjects(
    task_path: &Path,
    capsule_path: &Path,
    expected_artifacts: Option<&[MemberDigest]>,
) -> anyhow::Result<Vec<MemberDigest>> {
    let task_bytes = read_regular_file(task_path, "TaskSpec", MAX_EVIDENCE_BYTES)?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes).context("parse submitted TaskSpec")?;
    task.validate().context("validate submitted TaskSpec")?;
    let task_sha256 = sha256_bytes(&task_bytes);

    let capsule_bytes =
        read_regular_file(capsule_path, "Failure Capsule manifest", MAX_EVIDENCE_BYTES)?;
    let capsule: FailureCapsule =
        serde_json::from_slice(&capsule_bytes).context("parse submitted Failure Capsule")?;
    capsule
        .validate()
        .context("validate submitted Failure Capsule")?;
    let capsule_dir = capsule_path
        .parent()
        .context("Failure Capsule manifest has no parent directory")?;
    failure_capsule::verify_directory(capsule_dir)
        .context("verify submitted Failure Capsule directory")?;
    anyhow::ensure!(
        capsule.artifacts.len() <= MAX_CAPSULE_ARTIFACTS,
        "Failure Capsule exceeds the {MAX_CAPSULE_ARTIFACTS}-artifact intake limit"
    );

    let task_refs = capsule
        .artifacts
        .iter()
        .filter(|artifact| artifact.kind == "rne_task_spec" && artifact.sha256 == task_sha256)
        .count();
    anyhow::ensure!(
        task_refs == 1,
        "Failure Capsule must bind the submitted TaskSpec exactly once"
    );

    let artifacts = capsule
        .artifacts
        .iter()
        .map(|artifact| {
            let path = capsule_dir.join(&artifact.path);
            let mut digest = digest_file(&path, "Failure Capsule artifact", MAX_EVIDENCE_BYTES)?;
            digest.path.clone_from(&artifact.path);
            anyhow::ensure!(
                digest.sha256 == artifact.sha256,
                "Failure Capsule artifact digest drifted for {}",
                artifact.path
            );
            Ok(digest)
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let total_bytes = artifacts.iter().try_fold(0_u64, |total, artifact| {
        total.checked_add(artifact.size_bytes)
    });
    anyhow::ensure!(
        total_bytes.is_some_and(|total| total <= MAX_CAPSULE_TOTAL_BYTES),
        "Failure Capsule exceeds the {MAX_CAPSULE_TOTAL_BYTES}-byte aggregate intake limit"
    );
    if let Some(expected) = expected_artifacts {
        anyhow::ensure!(
            artifacts == expected,
            "staged Failure Capsule members differ from the maintainer report"
        );
    }
    Ok(artifacts)
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
        "staged external-project {label} differs from the maintainer report"
    );
    Ok(())
}

fn parse_options(args: &mut impl Iterator<Item = String>) -> anyhow::Result<Option<Options>> {
    let mut release_archive = None;
    let mut task = None;
    let mut capsule_dir = None;
    let mut submission = None;
    let mut evidence_repo_dir = None;
    let mut revision = None;
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--release-archive" => release_archive = Some(path_arg(args, &argument)?),
            "--task" => task = Some(path_arg(args, &argument)?),
            "--capsule-dir" => capsule_dir = Some(path_arg(args, &argument)?),
            "--submission" => submission = Some(path_arg(args, &argument)?),
            "--evidence-repo-dir" => evidence_repo_dir = Some(path_arg(args, &argument)?),
            "--revision" => revision = Some(string_arg(args, &argument)?),
            "--output" => output = Some(path_arg(args, &argument)?),
            "--help" | "-h" => {
                println!("external-project-check --release-archive PATH --task PATH --capsule-dir PATH --submission PATH --evidence-repo-dir PATH --revision SHA --output PATH");
                return Ok(None);
            }
            other => bail!("unknown external-project-check argument: {other}"),
        }
    }
    Ok(Some(Options {
        release_archive: release_archive
            .context("external-project-check requires --release-archive PATH")?,
        task: task.context("external-project-check requires --task PATH")?,
        capsule_dir: capsule_dir.context("external-project-check requires --capsule-dir PATH")?,
        submission: submission.context("external-project-check requires --submission PATH")?,
        evidence_repo_dir: evidence_repo_dir
            .context("external-project-check requires --evidence-repo-dir PATH")?,
        revision: revision.context("external-project-check requires --revision SHA")?,
        output: output.context("external-project-check requires --output PATH")?,
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
            && candidate.candidate_status == CANDIDATE_STATUS
            && !candidate.author_assistance,
        "external-project submission identity or independence boundary drifted"
    );
    validate_external_repository(
        &candidate.evidence_repository.owner,
        &candidate.evidence_repository.url,
        revision,
    )?;
    anyhow::ensure!(
        candidate.release.tag == format!("v{RELEASE_VERSION}"),
        "external-project submission must name release v{RELEASE_VERSION}"
    );
    validate_platform(&candidate.platform, &candidate.release.target)?;
    anyhow::ensure!(
        candidate.release.archive.file_name == release_archive_name(&candidate.release.target)?,
        "external-project release archive name does not match the official target artifact"
    );
    validate_artifact_shape(&candidate.release.archive, "release archive")?;
    let official_archive_url = format!(
        "https://github.com/rsasaki0109/RoboSim/releases/download/v{RELEASE_VERSION}/{}",
        candidate.release.archive.file_name
    );
    anyhow::ensure!(
        candidate.release.archive.url == official_archive_url,
        "external-project submission must use the official immutable RNE release URL"
    );
    validate_date(&candidate.usage.first_used_on)?;
    validate_date(&candidate.usage.last_verified_on)?;
    anyhow::ensure!(
        candidate.usage.first_used_on <= candidate.usage.last_verified_on,
        "external-project first-use date is after its verification date"
    );
    for path in [
        &candidate.project.task_spec_path,
        &candidate.project.failure_capsule_path,
        &candidate.reproduction.stdout_log_path,
        &candidate.reproduction.stderr_log_path,
    ] {
        validate_relative_path(path)?;
    }
    anyhow::ensure!(
        candidate.project.task_spec_path.ends_with(".json")
            && candidate
                .project
                .failure_capsule_path
                .ends_with("/capsule.json")
            && candidate.project.task_spec_path != candidate.project.failure_capsule_path,
        "external-project candidate must name distinct TaskSpec JSON and Failure Capsule paths"
    );
    anyhow::ensure!(
        candidate.reproduction.commands.len() >= 3
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
        "external-project submission must retain at least three successful commands and matching zero exit statuses"
    );
    anyhow::ensure!(
        candidate.reproduction.stdout_log_path != candidate.reproduction.stderr_log_path,
        "external-project stdout and stderr logs must be distinct"
    );
    Ok(())
}

fn validate_date(value: &str) -> anyhow::Result<()> {
    let bytes = value.as_bytes();
    anyhow::ensure!(
        bytes.len() == 10
            && bytes[4] == b'-'
            && bytes[7] == b'-'
            && bytes
                .iter()
                .enumerate()
                .all(|(index, byte)| index == 4 || index == 7 || byte.is_ascii_digit()),
        "external-project dates must use YYYY-MM-DD"
    );
    let year = value[0..4].parse::<u32>()?;
    let month = value[5..7].parse::<u32>()?;
    let day = value[8..10].parse::<u32>()?;
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let max_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => 0,
    };
    anyhow::ensure!(
        year >= 2020 && day > 0 && day <= max_day,
        "invalid external-project date"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::external_submission::{sha256_bytes, Artifact};
    use super::*;
    use rne_core::{DeterminismContract, DeterminismScope};
    use rne_log::{ArtifactRef, BackendMetadata, BuildMetadata, FailureMetadata, RunMetadata};
    use std::process::Command;

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

    fn candidate() -> SubmissionCandidate {
        SubmissionCandidate {
            kind: SUBMISSION_KIND.to_string(),
            schema_version: SUBMISSION_SCHEMA_VERSION,
            candidate_status: CANDIDATE_STATUS.to_string(),
            author_assistance: false,
            evidence_repository: Repository {
                owner: "external-owner".to_string(),
                url: "https://github.com/external-owner/project".to_string(),
            },
            release: ReleaseIdentity {
                tag: format!("v{RELEASE_VERSION}"),
                target: "x86_64-pc-windows-msvc".to_string(),
                archive: Artifact {
                    url: "https://github.com/rsasaki0109/RoboSim/releases/download/v0.2.0/rne-0.2.0-x86_64-pc-windows-msvc.zip".to_string(),
                    file_name: "rne-0.2.0-x86_64-pc-windows-msvc.zip".to_string(),
                    size_bytes: 7,
                    sha256: sha256_bytes(b"archive"),
                },
            },
            platform: Platform {
                operating_system: "windows".to_string(),
                architecture: "x86_64".to_string(),
            },
            usage: Usage {
                first_used_on: "2026-08-01".to_string(),
                last_verified_on: "2026-08-28".to_string(),
            },
            project: ProjectArtifacts {
                task_spec_path: "evidence/task.json".to_string(),
                failure_capsule_path: "evidence/failure-capsule/capsule.json".to_string(),
            },
            reproduction: Reproduction {
                commands: vec![
                    "verify release".to_string(),
                    "run task".to_string(),
                    "verify capsule".to_string(),
                ],
                exit_statuses: vec![0, 0, 0],
                stdout_log_path: "logs/reproduction.stdout.txt".to_string(),
                stderr_log_path: "logs/reproduction.stderr.txt".to_string(),
            },
        }
    }

    #[test]
    fn candidate_is_acyclic_release_bound_and_independent() {
        let candidate = candidate();
        validate_candidate(&candidate, &"a".repeat(40)).unwrap();
        let mut value = serde_json::to_value(&candidate).unwrap();
        value["evidence_repository"]["revision"] = serde_json::json!("a".repeat(40));
        assert!(serde_json::from_value::<SubmissionCandidate>(value).is_err());
        let mut assisted = candidate.clone();
        assisted.author_assistance = true;
        assert!(validate_candidate(&assisted, &"a".repeat(40)).is_err());
    }

    #[test]
    fn candidate_rejects_bad_dates_and_nonzero_commands() {
        let mut bad_date = candidate();
        bad_date.usage.last_verified_on = "2026-02-30".to_string();
        assert!(validate_candidate(&bad_date, &"a".repeat(40)).is_err());
        let mut nonzero = candidate();
        nonzero.reproduction.exit_statuses[2] = 1;
        assert!(validate_candidate(&nonzero, &"a".repeat(40)).is_err());
        let mut unofficial = candidate();
        unofficial.release.archive.url =
            "https://example.invalid/rne-0.2.0-x86_64-pc-windows-msvc.zip".to_string();
        assert!(validate_candidate(&unofficial, &"a".repeat(40)).is_err());
    }

    #[test]
    fn options_require_the_full_capsule_directory() {
        let mut args = [
            "--release-archive",
            "release.zip",
            "--task",
            "task.json",
            "--capsule-dir",
            "capsule",
            "--submission",
            "submission.json",
            "--evidence-repo-dir",
            "repo",
            "--revision",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--output",
            "report.json",
        ]
        .into_iter()
        .map(str::to_string);
        assert!(parse_options(&mut args).unwrap().is_some());
        let mut incomplete = ["--task", "task.json"].into_iter().map(str::to_string);
        assert!(parse_options(&mut incomplete).is_err());
    }

    #[test]
    fn full_submission_binds_clean_git_release_task_and_every_capsule_member() {
        let directory = tempfile::tempdir().unwrap();
        let evidence_repo = directory.path().join("external-project");
        let capsule_dir = evidence_repo.join("evidence/failure-capsule");
        let task_path = capsule_dir.join("evidence/task.json");
        let replay_path = capsule_dir.join("replay.bin");
        let logs = evidence_repo.join("logs");
        fs::create_dir_all(task_path.parent().unwrap()).unwrap();
        fs::create_dir_all(&logs).unwrap();

        let task_bytes = include_bytes!("../../tests/golden/tasks/task-spec-v1.json");
        let replay_bytes = b"opaque deterministic replay fixture";
        fs::write(&task_path, task_bytes).unwrap();
        fs::write(&replay_path, replay_bytes).unwrap();
        let contract = DeterminismContract::exact(
            "external.failure",
            DeterminismScope::new("external.task", ["world.state"], 0, 3).unwrap(),
        )
        .unwrap();
        let capsule = FailureCapsule::new(
            FailureMetadata::new(
                "failure-1",
                "external.failure",
                "expected failure",
                2,
                30,
                7,
            ),
            RunMetadata::new("run-1", "external.task", 9, 10, 3, 2),
            BuildMetadata::new(
                "0.2.0",
                "0123456789abcdef",
                "release",
                "x86_64-pc-windows-msvc",
                "rustc 1.88.0",
                "c".repeat(64),
            ),
            BackendMetadata::new("rapier", "0.22"),
            contract,
            vec![
                ArtifactRef::new(
                    "evidence",
                    "rne_task_spec",
                    1,
                    "evidence/task.json",
                    sha256_bytes(task_bytes),
                )
                .unwrap(),
                ArtifactRef::new(
                    "replay",
                    "opaque_replay",
                    1,
                    "replay.bin",
                    sha256_bytes(replay_bytes),
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let capsule_path = capsule_dir.join("capsule.json");
        fs::write(&capsule_path, serde_json::to_vec_pretty(&capsule).unwrap()).unwrap();
        fs::write(
            logs.join("reproduction.stdout.txt"),
            b"all commands passed\n",
        )
        .unwrap();
        fs::write(logs.join("reproduction.stderr.txt"), b"no errors\n").unwrap();

        let mut candidate = candidate();
        candidate.project.task_spec_path =
            "evidence/failure-capsule/evidence/task.json".to_string();
        candidate.project.failure_capsule_path =
            "evidence/failure-capsule/capsule.json".to_string();
        let submission_path = evidence_repo.join("external-project-submission.json");
        fs::write(
            &submission_path,
            serde_json::to_vec_pretty(&candidate).unwrap(),
        )
        .unwrap();

        git(&evidence_repo, &["init"]);
        git(
            &evidence_repo,
            &["config", "user.email", "external@example.invalid"],
        );
        git(&evidence_repo, &["config", "user.name", "External Owner"]);
        git(
            &evidence_repo,
            &[
                "remote",
                "add",
                "origin",
                &candidate.evidence_repository.url,
            ],
        );
        git(&evidence_repo, &["add", "."]);
        git(
            &evidence_repo,
            &["commit", "-m", "retain independent evidence"],
        );
        let revision = git(&evidence_repo, &["rev-parse", "HEAD"]);

        let release_archive = directory
            .path()
            .join("rne-0.2.0-x86_64-pc-windows-msvc.zip");
        fs::write(&release_archive, b"archive").unwrap();
        let output = directory.path().join("maintainer-report.json");
        let mut args = vec![
            "--release-archive".to_string(),
            release_archive.to_string_lossy().into_owned(),
            "--task".to_string(),
            task_path.to_string_lossy().into_owned(),
            "--capsule-dir".to_string(),
            capsule_dir.to_string_lossy().into_owned(),
            "--submission".to_string(),
            submission_path.to_string_lossy().into_owned(),
            "--evidence-repo-dir".to_string(),
            evidence_repo.to_string_lossy().into_owned(),
            "--revision".to_string(),
            revision.clone(),
            "--output".to_string(),
            output.to_string_lossy().into_owned(),
        ]
        .into_iter();
        run(&mut args).unwrap();

        let report_bytes = fs::read(&output).unwrap();
        validate_staged_submission_report(
            &report_bytes,
            StagedSubmission {
                owner: &candidate.evidence_repository.owner,
                repository: &candidate.evidence_repository.url,
                revision: &revision,
                first_used_on: &candidate.usage.first_used_on,
                last_verified_on: &candidate.usage.last_verified_on,
                release_archive: &release_archive,
                task_spec: &task_path,
                failure_capsule: &capsule_path,
                submission_candidate: &submission_path,
                stdout_log: &logs.join("reproduction.stdout.txt"),
                stderr_log: &logs.join("reproduction.stderr.txt"),
            },
        )
        .unwrap();

        fs::write(&replay_path, b"tampered replay").unwrap();
        assert!(validate_staged_submission_report(
            &report_bytes,
            StagedSubmission {
                owner: &candidate.evidence_repository.owner,
                repository: &candidate.evidence_repository.url,
                revision: &revision,
                first_used_on: &candidate.usage.first_used_on,
                last_verified_on: &candidate.usage.last_verified_on,
                release_archive: &release_archive,
                task_spec: &task_path,
                failure_capsule: &capsule_path,
                submission_candidate: &submission_path,
                stdout_log: &logs.join("reproduction.stdout.txt"),
                stderr_log: &logs.join("reproduction.stderr.txt"),
            },
        )
        .is_err());
    }
}
