//! Create and verify portable Failure Capsule directories.
//!
//! The top-level `xtask` dispatcher exposes this module as
//! `failure-capsule create|verify`.

use anyhow::{bail, Context, Result};
use rne_ai::{BehaviorReplayAction, BehaviorReplayArtifact, TaskSpec, TASK_SPEC_KIND};
use rne_core::{DeterminismContract, DeterminismScope};
use rne_hardware_gateway::mock::{
    MockConformanceReport, MOCK_CONFORMANCE_REPORT_KIND, MOCK_CONFORMANCE_SCHEMA_VERSION,
};
use rne_hardware_gateway::shadow::{
    ShadowComparisonReport, SHADOW_COMPARISON_REPORT_KIND, SHADOW_COMPARISON_SCHEMA_VERSION,
};
use rne_hardware_gateway::wire::{
    HardwareSessionEvidence, HardwareWireTrace, HARDWARE_SESSION_EVIDENCE_KIND,
    HARDWARE_WIRE_SCHEMA_VERSION, HARDWARE_WIRE_TRACE_KIND,
};
use rne_hardware_lekiwi::session::{
    LeKiwiReferenceSessionEvidence, LEKIWI_REFERENCE_SESSION_KIND,
    LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
};
use rne_log::{
    ArtifactRef, BackendMetadata, BuildMetadata, FailureCapsule, FailureMetadata, ReplayArtifact,
    RunMetadata, FAILURE_CAPSULE_KIND as CAPSULE_KIND,
};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

const PHYSICS_CONFORMANCE_REPORT_KIND: &str = "rne_physics_conformance_report";

/// Runs the `failure-capsule create|verify` subcommand.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let command = args.next().ok_or_else(|| {
        anyhow::anyhow!(
            "failure-capsule requires `create` or `verify`; see docs/FAILURE_CAPSULE.md"
        )
    })?;

    match command.as_str() {
        "create" => create(args),
        "verify" => verify(args),
        "--help" | "-h" => {
            print_usage();
            Ok(())
        }
        other => bail!("unknown failure-capsule command `{other}`; expected `create` or `verify`"),
    }
}

fn print_usage() {
    println!(
        "failure-capsule create --replay PATH --output DIR [--evidence PATH]... [--backend NAME] [--backend-version VERSION]\n\
         failure-capsule verify DIR"
    );
}

#[derive(Debug)]
struct CreateOptions {
    replay: PathBuf,
    output: PathBuf,
    evidence: Vec<PathBuf>,
    backend: String,
    backend_version: String,
}

fn create(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let options = parse_create_options(args)?;
    let source = SourceReplay::read(&options.replay)?;
    let replay_bytes = read_regular_file(&options.replay)?;

    let mut evidence = Vec::with_capacity(options.evidence.len());
    for path in &options.evidence {
        evidence.push((path.clone(), read_regular_file(path)?));
    }

    let build = build_metadata()?;
    let backend = BackendMetadata::new(options.backend, options.backend_version);
    let plans = build_copy_plans(&options.replay, &replay_bytes, evidence)?;
    let capsule = source.capsule(&build, &backend, &plans)?;

    create_output_directory(&options.output)?;
    for plan in &plans {
        write_new_file(&options.output.join(&plan.relative_path), &plan.bytes)?;
    }
    let capsule_path = options.output.join("capsule.json");
    let capsule_json = serde_json::to_string_pretty(&capsule)?;
    write_new_file(&capsule_path, format!("{capsule_json}\n").as_bytes())?;

    println!(
        "created failure capsule {} ({})",
        options.output.display(),
        CAPSULE_KIND
    );
    Ok(())
}

fn verify(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("failure-capsule verify requires a capsule directory"))?;
    anyhow::ensure!(
        args.next().is_none(),
        "failure-capsule verify accepts exactly one capsule directory"
    );
    verify_directory(Path::new(&root))
}

fn parse_create_options(args: &mut impl Iterator<Item = String>) -> Result<CreateOptions> {
    let mut replay = None;
    let mut output = None;
    let mut evidence = Vec::new();
    let mut backend = "unknown".to_string();
    let mut backend_version = "unknown".to_string();

    while let Some(argument) = args.next() {
        let (option, inline_value) = argument
            .split_once('=')
            .map_or((argument.as_str(), None), |(name, value)| {
                (name, Some(value))
            });
        match option {
            "--replay" | "-r" => {
                replay = Some(PathBuf::from(option_value_owned(args, inline_value)?))
            }
            "--evidence" | "-e" => {
                evidence.push(PathBuf::from(option_value_owned(args, inline_value)?))
            }
            "--output" | "-o" => {
                output = Some(PathBuf::from(option_value_owned(args, inline_value)?))
            }
            "--backend" => backend = option_value_owned(args, inline_value)?,
            "--backend-version" => backend_version = option_value_owned(args, inline_value)?,
            "--help" | "-h" => {
                print_usage();
                bail!("help requested")
            }
            other => bail!("unknown failure-capsule create option `{other}`"),
        }
    }

    Ok(CreateOptions {
        replay: replay.ok_or_else(|| anyhow::anyhow!("create requires --replay PATH"))?,
        output: output.ok_or_else(|| anyhow::anyhow!("create requires --output DIR"))?,
        evidence,
        backend,
        backend_version,
    })
}

fn option_value_owned(
    args: &mut impl Iterator<Item = String>,
    inline_value: Option<&str>,
) -> Result<String> {
    if let Some(value) = inline_value {
        anyhow::ensure!(!value.is_empty(), "option value must not be empty");
        return Ok(value.to_string());
    }
    let value = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("option requires a value"))?;
    anyhow::ensure!(!value.is_empty(), "option value must not be empty");
    Ok(value)
}

#[derive(Debug)]
struct CopyPlan {
    relative_path: PathBuf,
    relative_path_string: String,
    bytes: Vec<u8>,
    role: String,
    kind: String,
    schema_version: u32,
}

impl CopyPlan {
    fn artifact_ref(&self) -> Result<ArtifactRef> {
        ArtifactRef::new(
            self.role.clone(),
            self.kind.clone(),
            self.schema_version,
            &self.relative_path_string,
            sha256_hex(&self.bytes),
        )
        .map_err(|error| anyhow::anyhow!("invalid generated artifact reference: {error}"))
    }
}

fn build_copy_plans(
    replay_path: &Path,
    replay_bytes: &[u8],
    evidence: Vec<(PathBuf, Vec<u8>)>,
) -> Result<Vec<CopyPlan>> {
    let replay_name = safe_file_name(replay_path)?;
    let mut plans = vec![CopyPlan {
        relative_path: PathBuf::from("replay").join(&replay_name),
        relative_path_string: format!("replay/{replay_name}"),
        bytes: replay_bytes.to_vec(),
        role: "replay".to_string(),
        kind: replay_kind(replay_bytes)?,
        schema_version: replay_schema_version(replay_bytes)?,
    }];

    for (path, bytes) in evidence {
        let name = safe_file_name(&path)?;
        let (kind, schema_version) = evidence_metadata(&bytes)?;
        plans.push(CopyPlan {
            relative_path: PathBuf::from("evidence").join(&name),
            relative_path_string: format!("evidence/{name}"),
            bytes,
            role: "evidence".to_string(),
            kind,
            schema_version,
        });
    }

    plans.sort_by(|left, right| left.relative_path_string.cmp(&right.relative_path_string));
    for pair in plans.windows(2) {
        anyhow::ensure!(
            pair[0].relative_path_string != pair[1].relative_path_string,
            "duplicate generated artifact path `{}`",
            pair[1].relative_path_string
        );
    }
    validate_hardware_evidence(
        plans
            .iter()
            .filter(|plan| plan.role == "evidence")
            .map(|plan| (plan.kind.as_str(), plan.bytes.as_slice())),
    )?;
    Ok(plans)
}

fn evidence_metadata(bytes: &[u8]) -> Result<(String, u32)> {
    let Ok(value) = serde_json::from_slice::<serde_json::Value>(bytes) else {
        return Ok(("evidence".to_string(), 1));
    };
    let Some(kind) = value.get("kind").and_then(serde_json::Value::as_str) else {
        return Ok(("evidence".to_string(), 1));
    };
    let recognized = matches!(
        kind,
        PHYSICS_CONFORMANCE_REPORT_KIND
            | TASK_SPEC_KIND
            | HARDWARE_SESSION_EVIDENCE_KIND
            | HARDWARE_WIRE_TRACE_KIND
            | SHADOW_COMPARISON_REPORT_KIND
            | MOCK_CONFORMANCE_REPORT_KIND
            | LEKIWI_REFERENCE_SESSION_KIND
    );
    if !recognized {
        return Ok(("evidence".to_string(), 1));
    }
    let schema_version = value
        .get("schema_version")
        .and_then(serde_json::Value::as_u64)
        .and_then(|version| u32::try_from(version).ok())
        .filter(|version| *version > 0)
        .ok_or_else(|| anyhow::anyhow!("{kind} evidence requires a positive u32 schema_version"))?;
    Ok((kind.to_string(), schema_version))
}

fn validate_hardware_evidence<'a>(
    evidence: impl Iterator<Item = (&'a str, &'a [u8])>,
) -> Result<()> {
    let evidence = evidence.collect::<Vec<_>>();
    let mut tasks = BTreeMap::<String, TaskSpec>::new();
    for (kind, bytes) in &evidence {
        if *kind != TASK_SPEC_KIND {
            continue;
        }
        let task: TaskSpec = serde_json::from_slice(bytes)
            .with_context(|| format!("invalid {TASK_SPEC_KIND} hardware evidence"))?;
        task.validate()
            .map_err(|error| anyhow::anyhow!("invalid hardware evidence TaskSpec: {error}"))?;
        anyhow::ensure!(
            tasks.insert(task.task_id.clone(), task).is_none(),
            "duplicate hardware evidence TaskSpec identity"
        );
    }

    for (kind, bytes) in evidence {
        match kind {
            HARDWARE_SESSION_EVIDENCE_KIND => {
                let session: HardwareSessionEvidence = serde_json::from_slice(bytes)
                    .context("invalid hardware session evidence JSON")?;
                anyhow::ensure!(
                    session.schema_version == HARDWARE_WIRE_SCHEMA_VERSION,
                    "unsupported hardware session evidence schema {}",
                    session.schema_version
                );
                let normalized = HardwareSessionEvidence::new(
                    session.wire_trace.clone(),
                    session.gateway.clone(),
                )
                .map_err(|error| anyhow::anyhow!("invalid hardware session evidence: {error}"))?;
                anyhow::ensure!(
                    normalized == session,
                    "hardware session evidence top-level metadata is inconsistent"
                );
                anyhow::ensure!(
                    tasks.contains_key(&session.task_id),
                    "hardware session evidence requires matching {TASK_SPEC_KIND} evidence for {:?}",
                    session.task_id
                );
            }
            LEKIWI_REFERENCE_SESSION_KIND => {
                let reference: LeKiwiReferenceSessionEvidence = serde_json::from_slice(bytes)
                    .context("invalid LeKiwi reference-session evidence JSON")?;
                anyhow::ensure!(
                    reference.schema_version == LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
                    "unsupported LeKiwi reference-session schema {}",
                    reference.schema_version
                );
                reference.validate().map_err(|error| {
                    anyhow::anyhow!("invalid LeKiwi reference-session evidence: {error}")
                })?;
                let task = tasks.get(&reference.profile.task.task_id).ok_or_else(|| {
                    anyhow::anyhow!(
                        "LeKiwi reference-session evidence requires matching {TASK_SPEC_KIND} evidence for {:?}",
                        reference.profile.task.task_id
                    )
                })?;
                anyhow::ensure!(
                    task == &reference.profile.task,
                    "LeKiwi reference-session embedded TaskSpec differs from matching evidence"
                );
            }
            HARDWARE_WIRE_TRACE_KIND => {
                let trace: HardwareWireTrace =
                    serde_json::from_slice(bytes).context("invalid hardware wire trace JSON")?;
                trace
                    .validate()
                    .map_err(|error| anyhow::anyhow!("invalid hardware wire trace: {error}"))?;
                anyhow::ensure!(
                    tasks.contains_key(&trace.task_id),
                    "hardware wire trace requires matching {TASK_SPEC_KIND} evidence for {:?}",
                    trace.task_id
                );
            }
            SHADOW_COMPARISON_REPORT_KIND => {
                let report: ShadowComparisonReport = serde_json::from_slice(bytes)
                    .context("invalid hardware shadow comparison JSON")?;
                anyhow::ensure!(
                    report.schema_version == SHADOW_COMPARISON_SCHEMA_VERSION,
                    "unsupported hardware shadow comparison schema {}",
                    report.schema_version
                );
                let task = tasks.get(&report.task_id).ok_or_else(|| {
                    anyhow::anyhow!(
                        "hardware shadow comparison requires matching {TASK_SPEC_KIND} evidence for {:?}",
                        report.task_id
                    )
                })?;
                report.validate_against(task).map_err(|error| {
                    anyhow::anyhow!("invalid hardware shadow comparison evidence: {error}")
                })?;
            }
            MOCK_CONFORMANCE_REPORT_KIND => {
                let report: MockConformanceReport = serde_json::from_slice(bytes)
                    .context("invalid hardware mock conformance JSON")?;
                anyhow::ensure!(
                    report.schema_version == MOCK_CONFORMANCE_SCHEMA_VERSION,
                    "unsupported hardware mock conformance schema {}",
                    report.schema_version
                );
                report.validate().map_err(|error| {
                    anyhow::anyhow!("invalid hardware mock conformance evidence: {error}")
                })?;
            }
            _ => {}
        }
    }
    Ok(())
}

fn replay_kind(bytes: &[u8]) -> Result<String> {
    if BehaviorReplayArtifact::from_json(
        std::str::from_utf8(bytes).context("replay must be UTF-8 JSON")?,
    )
    .is_ok()
    {
        return Ok("rne_behavior_replay".to_string());
    }
    if ReplayArtifact::from_json(std::str::from_utf8(bytes).context("replay must be UTF-8 JSON")?)
        .is_ok()
    {
        return Ok("rne_replay".to_string());
    }
    bail!("replay is neither a valid behavior replay nor a generic replay")
}

fn replay_schema_version(bytes: &[u8]) -> Result<u32> {
    let text = std::str::from_utf8(bytes).context("replay must be UTF-8 JSON")?;
    if let Ok(artifact) = serde_json::from_str::<BehaviorReplayArtifact>(text) {
        return Ok(artifact.schema_version);
    }
    let artifact = serde_json::from_str::<ReplayArtifact>(text)
        .context("replay is neither a valid behavior replay nor a generic replay")?;
    Ok(artifact.version)
}

fn safe_file_name(path: &Path) -> Result<String> {
    let name = path
        .file_name()
        .and_then(|value| value.to_str())
        .ok_or_else(|| {
            anyhow::anyhow!(
                "artifact path must have a UTF-8 filename: {}",
                path.display()
            )
        })?;
    anyhow::ensure!(
        !name.is_empty() && name != "." && name != "..",
        "artifact path must have a non-empty filename: {}",
        path.display()
    );
    anyhow::ensure!(
        !name.contains('/') && !name.contains('\\'),
        "artifact filename must not contain path separators: {}",
        path.display()
    );
    Ok(name.to_string())
}

fn create_output_directory(path: &Path) -> Result<()> {
    anyhow::ensure!(
        !path.as_os_str().is_empty(),
        "output directory must not be empty"
    );
    if path.exists() || fs::symlink_metadata(path).is_ok() {
        bail!(
            "refusing to overwrite existing capsule directory `{}`",
            path.display()
        );
    }
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("could not create output parent `{}`", parent.display()))?;
    }
    fs::create_dir(path)
        .with_context(|| format!("could not create capsule directory `{}`", path.display()))?;
    fs::create_dir(path.join("replay"))?;
    fs::create_dir(path.join("evidence"))?;
    Ok(())
}

fn write_new_file(path: &Path, bytes: &[u8]) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .with_context(|| format!("could not create `{}`", path.display()))?;
    file.write_all(bytes)?;
    file.sync_all()?;
    Ok(())
}

fn read_regular_file(path: &Path) -> Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("could not inspect artifact `{}`", path.display()))?;
    anyhow::ensure!(
        !metadata.file_type().is_symlink() && metadata.is_file(),
        "artifact must be a regular non-symlink file: {}",
        path.display()
    );
    fs::read(path).with_context(|| format!("could not read artifact `{}`", path.display()))
}

fn verify_directory(root: &Path) -> Result<()> {
    let root_metadata = fs::symlink_metadata(root)
        .with_context(|| format!("could not inspect capsule directory `{}`", root.display()))?;
    anyhow::ensure!(
        !root_metadata.file_type().is_symlink() && root_metadata.is_dir(),
        "capsule root must be a regular directory, not a symlink: {}",
        root.display()
    );
    let canonical_root = fs::canonicalize(root)?;
    let capsule_path = root.join("capsule.json");
    let capsule_metadata = fs::symlink_metadata(&capsule_path)
        .with_context(|| format!("missing capsule metadata `{}`", capsule_path.display()))?;
    anyhow::ensure!(
        !capsule_metadata.file_type().is_symlink() && capsule_metadata.is_file(),
        "capsule.json must be a regular non-symlink file"
    );
    let capsule_text = fs::read_to_string(&capsule_path)?;
    let capsule: FailureCapsule = serde_json::from_str(&capsule_text)
        .context("capsule.json is not valid Failure Capsule JSON")?;
    capsule
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid failure capsule: {error}"))?;

    let mut verified_artifacts = Vec::with_capacity(capsule.artifacts.len());
    for artifact in &capsule.artifacts {
        let bytes = verify_artifact_path(&canonical_root, root, artifact)?;
        verified_artifacts.push((artifact.kind.as_str(), bytes));
    }
    validate_hardware_evidence(
        verified_artifacts
            .iter()
            .map(|(kind, bytes)| (*kind, bytes.as_slice())),
    )?;
    println!(
        "verified failure capsule {} ({} artifacts)",
        root.display(),
        capsule.artifacts.len()
    );
    Ok(())
}

fn verify_artifact_path(
    canonical_root: &Path,
    root: &Path,
    artifact: &ArtifactRef,
) -> Result<Vec<u8>> {
    let normalized = rne_log::normalize_relative_path(&artifact.path)
        .map_err(|error| anyhow::anyhow!("invalid artifact path `{}`: {error}", artifact.path))?;
    anyhow::ensure!(
        normalized == artifact.path,
        "artifact path is not canonical: `{}`",
        artifact.path
    );
    let path = root.join(Path::new(&artifact.path));
    let metadata = fs::symlink_metadata(&path)
        .with_context(|| format!("artifact is missing: `{}`", artifact.path))?;
    anyhow::ensure!(
        !metadata.file_type().is_symlink() && metadata.is_file(),
        "artifact must be a regular non-symlink file: `{}`",
        artifact.path
    );
    let canonical_path = fs::canonicalize(&path)
        .with_context(|| format!("could not resolve artifact `{}`", artifact.path))?;
    anyhow::ensure!(
        canonical_path.starts_with(canonical_root),
        "artifact path escapes capsule root through a symlink: `{}`",
        artifact.path
    );
    let bytes = fs::read(&path)?;
    let digest = sha256_hex(&bytes);
    anyhow::ensure!(
        digest == artifact.sha256,
        "SHA-256 mismatch for `{}`: expected {}, got {}",
        artifact.path,
        artifact.sha256,
        digest
    );

    if artifact.kind == "rne_replay" {
        let text = std::str::from_utf8(&bytes).context("generic replay is not UTF-8")?;
        ReplayArtifact::from_json(text).map_err(|error| {
            anyhow::anyhow!("invalid generic replay `{}`: {error}", artifact.path)
        })?;
    } else if artifact.kind == "rne_behavior_replay" {
        let text = std::str::from_utf8(&bytes).context("behavior replay is not UTF-8")?;
        BehaviorReplayArtifact::from_json(text).map_err(|error| {
            anyhow::anyhow!("invalid behavior replay `{}`: {error}", artifact.path)
        })?;
    } else if artifact.kind == PHYSICS_CONFORMANCE_REPORT_KIND {
        let value: serde_json::Value = serde_json::from_slice(&bytes)
            .with_context(|| format!("invalid conformance report `{}`", artifact.path))?;
        anyhow::ensure!(
            value.get("kind").and_then(serde_json::Value::as_str)
                == Some(PHYSICS_CONFORMANCE_REPORT_KIND),
            "conformance report kind mismatch in `{}`",
            artifact.path
        );
        anyhow::ensure!(
            value
                .get("schema_version")
                .and_then(serde_json::Value::as_u64)
                == Some(u64::from(artifact.schema_version)),
            "conformance report schema mismatch in `{}`",
            artifact.path
        );
    }
    Ok(bytes)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn build_metadata() -> Result<BuildMetadata> {
    let start = env::current_dir()?;
    let root = find_workspace_root(&start)?;
    let lock_bytes = fs::read(root.join("Cargo.lock"))?;
    let (target_triple, rustc_version) = rustc_metadata();
    let git_commit = git_commit(&root);
    let profile = env::var("PROFILE").unwrap_or_else(|_| {
        if cfg!(debug_assertions) {
            "debug".to_string()
        } else {
            "release".to_string()
        }
    });
    Ok(BuildMetadata::new(
        env!("CARGO_PKG_VERSION"),
        git_commit,
        profile,
        target_triple,
        rustc_version,
        sha256_hex(&lock_bytes),
    ))
}

fn find_workspace_root(start: &Path) -> Result<PathBuf> {
    let mut current = start;
    loop {
        if current.join("Cargo.lock").is_file() {
            return Ok(current.to_path_buf());
        }
        current = current.parent().ok_or_else(|| {
            anyhow::anyhow!(
                "could not find workspace Cargo.lock from `{}`",
                start.display()
            )
        })?;
    }
}

fn rustc_metadata() -> (String, String) {
    let Ok(output) = Command::new("rustc").arg("-vV").output() else {
        return (fallback_target_triple(), "unknown".to_string());
    };
    if !output.status.success() {
        return (fallback_target_triple(), "unknown".to_string());
    }
    let text = String::from_utf8_lossy(&output.stdout);
    let mut host = None;
    let mut release = None;
    for line in text.lines() {
        if let Some(value) = line.strip_prefix("host: ") {
            host = Some(value.to_string());
        } else if let Some(value) = line.strip_prefix("release: ") {
            release = Some(value.to_string());
        }
    }
    (
        host.unwrap_or_else(fallback_target_triple),
        release
            .map(|value| format!("rustc {value}"))
            .unwrap_or_else(|| "unknown".to_string()),
    )
}

fn fallback_target_triple() -> String {
    format!("{}-{}", env::consts::ARCH, env::consts::OS)
}

fn git_commit(root: &Path) -> String {
    let Ok(output) = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["rev-parse", "HEAD"])
        .output()
    else {
        return "unknown".to_string();
    };
    if !output.status.success() {
        return "unknown".to_string();
    }
    let value = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if value.is_empty() {
        "unknown".to_string()
    } else {
        value
    }
}

enum SourceReplay {
    Generic(ReplayArtifact),
    Behavior(BehaviorReplayArtifact),
}

impl SourceReplay {
    fn read(path: &Path) -> Result<Self> {
        let bytes = read_regular_file(path)?;
        let text = std::str::from_utf8(&bytes).context("replay must be UTF-8 JSON")?;
        if let Ok(artifact) = BehaviorReplayArtifact::from_json(text) {
            return Ok(Self::Behavior(artifact));
        }
        if let Ok(artifact) = ReplayArtifact::from_json(text) {
            return Ok(Self::Generic(artifact));
        }
        bail!(
            "replay `{}` is neither a valid behavior replay nor a generic replay",
            path.display()
        )
    }

    fn capsule(
        &self,
        build: &BuildMetadata,
        backend: &BackendMetadata,
        plans: &[CopyPlan],
    ) -> Result<FailureCapsule> {
        let artifacts = plans
            .iter()
            .map(CopyPlan::artifact_ref)
            .collect::<Result<Vec<_>>>()?;
        match self {
            Self::Generic(replay) => self.generic_capsule(build, backend, artifacts, replay),
            Self::Behavior(replay) => self.behavior_capsule(build, backend, artifacts, replay),
        }
    }

    fn generic_capsule(
        &self,
        build: &BuildMetadata,
        backend: &BackendMetadata,
        artifacts: Vec<ArtifactRef>,
        replay: &ReplayArtifact,
    ) -> Result<FailureCapsule> {
        let final_frame = replay
            .frames
            .last()
            .ok_or_else(|| anyhow::anyhow!("generic replay must contain at least one frame"))?;
        let failure_kind = replay.final_report.failure.ok_or_else(|| {
            anyhow::anyhow!(
                "generic replay has no final_report.failure; successful replays cannot form a failure capsule"
            )
        })?;
        let fixed_delta_ticks = recorded_fixed_delta_ticks(replay)?;
        let run_id = format!("{}-seed-{}", replay.scene, replay.seed);
        let contract = DeterminismContract::exact(
            "replay",
            DeterminismScope::new("replay", ["world.state"], 0, replay.clock.steps)?,
        )?;
        let failure = FailureMetadata::new(
            format!("replay-step-{}", final_frame.step),
            "replay",
            format!("generic replay failed: {failure_kind:?}"),
            final_frame.step,
            final_frame.sim_ticks,
            final_frame.physics_hash,
        );
        let run = RunMetadata::new(
            &run_id,
            &replay.scene,
            replay.seed,
            fixed_delta_ticks,
            replay.clock.steps,
            replay.frames.len() as u64,
        );
        FailureCapsule::new(
            failure,
            run,
            build.clone(),
            backend.clone(),
            contract,
            artifacts,
        )
        .map_err(|error| anyhow::anyhow!("could not build generic failure capsule: {error}"))
    }

    fn behavior_capsule(
        &self,
        build: &BuildMetadata,
        backend: &BackendMetadata,
        artifacts: Vec<ArtifactRef>,
        replay: &BehaviorReplayArtifact,
    ) -> Result<FailureCapsule> {
        let violation = &replay.failure.violation;
        let contract_name = replay.failure.contract.name.clone();
        let run_id = format!("{}-seed-{}", replay.scenario, replay.seed);
        let action_count = replay
            .frames
            .iter()
            .filter(|frame| frame.action == BehaviorReplayAction::Advance)
            .count() as u64;
        let contract = DeterminismContract::outcome(
            &contract_name,
            DeterminismScope::new(
                &replay.scenario,
                ["behavior.observation", "world.state"],
                0,
                replay.frames.len() as u64,
            )?,
            &contract_name,
        )?;
        let failure = FailureMetadata::new(
            format!("{contract_name}-step-{}", violation.step),
            &contract_name,
            &violation.message,
            violation.step,
            violation.sim_time_ticks,
            violation.state_digest,
        );
        let run = RunMetadata::new(
            &run_id,
            &replay.scenario,
            replay.seed,
            replay.fixed_delta_ticks,
            replay.frames.len() as u64,
            action_count,
        );
        let capsule = FailureCapsule::new(
            failure,
            run,
            build.clone(),
            backend.clone(),
            contract,
            artifacts,
        )
        .map_err(|error| anyhow::anyhow!("could not build behavior failure capsule: {error}"))?;
        // BehaviorMinimizationMetadata intentionally has no source step/action
        // counts. Keep the minimization provenance in the replay reference;
        // do not manufacture counts in the envelope.
        Ok(capsule)
    }
}

fn recorded_fixed_delta_ticks(replay: &ReplayArtifact) -> Result<u64> {
    let first = replay
        .frames
        .first()
        .ok_or_else(|| anyhow::anyhow!("generic replay must contain at least one frame"))?;
    let first_step_plus_one = first
        .step
        .checked_add(1)
        .ok_or_else(|| anyhow::anyhow!("generic replay first step overflows u64"))?;
    anyhow::ensure!(
        first_step_plus_one > 0,
        "generic replay first step denominator must be positive"
    );
    anyhow::ensure!(
        first.sim_ticks % first_step_plus_one == 0,
        "generic replay first frame timestamp {} is not divisible by step+1 {}",
        first.sim_ticks,
        first_step_plus_one
    );
    let delta_ticks = first.sim_ticks / first_step_plus_one;
    anyhow::ensure!(
        delta_ticks > 0,
        "generic replay recorded fixed timestep must be greater than zero"
    );

    for frame in &replay.frames {
        let step_plus_one = frame
            .step
            .checked_add(1)
            .ok_or_else(|| anyhow::anyhow!("generic replay frame step overflows u64"))?;
        let expected_ticks = step_plus_one
            .checked_mul(delta_ticks)
            .ok_or_else(|| anyhow::anyhow!("generic replay timestamp overflows u64"))?;
        anyhow::ensure!(
            frame.sim_ticks == expected_ticks,
            "generic replay frame {} has sim_ticks={}, expected {} for fixed delta {}",
            frame.step,
            frame.sim_ticks,
            expected_ticks,
            delta_ticks
        );
    }
    Ok(delta_ticks)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ai::{
        BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayFailure,
        BehaviorReplayFrame, BehaviorViolation,
    };
    use rne_hardware_gateway::wire::{
        DeviceWireFrame, DeviceWirePayload, HostWireFrame, HostWirePayload,
    };
    use rne_hardware_gateway::HardwareMode;
    use rne_hardware_lekiwi::lekiwi_base_task_spec;
    use rne_hardware_lekiwi::session::{
        LeKiwiMonotonicClock, LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionConfig,
        LeKiwiReferenceSessionRunner, LeKiwiTransportError, LeKiwiWireTransport,
    };
    use rne_log::{
        ReplayAction, ReplayClock, ReplayContact, ReplayControllerKind, ReplayFailureKind,
        ReplayFinalReport, ReplayFrame, ReplayObservation,
    };

    #[derive(Debug, Default)]
    struct FixtureClock(u64);

    impl LeKiwiMonotonicClock for FixtureClock {
        fn now_ms(&mut self) -> u64 {
            let now_ms = self.0;
            self.0 += 1;
            now_ms
        }
    }

    #[derive(Debug, Default)]
    struct FixtureLeKiwiTransport {
        observation_sequence: u64,
    }

    impl LeKiwiWireTransport for FixtureLeKiwiTransport {
        fn exchange(
            &mut self,
            request: &HostWireFrame,
        ) -> Result<DeviceWireFrame, LeKiwiTransportError> {
            let payload = match &request.payload {
                HostWirePayload::Open {
                    task_id,
                    observation_width,
                    action_width,
                    ..
                } => DeviceWirePayload::Ready {
                    device_id: rne_hardware_lekiwi::LEKIWI_MOCK_DEVICE_ID.to_string(),
                    task_id: task_id.clone(),
                    observation_width: *observation_width,
                    action_width: *action_width,
                },
                HostWirePayload::PollObservation => {
                    self.observation_sequence += 1;
                    DeviceWirePayload::Observation {
                        sequence: self.observation_sequence,
                        values: vec![0.0; 9],
                    }
                }
                HostWirePayload::Actuate { frame } => DeviceWirePayload::ActuationAccepted {
                    action_sequence: frame.action_sequence,
                    safety_stop: frame.safety_stop,
                },
                HostWirePayload::Close => DeviceWirePayload::Closed,
            };
            Ok(DeviceWireFrame::new(
                request.session_id.clone(),
                request.sequence,
                payload,
            ))
        }
    }

    fn lekiwi_reference_session_fixture() -> LeKiwiReferenceSessionEvidence {
        let mut runner = LeKiwiReferenceSessionRunner::new(
            FixtureLeKiwiTransport::default(),
            FixtureClock::default(),
            LeKiwiReferenceSessionConfig::new(
                "rne.lekiwi.capsule.fixture",
                HardwareMode::Shadow,
                1,
            ),
        )
        .expect("build reference runner");
        runner.open().expect("open reference runner");
        assert!(matches!(
            runner.sample(vec![0.0; 3]).expect("sample"),
            LeKiwiReferenceSampleOutcome::Sample(_)
        ));
        runner.close().expect("close reference runner")
    }
    use serde_json::Value;
    use tempfile::TempDir;

    fn generic_fixture() -> ReplayArtifact {
        ReplayArtifact::new(
            "fixture_scene",
            7,
            ReplayClock::new(2, 60.0),
            ReplayControllerKind::None,
            Vec::new(),
            vec![
                ReplayFrame::new(
                    0,
                    16_666_667,
                    ReplayAction::differential_drive(0.0),
                    ReplayObservation::new(None).with_contact(Some(ReplayContact::default())),
                    77,
                ),
                ReplayFrame::new(
                    1,
                    33_333_334,
                    ReplayAction::differential_drive(0.0),
                    ReplayObservation::new(None).with_contact(Some(ReplayContact::default())),
                    99,
                ),
            ],
            ReplayFinalReport::new(
                2,
                2.0 / 60.0,
                7,
                0,
                0,
                99,
                None,
                0,
                0.0,
                None,
                Some(ReplayFailureKind::Fell),
            ),
        )
    }

    fn behavior_fixture() -> BehaviorReplayArtifact {
        let descriptor = BehaviorContractDescriptor {
            name: "fixture_failure".to_string(),
            kind: BehaviorContractKind::Always,
            entities: Vec::new(),
        };
        BehaviorReplayArtifact::new(
            "behavior_fixture",
            11,
            3,
            10,
            Vec::new(),
            vec![descriptor.clone()],
            vec![BehaviorReplayFrame {
                step: 0,
                sim_time_ticks: 0,
                action: BehaviorReplayAction::InitialObservation,
                observation: Value::Null,
                state_digest: 42,
            }],
            BehaviorReplayFailure {
                contract: descriptor,
                violation: BehaviorViolation {
                    step: 0,
                    sim_time_ticks: 0,
                    state_digest: 42,
                    entities: Vec::new(),
                    message: "fixture failed".to_string(),
                },
            },
        )
        .expect("behavior fixture")
    }

    fn string_path(path: &Path) -> String {
        path.to_string_lossy().into_owned()
    }

    fn invoke_create(replay: &Path, output: &Path, evidence: &[&Path]) -> Result<()> {
        let mut arguments = vec![
            "create".to_string(),
            "--replay".to_string(),
            string_path(replay),
            "--output".to_string(),
            string_path(output),
        ];
        for path in evidence {
            arguments.push("--evidence".to_string());
            arguments.push(string_path(path));
        }
        run(&mut arguments.into_iter())
    }

    fn invoke_verify(output: &Path) -> Result<()> {
        run(&mut ["verify".to_string(), string_path(output)].into_iter())
    }

    #[test]
    fn generic_fixture_roundtrips_and_refuses_overwrite() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("generic.rne-replay");
        generic_fixture()
            .write_json(&replay_path)
            .expect("write replay");
        let evidence = temp.path().join("report.json");
        fs::write(&evidence, br#"{"status":"failed"}"#).expect("write evidence");
        let output = temp.path().join("capsule");

        invoke_create(&replay_path, &output, &[&evidence]).expect("create capsule");
        invoke_verify(&output).expect("verify capsule");
        let capsule: FailureCapsule =
            serde_json::from_str(&fs::read_to_string(output.join("capsule.json")).unwrap())
                .expect("capsule json");
        assert_eq!(capsule.artifacts.len(), 2);
        assert!(capsule
            .artifacts
            .iter()
            .any(|artifact| artifact.role == "replay"));
        assert!(capsule
            .artifacts
            .windows(2)
            .all(|window| window[0].path < window[1].path));
        assert_ne!(capsule.failure.id, capsule.run.id);
        assert!(capsule.failure.id.contains("replay-step-1"));
        let capsule_json = fs::read_to_string(output.join("capsule.json")).unwrap();
        assert_eq!(
            capsule_json,
            format!("{}\n", serde_json::to_string_pretty(&capsule).unwrap())
        );
        assert!(invoke_create(&replay_path, &output, &[]).is_err());
    }

    #[test]
    fn behavior_fixture_roundtrips_through_known_reader() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("behavior.rne-replay");
        behavior_fixture()
            .write_json(&replay_path)
            .expect("write behavior replay");
        let output = temp.path().join("capsule");
        invoke_create(&replay_path, &output, &[]).expect("create behavior capsule");
        invoke_verify(&output).expect("verify behavior capsule");
        let capsule: FailureCapsule =
            serde_json::from_str(&fs::read_to_string(output.join("capsule.json")).unwrap())
                .expect("capsule json");
        assert_eq!(capsule.artifacts[0].kind, "rne_behavior_replay");
        assert_eq!(capsule.failure.contract, "fixture_failure");
        assert_ne!(capsule.failure.id, capsule.run.id);
        assert!(capsule.failure.id.contains("fixture_failure-step-0"));
        assert!(capsule.minimization.is_none());
    }

    #[test]
    fn physics_conformance_evidence_preserves_kind_and_schema() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("divergence.rne-replay");
        behavior_fixture()
            .write_json(&replay_path)
            .expect("write behavior replay");
        let report_path = temp.path().join("conformance-report.json");
        fs::write(
            &report_path,
            br#"{"kind":"rne_physics_conformance_report","schema_version":2,"all_passed":false}"#,
        )
        .expect("write conformance evidence");
        let output = temp.path().join("capsule");

        invoke_create(&replay_path, &output, &[&report_path]).expect("create capsule");
        invoke_verify(&output).expect("verify capsule");
        let capsule: FailureCapsule =
            serde_json::from_str(&fs::read_to_string(output.join("capsule.json")).unwrap())
                .expect("capsule json");
        let report = capsule
            .artifacts
            .iter()
            .find(|artifact| artifact.role == "evidence")
            .expect("evidence reference");
        assert_eq!(report.kind, PHYSICS_CONFORMANCE_REPORT_KIND);
        assert_eq!(report.schema_version, 2);
    }

    #[test]
    fn hardware_evidence_roundtrips_with_task_bound_validation() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("shadow-failure.rne-replay");
        behavior_fixture()
            .write_json(&replay_path)
            .expect("write behavior replay");
        let task_path = temp.path().join("diff-drive-task.json");
        fs::write(
            &task_path,
            include_bytes!("../../assets/tasks/diff_drive_goal.task.json"),
        )
        .expect("write TaskSpec evidence");
        let session_path = temp.path().join("hardware-session.json");
        fs::write(
            &session_path,
            include_bytes!(
                "../../tests/golden/hardware/gateway-process-disconnect-session-v1.json"
            ),
        )
        .expect("write session evidence");
        let shadow_path = temp.path().join("shadow-comparison.json");
        fs::write(
            &shadow_path,
            include_bytes!("../../tests/golden/hardware/gateway-shadow-comparison-v1.json"),
        )
        .expect("write shadow evidence");
        let conformance_path = temp.path().join("mock-conformance.json");
        fs::write(
            &conformance_path,
            include_bytes!("../../tests/golden/hardware/gateway-mock-conformance-v1.json"),
        )
        .expect("write mock conformance evidence");
        let lekiwi_task_path = temp.path().join("lekiwi-task.json");
        fs::write(
            &lekiwi_task_path,
            serde_json::to_vec_pretty(&lekiwi_base_task_spec()).unwrap(),
        )
        .expect("write LeKiwi TaskSpec evidence");
        let lekiwi_session_path = temp.path().join("lekiwi-reference-session.json");
        fs::write(
            &lekiwi_session_path,
            serde_json::to_vec_pretty(&lekiwi_reference_session_fixture()).unwrap(),
        )
        .expect("write LeKiwi reference-session evidence");
        let output = temp.path().join("hardware-capsule");

        invoke_create(
            &replay_path,
            &output,
            &[
                &task_path,
                &session_path,
                &shadow_path,
                &conformance_path,
                &lekiwi_task_path,
                &lekiwi_session_path,
            ],
        )
        .expect("create hardware evidence capsule");
        invoke_verify(&output).expect("verify hardware evidence capsule");
        let capsule: FailureCapsule =
            serde_json::from_str(&fs::read_to_string(output.join("capsule.json")).unwrap())
                .expect("capsule json");
        for expected_kind in [
            TASK_SPEC_KIND,
            HARDWARE_SESSION_EVIDENCE_KIND,
            SHADOW_COMPARISON_REPORT_KIND,
            MOCK_CONFORMANCE_REPORT_KIND,
            LEKIWI_REFERENCE_SESSION_KIND,
        ] {
            assert!(capsule
                .artifacts
                .iter()
                .any(|artifact| artifact.kind == expected_kind));
        }
    }

    #[test]
    fn hardware_evidence_rejects_missing_task_and_tampered_summary() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("shadow-failure.rne-replay");
        behavior_fixture()
            .write_json(&replay_path)
            .expect("write behavior replay");
        let session_path = temp.path().join("hardware-session.json");
        fs::write(
            &session_path,
            include_bytes!(
                "../../tests/golden/hardware/gateway-process-disconnect-session-v1.json"
            ),
        )
        .expect("write session evidence");
        let missing_task_output = temp.path().join("missing-task-capsule");
        let error = invoke_create(&replay_path, &missing_task_output, &[&session_path])
            .expect_err("hardware session without TaskSpec must reject");
        assert!(error
            .to_string()
            .contains("requires matching rne_task_spec"));
        assert!(!missing_task_output.exists());

        let task_path = temp.path().join("diff-drive-task.json");
        fs::write(
            &task_path,
            include_bytes!("../../assets/tasks/diff_drive_goal.task.json"),
        )
        .expect("write TaskSpec evidence");
        let mut shadow: Value = serde_json::from_slice(include_bytes!(
            "../../tests/golden/hardware/gateway-shadow-comparison-v1.json"
        ))
        .expect("shadow json");
        shadow["summary"]["passed"] = Value::Bool(true);
        let shadow_path = temp.path().join("tampered-shadow.json");
        fs::write(&shadow_path, serde_json::to_vec_pretty(&shadow).unwrap())
            .expect("write tampered shadow");
        let tampered_output = temp.path().join("tampered-capsule");
        let error = invoke_create(&replay_path, &tampered_output, &[&task_path, &shadow_path])
            .expect_err("tampered shadow summary must reject");
        assert!(error.to_string().contains("summary does not match"));
        assert!(!tampered_output.exists());
    }

    #[test]
    fn successful_generic_replay_is_not_a_failure_capsule() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("success.rne-replay");
        let mut replay = generic_fixture();
        replay.final_report.failure = None;
        replay.write_json(&replay_path).expect("write replay");
        let output = temp.path().join("capsule");

        let error = invoke_create(&replay_path, &output, &[]).expect_err("success must reject");
        assert!(error.to_string().contains("no final_report.failure"));
        assert!(!output.exists());
    }

    #[test]
    fn generic_fixed_delta_comes_from_recorded_timestamps() {
        let mut replay = generic_fixture();
        assert_eq!(recorded_fixed_delta_ticks(&replay).unwrap(), 16_666_667);
        replay.frames[1].sim_ticks += 1;
        let error = recorded_fixed_delta_ticks(&replay).expect_err("irregular timestamps reject");
        assert!(error.to_string().contains("expected"));
    }

    #[test]
    fn verification_detects_mutated_bytes_and_traversal() {
        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("generic.rne-replay");
        generic_fixture()
            .write_json(&replay_path)
            .expect("write replay");
        let output = temp.path().join("capsule");
        invoke_create(&replay_path, &output, &[]).expect("create capsule");

        let replay_copy = output.join("replay/generic.rne-replay");
        let mut bytes = fs::read(&replay_copy).expect("read copied replay");
        bytes.push(b' ');
        fs::write(&replay_copy, bytes).expect("mutate copied replay");
        assert!(invoke_verify(&output).is_err());

        let mut value: Value =
            serde_json::from_str(&fs::read_to_string(output.join("capsule.json")).unwrap())
                .unwrap();
        value["artifacts"][0]["path"] = Value::String("../outside.json".to_string());
        fs::write(
            output.join("capsule.json"),
            serde_json::to_string_pretty(&value).unwrap(),
        )
        .unwrap();
        assert!(invoke_verify(&output).is_err());
    }

    #[cfg(unix)]
    #[test]
    fn verification_rejects_symlink_escape() {
        use std::os::unix::fs::symlink;

        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("generic.rne-replay");
        generic_fixture()
            .write_json(&replay_path)
            .expect("write replay");
        let output = temp.path().join("capsule");
        invoke_create(&replay_path, &output, &[]).expect("create capsule");
        let outside = temp.path().join("outside.json");
        fs::write(&outside, b"outside").expect("outside");
        fs::remove_file(output.join("replay/generic.rne-replay")).expect("remove copy");
        symlink(&outside, output.join("replay/generic.rne-replay")).expect("symlink");
        assert!(invoke_verify(&output).is_err());
    }

    #[cfg(windows)]
    #[test]
    fn verification_rejects_symlink_escape() {
        use std::os::windows::fs::symlink_file;

        let temp = TempDir::new().expect("tempdir");
        let replay_path = temp.path().join("generic.rne-replay");
        generic_fixture()
            .write_json(&replay_path)
            .expect("write replay");
        let output = temp.path().join("capsule");
        invoke_create(&replay_path, &output, &[]).expect("create capsule");
        let outside = temp.path().join("outside.json");
        fs::write(&outside, b"outside").expect("outside");
        fs::remove_file(output.join("replay/generic.rne-replay")).expect("remove copy");
        if symlink_file(&outside, output.join("replay/generic.rne-replay")).is_err() {
            return;
        }
        assert!(invoke_verify(&output).is_err());
    }
}
