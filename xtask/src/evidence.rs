//! Aggregate, validate, and inventory the v0.2 trust evidence.

use anyhow::{Context, Result};
use rne_log::FailureCapsule;
use rne_physics::PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Schema version for the aggregate evidence manifest.
pub(crate) const EVIDENCE_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Stable artifact discriminator for the aggregate evidence manifest.
pub(crate) const EVIDENCE_MANIFEST_KIND: &str = "rne_evidence_manifest";

const DEFAULT_OUTPUT: &str = "artifacts/evidence";
const FAILURE_CASE: &str =
    "crates/rne_ai/tests/fixtures/unitree_g1_dex3_invalid_tray.behavior-case.json";
const MINIMIZED_REPLAY: &str = "unitree_g1_dex3_invalid_tray-seed-0-minimized.rne-replay";
const REQUIRED_ARTIFACT_IDS: [&str; 4] = [
    "benchmark_report",
    "capability_report",
    "failure_capsule",
    "physics_conformance",
];

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct EvidenceManifest {
    kind: String,
    schema_version: u32,
    engine_version: String,
    source_commit: String,
    all_passed: bool,
    artifacts: Vec<EvidenceArtifact>,
}

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(deny_unknown_fields)]
struct EvidenceArtifact {
    id: String,
    kind: String,
    schema_version: u32,
    path: String,
    sha256: String,
}

#[derive(Debug)]
struct EvidenceOptions {
    output: PathBuf,
}

/// Runs every stable v0.2 evidence producer and verifies the resulting bundle.
pub(crate) fn evidence(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = crate::workspace_root()?;
    let Some(options) = parse_options(args, &root)? else {
        print_usage();
        return Ok(());
    };

    anyhow::ensure!(
        !options.output.exists(),
        "refusing to overwrite existing evidence output `{}`; choose a new --output directory",
        options.output.display()
    );
    let parent = options
        .output
        .parent()
        .context("evidence output must have a parent directory")?;
    fs::create_dir_all(parent)
        .with_context(|| format!("create evidence output parent {}", parent.display()))?;

    let name = options
        .output
        .file_name()
        .and_then(|name| name.to_str())
        .context("evidence output must have a UTF-8 directory name")?;
    let staging = parent.join(format!(".{name}.partial-{}", std::process::id()));
    anyhow::ensure!(
        !staging.exists(),
        "evidence staging directory already exists: {}",
        staging.display()
    );
    fs::create_dir(&staging)
        .with_context(|| format!("create evidence staging directory {}", staging.display()))?;

    run_evidence_producers(&root, &staging).with_context(|| {
        format!(
            "evidence generation failed; partial diagnostics remain at {}",
            staging.display()
        )
    })?;
    fs::rename(&staging, &options.output).with_context(|| {
        format!(
            "publish evidence directory {} -> {}",
            staging.display(),
            options.output.display()
        )
    })?;

    println!(
        "evidence bundle ok: output={} manifest={}",
        options.output.display(),
        options.output.join("manifest.json").display()
    );
    Ok(())
}

fn print_usage() {
    println!("evidence [--output DIR]");
}

fn parse_options(
    args: &mut impl Iterator<Item = String>,
    root: &Path,
) -> Result<Option<EvidenceOptions>> {
    let mut output = root.join(DEFAULT_OUTPUT);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" => {
                let path =
                    PathBuf::from(args.next().context("--output requires a directory path")?);
                output = if path.is_absolute() {
                    path
                } else {
                    root.join(path)
                };
            }
            "--help" | "-h" => return Ok(None),
            other => anyhow::bail!("unknown evidence argument: {other}"),
        }
    }
    Ok(Some(EvidenceOptions { output }))
}

fn run_evidence_producers(root: &Path, staging: &Path) -> Result<()> {
    let capability_path = staging.join("capability-report.json");
    let physics_path = staging.join("physics-conformance.json");
    let scenario_path = staging.join("scenario-scale.json");
    let benchmark_path = staging.join("benchmark-report.json");
    let benchmark_timings_path = staging.join("benchmark-timings.json");
    let behavior_dir = staging.join("behavior-ci");
    let behavior_report = behavior_dir.join("report.json");
    let behavior_junit = behavior_dir.join("junit.xml");
    let replay_dir = behavior_dir.join("replays");
    let minimized_replay = replay_dir.join(MINIMIZED_REPLAY);
    let capsule_dir = staging.join("failure-capsule");

    let mut capability_args =
        vec!["--output".to_string(), path_argument(&capability_path)].into_iter();
    crate::capability_report::capability_report(&mut capability_args)?;

    let mut physics_args = vec!["--json".to_string(), path_argument(&physics_path)].into_iter();
    crate::physics_conformance(&mut physics_args)?;

    let mut scenario_args = vec!["--json".to_string(), path_argument(&scenario_path)].into_iter();
    crate::scenario_scale(&mut scenario_args)?;

    let mut benchmark_args = vec![
        "--physics-report".to_string(),
        path_argument(&physics_path),
        "--scenario-report".to_string(),
        path_argument(&scenario_path),
        "--output".to_string(),
        path_argument(&benchmark_path),
        "--timings-output".to_string(),
        path_argument(&benchmark_timings_path),
        "--no-generate".to_string(),
    ]
    .into_iter();
    crate::benchmark::benchmark(&mut benchmark_args)?;

    let mut behavior_args = vec![
        "--case".to_string(),
        root.join(FAILURE_CASE).to_string_lossy().into_owned(),
        "--json".to_string(),
        path_argument(&behavior_report),
        "--junit".to_string(),
        path_argument(&behavior_junit),
        "--artifacts".to_string(),
        path_argument(&replay_dir),
    ]
    .into_iter();
    crate::behavior_ci(&mut behavior_args)?;
    anyhow::ensure!(
        minimized_replay.is_file(),
        "behavior fixture did not write minimized replay {}",
        minimized_replay.display()
    );

    let mut capsule_create_args = vec![
        "create".to_string(),
        "--replay".to_string(),
        path_argument(&minimized_replay),
        "--evidence".to_string(),
        path_argument(&physics_path),
        "--evidence".to_string(),
        path_argument(&benchmark_path),
        "--output".to_string(),
        path_argument(&capsule_dir),
        "--backend".to_string(),
        "rapier".to_string(),
        "--backend-version".to_string(),
        "0.22".to_string(),
    ]
    .into_iter();
    crate::failure_capsule::run(&mut capsule_create_args)?;

    let mut capsule_verify_args =
        vec!["verify".to_string(), path_argument(&capsule_dir)].into_iter();
    crate::failure_capsule::run(&mut capsule_verify_args)?;

    let entries = vec![
        validate_json_artifact(
            "benchmark_report",
            "rne_benchmark_report",
            1,
            &benchmark_path,
            staging,
            Some(("kind", "rne_benchmark_report")),
            None,
        )?,
        validate_json_artifact(
            "capability_report",
            "rne_capability_report",
            1,
            &capability_path,
            staging,
            Some(("kind", "rne_capability_report")),
            None,
        )?,
        validate_capsule_artifact(&capsule_dir.join("capsule.json"), staging)?,
        validate_json_artifact(
            "physics_conformance",
            "rne_physics_conformance_report",
            u32::from(PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION),
            &physics_path,
            staging,
            None,
            Some(("all_passed", true)),
        )?,
    ];
    let manifest = EvidenceManifest {
        kind: EVIDENCE_MANIFEST_KIND.to_string(),
        schema_version: EVIDENCE_MANIFEST_SCHEMA_VERSION,
        engine_version: crate::RELEASE_VERSION.to_string(),
        source_commit: git_commit(root)?,
        all_passed: true,
        artifacts: entries,
    };
    validate_manifest(&manifest)?;
    write_json(&staging.join("manifest.json"), &manifest)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_json_artifact(
    id: &str,
    kind: &str,
    schema_version: u32,
    path: &Path,
    staging: &Path,
    expected_string: Option<(&str, &str)>,
    expected_bool: Option<(&str, bool)>,
) -> Result<EvidenceArtifact> {
    let bytes = fs::read(path).with_context(|| format!("read evidence {}", path.display()))?;
    let value: Value = serde_json::from_slice(&bytes)
        .with_context(|| format!("parse evidence {}", path.display()))?;
    anyhow::ensure!(
        value.get("schema_version").and_then(Value::as_u64) == Some(u64::from(schema_version)),
        "{} schema_version mismatch",
        path.display()
    );
    if let Some((field, expected)) = expected_string {
        anyhow::ensure!(
            value.get(field).and_then(Value::as_str) == Some(expected),
            "{} {field} mismatch",
            path.display()
        );
    }
    if let Some((field, expected)) = expected_bool {
        anyhow::ensure!(
            value.get(field).and_then(Value::as_bool) == Some(expected),
            "{} {field} mismatch",
            path.display()
        );
    }
    artifact_entry(id, kind, schema_version, path, staging, &bytes)
}

fn validate_capsule_artifact(path: &Path, staging: &Path) -> Result<EvidenceArtifact> {
    let bytes = fs::read(path).with_context(|| format!("read capsule {}", path.display()))?;
    let capsule: FailureCapsule = serde_json::from_slice(&bytes)
        .with_context(|| format!("parse capsule {}", path.display()))?;
    capsule
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid generated capsule: {error}"))?;
    artifact_entry(
        "failure_capsule",
        rne_log::FAILURE_CAPSULE_KIND,
        rne_log::FAILURE_CAPSULE_SCHEMA_VERSION,
        path,
        staging,
        &bytes,
    )
}

fn artifact_entry(
    id: &str,
    kind: &str,
    schema_version: u32,
    path: &Path,
    staging: &Path,
    bytes: &[u8],
) -> Result<EvidenceArtifact> {
    let relative = path
        .strip_prefix(staging)
        .with_context(|| format!("evidence path escaped staging root: {}", path.display()))?;
    let path = relative.to_string_lossy().replace('\\', "/");
    Ok(EvidenceArtifact {
        id: id.to_string(),
        kind: kind.to_string(),
        schema_version,
        path,
        sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
    })
}

fn validate_manifest(manifest: &EvidenceManifest) -> Result<()> {
    anyhow::ensure!(
        manifest.kind == EVIDENCE_MANIFEST_KIND,
        "evidence manifest kind mismatch"
    );
    anyhow::ensure!(
        manifest.schema_version == EVIDENCE_MANIFEST_SCHEMA_VERSION,
        "evidence manifest schema mismatch"
    );
    anyhow::ensure!(manifest.all_passed, "evidence manifest did not pass");
    anyhow::ensure!(
        !manifest.engine_version.trim().is_empty() && !manifest.source_commit.trim().is_empty(),
        "evidence manifest provenance must not be empty"
    );
    let ids = manifest
        .artifacts
        .iter()
        .map(|artifact| artifact.id.as_str())
        .collect::<Vec<_>>();
    anyhow::ensure!(
        ids == REQUIRED_ARTIFACT_IDS,
        "evidence artifacts must be the fixed ordered set {REQUIRED_ARTIFACT_IDS:?}"
    );
    for artifact in &manifest.artifacts {
        anyhow::ensure!(
            !artifact.kind.trim().is_empty() && artifact.schema_version > 0,
            "evidence artifact metadata is incomplete: {}",
            artifact.id
        );
        anyhow::ensure!(
            !artifact.path.is_empty()
                && !artifact.path.starts_with('/')
                && !artifact.path.contains('\\')
                && !artifact.path.split('/').any(|part| part == ".."),
            "evidence artifact path is unsafe: {}",
            artifact.path
        );
        let digest = artifact.sha256.strip_prefix("sha256:").unwrap_or_default();
        anyhow::ensure!(
            digest.len() == 64
                && digest
                    .bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
            "evidence artifact digest is not canonical SHA-256: {}",
            artifact.id
        );
    }
    Ok(())
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write evidence {}", path.display()))
}

fn path_argument(path: &Path) -> String {
    path.to_string_lossy().into_owned()
}

fn git_commit(root: &Path) -> Result<String> {
    let output = Command::new("git")
        .current_dir(root)
        .args(["rev-parse", "HEAD"])
        .output()
        .context("run git rev-parse HEAD for evidence manifest")?;
    anyhow::ensure!(
        output.status.success(),
        "git rev-parse HEAD failed: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    let commit = String::from_utf8(output.stdout)?.trim().to_string();
    anyhow::ensure!(!commit.is_empty(), "git rev-parse HEAD returned no commit");
    Ok(commit)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn artifact(id: &str, path: &str, byte: char) -> EvidenceArtifact {
        let kind = match id {
            "benchmark_report" => "rne_benchmark_report",
            "capability_report" => "rne_capability_report",
            "failure_capsule" => "rne_failure_capsule",
            "physics_conformance" => "rne_physics_conformance_report",
            other => panic!("unexpected fixture artifact {other}"),
        };
        EvidenceArtifact {
            id: id.to_string(),
            kind: kind.to_string(),
            schema_version: if id == "physics_conformance" {
                u32::from(PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION)
            } else {
                1
            },
            path: path.to_string(),
            sha256: format!("sha256:{}", byte.to_string().repeat(64)),
        }
    }

    fn manifest() -> EvidenceManifest {
        EvidenceManifest {
            kind: EVIDENCE_MANIFEST_KIND.to_string(),
            schema_version: EVIDENCE_MANIFEST_SCHEMA_VERSION,
            engine_version: "0.1.0".to_string(),
            source_commit: "0123456789012345678901234567890123456789".to_string(),
            all_passed: true,
            artifacts: vec![
                artifact("benchmark_report", "benchmark-report.json", 'a'),
                artifact("capability_report", "capability-report.json", 'b'),
                artifact("failure_capsule", "failure-capsule/capsule.json", 'c'),
                artifact("physics_conformance", "physics-conformance.json", 'd'),
            ],
        }
    }

    #[test]
    fn manifest_has_stable_golden_shape() {
        let manifest = manifest();
        validate_manifest(&manifest).expect("valid fixture manifest");
        let serialized = serde_json::to_string_pretty(&manifest).expect("serialize manifest");
        let golden = include_str!("../../tests/golden/evidence/evidence-manifest-v1.json");
        assert_eq!(serialized, golden.trim_end());
        let decoded: EvidenceManifest = serde_json::from_str(golden).expect("parse golden");
        assert_eq!(decoded, manifest);
    }

    #[test]
    fn manifest_rejects_reordering_and_bad_digest() {
        let mut reordered = manifest();
        reordered.artifacts.swap(0, 1);
        assert!(validate_manifest(&reordered).is_err());

        let mut bad_digest = manifest();
        bad_digest.artifacts[0].sha256 = "sha256:ABC".to_string();
        assert!(validate_manifest(&bad_digest).is_err());
    }

    #[test]
    fn manifest_schema_rejects_unknown_fields() {
        let mut value = serde_json::to_value(manifest()).expect("manifest JSON");
        value["unexpected"] = Value::Bool(true);
        assert!(serde_json::from_value::<EvidenceManifest>(value).is_err());
    }

    #[test]
    fn options_resolve_relative_output_and_help() {
        let root = tempdir().expect("root");
        let mut args = ["--output", "artifacts/custom"]
            .into_iter()
            .map(str::to_string);
        let options = parse_options(&mut args, root.path())
            .expect("parse options")
            .expect("not help");
        assert_eq!(options.output, root.path().join("artifacts/custom"));

        let mut help = ["--help"].into_iter().map(str::to_string);
        assert!(parse_options(&mut help, root.path()).unwrap().is_none());
    }
}
