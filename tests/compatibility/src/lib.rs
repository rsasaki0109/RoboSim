//! Cross-version compatibility fixture verification for RNE release artifacts.
//!
//! The suite consumes the installed `release/compatibility-fixtures.toml`
//! registry. Every registered JSON artifact must still pass its typed reader,
//! while deterministic mutations prove that an unsupported schema and an
//! unknown top-level field fail closed.

#![deny(missing_docs)]

use anyhow::{bail, ensure, Context};
use rne_ai::{PortableBatchCheckpoint, PortableBatchOperation, TaskSpec};
use rne_data::{DatasetManifest, DepthPairEvaluationReport};
use rne_hardware_gateway::mock::MockConformanceReport;
use rne_log::{FailureCapsule, ReplayArtifact};
use rne_physics_conformance::ExternalPhysicsBackendConformanceReport;
use rne_physics_conformance_suite::ConformanceReport;
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

const FIXTURE_SPECS: [FixtureSpec; 9] = [
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
        id: "hardware_mock_conformance_v1",
        contract: "hardware_mock_conformance",
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
        id: "task_spec_v1",
        contract: "task_spec",
        schema_version: 1,
        version_field: "schema_version",
    },
];

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
                    "accepted; future schema and unknown field rejected".to_string()
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

    validate_typed(spec, value.clone()).context("accepted fixture was rejected")?;

    let mut future = value.clone();
    let future_object = future
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    future_object.insert(
        spec.version_field.to_string(),
        Value::from(u64::from(spec.schema_version) + 10_000),
    );
    let future_schema_rejected = validate_typed(spec, future).is_err();

    let mut unknown = value;
    let unknown_object = unknown
        .as_object_mut()
        .context("compatibility fixture must be a JSON object")?;
    unknown_object.insert(
        "rne_unknown_compatibility_field".to_string(),
        Value::Bool(true),
    );
    let unknown_field_rejected = validate_typed(spec, unknown).is_err();
    Ok((true, future_schema_rejected, unknown_field_rejected))
}

fn validate_typed(spec: FixtureSpec, value: Value) -> anyhow::Result<()> {
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
        "dataset_bundle" => {
            let fixture: DatasetManifest = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "dataset_offline_evaluation" => {
            let fixture: DepthPairEvaluationReport = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "failure_capsule" => {
            let fixture: FailureCapsule = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "generic_replay" => {
            let fixture: ReplayArtifact = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        "hardware_mock_conformance" => {
            let fixture: MockConformanceReport = serde_json::from_value(value)?;
            fixture.validate()?;
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
        "task_spec" => {
            let fixture: TaskSpec = serde_json::from_value(value)?;
            fixture.validate()?;
        }
        other => bail!("unsupported compatibility contract {other}"),
    }
    Ok(())
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
