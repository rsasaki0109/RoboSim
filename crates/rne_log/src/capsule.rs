//! Backend-neutral failure capsule metadata.
//!
//! A failure capsule is a small, immutable envelope around a failed run.  It
//! records enough stable metadata for a later command to locate and verify
//! replay/evidence files without putting an action schema, a physics handle,
//! or archive bytes into the envelope itself.  The referenced files remain
//! transportable as a directory, archive, or remote object store in a future
//! format.

use rne_core::{DeterminismContract, DeterminismContractError};
use serde::{Deserialize, Serialize};
use std::fmt;
use thiserror::Error;

/// Current schema version for [`FailureCapsule`].
pub const FAILURE_CAPSULE_SCHEMA_VERSION: u32 = 1;

/// Stable serialized kind discriminator for [`FailureCapsule`].
pub const FAILURE_CAPSULE_KIND: &str = "rne_failure_capsule";

/// Alias retained for callers that use the shorter version constant name.
pub const FAILURE_CAPSULE_VERSION: u32 = FAILURE_CAPSULE_SCHEMA_VERSION;

const SHA256_HEX_LENGTH: usize = 64;

/// Errors raised when a failure capsule or one of its artifact references is
/// not a valid, canonical declaration.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum FailureCapsuleError {
    /// The capsule uses a schema version unsupported by this crate.
    #[error("unsupported failure capsule schema: expected {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Schema version supported by this crate.
        expected: u32,
        /// Schema version found in the capsule.
        actual: u32,
    },
    /// The capsule kind discriminator does not identify a failure capsule.
    #[error("invalid failure capsule kind: expected `{expected}`, got `{actual}`")]
    InvalidKind {
        /// Kind discriminator required by this crate.
        expected: String,
        /// Kind discriminator found in the capsule.
        actual: String,
    },
    /// A required metadata identifier is empty or only whitespace.
    #[error("{field} must not be empty")]
    EmptyIdentifier {
        /// Fully-qualified metadata field name.
        field: &'static str,
    },
    /// The determinism contract embedded in the capsule is invalid.
    #[error("invalid determinism contract: {0}")]
    InvalidDeterminismContract(#[source] DeterminismContractError),
    /// The failure names a different contract than the embedded declaration.
    #[error("failure contract `{failure}` does not match determinism contract `{determinism}`")]
    FailureContractMismatch {
        /// Contract name recorded by the failure metadata.
        failure: String,
        /// Contract name recorded by the determinism declaration.
        determinism: String,
    },
    /// An artifact reference has an invalid path.
    #[error("invalid artifact path `{path}`: {reason}")]
    InvalidArtifactPath {
        /// Path supplied by the artifact reference.
        path: String,
        /// Reason the path is invalid.
        reason: ArtifactPathError,
    },
    /// An artifact path is valid but is not in canonical slash-separated form.
    #[error("artifact path `{path}` is not normalized; expected `{expected}`")]
    NonCanonicalArtifactPath {
        /// Path supplied by the artifact reference.
        path: String,
        /// Canonical relative path expected by the schema.
        expected: String,
    },
    /// An artifact digest is not exactly 64 lowercase hexadecimal characters.
    #[error(
        "invalid SHA-256 digest for artifact `{path}`: expected 64 lowercase hexadecimal characters"
    )]
    InvalidArtifactDigest {
        /// Path identifying the artifact whose digest is invalid.
        path: String,
    },
    /// A build metadata SHA-256 digest is not canonical.
    #[error(
        "invalid SHA-256 digest for build field `{field}`: expected 64 lowercase hexadecimal characters"
    )]
    InvalidBuildDigest {
        /// Build field containing the invalid digest.
        field: &'static str,
    },
    /// An artifact kind or role has no stable identifier.
    #[error("{field} must not be empty for artifact `{path}`")]
    EmptyArtifactIdentifier {
        /// Artifact field that is empty.
        field: &'static str,
        /// Path identifying the artifact.
        path: String,
    },
    /// An artifact schema version is zero and therefore unspecified.
    #[error("artifact `{path}` schema_version must be greater than zero")]
    InvalidArtifactSchemaVersion {
        /// Path identifying the artifact.
        path: String,
    },
    /// Two artifact references point at the same relative path.
    #[error("duplicate artifact path `{path}`")]
    DuplicateArtifactPath {
        /// Duplicated relative artifact path.
        path: String,
    },
    /// Artifact references are not strictly sorted by canonical relative path.
    #[error("artifact references are not sorted: `{previous}` must precede `{current}`")]
    UnsortedArtifacts {
        /// Previous path in the serialized list.
        previous: String,
        /// Current path in the serialized list.
        current: String,
    },
    /// A capsule contains no replay or evidence references.
    #[error("failure capsule must contain at least one artifact reference")]
    EmptyArtifacts,
    /// A capsule does not contain the replay artifact required for reproduction.
    #[error("failure capsule must contain an artifact with role `replay`")]
    MissingReplayArtifact,
    /// A count field is zero where a non-empty run is required.
    #[error("{field} must be greater than zero")]
    ZeroCount {
        /// Fully-qualified count field name.
        field: &'static str,
    },
    /// A failure refers to a step outside the recorded run.
    #[error("failure step {step} is outside run step_count {step_count}")]
    FailureStepOutOfRange {
        /// First failing simulation step.
        step: u64,
        /// Number of recorded simulation steps.
        step_count: u64,
    },
    /// The action count cannot exceed the number of recorded steps.
    #[error("run action_count {action_count} exceeds step_count {step_count}")]
    ActionCountExceedsSteps {
        /// Number of recorded actions.
        action_count: u64,
        /// Number of recorded steps.
        step_count: u64,
    },
    /// The determinism scope cannot fit inside the recorded run.
    #[error("determinism scope ends at step {scope_end}, outside run step_count {step_count}")]
    DeterminismScopeOutOfRange {
        /// Inclusive final step in the contract scope.
        scope_end: u64,
        /// Number of recorded simulation steps.
        step_count: u64,
    },
    /// Minimization metadata describes a larger result than its source.
    #[error("minimization {field} {minimized} exceeds source {original}")]
    MinimizationIncreased {
        /// Count being compared.
        field: &'static str,
        /// Minimized count.
        minimized: u64,
        /// Original count.
        original: u64,
    },
    /// The current run counts do not match minimization metadata.
    #[error("minimization {field} does not match current run: expected {expected}, got {actual}")]
    MinimizationCurrentRunMismatch {
        /// Count field being compared.
        field: &'static str,
        /// Count declared by the current run.
        expected: u64,
        /// Count declared by minimization metadata.
        actual: u64,
    },
}

/// The reason a relative artifact path cannot be accepted.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ArtifactPathError {
    /// The path is empty after trimming and normalization.
    #[error("path must not be empty")]
    Empty,
    /// The path is absolute or contains a drive/UNC prefix.
    #[error("path must be relative")]
    Absolute,
    /// The path contains a parent traversal component.
    #[error("parent traversal (`..`) is not allowed")]
    ParentTraversal,
    /// The path contains a NUL byte.
    #[error("path must not contain NUL")]
    Nul,
}

/// A content-addressed replay or evidence file referenced by a capsule.
///
/// `path` is always a relative, canonical, slash-separated path inside the
/// future capsule transport.  The reference does not read the path or assert
/// that a file exists; an `xtask` verifier can perform that check later.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactRef {
    /// Stable consumer-defined role, such as `replay` or `evidence`.
    pub role: String,
    /// Stable artifact format kind, such as `rne_replay` or `json`.
    pub kind: String,
    /// Schema version of the referenced artifact format.
    pub schema_version: u32,
    /// Canonical relative slash-separated path inside the capsule transport.
    pub path: String,
    /// Lowercase hexadecimal SHA-256 digest of the file bytes.
    pub sha256: String,
}

impl ArtifactRef {
    /// Creates and validates an artifact reference.
    ///
    /// Separators are normalized to `/` and redundant `.` components are
    /// removed. Absolute paths and parent traversal are rejected.
    pub fn new(
        role: impl Into<String>,
        kind: impl Into<String>,
        schema_version: u32,
        path: impl AsRef<str>,
        sha256: impl Into<String>,
    ) -> Result<Self, FailureCapsuleError> {
        let path = normalize_relative_path(path.as_ref()).map_err(|reason| {
            FailureCapsuleError::InvalidArtifactPath {
                path: path.as_ref().to_string(),
                reason,
            }
        })?;
        let artifact = Self {
            role: role.into(),
            kind: kind.into(),
            schema_version,
            path,
            sha256: sha256.into().to_ascii_lowercase(),
        };
        artifact.validate()?;
        Ok(artifact)
    }

    /// Creates a reference using the conventional `(path, role, kind, ... )`
    /// naming at call sites.
    pub fn from_path(
        path: impl AsRef<str>,
        role: impl Into<String>,
        kind: impl Into<String>,
        schema_version: u32,
        sha256: impl Into<String>,
    ) -> Result<Self, FailureCapsuleError> {
        Self::new(role, kind, schema_version, path, sha256)
    }

    /// Validates canonical path, identifier, schema-version, and digest rules.
    pub fn validate(&self) -> Result<(), FailureCapsuleError> {
        validate_artifact_identifier("artifact.role", &self.role, &self.path)?;
        validate_artifact_identifier("artifact.kind", &self.kind, &self.path)?;
        if self.schema_version == 0 {
            return Err(FailureCapsuleError::InvalidArtifactSchemaVersion {
                path: self.path.clone(),
            });
        }

        let normalized = normalize_relative_path(&self.path).map_err(|reason| {
            FailureCapsuleError::InvalidArtifactPath {
                path: self.path.clone(),
                reason,
            }
        })?;
        if normalized != self.path {
            return Err(FailureCapsuleError::NonCanonicalArtifactPath {
                path: self.path.clone(),
                expected: normalized,
            });
        }
        if !is_canonical_sha256(&self.sha256) {
            return Err(FailureCapsuleError::InvalidArtifactDigest {
                path: self.path.clone(),
            });
        }
        Ok(())
    }
}

fn is_canonical_sha256(value: &str) -> bool {
    value.len() == SHA256_HEX_LENGTH
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_artifact_identifier(
    field: &'static str,
    value: &str,
    path: &str,
) -> Result<(), FailureCapsuleError> {
    if value.trim().is_empty() {
        return Err(FailureCapsuleError::EmptyArtifactIdentifier {
            field,
            path: path.to_string(),
        });
    }
    Ok(())
}

/// Normalizes a relative path for use in an [`ArtifactRef`].
pub fn normalize_relative_path(path: &str) -> Result<String, ArtifactPathError> {
    if path.as_bytes().contains(&0) {
        return Err(ArtifactPathError::Nul);
    }
    if path.trim().is_empty() {
        return Err(ArtifactPathError::Empty);
    }

    let slash_path = path.replace('\\', "/");
    let has_drive_prefix = slash_path.len() >= 2
        && slash_path.as_bytes()[0].is_ascii_alphabetic()
        && slash_path.as_bytes()[1] == b':';
    if slash_path.starts_with('/') || has_drive_prefix {
        return Err(ArtifactPathError::Absolute);
    }

    let mut components = Vec::new();
    for component in slash_path.split('/') {
        match component {
            "" | "." => {}
            ".." => return Err(ArtifactPathError::ParentTraversal),
            value => components.push(value),
        }
    }
    if components.is_empty() {
        return Err(ArtifactPathError::Empty);
    }
    Ok(components.join("/"))
}

/// Metadata describing the first failing observation in a run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FailureMetadata {
    /// Stable failure identifier, usually a contract or test-case identifier.
    pub id: String,
    /// Name of the determinism contract whose evidence failed.
    pub contract: String,
    /// Human-readable, stable summary of the failure.
    pub message: String,
    /// First failing simulation step.
    pub step: u64,
    /// Simulation timestamp at the first failing step in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Stable state digest recorded at the first failing step.
    pub state_digest: u64,
}

impl FailureMetadata {
    /// Creates failure metadata. The enclosing capsule performs validation.
    pub fn new(
        id: impl Into<String>,
        contract: impl Into<String>,
        message: impl Into<String>,
        step: u64,
        sim_time_ticks: u64,
        state_digest: u64,
    ) -> Self {
        Self {
            id: id.into(),
            contract: contract.into(),
            message: message.into(),
            step,
            sim_time_ticks,
            state_digest,
        }
    }
}

/// Stable identity and count metadata for one simulation run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunMetadata {
    /// Stable run identifier.
    pub id: String,
    /// Stable scenario or test-case identifier.
    pub scenario: String,
    /// Explicit deterministic world seed.
    pub seed: u64,
    /// Fixed simulation timestep in nanosecond ticks.
    pub fixed_delta_ticks: u64,
    /// Number of recorded simulation steps.
    pub step_count: u64,
    /// Number of recorded action entries.
    pub action_count: u64,
}

impl RunMetadata {
    /// Creates run metadata. The enclosing capsule performs validation.
    pub fn new(
        id: impl Into<String>,
        scenario: impl Into<String>,
        seed: u64,
        fixed_delta_ticks: u64,
        step_count: u64,
        action_count: u64,
    ) -> Self {
        Self {
            id: id.into(),
            scenario: scenario.into(),
            seed,
            fixed_delta_ticks,
            step_count,
            action_count,
        }
    }
}

/// Build provenance captured at capsule creation time.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BuildMetadata {
    /// RNE engine/package version that produced the run.
    pub engine_version: String,
    /// Exact source revision when known.
    pub git_commit: String,
    /// Build profile, such as `debug` or `release`.
    pub profile: String,
    /// Target triple used to build the engine.
    pub target_triple: String,
    /// Rust compiler version used to build the engine.
    pub rustc_version: String,
    /// Canonical SHA-256 digest of the exact Cargo.lock used for the build.
    pub cargo_lock_sha256: String,
}

impl BuildMetadata {
    /// Creates build provenance. The enclosing capsule performs validation.
    pub fn new(
        engine_version: impl Into<String>,
        git_commit: impl Into<String>,
        profile: impl Into<String>,
        target_triple: impl Into<String>,
        rustc_version: impl Into<String>,
        cargo_lock_sha256: impl Into<String>,
    ) -> Self {
        Self {
            engine_version: engine_version.into(),
            git_commit: git_commit.into(),
            profile: profile.into(),
            target_triple: target_triple.into(),
            rustc_version: rustc_version.into(),
            cargo_lock_sha256: cargo_lock_sha256.into().to_ascii_lowercase(),
        }
    }
}

/// Backend provenance captured without exposing backend-specific handles.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendMetadata {
    /// Stable backend family identifier, such as `rapier` or `analytic`.
    pub name: String,
    /// Backend implementation version or compatibility identifier.
    pub version: String,
}

impl BackendMetadata {
    /// Creates backend provenance. The enclosing capsule performs validation.
    pub fn new(name: impl Into<String>, version: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            version: version.into(),
        }
    }
}

/// Deterministic provenance for a minimized failure run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MinimizationMetadata {
    /// Run identifier of the source failure before minimization.
    pub source_run_id: String,
    /// Number of reproduction attempts made by the minimizer.
    pub attempts: u32,
    /// Number of steps in the source failure.
    pub original_step_count: u64,
    /// Number of steps in the minimized failure.
    pub minimized_step_count: u64,
    /// Number of actions in the source failure.
    pub original_action_count: u64,
    /// Number of actions in the minimized failure.
    pub minimized_action_count: u64,
}

impl MinimizationMetadata {
    /// Creates minimization provenance. The enclosing capsule performs
    /// validation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        source_run_id: impl Into<String>,
        attempts: u32,
        original_step_count: u64,
        minimized_step_count: u64,
        original_action_count: u64,
        minimized_action_count: u64,
    ) -> Self {
        Self {
            source_run_id: source_run_id.into(),
            attempts,
            original_step_count,
            minimized_step_count,
            original_action_count,
            minimized_action_count,
        }
    }
}

/// Small, backend-neutral envelope for a reproducible simulation failure.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FailureCapsule {
    /// Capsule schema version.
    pub schema_version: u32,
    /// Stable capsule kind discriminator.
    pub kind: String,
    /// First failure metadata.
    pub failure: FailureMetadata,
    /// Run identity, deterministic seed, and replay counts.
    pub run: RunMetadata,
    /// Build provenance.
    pub build: BuildMetadata,
    /// Backend provenance without backend-specific types.
    pub backend: BackendMetadata,
    /// Determinism promise for the referenced evidence.
    pub determinism: DeterminismContract,
    /// Optional minimization provenance for a minimized failure.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minimization: Option<MinimizationMetadata>,
    /// Sorted replay/evidence file references.
    pub artifacts: Vec<ArtifactRef>,
}

impl FailureCapsule {
    /// Creates and validates a failure capsule with no minimization metadata.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        failure: FailureMetadata,
        run: RunMetadata,
        build: BuildMetadata,
        backend: BackendMetadata,
        determinism: DeterminismContract,
        artifacts: Vec<ArtifactRef>,
    ) -> Result<Self, FailureCapsuleError> {
        let capsule = Self {
            schema_version: FAILURE_CAPSULE_SCHEMA_VERSION,
            kind: FAILURE_CAPSULE_KIND.to_string(),
            failure,
            run,
            build,
            backend,
            determinism,
            minimization: None,
            artifacts,
        };
        capsule.validate()?;
        Ok(capsule)
    }

    /// Adds validated minimization provenance and returns the updated capsule.
    pub fn with_minimization(
        mut self,
        minimization: MinimizationMetadata,
    ) -> Result<Self, FailureCapsuleError> {
        self.minimization = Some(minimization);
        self.validate()?;
        Ok(self)
    }

    /// Validates schema, metadata, determinism, count, and artifact invariants.
    ///
    /// Validation is deliberately filesystem-free. It checks the declared
    /// paths and digests but never opens, hashes, or otherwise reads a file.
    pub fn validate(&self) -> Result<(), FailureCapsuleError> {
        if self.schema_version != FAILURE_CAPSULE_SCHEMA_VERSION {
            return Err(FailureCapsuleError::UnsupportedSchemaVersion {
                expected: FAILURE_CAPSULE_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.kind != FAILURE_CAPSULE_KIND {
            return Err(FailureCapsuleError::InvalidKind {
                expected: FAILURE_CAPSULE_KIND.to_string(),
                actual: self.kind.clone(),
            });
        }

        validate_identifier("failure.id", &self.failure.id)?;
        validate_identifier("failure.contract", &self.failure.contract)?;
        validate_identifier("failure.message", &self.failure.message)?;
        validate_identifier("run.id", &self.run.id)?;
        validate_identifier("run.scenario", &self.run.scenario)?;
        validate_identifier("build.engine_version", &self.build.engine_version)?;
        validate_identifier("build.git_commit", &self.build.git_commit)?;
        validate_identifier("build.profile", &self.build.profile)?;
        validate_identifier("build.target_triple", &self.build.target_triple)?;
        validate_identifier("build.rustc_version", &self.build.rustc_version)?;
        validate_identifier("backend.name", &self.backend.name)?;
        validate_identifier("backend.version", &self.backend.version)?;
        if !is_canonical_sha256(&self.build.cargo_lock_sha256) {
            return Err(FailureCapsuleError::InvalidBuildDigest {
                field: "build.cargo_lock_sha256",
            });
        }

        self.determinism
            .validate()
            .map_err(FailureCapsuleError::InvalidDeterminismContract)?;
        if self.failure.contract != self.determinism.name {
            return Err(FailureCapsuleError::FailureContractMismatch {
                failure: self.failure.contract.clone(),
                determinism: self.determinism.name.clone(),
            });
        }

        if self.run.fixed_delta_ticks == 0 {
            return Err(FailureCapsuleError::ZeroCount {
                field: "run.fixed_delta_ticks",
            });
        }
        if self.run.step_count == 0 {
            return Err(FailureCapsuleError::ZeroCount {
                field: "run.step_count",
            });
        }
        if self.run.action_count > self.run.step_count {
            return Err(FailureCapsuleError::ActionCountExceedsSteps {
                action_count: self.run.action_count,
                step_count: self.run.step_count,
            });
        }
        if self.failure.step >= self.run.step_count {
            return Err(FailureCapsuleError::FailureStepOutOfRange {
                step: self.failure.step,
                step_count: self.run.step_count,
            });
        }
        let scope_end = self
            .determinism
            .scope
            .last_step()
            .expect("validated determinism scope has a finite last step");
        if scope_end >= self.run.step_count {
            return Err(FailureCapsuleError::DeterminismScopeOutOfRange {
                scope_end,
                step_count: self.run.step_count,
            });
        }

        if let Some(minimization) = &self.minimization {
            validate_identifier("minimization.source_run_id", &minimization.source_run_id)?;
            validate_minimization_count(
                "step_count",
                minimization.minimized_step_count,
                minimization.original_step_count,
            )?;
            validate_minimization_count(
                "action_count",
                minimization.minimized_action_count,
                minimization.original_action_count,
            )?;
            if minimization.minimized_step_count != self.run.step_count {
                return Err(FailureCapsuleError::MinimizationCurrentRunMismatch {
                    field: "current step_count",
                    expected: self.run.step_count,
                    actual: minimization.minimized_step_count,
                });
            }
            if minimization.minimized_action_count != self.run.action_count {
                return Err(FailureCapsuleError::MinimizationCurrentRunMismatch {
                    field: "current action_count",
                    expected: self.run.action_count,
                    actual: minimization.minimized_action_count,
                });
            }
        }

        validate_artifacts(&self.artifacts)
    }
}

fn validate_identifier(field: &'static str, value: &str) -> Result<(), FailureCapsuleError> {
    if value.trim().is_empty() {
        Err(FailureCapsuleError::EmptyIdentifier { field })
    } else {
        Ok(())
    }
}

fn validate_minimization_count(
    field: &'static str,
    minimized: u64,
    original: u64,
) -> Result<(), FailureCapsuleError> {
    if minimized > original {
        Err(FailureCapsuleError::MinimizationIncreased {
            field,
            minimized,
            original,
        })
    } else {
        Ok(())
    }
}

fn validate_artifacts(artifacts: &[ArtifactRef]) -> Result<(), FailureCapsuleError> {
    if artifacts.is_empty() {
        return Err(FailureCapsuleError::EmptyArtifacts);
    }
    if !artifacts.iter().any(|artifact| artifact.role == "replay") {
        return Err(FailureCapsuleError::MissingReplayArtifact);
    }
    for artifact in artifacts {
        artifact.validate()?;
    }
    for pair in artifacts.windows(2) {
        match pair[0].path.cmp(&pair[1].path) {
            std::cmp::Ordering::Less => {}
            std::cmp::Ordering::Equal => {
                return Err(FailureCapsuleError::DuplicateArtifactPath {
                    path: pair[1].path.clone(),
                });
            }
            std::cmp::Ordering::Greater => {
                return Err(FailureCapsuleError::UnsortedArtifacts {
                    previous: pair[0].path.clone(),
                    current: pair[1].path.clone(),
                });
            }
        }
    }
    Ok(())
}

impl fmt::Display for FailureCapsule {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}:{}", self.kind, self.run.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_core::{DeterminismContract, DeterminismScope};

    const DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const DIGEST_C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn contract() -> DeterminismContract {
        DeterminismContract::exact(
            "contact_failure",
            DeterminismScope::new("episode", ["world.state"], 0, 3).expect("scope"),
        )
        .expect("contract")
    }

    fn artifact(path: &str, digest: &str) -> ArtifactRef {
        ArtifactRef::new("replay", "rne_replay", 1, path, digest).expect("artifact")
    }

    fn capsule() -> FailureCapsule {
        FailureCapsule::new(
            FailureMetadata::new(
                "failure-1",
                "contact_failure",
                "contact contract violated",
                2,
                20,
                0x1234,
            ),
            RunMetadata::new("run-1", "arm_pick", 7, 10, 3, 2),
            BuildMetadata::new(
                "0.2.0",
                "0123456789abcdef",
                "release",
                "x86_64-pc-windows-msvc",
                "rustc 1.88.0",
                DIGEST_C,
            ),
            BackendMetadata::new("rapier", "0.22"),
            contract(),
            vec![artifact("replays/run-1.rne-replay", DIGEST_A)],
        )
        .expect("capsule")
    }

    #[test]
    fn artifact_path_is_normalized_without_reading_files() {
        let reference = artifact("replays\\.\\run.rne-replay", DIGEST_A);
        assert_eq!(reference.path, "replays/run.rne-replay");
        reference.validate().expect("normalized reference");
    }

    #[test]
    fn artifact_path_rejects_absolute_and_parent_traversal() {
        for path in ["/tmp/replay", "C:\\tmp\\replay", "\\\\server\\replay"] {
            assert!(matches!(
                ArtifactRef::new("replay", "json", 1, path, DIGEST_A),
                Err(FailureCapsuleError::InvalidArtifactPath {
                    reason: ArtifactPathError::Absolute,
                    ..
                })
            ));
        }
        assert!(matches!(
            ArtifactRef::new("replay", "json", 1, "replays/../replay", DIGEST_A),
            Err(FailureCapsuleError::InvalidArtifactPath {
                reason: ArtifactPathError::ParentTraversal,
                ..
            })
        ));
    }

    #[test]
    fn artifact_validation_rejects_bad_digest_and_noncanonical_paths() {
        let mut reference = artifact("replay.rne-replay", DIGEST_A);
        reference.sha256 = "not-a-digest".to_string();
        assert!(matches!(
            reference.validate(),
            Err(FailureCapsuleError::InvalidArtifactDigest { .. })
        ));

        let mut reference = artifact("replay.rne-replay", DIGEST_A);
        reference.path = "replays\\replay.rne-replay".to_string();
        assert!(matches!(
            reference.validate(),
            Err(FailureCapsuleError::NonCanonicalArtifactPath { .. })
        ));

        let mut reference = artifact("replay.rne-replay", DIGEST_A);
        reference.sha256 = DIGEST_A.to_ascii_uppercase();
        assert!(matches!(
            reference.validate(),
            Err(FailureCapsuleError::InvalidArtifactDigest { .. })
        ));
    }

    #[test]
    fn artifact_constructor_canonicalizes_uppercase_digest() {
        let reference = artifact(
            "replay.rne-replay",
            "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        );
        assert_eq!(reference.sha256, DIGEST_A);
    }

    #[test]
    fn capsule_json_roundtrip_has_stable_golden_shape() {
        let capsule = capsule();
        let json = serde_json::to_string_pretty(&capsule).expect("serialize capsule");
        let expected =
            include_str!("../../../tests/golden/evidence/failure-capsule-v1.json").trim_end();
        assert_eq!(json, expected);

        let decoded: FailureCapsule = serde_json::from_str(&json).expect("deserialize capsule");
        assert_eq!(decoded, capsule);
        decoded.validate().expect("round-tripped capsule validates");
    }

    #[test]
    fn capsule_validation_rejects_schema_kind_contract_and_identifiers() {
        let mut value = capsule();
        value.schema_version += 1;
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::UnsupportedSchemaVersion { .. })
        ));

        let mut value = capsule();
        value.kind = "other".to_string();
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::InvalidKind { .. })
        ));

        let mut value = capsule();
        value.determinism.schema_version += 1;
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::InvalidDeterminismContract(_))
        ));

        let mut value = capsule();
        value.run.id.clear();
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::EmptyIdentifier { field: "run.id" })
        ));

        let mut value = capsule();
        value.build.target_triple.clear();
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::EmptyIdentifier {
                field: "build.target_triple"
            })
        ));

        let mut value = capsule();
        value.build.cargo_lock_sha256 = DIGEST_C.to_ascii_uppercase();
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::InvalidBuildDigest {
                field: "build.cargo_lock_sha256"
            })
        ));
    }

    #[test]
    fn capsule_validation_rejects_duplicate_unsorted_and_inconsistent_counts() {
        let mut value = capsule();
        value.artifacts = vec![artifact("z.json", DIGEST_A), artifact("a.json", DIGEST_B)];
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::UnsortedArtifacts { .. })
        ));

        let mut value = capsule();
        value.artifacts = vec![artifact("a.json", DIGEST_A), artifact("a.json", DIGEST_B)];
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::DuplicateArtifactPath { .. })
        ));

        let mut value = capsule();
        value.run.action_count = value.run.step_count + 1;
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::ActionCountExceedsSteps { .. })
        ));

        let mut value = capsule();
        value.artifacts.clear();
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::EmptyArtifacts)
        ));

        let mut value = capsule();
        value.artifacts = vec![
            ArtifactRef::new("evidence", "json", 1, "evidence.json", DIGEST_A).expect("artifact"),
        ];
        assert!(matches!(
            value.validate(),
            Err(FailureCapsuleError::MissingReplayArtifact)
        ));
    }

    #[test]
    fn minimization_metadata_is_optional_and_count_checked() {
        let value = capsule()
            .with_minimization(MinimizationMetadata::new("run-original", 2, 5, 3, 4, 2))
            .expect("valid minimization");
        assert!(value.minimization.is_some());

        let error = capsule()
            .with_minimization(MinimizationMetadata::new("run-original", 2, 5, 4, 4, 2))
            .expect_err("current run count mismatch must be rejected");
        assert!(matches!(
            error,
            FailureCapsuleError::MinimizationCurrentRunMismatch {
                field: "current step_count",
                ..
            }
        ));
    }
}
