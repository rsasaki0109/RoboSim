//! Portable failure metadata for replaying timestamped recorded data.
//!
//! This schema is deliberately separate from the fixed-step simulation
//! [`crate::FailureCapsule`]. It never maps a physical source timestamp onto a
//! simulation tick and never invents a fixed period for nonuniform captures.

use crate::{ArtifactRef, BuildMetadata, FailureCapsuleError};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

/// Current schema version for [`RecordedFailureCapsule`].
pub const RECORDED_FAILURE_CAPSULE_SCHEMA_VERSION: u32 = 1;
/// Stable kind discriminator for [`RecordedFailureCapsule`].
pub const RECORDED_FAILURE_CAPSULE_KIND: &str = "rne_recorded_failure_capsule";

const SHA256_HEX_LENGTH: usize = 64;

/// Validation errors for recorded-data failure capsules.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RecordedFailureCapsuleError {
    /// The schema version is not supported by this crate.
    #[error("unsupported recorded failure capsule schema: expected {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Supported schema version.
        expected: u32,
        /// Supplied schema version.
        actual: u32,
    },
    /// The serialized kind is not a recorded failure capsule.
    #[error("invalid recorded failure capsule kind: `{0}`")]
    InvalidKind(String),
    /// A semantic invariant is not satisfied.
    #[error("invalid recorded failure capsule: {0}")]
    Invalid(&'static str),
    /// A shared artifact reference is malformed.
    #[error(transparent)]
    Artifact(#[from] FailureCapsuleError),
}

/// Explicit integer unit used by the source timestamp channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecordedTimeUnit {
    /// Source timestamp ticks are nanoseconds.
    Nanoseconds,
    /// Source timestamp ticks are microseconds.
    Microseconds,
    /// Source timestamp ticks are milliseconds.
    Milliseconds,
}

/// Declared sampling behavior of the recorded source clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum RecordedClockSampling {
    /// Every adjacent record is separated by one exact period.
    Uniform {
        /// Exact positive source-clock period.
        period_ticks: u64,
    },
    /// Adjacent source timestamps are strictly increasing but not uniformly spaced.
    Nonuniform {
        /// Smallest positive adjacent source-clock interval in the retained run.
        minimum_delta_ticks: u64,
        /// Largest adjacent source-clock interval in the retained run.
        maximum_delta_ticks: u64,
    },
}

/// Source-clock contract for replaying a recorded input artifact.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedClockMetadata {
    /// Stable source clock domain, such as `device_monotonic`.
    pub domain: String,
    /// Explicit unit of every source timestamp tick.
    pub unit: RecordedTimeUnit,
    /// Channel in the recorded input that owns the source timestamp.
    pub timestamp_channel: String,
    /// Uniform or explicitly nonuniform source sampling declaration.
    pub sampling: RecordedClockSampling,
}

/// First failing observation in recorded-source coordinates.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedFailureMetadata {
    /// Stable failure identifier.
    pub id: String,
    /// Frozen evaluation contract name.
    pub contract: String,
    /// Stable human-readable failure summary.
    pub message: String,
    /// Zero-based record index of the first failure.
    pub record_index: u64,
    /// Exact source timestamp at `record_index`, in the declared clock unit.
    pub source_time_ticks: u64,
    /// SHA-256 of the canonical controller-visible observation at the failure.
    pub observation_sha256: String,
}

/// Identity and counts for one recorded-source evaluation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedRunMetadata {
    /// Stable run identifier.
    pub id: String,
    /// Stable source/dataset identity.
    pub source_id: String,
    /// Number of complete records in the retained input run.
    pub record_count: u64,
    /// Number of records evaluated through and including the first failure.
    pub evaluated_record_count: u64,
    /// Number of recorded controller commands, which may be zero.
    pub command_count: u64,
}

/// Evaluator identity bound without pretending it is a physics backend.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedEvaluatorMetadata {
    /// Stable evaluator implementation name.
    pub name: String,
    /// Evaluator implementation version or source revision.
    pub version: String,
    /// SHA-256 of the frozen evaluation protocol.
    pub protocol_sha256: String,
    /// Explicit seed when evaluation uses randomness; absent for deterministic evaluators.
    pub random_seed: Option<u64>,
}

/// Backend-neutral envelope around a failed recorded-data evaluation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedFailureCapsule {
    /// Capsule schema version.
    pub schema_version: u32,
    /// Stable kind discriminator.
    pub kind: String,
    /// First failure in recorded-source coordinates.
    pub failure: RecordedFailureMetadata,
    /// Run identity and exact evaluation counts.
    pub run: RecordedRunMetadata,
    /// Source timestamp semantics, including explicit nonuniformity.
    pub clock: RecordedClockMetadata,
    /// Build provenance for the evaluator.
    pub build: BuildMetadata,
    /// Evaluator and frozen-protocol identity.
    pub evaluator: RecordedEvaluatorMetadata,
    /// Sorted content-addressed input, evaluation, and protocol artifacts.
    pub artifacts: Vec<ArtifactRef>,
}

impl RecordedFailureCapsule {
    /// Construct and validate a recorded-data failure capsule.
    pub fn new(
        failure: RecordedFailureMetadata,
        run: RecordedRunMetadata,
        clock: RecordedClockMetadata,
        build: BuildMetadata,
        evaluator: RecordedEvaluatorMetadata,
        artifacts: Vec<ArtifactRef>,
    ) -> Result<Self, RecordedFailureCapsuleError> {
        let capsule = Self {
            schema_version: RECORDED_FAILURE_CAPSULE_SCHEMA_VERSION,
            kind: RECORDED_FAILURE_CAPSULE_KIND.to_owned(),
            failure,
            run,
            clock,
            build,
            evaluator,
            artifacts,
        };
        capsule.validate()?;
        Ok(capsule)
    }

    /// Validate clock, provenance, count, digest, role, and ordering invariants.
    pub fn validate(&self) -> Result<(), RecordedFailureCapsuleError> {
        if self.schema_version != RECORDED_FAILURE_CAPSULE_SCHEMA_VERSION {
            return Err(RecordedFailureCapsuleError::UnsupportedSchemaVersion {
                expected: RECORDED_FAILURE_CAPSULE_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.kind != RECORDED_FAILURE_CAPSULE_KIND {
            return Err(RecordedFailureCapsuleError::InvalidKind(self.kind.clone()));
        }
        for value in [
            &self.failure.id,
            &self.failure.contract,
            &self.failure.message,
            &self.run.id,
            &self.run.source_id,
            &self.clock.domain,
            &self.clock.timestamp_channel,
            &self.build.engine_version,
            &self.build.git_commit,
            &self.build.profile,
            &self.build.target_triple,
            &self.build.rustc_version,
            &self.evaluator.name,
            &self.evaluator.version,
        ] {
            if value.trim().is_empty() {
                return Err(RecordedFailureCapsuleError::Invalid(
                    "required identifier is empty",
                ));
            }
        }
        if !is_sha256(&self.failure.observation_sha256)
            || !is_sha256(&self.build.cargo_lock_sha256)
            || !is_sha256(&self.evaluator.protocol_sha256)
        {
            return Err(RecordedFailureCapsuleError::Invalid(
                "digest is not canonical lowercase SHA-256",
            ));
        }
        if self.run.record_count == 0
            || self.run.evaluated_record_count == 0
            || self.run.evaluated_record_count > self.run.record_count
            || self.failure.record_index.checked_add(1) != Some(self.run.evaluated_record_count)
        {
            return Err(RecordedFailureCapsuleError::Invalid(
                "record counts or first-failure index are inconsistent",
            ));
        }
        match self.clock.sampling {
            RecordedClockSampling::Uniform { period_ticks: 0 } => {
                return Err(RecordedFailureCapsuleError::Invalid(
                    "uniform source period must be positive",
                ));
            }
            RecordedClockSampling::Nonuniform {
                minimum_delta_ticks,
                maximum_delta_ticks,
            } if minimum_delta_ticks == 0 || minimum_delta_ticks >= maximum_delta_ticks => {
                return Err(RecordedFailureCapsuleError::Invalid(
                    "nonuniform source deltas must be positive and distinct",
                ));
            }
            _ => {}
        }
        if self.artifacts.is_empty() {
            return Err(RecordedFailureCapsuleError::Invalid(
                "artifact references are empty",
            ));
        }
        let mut paths = BTreeSet::new();
        let mut roles = BTreeSet::new();
        let mut previous = None;
        for artifact in &self.artifacts {
            artifact.validate()?;
            if let Some(previous_path) = previous {
                if previous_path >= artifact.path.as_str() {
                    return Err(RecordedFailureCapsuleError::Invalid(
                        "artifact paths are not strictly sorted",
                    ));
                }
            }
            previous = Some(artifact.path.as_str());
            if !paths.insert(&artifact.path) {
                return Err(RecordedFailureCapsuleError::Invalid(
                    "artifact path is duplicated",
                ));
            }
            roles.insert(artifact.role.as_str());
        }
        for required in ["evaluation", "protocol", "recorded_input"] {
            if !roles.contains(required) {
                return Err(RecordedFailureCapsuleError::Invalid(
                    "required recorded replay artifact role is missing",
                ));
            }
        }
        let protocols = self
            .artifacts
            .iter()
            .filter(|artifact| artifact.role == "protocol")
            .collect::<Vec<_>>();
        if protocols.len() != 1 || protocols[0].sha256 != self.evaluator.protocol_sha256 {
            return Err(RecordedFailureCapsuleError::Invalid(
                "protocol artifact does not match evaluator protocol digest",
            ));
        }
        Ok(())
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == SHA256_HEX_LENGTH
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn capsule() -> RecordedFailureCapsule {
        RecordedFailureCapsule::new(
            RecordedFailureMetadata {
                id: "current-bias".into(),
                contract: "recorded-pmdc-v1".into(),
                message: "current bias exceeded its bound".into(),
                record_index: 39,
                source_time_ticks: 391_337,
                observation_sha256: A.into(),
            },
            RecordedRunMetadata {
                id: "trial-10".into(),
                source_id: "published-pmdc".into(),
                record_count: 2_009,
                evaluated_record_count: 40,
                command_count: 2_009,
            },
            RecordedClockMetadata {
                domain: "device_monotonic".into(),
                unit: RecordedTimeUnit::Microseconds,
                timestamp_channel: "time".into(),
                sampling: RecordedClockSampling::Nonuniform {
                    minimum_delta_ticks: 9_977,
                    maximum_delta_ticks: 10_041,
                },
            },
            BuildMetadata::new("0.2.0", "commit", "release", "target", "rustc", C),
            RecordedEvaluatorMetadata {
                name: "pmdc-final".into(),
                version: "1".into(),
                protocol_sha256: B.into(),
                random_seed: None,
            },
            vec![
                ArtifactRef::new("evaluation", "json", 1, "evidence/failure.json", A).unwrap(),
                ArtifactRef::new("protocol", "json", 1, "protocol/final.json", B).unwrap(),
                ArtifactRef::new("recorded_input", "jsonl", 1, "recorded/trial-10.jsonl", C)
                    .unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn nonuniform_recorded_capsule_has_no_simulation_clock_fields() {
        let capsule = capsule();
        capsule.validate().unwrap();
        let text = serde_json::to_string(&capsule).unwrap();
        assert!(!text.contains("fixed_delta"));
        assert!(!text.contains("sim_time"));
        assert!(text.contains("source_time_ticks"));
        assert!(text.contains("nonuniform"));
    }

    #[test]
    fn invalid_clock_counts_roles_and_future_fields_fail_closed() {
        let mut changed = capsule();
        changed.clock.sampling = RecordedClockSampling::Nonuniform {
            minimum_delta_ticks: 10,
            maximum_delta_ticks: 10,
        };
        assert!(changed.validate().is_err());

        let mut changed = capsule();
        changed.failure.record_index = changed.run.evaluated_record_count;
        assert!(changed.validate().is_err());

        let mut changed = capsule();
        changed.evaluator.protocol_sha256 = A.into();
        assert!(changed.validate().is_err());

        let mut changed = capsule();
        changed
            .artifacts
            .retain(|artifact| artifact.role != "protocol");
        assert!(changed.validate().is_err());

        let mut value = serde_json::to_value(capsule()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("sim_time_ticks".into(), serde_json::json!(1));
        assert!(serde_json::from_value::<RecordedFailureCapsule>(value).is_err());
    }

    #[test]
    fn legacy_failure_capsule_schema_version_is_unchanged() {
        assert_eq!(crate::FAILURE_CAPSULE_SCHEMA_VERSION, 1);
        let mut changed = capsule();
        changed.schema_version += 1;
        assert!(matches!(
            changed.validate(),
            Err(RecordedFailureCapsuleError::UnsupportedSchemaVersion { .. })
        ));
    }
}
