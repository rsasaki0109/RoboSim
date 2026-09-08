//! Bounded metadata for execution failures with potentially unavailable state.
//! Validation checks declarations only: it neither reads referenced files nor
//! executes replay, and a serialized replay claim is not trusted verification.

use crate::{ArtifactRef, FailureCapsuleError};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Maximum encoded metadata size; referenced artifacts have independent limits.
pub const MAX_EXECUTION_CAPSULE_BYTES: usize = 1024 * 1024;

/// Whether post-attempt state evidence could actually be captured.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum PostAttemptEvidence {
    /// Reference to captured evidence, whose completeness is defined by its schema.
    Captured {
        /// Exact-byte reference with role `post_attempt`.
        artifact: ArtifactRef,
    },
    /// No physical state hash is implied, especially not a zero sentinel.
    Unavailable {
        /// Explicit reason state evidence was not captured.
        reason: String,
    },
}

/// Producer-declared replay outcome, never automatically trusted by decoding.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum ExecutionReplayClaim {
    /// No replay execution has been attempted.
    NotAttempted,
    /// A replay ran but did not establish the original failure.
    NotReproduced {
        /// Report with role `replay_report`, including mismatch/limitation details.
        report: ArtifactRef,
    },
    /// A producer claims reproduction; consumers must verify the referenced report
    /// and execute the applicable replay before accepting this as proof.
    Reproduced {
        /// Report with role `replay_report`, specifying exactly which fields matched.
        report: ArtifactRef,
    },
}

/// Common envelope for execution-failure artifacts, distinct from legacy v1.
/// Contract evidence must bind TaskSpec, backend and build; attempt evidence must
/// distinguish requested operations, completed work, lane clocks and learner updates.
/// Their domain-specific schemas are validated by adapters, not guessed here.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecutionFailureCapsule {
    /// Must be `rne_execution_failure_capsule`.
    pub kind: String,
    /// Must be 1; this does not change the legacy failure-capsule version.
    pub schema_version: u32,
    /// Stable domain-specific failure code, not a fabricated physical timestamp.
    pub failure_code: String,
    /// Diagnostic text, bounded independently of referenced payloads.
    pub message: String,
    /// Frozen TaskSpec/backend/build contract, role `execution_contract`.
    pub contract: ArtifactRef,
    /// Last valid history/checkpoint before the attempted operation, role `prior_history`.
    pub prior_history: ArtifactRef,
    /// Actual requested operation and observed partial results, role `execution_attempt`.
    pub attempt: ArtifactRef,
    /// Explicit capture or absence of post-attempt state evidence.
    pub post_attempt: PostAttemptEvidence,
    /// An untrusted producer claim, not the return value of a replay verifier.
    pub replay: ExecutionReplayClaim,
}

/// Structural or bounded-decoding errors, without filesystem side effects.
#[derive(Debug, thiserror::Error)]
pub enum ExecutionCapsuleError {
    /// Metadata exceeds the input byte limit.
    #[error("execution capsule exceeds byte limit")]
    TooLarge,
    /// Invalid JSON or unknown fields.
    #[error("invalid execution capsule JSON: {0}")]
    Json(#[from] serde_json::Error),
    /// Invalid canonical artifact declaration.
    #[error("invalid execution artifact: {0}")]
    Artifact(#[from] FailureCapsuleError),
    /// Invalid kind, version, field bound, role or duplicate path.
    #[error("invalid execution capsule: {0}")]
    Invalid(&'static str),
}

impl ExecutionFailureCapsule {
    /// Checks bounded declarations only; no file reads or replay-status upgrade.
    pub fn validate(&self) -> Result<(), ExecutionCapsuleError> {
        use ExecutionCapsuleError::Invalid;
        if self.kind != "rne_execution_failure_capsule" || self.schema_version != 1 {
            return Err(Invalid("kind/version"));
        }
        for value in [&self.failure_code, &self.message] {
            if value.trim().is_empty() || value.len() > 4096 {
                return Err(Invalid("failure text"));
            }
        }
        let mut refs = vec![
            (&self.contract, "execution_contract"),
            (&self.prior_history, "prior_history"),
            (&self.attempt, "execution_attempt"),
        ];
        match &self.post_attempt {
            PostAttemptEvidence::Captured { artifact } => refs.push((artifact, "post_attempt")),
            PostAttemptEvidence::Unavailable { reason } => {
                if reason.trim().is_empty() || reason.len() > 4096 {
                    return Err(Invalid("unavailable reason"));
                }
            }
        }
        match &self.replay {
            ExecutionReplayClaim::NotAttempted => {}
            ExecutionReplayClaim::NotReproduced { report }
            | ExecutionReplayClaim::Reproduced { report } => refs.push((report, "replay_report")),
        }
        let mut paths = BTreeSet::new();
        for (artifact, role) in refs {
            artifact.validate()?;
            if artifact.role != role
                || artifact.path.len() > 4096
                || artifact.kind.len() > 256
                || !paths.insert(&artifact.path)
            {
                return Err(Invalid("artifact role/bound/duplicate path"));
            }
        }
        Ok(())
    }

    /// Decodes a bounded metadata slice; a replay claim remains merely a claim.
    pub fn decode(bytes: &[u8]) -> Result<Self, ExecutionCapsuleError> {
        if bytes.len() > MAX_EXECUTION_CAPSULE_BYTES {
            return Err(ExecutionCapsuleError::TooLarge);
        }
        let capsule: Self = serde_json::from_slice(bytes)?;
        capsule.validate()?;
        Ok(capsule)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> ExecutionFailureCapsule {
        let reference = |role: &str| {
            ArtifactRef::new(
                role,
                "test_evidence",
                1,
                format!("{role}.json"),
                "a".repeat(64),
            )
            .unwrap()
        };
        ExecutionFailureCapsule {
            kind: "rne_execution_failure_capsule".into(),
            schema_version: 1,
            failure_code: "backend_error".into(),
            message: "state unavailable after backend error".into(),
            contract: reference("execution_contract"),
            prior_history: reference("prior_history"),
            attempt: reference("execution_attempt"),
            post_attempt: PostAttemptEvidence::Unavailable {
                reason: "backend advanced before ECS synchronization".into(),
            },
            replay: ExecutionReplayClaim::NotAttempted,
        }
    }
    #[test]
    fn unavailable_state_roundtrips_without_inventing_hash_or_replay() {
        let capsule = fixture();
        assert_eq!(
            ExecutionFailureCapsule::decode(&serde_json::to_vec(&capsule).unwrap()).unwrap(),
            capsule
        );
    }
    #[test]
    fn malformed_roles_absence_and_oversized_input_are_rejected() {
        let mut capsule = fixture();
        capsule.attempt.role = "prior_history".into();
        assert!(capsule.validate().is_err());
        let mut capsule = fixture();
        capsule.post_attempt = PostAttemptEvidence::Unavailable {
            reason: String::new(),
        };
        assert!(capsule.validate().is_err());
        assert!(matches!(
            ExecutionFailureCapsule::decode(&vec![b' '; MAX_EXECUTION_CAPSULE_BYTES + 1]),
            Err(ExecutionCapsuleError::TooLarge)
        ));
    }
}
