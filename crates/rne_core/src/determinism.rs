//! Backend-neutral determinism contracts for simulation evidence.
//!
//! A contract describes what a caller promises to compare, while the replay,
//! physics, sensor, and runner crates provide the evidence used to verify it.
//! Keeping this declaration in `rne_core` prevents any particular backend,
//! renderer, or transport from becoming part of the public contract.

use serde::{Deserialize, Deserializer, Serialize};
use thiserror::Error;

/// Current schema version for [`DeterminismContract`].
pub const DETERMINISM_CONTRACT_SCHEMA_VERSION: u16 = 1;

/// Strength of a determinism guarantee.
///
/// The tiers describe the comparison policy, not the source of the evidence:
/// an exact contract may be backed by a stable digest, while a tolerance or
/// outcome contract may be backed by typed observations or task reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeterminismTier {
    /// Every declared observable must compare exactly.
    Exact,
    /// Numeric observables may differ within declared absolute and relative bounds.
    Tolerance,
    /// The declared semantic outcome must be the same, without requiring the trajectory to match.
    Outcome,
}

impl DeterminismTier {
    /// Returns the stable identifier used by reports and serialized manifests.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Exact => "exact",
            Self::Tolerance => "tolerance",
            Self::Outcome => "outcome",
        }
    }
}

/// A logical simulation region and its declared observable streams.
///
/// `first_step` and `step_count` identify a finite inclusive step window. The
/// names in `observables` are backend-neutral labels such as `world.state`,
/// `sensor.lidar`, or `episode.outcome`; their interpretation belongs to the
/// evidence producer. Declaration order is preserved so callers can use the
/// same order in reports and replay manifests.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeterminismScope {
    /// Stable logical name of the simulation region under comparison.
    pub subject: String,
    /// Stable backend-neutral observable or stream names covered by the contract.
    pub observables: Vec<String>,
    /// First simulation step covered by the contract.
    pub first_step: u64,
    /// Number of simulation steps covered by the contract.
    pub step_count: u64,
}

impl DeterminismScope {
    /// Creates and validates a finite determinism scope.
    pub fn new(
        subject: impl Into<String>,
        observables: impl IntoIterator<Item = impl Into<String>>,
        first_step: u64,
        step_count: u64,
    ) -> Result<Self, DeterminismContractError> {
        let scope = Self {
            subject: subject.into(),
            observables: observables.into_iter().map(Into::into).collect(),
            first_step,
            step_count,
        };
        scope.validate()?;
        Ok(scope)
    }

    /// Validates naming, uniqueness, and step-range invariants.
    pub fn validate(&self) -> Result<(), DeterminismContractError> {
        if self.subject.trim().is_empty() {
            return Err(DeterminismContractError::EmptyScopeSubject);
        }
        if self.observables.is_empty() {
            return Err(DeterminismContractError::EmptyObservableSet);
        }
        if self
            .observables
            .iter()
            .any(|observable| observable.trim().is_empty())
        {
            return Err(DeterminismContractError::EmptyObservable);
        }
        for (index, observable) in self.observables.iter().enumerate() {
            if self.observables[..index].contains(observable) {
                return Err(DeterminismContractError::DuplicateObservable);
            }
        }
        if self.step_count == 0 {
            return Err(DeterminismContractError::ZeroStepCount);
        }
        self.first_step
            .checked_add(self.step_count - 1)
            .ok_or(DeterminismContractError::StepRangeOverflow)?;
        Ok(())
    }

    /// Returns the final inclusive step in this scope.
    ///
    /// A scope constructed through [`Self::new`] always has a representable
    /// final step. For a manually assembled invalid value, this returns `None`.
    pub fn last_step(&self) -> Option<u64> {
        self.step_count
            .checked_sub(1)
            .and_then(|offset| self.first_step.checked_add(offset))
    }
}

/// Comparison policy and its tier-specific guarantee data.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "tier", rename_all = "snake_case")]
pub enum DeterminismGuarantee {
    /// Requires exact equality for every observable in the scope.
    Exact,
    /// Allows numeric differences bounded by `max(absolute, relative * |expected|)`.
    Tolerance {
        /// Non-negative absolute error bound in the observable's declared units.
        absolute: f64,
        /// Non-negative relative error bound applied to the expected value.
        relative: f64,
    },
    /// Requires a stable semantic outcome identifier to match.
    Outcome {
        /// Stable task or evaluation criterion, not an executable expression.
        criterion: String,
    },
}

impl<'de> Deserialize<'de> for DeterminismGuarantee {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(tag = "tier", rename_all = "snake_case", deny_unknown_fields)]
        enum WireGuarantee {
            Exact {},
            Tolerance { absolute: f64, relative: f64 },
            Outcome { criterion: String },
        }

        Ok(match WireGuarantee::deserialize(deserializer)? {
            WireGuarantee::Exact {} => Self::Exact,
            WireGuarantee::Tolerance { absolute, relative } => {
                Self::Tolerance { absolute, relative }
            }
            WireGuarantee::Outcome { criterion } => Self::Outcome { criterion },
        })
    }
}

impl DeterminismGuarantee {
    /// Creates an exact guarantee.
    pub const fn exact() -> Self {
        Self::Exact
    }

    /// Creates and validates a numeric tolerance guarantee.
    pub fn tolerance(absolute: f64, relative: f64) -> Result<Self, DeterminismContractError> {
        let guarantee = Self::Tolerance { absolute, relative };
        guarantee.validate()?;
        Ok(guarantee)
    }

    /// Creates and validates an outcome guarantee.
    pub fn outcome(criterion: impl Into<String>) -> Result<Self, DeterminismContractError> {
        let guarantee = Self::Outcome {
            criterion: criterion.into(),
        };
        guarantee.validate()?;
        Ok(guarantee)
    }

    /// Returns the tier represented by this guarantee.
    pub const fn tier(&self) -> DeterminismTier {
        match self {
            Self::Exact => DeterminismTier::Exact,
            Self::Tolerance { .. } => DeterminismTier::Tolerance,
            Self::Outcome { .. } => DeterminismTier::Outcome,
        }
    }

    /// Returns the absolute and relative bounds for a tolerance guarantee.
    pub const fn tolerance_bounds(&self) -> Option<(f64, f64)> {
        match self {
            Self::Tolerance { absolute, relative } => Some((*absolute, *relative)),
            Self::Exact | Self::Outcome { .. } => None,
        }
    }

    /// Returns the semantic criterion for an outcome guarantee.
    pub fn outcome_criterion(&self) -> Option<&str> {
        match self {
            Self::Outcome { criterion } => Some(criterion),
            Self::Exact | Self::Tolerance { .. } => None,
        }
    }

    /// Validates tier-specific values and invariants.
    pub fn validate(&self) -> Result<(), DeterminismContractError> {
        match self {
            Self::Exact => Ok(()),
            Self::Tolerance { absolute, relative } => {
                if !absolute.is_finite()
                    || !relative.is_finite()
                    || *absolute < 0.0
                    || *relative < 0.0
                    || (*absolute == 0.0 && *relative == 0.0)
                {
                    return Err(DeterminismContractError::InvalidTolerance);
                }
                Ok(())
            }
            Self::Outcome { criterion } if criterion.trim().is_empty() => {
                Err(DeterminismContractError::EmptyOutcomeCriterion)
            }
            Self::Outcome { .. } => Ok(()),
        }
    }
}

/// Declarative, backend-neutral promise used to classify replay evidence.
///
/// A contract does not execute a predicate or own a backend handle. It only
/// declares the comparison scope and guarantee so later physics, sensor,
/// runner, and evaluation code can consume the same metadata.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeterminismContract {
    /// Schema version for this contract declaration.
    pub schema_version: u16,
    /// Stable name used in reports and replay manifests.
    pub name: String,
    /// Observable streams and finite step window covered by the contract.
    pub scope: DeterminismScope,
    /// Comparison policy promised for the scope.
    pub guarantee: DeterminismGuarantee,
}

impl DeterminismContract {
    /// Creates and validates a contract using the current schema version.
    pub fn new(
        name: impl Into<String>,
        scope: DeterminismScope,
        guarantee: DeterminismGuarantee,
    ) -> Result<Self, DeterminismContractError> {
        let contract = Self {
            schema_version: DETERMINISM_CONTRACT_SCHEMA_VERSION,
            name: name.into(),
            scope,
            guarantee,
        };
        contract.validate()?;
        Ok(contract)
    }

    /// Creates an exact contract.
    pub fn exact(
        name: impl Into<String>,
        scope: DeterminismScope,
    ) -> Result<Self, DeterminismContractError> {
        Self::new(name, scope, DeterminismGuarantee::exact())
    }

    /// Creates a tolerance contract with absolute and relative numeric bounds.
    pub fn tolerance(
        name: impl Into<String>,
        scope: DeterminismScope,
        absolute: f64,
        relative: f64,
    ) -> Result<Self, DeterminismContractError> {
        Self::new(
            name,
            scope,
            DeterminismGuarantee::tolerance(absolute, relative)?,
        )
    }

    /// Creates an outcome contract for a stable semantic criterion.
    pub fn outcome(
        name: impl Into<String>,
        scope: DeterminismScope,
        criterion: impl Into<String>,
    ) -> Result<Self, DeterminismContractError> {
        Self::new(name, scope, DeterminismGuarantee::outcome(criterion)?)
    }

    /// Validates the schema, name, scope, and guarantee.
    pub fn validate(&self) -> Result<(), DeterminismContractError> {
        if self.schema_version != DETERMINISM_CONTRACT_SCHEMA_VERSION {
            return Err(DeterminismContractError::UnsupportedSchemaVersion {
                expected: DETERMINISM_CONTRACT_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.name.trim().is_empty() {
            return Err(DeterminismContractError::EmptyContractName);
        }
        self.scope.validate()?;
        self.guarantee.validate()
    }

    /// Returns the comparison tier of this contract.
    pub const fn tier(&self) -> DeterminismTier {
        self.guarantee.tier()
    }
}

/// Invalid determinism-contract declaration.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum DeterminismContractError {
    /// The serialized contract uses a schema version this crate does not understand.
    #[error("unsupported determinism contract schema: expected {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Schema version supported by this crate.
        expected: u16,
        /// Schema version found in the declaration.
        actual: u16,
    },
    /// The contract name is empty or only whitespace.
    #[error("determinism contract name must not be empty")]
    EmptyContractName,
    /// The scope subject is empty or only whitespace.
    #[error("determinism scope subject must not be empty")]
    EmptyScopeSubject,
    /// A scope must declare at least one observable.
    #[error("determinism scope must declare at least one observable")]
    EmptyObservableSet,
    /// An observable name is empty or only whitespace.
    #[error("determinism observable name must not be empty")]
    EmptyObservable,
    /// An observable occurs more than once in a scope.
    #[error("determinism scope observables must be unique")]
    DuplicateObservable,
    /// A scope must cover at least one step.
    #[error("determinism scope step_count must be greater than zero")]
    ZeroStepCount,
    /// The inclusive final step cannot be represented by `u64`.
    #[error("determinism scope step range overflows u64")]
    StepRangeOverflow,
    /// Tolerance bounds are not finite, are negative, or are both zero.
    #[error("determinism tolerance bounds must be finite, non-negative, and not both zero")]
    InvalidTolerance,
    /// An outcome guarantee needs a stable non-empty criterion identifier.
    #[error("determinism outcome criterion must not be empty")]
    EmptyOutcomeCriterion,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scope() -> DeterminismScope {
        DeterminismScope::new("episode", ["world.state", "sensor.lidar"], 4, 3)
            .expect("valid scope")
    }

    #[test]
    fn exact_contract_declares_scope_and_tier() {
        let contract = DeterminismContract::exact("replay", scope()).expect("valid contract");

        assert_eq!(contract.schema_version, DETERMINISM_CONTRACT_SCHEMA_VERSION);
        assert_eq!(contract.scope.last_step(), Some(6));
        assert_eq!(contract.tier(), DeterminismTier::Exact);
        assert_eq!(contract.guarantee.tolerance_bounds(), None);
    }

    #[test]
    fn tolerance_contract_exposes_both_bounds() {
        let contract = DeterminismContract::tolerance("physics", scope(), 1.0e-6, 1.0e-4)
            .expect("valid tolerance");

        assert_eq!(contract.tier(), DeterminismTier::Tolerance);
        assert_eq!(
            contract.guarantee.tolerance_bounds(),
            Some((1.0e-6, 1.0e-4))
        );
    }

    #[test]
    fn outcome_contract_keeps_semantic_criterion() {
        let contract =
            DeterminismContract::outcome("task", scope(), "placed_in_zone").expect("valid outcome");

        assert_eq!(contract.tier(), DeterminismTier::Outcome);
        assert_eq!(
            contract.guarantee.outcome_criterion(),
            Some("placed_in_zone")
        );
    }

    #[test]
    fn invalid_values_are_rejected() {
        assert_eq!(
            DeterminismScope::new("episode", Vec::<String>::new(), 0, 1).unwrap_err(),
            DeterminismContractError::EmptyObservableSet
        );
        assert_eq!(
            DeterminismScope::new("episode", ["world.state", "world.state"], 0, 1).unwrap_err(),
            DeterminismContractError::DuplicateObservable
        );
        assert_eq!(
            DeterminismGuarantee::tolerance(0.0, 0.0).unwrap_err(),
            DeterminismContractError::InvalidTolerance
        );
        assert_eq!(
            DeterminismGuarantee::tolerance(f64::NAN, 0.1).unwrap_err(),
            DeterminismContractError::InvalidTolerance
        );
        assert_eq!(
            DeterminismGuarantee::outcome(" ").unwrap_err(),
            DeterminismContractError::EmptyOutcomeCriterion
        );
    }

    #[test]
    fn scope_overflow_is_rejected() {
        assert_eq!(
            DeterminismScope::new("episode", ["world.state"], u64::MAX, 2).unwrap_err(),
            DeterminismContractError::StepRangeOverflow
        );
    }

    #[test]
    fn manually_changed_schema_is_rejected() {
        let mut contract = DeterminismContract::exact("replay", scope()).expect("valid contract");
        contract.schema_version += 1;

        assert_eq!(
            contract.validate().unwrap_err(),
            DeterminismContractError::UnsupportedSchemaVersion {
                expected: DETERMINISM_CONTRACT_SCHEMA_VERSION,
                actual: DETERMINISM_CONTRACT_SCHEMA_VERSION + 1,
            }
        );
    }

    #[test]
    fn json_round_trip_preserves_stable_tolerance_shape() {
        let contract = DeterminismContract::tolerance("physics", scope(), 1.0e-6, 1.0e-4)
            .expect("valid tolerance");
        let json = serde_json::to_string_pretty(&contract).expect("serialize contract");
        let expected = concat!(
            "{\n",
            "  \"schema_version\": 1,\n",
            "  \"name\": \"physics\",\n",
            "  \"scope\": {\n",
            "    \"subject\": \"episode\",\n",
            "    \"observables\": [\n",
            "      \"world.state\",\n",
            "      \"sensor.lidar\"\n",
            "    ],\n",
            "    \"first_step\": 4,\n",
            "    \"step_count\": 3\n",
            "  },\n",
            "  \"guarantee\": {\n",
            "    \"tier\": \"tolerance\",\n",
            "    \"absolute\": 1e-6,\n",
            "    \"relative\": 0.0001\n",
            "  }\n",
            "}"
        );

        assert_eq!(json, expected);
        let decoded: DeterminismContract = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(decoded, contract);
        decoded
            .validate()
            .expect("round-tripped contract validates");
    }

    #[test]
    fn json_rejects_unknown_fields() {
        let json = concat!(
            "{",
            "\"schema_version\":1,",
            "\"name\":\"replay\",",
            "\"scope\":{",
            "\"subject\":\"episode\",",
            "\"observables\":[\"world.state\"],",
            "\"first_step\":0,",
            "\"step_count\":1,",
            "\"unexpected\":true",
            "},",
            "\"guarantee\":{\"tier\":\"exact\"}",
            "}"
        );

        let error = serde_json::from_str::<DeterminismContract>(json)
            .expect_err("unknown scope fields must be rejected");
        assert!(error.to_string().contains("unknown field"));

        let guarantee_json = concat!(
            "{",
            "\"schema_version\":1,",
            "\"name\":\"replay\",",
            "\"scope\":{",
            "\"subject\":\"episode\",",
            "\"observables\":[\"world.state\"],",
            "\"first_step\":0,",
            "\"step_count\":1",
            "},",
            "\"guarantee\":{\"tier\":\"exact\",\"unexpected\":true}",
            "}"
        );
        let error = serde_json::from_str::<DeterminismContract>(guarantee_json)
            .expect_err("unknown guarantee fields must be rejected");
        assert!(error.to_string().contains("unknown field"));
    }

    #[test]
    fn json_rejects_unsupported_schema_version() {
        let json = concat!(
            "{",
            "\"schema_version\":2,",
            "\"name\":\"replay\",",
            "\"scope\":{",
            "\"subject\":\"episode\",",
            "\"observables\":[\"world.state\"],",
            "\"first_step\":0,",
            "\"step_count\":1",
            "},",
            "\"guarantee\":{\"tier\":\"exact\"}",
            "}"
        );

        let decoded: DeterminismContract =
            serde_json::from_str(json).expect("schema is structurally valid JSON");
        assert_eq!(
            decoded.validate().unwrap_err(),
            DeterminismContractError::UnsupportedSchemaVersion {
                expected: DETERMINISM_CONTRACT_SCHEMA_VERSION,
                actual: 2,
            }
        );
    }
}
