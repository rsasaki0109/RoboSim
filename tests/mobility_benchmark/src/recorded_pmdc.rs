//! Frozen pre-final protocol for PMDC geared-motor effective identification.

use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub mod data;
pub mod evaluation;
pub mod final_evaluation;
pub mod final_protocol;
pub mod identification;

/// Stable artifact kind for the predeclared PMDC identification protocol.
pub const PMDC_PROTOCOL_KIND: &str = "rne_pmdc_identification_protocol";
/// Frozen deterministic JSON digest of protocol v1.
pub const PMDC_PROTOCOL_SHA256: &str =
    "36f24bd335546e1d72d4008258095cb7d0bd6fb9ddac5d86d1dd62520dafb874";
/// Exact source workbook digest admitted by protocol v1.
pub const PMDC_SOURCE_SHA256: &str =
    "85203c4b3ad6fbdd05221e1be7fd41ce733376c0f316d7fd5542b604a6854605";
/// Exact training-record stream admitted by protocol v1.
pub const PMDC_TRAINING_RECORDS_SHA256: &str =
    "8de4971cbfc8bfd26e3244ead3357ad6950f56a8d57759a7ceab44f0feb12e5b";
/// Exact development-record stream admitted by protocol v1.
pub const PMDC_DEVELOPMENT_RECORDS_SHA256: &str =
    "a9e559ee901221a3663274c4f20939b202491426145580d8e1bd278aa0d04dec";

/// Encoder-to-speed reconstruction fixed before development evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PmdcVelocityReconstruction {
    /// Backward encoder-count difference divided by the actual adjacent source timestamps.
    ActualTimestampBackwardDifference,
}

/// Nested electrical candidates compared using training runs only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PmdcElectricalCandidate {
    /// `I = q_v V + q_w omega + q_0`; no inductance claim.
    QuasiStaticEffective,
    /// `delta I = dt(a_v V + a_i I + a_w omega + a_0)`.
    DynamicEulerEffective,
}

/// One normalized development metric and its frozen upper bound.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcMetricSpec {
    /// Stable metric identifier.
    pub id: String,
    /// Unit after normalization.
    pub unit: String,
    /// Inclusive maximum accepted value.
    pub maximum: f64,
}

/// Training-only model-selection procedure.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcSelectionSpec {
    /// Use each complete training run once as the held-out CV run.
    pub leave_one_run_out: bool,
    /// Exact training-only cross-validation score.
    pub cross_validation_metric: String,
    /// Scale each held-out RMSE by its P95 minus P5 using type 7 quantiles.
    pub normalization: String,
    /// Deterministic least-squares solver and pivot tie rule.
    pub solver: String,
    /// Relative diagonal threshold below which a design is rejected as rank deficient.
    pub rank_relative_diagonal_min: f64,
    /// Absolute current-NRMSE improvement required to select the dynamic candidate.
    pub dynamic_min_current_nrmse_improvement: f64,
    /// Reject coefficients inconsistent with passive PMDC signs.
    pub require_physical_signs: bool,
    /// No data-dependent sample deletion is allowed.
    pub outlier_removal: bool,
    /// No timestamp resampling is allowed.
    pub resampling: bool,
    /// No implicit signal filtering is allowed.
    pub filtering: bool,
}

/// Complete protocol frozen before inspecting development responses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcIdentificationProtocol {
    /// Artifact discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Immutable workbook identity.
    pub source_sha256: String,
    /// Immutable converted training-record identity.
    pub training_records_sha256: String,
    /// Immutable converted development-record identity.
    pub development_records_sha256: String,
    /// Whole source trials used for training and cross-validation.
    pub training_trials: Vec<u8>,
    /// Whole source trial used exactly once for development evaluation.
    pub development_trials: Vec<u8>,
    /// Header cells that remain sealed for final evaluation.
    pub sealed_final_header_cells: Vec<String>,
    /// Encoder resolution declared by the source authors.
    pub encoder_pulses_per_revolution: u32,
    /// Gear reduction numerator declared by the source authors.
    pub gear_reduction_numerator: u32,
    /// Gear reduction denominator declared by the source authors.
    pub gear_reduction_denominator: u32,
    /// Explicit speed reconstruction.
    pub velocity_reconstruction: PmdcVelocityReconstruction,
    /// Electrical candidates, ordered from simpler to more complex.
    pub electrical_candidates: Vec<PmdcElectricalCandidate>,
    /// Fixed mechanical effective model equation.
    pub mechanical_model: String,
    /// Training-only selection rules.
    pub selection: PmdcSelectionSpec,
    /// One-shot development acceptance metrics.
    pub development_metrics: Vec<PmdcMetricSpec>,
    /// Maximum absolute allowed coefficient refit drift across repeated execution.
    pub deterministic_coefficient_tolerance: f64,
    /// Honest scope: only coefficient ratios convolved with the acquisition path.
    pub qualification_scope: String,
}

impl PmdcIdentificationProtocol {
    /// Require byte-for-byte semantic equality with the frozen protocol constructor.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self == &pmdc_identification_protocol(),
            "PMDC identification protocol drift"
        );
        ensure!(
            self.development_metrics
                .iter()
                .all(|metric| metric.maximum.is_finite() && metric.maximum > 0.0),
            "invalid PMDC development metric"
        );
        Ok(())
    }

    /// SHA-256 of deterministic serde JSON for provenance binding.
    pub fn sha256(&self) -> Result<String> {
        self.validate()?;
        Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(self)?)))
    }
}

/// Construct protocol v1 without reading training, development, or final response values.
pub fn pmdc_identification_protocol() -> PmdcIdentificationProtocol {
    PmdcIdentificationProtocol {
        kind: PMDC_PROTOCOL_KIND.into(),
        schema_version: 1,
        source_sha256: PMDC_SOURCE_SHA256.into(),
        training_records_sha256: PMDC_TRAINING_RECORDS_SHA256.into(),
        development_records_sha256: PMDC_DEVELOPMENT_RECORDS_SHA256.into(),
        training_trials: (1..=8).collect(),
        development_trials: vec![9],
        sealed_final_header_cells: vec!["A18101".into(), "A20112".into()],
        encoder_pulses_per_revolution: 1800,
        gear_reduction_numerator: 1,
        gear_reduction_denominator: 17,
        velocity_reconstruction: PmdcVelocityReconstruction::ActualTimestampBackwardDifference,
        electrical_candidates: vec![
            PmdcElectricalCandidate::QuasiStaticEffective,
            PmdcElectricalCandidate::DynamicEulerEffective,
        ],
        mechanical_model: "delta_omega=dt*(b_i*current+b_w*omega+b_s*sign(omega)+b_0)".into(),
        selection: PmdcSelectionSpec {
            leave_one_run_out: true,
            cross_validation_metric: "mean_heldout_run_current_one_step_nrmse".into(),
            normalization: "heldout_run_hyndman_fan_type7_p95_minus_p5".into(),
            solver: "f64_householder_qr_column_pivoting_ties_by_original_index".into(),
            rank_relative_diagonal_min: 1e-10,
            dynamic_min_current_nrmse_improvement: 0.02,
            require_physical_signs: true,
            outlier_removal: false,
            resampling: false,
            filtering: false,
        },
        development_metrics: vec![
            PmdcMetricSpec {
                id: "current_rollout_nrmse".into(),
                unit: "1".into(),
                maximum: 0.20,
            },
            PmdcMetricSpec {
                id: "output_speed_rollout_nrmse".into(),
                unit: "1".into(),
                maximum: 0.15,
            },
            PmdcMetricSpec {
                id: "current_signed_bias_fraction".into(),
                unit: "1".into(),
                maximum: 0.05,
            },
            PmdcMetricSpec {
                id: "output_speed_signed_bias_fraction".into(),
                unit: "1".into(),
                maximum: 0.05,
            },
        ],
        deterministic_coefficient_tolerance: 1e-12,
        qualification_scope:
            "effective_coefficient_ratios_with_unqualified_sensor_and_timing_dynamics".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_is_deterministic_and_keeps_final_sealed() {
        let first = pmdc_identification_protocol();
        let second = pmdc_identification_protocol();
        first.validate().unwrap();
        assert_eq!(first, second);
        assert_eq!(first.sha256().unwrap(), second.sha256().unwrap());
        assert_eq!(first.sha256().unwrap(), PMDC_PROTOCOL_SHA256);
        assert_eq!(first.training_trials, (1..=8).collect::<Vec<_>>());
        assert_eq!(first.development_trials, [9]);
        assert_eq!(first.sealed_final_header_cells, ["A18101", "A20112"]);
        assert!(!first.selection.outlier_removal);
        assert!(!first.selection.resampling);
        assert!(!first.selection.filtering);
    }

    #[test]
    fn any_protocol_or_threshold_drift_is_rejected() {
        let mut changed = pmdc_identification_protocol();
        changed.training_trials.push(10);
        assert!(changed.validate().is_err());
        let mut changed = pmdc_identification_protocol();
        changed.development_metrics[0].maximum = 1.0;
        assert!(changed.validate().is_err());
        let mut changed = pmdc_identification_protocol();
        changed.qualification_scope = "physical_motor_constants".into();
        assert!(changed.validate().is_err());
    }
}
