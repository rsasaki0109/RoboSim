//! One-shot development rollout for the frozen PMDC effective model protocol.

use super::{
    data::{PmdcDevelopmentSet, PmdcObservation, PmdcTrainingSet},
    identification::{
        robust_range, verify_observations, verify_pmdc_training_identification,
        PmdcTrainingIdentification,
    },
    pmdc_identification_protocol, PmdcElectricalCandidate, PMDC_DEVELOPMENT_RECORDS_SHA256,
    PMDC_SOURCE_SHA256, PMDC_TRAINING_RECORDS_SHA256,
};
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable artifact kind for one-shot PMDC development evaluation.
pub const PMDC_DEVELOPMENT_EVALUATION_KIND: &str = "rne_pmdc_development_evaluation";

/// One predeclared normalized development gate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcDevelopmentMetric {
    /// Metric identifier from the frozen protocol.
    pub id: String,
    /// Dimensionless normalized value.
    pub value: f64,
    /// Inclusive predeclared maximum.
    pub maximum: f64,
    /// Whether the metric meets its maximum.
    pub passed: bool,
}

/// Content-bound result of the one permitted trial-9 evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcDevelopmentEvaluation {
    /// Artifact discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Frozen protocol identity.
    pub protocol_sha256: String,
    /// Exact selected training-identification identity.
    pub training_identification_sha256: String,
    /// Exact trial-9 record-stream identity.
    pub development_records_sha256: String,
    /// Number of free-run comparisons after the initial observed state.
    pub rollout_steps: usize,
    /// Four metrics in frozen protocol order.
    pub metrics: Vec<PmdcDevelopmentMetric>,
    /// Conjunction of all predeclared gates.
    pub passed: bool,
    /// Development data were evaluated once by this artifact.
    pub development_evaluated: bool,
    /// Final trial data were not read.
    pub final_partition_read: bool,
    /// Effective development performance does not qualify physical constants.
    pub physical_parameters_qualified: bool,
    /// SHA-256 over serde JSON with this field empty.
    pub content_sha256: String,
}

fn rmse(values: &[f64]) -> Result<f64> {
    ensure!(!values.is_empty(), "empty PMDC development residuals");
    let value =
        (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    ensure!(value.is_finite(), "nonfinite PMDC development RMSE");
    Ok(value)
}

fn mean(values: &[f64]) -> Result<f64> {
    ensure!(!values.is_empty(), "empty PMDC development residuals");
    let value = values.iter().sum::<f64>() / values.len() as f64;
    ensure!(value.is_finite(), "nonfinite PMDC development bias");
    Ok(value)
}

fn evaluation_digest(evidence: &PmdcDevelopmentEvaluation) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

fn evaluate_observations(
    training_identification: &PmdcTrainingIdentification,
    training_runs: &[Vec<PmdcObservation>],
    development_records_sha256: &str,
    development: &[PmdcObservation],
) -> Result<PmdcDevelopmentEvaluation> {
    ensure!(
        development.len() >= 3,
        "PMDC development run has too few observations"
    );
    verify_observations(
        training_identification,
        &training_identification.training_records_sha256,
        training_runs,
    )?;
    let current_scale = robust_range(
        &training_runs
            .iter()
            .flatten()
            .map(|sample| sample.current_a)
            .collect::<Vec<_>>(),
    )?;
    let speed_scale = robust_range(
        &training_runs
            .iter()
            .flatten()
            .map(|sample| sample.output_speed_rad_s)
            .collect::<Vec<_>>(),
    )?;
    let electrical = &training_identification.electrical_coefficients;
    let mechanical = &training_identification.mechanical_coefficients;
    let mut current = development[0].current_a;
    let mut speed = development[0].output_speed_rad_s;
    let mut current_residuals = Vec::with_capacity(development.len() - 1);
    let mut speed_residuals = Vec::with_capacity(development.len() - 1);
    for pair in development.windows(2) {
        let dt = pair[1].source_time_s - pair[0].source_time_s;
        ensure!(
            dt.is_finite() && dt > 0.0,
            "invalid PMDC development interval"
        );
        let next_speed = speed
            + dt * (mechanical[0] * current
                + mechanical[1] * speed
                + mechanical[2] * speed.signum()
                + mechanical[3]);
        let next_current = match training_identification.selected_electrical_candidate {
            PmdcElectricalCandidate::QuasiStaticEffective => {
                electrical[0] * pair[1].terminal_voltage_v
                    + electrical[1] * next_speed
                    + electrical[2]
            }
            PmdcElectricalCandidate::DynamicEulerEffective => {
                current
                    + dt * (electrical[0] * pair[0].terminal_voltage_v
                        + electrical[1] * current
                        + electrical[2] * speed
                        + electrical[3])
            }
        };
        ensure!(
            next_current.is_finite() && next_speed.is_finite(),
            "nonfinite PMDC development rollout"
        );
        current_residuals.push(next_current - pair[1].current_a);
        speed_residuals.push(next_speed - pair[1].output_speed_rad_s);
        current = next_current;
        speed = next_speed;
    }
    let values = [
        rmse(&current_residuals)? / current_scale,
        rmse(&speed_residuals)? / speed_scale,
        mean(&current_residuals)?.abs() / current_scale,
        mean(&speed_residuals)?.abs() / speed_scale,
    ];
    let protocol = pmdc_identification_protocol();
    let metrics = protocol
        .development_metrics
        .iter()
        .zip(values)
        .map(|(spec, value)| PmdcDevelopmentMetric {
            id: spec.id.clone(),
            value,
            maximum: spec.maximum,
            passed: value <= spec.maximum,
        })
        .collect::<Vec<_>>();
    let passed = metrics.iter().all(|metric| metric.passed);
    let mut evidence = PmdcDevelopmentEvaluation {
        kind: PMDC_DEVELOPMENT_EVALUATION_KIND.into(),
        schema_version: 1,
        protocol_sha256: protocol.sha256()?,
        training_identification_sha256: training_identification.content_sha256.clone(),
        development_records_sha256: development_records_sha256.into(),
        rollout_steps: development.len() - 1,
        metrics,
        passed,
        development_evaluated: true,
        final_partition_read: false,
        physical_parameters_qualified: false,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = evaluation_digest(&evidence)?;
    Ok(evidence)
}

impl PmdcDevelopmentEvaluation {
    /// Validate frozen metric order, verdict, scope and content digest.
    pub fn validate(&self) -> Result<()> {
        let protocol = pmdc_identification_protocol();
        ensure!(
            self.kind == PMDC_DEVELOPMENT_EVALUATION_KIND && self.schema_version == 1,
            "PMDC development evaluation kind/schema drift"
        );
        ensure!(
            self.protocol_sha256 == protocol.sha256()?
                && self.development_records_sha256 == PMDC_DEVELOPMENT_RECORDS_SHA256,
            "PMDC development evaluation provenance drift"
        );
        ensure!(
            self.rollout_steps == 2007 && self.training_identification_sha256.len() == 64,
            "PMDC development rollout provenance drift"
        );
        ensure!(
            self.metrics.len() == protocol.development_metrics.len(),
            "PMDC development metric-count drift"
        );
        for (metric, spec) in self.metrics.iter().zip(&protocol.development_metrics) {
            ensure!(
                metric.id == spec.id
                    && metric.maximum == spec.maximum
                    && metric.value.is_finite()
                    && metric.value >= 0.0
                    && metric.passed == (metric.value <= metric.maximum),
                "PMDC development metric drift"
            );
        }
        ensure!(
            self.passed == self.metrics.iter().all(|metric| metric.passed),
            "PMDC development verdict drift"
        );
        ensure!(
            self.development_evaluated
                && !self.final_partition_read
                && !self.physical_parameters_qualified,
            "PMDC development evidence overclaims its scope"
        );
        ensure!(
            self.content_sha256 == evaluation_digest(self)?,
            "PMDC development evidence digest drift"
        );
        Ok(())
    }
}

/// Execute the frozen one-shot development rollout against exact training evidence.
pub fn evaluate_pmdc_development(
    training: &PmdcTrainingSet,
    training_identification: &PmdcTrainingIdentification,
    development: &PmdcDevelopmentSet,
) -> Result<PmdcDevelopmentEvaluation> {
    ensure!(
        training.source_sha256 == PMDC_SOURCE_SHA256
            && training.records_sha256 == PMDC_TRAINING_RECORDS_SHA256
            && development.source_sha256 == PMDC_SOURCE_SHA256
            && development.records_sha256 == PMDC_DEVELOPMENT_RECORDS_SHA256,
        "PMDC development input identity drift"
    );
    verify_pmdc_training_identification(training_identification, training)?;
    let training_runs = training
        .runs
        .iter()
        .map(|run| run.observations())
        .collect::<Result<Vec<_>>>()?;
    let development_observations = development.run.observations()?;
    let evidence = evaluate_observations(
        training_identification,
        &training_runs,
        &development.records_sha256,
        &development_observations,
    )?;
    evidence.validate()?;
    Ok(evidence)
}

/// Rerun training identification and development rollout and require exact equality.
pub fn verify_pmdc_development_evaluation(
    evidence: &PmdcDevelopmentEvaluation,
    training: &PmdcTrainingSet,
    training_identification: &PmdcTrainingIdentification,
    development: &PmdcDevelopmentSet,
) -> Result<()> {
    evidence.validate()?;
    ensure!(
        evidence == &evaluate_pmdc_development(training, training_identification, development)?,
        "PMDC development evidence does not reproduce"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_pmdc::identification::identify_observations;

    fn synthetic_run(trial: i32) -> Vec<PmdcObservation> {
        let mut current = -0.3 + f64::from(trial) * 0.08;
        let mut speed = -2.0 + f64::from(trial) * 0.5;
        let mut run = Vec::new();
        for index in 0..800 {
            let time = f64::from(index) * 0.01;
            let voltage = if (index / 40 + trial) % 2 == 0 {
                12.0
            } else {
                -12.0
            };
            run.push(PmdcObservation {
                source_row: index as u32,
                source_time_s: time,
                encoder_interval_s: 0.01,
                current_a: current,
                terminal_voltage_v: voltage,
                output_speed_rad_s: speed,
            });
            let next_current =
                current + 0.01 * (8.0 * voltage - 12.0 * current - 50.0 * speed + 0.1);
            let next_speed =
                speed + 0.01 * (2.0 * current - 0.15 * speed - 0.03 * speed.signum() + 0.02);
            current = next_current;
            speed = next_speed;
        }
        run
    }

    #[test]
    fn exact_synthetic_development_rollout_passes_without_final_or_physical_claims() {
        let training_runs = (0..8).map(synthetic_run).collect::<Vec<_>>();
        let training = identify_observations("synthetic-training", &training_runs).unwrap();
        let development = synthetic_run(8);
        let evidence = evaluate_observations(
            &training,
            &training_runs,
            "synthetic-development",
            &development,
        )
        .unwrap();
        assert!(evidence.passed);
        assert!(evidence.metrics.iter().all(|metric| metric.value < 1e-10));
        assert!(evidence.development_evaluated);
        assert!(!evidence.final_partition_read);
        assert!(!evidence.physical_parameters_qualified);
        assert_eq!(
            evidence.content_sha256,
            evaluation_digest(&evidence).unwrap()
        );

        let mut bound = evidence;
        bound.protocol_sha256 = pmdc_identification_protocol().sha256().unwrap();
        bound.training_identification_sha256 = "0".repeat(64);
        bound.development_records_sha256 = PMDC_DEVELOPMENT_RECORDS_SHA256.into();
        bound.rollout_steps = 2007;
        bound.content_sha256 = evaluation_digest(&bound).unwrap();
        bound.validate().unwrap();

        let mut changed = bound;
        changed.metrics[0].value = 1.0;
        changed.content_sha256 = evaluation_digest(&changed).unwrap();
        assert!(changed.validate().is_err());
    }
}
