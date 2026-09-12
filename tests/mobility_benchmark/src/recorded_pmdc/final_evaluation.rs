//! One-shot evaluation of the two frozen PMDC final runs.

use super::{
    data::{PmdcFinalSet, PmdcObservation, PmdcTrainingSet},
    final_protocol::{
        pmdc_final_evaluation_protocol, PmdcFinalAggregation, PMDC_FINAL_ARTIFACT_SHA256,
        PMDC_FINAL_RECORDS_SHA256, PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256,
    },
    identification::{
        robust_range, verify_pmdc_training_identification, PmdcTrainingIdentification,
    },
    PmdcElectricalCandidate, PMDC_SOURCE_SHA256, PMDC_TRAINING_RECORDS_SHA256,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable artifact kind for the one permitted PMDC final evaluation.
pub const PMDC_FINAL_EVALUATION_KIND: &str = "rne_pmdc_final_evaluation";
/// SHA-256 of the one retained pretty-JSON final evaluation file.
pub const PMDC_FINAL_EVALUATION_ARTIFACT_SHA256: &str =
    "59f0f967d89ac8d76ff9f3af0821e05f293175b4d4596bc0a0e383c8524d2c32";
/// Exact byte length of the one retained final evaluation file.
pub const PMDC_FINAL_EVALUATION_ARTIFACT_BYTES: usize = 4_791;
/// Content digest emitted by the sole final evaluation.
pub const PMDC_FINAL_EVALUATION_CONTENT_SHA256: &str =
    "5f4e0b8abaf4a26ec60d901ebb8513253ff8a6a7ca7633175b5722425b9a520b";

/// One scalar contributing to a final gate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcFinalMetricValue {
    /// Run identity for a per-run value; absent for pooled and worst-run values.
    pub run_id: Option<String>,
    /// Dimensionless normalized metric value.
    pub value: f64,
    /// Whether this value meets the unchanged inclusive maximum.
    pub passed: bool,
}

/// One of the twelve predeclared final gates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcFinalMetric {
    /// Metric identifier from the frozen protocol.
    pub id: String,
    /// Frozen aggregation level.
    pub aggregation: PmdcFinalAggregation,
    /// Unchanged inclusive maximum.
    pub maximum: f64,
    /// Two values for `per_run`, otherwise one aggregate value.
    pub values: Vec<PmdcFinalMetricValue>,
    /// Conjunction of all values belonging to this gate.
    pub passed: bool,
}

/// Content-bound result of the sole two-run final evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcFinalEvaluation {
    /// Artifact discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Frozen final protocol identity.
    pub final_protocol_sha256: String,
    /// Exact selected training-identification identity.
    pub training_identification_sha256: String,
    /// Exact complete final artifact identity.
    pub final_artifact_sha256: String,
    /// Exact canonical final record-stream identity.
    pub final_records_sha256: String,
    /// Final run identifiers in frozen order.
    pub run_ids: Vec<String>,
    /// Free-rollout comparisons in each complete run.
    pub rollout_steps_per_run: Vec<usize>,
    /// Twelve gates in frozen metric/aggregation order.
    pub metrics: Vec<PmdcFinalMetric>,
    /// Conjunction of every scalar in all twelve gates.
    pub passed: bool,
    /// Final response values were evaluated by this artifact.
    pub final_responses_evaluated: bool,
    /// No coefficient refit occurred after development exposure.
    pub refit_performed: bool,
    /// No threshold changed after development exposure.
    pub threshold_changed: bool,
    /// Common fixed-clock Failure Capsule packaging remains unsupported.
    pub common_failure_capsule_created: bool,
    /// A failed verdict must still be retained by the exclusive evidence writer.
    pub retain_failed_evaluation_evidence: bool,
    /// Individual physical parameters remain unqualified.
    pub physical_parameters_qualified: bool,
    /// SHA-256 over serde JSON with this field empty.
    pub content_sha256: String,
}

#[derive(Debug)]
struct Residuals {
    current: Vec<f64>,
    speed: Vec<f64>,
}

fn rmse(values: &[f64]) -> Result<f64> {
    ensure!(!values.is_empty(), "empty PMDC final residuals");
    let value =
        (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    ensure!(value.is_finite(), "nonfinite PMDC final RMSE");
    Ok(value)
}

fn mean(values: &[f64]) -> Result<f64> {
    ensure!(!values.is_empty(), "empty PMDC final residuals");
    let value = values.iter().sum::<f64>() / values.len() as f64;
    ensure!(value.is_finite(), "nonfinite PMDC final bias");
    Ok(value)
}

fn rollout(
    identification: &PmdcTrainingIdentification,
    observations: &[PmdcObservation],
) -> Result<Residuals> {
    ensure!(observations.len() >= 3, "PMDC final run is too short");
    let electrical = &identification.electrical_coefficients;
    let mechanical = &identification.mechanical_coefficients;
    let mut current = observations[0].current_a;
    let mut speed = observations[0].output_speed_rad_s;
    let mut current_residuals = Vec::with_capacity(observations.len() - 1);
    let mut speed_residuals = Vec::with_capacity(observations.len() - 1);
    for pair in observations.windows(2) {
        let dt = pair[1].source_time_s - pair[0].source_time_s;
        ensure!(dt.is_finite() && dt > 0.0, "invalid PMDC final interval");
        let next_speed = speed
            + dt * (mechanical[0] * current
                + mechanical[1] * speed
                + mechanical[2] * speed.signum()
                + mechanical[3]);
        let next_current = match identification.selected_electrical_candidate {
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
            "nonfinite PMDC final rollout"
        );
        current_residuals.push(next_current - pair[1].current_a);
        speed_residuals.push(next_speed - pair[1].output_speed_rad_s);
        current = next_current;
        speed = next_speed;
    }
    Ok(Residuals {
        current: current_residuals,
        speed: speed_residuals,
    })
}

fn metric_value(
    id: &str,
    residuals: &Residuals,
    current_scale: f64,
    speed_scale: f64,
) -> Result<f64> {
    match id {
        "current_rollout_nrmse" => Ok(rmse(&residuals.current)? / current_scale),
        "output_speed_rollout_nrmse" => Ok(rmse(&residuals.speed)? / speed_scale),
        "current_signed_bias_fraction" => Ok(mean(&residuals.current)?.abs() / current_scale),
        "output_speed_signed_bias_fraction" => Ok(mean(&residuals.speed)?.abs() / speed_scale),
        _ => anyhow::bail!("unknown PMDC final metric"),
    }
}

fn evaluation_digest(evidence: &PmdcFinalEvaluation) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

impl PmdcFinalEvaluation {
    fn validate_structure(&self) -> Result<()> {
        let protocol = pmdc_final_evaluation_protocol();
        ensure!(
            self.kind == PMDC_FINAL_EVALUATION_KIND
                && self.schema_version == 1
                && self.final_protocol_sha256 == protocol.sha256()?
                && self.training_identification_sha256
                    == PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256
                && self.final_artifact_sha256 == PMDC_FINAL_ARTIFACT_SHA256
                && self.final_records_sha256 == PMDC_FINAL_RECORDS_SHA256,
            "PMDC final evaluation provenance drift"
        );
        ensure!(
            self.run_ids == ["prbs9_motor_a_trial_10", "prbs9_motor_a_trial_11"]
                && self.rollout_steps_per_run == [2007, 2007],
            "PMDC final run provenance drift"
        );
        ensure!(
            self.metrics.len() == protocol.metrics.len(),
            "PMDC final metric-count drift"
        );
        for (metric, spec) in self.metrics.iter().zip(&protocol.metrics) {
            ensure!(
                metric.id == spec.id
                    && metric.aggregation == spec.aggregation
                    && metric.maximum == spec.maximum,
                "PMDC final metric specification drift"
            );
            let expected_values = match spec.aggregation {
                PmdcFinalAggregation::PerRun => 2,
                PmdcFinalAggregation::Pooled | PmdcFinalAggregation::WorstRun => 1,
            };
            ensure!(
                metric.values.len() == expected_values
                    && metric.values.iter().all(|value| {
                        value.value.is_finite()
                            && value.value >= 0.0
                            && value.passed == (value.value <= metric.maximum)
                    })
                    && metric.passed == metric.values.iter().all(|value| value.passed),
                "PMDC final metric value drift"
            );
            match spec.aggregation {
                PmdcFinalAggregation::PerRun => ensure!(
                    metric.values[0].run_id.as_deref() == Some(&self.run_ids[0])
                        && metric.values[1].run_id.as_deref() == Some(&self.run_ids[1]),
                    "PMDC per-run metric identity drift"
                ),
                PmdcFinalAggregation::Pooled | PmdcFinalAggregation::WorstRun => ensure!(
                    metric.values[0].run_id.is_none(),
                    "PMDC aggregate metric unexpectedly names a run"
                ),
            }
        }
        for group in self.metrics.chunks_exact(3) {
            ensure!(
                group[0].id == group[1].id
                    && group[0].id == group[2].id
                    && group[0].aggregation == PmdcFinalAggregation::PerRun
                    && group[1].aggregation == PmdcFinalAggregation::Pooled
                    && group[2].aggregation == PmdcFinalAggregation::WorstRun,
                "PMDC final aggregation group drift"
            );
            let expected_worst = group[0]
                .values
                .iter()
                .map(|value| value.value)
                .fold(f64::NEG_INFINITY, f64::max);
            ensure!(
                group[2].values[0].value == expected_worst,
                "PMDC worst-run value drift"
            );
        }
        ensure!(
            self.passed == self.metrics.iter().all(|metric| metric.passed),
            "PMDC final verdict drift"
        );
        ensure!(
            self.final_responses_evaluated
                && !self.refit_performed
                && !self.threshold_changed
                && !self.common_failure_capsule_created
                && self.retain_failed_evaluation_evidence
                && !self.physical_parameters_qualified,
            "PMDC final evidence overclaims its scope"
        );
        ensure!(
            self.content_sha256 == evaluation_digest(self)?,
            "PMDC final evidence digest drift"
        );
        Ok(())
    }

    /// Validate the exact sole result, including order, gates, scope and frozen digest.
    pub fn validate(&self) -> Result<()> {
        self.validate_structure()?;
        ensure!(
            self.content_sha256 == PMDC_FINAL_EVALUATION_CONTENT_SHA256,
            "PMDC final evaluation result identity drift"
        );
        Ok(())
    }
}

/// Decode and validate only the exact retained final-evaluation artifact.
pub fn decode_pmdc_final_evaluation(bytes: &[u8]) -> Result<PmdcFinalEvaluation> {
    ensure!(
        bytes.len() == PMDC_FINAL_EVALUATION_ARTIFACT_BYTES,
        "PMDC final evaluation byte length drift"
    );
    ensure!(
        format!("{:x}", Sha256::digest(bytes)) == PMDC_FINAL_EVALUATION_ARTIFACT_SHA256,
        "PMDC final evaluation artifact digest drift"
    );
    ensure!(
        bytes.ends_with(b"\n") && !bytes.contains(&b'\r'),
        "PMDC final evaluation must use canonical LF text"
    );
    let evidence: PmdcFinalEvaluation =
        serde_json::from_slice(bytes).context("decode PMDC final evaluation")?;
    evidence.validate()?;
    Ok(evidence)
}

/// Evaluate both exact final runs once with the frozen training model and gates.
pub fn evaluate_pmdc_final(
    training: &PmdcTrainingSet,
    identification: &PmdcTrainingIdentification,
    final_set: &PmdcFinalSet,
) -> Result<PmdcFinalEvaluation> {
    let protocol = pmdc_final_evaluation_protocol();
    ensure!(
        training.source_sha256 == PMDC_SOURCE_SHA256
            && training.records_sha256 == PMDC_TRAINING_RECORDS_SHA256
            && final_set.source_sha256 == PMDC_SOURCE_SHA256
            && final_set.final_protocol_sha256 == protocol.sha256()?
            && final_set.records_sha256 == PMDC_FINAL_RECORDS_SHA256,
        "PMDC final input identity drift"
    );
    verify_pmdc_training_identification(identification, training)?;
    ensure!(
        identification.content_sha256 == PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256,
        "PMDC final selected-model identity drift"
    );
    let training_runs = training
        .runs
        .iter()
        .map(|run| run.observations())
        .collect::<Result<Vec<_>>>()?;
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
    let run_ids = final_set
        .runs
        .iter()
        .map(|run| run.run_id.clone())
        .collect::<Vec<_>>();
    let observations = final_set
        .runs
        .iter()
        .map(|run| run.observations())
        .collect::<Result<Vec<_>>>()?;
    let residuals = observations
        .iter()
        .map(|run| rollout(identification, run))
        .collect::<Result<Vec<_>>>()?;
    let rollout_steps_per_run = residuals
        .iter()
        .map(|run| run.current.len())
        .collect::<Vec<_>>();
    let mut metrics = Vec::with_capacity(protocol.metrics.len());
    for spec in &protocol.metrics {
        let per_run = residuals
            .iter()
            .map(|run| metric_value(&spec.id, run, current_scale, speed_scale))
            .collect::<Result<Vec<_>>>()?;
        let values = match spec.aggregation {
            PmdcFinalAggregation::PerRun => per_run
                .iter()
                .enumerate()
                .map(|(index, value)| PmdcFinalMetricValue {
                    run_id: Some(run_ids[index].clone()),
                    value: *value,
                    passed: *value <= spec.maximum,
                })
                .collect(),
            PmdcFinalAggregation::WorstRun => {
                let value = per_run.into_iter().fold(f64::NEG_INFINITY, f64::max);
                vec![PmdcFinalMetricValue {
                    run_id: None,
                    value,
                    passed: value <= spec.maximum,
                }]
            }
            PmdcFinalAggregation::Pooled => {
                let pooled = Residuals {
                    current: residuals
                        .iter()
                        .flat_map(|run| run.current.iter().copied())
                        .collect(),
                    speed: residuals
                        .iter()
                        .flat_map(|run| run.speed.iter().copied())
                        .collect(),
                };
                let value = metric_value(&spec.id, &pooled, current_scale, speed_scale)?;
                vec![PmdcFinalMetricValue {
                    run_id: None,
                    value,
                    passed: value <= spec.maximum,
                }]
            }
        };
        let passed = values.iter().all(|value| value.passed);
        metrics.push(PmdcFinalMetric {
            id: spec.id.clone(),
            aggregation: spec.aggregation,
            maximum: spec.maximum,
            values,
            passed,
        });
    }
    let passed = metrics.iter().all(|metric| metric.passed);
    let mut evidence = PmdcFinalEvaluation {
        kind: PMDC_FINAL_EVALUATION_KIND.into(),
        schema_version: 1,
        final_protocol_sha256: protocol.sha256()?,
        training_identification_sha256: identification.content_sha256.clone(),
        final_artifact_sha256: PMDC_FINAL_ARTIFACT_SHA256.into(),
        final_records_sha256: final_set.records_sha256.clone(),
        run_ids,
        rollout_steps_per_run,
        metrics,
        passed,
        final_responses_evaluated: true,
        refit_performed: false,
        threshold_changed: false,
        common_failure_capsule_created: false,
        retain_failed_evaluation_evidence: true,
        physical_parameters_qualified: false,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = evaluation_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aggregate_math_distinguishes_pooled_bias_from_worst_run() {
        let runs = [
            Residuals {
                current: vec![1.0, 1.0],
                speed: vec![2.0, 2.0],
            },
            Residuals {
                current: vec![-1.0, -1.0],
                speed: vec![-2.0, -2.0],
            },
        ];
        assert_eq!(
            metric_value("current_signed_bias_fraction", &runs[0], 2.0, 4.0).unwrap(),
            0.5
        );
        let pooled = Residuals {
            current: runs
                .iter()
                .flat_map(|run| run.current.iter().copied())
                .collect(),
            speed: runs
                .iter()
                .flat_map(|run| run.speed.iter().copied())
                .collect(),
        };
        assert_eq!(
            metric_value("current_signed_bias_fraction", &pooled, 2.0, 4.0).unwrap(),
            0.0
        );
        assert_eq!(
            metric_value("output_speed_rollout_nrmse", &pooled, 2.0, 4.0).unwrap(),
            0.5
        );
    }

    fn schema_fixture() -> PmdcFinalEvaluation {
        let protocol = pmdc_final_evaluation_protocol();
        let run_ids = vec![
            "prbs9_motor_a_trial_10".to_owned(),
            "prbs9_motor_a_trial_11".to_owned(),
        ];
        let metrics = protocol
            .metrics
            .iter()
            .map(|spec| {
                let values = match spec.aggregation {
                    PmdcFinalAggregation::PerRun => run_ids
                        .iter()
                        .map(|run_id| PmdcFinalMetricValue {
                            run_id: Some(run_id.clone()),
                            value: 0.01,
                            passed: true,
                        })
                        .collect(),
                    PmdcFinalAggregation::Pooled | PmdcFinalAggregation::WorstRun => {
                        vec![PmdcFinalMetricValue {
                            run_id: None,
                            value: 0.01,
                            passed: true,
                        }]
                    }
                };
                PmdcFinalMetric {
                    id: spec.id.clone(),
                    aggregation: spec.aggregation,
                    maximum: spec.maximum,
                    values,
                    passed: true,
                }
            })
            .collect();
        let mut evidence = PmdcFinalEvaluation {
            kind: PMDC_FINAL_EVALUATION_KIND.into(),
            schema_version: 1,
            final_protocol_sha256: protocol.sha256().unwrap(),
            training_identification_sha256: PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256.into(),
            final_artifact_sha256: PMDC_FINAL_ARTIFACT_SHA256.into(),
            final_records_sha256: PMDC_FINAL_RECORDS_SHA256.into(),
            run_ids,
            rollout_steps_per_run: vec![2007, 2007],
            metrics,
            passed: true,
            final_responses_evaluated: true,
            refit_performed: false,
            threshold_changed: false,
            common_failure_capsule_created: false,
            retain_failed_evaluation_evidence: true,
            physical_parameters_qualified: false,
            content_sha256: String::new(),
        };
        evidence.content_sha256 = evaluation_digest(&evidence).unwrap();
        evidence
    }

    #[test]
    fn evidence_schema_rejects_worst_run_or_scope_drift() {
        let evidence = schema_fixture();
        evidence.validate_structure().unwrap();
        assert!(evidence.validate().is_err());

        let mut changed = evidence.clone();
        changed.metrics[2].values[0].value = 0.02;
        changed.metrics[2].values[0].passed = true;
        changed.content_sha256 = evaluation_digest(&changed).unwrap();
        assert!(changed.validate_structure().is_err());

        let mut changed = evidence;
        changed.common_failure_capsule_created = true;
        changed.content_sha256 = evaluation_digest(&changed).unwrap();
        assert!(changed.validate_structure().is_err());
    }

    #[test]
    fn sole_result_and_artifact_identities_are_frozen() {
        assert_eq!(PMDC_FINAL_EVALUATION_ARTIFACT_BYTES, 4_791);
        assert_eq!(PMDC_FINAL_EVALUATION_ARTIFACT_SHA256.len(), 64);
        assert_eq!(PMDC_FINAL_EVALUATION_CONTENT_SHA256.len(), 64);
        assert!(decode_pmdc_final_evaluation(b"not-the-retained-evidence").is_err());
    }
}
