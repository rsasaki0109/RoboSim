//! Deterministic training-only effective PMDC identification and whole-run selection.

use super::{
    data::{PmdcObservation, PmdcTrainingSet},
    pmdc_identification_protocol, PmdcElectricalCandidate,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable artifact kind for training-only PMDC effective identification.
pub const PMDC_TRAINING_IDENTIFICATION_KIND: &str = "rne_pmdc_training_identification";

/// Cross-validation result for one predeclared electrical candidate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcCandidateCrossValidation {
    /// Candidate model identity.
    pub candidate: PmdcElectricalCandidate,
    /// Mean of eight held-out whole-run current NRMSE values, when valid.
    pub mean_current_nrmse: Option<f64>,
    /// Per-trial NRMSE in frozen trial order, empty when rejected.
    pub heldout_current_nrmse: Vec<f64>,
    /// Stable rejection reason, or `None` when numerically and physically valid.
    pub rejection: Option<String>,
}

/// Training-only selected effective model; no individual physical constants.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcTrainingIdentification {
    /// Artifact discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Frozen protocol identity.
    pub protocol_sha256: String,
    /// Exact training record-stream identity.
    pub training_records_sha256: String,
    /// Both candidates in predeclared order.
    pub candidate_cross_validation: Vec<PmdcCandidateCrossValidation>,
    /// Candidate selected without development data.
    pub selected_electrical_candidate: PmdcElectricalCandidate,
    /// `[q_v,q_w,q_0]` or `[a_v,a_i,a_w,a_0]` in documented SI-derived units.
    pub electrical_coefficients: Vec<f64>,
    /// `[b_i,b_w,b_s,b_0]` for the effective mechanical equation.
    pub mechanical_coefficients: Vec<f64>,
    /// Smallest absolute QR diagonal divided by the largest across final fits.
    pub minimum_relative_qr_diagonal: f64,
    /// Development input was not read by this producer.
    pub development_evaluated: bool,
    /// Final input was not read by this producer.
    pub final_partition_read: bool,
    /// Individual motor/transmission physical constants remain unqualified.
    pub physical_parameters_qualified: bool,
    /// SHA-256 over serde JSON with this field empty.
    pub content_sha256: String,
}

#[derive(Clone, Debug)]
struct LeastSquaresFit {
    coefficients: Vec<f64>,
    minimum_relative_diagonal: f64,
}

fn solve_householder_qr(
    rows: &[Vec<f64>],
    targets: &[f64],
    relative_diagonal_min: f64,
) -> Result<LeastSquaresFit> {
    ensure!(
        !rows.is_empty() && rows.len() == targets.len(),
        "least-squares row/target mismatch"
    );
    let column_count = rows[0].len();
    ensure!(
        column_count > 0 && rows.len() >= column_count,
        "underdetermined least-squares system"
    );
    ensure!(
        relative_diagonal_min.is_finite() && relative_diagonal_min > 0.0,
        "invalid QR rank threshold"
    );
    ensure!(
        rows.iter()
            .all(|row| { row.len() == column_count && row.iter().all(|value| value.is_finite()) })
            && targets.iter().all(|value| value.is_finite()),
        "nonfinite or ragged least-squares input"
    );
    let row_count = rows.len();
    let mut matrix = rows.iter().flatten().copied().collect::<Vec<_>>();
    let mut rhs = targets.to_vec();
    let mut permutation = (0..column_count).collect::<Vec<_>>();

    for pivot_row in 0..column_count {
        let mut best_column = pivot_row;
        let mut best_norm = -1.0_f64;
        for column in pivot_row..column_count {
            let norm = (pivot_row..row_count)
                .map(|row| matrix[row * column_count + column].powi(2))
                .sum::<f64>();
            ensure!(norm.is_finite(), "nonfinite QR column norm");
            if norm > best_norm
                || (norm == best_norm && permutation[column] < permutation[best_column])
            {
                best_norm = norm;
                best_column = column;
            }
        }
        if best_column != pivot_row {
            for row in 0..row_count {
                matrix.swap(
                    row * column_count + pivot_row,
                    row * column_count + best_column,
                );
            }
            permutation.swap(pivot_row, best_column);
        }

        let norm = (pivot_row..row_count)
            .map(|row| matrix[row * column_count + pivot_row].powi(2))
            .sum::<f64>()
            .sqrt();
        ensure!(norm.is_finite() && norm > 0.0, "rank-deficient QR column");
        let first = matrix[pivot_row * column_count + pivot_row];
        let alpha = if first >= 0.0 { -norm } else { norm };
        let mut reflector = (pivot_row..row_count)
            .map(|row| matrix[row * column_count + pivot_row])
            .collect::<Vec<_>>();
        reflector[0] -= alpha;
        let reflector_norm = reflector.iter().map(|value| value * value).sum::<f64>();
        ensure!(
            reflector_norm.is_finite() && reflector_norm > 0.0,
            "invalid QR reflector"
        );
        let factor = 2.0 / reflector_norm;
        for column in pivot_row..column_count {
            let projection = (pivot_row..row_count)
                .enumerate()
                .map(|(index, row)| reflector[index] * matrix[row * column_count + column])
                .sum::<f64>();
            for (index, row) in (pivot_row..row_count).enumerate() {
                matrix[row * column_count + column] -= factor * reflector[index] * projection;
            }
        }
        let projection = (pivot_row..row_count)
            .enumerate()
            .map(|(index, row)| reflector[index] * rhs[row])
            .sum::<f64>();
        for (index, row) in (pivot_row..row_count).enumerate() {
            rhs[row] -= factor * reflector[index] * projection;
        }
        matrix[pivot_row * column_count + pivot_row] = alpha;
        for row in pivot_row + 1..row_count {
            matrix[row * column_count + pivot_row] = 0.0;
        }
    }

    let diagonals = (0..column_count)
        .map(|index| matrix[index * column_count + index].abs())
        .collect::<Vec<_>>();
    let maximum_diagonal = diagonals.iter().copied().fold(0.0_f64, f64::max);
    let minimum_relative_diagonal = diagonals
        .iter()
        .map(|diagonal| diagonal / maximum_diagonal)
        .fold(f64::INFINITY, f64::min);
    ensure!(
        maximum_diagonal.is_finite()
            && minimum_relative_diagonal.is_finite()
            && minimum_relative_diagonal > relative_diagonal_min,
        "rank-deficient QR design"
    );
    let mut pivoted = vec![0.0; column_count];
    for row in (0..column_count).rev() {
        let known = ((row + 1)..column_count)
            .map(|column| matrix[row * column_count + column] * pivoted[column])
            .sum::<f64>();
        pivoted[row] = (rhs[row] - known) / matrix[row * column_count + row];
    }
    let mut coefficients = vec![0.0; column_count];
    for (pivoted_column, original_column) in permutation.into_iter().enumerate() {
        coefficients[original_column] = pivoted[pivoted_column];
    }
    ensure!(
        coefficients.iter().all(|value| value.is_finite()),
        "nonfinite QR solution"
    );
    Ok(LeastSquaresFit {
        coefficients,
        minimum_relative_diagonal,
    })
}

fn type7_quantile(values: &[f64], probability: f64) -> Result<f64> {
    ensure!(
        !values.is_empty() && (0.0..=1.0).contains(&probability),
        "invalid type-7 quantile request"
    );
    ensure!(
        values.iter().all(|value| value.is_finite()),
        "nonfinite quantile input"
    );
    let mut ordered = values.to_vec();
    ordered.sort_by(f64::total_cmp);
    let position = (ordered.len() - 1) as f64 * probability;
    let lower = position.floor() as usize;
    let fraction = position - lower as f64;
    let upper = (lower + 1).min(ordered.len() - 1);
    Ok(ordered[lower] + fraction * (ordered[upper] - ordered[lower]))
}

pub(super) fn robust_range(values: &[f64]) -> Result<f64> {
    let range = type7_quantile(values, 0.95)? - type7_quantile(values, 0.05)?;
    ensure!(
        range.is_finite() && range > 0.0,
        "invalid P95-P5 normalization range"
    );
    Ok(range)
}

fn fit_electrical(
    candidate: PmdcElectricalCandidate,
    training_runs: &[Vec<PmdcObservation>],
    threshold: f64,
) -> Result<LeastSquaresFit> {
    let mut rows = Vec::new();
    let mut targets = Vec::new();
    for run in training_runs {
        match candidate {
            PmdcElectricalCandidate::QuasiStaticEffective => {
                for sample in run {
                    rows.push(vec![
                        sample.terminal_voltage_v,
                        sample.output_speed_rad_s,
                        1.0,
                    ]);
                    targets.push(sample.current_a);
                }
            }
            PmdcElectricalCandidate::DynamicEulerEffective => {
                for pair in run.windows(2) {
                    let dt = pair[1].source_time_s - pair[0].source_time_s;
                    ensure!(
                        dt.is_finite() && dt > 0.0,
                        "invalid dynamic current interval"
                    );
                    rows.push(vec![
                        dt * pair[0].terminal_voltage_v,
                        dt * pair[0].current_a,
                        dt * pair[0].output_speed_rad_s,
                        dt,
                    ]);
                    targets.push(pair[1].current_a - pair[0].current_a);
                }
            }
        }
    }
    solve_householder_qr(&rows, &targets, threshold)
}

fn electrical_signs_valid(candidate: PmdcElectricalCandidate, coefficients: &[f64]) -> bool {
    match candidate {
        PmdcElectricalCandidate::QuasiStaticEffective => {
            coefficients.len() == 3 && coefficients[0] > 0.0 && coefficients[1] < 0.0
        }
        PmdcElectricalCandidate::DynamicEulerEffective => {
            coefficients.len() == 4
                && coefficients[0] > 0.0
                && coefficients[1] < 0.0
                && coefficients[2] < 0.0
        }
    }
}

fn heldout_current_nrmse(
    candidate: PmdcElectricalCandidate,
    coefficients: &[f64],
    heldout: &[PmdcObservation],
) -> Result<f64> {
    let current_values = heldout
        .iter()
        .map(|sample| sample.current_a)
        .collect::<Vec<_>>();
    let scale = robust_range(&current_values)?;
    let mut squared = Vec::new();
    match candidate {
        PmdcElectricalCandidate::QuasiStaticEffective => {
            for sample in heldout {
                let predicted = coefficients[0] * sample.terminal_voltage_v
                    + coefficients[1] * sample.output_speed_rad_s
                    + coefficients[2];
                squared.push((sample.current_a - predicted).powi(2));
            }
        }
        PmdcElectricalCandidate::DynamicEulerEffective => {
            for pair in heldout.windows(2) {
                let dt = pair[1].source_time_s - pair[0].source_time_s;
                ensure!(
                    dt.is_finite() && dt > 0.0,
                    "invalid held-out current interval"
                );
                let predicted = pair[0].current_a
                    + dt * (coefficients[0] * pair[0].terminal_voltage_v
                        + coefficients[1] * pair[0].current_a
                        + coefficients[2] * pair[0].output_speed_rad_s
                        + coefficients[3]);
                squared.push((pair[1].current_a - predicted).powi(2));
            }
        }
    }
    ensure!(!squared.is_empty(), "empty held-out current evaluation");
    let rmse = (squared.iter().sum::<f64>() / squared.len() as f64).sqrt();
    ensure!(rmse.is_finite(), "nonfinite held-out current RMSE");
    Ok(rmse / scale)
}

fn cross_validate_candidate(
    candidate: PmdcElectricalCandidate,
    runs: &[Vec<PmdcObservation>],
    threshold: f64,
) -> PmdcCandidateCrossValidation {
    let mut scores = Vec::with_capacity(runs.len());
    for heldout_index in 0..runs.len() {
        let training = runs
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != heldout_index)
            .map(|(_, run)| run.clone())
            .collect::<Vec<_>>();
        let fit = match fit_electrical(candidate, &training, threshold) {
            Ok(fit) => fit,
            Err(error) => {
                return PmdcCandidateCrossValidation {
                    candidate,
                    mean_current_nrmse: None,
                    heldout_current_nrmse: Vec::new(),
                    rejection: Some(format!("numerical:{error}")),
                };
            }
        };
        if !electrical_signs_valid(candidate, &fit.coefficients) {
            return PmdcCandidateCrossValidation {
                candidate,
                mean_current_nrmse: None,
                heldout_current_nrmse: Vec::new(),
                rejection: Some("physical_signs".into()),
            };
        }
        match heldout_current_nrmse(candidate, &fit.coefficients, &runs[heldout_index]) {
            Ok(score) => scores.push(score),
            Err(error) => {
                return PmdcCandidateCrossValidation {
                    candidate,
                    mean_current_nrmse: None,
                    heldout_current_nrmse: Vec::new(),
                    rejection: Some(format!("evaluation:{error}")),
                };
            }
        }
    }
    let mean = scores.iter().sum::<f64>() / scores.len() as f64;
    PmdcCandidateCrossValidation {
        candidate,
        mean_current_nrmse: Some(mean),
        heldout_current_nrmse: scores,
        rejection: None,
    }
}

fn fit_mechanical(runs: &[Vec<PmdcObservation>], threshold: f64) -> Result<LeastSquaresFit> {
    let mut rows = Vec::new();
    let mut targets = Vec::new();
    for run in runs {
        for pair in run.windows(2) {
            let dt = pair[1].source_time_s - pair[0].source_time_s;
            ensure!(dt.is_finite() && dt > 0.0, "invalid mechanical interval");
            rows.push(vec![
                dt * pair[0].current_a,
                dt * pair[0].output_speed_rad_s,
                dt * pair[0].output_speed_rad_s.signum(),
                dt,
            ]);
            targets.push(pair[1].output_speed_rad_s - pair[0].output_speed_rad_s);
        }
    }
    let fit = solve_householder_qr(&rows, &targets, threshold)?;
    ensure!(
        fit.coefficients[0] > 0.0 && fit.coefficients[1] <= 0.0 && fit.coefficients[2] <= 0.0,
        "mechanical effective coefficients violate physical signs"
    );
    Ok(fit)
}

pub(super) fn identify_observations(
    records_sha256: &str,
    runs: &[Vec<PmdcObservation>],
) -> Result<PmdcTrainingIdentification> {
    let protocol = pmdc_identification_protocol();
    protocol.validate()?;
    ensure!(
        runs.len() == 8,
        "PMDC identification requires eight training runs"
    );
    let candidates = protocol
        .electrical_candidates
        .iter()
        .map(|candidate| {
            cross_validate_candidate(
                *candidate,
                runs,
                protocol.selection.rank_relative_diagonal_min,
            )
        })
        .collect::<Vec<_>>();
    let quasi_score = candidates[0]
        .mean_current_nrmse
        .context("quasi-static PMDC candidate rejected")?;
    let dynamic_score = candidates[1].mean_current_nrmse;
    let selected = if dynamic_score.is_some_and(|score| {
        quasi_score - score >= protocol.selection.dynamic_min_current_nrmse_improvement
    }) {
        PmdcElectricalCandidate::DynamicEulerEffective
    } else {
        PmdcElectricalCandidate::QuasiStaticEffective
    };
    let electrical = fit_electrical(
        selected,
        runs,
        protocol.selection.rank_relative_diagonal_min,
    )?;
    ensure!(
        electrical_signs_valid(selected, &electrical.coefficients),
        "selected PMDC electrical fit violates physical signs"
    );
    let mechanical = fit_mechanical(runs, protocol.selection.rank_relative_diagonal_min)?;
    let mut evidence = PmdcTrainingIdentification {
        kind: PMDC_TRAINING_IDENTIFICATION_KIND.into(),
        schema_version: 1,
        protocol_sha256: protocol.sha256()?,
        training_records_sha256: records_sha256.into(),
        candidate_cross_validation: candidates,
        selected_electrical_candidate: selected,
        electrical_coefficients: electrical.coefficients,
        mechanical_coefficients: mechanical.coefficients,
        minimum_relative_qr_diagonal: electrical
            .minimum_relative_diagonal
            .min(mechanical.minimum_relative_diagonal),
        development_evaluated: false,
        final_partition_read: false,
        physical_parameters_qualified: false,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = evidence_digest(&evidence)?;
    Ok(evidence)
}

fn evidence_digest(evidence: &PmdcTrainingIdentification) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

impl PmdcTrainingIdentification {
    /// Validate artifact scope, finite metrics and content digest.
    pub fn validate(&self) -> Result<()> {
        let protocol = pmdc_identification_protocol();
        ensure!(
            self.kind == PMDC_TRAINING_IDENTIFICATION_KIND && self.schema_version == 1,
            "PMDC training identification kind/schema drift"
        );
        ensure!(
            self.protocol_sha256 == protocol.sha256()?
                && self.training_records_sha256 == super::PMDC_TRAINING_RECORDS_SHA256,
            "PMDC training identification provenance drift"
        );
        ensure!(
            self.candidate_cross_validation.len() == protocol.electrical_candidates.len()
                && self
                    .candidate_cross_validation
                    .iter()
                    .zip(&protocol.electrical_candidates)
                    .all(|(result, expected)| result.candidate == *expected),
            "PMDC candidate order drift"
        );
        for result in &self.candidate_cross_validation {
            match (result.mean_current_nrmse, &result.rejection) {
                (Some(mean), None) => ensure!(
                    mean.is_finite()
                        && mean >= 0.0
                        && result.heldout_current_nrmse.len() == 8
                        && result
                            .heldout_current_nrmse
                            .iter()
                            .all(|value| value.is_finite() && *value >= 0.0),
                    "invalid PMDC candidate CV metrics"
                ),
                (None, Some(reason)) => ensure!(
                    !reason.is_empty() && result.heldout_current_nrmse.is_empty(),
                    "invalid PMDC candidate rejection"
                ),
                _ => anyhow::bail!("inconsistent PMDC candidate result"),
            }
        }
        let quasi_score = self.candidate_cross_validation[0]
            .mean_current_nrmse
            .context("frozen quasi-static candidate must remain valid")?;
        let expected_selected = if self.candidate_cross_validation[1]
            .mean_current_nrmse
            .is_some_and(|score| {
                quasi_score - score >= protocol.selection.dynamic_min_current_nrmse_improvement
            }) {
            PmdcElectricalCandidate::DynamicEulerEffective
        } else {
            PmdcElectricalCandidate::QuasiStaticEffective
        };
        ensure!(
            self.selected_electrical_candidate == expected_selected
                && electrical_signs_valid(expected_selected, &self.electrical_coefficients),
            "PMDC selected electrical result drift"
        );
        ensure!(
            self.mechanical_coefficients.len() == 4
                && self.mechanical_coefficients[0] > 0.0
                && self.mechanical_coefficients[1] <= 0.0
                && self.mechanical_coefficients[2] <= 0.0,
            "PMDC mechanical result drift"
        );
        ensure!(
            !self.development_evaluated
                && !self.final_partition_read
                && !self.physical_parameters_qualified,
            "PMDC training evidence overclaims its scope"
        );
        ensure!(
            self.electrical_coefficients
                .iter()
                .all(|value| value.is_finite())
                && self
                    .mechanical_coefficients
                    .iter()
                    .all(|value| value.is_finite())
                && self.minimum_relative_qr_diagonal.is_finite()
                && self.minimum_relative_qr_diagonal
                    > protocol.selection.rank_relative_diagonal_min,
            "nonfinite PMDC training evidence"
        );
        ensure!(
            self.content_sha256 == evidence_digest(self)?,
            "PMDC training evidence digest drift"
        );
        Ok(())
    }
}

/// Fit and select effective models using only the exact retained training runs.
pub fn identify_pmdc_training(training: &PmdcTrainingSet) -> Result<PmdcTrainingIdentification> {
    ensure!(
        training.source_sha256 == super::PMDC_SOURCE_SHA256
            && training.records_sha256 == super::PMDC_TRAINING_RECORDS_SHA256,
        "PMDC training identity drift"
    );
    let runs = training
        .runs
        .iter()
        .map(|run| run.observations())
        .collect::<Result<Vec<_>>>()?;
    let evidence = identify_observations(&training.records_sha256, &runs)?;
    evidence.validate()?;
    Ok(evidence)
}

pub(super) fn verify_observations(
    evidence: &PmdcTrainingIdentification,
    records_sha256: &str,
    runs: &[Vec<PmdcObservation>],
) -> Result<()> {
    ensure!(
        evidence == &identify_observations(records_sha256, runs)?,
        "PMDC training identification does not reproduce from source observations"
    );
    Ok(())
}

/// Rerun the complete training fit and require exact evidence equality.
pub fn verify_pmdc_training_identification(
    evidence: &PmdcTrainingIdentification,
    training: &PmdcTrainingSet,
) -> Result<()> {
    evidence.validate()?;
    ensure!(
        training.source_sha256 == super::PMDC_SOURCE_SHA256
            && training.records_sha256 == super::PMDC_TRAINING_RECORDS_SHA256,
        "PMDC verification input identity drift"
    );
    let runs = training
        .runs
        .iter()
        .map(|run| run.observations())
        .collect::<Result<Vec<_>>>()?;
    verify_observations(evidence, &training.records_sha256, &runs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pivoted_householder_recovers_coefficients_and_rejects_rank_loss() {
        let rows = vec![
            vec![1.0, 0.0, 1.0],
            vec![0.0, 1.0, 1.0],
            vec![2.0, 1.0, 1.0],
            vec![-1.0, 3.0, 1.0],
        ];
        let expected = [2.0, -3.0, 0.5];
        let targets = rows
            .iter()
            .map(|row| row.iter().zip(expected).map(|(x, c)| x * c).sum())
            .collect::<Vec<f64>>();
        let fit = solve_householder_qr(&rows, &targets, 1e-10).unwrap();
        for (actual, expected) in fit.coefficients.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-12);
        }
        let rank_deficient = vec![vec![1.0, 2.0], vec![2.0, 4.0], vec![3.0, 6.0]];
        assert!(solve_householder_qr(&rank_deficient, &[1.0, 2.0, 3.0], 1e-10).is_err());
    }

    #[test]
    fn type7_quantile_matches_declared_linear_interpolation() {
        let values = [0.0, 10.0, 20.0, 30.0, 40.0];
        assert_eq!(type7_quantile(&values, 0.05).unwrap(), 2.0);
        assert_eq!(type7_quantile(&values, 0.95).unwrap(), 38.0);
        assert_eq!(robust_range(&values).unwrap(), 36.0);
    }

    fn synthetic_runs() -> Vec<Vec<PmdcObservation>> {
        (0..8)
            .map(|trial| {
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
                    let next_speed = speed
                        + 0.01 * (2.0 * current - 0.15 * speed - 0.03 * speed.signum() + 0.02);
                    current = next_current;
                    speed = next_speed;
                }
                run
            })
            .collect()
    }

    #[test]
    fn whole_run_selection_recovers_dynamic_effective_model_without_scope_claims() {
        let runs = synthetic_runs();
        let quasi_fit = fit_electrical(
            PmdcElectricalCandidate::QuasiStaticEffective,
            &runs[..7],
            1e-10,
        )
        .unwrap();
        assert!(
            electrical_signs_valid(
                PmdcElectricalCandidate::QuasiStaticEffective,
                &quasi_fit.coefficients
            ),
            "{:?}",
            quasi_fit.coefficients
        );
        let quasi =
            cross_validate_candidate(PmdcElectricalCandidate::QuasiStaticEffective, &runs, 1e-10);
        assert!(quasi.rejection.is_none(), "{quasi:#?}");
        let evidence = identify_observations("synthetic", &runs).unwrap();
        assert_eq!(evidence.content_sha256, evidence_digest(&evidence).unwrap());
        verify_observations(&evidence, "synthetic", &runs).unwrap();
        let mut forged = evidence.clone();
        forged.electrical_coefficients[0] += 1e-6;
        forged.content_sha256 = evidence_digest(&forged).unwrap();
        assert!(verify_observations(&forged, "synthetic", &runs).is_err());
        assert_eq!(
            evidence.selected_electrical_candidate,
            PmdcElectricalCandidate::DynamicEulerEffective
        );
        let expected_electrical = [8.0, -12.0, -50.0, 0.1];
        for (actual, expected) in evidence
            .electrical_coefficients
            .iter()
            .zip(expected_electrical)
        {
            assert!((actual - expected).abs() < 1e-8);
        }
        let expected_mechanical = [2.0, -0.15, -0.03, 0.02];
        for (actual, expected) in evidence
            .mechanical_coefficients
            .iter()
            .zip(expected_mechanical)
        {
            assert!((actual - expected).abs() < 1e-8);
        }
        assert!(!evidence.development_evaluated);
        assert!(!evidence.final_partition_read);
        assert!(!evidence.physical_parameters_qualified);
    }
}
