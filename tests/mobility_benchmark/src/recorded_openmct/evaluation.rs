//! Frozen training-law diagnostics, without holdout refitting or qualification.

use super::{
    calibration::{
        reproduce_current_calibration, CalibrationRowUse, CurrentCalibrationReproduction,
    },
    OpenMctDmm, OpenMctSeries,
};
use anyhow::{ensure, Result};
use std::collections::BTreeMap;

/// Evidence for every original evaluation row, including unavailable references.
#[derive(Clone, Debug, PartialEq)]
pub struct CurrentEvaluationRow {
    /// Zero-based original source index.
    pub source_row: usize,
    /// Original asynchronous reference, including signed current and age.
    pub dmm: Option<OpenMctDmm>,
    /// Unclipped prediction from the training-only law.
    pub predicted_magnitude_a: f64,
    /// Absolute reference current minus prediction; absent only without DMM.
    pub residual_a: Option<f64>,
    /// Selected by age <= 10 ms and freshest ID, never by residual size.
    pub selected: bool,
    /// Original ADC input to the frozen law.
    pub current_adc_counts: f64,
    /// Outside the inclusive retained-fit ADC interval; not a rejection rule.
    pub extrapolated: bool,
}

/// Separate-capture diagnostic; does not attest experiment independence.
#[derive(Clone, Debug, PartialEq)]
pub struct CurrentCalibrationEvaluation {
    /// Recomputed training-only fit and complete training selection evidence.
    pub training: CurrentCalibrationReproduction,
    /// Exact bytes of the evaluation source.
    pub evaluation_sha256: String,
    /// All original evaluation rows; stale/reused references are not erased.
    pub rows: Vec<CurrentEvaluationRow>,
    /// Number of distinct selected DMM references.
    pub selected_count: usize,
    /// RMSE on selected references, with no residual trimming.
    pub rmse_a: f64,
    /// Maximum absolute selected residual, with no clipping.
    pub max_absolute_residual_a: f64,
    /// Inclusive minimum/maximum ADC counts from retained fitting rows only.
    /// Being inside this interval does not imply calibrated accuracy.
    pub fitted_adc_range_counts: [f64; 2],
}

/// Fit only the training capture, then apply its frozen law to another capture.
///
/// Rejects identical bytes or identical numeric rows (including re-encoded logs).
/// These checks cannot prove independent acquisition. No acceptance threshold is
/// selected here. DMM age is a source declaration, not integration-time alignment.
pub fn evaluate_current_calibration(
    training: &OpenMctSeries,
    evaluation: &OpenMctSeries,
) -> Result<CurrentCalibrationEvaluation> {
    ensure!(
        training.source_sha256() != evaluation.source_sha256()
            && training.samples() != evaluation.samples(),
        "evaluation duplicates training capture"
    );
    let fit = reproduce_current_calibration(training)?;
    let mut fitted_adc_range_counts = [f64::INFINITY, f64::NEG_INFINITY];
    for (sample, decision) in training.samples().iter().zip(&fit.row_use) {
        if *decision == CalibrationRowUse::Fit {
            fitted_adc_range_counts[0] = fitted_adc_range_counts[0].min(sample.current_adc_counts);
            fitted_adc_range_counts[1] = fitted_adc_range_counts[1].max(sample.current_adc_counts);
        }
    }
    let training = fit;
    let mut freshest = BTreeMap::<u64, (usize, f64)>::new();
    let mut rows = Vec::with_capacity(evaluation.samples().len());
    for (source_row, sample) in evaluation.samples().iter().enumerate() {
        let predicted_magnitude_a =
            training.law.slope_a_per_count * sample.current_adc_counts + training.law.intercept_a;
        ensure!(
            predicted_magnitude_a.is_finite(),
            "nonfinite evaluation prediction"
        );
        let residual_a = sample
            .dmm
            .as_ref()
            .map(|d| d.current_a.abs() - predicted_magnitude_a);
        ensure!(
            residual_a.is_none_or(f64::is_finite),
            "nonfinite evaluation residual"
        );
        if let Some(dmm) = &sample.dmm {
            if dmm.age_ms <= 10.0 {
                let entry = freshest
                    .entry(dmm.sample_id)
                    .or_insert((source_row, dmm.age_ms));
                if dmm.age_ms < entry.1 {
                    *entry = (source_row, dmm.age_ms);
                }
            }
        }
        rows.push(CurrentEvaluationRow {
            source_row,
            dmm: sample.dmm.clone(),
            predicted_magnitude_a,
            residual_a,
            selected: false,
            current_adc_counts: sample.current_adc_counts,
            extrapolated: sample.current_adc_counts < fitted_adc_range_counts[0]
                || sample.current_adc_counts > fitted_adc_range_counts[1],
        });
    }
    ensure!(!freshest.is_empty(), "no eligible evaluation references");
    let mut squared = 0.0;
    let mut max_absolute_residual_a = 0.0_f64;
    for &(row, _) in freshest.values() {
        rows[row].selected = true;
        let residual = rows[row].residual_a.expect("selected reference");
        squared += residual * residual;
        max_absolute_residual_a = max_absolute_residual_a.max(residual.abs());
    }
    let rmse_a = (squared / freshest.len() as f64).sqrt();
    ensure!(rmse_a.is_finite(), "nonfinite evaluation RMSE");
    Ok(CurrentCalibrationEvaluation {
        training,
        evaluation_sha256: evaluation.source_sha256().to_owned(),
        rows,
        selected_count: freshest.len(),
        rmse_a,
        max_absolute_residual_a,
        fitted_adc_range_counts,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_openmct::read_openmct;

    #[test]
    fn rejected_training_extreme_does_not_expand_fitted_support() {
        let header = format!("Mode: Test\nDate: fixture\n{}\n", super::super::HEADER);
        let mut text = header.clone();
        for i in 0..21 {
            let error = if i == 20 {
                10.0
            } else if i % 2 == 0 {
                0.001
            } else {
                -0.001
            };
            let current = 1.0 + 0.1 * i as f64 + error;
            text.push_str(&format!("0,0,10,{i},0,0,{current},{i},1,{i}\n"));
        }
        let train = read_openmct(text.as_bytes()).unwrap();
        let held = read_openmct(
            format!("{header}0,0,10,19,0,0,2.9,0,1,0\n0,0,10,20,0,0,3,1,1,1\n").as_bytes(),
        )
        .unwrap();
        let result = evaluate_current_calibration(&train, &held).unwrap();
        assert_eq!(
            result.training.row_use[20],
            CalibrationRowUse::ResidualRejected
        );
        assert_eq!(result.fitted_adc_range_counts, [0.0, 19.0]);
        assert!(!result.rows[0].extrapolated);
        assert!(result.rows[1].extrapolated);
        assert_eq!(result.selected_count, 2);
        assert!(result.rows.iter().all(|row| row.selected));
    }

    #[test]
    fn selection_preserves_missing_stale_ties_and_negative_predictions() {
        let header = format!("Mode: Test\nDate: fixture\n{}\n", super::super::HEADER);
        let train = read_openmct(
            format!("{header}0,0,10,3,0,0,1,0,1,1\n0,0,10,4,0,0,2,1,1,2\n").as_bytes(),
        )
        .unwrap();
        let held_text = format!("{header}0,0,10,0,0,0,nan,nan,nan,-1\n0,0,10,0,0,0,0,0,10,1\n0,0,10,1,0,0,0,0,9,1\n0,0,10,2,0,0,0,0,9,1\n0,0,10,3,0,0,2,1,11,2\n");
        let held = read_openmct(held_text.as_bytes()).unwrap();
        let report = evaluate_current_calibration(&train, &held).unwrap();
        assert_eq!(report.rows.len(), 5);
        assert_eq!(report.selected_count, 1);
        assert_eq!(report.fitted_adc_range_counts, [3.0, 4.0]);
        assert!(report.rows[2].extrapolated);
        assert!(!report.rows[4].extrapolated);
        assert_eq!(report.rows[2].current_adc_counts, 1.0);
        assert_eq!(
            report.rows.iter().map(|r| r.selected).collect::<Vec<_>>(),
            vec![false, false, true, false, false]
        );
        assert_eq!(report.rows[0].residual_a, None);
        assert_eq!(report.rows[0].predicted_magnitude_a, -2.0);
        assert_eq!(report.rows[2].predicted_magnitude_a, -1.0);
        assert_eq!(report.rmse_a, 1.0);
        assert_eq!(report.rows[4].residual_a, Some(1.0));
        for (result_row, source_row) in report.rows.iter().zip(held.samples()) {
            assert_eq!(result_row.dmm, source_row.dmm);
        }
        for body in [
            "0,0,10,0,0,0,nan,nan,nan,-1\n",
            "0,0,10,1,0,0,1,0,11,1\n",
            "0,0,10,1e308,0,0,1,0,1,1\n",
        ] {
            let invalid = read_openmct(format!("{header}{body}").as_bytes()).unwrap();
            assert!(evaluate_current_calibration(&train, &invalid).is_err());
        }
    }

    #[test]
    fn holdout_errors_are_retained_without_refitting() {
        let text = format!(
            "Mode: Test\nDate: fixture\n{}\n0,0,10,1,0,0,1,0,1,1\n0,0,10,2,0,0,2,1,1,2\n",
            super::super::HEADER
        );
        let train = read_openmct(text.as_bytes()).unwrap();
        assert!(evaluate_current_calibration(&train, &train).is_err());
        let reencoded = read_openmct(text.replace('\n', "\r\n").as_bytes()).unwrap();
        assert!(evaluate_current_calibration(&train, &reencoded).is_err());
        let holdout = read_openmct(text.replace(",2,1,1,2", ",102,1,1,2").as_bytes()).unwrap();
        let report = evaluate_current_calibration(&train, &holdout).unwrap();
        assert_eq!(
            report,
            evaluate_current_calibration(&train, &holdout).unwrap()
        );
        assert_eq!(
            report.training,
            reproduce_current_calibration(&train).unwrap()
        );
        assert_eq!(report.selected_count, 2);
        assert_eq!(report.rows[1].residual_a, Some(100.0));
        assert!(report.rmse_a > 70.0);
        assert_eq!(report.max_absolute_residual_a, 100.0);
    }
}
