//! Reproduction of the published current-magnitude fit, not physical qualification.

use super::OpenMctSeries;
use anyhow::{ensure, Result};
use std::collections::BTreeMap;

/// Why a source row did or did not enter the final fit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationRowUse {
    /// No DMM reference was recorded.
    MissingDmm,
    /// Reference age exceeds the fixed published 10 ms threshold.
    StaleDmm,
    /// Another eligible row carries the same DMM ID with a smaller age.
    ReusedDmm,
    /// Rejected by the published initial-residual MAD rule; not a known fault.
    ResidualRejected,
    /// Included in the final fit; not an independent validation measurement.
    Fit,
}

/// Reproduced affine current-magnitude law, without clipping.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurrentMagnitudeLaw {
    /// Slope in amperes per ADC count.
    pub slope_a_per_count: f64,
    /// Intercept in amperes.
    pub intercept_a: f64,
}

/// Fully retained selection and fit diagnostics for one source recording.
#[derive(Clone, Debug, PartialEq)]
pub struct CurrentCalibrationReproduction {
    /// Exact original raw-log hash.
    pub source_sha256: String,
    /// One decision per original row, in original order.
    pub row_use: Vec<CalibrationRowUse>,
    /// Initial untrimmed candidate fit.
    pub initial_law: CurrentMagnitudeLaw,
    /// Final law after one MAD rejection pass; may be nonphysical.
    pub law: CurrentMagnitudeLaw,
    /// Candidate source row indices, before residual rejection.
    pub candidate_rows: Vec<usize>,
    /// Final-law residual (reference minus predicted current) for every candidate.
    pub candidate_residual_a: Vec<f64>,
    /// Final-law RMSE on all candidates, including rejected residuals.
    pub all_candidate_rmse_a: f64,
    /// Final-law RMSE only on retained fitting rows.
    pub retained_rmse_a: f64,
}

fn predict(law: CurrentMagnitudeLaw, x: f64) -> f64 {
    law.slope_a_per_count * x + law.intercept_a
}

fn fit(points: &[(f64, f64)]) -> Result<CurrentMagnitudeLaw> {
    ensure!(points.len() >= 2, "insufficient calibration points");
    let n = points.len() as f64;
    let mx = points.iter().map(|p| p.0 / n).sum::<f64>();
    let my = points.iter().map(|p| p.1 / n).sum::<f64>();
    let mut xx = 0.0;
    let mut xy = 0.0;
    for &(x, y) in points {
        xx += (x - mx) * (x - mx);
        xy += (x - mx) * (y - my);
    }
    ensure!(
        xx.is_finite() && xy.is_finite() && xx > 0.0,
        "invalid calibration excitation"
    );
    let law = CurrentMagnitudeLaw {
        slope_a_per_count: xy / xx,
        intercept_a: my - xy / xx * mx,
    };
    ensure!(
        law.slope_a_per_count.is_finite() && law.intercept_a.is_finite(),
        "nonfinite calibration law"
    );
    Ok(law)
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    let mid = values.len() / 2;
    if values.len().is_multiple_of(2) {
        values[mid - 1] / 2.0 + values[mid] / 2.0
    } else {
        values[mid]
    }
}

/// Reproduce age <= 10 ms, freshest-per-ID, OLS and one 4*1.4826*MAD refit.
///
/// This is the author's fixed descriptive protocol, not train/holdout calibration.
/// Current is explicitly converted to magnitude; signed regenerative-current
/// information is not inferred. Missing/stale/reused/rejected rows are retained
/// as decisions, and no negative prediction is clipped. Numerical failures return
/// an error instead of fabricating coefficients or a successful verdict.
pub fn reproduce_current_calibration(
    source: &OpenMctSeries,
) -> Result<CurrentCalibrationReproduction> {
    let mut row_use = vec![CalibrationRowUse::MissingDmm; source.samples().len()];
    let mut freshest = BTreeMap::<u64, (usize, f64)>::new();
    for (row, sample) in source.samples().iter().enumerate() {
        if let Some(dmm) = &sample.dmm {
            if dmm.age_ms > 10.0 {
                row_use[row] = CalibrationRowUse::StaleDmm;
                continue;
            }
            row_use[row] = CalibrationRowUse::ReusedDmm;
            let entry = freshest.entry(dmm.sample_id).or_insert((row, dmm.age_ms));
            if dmm.age_ms < entry.1 {
                *entry = (row, dmm.age_ms);
            }
        }
    }
    let candidate_rows: Vec<_> = freshest.values().map(|entry| entry.0).collect();
    let points: Vec<_> = candidate_rows
        .iter()
        .map(|&row| {
            let sample = &source.samples()[row];
            (
                sample.current_adc_counts,
                sample.dmm.as_ref().expect("selected DMM").current_a.abs(),
            )
        })
        .collect();
    let initial_law = fit(&points)?;
    let residuals: Vec<_> = points
        .iter()
        .map(|&(x, y)| y - predict(initial_law, x))
        .collect();
    ensure!(
        residuals.iter().all(|r| r.is_finite()),
        "nonfinite initial residual"
    );
    let center = median(residuals.clone());
    let deviations: Vec<_> = residuals.iter().map(|r| (r - center).abs()).collect();
    ensure!(
        deviations.iter().all(|r| r.is_finite()),
        "nonfinite residual deviation"
    );
    let mad = median(deviations.clone());
    let threshold = 4.0 * 1.4826 * mad;
    ensure!(threshold.is_finite(), "nonfinite MAD threshold");
    let keep: Vec<_> = deviations
        .iter()
        .map(|&d| points.len() < 5 || mad == 0.0 || d <= threshold)
        .collect();
    let retained: Vec<_> = points
        .iter()
        .zip(&keep)
        .filter_map(|(&p, &keep)| keep.then_some(p))
        .collect();
    let law = fit(&retained)?;
    let candidate_residual_a: Vec<_> = points.iter().map(|&(x, y)| y - predict(law, x)).collect();
    let mut all_squared = 0.0;
    let mut retained_squared = 0.0;
    for ((&row, &keep), &r) in candidate_rows.iter().zip(&keep).zip(&candidate_residual_a) {
        ensure!(r.is_finite(), "nonfinite final residual");
        all_squared += r * r;
        if keep {
            retained_squared += r * r;
        }
        row_use[row] = if keep {
            CalibrationRowUse::Fit
        } else {
            CalibrationRowUse::ResidualRejected
        };
    }
    let all_candidate_rmse_a = (all_squared / points.len() as f64).sqrt();
    let retained_rmse_a = (retained_squared / retained.len() as f64).sqrt();
    ensure!(
        all_candidate_rmse_a.is_finite() && retained_rmse_a.is_finite(),
        "nonfinite calibration RMSE"
    );
    Ok(CurrentCalibrationReproduction {
        source_sha256: source.source_sha256().to_owned(),
        row_use,
        initial_law,
        law,
        candidate_rows,
        candidate_residual_a,
        all_candidate_rmse_a,
        retained_rmse_a,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_openmct::read_openmct;

    fn source_from_points(points: &[(f64, f64)]) -> OpenMctSeries {
        let mut text = format!("Mode: Test\nDate: fixture\n{}\n", super::super::HEADER);
        for (i, &(x, y)) in points.iter().enumerate() {
            text.push_str(&format!("0,0,20,{x},0,0,{y},{i},1,{i}\n"));
        }
        read_openmct(text.as_bytes()).unwrap()
    }

    #[test]
    fn rejected_residuals_remain_visible_in_all_candidate_error() {
        let points: Vec<_> = (0..21)
            .map(|i| {
                let x = i as f64;
                let error = if i == 20 {
                    10.0
                } else if i % 2 == 0 {
                    0.001
                } else {
                    -0.001
                };
                (x, 1.0 + 0.1 * x + error)
            })
            .collect();
        let source = source_from_points(&points);
        let result = reproduce_current_calibration(&source).unwrap();
        assert_eq!(result, reproduce_current_calibration(&source).unwrap());
        assert_eq!(result.candidate_rows, (0..21).collect::<Vec<_>>());
        assert_eq!(result.candidate_residual_a.len(), 21);
        assert!(result.row_use[..20]
            .iter()
            .all(|&r| r == CalibrationRowUse::Fit));
        assert_eq!(result.row_use[20], CalibrationRowUse::ResidualRejected);
        assert!(result.retained_rmse_a < 0.002);
        assert!(result.all_candidate_rmse_a > 2.0);
        assert!(result.candidate_residual_a[20] > 9.9);
        assert!((result.law.slope_a_per_count - 0.1).abs() < 0.001);
        assert!((result.initial_law.slope_a_per_count - result.law.slope_a_per_count).abs() > 0.1);
    }

    #[test]
    fn zero_mad_and_numerical_failures_follow_explicit_protocol() {
        let source =
            source_from_points(&[(0.0, 1.0), (1.0, 1.0), (2.0, 1.0), (3.0, 1.0), (4.0, 1.0)]);
        let result = reproduce_current_calibration(&source).unwrap();
        assert!(result.row_use.iter().all(|&r| r == CalibrationRowUse::Fit));
        assert_eq!(result.law.slope_a_per_count, 0.0);
        assert_eq!(result.all_candidate_rmse_a, 0.0);
        for points in [
            vec![(0.0, 1.0)],
            vec![(1.0, 1.0), (1.0, 2.0)],
            vec![(0.0, 1.0), (1e308, 2.0)],
        ] {
            assert!(reproduce_current_calibration(&source_from_points(&points)).is_err());
        }
    }

    #[test]
    fn exact_magnitude_fit_preserves_missing_stale_and_reused_rows() {
        let text = format!("Mode: Test\nDate: fixture\n{}\n0,0,20,0,0,0,nan,nan,nan,-1\n0,0,20,10,0,0,-0.021,0,2,1\n0,0,20,10,0,0,-0.021,0,1,1\n0,0,20,20,0,0,-0.041,1,1,2\n0,0,20,30,0,0,-0.061,2,11,3\n", super::super::HEADER);
        let source = read_openmct(text.as_bytes()).unwrap();
        let result = reproduce_current_calibration(&source).unwrap();
        assert_eq!(result, reproduce_current_calibration(&source).unwrap());
        assert_eq!(result.candidate_rows, vec![2, 3]);
        assert_eq!(
            result.row_use,
            vec![
                CalibrationRowUse::MissingDmm,
                CalibrationRowUse::ReusedDmm,
                CalibrationRowUse::Fit,
                CalibrationRowUse::Fit,
                CalibrationRowUse::StaleDmm
            ]
        );
        assert!((result.law.slope_a_per_count - 0.002).abs() < 1e-14);
        assert!((result.law.intercept_a - 0.001).abs() < 1e-14);
        assert!(result.all_candidate_rmse_a < 1e-14);
        assert!(reproduce_current_calibration(
            &read_openmct(text.replace(",20,0,0,-0.041", ",10,0,0,-0.041").as_bytes()).unwrap()
        )
        .is_err());
    }
}
