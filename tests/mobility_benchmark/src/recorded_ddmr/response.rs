//! Exploratory first-order recorded wheel response, not physical identification.
//!
//! For each wheel independently, fit `w[k+1] = a*w[k] + b*u[k] + c` by centered
//! least squares. The fixed one-row delay is a modeling assumption, not measured
//! actuator latency. Coefficients are empirical and must not become motor/tire
//! constants. No held-out samples enter fitting, no stability clamp is applied.

use super::DdmrPartition;
use anyhow::{ensure, Result};
use serde::Serialize;

/// Fixed model convention used in evidence reports.
pub const RESPONSE_ALGORITHM: &str = "independent_wheel_arx_1_1_1_offset_ols_v1";

/// Immutable two-wheel empirical model learned from one raw-row partition.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct WheelResponseModel {
    /// Each wheel's `[a, b, c]`: dimensionless, (rad/s)/V, rad/s respectively.
    coefficients: [[f64; 3]; 2],
    /// Assumed discrete sample interval in source seconds.
    sample_interval_s: f64,
    /// Allowed absolute source-interval discrepancy, not sensor jitter calibration.
    interval_tolerance_s: f64,
    /// Number of training transitions used per wheel.
    training_transitions: usize,
}

impl WheelResponseModel {
    /// Fit only the supplied training partition, requiring excited independent regressors.
    ///
    /// A normalized determinant below 1e-10 is rejected as ill-conditioned. Both
    /// speed and voltage must vary. Intervals must match the explicit model clock.
    /// This tests numerical identifiability of the regression, not physical
    /// identifiability of motor parameters or unbiasedness under sensor noise.
    pub fn fit(
        training: &DdmrPartition<'_>,
        sample_interval_s: f64,
        interval_tolerance_s: f64,
    ) -> Result<Self> {
        check_clock(training, sample_interval_s, interval_tolerance_s)?;
        let n = training.samples.len() - 1;
        ensure!(
            n >= 4,
            "wheel response needs at least four training transitions"
        );
        let mut coefficients = [[0.0; 3]; 2];
        for (wheel, coefficient) in coefficients.iter_mut().enumerate() {
            let mut means = [0.0; 3];
            for pair in training.samples.windows(2) {
                means[0] += pair[0].recorded_speed_rad_s[wheel];
                means[1] += pair[0].recorded_voltage_v[wheel];
                means[2] += pair[1].recorded_speed_rad_s[wheel];
            }
            for mean in &mut means {
                *mean /= n as f64;
            }
            let (mut xx, mut uu, mut xu, mut xy, mut uy) = (0.0, 0.0, 0.0, 0.0, 0.0);
            for pair in training.samples.windows(2) {
                let x = pair[0].recorded_speed_rad_s[wheel] - means[0];
                let u = pair[0].recorded_voltage_v[wheel] - means[1];
                let y = pair[1].recorded_speed_rad_s[wheel] - means[2];
                xx += x * x;
                uu += u * u;
                xu += x * u;
                xy += x * y;
                uy += u * y;
            }
            ensure!(
                [xx, uu, xu, xy, uy].iter().all(|v| v.is_finite())
                    && means.iter().all(|v| v.is_finite()),
                "wheel {wheel}: nonfinite regression arithmetic"
            );
            ensure!(xx > 0.0 && uu > 0.0, "wheel {wheel}: unexcited regression");
            // Normalize before forming the determinant to avoid multiplying energies.
            let correlation = (xu / xx.sqrt()) / uu.sqrt();
            let determinant = 1.0 - correlation * correlation;
            ensure!(
                determinant.is_finite() && determinant > 1e-10,
                "wheel {wheel}: rank-deficient or ill-conditioned regression"
            );
            let a = (xy / xx - (xu / xx) * (uy / uu)) / determinant;
            let b = (uy / uu - (xu / uu) * (xy / xx)) / determinant;
            let c = means[2] - a * means[0] - b * means[1];
            ensure!(
                [a, b, c].iter().all(|v| v.is_finite()),
                "wheel {wheel}: nonfinite coefficients"
            );
            *coefficient = [a, b, c];
        }
        Ok(Self {
            coefficients,
            sample_interval_s,
            interval_tolerance_s,
            training_transitions: n,
        })
    }

    /// Frozen empirical coefficients, with no physical-parameter interpretation.
    pub fn coefficients(&self) -> [[f64; 3]; 2] {
        self.coefficients
    }

    /// Whether each scalar recurrence has a pole strictly inside the unit circle.
    /// This is not a real-world validity or controller-stability guarantee.
    pub fn stable_poles(&self) -> [bool; 2] {
        self.coefficients
            .map(|coefficient| coefficient[0].abs() < 1.0)
    }

    /// Evaluate frozen parameters without fitting or choosing an initial state.
    ///
    /// Both modes start at the partition's first measured speed. One-step mode
    /// then uses each previous measured speed; free-run mode uses only previous
    /// predictions. Both consume recorded voltage at k to predict row k+1.
    /// Their baselines respectively persist the last measurement or the initial
    /// measurement. Every transition is scored, including poor predictions.
    /// Nonfinite arithmetic returns an error rather than clipped/partial metrics.
    pub fn evaluate(
        &self,
        partition: &DdmrPartition<'_>,
        mode: ResponseMode,
    ) -> Result<ResponseMetrics> {
        check_clock(partition, self.sample_interval_s, self.interval_tolerance_s)?;
        let initial = partition.samples[0].recorded_speed_rad_s;
        let mut predicted = initial;
        let (mut squared, mut baseline_squared, mut maximum) =
            ([0.0_f64; 2], [0.0_f64; 2], [0.0_f64; 2]);
        for (index, pair) in partition.samples.windows(2).enumerate() {
            for wheel in 0..2 {
                let prior = match mode {
                    ResponseMode::OneStep => pair[0].recorded_speed_rad_s[wheel],
                    ResponseMode::FreeRun => predicted[wheel],
                };
                let baseline = match mode {
                    ResponseMode::OneStep => pair[0].recorded_speed_rad_s[wheel],
                    ResponseMode::FreeRun => initial[wheel],
                };
                let [a, b, c] = self.coefficients[wheel];
                predicted[wheel] = a * prior + b * pair[0].recorded_voltage_v[wheel] + c;
                let error = predicted[wheel] - pair[1].recorded_speed_rad_s[wheel];
                let baseline_error = baseline - pair[1].recorded_speed_rad_s[wheel];
                squared[wheel] += error * error;
                baseline_squared[wheel] += baseline_error * baseline_error;
                maximum[wheel] = maximum[wheel].max(error.abs());
                ensure!(
                    predicted[wheel].is_finite()
                        && squared[wheel].is_finite()
                        && baseline_squared[wheel].is_finite(),
                    "wheel {wheel}: nonfinite evaluation at transition {index}"
                );
            }
        }
        let transitions = partition.samples.len() - 1;
        Ok(ResponseMetrics {
            mode,
            transitions,
            rmse_rad_s: squared.map(|value| (value / transitions as f64).sqrt()),
            baseline_rmse_rad_s: baseline_squared.map(|value| (value / transitions as f64).sqrt()),
            maximum_absolute_error_rad_s: maximum,
        })
    }
}

/// Explicit distinction between corrected prediction and uncorrected simulation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseMode {
    /// Previous measured output is supplied at every transition.
    OneStep,
    /// Only the first measured output initializes the model state.
    FreeRun,
}

/// SI-unit errors over every transition in a single partition, ordered left/right.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ResponseMetrics {
    /// Evaluation and baseline interpretation.
    pub mode: ResponseMode,
    /// Identical scored population for model and baseline, per wheel.
    pub transitions: usize,
    /// Model root-mean-square wheel-speed error.
    pub rmse_rad_s: [f64; 2],
    /// Persistence or initial-state baseline, according to mode.
    pub baseline_rmse_rad_s: [f64; 2],
    /// Worst absolute model error; failures are not trimmed.
    pub maximum_absolute_error_rad_s: [f64; 2],
}

fn check_clock(partition: &DdmrPartition<'_>, dt: f64, tolerance: f64) -> Result<()> {
    ensure!(
        dt.is_finite() && dt > 0.0 && tolerance.is_finite() && tolerance >= 0.0 && tolerance < dt,
        "invalid wheel-response source interval or tolerance"
    );
    ensure!(
        partition.samples.len() >= 2,
        "wheel-response partition has no transitions"
    );
    ensure!(
        partition
            .samples
            .windows(2)
            .all(|pair| ((pair[1].source_time_s - pair[0].source_time_s) - dt).abs() <= tolerance),
        "source intervals do not match the discrete wheel-response model"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_ddmr::{read_ddmr_samples, HEADER};

    fn synthetic() -> String {
        let mut text = format!("{HEADER}\n");
        let mut speed = [0.4, -0.7];
        for i in 0..120 {
            let voltage = [(i % 7) as f64 - 3.0, (i % 11) as f64 - 5.0];
            text.push_str(&format!(
                "{},{},{},{},{}\n",
                i as f64 * 0.01,
                voltage[0],
                voltage[1],
                speed[0],
                speed[1]
            ));
            speed = [
                0.7 * speed[0] + 0.3 * voltage[0] + 0.1,
                0.5 * speed[1] + 0.4 * voltage[1] - 0.2,
            ];
        }
        text
    }

    #[test]
    fn exact_synthetic_recovery_and_both_holdout_modes() {
        let series = read_ddmr_samples(synthetic().as_bytes()).unwrap();
        let split = series.split(60, 90).unwrap();
        let model = WheelResponseModel::fit(&split.training, 0.01, 1e-9).unwrap();
        assert_eq!(
            model,
            WheelResponseModel::fit(&split.training, 0.01, 1e-9).unwrap()
        );
        for (actual, expected) in model
            .coefficients()
            .into_iter()
            .flatten()
            .zip([0.7, 0.3, 0.1, 0.5, 0.4, -0.2])
        {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert_eq!(model.stable_poles(), [true, true]);
        for partition in [&split.validation, &split.test] {
            for mode in [ResponseMode::OneStep, ResponseMode::FreeRun] {
                let metrics = model.evaluate(partition, mode).unwrap();
                assert_eq!(metrics.transitions, 29);
                assert!(metrics.rmse_rad_s.iter().all(|v| *v < 1e-12));
                assert!(metrics.baseline_rmse_rad_s.iter().all(|v| *v > 0.01));
            }
        }
    }

    #[test]
    fn heldout_labels_do_not_fit_or_correct_free_running_state() {
        let mut series = read_ddmr_samples(synthetic().as_bytes()).unwrap();
        let original = series.split(60, 90).unwrap();
        let model = WheelResponseModel::fit(&original.training, 0.01, 1e-9).unwrap();
        // Change one noninitial held-out label. Free-running predictions remain
        // exact elsewhere, so its total squared error is exactly 10^2.
        series.samples[95].recorded_speed_rad_s[0] += 10.0;
        let split = series.split(60, 90).unwrap();
        assert_eq!(
            model,
            WheelResponseModel::fit(&split.training, 0.01, 1e-9).unwrap()
        );
        let free = model.evaluate(&split.test, ResponseMode::FreeRun).unwrap();
        let one = model.evaluate(&split.test, ResponseMode::OneStep).unwrap();
        assert!((free.rmse_rad_s[0].powi(2) * 29.0 - 100.0).abs() < 1e-9);
        assert!((one.rmse_rad_s[0].powi(2) * 29.0 - 149.0).abs() < 1e-9);
    }

    #[test]
    fn bad_clock_rank_and_numerical_overflow_fail_closed() {
        let mut series = read_ddmr_samples(synthetic().as_bytes()).unwrap();
        let split = series.split(60, 90).unwrap();
        assert!(WheelResponseModel::fit(&split.training, 0.02, 1e-9).is_err());
        assert!(WheelResponseModel::fit(&split.training, 0.01, 0.01).is_err());
        let model = WheelResponseModel::fit(&split.training, 0.01, 1e-9).unwrap();
        for row in &mut series.samples[..60] {
            row.recorded_voltage_v = row.recorded_speed_rad_s;
        }
        assert!(
            WheelResponseModel::fit(&series.split(60, 90).unwrap().training, 0.01, 1e-9).is_err()
        );
        series.samples[95].recorded_voltage_v = [f64::MAX; 2];
        assert!(model
            .evaluate(&series.split(60, 90).unwrap().test, ResponseMode::FreeRun)
            .is_err());
    }
}
