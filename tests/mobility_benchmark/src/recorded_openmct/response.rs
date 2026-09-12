//! Fixed first-order PWM-to-speed response diagnostics, not physical identification.

use super::OpenMctSeries;
use anyhow::{ensure, Result};

/// Stable scalar model with an explicit source-unit gain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FirstOrderSpeedModel {
    /// Steady-state speed gain; PWM is a command, not measured voltage.
    pub gain_rpm_per_pwm_count: f64,
    /// Positive time constant in seconds.
    pub time_constant_s: f64,
}

/// One preceding-row input hold and its next-row measurement comparison.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeedResponseTransition {
    /// Zero-based target row; the command and duration come from target minus one.
    pub target_row: usize,
    /// Declared preceding-row interval; not a certified acquisition clock.
    pub interval_s: f64,
    /// Preceding-row held PWM command.
    pub pwm_command: f64,
    /// Target-row measured speed.
    pub measured_speed_rpm: f64,
    /// Prediction propagated without subsequent measurement corrections.
    pub free_running_speed_rpm: f64,
    /// Prediction initialized from the preceding measured speed.
    pub one_step_speed_rpm: f64,
    /// Target minus free-running prediction.
    pub free_running_residual_rpm: f64,
    /// Target minus one-step prediction.
    pub one_step_residual_rpm: f64,
    /// Target minus preceding measurement (persistence baseline).
    pub persistence_residual_rpm: f64,
}

/// Reproducible diagnostics with fixed model, raw-byte identity and all transitions.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeedResponseEvaluation {
    /// Exact source bytes, not an authentication claim.
    pub source_sha256: String,
    /// Supplied model, never fitted or selected by this evaluator.
    pub model: FirstOrderSpeedModel,
    /// First measured speed, used once to initialize the free-running state.
    pub initial_speed_rpm: f64,
    /// All adjacent transitions in source order, including large residuals.
    pub transitions: Vec<SpeedResponseTransition>,
    /// Untrimmed free-running RMSE.
    pub free_running_rmse_rpm: f64,
    /// Untrimmed one-step RMSE.
    pub one_step_rmse_rpm: f64,
    /// Untrimmed persistence RMSE.
    pub persistence_rmse_rpm: f64,
}

/// Maximum row lag accepted by the bounded residual diagnostic.
pub const MAX_SPEED_RESIDUAL_LAG_ROWS: usize = 1024;

/// One descriptive lag statistic with the actual input-age range.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeedResidualLag {
    /// Transition-index lag; zero uses each transition's own preceding command.
    pub lag_rows: usize,
    /// Number of overlapping transition pairs retained at this lag.
    pub pair_count: usize,
    /// Centered one-step residual autocorrelation using full-record energy.
    pub residual_autocorrelation: f64,
    /// Centered residual versus past-PWM correlation using full-record energies.
    pub residual_past_pwm_correlation: f64,
    /// Minimum declared time from the selected PWM row to its target measurement.
    pub input_age_min_s: f64,
    /// Mean declared time from the selected PWM row to its target measurement.
    pub input_age_mean_s: f64,
    /// Maximum declared time from the selected PWM row to its target measurement.
    pub input_age_max_s: f64,
}

/// Descriptive residual correlations; no hypothesis-test threshold is implied.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeedResidualDiagnostics {
    /// Exact evaluated source identity.
    pub source_sha256: String,
    /// Number of response transitions used to compute global means and energies.
    pub transition_count: usize,
    /// Full-record mean of one-step residuals.
    pub residual_mean_rpm: f64,
    /// Full-record mean of preceding-row PWM commands.
    pub pwm_mean_count: f64,
    /// Every requested lag from zero through the inclusive maximum.
    pub lags: Vec<SpeedResidualLag>,
}

/// Diagnose one-step residual memory and correlation with preceding PWM inputs.
///
/// The function does not alter or refit the supplied response evaluation. Means
/// and normalization energies use all transitions. A lagged numerator uses only
/// its overlapping pairs. Input age sums the original declared intervals from
/// the selected past command through the target measurement, so row lag is not
/// mislabeled as fixed time for nonuniform captures. Values are descriptive:
/// this API does not compute confidence intervals, p-values or a pass verdict.
pub fn diagnose_one_step_residual_lags(
    evaluation: &SpeedResponseEvaluation,
    max_lag_rows: usize,
) -> Result<SpeedResidualDiagnostics> {
    let rows = &evaluation.transitions;
    ensure!(rows.len() >= 2, "insufficient residual transitions");
    ensure!(
        max_lag_rows < rows.len() && max_lag_rows <= MAX_SPEED_RESIDUAL_LAG_ROWS,
        "residual lag is out of bounds"
    );
    ensure!(
        rows.iter().all(|row| {
            row.interval_s.is_finite()
                && row.interval_s > 0.0
                && row.pwm_command.is_finite()
                && row.one_step_residual_rpm.is_finite()
        }),
        "invalid residual transition"
    );
    let n = rows.len() as f64;
    let residual_mean_rpm = rows
        .iter()
        .map(|row| row.one_step_residual_rpm)
        .sum::<f64>()
        / n;
    let pwm_mean_count = rows.iter().map(|row| row.pwm_command).sum::<f64>() / n;
    ensure!(
        residual_mean_rpm.is_finite() && pwm_mean_count.is_finite(),
        "nonfinite residual means"
    );
    let residual_energy = rows
        .iter()
        .map(|row| (row.one_step_residual_rpm - residual_mean_rpm).powi(2))
        .sum::<f64>();
    let pwm_energy = rows
        .iter()
        .map(|row| (row.pwm_command - pwm_mean_count).powi(2))
        .sum::<f64>();
    ensure!(
        residual_energy.is_finite()
            && residual_energy > 0.0
            && pwm_energy.is_finite()
            && pwm_energy > 0.0,
        "residual diagnostic needs varying finite residual and PWM signals"
    );
    let cross_scale = (residual_energy * pwm_energy).sqrt();
    ensure!(
        cross_scale.is_finite() && cross_scale > 0.0,
        "nonfinite residual normalization"
    );

    let mut input_ages_s: Vec<f64> = rows.iter().map(|row| row.interval_s).collect();
    let mut lags = Vec::with_capacity(max_lag_rows + 1);
    for lag_rows in 0..=max_lag_rows {
        if lag_rows > 0 {
            for target in lag_rows..rows.len() {
                input_ages_s[target] += rows[target - lag_rows].interval_s;
            }
        }
        let mut residual_product = 0.0;
        let mut input_product = 0.0;
        let mut age_sum_s = 0.0;
        let mut age_min_s = f64::INFINITY;
        let mut age_max_s: f64 = 0.0;
        for target in lag_rows..rows.len() {
            let residual = rows[target].one_step_residual_rpm - residual_mean_rpm;
            residual_product +=
                residual * (rows[target - lag_rows].one_step_residual_rpm - residual_mean_rpm);
            input_product += residual * (rows[target - lag_rows].pwm_command - pwm_mean_count);
            let age_s = input_ages_s[target];
            age_sum_s += age_s;
            age_min_s = age_min_s.min(age_s);
            age_max_s = age_max_s.max(age_s);
        }
        let pair_count = rows.len() - lag_rows;
        let residual_autocorrelation = residual_product / residual_energy;
        let residual_past_pwm_correlation = input_product / cross_scale;
        let input_age_mean_s = age_sum_s / pair_count as f64;
        ensure!(
            [
                residual_product,
                input_product,
                age_sum_s,
                age_min_s,
                age_max_s,
                residual_autocorrelation,
                residual_past_pwm_correlation,
                input_age_mean_s,
            ]
            .iter()
            .all(|value| value.is_finite()),
            "nonfinite residual lag arithmetic"
        );
        lags.push(SpeedResidualLag {
            lag_rows,
            pair_count,
            residual_autocorrelation,
            residual_past_pwm_correlation,
            input_age_min_s: age_min_s,
            input_age_mean_s,
            input_age_max_s: age_max_s,
        });
    }
    Ok(SpeedResidualDiagnostics {
        source_sha256: evaluation.source_sha256.clone(),
        transition_count: rows.len(),
        residual_mean_rpm,
        pwm_mean_count,
        lags,
    })
}

/// Apply exact scalar zero-order-hold propagation to preceding-row PWM and DT.
///
/// This alignment is an explicit assumption, not inferred from fit quality.
/// REF is never substituted for PWM. Nonuniform declared intervals are retained.
/// No clipping, refitting, outlier rejection or physical acceptance is performed.
pub fn evaluate_speed_response(
    source: &OpenMctSeries,
    model: FirstOrderSpeedModel,
) -> Result<SpeedResponseEvaluation> {
    ensure!(
        model.gain_rpm_per_pwm_count.is_finite()
            && model.time_constant_s.is_finite()
            && model.time_constant_s > 0.0,
        "invalid first-order speed model"
    );
    ensure!(source.samples().len() >= 2, "insufficient response rows");
    let initial_speed_rpm = source.samples()[0].measured_speed_rpm;
    let mut state = initial_speed_rpm;
    let mut squared = [0.0; 3];
    let mut transitions = Vec::with_capacity(source.samples().len() - 1);
    for (i, pair) in source.samples().windows(2).enumerate() {
        let interval_s = pair[0].loop_interval_ms / 1000.0;
        ensure!(interval_s > 0.0, "response interval underflow");
        let exponent = -interval_s / model.time_constant_s;
        let a = exponent.exp();
        let b = -exponent.exp_m1() * model.gain_rpm_per_pwm_count;
        let input = b * pair[0].pwm_command;
        state = a * state + input;
        let one_step = a * pair[0].measured_speed_rpm + input;
        let measured = pair[1].measured_speed_rpm;
        let residuals = [
            measured - state,
            measured - one_step,
            measured - pair[0].measured_speed_rpm,
        ];
        ensure!(
            [state, one_step]
                .iter()
                .chain(residuals.iter())
                .all(|v| v.is_finite()),
            "nonfinite response prediction or residual"
        );
        for (sum, residual) in squared.iter_mut().zip(residuals) {
            *sum += residual * residual;
        }
        transitions.push(SpeedResponseTransition {
            target_row: i + 1,
            interval_s,
            pwm_command: pair[0].pwm_command,
            measured_speed_rpm: measured,
            free_running_speed_rpm: state,
            one_step_speed_rpm: one_step,
            free_running_residual_rpm: residuals[0],
            one_step_residual_rpm: residuals[1],
            persistence_residual_rpm: residuals[2],
        });
    }
    let errors = squared.map(|sum| (sum / transitions.len() as f64).sqrt());
    ensure!(
        errors.iter().all(|v| v.is_finite()),
        "nonfinite response RMSE"
    );
    Ok(SpeedResponseEvaluation {
        source_sha256: source.source_sha256().to_owned(),
        model,
        initial_speed_rpm,
        transitions,
        free_running_rmse_rpm: errors[0],
        one_step_rmse_rpm: errors[1],
        persistence_rmse_rpm: errors[2],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_openmct::read_openmct;

    #[test]
    fn residual_lags_preserve_nonuniform_input_age_and_reject_bad_evidence() {
        let text = format!(
            "Mode: Test\nDate: fixture\n{}\n0,0,10,0,0,1,nan,nan,nan,-1\n0,0,20,0,0,0,nan,nan,nan,-1\n0,2,30,0,0,2,nan,nan,nan,-1\n0,1,40,0,0,-1,nan,nan,nan,-1\n0,4,50,0,0,0,nan,nan,nan,-1\n",
            super::super::HEADER
        );
        let source = read_openmct(text.as_bytes()).unwrap();
        let response = evaluate_speed_response(
            &source,
            FirstOrderSpeedModel {
                gain_rpm_per_pwm_count: 0.5,
                time_constant_s: 0.1,
            },
        )
        .unwrap();
        let result = diagnose_one_step_residual_lags(&response, 2).unwrap();
        assert_eq!(
            result,
            diagnose_one_step_residual_lags(&response, 2).unwrap()
        );
        assert_eq!(result.transition_count, 4);
        assert_eq!(
            result
                .lags
                .iter()
                .map(|lag| lag.pair_count)
                .collect::<Vec<_>>(),
            vec![4, 3, 2]
        );
        assert!((result.lags[0].residual_autocorrelation - 1.0).abs() < 1e-15);
        assert!((result.lags[0].input_age_min_s - 0.01).abs() < 1e-15);
        assert!((result.lags[0].input_age_mean_s - 0.025).abs() < 1e-15);
        assert!((result.lags[0].input_age_max_s - 0.04).abs() < 1e-15);
        assert!((result.lags[1].input_age_min_s - 0.03).abs() < 1e-15);
        assert!((result.lags[1].input_age_mean_s - 0.05).abs() < 1e-15);
        assert!((result.lags[1].input_age_max_s - 0.07).abs() < 1e-15);
        assert!(diagnose_one_step_residual_lags(&response, 4).is_err());
        assert!(
            diagnose_one_step_residual_lags(&response, MAX_SPEED_RESIDUAL_LAG_ROWS + 1).is_err()
        );

        let mut invalid = response.clone();
        invalid.transitions[0].interval_s = f64::NAN;
        assert!(diagnose_one_step_residual_lags(&invalid, 0).is_err());
        let mut constant_residual = response.clone();
        for row in &mut constant_residual.transitions {
            row.one_step_residual_rpm = 1.0;
        }
        assert!(diagnose_one_step_residual_lags(&constant_residual, 0).is_err());
        let mut constant_pwm = response;
        for row in &mut constant_pwm.transitions {
            row.pwm_command = 1.0;
        }
        assert!(diagnose_one_step_residual_lags(&constant_pwm, 0).is_err());
    }

    #[test]
    fn tiny_holds_preserve_input_and_invalid_arithmetic_is_rejected() {
        let header = format!("Mode: Test\nDate: fixture\n{}\n", super::super::HEADER);
        let make = |dt: &str, pwm: &str, measured: &str| {
            read_openmct(format!("{header}0,0,{dt},0,0,{pwm},nan,nan,nan,-1\n0,{measured},10,0,0,0,nan,nan,nan,-1\n").as_bytes()).unwrap()
        };
        let model = FirstOrderSpeedModel {
            gain_rpm_per_pwm_count: 1.0,
            time_constant_s: 1.0,
        };
        let tiny = evaluate_speed_response(&make("1e-15", "1", "0"), model).unwrap();
        assert!(tiny.transitions[0].free_running_speed_rpm > 0.0);
        assert!((tiny.transitions[0].free_running_speed_rpm / 1e-18 - 1.0).abs() < 1e-12);
        let settled = evaluate_speed_response(&make("1e308", "-2", "-2"), model).unwrap();
        assert_eq!(settled.transitions[0].free_running_speed_rpm, -2.0);
        assert_eq!(settled.free_running_rmse_rpm, 0.0);
        let normal = make("10", "2", "0");
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(evaluate_speed_response(
                &normal,
                FirstOrderSpeedModel {
                    time_constant_s: value,
                    ..model
                }
            )
            .is_err());
        }
        for value in [f64::NAN, f64::INFINITY] {
            assert!(evaluate_speed_response(
                &normal,
                FirstOrderSpeedModel {
                    gain_rpm_per_pwm_count: value,
                    ..model
                }
            )
            .is_err());
        }
        assert!(evaluate_speed_response(&make("5e-324", "1", "0"), model).is_err());
        assert!(evaluate_speed_response(
            &make("1000", "1e308", "0"),
            FirstOrderSpeedModel {
                gain_rpm_per_pwm_count: 10.0,
                ..model
            }
        )
        .is_err());
        assert!(evaluate_speed_response(&make("10", "0", "1e308"), model).is_err());
        let single =
            read_openmct(format!("{header}0,0,10,0,0,0,nan,nan,nan,-1\n").as_bytes()).unwrap();
        assert!(evaluate_speed_response(&single, model).is_err());
    }

    #[test]
    fn nonuniform_holds_are_causal_and_do_not_correct_free_running_state() {
        let text = format!("Mode: Test\nDate: fixture\n{}\n999,0,1000,0,0,2,nan,nan,nan,-1\n999,1,2000,0,0,0,nan,nan,nan,-1\n999,0.25,10,0,0,99,nan,nan,nan,-1\n", super::super::HEADER);
        let source = read_openmct(text.as_bytes()).unwrap();
        let model = FirstOrderSpeedModel {
            gain_rpm_per_pwm_count: 1.0,
            time_constant_s: 1.0 / 2.0_f64.ln(),
        };
        let result = evaluate_speed_response(&source, model).unwrap();
        assert_eq!(result, evaluate_speed_response(&source, model).unwrap());
        assert!(result.free_running_rmse_rpm < 1e-14);
        assert!(result.one_step_rmse_rpm < 1e-14);
        assert_eq!(result.transitions[1].interval_s, 2.0);
        let altered = read_openmct(text.replace("999,1,2000", "999,100,2000").as_bytes()).unwrap();
        let changed = evaluate_speed_response(&altered, model).unwrap();
        assert_eq!(
            result.transitions[1].free_running_speed_rpm,
            changed.transitions[1].free_running_speed_rpm
        );
        assert!(changed.transitions[1].one_step_residual_rpm.abs() > 20.0);
        assert!(evaluate_speed_response(
            &source,
            FirstOrderSpeedModel {
                time_constant_s: 0.0,
                ..model
            }
        )
        .is_err());
    }
}
