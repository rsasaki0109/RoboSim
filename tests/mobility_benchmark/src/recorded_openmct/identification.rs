//! Training-only empirical PWM response identification, not motor constants.

use super::{response::FirstOrderSpeedModel, OpenMctSeries};
use anyhow::{ensure, Result};

/// Unclamped zero-offset regression `speed[k+1] = a*speed[k] + b*PWM[k]`.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeedIdentification {
    /// Exact training source identity.
    pub training_sha256: String,
    /// All adjacent training transitions used by the regression.
    pub training_transitions: usize,
    /// Uniform declared training interval, not certified hardware timing.
    pub interval_s: f64,
    /// Unclamped dimensionless discrete pole.
    pub pole: f64,
    /// Discrete input gain in RPM per PWM count.
    pub input_gain_rpm_per_pwm_count: f64,
    /// Normalized regressor determinant; numerical excitation, not physical proof.
    pub normalized_determinant: f64,
}

impl SpeedIdentification {
    /// Convert only a strictly positive stable pole to a scalar continuous model.
    ///
    /// Negative, zero, unit and unstable poles remain in this report but cannot
    /// represent a stable real first-order continuous pole. No clamp is applied.
    pub fn continuous_model(&self) -> Result<FirstOrderSpeedModel> {
        ensure!(
            self.pole > 0.0 && self.pole < 1.0,
            "pole has no stable scalar continuous realization"
        );
        ensure!(
            self.interval_s.is_finite() && self.interval_s > 0.0,
            "invalid model interval"
        );
        let time_constant_s = -self.interval_s / self.pole.ln();
        let gain_rpm_per_pwm_count = self.input_gain_rpm_per_pwm_count / (1.0 - self.pole);
        ensure!(
            time_constant_s.is_finite()
                && time_constant_s > 0.0
                && gain_rpm_per_pwm_count.is_finite(),
            "nonfinite continuous conversion"
        );
        Ok(FirstOrderSpeedModel {
            gain_rpm_per_pwm_count,
            time_constant_s,
        })
    }
}

/// Fit all training transitions with a fixed zero-offset, preceding-input OLS model.
///
/// Requires exactly uniform declared intervals. No resampling, delay search,
/// centering, clipping or residual selection occurs. The normalized determinant
/// must exceed 1e-10. Measured-speed noise may bias this empirical regression;
/// numerical identifiability does not establish physical identifiability.
pub fn identify_speed(training: &OpenMctSeries) -> Result<SpeedIdentification> {
    let rows = training.samples();
    ensure!(rows.len() >= 4, "need at least three training transitions");
    let dt_ms = rows[0].loop_interval_ms;
    let interval_s = dt_ms / 1000.0;
    ensure!(interval_s > 0.0, "training interval underflow");
    let (mut xx, mut uu, mut xu, mut xy, mut uy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for pair in rows.windows(2) {
        ensure!(
            pair[0].loop_interval_ms == dt_ms,
            "nonuniform training clock"
        );
        let x = pair[0].measured_speed_rpm;
        let u = pair[0].pwm_command;
        let y = pair[1].measured_speed_rpm;
        xx += x * x;
        uu += u * u;
        xu += x * u;
        xy += x * y;
        uy += u * y;
    }
    ensure!(
        [xx, uu, xu, xy, uy].iter().all(|v| v.is_finite()),
        "nonfinite regression arithmetic"
    );
    ensure!(xx > 0.0 && uu > 0.0, "unexcited training regressors");
    let correlation = (xu / xx.sqrt()) / uu.sqrt();
    let normalized_determinant = 1.0 - correlation * correlation;
    ensure!(
        normalized_determinant.is_finite() && normalized_determinant > 1e-10,
        "rank deficient training regressors"
    );
    let pole = (xy / xx - (xu / xx) * (uy / uu)) / normalized_determinant;
    let input_gain_rpm_per_pwm_count = (uy / uu - (xu / uu) * (xy / xx)) / normalized_determinant;
    ensure!(
        pole.is_finite() && input_gain_rpm_per_pwm_count.is_finite(),
        "nonfinite fitted coefficients"
    );
    Ok(SpeedIdentification {
        training_sha256: training.source_sha256().to_owned(),
        training_transitions: rows.len() - 1,
        interval_s,
        pole,
        input_gain_rpm_per_pwm_count,
        normalized_determinant,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recorded_openmct::{read_openmct, HEADER};

    #[test]
    fn degenerate_arithmetic_and_unrealizable_poles_are_rejected() {
        let read = |rows: &str| {
            read_openmct(format!("Mode: fixture\nDate: synthetic\n{HEADER}\n{rows}").as_bytes())
                .unwrap()
        };
        let row = |x: &str, u: &str| format!("0,{x},10,0,0,{u},nan,nan,nan,-1\n");
        assert!(identify_speed(&read(&row("1", "1").repeat(4))).is_err());
        assert!(identify_speed(&read(&row("1e308", "1").repeat(4))).is_err());
        assert!(identify_speed(&read(&row("1", "2").repeat(3))).is_err());
        let mut fit = SpeedIdentification {
            training_sha256: "synthetic".into(),
            training_transitions: 4,
            interval_s: 0.01,
            pole: 0.5,
            input_gain_rpm_per_pwm_count: 0.25,
            normalized_determinant: 1.0,
        };
        let model = fit.continuous_model().unwrap();
        assert!(((-0.02 / model.time_constant_s).exp() - 0.25).abs() < 1e-14);
        for pole in [-0.5, 0.0, 1.0, 1.5, f64::NAN, f64::INFINITY] {
            fit.pole = pole;
            assert!(fit.continuous_model().is_err());
        }
        fit.pole = 0.5;
        fit.input_gain_rpm_per_pwm_count = f64::MAX;
        assert!(fit.continuous_model().is_err());
        fit.input_gain_rpm_per_pwm_count = 0.25;
        for dt in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            fit.interval_s = dt;
            assert!(fit.continuous_model().is_err());
        }
    }

    #[test]
    fn recovers_known_model_and_rejects_bad_training_without_clamping() {
        let fixture = |a: f64| {
            let mut text = format!("Mode: fixture\nDate: synthetic\n{HEADER}\n");
            let mut speed = 0.0;
            for u in [2.0, 0.0, -1.0, 3.0, 0.0] {
                text.push_str(&format!("999,{speed},10,0,0,{u},nan,nan,nan,-1\n"));
                speed = a * speed + 0.25 * u;
            }
            text
        };
        let text = fixture(0.5);
        let source = read_openmct(text.as_bytes()).unwrap();
        let fit = identify_speed(&source).unwrap();
        assert_eq!(fit, identify_speed(&source).unwrap());
        assert_eq!(fit.training_transitions, 4);
        assert!((fit.pole - 0.5).abs() < 1e-12);
        assert!((fit.input_gain_rpm_per_pwm_count - 0.25).abs() < 1e-12);
        assert!((fit.continuous_model().unwrap().gain_rpm_per_pwm_count - 0.5).abs() < 1e-12);
        let unstable = identify_speed(&read_openmct(fixture(1.5).as_bytes()).unwrap()).unwrap();
        assert!(unstable.pole > 1.0);
        assert!(unstable.continuous_model().is_err());
        let irregular = text.replacen(",10,", ",20,", 1);
        assert!(identify_speed(&read_openmct(irregular.as_bytes()).unwrap()).is_err());
        let flat = format!(
            "Mode: fixture\nDate: synthetic\n{HEADER}\n{}",
            "0,0,10,0,0,0,nan,nan,nan,-1\n".repeat(4)
        );
        assert!(identify_speed(&read_openmct(flat.as_bytes()).unwrap()).is_err());
    }
}
