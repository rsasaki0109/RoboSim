//! Bounded overlapping Allan deviation for uniformly captured scalar measurements.
//!
//! This computes statistics, not calibration or confidence bounds. It neither
//! resamples gaps nor drops nonfinite records. Use capture times, not arrival times.
//! For N rate samples and averaging factor m, there are N - 2m + 1 overlapping
//! pairs of adjacent m-sample means. These pairs are not statistically independent.

use rne_core::SimDuration;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod feedback;
pub mod timed;

/// Maximum scalar input records accepted by one analysis.
pub const MAX_ALLAN_SAMPLES: usize = 1_000_000;
/// Maximum requested averaging factors, bounding work to O(samples * factors).
pub const MAX_ALLAN_FACTORS: usize = 64;

/// One scalar observation with its actual capture time in simulation ticks.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AllanSample {
    /// Actual capture time, not scheduled time or availability time.
    pub capture_ticks: u64,
    /// Scalar measurement, in a consistent caller-declared unit such as rad/s.
    pub value: f64,
}

/// One overlapping Allan-deviation point, without an inferred confidence interval.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AllanPoint {
    /// Number of samples in each adjacent mean.
    pub averaging_samples: usize,
    /// Averaging time in simulation ticks.
    pub averaging_ticks: u64,
    /// Number of overlapping pairs contributing to this point, not degrees of freedom.
    pub pair_count: usize,
    /// Allan variance, in the square of the input unit.
    pub variance: f64,
    /// Allan deviation, in the input unit.
    pub deviation: f64,
}

/// Input or arithmetic failure; no partial result is returned.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AllanError {
    /// Too few/too many samples, or too few/too many averaging factors.
    #[error("Allan analysis sample/factor count is outside its bounds")]
    Size,
    /// A zero sample period is not meaningful.
    #[error("Allan analysis requires a positive capture period")]
    Period,
    /// Factors must be positive, strictly increasing and fit two adjacent means.
    #[error("invalid Allan averaging factor at index {0}")]
    Factor(usize),
    /// A nonfinite scalar cannot be silently trimmed or replaced.
    #[error("nonfinite Allan sample at index {0}")]
    Nonfinite(usize),
    /// Capture times must be exactly spaced by the supplied period without overflow.
    #[error("nonuniform or overflowing Allan capture time at index {0}")]
    CaptureTime(usize),
    /// Finite source values can still overflow during analysis.
    #[error("nonfinite Allan arithmetic")]
    Arithmetic,
}

/// Compute half the mean squared difference of adjacent m-sample averages.
///
/// Requested factors are preserved exactly: no rounding, automatic factor removal,
/// gap interpolation, detrending or mean-frequency normalization is performed.
/// Subtraction of the first value is solely an offset-invariant numerical centering
/// step; compensated prefix sums reduce accumulation error. Both means stay wholly
/// within the supplied series. Even a one-pair result is labelled with its count,
/// not certified as sufficient calibration evidence. Memory is O(samples).
pub fn overlapping_allan_deviation(
    samples: &[AllanSample],
    sample_period: SimDuration,
    averaging_factors: &[usize],
) -> Result<Vec<AllanPoint>, AllanError> {
    if !(2..=MAX_ALLAN_SAMPLES).contains(&samples.len())
        || !(1..=MAX_ALLAN_FACTORS).contains(&averaging_factors.len())
    {
        return Err(AllanError::Size);
    }
    let period = sample_period.ticks();
    if period == 0 {
        return Err(AllanError::Period);
    }
    for (index, &m) in averaging_factors.iter().enumerate() {
        if m == 0
            || m > samples.len() / 2
            || (index > 0 && m <= averaging_factors[index - 1])
            || period.checked_mul(m as u64).is_none()
        {
            return Err(AllanError::Factor(index));
        }
    }
    for (index, sample) in samples.iter().enumerate() {
        if !sample.value.is_finite() {
            return Err(AllanError::Nonfinite(index));
        }
        if index > 0
            && samples[index - 1].capture_ticks.checked_add(period) != Some(sample.capture_ticks)
        {
            return Err(AllanError::CaptureTime(index));
        }
    }
    let mut prefix = Vec::with_capacity(samples.len() + 1);
    prefix.push(0.0);
    let (mut sum, mut correction) = (0.0, 0.0);
    for sample in samples {
        let centered = sample.value - samples[0].value;
        let adjusted = centered - correction;
        let next = sum + adjusted;
        correction = (next - sum) - adjusted;
        sum = next;
        if !sum.is_finite() || !correction.is_finite() {
            return Err(AllanError::Arithmetic);
        }
        prefix.push(sum);
    }
    let mut points = Vec::with_capacity(averaging_factors.len());
    for &m in averaging_factors {
        let count = samples.len() - 2 * m + 1;
        let (mut squared_sum, mut correction) = (0.0, 0.0);
        for start in 0..count {
            let first = (prefix[start + m] - prefix[start]) / m as f64;
            let second = (prefix[start + 2 * m] - prefix[start + m]) / m as f64;
            let difference = second - first;
            let adjusted = difference * difference - correction;
            let next = squared_sum + adjusted;
            correction = (next - squared_sum) - adjusted;
            squared_sum = next;
        }
        let variance = squared_sum / (2.0 * count as f64);
        if !variance.is_finite() || variance < 0.0 {
            return Err(AllanError::Arithmetic);
        }
        points.push(AllanPoint {
            averaging_samples: m,
            averaging_ticks: period * m as u64,
            pair_count: count,
            variance,
            deviation: variance.sqrt(),
        });
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn samples(values: &[f64]) -> Vec<AllanSample> {
        values
            .iter()
            .enumerate()
            .map(|(i, &value)| AllanSample {
                capture_ticks: 100 + i as u64 * 10,
                value,
            })
            .collect()
    }
    fn analyze(values: &[f64], factors: &[usize]) -> Vec<AllanPoint> {
        overlapping_allan_deviation(&samples(values), SimDuration::from_ticks(10), factors).unwrap()
    }
    #[test]
    fn hand_computed_constant_ramp_and_alternating_samples() {
        assert!(analyze(&[42.0; 8], &[1, 2, 4])
            .iter()
            .all(|p| p.variance == 0.0));
        let ramp = analyze(&[0., 2., 4., 6., 8., 10., 12., 14.], &[1, 2, 3, 4]);
        assert_eq!(
            ramp.iter().map(|p| p.variance).collect::<Vec<_>>(),
            [2., 8., 18., 32.]
        );
        assert_eq!(
            ramp.iter().map(|p| p.pair_count).collect::<Vec<_>>(),
            [7, 5, 3, 1]
        );
        assert_eq!(ramp[3].averaging_ticks, 40);
        let alternating = analyze(&[1., -1., 1., -1., 1., -1., 1., -1.], &[1, 2, 4]);
        assert_eq!(
            alternating.iter().map(|p| p.variance).collect::<Vec<_>>(),
            [2., 0., 0.]
        );
    }
    #[test]
    fn direct_mean_oracle_and_offset_scale_invariance() {
        let values = [3., -2., 7., 1., 9., -5., 4., 11., 2.];
        let actual = analyze(&values, &[1, 2, 3, 4]);
        for point in &actual {
            let m = point.averaging_samples;
            let oracle = values
                .windows(2 * m)
                .map(|window| {
                    let diff = (window[m..].iter().sum::<f64>() - window[..m].iter().sum::<f64>())
                        / m as f64;
                    diff * diff
                })
                .sum::<f64>()
                / (2.0 * point.pair_count as f64);
            assert!((point.variance - oracle).abs() < 1e-12);
        }
        let shifted: Vec<_> = values.iter().map(|v| v + 1e12).collect();
        assert_eq!(actual, analyze(&shifted, &[1, 2, 3, 4]));
        let scaled: Vec<_> = values.iter().map(|v| v * 2.0).collect();
        for (a, b) in actual.iter().zip(analyze(&scaled, &[1, 2, 3, 4])) {
            assert!((b.variance - 4.0 * a.variance).abs() < 1e-12);
        }
    }
    #[test]
    fn invalid_inputs_are_not_trimmed_or_resampled() {
        let period = SimDuration::from_ticks(10);
        let data = samples(&[0., 1., 2., 3.]);
        for factors in [
            vec![],
            vec![0],
            vec![3],
            vec![2, 1],
            vec![1, 1],
            vec![usize::MAX],
            vec![1; 65],
        ] {
            assert!(overlapping_allan_deviation(&data, period, &factors).is_err());
        }
        assert_eq!(
            overlapping_allan_deviation(&data, SimDuration::ZERO, &[1]),
            Err(AllanError::Period)
        );
        for ticks in [110, 121, 130, u64::MAX] {
            let mut changed = data.clone();
            changed[2].capture_ticks = ticks;
            assert!(overlapping_allan_deviation(&changed, period, &[1]).is_err());
        }
        let mut changed = data.clone();
        changed[2].value = f64::NAN;
        assert_eq!(
            overlapping_allan_deviation(&changed, period, &[1]),
            Err(AllanError::Nonfinite(2))
        );
        assert!(
            overlapping_allan_deviation(&samples(&[f64::MAX, -f64::MAX]), period, &[1]).is_err()
        );
        assert!(overlapping_allan_deviation(&data[..1], period, &[1]).is_err());
        assert!(
            overlapping_allan_deviation(&vec![data[0]; MAX_ALLAN_SAMPLES + 1], period, &[1])
                .is_err()
        );
    }
}
