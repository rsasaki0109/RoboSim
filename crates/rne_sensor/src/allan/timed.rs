//! Sample-count-weighted adjacent time windows; not uniform ADEV or calibration.
use super::{AllanError, AllanSample};
use std::collections::VecDeque;

/// Explicit finite evaluation grid in the same tick coordinates as the source.
#[derive(Clone, Copy, Debug)]
pub struct TimeWindowPlan {
    /// Duration of each adjacent half-open window, in ticks.
    pub window_ticks: u64,
    /// First right-hand endpoint; both windows must fit the source time domain.
    pub first_endpoint_ticks: u64,
    /// Positive spacing between evaluation endpoints.
    pub endpoint_period_ticks: u64,
    /// Number of evaluation endpoints, at most one million.
    pub endpoints: u64,
}

/// Complete-source statistic with explicit missing-window accounting.
#[derive(Clone, Debug, PartialEq)]
pub struct TimedDeviation {
    /// Number of source samples validated, including those outside evaluation windows.
    pub source_samples: u64,
    /// Endpoints with measurements in both adjacent windows.
    pub valid_pairs: u64,
    /// Endpoints with at least one empty window; never filled with artificial values.
    pub empty_pairs: u64,
    /// Sum of products of adjacent sample counts, not independent degrees of freedom.
    pub total_weight: u64,
    /// Weighted variance in squared source units.
    pub variance: f64,
    /// Square root of the weighted variance in source units.
    pub deviation: f64,
}

#[derive(Default)]
struct Sum {
    value: f64,
    correction: f64,
}
impl Sum {
    fn add(&mut self, x: f64) -> Result<(), AllanError> {
        let y = x - self.correction;
        let next = self.value + y;
        self.correction = (next - self.value) - y;
        self.value = next;
        if self.value.is_finite() && self.correction.is_finite() {
            Ok(())
        } else {
            Err(AllanError::Arithmetic)
        }
    }
}

/// Analyze an ordered fallible stream without interpolation, sorting or truncation.
///
/// At endpoint t, compare sample means in [t-2*tau,t-tau) and [t-tau,t),
/// weighted by the product of their counts (FAVAR paper equations 4-5).
/// This is not a time-weighted integral or a fitted noise density. Validate the
/// entire stream before returning; even a malformed trailing record fails.
/// Source errors may be mapped to `AllanError` by the caller. No partial result
/// is returned. Bounds: 20 million source samples, one million endpoints and
/// one million retained samples. Exceeding a bound fails rather than thinning.
/// Runtime is O(samples + endpoints); memory depends on retained window samples.
pub fn weighted_time_deviation(
    samples: impl IntoIterator<Item = Result<AllanSample, AllanError>>,
    plan: TimeWindowPlan,
) -> Result<TimedDeviation, AllanError> {
    if plan.window_ticks == 0 || plan.endpoint_period_ticks == 0 {
        return Err(AllanError::Period);
    }
    if plan.endpoints == 0 || plan.endpoints > 1_000_000 {
        return Err(AllanError::Size);
    }
    let span = plan
        .window_ticks
        .checked_mul(2)
        .ok_or(AllanError::Arithmetic)?;
    let start = plan
        .first_endpoint_ticks
        .checked_sub(span)
        .ok_or(AllanError::Arithmetic)?;
    let last_endpoint = plan
        .endpoint_period_ticks
        .checked_mul(plan.endpoints - 1)
        .and_then(|v| plan.first_endpoint_ticks.checked_add(v))
        .ok_or(AllanError::Arithmetic)?;
    let mut left: VecDeque<AllanSample> = VecDeque::new();
    let mut right: VecDeque<AllanSample> = VecDeque::new();
    let (mut ls, mut rs, mut squares) = (Sum::default(), Sum::default(), Sum::default());
    let mut result = TimedDeviation {
        source_samples: 0,
        valid_pairs: 0,
        empty_pairs: 0,
        total_weight: 0,
        variance: 0.0,
        deviation: 0.0,
    };
    let mut previous = None;
    let mut offset = None;
    let mut evaluated = 0;
    for sample in samples {
        let mut sample = sample?;
        if result.source_samples >= 20_000_000 {
            return Err(AllanError::Size);
        }
        if !sample.value.is_finite() {
            return Err(AllanError::Nonfinite(result.source_samples as usize));
        }
        if previous.is_some_and(|t| sample.capture_ticks <= t)
            || (previous.is_none() && sample.capture_ticks > start)
        {
            return Err(AllanError::CaptureTime(result.source_samples as usize));
        }
        previous = Some(sample.capture_ticks);
        result.source_samples += 1;
        while evaluated < plan.endpoints {
            let t = plan.first_endpoint_ticks + evaluated * plan.endpoint_period_ticks;
            if t > sample.capture_ticks {
                break;
            }
            while right
                .front()
                .is_some_and(|v| v.capture_ticks < t - plan.window_ticks)
            {
                let v = right.pop_front().unwrap();
                rs.add(-v.value)?;
                ls.add(v.value)?;
                left.push_back(v);
            }
            while left.front().is_some_and(|v| v.capture_ticks < t - span) {
                ls.add(-left.pop_front().unwrap().value)?;
            }
            // Reset cancellation residue once a window becomes truly empty.
            if left.is_empty() {
                ls = Sum::default();
            }
            if right.is_empty() {
                rs = Sum::default();
            }
            let weight = (left.len() as u64) * (right.len() as u64);
            if weight == 0 {
                result.empty_pairs += 1;
            } else {
                let d = rs.value / right.len() as f64 - ls.value / left.len() as f64;
                squares.add(weight as f64 * d * d)?;
                result.total_weight = result
                    .total_weight
                    .checked_add(weight)
                    .ok_or(AllanError::Arithmetic)?;
                result.valid_pairs += 1;
            }
            evaluated += 1;
        }
        if evaluated < plan.endpoints && sample.capture_ticks >= start {
            if left.len() + right.len() >= 1_000_000 {
                return Err(AllanError::Size);
            }
            sample.value -= *offset.get_or_insert(sample.value);
            rs.add(sample.value)?;
            right.push_back(sample);
        }
    }
    if previous.is_none_or(|t| t < last_endpoint) || evaluated != plan.endpoints {
        return Err(AllanError::CaptureTime(result.source_samples as usize));
    }
    if result.total_weight == 0 {
        return Err(AllanError::Size);
    }
    result.variance = squares.value / (2.0 * result.total_weight as f64);
    if !result.variance.is_finite() || result.variance < 0.0 {
        return Err(AllanError::Arithmetic);
    }
    result.deviation = result.variance.sqrt();
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn uniform_bridge_and_irregular_empty_windows() {
        for n in 2..50 {
            for m in 1..=n / 2 {
                let values: Vec<_> = (0..n)
                    .map(|i| AllanSample {
                        capture_ticks: i as u64,
                        value: ((i * 13) % 19) as f64,
                    })
                    .collect();
                let expected = crate::allan::overlapping_allan_deviation(
                    &values,
                    rne_core::SimDuration::from_ticks(1),
                    &[m],
                )
                .unwrap();
                let mut stream = values;
                // Endpoint coverage witness is outside all half-open evaluation windows.
                stream.push(AllanSample {
                    capture_ticks: n as u64,
                    value: 999.0,
                });
                let got = weighted_time_deviation(
                    stream.into_iter().map(Ok),
                    TimeWindowPlan {
                        window_ticks: m as u64,
                        first_endpoint_ticks: 2 * m as u64,
                        endpoint_period_ticks: 1,
                        endpoints: (n - 2 * m + 1) as u64,
                    },
                )
                .unwrap();
                assert!((got.variance - expected[0].variance).abs() < 1e-11);
                assert_eq!(got.valid_pairs, expected[0].pair_count as u64);
            }
        }
        let samples = [0, 1, 3, 4, 8, 9, 10].map(|t| {
            Ok(AllanSample {
                capture_ticks: t,
                value: 2.0 * t as f64,
            })
        });
        let got = weighted_time_deviation(
            samples,
            TimeWindowPlan {
                window_ticks: 2,
                first_endpoint_ticks: 4,
                endpoint_period_ticks: 1,
                endpoints: 7,
            },
        )
        .unwrap();
        assert_eq!(
            (got.valid_pairs, got.empty_pairs, got.total_weight),
            (3, 4, 5)
        );
        assert_eq!(got.variance, 10.4);
    }
    #[test]
    fn irregular_direct_oracle_handles_offsets_and_recovery_after_gaps() {
        for offset in [0.0, 9.81, 1e9] {
            let samples: Vec<_> = (0..=600)
                .filter(|i| !(150..300).contains(i) && (i % 7 != 3))
                .map(|i| AllanSample {
                    capture_ticks: i * 3 + (i % 3),
                    value: offset + ((i * 71 % 101) as f64 - 50.0) * 0.0001,
                })
                .collect();
            for width in [1, 5, 31, 101] {
                let plan = TimeWindowPlan {
                    window_ticks: width,
                    first_endpoint_ticks: 2 * width,
                    endpoint_period_ticks: 7,
                    endpoints: (1800 - 2 * width) / 7 + 1,
                };
                let got = weighted_time_deviation(samples.iter().copied().map(Ok), plan).unwrap();
                let (mut q, mut weight, mut valid, mut empty) = (0.0, 0_u64, 0, 0);
                for i in 0..plan.endpoints {
                    let t = plan.first_endpoint_ticks + i * plan.endpoint_period_ticks;
                    let mean = |start, end| {
                        let values: Vec<_> = samples
                            .iter()
                            .filter(|s| s.capture_ticks >= start && s.capture_ticks < end)
                            .map(|s| s.value - samples[0].value)
                            .collect();
                        (
                            values.iter().sum::<f64>() / values.len().max(1) as f64,
                            values.len() as u64,
                        )
                    };
                    let (a, na) = mean(t - 2 * width, t - width);
                    let (b, nb) = mean(t - width, t);
                    if na * nb == 0 {
                        empty += 1;
                    } else {
                        valid += 1;
                        weight += na * nb;
                        q += (na * nb) as f64 * (a - b).powi(2);
                    }
                }
                assert_eq!(
                    (got.valid_pairs, got.empty_pairs, got.total_weight),
                    (valid, empty, weight)
                );
                let expected = q / (2.0 * weight as f64);
                assert!((got.variance - expected).abs() <= 1e-12 * expected.max(1e-10));
                assert!(empty > 0 && valid > 0);
            }
        }
    }

    #[test]
    fn invalid_grids_overflow_and_resource_limits_fail_closed() {
        let plan = TimeWindowPlan {
            window_ticks: 1,
            first_endpoint_ticks: 2,
            endpoint_period_ticks: 1,
            endpoints: 1,
        };
        for bad in [
            TimeWindowPlan {
                window_ticks: 0,
                ..plan
            },
            TimeWindowPlan {
                window_ticks: u64::MAX,
                ..plan
            },
            TimeWindowPlan {
                first_endpoint_ticks: 1,
                ..plan
            },
            TimeWindowPlan {
                endpoint_period_ticks: 0,
                ..plan
            },
            TimeWindowPlan {
                endpoints: 0,
                ..plan
            },
            TimeWindowPlan {
                endpoints: 1_000_001,
                ..plan
            },
            TimeWindowPlan {
                first_endpoint_ticks: u64::MAX,
                endpoints: 2,
                ..plan
            },
        ] {
            assert!(weighted_time_deviation(std::iter::empty(), bad).is_err());
        }
        let make = |t| {
            Ok(AllanSample {
                capture_ticks: t,
                value: 1.0,
            })
        };
        assert!(weighted_time_deviation([make(0), make(2)], plan).is_err());
        assert!(weighted_time_deviation([make(1), make(2)], plan).is_err());
        assert!(weighted_time_deviation(
            [
                make(0),
                make(1),
                make(2),
                Ok(AllanSample {
                    capture_ticks: 3,
                    value: f64::NAN
                })
            ],
            plan
        )
        .is_err());
        assert_eq!(
            weighted_time_deviation(
                [
                    Ok(AllanSample {
                        capture_ticks: 0,
                        value: f64::MAX
                    }),
                    Ok(AllanSample {
                        capture_ticks: 1,
                        value: -f64::MAX
                    }),
                    make(2)
                ],
                plan
            ),
            Err(AllanError::Arithmetic)
        );
        let large = TimeWindowPlan {
            window_ticks: 1_000_001,
            first_endpoint_ticks: 2_000_002,
            ..plan
        };
        assert_eq!(
            weighted_time_deviation((0..=1_000_000).map(make), large),
            Err(AllanError::Size)
        );
        // Trailing records are validated even after the sole endpoint was evaluated.
        assert_eq!(
            weighted_time_deviation((0..=20_000_000).map(make), plan),
            Err(AllanError::Size)
        );
    }

    #[test]
    fn trailing_errors_and_insufficient_coverage_fail() {
        let p = TimeWindowPlan {
            window_ticks: 1,
            first_endpoint_ticks: 2,
            endpoint_period_ticks: 1,
            endpoints: 1,
        };
        let samples: Vec<_> = (0..3)
            .map(|t| {
                Ok(AllanSample {
                    capture_ticks: t,
                    value: t as f64,
                })
            })
            .collect();
        assert!(weighted_time_deviation(samples[..2].iter().cloned(), p).is_err());
        let mut broken = samples.clone();
        broken.push(Err(AllanError::Arithmetic));
        assert_eq!(
            weighted_time_deviation(broken, p),
            Err(AllanError::Arithmetic)
        );
        let mut backwards = samples;
        backwards.push(Ok(AllanSample {
            capture_ticks: 1,
            value: 0.0,
        }));
        assert!(weighted_time_deviation(backwards, p).is_err());
    }
}
