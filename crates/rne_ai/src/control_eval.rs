//! Controller evaluation metrics over recorded tracking runs.
//!
//! A controller cannot be judged from a single pass/fail bit. This module computes the
//! standard quantities a control engineer reads off a tracking run — error statistics,
//! settling, overshoot, effort, smoothness, saturation exposure — from a plain record
//! of samples, and aggregates them across seeds so a claim like "controller A beats
//! controller B" carries a mean and a spread instead of an anecdote.
//!
//! The input is deliberately dumb data ([`ControlTrackingSample`]), not a trait over
//! environments: any loop that can log time, tracking error, and its command can be
//! evaluated, whether it runs the kinematic plant, the dynamic plant, or hardware
//! telemetry read back from a log.
//!
//! Everything here is deterministic arithmetic; aggregation sorts by seed so reports
//! are byte-stable regardless of evaluation order.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// One logged step of a closed-loop tracking run.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ControlTrackingSample {
    /// Simulation time of the sample in seconds.
    pub time_s: f64,
    /// Absolute tracking error in meters (cross-track for path following).
    pub tracking_error_m: f64,
    /// Controller command at this step, in the controller's own unit.
    pub command: f64,
    /// Whether any actuator or tire was saturated during this step.
    pub saturated: bool,
    /// Whether a hard constraint (course bound, collision, signal) was violated.
    pub violation: bool,
}

/// Metrics computed from one tracking run.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ControlMetrics {
    /// Root-mean-square tracking error in meters.
    pub rms_error_m: f64,
    /// Largest tracking error in meters.
    pub max_error_m: f64,
    /// Mean tracking error over the settled tail in meters; steady-state error.
    pub steady_state_error_m: f64,
    /// First time after which the error stays inside the settling band, in seconds.
    ///
    /// `None` when the run never settles, which is itself a result.
    pub settling_time_s: Option<f64>,
    /// Largest error after first entering the settling band, in meters; overshoot.
    pub overshoot_m: f64,
    /// Time integral of the command magnitude; control effort.
    pub effort: f64,
    /// Time integral of the command rate magnitude; control smoothness (lower is smoother).
    pub smoothness: f64,
    /// Fraction of steps spent with a saturated actuator or tire, in `[0, 1]`.
    pub saturated_fraction: f64,
    /// Number of steps that violated a hard constraint.
    pub violation_count: usize,
    /// Duration covered by the samples in seconds.
    pub duration_s: f64,
}

impl ControlMetrics {
    /// Computes metrics from a run of samples ordered by time.
    ///
    /// `settling_band_m` defines both settling and overshoot: settling time is the
    /// first instant after which the error never leaves the band again, and overshoot
    /// is the largest error seen after the error first enters the band.
    pub fn from_samples(samples: &[ControlTrackingSample], settling_band_m: f64) -> Option<Self> {
        if samples.len() < 2 {
            return None;
        }
        let band = settling_band_m.max(0.0);

        let mut squared_sum = 0.0;
        let mut max_error = 0.0_f64;
        let mut effort = 0.0;
        let mut smoothness = 0.0;
        let mut saturated_steps = 0_usize;
        let mut violation_count = 0_usize;

        for (index, sample) in samples.iter().enumerate() {
            squared_sum += sample.tracking_error_m * sample.tracking_error_m;
            max_error = max_error.max(sample.tracking_error_m.abs());
            if sample.saturated {
                saturated_steps += 1;
            }
            if sample.violation {
                violation_count += 1;
            }
            if index > 0 {
                let dt = (sample.time_s - samples[index - 1].time_s).max(0.0);
                effort += sample.command.abs() * dt;
                if dt > 0.0 {
                    smoothness += (sample.command - samples[index - 1].command).abs();
                }
            }
        }

        // Settling: walk backward from the end to find the first index after which the
        // error never leaves the band again.
        let mut settled_from = None;
        for (index, sample) in samples.iter().enumerate().rev() {
            if sample.tracking_error_m.abs() > band {
                break;
            }
            settled_from = Some(index);
        }
        let settling_time_s = settled_from
            .filter(|index| *index > 0)
            .map(|index| samples[index].time_s - samples[0].time_s)
            .or(if settled_from == Some(0) {
                Some(0.0)
            } else {
                None
            });

        // Overshoot: the worst error after the band is first entered.
        let first_entry = samples
            .iter()
            .position(|sample| sample.tracking_error_m.abs() <= band);
        let overshoot_m = first_entry
            .map(|index| {
                samples[index..]
                    .iter()
                    .map(|sample| sample.tracking_error_m.abs())
                    .fold(0.0_f64, f64::max)
            })
            .unwrap_or(max_error);

        // Steady-state error: mean over the settled tail, or the final quarter when
        // the run never settles.
        let tail_start = settled_from.unwrap_or(samples.len() - samples.len() / 4 - 1);
        let tail = &samples[tail_start..];
        let steady_state_error_m = tail
            .iter()
            .map(|sample| sample.tracking_error_m.abs())
            .sum::<f64>()
            / tail.len() as f64;

        Some(Self {
            rms_error_m: (squared_sum / samples.len() as f64).sqrt(),
            max_error_m: max_error,
            steady_state_error_m,
            settling_time_s,
            overshoot_m,
            effort,
            smoothness,
            saturated_fraction: saturated_steps as f64 / samples.len() as f64,
            violation_count,
            duration_s: samples.last().unwrap().time_s - samples[0].time_s,
        })
    }
}

/// Mean and spread of one metric across seeds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MetricSpread {
    /// Mean across seeds.
    pub mean: f64,
    /// Population standard deviation across seeds.
    pub stddev: f64,
    /// Smallest value across seeds.
    pub min: f64,
    /// Largest value across seeds.
    pub max: f64,
}

impl MetricSpread {
    fn from_values(values: &[f64]) -> Self {
        let count = values.len().max(1) as f64;
        let mean = values.iter().sum::<f64>() / count;
        let variance = values
            .iter()
            .map(|value| (value - mean) * (value - mean))
            .sum::<f64>()
            / count;
        Self {
            mean,
            stddev: variance.sqrt(),
            min: values.iter().copied().fold(f64::INFINITY, f64::min),
            max: values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        }
    }
}

/// Multi-seed evaluation report for one controller on one scenario.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ControlEvalReport {
    /// Report payload schema version.
    pub schema_version: u32,
    /// Scenario name for provenance.
    pub scenario: String,
    /// Controller name for provenance.
    pub controller: String,
    /// Per-seed metrics in ascending seed order.
    pub seeds: BTreeMap<u64, ControlMetrics>,
    /// RMS tracking error across seeds.
    pub rms_error_m: MetricSpread,
    /// Maximum tracking error across seeds.
    pub max_error_m: MetricSpread,
    /// Control effort across seeds.
    pub effort: MetricSpread,
    /// Saturated fraction across seeds.
    pub saturated_fraction: MetricSpread,
    /// Total constraint violations across every seed.
    pub total_violations: usize,
    /// Number of seeds whose runs never settled.
    pub unsettled_seeds: usize,
}

impl ControlEvalReport {
    /// Current schema version of the serialized report.
    pub const SCHEMA_VERSION: u32 = 1;

    /// Aggregates per-seed metrics into a report.
    pub fn from_seed_metrics(
        scenario: impl Into<String>,
        controller: impl Into<String>,
        seeds: BTreeMap<u64, ControlMetrics>,
    ) -> Self {
        let collect = |extract: fn(&ControlMetrics) -> f64| {
            MetricSpread::from_values(&seeds.values().map(extract).collect::<Vec<_>>())
        };
        Self {
            schema_version: Self::SCHEMA_VERSION,
            scenario: scenario.into(),
            controller: controller.into(),
            rms_error_m: collect(|metrics| metrics.rms_error_m),
            max_error_m: collect(|metrics| metrics.max_error_m),
            effort: collect(|metrics| metrics.effort),
            saturated_fraction: collect(|metrics| metrics.saturated_fraction),
            total_violations: seeds.values().map(|metrics| metrics.violation_count).sum(),
            unsettled_seeds: seeds
                .values()
                .filter(|metrics| metrics.settling_time_s.is_none())
                .count(),
            seeds,
        }
    }

    /// Serializes the report as pretty JSON for artifacts.
    pub fn to_json_pretty(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(time_s: f64, error_m: f64, command: f64) -> ControlTrackingSample {
        ControlTrackingSample {
            time_s,
            tracking_error_m: error_m,
            command,
            saturated: false,
            violation: false,
        }
    }

    /// A decaying oscillation that enters the band at 2 s and stays after 3 s.
    fn decaying_run() -> Vec<ControlTrackingSample> {
        vec![
            sample(0.0, 1.0, 0.5),
            sample(1.0, 0.6, 0.4),
            sample(2.0, 0.09, 0.3),
            sample(3.0, 0.15, 0.2),
            sample(4.0, 0.05, 0.1),
            sample(5.0, 0.02, 0.1),
            sample(6.0, 0.01, 0.1),
        ]
    }

    #[test]
    fn metrics_match_hand_computed_values() {
        let run = vec![sample(0.0, 3.0, 1.0), sample(1.0, 4.0, -1.0)];
        let metrics = ControlMetrics::from_samples(&run, 0.1).unwrap();

        assert!((metrics.rms_error_m - (12.5_f64).sqrt()).abs() < 1e-12);
        assert!((metrics.max_error_m - 4.0).abs() < 1e-12);
        // Effort integrates |command| over the one interval; smoothness its change.
        assert!((metrics.effort - 1.0).abs() < 1e-12);
        assert!((metrics.smoothness - 2.0).abs() < 1e-12);
        assert_eq!(metrics.settling_time_s, None);
        assert!((metrics.duration_s - 1.0).abs() < 1e-12);
    }

    #[test]
    fn settling_ignores_a_temporary_band_entry() {
        let metrics = ControlMetrics::from_samples(&decaying_run(), 0.1).unwrap();

        // The error dips into the band at 2 s but leaves again at 3 s, so settling is
        // only declared from 4 s on.
        assert_eq!(metrics.settling_time_s, Some(4.0));
        // Overshoot is the worst error after the first band entry: the 0.15 bounce.
        assert!((metrics.overshoot_m - 0.15).abs() < 1e-12);
        // Steady state averages the settled tail.
        let expected = (0.05 + 0.02 + 0.01) / 3.0;
        assert!((metrics.steady_state_error_m - expected).abs() < 1e-12);
    }

    #[test]
    fn a_run_that_never_settles_reports_none() {
        let run = vec![
            sample(0.0, 1.0, 0.0),
            sample(1.0, 0.9, 0.0),
            sample(2.0, 1.1, 0.0),
        ];
        let metrics = ControlMetrics::from_samples(&run, 0.1).unwrap();
        assert_eq!(metrics.settling_time_s, None);
        assert_eq!(metrics.overshoot_m, 1.1);
    }

    #[test]
    fn saturation_and_violations_are_counted() {
        let mut run = decaying_run();
        run[1].saturated = true;
        run[2].saturated = true;
        run[3].violation = true;
        let metrics = ControlMetrics::from_samples(&run, 0.1).unwrap();

        assert!((metrics.saturated_fraction - 2.0 / 7.0).abs() < 1e-12);
        assert_eq!(metrics.violation_count, 1);
    }

    #[test]
    fn too_short_runs_are_rejected() {
        assert!(ControlMetrics::from_samples(&[], 0.1).is_none());
        assert!(ControlMetrics::from_samples(&[sample(0.0, 1.0, 0.0)], 0.1).is_none());
    }

    #[test]
    fn report_aggregates_deterministically_across_seeds() {
        let mut seeds = BTreeMap::new();
        for seed in [3_u64, 1, 2] {
            let scale = seed as f64;
            let run = vec![
                sample(0.0, scale, 0.1 * scale),
                sample(1.0, 0.05, 0.1),
                sample(2.0, 0.04, 0.1),
            ];
            seeds.insert(seed, ControlMetrics::from_samples(&run, 0.1).unwrap());
        }
        let report = ControlEvalReport::from_seed_metrics("course", "pure_pursuit", seeds.clone());

        assert_eq!(report.schema_version, ControlEvalReport::SCHEMA_VERSION);
        assert_eq!(
            report.seeds.keys().copied().collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
        assert!((report.max_error_m.min - 1.0).abs() < 1e-12);
        assert!((report.max_error_m.max - 3.0).abs() < 1e-12);
        assert_eq!(report.total_violations, 0);
        assert_eq!(report.unsettled_seeds, 0);

        // Identical inputs give an identical serialized report.
        let again = ControlEvalReport::from_seed_metrics("course", "pure_pursuit", seeds);
        assert_eq!(
            report.to_json_pretty().unwrap(),
            again.to_json_pretty().unwrap()
        );
    }
}
