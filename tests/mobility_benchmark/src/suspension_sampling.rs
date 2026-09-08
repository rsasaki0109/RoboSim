//! Offline distinction between timestamp-label error and displaced signal capture.
//! No wall clock, implicit interpolation, transport model or physical qualification.

use anyhow::{ensure, Result};
use rne_robot::SuspensionForceSample;
use serde::{Deserialize, Serialize};

use crate::suspension_derivative::SuspensionDerivativeOperator;

/// Separate physical sampling instant and instrument-reported timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionCaptureTiming {
    /// Time at which the supplied signal is evaluated, in seconds.
    pub physical_capture_time_s: f64,
    /// Timestamp exposed to the offline derivative/identifier, in seconds.
    pub reported_capture_time_s: f64,
}

/// One training acquisition sampled from an explicitly supplied signal model.
#[derive(Clone, Copy, Debug)]
pub struct SuspensionSignalRun<'a> {
    /// Unique acquisition identity, also selecting independent timing streams.
    pub acquisition_id: u64,
    /// Nominal physical instants and reported timestamps, in capture order.
    pub timing: &'a [SuspensionCaptureTiming],
    /// Explicit offline derivative applied to sampled positions and reported times.
    pub operator: SuspensionDerivativeOperator,
}

/// Temporal sharing within one acquisition; different acquisitions are independent.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionTimingScope {
    /// One constant time displacement for the entire acquisition.
    AcquisitionOffset,
    /// One independent displacement per row, without sorting after perturbation.
    IndependentSamples,
}

/// One independent time-error source coupling physical and reported clocks.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTimingFactor {
    /// Stable unique factor identity, in strictly increasing model order.
    pub factor_id: u64,
    /// Explicit zero-mean unit-variance distribution.
    pub distribution: crate::suspension_uncertainty::SuspensionErrorDistribution,
    /// Constant acquisition offset or independent sample displacements.
    pub scope: SuspensionTimingScope,
    /// Signed physical capture-time loading in seconds per latent unit.
    pub physical_loading_s: f64,
    /// Signed reported timestamp loading in seconds per the same latent unit.
    pub reported_loading_s: f64,
}

/// Bounded generator for one timing realization; no calibration/coverage assertion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTimingErrorModel {
    /// Must be `rne_suspension_timing_error_model`.
    pub kind: String,
    /// Currently 1, independent of affine and additive models.
    pub schema_version: u32,
    /// Explicit WorldRandom root seed.
    pub seed: u64,
    /// One to sixteen independent sources.
    pub factors: Vec<SuspensionTimingFactor>,
}

impl SuspensionTimingErrorModel {
    /// Samples a declared signal for each realized physical clock, then refits.
    /// `signal(acquisition_id, physical_time_s)` returns position/force in SI units
    /// and must be pure and deterministic. No recorded-log interpolation is inferred.
    /// All runs are training inputs; no holdout or live physics advancement occurs.
    /// Signal-domain, clock and derivative failures remain `InvalidSample` in their
    /// original slots, including the baseline. Estimator errors are kept unchanged.
    /// Caps 64 unique runs, 100,000 combined rows and 10 million row-factor-draw
    /// evaluations. Callback cost and authenticity are outside this numerical API.
    pub fn propagate_signal(
        &self,
        spec: rne_robot::SuspensionIdentificationSpec,
        runs: &[SuspensionSignalRun<'_>],
        draws: usize,
        signal: impl Fn(u64, f64) -> Result<(f64, f64)>,
    ) -> Result<crate::suspension_uncertainty::SuspensionErrorPropagation> {
        use rne_robot::{
            fit_suspension_training_runs, SuspensionIdentificationError,
            SuspensionIdentificationRun,
        };
        self.validate()?;
        ensure!(spec.is_valid(), "invalid sampled identification spec");
        ensure!(
            (1..=64).contains(&runs.len()) && (1..=4096).contains(&draws),
            "invalid sampled propagation shape"
        );
        let mut ids = std::collections::BTreeSet::new();
        let mut rows = 0usize;
        for run in runs {
            ensure!(
                ids.insert(run.acquisition_id),
                "duplicate sampled acquisition identity"
            );
            rows = rows
                .checked_add(run.timing.len())
                .ok_or_else(|| anyhow::anyhow!("sampled row overflow"))?;
            ensure!(rows <= 100_000, "too many sampled rows");
            validate_times(
                &run.timing
                    .iter()
                    .map(|t| t.physical_capture_time_s)
                    .collect::<Vec<_>>(),
            )?;
            validate_times(
                &run.timing
                    .iter()
                    .map(|t| t.reported_capture_time_s)
                    .collect::<Vec<_>>(),
            )?;
        }
        ensure!(
            rows.saturating_mul(draws)
                .saturating_mul(self.factors.len())
                <= 10_000_000,
            "sampled propagation work budget exceeded"
        );
        let evaluate = |draw: Option<usize>| {
            let samples = runs
                .iter()
                .map(|run| {
                    let timing = match draw {
                        Some(draw) => self.realize(run.timing, run.acquisition_id, draw)?,
                        None => run.timing.to_vec(),
                    };
                    capture_suspension_signal(
                        &timing,
                        |t| signal(run.acquisition_id, t),
                        run.operator,
                    )
                })
                .collect::<Result<Vec<_>>>();
            match samples {
                Err(_) => Err(SuspensionIdentificationError::InvalidSample),
                Ok(samples) => {
                    let training: Vec<_> = runs
                        .iter()
                        .zip(&samples)
                        .map(|(run, samples)| SuspensionIdentificationRun {
                            acquisition_id: run.acquisition_id,
                            samples,
                        })
                        .collect();
                    fit_suspension_training_runs(spec, &training)
                }
            }
        };
        let baseline = evaluate(None);
        let draws = (0..draws).map(|draw| evaluate(Some(draw))).collect();
        Ok(crate::suspension_uncertainty::SuspensionErrorPropagation { baseline, draws })
    }

    fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_suspension_timing_error_model" && self.schema_version == 1,
            "timing error model kind/schema drift"
        );
        ensure!(
            (1..=16).contains(&self.factors.len()),
            "invalid timing factor count"
        );
        ensure!(
            self.factors
                .windows(2)
                .all(|pair| pair[0].factor_id < pair[1].factor_id),
            "timing factor IDs must be increasing and unique"
        );
        ensure!(
            self.factors
                .iter()
                .all(|factor| factor.physical_loading_s.is_finite()
                    && factor.reported_loading_s.is_finite()),
            "nonfinite timing loading"
        );
        Ok(())
    }

    /// Propagates timestamp-label errors through the actual pooled training fit.
    /// Measured position/force pairs are fixed. Physical-time loadings are rejected
    /// because a recorded dataset does not define an underlying continuous signal.
    /// No holdout values participate in sampling or fitting. Draw errors are kept
    /// in order as `InvalidSample`; actual estimator failures are preserved.
    /// This numerical path does not verify calibration or retained source files.
    pub fn propagate_labels(
        &self,
        request: &crate::suspension_runs::SuspensionRunRequest,
        operators: &[SuspensionDerivativeOperator],
        draws: usize,
    ) -> Result<crate::suspension_uncertainty::SuspensionErrorPropagation> {
        use rne_robot::{
            fit_suspension_training_runs, SuspensionIdentificationError,
            SuspensionIdentificationRun,
        };
        self.validate()?;
        request.validate()?;
        ensure!(
            (1..=4096).contains(&draws) && operators.len() == request.training.len(),
            "invalid timestamp propagation shape"
        );
        ensure!(
            self.factors
                .iter()
                .all(|factor| factor.physical_loading_s == 0.0),
            "physical timing errors require a declared signal, not timestamp relabeling"
        );
        let rows: usize = request
            .training
            .iter()
            .map(|run| run.dataset.samples.len())
            .sum();
        ensure!(
            rows.saturating_mul(draws)
                .saturating_mul(self.factors.len())
                <= 10_000_000,
            "timestamp propagation work budget exceeded"
        );
        let nominal: Vec<Vec<_>> = request
            .training
            .iter()
            .map(|run| {
                run.dataset
                    .samples
                    .iter()
                    .map(|s| SuspensionCaptureTiming {
                        physical_capture_time_s: s.capture_time_s,
                        reported_capture_time_s: s.capture_time_s,
                    })
                    .collect()
            })
            .collect();
        let fit = |samples: &[Vec<SuspensionForceSample>]| {
            let runs: Vec<_> = request
                .training
                .iter()
                .zip(samples)
                .map(|(run, samples)| SuspensionIdentificationRun {
                    acquisition_id: run.acquisition_id,
                    samples,
                })
                .collect();
            fit_suspension_training_runs(request.spec, &runs)
        };
        let baseline_samples = request
            .training
            .iter()
            .zip(&nominal)
            .zip(operators)
            .map(|((run, times), operator)| {
                let times: Vec<_> = times.iter().map(|t| t.reported_capture_time_s).collect();
                relabel_suspension_timestamps(&run.dataset.samples, &times, *operator)
            })
            .collect::<Result<Vec<_>>>()?;
        let baseline = fit(&baseline_samples);
        let mut outcomes = Vec::with_capacity(draws);
        for draw in 0..draws {
            let samples = request
                .training
                .iter()
                .zip(&nominal)
                .zip(operators)
                .map(|((run, times), operator)| {
                    let timing = self.realize(times, run.acquisition_id, draw)?;
                    let reported: Vec<_> =
                        timing.iter().map(|t| t.reported_capture_time_s).collect();
                    relabel_suspension_timestamps(&run.dataset.samples, &reported, *operator)
                })
                .collect::<Result<Vec<_>>>();
            outcomes.push(match samples {
                Ok(samples) => fit(&samples),
                Err(_) => Err(SuspensionIdentificationError::InvalidSample),
            });
        }
        Ok(crate::suspension_uncertainty::SuspensionErrorPropagation {
            baseline,
            draws: outcomes,
        })
    }

    /// Generates draw 0..4095 for an explicit acquisition identity.
    /// Stream derivation uses acquisition, factor and draw identities in separate
    /// levels. Results do not depend on call order or on requesting later draws.
    /// Nominal clocks and realized clocks must remain strictly increasing. A bad
    /// realization is returned as an error, never clipped, sorted or resampled;
    /// callers collecting draws must retain that error at its original draw index.
    pub fn realize(
        &self,
        nominal: &[SuspensionCaptureTiming],
        acquisition_id: u64,
        draw: usize,
    ) -> Result<Vec<SuspensionCaptureTiming>> {
        use crate::suspension_uncertainty::{draw_rng, latent};
        use rne_world::{RandomStreamId, WorldRandom};
        self.validate()?;
        ensure!(
            (1..=16).contains(&self.factors.len())
                && draw < 4096
                && (3..=100_000).contains(&nominal.len()),
            "invalid timing error workload"
        );
        validate_times(
            &nominal
                .iter()
                .map(|t| t.physical_capture_time_s)
                .collect::<Vec<_>>(),
        )?;
        validate_times(
            &nominal
                .iter()
                .map(|t| t.reported_capture_time_s)
                .collect::<Vec<_>>(),
        )?;
        let root = WorldRandom::new(self.seed);
        let acquisition = WorldRandom::new(root.stream_seed(RandomStreamId::new(acquisition_id)));
        let mut timing = nominal.to_vec();
        for factor in &self.factors {
            let mut rng = draw_rng(&acquisition, factor.factor_id, draw);
            let common = latent(&mut rng, factor.distribution);
            for time in &mut timing {
                let value = match factor.scope {
                    SuspensionTimingScope::AcquisitionOffset => common,
                    SuspensionTimingScope::IndependentSamples => {
                        latent(&mut rng, factor.distribution)
                    }
                };
                time.physical_capture_time_s += value * factor.physical_loading_s;
                time.reported_capture_time_s += value * factor.reported_loading_s;
            }
        }
        validate_times(
            &timing
                .iter()
                .map(|t| t.physical_capture_time_s)
                .collect::<Vec<_>>(),
        )?;
        validate_times(
            &timing
                .iter()
                .map(|t| t.reported_capture_time_s)
                .collect::<Vec<_>>(),
        )?;
        Ok(timing)
    }
}

fn validate_times(times: &[f64]) -> Result<()> {
    ensure!(
        (3..=100_000).contains(&times.len()),
        "invalid timing row count"
    );
    ensure!(
        times.iter().all(|t| t.is_finite()),
        "nonfinite capture time"
    );
    ensure!(
        times.windows(2).all(|pair| {
            let dt_s = pair[1] - pair[0];
            dt_s.is_finite() && dt_s > 0.0
        }),
        "invalid capture interval"
    );
    Ok(())
}

fn derive(
    mut samples: Vec<SuspensionForceSample>,
    operator: SuspensionDerivativeOperator,
) -> Result<Vec<SuspensionForceSample>> {
    let times: Vec<_> = samples.iter().map(|s| s.capture_time_s).collect();
    let positions: Vec<_> = samples.iter().map(|s| s.position_m).collect();
    for (sample, velocity) in samples
        .iter_mut()
        .zip(operator.reconstruct(&times, &positions)?)
    {
        sample.velocity_m_s = velocity;
    }
    Ok(samples)
}

/// Changes timestamp labels while retaining each acquired position/force pair.
/// Velocity is rederived using the replacement labels. This does not simulate
/// physical aperture jitter, resample a trajectory or change transport latency.
/// Rejects malformed source clocks and replacement clocks; never sorts or trims.
pub fn relabel_suspension_timestamps(
    samples: &[SuspensionForceSample],
    reported_times_s: &[f64],
    operator: SuspensionDerivativeOperator,
) -> Result<Vec<SuspensionForceSample>> {
    ensure!(
        samples.len() == reported_times_s.len(),
        "timestamp row mismatch"
    );
    validate_times(reported_times_s)?;
    let source_times: Vec<_> = samples.iter().map(|s| s.capture_time_s).collect();
    validate_times(&source_times)?;
    ensure!(
        samples
            .iter()
            .all(|s| [s.position_m, s.velocity_m_s, s.force_n]
                .into_iter()
                .all(f64::is_finite)),
        "nonfinite source sample"
    );
    let corrected = samples
        .iter()
        .zip(reported_times_s)
        .map(|(sample, time)| SuspensionForceSample {
            capture_time_s: *time,
            velocity_m_s: 0.0,
            ..*sample
        })
        .collect();
    derive(corrected, operator)
}

/// Samples a caller-supplied continuous signal at explicit physical instants.
/// The callback returns `(position_m, force_n)` and must be pure and deterministic;
/// its model/validity interval are the caller's responsibility. No interpolation
/// or extrapolation is inferred from a recorded dataset. Finite aperture averaging
/// is not modeled: these are point captures. Errors reject the entire realization.
/// Derived velocity uses only reported times, never privileged physical times.
pub fn capture_suspension_signal(
    timing: &[SuspensionCaptureTiming],
    signal: impl Fn(f64) -> Result<(f64, f64)>,
    operator: SuspensionDerivativeOperator,
) -> Result<Vec<SuspensionForceSample>> {
    ensure!(
        (3..=100_000).contains(&timing.len()),
        "invalid timing row count"
    );
    let physical: Vec<_> = timing.iter().map(|t| t.physical_capture_time_s).collect();
    let reported: Vec<_> = timing.iter().map(|t| t.reported_capture_time_s).collect();
    // Validate both clocks before evaluating any signal values.
    validate_times(&physical)?;
    validate_times(&reported)?;
    let samples = timing
        .iter()
        .map(|time| {
            let (position_m, force_n) = signal(time.physical_capture_time_s)?;
            ensure!(
                position_m.is_finite() && force_n.is_finite(),
                "nonfinite captured signal"
            );
            Ok(SuspensionForceSample {
                capture_time_s: time.reported_capture_time_s,
                position_m,
                force_n,
                velocity_m_s: 0.0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    derive(samples, operator)
}

#[cfg(test)]
mod tests {
    use super::*;
    const OP: SuspensionDerivativeOperator =
        SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;

    #[test]
    fn sampled_propagation_rejects_invalid_requests_before_signal_evaluation() {
        use crate::suspension_identification::suspension_identification_spec;
        use crate::suspension_uncertainty::SuspensionErrorDistribution;
        let timing: Vec<_> = (0..200)
            .map(|i| SuspensionCaptureTiming {
                physical_capture_time_s: i as f64 * 0.01,
                reported_capture_time_s: i as f64 * 0.01,
            })
            .collect();
        let model = SuspensionTimingErrorModel {
            kind: "rne_suspension_timing_error_model".into(),
            schema_version: 1,
            seed: 42,
            factors: vec![SuspensionTimingFactor {
                factor_id: 7,
                distribution: SuspensionErrorDistribution::Rectangular,
                scope: SuspensionTimingScope::IndependentSamples,
                physical_loading_s: 0.001,
                reported_loading_s: 0.0,
            }],
        };
        let run = SuspensionSignalRun {
            acquisition_id: 1,
            timing: &timing,
            operator: OP,
        };
        let spec = suspension_identification_spec();
        let never_sample = |_, _| panic!("invalid request must not evaluate a signal");
        for draws in [0, 4097] {
            assert!(model
                .propagate_signal(spec, &[run], draws, never_sample)
                .is_err());
        }
        assert!(model.propagate_signal(spec, &[], 1, never_sample).is_err());
        let many_runs: Vec<_> = (0..64)
            .map(|acquisition_id| SuspensionSignalRun {
                acquisition_id,
                ..run
            })
            .collect();
        assert!(model
            .propagate_signal(spec, &many_runs, 4096, never_sample)
            .is_err());
        let mut malformed = timing.clone();
        malformed[1].reported_capture_time_s = malformed[0].reported_capture_time_s;
        let bad_run = SuspensionSignalRun {
            timing: &malformed,
            ..run
        };
        assert!(model
            .propagate_signal(spec, &[bad_run], 1, never_sample)
            .is_err());
    }

    #[test]
    fn sampled_draws_refit_actual_signals_and_retain_domain_failures() {
        use crate::suspension_identification::suspension_identification_spec;
        use crate::suspension_uncertainty::SuspensionErrorDistribution;
        use rne_robot::SuspensionIdentificationError;
        let timing: Vec<_> = (0..200)
            .map(|i| SuspensionCaptureTiming {
                physical_capture_time_s: i as f64 * 0.01,
                reported_capture_time_s: i as f64 * 0.01,
            })
            .collect();
        let runs = [
            SuspensionSignalRun {
                acquisition_id: 1,
                timing: &timing,
                operator: OP,
            },
            SuspensionSignalRun {
                acquisition_id: 2,
                timing: &timing,
                operator: OP,
            },
        ];
        let mut model = SuspensionTimingErrorModel {
            kind: "rne_suspension_timing_error_model".into(),
            schema_version: 1,
            seed: 42,
            factors: vec![SuspensionTimingFactor {
                factor_id: 7,
                distribution: SuspensionErrorDistribution::Rectangular,
                scope: SuspensionTimingScope::IndependentSamples,
                physical_loading_s: 0.001,
                reported_loading_s: 0.0,
            }],
        };
        let signal = |id: u64, t: f64| {
            let x = -0.06 + 0.01 * t * t + 0.001 * id as f64;
            Ok((x, -200_000.0 * (x + 0.06) - 15_000.0 * 0.02 * t))
        };
        let spec = suspension_identification_spec();
        let hidden = model.propagate_signal(spec, &runs, 8, signal).unwrap();
        assert!(hidden.baseline.is_ok());
        assert_eq!(
            hidden,
            model.propagate_signal(spec, &runs, 8, signal).unwrap()
        );
        assert_eq!(
            &model
                .propagate_signal(spec, &runs, 16, signal)
                .unwrap()
                .draws[..8],
            &hidden.draws
        );
        model.factors[0].reported_loading_s = 0.001;
        let reported = model.propagate_signal(spec, &runs, 8, signal).unwrap();
        assert_ne!(hidden.draws, reported.draws);
        assert!(reported.draws.iter().all(Result::is_ok));
        model.factors[0].physical_loading_s = 0.0;
        model.factors[0].reported_loading_s = 0.0;
        let zero = model.propagate_signal(spec, &runs, 8, signal).unwrap();
        assert!(zero.draws.iter().all(|draw| *draw == zero.baseline));
        let failure = model
            .propagate_signal(spec, &runs, 8, |_, _| {
                anyhow::bail!("signal domain unavailable")
            })
            .unwrap();
        assert_eq!(
            failure.baseline,
            Err(SuspensionIdentificationError::InvalidSample)
        );
        assert_eq!(
            failure.draws,
            vec![Err(SuspensionIdentificationError::InvalidSample); 8]
        );
        assert!(model
            .propagate_signal(spec, &[runs[0], runs[0]], 8, |_, _| panic!(
                "invalid request"
            ))
            .is_err());
        model.factors[0].physical_loading_s = 10.0;
        let invalid = model.propagate_signal(spec, &runs, 8, signal).unwrap();
        assert!(invalid.baseline.is_ok());
        assert_eq!(
            invalid.draws,
            vec![Err(SuspensionIdentificationError::InvalidSample); 8]
        );
    }

    #[test]
    fn seeded_timing_preserves_correlations_and_rejects_invalid_draws() {
        use crate::suspension_uncertainty::SuspensionErrorDistribution;
        let nominal: Vec<_> = (0..100)
            .map(|i| SuspensionCaptureTiming {
                physical_capture_time_s: i as f64 * 0.01,
                reported_capture_time_s: i as f64 * 0.01,
            })
            .collect();
        let mut model = SuspensionTimingErrorModel {
            kind: "rne_suspension_timing_error_model".into(),
            schema_version: 1,
            seed: 42,
            factors: vec![SuspensionTimingFactor {
                factor_id: 7,
                distribution: SuspensionErrorDistribution::Rectangular,
                scope: SuspensionTimingScope::IndependentSamples,
                physical_loading_s: 0.001,
                reported_loading_s: 0.001,
            }],
        };
        let first = model.realize(&nominal, 17, 0).unwrap();
        assert!(first
            .iter()
            .all(|t| t.physical_capture_time_s == t.reported_capture_time_s));
        assert_ne!(first, model.realize(&nominal, 18, 0).unwrap());
        assert_ne!(first, model.realize(&nominal, 17, 1).unwrap());
        assert_eq!(first, model.realize(&nominal, 17, 0).unwrap());
        let samples = capture_suspension_signal(&first, |t| Ok((t * t, 1.0)), OP).unwrap();
        for (sample, time) in samples[1..99].iter().zip(&first[1..99]) {
            assert!((sample.velocity_m_s - 2.0 * time.physical_capture_time_s).abs() < 1e-12);
        }
        model.factors[0].reported_loading_s = 0.0;
        let hidden = model.realize(&nominal, 17, 0).unwrap();
        assert!(hidden
            .iter()
            .zip(&nominal)
            .all(|(a, b)| a.reported_capture_time_s == b.reported_capture_time_s));
        assert!(hidden
            .iter()
            .zip(&first)
            .all(|(a, b)| a.physical_capture_time_s == b.physical_capture_time_s));
        model.factors[0].physical_loading_s = 1.0;
        let outcomes: Vec<_> = (0..16)
            .map(|draw| model.realize(&nominal, 17, draw))
            .collect();
        assert_eq!(outcomes.len(), 16);
        assert!(outcomes.iter().all(Result::is_err));
        model.factors[0].scope = SuspensionTimingScope::AcquisitionOffset;
        assert!(model.realize(&nominal, 17, 0).is_ok());
        assert!(model.realize(&nominal, 17, 4096).is_err());
        model.factors.push(model.factors[0].clone());
        assert!(model.realize(&nominal, 17, 0).is_err());
    }

    #[test]
    fn label_error_and_physical_sampling_are_distinct() {
        let nominal = [0.0, 0.5, 1.0, 2.0];
        let displaced = [0.0, 0.6, 1.0, 2.0];
        let timing = |physical: &[f64], reported: &[f64]| {
            physical
                .iter()
                .zip(reported)
                .map(|(p, r)| SuspensionCaptureTiming {
                    physical_capture_time_s: *p,
                    reported_capture_time_s: *r,
                })
                .collect::<Vec<_>>()
        };
        let signal = |t: f64| Ok((t * t, 3.0 * t + 1.0));
        let source = capture_suspension_signal(&timing(&nominal, &nominal), signal, OP).unwrap();
        let labels = relabel_suspension_timestamps(&source, &displaced, OP).unwrap();
        assert_eq!(labels[1].position_m, 0.25);
        assert_eq!(labels[1].force_n, 2.5);
        let capture = capture_suspension_signal(&timing(&displaced, &nominal), signal, OP).unwrap();
        assert_eq!(capture[1].capture_time_s, 0.5);
        assert!((capture[1].position_m - 0.36).abs() < 1e-12);
        assert!((capture[1].force_n - 2.8).abs() < 1e-12);
        assert_ne!(labels, capture);
        let truthful =
            capture_suspension_signal(&timing(&displaced, &displaced), signal, OP).unwrap();
        assert!((truthful[1].velocity_m_s - 1.2).abs() < 1e-12);
        assert!((truthful[2].velocity_m_s - 2.0).abs() < 1e-12);
        assert_ne!(capture[1].velocity_m_s, truthful[1].velocity_m_s);
        assert_eq!(
            capture,
            capture_suspension_signal(&timing(&displaced, &nominal), signal, OP).unwrap()
        );
        assert_eq!(
            source,
            relabel_suspension_timestamps(&source, &nominal, OP).unwrap()
        );
        assert_eq!(source[1].capture_time_s, 0.5);
    }

    #[test]
    fn actual_estimator_distinguishes_clock_labels_from_sampling_instants() {
        use crate::suspension_identification::suspension_identification_spec;
        use rne_robot::{fit_suspension_training_runs, SuspensionIdentificationRun};
        let scale = 1.1;
        let times: Vec<_> = (0..200).map(|i| i as f64 * 0.01).collect();
        let shifted: Vec<_> = times.iter().map(|t| scale * t).collect();
        let timing = |physical: &[f64], reported: &[f64]| {
            physical
                .iter()
                .zip(reported)
                .map(|(physical, reported)| SuspensionCaptureTiming {
                    physical_capture_time_s: *physical,
                    reported_capture_time_s: *reported,
                })
                .collect::<Vec<_>>()
        };
        // Continuous quadratic motion with a physical linear spring/damper law.
        // All rows, including secant endpoints, enter the actual estimator.
        let signal = |t: f64| {
            let x = -0.06 + 0.01 * t * t;
            let v = 0.02 * t;
            Ok((x, -200_000.0 * (x + 0.06) - 15_000.0 * v))
        };
        let source = capture_suspension_signal(&timing(&times, &times), signal, OP).unwrap();
        let labels = relabel_suspension_timestamps(&source, &shifted, OP).unwrap();
        let actual = capture_suspension_signal(&timing(&shifted, &shifted), signal, OP).unwrap();
        let hidden = capture_suspension_signal(&timing(&shifted, &times), signal, OP).unwrap();
        let fit = |samples: &[SuspensionForceSample]| {
            fit_suspension_training_runs(
                suspension_identification_spec(),
                &[SuspensionIdentificationRun {
                    acquisition_id: 1,
                    samples,
                }],
            )
            .unwrap()
        };
        let source_fit = fit(&source);
        let label_fit = fit(&labels);
        let actual_fit = fit(&actual);
        let hidden_fit = fit(&hidden);
        assert!((label_fit.damping_n_s_per_m - scale * source_fit.damping_n_s_per_m).abs() < 1e-6);
        assert!((hidden_fit.damping_n_s_per_m - actual_fit.damping_n_s_per_m / scale).abs() < 1e-6);
        assert!((label_fit.stiffness_n_per_m - source_fit.stiffness_n_per_m).abs() < 1e-5);
        assert!((hidden_fit.stiffness_n_per_m - actual_fit.stiffness_n_per_m).abs() < 1e-5);
        assert!(label_fit.damping_n_s_per_m > source_fit.damping_n_s_per_m);
        assert!(hidden_fit.damping_n_s_per_m < actual_fit.damping_n_s_per_m);
        assert_eq!(hidden_fit, fit(&hidden));
        assert_ne!(source[100].force_n, actual[100].force_n);
        assert_eq!(source[100].force_n, labels[100].force_n);
        assert_eq!(actual[100].force_n, hidden[100].force_n);
    }

    #[test]
    fn malformed_clocks_and_signal_errors_are_not_hidden() {
        let valid: Vec<_> = [0.0, 0.5, 1.0]
            .into_iter()
            .map(|t| SuspensionCaptureTiming {
                physical_capture_time_s: t,
                reported_capture_time_s: t,
            })
            .collect();
        for bad_time in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            for physical in [true, false] {
                let mut bad = valid.clone();
                if physical {
                    bad[1].physical_capture_time_s = bad_time;
                } else {
                    bad[1].reported_capture_time_s = bad_time;
                }
                assert!(capture_suspension_signal(
                    &bad,
                    |_| panic!("invalid clock must fail before sampling"),
                    OP
                )
                .is_err());
            }
        }
        assert!(capture_suspension_signal(&valid, |_| Ok((f64::INFINITY, 1.0)), OP).is_err());
        assert!(
            capture_suspension_signal(&valid, |_| anyhow::bail!("outside signal domain"), OP)
                .is_err()
        );
        let samples = capture_suspension_signal(&valid, |t| Ok((t, 1.0)), OP).unwrap();
        assert!(relabel_suspension_timestamps(&samples, &[0.0, 0.0, 1.0], OP).is_err());
        assert!(relabel_suspension_timestamps(&samples, &[0.0, 1.0], OP).is_err());
        let mut bad = samples;
        bad[1].capture_time_s = 0.0;
        assert!(relabel_suspension_timestamps(&bad, &[0.0, 0.5, 1.0], OP).is_err());
    }
}
