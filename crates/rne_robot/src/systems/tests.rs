use super::*;
use crate::actuator::ActuatorLimits;
use crate::components::{
    AckermannDrive, JointKind, JointLimits, LateralLoadTransferSpec, Link, MultirotorFlight, Robot,
    RobotId,
};
use rne_core::{SimClock, SimTime};
use rne_ecs::spawn_named;
use rne_math::Seconds;

#[test]
fn suspension_influence_preserves_deleted_run_failures_and_local_clocks() {
    let samples = suspension_identification_samples();
    let run = SuspensionIdentificationRun {
        acquisition_id: 7,
        samples: &samples,
    };
    let spec = suspension_identification_spec();
    let single = suspension_training_influence(spec, &[run]).unwrap();
    assert!(single.baseline.is_ok());
    assert_eq!(
        single.deletions[0].coefficients,
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    let other = SuspensionIdentificationRun {
        acquisition_id: 9,
        samples: &samples,
    };
    let report = suspension_training_influence(spec, &[run, other]).unwrap();
    assert_eq!(report.deletions[0].omitted_acquisition_id, 7);
    assert_eq!(report.deletions[1].omitted_acquisition_id, 9);
    assert_eq!(report.deletions[0].coefficients, single.baseline);
    assert_eq!(
        report,
        suspension_training_influence(spec, &[run, other]).unwrap()
    );
    assert_eq!(
        suspension_training_influence(spec, &[run, run]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    let mut constant = samples.clone();
    for sample in &mut constant {
        sample.position_m = 0.0;
        sample.velocity_m_s = 0.0;
    }
    let rankless = SuspensionIdentificationRun {
        acquisition_id: 11,
        samples: &constant,
    };
    let report = suspension_training_influence(spec, &[run, rankless]).unwrap();
    assert_eq!(
        report.deletions[0].coefficients,
        Err(SuspensionIdentificationError::RankDeficient)
    );
    assert_eq!(report.deletions[1].coefficients, single.baseline);
}

#[test]
fn suspension_training_estimator_matches_baseline_and_propagates_common_offset() {
    let spec = suspension_identification_spec();
    let samples = suspension_identification_samples();
    let run = SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &samples,
    };
    let baseline = fit_suspension_training_runs(spec, &[run]).unwrap();
    assert_eq!(
        Ok(baseline),
        suspension_training_influence(spec, &[run])
            .unwrap()
            .baseline
    );
    let mut shifted = samples.clone();
    for sample in &mut shifted {
        sample.position_m += 0.001;
        sample.force_n += 100.0;
    }
    let shifted_run = SuspensionIdentificationRun {
        acquisition_id: 2,
        samples: &shifted,
    };
    let fit = fit_suspension_training_runs(spec, &[shifted_run]).unwrap();
    assert!((fit.stiffness_n_per_m - baseline.stiffness_n_per_m).abs() < 1e-6);
    assert!((fit.damping_n_s_per_m - baseline.damping_n_s_per_m).abs() < 1e-6);
    assert!(
        (fit.equilibrium_position_m
            - baseline.equilibrium_position_m
            - 0.001
            - 100.0 / baseline.stiffness_n_per_m)
            .abs()
            < 1e-10
    );
    // Repeated samples from the same calibration do not erase its common offset.
    let repeated = SuspensionIdentificationRun {
        acquisition_id: 3,
        samples: &shifted,
    };
    let doubled = fit_suspension_training_runs(spec, &[shifted_run, repeated]).unwrap();
    assert!((doubled.equilibrium_position_m - fit.equilibrium_position_m).abs() < 1e-10);
    assert_eq!(
        fit_suspension_training_runs(spec, &[run, run]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    assert_eq!(
        fit_suspension_training_runs(spec, &[]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
}

#[test]
fn suspension_influence_retains_nonphysical_refits_without_residual_selection() {
    let samples = suspension_identification_samples();
    let mut shifted = samples.clone();
    // A capture-wide force offset changes equilibrium, not stiffness/damping.
    for sample in &mut shifted {
        sample.force_n += 10_000.0;
    }
    let runs = [
        SuspensionIdentificationRun {
            acquisition_id: 1,
            samples: &samples,
        },
        SuspensionIdentificationRun {
            acquisition_id: 2,
            samples: &shifted,
        },
    ];
    let mut spec = suspension_identification_spec();
    let report = suspension_training_influence(spec, &runs).unwrap();
    let baseline = report.baseline.unwrap();
    let original = report.deletions[1].coefficients.unwrap();
    assert!(
        (baseline.equilibrium_position_m
            - original.equilibrium_position_m
            - 5_000.0 / original.stiffness_n_per_m)
            .abs()
            < 1e-10
    );
    assert_eq!(
        report.deletions[0].coefficients,
        Err(SuspensionIdentificationError::NonPhysicalResult)
    );
    // Residual limits cannot filter runs or change this training-only diagnostic.
    spec.maximum_training_rmse_n = 0.0;
    spec.maximum_holdout_rmse_n = 0.0;
    assert_eq!(report, suspension_training_influence(spec, &runs).unwrap());
    let failed = suspension_training_influence(spec, &runs[1..]).unwrap();
    assert_eq!(
        failed.baseline,
        Err(SuspensionIdentificationError::NonPhysicalResult)
    );
    assert_eq!(failed.deletions.len(), 1);
}

#[test]
fn suspension_influence_validates_every_capture_before_refitting() {
    let samples = suspension_identification_samples();
    let spec = suspension_identification_spec();
    let run = SuspensionIdentificationRun {
        acquisition_id: 0,
        samples: &samples,
    };
    assert_eq!(
        suspension_training_influence(spec, &[]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    let many: Vec<_> = (0..65)
        .map(|acquisition_id| SuspensionIdentificationRun {
            acquisition_id,
            samples: &samples,
        })
        .collect();
    assert_eq!(
        suspension_training_influence(spec, &many),
        Err(SuspensionIdentificationError::InvalidSpec)
    );
    let mut invalid = samples.clone();
    invalid[1].capture_time_s = invalid[0].capture_time_s;
    let bad = SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &invalid,
    };
    assert_eq!(
        suspension_training_influence(spec, &[run, bad]),
        Err(SuspensionIdentificationError::InvalidSample)
    );
    invalid[1].capture_time_s = samples[1].capture_time_s;
    invalid[0].force_n = f64::NAN;
    let bad = SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &invalid,
    };
    assert_eq!(
        suspension_training_influence(spec, &[run, bad]),
        Err(SuspensionIdentificationError::InvalidSample)
    );
}

fn suspension_identification_spec() -> SuspensionIdentificationSpec {
    SuspensionIdentificationSpec {
        holdout_stride: 5,
        minimum_training_samples: 40,
        minimum_holdout_samples: 10,
        stiffness_bounds_n_per_m: [100_000.0, 300_000.0],
        damping_bounds_n_s_per_m: [5_000.0, 30_000.0],
        equilibrium_position_bounds_m: [-0.10, -0.02],
        maximum_training_rmse_n: 5.0,
        maximum_holdout_rmse_n: 5.0,
    }
}

fn suspension_identification_samples() -> Vec<SuspensionForceSample> {
    let stiffness_n_per_m = 200_000.0;
    let damping_n_s_per_m = 15_000.0;
    let equilibrium_position_m = -0.061;
    (0..200)
        .map(|index| {
            let time_s = index as f64 * 0.01;
            let fast_phase = std::f64::consts::TAU * 1.2 * time_s;
            let slow_phase = std::f64::consts::TAU * 0.37 * time_s;
            let position_m = -0.055 + 0.010 * fast_phase.sin() + 0.004 * slow_phase.sin();
            let velocity_m_s = 0.010 * std::f64::consts::TAU * 1.2 * fast_phase.cos()
                + 0.004 * std::f64::consts::TAU * 0.37 * slow_phase.cos();
            let deterministic_noise_n = ((index * 17 % 11) as f64 - 5.0) * 0.2;
            let force_n = stiffness_n_per_m * (equilibrium_position_m - position_m)
                - damping_n_s_per_m * velocity_m_s
                + deterministic_noise_n;
            SuspensionForceSample {
                capture_time_s: time_s,
                position_m,
                velocity_m_s,
                force_n,
            }
        })
        .collect()
}

#[test]
fn suspension_residual_timing_preserves_clock_and_constant_missingness() {
    let fit = identify_suspension_strut(
        suspension_identification_spec(),
        &suspension_identification_samples(),
    )
    .unwrap();
    let mut samples: Vec<_> = [1.0, -1.0, 1.0, -1.0]
        .into_iter()
        .enumerate()
        .map(|(i, residual)| SuspensionForceSample {
            capture_time_s: i as f64,
            position_m: fit.equilibrium_position_m,
            velocity_m_s: 0.0,
            force_n: -residual,
        })
        .collect();
    let diagnose = |samples: &[SuspensionForceSample], tolerance| {
        suspension_residual_timing(
            fit,
            SuspensionIdentificationRun {
                acquisition_id: 42,
                samples,
            },
            tolerance,
        )
        .unwrap()
    };
    let regular = diagnose(&samples, 0.0);
    assert_eq!(regular.lag_one_autocorrelation, Some(-0.75));
    assert_eq!(regular.mean_residual_n, 0.0);
    assert_eq!(regular.minimum_interval_s, 1.0);
    assert_eq!(regular.acquisition_id, 42);
    samples[3].capture_time_s = 4.0;
    let irregular = diagnose(&samples, 0.0);
    assert!(!irregular.uniform_within_tolerance);
    assert_eq!(irregular.maximum_interval_s, 2.0);
    assert_eq!(irregular.lag_one_autocorrelation, None);
    samples[3].capture_time_s = 3.0;
    for sample in &mut samples {
        sample.force_n = -2.0;
    }
    let constant = diagnose(&samples, 0.0);
    assert!(constant.uniform_within_tolerance);
    assert_eq!(constant.mean_residual_n, 2.0);
    assert_eq!(constant.lag_one_autocorrelation, None);
}

#[test]
fn suspension_residual_timing_rejects_invalid_arithmetic_and_respects_tolerance() {
    let fit = identify_suspension_strut(
        suspension_identification_spec(),
        &suspension_identification_samples(),
    )
    .unwrap();
    let mut samples: Vec<_> = [0.0, 1.0, 2.125]
        .into_iter()
        .enumerate()
        .map(|(i, capture_time_s)| SuspensionForceSample {
            capture_time_s,
            position_m: fit.equilibrium_position_m,
            velocity_m_s: 0.0,
            force_n: i as f64 - 1.0,
        })
        .collect();
    let diagnose = |samples: &[SuspensionForceSample], tolerance| {
        suspension_residual_timing(
            fit,
            SuspensionIdentificationRun {
                acquisition_id: 7,
                samples,
            },
            tolerance,
        )
    };
    let accepted = diagnose(&samples, 0.125).unwrap();
    assert!(accepted.uniform_within_tolerance);
    assert_eq!(accepted.lag_one_autocorrelation, Some(0.0));
    assert_eq!(
        diagnose(&samples, 0.0625).unwrap().lag_one_autocorrelation,
        None
    );
    for sample in &mut samples {
        sample.capture_time_s += 1024.0;
    }
    assert_eq!(diagnose(&samples, 0.125).unwrap(), accepted);
    for tolerance in [-1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            diagnose(&samples, tolerance),
            Err(SuspensionIdentificationError::InvalidSpec)
        );
    }
    assert_eq!(
        diagnose(&samples[..2], 0.0),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    samples[0].capture_time_s = -f64::MAX;
    samples[1].capture_time_s = f64::MAX / 2.0;
    samples[2].capture_time_s = f64::MAX;
    assert_eq!(
        diagnose(&samples, 0.0),
        Err(SuspensionIdentificationError::InvalidSample)
    );
    for (i, sample) in samples.iter_mut().enumerate() {
        sample.capture_time_s = i as f64;
    }
    samples[0].position_m = f64::MAX;
    assert_eq!(
        diagnose(&samples, 0.0),
        Err(SuspensionIdentificationError::ResidualExceeded)
    );
}

#[test]
fn suspension_excitation_extreme_scales_and_invalid_inputs_are_explicit() {
    for scale in [f64::MIN_POSITIVE, 1.0, f64::MAX] {
        let mut samples: Vec<_> = [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
            .into_iter()
            .enumerate()
            .map(|(i, (x, v))| SuspensionForceSample {
                capture_time_s: i as f64,
                position_m: x * scale,
                velocity_m_s: v * scale,
                force_n: 0.0,
            })
            .collect();
        let result = suspension_training_excitation(&[SuspensionIdentificationRun {
            acquisition_id: 1,
            samples: &samples,
        }])
        .unwrap();
        assert_eq!(result.position_rms_m, scale);
        assert_eq!(result.velocity_rms_m_s, scale);
        assert_eq!(result.normalized_design_condition, Some(1.0));
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            samples[0].force_n = invalid;
            assert_eq!(
                suspension_training_excitation(&[SuspensionIdentificationRun {
                    acquisition_id: 1,
                    samples: &samples
                }]),
                Err(SuspensionIdentificationError::InvalidSample)
            );
        }
    }
}

#[test]
fn suspension_excitation_near_collinearity_and_force_independence() {
    // Orthogonal x/z columns with equal norm; v = x + epsilon*z.
    // The normalized design condition is (sqrt(1+epsilon^2)+1)/epsilon.
    let epsilon = 0.01;
    let mut samples: Vec<_> = [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
        .into_iter()
        .enumerate()
        .map(|(i, (x, z))| SuspensionForceSample {
            capture_time_s: i as f64,
            position_m: x,
            velocity_m_s: x + epsilon * z,
            force_n: 0.0,
        })
        .collect();
    let diagnose = |samples: &[SuspensionForceSample]| {
        suspension_training_excitation(&[SuspensionIdentificationRun {
            acquisition_id: 1,
            samples,
        }])
        .unwrap()
    };
    let baseline = diagnose(&samples);
    let expected = ((1.0 + epsilon * epsilon).sqrt() + 1.0) / epsilon;
    let actual = baseline.normalized_design_condition.unwrap();
    assert!((actual - expected).abs() < expected * 1.0e-10);
    assert!(actual > 200.0);
    for (i, sample) in samples.iter_mut().enumerate() {
        sample.force_n = 1.0e100 * (i as f64 - 2.0);
    }
    assert_eq!(diagnose(&samples), baseline);
    let run = SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &samples,
    };
    assert_eq!(
        suspension_training_excitation(&[run, run]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    assert_eq!(
        suspension_training_excitation(&[]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    samples[1].capture_time_s = samples[0].capture_time_s;
    assert_eq!(
        suspension_training_excitation(&[SuspensionIdentificationRun {
            acquisition_id: 1,
            samples: &samples
        }]),
        Err(SuspensionIdentificationError::InvalidSample)
    );
}

#[test]
fn suspension_excitation_orthogonal_collinear_and_constant_designs() {
    let mut samples: Vec<_> = [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
        .into_iter()
        .enumerate()
        .map(|(i, (x, v))| SuspensionForceSample {
            capture_time_s: i as f64,
            position_m: x,
            velocity_m_s: v,
            force_n: 0.0,
        })
        .collect();
    let diagnose = |samples: &[SuspensionForceSample]| {
        suspension_training_excitation(&[SuspensionIdentificationRun {
            acquisition_id: 1,
            samples,
        }])
        .unwrap()
    };
    let result = diagnose(&samples);
    assert_eq!(result.normalized_design_condition, Some(1.0));
    assert_eq!(result.position_rms_m, 1.0);
    for sample in &mut samples {
        sample.position_m *= 1.0e150;
        sample.velocity_m_s *= 1.0e-150;
        sample.force_n = 999.0;
    }
    let scaled = diagnose(&samples);
    assert_eq!(scaled.normalized_design_condition, Some(1.0));
    assert_eq!(scaled.position_rms_m, 1.0e150);
    assert_eq!(scaled.velocity_rms_m_s, 1.0e-150);
    for sample in &mut samples {
        sample.velocity_m_s = -sample.position_m;
    }
    let singular = diagnose(&samples);
    assert_eq!(singular.position_velocity_correlation, Some(-1.0));
    assert_eq!(singular.normalized_design_condition, None);
    for sample in &mut samples {
        sample.position_m = 0.123;
    }
    let constant = diagnose(&samples);
    assert_eq!(constant.position_rms_m, 0.0);
    assert_eq!(constant.position_velocity_correlation, None);
    assert_eq!(constant.normalized_design_condition, None);
}

#[test]
fn suspension_run_report_exposes_small_failed_run_hidden_by_pooled_rmse() {
    let samples = suspension_identification_samples();
    let spec = suspension_identification_spec();
    let training = [SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &samples,
    }];
    let mut bad = samples[..10].to_vec();
    for sample in &mut bad {
        sample.force_n += 10.0;
    }
    let holdout = [
        SuspensionIdentificationRun {
            acquisition_id: 2,
            samples: &samples,
        },
        SuspensionIdentificationRun {
            acquisition_id: 3,
            samples: &bad,
        },
    ];
    let report = identify_suspension_strut_runs_report(spec, &training, &holdout).unwrap();
    assert!(report.fit.holdout_rmse_n < spec.maximum_holdout_rmse_n);
    assert!(!report.passed);
    assert!(report.training_runs[0].passed);
    assert!(report.holdout_runs[0].passed);
    assert!(!report.holdout_runs[1].passed);
    assert_eq!(report.holdout_runs[1].acquisition_id, 3);
    assert_eq!(report.holdout_runs[1].sample_count, 10);
    assert!(report.holdout_runs[1].rmse_n > 9.0);
    assert_eq!(
        report,
        identify_suspension_strut_runs_report(spec, &training, &holdout).unwrap()
    );
    let healthy = identify_suspension_strut_runs_report(spec, &training, &holdout[..1]).unwrap();
    assert!(healthy.passed);
    assert_eq!(
        healthy.fit.stiffness_n_per_m.to_bits(),
        report.fit.stiffness_n_per_m.to_bits()
    );
    assert_eq!(
        healthy.fit.damping_n_s_per_m.to_bits(),
        report.fit.damping_n_s_per_m.to_bits()
    );
}

#[test]
fn suspension_run_split_excludes_holdout_from_fit_and_preserves_v1_arithmetic() {
    let samples = suspension_identification_samples();
    let spec = suspension_identification_spec();
    let train: Vec<_> = samples
        .iter()
        .copied()
        .enumerate()
        .filter(|(i, _)| !(i + 1).is_multiple_of(spec.holdout_stride))
        .map(|(_, sample)| sample)
        .collect();
    let mut holdout: Vec<_> = samples
        .iter()
        .copied()
        .enumerate()
        .filter(|(i, _)| (i + 1).is_multiple_of(spec.holdout_stride))
        .map(|(_, sample)| sample)
        .collect();
    let training_runs = [SuspensionIdentificationRun {
        acquisition_id: 1,
        samples: &train,
    }];
    let fit = |validation: &[SuspensionForceSample]| {
        identify_suspension_strut_runs(
            spec,
            &training_runs,
            &[SuspensionIdentificationRun {
                acquisition_id: 2,
                samples: validation,
            }],
        )
        .unwrap()
    };
    let baseline = fit(&holdout);
    // Exactly the same ordered inputs reach the shared solver, preserving v1 bytes.
    assert_eq!(baseline, identify_suspension_strut(spec, &samples).unwrap());
    for sample in &mut holdout {
        sample.force_n += 2.0;
        // Independent acquisitions may restart their capture clocks.
        sample.capture_time_s -= 0.04;
    }
    let changed = fit(&holdout);
    assert_eq!(
        baseline.stiffness_n_per_m.to_bits(),
        changed.stiffness_n_per_m.to_bits()
    );
    assert_eq!(
        baseline.damping_n_s_per_m.to_bits(),
        changed.damping_n_s_per_m.to_bits()
    );
    assert_eq!(
        baseline.equilibrium_position_m.to_bits(),
        changed.equilibrium_position_m.to_bits()
    );
    assert_eq!(
        baseline.training_rmse_n.to_bits(),
        changed.training_rmse_n.to_bits()
    );
    assert!(changed.holdout_rmse_n > baseline.holdout_rmse_n);
    assert_eq!(changed, fit(&holdout));
}

#[test]
fn suspension_run_split_rejects_identity_overlap_empty_runs_and_local_time_drift() {
    let samples = suspension_identification_samples();
    let spec = suspension_identification_spec();
    let run = SuspensionIdentificationRun {
        acquisition_id: 10,
        samples: &samples,
    };
    let other = SuspensionIdentificationRun {
        acquisition_id: 11,
        samples: &samples,
    };
    assert_eq!(
        identify_suspension_strut_runs(spec, &[run], &[run]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    assert_eq!(
        identify_suspension_strut_runs(spec, &[run, run], &[other]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    assert_eq!(
        identify_suspension_strut_runs(spec, &[run], &[other, other]),
        Err(SuspensionIdentificationError::DuplicateAcquisition)
    );
    assert_eq!(
        identify_suspension_strut_runs(spec, &[], &[other]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    assert_eq!(
        identify_suspension_strut_runs(spec, &[run], &[]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    let empty = SuspensionIdentificationRun {
        acquisition_id: 12,
        samples: &[],
    };
    assert_eq!(
        identify_suspension_strut_runs(spec, &[run], &[empty]),
        Err(SuspensionIdentificationError::InsufficientSamples)
    );
    let mut invalid = samples.clone();
    invalid[10].capture_time_s = invalid[9].capture_time_s;
    let invalid_run = SuspensionIdentificationRun {
        acquisition_id: 12,
        samples: &invalid,
    };
    for (training, holdout) in [([run], [invalid_run]), ([invalid_run], [run])] {
        assert_eq!(
            identify_suspension_strut_runs(spec, &training, &holdout),
            Err(SuspensionIdentificationError::InvalidSample)
        );
    }
    // Distinct run IDs and restarted clocks are legal; not proof of real independence.
    let result = identify_suspension_strut_runs(
        spec,
        &[run, other],
        &[SuspensionIdentificationRun {
            acquisition_id: 12,
            samples: &samples,
        }],
    )
    .unwrap();
    assert_eq!(result.training_sample_count, 400);
    assert_eq!(result.holdout_sample_count, 200);
}

#[test]
fn suspension_identification_recovers_parameters_and_holdout_residuals() {
    let result = identify_suspension_strut(
        suspension_identification_spec(),
        &suspension_identification_samples(),
    )
    .unwrap();

    assert!((result.stiffness_n_per_m - 200_000.0).abs() < 20.0);
    assert!((result.damping_n_s_per_m - 15_000.0).abs() < 2.0);
    assert!((result.equilibrium_position_m + 0.061).abs() < 1.0e-5);
    assert_eq!(result.training_sample_count, 160);
    assert_eq!(result.holdout_sample_count, 40);
    assert!(result.training_rmse_n < 1.0);
    assert!(result.holdout_rmse_n < 1.0);
    assert!(result.maximum_absolute_holdout_residual_n < 2.0);
}

#[test]
fn suspension_identification_rejects_nonfinite_holdout_arithmetic() {
    for (position_m, velocity_m_s) in [(1.0e308, -1.0e308), (1.0e308, 0.0), (1.0e160, 0.0)] {
        let mut samples = suspension_identification_samples();
        // Only a held-out sample changes: fitted coefficients remain ordinary.
        samples[199].position_m = position_m;
        samples[199].velocity_m_s = velocity_m_s;
        assert_eq!(
            identify_suspension_strut(suspension_identification_spec(), &samples),
            Err(SuspensionIdentificationError::ResidualExceeded),
            "finite inputs must not admit non-finite residual arithmetic: {position_m}, {velocity_m_s}"
        );
    }
}

#[test]
fn suspension_identification_rejects_rank_loss_time_drift_and_holdout_failure() {
    let spec = suspension_identification_spec();
    let mut rank_deficient = suspension_identification_samples();
    for sample in &mut rank_deficient {
        sample.velocity_m_s = 0.0;
    }
    assert_eq!(
        identify_suspension_strut(spec, &rank_deficient),
        Err(SuspensionIdentificationError::RankDeficient)
    );

    let mut unordered = suspension_identification_samples();
    unordered[10].capture_time_s = unordered[9].capture_time_s;
    assert_eq!(
        identify_suspension_strut(spec, &unordered),
        Err(SuspensionIdentificationError::InvalidSample)
    );

    let mut corrupted_holdout = suspension_identification_samples();
    for (index, sample) in corrupted_holdout.iter_mut().enumerate() {
        if (index + 1).is_multiple_of(spec.holdout_stride) {
            sample.force_n += 50.0;
        }
    }
    assert_eq!(
        identify_suspension_strut(spec, &corrupted_holdout),
        Err(SuspensionIdentificationError::ResidualExceeded)
    );
}

#[test]
fn rigid_road_geometry_places_the_declared_surface_center_exactly() {
    let patch = RigidRoadPatchSpec {
        surface_center_world_m: Vec3::new(3.0, 0.4, 0.0),
        surface_length_m: 4.0,
        half_width_m: 1.5,
        thickness_m: 0.2,
        grade_rad: 0.1,
        friction_scale: 0.8,
    };
    let geometry = rigid_road_patch_geometry(patch).unwrap();
    let reconstructed_surface_center =
        geometry.solid_transform.translation + geometry.normal_world * (0.5 * patch.thickness_m);

    assert!((reconstructed_surface_center - patch.surface_center_world_m).length() < 1.0e-12);
    assert!((geometry.normal_world.length() - 1.0).abs() < 1.0e-12);
    assert!(geometry.longitudinal_tangent_world.y > 0.0);
    assert_eq!(geometry.solid_half_extents_m, Vec3::new(2.0, 0.1, 1.5));
}

#[test]
fn rigid_road_sampling_exposes_grade_friction_and_gaps_deterministically() {
    let profile = RigidRoadProfileSpec {
        patches: vec![
            RigidRoadPatchSpec {
                surface_center_world_m: Vec3::new(0.0, 0.0, 0.0),
                surface_length_m: 2.0,
                half_width_m: 1.0,
                thickness_m: 0.2,
                grade_rad: 0.0,
                friction_scale: 1.0,
            },
            RigidRoadPatchSpec {
                surface_center_world_m: Vec3::new(3.0, 0.2, 0.0),
                surface_length_m: 2.0,
                half_width_m: 1.0,
                thickness_m: 0.2,
                grade_rad: 0.1,
                friction_scale: 0.6,
            },
        ],
    };
    assert!(profile.is_valid());
    let flat = sample_rigid_road_profile(&profile, Vec3::new(0.5, 1.0, 0.2))
        .unwrap()
        .unwrap();
    assert_eq!(flat.patch_index, 0);
    assert_eq!(flat.friction_scale, 1.0);
    assert_eq!(flat.point_world_m, Vec3::new(0.5, 0.0, 0.2));
    assert!(
        sample_rigid_road_profile(&profile, Vec3::new(1.5, 0.0, 0.0))
            .unwrap()
            .is_none()
    );

    let slope = sample_rigid_road_profile(&profile, Vec3::new(3.4, 1.0, -0.3))
        .unwrap()
        .unwrap();
    assert_eq!(slope.patch_index, 1);
    assert_eq!(slope.friction_scale, 0.6);
    assert!(slope.normal_world.x < 0.0);
    assert!(slope.longitudinal_tangent_world.y > 0.0);

    let mut invalid = profile.clone();
    invalid.patches.swap(0, 1);
    assert!(!invalid.is_valid());
    assert_eq!(
        sample_rigid_road_profile(&invalid, Vec3::ZERO),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
}

#[test]
fn suspension_strut_maps_exact_si_force_law() {
    let spec = SuspensionStrutSpec {
        axis_body: Vec3::Y,
        equilibrium_position_m: -0.02,
        minimum_position_m: -0.10,
        maximum_position_m: 0.06,
        stiffness_n_per_m: 24_000.0,
        damping_n_s_per_m: 1_800.0,
        maximum_force_n: 8_000.0,
        unsprung_mass_kg: 18.0,
    };

    assert_eq!(
        evaluate_suspension_strut(spec, 0.01, -0.2).unwrap(),
        JointActuation::PrismaticEffort {
            force_n: -360.0,
            max_force_n: 8_000.0,
        }
    );
}

#[test]
fn suspension_strut_rejects_inverted_travel_and_non_unit_axis() {
    let inverted = SuspensionStrutSpec {
        minimum_position_m: 0.1,
        maximum_position_m: -0.1,
        ..SuspensionStrutSpec::default()
    };
    assert_eq!(
        evaluate_suspension_strut(inverted, 0.0, 0.0),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
    let non_unit = SuspensionStrutSpec {
        axis_body: Vec3::new(0.0, 2.0, 0.0),
        ..SuspensionStrutSpec::default()
    };
    assert_eq!(
        evaluate_suspension_strut(non_unit, 0.0, 0.0),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
    assert_eq!(
        evaluate_suspension_strut(SuspensionStrutSpec::default(), f64::NAN, 0.0),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
    let preloaded_at_droop = SuspensionStrutSpec {
        equilibrium_position_m: -0.12,
        minimum_position_m: -0.08,
        ..SuspensionStrutSpec::default()
    };
    assert!(preloaded_at_droop.is_valid());
    assert!(matches!(
        evaluate_suspension_strut(preloaded_at_droop, -0.08, 0.0),
        Ok(JointActuation::PrismaticEffort { force_n, .. }) if force_n < 0.0
    ));
}

#[test]
fn wheel_station_frame_includes_steering_and_rigid_lever_velocity() {
    let spec = WheelStationSpec {
        center_body_m: Vec3::new(1.0, -0.2, 0.5),
        maximum_steering_rad: std::f64::consts::FRAC_PI_4,
        ..WheelStationSpec::default()
    };
    let frame = resolve_wheel_station_frame(
        spec,
        std::f64::consts::FRAC_PI_6,
        Transform3::from_translation_rotation(Vec3::new(2.0, 1.0, -3.0), Quat::IDENTITY),
        Vec3::new(4.0, 0.0, 1.0),
        Vec3::new(0.0, 2.0, 0.0),
    )
    .unwrap();

    assert!((frame.center_world_m - Vec3::new(3.0, 0.8, -2.5)).length() < 1.0e-12);
    assert!((frame.forward_world - Vec3::new(3.0_f64.sqrt() / 2.0, 0.0, -0.5)).length() < 1.0e-12);
    assert!((frame.lateral_world - Vec3::new(0.5, 0.0, 3.0_f64.sqrt() / 2.0)).length() < 1.0e-12);
    assert!((frame.carrier_velocity_world_m_s - Vec3::new(5.0, 0.0, -1.0)).length() < 1.0e-12);
}

#[test]
fn wheel_station_frame_rejects_steering_beyond_declared_limit() {
    let error = resolve_wheel_station_frame(
        WheelStationSpec::default(),
        0.01,
        Transform3::IDENTITY,
        Vec3::ZERO,
        Vec3::ZERO,
    )
    .unwrap_err();
    assert_eq!(error, MobilityPlantEvaluationError::InvalidInput);
}

#[test]
fn wheel_station_frame_normalizes_backend_quaternion_roundoff() {
    let transform =
        Transform3::from_translation_rotation(Vec3::ZERO, Quat::from_xyzw(0.0, 0.001, 0.0, 1.0));
    let frame = resolve_wheel_station_frame(
        WheelStationSpec::default(),
        0.0,
        transform,
        Vec3::ZERO,
        Vec3::ZERO,
    )
    .unwrap();

    assert!((frame.forward_world.length() - 1.0).abs() < 1.0e-12);
    assert!((frame.lateral_world.length() - 1.0).abs() < 1.0e-12);
    assert!(frame.forward_world.dot(frame.lateral_world).abs() < 1.0e-12);
}

fn setup_robot_with_joint() -> (World, Entity, Entity, Entity) {
    let mut world = World::new();
    let robot_entity = spawn_named(&mut world, "robot");
    let base = spawn_named(&mut world, "base");
    let wheel = spawn_named(&mut world, "wheel");

    world.entity_mut(robot_entity).insert(Robot {
        robot_id: RobotId::default(),
        model_name: "test".into(),
        base_link: base,
    });
    world.entity_mut(base).insert(Link {
        robot: robot_entity,
        name: "base".into(),
    });
    world.entity_mut(wheel).insert((
        Link {
            robot: robot_entity,
            name: "wheel".into(),
        },
        Joint {
            robot: robot_entity,
            parent_link: base,
            child_link: wheel,
            kind: JointKind::Continuous,
            limits: JointLimits::default(),
            axis: Vec3::Y,
            position: 0.0,
            velocity: 0.0,
        },
        Actuator {
            robot: robot_entity,
            joint: Some(wheel),
            name: "wheel_motor".into(),
            mode: ControlMode::Velocity,
            target: Default::default(),
            limits: ActuatorLimits::default(),
        },
    ));

    (world, robot_entity, wheel, wheel)
}

#[test]
fn dc_motor_locked_rotor_obeys_current_and_voltage_limits() {
    let evaluation = evaluate_dc_motor(
        DcMotorSpec::default(),
        DcMotorState::default(),
        48.0,
        0.0,
        0.001,
    )
    .unwrap();

    assert_eq!(evaluation.terminal_voltage_v, 24.0);
    assert_eq!(evaluation.state.current_a, 20.0);
    let telemetry = evaluation.completed_telemetry(DcMotorFailureMode::Nominal, None);
    assert_eq!(telemetry.terminal_voltage_v, 24.0);
    assert_eq!(telemetry.current_a, 20.0);
    assert_eq!(telemetry.winding_temperature_c, None);
    assert!(telemetry.current_saturated);
    assert_eq!(evaluation.electromagnetic_torque_nm, 1.6);
    assert_eq!(evaluation.shaft_loss_torque_nm, 0.01);
    assert_eq!(evaluation.shaft_torque_nm, 1.59);
    assert!(evaluation.voltage_saturated);
    assert!(evaluation.current_saturated);
}

#[test]
fn pwm_motor_frontend_preserves_the_command_voltage_plant_boundary() {
    let frontend = PwmMotorCommandFrontendSpec {
        full_scale_command_count: 100.0,
        bridge_on_state_voltage_drop_v: 2.0,
        polarity: PwmMotorCommandPolarity::Normal,
    };
    let mapped = evaluate_pwm_motor_command(frontend, 25.0, 12.0).unwrap();
    assert_eq!(mapped.clamped_command_count, 25.0);
    assert_eq!(mapped.signed_duty_ratio, 0.25);
    assert_eq!(mapped.ideal_average_voltage_v, 3.0);
    assert_eq!(mapped.terminal_voltage_request_v, 2.5);
    assert_eq!(mapped.average_bridge_loss_v, 0.5);
    assert!(!mapped.command_saturated);

    let motor = evaluate_dc_motor(
        DcMotorSpec {
            supply_voltage_v: 12.0,
            ..DcMotorSpec::default()
        },
        DcMotorState::default(),
        mapped.terminal_voltage_request_v,
        0.0,
        0.001,
    )
    .unwrap();
    assert_eq!(motor.terminal_voltage_v, 2.5);
}

#[test]
fn pwm_motor_frontend_clamps_counts_and_reports_polarity_and_losses() {
    let frontend = PwmMotorCommandFrontendSpec {
        full_scale_command_count: 100.0,
        bridge_on_state_voltage_drop_v: 20.0,
        polarity: PwmMotorCommandPolarity::Inverted,
    };
    let mapped = evaluate_pwm_motor_command(frontend, 125.0, 12.0).unwrap();
    assert_eq!(mapped.clamped_command_count, 100.0);
    assert_eq!(mapped.signed_duty_ratio, -1.0);
    assert_eq!(mapped.ideal_average_voltage_v, -12.0);
    assert_eq!(mapped.terminal_voltage_request_v, 0.0);
    assert_eq!(mapped.average_bridge_loss_v, 12.0);
    assert!(mapped.command_saturated);
}

#[test]
fn pwm_motor_frontend_rejects_invalid_electrical_evidence() {
    let invalid_spec = PwmMotorCommandFrontendSpec {
        full_scale_command_count: 0.0,
        ..PwmMotorCommandFrontendSpec::default()
    };
    assert_eq!(
        evaluate_pwm_motor_command(invalid_spec, 0.0, 12.0),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
    assert_eq!(
        evaluate_pwm_motor_command(PwmMotorCommandFrontendSpec::default(), f64::NAN, 12.0,),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
    assert_eq!(
        evaluate_pwm_motor_command(PwmMotorCommandFrontendSpec::default(), 1.0, -12.0),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
}

#[test]
fn steering_actuator_uses_exact_first_order_response() {
    let spec = SteeringActuatorSpec {
        time_constant_s: 0.2,
        maximum_rate_rad_s: 100.0,
        minimum_position_rad: -1.0,
        maximum_position_rad: 1.0,
        ..SteeringActuatorSpec::default()
    };
    let evaluation =
        evaluate_steering_actuator(spec, SteeringActuatorState::default(), 0.5, 0.1).unwrap();
    let expected_position_rad = 0.5 * (1.0 - (-0.5_f64).exp());

    assert!((evaluation.state.position_rad - expected_position_rad).abs() < 1.0e-12);
    assert!((evaluation.realized_rate_rad_s - expected_position_rad / 0.1).abs() < 1.0e-12);
    assert_eq!(evaluation.clamped_target_rad, 0.5);
    assert!(!evaluation.command_saturated);
    assert!(!evaluation.rate_limited);
    assert!(!evaluation.stuck);
    assert_eq!(
        evaluation,
        evaluate_steering_actuator(spec, SteeringActuatorState::default(), 0.5, 0.1,).unwrap()
    );
}

#[test]
fn steering_actuator_reports_rate_and_travel_saturation() {
    let spec = SteeringActuatorSpec {
        time_constant_s: 0.1,
        maximum_rate_rad_s: 1.0,
        minimum_position_rad: -0.5,
        maximum_position_rad: 0.5,
        ..SteeringActuatorSpec::default()
    };
    let evaluation =
        evaluate_steering_actuator(spec, SteeringActuatorState::default(), 2.0, 0.1).unwrap();

    assert_eq!(evaluation.clamped_target_rad, 0.5);
    assert_eq!(evaluation.state.position_rad, 0.1);
    assert_eq!(evaluation.realized_rate_rad_s, 1.0);
    assert!(evaluation.command_saturated);
    assert!(evaluation.rate_limited);
}

#[test]
fn steering_actuator_deadband_and_stuck_failure_hold_completed_state() {
    let state = SteeringActuatorState { position_rad: 0.1 };
    let deadband = evaluate_steering_actuator(
        SteeringActuatorSpec {
            command_deadband_rad: 0.02,
            ..SteeringActuatorSpec::default()
        },
        state,
        0.11,
        0.01,
    )
    .unwrap();
    assert_eq!(deadband.state, state);
    assert_eq!(deadband.realized_rate_rad_s, 0.0);
    assert!(!deadband.stuck);

    let stuck = evaluate_steering_actuator(
        SteeringActuatorSpec {
            failure_mode: SteeringActuatorFailureMode::Stuck,
            ..SteeringActuatorSpec::default()
        },
        state,
        -0.4,
        0.01,
    )
    .unwrap();
    assert_eq!(stuck.state, state);
    assert_eq!(stuck.realized_rate_rad_s, 0.0);
    assert!(stuck.stuck);
}

#[test]
fn steering_actuator_rejects_invalid_spec_state_command_and_step() {
    assert_eq!(
        evaluate_steering_actuator(
            SteeringActuatorSpec {
                time_constant_s: 0.0,
                ..SteeringActuatorSpec::default()
            },
            SteeringActuatorState::default(),
            0.0,
            0.01,
        ),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
    assert_eq!(
        evaluate_steering_actuator(
            SteeringActuatorSpec::default(),
            SteeringActuatorState { position_rad: 1.0 },
            0.0,
            0.01,
        ),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
    assert_eq!(
        evaluate_steering_actuator(
            SteeringActuatorSpec::default(),
            SteeringActuatorState::default(),
            f64::NAN,
            0.01,
        ),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
    assert_eq!(
        evaluate_steering_actuator(
            SteeringActuatorSpec::default(),
            SteeringActuatorState::default(),
            0.0,
            0.0,
        ),
        Err(MobilityPlantEvaluationError::InvalidTimeStep)
    );
}

fn steering_identification_spec() -> SteeringActuatorIdentificationSpec {
    SteeringActuatorIdentificationSpec {
        training_transition_count: 6,
        interval_tolerance_s: 1.0e-12,
        minimum_abs_command_error_rad: 0.01,
        minimum_time_constant_s: 0.01,
        maximum_time_constant_s: 0.5,
        maximum_training_rms_rad: 1.0e-12,
        maximum_holdout_rms_rad: 1.0e-12,
        minimum_position_rad: -0.5,
        maximum_position_rad: 0.5,
    }
}

fn steering_identification_samples() -> Vec<SteeringActuatorIdentificationSample> {
    let dt_s = 0.01;
    let response = 1.0 - (-dt_s / 0.08_f64).exp();
    let mut position_rad = 0.0;
    (0..=12)
        .map(|index| {
            let command_target_rad = match index {
                0..=3 => 0.4,
                4..=7 => -0.3,
                _ => 0.2,
            };
            let sample = SteeringActuatorIdentificationSample {
                capture_time_s: index as f64 * dt_s,
                command_target_rad,
                measured_position_rad: position_rad,
            };
            position_rad += response * (command_target_rad - position_rad);
            sample
        })
        .collect()
}

#[test]
fn steering_identification_recovers_time_constant_without_holdout_refit() {
    let result = identify_steering_actuator_first_order(
        steering_identification_spec(),
        &steering_identification_samples(),
    )
    .unwrap();

    assert!((result.capture_interval_s - 0.01).abs() < 1.0e-15);
    assert!((result.time_constant_s - 0.08).abs() < 1.0e-12);
    assert_eq!(result.training_transition_count, 6);
    assert_eq!(result.holdout_transition_count, 6);
    assert!(result.training_rms_rad < 1.0e-15);
    assert!(result.holdout_rms_rad < 1.0e-15);
}

#[test]
fn steering_identification_rejects_clock_drift_echo_and_holdout_error() {
    let spec = steering_identification_spec();
    let mut nonuniform = steering_identification_samples();
    nonuniform[3].capture_time_s += 0.001;
    assert_eq!(
        identify_steering_actuator_first_order(spec, &nonuniform),
        Err(SteeringActuatorIdentificationError::InvalidSample)
    );

    let command_echo = (0..=12)
        .map(|index| SteeringActuatorIdentificationSample {
            capture_time_s: index as f64 * 0.01,
            command_target_rad: 0.2,
            measured_position_rad: 0.2,
        })
        .collect::<Vec<_>>();
    assert_eq!(
        identify_steering_actuator_first_order(spec, &command_echo),
        Err(SteeringActuatorIdentificationError::InsufficientExcitation)
    );

    let mut corrupted_holdout = steering_identification_samples();
    corrupted_holdout[10].measured_position_rad += 0.01;
    assert_eq!(
        identify_steering_actuator_first_order(spec, &corrupted_holdout),
        Err(SteeringActuatorIdentificationError::ResidualExceeded)
    );
}

#[test]
fn dc_motor_back_emf_and_failures_are_explicit() {
    let spec = DcMotorSpec::default();
    let free_speed_rad_s = spec.supply_voltage_v / spec.back_emf_constant_v_s_rad;
    let nominal = evaluate_dc_motor(
        spec,
        DcMotorState::default(),
        spec.supply_voltage_v,
        free_speed_rad_s,
        0.001,
    )
    .unwrap();
    assert_eq!(nominal.state.current_a, 0.0);
    assert!(nominal.shaft_torque_nm < 0.0);

    let open = evaluate_dc_motor(
        DcMotorSpec {
            failure_mode: DcMotorFailureMode::OpenCircuit,
            ..spec
        },
        DcMotorState { current_a: 5.0 },
        24.0,
        100.0,
        0.001,
    )
    .unwrap();
    assert_eq!(open.state.current_a, 0.0);
    assert_eq!(open.electromagnetic_torque_nm, 0.0);

    let short = evaluate_dc_motor(
        DcMotorSpec {
            failure_mode: DcMotorFailureMode::ShortCircuit,
            ..spec
        },
        DcMotorState::default(),
        24.0,
        100.0,
        0.001,
    )
    .unwrap();
    assert!(short.state.current_a < 0.0);
    assert!(short.shaft_torque_nm < 0.0);
}

#[test]
fn dc_motor_inductance_retains_deterministic_current_state() {
    let spec = DcMotorSpec {
        resistance_ohm: 1.0,
        torque_constant_nm_a: 1.0,
        back_emf_constant_v_s_rad: 1.0,
        supply_voltage_v: 12.0,
        current_limit_a: 20.0,
        viscous_friction_nm_s_rad: 0.0,
        coulomb_friction_nm: 0.0,
        inductance_h: Some(0.1),
        ..DcMotorSpec::default()
    };
    let first = evaluate_dc_motor(spec, DcMotorState::default(), 1.0, 0.0, 0.01).unwrap();
    let second = evaluate_dc_motor(spec, first.state, 1.0, 0.0, 0.01).unwrap();
    assert!((first.state.current_a - 0.1).abs() < 1.0e-12);
    assert!((second.state.current_a - 0.19).abs() < 1.0e-12);
}

#[test]
fn transmission_maps_directional_efficiency_and_reflected_inertia() {
    let spec = TransmissionSpec::default();
    let drive = evaluate_transmission(spec, 0.001, 1.0, 2.0).unwrap();
    assert_eq!(drive.motor_velocity_rad_s, 40.0);
    assert_eq!(drive.wheel_torque_nm, 18.0);
    assert_eq!(drive.reflected_rotor_inertia_kg_m2, 0.4);
    assert_eq!(drive.applied_efficiency_ratio, 0.9);

    let backdrive = evaluate_transmission(spec, 0.001, -1.0, 2.0).unwrap();
    assert_eq!(backdrive.wheel_torque_nm, -15.0);
    assert_eq!(backdrive.applied_efficiency_ratio, 0.75);
}

#[test]
fn wheel_rolling_resistance_opposes_motion_without_inventing_direction() {
    let spec = WheelAssemblySpec::default();
    assert_eq!(
        wheel_rolling_resistance_torque_nm(spec, 100.0, 0.0).unwrap(),
        0.0
    );
    let forward = wheel_rolling_resistance_torque_nm(spec, 100.0, 2.0).unwrap();
    let reverse = wheel_rolling_resistance_torque_nm(spec, 100.0, -2.0).unwrap();
    assert!((forward + 0.15).abs() < 1.0e-12);
    assert!((reverse - 0.15).abs() < 1.0e-12);
}

#[test]
fn step_bounded_rolling_resistance_cannot_reverse_a_slow_wheel() {
    let spec = WheelAssemblySpec::default();
    let inertia_kg_m2 = 0.5;
    let dt_s = 0.001;
    let velocity_rad_s = 1.0e-6;
    let torque_nm = bounded_rolling_resistance_torque_nm(
        spec,
        100_000_000.0,
        velocity_rad_s,
        inertia_kg_m2,
        dt_s,
    )
    .unwrap();
    assert!((torque_nm + 0.0005).abs() < 1.0e-12);
    let completed_velocity_rad_s = velocity_rad_s + torque_nm / inertia_kg_m2 * dt_s;
    assert!(completed_velocity_rad_s.abs() < 1.0e-15);
}

#[test]
fn mobility_plant_evaluators_reject_invalid_specs_and_inputs() {
    let invalid_motor = DcMotorSpec {
        resistance_ohm: 0.0,
        ..DcMotorSpec::default()
    };
    assert_eq!(
        evaluate_dc_motor(invalid_motor, DcMotorState::default(), 0.0, 0.0, 0.001),
        Err(MobilityPlantEvaluationError::InvalidSpec)
    );
    assert_eq!(
        evaluate_transmission(TransmissionSpec::default(), 0.001, f64::NAN, 0.0),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
    assert_eq!(
        wheel_rolling_resistance_torque_nm(WheelAssemblySpec::default(), -1.0, 0.0),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
}

#[test]
fn valid_command_applies() {
    let (mut world, _, joint, actuator) = setup_robot_with_joint();
    let mut buffer = ActuatorCommandBuffer::new();
    buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: actuator,
            velocity_rad_s: 3.0,
        },
        SimTime::ZERO,
    );
    apply_actuator_commands(&mut world, &mut buffer);
    assert_eq!(
        world
            .get::<Actuator>(actuator)
            .unwrap()
            .target
            .velocity_rad_s,
        3.0
    );
    assert_eq!(world.get::<Joint>(joint).unwrap().velocity, 3.0);
}

#[test]
fn actuator_modes_map_to_unit_explicit_physics_commands() {
    let (mut world, _, joint, actuator) = setup_robot_with_joint();
    {
        let mut actuator = world.get_mut::<Actuator>(actuator).unwrap();
        actuator.mode = ControlMode::Position;
        actuator.target.position_rad = 0.4;
    }
    sync_all_joint_motors_from_actuators(&mut world);
    assert!(matches!(
        world.get::<JointActuation>(joint),
        Some(JointActuation::RevolutePosition {
            target_position_rad: 0.4,
            ..
        })
    ));

    {
        let mut actuator = world.get_mut::<Actuator>(actuator).unwrap();
        actuator.mode = ControlMode::Effort;
        actuator.target.effort_nm = 12.0;
    }
    sync_all_joint_motors_from_actuators(&mut world);
    assert_eq!(
        world.get::<JointActuation>(joint),
        Some(&JointActuation::RevoluteEffort {
            effort_nm: 12.0,
            max_effort_nm: 100.0,
        })
    );
}

#[test]
fn invalid_joint_command_rejected() {
    let (mut world, _, joint, _) = setup_robot_with_joint();
    world.get_mut::<Joint>(joint).unwrap().kind = JointKind::Fixed;
    let result = apply_joint_velocity(&mut world, joint, 1.0);
    assert!(matches!(
        result,
        CommandApplyResult::JointRejected(JointValidationError::FixedJointNonZero)
    ));
}

#[test]
fn diff_drive_moves_forward() {
    let mut world = World::new();
    let spawned = crate::diff_drive::spawn_diff_drive_robot(
        &mut world,
        &crate::diff_drive::DiffDriveConfig::default(),
    );

    let mut buffer = ActuatorCommandBuffer::new();
    buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: spawned.left_actuator,
            velocity_rad_s: 5.0,
        },
        SimTime::ZERO,
    );
    buffer.push(
        ActuatorCommand::WheelVelocity {
            wheel: spawned.right_actuator,
            velocity_rad_s: 5.0,
        },
        SimTime::ZERO,
    );
    apply_actuator_commands(&mut world, &mut buffer);

    differential_drive_kinematics(
        &mut world,
        &[spawned.drive],
        SimDuration::from_seconds(Seconds::new(1.0)),
    );

    let x = world
        .get::<Transform3>(spawned.base_link)
        .unwrap()
        .translation
        .x;
    assert!(x > 0.0, "robot should move forward, x={x}");
    for wheel in [spawned.left_wheel, spawned.right_wheel] {
        let joint = world.get::<Joint>(wheel).unwrap();
        assert_eq!(joint.position, 5.0);
        assert_eq!(joint.velocity, 5.0);
    }
}

#[test]
fn ackermann_commands_clamp_and_integrate_from_sim_clock() {
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "test_vehicle");
    world
        .entity_mut(vehicle)
        .insert((Transform3::default(), AckermannDrive::default()));
    assert_eq!(
        command_ackermann_drive(&mut world, vehicle, 100.0, 2.0),
        AckermannCommandResult::Applied
    );
    let commanded = world.get::<AckermannDrive>(vehicle).unwrap();
    assert_eq!(commanded.target_speed_m_s, commanded.max_speed_m_s);
    assert_eq!(commanded.target_steering_rad, commanded.max_steering_rad);

    let fixed_delta = SimDuration::from_seconds(Seconds::new(1.0 / 60.0));
    let mut clock = SimClock::new(fixed_delta);
    for _ in 0..60 {
        assert_eq!(clock.advance(fixed_delta), 1);
        ackermann_kinematics(&mut world, clock.fixed_delta());
    }
    let transform = world.get::<Transform3>(vehicle).unwrap();
    let drive = world.get::<AckermannDrive>(vehicle).unwrap();
    assert!(drive.speed_m_s > 2.4 && drive.speed_m_s < 2.6);
    assert!(transform.translation.length() > 1.0);
    assert_eq!(clock.sim_time().ticks(), fixed_delta.ticks() * 60);
}

#[test]
fn ackermann_rejects_non_finite_command_without_mutation() {
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "test_vehicle");
    world
        .entity_mut(vehicle)
        .insert((Transform3::default(), AckermannDrive::default()));
    let before = world.get::<AckermannDrive>(vehicle).unwrap().clone();
    assert_eq!(
        command_ackermann_drive(&mut world, vehicle, f64::NAN, 0.0),
        AckermannCommandResult::NonFiniteCommand
    );
    assert_eq!(world.get::<AckermannDrive>(vehicle).unwrap(), &before);
}

fn run_multirotor_replay() -> (Transform3, MultirotorFlight, f64, f64, f64, f64) {
    let mut world = World::new();
    let aircraft = spawn_named(&mut world, "showcase_uav");
    world.entity_mut(aircraft).insert((
        Transform3 {
            translation: Vec3::new(-18.0, 8.0, 12.0),
            ..Transform3::IDENTITY
        },
        MultirotorFlight::default(),
        RigidBody::default(),
    ));
    assert_eq!(
        command_multirotor(&mut world, aircraft, Vec3::new(22.0, 14.0, -16.0), 1.1,),
        MultirotorCommandResult::Applied
    );

    let dt = SimDuration::from_seconds(Seconds::new(1.0 / 60.0));
    let mut maximum_speed_m_s: f64 = 0.0;
    let mut maximum_acceleration_m_s2: f64 = 0.0;
    let mut maximum_tilt_rad: f64 = 0.0;
    let mut maximum_yaw_rate_rad_s: f64 = 0.0;
    for _ in 0..720 {
        multirotor_flight(&mut world, dt);
        let flight = world.get::<MultirotorFlight>(aircraft).unwrap();
        let transform = world.get::<Transform3>(aircraft).unwrap();
        maximum_speed_m_s = maximum_speed_m_s.max(flight.velocity_m_s.length());
        maximum_acceleration_m_s2 =
            maximum_acceleration_m_s2.max(flight.commanded_acceleration_m_s2.length());
        let body_up = transform.rotation * Vec3::Y;
        maximum_tilt_rad = maximum_tilt_rad.max(body_up.dot(Vec3::Y).clamp(-1.0, 1.0).acos());
        maximum_yaw_rate_rad_s = maximum_yaw_rate_rad_s.max(
            world
                .get::<RigidBody>(aircraft)
                .unwrap()
                .angular_velocity_rad_s
                .y
                .abs(),
        );
    }
    (
        *world.get::<Transform3>(aircraft).unwrap(),
        *world.get::<MultirotorFlight>(aircraft).unwrap(),
        maximum_speed_m_s,
        maximum_acceleration_m_s2,
        maximum_tilt_rad,
        maximum_yaw_rate_rad_s,
    )
}

#[test]
fn multirotor_tracks_target_with_bounded_flight_state() {
    let (
        transform,
        flight,
        maximum_speed_m_s,
        maximum_acceleration_m_s2,
        maximum_tilt_rad,
        maximum_yaw_rate_rad_s,
    ) = run_multirotor_replay();
    let error_m = (transform.translation - flight.target_position_m).length();
    assert!(error_m < 0.15, "position error was {error_m:.3} m");
    assert!(
        maximum_speed_m_s
            <= flight
                .max_horizontal_speed_m_s
                .hypot(flight.max_climb_speed_m_s)
                + 1.0e-9
    );
    assert!(maximum_acceleration_m_s2 <= flight.max_acceleration_m_s2 + 1.0e-9);
    assert!(maximum_tilt_rad <= flight.max_tilt_rad + 1.0e-6);
    assert!(maximum_yaw_rate_rad_s <= flight.max_yaw_rate_rad_s + 1.0e-9);
    assert!(wrap_angle_rad(flight.yaw_rad - flight.target_yaw_rad).abs() < 1.0e-6);
}

#[test]
fn multirotor_replay_is_exactly_deterministic() {
    assert_eq!(run_multirotor_replay(), run_multirotor_replay());
}

#[test]
fn multirotor_rejects_non_finite_command_without_mutation() {
    let mut world = World::new();
    let aircraft = spawn_named(&mut world, "showcase_uav");
    world
        .entity_mut(aircraft)
        .insert((Transform3::IDENTITY, MultirotorFlight::default()));
    let before = *world.get::<MultirotorFlight>(aircraft).unwrap();
    assert_eq!(
        command_multirotor(&mut world, aircraft, Vec3::new(f64::NAN, 2.0, 3.0), 0.0),
        MultirotorCommandResult::NonFiniteCommand
    );
    assert_eq!(*world.get::<MultirotorFlight>(aircraft).unwrap(), before);
}

#[test]
fn invalid_multirotor_configuration_is_transactional() {
    let mut world = World::new();
    let aircraft = spawn_named(&mut world, "showcase_uav");
    let flight = MultirotorFlight {
        max_tilt_rad: std::f64::consts::PI,
        ..MultirotorFlight::default()
    };
    let transform = Transform3 {
        translation: Vec3::new(1.0, 2.0, 3.0),
        ..Transform3::IDENTITY
    };
    world.entity_mut(aircraft).insert((transform, flight));
    multirotor_flight(
        &mut world,
        SimDuration::from_seconds(Seconds::new(1.0 / 60.0)),
    );
    assert_eq!(*world.get::<Transform3>(aircraft).unwrap(), transform);
    assert_eq!(*world.get::<MultirotorFlight>(aircraft).unwrap(), flight);
}

#[test]
fn pure_pursuit_steers_toward_lateral_target() {
    let transform = Transform3::default();
    let steering = pure_pursuit_steering(&transform, Vec3::new(5.0, 0.0, 2.0), 2.7, 5.0);
    assert!(steering < 0.0);
}

fn spawn_dynamic_vehicle(
    world: &mut World,
    drive: AckermannDrive,
    dynamics: VehicleDynamics,
) -> Entity {
    let vehicle = world.spawn_empty().id();
    world
        .entity_mut(vehicle)
        .insert((drive, dynamics, Transform3::IDENTITY, RigidBody::default()));
    vehicle
}

fn hot_lap_drive(speed_m_s: f64, steering_rad: f64) -> AckermannDrive {
    AckermannDrive {
        max_speed_m_s: 60.0,
        max_acceleration_m_s2: 1_000.0,
        max_deceleration_m_s2: 1_000.0,
        max_steering_rate_rad_s: 1_000.0,
        speed_m_s,
        target_speed_m_s: speed_m_s,
        steering_rad,
        target_steering_rad: steering_rad,
        ..AckermannDrive::default()
    }
}

fn step_seconds(world: &mut World, seconds: f64) {
    let dt = SimDuration::from_seconds(rne_math::Seconds::new(1.0 / 240.0));
    for _ in 0..(seconds * 240.0) as usize {
        vehicle_dynamics(world, dt);
    }
}

#[test]
fn dynamic_model_matches_kinematics_at_low_speed() {
    // 1.5 m/s is inside the blend region, so the no-slip solution applies.
    let speed = 1.5;
    let steering = 0.3;

    let mut dynamic_world = World::new();
    let vehicle = spawn_dynamic_vehicle(
        &mut dynamic_world,
        hot_lap_drive(speed, steering),
        VehicleDynamics::default(),
    );
    step_seconds(&mut dynamic_world, 2.0);

    let mut kinematic_world = World::new();
    let reference = kinematic_world.spawn_empty().id();
    kinematic_world.entity_mut(reference).insert((
        hot_lap_drive(speed, steering),
        Transform3::IDENTITY,
        RigidBody::default(),
    ));
    let dt = SimDuration::from_seconds(rne_math::Seconds::new(1.0 / 240.0));
    for _ in 0..480 {
        ackermann_kinematics(&mut kinematic_world, dt);
    }

    let dynamic_transform = *dynamic_world.get::<Transform3>(vehicle).unwrap();
    let kinematic_transform = *kinematic_world.get::<Transform3>(reference).unwrap();

    // Headings must agree: the blend takes the no-slip yaw rate exactly.
    let dynamic_forward = dynamic_transform.rotation * Vec3::X;
    let kinematic_forward = kinematic_transform.rotation * Vec3::X;
    assert!(dynamic_forward.dot(kinematic_forward) > 0.999_999);

    // The two models track different chassis points — the dynamic model follows the
    // center of mass, the kinematic one its reference axle — so their paths differ
    // laterally by at most the CG offset times the accumulated yaw.
    let total_yaw = 1.5 / VehicleDynamics::default().wheelbase_m() * 0.3_f64.tan() * 2.0;
    let bound = VehicleDynamics::default().rear_axle_m * total_yaw + 0.05;
    let divergence = (dynamic_transform.translation - kinematic_transform.translation).length();
    assert!(
        divergence < bound,
        "low-speed divergence {divergence:.3} m exceeds the CG-offset bound {bound:.3} m"
    );
}

#[test]
fn tire_slip_widens_the_line_as_speed_rises() {
    // Identical steering at rising speeds; the no-slip model would keep the turn
    // radius constant, tire slip must widen it. Gentle enough that neither axle
    // reaches the friction limit: the widening is pure slip, not saturation.
    let steering = 0.08;
    let radius_at = |speed: f64| {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(speed, steering),
            VehicleDynamics::default(),
        );
        step_seconds(&mut world, 6.0);
        let dynamics = world.get::<VehicleDynamics>(vehicle).unwrap();
        // Steady-state turn radius follows from speed over yaw rate.
        (speed / dynamics.yaw_rate_rad_s, *dynamics)
    };

    let (slow_radius, slow_dynamics) = radius_at(5.0);
    let (fast_radius, fast_dynamics) = radius_at(12.0);

    assert!(slow_radius > 0.0 && fast_radius > 0.0);
    assert!(
        fast_radius > slow_radius * 1.05,
        "line must widen with speed: {slow_radius:.2} m -> {fast_radius:.2} m"
    );
    // The widening comes from real slip angles, not from saturation.
    assert!(fast_dynamics.front_slip_rad.abs() > slow_dynamics.front_slip_rad.abs());
    assert!(!fast_dynamics.front_saturated);
}

#[test]
fn friction_limit_saturates_the_front_axle_and_understeers() {
    // A hard corner at speed exceeds mu Fz on the front axle.
    let mut world = World::new();
    let vehicle = spawn_dynamic_vehicle(
        &mut world,
        hot_lap_drive(24.0, 0.5),
        VehicleDynamics::default(),
    );
    step_seconds(&mut world, 4.0);

    let dynamics = *world.get::<VehicleDynamics>(vehicle).unwrap();
    assert!(dynamics.front_saturated, "front axle must saturate");

    // Saturated fronts cannot deliver the kinematic yaw rate: understeer.
    let kinematic_yaw = 24.0 / VehicleDynamics::default().wheelbase_m() * 0.5_f64.tan();
    assert!(
        dynamics.yaw_rate_rad_s < kinematic_yaw * 0.5,
        "yaw rate {:.3} should be far below the no-slip {:.3}",
        dynamics.yaw_rate_rad_s,
        kinematic_yaw
    );
}

#[test]
fn load_transfer_shifts_grip_between_axles() {
    let dynamics = VehicleDynamics::default();
    let total = dynamics.static_front_load_n() + dynamics.static_rear_load_n();
    assert!((total - dynamics.mass_kg * 9.81).abs() < 1e-9);
    // The default sedan is nose-heavy: more static load on the front axle.
    assert!(dynamics.static_front_load_n() > dynamics.static_rear_load_n());
}

#[test]
fn vehicle_dynamics_is_deterministic() {
    let run = || {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(18.0, 0.35),
            VehicleDynamics::default(),
        );
        step_seconds(&mut world, 5.0);
        (
            world.get::<Transform3>(vehicle).unwrap().translation,
            *world.get::<VehicleDynamics>(vehicle).unwrap(),
        )
    };

    assert_eq!(run(), run());
}

#[test]
fn cornering_stiffness_load_sensitivity_absent_returns_reference_unchanged() {
    // The absent-spec path must be bit-for-bit identical to the original
    // constant-stiffness formula: effective_cornering_stiffness must hand back
    // exactly the declared value, untouched, for every load -- not merely close.
    for axle_load_n in [0.0, 1.0, 2_500.0, 8_175.0, 1.0e6] {
        assert_eq!(
            effective_cornering_stiffness(80_000.0, axle_load_n, 8_175.0, None),
            80_000.0
        );
    }
}

#[test]
fn cornering_stiffness_load_sensitivity_absent_trajectory_is_bit_identical() {
    // Full-pipeline version of the same guarantee: a braking-and-turning
    // transient run with the spec absent must match, field for field, the same
    // transient with an explicit zero-gain spec present. Zero gain multiplies
    // stiffness by exactly 1.0 (IEEE-754 exact), so if this ever diverges the
    // new load-dependent term is leaking into the constant-stiffness path.
    let run = |sensitivity| {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            AckermannDrive {
                target_speed_m_s: 6.0,
                speed_m_s: 22.0,
                max_deceleration_m_s2: 5.0,
                max_acceleration_m_s2: 1_000.0,
                max_steering_rate_rad_s: 1_000.0,
                steering_rad: 0.1,
                target_steering_rad: 0.1,
                max_speed_m_s: 60.0,
                ..AckermannDrive::default()
            },
            VehicleDynamics {
                cornering_stiffness_load_sensitivity: sensitivity,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 3.0);
        let dynamics = *world.get::<VehicleDynamics>(vehicle).unwrap();
        // Compare every numerically meaningful output, but not the sensitivity
        // spec itself (which trivially differs between the two runs).
        (
            world.get::<Transform3>(vehicle).unwrap().translation,
            dynamics.lateral_velocity_m_s,
            dynamics.yaw_rate_rad_s,
            dynamics.front_slip_rad,
            dynamics.rear_slip_rad,
            dynamics.front_saturated,
            dynamics.rear_saturated,
        )
    };

    let absent = run(None);
    let zero_gain = run(Some(CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.0,
        maximum_load_ratio: 3.0,
    }));
    assert_eq!(absent, zero_gain);
}

#[test]
fn cornering_stiffness_at_reference_load_equals_declared_value_exactly() {
    let sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.4,
        maximum_load_ratio: 3.0,
    };
    // At the axle's own static load the ratio is exactly 1.0, so the declared
    // parameter keeps its current meaning bit-for-bit.
    assert_eq!(
        effective_cornering_stiffness(80_000.0, 8_175.0, 8_175.0, Some(sensitivity)),
        80_000.0
    );
}

#[test]
fn cornering_stiffness_rises_with_load_and_falls_when_unloaded() {
    let sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.3,
        maximum_load_ratio: 3.0,
    };
    let reference = 80_000.0;
    let static_load = 8_175.0;
    let loaded =
        effective_cornering_stiffness(reference, static_load * 1.4, static_load, Some(sensitivity));
    let unloaded =
        effective_cornering_stiffness(reference, static_load * 0.6, static_load, Some(sensitivity));
    assert!(loaded > reference, "loaded stiffness {loaded} must rise");
    assert!(
        unloaded < reference,
        "unloaded stiffness {unloaded} must fall"
    );
    assert!(loaded.is_finite() && loaded > 0.0);
    assert!(unloaded.is_finite() && unloaded > 0.0);
}

#[test]
fn cornering_stiffness_load_sensitivity_is_sub_linear_in_load_ratio() {
    let sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.5,
        maximum_load_ratio: 3.0,
    };
    let reference = 80_000.0;
    let static_load = 1_000.0;
    // load_ratio 1.2 -> 2.4 is exactly a doubling of the ratio.
    let at_1_2 = effective_cornering_stiffness(reference, 1_200.0, static_load, Some(sensitivity));
    let at_2_4 = effective_cornering_stiffness(reference, 2_400.0, static_load, Some(sensitivity));
    assert!(
        at_2_4 < 2.0 * at_1_2,
        "doubling the load ratio must not double stiffness: {at_1_2} -> {at_2_4}"
    );
}

#[test]
fn cornering_stiffness_load_sensitivity_validation_rejects_bad_parameters() {
    let valid = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.3,
        maximum_load_ratio: 3.0,
    };
    assert!(valid.is_valid());

    let non_finite_sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: f64::NAN,
        ..valid
    };
    assert!(!non_finite_sensitivity.is_valid());

    let negative_sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: -0.1,
        ..valid
    };
    assert!(!negative_sensitivity.is_valid());

    let unit_sensitivity = CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 1.0,
        ..valid
    };
    assert!(!unit_sensitivity.is_valid());

    let sub_unity_max_ratio = CorneringStiffnessLoadSensitivity {
        maximum_load_ratio: 0.5,
        ..valid
    };
    assert!(!sub_unity_max_ratio.is_valid());

    let infinite_max_ratio = CorneringStiffnessLoadSensitivity {
        maximum_load_ratio: f64::INFINITY,
        ..valid
    };
    assert!(!infinite_max_ratio.is_valid());

    // An otherwise-valid VehicleDynamics is invalidated by a bad nested spec.
    let dynamics = VehicleDynamics {
        cornering_stiffness_load_sensitivity: Some(non_finite_sensitivity),
        ..VehicleDynamics::default()
    };
    assert!(!dynamics.is_valid());
}

#[test]
fn cornering_stiffness_load_sensitivity_is_deterministic() {
    let sensitivity = Some(CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.35,
        maximum_load_ratio: 3.0,
    });
    let run = || {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            AckermannDrive {
                target_speed_m_s: 6.0,
                speed_m_s: 22.0,
                max_deceleration_m_s2: 5.0,
                max_acceleration_m_s2: 1_000.0,
                max_steering_rate_rad_s: 1_000.0,
                steering_rad: 0.1,
                target_steering_rad: 0.1,
                max_speed_m_s: 60.0,
                ..AckermannDrive::default()
            },
            VehicleDynamics {
                cornering_stiffness_load_sensitivity: sensitivity,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 3.0);
        (
            world.get::<Transform3>(vehicle).unwrap().translation,
            *world.get::<VehicleDynamics>(vehicle).unwrap(),
        )
    };

    assert_eq!(run(), run());
}

#[test]
fn load_dependent_stiffness_measurably_changes_yaw_response_under_braking() {
    // The test that fails if stiffness stayed constant: identical steering and
    // braking commands, spec absent vs. present. If front_stiffness_n_rad /
    // rear_stiffness_n_rad were silently ignored in favor of the constant
    // fields, these two runs would be bit-identical and the assertion below
    // would fail.
    let yaw_rate_after_braking_turn = |sensitivity| {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            AckermannDrive {
                target_speed_m_s: 4.0,
                speed_m_s: 22.0,
                max_deceleration_m_s2: 7.0,
                max_acceleration_m_s2: 1_000.0,
                max_steering_rate_rad_s: 1_000.0,
                steering_rad: 0.1,
                target_steering_rad: 0.1,
                max_speed_m_s: 60.0,
                ..AckermannDrive::default()
            },
            VehicleDynamics {
                cornering_stiffness_load_sensitivity: sensitivity,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 1.0);
        world
            .get::<VehicleDynamics>(vehicle)
            .unwrap()
            .yaw_rate_rad_s
    };

    let constant = yaw_rate_after_braking_turn(None);
    let load_dependent = yaw_rate_after_braking_turn(Some(CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.6,
        maximum_load_ratio: 3.0,
    }));

    let relative_difference = (load_dependent - constant).abs() / constant.abs();
    assert!(
        relative_difference > 0.01,
        "load-dependent stiffness must measurably change the yaw response: \
         constant={constant:.6} rad/s, load_dependent={load_dependent:.6} rad/s, \
         relative_difference={relative_difference:.6}"
    );
}

#[test]
fn zero_lateral_transfer_matches_the_single_tire_axle_bit_for_bit() {
    let sensitivity = Some(CorneringStiffnessLoadSensitivity {
        load_sensitivity_per_load_ratio: 0.4,
        maximum_load_ratio: 3.0,
    });
    let cases = [
        (80_000.0, 8_000.0, 8_000.0, 0.05, 0.9, None),
        (80_000.0, 8_000.0, 12_000.0, 0.2, 0.9, None),
        (80_000.0, 8_000.0, 5_000.0, -0.1, 1.1, None),
        (80_000.0, 8_000.0, 8_000.0, 0.05, 0.9, sensitivity),
        (80_000.0, 8_000.0, 12_000.0, 0.2, 0.9, sensitivity),
        (88_000.0, 9_000.0, 9_000.0, 0.0, 0.9, sensitivity),
    ];
    for (stiffness, static_load, load, slip, mu, sens) in cases {
        let axle_effective = effective_cornering_stiffness(stiffness, load, static_load, sens);
        let limit_n = mu * load;
        let expected_force_n = (-axle_effective * slip).clamp(-limit_n, limit_n);
        let expected_saturated = (axle_effective * slip).abs() > limit_n;

        let (force_n, saturated) =
            axle_lateral_force_n(stiffness, static_load, load, slip, mu, sens, 0.0);
        assert_eq!(
            force_n.to_bits(),
            expected_force_n.to_bits(),
            "split axle must reproduce the single tire exactly for slip {slip}"
        );
        assert_eq!(saturated, expected_saturated);
    }
}

#[test]
fn lateral_load_transfer_reduces_usable_axle_force() {
    // A moderate slip angle saturates the loaded side before the unloaded one,
    // which is exactly where the left/right split costs the axle grip.
    let (single_force_n, single_saturated) =
        axle_lateral_force_n(80_000.0, 8_000.0, 8_000.0, 0.15, 1.0, None, 0.0);
    let (split_force_n, split_saturated) =
        axle_lateral_force_n(80_000.0, 8_000.0, 8_000.0, 0.15, 1.0, None, 3_000.0);

    assert!(single_saturated && split_saturated);
    assert!(
        split_force_n.abs() < single_force_n.abs(),
        "load transfer must cost axle grip: single={single_force_n}, split={split_force_n}"
    );
    // Loaded side 7000 N saturates at mu*7000; unloaded side 1000 N is already
    // friction limited, so the axle carries 7000 N instead of 8000 N.
    assert!((split_force_n.abs() - 7_000.0).abs() < 1.0e-9);
}

#[test]
fn lateral_load_transfer_validation_rejects_bad_parameters() {
    let valid = LateralLoadTransferSpec {
        track_width_m: 1.6,
        front_roll_stiffness_fraction: 0.6,
    };
    assert!(valid.is_valid());

    assert!(!LateralLoadTransferSpec {
        track_width_m: 0.0,
        ..valid
    }
    .is_valid());
    assert!(!LateralLoadTransferSpec {
        track_width_m: f64::NAN,
        ..valid
    }
    .is_valid());
    assert!(!LateralLoadTransferSpec {
        front_roll_stiffness_fraction: -0.1,
        ..valid
    }
    .is_valid());
    assert!(!LateralLoadTransferSpec {
        front_roll_stiffness_fraction: 1.1,
        ..valid
    }
    .is_valid());

    let dynamics = VehicleDynamics {
        lateral_load_transfer: Some(LateralLoadTransferSpec {
            track_width_m: -1.0,
            ..valid
        }),
        ..VehicleDynamics::default()
    };
    assert!(!dynamics.is_valid());
}

#[test]
fn lateral_load_transfer_measurably_changes_yaw_response_in_a_steady_turn() {
    let yaw_rate = |transfer: Option<LateralLoadTransferSpec>| {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            AckermannDrive {
                target_speed_m_s: 25.0,
                speed_m_s: 25.0,
                max_acceleration_m_s2: 1_000.0,
                max_steering_rate_rad_s: 1_000.0,
                steering_rad: 0.18,
                target_steering_rad: 0.18,
                max_speed_m_s: 60.0,
                ..AckermannDrive::default()
            },
            VehicleDynamics {
                lateral_load_transfer: transfer,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 2.0);
        world
            .get::<VehicleDynamics>(vehicle)
            .unwrap()
            .yaw_rate_rad_s
    };

    let without = yaw_rate(None);
    let with = yaw_rate(Some(LateralLoadTransferSpec {
        track_width_m: 1.6,
        front_roll_stiffness_fraction: 0.6,
    }));

    let relative_difference = (with - without).abs() / without.abs();
    assert!(
        relative_difference > 0.01,
        "lateral load transfer must measurably change the yaw response: \
         without={without:.6} rad/s, with={with:.6} rad/s, \
         relative_difference={relative_difference:.6}"
    );
}

#[test]
fn lateral_load_transfer_is_deterministic() {
    let transfer = Some(LateralLoadTransferSpec {
        track_width_m: 1.55,
        front_roll_stiffness_fraction: 0.55,
    });
    let run = || {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(18.0, 0.12),
            VehicleDynamics {
                lateral_load_transfer: transfer,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 3.0);
        (
            world.get::<Transform3>(vehicle).unwrap().translation,
            *world.get::<VehicleDynamics>(vehicle).unwrap(),
        )
    };

    assert_eq!(run(), run());
}

fn four_wheel_spec() -> FourWheelVehicleSpec {
    FourWheelVehicleSpec {
        track_width_m: 1.6,
        front_roll_stiffness_fraction: 0.6,
        ackermann_fraction: 1.0,
    }
}

fn run_four_wheel(
    drive: AckermannDrive,
    four_wheel: Option<FourWheelVehicleSpec>,
    seconds: f64,
) -> VehicleDynamics {
    let mut world = World::new();
    let vehicle = spawn_dynamic_vehicle(
        &mut world,
        drive,
        VehicleDynamics {
            four_wheel,
            ..VehicleDynamics::default()
        },
    );
    step_seconds(&mut world, seconds);
    *world.get::<VehicleDynamics>(vehicle).unwrap()
}

#[test]
fn four_wheel_validation_rejects_bad_parameters() {
    let valid = four_wheel_spec();
    assert!(valid.is_valid());

    assert!(!FourWheelVehicleSpec {
        track_width_m: 0.0,
        ..valid
    }
    .is_valid());
    assert!(!FourWheelVehicleSpec {
        track_width_m: f64::NAN,
        ..valid
    }
    .is_valid());
    assert!(!FourWheelVehicleSpec {
        front_roll_stiffness_fraction: -0.1,
        ..valid
    }
    .is_valid());
    assert!(!FourWheelVehicleSpec {
        front_roll_stiffness_fraction: 1.1,
        ..valid
    }
    .is_valid());
    assert!(!FourWheelVehicleSpec {
        ackermann_fraction: -0.1,
        ..valid
    }
    .is_valid());
    assert!(!FourWheelVehicleSpec {
        ackermann_fraction: f64::NAN,
        ..valid
    }
    .is_valid());

    let dynamics = VehicleDynamics {
        four_wheel: Some(FourWheelVehicleSpec {
            track_width_m: -1.0,
            ..valid
        }),
        ..VehicleDynamics::default()
    };
    assert!(!dynamics.is_valid());
}

#[test]
fn four_wheel_ackermann_spreads_the_front_slip_angles() {
    let drive = || hot_lap_drive(15.0, 0.1);
    // The front-left/right slip spread comes only from the Ackermann geometry and
    // the `vx + r z` wheel-speed difference. Exact Ackermann must spread the front
    // slips more than parallel steering does.
    let front_spread = |ackermann_fraction: f64| {
        let dynamics = run_four_wheel(
            drive(),
            Some(FourWheelVehicleSpec {
                ackermann_fraction,
                ..four_wheel_spec()
            }),
            2.0,
        );
        (dynamics.wheel_slip_rad[1] - dynamics.wheel_slip_rad[0]).abs()
    };

    let parallel = front_spread(0.0);
    let ackermann = front_spread(1.0);
    assert!(
        ackermann > parallel * 1.5,
        "Ackermann steering must spread the front slips more than parallel: \
         parallel={parallel:.6}, ackermann={ackermann:.6}"
    );
}

#[test]
fn four_wheel_populates_per_wheel_telemetry_and_keeps_axle_means() {
    let dynamics = run_four_wheel(hot_lap_drive(15.0, 0.1), Some(four_wheel_spec()), 2.0);
    assert!(dynamics.wheel_slip_rad.iter().all(|slip| slip.is_finite()));
    assert!(dynamics.wheel_slip_rad != [0.0; 4]);
    // The axle slip fields are the per-wheel mean, not an independent value.
    assert_eq!(
        dynamics.front_slip_rad,
        0.5 * (dynamics.wheel_slip_rad[0] + dynamics.wheel_slip_rad[1])
    );
    assert_eq!(
        dynamics.rear_slip_rad,
        0.5 * (dynamics.wheel_slip_rad[2] + dynamics.wheel_slip_rad[3])
    );
}

#[test]
fn four_wheel_measurably_changes_yaw_response_versus_single_track() {
    let yaw_rate = |four_wheel: Option<FourWheelVehicleSpec>| {
        run_four_wheel(hot_lap_drive(20.0, 0.15), four_wheel, 2.0).yaw_rate_rad_s
    };

    let single_track = yaw_rate(None);
    let four_wheel = yaw_rate(Some(four_wheel_spec()));

    let relative_difference = (four_wheel - single_track).abs() / single_track.abs();
    assert!(
        relative_difference > 0.01,
        "the four-wheel model must measurably change the yaw response: \
         single_track={single_track:.6} rad/s, four_wheel={four_wheel:.6} rad/s, \
         relative_difference={relative_difference:.6}"
    );
}

#[test]
fn four_wheel_is_deterministic() {
    let run = || {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            hot_lap_drive(18.0, 0.12),
            VehicleDynamics {
                four_wheel: Some(four_wheel_spec()),
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, 3.0);
        (
            world.get::<Transform3>(vehicle).unwrap().translation,
            *world.get::<VehicleDynamics>(vehicle).unwrap(),
        )
    };

    assert_eq!(run(), run());
}

#[test]
fn steering_lag_delays_the_response_and_zero_lag_matches_legacy() {
    let steering_after = |lag_s: f64, seconds: f64| {
        let mut world = World::new();
        let vehicle = spawn_dynamic_vehicle(
            &mut world,
            AckermannDrive {
                target_steering_rad: 0.3,
                speed_m_s: 10.0,
                target_speed_m_s: 10.0,
                max_speed_m_s: 30.0,
                // High enough that the rate limit never binds: this test isolates
                // the first-order lag. Their composition is covered implicitly by
                // every other dynamic-model test using the default rate.
                max_steering_rate_rad_s: 100.0,
                ..AckermannDrive::default()
            },
            VehicleDynamics {
                steering_lag_s: lag_s,
                ..VehicleDynamics::default()
            },
        );
        step_seconds(&mut world, seconds);
        world.get::<AckermannDrive>(vehicle).unwrap().steering_rad
    };

    // Without lag the rate limit alone reaches the target quickly.
    let instant = steering_after(0.0, 0.5);
    assert!((instant - 0.3).abs() < 1e-9);
    // One time constant reaches ~63 percent of the step.
    let lagged = steering_after(0.2, 0.2);
    assert!((lagged - 0.3 * 0.632).abs() < 0.01, "got {lagged}");
    // The lag converges eventually.
    assert!((steering_after(0.2, 2.0) - 0.3).abs() < 1e-3);
}

#[test]
fn rigid_body_velocity_includes_the_lateral_component() {
    let mut world = World::new();
    let vehicle = spawn_dynamic_vehicle(
        &mut world,
        hot_lap_drive(12.0, 0.08),
        VehicleDynamics::default(),
    );
    step_seconds(&mut world, 3.0);

    let dynamics = *world.get::<VehicleDynamics>(vehicle).unwrap();
    let transform = *world.get::<Transform3>(vehicle).unwrap();
    let body = world.get::<RigidBody>(vehicle).unwrap();

    // Velocity is not aligned with the nose: the slip is visible in the world state,
    // which is what a mounted IMU or wheel-speed sensor would observe. The velocity
    // uses the mid-step attitude, so the comparison allows the half-step of yaw.
    let forward = transform.rotation * Vec3::X;
    let along = body.linear_velocity_m_s.dot(forward);
    let across = (body.linear_velocity_m_s - forward * along).length();
    assert!(dynamics.lateral_velocity_m_s.abs() > 0.01);
    assert!((across - dynamics.lateral_velocity_m_s.abs()).abs() < 0.05);
}

fn test_patch(wheel_entity: Entity, velocity_m_s: Vec3, load_n: f64) -> WheelContactPatch {
    WheelContactPatch {
        wheel_entity,
        point_world_m: Vec3::new(0.0, 0.0, 0.0),
        normal_road_to_wheel_world: Vec3::Y,
        wheel_relative_to_road_world_m_s: velocity_m_s,
        normal_load_n: load_n,
    }
}

fn test_tire_input(
    patch: Option<WheelContactPatch>,
    wheel_circumferential_speed_m_s: f64,
    road_friction_scale: f64,
) -> CombinedSlipTireInput {
    CombinedSlipTireInput {
        patch,
        forward_world: Vec3::X,
        lateral_world: Vec3::Z,
        wheel_circumferential_speed_m_s,
        road_friction_scale,
    }
}

#[test]
fn contact_patch_normalizes_canonical_entity_orientation() {
    let mut world = World::new();
    let road = world.spawn_empty().id();
    let wheel = world.spawn_empty().id();
    let samples = [
        ContactPointSample {
            entity_a: road,
            entity_b: wheel,
            point_world_m: Vec3::new(-0.1, 0.0, 0.0),
            normal_a_to_b: Vec3::Y,
            velocity_b_relative_to_a_world_m_s: Vec3::new(-2.0, 0.0, 0.5),
            normal_force_n: 300.0,
        },
        ContactPointSample {
            entity_a: road,
            entity_b: wheel,
            point_world_m: Vec3::new(0.1, 0.0, 0.0),
            normal_a_to_b: Vec3::Y,
            velocity_b_relative_to_a_world_m_s: Vec3::new(-1.0, 0.0, 0.5),
            normal_force_n: 100.0,
        },
    ];
    let patch = aggregate_wheel_contact_patch(wheel, &samples, Vec3::X, Vec3::Z)
        .unwrap()
        .unwrap();
    assert_eq!(patch.normal_load_n, 400.0);
    assert_eq!(patch.point_world_m, Vec3::new(-0.05, 0.0, 0.0));
    assert_eq!(patch.normal_road_to_wheel_world, Vec3::Y);
    assert_eq!(
        patch.wheel_relative_to_road_world_m_s,
        Vec3::new(-1.75, 0.0, 0.5)
    );

    let inverted = [ContactPointSample {
        entity_a: wheel,
        entity_b: road,
        point_world_m: Vec3::ZERO,
        normal_a_to_b: Vec3::NEG_Y,
        velocity_b_relative_to_a_world_m_s: Vec3::new(2.0, 0.0, -0.5),
        normal_force_n: 400.0,
    }];
    let inverted_patch = aggregate_wheel_contact_patch(wheel, &inverted, Vec3::X, Vec3::Z)
        .unwrap()
        .unwrap();
    assert_eq!(inverted_patch.normal_road_to_wheel_world, Vec3::Y);
    assert_eq!(
        inverted_patch.wheel_relative_to_road_world_m_s,
        Vec3::new(-2.0, 0.0, 0.5)
    );
}

#[test]
fn combined_slip_force_has_physical_sign_and_bounded_ellipse() {
    let mut world = World::new();
    let wheel = world.spawn_empty().id();
    let spec = CombinedSlipTireSpec {
        longitudinal_relaxation_length_m: 0.0,
        lateral_relaxation_length_m: 0.0,
        ..CombinedSlipTireSpec::default()
    };
    let evaluation = evaluate_combined_slip_tire(
        spec,
        CombinedSlipTireState::default(),
        test_tire_input(
            Some(test_patch(wheel, Vec3::new(-5.0, 0.0, 3.0), 1_000.0)),
            10.0,
            1.0,
        ),
        0.01,
    )
    .unwrap();
    assert!(evaluation.longitudinal_force_n > 0.0);
    assert!(evaluation.lateral_force_n < 0.0);
    assert!(evaluation.friction_utilization <= 1.0);
    let ellipse = (evaluation.longitudinal_force_n / evaluation.longitudinal_peak_force_n)
        .hypot(evaluation.lateral_force_n / evaluation.lateral_peak_force_n);
    assert!(ellipse <= 1.0);

    let repeat = evaluate_combined_slip_tire(
        spec,
        CombinedSlipTireState::default(),
        test_tire_input(
            Some(test_patch(wheel, Vec3::new(-5.0, 0.0, 3.0), 1_000.0)),
            10.0,
            1.0,
        ),
        0.01,
    )
    .unwrap();
    assert_eq!(evaluation, repeat);
}

fn tire_identification_spec() -> TireIdentificationSpec {
    TireIdentificationSpec {
        longitudinal_stiffness_bounds_n: [6_000.0, 10_000.0],
        lateral_stiffness_bounds_n: [5_000.0, 9_000.0],
        longitudinal_peak_friction_bounds: [0.5, 1.3],
        lateral_peak_friction_bounds: [0.4, 1.2],
        pure_slip_tolerance: 0.001,
        minimum_excited_slip: 0.02,
        maximum_linear_slip: 0.06,
        minimum_peak_slip: 0.4,
        maximum_abs_slip: 1.0,
        minimum_training_samples_per_axis: 6,
        minimum_combined_holdout_samples: 6,
        minimum_holdout_samples_per_condition: 3,
        grid_points_per_axis: 9,
        refinement_passes: 3,
        maximum_training_rms_n: 1.0e-8,
        maximum_holdout_rms_n: 1.0e-8,
        maximum_worst_condition_rms_n: 1.0e-8,
    }
}

fn identified_tire_template() -> CombinedSlipTireSpec {
    CombinedSlipTireSpec {
        longitudinal_stiffness_n: 8_000.0,
        lateral_stiffness_n: 7_000.0,
        longitudinal_peak_friction: 0.9,
        lateral_peak_friction: 0.8,
        ..CombinedSlipTireSpec::default()
    }
}

fn tire_sample(
    time_index: usize,
    longitudinal_slip_ratio: f64,
    lateral_slip_tangent: f64,
    normal_load_n: f64,
    road_friction_scale: f64,
) -> TireForceIdentificationSample {
    let (longitudinal_force_n, lateral_force_n) = steady_tire_forces(
        identified_tire_template(),
        longitudinal_slip_ratio,
        lateral_slip_tangent,
        normal_load_n,
        road_friction_scale,
    );
    TireForceIdentificationSample {
        capture_time_s: time_index as f64 * 0.01,
        longitudinal_slip_ratio,
        lateral_slip_tangent,
        normal_load_n,
        longitudinal_force_n,
        lateral_force_n,
    }
}

fn pure_tire_training_samples() -> Vec<TireForceIdentificationSample> {
    let slips = [-0.6, -0.2, -0.05, 0.05, 0.2, 0.6];
    slips
        .into_iter()
        .map(|slip| (slip, 0.0))
        .chain(slips.into_iter().map(|slip| (0.0, slip)))
        .enumerate()
        .map(|(index, (longitudinal, lateral))| {
            tire_sample(index, longitudinal, lateral, 1_000.0, 1.0)
        })
        .collect()
}

fn combined_tire_holdout_samples(road_friction_scale: f64) -> Vec<TireForceIdentificationSample> {
    [(-0.35, 0.20), (0.25, 0.30), (0.45, -0.25)]
        .into_iter()
        .enumerate()
        .map(|(index, (longitudinal, lateral))| {
            tire_sample(
                index,
                longitudinal,
                lateral,
                800.0 + index as f64 * 200.0,
                road_friction_scale,
            )
        })
        .collect()
}

#[test]
fn tire_identification_fits_pure_slip_and_holds_out_combined_conditions() {
    let training = pure_tire_training_samples();
    let dry = combined_tire_holdout_samples(1.0);
    let low_friction = combined_tire_holdout_samples(0.6);
    let result = identify_combined_slip_tire_steady(
        tire_identification_spec(),
        CombinedSlipTireSpec::default(),
        &[TireIdentificationRun {
            acquisition_id: 1,
            condition_id: 10,
            road_friction_scale: 1.0,
            samples: &training,
        }],
        &[
            TireIdentificationRun {
                acquisition_id: 2,
                condition_id: 20,
                road_friction_scale: 1.0,
                samples: &dry,
            },
            TireIdentificationRun {
                acquisition_id: 3,
                condition_id: 30,
                road_friction_scale: 0.6,
                samples: &low_friction,
            },
        ],
    )
    .unwrap();
    let repeat = identify_combined_slip_tire_steady(
        tire_identification_spec(),
        CombinedSlipTireSpec::default(),
        &[TireIdentificationRun {
            acquisition_id: 1,
            condition_id: 10,
            road_friction_scale: 1.0,
            samples: &training,
        }],
        &[
            TireIdentificationRun {
                acquisition_id: 2,
                condition_id: 20,
                road_friction_scale: 1.0,
                samples: &dry,
            },
            TireIdentificationRun {
                acquisition_id: 3,
                condition_id: 30,
                road_friction_scale: 0.6,
                samples: &low_friction,
            },
        ],
    )
    .unwrap();

    assert_eq!(result.tire_spec, identified_tire_template());
    assert_eq!(result, repeat);
    assert_eq!(result.longitudinal_training_sample_count, 6);
    assert_eq!(result.lateral_training_sample_count, 6);
    assert!(result.training_rms_n < 1.0e-10);
    assert!(result.holdout_rms_n < 1.0e-10);
    assert_eq!(
        result
            .condition_residuals
            .iter()
            .map(|condition| condition.condition_id)
            .collect::<Vec<_>>(),
        [20, 30]
    );
}

#[test]
fn steady_tire_force_api_matches_identification_law_and_rejects_invalid_input() {
    let spec = identified_tire_template();
    let expected = steady_tire_forces(spec, 0.2, -0.3, 900.0, 0.7);
    assert_eq!(
        evaluate_combined_slip_tire_steady_force(spec, 0.2, -0.3, 900.0, 0.7),
        Ok(expected)
    );
    assert_eq!(
        evaluate_combined_slip_tire_steady_force(spec, 0.2, -0.3, 0.0, 0.7),
        Err(MobilityPlantEvaluationError::InvalidInput)
    );
}

#[test]
fn tire_identification_rejects_split_overlap_and_holdout_degradation() {
    let training = pure_tire_training_samples();
    let dry = combined_tire_holdout_samples(1.0);
    let low_friction = combined_tire_holdout_samples(0.6);
    let training_run = TireIdentificationRun {
        acquisition_id: 1,
        condition_id: 10,
        road_friction_scale: 1.0,
        samples: &training,
    };
    let duplicate = TireIdentificationRun {
        acquisition_id: 1,
        condition_id: 20,
        road_friction_scale: 1.0,
        samples: &dry,
    };
    assert_eq!(
        identify_combined_slip_tire_steady(
            tire_identification_spec(),
            CombinedSlipTireSpec::default(),
            &[training_run],
            &[duplicate],
        ),
        Err(TireIdentificationError::DuplicateAcquisition)
    );

    let mut corrupted = low_friction;
    corrupted[1].lateral_force_n += 100.0;
    let error = identify_combined_slip_tire_steady(
        tire_identification_spec(),
        CombinedSlipTireSpec::default(),
        &[training_run],
        &[
            TireIdentificationRun {
                acquisition_id: 2,
                condition_id: 20,
                road_friction_scale: 1.0,
                samples: &dry,
            },
            TireIdentificationRun {
                acquisition_id: 3,
                condition_id: 30,
                road_friction_scale: 0.6,
                samples: &corrupted,
            },
        ],
    )
    .unwrap_err();
    assert_eq!(error, TireIdentificationError::ResidualExceeded);
}

fn tire_load_sensitivity_spec() -> TireLoadSensitivityIdentificationSpec {
    TireLoadSensitivityIdentificationSpec {
        load_sensitivity_bounds_per_load_ratio: [0.0, 0.4],
        minimum_combined_axis_slip: 0.15,
        maximum_abs_slip: 1.0,
        minimum_training_load_ratio_span: 0.8,
        minimum_training_samples: 9,
        minimum_holdout_samples: 6,
        minimum_holdout_samples_per_condition: 3,
        grid_points: 17,
        refinement_passes: 4,
        maximum_training_rms_n: 1.0e-8,
        maximum_holdout_rms_n: 1.0e-8,
        maximum_worst_condition_rms_n: 1.0e-8,
    }
}

fn load_sensitivity_samples(
    loads_n: &[f64],
    road_friction_scale: f64,
) -> Vec<TireForceIdentificationSample> {
    let true_tire = CombinedSlipTireSpec {
        load_sensitivity_per_load_ratio: 0.2,
        ..identified_tire_template()
    };
    loads_n
        .iter()
        .flat_map(|load_n| {
            [(0.50, 0.25), (-0.45, 0.30), (0.35, -0.40)]
                .into_iter()
                .map(move |slip| (*load_n, slip))
        })
        .enumerate()
        .map(|(index, (load_n, (longitudinal, lateral)))| {
            let (longitudinal_force_n, lateral_force_n) = steady_tire_forces(
                true_tire,
                longitudinal,
                lateral,
                load_n,
                road_friction_scale,
            );
            TireForceIdentificationSample {
                capture_time_s: index as f64 * 0.01,
                longitudinal_slip_ratio: longitudinal,
                lateral_slip_tangent: lateral,
                normal_load_n: load_n,
                longitudinal_force_n,
                lateral_force_n,
            }
        })
        .collect()
}

#[test]
fn tire_load_sensitivity_fit_brackets_reference_load_and_holds_out_conditions() {
    let training = load_sensitivity_samples(&[600.0, 1_000.0, 1_600.0], 1.0);
    let dry_holdout = load_sensitivity_samples(&[800.0], 1.0);
    let wet_holdout = load_sensitivity_samples(&[1_400.0], 0.7);
    let frozen = identified_tire_template();
    let result = identify_tire_load_sensitivity(
        tire_load_sensitivity_spec(),
        frozen,
        &[TireIdentificationRun {
            acquisition_id: 40,
            condition_id: 400,
            road_friction_scale: 1.0,
            samples: &training,
        }],
        &[
            TireIdentificationRun {
                acquisition_id: 41,
                condition_id: 410,
                road_friction_scale: 1.0,
                samples: &dry_holdout,
            },
            TireIdentificationRun {
                acquisition_id: 42,
                condition_id: 420,
                road_friction_scale: 0.7,
                samples: &wet_holdout,
            },
        ],
    )
    .unwrap();

    assert_eq!(result.load_sensitivity_per_load_ratio, 0.2);
    assert_eq!(
        result.tire_spec,
        CombinedSlipTireSpec {
            load_sensitivity_per_load_ratio: 0.2,
            ..frozen
        }
    );
    assert_eq!(result.training_sample_count, 9);
    assert_eq!(result.holdout_sample_count, 6);
    assert_eq!(result.minimum_training_load_ratio, 0.6);
    assert_eq!(result.maximum_training_load_ratio, 1.6);
    assert!(result.training_rms_n < 1.0e-10);
    assert!(result.holdout_rms_n < 1.0e-10);
    assert_eq!(
        result
            .condition_residuals
            .iter()
            .map(|residual| residual.condition_id)
            .collect::<Vec<_>>(),
        [410, 420]
    );
}

#[test]
fn tire_load_sensitivity_rejects_unbracketed_training_and_degraded_holdout() {
    let unbracketed = load_sensitivity_samples(&[1_200.0, 1_400.0, 1_600.0], 1.0);
    let dry_holdout = load_sensitivity_samples(&[800.0], 1.0);
    let wet_holdout = load_sensitivity_samples(&[1_400.0], 0.7);
    let frozen = identified_tire_template();
    let run = |samples| TireIdentificationRun {
        acquisition_id: 50,
        condition_id: 500,
        road_friction_scale: 1.0,
        samples,
    };
    assert_eq!(
        identify_tire_load_sensitivity(
            tire_load_sensitivity_spec(),
            frozen,
            &[run(&unbracketed)],
            &[
                TireIdentificationRun {
                    acquisition_id: 51,
                    condition_id: 510,
                    road_friction_scale: 1.0,
                    samples: &dry_holdout,
                },
                TireIdentificationRun {
                    acquisition_id: 52,
                    condition_id: 520,
                    road_friction_scale: 0.7,
                    samples: &wet_holdout,
                },
            ],
        ),
        Err(TireIdentificationError::InsufficientExcitation)
    );

    let training = load_sensitivity_samples(&[600.0, 1_000.0, 1_600.0], 1.0);
    let mut degraded = wet_holdout;
    degraded[1].lateral_force_n += 50.0;
    assert_eq!(
        identify_tire_load_sensitivity(
            tire_load_sensitivity_spec(),
            frozen,
            &[run(&training)],
            &[
                TireIdentificationRun {
                    acquisition_id: 51,
                    condition_id: 510,
                    road_friction_scale: 1.0,
                    samples: &dry_holdout,
                },
                TireIdentificationRun {
                    acquisition_id: 52,
                    condition_id: 520,
                    road_friction_scale: 0.7,
                    samples: &degraded,
                },
            ],
        ),
        Err(TireIdentificationError::ResidualExceeded)
    );
}

#[test]
fn tire_identification_rejects_underpopulated_holdout_condition() {
    let training = pure_tire_training_samples();
    let dry = combined_tire_holdout_samples(1.0);
    let low_friction = combined_tire_holdout_samples(0.6);
    let error = identify_combined_slip_tire_steady(
        tire_identification_spec(),
        CombinedSlipTireSpec::default(),
        &[TireIdentificationRun {
            acquisition_id: 1,
            condition_id: 10,
            road_friction_scale: 1.0,
            samples: &training,
        }],
        &[
            TireIdentificationRun {
                acquisition_id: 2,
                condition_id: 20,
                road_friction_scale: 1.0,
                samples: &dry,
            },
            TireIdentificationRun {
                acquisition_id: 3,
                condition_id: 30,
                road_friction_scale: 0.6,
                samples: &low_friction[..2],
            },
        ],
    )
    .unwrap_err();

    assert_eq!(error, TireIdentificationError::InsufficientExcitation);
}

fn tire_relaxation_identification_spec() -> TireRelaxationIdentificationSpec {
    TireRelaxationIdentificationSpec {
        relaxation_length_bounds_m: [0.10, 0.60],
        minimum_transport_speed_m_s: 1.0,
        minimum_slip_excitation: 0.01,
        maximum_abs_slip: 1.0,
        maximum_force_utilization: 0.95,
        minimum_training_transitions: 40,
        minimum_holdout_transitions: 80,
        minimum_holdout_transitions_per_condition: 40,
        grid_points: 11,
        refinement_passes: 3,
        maximum_training_rms_slip: 1.0e-12,
        maximum_holdout_rms_slip: 1.0e-12,
        maximum_worst_condition_rms_slip: 1.0e-12,
    }
}

fn tire_relaxation_run(
    tire: CombinedSlipTireSpec,
    axis: TireRelaxationAxis,
    relaxation_length_m: f64,
    phase: usize,
) -> Vec<TireRelaxationIdentificationSample> {
    let targets = [0.15, -0.12, 0.08, -0.15];
    let dt_s = 0.01;
    let mut relaxed_slip = 0.0;
    (0..240)
        .map(|index| {
            let target_slip = targets[((index / 30) + phase) % targets.len()];
            let transport_speed_m_s = 4.0 + (index % 17) as f64 * 0.02;
            let (longitudinal_slip, lateral_slip) = match axis {
                TireRelaxationAxis::Longitudinal => (relaxed_slip, 0.0),
                TireRelaxationAxis::Lateral => (0.0, relaxed_slip),
            };
            let forces = steady_tire_forces(tire, longitudinal_slip, lateral_slip, 1_000.0, 1.0);
            let sample = TireRelaxationIdentificationSample {
                capture_time_s: index as f64 * dt_s,
                transport_speed_m_s,
                target_slip,
                normal_load_n: 1_000.0,
                road_friction_scale: 1.0,
                measured_force_n: match axis {
                    TireRelaxationAxis::Longitudinal => forces.0,
                    TireRelaxationAxis::Lateral => forces.1,
                },
            };
            relaxed_slip = relax_slip(
                relaxed_slip,
                target_slip,
                relaxation_length_m,
                transport_speed_m_s,
                dt_s,
            );
            sample
        })
        .collect()
}

#[test]
fn tire_relaxation_identification_recovers_length_and_holds_out_conditions() {
    let tire = identified_tire_template();
    let axis = TireRelaxationAxis::Longitudinal;
    let training = tire_relaxation_run(tire, axis, 0.35, 0);
    let holdout_a = tire_relaxation_run(tire, axis, 0.35, 1);
    let holdout_b = tire_relaxation_run(tire, axis, 0.35, 2);
    let identify = || {
        identify_tire_relaxation_length(
            tire_relaxation_identification_spec(),
            tire,
            axis,
            &[TireRelaxationIdentificationRun {
                acquisition_id: 1,
                condition_id: 10,
                samples: &training,
            }],
            &[
                TireRelaxationIdentificationRun {
                    acquisition_id: 2,
                    condition_id: 20,
                    samples: &holdout_a,
                },
                TireRelaxationIdentificationRun {
                    acquisition_id: 3,
                    condition_id: 30,
                    samples: &holdout_b,
                },
            ],
        )
        .unwrap()
    };
    let first = identify();
    let second = identify();
    assert_eq!(first, second);
    assert!((first.relaxation_length_m - 0.35).abs() < 1.0e-12);
    assert!(first.training_rms_slip < 1.0e-14);
    assert!(first.holdout_rms_slip < 1.0e-14);
    assert_eq!(first.condition_residuals.len(), 2);

    let axis = TireRelaxationAxis::Lateral;
    let lateral_training = tire_relaxation_run(tire, axis, 0.35, 0);
    let lateral_holdout_a = tire_relaxation_run(tire, axis, 0.35, 1);
    let lateral_holdout_b = tire_relaxation_run(tire, axis, 0.35, 2);
    let lateral = identify_tire_relaxation_length(
        tire_relaxation_identification_spec(),
        tire,
        axis,
        &[TireRelaxationIdentificationRun {
            acquisition_id: 11,
            condition_id: 10,
            samples: &lateral_training,
        }],
        &[
            TireRelaxationIdentificationRun {
                acquisition_id: 12,
                condition_id: 20,
                samples: &lateral_holdout_a,
            },
            TireRelaxationIdentificationRun {
                acquisition_id: 13,
                condition_id: 30,
                samples: &lateral_holdout_b,
            },
        ],
    )
    .unwrap();
    assert!((lateral.relaxation_length_m - 0.35).abs() < 1.0e-12);
}

#[test]
fn tire_relaxation_identification_rejects_overlap_clock_and_holdout_drift() {
    let tire = identified_tire_template();
    let axis = TireRelaxationAxis::Longitudinal;
    let training = tire_relaxation_run(tire, axis, 0.35, 0);
    let holdout_a = tire_relaxation_run(tire, axis, 0.35, 1);
    let mut holdout_b = tire_relaxation_run(tire, axis, 0.35, 2);
    let training_run = TireRelaxationIdentificationRun {
        acquisition_id: 1,
        condition_id: 10,
        samples: &training,
    };
    assert_eq!(
        identify_tire_relaxation_length(
            tire_relaxation_identification_spec(),
            tire,
            axis,
            &[training_run],
            &[TireRelaxationIdentificationRun {
                acquisition_id: 1,
                condition_id: 20,
                samples: &holdout_a,
            }],
        ),
        Err(TireRelaxationIdentificationError::DuplicateAcquisition)
    );

    let mut bad_clock = holdout_a.clone();
    bad_clock[10].capture_time_s = bad_clock[9].capture_time_s;
    assert_eq!(
        identify_tire_relaxation_length(
            tire_relaxation_identification_spec(),
            tire,
            axis,
            &[training_run],
            &[
                TireRelaxationIdentificationRun {
                    acquisition_id: 2,
                    condition_id: 20,
                    samples: &bad_clock,
                },
                TireRelaxationIdentificationRun {
                    acquisition_id: 3,
                    condition_id: 30,
                    samples: &holdout_b,
                },
            ],
        ),
        Err(TireRelaxationIdentificationError::InvalidSample)
    );

    holdout_b[80].measured_force_n += 10.0;
    assert_eq!(
        identify_tire_relaxation_length(
            tire_relaxation_identification_spec(),
            tire,
            axis,
            &[training_run],
            &[
                TireRelaxationIdentificationRun {
                    acquisition_id: 2,
                    condition_id: 20,
                    samples: &holdout_a,
                },
                TireRelaxationIdentificationRun {
                    acquisition_id: 3,
                    condition_id: 30,
                    samples: &holdout_b,
                },
            ],
        ),
        Err(TireRelaxationIdentificationError::ResidualExceeded)
    );

    let mut saturated = holdout_a;
    saturated[20].measured_force_n = 0.96
        * tire.longitudinal_peak_friction
        * saturated[20].normal_load_n
        * saturated[20].road_friction_scale;
    assert_eq!(
        identify_tire_relaxation_length(
            tire_relaxation_identification_spec(),
            tire,
            axis,
            &[training_run],
            &[
                TireRelaxationIdentificationRun {
                    acquisition_id: 2,
                    condition_id: 20,
                    samples: &saturated,
                },
                TireRelaxationIdentificationRun {
                    acquisition_id: 3,
                    condition_id: 30,
                    samples: &holdout_b,
                },
            ],
        ),
        Err(TireRelaxationIdentificationError::InvalidSample)
    );
}

#[test]
fn road_scale_and_load_sensitivity_change_available_force() {
    let mut world = World::new();
    let wheel = world.spawn_empty().id();
    let spec = CombinedSlipTireSpec {
        longitudinal_relaxation_length_m: 0.0,
        lateral_relaxation_length_m: 0.0,
        ..CombinedSlipTireSpec::default()
    };
    let evaluate = |load_n, road_scale| {
        evaluate_combined_slip_tire(
            spec,
            CombinedSlipTireState::default(),
            test_tire_input(
                Some(test_patch(wheel, Vec3::new(-20.0, 0.0, 0.0), load_n)),
                10.0,
                road_scale,
            ),
            0.01,
        )
        .unwrap()
    };
    let dry = evaluate(1_000.0, 1.0);
    let split_low = evaluate(1_000.0, 0.4);
    assert!(split_low.longitudinal_force_n < dry.longitudinal_force_n);
    assert_eq!(
        split_low.longitudinal_peak_force_n,
        dry.longitudinal_peak_force_n * 0.4
    );
    let double_load = evaluate(2_000.0, 1.0);
    assert!(double_load.longitudinal_peak_force_n < dry.longitudinal_peak_force_n * 2.0);
}

#[test]
fn low_speed_relaxation_and_lift_off_are_explicit() {
    let mut world = World::new();
    let wheel = world.spawn_empty().id();
    let spec = CombinedSlipTireSpec::default();
    let first = evaluate_combined_slip_tire(
        spec,
        CombinedSlipTireState::default(),
        test_tire_input(
            Some(test_patch(wheel, Vec3::new(-0.01, 0.0, 0.0), 800.0)),
            0.0,
            1.0,
        ),
        0.01,
    )
    .unwrap();
    assert!(first.state.longitudinal_slip_ratio.is_finite());
    assert!(first.state.longitudinal_slip_ratio > 0.0);
    assert!(first.state.longitudinal_slip_ratio < 0.1);

    let second = evaluate_combined_slip_tire(
        spec,
        first.state,
        test_tire_input(
            Some(test_patch(wheel, Vec3::new(-0.01, 0.0, 0.0), 800.0)),
            0.0,
            1.0,
        ),
        0.01,
    )
    .unwrap();
    assert!(second.state.longitudinal_slip_ratio > first.state.longitudinal_slip_ratio);

    let lifted =
        evaluate_combined_slip_tire(spec, second.state, test_tire_input(None, 0.0, 1.0), 0.01)
            .unwrap();
    assert_eq!(lifted, zero_tire_evaluation());
}

#[test]
fn tire_wrench_preserves_patch_point_and_world_axes() {
    let mut world = World::new();
    let wheel = world.spawn_empty().id();
    let patch = WheelContactPatch {
        point_world_m: Vec3::new(1.0, 2.0, 3.0),
        ..test_patch(wheel, Vec3::ZERO, 100.0)
    };
    let evaluation = CombinedSlipTireEvaluation {
        longitudinal_force_n: 20.0,
        lateral_force_n: -5.0,
        ..zero_tire_evaluation()
    };
    let wrench = combined_slip_tire_wrench(patch, evaluation, Vec3::X, Vec3::Z).unwrap();
    assert_eq!(wrench.entity, wheel);
    assert_eq!(wrench.point_world_m, patch.point_world_m);
    assert_eq!(wrench.force_world_n, Vec3::new(20.0, 0.0, -5.0));
}

#[test]
fn tire_wrench_is_tangent_to_tilted_contact_plane() {
    let mut world = World::new();
    let wheel = world.spawn_empty().id();
    let normal = Vec3::new(0.2, 0.97, 0.1).normalize();
    let patch = WheelContactPatch {
        normal_road_to_wheel_world: normal,
        ..test_patch(wheel, Vec3::ZERO, 100.0)
    };
    let evaluation = CombinedSlipTireEvaluation {
        longitudinal_force_n: 20.0,
        lateral_force_n: -5.0,
        ..zero_tire_evaluation()
    };

    let wrench = combined_slip_tire_wrench(patch, evaluation, Vec3::X, Vec3::Z).unwrap();

    assert!(wrench.force_world_n.dot(normal).abs() < 1.0e-12);
    assert!((wrench.force_world_n.length() - 20.0_f64.hypot(5.0)).abs() < 1.0e-12);
}

fn longitudinal_plant_spec(road_friction_scale: f64) -> LongitudinalMobilityPlantSpec {
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: 100.0,
        driven_wheel_count: 2,
        normal_load_per_driven_wheel_n: 490.3325,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.4,
        road_friction_scale,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec::default(),
        tire: CombinedSlipTireSpec {
            reference_load_n: 490.3325,
            ..CombinedSlipTireSpec::default()
        },
        longitudinal_load_transfer: None,
    }
}

fn run_longitudinal_plant(
    spec: LongitudinalMobilityPlantSpec,
    mut state: LongitudinalMobilityPlantState,
    command_voltage_v: f64,
    dt_s: f64,
    steps: usize,
) -> (LongitudinalMobilityPlantState, f64) {
    let mut maximum_utilization = 0.0_f64;
    for _ in 0..steps {
        let evaluation =
            evaluate_longitudinal_mobility_plant(spec, state, command_voltage_v, dt_s).unwrap();
        state = evaluation.state;
        maximum_utilization = maximum_utilization.max(evaluation.tire.friction_utilization);
    }
    (state, maximum_utilization)
}

#[test]
fn longitudinal_plant_stays_at_rest_without_voltage() {
    let spec = longitudinal_plant_spec(1.0);
    let (state, utilization) = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        0.0,
        0.001,
        1_000,
    );

    assert_eq!(state, LongitudinalMobilityPlantState::default());
    assert_eq!(utilization, 0.0);
}

#[test]
fn longitudinal_drive_path_composes_carrier_and_wheel_surface_velocity_once() {
    let spec = longitudinal_plant_spec(1.0);
    let body = Entity::from_raw(41);
    let evaluation = evaluate_longitudinal_drive_path(
        spec,
        LongitudinalDrivePathState::default(),
        LongitudinalDrivePathInput {
            carrier_patch: Some(WheelContactPatch {
                wheel_entity: body,
                point_world_m: Vec3::new(0.0, -0.25, 0.0),
                normal_road_to_wheel_world: Vec3::Y,
                wheel_relative_to_road_world_m_s: Vec3::X,
                normal_load_n: spec.normal_load_per_driven_wheel_n,
            }),
            forward_world: Vec3::X,
            lateral_world: Vec3::Z,
            command_voltage_v: 0.0,
        },
        0.001,
    )
    .unwrap();

    assert!(evaluation.tire.state.longitudinal_slip_ratio < 0.0);
    assert!(evaluation.tire.longitudinal_force_n < 0.0);
    let wrench = evaluation.tire_wrench.expect("contact wrench");
    assert_eq!(wrench.entity, body);
    assert!(wrench.force_world_n.x < 0.0);
    assert_eq!(wrench.force_world_n.y, 0.0);
}

#[test]
fn longitudinal_drive_path_lift_resets_tire_and_emits_no_wrench() {
    let spec = longitudinal_plant_spec(1.0);
    let evaluation = evaluate_longitudinal_drive_path(
        spec,
        LongitudinalDrivePathState {
            tire_state: CombinedSlipTireState {
                longitudinal_slip_ratio: 0.4,
                lateral_slip_tangent: -0.2,
            },
            ..LongitudinalDrivePathState::default()
        },
        LongitudinalDrivePathInput {
            carrier_patch: None,
            forward_world: Vec3::X,
            lateral_world: Vec3::Z,
            command_voltage_v: 24.0,
        },
        0.001,
    )
    .unwrap();

    assert_eq!(evaluation.tire, zero_tire_evaluation());
    assert_eq!(evaluation.rolling_resistance_torque_nm, 0.0);
    assert!(evaluation.state.wheel_velocity_rad_s > 0.0);
    assert_eq!(evaluation.tire_wrench, None);
}

#[test]
fn longitudinal_plant_couples_current_wheel_slip_and_chassis_acceleration() {
    let spec = longitudinal_plant_spec(1.0);
    let first = evaluate_longitudinal_mobility_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
    )
    .unwrap();
    assert_eq!(first.motor.state.current_a, spec.motor.current_limit_a);
    assert!(first.motor.current_saturated);
    assert!(first.state.wheel_velocity_rad_s > 0.0);
    assert_eq!(first.state.velocity_m_s, 0.0);
    assert_eq!(first.motor_telemetry.current_a, first.motor.state.current_a);

    let (state, maximum_utilization) =
        run_longitudinal_plant(spec, first.state, 24.0, 0.001, 1_999);
    assert!(state.position_m > 1.0);
    assert!(state.velocity_m_s > 1.0);
    assert!(state.wheel_velocity_rad_s * spec.wheel.radius_m > state.velocity_m_s);
    assert!(maximum_utilization > 0.1 && maximum_utilization <= 1.0);
}

#[test]
fn low_friction_reduces_speed_and_increases_wheel_spin() {
    let high_spec = longitudinal_plant_spec(1.0);
    // Ice-like road scaling deliberately places the nominal drive in the
    // traction-limited regime. A 0.2 scale is still motor-limited for this
    // plant and therefore cannot provide a useful friction regression.
    let low_spec = longitudinal_plant_spec(0.05);
    let (high, _) = run_longitudinal_plant(
        high_spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        2_000,
    );
    let (low, _) = run_longitudinal_plant(
        low_spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        2_000,
    );

    assert!(low.velocity_m_s < high.velocity_m_s);
    let high_slip_speed = high.wheel_velocity_rad_s * high_spec.wheel.radius_m - high.velocity_m_s;
    let low_slip_speed = low.wheel_velocity_rad_s * low_spec.wheel.radius_m - low.velocity_m_s;
    assert!(low_slip_speed > high_slip_speed);
}

#[test]
fn negative_voltage_regeneratively_brakes_a_moving_vehicle() {
    let spec = longitudinal_plant_spec(1.0);
    let (accelerated, _) = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        1_500,
    );
    let before_velocity_m_s = accelerated.velocity_m_s;
    let braking = evaluate_longitudinal_mobility_plant(spec, accelerated, -24.0, 0.001).unwrap();
    assert!(braking.motor.state.current_a < 0.0);
    let (braked, _) = run_longitudinal_plant(spec, braking.state, -24.0, 0.001, 499);
    assert!(braked.velocity_m_s < before_velocity_m_s);
}

#[test]
fn longitudinal_plant_is_deterministic_symmetric_and_step_convergent() {
    let spec = longitudinal_plant_spec(1.0);
    let forward = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        12.0,
        0.001,
        2_000,
    )
    .0;
    let replay = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        12.0,
        0.001,
        2_000,
    )
    .0;
    let reverse = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        -12.0,
        0.001,
        2_000,
    )
    .0;
    let finer = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        12.0,
        0.0005,
        4_000,
    )
    .0;

    assert_eq!(forward, replay);
    assert!((forward.position_m + reverse.position_m).abs() < 1.0e-9);
    assert!((forward.velocity_m_s + reverse.velocity_m_s).abs() < 1.0e-9);
    assert!((forward.velocity_m_s - finer.velocity_m_s).abs() < 0.15);
    assert!((forward.position_m - finer.position_m).abs() < 0.15);
}

fn load_transfer_geometry(driven_axle: DrivenAxle) -> LongitudinalLoadTransferSpec {
    LongitudinalLoadTransferSpec {
        wheelbase_m: 1.2,
        cg_height_m: 0.35,
        driven_axle,
    }
}

/// Reference implementation of the plant's pre-load-transfer integration
/// loop, copied verbatim from `evaluate_longitudinal_mobility_plant`
/// before load transfer existed (constant `normal_load_per_driven_wheel_n`
/// fed to the drive path every step). Used only to prove that an absent
/// `longitudinal_load_transfer` reproduces the old behavior bit-for-bit,
/// independent of the production function's internals.
fn reference_step_without_load_transfer(
    spec: LongitudinalMobilityPlantSpec,
    state: LongitudinalMobilityPlantState,
    command_voltage_v: f64,
    dt_s: f64,
) -> LongitudinalMobilityPlantState {
    let drive = evaluate_longitudinal_drive_path(
        spec,
        LongitudinalDrivePathState {
            wheel_position_rad: state.wheel_position_rad,
            wheel_velocity_rad_s: state.wheel_velocity_rad_s,
            motor_state: state.motor_state,
            tire_state: state.tire_state,
        },
        LongitudinalDrivePathInput {
            carrier_patch: Some(WheelContactPatch {
                wheel_entity: Entity::PLACEHOLDER,
                point_world_m: Vec3::ZERO,
                normal_road_to_wheel_world: Vec3::Y,
                wheel_relative_to_road_world_m_s: Vec3::X * state.velocity_m_s,
                normal_load_n: spec.normal_load_per_driven_wheel_n,
            }),
            forward_world: Vec3::X,
            lateral_world: Vec3::Z,
            command_voltage_v,
        },
        dt_s,
    )
    .unwrap();
    let aerodynamic_force_n =
        -spec.aerodynamic_drag_n_s2_m2 * state.velocity_m_s * state.velocity_m_s.abs();
    let grade_resistance_force_n = spec.vehicle_mass_kg * 9.806_65 * spec.road_grade_rad.sin();
    let chassis_acceleration_m_s2 = (f64::from(spec.driven_wheel_count)
        * drive.tire.longitudinal_force_n
        + aerodynamic_force_n
        - grade_resistance_force_n)
        / spec.vehicle_mass_kg;
    let velocity_m_s = state.velocity_m_s + chassis_acceleration_m_s2 * dt_s;
    LongitudinalMobilityPlantState {
        position_m: state.position_m + velocity_m_s * dt_s,
        velocity_m_s,
        wheel_position_rad: drive.state.wheel_position_rad,
        wheel_velocity_rad_s: drive.state.wheel_velocity_rad_s,
        motor_state: drive.state.motor_state,
        tire_state: drive.state.tire_state,
        previous_chassis_acceleration_m_s2: chassis_acceleration_m_s2,
    }
}

#[test]
fn longitudinal_plant_without_load_transfer_matches_pre_change_trajectory_bit_for_bit() {
    let spec = longitudinal_plant_spec(1.0);
    assert!(spec.longitudinal_load_transfer.is_none());

    let mut reference_state = LongitudinalMobilityPlantState::default();
    let mut actual_state = LongitudinalMobilityPlantState::default();
    for _ in 0..1_500 {
        reference_state = reference_step_without_load_transfer(spec, reference_state, 24.0, 0.001);
        actual_state = evaluate_longitudinal_mobility_plant(spec, actual_state, 24.0, 0.001)
            .unwrap()
            .state;
        assert_eq!(actual_state, reference_state);
    }
    // The transient covers a hard-acceleration phase, so
    // `previous_chassis_acceleration_m_s2` is meaningfully nonzero by the
    // end -- proving the absent-spec path never lets it influence load.
    assert!(actual_state.previous_chassis_acceleration_m_s2.abs() > 1.0e-6);
}

#[test]
fn load_transfer_present_with_zero_acceleration_matches_static_load() {
    let spec = longitudinal_plant_spec(1.0);
    for driven_axle in [DrivenAxle::Front, DrivenAxle::Rear] {
        let load_n = resolve_driven_wheel_normal_load_n(
            spec,
            Some(load_transfer_geometry(driven_axle)),
            0.0,
        );
        assert_eq!(load_n, spec.normal_load_per_driven_wheel_n);
    }
}

#[test]
fn load_transfer_shifts_load_by_braking_or_accelerating_and_conserves_total() {
    let spec = longitudinal_plant_spec(1.0);
    let rear = load_transfer_geometry(DrivenAxle::Rear);
    let front = load_transfer_geometry(DrivenAxle::Front);
    let static_per_wheel_n = spec.normal_load_per_driven_wheel_n;

    // Braking (negative a_x): a rear-driven wheel loses load, a
    // front-driven wheel gains it.
    let braking_rear_n = resolve_driven_wheel_normal_load_n(spec, Some(rear), -4.0);
    let braking_front_n = resolve_driven_wheel_normal_load_n(spec, Some(front), -4.0);
    assert!(braking_rear_n < static_per_wheel_n);
    assert!(braking_front_n > static_per_wheel_n);

    // Accelerating (positive a_x): the reverse.
    let accel_rear_n = resolve_driven_wheel_normal_load_n(spec, Some(rear), 4.0);
    let accel_front_n = resolve_driven_wheel_normal_load_n(spec, Some(front), 4.0);
    assert!(accel_rear_n > static_per_wheel_n);
    assert!(accel_front_n < static_per_wheel_n);

    // The front and rear deltas are equal and opposite for the same
    // acceleration: whatever one axle gains, the other loses, so the
    // vehicle's total normal load is conserved.
    let rear_delta_n = accel_rear_n - static_per_wheel_n;
    let front_delta_n = accel_front_n - static_per_wheel_n;
    assert!((rear_delta_n + front_delta_n).abs() < 1.0e-9);
}

#[test]
fn load_transfer_clamps_unloaded_axle_to_zero_not_negative() {
    let spec = longitudinal_plant_spec(1.0);
    let rear = load_transfer_geometry(DrivenAxle::Rear);
    // A deceleration large enough that the analytic transfer alone
    // demands more load than the rear axle statically carries.
    let load_n = resolve_driven_wheel_normal_load_n(spec, Some(rear), -1_000.0);
    assert_eq!(load_n, 0.0);
}

#[test]
fn load_transfer_spec_validation_rejects_bad_geometry() {
    let valid = load_transfer_geometry(DrivenAxle::Rear);
    assert!(valid.is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        wheelbase_m: 0.0,
        ..valid
    }
    .is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        wheelbase_m: -1.0,
        ..valid
    }
    .is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        wheelbase_m: f64::NAN,
        ..valid
    }
    .is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        wheelbase_m: f64::INFINITY,
        ..valid
    }
    .is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        cg_height_m: -0.01,
        ..valid
    }
    .is_valid());
    assert!(!LongitudinalLoadTransferSpec {
        cg_height_m: f64::NAN,
        ..valid
    }
    .is_valid());

    let invalid_plant_spec = LongitudinalMobilityPlantSpec {
        longitudinal_load_transfer: Some(LongitudinalLoadTransferSpec {
            wheelbase_m: 0.0,
            ..valid
        }),
        ..longitudinal_plant_spec(1.0)
    };
    assert!(!invalid_plant_spec.is_valid());
    let error = evaluate_longitudinal_mobility_plant(
        invalid_plant_spec,
        LongitudinalMobilityPlantState::default(),
        1.0,
        0.001,
    )
    .unwrap_err();
    assert_eq!(error, MobilityPlantEvaluationError::InvalidSpec);
}

#[test]
fn longitudinal_plant_with_load_transfer_is_deterministic() {
    let spec = LongitudinalMobilityPlantSpec {
        longitudinal_load_transfer: Some(load_transfer_geometry(DrivenAxle::Rear)),
        ..longitudinal_plant_spec(1.0)
    };
    let forward = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        2_000,
    )
    .0;
    let replay = run_longitudinal_plant(
        spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        2_000,
    )
    .0;
    assert_eq!(forward, replay);
}

#[test]
fn load_transfer_changes_tire_force_during_hard_acceleration_transient() {
    let base_spec = longitudinal_plant_spec(1.0);
    let transfer = load_transfer_geometry(DrivenAxle::Rear);
    let spec_with_transfer = LongitudinalMobilityPlantSpec {
        longitudinal_load_transfer: Some(transfer),
        ..base_spec
    };

    // Warm up a physically realistic hard-acceleration state (nonzero
    // wheel spin, relaxed slip, and motor current) with the baseline
    // (no-transfer) plant, then branch both plants from that one shared
    // state for a single step. This isolates the load-transfer effect
    // from the chaotic divergence a multi-step rollout would introduce.
    let (warmed_up_state, _) = run_longitudinal_plant(
        base_spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        0.001,
        200,
    );
    assert!(warmed_up_state.previous_chassis_acceleration_m_s2.abs() > 0.5);

    let without_transfer =
        evaluate_longitudinal_mobility_plant(base_spec, warmed_up_state, 24.0, 0.001).unwrap();
    let with_transfer =
        evaluate_longitudinal_mobility_plant(spec_with_transfer, warmed_up_state, 24.0, 0.001)
            .unwrap();

    let expected_load_n = resolve_driven_wheel_normal_load_n(
        spec_with_transfer,
        Some(transfer),
        warmed_up_state.previous_chassis_acceleration_m_s2,
    );
    assert!(expected_load_n > base_spec.normal_load_per_driven_wheel_n);

    // If load stayed constant (the bug this slice fixes), these two
    // peak forces -- and thus the resulting drive force -- would be
    // bit-for-bit identical, since both plants share the same tire
    // spec, motor, wheel state, and command voltage.
    assert!(
        with_transfer.tire.longitudinal_peak_force_n
            > without_transfer.tire.longitudinal_peak_force_n,
        "with={} without={}",
        with_transfer.tire.longitudinal_peak_force_n,
        without_transfer.tire.longitudinal_peak_force_n
    );
    assert_ne!(
        with_transfer.tire.longitudinal_force_n, without_transfer.tire.longitudinal_force_n,
        "with={} without={}",
        with_transfer.tire.longitudinal_force_n, without_transfer.tire.longitudinal_force_n
    );
    assert_ne!(with_transfer.state, without_transfer.state);
}
