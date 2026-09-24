use super::*;

/// Fits complete training acquisitions without inspecting holdout data or residual gates.
///
/// Uses the same centered least-squares solver and physical parameter bounds as
/// ordinary identification. This is an estimator, not a model acceptance verdict.
/// Input clocks are validated separately for every acquisition. Callers performing
/// repeated uncertainty draws must retain failures and bound their total workload.
pub fn fit_suspension_training_runs(
    spec: SuspensionIdentificationSpec,
    runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionTrainingCoefficients, SuspensionIdentificationError> {
    validate_suspension_training_runs(spec, runs)?;
    let (stiffness_n_per_m, damping_n_s_per_m, equilibrium_position_m) =
        fit_suspension_training_coefficients(
            spec,
            runs.iter().flat_map(|run| run.samples.iter().copied()),
        )?;
    Ok(SuspensionTrainingCoefficients {
        stiffness_n_per_m,
        damping_n_s_per_m,
        equilibrium_position_m,
    })
}

pub(crate) fn validate_suspension_training_runs(
    spec: SuspensionIdentificationSpec,
    runs: &[SuspensionIdentificationRun<'_>],
) -> Result<(), SuspensionIdentificationError> {
    if !spec.is_valid() || runs.len() > 64 {
        return Err(SuspensionIdentificationError::InvalidSpec);
    }
    if runs.is_empty() {
        return Err(SuspensionIdentificationError::InsufficientSamples);
    }
    let mut ids = std::collections::BTreeSet::new();
    for run in runs {
        if !ids.insert(run.acquisition_id) {
            return Err(SuspensionIdentificationError::DuplicateAcquisition);
        }
        if run.samples.is_empty() {
            return Err(SuspensionIdentificationError::InsufficientSamples);
        }
        validate_suspension_samples(run.samples)?;
    }
    Ok(())
}

/// Refits after deleting each complete training run, preserving sample/local-clock order.
///
/// Accepts 1 through 64 runs; a single run retains an insufficient-samples deletion.
/// Validates all inputs before fitting. Uses coefficient and training-count bounds,
/// but does not evaluate training/holdout residual gates. No holdout data are accepted.
/// This diagnostic neither selects a model nor establishes uncertainty coverage.
pub fn suspension_training_influence(
    spec: SuspensionIdentificationSpec,
    runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionTrainingInfluence, SuspensionIdentificationError> {
    validate_suspension_training_runs(spec, runs)?;
    let fit = |omit: Option<usize>| {
        fit_suspension_training_coefficients(
            spec,
            runs.iter()
                .enumerate()
                .filter(move |(index, _)| Some(*index) != omit)
                .flat_map(|(_, run)| run.samples.iter().copied()),
        )
        .map(
            |(stiffness_n_per_m, damping_n_s_per_m, equilibrium_position_m)| {
                SuspensionTrainingCoefficients {
                    stiffness_n_per_m,
                    damping_n_s_per_m,
                    equilibrium_position_m,
                }
            },
        )
    };
    Ok(SuspensionTrainingInfluence {
        baseline: fit(None),
        deletions: runs
            .iter()
            .enumerate()
            .map(|(index, run)| SuspensionAcquisitionInfluence {
                omitted_acquisition_id: run.acquisition_id,
                coefficients: fit(Some(index)),
            })
            .collect(),
    })
}

/// Within-acquisition residual timing and lag-one diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionResidualTiming {
    /// Caller-supplied acquisition identity.
    pub acquisition_id: u64,
    /// Number of evaluated samples.
    pub sample_count: usize,
    /// Smallest observed adjacent capture interval in seconds.
    pub minimum_interval_s: f64,
    /// Largest observed adjacent capture interval in seconds.
    pub maximum_interval_s: f64,
    /// Caller-declared absolute tolerance against the first interval.
    pub interval_tolerance_s: f64,
    /// Whether every interval matches the first within the declared tolerance.
    pub uniform_within_tolerance: bool,
    /// Mean predicted-minus-measured force in newtons.
    pub mean_residual_n: f64,
    /// Lag-one centered autocorrelation with full-run energy denominator.
    /// Absent for nonuniform timing or constant residuals, never replaced by zero.
    pub lag_one_autocorrelation: Option<f64>,
}

/// Evaluates frozen fit residuals without refitting or joining acquisition clocks.
///
/// Uses `sum((e[i]-mean)*(e[i+1]-mean))/sum((e[i]-mean)^2)` only on a
/// uniform-within-tolerance grid. Unequal intervals are retained as diagnostics;
/// no interpolation, effective sample size or confidence interval is invented.
/// The caller must justify the tolerance from clock evidence. This function is
/// a diagnostic and does not certify the provenance or acceptance of `fit`.
pub fn suspension_residual_timing(
    fit: SuspensionIdentificationResult,
    run: SuspensionIdentificationRun<'_>,
    interval_tolerance_s: f64,
) -> Result<SuspensionResidualTiming, SuspensionIdentificationError> {
    if !interval_tolerance_s.is_finite()
        || interval_tolerance_s < 0.0
        || !fit.stiffness_n_per_m.is_finite()
        || fit.stiffness_n_per_m <= 0.0
        || !fit.damping_n_s_per_m.is_finite()
        || fit.damping_n_s_per_m < 0.0
        || !fit.equilibrium_position_m.is_finite()
    {
        return Err(SuspensionIdentificationError::InvalidSpec);
    }
    validate_suspension_samples(run.samples)?;
    if run.samples.len() < 3 {
        return Err(SuspensionIdentificationError::InsufficientSamples);
    }
    let first_interval = run.samples[1].capture_time_s - run.samples[0].capture_time_s;
    let mut minimum_interval_s = f64::INFINITY;
    let mut maximum_interval_s = 0.0_f64;
    let mut uniform_within_tolerance = true;
    for pair in run.samples.windows(2) {
        let interval = pair[1].capture_time_s - pair[0].capture_time_s;
        if !interval.is_finite() {
            return Err(SuspensionIdentificationError::InvalidSample);
        }
        minimum_interval_s = minimum_interval_s.min(interval);
        maximum_interval_s = maximum_interval_s.max(interval);
        uniform_within_tolerance &= (interval - first_interval).abs() <= interval_tolerance_s;
    }
    let residual = |s: &SuspensionForceSample| {
        fit.stiffness_n_per_m * (fit.equilibrium_position_m - s.position_m)
            - fit.damping_n_s_per_m * s.velocity_m_s
            - s.force_n
    };
    let mut scale = 0.0_f64;
    for s in run.samples {
        let value = residual(s);
        if !value.is_finite() {
            return Err(SuspensionIdentificationError::ResidualExceeded);
        }
        scale = scale.max(value.abs());
    }
    let scale = if scale == 0.0 { 1.0 } else { scale };
    let origin = residual(&run.samples[0]) / scale;
    let offset = run
        .samples
        .iter()
        .map(|s| (residual(s) / scale - origin) / run.samples.len() as f64)
        .sum::<f64>();
    let mean_residual_n = (origin + offset) * scale;
    if !mean_residual_n.is_finite() {
        return Err(SuspensionIdentificationError::ResidualExceeded);
    }
    let centered = |s: &SuspensionForceSample| (residual(s) / scale - origin) - offset;
    let energy = run.samples.iter().map(|s| centered(s).powi(2)).sum::<f64>();
    let lag_one_autocorrelation = if uniform_within_tolerance && energy > 0.0 {
        Some(
            run.samples
                .windows(2)
                .map(|p| centered(&p[0]) * centered(&p[1]))
                .sum::<f64>()
                / energy,
        )
    } else {
        None
    };
    Ok(SuspensionResidualTiming {
        acquisition_id: run.acquisition_id,
        sample_count: run.samples.len(),
        minimum_interval_s,
        maximum_interval_s,
        interval_tolerance_s,
        uniform_within_tolerance,
        mean_residual_n,
        lag_one_autocorrelation,
    })
}

/// Training-only excitation diagnostics for the centered two-column design.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionExcitationDiagnostics {
    /// Number of training samples, pooled in caller order.
    pub sample_count: usize,
    /// Centered position RMS in meters (population normalization).
    pub position_rms_m: f64,
    /// Centered velocity RMS in meters per second (population normalization).
    pub velocity_rms_m_s: f64,
    /// Position/velocity correlation; absent when either column has zero energy.
    pub position_velocity_correlation: Option<f64>,
    /// L2 condition number of the centered, unit-column-norm design, not its Gram
    /// matrix. Absent for zero energy or numerically singular correlation.
    pub normalized_design_condition: Option<f64>,
}

/// Measures training excitation without accepting holdout data or using forces.
///
/// With normalized centered columns the Gram eigenvalues are `1 +/- |rho|`,
/// so the design condition is `sqrt((1 + |rho|)/(1 - |rho|))`. This diagnoses
/// collinearity only: retain the SI RMS values to inspect excitation magnitude.
/// It is not a covariance estimate or a parameter-acceptance gate. Input samples
/// still require valid finite force fields and ordered per-acquisition clocks.
/// Scaling before centering avoids squaring large SI coordinates directly.
pub fn suspension_training_excitation(
    runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionExcitationDiagnostics, SuspensionIdentificationError> {
    let mut ids = std::collections::BTreeSet::new();
    for run in runs {
        if !ids.insert(run.acquisition_id) {
            return Err(SuspensionIdentificationError::DuplicateAcquisition);
        }
        if run.samples.is_empty() {
            return Err(SuspensionIdentificationError::InsufficientSamples);
        }
        validate_suspension_samples(run.samples)?;
    }
    let samples = || runs.iter().flat_map(|run| run.samples.iter());
    let sample_count = samples().count();
    if sample_count < 3 {
        return Err(SuspensionIdentificationError::InsufficientSamples);
    }
    let (x_scale, v_scale) = samples().fold((0.0_f64, 0.0_f64), |(x, v), s| {
        (x.max(s.position_m.abs()), v.max(s.velocity_m_s.abs()))
    });
    let x_scale = if x_scale == 0.0 { 1.0 } else { x_scale };
    let v_scale = if v_scale == 0.0 { 1.0 } else { v_scale };
    let n = sample_count as f64;
    let first = runs[0].samples[0];
    let x_origin = first.position_m / x_scale;
    let v_origin = first.velocity_m_s / v_scale;
    let (x_offset, v_offset) = samples().fold((0.0, 0.0), |(x, v), s| {
        (
            x + (s.position_m / x_scale - x_origin) / n,
            v + (s.velocity_m_s / v_scale - v_origin) / n,
        )
    });
    let (xx, vv, xv) = samples().fold((0.0, 0.0, 0.0), |(xx, vv, xv), s| {
        let x = (s.position_m / x_scale - x_origin) - x_offset;
        let v = (s.velocity_m_s / v_scale - v_origin) - v_offset;
        (xx + x * x, vv + v * v, xv + x * v)
    });
    let position_rms_m = (xx / n).sqrt() * x_scale;
    let velocity_rms_m_s = (vv / n).sqrt() * v_scale;
    if !position_rms_m.is_finite() || !velocity_rms_m_s.is_finite() {
        return Err(SuspensionIdentificationError::InvalidSample);
    }
    let correlation = if xx > 0.0 && vv > 0.0 {
        Some((xv / xx.sqrt() / vv.sqrt()).clamp(-1.0, 1.0))
    } else {
        None
    };
    let condition = correlation.and_then(|rho| {
        let gap = 1.0 - rho.abs();
        (gap > f64::EPSILON * 8.0).then(|| ((1.0 + rho.abs()) / gap).sqrt())
    });
    Ok(SuspensionExcitationDiagnostics {
        sample_count,
        position_rms_m,
        velocity_rms_m_s,
        position_velocity_correlation: correlation,
        normalized_design_condition: condition,
    })
}

/// Residual evidence for one acquisition, evaluated with frozen fitted parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionRunResidual {
    /// Caller-supplied acquisition identity.
    pub acquisition_id: u64,
    /// Number of samples in this run.
    pub sample_count: usize,
    /// Force root-mean-square error in newtons.
    pub rmse_n: f64,
    /// Largest absolute force error in newtons (diagnostic, not separately gated).
    pub maximum_absolute_residual_n: f64,
    /// Whether this run meets the RMSE bound for its training or holdout role.
    pub passed: bool,
}

/// Pooled fit and individual acquisition checks; not physical qualification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionRunIdentificationReport {
    /// Fit accepted by the pooled v1 parameter and residual gates.
    pub fit: SuspensionIdentificationResult,
    /// Training-run metrics in caller-specified order.
    pub training_runs: Vec<SuspensionRunResidual>,
    /// Holdout-run metrics in caller-specified order.
    pub holdout_runs: Vec<SuspensionRunResidual>,
    /// True only when every individual run also meets its role's RMSE bound.
    pub passed: bool,
}

/// Identifies a run split and checks that no individual run is hidden by pooling.
///
/// Pooled fit failures return an error. Individual RMSE failures remain in the
/// returned report with `passed = false`, preserving diagnostics. The minimum
/// sample counts apply to the pooled roles, not individual runs. No confidence
/// interval, calibration attestation or independent-acquisition proof is implied.
pub fn identify_suspension_strut_runs_report(
    spec: SuspensionIdentificationSpec,
    training_runs: &[SuspensionIdentificationRun<'_>],
    holdout_runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionRunIdentificationReport, SuspensionIdentificationError> {
    let fit = identify_suspension_strut_runs(spec, training_runs, holdout_runs)?;
    let evaluate = |runs: &[SuspensionIdentificationRun<'_>], bound_n: f64| {
        runs.iter()
            .map(|run| {
                let mut squared_error = 0.0;
                let mut maximum_absolute_residual_n = 0.0_f64;
                for sample in run.samples {
                    let residual_n = fit.stiffness_n_per_m
                        * (fit.equilibrium_position_m - sample.position_m)
                        - fit.damping_n_s_per_m * sample.velocity_m_s
                        - sample.force_n;
                    squared_error += residual_n * residual_n;
                    maximum_absolute_residual_n = maximum_absolute_residual_n.max(residual_n.abs());
                }
                let rmse_n = (squared_error / run.samples.len() as f64).sqrt();
                if !rmse_n.is_finite() || !maximum_absolute_residual_n.is_finite() {
                    return Err(SuspensionIdentificationError::ResidualExceeded);
                }
                Ok(SuspensionRunResidual {
                    acquisition_id: run.acquisition_id,
                    sample_count: run.samples.len(),
                    rmse_n,
                    maximum_absolute_residual_n,
                    passed: rmse_n <= bound_n,
                })
            })
            .collect::<Result<Vec<_>, _>>()
    };
    let training_runs = evaluate(training_runs, spec.maximum_training_rmse_n)?;
    let holdout_runs = evaluate(holdout_runs, spec.maximum_holdout_rmse_n)?;
    let passed = training_runs
        .iter()
        .chain(&holdout_runs)
        .all(|run| run.passed);
    Ok(SuspensionRunIdentificationReport {
        fit,
        training_runs,
        holdout_runs,
        passed,
    })
}

/// Fits complete training acquisitions and evaluates complete held-out acquisitions.
///
/// No held-out force enters coefficient fitting. Run order and sample order are
/// preserved for deterministic accumulation. Each run must be nonempty and have
/// finite samples and a strictly increasing local clock. IDs must be unique even
/// within one split role. Caller identities do not prove physical independence.
///
/// This additive API reuses v1 parameter, sample-count and pooled residual gates;
/// `holdout_stride` must remain valid but does not select samples here. It returns
/// pooled, sample-weighted residuals, not per-run uncertainty or qualification.
pub fn identify_suspension_strut_runs(
    spec: SuspensionIdentificationSpec,
    training_runs: &[SuspensionIdentificationRun<'_>],
    holdout_runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionIdentificationResult, SuspensionIdentificationError> {
    if !spec.is_valid() {
        return Err(SuspensionIdentificationError::InvalidSpec);
    }
    let mut identities = std::collections::BTreeSet::new();
    for run in training_runs.iter().chain(holdout_runs) {
        if !identities.insert(run.acquisition_id) {
            return Err(SuspensionIdentificationError::DuplicateAcquisition);
        }
        if run.samples.is_empty() {
            return Err(SuspensionIdentificationError::InsufficientSamples);
        }
        validate_suspension_samples(run.samples)?;
    }
    identify_suspension_split(
        spec,
        training_runs
            .iter()
            .flat_map(|run| run.samples.iter().copied()),
        holdout_runs
            .iter()
            .flat_map(|run| run.samples.iter().copied()),
    )
}

/// One timestamped force/position/velocity sample from a suspension log.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionForceSample {
    /// Monotonic capture time in seconds.
    pub capture_time_s: f64,
    /// Measured suspension coordinate in meters.
    pub position_m: f64,
    /// Measured suspension-coordinate velocity in meters per second.
    pub velocity_m_s: f64,
    /// Measured generalized strut force in newtons.
    pub force_n: f64,
}

/// Frozen split, physical bounds, and residual gates for suspension identification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionIdentificationSpec {
    /// Every `holdout_stride`th sample is reserved for holdout evaluation.
    pub holdout_stride: usize,
    /// Minimum number of samples used to fit the three coefficients.
    pub minimum_training_samples: usize,
    /// Minimum number of samples retained exclusively for holdout evaluation.
    pub minimum_holdout_samples: usize,
    /// Inclusive stiffness bound in newtons per meter.
    pub stiffness_bounds_n_per_m: [f64; 2],
    /// Inclusive damping bound in newton-seconds per meter.
    pub damping_bounds_n_s_per_m: [f64; 2],
    /// Inclusive unloaded equilibrium-coordinate bound in meters.
    pub equilibrium_position_bounds_m: [f64; 2],
    /// Maximum training root-mean-square force residual in newtons.
    pub maximum_training_rmse_n: f64,
    /// Maximum holdout root-mean-square force residual in newtons.
    pub maximum_holdout_rmse_n: f64,
}

impl SuspensionIdentificationSpec {
    /// Returns whether the split, physical bounds, and residual gates are usable.
    pub fn is_valid(self) -> bool {
        self.holdout_stride >= 2
            && self.minimum_training_samples >= 3
            && self.minimum_holdout_samples >= 1
            && valid_positive_bounds(self.stiffness_bounds_n_per_m)
            && valid_nonnegative_bounds(self.damping_bounds_n_s_per_m)
            && valid_finite_bounds(self.equilibrium_position_bounds_m)
            && self.maximum_training_rmse_n.is_finite()
            && self.maximum_training_rmse_n >= 0.0
            && self.maximum_holdout_rmse_n.is_finite()
            && self.maximum_holdout_rmse_n >= 0.0
    }
}

/// Identified linear strut parameters and independent split residuals.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionIdentificationResult {
    /// Fitted spring stiffness in newtons per meter.
    pub stiffness_n_per_m: f64,
    /// Fitted viscous damping in newton-seconds per meter.
    pub damping_n_s_per_m: f64,
    /// Fitted unloaded equilibrium coordinate in meters.
    pub equilibrium_position_m: f64,
    /// Number of samples used by least squares.
    pub training_sample_count: usize,
    /// Number of samples excluded from fitting and used only for validation.
    pub holdout_sample_count: usize,
    /// Training root-mean-square force residual in newtons.
    pub training_rmse_n: f64,
    /// Holdout root-mean-square force residual in newtons.
    pub holdout_rmse_n: f64,
    /// Largest absolute holdout force residual in newtons.
    pub maximum_absolute_holdout_residual_n: f64,
}

/// Identifies the unclamped linear strut law from timestamped force samples.
///
/// The fitted model is `F = k * (x_eq - x) - c * x_dot`. Samples whose
/// zero-based index plus one is divisible by `holdout_stride` never enter the
/// fit. The remaining samples are solved by centered ordinary least squares;
/// holdout residuals are then evaluated with the frozen result. This routine is
/// deterministic and performs no random resampling or wall-clock access.
pub fn identify_suspension_strut(
    spec: SuspensionIdentificationSpec,
    samples: &[SuspensionForceSample],
) -> Result<SuspensionIdentificationResult, SuspensionIdentificationError> {
    if !spec.is_valid() {
        return Err(SuspensionIdentificationError::InvalidSpec);
    }
    validate_suspension_samples(samples)?;
    let is_holdout = |index: usize| (index + 1).is_multiple_of(spec.holdout_stride);
    identify_suspension_split(
        spec,
        samples
            .iter()
            .copied()
            .enumerate()
            .filter(|(index, _)| !is_holdout(*index))
            .map(|(_, sample)| sample),
        samples
            .iter()
            .copied()
            .enumerate()
            .filter(|(index, _)| is_holdout(*index))
            .map(|(_, sample)| sample),
    )
}

pub(crate) fn validate_suspension_samples(
    samples: &[SuspensionForceSample],
) -> Result<(), SuspensionIdentificationError> {
    if samples.iter().any(|sample| {
        !sample.capture_time_s.is_finite()
            || !sample.position_m.is_finite()
            || !sample.velocity_m_s.is_finite()
            || !sample.force_n.is_finite()
    }) || samples
        .windows(2)
        .any(|pair| pair[0].capture_time_s >= pair[1].capture_time_s)
    {
        return Err(SuspensionIdentificationError::InvalidSample);
    }
    Ok(())
}

// Shared coefficient solver: deliberately has no held-out observations or gates.
pub(crate) fn fit_suspension_training_coefficients(
    spec: SuspensionIdentificationSpec,
    training_samples: impl Iterator<Item = SuspensionForceSample> + Clone,
) -> Result<(f64, f64, f64), SuspensionIdentificationError> {
    let training_sample_count = training_samples.clone().count();
    if training_sample_count < spec.minimum_training_samples {
        return Err(SuspensionIdentificationError::InsufficientSamples);
    }

    let training = || training_samples.clone();
    let count = training_sample_count as f64;
    let (position_sum, velocity_sum, force_sum) = training().fold(
        (0.0, 0.0, 0.0),
        |(position_sum, velocity_sum, force_sum), sample| {
            (
                position_sum + sample.position_m,
                velocity_sum + sample.velocity_m_s,
                force_sum + sample.force_n,
            )
        },
    );
    let position_mean = position_sum / count;
    let velocity_mean = velocity_sum / count;
    let force_mean = force_sum / count;
    let (position_energy, velocity_energy, cross_energy, position_force, velocity_force) =
        training().fold((0.0, 0.0, 0.0, 0.0, 0.0), |sums, sample| {
            let position = sample.position_m - position_mean;
            let velocity = sample.velocity_m_s - velocity_mean;
            let force = sample.force_n - force_mean;
            (
                sums.0 + position * position,
                sums.1 + velocity * velocity,
                sums.2 + position * velocity,
                sums.3 + position * force,
                sums.4 + velocity * force,
            )
        });
    let determinant = position_energy * velocity_energy - cross_energy * cross_energy;
    if position_energy <= 0.0
        || velocity_energy <= 0.0
        || determinant <= 1.0e-12 * position_energy * velocity_energy
    {
        return Err(SuspensionIdentificationError::RankDeficient);
    }
    let position_coefficient =
        (position_force * velocity_energy - velocity_force * cross_energy) / determinant;
    let velocity_coefficient =
        (velocity_force * position_energy - position_force * cross_energy) / determinant;
    let intercept =
        force_mean - position_coefficient * position_mean - velocity_coefficient * velocity_mean;
    let stiffness_n_per_m = -position_coefficient;
    let damping_n_s_per_m = -velocity_coefficient;
    let equilibrium_position_m = intercept / stiffness_n_per_m;
    if !stiffness_n_per_m.is_finite()
        || !damping_n_s_per_m.is_finite()
        || !equilibrium_position_m.is_finite()
        || !(spec.stiffness_bounds_n_per_m[0]..=spec.stiffness_bounds_n_per_m[1])
            .contains(&stiffness_n_per_m)
        || !(spec.damping_bounds_n_s_per_m[0]..=spec.damping_bounds_n_s_per_m[1])
            .contains(&damping_n_s_per_m)
        || !(spec.equilibrium_position_bounds_m[0]..=spec.equilibrium_position_bounds_m[1])
            .contains(&equilibrium_position_m)
    {
        return Err(SuspensionIdentificationError::NonPhysicalResult);
    }

    Ok((stiffness_n_per_m, damping_n_s_per_m, equilibrium_position_m))
}

pub(crate) fn identify_suspension_split(
    spec: SuspensionIdentificationSpec,
    training_samples: impl Iterator<Item = SuspensionForceSample> + Clone,
    holdout_samples: impl Iterator<Item = SuspensionForceSample> + Clone,
) -> Result<SuspensionIdentificationResult, SuspensionIdentificationError> {
    let training_sample_count = training_samples.clone().count();
    let holdout_sample_count = holdout_samples.clone().count();
    if training_sample_count < spec.minimum_training_samples
        || holdout_sample_count < spec.minimum_holdout_samples
    {
        return Err(SuspensionIdentificationError::InsufficientSamples);
    }
    let (stiffness_n_per_m, damping_n_s_per_m, equilibrium_position_m) =
        fit_suspension_training_coefficients(spec, training_samples.clone())?;
    let training = || training_samples.clone();
    let predict = |sample: SuspensionForceSample| {
        stiffness_n_per_m * (equilibrium_position_m - sample.position_m)
            - damping_n_s_per_m * sample.velocity_m_s
    };
    let training_squared_error = training()
        .map(|sample| (predict(sample) - sample.force_n).powi(2))
        .sum::<f64>();
    let mut holdout_squared_error = 0.0;
    let mut maximum_absolute_holdout_residual_n = 0.0_f64;
    for sample in holdout_samples {
        let residual_n = predict(sample) - sample.force_n;
        holdout_squared_error += residual_n.powi(2);
        maximum_absolute_holdout_residual_n =
            maximum_absolute_holdout_residual_n.max(residual_n.abs());
    }
    let training_rmse_n = (training_squared_error / training_sample_count as f64).sqrt();
    let holdout_rmse_n = (holdout_squared_error / holdout_sample_count as f64).sqrt();
    if !training_rmse_n.is_finite()
        || !holdout_rmse_n.is_finite()
        || !maximum_absolute_holdout_residual_n.is_finite()
        || training_rmse_n > spec.maximum_training_rmse_n
        || holdout_rmse_n > spec.maximum_holdout_rmse_n
    {
        return Err(SuspensionIdentificationError::ResidualExceeded);
    }
    Ok(SuspensionIdentificationResult {
        stiffness_n_per_m,
        damping_n_s_per_m,
        equilibrium_position_m,
        training_sample_count,
        holdout_sample_count,
        training_rmse_n,
        holdout_rmse_n,
        maximum_absolute_holdout_residual_n,
    })
}

pub(crate) fn valid_finite_bounds(bounds: [f64; 2]) -> bool {
    bounds.into_iter().all(f64::is_finite) && bounds[0] <= bounds[1]
}

pub(crate) fn valid_positive_bounds(bounds: [f64; 2]) -> bool {
    valid_finite_bounds(bounds) && bounds[0] > 0.0
}

pub(crate) fn valid_nonnegative_bounds(bounds: [f64; 2]) -> bool {
    valid_finite_bounds(bounds) && bounds[0] >= 0.0
}

/// Evaluates one suspension strut as an explicit generalized spring-damper force.
///
/// The backend-neutral law is `k * (x_eq - x) - c * x_dot`, clamped to the
/// declared force limit. Returning direct prismatic effort avoids interpreting
/// physical spring units through a backend-native position-servo model. Travel
/// stops remain part of the paired prismatic-joint description.
pub fn evaluate_suspension_strut(
    spec: SuspensionStrutSpec,
    position_m: f64,
    velocity_m_s: f64,
) -> Result<JointActuation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !position_m.is_finite() || !velocity_m_s.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let force_n = (spec.stiffness_n_per_m * (spec.equilibrium_position_m - position_m)
        - spec.damping_n_s_per_m * velocity_m_s)
        .clamp(-spec.maximum_force_n, spec.maximum_force_n);
    Ok(JointActuation::PrismaticEffort {
        force_n,
        max_force_n: spec.maximum_force_n,
    })
}

/// Completed DC motor electrical and shaft-torque evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotorEvaluation {
    /// State to retain for the next completed step.
    pub state: DcMotorState,
    /// Voltage actually applied after supply limits and failure behavior, in volts.
    pub terminal_voltage_v: f64,
    /// Back-EMF at the supplied rotor speed, in volts.
    pub back_emf_v: f64,
    /// Electromagnetic torque before shaft losses, in newton-meters.
    pub electromagnetic_torque_nm: f64,
    /// Viscous plus Coulomb torque opposing the shaft, in newton-meters.
    pub shaft_loss_torque_nm: f64,
    /// Net torque available at the motor shaft, in newton-meters.
    pub shaft_torque_nm: f64,
    /// Whether the requested terminal voltage exceeded the supply limit.
    pub voltage_saturated: bool,
    /// Whether the unconstrained armature current exceeded the current limit.
    pub current_saturated: bool,
}

/// Completed averaged mapping from a signed PWM command to a voltage request.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwmMotorCommandEvaluation {
    /// Command after saturation at the declared full-scale count.
    pub clamped_command_count: f64,
    /// Signed duty ratio after polarity mapping, bounded to `[-1, 1]`.
    pub signed_duty_ratio: f64,
    /// Ideal average terminal voltage before bridge on-state loss, in volts.
    pub ideal_average_voltage_v: f64,
    /// Average terminal-voltage request after bridge on-state loss, in volts.
    pub terminal_voltage_request_v: f64,
    /// Non-negative average voltage magnitude removed by bridge on-state loss, in volts.
    pub average_bridge_loss_v: f64,
    /// Whether the requested command exceeded the declared command-count range.
    pub command_saturated: bool,
}

/// Completed first-order steering-actuator evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteeringActuatorEvaluation {
    /// State to retain for the next completed step.
    pub state: SteeringActuatorState,
    /// Finite angle requested by the caller, in radians.
    pub requested_target_rad: f64,
    /// Target after steering-travel limits, in radians.
    pub clamped_target_rad: f64,
    /// Completed steering rate over this fixed step, in radians per second.
    pub realized_rate_rad_s: f64,
    /// Whether the requested target exceeded the declared travel limits.
    pub command_saturated: bool,
    /// Whether the unconstrained first-order response exceeded the rate limit.
    pub rate_limited: bool,
    /// Whether an explicit stuck failure held the completed position.
    pub stuck: bool,
}

impl DcMotorEvaluation {
    /// Converts this completed evaluation into sensor-source telemetry.
    ///
    /// Temperature is supplied separately because the v1 electrical evaluator
    /// deliberately has no thermal state.
    pub fn completed_telemetry(
        self,
        failure_mode: DcMotorFailureMode,
        winding_temperature_c: Option<f64>,
    ) -> DcMotorCompletedTelemetry {
        DcMotorCompletedTelemetry {
            terminal_voltage_v: self.terminal_voltage_v,
            current_a: self.state.current_a,
            back_emf_v: self.back_emf_v,
            winding_temperature_c,
            voltage_saturated: self.voltage_saturated,
            current_saturated: self.current_saturated,
            failure_mode,
        }
    }
}

/// Completed static transmission evaluation at one wheel coordinate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransmissionEvaluation {
    /// Motor-shaft velocity implied by the wheel coordinate, in radians per second.
    pub motor_velocity_rad_s: f64,
    /// Wheel-side torque after ratio and directional efficiency, in newton-meters.
    pub wheel_torque_nm: f64,
    /// Motor rotor inertia reflected to the wheel coordinate, in kilogram square meters.
    pub reflected_rotor_inertia_kg_m2: f64,
    /// Efficiency selected from the direction of mechanical power flow.
    pub applied_efficiency_ratio: f64,
}

/// Load-weighted contact patch reconstructed from completed backend contact evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelContactPatch {
    /// Wheel entity represented by this patch.
    pub wheel_entity: Entity,
    /// Load-weighted application point in world coordinates, in meters.
    pub point_world_m: Vec3,
    /// Load-weighted unit normal pointing from the road toward the wheel.
    pub normal_road_to_wheel_world: Vec3,
    /// Wheel-surface velocity relative to the road at the patch, in meters per second.
    pub wheel_relative_to_road_world_m_s: Vec3,
    /// Total step-average normal load carried by the patch, in newtons.
    pub normal_load_n: f64,
}

/// Primitive collision geometry derived from one rigid-road patch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidRoadPatchGeometry {
    /// World transform of the collision solid beneath the driving surface.
    pub solid_transform: Transform3,
    /// Local cuboid half extents in meters.
    pub solid_half_extents_m: Vec3,
    /// Unit tangent pointing uphill along the patch.
    pub longitudinal_tangent_world: Vec3,
    /// Unit normal pointing out of the driving surface.
    pub normal_world: Vec3,
}

/// Metric road properties sampled from a finite rigid-road profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidRoadSurfaceSample {
    /// Index of the selected canonical patch.
    pub patch_index: usize,
    /// Closest point on the finite driving surface, in world meters.
    pub point_world_m: Vec3,
    /// Unit surface normal in world coordinates.
    pub normal_world: Vec3,
    /// Unit longitudinal tangent in world coordinates.
    pub longitudinal_tangent_world: Vec3,
    /// Tire-road friction multiplier at this patch.
    pub friction_scale: f64,
}

/// Maps one metric road patch to a backend-neutral cuboid pose and dimensions.
pub fn rigid_road_patch_geometry(
    patch: RigidRoadPatchSpec,
) -> Result<RigidRoadPatchGeometry, MobilityPlantEvaluationError> {
    if !patch.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    let rotation = Quat::from_rotation_z(patch.grade_rad);
    let longitudinal_tangent_world = rotation * Vec3::X;
    let normal_world = rotation * Vec3::Y;
    Ok(RigidRoadPatchGeometry {
        solid_transform: Transform3::from_translation_rotation(
            patch.surface_center_world_m - normal_world * (0.5 * patch.thickness_m),
            rotation,
        ),
        solid_half_extents_m: Vec3::new(
            0.5 * patch.surface_length_m,
            0.5 * patch.thickness_m,
            patch.half_width_m,
        ),
        longitudinal_tangent_world,
        normal_world,
    })
}

/// Samples the closest finite planar road patch containing a world location.
///
/// Selection is deterministic for overlapping patches: the surface with the
/// smallest absolute normal distance wins, followed by canonical patch index.
/// `None` is returned for a profile gap or a location outside road width.
pub fn sample_rigid_road_profile(
    profile: &RigidRoadProfileSpec,
    location_world_m: Vec3,
) -> Result<Option<RigidRoadSurfaceSample>, MobilityPlantEvaluationError> {
    if !profile.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !location_world_m.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    const BOUNDARY_TOLERANCE_M: f64 = 1.0e-9;
    let mut selected: Option<(f64, RigidRoadSurfaceSample)> = None;
    for (patch_index, patch) in profile.patches.iter().copied().enumerate() {
        let geometry = rigid_road_patch_geometry(patch)?;
        let relative = location_world_m - patch.surface_center_world_m;
        let longitudinal_m = relative.dot(geometry.longitudinal_tangent_world);
        let lateral_m = relative.z;
        if longitudinal_m.abs() > 0.5 * patch.surface_length_m + BOUNDARY_TOLERANCE_M
            || lateral_m.abs() > patch.half_width_m + BOUNDARY_TOLERANCE_M
        {
            continue;
        }
        let normal_distance_m = relative.dot(geometry.normal_world);
        let sample = RigidRoadSurfaceSample {
            patch_index,
            point_world_m: patch.surface_center_world_m
                + geometry.longitudinal_tangent_world * longitudinal_m
                + Vec3::Z * lateral_m,
            normal_world: geometry.normal_world,
            longitudinal_tangent_world: geometry.longitudinal_tangent_world,
            friction_scale: patch.friction_scale,
        };
        let distance = normal_distance_m.abs();
        if selected
            .as_ref()
            .is_none_or(|(best_distance, _)| distance < *best_distance)
        {
            selected = Some((distance, sample));
        }
    }
    Ok(selected.map(|(_, sample)| sample))
}

/// Completed world-frame geometry and rigid-carrier velocity for one wheel station.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelStationFrame {
    /// Wheel-center position in world coordinates, in meters.
    pub center_world_m: Vec3,
    /// Positive free-rolling unit axis in world coordinates.
    pub forward_world: Vec3,
    /// Positive axle/lateral unit axis in world coordinates.
    pub lateral_world: Vec3,
    /// Carrier velocity at the wheel center before wheel spin, in meters per second.
    pub carrier_velocity_world_m_s: Vec3,
}

/// Resolves one physical wheel station from completed rigid-body state.
///
/// Steering rotates the rolling and axle axes about the declared body-frame steering axis.
/// The carrier velocity includes the body's angular contribution at the station lever arm;
/// wheel circumference speed is intentionally excluded and is added exactly once by the
/// tire/drive-path evaluator.
pub fn resolve_wheel_station_frame(
    spec: WheelStationSpec,
    steering_rad: f64,
    body_transform: Transform3,
    body_linear_velocity_world_m_s: Vec3,
    body_angular_velocity_world_rad_s: Vec3,
) -> Result<WheelStationFrame, MobilityPlantEvaluationError> {
    if !spec.is_valid()
        || !steering_rad.is_finite()
        || steering_rad.abs() > spec.maximum_steering_rad
        || !body_transform.translation.is_finite()
        || !body_transform.rotation.is_finite()
        || !body_linear_velocity_world_m_s.is_finite()
        || !body_angular_velocity_world_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let rotation_length_squared = body_transform.rotation.length_squared();
    if !rotation_length_squared.is_finite() || rotation_length_squared <= 1.0e-18 {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    // Physics backends commonly store unit quaternions in f32. Normalize after
    // promotion to f64 so strict tire/contact axis validation does not interpret
    // harmless solver roundoff as a non-unit wheel frame.
    let body_rotation = body_transform.rotation.normalize();
    let steering = Quat::from_axis_angle(spec.steering_axis_body, steering_rad);
    let center_offset_world_m = body_rotation * spec.center_body_m;
    let forward_world = body_rotation * (steering * spec.zero_steer_forward_body);
    let lateral_world = body_rotation * (steering * spec.zero_steer_axle_body);
    let carrier_velocity_world_m_s = body_linear_velocity_world_m_s
        + body_angular_velocity_world_rad_s.cross(center_offset_world_m);
    Ok(WheelStationFrame {
        center_world_m: body_transform.translation + center_offset_world_m,
        forward_world,
        lateral_world,
        carrier_velocity_world_m_s,
    })
}

/// Completed force and state from one transient combined-slip tire step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombinedSlipTireEvaluation {
    /// State to retain for the next completed tire step.
    pub state: CombinedSlipTireState,
    /// Longitudinal force on the wheel in its positive-forward direction, in newtons.
    pub longitudinal_force_n: f64,
    /// Lateral force on the wheel in its positive-lateral direction, in newtons.
    pub lateral_force_n: f64,
    /// Load-sensitive longitudinal force limit after road scaling, in newtons.
    pub longitudinal_peak_force_n: f64,
    /// Load-sensitive lateral force limit after road scaling, in newtons.
    pub lateral_peak_force_n: f64,
    /// Combined utilization of the friction ellipse, bounded by one.
    pub friction_utilization: f64,
}

/// Completed contact and wheel-frame inputs for one combined-slip tire step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombinedSlipTireInput {
    /// Aggregated completed-step contact, or `None` when the wheel is lifted.
    pub patch: Option<WheelContactPatch>,
    /// Positive wheel-forward unit axis in world coordinates.
    pub forward_world: Vec3,
    /// Positive wheel-lateral unit axis in world coordinates.
    pub lateral_world: Vec3,
    /// Signed wheel circumference speed from its completed angular coordinate, in meters per second.
    pub wheel_circumferential_speed_m_s: f64,
    /// Non-negative road friction multiplier for this patch.
    pub road_friction_scale: f64,
}

/// Completed backend contact and command input for one driven-wheel path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalDrivePathInput {
    /// Completed contact evidence before wheel circumferential velocity is added.
    ///
    /// The patch velocity is the rigid wheel carrier's contact-point velocity
    /// relative to the road. The evaluator subtracts wheel circumferential speed
    /// along `forward_world` exactly once.
    pub carrier_patch: Option<WheelContactPatch>,
    /// Positive wheel-forward unit axis in world coordinates.
    pub forward_world: Vec3,
    /// Positive wheel-lateral unit axis in world coordinates.
    pub lateral_world: Vec3,
    /// Requested motor terminal voltage in volts.
    pub command_voltage_v: f64,
}

/// Completed motor-to-contact evaluation for one driven-wheel path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalDrivePathEvaluation {
    /// State to retain for the next completed path step.
    pub state: LongitudinalDrivePathState,
    /// Completed motor equivalent-circuit evaluation.
    pub motor: DcMotorEvaluation,
    /// Completed rigid transmission evaluation.
    pub transmission: TransmissionEvaluation,
    /// Completed transient tire-force evaluation.
    pub tire: CombinedSlipTireEvaluation,
    /// Completed motor telemetry suitable for a measurement frontend.
    pub motor_telemetry: DcMotorCompletedTelemetry,
    /// Wheel angular acceleration in radians per second squared.
    pub wheel_acceleration_rad_s2: f64,
    /// Rolling-resistance torque on the wheel in newton-meters.
    pub rolling_resistance_torque_nm: f64,
    /// Per-wheel force-at-contact for the next backend step, or `None` on lift.
    pub tire_wrench: Option<ExternalBodyWrench>,
}

/// Completed coupled motor, transmission, wheel, tire, and chassis step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalMobilityPlantEvaluation {
    /// State to retain for the next fixed step.
    pub state: LongitudinalMobilityPlantState,
    /// Completed motor equivalent-circuit evaluation.
    pub motor: DcMotorEvaluation,
    /// Completed rigid transmission evaluation.
    pub transmission: TransmissionEvaluation,
    /// Completed transient tire-force evaluation.
    pub tire: CombinedSlipTireEvaluation,
    /// Completed motor telemetry suitable for a measurement frontend.
    pub motor_telemetry: DcMotorCompletedTelemetry,
    /// Representative wheel angular acceleration in radians per second squared.
    pub wheel_acceleration_rad_s2: f64,
    /// Chassis longitudinal acceleration in meters per second squared.
    pub chassis_acceleration_m_s2: f64,
    /// Quadratic aerodynamic force on the chassis in newtons.
    pub aerodynamic_force_n: f64,
    /// Gravity force along the road, positive uphill resistance, in newtons.
    pub grade_resistance_force_n: f64,
    /// Rolling-resistance torque on one driven wheel in newton-meters.
    pub rolling_resistance_torque_nm: f64,
}

/// Advances one motor/transmission/wheel/tire path from completed contact evidence.
///
/// The backend owns chassis motion. This evaluator owns only electrical current,
/// wheel rotation, and transient tire slip, then returns a backend-neutral wrench
/// for one driven wheel. Contact evidence is necessarily from the completed step,
/// so the returned wrench is applied during the following step.
pub fn evaluate_longitudinal_drive_path(
    spec: LongitudinalMobilityPlantSpec,
    state: LongitudinalDrivePathState,
    input: LongitudinalDrivePathInput,
    dt_s: f64,
) -> Result<LongitudinalDrivePathEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.wheel_position_rad.is_finite()
        || !state.wheel_velocity_rad_s.is_finite()
        || !state.motor_state.current_a.is_finite()
        || !state.tire_state.longitudinal_slip_ratio.is_finite()
        || !state.tire_state.lateral_slip_tangent.is_finite()
        || !input.command_voltage_v.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let motor_velocity_rad_s =
        state.wheel_velocity_rad_s * spec.transmission.ratio_motor_rad_per_wheel_rad;
    let motor = evaluate_dc_motor(
        spec.motor,
        state.motor_state,
        input.command_voltage_v,
        motor_velocity_rad_s,
        dt_s,
    )?;
    let transmission = evaluate_transmission(
        spec.transmission,
        spec.motor.rotor_inertia_kg_m2,
        motor.shaft_torque_nm,
        state.wheel_velocity_rad_s,
    )?;
    let wheel_circumferential_speed_m_s = state.wheel_velocity_rad_s * spec.wheel.radius_m;
    let (contact_forward_world, contact_lateral_world) =
        input
            .carrier_patch
            .map_or(Ok((input.forward_world, input.lateral_world)), |patch| {
                contact_tangent_axes(
                    input.forward_world,
                    input.lateral_world,
                    patch.normal_road_to_wheel_world,
                )
            })?;
    let tire_patch = input.carrier_patch.map(|mut patch| {
        patch.wheel_relative_to_road_world_m_s -=
            contact_forward_world * wheel_circumferential_speed_m_s;
        patch
    });
    let tire = evaluate_combined_slip_tire(
        spec.tire,
        state.tire_state,
        CombinedSlipTireInput {
            patch: tire_patch,
            forward_world: contact_forward_world,
            lateral_world: contact_lateral_world,
            wheel_circumferential_speed_m_s,
            road_friction_scale: spec.road_friction_scale,
        },
        dt_s,
    )?;
    let total_wheel_inertia_kg_m2 =
        spec.wheel.inertia_kg_m2 + transmission.reflected_rotor_inertia_kg_m2;
    if total_wheel_inertia_kg_m2 <= 0.0 || !total_wheel_inertia_kg_m2.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    let torque_before_rolling_nm =
        transmission.wheel_torque_nm - tire.longitudinal_force_n * spec.wheel.radius_m;
    let wheel_velocity_before_rolling_rad_s =
        state.wheel_velocity_rad_s + torque_before_rolling_nm / total_wheel_inertia_kg_m2 * dt_s;
    let normal_load_n = tire_patch.map_or(0.0, |patch| patch.normal_load_n);
    let rolling_resistance_torque_nm = bounded_rolling_resistance_torque_nm(
        spec.wheel,
        normal_load_n,
        wheel_velocity_before_rolling_rad_s,
        total_wheel_inertia_kg_m2,
        dt_s,
    )?;
    let wheel_acceleration_rad_s2 =
        (torque_before_rolling_nm + rolling_resistance_torque_nm) / total_wheel_inertia_kg_m2;
    let wheel_velocity_rad_s = state.wheel_velocity_rad_s + wheel_acceleration_rad_s2 * dt_s;
    let next_state = LongitudinalDrivePathState {
        wheel_position_rad: state.wheel_position_rad + wheel_velocity_rad_s * dt_s,
        wheel_velocity_rad_s,
        motor_state: motor.state,
        tire_state: tire.state,
    };
    if !next_state.wheel_position_rad.is_finite() || !next_state.wheel_velocity_rad_s.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let tire_wrench = tire_patch
        .map(|patch| {
            combined_slip_tire_wrench(patch, tire, contact_forward_world, contact_lateral_world)
        })
        .transpose()?;

    Ok(LongitudinalDrivePathEvaluation {
        state: next_state,
        motor,
        transmission,
        tire,
        motor_telemetry: motor.completed_telemetry(spec.motor.failure_mode, None),
        wheel_acceleration_rad_s2,
        rolling_resistance_torque_nm,
        tire_wrench,
    })
}

/// Aggregates deterministic point-contact evidence for one wheel.
///
/// `forward_world` and `lateral_world` must be finite, unit length, and orthogonal.
/// Samples not containing `wheel_entity`, non-positive loads, and non-finite samples are
/// ignored. The returned surface velocity and normal always use the road-to-wheel
/// convention, independently of canonical entity ordering.
pub fn aggregate_wheel_contact_patch(
    wheel_entity: Entity,
    samples: &[ContactPointSample],
    forward_world: Vec3,
    lateral_world: Vec3,
) -> Result<Option<WheelContactPatch>, MobilityPlantEvaluationError> {
    const AXIS_TOLERANCE: f64 = 1.0e-6;
    if !forward_world.is_finite()
        || !lateral_world.is_finite()
        || (forward_world.length() - 1.0).abs() > AXIS_TOLERANCE
        || (lateral_world.length() - 1.0).abs() > AXIS_TOLERANCE
        || forward_world.dot(lateral_world).abs() > AXIS_TOLERANCE
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    let mut normal_load_n = 0.0;
    let mut weighted_point = Vec3::ZERO;
    let mut weighted_normal = Vec3::ZERO;
    let mut weighted_velocity = Vec3::ZERO;
    for sample in samples {
        if sample.normal_force_n <= 0.0
            || !sample.normal_force_n.is_finite()
            || !sample.point_world_m.is_finite()
            || !sample.normal_a_to_b.is_finite()
            || !sample.velocity_b_relative_to_a_world_m_s.is_finite()
        {
            continue;
        }
        let (normal_road_to_wheel, wheel_relative_to_road) = if sample.entity_a == wheel_entity {
            (
                -sample.normal_a_to_b,
                -sample.velocity_b_relative_to_a_world_m_s,
            )
        } else if sample.entity_b == wheel_entity {
            (
                sample.normal_a_to_b,
                sample.velocity_b_relative_to_a_world_m_s,
            )
        } else {
            continue;
        };
        let weight = sample.normal_force_n;
        normal_load_n += weight;
        weighted_point += sample.point_world_m * weight;
        weighted_normal += normal_road_to_wheel * weight;
        weighted_velocity += wheel_relative_to_road * weight;
    }
    if normal_load_n == 0.0 {
        return Ok(None);
    }
    let normal = weighted_normal.normalize_or_zero();
    if normal == Vec3::ZERO {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(Some(WheelContactPatch {
        wheel_entity,
        point_world_m: weighted_point / normal_load_n,
        normal_road_to_wheel_world: normal,
        wheel_relative_to_road_world_m_s: weighted_velocity / normal_load_n,
        normal_load_n,
    }))
}

/// Caps a normalized load ratio (`load_n / reference_load_n`) at `maximum_load_ratio`.
///
/// This is the load-sensitivity envelope clamp shared by [`CombinedSlipTireSpec`]
/// (evaluated in [`evaluate_combined_slip_tire`]) and
/// [`CorneringStiffnessLoadSensitivity`] (evaluated in [`vehicle_dynamics`]): both
/// treat load beyond this ratio as outside the model's identified validity range.
pub(crate) fn capped_load_ratio(
    load_n: f64,
    reference_load_n: f64,
    maximum_load_ratio: f64,
) -> f64 {
    (load_n / reference_load_n).min(maximum_load_ratio)
}

/// Evaluates one deterministic, identifiable transient combined-slip tire force.
///
/// The contact velocity already includes wheel rotation. Positive longitudinal slip is
/// therefore `-surface_velocity / (abs(circumferential_speed) + v_num)`, matching the
/// low-speed-safe convention used by handling-oriented tire models. `road_friction_scale`
/// enables spatial or randomized road friction without changing the identified tire spec.
pub fn evaluate_combined_slip_tire(
    spec: CombinedSlipTireSpec,
    state: CombinedSlipTireState,
    input: CombinedSlipTireInput,
    dt_s: f64,
) -> Result<CombinedSlipTireEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.longitudinal_slip_ratio.is_finite()
        || !state.lateral_slip_tangent.is_finite()
        || !input.wheel_circumferential_speed_m_s.is_finite()
        || !input.road_friction_scale.is_finite()
        || input.road_friction_scale < 0.0
        || !input.forward_world.is_finite()
        || !input.lateral_world.is_finite()
        || (input.forward_world.length() - 1.0).abs() > 1.0e-6
        || (input.lateral_world.length() - 1.0).abs() > 1.0e-6
        || input.forward_world.dot(input.lateral_world).abs() > 1.0e-6
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }
    let Some(patch) = input.patch else {
        return Ok(zero_tire_evaluation());
    };
    if !patch.point_world_m.is_finite()
        || !patch.normal_road_to_wheel_world.is_finite()
        || !patch.wheel_relative_to_road_world_m_s.is_finite()
        || !patch.normal_load_n.is_finite()
        || patch.normal_load_n <= 0.0
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    let (contact_forward_world, contact_lateral_world) = contact_tangent_axes(
        input.forward_world,
        input.lateral_world,
        patch.normal_road_to_wheel_world,
    )?;
    let transport_speed_m_s =
        input.wheel_circumferential_speed_m_s.abs() + spec.low_speed_regularization_m_s;
    let longitudinal_surface_speed_m_s = patch
        .wheel_relative_to_road_world_m_s
        .dot(contact_forward_world);
    let lateral_surface_speed_m_s = patch
        .wheel_relative_to_road_world_m_s
        .dot(contact_lateral_world);
    let target_longitudinal_slip_ratio = -longitudinal_surface_speed_m_s / transport_speed_m_s;
    let target_lateral_slip_tangent = -lateral_surface_speed_m_s / transport_speed_m_s;
    let next_longitudinal_slip = relax_slip(
        state.longitudinal_slip_ratio,
        target_longitudinal_slip_ratio,
        spec.longitudinal_relaxation_length_m,
        transport_speed_m_s,
        dt_s,
    );
    let next_lateral_slip = relax_slip(
        state.lateral_slip_tangent,
        target_lateral_slip_tangent,
        spec.lateral_relaxation_length_m,
        transport_speed_m_s,
        dt_s,
    );

    let load_ratio = capped_load_ratio(
        patch.normal_load_n,
        spec.reference_load_n,
        spec.maximum_load_ratio,
    );
    let friction_ratio = (1.0 - spec.load_sensitivity_per_load_ratio * (load_ratio - 1.0))
        .max(spec.minimum_friction_ratio);
    let longitudinal_peak_force_n = spec.longitudinal_peak_friction
        * friction_ratio
        * patch.normal_load_n
        * input.road_friction_scale;
    let lateral_peak_force_n = spec.lateral_peak_friction
        * friction_ratio
        * patch.normal_load_n
        * input.road_friction_scale;
    let raw_longitudinal_force_n =
        spec.longitudinal_stiffness_n * load_ratio * next_longitudinal_slip;
    let raw_lateral_force_n = spec.lateral_stiffness_n * load_ratio * next_lateral_slip;
    let normalized_longitudinal = if longitudinal_peak_force_n > 0.0 {
        raw_longitudinal_force_n / longitudinal_peak_force_n
    } else {
        0.0
    };
    let normalized_lateral = if lateral_peak_force_n > 0.0 {
        raw_lateral_force_n / lateral_peak_force_n
    } else {
        0.0
    };
    let demand = normalized_longitudinal.hypot(normalized_lateral);
    let force_scale = if demand > 1.0e-12 {
        demand.tanh() / demand
    } else {
        1.0
    };
    let force_scale = if input.road_friction_scale == 0.0 {
        0.0
    } else {
        force_scale
    };
    Ok(CombinedSlipTireEvaluation {
        state: CombinedSlipTireState {
            longitudinal_slip_ratio: next_longitudinal_slip,
            lateral_slip_tangent: next_lateral_slip,
        },
        longitudinal_force_n: raw_longitudinal_force_n * force_scale,
        lateral_force_n: raw_lateral_force_n * force_scale,
        longitudinal_peak_force_n,
        lateral_peak_force_n,
        friction_utilization: demand.tanh(),
    })
}

/// Numerical floor applied to a load-transfer-derived per-wheel normal load
/// before it reaches the tire patch, in newtons.
///
/// The tire law requires a strictly positive contact load. An analytically
/// unloaded axle (wheel lift) is clamped to zero for reporting and physical
/// interpretation, but the patch itself is floored just above zero so the
/// step still evaluates: the resulting tire force is negligible at this
/// floor, not physically meaningful.
const MINIMUM_DRIVEN_WHEEL_NORMAL_LOAD_N: f64 = 1.0e-6;

/// Derives the per-driven-wheel normal load from an analytic longitudinal
/// weight-transfer model, or returns the plant's constant static load when
/// `transfer` is `None`.
///
/// Implements `delta_F_z = m * a_x * h_cg / L`, applied to the driven axle's
/// static total load and split evenly across `spec.driven_wheel_count`
/// identical wheels. `chassis_acceleration_m_s2` already reflects the
/// plant's road grade and aerodynamic drag (see
/// [`evaluate_longitudinal_mobility_plant`]), so grade is accounted for
/// without a separate term here. The result is clamped to non-negative
/// before the wheel split, so a wheel never carries negative load; it is not
/// floored to a strictly positive value here (see
/// [`MINIMUM_DRIVEN_WHEEL_NORMAL_LOAD_N`] for the caller-side patch floor).
///
/// This is a rigid-body, no-suspension model with longitudinal transfer
/// only: no lateral/cornering transfer, and no measured-vehicle calibration.
pub(crate) fn resolve_driven_wheel_normal_load_n(
    spec: LongitudinalMobilityPlantSpec,
    transfer: Option<LongitudinalLoadTransferSpec>,
    chassis_acceleration_m_s2: f64,
) -> f64 {
    let Some(transfer) = transfer else {
        return spec.normal_load_per_driven_wheel_n;
    };
    let static_axle_load_n =
        spec.normal_load_per_driven_wheel_n * f64::from(spec.driven_wheel_count);
    let transfer_n = spec.vehicle_mass_kg * chassis_acceleration_m_s2 * transfer.cg_height_m
        / transfer.wheelbase_m;
    let signed_transfer_n = match transfer.driven_axle {
        // Forward acceleration shifts load onto the rear axle.
        DrivenAxle::Rear => transfer_n,
        // Forward acceleration shifts load off the front axle.
        DrivenAxle::Front => -transfer_n,
    };
    let dynamic_axle_load_n = (static_axle_load_n + signed_transfer_n).max(0.0);
    dynamic_axle_load_n / f64::from(spec.driven_wheel_count)
}

/// Advances a coupled straight-line motor-to-road plant by one fixed step.
///
/// The representative wheel obeys
/// `J_total * wheel_accel = transmission_torque + rolling_torque - radius * tire_force`.
/// The chassis obeys longitudinal force balance from every identical driven tire,
/// aerodynamic drag, and road grade. Wheel and chassis velocities are both dynamic
/// states, so traction and braking slip emerge rather than being prescribed.
///
/// When `spec.longitudinal_load_transfer` is set, the driven wheel's normal
/// load is derived from the chassis acceleration completed on the *previous*
/// step (`state.previous_chassis_acceleration_m_s2`) rather than held at
/// `spec.normal_load_per_driven_wheel_n`. This mirrors the plant's existing
/// semi-implicit integration, where contact evidence for a step is always
/// derived from completed motion, and avoids a circular solve within one
/// step. When the spec is absent, the plant is bit-for-bit identical to
/// before this field existed.
pub fn evaluate_longitudinal_mobility_plant(
    spec: LongitudinalMobilityPlantSpec,
    state: LongitudinalMobilityPlantState,
    command_voltage_v: f64,
    dt_s: f64,
) -> Result<LongitudinalMobilityPlantEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !command_voltage_v.is_finite()
        || !state.position_m.is_finite()
        || !state.velocity_m_s.is_finite()
        || !state.wheel_position_rad.is_finite()
        || !state.wheel_velocity_rad_s.is_finite()
        || !state.motor_state.current_a.is_finite()
        || !state.tire_state.longitudinal_slip_ratio.is_finite()
        || !state.tire_state.lateral_slip_tangent.is_finite()
        || !state.previous_chassis_acceleration_m_s2.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let driven_wheel_normal_load_n = resolve_driven_wheel_normal_load_n(
        spec,
        spec.longitudinal_load_transfer,
        state.previous_chassis_acceleration_m_s2,
    );
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
                normal_load_n: driven_wheel_normal_load_n.max(MINIMUM_DRIVEN_WHEEL_NORMAL_LOAD_N),
            }),
            forward_world: Vec3::X,
            lateral_world: Vec3::Z,
            command_voltage_v,
        },
        dt_s,
    )?;
    let aerodynamic_force_n =
        -spec.aerodynamic_drag_n_s2_m2 * state.velocity_m_s * state.velocity_m_s.abs();
    let grade_resistance_force_n = spec.vehicle_mass_kg * 9.806_65 * spec.road_grade_rad.sin();
    let chassis_acceleration_m_s2 = (f64::from(spec.driven_wheel_count)
        * drive.tire.longitudinal_force_n
        + aerodynamic_force_n
        - grade_resistance_force_n)
        / spec.vehicle_mass_kg;

    let velocity_m_s = state.velocity_m_s + chassis_acceleration_m_s2 * dt_s;
    let next_state = LongitudinalMobilityPlantState {
        position_m: state.position_m + velocity_m_s * dt_s,
        velocity_m_s,
        wheel_position_rad: drive.state.wheel_position_rad,
        wheel_velocity_rad_s: drive.state.wheel_velocity_rad_s,
        motor_state: drive.state.motor_state,
        tire_state: drive.state.tire_state,
        previous_chassis_acceleration_m_s2: chassis_acceleration_m_s2,
    };
    if !next_state.position_m.is_finite()
        || !next_state.velocity_m_s.is_finite()
        || !next_state.wheel_position_rad.is_finite()
        || !next_state.wheel_velocity_rad_s.is_finite()
        || !next_state.previous_chassis_acceleration_m_s2.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    Ok(LongitudinalMobilityPlantEvaluation {
        state: next_state,
        motor: drive.motor,
        transmission: drive.transmission,
        tire: drive.tire,
        motor_telemetry: drive.motor_telemetry,
        wheel_acceleration_rad_s2: drive.wheel_acceleration_rad_s2,
        chassis_acceleration_m_s2,
        aerodynamic_force_n,
        grade_resistance_force_n,
        rolling_resistance_torque_nm: drive.rolling_resistance_torque_nm,
    })
}

/// Converts a tire evaluation into the backend-neutral one-step wrench boundary.
pub fn combined_slip_tire_wrench(
    patch: WheelContactPatch,
    evaluation: CombinedSlipTireEvaluation,
    forward_world: Vec3,
    lateral_world: Vec3,
) -> Result<ExternalBodyWrench, MobilityPlantEvaluationError> {
    if !forward_world.is_finite()
        || !lateral_world.is_finite()
        || !evaluation.longitudinal_force_n.is_finite()
        || !evaluation.lateral_force_n.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let (contact_forward_world, contact_lateral_world) = contact_tangent_axes(
        forward_world,
        lateral_world,
        patch.normal_road_to_wheel_world,
    )?;
    let wrench = ExternalBodyWrench {
        entity: patch.wheel_entity,
        point_world_m: patch.point_world_m,
        force_world_n: contact_forward_world * evaluation.longitudinal_force_n
            + contact_lateral_world * evaluation.lateral_force_n,
        torque_world_nm: Vec3::ZERO,
    };
    if !wrench.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(wrench)
}

pub(crate) fn contact_tangent_axes(
    forward_world: Vec3,
    lateral_world: Vec3,
    normal_road_to_wheel_world: Vec3,
) -> Result<(Vec3, Vec3), MobilityPlantEvaluationError> {
    if !forward_world.is_finite()
        || !lateral_world.is_finite()
        || !normal_road_to_wheel_world.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let normal_length_squared = normal_road_to_wheel_world.length_squared();
    if normal_length_squared <= 1.0e-18 {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let normal = normal_road_to_wheel_world / normal_length_squared.sqrt();
    let projected_forward = forward_world - normal * forward_world.dot(normal);
    let projected_forward_length_squared = projected_forward.length_squared();
    if projected_forward_length_squared <= 1.0e-18 {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let contact_forward = projected_forward / projected_forward_length_squared.sqrt();
    let mut contact_lateral = contact_forward.cross(normal);
    if contact_lateral.dot(lateral_world) < 0.0 {
        contact_lateral = -contact_lateral;
    }
    Ok((contact_forward, contact_lateral))
}

pub(crate) fn relax_slip(
    current: f64,
    target: f64,
    length_m: f64,
    speed_m_s: f64,
    dt_s: f64,
) -> f64 {
    if length_m == 0.0 {
        target
    } else {
        let fraction = 1.0 - (-speed_m_s * dt_s / length_m).exp();
        current + fraction * (target - current)
    }
}

pub(crate) fn zero_tire_evaluation() -> CombinedSlipTireEvaluation {
    CombinedSlipTireEvaluation {
        state: CombinedSlipTireState::default(),
        longitudinal_force_n: 0.0,
        lateral_force_n: 0.0,
        longitudinal_peak_force_n: 0.0,
        lateral_peak_force_n: 0.0,
        friction_utilization: 0.0,
    }
}

/// Maps signed PWM command counts to an average motor-terminal voltage request.
///
/// The ideal switching-cycle average is `duty * bus_voltage`. The declared H-bridge
/// on-state loss is present only during the energized fraction, so the returned request is
/// `duty * max(bus_voltage - bridge_drop, 0)`. This deterministic control-oriented map does
/// not choose coast versus brake recirculation and must not be used to infer motor electrical
/// constants from command-count response data alone.
pub fn evaluate_pwm_motor_command(
    spec: PwmMotorCommandFrontendSpec,
    command_count: f64,
    bus_voltage_v: f64,
) -> Result<PwmMotorCommandEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !command_count.is_finite() || !bus_voltage_v.is_finite() || bus_voltage_v < 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }

    let command_saturated = command_count.abs() > spec.full_scale_command_count;
    let clamped_command_count = command_count.clamp(
        -spec.full_scale_command_count,
        spec.full_scale_command_count,
    );
    let polarity_sign = match spec.polarity {
        PwmMotorCommandPolarity::Normal => 1.0,
        PwmMotorCommandPolarity::Inverted => -1.0,
    };
    let signed_duty_ratio = polarity_sign * clamped_command_count / spec.full_scale_command_count;
    let ideal_average_voltage_v = signed_duty_ratio * bus_voltage_v;
    let available_on_state_voltage_v =
        (bus_voltage_v - spec.bridge_on_state_voltage_drop_v).max(0.0);
    let terminal_voltage_request_v = signed_duty_ratio * available_on_state_voltage_v;
    let average_bridge_loss_v =
        signed_duty_ratio.abs() * bus_voltage_v.min(spec.bridge_on_state_voltage_drop_v);

    Ok(PwmMotorCommandEvaluation {
        clamped_command_count,
        signed_duty_ratio,
        ideal_average_voltage_v,
        terminal_voltage_request_v,
        average_bridge_loss_v,
        command_saturated,
    })
}

/// Advances a backend-neutral first-order steering actuator by one fixed step.
///
/// The exact zero-order-hold first-order response is evaluated first, then
/// bounded by the measured steering-rate and travel limits. A command inside
/// the declared deadband holds the completed position. The returned position
/// is suitable as the target for a backend joint-position constraint; it is
/// not a measured steering angle or a torque-producing servo simulation.
pub fn evaluate_steering_actuator(
    spec: SteeringActuatorSpec,
    state: SteeringActuatorState,
    requested_target_rad: f64,
    dt_s: f64,
) -> Result<SteeringActuatorEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.position_rad.is_finite()
        || state.position_rad < spec.minimum_position_rad
        || state.position_rad > spec.maximum_position_rad
        || !requested_target_rad.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let clamped_target_rad =
        requested_target_rad.clamp(spec.minimum_position_rad, spec.maximum_position_rad);
    let command_saturated = clamped_target_rad != requested_target_rad;
    let stuck = spec.failure_mode == SteeringActuatorFailureMode::Stuck;
    let error_rad = clamped_target_rad - state.position_rad;
    let unconstrained_delta_rad = if stuck || error_rad.abs() <= spec.command_deadband_rad {
        0.0
    } else {
        error_rad * (1.0 - (-dt_s / spec.time_constant_s).exp())
    };
    let maximum_delta_rad = spec.maximum_rate_rad_s * dt_s;
    let delta_rad = unconstrained_delta_rad.clamp(-maximum_delta_rad, maximum_delta_rad);
    let rate_limited = delta_rad != unconstrained_delta_rad;
    let position_rad = (state.position_rad + delta_rad)
        .clamp(spec.minimum_position_rad, spec.maximum_position_rad);

    Ok(SteeringActuatorEvaluation {
        state: SteeringActuatorState { position_rad },
        requested_target_rad,
        clamped_target_rad,
        realized_rate_rad_s: (position_rad - state.position_rad) / dt_s,
        command_saturated,
        rate_limited,
        stuck,
    })
}

/// Identifies an unsaturated first-order command-to-steering-angle response.
///
/// The fit uses only the leading training transitions and evaluates a frozen
/// coefficient on the remaining holdout transitions. For each excited interval,
/// it solves `position[k+1] - position[k] = b * (command[k] - position[k])`,
/// then reports `tau = -dt / ln(1 - b)`. Samples must share one uniform capture
/// grid and remain inside declared travel. The caller must prequalify that the
/// selected acquisition excludes rate saturation, deadband, backlash, and stuck
/// faults; this function does not silently absorb those effects into `tau`.
pub fn identify_steering_actuator_first_order(
    spec: SteeringActuatorIdentificationSpec,
    samples: &[SteeringActuatorIdentificationSample],
) -> Result<SteeringActuatorIdentificationResult, SteeringActuatorIdentificationError> {
    if !spec.is_valid() {
        return Err(SteeringActuatorIdentificationError::InvalidSpec);
    }
    if samples.len() < 5 || spec.training_transition_count >= samples.len() - 1 {
        return Err(SteeringActuatorIdentificationError::InsufficientExcitation);
    }
    if samples.iter().any(|sample| {
        !sample.capture_time_s.is_finite()
            || !sample.command_target_rad.is_finite()
            || !sample.measured_position_rad.is_finite()
            || !(spec.minimum_position_rad..=spec.maximum_position_rad)
                .contains(&sample.command_target_rad)
            || !(spec.minimum_position_rad..=spec.maximum_position_rad)
                .contains(&sample.measured_position_rad)
    }) {
        return Err(SteeringActuatorIdentificationError::InvalidSample);
    }
    let capture_interval_s = samples[1].capture_time_s - samples[0].capture_time_s;
    if !capture_interval_s.is_finite() || capture_interval_s <= 0.0 {
        return Err(SteeringActuatorIdentificationError::InvalidSample);
    }
    for pair in samples.windows(2) {
        let interval_s = pair[1].capture_time_s - pair[0].capture_time_s;
        if !interval_s.is_finite()
            || interval_s <= 0.0
            || (interval_s - capture_interval_s).abs() > spec.interval_tolerance_s
        {
            return Err(SteeringActuatorIdentificationError::InvalidSample);
        }
    }

    let mut training_error_delta_sum = 0.0;
    let mut training_error_squared_sum = 0.0;
    let mut training_transition_count = 0;
    let mut holdout_transition_count = 0;
    for (index, pair) in samples.windows(2).enumerate() {
        let error_rad = pair[0].command_target_rad - pair[0].measured_position_rad;
        if error_rad.abs() < spec.minimum_abs_command_error_rad {
            continue;
        }
        if index < spec.training_transition_count {
            let delta_rad = pair[1].measured_position_rad - pair[0].measured_position_rad;
            training_error_delta_sum += error_rad * delta_rad;
            training_error_squared_sum += error_rad * error_rad;
            training_transition_count += 1;
        } else {
            holdout_transition_count += 1;
        }
    }
    if training_transition_count < 2 || holdout_transition_count < 2 {
        return Err(SteeringActuatorIdentificationError::InsufficientExcitation);
    }
    if !training_error_squared_sum.is_finite()
        || training_error_squared_sum <= f64::EPSILON
        || !training_error_delta_sum.is_finite()
    {
        return Err(SteeringActuatorIdentificationError::Unidentifiable);
    }
    let discrete_response_ratio = training_error_delta_sum / training_error_squared_sum;
    if !discrete_response_ratio.is_finite()
        || discrete_response_ratio <= 0.0
        || discrete_response_ratio >= 1.0
    {
        return Err(SteeringActuatorIdentificationError::Unidentifiable);
    }
    let time_constant_s = -capture_interval_s / (-discrete_response_ratio).ln_1p();
    if !time_constant_s.is_finite()
        || !(spec.minimum_time_constant_s..=spec.maximum_time_constant_s).contains(&time_constant_s)
    {
        return Err(SteeringActuatorIdentificationError::NonPhysicalResult);
    }

    let residual_sum = |training: bool| {
        samples
            .windows(2)
            .enumerate()
            .filter_map(|(index, pair)| {
                let error_rad = pair[0].command_target_rad - pair[0].measured_position_rad;
                ((error_rad.abs() >= spec.minimum_abs_command_error_rad)
                    && ((index < spec.training_transition_count) == training))
                    .then(|| {
                        let predicted_rad =
                            pair[0].measured_position_rad + discrete_response_ratio * error_rad;
                        (predicted_rad - pair[1].measured_position_rad).powi(2)
                    })
            })
            .sum::<f64>()
    };
    let training_rms_rad = (residual_sum(true) / training_transition_count as f64).sqrt();
    let holdout_rms_rad = (residual_sum(false) / holdout_transition_count as f64).sqrt();
    if !training_rms_rad.is_finite()
        || !holdout_rms_rad.is_finite()
        || training_rms_rad > spec.maximum_training_rms_rad
        || holdout_rms_rad > spec.maximum_holdout_rms_rad
    {
        return Err(SteeringActuatorIdentificationError::ResidualExceeded);
    }

    Ok(SteeringActuatorIdentificationResult {
        capture_interval_s,
        time_constant_s,
        discrete_response_ratio,
        training_transition_count,
        holdout_transition_count,
        training_rms_rad,
        holdout_rms_rad,
    })
}

#[derive(Clone, Copy)]
pub(crate) enum TireFitAxis {
    Longitudinal,
    Lateral,
}

/// Identifies steady tire stiffness and peak friction from pure-slip training runs.
///
/// Complete acquisitions are assigned by the caller to training or holdout and
/// may not share identities. Only pure-longitudinal and pure-lateral training
/// samples enter the deterministic bounded search. The fitted profile is then
/// frozen and evaluated on combined-slip holdout samples, both pooled and by
/// declared condition. Load sensitivity, road scale, low-speed regularization,
/// and relaxation lengths come from `template` and are never tuned on holdout.
#[allow(clippy::too_many_lines)] // TODO(cleanup): split (152/150 lines); see PR body
pub fn identify_combined_slip_tire_steady(
    identification: TireIdentificationSpec,
    template: CombinedSlipTireSpec,
    training_runs: &[TireIdentificationRun<'_>],
    holdout_runs: &[TireIdentificationRun<'_>],
) -> Result<CombinedSlipTireIdentificationResult, TireIdentificationError> {
    if !identification.is_valid() || !template.is_valid() {
        return Err(TireIdentificationError::InvalidSpec);
    }
    validate_tire_identification_runs(identification, template, training_runs, holdout_runs)?;

    let mut longitudinal = Vec::new();
    let mut lateral = Vec::new();
    for run in training_runs {
        for sample in run.samples {
            if sample.lateral_slip_tangent.abs() <= identification.pure_slip_tolerance
                && sample.longitudinal_slip_ratio.abs() >= identification.minimum_excited_slip
            {
                longitudinal.push((
                    sample.longitudinal_slip_ratio,
                    sample.normal_load_n,
                    run.road_friction_scale,
                    sample.longitudinal_force_n,
                ));
            }
            if sample.longitudinal_slip_ratio.abs() <= identification.pure_slip_tolerance
                && sample.lateral_slip_tangent.abs() >= identification.minimum_excited_slip
            {
                lateral.push((
                    sample.lateral_slip_tangent,
                    sample.normal_load_n,
                    run.road_friction_scale,
                    sample.lateral_force_n,
                ));
            }
        }
    }
    let excited = |samples: &[(f64, f64, f64, f64)]| {
        samples.len() >= identification.minimum_training_samples_per_axis
            && samples
                .iter()
                .any(|sample| sample.0.abs() <= identification.maximum_linear_slip)
            && samples
                .iter()
                .any(|sample| sample.0.abs() >= identification.minimum_peak_slip)
    };
    if !excited(&longitudinal) || !excited(&lateral) {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let work = (longitudinal.len() + lateral.len())
        .saturating_mul(identification.grid_points_per_axis)
        .saturating_mul(identification.grid_points_per_axis)
        .saturating_mul(identification.refinement_passes);
    if work > 10_000_000 {
        return Err(TireIdentificationError::InvalidSpec);
    }

    let (longitudinal_stiffness_n, longitudinal_peak_friction) = fit_tire_axis(
        identification,
        template,
        TireFitAxis::Longitudinal,
        &longitudinal,
    )?;
    let (lateral_stiffness_n, lateral_peak_friction) =
        fit_tire_axis(identification, template, TireFitAxis::Lateral, &lateral)?;
    let tire_spec = CombinedSlipTireSpec {
        longitudinal_stiffness_n,
        lateral_stiffness_n,
        longitudinal_peak_friction,
        lateral_peak_friction,
        ..template
    };
    if !tire_spec.is_valid() {
        return Err(TireIdentificationError::NonPhysicalResult);
    }

    let longitudinal_squared_error = axis_squared_error(
        template,
        TireFitAxis::Longitudinal,
        longitudinal_stiffness_n,
        longitudinal_peak_friction,
        &longitudinal,
    );
    let lateral_squared_error = axis_squared_error(
        template,
        TireFitAxis::Lateral,
        lateral_stiffness_n,
        lateral_peak_friction,
        &lateral,
    );
    let training_rms_n = ((longitudinal_squared_error + lateral_squared_error)
        / (longitudinal.len() + lateral.len()) as f64)
        .sqrt();

    let mut conditions = std::collections::BTreeMap::<u64, (usize, f64)>::new();
    let mut holdout_count = 0_usize;
    let mut holdout_squared_error = 0.0;
    for run in holdout_runs {
        for sample in run.samples {
            if sample.longitudinal_slip_ratio.abs() < identification.minimum_excited_slip
                || sample.lateral_slip_tangent.abs() < identification.minimum_excited_slip
            {
                continue;
            }
            let (predicted_longitudinal_n, predicted_lateral_n) = steady_tire_forces(
                tire_spec,
                sample.longitudinal_slip_ratio,
                sample.lateral_slip_tangent,
                sample.normal_load_n,
                run.road_friction_scale,
            );
            let squared_error = (predicted_longitudinal_n - sample.longitudinal_force_n).powi(2)
                + (predicted_lateral_n - sample.lateral_force_n).powi(2);
            if !squared_error.is_finite() {
                return Err(TireIdentificationError::NonPhysicalResult);
            }
            holdout_count += 1;
            holdout_squared_error += squared_error;
            let condition = conditions.entry(run.condition_id).or_default();
            condition.0 += 1;
            condition.1 += squared_error;
        }
    }
    if holdout_count < identification.minimum_combined_holdout_samples
        || conditions.len() < 2
        || conditions.values().any(|(sample_count, _)| {
            *sample_count < identification.minimum_holdout_samples_per_condition
        })
    {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let holdout_rms_n = (holdout_squared_error / holdout_count as f64).sqrt();
    let condition_residuals = conditions
        .into_iter()
        .map(
            |(condition_id, (sample_count, squared_error))| TireConditionResidual {
                condition_id,
                sample_count,
                vector_force_rms_n: (squared_error / sample_count as f64).sqrt(),
            },
        )
        .collect::<Vec<_>>();
    let worst_condition_rms_n = condition_residuals
        .iter()
        .map(|condition| condition.vector_force_rms_n)
        .fold(0.0_f64, f64::max);
    if !training_rms_n.is_finite()
        || !holdout_rms_n.is_finite()
        || training_rms_n > identification.maximum_training_rms_n
        || holdout_rms_n > identification.maximum_holdout_rms_n
        || worst_condition_rms_n > identification.maximum_worst_condition_rms_n
    {
        return Err(TireIdentificationError::ResidualExceeded);
    }

    Ok(CombinedSlipTireIdentificationResult {
        tire_spec,
        longitudinal_training_sample_count: longitudinal.len(),
        lateral_training_sample_count: lateral.len(),
        training_rms_n,
        holdout_rms_n,
        condition_residuals,
    })
}

type TireLoadSensitivityObservation = (f64, f64, f64, f64, f64, f64, u64);

/// Identifies the load-dependent peak-friction slope after the steady tire fit is frozen.
///
/// Only combined-slip training samples whose normal loads bracket the reference load enter the
/// deterministic one-parameter search. Complete acquisitions remain assigned to one split,
/// and the returned coefficient is evaluated without refitting on pooled and per-condition
/// holdout samples. The function never estimates road friction, stiffness, reference-load peak
/// friction, the minimum-friction clamp, or relaxation length.
pub fn identify_tire_load_sensitivity(
    spec: TireLoadSensitivityIdentificationSpec,
    frozen_tire: CombinedSlipTireSpec,
    training_runs: &[TireIdentificationRun<'_>],
    holdout_runs: &[TireIdentificationRun<'_>],
) -> Result<TireLoadSensitivityIdentificationResult, TireIdentificationError> {
    if !spec.is_valid() || !frozen_tire.is_valid() {
        return Err(TireIdentificationError::InvalidSpec);
    }
    let (training, holdout) =
        load_sensitivity_observations(spec, frozen_tire, training_runs, holdout_runs)?;
    if training.len() < spec.minimum_training_samples
        || holdout.len() < spec.minimum_holdout_samples
    {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let minimum_training_load_ratio = training
        .iter()
        .map(|sample| sample.2 / frozen_tire.reference_load_n)
        .fold(f64::INFINITY, f64::min);
    let maximum_training_load_ratio = training
        .iter()
        .map(|sample| sample.2 / frozen_tire.reference_load_n)
        .fold(f64::NEG_INFINITY, f64::max);
    if minimum_training_load_ratio >= 1.0
        || maximum_training_load_ratio <= 1.0
        || maximum_training_load_ratio - minimum_training_load_ratio
            < spec.minimum_training_load_ratio_span
    {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let maximum_observed_load_ratio = training
        .iter()
        .chain(&holdout)
        .map(|sample| sample.2 / frozen_tire.reference_load_n)
        .fold(0.0_f64, f64::max);
    let maximum_candidate = spec.load_sensitivity_bounds_per_load_ratio[1];
    if 1.0 - maximum_candidate * (maximum_observed_load_ratio - 1.0)
        <= frozen_tire.minimum_friction_ratio
    {
        return Err(TireIdentificationError::InvalidSpec);
    }
    let work = training
        .len()
        .saturating_mul(spec.grid_points)
        .saturating_mul(spec.refinement_passes);
    if work > 10_000_000 {
        return Err(TireIdentificationError::InvalidSpec);
    }

    let load_sensitivity_per_load_ratio = fit_load_sensitivity(spec, frozen_tire, &training)?;
    let tire_spec = CombinedSlipTireSpec {
        load_sensitivity_per_load_ratio,
        ..frozen_tire
    };
    if !tire_spec.is_valid() {
        return Err(TireIdentificationError::NonPhysicalResult);
    }
    let training_squared_error = load_sensitivity_squared_error(tire_spec, &training);
    let holdout_squared_error = load_sensitivity_squared_error(tire_spec, &holdout);
    if !training_squared_error.is_finite() || !holdout_squared_error.is_finite() {
        return Err(TireIdentificationError::NonPhysicalResult);
    }
    let training_rms_n = (training_squared_error / training.len() as f64).sqrt();
    let holdout_rms_n = (holdout_squared_error / holdout.len() as f64).sqrt();
    let mut conditions = std::collections::BTreeMap::<u64, (usize, f64)>::new();
    for sample in &holdout {
        let squared_error = load_sensitivity_sample_squared_error(tire_spec, *sample);
        let condition = conditions.entry(sample.6).or_default();
        condition.0 += 1;
        condition.1 += squared_error;
    }
    if conditions.len() < 2
        || conditions
            .values()
            .any(|(count, _)| *count < spec.minimum_holdout_samples_per_condition)
    {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let condition_residuals = conditions
        .into_iter()
        .map(
            |(condition_id, (sample_count, squared_error))| TireConditionResidual {
                condition_id,
                sample_count,
                vector_force_rms_n: (squared_error / sample_count as f64).sqrt(),
            },
        )
        .collect::<Vec<_>>();
    let worst_condition_rms_n = condition_residuals
        .iter()
        .map(|condition| condition.vector_force_rms_n)
        .fold(0.0_f64, f64::max);
    if !training_rms_n.is_finite()
        || !holdout_rms_n.is_finite()
        || !worst_condition_rms_n.is_finite()
    {
        return Err(TireIdentificationError::NonPhysicalResult);
    }
    if training_rms_n > spec.maximum_training_rms_n
        || holdout_rms_n > spec.maximum_holdout_rms_n
        || worst_condition_rms_n > spec.maximum_worst_condition_rms_n
    {
        return Err(TireIdentificationError::ResidualExceeded);
    }

    Ok(TireLoadSensitivityIdentificationResult {
        tire_spec,
        load_sensitivity_per_load_ratio,
        minimum_training_load_ratio,
        maximum_training_load_ratio,
        training_sample_count: training.len(),
        holdout_sample_count: holdout.len(),
        training_rms_n,
        holdout_rms_n,
        condition_residuals,
    })
}

pub(crate) fn load_sensitivity_observations(
    spec: TireLoadSensitivityIdentificationSpec,
    tire: CombinedSlipTireSpec,
    training_runs: &[TireIdentificationRun<'_>],
    holdout_runs: &[TireIdentificationRun<'_>],
) -> Result<
    (
        Vec<TireLoadSensitivityObservation>,
        Vec<TireLoadSensitivityObservation>,
    ),
    TireIdentificationError,
> {
    if training_runs.is_empty() || holdout_runs.is_empty() {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let mut acquisition_ids = std::collections::BTreeSet::new();
    let mut total_samples = 0_usize;
    let collect = |runs: &[TireIdentificationRun<'_>],
                   acquisition_ids: &mut std::collections::BTreeSet<u64>,
                   total_samples: &mut usize|
     -> Result<Vec<TireLoadSensitivityObservation>, TireIdentificationError> {
        let mut observations = Vec::new();
        for run in runs {
            if !acquisition_ids.insert(run.acquisition_id) {
                return Err(TireIdentificationError::DuplicateAcquisition);
            }
            if run.samples.is_empty()
                || !run.road_friction_scale.is_finite()
                || run.road_friction_scale <= 0.0
            {
                return Err(TireIdentificationError::InvalidSample);
            }
            *total_samples = (*total_samples).saturating_add(run.samples.len());
            if *total_samples > 100_000 {
                return Err(TireIdentificationError::InvalidSpec);
            }
            let mut previous_time_s = None;
            for sample in run.samples {
                if [
                    sample.capture_time_s,
                    sample.longitudinal_slip_ratio,
                    sample.lateral_slip_tangent,
                    sample.normal_load_n,
                    sample.longitudinal_force_n,
                    sample.lateral_force_n,
                ]
                .iter()
                .any(|value| !value.is_finite())
                    || sample.normal_load_n <= 0.0
                    || sample.normal_load_n > tire.reference_load_n * tire.maximum_load_ratio
                    || sample.longitudinal_slip_ratio.abs() > spec.maximum_abs_slip
                    || sample.lateral_slip_tangent.abs() > spec.maximum_abs_slip
                    || previous_time_s.is_some_and(|time| sample.capture_time_s <= time)
                {
                    return Err(TireIdentificationError::InvalidSample);
                }
                previous_time_s = Some(sample.capture_time_s);
                if sample.longitudinal_slip_ratio.abs() >= spec.minimum_combined_axis_slip
                    && sample.lateral_slip_tangent.abs() >= spec.minimum_combined_axis_slip
                {
                    observations.push((
                        sample.longitudinal_slip_ratio,
                        sample.lateral_slip_tangent,
                        sample.normal_load_n,
                        run.road_friction_scale,
                        sample.longitudinal_force_n,
                        sample.lateral_force_n,
                        run.condition_id,
                    ));
                }
            }
        }
        Ok(observations)
    };
    let training = collect(training_runs, &mut acquisition_ids, &mut total_samples)?;
    let holdout = collect(holdout_runs, &mut acquisition_ids, &mut total_samples)?;
    Ok((training, holdout))
}

pub(crate) fn fit_load_sensitivity(
    spec: TireLoadSensitivityIdentificationSpec,
    frozen_tire: CombinedSlipTireSpec,
    training: &[TireLoadSensitivityObservation],
) -> Result<f64, TireIdentificationError> {
    let original = spec.load_sensitivity_bounds_per_load_ratio;
    let mut bounds = original;
    let divisions = (spec.grid_points - 1) as f64;
    let mut best = (bounds[0], f64::INFINITY);
    for _ in 0..spec.refinement_passes {
        let step = (bounds[1] - bounds[0]) / divisions;
        for index in 0..spec.grid_points {
            let candidate = bounds[0] + step * index as f64;
            let tire = CombinedSlipTireSpec {
                load_sensitivity_per_load_ratio: candidate,
                ..frozen_tire
            };
            let squared_error = load_sensitivity_squared_error(tire, training);
            if squared_error < best.1 {
                best = (candidate, squared_error);
            }
        }
        if !best.1.is_finite() {
            return Err(TireIdentificationError::NonPhysicalResult);
        }
        bounds = [
            (best.0 - step).max(original[0]),
            (best.0 + step).min(original[1]),
        ];
    }
    Ok(best.0)
}

pub(crate) fn load_sensitivity_squared_error(
    tire: CombinedSlipTireSpec,
    observations: &[TireLoadSensitivityObservation],
) -> f64 {
    observations
        .iter()
        .map(|sample| load_sensitivity_sample_squared_error(tire, *sample))
        .sum()
}

pub(crate) fn load_sensitivity_sample_squared_error(
    tire: CombinedSlipTireSpec,
    sample: TireLoadSensitivityObservation,
) -> f64 {
    let predicted = steady_tire_forces(tire, sample.0, sample.1, sample.2, sample.3);
    (predicted.0 - sample.4).powi(2) + (predicted.1 - sample.5).powi(2)
}

type TireRelaxationTransition = (f64, f64, f64, f64, f64, u64);

/// Identifies one tire-slip relaxation length from complete transient acquisitions.
///
/// Row `i` supplies measured axis force, load, road scale, transport speed, and the
/// zero-order-held kinematic target for the interval ending at row `i + 1`. The
/// already-frozen pure-slip steady force law is inverted below the declared force
/// utilization limit to reconstruct the relaxation state. Complete acquisition IDs
/// may occur only once across training and holdout. The fit uses training transitions
/// only; pooled and per-condition holdout residuals are computed after freezing the length.
pub fn identify_tire_relaxation_length(
    spec: TireRelaxationIdentificationSpec,
    tire: CombinedSlipTireSpec,
    axis: TireRelaxationAxis,
    training_runs: &[TireRelaxationIdentificationRun<'_>],
    holdout_runs: &[TireRelaxationIdentificationRun<'_>],
) -> Result<TireRelaxationIdentificationResult, TireRelaxationIdentificationError> {
    if !spec.is_valid()
        || evaluate_combined_slip_tire_steady_force(tire, 0.0, 0.0, tire.reference_load_n, 1.0)
            .is_err()
    {
        return Err(TireRelaxationIdentificationError::InvalidSpec);
    }
    if training_runs.is_empty() || holdout_runs.is_empty() {
        return Err(TireRelaxationIdentificationError::InsufficientExcitation);
    }
    let mut acquisition_ids = std::collections::BTreeSet::new();
    let mut total_samples = 0_usize;
    for run in training_runs.iter().chain(holdout_runs) {
        if !acquisition_ids.insert(run.acquisition_id) {
            return Err(TireRelaxationIdentificationError::DuplicateAcquisition);
        }
        if run.samples.len() < 2 {
            return Err(TireRelaxationIdentificationError::InsufficientExcitation);
        }
        total_samples = total_samples.saturating_add(run.samples.len());
        if total_samples > 100_000 {
            return Err(TireRelaxationIdentificationError::InvalidSpec);
        }
        for (index, sample) in run.samples.iter().enumerate() {
            if !sample.capture_time_s.is_finite()
                || !sample.transport_speed_m_s.is_finite()
                || sample.transport_speed_m_s <= 0.0
                || !sample.target_slip.is_finite()
                || !sample.normal_load_n.is_finite()
                || sample.normal_load_n <= 0.0
                || sample.normal_load_n > tire.reference_load_n * tire.maximum_load_ratio
                || !sample.road_friction_scale.is_finite()
                || sample.road_friction_scale <= 0.0
                || !sample.measured_force_n.is_finite()
                || sample.target_slip.abs() > spec.maximum_abs_slip
                || index > 0 && sample.capture_time_s <= run.samples[index - 1].capture_time_s
            {
                return Err(TireRelaxationIdentificationError::InvalidSample);
            }
        }
    }

    let training = tire_relaxation_transitions(spec, tire, axis, training_runs)?;
    let holdout = tire_relaxation_transitions(spec, tire, axis, holdout_runs)?;
    if training.len() < spec.minimum_training_transitions
        || holdout.len() < spec.minimum_holdout_transitions
    {
        return Err(TireRelaxationIdentificationError::InsufficientExcitation);
    }
    let work = training
        .len()
        .saturating_mul(spec.grid_points)
        .saturating_mul(spec.refinement_passes);
    if work > 10_000_000 {
        return Err(TireRelaxationIdentificationError::InvalidSpec);
    }

    let relaxation_length_m = fit_tire_relaxation_length(spec, &training)?;
    let training_squared_error = tire_relaxation_squared_error(relaxation_length_m, &training);
    let holdout_squared_error = tire_relaxation_squared_error(relaxation_length_m, &holdout);
    if !training_squared_error.is_finite() || !holdout_squared_error.is_finite() {
        return Err(TireRelaxationIdentificationError::NonPhysicalResult);
    }
    let training_rms_slip = (training_squared_error / training.len() as f64).sqrt();
    let holdout_rms_slip = (holdout_squared_error / holdout.len() as f64).sqrt();
    let mut conditions = std::collections::BTreeMap::<u64, (usize, f64)>::new();
    for transition in &holdout {
        let error = tire_relaxation_prediction(relaxation_length_m, *transition) - transition.4;
        let condition = conditions.entry(transition.5).or_default();
        condition.0 += 1;
        condition.1 += error * error;
    }
    if conditions.len() < 2
        || conditions
            .values()
            .any(|(count, _)| *count < spec.minimum_holdout_transitions_per_condition)
    {
        return Err(TireRelaxationIdentificationError::InsufficientExcitation);
    }
    let condition_residuals = conditions
        .into_iter()
        .map(
            |(condition_id, (transition_count, squared_error))| TireRelaxationConditionResidual {
                condition_id,
                transition_count,
                rms_slip: (squared_error / transition_count as f64).sqrt(),
            },
        )
        .collect::<Vec<_>>();
    let worst_condition_rms_slip = condition_residuals
        .iter()
        .map(|condition| condition.rms_slip)
        .fold(0.0_f64, f64::max);
    if !training_rms_slip.is_finite()
        || !holdout_rms_slip.is_finite()
        || !worst_condition_rms_slip.is_finite()
    {
        return Err(TireRelaxationIdentificationError::NonPhysicalResult);
    }
    if training_rms_slip > spec.maximum_training_rms_slip
        || holdout_rms_slip > spec.maximum_holdout_rms_slip
        || worst_condition_rms_slip > spec.maximum_worst_condition_rms_slip
    {
        return Err(TireRelaxationIdentificationError::ResidualExceeded);
    }

    Ok(TireRelaxationIdentificationResult {
        relaxation_length_m,
        training_transition_count: training.len(),
        holdout_transition_count: holdout.len(),
        training_rms_slip,
        holdout_rms_slip,
        condition_residuals,
    })
}

pub(crate) fn tire_relaxation_transitions(
    spec: TireRelaxationIdentificationSpec,
    tire: CombinedSlipTireSpec,
    axis: TireRelaxationAxis,
    runs: &[TireRelaxationIdentificationRun<'_>],
) -> Result<Vec<TireRelaxationTransition>, TireRelaxationIdentificationError> {
    let mut transitions = Vec::new();
    for run in runs {
        for pair in run.samples.windows(2) {
            let current = pair[0];
            let next = pair[1];
            let current_slip = tire_relaxation_observed_slip(spec, tire, axis, current)?;
            let next_slip = tire_relaxation_observed_slip(spec, tire, axis, next)?;
            if current.transport_speed_m_s < spec.minimum_transport_speed_m_s
                || (current.target_slip - current_slip).abs() < spec.minimum_slip_excitation
            {
                continue;
            }
            transitions.push((
                current_slip,
                current.target_slip,
                current.transport_speed_m_s,
                next.capture_time_s - current.capture_time_s,
                next_slip,
                run.condition_id,
            ));
        }
    }
    Ok(transitions)
}

pub(crate) fn tire_relaxation_observed_slip(
    identification: TireRelaxationIdentificationSpec,
    tire: CombinedSlipTireSpec,
    axis: TireRelaxationAxis,
    sample: TireRelaxationIdentificationSample,
) -> Result<f64, TireRelaxationIdentificationError> {
    let load_ratio = (sample.normal_load_n / tire.reference_load_n).min(tire.maximum_load_ratio);
    let friction_ratio = (1.0 - tire.load_sensitivity_per_load_ratio * (load_ratio - 1.0))
        .max(tire.minimum_friction_ratio);
    let (stiffness_n, peak_friction) = match axis {
        TireRelaxationAxis::Longitudinal => (
            tire.longitudinal_stiffness_n,
            tire.longitudinal_peak_friction,
        ),
        TireRelaxationAxis::Lateral => (tire.lateral_stiffness_n, tire.lateral_peak_friction),
    };
    let peak_force_n =
        peak_friction * friction_ratio * sample.normal_load_n * sample.road_friction_scale;
    let utilization = sample.measured_force_n / peak_force_n;
    if !utilization.is_finite() || utilization.abs() > identification.maximum_force_utilization {
        return Err(TireRelaxationIdentificationError::InvalidSample);
    }
    let slip = utilization.atanh() * peak_force_n / (stiffness_n * load_ratio);
    if !slip.is_finite() || slip.abs() > identification.maximum_abs_slip {
        return Err(TireRelaxationIdentificationError::InvalidSample);
    }
    Ok(slip)
}

pub(crate) fn fit_tire_relaxation_length(
    spec: TireRelaxationIdentificationSpec,
    transitions: &[TireRelaxationTransition],
) -> Result<f64, TireRelaxationIdentificationError> {
    let original = spec.relaxation_length_bounds_m;
    let mut bounds = original;
    let divisions = (spec.grid_points - 1) as f64;
    let mut best = (bounds[0], f64::INFINITY);
    for _ in 0..spec.refinement_passes {
        let step = (bounds[1] - bounds[0]) / divisions;
        for index in 0..spec.grid_points {
            let length_m = bounds[0] + step * index as f64;
            let squared_error = tire_relaxation_squared_error(length_m, transitions);
            if squared_error < best.1 {
                best = (length_m, squared_error);
            }
        }
        if !best.1.is_finite() {
            return Err(TireRelaxationIdentificationError::NonPhysicalResult);
        }
        bounds = [
            (best.0 - step).max(original[0]),
            (best.0 + step).min(original[1]),
        ];
    }
    Ok(best.0)
}

pub(crate) fn tire_relaxation_squared_error(
    relaxation_length_m: f64,
    transitions: &[TireRelaxationTransition],
) -> f64 {
    transitions
        .iter()
        .map(|transition| {
            let error = tire_relaxation_prediction(relaxation_length_m, *transition) - transition.4;
            error * error
        })
        .sum()
}

pub(crate) fn tire_relaxation_prediction(
    relaxation_length_m: f64,
    transition: TireRelaxationTransition,
) -> f64 {
    let (current, target, speed_m_s, dt_s, _, _) = transition;
    target + (current - target) * (-speed_m_s * dt_s / relaxation_length_m).exp()
}

pub(crate) fn validate_tire_identification_runs(
    identification: TireIdentificationSpec,
    template: CombinedSlipTireSpec,
    training_runs: &[TireIdentificationRun<'_>],
    holdout_runs: &[TireIdentificationRun<'_>],
) -> Result<(), TireIdentificationError> {
    if training_runs.is_empty() || holdout_runs.is_empty() {
        return Err(TireIdentificationError::InsufficientExcitation);
    }
    let mut acquisition_ids = std::collections::BTreeSet::new();
    let mut total_samples = 0_usize;
    for run in training_runs.iter().chain(holdout_runs) {
        if !acquisition_ids.insert(run.acquisition_id) {
            return Err(TireIdentificationError::DuplicateAcquisition);
        }
        if run.samples.is_empty()
            || !run.road_friction_scale.is_finite()
            || run.road_friction_scale <= 0.0
        {
            return Err(TireIdentificationError::InvalidSample);
        }
        total_samples = total_samples.saturating_add(run.samples.len());
        if total_samples > 100_000 {
            return Err(TireIdentificationError::InvalidSpec);
        }
        let mut previous_time_s = None;
        for sample in run.samples {
            if [
                sample.capture_time_s,
                sample.longitudinal_slip_ratio,
                sample.lateral_slip_tangent,
                sample.normal_load_n,
                sample.longitudinal_force_n,
                sample.lateral_force_n,
            ]
            .iter()
            .any(|value| !value.is_finite())
                || sample.longitudinal_slip_ratio.abs() > identification.maximum_abs_slip
                || sample.lateral_slip_tangent.abs() > identification.maximum_abs_slip
                || sample.normal_load_n <= 0.0
                || sample.normal_load_n > template.reference_load_n * template.maximum_load_ratio
                || previous_time_s.is_some_and(|previous| sample.capture_time_s <= previous)
            {
                return Err(TireIdentificationError::InvalidSample);
            }
            previous_time_s = Some(sample.capture_time_s);
        }
    }
    Ok(())
}

pub(crate) fn fit_tire_axis(
    identification: TireIdentificationSpec,
    template: CombinedSlipTireSpec,
    axis: TireFitAxis,
    samples: &[(f64, f64, f64, f64)],
) -> Result<(f64, f64), TireIdentificationError> {
    let mut stiffness_bounds = match axis {
        TireFitAxis::Longitudinal => identification.longitudinal_stiffness_bounds_n,
        TireFitAxis::Lateral => identification.lateral_stiffness_bounds_n,
    };
    let original_stiffness_bounds = stiffness_bounds;
    let mut friction_bounds = match axis {
        TireFitAxis::Longitudinal => identification.longitudinal_peak_friction_bounds,
        TireFitAxis::Lateral => identification.lateral_peak_friction_bounds,
    };
    let original_friction_bounds = friction_bounds;
    let divisions = (identification.grid_points_per_axis - 1) as f64;
    let mut best = (stiffness_bounds[0], friction_bounds[0], f64::INFINITY);
    for _ in 0..identification.refinement_passes {
        let stiffness_step = (stiffness_bounds[1] - stiffness_bounds[0]) / divisions;
        let friction_step = (friction_bounds[1] - friction_bounds[0]) / divisions;
        for stiffness_index in 0..identification.grid_points_per_axis {
            let stiffness_n = stiffness_bounds[0] + stiffness_step * stiffness_index as f64;
            for friction_index in 0..identification.grid_points_per_axis {
                let peak_friction = friction_bounds[0] + friction_step * friction_index as f64;
                let squared_error =
                    axis_squared_error(template, axis, stiffness_n, peak_friction, samples);
                if squared_error < best.2 {
                    best = (stiffness_n, peak_friction, squared_error);
                }
            }
        }
        if !best.2.is_finite() {
            return Err(TireIdentificationError::NonPhysicalResult);
        }
        stiffness_bounds = [
            (best.0 - stiffness_step).max(original_stiffness_bounds[0]),
            (best.0 + stiffness_step).min(original_stiffness_bounds[1]),
        ];
        friction_bounds = [
            (best.1 - friction_step).max(original_friction_bounds[0]),
            (best.1 + friction_step).min(original_friction_bounds[1]),
        ];
    }
    Ok((best.0, best.1))
}

pub(crate) fn axis_squared_error(
    template: CombinedSlipTireSpec,
    axis: TireFitAxis,
    stiffness_n: f64,
    peak_friction: f64,
    samples: &[(f64, f64, f64, f64)],
) -> f64 {
    samples
        .iter()
        .map(|(slip, normal_load_n, road_scale, measured_force_n)| {
            let (longitudinal_slip, lateral_slip) = match axis {
                TireFitAxis::Longitudinal => (*slip, 0.0),
                TireFitAxis::Lateral => (0.0, *slip),
            };
            let mut candidate = template;
            match axis {
                TireFitAxis::Longitudinal => {
                    candidate.longitudinal_stiffness_n = stiffness_n;
                    candidate.longitudinal_peak_friction = peak_friction;
                }
                TireFitAxis::Lateral => {
                    candidate.lateral_stiffness_n = stiffness_n;
                    candidate.lateral_peak_friction = peak_friction;
                }
            }
            let predicted = steady_tire_forces(
                candidate,
                longitudinal_slip,
                lateral_slip,
                *normal_load_n,
                *road_scale,
            );
            let predicted_force_n = match axis {
                TireFitAxis::Longitudinal => predicted.0,
                TireFitAxis::Lateral => predicted.1,
            };
            (predicted_force_n - measured_force_n).powi(2)
        })
        .sum()
}

pub(crate) fn steady_tire_forces(
    spec: CombinedSlipTireSpec,
    longitudinal_slip_ratio: f64,
    lateral_slip_tangent: f64,
    normal_load_n: f64,
    road_friction_scale: f64,
) -> (f64, f64) {
    let load_ratio = (normal_load_n / spec.reference_load_n).min(spec.maximum_load_ratio);
    let friction_ratio = (1.0 - spec.load_sensitivity_per_load_ratio * (load_ratio - 1.0))
        .max(spec.minimum_friction_ratio);
    let longitudinal_peak_n =
        spec.longitudinal_peak_friction * friction_ratio * normal_load_n * road_friction_scale;
    let lateral_peak_n =
        spec.lateral_peak_friction * friction_ratio * normal_load_n * road_friction_scale;
    let raw_longitudinal_n = spec.longitudinal_stiffness_n * load_ratio * longitudinal_slip_ratio;
    let raw_lateral_n = spec.lateral_stiffness_n * load_ratio * lateral_slip_tangent;
    let demand = (raw_longitudinal_n / longitudinal_peak_n).hypot(raw_lateral_n / lateral_peak_n);
    let scale = if demand > 1.0e-12 {
        demand.tanh() / demand
    } else {
        1.0
    };
    (raw_longitudinal_n * scale, raw_lateral_n * scale)
}

/// Evaluates the steady-state force law used by combined-slip identification.
///
/// Slip coordinates are dimensionless, load and returned forces are newtons, and
/// `road_friction_scale` is a positive dimensionless multiplier. This function
/// excludes relaxation dynamics and therefore must not be used as transient evidence.
pub fn evaluate_combined_slip_tire_steady_force(
    spec: CombinedSlipTireSpec,
    longitudinal_slip_ratio: f64,
    lateral_slip_tangent: f64,
    normal_load_n: f64,
    road_friction_scale: f64,
) -> Result<(f64, f64), MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !longitudinal_slip_ratio.is_finite()
        || !lateral_slip_tangent.is_finite()
        || !normal_load_n.is_finite()
        || normal_load_n <= 0.0
        || !road_friction_scale.is_finite()
        || road_friction_scale <= 0.0
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(steady_tire_forces(
        spec,
        longitudinal_slip_ratio,
        lateral_slip_tangent,
        normal_load_n,
        road_friction_scale,
    ))
}
