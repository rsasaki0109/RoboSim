use super::*;

/// Invalid configuration or input supplied to a mobility-plant evaluator.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum MobilityPlantEvaluationError {
    /// A model specification failed its physical validity checks.
    #[error("invalid mobility plant specification")]
    InvalidSpec,
    /// A command or completed-state input was non-finite or outside its physical domain.
    #[error("invalid mobility plant input")]
    InvalidInput,
    /// The fixed step was zero, negative, or non-finite.
    #[error("mobility plant timestep must be finite and positive")]
    InvalidTimeStep,
}

/// Failure returned by first-order steering-actuator identification.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq, Serialize, Deserialize)]
pub enum SteeringActuatorIdentificationError {
    /// Bounds, split, timing tolerance, or residual gates are invalid.
    #[error("invalid steering actuator identification specification")]
    InvalidSpec,
    /// A sample is non-finite, unordered, nonuniform, or outside declared travel.
    #[error("invalid steering actuator identification sample")]
    InvalidSample,
    /// Training or holdout lacks enough command-error excitation.
    #[error("insufficient steering actuator identification excitation")]
    InsufficientExcitation,
    /// The fitted discrete response is not a stable first-order lag.
    #[error("steering actuator response is not an identifiable stable first-order lag")]
    Unidentifiable,
    /// The fitted time constant falls outside the declared physical bounds.
    #[error("identified steering actuator time constant is outside declared bounds")]
    NonPhysicalResult,
    /// Training or holdout residual exceeds its declared bound.
    #[error("steering actuator identification residual exceeds its declared bound")]
    ResidualExceeded,
}

/// One synchronized command/direct-angle sample for steering identification.
///
/// `command_target_rad` is the target held from this capture until the next
/// sample. `measured_position_rad` must be a direct steering-angle measurement,
/// not a command echo or pose-derived proxy.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringActuatorIdentificationSample {
    /// Monotonic capture time in seconds within one acquisition.
    pub capture_time_s: f64,
    /// Steering target held over the following interval, in radians.
    pub command_target_rad: f64,
    /// Direct completed steering-angle measurement in radians.
    pub measured_position_rad: f64,
}

/// Frozen split and acceptance gates for first-order steering identification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringActuatorIdentificationSpec {
    /// Number of leading transitions used exclusively for fitting.
    pub training_transition_count: usize,
    /// Allowed absolute deviation from the first capture interval, in seconds.
    pub interval_tolerance_s: f64,
    /// Minimum absolute command error retained as an excited transition, in radians.
    pub minimum_abs_command_error_rad: f64,
    /// Minimum accepted time constant in seconds.
    pub minimum_time_constant_s: f64,
    /// Maximum accepted time constant in seconds.
    pub maximum_time_constant_s: f64,
    /// Maximum training one-step RMS angle residual in radians.
    pub maximum_training_rms_rad: f64,
    /// Maximum holdout one-step RMS angle residual in radians.
    pub maximum_holdout_rms_rad: f64,
    /// Minimum declared steering travel in radians.
    pub minimum_position_rad: f64,
    /// Maximum declared steering travel in radians.
    pub maximum_position_rad: f64,
}

impl SteeringActuatorIdentificationSpec {
    pub(crate) fn is_valid(&self) -> bool {
        [
            self.interval_tolerance_s,
            self.minimum_abs_command_error_rad,
            self.minimum_time_constant_s,
            self.maximum_time_constant_s,
            self.maximum_training_rms_rad,
            self.maximum_holdout_rms_rad,
            self.minimum_position_rad,
            self.maximum_position_rad,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.training_transition_count >= 2
            && self.interval_tolerance_s >= 0.0
            && self.minimum_abs_command_error_rad > 0.0
            && self.minimum_time_constant_s > 0.0
            && self.minimum_time_constant_s < self.maximum_time_constant_s
            && self.maximum_training_rms_rad >= 0.0
            && self.maximum_holdout_rms_rad >= 0.0
            && self.minimum_position_rad < self.maximum_position_rad
            && self.minimum_abs_command_error_rad
                < self.maximum_position_rad - self.minimum_position_rad
    }
}

/// Accepted first-order steering fit and frozen holdout evidence.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SteeringActuatorIdentificationResult {
    /// Uniform capture interval used by the discrete fit, in seconds.
    pub capture_interval_s: f64,
    /// Fitted continuous-time first-order time constant in seconds.
    pub time_constant_s: f64,
    /// Fitted discrete response fraction `1 - exp(-dt / tau)`.
    pub discrete_response_ratio: f64,
    /// Excited training transitions used by the fit.
    pub training_transition_count: usize,
    /// Excited holdout transitions evaluated without refitting.
    pub holdout_transition_count: usize,
    /// Training one-step RMS angle residual in radians.
    pub training_rms_rad: f64,
    /// Holdout one-step RMS angle residual in radians.
    pub holdout_rms_rad: f64,
}

/// Failure returned by steady combined-slip tire identification.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq, Serialize, Deserialize)]
pub enum TireIdentificationError {
    /// Bounds, excitation thresholds, search budget, or residual gates are invalid.
    #[error("invalid tire identification specification")]
    InvalidSpec,
    /// A run or sample is non-finite, unordered, duplicated, or outside the model envelope.
    #[error("invalid tire identification sample")]
    InvalidSample,
    /// Pure-slip training or combined-slip holdout excitation is insufficient.
    #[error("insufficient tire identification excitation")]
    InsufficientExcitation,
    /// The deterministic search produced a non-finite or out-of-bounds profile.
    #[error("tire identification produced a nonphysical result")]
    NonPhysicalResult,
    /// Training, pooled holdout, or worst-condition residual exceeds its declared gate.
    #[error("tire identification residual exceeds its declared bound")]
    ResidualExceeded,
    /// An acquisition identity is duplicated across training and holdout.
    #[error("tire acquisition IDs must be unique across split roles")]
    DuplicateAcquisition,
}

/// One synchronized steady tire-force observation.
///
/// Slip coordinates must use the same signs and relaxed coordinates as
/// [`CombinedSlipTireState`]. Forces must be independently measured or derived
/// from a declared force-measurement system; trajectory agreement alone is not
/// accepted as tire-force evidence.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireForceIdentificationSample {
    /// Monotonic capture time in seconds within one acquisition.
    pub capture_time_s: f64,
    /// Relaxed longitudinal slip ratio.
    pub longitudinal_slip_ratio: f64,
    /// Relaxed tangent of lateral slip angle.
    pub lateral_slip_tangent: f64,
    /// Independently measured normal load in newtons.
    pub normal_load_n: f64,
    /// Independently measured longitudinal tire force in newtons.
    pub longitudinal_force_n: f64,
    /// Independently measured lateral tire force in newtons.
    pub lateral_force_n: f64,
}

/// One complete tire-force acquisition with a declared road condition.
///
/// Acquisition identity is caller-supplied and is not proof of independent
/// collection. `road_friction_scale` must come from the acquisition contract,
/// not be tuned on holdout residuals.
#[derive(Clone, Copy, Debug)]
pub struct TireIdentificationRun<'a> {
    /// Stable identity unique across both split roles.
    pub acquisition_id: u64,
    /// Stable road/tire condition identity used for worst-condition reporting.
    pub condition_id: u64,
    /// Independently declared road-friction scale applied to this run.
    pub road_friction_scale: f64,
    /// Samples in strict capture order.
    pub samples: &'a [TireForceIdentificationSample],
}

/// Frozen search, excitation, and residual contract for tire identification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireIdentificationSpec {
    /// Inclusive longitudinal stiffness bounds in newtons.
    pub longitudinal_stiffness_bounds_n: [f64; 2],
    /// Inclusive lateral stiffness bounds in newtons.
    pub lateral_stiffness_bounds_n: [f64; 2],
    /// Inclusive longitudinal peak-friction bounds.
    pub longitudinal_peak_friction_bounds: [f64; 2],
    /// Inclusive lateral peak-friction bounds.
    pub lateral_peak_friction_bounds: [f64; 2],
    /// Maximum absolute cross-axis slip for a pure-slip training sample.
    pub pure_slip_tolerance: f64,
    /// Minimum absolute primary-axis slip retained for fitting.
    pub minimum_excited_slip: f64,
    /// Maximum absolute slip counted as small-slip excitation.
    pub maximum_linear_slip: f64,
    /// Minimum absolute slip counted as peak-region excitation.
    pub minimum_peak_slip: f64,
    /// Maximum absolute slip coordinate admitted by this identification envelope.
    pub maximum_abs_slip: f64,
    /// Minimum retained pure-slip training samples per axis.
    pub minimum_training_samples_per_axis: usize,
    /// Minimum combined-slip holdout samples over all runs.
    pub minimum_combined_holdout_samples: usize,
    /// Minimum retained combined-slip samples in every holdout condition.
    pub minimum_holdout_samples_per_condition: usize,
    /// Grid points on each parameter axis per refinement pass.
    pub grid_points_per_axis: usize,
    /// Deterministic coarse-to-fine refinement passes.
    pub refinement_passes: usize,
    /// Maximum pure-slip training force RMS in newtons.
    pub maximum_training_rms_n: f64,
    /// Maximum pooled combined-slip holdout vector-force RMS in newtons.
    pub maximum_holdout_rms_n: f64,
    /// Maximum combined-slip RMS for any declared holdout condition, in newtons.
    pub maximum_worst_condition_rms_n: f64,
}

impl TireIdentificationSpec {
    pub(crate) fn is_valid(&self) -> bool {
        let bounds_valid = |bounds: [f64; 2]| {
            bounds.iter().all(|value| value.is_finite()) && bounds[0] > 0.0 && bounds[0] < bounds[1]
        };
        bounds_valid(self.longitudinal_stiffness_bounds_n)
            && bounds_valid(self.lateral_stiffness_bounds_n)
            && bounds_valid(self.longitudinal_peak_friction_bounds)
            && bounds_valid(self.lateral_peak_friction_bounds)
            && [
                self.pure_slip_tolerance,
                self.minimum_excited_slip,
                self.maximum_linear_slip,
                self.minimum_peak_slip,
                self.maximum_abs_slip,
                self.maximum_training_rms_n,
                self.maximum_holdout_rms_n,
                self.maximum_worst_condition_rms_n,
            ]
            .iter()
            .all(|value| value.is_finite())
            && self.pure_slip_tolerance >= 0.0
            && self.minimum_excited_slip > 0.0
            && self.minimum_excited_slip <= self.maximum_linear_slip
            && self.maximum_linear_slip < self.minimum_peak_slip
            && self.minimum_peak_slip < self.maximum_abs_slip
            && self.pure_slip_tolerance < self.minimum_excited_slip
            && self.minimum_training_samples_per_axis >= 4
            && self.minimum_combined_holdout_samples >= 2
            && self.minimum_holdout_samples_per_condition >= 2
            && self.minimum_combined_holdout_samples / 2
                >= self.minimum_holdout_samples_per_condition
            && (5..=33).contains(&self.grid_points_per_axis)
            && self.grid_points_per_axis % 2 == 1
            && (1..=8).contains(&self.refinement_passes)
            && self.maximum_training_rms_n >= 0.0
            && self.maximum_holdout_rms_n >= 0.0
            && self.maximum_worst_condition_rms_n >= 0.0
    }
}

/// Combined-slip holdout residual for one declared road/tire condition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireConditionResidual {
    /// Stable condition identity from the input runs.
    pub condition_id: u64,
    /// Retained combined-slip sample count.
    pub sample_count: usize,
    /// Vector-force RMS `sqrt(mean(ex^2 + ey^2))` in newtons.
    pub vector_force_rms_n: f64,
}

/// Identified tire profile with frozen pure/combined split evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CombinedSlipTireIdentificationResult {
    /// Template with the four identified steady-force parameters replaced.
    pub tire_spec: CombinedSlipTireSpec,
    /// Retained pure-longitudinal training sample count.
    pub longitudinal_training_sample_count: usize,
    /// Retained pure-lateral training sample count.
    pub lateral_training_sample_count: usize,
    /// Pure-slip force RMS pooled across both fitted axes, in newtons.
    pub training_rms_n: f64,
    /// Combined-slip holdout vector-force RMS pooled across conditions, in newtons.
    pub holdout_rms_n: f64,
    /// Per-condition combined-slip residuals in ascending condition-ID order.
    pub condition_residuals: Vec<TireConditionResidual>,
}

/// Frozen search and acceptance contract for the tire load-sensitivity stage.
///
/// The preceding steady fit supplies stiffness and reference-load peak friction. This stage
/// changes only [`CombinedSlipTireSpec::load_sensitivity_per_load_ratio`] using combined-slip
/// samples that bracket the reference load; road friction remains an independently supplied
/// run input.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityIdentificationSpec {
    /// Inclusive bounds for fractional friction loss per unit load ratio.
    pub load_sensitivity_bounds_per_load_ratio: [f64; 2],
    /// Minimum absolute slip required on each axis for a retained combined-slip sample.
    pub minimum_combined_axis_slip: f64,
    /// Maximum absolute slip admitted on either axis.
    pub maximum_abs_slip: f64,
    /// Minimum span of retained training `normal_load / reference_load` values.
    pub minimum_training_load_ratio_span: f64,
    /// Minimum retained training samples.
    pub minimum_training_samples: usize,
    /// Minimum retained pooled holdout samples.
    pub minimum_holdout_samples: usize,
    /// Minimum retained holdout samples in every condition.
    pub minimum_holdout_samples_per_condition: usize,
    /// Odd grid width for each deterministic refinement pass.
    pub grid_points: usize,
    /// Number of deterministic coarse-to-fine refinement passes.
    pub refinement_passes: usize,
    /// Maximum vector-force RMS on training samples, in newtons.
    pub maximum_training_rms_n: f64,
    /// Maximum pooled vector-force RMS on holdout samples, in newtons.
    pub maximum_holdout_rms_n: f64,
    /// Maximum vector-force RMS in any holdout condition, in newtons.
    pub maximum_worst_condition_rms_n: f64,
}

impl TireLoadSensitivityIdentificationSpec {
    pub(crate) fn is_valid(self) -> bool {
        self.load_sensitivity_bounds_per_load_ratio
            .iter()
            .all(|value| value.is_finite())
            && self.load_sensitivity_bounds_per_load_ratio[0] >= 0.0
            && self.load_sensitivity_bounds_per_load_ratio[0]
                < self.load_sensitivity_bounds_per_load_ratio[1]
            && self.load_sensitivity_bounds_per_load_ratio[1] < 1.0
            && [
                self.minimum_combined_axis_slip,
                self.maximum_abs_slip,
                self.minimum_training_load_ratio_span,
                self.maximum_training_rms_n,
                self.maximum_holdout_rms_n,
                self.maximum_worst_condition_rms_n,
            ]
            .iter()
            .all(|value| value.is_finite())
            && self.minimum_combined_axis_slip > 0.0
            && self.minimum_combined_axis_slip < self.maximum_abs_slip
            && self.minimum_training_load_ratio_span > 0.0
            && self.minimum_training_samples >= 4
            && self.minimum_holdout_samples_per_condition >= 2
            && self.minimum_holdout_samples / 2 >= self.minimum_holdout_samples_per_condition
            && (5..=101).contains(&self.grid_points)
            && self.grid_points % 2 == 1
            && (1..=10).contains(&self.refinement_passes)
            && self.maximum_training_rms_n >= 0.0
            && self.maximum_holdout_rms_n >= 0.0
            && self.maximum_worst_condition_rms_n >= 0.0
    }
}

/// Frozen load-sensitivity fit and train/holdout residual evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityIdentificationResult {
    /// Input tire with only the load-sensitivity coefficient replaced.
    pub tire_spec: CombinedSlipTireSpec,
    /// Identified fractional friction loss per unit load ratio.
    pub load_sensitivity_per_load_ratio: f64,
    /// Minimum retained training load ratio.
    pub minimum_training_load_ratio: f64,
    /// Maximum retained training load ratio.
    pub maximum_training_load_ratio: f64,
    /// Retained training sample count.
    pub training_sample_count: usize,
    /// Retained holdout sample count.
    pub holdout_sample_count: usize,
    /// Training vector-force RMS in newtons.
    pub training_rms_n: f64,
    /// Pooled holdout vector-force RMS in newtons.
    pub holdout_rms_n: f64,
    /// Deterministically ordered per-condition holdout residuals.
    pub condition_residuals: Vec<TireConditionResidual>,
}

/// Failure returned by transient tire relaxation-length identification.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum TireRelaxationIdentificationError {
    /// Bounds, gates, or deterministic work limits are invalid.
    #[error("invalid tire relaxation identification specification")]
    InvalidSpec,
    /// A run contains invalid timing, speed, or slip data.
    #[error("invalid tire relaxation identification sample")]
    InvalidSample,
    /// Training or holdout does not contain the required transient excitation.
    #[error("insufficient tire relaxation excitation")]
    InsufficientExcitation,
    /// An acquisition identity occurs in more than one split entry.
    #[error("duplicate tire relaxation acquisition identity")]
    DuplicateAcquisition,
    /// Candidate evaluation produced non-finite arithmetic.
    #[error("non-physical tire relaxation result")]
    NonPhysicalResult,
    /// Training, pooled holdout, or worst-condition residual exceeded its gate.
    #[error("tire relaxation residual exceeded")]
    ResidualExceeded,
}

/// Tire-force axis whose relaxation length is identified.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireRelaxationAxis {
    /// Longitudinal slip ratio and longitudinal force.
    Longitudinal,
    /// Lateral slip tangent and lateral force.
    Lateral,
}

/// One physically observable input/force row for the first-order relaxation law.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationIdentificationSample {
    /// Source capture time in seconds.
    pub capture_time_s: f64,
    /// Positive regularized transport speed used by the tire law, in meters per second.
    pub transport_speed_m_s: f64,
    /// Kinematic slip input held until the next row.
    pub target_slip: f64,
    /// Contact-normal load at this row, in newtons.
    pub normal_load_n: f64,
    /// Independently established dimensionless road-friction multiplier.
    pub road_friction_scale: f64,
    /// Measured force along the selected tire axis, in newtons.
    pub measured_force_n: f64,
}

/// One complete transient acquisition assigned wholly to training or holdout.
#[derive(Clone, Copy, Debug)]
pub struct TireRelaxationIdentificationRun<'a> {
    /// Stable acquisition identity.
    pub acquisition_id: u64,
    /// Stable speed/road/tire condition identity used for worst-case scoring.
    pub condition_id: u64,
    /// Strictly ordered rows from this acquisition.
    pub samples: &'a [TireRelaxationIdentificationSample],
}

/// Bounds and acceptance gates for relaxation-length identification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationIdentificationSpec {
    /// Inclusive search bounds for relaxation length, in meters.
    pub relaxation_length_bounds_m: [f64; 2],
    /// Minimum transport speed retained in a transition, in meters per second.
    pub minimum_transport_speed_m_s: f64,
    /// Minimum absolute target/current slip difference retained as excitation.
    pub minimum_slip_excitation: f64,
    /// Maximum absolute target or observed slip admitted by the fit.
    pub maximum_abs_slip: f64,
    /// Maximum absolute measured-force/peak-force ratio admitted for stable inversion.
    pub maximum_force_utilization: f64,
    /// Minimum retained training transitions.
    pub minimum_training_transitions: usize,
    /// Minimum retained combined holdout transitions.
    pub minimum_holdout_transitions: usize,
    /// Minimum retained transitions in every holdout condition.
    pub minimum_holdout_transitions_per_condition: usize,
    /// Odd grid width for each deterministic refinement pass.
    pub grid_points: usize,
    /// Number of deterministic coarse-to-fine refinement passes.
    pub refinement_passes: usize,
    /// Maximum training slip RMS.
    pub maximum_training_rms_slip: f64,
    /// Maximum pooled holdout slip RMS.
    pub maximum_holdout_rms_slip: f64,
    /// Maximum holdout slip RMS in any declared condition.
    pub maximum_worst_condition_rms_slip: f64,
}

impl TireRelaxationIdentificationSpec {
    pub(crate) fn is_valid(self) -> bool {
        self.relaxation_length_bounds_m
            .iter()
            .all(|value| value.is_finite())
            && self.relaxation_length_bounds_m[0] > 0.0
            && self.relaxation_length_bounds_m[0] < self.relaxation_length_bounds_m[1]
            && [
                self.minimum_transport_speed_m_s,
                self.minimum_slip_excitation,
                self.maximum_abs_slip,
                self.maximum_force_utilization,
                self.maximum_training_rms_slip,
                self.maximum_holdout_rms_slip,
                self.maximum_worst_condition_rms_slip,
            ]
            .iter()
            .all(|value| value.is_finite())
            && self.minimum_transport_speed_m_s > 0.0
            && self.minimum_slip_excitation > 0.0
            && self.minimum_slip_excitation < self.maximum_abs_slip
            && self.maximum_force_utilization > 0.0
            && self.maximum_force_utilization < 1.0
            && self.minimum_training_transitions >= 4
            && self.minimum_holdout_transitions_per_condition >= 2
            && self.minimum_holdout_transitions / 2
                >= self.minimum_holdout_transitions_per_condition
            && (5..=101).contains(&self.grid_points)
            && self.grid_points % 2 == 1
            && (1..=10).contains(&self.refinement_passes)
            && self.maximum_training_rms_slip >= 0.0
            && self.maximum_holdout_rms_slip >= 0.0
            && self.maximum_worst_condition_rms_slip >= 0.0
    }
}

/// Holdout slip residual for one declared transient condition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationConditionResidual {
    /// Stable condition identity.
    pub condition_id: u64,
    /// Retained transition count.
    pub transition_count: usize,
    /// One-step relaxed-slip RMS.
    pub rms_slip: f64,
}

/// Frozen relaxation-length fit and train/holdout diagnostics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationIdentificationResult {
    /// Identified relaxation length in meters.
    pub relaxation_length_m: f64,
    /// Retained training transition count.
    pub training_transition_count: usize,
    /// Retained holdout transition count.
    pub holdout_transition_count: usize,
    /// Training one-step slip RMS.
    pub training_rms_slip: f64,
    /// Pooled holdout one-step slip RMS.
    pub holdout_rms_slip: f64,
    /// Deterministically ordered per-condition residuals.
    pub condition_residuals: Vec<TireRelaxationConditionResidual>,
}

/// Failure returned by deterministic suspension-force identification.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq, Serialize, Deserialize)]
pub enum SuspensionIdentificationError {
    /// The split, parameter bounds, or residual bounds are invalid.
    #[error("invalid suspension identification specification")]
    InvalidSpec,
    /// A sample is non-finite, unordered, or outside the declared force model.
    #[error("invalid suspension identification sample")]
    InvalidSample,
    /// The input does not contain enough training and holdout samples.
    #[error("insufficient suspension identification samples")]
    InsufficientSamples,
    /// Position and velocity excitation cannot identify all three coefficients.
    #[error("suspension identification design matrix is rank deficient")]
    RankDeficient,
    /// The fitted stiffness, damping, or equilibrium position is outside physical bounds.
    #[error("identified suspension parameters are outside declared physical bounds")]
    NonPhysicalResult,
    /// Training or holdout residuals are non-finite or exceed acceptance bounds.
    #[error("suspension identification residual exceeds its declared bound")]
    ResidualExceeded,
    /// An acquisition ID appears more than once, including across split roles.
    #[error("suspension acquisition IDs must be unique across training and holdout")]
    DuplicateAcquisition,
}

/// One complete acquisition with its own strictly increasing capture clock.
///
/// Identity is caller-supplied, not an attestation of independent acquisition.
#[derive(Clone, Copy, Debug)]
pub struct SuspensionIdentificationRun<'a> {
    /// Stable acquisition identity, unique across both split roles.
    pub acquisition_id: u64,
    /// SI samples in capture order; clocks may restart between acquisitions.
    pub samples: &'a [SuspensionForceSample],
}

/// Physical coefficients from a training-only fit, without a residual verdict.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTrainingCoefficients {
    /// Spring stiffness in newtons per meter.
    pub stiffness_n_per_m: f64,
    /// Damping in newton-seconds per meter.
    pub damping_n_s_per_m: f64,
    /// Unloaded equilibrium coordinate in meters.
    pub equilibrium_position_m: f64,
}

/// One whole-acquisition deletion, retaining unsuccessful coefficient fits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAcquisitionInfluence {
    /// Training acquisition omitted from this refit.
    pub omitted_acquisition_id: u64,
    /// Remaining coefficients, or the exact fitting failure.
    pub coefficients: Result<SuspensionTrainingCoefficients, SuspensionIdentificationError>,
}

/// Training-only deletion diagnostics; not a confidence interval or acceptance gate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTrainingInfluence {
    /// Fit using every training acquisition; failures are retained.
    pub baseline: Result<SuspensionTrainingCoefficients, SuspensionIdentificationError>,
    /// Exactly one refit per input acquisition, in caller order.
    pub deletions: Vec<SuspensionAcquisitionInfluence>,
}
