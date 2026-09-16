//! Robot control systems.

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState,
    CorneringStiffnessLoadSensitivity, DcMotorCompletedTelemetry, DcMotorFailureMode, DcMotorSpec,
    DcMotorState, DrivenAxle, Joint, JointKind, LongitudinalDrivePathState,
    LongitudinalLoadTransferSpec, LongitudinalMobilityPlantSpec, LongitudinalMobilityPlantState,
    MultirotorFlight, PwmMotorCommandFrontendSpec, PwmMotorCommandPolarity, RigidRoadPatchSpec,
    RigidRoadProfileSpec, SteeringActuatorFailureMode, SteeringActuatorSpec, SteeringActuatorState,
    SuspensionStrutSpec, TransmissionSpec, VehicleDynamics, WheelAssemblySpec, WheelStationSpec,
};
use crate::diff_drive::DifferentialDrive;
use crate::joint::{validate_joint_position, validate_joint_velocity, JointValidationError};
use bevy_ecs::prelude::{Entity, World};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ContactPointSample, ExternalBodyWrench, JointActuation, JointMotor,
    RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

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
    fn is_valid(&self) -> bool {
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
    fn is_valid(&self) -> bool {
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
    fn is_valid(self) -> bool {
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
    fn is_valid(self) -> bool {
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

fn validate_suspension_training_runs(
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

fn validate_suspension_samples(
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
fn fit_suspension_training_coefficients(
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

fn identify_suspension_split(
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

fn valid_finite_bounds(bounds: [f64; 2]) -> bool {
    bounds.into_iter().all(f64::is_finite) && bounds[0] <= bounds[1]
}

fn valid_positive_bounds(bounds: [f64; 2]) -> bool {
    valid_finite_bounds(bounds) && bounds[0] > 0.0
}

fn valid_nonnegative_bounds(bounds: [f64; 2]) -> bool {
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
fn capped_load_ratio(load_n: f64, reference_load_n: f64, maximum_load_ratio: f64) -> f64 {
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
fn resolve_driven_wheel_normal_load_n(
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

fn contact_tangent_axes(
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

fn relax_slip(current: f64, target: f64, length_m: f64, speed_m_s: f64, dt_s: f64) -> f64 {
    if length_m == 0.0 {
        target
    } else {
        let fraction = 1.0 - (-speed_m_s * dt_s / length_m).exp();
        current + fraction * (target - current)
    }
}

fn zero_tire_evaluation() -> CombinedSlipTireEvaluation {
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
enum TireFitAxis {
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

fn load_sensitivity_observations(
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

fn fit_load_sensitivity(
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

fn load_sensitivity_squared_error(
    tire: CombinedSlipTireSpec,
    observations: &[TireLoadSensitivityObservation],
) -> f64 {
    observations
        .iter()
        .map(|sample| load_sensitivity_sample_squared_error(tire, *sample))
        .sum()
}

fn load_sensitivity_sample_squared_error(
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

fn tire_relaxation_transitions(
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

fn tire_relaxation_observed_slip(
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

fn fit_tire_relaxation_length(
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

fn tire_relaxation_squared_error(
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

fn tire_relaxation_prediction(
    relaxation_length_m: f64,
    transition: TireRelaxationTransition,
) -> f64 {
    let (current, target, speed_m_s, dt_s, _, _) = transition;
    target + (current - target) * (-speed_m_s * dt_s / relaxation_length_m).exp()
}

fn validate_tire_identification_runs(
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

fn fit_tire_axis(
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

fn axis_squared_error(
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

fn steady_tire_forces(
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

/// Evaluates one DC motor from terminal voltage and completed rotor velocity.
///
/// With no inductance, current is the algebraic equivalent-circuit solution
/// `I = (V - k_e omega) / R`. With inductance, current advances by explicit Euler from
/// `dI/dt = (V - k_e omega - R I) / L` and is then limited. Shaft loss combines viscous
/// friction and a regularized Coulomb term: at standstill Coulomb friction cancels available
/// electromagnetic torque up to its declared magnitude rather than inventing motion.
pub fn evaluate_dc_motor(
    spec: DcMotorSpec,
    state: DcMotorState,
    command_voltage_v: f64,
    rotor_velocity_rad_s: f64,
    dt_s: f64,
) -> Result<DcMotorEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.current_a.is_finite()
        || !command_voltage_v.is_finite()
        || !rotor_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let voltage_saturated = command_voltage_v.abs() > spec.supply_voltage_v;
    let limited_command_voltage_v =
        command_voltage_v.clamp(-spec.supply_voltage_v, spec.supply_voltage_v);
    let terminal_voltage_v = match spec.failure_mode {
        DcMotorFailureMode::Nominal => limited_command_voltage_v,
        DcMotorFailureMode::OpenCircuit | DcMotorFailureMode::ShortCircuit => 0.0,
    };
    let back_emf_v = spec.back_emf_constant_v_s_rad * rotor_velocity_rad_s;
    let unconstrained_current_a = match (spec.failure_mode, spec.inductance_h) {
        (DcMotorFailureMode::OpenCircuit, _) => 0.0,
        (_, Some(inductance_h)) => {
            state.current_a
                + (terminal_voltage_v - back_emf_v - spec.resistance_ohm * state.current_a)
                    / inductance_h
                    * dt_s
        }
        (_, None) => (terminal_voltage_v - back_emf_v) / spec.resistance_ohm,
    };
    let current_saturated = unconstrained_current_a.abs() > spec.current_limit_a;
    let current_a = unconstrained_current_a.clamp(-spec.current_limit_a, spec.current_limit_a);
    let electromagnetic_torque_nm = spec.torque_constant_nm_a * current_a;
    let viscous_loss_nm = spec.viscous_friction_nm_s_rad * rotor_velocity_rad_s;
    let coulomb_loss_nm = if rotor_velocity_rad_s.abs() > 1.0e-12 {
        spec.coulomb_friction_nm * rotor_velocity_rad_s.signum()
    } else {
        electromagnetic_torque_nm.clamp(-spec.coulomb_friction_nm, spec.coulomb_friction_nm)
    };
    let shaft_loss_torque_nm = viscous_loss_nm + coulomb_loss_nm;

    Ok(DcMotorEvaluation {
        state: DcMotorState { current_a },
        terminal_voltage_v,
        back_emf_v,
        electromagnetic_torque_nm,
        shaft_loss_torque_nm,
        shaft_torque_nm: electromagnetic_torque_nm - shaft_loss_torque_nm,
        voltage_saturated,
        current_saturated,
    })
}

/// Maps motor torque and rotor inertia to a wheel coordinate without backend types.
///
/// This is the rigid static map. Declared backlash and compliance require a later stateful
/// driveline evaluator and are intentionally not approximated by hidden backend joints.
pub fn evaluate_transmission(
    spec: TransmissionSpec,
    motor_rotor_inertia_kg_m2: f64,
    motor_torque_nm: f64,
    wheel_velocity_rad_s: f64,
) -> Result<TransmissionEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !motor_rotor_inertia_kg_m2.is_finite()
        || motor_rotor_inertia_kg_m2 < 0.0
        || !motor_torque_nm.is_finite()
        || !wheel_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let ratio = spec.ratio_motor_rad_per_wheel_rad;
    let motor_velocity_rad_s = wheel_velocity_rad_s * ratio;
    let applied_efficiency_ratio = if motor_torque_nm * motor_velocity_rad_s >= 0.0 {
        spec.drive_efficiency_ratio
    } else {
        spec.backdrive_efficiency_ratio
    };
    Ok(TransmissionEvaluation {
        motor_velocity_rad_s,
        wheel_torque_nm: motor_torque_nm * ratio * applied_efficiency_ratio,
        reflected_rotor_inertia_kg_m2: motor_rotor_inertia_kg_m2 * ratio * ratio,
        applied_efficiency_ratio,
    })
}

/// Returns wheel rolling-resistance torque opposing completed wheel motion.
///
/// The v1 law is `Crr * normal_load * radius` and returns zero at exact standstill so it
/// cannot create a direction. A later wheel/ground solver may use impending slip to model
/// static rolling resistance.
pub fn wheel_rolling_resistance_torque_nm(
    spec: WheelAssemblySpec,
    normal_load_n: f64,
    wheel_velocity_rad_s: f64,
) -> Result<f64, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !normal_load_n.is_finite() || normal_load_n < 0.0 || !wheel_velocity_rad_s.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(if wheel_velocity_rad_s == 0.0 {
        0.0
    } else {
        -wheel_velocity_rad_s.signum()
            * spec.rolling_resistance_coefficient
            * normal_load_n
            * spec.radius_m
    })
}

/// Bounds Coulomb rolling resistance so one explicit step can stop, but not reverse, a wheel.
fn bounded_rolling_resistance_torque_nm(
    spec: WheelAssemblySpec,
    normal_load_n: f64,
    wheel_velocity_before_rolling_rad_s: f64,
    total_wheel_inertia_kg_m2: f64,
    dt_s: f64,
) -> Result<f64, MobilityPlantEvaluationError> {
    if !total_wheel_inertia_kg_m2.is_finite()
        || total_wheel_inertia_kg_m2 <= 0.0
        || !dt_s.is_finite()
        || dt_s <= 0.0
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let unconstrained = wheel_rolling_resistance_torque_nm(
        spec,
        normal_load_n,
        wheel_velocity_before_rolling_rad_s,
    )?;
    let stopping_torque_nm =
        wheel_velocity_before_rolling_rad_s.abs() * total_wheel_inertia_kg_m2 / dt_s;
    Ok(unconstrained.signum() * unconstrained.abs().min(stopping_torque_nm))
}

/// Result of applying one actuator command.
#[derive(Clone, Debug, PartialEq)]
pub enum CommandApplyResult {
    /// Command applied successfully.
    Applied,
    /// Command rejected because the target entity was invalid.
    InvalidTarget,
    /// Command rejected because the joint validation failed.
    JointRejected(JointValidationError),
    /// Command ignored because it was stale.
    Stale,
}

/// Result of commanding a kinematic Ackermann drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckermannCommandResult {
    /// The finite command was clamped to the drive limits and applied.
    Applied,
    /// The target entity has no valid [`AckermannDrive`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}

/// Result of commanding a multirotor position target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultirotorCommandResult {
    /// The finite position and heading target was applied.
    Applied,
    /// The target entity has no valid [`MultirotorFlight`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}

/// Applies a world-space position and heading target to one multirotor.
///
/// Commands are accepted only when both the target and the existing flight
/// component are valid. Rejected commands leave the previous target unchanged.
pub fn command_multirotor(
    world: &mut World,
    aircraft: Entity,
    target_position_m: Vec3,
    target_yaw_rad: f64,
) -> MultirotorCommandResult {
    if !target_position_m.is_finite() || !target_yaw_rad.is_finite() {
        return MultirotorCommandResult::NonFiniteCommand;
    }
    let Some(mut flight) = world.get_mut::<MultirotorFlight>(aircraft) else {
        return MultirotorCommandResult::InvalidTarget;
    };
    if !flight.is_valid() {
        return MultirotorCommandResult::InvalidTarget;
    }
    flight.target_position_m = target_position_m;
    flight.target_yaw_rad = wrap_angle_rad(target_yaw_rad);
    MultirotorCommandResult::Applied
}

/// Advances every valid multirotor in stable entity order for one fixed step.
///
/// The deterministic cascade is position error to desired velocity, desired
/// velocity to bounded acceleration, then semi-implicit position integration.
/// A Y-up body attitude follows the required thrust direction without exceeding
/// [`MultirotorFlight::max_tilt_rad`]. Entities with invalid configurations or
/// without a [`Transform3`] are left unchanged.
pub fn multirotor_flight(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut aircraft: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| entity.contains::<MultirotorFlight>() && entity.contains::<Transform3>())
        .map(|entity| entity.id())
        .collect();
    aircraft.sort_by_key(|entity| entity.to_bits());

    for entity in aircraft {
        let Some(mut flight) = world.get::<MultirotorFlight>(entity).copied() else {
            continue;
        };
        let Some(mut transform) = world.get::<Transform3>(entity).copied() else {
            continue;
        };
        if !flight.is_valid()
            || !transform.translation.is_finite()
            || !transform.rotation.is_finite()
        {
            continue;
        }

        let position_error_m = flight.target_position_m - transform.translation;
        let mut desired_velocity_m_s = position_error_m * flight.position_gain_s_inv;
        desired_velocity_m_s.y = desired_velocity_m_s
            .y
            .clamp(-flight.max_climb_speed_m_s, flight.max_climb_speed_m_s);
        let horizontal_speed_m_s = desired_velocity_m_s.x.hypot(desired_velocity_m_s.z);
        if horizontal_speed_m_s > flight.max_horizontal_speed_m_s {
            let scale = flight.max_horizontal_speed_m_s / horizontal_speed_m_s;
            desired_velocity_m_s.x *= scale;
            desired_velocity_m_s.z *= scale;
        }

        let mut acceleration_m_s2 =
            (desired_velocity_m_s - flight.velocity_m_s) * flight.velocity_gain_s_inv;
        acceleration_m_s2 = clamp_length(acceleration_m_s2, flight.max_acceleration_m_s2);
        let horizontal_tilt_limit_m_s2 = 9.81 * flight.max_tilt_rad.tan();
        let horizontal_acceleration_m_s2 = acceleration_m_s2.x.hypot(acceleration_m_s2.z);
        if horizontal_acceleration_m_s2 > horizontal_tilt_limit_m_s2 {
            let scale = horizontal_tilt_limit_m_s2 / horizontal_acceleration_m_s2;
            acceleration_m_s2.x *= scale;
            acceleration_m_s2.z *= scale;
        }

        flight.velocity_m_s += acceleration_m_s2 * dt_s;
        flight.velocity_m_s.y = flight
            .velocity_m_s
            .y
            .clamp(-flight.max_climb_speed_m_s, flight.max_climb_speed_m_s);
        let horizontal_velocity_m_s = flight.velocity_m_s.x.hypot(flight.velocity_m_s.z);
        if horizontal_velocity_m_s > flight.max_horizontal_speed_m_s {
            let scale = flight.max_horizontal_speed_m_s / horizontal_velocity_m_s;
            flight.velocity_m_s.x *= scale;
            flight.velocity_m_s.z *= scale;
        }
        transform.translation += flight.velocity_m_s * dt_s;

        let yaw_error_rad = wrap_angle_rad(flight.target_yaw_rad - flight.yaw_rad);
        let yaw_rate_rad_s =
            (yaw_error_rad * 3.0).clamp(-flight.max_yaw_rate_rad_s, flight.max_yaw_rate_rad_s);
        flight.yaw_rad = wrap_angle_rad(flight.yaw_rad + yaw_rate_rad_s * dt_s);

        let horizontal_acceleration = Vec3::new(acceleration_m_s2.x, 0.0, acceleration_m_s2.z);
        let desired_up = (Vec3::Y + horizontal_acceleration / 9.81).normalize_or_zero();
        let tilt = Quat::from_rotation_arc(Vec3::Y, desired_up);
        let yaw = Quat::from_rotation_y(flight.yaw_rad);
        let desired_rotation = (tilt * yaw).normalize();
        let attitude_blend = if flight.attitude_response_s == 0.0 {
            1.0
        } else {
            1.0 - (-dt_s / flight.attitude_response_s).exp()
        };
        transform.rotation = transform
            .rotation
            .slerp(desired_rotation, attitude_blend)
            .normalize();

        flight.commanded_acceleration_m_s2 = acceleration_m_s2;
        if let Some(mut body) = world.get_mut::<RigidBody>(entity) {
            body.linear_velocity_m_s = flight.velocity_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rate_rad_s, 0.0);
        }
        world.entity_mut(entity).insert((flight, transform));
    }
}

/// Applies a bounded speed and steering target to one kinematic Ackermann vehicle.
pub fn command_ackermann_drive(
    world: &mut World,
    vehicle: Entity,
    speed_m_s: f64,
    steering_rad: f64,
) -> AckermannCommandResult {
    if !speed_m_s.is_finite() || !steering_rad.is_finite() {
        return AckermannCommandResult::NonFiniteCommand;
    }
    let Some(mut drive) = world.get_mut::<AckermannDrive>(vehicle) else {
        return AckermannCommandResult::InvalidTarget;
    };
    if !drive.is_valid() {
        return AckermannCommandResult::InvalidTarget;
    }
    drive.target_speed_m_s = speed_m_s.clamp(-drive.max_speed_m_s, drive.max_speed_m_s);
    drive.target_steering_rad = steering_rad.clamp(-drive.max_steering_rad, drive.max_steering_rad);
    AckermannCommandResult::Applied
}

/// Integrates every valid Ackermann vehicle in stable entity order for one fixed step.
///
/// Invalid drive configurations and entities without a [`Transform3`] are left unchanged.
pub fn ackermann_kinematics(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut vehicles: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| {
            entity.contains::<AckermannDrive>()
                && entity.contains::<Transform3>()
                // Vehicles carrying VehicleDynamics are integrated by the dynamic
                // model instead; running both would double-integrate the chassis.
                && !entity.contains::<VehicleDynamics>()
        })
        .map(|entity| entity.id())
        .collect();
    vehicles.sort_by_key(|entity| entity.to_bits());

    for vehicle in vehicles {
        let Some(mut drive) = world.get::<AckermannDrive>(vehicle).cloned() else {
            continue;
        };
        if !drive.is_valid() {
            continue;
        }
        let accelerating = drive.target_speed_m_s.signum() == drive.speed_m_s.signum()
            && drive.target_speed_m_s.abs() > drive.speed_m_s.abs();
        let speed_rate_m_s2 = if accelerating {
            drive.max_acceleration_m_s2
        } else {
            drive.max_deceleration_m_s2
        };
        drive.speed_m_s = move_towards(
            drive.speed_m_s,
            drive.target_speed_m_s,
            speed_rate_m_s2 * dt_s,
        );
        drive.steering_rad = move_towards(
            drive.steering_rad,
            drive.target_steering_rad,
            drive.max_steering_rate_rad_s * dt_s,
        );
        let yaw_rad_s = drive.speed_m_s / drive.wheelbase_m * drive.steering_rad.tan();
        let yaw_delta_rad = yaw_rad_s * dt_s;
        let mut forward = Vec3::X;
        if let Some(mut transform) = world.get_mut::<Transform3>(vehicle) {
            let midpoint_rotation =
                (Quat::from_rotation_y(yaw_delta_rad * 0.5) * transform.rotation).normalize();
            forward = midpoint_rotation * Vec3::X;
            transform.translation += forward * drive.speed_m_s * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_delta_rad) * transform.rotation).normalize();
        }
        if let Some(mut body) = world.get_mut::<RigidBody>(vehicle) {
            body.linear_velocity_m_s = forward * drive.speed_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
        world.entity_mut(vehicle).insert(drive);
    }
}

/// Computes a pure-pursuit steering target toward a world-space lookahead point.
///
/// The returned angle follows the Ackermann convention used by
/// [`ackermann_kinematics`] and is not clamped to a particular vehicle's limits.
pub fn pure_pursuit_steering(
    transform: &Transform3,
    target_m: Vec3,
    wheelbase_m: f64,
    lookahead_m: f64,
) -> f64 {
    if !wheelbase_m.is_finite()
        || !lookahead_m.is_finite()
        || wheelbase_m <= 0.0
        || lookahead_m <= 0.0
    {
        return 0.0;
    }
    let local_target = transform.rotation.conjugate() * (target_m - transform.translation);
    (-2.0 * wheelbase_m * local_target.z).atan2(lookahead_m * lookahead_m)
}

/// Evaluates one axle's cornering stiffness, applying [`CorneringStiffnessLoadSensitivity`]
/// when present.
///
/// Absent `sensitivity` returns `reference_stiffness_n_rad` unchanged, which keeps
/// [`vehicle_dynamics`]'s constant-stiffness path bit-for-bit identical to before this
/// adjustment existed. A non-positive `static_axle_load_n` also falls back to the
/// unchanged value; that should not occur once [`VehicleDynamics::is_valid`] holds, since
/// a valid spec has strictly positive mass and axle distances.
///
/// When present, stiffness scales affinely with the axle's instantaneous load ratio
/// relative to its own static load, reusing [`CombinedSlipTireSpec`]'s load-ratio clamp
/// through [`capped_load_ratio`] and its `load_sensitivity_per_load_ratio` functional
/// form, with the slope sign flipped: cornering stiffness increases with load where tire
/// friction decreases with it. Because the affine map has a nonzero intercept at
/// `load_ratio == 1`, it is sub-linear in the load ratio (doubling the ratio does not
/// double the output), and it evaluates to exactly `reference_stiffness_n_rad` at the
/// axle's static load, preserving that parameter's declared meaning. Validation bounds
/// `load_sensitivity_per_load_ratio` to `[0.0, 1.0)` and `load_ratio` is never negative,
/// so the result stays strictly positive.
fn effective_cornering_stiffness(
    reference_stiffness_n_rad: f64,
    axle_load_n: f64,
    static_axle_load_n: f64,
    sensitivity: Option<CorneringStiffnessLoadSensitivity>,
) -> f64 {
    let Some(sensitivity) = sensitivity else {
        return reference_stiffness_n_rad;
    };
    if static_axle_load_n <= 0.0 {
        return reference_stiffness_n_rad;
    }
    let load_ratio = capped_load_ratio(
        axle_load_n,
        static_axle_load_n,
        sensitivity.maximum_load_ratio,
    );
    let stiffness_ratio =
        (1.0 + sensitivity.load_sensitivity_per_load_ratio * (load_ratio - 1.0)).max(0.0);
    reference_stiffness_n_rad * stiffness_ratio
}

/// Lateral force from one axle, optionally split into two equal-slip tires to
/// represent left/right load transfer.
///
/// With `lateral_transfer_n == 0` the two half-load tires reproduce the
/// single-tire axle exactly, including the friction clamp and the saturation
/// flag: halving and doubling are exact in binary floating point, so the
/// per-side stiffness sums to the axle stiffness and the per-side clamps sum to
/// the axle clamp whenever both sides are in the same regime.
fn axle_lateral_force_n(
    axle_stiffness_n_rad: f64,
    static_axle_load_n: f64,
    axle_load_n: f64,
    slip_rad: f64,
    friction_coefficient: f64,
    sensitivity: Option<CorneringStiffnessLoadSensitivity>,
    lateral_transfer_n: f64,
) -> (f64, bool) {
    let half_load_n = 0.5 * axle_load_n;
    let half_static_n = 0.5 * static_axle_load_n;
    let reference_stiffness_n_rad = 0.5 * axle_stiffness_n_rad;
    let transfer_n = lateral_transfer_n.max(0.0);
    let outer_load_n = (half_load_n + transfer_n).max(0.0);
    let inner_load_n = (half_load_n - transfer_n).max(0.0);

    let mut saturated = false;
    let mut side_forces_n = [0.0_f64; 2];
    for (index, side_load_n) in [outer_load_n, inner_load_n].into_iter().enumerate() {
        let stiffness_n_rad = effective_cornering_stiffness(
            reference_stiffness_n_rad,
            side_load_n,
            half_static_n,
            sensitivity,
        );
        let limit_n = friction_coefficient * side_load_n;
        let demanded_n = -stiffness_n_rad * slip_rad;
        if demanded_n.abs() > limit_n {
            saturated = true;
        }
        side_forces_n[index] = demanded_n.clamp(-limit_n, limit_n);
    }
    // Summing the two sides directly, rather than from a `+0.0` accumulator, keeps
    // the sign of a zero-slip force identical to the single-tire formula.
    (side_forces_n[0] + side_forces_n[1], saturated)
}

/// Advances vehicles that carry both [`AckermannDrive`] and [`VehicleDynamics`] with a
/// planar dynamic bicycle model.
///
/// [`ackermann_kinematics`] must not also run over these vehicles; this system is the
/// dynamic replacement, not a correction pass. Command shaping (speed and steering rate
/// limits) is shared with the kinematic path so the two models receive identical inputs
/// and differ only in how the chassis answers them.
///
/// Per step, for forward speed `vx`, lateral speed `vy`, yaw rate `r`, steering `delta`,
/// axle distances `a`/`b`, and per-axle cornering stiffness `C`:
///
/// ```text
/// alpha_f = atan((vy + a r) / vx) - delta      front slip angle
/// alpha_r = atan((vy - b r) / vx)              rear slip angle
/// Fy      = clamp(-C alpha, +/- mu Fz)         linear tire, friction saturated
/// m (vy' + vx r) = Fyf cos(delta) + Fyr        lateral balance
/// Iz r'          = a Fyf cos(delta) - b Fyr    yaw balance
/// ```
///
/// `Fz` per axle includes longitudinal load transfer `m ax h / L`, so braking loads the
/// front tires and throttle loads the rear — which is why the same corner behaves
/// differently on and off the power. Below [`VehicleDynamics::blend_low_speed_m_s`] the
/// lateral states relax toward the kinematic solution to avoid the `1/vx` singularity.
/// `C` itself is constant unless [`VehicleDynamics::cornering_stiffness_load_sensitivity`]
/// is present, in which case `effective_cornering_stiffness` scales it with the same
/// per-axle `Fz`.
pub fn vehicle_dynamics(world: &mut World, dt: SimDuration) {
    let dt_s = dt.as_seconds().value();
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return;
    }
    let mut vehicles: Vec<Entity> = world
        .iter_entities()
        .filter(|entity| {
            entity.contains::<AckermannDrive>()
                && entity.contains::<VehicleDynamics>()
                && entity.contains::<Transform3>()
        })
        .map(|entity| entity.id())
        .collect();
    vehicles.sort_by_key(|entity| entity.to_bits());

    for vehicle in vehicles {
        let Some(mut drive) = world.get::<AckermannDrive>(vehicle).cloned() else {
            continue;
        };
        let Some(mut dynamics) = world.get::<VehicleDynamics>(vehicle).copied() else {
            continue;
        };
        if !drive.is_valid() || !dynamics.is_valid() {
            continue;
        }

        // Shared command shaping, identical to the kinematic path.
        let accelerating = drive.target_speed_m_s.signum() == drive.speed_m_s.signum()
            && drive.target_speed_m_s.abs() > drive.speed_m_s.abs();
        let speed_rate_m_s2 = if accelerating {
            drive.max_acceleration_m_s2
        } else {
            drive.max_deceleration_m_s2
        };
        let previous_speed_m_s = drive.speed_m_s;
        drive.speed_m_s = move_towards(
            drive.speed_m_s,
            drive.target_speed_m_s,
            speed_rate_m_s2 * dt_s,
        );
        // Steering passes through the first-order actuator lag before the rate limit.
        // With a zero time constant the lag target is the command itself and this
        // reduces exactly to the kinematic path's shaping.
        let lag_target = if dynamics.steering_lag_s > 0.0 {
            let alpha = 1.0 - (-dt_s / dynamics.steering_lag_s).exp();
            drive.steering_rad + (drive.target_steering_rad - drive.steering_rad) * alpha
        } else {
            drive.target_steering_rad
        };
        drive.steering_rad = move_towards(
            drive.steering_rad,
            lag_target,
            drive.max_steering_rate_rad_s * dt_s,
        );

        let vx = drive.speed_m_s;
        let ax = (drive.speed_m_s - previous_speed_m_s) / dt_s;
        let delta = drive.steering_rad;
        let wheelbase = dynamics.wheelbase_m();

        // Axle loads with longitudinal transfer; clamped so neither axle lifts.
        let transfer_n = dynamics.mass_kg * ax * dynamics.center_of_mass_height_m / wheelbase;
        let front_load_n = (dynamics.static_front_load_n() - transfer_n).max(0.0);
        let rear_load_n = (dynamics.static_rear_load_n() + transfer_n).max(0.0);

        let kinematic_yaw_rate = vx / wheelbase * delta.tan();
        let speed_abs = vx.abs();

        if speed_abs <= dynamics.blend_low_speed_m_s.max(f64::EPSILON) {
            // Kinematic regime: slip angles are undefined, so the lateral states take
            // the no-slip solution directly.
            dynamics.yaw_rate_rad_s = kinematic_yaw_rate;
            dynamics.lateral_velocity_m_s = kinematic_yaw_rate * dynamics.rear_axle_m;
            dynamics.front_slip_rad = 0.0;
            dynamics.rear_slip_rad = 0.0;
            dynamics.front_saturated = false;
            dynamics.rear_saturated = false;
        } else {
            let vy = dynamics.lateral_velocity_m_s;
            let r = dynamics.yaw_rate_rad_s;

            let alpha_f = ((vy + dynamics.front_axle_m * r) / vx).atan() - delta;
            let alpha_r = ((vy - dynamics.rear_axle_m * r) / vx).atan();

            // Left/right load transfer is opt-in. Using the steady centripetal
            // acceleration from the current state rather than this step's forces keeps
            // the load/force relation explicit and loop-free, mirroring the
            // longitudinal path that uses the previous chassis acceleration.
            let (front_transfer_n, rear_transfer_n) = match dynamics.lateral_load_transfer {
                Some(spec) => {
                    let centripetal_acceleration_m_s2 = vx * r;
                    let total_transfer_n = dynamics.mass_kg
                        * centripetal_acceleration_m_s2.abs()
                        * dynamics.center_of_mass_height_m
                        / spec.track_width_m;
                    (
                        spec.front_roll_stiffness_fraction * total_transfer_n,
                        (1.0 - spec.front_roll_stiffness_fraction) * total_transfer_n,
                    )
                }
                None => (0.0, 0.0),
            };

            let (front_force_n, front_saturated) = axle_lateral_force_n(
                dynamics.front_cornering_stiffness_n_rad,
                dynamics.static_front_load_n(),
                front_load_n,
                alpha_f,
                dynamics.friction_coefficient,
                dynamics.cornering_stiffness_load_sensitivity,
                front_transfer_n,
            );
            let (rear_force_n, rear_saturated) = axle_lateral_force_n(
                dynamics.rear_cornering_stiffness_n_rad,
                dynamics.static_rear_load_n(),
                rear_load_n,
                alpha_r,
                dynamics.friction_coefficient,
                dynamics.cornering_stiffness_load_sensitivity,
                rear_transfer_n,
            );

            dynamics.front_slip_rad = alpha_f;
            dynamics.rear_slip_rad = alpha_r;
            dynamics.front_saturated = front_saturated;
            dynamics.rear_saturated = rear_saturated;

            let lateral_acceleration =
                (front_force_n * delta.cos() + rear_force_n) / dynamics.mass_kg - vx * r;
            let yaw_acceleration = (dynamics.front_axle_m * front_force_n * delta.cos()
                - dynamics.rear_axle_m * rear_force_n)
                / dynamics.yaw_inertia_kg_m2;

            dynamics.lateral_velocity_m_s += lateral_acceleration * dt_s;
            dynamics.yaw_rate_rad_s += yaw_acceleration * dt_s;
        }

        let yaw_delta_rad = dynamics.yaw_rate_rad_s * dt_s;
        let mut velocity_world = Vec3::ZERO;
        if let Some(mut transform) = world.get_mut::<Transform3>(vehicle) {
            let midpoint_rotation =
                (Quat::from_rotation_y(yaw_delta_rad * 0.5) * transform.rotation).normalize();
            // The body carries both forward and lateral velocity; slip is precisely
            // the difference between where the nose points and where the car goes.
            velocity_world = midpoint_rotation * Vec3::new(vx, 0.0, -dynamics.lateral_velocity_m_s);
            transform.translation += velocity_world * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_delta_rad) * transform.rotation).normalize();
        }
        if let Some(mut body) = world.get_mut::<RigidBody>(vehicle) {
            body.linear_velocity_m_s = velocity_world;
            body.angular_velocity_rad_s = Vec3::new(0.0, dynamics.yaw_rate_rad_s, 0.0);
        }
        world.entity_mut(vehicle).insert((drive, dynamics));
    }
}

fn move_towards(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

fn clamp_length(value: Vec3, max_length: f64) -> Vec3 {
    let length = value.length();
    if length > max_length && length > 0.0 {
        value * (max_length / length)
    } else {
        value
    }
}

fn wrap_angle_rad(mut angle_rad: f64) -> f64 {
    while angle_rad > std::f64::consts::PI {
        angle_rad -= std::f64::consts::TAU;
    }
    while angle_rad < -std::f64::consts::PI {
        angle_rad += std::f64::consts::TAU;
    }
    angle_rad
}

/// Applies queued actuator commands to actuators and joints.
pub fn apply_actuator_commands(world: &mut World, buffer: &mut ActuatorCommandBuffer) {
    let entries: Vec<_> = buffer.drain().collect();

    for entry in entries {
        let _ = apply_one_command(world, &entry.command);
    }
}

fn apply_one_command(world: &mut World, command: &ActuatorCommand) -> CommandApplyResult {
    match command {
        ActuatorCommand::JointPosition {
            joint,
            position_rad,
        } => apply_joint_position(world, *joint, *position_rad),
        ActuatorCommand::JointVelocity {
            joint,
            velocity_rad_s,
        } => apply_joint_velocity(world, *joint, *velocity_rad_s),
        ActuatorCommand::JointEffort { joint, effort_nm } => {
            apply_joint_effort(world, *joint, *effort_nm)
        }
        ActuatorCommand::WheelVelocity {
            wheel,
            velocity_rad_s,
        } => apply_wheel_velocity(world, *wheel, *velocity_rad_s),
        ActuatorCommand::GripperWidth { .. } | ActuatorCommand::BodyWrench { .. } => {
            CommandApplyResult::InvalidTarget
        }
        ActuatorCommand::Ackermann {
            vehicle,
            speed_m_s,
            steering_rad,
        } => match command_ackermann_drive(world, *vehicle, *speed_m_s, *steering_rad) {
            AckermannCommandResult::Applied => CommandApplyResult::Applied,
            AckermannCommandResult::InvalidTarget | AckermannCommandResult::NonFiniteCommand => {
                CommandApplyResult::InvalidTarget
            }
        },
    }
}

fn apply_joint_position(
    world: &mut World,
    joint_entity: Entity,
    position_rad: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let validated = match validate_joint_position(&joint, position_rad) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };
    joint_mut.position = validated;

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Position;
            actuator.target.position_rad = actuator.limits.clamp_position(validated);
        }
    }

    CommandApplyResult::Applied
}

fn apply_joint_velocity(
    world: &mut World,
    joint_entity: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    if joint.kind == JointKind::Fixed && velocity_rad_s.abs() > f64::EPSILON {
        return CommandApplyResult::JointRejected(JointValidationError::FixedJointNonZero);
    }

    let validated = match validate_joint_velocity(&joint, velocity_rad_s) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    if let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) {
        joint_mut.velocity = validated;
    }

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Velocity;
            actuator.target.velocity_rad_s = actuator.limits.clamp_velocity(validated);
        }
    }

    CommandApplyResult::Applied
}

fn apply_joint_effort(
    world: &mut World,
    joint_entity: Entity,
    effort_nm: f64,
) -> CommandApplyResult {
    let Some(_joint) = world.get::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Effort;
            actuator.target.effort_nm = effort_nm.clamp(
                -actuator.limits.max_effort_nm,
                actuator.limits.max_effort_nm,
            );
            return CommandApplyResult::Applied;
        }
    }

    CommandApplyResult::InvalidTarget
}

fn apply_wheel_velocity(
    world: &mut World,
    wheel_actuator: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(actuator) = world.get::<Actuator>(wheel_actuator).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let clamped = actuator.limits.clamp_velocity(velocity_rad_s);
    let Some(mut actuator_mut) = world.get_mut::<Actuator>(wheel_actuator) else {
        return CommandApplyResult::InvalidTarget;
    };
    actuator_mut.mode = ControlMode::Velocity;
    actuator_mut.target.velocity_rad_s = clamped;

    if let Some(joint_entity) = actuator_mut.joint {
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.velocity = clamped;
        }
    }

    CommandApplyResult::Applied
}

fn find_actuator_for_joint(world: &World, joint_entity: Entity) -> Option<Entity> {
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world
            .get::<Actuator>(entity)
            .is_some_and(|actuator| actuator.joint == Some(joint_entity))
        {
            return Some(entity);
        }
    }
    None
}

/// Integrates differential drive kinematics for one simulation step.
pub fn differential_drive_kinematics(
    world: &mut World,
    drives: &[DifferentialDrive],
    dt: SimDuration,
) {
    let dt_s = dt.as_seconds().value();

    for drive in drives {
        let Some(left) = world.get::<Actuator>(drive.left_actuator) else {
            continue;
        };
        let Some(right) = world.get::<Actuator>(drive.right_actuator) else {
            continue;
        };

        let v_left = left.target.velocity_rad_s * drive.wheel_radius_m;
        let v_right = right.target.velocity_rad_s * drive.wheel_radius_m;
        let linear_m_s = (v_left + v_right) * 0.5;
        let yaw_rad_s = (v_right - v_left) / drive.track_width_m;

        let (base_snapshot, forward) = {
            let Some(mut transform) = world.get_mut::<Transform3>(drive.base_link) else {
                continue;
            };

            let forward = transform.rotation * Vec3::X;
            transform.translation += forward * linear_m_s * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_rad_s * dt_s) * transform.rotation).normalize();
            (*transform, forward)
        };

        if world
            .get::<RigidBody>(drive.base_link)
            .is_some_and(|body| body.body_type == RigidBodyType::Kinematic)
        {
            integrate_kinematic_wheel_joints(world, drive, dt_s);
            sync_wheel_transforms(world, drive, &base_snapshot);
        }

        if let Some(mut body) = world.get_mut::<RigidBody>(drive.base_link) {
            let forward_flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
            body.linear_velocity_m_s = forward_flat * linear_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
    }
}

fn integrate_kinematic_wheel_joints(world: &mut World, drive: &DifferentialDrive, dt_s: f64) {
    for actuator_entity in [drive.left_actuator, drive.right_actuator] {
        let Some(joint_entity) = world
            .get::<Actuator>(actuator_entity)
            .and_then(|actuator| actuator.joint)
        else {
            continue;
        };
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.position += joint.velocity * dt_s;
        }
    }
}

fn sync_wheel_transforms(world: &mut World, drive: &DifferentialDrive, base: &Transform3) {
    let half_track = drive.track_width_m * 0.5;
    let wheel_y = world
        .get::<Collider>(drive.base_link)
        .and_then(|collider| match collider.shape {
            ColliderShape::Cuboid { half_extents_m } => {
                Some(-half_extents_m.y + drive.wheel_radius_m)
            }
            _ => None,
        })
        .unwrap_or(0.0);

    for (wheel, x_offset) in [
        (drive.left_actuator, -half_track),
        (drive.right_actuator, half_track),
    ] {
        let Some(actuator) = world.get::<Actuator>(wheel) else {
            continue;
        };
        let Some(wheel_entity) = actuator.joint else {
            continue;
        };
        let Some(mut wheel_transform) = world.get_mut::<Transform3>(wheel_entity) else {
            continue;
        };
        let offset = base.rotation * Vec3::new(x_offset, wheel_y, 0.0);
        wheel_transform.translation = base.translation + offset;
        wheel_transform.rotation = base.rotation;
    }
}

/// Copies every actuator target into unit-explicit [`JointActuation`].
///
/// The optional `drives` argument on [`sync_joint_motors_from_actuators`] is kept
/// for source compatibility with older diff-drive callers. Named URDF actuators
/// use this function directly and are resolved through their [`Joint`] child link.
/// Existing [`JointMotor`] components are updated as a compatibility path.
pub fn sync_all_joint_motors_from_actuators(world: &mut World) {
    let mut actuator_entities: Vec<_> = world
        .iter_entities()
        .map(|entity| entity.id())
        .filter(|entity| world.get::<Actuator>(*entity).is_some())
        .collect();
    actuator_entities.sort_unstable();

    for actuator_entity in actuator_entities {
        let Some((joint_entity, mode, target, limits)) =
            world.get::<Actuator>(actuator_entity).map(|actuator| {
                (
                    actuator.joint,
                    actuator.mode,
                    actuator.target,
                    actuator.limits,
                )
            })
        else {
            continue;
        };
        let Some(joint_entity) = joint_entity else {
            continue;
        };
        let Some((child_link, joint_kind)) = world
            .get::<Joint>(joint_entity)
            .map(|joint| (joint.child_link, joint.kind))
        else {
            continue;
        };
        let tuning = world
            .get::<JointMotor>(child_link)
            .copied()
            .unwrap_or_default();
        let max_output = if limits.max_effort_nm.is_finite() {
            limits.max_effort_nm.max(0.0)
        } else {
            0.0
        };
        let stiffness = if tuning.stiffness.is_finite() && tuning.stiffness > 0.0 {
            tuning.stiffness
        } else {
            40.0
        };
        let gain = if tuning.gain.is_finite() && tuning.gain >= 0.0 {
            tuning.gain
        } else {
            1.0
        };
        let actuation = match (joint_kind, mode) {
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Position) => {
                JointActuation::RevolutePosition {
                    target_position_rad: target.position_rad,
                    stiffness_nm_per_rad: stiffness,
                    damping_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Velocity) => {
                JointActuation::RevoluteVelocity {
                    target_velocity_rad_s: target.velocity_rad_s,
                    gain_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Effort) => {
                JointActuation::RevoluteEffort {
                    effort_nm: target.effort_nm,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Prismatic, ControlMode::Position) => JointActuation::PrismaticPosition {
                target_position_m: target.position_rad,
                stiffness_n_per_m: stiffness,
                damping_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Velocity) => JointActuation::PrismaticVelocity {
                target_velocity_m_s: target.velocity_rad_s,
                gain_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Effort) => JointActuation::PrismaticEffort {
                force_n: target.effort_nm,
                max_force_n: max_output,
            },
            (JointKind::Fixed, _) => JointActuation::Disabled,
        };
        world.entity_mut(child_link).insert(actuation);
        if let Some(mut motor) = world.get_mut::<JointMotor>(child_link) {
            motor.velocity_rad_s = match mode {
                ControlMode::Velocity => target.velocity_rad_s,
                ControlMode::Position | ControlMode::Effort => 0.0,
            };
            if mode == ControlMode::Position {
                motor.target_position = target.position_rad;
                motor.stiffness = stiffness;
            }
        }
    }
}

/// Copies actuator velocity targets into [`JointMotor`] components for physics stepping.
pub fn sync_joint_motors_from_actuators(world: &mut World, _drives: &[DifferentialDrive]) {
    sync_all_joint_motors_from_actuators(world);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuator::ActuatorLimits;
    use crate::components::{
        AckermannDrive, JointKind, JointLimits, LateralLoadTransferSpec, Link, MultirotorFlight,
        Robot, RobotId,
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
        let healthy =
            identify_suspension_strut_runs_report(spec, &training, &holdout[..1]).unwrap();
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
        let reconstructed_surface_center = geometry.solid_transform.translation
            + geometry.normal_world * (0.5 * patch.thickness_m);

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
        assert!(
            (frame.forward_world - Vec3::new(3.0_f64.sqrt() / 2.0, 0.0, -0.5)).length() < 1.0e-12
        );
        assert!(
            (frame.lateral_world - Vec3::new(0.5, 0.0, 3.0_f64.sqrt() / 2.0)).length() < 1.0e-12
        );
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
        let transform = Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_xyzw(0.0, 0.001, 0.0, 1.0),
        );
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
        world.entity_mut(vehicle).insert((
            drive,
            dynamics,
            Transform3::IDENTITY,
            RigidBody::default(),
        ));
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
        let loaded = effective_cornering_stiffness(
            reference,
            static_load * 1.4,
            static_load,
            Some(sensitivity),
        );
        let unloaded = effective_cornering_stiffness(
            reference,
            static_load * 0.6,
            static_load,
            Some(sensitivity),
        );
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
        let at_1_2 =
            effective_cornering_stiffness(reference, 1_200.0, static_load, Some(sensitivity));
        let at_2_4 =
            effective_cornering_stiffness(reference, 2_400.0, static_load, Some(sensitivity));
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

    fn combined_tire_holdout_samples(
        road_friction_scale: f64,
    ) -> Vec<TireForceIdentificationSample> {
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
                let forces =
                    steady_tire_forces(tire, longitudinal_slip, lateral_slip, 1_000.0, 1.0);
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
        let high_slip_speed =
            high.wheel_velocity_rad_s * high_spec.wheel.radius_m - high.velocity_m_s;
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
        let braking =
            evaluate_longitudinal_mobility_plant(spec, accelerated, -24.0, 0.001).unwrap();
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
            reference_state =
                reference_step_without_load_transfer(spec, reference_state, 24.0, 0.001);
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
}
