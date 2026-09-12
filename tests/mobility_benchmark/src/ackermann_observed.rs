//! Sensor-only speed and yaw control over the suspended Ackermann plant.

use anyhow::{bail, ensure, Context, Result};
use rne_ai::{
    AckermannEncoderStreams, AckermannImuOdometry, AckermannImuOdometryConfig,
    AckermannImuOdometryError, AckermannImuOdometryHealth, ActionSpec, ObservationSpec, ResetSpec,
    RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType, TensorSpec,
    TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, ImuFeedback, ImuFeedbackStatus, InMemoryDataBus, IncrementalEncoderFeedback,
    IncrementalEncoderStatus, MotorElectricalFeedback, MotorElectricalFeedbackStatus, PoseSample,
    StreamId,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, CollisionGroups, ExternalBodyWrench, JointActuation, JointState,
    MultibodyLink, PhysicsBackend, PhysicsBackendManifest, PhysicsCapability, PhysicsWorldDesc,
    RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, evaluate_steering_actuator,
    evaluate_suspension_strut, Actuator, ActuatorLimits, ActuatorTarget, ControlMode,
    DcMotorCompletedTelemetry, Joint, JointKind, JointLimits, LongitudinalDrivePathInput,
    LongitudinalDrivePathState, LongitudinalMobilityPlantSpec, SteeringActuatorSpec,
    SteeringActuatorState, SuspensionStrutSpec, WheelStationSpec,
};
use rne_sensor::{
    sample_imu_feedback_sensors, sample_incremental_encoder_sensors,
    sample_motor_electrical_feedback_sensors, ImuAxisErrors, ImuFeedbackFault, ImuFeedbackSensor,
    ImuFeedbackSensorState, ImuMount, ImuSpec, IncrementalEncoderFault,
    IncrementalEncoderOverflowBehavior, IncrementalEncoderSensor, IncrementalEncoderSensorState,
    IncrementalEncoderSpec, MotorElectricalFeedbackFault, MotorElectricalFeedbackSensor,
    MotorElectricalFeedbackSensorState, MotorElectricalFeedbackSpec,
};
use rne_world::{Transform3, WorldRandom};
use serde::{Deserialize, Serialize};

use crate::ackermann_suspension::{
    frictionless_cuboid, front_steering_targets, spawn_stations, steering_actuator_spec,
    suspension_spec, wheel_plant_spec, wheel_station_specs, CHASSIS_MASS_KG,
    CONTACT_LOAD_FILTER_TIME_CONSTANT_S, INITIAL_SUSPENSION_POSITION_M, SELF_COLLISION_GROUP,
    TRACK_WIDTH_M, WHEELBASE_M, WHEEL_RADIUS_M,
};
use crate::MobilityBenchmarkMetric;

/// Stable artifact discriminator for one sensor-only Ackermann run.
pub const ACKERMANN_OBSERVED_TRACE_KIND: &str = "rne_mobility_ackermann_observed_trace";
/// Stable artifact discriminator for a two-backend sensor-only comparison.
pub const ACKERMANN_OBSERVED_COMPARISON_KIND: &str = "rne_mobility_ackermann_observed_comparison";
/// Trace and comparison schema version.
pub const ACKERMANN_OBSERVED_SCHEMA_VERSION: u32 = 2;
/// Stable TaskSpec identity shared by both rigid-body backends.
pub const ACKERMANN_OBSERVED_TASK_ID: &str = "mobility_ackermann_sensor_closed_loop_v2";
/// One-millisecond rigid-body and tire integration step, in simulation ticks.
pub const ACKERMANN_OBSERVED_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SENSOR_PERIOD_TICKS: u64 = 10_000_000;
const SENSOR_LATENCY_TICKS: u64 = 2_000_000;
const SETTLE_STEPS: u64 = 1_500;
const TURN_START_STEP: u64 = 3_500;
const TOTAL_STEPS: u64 = 6_000;
const TARGET_SPEED_M_S: f64 = 1.0;
const TARGET_YAW_RATE_RAD_S: f64 = 0.12;
const MAXIMUM_VOLTAGE_V: f64 = 12.0;
const MAXIMUM_CENTER_STEERING_RAD: f64 = 0.35;
const WHEEL_ENCODER_COUNTS_PER_REVOLUTION: u32 = 2_048;
const STEERING_ENCODER_COUNTS_PER_REVOLUTION: u32 = 16_384;
const WORLD_SEED: u64 = 0;

const STREAMS: AckermannEncoderStreams = AckermannEncoderStreams {
    front_left_wheel: StreamId::new(3_001),
    rear_left_wheel: StreamId::new(3_002),
    front_right_wheel: StreamId::new(3_003),
    rear_right_wheel: StreamId::new(3_004),
    front_left_steering: StreamId::new(3_005),
    front_right_steering: StreamId::new(3_006),
    imu: StreamId::new(3_007),
};
const MOTOR_STREAMS: [StreamId; 4] = [
    StreamId::new(3_011),
    StreamId::new(3_012),
    StreamId::new(3_013),
    StreamId::new(3_014),
];

/// One deterministic sensor fault applied without modifying plant truth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum AckermannObservedFault {
    /// No injected fault.
    #[default]
    None,
    /// Drop one front-left wheel encoder capture.
    FrontLeftWheelDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold the front-left wheel encoder from this sequence onward.
    FrontLeftWheelStuck {
        /// One-based first attempted sequence that reuses the prior value.
        sequence: u64,
    },
    /// Saturate the front-left wheel encoder at a reduced signed counter width.
    FrontLeftWheelSaturate {
        /// Reduced signed hardware-counter width.
        counter_bits: u8,
    },
    /// Drop one front-right steering encoder capture.
    FrontRightSteeringDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold the front-right steering encoder from this sequence onward.
    FrontRightSteeringStuck {
        /// One-based first attempted sequence that reuses the prior value.
        sequence: u64,
    },
    /// Drop one front-left motor-current capture.
    FrontLeftMotorDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Drop one mounted-IMU capture.
    ImuDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold the mounted IMU from this sequence onward.
    ImuStuck {
        /// One-based first attempted sequence that reuses the prior value.
        sequence: u64,
    },
}

/// Frozen frontend, estimator, and controller configuration retained in the trace.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AckermannObservedContract {
    /// Wheel encoder decoded counts per revolution.
    pub wheel_encoder_counts_per_revolution: u32,
    /// Steering angle encoder decoded counts per revolution.
    pub steering_encoder_counts_per_revolution: u32,
    /// Signed wheel and steering counter width.
    pub encoder_counter_bits: u8,
    /// Exact frontend capture period in simulation ticks.
    pub sensor_period_ticks: u64,
    /// Capture-to-availability delay in simulation ticks.
    pub sensor_latency_ticks: u64,
    /// Calibrated mounted-IMU yaw bias, in rad/s.
    pub calibrated_gyro_z_bias_rad_s: f64,
    /// Longitudinal speed proportional gain, in V per `(m/s)`.
    pub speed_kp_v_s_m: f64,
    /// Longitudinal speed integral gain, in V per m.
    pub speed_ki_v_m: f64,
    /// Yaw-rate feedback gain from rad/s error to steering radians, in seconds.
    pub yaw_rate_kp_s: f64,
    /// Selected deterministic recoverable fault.
    pub fault: AckermannObservedFault,
}

impl AckermannObservedContract {
    fn nominal() -> Self {
        Self {
            wheel_encoder_counts_per_revolution: WHEEL_ENCODER_COUNTS_PER_REVOLUTION,
            steering_encoder_counts_per_revolution: STEERING_ENCODER_COUNTS_PER_REVOLUTION,
            encoder_counter_bits: 32,
            sensor_period_ticks: SENSOR_PERIOD_TICKS,
            sensor_latency_ticks: SENSOR_LATENCY_TICKS,
            calibrated_gyro_z_bias_rad_s: 0.001,
            speed_kp_v_s_m: 8.0,
            speed_ki_v_m: 3.0,
            yaw_rate_kp_s: 0.6,
            fault: AckermannObservedFault::None,
        }
    }

    fn validate(&self) -> Result<()> {
        let mut expected = Self::nominal();
        expected.fault = self.fault;
        ensure!(*self == expected, "Ackermann observed contract drift");
        match self.fault {
            AckermannObservedFault::None => {}
            AckermannObservedFault::FrontLeftWheelDrop { sequence }
            | AckermannObservedFault::FrontLeftWheelStuck { sequence }
            | AckermannObservedFault::FrontRightSteeringDrop { sequence }
            | AckermannObservedFault::FrontRightSteeringStuck { sequence }
            | AckermannObservedFault::FrontLeftMotorDrop { sequence }
            | AckermannObservedFault::ImuDrop { sequence }
            | AckermannObservedFault::ImuStuck { sequence } => {
                ensure!(sequence > 1, "fault sequence must follow initialization");
            }
            AckermannObservedFault::FrontLeftWheelSaturate { counter_bits } => {
                ensure!((2..=16).contains(&counter_bits), "invalid saturation width");
            }
        }
        Ok(())
    }
}

/// One controller decision with actor-visible measurements and separate scoring truth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AckermannObservedSample {
    /// Completed physics step.
    pub step: u64,
    /// Estimator/controller decision time in simulation ticks.
    pub decision_ticks: u64,
    /// Newest synchronized sensor capture time in ticks.
    pub capture_ticks: u64,
    /// Four physical wheel encoder sequences in FL, RL, FR, RR order.
    pub wheel_encoder_sequences: [u64; 4],
    /// Four stable wheel encoder frontend status codes.
    pub wheel_encoder_status_codes: [u8; 4],
    /// Two steering encoder sequences in FL, FR order.
    pub steering_encoder_sequences: [u64; 2],
    /// Two stable steering encoder frontend status codes.
    pub steering_encoder_status_codes: [u8; 2],
    /// Measured front steering angles in radians.
    pub measured_steering_rad: [f64; 2],
    /// Four measured motor currents in amperes.
    pub measured_motor_current_a: [f64; 4],
    /// Four motor telemetry sequences.
    pub motor_sequences: [u64; 4],
    /// Four stable motor frontend status codes.
    pub motor_status_codes: [u8; 4],
    /// Mounted IMU sequence.
    pub imu_sequence: u64,
    /// Stable mounted-IMU frontend status code.
    pub imu_status_code: u8,
    /// Stable estimator health code.
    pub estimator_health_code: u8,
    /// Estimated rear-axle-center forward speed in m/s.
    pub estimated_speed_m_s: f64,
    /// Estimated yaw rate in rad/s.
    pub estimated_yaw_rate_rad_s: f64,
    /// Estimated path curvature in 1/m.
    pub estimated_curvature_per_m: f64,
    /// Estimated planar pose `[x_m, lateral_m, yaw_rad]`.
    pub estimated_pose: [f64; 3],
    /// Estimated planar position covariance in row-major order, in square meters.
    pub estimated_position_covariance_m2: [f64; 4],
    /// Estimated yaw variance in square radians.
    pub estimated_yaw_variance_rad2: f64,
    /// Target speed exposed to the actor, in m/s.
    pub target_speed_m_s: f64,
    /// Target yaw rate exposed to the actor, in rad/s.
    pub target_yaw_rate_rad_s: f64,
    /// Actor-produced four motor terminal voltage commands, in volts.
    pub command_voltage_v: [f64; 4],
    /// Actor-produced center steering target, in radians.
    pub center_steering_target_rad: f64,
    /// Completed center steering-actuator target used during this physics step, in radians.
    pub steering_actuator_target_rad: f64,
    /// Privileged chassis forward displacement used only for scoring, in meters.
    pub privileged_forward_distance_m: f64,
    /// Privileged chassis speed used only for scoring, in m/s.
    pub privileged_forward_speed_m_s: f64,
    /// Privileged chassis yaw rate used only for scoring, in rad/s.
    pub privileged_yaw_rate_rad_s: f64,
}

impl AckermannObservedSample {
    fn validate(&self, contract: &AckermannObservedContract) -> Result<()> {
        ensure!(self.step <= TOTAL_STEPS, "sample step outside task");
        ensure!(
            self.decision_ticks == self.step * ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
            "decision timestamp drift"
        );
        ensure!(
            self.capture_ticks <= self.decision_ticks
                && self.decision_ticks - self.capture_ticks <= contract.sensor_latency_ticks,
            "capture age drift"
        );
        ensure!(
            self.measured_steering_rad
                .iter()
                .chain(self.measured_motor_current_a.iter())
                .chain(
                    [
                        self.estimated_speed_m_s,
                        self.estimated_yaw_rate_rad_s,
                        self.estimated_curvature_per_m,
                        self.target_speed_m_s,
                        self.target_yaw_rate_rad_s,
                        self.center_steering_target_rad,
                        self.steering_actuator_target_rad,
                        self.privileged_forward_distance_m,
                        self.privileged_forward_speed_m_s,
                        self.privileged_yaw_rate_rad_s,
                    ]
                    .iter()
                )
                .chain(self.estimated_pose.iter())
                .chain(self.estimated_position_covariance_m2.iter())
                .chain(std::iter::once(&self.estimated_yaw_variance_rad2))
                .chain(self.command_voltage_v.iter())
                .all(|value| value.is_finite()),
            "non-finite observed sample"
        );
        ensure!(
            self.command_voltage_v
                .iter()
                .all(|value| value.abs() <= MAXIMUM_VOLTAGE_V),
            "voltage outside TaskSpec"
        );
        ensure!(
            self.center_steering_target_rad.abs() <= MAXIMUM_CENTER_STEERING_RAD,
            "steering outside TaskSpec"
        );
        ensure!(
            self.steering_actuator_target_rad.abs()
                <= steering_actuator_spec().maximum_position_rad,
            "steering actuator target outside travel"
        );
        ensure!(
            self.wheel_encoder_status_codes
                .iter()
                .chain(self.steering_encoder_status_codes.iter())
                .all(|status| *status <= 3)
                && self.motor_status_codes.iter().all(|status| *status <= 3)
                && self.imu_status_code <= 2
                && self.estimator_health_code <= 5,
            "sensor status code outside TaskSpec"
        );
        Ok(())
    }
}

/// Self-verifying sensor-only Ackermann trace from one rigid-body backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AckermannObservedTrace {
    /// Artifact kind.
    pub kind: String,
    /// Schema version.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact portable actor contract.
    pub task_spec: TaskSpec,
    /// Exact frontend, estimator, and controller contract.
    pub contract: AckermannObservedContract,
    /// Shared motor, transmission, wheel, and tire profile.
    pub plant_spec: LongitudinalMobilityPlantSpec,
    /// Ordered physical wheel station geometry.
    pub wheel_station_specs: [WheelStationSpec; 4],
    /// Shared suspension profile.
    pub suspension_spec: SuspensionStrutSpec,
    /// Shared center-steering actuator response contract.
    pub steering_actuator_spec: SteeringActuatorSpec,
    /// Fixed physics step in simulation ticks.
    pub fixed_delta_ticks: u64,
    /// Deterministic world and sensor seed root.
    pub seed: u64,
    /// Completed physics steps.
    pub steps: u64,
    /// Ordered controller decisions.
    pub samples: Vec<AckermannObservedSample>,
    /// SI-unit acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Overall verdict.
    pub passed: bool,
    /// FNV-1a digest over every preceding field.
    pub content_digest: String,
}

impl AckermannObservedTrace {
    /// Recomputes the frozen contract, sample invariants, metrics, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == ACKERMANN_OBSERVED_TRACE_KIND,
            "trace kind drift"
        );
        ensure!(
            self.schema_version == ACKERMANN_OBSERVED_SCHEMA_VERSION,
            "trace schema drift"
        );
        self.backend.validate()?;
        ensure!(
            self.task_spec == ackermann_observed_task_spec(),
            "TaskSpec drift"
        );
        ensure!(
            self.task_spec
                .observation
                .tensors
                .iter()
                .all(|tensor| !tensor.name.contains("truth") && !tensor.name.contains("privileged")),
            "actor observation exposes privileged truth"
        );
        self.contract.validate()?;
        ensure!(self.plant_spec == wheel_plant_spec(), "plant drift");
        ensure!(
            self.wheel_station_specs == wheel_station_specs(),
            "station drift"
        );
        ensure!(
            self.suspension_spec == suspension_spec(),
            "suspension drift"
        );
        ensure!(
            self.steering_actuator_spec == steering_actuator_spec(),
            "steering actuator drift"
        );
        ensure!(
            self.fixed_delta_ticks == ACKERMANN_OBSERVED_FIXED_DELTA_TICKS
                && self.seed == WORLD_SEED
                && self.steps == TOTAL_STEPS,
            "runtime contract drift"
        );
        ensure!(!self.samples.is_empty(), "trace omitted actor samples");
        for pair in self.samples.windows(2) {
            ensure!(pair[0].step < pair[1].step, "sample order drift");
            ensure!(
                pair[0]
                    .wheel_encoder_sequences
                    .iter()
                    .zip(pair[1].wheel_encoder_sequences)
                    .all(|(previous, current)| previous < &current),
                "wheel sequence did not advance"
            );
            ensure!(
                pair[0]
                    .steering_encoder_sequences
                    .iter()
                    .zip(pair[1].steering_encoder_sequences)
                    .all(|(previous, current)| previous < &current),
                "steering sequence did not advance"
            );
        }
        for sample in &self.samples {
            sample.validate(&self.contract)?;
        }
        let dt_s = SimDuration::from_ticks(self.fixed_delta_ticks)
            .as_seconds()
            .value();
        let mut steering_state = SteeringActuatorState::default();
        let mut steering_command_rad = 0.0;
        let mut previous_step = 0;
        for sample in &self.samples {
            for _ in previous_step..sample.step {
                steering_state = evaluate_steering_actuator(
                    self.steering_actuator_spec,
                    steering_state,
                    steering_command_rad,
                    dt_s,
                )?
                .state;
            }
            ensure!(
                sample.steering_actuator_target_rad == steering_state.position_rad,
                "steering actuator replay drift"
            );
            steering_command_rad = sample.center_steering_target_rad;
            previous_step = sample.step;
        }
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == trace_digest(self)?,
            "trace digest drift"
        );
        Ok(())
    }
}

/// Stable fail-closed classification for an Ackermann motion input.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AckermannObservedFailureCode {
    /// A wheel encoder reported a held value.
    WheelEncoderStuck,
    /// A wheel encoder finite counter saturated.
    WheelEncoderSaturated,
    /// A steering encoder reported a held value.
    SteeringEncoderStuck,
    /// The mounted IMU reported a held value.
    ImuStuck,
}

/// Self-verifying snapshot emitted when an unsafe Ackermann input fails closed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AckermannObservedFailureCapsule {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// Backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact actor/action task contract.
    pub task_spec: TaskSpec,
    /// Exact sensor/controller/fault contract.
    pub contract: AckermannObservedContract,
    /// Stable fail-closed category.
    pub failure_code: AckermannObservedFailureCode,
    /// Completed physics step at which the controller rejected input.
    pub failed_step: u64,
    /// Controller decision timestamp in ticks.
    pub decision_ticks: u64,
    /// Latest wheel encoder sequences in FL, RL, FR, RR order.
    pub wheel_encoder_sequences: [u64; 4],
    /// Latest wheel encoder status codes.
    pub wheel_encoder_status_codes: [u8; 4],
    /// Latest steering encoder sequences in FL, FR order.
    pub steering_encoder_sequences: [u64; 2],
    /// Latest steering encoder status codes.
    pub steering_encoder_status_codes: [u8; 2],
    /// Latest motor-feedback sequences.
    pub motor_sequences: [u64; 4],
    /// Latest motor-feedback status codes.
    pub motor_status_codes: [u8; 4],
    /// Latest mounted-IMU sequence.
    pub imu_sequence: u64,
    /// Latest mounted-IMU status code.
    pub imu_status_code: u8,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl AckermannObservedFailureCapsule {
    /// Recomputes fault compatibility, timing, status evidence, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_mobility_ackermann_observed_failure_capsule",
            "capsule kind mismatch"
        );
        ensure!(
            self.schema_version == ACKERMANN_OBSERVED_SCHEMA_VERSION,
            "capsule schema mismatch"
        );
        self.backend.validate()?;
        self.task_spec.validate()?;
        ensure!(
            self.task_spec == ackermann_observed_task_spec(),
            "TaskSpec drift"
        );
        self.contract.validate()?;
        ensure!(
            self.failed_step > 0
                && self.failed_step <= TOTAL_STEPS
                && self.decision_ticks == self.failed_step * ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
            "failure timing mismatch"
        );
        ensure!(
            matches!(
                (self.contract.fault, self.failure_code),
                (
                    AckermannObservedFault::FrontLeftWheelStuck { .. },
                    AckermannObservedFailureCode::WheelEncoderStuck
                ) | (
                    AckermannObservedFault::FrontLeftWheelSaturate { .. },
                    AckermannObservedFailureCode::WheelEncoderSaturated
                ) | (
                    AckermannObservedFault::FrontRightSteeringStuck { .. },
                    AckermannObservedFailureCode::SteeringEncoderStuck
                ) | (
                    AckermannObservedFault::ImuStuck { .. },
                    AckermannObservedFailureCode::ImuStuck
                )
            ),
            "fault/failure-code mismatch"
        );
        ensure!(
            self.wheel_encoder_status_codes
                .iter()
                .chain(self.steering_encoder_status_codes.iter())
                .all(|status| *status <= 3)
                && self.motor_status_codes.iter().all(|status| *status <= 3)
                && self.imu_status_code <= 2,
            "invalid status code"
        );
        match self.failure_code {
            AckermannObservedFailureCode::WheelEncoderStuck => {
                ensure!(
                    self.wheel_encoder_status_codes[0] == 3,
                    "stuck status missing"
                );
            }
            AckermannObservedFailureCode::WheelEncoderSaturated => {
                ensure!(
                    self.wheel_encoder_status_codes[0] == 2,
                    "saturation status missing"
                );
            }
            AckermannObservedFailureCode::SteeringEncoderStuck => {
                ensure!(
                    self.steering_encoder_status_codes[1] == 3,
                    "steering stuck status missing"
                );
            }
            AckermannObservedFailureCode::ImuStuck => {
                ensure!(self.imu_status_code == 2, "IMU stuck status missing");
            }
        }
        ensure!(
            self.content_digest == failure_capsule_digest(self)?,
            "capsule digest mismatch"
        );
        Ok(())
    }
}

enum AckermannObservedRunOutcome {
    Trace(Box<AckermannObservedTrace>),
    Failure(Box<AckermannObservedFailureCapsule>),
}

/// Self-verifying unit-aware Rapier/MuJoCo comparison.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AckermannObservedComparison {
    /// Artifact kind.
    pub kind: String,
    /// Schema version.
    pub schema_version: u32,
    /// First complete backend trace.
    pub first: AckermannObservedTrace,
    /// Second complete backend trace.
    pub second: AckermannObservedTrace,
    /// SI-unit cross-backend gaps.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Overall comparison verdict.
    pub passed: bool,
    /// FNV-1a digest over every preceding field.
    pub content_digest: String,
}

impl AckermannObservedComparison {
    /// Validates both traces, exact shared contracts, tolerances, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == ACKERMANN_OBSERVED_COMPARISON_KIND,
            "comparison kind drift"
        );
        ensure!(
            self.schema_version == ACKERMANN_OBSERVED_SCHEMA_VERSION,
            "comparison schema drift"
        );
        self.first.validate()?;
        self.second.validate()?;
        ensure!(
            self.first.task_spec == self.second.task_spec,
            "TaskSpec mismatch"
        );
        ensure!(
            self.first.contract == self.second.contract,
            "contract mismatch"
        );
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "same backend"
        );
        ensure!(
            self.metrics == comparison_metrics(&self.first, &self.second)?,
            "comparison metric drift"
        );
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "comparison digest drift"
        );
        Ok(())
    }
}

/// Returns the actor-visible sensor-only Ackermann observation/action contract.
pub fn ackermann_observed_task_spec() -> TaskSpec {
    TaskSpec::new(
        ACKERMANN_OBSERVED_TASK_ID,
        SENSOR_PERIOD_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("estimated_position_m", TensorDType::F64, vec![2], "m"),
            TensorSpec::new("estimated_yaw_rad", TensorDType::F64, vec![1], "rad"),
            TensorSpec::new(
                "estimated_position_covariance_m2",
                TensorDType::F64,
                vec![2, 2],
                "m^2",
            ),
            TensorSpec::new(
                "estimated_yaw_variance_rad2",
                TensorDType::F64,
                vec![1],
                "rad^2",
            ),
            TensorSpec::new("estimated_speed_m_s", TensorDType::F64, vec![1], "m/s"),
            TensorSpec::new(
                "estimated_yaw_rate_rad_s",
                TensorDType::F64,
                vec![1],
                "rad/s",
            ),
            TensorSpec::new(
                "estimated_curvature_per_m",
                TensorDType::F64,
                vec![1],
                "1/m",
            ),
            TensorSpec::new("measured_steering_rad", TensorDType::F64, vec![2], "rad"),
            TensorSpec::new("measured_motor_current_a", TensorDType::F64, vec![4], "A"),
            TensorSpec::new("wheel_encoder_sequence", TensorDType::I64, vec![4], "1"),
            TensorSpec::new("wheel_encoder_status_code", TensorDType::U8, vec![4], "1")
                .with_bounds(TensorBounds::new(vec![0.0; 4], vec![3.0; 4])),
            TensorSpec::new("steering_encoder_sequence", TensorDType::I64, vec![2], "1"),
            TensorSpec::new(
                "steering_encoder_status_code",
                TensorDType::U8,
                vec![2],
                "1",
            )
            .with_bounds(TensorBounds::new(vec![0.0; 2], vec![3.0; 2])),
            TensorSpec::new("motor_sequence", TensorDType::I64, vec![4], "1"),
            TensorSpec::new("motor_status_code", TensorDType::U8, vec![4], "1")
                .with_bounds(TensorBounds::new(vec![0.0; 4], vec![3.0; 4])),
            TensorSpec::new("imu_sequence", TensorDType::I64, vec![1], "1"),
            TensorSpec::new("imu_status_code", TensorDType::U8, vec![1], "1")
                .with_bounds(TensorBounds::new(vec![0.0], vec![2.0])),
            TensorSpec::new("estimator_health_code", TensorDType::U8, vec![1], "1")
                .with_bounds(TensorBounds::new(vec![0.0], vec![5.0])),
            TensorSpec::new("target_speed_m_s", TensorDType::F64, vec![1], "m/s"),
            TensorSpec::new("target_yaw_rate_rad_s", TensorDType::F64, vec![1], "rad/s"),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new(
                "four_motor_terminal_voltage_v",
                TensorDType::F64,
                vec![4],
                "V",
            )
            .with_bounds(TensorBounds::new(
                vec![-MAXIMUM_VOLTAGE_V; 4],
                vec![MAXIMUM_VOLTAGE_V; 4],
            )),
            TensorSpec::new(
                "center_steering_target_rad",
                TensorDType::F64,
                vec![1],
                "rad",
            )
            .with_bounds(TensorBounds::new(
                vec![-MAXIMUM_CENTER_STEERING_RAD],
                vec![MAXIMUM_CENTER_STEERING_RAD],
            )),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("diagnostic_speed_tracking", 1.0, "m/s"),
            RewardTermSpec::new("diagnostic_yaw_rate_tracking", 1.0, "rad/s"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "diagnostic_out_of_bounds",
                TerminationKind::Failure,
            )],
            Some(TOTAL_STEPS / 10),
        ),
        ResetSpec::splitmix64(false),
    )
    .with_privileged_observation(ObservationSpec::new(vec![
        TensorSpec::new(
            "privileged_forward_distance_m",
            TensorDType::F64,
            vec![1],
            "m",
        ),
        TensorSpec::new(
            "privileged_forward_speed_m_s",
            TensorDType::F64,
            vec![1],
            "m/s",
        ),
        TensorSpec::new(
            "privileged_yaw_rate_rad_s",
            TensorDType::F64,
            vec![1],
            "rad/s",
        ),
    ]))
    .with_diagnostic_observation(ObservationSpec::new(vec![
        TensorSpec::new("decision_ticks", TensorDType::I64, vec![1], "tick"),
        TensorSpec::new("capture_ticks", TensorDType::I64, vec![1], "tick"),
    ]))
}

#[derive(Clone, Copy, Debug)]
struct SensorRig {
    wheel_joints: [Entity; 4],
    motor_entities: [Entity; 4],
}

/// Runs the nominal sensor-only controller through the suspended Ackermann plant.
pub fn run_ackermann_observed_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<AckermannObservedTrace> {
    run_ackermann_observed_trace_with_fault(backend, manifest, AckermannObservedFault::None)
}

fn run_ackermann_observed_trace_with_fault<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    fault: AckermannObservedFault,
) -> Result<AckermannObservedTrace> {
    match run_ackermann_observed_outcome(backend, manifest, fault)? {
        AckermannObservedRunOutcome::Trace(trace) => Ok(*trace),
        AckermannObservedRunOutcome::Failure(capsule) => {
            bail!(
                "sensor-only Ackermann failed closed: {:?}",
                capsule.failure_code
            )
        }
    }
}

/// Executes one unsafe sensor fault and returns its self-verifying Failure Capsule.
pub fn run_ackermann_observed_failure_capsule<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    fault: AckermannObservedFault,
) -> Result<AckermannObservedFailureCapsule> {
    match run_ackermann_observed_outcome(backend, manifest, fault)? {
        AckermannObservedRunOutcome::Failure(capsule) => Ok(*capsule),
        AckermannObservedRunOutcome::Trace(_) => bail!("fault did not fail closed"),
    }
}

fn run_ackermann_observed_outcome<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
    fault: AckermannObservedFault,
) -> Result<AckermannObservedRunOutcome> {
    manifest.validate()?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::Articulation,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ContactPointKinematics,
            PhysicsCapability::ExternalBodyWrench,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "capability drift"
    );
    let task_spec = ackermann_observed_task_spec();
    task_spec.validate()?;
    let mut contract = AckermannObservedContract::nominal();
    contract.fault = fault;
    contract.validate()?;
    let plant = wheel_plant_spec();
    let suspension = suspension_spec();
    let steering_actuator = steering_actuator_spec();
    ensure!(
        steering_actuator.is_valid(),
        "invalid steering actuator fixture"
    );
    let fixed_delta = SimDuration::from_ticks(ACKERMANN_OBSERVED_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 48,
    })?;
    let mut world = World::new();
    world.insert_resource(WorldRandom::new(WORLD_SEED));
    let ground = spawn_named(&mut world, "ackermann_observed_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_cuboid(Vec3::new(30.0, 0.5, 30.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let chassis = spawn_named(&mut world, "ackermann_observed_chassis");
    world.entity_mut(chassis).insert((
        RigidBody {
            mass_kg: CHASSIS_MASS_KG,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(0.0, -0.08, 0.0),
            ixx_kg_m2: 220.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 360.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 500.0,
        },
        MultibodyLink,
        frictionless_cuboid(Vec3::new(1.25, 0.10, 0.72)),
        CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
        Transform3::from_translation_rotation(
            Vec3::new(
                0.0,
                WHEEL_RADIUS_M + 0.15 - INITIAL_SUSPENSION_POSITION_M,
                0.0,
            ),
            Quat::IDENTITY,
        ),
    ));
    let stations = spawn_stations(&mut world, chassis, suspension)?;
    let rig = spawn_sensor_rig(&mut world, chassis, &stations, &contract);
    backend.sync_from_ecs(&mut world, physics_world)?;

    let initial_transform = *world
        .get::<Transform3>(chassis)
        .context("initial chassis")?;
    let mut drive_states = [LongitudinalDrivePathState::default(); 4];
    let mut conditioned_load_n = [0.0; 4];
    let mut pending_wrenches: Vec<ExternalBodyWrench> = Vec::new();
    let mut command_voltage_v = [0.0; 4];
    let mut center_steering_target_rad = 0.0;
    let mut steering_actuator_state = SteeringActuatorState::default();
    let mut bus = InMemoryDataBus::new();
    let mut estimator =
        AckermannImuOdometry::new(odometry_config(&contract), PoseSample::default())?;
    let mut controller = SpeedSteeringController::new(&contract);
    let mut last_wheel_sequence = 0;
    let mut samples = Vec::new();
    let mut squared_speed_estimation_error = 0.0;
    let mut squared_yaw_estimation_error = 0.0;
    let mut squared_speed_tracking_error = 0.0;
    let mut squared_yaw_tracking_error = 0.0;
    let mut scoring_samples = 0_u64;

    for zero_based_step in 0..TOTAL_STEPS {
        steering_actuator_state = evaluate_steering_actuator(
            steering_actuator,
            steering_actuator_state,
            center_steering_target_rad,
            dt_s,
        )?
        .state;
        let steering_targets = front_steering_targets(steering_actuator_state.position_rad);
        for (index, station) in stations.iter().enumerate() {
            if station.front {
                let target = if index == 0 {
                    steering_targets[0]
                } else {
                    steering_targets[1]
                };
                world
                    .entity_mut(station.wheel)
                    .insert(JointActuation::RevolutePosition {
                        target_position_rad: target,
                        stiffness_nm_per_rad: 1_500.0,
                        damping_nm_s_per_rad: 120.0,
                        max_effort_nm: 2_000.0,
                    });
            }
        }
        backend.sync_from_ecs(&mut world, physics_world)?;
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;

        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        let contacts = backend.contact_points(physics_world)?;
        let load_alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
        for (index, station) in stations.iter().enumerate() {
            let suspension_state = *world
                .get::<JointState>(station.slider)
                .with_context(|| format!("suspension state {index}"))?;
            let (position_m, velocity_m_s) = match suspension_state {
                JointState::Prismatic {
                    position_m,
                    velocity_m_s,
                } => (position_m, velocity_m_s),
                _ => anyhow::bail!("suspension joint state kind"),
            };
            world
                .entity_mut(station.slider)
                .insert(evaluate_suspension_strut(
                    suspension,
                    position_m,
                    velocity_m_s,
                )?);
            let wheel_transform = *world
                .get::<Transform3>(station.wheel)
                .with_context(|| format!("wheel transform {index}"))?;
            let forward_world = (wheel_transform.rotation * Vec3::X).normalize();
            let lateral_world = (wheel_transform.rotation * Vec3::Z).normalize();
            let raw_patch = aggregate_wheel_contact_patch(
                station.wheel,
                contacts,
                forward_world,
                lateral_world,
            )?;
            let bounded_load_n = raw_patch
                .map_or(0.0, |patch| patch.normal_load_n)
                .min(plant.tire.reference_load_n * plant.tire.maximum_load_ratio);
            conditioned_load_n[index] += load_alpha * (bounded_load_n - conditioned_load_n[index]);
            let patch = raw_patch.map(|mut patch| {
                patch.normal_load_n = conditioned_load_n[index];
                patch
            });
            let evaluation = evaluate_longitudinal_drive_path(
                plant,
                drive_states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: patch,
                    forward_world,
                    lateral_world,
                    command_voltage_v: command_voltage_v[index],
                },
                dt_s,
            )?;
            drive_states[index] = evaluation.state;
            if let Some(wrench) = evaluation.tire_wrench {
                pending_wrenches.push(wrench);
            }
            world
                .entity_mut(rig.wheel_joints[index])
                .insert(JointState::Revolute {
                    position_rad: evaluation.state.wheel_position_rad,
                    velocity_rad_s: evaluation.state.wheel_velocity_rad_s,
                });
            world
                .entity_mut(rig.motor_entities[index])
                .insert(evaluation.motor_telemetry);
        }

        let step = zero_based_step + 1;
        let decision_time = SimTime::from_ticks(step * ACKERMANN_OBSERVED_FIXED_DELTA_TICKS);
        sample_frontends(&mut world, decision_time, &mut bus)?;
        let Some(front_left) = bus.latest_available::<IncrementalEncoderFeedback>(
            STREAMS.front_left_wheel,
            decision_time,
        ) else {
            continue;
        };
        if front_left.sequence <= last_wheel_sequence {
            continue;
        }
        let estimate = match estimator.update(&bus, STREAMS, decision_time) {
            Ok(estimate) => estimate,
            Err(
                AckermannImuOdometryError::MissingAvailableFrame { .. }
                | AckermannImuOdometryError::NoNewSensorSet
                | AckermannImuOdometryError::InputSkew { .. }
                | AckermannImuOdometryError::StaleInput { .. },
            ) => continue,
            Err(error) => {
                if let Some(capsule) = build_failure_capsule(
                    &manifest,
                    &task_spec,
                    &contract,
                    step,
                    decision_time,
                    &bus,
                    error,
                )? {
                    return Ok(AckermannObservedRunOutcome::Failure(Box::new(capsule)));
                }
                return Err(error.into());
            }
        };
        last_wheel_sequence = front_left.sequence;
        let (target_speed_m_s, target_yaw_rate_rad_s) = targets_for_step(step);
        (command_voltage_v, center_steering_target_rad) = controller.update(
            estimate.linear_velocity_m_s,
            estimate.angular_velocity_rad_s,
            target_speed_m_s,
            target_yaw_rate_rad_s,
            contract.sensor_period_ticks as f64 / 1.0e9,
        );
        let wheel_frames = wheel_streams().map(|stream| {
            bus.latest_available::<IncrementalEncoderFeedback>(stream, decision_time)
                .expect("estimator accepted wheel frame")
        });
        let steering_frames = steering_streams().map(|stream| {
            bus.latest_available::<IncrementalEncoderFeedback>(stream, decision_time)
                .expect("estimator accepted steering frame")
        });
        let motor_frames = MOTOR_STREAMS.map(|stream| {
            bus.latest_available::<MotorElectricalFeedback>(stream, decision_time)
                .expect("motor feedback synchronized")
        });
        let imu_frame = bus
            .latest_available::<ImuFeedback>(STREAMS.imu, decision_time)
            .context("IMU unavailable after estimator update")?;
        if step > SETTLE_STEPS && estimate.health != AckermannImuOdometryHealth::Initializing {
            squared_speed_estimation_error +=
                (estimate.linear_velocity_m_s - body.linear_velocity_m_s.x).powi(2);
            squared_yaw_estimation_error +=
                (estimate.angular_velocity_rad_s - body.angular_velocity_rad_s.y).powi(2);
            squared_speed_tracking_error += (body.linear_velocity_m_s.x - target_speed_m_s).powi(2);
            squared_yaw_tracking_error +=
                (body.angular_velocity_rad_s.y - target_yaw_rate_rad_s).powi(2);
            scoring_samples += 1;
        }
        samples.push(AckermannObservedSample {
            step,
            decision_ticks: decision_time.ticks(),
            capture_ticks: estimate.provenance.capture_ticks,
            wheel_encoder_sequences: wheel_frames.each_ref().map(|frame| frame.sequence),
            wheel_encoder_status_codes: wheel_frames
                .each_ref()
                .map(|frame| encoder_status_code(frame.payload.status)),
            steering_encoder_sequences: steering_frames.each_ref().map(|frame| frame.sequence),
            steering_encoder_status_codes: steering_frames
                .each_ref()
                .map(|frame| encoder_status_code(frame.payload.status)),
            measured_steering_rad: steering_frames
                .each_ref()
                .map(|frame| frame.payload.position_rad),
            measured_motor_current_a: motor_frames.each_ref().map(|frame| frame.payload.current_a),
            motor_sequences: motor_frames.each_ref().map(|frame| frame.sequence),
            motor_status_codes: motor_frames
                .each_ref()
                .map(|frame| motor_status_code(frame.payload.status)),
            imu_sequence: imu_frame.sequence,
            imu_status_code: imu_status_code(imu_frame.payload.status),
            estimator_health_code: health_code(estimate.health),
            estimated_speed_m_s: estimate.linear_velocity_m_s,
            estimated_yaw_rate_rad_s: estimate.angular_velocity_rad_s,
            estimated_curvature_per_m: estimate.curvature_per_m,
            estimated_pose: [
                estimate.pose.position_m.x,
                estimate.pose.position_m.y,
                estimate.pose.yaw_rad,
            ],
            estimated_position_covariance_m2: [
                estimate.pose_covariance[0][0],
                estimate.pose_covariance[0][1],
                estimate.pose_covariance[1][0],
                estimate.pose_covariance[1][1],
            ],
            estimated_yaw_variance_rad2: estimate.pose_covariance[2][2],
            target_speed_m_s,
            target_yaw_rate_rad_s,
            command_voltage_v,
            center_steering_target_rad,
            steering_actuator_target_rad: steering_actuator_state.position_rad,
            privileged_forward_distance_m: transform.translation.x
                - initial_transform.translation.x,
            privileged_forward_speed_m_s: body.linear_velocity_m_s.x,
            privileged_yaw_rate_rad_s: body.angular_velocity_rad_s.y,
        });
    }
    ensure!(scoring_samples > 0, "no scoring samples");
    let final_sample = samples.last().context("no actor samples")?;
    let mut metrics = vec![
        metric(
            "final_forward_distance_m",
            "m",
            final_sample.privileged_forward_distance_m,
            0.5,
            10.0,
        ),
        metric(
            "final_estimated_speed_m_s",
            "m/s",
            final_sample.estimated_speed_m_s,
            0.3,
            2.0,
        ),
        metric(
            "final_estimated_yaw_rate_rad_s",
            "rad/s",
            final_sample.estimated_yaw_rate_rad_s,
            0.03,
            0.30,
        ),
        metric(
            "rms_speed_estimation_error_m_s",
            "m/s",
            (squared_speed_estimation_error / scoring_samples as f64).sqrt(),
            0.0,
            0.35,
        ),
        metric(
            "rms_yaw_rate_estimation_error_rad_s",
            "rad/s",
            (squared_yaw_estimation_error / scoring_samples as f64).sqrt(),
            0.0,
            0.12,
        ),
        metric(
            "rms_speed_tracking_error_m_s",
            "m/s",
            (squared_speed_tracking_error / scoring_samples as f64).sqrt(),
            0.0,
            0.70,
        ),
        metric(
            "rms_yaw_rate_tracking_error_rad_s",
            "rad/s",
            (squared_yaw_tracking_error / scoring_samples as f64).sqrt(),
            0.0,
            0.12,
        ),
        metric(
            "maximum_measured_motor_current_a",
            "A",
            samples
                .iter()
                .flat_map(|sample| sample.measured_motor_current_a)
                .map(f64::abs)
                .fold(0.0, f64::max),
            0.1,
            25.0,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = AckermannObservedTrace {
        kind: ACKERMANN_OBSERVED_TRACE_KIND.to_string(),
        schema_version: ACKERMANN_OBSERVED_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        contract,
        plant_spec: plant,
        wheel_station_specs: stations.map(|station| station.spec),
        suspension_spec: suspension,
        steering_actuator_spec: steering_actuator,
        fixed_delta_ticks: ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        seed: WORLD_SEED,
        steps: TOTAL_STEPS,
        samples,
        passed: metrics.iter().all(|metric| metric.passed),
        metrics,
        content_digest: String::new(),
    };
    trace.content_digest = trace_digest(&trace)?;
    trace.validate()?;
    Ok(AckermannObservedRunOutcome::Trace(Box::new(trace)))
}

/// Builds a self-verifying unit-aware comparison from two complete backend traces.
pub fn compare_ackermann_observed_traces(
    first: AckermannObservedTrace,
    second: AckermannObservedTrace,
) -> Result<AckermannObservedComparison> {
    first.validate()?;
    second.validate()?;
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = AckermannObservedComparison {
        kind: ACKERMANN_OBSERVED_COMPARISON_KIND.to_string(),
        schema_version: ACKERMANN_OBSERVED_SCHEMA_VERSION,
        first,
        second,
        passed: metrics.iter().all(|metric| metric.passed),
        metrics,
        content_digest: String::new(),
    };
    comparison.content_digest = comparison_digest(&comparison)?;
    comparison.validate()?;
    Ok(comparison)
}

#[allow(clippy::too_many_arguments)]
fn build_failure_capsule(
    manifest: &PhysicsBackendManifest,
    task_spec: &TaskSpec,
    contract: &AckermannObservedContract,
    failed_step: u64,
    decision_time: SimTime,
    bus: &InMemoryDataBus,
    error: AckermannImuOdometryError,
) -> Result<Option<AckermannObservedFailureCapsule>> {
    let failure_code = match (contract.fault, error) {
        (
            AckermannObservedFault::FrontLeftWheelStuck { .. },
            AckermannImuOdometryError::StuckValue { .. },
        ) => AckermannObservedFailureCode::WheelEncoderStuck,
        (
            AckermannObservedFault::FrontLeftWheelSaturate { .. },
            AckermannImuOdometryError::EncoderSaturated { .. },
        ) => AckermannObservedFailureCode::WheelEncoderSaturated,
        (
            AckermannObservedFault::FrontRightSteeringStuck { .. },
            AckermannImuOdometryError::StuckValue { .. },
        ) => AckermannObservedFailureCode::SteeringEncoderStuck,
        (AckermannObservedFault::ImuStuck { .. }, AckermannImuOdometryError::StuckValue { .. }) => {
            AckermannObservedFailureCode::ImuStuck
        }
        _ => return Ok(None),
    };
    let wheel_frames = wheel_streams()
        .map(|stream| bus.latest_available::<IncrementalEncoderFeedback>(stream, decision_time));
    let steering_frames = steering_streams()
        .map(|stream| bus.latest_available::<IncrementalEncoderFeedback>(stream, decision_time));
    let motor_frames = MOTOR_STREAMS
        .map(|stream| bus.latest_available::<MotorElectricalFeedback>(stream, decision_time));
    let imu_frame = bus.latest_available::<ImuFeedback>(STREAMS.imu, decision_time);
    let mut capsule = AckermannObservedFailureCapsule {
        kind: "rne_mobility_ackermann_observed_failure_capsule".to_string(),
        schema_version: ACKERMANN_OBSERVED_SCHEMA_VERSION,
        backend: manifest.clone(),
        task_spec: task_spec.clone(),
        contract: contract.clone(),
        failure_code,
        failed_step,
        decision_ticks: decision_time.ticks(),
        wheel_encoder_sequences: wheel_frames
            .each_ref()
            .map(|frame| frame.as_ref().map_or(0, |frame| frame.sequence)),
        wheel_encoder_status_codes: wheel_frames.each_ref().map(|frame| {
            frame
                .as_ref()
                .map_or(0, |frame| encoder_status_code(frame.payload.status))
        }),
        steering_encoder_sequences: steering_frames
            .each_ref()
            .map(|frame| frame.as_ref().map_or(0, |frame| frame.sequence)),
        steering_encoder_status_codes: steering_frames.each_ref().map(|frame| {
            frame
                .as_ref()
                .map_or(0, |frame| encoder_status_code(frame.payload.status))
        }),
        motor_sequences: motor_frames
            .each_ref()
            .map(|frame| frame.as_ref().map_or(0, |frame| frame.sequence)),
        motor_status_codes: motor_frames.each_ref().map(|frame| {
            frame
                .as_ref()
                .map_or(0, |frame| motor_status_code(frame.payload.status))
        }),
        imu_sequence: imu_frame.as_ref().map_or(0, |frame| frame.sequence),
        imu_status_code: imu_frame
            .as_ref()
            .map_or(0, |frame| imu_status_code(frame.payload.status)),
        content_digest: String::new(),
    };
    capsule.content_digest = failure_capsule_digest(&capsule)?;
    capsule.validate()?;
    Ok(Some(capsule))
}

fn spawn_sensor_rig(
    world: &mut World,
    chassis: Entity,
    stations: &[crate::ackermann_suspension::StationEntities; 4],
    contract: &AckermannObservedContract,
) -> SensorRig {
    let names = ["front_left", "rear_left", "front_right", "rear_right"];
    let wheel_joints = std::array::from_fn(|index| {
        spawn_virtual_wheel_encoder(
            world,
            chassis,
            names[index],
            wheel_streams()[index],
            if index == 0 {
                match contract.fault {
                    AckermannObservedFault::FrontLeftWheelDrop { sequence } => {
                        IncrementalEncoderFault::DropSequence { sequence }
                    }
                    AckermannObservedFault::FrontLeftWheelStuck { sequence } => {
                        IncrementalEncoderFault::StuckFromSequence { sequence }
                    }
                    _ => IncrementalEncoderFault::None,
                }
            } else {
                IncrementalEncoderFault::None
            },
            contract,
        )
    });
    for (front_index, station_index) in [0_usize, 2].into_iter().enumerate() {
        spawn_encoder_sensor(
            world,
            chassis,
            stations[station_index].wheel,
            ["front_left_steering", "front_right_steering"][front_index],
            steering_streams()[front_index],
            if front_index == 1 {
                match contract.fault {
                    AckermannObservedFault::FrontRightSteeringDrop { sequence } => {
                        IncrementalEncoderFault::DropSequence { sequence }
                    }
                    AckermannObservedFault::FrontRightSteeringStuck { sequence } => {
                        IncrementalEncoderFault::StuckFromSequence { sequence }
                    }
                    _ => IncrementalEncoderFault::None,
                }
            } else {
                IncrementalEncoderFault::None
            },
            contract.steering_encoder_counts_per_revolution,
            contract,
        );
    }
    let imu = spawn_named(world, "ackermann_observed_imu");
    world.entity_mut(imu).insert((
        ImuMount {
            body_entity: chassis,
            body_from_sensor: Transform3::from_translation_rotation(
                Vec3::new(0.2, 0.1, 0.0),
                Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            ),
        },
        ImuFeedbackSensor {
            spec: realistic_imu_spec(contract.calibrated_gyro_z_bias_rad_s),
            update_rate_hz: 100.0,
            sample_period_ticks: Some(contract.sensor_period_ticks),
            phase_offset_ticks: 0,
            latency_ticks: contract.sensor_latency_ticks,
            enabled: true,
            stream_id: STREAMS.imu,
            fault: match contract.fault {
                AckermannObservedFault::ImuDrop { sequence } => {
                    ImuFeedbackFault::DropSequence { sequence }
                }
                AckermannObservedFault::ImuStuck { sequence } => {
                    ImuFeedbackFault::StuckFromSequence { sequence }
                }
                _ => ImuFeedbackFault::None,
            },
        },
        ImuFeedbackSensorState::default(),
    ));
    let motor_entities = std::array::from_fn(|index| {
        let motor_entity = world.spawn(DcMotorCompletedTelemetry::default()).id();
        let sensor = spawn_named(world, format!("{}_motor_feedback", names[index]));
        world.entity_mut(sensor).insert((
            MotorElectricalFeedbackSensor {
                spec: MotorElectricalFeedbackSpec {
                    motor_entity,
                    current_range_a: 25.0,
                    voltage_range_v: 30.0,
                    minimum_temperature_c: -40.0,
                    maximum_temperature_c: 180.0,
                    current_offset_a: 0.02,
                    voltage_offset_v: -0.01,
                    temperature_offset_c: 0.0,
                    current_noise_std_a: 0.02,
                    voltage_noise_std_v: 0.01,
                    temperature_noise_std_c: 0.1,
                    current_resolution_a: 0.01,
                    voltage_resolution_v: 0.01,
                    temperature_resolution_c: 0.1,
                    seed: 71 + index as u64,
                },
                update_rate_hz: 100.0,
                sample_period_ticks: Some(contract.sensor_period_ticks),
                phase_offset_ticks: 0,
                latency_ticks: contract.sensor_latency_ticks,
                enabled: true,
                stream_id: MOTOR_STREAMS[index],
                fault: if index == 0 {
                    match contract.fault {
                        AckermannObservedFault::FrontLeftMotorDrop { sequence } => {
                            MotorElectricalFeedbackFault::DropSequence { sequence }
                        }
                        _ => MotorElectricalFeedbackFault::None,
                    }
                } else {
                    MotorElectricalFeedbackFault::None
                },
            },
            MotorElectricalFeedbackSensorState::default(),
        ));
        motor_entity
    });
    SensorRig {
        wheel_joints,
        motor_entities,
    }
}

fn spawn_virtual_wheel_encoder(
    world: &mut World,
    chassis: Entity,
    name: &str,
    stream: StreamId,
    fault: IncrementalEncoderFault,
    contract: &AckermannObservedContract,
) -> Entity {
    let joint = world
        .spawn((
            Joint {
                robot: chassis,
                parent_link: chassis,
                child_link: Entity::PLACEHOLDER,
                kind: JointKind::Continuous,
                limits: JointLimits::default(),
                axis: Vec3::Z,
                position: 0.0,
                velocity: 0.0,
            },
            JointState::Revolute {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
            },
        ))
        .id();
    world.get_mut::<Joint>(joint).expect("joint").child_link = joint;
    spawn_encoder_sensor(
        world,
        chassis,
        joint,
        &format!("{name}_wheel"),
        stream,
        fault,
        contract.wheel_encoder_counts_per_revolution,
        contract,
    );
    joint
}

#[allow(clippy::too_many_arguments)]
fn spawn_encoder_sensor(
    world: &mut World,
    chassis: Entity,
    joint: Entity,
    name: &str,
    stream: StreamId,
    fault: IncrementalEncoderFault,
    counts_per_revolution: u32,
    contract: &AckermannObservedContract,
) {
    let actuator = world
        .spawn(Actuator {
            robot: chassis,
            joint: Some(joint),
            name: format!("{name}_encoder_source"),
            mode: ControlMode::Position,
            target: ActuatorTarget::default(),
            limits: ActuatorLimits::default(),
        })
        .id();
    let sensor = spawn_named(world, format!("{name}_encoder"));
    world.entity_mut(sensor).insert((
        IncrementalEncoderSensor {
            spec: IncrementalEncoderSpec {
                actuator,
                counts_per_revolution,
                direction: 1,
                zero_offset_rad: 0.0,
                counter_bits: match contract.fault {
                    AckermannObservedFault::FrontLeftWheelSaturate { counter_bits }
                        if stream == STREAMS.front_left_wheel =>
                    {
                        counter_bits
                    }
                    _ => contract.encoder_counter_bits,
                },
                overflow_behavior: if matches!(
                    contract.fault,
                    AckermannObservedFault::FrontLeftWheelSaturate { .. }
                ) && stream == STREAMS.front_left_wheel
                {
                    IncrementalEncoderOverflowBehavior::Saturate
                } else {
                    IncrementalEncoderOverflowBehavior::Wrap
                },
                velocity_window_samples: 2,
                index_phase_rad: Some(0.0),
            },
            update_rate_hz: 100.0,
            sample_period_ticks: Some(contract.sensor_period_ticks),
            phase_offset_ticks: 0,
            latency_ticks: contract.sensor_latency_ticks,
            enabled: true,
            stream_id: stream,
            fault,
        },
        IncrementalEncoderSensorState::default(),
    ));
}

fn realistic_imu_spec(calibrated_bias_rad_s: f64) -> ImuSpec {
    ImuSpec {
        seed: 69,
        gyro: ImuAxisErrors {
            random_walk: 0.000_2,
            bias_instability: 0.000_02,
            bias_correlation_time_s: 100.0,
            rate_random_walk: 0.000_01,
            turn_on_bias: Vec3::new(0.0, 0.0, calibrated_bias_rad_s),
            scale_factor_error: Vec3::new(0.000_2, -0.000_1, 0.000_3),
            misalignment_rad: Vec3::new(0.000_1, -0.000_1, 0.000_1),
        },
        accel: ImuAxisErrors {
            random_walk: 0.002,
            bias_instability: 0.000_2,
            bias_correlation_time_s: 100.0,
            rate_random_walk: 0.000_1,
            turn_on_bias: Vec3::new(0.002, -0.001, 0.001),
            scale_factor_error: Vec3::new(0.000_3, -0.000_2, 0.000_1),
            misalignment_rad: Vec3::new(0.000_1, 0.000_1, -0.000_1),
        },
        gyro_range_rad_s: 10.0,
        accel_range_m_s2: 40.0,
        gyro_resolution_rad_s: 0.000_1,
        accel_resolution_m_s2: 0.001,
        ..ImuSpec::default()
    }
}

fn odometry_config(contract: &AckermannObservedContract) -> AckermannImuOdometryConfig {
    AckermannImuOdometryConfig {
        wheel_radius_m: WHEEL_RADIUS_M,
        wheelbase_m: WHEELBASE_M,
        track_width_m: TRACK_WIDTH_M,
        wheel_counts_per_revolution: [contract.wheel_encoder_counts_per_revolution; 4],
        wheel_counter_bits: [contract.encoder_counter_bits; 4],
        wheel_direction: [1; 4],
        max_abs_wheel_delta_counts: 20_000,
        gyro_z_direction: 1.0,
        gyro_z_bias_rad_s: contract.calibrated_gyro_z_bias_rad_s,
        gyro_yaw_weight: 0.8,
        disagreement_gyro_yaw_weight: 1.0,
        disagreement_threshold_rad: 0.01,
        steering_curvature_disagreement_per_m: 0.03,
        wheel_distance_std_m: 0.001,
        encoder_yaw_std_rad: 0.002,
        gyro_rate_std_rad_s: 0.001,
        max_input_skew_ticks: 0,
        max_frame_age_ticks: contract.sensor_latency_ticks,
    }
}

fn sample_frontends(world: &mut World, sim_time: SimTime, bus: &mut InMemoryDataBus) -> Result<()> {
    sample_incremental_encoder_sensors(world, sim_time, bus)?;
    sample_imu_feedback_sensors(world, sim_time, bus)?;
    sample_motor_electrical_feedback_sensors(world, sim_time, bus)?;
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct SpeedSteeringController {
    speed_kp_v_s_m: f64,
    speed_ki_v_m: f64,
    yaw_rate_kp_s: f64,
    speed_integral_error_m: f64,
}

impl SpeedSteeringController {
    fn new(contract: &AckermannObservedContract) -> Self {
        Self {
            speed_kp_v_s_m: contract.speed_kp_v_s_m,
            speed_ki_v_m: contract.speed_ki_v_m,
            yaw_rate_kp_s: contract.yaw_rate_kp_s,
            speed_integral_error_m: 0.0,
        }
    }

    fn update(
        &mut self,
        estimated_speed_m_s: f64,
        estimated_yaw_rate_rad_s: f64,
        target_speed_m_s: f64,
        target_yaw_rate_rad_s: f64,
        dt_s: f64,
    ) -> ([f64; 4], f64) {
        if target_speed_m_s == 0.0 {
            self.speed_integral_error_m = 0.0;
            return ([0.0; 4], 0.0);
        }
        let speed_error_m_s = target_speed_m_s - estimated_speed_m_s;
        self.speed_integral_error_m =
            (self.speed_integral_error_m + speed_error_m_s * dt_s).clamp(-2.0, 2.0);
        let voltage_v = (self.speed_kp_v_s_m * speed_error_m_s
            + self.speed_ki_v_m * self.speed_integral_error_m)
            .clamp(-MAXIMUM_VOLTAGE_V, MAXIMUM_VOLTAGE_V);
        let steering_feedforward_rad =
            (WHEELBASE_M * target_yaw_rate_rad_s / estimated_speed_m_s.abs().max(0.5)).atan();
        let steering_feedback_rad =
            self.yaw_rate_kp_s * (target_yaw_rate_rad_s - estimated_yaw_rate_rad_s);
        let steering_rad = (steering_feedforward_rad + steering_feedback_rad)
            .clamp(-MAXIMUM_CENTER_STEERING_RAD, MAXIMUM_CENTER_STEERING_RAD);
        ([voltage_v; 4], steering_rad)
    }
}

fn targets_for_step(step: u64) -> (f64, f64) {
    if step <= SETTLE_STEPS {
        (0.0, 0.0)
    } else if step <= TURN_START_STEP {
        (TARGET_SPEED_M_S, 0.0)
    } else {
        (TARGET_SPEED_M_S, TARGET_YAW_RATE_RAD_S)
    }
}

fn wheel_streams() -> [StreamId; 4] {
    [
        STREAMS.front_left_wheel,
        STREAMS.rear_left_wheel,
        STREAMS.front_right_wheel,
        STREAMS.rear_right_wheel,
    ]
}

fn steering_streams() -> [StreamId; 2] {
    [STREAMS.front_left_steering, STREAMS.front_right_steering]
}

fn health_code(health: AckermannImuOdometryHealth) -> u8 {
    match health {
        AckermannImuOdometryHealth::Initializing => 0,
        AckermannImuOdometryHealth::Nominal => 1,
        AckermannImuOdometryHealth::InputSequenceGap => 2,
        AckermannImuOdometryHealth::ImuSaturated => 3,
        AckermannImuOdometryHealth::SteeringDisagreement => 4,
        AckermannImuOdometryHealth::WheelImuDisagreement => 5,
    }
}

fn encoder_status_code(status: IncrementalEncoderStatus) -> u8 {
    match status {
        IncrementalEncoderStatus::Initializing => 0,
        IncrementalEncoderStatus::Nominal => 1,
        IncrementalEncoderStatus::CounterSaturated => 2,
        IncrementalEncoderStatus::StuckValue => 3,
    }
}

fn motor_status_code(status: MotorElectricalFeedbackStatus) -> u8 {
    match status {
        MotorElectricalFeedbackStatus::Nominal => 0,
        MotorElectricalFeedbackStatus::Saturated => 1,
        MotorElectricalFeedbackStatus::TemperatureUnavailable => 2,
        MotorElectricalFeedbackStatus::StuckValue => 3,
    }
}

fn imu_status_code(status: ImuFeedbackStatus) -> u8 {
    match status {
        ImuFeedbackStatus::Nominal => 0,
        ImuFeedbackStatus::Saturated => 1,
        ImuFeedbackStatus::StuckValue => 2,
    }
}

fn metric(id: &str, unit: &str, value: f64, minimum: f64, maximum: f64) -> MobilityBenchmarkMetric {
    MobilityBenchmarkMetric {
        id: id.to_string(),
        unit: unit.to_string(),
        value,
        minimum,
        maximum,
        passed: value.is_finite() && (minimum..=maximum).contains(&value),
    }
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "missing metrics");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metrics are not canonical"
    );
    for metric in metrics {
        ensure!(
            metric.value.is_finite()
                && metric.minimum.is_finite()
                && metric.maximum.is_finite()
                && metric.minimum <= metric.maximum,
            "invalid metric {}",
            metric.id
        );
        ensure!(
            metric.passed == (metric.minimum..=metric.maximum).contains(&metric.value),
            "metric verdict drift"
        );
    }
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "verdict drift"
    );
    Ok(())
}

fn comparison_metrics(
    first: &AckermannObservedTrace,
    second: &AckermannObservedTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let mut metrics = vec![
        gap_metric(
            "forward_distance_gap_m",
            "m",
            first,
            second,
            "final_forward_distance_m",
            0.08,
        )?,
        gap_metric(
            "estimated_speed_gap_m_s",
            "m/s",
            first,
            second,
            "final_estimated_speed_m_s",
            0.08,
        )?,
        gap_metric(
            "estimated_yaw_rate_gap_rad_s",
            "rad/s",
            first,
            second,
            "final_estimated_yaw_rate_rad_s",
            0.03,
        )?,
        gap_metric(
            "speed_estimation_error_gap_m_s",
            "m/s",
            first,
            second,
            "rms_speed_estimation_error_m_s",
            0.08,
        )?,
        gap_metric(
            "yaw_estimation_error_gap_rad_s",
            "rad/s",
            first,
            second,
            "rms_yaw_rate_estimation_error_rad_s",
            0.03,
        )?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(metrics)
}

fn gap_metric(
    id: &str,
    unit: &str,
    first: &AckermannObservedTrace,
    second: &AckermannObservedTrace,
    source: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let gap = (metric_value(first, source)? - metric_value(second, source)?).abs();
    Ok(metric(id, unit, gap, 0.0, maximum))
}

fn metric_value(trace: &AckermannObservedTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn trace_digest(trace: &AckermannObservedTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn failure_capsule_digest(capsule: &AckermannObservedFailureCapsule) -> Result<String> {
    let mut canonical = capsule.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &AckermannObservedComparison) -> Result<String> {
    let mut canonical = comparison.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    format!("fnv1a64:{digest:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn actor_contract_contains_no_truth_tensor() {
        let task = ackermann_observed_task_spec();
        task.validate().unwrap();
        assert!(task
            .observation
            .tensors
            .iter()
            .all(|tensor| !tensor.name.contains("truth") && !tensor.name.contains("privileged")));
        assert_eq!(
            task.privileged_observation
                .as_ref()
                .unwrap()
                .tensors
                .iter()
                .map(|tensor| tensor.name.as_str())
                .collect::<Vec<_>>(),
            [
                "privileged_forward_distance_m",
                "privileged_forward_speed_m_s",
                "privileged_yaw_rate_rad_s",
            ]
        );
        assert_eq!(
            task.diagnostic_observation
                .as_ref()
                .unwrap()
                .tensors
                .iter()
                .map(|tensor| tensor.name.as_str())
                .collect::<Vec<_>>(),
            ["decision_ticks", "capture_ticks"]
        );
    }

    #[test]
    fn rapier_sensor_only_ackermann_trace_passes_and_is_deterministic() {
        let first =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        first.validate().unwrap();
        let second =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn wheel_and_steering_drops_reach_estimator_health_and_recover() {
        for fault in [
            AckermannObservedFault::FrontLeftWheelDrop { sequence: 200 },
            AckermannObservedFault::FrontRightSteeringDrop { sequence: 200 },
            AckermannObservedFault::ImuDrop { sequence: 200 },
        ] {
            let trace = run_ackermann_observed_trace_with_fault(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap();
            assert!(trace
                .samples
                .iter()
                .any(|sample| sample.estimator_health_code == 2));
            assert_eq!(
                trace.samples.last().unwrap().estimator_health_code,
                health_code(AckermannImuOdometryHealth::Nominal)
            );
        }
    }

    #[test]
    fn motor_drop_is_visible_without_changing_plant_or_controller_contract() {
        let nominal =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let dropped = run_ackermann_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            AckermannObservedFault::FrontLeftMotorDrop { sequence: 200 },
        )
        .unwrap();
        assert!(dropped
            .samples
            .windows(2)
            .any(|pair| { pair[1].motor_sequences[0] > pair[0].motor_sequences[0] + 1 }));
        assert_eq!(nominal.task_spec, dropped.task_spec);
        assert_eq!(nominal.plant_spec, dropped.plant_spec);
    }

    #[test]
    fn fatal_motion_inputs_emit_deterministic_failure_capsules() {
        for (fault, expected_code) in [
            (
                AckermannObservedFault::FrontLeftWheelStuck { sequence: 200 },
                AckermannObservedFailureCode::WheelEncoderStuck,
            ),
            (
                AckermannObservedFault::FrontLeftWheelSaturate { counter_bits: 4 },
                AckermannObservedFailureCode::WheelEncoderSaturated,
            ),
            (
                AckermannObservedFault::FrontRightSteeringStuck { sequence: 200 },
                AckermannObservedFailureCode::SteeringEncoderStuck,
            ),
            (
                AckermannObservedFault::ImuStuck { sequence: 200 },
                AckermannObservedFailureCode::ImuStuck,
            ),
        ] {
            let first = run_ackermann_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap();
            let second = run_ackermann_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap();
            assert_eq!(first.failure_code, expected_code);
            assert_eq!(first, second);
            first.validate().unwrap();

            let mut tampered = first.clone();
            tampered.failed_step += 1;
            assert!(tampered.validate().is_err());

            let error = run_ackermann_observed_trace_with_fault(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap_err();
            assert!(error.to_string().contains("failed closed"));
        }
    }

    #[test]
    fn trace_tampering_is_detected() {
        let mut trace =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples[1].estimated_speed_m_s += 0.1;
        assert!(trace.validate().is_err());
    }

    #[test]
    fn completed_steering_target_must_match_controller_command_replay() {
        let mut trace =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let sample = trace
            .samples
            .iter_mut()
            .find(|sample| sample.steering_actuator_target_rad.abs() > 0.01)
            .unwrap();
        sample.steering_actuator_target_rad += 0.001;
        trace.content_digest = trace_digest(&trace).unwrap();

        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_sensor_only_ackermann_traces_pass() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_ackermann_observed_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(
                ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_ackermann_observed_traces(rapier, mujoco).unwrap();
        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_emit_compatible_ackermann_failure_capsules() {
        use rne_physics_mujoco::MuJoCoBackend;

        for fault in [
            AckermannObservedFault::FrontLeftWheelStuck { sequence: 200 },
            AckermannObservedFault::FrontLeftWheelSaturate { counter_bits: 4 },
            AckermannObservedFault::FrontRightSteeringStuck { sequence: 200 },
            AckermannObservedFault::ImuStuck { sequence: 200 },
        ] {
            let rapier = run_ackermann_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap();
            let mujoco = run_ackermann_observed_failure_capsule(
                MuJoCoBackend::new(SimDuration::from_ticks(
                    ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
                ))
                .unwrap(),
                MuJoCoBackend::manifest(),
                fault,
            )
            .unwrap();
            rapier.validate().unwrap();
            mujoco.validate().unwrap();
            assert_eq!(rapier.task_spec, mujoco.task_spec);
            assert_eq!(rapier.contract, mujoco.contract);
            assert_eq!(rapier.failure_code, mujoco.failure_code);
        }
    }
}
