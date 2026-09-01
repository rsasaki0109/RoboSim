//! Sensor-only yaw-rate control over the four-wheel skid plant.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    ActionSpec, FourWheelEncoderStreams, FourWheelSideEncoderFusion,
    FourWheelSideEncoderFusionConfig, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec,
    SideEncoderStreams, TaskSpec, TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec,
    TerminationKind, TerminationSpec, WheelImuOdometry, WheelImuOdometryConfig,
    WheelImuOdometryError, WheelImuOdometryHealth, WheelImuOdometryStreams,
    FOUR_WHEEL_SIDE_FUSED_COUNTER_BITS,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, ImuFeedback, ImuFeedbackStatus, InMemoryDataBus, IncrementalEncoderFeedback,
    MotorElectricalFeedback, MotorElectricalFeedbackStatus, PoseSample, StreamId,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, CollisionGroups, ExternalBodyWrench, PhysicsBackend,
    PhysicsBackendManifest, PhysicsCapability, PhysicsWorldDesc, RigidBody, RigidBodyInertia,
    RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, resolve_wheel_station_frame,
    Actuator, ActuatorLimits, ActuatorTarget, ControlMode, DcMotorCompletedTelemetry, Joint,
    JointKind, JointLimits, LongitudinalDrivePathInput, LongitudinalDrivePathState,
    LongitudinalMobilityPlantSpec, WheelStationSpec,
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

use crate::per_wheel::{
    frictionless_cuboid, spawn_wheel_stations, wheel_plant_spec, wheel_station_specs,
    CONTACT_LOAD_FILTER_TIME_CONSTANT_S,
};
use crate::MobilityBenchmarkMetric;

/// Trace discriminator for the four-wheel sensor-only skid controller.
pub const PER_WHEEL_OBSERVED_TRACE_KIND: &str = "rne_mobility_per_wheel_observed_trace";
/// Schema version for the four-wheel sensor-only trace.
pub const PER_WHEEL_OBSERVED_SCHEMA_VERSION: u32 = 1;
/// Stable task identity shared by both rigid-body backends.
pub const PER_WHEEL_OBSERVED_TASK_ID: &str = "mobility_per_wheel_sensor_yaw_rate_v1";
/// One-millisecond physics step in simulation ticks.
pub const PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SENSOR_PERIOD_TICKS: u64 = 10_000_000;
const SENSOR_LATENCY_TICKS: u64 = 2_000_000;
const SETTLE_STEPS: u64 = 500;
const DRIVE_STEPS: u64 = 3_000;
const TOTAL_STEPS: u64 = SETTLE_STEPS + DRIVE_STEPS;
const TARGET_YAW_RATE_RAD_S: f64 = 0.30;
const MAXIMUM_VOLTAGE_V: f64 = 24.0;
const ENCODER_COUNTS_PER_REVOLUTION: u32 = 2_048;
const WORLD_SEED: u64 = 0;
const SELF_COLLISION_GROUP: u32 = 1;
const PHYSICAL_ENCODER_STREAMS: FourWheelEncoderStreams = FourWheelEncoderStreams {
    front_left: StreamId::new(2_001),
    rear_left: StreamId::new(2_002),
    front_right: StreamId::new(2_003),
    rear_right: StreamId::new(2_004),
};
const SIDE_ENCODER_STREAMS: SideEncoderStreams = SideEncoderStreams {
    left: StreamId::new(2_005),
    right: StreamId::new(2_006),
};
const IMU_STREAM: StreamId = StreamId::new(2_007);
const MOTOR_STREAMS: [StreamId; 4] = [
    StreamId::new(2_011),
    StreamId::new(2_012),
    StreamId::new(2_013),
    StreamId::new(2_014),
];

/// Frozen sensor and controller configuration retained in every trace.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelObservedContract {
    /// Physical encoder resolution in decoded counts per revolution.
    pub encoder_counts_per_revolution: u32,
    /// Physical signed encoder counter width.
    pub encoder_counter_bits: u8,
    /// Sensor capture period in simulation ticks.
    pub sensor_period_ticks: u64,
    /// Capture-to-availability delay in simulation ticks.
    pub sensor_latency_ticks: u64,
    /// Calibrated IMU yaw-rate bias in radians per second.
    pub calibrated_gyro_z_bias_rad_s: f64,
    /// Yaw-rate PI proportional gain in volts per `(rad/s)`.
    pub controller_kp_v_s_rad: f64,
    /// Yaw-rate PI integral gain in volts per radian.
    pub controller_ki_v_rad: f64,
    /// Deterministic frontend fault selected for this run.
    pub fault: PerWheelObservedFault,
}

/// One typed sensor fault applied without modifying plant truth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PerWheelObservedFault {
    /// Nominal sensor suite.
    #[default]
    None,
    /// Drop one front-left physical encoder frame.
    FrontLeftEncoderDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold the front-left physical encoder from this sequence onward.
    FrontLeftEncoderStuck {
        /// One-based first attempted sequence to hold.
        sequence: u64,
    },
    /// Saturate the front-left encoder's signed counter at the declared width.
    FrontLeftEncoderSaturate {
        /// Reduced signed hardware-counter width.
        counter_bits: u8,
    },
    /// Drop one front-left motor-current frame.
    FrontLeftMotorDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold front-left motor feedback from this sequence onward.
    FrontLeftMotorStuck {
        /// One-based first attempted sequence to hold.
        sequence: u64,
    },
    /// Drop one mounted-IMU frame.
    ImuDrop {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold mounted-IMU output from this sequence onward.
    ImuStuck {
        /// One-based first attempted sequence to hold.
        sequence: u64,
    },
    /// Reduce the gyro range so physical yaw-rate clips.
    ImuSaturate {
        /// Symmetric gyroscope clipping range in radians per second.
        gyro_range_rad_s: f64,
    },
}

impl PerWheelObservedContract {
    fn nominal() -> Self {
        Self {
            encoder_counts_per_revolution: ENCODER_COUNTS_PER_REVOLUTION,
            encoder_counter_bits: 32,
            sensor_period_ticks: SENSOR_PERIOD_TICKS,
            sensor_latency_ticks: SENSOR_LATENCY_TICKS,
            calibrated_gyro_z_bias_rad_s: 0.001,
            controller_kp_v_s_rad: 20.0,
            controller_ki_v_rad: 18.0,
            fault: PerWheelObservedFault::None,
        }
    }

    fn validate(&self) -> Result<()> {
        let mut expected = Self::nominal();
        expected.fault = self.fault;
        ensure!(*self == expected, "contract drift");
        match self.fault {
            PerWheelObservedFault::None => {}
            PerWheelObservedFault::FrontLeftEncoderDrop { sequence }
            | PerWheelObservedFault::FrontLeftEncoderStuck { sequence }
            | PerWheelObservedFault::FrontLeftMotorDrop { sequence }
            | PerWheelObservedFault::FrontLeftMotorStuck { sequence }
            | PerWheelObservedFault::ImuDrop { sequence }
            | PerWheelObservedFault::ImuStuck { sequence } => {
                ensure!(sequence > 1, "invalid fault sequence");
            }
            PerWheelObservedFault::FrontLeftEncoderSaturate { counter_bits } => {
                ensure!((2..=16).contains(&counter_bits), "invalid saturation width");
            }
            PerWheelObservedFault::ImuSaturate { gyro_range_rad_s } => {
                ensure!(
                    gyro_range_rad_s.is_finite() && gyro_range_rad_s > 0.0,
                    "invalid gyro saturation range"
                );
            }
        }
        Ok(())
    }
}

/// One controller decision with actor measurements and separately named truth evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelObservedSample {
    /// Completed physics step.
    pub step: u64,
    /// Decision timestamp in simulation ticks.
    pub decision_ticks: u64,
    /// Newest synchronized capture timestamp in ticks.
    pub capture_ticks: u64,
    /// Four physical encoder sequences in station order.
    pub encoder_sequences: [u64; 4],
    /// Four measured motor-current values in amperes.
    pub measured_motor_current_a: [f64; 4],
    /// Four motor-feedback stream sequences.
    pub motor_sequences: [u64; 4],
    /// Four stable motor-feedback status codes.
    pub motor_status_codes: [u8; 4],
    /// Mounted-IMU stream sequence.
    pub imu_sequence: u64,
    /// Stable mounted-IMU status code.
    pub imu_status_code: u8,
    /// Sensor-only estimated yaw rate in radians per second.
    pub estimated_yaw_rate_rad_s: f64,
    /// Wheel/IMU yaw innovation in radians.
    pub yaw_innovation_rad: f64,
    /// Stable estimator health code.
    pub health_code: u8,
    /// Task-owned target yaw rate in radians per second.
    pub target_yaw_rate_rad_s: f64,
    /// Left/right voltage action generated from actor-visible inputs.
    pub command_voltage_v: [f64; 2],
    /// Privileged integrated chassis yaw in radians.
    pub privileged_integrated_yaw_rad: f64,
    /// Privileged chassis yaw rate in radians per second.
    pub privileged_yaw_rate_rad_s: f64,
}

/// Self-verifying sensor-only four-wheel trace.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelObservedTrace {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// Physics backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact actor/action task contract.
    pub task_spec: TaskSpec,
    /// Exact sensor and controller contract.
    pub contract: PerWheelObservedContract,
    /// Exact motor, transmission, wheel, tire, and road force-element profile.
    pub plant_spec: LongitudinalMobilityPlantSpec,
    /// Ordered physical station geometry matching encoder/current sample order.
    pub wheel_station_specs: [WheelStationSpec; 4],
    /// First-order raw-contact to tire-load filter time constant in seconds.
    pub contact_load_filter_time_constant_s: f64,
    /// Fixed physics step in simulation ticks.
    pub fixed_delta_ticks: u64,
    /// Explicit deterministic world seed.
    pub seed: u64,
    /// Completed physics steps.
    pub steps: u64,
    /// Ordered controller decisions.
    pub samples: Vec<PerWheelObservedSample>,
    /// Unit-bearing acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Aggregate verdict.
    pub passed: bool,
    /// FNV-1a digest of this trace with the field empty.
    pub content_digest: String,
}

/// Two complete sensor-only traces plus explicit SI-unit backend tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PerWheelObservedComparison {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// First complete backend trace.
    pub first: PerWheelObservedTrace,
    /// Second complete backend trace.
    pub second: PerWheelObservedTrace,
    /// Ordered absolute cross-backend gaps.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Aggregate comparison verdict.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl PerWheelObservedTrace {
    /// Recomputes frozen contracts, ordering, metric verdicts, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == PER_WHEEL_OBSERVED_TRACE_KIND, "kind mismatch");
        ensure!(
            self.schema_version == PER_WHEEL_OBSERVED_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.backend.validate()?;
        self.task_spec.validate()?;
        ensure!(
            self.task_spec == per_wheel_observed_task_spec(),
            "TaskSpec drift"
        );
        ensure!(
            self.task_spec.observation.tensors.iter().all(|tensor| {
                !tensor.name.contains("truth") && !tensor.name.contains("privileged")
            }),
            "actor observation exposes truth"
        );
        self.contract.validate()?;
        ensure!(self.plant_spec == wheel_plant_spec(), "plant drift");
        ensure!(
            self.wheel_station_specs == wheel_station_specs(),
            "station drift"
        );
        ensure!(
            self.contact_load_filter_time_constant_s == CONTACT_LOAD_FILTER_TIME_CONSTANT_S,
            "contact load filter drift"
        );
        ensure!(
            self.fixed_delta_ticks == PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
            "step drift"
        );
        ensure!(
            self.seed == WORLD_SEED && self.steps == TOTAL_STEPS,
            "execution drift"
        );
        ensure!(self.samples.len() > 100, "actor decisions omitted");
        ensure!(
            self.samples.windows(2).all(|pair| {
                pair[0].decision_ticks < pair[1].decision_ticks
                    && (0..4).all(|index| {
                        pair[0].encoder_sequences[index] < pair[1].encoder_sequences[index]
                    })
            }),
            "decision order"
        );
        for sample in &self.samples {
            ensure!(
                sample.decision_ticks == sample.step * self.fixed_delta_ticks,
                "timestamp drift"
            );
            ensure!(
                sample.capture_ticks <= sample.decision_ticks,
                "future sensor frame"
            );
            ensure!(
                sample.decision_ticks - sample.capture_ticks <= SENSOR_LATENCY_TICKS,
                "stale sensor frame"
            );
            ensure!(
                sample
                    .measured_motor_current_a
                    .iter()
                    .all(|value| value.is_finite()),
                "non-finite current"
            );
            ensure!(
                sample.motor_status_codes.iter().all(|status| *status <= 3)
                    && sample.imu_status_code <= 2,
                "invalid sensor status code"
            );
            ensure!(
                sample.estimated_yaw_rate_rad_s.is_finite()
                    && sample.yaw_innovation_rad.is_finite()
                    && sample.privileged_yaw_rate_rad_s.is_finite(),
                "non-finite yaw rate"
            );
            let expected_target = if sample.step > SETTLE_STEPS {
                TARGET_YAW_RATE_RAD_S
            } else {
                0.0
            };
            ensure!(
                sample.target_yaw_rate_rad_s == expected_target,
                "target drift"
            );
            ensure!(
                sample
                    .command_voltage_v
                    .iter()
                    .all(|value| (-MAXIMUM_VOLTAGE_V..=MAXIMUM_VOLTAGE_V).contains(value)),
                "action bound"
            );
            ensure!(
                sample.command_voltage_v[0] == -sample.command_voltage_v[1],
                "non-differential action"
            );
        }
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == trace_digest(self)?,
            "digest mismatch"
        );
        Ok(())
    }
}

impl PerWheelObservedComparison {
    /// Recomputes trace integrity, shared contracts, cross-backend metrics, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_mobility_per_wheel_observed_comparison",
            "comparison kind mismatch"
        );
        ensure!(
            self.schema_version == PER_WHEEL_OBSERVED_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.first.validate()?;
        self.second.validate()?;
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "distinct backends required"
        );
        ensure!(
            self.first.task_spec == self.second.task_spec
                && self.first.contract == self.second.contract
                && self.first.plant_spec == self.second.plant_spec
                && self.first.wheel_station_specs == self.second.wheel_station_specs
                && self.first.contact_load_filter_time_constant_s
                    == self.second.contact_load_filter_time_constant_s
                && self.first.fixed_delta_ticks == self.second.fixed_delta_ticks,
            "execution contract mismatch"
        );
        ensure!(
            self.metrics == comparison_metrics(&self.first, &self.second)?,
            "comparison metric drift"
        );
        validate_metrics(&self.metrics, self.passed)?;
        ensure!(
            self.content_digest == comparison_digest(self)?,
            "comparison digest mismatch"
        );
        Ok(())
    }
}

/// Returns the exact sensor-only yaw-rate task shared by both backends.
pub fn per_wheel_observed_task_spec() -> TaskSpec {
    TaskSpec::new(
        PER_WHEEL_OBSERVED_TASK_ID,
        SENSOR_PERIOD_TICKS as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new(
                "estimated_yaw_rate_rad_s",
                TensorDType::F64,
                vec![],
                "rad/s",
            ),
            TensorSpec::new("yaw_innovation_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("max_input_age_s", TensorDType::F64, vec![], "s")
                .with_bounds(TensorBounds::broadcast(0.0, 0.01)),
            TensorSpec::new("health_code", TensorDType::U8, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 4.0)),
            TensorSpec::new("physical_encoder_sequence", TensorDType::I64, vec![4], "1"),
            TensorSpec::new("measured_motor_current_a", TensorDType::F64, vec![4], "A"),
            TensorSpec::new("motor_sequence", TensorDType::I64, vec![4], "1"),
            TensorSpec::new("motor_status_code", TensorDType::U8, vec![4], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 3.0)),
            TensorSpec::new("imu_sequence", TensorDType::I64, vec![], "1"),
            TensorSpec::new("imu_status_code", TensorDType::U8, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 2.0)),
            TensorSpec::new("target_yaw_rate_rad_s", TensorDType::F64, vec![], "rad/s")
                .with_bounds(TensorBounds::broadcast(0.0, TARGET_YAW_RATE_RAD_S)),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "left_right_motor_terminal_voltage_v",
            TensorDType::F64,
            vec![2],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(
            -MAXIMUM_VOLTAGE_V,
            MAXIMUM_VOLTAGE_V,
        ))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_yaw_rate_tracking_error_rad_s", -1.0, "rad/s"),
            RewardTermSpec::new("task_step", -0.001, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "truth_out_of_bounds",
                TerminationKind::Failure,
            )],
            Some(TOTAL_STEPS / 10),
        ),
        ResetSpec::splitmix64(false),
    )
}

#[derive(Clone, Copy, Debug)]
struct SensorRig {
    wheel_joints: [Entity; 4],
    motor_entities: [Entity; 4],
}

/// Runs the sensor-only controller through the physical four-wheel plant.
pub fn run_per_wheel_observed_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<PerWheelObservedTrace> {
    run_per_wheel_observed_trace_with_fault(backend, manifest, PerWheelObservedFault::None)
}

fn run_per_wheel_observed_trace_with_fault<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
    fault: PerWheelObservedFault,
) -> Result<PerWheelObservedTrace> {
    manifest.validate()?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ContactPointKinematics,
            PhysicsCapability::ExternalBodyWrench,
            PhysicsCapability::Articulation,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "capability drift"
    );
    let task_spec = per_wheel_observed_task_spec();
    task_spec.validate()?;
    let mut contract = PerWheelObservedContract::nominal();
    contract.fault = fault;
    contract.validate()?;
    let fixed_delta = SimDuration::from_ticks(PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 24,
    })?;
    let mut world = World::new();
    world.insert_resource(WorldRandom::new(WORLD_SEED));
    let ground = spawn_named(&mut world, "per_wheel_observed_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_cuboid(Vec3::new(20.0, 0.5, 20.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let chassis = spawn_named(&mut world, "per_wheel_observed_chassis");
    world.entity_mut(chassis).insert((
        RigidBody {
            mass_kg: 80.0,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: 8.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 12.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 10.0,
        },
        frictionless_cuboid(Vec3::new(0.5, 0.12, 0.24)),
        CollisionGroups::without_self_collision(SELF_COLLISION_GROUP),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.37, 0.0), Quat::IDENTITY),
    ));
    let stations = spawn_wheel_stations(&mut world, chassis);
    let rig = spawn_sensor_rig(&mut world, chassis, &contract);
    backend.sync_from_ecs(&mut world, physics_world)?;

    let plant = wheel_plant_spec();
    let mut states = [LongitudinalDrivePathState::default(); 4];
    let mut pending_wrenches: Vec<ExternalBodyWrench> = Vec::new();
    let mut conditioned_load_n = [0.0; 4];
    let mut command_voltage_v = [0.0; 2];
    let mut integrated_yaw_rad = 0.0;
    let mut bus = InMemoryDataBus::new();
    let mut side_fusion = FourWheelSideEncoderFusion::new(FourWheelSideEncoderFusionConfig {
        counts_per_revolution: ENCODER_COUNTS_PER_REVOLUTION,
        counter_bits: contract.encoder_counter_bits,
        max_abs_wheel_delta_counts: 10_000,
        max_input_skew_ticks: 0,
        max_frame_age_ticks: SENSOR_LATENCY_TICKS,
    })?;
    let mut estimator = WheelImuOdometry::new(odometry_config(&contract), PoseSample::default())?;
    let mut controller = YawRateController::new(&contract);
    let mut last_left_sequence = 0;
    let mut samples = Vec::new();
    let mut squared_estimation_error = 0.0;
    let mut squared_tracking_error = 0.0;
    let mut scoring_samples = 0_u64;

    for zero_based_step in 0..TOTAL_STEPS {
        for wrench in pending_wrenches.drain(..) {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;
        let transform = *world.get::<Transform3>(chassis).context("chassis pose")?;
        let body = *world.get::<RigidBody>(chassis).context("chassis body")?;
        integrated_yaw_rad += body.angular_velocity_rad_s.y * dt_s;
        let contacts = backend.contact_points(physics_world)?;
        for (index, station) in stations.iter().enumerate() {
            let frame = resolve_wheel_station_frame(
                station.spec,
                0.0,
                transform,
                body.linear_velocity_m_s,
                body.angular_velocity_rad_s,
            )?;
            let raw_patch = aggregate_wheel_contact_patch(
                station.entity,
                contacts,
                frame.forward_world,
                frame.lateral_world,
            )?;
            let raw_load = raw_patch.map_or(0.0, |patch| patch.normal_load_n);
            let bounded_load =
                raw_load.min(plant.tire.reference_load_n * plant.tire.maximum_load_ratio);
            let alpha = dt_s / (CONTACT_LOAD_FILTER_TIME_CONSTANT_S + dt_s);
            conditioned_load_n[index] += alpha * (bounded_load - conditioned_load_n[index]);
            let patch = raw_patch.map(|mut patch| {
                patch.normal_load_n = conditioned_load_n[index];
                patch
            });
            let side = usize::from(station.spec.center_body_m.z <= 0.0);
            let evaluation = evaluate_longitudinal_drive_path(
                plant,
                states[index],
                LongitudinalDrivePathInput {
                    carrier_patch: patch,
                    forward_world: frame.forward_world,
                    lateral_world: frame.lateral_world,
                    command_voltage_v: command_voltage_v[side],
                },
                dt_s,
            )?;
            states[index] = evaluation.state;
            if let Some(mut wrench) = evaluation.tire_wrench {
                wrench.entity = chassis;
                pending_wrenches.push(wrench);
            }
            world
                .entity_mut(rig.wheel_joints[index])
                .insert(rne_physics::JointState::Revolute {
                    position_rad: states[index].wheel_position_rad,
                    velocity_rad_s: states[index].wheel_velocity_rad_s,
                });
            world
                .entity_mut(rig.motor_entities[index])
                .insert(evaluation.motor_telemetry);
        }

        let step = zero_based_step + 1;
        let decision_time = SimTime::from_ticks(step * PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS);
        sample_frontends(&mut world, decision_time, &mut bus)?;
        match side_fusion.publish_latest(
            &mut bus,
            PHYSICAL_ENCODER_STREAMS,
            SIDE_ENCODER_STREAMS,
            decision_time,
        ) {
            Ok(false) | Err(WheelImuOdometryError::MissingAvailableFrame { .. }) => continue,
            Ok(true) => {}
            Err(error) => return Err(error.into()),
        }
        let Some(left) = bus.latest_available::<IncrementalEncoderFeedback>(
            SIDE_ENCODER_STREAMS.left,
            decision_time,
        ) else {
            continue;
        };
        if left.sequence <= last_left_sequence {
            continue;
        }
        let estimate = match estimator.update(
            &bus,
            WheelImuOdometryStreams {
                left_encoder: SIDE_ENCODER_STREAMS.left,
                right_encoder: SIDE_ENCODER_STREAMS.right,
                imu: IMU_STREAM,
            },
            decision_time,
        ) {
            Ok(estimate) => estimate,
            Err(
                WheelImuOdometryError::MissingAvailableFrame { .. }
                | WheelImuOdometryError::NoNewEncoderPair
                | WheelImuOdometryError::InputSkew { .. }
                | WheelImuOdometryError::StaleInput { .. },
            ) => continue,
            Err(error) => return Err(error.into()),
        };
        last_left_sequence = left.sequence;
        let target = if step > SETTLE_STEPS {
            TARGET_YAW_RATE_RAD_S
        } else {
            0.0
        };
        command_voltage_v = controller.update(
            estimate.angular_velocity_rad_s,
            target,
            SENSOR_PERIOD_TICKS as f64 / 1.0e9,
        );
        let encoder_sequences = [
            PHYSICAL_ENCODER_STREAMS.front_left,
            PHYSICAL_ENCODER_STREAMS.rear_left,
            PHYSICAL_ENCODER_STREAMS.front_right,
            PHYSICAL_ENCODER_STREAMS.rear_right,
        ]
        .map(|stream| {
            bus.latest_available::<IncrementalEncoderFeedback>(stream, decision_time)
                .expect("fused source")
                .sequence
        });
        let motor_frames = MOTOR_STREAMS.map(|stream| {
            bus.latest_available::<MotorElectricalFeedback>(stream, decision_time)
                .expect("motor feedback synchronized")
        });
        let measured_motor_current_a = motor_frames.each_ref().map(|frame| frame.payload.current_a);
        let motor_sequences = motor_frames.each_ref().map(|frame| frame.sequence);
        let motor_status_codes = motor_frames
            .each_ref()
            .map(|frame| motor_status_code(frame.payload.status));
        let imu_frame = bus
            .latest_available::<ImuFeedback>(IMU_STREAM, decision_time)
            .context("IMU feedback unavailable after estimator update")?;
        if target > 0.0 && estimate.health != WheelImuOdometryHealth::Initializing {
            squared_estimation_error +=
                (estimate.angular_velocity_rad_s - body.angular_velocity_rad_s.y).powi(2);
            squared_tracking_error += (body.angular_velocity_rad_s.y - target).powi(2);
            scoring_samples += 1;
        }
        samples.push(PerWheelObservedSample {
            step,
            decision_ticks: decision_time.ticks(),
            capture_ticks: estimate.provenance.capture_ticks,
            encoder_sequences,
            measured_motor_current_a,
            motor_sequences,
            motor_status_codes,
            imu_sequence: imu_frame.sequence,
            imu_status_code: imu_status_code(imu_frame.payload.status),
            estimated_yaw_rate_rad_s: estimate.angular_velocity_rad_s,
            yaw_innovation_rad: estimate.yaw_innovation_rad,
            health_code: health_code(estimate.health),
            target_yaw_rate_rad_s: target,
            command_voltage_v,
            privileged_integrated_yaw_rad: integrated_yaw_rad,
            privileged_yaw_rate_rad_s: body.angular_velocity_rad_s.y,
        });
    }
    ensure!(scoring_samples > 0, "no scoring samples");
    let final_sample = samples.last().context("no actor samples")?;
    let mut metrics = vec![
        metric(
            "final_estimated_yaw_rate_rad_s",
            "rad/s",
            final_sample.estimated_yaw_rate_rad_s,
            0.15,
            0.50,
        ),
        metric(
            "final_truth_yaw_rate_rad_s",
            "rad/s",
            final_sample.privileged_yaw_rate_rad_s,
            0.15,
            0.50,
        ),
        metric(
            "rms_yaw_rate_estimation_error_rad_s",
            "rad/s",
            (squared_estimation_error / scoring_samples as f64).sqrt(),
            0.0,
            0.25,
        ),
        metric(
            "rms_truth_tracking_error_rad_s",
            "rad/s",
            (squared_tracking_error / scoring_samples as f64).sqrt(),
            0.0,
            0.25,
        ),
        metric(
            "absolute_integrated_yaw_rad",
            "rad",
            final_sample.privileged_integrated_yaw_rad.abs(),
            0.3,
            3.0,
        ),
        metric(
            "maximum_measured_motor_current_a",
            "A",
            samples
                .iter()
                .flat_map(|sample| sample.measured_motor_current_a)
                .map(f64::abs)
                .fold(0.0, f64::max),
            0.5,
            25.0,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = PerWheelObservedTrace {
        kind: PER_WHEEL_OBSERVED_TRACE_KIND.to_string(),
        schema_version: PER_WHEEL_OBSERVED_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        contract,
        plant_spec: plant,
        wheel_station_specs: stations.map(|station| station.spec),
        contact_load_filter_time_constant_s: CONTACT_LOAD_FILTER_TIME_CONSTANT_S,
        fixed_delta_ticks: PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
        seed: WORLD_SEED,
        steps: TOTAL_STEPS,
        samples,
        passed: metrics.iter().all(|metric| metric.passed),
        metrics,
        content_digest: String::new(),
    };
    trace.content_digest = trace_digest(&trace)?;
    trace.validate()?;
    Ok(trace)
}

/// Builds a self-verifying SI-unit comparison from two complete backend traces.
pub fn compare_per_wheel_observed_traces(
    first: PerWheelObservedTrace,
    second: PerWheelObservedTrace,
) -> Result<PerWheelObservedComparison> {
    first.validate()?;
    second.validate()?;
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = PerWheelObservedComparison {
        kind: "rne_mobility_per_wheel_observed_comparison".to_string(),
        schema_version: PER_WHEEL_OBSERVED_SCHEMA_VERSION,
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

fn spawn_sensor_rig(
    world: &mut World,
    chassis: Entity,
    contract: &PerWheelObservedContract,
) -> SensorRig {
    let encoder_streams = [
        PHYSICAL_ENCODER_STREAMS.front_left,
        PHYSICAL_ENCODER_STREAMS.rear_left,
        PHYSICAL_ENCODER_STREAMS.front_right,
        PHYSICAL_ENCODER_STREAMS.rear_right,
    ];
    let names = ["front_left", "rear_left", "front_right", "rear_right"];
    let wheel_joints = std::array::from_fn(|index| {
        let fault = match (index, contract.fault) {
            (0, PerWheelObservedFault::FrontLeftEncoderDrop { sequence }) => {
                IncrementalEncoderFault::DropSequence { sequence }
            }
            (0, PerWheelObservedFault::FrontLeftEncoderStuck { sequence }) => {
                IncrementalEncoderFault::StuckFromSequence { sequence }
            }
            _ => IncrementalEncoderFault::None,
        };
        spawn_encoder_channel(
            world,
            chassis,
            names[index],
            encoder_streams[index],
            fault,
            contract,
        )
    });
    let imu = spawn_named(world, "per_wheel_observed_imu");
    world.entity_mut(imu).insert((
        ImuMount {
            body_entity: chassis,
            body_from_sensor: Transform3::from_translation_rotation(
                Vec3::new(0.1, 0.05, 0.0),
                Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            ),
        },
        ImuFeedbackSensor {
            spec: realistic_imu_spec(
                contract.calibrated_gyro_z_bias_rad_s,
                match contract.fault {
                    PerWheelObservedFault::ImuSaturate { gyro_range_rad_s } => {
                        Some(gyro_range_rad_s)
                    }
                    _ => None,
                },
            ),
            update_rate_hz: 100.0,
            sample_period_ticks: Some(contract.sensor_period_ticks),
            phase_offset_ticks: 0,
            latency_ticks: contract.sensor_latency_ticks,
            enabled: true,
            stream_id: IMU_STREAM,
            fault: match contract.fault {
                PerWheelObservedFault::ImuDrop { sequence } => {
                    ImuFeedbackFault::DropSequence { sequence }
                }
                PerWheelObservedFault::ImuStuck { sequence } => {
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
                    seed: 31 + index as u64,
                },
                update_rate_hz: 100.0,
                sample_period_ticks: Some(contract.sensor_period_ticks),
                phase_offset_ticks: 0,
                latency_ticks: contract.sensor_latency_ticks,
                enabled: true,
                stream_id: MOTOR_STREAMS[index],
                fault: match (index, contract.fault) {
                    (0, PerWheelObservedFault::FrontLeftMotorDrop { sequence }) => {
                        MotorElectricalFeedbackFault::DropSequence { sequence }
                    }
                    (0, PerWheelObservedFault::FrontLeftMotorStuck { sequence }) => {
                        MotorElectricalFeedbackFault::StuckFromSequence { sequence }
                    }
                    _ => MotorElectricalFeedbackFault::None,
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

fn spawn_encoder_channel(
    world: &mut World,
    chassis: Entity,
    name: &str,
    stream_id: StreamId,
    fault: IncrementalEncoderFault,
    contract: &PerWheelObservedContract,
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
            rne_physics::JointState::Revolute {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
            },
        ))
        .id();
    world.get_mut::<Joint>(joint).expect("joint").child_link = joint;
    let actuator = world
        .spawn(Actuator {
            robot: chassis,
            joint: Some(joint),
            name: format!("{name}_motor"),
            mode: ControlMode::Velocity,
            target: ActuatorTarget::default(),
            limits: ActuatorLimits::default(),
        })
        .id();
    let sensor = spawn_named(world, format!("{name}_encoder"));
    world.entity_mut(sensor).insert((
        IncrementalEncoderSensor {
            spec: IncrementalEncoderSpec {
                actuator,
                counts_per_revolution: contract.encoder_counts_per_revolution,
                direction: 1,
                zero_offset_rad: 0.0,
                counter_bits: match contract.fault {
                    PerWheelObservedFault::FrontLeftEncoderSaturate { counter_bits }
                        if name == "front_left" =>
                    {
                        counter_bits
                    }
                    _ => contract.encoder_counter_bits,
                },
                overflow_behavior: if matches!(
                    contract.fault,
                    PerWheelObservedFault::FrontLeftEncoderSaturate { .. }
                ) && name == "front_left"
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
            stream_id,
            fault,
        },
        IncrementalEncoderSensorState::default(),
    ));
    joint
}

fn realistic_imu_spec(calibrated_bias_rad_s: f64, gyro_range_rad_s: Option<f64>) -> ImuSpec {
    ImuSpec {
        seed: 29,
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
        gyro_range_rad_s: gyro_range_rad_s.unwrap_or(10.0),
        accel_range_m_s2: 40.0,
        gyro_resolution_rad_s: 0.000_1,
        accel_resolution_m_s2: 0.001,
        ..ImuSpec::default()
    }
}

fn odometry_config(contract: &PerWheelObservedContract) -> WheelImuOdometryConfig {
    WheelImuOdometryConfig {
        wheel_radius_m: wheel_plant_spec().wheel.radius_m,
        track_width_m: 0.6,
        left_counts_per_revolution: contract.encoder_counts_per_revolution * 2,
        right_counts_per_revolution: contract.encoder_counts_per_revolution * 2,
        left_counter_bits: FOUR_WHEEL_SIDE_FUSED_COUNTER_BITS,
        right_counter_bits: FOUR_WHEEL_SIDE_FUSED_COUNTER_BITS,
        left_direction: -1,
        right_direction: -1,
        gyro_z_direction: 1.0,
        max_abs_wheel_delta_counts: 20_000,
        gyro_z_bias_rad_s: contract.calibrated_gyro_z_bias_rad_s,
        wheel_distance_std_m: 0.001,
        encoder_yaw_std_rad: 0.002,
        gyro_rate_std_rad_s: 0.001,
        gyro_yaw_weight: 0.8,
        disagreement_gyro_yaw_weight: 1.0,
        disagreement_threshold_rad: 0.001,
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
struct YawRateController {
    kp_v_s_rad: f64,
    ki_v_rad: f64,
    integral_error_rad: f64,
}

impl YawRateController {
    fn new(contract: &PerWheelObservedContract) -> Self {
        Self {
            kp_v_s_rad: contract.controller_kp_v_s_rad,
            ki_v_rad: contract.controller_ki_v_rad,
            integral_error_rad: 0.0,
        }
    }

    fn update(&mut self, estimated_rad_s: f64, target_rad_s: f64, dt_s: f64) -> [f64; 2] {
        if target_rad_s == 0.0 {
            self.integral_error_rad = 0.0;
            return [0.0; 2];
        }
        let error = target_rad_s - estimated_rad_s;
        let candidate = (self.integral_error_rad + error * dt_s).clamp(-2.0, 2.0);
        let unconstrained = self.kp_v_s_rad * error + self.ki_v_rad * candidate;
        let drive = unconstrained.clamp(-MAXIMUM_VOLTAGE_V, MAXIMUM_VOLTAGE_V);
        if drive == unconstrained || error.signum() != unconstrained.signum() {
            self.integral_error_rad = candidate;
        }
        [drive, -drive]
    }
}

fn health_code(health: WheelImuOdometryHealth) -> u8 {
    match health {
        WheelImuOdometryHealth::Initializing => 0,
        WheelImuOdometryHealth::Nominal => 1,
        WheelImuOdometryHealth::InputSequenceGap => 2,
        WheelImuOdometryHealth::ImuSaturated => 3,
        WheelImuOdometryHealth::WheelImuDisagreement => 4,
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
        passed: value >= minimum && value <= maximum,
    }
}

fn validate_metrics(metrics: &[MobilityBenchmarkMetric], passed: bool) -> Result<()> {
    ensure!(!metrics.is_empty(), "metrics omitted");
    ensure!(
        metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
        "metric order"
    );
    ensure!(
        metrics.iter().all(|metric| metric.value.is_finite()
            && metric.passed == (metric.value >= metric.minimum && metric.value <= metric.maximum)),
        "metric drift"
    );
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "verdict drift"
    );
    Ok(())
}

fn trace_digest(trace: &PerWheelObservedTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in serde_json::to_vec(&canonical)? {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

fn comparison_metrics(
    first: &PerWheelObservedTrace,
    second: &PerWheelObservedTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    let mut metrics = vec![
        comparison_metric(
            "final_truth_yaw_rate_gap_rad_s",
            "rad/s",
            first,
            second,
            "final_truth_yaw_rate_rad_s",
            0.20,
        )?,
        comparison_metric(
            "integrated_yaw_gap_rad",
            "rad",
            first,
            second,
            "absolute_integrated_yaw_rad",
            0.50,
        )?,
        comparison_metric(
            "rms_estimation_error_gap_rad_s",
            "rad/s",
            first,
            second,
            "rms_yaw_rate_estimation_error_rad_s",
            0.15,
        )?,
        comparison_metric(
            "rms_tracking_error_gap_rad_s",
            "rad/s",
            first,
            second,
            "rms_truth_tracking_error_rad_s",
            0.15,
        )?,
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(metrics)
}

fn comparison_metric(
    id: &str,
    unit: &str,
    first: &PerWheelObservedTrace,
    second: &PerWheelObservedTrace,
    source_id: &str,
    maximum: f64,
) -> Result<MobilityBenchmarkMetric> {
    let value = (metric_value(first, source_id)? - metric_value(second, source_id)?).abs();
    Ok(metric(id, unit, value, 0.0, maximum))
}

fn metric_value(trace: &PerWheelObservedTrace, id: &str) -> Result<f64> {
    trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .map(|metric| metric.value)
        .with_context(|| format!("missing metric {id}"))
}

fn comparison_digest(comparison: &PerWheelObservedComparison) -> Result<String> {
    let mut canonical = comparison.clone();
    canonical.content_digest.clear();
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in serde_json::to_vec(&canonical)? {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn rapier_per_wheel_observed_trace_is_deterministic() {
        let first =
            run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let second =
            run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(first.passed, "{:#?}", first.metrics);
        assert_eq!(first, second);
        first.validate().unwrap();
    }

    #[test]
    fn actor_evidence_tampering_is_detected() {
        let mut trace =
            run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples[1].estimated_yaw_rate_rad_s += 1.0;
        assert!(trace.validate().is_err());
    }

    #[test]
    fn physical_encoder_drop_reaches_estimator_health_without_truth_access() {
        let trace = run_per_wheel_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            PerWheelObservedFault::FrontLeftEncoderDrop { sequence: 30 },
        )
        .unwrap();
        assert_eq!(
            trace.contract.fault,
            PerWheelObservedFault::FrontLeftEncoderDrop { sequence: 30 }
        );
        assert!(trace.samples.iter().any(|sample| sample.health_code == 2));
        assert!(trace
            .samples
            .windows(2)
            .any(|pair| { pair[1].encoder_sequences[0] > pair[0].encoder_sequences[0] + 1 }));
    }

    #[test]
    fn motor_drop_and_stuck_are_visible_without_changing_plant_truth() {
        let dropped = run_per_wheel_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            PerWheelObservedFault::FrontLeftMotorDrop { sequence: 30 },
        )
        .unwrap();
        assert!(dropped
            .samples
            .windows(2)
            .any(|pair| { pair[1].motor_sequences[0] > pair[0].motor_sequences[0] + 1 }));

        let stuck = run_per_wheel_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            PerWheelObservedFault::FrontLeftMotorStuck { sequence: 30 },
        )
        .unwrap();
        assert!(stuck
            .samples
            .iter()
            .any(|sample| sample.motor_status_codes[0] == 3));
    }

    #[test]
    fn imu_drop_and_saturation_reach_estimator_health() {
        let dropped = run_per_wheel_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            PerWheelObservedFault::ImuDrop { sequence: 30 },
        )
        .unwrap();
        assert!(dropped.samples.iter().any(|sample| sample.health_code == 2));
        assert!(dropped
            .samples
            .windows(2)
            .any(|pair| pair[1].imu_sequence > pair[0].imu_sequence + 1));

        let saturated = run_per_wheel_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            PerWheelObservedFault::ImuSaturate {
                gyro_range_rad_s: 0.05,
            },
        )
        .unwrap();
        assert!(saturated
            .samples
            .iter()
            .any(|sample| sample.imu_status_code == 1 && sample.health_code == 3));
    }

    #[test]
    fn stuck_and_saturated_motion_inputs_fail_closed() {
        for fault in [
            PerWheelObservedFault::FrontLeftEncoderStuck { sequence: 30 },
            PerWheelObservedFault::FrontLeftEncoderSaturate { counter_bits: 4 },
            PerWheelObservedFault::ImuStuck { sequence: 30 },
        ] {
            let error = run_per_wheel_observed_trace_with_fault(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )
            .unwrap_err();
            let message = error.to_string();
            assert!(
                message.contains("stuck") || message.contains("saturated"),
                "fault={fault:?}, error={message}"
            );
        }
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_per_wheel_observed_traces_pass() {
        use rne_physics_mujoco::MuJoCoBackend;
        let rapier =
            run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_per_wheel_observed_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(
                PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_per_wheel_observed_traces(rapier, mujoco).unwrap();
        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }
}
