//! Sensor-observed longitudinal control executed on interchangeable physics backends.

use anyhow::{ensure, Context, Result};
use rne_ai::{
    wheel_imu_sensor_only_task_spec, ActionSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec,
    TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind,
    TerminationSpec, WheelImuActorObservation, WheelImuOdometry, WheelImuOdometryConfig,
    WheelImuOdometryError, WheelImuOdometryHealth, WheelImuOdometryStreams,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, InMemoryDataBus, IncrementalEncoderFeedback, MotorElectricalFeedback, PoseSample,
    StreamId,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, Collider, JointState, PhysicsBackend, PhysicsBackendManifest,
    PhysicsCapability, PhysicsMaterial, PhysicsWorldDesc, RigidBody, RigidBodyInertia,
    RigidBodyType,
};
use rne_robot::{
    aggregate_wheel_contact_patch, evaluate_longitudinal_drive_path, Actuator, ActuatorLimits,
    ActuatorTarget, CombinedSlipTireSpec, ControlMode, DcMotorCompletedTelemetry, DcMotorSpec,
    Joint, JointKind, JointLimits, LongitudinalDrivePathInput, LongitudinalDrivePathState,
    LongitudinalMobilityPlantSpec, TransmissionSpec, WheelAssemblySpec,
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

use crate::MobilityBenchmarkMetric;

/// Artifact discriminator for one sensor-observed backend trace.
pub const SENSOR_OBSERVED_TRACE_KIND: &str = "rne_mobility_sensor_observed_trace";
/// Current sensor-observed trace schema version.
pub const SENSOR_OBSERVED_TRACE_SCHEMA_VERSION: u32 = 1;
/// Artifact discriminator for a sensor-observed cross-backend comparison.
pub const SENSOR_OBSERVED_COMPARISON_KIND: &str = "rne_mobility_sensor_observed_comparison";
/// Current sensor-observed comparison schema version.
pub const SENSOR_OBSERVED_COMPARISON_SCHEMA_VERSION: u32 = 1;
/// Stable task identity for the longitudinal sensor-only controller.
pub const SENSOR_OBSERVED_TASK_ID: &str = "mobility_longitudinal_sensor_observed_v1";
/// One millisecond physics integration step in simulation ticks.
pub const SENSOR_OBSERVED_FIXED_DELTA_TICKS: u64 = 1_000_000;

const SENSOR_PERIOD_TICKS: u64 = 10_000_000;
const SENSOR_LATENCY_TICKS: u64 = 2_000_000;
const SETTLE_STEPS: u64 = 300;
const DRIVE_STEPS: u64 = 3_000;
const TOTAL_STEPS: u64 = SETTLE_STEPS + DRIVE_STEPS;
const TARGET_VELOCITY_M_S: f64 = 1.0;
const MAXIMUM_VOLTAGE_V: f64 = 24.0;
const WORLD_SEED: u64 = 0;
const ENCODER_COUNTS_PER_REVOLUTION: u32 = 2_048;
const LEFT_ENCODER_STREAM: StreamId = StreamId::new(1_001);
const RIGHT_ENCODER_STREAM: StreamId = StreamId::new(1_002);
const IMU_STREAM: StreamId = StreamId::new(1_003);
const MOTOR_STREAM: StreamId = StreamId::new(1_004);

/// Fully explicit sensor, estimator, and controller contract retained with evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorObservedContract {
    /// Decoded quadrature counts per wheel revolution.
    pub encoder_counts_per_revolution: u32,
    /// Signed hardware counter width.
    pub encoder_counter_bits: u8,
    /// Exact encoder, IMU, and motor-feedback capture period.
    pub sensor_period_ticks: u64,
    /// Capture-to-availability delay shared by the three frontends.
    pub sensor_latency_ticks: u64,
    /// Gyroscope turn-on bias removed by estimator calibration, in rad/s.
    pub calibrated_gyro_z_bias_rad_s: f64,
    /// PI proportional gain in volts per `(m/s)`.
    pub controller_kp_v_s_m: f64,
    /// PI integral gain in volts per meter.
    pub controller_ki_v_m: f64,
    /// Optional deterministic left-encoder sequence drop.
    pub left_encoder_drop_sequence: Option<u64>,
}

impl SensorObservedContract {
    fn nominal() -> Self {
        Self {
            encoder_counts_per_revolution: ENCODER_COUNTS_PER_REVOLUTION,
            encoder_counter_bits: 32,
            sensor_period_ticks: SENSOR_PERIOD_TICKS,
            sensor_latency_ticks: SENSOR_LATENCY_TICKS,
            calibrated_gyro_z_bias_rad_s: 0.001,
            controller_kp_v_s_m: 15.0,
            controller_ki_v_m: 20.0,
            left_encoder_drop_sequence: None,
        }
    }

    fn validate(&self) -> Result<()> {
        let mut expected = Self::nominal();
        expected.left_encoder_drop_sequence = self.left_encoder_drop_sequence;
        ensure!(*self == expected, "sensor/controller contract drift");
        if let Some(sequence) = self.left_encoder_drop_sequence {
            ensure!(sequence > 1, "invalid encoder drop sequence");
        }
        Ok(())
    }
}

/// One actor decision plus separately labeled privileged scoring values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorObservedSample {
    /// Completed physics step at the controller decision.
    pub step: u64,
    /// Estimator decision time in simulation ticks.
    pub decision_ticks: u64,
    /// Newest synchronized sensor capture time in ticks.
    pub capture_ticks: u64,
    /// Maximum age of synchronized inputs at the decision, in ticks.
    pub maximum_input_age_ticks: u64,
    /// Left incremental-encoder frame sequence.
    pub left_encoder_sequence: u64,
    /// Right incremental-encoder frame sequence.
    pub right_encoder_sequence: u64,
    /// IMU frame sequence.
    pub imu_sequence: u64,
    /// Motor electrical-feedback frame sequence.
    pub motor_sequence: u64,
    /// Sensor-only estimated planar position, in meters.
    pub estimated_position_m: [f64; 2],
    /// Sensor-only estimated yaw, in radians.
    pub estimated_yaw_rad: f64,
    /// Sensor-only estimated longitudinal speed, in meters per second.
    pub estimated_velocity_m_s: f64,
    /// Estimated yaw rate, in radians per second.
    pub estimated_yaw_rate_rad_s: f64,
    /// Estimated x-position variance, in square meters.
    pub estimated_position_variance_m2: f64,
    /// Wheel/IMU yaw innovation, in radians.
    pub yaw_innovation_rad: f64,
    /// Stable sensor-estimator health code.
    pub health_code: u8,
    /// Number of skipped source sequences detected by this update.
    pub skipped_sequences: u64,
    /// Measured motor terminal voltage, in volts.
    pub measured_motor_voltage_v: f64,
    /// Measured motor current, in amperes.
    pub measured_motor_current_a: f64,
    /// Task-owned velocity target, in meters per second.
    pub target_velocity_m_s: f64,
    /// Bounded voltage action computed only from this actor observation.
    pub command_voltage_v: f64,
    /// Privileged chassis displacement used only for scoring, in meters.
    pub privileged_forward_distance_m: f64,
    /// Privileged chassis speed used only for scoring, in meters per second.
    pub privileged_forward_velocity_m_s: f64,
}

impl SensorObservedSample {
    fn is_finite(&self) -> bool {
        self.estimated_position_m
            .iter()
            .chain([
                &self.estimated_yaw_rad,
                &self.estimated_velocity_m_s,
                &self.estimated_yaw_rate_rad_s,
                &self.estimated_position_variance_m2,
                &self.yaw_innovation_rad,
                &self.measured_motor_voltage_v,
                &self.measured_motor_current_a,
                &self.target_velocity_m_s,
                &self.command_voltage_v,
                &self.privileged_forward_distance_m,
                &self.privileged_forward_velocity_m_s,
            ])
            .all(|value| value.is_finite())
    }
}

/// Deterministic sensor-only control trace from one rigid-body backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorObservedTrace {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Trace schema version.
    pub schema_version: u32,
    /// Exact backend identity and capabilities.
    pub backend: PhysicsBackendManifest,
    /// Exact actor observation and action contract.
    pub task_spec: TaskSpec,
    /// Sensor, estimator, and controller configuration.
    pub contract: SensorObservedContract,
    /// Physics integration step in ticks.
    pub fixed_delta_ticks: u64,
    /// Explicit deterministic world seed.
    pub seed: u64,
    /// Number of completed physics steps.
    pub steps: u64,
    /// Every accepted estimator update in decision order.
    pub samples: Vec<SensorObservedSample>,
    /// Ordered unit-bearing acceptance metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every acceptance metric passed.
    pub passed: bool,
    /// FNV-1a digest of the same trace with this field empty.
    pub content_digest: String,
}

impl SensorObservedTrace {
    /// Recomputes actor isolation, timing, ordering, verdict, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == SENSOR_OBSERVED_TRACE_KIND, "kind mismatch");
        ensure!(
            self.schema_version == SENSOR_OBSERVED_TRACE_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.backend.validate().context("backend manifest")?;
        self.task_spec.validate().context("TaskSpec")?;
        ensure!(
            self.task_spec == sensor_observed_task_spec(),
            "exact TaskSpec mismatch"
        );
        ensure!(
            self.task_spec.observation.tensors.iter().all(|tensor| {
                !tensor.name.contains("truth") && !tensor.name.contains("privileged")
            }),
            "actor observation exposes truth"
        );
        self.contract.validate()?;
        ensure!(
            self.fixed_delta_ticks == SENSOR_OBSERVED_FIXED_DELTA_TICKS,
            "fixed-step mismatch"
        );
        ensure!(self.seed == WORLD_SEED, "seed mismatch");
        ensure!(self.steps == TOTAL_STEPS, "step-count mismatch");
        ensure!(self.samples.len() > 100, "trace omitted actor decisions");
        ensure!(
            self.samples
                .windows(2)
                .all(|pair| pair[0].decision_ticks < pair[1].decision_ticks
                    && pair[0].left_encoder_sequence < pair[1].left_encoder_sequence
                    && pair[0].right_encoder_sequence < pair[1].right_encoder_sequence
                    && pair[0].imu_sequence < pair[1].imu_sequence
                    && pair[0].motor_sequence < pair[1].motor_sequence),
            "sensor decisions are not strictly ordered"
        );
        for sample in &self.samples {
            ensure!(sample.is_finite(), "sample {} is non-finite", sample.step);
            ensure!(
                sample.decision_ticks == sample.step * self.fixed_delta_ticks,
                "sample {} decision timestamp mismatch",
                sample.step
            );
            ensure!(
                sample.capture_ticks <= sample.decision_ticks
                    && sample.maximum_input_age_ticks
                        == sample.decision_ticks - sample.capture_ticks
                    && sample.maximum_input_age_ticks <= self.contract.sensor_latency_ticks,
                "sample {} violates sensor availability timing",
                sample.step
            );
            let expected_target =
                if sample.decision_ticks >= SETTLE_STEPS * SENSOR_OBSERVED_FIXED_DELTA_TICKS {
                    TARGET_VELOCITY_M_S
                } else {
                    0.0
                };
            ensure!(
                sample.target_velocity_m_s == expected_target,
                "sample {} target schedule mismatch",
                sample.step
            );
            ensure!(
                (-MAXIMUM_VOLTAGE_V..=MAXIMUM_VOLTAGE_V).contains(&sample.command_voltage_v),
                "sample {} action escaped voltage bounds",
                sample.step
            );
            ensure!(
                sample.estimated_position_variance_m2 >= 0.0 && sample.health_code <= 4,
                "sample {} estimator evidence invalid",
                sample.step
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

/// Two complete sensor-observed traces with explicit SI-unit tolerances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorObservedComparison {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Comparison schema version.
    pub schema_version: u32,
    /// First backend trace.
    pub first: SensorObservedTrace,
    /// Second backend trace.
    pub second: SensorObservedTrace,
    /// Ordered absolute cross-backend gaps.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every gap remains within tolerance.
    pub passed: bool,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl SensorObservedComparison {
    /// Recomputes trace integrity, shared contracts, gap metrics, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == SENSOR_OBSERVED_COMPARISON_KIND,
            "kind mismatch"
        );
        ensure!(
            self.schema_version == SENSOR_OBSERVED_COMPARISON_SCHEMA_VERSION,
            "schema mismatch"
        );
        self.first.validate().context("first trace")?;
        self.second.validate().context("second trace")?;
        ensure!(
            self.first.backend.backend_id != self.second.backend.backend_id,
            "comparison requires distinct backends"
        );
        ensure!(
            self.first.task_spec == self.second.task_spec
                && self.first.contract == self.second.contract
                && self.first.fixed_delta_ticks == self.second.fixed_delta_ticks
                && self.first.seed == self.second.seed,
            "backend execution contract mismatch"
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

/// Returns the exact actor contract used by sensor-observed backend runs.
pub fn sensor_observed_task_spec() -> TaskSpec {
    let sensor_only = wheel_imu_sensor_only_task_spec(TOTAL_STEPS / 10, MAXIMUM_VOLTAGE_V);
    let mut observation = sensor_only.observation;
    observation.tensors.push(TensorSpec::new(
        "measured_motor_voltage_v",
        TensorDType::F64,
        vec![],
        "V",
    ));
    observation.tensors.push(TensorSpec::new(
        "measured_motor_current_a",
        TensorDType::F64,
        vec![],
        "A",
    ));
    observation.tensors.push(
        TensorSpec::new("target_velocity_m_s", TensorDType::F64, vec![], "m/s")
            .with_bounds(TensorBounds::broadcast(0.0, TARGET_VELOCITY_M_S)),
    );
    TaskSpec::new(
        SENSOR_OBSERVED_TASK_ID,
        SENSOR_PERIOD_TICKS as f64 / 1_000_000_000.0,
        observation,
        ActionSpec::new(vec![TensorSpec::new(
            "motor_terminal_voltage_v",
            TensorDType::F64,
            vec![1],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(
            -MAXIMUM_VOLTAGE_V,
            MAXIMUM_VOLTAGE_V,
        ))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_velocity_tracking_error_m_s", -1.0, "m/s"),
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

/// Runs realistic sensor frontends, sensor-only estimation, and feedback control on one backend.
pub fn run_sensor_observed_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
) -> Result<SensorObservedTrace> {
    run_sensor_observed_trace_with_fault(backend, manifest, None)
}

fn run_sensor_observed_trace_with_fault<B: PhysicsBackend>(
    mut backend: B,
    manifest: PhysicsBackendManifest,
    left_encoder_drop_sequence: Option<u64>,
) -> Result<SensorObservedTrace> {
    manifest.validate().context("backend manifest")?;
    require_capabilities(
        backend.capabilities(),
        &[
            PhysicsCapability::RigidBody,
            PhysicsCapability::ContactForce,
            PhysicsCapability::ExternalBodyWrench,
            PhysicsCapability::ContactPointKinematics,
        ],
    )?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "backend manifest capability drift"
    );
    let task_spec = sensor_observed_task_spec();
    task_spec.validate().context("TaskSpec")?;
    let mut contract = SensorObservedContract::nominal();
    contract.left_encoder_drop_sequence = left_encoder_drop_sequence;
    contract.validate()?;

    let fixed_delta = SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS);
    let dt_s = fixed_delta.as_seconds().value();
    let physics_world = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -9.806_65, 0.0),
        solver_iterations: 16,
    })?;
    let mut world = World::new();
    world.insert_resource(WorldRandom::new(WORLD_SEED));
    let ground = spawn_named(&mut world, "sensor_observed_ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        frictionless_collider(Vec3::new(20.0, 0.5, 5.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let vehicle = spawn_named(&mut world, "sensor_observed_vehicle");
    world.entity_mut(vehicle).insert((
        RigidBody {
            mass_kg: 100.0,
            ..RigidBody::default()
        },
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: 5.083_333_333_333_333,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 11.333_333_333_334,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 10.416_666_666_666_666,
        },
        frictionless_collider(Vec3::new(0.5, 0.25, 0.3)),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.251, 0.0), Quat::IDENTITY),
    ));
    let rig = spawn_sensor_rig(&mut world, vehicle, &contract);
    backend.sync_from_ecs(&mut world, physics_world)?;

    let plant = plant_spec();
    let mut estimator = WheelImuOdometry::new(
        odometry_config(&contract, plant.wheel.radius_m),
        PoseSample::default(),
    )?;
    let mut bus = InMemoryDataBus::new();
    sample_frontends(&mut world, SimTime::ZERO, &mut bus)?;

    let initial_position = *world
        .get::<Transform3>(vehicle)
        .context("initial vehicle transform")?;
    let mut drive_state = LongitudinalDrivePathState::default();
    let mut pending_wrench = None;
    let mut command_voltage_v = 0.0;
    let mut controller = VelocityController::new(&contract);
    let mut samples = Vec::new();
    let mut contact_drive_steps = 0_u64;
    let mut squared_position_error_sum_m2 = 0.0;
    let mut squared_velocity_error_sum_m2_s2 = 0.0;
    let mut scoring_samples = 0_u64;
    let mut maximum_measured_current_a = 0.0_f64;
    let mut last_estimator_left_sequence = 0_u64;

    for zero_based_step in 0..TOTAL_STEPS {
        if let Some(wrench) = pending_wrench.take() {
            backend.apply_external_body_wrench(physics_world, wrench)?;
        }
        backend.step(physics_world, fixed_delta)?;
        backend.sync_to_ecs(&mut world, physics_world)?;

        let carrier_patch = aggregate_wheel_contact_patch(
            vehicle,
            backend.contact_points(physics_world)?,
            Vec3::X,
            Vec3::Z,
        )?;
        if zero_based_step >= SETTLE_STEPS && carrier_patch.is_some() {
            contact_drive_steps += 1;
        }
        let drive = evaluate_longitudinal_drive_path(
            plant,
            drive_state,
            LongitudinalDrivePathInput {
                carrier_patch,
                forward_world: Vec3::X,
                lateral_world: Vec3::Z,
                command_voltage_v,
            },
            dt_s,
        )?;
        drive_state = drive.state;
        pending_wrench = drive.tire_wrench;
        for joint in rig.wheel_joints {
            world.entity_mut(joint).insert(JointState::Revolute {
                position_rad: drive_state.wheel_position_rad,
                velocity_rad_s: drive_state.wheel_velocity_rad_s,
            });
        }
        world
            .entity_mut(rig.motor_entity)
            .insert(drive.motor_telemetry);

        let step = zero_based_step + 1;
        let decision_time = SimTime::from_ticks(step * SENSOR_OBSERVED_FIXED_DELTA_TICKS);
        sample_frontends(&mut world, decision_time, &mut bus)?;
        let Some(left_frame) =
            bus.latest_available::<IncrementalEncoderFeedback>(LEFT_ENCODER_STREAM, decision_time)
        else {
            continue;
        };
        if left_frame.sequence <= last_estimator_left_sequence {
            continue;
        }
        let estimate = match estimator.update(&bus, rig.streams, decision_time) {
            Ok(estimate) => {
                last_estimator_left_sequence = left_frame.sequence;
                estimate
            }
            Err(
                WheelImuOdometryError::MissingAvailableFrame { .. }
                | WheelImuOdometryError::NoNewEncoderPair,
            ) => continue,
            Err(error) => return Err(error.into()),
        };
        let motor = bus
            .latest_available::<MotorElectricalFeedback>(MOTOR_STREAM, decision_time)
            .context("motor feedback unavailable at estimator decision")?;
        let actor = WheelImuActorObservation::from_estimate(estimate, [0.0, 0.0]);
        let target_velocity_m_s =
            if decision_time.ticks() >= SETTLE_STEPS * SENSOR_OBSERVED_FIXED_DELTA_TICKS {
                TARGET_VELOCITY_M_S
            } else {
                0.0
            };
        command_voltage_v = controller.update(
            actor.estimated_linear_velocity_m_s,
            target_velocity_m_s,
            SENSOR_PERIOD_TICKS as f64 / 1_000_000_000.0,
        );
        let transform = *world
            .get::<Transform3>(vehicle)
            .context("vehicle transform at actor decision")?;
        let body = *world
            .get::<RigidBody>(vehicle)
            .context("vehicle body at actor decision")?;
        let truth_distance_m = transform.translation.x - initial_position.translation.x;
        if target_velocity_m_s > 0.0 && estimate.health != WheelImuOdometryHealth::Initializing {
            squared_position_error_sum_m2 +=
                (actor.estimated_position_m[0] - truth_distance_m).powi(2);
            squared_velocity_error_sum_m2_s2 +=
                (actor.estimated_linear_velocity_m_s - body.linear_velocity_m_s.x).powi(2);
            scoring_samples += 1;
        }
        maximum_measured_current_a = maximum_measured_current_a.max(motor.payload.current_a.abs());
        samples.push(SensorObservedSample {
            step,
            decision_ticks: decision_time.ticks(),
            capture_ticks: estimate.provenance.capture_ticks,
            maximum_input_age_ticks: estimate.provenance.max_age_ticks,
            left_encoder_sequence: estimate.provenance.left_sequence,
            right_encoder_sequence: estimate.provenance.right_sequence,
            imu_sequence: estimate.provenance.imu_sequence,
            motor_sequence: motor.sequence,
            estimated_position_m: actor.estimated_position_m,
            estimated_yaw_rad: actor.estimated_yaw_rad,
            estimated_velocity_m_s: actor.estimated_linear_velocity_m_s,
            estimated_yaw_rate_rad_s: actor.estimated_angular_velocity_rad_s,
            estimated_position_variance_m2: actor.position_covariance_m2[0][0],
            yaw_innovation_rad: actor.yaw_innovation_rad,
            health_code: actor.health_code,
            skipped_sequences: actor.skipped_sequences,
            measured_motor_voltage_v: motor.payload.terminal_voltage_v,
            measured_motor_current_a: motor.payload.current_a,
            target_velocity_m_s,
            command_voltage_v,
            privileged_forward_distance_m: truth_distance_m,
            privileged_forward_velocity_m_s: body.linear_velocity_m_s.x,
        });
    }

    ensure!(
        scoring_samples > 0,
        "sensor-observed run omitted scoring samples"
    );
    let final_transform = *world
        .get::<Transform3>(vehicle)
        .context("final vehicle transform")?;
    let final_body = *world
        .get::<RigidBody>(vehicle)
        .context("final vehicle body")?;
    let final_sample = samples.last().context("trace omitted final actor sample")?;
    let nominal_samples = samples
        .iter()
        .filter(|sample| sample.health_code == 1)
        .count();
    let mut metrics = vec![
        metric(
            "contact_drive_fraction",
            "1",
            contact_drive_steps as f64 / DRIVE_STEPS as f64,
            0.95,
            1.0,
        ),
        metric(
            "final_estimated_distance_m",
            "m",
            final_sample.estimated_position_m[0],
            1.0,
            5.0,
        ),
        metric(
            "final_estimated_velocity_m_s",
            "m/s",
            final_sample.estimated_velocity_m_s,
            0.85,
            1.15,
        ),
        metric(
            "final_truth_distance_m",
            "m",
            final_transform.translation.x - initial_position.translation.x,
            1.0,
            5.0,
        ),
        metric(
            "final_truth_velocity_m_s",
            "m/s",
            final_body.linear_velocity_m_s.x,
            0.9,
            1.1,
        ),
        metric(
            "final_truth_velocity_tracking_error_m_s",
            "m/s",
            (final_body.linear_velocity_m_s.x - TARGET_VELOCITY_M_S).abs(),
            0.0,
            0.1,
        ),
        metric(
            "maximum_input_age_s",
            "s",
            samples
                .iter()
                .map(|sample| sample.maximum_input_age_ticks)
                .max()
                .unwrap_or_default() as f64
                / 1_000_000_000.0,
            SENSOR_LATENCY_TICKS as f64 / 1_000_000_000.0,
            SENSOR_LATENCY_TICKS as f64 / 1_000_000_000.0,
        ),
        metric(
            "maximum_measured_current_a",
            "A",
            maximum_measured_current_a,
            1.0,
            25.0,
        ),
        metric(
            "nominal_health_fraction",
            "1",
            nominal_samples as f64 / samples.len() as f64,
            0.95,
            1.0,
        ),
        metric(
            "rms_position_estimation_error_m",
            "m",
            (squared_position_error_sum_m2 / scoring_samples as f64).sqrt(),
            0.0,
            0.25,
        ),
        metric(
            "rms_velocity_estimation_error_m_s",
            "m/s",
            (squared_velocity_error_sum_m2_s2 / scoring_samples as f64).sqrt(),
            0.0,
            0.25,
        ),
    ];
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let mut trace = SensorObservedTrace {
        kind: SENSOR_OBSERVED_TRACE_KIND.to_string(),
        schema_version: SENSOR_OBSERVED_TRACE_SCHEMA_VERSION,
        backend: manifest,
        task_spec,
        contract,
        fixed_delta_ticks: SENSOR_OBSERVED_FIXED_DELTA_TICKS,
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

/// Builds and verifies a unit-aware sensor-observed backend comparison.
pub fn compare_sensor_observed_traces(
    first: SensorObservedTrace,
    second: SensorObservedTrace,
) -> Result<SensorObservedComparison> {
    first.validate().context("first trace")?;
    second.validate().context("second trace")?;
    let metrics = comparison_metrics(&first, &second)?;
    let mut comparison = SensorObservedComparison {
        kind: SENSOR_OBSERVED_COMPARISON_KIND.to_string(),
        schema_version: SENSOR_OBSERVED_COMPARISON_SCHEMA_VERSION,
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

#[derive(Clone, Copy, Debug)]
struct SensorRig {
    wheel_joints: [Entity; 2],
    motor_entity: Entity,
    streams: WheelImuOdometryStreams,
}

fn spawn_sensor_rig(
    world: &mut World,
    vehicle: Entity,
    contract: &SensorObservedContract,
) -> SensorRig {
    let left_joint = spawn_encoder_channel(
        world,
        vehicle,
        "left_wheel",
        LEFT_ENCODER_STREAM,
        contract
            .left_encoder_drop_sequence
            .map_or(IncrementalEncoderFault::None, |sequence| {
                IncrementalEncoderFault::DropSequence { sequence }
            }),
        contract,
    );
    let right_joint = spawn_encoder_channel(
        world,
        vehicle,
        "right_wheel",
        RIGHT_ENCODER_STREAM,
        IncrementalEncoderFault::None,
        contract,
    );
    let imu = spawn_named(world, "sensor_observed_imu");
    world.entity_mut(imu).insert((
        ImuMount {
            body_entity: vehicle,
            // The estimator consumes sensor Z as yaw; RNE rigid bodies are Y-up.
            body_from_sensor: Transform3::from_translation_rotation(
                Vec3::new(0.1, 0.05, 0.0),
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
            stream_id: IMU_STREAM,
            fault: ImuFeedbackFault::None,
        },
        ImuFeedbackSensorState::default(),
    ));
    let motor_entity = world.spawn(DcMotorCompletedTelemetry::default()).id();
    let motor_sensor = spawn_named(world, "sensor_observed_motor_feedback");
    world.entity_mut(motor_sensor).insert((
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
                seed: 23,
            },
            update_rate_hz: 100.0,
            sample_period_ticks: Some(contract.sensor_period_ticks),
            phase_offset_ticks: 0,
            latency_ticks: contract.sensor_latency_ticks,
            enabled: true,
            stream_id: MOTOR_STREAM,
            fault: MotorElectricalFeedbackFault::None,
        },
        MotorElectricalFeedbackSensorState::default(),
    ));
    SensorRig {
        wheel_joints: [left_joint, right_joint],
        motor_entity,
        streams: WheelImuOdometryStreams {
            left_encoder: LEFT_ENCODER_STREAM,
            right_encoder: RIGHT_ENCODER_STREAM,
            imu: IMU_STREAM,
        },
    }
}

fn spawn_encoder_channel(
    world: &mut World,
    vehicle: Entity,
    name: &str,
    stream_id: StreamId,
    fault: IncrementalEncoderFault,
    contract: &SensorObservedContract,
) -> Entity {
    let joint = world
        .spawn((
            Joint {
                robot: vehicle,
                parent_link: vehicle,
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
    world.get_mut::<Joint>(joint).unwrap().child_link = joint;
    let actuator = world
        .spawn(Actuator {
            robot: vehicle,
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
                counter_bits: contract.encoder_counter_bits,
                overflow_behavior: IncrementalEncoderOverflowBehavior::Wrap,
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

fn realistic_imu_spec(calibrated_bias_rad_s: f64) -> ImuSpec {
    ImuSpec {
        seed: 17,
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

fn odometry_config(
    contract: &SensorObservedContract,
    wheel_radius_m: f64,
) -> WheelImuOdometryConfig {
    WheelImuOdometryConfig {
        wheel_radius_m,
        track_width_m: 0.5,
        left_counts_per_revolution: contract.encoder_counts_per_revolution,
        right_counts_per_revolution: contract.encoder_counts_per_revolution,
        left_counter_bits: contract.encoder_counter_bits,
        right_counter_bits: contract.encoder_counter_bits,
        left_direction: 1,
        right_direction: 1,
        gyro_z_direction: 1.0,
        max_abs_wheel_delta_counts: 10_000,
        gyro_z_bias_rad_s: contract.calibrated_gyro_z_bias_rad_s,
        wheel_distance_std_m: 0.000_5,
        encoder_yaw_std_rad: 0.000_5,
        gyro_rate_std_rad_s: 0.001,
        gyro_yaw_weight: 0.2,
        disagreement_gyro_yaw_weight: 0.8,
        disagreement_threshold_rad: 0.02,
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
struct VelocityController {
    kp_v_s_m: f64,
    ki_v_m: f64,
    integral_error_m: f64,
}

impl VelocityController {
    fn new(contract: &SensorObservedContract) -> Self {
        Self {
            kp_v_s_m: contract.controller_kp_v_s_m,
            ki_v_m: contract.controller_ki_v_m,
            integral_error_m: 0.0,
        }
    }

    fn update(&mut self, estimated_velocity_m_s: f64, target_m_s: f64, dt_s: f64) -> f64 {
        if target_m_s == 0.0 {
            self.integral_error_m = 0.0;
            return 0.0;
        }
        let error_m_s = target_m_s - estimated_velocity_m_s;
        let candidate_integral_m = (self.integral_error_m + error_m_s * dt_s).clamp(-2.0, 2.0);
        let unconstrained_v = self.kp_v_s_m * error_m_s + self.ki_v_m * candidate_integral_m;
        let constrained_v = unconstrained_v.clamp(-MAXIMUM_VOLTAGE_V, MAXIMUM_VOLTAGE_V);
        if unconstrained_v == constrained_v || error_m_s.signum() != unconstrained_v.signum() {
            self.integral_error_m = candidate_integral_m;
        }
        constrained_v
    }
}

fn plant_spec() -> LongitudinalMobilityPlantSpec {
    let static_load_n = 100.0 * 9.806_65;
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: 100.0,
        driven_wheel_count: 1,
        normal_load_per_driven_wheel_n: static_load_n,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.0,
        road_friction_scale: 1.0,
        motor: DcMotorSpec::default(),
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec::default(),
        tire: CombinedSlipTireSpec {
            reference_load_n: static_load_n,
            ..CombinedSlipTireSpec::default()
        },
    }
}

fn frictionless_collider(half_extents_m: Vec3) -> Collider {
    let mut collider = Collider::cuboid(half_extents_m);
    collider.material = PhysicsMaterial {
        friction: 0.0,
        restitution: 0.0,
    };
    collider
}

fn comparison_metrics(
    first: &SensorObservedTrace,
    second: &SensorObservedTrace,
) -> Result<Vec<MobilityBenchmarkMetric>> {
    [
        ("final_estimated_distance_m", "m", 0.10),
        ("final_estimated_velocity_m_s", "m/s", 0.10),
        ("final_truth_distance_m", "m", 0.10),
        ("final_truth_velocity_m_s", "m/s", 0.10),
        ("final_truth_velocity_tracking_error_m_s", "m/s", 0.01),
        ("rms_position_estimation_error_m", "m", 0.05),
        ("rms_velocity_estimation_error_m_s", "m/s", 0.05),
    ]
    .into_iter()
    .map(|(id, unit, maximum)| {
        Ok(metric(
            &format!("absolute_{id}_gap"),
            unit,
            (metric_value(first, id, unit)? - metric_value(second, id, unit)?).abs(),
            0.0,
            maximum,
        ))
    })
    .collect()
}

fn metric_value(trace: &SensorObservedTrace, id: &str, unit: &str) -> Result<f64> {
    let metric = trace
        .metrics
        .iter()
        .find(|metric| metric.id == id)
        .with_context(|| format!("trace omitted metric {id}"))?;
    ensure!(metric.unit == unit, "metric {id} unit mismatch");
    Ok(metric.value)
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
        "metrics are not strictly sorted"
    );
    for metric in metrics {
        ensure!(
            metric.value.is_finite()
                && metric.minimum.is_finite()
                && metric.maximum.is_finite()
                && metric.minimum <= metric.maximum,
            "metric {} is invalid",
            metric.id
        );
        ensure!(
            metric.passed == (metric.value >= metric.minimum && metric.value <= metric.maximum),
            "metric {} verdict mismatch",
            metric.id
        );
    }
    ensure!(
        passed == metrics.iter().all(|metric| metric.passed),
        "aggregate verdict mismatch"
    );
    Ok(())
}

fn trace_digest(trace: &SensorObservedTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn comparison_digest(comparison: &SensorObservedComparison) -> Result<String> {
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
    fn rapier_sensor_observed_trace_is_exactly_repeatable() {
        let first = run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())
            .expect("first trace");
        let second = run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())
            .expect("second trace");

        assert!(first.passed, "{:#?}", first.metrics);
        assert_eq!(first, second);
        first.validate().unwrap();
        assert!(first
            .task_spec
            .observation
            .tensors
            .iter()
            .all(|tensor| !tensor.name.contains("truth")));
    }

    #[test]
    fn one_encoder_drop_is_visible_to_the_estimator_and_controller() {
        let trace = run_sensor_observed_trace_with_fault(
            RapierBackend::new(),
            RapierBackend::manifest(),
            Some(120),
        )
        .expect("dropout trace");

        trace.validate().unwrap();
        assert!(trace
            .samples
            .iter()
            .any(|sample| sample.health_code == 2 && sample.skipped_sequences > 0));
    }

    #[test]
    fn trace_tampering_is_detected() {
        let mut trace =
            run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        trace.samples.last_mut().unwrap().estimated_velocity_m_s += 0.5;
        assert!(trace.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_sensor_observed_loops_stay_within_tolerance() {
        use rne_physics_mujoco::MuJoCoBackend;

        let rapier =
            run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        let mujoco = run_sensor_observed_trace(
            MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                .expect("MuJoCo runtime"),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        let comparison = compare_sensor_observed_traces(rapier, mujoco).unwrap();

        assert!(comparison.passed, "{:#?}", comparison.metrics);
        comparison.validate().unwrap();
    }
}
