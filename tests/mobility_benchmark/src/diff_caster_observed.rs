//! Sensor-only closed-loop execution over the explicit differential-caster plant.

use anyhow::{ensure, Result};
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationSpec, WheelImuOdometry, WheelImuOdometryConfig,
    WheelImuOdometryError, WheelImuOdometryEstimate, WheelImuOdometryHealth,
    WheelImuOdometryProvenance, WheelImuOdometryStreams,
};
use rne_core::SimTime;
use rne_data::{
    DataBus, FrameHeader, InMemoryDataBus, MotorElectricalFeedback, PoseSample, StreamId,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    require_capabilities, JointState, PhysicsBackend, PhysicsBackendManifest, PhysicsCapability,
};
use rne_robot::{
    Actuator, ActuatorLimits, ActuatorTarget, ControlMode, DcMotorCompletedTelemetry, Joint,
    JointKind, JointLimits,
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
use sha2::{Digest, Sha256};

use crate::diff_caster::{CasterPlant, DIFF_CASTER_FIXED_DELTA_TICKS};
use crate::diff_caster_control::{
    DifferentialCasterControlSpec, DifferentialCasterControlStatus, DifferentialCasterController,
};

const STEPS: u64 = 10_000;
const PERIOD_TICKS: u64 = 10_000_000;
const LATENCY_TICKS: u64 = 2_000_000;
const COUNTS_PER_REVOLUTION: u32 = 2048;
const ENCODERS: [StreamId; 2] = [StreamId::new(4101), StreamId::new(4102)];
const IMU: StreamId = StreamId::new(4103);
const MOTORS: [StreamId; 2] = [StreamId::new(4104), StreamId::new(4105)];

/// One fixed-step controller decision, including held and expired commands.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CasterObservedSample {
    /// Decision instant; voltages below apply to the next motor/tire interval.
    pub decision_ticks: u64,
    /// Accepted measurement capture instant, absent on held/expired decisions.
    pub capture_ticks: Option<u64>,
    /// Oldest measurement capture underlying the accepted estimate.
    pub oldest_source_capture_ticks: Option<u64>,
    /// Initializing=0, nominal=1, sequence gap=2, saturated=3, disagreement=4.
    pub estimate_health_code: Option<u8>,
    /// Left encoder, right encoder and IMU sequences on an accepted update.
    pub source_sequences: Option<[u64; 3]>,
    /// Estimated forward speed and yaw rate, only on a new estimator update.
    pub estimated_twist_m_s_rad_s: Option<[f64; 2]>,
    /// Estimated planar x/y position, only on a new estimator update.
    pub estimated_position_m: Option<[f64; 2]>,
    /// Task reference: forward speed and counterclockwise yaw rate.
    pub target_twist_m_s_rad_s: [f64; 2],
    /// Left/right terminal voltage selected for the next drive-path interval.
    pub command_voltage_v: [f64; 2],
    /// Voltage actually supplied to this step's motor/tire update, before this decision.
    pub drive_interval_voltage_v: [f64; 2],
    /// Explicit controller reason, including every timeout step.
    pub control_status: DifferentialCasterControlStatus,
    /// Latest available measured currents; not substituted with commanded effort.
    pub motor_measurements: [Option<CasterMotorMeasurement>; 2],
    /// Scoring-only forward speed and planar yaw rate, never controller input.
    pub privileged_twist_m_s_rad_s: [f64; 2],
    /// Scoring-only world x/z position relative to the initial chassis origin.
    pub privileged_position_m: [f64; 2],
    /// Scoring-only caster swivel angle, in radians.
    pub privileged_caster_swivel_rad: f64,
}

/// A measured motor payload and its original `DataBus` timing/source header.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CasterMotorMeasurement {
    /// Original stream, entity, sequence, capture and availability metadata.
    pub header: FrameHeader,
    /// Completed electrical measurement, including saturation and availability status.
    pub payload: MotorElectricalFeedback,
}

/// Preliminary closed-loop evidence; physical calibration/acceptance is not implied.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CasterObservedRun {
    /// Stable evidence discriminator, unlike the earlier unversioned preview exports.
    pub kind: String,
    /// Version 2 adds explicit applied commands and sensor provenance validation.
    pub schema_version: u32,
    /// Backend manifest used for this run.
    pub backend: PhysicsBackendManifest,
    /// Identical portable controller contract on either backend.
    pub task_spec: TaskSpec,
    /// Seeded synthetic sensor/model contract; not a fitted physical profile.
    pub controller_spec: DifferentialCasterControlSpec,
    /// Whether IMU capture is disabled on steps 2000 through 2049.
    pub imu_blackout: bool,
    /// Every fixed-step decision, in deterministic time order.
    pub samples: Vec<CasterObservedSample>,
    /// SHA-256 of this record with this field empty; integrity, not authentication.
    pub content_digest: String,
}

impl CasterObservedRun {
    /// Validates timing, applied-command continuity, finite measurements and integrity.
    ///
    /// Replays the controller from recorded estimates and references, never from
    /// privileged truth. This verifies controller decisions, not the estimator's
    /// raw-sensor computation or the backend's physics trajectory. The digest
    /// detects accidental edits but is not a digital signature or source attestation.
    #[allow(clippy::too_many_lines)] // TODO(cleanup): split (198/150 lines); see PR body
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_diff_caster_sensor_trace" && self.schema_version == 2,
            "unsupported caster evidence"
        );
        self.backend.validate()?;
        ensure!(
            self.task_spec == caster_observed_task_spec(),
            "TaskSpec drift"
        );
        ensure!(
            self.samples.len() == STEPS as usize,
            "incomplete decision trace"
        );
        ensure!(
            self.controller_spec.maximum_voltage_v <= 12.0,
            "voltage contract drift"
        );
        let mut controller = DifferentialCasterController::new(self.controller_spec)?;
        let mut previous_command = [0.0; 2];
        let mut previous_motor: [Option<&CasterMotorMeasurement>; 2] = [None, None];
        for (index, sample) in self.samples.iter().enumerate() {
            let step = index as u64 + 1;
            let now = step * DIFF_CASTER_FIXED_DELTA_TICKS;
            ensure!(sample.decision_ticks == now, "decision clock drift");
            ensure!(
                sample.target_twist_m_s_rad_s == target_for_step(step),
                "reference drift"
            );
            ensure!(
                sample.drive_interval_voltage_v == previous_command,
                "applied command not previous decision"
            );
            ensure!(
                sample
                    .command_voltage_v
                    .iter()
                    .all(|v| v.is_finite() && v.abs() <= self.controller_spec.maximum_voltage_v),
                "invalid voltage"
            );
            ensure!(
                sample
                    .privileged_position_m
                    .iter()
                    .chain(sample.privileged_twist_m_s_rad_s.iter())
                    .all(|v| v.is_finite())
                    && sample.privileged_caster_swivel_rad.is_finite(),
                "nonfinite scoring evidence"
            );
            let present = sample.capture_ticks.is_some();
            let expected_capture = now.checked_sub(LATENCY_TICKS);
            let expected_estimate = expected_capture.is_some_and(|capture| {
                (capture == DIFF_CASTER_FIXED_DELTA_TICKS
                    || (capture >= PERIOD_TICKS && capture % PERIOD_TICKS == 0))
                    && (!self.imu_blackout || !(2_000_000_000..2_050_000_000).contains(&capture))
            });
            ensure!(
                present == expected_estimate,
                "missing or unexpected estimator update"
            );
            ensure!(
                [
                    sample.oldest_source_capture_ticks.is_some(),
                    sample.estimate_health_code.is_some(),
                    sample.source_sequences.is_some(),
                    sample.estimated_twist_m_s_rad_s.is_some(),
                    sample.estimated_position_m.is_some()
                ]
                .iter()
                .all(|value| *value == present),
                "partial estimate evidence"
            );
            let estimate = if let Some(capture) = sample.capture_ticks {
                let oldest = sample
                    .oldest_source_capture_ticks
                    .expect("presence checked");
                ensure!(
                    capture.checked_add(LATENCY_TICKS) == Some(now) && oldest == capture,
                    "future, stale or skewed estimate"
                );
                ensure!(
                    !self.imu_blackout || !(2_000_000_000..2_050_000_000).contains(&capture),
                    "capture during IMU outage"
                );
                let twist = sample.estimated_twist_m_s_rad_s.expect("presence checked");
                let position = sample.estimated_position_m.expect("presence checked");
                ensure!(
                    twist.iter().chain(position.iter()).all(|v| v.is_finite()),
                    "nonfinite estimate"
                );
                let health = match sample.estimate_health_code.expect("presence checked") {
                    0 => WheelImuOdometryHealth::Initializing,
                    1 => WheelImuOdometryHealth::Nominal,
                    2 => WheelImuOdometryHealth::InputSequenceGap,
                    3 => WheelImuOdometryHealth::ImuSaturated,
                    4 => WheelImuOdometryHealth::WheelImuDisagreement,
                    _ => anyhow::bail!("invalid estimator health"),
                };
                let sequences = sample.source_sequences.expect("presence checked");
                Some(WheelImuOdometryEstimate {
                    pose: PoseSample {
                        position_m: Vec3::new(position[0], position[1], 0.0),
                        ..Default::default()
                    },
                    linear_velocity_m_s: twist[0],
                    angular_velocity_rad_s: twist[1],
                    health,
                    provenance: WheelImuOdometryProvenance {
                        left_sequence: sequences[0],
                        right_sequence: sequences[1],
                        imu_sequence: sequences[2],
                        capture_ticks: capture,
                        decision_ticks: now,
                        max_age_ticks: now - oldest,
                        skipped_sequences: 0,
                    },
                    // Not consumed by this controller; this is not estimator replay.
                    encoder_delta_yaw_rad: 0.0,
                    gyro_delta_yaw_rad: 0.0,
                    yaw_innovation_rad: 0.0,
                    pose_covariance: [[0.0; 3]; 3],
                })
            } else {
                None
            };
            let replay = controller.update(
                SimTime::from_ticks(now),
                estimate.as_ref(),
                sample.target_twist_m_s_rad_s,
            )?;
            ensure!(
                replay.status == sample.control_status
                    && replay.voltage_v == sample.command_voltage_v,
                "controller replay mismatch"
            );
            for (side, measurement) in sample.motor_measurements.iter().enumerate() {
                ensure!(
                    measurement.is_some() == (now >= DIFF_CASTER_FIXED_DELTA_TICKS + LATENCY_TICKS),
                    "missing motor frame"
                );
                if let Some(measurement) = measurement {
                    let h = &measurement.header;
                    let p = &measurement.payload;
                    let expected_capture = ((now - LATENCY_TICKS) / PERIOD_TICKS * PERIOD_TICKS)
                        .max(DIFF_CASTER_FIXED_DELTA_TICKS);
                    ensure!(
                        h.capture_ticks == expected_capture
                            && h.sequence == expected_capture / PERIOD_TICKS + 1,
                        "motor sampling schedule drift"
                    );
                    ensure!(
                        h.stream_id == MOTORS[side]
                            && h.sequence > 0
                            && h.capture_ticks.checked_add(LATENCY_TICKS)
                                == Some(h.available_ticks)
                            && h.available_ticks <= now,
                        "invalid motor source timing"
                    );
                    ensure!(now - h.available_ticks < PERIOD_TICKS, "stale motor frame");
                    ensure!(
                        p.schema_version == MotorElectricalFeedback::SCHEMA_VERSION
                            && p.scheduled_capture_ticks
                                .checked_add(p.sample_phase_error_ticks)
                                == Some(h.capture_ticks),
                        "motor payload timing drift"
                    );
                    ensure!(
                        p.current_a.is_finite()
                            && p.current_a.abs() <= 25.0
                            && p.terminal_voltage_v.is_finite()
                            && p.terminal_voltage_v.abs() <= 30.0
                            && p.winding_temperature_c
                                .is_none_or(|v| v.is_finite() && (-40.0..=180.0).contains(&v)),
                        "invalid motor measurement"
                    );
                    if let Some(previous) = previous_motor[side] {
                        ensure!(
                            previous.header.entity_index == h.entity_index,
                            "motor entity changed"
                        );
                        if h.sequence == previous.header.sequence {
                            ensure!(measurement == previous, "held motor frame changed");
                        } else {
                            ensure!(
                                h.sequence > previous.header.sequence
                                    && h.capture_ticks > previous.header.capture_ticks,
                                "nonadvancing motor frame"
                            );
                        }
                    }
                    previous_motor[side] = Some(measurement);
                }
            }
            previous_command = sample.command_voltage_v;
        }
        ensure!(
            self.content_digest == observed_digest(self)?,
            "caster content digest mismatch"
        );
        Ok(())
    }
}

/// Defines the actor-visible speed/yaw tracking task without privileged tensors.
pub fn caster_observed_task_spec() -> TaskSpec {
    TaskSpec::new(
        "mobility_diff_caster_sensor_twist_v2",
        0.001,
        ObservationSpec::new(vec![
            TensorSpec::new("new_estimate_available", TensorDType::U8, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("estimate_health_code", TensorDType::U8, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 4.0)),
            TensorSpec::new(
                "estimated_forward_velocity_m_s",
                TensorDType::F64,
                vec![],
                "m/s",
            ),
            TensorSpec::new(
                "estimated_yaw_rate_rad_s",
                TensorDType::F64,
                vec![],
                "rad/s",
            ),
            TensorSpec::new(
                "target_forward_velocity_m_s",
                TensorDType::F64,
                vec![],
                "m/s",
            ),
            TensorSpec::new("target_yaw_rate_rad_s", TensorDType::F64, vec![], "rad/s"),
            TensorSpec::new("oldest_source_age_s", TensorDType::F64, vec![], "s"),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "left_right_terminal_voltage_v",
            TensorDType::F64,
            vec![2],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(-12.0, 12.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_speed_error", -1.0, "m/s"),
            RewardTermSpec::new("truth_yaw_rate_error", -1.0, "rad/s"),
        ]),
        TerminationSpec::new(vec![], Some(STEPS)),
        ResetSpec::splitmix64(false),
    )
}

/// Runs real sensor frontends, sensor-only estimation and PI control on the caster plant.
///
/// Sensor capture period is 10 ms, delivery latency 2 ms, encoder resolution 2048
/// counts/revolution and all noise seeds are explicit. An optional 50-ms IMU capture
/// outage checks timeout/recovery. The drive path retains its one-step wrench delay.
/// This runner emits evidence for evaluation, not a physical-fidelity certificate.
pub fn run_caster_observed<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    imu_blackout: bool,
) -> Result<CasterObservedRun> {
    run_caster_observed_with_control_spec(
        backend,
        manifest,
        imu_blackout,
        DifferentialCasterControlSpec::default(),
    )
}

/// Derives velocity-only feedforward from the fixed nominal fixture motor model.
///
/// This uses nominal constants, never a running plant's sampled parameters or
/// instantaneous truth. It excludes static friction, load and acceleration terms.
/// `k_v = (K_e + R*b/K_t)*gear_ratio/radius`; yaw uses `k_v*track_width/2`.
/// These are analytical synthetic-model coefficients, not a physical calibration.
pub fn nominal_caster_feedforward_spec() -> DifferentialCasterControlSpec {
    let motor = rne_robot::DcMotorSpec::default();
    let transmission = rne_robot::TransmissionSpec::default();
    let speed_feedforward_v_s_m = (motor.back_emf_constant_v_s_rad
        + motor.resistance_ohm * motor.viscous_friction_nm_s_rad / motor.torque_constant_nm_a)
        * transmission.ratio_motor_rad_per_wheel_rad
        / 0.12;
    DifferentialCasterControlSpec {
        speed_feedforward_v_s_m,
        yaw_feedforward_v_s_rad: speed_feedforward_v_s_m * 0.6 / 2.0,
        ..DifferentialCasterControlSpec::default()
    }
}

/// Runs the same task and sensors with an explicitly supplied controller contract.
///
/// The controller contract is recorded alongside results for PI/feedforward
/// comparisons. All voltage limits must fit the unchanged `TaskSpec` action bounds.
#[allow(clippy::too_many_lines)] // TODO(cleanup): split (158/150 lines); see PR body
pub fn run_caster_observed_with_control_spec<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    imu_blackout: bool,
    controller_spec: DifferentialCasterControlSpec,
) -> Result<CasterObservedRun> {
    manifest.validate()?;
    ensure!(
        manifest.capabilities == backend.capabilities(),
        "capability drift"
    );
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
    let task_spec = caster_observed_task_spec();
    task_spec.validate()?;
    ensure!(
        controller_spec.maximum_voltage_v <= 12.0,
        "controller exceeds TaskSpec voltage bound"
    );
    let mut controller = DifferentialCasterController::new(controller_spec)?;
    let mut plant = CasterPlant::new(backend)?;
    plant.world.insert_resource(WorldRandom::new(0));
    let rig = spawn_rig(&mut plant.world, plant.chassis);
    let mut estimator = WheelImuOdometry::new(
        WheelImuOdometryConfig {
            wheel_radius_m: plant.plant.wheel.radius_m,
            track_width_m: 0.6,
            left_counts_per_revolution: COUNTS_PER_REVOLUTION,
            right_counts_per_revolution: COUNTS_PER_REVOLUTION,
            left_counter_bits: 32,
            right_counter_bits: 32,
            left_direction: 1,
            right_direction: 1,
            gyro_z_direction: 1.0,
            max_abs_wheel_delta_counts: 20000,
            gyro_z_bias_rad_s: 0.0,
            wheel_distance_std_m: 0.001,
            encoder_yaw_std_rad: 0.002,
            gyro_rate_std_rad_s: 0.001,
            gyro_yaw_weight: 0.8,
            disagreement_gyro_yaw_weight: 1.0,
            disagreement_threshold_rad: 0.001,
            max_input_skew_ticks: 0,
            max_frame_age_ticks: LATENCY_TICKS,
        },
        PoseSample::default(),
    )?;
    let mut bus = InMemoryDataBus::new();
    let mut command = [0.0; 2];
    let mut samples = Vec::with_capacity(STEPS as usize);
    for step in 1..=STEPS {
        let drive_interval_voltage_v = command;
        let completed = plant.step(command)?;
        for index in 0..2 {
            plant
                .world
                .entity_mut(rig.joints[index])
                .insert(JointState::Revolute {
                    position_rad: plant.drive_states[index].wheel_position_rad,
                    velocity_rad_s: plant.drive_states[index].wheel_velocity_rad_s,
                });
            plant
                .world
                .entity_mut(rig.motors[index])
                .insert(completed.motor_telemetry[index]);
        }
        plant
            .world
            .get_mut::<ImuFeedbackSensor>(rig.imu)
            .expect("IMU")
            .enabled = !(imu_blackout && (2000..2050).contains(&step));
        let now = SimTime::from_ticks(step * DIFF_CASTER_FIXED_DELTA_TICKS);
        sample_incremental_encoder_sensors(&mut plant.world, now, &mut bus)?;
        sample_imu_feedback_sensors(&mut plant.world, now, &mut bus)?;
        sample_motor_electrical_feedback_sensors(&mut plant.world, now, &mut bus)?;
        let estimate = match estimator.update(
            &bus,
            WheelImuOdometryStreams {
                left_encoder: ENCODERS[0],
                right_encoder: ENCODERS[1],
                imu: IMU,
            },
            now,
        ) {
            Ok(estimate) => Some(estimate),
            Err(
                WheelImuOdometryError::MissingAvailableFrame { .. }
                | WheelImuOdometryError::NoNewEncoderPair
                | WheelImuOdometryError::StaleInput { .. }
                | WheelImuOdometryError::InputSkew { .. },
            ) => None,
            Err(error) => return Err(error.into()),
        };
        let target = target_for_step(step);
        let output = controller.update(now, estimate.as_ref(), target)?;
        command = output.voltage_v;
        samples.push(CasterObservedSample {
            decision_ticks: now.ticks(),
            capture_ticks: estimate.map(|e| e.provenance.capture_ticks),
            oldest_source_capture_ticks: estimate
                .map(|e| e.provenance.decision_ticks - e.provenance.max_age_ticks),
            estimate_health_code: estimate.map(|e| health_code(e.health)),
            source_sequences: estimate.map(|e| {
                [
                    e.provenance.left_sequence,
                    e.provenance.right_sequence,
                    e.provenance.imu_sequence,
                ]
            }),
            estimated_twist_m_s_rad_s: estimate
                .map(|e| [e.linear_velocity_m_s, e.angular_velocity_rad_s]),
            estimated_position_m: estimate.map(|e| [e.pose.position_m.x, e.pose.position_m.y]),
            target_twist_m_s_rad_s: target,
            command_voltage_v: command,
            drive_interval_voltage_v,
            control_status: output.status,
            motor_measurements: MOTORS.map(|stream| {
                bus.latest_available::<MotorElectricalFeedback>(stream, now)
                    .map(|frame| CasterMotorMeasurement {
                        header: FrameHeader {
                            stream_id: frame.stream_id,
                            entity_index: frame.entity.index(),
                            sequence: frame.sequence,
                            capture_ticks: frame.capture_time.ticks(),
                            available_ticks: frame.available_time.ticks(),
                        },
                        payload: frame.payload,
                    })
            }),
            privileged_twist_m_s_rad_s: [
                completed
                    .body
                    .linear_velocity_m_s
                    .dot(completed.transform.rotation * Vec3::X),
                -completed.body.angular_velocity_rad_s.y,
            ],
            privileged_position_m: [
                completed.transform.translation.x,
                completed.transform.translation.z,
            ],
            privileged_caster_swivel_rad: completed.caster_swivel_rad,
        });
    }
    let mut run = CasterObservedRun {
        kind: "rne_diff_caster_sensor_trace".into(),
        schema_version: 2,
        backend: manifest,
        task_spec,
        controller_spec,
        imu_blackout,
        samples,
        content_digest: String::new(),
    };
    run.content_digest = observed_digest(&run)?;
    run.validate()?;
    Ok(run)
}

fn target_for_step(step: u64) -> [f64; 2] {
    match step {
        0..=1000 => [0.0, 0.0],
        1001..=5000 => [0.25, 0.15],
        5001..=7000 => [0.25, 0.0],
        _ => [-0.15, 0.0],
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

fn observed_digest(run: &CasterObservedRun) -> Result<String> {
    let mut unhashed = run.clone();
    unhashed.content_digest.clear();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&unhashed)?)
    ))
}

#[derive(Clone, Copy, Debug)]
struct SensorRig {
    joints: [Entity; 2],
    motors: [Entity; 2],
    imu: Entity,
}

fn spawn_rig(world: &mut World, chassis: Entity) -> SensorRig {
    let joints = std::array::from_fn(|index| {
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
        let actuator = world
            .spawn(Actuator {
                robot: chassis,
                joint: Some(joint),
                name: format!("caster_drive_{index}"),
                mode: ControlMode::Velocity,
                target: ActuatorTarget::default(),
                limits: ActuatorLimits::default(),
            })
            .id();
        let sensor = spawn_named(world, format!("caster_encoder_{index}"));
        world.entity_mut(sensor).insert((
            IncrementalEncoderSensor {
                spec: IncrementalEncoderSpec {
                    actuator,
                    counts_per_revolution: COUNTS_PER_REVOLUTION,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    counter_bits: 32,
                    overflow_behavior: IncrementalEncoderOverflowBehavior::Wrap,
                    velocity_window_samples: 2,
                    index_phase_rad: Some(0.0),
                },
                update_rate_hz: 100.0,
                sample_period_ticks: Some(PERIOD_TICKS),
                phase_offset_ticks: 0,
                latency_ticks: LATENCY_TICKS,
                enabled: true,
                stream_id: ENCODERS[index],
                fault: IncrementalEncoderFault::None,
            },
            IncrementalEncoderSensorState::default(),
        ));
        joint
    });
    let imu = spawn_named(world, "caster_imu");
    world.entity_mut(imu).insert((
        ImuMount {
            body_entity: chassis,
            // Sensor +Z maps to body -Y, matching counterclockwise yaw in world x/z.
            body_from_sensor: Transform3::from_translation_rotation(
                Vec3::new(0.1, 0.05, 0.0),
                Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            ),
        },
        ImuFeedbackSensor {
            spec: ImuSpec {
                seed: 29,
                gyro: ImuAxisErrors {
                    random_walk: 0.0002,
                    bias_instability: 0.00002,
                    bias_correlation_time_s: 100.0,
                    rate_random_walk: 0.00001,
                    ..ImuAxisErrors::default()
                },
                accel: ImuAxisErrors {
                    random_walk: 0.002,
                    ..ImuAxisErrors::default()
                },
                gyro_range_rad_s: 10.0,
                accel_range_m_s2: 40.0,
                gyro_resolution_rad_s: 0.0001,
                accel_resolution_m_s2: 0.001,
                ..ImuSpec::default()
            },
            update_rate_hz: 100.0,
            sample_period_ticks: Some(PERIOD_TICKS),
            phase_offset_ticks: 0,
            latency_ticks: LATENCY_TICKS,
            enabled: true,
            stream_id: IMU,
            fault: ImuFeedbackFault::None,
        },
        ImuFeedbackSensorState::default(),
    ));
    let motors = std::array::from_fn(|index| {
        let entity = world.spawn(DcMotorCompletedTelemetry::default()).id();
        let sensor = spawn_named(world, format!("caster_motor_feedback_{index}"));
        world.entity_mut(sensor).insert((
            MotorElectricalFeedbackSensor {
                spec: MotorElectricalFeedbackSpec {
                    motor_entity: entity,
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
                sample_period_ticks: Some(PERIOD_TICKS),
                phase_offset_ticks: 0,
                latency_ticks: LATENCY_TICKS,
                enabled: true,
                stream_id: MOTORS[index],
                fault: MotorElectricalFeedbackFault::None,
            },
            MotorElectricalFeedbackSensorState::default(),
        ));
        entity
    });
    SensorRig {
        joints,
        motors,
        imu,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    fn arc_rmse(run: &CasterObservedRun) -> [f64; 2] {
        let arc: Vec<_> = run
            .samples
            .iter()
            .filter(|s| (4_000_000_000..5_000_000_000).contains(&s.decision_ticks))
            .collect();
        std::array::from_fn(|axis| {
            (arc.iter()
                .map(|s| {
                    (s.privileged_twist_m_s_rad_s[axis] - s.target_twist_m_s_rad_s[axis]).powi(2)
                })
                .sum::<f64>()
                / arc.len() as f64)
                .sqrt()
        })
    }

    fn check_nominal_feedforward<B: PhysicsBackend>(
        mut make_backend: impl FnMut() -> B,
        manifest: PhysicsBackendManifest,
    ) {
        let baseline = run_caster_observed(make_backend(), manifest.clone(), false).unwrap();
        let spec = nominal_caster_feedforward_spec();
        assert!((spec.speed_feedforward_v_s_m - 14.375).abs() < 1.0e-12);
        assert!((spec.yaw_feedforward_v_s_rad - 4.3125).abs() < 1.0e-12);
        let candidate =
            run_caster_observed_with_control_spec(make_backend(), manifest.clone(), false, spec)
                .unwrap();
        let repeated =
            run_caster_observed_with_control_spec(make_backend(), manifest.clone(), false, spec)
                .unwrap();
        assert_eq!(candidate, repeated);
        assert_eq!(baseline.task_spec, candidate.task_spec);
        let original = arc_rmse(&baseline);
        let improved = arc_rmse(&candidate);
        eprintln!(
            "PI arc RMSE={original:?}; nominal FF+PI arc RMSE={improved:?}; final reverse={} m/s",
            candidate.samples.last().unwrap().privileged_twist_m_s_rad_s[0]
        );
        // Predeclared synthetic-controller improvement checks, not real-data fitting.
        assert!(improved[0] < original[0] * 0.5, "speed RMSE did not halve");
        assert!(improved[1] < original[1] * 0.5, "yaw RMSE did not halve");
        assert!(
            candidate.samples.last().unwrap().privileged_twist_m_s_rad_s[0] < -0.10,
            "reverse remains too slow"
        );
        let outage =
            run_caster_observed_with_control_spec(make_backend(), manifest, true, spec).unwrap();
        let expired: Vec<_> = outage
            .samples
            .iter()
            .filter(|s| s.control_status == DifferentialCasterControlStatus::Expired)
            .collect();
        assert!(!expired.is_empty());
        assert!(expired.iter().all(|s| s.command_voltage_v == [0.0; 2]));
        assert!(outage
            .samples
            .iter()
            .any(|s| s.decision_ticks > 2_050_000_000
                && s.control_status == DifferentialCasterControlStatus::Tracking));
        assert!(outage.samples.iter().all(|s| s
            .command_voltage_v
            .iter()
            .all(|v| v.is_finite() && v.abs() <= 12.0)));
    }

    #[test]
    fn nominal_feedforward_improves_rapier_tracking() {
        check_nominal_feedforward(RapierBackend::new, RapierBackend::manifest());
    }

    #[test]
    fn versioned_trace_roundtrips_and_rejects_rehashed_semantic_mutations() {
        let original = run_caster_observed_with_control_spec(
            RapierBackend::new(),
            RapierBackend::manifest(),
            false,
            nominal_caster_feedforward_spec(),
        )
        .unwrap();
        let bytes = serde_json::to_vec(&original).unwrap();
        let restored: CasterObservedRun = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(original, restored);
        restored.validate().unwrap();
        let mut changed = original.clone();
        changed.samples[0].privileged_position_m[0] += 1.0;
        assert!(
            changed.validate().is_err(),
            "digest must detect changed scoring values"
        );
        for mutation in 0..7 {
            let mut changed = original.clone();
            match mutation {
                0 => changed.samples[2001].command_voltage_v[0] += 0.01,
                1 => changed.samples[0].drive_interval_voltage_v = [1.0; 2],
                2 => changed.samples[2].oldest_source_capture_ticks = Some(2_000_000),
                3 => changed.samples[2].capture_ticks = None,
                4 => {
                    changed.samples[2].motor_measurements[0]
                        .as_mut()
                        .unwrap()
                        .header
                        .available_ticks = 4_000_000
                }
                5 => changed.imu_blackout = true,
                _ => {
                    changed.samples[2].motor_measurements[0]
                        .as_mut()
                        .unwrap()
                        .payload
                        .current_a = f64::NAN
                }
            }
            // Recomputing the checksum must not bypass timing/controller invariants.
            changed.content_digest = observed_digest(&changed).unwrap();
            assert!(changed.validate().is_err(), "accepted mutation {mutation}");
        }
        let mut value = serde_json::to_value(&original).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("unknown_contract".into(), serde_json::json!(true));
        assert!(serde_json::from_value::<CasterObservedRun>(value).is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn nominal_feedforward_improves_mujoco_tracking() {
        use rne_physics_mujoco::MuJoCoBackend;
        let make_backend = || {
            MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                DIFF_CASTER_FIXED_DELTA_TICKS,
            ))
            .unwrap()
        };
        check_nominal_feedforward(make_backend, MuJoCoBackend::manifest());
    }

    #[test]
    fn rapier_closed_loop_uses_delayed_sensors_and_repeats() {
        let run =
            run_caster_observed(RapierBackend::new(), RapierBackend::manifest(), false).unwrap();
        let repeated =
            run_caster_observed(RapierBackend::new(), RapierBackend::manifest(), false).unwrap();
        assert_eq!(run, repeated);
        assert_eq!(run.samples.len(), STEPS as usize);
        let accepted: Vec<_> = run
            .samples
            .iter()
            .filter(|s| s.capture_ticks.is_some())
            .collect();
        assert!(accepted.len() > 900);
        for sample in &accepted {
            assert_eq!(
                sample.decision_ticks - sample.capture_ticks.unwrap(),
                LATENCY_TICKS
            );
            assert!(sample.motor_measurements.iter().all(Option::is_some));
        }
        let arc: Vec<_> = run
            .samples
            .iter()
            .filter(|s| (4_000_000_000..5_000_000_000).contains(&s.decision_ticks))
            .collect();
        let speed_rmse = (arc
            .iter()
            .map(|s| (s.privileged_twist_m_s_rad_s[0] - 0.25).powi(2))
            .sum::<f64>()
            / arc.len() as f64)
            .sqrt();
        let yaw_rmse = (arc
            .iter()
            .map(|s| (s.privileged_twist_m_s_rad_s[1] - 0.15).powi(2))
            .sum::<f64>()
            / arc.len() as f64)
            .sqrt();
        eprintln!("caster sensor loop: arc speed RMSE={speed_rmse} m/s, yaw RMSE={yaw_rmse} rad/s, final={:?}", run.samples.last().unwrap());
        assert!(speed_rmse < 0.15, "speed tracking {speed_rmse} m/s");
        assert!(yaw_rmse < 0.10, "yaw tracking {yaw_rmse} rad/s");
        assert!(
            run.samples.last().unwrap().privileged_twist_m_s_rad_s[0] < -0.02,
            "reverse motion"
        );
        let last = accepted.last().unwrap();
        let estimated = last.estimated_position_m.unwrap();
        let truth = last.privileged_position_m;
        let position_error_m = (estimated[0] - truth[0]).hypot(estimated[1] - truth[1]);
        assert!(
            position_error_m < 0.25,
            "planar estimate {position_error_m} m"
        );
    }

    #[test]
    fn imu_outage_expires_voltage_and_fresh_capture_recovers() {
        let run =
            run_caster_observed(RapierBackend::new(), RapierBackend::manifest(), true).unwrap();
        let expired: Vec<_> = run
            .samples
            .iter()
            .filter(|s| s.control_status == DifferentialCasterControlStatus::Expired)
            .collect();
        assert!(!expired.is_empty());
        assert!(expired
            .iter()
            .all(|s| s.command_voltage_v == [0.0; 2] && s.capture_ticks.is_none()));
        assert!(run.samples.iter().any(|s| s.decision_ticks > 2_050_000_000
            && s.control_status == DifferentialCasterControlStatus::Tracking));
        assert!(run.samples.iter().all(|s| s
            .command_voltage_v
            .iter()
            .all(|v| v.is_finite() && v.abs() <= 12.0)));
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn mujoco_sensor_loop_repeats_and_matches_the_rapier_contract() {
        use rne_physics_mujoco::MuJoCoBackend;
        let make_backend = || {
            MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                DIFF_CASTER_FIXED_DELTA_TICKS,
            ))
            .unwrap()
        };
        let mujoco = run_caster_observed(make_backend(), MuJoCoBackend::manifest(), false).unwrap();
        let repeated =
            run_caster_observed(make_backend(), MuJoCoBackend::manifest(), false).unwrap();
        assert_eq!(mujoco, repeated);
        let rapier =
            run_caster_observed(RapierBackend::new(), RapierBackend::manifest(), false).unwrap();
        assert_eq!(mujoco.task_spec, rapier.task_spec);
        assert_eq!(mujoco.controller_spec, rapier.controller_spec);
        assert_eq!(mujoco.samples.len(), rapier.samples.len());
        let mut squared_twist_gap = [0.0; 2];
        for (left, right) in mujoco.samples.iter().zip(&rapier.samples) {
            assert_eq!(left.decision_ticks, right.decision_ticks);
            assert_eq!(left.capture_ticks, right.capture_ticks);
            assert_eq!(left.source_sequences, right.source_sequences);
            for (axis, sum) in squared_twist_gap.iter_mut().enumerate() {
                *sum += (left.privileged_twist_m_s_rad_s[axis]
                    - right.privileged_twist_m_s_rad_s[axis])
                    .powi(2);
            }
        }
        // Synthetic-fixture regression envelopes, not physical validation limits.
        assert!(
            (squared_twist_gap[0] / STEPS as f64).sqrt() < 0.02,
            "speed gap m/s"
        );
        assert!(
            (squared_twist_gap[1] / STEPS as f64).sqrt() < 0.02,
            "yaw-rate gap rad/s"
        );
        let left = mujoco.samples.last().unwrap().privileged_position_m;
        let right = rapier.samples.last().unwrap().privileged_position_m;
        assert!(
            (left[0] - right[0]).hypot(left[1] - right[1]) < 0.05,
            "final position gap m"
        );
        let outage = run_caster_observed(make_backend(), MuJoCoBackend::manifest(), true).unwrap();
        let expired: Vec<_> = outage
            .samples
            .iter()
            .filter(|s| s.control_status == DifferentialCasterControlStatus::Expired)
            .collect();
        assert!(!expired.is_empty());
        assert!(expired.iter().all(|s| s.command_voltage_v == [0.0; 2]));
        assert!(outage
            .samples
            .iter()
            .any(|s| s.decision_ticks > 2_050_000_000
                && s.control_status == DifferentialCasterControlStatus::Tracking));
    }
}
