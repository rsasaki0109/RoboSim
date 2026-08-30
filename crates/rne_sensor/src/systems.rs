//! Sensor sampling systems.

use crate::camera::sample_camera_rgbd_keyed;
use crate::components::{
    ImuFeedbackFault, ImuFeedbackSensor, ImuFeedbackSensorState, ImuKinematicState, ImuMount,
    ImuState, IncrementalEncoderFault, IncrementalEncoderOverflowBehavior,
    IncrementalEncoderSensor, IncrementalEncoderSensorState, JointFeedbackFault,
    JointFeedbackSensor, JointFeedbackSensorState, Sensor, SensorKind, SensorState,
};
use crate::imu::{
    sample_imu_stateful_diagnostic_with_kinematics, sample_imu_stateful_with_kinematics,
    ImuSampleError,
};
use crate::lidar::sample_lidar_at_entity_keyed;
use crate::noise::SensorNoiseKey;
use crate::wheel_encoder::sample_wheel_encoder;
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, Frame, FramePayload, ImuFeedback, ImuFeedbackStatus, IncrementalEncoderFeedback,
    IncrementalEncoderStatus, JointCommandFeedback, JointCommandMode, JointCoordinateFeedback,
    JointEffortFeedback, JointFeedback, JointFeedbackChannel, JointFeedbackStatus,
};
use rne_ecs::{Entity, World};
use rne_physics::{
    JointActuation, JointEffortMeasurement, JointState, PhysicsBackend, PhysicsWorldId,
};
use rne_render::{HeadlessRenderBackend, RenderBackend, RenderScene};
use rne_robot::{Actuator, Joint, JointKind};
use rne_world::{Transform3, WorldRandom};
use thiserror::Error;

/// Context required to sample sensors in the simulation loop.
pub struct SensorSampleContext<'a, B: PhysicsBackend> {
    /// ECS world.
    pub world: &'a mut World,
    /// Current simulation time.
    pub sim_time: SimTime,
    /// Physics backend used for raycast sensors.
    pub physics: &'a B,
    /// Physics world identifier.
    pub physics_world: PhysicsWorldId,
    /// Optional render backend for camera sensors.
    pub render: Option<&'a mut dyn RenderBackend>,
    /// Optional scene geometry for camera depth sampling.
    pub scene: Option<&'a RenderScene>,
}

/// Stream-id offset for paired depth frames published beside RGB camera streams.
pub const CAMERA_DEPTH_STREAM_OFFSET: u64 = 50;

/// Samples all enabled sensors and publishes frames to the DataBus.
pub fn sample_sensors<B: PhysicsBackend>(
    ctx: &mut SensorSampleContext<'_, B>,
    bus: &mut impl DataBus,
) -> usize {
    let mut published = 0_usize;
    let mut updates: Vec<(rne_ecs::Entity, SensorState)> = Vec::new();
    let mut imu_updates: Vec<(rne_ecs::Entity, ImuState, ImuKinematicState)> = Vec::new();
    let mut headless_render = HeadlessRenderBackend::new();
    let empty_scene = RenderScene::new();
    let world_seed = ctx
        .world
        .get_resource::<WorldRandom>()
        .map(WorldRandom::seed)
        .unwrap_or(0);

    for entity_ref in ctx.world.iter_entities() {
        let entity = entity_ref.id();
        let Some(sensor) = ctx.world.get::<Sensor>(entity).cloned() else {
            continue;
        };
        if !sensor.enabled {
            continue;
        }

        let mut state = ctx
            .world
            .get::<SensorState>(entity)
            .cloned()
            .unwrap_or_default();

        if !should_sample(&sensor, &state, ctx.sim_time) {
            continue;
        }

        state.last_sequence += 1;
        state.frame_count += 1;
        state.last_sample_ticks = ctx.sim_time.ticks();

        match &sensor.kind {
            SensorKind::Imu(spec) => {
                // The IMU error model is time correlated, so its state rides on the
                // sensor entity and is written back with the sampling state below.
                let mut imu_state = ctx
                    .world
                    .get::<ImuState>(entity)
                    .copied()
                    .unwrap_or_default();
                let mut kinematic_state = ctx
                    .world
                    .get::<ImuKinematicState>(entity)
                    .copied()
                    .unwrap_or_default();
                let sample = sample_imu_stateful_with_kinematics(
                    ctx.world,
                    entity,
                    spec,
                    SensorNoiseKey::new(
                        world_seed,
                        spec.seed,
                        sensor.stream_id.0,
                        state.last_sequence,
                    ),
                    ctx.sim_time,
                    &mut imu_state,
                    &mut kinematic_state,
                );
                imu_updates.push((entity, imu_state, kinematic_state));
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample,
                    )
                    .with_latency(sensor.latency()),
                );
            }
            SensorKind::Lidar(spec) => {
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample_lidar_at_entity_keyed(
                            ctx.physics,
                            ctx.physics_world,
                            ctx.world,
                            entity,
                            spec,
                            SensorNoiseKey::new(
                                world_seed,
                                spec.seed,
                                sensor.stream_id.0,
                                state.last_sequence,
                            ),
                        ),
                    )
                    .with_latency(sensor.latency()),
                );
            }
            SensorKind::Camera(spec) => {
                let transform = ctx
                    .world
                    .get::<Transform3>(entity)
                    .copied()
                    .unwrap_or_default();
                let scene = ctx.scene.unwrap_or(&empty_scene);
                let noise_key = SensorNoiseKey::new(
                    world_seed,
                    spec.seed,
                    sensor.stream_id.0,
                    state.last_sequence,
                );
                let sample = if let Some(render) = &mut ctx.render {
                    sample_camera_rgbd_keyed(
                        *render,
                        &transform,
                        spec,
                        ctx.sim_time,
                        scene,
                        noise_key,
                    )
                } else {
                    sample_camera_rgbd_keyed(
                        &mut headless_render,
                        &transform,
                        spec,
                        ctx.sim_time,
                        scene,
                        noise_key,
                    )
                };
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample.rgb,
                    )
                    .with_latency(sensor.latency()),
                );
                publish_frame(
                    bus,
                    Frame::new(
                        rne_data::StreamId::new(sensor.stream_id.0 + CAMERA_DEPTH_STREAM_OFFSET),
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample.depth,
                    )
                    .with_latency(sensor.latency()),
                );
            }
            SensorKind::WheelEncoder(spec) => {
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample_wheel_encoder(ctx.world, spec),
                    )
                    .with_latency(sensor.latency()),
                );
            }
        }

        published += 1;
        updates.push((entity, state));
    }

    for (entity, state) in updates {
        if let Some(mut component) = ctx.world.get_mut::<SensorState>(entity) {
            *component = state;
        }
    }

    for (entity, state, kinematic_state) in imu_updates {
        ctx.world
            .entity_mut(entity)
            .insert((state, kinematic_state));
    }

    published
}

fn publish_frame<T: FramePayload>(bus: &mut impl DataBus, frame: Frame<T>) {
    bus.publish(frame);
}

fn should_sample(sensor: &Sensor, state: &SensorState, sim_time: SimTime) -> bool {
    let period = sensor.period();
    if period.ticks() == 0 {
        return false;
    }

    if state.frame_count == 0 {
        return true;
    }

    sim_time.ticks().saturating_sub(state.last_sample_ticks) >= period.ticks()
}

/// Typed IMU-feedback sampling error.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ImuFeedbackError {
    /// Sensor configuration is invalid or its exact period is zero.
    #[error("invalid IMU feedback sensor on entity {sensor_entity_index}")]
    InvalidSensor {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The sensor entity has no explicit mount calibration.
    #[error("IMU feedback sensor {sensor_entity_index} has no ImuMount")]
    MissingMount {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The sample schedule overflowed simulation ticks.
    #[error("IMU feedback schedule overflow on entity {sensor_entity_index}")]
    ScheduleOverflow {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// Mount or body kinematics failed validation before publication.
    #[error("IMU feedback sensor {sensor_entity_index} cannot sample: {source}")]
    Sampling {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
        /// Precise mount/kinematics failure.
        source: ImuSampleError,
    },
    /// A stuck-value fault has no prior emitted value to hold.
    #[error("IMU stuck-value fault on entity {sensor_entity_index} has no prior sample")]
    StuckWithoutPrevious {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
}

/// Samples every typed IMU-feedback sensor in deterministic entity order.
///
/// Processing order is fixed: schedule, mount-aware truth and raw measurement,
/// range saturation, quantization/noise, stuck substitution, frame dropout,
/// then availability latency. Every due sensor is validated before any state or
/// frame is published, so an invalid mount fails the complete sampling call
/// closed. Truth returned by the diagnostic sampler is deliberately discarded;
/// it belongs in validation evidence, never in the raw sensor payload.
pub fn sample_imu_feedback_sensors(
    world: &mut World,
    sim_time: SimTime,
    bus: &mut impl DataBus,
) -> Result<usize, ImuFeedbackError> {
    let mut sensors: Vec<(Entity, ImuFeedbackSensor)> = world
        .iter_entities()
        .filter_map(|entity_ref| {
            entity_ref
                .get::<ImuFeedbackSensor>()
                .cloned()
                .map(|sensor| (entity_ref.id(), sensor))
        })
        .collect();
    sensors.sort_unstable_by_key(|(entity, _)| entity.index());
    let world_seed = world
        .get_resource::<WorldRandom>()
        .map(WorldRandom::seed)
        .unwrap_or(0);

    let mut pending = Vec::new();
    for (sensor_entity, sensor) in sensors {
        if !sensor.enabled {
            continue;
        }
        if !sensor.is_valid() || sensor.period().ticks() == 0 {
            return Err(ImuFeedbackError::InvalidSensor {
                sensor_entity_index: sensor_entity.index(),
            });
        }
        if world.get::<ImuMount>(sensor_entity).is_none() {
            return Err(ImuFeedbackError::MissingMount {
                sensor_entity_index: sensor_entity.index(),
            });
        }
        let mut state = world
            .get::<ImuFeedbackSensorState>(sensor_entity)
            .cloned()
            .unwrap_or_default();
        let scheduled_capture_ticks = sensor
            .period()
            .ticks()
            .checked_mul(state.attempted_sequence)
            .and_then(|ticks| sensor.phase_offset_ticks.checked_add(ticks))
            .ok_or(ImuFeedbackError::ScheduleOverflow {
                sensor_entity_index: sensor_entity.index(),
            })?;
        if sim_time.ticks() < scheduled_capture_ticks {
            continue;
        }
        let sequence =
            state
                .attempted_sequence
                .checked_add(1)
                .ok_or(ImuFeedbackError::ScheduleOverflow {
                    sensor_entity_index: sensor_entity.index(),
                })?;
        let diagnostic = sample_imu_stateful_diagnostic_with_kinematics(
            world,
            sensor_entity,
            &sensor.spec,
            SensorNoiseKey::new(world_seed, sensor.spec.seed, sensor.stream_id.0, sequence),
            sim_time,
            &mut state.imu_state,
            &mut state.kinematic_state,
        )
        .map_err(|source| ImuFeedbackError::Sampling {
            sensor_entity_index: sensor_entity.index(),
            source,
        })?;
        let saturated = diagnostic.gyro_saturated.into_iter().any(|value| value)
            || diagnostic.accel_saturated.into_iter().any(|value| value);
        let mut payload = ImuFeedback {
            schema_version: ImuFeedback::SCHEMA_VERSION,
            scheduled_capture_ticks,
            sample_phase_error_ticks: sim_time.ticks() - scheduled_capture_ticks,
            status: if saturated {
                ImuFeedbackStatus::Saturated
            } else {
                ImuFeedbackStatus::Nominal
            },
            angular_velocity_rad_s: diagnostic.measurement.angular_velocity_rad_s,
            specific_force_m_s2: diagnostic.measurement.linear_acceleration_m_s2,
            gyro_saturated: diagnostic.gyro_saturated,
            accel_saturated: diagnostic.accel_saturated,
        };
        if matches!(
            sensor.fault,
            ImuFeedbackFault::StuckFromSequence { sequence: start } if sequence >= start
        ) {
            let previous =
                state
                    .last_emitted
                    .as_ref()
                    .ok_or(ImuFeedbackError::StuckWithoutPrevious {
                        sensor_entity_index: sensor_entity.index(),
                    })?;
            payload.angular_velocity_rad_s = previous.angular_velocity_rad_s;
            payload.specific_force_m_s2 = previous.specific_force_m_s2;
            payload.gyro_saturated = previous.gyro_saturated;
            payload.accel_saturated = previous.accel_saturated;
            payload.status = ImuFeedbackStatus::StuckValue;
        }

        state.attempted_sequence = sequence;
        let dropped = matches!(
            sensor.fault,
            ImuFeedbackFault::DropSequence { sequence: dropped } if sequence == dropped
        );
        let frame = if dropped {
            None
        } else {
            state.emitted_frames += 1;
            state.last_emitted = Some(payload);
            Some(
                Frame::new(sensor.stream_id, sensor_entity, sequence, sim_time, payload)
                    .with_latency(SimDuration::from_ticks(sensor.latency_ticks)),
            )
        };
        pending.push((sensor_entity, state, frame));
    }

    let mut published = 0;
    for (sensor_entity, state, frame) in pending {
        world.entity_mut(sensor_entity).insert(state);
        if let Some(frame) = frame {
            bus.publish(frame);
            published += 1;
        }
    }
    Ok(published)
}

/// Incremental encoder frontend sampling error.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum IncrementalEncoderError {
    /// Sensor timing, calibration, counter, or fault configuration is invalid.
    #[error("invalid incremental encoder sensor on entity {sensor_entity_index}")]
    InvalidSensor {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The sample schedule overflowed simulation ticks.
    #[error("incremental encoder schedule overflow on entity {sensor_entity_index}")]
    ScheduleOverflow {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The configured actuator or its joint is missing.
    #[error("incremental encoder entity {sensor_entity_index} has no valid actuator joint")]
    MissingActuatorJoint {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The configured actuator is not attached to a revolute joint.
    #[error("incremental encoder entity {sensor_entity_index} requires a revolute joint")]
    NonRevoluteJoint {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// Completed joint position is non-finite or outside the supported count range.
    #[error("incremental encoder entity {sensor_entity_index} has invalid completed position")]
    InvalidPosition {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// Count-difference accumulation overflowed its signed representation.
    #[error("incremental encoder count history overflow on entity {sensor_entity_index}")]
    CountHistoryOverflow {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// A stuck-value fault has no prior emitted value to hold.
    #[error(
        "incremental encoder stuck-value fault on entity {sensor_entity_index} has no prior sample"
    )]
    StuckWithoutPrevious {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
}

/// Samples typed incremental encoders in deterministic entity order.
///
/// Processing order is fixed: schedule, completed-position edge generation,
/// calibration, finite-counter wrap or saturation, count/time velocity
/// reconstruction, index detection, stuck substitution, frame dropout, then
/// output latency. Completed joint velocity is never read. Every due sensor is
/// validated before any state or frame is published, so failures are atomic.
pub fn sample_incremental_encoder_sensors(
    world: &mut World,
    sim_time: SimTime,
    bus: &mut impl DataBus,
) -> Result<usize, IncrementalEncoderError> {
    let mut sensors: Vec<(Entity, IncrementalEncoderSensor)> = world
        .iter_entities()
        .filter_map(|entity_ref| {
            entity_ref
                .get::<IncrementalEncoderSensor>()
                .cloned()
                .map(|sensor| (entity_ref.id(), sensor))
        })
        .collect();
    sensors.sort_unstable_by_key(|(entity, _)| entity.index());

    let mut pending = Vec::new();
    for (sensor_entity, sensor) in sensors {
        if !sensor.enabled {
            continue;
        }
        if !sensor.is_valid() || sensor.period().ticks() == 0 {
            return Err(IncrementalEncoderError::InvalidSensor {
                sensor_entity_index: sensor_entity.index(),
            });
        }
        let mut state = world
            .get::<IncrementalEncoderSensorState>(sensor_entity)
            .cloned()
            .unwrap_or_default();
        let scheduled_capture_ticks = sensor
            .period()
            .ticks()
            .checked_mul(state.attempted_sequence)
            .and_then(|ticks| sensor.phase_offset_ticks.checked_add(ticks))
            .ok_or(IncrementalEncoderError::ScheduleOverflow {
                sensor_entity_index: sensor_entity.index(),
            })?;
        if sim_time.ticks() < scheduled_capture_ticks {
            continue;
        }
        let sequence = state.attempted_sequence.checked_add(1).ok_or(
            IncrementalEncoderError::ScheduleOverflow {
                sensor_entity_index: sensor_entity.index(),
            },
        )?;
        let position_rad = completed_encoder_position(world, &sensor, sensor_entity)?;
        let ideal_count = encoder_ideal_count(position_rad, &sensor, sensor_entity)?;
        let (raw_count, saturated) = finite_encoder_count(ideal_count, &sensor);
        let (delta_count, counter_wrapped) =
            state.previous_raw_count.map_or((0, false), |previous| {
                observed_counter_delta(previous, raw_count, &sensor)
            });
        state.observed_accumulated_count = state
            .observed_accumulated_count
            .checked_add(delta_count)
            .ok_or(IncrementalEncoderError::CountHistoryOverflow {
                sensor_entity_index: sensor_entity.index(),
            })?;
        state
            .velocity_history
            .push((sim_time.ticks(), state.observed_accumulated_count));
        let maximum_observations = sensor.spec.velocity_window_samples as usize + 1;
        if state.velocity_history.len() > maximum_observations {
            state
                .velocity_history
                .drain(..state.velocity_history.len() - maximum_observations);
        }
        let velocity_rad_s = encoder_velocity_rad_s(&state, &sensor);
        let index_pulse = sensor.spec.index_phase_rad.is_some_and(|index_phase_rad| {
            state.previous_ideal_count.is_some_and(|previous| {
                crossed_encoder_index(previous, ideal_count, index_phase_rad, &sensor)
            })
        });
        let mut payload = IncrementalEncoderFeedback {
            schema_version: IncrementalEncoderFeedback::SCHEMA_VERSION,
            scheduled_capture_ticks,
            sample_phase_error_ticks: sim_time.ticks() - scheduled_capture_ticks,
            status: if saturated {
                IncrementalEncoderStatus::CounterSaturated
            } else if state.previous_raw_count.is_none() {
                IncrementalEncoderStatus::Initializing
            } else {
                IncrementalEncoderStatus::Nominal
            },
            raw_count,
            delta_count,
            position_rad: sensor.spec.zero_offset_rad
                + f64::from(sensor.spec.direction) * raw_count as f64 * std::f64::consts::TAU
                    / f64::from(sensor.spec.counts_per_revolution),
            velocity_rad_s,
            counter_wrapped,
            index_pulse,
        };
        if matches!(
            sensor.fault,
            IncrementalEncoderFault::StuckFromSequence { sequence: start } if sequence >= start
        ) {
            let previous = state.last_emitted.as_ref().ok_or(
                IncrementalEncoderError::StuckWithoutPrevious {
                    sensor_entity_index: sensor_entity.index(),
                },
            )?;
            payload.raw_count = previous.raw_count;
            payload.delta_count = previous.delta_count;
            payload.position_rad = previous.position_rad;
            payload.velocity_rad_s = previous.velocity_rad_s;
            payload.counter_wrapped = previous.counter_wrapped;
            payload.index_pulse = previous.index_pulse;
            payload.status = IncrementalEncoderStatus::StuckValue;
        }

        state.previous_ideal_count = Some(ideal_count);
        state.previous_raw_count = Some(raw_count);
        state.attempted_sequence = sequence;
        let dropped = matches!(
            sensor.fault,
            IncrementalEncoderFault::DropSequence { sequence: dropped } if sequence == dropped
        );
        let frame = if dropped {
            None
        } else {
            state.emitted_frames += 1;
            state.last_emitted = Some(payload);
            Some(
                Frame::new(sensor.stream_id, sensor_entity, sequence, sim_time, payload)
                    .with_latency(SimDuration::from_ticks(sensor.latency_ticks)),
            )
        };
        pending.push((sensor_entity, state, frame));
    }

    let mut published = 0;
    for (sensor_entity, state, frame) in pending {
        world.entity_mut(sensor_entity).insert(state);
        if let Some(frame) = frame {
            bus.publish(frame);
            published += 1;
        }
    }
    Ok(published)
}

fn completed_encoder_position(
    world: &World,
    sensor: &IncrementalEncoderSensor,
    sensor_entity: Entity,
) -> Result<f64, IncrementalEncoderError> {
    let actuator = world.get::<Actuator>(sensor.spec.actuator).ok_or(
        IncrementalEncoderError::MissingActuatorJoint {
            sensor_entity_index: sensor_entity.index(),
        },
    )?;
    let joint_entity = actuator
        .joint
        .ok_or(IncrementalEncoderError::MissingActuatorJoint {
            sensor_entity_index: sensor_entity.index(),
        })?;
    let position_rad = if let Some(state) = world.get::<JointState>(joint_entity).copied() {
        match state {
            JointState::Revolute { position_rad, .. } => position_rad,
            JointState::Prismatic { .. } | JointState::Fixed => {
                return Err(IncrementalEncoderError::NonRevoluteJoint {
                    sensor_entity_index: sensor_entity.index(),
                });
            }
        }
    } else {
        let joint = world.get::<Joint>(joint_entity).ok_or(
            IncrementalEncoderError::MissingActuatorJoint {
                sensor_entity_index: sensor_entity.index(),
            },
        )?;
        if !matches!(joint.kind, JointKind::Revolute | JointKind::Continuous) {
            return Err(IncrementalEncoderError::NonRevoluteJoint {
                sensor_entity_index: sensor_entity.index(),
            });
        }
        joint.position
    };
    if !position_rad.is_finite() {
        return Err(IncrementalEncoderError::InvalidPosition {
            sensor_entity_index: sensor_entity.index(),
        });
    }
    Ok(position_rad)
}

fn encoder_ideal_count(
    position_rad: f64,
    sensor: &IncrementalEncoderSensor,
    sensor_entity: Entity,
) -> Result<i64, IncrementalEncoderError> {
    let counts = (position_rad - sensor.spec.zero_offset_rad)
        * f64::from(sensor.spec.direction)
        * f64::from(sensor.spec.counts_per_revolution)
        / std::f64::consts::TAU;
    if !counts.is_finite() || counts < i64::MIN as f64 || counts >= i64::MAX as f64 {
        return Err(IncrementalEncoderError::InvalidPosition {
            sensor_entity_index: sensor_entity.index(),
        });
    }
    Ok(counts.trunc() as i64)
}

fn signed_counter_bounds(counter_bits: u8) -> (i64, i64, i128) {
    let half_span = 1_i64 << (counter_bits - 1);
    (-half_span, half_span - 1, i128::from(half_span) * 2)
}

fn finite_encoder_count(ideal_count: i64, sensor: &IncrementalEncoderSensor) -> (i64, bool) {
    let (minimum, maximum, span) = signed_counter_bounds(sensor.spec.counter_bits);
    match sensor.spec.overflow_behavior {
        IncrementalEncoderOverflowBehavior::Wrap => {
            let raw = (i128::from(ideal_count) - i128::from(minimum)).rem_euclid(span)
                + i128::from(minimum);
            (raw as i64, false)
        }
        IncrementalEncoderOverflowBehavior::Saturate => {
            let raw = ideal_count.clamp(minimum, maximum);
            (raw, raw != ideal_count)
        }
    }
}

fn observed_counter_delta(
    previous: i64,
    current: i64,
    sensor: &IncrementalEncoderSensor,
) -> (i64, bool) {
    let (_, _, span) = signed_counter_bounds(sensor.spec.counter_bits);
    let mut delta = i128::from(current) - i128::from(previous);
    let mut wrapped = false;
    if sensor.spec.overflow_behavior == IncrementalEncoderOverflowBehavior::Wrap {
        let half_span = span / 2;
        if delta > half_span {
            delta -= span;
            wrapped = true;
        } else if delta < -half_span {
            delta += span;
            wrapped = true;
        }
    }
    (delta as i64, wrapped)
}

fn encoder_velocity_rad_s(
    state: &IncrementalEncoderSensorState,
    sensor: &IncrementalEncoderSensor,
) -> f64 {
    let Some((first_ticks, first_count)) = state.velocity_history.first().copied() else {
        return 0.0;
    };
    let Some((last_ticks, last_count)) = state.velocity_history.last().copied() else {
        return 0.0;
    };
    let elapsed_ticks = last_ticks.saturating_sub(first_ticks);
    if elapsed_ticks == 0 {
        return 0.0;
    }
    let elapsed_s = SimDuration::from_ticks(elapsed_ticks).as_seconds().value();
    f64::from(sensor.spec.direction) * (last_count - first_count) as f64 * std::f64::consts::TAU
        / f64::from(sensor.spec.counts_per_revolution)
        / elapsed_s
}

fn crossed_encoder_index(
    previous_count: i64,
    current_count: i64,
    index_phase_rad: f64,
    sensor: &IncrementalEncoderSensor,
) -> bool {
    if previous_count == current_count {
        return false;
    }
    let period = i64::from(sensor.spec.counts_per_revolution);
    let index_count = (index_phase_rad
        * f64::from(sensor.spec.direction)
        * f64::from(sensor.spec.counts_per_revolution)
        / std::f64::consts::TAU)
        .round() as i64;
    if current_count > previous_count {
        (current_count - index_count).div_euclid(period)
            > (previous_count - index_count).div_euclid(period)
    } else {
        (index_count - current_count).div_euclid(period)
            > (index_count - previous_count).div_euclid(period)
    }
}

/// Joint-feedback sampling error.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum JointFeedbackError {
    /// Sensor configuration is invalid.
    #[error("invalid joint-feedback sensor on entity {sensor_entity_index}")]
    InvalidSensor {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// The sample schedule overflowed simulation ticks.
    #[error("joint-feedback schedule overflow on entity {sensor_entity_index}")]
    ScheduleOverflow {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
    /// A configured joint has no completed-step state.
    #[error("joint-feedback channel {joint_name} has no state on entity {joint_entity_index}")]
    MissingJointState {
        /// Stable joint name.
        joint_name: String,
        /// Stable joint entity index.
        joint_entity_index: u32,
    },
    /// Joint state contains a non-finite value.
    #[error("joint-feedback channel {joint_name} contains non-finite state")]
    InvalidJointState {
        /// Stable joint name.
        joint_name: String,
    },
    /// Actuation mode does not match the joint coordinate kind.
    #[error("joint-feedback channel {joint_name} has mismatched or invalid actuation")]
    InvalidActuation {
        /// Stable joint name.
        joint_name: String,
    },
    /// A backend effort value is non-finite or disagrees with the joint kind.
    #[error("joint-feedback channel {joint_name} has invalid realized effort")]
    InvalidEffortMeasurement {
        /// Stable joint name.
        joint_name: String,
    },
    /// A stuck-value fault has no prior emitted value to hold.
    #[error(
        "joint-feedback stuck-value fault on entity {sensor_entity_index} has no prior sample"
    )]
    StuckWithoutPrevious {
        /// Stable sensor entity index.
        sensor_entity_index: u32,
    },
}

/// Samples every typed joint-feedback sensor in deterministic entity order.
///
/// Processing order is fixed and externally observable: schedule, completed-step
/// coordinate read, command reconstruction and limiting, stuck-value substitution,
/// frame dropout, then output latency. Errors are validated for every due sensor
/// before any frame or runtime state is published, so a bad channel fails closed.
pub fn sample_joint_feedback_sensors(
    world: &mut World,
    sim_time: SimTime,
    bus: &mut impl DataBus,
) -> Result<usize, JointFeedbackError> {
    let mut sensors: Vec<(Entity, JointFeedbackSensor)> = world
        .iter_entities()
        .filter_map(|entity_ref| {
            entity_ref
                .get::<JointFeedbackSensor>()
                .cloned()
                .map(|sensor| (entity_ref.id(), sensor))
        })
        .collect();
    sensors.sort_unstable_by_key(|(entity, _)| entity.index());

    let mut pending = Vec::new();
    for (sensor_entity, sensor) in sensors {
        if !sensor.enabled {
            continue;
        }
        if !sensor.is_valid() || sensor.period().ticks() == 0 {
            return Err(JointFeedbackError::InvalidSensor {
                sensor_entity_index: sensor_entity.index(),
            });
        }
        let mut state = world
            .get::<JointFeedbackSensorState>(sensor_entity)
            .cloned()
            .unwrap_or_default();
        let scheduled_capture_ticks = sensor
            .period()
            .ticks()
            .checked_mul(state.attempted_sequence)
            .and_then(|ticks| sensor.phase_offset_ticks.checked_add(ticks))
            .ok_or(JointFeedbackError::ScheduleOverflow {
                sensor_entity_index: sensor_entity.index(),
            })?;
        if sim_time.ticks() < scheduled_capture_ticks {
            continue;
        }
        let sequence = state.attempted_sequence.checked_add(1).ok_or(
            JointFeedbackError::ScheduleOverflow {
                sensor_entity_index: sensor_entity.index(),
            },
        )?;
        let mut payload = build_joint_feedback(world, &sensor, scheduled_capture_ticks, sim_time)?;
        if matches!(
            sensor.fault,
            JointFeedbackFault::StuckFromSequence { sequence: start } if sequence >= start
        ) {
            let previous =
                state
                    .last_emitted
                    .as_ref()
                    .ok_or(JointFeedbackError::StuckWithoutPrevious {
                        sensor_entity_index: sensor_entity.index(),
                    })?;
            payload.joints.clone_from(&previous.joints);
            payload.status = JointFeedbackStatus::StuckValue;
        }

        state.attempted_sequence = sequence;
        let dropped = matches!(
            sensor.fault,
            JointFeedbackFault::DropSequence { sequence: dropped } if sequence == dropped
        );
        let frame = if dropped {
            None
        } else {
            state.emitted_frames += 1;
            state.last_emitted = Some(payload.clone());
            Some(
                Frame::new(sensor.stream_id, sensor_entity, sequence, sim_time, payload)
                    .with_latency(SimDuration::from_ticks(sensor.latency_ticks)),
            )
        };
        pending.push((sensor_entity, state, frame));
    }

    let mut published = 0;
    for (sensor_entity, state, frame) in pending {
        world.entity_mut(sensor_entity).insert(state);
        if let Some(frame) = frame {
            bus.publish(frame);
            published += 1;
        }
    }
    Ok(published)
}

fn build_joint_feedback(
    world: &World,
    sensor: &JointFeedbackSensor,
    scheduled_capture_ticks: u64,
    sim_time: SimTime,
) -> Result<JointFeedback, JointFeedbackError> {
    let joints = sensor
        .channels
        .iter()
        .map(|channel| {
            let state = world
                .get::<JointState>(channel.joint_entity)
                .copied()
                .ok_or_else(|| JointFeedbackError::MissingJointState {
                    joint_name: channel.name.clone(),
                    joint_entity_index: channel.joint_entity.index(),
                })?;
            let actuation = world
                .get::<JointActuation>(channel.joint_entity)
                .copied()
                .unwrap_or_default();
            let (coordinate, command) =
                joint_coordinate_and_command(&channel.name, state, actuation)?;
            let effort = joint_effort_feedback(
                &channel.name,
                state,
                world
                    .get::<JointEffortMeasurement>(channel.joint_entity)
                    .copied(),
            )?;
            Ok(JointFeedbackChannel {
                name: channel.name.clone(),
                coordinate,
                command,
                effort,
            })
        })
        .collect::<Result<Vec<_>, JointFeedbackError>>()?;
    Ok(JointFeedback {
        schema_version: JointFeedback::SCHEMA_VERSION,
        scheduled_capture_ticks,
        sample_phase_error_ticks: sim_time.ticks() - scheduled_capture_ticks,
        status: JointFeedbackStatus::Nominal,
        joints,
    })
}

fn joint_effort_feedback(
    joint_name: &str,
    state: JointState,
    measurement: Option<JointEffortMeasurement>,
) -> Result<JointEffortFeedback, JointFeedbackError> {
    let Some(measurement) = measurement else {
        return Ok(JointEffortFeedback::Unavailable);
    };
    if !measurement.has_valid_value() {
        return Err(JointFeedbackError::InvalidEffortMeasurement {
            joint_name: joint_name.to_owned(),
        });
    }
    match (state, measurement) {
        (JointState::Revolute { .. }, JointEffortMeasurement::Revolute { measured_effort_nm }) => {
            Ok(JointEffortFeedback::Revolute { measured_effort_nm })
        }
        (JointState::Prismatic { .. }, JointEffortMeasurement::Prismatic { measured_force_n }) => {
            Ok(JointEffortFeedback::Prismatic { measured_force_n })
        }
        _ => Err(JointFeedbackError::InvalidEffortMeasurement {
            joint_name: joint_name.to_owned(),
        }),
    }
}

fn joint_coordinate_and_command(
    joint_name: &str,
    state: JointState,
    actuation: JointActuation,
) -> Result<(JointCoordinateFeedback, JointCommandFeedback), JointFeedbackError> {
    if !actuation.has_valid_values() {
        return Err(JointFeedbackError::InvalidActuation {
            joint_name: joint_name.to_owned(),
        });
    }
    match state {
        JointState::Revolute {
            position_rad,
            velocity_rad_s,
        } => {
            if !position_rad.is_finite() || !velocity_rad_s.is_finite() {
                return Err(JointFeedbackError::InvalidJointState {
                    joint_name: joint_name.to_owned(),
                });
            }
            let command = revolute_command(joint_name, position_rad, velocity_rad_s, actuation)?;
            Ok((
                JointCoordinateFeedback::Revolute {
                    position_rad,
                    velocity_rad_s,
                },
                command,
            ))
        }
        JointState::Prismatic {
            position_m,
            velocity_m_s,
        } => {
            if !position_m.is_finite() || !velocity_m_s.is_finite() {
                return Err(JointFeedbackError::InvalidJointState {
                    joint_name: joint_name.to_owned(),
                });
            }
            let command = prismatic_command(joint_name, position_m, velocity_m_s, actuation)?;
            Ok((
                JointCoordinateFeedback::Prismatic {
                    position_m,
                    velocity_m_s,
                },
                command,
            ))
        }
        JointState::Fixed if actuation == JointActuation::Disabled => Ok((
            JointCoordinateFeedback::Fixed,
            JointCommandFeedback::Disabled,
        )),
        JointState::Fixed => Err(JointFeedbackError::InvalidActuation {
            joint_name: joint_name.to_owned(),
        }),
    }
}

fn revolute_command(
    joint_name: &str,
    position_rad: f64,
    velocity_rad_s: f64,
    actuation: JointActuation,
) -> Result<JointCommandFeedback, JointFeedbackError> {
    let (mode, target_position_rad, target_velocity_rad_s, request, limit) = match actuation {
        JointActuation::Disabled => return Ok(JointCommandFeedback::Disabled),
        JointActuation::RevolutePosition {
            target_position_rad,
            stiffness_nm_per_rad,
            damping_nm_s_per_rad,
            max_effort_nm,
        } => (
            JointCommandMode::Position,
            Some(target_position_rad),
            None,
            stiffness_nm_per_rad * (target_position_rad - position_rad)
                - damping_nm_s_per_rad * velocity_rad_s,
            max_effort_nm,
        ),
        JointActuation::RevoluteVelocity {
            target_velocity_rad_s,
            gain_nm_s_per_rad,
            max_effort_nm,
        } => (
            JointCommandMode::Velocity,
            None,
            Some(target_velocity_rad_s),
            gain_nm_s_per_rad * (target_velocity_rad_s - velocity_rad_s),
            max_effort_nm,
        ),
        JointActuation::RevoluteEffort {
            effort_nm,
            max_effort_nm,
        } => (
            JointCommandMode::Effort,
            None,
            None,
            effort_nm,
            max_effort_nm,
        ),
        _ => {
            return Err(JointFeedbackError::InvalidActuation {
                joint_name: joint_name.to_owned(),
            });
        }
    };
    let limited = request.clamp(-limit, limit);
    Ok(JointCommandFeedback::Revolute {
        mode,
        target_position_rad,
        target_velocity_rad_s,
        unconstrained_effort_request_nm: request,
        limited_effort_command_nm: limited,
        effort_limit_nm: limit,
        saturated: limited != request,
    })
}

fn prismatic_command(
    joint_name: &str,
    position_m: f64,
    velocity_m_s: f64,
    actuation: JointActuation,
) -> Result<JointCommandFeedback, JointFeedbackError> {
    let (mode, target_position_m, target_velocity_m_s, request, limit) = match actuation {
        JointActuation::Disabled => return Ok(JointCommandFeedback::Disabled),
        JointActuation::PrismaticPosition {
            target_position_m,
            stiffness_n_per_m,
            damping_n_s_per_m,
            max_force_n,
        } => (
            JointCommandMode::Position,
            Some(target_position_m),
            None,
            stiffness_n_per_m * (target_position_m - position_m) - damping_n_s_per_m * velocity_m_s,
            max_force_n,
        ),
        JointActuation::PrismaticVelocity {
            target_velocity_m_s,
            gain_n_s_per_m,
            max_force_n,
        } => (
            JointCommandMode::Velocity,
            None,
            Some(target_velocity_m_s),
            gain_n_s_per_m * (target_velocity_m_s - velocity_m_s),
            max_force_n,
        ),
        JointActuation::PrismaticEffort {
            force_n,
            max_force_n,
        } => (JointCommandMode::Effort, None, None, force_n, max_force_n),
        _ => {
            return Err(JointFeedbackError::InvalidActuation {
                joint_name: joint_name.to_owned(),
            });
        }
    };
    let limited = request.clamp(-limit, limit);
    Ok(JointCommandFeedback::Prismatic {
        mode,
        target_position_m,
        target_velocity_m_s,
        unconstrained_force_request_n: request,
        limited_force_command_n: limited,
        force_limit_n: limit,
        saturated: limited != request,
    })
}

/// Trait for sensor backends used by higher-level schedulers.
pub trait SensorSampler {
    /// Returns true if the sensor should emit on this tick.
    fn should_sample(&self, period: SimDuration, last_sample: SimTime, now: SimTime) -> bool;
}

impl SensorSampler for Sensor {
    fn should_sample(&self, period: SimDuration, last_sample: SimTime, now: SimTime) -> bool {
        if period.ticks() == 0 {
            return false;
        }
        if last_sample == SimTime::ZERO {
            return true;
        }
        now.ticks().saturating_sub(last_sample.ticks()) >= period.ticks()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::CameraSpec;
    use crate::imu::ImuSpec;
    use crate::noise::NoiseModel;
    use crate::Sensor;
    use rne_data::{InMemoryDataBus, StreamId};
    use rne_ecs::spawn_named;
    use rne_math::{Quat, Seconds, Vec3};
    use rne_physics::{
        ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
        PhysicsWorldId, RaycastHit, RaycastQuery, RigidBody,
    };
    use rne_robot::{ActuatorLimits, ActuatorTarget, ControlMode, JointLimits};

    struct NullPhysics;

    impl PhysicsBackend for NullPhysics {
        type BodyHandle = ();
        type ColliderHandle = ();

        fn create_world(&mut self, _: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
            Ok(PhysicsWorldId::DEFAULT)
        }
        fn sync_from_ecs(
            &mut self,
            _: &mut rne_ecs::World,
            _: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn step(&mut self, _: PhysicsWorldId, _: SimDuration) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn sync_to_ecs(
            &mut self,
            _: &mut rne_ecs::World,
            _: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn raycast(
            &self,
            _: PhysicsWorldId,
            _: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            Ok(Vec::new())
        }
        fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&[])
        }
        fn capabilities(&self) -> &[PhysicsCapability] {
            &[]
        }
    }

    struct LidarHitPhysics {
        target: rne_ecs::Entity,
    }

    impl PhysicsBackend for LidarHitPhysics {
        type BodyHandle = ();
        type ColliderHandle = ();

        fn create_world(&mut self, _: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
            Ok(PhysicsWorldId::DEFAULT)
        }
        fn sync_from_ecs(
            &mut self,
            _: &mut rne_ecs::World,
            _: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn step(&mut self, _: PhysicsWorldId, _: SimDuration) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn sync_to_ecs(
            &mut self,
            _: &mut rne_ecs::World,
            _: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }
        fn raycast(
            &self,
            _: PhysicsWorldId,
            query: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            Ok(vec![RaycastHit {
                entity: self.target,
                point_m: query.origin_m + query.direction * 5.0,
                normal: -query.direction,
                distance_m: 5.0,
            }])
        }
        fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&[])
        }
        fn capabilities(&self) -> &[PhysicsCapability] {
            &[]
        }
    }

    #[test]
    fn sensor_emits_at_configured_rate() {
        let mut world = World::new();
        let sensor_entity = spawn_named(&mut world, "imu");
        world.entity_mut(sensor_entity).insert((
            Sensor {
                kind: SensorKind::Imu(ImuSpec {
                    noise: NoiseModel::default(),
                    seed: 1,
                    ..ImuSpec::default()
                }),
                update_rate_hz: 10.0,
                latency_ticks: 0,
                frame_id: 1,
                enabled: true,
                stream_id: StreamId::new(1),
            },
            SensorState::default(),
            Transform3::default(),
        ));

        let mut bus = InMemoryDataBus::new();
        let physics = NullPhysics;

        for tick in 0..60 {
            let sim_time = SimTime::from_seconds(Seconds::new(tick as f64 / 60.0));
            sample_sensors(
                &mut SensorSampleContext {
                    world: &mut world,
                    sim_time,
                    physics: &physics,
                    physics_world: PhysicsWorldId::DEFAULT,
                    render: None,
                    scene: None,
                },
                &mut bus,
            );
        }

        assert_eq!(bus.frame_count(StreamId::new(1)), 10);
    }

    #[test]
    fn camera_sensor_publishes_image() {
        let mut world = World::new();
        let sensor_entity = spawn_named(&mut world, "camera");
        world.entity_mut(sensor_entity).insert((
            Sensor {
                kind: SensorKind::Camera(CameraSpec {
                    width: 8,
                    height: 8,
                    ..CameraSpec::default()
                }),
                update_rate_hz: 10.0,
                latency_ticks: 0,
                frame_id: 2,
                enabled: true,
                stream_id: StreamId::new(2),
            },
            SensorState::default(),
            Transform3::default(),
        ));

        let mut bus = InMemoryDataBus::new();
        let physics = NullPhysics;
        sample_sensors(
            &mut SensorSampleContext {
                world: &mut world,
                sim_time: SimTime::from_seconds(Seconds::new(0.0)),
                physics: &physics,
                physics_world: PhysicsWorldId::DEFAULT,
                render: None,
                scene: None,
            },
            &mut bus,
        );

        let image = bus.latest::<rne_data::ImageRgb8>(StreamId::new(2)).unwrap();
        assert_eq!(image.payload.width, 8);
        assert_eq!(image.payload.rgba8.len(), 8 * 8 * 4);
        let depth = bus
            .latest::<rne_data::ImageDepth>(StreamId::new(52))
            .unwrap();
        assert_eq!(depth.payload.width, 8);
    }

    #[test]
    fn lidar_frame_has_material_attributes_and_explicit_latency() {
        let mut world = World::new();
        world.insert_resource(WorldRandom::new(42));
        let target = spawn_named(&mut world, "painted_target");
        world
            .entity_mut(target)
            .insert(crate::LidarMaterial::painted_metal());
        let sensor_entity = spawn_named(&mut world, "physics_lidar");
        world.entity_mut(sensor_entity).insert((
            Sensor {
                kind: SensorKind::Lidar(crate::LidarSpec {
                    ray_count: 1,
                    min_angle_rad: 0.0,
                    max_angle_rad: 0.0,
                    seed: 9,
                    ..crate::LidarSpec::default()
                }),
                update_rate_hz: 10.0,
                latency_ticks: 25,
                frame_id: 3,
                enabled: true,
                stream_id: StreamId::new(78),
            },
            SensorState::default(),
            Transform3::IDENTITY,
        ));

        let mut bus = InMemoryDataBus::new();
        let physics = LidarHitPhysics { target };
        sample_sensors(
            &mut SensorSampleContext {
                world: &mut world,
                sim_time: SimTime::from_ticks(100),
                physics: &physics,
                physics_world: PhysicsWorldId::DEFAULT,
                render: None,
                scene: None,
            },
            &mut bus,
        );

        let frame = bus
            .latest::<rne_data::PointCloud>(StreamId::new(78))
            .expect("LiDAR frame");
        assert_eq!(frame.capture_time, SimTime::from_ticks(100));
        assert_eq!(frame.available_time, SimTime::from_ticks(125));
        assert_eq!(frame.payload.points_m.len(), 1);
        assert!(frame.payload.intensities[0] > 0.0);
        assert_eq!(frame.payload.ray_indices, vec![0]);
        assert_eq!(frame.payload.return_indices, vec![1]);
        assert!(frame.payload.attributes_are_aligned());
    }

    #[test]
    fn imu_noise_changes_by_sample_sequence() {
        let mut world = World::new();
        world.insert_resource(WorldRandom::new(123));
        let sensor_entity = spawn_named(&mut world, "imu");
        world.entity_mut(sensor_entity).insert((
            Sensor {
                kind: SensorKind::Imu(ImuSpec {
                    noise: NoiseModel {
                        angular_stddev_rad_s: 0.1,
                        linear_stddev_m_s2: 0.2,
                        linear_bias_m_s2: rne_math::Vec3::ZERO,
                    },
                    seed: 9,
                    ..ImuSpec::default()
                }),
                update_rate_hz: 60.0,
                latency_ticks: 0,
                frame_id: 1,
                enabled: true,
                stream_id: StreamId::new(77),
            },
            SensorState::default(),
            Transform3::default(),
        ));

        let mut bus = InMemoryDataBus::new();
        let physics = NullPhysics;
        sample_sensors(
            &mut SensorSampleContext {
                world: &mut world,
                sim_time: SimTime::from_ticks(0),
                physics: &physics,
                physics_world: PhysicsWorldId::DEFAULT,
                render: None,
                scene: None,
            },
            &mut bus,
        );
        let first = bus
            .latest::<rne_data::ImuSample>(StreamId::new(77))
            .unwrap();

        sample_sensors(
            &mut SensorSampleContext {
                world: &mut world,
                sim_time: SimTime::from_ticks(16_666_666),
                physics: &physics,
                physics_world: PhysicsWorldId::DEFAULT,
                render: None,
                scene: None,
            },
            &mut bus,
        );
        let second = bus
            .latest::<rne_data::ImuSample>(StreamId::new(77))
            .unwrap();

        assert_ne!(
            first.payload.linear_acceleration_m_s2,
            second.payload.linear_acceleration_m_s2
        );
    }

    fn imu_feedback_fixture(
        fault: ImuFeedbackFault,
    ) -> (World, Entity, Entity, StreamId, InMemoryDataBus) {
        let mut world = World::new();
        world.insert_resource(WorldRandom::new(123));
        let body = spawn_named(&mut world, "imu_body");
        world.entity_mut(body).insert((
            Transform3::IDENTITY,
            RigidBody {
                angular_velocity_rad_s: Vec3::new(0.0, 0.0, 0.5),
                ..RigidBody::default()
            },
        ));
        let sensor = spawn_named(&mut world, "typed_imu");
        let stream = StreamId::new(79);
        world.entity_mut(sensor).insert((
            ImuMount {
                body_entity: body,
                body_from_sensor: Transform3::from_translation_rotation(
                    Vec3::new(0.1, 0.0, 0.0),
                    Quat::IDENTITY,
                ),
            },
            ImuFeedbackSensor {
                spec: ImuSpec::default(),
                update_rate_hz: 1_000_000.0,
                sample_period_ticks: Some(1_000),
                phase_offset_ticks: 5,
                latency_ticks: 7,
                enabled: true,
                stream_id: stream,
                fault,
            },
            ImuFeedbackSensorState::default(),
        ));
        (world, body, sensor, stream, InMemoryDataBus::new())
    }

    #[test]
    fn imu_feedback_exposes_mount_schedule_latency_units_and_status() {
        let (mut world, _, _, stream, mut bus) = imu_feedback_fixture(ImuFeedbackFault::None);
        assert_eq!(
            sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(4), &mut bus).unwrap(),
            0
        );
        assert_eq!(
            sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap(),
            1
        );

        let frame = bus.latest::<ImuFeedback>(stream).expect("IMU feedback");
        assert_eq!(frame.sequence, 1);
        assert_eq!(frame.capture_time.ticks(), 5);
        assert_eq!(frame.available_time.ticks(), 12);
        assert_eq!(frame.payload.schema_version, ImuFeedback::SCHEMA_VERSION);
        assert_eq!(frame.payload.scheduled_capture_ticks, 5);
        assert_eq!(frame.payload.sample_phase_error_ticks, 0);
        assert_eq!(frame.payload.status, ImuFeedbackStatus::Nominal);
        assert_eq!(
            frame.payload.angular_velocity_rad_s,
            Vec3::new(0.0, 0.0, 0.5)
        );
        assert_eq!(
            frame.payload.specific_force_m_s2,
            Vec3::new(-0.025, 9.81, 0.0)
        );
        assert!(bus
            .latest_available::<ImuFeedback>(stream, SimTime::from_ticks(11))
            .is_none());
        assert!(bus
            .latest_available::<ImuFeedback>(stream, SimTime::from_ticks(12))
            .is_some());
    }

    #[test]
    fn imu_feedback_drop_creates_a_gap_but_advances_physical_state() {
        let (mut world, body, sensor, stream, mut bus) =
            imu_feedback_fixture(ImuFeedbackFault::DropSequence { sequence: 2 });
        sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        world
            .get_mut::<RigidBody>(body)
            .unwrap()
            .linear_velocity_m_s = Vec3::X;
        assert_eq!(
            sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(1_005), &mut bus).unwrap(),
            0
        );
        let state_after_drop = world
            .get::<ImuFeedbackSensorState>(sensor)
            .expect("IMU sensor state");
        assert_eq!(state_after_drop.attempted_sequence, 2);
        assert_eq!(state_after_drop.emitted_frames, 1);
        assert_eq!(
            state_after_drop.imu_state.previous_linear_velocity_m_s,
            Vec3::X
        );

        assert_eq!(
            sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(2_005), &mut bus).unwrap(),
            1
        );
        assert_eq!(bus.frame_count(stream), 2);
        assert_eq!(bus.latest::<ImuFeedback>(stream).unwrap().sequence, 3);
    }

    #[test]
    fn imu_feedback_stuck_fault_holds_values_but_advances_kinematics() {
        let (mut world, body, sensor, stream, mut bus) =
            imu_feedback_fixture(ImuFeedbackFault::StuckFromSequence { sequence: 2 });
        sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        world
            .get_mut::<RigidBody>(body)
            .unwrap()
            .angular_velocity_rad_s = Vec3::Z;
        sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(1_005), &mut bus).unwrap();

        let frame = bus.latest::<ImuFeedback>(stream).expect("stuck IMU frame");
        assert_eq!(frame.sequence, 2);
        assert_eq!(frame.payload.status, ImuFeedbackStatus::StuckValue);
        assert_eq!(
            frame.payload.angular_velocity_rad_s,
            Vec3::new(0.0, 0.0, 0.5)
        );
        assert_eq!(
            world
                .get::<ImuFeedbackSensorState>(sensor)
                .unwrap()
                .kinematic_state
                .previous_angular_velocity_rad_s,
            Vec3::Z
        );
    }

    #[test]
    fn missing_imu_mount_fails_all_due_publication_and_state_closed() {
        let (mut world, _, valid_sensor, valid_stream, mut bus) =
            imu_feedback_fixture(ImuFeedbackFault::None);
        let invalid_sensor = spawn_named(&mut world, "unmounted_imu");
        world.entity_mut(invalid_sensor).insert(ImuFeedbackSensor {
            spec: ImuSpec::default(),
            update_rate_hz: 1_000_000.0,
            sample_period_ticks: Some(1_000),
            phase_offset_ticks: 5,
            latency_ticks: 0,
            enabled: true,
            stream_id: StreamId::new(80),
            fault: ImuFeedbackFault::None,
        });

        let error = sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus)
            .expect_err("missing mount must fail closed");
        assert_eq!(
            error,
            ImuFeedbackError::MissingMount {
                sensor_entity_index: invalid_sensor.index()
            }
        );
        assert_eq!(bus.frame_count(valid_stream), 0);
        assert_eq!(
            world
                .get::<ImuFeedbackSensorState>(valid_sensor)
                .unwrap()
                .attempted_sequence,
            0
        );
    }

    fn incremental_encoder_fixture(
        fault: IncrementalEncoderFault,
        overflow_behavior: IncrementalEncoderOverflowBehavior,
    ) -> (World, Entity, Entity, StreamId, InMemoryDataBus) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let parent = spawn_named(&mut world, "base");
        let joint = world
            .spawn((
                Joint {
                    robot,
                    parent_link: parent,
                    child_link: Entity::PLACEHOLDER,
                    kind: JointKind::Continuous,
                    limits: JointLimits::default(),
                    axis: Vec3::Y,
                    position: 0.0,
                    velocity: 999.0,
                },
                JointState::Revolute {
                    position_rad: 0.0,
                    velocity_rad_s: 999.0,
                },
            ))
            .id();
        world.get_mut::<Joint>(joint).unwrap().child_link = joint;
        let actuator = world
            .spawn(Actuator {
                robot,
                joint: Some(joint),
                name: "wheel_motor".into(),
                mode: ControlMode::Velocity,
                target: ActuatorTarget::default(),
                limits: ActuatorLimits::default(),
            })
            .id();
        let sensor_entity = spawn_named(&mut world, "incremental_encoder");
        let stream = StreamId::new(87);
        world.entity_mut(sensor_entity).insert((
            IncrementalEncoderSensor {
                spec: crate::IncrementalEncoderSpec {
                    actuator,
                    counts_per_revolution: 16,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    counter_bits: 4,
                    overflow_behavior,
                    velocity_window_samples: 2,
                    index_phase_rad: Some(0.0),
                },
                update_rate_hz: 1.0,
                sample_period_ticks: Some(1_000_000_000),
                phase_offset_ticks: 5,
                latency_ticks: 7,
                enabled: true,
                stream_id: stream,
                fault,
            },
            IncrementalEncoderSensorState::default(),
        ));
        (world, joint, sensor_entity, stream, InMemoryDataBus::new())
    }

    fn set_encoder_position(world: &mut World, joint: Entity, count: i64) {
        world.entity_mut(joint).insert(JointState::Revolute {
            position_rad: count as f64 * std::f64::consts::TAU / 16.0,
            velocity_rad_s: -1234.0,
        });
    }

    #[test]
    fn incremental_encoder_uses_integer_counts_and_capture_time_not_truth_velocity() {
        let (mut world, joint, _, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::None,
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        assert_eq!(
            sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(4), &mut bus)
                .unwrap(),
            0
        );
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        let first = bus
            .latest::<IncrementalEncoderFeedback>(stream)
            .expect("initial encoder frame");
        assert_eq!(first.payload.status, IncrementalEncoderStatus::Initializing);
        assert_eq!(first.payload.velocity_rad_s, 0.0);
        assert_eq!(first.capture_time.ticks(), 5);
        assert_eq!(first.available_time.ticks(), 12);

        set_encoder_position(&mut world, joint, 3);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_005),
            &mut bus,
        )
        .unwrap();
        let second = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(second.payload.raw_count, 3);
        assert_eq!(second.payload.delta_count, 3);
        assert_eq!(second.payload.status, IncrementalEncoderStatus::Nominal);
        let expected_velocity = 3.0 * std::f64::consts::TAU / 16.0;
        assert!((second.payload.velocity_rad_s - expected_velocity).abs() < 1.0e-12);
        assert_ne!(second.payload.velocity_rad_s, -1234.0);

        set_encoder_position(&mut world, joint, 5);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(2_000_000_005),
            &mut bus,
        )
        .unwrap();
        let third = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        let expected_windowed_velocity = 2.5 * std::f64::consts::TAU / 16.0;
        assert!((third.payload.velocity_rad_s - expected_windowed_velocity).abs() < 1.0e-12);
        assert_eq!(third.payload.scheduled_capture_ticks, 2_000_000_005);
        assert_eq!(third.payload.sample_phase_error_ticks, 0);
    }

    #[test]
    fn incremental_encoder_resolves_signed_wrap_and_index_crossing() {
        let (mut world, joint, _, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::None,
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        set_encoder_position(&mut world, joint, 7);
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        set_encoder_position(&mut world, joint, 8);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_005),
            &mut bus,
        )
        .unwrap();
        let wrapped = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(wrapped.payload.raw_count, -8);
        assert_eq!(wrapped.payload.delta_count, 1);
        assert!(wrapped.payload.counter_wrapped);

        set_encoder_position(&mut world, joint, 15);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(2_000_000_005),
            &mut bus,
        )
        .unwrap();
        set_encoder_position(&mut world, joint, 16);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(3_000_000_005),
            &mut bus,
        )
        .unwrap();
        assert!(
            bus.latest::<IncrementalEncoderFeedback>(stream)
                .unwrap()
                .payload
                .index_pulse
        );
    }

    #[test]
    fn incremental_encoder_saturation_is_explicit_and_stops_count_velocity() {
        let (mut world, joint, _, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::None,
            IncrementalEncoderOverflowBehavior::Saturate,
        );
        set_encoder_position(&mut world, joint, 9);
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        let first = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(first.payload.raw_count, 7);
        assert_eq!(
            first.payload.status,
            IncrementalEncoderStatus::CounterSaturated
        );
        set_encoder_position(&mut world, joint, 10);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_005),
            &mut bus,
        )
        .unwrap();
        let second = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(second.payload.delta_count, 0);
        assert_eq!(second.payload.velocity_rad_s, 0.0);
    }

    #[test]
    fn incremental_encoder_applies_direction_zero_calibration_and_low_speed_quantization() {
        let (mut world, joint, sensor, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::None,
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        {
            let mut encoder = world.get_mut::<IncrementalEncoderSensor>(sensor).unwrap();
            encoder.spec.direction = -1;
            encoder.spec.zero_offset_rad = std::f64::consts::TAU / 16.0;
        }
        set_encoder_position(&mut world, joint, 3);
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(8), &mut bus).unwrap();
        let calibrated = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(calibrated.payload.raw_count, -2);
        assert!(
            (calibrated.payload.position_rad - 3.0 * std::f64::consts::TAU / 16.0).abs() < 1.0e-12
        );
        assert_eq!(calibrated.payload.scheduled_capture_ticks, 5);
        assert_eq!(calibrated.payload.sample_phase_error_ticks, 3);

        world.entity_mut(joint).insert(JointState::Revolute {
            position_rad: 3.5 * std::f64::consts::TAU / 16.0,
            velocity_rad_s: 500.0,
        });
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_008),
            &mut bus,
        )
        .unwrap();
        let below_one_edge = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(below_one_edge.payload.delta_count, 0);
        assert_eq!(below_one_edge.payload.velocity_rad_s, 0.0);
    }

    #[test]
    fn incremental_encoder_drop_and_stuck_faults_preserve_internal_edge_history() {
        let (mut world, joint, sensor, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::DropSequence { sequence: 2 },
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        set_encoder_position(&mut world, joint, 1);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_005),
            &mut bus,
        )
        .unwrap();
        set_encoder_position(&mut world, joint, 2);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(2_000_000_005),
            &mut bus,
        )
        .unwrap();
        assert_eq!(bus.frame_count(stream), 2);
        assert_eq!(
            bus.latest::<IncrementalEncoderFeedback>(stream)
                .unwrap()
                .sequence,
            3
        );
        let state = world.get::<IncrementalEncoderSensorState>(sensor).unwrap();
        assert_eq!(state.attempted_sequence, 3);
        assert_eq!(state.emitted_frames, 2);
        assert_eq!(state.observed_accumulated_count, 2);

        let (mut world, joint, _, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::StuckFromSequence { sequence: 2 },
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        set_encoder_position(&mut world, joint, 4);
        sample_incremental_encoder_sensors(
            &mut world,
            SimTime::from_ticks(1_000_000_005),
            &mut bus,
        )
        .unwrap();
        let stuck = bus.latest::<IncrementalEncoderFeedback>(stream).unwrap();
        assert_eq!(stuck.payload.raw_count, 0);
        assert_eq!(stuck.payload.status, IncrementalEncoderStatus::StuckValue);
    }

    #[test]
    fn invalid_incremental_encoder_fails_all_due_sensors_atomically() {
        let (mut world, _, valid_sensor, stream, mut bus) = incremental_encoder_fixture(
            IncrementalEncoderFault::None,
            IncrementalEncoderOverflowBehavior::Wrap,
        );
        let invalid_sensor = spawn_named(&mut world, "invalid_encoder");
        let invalid_actuator = Entity::from_raw(u32::MAX);
        world.entity_mut(invalid_sensor).insert((
            IncrementalEncoderSensor {
                spec: crate::IncrementalEncoderSpec {
                    actuator: invalid_actuator,
                    counts_per_revolution: 16,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    counter_bits: 16,
                    overflow_behavior: IncrementalEncoderOverflowBehavior::Wrap,
                    velocity_window_samples: 1,
                    index_phase_rad: None,
                },
                update_rate_hz: 1.0,
                sample_period_ticks: Some(1_000_000_000),
                phase_offset_ticks: 5,
                latency_ticks: 0,
                enabled: true,
                stream_id: StreamId::new(99),
                fault: IncrementalEncoderFault::None,
            },
            IncrementalEncoderSensorState::default(),
        ));
        assert!(matches!(
            sample_incremental_encoder_sensors(&mut world, SimTime::from_ticks(5), &mut bus),
            Err(IncrementalEncoderError::MissingActuatorJoint { .. })
        ));
        assert_eq!(bus.frame_count(stream), 0);
        assert_eq!(
            world
                .get::<IncrementalEncoderSensorState>(valid_sensor)
                .unwrap()
                .attempted_sequence,
            0
        );
    }

    fn joint_feedback_fixture(
        fault: JointFeedbackFault,
    ) -> (World, Entity, Entity, StreamId, InMemoryDataBus) {
        let mut world = World::new();
        let joint = spawn_named(&mut world, "shoulder_joint");
        world.entity_mut(joint).insert((
            JointState::Revolute {
                position_rad: 0.1,
                velocity_rad_s: 0.2,
            },
            JointActuation::RevolutePosition {
                target_position_rad: 1.0,
                stiffness_nm_per_rad: 20.0,
                damping_nm_s_per_rad: 2.0,
                max_effort_nm: 5.0,
            },
        ));
        let sensor_entity = spawn_named(&mut world, "joint_feedback");
        let stream = StreamId::new(88);
        world.entity_mut(sensor_entity).insert((
            JointFeedbackSensor {
                update_rate_hz: 1_000.0,
                sample_period_ticks: None,
                phase_offset_ticks: 5,
                latency_ticks: 7,
                enabled: true,
                stream_id: stream,
                channels: vec![crate::JointFeedbackChannelSpec {
                    name: "shoulder_joint".into(),
                    joint_entity: joint,
                }],
                fault,
            },
            JointFeedbackSensorState::default(),
        ));
        (world, joint, sensor_entity, stream, InMemoryDataBus::new())
    }

    #[test]
    fn joint_feedback_exposes_schedule_latency_units_and_saturation() {
        let (mut world, _, _, stream, mut bus) = joint_feedback_fixture(JointFeedbackFault::None);
        assert_eq!(
            sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(4), &mut bus).unwrap(),
            0
        );
        assert_eq!(
            sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap(),
            1
        );
        let frame = bus.latest::<JointFeedback>(stream).expect("joint feedback");
        assert_eq!(frame.sequence, 1);
        assert_eq!(frame.payload.schema_version, JointFeedback::SCHEMA_VERSION);
        assert_eq!(frame.capture_time.ticks(), 5);
        assert_eq!(frame.available_time.ticks(), 12);
        assert!(bus
            .latest_available::<JointFeedback>(stream, SimTime::from_ticks(11))
            .is_none());
        assert!(bus
            .latest_available::<JointFeedback>(stream, SimTime::from_ticks(12))
            .is_some());
        assert_eq!(frame.payload.scheduled_capture_ticks, 5);
        assert_eq!(frame.payload.sample_phase_error_ticks, 0);
        assert_eq!(frame.payload.status, JointFeedbackStatus::Nominal);
        assert!(matches!(
            frame.payload.joints[0].coordinate,
            JointCoordinateFeedback::Revolute {
                position_rad: 0.1,
                velocity_rad_s: 0.2
            }
        ));
        assert!(matches!(
            frame.payload.joints[0].command,
            JointCommandFeedback::Revolute {
                unconstrained_effort_request_nm: 17.6,
                limited_effort_command_nm: 5.0,
                effort_limit_nm: 5.0,
                saturated: true,
                ..
            }
        ));
        assert_eq!(
            frame.payload.joints[0].effort,
            JointEffortFeedback::Unavailable
        );
    }

    #[test]
    fn joint_feedback_preserves_measured_effort_and_rejects_wrong_units() {
        let (mut world, joint, _, stream, mut bus) =
            joint_feedback_fixture(JointFeedbackFault::None);
        world
            .entity_mut(joint)
            .insert(JointEffortMeasurement::Revolute {
                measured_effort_nm: -1.25,
            });
        sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        let frame = bus.latest::<JointFeedback>(stream).expect("joint feedback");
        assert_eq!(
            frame.payload.joints[0].effort,
            JointEffortFeedback::Revolute {
                measured_effort_nm: -1.25
            }
        );
        assert_eq!(frame.capture_time.ticks(), 5);
        assert_eq!(frame.available_time.ticks(), 12);

        world
            .entity_mut(joint)
            .insert(JointEffortMeasurement::Prismatic {
                measured_force_n: 2.0,
            });
        assert!(matches!(
            sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(1_000_005), &mut bus),
            Err(JointFeedbackError::InvalidEffortMeasurement { .. })
        ));
        assert_eq!(
            bus.frame_count(stream),
            1,
            "invalid effort must fail closed"
        );
    }

    #[test]
    fn joint_feedback_drop_creates_a_sequence_gap() {
        let (mut world, _, sensor, stream, mut bus) =
            joint_feedback_fixture(JointFeedbackFault::DropSequence { sequence: 2 });
        for ticks in [5, 1_000_005, 2_000_005] {
            sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(ticks), &mut bus)
                .unwrap();
        }
        assert_eq!(bus.frame_count(stream), 2);
        assert_eq!(bus.latest::<JointFeedback>(stream).unwrap().sequence, 3);
        let state = world
            .get::<JointFeedbackSensorState>(sensor)
            .expect("sensor state");
        assert_eq!(state.attempted_sequence, 3);
        assert_eq!(state.emitted_frames, 2);
    }

    #[test]
    fn joint_feedback_stuck_fault_holds_the_previous_value() {
        let (mut world, joint, _, stream, mut bus) =
            joint_feedback_fixture(JointFeedbackFault::StuckFromSequence { sequence: 2 });
        sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(5), &mut bus).unwrap();
        world.entity_mut(joint).insert(JointState::Revolute {
            position_rad: 0.9,
            velocity_rad_s: -0.4,
        });
        sample_joint_feedback_sensors(&mut world, SimTime::from_ticks(1_000_005), &mut bus)
            .unwrap();
        let frame = bus.latest::<JointFeedback>(stream).expect("stuck frame");
        assert_eq!(frame.sequence, 2);
        assert_eq!(frame.payload.status, JointFeedbackStatus::StuckValue);
        assert!(matches!(
            frame.payload.joints[0].coordinate,
            JointCoordinateFeedback::Revolute {
                position_rad: 0.1,
                velocity_rad_s: 0.2
            }
        ));
    }
}
