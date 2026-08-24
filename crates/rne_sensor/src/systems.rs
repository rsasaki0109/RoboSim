//! Sensor sampling systems.

use crate::camera::sample_camera_rgbd_keyed;
use crate::components::{
    ImuState, JointFeedbackFault, JointFeedbackSensor, JointFeedbackSensorState, Sensor,
    SensorKind, SensorState,
};
use crate::imu::sample_imu_stateful;
use crate::lidar::sample_lidar_at_entity_keyed;
use crate::noise::SensorNoiseKey;
use crate::wheel_encoder::sample_wheel_encoder;
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, Frame, FramePayload, JointCommandFeedback, JointCommandMode, JointCoordinateFeedback,
    JointEffortFeedback, JointFeedback, JointFeedbackChannel, JointFeedbackStatus,
};
use rne_ecs::{Entity, World};
use rne_physics::{JointActuation, JointState, PhysicsBackend, PhysicsWorldId};
use rne_render::{HeadlessRenderBackend, RenderBackend, RenderScene};
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
    let mut imu_updates: Vec<(rne_ecs::Entity, ImuState)> = Vec::new();
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
                let sample = sample_imu_stateful(
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
                );
                imu_updates.push((entity, imu_state));
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

    for (entity, state) in imu_updates {
        ctx.world.entity_mut(entity).insert(state);
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
            Ok(JointFeedbackChannel {
                name: channel.name.clone(),
                coordinate,
                command,
                effort: JointEffortFeedback::Unavailable,
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
    use rne_math::Seconds;
    use rne_physics::{
        ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
        PhysicsWorldId, RaycastHit, RaycastQuery,
    };

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
