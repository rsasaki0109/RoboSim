//! Sensor sampling systems.

use crate::camera::sample_camera_rgbd;
use crate::components::{Sensor, SensorKind, SensorState};
use crate::imu::sample_imu_keyed;
use crate::lidar::sample_lidar_at_entity;
use crate::noise::SensorNoiseKey;
use crate::wheel_encoder::sample_wheel_encoder;
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, FramePayload};
use rne_ecs::World;
use rne_physics::{PhysicsBackend, PhysicsWorldId};
use rne_render::{HeadlessRenderBackend, RenderBackend, RenderScene};
use rne_world::{Transform3, WorldRandom};

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
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample_imu_keyed(
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
            SensorKind::Lidar(spec) => {
                publish_frame(
                    bus,
                    Frame::new(
                        sensor.stream_id,
                        entity,
                        state.last_sequence,
                        ctx.sim_time,
                        sample_lidar_at_entity(
                            ctx.physics,
                            ctx.physics_world,
                            ctx.world,
                            entity,
                            spec,
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
                let sample = if let Some(render) = &mut ctx.render {
                    sample_camera_rgbd(*render, &transform, spec, ctx.sim_time, scene)
                } else {
                    sample_camera_rgbd(&mut headless_render, &transform, spec, ctx.sim_time, scene)
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

    #[test]
    fn sensor_emits_at_configured_rate() {
        let mut world = World::new();
        let sensor_entity = spawn_named(&mut world, "imu");
        world.entity_mut(sensor_entity).insert((
            Sensor {
                kind: SensorKind::Imu(ImuSpec {
                    noise: NoiseModel::default(),
                    seed: 1,
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
}
