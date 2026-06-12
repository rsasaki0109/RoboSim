//! Headless differential drive simulation.

use crate::observation::DiffDriveObservation;
use rne_core::{SimDuration, SimTime};
use rne_data::DataBus;
use rne_data::{InMemoryDataBus, StreamId};
use rne_ecs::{spawn_named, World};
use rne_log::SimulationLog;
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, DiffDriveConfig, DiffDriveSpawned,
};
use rne_sensor::{sample_sensors, ImuSpec, Sensor, SensorKind, SensorSampleContext, SensorState};
use rne_world::Transform3;

const IMU_STREAM: StreamId = StreamId::new(100);

/// Headless differential drive environment.
pub struct DiffDriveSim {
    world: World,
    backend: RapierBackend,
    physics_world: rne_physics::PhysicsWorldId,
    robot: DiffDriveSpawned,
    command_buffer: ActuatorCommandBuffer,
    data_bus: InMemoryDataBus,
    sim_time: SimTime,
    dt: SimDuration,
    step_count: u64,
}

impl DiffDriveSim {
    /// Creates a new diff drive simulation with a flat ground plane.
    pub fn new() -> Self {
        Self::with_initial_translation(Vec3::new(0.0, 0.25, 0.0))
    }

    /// Creates a simulation with a custom initial robot translation.
    pub fn with_initial_translation(initial_translation_m: Vec3) -> Self {
        let mut world = World::new();
        spawn_ground(&mut world);
        let robot = spawn_diff_drive_robot(
            &mut world,
            &DiffDriveConfig {
                initial_translation_m,
                ..DiffDriveConfig::default()
            },
        );
        attach_imu(&mut world, robot.base_link);

        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");
        backend.sync_from_ecs(&mut world, physics_world).unwrap();

        Self {
            world,
            backend,
            physics_world,
            robot,
            command_buffer: ActuatorCommandBuffer::new(),
            data_bus: InMemoryDataBus::new(),
            sim_time: SimTime::ZERO,
            dt: SimDuration::from_hertz(Hertz::new(60.0)),
            step_count: 0,
        }
    }

    /// Resets the simulation to its initial state.
    pub fn reset(&mut self) -> DiffDriveObservation {
        let initial = self
            .world
            .get::<Transform3>(self.robot.base_link)
            .map(|tf| tf.translation)
            .unwrap_or_else(|| Vec3::new(0.0, 0.25, 0.0));
        *self = Self::with_initial_translation(initial);
        self.observe()
    }

    /// Applies wheel velocities and advances one simulation step.
    pub fn step(
        &mut self,
        left_velocity_rad_s: f64,
        right_velocity_rad_s: f64,
    ) -> DiffDriveObservation {
        self.step_with_recording(left_velocity_rad_s, right_velocity_rad_s, false, &mut ())
    }

    /// Applies wheel velocities, optionally recording actuator commands to a log.
    pub fn step_with_recording(
        &mut self,
        left_velocity_rad_s: f64,
        right_velocity_rad_s: f64,
        record_log: bool,
        log: &mut impl StepLogTarget,
    ) -> DiffDriveObservation {
        self.command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: self.robot.left_actuator,
                velocity_rad_s: left_velocity_rad_s,
            },
            self.sim_time,
        );
        self.command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: self.robot.right_actuator,
                velocity_rad_s: right_velocity_rad_s,
            },
            self.sim_time,
        );

        let entries: Vec<_> = if record_log {
            self.command_buffer.pending().cloned().collect()
        } else {
            Vec::new()
        };

        apply_actuator_commands(&mut self.world, &mut self.command_buffer);

        if record_log {
            for entry in entries {
                log.record_actuator_command(&entry);
            }
        }
        let drive = self
            .world
            .get::<DiffDriveComponent>(self.robot.robot)
            .expect("drive component")
            .0;
        differential_drive_kinematics(&mut self.world, &[drive], self.dt);
        step_physics(
            &mut self.backend,
            &mut self.world,
            self.physics_world,
            self.dt,
        )
        .unwrap();

        sample_sensors(
            &mut SensorSampleContext {
                world: &mut self.world,
                sim_time: self.sim_time,
                physics: &self.backend,
                physics_world: self.physics_world,
                render: None,
            },
            &mut self.data_bus,
        );

        self.sim_time = self.sim_time + self.dt;
        self.step_count += 1;
        self.observe()
    }

    /// Returns the number of completed simulation steps.
    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    /// Builds an observation from the current world state.
    pub fn observe(&self) -> DiffDriveObservation {
        let pose = self
            .world
            .get::<Transform3>(self.robot.base_link)
            .copied()
            .unwrap_or_default()
            .translation;
        let imu = self
            .data_bus
            .latest::<rne_data::ImuSample>(IMU_STREAM)
            .map(|frame| frame.payload.linear_acceleration_m_s2.y)
            .unwrap_or(0.0);

        DiffDriveObservation {
            base_x_m: pose.x,
            base_y_m: pose.y,
            base_z_m: pose.z,
            imu_ay_m_s2: imu,
            lidar_points: 0,
        }
    }
}

impl Default for DiffDriveSim {
    fn default() -> Self {
        Self::new()
    }
}

/// Target for optional actuator command recording during a step.
pub trait StepLogTarget {
    /// Records one actuator command entry.
    fn record_actuator_command(&mut self, entry: &rne_robot::ActuatorCommandEntry);
}

impl StepLogTarget for () {
    fn record_actuator_command(&mut self, _entry: &rne_robot::ActuatorCommandEntry) {}
}

impl StepLogTarget for SimulationLog {
    fn record_actuator_command(&mut self, entry: &rne_robot::ActuatorCommandEntry) {
        SimulationLog::record_actuator_command(self, entry);
    }
}

fn spawn_ground(world: &mut World) {
    let ground = spawn_named(world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(20.0, 0.5, 20.0),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
}

fn attach_imu(world: &mut World, base_link: rne_ecs::Entity) {
    world.entity_mut(base_link).insert((
        Sensor {
            kind: SensorKind::Imu(ImuSpec::default()),
            update_rate_hz: 60.0,
            latency_ticks: 0,
            frame_id: 10,
            enabled: true,
            stream_id: IMU_STREAM,
        },
        SensorState::default(),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diff_drive_moves_forward_under_equal_wheel_speeds() {
        let mut sim = DiffDriveSim::new();
        let mut final_x = 0.0;

        for _ in 0..180 {
            let obs = sim.step(6.0, 6.0);
            final_x = obs.base_x_m;
        }

        assert!(final_x > 1.5, "expected forward motion, got x={final_x}");
    }
}
