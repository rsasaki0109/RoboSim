//! Headless differential drive simulation.

use crate::action::DiffDriveAction;
use crate::observation::DiffDriveObservation;
use rne_assets::{load_and_spawn_scene, load_scene_bundle, mesh_package_roots, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_data::DataBus;
use rne_data::{InMemoryDataBus, StreamId};
use rne_ecs::{spawn_named, Entity, World};
use rne_log::SimulationLog;
use rne_math::{yaw_rad, Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    sync_joint_motors_from_actuators, Actuator, ActuatorCommand, ActuatorCommandBuffer,
    DiffDriveComponent, DiffDriveConfig, DiffDriveDriveMode, DiffDriveSpawned, Link,
};
use rne_sensor::{sample_sensors, ImuSpec, Sensor, SensorKind, SensorSampleContext, SensorState};
use rne_world::{Transform3, WorldEntity};
use std::path::{Path, PathBuf};

const IMU_STREAM_BASE: u32 = 100;

/// Headless differential drive environment.
pub struct DiffDriveSim {
    scene_path: Option<PathBuf>,
    world_seed: u64,
    world: World,
    backend: RapierBackend,
    physics_world: rne_physics::PhysicsWorldId,
    robots: Vec<DiffDriveSpawned>,
    command_buffer: ActuatorCommandBuffer,
    data_bus: InMemoryDataBus,
    sim_time: SimTime,
    dt: SimDuration,
    step_count: u64,
    drive_mode: DiffDriveDriveMode,
    mesh_package_roots: Vec<PathBuf>,
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
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
        );
        Self::from_spawned_world(
            world,
            vec![robot],
            None,
            0,
            DiffDriveDriveMode::JointDriven,
            Vec::new(),
        )
    }

    /// Creates a simulation with multiple diff-drive robots on a shared ground plane.
    pub fn with_robot_configs(configs: &[DiffDriveConfig]) -> Self {
        assert!(
            !configs.is_empty(),
            "with_robot_configs requires at least one robot"
        );
        let mut world = World::new();
        spawn_ground(&mut world);
        let robots = configs
            .iter()
            .map(|config| spawn_diff_drive_robot(&mut world, config))
            .collect();
        let drive_mode = configs[0].drive_mode;
        Self::from_spawned_world(world, robots, None, 0, drive_mode, Vec::new())
    }

    /// Loads a `.rne.scene.toml` file and its referenced robot assets.
    pub fn from_scene_path(scene_path: &Path) -> Result<Self, AssetError> {
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, scene_path)?;
        let world_seed = world
            .get::<WorldEntity>(spawned.world)
            .map(|world_entity| world_entity.seed)
            .unwrap_or(0);
        let (_, robot) = spawned.robots.first().ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: "no robots".into(),
        })?;
        let drive = world
            .get::<DiffDriveComponent>(robot.robot)
            .ok_or_else(|| AssetError::Invalid {
                path: scene_path.display().to_string(),
                message: "first robot is not a diff drive".into(),
            })?
            .0;
        let left_wheel =
            find_robot_link(&mut world, drive.robot, "left_wheel").ok_or_else(|| {
                AssetError::Invalid {
                    path: scene_path.display().to_string(),
                    message: "missing left_wheel link".into(),
                }
            })?;
        let right_wheel =
            find_robot_link(&mut world, drive.robot, "right_wheel").ok_or_else(|| {
                AssetError::Invalid {
                    path: scene_path.display().to_string(),
                    message: "missing right_wheel link".into(),
                }
            })?;

        let robot_spawned = DiffDriveSpawned {
            robot: drive.robot,
            base_link: drive.base_link,
            left_wheel,
            right_wheel,
            left_actuator: drive.left_actuator,
            right_actuator: drive.right_actuator,
            drive,
        };

        let bundle = load_scene_bundle(scene_path)?;
        let mesh_roots = mesh_package_roots(&bundle);

        Ok(Self::from_spawned_world(
            world,
            vec![robot_spawned],
            Some(scene_path.to_path_buf()),
            world_seed,
            DiffDriveDriveMode::Kinematic,
            mesh_roots,
        ))
    }

    /// Returns the world seed from a loaded scene, or zero for built-in scenes.
    pub fn world_seed(&self) -> u64 {
        self.world_seed
    }

    /// Returns the loaded scene path when the simulation was created from assets.
    pub fn scene_path(&self) -> Option<&Path> {
        self.scene_path.as_deref()
    }

    /// Reloads the simulation from its scene asset when one was used to create it.
    pub fn reload_scene(&mut self) -> Result<(), AssetError> {
        let Some(scene_path) = self.scene_path.clone() else {
            return Ok(());
        };
        *self = Self::from_scene_path(&scene_path)?;
        Ok(())
    }

    /// Provides read access to the ECS world (for rendering or inspection).
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Provides mutable access to the ECS world (for spawning agents or inspecting).
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// Provides read access to the primary diff-drive robot handles.
    pub fn robot(&self) -> &DiffDriveSpawned {
        &self.robots[0]
    }

    /// Returns every diff-drive robot spawned in this simulation.
    pub fn robots(&self) -> &[DiffDriveSpawned] {
        &self.robots
    }

    /// Returns package roots used to resolve URDF mesh assets for the loaded scene.
    pub fn mesh_package_roots(&self) -> &[PathBuf] {
        &self.mesh_package_roots
    }

    /// Spawns an additional diff-drive robot into the live simulation world.
    pub fn spawn_robot(&mut self, config: DiffDriveConfig) -> DiffDriveSpawned {
        let spawned = spawn_diff_drive_robot(&mut self.world, &config);
        attach_imu(
            &mut self.world,
            spawned.base_link,
            imu_stream_for_index(self.robots.len()),
        );
        self.backend
            .sync_from_ecs(&mut self.world, self.physics_world)
            .expect("sync spawned robot into physics");
        self.robots.push(spawned);
        spawned
    }

    fn from_spawned_world(
        mut world: World,
        robots: Vec<DiffDriveSpawned>,
        scene_path: Option<PathBuf>,
        world_seed: u64,
        drive_mode: DiffDriveDriveMode,
        mesh_package_roots: Vec<PathBuf>,
    ) -> Self {
        for (index, robot) in robots.iter().enumerate() {
            attach_imu(&mut world, robot.base_link, imu_stream_for_index(index));
        }

        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");
        backend.sync_from_ecs(&mut world, physics_world).unwrap();

        Self {
            scene_path,
            world_seed,
            world,
            backend,
            physics_world,
            robots,
            command_buffer: ActuatorCommandBuffer::new(),
            data_bus: InMemoryDataBus::new(),
            sim_time: SimTime::ZERO,
            dt: SimDuration::from_hertz(Hertz::new(60.0)),
            step_count: 0,
            drive_mode,
            mesh_package_roots,
        }
    }

    /// Resets the simulation to its initial state.
    pub fn reset(&mut self) -> DiffDriveObservation {
        if let Some(scene_path) = self.scene_path.clone() {
            *self = Self::from_scene_path(&scene_path).expect("reload scene");
        } else {
            let initial = self
                .world
                .get::<Transform3>(self.robots[0].base_link)
                .map(|tf| tf.translation)
                .unwrap_or_else(|| Vec3::new(0.0, 0.25, 0.0));
            *self = Self::with_initial_translation(initial);
        }
        self.observe()
    }

    /// Applies wheel velocities to the primary robot and advances one simulation step.
    pub fn step(
        &mut self,
        left_velocity_rad_s: f64,
        right_velocity_rad_s: f64,
    ) -> DiffDriveObservation {
        self.step_with_recording(left_velocity_rad_s, right_velocity_rad_s, false, &mut ())
    }

    /// Applies a diff-drive action to the primary robot and advances one simulation step.
    pub fn step_action(&mut self, action: DiffDriveAction) -> DiffDriveObservation {
        self.step_robot_action(self.robots[0].robot, action, None)
    }

    /// Applies a diff-drive action to one robot and advances one simulation step.
    pub fn step_robot_action(
        &mut self,
        robot: Entity,
        action: DiffDriveAction,
        goal_x_m: Option<f64>,
    ) -> DiffDriveObservation {
        self.queue_robot_action(robot, action);
        self.advance_one_tick(false, &mut ());
        self.observe_robot_with_goal(robot, goal_x_m)
    }

    /// Applies actions for multiple robots, then advances the simulation once.
    pub fn step_robots_actions(
        &mut self,
        actions: &[(Entity, DiffDriveAction)],
    ) -> Vec<(Entity, DiffDriveObservation)> {
        for (robot, action) in actions {
            self.queue_robot_action(*robot, *action);
        }
        self.advance_one_tick(false, &mut ());

        actions
            .iter()
            .map(|(robot, _)| (*robot, self.observe_robot(*robot)))
            .collect()
    }

    /// Applies wheel velocities, optionally recording actuator commands to a log.
    pub fn step_with_recording(
        &mut self,
        left_velocity_rad_s: f64,
        right_velocity_rad_s: f64,
        record_log: bool,
        log: &mut impl StepLogTarget,
    ) -> DiffDriveObservation {
        self.queue_robot_action(
            self.robots[0].robot,
            DiffDriveAction {
                left_velocity_rad_s,
                right_velocity_rad_s,
            },
        );
        self.advance_one_tick(record_log, log);
        self.observe()
    }

    /// Returns the number of completed simulation steps.
    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    /// Builds an observation from the primary robot state.
    pub fn observe(&self) -> DiffDriveObservation {
        self.observe_robot(self.robots[0].robot)
    }

    /// Builds an observation for a specific diff-drive robot in this world.
    pub fn observe_robot(&self, robot: Entity) -> DiffDriveObservation {
        self.observe_robot_with_goal(robot, None)
    }

    /// Builds an observation for a robot, optionally including goal-relative features.
    pub fn observe_robot_with_goal(
        &self,
        robot: Entity,
        goal_x_m: Option<f64>,
    ) -> DiffDriveObservation {
        let spawned = self
            .robots
            .iter()
            .find(|spawned| spawned.robot == robot)
            .unwrap_or(&self.robots[0]);
        let base_link = spawned.base_link;
        let transform = self
            .world
            .get::<Transform3>(base_link)
            .copied()
            .unwrap_or_default();
        let left_wheel_velocity_rad_s = self
            .world
            .get::<Actuator>(spawned.left_actuator)
            .map(|actuator| actuator.target.velocity_rad_s)
            .unwrap_or(0.0);
        let right_wheel_velocity_rad_s = self
            .world
            .get::<Actuator>(spawned.right_actuator)
            .map(|actuator| actuator.target.velocity_rad_s)
            .unwrap_or(0.0);
        let imu = imu_ay_for_base(&self.world, &self.data_bus, base_link);

        DiffDriveObservation {
            base_x_m: transform.translation.x,
            base_y_m: transform.translation.y,
            base_z_m: transform.translation.z,
            base_yaw_rad: yaw_rad(transform.rotation),
            left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s,
            imu_ay_m_s2: imu,
            lidar_points: 0,
            goal_delta_x_m: goal_x_m.map(|goal| goal - transform.translation.x),
        }
    }

    fn queue_robot_action(&mut self, robot: Entity, action: DiffDriveAction) {
        let spawned = self
            .robots
            .iter()
            .find(|spawned| spawned.robot == robot)
            .unwrap_or(&self.robots[0]);
        self.command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: spawned.left_actuator,
                velocity_rad_s: action.left_velocity_rad_s,
            },
            self.sim_time,
        );
        self.command_buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: spawned.right_actuator,
                velocity_rad_s: action.right_velocity_rad_s,
            },
            self.sim_time,
        );
    }

    fn advance_one_tick(&mut self, record_log: bool, log: &mut impl StepLogTarget) {
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

        let drives = collect_drives(&mut self.world);
        match self.drive_mode {
            DiffDriveDriveMode::Kinematic => {
                differential_drive_kinematics(&mut self.world, &drives, self.dt);
            }
            DiffDriveDriveMode::JointDriven => {
                sync_joint_motors_from_actuators(&mut self.world, &drives);
            }
        }
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

fn imu_stream_for_index(index: usize) -> StreamId {
    StreamId::new(IMU_STREAM_BASE as u64 + index as u64)
}

fn collect_drives(world: &mut World) -> Vec<rne_robot::DifferentialDrive> {
    let mut query = world.query::<&DiffDriveComponent>();
    query.iter(world).map(|component| component.0).collect()
}

fn imu_ay_for_base(world: &World, data_bus: &InMemoryDataBus, base_link: Entity) -> f64 {
    let Some(sensor) = world.get::<Sensor>(base_link) else {
        return 0.0;
    };
    let SensorKind::Imu(_) = sensor.kind else {
        return 0.0;
    };
    data_bus
        .latest::<rne_data::ImuSample>(sensor.stream_id)
        .map(|frame| frame.payload.linear_acceleration_m_s2.y)
        .unwrap_or(0.0)
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

fn attach_imu(world: &mut World, base_link: rne_ecs::Entity, stream_id: StreamId) {
    world.entity_mut(base_link).insert((
        Sensor {
            kind: SensorKind::Imu(ImuSpec::default()),
            update_rate_hz: 60.0,
            latency_ticks: 0,
            frame_id: 10,
            enabled: true,
            stream_id,
        },
        SensorState::default(),
    ));
}

fn find_robot_link(world: &mut World, robot: Entity, link_name: &str) -> Option<Entity> {
    let mut query = world.query::<(Entity, &Link)>();
    query
        .iter(world)
        .find(|(_, link)| link.robot == robot && link.name == link_name)
        .map(|(entity, _)| entity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn diff_drive_moves_forward_under_equal_wheel_speeds() {
        let mut sim = DiffDriveSim::new();
        let mut final_x = 0.0;

        for _ in 0..300 {
            let obs = sim.step(6.0, 6.0);
            final_x = obs.base_x_m;
        }

        assert!(final_x > 0.5, "expected forward motion, got x={final_x}");
    }

    #[test]
    fn observation_includes_yaw_and_wheel_velocities() {
        let mut sim = DiffDriveSim::new();
        let obs = sim.step(4.0, 2.0);

        assert!(obs.left_wheel_velocity_rad_s.abs() > 0.0);
        assert!(obs.right_wheel_velocity_rad_s.abs() > 0.0);
        assert!(obs.base_yaw_rad.abs() < 0.5);
    }

    #[test]
    fn multi_robot_single_tick_applies_distinct_commands() {
        let mut sim = DiffDriveSim::with_robot_configs(&[
            DiffDriveConfig {
                model_name: "robot_a".into(),
                initial_translation_m: Vec3::new(0.0, 0.25, -1.0),
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
            DiffDriveConfig {
                model_name: "robot_b".into(),
                initial_translation_m: Vec3::new(0.0, 0.25, 1.0),
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
        ]);
        let robot_a = sim.robots()[0].robot;
        let robot_b = sim.robots()[1].robot;

        sim.step_robots_actions(&[
            (robot_a, DiffDriveAction::forward(6.0)),
            (robot_b, DiffDriveAction::forward(2.0)),
        ]);

        let obs_a = sim.observe_robot(robot_a);
        let obs_b = sim.observe_robot(robot_b);
        assert!(obs_a.left_wheel_velocity_rad_s > obs_b.left_wheel_velocity_rad_s);
    }

    #[test]
    fn goal_relative_observation_is_populated() {
        let sim = DiffDriveSim::new();
        let obs = sim.observe_robot_with_goal(sim.robot().robot, Some(2.0));
        assert_eq!(obs.goal_delta_x_m, Some(2.0));
    }

    #[test]
    fn scene_asset_loads_and_moves_forward() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        assert_eq!(sim.world_seed(), 42);

        let mut final_x = 0.0;
        for _ in 0..180 {
            let obs = sim.step(6.0, 6.0);
            final_x = obs.base_x_m;
        }

        assert!(final_x > 1.5, "expected forward motion, got x={final_x}");
    }

    #[test]
    fn reload_scene_from_fixture() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        assert_eq!(sim.world_seed(), 42);
        sim.reload_scene().expect("reload scene");
        assert_eq!(sim.world_seed(), 42);
    }
}
