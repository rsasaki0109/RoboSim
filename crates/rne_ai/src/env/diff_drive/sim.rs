//! Headless differential drive simulation.

use crate::action::DiffDriveAction;
use crate::lidar::{lidar_mounts_from_spawned, sync_lidar_mounts, LidarMount};
use crate::observation::DiffDriveObservation;
use bevy_ecs::prelude::{Component, Mut};
use rne_assets::{load_and_spawn_scene, load_scene_bundle, mesh_package_roots, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_data::DataBus;
use rne_data::{
    Frame, FramePayload, ImuSample, InMemoryDataBus, JointState, PointCloud, StreamId,
    WheelEncoderSample,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_log::SimulationLog;
use rne_math::{yaw_rad, Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ContactEvent, JointMotor, PhysicsBackend, PhysicsError,
    PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, spawn_diff_drive_robot,
    sync_joint_motors_from_actuators, Actuator, ActuatorCommand, ActuatorCommandBuffer,
    ActuatorTarget, ControlMode, DiffDriveComponent, DiffDriveConfig, DiffDriveDriveMode,
    DiffDriveSpawned, Link,
};
use rne_sensor::{
    sample_sensors, ImuSpec, LidarSpec, Sensor, SensorKind, SensorSampleContext, SensorState,
};
use rne_world::{world_transform_of, Transform3, WorldEntity, WorldRandom, WorldRandomSnapshot};
use serde::{Deserialize, Serialize};
use std::any::type_name;
use std::path::{Path, PathBuf};

const IMU_STREAM_BASE: u32 = 100;
const DIFF_DRIVE_SIM_SNAPSHOT_VERSION: u32 = 1;

/// Error restoring or creating a differential-drive simulation snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum DiffDriveSimSnapshotError {
    /// Snapshot was requested while deferred commands are still pending.
    PendingCommands {
        /// Number of pending commands that were not captured.
        pending: usize,
    },
    /// Snapshot payload schema is not supported by this engine.
    UnsupportedSchemaVersion {
        /// Expected snapshot schema version.
        expected: u32,
        /// Actual snapshot schema version.
        actual: u32,
    },
    /// Snapshot references an entity that is not alive in this simulation world.
    MissingEntity {
        /// Missing entity index.
        entity_index: u32,
    },
    /// Snapshot references a component missing from an entity.
    MissingComponent {
        /// Entity index missing the component.
        entity_index: u32,
        /// Component type name.
        component: &'static str,
    },
    /// Physics backend failed while rebuilding from restored ECS state.
    Physics(PhysicsError),
}

/// Local transform snapshot for one entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveTransformSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Local transform component value.
    pub transform: Transform3,
}

/// Rigid-body velocity snapshot for one entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveRigidBodySnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Linear velocity in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

/// Actuator runtime state snapshot for one entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveActuatorSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Current actuator control mode.
    pub mode: ControlMode,
    /// Current actuator command target.
    pub target: ActuatorTarget,
}

/// Joint motor runtime state snapshot for one entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveJointMotorSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Physics joint motor command state.
    pub motor: JointMotor,
}

/// Sensor sampling state snapshot for one entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DiffDriveSensorStateSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Last published sequence number.
    pub last_sequence: u64,
    /// Simulation ticks of the last sample.
    pub last_sample_ticks: u64,
    /// Total emitted frames.
    pub frame_count: u64,
}

/// Latest typed DataBus frame snapshot for one stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveFrameSnapshot<T> {
    /// Stream identifier.
    pub stream_id: StreamId,
    /// Source entity index.
    pub entity_index: u32,
    /// Monotonic sequence number within the stream.
    pub sequence: u64,
    /// Simulation time ticks.
    pub sim_ticks: u64,
    /// Capture time ticks.
    pub capture_ticks: u64,
    /// Available time ticks.
    pub available_ticks: u64,
    /// Typed payload.
    pub payload: T,
}

impl<T: FramePayload> DiffDriveFrameSnapshot<T> {
    fn from_frame(frame: Frame<T>) -> Self {
        Self {
            stream_id: frame.stream_id,
            entity_index: frame.entity.index(),
            sequence: frame.sequence,
            sim_ticks: frame.sim_time.ticks(),
            capture_ticks: frame.capture_time.ticks(),
            available_ticks: frame.available_time.ticks(),
            payload: frame.payload,
        }
    }

    fn to_frame(&self) -> Frame<T> {
        Frame {
            stream_id: self.stream_id,
            entity: Entity::from_raw(self.entity_index),
            sequence: self.sequence,
            sim_time: SimTime::from_ticks(self.sim_ticks),
            capture_time: SimTime::from_ticks(self.capture_ticks),
            available_time: SimTime::from_ticks(self.available_ticks),
            payload: self.payload.clone(),
        }
    }
}

/// Completed-tick snapshot of a [`DiffDriveSim`].
///
/// This is intended for restoring a simulation with the same scene topology and
/// stable entity indices. It captures ECS motion state, actuator and motor
/// targets, sensor sequence state, latest DataBus sensor frames, world random
/// state, simulation time, and command sequence. It does not capture arbitrary
/// user-added resources.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveSimSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Current simulation time in nanosecond ticks.
    pub sim_ticks: u64,
    /// Number of completed simulation steps.
    pub step_count: u64,
    /// Next actuator command sequence number.
    pub command_sequence: u64,
    /// World-level deterministic random state.
    pub world_random: WorldRandomSnapshot,
    /// Local transform components.
    pub transforms: Vec<DiffDriveTransformSnapshot>,
    /// Rigid-body velocity components.
    pub rigid_bodies: Vec<DiffDriveRigidBodySnapshot>,
    /// Actuator command target state.
    pub actuators: Vec<DiffDriveActuatorSnapshot>,
    /// Physics joint motor state.
    pub joint_motors: Vec<DiffDriveJointMotorSnapshot>,
    /// Sensor runtime sequence state.
    pub sensor_states: Vec<DiffDriveSensorStateSnapshot>,
    /// Latest IMU frames by stream.
    pub imu_frames: Vec<DiffDriveFrameSnapshot<ImuSample>>,
    /// Latest LiDAR frames by stream.
    pub lidar_frames: Vec<DiffDriveFrameSnapshot<PointCloud>>,
    /// Latest wheel encoder frames by stream.
    pub wheel_encoder_frames: Vec<DiffDriveFrameSnapshot<WheelEncoderSample>>,
}

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
    lidar_mounts: Vec<LidarMount>,
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
        Self::from_spawned_world(world, robots, None, 0, drive_mode, Vec::new(), Vec::new())
    }

    /// Loads a `.rne.scene.toml` file and its referenced robot assets.
    pub fn from_scene_path(scene_path: &Path) -> Result<Self, AssetError> {
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, scene_path)?;
        let world_seed = world
            .get::<WorldEntity>(spawned.world)
            .map(|world_entity| world_entity.seed)
            .unwrap_or(0);
        let (_, first_robot) = spawned.robots.first().ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: "no robots".into(),
        })?;

        let mut robots = Vec::new();
        for (_, spawned_robot) in &spawned.robots {
            robots.push(build_diff_drive_spawned(
                &mut world,
                spawned_robot.robot,
                scene_path,
            )?);
        }

        let drive_mode = infer_drive_mode(&world, first_robot.robot);

        let bundle = load_scene_bundle(scene_path)?;
        let mesh_roots = mesh_package_roots(&bundle);

        let lidar_mounts = lidar_mounts_from_spawned(&spawned.lidar_mounts);

        Ok(Self::from_spawned_world(
            world,
            robots,
            Some(scene_path.to_path_buf()),
            world_seed,
            drive_mode,
            mesh_roots,
            lidar_mounts,
        ))
    }

    /// Returns the wheel actuation model used by this simulation.
    pub fn drive_mode(&self) -> DiffDriveDriveMode {
        self.drive_mode
    }

    /// Returns contact events produced by the last physics step.
    pub fn last_contacts(&self) -> &[ContactEvent] {
        self.backend.contacts(self.physics_world).unwrap_or(&[])
    }

    /// Returns the world seed from a loaded scene, or zero for built-in scenes.
    pub fn world_seed(&self) -> u64 {
        self.world_seed
    }

    /// Returns the fixed simulation timestep.
    pub fn fixed_delta(&self) -> SimDuration {
        self.dt
    }

    /// Returns the current simulation time.
    pub fn sim_time(&self) -> SimTime {
        self.sim_time
    }

    /// Returns the world-level deterministic random state.
    pub fn world_random_snapshot(&self) -> WorldRandomSnapshot {
        self.world
            .get_resource::<WorldRandom>()
            .map(WorldRandom::snapshot)
            .unwrap_or(WorldRandomSnapshot {
                seed: self.world_seed,
                main_rng_state: self.world_seed,
            })
    }

    /// Restores the world-level deterministic random state.
    pub fn restore_world_random_snapshot(&mut self, snapshot: WorldRandomSnapshot) {
        self.world_seed = snapshot.seed;
        if let Some(mut world_random) = self.world.get_resource_mut::<WorldRandom>() {
            world_random.restore(snapshot);
        } else {
            self.world
                .insert_resource(WorldRandom::from_snapshot(snapshot));
        }
    }

    /// Returns the loaded scene path when the simulation was created from assets.
    pub fn scene_path(&self) -> Option<&Path> {
        self.scene_path.as_deref()
    }

    /// Captures a completed-tick simulation snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`DiffDriveSimSnapshotError::PendingCommands`] if commands have
    /// been queued but not yet applied. Capture snapshots after a completed
    /// simulation tick, before queuing the next action.
    pub fn snapshot(&self) -> Result<DiffDriveSimSnapshot, DiffDriveSimSnapshotError> {
        if !self.command_buffer.is_empty() {
            return Err(DiffDriveSimSnapshotError::PendingCommands {
                pending: self.command_buffer.len(),
            });
        }

        let mut snapshot = DiffDriveSimSnapshot {
            schema_version: DIFF_DRIVE_SIM_SNAPSHOT_VERSION,
            sim_ticks: self.sim_time.ticks(),
            step_count: self.step_count,
            command_sequence: self.command_buffer.next_sequence(),
            world_random: self.world_random_snapshot(),
            transforms: Vec::new(),
            rigid_bodies: Vec::new(),
            actuators: Vec::new(),
            joint_motors: Vec::new(),
            sensor_states: Vec::new(),
            imu_frames: Vec::new(),
            lidar_frames: Vec::new(),
            wheel_encoder_frames: Vec::new(),
        };

        for entity in sorted_world_entities(&self.world) {
            let entity_index = entity.index();
            if let Some(transform) = self.world.get::<Transform3>(entity) {
                snapshot.transforms.push(DiffDriveTransformSnapshot {
                    entity_index,
                    transform: *transform,
                });
            }
            if let Some(body) = self.world.get::<RigidBody>(entity) {
                snapshot.rigid_bodies.push(DiffDriveRigidBodySnapshot {
                    entity_index,
                    linear_velocity_m_s: body.linear_velocity_m_s,
                    angular_velocity_rad_s: body.angular_velocity_rad_s,
                });
            }
            if let Some(actuator) = self.world.get::<Actuator>(entity) {
                snapshot.actuators.push(DiffDriveActuatorSnapshot {
                    entity_index,
                    mode: actuator.mode,
                    target: actuator.target,
                });
            }
            if let Some(motor) = self.world.get::<JointMotor>(entity) {
                snapshot.joint_motors.push(DiffDriveJointMotorSnapshot {
                    entity_index,
                    motor: *motor,
                });
            }
            if let Some(state) = self.world.get::<SensorState>(entity) {
                snapshot.sensor_states.push(DiffDriveSensorStateSnapshot {
                    entity_index,
                    last_sequence: state.last_sequence,
                    last_sample_ticks: state.last_sample_ticks,
                    frame_count: state.frame_count,
                });
            }
            if let Some(sensor) = self.world.get::<Sensor>(entity) {
                match sensor.kind {
                    SensorKind::Imu(_) => {
                        if let Some(frame) = self.data_bus.latest::<ImuSample>(sensor.stream_id) {
                            snapshot
                                .imu_frames
                                .push(DiffDriveFrameSnapshot::from_frame(frame));
                        }
                    }
                    SensorKind::Lidar(_) => {
                        if let Some(frame) = self.data_bus.latest::<PointCloud>(sensor.stream_id) {
                            snapshot
                                .lidar_frames
                                .push(DiffDriveFrameSnapshot::from_frame(frame));
                        }
                    }
                    SensorKind::WheelEncoder(_) => {
                        if let Some(frame) =
                            self.data_bus.latest::<WheelEncoderSample>(sensor.stream_id)
                        {
                            snapshot
                                .wheel_encoder_frames
                                .push(DiffDriveFrameSnapshot::from_frame(frame));
                        }
                    }
                    SensorKind::Camera(_) => {}
                }
            }
        }

        Ok(snapshot)
    }

    /// Restores this simulation from a completed-tick snapshot.
    ///
    /// # Errors
    ///
    /// Returns an error if the snapshot schema is unsupported, if it references
    /// missing entities/components, or if the physics backend cannot be rebuilt.
    pub fn restore_snapshot(
        &mut self,
        snapshot: &DiffDriveSimSnapshot,
    ) -> Result<(), DiffDriveSimSnapshotError> {
        if snapshot.schema_version != DIFF_DRIVE_SIM_SNAPSHOT_VERSION {
            return Err(DiffDriveSimSnapshotError::UnsupportedSchemaVersion {
                expected: DIFF_DRIVE_SIM_SNAPSHOT_VERSION,
                actual: snapshot.schema_version,
            });
        }

        for item in &snapshot.transforms {
            *component_mut::<Transform3>(&mut self.world, item.entity_index)? = item.transform;
        }
        for item in &snapshot.rigid_bodies {
            let mut body = component_mut::<RigidBody>(&mut self.world, item.entity_index)?;
            body.linear_velocity_m_s = item.linear_velocity_m_s;
            body.angular_velocity_rad_s = item.angular_velocity_rad_s;
        }
        for item in &snapshot.actuators {
            let mut actuator = component_mut::<Actuator>(&mut self.world, item.entity_index)?;
            actuator.mode = item.mode;
            actuator.target = item.target;
        }
        for item in &snapshot.joint_motors {
            *component_mut::<JointMotor>(&mut self.world, item.entity_index)? = item.motor;
        }
        for item in &snapshot.sensor_states {
            let mut state = component_mut::<SensorState>(&mut self.world, item.entity_index)?;
            state.last_sequence = item.last_sequence;
            state.last_sample_ticks = item.last_sample_ticks;
            state.frame_count = item.frame_count;
        }

        self.sim_time = SimTime::from_ticks(snapshot.sim_ticks);
        self.step_count = snapshot.step_count;
        self.restore_world_random_snapshot(snapshot.world_random);
        self.command_buffer.restore_empty(snapshot.command_sequence);
        self.data_bus = InMemoryDataBus::new();
        for frame in &snapshot.imu_frames {
            self.data_bus.publish(frame.to_frame());
        }
        for frame in &snapshot.lidar_frames {
            self.data_bus.publish(frame.to_frame());
        }
        for frame in &snapshot.wheel_encoder_frames {
            self.data_bus.publish(frame.to_frame());
        }
        self.rebuild_physics_from_ecs()?;
        Ok(())
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

    /// Provides read access to the simulation DataBus (sensor frames).
    pub fn data_bus(&self) -> &InMemoryDataBus {
        &self.data_bus
    }

    /// Returns tracked LiDAR mounts loaded from scene robot assets.
    pub fn lidar_mounts(&self) -> &[LidarMount] {
        &self.lidar_mounts
    }

    /// Returns the latest point cloud from the primary robot's LiDAR sensor.
    pub fn latest_lidar_cloud(&self) -> Option<PointCloud> {
        let mount = self.lidar_mounts.first()?;
        let sensor = self.world.get::<Sensor>(mount.lidar)?;
        self.data_bus
            .latest::<PointCloud>(sensor.stream_id)
            .map(|frame| frame.payload.clone())
    }

    /// Returns the world-space transform of the primary LiDAR mount.
    pub fn primary_lidar_world_transform(&self) -> Option<Transform3> {
        let mount = self.lidar_mounts.first()?;
        Some(world_transform_of(&self.world, mount.lidar))
    }

    /// Returns the LiDAR specification for the primary robot sensor.
    pub fn primary_lidar_spec(&self) -> Option<LidarSpec> {
        let mount = self.lidar_mounts.first()?;
        let sensor = self.world.get::<Sensor>(mount.lidar)?;
        match sensor.kind {
            SensorKind::Lidar(spec) => Some(spec),
            _ => None,
        }
    }

    /// Returns wheel joint positions and velocities for ROS `/joint_states`.
    pub fn joint_state(&self) -> JointState {
        let robot = self.robot();
        let left = wheel_joint_sample(&self.world, robot.left_wheel, robot.left_actuator);
        let right = wheel_joint_sample(&self.world, robot.right_wheel, robot.right_actuator);
        JointState {
            names: vec!["left_wheel_joint".into(), "right_wheel_joint".into()],
            positions_rad: vec![left.position_rad, right.position_rad],
            velocities_rad_s: vec![left.velocity_rad_s, right.velocity_rad_s],
        }
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
        lidar_mounts: Vec<LidarMount>,
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
            lidar_mounts,
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
        let lidar_points =
            lidar_point_count_for_robot(&self.world, &self.data_bus, spawned, &self.lidar_mounts);
        let peer = crate::multi_robot::nearest_peer_observation(self, robot);

        DiffDriveObservation {
            base_x_m: transform.translation.x,
            base_y_m: transform.translation.y,
            base_z_m: transform.translation.z,
            base_yaw_rad: yaw_rad(transform.rotation),
            left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s,
            imu_ay_m_s2: imu,
            lidar_points,
            goal_delta_x_m: goal_x_m.map(|goal| goal - transform.translation.x),
            peer_delta_x_m: peer.map(|peer| peer.delta_x_m),
            peer_delta_z_m: peer.map(|peer| peer.delta_z_m),
            peer_separation_m: peer.map(|peer| peer.separation_m),
        }
    }

    pub(crate) fn queue_robot_action(&mut self, robot: Entity, action: DiffDriveAction) {
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

    pub(crate) fn advance_one_tick(&mut self, record_log: bool, log: &mut impl StepLogTarget) {
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

        sync_lidar_mounts(&mut self.world, &self.lidar_mounts);
        sample_sensors(
            &mut SensorSampleContext {
                world: &mut self.world,
                sim_time: self.sim_time,
                physics: &self.backend,
                physics_world: self.physics_world,
                render: None,
                scene: None,
            },
            &mut self.data_bus,
        );

        self.sim_time = self.sim_time + self.dt;
        self.step_count += 1;
    }

    fn rebuild_physics_from_ecs(&mut self) -> Result<(), PhysicsError> {
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc::default())?;
        backend.sync_from_ecs(&mut self.world, physics_world)?;
        self.backend = backend;
        self.physics_world = physics_world;
        Ok(())
    }
}

impl From<PhysicsError> for DiffDriveSimSnapshotError {
    fn from(error: PhysicsError) -> Self {
        Self::Physics(error)
    }
}

fn sorted_world_entities(world: &World) -> Vec<Entity> {
    let mut entities: Vec<Entity> = world.iter_entities().map(|entity| entity.id()).collect();
    entities.sort_unstable();
    entities
}

fn component_mut<T: Component>(
    world: &mut World,
    entity_index: u32,
) -> Result<Mut<'_, T>, DiffDriveSimSnapshotError> {
    let entity = Entity::from_raw(entity_index);
    if !world
        .iter_entities()
        .any(|entity_ref| entity_ref.id() == entity)
    {
        return Err(DiffDriveSimSnapshotError::MissingEntity { entity_index });
    }
    world
        .get_mut::<T>(entity)
        .ok_or(DiffDriveSimSnapshotError::MissingComponent {
            entity_index,
            component: type_name::<T>(),
        })
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

fn lidar_point_count_for_robot(
    world: &World,
    data_bus: &InMemoryDataBus,
    spawned: &DiffDriveSpawned,
    lidar_mounts: &[LidarMount],
) -> usize {
    let Some(mount) = lidar_mounts
        .iter()
        .find(|mount| mount.base_link == spawned.base_link)
    else {
        return 0;
    };
    let Some(sensor) = world.get::<Sensor>(mount.lidar) else {
        return 0;
    };
    data_bus
        .latest::<rne_data::PointCloud>(sensor.stream_id)
        .map(|frame| frame.payload.points_m.len())
        .unwrap_or(0)
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

struct WheelJointSample {
    position_rad: f64,
    velocity_rad_s: f64,
}

fn wheel_joint_sample(world: &World, wheel: Entity, actuator: Entity) -> WheelJointSample {
    let position_rad = world
        .get::<Transform3>(wheel)
        .map(|transform| 2.0 * f64::atan2(transform.rotation.z, transform.rotation.w))
        .unwrap_or(0.0);
    let velocity_rad_s = world
        .get::<Actuator>(actuator)
        .map(|actuator| actuator.target.velocity_rad_s)
        .unwrap_or(0.0);
    WheelJointSample {
        position_rad,
        velocity_rad_s,
    }
}

fn build_diff_drive_spawned(
    world: &mut World,
    robot: Entity,
    scene_path: &Path,
) -> Result<DiffDriveSpawned, AssetError> {
    let drive = world
        .get::<DiffDriveComponent>(robot)
        .ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: format!("robot {:?} is not a diff drive", robot),
        })?
        .0;
    let left_wheel =
        find_robot_link(world, drive.robot, "left_wheel").ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: "missing left_wheel link".into(),
        })?;
    let right_wheel =
        find_robot_link(world, drive.robot, "right_wheel").ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: "missing right_wheel link".into(),
        })?;

    Ok(DiffDriveSpawned {
        robot: drive.robot,
        base_link: drive.base_link,
        left_wheel,
        right_wheel,
        left_actuator: drive.left_actuator,
        right_actuator: drive.right_actuator,
        drive,
    })
}

fn infer_drive_mode(world: &World, robot: Entity) -> DiffDriveDriveMode {
    let Some(drive) = world.get::<DiffDriveComponent>(robot) else {
        return DiffDriveDriveMode::Kinematic;
    };
    match world
        .get::<RigidBody>(drive.0.base_link)
        .map(|body| body.body_type)
    {
        Some(RigidBodyType::Dynamic) => DiffDriveDriveMode::JointDriven,
        _ => DiffDriveDriveMode::Kinematic,
    }
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
        assert_eq!(sim.world_random_snapshot().seed, 42);

        let mut final_x = 0.0;
        for _ in 0..180 {
            let obs = sim.step(6.0, 6.0);
            final_x = obs.base_x_m;
        }

        assert!(final_x > 1.5, "expected forward motion, got x={final_x}");
    }

    #[test]
    fn world_random_snapshot_restores_main_stream_position() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        sim.world_mut().resource_mut::<WorldRandom>().next_u64();
        let snapshot = sim.world_random_snapshot();
        let expected = sim.world_mut().resource_mut::<WorldRandom>().next_u64();

        sim.world_mut().resource_mut::<WorldRandom>().next_u64();
        sim.restore_world_random_snapshot(snapshot);

        assert_eq!(sim.world_seed(), 42);
        assert_eq!(
            sim.world_mut().resource_mut::<WorldRandom>().next_u64(),
            expected
        );
    }

    #[test]
    fn snapshot_rejects_pending_commands() {
        let mut sim = DiffDriveSim::new();

        sim.queue_robot_action(sim.robot().robot, DiffDriveAction::forward(1.0));

        assert_eq!(
            sim.snapshot(),
            Err(DiffDriveSimSnapshotError::PendingCommands { pending: 2 })
        );
    }

    #[test]
    fn snapshot_restores_kinematic_state_and_sensor_frame() {
        let mut sim = DiffDriveSim::with_robot_configs(&[DiffDriveConfig {
            drive_mode: DiffDriveDriveMode::Kinematic,
            ..DiffDriveConfig::default()
        }]);
        sim.step(3.0, 3.0);
        let snapshot = sim.snapshot().unwrap();
        let observation_at_snapshot = sim.observe();
        let expected_next = sim.step(1.0, 2.0);
        let expected_after_next = sim.snapshot().unwrap();

        sim.step(-2.0, 5.0);
        sim.step(4.0, -1.0);

        sim.restore_snapshot(&snapshot).unwrap();
        assert_eq!(sim.observe(), observation_at_snapshot);
        let restored_next = sim.step(1.0, 2.0);
        let restored_after_next = sim.snapshot().unwrap();

        assert_eq!(restored_next, expected_next);
        assert_eq!(restored_after_next, expected_after_next);
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

    #[test]
    fn lidar_scene_produces_point_cloud_hits() {
        let scene_text = r#"
[ground]
enabled = true

[[robots]]
path = "robot.rne.robot.toml"

[[obstacles]]
name = "front_wall"
translation_m = [0.0, 1.0, 8.0]
half_extents_m = [8.0, 1.0, 0.25]
"#;
        let robot_text = r#"
kind = "diff_drive"
model_name = "diff_drive"

[diff_drive]

[lidar]
"#;
        let dir = std::env::temp_dir().join(format!("rne_ai_lidar_sim_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("robot.rne.robot.toml"), robot_text).unwrap();
        let scene_path = dir.join("scene.rne.scene.toml");
        std::fs::write(&scene_path, scene_text).unwrap();

        let mut sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        for _ in 0..120 {
            sim.step(6.0, 6.0);
        }
        let obs = sim.observe();
        assert!(
            obs.lidar_points >= 8,
            "expected lidar hits from scene asset, got {}",
            obs.lidar_points
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn multi_robot_scene_loads_all_robots() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/multi_robot_collision.rne.scene.toml");
        let sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        assert_eq!(sim.robots().len(), 2);
        assert_eq!(sim.drive_mode(), DiffDriveDriveMode::JointDriven);
    }
}
