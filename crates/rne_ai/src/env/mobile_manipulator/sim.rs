//! Headless mobile manipulator environment (fixed-base arm and diff-drive mobile variant).

use super::drive::{
    wheel_command_to_motor_rad_s, MM_MOBILE_TRACK_WIDTH_M, MM_MOBILE_WHEEL_RADIUS_M,
};
use crate::action::MobileManipulatorAction;
use crate::camera::{
    sync_wrist_camera_mounts, wrist_camera_depth_stream, wrist_camera_mounts_from_spawned,
    WristCameraMount,
};
use crate::observation::MobileManipulatorObservation;
use crate::render::build_visual_render_scene;
use bevy_ecs::prelude::{Component, Mut};
use rne_assets::{load_and_spawn_scene, load_scene_bundle, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, Frame, FramePayload, ImageDepth, ImageRgb8, InMemoryDataBus, JointState, StreamId,
};
use rne_ecs::{Entity, World};
use rne_math::{yaw_rad, Hertz, Quat, Vec3};
use rne_physics::{
    ContactEvent, FixedJointDesc, JointMotor, PhysicsBackend, PhysicsError, PhysicsWorldDesc,
    PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::HeadlessRenderBackend;
use rne_robot::Link;
use rne_sensor::{sample_sensors, Sensor, SensorSampleContext, SensorState};
use rne_urdf_import::UrdfRobot;
use rne_world::{world_transform_of, Transform3, WorldEntity, WorldRandom, WorldRandomSnapshot};
use serde::{Deserialize, Serialize};
use std::any::type_name;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

const JOINT_STATE_STREAM: u32 = 300;
const MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION: u32 = 2;
/// Oldest supported mobile-manipulator snapshot schema (v1 had no wrist depth frame).
const MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION: u32 = 1;

/// Error restoring or creating a mobile-manipulator simulation snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum MobileManipulatorSimSnapshotError {
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

/// Local transform snapshot for one mobile-manipulator entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorTransformSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Local transform component value.
    pub transform: Transform3,
}

/// Rigid-body velocity snapshot for one mobile-manipulator entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorRigidBodySnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Linear velocity in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

/// Joint motor runtime state snapshot for one mobile-manipulator entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorJointMotorSnapshot {
    /// Entity index in the simulation world.
    pub entity_index: u32,
    /// Physics joint motor command state.
    pub motor: JointMotor,
}

/// Fixed joint runtime state snapshot for one mobile-manipulator entity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorFixedJointSnapshot {
    /// Child entity index carrying the fixed joint component.
    pub entity_index: u32,
    /// Parent rigid-body entity index.
    pub parent_index: u32,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child frame relative to the parent frame.
    pub relative_rotation: Quat,
}

/// Sensor sampling state snapshot for one mobile-manipulator entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MobileManipulatorSensorStateSnapshot {
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
pub struct MobileManipulatorFrameSnapshot<T> {
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

impl<T: FramePayload> MobileManipulatorFrameSnapshot<T> {
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

/// Completed-tick snapshot of a [`MobileManipulatorSim`].
///
/// This is intended for restoring a simulation with the same scene topology and
/// stable entity indices. It captures ECS motion state, joint motor targets,
/// grasp welds, latest DataBus frames, world random state, simulation time, and
/// stream sequence state. It does not capture arbitrary user-added resources.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorSimSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Current simulation time in nanosecond ticks.
    pub sim_ticks: u64,
    /// Number of completed simulation steps.
    pub step_count: u64,
    /// Next joint-state sequence number.
    pub joint_sequence: u64,
    /// Integrated target height for vertical lift joints.
    pub lift_target_m: f64,
    /// Currently grasped object entity index, if any.
    pub grasped_object_index: Option<u32>,
    /// World-level deterministic random state.
    pub world_random: WorldRandomSnapshot,
    /// Local transform components.
    pub transforms: Vec<MobileManipulatorTransformSnapshot>,
    /// Rigid-body velocity components.
    pub rigid_bodies: Vec<MobileManipulatorRigidBodySnapshot>,
    /// Physics joint motor state.
    pub joint_motors: Vec<MobileManipulatorJointMotorSnapshot>,
    /// Fixed joint components, including runtime grasp welds.
    pub fixed_joints: Vec<MobileManipulatorFixedJointSnapshot>,
    /// Sensor runtime sequence state.
    pub sensor_states: Vec<MobileManipulatorSensorStateSnapshot>,
    /// Latest joint-state DataBus frame.
    pub joint_state_frame: Option<MobileManipulatorFrameSnapshot<JointState>>,
    /// Latest wrist camera DataBus frame.
    pub wrist_camera_frame: Option<MobileManipulatorFrameSnapshot<ImageRgb8>>,
    /// Latest wrist depth DataBus frame (schema v2+).
    #[serde(default)]
    pub wrist_depth_frame: Option<MobileManipulatorFrameSnapshot<ImageDepth>>,
}

/// Default scene asset for the fixed-base `mm_minimal` robot.
pub fn mm_minimal_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/scenes/mm_minimal.rne.scene.toml")
}

/// Default scene asset for the diff-drive `mm_mobile` robot.
pub fn mm_mobile_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/scenes/mm_mobile.rne.scene.toml")
}

/// Default scene asset for the lift-equipped `mm_lift` robot.
pub fn mm_lift_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/scenes/mm_lift.rne.scene.toml")
}

/// Scene asset with a cube on the ground under the lift robot's top-down gripper,
/// for vertical pick-and-lift tests.
pub fn mm_lift_pick_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mm_lift_pick.rne.scene.toml")
}

/// Scene asset with a tabletop cube for gripper contact smoke tests.
pub fn mm_minimal_grasp_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mm_minimal_grasp.rne.scene.toml")
}

/// Scene asset with a dynamic cube for grasp-and-transport smoke tests.
pub fn mm_minimal_transport_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mm_minimal_transport.rne.scene.toml")
}

/// Scene asset with three tabletop cubes for clutter pick episodes.
pub fn mm_minimal_clutter_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mm_minimal_clutter.rne.scene.toml")
}

/// Scene asset with a diff-drive base and three cubes spread along X.
pub fn mm_mobile_clutter_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mm_mobile_clutter.rne.scene.toml")
}

/// How an actuated joint's position is read back and how its command maps to a motor.
#[derive(Clone, Copy, PartialEq)]
enum JointReadAxis {
    /// Revolute about Y (arm joints): position from local yaw.
    YawY,
    /// Revolute about Z (wheel joints): position from local Z rotation; command
    /// is scaled to a wheel motor velocity.
    RotZ,
    /// Prismatic along Y (vertical lift): position from local Y translation.
    LiftY,
}

struct ActuatedJoint {
    link: Entity,
    axis: JointReadAxis,
}

/// Position-tracking stiffness for the vertical lift motor. The lift is a position
/// (spring-damper) motor, not a velocity motor, so it holds the arm's weight at a
/// commanded height without drift; the command integrates into the height target.
const LIFT_MOTOR_STIFFNESS: f64 = 600.0;
/// Damping for the vertical lift motor (竕・critical for the ~6 kg arm), so the lift
/// settles to its target height without oscillating.
const LIFT_MOTOR_DAMPING: f64 = 120.0;
/// Travel limits of the lift height target, in meters about its rest position.
/// The carriage rests partway up the column so the gripper can be lowered toward
/// the ground (negative) to pick and raised (positive) to carry.
const LIFT_TARGET_MIN_M: f64 = -0.5;
const LIFT_TARGET_MAX_M: f64 = 0.5;
/// Constraint solver iterations for the lift robot's world. Its tall jointed chain
/// swings chaotically at Rapier's default (4); 16 holds the arm stable.
const LIFT_SOLVER_ITERATIONS: usize = 16;
/// Position stiffness/damping for the lift robot's arm revolute joints. A plain
/// velocity motor (gain 1.0) is too weak to move or hold the heavy arm, so the arm
/// joints are position (spring-damper) motors that drive to a commanded angle and
/// hold it. Stable now that the column geometry settles the arm straight.
const ARM_MOTOR_STIFFNESS: f64 = 400.0;
const ARM_MOTOR_DAMPING: f64 = 60.0;
/// Extra stiffness when writing absolute lift-arm joint targets (direct IK hold).
const ARM_DIRECT_TARGET_STIFFNESS: f64 = 1200.0;
const ARM_DIRECT_TARGET_DAMPING: f64 = 100.0;
/// Torque cap for the lift robot's arm joints (overrides the 50 Nﾂｷm revolute default),
/// so the position motor can move and settle the heavy arm reasonably quickly.
const ARM_MOTOR_MAX_FORCE: f64 = 200.0;
/// Clamp on a position-holding arm joint's integrated angle target (radians).
const ARM_TARGET_LIMIT_RAD: f64 = std::f64::consts::PI;
/// Maximum lead the mobile arm's integrated angle target may hold over the joint's
/// measured position (radians). Anti-windup: keeps a long velocity command from
/// ramping the spring target far past the lagging joint.
const ARM_TARGET_LEAD_RAD: f64 = 0.15;
/// Nominal mobile-base center height. The URDF places the base at 0.25 m so
/// its wheels sit on the ground plane.
const MOBILE_BASE_NOMINAL_Y_M: f64 = 0.25;

/// Headless environment for minimal mobile manipulator URDFs.
pub struct MobileManipulatorSim {
    scene_path: Option<PathBuf>,
    world_seed: u64,
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    robot: Entity,
    base_link: Entity,
    ee_link: Entity,
    finger_links: Vec<Entity>,
    grasped_object: Option<Entity>,
    actuated: Vec<ActuatedJoint>,
    /// Commanded height target of the vertical lift, integrated from lift velocity.
    lift_target_m: f64,
    joint_names: Vec<String>,
    robot_links: HashMap<String, Entity>,
    named_entities: HashMap<String, Entity>,
    wrist_camera: Option<WristCameraMount>,
    wrist_camera_stream: Option<StreamId>,
    wrist_depth_stream: Option<StreamId>,
    render_backend: HeadlessRenderBackend,
    mobile_base: bool,
    /// When true, planar base motion is zeroed after each physics step (arm-only manipulation).
    base_planar_locked: bool,
    /// Commanded forward speed for the pending physics tick, used to kinematically
    /// integrate the base pose in [`Self::stabilize_mobile_base`].
    base_command_forward_m_s: f64,
    /// Commanded yaw rate for the pending physics tick (see `base_command_forward_m_s`).
    base_command_yaw_rate_rad_s: f64,
    /// Base translation captured immediately before the physics step, used as the
    /// kinematic integration's starting point.
    base_pose_before_step: (Vec3, f64),
    data_bus: InMemoryDataBus,
    joint_stream: StreamId,
    sim_time: SimTime,
    dt: SimDuration,
    step_count: u64,
    joint_sequence: u64,
}

impl MobileManipulatorSim {
    /// Creates the built-in `mm_minimal` fixed-base arm scene.
    pub fn new_mm_minimal() -> Self {
        Self::from_scene_path(&mm_minimal_scene_path()).expect("built-in mm_minimal scene")
    }

    /// Creates the built-in diff-drive base with a 2-DOF arm.
    pub fn new_mm_mobile() -> Self {
        Self::from_scene_path(&mm_mobile_scene_path()).expect("built-in mm_mobile scene")
    }

    /// Creates the built-in fixed-base arm with a vertical lift column.
    pub fn new_mm_lift() -> Self {
        Self::from_scene_path(&mm_lift_scene_path()).expect("built-in mm_lift scene")
    }

    /// Loads a `.rne.scene.toml` with a single URDF mobile-manipulator robot.
    pub fn from_scene_path(scene_path: &Path) -> Result<Self, AssetError> {
        let bundle = load_scene_bundle(scene_path)?;
        if bundle.robots.len() != 1 {
            return Err(AssetError::Invalid {
                path: scene_path.display().to_string(),
                message: format!("expected exactly one robot, found {}", bundle.robots.len()),
            });
        }

        let (_, robot_asset) = &bundle.robots[0];
        if robot_asset.urdf.is_none() {
            return Err(AssetError::Invalid {
                path: scene_path.display().to_string(),
                message: "scene robot must be kind = \"urdf\" with articulation enabled".into(),
            });
        }

        let mut world = World::new();
        let spawned_scene = load_and_spawn_scene(&mut world, scene_path)?;
        let world_seed = world
            .get::<WorldEntity>(spawned_scene.world)
            .map(|world_entity| world_entity.seed)
            .unwrap_or(0);
        let (_, spawned_robot) =
            spawned_scene
                .robots
                .first()
                .ok_or_else(|| AssetError::Invalid {
                    path: scene_path.display().to_string(),
                    message: "no robots".into(),
                })?;

        let links = collect_robot_links(&mut world, spawned_robot.robot);
        let named_entities = index_named_entities(&mut world);
        let mobile_base = links.contains_key("left_wheel");
        let (actuated, joint_names) = actuated_joints_for_robot(mobile_base, &links)?;
        let wrist_camera_mounts =
            wrist_camera_mounts_from_spawned(&spawned_scene.wrist_camera_mounts);
        let wrist_camera = wrist_camera_mounts.into_iter().next();
        let wrist_camera_stream = wrist_camera.as_ref().and_then(|mount| {
            world
                .get::<Sensor>(mount.camera)
                .map(|sensor| sensor.stream_id)
        });

        let mut sim = Self::from_spawned(
            world,
            UrdfRobot {
                name: robot_asset.model_name.clone(),
                links: Vec::new(),
                joints: Vec::new(),
            },
            spawned_robot.robot,
            spawned_robot.base_link,
            links,
            named_entities,
            actuated,
            joint_names,
            wrist_camera,
            wrist_camera_stream,
            mobile_base,
            world_seed,
            None,
        );
        sim.scene_path = Some(scene_path.to_path_buf());
        Ok(sim)
    }

    /// Returns the scene asset path when loaded from a scene file.
    pub fn scene_path(&self) -> Option<&Path> {
        self.scene_path.as_deref()
    }

    /// Resets the simulation to its initial pose.
    pub fn reset(&mut self) -> MobileManipulatorObservation {
        let scene_path = self.scene_path.clone().unwrap_or_else(|| {
            if self.mobile_base {
                mm_mobile_scene_path()
            } else {
                mm_minimal_scene_path()
            }
        });
        *self = Self::from_scene_path(&scene_path).expect("reload mobile manipulator scene");
        self.observe()
    }

    /// Applies joint velocities and advances one simulation tick.
    pub fn step(&mut self, action: MobileManipulatorAction) -> MobileManipulatorObservation {
        self.apply_action(action);
        step_physics(
            &mut self.backend,
            &mut self.world,
            self.physics_world,
            self.dt,
        )
        .expect("physics step");
        self.stabilize_mobile_base();
        self.update_grasp(action);
        if let Some(mount) = self.wrist_camera {
            sync_wrist_camera_mounts(&mut self.world, &[mount]);
            let render_scene = build_visual_render_scene(&self.world);
            sample_sensors(
                &mut SensorSampleContext {
                    world: &mut self.world,
                    sim_time: self.sim_time,
                    physics: &self.backend,
                    physics_world: self.physics_world,
                    render: Some(&mut self.render_backend),
                    scene: Some(&render_scene),
                },
                &mut self.data_bus,
            );
        }
        self.publish_joint_state();
        self.sim_time = self.sim_time + self.dt;
        self.step_count += 1;
        self.observe()
    }

    /// Returns the latest observation without stepping.
    pub fn observe(&self) -> MobileManipulatorObservation {
        let base = world_transform_of(&self.world, self.base_link);
        let ee = world_transform_of(&self.world, self.ee_link).translation;
        let shoulder = self.joint_position_rad("shoulder_joint");
        let elbow = self.joint_position_rad("elbow_joint");
        let lift_position_m = self.lift_position_m();
        let gripper_position_rad = self.gripper_position_rad();
        let joint_state_count = self
            .data_bus
            .latest::<JointState>(self.joint_stream)
            .map(|frame| frame.payload.positions_rad.len())
            .unwrap_or(0);

        let wrist_camera_pixels = self
            .wrist_camera_stream
            .and_then(|stream| {
                self.data_bus
                    .latest::<ImageRgb8>(stream)
                    .map(|frame| frame.payload.rgba8.len())
            })
            .unwrap_or(0);

        let (wrist_depth_center_m, wrist_depth_min_m) = self
            .wrist_depth_stream
            .and_then(|stream| self.data_bus.latest::<ImageDepth>(stream))
            .map(|frame| {
                (
                    f64::from(frame.payload.center_depth_m()),
                    f64::from(frame.payload.min_depth_m()),
                )
            })
            .unwrap_or((0.0, 0.0));

        MobileManipulatorObservation {
            base_x_m: base.translation.x,
            base_y_m: base.translation.y,
            base_z_m: base.translation.z,
            base_yaw_rad: yaw_rad(base.rotation),
            ee_x_m: ee.x,
            ee_y_m: ee.y,
            ee_z_m: ee.z,
            shoulder_position_rad: shoulder,
            elbow_position_rad: elbow,
            gripper_position_rad,
            lift_position_m,
            wrist_camera_pixels,
            joint_state_count,
            target_dx_m: 0.0,
            target_dy_m: 0.0,
            target_dz_m: 0.0,
            wrist_depth_center_m,
            wrist_depth_min_m,
            target_object_index: 0,
            pick_object_x_m: 0.0,
            pick_object_y_m: 0.0,
            pick_object_z_m: 0.0,
            gripper_target_dx_m: 0.0,
            gripper_target_dy_m: 0.0,
            gripper_target_dz_m: 0.0,
        }
    }

    /// Provides read access to the simulation DataBus.
    pub fn data_bus(&self) -> &InMemoryDataBus {
        &self.data_bus
    }

    /// Provides read access to the ECS world.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Returns the robot root entity.
    pub fn robot(&self) -> Entity {
        self.robot
    }

    /// Returns the end-effector link entity.
    pub fn ee_link(&self) -> Entity {
        self.ee_link
    }

    /// Returns actuated joint names in publish order.
    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    /// Returns true when the robot uses a diff-drive base.
    pub fn mobile_base(&self) -> bool {
        self.mobile_base
    }

    /// Returns the number of completed simulation steps.
    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    /// Returns the latest joint state published on the DataBus.
    pub fn latest_joint_state(&self) -> JointState {
        self.data_bus()
            .latest::<JointState>(self.joint_stream)
            .map(|frame| frame.payload.clone())
            .unwrap_or_else(|| JointState {
                names: self.joint_names.clone(),
                positions_rad: Vec::new(),
                velocities_rad_s: Vec::new(),
            })
    }

    /// Returns true when a wrist camera is configured on this robot.
    pub fn wrist_camera_enabled(&self) -> bool {
        self.wrist_camera.is_some()
    }

    /// Returns the latest wrist camera image when configured.
    pub fn latest_wrist_camera(&self) -> Option<ImageRgb8> {
        self.wrist_camera_stream.and_then(|stream| {
            self.data_bus
                .latest::<ImageRgb8>(stream)
                .map(|frame| frame.payload.clone())
        })
    }

    /// Returns the latest wrist depth frame when a wrist camera is present.
    pub fn latest_wrist_depth(&self) -> Option<ImageDepth> {
        self.wrist_depth_stream.and_then(|stream| {
            self.data_bus
                .latest::<ImageDepth>(stream)
                .map(|frame| frame.payload.clone())
        })
    }

    /// Returns the joint-state stream identifier.
    pub fn joint_stream(&self) -> StreamId {
        self.joint_stream
    }

    /// Returns the world seed from a loaded scene, or zero when unspecified.
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

    /// Captures a completed-tick simulation snapshot.
    pub fn snapshot(&self) -> MobileManipulatorSimSnapshot {
        let mut snapshot = MobileManipulatorSimSnapshot {
            schema_version: MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
            sim_ticks: self.sim_time.ticks(),
            step_count: self.step_count,
            joint_sequence: self.joint_sequence,
            lift_target_m: self.lift_target_m,
            grasped_object_index: self.grasped_object.map(Entity::index),
            world_random: self.world_random_snapshot(),
            transforms: Vec::new(),
            rigid_bodies: Vec::new(),
            joint_motors: Vec::new(),
            fixed_joints: Vec::new(),
            sensor_states: Vec::new(),
            joint_state_frame: self
                .data_bus
                .latest::<JointState>(self.joint_stream)
                .map(MobileManipulatorFrameSnapshot::from_frame),
            wrist_camera_frame: self.wrist_camera_stream.and_then(|stream| {
                self.data_bus
                    .latest::<ImageRgb8>(stream)
                    .map(MobileManipulatorFrameSnapshot::from_frame)
            }),
            wrist_depth_frame: self.wrist_depth_stream.and_then(|stream| {
                self.data_bus
                    .latest::<ImageDepth>(stream)
                    .map(MobileManipulatorFrameSnapshot::from_frame)
            }),
        };

        for entity in sorted_world_entities(&self.world) {
            let entity_index = entity.index();
            if let Some(transform) = self.world.get::<Transform3>(entity) {
                snapshot
                    .transforms
                    .push(MobileManipulatorTransformSnapshot {
                        entity_index,
                        transform: *transform,
                    });
            }
            if let Some(body) = self.world.get::<RigidBody>(entity) {
                snapshot
                    .rigid_bodies
                    .push(MobileManipulatorRigidBodySnapshot {
                        entity_index,
                        linear_velocity_m_s: body.linear_velocity_m_s,
                        angular_velocity_rad_s: body.angular_velocity_rad_s,
                    });
            }
            if let Some(motor) = self.world.get::<JointMotor>(entity) {
                snapshot
                    .joint_motors
                    .push(MobileManipulatorJointMotorSnapshot {
                        entity_index,
                        motor: *motor,
                    });
            }
            if let Some(desc) = self.world.get::<FixedJointDesc>(entity) {
                snapshot
                    .fixed_joints
                    .push(MobileManipulatorFixedJointSnapshot {
                        entity_index,
                        parent_index: desc.parent.index(),
                        anchor_parent_m: desc.anchor_parent_m,
                        anchor_child_m: desc.anchor_child_m,
                        relative_rotation: desc.relative_rotation,
                    });
            }
            if let Some(state) = self.world.get::<SensorState>(entity) {
                snapshot
                    .sensor_states
                    .push(MobileManipulatorSensorStateSnapshot {
                        entity_index,
                        last_sequence: state.last_sequence,
                        last_sample_ticks: state.last_sample_ticks,
                        frame_count: state.frame_count,
                    });
            }
        }

        snapshot
    }

    /// Restores this simulation from a completed-tick snapshot.
    ///
    /// # Errors
    ///
    /// Returns an error if the snapshot schema is unsupported, if it references
    /// missing entities/components, or if the physics backend cannot be rebuilt.
    pub fn restore_snapshot(
        &mut self,
        snapshot: &MobileManipulatorSimSnapshot,
    ) -> Result<(), MobileManipulatorSimSnapshotError> {
        if snapshot.schema_version < MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION
            || snapshot.schema_version > MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION
        {
            return Err(
                MobileManipulatorSimSnapshotError::UnsupportedSchemaVersion {
                    expected: MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
                    actual: snapshot.schema_version,
                },
            );
        }

        for item in &snapshot.transforms {
            *component_mut::<Transform3>(&mut self.world, item.entity_index)? = item.transform;
        }
        for item in &snapshot.rigid_bodies {
            let mut body = component_mut::<RigidBody>(&mut self.world, item.entity_index)?;
            body.linear_velocity_m_s = item.linear_velocity_m_s;
            body.angular_velocity_rad_s = item.angular_velocity_rad_s;
        }
        for item in &snapshot.joint_motors {
            *component_mut::<JointMotor>(&mut self.world, item.entity_index)? = item.motor;
        }

        for entity in sorted_world_entities(&self.world) {
            self.world.entity_mut(entity).remove::<FixedJointDesc>();
        }
        for item in &snapshot.fixed_joints {
            if !entity_exists(&self.world, item.parent_index) {
                return Err(MobileManipulatorSimSnapshotError::MissingEntity {
                    entity_index: item.parent_index,
                });
            }
            let entity = Entity::from_raw(item.entity_index);
            if !entity_exists(&self.world, item.entity_index) {
                return Err(MobileManipulatorSimSnapshotError::MissingEntity {
                    entity_index: item.entity_index,
                });
            }
            self.world.entity_mut(entity).insert(FixedJointDesc {
                parent: Entity::from_raw(item.parent_index),
                anchor_parent_m: item.anchor_parent_m,
                anchor_child_m: item.anchor_child_m,
                relative_rotation: item.relative_rotation,
            });
        }

        for item in &snapshot.sensor_states {
            let mut state = component_mut::<SensorState>(&mut self.world, item.entity_index)?;
            state.last_sequence = item.last_sequence;
            state.last_sample_ticks = item.last_sample_ticks;
            state.frame_count = item.frame_count;
        }

        if let Some(entity_index) = snapshot.grasped_object_index {
            if !entity_exists(&self.world, entity_index) {
                return Err(MobileManipulatorSimSnapshotError::MissingEntity { entity_index });
            }
            self.grasped_object = Some(Entity::from_raw(entity_index));
        } else {
            self.grasped_object = None;
        }

        self.sim_time = SimTime::from_ticks(snapshot.sim_ticks);
        self.step_count = snapshot.step_count;
        self.joint_sequence = snapshot.joint_sequence;
        self.lift_target_m = snapshot.lift_target_m;
        self.restore_world_random_snapshot(snapshot.world_random);
        self.data_bus = InMemoryDataBus::new();
        if let Some(frame) = &snapshot.joint_state_frame {
            self.data_bus.publish(frame.to_frame());
        }
        if let Some(frame) = &snapshot.wrist_camera_frame {
            self.data_bus.publish(frame.to_frame());
        }
        if let Some(frame) = &snapshot.wrist_depth_frame {
            self.data_bus.publish(frame.to_frame());
        }
        self.rebuild_physics_from_ecs()?;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn from_spawned(
        world: World,
        _urdf: UrdfRobot,
        robot: Entity,
        base_link: Entity,
        links: HashMap<String, Entity>,
        named_entities: HashMap<String, Entity>,
        actuated: Vec<ActuatedJoint>,
        joint_names: Vec<String>,
        wrist_camera: Option<WristCameraMount>,
        wrist_camera_stream: Option<StreamId>,
        mobile_base: bool,
        world_seed: u64,
        _base_y_m: Option<f64>,
    ) -> Self {
        let mut backend = RapierBackend::new();
        // The lift robot's tall jointed chain (lift + shoulder + elbow + gripper) needs
        // more constraint-solver iterations to stay stable, and so does the mobile
        // robot's position-held arm on its floating diff-drive base; fixed-base robots
        // keep the default.
        let has_lift = actuated.iter().any(|j| j.axis == JointReadAxis::LiftY);
        let physics_world = backend
            .create_world(PhysicsWorldDesc {
                solver_iterations: if has_lift || mobile_base {
                    LIFT_SOLVER_ITERATIONS
                } else {
                    0
                },
                ..PhysicsWorldDesc::default()
            })
            .expect("physics world");

        let ee_link = links
            .get("forearm_link")
            .copied()
            .expect("forearm_link missing from URDF robot");
        let finger_links = ["left_finger_link", "right_finger_link"]
            .iter()
            .filter_map(|name| links.get(*name).copied())
            .collect();

        let wrist_depth_stream = wrist_camera_stream.map(wrist_camera_depth_stream);

        let mut sim = Self {
            scene_path: None,
            world_seed,
            world,
            backend,
            physics_world,
            robot,
            base_link,
            ee_link,
            finger_links,
            grasped_object: None,
            actuated,
            lift_target_m: 0.0,
            joint_names,
            robot_links: links,
            named_entities,
            wrist_camera,
            wrist_camera_stream,
            wrist_depth_stream,
            render_backend: HeadlessRenderBackend::new(),
            mobile_base,
            base_planar_locked: false,
            base_command_forward_m_s: 0.0,
            base_command_yaw_rate_rad_s: 0.0,
            base_pose_before_step: (Vec3::ZERO, 0.0),
            data_bus: InMemoryDataBus::new(),
            joint_stream: StreamId::new(JOINT_STATE_STREAM as u64),
            sim_time: SimTime::ZERO,
            dt: SimDuration::from_hertz(Hertz::new(60.0)),
            step_count: 0,
            joint_sequence: 0,
        };
        sim.configure_lift_motor();
        sim.configure_mobile_arm_motors();
        sim.warmup_physics();
        sim
    }

    /// Returns contact events produced by the last physics step.
    pub fn last_contacts(&self) -> &[ContactEvent] {
        self.backend.contacts(self.physics_world).unwrap_or(&[])
    }

    /// Returns the first entity with the given ECS name.
    pub fn entity_named(&self, name: &str) -> Option<Entity> {
        self.named_entities.get(name).copied()
    }

    /// Returns the world-frame translation of a named entity.
    pub fn named_translation_m(&self, name: &str) -> Option<(f64, f64, f64)> {
        self.entity_named(name).map(|entity| {
            let translation = world_transform_of(&self.world, entity).translation;
            (translation.x, translation.y, translation.z)
        })
    }

    /// Returns the prismatic lift displacement in meters (zero when absent).
    pub fn lift_position_m(&self) -> f64 {
        self.joint_position_rad("lift_joint")
    }

    /// Drives the `mm_lift` lift / shoulder / elbow position motors to absolute targets.
    pub fn set_lift_joint_targets(&mut self, target: crate::mm_lift_kinematics::MmLiftJointTarget) {
        self.apply_lift_joint_targets(target);
    }

    /// Returns the world-frame translation of a URDF link on this robot.
    pub fn link_translation_m(&self, link_name: &str) -> Option<(f64, f64, f64)> {
        self.robot_links.get(link_name).map(|entity| {
            let translation = world_transform_of(&self.world, *entity).translation;
            (translation.x, translation.y, translation.z)
        })
    }

    /// Returns true when the two entities contacted on the last physics step.
    pub fn contacts_between(&self, entity_a: Entity, entity_b: Entity) -> bool {
        self.last_contacts().iter().any(|contact| {
            (contact.entity_a == entity_a && contact.entity_b == entity_b)
                || (contact.entity_a == entity_b && contact.entity_b == entity_a)
        })
    }

    /// Returns true when an object is currently welded into the gripper.
    pub fn is_grasping(&self) -> bool {
        self.grasped_object.is_some()
    }

    /// Returns the entity of the currently grasped object, if any.
    pub fn grasped_object(&self) -> Option<Entity> {
        self.grasped_object
    }

    /// Attaches or releases a grasp based on the gripper command and finger contacts.
    ///
    /// Closing the gripper (`gripper_velocity_rad_s` below a small negative threshold)
    /// onto a graspable body welds it to the end-effector link at its current relative
    /// pose; opening the gripper releases the weld. This contact-triggered weld is a
    /// robust stand-in for friction-based grasping.
    fn update_grasp(&mut self, action: MobileManipulatorAction) {
        const CLOSE_THRESHOLD_RAD_S: f64 = -0.05;
        const OPEN_THRESHOLD_RAD_S: f64 = 0.05;
        let command = action.gripper_velocity_rad_s;

        if self.grasped_object.is_none() && command < CLOSE_THRESHOLD_RAD_S {
            if let Some(object) = self.find_graspable_in_contact() {
                self.attach_grasp(object);
            }
        } else if self.grasped_object.is_some() && command > OPEN_THRESHOLD_RAD_S {
            self.release_grasp();
        }
    }

    /// Finds a graspable body currently contacting a gripper finger.
    fn find_graspable_in_contact(&self) -> Option<Entity> {
        for contact in self.last_contacts() {
            for finger in &self.finger_links {
                let other = if contact.entity_a == *finger {
                    Some(contact.entity_b)
                } else if contact.entity_b == *finger {
                    Some(contact.entity_a)
                } else {
                    None
                };
                if let Some(other) = other {
                    if self.is_graspable(other) {
                        return Some(other);
                    }
                }
            }
        }
        None
    }

    /// A body is graspable when it is dynamic and not part of the robot articulation.
    fn is_graspable(&self, entity: Entity) -> bool {
        let dynamic = self
            .world
            .get::<RigidBody>(entity)
            .map(|body| body.body_type == RigidBodyType::Dynamic)
            .unwrap_or(false);
        let is_robot_link = self
            .world
            .get::<Link>(entity)
            .map(|link| link.robot == self.robot)
            .unwrap_or(false);
        dynamic && !is_robot_link
    }

    /// Welds `object` to the end-effector link, preserving its current relative pose.
    fn attach_grasp(&mut self, object: Entity) {
        let ee = world_transform_of(&self.world, self.ee_link);
        let obj = world_transform_of(&self.world, object);
        let ee_rotation_inverse = ee.rotation.inverse();
        let relative_translation = ee_rotation_inverse * (obj.translation - ee.translation);
        let relative_rotation = ee_rotation_inverse * obj.rotation;
        self.world.entity_mut(object).insert(FixedJointDesc {
            parent: self.ee_link,
            anchor_parent_m: relative_translation,
            anchor_child_m: Vec3::ZERO,
            relative_rotation,
        });
        self.grasped_object = Some(object);
    }

    /// Releases the current grasp by removing the weld joint.
    fn release_grasp(&mut self) {
        if let Some(object) = self.grasped_object.take() {
            self.world.entity_mut(object).remove::<FixedJointDesc>();
        }
    }

    fn joint_position_rad(&self, joint_name: &str) -> f64 {
        let index = self.joint_names.iter().position(|name| name == joint_name);
        index
            .map(|idx| joint_sample(&self.world, &self.actuated[idx]).position_rad)
            .unwrap_or(0.0)
    }

    fn gripper_position_rad(&self) -> f64 {
        let left = self.joint_position_rad("left_finger_joint");
        let right = self.joint_position_rad("right_finger_joint");
        if self
            .joint_names
            .iter()
            .any(|name| name == "left_finger_joint")
        {
            0.5 * (left - right)
        } else {
            0.0
        }
    }

    fn apply_action(&mut self, action: MobileManipulatorAction) {
        if let Some(target) = action.lift_joint_target {
            self.apply_lift_joint_targets(target);
            self.apply_gripper_and_base_velocities(action);
            return;
        }

        // Integrate the lift command into a height target so the position motor
        // holds the commanded height instead of drifting under the arm's weight.
        let dt_s = self.dt.as_seconds().value();
        self.lift_target_m = (self.lift_target_m + action.lift_velocity_m_s * dt_s)
            .clamp(LIFT_TARGET_MIN_M, LIFT_TARGET_MAX_M);

        for (index, (joint, joint_name)) in self
            .actuated
            .iter()
            .zip(self.joint_names.iter())
            .enumerate()
        {
            let velocity = velocity_for_joint(joint_name, action);
            // Anti-windup lead for the mobile arm: without it the integrated angle
            // target runs several radians ahead of the lagging joint during long
            // moves, and the spring then drags the joint far past the commanded
            // stop, oscillating the carried payload. Applied only while a velocity
            // command is integrating: at zero command the target must hold firm,
            // otherwise external disturbances (payload swings, base turns) re-base
            // the clamped target onto the back-driven joint and permanently deform
            // the held pose instead of springing back.
            let windup_position_rad =
                (self.mobile_base && joint.axis == JointReadAxis::YawY && velocity != 0.0)
                    .then(|| joint_sample(&self.world, &self.actuated[index]).position_rad);
            if let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) {
                if joint.axis == JointReadAxis::LiftY {
                    // Position (spring-damper) control with the velocity as feedforward.
                    motor.velocity_rad_s = velocity;
                    motor.target_position = self.lift_target_m;
                } else if motor.stiffness > 0.0 {
                    // Position-holding revolute arm joint: integrate the velocity command
                    // into a held angle so the heavy arm moves to and holds the commanded
                    // pose (a plain velocity motor is too weak to move or hold it).
                    motor.stiffness = ARM_MOTOR_STIFFNESS;
                    motor.gain = ARM_MOTOR_DAMPING;
                    let mut target = (motor.target_position + velocity * dt_s)
                        .clamp(-ARM_TARGET_LIMIT_RAD, ARM_TARGET_LIMIT_RAD);
                    if let Some(position_rad) = windup_position_rad {
                        target = target.clamp(
                            position_rad - ARM_TARGET_LEAD_RAD,
                            position_rad + ARM_TARGET_LEAD_RAD,
                        );
                    }
                    motor.target_position = target;
                    motor.velocity_rad_s = velocity;
                } else {
                    motor.velocity_rad_s = if joint.axis == JointReadAxis::RotZ {
                        wheel_command_to_motor_rad_s(velocity)
                    } else {
                        velocity
                    };
                }
            }
        }
        self.apply_mobile_base_planar_drive(action);
    }

    fn apply_lift_joint_targets(&mut self, target: crate::mm_lift_kinematics::MmLiftJointTarget) {
        self.lift_target_m = target.lift_m.clamp(LIFT_TARGET_MIN_M, LIFT_TARGET_MAX_M);
        for (joint, joint_name) in self.actuated.iter().zip(self.joint_names.iter()) {
            let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) else {
                continue;
            };
            match joint_name.as_str() {
                "lift_joint" => {
                    motor.target_position = self.lift_target_m;
                    motor.velocity_rad_s = 0.0;
                }
                "shoulder_joint" => {
                    motor.target_position = target
                        .shoulder_rad
                        .clamp(-ARM_TARGET_LIMIT_RAD, ARM_TARGET_LIMIT_RAD);
                    motor.velocity_rad_s = 0.0;
                    motor.stiffness = ARM_DIRECT_TARGET_STIFFNESS;
                    motor.gain = ARM_DIRECT_TARGET_DAMPING;
                }
                "elbow_joint" => {
                    motor.target_position = target
                        .elbow_rad
                        .clamp(-ARM_TARGET_LIMIT_RAD, ARM_TARGET_LIMIT_RAD);
                    motor.velocity_rad_s = 0.0;
                    motor.stiffness = ARM_DIRECT_TARGET_STIFFNESS;
                    motor.gain = ARM_DIRECT_TARGET_DAMPING;
                }
                _ => {}
            }
        }
    }

    fn apply_gripper_and_base_velocities(&mut self, action: MobileManipulatorAction) {
        for (joint, joint_name) in self.actuated.iter().zip(self.joint_names.iter()) {
            let velocity = velocity_for_joint(joint_name, action);
            if matches!(
                joint_name.as_str(),
                "lift_joint" | "shoulder_joint" | "elbow_joint"
            ) {
                continue;
            }
            if let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) {
                motor.velocity_rad_s = if joint.axis == JointReadAxis::RotZ {
                    wheel_command_to_motor_rad_s(velocity)
                } else {
                    velocity
                };
            }
        }
        self.apply_mobile_base_planar_drive(action);
    }

    fn apply_mobile_base_planar_drive(&mut self, action: MobileManipulatorAction) {
        if !self.mobile_base {
            return;
        }

        let left_m_s = action.left_wheel_velocity_rad_s * MM_MOBILE_WHEEL_RADIUS_M;
        let right_m_s = action.right_wheel_velocity_rad_s * MM_MOBILE_WHEEL_RADIUS_M;
        let forward_m_s = 0.5 * (left_m_s + right_m_s);
        let yaw_rate_rad_s = (right_m_s - left_m_s) / MM_MOBILE_TRACK_WIDTH_M;
        self.base_planar_locked = forward_m_s.abs() < 1.0e-9 && yaw_rate_rad_s.abs() < 1.0e-9;
        self.base_command_forward_m_s = forward_m_s;
        self.base_command_yaw_rate_rad_s = yaw_rate_rad_s;
        let transform = world_transform_of(&self.world, self.base_link);
        let yaw = planar_yaw_rad(transform.rotation);
        self.base_pose_before_step = (transform.translation, yaw);
        let forward = Quat::from_rotation_y(yaw) * Vec3::X;

        if let Some(mut body) = self.world.get_mut::<RigidBody>(self.base_link) {
            body.linear_velocity_m_s = forward * forward_m_s;
            body.linear_velocity_m_s.y = 0.0;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rate_rad_s, 0.0);
        }
    }

    /// Re-pins the mobile base to a deterministic kinematic pose after the physics step.
    ///
    /// The base is meant to behave as a planar diff-drive platform, but letting Rapier's
    /// dynamic solver own its XZ translation and yaw lets wheel-ground contact noise (and
    /// the arm's own overturning torque, since the arm's reach is long relative to the
    /// chassis) leak into the tracked pose, compounding tick over tick into large,
    /// unstable drift and heading oscillation under sustained driving. Since vertical
    /// position and roll/pitch were already fully re-pinned here, this extends the same
    /// treatment to X/Z translation and yaw: both are integrated analytically from the
    /// exact command applied in [`Self::apply_mobile_base_planar_drive`] and the
    /// pre-step pose, discarding whatever the dynamic solve produced for those channels.
    /// Physics still governs the arm, grasped payload, and ground/obstacle contact.
    fn stabilize_mobile_base(&mut self) {
        if !self.mobile_base {
            return;
        }

        let dt_s = self.dt.as_seconds().value();
        let (pos0, yaw0) = self.base_pose_before_step;
        let forward_dir = Quat::from_rotation_y(yaw0) * Vec3::X;
        let new_pos = pos0 + forward_dir * (self.base_command_forward_m_s * dt_s);
        // `mm_mobile_twist_to_wheel_velocities` puts the right wheel faster than the left
        // for a positive commanded yaw rate (standard diff-drive: right-faster steers
        // left, i.e. toward -Z given the base's `Quat::from_rotation_y(yaw) * X` forward
        // axis), so a positive commanded yaw rate increases yaw (see
        // `mobile_twist_positive_yaw_rate_increases_observed_yaw`). This matches the
        // real-dynamics trajectory example 32's hero script was tuned against: on the
        // dynamic path (pre-kinematic-repin) the same wheel commands steered the base
        // toward -Z to reach its pick/place targets there.
        let new_yaw = wrap_yaw_rad(yaw0 + self.base_command_yaw_rate_rad_s * dt_s);

        if let Some(mut transform) = self.world.get_mut::<Transform3>(self.base_link) {
            transform.translation.x = new_pos.x;
            transform.translation.y = MOBILE_BASE_NOMINAL_Y_M;
            transform.translation.z = new_pos.z;
            transform.rotation = Quat::from_rotation_y(new_yaw);
        }
        if let Some(mut body) = self.world.get_mut::<RigidBody>(self.base_link) {
            body.linear_velocity_m_s.y = 0.0;
            body.angular_velocity_rad_s.x = 0.0;
            body.angular_velocity_rad_s.z = 0.0;
            if self.base_planar_locked {
                body.linear_velocity_m_s.x = 0.0;
                body.linear_velocity_m_s.z = 0.0;
                body.angular_velocity_rad_s.y = 0.0;
            }
        }
    }

    /// Configures the mobile base robot's shoulder/elbow as position (spring-damper)
    /// motors, mirroring the lift robot's arm setup: on the floating diff-drive base a
    /// plain velocity motor cannot hold the extended arm against gravity, so it droops
    /// onto scene geometry and whips when the base moves. The position hold keeps the
    /// arm at its commanded pose while driving and lets velocity commands integrate
    /// into tracked angle targets (see `apply_action`).
    fn configure_mobile_arm_motors(&mut self) {
        if !self.mobile_base {
            return;
        }
        for (joint, name) in self.actuated.iter().zip(self.joint_names.iter()) {
            if name != "shoulder_joint" && name != "elbow_joint" {
                continue;
            }
            let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) else {
                continue;
            };
            motor.stiffness = ARM_MOTOR_STIFFNESS;
            motor.gain = ARM_MOTOR_DAMPING;
            motor.target_position = 0.0;
            motor.max_force = ARM_MOTOR_MAX_FORCE;
        }
    }

    /// Configures the vertical lift as a position (spring-damper) motor so it holds
    /// the arm's weight against gravity at its commanded height without drift.
    fn configure_lift_motor(&mut self) {
        let has_lift = self.actuated.iter().any(|j| j.axis == JointReadAxis::LiftY);
        if !has_lift {
            return;
        }
        for (joint, name) in self.actuated.iter().zip(self.joint_names.iter()) {
            let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) else {
                continue;
            };
            if joint.axis == JointReadAxis::LiftY {
                motor.stiffness = LIFT_MOTOR_STIFFNESS;
                motor.gain = LIFT_MOTOR_DAMPING;
                motor.target_position = 0.0;
            } else if name == "shoulder_joint" || name == "elbow_joint" {
                motor.stiffness = ARM_MOTOR_STIFFNESS;
                motor.gain = ARM_MOTOR_DAMPING;
                motor.target_position = 0.0;
                motor.max_force = ARM_MOTOR_MAX_FORCE;
            }
        }
    }

    fn warmup_physics(&mut self) {
        self.zero_joint_motors();
        self.backend
            .sync_from_ecs(&mut self.world, self.physics_world)
            .expect("physics sync from ECS");
    }

    fn rebuild_physics_from_ecs(&mut self) -> Result<(), PhysicsError> {
        let has_lift = self.actuated.iter().any(|j| j.axis == JointReadAxis::LiftY);
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc {
            solver_iterations: if has_lift || self.mobile_base {
                LIFT_SOLVER_ITERATIONS
            } else {
                0
            },
            ..PhysicsWorldDesc::default()
        })?;
        backend.sync_from_ecs(&mut self.world, physics_world)?;
        self.backend = backend;
        self.physics_world = physics_world;
        Ok(())
    }

    fn zero_joint_motors(&mut self) {
        for joint in &self.actuated {
            if let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) {
                motor.velocity_rad_s = 0.0;
            }
        }
    }

    fn publish_joint_state(&mut self) {
        let mut positions_rad = Vec::with_capacity(self.actuated.len());
        let mut velocities_rad_s = Vec::with_capacity(self.actuated.len());
        for joint in &self.actuated {
            let sample = joint_sample(&self.world, joint);
            positions_rad.push(sample.position_rad);
            velocities_rad_s.push(sample.velocity_rad_s);
        }

        let frame = Frame::new(
            self.joint_stream,
            self.robot,
            self.joint_sequence,
            self.sim_time,
            JointState {
                names: self.joint_names.clone(),
                positions_rad,
                velocities_rad_s,
            },
        );
        self.data_bus.publish(frame);
        self.joint_sequence += 1;
    }
}

impl From<PhysicsError> for MobileManipulatorSimSnapshotError {
    fn from(error: PhysicsError) -> Self {
        Self::Physics(error)
    }
}

fn sorted_world_entities(world: &World) -> Vec<Entity> {
    let mut entities: Vec<Entity> = world.iter_entities().map(|entity| entity.id()).collect();
    entities.sort_unstable();
    entities
}

fn entity_exists(world: &World, entity_index: u32) -> bool {
    let entity = Entity::from_raw(entity_index);
    world
        .iter_entities()
        .any(|entity_ref| entity_ref.id() == entity)
}

fn component_mut<T: Component>(
    world: &mut World,
    entity_index: u32,
) -> Result<Mut<'_, T>, MobileManipulatorSimSnapshotError> {
    if !entity_exists(world, entity_index) {
        return Err(MobileManipulatorSimSnapshotError::MissingEntity { entity_index });
    }
    world.get_mut::<T>(Entity::from_raw(entity_index)).ok_or(
        MobileManipulatorSimSnapshotError::MissingComponent {
            entity_index,
            component: type_name::<T>(),
        },
    )
}

fn velocity_for_joint(joint_name: &str, action: MobileManipulatorAction) -> f64 {
    match joint_name {
        "left_wheel_joint" => action.left_wheel_velocity_rad_s,
        "right_wheel_joint" => action.right_wheel_velocity_rad_s,
        "lift_joint" => action.lift_velocity_m_s,
        "shoulder_joint" => action.shoulder_velocity_rad_s,
        "elbow_joint" => action.elbow_velocity_rad_s,
        "left_finger_joint" => action.gripper_velocity_rad_s,
        "right_finger_joint" => -action.gripper_velocity_rad_s,
        _ => 0.0,
    }
}

struct JointSample {
    position_rad: f64,
    velocity_rad_s: f64,
}

fn joint_sample(world: &World, joint: &ActuatedJoint) -> JointSample {
    let position_rad = world
        .get::<Transform3>(joint.link)
        .map(|transform| match joint.axis {
            JointReadAxis::RotZ => z_rotation_rad(transform.rotation),
            JointReadAxis::LiftY => transform.translation.y,
            JointReadAxis::YawY => yaw_rad(transform.rotation),
        })
        .unwrap_or(0.0);
    let velocity_rad_s = world
        .get::<rne_physics::JointMotor>(joint.link)
        .map(|motor| motor.velocity_rad_s)
        .unwrap_or(0.0);

    JointSample {
        position_rad,
        velocity_rad_s,
    }
}

fn z_rotation_rad(rotation: Quat) -> f64 {
    2.0 * f64::atan2(rotation.z, rotation.w)
}

/// Planar heading in radians, extracted by projecting the rotated +X axis onto the
/// world XZ plane rather than taking a raw Euler `yaw_rad` decomposition.
///
/// For a pure yaw rotation the two agree exactly, but `yaw_rad`'s YXZ Euler
/// decomposition can alias transient roll/pitch (e.g. from a single physics tick of
/// wheel-ground contact) into a badly corrupted "yaw" value. Projecting onto the
/// horizontal plane recovers the intended planar heading even when tilt is present.
fn planar_yaw_rad(rotation: Quat) -> f64 {
    let forward = rotation * Vec3::X;
    (-forward.z).atan2(forward.x)
}

/// Wraps an angle in radians to `(-PI, PI]`.
fn wrap_yaw_rad(angle_rad: f64) -> f64 {
    let mut wrapped = angle_rad % std::f64::consts::TAU;
    if wrapped <= -std::f64::consts::PI {
        wrapped += std::f64::consts::TAU;
    } else if wrapped > std::f64::consts::PI {
        wrapped -= std::f64::consts::TAU;
    }
    wrapped
}

fn collect_robot_links(world: &mut World, robot: Entity) -> HashMap<String, Entity> {
    let mut links = HashMap::new();
    let mut query = world.query::<(Entity, &Link)>();
    for (entity, link) in query.iter(world) {
        if link.robot == robot {
            links.insert(link.name.clone(), entity);
        }
    }
    links
}

fn index_named_entities(world: &mut World) -> HashMap<String, Entity> {
    let mut names = HashMap::new();
    let mut query = world.query::<(Entity, &rne_ecs::Name)>();
    for (entity, name) in query.iter(world) {
        names.insert(name.0.clone(), entity);
    }
    names
}

fn actuated_joints_for_robot(
    mobile_base: bool,
    links: &HashMap<String, Entity>,
) -> Result<(Vec<ActuatedJoint>, Vec<String>), AssetError> {
    let mut joints: Vec<ActuatedJoint> = Vec::new();
    let mut names: Vec<String> = Vec::new();

    if mobile_base {
        joints.push(ActuatedJoint {
            link: link_entity(links, "left_wheel")?,
            axis: JointReadAxis::RotZ,
        });
        joints.push(ActuatedJoint {
            link: link_entity(links, "right_wheel")?,
            axis: JointReadAxis::RotZ,
        });
        names.push("left_wheel_joint".into());
        names.push("right_wheel_joint".into());
    }

    // Optional vertical lift carriage between the base and the shoulder.
    if let Ok(torso) = link_entity(links, "torso_link") {
        joints.push(ActuatedJoint {
            link: torso,
            axis: JointReadAxis::LiftY,
        });
        names.push("lift_joint".into());
    }

    joints.push(ActuatedJoint {
        link: link_entity(links, "upper_arm_link")?,
        axis: JointReadAxis::YawY,
    });
    joints.push(ActuatedJoint {
        link: link_entity(links, "forearm_link")?,
        axis: JointReadAxis::YawY,
    });
    names.push("shoulder_joint".into());
    names.push("elbow_joint".into());

    Ok(append_gripper_joints(joints, names, links))
}

fn append_gripper_joints(
    mut joints: Vec<ActuatedJoint>,
    mut names: Vec<String>,
    links: &HashMap<String, Entity>,
) -> (Vec<ActuatedJoint>, Vec<String>) {
    if let (Ok(left), Ok(right)) = (
        link_entity(links, "left_finger_link"),
        link_entity(links, "right_finger_link"),
    ) {
        joints.push(ActuatedJoint {
            link: left,
            axis: JointReadAxis::YawY,
        });
        joints.push(ActuatedJoint {
            link: right,
            axis: JointReadAxis::YawY,
        });
        names.push("left_finger_joint".into());
        names.push("right_finger_joint".into());
    }
    (joints, names)
}

fn link_entity(links: &HashMap<String, Entity>, name: &str) -> Result<Entity, AssetError> {
    links.get(name).copied().ok_or_else(|| AssetError::Invalid {
        path: "mobile_manipulator".into(),
        message: format!("missing link `{name}`"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_render::{Visual, VisualShape};

    #[test]
    fn physics_init_produces_repeatable_pose() {
        let reference = {
            let sim = MobileManipulatorSim::new_mm_minimal();
            sim.observe()
        };

        for attempt in 1..5 {
            let sim = MobileManipulatorSim::new_mm_minimal();
            let obs = sim.observe();
            let dx = (obs.ee_x_m - reference.ee_x_m).abs();
            let dy = (obs.ee_y_m - reference.ee_y_m).abs();
            let dz = (obs.ee_z_m - reference.ee_z_m).abs();
            assert!(
                dx < 1e-4 && dy < 1e-4 && dz < 1e-4,
                "attempt {attempt}: ee drift ({dx}, {dy}, {dz}) m"
            );
        }
    }

    #[test]
    fn shoulder_velocity_moves_end_effector() {
        let mut sim = MobileManipulatorSim::new_mm_minimal();
        let initial = sim.observe();
        for _ in 0..720 {
            sim.step(MobileManipulatorAction {
                left_wheel_velocity_rad_s: 0.0,
                right_wheel_velocity_rad_s: 0.0,
                shoulder_velocity_rad_s: 3.0,
                elbow_velocity_rad_s: 0.0,
                gripper_velocity_rad_s: 0.0,
                lift_velocity_m_s: 0.0,
                ..MobileManipulatorAction::default()
            });
        }
        let final_obs = sim.observe();
        let displacement = ((final_obs.ee_x_m - initial.ee_x_m).powi(2)
            + (final_obs.ee_y_m - initial.ee_y_m).powi(2)
            + (final_obs.ee_z_m - initial.ee_z_m).powi(2))
        .sqrt();
        let shoulder_delta =
            (final_obs.shoulder_position_rad - initial.shoulder_position_rad).abs();
        assert!(
            displacement > 0.015 || shoulder_delta > 0.03,
            "ee displacement {displacement} m shoulder_delta {shoulder_delta} rad"
        );
    }

    #[test]
    fn joint_state_publishes_to_data_bus() {
        let mut sim = MobileManipulatorSim::new_mm_minimal();
        sim.step(MobileManipulatorAction {
            left_wheel_velocity_rad_s: 0.0,
            right_wheel_velocity_rad_s: 0.0,
            shoulder_velocity_rad_s: 1.0,
            elbow_velocity_rad_s: 0.5,
            gripper_velocity_rad_s: 0.0,
            lift_velocity_m_s: 0.0,
            ..MobileManipulatorAction::default()
        });
        let obs = sim.observe();
        assert_eq!(obs.joint_state_count, 4);
        let frame = sim
            .data_bus()
            .latest::<JointState>(StreamId::new(JOINT_STATE_STREAM as u64))
            .expect("joint state frame");
        assert_eq!(frame.payload.positions_rad.len(), 4);
        assert_eq!(
            frame.payload.names,
            vec![
                "shoulder_joint",
                "elbow_joint",
                "left_finger_joint",
                "right_finger_joint"
            ]
        );
    }

    #[test]
    fn snapshot_restores_observation_and_data_bus_frames() {
        let mut sim = MobileManipulatorSim::new_mm_minimal();
        sim.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: 1.0,
            elbow_velocity_rad_s: 0.5,
            gripper_velocity_rad_s: -0.25,
            ..MobileManipulatorAction::default()
        });

        let snapshot = sim.snapshot();
        let observation_at_snapshot = sim.observe();
        let joint_state_at_snapshot = sim.latest_joint_state();
        let wrist_camera_at_snapshot = sim.latest_wrist_camera();

        sim.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: -2.0,
            elbow_velocity_rad_s: 1.0,
            gripper_velocity_rad_s: 1.5,
            ..MobileManipulatorAction::default()
        });
        sim.step(MobileManipulatorAction::default());

        sim.restore_snapshot(&snapshot).unwrap();

        assert_eq!(sim.observe(), observation_at_snapshot);
        assert_eq!(sim.latest_joint_state(), joint_state_at_snapshot);
        assert_eq!(sim.latest_wrist_camera(), wrist_camera_at_snapshot);
        assert_eq!(sim.step_count(), snapshot.step_count);
        assert_eq!(sim.snapshot(), snapshot);
    }

    #[test]
    fn mobile_base_drives_forward() {
        let mut sim = MobileManipulatorSim::new_mm_mobile();
        let initial = sim.observe();
        for _ in 0..240 {
            sim.step(MobileManipulatorAction {
                left_wheel_velocity_rad_s: 6.0,
                right_wheel_velocity_rad_s: 6.0,
                shoulder_velocity_rad_s: 0.0,
                elbow_velocity_rad_s: 0.0,
                gripper_velocity_rad_s: 0.0,
                lift_velocity_m_s: 0.0,
                ..MobileManipulatorAction::default()
            });
        }
        let final_obs = sim.observe();
        let delta_x = final_obs.base_x_m - initial.base_x_m;
        assert!(
            delta_x.abs() > 0.05,
            "expected base translation, delta_x={delta_x}"
        );
        assert_mobile_base_planar(&sim);
        assert_eq!(sim.joint_names().len(), 6);
    }

    #[test]
    fn mobile_wheel_visuals_are_lateral_disks() {
        let sim = MobileManipulatorSim::new_mm_mobile();

        for wheel_name in ["left_wheel", "right_wheel"] {
            let wheel = link_entity_named(&sim, wheel_name);
            let visual = sim
                .world
                .get::<Visual>(wheel)
                .unwrap_or_else(|| panic!("{wheel_name} should have a visual"));
            match visual.shape {
                VisualShape::Cylinder { radius_m, length_m } => {
                    assert!(
                        (radius_m - 0.1).abs() < 1.0e-9 && (length_m - 0.05).abs() < 1.0e-9,
                        "{wheel_name} should render as a thin wheel cylinder, got radius={radius_m}, length={length_m}"
                    );
                }
                ref shape => panic!("{wheel_name} should render as a cylinder, got {shape:?}"),
            }
            assert_eq!(
                visual.color_rgba,
                [0.08, 0.08, 0.08, 1.0],
                "{wheel_name} should use the URDF wheel material color"
            );

            let cylinder_axis = visual.local_offset.rotation * Vec3::Z;
            assert!(
                cylinder_axis.dot(Vec3::Z).abs() > 0.999_999 && cylinder_axis.y.abs() < 1.0e-9,
                "{wheel_name} cylinder axis should be lateral Z, got {cylinder_axis:?}"
            );
        }
    }

    #[test]
    fn mobile_base_stays_planar_during_reach_rollout() {
        let mut sim = MobileManipulatorSim::new_mm_mobile();
        for _ in 0..120 {
            sim.step(MobileManipulatorAction::default());
        }
        let start = sim.observe();

        for step in 0..420 {
            let action = match step {
                0..=90 => MobileManipulatorAction {
                    left_wheel_velocity_rad_s: 1.2,
                    right_wheel_velocity_rad_s: 1.2,
                    ..MobileManipulatorAction::default()
                },
                91..=170 => MobileManipulatorAction {
                    left_wheel_velocity_rad_s: 0.35,
                    right_wheel_velocity_rad_s: 1.0,
                    shoulder_velocity_rad_s: 0.8,
                    ..MobileManipulatorAction::default()
                },
                _ => MobileManipulatorAction {
                    shoulder_velocity_rad_s: 1.2,
                    elbow_velocity_rad_s: -0.8,
                    ..MobileManipulatorAction::default()
                },
            };
            sim.step(action);
            assert_mobile_base_planar(&sim);
        }

        let final_obs = sim.observe();
        let base_travel_m = ((final_obs.base_x_m - start.base_x_m).powi(2)
            + (final_obs.base_z_m - start.base_z_m).powi(2))
        .sqrt();
        let ee_travel_m = ((final_obs.ee_x_m - start.ee_x_m).powi(2)
            + (final_obs.ee_y_m - start.ee_y_m).powi(2)
            + (final_obs.ee_z_m - start.ee_z_m).powi(2))
        .sqrt();
        assert!(
            base_travel_m > 0.05,
            "mobile base should navigate without tipping, base_travel={base_travel_m:.3}"
        );
        assert!(
            ee_travel_m > 0.15,
            "arm should still reach while base is stabilized, ee_travel={ee_travel_m:.3}"
        );
    }

    #[test]
    fn mobile_twist_positive_yaw_rate_increases_observed_yaw() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_mobile_clutter.rne.scene.toml");
        let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("scene");
        for _ in 0..80 {
            sim.step(MobileManipulatorAction::default());
        }
        let yaw_start = sim.observe().base_yaw_rad;
        let (left, right) = crate::mm_mobile_twist_to_wheel_velocities(0.0, 0.5);
        for _ in 0..120 {
            sim.step(MobileManipulatorAction {
                left_wheel_velocity_rad_s: left,
                right_wheel_velocity_rad_s: right,
                ..MobileManipulatorAction::default()
            });
        }
        let yaw_end = sim.observe().base_yaw_rad;
        assert!(
            yaw_end > yaw_start + 0.2,
            "positive twist yaw rate increases observed base yaw in sim, start={yaw_start:.3} end={yaw_end:.3}"
        );
    }

    /// Guards the fix for mm_mobile's finger colliders: without collision geometry the
    /// finger joints never articulate and the contact-weld grasp can never fire.
    #[test]
    fn mm_mobile_gripper_fingers_articulate() {
        let mut sim = MobileManipulatorSim::new_mm_mobile();
        for _ in 0..40 {
            sim.step(MobileManipulatorAction::default());
        }
        let before = sim.observe().gripper_position_rad;
        for _ in 0..60 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
        }
        let after = sim.observe().gripper_position_rad;
        assert!(
            (after - before).abs() > 0.2,
            "mm_mobile fingers should close under a gripper command, before={before:.3} after={after:.3}"
        );
    }

    /// Guards the fix for mm_mobile's arm actuation: interpenetrating chassis/arm
    /// collision boxes used to lock the shoulder and elbow joints solid.
    #[test]
    fn mm_mobile_arm_tracks_joint_commands() {
        let mut sim = MobileManipulatorSim::new_mm_mobile();
        for _ in 0..80 {
            sim.step(MobileManipulatorAction::default());
        }
        for _ in 0..240 {
            sim.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.8,
                elbow_velocity_rad_s: -0.8,
                ..MobileManipulatorAction::default()
            });
        }
        let obs = sim.observe();
        assert!(
            obs.shoulder_position_rad > 0.8,
            "shoulder should track a sustained positive rate, got {:.3}",
            obs.shoulder_position_rad
        );
        assert!(
            obs.elbow_position_rad < -0.8,
            "elbow should track a sustained negative rate, got {:.3}",
            obs.elbow_position_rad
        );
    }

    #[test]
    fn loads_mm_mobile_scene_asset() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_mobile.rne.scene.toml");
        let sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load mm_mobile scene");
        assert!(sim.mobile_base());
        assert_eq!(sim.joint_names().len(), 6);
        assert!(sim.wrist_camera_enabled());
        assert_eq!(sim.scene_path(), Some(scene_path.as_path()));
    }

    #[test]
    fn loads_mm_minimal_scene_asset() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let sim =
            MobileManipulatorSim::from_scene_path(&scene_path).expect("load mm_minimal scene");
        assert!(!sim.mobile_base());
        assert_eq!(sim.joint_names().len(), 4);
        assert!(sim.wrist_camera_enabled());
    }

    #[test]
    fn wrist_camera_publishes_image_on_data_bus() {
        use crate::wrist_camera_image_valid;

        let scene_path = mm_minimal_grasp_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load grasp scene");
        for _ in 0..40 {
            sim.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.4,
                ..MobileManipulatorAction::default()
            });
        }
        let obs = sim.observe();
        assert!(
            obs.wrist_camera_pixels >= 64 * 48 * 4,
            "expected wrist camera pixels, got {}",
            obs.wrist_camera_pixels
        );
        let image = sim.latest_wrist_camera().expect("wrist camera frame");
        assert!(wrist_camera_image_valid(&image, 64 * 48 * 4));
        assert!(
            obs.wrist_depth_center_m > 0.0 && obs.wrist_depth_center_m < 50.0,
            "expected scene depth toward grasp cube, got {}",
            obs.wrist_depth_center_m
        );
    }

    #[test]
    fn clutter_scene_cubes_settle_on_table_after_physics() {
        let scene_path = mm_minimal_clutter_scene_path();
        let mut sim =
            MobileManipulatorSim::from_scene_path(&scene_path).expect("load clutter scene");
        for _ in 0..80 {
            sim.step(MobileManipulatorAction::default());
        }
        const TABLE_TOP_Y_M: f64 = 0.54;
        for name in ["clutter_cube_a", "clutter_cube_b", "clutter_cube_c"] {
            let (_, y, _) = sim.named_translation_m(name).expect(name);
            assert!(
                y >= TABLE_TOP_Y_M - 0.02,
                "{name} should stay on the clutter table after settle, y={y:.3} m"
            );
        }
    }

    #[test]
    fn restore_snapshot_accepts_schema_v1_without_depth_frame() {
        let mut sim = MobileManipulatorSim::from_scene_path(&mm_minimal_grasp_scene_path())
            .expect("load grasp scene");
        sim.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: 0.5,
            ..MobileManipulatorAction::default()
        });
        let mut snapshot = sim.snapshot();
        snapshot.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION;
        snapshot.wrist_depth_frame = None;
        sim.restore_snapshot(&snapshot).unwrap();
        assert_eq!(sim.step_count(), snapshot.step_count);
        assert_eq!(sim.latest_wrist_depth(), None);
    }

    #[test]
    fn mobile_clutter_scene_cubes_settle_on_table() {
        let scene_path = mm_mobile_clutter_scene_path();
        let mut sim =
            MobileManipulatorSim::from_scene_path(&scene_path).expect("load mobile clutter scene");
        for _ in 0..80 {
            sim.step(MobileManipulatorAction::default());
        }
        // `mm_mobile`'s arm has no lift joint, so the mobile clutter table sits lower
        // than the fixed-base clutter table (see `mm_mobile_clutter.rne.scene.toml`).
        const TABLE_TOP_Y_M: f64 = 0.34;
        for name in ["clutter_cube_a", "clutter_cube_b", "clutter_cube_c"] {
            let (_, y, _) = sim.named_translation_m(name).expect(name);
            assert!(
                y >= TABLE_TOP_Y_M - 0.02,
                "{name} should stay on the clutter table after settle, y={y:.3} m"
            );
        }
    }

    #[test]
    fn wrist_depth_hash_is_deterministic() {
        use rne_data::ImageDepth;

        fn depth_hash_after_steps(scene_path: &Path, steps: u64) -> u64 {
            let mut sim =
                MobileManipulatorSim::from_scene_path(scene_path).expect("load scene for depth");
            let action = MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.3,
                ..MobileManipulatorAction::default()
            };
            for _ in 0..steps {
                sim.step(action);
            }
            sim.latest_wrist_depth()
                .unwrap_or_else(|| ImageDepth::new(1, 1, vec![0.0]))
                .hash_depth()
        }

        let scene_path = mm_minimal_grasp_scene_path();
        let first = depth_hash_after_steps(&scene_path, 40);
        let second = depth_hash_after_steps(&scene_path, 40);
        assert_eq!(first, second);
        assert_ne!(first, 0, "depth hash should reflect scene geometry");
    }

    #[test]
    fn gripper_close_contacts_grasp_cube() {
        use crate::finger_contacts_named;

        let scene_path = mm_minimal_grasp_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load grasp scene");
        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };

        let mut contacted = false;
        for _ in 0..360 {
            sim.step(close);
            if finger_contacts_named(&sim, "grasp_cube") {
                contacted = true;
                break;
            }
        }
        assert!(
            contacted,
            "expected finger contact with grasp_cube while closing gripper"
        );
    }

    #[test]
    fn lift_lowers_gripper_toward_ground() {
        // Phase 1 of the manipulator redesign: the column base lets the carriage slide
        // the gripper down to near ground level so it can reach a low object.
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        let settled_y = sim.observe().ee_y_m;
        for _ in 0..240 {
            sim.step(MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                ..MobileManipulatorAction::default()
            });
        }
        let lowered_y = sim.observe().ee_y_m;
        assert!(
            lowered_y < settled_y - 0.3,
            "lift should lower the gripper toward the ground: settled_y={settled_y:.3}, lowered_y={lowered_y:.3}"
        );
        assert!(
            lowered_y < 0.35,
            "lowered gripper should reach near ground height, ee_y={lowered_y:.3}"
        );
    }

    #[test]
    fn lift_arm_tracks_and_holds_commanded_pose() {
        // Phase 2 of the manipulator redesign: the arm joints are position motors, so
        // the heavy arm moves to a commanded angle and holds it. A plain velocity motor
        // could neither move nor hold the arm (it stayed put under a velocity command).
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..120 {
            sim.step(MobileManipulatorAction::default());
        }
        let rest = sim.observe();

        // Swing the shoulder to aim the gripper sideways, then release and let it settle
        // (the position motor keeps driving to the integrated target after the command).
        for _ in 0..60 {
            sim.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 1.0,
                ..MobileManipulatorAction::default()
            });
        }
        for _ in 0..360 {
            sim.step(MobileManipulatorAction::default());
        }
        let reached = sim.observe();
        assert!(
            (reached.ee_z_m - rest.ee_z_m).abs() > 0.3,
            "shoulder command should swing the gripper sideways: rest_z={:.3}, reached_z={:.3}",
            rest.ee_z_m,
            reached.ee_z_m
        );

        // The commanded pose holds: with no command the gripper stays put.
        let mut max_drift = 0.0_f64;
        for _ in 0..90 {
            let o = sim.step(MobileManipulatorAction::default());
            let drift = ((o.ee_x_m - reached.ee_x_m).powi(2)
                + (o.ee_y_m - reached.ee_y_m).powi(2)
                + (o.ee_z_m - reached.ee_z_m).powi(2))
            .sqrt();
            max_drift = max_drift.max(drift);
        }
        assert!(
            max_drift < 0.06,
            "the commanded arm pose should hold steady, max drift={max_drift:.3} m"
        );
    }

    #[test]
    fn lift_picks_carries_and_places_cube() {
        // Phase 4 of the manipulator redesign: the full pick-and-place 窶・lower the claw
        // over a ground cube, grasp it, lift it, swing the arm to a new location, lower it
        // back down, and open to release. The cube ends resting at a different spot.
        let scene = mm_lift_pick_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene).expect("load mm_lift_pick");
        let start = sim.named_translation_m("lift_cube").expect("cube");

        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            ..MobileManipulatorAction::default()
        };

        // Settle, lower over the cube, and grasp it.
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        for _ in 0..200 {
            sim.step(MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
        }
        for _ in 0..120 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if sim.is_grasping() {
                break;
            }
        }
        assert!(sim.is_grasping(), "claw should grasp the cube");

        // Lift, swing to a new spot, and let the arm settle there (holding the grasp).
        for _ in 0..150 {
            sim.step(MobileManipulatorAction {
                lift_velocity_m_s: 0.3,
                ..close
            });
        }
        for _ in 0..90 {
            sim.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.8,
                ..close
            });
        }
        for _ in 0..150 {
            sim.step(close);
        }

        // Lower at the new spot and open to release.
        for _ in 0..200 {
            sim.step(MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                ..close
            });
        }
        for _ in 0..120 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: 3.0,
                ..MobileManipulatorAction::default()
            });
        }

        assert!(
            !sim.is_grasping(),
            "opening the claw should release the cube"
        );
        let placed = sim.named_translation_m("lift_cube").expect("cube");
        let moved = ((placed.0 - start.0).powi(2) + (placed.2 - start.2).powi(2)).sqrt();
        assert!(
            moved > 0.5,
            "cube should be carried to a new location: start=({:.2},{:.2}), placed=({:.2},{:.2}), moved={moved:.2} m",
            start.0,
            start.2,
            placed.0,
            placed.2
        );
        assert!(
            placed.1 < 0.1,
            "placed cube should rest on the ground, y={:.3}",
            placed.1
        );
    }

    enum SwingPolicyKind {
        Ik,
        Scripted,
    }

    /// Runs the pick-and-place policy with the given swing and returns the
    /// cube's resting (x, z) after release.
    fn place_location_for_swing(swing_steps: u64, kind: SwingPolicyKind) -> (f64, f64) {
        let scene = mm_lift_pick_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene).expect("load mm_lift_pick");
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        match kind {
            SwingPolicyKind::Ik => {
                let mut policy = crate::IkLiftPickPlacePolicy::with_swing_steps(swing_steps);
                for _ in 0..policy.total_steps() {
                    let obs = sim.observe();
                    sim.step(policy.next_action(&obs));
                }
            }
            SwingPolicyKind::Scripted => {
                let mut policy = crate::LiftPickPlacePolicy::with_swing_steps(swing_steps);
                for _ in 0..policy.total_steps() {
                    let obs = sim.observe();
                    sim.step(policy.next_action(&obs));
                }
            }
        }
        let placed = sim.named_translation_m("lift_cube").expect("cube");
        (placed.0, placed.2)
    }

    #[test]
    fn lift_place_swing_controls_drop_location() {
        // A longer carry swing rotates the arm further around the column, so the cube is
        // released at a different spot 窶・the place location is controllable.
        let near = place_location_for_swing(60, SwingPolicyKind::Ik);
        let far = place_location_for_swing(120, SwingPolicyKind::Ik);
        let separation = ((far.0 - near.0).powi(2) + (far.1 - near.1).powi(2)).sqrt();
        assert!(
            separation > 0.3,
            "different swings should place the cube at different spots: near={near:?}, far={far:?}, separation={separation:.2} m"
        );
    }

    #[test]
    fn scripted_lift_place_swing_controls_drop_location() {
        let near = place_location_for_swing(60, SwingPolicyKind::Scripted);
        let far = place_location_for_swing(120, SwingPolicyKind::Scripted);
        let separation = ((far.0 - near.0).powi(2) + (far.1 - near.1).powi(2)).sqrt();
        assert!(
            separation > 0.3,
            "scripted swings should place the cube at different spots: near={near:?}, far={far:?}, separation={separation:.2} m"
        );
    }

    #[test]
    fn lift_picks_cube_off_ground_and_raises_it() {
        // Phase 3 of the manipulator redesign: the top-down claw lowers over a cube on
        // the ground, grasps it (contact-triggered weld), and the lift raises it 窶・a real
        // 3D pick that the previous side-grip geometry could not perform.
        let scene = mm_lift_pick_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene).expect("load mm_lift_pick");

        // Settle, then lower the gripper down around the cube.
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        for _ in 0..200 {
            sim.step(MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                ..MobileManipulatorAction::default()
            });
        }
        // Close the claw to grasp the cube.
        for _ in 0..150 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if sim.is_grasping() {
                break;
            }
        }
        assert!(
            sim.is_grasping(),
            "claw should grasp the cube on the ground"
        );
        let grasped_y = sim.named_translation_m("lift_cube").expect("cube").1;

        // Raise the lift; the grasped cube must come up off the ground with it.
        for _ in 0..200 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: -2.0,
                lift_velocity_m_s: 0.3,
                ..MobileManipulatorAction::default()
            });
        }
        let lifted_y = sim.named_translation_m("lift_cube").expect("cube").1;
        assert!(
            lifted_y > grasped_y + 0.4,
            "grasped cube should be lifted off the ground: grasped_y={grasped_y:.3}, lifted_y={lifted_y:.3}"
        );
        assert!(
            lifted_y > 0.5,
            "lifted cube should be well off the ground, y={lifted_y:.3}"
        );
    }

    #[test]
    fn loads_mm_lift_scene_asset() {
        let sim = MobileManipulatorSim::new_mm_lift();
        assert!(!sim.mobile_base());
        assert_eq!(
            sim.joint_names(),
            &[
                "lift_joint",
                "shoulder_joint",
                "elbow_joint",
                "left_finger_joint",
                "right_finger_joint",
            ]
        );
    }

    /// Steps `count` times with `action`, returning the mean end-effector height
    /// over the final `avg` steps (smooths the arm's settling transient).
    fn mean_ee_height(
        sim: &mut MobileManipulatorSim,
        action: MobileManipulatorAction,
        count: usize,
        avg: usize,
    ) -> f64 {
        let mut sum = 0.0;
        for step in 0..count {
            let obs = sim.step(action);
            if step >= count - avg {
                sum += obs.ee_y_m;
            }
        }
        sum / avg as f64
    }

    #[test]
    fn mm_lift_arm_holds_pose_at_idle() {
        // The lift robot's tall jointed chain only stays still with enough constraint
        // solver iterations (see LIFT_SOLVER_ITERATIONS); at Rapier's default the arm
        // swings chaotically. Verify the settled arm holds its pose with no command.
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..200 {
            sim.step(MobileManipulatorAction::default());
        }
        let settled = sim.observe();
        let mut max_drift = 0.0_f64;
        for _ in 0..90 {
            let o = sim.step(MobileManipulatorAction::default());
            let drift = ((o.ee_x_m - settled.ee_x_m).powi(2)
                + (o.ee_y_m - settled.ee_y_m).powi(2)
                + (o.ee_z_m - settled.ee_z_m).powi(2))
            .sqrt();
            max_drift = max_drift.max(drift);
        }
        assert!(
            max_drift < 0.05,
            "settled lift arm should hold its pose at idle, max ee drift={max_drift:.3} m"
        );
    }

    #[test]
    fn lift_provides_controllable_vertical_motion() {
        let mut sim = MobileManipulatorSim::new_mm_lift();
        let up = MobileManipulatorAction {
            lift_velocity_m_s: 0.3,
            ..MobileManipulatorAction::default()
        };
        let down = MobileManipulatorAction {
            lift_velocity_m_s: -0.3,
            ..MobileManipulatorAction::default()
        };

        // Let the arm settle on the lift, then record its resting height. The lift
        // is a position motor, so it holds the ~6 kg arm against gravity here 窶・a
        // plain velocity motor sagged or oscillated instead.
        let baseline = mean_ee_height(&mut sim, MobileManipulatorAction::default(), 120, 30);

        // Raising the lift carries the whole arm well above the resting height.
        let raised = mean_ee_height(&mut sim, up, 180, 30);
        assert!(
            raised > baseline + 0.15,
            "lift up should raise the arm against gravity: baseline={baseline:.3}, raised={raised:.3}"
        );

        // Lowering the lift brings it back down 窶・the motion is reversible.
        let lowered = mean_ee_height(&mut sim, down, 240, 30);
        assert!(
            lowered < raised - 0.15,
            "lift down should lower the arm: raised={raised:.3}, lowered={lowered:.3}"
        );
    }

    #[test]
    fn grasp_attaches_and_releases_object() {
        let scene_path = mm_minimal_transport_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load");
        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let carry = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 0.6,
            ..MobileManipulatorAction::default()
        };
        let open = MobileManipulatorAction {
            gripper_velocity_rad_s: 3.0,
            ..MobileManipulatorAction::default()
        };

        for _ in 0..30 {
            sim.step(close);
            if sim.is_grasping() {
                break;
            }
        }
        assert!(
            sim.is_grasping(),
            "gripper should grasp the cube on contact"
        );

        // The grasped cube is carried by the arm instead of falling to the ground.
        for _ in 0..120 {
            sim.step(carry);
        }
        let carried = sim.named_translation_m("grasp_cube").unwrap();
        assert!(
            carried.1 > 0.3,
            "grasped cube should be held off the ground, y={}",
            carried.1
        );

        // Opening the gripper releases the weld.
        for _ in 0..30 {
            sim.step(open);
        }
        assert!(
            !sim.is_grasping(),
            "opening the gripper should release the grasp"
        );
    }

    #[test]
    fn fk_shoulder_sign_matches_positive_velocity_swing() {
        use crate::mm_lift_kinematics::{MmLiftJointTarget, MmLiftKinematics};

        let kin = MmLiftKinematics::mm_lift();
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..120 {
            sim.step(MobileManipulatorAction::default());
        }
        for _ in 0..90 {
            sim.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.8,
                ..MobileManipulatorAction::default()
            });
        }
        let obs = sim.observe();
        let gripper = sim
            .link_translation_m("gripper_base_link")
            .expect("gripper link");
        let fk = kin.forward_kinematics(MmLiftJointTarget {
            lift_m: sim.lift_position_m(),
            shoulder_rad: obs.shoulder_position_rad,
            elbow_rad: obs.elbow_position_rad,
        });
        let err = ((fk.x_m - gripper.0).powi(2)
            + (fk.y_m - gripper.1).powi(2)
            + (fk.z_m - gripper.2).powi(2))
        .sqrt();
        eprintln!(
            "shoulder={:.3} sim=({:.3},{:.3},{:.3}) fk=({:.3},{:.3},{:.3}) err={:.3}",
            obs.shoulder_position_rad, gripper.0, gripper.1, gripper.2, fk.x_m, fk.y_m, fk.z_m, err
        );
        assert!(
            err < 0.05,
            "FK should match sim after shoulder swing, err={err:.3} m"
        );
    }

    #[test]
    fn ik_reaches_arbitrary_target() {
        use crate::joint_trajectory::hold_lift_joint_action;
        use crate::mm_lift_kinematics::{MmLiftJointTarget, MmLiftKinematics};

        let kin = MmLiftKinematics::mm_lift();
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..120 {
            sim.step(MobileManipulatorAction::default());
        }
        let obs = sim.observe();
        let goal = MmLiftJointTarget {
            lift_m: (obs.lift_position_m - 0.05).clamp(-0.5, 0.5),
            shoulder_rad: obs.shoulder_position_rad + 0.20,
            elbow_rad: obs.elbow_position_rad + 0.15,
        };
        let target = kin.forward_kinematics(goal);
        kin.inverse_kinematics(target)
            .expect("synthesized target should lie in the analytic workspace");
        for _ in 0..480 {
            sim.step(hold_lift_joint_action(goal, 0.0));
        }

        let obs = sim.observe();
        let fk = kin.forward_kinematics(MmLiftJointTarget {
            lift_m: obs.lift_position_m,
            shoulder_rad: obs.shoulder_position_rad,
            elbow_rad: obs.elbow_position_rad,
        });
        let error_m = ((fk.x_m - target.x_m).powi(2)
            + (fk.y_m - target.y_m).powi(2)
            + (fk.z_m - target.z_m).powi(2))
        .sqrt();
        assert!(
            error_m < 0.08,
            "FK gripper pose should match IK target, error={error_m:.3} m"
        );
        assert!(
            (obs.lift_position_m - goal.lift_m).abs() < 0.06,
            "lift should reach IK target, err={:.3} m",
            (obs.lift_position_m - goal.lift_m).abs()
        );
    }

    fn assert_mobile_base_planar(sim: &MobileManipulatorSim) {
        let base = world_transform_of(&sim.world, sim.base_link);
        let yaw_only = Quat::from_rotation_y(yaw_rad(base.rotation));
        let orientation_dot = base.rotation.dot(yaw_only).abs();
        assert!(
            (base.translation.y - MOBILE_BASE_NOMINAL_Y_M).abs() < 1.0e-9,
            "mobile base height should stay planar: y={}",
            base.translation.y
        );
        assert!(
            orientation_dot > 0.999_999,
            "mobile base should keep yaw-only orientation: rotation={:?}, yaw_only={:?}",
            base.rotation,
            yaw_only
        );
    }

    fn link_entity_named(sim: &MobileManipulatorSim, name: &str) -> Entity {
        sim.world
            .iter_entities()
            .find_map(|entity| {
                let link = entity.get::<Link>()?;
                (link.name == name).then_some(entity.id())
            })
            .unwrap_or_else(|| panic!("expected link `{name}`"))
    }
}
