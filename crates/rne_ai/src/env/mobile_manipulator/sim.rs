//! Headless mobile manipulator environment (fixed-base arm and diff-drive mobile variant).

use super::drive::wheel_command_to_motor_rad_s;
use crate::action::MobileManipulatorAction;
use crate::camera::{sync_wrist_camera_mounts, wrist_camera_mounts_from_spawned, WristCameraMount};
use crate::observation::MobileManipulatorObservation;
use rne_assets::{load_and_spawn_scene, load_scene_bundle, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, ImageRgb8, InMemoryDataBus, JointState, StreamId};
use rne_ecs::{Entity, World};
use rne_math::{yaw_rad, Hertz, Quat, Vec3};
use rne_physics::{
    ContactEvent, FixedJointDesc, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::Link;
use rne_sensor::{sample_sensors, Sensor, SensorSampleContext};
use rne_urdf_import::UrdfRobot;
use rne_world::{world_transform_of, Transform3};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

const JOINT_STATE_STREAM: u32 = 300;

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
/// Damping for the vertical lift motor (≈ critical for the ~6 kg arm), so the lift
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
/// Torque cap for the lift robot's arm joints (overrides the 50 N·m revolute default),
/// so the position motor can move and settle the heavy arm reasonably quickly.
const ARM_MOTOR_MAX_FORCE: f64 = 200.0;
/// Clamp on a position-holding arm joint's integrated angle target (radians).
const ARM_TARGET_LIMIT_RAD: f64 = std::f64::consts::PI;

/// Headless environment for minimal mobile manipulator URDFs.
pub struct MobileManipulatorSim {
    scene_path: Option<PathBuf>,
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
    named_entities: HashMap<String, Entity>,
    wrist_camera: Option<WristCameraMount>,
    wrist_camera_stream: Option<StreamId>,
    mobile_base: bool,
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
        self.update_grasp(action);
        if let Some(mount) = self.wrist_camera {
            sync_wrist_camera_mounts(&mut self.world, &[mount]);
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
            wrist_camera_pixels,
            joint_state_count,
            target_dx_m: 0.0,
            target_dy_m: 0.0,
            target_dz_m: 0.0,
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

    /// Returns the joint-state stream identifier.
    pub fn joint_stream(&self) -> StreamId {
        self.joint_stream
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
        _base_y_m: Option<f64>,
    ) -> Self {
        let mut backend = RapierBackend::new();
        // The lift robot's tall jointed chain (lift + shoulder + elbow + gripper) needs
        // more constraint-solver iterations to stay stable; other robots keep the default.
        let has_lift = actuated.iter().any(|j| j.axis == JointReadAxis::LiftY);
        let physics_world = backend
            .create_world(PhysicsWorldDesc {
                solver_iterations: if has_lift { LIFT_SOLVER_ITERATIONS } else { 0 },
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

        let mut sim = Self {
            scene_path: None,
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
            named_entities,
            wrist_camera,
            wrist_camera_stream,
            mobile_base,
            data_bus: InMemoryDataBus::new(),
            joint_stream: StreamId::new(JOINT_STATE_STREAM as u64),
            sim_time: SimTime::ZERO,
            dt: SimDuration::from_hertz(Hertz::new(60.0)),
            step_count: 0,
            joint_sequence: 0,
        };
        sim.configure_lift_motor();
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
        // Integrate the lift command into a height target so the position motor
        // holds the commanded height instead of drifting under the arm's weight.
        let dt_s = self.dt.as_seconds().value();
        self.lift_target_m = (self.lift_target_m + action.lift_velocity_m_s * dt_s)
            .clamp(LIFT_TARGET_MIN_M, LIFT_TARGET_MAX_M);

        for (joint, joint_name) in self.actuated.iter().zip(self.joint_names.iter()) {
            let velocity = velocity_for_joint(joint_name, action);
            if let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) {
                if joint.axis == JointReadAxis::LiftY {
                    // Position (spring-damper) control with the velocity as feedforward.
                    motor.velocity_rad_s = velocity;
                    motor.target_position = self.lift_target_m;
                } else if motor.stiffness > 0.0 {
                    // Position-holding revolute arm joint: integrate the velocity command
                    // into a held angle so the heavy arm moves to and holds the commanded
                    // pose (a plain velocity motor is too weak to move or hold it).
                    motor.target_position = (motor.target_position + velocity * dt_s)
                        .clamp(-ARM_TARGET_LIMIT_RAD, ARM_TARGET_LIMIT_RAD);
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
            });
        }
        let final_obs = sim.observe();
        let delta_x = final_obs.base_x_m - initial.base_x_m;
        assert!(
            delta_x.abs() > 0.05,
            "expected base translation, delta_x={delta_x}"
        );
        assert_eq!(sim.joint_names().len(), 4);
    }

    #[test]
    fn loads_mm_mobile_scene_asset() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_mobile.rne.scene.toml");
        let sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load mm_mobile scene");
        assert!(sim.mobile_base());
        assert_eq!(sim.joint_names().len(), 4);
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

        let mut sim = MobileManipulatorSim::new_mm_minimal();
        for _ in 0..12 {
            sim.step(MobileManipulatorAction {
                gripper_velocity_rad_s: 0.0,
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
        // Phase 4 of the manipulator redesign: the full pick-and-place — lower the claw
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

    /// Runs the scripted pick-and-place policy with the given swing and returns the
    /// cube's resting (x, z) after release.
    fn place_location_for_swing(swing_steps: u64) -> (f64, f64) {
        let scene = mm_lift_pick_scene_path();
        let mut sim = MobileManipulatorSim::from_scene_path(&scene).expect("load mm_lift_pick");
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        let mut policy = crate::LiftPickPlacePolicy::with_swing_steps(swing_steps);
        for _ in 0..policy.total_steps() {
            sim.step(policy.next_action());
        }
        let placed = sim.named_translation_m("lift_cube").expect("cube");
        (placed.0, placed.2)
    }

    #[test]
    fn lift_place_swing_controls_drop_location() {
        // A longer carry swing rotates the arm further around the column, so the cube is
        // released at a different spot — the place location is controllable.
        let near = place_location_for_swing(60);
        let far = place_location_for_swing(120);
        let separation = ((far.0 - near.0).powi(2) + (far.1 - near.1).powi(2)).sqrt();
        assert!(
            separation > 0.3,
            "different swings should place the cube at different spots: near={near:?}, far={far:?}, separation={separation:.2} m"
        );
    }

    #[test]
    fn lift_picks_cube_off_ground_and_raises_it() {
        // Phase 3 of the manipulator redesign: the top-down claw lowers over a cube on
        // the ground, grasps it (contact-triggered weld), and the lift raises it — a real
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
        // is a position motor, so it holds the ~6 kg arm against gravity here — a
        // plain velocity motor sagged or oscillated instead.
        let baseline = mean_ee_height(&mut sim, MobileManipulatorAction::default(), 120, 30);

        // Raising the lift carries the whole arm well above the resting height.
        let raised = mean_ee_height(&mut sim, up, 180, 30);
        assert!(
            raised > baseline + 0.15,
            "lift up should raise the arm against gravity: baseline={baseline:.3}, raised={raised:.3}"
        );

        // Lowering the lift brings it back down — the motion is reversible.
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
}
