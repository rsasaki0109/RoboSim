//! Headless mobile manipulator environment (fixed-base arm and diff-drive mobile variant).

use super::drive::wheel_command_to_motor_rad_s;
use crate::action::MobileManipulatorAction;
use crate::camera::{sync_wrist_camera_mounts, wrist_camera_mounts_from_spawned, WristCameraMount};
use crate::observation::MobileManipulatorObservation;
use rne_assets::{load_and_spawn_scene, load_scene_bundle, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, ImageRgb8, InMemoryDataBus, JointState, StreamId};
use rne_ecs::{Entity, World};
use rne_math::{yaw_rad, Hertz, Quat};
use rne_physics::{ContactEvent, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId};
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

struct ActuatedJoint {
    link: Entity,
    /// When true, joint angle is read from local Z rotation (wheel joints).
    axis_z: bool,
}

/// Headless environment for minimal mobile manipulator URDFs.
pub struct MobileManipulatorSim {
    scene_path: Option<PathBuf>,
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    robot: Entity,
    base_link: Entity,
    ee_link: Entity,
    actuated: Vec<ActuatedJoint>,
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
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");

        let ee_link = links
            .get("forearm_link")
            .copied()
            .expect("forearm_link missing from URDF robot");

        let mut sim = Self {
            scene_path: None,
            world,
            backend,
            physics_world,
            robot,
            base_link,
            ee_link,
            actuated,
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
        for (joint, joint_name) in self.actuated.iter().zip(self.joint_names.iter()) {
            let velocity = velocity_for_joint(joint_name, action);
            if let Some(mut motor) = self.world.get_mut::<rne_physics::JointMotor>(joint.link) {
                motor.velocity_rad_s = if joint.axis_z {
                    wheel_command_to_motor_rad_s(velocity)
                } else {
                    velocity
                };
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
        .map(|transform| {
            if joint.axis_z {
                z_rotation_rad(transform.rotation)
            } else {
                yaw_rad(transform.rotation)
            }
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
    if mobile_base {
        let left_wheel = link_entity(links, "left_wheel")?;
        let right_wheel = link_entity(links, "right_wheel")?;
        let upper_arm = link_entity(links, "upper_arm_link")?;
        let forearm = link_entity(links, "forearm_link")?;
        Ok(append_gripper_joints(
            vec![
                ActuatedJoint {
                    link: left_wheel,
                    axis_z: true,
                },
                ActuatedJoint {
                    link: right_wheel,
                    axis_z: true,
                },
                ActuatedJoint {
                    link: upper_arm,
                    axis_z: false,
                },
                ActuatedJoint {
                    link: forearm,
                    axis_z: false,
                },
            ],
            vec![
                "left_wheel_joint".into(),
                "right_wheel_joint".into(),
                "shoulder_joint".into(),
                "elbow_joint".into(),
            ],
            links,
        ))
    } else {
        let upper_arm = link_entity(links, "upper_arm_link")?;
        let forearm = link_entity(links, "forearm_link")?;
        Ok(append_gripper_joints(
            vec![
                ActuatedJoint {
                    link: upper_arm,
                    axis_z: false,
                },
                ActuatedJoint {
                    link: forearm,
                    axis_z: false,
                },
            ],
            vec!["shoulder_joint".into(), "elbow_joint".into()],
            links,
        ))
    }
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
            axis_z: false,
        });
        joints.push(ActuatedJoint {
            link: right,
            axis_z: false,
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
}
