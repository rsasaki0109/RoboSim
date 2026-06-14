//! Headless mobile manipulator environment (fixed-base arm and diff-drive mobile variant).

use super::drive::wheel_command_to_motor_rad_s;
use crate::action::MobileManipulatorAction;
use crate::observation::MobileManipulatorObservation;
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, InMemoryDataBus, JointState, StreamId};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{yaw_rad, Hertz, Quat, Vec3};
use rne_physics::{
    Collider, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_urdf_import::{
    attach_urdf_articulation, parse_urdf, spawn_urdf_robot_with_config, UrdfArticulationConfig,
    UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};

const MM_MINIMAL_URDF: &str =
    include_str!("../../../../rne_urdf_import/tests/fixtures/mm_minimal_arm.urdf");
const MM_MOBILE_URDF: &str =
    include_str!("../../../../rne_urdf_import/tests/fixtures/mm_mobile.urdf");
const JOINT_STATE_STREAM: u32 = 300;
const DEFAULT_BASE_Y_M: f64 = 0.3;
const MOBILE_BASE_Y_M: f64 = 0.25;

struct ActuatedJoint {
    link: Entity,
    /// When true, joint angle is read from local Z rotation (wheel joints).
    axis_z: bool,
}

/// Headless environment for minimal mobile manipulator URDFs.
pub struct MobileManipulatorSim {
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    robot: Entity,
    base_link: Entity,
    ee_link: Entity,
    actuated: Vec<ActuatedJoint>,
    joint_names: Vec<String>,
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
        Self::spawn_fixed_base(MM_MINIMAL_URDF, DEFAULT_BASE_Y_M)
    }

    /// Creates the built-in diff-drive base with a 2-DOF arm.
    pub fn new_mm_mobile() -> Self {
        Self::spawn_mobile_base(MM_MOBILE_URDF, MOBILE_BASE_Y_M)
    }

    /// Resets the simulation to its initial pose.
    pub fn reset(&mut self) -> MobileManipulatorObservation {
        let replacement = if self.mobile_base {
            Self::new_mm_mobile()
        } else {
            Self::new_mm_minimal()
        };
        *self = replacement;
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
        self.publish_joint_state();
        self.sim_time = self.sim_time + self.dt;
        self.step_count += 1;
        self.observe()
    }

    /// Returns the latest observation without stepping.
    pub fn observe(&self) -> MobileManipulatorObservation {
        let base = world_transform_of(&self.world, self.base_link);
        let ee = world_transform_of(&self.world, self.ee_link).translation;
        let shoulder = joint_sample(
            &self.world,
            &self.actuated[shoulder_index(self.mobile_base)],
        );
        let elbow = joint_sample(&self.world, &self.actuated[elbow_index(self.mobile_base)]);
        let joint_state_count = self
            .data_bus
            .latest::<JointState>(self.joint_stream)
            .map(|frame| frame.payload.positions_rad.len())
            .unwrap_or(0);

        MobileManipulatorObservation {
            base_x_m: base.translation.x,
            base_y_m: base.translation.y,
            base_z_m: base.translation.z,
            base_yaw_rad: yaw_rad(base.rotation),
            ee_x_m: ee.x,
            ee_y_m: ee.y,
            ee_z_m: ee.z,
            shoulder_position_rad: shoulder.position_rad,
            elbow_position_rad: elbow.position_rad,
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

    /// Returns the joint-state stream identifier.
    pub fn joint_stream(&self) -> StreamId {
        self.joint_stream
    }

    fn spawn_fixed_base(urdf_src: &str, base_y_m: f64) -> Self {
        let urdf = parse_urdf(urdf_src).expect("parse fixed-base URDF");
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Fixed,
                ..UrdfSpawnConfig::default()
            },
        )
        .expect("spawn fixed-base URDF");

        attach_urdf_articulation(
            &mut world,
            &urdf,
            &spawned,
            UrdfArticulationConfig::default(),
        )
        .expect("attach fixed-base articulation");

        world
            .entity_mut(spawned.base_link)
            .insert(Transform3::from_translation_rotation(
                Vec3::new(0.0, base_y_m, 0.0),
                Quat::IDENTITY,
            ));

        spawn_ground(&mut world);

        let actuated = vec![
            ActuatedJoint {
                link: spawned.links["upper_arm_link"],
                axis_z: false,
            },
            ActuatedJoint {
                link: spawned.links["forearm_link"],
                axis_z: false,
            },
        ];
        let joint_names = vec!["shoulder_joint".into(), "elbow_joint".into()];

        Self::from_spawned(world, urdf, spawned, actuated, joint_names, false, base_y_m)
    }

    fn spawn_mobile_base(urdf_src: &str, base_y_m: f64) -> Self {
        let urdf = parse_urdf(urdf_src).expect("parse mobile-base URDF");
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                base_body_type: RigidBodyType::Dynamic,
                ..UrdfSpawnConfig::default()
            },
        )
        .expect("spawn mobile-base URDF");

        attach_urdf_articulation(
            &mut world,
            &urdf,
            &spawned,
            UrdfArticulationConfig {
                base_body_type: RigidBodyType::Dynamic,
                ..UrdfArticulationConfig::default()
            },
        )
        .expect("attach mobile-base articulation");

        world
            .entity_mut(spawned.base_link)
            .insert(Transform3::from_translation_rotation(
                Vec3::new(0.0, base_y_m, 0.0),
                Quat::IDENTITY,
            ));

        spawn_ground(&mut world);

        let actuated = vec![
            ActuatedJoint {
                link: spawned.links["left_wheel"],
                axis_z: true,
            },
            ActuatedJoint {
                link: spawned.links["right_wheel"],
                axis_z: true,
            },
            ActuatedJoint {
                link: spawned.links["upper_arm_link"],
                axis_z: false,
            },
            ActuatedJoint {
                link: spawned.links["forearm_link"],
                axis_z: false,
            },
        ];
        let joint_names = vec![
            "left_wheel_joint".into(),
            "right_wheel_joint".into(),
            "shoulder_joint".into(),
            "elbow_joint".into(),
        ];

        Self::from_spawned(world, urdf, spawned, actuated, joint_names, true, base_y_m)
    }

    fn from_spawned(
        world: World,
        _urdf: rne_urdf_import::UrdfRobot,
        spawned: rne_urdf_import::SpawnedUrdfRobot,
        actuated: Vec<ActuatedJoint>,
        joint_names: Vec<String>,
        mobile_base: bool,
        _base_y_m: f64,
    ) -> Self {
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");

        let mut sim = Self {
            world,
            backend,
            physics_world,
            robot: spawned.robot,
            base_link: spawned.base_link,
            ee_link: spawned.links["forearm_link"],
            actuated,
            joint_names,
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

    fn apply_action(&mut self, action: MobileManipulatorAction) {
        let velocities = if self.mobile_base {
            vec![
                action.left_wheel_velocity_rad_s,
                action.right_wheel_velocity_rad_s,
                action.shoulder_velocity_rad_s,
                action.elbow_velocity_rad_s,
            ]
        } else {
            vec![action.shoulder_velocity_rad_s, action.elbow_velocity_rad_s]
        };

        for (joint, velocity) in self.actuated.iter().zip(velocities) {
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
        step_physics(
            &mut self.backend,
            &mut self.world,
            self.physics_world,
            self.dt,
        )
        .expect("warmup physics step");
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

fn shoulder_index(mobile_base: bool) -> usize {
    if mobile_base {
        2
    } else {
        0
    }
}

fn elbow_index(mobile_base: bool) -> usize {
    if mobile_base {
        3
    } else {
        1
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

fn spawn_ground(world: &mut World) {
    let ground = spawn_named(world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::cuboid(Vec3::new(10.0, 0.05, 10.0)),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.05, 0.0), Quat::IDENTITY),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shoulder_velocity_moves_end_effector() {
        let mut sim = MobileManipulatorSim::new_mm_minimal();
        let initial = sim.observe();
        for _ in 0..360 {
            sim.step(MobileManipulatorAction {
                left_wheel_velocity_rad_s: 0.0,
                right_wheel_velocity_rad_s: 0.0,
                shoulder_velocity_rad_s: 3.0,
                elbow_velocity_rad_s: 0.0,
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
            displacement > 0.03 || shoulder_delta > 0.04,
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
        });
        let obs = sim.observe();
        assert_eq!(obs.joint_state_count, 2);
        let frame = sim
            .data_bus()
            .latest::<JointState>(StreamId::new(JOINT_STATE_STREAM as u64))
            .expect("joint state frame");
        assert_eq!(frame.payload.positions_rad.len(), 2);
        assert_eq!(frame.payload.names, vec!["shoulder_joint", "elbow_joint"]);
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
            });
        }
        let final_obs = sim.observe();
        let delta_x = final_obs.base_x_m - initial.base_x_m;
        assert!(
            delta_x.abs() > 0.15,
            "expected base translation, delta_x={delta_x}"
        );
        assert_eq!(sim.joint_names().len(), 4);
    }
}
