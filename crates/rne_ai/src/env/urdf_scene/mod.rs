//! Headless simulation for scenes that spawn URDF articulation robots.

mod humanoid_episode;
mod lekiwi_drive;
mod quadruped;
mod quadruped_episode;
mod unitree_g1_episode;
mod unitree_g1_gait;
mod unitree_g1_gait_episode;

pub use humanoid_episode::{
    HumanoidAction, HumanoidEpisode, HumanoidEpisodeConfig, HumanoidObservation,
};
pub use lekiwi_drive::{
    lekiwi_twist_to_wheel_velocities, lekiwi_wheel_command_to_motor_rad_s, UrdfKiwiAction,
    LEKIWI_DRIVE_WHEEL_LINKS, LEKIWI_WHEEL_AZIMUTH_RAD, LEKIWI_WHEEL_JOINT_SIGN,
    LEKIWI_WHEEL_PIVOT_RADIUS_M, LEKIWI_WHEEL_RADIUS_M,
};
pub use quadruped::{quadruped_trot_targets, QUADRUPED_FOOT_LINKS};
pub use quadruped_episode::{
    QuadrupedAction, QuadrupedEpisode, QuadrupedEpisodeConfig, QuadrupedObservation,
};
pub use unitree_g1_episode::{
    UnitreeG1Action, UnitreeG1Episode, UnitreeG1EpisodeConfig, UnitreeG1Observation,
};
pub use unitree_g1_gait::{unitree_g1_gait_targets, UnitreeG1GaitCommand};
pub use unitree_g1_gait_episode::{
    UnitreeG1GaitAction, UnitreeG1GaitEpisode, UnitreeG1GaitEpisodeConfig, UnitreeG1GaitObservation,
};

use rne_assets::{load_and_spawn_scene, load_scene_bundle, mesh_package_roots, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{Entity, World};
use rne_math::{yaw_rad, Hertz};
use rne_physics::{JointMotor, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::Link;
use rne_world::world_transform_of;
use std::path::{Path, PathBuf};

/// Observation for a generic URDF scene simulation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfSceneObservation {
    /// Base link world X position in meters.
    pub base_x_m: f64,
    /// Base link world Y position in meters.
    pub base_y_m: f64,
    /// Base link world Z position in meters.
    pub base_z_m: f64,
    /// Base yaw in radians (Y-up world).
    pub base_yaw_rad: f64,
    /// Number of revolute / continuous joints with motors in the scene.
    pub actuated_joint_count: usize,
}

/// Action for driving a URDF diff-drive cart (continuous wheel joints).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UrdfCartAction {
    /// Left wheel angular velocity in rad/s.
    pub left_velocity_rad_s: f64,
    /// Right wheel angular velocity in rad/s.
    pub right_velocity_rad_s: f64,
}

/// Action for teleoperating the first arm joint of a fixed-base URDF arm.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UrdfArmAction {
    /// Shoulder pan motor velocity in rad/s.
    pub shoulder_pan_velocity_rad_s: f64,
}

/// Position target for a named actuated URDF child link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfJointPositionTarget<'a> {
    /// Child link name whose articulation joint owns the motor.
    pub link_name: &'a str,
    /// Desired revolute angle in radians or prismatic displacement in meters.
    pub position: f64,
}

/// Headless URDF scene simulation (cart drive or fixed-base arm viewing).
pub struct UrdfSceneSim {
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    scene_path: PathBuf,
    mesh_package_roots: Vec<PathBuf>,
    world_seed: u64,
    base_link: Entity,
    left_wheel: Option<Entity>,
    right_wheel: Option<Entity>,
    kiwi_wheels: [Option<Entity>; 3],
    shoulder_pan_link: Option<Entity>,
    actuated_joint_count: usize,
    sim_time: SimTime,
    dt: SimDuration,
}

impl UrdfSceneSim {
    /// Loads a `.rne.scene.toml` whose primary robot is a URDF articulation.
    pub fn from_scene_path(scene_path: &Path) -> Result<Self, AssetError> {
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, scene_path)?;
        let world_seed = world
            .get::<rne_world::WorldEntity>(spawned.world)
            .map(|world_entity| world_entity.seed)
            .unwrap_or(0);

        let (_, first_robot) = spawned.robots.first().ok_or_else(|| AssetError::Invalid {
            path: scene_path.display().to_string(),
            message: "no robots".into(),
        })?;

        let base_link = first_robot.base_link;
        let left_wheel = find_link_by_name(&world, "left_wheel");
        let right_wheel = find_link_by_name(&world, "right_wheel");
        let kiwi_wheels = [
            find_link_by_name(&world, LEKIWI_DRIVE_WHEEL_LINKS[0]),
            find_link_by_name(&world, LEKIWI_DRIVE_WHEEL_LINKS[1]),
            find_link_by_name(&world, LEKIWI_DRIVE_WHEEL_LINKS[2]),
        ];
        let shoulder_pan_link = find_link_by_name(&world, "shoulder_link");
        let actuated_joint_count = world
            .iter_entities()
            .filter(|entity_ref| world.get::<JointMotor>(entity_ref.id()).is_some())
            .count();

        let bundle = load_scene_bundle(scene_path)?;
        let mesh_roots = mesh_package_roots(&bundle);

        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .map_err(|error| asset_physics_error(scene_path, error))?;

        let mut sim = Self {
            world,
            backend,
            physics_world,
            scene_path: scene_path.to_path_buf(),
            mesh_package_roots: mesh_roots,
            world_seed,
            base_link,
            left_wheel,
            right_wheel,
            kiwi_wheels,
            shoulder_pan_link,
            actuated_joint_count,
            sim_time: SimTime::default(),
            dt: SimDuration::from_hertz(Hertz::new(60.0)),
        };
        sim.backend
            .sync_from_ecs(&mut sim.world, sim.physics_world)
            .map_err(|error| asset_physics_error(scene_path, error))?;
        Ok(sim)
    }

    /// Built-in SO-101 scene path.
    pub fn so101_scene_path() -> PathBuf {
        so101_scene_path()
    }

    /// Built-in cart scene path.
    pub fn cart_minimal_scene_path() -> PathBuf {
        cart_minimal_scene_path()
    }

    /// Built-in LeKiwi base scene path.
    pub fn lekiwi_scene_path() -> PathBuf {
        lekiwi_scene_path()
    }

    /// Built-in LeKiwi + SO-101 composite scene path.
    pub fn lekiwi_so101_scene_path() -> PathBuf {
        lekiwi_so101_scene_path()
    }

    /// Built-in 12-DoF RNE quadruped scene path.
    pub fn quadruped_scene_path() -> PathBuf {
        quadruped_scene_path()
    }

    /// Built-in 12-DoF RNE humanoid scene path.
    pub fn humanoid_scene_path() -> PathBuf {
        humanoid_scene_path()
    }

    /// Vendored official Unitree Go2 scene path.
    pub fn unitree_go2_scene_path() -> PathBuf {
        unitree_go2_scene_path()
    }

    /// Vendored official Unitree Go2 dynamic multibody scene path.
    pub fn unitree_go2_dynamic_scene_path() -> PathBuf {
        unitree_go2_dynamic_scene_path()
    }

    /// Vendored official Unitree G1 23-DoF scene path.
    pub fn unitree_g1_scene_path() -> PathBuf {
        unitree_g1_scene_path()
    }

    /// Vendored official Unitree G1 dynamic standing scene path.
    pub fn unitree_g1_dynamic_scene_path() -> PathBuf {
        unitree_g1_dynamic_scene_path()
    }

    /// Returns whether this scene has diff-drive wheel motors.
    pub fn left_wheel(&self) -> Option<Entity> {
        self.left_wheel
    }

    /// Returns whether this scene uses LeKiwi kiwi-drive wheel motors.
    pub fn is_kiwi_drive(&self) -> bool {
        self.kiwi_wheels[0].is_some()
    }

    /// Returns whether a SO-101 style shoulder pan joint is present.
    pub fn has_arm(&self) -> bool {
        self.shoulder_pan_link.is_some()
    }
    /// Returns the loaded scene path.
    pub fn scene_path(&self) -> &Path {
        &self.scene_path
    }

    /// Returns mesh package roots for rendering.
    pub fn mesh_package_roots(&self) -> &[PathBuf] {
        &self.mesh_package_roots
    }

    /// Returns the world seed from the scene.
    pub fn world_seed(&self) -> u64 {
        self.world_seed
    }

    /// Returns the ECS world.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Returns a named URDF link's world translation in meters.
    pub fn link_translation_m(&self, link_name: &str) -> Option<(f64, f64, f64)> {
        let entity = find_link_by_name(&self.world, link_name)?;
        let translation = world_transform_of(&self.world, entity).translation;
        Some((translation.x, translation.y, translation.z))
    }

    /// Applies a diff-drive wheel action and steps one simulation tick.
    pub fn step_cart(&mut self, action: UrdfCartAction) {
        if let Some(left) = self.left_wheel {
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(left) {
                motor.velocity_rad_s = action.left_velocity_rad_s;
            }
        }
        if let Some(right) = self.right_wheel {
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(right) {
                motor.velocity_rad_s = action.right_velocity_rad_s;
            }
        }
        self.step_physics();
    }

    /// Applies a kiwi-drive planar action and steps one simulation tick.
    pub fn step_kiwi(&mut self, action: UrdfKiwiAction) {
        use lekiwi_drive::{lekiwi_twist_to_wheel_velocities, lekiwi_wheel_command_to_motor_rad_s};
        let wheel_velocities_rad_s = lekiwi_twist_to_wheel_velocities(action);
        for (wheel, velocity_rad_s) in self.kiwi_wheels.iter().zip(wheel_velocities_rad_s) {
            if let Some(entity) = wheel {
                if let Some(mut motor) = self.world.get_mut::<JointMotor>(*entity) {
                    motor.velocity_rad_s = lekiwi_wheel_command_to_motor_rad_s(velocity_rad_s);
                }
            }
        }
        self.step_physics();
    }

    /// Applies kiwi-drive and arm teleop actions, then steps one simulation tick.
    ///
    /// Used by the LeKiwi + SO-101 composite where wheel and arm motors share one articulation.
    pub fn step_kiwi_and_arm(&mut self, kiwi: UrdfKiwiAction, arm: UrdfArmAction) {
        use lekiwi_drive::{lekiwi_twist_to_wheel_velocities, lekiwi_wheel_command_to_motor_rad_s};
        let wheel_velocities_rad_s = lekiwi_twist_to_wheel_velocities(kiwi);
        for (wheel, velocity_rad_s) in self.kiwi_wheels.iter().zip(wheel_velocities_rad_s) {
            if let Some(entity) = wheel {
                if let Some(mut motor) = self.world.get_mut::<JointMotor>(*entity) {
                    motor.velocity_rad_s = lekiwi_wheel_command_to_motor_rad_s(velocity_rad_s);
                }
            }
        }
        if let Some(shoulder) = self.shoulder_pan_link {
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(shoulder) {
                motor.velocity_rad_s = arm.shoulder_pan_velocity_rad_s;
            }
        }
        self.step_physics();
    }

    /// Applies an arm teleop action and steps one simulation tick.
    pub fn step_arm(&mut self, action: UrdfArmAction) {
        if let Some(shoulder) = self.shoulder_pan_link {
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(shoulder) {
                motor.velocity_rad_s = action.shoulder_pan_velocity_rad_s;
            }
        }
        self.step_physics();
    }

    /// Applies named joint position targets and steps one simulation tick.
    ///
    /// Unknown link names and links without a [`JointMotor`] are ignored, which
    /// lets a controller send a stable superset of targets across related URDF
    /// variants. Motors retain their existing force, stiffness, and damping.
    pub fn step_joint_position_targets(&mut self, targets: &[UrdfJointPositionTarget<'_>]) {
        for target in targets {
            let Some(entity) = find_link_by_name(&self.world, target.link_name) else {
                continue;
            };
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(entity) {
                motor.target_position = target.position;
                motor.velocity_rad_s = 0.0;
            }
        }
        self.step_physics();
    }

    /// Configures every actuated joint as a force-limited position motor.
    ///
    /// `stiffness` and `damping` use the backend-neutral [`JointMotor`] gains;
    /// `max_force` is expressed in N for prismatic joints and N·m for revolute
    /// joints. This is a one-time setup helper for legged standing controllers.
    pub fn configure_position_motors(&mut self, stiffness: f64, damping: f64, max_force: f64) {
        let motor_entities: Vec<_> = self
            .world
            .iter_entities()
            .filter_map(|entity_ref| {
                self.world
                    .get::<JointMotor>(entity_ref.id())
                    .is_some()
                    .then_some(entity_ref.id())
            })
            .collect();
        for entity in motor_entities {
            if let Some(mut motor) = self.world.get_mut::<JointMotor>(entity) {
                motor.stiffness = stiffness;
                motor.gain = damping;
                motor.max_force = max_force;
            }
        }
    }

    /// Returns the summed normal contact impulse for a named link in N·s.
    ///
    /// This is intended for deterministic foot-contact observations. It returns
    /// zero when the link is absent, has no contact, or contacts have not yet
    /// been produced by a physics step.
    pub fn link_contact_impulse_ns(&self, link_name: &str) -> f64 {
        let Some(link) = find_link_by_name(&self.world, link_name) else {
            return 0.0;
        };
        self.backend
            .contacts(self.physics_world)
            .map(|contacts| {
                contacts
                    .iter()
                    .filter(|contact| contact.entity_a == link || contact.entity_b == link)
                    .map(|contact| f64::from(contact.impulse))
                    .sum()
            })
            .unwrap_or(0.0)
    }

    /// Returns the latest observation.
    pub fn observe(&self) -> UrdfSceneObservation {
        let base = world_transform_of(&self.world, self.base_link);
        UrdfSceneObservation {
            base_x_m: base.translation.x,
            base_y_m: base.translation.y,
            base_z_m: base.translation.z,
            base_yaw_rad: yaw_rad(base.rotation),
            actuated_joint_count: self.actuated_joint_count,
        }
    }

    fn step_physics(&mut self) {
        step_physics(
            &mut self.backend,
            &mut self.world,
            self.physics_world,
            self.dt,
        )
        .expect("urdf scene physics step");
        self.sim_time = self.sim_time + self.dt;
    }
}

fn asset_physics_error(path: &Path, error: rne_physics::PhysicsError) -> AssetError {
    AssetError::Invalid {
        path: path.display().to_string(),
        message: error.to_string(),
    }
}

fn find_link_by_name(world: &World, name: &str) -> Option<Entity> {
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world
            .get::<Link>(entity)
            .is_some_and(|link| link.name == name)
        {
            return Some(entity);
        }
    }
    None
}

fn assets_scene_path(file_name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes")
        .join(file_name)
}

/// Built-in SO-101 scene path.
pub fn so101_scene_path() -> PathBuf {
    assets_scene_path("so101.rne.scene.toml")
}

/// Built-in cart scene path.
pub fn cart_minimal_scene_path() -> PathBuf {
    assets_scene_path("cart_minimal.rne.scene.toml")
}

/// Built-in LeKiwi base scene path.
pub fn lekiwi_scene_path() -> PathBuf {
    assets_scene_path("lekiwi.rne.scene.toml")
}

/// Built-in LeKiwi + SO-101 composite scene path.
pub fn lekiwi_so101_scene_path() -> PathBuf {
    assets_scene_path("lekiwi_so101.rne.scene.toml")
}

/// Built-in 12-DoF RNE quadruped scene path.
pub fn quadruped_scene_path() -> PathBuf {
    assets_scene_path("rne_quadruped.rne.scene.toml")
}

/// Built-in 12-DoF RNE humanoid scene path.
pub fn humanoid_scene_path() -> PathBuf {
    assets_scene_path("rne_humanoid.rne.scene.toml")
}

/// Vendored official Unitree Go2 scene path.
pub fn unitree_go2_scene_path() -> PathBuf {
    assets_scene_path("unitree_go2.rne.scene.toml")
}

/// Vendored official Unitree Go2 dynamic multibody scene path.
pub fn unitree_go2_dynamic_scene_path() -> PathBuf {
    assets_scene_path("unitree_go2_dynamic.rne.scene.toml")
}

/// Vendored official Unitree G1 23-DoF scene path.
pub fn unitree_g1_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1.rne.scene.toml")
}

/// Vendored official Unitree G1 scene using primitive-only collision geometry.
pub fn unitree_g1_dynamic_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1_dynamic.rne.scene.toml")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn so101_urdf_parses_and_spawns_from_scene() {
        let scene_path = UrdfSceneSim::so101_scene_path();
        assert!(scene_path.is_file(), "missing {}", scene_path.display());
        let sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn so101");
        assert!(sim.actuated_joint_count >= 5);
        assert!(!sim.mesh_package_roots().is_empty());
        let obs = sim.observe();
        assert!(obs.base_y_m >= 0.0);
    }

    #[test]
    fn named_joint_position_target_updates_motor() {
        let scene_path = UrdfSceneSim::so101_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn so101");
        sim.step_joint_position_targets(&[UrdfJointPositionTarget {
            link_name: "shoulder_link",
            position: 0.35,
        }]);
        let shoulder = find_link_by_name(sim.world(), "shoulder_link").expect("shoulder link");
        let motor = sim
            .world()
            .get::<JointMotor>(shoulder)
            .expect("shoulder motor");
        assert_eq!(motor.target_position, 0.35);
        assert_eq!(motor.velocity_rad_s, 0.0);
    }

    #[test]
    fn named_link_contact_impulse_reports_wheel_ground_load() {
        let scene_path = cart_minimal_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn cart");
        for _ in 0..30 {
            sim.step_joint_position_targets(&[]);
        }
        let left_impulse_ns = sim.link_contact_impulse_ns("left_wheel");
        assert!(
            left_impulse_ns > 0.0,
            "settled wheel should report ground-contact impulse, got {left_impulse_ns} N·s"
        );
        assert_eq!(sim.link_contact_impulse_ns("missing_foot"), 0.0);
    }

    #[test]
    fn quadruped_spawns_with_twelve_motors_and_four_loaded_feet() {
        let scene_path = quadruped_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn quadruped");
        assert_eq!(sim.observe().actuated_joint_count, 12);
        sim.configure_position_motors(1200.0, 70.0, 40.0);
        for _ in 0..90 {
            sim.step_joint_position_targets(&[]);
        }
        for foot in ["fl_foot", "fr_foot", "rl_foot", "rr_foot"] {
            let impulse_ns = sim.link_contact_impulse_ns(foot);
            assert!(
                impulse_ns > 0.0,
                "standing quadruped foot `{foot}` should bear load, got {impulse_ns} N·s"
            );
        }
        let base_y_m = sim.observe().base_y_m;
        assert!(
            base_y_m > 0.35,
            "position-held quadruped should keep its body above ground, y={base_y_m} m"
        );
    }

    #[test]
    fn quadruped_trot_is_stable_and_deterministic() {
        fn rollout() -> UrdfSceneObservation {
            let mut sim =
                UrdfSceneSim::from_scene_path(&quadruped_scene_path()).expect("spawn quadruped");
            sim.configure_position_motors(1200.0, 70.0, 40.0);
            for _ in 0..180 {
                sim.step_joint_position_targets(&[]);
            }
            for step in 0..360 {
                sim.step_joint_position_targets(&quadruped_trot_targets(step));
            }
            sim.observe()
        }

        let first = rollout();
        let second = rollout();
        assert_eq!(first, second, "identical trot rollouts must replay exactly");
        assert!(
            first.base_x_m.abs() > 0.005,
            "trot should produce measurable planar motion, x={} m",
            first.base_x_m
        );
        assert!(
            first.base_y_m > 0.35,
            "trot should keep the body standing, y={} m",
            first.base_y_m
        );
    }

    #[test]
    fn humanoid_spawns_with_twelve_motors_and_two_loaded_feet() {
        let scene_path = humanoid_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn humanoid");
        assert_eq!(sim.observe().actuated_joint_count, 12);
        sim.configure_position_motors(1800.0, 85.0, 80.0);
        for _ in 0..180 {
            sim.step_joint_position_targets(&[]);
        }
        for foot in ["left_foot", "right_foot"] {
            let impulse_ns = sim.link_contact_impulse_ns(foot);
            assert!(
                impulse_ns > 0.0,
                "standing humanoid foot `{foot}` should bear load, got {impulse_ns} N·s"
            );
        }
        let base_y_m = sim.observe().base_y_m;
        assert!(
            base_y_m > 0.70,
            "position-held humanoid should remain upright, y={base_y_m} m"
        );
    }

    #[test]
    fn official_unitree_go2_urdf_spawns_with_twelve_motors() {
        let scene_path = unitree_go2_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn Unitree Go2");
        assert_eq!(sim.observe().actuated_joint_count, 12);
        sim.configure_position_motors(80.0, 12.0, 23.7);
        for _ in 0..30 {
            sim.step_joint_position_targets(&[]);
        }
        assert!((sim.observe().base_y_m - 0.36).abs() < 1.0e-9);
        assert!(!sim.mesh_package_roots().is_empty());
    }

    #[test]
    fn official_unitree_go2_dynamic_multibody_stands_on_four_feet() {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("spawn dynamic Unitree Go2");
        sim.configure_position_motors(180.0, 18.0, 23.7);
        let legs = ["FL", "FR", "RL", "RR"];
        for _ in 0..180 {
            let mut targets = Vec::with_capacity(12);
            for leg in legs {
                targets.push(UrdfJointPositionTarget {
                    link_name: match leg {
                        "FL" => "FL_hip",
                        "FR" => "FR_hip",
                        "RL" => "RL_hip",
                        _ => "RR_hip",
                    },
                    position: 0.0,
                });
                targets.push(UrdfJointPositionTarget {
                    link_name: match leg {
                        "FL" => "FL_thigh",
                        "FR" => "FR_thigh",
                        "RL" => "RL_thigh",
                        _ => "RR_thigh",
                    },
                    position: 0.8,
                });
                targets.push(UrdfJointPositionTarget {
                    link_name: match leg {
                        "FL" => "FL_calf",
                        "FR" => "FR_calf",
                        "RL" => "RL_calf",
                        _ => "RR_calf",
                    },
                    position: -1.5,
                });
            }
            sim.step_joint_position_targets(&targets);
        }
        let observation = sim.observe();
        assert!(observation.base_y_m > 0.18, "Go2 fell: {observation:?}");
        let load: f64 = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
            .iter()
            .map(|foot| sim.link_contact_impulse_ns(foot))
            .sum();
        assert!(load > 0.0, "Go2 feet should contact the ground");
    }

    #[test]
    fn official_unitree_g1_urdf_spawns_with_twenty_three_motors() {
        let scene_path = unitree_g1_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn Unitree G1");
        assert_eq!(sim.observe().actuated_joint_count, 23);
        sim.configure_position_motors(80.0, 12.0, 60.0);
        for _ in 0..30 {
            sim.step_joint_position_targets(&[]);
        }
        assert!((sim.observe().base_y_m - 0.80).abs() < 1.0e-9);
        assert!(!sim.mesh_package_roots().is_empty());
    }

    #[test]
    fn official_unitree_g1_dynamic_scene_contacts_ground_without_exploding() {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("spawn dynamic Unitree G1");
        sim.configure_position_motors(220.0, 24.0, 88.0);
        let targets = [
            UrdfJointPositionTarget {
                link_name: "left_hip_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "left_knee_link",
                position: 0.36,
            },
            UrdfJointPositionTarget {
                link_name: "left_ankle_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "right_hip_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "right_knee_link",
                position: 0.36,
            },
            UrdfJointPositionTarget {
                link_name: "right_ankle_pitch_link",
                position: -0.18,
            },
        ];
        for _ in 0..240 {
            sim.step_joint_position_targets(&targets);
        }
        let observation = sim.observe();
        assert!(observation.base_y_m.is_finite());
        assert!(observation.base_y_m > 0.35, "G1 fell: {observation:?}");
        let foot_impulse_ns = sim.link_contact_impulse_ns("left_ankle_roll_link")
            + sim.link_contact_impulse_ns("right_ankle_roll_link");
        assert!(foot_impulse_ns > 0.0, "G1 feet should contact the ground");
    }

    #[test]
    fn official_unitree_g1_scripted_gait_advances_without_falling() {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("spawn walking Unitree G1");
        let pelvis = sim
            .world()
            .iter_entities()
            .find(|entity| {
                sim.world()
                    .get::<Link>(entity.id())
                    .is_some_and(|link| link.name == "pelvis")
            })
            .expect("G1 pelvis")
            .id();
        assert_eq!(
            sim.world()
                .get::<rne_physics::RigidBody>(pelvis)
                .expect("pelvis body")
                .body_type,
            rne_physics::RigidBodyType::Dynamic
        );
        sim.configure_position_motors(220.0, 24.0, 88.0);
        let stand = [
            UrdfJointPositionTarget {
                link_name: "left_hip_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "left_knee_link",
                position: 0.36,
            },
            UrdfJointPositionTarget {
                link_name: "left_ankle_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "right_hip_pitch_link",
                position: -0.18,
            },
            UrdfJointPositionTarget {
                link_name: "right_knee_link",
                position: 0.36,
            },
            UrdfJointPositionTarget {
                link_name: "right_ankle_pitch_link",
                position: -0.18,
            },
        ];
        for _ in 0..120 {
            sim.step_joint_position_targets(&stand);
        }
        let command = UnitreeG1GaitCommand::default();
        for step in 0..120 {
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, command));
        }
        let observation = sim.observe();
        assert!(observation.base_x_m.is_finite());
        assert!(
            observation.base_x_m > 0.005,
            "G1 did not advance: {observation:?}"
        );
        assert!(observation.base_y_m > 0.35, "G1 gait fell: {observation:?}");
        assert!(
            observation.base_yaw_rad.abs() < 1.2,
            "G1 yaw drifted: {observation:?}"
        );
    }

    #[test]
    fn cart_minimal_drives_forward_under_wheel_velocity() {
        let scene_path = UrdfSceneSim::cart_minimal_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn cart");
        let initial_x = sim.observe().base_x_m;
        // Linux CI physics stepping is slightly slower to accumulate wheel travel.
        for _ in 0..240 {
            sim.step_cart(UrdfCartAction {
                left_velocity_rad_s: 4.0,
                right_velocity_rad_s: 4.0,
            });
        }
        let moved = (sim.observe().base_x_m - initial_x).abs();
        assert!(
            moved > 0.05,
            "cart should advance under wheel motors, |moved|={moved}"
        );
    }

    #[test]
    fn lekiwi_spawns_with_three_drive_wheels() {
        let scene_path = lekiwi_scene_path();
        assert!(scene_path.is_file(), "missing {}", scene_path.display());
        let sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn lekiwi");
        assert!(sim.is_kiwi_drive());
        assert_eq!(sim.actuated_joint_count, 3);
    }

    #[test]
    fn lekiwi_drives_forward_under_positive_vx() {
        let scene_path = lekiwi_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn lekiwi");
        let initial = sim.observe();
        for _ in 0..180 {
            sim.step_kiwi(UrdfKiwiAction {
                vx_m_s: 0.15,
                vz_m_s: 0.0,
                wz_rad_s: 0.0,
            });
        }
        let obs = sim.observe();
        let dx_m = obs.base_x_m - initial.base_x_m;
        let dz_m = obs.base_z_m - initial.base_z_m;
        let planar_m = (dx_m * dx_m + dz_m * dz_m).sqrt();
        assert!(
            planar_m > 0.03,
            "lekiwi should translate under +vx, planar={planar_m:.4} m (dx={dx_m:.4}, dz={dz_m:.4})"
        );
    }

    #[test]
    fn lekiwi_yaw_under_positive_wz() {
        let scene_path = lekiwi_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn lekiwi");
        let initial_yaw = sim.observe().base_yaw_rad;
        for _ in 0..240 {
            sim.step_kiwi(UrdfKiwiAction {
                vx_m_s: 0.0,
                vz_m_s: 0.0,
                wz_rad_s: 0.4,
            });
        }
        let yaw_delta = (sim.observe().base_yaw_rad - initial_yaw).abs();
        assert!(
            yaw_delta > 0.05,
            "lekiwi should yaw under +wz, |delta|={yaw_delta:.4} rad"
        );
    }

    #[test]
    fn lekiwi_so101_loads_with_kiwi_drive_and_arm() {
        let scene_path = lekiwi_so101_scene_path();
        assert!(scene_path.is_file(), "missing {}", scene_path.display());
        let sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn lekiwi_so101");
        assert!(sim.is_kiwi_drive());
        assert!(sim.has_arm());
        assert!(
            sim.actuated_joint_count >= 8,
            "expected wheel + arm motors, got {}",
            sim.actuated_joint_count
        );
        assert!(!sim.mesh_package_roots().is_empty());
    }

    #[test]
    fn lekiwi_so101_drives_and_arm_joints_exist() {
        let scene_path = lekiwi_so101_scene_path();
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn lekiwi_so101");
        let initial = sim.observe();
        for _ in 0..180 {
            sim.step_kiwi_and_arm(
                UrdfKiwiAction {
                    vx_m_s: 0.15,
                    vz_m_s: 0.0,
                    wz_rad_s: 0.0,
                },
                UrdfArmAction {
                    shoulder_pan_velocity_rad_s: 0.5,
                },
            );
        }
        let obs = sim.observe();
        let dx_m = obs.base_x_m - initial.base_x_m;
        let dz_m = obs.base_z_m - initial.base_z_m;
        let planar_m = (dx_m * dx_m + dz_m * dz_m).sqrt();
        assert!(
            planar_m > 0.03,
            "lekiwi_so101 should translate under +vx, planar={planar_m:.4} m"
        );
        assert!(
            planar_m < 2.0,
            "lekiwi_so101 planar displacement should stay bounded, planar={planar_m:.4} m"
        );
        assert!(
            obs.actuated_joint_count >= 8,
            "arm joints should remain after stepping, got {}",
            obs.actuated_joint_count
        );
    }
}
