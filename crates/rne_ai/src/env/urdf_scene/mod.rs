//! Headless simulation for scenes that spawn URDF articulation robots.

mod humanoid_episode;
mod lekiwi_drive;
mod quadruped;
mod quadruped_episode;
mod unitree_g1_dex3;
mod unitree_g1_dex3_episode;
mod unitree_g1_episode;
mod unitree_g1_gait;
mod unitree_g1_gait_episode;
mod unitree_g1_inspection;
mod unitree_g1_inspection_episode;
mod unitree_g1_parts_episode;
mod unitree_go2_episode;
mod unitree_go2_gait;

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
pub use unitree_g1_dex3::{unitree_g1_dex3_pick_targets, UnitreeG1Dex3HandCommand};
pub use unitree_g1_dex3_episode::{
    UnitreeG1Dex3Action, UnitreeG1Dex3Episode, UnitreeG1Dex3EpisodeConfig,
    UnitreeG1Dex3Observation, UnitreeG1Dex3Phase,
};
pub use unitree_g1_episode::{
    UnitreeG1Action, UnitreeG1Episode, UnitreeG1EpisodeConfig, UnitreeG1Observation,
};
pub use unitree_g1_gait::{unitree_g1_gait_targets, UnitreeG1GaitCommand};
pub use unitree_g1_gait_episode::{
    UnitreeG1GaitAction, UnitreeG1GaitEpisode, UnitreeG1GaitEpisodeConfig, UnitreeG1GaitObservation,
};
pub use unitree_g1_inspection::unitree_g1_inspection_targets;
pub use unitree_g1_inspection_episode::{
    UnitreeG1InspectionAction, UnitreeG1InspectionEpisode, UnitreeG1InspectionEpisodeConfig,
    UnitreeG1InspectionObservation,
};
pub use unitree_g1_parts_episode::{
    UnitreeG1PartsAction, UnitreeG1PartsEpisode, UnitreeG1PartsEpisodeConfig,
    UnitreeG1PartsObservation, UnitreeG1PartsPhase,
};
pub use unitree_go2_episode::{
    UnitreeGo2Action, UnitreeGo2Episode, UnitreeGo2EpisodeConfig, UnitreeGo2Observation,
};
pub use unitree_go2_gait::{unitree_go2_trot_targets, UnitreeGo2GaitCommand};

use rne_assets::{load_and_spawn_scene, load_scene_bundle, mesh_package_roots, AssetError};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{Entity, Name, Parent, World};
use rne_math::{y_up_euler_rad, Hertz, Quat};
use rne_physics::{
    Collider, ColliderShape, CollisionGroups, FixedJointDesc, JointMotor, PhysicsBackend,
    PhysicsWorldDesc, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::Link;
use rne_world::{world_transform_of, TaskMarker, Transform3 as WorldTransform3};
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
    /// Base pitch about the world/local X axis in radians.
    pub base_pitch_rad: f64,
    /// Base roll about the world/local Z axis in radians.
    pub base_roll_rad: f64,
    /// Base linear velocity along X in meters per second.
    pub base_linear_velocity_x_m_s: f64,
    /// Base linear velocity along Y in meters per second.
    pub base_linear_velocity_y_m_s: f64,
    /// Base linear velocity along Z in meters per second.
    pub base_linear_velocity_z_m_s: f64,
    /// Base angular velocity about X in radians per second.
    pub base_angular_velocity_x_rad_s: f64,
    /// Base angular velocity about Y in radians per second.
    pub base_angular_velocity_y_rad_s: f64,
    /// Base angular velocity about Z in radians per second.
    pub base_angular_velocity_z_rad_s: f64,
    /// Base yaw relative to its scene-load orientation in radians.
    pub base_relative_yaw_rad: f64,
    /// Base pitch relative to its scene-load orientation in radians.
    pub base_relative_pitch_rad: f64,
    /// Base roll relative to its scene-load orientation in radians.
    pub base_relative_roll_rad: f64,
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
    base_reference_rotation: Quat,
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
        Self::from_scene_path_with_solver_iterations(scene_path, 0)
    }

    /// Loads a URDF scene with an explicit constraint-solver iteration count.
    ///
    /// `solver_iterations == 0` selects the backend default. Higher values are
    /// useful for small articulated fingers and tall chains under contact load.
    pub fn from_scene_path_with_solver_iterations(
        scene_path: &Path,
        solver_iterations: usize,
    ) -> Result<Self, AssetError> {
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
        let base_reference_rotation = world_transform_of(&world, base_link).rotation;
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
            .create_world(PhysicsWorldDesc {
                solver_iterations,
                ..PhysicsWorldDesc::default()
            })
            .map_err(|error| asset_physics_error(scene_path, error))?;

        let mut sim = Self {
            world,
            backend,
            physics_world,
            scene_path: scene_path.to_path_buf(),
            mesh_package_roots: mesh_roots,
            world_seed,
            base_link,
            base_reference_rotation,
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

    /// Built-in Unitree G1 factory inspection scene path.
    pub fn unitree_g1_factory_scene_path() -> PathBuf {
        unitree_g1_factory_scene_path()
    }

    /// Fixed-base Unitree G1 parts pick-and-place scene path.
    pub fn unitree_g1_parts_pick_place_scene_path() -> PathBuf {
        unitree_g1_parts_pick_place_scene_path()
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

    /// Returns a named task marker's world translation and interaction radius.
    pub fn task_marker(&self, name: &str) -> Option<(f64, f64, f64, f64)> {
        for entity_ref in self.world.iter_entities() {
            let entity = entity_ref.id();
            if self
                .world
                .get::<Name>(entity)
                .is_none_or(|entity_name| entity_name.0 != name)
            {
                continue;
            }
            let marker = self.world.get::<TaskMarker>(entity)?;
            let translation = world_transform_of(&self.world, entity).translation;
            return Some((translation.x, translation.y, translation.z, marker.radius_m));
        }
        None
    }

    /// Returns a named URDF link's world translation in meters.
    pub fn link_translation_m(&self, link_name: &str) -> Option<(f64, f64, f64)> {
        let entity = find_link_by_name(&self.world, link_name)?;
        let translation = world_transform_of(&self.world, entity).translation;
        Some((translation.x, translation.y, translation.z))
    }

    /// Returns any named scene entity's world translation in meters.
    pub fn named_translation_m(&self, name: &str) -> Option<(f64, f64, f64)> {
        let entity = find_entity_by_name(&self.world, name)?;
        let translation = world_transform_of(&self.world, entity).translation;
        Some((translation.x, translation.y, translation.z))
    }

    /// Returns any named scene entity's world transform.
    pub fn named_transform(&self, name: &str) -> Option<rne_world::Transform3> {
        let entity = find_entity_by_name(&self.world, name)?;
        Some(world_transform_of(&self.world, entity))
    }

    /// Repositions an unparented named rigid body and clears its velocity.
    ///
    /// This is intended for deterministic episode reset randomization before
    /// simulation resumes. Returns false for missing, parented, non-rigid, or
    /// non-finite targets.
    pub fn set_named_body_translation_m(&mut self, name: &str, translation_m: [f64; 3]) -> bool {
        if translation_m.iter().any(|value| !value.is_finite()) {
            return false;
        }
        let Some(entity) = find_entity_by_name(&self.world, name) else {
            return false;
        };
        if self.world.get::<Parent>(entity).is_some()
            || self.world.get::<RigidBody>(entity).is_none()
        {
            return false;
        }
        let mut transform = world_transform_of(&self.world, entity);
        transform.translation =
            rne_math::Vec3::new(translation_m[0], translation_m[1], translation_m[2]);
        self.world.entity_mut(entity).insert(transform);
        let mut body = self
            .world
            .get_mut::<RigidBody>(entity)
            .expect("rigid body checked above");
        body.linear_velocity_m_s = rne_math::Vec3::ZERO;
        body.angular_velocity_rad_s = rne_math::Vec3::ZERO;
        true
    }

    /// Switches a named rigid body between kinematic following and dynamic simulation.
    ///
    /// Velocities are cleared on every transition so releasing a pose-followed
    /// payload starts from rest. Returns false when the named entity is not a body.
    pub fn set_named_body_kinematic(&mut self, name: &str, kinematic: bool) -> bool {
        let Some(entity) = find_entity_by_name(&self.world, name) else {
            return false;
        };
        let Some(mut body) = self.world.get_mut::<RigidBody>(entity) else {
            return false;
        };
        body.body_type = if kinematic {
            RigidBodyType::Kinematic
        } else {
            RigidBodyType::Dynamic
        };
        body.linear_velocity_m_s = rne_math::Vec3::ZERO;
        body.angular_velocity_rad_s = rne_math::Vec3::ZERO;
        true
    }

    /// Returns a named rigid body's linear speed in meters per second.
    pub fn named_linear_speed_m_s(&self, name: &str) -> Option<f64> {
        let entity = find_entity_by_name(&self.world, name)?;
        let velocity = self.world.get::<RigidBody>(entity)?.linear_velocity_m_s;
        Some(velocity.x.hypot(velocity.y).hypot(velocity.z))
    }

    /// Returns whether two named entities contacted during the latest physics step.
    pub fn named_entities_in_contact(&self, first_name: &str, second_name: &str) -> bool {
        let Some(first) = find_entity_by_name(&self.world, first_name) else {
            return false;
        };
        let Some(second) = find_entity_by_name(&self.world, second_name) else {
            return false;
        };
        self.backend
            .contacts(self.physics_world)
            .is_ok_and(|contacts| {
                contacts.iter().any(|contact| {
                    (contact.entity_a == first && contact.entity_b == second)
                        || (contact.entity_a == second && contact.entity_b == first)
                })
            })
    }

    /// Welds a named child body to a named parent only when they are in contact.
    ///
    /// The current relative pose is captured, so a successful attachment does not
    /// teleport either body. Returns false when either entity is missing or the
    /// latest physics step did not report contact between them.
    pub fn weld_named_child_on_contact(&mut self, parent_name: &str, child_name: &str) -> bool {
        if !self.named_entities_in_contact(parent_name, child_name) {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let Some(child) = find_entity_by_name(&self.world, child_name) else {
            return false;
        };
        let parent_transform = world_transform_of(&self.world, parent);
        let child_transform = world_transform_of(&self.world, child);
        self.world.entity_mut(child).insert(FixedJointDesc {
            parent,
            anchor_parent_m: parent_transform.rotation.conjugate()
                * (child_transform.translation - parent_transform.translation),
            anchor_child_m: rne_math::Vec3::ZERO,
            relative_rotation: parent_transform.rotation.conjugate() * child_transform.rotation,
        });
        true
    }

    /// Welds a contacting child at a canonical pose in the parent's local frame.
    ///
    /// Unlike [`Self::weld_named_child_on_contact`], this places the child's origin
    /// at `anchor_parent_m` and aligns its rotation with the parent. The contact
    /// requirement prevents a distant body from being attached through scripting.
    pub fn weld_named_child_on_contact_at_parent_anchor(
        &mut self,
        parent_name: &str,
        child_name: &str,
        anchor_parent_m: [f64; 3],
    ) -> bool {
        if anchor_parent_m.iter().any(|value| !value.is_finite())
            || !self.named_entities_in_contact(parent_name, child_name)
        {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let Some(child) = find_entity_by_name(&self.world, child_name) else {
            return false;
        };
        self.world.entity_mut(child).insert(FixedJointDesc {
            parent,
            anchor_parent_m: rne_math::Vec3::new(
                anchor_parent_m[0],
                anchor_parent_m[1],
                anchor_parent_m[2],
            ),
            anchor_child_m: rne_math::Vec3::ZERO,
            relative_rotation: rne_math::Quat::IDENTITY,
        });
        true
    }

    /// Welds a child to a parent only after two distinct named contacts.
    ///
    /// Both `first_contact_name` and `second_contact_name` must contact the child
    /// during the latest physics step. This supports deterministic two-sided
    /// pinch gates while keeping the attachment parent at a palm or tool frame.
    pub fn weld_named_child_on_dual_contact_at_parent_anchor(
        &mut self,
        parent_name: &str,
        first_contact_name: &str,
        second_contact_name: &str,
        child_name: &str,
        anchor_parent_m: [f64; 3],
    ) -> bool {
        if anchor_parent_m.iter().any(|value| !value.is_finite())
            || !self.named_child_has_distinct_dual_contact(
                first_contact_name,
                second_contact_name,
                child_name,
            )
        {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let Some(child) = find_entity_by_name(&self.world, child_name) else {
            return false;
        };
        self.world.entity_mut(child).insert(FixedJointDesc {
            parent,
            anchor_parent_m: rne_math::Vec3::new(
                anchor_parent_m[0],
                anchor_parent_m[1],
                anchor_parent_m[2],
            ),
            anchor_child_m: rne_math::Vec3::ZERO,
            relative_rotation: rne_math::Quat::IDENTITY,
        });
        true
    }

    /// Welds a child to a parent at its current relative pose after two distinct contacts.
    ///
    /// Both named contact entities must touch the child during the latest physics
    /// step. Unlike the canonical-anchor variant, this captures the current
    /// translation and rotation, so confirming a grasp never snaps the payload.
    pub fn weld_named_child_on_dual_contact(
        &mut self,
        parent_name: &str,
        first_contact_name: &str,
        second_contact_name: &str,
        child_name: &str,
    ) -> bool {
        if !self.named_child_has_distinct_dual_contact(
            first_contact_name,
            second_contact_name,
            child_name,
        ) {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let Some(child) = find_entity_by_name(&self.world, child_name) else {
            return false;
        };
        let parent_transform = world_transform_of(&self.world, parent);
        let child_transform = world_transform_of(&self.world, child);
        self.world.entity_mut(child).insert(FixedJointDesc {
            parent,
            anchor_parent_m: parent_transform.rotation.conjugate()
                * (child_transform.translation - parent_transform.translation),
            anchor_child_m: rne_math::Vec3::ZERO,
            relative_rotation: parent_transform.rotation.conjugate() * child_transform.rotation,
        });
        true
    }

    /// Returns whether two distinct named entities both contact a named child.
    ///
    /// Contacts must come from the latest physics step. Passing the same entity
    /// name twice always returns false, which prevents one collider from
    /// masquerading as a two-sided pinch.
    pub fn named_child_has_distinct_dual_contact(
        &self,
        first_contact_name: &str,
        second_contact_name: &str,
        child_name: &str,
    ) -> bool {
        first_contact_name != second_contact_name
            && self.named_entities_in_contact(first_contact_name, child_name)
            && self.named_entities_in_contact(second_contact_name, child_name)
    }

    /// Releases a named child body previously attached by a fixed joint.
    pub fn release_named_child(&mut self, child_name: &str) -> bool {
        let Some(child) = find_entity_by_name(&self.world, child_name) else {
            return false;
        };
        let was_welded = self.world.get::<FixedJointDesc>(child).is_some();
        self.world.entity_mut(child).remove::<FixedJointDesc>();
        was_welded
    }

    /// Returns whether a named child currently carries a fixed-joint attachment.
    pub fn named_child_is_welded(&self, child_name: &str) -> bool {
        find_entity_by_name(&self.world, child_name)
            .is_some_and(|child| self.world.get::<FixedJointDesc>(child).is_some())
    }

    /// Adds a box-shaped physics contact proxy to a named entity without a collider.
    ///
    /// `size_m` contains full X/Y/Z extents in meters. This is useful when a
    /// visual-quality URDF mesh intentionally has mesh collisions disabled but
    /// a small end-effector proxy is needed for deterministic interaction.
    pub fn add_named_box_contact_proxy(&mut self, name: &str, size_m: [f64; 3]) -> bool {
        self.add_named_box_contact_proxy_at(name, size_m, [0.0; 3])
    }

    /// Adds an offset box-shaped physics contact proxy to a named entity.
    ///
    /// `size_m` contains full X/Y/Z extents and `offset_m` locates the proxy in
    /// the entity's local frame. This is useful for URDF end-effector meshes whose
    /// link origin is at the wrist rather than inside the visible palm.
    pub fn add_named_box_contact_proxy_at(
        &mut self,
        name: &str,
        size_m: [f64; 3],
        offset_m: [f64; 3],
    ) -> bool {
        if size_m
            .iter()
            .any(|extent_m| !extent_m.is_finite() || *extent_m <= 0.0)
            || offset_m.iter().any(|value| !value.is_finite())
        {
            return false;
        }
        let Some(entity) = find_entity_by_name(&self.world, name) else {
            return false;
        };
        if self.world.get::<Collider>(entity).is_some() {
            return false;
        }
        self.world.entity_mut(entity).insert(Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: rne_math::Vec3::new(size_m[0], size_m[1], size_m[2]) * 0.5,
            },
            local_offset: rne_world::Transform3::from_translation_rotation(
                rne_math::Vec3::new(offset_m[0], offset_m[1], offset_m[2]),
                rne_math::Quat::IDENTITY,
            ),
            ..Collider::default()
        });
        true
    }

    /// Sets backend-neutral collision membership and filter masks on a named entity.
    ///
    /// Returns false if the entity is missing. The masks take effect on the next
    /// physics synchronization and can be set before or after adding a collider.
    pub fn set_named_collision_groups(&mut self, name: &str, groups: CollisionGroups) -> bool {
        let Some(entity) = find_entity_by_name(&self.world, name) else {
            return false;
        };
        self.world.entity_mut(entity).insert(groups);
        true
    }

    /// Enables or disables non-reactive overlap sensing on a named collider.
    ///
    /// Sensor overlaps appear in [`Self::named_entities_in_contact`] but do not
    /// apply contact forces. Returns false if the entity or collider is missing.
    pub fn set_named_collider_sensor(&mut self, name: &str, sensor: bool) -> bool {
        let Some(entity) = find_entity_by_name(&self.world, name) else {
            return false;
        };
        let Some(mut collider) = self.world.get_mut::<Collider>(entity) else {
            return false;
        };
        collider.sensor = sensor;
        true
    }

    /// Adds an invisible box sensor that follows a named parent entity.
    ///
    /// The sensor is a separate fixed physics body, so it does not alter a
    /// multibody link's mass or inertia. `size_m` contains full X/Y/Z extents
    /// and `offset_m` is expressed in the parent's local frame.
    pub fn add_named_child_box_sensor(
        &mut self,
        parent_name: &str,
        sensor_name: &str,
        size_m: [f64; 3],
        offset_m: [f64; 3],
    ) -> bool {
        if sensor_name.is_empty()
            || find_entity_by_name(&self.world, sensor_name).is_some()
            || size_m
                .iter()
                .any(|extent_m| !extent_m.is_finite() || *extent_m <= 0.0)
            || offset_m.iter().any(|value| !value.is_finite())
        {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let sensor = rne_ecs::spawn_named(&mut self.world, sensor_name);
        self.world.entity_mut(sensor).insert((
            Parent(parent),
            WorldTransform3::from_translation_rotation(
                rne_math::Vec3::new(offset_m[0], offset_m[1], offset_m[2]),
                rne_math::Quat::IDENTITY,
            ),
            RigidBody {
                body_type: rne_physics::RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: rne_math::Vec3::new(size_m[0], size_m[1], size_m[2]) * 0.5,
                },
                sensor: true,
                ..Collider::default()
            },
        ));
        true
    }

    /// Adds an invisible coordinate frame that follows a named parent entity.
    ///
    /// The frame has no rigid body or collider, so it cannot change a multibody
    /// link's mass, inertia, or collision geometry. `offset_m` is expressed in
    /// the parent's local frame.
    pub fn add_named_child_frame(
        &mut self,
        parent_name: &str,
        frame_name: &str,
        offset_m: [f64; 3],
    ) -> bool {
        if frame_name.is_empty()
            || find_entity_by_name(&self.world, frame_name).is_some()
            || offset_m.iter().any(|value| !value.is_finite())
        {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let frame = rne_ecs::spawn_named(&mut self.world, frame_name);
        self.world.entity_mut(frame).insert((
            Parent(parent),
            WorldTransform3::from_translation_rotation(
                rne_math::Vec3::new(offset_m[0], offset_m[1], offset_m[2]),
                rne_math::Quat::IDENTITY,
            ),
        ));
        true
    }

    /// Adds a child frame whose world pose initially matches a named body.
    ///
    /// The body's current pose is converted into the parent's local frame. This
    /// supports pose-preserving deterministic grasp followers without a capture
    /// teleport. Returns false when a name is missing or the frame already exists.
    pub fn add_named_child_frame_from_body(
        &mut self,
        parent_name: &str,
        frame_name: &str,
        body_name: &str,
    ) -> bool {
        if frame_name.is_empty() || find_entity_by_name(&self.world, frame_name).is_some() {
            return false;
        }
        let Some(parent) = find_entity_by_name(&self.world, parent_name) else {
            return false;
        };
        let Some(body) = find_entity_by_name(&self.world, body_name) else {
            return false;
        };
        let parent_transform = world_transform_of(&self.world, parent);
        let body_transform = world_transform_of(&self.world, body);
        let inverse_parent_rotation = parent_transform.rotation.conjugate();
        let local_transform = WorldTransform3 {
            translation: inverse_parent_rotation
                * ((body_transform.translation - parent_transform.translation)
                    / parent_transform.scale),
            rotation: inverse_parent_rotation * body_transform.rotation,
            scale: body_transform.scale / parent_transform.scale,
        };
        let frame = rne_ecs::spawn_named(&mut self.world, frame_name);
        self.world
            .entity_mut(frame)
            .insert((Parent(parent), local_transform));
        true
    }

    /// Places an unparented named rigid body at a named frame and clears its velocity.
    ///
    /// This is intended for deterministic contact-gated grasp followers. The
    /// body remains dynamic, so callers can stop following it and let physics
    /// resume naturally on release. Returns false if either entity is missing,
    /// the body is parented, or it has no rigid-body component.
    pub fn follow_named_body_to_frame(&mut self, body_name: &str, frame_name: &str) -> bool {
        let Some(body_entity) = find_entity_by_name(&self.world, body_name) else {
            return false;
        };
        let Some(frame_entity) = find_entity_by_name(&self.world, frame_name) else {
            return false;
        };
        if self.world.get::<Parent>(body_entity).is_some()
            || self.world.get::<RigidBody>(body_entity).is_none()
        {
            return false;
        }
        let frame_transform = world_transform_of(&self.world, frame_entity);
        self.world.entity_mut(body_entity).insert(frame_transform);
        let mut body = self
            .world
            .get_mut::<RigidBody>(body_entity)
            .expect("rigid body checked above");
        body.linear_velocity_m_s = rne_math::Vec3::ZERO;
        body.angular_velocity_rad_s = rne_math::Vec3::ZERO;
        true
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

    /// Configures one named actuated link as a force-limited position motor.
    ///
    /// `max_force` is expressed in N for prismatic joints and N·m for revolute
    /// joints. Returns false for a missing link, a non-actuated link, or invalid
    /// negative/non-finite parameters.
    pub fn configure_named_position_motor(
        &mut self,
        link_name: &str,
        stiffness: f64,
        damping: f64,
        max_force: f64,
    ) -> bool {
        if [stiffness, damping, max_force]
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return false;
        }
        let Some(entity) = find_link_by_name(&self.world, link_name) else {
            return false;
        };
        let Some(mut motor) = self.world.get_mut::<JointMotor>(entity) else {
            return false;
        };
        motor.stiffness = stiffness;
        motor.gain = damping;
        motor.max_force = max_force;
        true
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
        let (base_yaw_rad, base_pitch_rad, base_roll_rad) = y_up_euler_rad(base.rotation);
        let relative_rotation = self.base_reference_rotation.conjugate() * base.rotation;
        let (base_relative_yaw_rad, base_relative_pitch_rad, base_relative_roll_rad) =
            y_up_euler_rad(relative_rotation);
        let body = self
            .world
            .get::<RigidBody>(self.base_link)
            .copied()
            .unwrap_or_default();
        UrdfSceneObservation {
            base_x_m: base.translation.x,
            base_y_m: base.translation.y,
            base_z_m: base.translation.z,
            base_yaw_rad,
            base_pitch_rad,
            base_roll_rad,
            base_linear_velocity_x_m_s: body.linear_velocity_m_s.x,
            base_linear_velocity_y_m_s: body.linear_velocity_m_s.y,
            base_linear_velocity_z_m_s: body.linear_velocity_m_s.z,
            base_angular_velocity_x_rad_s: body.angular_velocity_rad_s.x,
            base_angular_velocity_y_rad_s: body.angular_velocity_rad_s.y,
            base_angular_velocity_z_rad_s: body.angular_velocity_rad_s.z,
            base_relative_yaw_rad,
            base_relative_pitch_rad,
            base_relative_roll_rad,
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

fn find_entity_by_name(world: &World, name: &str) -> Option<Entity> {
    world.iter_entities().find_map(|entity_ref| {
        world
            .get::<Name>(entity_ref.id())
            .is_some_and(|entity_name| entity_name.0 == name)
            .then_some(entity_ref.id())
    })
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

/// Vendored official Unitree G1 factory inspection scene path.
pub fn unitree_g1_factory_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1_factory.rne.scene.toml")
}

/// Fixed-base official Unitree G1 parts pick-and-place scene path.
pub fn unitree_g1_parts_pick_place_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1_parts_pick_place.rne.scene.toml")
}

/// Fixed-base official Unitree G1 29-DoF scene with dual Dex3-1 hands.
pub fn unitree_g1_dex3_scene_path() -> PathBuf {
    assets_scene_path("unitree_g1_dex3_pick_place.rne.scene.toml")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn observation_exposes_finite_base_orientation_and_velocity() {
        let mut sim =
            UrdfSceneSim::from_scene_path(&cart_minimal_scene_path()).expect("spawn observed cart");
        let initial = sim.observe();
        assert_eq!(initial.base_relative_yaw_rad, 0.0);
        assert_eq!(initial.base_relative_pitch_rad, 0.0);
        assert_eq!(initial.base_relative_roll_rad, 0.0);
        for _ in 0..5 {
            sim.step_cart(UrdfCartAction {
                left_velocity_rad_s: 2.0,
                right_velocity_rad_s: 2.0,
            });
        }
        let observation = sim.observe();
        for value in [
            observation.base_yaw_rad,
            observation.base_pitch_rad,
            observation.base_roll_rad,
            observation.base_linear_velocity_x_m_s,
            observation.base_linear_velocity_y_m_s,
            observation.base_linear_velocity_z_m_s,
            observation.base_angular_velocity_x_rad_s,
            observation.base_angular_velocity_y_rad_s,
            observation.base_angular_velocity_z_rad_s,
            observation.base_relative_yaw_rad,
            observation.base_relative_pitch_rad,
            observation.base_relative_roll_rad,
        ] {
            assert!(value.is_finite());
        }
    }

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
    fn official_unitree_go2_dynamic_trot_remains_upright() {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("spawn trotting Unitree Go2");
        sim.configure_position_motors(180.0, 18.0, 23.7);
        let stand = unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 90,
            },
        );
        for _ in 0..120 {
            sim.step_joint_position_targets(&stand);
        }
        let initial = sim.observe();
        for step in 0..120 {
            sim.step_joint_position_targets(&unitree_go2_trot_targets(
                step,
                UnitreeGo2GaitCommand::default(),
            ));
        }
        let observation = sim.observe();
        assert!(
            observation.base_y_m > 0.18,
            "Go2 trot fell: {observation:?}"
        );
        assert!(observation.base_x_m.is_finite());
        assert!(
            (observation.base_x_m - initial.base_x_m).abs() > 0.01,
            "Go2 trot should translate through contact: {observation:?}"
        );
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
    fn official_unitree_g1_dex3_urdf_spawns_with_forty_three_motors() {
        let scene_path = unitree_g1_dex3_scene_path();
        let sim = UrdfSceneSim::from_scene_path(&scene_path).expect("spawn Unitree G1 with Dex3");
        assert_eq!(sim.observe().actuated_joint_count, 43);
        for link_name in [
            "left_hand_palm_link",
            "left_hand_thumb_2_link",
            "left_hand_index_1_link",
            "right_wrist_pitch_link",
            "right_wrist_yaw_link",
            "right_hand_palm_link",
            "right_hand_thumb_2_link",
            "right_hand_index_1_link",
        ] {
            assert!(
                sim.link_translation_m(link_name).is_some(),
                "missing Dex3 link `{link_name}`"
            );
        }
        assert!(!sim.mesh_package_roots().is_empty());
    }

    #[test]
    fn official_unitree_g1_dex3_fingers_articulate() {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dex3_scene_path())
            .expect("spawn Unitree G1 with Dex3");
        sim.configure_position_motors(220.0, 24.0, 88.0);
        for _ in 0..20 {
            sim.step_joint_position_targets(&unitree_g1_dex3_pick_targets(
                1.0,
                0.0,
                UnitreeG1Dex3HandCommand { closure: 0.0 },
            ));
        }
        let open_thumb = sim
            .named_transform("right_hand_thumb_2_link")
            .expect("right thumb tip")
            .translation;
        let open_index = sim
            .named_transform("right_hand_index_1_link")
            .expect("right index tip")
            .translation;
        for _ in 0..40 {
            sim.step_joint_position_targets(&unitree_g1_dex3_pick_targets(
                1.0,
                0.0,
                UnitreeG1Dex3HandCommand { closure: 1.0 },
            ));
        }
        let closed_thumb = sim
            .named_transform("right_hand_thumb_2_link")
            .expect("right thumb tip")
            .translation;
        let closed_index = sim
            .named_transform("right_hand_index_1_link")
            .expect("right index tip")
            .translation;
        let open_gap_m = open_thumb.distance(open_index);
        let closed_gap_m = closed_thumb.distance(closed_index);
        assert!(
            closed_gap_m < open_gap_m - 0.02,
            "Dex3 pinch did not close: open={open_gap_m} m closed={closed_gap_m} m"
        );
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
