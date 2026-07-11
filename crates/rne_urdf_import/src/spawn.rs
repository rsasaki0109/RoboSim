//! Spawn RNE entities from parsed URDF.

use crate::geometry::{collider_from_link, visual_from_element};
use crate::parse::rpy_to_quat;
use crate::schema::{UrdfJoint, UrdfJointType, UrdfLink, UrdfRobot};
use rne_ecs::{spawn_named, Entity, World};
use rne_physics::{RigidBody, RigidBodyType};
use rne_render::{LinkVisuals, Visual};
use rne_robot::{Joint, JointKind, JointLimits, Link, Robot, RobotId};
use rne_world::Transform3;
use std::collections::HashMap;
use std::path::PathBuf;
use thiserror::Error;

/// Configuration for URDF entity spawning.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfSpawnConfig {
    /// When true, links with collision geometry receive rigid bodies.
    pub attach_physics: bool,
    /// When true, URDF collision geometry is attached to link entities.
    pub attach_colliders: bool,
    /// Rigid body type applied to the base link.
    pub base_body_type: RigidBodyType,
    /// Default RGBA color for visual elements.
    pub visual_color_rgba: [f32; 4],
    /// Package root used to resolve `package://` mesh URIs for mesh collision AABBs.
    ///
    /// When `None`, mesh collision elements are skipped (legacy behavior).
    pub mesh_assets_root: Option<PathBuf>,
}

impl Default for UrdfSpawnConfig {
    fn default() -> Self {
        Self {
            attach_physics: true,
            attach_colliders: true,
            base_body_type: RigidBodyType::Kinematic,
            visual_color_rgba: [0.7, 0.7, 0.75, 1.0],
            mesh_assets_root: None,
        }
    }
}

/// Entities created from a URDF import.
#[derive(Clone, Debug, PartialEq)]
pub struct SpawnedUrdfRobot {
    /// Robot root entity.
    pub robot: Entity,
    /// Base link entity.
    pub base_link: Entity,
    /// Link entities keyed by URDF link name.
    pub links: HashMap<String, Entity>,
    /// Joint entities keyed by URDF joint name.
    pub joints: HashMap<String, Entity>,
    /// Number of colliders attached to link entities.
    pub collider_count: usize,
    /// Number of visuals attached to link entities.
    pub visual_count: usize,
}

/// URDF spawn error.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum UrdfSpawnError {
    /// Referenced link does not exist.
    #[error("unknown link {0}")]
    UnknownLink(String),
    /// Referenced parent/child relationship is invalid.
    #[error("invalid joint graph: {0}")]
    InvalidGraph(String),
}

/// Spawns Robot, Link, and Joint entities from a parsed URDF model.
pub fn spawn_urdf_robot(
    world: &mut World,
    urdf: &UrdfRobot,
) -> Result<SpawnedUrdfRobot, UrdfSpawnError> {
    spawn_urdf_robot_with_config(world, urdf, UrdfSpawnConfig::default())
}

/// Attaches URDF visual components to existing link entities keyed by link name.
///
/// All visual elements on each link are applied. Colliders and rigid bodies are not modified.
pub fn attach_urdf_visuals(
    world: &mut World,
    urdf: &UrdfRobot,
    links: &HashMap<String, Entity>,
    color_rgba: [f32; 4],
) -> usize {
    let mut visual_count = 0;
    for link in &urdf.links {
        let Some(entity) = links.get(&link.name) else {
            continue;
        };
        visual_count += attach_link_visuals(world, *entity, link, color_rgba);
    }
    visual_count
}

/// Spawns a URDF robot with explicit spawn configuration.
pub fn spawn_urdf_robot_with_config(
    world: &mut World,
    urdf: &UrdfRobot,
    config: UrdfSpawnConfig,
) -> Result<SpawnedUrdfRobot, UrdfSpawnError> {
    let robot_entity = spawn_named(world, &urdf.name);
    let mut links = HashMap::new();
    let mut link_defs = HashMap::new();

    for link in &urdf.links {
        let entity = spawn_named(world, &link.name);
        world.entity_mut(entity).insert(Link {
            robot: robot_entity,
            name: link.name.clone(),
        });
        links.insert(link.name.clone(), entity);
        link_defs.insert(link.name.clone(), link.clone());
    }

    let child_links: std::collections::HashSet<&str> = urdf
        .joints
        .iter()
        .map(|joint| joint.child.as_str())
        .collect();
    let mut root_names: Vec<&str> = urdf
        .links
        .iter()
        .map(|link| link.name.as_str())
        .filter(|name| !child_links.contains(name))
        .collect();
    root_names.sort_unstable();
    let root_name = match root_names.as_slice() {
        [root] => *root,
        [] => {
            return Err(UrdfSpawnError::InvalidGraph(
                "no root link (joint graph contains a cycle)".into(),
            ));
        }
        roots => {
            return Err(UrdfSpawnError::InvalidGraph(format!(
                "multiple root links: {}",
                roots.join(", ")
            )));
        }
    };
    let base_link = links[root_name];

    world.entity_mut(base_link).insert(Transform3::IDENTITY);

    world.entity_mut(robot_entity).insert(Robot {
        robot_id: RobotId::new_v4(),
        model_name: urdf.name.clone(),
        base_link,
    });

    let mut joints = HashMap::new();
    for joint in &urdf.joints {
        let parent = *links
            .get(&joint.parent)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.parent.clone()))?;
        let child = *links
            .get(&joint.child)
            .ok_or_else(|| UrdfSpawnError::UnknownLink(joint.child.clone()))?;

        let entity = spawn_named(world, &joint.name);
        world.entity_mut(entity).insert(Joint {
            robot: robot_entity,
            parent_link: parent,
            child_link: child,
            kind: map_joint_kind(joint.joint_type),
            limits: joint_limits_from_urdf(joint),
            axis: joint.axis,
            position: 0.0,
            velocity: 0.0,
        });
        world.entity_mut(child).insert((
            Transform3::from_translation_rotation(joint.origin_xyz, rpy_to_quat(joint.origin_rpy)),
            rne_ecs::Parent(parent),
        ));
        joints.insert(joint.name.clone(), entity);
    }

    let mut collider_count = 0;
    let mut visual_count = 0;
    for (name, entity) in &links {
        let Some(link) = link_defs.get(name) else {
            continue;
        };
        let counts = attach_link_geometry(world, *entity, link, *entity == base_link, &config);
        collider_count += counts.colliders;
        visual_count += counts.visuals;
    }

    Ok(SpawnedUrdfRobot {
        robot: robot_entity,
        base_link,
        links,
        joints,
        collider_count,
        visual_count,
    })
}

struct AttachCounts {
    colliders: usize,
    visuals: usize,
}

fn attach_link_geometry(
    world: &mut World,
    entity: Entity,
    link: &UrdfLink,
    is_base_link: bool,
    config: &UrdfSpawnConfig,
) -> AttachCounts {
    let mut counts = AttachCounts {
        colliders: 0,
        visuals: 0,
    };
    let assets_root = config.mesh_assets_root.as_deref();

    if config.attach_colliders {
        if let Some(collider) = collider_from_link(link, assets_root) {
            world.entity_mut(entity).insert(collider);
            counts.colliders += 1;
        }
    }

    // A URDF link can legitimately have inertia and joints without collision geometry.
    // It still needs a rigid body so an articulated visual-only root can anchor its tree.
    if config.attach_physics {
        let body_type = if is_base_link {
            config.base_body_type
        } else {
            RigidBodyType::Dynamic
        };
        world.entity_mut(entity).insert(RigidBody {
            body_type,
            mass_kg: if is_base_link { 5.0 } else { 1.0 },
            ..RigidBody::default()
        });
    }

    counts.visuals += attach_link_visuals(world, entity, link, config.visual_color_rgba);

    counts
}

fn attach_link_visuals(
    world: &mut World,
    entity: Entity,
    link: &UrdfLink,
    color_rgba: [f32; 4],
) -> usize {
    if link.visuals.is_empty() {
        return 0;
    }
    if link.visuals.len() == 1 {
        world
            .entity_mut(entity)
            .insert(visual_from_element(&link.visuals[0], color_rgba));
        return 1;
    }
    let visuals: Vec<Visual> = link
        .visuals
        .iter()
        .map(|element| visual_from_element(element, color_rgba))
        .collect();
    world.entity_mut(entity).insert(LinkVisuals { visuals });
    link.visuals.len()
}

fn joint_limits_from_urdf(joint: &UrdfJoint) -> JointLimits {
    let Some(limit) = joint.limit else {
        return JointLimits::default();
    };
    JointLimits {
        lower: limit.lower,
        upper: limit.upper,
        max_velocity: limit.max_velocity_rad_s,
        max_effort: limit.max_effort_nm,
    }
}

fn map_joint_kind(joint_type: UrdfJointType) -> JointKind {
    match joint_type {
        UrdfJointType::Fixed => JointKind::Fixed,
        UrdfJointType::Revolute => JointKind::Revolute,
        UrdfJointType::Continuous => JointKind::Continuous,
        UrdfJointType::Prismatic => JointKind::Prismatic,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parse::{parse_urdf, parse_urdf_file};
    use rne_physics::{Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc};
    use rne_physics_rapier::RapierBackend;
    use rne_render::Visual;
    use std::path::PathBuf;

    const FIXTURE: &str = include_str!("../tests/fixtures/minimal_diff_drive.urdf");

    #[test]
    fn fixture_urdf_spawns_robot_links_and_joints() {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot(&mut world, &urdf).unwrap();

        assert!(world.get::<Robot>(spawned.robot).is_some());
        assert_eq!(spawned.links.len(), 3);
        assert_eq!(spawned.joints.len(), 2);
        assert!(world.get::<Link>(spawned.base_link).is_some());

        for joint_entity in spawned.joints.values() {
            assert!(world.get::<Joint>(*joint_entity).is_some());
        }
    }

    #[test]
    fn fixture_attaches_colliders_and_visuals() {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot(&mut world, &urdf).unwrap();

        assert_eq!(spawned.collider_count, 3);
        assert_eq!(spawned.visual_count, 3);
        assert!(world.get::<Collider>(spawned.base_link).is_some());
        assert!(world.get::<Visual>(spawned.base_link).is_some());
        assert!(world.get::<RigidBody>(spawned.base_link).is_some());

        let left_wheel = spawned.links["left_wheel"];
        let collider = world.get::<Collider>(left_wheel).expect("wheel collider");
        assert!(matches!(collider.shape, ColliderShape::Capsule { .. }));
    }

    #[test]
    fn attach_urdf_visuals_matches_existing_links() {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let mut world = World::new();
        let spawned = spawn_urdf_robot(&mut world, &urdf).unwrap();

        for entity in spawned.links.values() {
            world.entity_mut(*entity).remove::<Visual>();
            world.entity_mut(*entity).remove::<LinkVisuals>();
        }

        let attached =
            attach_urdf_visuals(&mut world, &urdf, &spawned.links, [0.7, 0.7, 0.75, 1.0]);
        assert_eq!(attached, 3);
        assert!(world.get::<Visual>(spawned.base_link).is_some());
    }

    #[test]
    fn mesh_collision_fixture_spawns_aabb_collider() {
        let fixture = include_str!("../tests/fixtures/mesh_collision_base.urdf");
        let urdf = parse_urdf(fixture).unwrap();
        let package_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/mesh_diff_drive_package");
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                mesh_assets_root: Some(package_root),
                ..UrdfSpawnConfig::default()
            },
        )
        .unwrap();
        assert_eq!(spawned.collider_count, 1);
        let collider = world
            .get::<Collider>(spawned.base_link)
            .expect("mesh aabb collider");
        assert!(matches!(collider.shape, ColliderShape::Cuboid { .. }));
    }

    #[test]
    fn vendored_so101_urdf_parses() {
        let path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots/so101/so101.urdf");
        assert!(path.is_file(), "missing {}", path.display());
        let robot = parse_urdf_file(&path).expect("parse so101");
        assert_eq!(robot.name, "so101_new_calib");
        assert!(robot.links.len() >= 6);
        assert!(robot.joints.iter().any(|joint| joint.limit.is_some()));
        let base = robot
            .links
            .iter()
            .find(|link| link.name == "base_link")
            .expect("base_link");
        assert!(base.visuals.len() >= 2);
        assert!(base.collisions.len() >= 2);
    }

    #[test]
    fn fixture_colliders_sync_to_physics() {
        let urdf = parse_urdf(FIXTURE).unwrap();
        let mut world = World::new();
        spawn_urdf_robot(&mut world, &urdf).unwrap();

        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
    }
}
