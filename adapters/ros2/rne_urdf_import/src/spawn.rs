//! Spawn RNE entities from parsed URDF.

use crate::geometry::{collider_from_element, visual_from_element};
use crate::parse::rpy_to_quat;
use crate::schema::{UrdfJointType, UrdfLink, UrdfRobot};
use rne_ecs::{spawn_named, Entity, World};
use rne_physics::{RigidBody, RigidBodyType};
use rne_robot::{Joint, JointKind, JointLimits, Link, Robot, RobotId};
use rne_world::Transform3;
use std::collections::HashMap;
use thiserror::Error;

/// Configuration for URDF entity spawning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfSpawnConfig {
    /// When true, links with collision geometry receive rigid bodies.
    pub attach_physics: bool,
    /// Rigid body type applied to the base link.
    pub base_body_type: RigidBodyType,
    /// Default RGBA color for visual elements.
    pub visual_color_rgba: [f32; 4],
}

impl Default for UrdfSpawnConfig {
    fn default() -> Self {
        Self {
            attach_physics: true,
            base_body_type: RigidBodyType::Kinematic,
            visual_color_rgba: [0.7, 0.7, 0.75, 1.0],
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
/// Only the first visual element on each link is applied. Colliders and rigid bodies
/// are not modified.
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
        let Some(element) = link.visuals.first() else {
            continue;
        };
        world
            .entity_mut(*entity)
            .insert(visual_from_element(element, color_rgba));
        visual_count += 1;
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

    let base_link = links
        .get("base_link")
        .copied()
        .or_else(|| links.values().copied().next())
        .ok_or_else(|| UrdfSpawnError::InvalidGraph("no links".into()))?;

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
            limits: JointLimits::default(),
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
        let counts = attach_link_geometry(world, *entity, link, *entity == base_link, config);
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
    config: UrdfSpawnConfig,
) -> AttachCounts {
    let mut counts = AttachCounts {
        colliders: 0,
        visuals: 0,
    };

    if let Some(element) = link.collisions.first() {
        if let Some(collider) = collider_from_element(element) {
            world.entity_mut(entity).insert(collider);
            counts.colliders += 1;

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
        }
    }

    if let Some(element) = link.visuals.first() {
        world
            .entity_mut(entity)
            .insert(visual_from_element(element, config.visual_color_rgba));
        counts.visuals += 1;
    }

    counts
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
    use crate::parse::parse_urdf;
    use rne_physics::{Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc};
    use rne_physics_rapier::RapierBackend;
    use rne_render::Visual;

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
        }

        let attached =
            attach_urdf_visuals(&mut world, &urdf, &spawned.links, [0.7, 0.7, 0.75, 1.0]);
        assert_eq!(attached, 3);
        assert!(world.get::<Visual>(spawned.base_link).is_some());
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
