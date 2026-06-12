//! Spawn RNE entities from parsed URDF.

use crate::parse::rpy_to_quat;
use crate::schema::{UrdfJointType, UrdfRobot};
use rne_ecs::{spawn_named, Entity, World};
use rne_robot::{Joint, JointKind, JointLimits, Link, Robot, RobotId};
use rne_world::Transform3;
use std::collections::HashMap;
use thiserror::Error;

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
    let robot_entity = spawn_named(world, &urdf.name);
    let mut links = HashMap::new();

    for link in &urdf.links {
        let entity = spawn_named(world, &link.name);
        world.entity_mut(entity).insert(Link {
            robot: robot_entity,
            name: link.name.clone(),
        });
        links.insert(link.name.clone(), entity);
    }

    let base_link = links
        .get("base_link")
        .copied()
        .or_else(|| links.values().copied().next())
        .ok_or_else(|| UrdfSpawnError::InvalidGraph("no links".into()))?;

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

    Ok(SpawnedUrdfRobot {
        robot: robot_entity,
        base_link,
        links,
        joints,
    })
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
}
