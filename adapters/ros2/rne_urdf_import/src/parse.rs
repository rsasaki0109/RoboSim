//! URDF parsing.

use crate::schema::{UrdfJoint, UrdfJointType, UrdfLink, UrdfRobot};
use rne_math::{Quat, Vec3};
use roxmltree::Document;
use thiserror::Error;

/// URDF parse error.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum UrdfParseError {
    /// XML parsing failed.
    #[error("invalid XML: {0}")]
    InvalidXml(String),
    /// Missing required element.
    #[error("missing {0}")]
    Missing(String),
    /// Unsupported or invalid value.
    #[error("invalid {field}: {value}")]
    InvalidValue {
        /// Field name.
        field: String,
        /// Offending value.
        value: String,
    },
}

/// Parses a URDF document from XML text.
pub fn parse_urdf(xml: &str) -> Result<UrdfRobot, UrdfParseError> {
    let document =
        Document::parse(xml).map_err(|error| UrdfParseError::InvalidXml(error.to_string()))?;
    let robot = document.root_element();
    if robot.tag_name().name() != "robot" {
        return Err(UrdfParseError::Missing("robot element".into()));
    }

    let name = robot
        .attribute("name")
        .ok_or_else(|| UrdfParseError::Missing("robot@name".into()))?
        .to_string();

    let mut links = Vec::new();
    let mut joints = Vec::new();

    for child in robot.children().filter(|node| node.is_element()) {
        match child.tag_name().name() {
            "link" => links.push(parse_link(child)?),
            "joint" => joints.push(parse_joint(child)?),
            _ => {}
        }
    }

    if links.is_empty() {
        return Err(UrdfParseError::Missing("at least one link".into()));
    }

    Ok(UrdfRobot {
        name,
        links,
        joints,
    })
}

fn parse_link(node: roxmltree::Node<'_, '_>) -> Result<UrdfLink, UrdfParseError> {
    let name = node
        .attribute("name")
        .ok_or_else(|| UrdfParseError::Missing("link@name".into()))?
        .to_string();
    Ok(UrdfLink { name })
}

fn parse_joint(node: roxmltree::Node<'_, '_>) -> Result<UrdfJoint, UrdfParseError> {
    let name = node
        .attribute("name")
        .ok_or_else(|| UrdfParseError::Missing("joint@name".into()))?
        .to_string();
    let joint_type = node
        .attribute("type")
        .ok_or_else(|| UrdfParseError::Missing("joint@type".into()))
        .and_then(parse_joint_type)?;

    let parent = child_text(node, "parent", "link")?;
    let child = child_text(node, "child", "link")?;
    let origin = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "origin");
    let (origin_xyz, origin_rpy) = parse_origin(origin);
    let axis = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "axis")
        .and_then(|node| node.attribute("xyz"))
        .map(parse_vec3)
        .transpose()?
        .unwrap_or(Vec3::Z);

    Ok(UrdfJoint {
        name,
        joint_type,
        parent,
        child,
        origin_xyz,
        origin_rpy,
        axis,
    })
}

fn child_text(
    node: roxmltree::Node<'_, '_>,
    tag: &str,
    attribute: &str,
) -> Result<String, UrdfParseError> {
    node.children()
        .find(|node| node.is_element() && node.tag_name().name() == tag)
        .and_then(|node| node.attribute(attribute))
        .map(str::to_string)
        .ok_or_else(|| UrdfParseError::Missing(format!("joint/{tag}@{attribute}")))
}

fn parse_joint_type(value: &str) -> Result<UrdfJointType, UrdfParseError> {
    match value {
        "fixed" => Ok(UrdfJointType::Fixed),
        "revolute" => Ok(UrdfJointType::Revolute),
        "continuous" => Ok(UrdfJointType::Continuous),
        "prismatic" => Ok(UrdfJointType::Prismatic),
        other => Err(UrdfParseError::InvalidValue {
            field: "joint@type".into(),
            value: other.into(),
        }),
    }
}

fn parse_origin(origin: Option<roxmltree::Node<'_, '_>>) -> (Vec3, Vec3) {
    let Some(origin) = origin else {
        return (Vec3::ZERO, Vec3::ZERO);
    };

    let xyz = origin
        .attribute("xyz")
        .and_then(|value| parse_vec3(value).ok())
        .unwrap_or(Vec3::ZERO);
    let rpy = origin
        .attribute("rpy")
        .and_then(|value| parse_vec3(value).ok())
        .unwrap_or(Vec3::ZERO);
    (xyz, rpy)
}

fn parse_vec3(value: &str) -> Result<Vec3, UrdfParseError> {
    let parts: Vec<_> = value.split_whitespace().collect();
    if parts.len() != 3 {
        return Err(UrdfParseError::InvalidValue {
            field: "vec3".into(),
            value: value.into(),
        });
    }

    let parse = |part: &str| -> Result<f64, UrdfParseError> {
        part.parse().map_err(|_| UrdfParseError::InvalidValue {
            field: "vec3".into(),
            value: value.into(),
        })
    };

    Ok(Vec3::new(
        parse(parts[0])?,
        parse(parts[1])?,
        parse(parts[2])?,
    ))
}

/// Converts roll-pitch-yaw to a quaternion.
pub fn rpy_to_quat(rpy: Vec3) -> Quat {
    Quat::from_rotation_z(rpy.z) * Quat::from_rotation_y(rpy.y) * Quat::from_rotation_x(rpy.x)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../tests/fixtures/minimal_diff_drive.urdf");

    #[test]
    fn parse_minimal_diff_drive() {
        let robot = parse_urdf(FIXTURE).unwrap();
        assert_eq!(robot.name, "diff_drive");
        assert_eq!(robot.links.len(), 3);
        assert_eq!(robot.joints.len(), 2);
        assert_eq!(robot.joints[0].joint_type, UrdfJointType::Continuous);
    }
}
