//! URDF parsing.

use crate::schema::{
    UrdfGeometry, UrdfGeometryElement, UrdfJoint, UrdfJointLimit, UrdfJointMimic, UrdfJointType,
    UrdfLink, UrdfRobot,
};
use rne_math::{Quat, Vec3};
use roxmltree::Document;
use std::path::Path;
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

/// Parses a URDF document from a file path.
pub fn parse_urdf_file(path: &Path) -> Result<UrdfRobot, UrdfParseError> {
    let xml = std::fs::read_to_string(path).map_err(|error| {
        UrdfParseError::InvalidXml(format!("failed to read {}: {error}", path.display()))
    })?;
    parse_urdf(&xml)
}

fn parse_link(node: roxmltree::Node<'_, '_>) -> Result<UrdfLink, UrdfParseError> {
    let name = node
        .attribute("name")
        .ok_or_else(|| UrdfParseError::Missing("link@name".into()))?
        .to_string();

    let mut collisions = Vec::new();
    let mut visuals = Vec::new();
    for child in node.children().filter(|node| node.is_element()) {
        match child.tag_name().name() {
            "collision" => collisions.push(parse_geometry_element(child)?),
            "visual" => visuals.push(parse_geometry_element(child)?),
            _ => {}
        }
    }

    Ok(UrdfLink {
        name,
        collisions,
        visuals,
    })
}

fn parse_geometry_element(
    node: roxmltree::Node<'_, '_>,
) -> Result<UrdfGeometryElement, UrdfParseError> {
    let origin = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "origin");
    let (origin_xyz, origin_rpy) = parse_origin(origin);
    let geometry = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "geometry")
        .ok_or_else(|| UrdfParseError::Missing("geometry".into()))
        .and_then(parse_geometry)?;
    let material_rgba = parse_material_rgba(node)?;

    Ok(UrdfGeometryElement {
        origin_xyz,
        origin_rpy,
        material_rgba,
        geometry,
    })
}

fn parse_material_rgba(node: roxmltree::Node<'_, '_>) -> Result<Option<[f32; 4]>, UrdfParseError> {
    let Some(material) = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "material")
    else {
        return Ok(None);
    };
    let Some(color) = material
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "color")
    else {
        return Ok(None);
    };
    color.attribute("rgba").map(parse_rgba).transpose()
}

fn parse_geometry(node: roxmltree::Node<'_, '_>) -> Result<UrdfGeometry, UrdfParseError> {
    let primitive = node
        .children()
        .find(|node| node.is_element())
        .ok_or_else(|| UrdfParseError::Missing("geometry primitive".into()))?;

    match primitive.tag_name().name() {
        "box" => {
            let size = primitive
                .attribute("size")
                .ok_or_else(|| UrdfParseError::Missing("box@size".into()))
                .and_then(parse_vec3)?;
            Ok(UrdfGeometry::Box { size_m: size })
        }
        "sphere" => {
            let radius_m = primitive
                .attribute("radius")
                .ok_or_else(|| UrdfParseError::Missing("sphere@radius".into()))
                .and_then(parse_f64)?;
            Ok(UrdfGeometry::Sphere { radius_m })
        }
        "cylinder" => {
            let radius_m = primitive
                .attribute("radius")
                .ok_or_else(|| UrdfParseError::Missing("cylinder@radius".into()))
                .and_then(parse_f64)?;
            let length_m = primitive
                .attribute("length")
                .ok_or_else(|| UrdfParseError::Missing("cylinder@length".into()))
                .and_then(parse_f64)?;
            Ok(UrdfGeometry::Cylinder { radius_m, length_m })
        }
        "mesh" => {
            let path = primitive
                .attribute("filename")
                .ok_or_else(|| UrdfParseError::Missing("mesh@filename".into()))?
                .to_string();
            let scale = primitive
                .attribute("scale")
                .map(parse_vec3)
                .transpose()?
                .unwrap_or(Vec3::ONE);
            Ok(UrdfGeometry::Mesh { path, scale })
        }
        other => Err(UrdfParseError::InvalidValue {
            field: "geometry".into(),
            value: other.into(),
        }),
    }
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

    let limit = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "limit")
        .map(parse_joint_limit)
        .transpose()?;
    let mimic = node
        .children()
        .find(|node| node.is_element() && node.tag_name().name() == "mimic")
        .map(parse_joint_mimic)
        .transpose()?;

    Ok(UrdfJoint {
        name,
        joint_type,
        parent,
        child,
        origin_xyz,
        origin_rpy,
        axis,
        limit,
        mimic,
    })
}

fn parse_joint_limit(node: roxmltree::Node<'_, '_>) -> Result<UrdfJointLimit, UrdfParseError> {
    let lower = node
        .attribute("lower")
        .ok_or_else(|| UrdfParseError::Missing("limit@lower".into()))
        .and_then(parse_f64)?;
    let upper = node
        .attribute("upper")
        .ok_or_else(|| UrdfParseError::Missing("limit@upper".into()))
        .and_then(parse_f64)?;
    let max_velocity_rad_s = node
        .attribute("velocity")
        .ok_or_else(|| UrdfParseError::Missing("limit@velocity".into()))
        .and_then(parse_f64)?;
    let max_effort_nm = node
        .attribute("effort")
        .ok_or_else(|| UrdfParseError::Missing("limit@effort".into()))
        .and_then(parse_f64)?;
    Ok(UrdfJointLimit {
        lower,
        upper,
        max_velocity_rad_s,
        max_effort_nm,
    })
}

fn parse_joint_mimic(node: roxmltree::Node<'_, '_>) -> Result<UrdfJointMimic, UrdfParseError> {
    let joint = node
        .attribute("joint")
        .ok_or_else(|| UrdfParseError::Missing("mimic@joint".into()))?
        .to_string();
    let multiplier = node
        .attribute("multiplier")
        .map(parse_f64)
        .transpose()?
        .unwrap_or(1.0);
    let offset = node
        .attribute("offset")
        .map(parse_f64)
        .transpose()?
        .unwrap_or(0.0);
    Ok(UrdfJointMimic {
        joint,
        multiplier,
        offset,
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

fn parse_rgba(value: &str) -> Result<[f32; 4], UrdfParseError> {
    let parts: Vec<_> = value.split_whitespace().collect();
    if parts.len() != 4 {
        return Err(UrdfParseError::InvalidValue {
            field: "material/color@rgba".into(),
            value: value.into(),
        });
    }

    let parse = |part: &str| -> Result<f32, UrdfParseError> {
        let component: f32 = part.parse().map_err(|_| UrdfParseError::InvalidValue {
            field: "material/color@rgba".into(),
            value: value.into(),
        })?;
        if !component.is_finite() {
            return Err(UrdfParseError::InvalidValue {
                field: "material/color@rgba".into(),
                value: value.into(),
            });
        }
        Ok(component)
    };

    Ok([
        parse(parts[0])?,
        parse(parts[1])?,
        parse(parts[2])?,
        parse(parts[3])?,
    ])
}

fn parse_f64(value: &str) -> Result<f64, UrdfParseError> {
    value.parse().map_err(|_| UrdfParseError::InvalidValue {
        field: "f64".into(),
        value: value.into(),
    })
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

        let base = robot
            .links
            .iter()
            .find(|link| link.name == "base_link")
            .expect("base_link");
        assert_eq!(base.collisions.len(), 1);
        assert_eq!(base.visuals.len(), 1);
        assert!(matches!(
            base.collisions[0].geometry,
            UrdfGeometry::Box { .. }
        ));

        let wheel = robot
            .links
            .iter()
            .find(|link| link.name == "left_wheel")
            .expect("left_wheel");
        assert!(matches!(
            wheel.collisions[0].geometry,
            UrdfGeometry::Cylinder { .. }
        ));
    }

    #[test]
    fn parse_mesh_diff_drive() {
        let fixture = include_str!("../tests/fixtures/mesh_diff_drive.urdf");
        let robot = parse_urdf(fixture).unwrap();
        let base = robot
            .links
            .iter()
            .find(|link| link.name == "base_link")
            .expect("base_link");
        assert!(matches!(
            base.visuals[0].geometry,
            UrdfGeometry::Mesh { .. }
        ));
    }

    #[test]
    fn parses_revolute_joint_limits() {
        let robot = parse_urdf(
            r#"
            <robot name="limited_arm">
              <link name="base_link"/>
              <link name="arm_link"/>
              <joint name="shoulder" type="revolute">
                <parent link="base_link"/>
                <child link="arm_link"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.5" upper="1.5" velocity="2.0" effort="10.0"/>
              </joint>
            </robot>
            "#,
        )
        .unwrap();
        let limit = robot.joints[0].limit.expect("limit");
        assert_eq!(limit.lower, -1.5);
        assert_eq!(limit.upper, 1.5);
        assert_eq!(limit.max_velocity_rad_s, 2.0);
        assert_eq!(limit.max_effort_nm, 10.0);
    }

    #[test]
    fn parses_joint_mimic_metadata() {
        let robot = parse_urdf(
            r#"
            <robot name="mimic_gripper">
              <link name="base_link"/>
              <link name="finger_a"/>
              <link name="finger_b"/>
              <joint name="finger_a_joint" type="revolute">
                <parent link="base_link"/>
                <child link="finger_a"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="finger_b_joint" type="revolute">
                <parent link="base_link"/>
                <child link="finger_b"/>
                <axis xyz="0 0 1"/>
                <mimic joint="finger_a_joint" multiplier="-1.0" offset="0.0"/>
              </joint>
            </robot>
            "#,
        )
        .unwrap();
        let mimic = robot.joints[1].mimic.as_ref().expect("mimic");
        assert_eq!(mimic.joint, "finger_a_joint");
        assert_eq!(mimic.multiplier, -1.0);
        assert_eq!(mimic.offset, 0.0);
    }

    #[test]
    fn parses_inline_visual_material_color() {
        let robot = parse_urdf(
            r#"
            <robot name="material_robot">
              <link name="wheel">
                <visual>
                  <material name="wheel_black">
                    <color rgba="0.08 0.08 0.08 1.0"/>
                  </material>
                  <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                  </geometry>
                </visual>
              </link>
            </robot>
            "#,
        )
        .unwrap();
        assert_eq!(
            robot.links[0].visuals[0].material_rgba,
            Some([0.08, 0.08, 0.08, 1.0])
        );
    }
}
