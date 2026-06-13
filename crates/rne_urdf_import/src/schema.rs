//! Parsed URDF schema.

use rne_math::Vec3;

/// Parsed URDF robot description.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfRobot {
    /// Robot model name.
    pub name: String,
    /// Link definitions.
    pub links: Vec<UrdfLink>,
    /// Joint definitions.
    pub joints: Vec<UrdfJoint>,
}

/// Parsed URDF link.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfLink {
    /// Link name.
    pub name: String,
    /// Collision elements.
    pub collisions: Vec<UrdfGeometryElement>,
    /// Visual elements.
    pub visuals: Vec<UrdfGeometryElement>,
}

/// Geometry attached to a URDF collision or visual element.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfGeometryElement {
    /// Origin translation in meters.
    pub origin_xyz: Vec3,
    /// Origin roll-pitch-yaw in radians.
    pub origin_rpy: Vec3,
    /// Primitive or mesh geometry.
    pub geometry: UrdfGeometry,
}

/// Supported URDF geometry primitives.
#[derive(Clone, Debug, PartialEq)]
pub enum UrdfGeometry {
    /// Axis-aligned box with full size in meters.
    Box {
        /// Full size in meters.
        size_m: Vec3,
    },
    /// Sphere with radius in meters.
    Sphere {
        /// Radius in meters.
        radius_m: f64,
    },
    /// Cylinder aligned with the local Z axis.
    Cylinder {
        /// Radius in meters.
        radius_m: f64,
        /// Full length in meters.
        length_m: f64,
    },
    /// External mesh asset.
    Mesh {
        /// Mesh file path.
        path: String,
        /// Non-uniform scale.
        scale: Vec3,
    },
}

/// Supported URDF joint types.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UrdfJointType {
    /// Fixed joint.
    Fixed,
    /// Revolute joint.
    Revolute,
    /// Continuous revolute joint.
    Continuous,
    /// Prismatic joint.
    Prismatic,
}

/// Parsed URDF joint.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfJoint {
    /// Joint name.
    pub name: String,
    /// Joint type.
    pub joint_type: UrdfJointType,
    /// Parent link name.
    pub parent: String,
    /// Child link name.
    pub child: String,
    /// Origin translation in meters.
    pub origin_xyz: Vec3,
    /// Origin roll-pitch-yaw in radians.
    pub origin_rpy: Vec3,
    /// Joint axis in parent frame.
    pub axis: Vec3,
}
