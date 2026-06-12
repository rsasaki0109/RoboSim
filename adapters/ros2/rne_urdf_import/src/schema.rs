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
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UrdfLink {
    /// Link name.
    pub name: String,
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
