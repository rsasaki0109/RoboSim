//! Parsed URDF schema.

use rne_math::Vec3;
use std::collections::BTreeMap;

/// Parsed URDF document with optional extended properties kept outside the
/// compatibility-stable [`UrdfRobot`], [`UrdfLink`], and [`UrdfJoint`] layouts.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfDocument {
    /// Robot topology, geometry, joints, and legacy inertial mass fields.
    pub robot: UrdfRobot,
    /// Complete inertial properties keyed by link name.
    pub inertials: BTreeMap<String, UrdfInertial>,
    /// Passive dynamics keyed by joint name.
    pub joint_dynamics: BTreeMap<String, UrdfJointDynamics>,
}

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
    /// Optional mass declared by `<inertial><mass value="..."/></inertial>`, in kilograms.
    pub inertial_mass_kg: Option<f64>,
    /// Collision elements.
    pub collisions: Vec<UrdfGeometryElement>,
    /// Visual elements.
    pub visuals: Vec<UrdfGeometryElement>,
}

/// Complete inertial properties declared by a URDF `<inertial>` element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfInertial {
    /// Centre-of-mass translation in the link frame, in metres.
    pub origin_xyz_m: Vec3,
    /// Inertial-frame roll-pitch-yaw rotation relative to the link frame, in radians.
    pub origin_rpy_rad: Vec3,
    /// Declared mass in kilograms.
    pub mass_kg: f64,
    /// Inertia tensor x-x entry in kg·m², expressed in the inertial frame.
    pub ixx_kg_m2: f64,
    /// Inertia tensor x-y entry in kg·m², expressed in the inertial frame.
    pub ixy_kg_m2: f64,
    /// Inertia tensor x-z entry in kg·m², expressed in the inertial frame.
    pub ixz_kg_m2: f64,
    /// Inertia tensor y-y entry in kg·m², expressed in the inertial frame.
    pub iyy_kg_m2: f64,
    /// Inertia tensor y-z entry in kg·m², expressed in the inertial frame.
    pub iyz_kg_m2: f64,
    /// Inertia tensor z-z entry in kg·m², expressed in the inertial frame.
    pub izz_kg_m2: f64,
}

/// Geometry attached to a URDF collision or visual element.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfGeometryElement {
    /// Origin translation in meters.
    pub origin_xyz: Vec3,
    /// Origin roll-pitch-yaw in radians.
    pub origin_rpy: Vec3,
    /// Optional material color in RGBA components.
    pub material_rgba: Option<[f32; 4]>,
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

/// Optional joint limits from a URDF `<limit>` element.
///
/// Units depend on [`UrdfJointType`]:
/// - Revolute / continuous: `lower` and `upper` are radians; `max_velocity_rad_s` is rad/s;
///   `max_effort_nm` is newton-meters.
/// - Prismatic: `lower` and `upper` are meters; `max_velocity_rad_s` is m/s; `max_effort_nm` is
///   newtons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfJointLimit {
    /// Lower position limit.
    pub lower: f64,
    /// Upper position limit.
    pub upper: f64,
    /// Maximum velocity.
    pub max_velocity_rad_s: f64,
    /// Maximum effort / force.
    pub max_effort_nm: f64,
}

/// Parsed URDF `<mimic>` element (kinematic coupling metadata only).
///
/// Mimic joints are recorded at import time but are **not** wired into the physics backend.
/// Actuators must drive the leader joint explicitly.
#[derive(Clone, Debug, PartialEq)]
pub struct UrdfJointMimic {
    /// Name of the joint whose motion is followed.
    pub joint: String,
    /// Position multiplier applied to the leader joint value.
    pub multiplier: f64,
    /// Constant offset added after scaling.
    pub offset: f64,
}

/// Parsed URDF `<dynamics>` coefficients for a joint.
///
/// URDF uses the same attributes for revolute and prismatic joints; their SI
/// units are interpreted from [`UrdfJointType`] when the articulation is
/// attached to the physics world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UrdfJointDynamics {
    /// Viscous damping coefficient. Units are N·m·s/rad for revolute joints and
    /// N·s/m for prismatic joints.
    pub damping: f64,
    /// Coulomb-friction magnitude. Units are N·m for revolute joints and N for
    /// prismatic joints.
    pub friction: f64,
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
    /// Optional joint limits from `<limit>`.
    pub limit: Option<UrdfJointLimit>,
    /// Optional mimic coupling from `<mimic>` (parse-only; not simulated).
    pub mimic: Option<UrdfJointMimic>,
}
