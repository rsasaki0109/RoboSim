//! Minimal URDF importer for Robot Native Engine.

#![deny(missing_docs)]

pub mod articulation;
pub mod geometry;
pub mod parse;
pub mod schema;
pub mod spawn;

pub use articulation::{
    attach_urdf_articulation, UrdfArticulationAttached, UrdfArticulationConfig,
};
pub use parse::{
    parse_urdf, parse_urdf_document, parse_urdf_document_file, parse_urdf_file, rpy_to_quat,
    UrdfParseError, URDF_MAX_INPUT_BYTES,
};
pub use schema::{
    UrdfDocument, UrdfGeometry, UrdfGeometryElement, UrdfInertial, UrdfJoint, UrdfJointDynamics,
    UrdfJointLimit, UrdfJointMimic, UrdfJointType, UrdfLink, UrdfRobot,
};
pub use spawn::{
    attach_urdf_visuals, spawn_urdf_document, spawn_urdf_document_with_config, spawn_urdf_robot,
    spawn_urdf_robot_with_config, SpawnedUrdfRobot, UrdfSpawnConfig, UrdfSpawnError,
};
