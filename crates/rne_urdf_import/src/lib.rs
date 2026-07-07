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
pub use parse::{parse_urdf, parse_urdf_file, rpy_to_quat, UrdfParseError};
pub use schema::{
    UrdfGeometry, UrdfGeometryElement, UrdfJoint, UrdfJointLimit, UrdfJointMimic, UrdfJointType,
    UrdfLink, UrdfRobot,
};
pub use spawn::{
    attach_urdf_visuals, spawn_urdf_robot, spawn_urdf_robot_with_config, SpawnedUrdfRobot,
    UrdfSpawnConfig, UrdfSpawnError,
};
