//! Scene and robot asset formats for Robot Native Engine.

#![deny(missing_docs)]

pub mod error;
pub mod robot;
pub mod scene;
pub mod spawn;

pub use error::AssetError;
pub use robot::{load_robot_asset, RobotAsset, RobotKind, UrdfRobotAsset};
pub use scene::{load_scene_asset, SceneAsset};
pub use spawn::{load_and_spawn_scene, spawn_robot_asset, spawn_scene, SpawnedRobot, SpawnedScene};
