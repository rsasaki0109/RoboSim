//! Scene and robot asset formats for Robot Native Engine.

#![deny(missing_docs)]

pub mod error;
pub mod pipeline;
pub mod robot;
pub mod scene;
pub mod spawn;

pub use error::AssetError;
pub use pipeline::{
    inspect_asset, is_robot_asset_path, is_scene_asset_path, load_scene_bundle,
    scene_dependency_paths, smoke_spawn_scene, validate_asset, validate_robot_references,
    AssetHotReloader, AssetRevision, SceneAssetBundle, ValidatedAsset,
};
pub use robot::{load_robot_asset, RobotAsset, RobotKind, UrdfRobotAsset};
pub use scene::{load_scene_asset, SceneAsset};
pub use spawn::{load_and_spawn_scene, spawn_robot_asset, spawn_scene, SpawnedRobot, SpawnedScene};
