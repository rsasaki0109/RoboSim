//! Scene and robot asset formats for Robot Native Engine.

#![deny(missing_docs)]

pub mod error;
pub mod pipeline;
pub mod robot;
pub mod scene;
pub mod spawn;

pub use error::AssetError;
pub use pipeline::{
    inspect_asset, is_robot_asset_path, is_scene_asset_path, load_scene_bundle, mesh_package_roots,
    scene_dependency_paths, smoke_spawn_scene, validate_asset, validate_robot_references,
    AssetHotReloader, AssetRevision, SceneAssetBundle, ValidatedAsset,
};
pub use robot::{
    load_robot_asset, parse_robot_asset, LidarRobotAsset, RobotAsset, RobotKind, UrdfRobotAsset,
    VisualsRobotAsset, WristCameraRobotAsset,
};
pub use scene::{load_scene_asset, SceneAsset, SceneObstacleAsset};
pub use spawn::{
    load_and_spawn_scene, spawn_robot_asset, spawn_scene, LidarMountSpawned, RobotSensorMounts,
    SpawnedRobot, SpawnedScene, WristCameraMountSpawned,
};
