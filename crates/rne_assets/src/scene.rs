//! Scene asset schema and loading.

use crate::error::AssetError;
use crate::robot::RobotAsset;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Parsed `.rne.scene.toml` asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneAsset {
    /// World-level configuration.
    #[serde(default)]
    pub world: SceneWorldAsset,
    /// Optional ground plane configuration.
    #[serde(default)]
    pub ground: GroundAsset,
    /// Robot asset references to spawn into the scene.
    #[serde(default)]
    pub robots: Vec<SceneRobotRef>,
}

/// World configuration stored in a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneWorldAsset {
    /// Gravity vector in meters per second squared.
    #[serde(default = "default_gravity")]
    pub gravity_m_s2: [f64; 3],
    /// Deterministic random seed.
    #[serde(default)]
    pub seed: u64,
}

/// Ground plane configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundAsset {
    /// When true, a fixed ground collider is spawned.
    #[serde(default = "default_enabled")]
    pub enabled: bool,
}

/// Reference to a robot asset file from a scene.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneRobotRef {
    /// Path to a `.rne.robot.toml` file, relative to the scene file unless absolute.
    pub path: String,
}

impl Default for SceneWorldAsset {
    fn default() -> Self {
        Self {
            gravity_m_s2: default_gravity(),
            seed: 0,
        }
    }
}

impl Default for GroundAsset {
    fn default() -> Self {
        Self {
            enabled: default_enabled(),
        }
    }
}

impl SceneRobotRef {
    /// Resolves the robot asset path relative to a base directory.
    pub fn resolve_path(&self, base_dir: &Path) -> PathBuf {
        let path = Path::new(&self.path);
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            base_dir.join(path)
        }
    }
}

/// Loads a scene asset from a `.rne.scene.toml` file.
pub fn load_scene_asset(path: &Path) -> Result<SceneAsset, AssetError> {
    let text = std::fs::read_to_string(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_scene_asset(&text, path)
}

/// Parses scene asset TOML from memory.
pub fn parse_scene_asset(text: &str, path: &Path) -> Result<SceneAsset, AssetError> {
    toml::from_str(text)
        .map_err(|error| AssetError::invalid(path.display().to_string(), error.to_string()))
}

/// Loads all robot assets referenced by a scene file.
pub fn load_scene_robots(
    scene_path: &Path,
    scene: &SceneAsset,
) -> Result<Vec<(PathBuf, RobotAsset)>, AssetError> {
    let base_dir = scene_path.parent().unwrap_or_else(|| Path::new("."));
    scene
        .robots
        .iter()
        .map(|reference| {
            let robot_path = reference.resolve_path(base_dir);
            let robot = crate::robot::load_robot_asset(&robot_path)?;
            Ok((robot_path, robot))
        })
        .collect()
}

fn default_gravity() -> [f64; 3] {
    [0.0, -9.81, 0.0]
}

fn default_enabled() -> bool {
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::RobotKind;

    const SCENE: &str = include_str!("../tests/fixtures/episode_diff_drive.rne.scene.toml");

    #[test]
    fn parses_scene_fixture() {
        let scene = super::parse_scene_asset(SCENE, Path::new("scene.toml")).unwrap();
        assert_eq!(scene.world.seed, 42);
        assert!(scene.ground.enabled);
        assert_eq!(scene.robots.len(), 1);
    }

    #[test]
    fn load_scene_robots_from_fixture_dir() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/episode_diff_drive.rne.scene.toml");
        let scene = load_scene_asset(&scene_path).unwrap();
        let robots = load_scene_robots(&scene_path, &scene).unwrap();
        assert_eq!(robots.len(), 1);
        assert_eq!(robots[0].1.kind, RobotKind::DiffDrive);
    }
}
