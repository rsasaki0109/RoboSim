//! Scene asset schema and loading.

use crate::error::AssetError;
use crate::robot::RobotAsset;
use serde::de::{self, Visitor};
use serde::{Deserialize, Serialize};
use std::fmt;
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
    /// Fixed obstacles spawned as cuboid colliders.
    #[serde(default)]
    pub obstacles: Vec<SceneObstacleAsset>,
}

/// Fixed cuboid obstacle in a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneObstacleAsset {
    /// Entity name used when spawning.
    pub name: String,
    /// Obstacle center translation in meters.
    #[serde(default)]
    pub translation_m: [f64; 3],
    /// Cuboid half extents in meters.
    pub half_extents_m: [f64; 3],
    /// Physics body type (`fixed` or `dynamic`).
    #[serde(default)]
    pub body_type: ObstacleBodyType,
    /// Mass in kilograms when `body_type = "dynamic"`.
    #[serde(default = "default_obstacle_mass_kg")]
    pub mass_kg: f64,
}

/// Obstacle rigid-body type for scene spawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObstacleBodyType {
    /// Immovable scene geometry.
    #[default]
    Fixed,
    /// Simulated graspable / pushable object.
    Dynamic,
}

fn default_obstacle_mass_kg() -> f64 {
    0.08
}

/// World configuration stored in a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneWorldAsset {
    /// Gravity vector in meters per second squared.
    #[serde(default = "default_gravity")]
    pub gravity_m_s2: [f64; 3],
    /// Deterministic random seed.
    ///
    /// Bare integer seeds are supported for the signed 64-bit TOML portable
    /// range. Full `u64` values should be written as decimal or `0x` strings.
    #[serde(default, deserialize_with = "deserialize_scene_seed")]
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

fn deserialize_scene_seed<'de, D>(deserializer: D) -> Result<u64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    struct SceneSeedVisitor;

    impl Visitor<'_> for SceneSeedVisitor {
        type Value = u64;

        fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
            formatter.write_str("a non-negative u64 seed or a decimal/0x string")
        }

        fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            Ok(value)
        }

        fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            u64::try_from(value).map_err(|_| E::custom("scene seed must be non-negative"))
        }

        fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            parse_scene_seed_string(value).map_err(E::custom)
        }

        fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            self.visit_str(&value)
        }
    }

    deserializer.deserialize_any(SceneSeedVisitor)
}

fn parse_scene_seed_string(value: &str) -> Result<u64, String> {
    let compact = value.trim().replace('_', "");
    if compact.is_empty() {
        return Err("scene seed string must not be empty".to_string());
    }

    if let Some(hex) = compact
        .strip_prefix("0x")
        .or_else(|| compact.strip_prefix("0X"))
    {
        return u64::from_str_radix(hex, 16)
            .map_err(|_| "scene seed hex string must fit in u64".to_string());
    }

    compact
        .parse::<u64>()
        .map_err(|_| "scene seed decimal string must fit in u64".to_string())
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
        assert!(scene.obstacles.is_empty());
    }

    #[test]
    fn parses_missing_world_seed_as_zero() {
        let scene = parse_scene_asset("", Path::new("scene.toml")).unwrap();

        assert_eq!(scene.world.seed, 0);
    }

    #[test]
    fn parses_world_seed_from_decimal_string() {
        let scene = parse_scene_asset(
            r#"
[world]
seed = "18446744073709551615"
"#,
            Path::new("scene.toml"),
        )
        .unwrap();

        assert_eq!(scene.world.seed, u64::MAX);
    }

    #[test]
    fn parses_world_seed_from_hex_string() {
        let scene = parse_scene_asset(
            r#"
[world]
seed = "0xffff_ffff_ffff_ffff"
"#,
            Path::new("scene.toml"),
        )
        .unwrap();

        assert_eq!(scene.world.seed, u64::MAX);
    }

    #[test]
    fn rejects_negative_world_seed() {
        let result = parse_scene_asset(
            r#"
[world]
seed = -1
"#,
            Path::new("scene.toml"),
        );

        assert!(result.is_err());
    }

    #[test]
    fn parses_scene_with_obstacles() {
        let text = r#"
[[obstacles]]
name = "wall"
translation_m = [0.0, 1.0, 8.0]
half_extents_m = [8.0, 1.0, 0.25]
"#;
        let scene = parse_scene_asset(text, Path::new("scene.toml")).unwrap();
        assert_eq!(scene.obstacles.len(), 1);
        assert_eq!(scene.obstacles[0].name, "wall");
        assert_eq!(scene.obstacles[0].body_type, ObstacleBodyType::Fixed);
    }

    #[test]
    fn parses_dynamic_obstacle() {
        let text = r#"
[[obstacles]]
name = "cube"
translation_m = [0.5, 0.4, 0.0]
half_extents_m = [0.03, 0.03, 0.03]
body_type = "dynamic"
mass_kg = 0.05
"#;
        let scene = parse_scene_asset(text, Path::new("scene.toml")).unwrap();
        assert_eq!(scene.obstacles[0].body_type, ObstacleBodyType::Dynamic);
        assert!((scene.obstacles[0].mass_kg - 0.05).abs() < f64::EPSILON);
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
