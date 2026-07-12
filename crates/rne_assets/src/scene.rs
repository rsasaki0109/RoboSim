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
    /// Named environment objects with independent visual and collision shapes.
    #[serde(default)]
    pub objects: Vec<SceneObjectAsset>,
}

/// Named environment object loaded from a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneObjectAsset {
    /// Entity name used for task lookup and spawning.
    pub name: String,
    /// Object origin translation in meters.
    #[serde(default)]
    pub translation_m: [f64; 3],
    /// Object roll, pitch, and yaw in radians.
    #[serde(default)]
    pub rotation_rpy_rad: [f64; 3],
    /// Physics body type (`fixed` or `dynamic`).
    #[serde(default)]
    pub body_type: ObstacleBodyType,
    /// Mass in kilograms for dynamic objects.
    #[serde(default = "default_obstacle_mass_kg")]
    pub mass_kg: f64,
    /// Coulomb friction coefficient.
    #[serde(default)]
    pub friction: Option<f32>,
    /// Coefficient of restitution.
    #[serde(default)]
    pub restitution: Option<f32>,
    /// Optional render shape. Objects may be collision-only.
    #[serde(default)]
    pub visual: Option<SceneVisualAsset>,
    /// Optional physics shape. Objects may be visual-only task markers.
    #[serde(default)]
    pub collision: Option<SceneCollisionAsset>,
}

/// Render shape for a scene environment object.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum SceneVisualAsset {
    /// Solid box with full extents in meters.
    Box {
        /// Full box size in meters.
        size_m: [f64; 3],
        /// Linear RGBA color.
        #[serde(default = "default_object_color")]
        color_rgba: [f32; 4],
    },
    /// Solid sphere.
    Sphere {
        /// Sphere radius in meters.
        radius_m: f64,
        /// Linear RGBA color.
        #[serde(default = "default_object_color")]
        color_rgba: [f32; 4],
    },
    /// External STL mesh resolved relative to the scene file.
    Mesh {
        /// Mesh path or package URI.
        path: String,
        /// Non-uniform mesh scale.
        #[serde(default = "default_object_scale")]
        scale: [f64; 3],
        /// Linear RGBA tint.
        #[serde(default = "default_object_color")]
        color_rgba: [f32; 4],
    },
}

/// Physics collision shape for a scene environment object.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum SceneCollisionAsset {
    /// Cuboid collider with full extents in meters.
    Box {
        /// Full collider size in meters.
        size_m: [f64; 3],
    },
    /// Sphere collider.
    Sphere {
        /// Sphere radius in meters.
        radius_m: f64,
    },
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
    /// Coulomb friction coefficient of the obstacle's collider surface.
    ///
    /// When unset, the spawned collider keeps the engine's default material
    /// friction (see [`rne_physics::PhysicsMaterial`]). Lets a scene author
    /// dial down friction on a specific object (e.g. to test that a
    /// friction-based grasp actually slips on a low-friction surface) without
    /// touching every other obstacle in the scene.
    #[serde(default)]
    pub friction: Option<f32>,
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

fn default_object_color() -> [f32; 4] {
    [0.55, 0.58, 0.62, 1.0]
}

fn default_object_scale() -> [f64; 3] {
    [1.0, 1.0, 1.0]
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

/// Parses robot assets referenced by a scene from in-memory TOML.
///
/// `robot_texts` must list `(resolved_robot_path, toml_text)` pairs matching the
/// scene's robot references after path resolution relative to `scene_path`.
pub fn parse_scene_robots(
    scene_path: &Path,
    scene: &SceneAsset,
    robot_texts: &[(PathBuf, &str)],
) -> Result<Vec<(PathBuf, RobotAsset)>, AssetError> {
    if robot_texts.len() != scene.robots.len() {
        return Err(AssetError::invalid(
            scene_path.display().to_string(),
            format!(
                "expected {} robot assets, got {}",
                scene.robots.len(),
                robot_texts.len()
            ),
        ));
    }

    let base_dir = scene_path.parent().unwrap_or_else(|| Path::new("."));
    scene
        .robots
        .iter()
        .enumerate()
        .map(|(index, reference)| {
            let robot_path = reference.resolve_path(base_dir);
            let (_, text) = robot_texts.get(index).ok_or_else(|| {
                AssetError::invalid(
                    scene_path.display().to_string(),
                    format!("missing robot asset text for index {index}"),
                )
            })?;
            let robot = crate::robot::parse_robot_asset(text, &robot_path)?;
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
        assert!(scene.objects.is_empty());
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
    fn parses_environment_objects_with_separate_visual_and_collision() {
        let text = r#"
[[objects]]
name = "inspection_panel"
translation_m = [1.0, 0.8, -0.3]
rotation_rpy_rad = [0.0, 0.2, 0.0]
visual = { shape = "mesh", path = "world/panel.stl", scale = [0.5, 0.5, 0.5], color_rgba = [0.1, 0.8, 0.4, 1.0] }
collision = { shape = "box", size_m = [0.2, 0.8, 0.2] }
friction = 0.7
restitution = 0.1
"#;
        let scene = parse_scene_asset(text, Path::new("scene.toml")).unwrap();
        assert_eq!(scene.objects.len(), 1);
        assert!(matches!(
            scene.objects[0].visual,
            Some(SceneVisualAsset::Mesh { .. })
        ));
        assert_eq!(
            scene.objects[0].collision,
            Some(SceneCollisionAsset::Box {
                size_m: [0.2, 0.8, 0.2]
            })
        );
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
