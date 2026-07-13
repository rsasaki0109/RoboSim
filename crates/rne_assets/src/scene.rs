//! Scene asset schema and loading.

use crate::error::AssetError;
use crate::robot::RobotAsset;
use serde::de::{self, Visitor};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
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
    /// Cable and cloth entities simulated by the deformable solver.
    #[serde(default)]
    pub deformables: Vec<SceneDeformableAsset>,
    /// Named semantic task locations spawned without physics or visuals.
    #[serde(default)]
    pub task_markers: Vec<SceneTaskMarkerAsset>,
}

/// Scene-authored deformable material overrides.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneDeformableMaterialAsset {
    /// Particle collision radius in meters.
    #[serde(default = "default_deformable_collision_radius_m")]
    pub collision_radius_m: f64,
    /// Structural XPBD compliance in meters per newton.
    #[serde(default = "default_structural_compliance_m_n")]
    pub structural_compliance_m_n: f64,
    /// Shear XPBD compliance in meters per newton.
    #[serde(default = "default_shear_compliance_m_n")]
    pub shear_compliance_m_n: f64,
    /// Bending XPBD compliance in meters per newton.
    #[serde(default = "default_bending_compliance_m_n")]
    pub bending_compliance_m_n: f64,
    /// Fraction of velocity retained per second.
    #[serde(default = "default_velocity_retention_per_s")]
    pub velocity_retention_per_s: f64,
    /// Enable deterministic collision between non-adjacent particles.
    #[serde(default)]
    pub self_collision: bool,
}

impl Default for SceneDeformableMaterialAsset {
    fn default() -> Self {
        Self {
            collision_radius_m: default_deformable_collision_radius_m(),
            structural_compliance_m_n: default_structural_compliance_m_n(),
            shear_compliance_m_n: default_shear_compliance_m_n(),
            bending_compliance_m_n: default_bending_compliance_m_n(),
            velocity_retention_per_s: default_velocity_retention_per_s(),
            self_collision: false,
        }
    }
}

/// Cable or rectangular cloth declared in a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SceneDeformableAsset {
    /// One-dimensional particle cable.
    Cable {
        /// Entity name used for deterministic lookup.
        name: String,
        /// First endpoint in world-space meters.
        start_m: [f64; 3],
        /// Second endpoint in world-space meters.
        end_m: [f64; 3],
        /// Number of particles, at least two.
        particle_count: usize,
        /// Total cable mass in kilograms.
        total_mass_kg: f64,
        /// Pin the first endpoint.
        #[serde(default)]
        pin_start: bool,
        /// Pin the second endpoint.
        #[serde(default)]
        pin_end: bool,
        /// Physical material values.
        #[serde(default)]
        material: SceneDeformableMaterialAsset,
        /// Linear RGBA render color.
        #[serde(default = "default_cable_color")]
        color_rgba: [f32; 4],
    },
    /// Two-dimensional rectangular cloth grid.
    Cloth {
        /// Entity name used for deterministic lookup.
        name: String,
        /// Grid origin in world-space meters.
        origin_m: [f64; 3],
        /// Vector across all grid columns in meters.
        width_direction_m: [f64; 3],
        /// Vector across all grid rows in meters.
        height_direction_m: [f64; 3],
        /// Grid column count, at least two.
        columns: usize,
        /// Grid row count, at least two.
        rows: usize,
        /// Total cloth mass in kilograms.
        total_mass_kg: f64,
        /// Pin every particle in the first row.
        #[serde(default)]
        pin_top_edge: bool,
        /// Physical material values.
        #[serde(default)]
        material: SceneDeformableMaterialAsset,
        /// Linear RGBA render color.
        #[serde(default = "default_cloth_color")]
        color_rgba: [f32; 4],
    },
}

impl SceneDeformableAsset {
    /// Returns the scene-unique entity name.
    pub fn name(&self) -> &str {
        match self {
            Self::Cable { name, .. } | Self::Cloth { name, .. } => name,
        }
    }
}

fn default_deformable_collision_radius_m() -> f64 {
    0.01
}

fn default_structural_compliance_m_n() -> f64 {
    1.0e-7
}

fn default_shear_compliance_m_n() -> f64 {
    2.0e-7
}

fn default_bending_compliance_m_n() -> f64 {
    2.0e-4
}

fn default_velocity_retention_per_s() -> f64 {
    0.995
}

fn default_cable_color() -> [f32; 4] {
    [0.92, 0.45, 0.08, 1.0]
}

fn default_cloth_color() -> [f32; 4] {
    [0.08, 0.48, 0.82, 1.0]
}

/// Semantic task location stored in a scene asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneTaskMarkerAsset {
    /// Entity name used for deterministic lookup.
    pub name: String,
    /// Application-defined marker kind.
    pub kind: String,
    /// Marker center translation in meters.
    pub translation_m: [f64; 3],
    /// Success or interaction radius in meters.
    #[serde(default = "default_task_marker_radius_m")]
    pub radius_m: f64,
}

fn default_task_marker_radius_m() -> f64 {
    0.25
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
    /// Solid cylinder aligned with the local Z axis.
    Cylinder {
        /// Cylinder radius in meters.
        radius_m: f64,
        /// Full cylinder length in meters.
        length_m: f64,
        /// Linear RGBA color.
        #[serde(default = "default_object_color")]
        color_rgba: [f32; 4],
    },
    /// External STL or OBJ mesh resolved relative to the scene file.
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
    /// Capsule collider aligned with the local Y axis.
    Capsule {
        /// Half height of the cylindrical section in meters.
        half_height_m: f64,
        /// Capsule radius in meters.
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
    let scene = toml::from_str(text)
        .map_err(|error| AssetError::invalid(path.display().to_string(), error.to_string()))?;
    validate_scene_asset(&scene, path)?;
    Ok(scene)
}

fn validate_scene_asset(scene: &SceneAsset, path: &Path) -> Result<(), AssetError> {
    let invalid = |message: String| AssetError::invalid(path.display().to_string(), message);
    if !scene
        .world
        .gravity_m_s2
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(invalid("world gravity_m_s2 must be finite".into()));
    }

    let mut names = HashSet::new();
    for obstacle in &scene.obstacles {
        validate_name(&obstacle.name, "obstacle", &mut names).map_err(&invalid)?;
        validate_finite_vec(
            &obstacle.translation_m,
            &format!("obstacle `{}` translation_m", obstacle.name),
        )
        .map_err(&invalid)?;
        validate_positive_vec(
            &obstacle.half_extents_m,
            &format!("obstacle `{}` half_extents_m", obstacle.name),
        )
        .map_err(&invalid)?;
        if obstacle.body_type == ObstacleBodyType::Dynamic
            && (!obstacle.mass_kg.is_finite() || obstacle.mass_kg <= 0.0)
        {
            return Err(invalid(format!(
                "dynamic obstacle `{}` mass_kg must be finite and greater than zero",
                obstacle.name
            )));
        }
    }
    for object in &scene.objects {
        validate_name(&object.name, "object", &mut names).map_err(&invalid)?;
        validate_finite_vec(
            &object.translation_m,
            &format!("object `{}` translation_m", object.name),
        )
        .map_err(&invalid)?;
        validate_finite_vec(
            &object.rotation_rpy_rad,
            &format!("object `{}` rotation_rpy_rad", object.name),
        )
        .map_err(&invalid)?;
        if object.body_type == ObstacleBodyType::Dynamic
            && (!object.mass_kg.is_finite() || object.mass_kg <= 0.0)
        {
            return Err(invalid(format!(
                "dynamic object `{}` mass_kg must be finite and greater than zero",
                object.name
            )));
        }
        validate_object_shapes(object).map_err(&invalid)?;
    }
    for deformable in &scene.deformables {
        validate_name(deformable.name(), "deformable", &mut names).map_err(&invalid)?;
        validate_deformable(deformable).map_err(&invalid)?;
    }
    for marker in &scene.task_markers {
        validate_name(&marker.name, "task marker", &mut names).map_err(&invalid)?;
        if marker.kind.trim().is_empty() {
            return Err(invalid(format!(
                "task marker `{}` kind must not be empty",
                marker.name
            )));
        }
        validate_finite_vec(
            &marker.translation_m,
            &format!("task marker `{}` translation_m", marker.name),
        )
        .map_err(&invalid)?;
        if !marker.radius_m.is_finite() || marker.radius_m <= 0.0 {
            return Err(invalid(format!(
                "task marker `{}` radius_m must be finite and greater than zero",
                marker.name
            )));
        }
    }
    Ok(())
}

fn validate_deformable(deformable: &SceneDeformableAsset) -> Result<(), String> {
    use rne_deformable::{build_cable, build_cloth, CableSpec, ClothSpec};
    let material = deformable_material(deformable);
    let result = match deformable {
        SceneDeformableAsset::Cable {
            start_m,
            end_m,
            particle_count,
            total_mass_kg,
            pin_start,
            pin_end,
            ..
        } => build_cable(CableSpec {
            start_m: vec3_from_array(*start_m),
            end_m: vec3_from_array(*end_m),
            particle_count: *particle_count,
            total_mass_kg: *total_mass_kg,
            pin_start: *pin_start,
            pin_end: *pin_end,
            material,
        }),
        SceneDeformableAsset::Cloth {
            origin_m,
            width_direction_m,
            height_direction_m,
            columns,
            rows,
            total_mass_kg,
            pin_top_edge,
            ..
        } => build_cloth(ClothSpec {
            origin_m: vec3_from_array(*origin_m),
            width_direction_m: vec3_from_array(*width_direction_m),
            height_direction_m: vec3_from_array(*height_direction_m),
            columns: *columns,
            rows: *rows,
            total_mass_kg: *total_mass_kg,
            pin_top_edge: *pin_top_edge,
            material,
        }),
    };
    result
        .map(|_| ())
        .map_err(|error| format!("deformable `{}` is invalid: {error}", deformable.name()))
}

fn deformable_material(deformable: &SceneDeformableAsset) -> rne_deformable::DeformableMaterial {
    let asset = match deformable {
        SceneDeformableAsset::Cable { material, .. }
        | SceneDeformableAsset::Cloth { material, .. } => *material,
    };
    rne_deformable::DeformableMaterial {
        collision_radius_m: asset.collision_radius_m,
        structural_compliance_m_n: asset.structural_compliance_m_n,
        shear_compliance_m_n: asset.shear_compliance_m_n,
        bending_compliance_m_n: asset.bending_compliance_m_n,
        velocity_retention_per_s: asset.velocity_retention_per_s,
        self_collision: asset.self_collision,
    }
}

fn vec3_from_array(values: [f64; 3]) -> rne_math::Vec3 {
    rne_math::Vec3::new(values[0], values[1], values[2])
}

fn validate_name(name: &str, kind: &str, names: &mut HashSet<String>) -> Result<(), String> {
    if name.trim().is_empty() {
        return Err(format!("{kind} name must not be empty"));
    }
    if !names.insert(name.to_owned()) {
        return Err(format!("duplicate scene entity name `{name}`"));
    }
    Ok(())
}

fn validate_finite_vec<const N: usize>(values: &[f64; N], field: &str) -> Result<(), String> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(format!("{field} must contain only finite values"))
    }
}

fn validate_positive_vec<const N: usize>(values: &[f64; N], field: &str) -> Result<(), String> {
    if values.iter().all(|value| value.is_finite() && *value > 0.0) {
        Ok(())
    } else {
        Err(format!(
            "{field} must contain finite values greater than zero"
        ))
    }
}

fn validate_object_shapes(object: &SceneObjectAsset) -> Result<(), String> {
    let positive = |value: f64, field: &str| {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(format!(
                "object `{}` {field} must be finite and greater than zero",
                object.name
            ))
        }
    };
    match &object.visual {
        Some(SceneVisualAsset::Box { size_m, .. }) => {
            validate_positive_vec(size_m, &format!("object `{}` visual size_m", object.name))?
        }
        Some(SceneVisualAsset::Sphere { radius_m, .. }) => positive(*radius_m, "visual radius_m")?,
        Some(SceneVisualAsset::Cylinder {
            radius_m, length_m, ..
        }) => {
            positive(*radius_m, "visual radius_m")?;
            positive(*length_m, "visual length_m")?;
        }
        Some(SceneVisualAsset::Mesh { scale, .. }) => {
            validate_positive_vec(scale, &format!("object `{}` visual scale", object.name))?
        }
        None => {}
    }
    match object.collision {
        Some(SceneCollisionAsset::Box { size_m }) => validate_positive_vec(
            &size_m,
            &format!("object `{}` collision size_m", object.name),
        )?,
        Some(SceneCollisionAsset::Sphere { radius_m }) => positive(radius_m, "collision radius_m")?,
        Some(SceneCollisionAsset::Capsule {
            half_height_m,
            radius_m,
        }) => {
            positive(half_height_m, "collision half_height_m")?;
            positive(radius_m, "collision radius_m")?;
        }
        None => {}
    }
    Ok(())
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
        assert!(scene.deformables.is_empty());
        assert!(scene.task_markers.is_empty());
    }

    #[test]
    fn parses_and_validates_cable_and_cloth_assets() {
        let scene = parse_scene_asset(
            r#"
[[deformables]]
kind = "cable"
name = "tether"
start_m = [-0.5, 1.0, 0.0]
end_m = [0.5, 1.0, 0.0]
particle_count = 17
total_mass_kg = 0.2
pin_start = true

[[deformables]]
kind = "cloth"
name = "flag"
origin_m = [-0.4, 1.2, 0.0]
width_direction_m = [0.8, 0.0, 0.0]
height_direction_m = [0.0, -0.6, 0.0]
columns = 8
rows = 6
total_mass_kg = 0.15
pin_top_edge = true

[deformables.material]
self_collision = true
"#,
            Path::new("deformables.rne.scene.toml"),
        )
        .expect("valid deformable scene");
        assert_eq!(scene.deformables.len(), 2);
        assert_eq!(scene.deformables[0].name(), "tether");
        assert_eq!(scene.deformables[1].name(), "flag");
        let SceneDeformableAsset::Cloth { material, .. } = &scene.deformables[1] else {
            panic!("second deformable must be cloth");
        };
        assert!(material.self_collision);
    }

    #[test]
    fn rejects_invalid_or_duplicate_deformables() {
        let invalid = parse_scene_asset(
            r#"
[[deformables]]
kind = "cable"
name = "bad"
start_m = [0.0, 1.0, 0.0]
end_m = [0.0, 1.0, 0.0]
particle_count = 1
total_mass_kg = 0.0
"#,
            Path::new("invalid.rne.scene.toml"),
        )
        .expect_err("invalid cable must be rejected");
        assert!(invalid.to_string().contains("deformable `bad` is invalid"));

        let duplicate = parse_scene_asset(
            r#"
[[objects]]
name = "shared"

[[deformables]]
kind = "cable"
name = "shared"
start_m = [0.0, 1.0, 0.0]
end_m = [1.0, 1.0, 0.0]
particle_count = 3
total_mass_kg = 0.1
"#,
            Path::new("duplicate.rne.scene.toml"),
        )
        .expect_err("duplicate scene names must be rejected");
        assert!(duplicate
            .to_string()
            .contains("duplicate scene entity name `shared`"));
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
    fn parses_cylinder_visual_and_capsule_collision() {
        let scene = parse_scene_asset(
            r#"
[[objects]]
name = "safety_post"
visual = { shape = "cylinder", radius_m = 0.08, length_m = 0.9 }
collision = { shape = "capsule", half_height_m = 0.37, radius_m = 0.08 }
"#,
            Path::new("scene.toml"),
        )
        .unwrap();
        assert!(matches!(
            scene.objects[0].visual,
            Some(SceneVisualAsset::Cylinder {
                radius_m: 0.08,
                length_m: 0.9,
                ..
            })
        ));
        assert_eq!(
            scene.objects[0].collision,
            Some(SceneCollisionAsset::Capsule {
                half_height_m: 0.37,
                radius_m: 0.08,
            })
        );
    }

    #[test]
    fn rejects_non_positive_environment_dimensions() {
        let error = parse_scene_asset(
            r#"
[[objects]]
name = "bad_post"
visual = { shape = "cylinder", radius_m = 0.0, length_m = 0.9 }
"#,
            Path::new("scene.toml"),
        )
        .expect_err("zero-radius cylinder must be rejected");
        assert!(error.to_string().contains("visual radius_m"));

        let error = parse_scene_asset(
            r#"
[[task_markers]]
name = "bad_goal"
kind = "inspection"
translation_m = [0.0, 0.0, 0.0]
radius_m = -0.1
"#,
            Path::new("scene.toml"),
        )
        .expect_err("negative marker radius must be rejected");
        assert!(error.to_string().contains("radius_m"));
    }

    #[test]
    fn rejects_duplicate_scene_entity_names() {
        let error = parse_scene_asset(
            r#"
[[objects]]
name = "station"

[[task_markers]]
name = "station"
kind = "inspection"
translation_m = [0.0, 0.0, 0.0]
"#,
            Path::new("scene.toml"),
        )
        .expect_err("duplicate named entities must be rejected");
        assert!(error
            .to_string()
            .contains("duplicate scene entity name `station`"));
    }

    #[test]
    fn parses_named_task_marker() {
        let scene = parse_scene_asset(
            r#"
[[task_markers]]
name = "panel_a_check"
kind = "inspection"
translation_m = [0.8, 0.0, -0.3]
radius_m = 0.4
"#,
            Path::new("scene.toml"),
        )
        .unwrap();
        assert_eq!(scene.task_markers[0].name, "panel_a_check");
        assert_eq!(scene.task_markers[0].kind, "inspection");
        assert_eq!(scene.task_markers[0].radius_m, 0.4);
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
