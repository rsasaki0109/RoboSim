//! Asset validation, inspection, dependency tracking, and hot reload.

use crate::robot::{load_robot_asset, RobotAsset, RobotKind};
use crate::scene::{load_scene_asset, load_scene_robots, SceneAsset};
use crate::spawn::load_and_spawn_scene;
use crate::AssetError;
use rne_ecs::World;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

/// Fully loaded scene asset graph including referenced robots.
#[derive(Clone, Debug, PartialEq)]
pub struct SceneAssetBundle {
    /// Path to the scene file.
    pub scene_path: PathBuf,
    /// Parsed scene asset.
    pub scene: SceneAsset,
    /// Referenced robot assets keyed by absolute path.
    pub robots: Vec<(PathBuf, RobotAsset)>,
}

/// Result of validating a scene or robot asset file.
#[derive(Clone, Debug, PartialEq)]
pub enum ValidatedAsset {
    /// Validated scene bundle.
    Scene(SceneAssetBundle),
    /// Validated robot asset.
    Robot {
        /// Robot asset path.
        path: PathBuf,
        /// Parsed robot asset.
        asset: Box<RobotAsset>,
    },
}

/// Tracks modification times for a set of asset files.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AssetRevision {
    paths: Vec<PathBuf>,
    modified_at: Vec<SystemTime>,
}

/// Polls a scene asset graph and reloads when any dependency changes.
#[derive(Clone, Debug, PartialEq)]
pub struct AssetHotReloader {
    bundle: SceneAssetBundle,
    revision: AssetRevision,
}

/// Returns true when the path looks like a scene asset file.
pub fn is_scene_asset_path(path: &Path) -> bool {
    path.extension().is_some_and(|ext| ext == "toml")
        && path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.ends_with(".rne.scene.toml"))
}

/// Returns true when the path looks like a robot asset file.
pub fn is_robot_asset_path(path: &Path) -> bool {
    path.extension().is_some_and(|ext| ext == "toml")
        && path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.ends_with(".rne.robot.toml"))
}

/// Loads and validates a scene asset and all referenced robot files.
pub fn load_scene_bundle(scene_path: &Path) -> Result<SceneAssetBundle, AssetError> {
    let scene = load_scene_asset(scene_path)?;
    let robots = load_scene_robots(scene_path, &scene)?;
    for (robot_path, robot) in &robots {
        validate_robot_references(robot_path, robot)?;
    }

    Ok(SceneAssetBundle {
        scene_path: scene_path.to_path_buf(),
        scene,
        robots,
    })
}

/// Validates on-disk references for a robot asset.
pub fn validate_robot_references(path: &Path, asset: &RobotAsset) -> Result<(), AssetError> {
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    if asset.kind == RobotKind::Urdf {
        let urdf = asset.urdf.as_ref().ok_or_else(|| {
            AssetError::invalid(path.display().to_string(), "missing urdf section")
        })?;
        let urdf_path = urdf.resolve_path(base_dir);
        if !urdf_path.is_file() {
            return Err(AssetError::invalid(
                path.display().to_string(),
                format!("urdf file not found: {}", urdf_path.display()),
            ));
        }
    }
    if let Some(visuals) = &asset.visuals {
        let urdf_path = visuals.resolve_urdf_path(base_dir);
        if !urdf_path.is_file() {
            return Err(AssetError::invalid(
                path.display().to_string(),
                format!("visuals urdf file not found: {}", urdf_path.display()),
            ));
        }
    }
    Ok(())
}

/// Returns package roots used to resolve mesh URIs for a scene bundle.
pub fn mesh_package_roots(bundle: &SceneAssetBundle) -> Vec<PathBuf> {
    let mut roots = Vec::new();
    for (robot_path, robot) in &bundle.robots {
        let base_dir = robot_path.parent().unwrap_or_else(|| Path::new("."));
        if let Some(visuals) = &robot.visuals {
            let urdf_path = visuals.resolve_urdf_path(base_dir);
            if let Some(parent) = urdf_path.parent() {
                roots.push(parent.to_path_buf());
            }
        }
        if let Some(urdf) = &robot.urdf {
            let urdf_path = urdf.resolve_path(base_dir);
            if let Some(parent) = urdf_path.parent() {
                roots.push(parent.to_path_buf());
            }
        }
    }
    roots.sort();
    roots.dedup();
    roots
}

/// Validates a scene or robot asset path.
pub fn validate_asset(path: &Path) -> Result<ValidatedAsset, AssetError> {
    if is_scene_asset_path(path) {
        return Ok(ValidatedAsset::Scene(load_scene_bundle(path)?));
    }

    if is_robot_asset_path(path) {
        let asset = load_robot_asset(path)?;
        validate_robot_references(path, &asset)?;
        return Ok(ValidatedAsset::Robot {
            path: path.to_path_buf(),
            asset: Box::new(asset),
        });
    }

    Err(AssetError::invalid(
        path.display().to_string(),
        "expected .rne.scene.toml or .rne.robot.toml",
    ))
}

/// Builds a human-readable inspection report for an asset file.
pub fn inspect_asset(path: &Path) -> Result<String, AssetError> {
    match validate_asset(path)? {
        ValidatedAsset::Scene(bundle) => Ok(format_scene_report(&bundle)),
        ValidatedAsset::Robot { path, asset } => Ok(format_robot_report(&path, &asset)),
    }
}

/// Returns all on-disk files that should trigger a scene reload when changed.
pub fn scene_dependency_paths(bundle: &SceneAssetBundle) -> Vec<PathBuf> {
    let mut paths = vec![bundle.scene_path.clone()];
    for (robot_path, robot) in &bundle.robots {
        paths.push(robot_path.clone());
        let base_dir = robot_path.parent().unwrap_or_else(|| Path::new("."));
        if let Some(urdf) = &robot.urdf {
            paths.push(urdf.resolve_path(base_dir));
        }
        if let Some(visuals) = &robot.visuals {
            paths.push(visuals.resolve_urdf_path(base_dir));
        }
    }
    paths.sort();
    paths.dedup();
    paths
}

impl AssetRevision {
    /// Captures the current modification times for the given paths.
    pub fn from_paths(paths: &[PathBuf]) -> Result<Self, AssetError> {
        let modified_at = paths
            .iter()
            .map(|path| read_modified_at(path))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            paths: paths.to_vec(),
            modified_at,
        })
    }

    /// Returns true when any tracked file changed or disappeared.
    pub fn has_changed(&self) -> Result<bool, AssetError> {
        if self.paths.len() != self.modified_at.len() {
            return Ok(true);
        }

        for (path, previous) in self.paths.iter().zip(&self.modified_at) {
            match read_modified_at(path) {
                Ok(current) if current == *previous => {}
                Ok(_) => return Ok(true),
                Err(AssetError::Io { .. }) => return Ok(true),
                Err(error) => return Err(error),
            }
        }

        Ok(false)
    }
}

impl AssetHotReloader {
    /// Loads a scene bundle and begins tracking its dependency graph.
    pub fn load(scene_path: &Path) -> Result<Self, AssetError> {
        let bundle = load_scene_bundle(scene_path)?;
        let revision = AssetRevision::from_paths(&scene_dependency_paths(&bundle))?;
        Ok(Self { bundle, revision })
    }

    /// Returns the currently loaded scene bundle.
    pub fn bundle(&self) -> &SceneAssetBundle {
        &self.bundle
    }

    /// Reloads the scene graph when any dependency file changed.
    pub fn poll(&mut self) -> Result<bool, AssetError> {
        if !self.revision.has_changed()? {
            return Ok(false);
        }

        self.bundle = load_scene_bundle(&self.bundle.scene_path)?;
        self.revision = AssetRevision::from_paths(&scene_dependency_paths(&self.bundle))?;
        Ok(true)
    }
}

/// Validates that a scene can be spawned into an ECS world.
pub fn smoke_spawn_scene(scene_path: &Path) -> Result<usize, AssetError> {
    let mut world = World::new();
    let spawned = load_and_spawn_scene(&mut world, scene_path)?;
    Ok(spawned.robots.len())
}

fn format_scene_report(bundle: &SceneAssetBundle) -> String {
    let mut lines = vec![
        format!("scene: {}", bundle.scene_path.display()),
        format!(
            "world: seed={} gravity={:?}",
            bundle.scene.world.seed, bundle.scene.world.gravity_m_s2
        ),
        format!("ground: enabled={}", bundle.scene.ground.enabled),
        format!("robots: {}", bundle.robots.len()),
    ];

    for (path, robot) in &bundle.robots {
        lines.push(format!(
            "  - {} kind={:?} model={}",
            path.display(),
            robot.kind,
            robot.model_name
        ));
    }

    lines.push(format!(
        "dependencies: {}",
        scene_dependency_paths(bundle).len()
    ));
    lines.join("\n")
}

fn format_robot_report(path: &Path, asset: &RobotAsset) -> String {
    let mut lines = vec![
        format!("robot: {}", path.display()),
        format!("kind: {:?}", asset.kind),
        format!("model_name: {}", asset.model_name),
    ];

    if let Some(diff_drive) = &asset.diff_drive {
        lines.push(format!(
            "diff_drive: wheel_radius_m={} track_width_m={}",
            diff_drive.wheel_radius_m, diff_drive.track_width_m
        ));
    }

    if let Some(urdf) = &asset.urdf {
        let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
        lines.push(format!("urdf: {}", urdf.resolve_path(base_dir).display()));
    }

    if let Some(visuals) = &asset.visuals {
        let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
        lines.push(format!(
            "visuals: {}",
            visuals.resolve_urdf_path(base_dir).display()
        ));
    }

    lines.join("\n")
}

fn read_modified_at(path: &Path) -> Result<SystemTime, AssetError> {
    let metadata = std::fs::metadata(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    metadata.modified().map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::RobotKind;
    use std::fs;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn validate_scene_fixture_bundle() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/episode_diff_drive.rne.scene.toml");
        let bundle = load_scene_bundle(&scene_path).unwrap();
        assert_eq!(bundle.robots.len(), 1);
        assert_eq!(bundle.robots[0].1.kind, RobotKind::DiffDrive);
        assert!(scene_dependency_paths(&bundle).len() >= 2);
    }

    #[test]
    fn hot_reload_detects_scene_edit() {
        let temp_dir =
            std::env::temp_dir().join(format!("rne_assets_hot_reload_{}", std::process::id()));
        let _ = fs::remove_dir_all(&temp_dir);
        fs::create_dir_all(&temp_dir).unwrap();

        let scene_path = temp_dir.join("episode_diff_drive.rne.scene.toml");
        let robot_path = temp_dir.join("diff_drive.rne.robot.toml");
        fs::copy(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/diff_drive.rne.robot.toml"),
            &robot_path,
        )
        .unwrap();
        fs::write(
            &scene_path,
            r#"
[world]
seed = 1

[ground]
enabled = true

[[robots]]
path = "diff_drive.rne.robot.toml"
"#,
        )
        .unwrap();

        let mut reloader = AssetHotReloader::load(&scene_path).unwrap();
        assert!(!reloader.poll().unwrap());
        assert_eq!(reloader.bundle().scene.world.seed, 1);

        thread::sleep(Duration::from_millis(1100));
        fs::write(
            &scene_path,
            r#"
[world]
seed = 99

[ground]
enabled = true

[[robots]]
path = "diff_drive.rne.robot.toml"
"#,
        )
        .unwrap();

        assert!(reloader.poll().unwrap());
        assert_eq!(reloader.bundle().scene.world.seed, 99);

        let _ = fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn urdf_robot_requires_existing_file() {
        let robot_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/diff_drive_urdf.rne.robot.toml");
        let asset = load_robot_asset(&robot_path).unwrap();
        validate_robot_references(&robot_path, &asset).unwrap();
    }

    #[test]
    fn inspect_scene_report_lists_robots() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/episode_diff_drive.rne.scene.toml");
        let report = inspect_asset(&scene_path).unwrap();
        assert!(report.contains("robots: 1"));
        assert!(report.contains("dependencies:"));
    }
}
