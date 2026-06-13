//! Robot asset schema and loading.

use crate::error::AssetError;
use rne_math::Vec3;
use rne_robot::{DiffDriveConfig, DiffDriveDriveMode};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Supported robot asset kinds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RobotKind {
    /// Built-in differential drive robot.
    DiffDrive,
    /// Robot imported from a URDF file.
    Urdf,
}

/// Parsed `.rne.robot.toml` asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotAsset {
    /// Robot kind tag.
    pub kind: RobotKind,
    /// Model name used when spawning entities.
    pub model_name: String,
    /// Diff-drive parameters when [`Self::kind`] is [`RobotKind::DiffDrive`].
    pub diff_drive: Option<DiffDriveRobotAsset>,
    /// URDF parameters when [`Self::kind`] is [`RobotKind::Urdf`].
    pub urdf: Option<UrdfRobotAsset>,
    /// Optional URDF visuals for diff-drive robots.
    pub visuals: Option<VisualsRobotAsset>,
}

/// Optional URDF visuals attached to diff-drive link entities.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VisualsRobotAsset {
    /// Path to a URDF file containing visual geometry, relative to the robot asset directory unless absolute.
    pub urdf: String,
}

/// Diff-drive section of a robot asset file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveRobotAsset {
    /// Wheel radius in meters.
    #[serde(default = "default_wheel_radius_m")]
    pub wheel_radius_m: f64,
    /// Track width in meters.
    #[serde(default = "default_track_width_m")]
    pub track_width_m: f64,
    /// Base link half extents in meters.
    #[serde(default = "default_base_half_extents_m")]
    pub base_half_extents_m: [f64; 3],
    /// Maximum wheel velocity in radians per second.
    #[serde(default = "default_max_wheel_velocity_rad_s")]
    pub max_wheel_velocity_rad_s: f64,
    /// Initial base translation in meters.
    #[serde(default = "default_initial_translation_m")]
    pub initial_translation_m: [f64; 3],
    /// Wheel actuation model used when spawning the robot.
    #[serde(default)]
    pub drive_mode: DiffDriveDriveMode,
}

/// URDF section of a robot asset file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UrdfRobotAsset {
    /// Path to the URDF file, relative to the robot asset directory unless absolute.
    pub path: String,
}

impl DiffDriveRobotAsset {
    /// Converts this asset section into a [`DiffDriveConfig`].
    pub fn to_config(&self, model_name: &str) -> DiffDriveConfig {
        DiffDriveConfig {
            model_name: model_name.to_string(),
            wheel_radius_m: self.wheel_radius_m,
            track_width_m: self.track_width_m,
            base_half_extents_m: vec3_from_array(self.base_half_extents_m),
            max_wheel_velocity_rad_s: self.max_wheel_velocity_rad_s,
            initial_translation_m: vec3_from_array(self.initial_translation_m),
            drive_mode: self.drive_mode,
        }
    }
}

impl UrdfRobotAsset {
    /// Resolves the URDF path relative to a base directory.
    pub fn resolve_path(&self, base_dir: &Path) -> PathBuf {
        resolve_asset_path(&self.path, base_dir)
    }
}

impl VisualsRobotAsset {
    /// Resolves the visuals URDF path relative to a base directory.
    pub fn resolve_urdf_path(&self, base_dir: &Path) -> PathBuf {
        resolve_asset_path(&self.urdf, base_dir)
    }
}

fn resolve_asset_path(path: &str, base_dir: &Path) -> PathBuf {
    let path = Path::new(path);
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        base_dir.join(path)
    }
}

/// Loads a robot asset from a `.rne.robot.toml` file.
pub fn load_robot_asset(path: &Path) -> Result<RobotAsset, AssetError> {
    let text = std::fs::read_to_string(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_robot_asset(&text, path)
}

/// Parses robot asset TOML from memory.
pub fn parse_robot_asset(text: &str, path: &Path) -> Result<RobotAsset, AssetError> {
    let asset: RobotAsset = toml::from_str(text)
        .map_err(|error| AssetError::invalid(path.display().to_string(), error.to_string()))?;
    validate_robot_asset(&asset, path)
}

fn validate_robot_asset(asset: &RobotAsset, path: &Path) -> Result<RobotAsset, AssetError> {
    let path = path.display().to_string();
    match asset.kind {
        RobotKind::DiffDrive if asset.diff_drive.is_none() => {
            return Err(AssetError::invalid(
                path,
                "diff_drive section is required when kind = \"diff_drive\"",
            ));
        }
        RobotKind::Urdf if asset.urdf.is_none() => {
            return Err(AssetError::invalid(
                path,
                "urdf section is required when kind = \"urdf\"",
            ));
        }
        RobotKind::DiffDrive if asset.urdf.is_some() => {
            return Err(AssetError::invalid(
                path,
                "urdf section is not allowed when kind = \"diff_drive\"",
            ));
        }
        RobotKind::Urdf if asset.diff_drive.is_some() => {
            return Err(AssetError::invalid(
                path,
                "diff_drive section is not allowed when kind = \"urdf\"",
            ));
        }
        RobotKind::Urdf if asset.visuals.is_some() => {
            return Err(AssetError::invalid(
                path,
                "visuals section is not allowed when kind = \"urdf\"",
            ));
        }
        _ => {}
    }

    Ok(asset.clone())
}

fn default_wheel_radius_m() -> f64 {
    0.1
}

fn default_track_width_m() -> f64 {
    0.45
}

fn default_base_half_extents_m() -> [f64; 3] {
    [0.25, 0.15, 0.2]
}

fn default_max_wheel_velocity_rad_s() -> f64 {
    10.0
}

fn default_initial_translation_m() -> [f64; 3] {
    [0.0, 0.25, 0.0]
}

fn vec3_from_array(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

#[cfg(test)]
mod tests {
    use super::*;

    const DIFF_DRIVE: &str = include_str!("../tests/fixtures/diff_drive.rne.robot.toml");
    const URDF: &str = include_str!("../tests/fixtures/diff_drive_urdf.rne.robot.toml");

    #[test]
    fn parse_diff_drive_robot_asset() {
        let asset = parse_robot_asset(DIFF_DRIVE, Path::new("test.toml")).unwrap();
        assert_eq!(asset.kind, RobotKind::DiffDrive);
        assert_eq!(asset.model_name, "diff_drive");
        assert!(asset.diff_drive.is_some());
    }

    #[test]
    fn parse_urdf_robot_asset() {
        let asset = parse_robot_asset(URDF, Path::new("test.toml")).unwrap();
        assert_eq!(asset.kind, RobotKind::Urdf);
        assert_eq!(asset.urdf.unwrap().path, "minimal_diff_drive.urdf");
    }

    #[test]
    fn rejects_missing_diff_drive_section() {
        let text = r#"
kind = "diff_drive"
model_name = "diff_drive"
"#;
        let error = parse_robot_asset(text, Path::new("bad.toml")).unwrap_err();
        assert!(matches!(error, AssetError::Invalid { .. }));
    }
}
