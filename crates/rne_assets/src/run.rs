//! Versioned headless simulation run manifests.

use crate::error::AssetError;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Current `.rne.run.toml` schema version.
pub const RUN_MANIFEST_VERSION: u32 = 1;

/// Versioned configuration for one headless simulation run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunManifest {
    /// Schema version, currently [`RUN_MANIFEST_VERSION`].
    pub version: u32,
    /// Scene asset path, relative to this manifest unless absolute.
    pub scene: PathBuf,
    /// Optional replacement for the scene's world seed.
    #[serde(default)]
    pub seed: Option<u64>,
    /// Fixed-step simulation clock configuration.
    #[serde(default)]
    pub clock: RunClock,
    /// Controller configuration for the first runner boundary.
    #[serde(default)]
    pub controller: RunController,
    /// Output and replay checks for this run.
    #[serde(default)]
    pub output: RunOutput,
}

impl RunManifest {
    /// Resolves [`Self::scene`] against the manifest's parent directory.
    pub fn resolve_scene_path(&self, manifest_path: &Path) -> PathBuf {
        if self.scene.is_absolute() {
            self.scene.clone()
        } else {
            manifest_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&self.scene)
        }
    }
}

/// Fixed-step clock settings in a [`RunManifest`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunClock {
    /// Number of fixed simulation steps.
    #[serde(default = "default_run_steps")]
    pub steps: u64,
    /// Fixed simulation rate in hertz.
    #[serde(default = "default_run_hz")]
    pub hz: f64,
}

impl Default for RunClock {
    fn default() -> Self {
        Self {
            steps: default_run_steps(),
            hz: default_run_hz(),
        }
    }
}

/// Controller kinds supported by the version 1 runner boundary.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunControllerKind {
    /// Do not inject actuator commands.
    #[default]
    None,
    /// Apply one wheel velocity to every differential-drive robot.
    DifferentialDrive,
}

/// Controller settings in a [`RunManifest`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunController {
    /// Controller implementation selected by the runner.
    #[serde(default)]
    pub kind: RunControllerKind,
    /// Wheel velocity command used by [`RunControllerKind::DifferentialDrive`].
    #[serde(default)]
    pub wheel_velocity_rad_s: f64,
}

impl Default for RunController {
    fn default() -> Self {
        Self {
            kind: RunControllerKind::None,
            wheel_velocity_rad_s: 0.0,
        }
    }
}

/// Output settings in a [`RunManifest`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunOutput {
    /// Repeat the run and require the final report to match exactly.
    #[serde(default)]
    pub determinism_check: bool,
}

/// Loads and validates a `.rne.run.toml` manifest from disk.
pub fn load_run_manifest(path: &Path) -> Result<RunManifest, AssetError> {
    let text = fs::read_to_string(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_run_manifest(path, &text)
}

/// Parses and validates a run manifest from TOML text.
pub fn parse_run_manifest(path: &Path, text: &str) -> Result<RunManifest, AssetError> {
    let manifest: RunManifest = toml::from_str(text).map_err(|error| {
        AssetError::invalid(path.display().to_string(), format!("TOML: {error}"))
    })?;
    validate_run_manifest(path, manifest)
}

fn validate_run_manifest(path: &Path, manifest: RunManifest) -> Result<RunManifest, AssetError> {
    if manifest.version != RUN_MANIFEST_VERSION {
        return Err(AssetError::invalid(
            path.display().to_string(),
            format!(
                "unsupported run manifest version {}; expected {}",
                manifest.version, RUN_MANIFEST_VERSION
            ),
        ));
    }
    if manifest.scene.as_os_str().is_empty() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "scene path must not be empty",
        ));
    }
    if !manifest.clock.hz.is_finite() || manifest.clock.hz <= 0.0 {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "clock.hz must be finite and positive",
        ));
    }
    if !manifest.controller.wheel_velocity_rad_s.is_finite() {
        return Err(AssetError::invalid(
            path.display().to_string(),
            "controller.wheel_velocity_rad_s must be finite",
        ));
    }
    Ok(manifest)
}

fn default_run_steps() -> u64 {
    600
}

fn default_run_hz() -> f64 {
    60.0
}

#[cfg(test)]
mod tests {
    use super::{parse_run_manifest, RunControllerKind, RUN_MANIFEST_VERSION};
    use std::path::Path;

    #[test]
    fn parses_and_resolves_v1_manifest() {
        let manifest = parse_run_manifest(
            Path::new("assets/runs/example.rne.run.toml"),
            r#"
version = 1
scene = "../scenes/mesh_diff_drive.rne.scene.toml"
seed = 7

[clock]
steps = 120
hz = 30.0

[controller]
kind = "differential_drive"
wheel_velocity_rad_s = 4.0

[output]
determinism_check = true
"#,
        )
        .expect("manifest");
        assert_eq!(manifest.version, RUN_MANIFEST_VERSION);
        assert_eq!(manifest.seed, Some(7));
        assert_eq!(
            manifest.controller.kind,
            RunControllerKind::DifferentialDrive
        );
        assert_eq!(manifest.clock.steps, 120);
        assert_eq!(
            manifest.resolve_scene_path(Path::new("assets/runs/example.rne.run.toml")),
            Path::new("assets/runs/../scenes/mesh_diff_drive.rne.scene.toml")
        );
    }

    #[test]
    fn rejects_unknown_version_and_non_finite_clock() {
        let version_error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 2\nscene = \"scene.rne.scene.toml\"",
        )
        .expect_err("version must be rejected");
        assert!(version_error
            .to_string()
            .contains("unsupported run manifest version"));

        let clock_error = parse_run_manifest(
            Path::new("bad.rne.run.toml"),
            "version = 1\nscene = \"scene.rne.scene.toml\"\n[clock]\nhz = 0.0",
        )
        .expect_err("clock must be rejected");
        assert!(clock_error.to_string().contains("clock.hz"));
    }
}
