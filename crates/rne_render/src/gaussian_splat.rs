//! Backend-neutral 3D Gaussian splat environment descriptors.
//!
//! Splat rendering lives in optional crates such as `rne_render_3dgs`. Core
//! simulation and contest scoring stay independent of these visual-only assets.

use crate::RenderScene;
use rne_math::{Quat, Transform3, Vec3};
use serde::Deserialize;
use std::fs;
use std::path::{Path, PathBuf};
use thiserror::Error;

/// Portable renderer identity recorded in dataset manifests.
pub const GAUSSIAN_SPLAT_RENDERER_ID_V1: &str = "rne.gaussian_splat.v1";

/// Error while loading or validating a Gaussian splat environment manifest.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GaussianSplatError {
    /// The manifest file could not be read.
    #[error("failed to read Gaussian splat manifest {path}: {message}")]
    Io {
        /// Manifest path.
        path: String,
        /// OS error text.
        message: String,
    },
    /// TOML parsing failed.
    #[error("failed to parse Gaussian splat manifest {path}: {message}")]
    Parse {
        /// Manifest path.
        path: String,
        /// Parser error text.
        message: String,
    },
    /// The manifest failed semantic validation.
    #[error("invalid Gaussian splat manifest {path}: {message}")]
    Invalid {
        /// Manifest path.
        path: String,
        /// Validation message.
        message: String,
    },
    /// The referenced PLY file is missing.
    #[error("Gaussian splat PLY missing at {path}")]
    MissingPly {
        /// Resolved PLY path.
        path: String,
    },
}

/// Loaded Gaussian splat environment referenced by a committed manifest.
#[derive(Clone, Debug, PartialEq)]
pub struct GaussianSplatEnvironment {
    /// Stable environment identifier from the manifest.
    pub environment_id: String,
    /// Absolute or workspace-relative PLY path.
    pub ply_path: PathBuf,
    /// World transform applied before rendering the splat cloud.
    pub transform: Transform3,
    /// Renderer identity string for capture reports.
    pub renderer_identity: String,
}

/// Hybrid scene: optional splat background plus mesh foreground.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HybridRenderScene {
    /// Visual-only radiance-field background.
    pub background: Option<GaussianSplatEnvironment>,
    /// Mesh and primitive foreground drawn on top of the splat pass.
    pub foreground: RenderScene,
}

impl HybridRenderScene {
    /// Creates a foreground-only hybrid scene.
    #[must_use]
    pub fn foreground_only(foreground: RenderScene) -> Self {
        Self {
            background: None,
            foreground,
        }
    }

    /// Creates a hybrid scene with both layers.
    #[must_use]
    pub fn new(background: GaussianSplatEnvironment, foreground: RenderScene) -> Self {
        Self {
            background: Some(background),
            foreground,
        }
    }
}

/// Returns the bundled Tsukuba confirmation splat manifest path.
#[must_use]
pub fn tsukuba_confirmation_splat_manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/environments/tsukuba_confirmation.rne.splat.toml")
}

/// Loads a Gaussian splat environment manifest from disk.
pub fn load_gaussian_splat_manifest(
    path: &Path,
) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
    let contents = fs::read_to_string(path).map_err(|error| GaussianSplatError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let raw: RawGaussianSplatManifest =
        toml::from_str(&contents).map_err(|error| GaussianSplatError::Parse {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    raw.into_environment(path)
}

/// Validates that the manifest and referenced PLY exist.
pub fn validate_gaussian_splat_manifest(
    path: &Path,
) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
    let environment = load_gaussian_splat_manifest(path)?;
    if !environment.ply_path.is_file() {
        return Err(GaussianSplatError::MissingPly {
            path: environment.ply_path.display().to_string(),
        });
    }
    Ok(environment)
}

#[derive(Debug, Deserialize)]
struct RawGaussianSplatManifest {
    kind: String,
    schema_version: u32,
    environment_id: String,
    ply_path: String,
    #[serde(default)]
    translation_m: [f64; 3],
    #[serde(default)]
    rotation_y_rad: f64,
    #[serde(default = "default_scale")]
    scale: f64,
    #[serde(default = "default_renderer_identity")]
    renderer_identity: String,
}

fn default_scale() -> f64 {
    1.0
}

fn default_renderer_identity() -> String {
    GAUSSIAN_SPLAT_RENDERER_ID_V1.to_string()
}

impl RawGaussianSplatManifest {
    fn into_environment(
        self,
        manifest_path: &Path,
    ) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
        let path = manifest_path.display().to_string();
        if self.kind != "rne_gaussian_splat_environment" {
            return Err(GaussianSplatError::Invalid {
                path,
                message: format!("unsupported kind `{}`", self.kind),
            });
        }
        if self.schema_version != 1 {
            return Err(GaussianSplatError::Invalid {
                path,
                message: format!("unsupported schema_version {}", self.schema_version),
            });
        }
        if self.environment_id.trim().is_empty() {
            return Err(GaussianSplatError::Invalid {
                path,
                message: "environment_id must be non-empty".into(),
            });
        }
        if !(self.scale.is_finite() && self.scale > 0.0) {
            return Err(GaussianSplatError::Invalid {
                path,
                message: "scale must be finite and positive".into(),
            });
        }
        let ply_path = manifest_path.parent().map_or_else(
            || PathBuf::from(&self.ply_path),
            |parent| parent.join(&self.ply_path),
        );
        Ok(GaussianSplatEnvironment {
            environment_id: self.environment_id,
            ply_path,
            transform: Transform3 {
                translation: Vec3::from_array(self.translation_m),
                rotation: Quat::from_rotation_y(self.rotation_y_rad),
                scale: Vec3::splat(self.scale),
            },
            renderer_identity: self.renderer_identity,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tsukuba_confirmation_fixture_manifest_loads() {
        let manifest = tsukuba_confirmation_splat_manifest_path();
        let environment = validate_gaussian_splat_manifest(&manifest).expect("fixture manifest");
        assert_eq!(
            environment.environment_id,
            "tsukuba.confirmation.fixture.v1"
        );
        assert!(environment.ply_path.is_file());
    }
}
