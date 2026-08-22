//! Backend-neutral 3D Gaussian splat environment descriptors.
//!
//! Splat rendering lives in optional crates such as `rne_render_3dgs`. Core
//! simulation and contest scoring stay independent of these visual-only assets.

use crate::RenderScene;
use rne_math::{Quat, Transform3, Vec3};
use serde::{Deserialize, Serialize};
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
    /// Absolute or workspace-relative PLY path actually used for capture.
    pub ply_path: PathBuf,
    /// World transform applied before rendering the splat cloud.
    pub transform: Transform3,
    /// Renderer identity string for capture reports.
    pub renderer_identity: String,
    /// True when the resolved PLY is a CI / stand-in fixture rather than the
    /// preferred Kenkyugakuen (or other) scan.
    pub standin: bool,
    /// Optional authoring note from the manifest.
    pub coordinate_note: Option<String>,
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

/// Portable capture report for splat / hybrid viewer exports.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GaussianSplatCaptureReport {
    /// Report schema identity.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Environment id from the splat manifest.
    pub environment_id: String,
    /// Renderer identity recorded with the capture.
    pub renderer_identity: String,
    /// Absolute path of the PLY that was loaded.
    pub ply_path: String,
    /// Hex SHA-256 of the PLY bytes.
    pub ply_sha256: String,
    /// True when a stand-in fixture was used instead of the preferred scan.
    pub standin: bool,
    /// Optional RGBA pixel hash from a GPU capture.
    pub rgba_hash: Option<u64>,
    /// Optional PNG output path.
    pub png_path: Option<String>,
}

impl GaussianSplatCaptureReport {
    /// Builds a report for a resolved environment and optional capture hash.
    #[must_use]
    pub fn new(
        environment: &GaussianSplatEnvironment,
        ply_sha256: String,
        rgba_hash: Option<u64>,
        png_path: Option<PathBuf>,
    ) -> Self {
        Self {
            kind: "rne_gaussian_splat_capture_report".into(),
            schema_version: 1,
            environment_id: environment.environment_id.clone(),
            renderer_identity: environment.renderer_identity.clone(),
            ply_path: environment.ply_path.display().to_string(),
            ply_sha256,
            standin: environment.standin,
            rgba_hash,
            png_path: png_path.map(|path| path.display().to_string()),
        }
    }
}

/// Returns the bundled Tsukuba confirmation splat manifest path.
#[must_use]
pub fn tsukuba_confirmation_splat_manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/environments/tsukuba_confirmation.rne.splat.toml")
}

/// Returns the bundled Kenkyugakuen splat manifest path.
#[must_use]
pub fn tsukuba_kenkyugakuen_splat_manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/environments/tsukuba_kenkyugakuen.rne.splat.toml")
}

/// Loads a Gaussian splat environment manifest from disk.
pub fn load_gaussian_splat_manifest(
    path: &Path,
) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
    load_gaussian_splat_manifest_with_override(path, None)
}

/// Loads a manifest, optionally forcing a PLY path (for local scan swaps).
pub fn load_gaussian_splat_manifest_with_override(
    path: &Path,
    ply_override: Option<&Path>,
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
    raw.into_environment(path, ply_override)
}

/// Validates that the manifest and resolved PLY exist.
pub fn validate_gaussian_splat_manifest(
    path: &Path,
) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
    validate_gaussian_splat_manifest_with_override(path, None)
}

/// Validates a manifest with an optional PLY override path.
pub fn validate_gaussian_splat_manifest_with_override(
    path: &Path,
    ply_override: Option<&Path>,
) -> Result<GaussianSplatEnvironment, GaussianSplatError> {
    let environment = load_gaussian_splat_manifest_with_override(path, ply_override)?;
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
    preferred_ply_path: Option<String>,
    #[serde(default)]
    translation_m: [f64; 3],
    #[serde(default)]
    rotation_y_rad: f64,
    #[serde(default)]
    rotation_xyzw: Option<[f64; 4]>,
    #[serde(default = "default_scale")]
    scale: f64,
    #[serde(default = "default_renderer_identity")]
    renderer_identity: String,
    #[serde(default)]
    standin: bool,
    #[serde(default)]
    coordinate_note: Option<String>,
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
        ply_override: Option<&Path>,
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
        if !self.rotation_y_rad.is_finite() {
            return Err(GaussianSplatError::Invalid {
                path,
                message: "rotation_y_rad must be finite".into(),
            });
        }
        let rotation = if let Some([x, y, z, w]) = self.rotation_xyzw {
            if self.rotation_y_rad != 0.0 {
                return Err(GaussianSplatError::Invalid {
                    path,
                    message: "rotation_xyzw and non-zero rotation_y_rad are mutually exclusive"
                        .into(),
                });
            }
            let rotation = Quat::from_xyzw(x, y, z, w);
            if !rotation.is_finite() || rotation.length_squared() <= f64::EPSILON {
                return Err(GaussianSplatError::Invalid {
                    path,
                    message: "rotation_xyzw must be finite and non-zero".into(),
                });
            }
            rotation.normalize()
        } else {
            Quat::from_rotation_y(self.rotation_y_rad)
        };
        let parent = manifest_path.parent();
        let resolve = |relative: &str| -> PathBuf {
            parent.map_or_else(|| PathBuf::from(relative), |parent| parent.join(relative))
        };

        let (ply_path, standin) = if let Some(override_path) = ply_override {
            (override_path.to_path_buf(), false)
        } else if let Some(preferred) = self.preferred_ply_path.as_deref() {
            let preferred_path = resolve(preferred);
            if preferred_path.is_file() {
                (preferred_path, false)
            } else {
                (resolve(&self.ply_path), self.standin)
            }
        } else {
            (resolve(&self.ply_path), self.standin)
        };

        Ok(GaussianSplatEnvironment {
            environment_id: self.environment_id,
            ply_path,
            transform: Transform3 {
                translation: Vec3::from_array(self.translation_m),
                rotation,
                scale: Vec3::splat(self.scale),
            },
            renderer_identity: self.renderer_identity,
            standin,
            coordinate_note: self.coordinate_note,
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
        assert!(!environment.standin);
    }

    #[test]
    fn kenkyugakuen_manifest_falls_back_to_fixture_standin() {
        let manifest = tsukuba_kenkyugakuen_splat_manifest_path();
        let environment = validate_gaussian_splat_manifest(&manifest).expect("kenkyugakuen");
        assert_eq!(environment.environment_id, "tsukuba.kenkyugakuen.v1");
        assert!(environment.ply_path.is_file());
        assert!(environment.standin);
        assert!(environment
            .ply_path
            .ends_with("tsukuba_confirmation_fixture.ply"));
    }

    #[test]
    fn ply_override_clears_standin_flag() {
        let manifest = tsukuba_kenkyugakuen_splat_manifest_path();
        let fixture = validate_gaussian_splat_manifest(&tsukuba_confirmation_splat_manifest_path())
            .expect("fixture")
            .ply_path;
        let environment = validate_gaussian_splat_manifest_with_override(&manifest, Some(&fixture))
            .expect("override");
        assert_eq!(environment.environment_id, "tsukuba.kenkyugakuen.v1");
        assert!(!environment.standin);
        assert_eq!(environment.ply_path, fixture);
    }

    #[test]
    fn manifest_quaternion_rotation_is_normalized_and_applied() {
        let raw = RawGaussianSplatManifest {
            kind: "rne_gaussian_splat_environment".into(),
            schema_version: 1,
            environment_id: "test.quaternion".into(),
            ply_path: "fixture.ply".into(),
            preferred_ply_path: None,
            translation_m: [0.0; 3],
            rotation_y_rad: 0.0,
            rotation_xyzw: Some([2.0, 0.0, 0.0, 2.0]),
            scale: 1.0,
            renderer_identity: GAUSSIAN_SPLAT_RENDERER_ID_V1.into(),
            standin: false,
            coordinate_note: None,
        };
        let environment = raw
            .into_environment(Path::new("fixture.rne.splat.toml"), None)
            .expect("quaternion manifest");
        assert!((environment.transform.rotation.length() - 1.0).abs() < 1.0e-12);
        let rotated = environment.transform.rotation * Vec3::Y;
        assert!((rotated - Vec3::Z).length() < 1.0e-12);
    }
}
