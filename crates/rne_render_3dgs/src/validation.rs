//! Registered depth evidence for real-capture Gaussian splat fixtures.

use crate::{splat_proxy_depth_from_ply, GaussianSplatError};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{Camera, GaussianSplatEnvironment};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::cmp::Ordering;
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;

const DEPTH_PATCH_RADIUS_PX: i32 = 1;
const MIN_FINITE_DEPTH_FRACTION: f64 = 0.75;
const MIN_MATCHED_LANDMARKS: usize = 6;
const MAX_MEAN_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.25;
const MAX_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.75;

/// One sparse, semantically identified depth comparison.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatDepthLandmark {
    /// Stable semantic landmark identifier from the validation fixture.
    pub landmark_id: String,
    /// Semantic class assigned to the landmark.
    pub semantic_class: String,
    /// Registered horizontal image coordinate in pixels.
    pub pixel_u_px: f64,
    /// Registered vertical image coordinate in pixels.
    pub pixel_v_px: f64,
    /// Sparse COLMAP optical depth in reconstruction units.
    pub reference_depth_source_units: f64,
    /// Median finite RNE proxy depth inside the registered pixel patch.
    pub rne_proxy_depth_source_units: Option<f64>,
    /// Absolute depth disagreement in reconstruction units.
    pub absolute_error_source_units: Option<f64>,
}

/// Content-bound sparse-depth validation for one registered real camera.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatDepthReport {
    /// Stable report kind.
    pub kind: &'static str,
    /// Report schema version.
    pub schema_version: u32,
    /// Real-capture environment identity.
    pub environment_id: String,
    /// Registered camera identity.
    pub camera_id: String,
    /// SHA-256 of the exact registered camera calibration object.
    pub camera_calibration_sha256: String,
    /// Stable identity of the RNE proxy-depth algorithm.
    pub depth_algorithm_identity: &'static str,
    /// SHA-256 of the Gaussian PLY consumed by RNE.
    pub ply_sha256: String,
    /// Width of the generated depth frame in pixels.
    pub width_px: u32,
    /// Height of the generated depth frame in pixels.
    pub height_px: u32,
    /// Stable hash of the complete RNE proxy-depth frame.
    pub depth_frame_hash: u64,
    /// Pixels containing a Gaussian proxy hit before the far plane.
    pub finite_depth_pixels: usize,
    /// Fraction of pixels containing a Gaussian proxy hit.
    pub finite_depth_fraction: f64,
    /// Radius of the robust landmark sampling patch in pixels.
    pub patch_radius_px: i32,
    /// Per-landmark sparse depth comparisons.
    pub landmarks: Vec<RegisteredSplatDepthLandmark>,
    /// Number of landmarks with an RNE proxy-depth observation.
    pub matched_landmarks: usize,
    /// Mean absolute matched-landmark error in reconstruction units.
    pub mean_absolute_error_source_units: Option<f64>,
    /// Maximum absolute matched-landmark error in reconstruction units.
    pub max_absolute_error_source_units: Option<f64>,
    /// Fixed acceptance limits for sparse proxy-depth alignment.
    pub tolerances: RegisteredSplatDepthTolerances,
    /// Whether the source-unit sparse-depth contract passed.
    pub passed: bool,
    /// Whether an independent physical scale anchor qualifies these values as metric.
    pub metric_qualified: bool,
    /// Explicit units and qualification note.
    pub units_note: String,
}

/// Fixed limits for registered sparse 3DGS proxy-depth evidence.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatDepthTolerances {
    /// Minimum fraction of the calibrated camera covered by proxy depth.
    pub min_finite_depth_fraction: f64,
    /// Minimum number of semantically registered sparse matches.
    pub min_matched_landmarks: usize,
    /// Maximum mean absolute sparse-depth error in reconstruction units.
    pub max_mean_absolute_error_source_units: f64,
    /// Maximum individual sparse-depth error in reconstruction units.
    pub max_absolute_error_source_units: f64,
}

/// Re-renders deterministic proxy depth at a registered real camera and compares
/// it with semantically identified sparse COLMAP depths.
pub fn validate_registered_splat_depth(
    environment: &GaussianSplatEnvironment,
    fixture_path: &Path,
    camera_id: &str,
) -> Result<RegisteredSplatDepthReport, GaussianSplatError> {
    let fixture_bytes = fs::read(fixture_path).map_err(|error| GaussianSplatError::Ply {
        path: fixture_path.display().to_string(),
        message: error.to_string(),
    })?;
    let fixture: ValidationFixture =
        serde_json::from_slice(&fixture_bytes).map_err(|error| GaussianSplatError::Ply {
            path: fixture_path.display().to_string(),
            message: format!("invalid validation fixture: {error}"),
        })?;
    if fixture.environment_id != environment.environment_id {
        return Err(GaussianSplatError::Ply {
            path: fixture_path.display().to_string(),
            message: "validation fixture environment identity drifted".into(),
        });
    }
    let registered = fixture
        .camera_calibration
        .cameras
        .iter()
        .find(|camera| camera.camera_id == camera_id)
        .ok_or_else(|| GaussianSplatError::Ply {
            path: fixture_path.display().to_string(),
            message: format!("registered camera {camera_id} is absent"),
        })?;
    let mut camera = Camera::new(
        registered.intrinsics.width_px,
        registered.intrinsics.height_px,
        registered.intrinsics.fov_y_rad,
    );
    camera.near_m = 0.01;
    camera.far_m = 100.0;
    let view = Transform3 {
        translation: Vec3::from_array(registered.rne_camera_to_world.translation_m),
        rotation: Quat::from_xyzw(
            registered.rne_camera_to_world.rotation_xyzw[0],
            registered.rne_camera_to_world.rotation_xyzw[1],
            registered.rne_camera_to_world.rotation_xyzw[2],
            registered.rne_camera_to_world.rotation_xyzw[3],
        ),
        scale: Vec3::ONE,
    };
    let depth = splat_proxy_depth_from_ply(
        &environment.ply_path,
        &camera,
        &view,
        &environment.transform,
    )?;
    let finite_depth_pixels = depth
        .depth_m
        .iter()
        .filter(|value| value.is_finite() && **value < camera.far_m as f32 * 0.999)
        .count();

    let landmarks = fixture
        .semantic_landmarks
        .iter()
        .filter(|landmark| landmark.camera_id == camera_id)
        .map(|landmark| {
            let proxy = median_patch_depth(
                &depth.depth_m,
                depth.width,
                depth.height,
                landmark.observed_pixel_uv[0].round() as i32,
                landmark.observed_pixel_uv[1].round() as i32,
                camera.far_m as f32,
            );
            RegisteredSplatDepthLandmark {
                landmark_id: landmark.landmark_id.clone(),
                semantic_class: landmark.semantic_class.clone(),
                pixel_u_px: landmark.observed_pixel_uv[0],
                pixel_v_px: landmark.observed_pixel_uv[1],
                reference_depth_source_units: landmark.optical_depth_source_units,
                rne_proxy_depth_source_units: proxy,
                absolute_error_source_units: proxy
                    .map(|value| (value - landmark.optical_depth_source_units).abs()),
            }
        })
        .collect::<Vec<_>>();
    let errors = landmarks
        .iter()
        .filter_map(|landmark| landmark.absolute_error_source_units)
        .collect::<Vec<_>>();
    let mean_absolute_error_source_units =
        (!errors.is_empty()).then(|| errors.iter().sum::<f64>() / errors.len() as f64);
    let max_absolute_error_source_units = errors.iter().copied().reduce(f64::max);
    let tolerances = RegisteredSplatDepthTolerances {
        min_finite_depth_fraction: MIN_FINITE_DEPTH_FRACTION,
        min_matched_landmarks: MIN_MATCHED_LANDMARKS,
        max_mean_absolute_error_source_units: MAX_MEAN_ABSOLUTE_ERROR_SOURCE_UNITS,
        max_absolute_error_source_units: MAX_ABSOLUTE_ERROR_SOURCE_UNITS,
    };
    let finite_depth_fraction = finite_depth_pixels as f64 / depth.depth_m.len() as f64;
    let passed = finite_depth_fraction >= tolerances.min_finite_depth_fraction
        && errors.len() >= tolerances.min_matched_landmarks
        && mean_absolute_error_source_units
            .is_some_and(|value| value <= tolerances.max_mean_absolute_error_source_units)
        && max_absolute_error_source_units
            .is_some_and(|value| value <= tolerances.max_absolute_error_source_units);
    let metric_qualified = fixture.metric_scale.status == "verified"
        && fixture.metric_scale.source_units_to_m.is_some();
    let units_note = if metric_qualified {
        "Depth values are scaled by the independently verified physical anchor."
    } else {
        "Depth values remain COLMAP reconstruction units, not metres, until an independent physical scale anchor is retained."
    };
    let camera_calibration_bytes =
        serde_json::to_vec(registered).map_err(|error| GaussianSplatError::Ply {
            path: fixture_path.display().to_string(),
            message: format!("failed to encode registered camera: {error}"),
        })?;

    Ok(RegisteredSplatDepthReport {
        kind: "rne_registered_splat_sparse_depth_report",
        schema_version: 1,
        environment_id: fixture.environment_id,
        camera_id: camera_id.to_owned(),
        camera_calibration_sha256: format!("{:x}", Sha256::digest(camera_calibration_bytes)),
        depth_algorithm_identity: "rne.gaussian_splat.proxy_depth.v1",
        ply_sha256: sha256_file(&environment.ply_path)?,
        width_px: depth.width,
        height_px: depth.height,
        depth_frame_hash: depth.hash_depth(),
        finite_depth_pixels,
        finite_depth_fraction,
        patch_radius_px: DEPTH_PATCH_RADIUS_PX,
        matched_landmarks: errors.len(),
        mean_absolute_error_source_units,
        max_absolute_error_source_units,
        tolerances,
        passed,
        landmarks,
        metric_qualified,
        units_note: units_note.into(),
    })
}

fn median_patch_depth(
    values: &[f32],
    width: u32,
    height: u32,
    center_x: i32,
    center_y: i32,
    far: f32,
) -> Option<f64> {
    let mut hits = Vec::new();
    for y in center_y - DEPTH_PATCH_RADIUS_PX..=center_y + DEPTH_PATCH_RADIUS_PX {
        for x in center_x - DEPTH_PATCH_RADIUS_PX..=center_x + DEPTH_PATCH_RADIUS_PX {
            if x < 0 || y < 0 || x >= width as i32 || y >= height as i32 {
                continue;
            }
            let value = values[(y as u32 * width + x as u32) as usize];
            if value.is_finite() && value < far * 0.999 {
                hits.push(value);
            }
        }
    }
    hits.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    hits.get(hits.len() / 2).map(|value| f64::from(*value))
}

fn sha256_file(path: &Path) -> Result<String, GaussianSplatError> {
    let mut file = File::open(path).map_err(|error| GaussianSplatError::Ply {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let count = file
            .read(&mut buffer)
            .map_err(|error| GaussianSplatError::Ply {
                path: path.display().to_string(),
                message: error.to_string(),
            })?;
        if count == 0 {
            break;
        }
        digest.update(&buffer[..count]);
    }
    Ok(format!("{:x}", digest.finalize()))
}

#[derive(Debug, Deserialize)]
struct ValidationFixture {
    environment_id: String,
    metric_scale: MetricScale,
    camera_calibration: CameraCalibration,
    semantic_landmarks: Vec<SemanticLandmark>,
}

#[derive(Debug, Deserialize)]
struct MetricScale {
    status: String,
    source_units_to_m: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct CameraCalibration {
    cameras: Vec<RegisteredCamera>,
}

#[derive(Debug, Deserialize, Serialize)]
struct RegisteredCamera {
    camera_id: String,
    intrinsics: RegisteredIntrinsics,
    rne_camera_to_world: RegisteredPose,
}

#[derive(Debug, Deserialize, Serialize)]
struct RegisteredIntrinsics {
    width_px: u32,
    height_px: u32,
    fov_y_rad: f64,
}

#[derive(Debug, Deserialize, Serialize)]
struct RegisteredPose {
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct SemanticLandmark {
    landmark_id: String,
    semantic_class: String,
    camera_id: String,
    observed_pixel_uv: [f64; 2],
    optical_depth_source_units: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drjohnson_sparse_depth_is_deterministic_and_explicitly_non_metric() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let manifest = repo
            .join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");
        let environment =
            rne_render::validate_gaussian_splat_manifest(&manifest).expect("Dr Johnson manifest");
        let fixture = environment
            .validation_fixture_path
            .as_ref()
            .expect("validation fixture");
        let first = validate_registered_splat_depth(&environment, fixture, "colmap.IMG_6293.jpg")
            .expect("registered sparse depth");
        let second = validate_registered_splat_depth(&environment, fixture, "colmap.IMG_6293.jpg")
            .expect("registered sparse depth replay");
        assert_eq!(first, second);
        assert!(!first.metric_qualified);
        assert_eq!(first.landmarks.len(), 6);
        assert_eq!(first.matched_landmarks, 6);
        assert!(first.passed);
        assert!(first.finite_depth_fraction > 0.25);
        assert!(first.units_note.contains("not metres"));
    }
}
