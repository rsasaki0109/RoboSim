//! Registered depth evidence for real-capture Gaussian splat fixtures.

use crate::{splat_proxy_depth_from_ply, GaussianSplatError};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{Camera, GaussianSplatEnvironment};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;

const DEPTH_PATCH_RADIUS_PX: i32 = 2;
const MIN_FINITE_DEPTH_FRACTION: f64 = 0.75;
const MIN_MATCHED_LANDMARKS: usize = 6;
const MAX_MEAN_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.25;
const MAX_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.75;
const MULTIVIEW_MIN_MATCHED_TRACKS: usize = 36;
const MULTIVIEW_MAX_MEAN_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.25;
const MULTIVIEW_MAX_ABSOLUTE_ERROR_SOURCE_UNITS: f64 = 0.75;
const MULTIVIEW_MAX_DEPTH_DELTA_MAE_SOURCE_UNITS: f64 = 0.25;
const MULTIVIEW_OCCLUSION_MARGIN_SOURCE_UNITS: f64 = 0.25;
const MULTIVIEW_MAX_FALSE_OCCLUSION_FRACTION: f64 = 0.10;

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

/// One complete deterministic proxy-depth frame in a multi-view audit.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatDepthFrameEvidence {
    /// Registered camera identity.
    pub camera_id: String,
    /// Width of the depth frame in pixels.
    pub width_px: u32,
    /// Height of the depth frame in pixels.
    pub height_px: u32,
    /// Stable hash of every depth sample.
    pub depth_frame_hash: u64,
    /// Number of pixels hit by a Gaussian proxy.
    pub finite_depth_pixels: usize,
    /// Fraction of pixels hit by a Gaussian proxy.
    pub finite_depth_fraction: f64,
}

/// One camera observation of a real multi-view COLMAP track.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatMultiviewViewEvidence {
    /// Registered camera identity.
    pub camera_id: String,
    /// Real-image observation coordinate in pixels.
    pub observed_pixel_uv: [f64; 2],
    /// COLMAP optical depth in reconstruction units.
    pub reference_depth_source_units: f64,
    /// Median RNE proxy depth near the registered observation.
    pub rne_proxy_depth_source_units: Option<f64>,
    /// Signed RNE-minus-reference depth residual in reconstruction units.
    pub signed_error_source_units: Option<f64>,
    /// Whether RNE places a proxy surface implausibly in front of a point that
    /// the real camera observed.
    pub false_occlusion: Option<bool>,
}

/// Cross-camera depth evidence for one real COLMAP feature track.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatMultiviewTrackEvidence {
    /// Stable COLMAP point identifier.
    pub colmap_point3d_id: u64,
    /// Per-camera depth observations.
    pub views: Vec<RegisteredSplatMultiviewViewEvidence>,
    /// Absolute disagreement between real and RNE depth change across cameras.
    pub depth_delta_error_source_units: Option<f64>,
}

/// Fixed acceptance limits for multi-view depth and visibility alignment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatMultiviewDepthTolerances {
    /// Minimum proxy-depth coverage required in each registered camera.
    pub min_finite_depth_fraction_per_camera: f64,
    /// Minimum tracks with proxy depth in both real camera observations.
    pub min_matched_tracks: usize,
    /// Maximum mean absolute depth residual across all matched views.
    pub max_mean_absolute_error_source_units: f64,
    /// Maximum absolute depth residual for any matched view.
    pub max_absolute_error_source_units: f64,
    /// Maximum mean absolute cross-camera depth-change disagreement.
    pub max_depth_delta_mae_source_units: f64,
    /// Depth lead required to classify a proxy as a false occluder.
    pub false_occlusion_margin_source_units: f64,
    /// Maximum fraction of matched views classified as false occlusions.
    pub max_false_occlusion_fraction: f64,
}

/// Content-bound multi-view depth and occlusion audit against real COLMAP tracks.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RegisteredSplatMultiviewDepthReport {
    /// Stable report kind.
    pub kind: &'static str,
    /// Report schema version.
    pub schema_version: u32,
    /// Real-capture environment identity.
    pub environment_id: String,
    /// SHA-256 of the selected real multi-view track artifact.
    pub track_fixture_sha256: String,
    /// SHA-256 of the Gaussian PLY consumed by RNE.
    pub ply_sha256: String,
    /// Stable proxy-depth algorithm identity.
    pub depth_algorithm_identity: &'static str,
    /// Complete depth-frame evidence for both registered cameras.
    pub frames: Vec<RegisteredSplatDepthFrameEvidence>,
    /// Per-track multi-view observations and residuals.
    pub tracks: Vec<RegisteredSplatMultiviewTrackEvidence>,
    /// Number of real tracks with RNE depth in both observations.
    pub matched_track_count: usize,
    /// Number of matched individual camera views.
    pub matched_view_count: usize,
    /// Mean absolute RNE depth residual in reconstruction units.
    pub mean_absolute_error_source_units: Option<f64>,
    /// Maximum absolute RNE depth residual in reconstruction units.
    pub max_absolute_error_source_units: Option<f64>,
    /// Mean cross-camera depth-change disagreement in reconstruction units.
    pub depth_delta_mae_source_units: Option<f64>,
    /// Number of real-visible observations hidden by an RNE proxy surface.
    pub false_occlusion_view_count: usize,
    /// Fraction of matched views hidden by an RNE proxy surface.
    pub false_occlusion_fraction: Option<f64>,
    /// Fixed registered limits used for the verdict.
    pub tolerances: RegisteredSplatMultiviewDepthTolerances,
    /// Whether the source-unit depth and visibility contract passed.
    pub passed: bool,
    /// Whether an independent physical anchor qualifies depths as metric.
    pub metric_qualified: bool,
    /// Explicit units and qualification note.
    pub units_note: String,
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
    let metric_qualified = false;
    let units_note = "Depth values remain COLMAP reconstruction units, not metres; metric qualification is owned by the independent physical-scale contract.";
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
        depth_algorithm_identity: "rne.gaussian_splat.alpha_proxy_depth.v2",
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

/// Renders deterministic proxy depth at two registered real cameras and audits
/// depth change plus false occlusion over shared real COLMAP tracks.
pub fn validate_registered_splat_multiview_depth(
    environment: &GaussianSplatEnvironment,
    track_fixture_path: &Path,
) -> Result<RegisteredSplatMultiviewDepthReport, GaussianSplatError> {
    let fixture_bytes = fs::read(track_fixture_path).map_err(|error| GaussianSplatError::Ply {
        path: track_fixture_path.display().to_string(),
        message: error.to_string(),
    })?;
    let fixture: MultiviewFixture =
        serde_json::from_slice(&fixture_bytes).map_err(|error| GaussianSplatError::Ply {
            path: track_fixture_path.display().to_string(),
            message: format!("invalid multi-view track fixture: {error}"),
        })?;
    if fixture.kind != "rne_registered_colmap_multiview_tracks"
        || fixture.schema_version != 1
        || fixture.status != "verified"
        || fixture.environment_id != environment.environment_id
        || fixture.track_count != fixture.tracks.len()
    {
        return Err(GaussianSplatError::Ply {
            path: track_fixture_path.display().to_string(),
            message: "multi-view track fixture identity or count drifted".into(),
        });
    }
    if fixture.cameras.len() != 2 {
        return Err(GaussianSplatError::Ply {
            path: track_fixture_path.display().to_string(),
            message: "multi-view depth requires exactly two registered cameras".into(),
        });
    }

    let mut depth_by_camera = BTreeMap::new();
    let mut frames = Vec::with_capacity(fixture.cameras.len());
    for registered in &fixture.cameras {
        let mut camera = Camera::new(
            registered.intrinsics.width_px,
            registered.intrinsics.height_px,
            registered.intrinsics.fov_y_rad,
        );
        camera.near_m = 0.01;
        camera.far_m = 100.0;
        let view = Transform3 {
            translation: Vec3::from_array(registered.rne_camera_to_world.translation_source_units),
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
        frames.push(RegisteredSplatDepthFrameEvidence {
            camera_id: registered.camera_id.clone(),
            width_px: depth.width,
            height_px: depth.height,
            depth_frame_hash: depth.hash_depth(),
            finite_depth_pixels,
            finite_depth_fraction: finite_depth_pixels as f64 / depth.depth_m.len() as f64,
        });
        if depth_by_camera
            .insert(registered.camera_id.clone(), depth)
            .is_some()
        {
            return Err(GaussianSplatError::Ply {
                path: track_fixture_path.display().to_string(),
                message: "duplicate registered multi-view camera".into(),
            });
        }
    }

    let mut absolute_errors = Vec::new();
    let mut delta_errors = Vec::new();
    let mut false_occlusion_view_count = 0;
    let mut matched_track_count = 0;
    let mut tracks = Vec::with_capacity(fixture.tracks.len());
    for track in &fixture.tracks {
        if track.views.len() != 2 {
            return Err(GaussianSplatError::Ply {
                path: track_fixture_path.display().to_string(),
                message: format!(
                    "multi-view track {} does not contain two observations",
                    track.colmap_point3d_id
                ),
            });
        }
        let mut views = Vec::with_capacity(2);
        for source_view in &track.views {
            let depth = depth_by_camera.get(&source_view.camera_id).ok_or_else(|| {
                GaussianSplatError::Ply {
                    path: track_fixture_path.display().to_string(),
                    message: format!("unknown multi-view camera {}", source_view.camera_id),
                }
            })?;
            let proxy = median_patch_depth(
                &depth.depth_m,
                depth.width,
                depth.height,
                source_view.observed_pixel_uv[0].round() as i32,
                source_view.observed_pixel_uv[1].round() as i32,
                100.0,
            );
            let signed_error = proxy.map(|value| value - source_view.reference_depth_source_units);
            let false_occlusion =
                signed_error.map(|value| value < -MULTIVIEW_OCCLUSION_MARGIN_SOURCE_UNITS);
            if let Some(error) = signed_error {
                absolute_errors.push(error.abs());
            }
            if false_occlusion == Some(true) {
                false_occlusion_view_count += 1;
            }
            views.push(RegisteredSplatMultiviewViewEvidence {
                camera_id: source_view.camera_id.clone(),
                observed_pixel_uv: source_view.observed_pixel_uv,
                reference_depth_source_units: source_view.reference_depth_source_units,
                rne_proxy_depth_source_units: proxy,
                signed_error_source_units: signed_error,
                false_occlusion,
            });
        }
        let depth_delta_error_source_units = match (
            views[0].rne_proxy_depth_source_units,
            views[1].rne_proxy_depth_source_units,
        ) {
            (Some(first), Some(second)) => {
                matched_track_count += 1;
                let reference_delta =
                    views[1].reference_depth_source_units - views[0].reference_depth_source_units;
                let proxy_delta = second - first;
                let error = (proxy_delta - reference_delta).abs();
                delta_errors.push(error);
                Some(error)
            }
            _ => None,
        };
        tracks.push(RegisteredSplatMultiviewTrackEvidence {
            colmap_point3d_id: track.colmap_point3d_id,
            views,
            depth_delta_error_source_units,
        });
    }

    let mean_absolute_error_source_units = mean(&absolute_errors);
    let max_absolute_error_source_units = absolute_errors.iter().copied().reduce(f64::max);
    let depth_delta_mae_source_units = mean(&delta_errors);
    let matched_view_count = absolute_errors.len();
    let false_occlusion_fraction = (matched_view_count > 0)
        .then(|| false_occlusion_view_count as f64 / matched_view_count as f64);
    let tolerances = RegisteredSplatMultiviewDepthTolerances {
        min_finite_depth_fraction_per_camera: MIN_FINITE_DEPTH_FRACTION,
        min_matched_tracks: MULTIVIEW_MIN_MATCHED_TRACKS,
        max_mean_absolute_error_source_units: MULTIVIEW_MAX_MEAN_ABSOLUTE_ERROR_SOURCE_UNITS,
        max_absolute_error_source_units: MULTIVIEW_MAX_ABSOLUTE_ERROR_SOURCE_UNITS,
        max_depth_delta_mae_source_units: MULTIVIEW_MAX_DEPTH_DELTA_MAE_SOURCE_UNITS,
        false_occlusion_margin_source_units: MULTIVIEW_OCCLUSION_MARGIN_SOURCE_UNITS,
        max_false_occlusion_fraction: MULTIVIEW_MAX_FALSE_OCCLUSION_FRACTION,
    };
    let passed = frames.iter().all(|frame| {
        frame.finite_depth_fraction >= tolerances.min_finite_depth_fraction_per_camera
    }) && matched_track_count >= tolerances.min_matched_tracks
        && mean_absolute_error_source_units
            .is_some_and(|value| value <= tolerances.max_mean_absolute_error_source_units)
        && max_absolute_error_source_units
            .is_some_and(|value| value <= tolerances.max_absolute_error_source_units)
        && depth_delta_mae_source_units
            .is_some_and(|value| value <= tolerances.max_depth_delta_mae_source_units)
        && false_occlusion_fraction
            .is_some_and(|value| value <= tolerances.max_false_occlusion_fraction);

    Ok(RegisteredSplatMultiviewDepthReport {
        kind: "rne_registered_splat_multiview_depth_report",
        schema_version: 1,
        environment_id: fixture.environment_id,
        track_fixture_sha256: format!("{:x}", Sha256::digest(fixture_bytes)),
        ply_sha256: sha256_file(&environment.ply_path)?,
        depth_algorithm_identity: "rne.gaussian_splat.alpha_proxy_depth.v2",
        frames,
        tracks,
        matched_track_count,
        matched_view_count,
        mean_absolute_error_source_units,
        max_absolute_error_source_units,
        depth_delta_mae_source_units,
        false_occlusion_view_count,
        false_occlusion_fraction,
        tolerances,
        passed,
        metric_qualified: false,
        units_note: fixture.units_note,
    })
}

fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
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
    camera_calibration: CameraCalibration,
    semantic_landmarks: Vec<SemanticLandmark>,
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

#[derive(Debug, Deserialize)]
struct MultiviewFixture {
    kind: String,
    schema_version: u32,
    environment_id: String,
    cameras: Vec<MultiviewCamera>,
    tracks: Vec<MultiviewTrack>,
    track_count: usize,
    status: String,
    units_note: String,
}

#[derive(Debug, Deserialize)]
struct MultiviewCamera {
    camera_id: String,
    intrinsics: RegisteredIntrinsics,
    rne_camera_to_world: MultiviewPose,
}

#[derive(Debug, Deserialize)]
struct MultiviewPose {
    translation_source_units: [f64; 3],
    rotation_xyzw: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct MultiviewTrack {
    colmap_point3d_id: u64,
    views: Vec<MultiviewTrackView>,
}

#[derive(Debug, Deserialize)]
struct MultiviewTrackView {
    camera_id: String,
    observed_pixel_uv: [f64; 2],
    reference_depth_source_units: f64,
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

    #[test]
    fn drjohnson_multiview_depth_is_deterministic_and_has_no_false_occlusion() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let manifest = repo
            .join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");
        let tracks = repo.join(
            "assets/environments/voxel51_drjohnson_3dgs/IMG_6292-IMG_6293.multiview-tracks.json",
        );
        let environment =
            rne_render::validate_gaussian_splat_manifest(&manifest).expect("Dr Johnson manifest");
        let first = validate_registered_splat_multiview_depth(&environment, &tracks)
            .expect("registered multi-view depth");
        let second = validate_registered_splat_multiview_depth(&environment, &tracks)
            .expect("registered multi-view depth replay");
        assert_eq!(first, second);
        assert!(first.passed);
        assert!(!first.metric_qualified);
        assert_eq!(first.frames.len(), 2);
        assert!(first.matched_track_count >= 36);
        assert_eq!(first.false_occlusion_view_count, 0);
        assert!(first.units_note.contains("not metres"));
    }
}
