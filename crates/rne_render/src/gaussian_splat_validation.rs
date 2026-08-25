//! Fail-closed validation for real-capture 3DGS robot fixtures.

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};
use thiserror::Error;

macro_rules! ensure {
    ($condition:expr, $($argument:tt)*) => {
        if !$condition {
            return Err(GaussianSplatValidationError::Invalid(format!($($argument)*)));
        }
    };
}

const FIXTURE_KIND: &str = "rne_gaussian_splat_validation_fixture";
const FIXTURE_SCHEMA_VERSION: u64 = 1;
const EXPECTED_CONTRACTS: [&str; 6] = [
    "floor_world_alignment",
    "camera_intrinsics_extrinsics",
    "semantic_landmark_reprojection",
    "collision_semantic_alignment",
    "independent_metric_scale_anchor",
    "real_sim_observation_comparison",
];

/// Result of rehashing and semantically auditing one 3DGS validation fixture.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GaussianSplatValidationAudit {
    /// Stable environment identifier.
    pub environment_id: String,
    /// SHA-256 of the complete fixture JSON bytes.
    pub fixture_sha256: String,
    /// Whether every required contract passed.
    pub qualifying: bool,
    /// Required contracts that passed.
    pub passed_contracts: Vec<String>,
    /// Required contracts that are explicitly missing evidence.
    pub missing_contracts: Vec<String>,
    /// Required contracts that have evidence but failed their tolerance.
    pub failed_contracts: Vec<String>,
}

/// Pixel and structure metrics for one registered real-versus-RNE observation.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct GaussianSplatObservationMetrics {
    /// Mean absolute RGB channel error in 8-bit intensity units.
    pub rgb_mae_8bit: f64,
    /// Root-mean-square RGB channel error in 8-bit intensity units.
    pub rgb_rmse_8bit: f64,
    /// Raw RGB peak signal-to-noise ratio in decibels.
    pub rgb_psnr_db: f64,
    /// Pearson correlation between reference and rendered luminance.
    pub luminance_pearson: f64,
    /// Pearson correlation between central-difference gradient magnitudes.
    pub gradient_magnitude_pearson: f64,
}

/// Registered acceptance limits for real-versus-RNE 3DGS observations.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct GaussianSplatObservationTolerances {
    /// Minimum accepted raw RGB PSNR.
    pub min_rgb_psnr_db: f64,
    /// Minimum accepted luminance correlation.
    pub min_luminance_pearson: f64,
    /// Minimum accepted gradient-magnitude correlation.
    pub min_gradient_magnitude_pearson: f64,
}

/// Returns the fixed registered observation limits used by fixture validation.
#[must_use]
pub const fn gaussian_splat_observation_tolerances() -> GaussianSplatObservationTolerances {
    GaussianSplatObservationTolerances {
        min_rgb_psnr_db: 12.0,
        min_luminance_pearson: 0.90,
        min_gradient_magnitude_pearson: 0.65,
    }
}

/// Error while loading or semantically verifying a 3DGS validation fixture.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GaussianSplatValidationError {
    /// A fixture or referenced artifact could not be read.
    #[error("failed to read 3DGS validation artifact {path}: {message}")]
    Io {
        /// Path that could not be read.
        path: String,
        /// Underlying I/O error.
        message: String,
    },
    /// The fixture JSON could not be decoded.
    #[error("failed to parse 3DGS validation fixture {path}: {message}")]
    Parse {
        /// Fixture path.
        path: String,
        /// JSON decoder error.
        message: String,
    },
    /// Typed or semantic validation failed.
    #[error("invalid 3DGS validation fixture: {0}")]
    Invalid(String),
    /// A referenced artifact digest, size, or image extent did not match.
    #[error("3DGS validation artifact mismatch for {label}: {message}")]
    ArtifactMismatch {
        /// Artifact field being checked.
        label: String,
        /// Mismatch details.
        message: String,
    },
    /// The fixture is structurally valid but not qualifying evidence.
    #[error("3DGS fixture is not qualifying; missing={missing:?} failed={failed:?}")]
    NotQualifying {
        /// Required contracts without evidence.
        missing: Vec<String>,
        /// Required contracts whose evidence failed.
        failed: Vec<String>,
    },
}

/// Rehashes and semantically audits a real-capture 3DGS validation fixture.
pub fn audit_gaussian_splat_validation_fixture(
    path: &Path,
) -> Result<GaussianSplatValidationAudit, GaussianSplatValidationError> {
    let bytes = fs::read(path).map_err(|error| GaussianSplatValidationError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let value: Value =
        serde_json::from_slice(&bytes).map_err(|error| GaussianSplatValidationError::Parse {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    let root = object(&value, "fixture")?;
    ensure_exact_keys(
        root,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "renderer_identity",
            "status",
            "qualifying",
            "provenance",
            "source_to_world",
            "metric_scale",
            "floor_alignment",
            "camera_calibration",
            "semantic_landmarks",
            "collision_semantic_alignment",
            "real_sim_observation_comparison",
            "contracts",
        ],
        "fixture",
    )?;
    ensure!(
        string(root, "kind")? == FIXTURE_KIND,
        "unexpected fixture kind"
    );
    ensure!(
        unsigned(root, "schema_version")? == FIXTURE_SCHEMA_VERSION,
        "unsupported fixture schema_version"
    );
    let environment_id = nonempty_string(root, "environment_id")?.to_string();
    let renderer_identity = nonempty_string(root, "renderer_identity")?;
    let declared_status = string(root, "status")?;
    let declared_qualifying = boolean(root, "qualifying")?;
    let parent = path.parent().unwrap_or_else(|| Path::new("."));

    validate_provenance(object(field(root, "provenance")?, "provenance")?, parent)?;
    validate_source_to_world(object(field(root, "source_to_world")?, "source_to_world")?)?;
    let floor_passed = validate_floor(object(field(root, "floor_alignment")?, "floor_alignment")?)?;
    let camera_passed = validate_camera_calibration(
        object(field(root, "camera_calibration")?, "camera_calibration")?,
        parent,
    )?;
    validate_semantic_landmarks(
        array(field(root, "semantic_landmarks")?, "semantic_landmarks")?,
        object(field(root, "camera_calibration")?, "camera_calibration")?,
    )?;
    let collision_passed = validate_collision_alignment(object(
        field(root, "collision_semantic_alignment")?,
        "collision_semantic_alignment",
    )?)?;
    let metric = object(field(root, "metric_scale")?, "metric_scale")?;
    let metric_passed = string(metric, "status")? == "verified"
        && !field(metric, "independent_physical_anchor")?.is_null();
    let observation_passed = validate_observation_comparison(
        object(
            field(root, "real_sim_observation_comparison")?,
            "real_sim_observation_comparison",
        )?,
        parent,
        &environment_id,
        renderer_identity,
    )?;

    let expected = BTreeMap::from([
        ("floor_world_alignment", pass_or_fail(floor_passed)),
        ("camera_intrinsics_extrinsics", pass_or_fail(camera_passed)),
        (
            "semantic_landmark_reprojection",
            pass_or_fail(camera_passed),
        ),
        (
            "collision_semantic_alignment",
            pass_or_fail(collision_passed),
        ),
        (
            "independent_metric_scale_anchor",
            if metric_passed { "passed" } else { "missing" },
        ),
        (
            "real_sim_observation_comparison",
            pass_or_fail(observation_passed),
        ),
    ]);
    let contracts = validate_contracts(array(field(root, "contracts")?, "contracts")?)?;
    ensure!(
        contracts == expected,
        "contract verdicts do not match retained evidence"
    );

    let mut passed_contracts = Vec::new();
    let mut missing_contracts = Vec::new();
    let mut failed_contracts = Vec::new();
    for id in EXPECTED_CONTRACTS {
        match expected[id] {
            "passed" => passed_contracts.push(id.to_string()),
            "missing" => missing_contracts.push(id.to_string()),
            "failed" => failed_contracts.push(id.to_string()),
            _ => unreachable!(),
        }
    }
    let qualifying = missing_contracts.is_empty() && failed_contracts.is_empty();
    ensure!(
        declared_qualifying == qualifying,
        "qualifying flag does not match contract verdicts"
    );
    ensure!(
        declared_status == if qualifying { "passed" } else { "incomplete" },
        "fixture status does not match contract verdicts"
    );
    Ok(GaussianSplatValidationAudit {
        environment_id,
        fixture_sha256: sha256_hex(&bytes),
        qualifying,
        passed_contracts,
        missing_contracts,
        failed_contracts,
    })
}

/// Requires a fixture to pass every metric, camera, semantic, collision, and
/// real/sim observation contract.
pub fn require_qualifying_gaussian_splat_fixture(
    path: &Path,
) -> Result<GaussianSplatValidationAudit, GaussianSplatValidationError> {
    let audit = audit_gaussian_splat_validation_fixture(path)?;
    if !audit.qualifying {
        return Err(GaussianSplatValidationError::NotQualifying {
            missing: audit.missing_contracts.clone(),
            failed: audit.failed_contracts.clone(),
        });
    }
    Ok(audit)
}

/// Recomputes registered RGB and structural metrics from retained image files.
pub fn compare_registered_gaussian_splat_observations(
    reference_path: &Path,
    rendered_path: &Path,
) -> Result<GaussianSplatObservationMetrics, GaussianSplatValidationError> {
    let reference = image::open(reference_path)
        .map_err(|error| GaussianSplatValidationError::ArtifactMismatch {
            label: "registered reference image".into(),
            message: error.to_string(),
        })?
        .to_rgb8();
    let rendered = image::open(rendered_path)
        .map_err(|error| GaussianSplatValidationError::ArtifactMismatch {
            label: "registered RNE render".into(),
            message: error.to_string(),
        })?
        .to_rgb8();
    ensure!(
        rendered.dimensions() == reference.dimensions(),
        "registered observation extents differ"
    );
    compare_observations(
        reference.as_raw(),
        rendered.as_raw(),
        reference.width() as usize,
        reference.height() as usize,
    )
}

fn validate_provenance(
    provenance: &Map<String, Value>,
    parent: &Path,
) -> Result<(), GaussianSplatValidationError> {
    let url = nonempty_string(provenance, "source_archive_url")?;
    ensure!(
        url.starts_with("https://"),
        "source archive URL must use HTTPS"
    );
    ensure!(
        unsigned(provenance, "source_archive_size_bytes")? > 0,
        "empty source archive"
    );
    for field_name in [
        "source_archive_sha256",
        "colmap_cameras_sha256",
        "colmap_images_sha256",
        "colmap_points3d_sha256",
    ] {
        ensure_sha256(string(provenance, field_name)?, field_name)?;
    }
    verify_artifact(
        object(field(provenance, "splat_manifest")?, "splat_manifest")?,
        parent,
        "provenance.splat_manifest",
    )?;
    verify_artifact(
        object(field(provenance, "splat_ply")?, "splat_ply")?,
        parent,
        "provenance.splat_ply",
    )?;
    Ok(())
}

fn validate_source_to_world(
    value: &Map<String, Value>,
) -> Result<(), GaussianSplatValidationError> {
    finite_vector(
        field(value, "translation_m")?,
        3,
        "source_to_world.translation_m",
    )?;
    finite_vector(
        field(value, "rotation_xyzw")?,
        4,
        "source_to_world.rotation_xyzw",
    )?;
    let scale = finite(value, "scale")?;
    ensure!(
        scale.is_sign_positive(),
        "source_to_world.scale must be positive"
    );
    let _ = nonempty_string(value, "source_units")?;
    let _ = nonempty_string(value, "world_units_claim")?;
    Ok(())
}

fn validate_floor(value: &Map<String, Value>) -> Result<bool, GaussianSplatValidationError> {
    let count = unsigned(value, "registered_candidate_count")?;
    let inliers = unsigned(value, "dominant_plane_inlier_count")?;
    ensure!(
        count > 0 && inliers > 0 && inliers <= count,
        "invalid floor inlier counts"
    );
    let height = finite(value, "dominant_plane_world_y_claimed_m")?;
    let rmse = finite(value, "dominant_plane_rmse_claimed_m")?;
    let tolerance = finite(value, "world_y_tolerance_claimed_m")?;
    ensure!(
        rmse >= 0.0 && tolerance > 0.0,
        "invalid floor tolerance metrics"
    );
    let passed = height.abs() <= tolerance && rmse <= tolerance;
    ensure!(
        string(value, "status")? == pass_status(passed),
        "floor status does not match metrics"
    );
    Ok(passed)
}

fn validate_camera_calibration(
    value: &Map<String, Value>,
    parent: &Path,
) -> Result<bool, GaussianSplatValidationError> {
    let cameras = array(field(value, "cameras")?, "camera_calibration.cameras")?;
    ensure!(!cameras.is_empty(), "camera calibration has no cameras");
    for (index, camera_value) in cameras.iter().enumerate() {
        let camera = object(camera_value, "camera")?;
        let intrinsics = object(field(camera, "intrinsics")?, "intrinsics")?;
        let width = unsigned(intrinsics, "width_px")?;
        let height = unsigned(intrinsics, "height_px")?;
        ensure!(width > 0 && height > 0, "camera extent must be positive");
        for name in ["fx_px", "fy_px"] {
            let focal_length = finite(intrinsics, name)?;
            ensure!(
                focal_length.is_sign_positive(),
                "camera focal length must be positive"
            );
        }
        for name in ["cx_px", "cy_px", "fov_y_rad"] {
            let _ = finite(intrinsics, name)?;
        }
        let reference = object(field(camera, "reference_image")?, "reference_image")?;
        let path = verify_artifact(
            reference,
            parent,
            &format!("camera[{index}].reference_image"),
        )?;
        let (actual_width, actual_height) = image::image_dimensions(&path).map_err(|error| {
            GaussianSplatValidationError::ArtifactMismatch {
                label: format!("camera[{index}].reference_image"),
                message: error.to_string(),
            }
        })?;
        ensure!(
            u64::from(actual_width) == width && u64::from(actual_height) == height,
            "reference image extent differs from camera intrinsics"
        );
        let pose = object(field(camera, "rne_camera_to_world")?, "rne_camera_to_world")?;
        finite_vector(field(pose, "translation_m")?, 3, "camera.translation_m")?;
        finite_vector(field(pose, "rotation_xyzw")?, 4, "camera.rotation_xyzw")?;
    }
    let count = unsigned(value, "semantic_landmark_count")?;
    ensure!(count > 0, "camera calibration has no semantic landmarks");
    let rmse = finite(value, "reprojection_rmse_px")?;
    let maximum = finite(value, "reprojection_max_error_px")?;
    let tolerance = finite(value, "tolerance_px")?;
    ensure!(
        rmse >= 0.0 && maximum >= rmse && tolerance > 0.0,
        "invalid reprojection metrics"
    );
    let passed = maximum <= tolerance;
    ensure!(
        string(value, "status")?
            == if passed {
                "verified_colmap_reprojection"
            } else {
                "failed"
            },
        "camera status does not match reprojection metrics"
    );
    Ok(passed)
}

fn validate_semantic_landmarks(
    landmarks: &[Value],
    camera: &Map<String, Value>,
) -> Result<(), GaussianSplatValidationError> {
    ensure!(
        landmarks.len() as u64 == unsigned(camera, "semantic_landmark_count")?,
        "semantic landmark count drifted"
    );
    let tolerance = finite(camera, "tolerance_px")?;
    let mut ids = BTreeSet::new();
    for landmark_value in landmarks {
        let landmark = object(landmark_value, "semantic landmark")?;
        ensure!(
            ids.insert(nonempty_string(landmark, "landmark_id")?),
            "duplicate landmark id"
        );
        let _ = nonempty_string(landmark, "semantic_class")?;
        finite_vector(
            field(landmark, "observed_pixel_uv")?,
            2,
            "observed_pixel_uv",
        )?;
        finite_vector(field(landmark, "source_position")?, 3, "source_position")?;
        finite_vector(field(landmark, "world_position_m")?, 3, "world_position_m")?;
        let reprojection = finite(landmark, "reprojection_error_px")?;
        ensure!(
            reprojection >= 0.0 && reprojection <= tolerance,
            "landmark reprojection exceeded tolerance"
        );
    }
    Ok(())
}

fn validate_collision_alignment(
    value: &Map<String, Value>,
) -> Result<bool, GaussianSplatValidationError> {
    let proxy = object(field(value, "proxy")?, "collision proxy")?;
    finite_vector(field(proxy, "center_world_m")?, 3, "proxy.center_world_m")?;
    let extents = finite_vector(field(proxy, "half_extents_m")?, 3, "proxy.half_extents_m")?;
    ensure!(
        extents.iter().all(|extent| *extent > 0.0),
        "proxy extents must be positive"
    );
    finite_vector(
        field(value, "projected_top_center_pixel_uv")?,
        2,
        "projected_top_center_pixel_uv",
    )?;
    let inside = boolean(value, "top_center_inside_expected_semantic_polygon")?;
    ensure!(
        string(value, "status")?
            == if inside {
                "verified_reference_projection"
            } else {
                "failed"
            },
        "collision alignment status drifted"
    );
    Ok(inside)
}

fn validate_observation_comparison(
    value: &Map<String, Value>,
    parent: &Path,
    environment_id: &str,
    renderer_identity: &str,
) -> Result<bool, GaussianSplatValidationError> {
    ensure_exact_keys(
        value,
        &[
            "status",
            "reference_camera_id",
            "reference_image",
            "rne_render",
            "report",
            "metrics",
            "tolerances",
            "photometric_note",
        ],
        "real_sim_observation_comparison",
    )?;
    let camera_id = nonempty_string(value, "reference_camera_id")?;
    let _ = nonempty_string(value, "photometric_note")?;
    let reference_artifact = object(field(value, "reference_image")?, "reference_image")?;
    let render_artifact = object(field(value, "rne_render")?, "rne_render")?;
    let reference_path = verify_artifact(
        reference_artifact,
        parent,
        "real_sim_observation_comparison.reference_image",
    )?;
    let render_path = verify_artifact(
        render_artifact,
        parent,
        "real_sim_observation_comparison.rne_render",
    )?;
    let report_path = verify_artifact(
        object(field(value, "report")?, "observation report")?,
        parent,
        "real_sim_observation_comparison.report",
    )?;
    let report_bytes =
        fs::read(&report_path).map_err(|error| GaussianSplatValidationError::Io {
            path: report_path.display().to_string(),
            message: error.to_string(),
        })?;
    let report_value: Value = serde_json::from_slice(&report_bytes).map_err(|error| {
        GaussianSplatValidationError::Parse {
            path: report_path.display().to_string(),
            message: error.to_string(),
        }
    })?;
    let report = object(&report_value, "observation report")?;
    ensure_exact_keys(
        report,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "camera_id",
            "renderer_identity",
            "gpu_adapter",
            "reference_image",
            "rne_render",
            "intrinsics",
            "metrics",
            "tolerances",
            "status",
            "photometric_note",
        ],
        "observation report",
    )?;
    ensure!(
        string(report, "kind")? == "rne_registered_real_sim_observation"
            && unsigned(report, "schema_version")? == 1,
        "unsupported observation report identity"
    );
    ensure!(
        string(report, "environment_id")? == environment_id
            && string(report, "camera_id")? == camera_id
            && string(report, "renderer_identity")? == renderer_identity,
        "observation report identity drifted"
    );
    let gpu_adapter = object(field(report, "gpu_adapter")?, "gpu_adapter")?;
    ensure_exact_keys(
        gpu_adapter,
        &[
            "name",
            "vendor",
            "device",
            "device_type",
            "driver",
            "driver_info",
            "backend",
        ],
        "gpu_adapter",
    )?;
    let _ = nonempty_string(gpu_adapter, "name")?;
    let _ = unsigned(gpu_adapter, "vendor")?;
    let _ = unsigned(gpu_adapter, "device")?;
    let _ = nonempty_string(gpu_adapter, "device_type")?;
    let _ = nonempty_string(gpu_adapter, "driver")?;
    let _ = string(gpu_adapter, "driver_info")?;
    let _ = nonempty_string(gpu_adapter, "backend")?;
    ensure!(
        field(report, "reference_image")? == field(value, "reference_image")?
            && field(report, "rne_render")? == field(value, "rne_render")?
            && field(report, "metrics")? == field(value, "metrics")?
            && field(report, "tolerances")? == field(value, "tolerances")?
            && field(report, "status")? == field(value, "status")?
            && field(report, "photometric_note")? == field(value, "photometric_note")?,
        "observation fixture and report disagree"
    );
    let intrinsics = object(field(report, "intrinsics")?, "observation intrinsics")?;
    let width = unsigned(intrinsics, "width_px")?;
    let height = unsigned(intrinsics, "height_px")?;
    for name in ["fx_px", "fy_px", "cx_px", "cy_px", "fov_y_rad"] {
        let _ = finite(intrinsics, name)?;
    }
    let reference_extent = image::image_dimensions(&reference_path).map_err(|error| {
        GaussianSplatValidationError::ArtifactMismatch {
            label: "real_sim_observation_comparison.reference_image".into(),
            message: error.to_string(),
        }
    })?;
    ensure!(
        u64::from(reference_extent.0) == width && u64::from(reference_extent.1) == height,
        "registered reference extent differs from intrinsics"
    );
    let actual = compare_registered_gaussian_splat_observations(&reference_path, &render_path)?;
    let declared = object(field(value, "metrics")?, "observation metrics")?;
    for (name, actual) in [
        ("rgb_mae_8bit", actual.rgb_mae_8bit),
        ("rgb_rmse_8bit", actual.rgb_rmse_8bit),
        ("rgb_psnr_db", actual.rgb_psnr_db),
        ("luminance_pearson", actual.luminance_pearson),
        (
            "gradient_magnitude_pearson",
            actual.gradient_magnitude_pearson,
        ),
    ] {
        let metric_matches = (finite(declared, name)? - actual).abs() <= 1.0e-9;
        ensure!(metric_matches, "observation metric {name} drifted");
    }
    let tolerances = object(field(value, "tolerances")?, "observation tolerances")?;
    let min_psnr = finite(tolerances, "min_rgb_psnr_db")?;
    let min_luminance = finite(tolerances, "min_luminance_pearson")?;
    let min_gradient = finite(tolerances, "min_gradient_magnitude_pearson")?;
    let registered_tolerances = gaussian_splat_observation_tolerances();
    ensure!(
        (min_psnr - registered_tolerances.min_rgb_psnr_db).abs() <= f64::EPSILON
            && (min_luminance - registered_tolerances.min_luminance_pearson).abs() <= f64::EPSILON
            && (min_gradient - registered_tolerances.min_gradient_magnitude_pearson).abs()
                <= f64::EPSILON,
        "observation tolerances are not the registered limits"
    );
    let passed = actual.rgb_psnr_db >= min_psnr
        && actual.luminance_pearson >= min_luminance
        && actual.gradient_magnitude_pearson >= min_gradient;
    ensure!(
        string(value, "status")? == pass_status(passed),
        "observation status does not match recomputed metrics"
    );
    Ok(passed)
}

fn compare_observations(
    reference_rgb8: &[u8],
    rendered_rgb8: &[u8],
    width: usize,
    height: usize,
) -> Result<GaussianSplatObservationMetrics, GaussianSplatValidationError> {
    ensure!(
        reference_rgb8.len() == rendered_rgb8.len()
            && reference_rgb8.len() == width.saturating_mul(height).saturating_mul(3)
            && width >= 3
            && height >= 3,
        "registered observation buffers are inconsistent"
    );
    let mut absolute_sum = 0.0;
    let mut squared_sum = 0.0;
    let mut reference_luminance = Vec::with_capacity(width * height);
    let mut rendered_luminance = Vec::with_capacity(width * height);
    for (reference, rendered) in reference_rgb8
        .chunks_exact(3)
        .zip(rendered_rgb8.chunks_exact(3))
    {
        for channel in 0..3 {
            let error = f64::from(reference[channel]) - f64::from(rendered[channel]);
            absolute_sum += error.abs();
            squared_sum += error * error;
        }
        reference_luminance.push(rgb_luminance(reference));
        rendered_luminance.push(rgb_luminance(rendered));
    }
    let count = reference_rgb8.len() as f64;
    let rmse = (squared_sum / count).sqrt();
    let reference_gradient = gradient_magnitudes(&reference_luminance, width, height);
    let rendered_gradient = gradient_magnitudes(&rendered_luminance, width, height);
    Ok(GaussianSplatObservationMetrics {
        rgb_mae_8bit: absolute_sum / count,
        rgb_rmse_8bit: rmse,
        rgb_psnr_db: 20.0 * (255.0 / rmse).log10(),
        luminance_pearson: pearson_correlation(&reference_luminance, &rendered_luminance)?,
        gradient_magnitude_pearson: pearson_correlation(&reference_gradient, &rendered_gradient)?,
    })
}

fn rgb_luminance(rgb: &[u8]) -> f64 {
    0.2126 * f64::from(rgb[0]) + 0.7152 * f64::from(rgb[1]) + 0.0722 * f64::from(rgb[2])
}

fn gradient_magnitudes(luminance: &[f64], width: usize, height: usize) -> Vec<f64> {
    let mut result = Vec::with_capacity((width - 2) * (height - 2));
    for y in 1..height - 1 {
        for x in 1..width - 1 {
            let dx = luminance[y * width + x + 1] - luminance[y * width + x - 1];
            let dy = luminance[(y + 1) * width + x] - luminance[(y - 1) * width + x];
            result.push(dx.hypot(dy));
        }
    }
    result
}

fn pearson_correlation(left: &[f64], right: &[f64]) -> Result<f64, GaussianSplatValidationError> {
    ensure!(
        left.len() == right.len() && !left.is_empty(),
        "correlation buffers are inconsistent"
    );
    let count = left.len() as f64;
    let left_mean = left.iter().sum::<f64>() / count;
    let right_mean = right.iter().sum::<f64>() / count;
    let mut covariance = 0.0;
    let mut left_variance = 0.0;
    let mut right_variance = 0.0;
    for (&left, &right) in left.iter().zip(right) {
        let left_delta = left - left_mean;
        let right_delta = right - right_mean;
        covariance += left_delta * right_delta;
        left_variance += left_delta * left_delta;
        right_variance += right_delta * right_delta;
    }
    ensure!(
        left_variance.is_sign_positive() && right_variance.is_sign_positive(),
        "correlation inputs must not be constant"
    );
    Ok(covariance / (left_variance * right_variance).sqrt())
}

fn validate_contracts(
    values: &[Value],
) -> Result<BTreeMap<&str, &str>, GaussianSplatValidationError> {
    ensure!(
        values.len() == EXPECTED_CONTRACTS.len(),
        "contract count drifted"
    );
    let mut contracts = BTreeMap::new();
    for value in values {
        let contract = object(value, "contract")?;
        let id = nonempty_string(contract, "id")?;
        let status = string(contract, "status")?;
        ensure!(
            matches!(status, "passed" | "missing" | "failed"),
            "unknown contract status"
        );
        ensure!(
            contracts.insert(id, status).is_none(),
            "duplicate contract id"
        );
    }
    ensure!(
        contracts.keys().copied().collect::<BTreeSet<_>>()
            == EXPECTED_CONTRACTS.into_iter().collect::<BTreeSet<_>>(),
        "contract ids drifted"
    );
    Ok(contracts)
}

fn verify_artifact(
    artifact: &Map<String, Value>,
    parent: &Path,
    label: &str,
) -> Result<PathBuf, GaussianSplatValidationError> {
    let relative = nonempty_string(artifact, "path")?;
    let parsed = Path::new(relative);
    ensure!(
        !parsed.is_absolute()
            && parsed
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "artifact path is not canonical relative path"
    );
    let path = parent.join(parsed);
    let metadata =
        fs::symlink_metadata(&path).map_err(|error| GaussianSplatValidationError::Io {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    if !metadata.is_file() || metadata.file_type().is_symlink() {
        return Err(GaussianSplatValidationError::ArtifactMismatch {
            label: label.to_string(),
            message: "artifact must be a regular non-symlink file".into(),
        });
    }
    let expected_size = unsigned(artifact, "size_bytes")?;
    if metadata.len() != expected_size {
        return Err(GaussianSplatValidationError::ArtifactMismatch {
            label: label.to_string(),
            message: format!("size {} != {expected_size}", metadata.len()),
        });
    }
    let expected_hash = string(artifact, "sha256")?;
    ensure_sha256(expected_hash, label)?;
    let bytes = fs::read(&path).map_err(|error| GaussianSplatValidationError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let actual_hash = sha256_hex(&bytes);
    if actual_hash != expected_hash {
        return Err(GaussianSplatValidationError::ArtifactMismatch {
            label: label.to_string(),
            message: format!("SHA-256 {actual_hash} != {expected_hash}"),
        });
    }
    Ok(path)
}

fn ensure_exact_keys(
    value: &Map<String, Value>,
    expected: &[&str],
    label: &str,
) -> Result<(), GaussianSplatValidationError> {
    let actual = value.keys().map(String::as_str).collect::<BTreeSet<_>>();
    let expected = expected.iter().copied().collect::<BTreeSet<_>>();
    ensure!(actual == expected, "{label} fields drifted");
    Ok(())
}

fn field<'a>(
    value: &'a Map<String, Value>,
    name: &str,
) -> Result<&'a Value, GaussianSplatValidationError> {
    value
        .get(name)
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("missing field {name}")))
}

fn object<'a>(
    value: &'a Value,
    label: &str,
) -> Result<&'a Map<String, Value>, GaussianSplatValidationError> {
    value
        .as_object()
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{label} is not an object")))
}

fn array<'a>(value: &'a Value, label: &str) -> Result<&'a [Value], GaussianSplatValidationError> {
    value
        .as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{label} is not an array")))
}

fn string<'a>(
    value: &'a Map<String, Value>,
    name: &str,
) -> Result<&'a str, GaussianSplatValidationError> {
    field(value, name)?
        .as_str()
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{name} is not a string")))
}

fn nonempty_string<'a>(
    value: &'a Map<String, Value>,
    name: &str,
) -> Result<&'a str, GaussianSplatValidationError> {
    let result = string(value, name)?;
    ensure!(!result.trim().is_empty(), "{name} must be non-empty");
    Ok(result)
}

fn unsigned(value: &Map<String, Value>, name: &str) -> Result<u64, GaussianSplatValidationError> {
    field(value, name)?.as_u64().ok_or_else(|| {
        GaussianSplatValidationError::Invalid(format!("{name} is not an unsigned integer"))
    })
}

fn boolean(value: &Map<String, Value>, name: &str) -> Result<bool, GaussianSplatValidationError> {
    field(value, name)?
        .as_bool()
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{name} is not a boolean")))
}

fn finite(value: &Map<String, Value>, name: &str) -> Result<f64, GaussianSplatValidationError> {
    let result = field(value, name)?
        .as_f64()
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{name} is not numeric")))?;
    ensure!(result.is_finite(), "{name} must be finite");
    Ok(result)
}

fn finite_vector(
    value: &Value,
    width: usize,
    label: &str,
) -> Result<Vec<f64>, GaussianSplatValidationError> {
    let values = array(value, label)?;
    ensure!(values.len() == width, "{label} width drifted");
    values
        .iter()
        .map(|value| {
            let number = value.as_f64().ok_or_else(|| {
                GaussianSplatValidationError::Invalid(format!("{label} contains non-number"))
            })?;
            ensure!(number.is_finite(), "{label} contains non-finite number");
            Ok(number)
        })
        .collect()
}

fn ensure_sha256(value: &str, label: &str) -> Result<(), GaussianSplatValidationError> {
    ensure!(
        value.len() == 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        "{label} is not lowercase SHA-256"
    );
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn pass_or_fail(passed: bool) -> &'static str {
    if passed {
        "passed"
    } else {
        "failed"
    }
}

fn pass_status(passed: bool) -> &'static str {
    if passed {
        "verified"
    } else {
        "failed"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn committed_fixture() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/environments/voxel51_drjohnson_3dgs/drjohnson.validation.json")
    }

    #[test]
    fn drjohnson_fixture_exposes_exact_remaining_contracts() {
        let audit = audit_gaussian_splat_validation_fixture(&committed_fixture()).unwrap();
        assert!(!audit.qualifying);
        assert_eq!(
            audit.passed_contracts,
            [
                "floor_world_alignment",
                "camera_intrinsics_extrinsics",
                "semantic_landmark_reprojection",
                "collision_semantic_alignment",
                "real_sim_observation_comparison",
            ]
        );
        assert_eq!(audit.missing_contracts, ["independent_metric_scale_anchor"]);
        assert!(audit.failed_contracts.is_empty());
        assert!(matches!(
            require_qualifying_gaussian_splat_fixture(&committed_fixture()),
            Err(GaussianSplatValidationError::NotQualifying { .. })
        ));
    }
}
