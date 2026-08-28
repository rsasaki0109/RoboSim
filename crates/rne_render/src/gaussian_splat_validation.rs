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
const EXPECTED_CONTRACTS: [&str; 8] = [
    "floor_world_alignment",
    "camera_intrinsics_extrinsics",
    "semantic_landmark_reprojection",
    "collision_semantic_alignment",
    "independent_metric_scale_anchor",
    "real_sim_observation_comparison",
    "sparse_depth_alignment",
    "multiview_depth_occlusion_alignment",
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
            "sparse_depth_alignment",
            "multiview_depth_occlusion_alignment",
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

    let provenance = object(field(root, "provenance")?, "provenance")?;
    validate_provenance(provenance, parent)?;
    let source_to_world = object(field(root, "source_to_world")?, "source_to_world")?;
    validate_source_to_world(source_to_world)?;
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
    let metric_passed = validate_metric_scale(
        metric,
        parent,
        &environment_id,
        finite(source_to_world, "scale")?,
    )?;
    let observation_passed = validate_observation_comparison(
        object(
            field(root, "real_sim_observation_comparison")?,
            "real_sim_observation_comparison",
        )?,
        parent,
        &environment_id,
        renderer_identity,
    )?;
    let sparse_depth_passed = validate_sparse_depth_alignment(
        object(
            field(root, "sparse_depth_alignment")?,
            "sparse_depth_alignment",
        )?,
        parent,
        &environment_id,
        string(
            object(field(provenance, "splat_ply")?, "splat_ply")?,
            "sha256",
        )?,
    )?;
    let multiview_depth_passed = validate_multiview_depth_occlusion_alignment(
        object(
            field(root, "multiview_depth_occlusion_alignment")?,
            "multiview_depth_occlusion_alignment",
        )?,
        parent,
        &environment_id,
        string(
            object(field(provenance, "splat_ply")?, "splat_ply")?,
            "sha256",
        )?,
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
        ("sparse_depth_alignment", pass_or_fail(sparse_depth_passed)),
        (
            "multiview_depth_occlusion_alignment",
            pass_or_fail(multiview_depth_passed),
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

fn validate_metric_scale(
    value: &Map<String, Value>,
    parent: &Path,
    environment_id: &str,
    source_to_world_scale: f64,
) -> Result<bool, GaussianSplatValidationError> {
    ensure_exact_keys(
        value,
        &[
            "status",
            "scale_to_m",
            "independent_physical_anchor",
            "reason",
        ],
        "metric_scale",
    )?;
    let status = string(value, "status")?;
    let scale_to_m = finite(value, "scale_to_m")?;
    ensure!(
        scale_to_m.is_sign_positive(),
        "metric scale must be positive"
    );
    ensure!(
        within_tolerance(scale_to_m, source_to_world_scale, f64::EPSILON),
        "metric scale and source-to-world scale disagree"
    );
    let _ = nonempty_string(value, "reason")?;
    let anchor_value = field(value, "independent_physical_anchor")?;
    if status == "unverified" {
        ensure!(
            anchor_value.is_null(),
            "unverified metric scale retained an anchor"
        );
        return Ok(false);
    }
    ensure!(status == "verified", "unknown metric scale status");
    let anchor = object(anchor_value, "independent physical anchor")?;
    ensure_exact_keys(
        anchor,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "record",
            "operator",
            "measurement",
            "endpoints",
            "source_distance_reconstruction_units",
            "derived_scale_m_per_source_unit",
            "scale_uncertainty_m_per_source_unit",
            "evidence_artifacts",
        ],
        "independent physical anchor",
    )?;
    ensure!(
        string(anchor, "kind")? == "rne_independent_metric_scale_anchor"
            && unsigned(anchor, "schema_version")? == 1
            && string(anchor, "environment_id")? == environment_id,
        "metric anchor identity drifted"
    );

    let operator = object(field(anchor, "operator")?, "metric anchor operator")?;
    ensure_exact_keys(
        operator,
        &[
            "organization",
            "operator_role",
            "independence_statement",
            "independent_from_rne_fixture_authoring",
        ],
        "metric anchor operator",
    )?;
    for name in ["organization", "operator_role", "independence_statement"] {
        let _ = nonempty_string(operator, name)?;
    }
    ensure!(
        boolean(operator, "independent_from_rne_fixture_authoring")?,
        "metric anchor operator did not attest independence"
    );

    let measurement = object(field(anchor, "measurement")?, "metric anchor measurement")?;
    ensure_exact_keys(
        measurement,
        &[
            "method",
            "captured_at_utc",
            "measured_distance_m",
            "uncertainty_m",
        ],
        "metric anchor measurement",
    )?;
    let _ = nonempty_string(measurement, "method")?;
    ensure!(
        is_canonical_utc_timestamp(nonempty_string(measurement, "captured_at_utc")?),
        "metric anchor capture time must be canonical RFC 3339 UTC"
    );
    let measured_distance = finite(measurement, "measured_distance_m")?;
    let uncertainty = finite(measurement, "uncertainty_m")?;
    ensure!(
        measured_distance.is_sign_positive()
            && uncertainty.is_sign_positive()
            && (0.0..measured_distance).contains(&uncertainty),
        "invalid metric anchor distance or uncertainty"
    );

    let endpoints = array(field(anchor, "endpoints")?, "metric anchor endpoints")?;
    ensure!(endpoints.len() == 2, "metric anchor requires two endpoints");
    let mut endpoint_ids = BTreeSet::new();
    let mut point_ids = BTreeSet::new();
    let mut camera_ids = BTreeSet::new();
    let mut positions = Vec::with_capacity(2);
    for endpoint_value in endpoints {
        let endpoint = object(endpoint_value, "metric anchor endpoint")?;
        ensure_exact_keys(
            endpoint,
            &[
                "endpoint_id",
                "camera_id",
                "pixel_uv",
                "colmap_point3d_id",
                "source_position",
                "registration_error_px",
            ],
            "metric anchor endpoint",
        )?;
        ensure!(
            endpoint_ids.insert(nonempty_string(endpoint, "endpoint_id")?),
            "metric anchor endpoint IDs must be distinct"
        );
        let camera_id = nonempty_string(endpoint, "camera_id")?;
        ensure!(
            camera_id.starts_with("colmap."),
            "metric anchor endpoint must name a COLMAP camera"
        );
        camera_ids.insert(camera_id);
        finite_vector(field(endpoint, "pixel_uv")?, 2, "metric anchor pixel_uv")?;
        ensure!(
            point_ids.insert(unsigned(endpoint, "colmap_point3d_id")?),
            "metric anchor point IDs must be distinct"
        );
        positions.push(finite_vector(
            field(endpoint, "source_position")?,
            3,
            "metric anchor source_position",
        )?);
        let registration_error = finite(endpoint, "registration_error_px")?;
        ensure!(
            (0.0..=1.0e-6).contains(&registration_error),
            "metric anchor endpoint registration error exceeded tolerance"
        );
    }
    ensure!(
        camera_ids.len() == 1,
        "metric anchor endpoints must share one retained camera frame"
    );
    let source_distance = positions[0]
        .iter()
        .zip(&positions[1])
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>()
        .sqrt();
    ensure!(
        source_distance.is_finite() && (1.0e-9..).contains(&source_distance),
        "metric anchor endpoints coincide"
    );
    let declared_source_distance = finite(anchor, "source_distance_reconstruction_units")?;
    ensure!(
        within_tolerance(declared_source_distance, source_distance, 1.0e-9),
        "metric anchor source distance drifted"
    );
    let derived_scale = measured_distance / source_distance;
    let scale_uncertainty = uncertainty / source_distance;
    ensure!(
        within_tolerance(
            finite(anchor, "derived_scale_m_per_source_unit")?,
            derived_scale,
            1.0e-9,
        ) && within_tolerance(
            finite(anchor, "scale_uncertainty_m_per_source_unit")?,
            scale_uncertainty,
            1.0e-9,
        ),
        "metric anchor derived scale drifted"
    );
    ensure!(
        within_tolerance(scale_to_m, derived_scale, scale_uncertainty),
        "source-to-world scale is outside metric anchor uncertainty"
    );

    let evidence = array(
        field(anchor, "evidence_artifacts")?,
        "metric anchor evidence",
    )?;
    ensure!(
        !evidence.is_empty(),
        "metric anchor has no evidence artifacts"
    );
    for (index, artifact_value) in evidence.iter().enumerate() {
        let artifact = object(artifact_value, "metric anchor evidence artifact")?;
        ensure_exact_keys(
            artifact,
            &["path", "size_bytes", "sha256", "description"],
            "metric anchor evidence artifact",
        )?;
        let _ = nonempty_string(artifact, "description")?;
        verify_artifact(
            artifact,
            parent,
            &format!("metric anchor evidence[{index}]"),
        )?;
    }

    let record_path = verify_artifact(
        object(field(anchor, "record")?, "metric anchor record")?,
        parent,
        "metric anchor record",
    )?;
    let record_bytes =
        fs::read(&record_path).map_err(|error| GaussianSplatValidationError::Io {
            path: record_path.display().to_string(),
            message: error.to_string(),
        })?;
    let record_value: Value = serde_json::from_slice(&record_bytes).map_err(|error| {
        GaussianSplatValidationError::Parse {
            path: record_path.display().to_string(),
            message: error.to_string(),
        }
    })?;
    let record = object(&record_value, "metric anchor record")?;
    ensure_exact_keys(
        record,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "operator",
            "measurement",
            "endpoints",
            "evidence_artifacts",
        ],
        "metric anchor record",
    )?;
    ensure!(
        field(record, "kind")? == field(anchor, "kind")?
            && field(record, "schema_version")? == field(anchor, "schema_version")?
            && field(record, "environment_id")? == field(anchor, "environment_id")?
            && field(record, "operator")? == field(anchor, "operator")?
            && field(record, "measurement")? == field(anchor, "measurement")?
            && field(record, "evidence_artifacts")? == field(anchor, "evidence_artifacts")?,
        "metric anchor record and fixture disagree"
    );
    let record_endpoints = array(
        field(record, "endpoints")?,
        "metric anchor record endpoints",
    )?;
    ensure!(
        record_endpoints.len() == endpoints.len(),
        "metric anchor endpoint count drifted"
    );
    for (record_value, resolved_value) in record_endpoints.iter().zip(endpoints) {
        let record_endpoint = object(record_value, "metric anchor record endpoint")?;
        let resolved_endpoint = object(resolved_value, "metric anchor resolved endpoint")?;
        ensure_exact_keys(
            record_endpoint,
            &["endpoint_id", "camera_id", "pixel_uv", "colmap_point3d_id"],
            "metric anchor record endpoint",
        )?;
        for name in ["endpoint_id", "camera_id", "pixel_uv", "colmap_point3d_id"] {
            ensure!(
                field(record_endpoint, name)? == field(resolved_endpoint, name)?,
                "metric anchor endpoint record drifted"
            );
        }
    }
    Ok(true)
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

fn validate_sparse_depth_alignment(
    value: &Map<String, Value>,
    parent: &Path,
    environment_id: &str,
    expected_ply_sha256: &str,
) -> Result<bool, GaussianSplatValidationError> {
    ensure_exact_keys(
        value,
        &[
            "status",
            "reference_camera_id",
            "depth_algorithm_identity",
            "report",
            "depth_frame_hash",
            "finite_depth_pixels",
            "finite_depth_fraction",
            "matched_landmarks",
            "mean_absolute_error_source_units",
            "max_absolute_error_source_units",
            "tolerances",
            "metric_qualified",
            "units_note",
        ],
        "sparse_depth_alignment",
    )?;
    let camera_id = nonempty_string(value, "reference_camera_id")?;
    let algorithm = nonempty_string(value, "depth_algorithm_identity")?;
    ensure!(
        algorithm == "rne.gaussian_splat.alpha_proxy_depth.v2",
        "unsupported sparse-depth algorithm"
    );
    let report_path = verify_artifact(
        object(field(value, "report")?, "sparse-depth report artifact")?,
        parent,
        "sparse_depth_alignment.report",
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
    let report = object(&report_value, "sparse-depth report")?;
    ensure_exact_keys(
        report,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "camera_id",
            "camera_calibration_sha256",
            "depth_algorithm_identity",
            "ply_sha256",
            "width_px",
            "height_px",
            "depth_frame_hash",
            "finite_depth_pixels",
            "finite_depth_fraction",
            "patch_radius_px",
            "landmarks",
            "matched_landmarks",
            "mean_absolute_error_source_units",
            "max_absolute_error_source_units",
            "tolerances",
            "passed",
            "metric_qualified",
            "units_note",
        ],
        "sparse-depth report",
    )?;
    ensure!(
        string(report, "kind")? == "rne_registered_splat_sparse_depth_report"
            && unsigned(report, "schema_version")? == 1,
        "unsupported sparse-depth report identity"
    );
    ensure!(
        string(report, "environment_id")? == environment_id
            && string(report, "camera_id")? == camera_id
            && string(report, "depth_algorithm_identity")? == algorithm,
        "sparse-depth report identity drifted"
    );
    ensure_sha256(
        string(report, "camera_calibration_sha256")?,
        "camera_calibration_sha256",
    )?;
    ensure!(
        string(report, "ply_sha256")? == expected_ply_sha256,
        "sparse-depth report PLY digest drifted"
    );
    let width = unsigned(report, "width_px")?;
    let height = unsigned(report, "height_px")?;
    ensure!(width > 0 && height > 0, "sparse-depth extent is empty");
    let finite_pixels = unsigned(report, "finite_depth_pixels")?;
    ensure!(
        finite_pixels <= width * height,
        "sparse-depth finite pixel count exceeds extent"
    );
    let finite_fraction = finite(report, "finite_depth_fraction")?;
    let recomputed_fraction = finite_pixels as f64 / (width * height) as f64;
    ensure!(
        within_tolerance(finite_fraction, recomputed_fraction, 1.0e-12),
        "sparse-depth coverage fraction drifted"
    );
    ensure!(
        unsigned(report, "patch_radius_px")? == 2,
        "sparse-depth sampling patch drifted"
    );

    let landmarks = array(field(report, "landmarks")?, "sparse-depth landmarks")?;
    let mut errors = Vec::with_capacity(landmarks.len());
    let mut landmark_ids = BTreeSet::new();
    for landmark in landmarks {
        let landmark = object(landmark, "sparse-depth landmark")?;
        ensure_exact_keys(
            landmark,
            &[
                "landmark_id",
                "semantic_class",
                "pixel_u_px",
                "pixel_v_px",
                "reference_depth_source_units",
                "rne_proxy_depth_source_units",
                "absolute_error_source_units",
            ],
            "sparse-depth landmark",
        )?;
        ensure!(
            landmark_ids.insert(nonempty_string(landmark, "landmark_id")?),
            "duplicate sparse-depth landmark"
        );
        let _ = nonempty_string(landmark, "semantic_class")?;
        let _ = finite(landmark, "pixel_u_px")?;
        let _ = finite(landmark, "pixel_v_px")?;
        let reference = finite(landmark, "reference_depth_source_units")?;
        let proxy = finite(landmark, "rne_proxy_depth_source_units")?;
        let declared_error = finite(landmark, "absolute_error_source_units")?;
        ensure!(
            within_tolerance(declared_error, (reference - proxy).abs(), 1.0e-9),
            "sparse-depth landmark error drifted"
        );
        errors.push(declared_error);
    }
    let matched = unsigned(report, "matched_landmarks")? as usize;
    ensure!(
        matched == landmarks.len() && matched == errors.len(),
        "sparse-depth matched landmark count drifted"
    );
    let mean_error = finite(report, "mean_absolute_error_source_units")?;
    let max_error = finite(report, "max_absolute_error_source_units")?;
    ensure!(!errors.is_empty(), "sparse-depth report has no landmarks");
    let recomputed_mean = errors.iter().sum::<f64>() / errors.len() as f64;
    let recomputed_max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    ensure!(
        within_tolerance(mean_error, recomputed_mean, 1.0e-12)
            && within_tolerance(max_error, recomputed_max, 1.0e-12),
        "sparse-depth aggregate metrics drifted"
    );

    let tolerances = object(field(report, "tolerances")?, "sparse-depth tolerances")?;
    ensure_exact_keys(
        tolerances,
        &[
            "min_finite_depth_fraction",
            "min_matched_landmarks",
            "max_mean_absolute_error_source_units",
            "max_absolute_error_source_units",
        ],
        "sparse-depth tolerances",
    )?;
    let min_fraction = finite(tolerances, "min_finite_depth_fraction")?;
    let min_matched = unsigned(tolerances, "min_matched_landmarks")? as usize;
    let max_mean = finite(tolerances, "max_mean_absolute_error_source_units")?;
    let max_single = finite(tolerances, "max_absolute_error_source_units")?;
    ensure!(
        min_fraction == 0.75 && min_matched == 6 && max_mean == 0.25 && max_single == 0.75,
        "sparse-depth tolerances are not the registered limits"
    );
    let passed = finite_fraction >= min_fraction
        && matched >= min_matched
        && mean_error <= max_mean
        && max_error <= max_single;
    ensure!(
        boolean(report, "passed")? == passed,
        "sparse-depth report verdict drifted"
    );
    ensure!(
        !boolean(report, "metric_qualified")?,
        "sparse-depth report must remain independent of metric qualification"
    );
    let units_note = nonempty_string(report, "units_note")?;
    ensure!(
        units_note.contains("not metres"),
        "source-unit sparse depth must disclaim metres"
    );
    ensure!(
        field(value, "depth_algorithm_identity")? == field(report, "depth_algorithm_identity")?
            && field(value, "depth_frame_hash")? == field(report, "depth_frame_hash")?
            && field(value, "finite_depth_pixels")? == field(report, "finite_depth_pixels")?
            && field(value, "finite_depth_fraction")? == field(report, "finite_depth_fraction")?
            && field(value, "matched_landmarks")? == field(report, "matched_landmarks")?
            && field(value, "mean_absolute_error_source_units")?
                == field(report, "mean_absolute_error_source_units")?
            && field(value, "max_absolute_error_source_units")?
                == field(report, "max_absolute_error_source_units")?
            && field(value, "tolerances")? == field(report, "tolerances")?
            && field(value, "metric_qualified")? == field(report, "metric_qualified")?
            && field(value, "units_note")? == field(report, "units_note")?,
        "sparse-depth fixture and report disagree"
    );
    ensure!(
        string(value, "status")? == pass_status(passed),
        "sparse-depth fixture status drifted"
    );
    Ok(passed)
}

fn validate_multiview_depth_occlusion_alignment(
    value: &Map<String, Value>,
    parent: &Path,
    environment_id: &str,
    expected_ply_sha256: &str,
) -> Result<bool, GaussianSplatValidationError> {
    ensure_exact_keys(
        value,
        &[
            "status",
            "track_fixture",
            "report",
            "depth_algorithm_identity",
            "frames",
            "matched_track_count",
            "matched_view_count",
            "mean_absolute_error_source_units",
            "max_absolute_error_source_units",
            "depth_delta_mae_source_units",
            "false_occlusion_view_count",
            "false_occlusion_fraction",
            "tolerances",
            "metric_qualified",
            "units_note",
        ],
        "multiview_depth_occlusion_alignment",
    )?;
    let track_path = verify_artifact(
        object(field(value, "track_fixture")?, "multi-view track fixture")?,
        parent,
        "multiview_depth_occlusion_alignment.track_fixture",
    )?;
    let report_path = verify_artifact(
        object(field(value, "report")?, "multi-view depth report")?,
        parent,
        "multiview_depth_occlusion_alignment.report",
    )?;
    let track_bytes = fs::read(&track_path).map_err(|error| GaussianSplatValidationError::Io {
        path: track_path.display().to_string(),
        message: error.to_string(),
    })?;
    let report_bytes =
        fs::read(&report_path).map_err(|error| GaussianSplatValidationError::Io {
            path: report_path.display().to_string(),
            message: error.to_string(),
        })?;
    let track_value: Value = serde_json::from_slice(&track_bytes).map_err(|error| {
        GaussianSplatValidationError::Parse {
            path: track_path.display().to_string(),
            message: error.to_string(),
        }
    })?;
    let report_value: Value = serde_json::from_slice(&report_bytes).map_err(|error| {
        GaussianSplatValidationError::Parse {
            path: report_path.display().to_string(),
            message: error.to_string(),
        }
    })?;
    let track_fixture = object(&track_value, "multi-view track fixture")?;
    ensure!(
        string(track_fixture, "kind")? == "rne_registered_colmap_multiview_tracks"
            && unsigned(track_fixture, "schema_version")? == 1
            && string(track_fixture, "environment_id")? == environment_id
            && string(track_fixture, "status")? == "verified",
        "multi-view track fixture identity drifted"
    );
    let source_tracks = array(field(track_fixture, "tracks")?, "multi-view source tracks")?;
    ensure!(
        unsigned(track_fixture, "track_count")? as usize == source_tracks.len(),
        "multi-view source track count drifted"
    );

    let report = object(&report_value, "multi-view depth report")?;
    ensure_exact_keys(
        report,
        &[
            "kind",
            "schema_version",
            "environment_id",
            "track_fixture_sha256",
            "ply_sha256",
            "depth_algorithm_identity",
            "frames",
            "tracks",
            "matched_track_count",
            "matched_view_count",
            "mean_absolute_error_source_units",
            "max_absolute_error_source_units",
            "depth_delta_mae_source_units",
            "false_occlusion_view_count",
            "false_occlusion_fraction",
            "tolerances",
            "passed",
            "metric_qualified",
            "units_note",
        ],
        "multi-view depth report",
    )?;
    ensure!(
        string(report, "kind")? == "rne_registered_splat_multiview_depth_report"
            && unsigned(report, "schema_version")? == 1
            && string(report, "environment_id")? == environment_id
            && string(report, "track_fixture_sha256")? == sha256_hex(&track_bytes)
            && string(report, "ply_sha256")? == expected_ply_sha256
            && string(report, "depth_algorithm_identity")?
                == "rne.gaussian_splat.alpha_proxy_depth.v2",
        "multi-view depth report identity drifted"
    );

    let frames = array(field(report, "frames")?, "multi-view depth frames")?;
    ensure!(
        frames.len() == 2,
        "multi-view report must retain two frames"
    );
    let mut camera_ids = BTreeSet::new();
    let mut frame_fractions = Vec::new();
    for frame in frames {
        let frame = object(frame, "multi-view depth frame")?;
        ensure_exact_keys(
            frame,
            &[
                "camera_id",
                "width_px",
                "height_px",
                "depth_frame_hash",
                "finite_depth_pixels",
                "finite_depth_fraction",
            ],
            "multi-view depth frame",
        )?;
        ensure!(
            camera_ids.insert(nonempty_string(frame, "camera_id")?),
            "duplicate multi-view frame camera"
        );
        let width = unsigned(frame, "width_px")?;
        let height = unsigned(frame, "height_px")?;
        let _ = unsigned(frame, "depth_frame_hash")?;
        let finite_pixels = unsigned(frame, "finite_depth_pixels")?;
        ensure!(
            width > 0 && height > 0 && finite_pixels <= width * height,
            "invalid multi-view depth frame extent or coverage"
        );
        let fraction = finite(frame, "finite_depth_fraction")?;
        ensure!(
            within_tolerance(
                fraction,
                finite_pixels as f64 / (width * height) as f64,
                1.0e-12,
            ),
            "multi-view frame coverage drifted"
        );
        frame_fractions.push(fraction);
    }

    let report_tracks = array(field(report, "tracks")?, "multi-view report tracks")?;
    ensure!(
        report_tracks.len() == source_tracks.len(),
        "multi-view report track count differs from source"
    );
    let mut absolute_errors = Vec::new();
    let mut delta_errors = Vec::new();
    let mut false_occlusions = 0_u64;
    for (source_track, report_track) in source_tracks.iter().zip(report_tracks) {
        let source_track = object(source_track, "multi-view source track")?;
        let report_track = object(report_track, "multi-view report track")?;
        ensure_exact_keys(
            report_track,
            &[
                "colmap_point3d_id",
                "views",
                "depth_delta_error_source_units",
            ],
            "multi-view report track",
        )?;
        let point_id = unsigned(source_track, "colmap_point3d_id")?;
        ensure!(
            unsigned(report_track, "colmap_point3d_id")? == point_id,
            "multi-view track identity drifted"
        );
        let source_views = array(field(source_track, "views")?, "multi-view source views")?;
        let report_views = array(field(report_track, "views")?, "multi-view report views")?;
        ensure!(
            source_views.len() == 2 && report_views.len() == 2,
            "multi-view track must retain two views"
        );
        let mut matched_depths = Vec::with_capacity(2);
        for (source_view, report_view) in source_views.iter().zip(report_views) {
            let source_view = object(source_view, "multi-view source view")?;
            let report_view = object(report_view, "multi-view report view")?;
            ensure_exact_keys(
                report_view,
                &[
                    "camera_id",
                    "observed_pixel_uv",
                    "reference_depth_source_units",
                    "rne_proxy_depth_source_units",
                    "signed_error_source_units",
                    "false_occlusion",
                ],
                "multi-view report view",
            )?;
            let source_pixel = finite_vector(
                field(source_view, "observed_pixel_uv")?,
                2,
                "multi-view source observed_pixel_uv",
            )?;
            let report_pixel = finite_vector(
                field(report_view, "observed_pixel_uv")?,
                2,
                "multi-view report observed_pixel_uv",
            )?;
            let source_reference = finite(source_view, "reference_depth_source_units")?;
            let report_reference = finite(report_view, "reference_depth_source_units")?;
            ensure!(
                field(source_view, "camera_id")? == field(report_view, "camera_id")?
                    && source_pixel
                        .iter()
                        .zip(&report_pixel)
                        .all(|(source, report)| within_tolerance(*source, *report, 1.0e-9))
                    && within_tolerance(source_reference, report_reference, 1.0e-9),
                "multi-view source and report observation drifted"
            );
            let reference = finite(report_view, "reference_depth_source_units")?;
            let proxy = optional_finite(report_view, "rne_proxy_depth_source_units")?;
            let signed = optional_finite(report_view, "signed_error_source_units")?;
            let occluded = optional_boolean(report_view, "false_occlusion")?;
            ensure!(
                proxy.is_some() == signed.is_some() && proxy.is_some() == occluded.is_some(),
                "multi-view optional depth fields disagree"
            );
            if let (Some(proxy), Some(signed), Some(occluded)) = (proxy, signed, occluded) {
                ensure!(
                    within_tolerance(signed, proxy - reference, 1.0e-9),
                    "multi-view signed depth error drifted"
                );
                ensure!(
                    occluded == (signed < -0.25),
                    "multi-view false-occlusion verdict drifted"
                );
                absolute_errors.push(signed.abs());
                false_occlusions += u64::from(occluded);
                matched_depths.push((reference, proxy));
            }
        }
        let declared_delta = optional_finite(report_track, "depth_delta_error_source_units")?;
        let expected_delta = (matched_depths.len() == 2).then(|| {
            let reference_delta = matched_depths[1].0 - matched_depths[0].0;
            let proxy_delta = matched_depths[1].1 - matched_depths[0].1;
            (proxy_delta - reference_delta).abs()
        });
        ensure!(
            declared_delta.is_some() == expected_delta.is_some()
                && declared_delta
                    .zip(expected_delta)
                    .is_none_or(|(declared, expected)| {
                        within_tolerance(declared, expected, 1.0e-9)
                    }),
            "multi-view depth-delta error drifted"
        );
        if let Some(delta) = expected_delta {
            delta_errors.push(delta);
        }
    }
    ensure!(
        !absolute_errors.is_empty() && !delta_errors.is_empty(),
        "multi-view report has no matched evidence"
    );
    let matched_tracks = delta_errors.len() as u64;
    let matched_views = absolute_errors.len() as u64;
    let mean_error = absolute_errors.iter().sum::<f64>() / absolute_errors.len() as f64;
    let max_error = absolute_errors
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let delta_mae = delta_errors.iter().sum::<f64>() / delta_errors.len() as f64;
    let false_fraction = false_occlusions as f64 / matched_views as f64;
    for (field_name, actual) in [
        ("mean_absolute_error_source_units", mean_error),
        ("max_absolute_error_source_units", max_error),
        ("depth_delta_mae_source_units", delta_mae),
        ("false_occlusion_fraction", false_fraction),
    ] {
        ensure!(
            within_tolerance(finite(report, field_name)?, actual, 1.0e-12),
            "multi-view aggregate {field_name} drifted"
        );
    }
    ensure!(
        unsigned(report, "matched_track_count")? == matched_tracks
            && unsigned(report, "matched_view_count")? == matched_views
            && unsigned(report, "false_occlusion_view_count")? == false_occlusions,
        "multi-view aggregate counts drifted"
    );

    let tolerances = object(field(report, "tolerances")?, "multi-view tolerances")?;
    let min_fraction = finite(tolerances, "min_finite_depth_fraction_per_camera")?;
    let min_tracks = unsigned(tolerances, "min_matched_tracks")?;
    let max_mean = finite(tolerances, "max_mean_absolute_error_source_units")?;
    let max_single = finite(tolerances, "max_absolute_error_source_units")?;
    let max_delta = finite(tolerances, "max_depth_delta_mae_source_units")?;
    let occlusion_margin = finite(tolerances, "false_occlusion_margin_source_units")?;
    let max_occlusion = finite(tolerances, "max_false_occlusion_fraction")?;
    ensure!(
        min_fraction == 0.75
            && min_tracks == 36
            && max_mean == 0.25
            && max_single == 0.75
            && max_delta == 0.25
            && occlusion_margin == 0.25
            && max_occlusion == 0.10,
        "multi-view tolerances are not the registered limits"
    );
    let passed = frame_fractions
        .iter()
        .all(|fraction| *fraction >= min_fraction)
        && matched_tracks >= min_tracks
        && mean_error <= max_mean
        && max_error <= max_single
        && delta_mae <= max_delta
        && false_fraction <= max_occlusion;
    ensure!(
        boolean(report, "passed")? == passed && !boolean(report, "metric_qualified")?,
        "multi-view report verdict or metric qualification drifted"
    );
    ensure!(
        nonempty_string(report, "units_note")?.contains("not metres"),
        "source-unit multi-view depth must disclaim metres"
    );
    for name in [
        "depth_algorithm_identity",
        "frames",
        "matched_track_count",
        "matched_view_count",
        "mean_absolute_error_source_units",
        "max_absolute_error_source_units",
        "depth_delta_mae_source_units",
        "false_occlusion_view_count",
        "false_occlusion_fraction",
        "tolerances",
        "metric_qualified",
        "units_note",
    ] {
        ensure!(
            field(value, name)? == field(report, name)?,
            "multi-view fixture field {name} differs from report"
        );
    }
    ensure!(
        string(value, "status")? == pass_status(passed),
        "multi-view fixture status drifted"
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

fn optional_boolean(
    value: &Map<String, Value>,
    name: &str,
) -> Result<Option<bool>, GaussianSplatValidationError> {
    let field = field(value, name)?;
    if field.is_null() {
        return Ok(None);
    }
    field.as_bool().map(Some).ok_or_else(|| {
        GaussianSplatValidationError::Invalid(format!("{name} is not boolean or null"))
    })
}

fn finite(value: &Map<String, Value>, name: &str) -> Result<f64, GaussianSplatValidationError> {
    let result = field(value, name)?
        .as_f64()
        .ok_or_else(|| GaussianSplatValidationError::Invalid(format!("{name} is not numeric")))?;
    ensure!(result.is_finite(), "{name} must be finite");
    Ok(result)
}

fn optional_finite(
    value: &Map<String, Value>,
    name: &str,
) -> Result<Option<f64>, GaussianSplatValidationError> {
    let field = field(value, name)?;
    if field.is_null() {
        return Ok(None);
    }
    let number = field.as_f64().ok_or_else(|| {
        GaussianSplatValidationError::Invalid(format!("{name} is not numeric or null"))
    })?;
    ensure!(number.is_finite(), "{name} must be finite");
    Ok(Some(number))
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

fn within_tolerance(value: f64, expected: f64, tolerance: f64) -> bool {
    value.is_finite()
        && expected.is_finite()
        && tolerance.is_finite()
        && tolerance.is_sign_positive()
        && (value - expected).abs() <= tolerance
}

fn is_canonical_utc_timestamp(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() < 20
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || bytes.last() != Some(&b'Z')
        || (bytes.len() > 20
            && (bytes.len() < 22
                || bytes[19] != b'.'
                || !bytes[20..bytes.len() - 1].iter().all(u8::is_ascii_digit)))
    {
        return false;
    }
    let parse = |range: std::ops::Range<usize>| value[range].parse::<u32>().ok();
    let Some((year, month, day, hour, minute, second)) = parse(0..4)
        .zip(parse(5..7))
        .zip(parse(8..10))
        .zip(parse(11..13))
        .zip(parse(14..16))
        .zip(parse(17..19))
        .map(|(((((year, month), day), hour), minute), second)| {
            (year, month, day, hour, minute, second)
        })
    else {
        return false;
    };
    let leap_year =
        year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400));
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year => 29,
        2 => 28,
        _ => return false,
    };
    (1..=days_in_month).contains(&day)
        && (0..24).contains(&hour)
        && (0..60).contains(&minute)
        && (0..60).contains(&second)
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
    use serde_json::json;

    fn committed_fixture() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/environments/voxel51_drjohnson_3dgs/drjohnson.validation.json")
    }

    fn metric_anchor_fixture_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/metric_anchor")
    }

    fn artifact(path: &Path, name: &str) -> Value {
        let bytes = fs::read(path).unwrap();
        json!({
            "path": name,
            "size_bytes": bytes.len(),
            "sha256": sha256_hex(&bytes),
        })
    }

    fn verified_metric_anchor() -> Value {
        let root = metric_anchor_fixture_root();
        let record: Value =
            serde_json::from_slice(&fs::read(root.join("anchor.json")).unwrap()).unwrap();
        let mut endpoints = record["endpoints"].as_array().unwrap().clone();
        endpoints[0]["source_position"] = json!([0.0, 0.0, 0.0]);
        endpoints[0]["registration_error_px"] = json!(0.0);
        endpoints[1]["source_position"] = json!([2.0, 0.0, 0.0]);
        endpoints[1]["registration_error_px"] = json!(0.0);
        json!({
            "status": "verified",
            "scale_to_m": 1.0,
            "independent_physical_anchor": {
                "kind": record["kind"].clone(),
                "schema_version": record["schema_version"].clone(),
                "environment_id": record["environment_id"].clone(),
                "record": artifact(&root.join("anchor.json"), "anchor.json"),
                "operator": record["operator"].clone(),
                "measurement": record["measurement"].clone(),
                "endpoints": endpoints,
                "source_distance_reconstruction_units": 2.0,
                "derived_scale_m_per_source_unit": 1.0,
                "scale_uncertainty_m_per_source_unit": 0.005,
                "evidence_artifacts": record["evidence_artifacts"].clone(),
            },
            "reason": "test-only verified independent measurement",
        })
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
                "sparse_depth_alignment",
                "multiview_depth_occlusion_alignment",
            ]
        );
        assert_eq!(audit.missing_contracts, ["independent_metric_scale_anchor"]);
        assert!(audit.failed_contracts.is_empty());
        assert!(matches!(
            require_qualifying_gaussian_splat_fixture(&committed_fixture()),
            Err(GaussianSplatValidationError::NotQualifying { .. })
        ));
    }

    #[test]
    fn sparse_depth_contract_rejects_declared_metric_tampering() {
        let fixture_path = committed_fixture();
        let parent = fixture_path.parent().unwrap();
        let fixture: Value = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
        let root = fixture.as_object().unwrap();
        let provenance = root["provenance"].as_object().unwrap();
        let ply_sha256 = provenance["splat_ply"]["sha256"].as_str().unwrap();
        let sparse = root["sparse_depth_alignment"].clone();
        assert!(validate_sparse_depth_alignment(
            sparse.as_object().unwrap(),
            parent,
            root["environment_id"].as_str().unwrap(),
            ply_sha256,
        )
        .unwrap());

        let mut forged = sparse;
        forged["mean_absolute_error_source_units"] = json!(0.0);
        assert!(matches!(
            validate_sparse_depth_alignment(
                forged.as_object().unwrap(),
                parent,
                root["environment_id"].as_str().unwrap(),
                ply_sha256,
            ),
            Err(GaussianSplatValidationError::Invalid(message))
                if message.contains("fixture and report disagree")
        ));
    }

    #[test]
    fn multiview_depth_contract_rejects_declared_metric_tampering() {
        let fixture_path = committed_fixture();
        let parent = fixture_path.parent().unwrap();
        let fixture: Value = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
        let root = fixture.as_object().unwrap();
        let provenance = root["provenance"].as_object().unwrap();
        let ply_sha256 = provenance["splat_ply"]["sha256"].as_str().unwrap();
        let retained = root["multiview_depth_occlusion_alignment"].clone();
        assert!(validate_multiview_depth_occlusion_alignment(
            retained.as_object().unwrap(),
            parent,
            root["environment_id"].as_str().unwrap(),
            ply_sha256,
        )
        .unwrap());

        let mut forged = retained;
        forged["false_occlusion_fraction"] = json!(0.05);
        assert!(matches!(
            validate_multiview_depth_occlusion_alignment(
                forged.as_object().unwrap(),
                parent,
                root["environment_id"].as_str().unwrap(),
                ply_sha256,
            ),
            Err(GaussianSplatValidationError::Invalid(message))
                if message.contains("differs from report")
        ));
    }

    #[test]
    fn metric_anchor_recomputes_scale_and_rejects_forged_derivation() {
        let root = metric_anchor_fixture_root();
        let value = verified_metric_anchor();
        assert!(validate_metric_scale(
            value.as_object().unwrap(),
            &root,
            "test.metric.anchor",
            1.0,
        )
        .unwrap());

        let mut forged = value;
        forged["independent_physical_anchor"]["derived_scale_m_per_source_unit"] = json!(0.9);
        assert!(matches!(
            validate_metric_scale(
                forged.as_object().unwrap(),
                &root,
                "test.metric.anchor",
                1.0,
            ),
            Err(GaussianSplatValidationError::Invalid(message))
                if message.contains("derived scale drifted")
        ));

        let mut invalid_time = verified_metric_anchor();
        invalid_time["independent_physical_anchor"]["measurement"]["captured_at_utc"] =
            json!("sometime");
        assert!(matches!(
            validate_metric_scale(
                invalid_time.as_object().unwrap(),
                &root,
                "test.metric.anchor",
                1.0,
            ),
            Err(GaussianSplatValidationError::Invalid(message))
                if message.contains("canonical RFC 3339 UTC")
        ));
    }
}
