//! Deterministic 3D point-to-point ICP for LiDAR/point-cloud registration.
//!
//! [`Icp3d::align`] matches a source cloud against a target cloud and returns
//! the rigid [`Transform3`] that best aligns them. Correspondences are found by
//! brute-force nearest neighbour (bounded by a distance gate) and the incremental
//! transform is recovered with Horn's closed-form quaternion method. The largest
//! eigenvector of the 4x4 Horn matrix is found by fixed-iteration power
//! iteration, so the result is bit-for-bit reproducible.

use rne_math::{Quat, Transform3, Vec3};
use serde::{Deserialize, Serialize};

const POWER_ITERATIONS: usize = 64;

/// Configuration for [`Icp3d::align`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IcpConfig {
    /// Maximum refinement iterations.
    pub max_iterations: usize,
    /// Maximum correspondence distance in meters.
    pub max_correspondence_distance_m: f64,
    /// Translation change below which the alignment is converged, in meters.
    pub translation_tolerance_m: f64,
    /// Rotation change below which the alignment is converged, in radians.
    pub rotation_tolerance_rad: f64,
    /// Source point stride; `1` uses every point.
    pub source_stride: usize,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            max_correspondence_distance_m: 1.0,
            translation_tolerance_m: 1.0e-4,
            rotation_tolerance_rad: 1.0e-4,
            source_stride: 1,
        }
    }
}

impl IcpConfig {
    fn is_valid(&self) -> bool {
        self.max_iterations > 0
            && self.max_correspondence_distance_m.is_finite()
            && self.max_correspondence_distance_m > 0.0
            && self.translation_tolerance_m.is_finite()
            && self.translation_tolerance_m > 0.0
            && self.rotation_tolerance_rad.is_finite()
            && self.rotation_tolerance_rad > 0.0
            && self.source_stride > 0
    }
}

/// Errors raised by [`Icp3d::align`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum IcpError {
    /// The configuration was degenerate.
    #[error("invalid ICP configuration")]
    InvalidConfig,
    /// The source cloud was empty.
    #[error("source cloud is empty")]
    EmptySource,
    /// The target cloud was empty.
    #[error("target cloud is empty")]
    EmptyTarget,
    /// A point or the initial transform contained a non-finite value.
    #[error("ICP input contained a non-finite value")]
    NonFinite,
    /// The correspondence set was geometrically degenerate.
    #[error("correspondence set is degenerate")]
    Degenerate,
}

/// Outcome of an alignment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IcpResult {
    /// Transform that maps source points into the target frame.
    pub transform: Transform3,
    /// Refinement iterations performed.
    pub iterations: usize,
    /// Correspondences used in the final iteration.
    pub correspondences: usize,
    /// Whether the transform change fell below both tolerances.
    pub converged: bool,
    /// Mean correspondence residual in meters in the final iteration.
    pub mean_residual_m: f64,
}

/// A stateless 3D ICP aligner.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Icp3d;

impl Icp3d {
    /// Aligns `source` onto `target` starting from `initial`.
    ///
    /// The returned transform maps source points into the target frame.
    pub fn align(
        source: &[Vec3],
        target: &[Vec3],
        initial: Transform3,
        config: &IcpConfig,
    ) -> Result<IcpResult, IcpError> {
        if !config.is_valid() {
            return Err(IcpError::InvalidConfig);
        }
        if source.is_empty() {
            return Err(IcpError::EmptySource);
        }
        if target.is_empty() {
            return Err(IcpError::EmptyTarget);
        }
        if !transform_is_finite(&initial)
            || source.iter().any(|p| !p.is_finite())
            || target.iter().any(|p| !p.is_finite())
        {
            return Err(IcpError::NonFinite);
        }

        let sampled: Vec<Vec3> = source
            .iter()
            .step_by(config.source_stride)
            .copied()
            .collect();
        if sampled.is_empty() {
            return Err(IcpError::EmptySource);
        }

        let mut transform = initial;
        let mut iterations = 0;
        let mut correspondences = 0;
        let mut mean_residual_m = 0.0;
        let mut converged = false;

        for iteration in 0..config.max_iterations {
            iterations = iteration + 1;
            let transformed: Vec<Vec3> = sampled
                .iter()
                .map(|point| transform.transform_point(*point))
                .collect();

            let mut source_pairs: Vec<Vec3> = Vec::new();
            let mut target_pairs: Vec<Vec3> = Vec::new();
            let mut residual_sum = 0.0;
            for point in &transformed {
                match nearest(point, target) {
                    Some((nearest_point, distance)) => {
                        if distance <= config.max_correspondence_distance_m {
                            source_pairs.push(*point);
                            target_pairs.push(nearest_point);
                            residual_sum += distance;
                        }
                    }
                    None => return Err(IcpError::EmptyTarget),
                }
            }
            correspondences = source_pairs.len();
            if correspondences < 3 {
                return Ok(IcpResult {
                    transform,
                    iterations,
                    correspondences,
                    converged: false,
                    mean_residual_m,
                });
            }
            mean_residual_m = residual_sum / correspondences as f64;

            let source_centroid = centroid(&source_pairs);
            let target_centroid = centroid(&target_pairs);
            let covariance = cross_covariance(
                &source_pairs,
                &target_pairs,
                source_centroid,
                target_centroid,
            );
            let quaternion = horn_quaternion(&covariance).ok_or(IcpError::Degenerate)?;
            let rotation =
                Quat::from_xyzw(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
            let translation = target_centroid - rotation * source_centroid;
            let delta = Transform3::from_translation_rotation(translation, rotation);
            transform = delta.mul_transform(&transform);

            let rotation_delta = 2.0 * quaternion[0].abs().min(1.0).acos();
            if translation.length() <= config.translation_tolerance_m
                && rotation_delta <= config.rotation_tolerance_rad
            {
                converged = true;
                break;
            }
        }

        Ok(IcpResult {
            transform,
            iterations,
            correspondences,
            converged,
            mean_residual_m,
        })
    }
}

fn transform_is_finite(transform: &Transform3) -> bool {
    transform.translation.is_finite()
        && transform.rotation.is_finite()
        && transform.scale.is_finite()
}

/// Returns the nearest target point and its distance, or `None` for an empty target.
fn nearest(point: &Vec3, target: &[Vec3]) -> Option<(Vec3, f64)> {
    let mut best: Option<(Vec3, f64)> = None;
    for candidate in target {
        let distance = (point - candidate).length();
        if best
            .map(|(_, best_distance)| distance < best_distance)
            .unwrap_or(true)
        {
            best = Some((*candidate, distance));
        }
    }
    best
}

fn centroid(points: &[Vec3]) -> Vec3 {
    let mut sum = Vec3::ZERO;
    for point in points {
        sum += *point;
    }
    sum / points.len() as f64
}

fn cross_covariance(
    source: &[Vec3],
    target: &[Vec3],
    source_centroid: Vec3,
    target_centroid: Vec3,
) -> [[f64; 3]; 3] {
    let mut covariance = [[0.0; 3]; 3];
    for (source_point, target_point) in source.iter().zip(target) {
        let p = *source_point - source_centroid;
        let q = *target_point - target_centroid;
        let p = [p.x, p.y, p.z];
        let q = [q.x, q.y, q.z];
        for (row, p_row) in covariance.iter_mut().zip(p) {
            for (cell, q_column) in row.iter_mut().zip(q) {
                *cell += p_row * q_column;
            }
        }
    }
    covariance
}

/// Horn's quaternion from a 3x3 cross-covariance matrix, as `(w, x, y, z)`.
fn horn_quaternion(covariance: &[[f64; 3]; 3]) -> Option<[f64; 4]> {
    let sxx = covariance[0][0];
    let sxy = covariance[0][1];
    let sxz = covariance[0][2];
    let syx = covariance[1][0];
    let syy = covariance[1][1];
    let syz = covariance[1][2];
    let szx = covariance[2][0];
    let szy = covariance[2][1];
    let szz = covariance[2][2];

    let n = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ];
    power_eigenvector(&n)
}

/// Dominant eigenvector of a symmetric 4x4 matrix by fixed-iteration power method.
///
/// A Gershgorin shift makes every eigenvalue non-negative before iterating, so
/// the power method converges to the algebraically largest eigenvector of the
/// original matrix even when its most negative eigenvalue has the larger
/// magnitude.
fn power_eigenvector(matrix: &[[f64; 4]; 4]) -> Option<[f64; 4]> {
    let shift = matrix
        .iter()
        .map(|row| row.iter().map(|value| value.abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let mut shifted = *matrix;
    for (i, row) in shifted.iter_mut().enumerate() {
        row[i] += shift;
    }

    let mut vector = [1.0, 0.0, 0.0, 0.0];
    for _ in 0..POWER_ITERATIONS {
        let mut next = [0.0; 4];
        for (row, matrix_row) in shifted.iter().enumerate() {
            next[row] = matrix_row
                .iter()
                .zip(vector)
                .map(|(value, component)| value * component)
                .sum();
        }
        let norm = next.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !norm.is_finite() || norm < f64::MIN_POSITIVE {
            return None;
        }
        for (component, value) in vector.iter_mut().zip(next) {
            *component = value / norm;
        }
    }
    let norm = vector.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !norm.is_finite() || norm < f64::MIN_POSITIVE {
        return None;
    }
    for value in &mut vector {
        *value /= norm;
    }
    Some(vector)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    fn cloud() -> Vec<Vec3> {
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 0.5, 0.25),
            Vec3::new(0.3, -0.7, 0.9),
        ]
    }

    fn transform(source: &[Vec3], transform: &Transform3) -> Vec<Vec3> {
        source
            .iter()
            .map(|p| transform.transform_point(*p))
            .collect()
    }

    #[test]
    fn recovers_a_known_rotation_and_translation() {
        let source = cloud();
        let truth = Transform3::from_translation_rotation(
            Vec3::new(0.4, -0.6, 0.2),
            Quat::from_rotation_z(0.5),
        );
        let target = transform(&source, &truth);
        let result = Icp3d::align(
            &source,
            &target,
            Transform3::IDENTITY,
            &IcpConfig::default(),
        )
        .unwrap();
        assert!(result.converged);
        assert_relative_eq!(
            result.transform.translation.x,
            truth.translation.x,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            result.transform.translation.y,
            truth.translation.y,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            result.transform.translation.z,
            truth.translation.z,
            epsilon = 1e-6
        );
        // Quaternions are double covers: compare the rotated basis instead.
        let probe = Vec3::new(0.7, -0.2, 0.5);
        let expected = truth.rotation * probe;
        let actual = result.transform.rotation * probe;
        assert_relative_eq!(actual.x, expected.x, epsilon = 1e-6);
        assert_relative_eq!(actual.y, expected.y, epsilon = 1e-6);
        assert_relative_eq!(actual.z, expected.z, epsilon = 1e-6);
        assert!(result.mean_residual_m < 1e-6);
    }

    #[test]
    fn recovers_a_compound_rotation() {
        let source = cloud();
        let truth = Transform3::from_translation_rotation(
            Vec3::new(-0.3, 0.9, -0.4),
            Quat::from_rotation_x(0.3) * Quat::from_rotation_y(-0.2) * Quat::from_rotation_z(0.4),
        );
        let target = transform(&source, &truth);
        let initial = Transform3::from_translation_rotation(
            Vec3::new(0.03, -0.02, 0.01),
            Quat::from_rotation_z(-0.04),
        )
        .mul_transform(&truth);
        let result = Icp3d::align(&source, &target, initial, &IcpConfig::default()).unwrap();
        assert!(result.converged);
        let probe = Vec3::new(0.1, 0.7, -0.3);
        let expected = truth.rotation * probe;
        let actual = result.transform.rotation * probe;
        assert_relative_eq!(actual.x, expected.x, epsilon = 1e-5);
        assert_relative_eq!(actual.y, expected.y, epsilon = 1e-5);
        assert_relative_eq!(actual.z, expected.z, epsilon = 1e-5);
    }

    #[test]
    fn aligns_from_a_perturbed_initial_guess() {
        let source = cloud();
        let truth = Transform3::from_translation_rotation(
            Vec3::new(0.1, 0.05, 0.0),
            Quat::from_rotation_y(FRAC_PI_2 * 0.1),
        );
        let target = transform(&source, &truth);
        let initial = Transform3::from_translation_rotation(
            Vec3::new(-0.05, 0.02, 0.01),
            Quat::from_rotation_y(-0.05),
        );
        let result = Icp3d::align(&source, &target, initial, &IcpConfig::default()).unwrap();
        assert!(result.converged);
        let probe = Vec3::new(0.2, -0.4, 0.6);
        let expected = truth.rotation * probe + truth.translation;
        let actual = result.transform.rotation * probe + result.transform.translation;
        assert_relative_eq!(actual.x, expected.x, epsilon = 1e-5);
        assert_relative_eq!(actual.y, expected.y, epsilon = 1e-5);
        assert_relative_eq!(actual.z, expected.z, epsilon = 1e-5);
    }

    #[test]
    fn replay_is_bit_identical() {
        let source = cloud();
        let truth = Transform3::from_translation_rotation(
            Vec3::new(0.2, 0.1, -0.3),
            Quat::from_rotation_z(0.2),
        );
        let target = transform(&source, &truth);
        let run = || {
            Icp3d::align(
                &source,
                &target,
                Transform3::IDENTITY,
                &IcpConfig::default(),
            )
            .unwrap()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn rejects_bad_inputs() {
        assert_eq!(
            Icp3d::align(&[], &cloud(), Transform3::IDENTITY, &IcpConfig::default()),
            Err(IcpError::EmptySource)
        );
        assert_eq!(
            Icp3d::align(&cloud(), &[], Transform3::IDENTITY, &IcpConfig::default()),
            Err(IcpError::EmptyTarget)
        );
    }
}
