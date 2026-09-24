//! Deterministic point-to-plane ICP and normal estimation.
//!
//! Point-to-plane reduces sliding along locally planar surfaces compared with
//! point-to-point ICP and is the scan-to-map update used by LiDAR-inertial
//! odometry. Correspondences are found through a deterministic voxel index;
//! each linearized step solves a 6x6 normal-equation system (with damping) for
//! a right-perturbation twist on SE(3). Normals are estimated by neighbourhood
//! PCA via power iteration, so the whole pipeline is bit-for-bit reproducible.

use crate::se3::Se3;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Configuration for [`IcpPointToPlane::align`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointToPlaneConfig {
    /// Maximum refinement iterations.
    pub max_iterations: usize,
    /// Maximum correspondence distance in meters.
    pub max_correspondence_distance_m: f64,
    /// Minimum correspondences required for a step.
    pub min_correspondences: usize,
    /// Voxel size of the target index in meters.
    pub target_voxel_size_m: f64,
    /// Twist step below which the alignment is converged, in radians/meters.
    pub step_tolerance: f64,
    /// Source point stride; `1` uses every point.
    pub source_stride: usize,
}

impl Default for PointToPlaneConfig {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            max_correspondence_distance_m: 1.0,
            min_correspondences: 10,
            target_voxel_size_m: 1.0,
            step_tolerance: 1.0e-6,
            source_stride: 1,
        }
    }
}

impl PointToPlaneConfig {
    fn is_valid(&self) -> bool {
        self.max_iterations > 0
            && self.max_correspondence_distance_m.is_finite()
            && self.max_correspondence_distance_m > 0.0
            && self.min_correspondences > 0
            && self.target_voxel_size_m.is_finite()
            && self.target_voxel_size_m > 0.0
            && self.step_tolerance.is_finite()
            && self.step_tolerance > 0.0
            && self.source_stride > 0
    }
}

/// Errors raised by point-to-plane registration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PointToPlaneError {
    /// The configuration was degenerate.
    #[error("invalid point-to-plane ICP configuration")]
    InvalidConfig,
    /// The source cloud was empty.
    #[error("source cloud is empty")]
    EmptySource,
    /// The target cloud was empty or its normals were missing.
    #[error("target cloud or normals are empty")]
    EmptyTarget,
    /// The number of target normals did not match the points.
    #[error("target normals length does not match the target cloud")]
    NormalLengthMismatch,
    /// A point, normal, or transform contained a non-finite value.
    #[error("point-to-plane ICP input contained a non-finite value")]
    NonFinite,
    /// The normal equations were singular even after damping.
    #[error("point-to-plane normal equations are singular")]
    SingularSystem,
}

/// Outcome of a point-to-plane alignment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointToPlaneResult {
    /// Transform that maps source points into the target frame.
    pub transform: Se3,
    /// Refinement iterations performed.
    pub iterations: usize,
    /// Correspondences used in the final iteration.
    pub correspondences: usize,
    /// Whether the step fell below the tolerance.
    pub converged: bool,
    /// Root-mean-square point-to-plane residual in meters.
    pub rmse_m: f64,
    /// Diagonal of the final normal-equation matrix (information proxy).
    pub information: [f64; 6],
}

/// A deterministic uniform voxel index over a point cloud.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelPointIndex {
    voxel_size_m: f64,
    points: Vec<Vec3>,
    buckets: BTreeMap<[i64; 3], Vec<usize>>,
}

impl VoxelPointIndex {
    /// Builds an index over `points` with the given voxel size.
    ///
    /// `voxel_size_m` is clamped to a positive value so a zero never divides.
    pub fn new(points: &[Vec3], voxel_size_m: f64) -> Self {
        let voxel_size_m = if voxel_size_m.is_finite() && voxel_size_m > 0.0 {
            voxel_size_m
        } else {
            1.0
        };
        let mut buckets: BTreeMap<[i64; 3], Vec<usize>> = BTreeMap::new();
        for (index, point) in points.iter().enumerate() {
            buckets
                .entry(key(*point, voxel_size_m))
                .or_default()
                .push(index);
        }
        Self {
            voxel_size_m,
            points: points.to_vec(),
            buckets,
        }
    }

    /// Number of indexed points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the index is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Nearest indexed point to `query`, returning its index and distance.
    pub fn nearest(&self, query: Vec3) -> Option<(usize, f64)> {
        let center = key(query, self.voxel_size_m);
        let mut best: Option<(usize, f64)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [center[0] + dx, center[1] + dy, center[2] + dz];
                    let Some(bucket) = self.buckets.get(&key) else {
                        continue;
                    };
                    for index in bucket {
                        let distance = (self.points[*index] - query).length();
                        if best
                            .map(|(_, best_distance)| distance < best_distance)
                            .unwrap_or(true)
                        {
                            best = Some((*index, distance));
                        }
                    }
                }
            }
        }
        best
    }

    /// Points within `radius_m` of `query`, as point positions.
    pub(crate) fn within_radius(&self, query: Vec3, radius_m: f64) -> Vec<Vec3> {
        let cells = (radius_m / self.voxel_size_m).ceil().max(1.0) as i64;
        let center = key(query, self.voxel_size_m);
        let radius_squared = radius_m * radius_m;
        let mut out = Vec::new();
        for dx in -cells..=cells {
            for dy in -cells..=cells {
                for dz in -cells..=cells {
                    let key = [center[0] + dx, center[1] + dy, center[2] + dz];
                    let Some(bucket) = self.buckets.get(&key) else {
                        continue;
                    };
                    for index in bucket {
                        let point = self.points[*index];
                        if (point - query).length_squared() <= radius_squared {
                            out.push(point);
                        }
                    }
                }
            }
        }
        out
    }
}

fn key(point: Vec3, voxel_size_m: f64) -> [i64; 3] {
    [
        (point.x / voxel_size_m).floor() as i64,
        (point.y / voxel_size_m).floor() as i64,
        (point.z / voxel_size_m).floor() as i64,
    ]
}

/// Estimates a unit normal per point from neighbours within `radius_m`.
///
/// The normal is the least-variance principal direction of the local covariance,
/// found as the largest eigenvector of `trace(C) I - C` by power iteration.
/// `None` is returned for points with too few neighbours or a degenerate
/// covariance (for example isolated points).
pub fn estimate_normals(points: &[Vec3], radius_m: f64) -> Vec<Option<Vec3>> {
    if points.is_empty() || !radius_m.is_finite() || radius_m <= 0.0 {
        return vec![None; points.len()];
    }
    let index = VoxelPointIndex::new(points, radius_m);
    points
        .iter()
        .map(|point| estimate_normal(point, &index, radius_m))
        .collect()
}

fn estimate_normal(point: &Vec3, index: &VoxelPointIndex, radius_m: f64) -> Option<Vec3> {
    let neighbors = index.within_radius(*point, radius_m);
    if neighbors.len() < 3 {
        return None;
    }
    let count = neighbors.len() as f64;
    let centroid = neighbors.iter().fold(Vec3::ZERO, |sum, p| sum + *p) / count;
    let mut covariance = [[0.0_f64; 3]; 3];
    for neighbor in &neighbors {
        let offset = *neighbor - centroid;
        let values = [offset.x, offset.y, offset.z];
        for row in 0..3 {
            for col in 0..3 {
                covariance[row][col] += values[row] * values[col];
            }
        }
    }
    let trace = covariance[0][0] + covariance[1][1] + covariance[2][2];
    // Largest eigenvector of `trace*I - C` is the smallest of `C`.
    let mut matrix = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            matrix[row][col] = if row == col { trace } else { 0.0 } - covariance[row][col];
        }
    }
    let normal = dominant_eigenvector(&matrix)?;
    let length = normal.length();
    if !length.is_finite() || length < 1.0e-9 {
        return None;
    }
    Some(normal / length)
}

fn dominant_eigenvector(matrix: &[[f64; 3]; 3]) -> Option<Vec3> {
    let shift = matrix
        .iter()
        .map(|row| row.iter().map(|value| value.abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let mut shifted = *matrix;
    for (i, row) in shifted.iter_mut().enumerate() {
        row[i] += shift;
    }
    // A generic start avoids a vanishing component when the dominant direction
    // is orthogonal to a coordinate axis (for example a flat floor's normal).
    let mut vector = [1.0_f64, 1.0, 1.0];
    let norm = vector.iter().map(|value| value * value).sum::<f64>().sqrt();
    for component in &mut vector {
        *component /= norm;
    }
    for _ in 0..64 {
        let mut next = [0.0_f64; 3];
        for (row, matrix_row) in shifted.iter().enumerate() {
            next[row] = matrix_row
                .iter()
                .zip(vector)
                .map(|(value, component)| value * component)
                .sum();
        }
        let norm = next.iter().map(|value| value * value).sum::<f64>().sqrt();
        if !norm.is_finite() || norm < f64::MIN_POSITIVE {
            return None;
        }
        for (component, value) in vector.iter_mut().zip(next) {
            *component = value / norm;
        }
    }
    Some(Vec3::new(vector[0], vector[1], vector[2]))
}

/// A stateless point-to-plane ICP aligner.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IcpPointToPlane;

impl IcpPointToPlane {
    /// Aligns `source` onto a `target` cloud with per-point `target_normals`.
    ///
    /// The returned transform maps source points into the target frame.
    pub fn align(
        source: &[Vec3],
        target: &[Vec3],
        target_normals: &[Vec3],
        initial: Se3,
        config: &PointToPlaneConfig,
    ) -> Result<PointToPlaneResult, PointToPlaneError> {
        if !config.is_valid() {
            return Err(PointToPlaneError::InvalidConfig);
        }
        if source.is_empty() {
            return Err(PointToPlaneError::EmptySource);
        }
        if target.is_empty() {
            return Err(PointToPlaneError::EmptyTarget);
        }
        if target_normals.len() != target.len() {
            return Err(PointToPlaneError::NormalLengthMismatch);
        }
        if !initial.is_finite()
            || source.iter().any(|point| !point.is_finite())
            || target.iter().any(|point| !point.is_finite())
            || target_normals.iter().any(|normal| !normal.is_finite())
        {
            return Err(PointToPlaneError::NonFinite);
        }

        let sampled: Vec<Vec3> = source
            .iter()
            .step_by(config.source_stride)
            .copied()
            .collect();
        let index = VoxelPointIndex::new(target, config.target_voxel_size_m);

        let mut transform = initial;
        let mut iterations = 0;
        let mut correspondences = 0;
        let mut rmse_m = 0.0;
        let mut converged = false;
        let mut information = [0.0; 6];

        for iteration in 0..config.max_iterations {
            iterations = iteration + 1;
            let mut h = [[0.0_f64; 6]; 6];
            let mut b = [0.0_f64; 6];
            let mut residual_squared = 0.0;
            let mut inliers = 0;
            for point in &sampled {
                let world = transform.transform_point(*point);
                let Some((nearest, distance)) = index.nearest(world) else {
                    return Err(PointToPlaneError::EmptyTarget);
                };
                if distance > config.max_correspondence_distance_m {
                    continue;
                }
                let normal_world = target_normals[nearest];
                let normal_length = normal_world.length();
                if !normal_length.is_finite() || normal_length < 1.0e-9 {
                    continue;
                }
                let normal_world = normal_world / normal_length;
                let normal_local = transform.rotation.conjugate().mul_vec3(normal_world);
                let residual = normal_world.dot(world - target[nearest]);
                let point_normal = point.cross(normal_local);
                let jacobian = [
                    point_normal.x,
                    point_normal.y,
                    point_normal.z,
                    normal_local.x,
                    normal_local.y,
                    normal_local.z,
                ];
                for row in 0..6 {
                    for col in 0..6 {
                        h[row][col] += jacobian[row] * jacobian[col];
                    }
                    // Gauss-Newton right-hand side is `-J^T r`.
                    b[row] -= jacobian[row] * residual;
                }
                residual_squared += residual * residual;
                inliers += 1;
            }
            correspondences = inliers;
            if inliers < config.min_correspondences {
                if iteration == 0 {
                    return Ok(PointToPlaneResult {
                        transform,
                        iterations,
                        correspondences,
                        converged: false,
                        rmse_m,
                        information,
                    });
                }
                break;
            }
            rmse_m = (residual_squared / inliers as f64).sqrt();
            information = [h[0][0], h[1][1], h[2][2], h[3][3], h[4][4], h[5][5]];

            let diagonal = 1.0e-6_f64;
            for (i, row) in h.iter_mut().enumerate() {
                row[i] += diagonal;
            }
            let delta = solve_cholesky_6(&h, &b).ok_or(PointToPlaneError::SingularSystem)?;
            transform = transform.compose(Se3::exp(delta));
            let max_step = delta
                .iter()
                .fold(0.0_f64, |max, value| max.max(value.abs()));
            if max_step <= config.step_tolerance {
                converged = true;
                break;
            }
        }

        Ok(PointToPlaneResult {
            transform,
            iterations,
            correspondences,
            converged,
            rmse_m,
            information,
        })
    }
}

/// Solves a symmetric positive-definite 6x6 system via Cholesky.
#[allow(clippy::needless_range_loop)]
fn solve_cholesky_6(h: &[[f64; 6]; 6], b: &[f64; 6]) -> Option<[f64; 6]> {
    const N: usize = 6;
    let mut lower = [[0.0_f64; N]; N];
    for row in 0..N {
        for col in 0..=row {
            let mut sum = h[row][col];
            for k in 0..col {
                sum -= lower[row][k] * lower[col][k];
            }
            if row == col {
                if sum <= 1.0e-18 {
                    return None;
                }
                lower[row][col] = sum.sqrt();
            } else {
                lower[row][col] = sum / lower[col][col];
            }
        }
    }
    let mut y = [0.0_f64; N];
    for row in 0..N {
        let mut sum = b[row];
        for k in 0..row {
            sum -= lower[row][k] * y[k];
        }
        y[row] = sum / lower[row][row];
    }
    let mut x = [0.0_f64; N];
    for row in (0..N).rev() {
        let mut sum = y[row];
        for k in (row + 1)..N {
            sum -= lower[k][row] * x[k];
        }
        x[row] = sum / lower[row][row];
    }
    if x.iter().all(|value| value.is_finite()) {
        Some(x)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Quat;

    /// A floor and two perpendicular walls, giving normals in three directions.
    fn scene() -> Vec<Vec3> {
        let mut points = Vec::new();
        for i in 0..25 {
            for j in 0..25 {
                points.push(Vec3::new(i as f64 * 0.2, 0.0, j as f64 * 0.2));
            }
        }
        for i in 0..25 {
            for k in 0..8 {
                points.push(Vec3::new(2.0, i as f64 * 0.2, k as f64 * 0.2));
            }
        }
        for i in 0..25 {
            for k in 0..8 {
                points.push(Vec3::new(i as f64 * 0.2, k as f64 * 0.2, 2.0));
            }
        }
        points
    }

    fn transform(points: &[Vec3], transform: Se3) -> Vec<Vec3> {
        points
            .iter()
            .map(|point| transform.transform_point(*point))
            .collect()
    }

    #[test]
    fn normals_point_out_of_planes() {
        let points = scene();
        let normals = estimate_normals(&points, 0.35);
        // Floor points should have a near-vertical normal.
        let floor_normal = normals[0].expect("floor normal");
        assert_relative_eq!(floor_normal.y.abs(), 1.0, epsilon = 1e-3);
    }

    #[test]
    fn recovers_a_known_small_motion() {
        let target = scene();
        let normals: Vec<Vec3> = estimate_normals(&target, 0.35)
            .into_iter()
            .map(|normal| normal.unwrap_or(Vec3::Y))
            .collect();
        let truth = Se3::new(Quat::from_rotation_y(0.05), Vec3::new(0.08, 0.0, -0.05));
        // Source is the scene pulled back by the truth, so aligning it recovers truth.
        let source = transform(&target, truth.inverse());
        let config = PointToPlaneConfig {
            max_correspondence_distance_m: 0.6,
            target_voxel_size_m: 0.4,
            ..PointToPlaneConfig::default()
        };
        let result = IcpPointToPlane::align(&source, &target, &normals, Se3::IDENTITY, &config)
            .expect("icp");
        assert!(result.converged, "did not converge");
        assert_relative_eq!(
            result.transform.translation.x,
            truth.translation.x,
            epsilon = 1e-3
        );
        assert_relative_eq!(
            result.transform.translation.z,
            truth.translation.z,
            epsilon = 1e-3
        );
        assert!(result.rmse_m < 1e-2, "rmse {}", result.rmse_m);
    }

    #[test]
    fn replay_is_bit_identical() {
        let target = scene();
        let normals: Vec<Vec3> = estimate_normals(&target, 0.35)
            .into_iter()
            .map(|normal| normal.unwrap_or(Vec3::Y))
            .collect();
        let truth = Se3::new(Quat::from_rotation_y(0.03), Vec3::new(0.05, 0.0, 0.02));
        let source = transform(&target, truth.inverse());
        let config = PointToPlaneConfig {
            target_voxel_size_m: 0.4,
            ..PointToPlaneConfig::default()
        };
        let run =
            || IcpPointToPlane::align(&source, &target, &normals, Se3::IDENTITY, &config).unwrap();
        assert_eq!(run(), run());
    }

    #[test]
    fn rejects_bad_inputs() {
        let points = scene();
        assert!(matches!(
            IcpPointToPlane::align(
                &[],
                &points,
                &points,
                Se3::IDENTITY,
                &PointToPlaneConfig::default()
            ),
            Err(PointToPlaneError::EmptySource)
        ));
        assert!(matches!(
            IcpPointToPlane::align(
                &points,
                &points,
                &points[1..],
                Se3::IDENTITY,
                &PointToPlaneConfig::default()
            ),
            Err(PointToPlaneError::NormalLengthMismatch)
        ));
    }
}
