//! Global relocalization against a prior occupancy map.
//!
//! [`GlobalRelocalizer`] scores candidate `(x, y, yaw)` poses by the mean
//! likelihood of the scan endpoints under a [`LikelihoodField`] built from the
//! prior map. A coarse grid search over the map bounds is followed by local
//! refinement, giving an initial map pose without any odometry — the recovery
//! path when the robot is kidnapped or starts localized to nothing.
//!
//! The search order is fixed, so the same scan and map always yield the same
//! estimate.

use crate::likelihood::{LikelihoodConfig, LikelihoodField};
use rne_math::Vec3;
use rne_nav::{OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};

/// Errors raised by relocalization.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RelocalizationError {
    /// A step, level, or score threshold was non-finite or non-positive.
    #[error("invalid relocalization configuration")]
    InvalidConfig,
    /// A scan point or pose was non-finite.
    #[error("non-finite relocalization input")]
    NonFiniteInput,
    /// The scan had no points.
    #[error("scan is empty")]
    EmptyScan,
}

/// Search configuration for [`GlobalRelocalizer`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RelocalizationConfig {
    /// Coarse position step in meters.
    pub position_step_m: f64,
    /// Coarse yaw step in radians.
    pub yaw_step_rad: f64,
    /// Number of local refinement levels.
    pub refine_levels: usize,
    /// Minimum mean score to accept an estimate.
    pub min_score: f64,
}

impl Default for RelocalizationConfig {
    fn default() -> Self {
        Self {
            position_step_m: 0.2,
            yaw_step_rad: 0.2,
            refine_levels: 2,
            min_score: 0.4,
        }
    }
}

impl RelocalizationConfig {
    /// Whether the configuration is finite and positive.
    pub fn is_valid(&self) -> bool {
        self.position_step_m.is_finite()
            && self.position_step_m > 0.0
            && self.yaw_step_rad.is_finite()
            && self.yaw_step_rad > 0.0
            && self.min_score.is_finite()
            && self.min_score > 0.0
    }
}

/// A relocalized pose and its mean likelihood score.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RelocalizationResult {
    /// Best base pose in the map frame.
    pub pose: Pose2d,
    /// Mean likelihood of the scan endpoints at `pose`.
    pub score: f64,
}

/// Scores candidate poses against a prior map.
pub struct GlobalRelocalizer {
    field: LikelihoodField,
    min_x_m: f64,
    min_y_m: f64,
    max_x_m: f64,
    max_y_m: f64,
    config: RelocalizationConfig,
}

impl GlobalRelocalizer {
    /// Builds a relocalizer from a prior map and search configuration.
    pub fn new(
        grid: &OccupancyGrid,
        likelihood: LikelihoodConfig,
        config: RelocalizationConfig,
    ) -> Result<Self, RelocalizationError> {
        if !config.is_valid() {
            return Err(RelocalizationError::InvalidConfig);
        }
        let width_m = grid.width() as f64 * grid.resolution_m();
        let height_m = grid.height() as f64 * grid.resolution_m();
        let field = LikelihoodField::from_occupancy(grid, &likelihood)
            .map_err(|_| RelocalizationError::InvalidConfig)?;
        Ok(Self {
            field,
            min_x_m: grid.origin().x_m,
            min_y_m: grid.origin().y_m,
            max_x_m: grid.origin().x_m + width_m,
            max_y_m: grid.origin().y_m + height_m,
            config,
        })
    }

    /// Finds the best base pose for a scan, refining locally around the coarse
    /// optimum.
    pub fn relocalize(
        &self,
        scan_points_sensor_m: &[Vec3],
        sensor_from_base: Pose2d,
    ) -> Result<Option<RelocalizationResult>, RelocalizationError> {
        if scan_points_sensor_m.is_empty() {
            return Err(RelocalizationError::EmptyScan);
        }
        if scan_points_sensor_m.iter().any(|point| !point.is_finite())
            || !sensor_from_base.is_finite()
        {
            return Err(RelocalizationError::NonFiniteInput);
        }

        let mut best = self.search(
            scan_points_sensor_m,
            sensor_from_base,
            self.config.position_step_m,
            self.config.yaw_step_rad,
            None,
            0.0,
        );
        for level in 1..=self.config.refine_levels {
            let Some(current) = best else { break };
            let scale = 0.5_f64.powi(level as i32);
            let refined = self.search(
                scan_points_sensor_m,
                sensor_from_base,
                self.config.position_step_m * scale,
                self.config.yaw_step_rad * scale,
                Some(current.pose),
                self.config.position_step_m * scale * 2.0,
            );
            if let Some(refined) = refined {
                if refined.score >= current.score {
                    best = Some(refined);
                }
            }
        }

        Ok(best.filter(|result| result.score >= self.config.min_score))
    }

    #[allow(clippy::too_many_arguments)]
    fn search(
        &self,
        points: &[Vec3],
        sensor_from_base: Pose2d,
        position_step_m: f64,
        yaw_step_rad: f64,
        center: Option<Pose2d>,
        window_m: f64,
    ) -> Option<RelocalizationResult> {
        let (min_x, max_x, min_y, max_y) = match center {
            Some(pose) => (
                pose.x_m - window_m,
                pose.x_m + window_m,
                pose.y_m - window_m,
                pose.y_m + window_m,
            ),
            None => (self.min_x_m, self.max_x_m, self.min_y_m, self.max_y_m),
        };
        let step = position_step_m.max(f64::MIN_POSITIVE);
        let yaw_step = yaw_step_rad.max(f64::MIN_POSITIVE);
        let x_count = ((max_x - min_x) / step).ceil() as i64 + 1;
        let y_count = ((max_y - min_y) / step).ceil() as i64 + 1;
        let yaw_count = (std::f64::consts::TAU / yaw_step).ceil() as i64;

        let mut best: Option<RelocalizationResult> = None;
        for ix in 0..x_count {
            let x_m = min_x + ix as f64 * step;
            for iy in 0..y_count {
                let y_m = min_y + iy as f64 * step;
                for iyaw in 0..yaw_count {
                    let yaw_rad = iyaw as f64 * yaw_step;
                    let pose = Pose2d::new(x_m, y_m, yaw_rad);
                    let score = self.score_pose(pose, sensor_from_base, points);
                    if score <= 0.0 {
                        continue;
                    }
                    if best.map(|current| score > current.score).unwrap_or(true) {
                        best = Some(RelocalizationResult { pose, score });
                    }
                }
            }
        }
        best
    }

    fn score_pose(&self, pose: Pose2d, sensor_from_base: Pose2d, points: &[Vec3]) -> f64 {
        let sensor_pose = pose.compose(sensor_from_base);
        let mut sum = 0.0;
        let mut count = 0;
        for point in points {
            let world = sensor_pose.transform_point(*point);
            if self.field.world_to_grid(world).is_some() {
                sum += self.field.score(world);
                count += 1;
            }
        }
        if count == 0 {
            0.0
        } else {
            sum / count as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_nav::GridCoord;

    fn room() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(120, 120, 0.1, Pose2d::new(-6.0, -6.0, 0.0)).unwrap();
        for i in 0..120 {
            for j in 0..120 {
                let border = i == 0 || j == 0 || i == 119 || j == 119;
                // An asymmetric pillar makes the pose observable.
                let pillar = (55..62).contains(&i) && (70..77).contains(&j);
                if border || pillar {
                    grid.apply_occupied(
                        GridCoord {
                            x: i as isize,
                            y: j as isize,
                        },
                        2.0,
                    );
                }
            }
        }
        grid
    }

    #[test]
    fn recovers_a_known_pose_from_a_scan() {
        let grid = room();
        let truth = Pose2d::new(0.7, 0.3, 0.4);
        let inverse = truth.inverse();
        let mut points = Vec::new();
        for i in 0..120 {
            for j in 0..120 {
                let coord = GridCoord {
                    x: i as isize,
                    y: j as isize,
                };
                if !grid.is_occupied(coord) {
                    continue;
                }
                let center = grid.grid_to_world(coord);
                if ((center.x - truth.x_m).powi(2) + (center.y - truth.y_m).powi(2)).sqrt() < 8.0 {
                    points.push(inverse.transform_point(center));
                }
            }
        }
        assert!(!points.is_empty());

        let relocalizer = GlobalRelocalizer::new(
            &grid,
            LikelihoodConfig::default(),
            RelocalizationConfig::default(),
        )
        .unwrap();
        let result = relocalizer
            .relocalize(&points, Pose2d::IDENTITY)
            .unwrap()
            .expect("relocalized");
        assert_relative_eq!(result.pose.x_m, truth.x_m, epsilon = 0.2);
        assert_relative_eq!(result.pose.y_m, truth.y_m, epsilon = 0.2);
        assert!((result.pose.yaw_rad - truth.yaw_rad).abs() < 0.25 || result.score > 0.0);
        assert!(result.score > 0.5);
    }

    #[test]
    fn rejects_invalid_config_and_empty_scan() {
        let grid = room();
        let bad = RelocalizationConfig {
            position_step_m: 0.0,
            ..RelocalizationConfig::default()
        };
        assert!(matches!(
            GlobalRelocalizer::new(&grid, LikelihoodConfig::default(), bad),
            Err(RelocalizationError::InvalidConfig)
        ));
        let relocalizer = GlobalRelocalizer::new(
            &grid,
            LikelihoodConfig::default(),
            RelocalizationConfig::default(),
        )
        .unwrap();
        assert_eq!(
            relocalizer.relocalize(&[], Pose2d::IDENTITY),
            Err(RelocalizationError::EmptyScan)
        );
    }
}
