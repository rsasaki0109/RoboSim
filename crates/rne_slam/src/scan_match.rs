//! Correlative scan matching against a [`LikelihoodField`].

use crate::likelihood::LikelihoodField;
use rne_math::Vec3;
use rne_nav::{LaserScan2d, Pose2d};
use serde::{Deserialize, Serialize};

/// Converts a scan into sensor-frame 2D endpoints, downsampled to `max_beams`.
pub fn scan_points_2d(scan: &LaserScan2d, max_beams: usize) -> Vec<Vec3> {
    if scan.ranges_m.is_empty() {
        return Vec::new();
    }
    let max_beams = max_beams.max(1);
    let stride = scan.beam_count().div_ceil(max_beams).max(1);
    let mut points = Vec::new();
    let mut index = 0;
    while index < scan.ranges_m.len() {
        let range = scan.ranges_m[index];
        if range.is_finite() && range >= scan.range_min_m && range <= scan.range_max_m {
            let angle = scan.angle_min_rad + scan.angle_increment_rad * index as f64;
            points.push(Vec3::new(angle.cos() * range, angle.sin() * range, 0.0));
        }
        index += stride;
    }
    points
}

/// Correlative scan match configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScanMatchConfig {
    /// Coarse linear search half-window in meters.
    pub linear_window_m: f64,
    /// Coarse angular search half-window in radians.
    pub angular_window_rad: f64,
    /// Coarse-to-fine refinement levels.
    pub levels: usize,
    /// Linear samples per search axis (x and y).
    pub linear_samples: usize,
    /// Angular samples per search.
    pub angular_samples: usize,
    /// Minimum number of in-field beams required to accept a match.
    pub min_valid_beams: usize,
    /// Max-pooling factor for the coarse first search level; one disables it.
    pub coarse_downsample_factor: usize,
}

impl Default for ScanMatchConfig {
    fn default() -> Self {
        Self {
            linear_window_m: 0.30,
            angular_window_rad: 0.20,
            levels: 3,
            linear_samples: 5,
            angular_samples: 5,
            min_valid_beams: 10,
            coarse_downsample_factor: 1,
        }
    }
}

/// Result of a scan match.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanMatchResult {
    /// Best base pose in the map frame.
    pub pose: Pose2d,
    /// Mean likelihood per valid beam in `[0, 1]`.
    pub score: f64,
    /// Number of pose candidates evaluated.
    pub evaluated: usize,
}

/// Correlative scan matcher.
#[derive(Clone, Copy, Debug)]
pub struct ScanMatcher {
    config: ScanMatchConfig,
}

impl ScanMatcher {
    /// Creates a matcher with the given configuration.
    pub fn new(config: ScanMatchConfig) -> Self {
        Self { config }
    }

    /// The matcher configuration.
    pub fn config(&self) -> &ScanMatchConfig {
        &self.config
    }

    /// Refines a base pose so sensor points align with the likelihood field.
    pub fn match_scan(
        &self,
        field: &LikelihoodField,
        points_sensor: &[Vec3],
        sensor_from_base: Pose2d,
        initial_base: Pose2d,
    ) -> Option<ScanMatchResult> {
        if points_sensor.is_empty() {
            return None;
        }
        let config = &self.config;
        let mut best_pose = initial_base;
        let mut best_score = f64::NEG_INFINITY;
        let mut window_linear = config.linear_window_m;
        let mut window_angular = config.angular_window_rad;
        let mut evaluated = 0;
        let coarse = (config.coarse_downsample_factor > 1)
            .then(|| field.downsample(config.coarse_downsample_factor));

        for level in 0..config.levels.max(1) {
            let eval_field: &LikelihoodField = if level == 0 {
                coarse.as_ref().unwrap_or(field)
            } else {
                field
            };
            let linear_step = if config.linear_samples > 1 {
                2.0 * window_linear / (config.linear_samples - 1) as f64
            } else {
                0.0
            };
            let angular_step = if config.angular_samples > 1 {
                2.0 * window_angular / (config.angular_samples - 1) as f64
            } else {
                0.0
            };

            let mut level_score = f64::NEG_INFINITY;
            let mut level_pose = best_pose;
            for i in 0..config.linear_samples {
                let dx = -window_linear + i as f64 * linear_step;
                for j in 0..config.linear_samples {
                    let dy = -window_linear + j as f64 * linear_step;
                    for k in 0..config.angular_samples {
                        let dtheta = -window_angular + k as f64 * angular_step;
                        let candidate = best_pose.compose(Pose2d::new(dx, dy, dtheta));
                        let score =
                            score_pose(eval_field, points_sensor, sensor_from_base, candidate);
                        evaluated += 1;
                        if score > level_score {
                            level_score = score;
                            level_pose = candidate;
                        }
                    }
                }
            }

            if level_score < 0.0 {
                break;
            }
            best_pose = level_pose;
            best_score = level_score;
            window_linear = linear_step.max(1.0e-3);
            window_angular = angular_step.max(1.0e-4);
        }

        if best_score < 0.0 {
            return None;
        }
        Some(ScanMatchResult {
            pose: best_pose,
            score: best_score,
            evaluated,
        })
    }
}

fn score_pose(
    field: &LikelihoodField,
    points_sensor: &[Vec3],
    sensor_from_base: Pose2d,
    base: Pose2d,
) -> f64 {
    let sensor_pose = base.compose(sensor_from_base);
    let mut total = 0.0;
    let mut valid = 0;
    for point in points_sensor {
        let world = sensor_pose.transform_point(*point);
        if field.world_to_grid(world).is_some() {
            total += field.score(world);
            valid += 1;
        }
    }
    if valid == 0 {
        -1.0
    } else {
        total / valid as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::likelihood::LikelihoodConfig;
    use rne_nav::{GridCoord, OccupancyGrid};
    use std::f64::consts::TAU;

    fn room_grid() -> OccupancyGrid {
        let resolution = 0.05;
        let mut grid = OccupancyGrid::new(
            (12.0 / resolution) as usize,
            (8.0 / resolution) as usize,
            resolution,
            Pose2d::new(-6.0, -4.0, 0.0),
        )
        .unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        for (min_x, min_y, max_x, max_y) in [
            (-5.0, -3.0, -5.0, 3.0),
            (5.0, -3.0, 5.0, 3.0),
            (-5.0, -3.0, 5.0, -3.0),
            (-5.0, 3.0, 5.0, 3.0),
        ] {
            let steps = 200;
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let world = Vec3::new(
                    min_x + (max_x - min_x) * t,
                    min_y + (max_y - min_y) * t,
                    0.0,
                );
                if let Some(coord) = grid.world_to_grid(world) {
                    grid.reset(coord);
                    grid.mark_occupied(coord);
                }
            }
        }
        grid
    }

    fn room_scan(x_m: f64, y_m: f64) -> LaserScan2d {
        let beams = 360;
        let mut ranges = Vec::with_capacity(beams);
        for beam in 0..beams {
            let angle = TAU * beam as f64 / beams as f64;
            ranges.push(ray_to_wall(x_m, y_m, angle));
        }
        LaserScan2d {
            time_s: 0.0,
            frame: rne_nav::FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 30.0,
            ranges_m: ranges,
        }
    }

    fn ray_to_wall(x: f64, y: f64, angle: f64) -> f64 {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for (bound, position, direction) in
            [(5.0, x, dx), (-5.0, x, dx), (3.0, y, dy), (-3.0, y, dy)]
        {
            if direction.abs() > 1.0e-9 {
                let t = (bound - position) / direction;
                if t > 0.0 {
                    best = best.min(t);
                }
            }
        }
        best
    }

    #[test]
    fn recovers_a_perturbed_pose() {
        let grid = room_grid();
        let field = LikelihoodField::from_occupancy(&grid, &LikelihoodConfig::default()).unwrap();
        let truth = Pose2d::new(1.0, 0.5, 0.3);
        let scan = room_scan(truth.x_m, truth.y_m);
        let points = scan_points_2d(&scan, 180);
        let matcher = ScanMatcher::new(ScanMatchConfig::default());
        let initial = Pose2d::new(truth.x_m + 0.15, truth.y_m - 0.1, truth.yaw_rad + 0.08);
        let result = matcher
            .match_scan(&field, &points, Pose2d::IDENTITY, initial)
            .unwrap();
        assert!(
            (result.pose.x_m - truth.x_m).abs() < 0.1 && (result.pose.y_m - truth.y_m).abs() < 0.1,
            "pose={:?}",
            result.pose
        );
        assert!(result.score > 0.5, "score={}", result.score);
    }

    #[test]
    fn multi_resolution_match_recovers_pose() {
        let grid = room_grid();
        let field = LikelihoodField::from_occupancy(&grid, &LikelihoodConfig::default()).unwrap();
        let truth = Pose2d::new(0.5, -0.5, -0.2);
        let scan = room_scan(truth.x_m, truth.y_m);
        let points = scan_points_2d(&scan, 180);
        let matcher = ScanMatcher::new(ScanMatchConfig {
            coarse_downsample_factor: 3,
            ..ScanMatchConfig::default()
        });
        let initial = Pose2d::new(truth.x_m + 0.2, truth.y_m + 0.15, truth.yaw_rad - 0.05);
        let result = matcher
            .match_scan(&field, &points, Pose2d::IDENTITY, initial)
            .unwrap();
        assert!(
            (result.pose.x_m - truth.x_m).abs() < 0.15
                && (result.pose.y_m - truth.y_m).abs() < 0.15,
            "pose={:?}",
            result.pose
        );
    }

    #[test]
    fn empty_points_do_not_match() {
        let grid = room_grid();
        let field = LikelihoodField::from_occupancy(&grid, &LikelihoodConfig::default()).unwrap();
        let matcher = ScanMatcher::new(ScanMatchConfig::default());
        assert!(matcher
            .match_scan(&field, &[], Pose2d::IDENTITY, Pose2d::IDENTITY)
            .is_none());
    }
}
