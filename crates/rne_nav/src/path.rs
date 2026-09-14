//! Planar paths and geometric queries used by planners and controllers.

use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// An ordered list of planar waypoints.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Path2d {
    waypoints: Vec<Pose2d>,
}

/// Closest point on a path to a query position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPoint {
    /// Segment index containing the closest point.
    pub segment: usize,
    /// Interpolation factor along the segment in `[0, 1]`.
    pub t: f64,
    /// Closest point in world coordinates.
    pub point_m: Vec3,
    /// Distance from the query position to the closest point in meters.
    pub distance_m: f64,
    /// Arc length from the path start to the closest point in meters.
    pub arc_length_m: f64,
}

/// Error returned by empty-path operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PathError {
    /// The path has no waypoints.
    #[error("path has no waypoints")]
    EmptyPath,
}

impl Path2d {
    /// Creates a path from waypoints.
    pub fn new(waypoints: Vec<Pose2d>) -> Self {
        Self { waypoints }
    }

    /// Creates a path from points, deriving yaw from each segment direction.
    ///
    /// The final waypoint reuses the direction of the last segment; a single
    /// point yields a zero-yaw waypoint.
    pub fn from_points(points: &[Vec3]) -> Self {
        let mut waypoints: Vec<Pose2d> = Vec::with_capacity(points.len());
        for (index, point) in points.iter().enumerate() {
            let next = points.get(index + 1);
            let yaw = if let Some(next) = next {
                (next.y - point.y).atan2(next.x - point.x)
            } else if let Some(previous) = points.get(index.wrapping_sub(1)) {
                (point.y - previous.y).atan2(point.x - previous.x)
            } else {
                0.0
            };
            waypoints.push(Pose2d::new(point.x, point.y, yaw));
        }
        Self { waypoints }
    }

    /// Waypoints in order.
    pub fn waypoints(&self) -> &[Pose2d] {
        &self.waypoints
    }

    /// Number of waypoints.
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// Whether the path has no waypoints.
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Total arc length in meters.
    pub fn length_m(&self) -> f64 {
        self.waypoints
            .windows(2)
            .map(|window| {
                let dx = window[1].x_m - window[0].x_m;
                let dy = window[1].y_m - window[0].y_m;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }

    /// First waypoint.
    pub fn start(&self) -> Option<Pose2d> {
        self.waypoints.first().copied()
    }

    /// Last waypoint.
    pub fn goal(&self) -> Option<Pose2d> {
        self.waypoints.last().copied()
    }

    /// Waypoint at an index.
    pub fn position_at(&self, index: usize) -> Option<Pose2d> {
        self.waypoints.get(index).copied()
    }

    /// Finds the closest point on the path to a world position.
    pub fn closest_point(&self, position_m: Vec3) -> Option<ClosestPoint> {
        if self.waypoints.is_empty() {
            return None;
        }
        if self.waypoints.len() == 1 {
            let waypoint = self.waypoints[0];
            let point = Vec3::new(waypoint.x_m, waypoint.y_m, 0.0);
            return Some(ClosestPoint {
                segment: 0,
                t: 0.0,
                point_m: point,
                distance_m: (point - position_m).length(),
                arc_length_m: 0.0,
            });
        }

        let mut best: Option<ClosestPoint> = None;
        let mut accumulated = 0.0;
        for (segment, window) in self.waypoints.windows(2).enumerate() {
            let start = Vec3::new(window[0].x_m, window[0].y_m, 0.0);
            let end = Vec3::new(window[1].x_m, window[1].y_m, 0.0);
            let direction = end - start;
            let segment_length = direction.length();
            let t = if segment_length > 0.0 {
                ((position_m - start).dot(direction) / (segment_length * segment_length))
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            let point = start + direction * t;
            let distance = (point - position_m).length();
            let candidate = ClosestPoint {
                segment,
                t,
                point_m: point,
                distance_m: distance,
                arc_length_m: accumulated + t * segment_length,
            };
            let replace = best
                .as_ref()
                .map(|current| distance < current.distance_m)
                .unwrap_or(true);
            if replace {
                best = Some(candidate);
            }
            accumulated += segment_length;
        }
        best
    }

    /// Point at an arc length from the path start, clamped to the path.
    pub fn point_at_distance(&self, distance_m: f64) -> Option<Vec3> {
        if self.is_empty() {
            return None;
        }
        if self.waypoints.len() == 1 {
            let waypoint = self.waypoints[0];
            return Some(Vec3::new(waypoint.x_m, waypoint.y_m, 0.0));
        }
        if distance_m <= 0.0 {
            let waypoint = self.waypoints[0];
            return Some(Vec3::new(waypoint.x_m, waypoint.y_m, 0.0));
        }

        let mut remaining = distance_m;
        for window in self.waypoints.windows(2) {
            let start = Vec3::new(window[0].x_m, window[0].y_m, 0.0);
            let end = Vec3::new(window[1].x_m, window[1].y_m, 0.0);
            let segment_length = (end - start).length();
            if remaining <= segment_length {
                let fraction = if segment_length > 0.0 {
                    remaining / segment_length
                } else {
                    0.0
                };
                return Some(start + (end - start) * fraction);
            }
            remaining -= segment_length;
        }
        let goal = self.waypoints[self.waypoints.len() - 1];
        Some(Vec3::new(goal.x_m, goal.y_m, 0.0))
    }

    /// Removes waypoints that lie within `tolerance_m` of the line through
    /// their neighbours.
    pub fn prune_collinear(&self, tolerance_m: f64) -> Path2d {
        if self.waypoints.len() <= 2 {
            return self.clone();
        }
        let mut kept: Vec<Pose2d> = Vec::with_capacity(self.waypoints.len());
        kept.push(self.waypoints[0]);
        for window in self.waypoints.windows(3) {
            let a = Vec3::new(window[0].x_m, window[0].y_m, 0.0);
            let b = Vec3::new(window[1].x_m, window[1].y_m, 0.0);
            let c = Vec3::new(window[2].x_m, window[2].y_m, 0.0);
            let base = c - a;
            let base_length = base.length();
            let deviation = if base_length > 0.0 {
                (b - a).cross(base).length() / base_length
            } else {
                (b - a).length()
            };
            if deviation > tolerance_m {
                kept.push(window[1]);
            }
        }
        kept.push(self.waypoints[self.waypoints.len() - 1]);
        Path2d::new(kept)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn length_and_closest_point() {
        let path = Path2d::from_points(&[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 2.0, 0.0),
        ]);
        assert_relative_eq!(path.length_m(), 4.0, epsilon = 1e-12);
        let closest = path.closest_point(Vec3::new(1.0, 0.5, 0.0)).unwrap();
        assert_relative_eq!(closest.distance_m, 0.5, epsilon = 1e-12);
        assert_relative_eq!(closest.arc_length_m, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn point_at_distance_clamps() {
        let path = Path2d::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0)]);
        assert_relative_eq!(path.point_at_distance(1.5).unwrap().x, 1.5, epsilon = 1e-12);
        assert_relative_eq!(path.point_at_distance(9.0).unwrap().x, 3.0, epsilon = 1e-12);
    }

    #[test]
    fn collinear_points_are_pruned() {
        let path = Path2d::from_points(&[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);
        assert_eq!(path.prune_collinear(0.01).len(), 2);
    }
}
