//! Path following and velocity commands.

use crate::path::{ClosestPoint, Path2d, PathError};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// A planar velocity command for a differential-drive base.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VelocityCommand2d {
    /// Forward velocity in meters per second.
    pub linear_m_s: f64,
    /// Yaw rate in radians per second.
    pub angular_rad_s: f64,
}

impl VelocityCommand2d {
    /// A zero command.
    pub const ZERO: Self = Self {
        linear_m_s: 0.0,
        angular_rad_s: 0.0,
    };

    /// Creates a command.
    pub fn new(linear_m_s: f64, angular_rad_s: f64) -> Self {
        Self {
            linear_m_s,
            angular_rad_s,
        }
    }
}

/// Pure-pursuit path-following configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PurePursuitConfig {
    /// Lookahead distance in meters.
    pub lookahead_m: f64,
    /// Maximum forward velocity in meters per second.
    pub max_linear_m_s: f64,
    /// Maximum yaw rate in radians per second.
    pub max_angular_rad_s: f64,
    /// Goal distance tolerance in meters.
    pub goal_tolerance_m: f64,
    /// Distance over which the robot decelerates to a stop at the goal.
    pub slow_radius_m: f64,
}

impl Default for PurePursuitConfig {
    fn default() -> Self {
        Self {
            lookahead_m: 0.5,
            max_linear_m_s: 1.0,
            max_angular_rad_s: 2.0,
            goal_tolerance_m: 0.1,
            slow_radius_m: 0.75,
        }
    }
}

/// Result of a pure-pursuit iteration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FollowResult {
    /// Commanded velocity.
    pub command: VelocityCommand2d,
    /// Remaining distance to the final waypoint in meters.
    pub distance_to_goal_m: f64,
    /// Whether the goal tolerance has been reached.
    pub reached: bool,
    /// Closest path point to the current pose.
    pub closest: ClosestPoint,
}

/// Computes a differential-drive command that follows a path.
pub fn pure_pursuit_follow(
    path: &Path2d,
    pose: Pose2d,
    config: &PurePursuitConfig,
) -> Result<FollowResult, PathError> {
    let goal = path.goal().ok_or(PathError::EmptyPath)?;
    let closest = path
        .closest_point(Vec3::new(pose.x_m, pose.y_m, 0.0))
        .ok_or(PathError::EmptyPath)?;

    let distance_to_goal_m = ((goal.x_m - pose.x_m).powi(2) + (goal.y_m - pose.y_m).powi(2)).sqrt();
    let reached = distance_to_goal_m <= config.goal_tolerance_m;

    let lookahead_m = config.lookahead_m.max(1.0e-6);
    let lookahead_point = path
        .point_at_distance(closest.arc_length_m + lookahead_m)
        .unwrap_or(Vec3::new(goal.x_m, goal.y_m, 0.0));

    let dx = lookahead_point.x - pose.x_m;
    let dy = lookahead_point.y - pose.y_m;
    let (sin, cos) = pose.yaw_rad.sin_cos();
    let local_y = -sin * dx + cos * dy;
    let distance = (dx * dx + dy * dy).sqrt();

    if reached {
        return Ok(FollowResult {
            command: VelocityCommand2d::ZERO,
            distance_to_goal_m,
            reached: true,
            closest,
        });
    }

    let command = if distance <= 1.0e-6 {
        VelocityCommand2d::ZERO
    } else {
        let curvature = 2.0 * local_y / (distance * distance);
        let turn_scale = (1.0 - curvature.abs().min(1.0)).clamp(0.2, 1.0);
        let mut linear = config.max_linear_m_s * turn_scale;
        if config.slow_radius_m > 0.0 && distance_to_goal_m < config.slow_radius_m {
            linear *= (distance_to_goal_m / config.slow_radius_m).clamp(0.0, 1.0);
        }
        let angular =
            (linear * curvature).clamp(-config.max_angular_rad_s, config.max_angular_rad_s);
        VelocityCommand2d::new(linear.max(0.0), angular)
    };

    Ok(FollowResult {
        command,
        distance_to_goal_m,
        reached: false,
        closest,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn drives_straight_toward_a_far_waypoint() {
        let path = Path2d::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 0.0)]);
        let result =
            pure_pursuit_follow(&path, Pose2d::IDENTITY, &PurePursuitConfig::default()).unwrap();
        assert!(result.command.linear_m_s > 0.0);
        assert_relative_eq!(result.command.angular_rad_s, 0.0, epsilon = 1e-9);
        assert!(!result.reached);
    }

    #[test]
    fn reaches_the_goal() {
        let path = Path2d::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0)]);
        let result = pure_pursuit_follow(
            &path,
            Pose2d::new(0.45, 0.0, 0.0),
            &PurePursuitConfig::default(),
        )
        .unwrap();
        assert!(result.reached);
        assert_eq!(result.command, VelocityCommand2d::ZERO);
    }

    #[test]
    fn turns_toward_an_offset_lookahead_point() {
        let path = Path2d::from_points(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(5.0, 3.0, 0.0)]);
        let result =
            pure_pursuit_follow(&path, Pose2d::IDENTITY, &PurePursuitConfig::default()).unwrap();
        assert!(result.command.angular_rad_s > 0.0);
    }
}
