//! Joint-space trajectories produced by planners.

use crate::error::PlanningError;

/// One timed configuration along a trajectory.
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryPoint {
    /// Joint positions in degree-of-freedom order.
    pub positions: Vec<f64>,
    /// Time from the trajectory start in seconds.
    pub time_from_start_s: f64,
}

/// A timed sequence of joint configurations.
///
/// This is the RNE analogue of MoveIt's `RobotTrajectory`: a validated,
/// deterministic series of waypoints with monotonically non-decreasing
/// timestamps. An empty trajectory is valid.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RobotTrajectory {
    points: Vec<TrajectoryPoint>,
}

impl RobotTrajectory {
    /// Validates and wraps a sequence of timed points.
    pub fn new(points: Vec<TrajectoryPoint>) -> Result<Self, PlanningError> {
        let mut previous_time = 0.0;
        let mut expected_dof = None;
        for (index, point) in points.iter().enumerate() {
            if !point.time_from_start_s.is_finite()
                || point.positions.iter().any(|value| !value.is_finite())
            {
                return Err(PlanningError::Invalid(format!(
                    "trajectory point {index} contains a non-finite value"
                )));
            }
            match expected_dof {
                Some(expected) if expected != point.positions.len() => {
                    return Err(PlanningError::JointCountMismatch {
                        provided: point.positions.len(),
                        expected,
                    });
                }
                None => expected_dof = Some(point.positions.len()),
                _ => {}
            }
            if point.time_from_start_s < previous_time {
                return Err(PlanningError::Invalid(format!(
                    "trajectory time decreases at point {index}"
                )));
            }
            previous_time = point.time_from_start_s;
        }
        Ok(Self { points })
    }

    /// Builds a trajectory from equally spaced waypoints.
    pub fn from_waypoints(waypoints: &[Vec<f64>], dt_s: f64) -> Result<Self, PlanningError> {
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(PlanningError::Invalid(
                "waypoint duration must be finite and non-negative".to_string(),
            ));
        }
        let points = waypoints
            .iter()
            .enumerate()
            .map(|(index, positions)| TrajectoryPoint {
                positions: positions.clone(),
                time_from_start_s: index as f64 * dt_s,
            })
            .collect();
        Self::new(points)
    }

    /// Timed points in order.
    pub fn points(&self) -> &[TrajectoryPoint] {
        &self.points
    }

    /// Number of points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the trajectory has no points.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Time from the first to the last point in seconds.
    pub fn duration_s(&self) -> f64 {
        self.points
            .last()
            .map_or(0.0, |point| point.time_from_start_s)
    }

    /// Joint positions at the first point.
    pub fn start_positions(&self) -> Option<&[f64]> {
        self.points.first().map(|point| point.positions.as_slice())
    }

    /// Joint positions at the last point.
    pub fn goal_positions(&self) -> Option<&[f64]> {
        self.points.last().map(|point| point.positions.as_slice())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_from_waypoints_sets_monotonic_times() {
        let trajectory =
            RobotTrajectory::from_waypoints(&[vec![0.0, 0.0], vec![1.0, 0.0]], 0.5).unwrap();
        assert_eq!(trajectory.len(), 2);
        assert_eq!(trajectory.duration_s(), 0.5);
        assert_eq!(trajectory.start_positions(), Some([0.0, 0.0].as_slice()));
        assert_eq!(trajectory.goal_positions(), Some([1.0, 0.0].as_slice()));
    }

    #[test]
    fn rejects_decreasing_time_and_mismatched_dof() {
        let decreasing = RobotTrajectory::new(vec![
            TrajectoryPoint {
                positions: vec![0.0],
                time_from_start_s: 1.0,
            },
            TrajectoryPoint {
                positions: vec![0.0],
                time_from_start_s: 0.5,
            },
        ]);
        assert!(matches!(decreasing, Err(PlanningError::Invalid(_))));

        let mismatched = RobotTrajectory::new(vec![
            TrajectoryPoint {
                positions: vec![0.0],
                time_from_start_s: 0.0,
            },
            TrajectoryPoint {
                positions: vec![0.0, 0.0],
                time_from_start_s: 0.5,
            },
        ]);
        assert!(matches!(
            mismatched,
            Err(PlanningError::JointCountMismatch { .. })
        ));
    }
}
