//! Dynamic-window local planner for differential-drive bases.
//!
//! The planner samples the dynamic window of reachable linear and angular
//! velocities, rolls each candidate out with a constant-twist model, rejects
//! trajectories that cross lethal or unknown cells, and scores the survivors by
//! heading, clearance, path alignment, and speed. Sampling order and scoring are
//! deterministic, so a given state always yields the same command.

use crate::control::VelocityCommand2d;
use crate::costmap::{Costmap, COST_LETHAL, COST_NO_INFORMATION};
use crate::path::Path2d;
use crate::pose2d::Pose2d;
use std::f64::consts::{PI, TAU};
use thiserror::Error;

/// Dynamic-window local planner configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DwaConfig {
    /// Maximum forward velocity in meters per second.
    pub max_linear_m_s: f64,
    /// Minimum forward velocity in meters per second.
    pub min_linear_m_s: f64,
    /// Maximum yaw rate in radians per second.
    pub max_angular_rad_s: f64,
    /// Maximum linear acceleration in meters per second squared.
    pub max_linear_accel_m_s2: f64,
    /// Maximum angular acceleration in radians per second squared.
    pub max_angular_accel_rad_s2: f64,
    /// Number of linear velocity samples.
    pub linear_samples: usize,
    /// Number of angular velocity samples.
    pub angular_samples: usize,
    /// Rollout horizon in seconds.
    pub simulate_time_s: f64,
    /// Rollout integration step in seconds.
    pub simulation_step_s: f64,
    /// Goal distance tolerance in meters.
    pub goal_tolerance_m: f64,
    /// Lookahead distance for heading scoring in meters.
    pub heading_lookahead_m: f64,
    /// Weight of the heading score.
    pub heading_weight: f64,
    /// Weight of the clearance score.
    pub clearance_weight: f64,
    /// Weight of the velocity score.
    pub velocity_weight: f64,
    /// Weight of the path-alignment score.
    pub path_weight: f64,
    /// Whether unknown cells are treated as obstacles.
    pub allow_unknown: bool,
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            max_linear_m_s: 1.0,
            min_linear_m_s: 0.0,
            max_angular_rad_s: 2.0,
            max_linear_accel_m_s2: 1.0,
            max_angular_accel_rad_s2: 3.0,
            linear_samples: 7,
            angular_samples: 15,
            simulate_time_s: 1.0,
            simulation_step_s: 0.1,
            goal_tolerance_m: 0.1,
            heading_lookahead_m: 0.5,
            heading_weight: 1.0,
            clearance_weight: 1.0,
            velocity_weight: 0.5,
            path_weight: 1.0,
            allow_unknown: false,
        }
    }
}

/// Outcome of a local planning iteration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DwaOutcome {
    /// Best admissible command.
    Command(VelocityCommand2d),
    /// The goal tolerance has been reached.
    Reached,
}

/// Error returned by the local planner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum DwaError {
    /// The path has no waypoints.
    #[error("path has no waypoints")]
    EmptyPath,
    /// No sampled trajectory was admissible.
    #[error("no admissible trajectory in the dynamic window")]
    NoValidTrajectory,
    /// The configuration or state contained a non-finite value.
    #[error("local planner input must be finite")]
    NonFinite,
}

/// Dynamic-window local planner.
#[derive(Clone, Copy, Debug)]
pub struct DwaPlanner {
    config: DwaConfig,
}

impl DwaPlanner {
    /// Creates a planner with the given configuration.
    pub fn new(config: DwaConfig) -> Self {
        Self { config }
    }

    /// The planner configuration.
    pub fn config(&self) -> &DwaConfig {
        &self.config
    }

    /// Computes the best velocity command toward `path`.
    pub fn compute_command(
        &self,
        costmap: &Costmap,
        path: &Path2d,
        pose: Pose2d,
        current: VelocityCommand2d,
    ) -> Result<DwaOutcome, DwaError> {
        if path.is_empty() {
            return Err(DwaError::EmptyPath);
        }
        let config = &self.config;
        if !pose.is_finite()
            || !current.linear_m_s.is_finite()
            || !current.angular_rad_s.is_finite()
            || !config.simulate_time_s.is_finite()
            || config.simulate_time_s <= 0.0
            || config.simulation_step_s <= 0.0
        {
            return Err(DwaError::NonFinite);
        }

        let goal = path.goal().ok_or(DwaError::EmptyPath)?;
        let distance_to_goal =
            ((goal.x_m - pose.x_m).powi(2) + (goal.y_m - pose.y_m).powi(2)).sqrt();
        if distance_to_goal <= config.goal_tolerance_m {
            return Ok(DwaOutcome::Reached);
        }

        let closest = path
            .closest_point(rne_math::Vec3::new(pose.x_m, pose.y_m, 0.0))
            .ok_or(DwaError::EmptyPath)?;
        let target = path
            .point_at_distance(closest.arc_length_m + config.heading_lookahead_m)
            .unwrap_or(rne_math::Vec3::new(goal.x_m, goal.y_m, 0.0));

        let linear_window = config.max_linear_accel_m_s2 * config.simulate_time_s;
        let angular_window = config.max_angular_accel_rad_s2 * config.simulate_time_s;
        let v_lo = (current.linear_m_s - linear_window).max(config.min_linear_m_s);
        let v_hi = (current.linear_m_s + linear_window).min(config.max_linear_m_s);
        let omega_lo = (current.angular_rad_s - angular_window).max(-config.max_angular_rad_s);
        let omega_hi = (current.angular_rad_s + angular_window).min(config.max_angular_rad_s);

        let linear_values = sample_range(v_lo, v_hi, config.linear_samples);
        let angular_values = sample_range(omega_lo, omega_hi, config.angular_samples);
        let steps = ((config.simulate_time_s / config.simulation_step_s).round() as usize).max(1);

        let mut best: Option<(f64, VelocityCommand2d)> = None;
        for &v in &linear_values {
            for &omega in &angular_values {
                let Some(score) =
                    self.score_candidate(costmap, path, pose, v, omega, target, steps, config)
                else {
                    continue;
                };
                let replace = best
                    .as_ref()
                    .map(|(best_score, _)| score > *best_score)
                    .unwrap_or(true);
                if replace {
                    best = Some((score, VelocityCommand2d::new(v, omega)));
                }
            }
        }

        match best {
            Some((_, command)) => Ok(DwaOutcome::Command(command)),
            None => Err(DwaError::NoValidTrajectory),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn score_candidate(
        &self,
        costmap: &Costmap,
        path: &Path2d,
        pose: Pose2d,
        v: f64,
        omega: f64,
        target: rne_math::Vec3,
        steps: usize,
        config: &DwaConfig,
    ) -> Option<f64> {
        let mut x = pose.x_m;
        let mut y = pose.y_m;
        let mut yaw = pose.yaw_rad;
        let dt = config.simulation_step_s;
        let mut min_cost = u8::MAX;
        for _ in 0..steps {
            x += v * yaw.cos() * dt;
            y += v * yaw.sin() * dt;
            yaw = wrap_angle(yaw + omega * dt);
            let coord = costmap.world_to_grid(rne_math::Vec3::new(x, y, 0.0))?;
            let cost = costmap.cost_at(coord)?;
            if cost == COST_LETHAL || (cost == COST_NO_INFORMATION && !config.allow_unknown) {
                return None;
            }
            min_cost = min_cost.min(cost);
        }

        let heading_error =
            wrap_angle((target.y - pose.y_m).atan2(target.x - pose.x_m) - pose.yaw_rad);
        let heading_score = 0.5 * (1.0 + heading_error.cos());
        let clearance_score = 1.0 - min_cost as f64 / COST_LETHAL as f64;
        let velocity_score = if config.max_linear_m_s > 0.0 {
            v / config.max_linear_m_s
        } else {
            0.0
        };
        let path_distance = path
            .closest_point(rne_math::Vec3::new(x, y, 0.0))
            .map(|point| point.distance_m)
            .unwrap_or(0.0);
        let path_score = 1.0 / (1.0 + path_distance);

        Some(
            config.heading_weight * heading_score
                + config.clearance_weight * clearance_score
                + config.velocity_weight * velocity_score
                + config.path_weight * path_score,
        )
    }
}

fn sample_range(low: f64, high: f64, samples: usize) -> Vec<f64> {
    let samples = samples.max(1);
    if samples == 1 {
        return vec![high];
    }
    let span = high - low;
    (0..samples)
        .map(|index| low + span * index as f64 / (samples - 1) as f64)
        .collect()
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::{CostmapConfig, COST_LETHAL};
    use crate::grid::{GridCoord, OccupancyGrid};

    fn open_costmap() -> Costmap {
        let mut grid = OccupancyGrid::new(40, 40, 0.1, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
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
        Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap()
    }

    #[test]
    fn accelerates_toward_a_straight_path() {
        let costmap = open_costmap();
        let path = Path2d::from_points(&[
            rne_math::Vec3::new(0.0, 0.0, 0.0),
            rne_math::Vec3::new(1.5, 0.0, 0.0),
        ]);
        let planner = DwaPlanner::new(DwaConfig::default());
        let outcome = planner
            .compute_command(&costmap, &path, Pose2d::IDENTITY, VelocityCommand2d::ZERO)
            .unwrap();
        match outcome {
            DwaOutcome::Command(command) => assert!(command.linear_m_s > 0.0),
            DwaOutcome::Reached => panic!("should not be at goal"),
        }
    }

    #[test]
    fn reports_reached_at_the_goal() {
        let costmap = open_costmap();
        let path = Path2d::from_points(&[
            rne_math::Vec3::new(0.0, 0.0, 0.0),
            rne_math::Vec3::new(0.1, 0.0, 0.0),
        ]);
        let planner = DwaPlanner::new(DwaConfig::default());
        assert_eq!(
            planner.compute_command(
                &costmap,
                &path,
                Pose2d::new(0.05, 0.0, 0.0),
                VelocityCommand2d::ZERO
            ),
            Ok(DwaOutcome::Reached)
        );
    }

    #[test]
    fn avoids_lethal_cells() {
        let mut grid = OccupancyGrid::new(60, 40, 0.1, Pose2d::new(-3.0, -2.0, 0.0)).unwrap();
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
        // Wall directly ahead at x = 0.5 m spanning the path corridor.
        for y in 15..25 {
            grid.reset(GridCoord {
                x: 35,
                y: y as isize,
            });
            grid.mark_occupied(GridCoord {
                x: 35,
                y: y as isize,
            });
        }
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        let path = Path2d::from_points(&[
            rne_math::Vec3::new(-1.0, 0.0, 0.0),
            rne_math::Vec3::new(2.0, 0.0, 0.0),
        ]);
        let planner = DwaPlanner::new(DwaConfig::default());
        let outcome = planner
            .compute_command(
                &costmap,
                &path,
                Pose2d::new(-1.0, 0.0, 0.0),
                VelocityCommand2d::ZERO,
            )
            .unwrap();
        match outcome {
            DwaOutcome::Command(command) => {
                // The chosen rollout must not cross a lethal cell.
                let mut x = -1.0;
                let mut y = 0.0;
                let mut yaw: f64 = 0.0;
                for _ in 0..10 {
                    x += command.linear_m_s * yaw.cos() * 0.1;
                    y += command.linear_m_s * yaw.sin() * 0.1;
                    yaw += command.angular_rad_s * 0.1;
                    let coord = costmap
                        .world_to_grid(rne_math::Vec3::new(x, y, 0.0))
                        .unwrap();
                    assert_ne!(costmap.cost_at(coord), Some(COST_LETHAL));
                }
            }
            DwaOutcome::Reached => panic!("should not be at goal"),
        }
    }
}
