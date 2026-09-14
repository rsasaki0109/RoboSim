//! Deterministic sampling-based local avoidance for multiple robots.
//!
//! This is a lightweight reciprocal sense-and-avoid layer: candidate velocity
//! commands are rolled out against predicted circular obstacles and the closest
//! collision-free command to the desired one is selected. It is not a full
//! ORCA solver, but it is deterministic and safe and composes with [`crate::DwaPlanner`].

use crate::control::VelocityCommand2d;
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// A moving circular obstacle (another robot) used for avoidance.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CircularObstacle {
    /// Current center in meters.
    pub center_m: Vec3,
    /// Current velocity in meters per second.
    pub velocity_m_s: Vec3,
    /// Collision radius in meters.
    pub radius_m: f64,
}

/// Avoidance sampling configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AvoidanceConfig {
    /// Prediction horizon in seconds.
    pub time_horizon_s: f64,
    /// Rollout integration step in seconds.
    pub simulation_step_s: f64,
    /// Number of forward velocity samples over `[0, max_linear]`.
    pub linear_samples: usize,
    /// Number of yaw-rate samples over `[-max_angular, max_angular]`.
    pub angular_samples: usize,
    /// Extra clearance in meters added to both radii.
    pub safety_margin_m: f64,
}

impl Default for AvoidanceConfig {
    fn default() -> Self {
        Self {
            time_horizon_s: 2.0,
            simulation_step_s: 0.2,
            linear_samples: 11,
            angular_samples: 21,
            safety_margin_m: 0.1,
        }
    }
}

/// Selects a collision-free command close to `desired`, or stops if none exists.
pub fn avoid_velocities(
    pose: Pose2d,
    self_radius_m: f64,
    obstacles: &[CircularObstacle],
    desired: VelocityCommand2d,
    max_linear_m_s: f64,
    max_angular_rad_s: f64,
    config: &AvoidanceConfig,
) -> VelocityCommand2d {
    let linear_samples = config.linear_samples.max(2);
    let angular_samples = config.angular_samples.max(2);
    let steps = ((config.time_horizon_s / config.simulation_step_s).round() as usize).max(1);
    let mut best: Option<(f64, VelocityCommand2d)> = None;

    for i in 0..linear_samples {
        let linear = max_linear_m_s * i as f64 / (linear_samples - 1) as f64;
        for j in 0..angular_samples {
            let angular = -max_angular_rad_s
                + 2.0 * max_angular_rad_s * j as f64 / (angular_samples - 1) as f64;
            let command = VelocityCommand2d::new(linear, angular);
            if rollout_collides(
                pose,
                command,
                self_radius_m,
                obstacles,
                steps,
                config.simulation_step_s,
                config.safety_margin_m,
            ) {
                continue;
            }
            let cost = (linear - desired.linear_m_s).abs() / max_linear_m_s.max(1.0e-9)
                + (angular - desired.angular_rad_s).abs() / max_angular_rad_s.max(1.0e-9);
            let replace = best
                .as_ref()
                .map(|(best_cost, _)| cost < *best_cost)
                .unwrap_or(true);
            if replace {
                best = Some((cost, command));
            }
        }
    }

    best.map(|(_, command)| command)
        .unwrap_or(VelocityCommand2d::ZERO)
}

/// Whether rolling out `command` collides with any obstacle within the horizon.
pub fn rollout_collides(
    pose: Pose2d,
    command: VelocityCommand2d,
    self_radius_m: f64,
    obstacles: &[CircularObstacle],
    steps: usize,
    dt: f64,
    safety_margin_m: f64,
) -> bool {
    let mut x = pose.x_m;
    let mut y = pose.y_m;
    let mut yaw = pose.yaw_rad;
    for step in 1..=steps {
        x += command.linear_m_s * yaw.cos() * dt;
        y += command.linear_m_s * yaw.sin() * dt;
        yaw += command.angular_rad_s * dt;
        let time = step as f64 * dt;
        for obstacle in obstacles {
            let predicted = obstacle.center_m + obstacle.velocity_m_s * time;
            let minimum = self_radius_m + obstacle.radius_m + safety_margin_m;
            let dx = x - predicted.x;
            let dy = y - predicted.y;
            if dx * dx + dy * dy < minimum * minimum {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn avoids_a_stationary_robot_ahead() {
        // A stationary obstacle 0.5 m ahead; the desired forward command would
        // reach it, so the sampler must pick a collision-free alternative.
        let obstacles = [CircularObstacle {
            center_m: Vec3::new(0.5, 0.0, 0.0),
            velocity_m_s: Vec3::ZERO,
            radius_m: 0.2,
        }];
        let config = AvoidanceConfig::default();
        let command = avoid_velocities(
            Pose2d::IDENTITY,
            0.2,
            &obstacles,
            VelocityCommand2d::new(0.5, 0.0),
            0.5,
            1.0,
            &config,
        );
        let steps = 10;
        assert!(
            !rollout_collides(
                Pose2d::IDENTITY,
                command,
                0.2,
                &obstacles,
                steps,
                config.simulation_step_s,
                config.safety_margin_m,
            ),
            "chosen command {command:?} collides"
        );
        // Deterministic.
        let again = avoid_velocities(
            Pose2d::IDENTITY,
            0.2,
            &obstacles,
            VelocityCommand2d::new(0.5, 0.0),
            0.5,
            1.0,
            &config,
        );
        assert_eq!(command, again);
    }

    #[test]
    fn follows_the_desired_command_when_clear() {
        let desired = VelocityCommand2d::new(0.4, 0.1);
        let command = avoid_velocities(
            Pose2d::IDENTITY,
            0.2,
            &[],
            desired,
            0.5,
            1.0,
            &AvoidanceConfig::default(),
        );
        assert!((command.linear_m_s - desired.linear_m_s).abs() < 0.06);
        assert!((command.angular_rad_s - desired.angular_rad_s).abs() < 0.11);
    }
}
