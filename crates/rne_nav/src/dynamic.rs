//! Dynamic-obstacle tracking and predictive local planning.
//!
//! [`ObstacleTracker`] associates detections to constant-velocity tracks, so
//! moving obstacles gain a velocity estimate. [`predictive_collision`] rolls a
//! candidate command forward against the predicted obstacle motion, and
//! [`select_predictive_command`] picks the fastest collision-free candidate.
//! Association is greedy and index-ordered, so a detection sequence replays
//! deterministically.

use crate::control::VelocityCommand2d;
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Errors raised by the tracker.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DynamicError {
    /// A configuration value was non-finite or non-positive.
    #[error("invalid dynamic obstacle configuration")]
    InvalidConfig,
    /// A detections or time input was non-finite.
    #[error("non-finite dynamic obstacle input")]
    NonFiniteInput,
    /// The time step was not strictly positive.
    #[error("time step must be finite and positive")]
    NonPositiveTime,
}

/// One tracked moving obstacle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Track {
    /// Stable track id.
    pub id: u32,
    /// Latest position in meters (map plane; `z` is ignored).
    pub position_m: Vec3,
    /// Estimated velocity in meters per second.
    pub velocity_m_s: Vec3,
    /// Obstacle radius in meters.
    pub radius_m: f64,
    /// Consecutive updates without an associated detection.
    pub missed: u32,
}

/// A single obstacle detection.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// Detection position in meters (map plane).
    pub position_m: Vec3,
    /// Detection radius in meters.
    pub radius_m: f64,
}

/// Configuration for [`ObstacleTracker`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObstacleTrackerConfig {
    /// Maximum association distance in meters.
    pub gate_m: f64,
    /// Updates a track may miss before it is dropped.
    pub max_missed: u32,
    /// Velocity smoothing gain in `(0, 1]`.
    pub velocity_gain: f64,
}

impl Default for ObstacleTrackerConfig {
    fn default() -> Self {
        Self {
            gate_m: 1.0,
            max_missed: 3,
            velocity_gain: 0.5,
        }
    }
}

impl ObstacleTrackerConfig {
    fn is_valid(&self) -> bool {
        self.gate_m.is_finite()
            && self.gate_m > 0.0
            && self.velocity_gain.is_finite()
            && self.velocity_gain > 0.0
            && self.velocity_gain <= 1.0
    }
}

/// A greedy nearest-neighbour constant-velocity tracker.
#[derive(Clone, Debug, PartialEq)]
pub struct ObstacleTracker {
    config: ObstacleTrackerConfig,
    tracks: Vec<Track>,
    next_id: u32,
}

impl ObstacleTracker {
    /// Creates an empty tracker.
    pub fn new(config: ObstacleTrackerConfig) -> Result<Self, DynamicError> {
        if !config.is_valid() {
            return Err(DynamicError::InvalidConfig);
        }
        Ok(Self {
            config,
            tracks: Vec::new(),
            next_id: 0,
        })
    }

    /// The current tracks.
    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    /// Number of active tracks.
    pub fn len(&self) -> usize {
        self.tracks.len()
    }

    /// Whether there are no active tracks.
    pub fn is_empty(&self) -> bool {
        self.tracks.is_empty()
    }

    /// Associates `detections` and updates the tracks over `dt_s`.
    pub fn update(&mut self, detections: &[Detection], dt_s: f64) -> Result<(), DynamicError> {
        if !dt_s.is_finite() {
            return Err(DynamicError::NonFiniteInput);
        }
        if dt_s <= 0.0 {
            return Err(DynamicError::NonPositiveTime);
        }
        if detections
            .iter()
            .any(|detection| !detection.position_m.is_finite() || !detection.radius_m.is_finite())
        {
            return Err(DynamicError::NonFiniteInput);
        }

        let mut assigned = vec![false; detections.len()];
        for track in &mut self.tracks {
            let mut best: Option<(usize, f64)> = None;
            for (index, detection) in detections.iter().enumerate() {
                if assigned[index] {
                    continue;
                }
                let distance = (detection.position_m - track.position_m).length();
                if distance <= self.config.gate_m
                    && best.map(|(_, best)| distance < best).unwrap_or(true)
                {
                    best = Some((index, distance));
                }
            }
            if let Some((index, _)) = best {
                assigned[index] = true;
                let detection = detections[index];
                let measured = (detection.position_m - track.position_m) / dt_s;
                track.velocity_m_s = track.velocity_m_s
                    + (measured - track.velocity_m_s) * self.config.velocity_gain;
                track.position_m = detection.position_m;
                track.radius_m = detection.radius_m;
                track.missed = 0;
            } else {
                track.missed += 1;
            }
        }
        self.tracks
            .retain(|track| track.missed <= self.config.max_missed);

        for (index, detection) in detections.iter().enumerate() {
            if assigned[index] {
                continue;
            }
            self.tracks.push(Track {
                id: self.next_id,
                position_m: detection.position_m,
                velocity_m_s: Vec3::ZERO,
                radius_m: detection.radius_m,
                missed: 0,
            });
            self.next_id += 1;
        }
        Ok(())
    }
}

/// Predictive-planning configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PredictiveConfig {
    /// Prediction horizon in seconds.
    pub horizon_s: f64,
    /// Prediction step in seconds.
    pub dt_s: f64,
    /// Robot radius in meters.
    pub robot_radius_m: f64,
    /// Extra clearance in meters.
    pub safety_margin_m: f64,
}

impl Default for PredictiveConfig {
    fn default() -> Self {
        Self {
            horizon_s: 2.0,
            dt_s: 0.1,
            robot_radius_m: 0.3,
            safety_margin_m: 0.1,
        }
    }
}

impl PredictiveConfig {
    fn is_valid(&self) -> bool {
        self.horizon_s.is_finite()
            && self.horizon_s > 0.0
            && self.dt_s.is_finite()
            && self.dt_s > 0.0
            && self.robot_radius_m.is_finite()
            && self.robot_radius_m >= 0.0
            && self.safety_margin_m.is_finite()
            && self.safety_margin_m >= 0.0
    }
}

/// Whether a candidate command collides with the predicted obstacle motion.
pub fn predictive_collision(
    pose: Pose2d,
    command: VelocityCommand2d,
    obstacles: &[Track],
    config: &PredictiveConfig,
) -> bool {
    if !config.is_valid()
        || !pose.is_finite()
        || !command.linear_m_s.is_finite()
        || !command.angular_rad_s.is_finite()
    {
        return true;
    }
    let steps = (config.horizon_s / config.dt_s).ceil() as usize;
    for step in 0..=steps {
        let time = step as f64 * config.dt_s;
        let robot_x = pose.x_m + command.linear_m_s * pose.yaw_rad.cos() * time;
        let robot_y = pose.y_m + command.linear_m_s * pose.yaw_rad.sin() * time;
        for obstacle in obstacles {
            let obstacle_x = obstacle.position_m.x + obstacle.velocity_m_s.x * time;
            let obstacle_y = obstacle.position_m.y + obstacle.velocity_m_s.y * time;
            let clearance = config.robot_radius_m + obstacle.radius_m + config.safety_margin_m;
            let distance = ((robot_x - obstacle_x).powi(2) + (robot_y - obstacle_y).powi(2)).sqrt();
            if distance < clearance {
                return true;
            }
        }
    }
    false
}

/// Picks the collision-free candidate that makes the most progress to `goal`.
///
/// Candidates are scored by `linear * cos(heading_error)`; ties keep the
/// earliest candidate. Returns `None` when every candidate collides.
pub fn select_predictive_command(
    pose: Pose2d,
    goal: Pose2d,
    candidates: &[VelocityCommand2d],
    obstacles: &[Track],
    config: &PredictiveConfig,
) -> Option<VelocityCommand2d> {
    let heading = (goal.y_m - pose.y_m).atan2(goal.x_m - pose.x_m);
    let error = wrap_angle(heading - pose.yaw_rad);
    let mut best: Option<(VelocityCommand2d, f64)> = None;
    for candidate in candidates {
        if predictive_collision(pose, *candidate, obstacles, config) {
            continue;
        }
        let score = candidate.linear_m_s * error.cos() - 0.1 * candidate.angular_rad_s.abs();
        if best
            .map(|(_, best_score)| score > best_score)
            .unwrap_or(true)
        {
            best = Some((*candidate, score));
        }
    }
    best.map(|(command, _)| command)
}

fn wrap_angle(angle_rad: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut wrapped = angle_rad % two_pi;
    if wrapped > std::f64::consts::PI {
        wrapped -= two_pi;
    } else if wrapped <= -std::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn tracks_a_moving_detection_and_estimates_velocity() {
        let mut tracker = ObstacleTracker::new(ObstacleTrackerConfig {
            velocity_gain: 1.0,
            ..ObstacleTrackerConfig::default()
        })
        .unwrap();
        for step in 0..8 {
            let x = step as f64 * 0.1;
            tracker
                .update(
                    &[Detection {
                        position_m: Vec3::new(x, 0.0, 0.0),
                        radius_m: 0.2,
                    }],
                    0.1,
                )
                .unwrap();
        }
        assert_eq!(tracker.len(), 1);
        let track = tracker.tracks()[0];
        assert_eq!(track.id, 0);
        assert_relative_eq!(track.velocity_m_s.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(track.position_m.x, 0.7, epsilon = 1e-9);
    }

    #[test]
    fn drops_tracks_after_missing_updates() {
        let config = ObstacleTrackerConfig {
            max_missed: 1,
            ..ObstacleTrackerConfig::default()
        };
        let mut tracker = ObstacleTracker::new(config).unwrap();
        tracker
            .update(
                &[Detection {
                    position_m: Vec3::ZERO,
                    radius_m: 0.2,
                }],
                0.1,
            )
            .unwrap();
        tracker.update(&[], 0.1).unwrap();
        assert_eq!(tracker.len(), 1);
        tracker.update(&[], 0.1).unwrap();
        assert!(tracker.is_empty());
    }

    #[test]
    fn predicts_a_crossing_obstacle() {
        let config = PredictiveConfig::default();
        let pose = Pose2d::IDENTITY;
        let obstacle = Track {
            id: 0,
            position_m: Vec3::new(1.0, -1.0, 0.0),
            velocity_m_s: Vec3::new(0.0, 1.0, 0.0),
            radius_m: 0.2,
            missed: 0,
        };
        // Driving straight ahead crosses the obstacle's path.
        assert!(predictive_collision(
            pose,
            VelocityCommand2d::new(0.8, 0.0),
            &[obstacle],
            &config
        ));
        // Turning away stays clear.
        assert!(!predictive_collision(
            pose,
            VelocityCommand2d::new(0.0, 1.0),
            &[obstacle],
            &config
        ));
    }

    #[test]
    fn selects_the_fastest_safe_candidate() {
        let config = PredictiveConfig::default();
        let pose = Pose2d::IDENTITY;
        let goal = Pose2d::new(10.0, 0.0, 0.0);
        let obstacle = Track {
            id: 0,
            position_m: Vec3::new(1.0, -0.5, 0.0),
            velocity_m_s: Vec3::new(0.0, 1.0, 0.0),
            radius_m: 0.2,
            missed: 0,
        };
        let candidates = [
            VelocityCommand2d::new(1.0, 0.0),
            VelocityCommand2d::new(0.3, 0.0),
            VelocityCommand2d::new(0.0, 1.0),
        ];
        let selected = select_predictive_command(pose, goal, &candidates, &[obstacle], &config)
            .expect("a safe candidate");
        // The fast straight command collides; the slower straight one is safest-fastest.
        assert_relative_eq!(selected.linear_m_s, 0.3, epsilon = 1e-12);
    }

    #[test]
    fn rejects_invalid_config_and_time() {
        let bad = ObstacleTrackerConfig {
            gate_m: 0.0,
            ..ObstacleTrackerConfig::default()
        };
        assert!(matches!(
            ObstacleTracker::new(bad),
            Err(DynamicError::InvalidConfig)
        ));
        let mut tracker = ObstacleTracker::new(ObstacleTrackerConfig::default()).unwrap();
        assert_eq!(tracker.update(&[], 0.0), Err(DynamicError::NonPositiveTime));
    }
}
