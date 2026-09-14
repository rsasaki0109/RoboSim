//! Nav2-style recovery behaviors.
//!
//! When a planner or controller cannot make progress, a Nav2 stack runs a
//! sequence of recovery behaviors: clear the costmap, spin in place, back up,
//! or wait. [`RecoveryBehavior`] executes a single action as a stateful
//! actuator that emits [`VelocityCommand2d`] setpoints, and [`RecoverySequence`]
//! chains several so a stuck robot tries each in order.
//!
//! Every action is time- or distance-bounded and deterministic: repeated
//! [`RecoveryBehavior::step`] calls with the same `dt_s` produce identical
//! commands and completion ticks.

use crate::control::VelocityCommand2d;
use crate::grid::{GridCoord, OccupancyGrid};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Log-odds applied when clearing a cell during a costmap-clearing recovery.
pub const CLEAR_LOG_ODDS: f64 = -5.0;

/// A single recovery action.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RecoveryAction {
    /// Clear occupied/unknown cells within a radius of the current pose.
    ClearCostmap {
        /// Radius to clear in meters.
        radius_m: f64,
    },
    /// Rotate in place through an accumulated angle.
    Spin {
        /// Total absolute angle to sweep in radians.
        target_yaw_rad: f64,
        /// Signed yaw rate in radians per second.
        angular_rad_s: f64,
    },
    /// Drive backward along the current heading.
    BackUp {
        /// Distance to travel in meters.
        distance_m: f64,
        /// Forward speed magnitude in meters per second (applied backward).
        linear_m_s: f64,
    },
    /// Hold a zero command for a fixed duration.
    Wait {
        /// Duration to hold in seconds.
        duration_s: f64,
    },
}

/// Errors raised by recovery behaviors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RecoveryError {
    /// An action parameter was non-finite or non-positive.
    #[error("invalid recovery action")]
    InvalidAction,
    /// A step time was non-finite or non-positive.
    #[error("time step must be finite and positive")]
    NonPositiveTime,
    /// A recovery sequence was empty.
    #[error("recovery sequence must not be empty")]
    EmptySequence,
}

/// Status of a recovery behavior after one step.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RecoveryStatus {
    /// The action is still executing and emitted a command.
    Running(VelocityCommand2d),
    /// The action completed successfully.
    Succeeded,
    /// The action cannot make progress and should be skipped.
    Failed,
}

/// Clears cells within `radius_m` of `center`, returning the number cleared.
///
/// Cells are set toward free space with `free_log_odds` (typically
/// [`CLEAR_LOG_ODDS`]); the caller then rebuilds the costmap.
pub fn clear_costmap_around(
    grid: &mut OccupancyGrid,
    center: Pose2d,
    radius_m: f64,
    free_log_odds: f64,
) -> Result<usize, RecoveryError> {
    if !center.is_finite() || !radius_m.is_finite() || radius_m <= 0.0 || !free_log_odds.is_finite()
    {
        return Err(RecoveryError::InvalidAction);
    }
    let Some(center_coord) = grid.world_to_grid(Vec3::new(center.x_m, center.y_m, 0.0)) else {
        return Ok(0);
    };
    let radius_cells = (radius_m / grid.resolution_m()).ceil() as isize;
    let mut cleared = 0;
    for dy in -radius_cells..=radius_cells {
        for dx in -radius_cells..=radius_cells {
            let coord = GridCoord {
                x: center_coord.x + dx,
                y: center_coord.y + dy,
            };
            if !grid.contains(coord) {
                continue;
            }
            let world = grid.grid_to_world(coord);
            let distance = ((world.x - center.x_m).powi(2) + (world.y - center.y_m).powi(2)).sqrt();
            if distance <= radius_m && grid.apply_free(coord, free_log_odds) {
                cleared += 1;
            }
        }
    }
    Ok(cleared)
}

/// Stateful executor for a single [`RecoveryAction`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryBehavior {
    action: RecoveryAction,
    progress: f64,
}

impl RecoveryBehavior {
    /// Creates an executor after validating the action.
    pub fn new(action: RecoveryAction) -> Result<Self, RecoveryError> {
        if !action_is_valid(&action) {
            return Err(RecoveryError::InvalidAction);
        }
        Ok(Self {
            action,
            progress: 0.0,
        })
    }

    /// The action being executed.
    pub fn action(&self) -> RecoveryAction {
        self.action
    }

    /// Resets accumulated progress so the behavior can run again.
    pub fn reset(&mut self) {
        self.progress = 0.0;
    }

    /// Advances the behavior by `dt_s`.
    pub fn step(&mut self, dt_s: f64) -> Result<RecoveryStatus, RecoveryError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(RecoveryError::NonPositiveTime);
        }
        match self.action {
            RecoveryAction::ClearCostmap { .. } => Ok(RecoveryStatus::Succeeded),
            RecoveryAction::Wait { duration_s } => {
                self.progress += dt_s;
                if self.progress >= duration_s {
                    Ok(RecoveryStatus::Succeeded)
                } else {
                    Ok(RecoveryStatus::Running(VelocityCommand2d::ZERO))
                }
            }
            RecoveryAction::Spin {
                target_yaw_rad,
                angular_rad_s,
            } => {
                self.progress += (angular_rad_s * dt_s).abs();
                if self.progress >= target_yaw_rad {
                    Ok(RecoveryStatus::Succeeded)
                } else {
                    Ok(RecoveryStatus::Running(VelocityCommand2d::new(
                        0.0,
                        angular_rad_s,
                    )))
                }
            }
            RecoveryAction::BackUp {
                distance_m,
                linear_m_s,
            } => {
                self.progress += (linear_m_s * dt_s).abs();
                if self.progress >= distance_m {
                    Ok(RecoveryStatus::Succeeded)
                } else {
                    Ok(RecoveryStatus::Running(VelocityCommand2d::new(
                        -linear_m_s,
                        0.0,
                    )))
                }
            }
        }
    }
}

/// Outcome of stepping a [`RecoverySequence`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum RecoveryOutcome {
    /// An action is executing and emitted a command.
    Running(VelocityCommand2d),
    /// An action completed; the caller should retry planning.
    Succeeded,
    /// Every action failed or was consumed without success.
    Exhausted,
}

/// Runs a list of [`RecoveryAction`]s in order until one succeeds.
#[derive(Clone, Debug, PartialEq)]
pub struct RecoverySequence {
    behaviors: Vec<RecoveryBehavior>,
    index: usize,
}

impl RecoverySequence {
    /// Creates a sequence after validating every action.
    pub fn new(actions: &[RecoveryAction]) -> Result<Self, RecoveryError> {
        if actions.is_empty() {
            return Err(RecoveryError::EmptySequence);
        }
        let behaviors = actions
            .iter()
            .map(|action| RecoveryBehavior::new(*action))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            behaviors,
            index: 0,
        })
    }

    /// Rewinds the sequence to the first action.
    pub fn reset(&mut self) {
        self.index = 0;
        for behavior in &mut self.behaviors {
            behavior.reset();
        }
    }

    /// The index of the action currently executing.
    pub fn current_index(&self) -> usize {
        self.index
    }

    /// Advances the active action by `dt_s`, applying costmap clearing to `grid`.
    pub fn step(
        &mut self,
        grid: &mut OccupancyGrid,
        center: Pose2d,
        dt_s: f64,
    ) -> Result<RecoveryOutcome, RecoveryError> {
        while self.index < self.behaviors.len() {
            let action = self.behaviors[self.index].action();
            if let RecoveryAction::ClearCostmap { radius_m } = action {
                clear_costmap_around(grid, center, radius_m, CLEAR_LOG_ODDS)?;
                self.index += 1;
                return Ok(RecoveryOutcome::Succeeded);
            }
            match self.behaviors[self.index].step(dt_s)? {
                RecoveryStatus::Running(command) => return Ok(RecoveryOutcome::Running(command)),
                RecoveryStatus::Succeeded => {
                    self.index += 1;
                    return Ok(RecoveryOutcome::Succeeded);
                }
                RecoveryStatus::Failed => {
                    self.index += 1;
                }
            }
        }
        Ok(RecoveryOutcome::Exhausted)
    }
}

fn action_is_valid(action: &RecoveryAction) -> bool {
    match *action {
        RecoveryAction::ClearCostmap { radius_m } => radius_m.is_finite() && radius_m > 0.0,
        RecoveryAction::Spin {
            target_yaw_rad,
            angular_rad_s,
        } => {
            target_yaw_rad.is_finite()
                && target_yaw_rad > 0.0
                && angular_rad_s.is_finite()
                && angular_rad_s != 0.0
        }
        RecoveryAction::BackUp {
            distance_m,
            linear_m_s,
        } => {
            distance_m.is_finite() && distance_m > 0.0 && linear_m_s.is_finite() && linear_m_s > 0.0
        }
        RecoveryAction::Wait { duration_s } => duration_s.is_finite() && duration_s > 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn grid() -> OccupancyGrid {
        OccupancyGrid::new(40, 40, 0.1, Pose2d::new(-2.0, -2.0, 0.0)).unwrap()
    }

    #[test]
    fn clear_costmap_clears_within_radius_only() {
        let mut grid = grid();
        let center = GridCoord { x: 20, y: 20 };
        grid.apply_occupied(center, 5.0);
        let outside = GridCoord { x: 20, y: 35 };
        grid.apply_occupied(outside, 5.0);
        let cleared =
            clear_costmap_around(&mut grid, Pose2d::new(0.0, 0.0, 0.0), 0.5, CLEAR_LOG_ODDS)
                .unwrap();
        assert!(cleared > 0);
        assert!(!grid.is_occupied(center));
        assert!(grid.is_occupied(outside));
    }

    #[test]
    fn spin_completes_after_the_swept_angle() {
        let mut behavior = RecoveryBehavior::new(RecoveryAction::Spin {
            target_yaw_rad: std::f64::consts::PI,
            angular_rad_s: 1.0,
        })
        .unwrap();
        let mut status = RecoveryStatus::Running(VelocityCommand2d::ZERO);
        for _ in 0..40 {
            status = behavior.step(0.1).unwrap();
        }
        assert_eq!(status, RecoveryStatus::Succeeded);
    }

    #[test]
    fn backup_drives_backward_until_the_distance_is_reached() {
        let mut behavior = RecoveryBehavior::new(RecoveryAction::BackUp {
            distance_m: 0.05,
            linear_m_s: 0.2,
        })
        .unwrap();
        let status = behavior.step(0.1).unwrap();
        match status {
            RecoveryStatus::Running(command) => {
                assert_relative_eq!(command.linear_m_s, -0.2, epsilon = 1e-12);
            }
            other => panic!("unexpected {other:?}"),
        }
        let status = behavior.step(1.0).unwrap();
        assert_eq!(status, RecoveryStatus::Succeeded);
    }

    #[test]
    fn wait_succeeds_after_the_duration() {
        let mut behavior = RecoveryBehavior::new(RecoveryAction::Wait { duration_s: 0.2 }).unwrap();
        assert!(matches!(
            behavior.step(0.1).unwrap(),
            RecoveryStatus::Running(_)
        ));
        assert_eq!(behavior.step(0.1).unwrap(), RecoveryStatus::Succeeded);
    }

    #[test]
    fn sequence_advances_past_a_failing_action() {
        let sequence = RecoverySequence::new(&[RecoveryAction::Spin {
            target_yaw_rad: 1.0,
            angular_rad_s: 0.0,
        }])
        .map_err(|_| ());
        // A spin with zero rate is invalid and rejected at construction.
        assert!(sequence.is_err());

        let mut sequence = RecoverySequence::new(&[
            RecoveryAction::Wait { duration_s: 100.0 },
            RecoveryAction::BackUp {
                distance_m: 0.1,
                linear_m_s: 0.5,
            },
        ])
        .unwrap();
        let mut grid = grid();
        let outcome = sequence.step(&mut grid, Pose2d::IDENTITY, 0.1).unwrap();
        assert!(matches!(outcome, RecoveryOutcome::Running(_)));
        // Force completion of the wait by stepping past its duration.
        let outcome = sequence.step(&mut grid, Pose2d::IDENTITY, 100.0).unwrap();
        assert_eq!(outcome, RecoveryOutcome::Succeeded);
        assert_eq!(sequence.current_index(), 1);
    }

    #[test]
    fn clear_costmap_action_clears_via_the_sequence() {
        let mut sequence =
            RecoverySequence::new(&[RecoveryAction::ClearCostmap { radius_m: 0.5 }]).unwrap();
        let mut grid = grid();
        grid.apply_occupied(GridCoord { x: 20, y: 20 }, 5.0);
        let outcome = sequence.step(&mut grid, Pose2d::IDENTITY, 0.1).unwrap();
        assert_eq!(outcome, RecoveryOutcome::Succeeded);
        assert!(!grid.is_occupied(GridCoord { x: 20, y: 20 }));
    }

    #[test]
    fn rejects_invalid_action_and_time() {
        assert!(RecoveryBehavior::new(RecoveryAction::Wait { duration_s: 0.0 }).is_err());
        assert!(RecoverySequence::new(&[]).is_err());
        let mut behavior = RecoveryBehavior::new(RecoveryAction::Wait { duration_s: 1.0 }).unwrap();
        assert_eq!(behavior.step(0.0), Err(RecoveryError::NonPositiveTime));
    }
}
