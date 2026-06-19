//! Reward helpers for built-in environments.

use crate::reach::ReachTarget;
use serde::{Deserialize, Serialize};

/// Tunable reward weights for [`crate::env::DiffDriveEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffDriveRewardConfig {
    /// Reward multiplier for forward progress in meters.
    pub progress_scale: f64,
    /// Per-step time penalty.
    pub step_penalty: f64,
    /// Bonus when the goal X position is reached.
    pub goal_bonus: f64,
}

impl Default for DiffDriveRewardConfig {
    fn default() -> Self {
        Self {
            progress_scale: 1.0,
            step_penalty: 0.001,
            goal_bonus: 10.0,
        }
    }
}

impl DiffDriveRewardConfig {
    /// Computes reward from forward progress and goal completion.
    pub fn compute(&self, delta_x_m: f64, reached_goal: bool) -> f64 {
        let mut reward = delta_x_m * self.progress_scale - self.step_penalty;
        if reached_goal {
            reward += self.goal_bonus;
        }
        reward
    }
}

/// Episode task variants for mobile manipulator environments.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MobileManipulatorTask {
    /// Reach the end effector toward a world-frame target.
    Reach {
        /// Target end-effector pose in world coordinates.
        target: ReachTarget,
        /// Success threshold in meters.
        success_m: f64,
    },
    /// Close the gripper on a named object.
    Grasp {
        /// Scene object name.
        object_name: String,
    },
    /// Move a grasped object into a named drop zone.
    Transport {
        /// Dynamic object name.
        object_name: String,
        /// Fixed drop zone obstacle name.
        drop_zone_name: String,
    },
    /// Pick up an object, carry it, and release it at a target location.
    Place {
        /// Dynamic object name.
        object_name: String,
        /// World-frame target location for the placed object.
        target: ReachTarget,
        /// Horizontal success tolerance in meters.
        place_tolerance_m: f64,
    },
    /// Produce wrist camera frames while moving the arm.
    Inspect {
        /// Minimum RGBA8 byte count for success.
        min_wrist_pixels: usize,
    },
}

/// Tunable reward weights for [`crate::env::MobileManipulatorEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MobileManipulatorRewardConfig {
    /// Reward multiplier for task progress each step.
    pub progress_scale: f64,
    /// Per-step time penalty.
    pub step_penalty: f64,
    /// Bonus when the task success condition is met.
    pub success_bonus: f64,
}

impl Default for MobileManipulatorRewardConfig {
    fn default() -> Self {
        Self {
            progress_scale: 1.0,
            step_penalty: 0.001,
            success_bonus: 10.0,
        }
    }
}

impl MobileManipulatorRewardConfig {
    /// Computes reward from scalar progress and task completion.
    pub fn compute(&self, progress: f64, success: bool) -> f64 {
        let mut reward = progress * self.progress_scale - self.step_penalty;
        if success {
            reward += self.success_bonus;
        }
        reward
    }
}
