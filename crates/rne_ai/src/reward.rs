//! Reward helpers for built-in environments.

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
