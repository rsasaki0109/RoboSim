//! Goal-conditioned policies and curriculum helpers.

use crate::action::DiffDriveAction;
use crate::env::DiffDriveEpisode;
use crate::observation::DiffDriveObservation;
use crate::policy::Policy;
use crate::rng::DeterministicRng;
use serde::{Deserialize, Serialize};

/// Snapshot of runtime goal-curriculum progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalCurriculumSnapshot {
    /// Active curriculum stage index.
    pub stage_index: usize,
    /// Number of successful episodes accumulated in the active stage.
    pub successes_in_stage: u32,
}

/// Error restoring goal-curriculum progress.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GoalCurriculumSnapshotError {
    /// The snapshot points beyond the configured stage list.
    StageOutOfRange {
        /// Invalid stage index from the snapshot.
        stage_index: usize,
        /// Number of stages configured for this curriculum.
        stage_count: usize,
    },
}

/// Resolves the goal X coordinate from an observation.
pub fn goal_x_from_observation(observation: &DiffDriveObservation) -> f64 {
    observation
        .goal_delta_x_m
        .map(|delta| observation.base_x_m + delta)
        .unwrap_or(observation.base_x_m)
}

/// Policy that consumes an explicit goal alongside each observation.
pub trait GoalConditionedPolicy {
    /// Chooses an action given the latest observation and target goal.
    fn act_toward_goal(
        &mut self,
        observation: &DiffDriveObservation,
        goal_x_m: f64,
    ) -> DiffDriveAction;
}

/// Drives forward or backward until the base X position reaches the goal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GoalSeekingPolicy {
    /// Wheel velocity magnitude in radians per second.
    pub velocity_rad_s: f64,
    /// Distance in meters within which the robot stops.
    pub tolerance_m: f64,
}

impl GoalSeekingPolicy {
    /// Creates a goal-seeking policy with the given speed and stop tolerance.
    pub fn new(velocity_rad_s: f64, tolerance_m: f64) -> Self {
        Self {
            velocity_rad_s,
            tolerance_m,
        }
    }
}

impl GoalConditionedPolicy for GoalSeekingPolicy {
    fn act_toward_goal(
        &mut self,
        observation: &DiffDriveObservation,
        goal_x_m: f64,
    ) -> DiffDriveAction {
        let delta_x_m = goal_x_m - observation.base_x_m;
        if delta_x_m.abs() <= self.tolerance_m {
            return DiffDriveAction::forward(0.0);
        }
        DiffDriveAction::forward(self.velocity_rad_s.copysign(delta_x_m))
    }
}

impl Policy<DiffDriveEpisode> for GoalSeekingPolicy {
    fn act(&mut self, observation: &DiffDriveObservation) -> DiffDriveAction {
        self.act_toward_goal(observation, goal_x_from_observation(observation))
    }
}

/// Wraps a [`GoalConditionedPolicy`] as a standard [`Policy`].
#[derive(Clone, Debug, PartialEq)]
pub struct GoalConditionedAdapter<P> {
    inner: P,
}

impl<P> GoalConditionedAdapter<P> {
    /// Creates an adapter around a goal-conditioned policy.
    pub fn new(inner: P) -> Self {
        Self { inner }
    }
}

impl<P> Policy<DiffDriveEpisode> for GoalConditionedAdapter<P>
where
    P: GoalConditionedPolicy,
{
    fn act(&mut self, observation: &DiffDriveObservation) -> DiffDriveAction {
        self.inner
            .act_toward_goal(observation, goal_x_from_observation(observation))
    }
}

/// Fixed set of goal positions sampled on each episode reset.
#[derive(Clone, Debug, PartialEq)]
pub struct GoalTaskSet {
    goals_x_m: Vec<f64>,
}

impl GoalTaskSet {
    /// Creates a task set from explicit goal positions.
    pub fn new(goals_x_m: Vec<f64>) -> Self {
        Self { goals_x_m }
    }

    /// Returns a small training set with near, medium, and far goals.
    pub fn forward_training() -> Self {
        Self::new(vec![1.0, 1.5, 2.0, 2.5])
    }

    /// Samples one goal uniformly from the task set.
    pub fn sample(&self, rng: &mut DeterministicRng) -> f64 {
        let index = rng.uniform_usize(self.goals_x_m.len());
        self.goals_x_m[index]
    }
}

/// One curriculum stage with a goal range and success threshold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GoalCurriculumStage {
    /// Inclusive goal X range in meters.
    pub goal_x_m_range: (f64, f64),
    /// Successful episodes required before advancing.
    pub successes_to_advance: u32,
}

/// Static curriculum definition used to build runtime state.
#[derive(Clone, Debug, PartialEq)]
pub struct GoalCurriculumConfig {
    /// Ordered stages from easy to hard.
    pub stages: Vec<GoalCurriculumStage>,
}

impl GoalCurriculumConfig {
    /// Three-stage curriculum from short to long forward goals.
    pub fn easy_to_hard() -> Self {
        Self {
            stages: vec![
                GoalCurriculumStage {
                    goal_x_m_range: (0.8, 1.2),
                    successes_to_advance: 2,
                },
                GoalCurriculumStage {
                    goal_x_m_range: (1.5, 2.0),
                    successes_to_advance: 2,
                },
                GoalCurriculumStage {
                    goal_x_m_range: (2.0, 3.0),
                    successes_to_advance: u32::MAX,
                },
            ],
        }
    }
}

/// Runtime curriculum state that tracks stage progress.
#[derive(Clone, Debug, PartialEq)]
pub struct GoalCurriculum {
    config: GoalCurriculumConfig,
    stage_index: usize,
    successes_in_stage: u32,
}

impl GoalCurriculum {
    /// Creates runtime curriculum state from a static config.
    pub fn new(config: GoalCurriculumConfig) -> Self {
        Self {
            config,
            stage_index: 0,
            successes_in_stage: 0,
        }
    }

    /// Returns the active stage index.
    pub fn stage_index(&self) -> usize {
        self.stage_index
    }

    /// Returns the number of configured stages.
    pub fn stage_count(&self) -> usize {
        self.config.stages.len()
    }

    /// Returns the number of successes accumulated in the active stage.
    pub fn successes_in_stage(&self) -> u32 {
        self.successes_in_stage
    }

    /// Returns a snapshot of runtime curriculum progress.
    pub fn snapshot(&self) -> GoalCurriculumSnapshot {
        GoalCurriculumSnapshot {
            stage_index: self.stage_index,
            successes_in_stage: self.successes_in_stage,
        }
    }

    /// Restores runtime curriculum progress from a snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`GoalCurriculumSnapshotError::StageOutOfRange`] if the snapshot
    /// was produced for a curriculum with more stages than this instance.
    pub fn restore_snapshot(
        &mut self,
        snapshot: GoalCurriculumSnapshot,
    ) -> Result<(), GoalCurriculumSnapshotError> {
        if snapshot.stage_index >= self.stage_count() {
            return Err(GoalCurriculumSnapshotError::StageOutOfRange {
                stage_index: snapshot.stage_index,
                stage_count: self.stage_count(),
            });
        }
        self.stage_index = snapshot.stage_index;
        self.successes_in_stage = snapshot.successes_in_stage;
        Ok(())
    }

    /// Samples a goal from the active stage range.
    pub fn sample_goal(&mut self, rng: &mut DeterministicRng) -> f64 {
        let stage = self
            .config
            .stages
            .get(self.stage_index)
            .copied()
            .unwrap_or_else(|| {
                self.config
                    .stages
                    .last()
                    .copied()
                    .expect("curriculum requires at least one stage")
            });
        let (min, max) = stage.goal_x_m_range;
        rng.uniform_f64(min, max)
    }

    /// Records episode completion and advances the stage when enough successes occur.
    pub fn record_episode_end(&mut self, terminated: bool) -> bool {
        if !terminated {
            return false;
        }

        let stage = match self.config.stages.get(self.stage_index) {
            Some(stage) => *stage,
            None => return false,
        };

        self.successes_in_stage += 1;
        if self.successes_in_stage < stage.successes_to_advance {
            return false;
        }
        if self.stage_index + 1 >= self.config.stages.len() {
            return false;
        }

        self.stage_index += 1;
        self.successes_in_stage = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn goal_seeking_reaches_forward_goal() {
        let mut policy = GoalSeekingPolicy::new(6.0, 0.05);
        let mut observation = DiffDriveObservation {
            base_x_m: 0.0,
            goal_delta_x_m: Some(1.5),
            ..DiffDriveObservation::default()
        };

        for _ in 0..200 {
            if observation
                .goal_delta_x_m
                .is_some_and(|delta| delta.abs() <= policy.tolerance_m)
            {
                break;
            }
            let action = policy.act_toward_goal(&observation, 1.5);
            assert!(action.left_velocity_rad_s > 0.0);
            observation.base_x_m += action.left_velocity_rad_s * 0.01;
            observation.goal_delta_x_m = Some(1.5 - observation.base_x_m);
        }

        assert!(observation.base_x_m >= 1.45);
    }

    #[test]
    fn goal_task_set_samples_known_values() {
        let tasks = GoalTaskSet::forward_training();
        let mut rng = DeterministicRng::new(3);
        let goal = tasks.sample(&mut rng);
        assert!(tasks.goals_x_m.contains(&goal));
    }

    #[test]
    fn curriculum_advances_after_successes() {
        let mut curriculum = GoalCurriculum::new(GoalCurriculumConfig::easy_to_hard());
        assert_eq!(curriculum.stage_index(), 0);
        assert!(!curriculum.record_episode_end(true));
        assert!(curriculum.record_episode_end(true));
        assert_eq!(curriculum.stage_index(), 1);
    }

    #[test]
    fn curriculum_snapshot_restores_progress() {
        let mut curriculum = GoalCurriculum::new(GoalCurriculumConfig::easy_to_hard());
        assert!(!curriculum.record_episode_end(true));
        let snapshot = curriculum.snapshot();
        assert!(curriculum.record_episode_end(true));

        curriculum.restore_snapshot(snapshot).unwrap();

        assert_eq!(curriculum.stage_index(), 0);
        assert_eq!(curriculum.successes_in_stage(), 1);
    }
}
