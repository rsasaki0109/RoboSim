//! Differential drive simulation and episode environment.

mod sim;
mod vectorized;

pub use sim::DiffDriveSim;
pub use vectorized::{VectorizedDiffDriveConfig, VectorizedDiffDriveEnv, VectorizedDiffDriveStep};

use crate::action::DiffDriveAction;
use crate::domain_randomization::DiffDriveDomainRandomization;
use crate::episode::{Episode, EpisodeStep};
use crate::goal::{GoalCurriculum, GoalCurriculumConfig, GoalTaskSet};
use crate::observation::DiffDriveObservation;
use crate::reward::DiffDriveRewardConfig;
use crate::rng::DeterministicRng;
use rne_log::SimulationLog;
use rne_math::Vec3;
use std::path::PathBuf;

/// Configuration for a forward-drive goal episode.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveEpisodeConfig {
    /// Maximum steps before truncation.
    pub max_steps: u64,
    /// Target base X position in meters.
    pub goal_x_m: f64,
    /// Initial robot translation in meters.
    pub initial_translation_m: Vec3,
    /// Reward weights applied each step.
    pub reward: DiffDriveRewardConfig,
    /// When true, actuator commands are appended to an internal log.
    pub record_log: bool,
    /// When set, the episode loads world and robot state from this scene asset.
    pub scene_path: Option<PathBuf>,
    /// Optional domain randomization applied on each reset.
    pub domain_randomization: Option<DiffDriveDomainRandomization>,
    /// Optional fixed goal set sampled on each reset.
    pub goal_tasks: Option<GoalTaskSet>,
    /// Optional curriculum that advances after successful episodes.
    pub goal_curriculum: Option<GoalCurriculumConfig>,
    /// Seed for reproducible domain randomization.
    pub rng_seed: u64,
}

impl Default for DiffDriveEpisodeConfig {
    fn default() -> Self {
        Self {
            max_steps: 300,
            goal_x_m: 2.0,
            initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
            reward: DiffDriveRewardConfig::default(),
            record_log: false,
            scene_path: None,
            domain_randomization: None,
            goal_tasks: None,
            goal_curriculum: None,
            rng_seed: 1,
        }
    }
}

/// Goal-reaching episode built on top of [`DiffDriveSim`].
pub struct DiffDriveEpisode {
    sim: DiffDriveSim,
    config: DiffDriveEpisodeConfig,
    episode_index: u32,
    step_in_episode: u64,
    prev_x_m: f64,
    total_reward: f64,
    log: SimulationLog,
    rng: DeterministicRng,
    goal_x_m: f64,
    curriculum: Option<GoalCurriculum>,
}

impl DiffDriveEpisode {
    /// Creates a new episode environment with the given configuration.
    pub fn new(config: DiffDriveEpisodeConfig) -> Self {
        let rng = DeterministicRng::new(config.rng_seed);
        let goal_x_m = config.goal_x_m;
        let curriculum = config.goal_curriculum.clone().map(GoalCurriculum::new);
        let sim = new_sim(&config, config.initial_translation_m).expect("episode simulation");
        Self {
            sim,
            config,
            episode_index: 0,
            step_in_episode: 0,
            prev_x_m: 0.0,
            total_reward: 0.0,
            log: SimulationLog::new(),
            rng,
            goal_x_m,
            curriculum,
        }
    }

    /// Returns the active curriculum stage when curriculum training is enabled.
    pub fn curriculum_stage_index(&self) -> Option<usize> {
        self.curriculum.as_ref().map(GoalCurriculum::stage_index)
    }

    /// Returns the goal X position for the current episode.
    pub fn goal_x_m(&self) -> f64 {
        self.goal_x_m
    }

    /// Returns the world seed when running from a scene asset.
    pub fn world_seed(&self) -> u64 {
        self.sim.world_seed()
    }

    /// Returns read access to the underlying simulation (for rendering or inspection).
    pub fn simulation(&self) -> &DiffDriveSim {
        &self.sim
    }

    /// Returns cumulative reward for the current episode.
    pub fn total_reward(&self) -> f64 {
        self.total_reward
    }

    /// Returns the simulation log when recording is enabled.
    pub fn log(&self) -> &SimulationLog {
        &self.log
    }

    fn sample_goal_for_reset(&mut self) -> f64 {
        if let Some(curriculum) = self.curriculum.as_mut() {
            return curriculum.sample_goal(&mut self.rng);
        }
        if let Some(tasks) = &self.config.goal_tasks {
            return tasks.sample(&mut self.rng);
        }
        self.config.goal_x_m
    }

    fn observe_with_current_goal(&self) -> DiffDriveObservation {
        self.sim
            .observe_robot_with_goal(self.sim.robot().robot, Some(self.goal_x_m))
    }

    fn queue_primary_action(&mut self, action: DiffDriveAction) {
        self.sim.queue_robot_action(self.sim.robot().robot, action);
    }

    fn advance_primary_sim_step(&mut self) {
        self.sim
            .advance_one_tick(self.config.record_log, &mut self.log);
    }

    fn make_step(
        &mut self,
        observation: DiffDriveObservation,
    ) -> EpisodeStep<DiffDriveObservation> {
        let delta_x_m = observation.base_x_m - self.prev_x_m;
        self.prev_x_m = observation.base_x_m;

        let reached_goal = observation.base_x_m >= self.goal_x_m;
        let truncated = !reached_goal && self.step_in_episode >= self.config.max_steps;

        let reward = self.config.reward.compute(delta_x_m, reached_goal);
        self.total_reward += reward;

        EpisodeStep {
            observation,
            reward,
            terminated: reached_goal,
            truncated,
        }
    }
}

impl Episode for DiffDriveEpisode {
    type Observation = DiffDriveObservation;
    type Action = DiffDriveAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        let mut initial_translation_m = self.config.initial_translation_m;
        self.goal_x_m = self.sample_goal_for_reset();
        if let Some(domain_randomization) = &self.config.domain_randomization {
            domain_randomization.apply(
                &mut self.rng,
                &mut initial_translation_m,
                &mut self.goal_x_m,
            );
        }

        self.sim = new_sim(&self.config, initial_translation_m).expect("episode simulation");
        self.step_in_episode = 0;
        self.prev_x_m = 0.0;
        self.total_reward = 0.0;
        self.log = SimulationLog::new();

        let observation = self.observe_with_current_goal();
        self.prev_x_m = observation.base_x_m;

        EpisodeStep {
            observation,
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        self.queue_primary_action(action);
        self.advance_primary_sim_step();
        self.step_in_episode += 1;

        let observation = self.observe_with_current_goal();
        let result = self.make_step(observation);
        if result.is_done() {
            if result.terminated {
                if let Some(curriculum) = self.curriculum.as_mut() {
                    curriculum.record_episode_end(true);
                }
            }
            self.episode_index += 1;
        }
        result
    }

    fn episode_index(&self) -> u32 {
        self.episode_index
    }

    fn step_in_episode(&self) -> u64 {
        self.step_in_episode
    }
}

fn new_sim(
    config: &DiffDriveEpisodeConfig,
    initial_translation_m: Vec3,
) -> Result<DiffDriveSim, rne_assets::AssetError> {
    if let Some(scene_path) = &config.scene_path {
        DiffDriveSim::from_scene_path(scene_path)
    } else {
        Ok(DiffDriveSim::with_initial_translation(
            initial_translation_m,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::{ConstantVelocityPolicy, Policy};

    #[test]
    fn episode_reaches_goal_with_forward_policy() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            max_steps: 300,
            goal_x_m: 1.5,
            ..DiffDriveEpisodeConfig::default()
        });
        let mut policy = ConstantVelocityPolicy::new(6.0);

        let mut step = env.reset();
        let mut done = false;

        while !done && env.step_in_episode() < 300 {
            let action = policy.act(&step.observation);
            step = env.step(action);
            done = step.is_done();
        }

        assert!(step.terminated, "expected success termination");
        assert!(step.observation.base_x_m >= 1.5);
        assert!(env.total_reward() > 0.0);
    }

    #[test]
    fn episode_truncates_at_max_steps() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            max_steps: 5,
            goal_x_m: 100.0,
            ..DiffDriveEpisodeConfig::default()
        });

        env.reset();
        let mut last = env.step(DiffDriveAction::forward(0.0));

        for _ in 1..5 {
            last = env.step(DiffDriveAction::forward(0.0));
        }

        assert!(last.truncated);
        assert!(!last.terminated);
    }

    #[test]
    fn scene_episode_reaches_goal() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_x_m: 2.0,
            max_steps: 300,
            scene_path: Some(scene_path),
            ..DiffDriveEpisodeConfig::default()
        });
        let mut policy = ConstantVelocityPolicy::new(6.0);

        let mut step = env.reset();
        assert_eq!(env.world_seed(), 42);

        while !step.is_done() {
            step = env.step(policy.act(&step.observation));
        }

        assert!(step.terminated);
        assert!(step.observation.base_x_m >= 2.0);
    }

    #[test]
    fn domain_randomization_changes_goal_on_reset() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            domain_randomization: Some(DiffDriveDomainRandomization::forward_goal_training()),
            rng_seed: 11,
            ..DiffDriveEpisodeConfig::default()
        });

        env.reset();
        let first_goal = env.goal_x_m();
        env.reset();
        let second_goal = env.goal_x_m();

        assert_ne!(first_goal, second_goal);
    }

    #[test]
    fn reset_populates_goal_relative_observation() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_x_m: 2.0,
            ..DiffDriveEpisodeConfig::default()
        });
        let step = env.reset();
        assert_eq!(step.observation.goal_delta_x_m, Some(2.0));
    }

    #[test]
    fn goal_seeking_policy_reaches_sampled_task_goal() {
        use crate::goal::{GoalCurriculumConfig, GoalSeekingPolicy};

        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_curriculum: Some(GoalCurriculumConfig::easy_to_hard()),
            max_steps: 400,
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });
        let mut policy = GoalSeekingPolicy::new(6.0, 0.05);

        let mut step = env.reset();
        assert!(step.observation.goal_delta_x_m.is_some());

        while !step.is_done() {
            step = env.step(policy.act(&step.observation));
        }

        assert!(
            step.terminated,
            "goal-seeking policy should reach curriculum goal"
        );
    }
}
