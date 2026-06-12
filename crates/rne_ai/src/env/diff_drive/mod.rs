//! Differential drive simulation and episode environment.

mod sim;

pub use sim::DiffDriveSim;

use crate::action::DiffDriveAction;
use crate::episode::{Episode, EpisodeStep};
use crate::observation::DiffDriveObservation;
use crate::reward::DiffDriveRewardConfig;
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
}

impl DiffDriveEpisode {
    /// Creates a new episode environment with the given configuration.
    pub fn new(config: DiffDriveEpisodeConfig) -> Self {
        let sim = new_sim(&config).expect("episode simulation");
        Self {
            sim,
            config,
            episode_index: 0,
            step_in_episode: 0,
            prev_x_m: 0.0,
            total_reward: 0.0,
            log: SimulationLog::new(),
        }
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

    fn make_step(
        &mut self,
        observation: DiffDriveObservation,
    ) -> EpisodeStep<DiffDriveObservation> {
        let delta_x_m = observation.base_x_m - self.prev_x_m;
        self.prev_x_m = observation.base_x_m;

        let reached_goal = observation.base_x_m >= self.config.goal_x_m;
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
        self.sim = new_sim(&self.config).expect("episode simulation");
        self.step_in_episode = 0;
        self.prev_x_m = 0.0;
        self.total_reward = 0.0;
        self.log = SimulationLog::new();

        let observation = self.sim.observe();
        self.prev_x_m = observation.base_x_m;

        EpisodeStep {
            observation,
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let observation = self.sim.step_with_recording(
            action.left_velocity_rad_s,
            action.right_velocity_rad_s,
            self.config.record_log,
            &mut self.log,
        );
        self.step_in_episode += 1;

        let result = self.make_step(observation);
        if result.is_done() {
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

fn new_sim(config: &DiffDriveEpisodeConfig) -> Result<DiffDriveSim, rne_assets::AssetError> {
    if let Some(scene_path) = &config.scene_path {
        DiffDriveSim::from_scene_path(scene_path)
    } else {
        Ok(DiffDriveSim::with_initial_translation(
            config.initial_translation_m,
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
}
