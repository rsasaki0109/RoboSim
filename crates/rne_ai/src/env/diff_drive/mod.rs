//! Differential drive simulation and episode environment.

mod sim;
mod vectorized;

pub use sim::{
    DiffDriveActuatorSnapshot, DiffDriveFrameSnapshot, DiffDriveJointMotorSnapshot,
    DiffDriveRigidBodySnapshot, DiffDriveSensorStateSnapshot, DiffDriveSim, DiffDriveSimSnapshot,
    DiffDriveSimSnapshotError, DiffDriveTransformSnapshot,
};
pub use vectorized::{
    VectorizedDiffDriveConfig, VectorizedDiffDriveEnv, VectorizedDiffDriveSnapshot,
    VectorizedDiffDriveSnapshotError, VectorizedDiffDriveStep,
};

use crate::action::DiffDriveAction;
use crate::domain_randomization::DiffDriveDomainRandomization;
use crate::episode::{Episode, EpisodeRandomSnapshot, EpisodeStep};
use crate::goal::{
    GoalCurriculum, GoalCurriculumConfig, GoalCurriculumSnapshot, GoalCurriculumSnapshotError,
    GoalTaskSet,
};
use crate::observation::DiffDriveObservation;
use crate::reward::DiffDriveRewardConfig;
use crate::rng::DeterministicRng;
use rne_log::{
    ReplayHeader, ReplayRandomSnapshot, ReplayRandomSnapshotError, ReplayRngState, SimulationLog,
};
use rne_math::Vec3;
use rne_world::{WorldRandomSnapshot, WORLD_RANDOM_STREAM_VERSION};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const DIFF_DRIVE_EPISODE_RNG_STATE: &str = "diff_drive_episode";
const DIFF_DRIVE_EPISODE_SNAPSHOT_VERSION: u32 = 1;

/// Error restoring or creating a differential-drive episode snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum DiffDriveEpisodeSnapshotError {
    /// Snapshot payload schema is not supported by this engine.
    UnsupportedSchemaVersion {
        /// Expected snapshot schema version.
        expected: u32,
        /// Actual snapshot schema version.
        actual: u32,
    },
    /// A deterministic checkpoint field is internally inconsistent.
    Mismatch {
        /// Field name that did not match.
        field: &'static str,
        /// Expected value.
        expected: String,
        /// Actual value from the snapshot.
        actual: String,
    },
    /// The embedded simulation snapshot failed.
    Simulation(DiffDriveSimSnapshotError),
    /// The embedded random checkpoint failed.
    Random(ReplayRandomSnapshotError),
    /// Snapshot does not contain curriculum state required by this episode.
    MissingCurriculum,
    /// Snapshot contains curriculum state but this episode has no curriculum.
    UnexpectedCurriculum,
    /// The embedded curriculum state failed.
    Curriculum(GoalCurriculumSnapshotError),
}

impl From<DiffDriveSimSnapshotError> for DiffDriveEpisodeSnapshotError {
    fn from(error: DiffDriveSimSnapshotError) -> Self {
        Self::Simulation(error)
    }
}

impl From<ReplayRandomSnapshotError> for DiffDriveEpisodeSnapshotError {
    fn from(error: ReplayRandomSnapshotError) -> Self {
        Self::Random(error)
    }
}

impl From<GoalCurriculumSnapshotError> for DiffDriveEpisodeSnapshotError {
    fn from(error: GoalCurriculumSnapshotError) -> Self {
        Self::Curriculum(error)
    }
}

/// Completed-tick checkpoint of a [`DiffDriveEpisode`].
///
/// This snapshot is intended to restore an episode created with compatible
/// configuration and the same scene topology. It does not persist the internal
/// recording log.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveEpisodeSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Underlying simulation state snapshot.
    pub simulation: DiffDriveSimSnapshot,
    /// Replay random checkpoint for world and episode RNG state.
    pub random: ReplayRandomSnapshot,
    /// Zero-based episode index.
    pub episode_index: u32,
    /// Completed steps in the current episode.
    pub step_in_episode: u64,
    /// Previous base X position used for reward shaping.
    pub prev_x_m: f64,
    /// Cumulative reward in the current episode.
    pub total_reward: f64,
    /// Active goal X position in meters.
    pub goal_x_m: f64,
    /// Runtime curriculum progress when curriculum training is enabled.
    pub curriculum: Option<GoalCurriculumSnapshot>,
}

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
        let log = simulation_log_for_sim(&sim);
        Self {
            sim,
            config,
            episode_index: 0,
            step_in_episode: 0,
            prev_x_m: 0.0,
            total_reward: 0.0,
            log,
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

    /// Returns a snapshot of the episode-owned randomization stream.
    pub fn random_snapshot(&self) -> EpisodeRandomSnapshot {
        EpisodeRandomSnapshot::new(self.rng.state())
    }

    /// Restores the episode-owned randomization stream from a snapshot.
    pub fn restore_random_snapshot(&mut self, snapshot: EpisodeRandomSnapshot) {
        self.rng = DeterministicRng::from_state(snapshot.rng_state);
    }

    /// Returns a replay checkpoint for deterministic random state.
    pub fn replay_random_snapshot(&self) -> ReplayRandomSnapshot {
        let world_random = self.sim.world_random_snapshot();
        ReplayRandomSnapshot::new(
            self.sim.sim_time(),
            self.sim.step_count(),
            world_random.seed,
            world_random.main_rng_state,
        )
        .with_rng_state(ReplayRngState::new(
            DIFF_DRIVE_EPISODE_RNG_STATE,
            self.rng.state(),
        ))
    }

    /// Appends the current replay random checkpoint to this episode's log.
    pub fn record_random_snapshot(&mut self) {
        let snapshot = self.replay_random_snapshot();
        self.log.record_random_snapshot(snapshot);
    }

    /// Restores deterministic random state from a replay checkpoint.
    ///
    /// This restores the world-level random stream and this episode's owned RNG.
    /// It does not restore ECS transforms, physics state, action buffers, or
    /// reward counters; callers must pair it with a matching simulation state
    /// snapshot for true mid-run resume.
    pub fn restore_replay_random_snapshot(
        &mut self,
        snapshot: &ReplayRandomSnapshot,
    ) -> Result<(), ReplayRandomSnapshotError> {
        snapshot.validate_current_schema()?;
        snapshot.validate_world_seed(self.sim.world_seed())?;
        self.sim.restore_world_random_snapshot(WorldRandomSnapshot {
            seed: snapshot.world_seed,
            main_rng_state: snapshot.world_main_rng_state,
        });
        let rng_state = snapshot.require_rng_state(DIFF_DRIVE_EPISODE_RNG_STATE)?;
        self.rng = DeterministicRng::from_state(rng_state);
        Ok(())
    }

    /// Returns a completed-tick checkpoint for this episode.
    ///
    /// # Errors
    ///
    /// Returns [`DiffDriveEpisodeSnapshotError::Simulation`] if the underlying
    /// simulation cannot be snapshotted, for example because deferred commands
    /// are still pending.
    pub fn checkpoint(&self) -> Result<DiffDriveEpisodeSnapshot, DiffDriveEpisodeSnapshotError> {
        Ok(DiffDriveEpisodeSnapshot {
            schema_version: DIFF_DRIVE_EPISODE_SNAPSHOT_VERSION,
            simulation: self.sim.snapshot()?,
            random: self.replay_random_snapshot(),
            episode_index: self.episode_index,
            step_in_episode: self.step_in_episode,
            prev_x_m: self.prev_x_m,
            total_reward: self.total_reward,
            goal_x_m: self.goal_x_m,
            curriculum: self.curriculum.as_ref().map(GoalCurriculum::snapshot),
        })
    }

    /// Restores this episode from a completed-tick checkpoint.
    ///
    /// # Errors
    ///
    /// Returns an error if the snapshot schema is unsupported, if embedded
    /// simulation/random/curriculum state is incompatible, or if the snapshot's
    /// random checkpoint does not correspond to the embedded simulation tick.
    pub fn restore_checkpoint(
        &mut self,
        snapshot: &DiffDriveEpisodeSnapshot,
    ) -> Result<(), DiffDriveEpisodeSnapshotError> {
        if snapshot.schema_version != DIFF_DRIVE_EPISODE_SNAPSHOT_VERSION {
            return Err(DiffDriveEpisodeSnapshotError::UnsupportedSchemaVersion {
                expected: DIFF_DRIVE_EPISODE_SNAPSHOT_VERSION,
                actual: snapshot.schema_version,
            });
        }
        check_snapshot_match(
            "random.sim_ticks",
            snapshot.simulation.sim_ticks,
            snapshot.random.sim_ticks,
        )?;
        check_snapshot_match(
            "random.sequence",
            snapshot.simulation.step_count,
            snapshot.random.sequence,
        )?;

        self.sim.restore_snapshot(&snapshot.simulation)?;
        self.restore_replay_random_snapshot(&snapshot.random)?;

        self.episode_index = snapshot.episode_index;
        self.step_in_episode = snapshot.step_in_episode;
        self.prev_x_m = snapshot.prev_x_m;
        self.total_reward = snapshot.total_reward;
        self.goal_x_m = snapshot.goal_x_m;

        match (&mut self.curriculum, snapshot.curriculum) {
            (Some(curriculum), Some(curriculum_snapshot)) => {
                curriculum.restore_snapshot(curriculum_snapshot)?;
            }
            (Some(_), None) => return Err(DiffDriveEpisodeSnapshotError::MissingCurriculum),
            (None, Some(_)) => return Err(DiffDriveEpisodeSnapshotError::UnexpectedCurriculum),
            (None, None) => {}
        }

        Ok(())
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
        self.log = simulation_log_for_sim(&self.sim);

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

fn simulation_log_for_sim(sim: &DiffDriveSim) -> SimulationLog {
    SimulationLog::new_with_header(ReplayHeader::new(
        sim.world_seed(),
        WORLD_RANDOM_STREAM_VERSION,
        sim.fixed_delta(),
    ))
}

fn check_snapshot_match<T>(
    field: &'static str,
    expected: T,
    actual: T,
) -> Result<(), DiffDriveEpisodeSnapshotError>
where
    T: Eq + ToString,
{
    if expected == actual {
        Ok(())
    } else {
        Err(DiffDriveEpisodeSnapshotError::Mismatch {
            field,
            expected: expected.to_string(),
            actual: actual.to_string(),
        })
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
    fn recorded_log_includes_replay_header() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            record_log: true,
            scene_path: Some(scene_path),
            ..DiffDriveEpisodeConfig::default()
        });

        env.reset();
        let header = env.log().header().expect("replay header");
        assert_eq!(header.world_seed, 42);
        assert_eq!(
            header.rng_algorithm_version,
            rne_core::DETERMINISTIC_RNG_VERSION
        );
        assert_eq!(header.keyed_random_version, rne_core::KEYED_RANDOM_VERSION);
        assert_eq!(
            header.stream_derivation_version,
            WORLD_RANDOM_STREAM_VERSION
        );
        assert_eq!(
            header.fixed_delta_ticks,
            env.simulation().fixed_delta().ticks()
        );

        env.step(DiffDriveAction::forward(1.0));

        assert_eq!(env.log().records().len(), 2);
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
    fn random_snapshot_restores_goal_sampling_position() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_tasks: Some(GoalTaskSet::new(vec![1.0, 1.5, 2.0, 2.5])),
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });

        let snapshot = env.random_snapshot();
        env.reset();
        let first_goal = env.goal_x_m();
        env.reset();
        env.restore_random_snapshot(snapshot);
        env.reset();

        assert_eq!(env.goal_x_m(), first_goal);
    }

    #[test]
    fn replay_random_snapshot_records_world_and_episode_rng_state() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_tasks: Some(GoalTaskSet::new(vec![1.0, 1.5, 2.0, 2.5])),
            scene_path: Some(scene_path),
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });

        env.reset();
        env.step(DiffDriveAction::forward(1.0));
        let episode_snapshot = env.random_snapshot();
        env.record_random_snapshot();
        let replay_snapshot = env.log().latest_random_snapshot().expect("snapshot");

        assert_eq!(replay_snapshot.world_seed, 42);
        assert_eq!(replay_snapshot.sequence, env.simulation().step_count());
        assert_eq!(
            replay_snapshot.sim_ticks,
            env.simulation().sim_time().ticks()
        );
        assert_eq!(
            replay_snapshot.rng_state(DIFF_DRIVE_EPISODE_RNG_STATE),
            Some(episode_snapshot.rng_state)
        );
    }

    #[test]
    fn replay_random_snapshot_restores_future_random_stream_positions() {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_assets/tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_tasks: Some(GoalTaskSet::new(vec![1.0, 1.5, 2.0, 2.5])),
            scene_path: Some(scene_path),
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });

        env.sim
            .world_mut()
            .resource_mut::<rne_world::WorldRandom>()
            .next_u64();
        let snapshot = env.replay_random_snapshot();
        let expected_world_next = env
            .sim
            .world_mut()
            .resource_mut::<rne_world::WorldRandom>()
            .next_u64();
        env.reset();
        let expected_goal = env.goal_x_m();
        env.reset();
        env.sim
            .world_mut()
            .resource_mut::<rne_world::WorldRandom>()
            .next_u64();

        env.restore_replay_random_snapshot(&snapshot).unwrap();
        let restored_world_next = env
            .sim
            .world_mut()
            .resource_mut::<rne_world::WorldRandom>()
            .next_u64();
        env.reset();

        assert_eq!(restored_world_next, expected_world_next);
        assert_eq!(env.goal_x_m(), expected_goal);
    }

    #[test]
    fn replay_random_snapshot_rejects_wrong_world_seed() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_tasks: Some(GoalTaskSet::new(vec![1.0, 1.5])),
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });
        let snapshot = ReplayRandomSnapshot::new(
            env.simulation().sim_time(),
            env.simulation().step_count(),
            99,
            99,
        )
        .with_rng_state(ReplayRngState::new(DIFF_DRIVE_EPISODE_RNG_STATE, 1));

        let error = env.restore_replay_random_snapshot(&snapshot).unwrap_err();

        assert!(matches!(
            error,
            ReplayRandomSnapshotError::Mismatch {
                field: "world_seed",
                ..
            }
        ));
    }

    #[test]
    fn checkpoint_restores_episode_state_and_next_step() {
        use rne_robot::{DiffDriveConfig, DiffDriveDriveMode};

        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_x_m: 100.0,
            max_steps: 20,
            ..DiffDriveEpisodeConfig::default()
        });

        env.reset();
        env.sim = DiffDriveSim::with_robot_configs(&[DiffDriveConfig {
            drive_mode: DiffDriveDriveMode::Kinematic,
            ..DiffDriveConfig::default()
        }]);
        env.step(DiffDriveAction::forward(2.0));
        let checkpoint = env.checkpoint().unwrap();
        let observation_at_checkpoint = env.observe_with_current_goal();
        let total_at_checkpoint = env.total_reward();
        let expected_next = env.step(DiffDriveAction::forward(0.0));

        env.step(DiffDriveAction::forward(6.0));
        env.step(DiffDriveAction::forward(-2.0));

        env.restore_checkpoint(&checkpoint).unwrap();
        assert_eq!(env.observe_with_current_goal(), observation_at_checkpoint);
        assert_eq!(env.total_reward(), total_at_checkpoint);
        assert_eq!(env.step_in_episode(), checkpoint.step_in_episode);

        let restored_next = env.step(DiffDriveAction::forward(0.0));
        assert_eq!(restored_next, expected_next);
    }

    #[test]
    fn checkpoint_restores_curriculum_progress() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_curriculum: Some(GoalCurriculumConfig::easy_to_hard()),
            rng_seed: 5,
            ..DiffDriveEpisodeConfig::default()
        });
        let curriculum = env.curriculum.as_mut().expect("curriculum");
        assert!(!curriculum.record_episode_end(true));
        let checkpoint = env.checkpoint().unwrap();
        assert!(env
            .curriculum
            .as_mut()
            .expect("curriculum")
            .record_episode_end(true));

        env.restore_checkpoint(&checkpoint).unwrap();

        let curriculum = env.curriculum.as_ref().expect("curriculum");
        assert_eq!(curriculum.stage_index(), 0);
        assert_eq!(curriculum.successes_in_stage(), 1);
    }

    #[test]
    fn checkpoint_rejects_mismatched_random_tick() {
        let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig::default());
        let mut checkpoint = env.checkpoint().unwrap();
        checkpoint.random.sim_ticks += 1;

        let error = env.restore_checkpoint(&checkpoint).unwrap_err();

        assert!(matches!(
            error,
            DiffDriveEpisodeSnapshotError::Mismatch {
                field: "random.sim_ticks",
                ..
            }
        ));
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
