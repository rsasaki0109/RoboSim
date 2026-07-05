//! Mobile manipulator episode environment.

use super::sim::{
    MobileManipulatorSim, MobileManipulatorSimSnapshot, MobileManipulatorSimSnapshotError,
};
use crate::action::MobileManipulatorAction;
use crate::episode::{Episode, EpisodeRandomSnapshot, EpisodeStep};
use crate::grasp::finger_contacts_named;
use crate::observation::MobileManipulatorObservation;
use crate::reach::{
    ee_distance_to_target_m, ReachCurriculumSnapshot, ReachCurriculumSnapshotError,
};
use crate::reward::{MobileManipulatorRewardConfig, MobileManipulatorTask};
use crate::transport::{
    body_moved_at_least_m, body_within_zone_m, had_finger_contact, named_translation_m,
    TRANSPORT_SUCCESS_M,
};
use rne_log::{ReplayRandomSnapshot, ReplayRandomSnapshotError, ReplayRngState};
use rne_world::WorldRandomSnapshot;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const MOBILE_MANIPULATOR_EPISODE_RNG_STATE: &str = "mobile_manipulator_episode";
const MOBILE_MANIPULATOR_EPISODE_SNAPSHOT_VERSION: u32 = 1;

/// Error restoring or creating a mobile-manipulator episode snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum MobileManipulatorEpisodeSnapshotError {
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
    Simulation(MobileManipulatorSimSnapshotError),
    /// The embedded random checkpoint failed.
    Random(ReplayRandomSnapshotError),
    /// Snapshot does not contain curriculum state required by this episode.
    MissingCurriculum,
    /// Snapshot contains curriculum state but this episode has no curriculum.
    UnexpectedCurriculum,
    /// The embedded curriculum state failed.
    Curriculum(ReachCurriculumSnapshotError),
}

impl From<MobileManipulatorSimSnapshotError> for MobileManipulatorEpisodeSnapshotError {
    fn from(error: MobileManipulatorSimSnapshotError) -> Self {
        Self::Simulation(error)
    }
}

impl From<ReplayRandomSnapshotError> for MobileManipulatorEpisodeSnapshotError {
    fn from(error: ReplayRandomSnapshotError) -> Self {
        Self::Random(error)
    }
}

impl From<ReachCurriculumSnapshotError> for MobileManipulatorEpisodeSnapshotError {
    fn from(error: ReachCurriculumSnapshotError) -> Self {
        Self::Curriculum(error)
    }
}

/// Reward-progress checkpoint for a mobile-manipulator episode.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorEpisodeProgressSnapshot {
    /// Previous end-effector error used for Reach reward shaping.
    pub ee_error_m: f64,
    /// Initial object position used for Transport reward shaping.
    pub object_initial: Option<(f64, f64, f64)>,
    /// Whether the gripper has contacted the task object.
    pub contacted_object: bool,
    /// Previous object-to-target distance used for Place reward shaping.
    pub place_error_m: f64,
    /// Whether the object has been grasped at least once this episode.
    pub was_grasped: bool,
    /// Previous end-effector-to-object distance used before grasp (Place).
    #[serde(default)]
    pub approach_error_m: f64,
    /// Previous base-to-object horizontal distance used before grasp on mobile robots.
    #[serde(default)]
    pub base_approach_error_m: f64,
}

/// Completed-tick checkpoint of a [`MobileManipulatorEpisode`].
///
/// This snapshot is intended to restore an episode created with compatible
/// configuration and the same scene topology. It does not persist an external
/// recording log.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobileManipulatorEpisodeSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Underlying simulation state snapshot.
    pub simulation: MobileManipulatorSimSnapshot,
    /// Replay random checkpoint for world and episode RNG state.
    pub random: ReplayRandomSnapshot,
    /// Zero-based episode index.
    pub episode_index: u32,
    /// Completed steps in the current episode.
    pub step_in_episode: u64,
    /// Cumulative reward in the current episode.
    pub total_reward: f64,
    /// Task currently active in this episode, including sampled Reach targets.
    pub effective_task: MobileManipulatorTask,
    /// Runtime reward-progress state.
    pub progress_state: MobileManipulatorEpisodeProgressSnapshot,
    /// Runtime reach-curriculum progress when curriculum training is enabled.
    pub reach_curriculum: Option<ReachCurriculumSnapshot>,
}

/// Randomized pick target among several named clutter objects.
#[derive(Clone, Debug, PartialEq)]
pub struct ClutterPickConfig {
    /// Candidate object entity names in the scene.
    pub object_names: Vec<String>,
    /// World-frame placement target after grasp.
    pub target: crate::reach::ReachTarget,
    /// Horizontal success tolerance in meters.
    pub place_tolerance_m: f64,
}

/// Configuration for a mobile manipulator manipulation episode.
#[derive(Clone, Debug, PartialEq)]
pub struct MobileManipulatorEpisodeConfig {
    /// Maximum steps before truncation.
    pub max_steps: u64,
    /// Scene asset path loaded on reset.
    pub scene_path: PathBuf,
    /// Task definition and success criteria.
    pub task: MobileManipulatorTask,
    /// Reward weights applied each step.
    pub reward: MobileManipulatorRewardConfig,
    /// When set (Reach task only), a fresh target is sampled from this region each reset.
    pub reach_randomization: Option<crate::reach::ReachRandomization>,
    /// When set (Reach task only), targets are sampled from a curriculum that widens as
    /// the policy succeeds (takes precedence over `reach_randomization`).
    pub reach_curriculum: Option<crate::reach::ReachCurriculumConfig>,
    /// When set, a random object from the list becomes the Place target each reset.
    pub clutter_pick: Option<ClutterPickConfig>,
    /// Seed for per-episode randomization.
    pub rng_seed: u64,
}

impl MobileManipulatorEpisodeConfig {
    /// Default transport episode on the built-in transport scene.
    pub fn transport() -> Self {
        Self {
            max_steps: 900,
            scene_path: crate::mm_minimal_transport_scene_path(),
            task: MobileManipulatorTask::Transport {
                object_name: "grasp_cube".into(),
                drop_zone_name: "drop_zone".into(),
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    /// End-effector reach episode toward a reachable world-frame target (dense reward).
    ///
    /// Suited to reinforcement learning: the per-step reward is the reduction in
    /// end-effector distance to the target, with a bonus on success.
    pub fn reach() -> Self {
        Self {
            max_steps: 300,
            scene_path: crate::mm_minimal_scene_path(),
            task: MobileManipulatorTask::Reach {
                target: crate::reach::ReachTarget::new(0.32, 0.64, 0.40),
                success_m: 0.1,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    /// Goal-conditioned reach: a fresh reachable target is sampled each episode and
    /// exposed in the observation (`target_d{x,y,z}_m`), so a policy must generalize
    /// rather than memorize one pose.
    pub fn reach_randomized(rng_seed: u64) -> Self {
        // The arm is a horizontal SCARA: it controls the end-effector in X/Z (via the
        // shoulder/elbow) but barely in Y, so the target Y stays near the natural EE
        // height and only X/Z are randomized.
        let randomization = crate::reach::ReachRandomization {
            min: crate::reach::ReachTarget::new(0.34, 0.585, 0.18),
            max: crate::reach::ReachTarget::new(0.46, 0.595, 0.36),
            success_m: 0.12,
        };
        Self {
            max_steps: 500,
            scene_path: crate::mm_minimal_scene_path(),
            task: MobileManipulatorTask::Reach {
                // Placeholder; replaced by a sampled target on every reset.
                target: crate::reach::ReachTarget::new(0.40, 0.59, 0.27),
                success_m: randomization.success_m,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: Some(randomization),
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed,
        }
    }

    /// Goal-conditioned reach with an easy→hard curriculum that widens the target region
    /// as the policy accumulates successes.
    pub fn reach_curriculum(rng_seed: u64) -> Self {
        Self {
            reach_randomization: None,
            reach_curriculum: Some(crate::reach::ReachCurriculumConfig::easy_to_hard()),
            ..Self::reach_randomized(rng_seed)
        }
    }

    /// Default pick-and-place episode: grasp the cube and set it down at a target.
    pub fn place() -> Self {
        Self {
            max_steps: 600,
            scene_path: crate::mm_minimal_transport_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "grasp_cube".into(),
                target: crate::reach::ReachTarget::new(1.23, 0.03, -0.53),
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    /// Pick one of three clutter cubes and place it at the fixed-base ground target.
    pub fn clutter_pick_place(rng_seed: u64) -> Self {
        let target = crate::mm_minimal_kinematics::mm_minimal_clutter_place_target();
        Self {
            max_steps: 960,
            scene_path: crate::mm_minimal_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "clutter_cube_a".into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: Some(ClutterPickConfig {
                object_names: vec![
                    "clutter_cube_a".into(),
                    "clutter_cube_b".into(),
                    "clutter_cube_c".into(),
                ],
                target,
                place_tolerance_m: 0.12,
            }),
            rng_seed,
        }
    }

    /// Fixed-base clutter place on the center cube (`clutter_cube_b`) at the ground target.
    ///
    /// Pins the pick target for reproducible RL benches (example 33 and `train_clutter.py`).
    pub fn clutter_pick_place_center(rng_seed: u64) -> Self {
        let target = crate::mm_minimal_kinematics::mm_minimal_clutter_place_target();
        Self {
            max_steps: 960,
            scene_path: crate::mm_minimal_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "clutter_cube_b".into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed,
        }
    }

    /// Navigate a diff-drive base to one of three distant cubes, then pick and place it.
    pub fn mobile_clutter_pick_place(rng_seed: u64) -> Self {
        let target = crate::mm_minimal_kinematics::mm_mobile_clutter_place_target();
        Self {
            max_steps: 1600,
            scene_path: crate::mm_mobile_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "clutter_cube_a".into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: Some(ClutterPickConfig {
                object_names: vec![
                    "clutter_cube_a".into(),
                    "clutter_cube_b".into(),
                    "clutter_cube_c".into(),
                ],
                target,
                place_tolerance_m: 0.12,
            }),
            rng_seed,
        }
    }

    /// Mobile navigate-and-place on the left clutter cube (`clutter_cube_a`).
    pub fn mobile_clutter_pick_place_center(rng_seed: u64) -> Self {
        let target = crate::mm_minimal_kinematics::mm_mobile_clutter_place_target();
        Self {
            max_steps: 1600,
            scene_path: crate::mm_mobile_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "clutter_cube_a".into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed,
        }
    }

    /// Vertical pick-and-place on the `mm_lift` robot: lower the top-down claw over a
    /// cube on the ground, grasp it, lift it, carry it to a target, and set it down.
    ///
    /// Unlike [`Self::place`] (a horizontal carry on the SCARA arm), this uses the
    /// lift robot's full 3D motion: the cube is picked off the ground and placed at a
    /// different spot. The target matches where the scripted pick-place trajectory
    /// lands the cube (see example 31).
    pub fn lift_pick_place() -> Self {
        Self {
            max_steps: 1200,
            scene_path: crate::mm_lift_pick_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "lift_cube".into(),
                target: crate::reach::ReachTarget::new(0.55, 0.03, -0.87),
                place_tolerance_m: 0.2,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    /// Default inspect episode on the built-in minimal scene.
    pub fn inspect() -> Self {
        Self {
            max_steps: 240,
            scene_path: crate::mm_minimal_scene_path(),
            task: MobileManipulatorTask::Inspect {
                min_wrist_pixels: 64 * 48 * 4,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }
}

/// Manipulation episode built on top of [`MobileManipulatorSim`].
pub struct MobileManipulatorEpisode {
    sim: MobileManipulatorSim,
    config: MobileManipulatorEpisodeConfig,
    /// Task actually used this episode (the config task, but with a sampled Reach target
    /// when randomization is enabled).
    effective_task: MobileManipulatorTask,
    rng: crate::rng::DeterministicRng,
    reach_curriculum: Option<crate::reach::ReachCurriculum>,
    episode_index: u32,
    step_in_episode: u64,
    total_reward: f64,
    progress_state: EpisodeProgressState,
}

#[derive(Clone, Debug, Default)]
struct EpisodeProgressState {
    ee_error_m: f64,
    object_initial: Option<(f64, f64, f64)>,
    contacted_object: bool,
    /// Horizontal object-to-target distance from the previous step (Place shaping).
    place_error_m: f64,
    /// True once the object has been grasped at least once this episode (Place).
    was_grasped: bool,
    /// End-effector distance to the task object before grasp (Place shaping).
    approach_error_m: f64,
    /// Base horizontal distance to the task object before grasp on mobile robots.
    base_approach_error_m: f64,
}

impl EpisodeProgressState {
    fn snapshot(&self) -> MobileManipulatorEpisodeProgressSnapshot {
        MobileManipulatorEpisodeProgressSnapshot {
            ee_error_m: self.ee_error_m,
            object_initial: self.object_initial,
            contacted_object: self.contacted_object,
            place_error_m: self.place_error_m,
            was_grasped: self.was_grasped,
            approach_error_m: self.approach_error_m,
            base_approach_error_m: self.base_approach_error_m,
        }
    }

    fn restore(snapshot: MobileManipulatorEpisodeProgressSnapshot) -> Self {
        Self {
            ee_error_m: snapshot.ee_error_m,
            object_initial: snapshot.object_initial,
            contacted_object: snapshot.contacted_object,
            place_error_m: snapshot.place_error_m,
            was_grasped: snapshot.was_grasped,
            approach_error_m: snapshot.approach_error_m,
            base_approach_error_m: snapshot.base_approach_error_m,
        }
    }
}

impl MobileManipulatorEpisode {
    /// Creates a new episode environment with the given configuration.
    pub fn new(config: MobileManipulatorEpisodeConfig) -> Self {
        let sim =
            MobileManipulatorSim::from_scene_path(&config.scene_path).expect("episode simulation");
        let effective_task = config.task.clone();
        let rng = crate::rng::DeterministicRng::new(config.rng_seed);
        let reach_curriculum = config
            .reach_curriculum
            .clone()
            .map(crate::reach::ReachCurriculum::new);
        let progress_state = initial_progress_state(&sim, &effective_task);
        Self {
            sim,
            config,
            effective_task,
            rng,
            reach_curriculum,
            episode_index: 0,
            step_in_episode: 0,
            total_reward: 0.0,
            progress_state,
        }
    }

    /// Fills the goal-relative offset (`target_d{x,y,z}_m`) in the observation so a
    /// policy can see where to go:
    /// - Reach: the target relative to the end-effector.
    /// - Place: before grasping, the object relative to the end-effector (where to reach
    ///   to pick); once grasped, the place target relative to the object (where to carry).
    fn fill_goal_delta(&self, observation: &mut MobileManipulatorObservation) {
        match &self.effective_task {
            MobileManipulatorTask::Reach { target, .. } => {
                observation.target_dx_m = target.x_m - observation.ee_x_m;
                observation.target_dy_m = target.y_m - observation.ee_y_m;
                observation.target_dz_m = target.z_m - observation.ee_z_m;
            }
            MobileManipulatorTask::Place {
                object_name,
                target,
                ..
            } => {
                if self.sim.is_grasping() {
                    if let Some((ox, oy, oz)) = named_translation_m(&self.sim, object_name) {
                        observation.target_dx_m = target.x_m - ox;
                        observation.target_dy_m = target.y_m - oy;
                        observation.target_dz_m = target.z_m - oz;
                    }
                } else if let Some((ox, oy, oz)) = named_translation_m(&self.sim, object_name) {
                    observation.pick_object_x_m = ox;
                    observation.pick_object_y_m = oy;
                    observation.pick_object_z_m = oz;
                    observation.target_dx_m = ox - observation.ee_x_m;
                    observation.target_dy_m = oy - observation.ee_y_m;
                    observation.target_dz_m = oz - observation.ee_z_m;
                    if let Some((gx, gy, gz)) = self.sim.link_translation_m("gripper_base_link") {
                        observation.gripper_target_dx_m = ox - gx;
                        observation.gripper_target_dy_m = oy - gy;
                        observation.gripper_target_dz_m = oz - gz;
                    } else {
                        observation.gripper_target_dx_m = observation.target_dx_m;
                        observation.gripper_target_dy_m = observation.target_dy_m;
                        observation.gripper_target_dz_m = observation.target_dz_m;
                    }
                }
                if let Some(clutter) = &self.config.clutter_pick {
                    observation.target_object_index = clutter
                        .object_names
                        .iter()
                        .position(|name| name == object_name)
                        .unwrap_or(0) as u32;
                }
            }
            _ => {}
        }
    }

    /// Returns read access to the underlying simulation.
    pub fn simulation(&self) -> &MobileManipulatorSim {
        &self.sim
    }

    /// Returns cumulative reward for the current episode.
    pub fn total_reward(&self) -> f64 {
        self.total_reward
    }

    /// Returns the active reach-curriculum stage index, if a curriculum is configured.
    pub fn curriculum_stage(&self) -> Option<usize> {
        self.reach_curriculum
            .as_ref()
            .map(|curriculum| curriculum.stage_index())
    }

    /// Returns a snapshot of the episode-owned randomization stream.
    pub fn random_snapshot(&self) -> EpisodeRandomSnapshot {
        EpisodeRandomSnapshot::new(self.rng.state())
    }

    /// Restores the episode-owned randomization stream from a snapshot.
    pub fn restore_random_snapshot(&mut self, snapshot: EpisodeRandomSnapshot) {
        self.rng = crate::rng::DeterministicRng::from_state(snapshot.rng_state);
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
            MOBILE_MANIPULATOR_EPISODE_RNG_STATE,
            self.rng.state(),
        ))
    }

    /// Restores deterministic random state from a replay checkpoint.
    ///
    /// This restores the world-level random stream and this episode's owned RNG.
    /// It does not restore ECS transforms, physics state, or reward counters;
    /// callers must pair it with a matching simulation snapshot for true
    /// mid-run resume.
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
        let rng_state = snapshot.require_rng_state(MOBILE_MANIPULATOR_EPISODE_RNG_STATE)?;
        self.rng = crate::rng::DeterministicRng::from_state(rng_state);
        Ok(())
    }

    /// Returns a completed-tick checkpoint for this episode.
    pub fn checkpoint(&self) -> MobileManipulatorEpisodeSnapshot {
        MobileManipulatorEpisodeSnapshot {
            schema_version: MOBILE_MANIPULATOR_EPISODE_SNAPSHOT_VERSION,
            simulation: self.sim.snapshot(),
            random: self.replay_random_snapshot(),
            episode_index: self.episode_index,
            step_in_episode: self.step_in_episode,
            total_reward: self.total_reward,
            effective_task: self.effective_task.clone(),
            progress_state: self.progress_state.snapshot(),
            reach_curriculum: self
                .reach_curriculum
                .as_ref()
                .map(|curriculum| curriculum.snapshot()),
        }
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
        snapshot: &MobileManipulatorEpisodeSnapshot,
    ) -> Result<(), MobileManipulatorEpisodeSnapshotError> {
        if snapshot.schema_version != MOBILE_MANIPULATOR_EPISODE_SNAPSHOT_VERSION {
            return Err(
                MobileManipulatorEpisodeSnapshotError::UnsupportedSchemaVersion {
                    expected: MOBILE_MANIPULATOR_EPISODE_SNAPSHOT_VERSION,
                    actual: snapshot.schema_version,
                },
            );
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
        self.total_reward = snapshot.total_reward;
        self.effective_task = snapshot.effective_task.clone();
        self.progress_state = EpisodeProgressState::restore(snapshot.progress_state.clone());

        match (&mut self.reach_curriculum, snapshot.reach_curriculum) {
            (Some(curriculum), Some(curriculum_snapshot)) => {
                curriculum.restore_snapshot(curriculum_snapshot)?;
            }
            (Some(_), None) => {
                return Err(MobileManipulatorEpisodeSnapshotError::MissingCurriculum)
            }
            (None, Some(_)) => {
                return Err(MobileManipulatorEpisodeSnapshotError::UnexpectedCurriculum);
            }
            (None, None) => {}
        }

        Ok(())
    }

    fn make_step(
        &mut self,
        mut observation: MobileManipulatorObservation,
    ) -> EpisodeStep<MobileManipulatorObservation> {
        let progress = task_progress(
            &self.effective_task,
            &observation,
            &mut self.progress_state,
            &self.sim,
        );
        let success = task_success(
            &self.effective_task,
            &observation,
            &self.progress_state,
            &self.sim,
        );
        let truncated = !success && self.step_in_episode >= self.config.max_steps;
        let reward = self.config.reward.compute(progress, success);
        self.total_reward += reward;

        self.fill_goal_delta(&mut observation);

        EpisodeStep {
            observation,
            reward,
            terminated: success,
            truncated,
        }
    }
}

impl Episode for MobileManipulatorEpisode {
    type Observation = MobileManipulatorObservation;
    type Action = MobileManipulatorAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = MobileManipulatorSim::from_scene_path(&self.config.scene_path)
            .expect("reload episode simulation");
        self.episode_index += 1;
        self.step_in_episode = 0;
        self.total_reward = 0.0;

        // Sample a fresh reach target when goal-conditioned randomization/curriculum is on.
        self.effective_task = self.config.task.clone();
        if let MobileManipulatorTask::Reach { target, success_m } = &mut self.effective_task {
            if let Some(curriculum) = &self.reach_curriculum {
                let (sampled, sampled_success_m) = curriculum.sample(&mut self.rng);
                *target = sampled;
                *success_m = sampled_success_m;
            } else if let Some(randomization) = self.config.reach_randomization {
                *target = randomization.sample(&mut self.rng);
                *success_m = randomization.success_m;
            }
        } else if let Some(clutter) = &self.config.clutter_pick {
            let index = self.rng.uniform_usize(clutter.object_names.len());
            let object_name = clutter.object_names[index].clone();
            self.effective_task = MobileManipulatorTask::Place {
                object_name,
                target: clutter.target,
                place_tolerance_m: clutter.place_tolerance_m,
            };
        }

        self.progress_state = initial_progress_state(&self.sim, &self.effective_task);

        let mut observation = self.sim.observe();
        self.progress_state.ee_error_m = initial_ee_error(&self.effective_task, &observation);
        self.fill_goal_delta(&mut observation);

        EpisodeStep {
            observation,
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        self.step_in_episode += 1;
        self.sim.step(action);
        let observation = self.sim.observe();
        let result = self.make_step(observation);
        if result.terminated {
            if let Some(curriculum) = self.reach_curriculum.as_mut() {
                curriculum.record_episode_end(true);
            }
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

fn check_snapshot_match<T>(
    field: &'static str,
    expected: T,
    actual: T,
) -> Result<(), MobileManipulatorEpisodeSnapshotError>
where
    T: Eq + ToString,
{
    if expected == actual {
        Ok(())
    } else {
        Err(MobileManipulatorEpisodeSnapshotError::Mismatch {
            field,
            expected: expected.to_string(),
            actual: actual.to_string(),
        })
    }
}

fn ee_object_distance_m(
    observation: &MobileManipulatorObservation,
    object: (f64, f64, f64),
) -> f64 {
    let dx = observation.ee_x_m - object.0;
    let dy = observation.ee_y_m - object.1;
    let dz = observation.ee_z_m - object.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn base_object_horizontal_distance_m(
    observation: &MobileManipulatorObservation,
    object: (f64, f64, f64),
) -> f64 {
    let dx = observation.base_x_m - object.0;
    let dz = observation.base_z_m - object.2;
    (dx * dx + dz * dz).sqrt()
}

fn initial_progress_state(
    sim: &MobileManipulatorSim,
    task: &MobileManipulatorTask,
) -> EpisodeProgressState {
    let observation = sim.observe();
    let object_initial = match task {
        MobileManipulatorTask::Transport { object_name, .. }
        | MobileManipulatorTask::Place { object_name, .. } => named_translation_m(sim, object_name),
        _ => None,
    };
    let place_error_m = match task {
        MobileManipulatorTask::Place {
            object_name,
            target,
            ..
        } => object_horizontal_distance_to_target_m(sim, object_name, *target).unwrap_or(0.0),
        _ => 0.0,
    };
    let (approach_error_m, base_approach_error_m) = match task {
        MobileManipulatorTask::Place { .. } => object_initial
            .map(|object| {
                (
                    ee_object_distance_m(&observation, object),
                    base_object_horizontal_distance_m(&observation, object),
                )
            })
            .unwrap_or((0.0, 0.0)),
        _ => (0.0, 0.0),
    };
    EpisodeProgressState {
        object_initial,
        place_error_m,
        approach_error_m,
        base_approach_error_m,
        ..EpisodeProgressState::default()
    }
}

fn initial_ee_error(task: &MobileManipulatorTask, obs: &MobileManipulatorObservation) -> f64 {
    match task {
        MobileManipulatorTask::Reach { target, .. } => ee_distance_to_target_m(obs, *target),
        _ => 0.0,
    }
}

/// Horizontal (XZ-plane) distance from a named body to a world-frame target.
fn object_horizontal_distance_to_target_m(
    sim: &MobileManipulatorSim,
    object_name: &str,
    target: crate::reach::ReachTarget,
) -> Option<f64> {
    named_translation_m(sim, object_name).map(|(x, _, z)| {
        let dx = x - target.x_m;
        let dz = z - target.z_m;
        (dx * dx + dz * dz).sqrt()
    })
}

fn task_progress(
    task: &MobileManipulatorTask,
    observation: &MobileManipulatorObservation,
    state: &mut EpisodeProgressState,
    sim: &MobileManipulatorSim,
) -> f64 {
    match task {
        MobileManipulatorTask::Reach { target, .. } => {
            let error = ee_distance_to_target_m(observation, *target);
            let progress = state.ee_error_m - error;
            state.ee_error_m = error;
            progress
        }
        MobileManipulatorTask::Grasp { object_name } => {
            if finger_contacts_named(sim, object_name) {
                1.0
            } else {
                0.0
            }
        }
        MobileManipulatorTask::Transport { object_name, .. } => {
            state.contacted_object = had_finger_contact(sim, object_name, state.contacted_object);
            if let (Some(initial), Some(current)) =
                (state.object_initial, named_translation_m(sim, object_name))
            {
                let dx = current.0 - initial.0;
                let dz = current.2 - initial.2;
                (dx * dx + dz * dz).sqrt()
            } else {
                0.0
            }
        }
        MobileManipulatorTask::Place {
            object_name,
            target,
            ..
        } => {
            state.was_grasped |= sim.is_grasping();
            if !state.was_grasped {
                let mut progress = 0.0;
                if let Some(object) = named_translation_m(sim, object_name) {
                    let ee_dist = ee_object_distance_m(observation, object);
                    progress += state.approach_error_m - ee_dist;
                    state.approach_error_m = ee_dist;
                    if sim.mobile_base() {
                        let base_dist = base_object_horizontal_distance_m(observation, object);
                        progress += 0.5 * (state.base_approach_error_m - base_dist);
                        state.base_approach_error_m = base_dist;
                    }
                }
                progress
            } else if let Some(current) =
                object_horizontal_distance_to_target_m(sim, object_name, *target)
            {
                let progress = state.place_error_m - current;
                state.place_error_m = current;
                progress
            } else {
                0.0
            }
        }
        MobileManipulatorTask::Inspect { .. } => {
            if observation.wrist_camera_pixels > 0 {
                1.0
            } else {
                0.0
            }
        }
    }
}

fn task_success(
    task: &MobileManipulatorTask,
    observation: &MobileManipulatorObservation,
    state: &EpisodeProgressState,
    sim: &MobileManipulatorSim,
) -> bool {
    match task {
        MobileManipulatorTask::Reach { target, success_m } => {
            ee_distance_to_target_m(observation, *target) < *success_m
        }
        MobileManipulatorTask::Grasp { object_name } => finger_contacts_named(sim, object_name),
        MobileManipulatorTask::Transport {
            object_name,
            drop_zone_name,
        } => {
            state.contacted_object
                && state.object_initial.is_some_and(|initial| {
                    body_within_zone_m(sim, object_name, drop_zone_name, 0.08)
                        || body_moved_at_least_m(sim, object_name, initial, TRANSPORT_SUCCESS_M)
                })
        }
        MobileManipulatorTask::Place {
            object_name,
            target,
            place_tolerance_m,
        } => {
            // Picked up, carried, released, and now resting near the target.
            state.was_grasped
                && !sim.is_grasping()
                && object_horizontal_distance_to_target_m(sim, object_name, *target)
                    .is_some_and(|distance| distance < *place_tolerance_m)
                && named_translation_m(sim, object_name)
                    .is_some_and(|(_, y, _)| y < PLACE_RESTING_Y_M)
        }
        MobileManipulatorTask::Inspect { min_wrist_pixels } => {
            observation.wrist_camera_pixels >= *min_wrist_pixels
                && observation.shoulder_position_rad.abs() > 0.05
        }
    }
}

/// Maximum object height to count as "set down" for a Place task success.
const PLACE_RESTING_Y_M: f64 = 0.1;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reach::{reach_action_proportional, ReachTarget};

    #[test]
    fn inspect_episode_publishes_wrist_camera() {
        let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::inspect());
        let _ = episode.reset();
        for _ in 0..120 {
            let step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: 2.0,
                ..MobileManipulatorAction::default()
            });
            if step.terminated {
                return;
            }
        }
        panic!("expected inspect episode to terminate with wrist camera frames");
    }

    #[test]
    fn transport_episode_moves_object_to_drop_zone() {
        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let transport = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 4.0,
            ..MobileManipulatorAction::default()
        };

        for _ in 0..3 {
            let mut episode =
                MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::transport());
            let _ = episode.reset();
            for _ in 0..120 {
                episode.step(close);
            }
            for _ in 0..720 {
                let step = episode.step(transport);
                if step.terminated {
                    return;
                }
            }
        }

        panic!("expected transport episode success within retry budget");
    }

    #[test]
    fn reach_preset_episode_is_solvable_and_needs_control() {
        // A solving control reaches the target...
        let mut solved = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach());
        let _ = solved.reset();
        let drive = MobileManipulatorAction {
            shoulder_velocity_rad_s: -3.0,
            ..MobileManipulatorAction::default()
        };
        let mut terminated = false;
        for _ in 0..300 {
            if solved.step(drive).terminated {
                terminated = true;
                break;
            }
        }
        assert!(
            terminated,
            "expected reach preset to be solvable under control"
        );

        // ...while doing nothing does not (the target needs active control).
        let mut idle = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach());
        let _ = idle.reset();
        let mut idle_terminated = false;
        for _ in 0..300 {
            if idle.step(MobileManipulatorAction::default()).terminated {
                idle_terminated = true;
                break;
            }
        }
        assert!(
            !idle_terminated,
            "reach preset should not be solved by a zero-action policy"
        );
    }

    #[test]
    fn goal_conditioned_reach_generalizes_across_sampled_targets() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_randomized(11));

        // A goal-conditioned proportional policy uses only the observation's goal delta.
        let policy = |obs: &MobileManipulatorObservation| MobileManipulatorAction {
            shoulder_velocity_rad_s: (2.5 * obs.target_dx_m - 0.5 * obs.target_dy_m)
                .clamp(-6.0, 6.0),
            elbow_velocity_rad_s: (1.5 * obs.target_dx_m + 3.0 * obs.target_dz_m).clamp(-6.0, 6.0),
            ..MobileManipulatorAction::default()
        };

        let mut goal_deltas = Vec::new();
        for _ in 0..3 {
            let reset = episode.reset();
            goal_deltas.push((reset.observation.target_dx_m, reset.observation.target_dz_m));
            let mut obs = reset.observation;
            let mut reached = false;
            for _ in 0..500 {
                let step = episode.step(policy(&obs));
                obs = step.observation;
                if step.terminated {
                    reached = true;
                    break;
                }
            }
            assert!(
                reached,
                "goal-conditioned policy should reach the sampled target"
            );
        }

        // Targets must actually vary between episodes (otherwise it is not generalizing).
        assert!(
            goal_deltas
                .windows(2)
                .any(|pair| (pair[0].0 - pair[1].0).abs() > 1e-6
                    || (pair[0].1 - pair[1].1).abs() > 1e-6),
            "expected sampled reach targets to differ across resets"
        );
    }

    #[test]
    fn random_snapshot_restores_reach_target_sampling_position() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_randomized(11));

        let snapshot = episode.random_snapshot();
        let _ = episode.reset();
        let first_target = match episode.effective_task {
            MobileManipulatorTask::Reach { target, .. } => target,
            _ => panic!("expected reach task"),
        };
        let _ = episode.reset();
        episode.restore_random_snapshot(snapshot);
        let _ = episode.reset();
        let restored_target = match episode.effective_task {
            MobileManipulatorTask::Reach { target, .. } => target,
            _ => panic!("expected reach task"),
        };

        assert_eq!(restored_target, first_target);
    }

    #[test]
    fn checkpoint_restores_episode_state() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_randomized(11));
        let _ = episode.reset();
        episode.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: 0.5,
            elbow_velocity_rad_s: -0.25,
            ..MobileManipulatorAction::default()
        });

        let checkpoint = episode.checkpoint();
        let observation_at_checkpoint = episode.simulation().observe();
        let total_at_checkpoint = episode.total_reward();

        let _ = episode.reset();
        episode.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: -1.0,
            elbow_velocity_rad_s: 0.75,
            ..MobileManipulatorAction::default()
        });

        episode.restore_checkpoint(&checkpoint).unwrap();

        assert_eq!(episode.simulation().observe(), observation_at_checkpoint);
        assert_eq!(episode.total_reward(), total_at_checkpoint);
        assert_eq!(episode.step_in_episode(), checkpoint.step_in_episode);
        assert_eq!(episode.checkpoint(), checkpoint);
    }

    #[test]
    fn checkpoint_rejects_mismatched_random_tick() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_randomized(11));
        let _ = episode.reset();
        let mut checkpoint = episode.checkpoint();
        checkpoint.random.sim_ticks += 1;

        let error = episode.restore_checkpoint(&checkpoint).unwrap_err();

        assert_eq!(
            error,
            MobileManipulatorEpisodeSnapshotError::Mismatch {
                field: "random.sim_ticks",
                expected: checkpoint.simulation.sim_ticks.to_string(),
                actual: checkpoint.random.sim_ticks.to_string()
            }
        );
    }

    #[test]
    fn reach_curriculum_advances_through_stages() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_curriculum(5));
        let policy = |obs: &MobileManipulatorObservation| MobileManipulatorAction {
            shoulder_velocity_rad_s: (2.5 * obs.target_dx_m - 0.5 * obs.target_dy_m)
                .clamp(-6.0, 6.0),
            elbow_velocity_rad_s: (1.5 * obs.target_dx_m + 3.0 * obs.target_dz_m).clamp(-6.0, 6.0),
            ..MobileManipulatorAction::default()
        };

        assert_eq!(episode.curriculum_stage(), Some(0));
        let mut reset = episode.reset();
        // Two stages of 3 successes each need only a handful of solved episodes.
        for _ in 0..15 {
            let mut obs = reset.observation;
            for _ in 0..500 {
                let step = episode.step(policy(&obs));
                obs = step.observation;
                if step.terminated || step.truncated {
                    break;
                }
            }
            reset = episode.reset();
        }
        assert_eq!(
            episode.curriculum_stage(),
            Some(2),
            "a reliable policy should advance the curriculum to the final stage"
        );
    }

    #[test]
    fn place_episode_picks_carries_and_sets_down() {
        let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::place());
        let _ = episode.reset();

        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let carry = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 0.6,
            ..MobileManipulatorAction::default()
        };
        let hold = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            ..MobileManipulatorAction::default()
        };
        let open = MobileManipulatorAction {
            gripper_velocity_rad_s: 3.0,
            ..MobileManipulatorAction::default()
        };

        for _ in 0..30 {
            episode.step(close);
            if episode.simulation().is_grasping() {
                break;
            }
        }
        for _ in 0..200 {
            episode.step(carry);
        }
        for _ in 0..30 {
            episode.step(hold);
        }
        for _ in 0..150 {
            let step = episode.step(open);
            if step.terminated {
                return;
            }
        }
        panic!("expected place episode to grasp, carry, release, and settle at the target");
    }

    #[test]
    fn place_observation_points_at_object_then_target() {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::lift_pick_place());
        let obs = episode.reset().observation;

        // Before grasping, the goal offset points from the gripper toward the cube on the
        // ground — it must be non-zero (it was always zero for Place tasks before) and
        // point downward (the cube sits below the raised gripper).
        let approach_mag =
            (obs.target_dx_m.powi(2) + obs.target_dy_m.powi(2) + obs.target_dz_m.powi(2)).sqrt();
        assert!(
            approach_mag > 0.1,
            "approach goal offset should be informative, got {approach_mag:.3}"
        );
        assert!(
            obs.target_dy_m < 0.0,
            "the cube is below the gripper, so target_dy should be negative, got {:.3}",
            obs.target_dy_m
        );

        // Lower and grasp; the goal offset should then point from the object toward the
        // place target instead.
        for _ in 0..200 {
            episode.step(MobileManipulatorAction {
                lift_velocity_m_s: -0.3,
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
        }
        let mut carry = MobileManipulatorObservation::default();
        for _ in 0..120 {
            carry = episode
                .step(MobileManipulatorAction {
                    gripper_velocity_rad_s: -2.5,
                    ..MobileManipulatorAction::default()
                })
                .observation;
            if episode.simulation().is_grasping() {
                break;
            }
        }
        assert!(
            episode.simulation().is_grasping(),
            "episode should grasp the cube"
        );
        let (cx, _, cz) = episode
            .simulation()
            .named_translation_m("lift_cube")
            .expect("cube");
        // target (0.55, 0.03, -0.87) relative to the grasped cube.
        assert!(
            (carry.target_dx_m - (0.55 - cx)).abs() < 0.05
                && (carry.target_dz_m - (-0.87 - cz)).abs() < 0.05,
            "carry goal offset should point object->target, got ({:.2},{:.2})",
            carry.target_dx_m,
            carry.target_dz_m
        );
    }

    #[test]
    fn lift_pick_place_episode_picks_carries_and_places() {
        use crate::IkLiftPickPlacePolicy;

        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::lift_pick_place());
        let _ = episode.reset();

        // Settle the arm, then drive the IK pick-and-place policy; it should grasp the
        // cube, carry it to the target, release it, and terminate with success.
        for _ in 0..150 {
            episode.step(MobileManipulatorAction::default());
        }
        let mut grasped = false;
        let mut policy = IkLiftPickPlacePolicy::new();
        let steps = policy.total_steps();
        for _ in 0..steps {
            let obs = episode.simulation().observe();
            let step = episode.step(policy.next_action(&obs));
            grasped |= episode.simulation().is_grasping();
            if step.terminated {
                assert!(grasped, "episode should have grasped before placing");
                return;
            }
        }
        panic!("expected lift pick-place episode to place the cube at the target and terminate");
    }

    #[test]
    fn scripted_lift_pick_place_episode_picks_carries_and_places() {
        use crate::LiftPickPlacePolicy;

        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::lift_pick_place());
        let _ = episode.reset();

        for _ in 0..150 {
            episode.step(MobileManipulatorAction::default());
        }
        let mut grasped = false;
        let mut policy = LiftPickPlacePolicy::new();
        let steps = policy.total_steps();
        for _ in 0..steps {
            let obs = episode.simulation().observe();
            let step = episode.step(policy.next_action(&obs));
            grasped |= episode.simulation().is_grasping();
            if step.terminated {
                assert!(grasped, "episode should have grasped before placing");
                return;
            }
        }
        panic!("expected scripted lift pick-place episode to place the cube and terminate");
    }

    #[test]
    fn clutter_pick_place_samples_target_object() {
        let config = MobileManipulatorEpisodeConfig::clutter_pick_place(7);
        let mut episode = MobileManipulatorEpisode::new(config);
        let first = episode.reset().observation.target_object_index;
        let _ = episode.reset();
        let second = episode.reset().observation.target_object_index;
        assert!(first < 3);
        assert!(second < 3);
    }

    #[test]
    fn clutter_scene_exposes_three_pick_targets() {
        let scene_path = crate::mm_minimal_clutter_scene_path();
        let sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load clutter scene");
        for name in ["clutter_cube_a", "clutter_cube_b", "clutter_cube_c"] {
            assert!(
                sim.named_translation_m(name).is_some(),
                "clutter scene should spawn `{name}`"
            );
        }
    }

    fn clutter_place_config(object_name: &str) -> MobileManipulatorEpisodeConfig {
        let target = crate::mm_minimal_kinematics::mm_minimal_clutter_place_target();
        MobileManipulatorEpisodeConfig {
            max_steps: 1300,
            scene_path: crate::mm_minimal_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: object_name.into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    fn mobile_clutter_place_config(object_name: &str) -> MobileManipulatorEpisodeConfig {
        let target = crate::mm_minimal_kinematics::mm_mobile_clutter_place_target();
        MobileManipulatorEpisodeConfig {
            max_steps: 1600,
            scene_path: crate::mm_mobile_clutter_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: object_name.into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        }
    }

    fn clutter_target_accepts_gripper_contact(
        episode: &mut MobileManipulatorEpisode,
        object_name: &str,
    ) -> bool {
        use crate::finger_contacts_named;

        let mut step = episode.reset();
        for _ in 0..20 {
            step = episode.step(MobileManipulatorAction::default());
        }
        for _ in 0..360 {
            let obs = &step.observation;
            step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: (4.0 * obs.target_dx_m).clamp(-6.0, 6.0),
                elbow_velocity_rad_s: (4.0 * obs.target_dz_m).clamp(-6.0, 6.0),
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if finger_contacts_named(episode.simulation(), object_name)
                || episode.simulation().is_grasping()
            {
                return true;
            }
        }
        false
    }

    fn grasp_clutter_cube(episode: &mut MobileManipulatorEpisode, object_name: &str) -> bool {
        let mut step = episode.reset();
        for _ in 0..20 {
            step = episode.step(MobileManipulatorAction::default());
        }
        if object_name == "clutter_cube_c" {
            for _ in 0..60 {
                let obs = &step.observation;
                step = episode.step(MobileManipulatorAction {
                    shoulder_velocity_rad_s: (4.5 * obs.target_dx_m).clamp(-5.0, 5.0),
                    gripper_velocity_rad_s: 1.0,
                    ..MobileManipulatorAction::default()
                });
            }
        }
        for _ in 0..360 {
            let obs = &step.observation;
            step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: (4.0 * obs.target_dx_m).clamp(-6.0, 6.0),
                elbow_velocity_rad_s: (4.0 * obs.target_dz_m).clamp(-6.0, 6.0),
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if episode.simulation().is_grasping() {
                return true;
            }
        }
        false
    }

    fn grasp_mobile_clutter_cube(
        episode: &mut MobileManipulatorEpisode,
        _object_name: &str,
    ) -> bool {
        let mut step = episode.reset();
        for _ in 0..80 {
            step = episode.step(MobileManipulatorAction::default());
        }
        for _ in 0..360 {
            let obs = &step.observation;
            step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: (4.0 * obs.gripper_target_dx_m).clamp(-6.0, 6.0),
                elbow_velocity_rad_s: (4.0 * obs.gripper_target_dz_m).clamp(-6.0, 6.0),
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if episode.simulation().is_grasping() {
                return true;
            }
        }
        false
    }

    /// Validates the clutter Place task matches the proven transport pick-and-place script.
    #[test]
    fn clutter_pick_place_task_matches_transport_place_script() {
        let target = ReachTarget::new(1.23, 0.03, -0.53);
        let config = MobileManipulatorEpisodeConfig {
            max_steps: 600,
            scene_path: crate::mm_minimal_transport_scene_path(),
            task: MobileManipulatorTask::Place {
                object_name: "grasp_cube".into(),
                target,
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        };
        let mut episode = MobileManipulatorEpisode::new(config);
        let _ = episode.reset();

        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let carry = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 0.6,
            ..MobileManipulatorAction::default()
        };
        let hold = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            ..MobileManipulatorAction::default()
        };
        let open = MobileManipulatorAction {
            gripper_velocity_rad_s: 3.0,
            ..MobileManipulatorAction::default()
        };

        for _ in 0..30 {
            episode.step(close);
            if episode.simulation().is_grasping() {
                break;
            }
        }
        for _ in 0..200 {
            episode.step(carry);
        }
        for _ in 0..30 {
            episode.step(hold);
        }
        for _ in 0..150 {
            if episode.step(open).terminated {
                return;
            }
        }
        panic!("expected clutter Place task parameters to complete the transport pick-and-place script");
    }

    #[test]
    fn clutter_targets_accept_gripper_contact() {
        for object_name in ["clutter_cube_a", "clutter_cube_b", "clutter_cube_c"] {
            let mut episode = MobileManipulatorEpisode::new(clutter_place_config(object_name));
            assert!(
                clutter_target_accepts_gripper_contact(&mut episode, object_name),
                "expected gripper contact with `{object_name}` on the clutter table"
            );
        }
    }

    fn run_clutter_place_after_grasp(
        episode: &mut MobileManipulatorEpisode,
        carry: MobileManipulatorAction,
        carry_steps: usize,
    ) -> bool {
        let hold = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let open = MobileManipulatorAction {
            gripper_velocity_rad_s: 3.0,
            ..MobileManipulatorAction::default()
        };
        for _ in 0..carry_steps {
            episode.step(carry);
        }
        for _ in 0..80 {
            episode.step(hold);
        }
        for _ in 0..150 {
            if episode.step(open).terminated {
                return true;
            }
        }
        false
    }

    #[test]
    fn fixed_clutter_carry_params_place_center_cube() {
        use crate::{IkClutterPickPlacePolicy, Policy};

        let mut episode = MobileManipulatorEpisode::new(clutter_place_config("clutter_cube_b"));
        let mut policy = IkClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..(20 + 360) {
            step = episode.step(policy.act(&step.observation));
        }
        assert!(
            episode.simulation().is_grasping(),
            "expected grasp before tuned fixed carry"
        );
        assert!(
            run_clutter_place_after_grasp(
                &mut episode,
                MobileManipulatorAction {
                    gripper_velocity_rad_s: -2.5,
                    shoulder_velocity_rad_s: -0.50,
                    elbow_velocity_rad_s: -0.69,
                    ..MobileManipulatorAction::default()
                },
                340,
            ),
            "expected tuned fixed carry to place clutter_cube_b"
        );
    }

    #[test]
    fn ik_clutter_policy_completes_center_place() {
        use crate::{IkClutterPickPlacePolicy, Policy};

        let mut episode = MobileManipulatorEpisode::new(clutter_place_config("clutter_cube_b"));
        let mut policy = IkClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.total_steps() {
            step = episode.step(policy.act(&step.observation));
        }
        assert!(
            step.terminated,
            "expected IK clutter policy to place clutter_cube_b on the ground target"
        );
    }

    #[test]
    fn ik_clutter_policy_grasps_center_target() {
        use crate::{IkClutterPickPlacePolicy, Policy};

        let mut episode = MobileManipulatorEpisode::new(clutter_place_config("clutter_cube_b"));
        let mut policy = IkClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        let approach_end = 20 + 360;
        for _ in 0..approach_end {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                return;
            }
        }
        panic!("expected IK clutter policy to grasp center cube during approach");
    }

    #[test]
    fn clutter_pick_place_episode_grasps_center_target() {
        let mut episode = MobileManipulatorEpisode::new(clutter_place_config("clutter_cube_b"));
        assert!(
            grasp_clutter_cube(&mut episode, "clutter_cube_b"),
            "expected scripted weld grasp of the center clutter cube"
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_transport_style_grasp_from_arm_start() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.arm_start_step() {
            step = episode.step(policy.act(&step.observation));
        }
        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let sweep = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 3.5,
            ..MobileManipulatorAction::default()
        };
        for _ in 0..120 {
            let _ = episode.step(close);
        }
        for _ in 0..480 {
            let _ = episode.step(sweep);
            if episode.simulation().is_grasping() {
                return;
            }
        }
        panic!("transport-style sweep failed to grasp");
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_ik_approach_grasps_from_arm_start() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.arm_start_step() {
            step = episode.step(policy.act(&step.observation));
            if policy.current_step() >= policy.arm_start_step() {
                break;
            }
        }
        let obs = &step.observation;
        eprintln!(
            "arm start: dx={:.3} dy={:.3} dz={:.3}",
            obs.target_dx_m, obs.target_dy_m, obs.target_dz_m
        );
        policy.set_step(policy.arm_start_step());
        for _ in 0..360 {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                return;
            }
        }
        let obs = &step.observation;
        panic!(
            "expected IK approach to grasp, dx={:.3} dy={:.3} dz={:.3}",
            obs.target_dx_m, obs.target_dy_m, obs.target_dz_m
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_grasp_helper_succeeds() {
        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        assert!(
            grasp_mobile_clutter_cube(&mut episode, "clutter_cube_a"),
            "expected grasp_clutter_cube helper to work on mobile clutter scene"
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_fixed_base_ik_policy_without_drive() {
        use crate::{IkClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.total_steps() {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                return;
            }
        }
        panic!("fixed-base IK policy failed on mobile scene without drive");
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_proportional_grasp_without_drive() {
        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut step = episode.reset();
        for _ in 0..80 {
            step = episode.step(MobileManipulatorAction::default());
        }
        for _ in 0..480 {
            let obs = &step.observation;
            step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: (2.5 * obs.target_dx_m - 0.5 * obs.target_dy_m)
                    .clamp(-6.0, 6.0),
                elbow_velocity_rad_s: (1.5 * obs.target_dx_m + 3.0 * obs.target_dz_m)
                    .clamp(-6.0, 6.0),
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if episode.simulation().is_grasping() {
                return;
            }
        }
        let obs = &step.observation;
        panic!(
            "no grasp without drive: dx={:.3} dy={:.3} dz={:.3}",
            obs.target_dx_m, obs.target_dy_m, obs.target_dz_m
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_scripted_drive_then_proportional_grasp() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.arm_start_step() {
            step = episode.step(policy.act(&step.observation));
            if policy.current_step() >= policy.arm_start_step() {
                break;
            }
        }
        for _ in 0..480 {
            let obs = &step.observation;
            step = episode.step(MobileManipulatorAction {
                shoulder_velocity_rad_s: (2.5 * obs.target_dx_m - 0.5 * obs.target_dy_m)
                    .clamp(-6.0, 6.0),
                elbow_velocity_rad_s: (1.5 * obs.target_dx_m + 3.0 * obs.target_dz_m)
                    .clamp(-6.0, 6.0),
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            });
            if episode.simulation().is_grasping() {
                return;
            }
        }
        let obs = &step.observation;
        panic!(
            "expected proportional grasp after drive, dx={:.3} dy={:.3} dz={:.3}",
            obs.target_dx_m, obs.target_dy_m, obs.target_dz_m
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_debug_drive_trajectory() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for i in 0..policy.total_steps() as usize {
            let prev_step = policy.arm_start_step();
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                eprintln!("grasped at episode step {i}");
                return;
            }
            if i % 100 == 0 {
                let obs = &step.observation;
                eprintln!(
                    "step {i}: dx={:.3} dz={:.3} dy={:.3} grasp={}",
                    obs.target_dx_m,
                    obs.target_dz_m,
                    obs.target_dy_m,
                    episode.simulation().is_grasping()
                );
            }
            let _ = prev_step;
        }
        panic!("no grasp in {} policy steps", policy.total_steps());
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_debug_cube_pose_after_settle() {
        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let step = episode.reset();
        let mut step = step;
        for _ in 0..80 {
            step = episode.step(MobileManipulatorAction::default());
        }
        let obs = &step.observation;
        let cube = episode
            .simulation()
            .named_translation_m("clutter_cube_a")
            .expect("cube");
        let gripper = episode
            .simulation()
            .link_translation_m("gripper_base_link")
            .expect("gripper");
        eprintln!(
            "cube=({:.3},{:.3},{:.3}) ee=({:.3},{:.3},{:.3}) base=({:.3},{:.3},{:.3}) gripper=({:.3},{:.3},{:.3}) target_d=({:.3},{:.3},{:.3}) gripper_d=({:.3},{:.3},{:.3})",
            cube.0,
            cube.1,
            cube.2,
            obs.ee_x_m,
            obs.ee_y_m,
            obs.ee_z_m,
            obs.base_x_m,
            obs.base_y_m,
            obs.base_z_m,
            gripper.0,
            gripper.1,
            gripper.2,
            obs.target_dx_m,
            obs.target_dy_m,
            obs.target_dz_m,
            obs.gripper_target_dx_m,
            obs.gripper_target_dy_m,
            obs.gripper_target_dz_m
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_clutter_episode_drives_base_toward_target() {
        use crate::{
            IkMobileClutterPickPlacePolicy, MmMinimalGripperTarget, MmMinimalKinematics, Policy,
        };

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.arm_start_step() {
            step = episode.step(policy.act(&step.observation));
            if policy.current_step() >= policy.arm_start_step() {
                break;
            }
        }
        let obs = &step.observation;
        let kin = MmMinimalKinematics::mm_mobile();
        let target = MmMinimalGripperTarget::new(
            obs.pick_object_x_m,
            kin.shoulder_y_at_base(obs.base_y_m),
            obs.pick_object_z_m,
        );
        assert!(
            kin.inverse_kinematics_at_base(
                obs.base_x_m,
                obs.base_y_m,
                obs.base_z_m,
                obs.base_yaw_rad,
                target,
            )
            .is_ok(),
            "expected analytic reach after drive, dx={:.3} dy={:.3}",
            obs.target_dx_m,
            obs.target_dy_m
        );
        assert!(
            obs.target_dx_m.abs() > 0.04 || obs.target_dz_m.abs() > 0.04,
            "expected non-trivial ee offset after settle, got dx={:.3} dz={:.3}",
            obs.target_dx_m,
            obs.target_dz_m
        );
        assert!(
            obs.target_dy_m.abs() <= 0.60,
            "expected lateral alignment after drive, got dy={:.3}",
            obs.target_dy_m
        );
    }

    #[test]
    #[ignore = "mobile clutter tuning helper"]
    fn mobile_log_drive_ready_state() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut step = episode.reset();
        for _ in 0..80 {
            step = episode.step(MobileManipulatorAction::default());
        }
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        for i in 0..480 {
            let obs = &step.observation;
            let ready =
                obs.target_dx_m > 0.10 && obs.target_dx_m <= 0.36 && obs.target_dz_m.abs() <= 0.18;
            if ready {
                eprintln!(
                    "ready at drive step {i}: dx={:.3} dz={:.3} dy={:.3} ee=({:.3},{:.3},{:.3}) base=({:.3},{:.3})",
                    obs.target_dx_m,
                    obs.target_dz_m,
                    obs.target_dy_m,
                    obs.ee_x_m,
                    obs.ee_y_m,
                    obs.ee_z_m,
                    obs.base_x_m,
                    obs.base_z_m
                );
                policy.set_step(policy.arm_start_step() - 40);
                break;
            }
            step = episode.step(policy.act(&step.observation));
        }
        let obs = &step.observation;
        eprintln!(
            "drive end: dx={:.3} dz={:.3} grasping={}",
            obs.target_dx_m,
            obs.target_dz_m,
            episode.simulation().is_grasping()
        );
        for i in 0..360 {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                eprintln!("grasped at approach step {i}");
                return;
            }
        }
        panic!("no grasp after approach");
    }

    #[test]
    fn mobile_clutter_policy_completes_place() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        let mut grasped = false;
        for _ in 0..policy.total_steps() {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                grasped = true;
            }
        }
        assert!(
            grasped,
            "expected mobile clutter policy to grasp clutter_cube_a before place"
        );
        assert!(
            step.terminated,
            "expected mobile clutter policy to place clutter_cube_a on the ground target"
        );
    }

    #[test]
    #[ignore = "manual tuning helper"]
    fn mobile_clutter_contact_without_drive() {
        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        assert!(
            clutter_target_accepts_gripper_contact(&mut episode, "clutter_cube_a"),
            "expected gripper contact from spawn on mobile clutter scene"
        );
    }

    #[test]
    fn mobile_clutter_transport_script_places_cube_a() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        let mut step = episode.reset();
        for _ in 0..policy.total_steps() {
            step = episode.step(policy.act(&step.observation));
            if step.terminated {
                return;
            }
        }
        panic!("expected mobile clutter policy to place clutter_cube_a");
    }

    #[test]
    #[ignore = "manual tuning helper"]
    fn mobile_clutter_transport_script_legacy() {
        use crate::{IkMobileClutterPickPlacePolicy, Policy};

        let mut episode =
            MobileManipulatorEpisode::new(mobile_clutter_place_config("clutter_cube_a"));
        let mut step = episode.reset();
        for _ in 0..80 {
            step = episode.step(MobileManipulatorAction::default());
        }
        for _ in 0..480 {
            let obs = &step.observation;
            let object_x = obs.ee_x_m + obs.target_dx_m;
            let object_z = obs.ee_z_m + obs.target_dz_m;
            let dx = object_x - obs.base_x_m;
            let dz = object_z - obs.base_z_m;
            let base_dist = (dx * dx + dz * dz).sqrt();
            if base_dist < 0.55 && obs.target_dx_m > 0.08 && obs.target_dz_m.abs() < 0.35 {
                break;
            }
            let forward = dx.clamp(0.0, 1.5);
            let turn = dz.clamp(-1.0, 1.0);
            step = episode.step(MobileManipulatorAction {
                left_wheel_velocity_rad_s: (2.5 * forward - 1.5 * turn).clamp(-3.0, 3.0),
                right_wheel_velocity_rad_s: (2.5 * forward + 1.5 * turn).clamp(-3.0, 3.0),
                ..MobileManipulatorAction::default()
            });
        }
        let mut policy = IkMobileClutterPickPlacePolicy::new();
        policy.set_step(policy.arm_start_step());
        let mut grasped = false;
        for _ in 0..policy.total_steps() - policy.arm_start_step() {
            step = episode.step(policy.act(&step.observation));
            if episode.simulation().is_grasping() {
                grasped = true;
            }
            if step.terminated {
                return;
            }
        }
        if !grasped {
            let obs = &step.observation;
            panic!(
                "grasp failed after drive: target_dx={:.3} target_dz={:.3} ee=({:.3},{:.3}) base=({:.3},{:.3})",
                obs.target_dx_m,
                obs.target_dz_m,
                obs.ee_x_m,
                obs.ee_z_m,
                obs.base_x_m,
                obs.base_z_m
            );
        }
        panic!("expected mobile transport script to terminate with place success");
    }

    #[test]
    fn reach_episode_accepts_proportional_policy() {
        let target = ReachTarget::new(0.50, 0.58, 0.10);
        let config = MobileManipulatorEpisodeConfig {
            max_steps: 720,
            scene_path: crate::mm_minimal_scene_path(),
            task: MobileManipulatorTask::Reach {
                target,
                success_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            clutter_pick: None,
            rng_seed: 0,
        };
        let mut episode = MobileManipulatorEpisode::new(config);
        let _ = episode.reset();
        for _ in 0..720 {
            let obs = episode.simulation().observe();
            let action = reach_action_proportional(&obs, target, 6.0);
            let step = episode.step(action);
            if step.terminated {
                return;
            }
        }
    }
}
