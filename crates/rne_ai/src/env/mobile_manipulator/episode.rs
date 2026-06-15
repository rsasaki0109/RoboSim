//! Mobile manipulator episode environment.

use crate::action::MobileManipulatorAction;
use crate::episode::{Episode, EpisodeStep};
use crate::grasp::finger_contacts_named;
use crate::observation::MobileManipulatorObservation;
use crate::reach::ee_distance_to_target_m;
use crate::reward::{MobileManipulatorRewardConfig, MobileManipulatorTask};
use crate::transport::{
    body_moved_at_least_m, body_within_zone_m, had_finger_contact, named_translation_m,
    TRANSPORT_SUCCESS_M,
};
use crate::MobileManipulatorSim;
use std::path::PathBuf;

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
                target: crate::reach::ReachTarget::new(0.35, 0.0, 1.0),
                place_tolerance_m: 0.12,
            },
            reward: MobileManipulatorRewardConfig::default(),
            reach_randomization: None,
            reach_curriculum: None,
            rng_seed: 0,
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
                    if let Some((ox, oy, oz)) = self.sim.named_translation_m(object_name) {
                        observation.target_dx_m = target.x_m - ox;
                        observation.target_dy_m = target.y_m - oy;
                        observation.target_dz_m = target.z_m - oz;
                    }
                } else if let Some((ox, oy, oz)) = self.sim.named_translation_m(object_name) {
                    observation.target_dx_m = ox - observation.ee_x_m;
                    observation.target_dy_m = oy - observation.ee_y_m;
                    observation.target_dz_m = oz - observation.ee_z_m;
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

fn initial_progress_state(
    sim: &MobileManipulatorSim,
    task: &MobileManipulatorTask,
) -> EpisodeProgressState {
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
    EpisodeProgressState {
        object_initial,
        place_error_m,
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
            let progress = (state.ee_error_m - error).max(0.0);
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
            if let Some(current) = object_horizontal_distance_to_target_m(sim, object_name, *target)
            {
                let progress = (state.place_error_m - current).max(0.0);
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
        use crate::LiftPickPlacePolicy;

        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::lift_pick_place());
        let _ = episode.reset();

        // Settle the arm, then drive the shared scripted pick-and-place policy; it should
        // grasp the cube, carry it to the target, release it, and terminate with success.
        for _ in 0..150 {
            episode.step(MobileManipulatorAction::default());
        }
        let mut grasped = false;
        let mut policy = LiftPickPlacePolicy::new();
        for _ in 0..1030 {
            let step = episode.step(policy.next_action());
            grasped |= episode.simulation().is_grasping();
            if step.terminated {
                assert!(grasped, "episode should have grasped before placing");
                return;
            }
        }
        panic!("expected lift pick-place episode to place the cube at the target and terminate");
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
