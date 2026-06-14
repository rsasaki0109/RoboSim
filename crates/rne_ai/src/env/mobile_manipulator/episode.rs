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
        }
    }
}

/// Manipulation episode built on top of [`MobileManipulatorSim`].
pub struct MobileManipulatorEpisode {
    sim: MobileManipulatorSim,
    config: MobileManipulatorEpisodeConfig,
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
        let progress_state = initial_progress_state(&sim, &config.task);
        Self {
            sim,
            config,
            episode_index: 0,
            step_in_episode: 0,
            total_reward: 0.0,
            progress_state,
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

    fn make_step(
        &mut self,
        observation: MobileManipulatorObservation,
    ) -> EpisodeStep<MobileManipulatorObservation> {
        let progress = task_progress(
            &self.config.task,
            &observation,
            &mut self.progress_state,
            &self.sim,
        );
        let success = task_success(
            &self.config.task,
            &observation,
            &self.progress_state,
            &self.sim,
        );
        let truncated = !success && self.step_in_episode >= self.config.max_steps;
        let reward = self.config.reward.compute(progress, success);
        self.total_reward += reward;

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
        self.progress_state = initial_progress_state(&self.sim, &self.config.task);

        let observation = self.sim.observe();
        self.progress_state.ee_error_m = initial_ee_error(&self.config.task, &observation);

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
        self.make_step(observation)
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
