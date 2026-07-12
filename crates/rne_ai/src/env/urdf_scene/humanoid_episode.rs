use super::{humanoid_scene_path, UrdfJointPositionTarget, UrdfSceneSim};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const STAND_SETTLE_STEPS: u64 = 240;
const FALLEN_BASE_HEIGHT_M: f64 = 0.70;
const NOMINAL_BASE_HEIGHT_M: f64 = 0.925;

/// Configuration for a deterministic humanoid balance episode.
#[derive(Clone, Debug, PartialEq)]
pub struct HumanoidEpisodeConfig {
    /// Scene containing the humanoid articulation.
    pub scene_path: PathBuf,
    /// Maximum controlled physics steps before truncation.
    pub max_steps: u64,
}

impl Default for HumanoidEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: humanoid_scene_path(),
            max_steps: 600,
        }
    }
}

/// Low-dimensional balance action mapped onto symmetric humanoid joints.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HumanoidAction {
    /// Shared hip-roll correction in radians, clamped to `[-0.3, 0.3]`.
    pub hip_roll_rad: f64,
    /// Symmetric knee bend in radians, clamped to `[0, 0.8]`.
    pub knee_bend_rad: f64,
    /// Opposed arm-roll counterbalance in radians, clamped to `[-1, 1]`.
    pub arm_counterbalance_rad: f64,
}

/// Observation returned by [`HumanoidEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HumanoidObservation {
    /// Torso X position in meters.
    pub base_x_m: f64,
    /// Torso height in meters.
    pub base_y_m: f64,
    /// Torso Z position in meters.
    pub base_z_m: f64,
    /// Torso yaw in radians.
    pub base_yaw_rad: f64,
    /// Left-foot normal contact impulse in N·s.
    pub left_foot_impulse_ns: f64,
    /// Right-foot normal contact impulse in N·s.
    pub right_foot_impulse_ns: f64,
    /// Normalized absolute left/right load difference in `[0, 1]`.
    pub foot_load_imbalance: f64,
    /// Normalized episode progress in `[0, 1]`.
    pub progress: f64,
}

/// Deterministic humanoid standing and balance episode.
pub struct HumanoidEpisode {
    config: HumanoidEpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
}

impl HumanoidEpisode {
    /// Loads and initializes a humanoid balance episode.
    pub fn new(config: HumanoidEpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle_standing(&mut sim);
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
        })
    }

    fn observation(&self) -> HumanoidObservation {
        let base = self.sim.observe();
        let left_foot_impulse_ns = self.sim.link_contact_impulse_ns("left_foot");
        let right_foot_impulse_ns = self.sim.link_contact_impulse_ns("right_foot");
        let total_impulse_ns = left_foot_impulse_ns + right_foot_impulse_ns;
        let foot_load_imbalance = if total_impulse_ns > 1.0e-9 {
            ((left_foot_impulse_ns - right_foot_impulse_ns).abs() / total_impulse_ns)
                .clamp(0.0, 1.0)
        } else {
            1.0
        };
        HumanoidObservation {
            base_x_m: base.base_x_m,
            base_y_m: base.base_y_m,
            base_z_m: base.base_z_m,
            base_yaw_rad: base.base_yaw_rad,
            left_foot_impulse_ns,
            right_foot_impulse_ns,
            foot_load_imbalance,
            progress: (self.step_in_episode as f64 / self.config.max_steps.max(1) as f64)
                .clamp(0.0, 1.0),
        }
    }
}

impl Episode for HumanoidEpisode {
    type Observation = HumanoidObservation;
    type Action = HumanoidAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload humanoid episode scene");
        settle_standing(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        EpisodeStep {
            observation: self.observation(),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let hip_roll_rad = action.hip_roll_rad.clamp(-0.3, 0.3);
        let knee_bend_rad = action.knee_bend_rad.clamp(0.0, 0.8);
        let arm_rad = action.arm_counterbalance_rad.clamp(-1.0, 1.0);
        let targets = [
            target("left_hip", hip_roll_rad),
            target("right_hip", hip_roll_rad),
            target("left_foot", knee_bend_rad),
            target("right_foot", knee_bend_rad),
            target("left_upper_arm", arm_rad),
            target("right_upper_arm", -arm_rad),
        ];
        self.sim.step_joint_position_targets(&targets);
        self.step_in_episode += 1;

        let observation = self.observation();
        let fallen = observation.base_y_m < FALLEN_BASE_HEIGHT_M;
        let height_error_m = (observation.base_y_m - NOMINAL_BASE_HEIGHT_M).abs();
        let reward = 1.0
            - 2.0 * height_error_m
            - 0.5 * observation.foot_load_imbalance
            - if fallen { 10.0 } else { 0.0 };
        EpisodeStep {
            observation,
            reward,
            terminated: fallen,
            truncated: self.step_in_episode >= self.config.max_steps,
        }
    }

    fn episode_index(&self) -> u32 {
        self.episode_index
    }

    fn step_in_episode(&self) -> u64 {
        self.step_in_episode
    }
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

fn settle_standing(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(1800.0, 85.0, 80.0);
    for _ in 0..STAND_SETTLE_STEPS {
        sim.step_joint_position_targets(&[]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn balance_episode_is_upright_rewarding_and_exactly_replayable() {
        let config = HumanoidEpisodeConfig {
            max_steps: 360,
            ..HumanoidEpisodeConfig::default()
        };
        let mut first = HumanoidEpisode::new(config.clone()).expect("first episode");
        let mut second = HumanoidEpisode::new(config).expect("second episode");
        let mut total_reward = 0.0;
        let mut final_step = None;
        for _ in 0..360 {
            let action = HumanoidAction::default();
            let a = first.step(action);
            let b = second.step(action);
            assert_eq!(a, b, "identical balance rollouts must replay exactly");
            total_reward += a.reward;
            final_step = Some(a);
        }
        let final_step = final_step.expect("balance step");
        assert!(final_step.truncated);
        assert!(!final_step.terminated);
        assert!(total_reward > 100.0);
        assert!(final_step.observation.left_foot_impulse_ns > 0.0);
        assert!(final_step.observation.right_foot_impulse_ns > 0.0);
    }
}
