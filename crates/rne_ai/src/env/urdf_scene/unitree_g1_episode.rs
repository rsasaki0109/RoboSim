use super::{unitree_g1_dynamic_scene_path, UrdfJointPositionTarget, UrdfSceneSim};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 240;
const NOMINAL_BASE_HEIGHT_M: f64 = 0.82;
const FALLEN_BASE_HEIGHT_M: f64 = 0.35;

/// Configuration for the official Unitree G1 balance episode.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1EpisodeConfig {
    /// Dynamic G1 scene used by the episode.
    pub scene_path: PathBuf,
    /// Maximum controlled physics steps before truncation.
    pub max_steps: u64,
}

impl Default for UnitreeG1EpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            max_steps: 600,
        }
    }
}

/// Low-dimensional balance command mapped onto the G1's 23 joints.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UnitreeG1Action {
    /// Lateral hip correction in radians, clamped to `[-0.15, 0.15]`.
    pub hip_roll_rad: f64,
    /// Additional symmetric knee bend in radians, clamped to `[-0.15, 0.3]`.
    pub knee_delta_rad: f64,
    /// Opposed shoulder-pitch counterbalance in radians, clamped to `[-0.6, 0.6]`.
    pub arm_counterbalance_rad: f64,
    /// Waist-yaw command in radians, clamped to `[-0.4, 0.4]`.
    pub waist_yaw_rad: f64,
}

/// Observation returned by [`UnitreeG1Episode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1Observation {
    /// Pelvis X position in meters.
    pub base_x_m: f64,
    /// Pelvis height in meters.
    pub base_y_m: f64,
    /// Pelvis Z position in meters.
    pub base_z_m: f64,
    /// Pelvis yaw in radians.
    pub base_yaw_rad: f64,
    /// Left-foot normal contact impulse in N·s.
    pub left_foot_impulse_ns: f64,
    /// Right-foot normal contact impulse in N·s.
    pub right_foot_impulse_ns: f64,
    /// Normalized left/right contact-load difference in `[0, 1]`.
    pub foot_load_imbalance: f64,
    /// Normalized episode progress in `[0, 1]`.
    pub progress: f64,
}

/// Deterministic balance episode for the official Unitree G1 23-DoF model.
pub struct UnitreeG1Episode {
    config: UnitreeG1EpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
}

impl UnitreeG1Episode {
    /// Loads the dynamic G1 scene and settles into its nominal standing pose.
    pub fn new(config: UnitreeG1EpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle(&mut sim);
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
        })
    }

    fn observation(&self) -> UnitreeG1Observation {
        let base = self.sim.observe();
        let left_foot_impulse_ns = self.sim.link_contact_impulse_ns("left_ankle_roll_link");
        let right_foot_impulse_ns = self.sim.link_contact_impulse_ns("right_ankle_roll_link");
        let total = left_foot_impulse_ns + right_foot_impulse_ns;
        let foot_load_imbalance = if total > 1.0e-9 {
            ((left_foot_impulse_ns - right_foot_impulse_ns).abs() / total).clamp(0.0, 1.0)
        } else {
            1.0
        };
        UnitreeG1Observation {
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

impl Episode for UnitreeG1Episode {
    type Observation = UnitreeG1Observation;
    type Action = UnitreeG1Action;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload Unitree G1 episode scene");
        settle(&mut self.sim);
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
        let targets = standing_targets(action);
        self.sim.step_joint_position_targets(&targets);
        self.step_in_episode += 1;
        let observation = self.observation();
        let fallen = observation.base_y_m < FALLEN_BASE_HEIGHT_M;
        let height_error_m = (observation.base_y_m - NOMINAL_BASE_HEIGHT_M).abs();
        let drift_m = observation.base_x_m.hypot(observation.base_z_m);
        let reward = 1.0
            - 2.0 * height_error_m
            - drift_m
            - 0.25 * observation.foot_load_imbalance
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

fn settle(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let targets = standing_targets(UnitreeG1Action::default());
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
}

fn standing_targets(action: UnitreeG1Action) -> [UrdfJointPositionTarget<'static>; 13] {
    let hip_roll = action.hip_roll_rad.clamp(-0.15, 0.15);
    let knee = 0.36 + action.knee_delta_rad.clamp(-0.15, 0.3);
    let arm = action.arm_counterbalance_rad.clamp(-0.6, 0.6);
    [
        target("left_hip_pitch_link", -0.18),
        target("left_hip_roll_link", hip_roll),
        target("left_knee_link", knee),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_hip_roll_link", hip_roll),
        target("right_knee_link", knee),
        target("right_ankle_pitch_link", -0.18),
        target("torso_link", action.waist_yaw_rad.clamp(-0.4, 0.4)),
        target("left_shoulder_pitch_link", arm),
        target("right_shoulder_pitch_link", -arm),
        target("left_elbow_link", 0.42),
        target("right_elbow_link", 0.42),
    ]
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g1_balance_rollout_is_upright_and_exactly_replayable() {
        let config = UnitreeG1EpisodeConfig {
            max_steps: 180,
            ..Default::default()
        };
        let mut first = UnitreeG1Episode::new(config.clone()).expect("first G1 episode");
        let mut second = UnitreeG1Episode::new(config).expect("second G1 episode");
        let mut reward = 0.0;
        let mut last = None;
        for _ in 0..180 {
            let a = first.step(UnitreeG1Action::default());
            let b = second.step(UnitreeG1Action::default());
            assert_eq!(a, b);
            reward += a.reward;
            last = Some(a);
        }
        let last = last.expect("G1 balance step");
        assert!(last.truncated);
        assert!(!last.terminated);
        assert!(reward > 50.0, "unexpected reward {reward}");
        assert!(last.observation.left_foot_impulse_ns > 0.0);
        assert!(last.observation.right_foot_impulse_ns > 0.0);
    }

    #[test]
    fn g1_balance_reset_is_repeatable() {
        let mut episode = UnitreeG1Episode::new(Default::default()).expect("G1 episode");
        let first = episode.reset();
        let second = episode.reset();
        assert_eq!(first.observation, second.observation);
        assert_eq!(episode.episode_index(), 2);
    }
}
