use super::{
    unitree_g1_dynamic_scene_path, unitree_g1_gait_targets, UnitreeG1GaitCommand,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 120;
const NOMINAL_HEIGHT_M: f64 = 0.80;
const FALLEN_HEIGHT_M: f64 = 0.35;

/// Configuration for the Unitree G1 gait episode.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1GaitEpisodeConfig {
    /// Dynamic multibody G1 scene.
    pub scene_path: PathBuf,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
    /// Simulation steps per gait cycle.
    pub cycle_steps: u64,
}

impl Default for UnitreeG1GaitEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            max_steps: 600,
            cycle_steps: 120,
        }
    }
}

/// Continuous gait parameters applied on the next physics step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1GaitAction {
    /// Hip-pitch stride amplitude in radians, clamped to `[0, 0.20]`.
    pub stride_rad: f64,
    /// Swing-leg knee lift in radians, clamped to `[0, 0.20]`.
    pub foot_lift_rad: f64,
    /// Waist-yaw correction in radians, clamped to `[-0.25, 0.25]`.
    pub yaw_correction_rad: f64,
}

impl Default for UnitreeG1GaitAction {
    fn default() -> Self {
        Self {
            stride_rad: 0.05,
            foot_lift_rad: 0.05,
            yaw_correction_rad: 0.0,
        }
    }
}

/// Observation emitted by [`UnitreeG1GaitEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1GaitObservation {
    /// Pelvis X position in meters.
    pub base_x_m: f64,
    /// Forward displacement during the latest step in meters.
    pub forward_delta_m: f64,
    /// Pelvis height in meters.
    pub base_y_m: f64,
    /// Pelvis lateral Z position in meters.
    pub base_z_m: f64,
    /// Pelvis yaw in radians.
    pub base_yaw_rad: f64,
    /// Left-foot normal contact impulse in N·s.
    pub left_foot_impulse_ns: f64,
    /// Right-foot normal contact impulse in N·s.
    pub right_foot_impulse_ns: f64,
    /// Normalized gait phase in `[0, 1)`.
    pub gait_phase: f64,
    /// Normalized episode progress in `[0, 1]`.
    pub progress: f64,
}

/// Deterministic forward-gait episode for the official Unitree G1 model.
pub struct UnitreeG1GaitEpisode {
    config: UnitreeG1GaitEpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
}

impl UnitreeG1GaitEpisode {
    /// Loads the dynamic G1 and settles it into the nominal standing pose.
    pub fn new(config: UnitreeG1GaitEpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle(&mut sim);
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
        })
    }

    fn observation(&self, forward_delta_m: f64) -> UnitreeG1GaitObservation {
        let base = self.sim.observe();
        UnitreeG1GaitObservation {
            base_x_m: base.base_x_m,
            forward_delta_m,
            base_y_m: base.base_y_m,
            base_z_m: base.base_z_m,
            base_yaw_rad: base.base_yaw_rad,
            left_foot_impulse_ns: self.sim.link_contact_impulse_ns("left_ankle_roll_link"),
            right_foot_impulse_ns: self.sim.link_contact_impulse_ns("right_ankle_roll_link"),
            gait_phase: (self.step_in_episode % self.config.cycle_steps.max(1)) as f64
                / self.config.cycle_steps.max(1) as f64,
            progress: (self.step_in_episode as f64 / self.config.max_steps.max(1) as f64)
                .clamp(0.0, 1.0),
        }
    }
}

impl Episode for UnitreeG1GaitEpisode {
    type Observation = UnitreeG1GaitObservation;
    type Action = UnitreeG1GaitAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload Unitree G1 gait scene");
        settle(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        EpisodeStep {
            observation: self.observation(0.0),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let before_x_m = self.sim.observe().base_x_m;
        let command = UnitreeG1GaitCommand {
            stride_rad: action.stride_rad.clamp(0.0, 0.20),
            foot_lift_rad: action.foot_lift_rad.clamp(0.0, 0.20),
            cycle_steps: self.config.cycle_steps,
        };
        let mut targets = unitree_g1_gait_targets(self.step_in_episode, command);
        targets[12].position = action.yaw_correction_rad.clamp(-0.25, 0.25);
        self.sim.step_joint_position_targets(&targets);
        self.step_in_episode += 1;

        let forward_delta_m = self.sim.observe().base_x_m - before_x_m;
        let observation = self.observation(forward_delta_m);
        let fallen = observation.base_y_m < FALLEN_HEIGHT_M;
        let height_error_m = (observation.base_y_m - NOMINAL_HEIGHT_M).abs();
        let reward = 0.5 + 100.0 * forward_delta_m
            - 2.0 * height_error_m
            - 0.1 * observation.base_yaw_rad.abs()
            - 0.2 * observation.base_z_m.abs()
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
    let targets = [
        target("left_hip_pitch_link", -0.18),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
    ];
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
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
    fn gait_episode_replays_short_rollout_exactly() {
        let config = UnitreeG1GaitEpisodeConfig {
            max_steps: 12,
            ..Default::default()
        };
        let mut first = UnitreeG1GaitEpisode::new(config.clone()).expect("first G1 gait");
        let mut second = UnitreeG1GaitEpisode::new(config).expect("second G1 gait");
        let mut reward = 0.0;
        let mut last = None;
        for _ in 0..12 {
            let a = first.step(UnitreeG1GaitAction::default());
            let b = second.step(UnitreeG1GaitAction::default());
            assert_eq!(a, b);
            reward += a.reward;
            last = Some(a);
        }
        let last = last.expect("gait step");
        assert!(last.truncated);
        assert!(!last.terminated);
        assert!(reward > 0.0);
        assert!(last.observation.gait_phase > 0.0);
    }
}
