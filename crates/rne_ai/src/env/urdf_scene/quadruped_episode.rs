use super::{quadruped_scene_path, quadruped_trot_targets, UrdfSceneSim, QUADRUPED_FOOT_LINKS};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const STAND_SETTLE_STEPS: u64 = 180;
const FALLEN_BASE_HEIGHT_M: f64 = 0.35;

/// Configuration for a headless quadruped gait episode.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadrupedEpisodeConfig {
    /// Scene containing the quadruped articulation.
    pub scene_path: PathBuf,
    /// Maximum controlled physics steps before truncation.
    pub max_steps: u64,
}

impl Default for QuadrupedEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: quadruped_scene_path(),
            max_steps: 600,
        }
    }
}

/// Continuous gait-shaping action applied to the deterministic trot template.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadrupedAction {
    /// Multiplier for hip-pitch stride targets, clamped to `[0, 1.5]`.
    pub stride_scale: f64,
    /// Multiplier for swing-knee flexion, clamped to `[0, 1.5]`.
    pub knee_lift_scale: f64,
}

impl Default for QuadrupedAction {
    fn default() -> Self {
        Self {
            stride_scale: 1.0,
            knee_lift_scale: 1.0,
        }
    }
}

/// Observation returned by [`QuadrupedEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadrupedObservation {
    /// Body X position in meters.
    pub base_x_m: f64,
    /// Body height in meters.
    pub base_y_m: f64,
    /// Body Z position in meters.
    pub base_z_m: f64,
    /// Body yaw in radians.
    pub base_yaw_rad: f64,
    /// Per-foot normal contact impulses in N·s, ordered by [`QUADRUPED_FOOT_LINKS`].
    pub foot_impulses_ns: [f64; 4],
    /// Normalized progress through the current episode in `[0, 1]`.
    pub progress: f64,
}

/// Deterministic quadruped locomotion episode using the built-in URDF scene.
pub struct QuadrupedEpisode {
    config: QuadrupedEpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
    previous_x_m: f64,
}

impl QuadrupedEpisode {
    /// Loads and initializes a quadruped episode.
    pub fn new(config: QuadrupedEpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle_standing(&mut sim);
        let previous_x_m = sim.observe().base_x_m;
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
            previous_x_m,
        })
    }

    fn observation(&self) -> QuadrupedObservation {
        let base = self.sim.observe();
        QuadrupedObservation {
            base_x_m: base.base_x_m,
            base_y_m: base.base_y_m,
            base_z_m: base.base_z_m,
            base_yaw_rad: base.base_yaw_rad,
            foot_impulses_ns: QUADRUPED_FOOT_LINKS
                .map(|foot| self.sim.link_contact_impulse_ns(foot)),
            progress: (self.step_in_episode as f64 / self.config.max_steps.max(1) as f64)
                .clamp(0.0, 1.0),
        }
    }
}

impl Episode for QuadrupedEpisode {
    type Observation = QuadrupedObservation;
    type Action = QuadrupedAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload quadruped episode scene");
        settle_standing(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        self.previous_x_m = self.sim.observe().base_x_m;
        EpisodeStep {
            observation: self.observation(),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let stride_scale = action.stride_scale.clamp(0.0, 1.5);
        let knee_lift_scale = action.knee_lift_scale.clamp(0.0, 1.5);
        let mut targets = quadruped_trot_targets(self.step_in_episode);
        for target in &mut targets {
            if target.link_name.ends_with("_thigh") {
                target.position *= stride_scale;
            } else if target.link_name.ends_with("_foot") {
                target.position *= knee_lift_scale;
            }
        }
        self.sim.step_joint_position_targets(&targets);
        self.step_in_episode += 1;

        let observation = self.observation();
        let dx_m = observation.base_x_m - self.previous_x_m;
        self.previous_x_m = observation.base_x_m;
        let fallen = observation.base_y_m < FALLEN_BASE_HEIGHT_M;
        let truncated = self.step_in_episode >= self.config.max_steps;
        EpisodeStep {
            observation,
            reward: 100.0 * dx_m - if fallen { 10.0 } else { 0.0 },
            terminated: fallen,
            truncated,
        }
    }

    fn episode_index(&self) -> u32 {
        self.episode_index
    }

    fn step_in_episode(&self) -> u64 {
        self.step_in_episode
    }
}

fn settle_standing(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(1200.0, 70.0, 40.0);
    for _ in 0..STAND_SETTLE_STEPS {
        sim.step_joint_position_targets(&[]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gait_episode_replays_actions_exactly_and_rewards_forward_motion() {
        let config = QuadrupedEpisodeConfig {
            max_steps: 360,
            ..QuadrupedEpisodeConfig::default()
        };
        let mut first = QuadrupedEpisode::new(config.clone()).expect("first episode");
        let mut second = QuadrupedEpisode::new(config).expect("second episode");
        let mut total_reward = 0.0;
        let mut final_step = None;
        for _ in 0..360 {
            let action = QuadrupedAction::default();
            let a = first.step(action);
            let b = second.step(action);
            assert_eq!(a, b, "identical action sequences must replay exactly");
            total_reward += a.reward;
            final_step = Some(a);
        }
        let final_step = final_step.expect("gait step");
        assert!(final_step.truncated);
        assert!(!final_step.terminated);
        assert!(
            total_reward > 0.5,
            "default gait should earn forward progress reward, got {total_reward}"
        );
        assert!(final_step.observation.base_y_m > FALLEN_BASE_HEIGHT_M);
    }
}
