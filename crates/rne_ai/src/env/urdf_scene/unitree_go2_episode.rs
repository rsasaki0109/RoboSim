use super::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 120;
const NOMINAL_HEIGHT_M: f64 = 0.23;
const FALLEN_HEIGHT_M: f64 = 0.12;

/// Configuration for the official Unitree Go2 trot episode.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeGo2EpisodeConfig {
    /// Dynamic multibody Go2 scene.
    pub scene_path: PathBuf,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
    /// Simulation steps per gait cycle.
    pub cycle_steps: u64,
    /// Maximum relative pitch/roll magnitude before termination in radians.
    pub max_tilt_rad: f64,
}

impl Default for UnitreeGo2EpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_go2_dynamic_scene_path(),
            max_steps: 600,
            cycle_steps: 90,
            max_tilt_rad: 1.2,
        }
    }
}

/// Continuous gait action for the official Go2.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2Action {
    /// Thigh stride amplitude in radians, clamped to `[0, 0.3]`.
    pub stride_rad: f64,
    /// Swing-leg calf flexion in radians, clamped to `[0, 0.4]`.
    pub foot_lift_rad: f64,
}

impl Default for UnitreeGo2Action {
    fn default() -> Self {
        Self {
            stride_rad: 0.12,
            foot_lift_rad: 0.16,
        }
    }
}

/// Observation returned by [`UnitreeGo2Episode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2Observation {
    /// Base X position in meters.
    pub base_x_m: f64,
    /// Base height in meters.
    pub base_y_m: f64,
    /// Base Z position in meters.
    pub base_z_m: f64,
    /// Base yaw in radians.
    pub base_yaw_rad: f64,
    /// Base pitch in radians, including the URDF-to-world basis rotation.
    pub base_pitch_rad: f64,
    /// Base roll in radians, including the URDF-to-world basis rotation.
    pub base_roll_rad: f64,
    /// Base linear velocity in meters per second.
    pub base_linear_velocity_m_s: [f64; 3],
    /// Base angular velocity in radians per second.
    pub base_angular_velocity_rad_s: [f64; 3],
    /// Base yaw relative to the loaded upright pose in radians.
    pub base_relative_yaw_rad: f64,
    /// Base pitch relative to the loaded upright pose in radians.
    pub base_relative_pitch_rad: f64,
    /// Base roll relative to the loaded upright pose in radians.
    pub base_relative_roll_rad: f64,
    /// Planar displacement during the latest step in meters.
    pub locomotion_delta_m: f64,
    /// Front-left foot contact impulse in N·s.
    pub fl_foot_impulse_ns: f64,
    /// Front-right foot contact impulse in N·s.
    pub fr_foot_impulse_ns: f64,
    /// Rear-left foot contact impulse in N·s.
    pub rl_foot_impulse_ns: f64,
    /// Rear-right foot contact impulse in N·s.
    pub rr_foot_impulse_ns: f64,
    /// Normalized gait phase in `[0, 1)`.
    pub gait_phase: f64,
    /// Normalized episode progress in `[0, 1]`.
    pub progress: f64,
}

/// Deterministic locomotion episode for the official Unitree Go2 model.
pub struct UnitreeGo2Episode {
    config: UnitreeGo2EpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
}

impl UnitreeGo2Episode {
    /// Loads and settles the dynamic Go2 multibody.
    pub fn new(config: UnitreeGo2EpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle(&mut sim, config.cycle_steps);
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
        })
    }

    fn observation(&self, locomotion_delta_m: f64) -> UnitreeGo2Observation {
        let base = self.sim.observe();
        UnitreeGo2Observation {
            base_x_m: base.base_x_m,
            base_y_m: base.base_y_m,
            base_z_m: base.base_z_m,
            base_yaw_rad: base.base_yaw_rad,
            base_pitch_rad: base.base_pitch_rad,
            base_roll_rad: base.base_roll_rad,
            base_linear_velocity_m_s: [
                base.base_linear_velocity_x_m_s,
                base.base_linear_velocity_y_m_s,
                base.base_linear_velocity_z_m_s,
            ],
            base_angular_velocity_rad_s: [
                base.base_angular_velocity_x_rad_s,
                base.base_angular_velocity_y_rad_s,
                base.base_angular_velocity_z_rad_s,
            ],
            base_relative_yaw_rad: base.base_relative_yaw_rad,
            base_relative_pitch_rad: base.base_relative_pitch_rad,
            base_relative_roll_rad: base.base_relative_roll_rad,
            locomotion_delta_m,
            fl_foot_impulse_ns: self.sim.link_contact_impulse_ns("FL_foot"),
            fr_foot_impulse_ns: self.sim.link_contact_impulse_ns("FR_foot"),
            rl_foot_impulse_ns: self.sim.link_contact_impulse_ns("RL_foot"),
            rr_foot_impulse_ns: self.sim.link_contact_impulse_ns("RR_foot"),
            gait_phase: (self.step_in_episode % self.config.cycle_steps.max(1)) as f64
                / self.config.cycle_steps.max(1) as f64,
            progress: (self.step_in_episode as f64 / self.config.max_steps.max(1) as f64)
                .clamp(0.0, 1.0),
        }
    }
}

impl Episode for UnitreeGo2Episode {
    type Observation = UnitreeGo2Observation;
    type Action = UnitreeGo2Action;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload Unitree Go2 episode scene");
        settle(&mut self.sim, self.config.cycle_steps);
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
        let before = self.sim.observe();
        let command = UnitreeGo2GaitCommand {
            stride_rad: action.stride_rad.clamp(0.0, 0.3),
            foot_lift_rad: action.foot_lift_rad.clamp(0.0, 0.4),
            cycle_steps: self.config.cycle_steps,
        };
        self.sim
            .step_joint_position_targets(&unitree_go2_trot_targets(self.step_in_episode, command));
        self.step_in_episode += 1;

        let after = self.sim.observe();
        let locomotion_delta_m =
            (after.base_x_m - before.base_x_m).hypot(after.base_z_m - before.base_z_m);
        let observation = self.observation(locomotion_delta_m);
        let tilt_rad = observation
            .base_relative_pitch_rad
            .hypot(observation.base_relative_roll_rad);
        let fallen = observation.base_y_m < FALLEN_HEIGHT_M || tilt_rad > self.config.max_tilt_rad;
        let height_error_m = (observation.base_y_m - NOMINAL_HEIGHT_M).abs();
        let reward = 0.5 + 50.0 * locomotion_delta_m
            - 2.0 * height_error_m
            - 0.2 * tilt_rad
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

fn settle(sim: &mut UrdfSceneSim, cycle_steps: u64) {
    sim.configure_position_motors(180.0, 18.0, 23.7);
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps,
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go2_episode_replays_short_trot_exactly() {
        let config = UnitreeGo2EpisodeConfig {
            max_steps: 16,
            ..Default::default()
        };
        let mut first = UnitreeGo2Episode::new(config.clone()).expect("first Go2 episode");
        let mut second = UnitreeGo2Episode::new(config).expect("second Go2 episode");
        let mut total_reward = 0.0;
        let mut last = None;
        for _ in 0..16 {
            let a = first.step(UnitreeGo2Action::default());
            let b = second.step(UnitreeGo2Action::default());
            assert_eq!(a, b);
            total_reward += a.reward;
            last = Some(a);
        }
        let last = last.expect("Go2 episode step");
        assert!(last.truncated);
        assert!(!last.terminated);
        assert!(total_reward > 0.0);
        assert!(last.observation.gait_phase > 0.0);
    }
}
