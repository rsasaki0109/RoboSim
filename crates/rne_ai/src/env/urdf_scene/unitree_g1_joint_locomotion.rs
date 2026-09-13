//! Joint-space Unitree G1 locomotion episode.
//!
//! This is the OSS-style control boundary that the earlier scripted "stepper"
//! could not express. The action is twelve leg-joint position offsets around a
//! nominal stance, tracked by the hybrid plant (eight proximal joints under
//! torque PD, four ankles position-servoed). Observations follow the
//! reinforcement-learning locomotion convention used by published humanoid
//! stacks: base angular velocity, projected gravity, velocity command, leg
//! joint position/velocity, the previous action, and a gait clock. The reward
//! is the same recipe: forward/yaw velocity tracking, upright, height,
//! lateral-slip and action-rate penalties, a swing air-time bonus, and a fall
//! penalty. It is a runnable, deterministic, headless plant intended to be
//! trained by an external learner (see `examples/93_g1_joint_locomotion_rl`).

use super::{
    step_unitree_g1_hybrid_joint_targets_with_limits, unitree_g1_dynamic_scene_path,
    unitree_g1_gait_targets, UnitreeG1GaitCommand, UrdfJointPositionTarget, UrdfSceneSim,
};
use crate::{
    ActionSpec, Episode, EpisodeStep, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec,
    TaskSpec, TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind,
    TerminationSpec,
};
use rne_assets::AssetError;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 30;
const NOMINAL_HEIGHT_M: f64 = 0.78;
const FALLEN_HEIGHT_M: f64 = 0.45;
const LEG_JOINT_COUNT: usize = 12;
const CONTROL_HZ: f64 = 60.0;

/// The twelve leg joints, in action/observation order: left hip pitch/roll/yaw,
/// knee, ankle pitch/roll, then the same for the right leg.
pub const UNITREE_G1_LEG_JOINT_LINKS: [&str; LEG_JOINT_COUNT] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
];

/// Nominal stance for the twelve leg joints in radians.
pub const UNITREE_G1_LEG_NOMINAL_RAD: [f64; LEG_JOINT_COUNT] = [
    -0.18, 0.05, 0.0, 0.36, -0.18, -0.03, -0.18, -0.05, 0.0, 0.36, -0.18, 0.03,
];

/// Configuration for [`UnitreeG1JointLocomotionEpisode`].
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1JointLocomotionConfig {
    /// Dynamic multibody G1 scene.
    pub scene_path: PathBuf,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
    /// Number of control steps in one gait-clock cycle.
    pub cycle_steps: u64,
    /// Target body-forward velocity in meters per second.
    pub command_forward_m_s: f64,
    /// Target body yaw rate in radians per second.
    pub command_yaw_rate_rad_s: f64,
    /// Maximum relative tilt before termination in radians.
    pub max_tilt_rad: f64,
    /// Pelvis height below which the episode terminates in meters.
    pub fall_height_m: f64,
    /// Nominal pelvis height used by the height reward in meters.
    pub nominal_height_m: f64,
    /// When true, the action is a residual on a nominal periodic leg gait, so a
    /// zero action reproduces the scripted pattern instead of standing still.
    pub nominal_gait: bool,
    /// Nominal hip-pitch stride of the residual gait in radians.
    pub nominal_stride_rad: f64,
    /// Nominal swing-leg knee lift of the residual gait in radians.
    pub nominal_foot_lift_rad: f64,
    /// Position offset per unit action in radians.
    pub action_scale_rad: f64,
    /// Position-servo stiffness for the servo-held joints.
    pub position_stiffness: f64,
    /// Position-servo damping for the servo-held joints.
    pub position_damping: f64,
    /// Proximal torque-PD stiffness in N·m/rad.
    pub torque_pd_stiffness: f64,
    /// Proximal torque-PD damping in N·m/(rad/s).
    pub torque_pd_damping: f64,
    /// Absolute actuator torque limit in N·m.
    pub torque_limit_nm: f64,
    /// Joint speed limit in rad/s used by the torque motor adapter.
    pub speed_limit_rad_s: f64,
}

impl Default for UnitreeG1JointLocomotionConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            max_steps: 500,
            cycle_steps: 60,
            command_forward_m_s: 0.4,
            command_yaw_rate_rad_s: 0.0,
            max_tilt_rad: 1.0,
            fall_height_m: FALLEN_HEIGHT_M,
            nominal_height_m: NOMINAL_HEIGHT_M,
            nominal_gait: true,
            nominal_stride_rad: 0.10,
            nominal_foot_lift_rad: 0.08,
            action_scale_rad: 0.15,
            position_stiffness: 220.0,
            position_damping: 24.0,
            torque_pd_stiffness: 300.0,
            torque_pd_damping: 10.0,
            torque_limit_nm: 88.0,
            speed_limit_rad_s: 20.0,
        }
    }
}

/// Normalized joint-space action in `[-1, 1]^12`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnitreeG1JointAction {
    /// Normalized leg-joint offsets from [`UNITREE_G1_LEG_NOMINAL_RAD`].
    pub leg_action: [f64; LEG_JOINT_COUNT],
}

impl Default for UnitreeG1JointAction {
    fn default() -> Self {
        Self {
            leg_action: [0.0; LEG_JOINT_COUNT],
        }
    }
}

impl UnitreeG1JointAction {
    /// Returns a finite action clamped to the normalized envelope.
    pub fn clamped(self) -> Self {
        let mut leg_action = [0.0; LEG_JOINT_COUNT];
        for (out, value) in leg_action.iter_mut().zip(self.leg_action) {
            *out = if value.is_finite() {
                value.clamp(-1.0, 1.0)
            } else {
                0.0
            };
        }
        Self { leg_action }
    }
}

/// Observation emitted by [`UnitreeG1JointLocomotionEpisode`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnitreeG1JointObservation {
    /// Base angular velocity in the body frame, rad/s.
    pub base_angular_velocity_rad_s: [f64; 3],
    /// Unit gravity direction projected into the body frame.
    pub projected_gravity: [f64; 3],
    /// Commanded body-forward speed and yaw rate.
    pub command: [f64; 2],
    /// Leg joint positions in radians.
    pub joint_position_rad: [f64; LEG_JOINT_COUNT],
    /// Leg joint velocities in radians per second.
    pub joint_velocity_rad_s: [f64; LEG_JOINT_COUNT],
    /// Previous normalized action.
    pub previous_action: [f64; LEG_JOINT_COUNT],
    /// Gait-clock sine and cosine.
    pub gait_clock: [f64; 2],
}

/// Returns the portable task contract for the joint-space G1 locomotion episode.
pub fn unitree_g1_joint_locomotion_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        "rne.unitree_g1.joint_locomotion.v1",
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new(
                "base_angular_velocity_rad_s",
                TensorDType::F64,
                vec![3],
                "rad/s",
            ),
            TensorSpec::new("projected_gravity", TensorDType::F64, vec![3], "1"),
            TensorSpec::new("command", TensorDType::F64, vec![2], "m/s"),
            TensorSpec::new("joint_position_rad", TensorDType::F64, vec![12], "rad"),
            TensorSpec::new("joint_velocity_rad_s", TensorDType::F64, vec![12], "rad/s"),
            TensorSpec::new("previous_action", TensorDType::F64, vec![12], "1"),
            TensorSpec::new("gait_clock", TensorDType::F64, vec![2], "1"),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "leg_action",
            TensorDType::F64,
            vec![LEG_JOINT_COUNT],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(-1.0, 1.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("forward_velocity_tracking", 3.0, "1"),
            RewardTermSpec::new("forward_progress_m_s", 1.0, "m/s"),
            RewardTermSpec::new("yaw_rate_tracking", 0.5, "1"),
            RewardTermSpec::new("upright", 1.0, "1"),
            RewardTermSpec::new("height_error_m", -2.0, "m"),
            RewardTermSpec::new("lateral_velocity_squared", -1.0, "(m/s)^2"),
            RewardTermSpec::new("action_rate_squared", -0.02, "1"),
            RewardTermSpec::new("action_squared", -0.001, "1"),
            RewardTermSpec::new("swing_air_time_s", 1.0, "s"),
            RewardTermSpec::new("fallen", -10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "fallen",
                TerminationKind::Failure,
            )],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Deterministic joint-space forward-locomotion episode for the official G1.
pub struct UnitreeG1JointLocomotionEpisode {
    config: UnitreeG1JointLocomotionConfig,
    sim: UrdfSceneSim,
    scene_seed: Option<u64>,
    episode_index: u32,
    step_in_episode: u64,
    previous_action: [f64; LEG_JOINT_COUNT],
    left_air_steps: u64,
    right_air_steps: u64,
    previous_left_contact: bool,
    previous_right_contact: bool,
}

impl UnitreeG1JointLocomotionEpisode {
    /// Loads the dynamic G1 and settles it into the nominal standing pose.
    pub fn new(config: UnitreeG1JointLocomotionConfig) -> Result<Self, AssetError> {
        Self::new_with_scene_seed(config, None)
    }

    /// Loads the dynamic G1 with an explicit deterministic world seed.
    pub fn new_with_seed(
        config: UnitreeG1JointLocomotionConfig,
        seed: u64,
    ) -> Result<Self, AssetError> {
        Self::new_with_scene_seed(config, Some(seed))
    }

    /// Returns read access to the underlying scene simulation.
    pub fn sim(&self) -> &UrdfSceneSim {
        &self.sim
    }

    /// Returns the episode configuration.
    pub fn config(&self) -> &UnitreeG1JointLocomotionConfig {
        &self.config
    }

    fn new_with_scene_seed(
        config: UnitreeG1JointLocomotionConfig,
        scene_seed: Option<u64>,
    ) -> Result<Self, AssetError> {
        let mut sim = match scene_seed {
            Some(seed) => UrdfSceneSim::from_scene_path_with_seed(&config.scene_path, seed)?,
            None => UrdfSceneSim::from_scene_path(&config.scene_path)?,
        };
        settle(&mut sim, &config);
        Ok(Self {
            config,
            sim,
            scene_seed,
            episode_index: 0,
            step_in_episode: 0,
            previous_action: [0.0; LEG_JOINT_COUNT],
            left_air_steps: 0,
            right_air_steps: 0,
            previous_left_contact: true,
            previous_right_contact: true,
        })
    }

    fn observation(&self, previous_action: &[f64; LEG_JOINT_COUNT]) -> UnitreeG1JointObservation {
        let base = self.sim.observe();
        let pelvis = self
            .sim
            .named_transform("pelvis")
            .expect("G1 pelvis transform");
        let gravity = pelvis.rotation.inverse() * Vec3::new(0.0, -1.0, 0.0);
        let mut joint_position_rad = [0.0; LEG_JOINT_COUNT];
        let mut joint_velocity_rad_s = [0.0; LEG_JOINT_COUNT];
        for (index, link) in UNITREE_G1_LEG_JOINT_LINKS.iter().enumerate() {
            joint_position_rad[index] = self.sim.named_joint_position(link).unwrap_or(0.0);
            joint_velocity_rad_s[index] = self.sim.named_joint_velocity(link).unwrap_or(0.0);
        }
        let cycle = self.config.cycle_steps.max(1) as f64;
        let phase = std::f64::consts::TAU
            * (self.step_in_episode % self.config.cycle_steps.max(1)) as f64
            / cycle;
        UnitreeG1JointObservation {
            base_angular_velocity_rad_s: [
                base.base_angular_velocity_x_rad_s,
                base.base_angular_velocity_y_rad_s,
                base.base_angular_velocity_z_rad_s,
            ],
            projected_gravity: [gravity.x, gravity.y, gravity.z],
            command: [
                self.config.command_forward_m_s,
                self.config.command_yaw_rate_rad_s,
            ],
            joint_position_rad,
            joint_velocity_rad_s,
            previous_action: *previous_action,
            gait_clock: [phase.sin(), phase.cos()],
        }
    }
}

impl Episode for UnitreeG1JointLocomotionEpisode {
    type Observation = UnitreeG1JointObservation;
    type Action = UnitreeG1JointAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = match self.scene_seed {
            Some(seed) => UrdfSceneSim::from_scene_path_with_seed(&self.config.scene_path, seed)
                .expect("reload seeded G1 joint locomotion scene"),
            None => UrdfSceneSim::from_scene_path(&self.config.scene_path)
                .expect("reload G1 joint locomotion scene"),
        };
        settle(&mut self.sim, &self.config);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        self.previous_action = [0.0; LEG_JOINT_COUNT];
        self.left_air_steps = 0;
        self.right_air_steps = 0;
        self.previous_left_contact = true;
        self.previous_right_contact = true;
        EpisodeStep {
            observation: self.observation(&self.previous_action),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let action = action.clamped();
        let targets = action_targets(&self.config, &action, self.step_in_episode);
        step_unitree_g1_hybrid_joint_targets_with_limits(
            &mut self.sim,
            &targets,
            [0.0; 8],
            self.config.torque_pd_stiffness,
            self.config.torque_pd_damping,
            self.config.torque_limit_nm,
            self.config.speed_limit_rad_s,
        );
        self.step_in_episode += 1;

        let base = self.sim.observe();
        let pelvis = self
            .sim
            .named_transform("pelvis")
            .expect("G1 pelvis transform");
        let world_velocity = Vec3::new(
            base.base_linear_velocity_x_m_s,
            base.base_linear_velocity_y_m_s,
            base.base_linear_velocity_z_m_s,
        );
        let body_velocity = pelvis.rotation.inverse() * world_velocity;
        let forward_m_s = body_velocity.z;
        let lateral_m_s = body_velocity.x;
        let up = pelvis.rotation.inverse() * Vec3::Y;
        let yaw_rate_rad_s = base.base_angular_velocity_y_rad_s;

        let left_contact = self.sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0;
        let right_contact = self.sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0;
        let dt_s = 1.0 / CONTROL_HZ;
        let mut swing_air_time_s = 0.0;
        if left_contact {
            if !self.previous_left_contact {
                swing_air_time_s += self.left_air_steps as f64
                    * dt_s
                    * swing_bonus(self.config.command_forward_m_s);
            }
            self.left_air_steps = 0;
        } else {
            self.left_air_steps += 1;
        }
        if right_contact {
            if !self.previous_right_contact {
                swing_air_time_s += self.right_air_steps as f64
                    * dt_s
                    * swing_bonus(self.config.command_forward_m_s);
            }
            self.right_air_steps = 0;
        } else {
            self.right_air_steps += 1;
        }
        self.previous_left_contact = left_contact;
        self.previous_right_contact = right_contact;

        let mut action_rate_squared = 0.0;
        let mut action_squared = 0.0;
        for index in 0..LEG_JOINT_COUNT {
            let delta = action.leg_action[index] - self.previous_action[index];
            action_rate_squared += delta * delta;
            action_squared += action.leg_action[index] * action.leg_action[index];
        }

        let tilt_rad = base
            .base_relative_pitch_rad
            .hypot(base.base_relative_roll_rad);
        let fallen =
            base.base_y_m < self.config.fall_height_m || tilt_rad > self.config.max_tilt_rad;
        let height_error_m = (base.base_y_m - self.config.nominal_height_m).abs();
        let tracking = (-(forward_m_s - self.config.command_forward_m_s).powi(2) / 0.25).exp();
        let yaw_tracking =
            (-(yaw_rate_rad_s - self.config.command_yaw_rate_rad_s).powi(2) / 0.25).exp();

        let mut reward = 3.0 * tracking;
        reward += forward_m_s.clamp(-0.5, 1.0);
        reward += 0.5 * yaw_tracking;
        reward += 1.0 * up.y;
        reward -= 2.0 * height_error_m;
        reward -= 1.0 * lateral_m_s * lateral_m_s;
        reward -= 0.02 * action_rate_squared;
        reward -= 0.001 * action_squared;
        reward += swing_air_time_s;
        if fallen {
            reward -= 10.0;
        }

        self.previous_action = action.leg_action;
        let observation = self.observation(&action.leg_action);
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

/// Builds the twenty-three hybrid joint targets for one normalized leg action.
///
/// Without a nominal gait the base is the standing pose. With one, the base is
/// the scripted periodic gait at `step`, so the action is a residual that can
/// reshape a stepping pattern rather than create one from scratch.
fn action_targets(
    config: &UnitreeG1JointLocomotionConfig,
    action: &UnitreeG1JointAction,
    step: u64,
) -> [UrdfJointPositionTarget<'static>; 23] {
    let (base_step, base_command) = if config.nominal_gait {
        (
            step,
            UnitreeG1GaitCommand {
                stride_rad: config.nominal_stride_rad,
                foot_lift_rad: config.nominal_foot_lift_rad,
                cycle_steps: config.cycle_steps,
            },
        )
    } else {
        (
            0,
            UnitreeG1GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: config.cycle_steps,
            },
        )
    };
    let mut targets = unitree_g1_gait_targets(base_step, base_command);
    for (index, link) in UNITREE_G1_LEG_JOINT_LINKS.iter().enumerate() {
        targets[index] = UrdfJointPositionTarget {
            link_name: link,
            position: targets[index].position + action.leg_action[index] * config.action_scale_rad,
        };
    }
    targets
}

/// Configuration for a parallel batch of [`UnitreeG1JointLocomotionEpisode`]s.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedUnitreeG1JointLocomotionConfig {
    /// Per-environment episode configuration.
    pub episode: UnitreeG1JointLocomotionConfig,
    /// Number of parallel environments.
    pub num_envs: usize,
    /// Root seed; environment `i` uses `seed + i`.
    pub seed: u64,
    /// When true, a finished environment is reset during the next step.
    pub auto_reset: bool,
}

impl Default for VectorizedUnitreeG1JointLocomotionConfig {
    fn default() -> Self {
        Self {
            episode: UnitreeG1JointLocomotionConfig::default(),
            num_envs: 8,
            seed: 1,
            auto_reset: true,
        }
    }
}

/// One parallel batch step result, in stable environment-index order.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedUnitreeG1JointLocomotionStep {
    /// Observations after the step, or the reset observation if auto-reset ran.
    pub observations: Vec<UnitreeG1JointObservation>,
    /// Rewards from the step (the terminal reward is retained across auto-reset).
    pub rewards: Vec<f64>,
    /// Terminal flags.
    pub terminated: Vec<bool>,
    /// Truncation flags.
    pub truncated: Vec<bool>,
    /// Whether the returned observation is a fresh reset.
    pub resets: Vec<bool>,
    /// Zero-based episode index per environment.
    pub episode_indices: Vec<u32>,
}

type BatchStepSlot = (UnitreeG1JointObservation, f64, bool, bool, bool, u32);

/// Runs many [`UnitreeG1JointLocomotionEpisode`]s in parallel with `std::thread`.
///
/// This is the fast training boundary: one Rust call steps every environment on
/// the available cores, avoiding the per-step Python round-trip that caps the
/// `SubprocVecEnv` path at a few hundred steps per second.
pub struct VectorizedUnitreeG1JointLocomotionEnv {
    episodes: Vec<UnitreeG1JointLocomotionEpisode>,
    auto_reset: bool,
}

impl VectorizedUnitreeG1JointLocomotionEnv {
    /// Creates and seeds every environment in stable index order.
    pub fn new(config: VectorizedUnitreeG1JointLocomotionConfig) -> Result<Self, AssetError> {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|index| {
                UnitreeG1JointLocomotionEpisode::new_with_seed(
                    config.episode.clone(),
                    config.seed.wrapping_add(index as u64),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            episodes,
            auto_reset: config.auto_reset,
        })
    }

    /// Returns the number of parallel environments.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// Returns read access to one underlying episode.
    pub fn episode(&self, index: usize) -> &UnitreeG1JointLocomotionEpisode {
        &self.episodes[index]
    }

    /// Resets every environment in parallel.
    pub fn reset(&mut self) -> VectorizedUnitreeG1JointLocomotionStep {
        let n = self.episodes.len();
        let threads = batch_threads(n);
        let chunk = n.div_ceil(threads);
        let mut slots: Vec<Option<UnitreeG1JointObservation>> = (0..n).map(|_| None).collect();
        std::thread::scope(|scope| {
            for (episodes, slots) in self.episodes.chunks_mut(chunk).zip(slots.chunks_mut(chunk)) {
                scope.spawn(move || {
                    for (episode, slot) in episodes.iter_mut().zip(slots.iter_mut()) {
                        *slot = Some(episode.reset().observation);
                    }
                });
            }
        });
        VectorizedUnitreeG1JointLocomotionStep {
            observations: slots
                .into_iter()
                .map(|slot| slot.expect("batch slot"))
                .collect(),
            rewards: vec![0.0; n],
            terminated: vec![false; n],
            truncated: vec![false; n],
            resets: vec![true; n],
            episode_indices: self
                .episodes
                .iter()
                .map(|episode| episode.episode_index())
                .collect(),
        }
    }

    /// Applies one action per environment in parallel and returns the batch result.
    pub fn step(
        &mut self,
        actions: &[UnitreeG1JointAction],
    ) -> VectorizedUnitreeG1JointLocomotionStep {
        self.step_repeat(actions, 1)
    }

    /// Applies one action per environment for `repeat` ticks before returning.
    ///
    /// Repeating a held action is the standard decimation used by OSS
    /// locomotion stacks and amortizes the per-call thread and FFI overhead.
    /// Rewards are summed; a terminal exit ends that environment's repeat.
    pub fn step_repeat(
        &mut self,
        actions: &[UnitreeG1JointAction],
        repeat: u32,
    ) -> VectorizedUnitreeG1JointLocomotionStep {
        let repeat = repeat.max(1);
        let n = self.episodes.len();
        assert_eq!(actions.len(), n, "one action per environment");
        let auto_reset = self.auto_reset;
        let threads = batch_threads(n);
        let chunk = n.div_ceil(threads);
        let mut slots: Vec<Option<BatchStepSlot>> = (0..n).map(|_| None).collect();
        std::thread::scope(|scope| {
            for ((episodes, slots), chunk_actions) in self
                .episodes
                .chunks_mut(chunk)
                .zip(slots.chunks_mut(chunk))
                .zip(actions.chunks(chunk))
            {
                scope.spawn(move || {
                    for ((episode, slot), action) in episodes
                        .iter_mut()
                        .zip(slots.iter_mut())
                        .zip(chunk_actions.iter())
                    {
                        let episode_index = episode.episode_index();
                        let mut reward = 0.0;
                        let mut terminated = false;
                        let mut truncated = false;
                        let mut observation = None;
                        for _ in 0..repeat {
                            let step = episode.step(*action);
                            reward += step.reward;
                            terminated = step.terminated;
                            truncated = step.truncated;
                            observation = Some(step.observation);
                            if terminated || truncated {
                                break;
                            }
                        }
                        let mut observation = observation.expect("repeat is at least one step");
                        let mut reset = false;
                        if (terminated || truncated) && auto_reset {
                            observation = episode.reset().observation;
                            reset = true;
                        }
                        *slot = Some((
                            observation,
                            reward,
                            terminated,
                            truncated,
                            reset,
                            episode_index,
                        ));
                    }
                });
            }
        });
        let mut result = VectorizedUnitreeG1JointLocomotionStep {
            observations: Vec::with_capacity(n),
            rewards: Vec::with_capacity(n),
            terminated: Vec::with_capacity(n),
            truncated: Vec::with_capacity(n),
            resets: Vec::with_capacity(n),
            episode_indices: Vec::with_capacity(n),
        };
        for slot in slots {
            let (observation, reward, terminated, truncated, reset, episode_index) =
                slot.expect("batch slot");
            result.observations.push(observation);
            result.rewards.push(reward);
            result.terminated.push(terminated);
            result.truncated.push(truncated);
            result.resets.push(reset);
            result.episode_indices.push(episode_index);
        }
        result
    }
}

fn batch_threads(num_envs: usize) -> usize {
    std::thread::available_parallelism()
        .map(|threads| threads.get())
        .unwrap_or(1)
        .min(num_envs)
        .max(1)
}

fn swing_bonus(command_forward_m_s: f64) -> f64 {
    if command_forward_m_s > 0.05 {
        1.0
    } else {
        0.0
    }
}

fn settle(sim: &mut UrdfSceneSim, config: &UnitreeG1JointLocomotionConfig) {
    sim.configure_position_motors(
        config.position_stiffness,
        config.position_damping,
        config.torque_limit_nm,
    );
    let mut standing = config.clone();
    standing.nominal_gait = false;
    let targets = action_targets(&standing, &UnitreeG1JointAction::default(), 0);
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn task_spec_exposes_46_observations_and_12_actions() {
        let spec = unitree_g1_joint_locomotion_task_spec(500);
        let observation_dim: usize = spec
            .observation
            .tensors
            .iter()
            .map(|tensor| tensor.shape.iter().product::<usize>())
            .sum();
        let action_dim: usize = spec
            .action
            .tensors
            .iter()
            .map(|tensor| tensor.shape.iter().product::<usize>())
            .sum();
        assert_eq!(observation_dim, 46);
        assert_eq!(action_dim, 12);
    }

    #[test]
    fn zero_action_keeps_the_g1_upright_and_finite() {
        let mut episode = UnitreeG1JointLocomotionEpisode::new(UnitreeG1JointLocomotionConfig {
            max_steps: 60,
            ..UnitreeG1JointLocomotionConfig::default()
        })
        .expect("G1 joint locomotion episode");
        let mut step = episode.reset();
        for _ in 0..60 {
            step = episode.step(UnitreeG1JointAction::default());
        }
        assert!(!step.terminated, "zero action must not fall in one second");
        assert!(step
            .observation
            .joint_position_rad
            .iter()
            .all(|v| v.is_finite()));
        assert!(step
            .observation
            .projected_gravity
            .iter()
            .all(|v| v.is_finite()));
        assert!(step.reward.is_finite());
    }

    #[test]
    fn vectorized_batch_steps_in_index_order_and_auto_resets() {
        let mut env =
            VectorizedUnitreeG1JointLocomotionEnv::new(VectorizedUnitreeG1JointLocomotionConfig {
                episode: UnitreeG1JointLocomotionConfig {
                    max_steps: 20,
                    ..UnitreeG1JointLocomotionConfig::default()
                },
                num_envs: 4,
                seed: 7,
                auto_reset: true,
            })
            .expect("vectorized G1 batch");
        let reset = env.reset();
        assert_eq!(reset.observations.len(), 4);
        assert_eq!(reset.episode_indices, vec![1, 1, 1, 1]);
        let actions = vec![UnitreeG1JointAction::default(); 4];
        let mut last = env.step(&actions);
        for _ in 0..25 {
            last = env.step(&actions);
        }
        assert!(
            last.episode_indices.iter().any(|index| *index > 1),
            "envs past max_steps must auto-reset"
        );
        assert!(last.observations.iter().all(|observation| observation
            .joint_position_rad
            .iter()
            .all(|value| value.is_finite())));
    }

    #[test]
    fn action_bounds_are_enforced() {
        let action = UnitreeG1JointAction {
            leg_action: [9.0; LEG_JOINT_COUNT],
        }
        .clamped();
        assert!(action.leg_action.iter().all(|value| *value == 1.0));
    }
}
