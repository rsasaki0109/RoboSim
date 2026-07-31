use super::{
    unitree_go2_dynamic_scene_path, unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 120;
const NOMINAL_HEIGHT_M: f64 = 0.23;
const FALLEN_HEIGHT_M: f64 = 0.12;

/// A deterministic shove applied to the Go2 base during an episode.
///
/// Expressed as a roll tilt about the body's forward axis. Rapier multibody links do
/// not respond to body-level forces or velocity writes, and a root translation
/// teleports the whole tree feet-included; a rotation instead changes the contact
/// configuration — feet on one side press into the ground, the other side lifts —
/// which produces the tipping dynamics a balance controller must catch.
///
/// With `duration_steps > 1` the total tilt is spread evenly across consecutive
/// steps, which models a *sustained* lean — someone pressing on the flank across
/// several gait cycles rather than slapping it once. Stiff position motors snap
/// back from any instantaneous tilt, so only a sustained push can separate a
/// balance controller from an open-loop gait.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2Push {
    /// Controlled step at which the shove starts.
    pub step: u64,
    /// Total roll tilt about the body forward axis, in radians.
    pub roll_tilt_rad: f64,
    /// Steps the shove is spread across; `1` is an instantaneous slap.
    pub duration_steps: u64,
}

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
    /// Optional disturbance shove; `None` reproduces the undisturbed episode exactly.
    pub push: Option<UnitreeGo2Push>,
    /// Joint position-motor stiffness (proportional gain).
    ///
    /// The default `180` is stiff enough to make the scripted trot passively stable
    /// against instantaneous tilts. Lowering it produces a *compliant* gait whose
    /// balance genuinely depends on feedback — the regime a fall-versus-save
    /// evaluation needs.
    pub motor_stiffness: f64,
    /// Joint position-motor damping (derivative gain).
    pub motor_damping: f64,
    /// Joint motor force limit in newtons, matching the real Go2 actuator.
    pub motor_max_force_n: f64,
}

impl Default for UnitreeGo2EpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_go2_dynamic_scene_path(),
            max_steps: 600,
            cycle_steps: 90,
            max_tilt_rad: 1.2,
            push: None,
            motor_stiffness: 180.0,
            motor_damping: 18.0,
            motor_max_force_n: 23.7,
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
    /// Hip-abduction posture correction in radians, clamped to `[-0.8, 0.8]`.
    ///
    /// A balance controller feeds measured body roll back through this term; the
    /// open-loop trot leaves it at zero.
    pub roll_correction_rad: f64,
    /// Thigh-pitch posture correction in radians, clamped to `[-0.3, 0.3]`.
    pub pitch_correction_rad: f64,
    /// Left/right differential calf extension in radians, clamped to `[-0.5, 0.5]`.
    ///
    /// The leg-length recovery channel: positive lengthens the left legs and
    /// shortens the right legs, rolling the body toward the right.
    pub lateral_extension_rad: f64,
}

impl Default for UnitreeGo2Action {
    fn default() -> Self {
        Self {
            stride_rad: 0.12,
            foot_lift_rad: 0.16,
            roll_correction_rad: 0.0,
            pitch_correction_rad: 0.0,
            lateral_extension_rad: 0.0,
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
        settle(&mut sim, &config);
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
        })
    }

    /// Read access to the underlying scene simulation, for rendering the episode.
    pub fn sim(&self) -> &UrdfSceneSim {
        &self.sim
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
        settle(&mut self.sim, &self.config);
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
        // The disturbance lands before the control targets apply, exactly like a
        // shove that arrives between two controller ticks. A sustained push spreads
        // the total tilt evenly across `duration_steps` consecutive steps.
        if let Some(push) = self.config.push {
            let duration = push.duration_steps.max(1);
            let active =
                self.step_in_episode >= push.step && self.step_in_episode < push.step + duration;
            if active {
                // Tilt about the body X axis: physically the lateral lean, which this
                // observation convention reports on `base_relative_pitch_rad`, and
                // exactly the axis the hip-abduction correction actuates.
                if let Some(pose) = self.sim.named_transform("base") {
                    let axis = (pose.rotation * rne_math::Vec3::X).normalize_or_zero();
                    let axis_angle = axis * (push.roll_tilt_rad / duration as f64);
                    self.sim
                        .tilt_named_body_rad("base", [axis_angle.x, axis_angle.y, axis_angle.z]);
                }
            }
        }
        let command = UnitreeGo2GaitCommand {
            stride_rad: action.stride_rad.clamp(0.0, 0.3),
            foot_lift_rad: action.foot_lift_rad.clamp(0.0, 0.4),
            cycle_steps: self.config.cycle_steps,
            roll_correction_rad: action.roll_correction_rad.clamp(-0.8, 0.8),
            pitch_correction_rad: action.pitch_correction_rad.clamp(-0.3, 0.3),
            lateral_extension_rad: action.lateral_extension_rad.clamp(-0.5, 0.5),
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

fn settle(sim: &mut UrdfSceneSim, config: &UnitreeGo2EpisodeConfig) {
    sim.configure_position_motors(
        config.motor_stiffness,
        config.motor_damping,
        config.motor_max_force_n,
    );
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: config.cycle_steps,
            ..UnitreeGo2GaitCommand::default()
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

    /// Outcome of one pushed trot run.
    struct PushOutcome {
        fell: bool,
        max_tilt_rad: f64,
        steps_survived: u64,
    }

    fn run_trot(
        config: UnitreeGo2EpisodeConfig,
        mut controller: impl FnMut(&UnitreeGo2Observation) -> UnitreeGo2Action,
    ) -> PushOutcome {
        let mut episode = UnitreeGo2Episode::new(config).expect("pushed Go2 episode");
        let mut observation = episode.reset().observation;
        let mut max_tilt_rad = 0.0_f64;
        let mut steps_survived = 0;
        loop {
            let step = episode.step(controller(&observation));
            observation = step.observation;
            max_tilt_rad = max_tilt_rad.max(
                observation
                    .base_relative_pitch_rad
                    .hypot(observation.base_relative_roll_rad),
            );
            steps_survived += 1;
            if step.terminated {
                return PushOutcome {
                    fell: true,
                    max_tilt_rad,
                    steps_survived,
                };
            }
            if step.truncated {
                return PushOutcome {
                    fell: false,
                    max_tilt_rad,
                    steps_survived,
                };
            }
        }
    }

    fn open_loop(_: &UnitreeGo2Observation) -> UnitreeGo2Action {
        UnitreeGo2Action::default()
    }

    /// Stiff-motor config with a single instantaneous slap at step 180.
    fn stiff_push_config(roll_tilt_rad: f64) -> UnitreeGo2EpisodeConfig {
        UnitreeGo2EpisodeConfig {
            max_steps: 420,
            push: Some(UnitreeGo2Push {
                step: 180,
                roll_tilt_rad,
                duration_steps: 1,
            }),
            ..Default::default()
        }
    }

    /// Proportional-derivative posture feedback from the measured base attitude.
    fn posture_feedback_with(
        roll_gain: f64,
    ) -> impl Fn(&UnitreeGo2Observation) -> UnitreeGo2Action {
        move |observation| {
            let roll = observation.base_relative_roll_rad;
            let roll_rate = observation.base_angular_velocity_rad_s[0];
            UnitreeGo2Action {
                roll_correction_rad: roll_gain * (roll + 0.1 * roll_rate),
                ..UnitreeGo2Action::default()
            }
        }
    }

    #[test]
    fn axis_derivation_probe() {
        // Two empirical measurements fix the feedback signs without guessing
        // conventions: which observation axis a body-forward-axis tilt lands on, and
        // which way a constant hip correction leans the standing body.
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("load Go2 scene");
        settle(&mut sim, &UnitreeGo2EpisodeConfig::default());
        let pose = sim.named_transform("base").expect("base pose");
        // Empirically, rotation about the body X axis registers as relative PITCH in
        // this observation convention, so the roll axis is the body Z axis.
        let roll_axis = (pose.rotation * rne_math::Vec3::Z).normalize_or_zero();
        assert!(sim.tilt_named_body_rad(
            "base",
            [roll_axis.x * 0.15, roll_axis.y * 0.15, roll_axis.z * 0.15]
        ));
        sim.step_joint_position_targets(&[]);
        let tilted = sim.observe();
        println!(
            "body-roll tilt +0.15: rel_roll={:.3} rel_pitch={:.3}",
            tilted.base_relative_roll_rad, tilted.base_relative_pitch_rad
        );
        // A body-forward-axis tilt must land on the relative roll observation.
        assert!(
            tilted.base_relative_roll_rad.abs() > 0.05,
            "roll tilt must register on rel_roll, got {:.3}",
            tilted.base_relative_roll_rad
        );

        // Which actuation pattern moves which observation axis? Measure uniform
        // hip, uniform thigh, and front/back differential thigh on a standing robot.
        let lean_of = |targets: [crate::env::urdf_scene::UrdfJointPositionTarget<'static>; 12]| {
            let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
                .expect("load Go2 scene");
            settle(&mut sim, &UnitreeGo2EpisodeConfig::default());
            for _ in 0..240 {
                sim.step_joint_position_targets(&targets);
            }
            let observed = sim.observe();
            (
                observed.base_relative_roll_rad,
                observed.base_relative_pitch_rad,
            )
        };

        let uniform_hip = lean_of(unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 90,
                roll_correction_rad: 0.2,
                pitch_correction_rad: 0.0,
                lateral_extension_rad: 0.0,
            },
        ));
        println!(
            "uniform hip +0.2: roll={:.3} pitch={:.3}",
            uniform_hip.0, uniform_hip.1
        );

        let uniform_thigh = lean_of(unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 90,
                roll_correction_rad: 0.0,
                pitch_correction_rad: 0.2,
                lateral_extension_rad: 0.0,
            },
        ));
        println!(
            "uniform thigh +0.2: roll={:.3} pitch={:.3}",
            uniform_thigh.0, uniform_thigh.1
        );

        // Front/back differential thigh, hand-built.
        let mut differential = unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 90,
                roll_correction_rad: 0.0,
                pitch_correction_rad: 0.0,
                lateral_extension_rad: 0.0,
            },
        );
        for target in differential.iter_mut() {
            if target.link_name.ends_with("_thigh") {
                let front = target.link_name.starts_with('F');
                target.position += if front { 0.15 } else { -0.15 };
            }
        }
        let differential_thigh = lean_of(differential);
        println!(
            "diff thigh F+0.15/R-0.15: roll={:.3} pitch={:.3}",
            differential_thigh.0, differential_thigh.1
        );

        // Left/right differential calf extension: the leg-length channel.
        let differential_calf = lean_of(unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 90,
                roll_correction_rad: 0.0,
                pitch_correction_rad: 0.0,
                lateral_extension_rad: 0.4,
            },
        ));
        println!(
            "diff calf L+0.4/R-0.4: roll={:.3} pitch={:.3}",
            differential_calf.0, differential_calf.1
        );

        // Pin the measured actuation map the feedback pairing relies on: uniform
        // hip abduction actuates the same axis a body-X tilt disturbs (labelled
        // relative pitch by this observation), no leg pattern actuates the
        // orthogonal rel_roll axis, and thigh offsets barely move the attitude.
        assert!(
            uniform_hip.1.abs() > 0.05,
            "uniform hip must actuate the lean axis, got {:.3}",
            uniform_hip.1
        );
        assert!(uniform_hip.0.abs() < 0.05);
        assert!(uniform_thigh.0.abs() < 0.05 && uniform_thigh.1.abs() < 0.05);
        assert!(differential_thigh.0.abs() < 0.05 && differential_thigh.1.abs() < 0.05);
        // The leg-length channel actuates the same lean axis, independently of the
        // hips — the second authority the two-channel save controller relies on.
        assert!(
            differential_calf.1 > 0.15,
            "differential calf must actuate the lean axis, got {:.3}",
            differential_calf.1
        );
        assert!(differential_calf.0.abs() < 0.1);
    }

    /// End state of a run stepped to a fixed horizon, ignoring termination — the
    /// physical outcome after the episode has already scored the fall.
    struct FullRunOutcome {
        max_tilt_rad: f64,
        end_height_m: f64,
        end_tilt_rad: f64,
    }

    fn run_to_horizon(
        config: UnitreeGo2EpisodeConfig,
        mut controller: impl FnMut(&UnitreeGo2Observation) -> UnitreeGo2Action,
        horizon_steps: u64,
    ) -> FullRunOutcome {
        let mut episode = UnitreeGo2Episode::new(config).expect("full-horizon Go2 episode");
        let mut observation = episode.reset().observation;
        let mut max_tilt_rad = 0.0_f64;
        for _ in 0..horizon_steps {
            observation = episode.step(controller(&observation)).observation;
            max_tilt_rad = max_tilt_rad.max(
                observation
                    .base_relative_pitch_rad
                    .hypot(observation.base_relative_roll_rad),
            );
        }
        FullRunOutcome {
            max_tilt_rad,
            end_height_m: observation.base_y_m,
            end_tilt_rad: observation
                .base_relative_pitch_rad
                .hypot(observation.base_relative_roll_rad),
        }
    }

    /// The empirically probed fall/save boundary: torque-limited motors and a
    /// sustained flank push strong enough to beat the passive recovery rate.
    fn weak_motor_push_config() -> UnitreeGo2EpisodeConfig {
        UnitreeGo2EpisodeConfig {
            max_steps: 480,
            push: Some(UnitreeGo2Push {
                step: 180,
                roll_tilt_rad: 1.8,
                duration_steps: 20,
            }),
            motor_max_force_n: 8.0,
            ..Default::default()
        }
    }

    /// The save controller for the fall-versus-save scenario.
    ///
    /// Hip abduction alone saturates its ±0.8 rad clamp and can only brace the
    /// fall in a deep propped lean; adding the independent leg-length channel is
    /// what turns the brace into a standing save.
    fn save_feedback() -> impl FnMut(&UnitreeGo2Observation) -> UnitreeGo2Action {
        two_channel_feedback(2.5, 5.0)
    }

    /// Two-channel recovery feedback: hip abduction plus differential calf
    /// extension, both driven from the measured lean.
    fn two_channel_feedback(
        ext_p_gain: f64,
        ext_d_gain: f64,
    ) -> impl FnMut(&UnitreeGo2Observation) -> UnitreeGo2Action {
        let mut previous_lean = 0.0_f64;
        move |observation| {
            let lean = observation.base_relative_pitch_rad;
            let lean_rate = lean - previous_lean;
            previous_lean = lean;
            UnitreeGo2Action {
                roll_correction_rad: 1.6 * lean + 6.0 * lean_rate,
                lateral_extension_rad: -(ext_p_gain * lean + ext_d_gain * lean_rate),
                ..UnitreeGo2Action::default()
            }
        }
    }

    /// Mid-walk shove shared by the motion-is-stability test and example 53.
    fn mid_walk_push_config(cycle_steps: u64) -> UnitreeGo2EpisodeConfig {
        UnitreeGo2EpisodeConfig {
            max_steps: 900,
            cycle_steps,
            push: Some(UnitreeGo2Push {
                step: 450,
                roll_tilt_rad: 1.8,
                duration_steps: 20,
            }),
            motor_max_force_n: 8.0,
            ..Default::default()
        }
    }

    #[test]
    fn walking_trot_shrugs_off_the_push_that_topples_the_slow_trot() {
        // Same 8 N*m motors, same sustained 1.8 rad flank push. The slow trot
        // (90-step cycle, 0.12 stride) capsizes; the fast walking trot (45-step
        // cycle, 0.24 stride, ~0.17 m/s) leans to ~0.9 rad and recovers with no
        // controller at all — cyclic foot replanting is itself a stabilizer.
        // This is measured plant physics, not a tuned demo.
        let slow = run_to_horizon(mid_walk_push_config(90), open_loop, 900);
        assert!(
            slow.end_tilt_rad > 1.3,
            "slow trot must capsize, got end tilt {:.2}",
            slow.end_tilt_rad
        );

        let walk_action = |_: &UnitreeGo2Observation| UnitreeGo2Action {
            stride_rad: 0.24,
            ..UnitreeGo2Action::default()
        };
        let mut episode =
            UnitreeGo2Episode::new(mid_walk_push_config(45)).expect("walking Go2 episode");
        let mut observation = episode.reset().observation;
        let start = (observation.base_x_m, observation.base_z_m);
        let mut max_tilt = 0.0_f64;
        for _ in 0..900 {
            observation = episode.step(walk_action(&observation)).observation;
            max_tilt = max_tilt.max(
                observation
                    .base_relative_pitch_rad
                    .hypot(observation.base_relative_roll_rad),
            );
        }
        let end_tilt = observation
            .base_relative_pitch_rad
            .hypot(observation.base_relative_roll_rad);
        let forward_m = (observation.base_x_m - start.0).hypot(observation.base_z_m - start.1);
        // The push registers hard but never crosses the termination tilt.
        assert!(
            max_tilt > 0.5 && max_tilt < 1.2,
            "walking peak lean should register without falling, got {max_tilt:.2}"
        );
        // The walk recovers upright and keeps covering ground.
        assert!(
            end_tilt < 0.15,
            "walking trot must end upright, got {end_tilt:.2}"
        );
        assert!(
            forward_m > 2.0,
            "walking trot must keep walking, got {forward_m:.2} m"
        );

        // Determinism: bit-identical repeat.
        let mut second =
            UnitreeGo2Episode::new(mid_walk_push_config(45)).expect("second walking episode");
        let mut second_observation = second.reset().observation;
        for _ in 0..900 {
            second_observation = second.step(walk_action(&second_observation)).observation;
        }
        assert_eq!(
            observation.base_x_m.to_bits(),
            second_observation.base_x_m.to_bits()
        );
        assert_eq!(
            observation.base_relative_pitch_rad.to_bits(),
            second_observation.base_relative_pitch_rad.to_bits()
        );
    }

    /// Windowed yaw of a 1440-step walk under joint targets produced by
    /// `targets_of(step)`, with the given motor force limit.
    fn windowed_yaw(
        max_force: f64,
        targets_of: impl Fn(u64) -> [crate::env::urdf_scene::UrdfJointPositionTarget<'static>; 12],
    ) -> (f64, f64, f64, f64) {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("load Go2 scene");
        settle(
            &mut sim,
            &UnitreeGo2EpisodeConfig {
                motor_max_force_n: max_force,
                ..Default::default()
            },
        );
        let start = sim.observe();
        let mut previous_yaw = start.base_relative_yaw_rad;
        let mut window_a = 0.0;
        let mut window_b = 0.0;
        let mut max_tilt = 0.0_f64;
        let mut min_height = f64::MAX;
        for step in 0..1440_u64 {
            sim.step_joint_position_targets(&targets_of(step));
            let observed = sim.observe();
            let mut delta = observed.base_relative_yaw_rad - previous_yaw;
            while delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            }
            while delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            if (480..960).contains(&step) {
                window_a += delta;
            } else if step >= 960 {
                window_b += delta;
            }
            previous_yaw = observed.base_relative_yaw_rad;
            max_tilt = max_tilt.max(
                observed
                    .base_relative_pitch_rad
                    .hypot(observed.base_relative_roll_rad),
            );
            min_height = min_height.min(observed.base_y_m);
        }
        (window_a, window_b, max_tilt, min_height)
    }

    fn fast_walk_command() -> UnitreeGo2GaitCommand {
        UnitreeGo2GaitCommand {
            stride_rad: 0.24,
            cycle_steps: 45,
            ..UnitreeGo2GaitCommand::default()
        }
    }

    #[test]
    fn yaw_plateau_is_not_torque_limited() {
        // Five times the actuator torque must not meaningfully change the
        // learned overlay's sustained yaw — the plateau is a contact/morphology
        // property, not an actuation-force limit.
        use super::super::unitree_go2_trot_targets_with_overlay;
        use super::super::UnitreeGo2GaitOverlay;

        let overlay_targets = |step: u64| {
            unitree_go2_trot_targets_with_overlay(
                step,
                fast_walk_command(),
                &UnitreeGo2GaitOverlay::LEARNED_TURN,
            )
        };
        let (nominal_a, nominal_b, _, _) = windowed_yaw(23.7, overlay_targets);
        let (strong_a, strong_b, _, _) = windowed_yaw(120.0, overlay_targets);
        let nominal = nominal_a.min(nominal_b);
        let strong = strong_a.min(strong_b);
        assert!(
            nominal > 0.12,
            "overlay must turn at nominal torque, got {nominal:+.3}"
        );
        assert!(
            strong < 1.5 * nominal,
            "yaw must not scale with torque: {strong:+.3} at 120 N*m vs {nominal:+.3} at 23.7"
        );
    }

    #[test]
    fn learned_schedule_turn_is_sustained_but_does_not_beat_the_overlay() {
        // The contact-schedule hypothesis, tested and refuted: the best
        // schedule the search found sustains a turn but stays below the
        // fixed-schedule overlay's rate. Pinning the comparison keeps the
        // negative result from silently rotting.
        use super::super::unitree_go2_scheduled_targets;
        use super::super::unitree_go2_trot_targets_with_overlay;
        use super::super::UnitreeGo2GaitOverlay;
        use super::super::UnitreeGo2GaitSchedule;

        let (schedule_a, schedule_b, max_tilt, min_height) = windowed_yaw(23.7, |step| {
            unitree_go2_scheduled_targets(
                step,
                fast_walk_command(),
                &UnitreeGo2GaitSchedule::LEARNED_TURN,
            )
        });
        assert!(
            schedule_a > 0.06 && schedule_b > 0.06,
            "schedule turn must be sustained: windows {schedule_a:+.3}/{schedule_b:+.3}"
        );
        assert!(
            max_tilt < 0.85 && min_height > 0.15,
            "schedule turn must stay upright: tilt {max_tilt:.2} height {min_height:.3}"
        );

        let (overlay_a, overlay_b, _, _) = windowed_yaw(23.7, |step| {
            unitree_go2_trot_targets_with_overlay(
                step,
                fast_walk_command(),
                &UnitreeGo2GaitOverlay::LEARNED_TURN,
            )
        });
        assert!(
            schedule_a.min(schedule_b) < overlay_a.min(overlay_b),
            "the schedule search did not beat the overlay in these measurements: {:+.3} vs {:+.3}",
            schedule_a.min(schedule_b),
            overlay_a.min(overlay_b)
        );

        // Determinism: bit-identical repeat.
        let (again_a, _, _, _) = windowed_yaw(23.7, |step| {
            unitree_go2_scheduled_targets(
                step,
                fast_walk_command(),
                &UnitreeGo2GaitSchedule::LEARNED_TURN,
            )
        });
        assert_eq!(schedule_a.to_bits(), again_a.to_bits());
    }

    #[test]
    fn learned_overlay_turns_the_walking_trot() {
        // The CEM-found overlay (examples/54_go2_learned_turn --train, seed 42)
        // must produce a *sustained* yaw rate — not the bounded elastic twist
        // every hand-scripted hip pattern produced. The gait winds up over the
        // first few seconds, so the assertions measure two disjoint late
        // windows: a saturating twist scores zero in both, a genuine turn keeps
        // rotating through each.
        use super::super::unitree_go2_trot_targets_with_overlay;
        use super::super::UnitreeGo2GaitOverlay;

        // Returns (yaw in steps 480..960, yaw in steps 960..1440, max tilt,
        // min height) over a 1440-step walk — the same protocol the search
        // scored, so the pinned coefficients are verified on their own terms.
        let run = |overlay: &UnitreeGo2GaitOverlay| {
            let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
                .expect("load Go2 scene");
            settle(&mut sim, &UnitreeGo2EpisodeConfig::default());
            let command = UnitreeGo2GaitCommand {
                stride_rad: 0.24,
                cycle_steps: 45,
                ..UnitreeGo2GaitCommand::default()
            };
            let start = sim.observe();
            let mut previous_yaw = start.base_relative_yaw_rad;
            let mut window_a = 0.0;
            let mut window_b = 0.0;
            let mut max_tilt = 0.0_f64;
            let mut min_height = f64::MAX;
            for step in 0..1440_u64 {
                sim.step_joint_position_targets(&unitree_go2_trot_targets_with_overlay(
                    step, command, overlay,
                ));
                let observed = sim.observe();
                let mut delta = observed.base_relative_yaw_rad - previous_yaw;
                while delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                }
                while delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                if (480..960).contains(&step) {
                    window_a += delta;
                } else if step >= 960 {
                    window_b += delta;
                }
                previous_yaw = observed.base_relative_yaw_rad;
                max_tilt = max_tilt.max(
                    observed
                        .base_relative_pitch_rad
                        .hypot(observed.base_relative_roll_rad),
                );
                min_height = min_height.min(observed.base_y_m);
            }
            (window_a, window_b, max_tilt, min_height)
        };

        let (window_a, window_b, max_tilt, min_height) = run(&UnitreeGo2GaitOverlay::LEARNED_TURN);
        // A sustained turn keeps rotating through both eight-second windows.
        assert!(
            window_a > 0.12 && window_b > 0.12,
            "learned gait must keep turning: windows {window_a:+.3} / {window_b:+.3}"
        );
        assert!(
            max_tilt < 0.8 && min_height > 0.15,
            "learned gait must stay upright: tilt {max_tilt:.2} height {min_height:.3}"
        );

        let (straight_a, straight_b, _, _) = run(&UnitreeGo2GaitOverlay::ZERO);
        assert!(
            (window_a + window_b).abs() > 4.0 * (straight_a + straight_b).abs(),
            "the overlay must out-turn the plain trot: {:+.3} vs {:+.3}",
            window_a + window_b,
            straight_a + straight_b
        );

        // Determinism: bit-identical repeat.
        let (again_a, again_b, _, _) = run(&UnitreeGo2GaitOverlay::LEARNED_TURN);
        assert_eq!(window_a.to_bits(), again_a.to_bits());
        assert_eq!(window_b.to_bits(), again_b.to_bits());
    }

    #[test]
    fn sustained_push_topples_weak_motor_trot_and_two_channel_feedback_saves_it() {
        // On torque-limited motors (8 N*m versus the stiff 23.7) a 1.8 rad flank
        // push spread over 20 steps beats the passive recovery rate. The honest
        // physics this test pins: the open-loop trot capsizes and ends flat on its
        // side, while two-channel feedback — hip abduction plus differential
        // leg-length extension, both driven from the measured lean — keeps the
        // peak lean under a third of the open-loop excursion and ends standing at
        // full height. Hip correction alone saturates and can only brace in a
        // deep propped lean; the leg-length channel is what makes it a save.
        let open_scored = run_trot(weak_motor_push_config(), open_loop);
        assert!(
            open_scored.fell && open_scored.steps_survived < 480,
            "open loop must fall: fell={} steps={}",
            open_scored.fell,
            open_scored.steps_survived
        );

        let open = run_to_horizon(weak_motor_push_config(), open_loop, 480);
        assert!(
            open.end_tilt_rad > 1.3,
            "open loop must end flat on its side, got tilt {:.2}",
            open.end_tilt_rad
        );

        let saved_scored = run_trot(weak_motor_push_config(), save_feedback());
        assert!(
            !saved_scored.fell && saved_scored.steps_survived == 480,
            "saved run must survive to truncation: fell={} steps={}",
            saved_scored.fell,
            saved_scored.steps_survived
        );
        let saved = run_to_horizon(weak_motor_push_config(), save_feedback(), 480);
        assert!(
            saved.max_tilt_rad < 0.5 * open.max_tilt_rad,
            "feedback must at least halve the peak lean: {:.2} vs {:.2}",
            saved.max_tilt_rad,
            open.max_tilt_rad
        );
        assert!(
            saved.end_tilt_rad < 0.55 && saved.end_height_m > 0.2,
            "saved run must end standing: tilt {:.2} height {:.3}",
            saved.end_tilt_rad,
            saved.end_height_m
        );

        // The inverted sign must not save the robot — pins the feedback pairing.
        let mut inner = two_channel_feedback(2.5, 5.0);
        let inverted = run_trot(weak_motor_push_config(), move |observation| {
            let mut action = inner(observation);
            action.roll_correction_rad = -action.roll_correction_rad;
            action.lateral_extension_rad = -action.lateral_extension_rad;
            action
        });
        assert!(inverted.fell, "inverted feedback must not save the robot");

        // Determinism: the saved run reproduces bit-identically.
        let again = run_to_horizon(weak_motor_push_config(), save_feedback(), 480);
        assert_eq!(saved.max_tilt_rad.to_bits(), again.max_tilt_rad.to_bits());
        assert_eq!(saved.end_tilt_rad.to_bits(), again.end_tilt_rad.to_bits());
    }

    #[test]
    fn pushed_trot_registers_recovers_and_feedback_reduces_peak_lean() {
        // A 0.4 rad lean is a hard shove; the scripted trot's stiff position motors
        // and wide stance make it passively stable against instantaneous tilts, so
        // the assertions pin the quantitative story rather than a staged fall: the
        // disturbance registers fully, the robot recovers, correctly signed posture
        // feedback strictly reduces the peak lean, and the wrong sign increases it.
        let tilt = 0.40;
        let open = run_trot(stiff_push_config(tilt), open_loop);
        let corrected = run_trot(stiff_push_config(tilt), posture_feedback_with(1.2));
        let inverted = run_trot(stiff_push_config(tilt), posture_feedback_with(-1.2));

        // The tilt lands in full: peak lean reaches at least the applied angle.
        assert!(
            open.max_tilt_rad >= tilt,
            "push did not register: {:.2}",
            open.max_tilt_rad
        );
        // Passive stability: the open-loop trot survives to truncation.
        assert!(!open.fell && open.steps_survived == 420);
        // Correct feedback strictly improves the peak; inverted strictly worsens it.
        assert!(
            corrected.max_tilt_rad < open.max_tilt_rad,
            "feedback must reduce peak lean: {:.3} vs {:.3}",
            corrected.max_tilt_rad,
            open.max_tilt_rad
        );
        assert!(
            inverted.max_tilt_rad > open.max_tilt_rad,
            "inverted feedback must worsen peak lean: {:.3} vs {:.3}",
            inverted.max_tilt_rad,
            open.max_tilt_rad
        );
        // Determinism: the same run reproduces bit-identically.
        let again = run_trot(stiff_push_config(tilt), posture_feedback_with(1.2));
        assert_eq!(
            corrected.max_tilt_rad.to_bits(),
            again.max_tilt_rad.to_bits()
        );
    }

    fn stand_targets() -> [crate::env::urdf_scene::UrdfJointPositionTarget<'static>; 12] {
        unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                ..UnitreeGo2GaitCommand::default()
            },
        )
    }

    /// Official Go2 actuator speed ceiling in rad/s.
    const GO2_JOINT_SPEED_LIMIT_RAD_S: f64 = 30.1;

    /// A Go2 settled into a static stand on the default stiff position motors.
    fn settled_stand() -> UrdfSceneSim {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("load Go2 scene");
        settle(&mut sim, &UnitreeGo2EpisodeConfig::default());
        let stand = stand_targets();
        for _ in 0..120 {
            sim.step_joint_position_targets(&stand);
        }
        sim
    }

    #[test]
    fn joint_state_readback_matches_the_position_convention() {
        // The reduced-coordinate readback must agree with the position-target
        // convention on every joint of the settled stand — this is what makes
        // closed-loop torque control possible at all.
        let sim = settled_stand();
        for target in stand_targets() {
            let position = sim
                .named_joint_position(target.link_name)
                .expect("multibody joint position");
            assert!(
                (position - target.position).abs() < 0.15,
                "{}: read {position:+.3} vs held target {:+.3}",
                target.link_name,
                target.position
            );
            let velocity = sim
                .named_joint_velocity(target.link_name)
                .expect("multibody joint velocity");
            assert!(
                velocity.abs() < 0.5,
                "{}: settled joint should be near rest, velocity {velocity:+.3}",
                target.link_name
            );
        }
        assert_eq!(sim.named_joint_position("no_such_link"), None);
        // The multibody root is not a single-DoF joint.
        assert_eq!(sim.named_joint_position("base"), None);
    }

    #[test]
    fn feed_forward_torque_moves_a_calf_with_the_commanded_sign() {
        use super::super::UrdfJointTorqueTarget;

        // Torque one calf while the other eleven joints stay position-held
        // (unlisted joints keep their motor configuration). Eight newton-meters
        // clears the stance gravity load in either direction, so the joint must
        // move with the commanded sign.
        let probe = |torque_nm: f64| {
            let mut sim = settled_stand();
            let start = sim.named_joint_position("FL_calf").expect("calf position");
            for _ in 0..40 {
                sim.step_joint_torques(&[UrdfJointTorqueTarget {
                    link_name: "FL_calf",
                    torque_nm,
                    max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
                }]);
            }
            sim.named_joint_position("FL_calf").expect("calf position") - start
        };
        let extended = probe(8.0);
        let flexed = probe(-8.0);
        println!("calf delta at +8 N*m: {extended:+.3}, at -8 N*m: {flexed:+.3}");
        assert!(
            extended > 0.03,
            "positive torque must drive the calf positive, got {extended:+.3}"
        );
        assert!(
            flexed < -0.03,
            "negative torque must drive the calf negative, got {flexed:+.3}"
        );
    }

    #[test]
    fn zero_torque_frees_every_joint_and_the_stand_collapses() {
        use super::super::UrdfJointTorqueTarget;

        // Zero torque on all twelve joints must leave them genuinely free — the
        // robot collapses under gravity, proving torque mode really turns the
        // position servos off instead of falling back to a default motor.
        let mut sim = settled_stand();
        let standing_height = sim.observe().base_y_m;
        assert!(
            standing_height > 0.2,
            "stand must start at height, got {standing_height:.3}"
        );
        let zero: Vec<UrdfJointTorqueTarget<'_>> = stand_targets()
            .iter()
            .map(|target| UrdfJointTorqueTarget {
                link_name: target.link_name,
                torque_nm: 0.0,
                max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
            })
            .collect();
        for _ in 0..180 {
            sim.step_joint_torques(&zero);
        }
        let height = sim.observe().base_y_m;
        assert!(
            height < 0.15,
            "unpowered joints must collapse the stand, base height {height:.3}"
        );
    }

    #[test]
    fn torque_pd_holds_the_stand_and_replays_exactly() {
        use super::super::UrdfJointTorqueTarget;

        // Closed-loop control entirely in torque space: a joint-level PD
        // computes feed-forward torques from the readback, clamped to the real
        // 23.7 N*m actuator limit. This is the actuation pathway the measured
        // steering plateau points at — the servo constraint is gone, and the
        // contact forces are shaped by commanded torques alone.
        // Returns (min height, end height, end tilt, max tilt) of a 360-step
        // torque-PD stand. Tilt is yaw-invariant: the angle between the body
        // direction that points up at the settled stand and world up (the Euler
        // observations wrap between branches when the body yaws).
        let run = |kp: f64, kd: f64| {
            let mut sim = settled_stand();
            let up_body = {
                let pose = sim.named_transform("base").expect("base pose");
                (pose.rotation.inverse() * rne_math::Vec3::Y).normalize_or_zero()
            };
            let true_tilt = |sim: &UrdfSceneSim| {
                let pose = sim.named_transform("base").expect("base pose");
                let up = (pose.rotation * up_body).normalize_or_zero();
                up.y.clamp(-1.0, 1.0).acos()
            };
            let stand = stand_targets();
            let mut min_height = f64::MAX;
            let mut max_tilt = 0.0_f64;
            for _ in 0..360 {
                let torques: Vec<UrdfJointTorqueTarget<'_>> = stand
                    .iter()
                    .map(|target| {
                        let q = sim
                            .named_joint_position(target.link_name)
                            .expect("joint position");
                        let qd = sim
                            .named_joint_velocity(target.link_name)
                            .expect("joint velocity");
                        UrdfJointTorqueTarget {
                            link_name: target.link_name,
                            torque_nm: (kp * (target.position - q) - kd * qd).clamp(-23.7, 23.7),
                            max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
                        }
                    })
                    .collect();
                sim.step_joint_torques(&torques);
                min_height = min_height.min(sim.observe().base_y_m);
                max_tilt = max_tilt.max(true_tilt(&sim));
            }
            let end_height = sim.observe().base_y_m;
            (min_height, end_height, true_tilt(&sim), max_tilt)
        };

        let (min_height, end_height, end_tilt, max_tilt) = run(25.0, 0.5);
        println!(
            "torque PD stand kp=25 kd=0.5: min_height {min_height:.3} end_height {end_height:.3} end_tilt {end_tilt:.3} max_tilt {max_tilt:.3}"
        );
        assert!(
            min_height > 0.18 && end_height > 0.2,
            "torque PD must keep the robot standing: min {min_height:.3} end {end_height:.3}"
        );
        assert!(
            max_tilt < 0.1,
            "torque PD stand must stay level throughout, max tilt {max_tilt:.3}"
        );

        // The stability boundary is the *velocity* feedback: at 60 Hz an
        // explicit kd larger than roughly 2I/dt for the light distal links
        // turns damping into excitation. Doubling kd must destabilize the same
        // stand — the discrete-rate lesson pinned so it cannot rot.
        let (_, _, _, overdamped_max_tilt) = run(25.0, 1.0);
        println!("overdamped kd=1 max tilt: {overdamped_max_tilt:.3}");
        assert!(
            overdamped_max_tilt > 3.0 * max_tilt,
            "explicit damping past the discrete stability bound must destabilize: {overdamped_max_tilt:.3} vs {max_tilt:.3}"
        );

        // Determinism: bit-identical repeat.
        let (again_min, again_end, _, _) = run(25.0, 0.5);
        assert_eq!(min_height.to_bits(), again_min.to_bits());
        assert_eq!(end_height.to_bits(), again_end.to_bits());
    }

    #[test]
    fn torque_pd_tracks_the_walking_trot() {
        use super::super::UrdfJointTorqueTarget;

        // The low-bandwidth torque PD (kd pinned at 0.5 by the 60 Hz discrete
        // stability bound) genuinely tracks the cycle-45 walking trot: kp 40
        // walks at position-servo speed while staying up, and pushing kp to 80
        // crosses the same discrete stability boundary the stand test pinned —
        // the gait thrashes and falls.
        let run = |kp: f64, kd: f64| {
            let mut sim = settled_stand();
            let up_body = {
                let pose = sim.named_transform("base").expect("base pose");
                (pose.rotation.inverse() * rne_math::Vec3::Y).normalize_or_zero()
            };
            let true_tilt = |sim: &UrdfSceneSim| {
                let pose = sim.named_transform("base").expect("base pose");
                let up = (pose.rotation * up_body).normalize_or_zero();
                up.y.clamp(-1.0, 1.0).acos()
            };
            let start = sim.observe();
            let mut min_height = f64::MAX;
            let mut max_tilt = 0.0_f64;
            for step in 0..720_u64 {
                let targets = unitree_go2_trot_targets(step, fast_walk_command());
                let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
                    .iter()
                    .map(|target| {
                        let q = sim
                            .named_joint_position(target.link_name)
                            .expect("joint position");
                        let qd = sim
                            .named_joint_velocity(target.link_name)
                            .expect("joint velocity");
                        UrdfJointTorqueTarget {
                            link_name: target.link_name,
                            torque_nm: (kp * (target.position - q) - kd * qd).clamp(-23.7, 23.7),
                            max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
                        }
                    })
                    .collect();
                sim.step_joint_torques(&torques);
                min_height = min_height.min(sim.observe().base_y_m);
                max_tilt = max_tilt.max(true_tilt(&sim));
            }
            let end = sim.observe();
            let distance = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
            (distance, min_height, max_tilt)
        };
        let (distance, min_height, max_tilt) = run(40.0, 0.5);
        println!(
            "torque PD walk kp=40 kd=0.5: distance {distance:.3} m min_height {min_height:.3} max_tilt {max_tilt:.3}"
        );
        assert!(
            distance > 1.4,
            "torque-PD walk must cover ground, got {distance:.2} m"
        );
        assert!(
            min_height > 0.15 && max_tilt < 0.8,
            "torque-PD walk must stay up: height {min_height:.3} tilt {max_tilt:.3}"
        );

        // The discrete stability boundary applies to the gait exactly as it did
        // to the stand: doubling kp past it makes the walk thrash and fall.
        let (_, _, unstable_tilt) = run(80.0, 0.5);
        println!("torque PD walk kp=80 kd=0.5: max_tilt {unstable_tilt:.3}");
        assert!(
            unstable_tilt > 1.2,
            "kp past the discrete bound must destabilize the walk, tilt {unstable_tilt:.3}"
        );

        // Determinism: bit-identical repeat.
        let (again_distance, again_height, _) = run(40.0, 0.5);
        assert_eq!(distance.to_bits(), again_distance.to_bits());
        assert_eq!(min_height.to_bits(), again_height.to_bits());
    }

    /// Windowed-yaw run of the torque-PD walking trot (kp 40, kd 0.5) with
    /// contact-gated feed-forward torques added on stance legs: `twist_hip` with
    /// diagonal-pair signs on the hips, `drive_diff` with left/right signs on
    /// the thighs. When `yaw_rate_gain` is non-zero the drive term is instead a
    /// feedback law: `drive = clamp(yaw_rate_gain * (0.3 - yaw_rate), 8)`.
    /// Returns (window_a, window_b, max_tilt, min_height, distance).
    fn torque_steer_run_with_feedback(
        twist_hip: f64,
        drive_diff: f64,
        yaw_rate_gain: f64,
    ) -> (f64, f64, f64, f64, f64) {
        use super::super::UrdfJointTorqueTarget;

        const KP: f64 = 40.0;
        const KD: f64 = 0.5;
        let mut sim = settled_stand();
        let up_body = {
            let pose = sim.named_transform("base").expect("base pose");
            (pose.rotation.inverse() * rne_math::Vec3::Y).normalize_or_zero()
        };
        let true_tilt = |sim: &UrdfSceneSim| {
            let pose = sim.named_transform("base").expect("base pose");
            let up = (pose.rotation * up_body).normalize_or_zero();
            up.y.clamp(-1.0, 1.0).acos()
        };
        let start = sim.observe();
        let mut previous_yaw = start.base_relative_yaw_rad;
        let mut window_a = 0.0;
        let mut window_b = 0.0;
        let mut max_tilt = 0.0_f64;
        let mut min_height = f64::MAX;
        for step in 0..1440_u64 {
            let targets = unitree_go2_trot_targets(step, fast_walk_command());
            let stance = [
                sim.link_contact_impulse_ns("FL_foot") > 0.0,
                sim.link_contact_impulse_ns("FR_foot") > 0.0,
                sim.link_contact_impulse_ns("RL_foot") > 0.0,
                sim.link_contact_impulse_ns("RR_foot") > 0.0,
            ];
            let drive_diff = if yaw_rate_gain != 0.0 {
                let yaw_rate = sim.observe().base_angular_velocity_y_rad_s;
                (yaw_rate_gain * (0.3 - yaw_rate)).clamp(-8.0, 8.0)
            } else {
                drive_diff
            };
            // Legs ordered FL, FR, RL, RR; joints hip, thigh, calf within each.
            let diag_sign = [1.0, -1.0, -1.0, 1.0];
            let side_sign = [1.0, -1.0, 1.0, -1.0];
            let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
                .iter()
                .enumerate()
                .map(|(index, target)| {
                    let q = sim
                        .named_joint_position(target.link_name)
                        .expect("joint position");
                    let qd = sim
                        .named_joint_velocity(target.link_name)
                        .expect("joint velocity");
                    let mut torque = KP * (target.position - q) - KD * qd;
                    let leg = index / 3;
                    if stance[leg] {
                        match index % 3 {
                            0 => torque += twist_hip * diag_sign[leg],
                            1 => torque += drive_diff * side_sign[leg],
                            _ => {}
                        }
                    }
                    UrdfJointTorqueTarget {
                        link_name: target.link_name,
                        torque_nm: torque.clamp(-23.7, 23.7),
                        max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
                    }
                })
                .collect();
            sim.step_joint_torques(&torques);
            let observed = sim.observe();
            let mut delta = observed.base_relative_yaw_rad - previous_yaw;
            while delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            }
            while delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            if (480..960).contains(&step) {
                window_a += delta;
            } else if step >= 960 {
                window_b += delta;
            }
            previous_yaw = observed.base_relative_yaw_rad;
            max_tilt = max_tilt.max(true_tilt(&sim));
            min_height = min_height.min(observed.base_y_m);
        }
        let end = sim.observe();
        let distance = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
        (window_a, window_b, max_tilt, min_height, distance)
    }

    #[test]
    fn learned_torque_overlay_out_turns_the_position_plateau() {
        use super::super::{UnitreeGo2TorqueOverlay, UrdfJointTorqueTarget};

        // The measurement the torque arc exists for. Three position-space
        // searches and a five-fold torque scan plateaued at ~0.02 rad/s and
        // localized the steering boundary in the contact mechanics under hard
        // position servos. The CEM-found contact-gated torque overlay
        // (examples/56_go2_torque_turn --train, seed 42) sustains ~0.034 rad/s
        // — above every position-space result — while the robot keeps walking:
        // force-level coordination genuinely moves the boundary joint-space
        // control could not.
        let run = |overlay: &UnitreeGo2TorqueOverlay| {
            let mut sim = settled_stand();
            let up_body = {
                let pose = sim.named_transform("base").expect("base pose");
                (pose.rotation.inverse() * rne_math::Vec3::Y).normalize_or_zero()
            };
            let true_tilt = |sim: &UrdfSceneSim| {
                let pose = sim.named_transform("base").expect("base pose");
                let up = (pose.rotation * up_body).normalize_or_zero();
                up.y.clamp(-1.0, 1.0).acos()
            };
            let start = sim.observe();
            let mut previous_yaw = start.base_relative_yaw_rad;
            let mut window_a = 0.0;
            let mut window_b = 0.0;
            let mut max_tilt = 0.0_f64;
            let mut min_height = f64::MAX;
            let cycle = fast_walk_command().cycle_steps;
            for step in 0..1440_u64 {
                let targets = unitree_go2_trot_targets(step, fast_walk_command());
                let stance = [
                    sim.link_contact_impulse_ns("FL_foot") > 0.0,
                    sim.link_contact_impulse_ns("FR_foot") > 0.0,
                    sim.link_contact_impulse_ns("RL_foot") > 0.0,
                    sim.link_contact_impulse_ns("RR_foot") > 0.0,
                ];
                let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
                let feed_forward = overlay.torques_nm(two_cycle_phase, stance);
                let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
                    .iter()
                    .zip(feed_forward.iter())
                    .map(|(target, extra)| {
                        let q = sim
                            .named_joint_position(target.link_name)
                            .expect("joint position");
                        let qd = sim
                            .named_joint_velocity(target.link_name)
                            .expect("joint velocity");
                        UrdfJointTorqueTarget {
                            link_name: target.link_name,
                            torque_nm: (40.0 * (target.position - q) - 0.5 * qd + extra)
                                .clamp(-23.7, 23.7),
                            max_velocity_rad_s: GO2_JOINT_SPEED_LIMIT_RAD_S,
                        }
                    })
                    .collect();
                sim.step_joint_torques(&torques);
                let observed = sim.observe();
                let mut delta = observed.base_relative_yaw_rad - previous_yaw;
                while delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                }
                while delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                if (480..960).contains(&step) {
                    window_a += delta;
                } else if step >= 960 {
                    window_b += delta;
                }
                previous_yaw = observed.base_relative_yaw_rad;
                max_tilt = max_tilt.max(true_tilt(&sim));
                min_height = min_height.min(observed.base_y_m);
            }
            (window_a, window_b, max_tilt, min_height)
        };

        let (window_a, window_b, max_tilt, min_height) =
            run(&UnitreeGo2TorqueOverlay::LEARNED_TURN);
        println!(
            "learned torque turn: windows {window_a:+.3}/{window_b:+.3} tilt {max_tilt:.3} height {min_height:.3}"
        );
        // The robust claim is window A: +0.304 rad per 8 s (0.038 rad/s, 52 %
        // above the position plateau's like-for-like window) is stable across
        // coefficient perturbations AND across platforms. A bounded elastic
        // twist saturates before this window opens at step 480, so a positive
        // window A is already twist-proof. Window B lies beyond the compliant
        // walk's ~16 s chaos horizon: single-ulp libm differences swing it by
        // ±0.3 rad (Linux CI measured -0.07 where Windows measures +0.27), so
        // it is only pinned against catastrophic reversal.
        assert!(
            window_a > 0.25,
            "torque turn must out-turn the position plateau in the robust window: {window_a:+.3}"
        );
        assert!(
            window_b > -0.35,
            "beyond the chaos horizon the turn must not reverse catastrophically: {window_b:+.3}"
        );
        assert!(
            max_tilt < 0.8 && min_height > 0.1,
            "torque turn must stay walking upright: tilt {max_tilt:.3} height {min_height:.3}"
        );

        // Determinism: bit-identical repeat.
        let (again_a, again_b, _, _) = run(&UnitreeGo2TorqueOverlay::LEARNED_TURN);
        assert_eq!(window_a.to_bits(), again_a.to_bits());
        assert_eq!(window_b.to_bits(), again_b.to_bits());
    }

    #[test]
    fn contact_gated_hand_torques_do_not_steer_the_torque_walk() {
        // Force-level steering, hand-designed, measured and refuted — the
        // torque-space mirror of the six position-space steering nulls. Three
        // mechanisms position control cannot express at all: contact-gated
        // diagonal hip twist torque, contact-gated left/right differential
        // stance thrust, and yaw-rate feedback through the thrust channel.
        // None sustains a turn through both measurement windows in either
        // direction (the overlay's honest 0.12 rad per window is the bar), and
        // the feed-forward thrust asymmetry stalls forward progress instead of
        // steering — the gait's own propulsion cycle absorbs it.
        let configs: [(&str, f64, f64, f64); 4] = [
            ("baseline", 0.0, 0.0, 0.0),
            ("diag hip twist 4", 4.0, 0.0, 0.0),
            ("diff thrust 4", 0.0, 4.0, 0.0),
            ("yaw-rate feedback 25", 0.0, 0.0, 25.0),
        ];
        for (label, twist, drive, gain) in configs {
            let (a, b, tilt, height, distance) = torque_steer_run_with_feedback(twist, drive, gain);
            println!(
                "steer [{label}]: windows {a:+.3}/{b:+.3} tilt {tilt:.3} height {height:.3} dist {distance:.2}"
            );
            assert!(
                a.min(b) < 0.12 && (-a).min(-b) < 0.12,
                "[{label}] must not sustain a turn either way: {a:+.3}/{b:+.3}"
            );
            assert!(
                tilt < 0.85 && height > 0.1,
                "[{label}] walk must survive the pattern: tilt {tilt:.3} height {height:.3}"
            );
            if label == "baseline" {
                assert!(
                    distance > 3.0,
                    "baseline torque walk must keep walking, got {distance:.2} m"
                );
            }
        }
    }
}
