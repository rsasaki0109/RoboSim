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
    /// Maximum relative pitch/roll magnitude before termination in radians.
    pub max_tilt_rad: f64,
}

impl Default for UnitreeG1GaitEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            max_steps: 600,
            cycle_steps: 120,
            max_tilt_rad: 1.2,
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
    /// Pelvis pitch in radians, including the URDF-to-world basis rotation.
    pub base_pitch_rad: f64,
    /// Pelvis roll in radians, including the URDF-to-world basis rotation.
    pub base_roll_rad: f64,
    /// Pelvis linear velocity in meters per second.
    pub base_linear_velocity_m_s: [f64; 3],
    /// Pelvis angular velocity in radians per second.
    pub base_angular_velocity_rad_s: [f64; 3],
    /// Pelvis yaw relative to the loaded upright pose in radians.
    pub base_relative_yaw_rad: f64,
    /// Pelvis pitch relative to the loaded upright pose in radians.
    pub base_relative_pitch_rad: f64,
    /// Pelvis roll relative to the loaded upright pose in radians.
    pub base_relative_roll_rad: f64,
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
        let tilt_rad = observation
            .base_relative_pitch_rad
            .hypot(observation.base_relative_roll_rad);
        let fallen = observation.base_y_m < FALLEN_HEIGHT_M || tilt_rad > self.config.max_tilt_rad;
        let height_error_m = (observation.base_y_m - NOMINAL_HEIGHT_M).abs();
        let reward = 0.5 + 100.0 * forward_delta_m
            - 2.0 * height_error_m
            - 0.1 * observation.base_relative_yaw_rad.abs()
            - 0.2 * tilt_rad
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
    use super::super::{UnitreeG1GaitCommand, UrdfJointTorqueTarget};
    use super::*;

    /// Conservative actuator speed ceiling for torque mode.
    const G1_SPEED_LIMIT_RAD_S: f64 = 30.0;

    fn g1_stand_targets() -> [UrdfJointPositionTarget<'static>; 23] {
        unitree_g1_gait_targets(
            0,
            UnitreeG1GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 120,
            },
        )
    }

    /// A G1 settled into its stand on the episode's stiff position motors.
    fn g1_settled_stand() -> UrdfSceneSim {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("load dynamic G1");
        settle(&mut sim);
        let stand = g1_stand_targets();
        for _ in 0..120 {
            sim.step_joint_position_targets(&stand);
        }
        sim
    }

    #[test]
    fn g1_joint_state_readback_matches_the_position_convention() {
        // The Go2 torque pathway, ported: the reduced-coordinate readback
        // must agree with the position-target convention on the settled
        // 23-DoF humanoid stand.
        let sim = g1_settled_stand();
        for target in g1_stand_targets() {
            let position = sim
                .named_joint_position(target.link_name)
                .expect("multibody joint position");
            assert!(
                (position - target.position).abs() < 0.2,
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
        assert_eq!(sim.named_joint_position("pelvis"), None);
    }

    /// Proximal torque joints of the hybrid architecture: hips and knees.
    /// The ankles' small inertia puts their discrete 60 Hz damping bound
    /// near zero, so they (and the light arms) stay position-held.
    const G1_TORQUE_LINKS: [&str; 8] = [
        "left_knee_link",
        "right_knee_link",
        "left_hip_pitch_link",
        "right_hip_pitch_link",
        "left_hip_roll_link",
        "right_hip_roll_link",
        "left_hip_yaw_link",
        "right_hip_yaw_link",
    ];

    /// Torque-PD stand on the proximal joints; returns (min height, finite).
    fn g1_torque_stand(kp: f64, kd: f64, steps: u64) -> (f64, bool) {
        let mut sim = g1_settled_stand();
        let stand: Vec<_> = g1_stand_targets()
            .into_iter()
            .filter(|target| G1_TORQUE_LINKS.contains(&target.link_name))
            .collect();
        let mut min_height = f64::MAX;
        for _ in 0..steps {
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
                        torque_nm: (kp * (target.position - q) - kd * qd).clamp(-88.0, 88.0),
                        max_velocity_rad_s: G1_SPEED_LIMIT_RAD_S,
                    }
                })
                .collect();
            if torques.iter().any(|torque| !torque.torque_nm.is_finite()) {
                return (min_height, false);
            }
            sim.step_joint_torques(&torques);
            let height = sim.observe().base_y_m;
            if !height.is_finite() {
                return (min_height, false);
            }
            min_height = min_height.min(height);
        }
        (min_height, true)
    }

    #[test]
    fn g1_feed_forward_torque_moves_a_knee_with_the_commanded_sign() {
        // The Go2 sign probe, ported: torque one knee both ways with every
        // other joint position-held, and it must move with the commanded
        // sign - the pathway is sound joint by joint on the humanoid too.
        let probe = |torque_nm: f64| {
            let mut sim = g1_settled_stand();
            let start = sim
                .named_joint_position("left_knee_link")
                .expect("knee position");
            for _ in 0..40 {
                sim.step_joint_torques(&[UrdfJointTorqueTarget {
                    link_name: "left_knee_link",
                    torque_nm,
                    max_velocity_rad_s: G1_SPEED_LIMIT_RAD_S,
                }]);
            }
            sim.named_joint_position("left_knee_link")
                .expect("knee position")
                - start
        };
        let extended = probe(20.0);
        let flexed = probe(-20.0);
        println!("knee delta at +20 N*m: {extended:+.3}, at -20 N*m: {flexed:+.3}");
        assert!(
            extended > 0.2,
            "positive torque must drive the knee positive, got {extended:+.3}"
        );
        assert!(
            flexed < -0.2,
            "negative torque must drive the knee negative, got {flexed:+.3}"
        );
    }

    #[test]
    fn g1_torque_pd_stand_needs_humanoid_gains() {
        // The gain lesson the explosions taught: the humanoid hip carries an
        // order of magnitude more gravity torque than the quadruped's, so
        // the Go2-scale gains that walk the dog FOLD the humanoid (sag ->
        // fall -> ground chatter -> solver blow-up), while hip-scale gains
        // stand it quietly. The hips' large driven inertia is what makes the
        // higher damping discretely stable at the same 60 Hz.
        let (folded_height, _) = g1_torque_stand(60.0, 2.0, 360);
        println!("kp60 kd2: minH {folded_height:.3}");
        assert!(
            folded_height < 0.5,
            "quadruped-scale gains must fold the humanoid, minH {folded_height:.3}"
        );

        let (standing_height, finite) = g1_torque_stand(300.0, 10.0, 360);
        println!("kp300 kd10: minH {standing_height:.3}");
        assert!(
            finite && standing_height > 0.7,
            "humanoid-scale gains must hold the stand, minH {standing_height:.3}"
        );

        // Determinism: bit-identical repeat.
        let (again_height, _) = g1_torque_stand(300.0, 10.0, 360);
        assert_eq!(standing_height.to_bits(), again_height.to_bits());
    }

    /// Hybrid rollout with a torque overlay, measuring straight-line window
    /// displacements — the transport metric of the learned-stride search.
    fn g1_hybrid_transport(overlay: &super::super::UnitreeG1TorqueOverlay) -> (f64, f64, f64, f64) {
        // Exact mirror of example 62's settle: the learned constants live on
        // the trajectory that protocol produces.
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("load dynamic G1");
        sim.configure_position_motors(220.0, 24.0, 88.0);
        let stand = unitree_g1_gait_targets(
            0,
            UnitreeG1GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: 120,
            },
        );
        for _ in 0..240 {
            sim.step_joint_position_targets(&stand);
        }
        let command = UnitreeG1GaitCommand {
            stride_rad: 0.05,
            foot_lift_rad: 0.08,
            cycle_steps: 120,
        };
        let start = sim.observe();
        let mut window_start = [start.base_x_m, start.base_z_m];
        let mut window_a = 0.0;
        let mut window_b = 0.0;
        let mut min_height = f64::MAX;
        let mut previous_yaw = start.base_relative_yaw_rad;
        let mut total_yaw = 0.0;
        for step in 0..1440_u64 {
            let targets = unitree_g1_gait_targets(step, command);
            let servo: Vec<_> = targets
                .iter()
                .filter(|target| !G1_TORQUE_LINKS.contains(&target.link_name))
                .copied()
                .collect();
            sim.set_joint_position_targets(&servo);
            let stance = [
                sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0,
                sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0,
            ];
            let two_cycle_phase = (step % 240) as f64 / 240.0;
            // The overlay's joint order: left hip pitch/roll/yaw, left knee,
            // then the right leg — matched here explicitly.
            const OVERLAY_ORDER: [&str; 8] = [
                "left_hip_pitch_link",
                "left_hip_roll_link",
                "left_hip_yaw_link",
                "left_knee_link",
                "right_hip_pitch_link",
                "right_hip_roll_link",
                "right_hip_yaw_link",
                "right_knee_link",
            ];
            let feed_forward = overlay.torques_nm(two_cycle_phase, stance);
            let torques: Vec<UrdfJointTorqueTarget<'_>> = OVERLAY_ORDER
                .iter()
                .enumerate()
                .map(|(index, link_name)| {
                    let target_position = targets
                        .iter()
                        .find(|target| target.link_name == *link_name)
                        .expect("torque link in gait targets")
                        .position;
                    let q = sim.named_joint_position(link_name).expect("joint position");
                    let qd = sim.named_joint_velocity(link_name).expect("joint velocity");
                    UrdfJointTorqueTarget {
                        link_name,
                        torque_nm: (300.0 * (target_position - q) - 10.0 * qd
                            + feed_forward[index])
                            .clamp(-88.0, 88.0),
                        max_velocity_rad_s: G1_SPEED_LIMIT_RAD_S,
                    }
                })
                .collect();
            sim.step_joint_torques(&torques);
            let observed = sim.observe();
            let mut yaw_delta = observed.base_relative_yaw_rad - previous_yaw;
            while yaw_delta > std::f64::consts::PI {
                yaw_delta -= 2.0 * std::f64::consts::PI;
            }
            while yaw_delta < -std::f64::consts::PI {
                yaw_delta += 2.0 * std::f64::consts::PI;
            }
            total_yaw += yaw_delta;
            previous_yaw = observed.base_relative_yaw_rad;
            if step + 1 == 480 {
                window_start = [observed.base_x_m, observed.base_z_m];
            } else if step + 1 == 960 {
                window_a = (observed.base_x_m - window_start[0])
                    .hypot(observed.base_z_m - window_start[1]);
                window_start = [observed.base_x_m, observed.base_z_m];
            } else if step + 1 == 1440 {
                window_b = (observed.base_x_m - window_start[0])
                    .hypot(observed.base_z_m - window_start[1]);
            }
            min_height = min_height.min(observed.base_y_m);
        }
        (window_a, window_b, min_height, total_yaw)
    }

    #[test]
    fn learned_torques_make_the_g1_stride() {
        use super::super::UnitreeG1TorqueOverlay;

        // The humanoid's first real steps, pinned at their cross-platform
        // bar. The scripted G1 gait is a near-stationary stepper, so
        // transport had to be CREATED by learned stance torques. On the
        // The unscaled training-platform winner walked 0.26 m per window;
        // the cross-platform 60% overlay walks about 0.19 m (over 2x the
        // stepper). On other platforms the ulp-shifted orbit can degrade —
        // and a degraded humanoid orbit does not merely score less, it can
        // blow the solver up mid-step. So the test applies the search's own
        // discipline: each replay runs under catch_unwind (a panic is a
        // fall), and the pinned claim is the MEDIAN of three ulp-perturbed
        // replays.
        let run = |perturbation: f64| -> Option<(f64, f64, f64, f64)> {
            std::panic::catch_unwind(|| {
                let mut overlay = UnitreeG1TorqueOverlay::LEARNED_STRIDE;
                overlay.coefficients[0][0] += perturbation;
                g1_hybrid_transport(&overlay)
            })
            .ok()
        };
        let (base_a, base_b, _, _) = g1_hybrid_transport(&UnitreeG1TorqueOverlay::ZERO);
        let members: Vec<Option<(f64, f64, f64, f64)>> =
            [0.0, 1.0e-9, 3.0e-9].iter().map(|p| run(*p)).collect();
        for (index, member) in members.iter().enumerate() {
            match member {
                Some((a, b, height, yaw)) => {
                    println!("member {index}: {a:.2}/{b:.2} m minH {height:.3} yaw {yaw:+.2}")
                }
                None => println!("member {index}: SOLVER PANIC (scored as fall)"),
            }
        }
        println!("stepper baseline: {base_a:.2}/{base_b:.2} m");

        // Median by min-window transport; a panicked member scores zero.
        let mut min_windows: Vec<f64> = members
            .iter()
            .map(|member| member.map_or(0.0, |(a, b, _, _)| a.min(b)))
            .collect();
        min_windows.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        let median = min_windows[1];
        let baseline_min = base_a.min(base_b);
        assert!(
            median > 2.0 * baseline_min && median > 0.15,
            "the median replay must stride: {median:.2} m vs stepper {baseline_min:.2} m"
        );

        // The median member (or better) must also be upright and straight.
        let best_valid = members
            .iter()
            .flatten()
            .find(|(a, b, _, _)| a.min(*b) >= median - 1.0e-9)
            .expect("a striding member exists");
        assert!(
            best_valid.2 > 0.7,
            "the striding member must stay at height, minH {:.3}",
            best_valid.2
        );
        assert!(
            best_valid.3.abs() < 0.3,
            "the striding member must stay straight, yaw {:+.2}",
            best_valid.3
        );

        // Determinism: bit-identical repeat of the unperturbed member.
        let first = run(0.0);
        let second = run(0.0);
        match (first, second) {
            (Some(a), Some(b)) => {
                assert_eq!(a.0.to_bits(), b.0.to_bits());
                assert_eq!(a.1.to_bits(), b.1.to_bits());
            }
            (None, None) => {}
            _ => panic!("panic behavior must itself be deterministic"),
        }
    }

    #[test]
    fn g1_hybrid_torque_gait_steps_in_place() {
        // The hybrid walking tick: ankles and arms servo-follow the scripted
        // gait targets (updated without stepping), hips and knees track the
        // same targets through torque PD, one physics step per tick. At the
        // scripted gait's stable operating point (its default near-stationary
        // step) the hybrid marches at full height. (At stride 0.20 the
        // scripted gait falls under BOTH regimes - a limit of the gait, not
        // of the torque pathway.)
        let run = || {
            let mut sim = g1_settled_stand();
            let command = UnitreeG1GaitCommand::default();
            let start_x = sim.observe().base_x_m;
            let mut min_height = f64::MAX;
            for step in 0..720_u64 {
                let targets = unitree_g1_gait_targets(step, command);
                let servo: Vec<_> = targets
                    .iter()
                    .filter(|target| !G1_TORQUE_LINKS.contains(&target.link_name))
                    .copied()
                    .collect();
                sim.set_joint_position_targets(&servo);
                let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
                    .iter()
                    .filter(|target| G1_TORQUE_LINKS.contains(&target.link_name))
                    .map(|target| {
                        let q = sim
                            .named_joint_position(target.link_name)
                            .expect("joint position");
                        let qd = sim
                            .named_joint_velocity(target.link_name)
                            .expect("joint velocity");
                        UrdfJointTorqueTarget {
                            link_name: target.link_name,
                            torque_nm: (300.0 * (target.position - q) - 10.0 * qd)
                                .clamp(-88.0, 88.0),
                            max_velocity_rad_s: G1_SPEED_LIMIT_RAD_S,
                        }
                    })
                    .collect();
                sim.step_joint_torques(&torques);
                min_height = min_height.min(sim.observe().base_y_m);
            }
            (sim.observe().base_x_m - start_x, min_height)
        };
        let (drift, min_height) = run();
        println!("hybrid step-in-place: drift {drift:+.3} m minH {min_height:.3}");
        assert!(
            min_height > 0.7,
            "the hybrid gait must keep stepping at height, minH {min_height:.3}"
        );
        assert!(
            drift.abs() < 0.4,
            "the near-stationary gait must not wander, drift {drift:+.3} m"
        );

        // Determinism: bit-identical repeat.
        let (again_drift, again_height) = run();
        assert_eq!(drift.to_bits(), again_drift.to_bits());
        assert_eq!(min_height.to_bits(), again_height.to_bits());
    }

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
