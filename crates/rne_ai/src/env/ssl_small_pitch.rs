//! RoboCup SSL Division B 2v2 analog scored by official field geometry.
//!
//! This is not a grSim clone and does not speak the SSL simulation protobuf
//! ports. The first slice is a headless 9 m × 6 m pitch with four 180 mm
//! robots and a golf-ball, judged by goal mouths, out-of-bounds, and the
//! 6.5 m/s ball-speed cap.

use crate::action::DiffDriveAction;
use crate::asset_path::bundled_asset_path;
use crate::behavior::{
    BehaviorContract, BehaviorContractError, BehaviorScenario, BehaviorScenarioStep,
};
use crate::behavior_replay::{stable_behavior_digest, BehaviorDimension, BehaviorDimensionValue};
use crate::env::DiffDriveSim;
use crate::task::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_assets::{load_scene_bundle, scene_dependency_paths, AssetError};
use rne_core::SimDuration;
use rne_ecs::{Entity, Name, World};
use rne_math::{Quat, Vec3};
use rne_physics::{hash_physics_state, RigidBody};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Portable task identity for the SSL 2v2 analog.
pub const SSL_SMALL_PITCH_TASK_ID: &str = "rne.ssl.small_pitch_2v2.v1";
/// Official Division B playing-field length in meters.
pub const SSL_DIV_B_FIELD_LENGTH_M: f64 = 9.0;
/// Official Division B playing-field width in meters.
pub const SSL_DIV_B_FIELD_WIDTH_M: f64 = 6.0;
/// Official goal inner width used by this analog in meters.
pub const SSL_GOAL_WIDTH_M: f64 = 1.0;
/// Official goal depth in meters.
pub const SSL_GOAL_DEPTH_M: f64 = 0.18;
/// Golf-ball radius used by SSL in meters.
pub const SSL_BALL_RADIUS_M: f64 = 0.0215;
/// Maximum robot cylinder radius in meters.
pub const SSL_ROBOT_MAX_RADIUS_M: f64 = 0.09;
/// Official maximum ball speed in meters per second.
pub const SSL_MAX_BALL_SPEED_M_S: f64 = 6.5;
/// Scene entity name of the ball.
pub const SSL_BALL_NAME: &str = "ssl_ball";

const CONTROL_HZ: f64 = 60.0;
const DEFAULT_MAX_STEPS: u64 = 2_000;
const CRUISE_WHEEL_RAD_S: f64 = 8.0;
const TURN_WHEEL_RAD_S: f64 = 4.0;
const HEADING_ALIGN_RAD: f64 = 0.35;
const ATTACKER_NAME: &str = "ssl_blue_0";
const ROBOT_NAMES: [&str; 4] = ["ssl_blue_0", "ssl_blue_1", "ssl_yellow_0", "ssl_yellow_1"];

/// Where the ball sits relative to the official field and goals.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SslBallRegion {
    /// Inside the playing area, not yet a goal.
    InPlay,
    /// Fully across the +X goal line inside the yellow mouth.
    YellowGoal,
    /// Fully across the −X goal line inside the blue mouth.
    BlueGoal,
    /// Past a sideline or an end line outside either goal mouth.
    OutOfField,
}

/// Official Division B pitch used by the geometric judges.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SslSmallPitch {
    /// Playing-field length along X in meters.
    pub field_length_m: f64,
    /// Playing-field width along Z in meters.
    pub field_width_m: f64,
    /// Goal inner width along Z in meters.
    pub goal_width_m: f64,
    /// Goal depth along X in meters.
    pub goal_depth_m: f64,
    /// Ball radius in meters.
    pub ball_radius_m: f64,
    /// Legal ball-speed cap in meters per second.
    pub max_ball_speed_m_s: f64,
}

impl Default for SslSmallPitch {
    fn default() -> Self {
        Self {
            field_length_m: SSL_DIV_B_FIELD_LENGTH_M,
            field_width_m: SSL_DIV_B_FIELD_WIDTH_M,
            goal_width_m: SSL_GOAL_WIDTH_M,
            goal_depth_m: SSL_GOAL_DEPTH_M,
            ball_radius_m: SSL_BALL_RADIUS_M,
            max_ball_speed_m_s: SSL_MAX_BALL_SPEED_M_S,
        }
    }
}

impl SslSmallPitch {
    /// Half length (distance from kickoff to a goal line).
    #[must_use]
    pub fn half_length_m(self) -> f64 {
        self.field_length_m * 0.5
    }

    /// Half width (distance from kickoff to a sideline).
    #[must_use]
    pub fn half_width_m(self) -> f64 {
        self.field_width_m * 0.5
    }

    /// Classifies a ball center against the official field and goal mouths.
    #[must_use]
    pub fn ball_region(self, ball_x_m: f64, ball_z_m: f64) -> SslBallRegion {
        evaluate_ssl_ball_region(self, ball_x_m, ball_z_m)
    }
}

/// Official SSL ball location: the center must fully cross a goal line
/// inside the mouth, otherwise leaving the rectangle is out of field.
#[must_use]
pub fn evaluate_ssl_ball_region(
    pitch: SslSmallPitch,
    ball_x_m: f64,
    ball_z_m: f64,
) -> SslBallRegion {
    let r = pitch.ball_radius_m;
    let half_l = pitch.half_length_m();
    let half_w = pitch.half_width_m();
    let half_goal = pitch.goal_width_m * 0.5;
    let in_goal_z = ball_z_m.abs() + r <= half_goal;
    let past_yellow_line = ball_x_m - r >= half_l;
    let past_blue_line = ball_x_m + r <= -half_l;
    let past_plus_sideline = ball_z_m - r >= half_w;
    let past_minus_sideline = ball_z_m + r <= -half_w;
    if past_yellow_line && in_goal_z && ball_x_m + r <= half_l + pitch.goal_depth_m {
        SslBallRegion::YellowGoal
    } else if past_blue_line && in_goal_z && ball_x_m - r >= -half_l - pitch.goal_depth_m {
        SslBallRegion::BlueGoal
    } else if past_yellow_line || past_blue_line || past_plus_sideline || past_minus_sideline {
        SslBallRegion::OutOfField
    } else {
        SslBallRegion::InPlay
    }
}

/// Returns the bundled 2v2 pitch scene path.
#[must_use]
pub fn ssl_small_pitch_scene_path() -> PathBuf {
    bundled_asset_path(Path::new("scenes/ssl_small_pitch_2v2.rne.scene.toml"))
}

/// Portable TaskSpec for the SSL 2v2 analog.
#[must_use]
pub fn ssl_small_pitch_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        SSL_SMALL_PITCH_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("ball_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("ball_speed_m_s", TensorDType::F64, vec![], "m/s"),
            TensorSpec::new("attacker_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("in_yellow_goal", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("in_blue_goal", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("out_of_field", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("ball_speed_illegal", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "wheel_velocity_rad_s",
            TensorDType::F64,
            vec![8],
            "rad/s",
        )
        .with_bounds(TensorBounds::broadcast(-10.0, 10.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("ball_progress_x_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("yellow_goal", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("yellow_goal", TerminationKind::Success),
                TerminationConditionSpec::new("blue_goal", TerminationKind::Failure),
                TerminationConditionSpec::new("out_of_field", TerminationKind::Failure),
                TerminationConditionSpec::new("ball_speed_illegal", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Injected 2v2 faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SslSmallPitchFault {
    /// Blue attacker pushes the ball into the yellow goal.
    #[default]
    None,
    /// Blue attacker drives the ball out over a sideline.
    DriveOut,
}

/// Headless observation consumed by SSL 2v2 behavior contracts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SslSmallPitchObservation {
    /// Completed simulation steps.
    pub step: u64,
    /// Ball X in meters.
    pub ball_x_m: f64,
    /// Ball Y in meters.
    pub ball_y_m: f64,
    /// Ball Z in meters.
    pub ball_z_m: f64,
    /// Planar ball speed in meters per second.
    pub ball_speed_m_s: f64,
    /// Attacking robot X in meters.
    pub attacker_x_m: f64,
    /// Attacking robot Z in meters.
    pub attacker_z_m: f64,
    /// Attacking robot yaw around world Y in radians.
    pub attacker_yaw_rad: f64,
    /// Official region of the ball.
    pub ball_region: SslBallRegion,
    /// True when ball speed exceeds the official cap.
    pub ball_speed_illegal: bool,
}

impl SslSmallPitchObservation {
    /// Ball is in the yellow goal mouth.
    #[must_use]
    pub fn yellow_goal(self) -> bool {
        self.ball_region == SslBallRegion::YellowGoal
    }

    /// Ball is in the blue goal mouth.
    #[must_use]
    pub fn blue_goal(self) -> bool {
        self.ball_region == SslBallRegion::BlueGoal
    }

    /// Ball left the field without scoring.
    #[must_use]
    pub fn out_of_field(self) -> bool {
        self.ball_region == SslBallRegion::OutOfField
    }
}

/// Headless 2v2 analog driven by a scripted attacker.
pub struct SslSmallPitchScenario {
    sim: DiffDriveSim,
    pitch: SslSmallPitch,
    fault: SslSmallPitchFault,
    max_steps: u64,
    observation: SslSmallPitchObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for SslSmallPitchScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SslSmallPitchScenario")
            .field("fault", &self.fault)
            .field("max_steps", &self.max_steps)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

impl SslSmallPitchScenario {
    /// Loads the bundled pitch for a successful scripted attack.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, SslSmallPitchFault::None)
    }

    /// Loads the bundled pitch with an injected fault.
    pub fn new(seed: u64, fault: SslSmallPitchFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = ssl_small_pitch_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let mut scenario = Self {
            sim,
            pitch: SslSmallPitch::default(),
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            observation: placeholder_observation(),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current 2v2 observation.
    #[must_use]
    pub fn current_observation(&self) -> SslSmallPitchObservation {
        self.observation
    }

    fn observe_world(&self) -> SslSmallPitchObservation {
        let ball = named_translation(self.sim.world(), SSL_BALL_NAME).unwrap_or(Vec3::ZERO);
        let ball_speed_m_s = named_planar_speed(self.sim.world(), SSL_BALL_NAME);
        let attacker = self
            .sim
            .robots()
            .iter()
            .find(|robot| entity_name(self.sim.world(), robot.robot) == Some(ATTACKER_NAME))
            .map(|robot| self.sim.observe_robot(robot.robot));
        let attacker_x_m = attacker.map(|obs| obs.base_x_m).unwrap_or(0.0);
        let attacker_z_m = attacker.map(|obs| obs.base_z_m).unwrap_or(0.0);
        let attacker_yaw_rad = attacker.map(|obs| obs.base_yaw_rad).unwrap_or(0.0);
        SslSmallPitchObservation {
            step: self.sim.step_count(),
            ball_x_m: ball.x,
            ball_y_m: ball.y,
            ball_z_m: ball.z,
            ball_speed_m_s,
            attacker_x_m,
            attacker_z_m,
            attacker_yaw_rad,
            ball_region: self.pitch.ball_region(ball.x, ball.z),
            ball_speed_illegal: ball_speed_m_s > self.pitch.max_ball_speed_m_s,
        }
    }

    fn actions(&self, observation: SslSmallPitchObservation) -> Vec<(Entity, DiffDriveAction)> {
        let target = match self.fault {
            SslSmallPitchFault::None => attack_target(self.pitch, observation),
            SslSmallPitchFault::DriveOut => drive_out_target(self.pitch, observation),
        };
        self.sim
            .robots()
            .iter()
            .map(|robot| {
                let is_attacker = entity_name(self.sim.world(), robot.robot) == Some(ATTACKER_NAME);
                let action = if is_attacker {
                    let pose = self.sim.observe_robot(robot.robot);
                    drive_toward(
                        pose.base_x_m,
                        pose.base_z_m,
                        pose.base_yaw_rad,
                        target.x,
                        target.z,
                    )
                } else {
                    DiffDriveAction::forward(0.0)
                };
                (robot.robot, action)
            })
            .collect()
    }

    fn deadline(&self) -> SimDuration {
        SimDuration::from_ticks(
            self.sim
                .fixed_delta()
                .ticks()
                .saturating_mul(self.max_steps),
        )
    }
}

impl BehaviorScenario for SslSmallPitchScenario {
    type Observation = SslSmallPitchObservation;

    fn fixed_delta(&self) -> SimDuration {
        self.sim.fixed_delta()
    }

    fn initial_observation(&self) -> Self::Observation {
        self.observation
    }

    fn state_digest(&self, _observation: &Self::Observation) -> u64 {
        hash_physics_state(self.sim.world())
    }

    fn scenario_digest(&self) -> u64 {
        let mut bytes = b"ssl_small_pitch_2v2_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&(ROBOT_NAMES.len() as u64).to_le_bytes());
        bytes.push(match self.fault {
            SslSmallPitchFault::None => 0,
            SslSmallPitchFault::DriveOut => 1,
        });
        for dimension in &self.dimensions {
            bytes.extend_from_slice(dimension.name.as_bytes());
            bytes.push(0);
            match &dimension.value {
                BehaviorDimensionValue::Boolean(value) => bytes.push(u8::from(*value)),
                BehaviorDimensionValue::Number(value) => {
                    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
                }
                BehaviorDimensionValue::Text(value) => bytes.extend_from_slice(value.as_bytes()),
            }
        }
        stable_behavior_digest(&bytes)
    }

    fn behavior_dimensions(&self) -> Vec<BehaviorDimension> {
        self.dimensions.clone()
    }

    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
        let deadline = self.deadline();
        Ok(vec![
            BehaviorContract::always(
                "ball_in_play_or_goal",
                |observation: &SslSmallPitchObservation| !observation.out_of_field(),
            )?
            .with_entities([SSL_BALL_NAME])?,
            BehaviorContract::always(
                "legal_ball_speed",
                |observation: &SslSmallPitchObservation| !observation.ball_speed_illegal,
            )?
            .with_entities([SSL_BALL_NAME])?,
            BehaviorContract::always("no_own_goal", |observation: &SslSmallPitchObservation| {
                !observation.blue_goal()
            })?
            .with_entities(["blue_goal"])?,
            BehaviorContract::eventually(
                "yellow_goal",
                deadline,
                |observation: &SslSmallPitchObservation| observation.yellow_goal(),
            )?
            .with_entities(["yellow_goal"])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let actions = self.actions(self.observation);
        self.sim.step_robots_actions(&actions);
        self.observation = self.observe_world();
        let done = self.observation.yellow_goal()
            || self.observation.blue_goal()
            || self.observation.out_of_field()
            || self.observation.ball_speed_illegal
            || self.observation.step >= self.max_steps;
        BehaviorScenarioStep {
            observation: self.observation,
            done,
        }
    }
}

fn drive_out_target(pitch: SslSmallPitch, observation: SslSmallPitchObservation) -> Vec3 {
    let ball = Vec3::new(observation.ball_x_m, 0.0, observation.ball_z_m);
    let approach = ball + Vec3::new(0.0, 0.0, -0.18);
    let attacker = Vec3::new(observation.attacker_x_m, 0.0, observation.attacker_z_m);
    if (attacker - approach).length() > 0.12 && observation.ball_z_m < 1.0 {
        approach
    } else {
        Vec3::new(ball.x, 0.0, pitch.half_width_m() + 1.5)
    }
}

fn attack_target(pitch: SslSmallPitch, observation: SslSmallPitchObservation) -> Vec3 {
    let goal = Vec3::new(pitch.half_length_m() + 0.05, 0.0, 0.0);
    let ball = Vec3::new(observation.ball_x_m, 0.0, observation.ball_z_m);
    let to_goal = (goal - ball).normalize_or_zero();
    let distance =
        (Vec3::new(observation.attacker_x_m, 0.0, observation.attacker_z_m) - ball).length();
    if distance < 0.22 {
        goal
    } else {
        ball - to_goal * 0.16
    }
}

fn drive_toward(
    x_m: f64,
    z_m: f64,
    yaw_rad: f64,
    target_x_m: f64,
    target_z_m: f64,
) -> DiffDriveAction {
    let desired = Vec3::new(target_x_m - x_m, 0.0, target_z_m - z_m).normalize_or_zero();
    if desired.length_squared() < 1.0e-8 {
        return DiffDriveAction::forward(0.0);
    }
    let forward = Quat::from_rotation_y(yaw_rad) * Vec3::X;
    let cross = forward.x * desired.z - forward.z * desired.x;
    let dot = forward.x * desired.x + forward.z * desired.z;
    let heading_error_rad = cross.atan2(dot);
    if heading_error_rad.abs() > HEADING_ALIGN_RAD {
        let turn = -heading_error_rad.signum() * TURN_WHEEL_RAD_S;
        DiffDriveAction {
            left_velocity_rad_s: -turn,
            right_velocity_rad_s: turn,
        }
    } else {
        DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
    }
}

fn placeholder_observation() -> SslSmallPitchObservation {
    SslSmallPitchObservation {
        step: 0,
        ball_x_m: 0.0,
        ball_y_m: SSL_BALL_RADIUS_M,
        ball_z_m: 0.0,
        ball_speed_m_s: 0.0,
        attacker_x_m: -2.0,
        attacker_z_m: 0.0,
        attacker_yaw_rad: 0.0,
        ball_region: SslBallRegion::InPlay,
        ball_speed_illegal: false,
    }
}

fn entity_name(world: &World, entity: Entity) -> Option<&str> {
    world.get::<Name>(entity).map(|name| name.0.as_str())
}

fn entity_named(world: &World, name: &str) -> Option<Entity> {
    world.iter_entities().find_map(|entity_ref| {
        world
            .get::<Name>(entity_ref.id())
            .is_some_and(|entity_name| entity_name.0 == name)
            .then_some(entity_ref.id())
    })
}

fn named_translation(world: &World, name: &str) -> Option<Vec3> {
    let entity = entity_named(world, name)?;
    world.get::<Transform3>(entity).map(|tf| tf.translation)
}

fn named_planar_speed(world: &World, name: &str) -> f64 {
    let Some(entity) = entity_named(world, name) else {
        return 0.0;
    };
    world
        .get::<RigidBody>(entity)
        .map(|body| Vec3::new(body.linear_velocity_m_s.x, 0.0, body.linear_velocity_m_s.z).length())
        .unwrap_or(0.0)
}

fn fault_dimensions(fault: SslSmallPitchFault) -> Vec<BehaviorDimension> {
    vec![BehaviorDimension {
        name: "drive_out".to_string(),
        value: BehaviorDimensionValue::Boolean(matches!(fault, SslSmallPitchFault::DriveOut)),
        baseline: BehaviorDimensionValue::Boolean(false),
    }]
}

fn digest_scene_inputs(scene_path: &Path) -> Result<u64, AssetError> {
    let bundle = load_scene_bundle(scene_path)?;
    let mut bytes = b"rne_behavior_scene_inputs_v1".to_vec();
    for path in scene_dependency_paths(&bundle) {
        let contents = fs::read(&path).map_err(|error| AssetError::Io {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
        bytes.extend_from_slice(&(contents.len() as u64).to_le_bytes());
        bytes.extend_from_slice(&contents);
    }
    Ok(stable_behavior_digest(&bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{run_behavior_scenarios, BehaviorContractStatus, BehaviorSeedStatus};

    #[test]
    fn ball_fully_across_goal_line_is_a_goal() {
        let pitch = SslSmallPitch::default();
        let just_in = evaluate_ssl_ball_region(pitch, 4.5 + SSL_BALL_RADIUS_M, 0.0);
        assert_eq!(just_in, SslBallRegion::YellowGoal);

        let still_on_line = evaluate_ssl_ball_region(pitch, 4.5, 0.0);
        assert_eq!(still_on_line, SslBallRegion::InPlay);

        let wide_of_mouth = evaluate_ssl_ball_region(pitch, 4.6, 0.8);
        assert_eq!(wide_of_mouth, SslBallRegion::OutOfField);

        let sideline = evaluate_ssl_ball_region(pitch, 0.0, 3.1);
        assert_eq!(sideline, SslBallRegion::OutOfField);

        let blue = evaluate_ssl_ball_region(pitch, -4.5 - SSL_BALL_RADIUS_M, 0.0);
        assert_eq!(blue, SslBallRegion::BlueGoal);
    }

    #[test]
    fn small_pitch_task_spec_matches_committed_artifact() {
        let spec = ssl_small_pitch_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let path =
            crate::asset_path::bundled_asset_path(Path::new("tasks/ssl_small_pitch_2v2.task.json"));
        let loaded: TaskSpec =
            serde_json::from_slice(&fs::read(path).expect("committed task spec"))
                .expect("parse task spec");
        assert_eq!(spec, loaded);
    }

    #[test]
    fn attacker_advances_from_kickoff() {
        let mut scenario = SslSmallPitchScenario::success(1).expect("scenario");
        let start = scenario.current_observation();
        for _ in 0..180 {
            let _ = scenario.advance();
        }
        let now = scenario.current_observation();
        assert!(
            now.attacker_x_m > start.attacker_x_m + 0.5,
            "attacker should drive toward kickoff, start={:.3} now={:.3}",
            start.attacker_x_m,
            now.attacker_x_m
        );
        assert!(
            now.attacker_z_m.abs() < 0.15,
            "attacker should stay on the center line, z={:.3}",
            now.attacker_z_m
        );
    }

    #[test]
    fn scripted_attack_scores_the_yellow_goal() {
        let report = run_behavior_scenarios(
            "ssl_small_pitch_success",
            [1],
            SslSmallPitchScenario::success,
        );
        assert!(report.passed(), "{report:?}");
        assert!(report.seeds[0].steps > 50);
    }

    #[test]
    fn driving_the_ball_out_fails_in_play() {
        let report = run_behavior_scenarios("ssl_small_pitch_drive_out", [1], |seed| {
            SslSmallPitchScenario::new(seed, SslSmallPitchFault::DriveOut)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let in_play = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "ball_in_play_or_goal")
            .expect("in-play contract");
        assert_eq!(in_play.status, BehaviorContractStatus::Failed);
    }
}
