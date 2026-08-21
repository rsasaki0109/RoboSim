//! Tsukuba Challenge 2026 shortened full-run analog with official stop-line geometry.
//!
//! This is not the 2.2 km city loop. It scores three pedestrian-crossing stop lines
//! using the official 1 m / 0.5 m box, timed signal waits, and no roadway entry.

use super::tsukuba_confirmation::{evaluate_tsukuba_stop_line, TsukubaPlanarAabb};
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
use rne_math::{Quat, Vec3};
use rne_physics::{hash_physics_state, RigidBody};
use rne_robot::DiffDriveSpawned;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Portable task identity for the shortened full-run analog.
pub const TSUKUBA_FULL_RUN_TASK_ID: &str = "rne.tsukuba.full_run.v1";

const CONTROL_HZ: f64 = 60.0;
const CRUISE_WHEEL_RAD_S: f64 = 5.0;
const STOPPED_SPEED_M_S: f64 = 0.05;
const STOPPED_WHEEL_RAD_S: f64 = 0.2;
const SIGNAL_RED_STEPS: u32 = 24;
const DEFAULT_MAX_STEPS: u64 = 2_400;

/// Scaled linear analog of three signalized crossings on one sidewalk segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TsukubaFullRunCourse {
    /// Sidewalk half-width in meters; `|z|` beyond this is roadway.
    pub sidewalk_half_width_m: f64,
    /// Three stop-line X positions in meters, travel along +X.
    pub stop_lines_x_m: [f64; 3],
    /// Full-run goal X in meters.
    pub goal_x_m: f64,
    /// Robot base half-length along X in meters.
    pub robot_half_x_m: f64,
    /// Robot base half-width along Z in meters.
    pub robot_half_z_m: f64,
    /// Consecutive stopped steps required to count a temporary stop.
    pub required_stop_steps: u32,
}

impl Default for TsukubaFullRunCourse {
    fn default() -> Self {
        Self {
            sidewalk_half_width_m: 1.0,
            stop_lines_x_m: [2.5, 5.0, 7.5],
            goal_x_m: 10.0,
            robot_half_x_m: 0.25,
            robot_half_z_m: 0.2,
            required_stop_steps: 12,
        }
    }
}

impl TsukubaFullRunCourse {
    /// Footprint of the robot base in the ground plane.
    #[must_use]
    pub fn robot_aabb(self, center_x_m: f64, center_z_m: f64, yaw_rad: f64) -> TsukubaPlanarAabb {
        planar_aabb(
            center_x_m,
            center_z_m,
            yaw_rad,
            self.robot_half_x_m,
            self.robot_half_z_m,
        )
    }
}

/// Injected full-run faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TsukubaFullRunFault {
    /// Stop at each stop line, wait for green, then finish at the goal.
    #[default]
    None,
    /// Drive through all stop lines without the required temporary stop.
    SkipStopLines,
}

/// Headless observation consumed by full-run behavior contracts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TsukubaFullRunObservation {
    /// Completed simulation steps.
    pub step: u64,
    /// Base X in meters.
    pub base_x_m: f64,
    /// Base Y in meters.
    pub base_y_m: f64,
    /// Base Z in meters.
    pub base_z_m: f64,
    /// Base yaw around world Y in radians.
    pub base_yaw_rad: f64,
    /// Planar speed in meters per second.
    pub speed_m_s: f64,
    /// True when the body is outside the sidewalk.
    pub in_roadway: bool,
    /// Completed official stop-line stops.
    pub stop_line_complete: [bool; 3],
    /// Passed a stop line without first completing its stop.
    pub unstopped_stop_line_overshoot: bool,
    /// Pedestrian signal is green at the active crossing.
    pub signal_green: bool,
    /// Completed the required red-signal wait at each crossing.
    pub signal_wait_complete: [bool; 3],
    /// Wheel commands and planar speed are at rest.
    pub stopped: bool,
    /// Base X is past the goal marker.
    pub past_goal: bool,
    /// All stop-line, signal-wait, and goal contracts are satisfied.
    pub full_run_complete: bool,
}

/// Headless shortened full-run analog driven by a scripted sidewalk policy.
pub struct TsukubaFullRunScenario {
    sim: DiffDriveSim,
    course: TsukubaFullRunCourse,
    fault: TsukubaFullRunFault,
    max_steps: u64,
    phase: ScriptPhase,
    phase_ticks: u32,
    stop_line_streak: [u32; 3],
    stop_line_complete: [bool; 3],
    signal_wait_complete: [bool; 3],
    unstopped_stop_line_overshoot: bool,
    observation: TsukubaFullRunObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for TsukubaFullRunScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("TsukubaFullRunScenario")
            .field("fault", &self.fault)
            .field("phase", &self.phase)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScriptPhase {
    ApproachStopLine(usize),
    HoldStopLine(usize),
    WaitSignal(usize),
    CrossStopLine(usize),
    Finish,
    Halt,
}

impl TsukubaFullRunScenario {
    /// Loads the bundled full-run scene for a successful scripted run.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, TsukubaFullRunFault::None)
    }

    /// Loads the bundled full-run scene with an injected fault.
    pub fn new(seed: u64, fault: TsukubaFullRunFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = tsukuba_full_run_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let course = TsukubaFullRunCourse::default();
        let mut scenario = Self {
            sim,
            course,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            phase: ScriptPhase::ApproachStopLine(0),
            phase_ticks: 0,
            stop_line_streak: [0; 3],
            stop_line_complete: [false; 3],
            signal_wait_complete: [false; 3],
            unstopped_stop_line_overshoot: false,
            observation: placeholder_observation(),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current full-run observation.
    #[must_use]
    pub fn current_observation(&self) -> TsukubaFullRunObservation {
        self.observation
    }

    fn observe_world(&self) -> TsukubaFullRunObservation {
        let drive = self.sim.observe();
        let spawned = &self.sim.robots()[0];
        let speed_m_s = planar_speed(&self.sim, spawned);
        let stopped = is_stopped(
            speed_m_s,
            drive.left_wheel_velocity_rad_s,
            drive.right_wheel_velocity_rad_s,
        );
        let aabb = self
            .course
            .robot_aabb(drive.base_x_m, drive.base_z_m, drive.base_yaw_rad);
        let in_roadway = aabb.max_z_m > self.course.sidewalk_half_width_m
            || aabb.min_z_m < -self.course.sidewalk_half_width_m;
        let past_goal = drive.base_x_m >= self.course.goal_x_m;
        let signal_green = matches!(self.phase, ScriptPhase::CrossStopLine(_))
            || (matches!(self.phase, ScriptPhase::WaitSignal(_))
                && self.phase_ticks >= SIGNAL_RED_STEPS)
            || matches!(self.phase, ScriptPhase::Finish | ScriptPhase::Halt);
        let full_run_complete = self.stop_line_complete.iter().all(|complete| *complete)
            && self.signal_wait_complete.iter().all(|complete| *complete)
            && past_goal
            && stopped;

        TsukubaFullRunObservation {
            step: self.sim.step_count(),
            base_x_m: drive.base_x_m,
            base_y_m: drive.base_y_m,
            base_z_m: drive.base_z_m,
            base_yaw_rad: drive.base_yaw_rad,
            speed_m_s,
            in_roadway,
            stop_line_complete: self.stop_line_complete,
            unstopped_stop_line_overshoot: self.unstopped_stop_line_overshoot,
            signal_green,
            signal_wait_complete: self.signal_wait_complete,
            stopped,
            past_goal,
            full_run_complete,
        }
    }

    fn update_judge(&mut self, observation: TsukubaFullRunObservation) {
        let aabb = self.course.robot_aabb(
            observation.base_x_m,
            observation.base_z_m,
            observation.base_yaw_rad,
        );
        for (index, line_x_m) in self.course.stop_lines_x_m.iter().copied().enumerate() {
            if self.stop_line_complete[index] {
                continue;
            }
            let judgement = evaluate_tsukuba_stop_line(aabb.min_x_m, aabb.max_x_m, line_x_m);
            if judgement.overshoot {
                self.unstopped_stop_line_overshoot = true;
            }
            if judgement.valid() && observation.stopped {
                self.stop_line_streak[index] = self.stop_line_streak[index]
                    .saturating_add(1)
                    .min(self.course.required_stop_steps);
                if self.stop_line_streak[index] >= self.course.required_stop_steps {
                    self.stop_line_complete[index] = true;
                }
            } else {
                self.stop_line_streak[index] = 0;
            }
        }
    }

    fn action(&mut self, observation: TsukubaFullRunObservation) -> DiffDriveAction {
        if matches!(self.fault, TsukubaFullRunFault::SkipStopLines) {
            return DiffDriveAction::forward(CRUISE_WHEEL_RAD_S);
        }

        match self.phase {
            ScriptPhase::ApproachStopLine(index) => {
                let line_x_m = self.course.stop_lines_x_m[index];
                let stop_x_m = line_x_m - self.course.robot_half_x_m - 0.2;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldStopLine(index);
                    self.phase_ticks = 0;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldStopLine(index) => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.stop_line_complete[index] {
                    self.phase = ScriptPhase::WaitSignal(index);
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::WaitSignal(index) => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= SIGNAL_RED_STEPS {
                    self.signal_wait_complete[index] = true;
                    self.phase = ScriptPhase::CrossStopLine(index);
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::CrossStopLine(index) => {
                let line_x_m = self.course.stop_lines_x_m[index];
                if observation.base_x_m > line_x_m + 0.45 {
                    self.phase = if index + 1 < self.course.stop_lines_x_m.len() {
                        ScriptPhase::ApproachStopLine(index + 1)
                    } else {
                        ScriptPhase::Finish
                    };
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
            }
            ScriptPhase::Finish => {
                if observation.past_goal && observation.stopped {
                    self.phase = ScriptPhase::Halt;
                    DiffDriveAction::forward(0.0)
                } else if observation.past_goal {
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::Halt => DiffDriveAction::forward(0.0),
        }
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

impl BehaviorScenario for TsukubaFullRunScenario {
    type Observation = TsukubaFullRunObservation;

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
        let mut bytes = b"tsukuba_full_run_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            TsukubaFullRunFault::None => 0,
            TsukubaFullRunFault::SkipStopLines => 1,
        });
        stable_behavior_digest(&bytes)
    }

    fn behavior_dimensions(&self) -> Vec<BehaviorDimension> {
        self.dimensions.clone()
    }

    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
        let deadline = self.deadline();
        Ok(vec![
            BehaviorContract::always(
                "no_roadway_entry",
                |observation: &TsukubaFullRunObservation| !observation.in_roadway,
            )?
            .with_entities(["tsukuba_full_run"])?,
            BehaviorContract::always(
                "no_unstopped_stop_line_overshoot",
                |observation: &TsukubaFullRunObservation| {
                    !observation.unstopped_stop_line_overshoot
                },
            )?
            .with_entities(["stop_line_1", "stop_line_2", "stop_line_3"])?,
            BehaviorContract::eventually(
                "first_stop_line_stop",
                deadline,
                |observation: &TsukubaFullRunObservation| observation.stop_line_complete[0],
            )?
            .with_entities(["stop_line_1"])?,
            BehaviorContract::eventually(
                "second_stop_line_stop",
                deadline,
                |observation: &TsukubaFullRunObservation| observation.stop_line_complete[1],
            )?
            .with_entities(["stop_line_2"])?,
            BehaviorContract::eventually(
                "third_stop_line_stop",
                deadline,
                |observation: &TsukubaFullRunObservation| observation.stop_line_complete[2],
            )?
            .with_entities(["stop_line_3"])?,
            BehaviorContract::eventually(
                "signal_wait_at_crossings",
                deadline,
                |observation: &TsukubaFullRunObservation| {
                    observation
                        .signal_wait_complete
                        .iter()
                        .all(|complete| *complete)
                },
            )?
            .with_entities(["stop_line_1", "stop_line_2", "stop_line_3"])?,
            BehaviorContract::eventually(
                "full_run_complete",
                deadline,
                |observation: &TsukubaFullRunObservation| observation.full_run_complete,
            )?
            .with_entities(["full_run_goal"])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let action = self.action(self.observation);
        self.sim.step_action(action);
        let mut observation = self.observe_world();
        self.update_judge(observation);
        observation = self.observe_world();
        self.observation = observation;
        let done = observation.full_run_complete
            || observation.unstopped_stop_line_overshoot
            || observation.in_roadway
            || observation.step >= self.max_steps;
        BehaviorScenarioStep { observation, done }
    }
}

/// Returns the bundled full-run scene path.
#[must_use]
pub fn tsukuba_full_run_scene_path() -> PathBuf {
    bundled_asset_path(Path::new("scenes/tsukuba_full_run.rne.scene.toml"))
}

/// Portable TaskSpec for the shortened full-run analog.
#[must_use]
pub fn tsukuba_full_run_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        TSUKUBA_FULL_RUN_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("in_roadway", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("stop_line_complete", TensorDType::F64, vec![3], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("signal_green", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("signal_wait_complete", TensorDType::F64, vec![3], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("past_goal", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "wheel_velocity_rad_s",
            TensorDType::F64,
            vec![2],
            "rad/s",
        )
        .with_bounds(TensorBounds::broadcast(-10.0, 10.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("forward_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("full_run_complete", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("full_run_complete", TerminationKind::Success),
                TerminationConditionSpec::new("stop_line_overshoot", TerminationKind::Failure),
                TerminationConditionSpec::new("roadway_entry", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

fn planar_aabb(
    center_x_m: f64,
    center_z_m: f64,
    yaw_rad: f64,
    half_x_m: f64,
    half_z_m: f64,
) -> TsukubaPlanarAabb {
    let rotation = Quat::from_rotation_y(yaw_rad);
    let locals = [
        Vec3::new(half_x_m, 0.0, half_z_m),
        Vec3::new(half_x_m, 0.0, -half_z_m),
        Vec3::new(-half_x_m, 0.0, half_z_m),
        Vec3::new(-half_x_m, 0.0, -half_z_m),
    ];
    let mut min_x_m = f64::INFINITY;
    let mut max_x_m = f64::NEG_INFINITY;
    let mut min_z_m = f64::INFINITY;
    let mut max_z_m = f64::NEG_INFINITY;
    for local in locals {
        let world = rotation * local;
        let x_m = center_x_m + world.x;
        let z_m = center_z_m + world.z;
        min_x_m = min_x_m.min(x_m);
        max_x_m = max_x_m.max(x_m);
        min_z_m = min_z_m.min(z_m);
        max_z_m = max_z_m.max(z_m);
    }
    TsukubaPlanarAabb {
        min_x_m,
        max_x_m,
        min_z_m,
        max_z_m,
    }
}

fn placeholder_observation() -> TsukubaFullRunObservation {
    TsukubaFullRunObservation {
        step: 0,
        base_x_m: 0.0,
        base_y_m: 0.0,
        base_z_m: 0.0,
        base_yaw_rad: 0.0,
        speed_m_s: 0.0,
        in_roadway: false,
        stop_line_complete: [false; 3],
        unstopped_stop_line_overshoot: false,
        signal_green: false,
        signal_wait_complete: [false; 3],
        stopped: true,
        past_goal: false,
        full_run_complete: false,
    }
}

fn is_stopped(speed_m_s: f64, left_wheel_rad_s: f64, right_wheel_rad_s: f64) -> bool {
    speed_m_s.abs() <= STOPPED_SPEED_M_S
        && left_wheel_rad_s.abs() <= STOPPED_WHEEL_RAD_S
        && right_wheel_rad_s.abs() <= STOPPED_WHEEL_RAD_S
}

fn planar_speed(sim: &DiffDriveSim, spawned: &DiffDriveSpawned) -> f64 {
    sim.world()
        .get::<RigidBody>(spawned.base_link)
        .map(|body| Vec3::new(body.linear_velocity_m_s.x, 0.0, body.linear_velocity_m_s.z).length())
        .unwrap_or(0.0)
}

fn fault_dimensions(fault: TsukubaFullRunFault) -> Vec<BehaviorDimension> {
    vec![BehaviorDimension {
        name: "skip_stop_lines".to_string(),
        value: BehaviorDimensionValue::Boolean(matches!(fault, TsukubaFullRunFault::SkipStopLines)),
        baseline: BehaviorDimensionValue::Boolean(false),
    }]
}

fn digest_scene_inputs(scene_path: &Path) -> Result<u64, AssetError> {
    let mut bytes = b"rne_behavior_scene_inputs_v1".to_vec();
    let bundle = load_scene_bundle(scene_path)?;
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
    fn full_run_task_spec_matches_committed_json() {
        let spec = tsukuba_full_run_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let path = bundled_asset_path(Path::new("tasks/tsukuba_full_run.task.json"));
        let loaded: TaskSpec =
            serde_json::from_slice(&fs::read(&path).expect("committed task spec"))
                .expect("parse task spec");
        assert_eq!(tsukuba_full_run_task_spec(DEFAULT_MAX_STEPS), loaded);
    }

    #[test]
    fn scripted_full_run_stops_at_three_stop_lines() {
        let report = run_behavior_scenarios(
            "tsukuba_full_run_success",
            [1],
            TsukubaFullRunScenario::success,
        );
        assert!(report.passed(), "{report:?}");
    }

    #[test]
    fn skipping_stop_lines_fails_first_stop_contract() {
        let report = run_behavior_scenarios("tsukuba_full_run_skip_stops", [1], |seed| {
            TsukubaFullRunScenario::new(seed, TsukubaFullRunFault::SkipStopLines)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let stop = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "first_stop_line_stop")
            .expect("first stop contract");
        assert_eq!(stop.status, BehaviorContractStatus::Failed);
    }
}
