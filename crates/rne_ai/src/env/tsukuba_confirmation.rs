//! Tsukuba Challenge 2026 confirmation-run scoring on a scaled linear analog.
//!
//! The official confirmation checklist is geometric, not photoreal: two
//! road-edge stops, no green-cone contact, an e-stop to zero speed, and no
//! roadway entry. Distances match
//! <https://tsukubachallenge.jp/2026/regulations/tasks>. The course is shortened
//! to a few meters so the same judges can run headlessly.

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
use rne_robot::DiffDriveSpawned;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Portable task identity for the confirmation-run analog.
pub const TSUKUBA_CONFIRMATION_TASK_ID: &str = "rne.tsukuba.confirmation.v1";
/// Official stop-line look-ahead: 1 m before the line.
pub const TSUKUBA_STOP_LINE_BEFORE_M: f64 = 1.0;
/// Official stop-line overrun limit: 0.5 m past the line.
pub const TSUKUBA_STOP_LINE_AFTER_M: f64 = 0.5;
/// Official road-edge look-ahead: 1.5 m before the edge.
pub const TSUKUBA_ROAD_EDGE_BEFORE_M: f64 = 1.5;
/// Scene entity name of the green confirmation obstacle.
pub const TSUKUBA_GREEN_CONE_NAME: &str = "green_cone";

const CONTROL_HZ: f64 = 60.0;
const CRUISE_WHEEL_RAD_S: f64 = 5.0;
const STOPPED_SPEED_M_S: f64 = 0.05;
const STOPPED_WHEEL_RAD_S: f64 = 0.2;
const OPERATOR_HOLD_STEPS: u32 = 12;
const DEFAULT_MAX_STEPS: u64 = 1_200;
const CONE_NAME: &str = TSUKUBA_GREEN_CONE_NAME;

/// Axis-aligned footprint used by the official along-track stop judges.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TsukubaPlanarAabb {
    /// Minimum world X in meters.
    pub min_x_m: f64,
    /// Maximum world X in meters.
    pub max_x_m: f64,
    /// Minimum world Z in meters.
    pub min_z_m: f64,
    /// Maximum world Z in meters.
    pub max_z_m: f64,
}

impl TsukubaPlanarAabb {
    /// Returns whether two footprints overlap in the ground plane.
    #[must_use]
    pub fn overlaps(self, other: Self) -> bool {
        intervals_overlap(self.min_x_m, self.max_x_m, other.min_x_m, other.max_x_m)
            && intervals_overlap(self.min_z_m, self.max_z_m, other.min_z_m, other.max_z_m)
    }
}

/// Result of one official stop-region evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TsukubaStopJudgement {
    /// Some part of the body intersects the allowed stop box.
    pub in_region: bool,
    /// Some part of the body is past the allowed far bound.
    pub overshoot: bool,
}

impl TsukubaStopJudgement {
    /// Official valid stop: inside the box and not past the far bound.
    #[must_use]
    pub const fn valid(self) -> bool {
        self.in_region && !self.overshoot
    }
}

/// Official stop-line box: 1 m before the line to 0.5 m after it.
#[must_use]
pub fn evaluate_tsukuba_stop_line(
    min_along_m: f64,
    max_along_m: f64,
    line_m: f64,
) -> TsukubaStopJudgement {
    evaluate_stop_interval(
        min_along_m,
        max_along_m,
        line_m - TSUKUBA_STOP_LINE_BEFORE_M,
        line_m + TSUKUBA_STOP_LINE_AFTER_M,
    )
}

/// Official road-edge box: 1.5 m before the edge, none past the edge.
#[must_use]
pub fn evaluate_tsukuba_road_edge_stop(
    min_along_m: f64,
    max_along_m: f64,
    edge_m: f64,
) -> TsukubaStopJudgement {
    evaluate_stop_interval(
        min_along_m,
        max_along_m,
        edge_m - TSUKUBA_ROAD_EDGE_BEFORE_M,
        edge_m,
    )
}

fn evaluate_stop_interval(
    min_along_m: f64,
    max_along_m: f64,
    region_min_m: f64,
    region_max_m: f64,
) -> TsukubaStopJudgement {
    TsukubaStopJudgement {
        in_region: intervals_overlap(min_along_m, max_along_m, region_min_m, region_max_m),
        overshoot: max_along_m > region_max_m,
    }
}

fn intervals_overlap(a_min: f64, a_max: f64, b_min: f64, b_max: f64) -> bool {
    a_min < b_max && b_min < a_max
}

/// Scaled linear analog of the confirmation-run geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TsukubaConfirmationCourse {
    /// Sidewalk half-width in meters; `|z|` beyond this is roadway.
    pub sidewalk_half_width_m: f64,
    /// Two road-edge X positions in meters, travel along +X.
    pub road_edges_x_m: [f64; 2],
    /// Green-cone center X in meters.
    pub cone_x_m: f64,
    /// Green-cone center Z in meters.
    pub cone_z_m: f64,
    /// Green-cone half-extent along X in meters.
    pub cone_half_x_m: f64,
    /// Green-cone half-extent along Z in meters.
    pub cone_half_z_m: f64,
    /// Confirmation-end marker X in meters.
    pub confirmation_end_x_m: f64,
    /// Robot base half-length along X in meters.
    pub robot_half_x_m: f64,
    /// Robot base half-width along Z in meters.
    pub robot_half_z_m: f64,
    /// Consecutive stopped steps required to count a temporary stop.
    pub required_stop_steps: u32,
}

impl Default for TsukubaConfirmationCourse {
    fn default() -> Self {
        Self {
            sidewalk_half_width_m: 1.0,
            road_edges_x_m: [2.2, 4.4],
            cone_x_m: 6.0,
            cone_z_m: 0.0,
            cone_half_x_m: 0.12,
            cone_half_z_m: 0.12,
            confirmation_end_x_m: 7.2,
            robot_half_x_m: 0.25,
            robot_half_z_m: 0.2,
            required_stop_steps: 12,
        }
    }
}

impl TsukubaConfirmationCourse {
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

    /// Footprint of the green cone in the ground plane.
    #[must_use]
    pub fn cone_aabb(self) -> TsukubaPlanarAabb {
        TsukubaPlanarAabb {
            min_x_m: self.cone_x_m - self.cone_half_x_m,
            max_x_m: self.cone_x_m + self.cone_half_x_m,
            min_z_m: self.cone_z_m - self.cone_half_z_m,
            max_z_m: self.cone_z_m + self.cone_half_z_m,
        }
    }
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

/// Returns the bundled confirmation-run scene path.
#[must_use]
pub fn tsukuba_confirmation_scene_path() -> PathBuf {
    bundled_asset_path(Path::new("scenes/tsukuba_confirmation.rne.scene.toml"))
}

/// Portable TaskSpec for the confirmation-run analog.
#[must_use]
pub fn tsukuba_confirmation_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        TSUKUBA_CONFIRMATION_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("lidar_point_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("in_roadway", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("cone_contact", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("first_edge_stop_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("second_edge_stop_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("e_stop_asserted", TensorDType::F64, vec![], "1")
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
            RewardTermSpec::new("confirmation_complete", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("confirmation_complete", TerminationKind::Success),
                TerminationConditionSpec::new("cone_contact", TerminationKind::Failure),
                TerminationConditionSpec::new("unstopped_edge_overshoot", TerminationKind::Failure),
                TerminationConditionSpec::new("roadway_entry", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Injected confirmation-run faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TsukubaConfirmationFault {
    /// Scripted policy that satisfies the official geometric checklist.
    #[default]
    None,
    /// Drive through both road edges without a temporary stop.
    IgnoreStops,
    /// Stop at both edges, then ram the green cone.
    HitCone,
}

/// Headless observation consumed by confirmation-run behavior contracts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TsukubaConfirmationObservation {
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
    /// Left wheel command in radians per second.
    pub left_wheel_velocity_rad_s: f64,
    /// Right wheel command in radians per second.
    pub right_wheel_velocity_rad_s: f64,
    /// Latest LiDAR point count.
    pub lidar_points: usize,
    /// True when the body is outside the sidewalk.
    pub in_roadway: bool,
    /// True when the body overlaps or physically contacts the green cone.
    pub cone_contact: bool,
    /// First official road-edge stop completed.
    pub first_edge_stop_complete: bool,
    /// Second official road-edge stop completed.
    pub second_edge_stop_complete: bool,
    /// Passed a road edge without first completing its stop.
    pub unstopped_edge_overshoot: bool,
    /// Stopped short of the cone or cleared it without contact.
    pub cone_handled: bool,
    /// Operator e-stop has been asserted.
    pub e_stop_asserted: bool,
    /// Wheel commands and planar speed are at rest.
    pub stopped: bool,
    /// Base X is past the confirmation-end marker.
    pub past_confirmation_end: bool,
    /// Both edge stops, cone handling, and e-stop rest are complete.
    pub confirmation_complete: bool,
}

/// Headless confirmation-run analog driven by a scripted sidewalk policy.
pub struct TsukubaConfirmationScenario {
    sim: DiffDriveSim,
    course: TsukubaConfirmationCourse,
    fault: TsukubaConfirmationFault,
    max_steps: u64,
    phase: ScriptPhase,
    phase_ticks: u32,
    e_stop_asserted: bool,
    edge_stop_streak: [u32; 2],
    edge_stop_complete: [bool; 2],
    unstopped_edge_overshoot: bool,
    cone_contact: bool,
    cone_handled: bool,
    observation: TsukubaConfirmationObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for TsukubaConfirmationScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("TsukubaConfirmationScenario")
            .field("fault", &self.fault)
            .field("max_steps", &self.max_steps)
            .field("phase", &self.phase)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScriptPhase {
    ApproachEdge(usize),
    HoldEdge(usize),
    OperatorEdge(usize),
    CrossEdge(usize),
    ApproachCone,
    HoldCone,
    OperatorCone,
    EmergencyStop,
    Halt,
}

impl TsukubaConfirmationScenario {
    /// Loads the bundled confirmation scene for a successful scripted run.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, TsukubaConfirmationFault::None)
    }

    /// Loads the bundled confirmation scene with an injected fault.
    pub fn new(seed: u64, fault: TsukubaConfirmationFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = tsukuba_confirmation_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let course = TsukubaConfirmationCourse::default();
        let mut scenario = Self {
            sim,
            course,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            phase: ScriptPhase::ApproachEdge(0),
            phase_ticks: 0,
            e_stop_asserted: false,
            edge_stop_streak: [0; 2],
            edge_stop_complete: [false; 2],
            unstopped_edge_overshoot: false,
            cone_contact: false,
            cone_handled: false,
            observation: placeholder_observation(),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current confirmation-run observation.
    #[must_use]
    pub fn current_observation(&self) -> TsukubaConfirmationObservation {
        self.observation
    }

    fn observe_world(&self) -> TsukubaConfirmationObservation {
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
        let cone = self.course.cone_aabb();
        let geometry_cone_contact = aabb.overlaps(cone);
        let physics_cone_contact = robot_contacts_named(&self.sim, spawned, CONE_NAME);
        let cone_contact = self.cone_contact || geometry_cone_contact || physics_cone_contact;
        let in_roadway = aabb.max_z_m > self.course.sidewalk_half_width_m
            || aabb.min_z_m < -self.course.sidewalk_half_width_m;
        let past_confirmation_end = drive.base_x_m >= self.course.confirmation_end_x_m;
        let confirmation_complete = self.edge_stop_complete[0]
            && self.edge_stop_complete[1]
            && self.cone_handled
            && self.e_stop_asserted
            && stopped;

        TsukubaConfirmationObservation {
            step: self.sim.step_count(),
            base_x_m: drive.base_x_m,
            base_y_m: drive.base_y_m,
            base_z_m: drive.base_z_m,
            base_yaw_rad: drive.base_yaw_rad,
            speed_m_s,
            left_wheel_velocity_rad_s: drive.left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s: drive.right_wheel_velocity_rad_s,
            lidar_points: drive.lidar_points,
            in_roadway,
            cone_contact,
            first_edge_stop_complete: self.edge_stop_complete[0],
            second_edge_stop_complete: self.edge_stop_complete[1],
            unstopped_edge_overshoot: self.unstopped_edge_overshoot,
            cone_handled: self.cone_handled,
            e_stop_asserted: self.e_stop_asserted,
            stopped,
            past_confirmation_end,
            confirmation_complete,
        }
    }

    fn update_judge(&mut self, observation: TsukubaConfirmationObservation) {
        let aabb = self.course.robot_aabb(
            observation.base_x_m,
            observation.base_z_m,
            observation.base_yaw_rad,
        );
        self.cone_contact = observation.cone_contact;
        for (index, edge_x_m) in self.course.road_edges_x_m.iter().copied().enumerate() {
            if self.edge_stop_complete[index] {
                continue;
            }
            let judgement = evaluate_tsukuba_road_edge_stop(aabb.min_x_m, aabb.max_x_m, edge_x_m);
            if judgement.overshoot {
                self.unstopped_edge_overshoot = true;
            }
            if judgement.valid() && observation.stopped {
                self.edge_stop_streak[index] = self.edge_stop_streak[index]
                    .saturating_add(1)
                    .min(self.course.required_stop_steps);
                if self.edge_stop_streak[index] >= self.course.required_stop_steps {
                    self.edge_stop_complete[index] = true;
                }
            } else {
                self.edge_stop_streak[index] = 0;
            }
        }
        let cone = self.course.cone_aabb();
        let stopped_in_front = observation.stopped
            && !observation.cone_contact
            && aabb.max_x_m < cone.min_x_m
            && aabb.min_x_m > cone.min_x_m - 2.0;
        let avoided = !observation.cone_contact && aabb.min_x_m > cone.max_x_m;
        if stopped_in_front || avoided {
            self.cone_handled = true;
        }
    }

    fn action(&mut self, observation: TsukubaConfirmationObservation) -> DiffDriveAction {
        if matches!(self.fault, TsukubaConfirmationFault::IgnoreStops) {
            return DiffDriveAction::forward(CRUISE_WHEEL_RAD_S);
        }

        match self.phase {
            ScriptPhase::ApproachEdge(index) => {
                let stop_x_m = self.course.road_edges_x_m[index] - self.course.robot_half_x_m - 0.2;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldEdge(index);
                    self.phase_ticks = 0;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldEdge(index) => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.edge_stop_complete[index] {
                    self.phase = ScriptPhase::OperatorEdge(index);
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::OperatorEdge(index) => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= OPERATOR_HOLD_STEPS {
                    self.phase = ScriptPhase::CrossEdge(index);
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::CrossEdge(index) => {
                if observation.base_x_m > self.course.road_edges_x_m[index] + 0.45 {
                    self.phase = if index == 0 {
                        ScriptPhase::ApproachEdge(1)
                    } else {
                        ScriptPhase::ApproachCone
                    };
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
            }
            ScriptPhase::ApproachCone => {
                if matches!(self.fault, TsukubaConfirmationFault::HitCone) {
                    return DiffDriveAction::forward(CRUISE_WHEEL_RAD_S);
                }
                let stop_x_m = self.course.cone_x_m
                    - self.course.cone_half_x_m
                    - self.course.robot_half_x_m
                    - 0.25;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldCone;
                    self.phase_ticks = 0;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldCone => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.cone_handled {
                    self.phase = ScriptPhase::OperatorCone;
                    self.phase_ticks = 0;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::OperatorCone => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= OPERATOR_HOLD_STEPS {
                    self.phase = ScriptPhase::EmergencyStop;
                    self.phase_ticks = 0;
                    self.e_stop_asserted = true;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::EmergencyStop => {
                self.e_stop_asserted = true;
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if observation.stopped && self.phase_ticks >= self.course.required_stop_steps {
                    self.phase = ScriptPhase::Halt;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::Halt => {
                self.e_stop_asserted = true;
                DiffDriveAction::forward(0.0)
            }
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

impl BehaviorScenario for TsukubaConfirmationScenario {
    type Observation = TsukubaConfirmationObservation;

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
        let mut bytes = b"tsukuba_confirmation_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            TsukubaConfirmationFault::None => 0,
            TsukubaConfirmationFault::IgnoreStops => 1,
            TsukubaConfirmationFault::HitCone => 2,
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
                "no_roadway_entry",
                |observation: &TsukubaConfirmationObservation| !observation.in_roadway,
            )?
            .with_entities(["tsukuba_confirmation"])?,
            BehaviorContract::always(
                "no_cone_contact",
                |observation: &TsukubaConfirmationObservation| !observation.cone_contact,
            )?
            .with_entities([CONE_NAME])?,
            BehaviorContract::always(
                "no_unstopped_edge_overshoot",
                |observation: &TsukubaConfirmationObservation| {
                    !observation.unstopped_edge_overshoot
                },
            )?
            .with_entities(["road_edge_1", "road_edge_2"])?,
            BehaviorContract::eventually(
                "first_road_edge_stop",
                deadline,
                |observation: &TsukubaConfirmationObservation| observation.first_edge_stop_complete,
            )?
            .with_entities(["road_edge_1"])?,
            BehaviorContract::eventually(
                "second_road_edge_stop",
                deadline,
                |observation: &TsukubaConfirmationObservation| {
                    observation.second_edge_stop_complete
                },
            )?
            .with_entities(["road_edge_2"])?,
            BehaviorContract::eventually(
                "confirmation_complete",
                deadline,
                |observation: &TsukubaConfirmationObservation| observation.confirmation_complete,
            )?
            .with_entities(["confirmation_end"])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let action = self.action(self.observation);
        self.sim.step_action(action);
        let mut observation = self.observe_world();
        self.update_judge(observation);
        observation = self.observe_world();
        self.observation = observation;
        let done = observation.confirmation_complete
            || observation.cone_contact
            || observation.unstopped_edge_overshoot
            || observation.in_roadway
            || observation.step >= self.max_steps;
        BehaviorScenarioStep { observation, done }
    }
}

fn placeholder_observation() -> TsukubaConfirmationObservation {
    TsukubaConfirmationObservation {
        step: 0,
        base_x_m: 0.0,
        base_y_m: 0.0,
        base_z_m: 0.0,
        base_yaw_rad: 0.0,
        speed_m_s: 0.0,
        left_wheel_velocity_rad_s: 0.0,
        right_wheel_velocity_rad_s: 0.0,
        lidar_points: 0,
        in_roadway: false,
        cone_contact: false,
        first_edge_stop_complete: false,
        second_edge_stop_complete: false,
        unstopped_edge_overshoot: false,
        cone_handled: false,
        e_stop_asserted: false,
        stopped: true,
        past_confirmation_end: false,
        confirmation_complete: false,
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

fn entity_named(world: &World, name: &str) -> Option<Entity> {
    world.iter_entities().find_map(|entity_ref| {
        world
            .get::<Name>(entity_ref.id())
            .is_some_and(|entity_name| entity_name.0 == name)
            .then_some(entity_ref.id())
    })
}

fn robot_contacts_named(sim: &DiffDriveSim, spawned: &DiffDriveSpawned, name: &str) -> bool {
    let Some(target) = entity_named(sim.world(), name) else {
        return false;
    };
    let bodies = [spawned.base_link, spawned.left_wheel, spawned.right_wheel];
    sim.last_contacts().iter().any(|contact| {
        (bodies.contains(&contact.entity_a) && contact.entity_b == target)
            || (bodies.contains(&contact.entity_b) && contact.entity_a == target)
    })
}

fn fault_dimensions(fault: TsukubaConfirmationFault) -> Vec<BehaviorDimension> {
    vec![
        BehaviorDimension {
            name: "hit_cone".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                TsukubaConfirmationFault::HitCone
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
        BehaviorDimension {
            name: "ignore_stops".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                TsukubaConfirmationFault::IgnoreStops
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
    ]
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
    fn stop_line_matches_official_before_and_after_bounds() {
        let inside = evaluate_tsukuba_stop_line(9.2, 9.7, 10.0);
        assert!(inside.valid());

        let overshoot = evaluate_tsukuba_stop_line(10.4, 10.7, 10.0);
        assert!(overshoot.in_region);
        assert!(overshoot.overshoot);
        assert!(!overshoot.valid());

        let too_far_before = evaluate_tsukuba_stop_line(8.0, 8.4, 10.0);
        assert!(!too_far_before.in_region);
        assert!(!too_far_before.overshoot);
    }

    #[test]
    fn road_edge_matches_official_one_point_five_meter_box() {
        let inside = evaluate_tsukuba_road_edge_stop(3.6, 4.1, 5.0);
        assert!(inside.valid());

        let past_edge = evaluate_tsukuba_road_edge_stop(4.8, 5.2, 5.0);
        assert!(past_edge.overshoot);
        assert!(!past_edge.valid());

        let too_far_before = evaluate_tsukuba_road_edge_stop(2.0, 2.4, 5.0);
        assert!(!too_far_before.in_region);
        assert!(!too_far_before.overshoot);
    }

    #[test]
    fn confirmation_task_spec_validates() {
        let spec = tsukuba_confirmation_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let path = crate::asset_path::bundled_asset_path(Path::new(
            "tasks/tsukuba_confirmation.task.json",
        ));
        let loaded: TaskSpec =
            serde_json::from_slice(&std::fs::read(path).expect("committed task spec"))
                .expect("parse task spec");
        assert_eq!(spec, loaded);
    }

    #[test]
    fn scripted_confirmation_run_passes_official_geometry() {
        let report = run_behavior_scenarios(
            "tsukuba_confirmation_success",
            [1],
            TsukubaConfirmationScenario::success,
        );
        assert!(report.passed(), "{report:?}");
        assert!(report.seeds[0].steps > 100);
    }

    #[test]
    fn cone_contact_fails_the_official_obstacle_contract() {
        let report = run_behavior_scenarios("tsukuba_confirmation_hit_cone", [1], |seed| {
            TsukubaConfirmationScenario::new(seed, TsukubaConfirmationFault::HitCone)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let cone = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_cone_contact")
            .expect("cone contract");
        assert_eq!(cone.status, BehaviorContractStatus::Failed);
    }

    #[test]
    fn ignored_edge_stops_fail_the_official_overshoot_contract() {
        let report = run_behavior_scenarios("tsukuba_confirmation_ignore_stops", [1], |seed| {
            TsukubaConfirmationScenario::new(seed, TsukubaConfirmationFault::IgnoreStops)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let overshoot = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_unstopped_edge_overshoot")
            .expect("overshoot contract");
        assert_eq!(overshoot.status, BehaviorContractStatus::Failed);
    }
}
