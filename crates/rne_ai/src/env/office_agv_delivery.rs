//! Office AGV dock-to-desk delivery scored on an analytic corridor.
//!
//! This is not a warehouse twin or Nav2 port. The first slice is a few-meter
//! office aisle: visit the pickup dock, then stop in the delivery box in front
//! of the desk, without leaving the corridor.

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

/// Portable task identity for the office AGV delivery analog.
pub const OFFICE_AGV_DELIVERY_TASK_ID: &str = "rne.office.agv_delivery.v1";
/// Look-ahead before the desk face used by the delivery stop box.
pub const OFFICE_DESK_DELIVERY_BEFORE_M: f64 = 1.2;
/// Scene entity name of the pickup dock pad.
pub const OFFICE_PICKUP_DOCK_NAME: &str = "pickup_dock";
/// Scene entity name of the delivery desk.
pub const OFFICE_DELIVERY_DESK_NAME: &str = "delivery_desk";

const CONTROL_HZ: f64 = 60.0;
const CRUISE_WHEEL_RAD_S: f64 = 5.0;
const TURN_WHEEL_RAD_S: f64 = 4.0;
const STOPPED_SPEED_M_S: f64 = 0.05;
const STOPPED_WHEEL_RAD_S: f64 = 0.2;
const DEFAULT_MAX_STEPS: u64 = 1_200;

/// Axis-aligned footprint used by the corridor and stop judges.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OfficePlanarAabb {
    /// Minimum world X in meters.
    pub min_x_m: f64,
    /// Maximum world X in meters.
    pub max_x_m: f64,
    /// Minimum world Z in meters.
    pub min_z_m: f64,
    /// Maximum world Z in meters.
    pub max_z_m: f64,
}

impl OfficePlanarAabb {
    /// Returns whether two footprints overlap in the ground plane.
    #[must_use]
    pub fn overlaps(self, other: Self) -> bool {
        intervals_overlap(self.min_x_m, self.max_x_m, other.min_x_m, other.max_x_m)
            && intervals_overlap(self.min_z_m, self.max_z_m, other.min_z_m, other.max_z_m)
    }
}

/// Result of one delivery-stop-region evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfficeStopJudgement {
    /// Some part of the body intersects the allowed stop box.
    pub in_region: bool,
    /// Some part of the body is past the allowed far bound.
    pub overshoot: bool,
}

impl OfficeStopJudgement {
    /// Valid delivery stop: inside the box and not past the desk face.
    #[must_use]
    pub const fn valid(self) -> bool {
        self.in_region && !self.overshoot
    }
}

/// Delivery stop box: [`OFFICE_DESK_DELIVERY_BEFORE_M`] before the desk face,
/// none past the face.
#[must_use]
pub fn evaluate_office_desk_delivery_stop(
    min_along_m: f64,
    max_along_m: f64,
    desk_face_x_m: f64,
) -> OfficeStopJudgement {
    evaluate_stop_interval(
        min_along_m,
        max_along_m,
        desk_face_x_m - OFFICE_DESK_DELIVERY_BEFORE_M,
        desk_face_x_m,
    )
}

fn evaluate_stop_interval(
    min_along_m: f64,
    max_along_m: f64,
    region_min_m: f64,
    region_max_m: f64,
) -> OfficeStopJudgement {
    OfficeStopJudgement {
        in_region: intervals_overlap(min_along_m, max_along_m, region_min_m, region_max_m),
        overshoot: max_along_m > region_max_m,
    }
}

fn intervals_overlap(a_min: f64, a_max: f64, b_min: f64, b_max: f64) -> bool {
    a_min < b_max && b_min < a_max
}

/// Analytic office aisle used by the delivery judges.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OfficeAgvDeliveryCourse {
    /// Corridor half-width in meters; `|z|` beyond this leaves the aisle.
    pub corridor_half_width_m: f64,
    /// Pickup-dock center X in meters.
    pub dock_x_m: f64,
    /// Pickup-dock center Z in meters.
    pub dock_z_m: f64,
    /// Pickup-dock half-extent along X in meters.
    pub dock_half_x_m: f64,
    /// Pickup-dock half-extent along Z in meters.
    pub dock_half_z_m: f64,
    /// Delivery-desk face X in meters (travel along +X).
    pub desk_face_x_m: f64,
    /// Robot base half-length along X in meters.
    pub robot_half_x_m: f64,
    /// Robot base half-width along Z in meters.
    pub robot_half_z_m: f64,
    /// Consecutive stopped steps required to count a delivery stop.
    pub required_stop_steps: u32,
    /// Consecutive dock-overlap steps required to count a pickup.
    pub required_dock_steps: u32,
}

impl Default for OfficeAgvDeliveryCourse {
    fn default() -> Self {
        Self {
            corridor_half_width_m: 1.0,
            dock_x_m: 2.5,
            dock_z_m: 0.0,
            dock_half_x_m: 0.4,
            dock_half_z_m: 0.6,
            desk_face_x_m: 7.1,
            robot_half_x_m: 0.25,
            robot_half_z_m: 0.2,
            required_stop_steps: 12,
            required_dock_steps: 6,
        }
    }
}

impl OfficeAgvDeliveryCourse {
    /// Footprint of the robot base in the ground plane.
    #[must_use]
    pub fn robot_aabb(self, center_x_m: f64, center_z_m: f64, yaw_rad: f64) -> OfficePlanarAabb {
        planar_aabb(
            center_x_m,
            center_z_m,
            yaw_rad,
            self.robot_half_x_m,
            self.robot_half_z_m,
        )
    }

    /// Footprint of the pickup dock in the ground plane.
    #[must_use]
    pub fn dock_aabb(self) -> OfficePlanarAabb {
        OfficePlanarAabb {
            min_x_m: self.dock_x_m - self.dock_half_x_m,
            max_x_m: self.dock_x_m + self.dock_half_x_m,
            min_z_m: self.dock_z_m - self.dock_half_z_m,
            max_z_m: self.dock_z_m + self.dock_half_z_m,
        }
    }
}

fn planar_aabb(
    center_x_m: f64,
    center_z_m: f64,
    yaw_rad: f64,
    half_x_m: f64,
    half_z_m: f64,
) -> OfficePlanarAabb {
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
    OfficePlanarAabb {
        min_x_m,
        max_x_m,
        min_z_m,
        max_z_m,
    }
}

/// Returns the bundled office AGV delivery scene path.
#[must_use]
pub fn office_agv_delivery_scene_path() -> PathBuf {
    bundled_asset_path(Path::new("scenes/office_agv_delivery.rne.scene.toml"))
}

/// Portable TaskSpec for the office AGV delivery analog.
#[must_use]
pub fn office_agv_delivery_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        OFFICE_AGV_DELIVERY_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("lidar_point_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("out_of_corridor", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("dock_pickup_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("desk_delivery_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("desk_without_pickup", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("desk_overshoot", TensorDType::F64, vec![], "1")
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
            RewardTermSpec::new("delivery_complete", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("delivery_complete", TerminationKind::Success),
                TerminationConditionSpec::new("out_of_corridor", TerminationKind::Failure),
                TerminationConditionSpec::new("desk_without_pickup", TerminationKind::Failure),
                TerminationConditionSpec::new("desk_overshoot", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Injected office AGV delivery faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OfficeAgvDeliveryFault {
    /// Scripted policy that visits the dock then stops at the desk.
    #[default]
    None,
    /// Skip the dock and drive into the desk stop region without pickup.
    SkipDock,
    /// Turn out of the corridor aisle.
    LeaveCorridor,
}

/// Headless observation consumed by office AGV behavior contracts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OfficeAgvDeliveryObservation {
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
    /// True when the body leaves the corridor half-width.
    pub out_of_corridor: bool,
    /// Pickup dock visit completed.
    pub dock_pickup_complete: bool,
    /// Desk delivery stop completed after pickup.
    pub desk_delivery_complete: bool,
    /// Entered or passed the desk region without a prior pickup.
    pub desk_without_pickup: bool,
    /// Passed the desk face without a completed delivery stop.
    pub desk_overshoot: bool,
    /// Wheel commands and planar speed are at rest.
    pub stopped: bool,
    /// Dock pickup and desk delivery are both complete while stopped.
    pub delivery_complete: bool,
}

/// Headless office AGV delivery driven by a scripted corridor policy.
pub struct OfficeAgvDeliveryScenario {
    sim: DiffDriveSim,
    course: OfficeAgvDeliveryCourse,
    fault: OfficeAgvDeliveryFault,
    max_steps: u64,
    phase: ScriptPhase,
    dock_overlap_streak: u32,
    dock_pickup_complete: bool,
    desk_stop_streak: u32,
    desk_delivery_complete: bool,
    desk_without_pickup: bool,
    desk_overshoot: bool,
    observation: OfficeAgvDeliveryObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for OfficeAgvDeliveryScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("OfficeAgvDeliveryScenario")
            .field("fault", &self.fault)
            .field("max_steps", &self.max_steps)
            .field("phase", &self.phase)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScriptPhase {
    ApproachDock,
    HoldDock,
    ApproachDesk,
    HoldDesk,
    LeaveCorridor,
    Halt,
}

impl OfficeAgvDeliveryScenario {
    /// Loads the bundled office scene for a successful scripted run.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, OfficeAgvDeliveryFault::None)
    }

    /// Loads the bundled office scene with an injected fault.
    pub fn new(seed: u64, fault: OfficeAgvDeliveryFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = office_agv_delivery_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let course = OfficeAgvDeliveryCourse::default();
        let phase = match fault {
            OfficeAgvDeliveryFault::LeaveCorridor => ScriptPhase::LeaveCorridor,
            _ => ScriptPhase::ApproachDock,
        };
        let mut scenario = Self {
            sim,
            course,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            phase,
            dock_overlap_streak: 0,
            dock_pickup_complete: false,
            desk_stop_streak: 0,
            desk_delivery_complete: false,
            desk_without_pickup: false,
            desk_overshoot: false,
            observation: placeholder_observation(),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current office AGV observation.
    #[must_use]
    pub fn current_observation(&self) -> OfficeAgvDeliveryObservation {
        self.observation
    }

    fn observe_world(&self) -> OfficeAgvDeliveryObservation {
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
        let out_of_corridor = aabb.max_z_m > self.course.corridor_half_width_m
            || aabb.min_z_m < -self.course.corridor_half_width_m;
        let delivery_complete = self.dock_pickup_complete && self.desk_delivery_complete && stopped;

        OfficeAgvDeliveryObservation {
            step: self.sim.step_count(),
            base_x_m: drive.base_x_m,
            base_y_m: drive.base_y_m,
            base_z_m: drive.base_z_m,
            base_yaw_rad: drive.base_yaw_rad,
            speed_m_s,
            left_wheel_velocity_rad_s: drive.left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s: drive.right_wheel_velocity_rad_s,
            lidar_points: drive.lidar_points,
            out_of_corridor,
            dock_pickup_complete: self.dock_pickup_complete,
            desk_delivery_complete: self.desk_delivery_complete,
            desk_without_pickup: self.desk_without_pickup,
            desk_overshoot: self.desk_overshoot,
            stopped,
            delivery_complete,
        }
    }

    fn update_judge(&mut self, observation: OfficeAgvDeliveryObservation) {
        let aabb = self.course.robot_aabb(
            observation.base_x_m,
            observation.base_z_m,
            observation.base_yaw_rad,
        );
        let dock = self.course.dock_aabb();
        let geometry_dock = aabb.overlaps(dock);
        let physics_dock =
            robot_contacts_named(&self.sim, &self.sim.robots()[0], OFFICE_PICKUP_DOCK_NAME);
        if !self.dock_pickup_complete {
            if observation.stopped && (geometry_dock || physics_dock) {
                self.dock_overlap_streak = self
                    .dock_overlap_streak
                    .saturating_add(1)
                    .min(self.course.required_dock_steps);
                if self.dock_overlap_streak >= self.course.required_dock_steps {
                    self.dock_pickup_complete = true;
                }
            } else {
                self.dock_overlap_streak = 0;
            }
        }

        let judgement = evaluate_office_desk_delivery_stop(
            aabb.min_x_m,
            aabb.max_x_m,
            self.course.desk_face_x_m,
        );
        if (judgement.in_region || judgement.overshoot) && !self.dock_pickup_complete {
            self.desk_without_pickup = true;
        }
        if !self.desk_delivery_complete {
            if judgement.overshoot {
                self.desk_overshoot = true;
            }
            if self.dock_pickup_complete && judgement.valid() && observation.stopped {
                self.desk_stop_streak = self
                    .desk_stop_streak
                    .saturating_add(1)
                    .min(self.course.required_stop_steps);
                if self.desk_stop_streak >= self.course.required_stop_steps {
                    self.desk_delivery_complete = true;
                }
            } else if !self.desk_delivery_complete {
                self.desk_stop_streak = 0;
            }
        }
    }

    fn action(&mut self, observation: OfficeAgvDeliveryObservation) -> DiffDriveAction {
        match self.phase {
            ScriptPhase::LeaveCorridor => DiffDriveAction {
                left_velocity_rad_s: CRUISE_WHEEL_RAD_S,
                right_velocity_rad_s: TURN_WHEEL_RAD_S,
            },
            ScriptPhase::ApproachDock => {
                if matches!(self.fault, OfficeAgvDeliveryFault::SkipDock) {
                    if observation.base_x_m >= self.course.desk_face_x_m + 0.4 {
                        self.phase = ScriptPhase::Halt;
                    }
                    return DiffDriveAction::forward(CRUISE_WHEEL_RAD_S);
                }
                let stop_x_m = self.course.dock_x_m;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldDock;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldDock => {
                if self.dock_pickup_complete {
                    self.phase = ScriptPhase::ApproachDesk;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::ApproachDesk => {
                let stop_x_m =
                    self.course.desk_face_x_m - self.course.robot_half_x_m - 0.35;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldDesk;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldDesk => {
                if self.desk_delivery_complete {
                    self.phase = ScriptPhase::Halt;
                }
                DiffDriveAction::forward(0.0)
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

impl BehaviorScenario for OfficeAgvDeliveryScenario {
    type Observation = OfficeAgvDeliveryObservation;

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
        let mut bytes = b"office_agv_delivery_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            OfficeAgvDeliveryFault::None => 0,
            OfficeAgvDeliveryFault::SkipDock => 1,
            OfficeAgvDeliveryFault::LeaveCorridor => 2,
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
                "no_corridor_exit",
                |observation: &OfficeAgvDeliveryObservation| !observation.out_of_corridor,
            )?
            .with_entities(["office_agv_delivery"])?,
            BehaviorContract::always(
                "no_desk_without_pickup",
                |observation: &OfficeAgvDeliveryObservation| !observation.desk_without_pickup,
            )?
            .with_entities([OFFICE_PICKUP_DOCK_NAME, OFFICE_DELIVERY_DESK_NAME])?,
            BehaviorContract::always(
                "no_desk_overshoot",
                |observation: &OfficeAgvDeliveryObservation| !observation.desk_overshoot,
            )?
            .with_entities([OFFICE_DELIVERY_DESK_NAME])?,
            BehaviorContract::eventually(
                "dock_pickup_complete",
                deadline,
                |observation: &OfficeAgvDeliveryObservation| observation.dock_pickup_complete,
            )?
            .with_entities([OFFICE_PICKUP_DOCK_NAME])?,
            BehaviorContract::eventually(
                "delivery_complete",
                deadline,
                |observation: &OfficeAgvDeliveryObservation| observation.delivery_complete,
            )?
            .with_entities([OFFICE_DELIVERY_DESK_NAME])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let action = self.action(self.observation);
        self.sim.step_action(action);
        let mut observation = self.observe_world();
        self.update_judge(observation);
        observation = self.observe_world();
        self.observation = observation;
        let done = observation.delivery_complete
            || observation.out_of_corridor
            || observation.desk_without_pickup
            || observation.desk_overshoot
            || observation.step >= self.max_steps;
        BehaviorScenarioStep { observation, done }
    }
}

fn placeholder_observation() -> OfficeAgvDeliveryObservation {
    OfficeAgvDeliveryObservation {
        step: 0,
        base_x_m: 0.0,
        base_y_m: 0.0,
        base_z_m: 0.0,
        base_yaw_rad: 0.0,
        speed_m_s: 0.0,
        left_wheel_velocity_rad_s: 0.0,
        right_wheel_velocity_rad_s: 0.0,
        lidar_points: 0,
        out_of_corridor: false,
        dock_pickup_complete: false,
        desk_delivery_complete: false,
        desk_without_pickup: false,
        desk_overshoot: false,
        stopped: true,
        delivery_complete: false,
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

fn fault_dimensions(fault: OfficeAgvDeliveryFault) -> Vec<BehaviorDimension> {
    vec![
        BehaviorDimension {
            name: "skip_dock".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                OfficeAgvDeliveryFault::SkipDock
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
        BehaviorDimension {
            name: "leave_corridor".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                OfficeAgvDeliveryFault::LeaveCorridor
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
    fn desk_delivery_stop_matches_before_face_box() {
        let inside = evaluate_office_desk_delivery_stop(5.9, 6.4, 7.1);
        assert!(inside.valid());

        let past_face = evaluate_office_desk_delivery_stop(7.0, 7.4, 7.1);
        assert!(past_face.overshoot);
        assert!(!past_face.valid());

        let too_far_before = evaluate_office_desk_delivery_stop(4.0, 4.5, 7.1);
        assert!(!too_far_before.in_region);
        assert!(!too_far_before.overshoot);
    }

    #[test]
    fn office_delivery_task_spec_validates() {
        let spec = office_agv_delivery_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let path = crate::asset_path::bundled_asset_path(Path::new(
            "tasks/office_agv_delivery.task.json",
        ));
        let loaded: TaskSpec =
            serde_json::from_slice(&std::fs::read(path).expect("committed task spec"))
                .expect("parse task spec");
        assert_eq!(spec, loaded);
    }

    #[test]
    fn scripted_office_delivery_passes_geometry() {
        let report = run_behavior_scenarios(
            "office_agv_delivery_success",
            [1],
            OfficeAgvDeliveryScenario::success,
        );
        assert!(report.passed(), "{report:?}");
        assert!(report.seeds[0].steps > 50);
    }

    #[test]
    fn skip_dock_fails_pickup_before_desk_contract() {
        let report = run_behavior_scenarios("office_agv_delivery_skip_dock", [1], |seed| {
            OfficeAgvDeliveryScenario::new(seed, OfficeAgvDeliveryFault::SkipDock)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let without_pickup = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_desk_without_pickup")
            .expect("desk without pickup contract");
        assert_eq!(without_pickup.status, BehaviorContractStatus::Failed);
    }

    #[test]
    fn leave_corridor_fails_aisle_contract() {
        let report = run_behavior_scenarios("office_agv_delivery_leave_corridor", [1], |seed| {
            OfficeAgvDeliveryScenario::new(seed, OfficeAgvDeliveryFault::LeaveCorridor)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let corridor = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_corridor_exit")
            .expect("corridor contract");
        assert_eq!(corridor.status, BehaviorContractStatus::Failed);
    }
}
