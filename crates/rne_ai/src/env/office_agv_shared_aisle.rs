//! Office AGV shared-aisle delivery: yield to an oncoming AGV, then deliver.
//!
//! Extends the dock-to-desk geometric checklist with a scripted opposing AGV.
//! This is not `rne_traffic` co-simulation; the other actor is a kinematic
//! planar footprint so judges stay headless and renderer-free.

use crate::action::DiffDriveAction;
use crate::asset_path::bundled_asset_path;
use crate::behavior::{
    BehaviorContract, BehaviorContractError, BehaviorScenario, BehaviorScenarioStep,
};
use crate::behavior_replay::{stable_behavior_digest, BehaviorDimension, BehaviorDimensionValue};
use crate::env::office_agv_delivery::{
    evaluate_office_desk_delivery_stop, office_agv_delivery_scene_path, OfficeAgvDeliveryCourse,
    OfficePlanarAabb, OFFICE_DELIVERY_DESK_NAME, OFFICE_PICKUP_DOCK_NAME,
};
use crate::env::DiffDriveSim;
use crate::task::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_assets::{load_scene_bundle, scene_dependency_paths, AssetError};
use rne_core::SimDuration;
use rne_ecs::{Entity, Name, World};
use rne_math::Vec3;
use rne_physics::{hash_physics_state, RigidBody};
use rne_robot::DiffDriveSpawned;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Portable task identity for the shared-aisle office AGV analog.
pub const OFFICE_AGV_SHARED_AISLE_TASK_ID: &str = "rne.office.agv_shared_aisle.v1";
/// Logical name for the scripted opposing AGV.
pub const OFFICE_ONCOMING_AGV_NAME: &str = "oncoming_agv";

const CONTROL_HZ: f64 = 60.0;
const CRUISE_WHEEL_RAD_S: f64 = 5.0;
const STOPPED_SPEED_M_S: f64 = 0.05;
const STOPPED_WHEEL_RAD_S: f64 = 0.2;
const DEFAULT_MAX_STEPS: u64 = 2_000;
const DT_S: f64 = 1.0 / CONTROL_HZ;

/// Shared-aisle geometry layered on the dock-to-desk course.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OfficeAgvSharedAisleCourse {
    /// Dock-to-desk corridor geometry.
    pub delivery: OfficeAgvDeliveryCourse,
    /// Shared-segment near bound along +X in meters.
    pub shared_min_x_m: f64,
    /// Shared-segment far bound along +X in meters.
    pub shared_max_x_m: f64,
    /// Oncoming AGV half-length along X in meters.
    pub other_half_x_m: f64,
    /// Oncoming AGV half-width along Z in meters.
    pub other_half_z_m: f64,
    /// Oncoming AGV start X in meters (travels toward −X).
    pub other_start_x_m: f64,
    /// Oncoming AGV constant speed in meters per second.
    pub other_speed_m_s: f64,
    /// Delay before the oncoming AGV starts moving, in seconds.
    pub other_departure_delay_s: f64,
    /// X below which the oncoming AGV has cleared the shared segment.
    pub other_clear_x_m: f64,
    /// Consecutive stopped yield steps required while the shared segment is busy.
    pub required_yield_steps: u32,
}

impl Default for OfficeAgvSharedAisleCourse {
    fn default() -> Self {
        Self {
            delivery: OfficeAgvDeliveryCourse::default(),
            shared_min_x_m: 3.5,
            shared_max_x_m: 5.5,
            other_half_x_m: 0.25,
            other_half_z_m: 0.2,
            other_start_x_m: 6.2,
            other_speed_m_s: 0.55,
            other_departure_delay_s: 4.5,
            other_clear_x_m: 7.2,
            required_yield_steps: 12,
        }
    }
}

impl OfficeAgvSharedAisleCourse {
    /// Footprint of the oncoming AGV in the ground plane.
    #[must_use]
    pub fn other_aabb(self, center_x_m: f64, center_z_m: f64) -> OfficePlanarAabb {
        OfficePlanarAabb {
            min_x_m: center_x_m - self.other_half_x_m,
            max_x_m: center_x_m + self.other_half_x_m,
            min_z_m: center_z_m - self.other_half_z_m,
            max_z_m: center_z_m + self.other_half_z_m,
        }
    }

    /// True when the oncoming AGV still blocks ego from entering the shared
    /// segment.
    #[must_use]
    pub fn other_blocks_ego(self, other_x_m: f64) -> bool {
        self.other_occupies_shared(other_x_m)
    }

    /// True when the oncoming footprint still intersects the shared segment.
    #[must_use]
    pub fn other_occupies_shared(self, other_x_m: f64) -> bool {
        let other = self.other_aabb(other_x_m, 0.0);
        other.max_x_m > self.shared_min_x_m && other.min_x_m < self.shared_max_x_m
    }
}

/// Returns whether ego must yield before entering the shared segment.
#[must_use]
pub fn evaluate_office_shared_aisle_block(
    ego_max_x_m: f64,
    other_occupies_shared: bool,
    shared_min_x_m: f64,
) -> bool {
    other_occupies_shared && ego_max_x_m >= shared_min_x_m
}

/// Portable TaskSpec for shared-aisle office delivery.
#[must_use]
pub fn office_agv_shared_aisle_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        OFFICE_AGV_SHARED_AISLE_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("other_agv_x_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("shared_aisle_occupied", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("yielded_for_shared_aisle", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("other_agv_contact", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
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
                TerminationConditionSpec::new("other_agv_contact", TerminationKind::Failure),
                TerminationConditionSpec::new("desk_without_pickup", TerminationKind::Failure),
                TerminationConditionSpec::new("desk_overshoot", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Injected shared-aisle faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OfficeAgvSharedAisleFault {
    /// Yield until the oncoming AGV clears, then complete dock-to-desk delivery.
    #[default]
    None,
    /// Drive into the occupied shared segment without yielding.
    IgnoreYield,
}

/// Headless observation for shared-aisle office delivery.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OfficeAgvSharedAisleObservation {
    /// Completed simulation steps.
    pub step: u64,
    /// Ego base X in meters.
    pub base_x_m: f64,
    /// Ego base Y in meters.
    pub base_y_m: f64,
    /// Ego base Z in meters.
    pub base_z_m: f64,
    /// Ego base yaw around world Y in radians.
    pub base_yaw_rad: f64,
    /// Planar speed in meters per second.
    pub speed_m_s: f64,
    /// Left wheel command in radians per second.
    pub left_wheel_velocity_rad_s: f64,
    /// Right wheel command in radians per second.
    pub right_wheel_velocity_rad_s: f64,
    /// Oncoming AGV center X in meters.
    pub other_agv_x_m: f64,
    /// Oncoming AGV still occupies the shared segment.
    pub shared_aisle_occupied: bool,
    /// Ego completed a required yield while the shared segment was busy.
    pub yielded_for_shared_aisle: bool,
    /// Ego footprint overlaps the oncoming AGV.
    pub other_agv_contact: bool,
    /// Ego left the corridor half-width.
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
    /// Yield, dock pickup, and desk delivery are complete while stopped.
    pub delivery_complete: bool,
}

/// Headless shared-aisle office AGV scenario.
pub struct OfficeAgvSharedAisleScenario {
    sim: DiffDriveSim,
    course: OfficeAgvSharedAisleCourse,
    fault: OfficeAgvSharedAisleFault,
    max_steps: u64,
    phase: ScriptPhase,
    other_agv_x_m: f64,
    other_motion: OtherMotion,
    yield_streak: u32,
    yielded_for_shared_aisle: bool,
    other_agv_contact: bool,
    dock_overlap_streak: u32,
    dock_pickup_complete: bool,
    desk_stop_streak: u32,
    desk_delivery_complete: bool,
    desk_without_pickup: bool,
    desk_overshoot: bool,
    observation: OfficeAgvSharedAisleObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for OfficeAgvSharedAisleScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("OfficeAgvSharedAisleScenario")
            .field("fault", &self.fault)
            .field("max_steps", &self.max_steps)
            .field("phase", &self.phase)
            .field("other_agv_x_m", &self.other_agv_x_m)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScriptPhase {
    ApproachDock,
    HoldDock,
    ApproachYield,
    HoldYield,
    ApproachDesk,
    HoldDesk,
    Halt,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OtherMotion {
    Waiting,
    Entering,
    Exiting,
    Cleared,
}

impl OfficeAgvSharedAisleScenario {
    /// Loads the bundled office scene for a successful scripted run.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, OfficeAgvSharedAisleFault::None)
    }

    /// Loads the bundled office scene with an injected fault.
    pub fn new(seed: u64, fault: OfficeAgvSharedAisleFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = office_agv_delivery_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let course = OfficeAgvSharedAisleCourse::default();
        let mut scenario = Self {
            sim,
            course,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            phase: ScriptPhase::ApproachDock,
            other_agv_x_m: course.other_start_x_m,
            other_motion: OtherMotion::Waiting,
            yield_streak: 0,
            yielded_for_shared_aisle: false,
            other_agv_contact: false,
            dock_overlap_streak: 0,
            dock_pickup_complete: false,
            desk_stop_streak: 0,
            desk_delivery_complete: false,
            desk_without_pickup: false,
            desk_overshoot: false,
            observation: placeholder_observation(course.other_start_x_m),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current shared-aisle observation.
    #[must_use]
    pub fn current_observation(&self) -> OfficeAgvSharedAisleObservation {
        self.observation
    }

    fn advance_other_agv(&mut self) {
        let elapsed_s = self.sim.step_count() as f64 * DT_S;
        let step_m = self.course.other_speed_m_s * DT_S;
        let turn_x_m = 0.5 * (self.course.shared_min_x_m + self.course.shared_max_x_m);
        match self.other_motion {
            OtherMotion::Waiting => {
                if elapsed_s >= self.course.other_departure_delay_s {
                    self.other_motion = OtherMotion::Entering;
                }
            }
            OtherMotion::Entering => {
                self.other_agv_x_m -= step_m;
                if self.other_agv_x_m <= turn_x_m {
                    self.other_motion = OtherMotion::Exiting;
                }
            }
            OtherMotion::Exiting => {
                self.other_agv_x_m += step_m;
                if self.other_agv_x_m >= self.course.other_clear_x_m {
                    self.other_agv_x_m = self.course.other_clear_x_m;
                    self.other_motion = OtherMotion::Cleared;
                }
            }
            OtherMotion::Cleared => {}
        }
    }

    fn observe_world(&self) -> OfficeAgvSharedAisleObservation {
        let drive = self.sim.observe();
        let spawned = &self.sim.robots()[0];
        let speed_m_s = planar_speed(&self.sim, spawned);
        let stopped = is_stopped(
            speed_m_s,
            drive.left_wheel_velocity_rad_s,
            drive.right_wheel_velocity_rad_s,
        );
        let aabb =
            self.course
                .delivery
                .robot_aabb(drive.base_x_m, drive.base_z_m, drive.base_yaw_rad);
        let other = self.course.other_aabb(self.other_agv_x_m, 0.0);
        let shared_aisle_occupied = self.course.other_occupies_shared(self.other_agv_x_m);
        let other_agv_contact = self.other_agv_contact || aabb.overlaps(other);
        let out_of_corridor = aabb.max_z_m > self.course.delivery.corridor_half_width_m
            || aabb.min_z_m < -self.course.delivery.corridor_half_width_m;
        let delivery_complete = self.yielded_for_shared_aisle
            && self.dock_pickup_complete
            && self.desk_delivery_complete
            && stopped;

        OfficeAgvSharedAisleObservation {
            step: self.sim.step_count(),
            base_x_m: drive.base_x_m,
            base_y_m: drive.base_y_m,
            base_z_m: drive.base_z_m,
            base_yaw_rad: drive.base_yaw_rad,
            speed_m_s,
            left_wheel_velocity_rad_s: drive.left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s: drive.right_wheel_velocity_rad_s,
            other_agv_x_m: self.other_agv_x_m,
            shared_aisle_occupied,
            yielded_for_shared_aisle: self.yielded_for_shared_aisle,
            other_agv_contact,
            out_of_corridor,
            dock_pickup_complete: self.dock_pickup_complete,
            desk_delivery_complete: self.desk_delivery_complete,
            desk_without_pickup: self.desk_without_pickup,
            desk_overshoot: self.desk_overshoot,
            stopped,
            delivery_complete,
        }
    }

    fn update_judge(&mut self, observation: OfficeAgvSharedAisleObservation) {
        let aabb = self.course.delivery.robot_aabb(
            observation.base_x_m,
            observation.base_z_m,
            observation.base_yaw_rad,
        );
        let other = self.course.other_aabb(self.other_agv_x_m, 0.0);
        self.other_agv_contact = observation.other_agv_contact || aabb.overlaps(other);

        if !self.yielded_for_shared_aisle {
            let waiting_at_line = observation.stopped
                && observation.shared_aisle_occupied
                && aabb.max_x_m < self.course.shared_min_x_m + 0.05;
            if waiting_at_line {
                self.yield_streak = self
                    .yield_streak
                    .saturating_add(1)
                    .min(self.course.required_yield_steps);
                if self.yield_streak >= self.course.required_yield_steps {
                    self.yielded_for_shared_aisle = true;
                }
            } else {
                self.yield_streak = 0;
            }
        }

        let dock = self.course.delivery.dock_aabb();
        let geometry_dock = aabb.overlaps(dock);
        let physics_dock =
            robot_contacts_named(&self.sim, &self.sim.robots()[0], OFFICE_PICKUP_DOCK_NAME);
        if !self.dock_pickup_complete {
            if observation.stopped && (geometry_dock || physics_dock) {
                self.dock_overlap_streak = self
                    .dock_overlap_streak
                    .saturating_add(1)
                    .min(self.course.delivery.required_dock_steps);
                if self.dock_overlap_streak >= self.course.delivery.required_dock_steps {
                    self.dock_pickup_complete = true;
                }
            } else {
                self.dock_overlap_streak = 0;
            }
        }

        let judgement = evaluate_office_desk_delivery_stop(
            aabb.min_x_m,
            aabb.max_x_m,
            self.course.delivery.desk_face_x_m,
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
                    .min(self.course.delivery.required_stop_steps);
                if self.desk_stop_streak >= self.course.delivery.required_stop_steps {
                    self.desk_delivery_complete = true;
                }
            } else if !self.desk_delivery_complete {
                self.desk_stop_streak = 0;
            }
        }
    }

    fn action(&mut self, observation: OfficeAgvSharedAisleObservation) -> DiffDriveAction {
        if matches!(self.fault, OfficeAgvSharedAisleFault::IgnoreYield) {
            return DiffDriveAction::forward(CRUISE_WHEEL_RAD_S);
        }

        match self.phase {
            ScriptPhase::ApproachDock => {
                let stop_x_m = self.course.delivery.dock_x_m;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldDock;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldDock => {
                if self.dock_pickup_complete {
                    self.phase = ScriptPhase::ApproachYield;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::ApproachYield => {
                let stop_x_m =
                    self.course.shared_min_x_m - self.course.delivery.robot_half_x_m - 0.15;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldYield;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldYield => {
                if self.yielded_for_shared_aisle && !observation.shared_aisle_occupied {
                    self.phase = ScriptPhase::ApproachDesk;
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::ApproachDesk => {
                let stop_x_m =
                    self.course.delivery.desk_face_x_m - self.course.delivery.robot_half_x_m - 0.35;
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

impl BehaviorScenario for OfficeAgvSharedAisleScenario {
    type Observation = OfficeAgvSharedAisleObservation;

    fn fixed_delta(&self) -> SimDuration {
        self.sim.fixed_delta()
    }

    fn initial_observation(&self) -> Self::Observation {
        self.observation
    }

    fn state_digest(&self, observation: &Self::Observation) -> u64 {
        let mut bytes = hash_physics_state(self.sim.world()).to_le_bytes().to_vec();
        bytes.extend_from_slice(&observation.other_agv_x_m.to_bits().to_le_bytes());
        bytes.push(u8::from(observation.yielded_for_shared_aisle));
        stable_behavior_digest(&bytes)
    }

    fn scenario_digest(&self) -> u64 {
        let mut bytes = b"office_agv_shared_aisle_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            OfficeAgvSharedAisleFault::None => 0,
            OfficeAgvSharedAisleFault::IgnoreYield => 1,
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
                |observation: &OfficeAgvSharedAisleObservation| !observation.out_of_corridor,
            )?
            .with_entities(["office_agv_shared_aisle"])?,
            BehaviorContract::always(
                "no_other_agv_contact",
                |observation: &OfficeAgvSharedAisleObservation| !observation.other_agv_contact,
            )?
            .with_entities([OFFICE_ONCOMING_AGV_NAME])?,
            BehaviorContract::always(
                "no_desk_without_pickup",
                |observation: &OfficeAgvSharedAisleObservation| !observation.desk_without_pickup,
            )?
            .with_entities([OFFICE_PICKUP_DOCK_NAME, OFFICE_DELIVERY_DESK_NAME])?,
            BehaviorContract::always(
                "no_desk_overshoot",
                |observation: &OfficeAgvSharedAisleObservation| !observation.desk_overshoot,
            )?
            .with_entities([OFFICE_DELIVERY_DESK_NAME])?,
            BehaviorContract::eventually(
                "yielded_for_shared_aisle",
                deadline,
                |observation: &OfficeAgvSharedAisleObservation| {
                    observation.yielded_for_shared_aisle
                },
            )?
            .with_entities([OFFICE_ONCOMING_AGV_NAME])?,
            BehaviorContract::eventually(
                "dock_pickup_complete",
                deadline,
                |observation: &OfficeAgvSharedAisleObservation| observation.dock_pickup_complete,
            )?
            .with_entities([OFFICE_PICKUP_DOCK_NAME])?,
            BehaviorContract::eventually(
                "delivery_complete",
                deadline,
                |observation: &OfficeAgvSharedAisleObservation| observation.delivery_complete,
            )?
            .with_entities([OFFICE_DELIVERY_DESK_NAME])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let action = self.action(self.observation);
        self.sim.step_action(action);
        self.advance_other_agv();
        let mut observation = self.observe_world();
        self.update_judge(observation);
        observation = self.observe_world();
        self.observation = observation;
        let done = observation.delivery_complete
            || observation.out_of_corridor
            || observation.other_agv_contact
            || observation.desk_without_pickup
            || observation.desk_overshoot
            || observation.step >= self.max_steps;
        BehaviorScenarioStep { observation, done }
    }
}

fn placeholder_observation(other_agv_x_m: f64) -> OfficeAgvSharedAisleObservation {
    OfficeAgvSharedAisleObservation {
        step: 0,
        base_x_m: 0.0,
        base_y_m: 0.0,
        base_z_m: 0.0,
        base_yaw_rad: 0.0,
        speed_m_s: 0.0,
        left_wheel_velocity_rad_s: 0.0,
        right_wheel_velocity_rad_s: 0.0,
        other_agv_x_m,
        shared_aisle_occupied: true,
        yielded_for_shared_aisle: false,
        other_agv_contact: false,
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

fn fault_dimensions(fault: OfficeAgvSharedAisleFault) -> Vec<BehaviorDimension> {
    vec![BehaviorDimension {
        name: "ignore_yield".to_string(),
        value: BehaviorDimensionValue::Boolean(matches!(
            fault,
            OfficeAgvSharedAisleFault::IgnoreYield
        )),
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

/// Bundled TaskSpec path for the shared-aisle analog.
#[must_use]
pub fn office_agv_shared_aisle_task_path() -> PathBuf {
    bundled_asset_path(Path::new("tasks/office_agv_shared_aisle.task.json"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{run_behavior_scenarios, BehaviorContractStatus, BehaviorSeedStatus};

    #[test]
    fn shared_aisle_block_requires_occupation_past_the_line() {
        assert!(evaluate_office_shared_aisle_block(3.6, true, 3.5));
        assert!(!evaluate_office_shared_aisle_block(3.2, true, 3.5));
        assert!(!evaluate_office_shared_aisle_block(3.6, false, 3.5));
    }

    #[test]
    fn shared_aisle_task_spec_validates() {
        let spec = office_agv_shared_aisle_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let loaded: TaskSpec = serde_json::from_slice(
            &std::fs::read(office_agv_shared_aisle_task_path()).expect("committed task spec"),
        )
        .expect("parse task spec");
        assert_eq!(spec, loaded);
    }

    #[test]
    fn scripted_shared_aisle_delivery_passes() {
        let report = run_behavior_scenarios(
            "office_agv_shared_aisle_success",
            [1],
            OfficeAgvSharedAisleScenario::success,
        );
        assert!(report.passed(), "{report:?}");
        assert!(report.seeds[0].steps > 100);
    }

    #[test]
    fn ignore_yield_fails_other_agv_contact_contract() {
        let report = run_behavior_scenarios("office_agv_shared_aisle_ignore_yield", [1], |seed| {
            OfficeAgvSharedAisleScenario::new(seed, OfficeAgvSharedAisleFault::IgnoreYield)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let contact = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_other_agv_contact")
            .expect("contact contract");
        assert_eq!(contact.status, BehaviorContractStatus::Failed);
    }
}
