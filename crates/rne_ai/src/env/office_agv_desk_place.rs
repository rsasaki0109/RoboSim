//! Office AGV desk place: shared-aisle delivery plus geometric cargo unload.
//!
//! Extends the shared-aisle checklist with a kinematic payload. After the desk
//! stop, the AGV must unload into the desk place box. This is not friction
//! grasp, G1 Dex3, or Nav2 — place remains a planar region judge so the slice
//! stays headless and renderer-free.

use crate::action::DiffDriveAction;
use crate::behavior::{
    BehaviorContract, BehaviorContractError, BehaviorScenario, BehaviorScenarioStep,
};
use crate::behavior_replay::{stable_behavior_digest, BehaviorDimension, BehaviorDimensionValue};
use crate::env::office_agv_delivery::{
    evaluate_office_desk_delivery_stop, office_agv_delivery_scene_path, OfficeAgvDeliveryCourse,
    OfficePlanarAabb, OFFICE_DELIVERY_DESK_NAME, OFFICE_PICKUP_DOCK_NAME,
};
use crate::env::office_agv_shared_aisle::{OfficeAgvSharedAisleCourse, OFFICE_ONCOMING_AGV_NAME};
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

/// Portable task identity for office AGV desk place.
pub const OFFICE_AGV_DESK_PLACE_TASK_ID: &str = "rne.office.agv_desk_place.v1";
/// Logical name for the kinematic cargo payload.
pub const OFFICE_CARGO_NAME: &str = "office_cargo";

const CONTROL_HZ: f64 = 60.0;
const CRUISE_WHEEL_RAD_S: f64 = 5.0;
const STOPPED_SPEED_M_S: f64 = 0.05;
const STOPPED_WHEEL_RAD_S: f64 = 0.2;
const DEFAULT_MAX_STEPS: u64 = 2_400;
const DT_S: f64 = 1.0 / CONTROL_HZ;

/// Desk place box layered on the shared-aisle course.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OfficeAgvDeskPlaceCourse {
    /// Shared-aisle + delivery geometry.
    pub aisle: OfficeAgvSharedAisleCourse,
    /// Place-box center X in meters (in front of the desk face).
    pub place_x_m: f64,
    /// Place-box center Z in meters.
    pub place_z_m: f64,
    /// Place-box half-extent along X in meters.
    pub place_half_x_m: f64,
    /// Place-box half-extent along Z in meters.
    pub place_half_z_m: f64,
    /// Consecutive unloaded-in-box steps required to count a place.
    pub required_place_steps: u32,
}

impl Default for OfficeAgvDeskPlaceCourse {
    fn default() -> Self {
        Self {
            aisle: OfficeAgvSharedAisleCourse::default(),
            place_x_m: 6.5,
            place_z_m: 0.0,
            place_half_x_m: 0.35,
            place_half_z_m: 0.35,
            required_place_steps: 12,
        }
    }
}

impl OfficeAgvDeskPlaceCourse {
    /// Footprint of the desk place box in the ground plane.
    #[must_use]
    pub fn place_aabb(self) -> OfficePlanarAabb {
        OfficePlanarAabb {
            min_x_m: self.place_x_m - self.place_half_x_m,
            max_x_m: self.place_x_m + self.place_half_x_m,
            min_z_m: self.place_z_m - self.place_half_z_m,
            max_z_m: self.place_z_m + self.place_half_z_m,
        }
    }

    /// Delivery course shortcut.
    #[must_use]
    pub fn delivery(self) -> OfficeAgvDeliveryCourse {
        self.aisle.delivery
    }
}

/// Returns whether a cargo center lies inside the desk place box.
#[must_use]
pub fn evaluate_office_desk_place(cargo_x_m: f64, cargo_z_m: f64, place: OfficePlanarAabb) -> bool {
    cargo_x_m >= place.min_x_m
        && cargo_x_m <= place.max_x_m
        && cargo_z_m >= place.min_z_m
        && cargo_z_m <= place.max_z_m
}

/// Portable TaskSpec for office AGV desk place.
#[must_use]
pub fn office_agv_desk_place_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        OFFICE_AGV_DESK_PLACE_TASK_ID,
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
            TensorSpec::new("cargo_loaded", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("desk_place_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("early_drop", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("out_of_corridor", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("dock_pickup_complete", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("desk_delivery_complete", TensorDType::F64, vec![], "1")
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
            RewardTermSpec::new("mission_complete", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("mission_complete", TerminationKind::Success),
                TerminationConditionSpec::new("out_of_corridor", TerminationKind::Failure),
                TerminationConditionSpec::new("other_agv_contact", TerminationKind::Failure),
                TerminationConditionSpec::new("early_drop", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Injected desk-place faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OfficeAgvDeskPlaceFault {
    /// Yield, deliver, and unload cargo into the desk place box.
    #[default]
    None,
    /// Reach the desk stop but never unload.
    SkipPlace,
    /// Drop cargo before the desk delivery stop.
    DropEarly,
}

/// Headless observation for office AGV desk place.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OfficeAgvDeskPlaceObservation {
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
    /// Cargo is loaded on the AGV.
    pub cargo_loaded: bool,
    /// Cargo was unloaded into the desk place box.
    pub desk_place_complete: bool,
    /// Cargo left the AGV before the desk delivery stop.
    pub early_drop: bool,
    /// Ego left the corridor half-width.
    pub out_of_corridor: bool,
    /// Pickup dock visit completed.
    pub dock_pickup_complete: bool,
    /// Desk delivery stop completed after pickup.
    pub desk_delivery_complete: bool,
    /// Wheel commands and planar speed are at rest.
    pub stopped: bool,
    /// Yield, dock, desk stop, and place are complete.
    pub mission_complete: bool,
}

/// Headless office AGV desk-place scenario.
pub struct OfficeAgvDeskPlaceScenario {
    sim: DiffDriveSim,
    course: OfficeAgvDeskPlaceCourse,
    fault: OfficeAgvDeskPlaceFault,
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
    cargo_loaded: bool,
    cargo_x_m: f64,
    cargo_z_m: f64,
    place_streak: u32,
    desk_place_complete: bool,
    early_drop: bool,
    observation: OfficeAgvDeskPlaceObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for OfficeAgvDeskPlaceScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("OfficeAgvDeskPlaceScenario")
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
    ApproachYield,
    HoldYield,
    ApproachDesk,
    HoldDesk,
    PlaceCargo,
    Halt,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OtherMotion {
    Waiting,
    Entering,
    Exiting,
    Cleared,
}

impl OfficeAgvDeskPlaceScenario {
    /// Loads the bundled office scene for a successful scripted run.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, OfficeAgvDeskPlaceFault::None)
    }

    /// Loads the bundled office scene with an injected fault.
    pub fn new(seed: u64, fault: OfficeAgvDeskPlaceFault) -> Result<Self, AssetError> {
        let _ = seed;
        let scene_path = office_agv_delivery_scene_path();
        let scenario_input_digest = digest_scene_inputs(&scene_path)?;
        let sim = DiffDriveSim::from_scene_path(&scene_path)?;
        let course = OfficeAgvDeskPlaceCourse::default();
        let mut scenario = Self {
            sim,
            course,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            phase: ScriptPhase::ApproachDock,
            other_agv_x_m: course.aisle.other_start_x_m,
            other_motion: OtherMotion::Waiting,
            yield_streak: 0,
            yielded_for_shared_aisle: false,
            other_agv_contact: false,
            dock_overlap_streak: 0,
            dock_pickup_complete: false,
            desk_stop_streak: 0,
            desk_delivery_complete: false,
            cargo_loaded: false,
            cargo_x_m: course.delivery().dock_x_m,
            cargo_z_m: 0.0,
            place_streak: 0,
            desk_place_complete: false,
            early_drop: false,
            observation: placeholder_observation(course.aisle.other_start_x_m),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe_world();
        Ok(scenario)
    }

    /// Current desk-place observation.
    #[must_use]
    pub fn current_observation(&self) -> OfficeAgvDeskPlaceObservation {
        self.observation
    }

    /// Provides read-only access to the ECS-backed AGV world for render
    /// capture. The oncoming AGV and cargo proxy are scenario state and are
    /// exposed in [`OfficeAgvDeskPlaceObservation`].
    #[must_use]
    pub fn simulation(&self) -> &DiffDriveSim {
        &self.sim
    }

    /// Returns the render-only cargo proxy position in world X/Z meters.
    ///
    /// Cargo remains scenario state rather than an ECS body so that the desk
    /// place judge can model a deterministic unload without coupling it to a
    /// physics backend.
    #[must_use]
    pub fn cargo_translation_m(&self) -> (f64, f64) {
        (self.cargo_x_m, self.cargo_z_m)
    }

    /// Returns the render-only oncoming AGV position in world X/Z meters.
    #[must_use]
    pub fn other_agv_translation_m(&self) -> (f64, f64) {
        (self.other_agv_x_m, 0.0)
    }

    fn advance_other_agv(&mut self) {
        let elapsed_s = self.sim.step_count() as f64 * DT_S;
        let step_m = self.course.aisle.other_speed_m_s * DT_S;
        let turn_x_m = 0.5 * (self.course.aisle.shared_min_x_m + self.course.aisle.shared_max_x_m);
        match self.other_motion {
            OtherMotion::Waiting => {
                if elapsed_s >= self.course.aisle.other_departure_delay_s {
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
                if self.other_agv_x_m >= self.course.aisle.other_clear_x_m {
                    self.other_agv_x_m = self.course.aisle.other_clear_x_m;
                    self.other_motion = OtherMotion::Cleared;
                }
            }
            OtherMotion::Cleared => {}
        }
    }

    fn observe_world(&self) -> OfficeAgvDeskPlaceObservation {
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
                .delivery()
                .robot_aabb(drive.base_x_m, drive.base_z_m, drive.base_yaw_rad);
        let other = self.course.aisle.other_aabb(self.other_agv_x_m, 0.0);
        let shared_aisle_occupied = self.course.aisle.other_occupies_shared(self.other_agv_x_m);
        let other_agv_contact = self.other_agv_contact || aabb.overlaps(other);
        let out_of_corridor = aabb.max_z_m > self.course.delivery().corridor_half_width_m
            || aabb.min_z_m < -self.course.delivery().corridor_half_width_m;
        let mission_complete = self.yielded_for_shared_aisle
            && self.dock_pickup_complete
            && self.desk_delivery_complete
            && self.desk_place_complete
            && !self.cargo_loaded
            && stopped;

        OfficeAgvDeskPlaceObservation {
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
            cargo_loaded: self.cargo_loaded,
            desk_place_complete: self.desk_place_complete,
            early_drop: self.early_drop,
            out_of_corridor,
            dock_pickup_complete: self.dock_pickup_complete,
            desk_delivery_complete: self.desk_delivery_complete,
            stopped,
            mission_complete,
        }
    }

    fn update_judge(&mut self, observation: OfficeAgvDeskPlaceObservation) {
        let aabb = self.course.delivery().robot_aabb(
            observation.base_x_m,
            observation.base_z_m,
            observation.base_yaw_rad,
        );
        let other = self.course.aisle.other_aabb(self.other_agv_x_m, 0.0);
        self.other_agv_contact = observation.other_agv_contact || aabb.overlaps(other);

        if !self.yielded_for_shared_aisle {
            let waiting_at_line = observation.stopped
                && observation.shared_aisle_occupied
                && aabb.max_x_m < self.course.aisle.shared_min_x_m + 0.05;
            if waiting_at_line {
                self.yield_streak = self
                    .yield_streak
                    .saturating_add(1)
                    .min(self.course.aisle.required_yield_steps);
                if self.yield_streak >= self.course.aisle.required_yield_steps {
                    self.yielded_for_shared_aisle = true;
                }
            } else {
                self.yield_streak = 0;
            }
        }

        let dock = self.course.delivery().dock_aabb();
        let geometry_dock = aabb.overlaps(dock);
        let physics_dock =
            robot_contacts_named(&self.sim, &self.sim.robots()[0], OFFICE_PICKUP_DOCK_NAME);
        if !self.dock_pickup_complete {
            if observation.stopped && (geometry_dock || physics_dock) {
                self.dock_overlap_streak = self
                    .dock_overlap_streak
                    .saturating_add(1)
                    .min(self.course.delivery().required_dock_steps);
                if self.dock_overlap_streak >= self.course.delivery().required_dock_steps {
                    self.dock_pickup_complete = true;
                    self.cargo_loaded = true;
                    self.cargo_x_m = observation.base_x_m;
                    self.cargo_z_m = observation.base_z_m;
                }
            } else {
                self.dock_overlap_streak = 0;
            }
        }

        if matches!(self.fault, OfficeAgvDeskPlaceFault::DropEarly)
            && self.dock_pickup_complete
            && !self.desk_delivery_complete
            && self.cargo_loaded
            && observation.base_x_m >= self.course.aisle.shared_min_x_m
        {
            self.cargo_loaded = false;
            self.early_drop = true;
        }

        let judgement = evaluate_office_desk_delivery_stop(
            aabb.min_x_m,
            aabb.max_x_m,
            self.course.delivery().desk_face_x_m,
        );
        if !self.desk_delivery_complete
            && self.dock_pickup_complete
            && judgement.valid()
            && observation.stopped
        {
            self.desk_stop_streak = self
                .desk_stop_streak
                .saturating_add(1)
                .min(self.course.delivery().required_stop_steps);
            if self.desk_stop_streak >= self.course.delivery().required_stop_steps {
                self.desk_delivery_complete = true;
            }
        } else if !self.desk_delivery_complete {
            self.desk_stop_streak = 0;
        }

        if self.cargo_loaded {
            self.cargo_x_m = observation.base_x_m;
            self.cargo_z_m = observation.base_z_m;
        }

        if !self.desk_place_complete
            && self.desk_delivery_complete
            && !self.cargo_loaded
            && evaluate_office_desk_place(self.cargo_x_m, self.cargo_z_m, self.course.place_aabb())
        {
            self.place_streak = self
                .place_streak
                .saturating_add(1)
                .min(self.course.required_place_steps);
            if self.place_streak >= self.course.required_place_steps {
                self.desk_place_complete = true;
            }
        } else if !self.desk_place_complete {
            self.place_streak = 0;
        }
    }

    fn action(&mut self, observation: OfficeAgvDeskPlaceObservation) -> DiffDriveAction {
        match self.phase {
            ScriptPhase::ApproachDock => {
                let stop_x_m = self.course.delivery().dock_x_m;
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
                    self.course.aisle.shared_min_x_m - self.course.delivery().robot_half_x_m - 0.15;
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
                let stop_x_m = self.course.delivery().desk_face_x_m
                    - self.course.delivery().robot_half_x_m
                    - 0.35;
                if observation.base_x_m >= stop_x_m {
                    self.phase = ScriptPhase::HoldDesk;
                    DiffDriveAction::forward(0.0)
                } else {
                    DiffDriveAction::forward(CRUISE_WHEEL_RAD_S)
                }
            }
            ScriptPhase::HoldDesk => {
                if self.desk_delivery_complete {
                    self.phase = if matches!(self.fault, OfficeAgvDeskPlaceFault::SkipPlace) {
                        ScriptPhase::Halt
                    } else {
                        ScriptPhase::PlaceCargo
                    };
                }
                DiffDriveAction::forward(0.0)
            }
            ScriptPhase::PlaceCargo => {
                if self.cargo_loaded {
                    self.cargo_loaded = false;
                    self.cargo_x_m = self.course.place_x_m;
                    self.cargo_z_m = self.course.place_z_m;
                }
                if self.desk_place_complete {
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

impl BehaviorScenario for OfficeAgvDeskPlaceScenario {
    type Observation = OfficeAgvDeskPlaceObservation;

    fn fixed_delta(&self) -> SimDuration {
        self.sim.fixed_delta()
    }

    fn initial_observation(&self) -> Self::Observation {
        self.observation
    }

    fn state_digest(&self, observation: &Self::Observation) -> u64 {
        let mut bytes = hash_physics_state(self.sim.world()).to_le_bytes().to_vec();
        bytes.extend_from_slice(&observation.other_agv_x_m.to_bits().to_le_bytes());
        bytes.extend_from_slice(&self.cargo_x_m.to_bits().to_le_bytes());
        bytes.push(u8::from(observation.cargo_loaded));
        bytes.push(u8::from(observation.desk_place_complete));
        stable_behavior_digest(&bytes)
    }

    fn scenario_digest(&self) -> u64 {
        let mut bytes = b"office_agv_desk_place_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            OfficeAgvDeskPlaceFault::None => 0,
            OfficeAgvDeskPlaceFault::SkipPlace => 1,
            OfficeAgvDeskPlaceFault::DropEarly => 2,
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
                |observation: &OfficeAgvDeskPlaceObservation| !observation.out_of_corridor,
            )?
            .with_entities(["office_agv_desk_place"])?,
            BehaviorContract::always(
                "no_other_agv_contact",
                |observation: &OfficeAgvDeskPlaceObservation| !observation.other_agv_contact,
            )?
            .with_entities([OFFICE_ONCOMING_AGV_NAME])?,
            BehaviorContract::always(
                "no_early_drop",
                |observation: &OfficeAgvDeskPlaceObservation| !observation.early_drop,
            )?
            .with_entities([OFFICE_CARGO_NAME])?,
            BehaviorContract::eventually(
                "yielded_for_shared_aisle",
                deadline,
                |observation: &OfficeAgvDeskPlaceObservation| observation.yielded_for_shared_aisle,
            )?
            .with_entities([OFFICE_ONCOMING_AGV_NAME])?,
            BehaviorContract::eventually(
                "dock_pickup_complete",
                deadline,
                |observation: &OfficeAgvDeskPlaceObservation| observation.dock_pickup_complete,
            )?
            .with_entities([OFFICE_PICKUP_DOCK_NAME])?,
            BehaviorContract::eventually(
                "desk_place_complete",
                deadline,
                |observation: &OfficeAgvDeskPlaceObservation| observation.desk_place_complete,
            )?
            .with_entities([OFFICE_DELIVERY_DESK_NAME, OFFICE_CARGO_NAME])?,
            BehaviorContract::eventually(
                "mission_complete",
                deadline,
                |observation: &OfficeAgvDeskPlaceObservation| observation.mission_complete,
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
        let done = observation.mission_complete
            || observation.out_of_corridor
            || observation.other_agv_contact
            || observation.early_drop
            || (matches!(self.fault, OfficeAgvDeskPlaceFault::SkipPlace)
                && observation.desk_delivery_complete
                && !observation.desk_place_complete
                && matches!(self.phase, ScriptPhase::Halt))
            || observation.step >= self.max_steps;
        BehaviorScenarioStep { observation, done }
    }
}

fn placeholder_observation(other_agv_x_m: f64) -> OfficeAgvDeskPlaceObservation {
    OfficeAgvDeskPlaceObservation {
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
        cargo_loaded: false,
        desk_place_complete: false,
        early_drop: false,
        out_of_corridor: false,
        dock_pickup_complete: false,
        desk_delivery_complete: false,
        stopped: true,
        mission_complete: false,
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

fn fault_dimensions(fault: OfficeAgvDeskPlaceFault) -> Vec<BehaviorDimension> {
    vec![
        BehaviorDimension {
            name: "skip_place".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                OfficeAgvDeskPlaceFault::SkipPlace
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
        BehaviorDimension {
            name: "drop_early".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                OfficeAgvDeskPlaceFault::DropEarly
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

/// Bundled TaskSpec path for the desk-place analog.
#[must_use]
pub fn office_agv_desk_place_task_path() -> PathBuf {
    crate::asset_path::bundled_asset_path(Path::new("tasks/office_agv_desk_place.task.json"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{run_behavior_scenarios, BehaviorContractStatus, BehaviorSeedStatus};

    #[test]
    fn desk_place_requires_cargo_center_inside_box() {
        let place = OfficeAgvDeskPlaceCourse::default().place_aabb();
        assert!(evaluate_office_desk_place(6.5, 0.0, place));
        assert!(!evaluate_office_desk_place(5.0, 0.0, place));
    }

    #[test]
    fn desk_place_task_spec_validates() {
        let spec = office_agv_desk_place_task_spec(DEFAULT_MAX_STEPS);
        spec.validate().expect("task spec");
        let loaded: TaskSpec = serde_json::from_slice(
            &std::fs::read(office_agv_desk_place_task_path()).expect("committed task spec"),
        )
        .expect("parse task spec");
        assert_eq!(spec, loaded);
    }

    #[test]
    fn scripted_desk_place_mission_passes() {
        let report = run_behavior_scenarios(
            "office_agv_desk_place_success",
            [1],
            OfficeAgvDeskPlaceScenario::success,
        );
        assert!(report.passed(), "{report:?}");
        assert!(report.seeds[0].steps > 100);
    }

    #[test]
    fn skip_place_fails_desk_place_contract() {
        let report = run_behavior_scenarios("office_agv_desk_place_skip", [1], |seed| {
            OfficeAgvDeskPlaceScenario::new(seed, OfficeAgvDeskPlaceFault::SkipPlace)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let place = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "desk_place_complete")
            .expect("place contract");
        assert_eq!(place.status, BehaviorContractStatus::Failed);
    }

    #[test]
    fn drop_early_fails_cargo_contract() {
        let report = run_behavior_scenarios("office_agv_desk_place_drop_early", [1], |seed| {
            OfficeAgvDeskPlaceScenario::new(seed, OfficeAgvDeskPlaceFault::DropEarly)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let early = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_early_drop")
            .expect("early drop contract");
        assert_eq!(early.status, BehaviorContractStatus::Failed);
    }
}
