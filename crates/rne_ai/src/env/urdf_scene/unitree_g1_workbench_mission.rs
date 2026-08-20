//! Grove-G1 style workbench mission: walk into the 0.5 m park, then pick and place.
//!
//! This is not a ROS 2 / Nav2 / MoveIt port. Approach uses the dynamic G1 factory
//! plant. Manipulation uses the existing pelvis-pinned Dex3 workcell, matching
//! Grove's split between a navigation world and `pin_pelvis` manipulation.

use super::{
    unitree_g1_factory_scene_path, UnitreeG1Dex3Action, UnitreeG1Dex3Episode,
    UnitreeG1Dex3EpisodeConfig, UnitreeG1InspectionAction, UnitreeG1InspectionEpisode,
    UnitreeG1InspectionEpisodeConfig, UrdfSceneSim,
};
use crate::{
    stable_behavior_digest, ActionSpec, BehaviorContract, BehaviorContractError, BehaviorDimension,
    BehaviorDimensionValue, BehaviorScenario, BehaviorScenarioStep, Episode, ObservationSpec,
    ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType, TensorSpec,
    TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_assets::{load_scene_bundle, scene_dependency_paths, AssetError};
use rne_core::SimDuration;
use rne_math::Hertz;
use rne_physics::hash_physics_state;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Portable task identity for the G1 workbench mission.
pub const G1_WORKBENCH_MISSION_TASK_ID: &str = "rne.g1.workbench_mission.v1";
/// Grove Nav2 park radius: stay within 0.5 m of the goal pose.
pub const G1_WORKBENCH_PARK_RADIUS_M: f64 = 0.5;
/// Grove arm usable window after the last closed-loop approach.
pub const G1_WORKBENCH_ARM_WINDOW_M: f64 = 0.2;
/// Pelvis height below which the walking plant is treated as fallen.
pub const G1_WORKBENCH_MIN_PELVIS_Y_M: f64 = 0.55;

const CONTROL_HZ: f64 = 60.0;
const DEFAULT_MAX_STEPS: u64 = 800;
const WORKBENCH_MARKER: &str = "inspection_parts_check";

/// Injected workbench-mission faults.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UnitreeG1WorkbenchFault {
    /// Walk into the park radius, then Dex3 pick and place.
    #[default]
    None,
    /// Skip the walking plant and start in the pinned-pelvis workcell.
    SkipApproach,
    /// Complete the approach, then never start manipulation.
    DropPart,
}

/// Combined mission observation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnitreeG1WorkbenchObservation {
    /// Completed mission steps across both plants.
    pub step: u64,
    /// Horizontal distance to the workbench marker in meters.
    pub workbench_distance_m: f64,
    /// Pelvis height in meters during approach, or the Dex3 base height later.
    pub pelvis_y_m: f64,
    /// Inside Grove's 0.5 m Nav2 park radius.
    pub parked: bool,
    /// Object is inside the 0.2 m arm window, or the pinned workcell has started.
    pub in_arm_window: bool,
    /// Walking plant stayed upright.
    pub upright: bool,
    /// Dex3 grasp was acquired.
    pub grasped: bool,
    /// Dex3 place settled.
    pub placed: bool,
    /// Park, arm window, grasp, and place all completed while upright.
    pub mission_complete: bool,
}

/// Headless Grove-style navigate-then-manipulate analog.
pub struct UnitreeG1WorkbenchMissionScenario {
    walk: Option<UnitreeG1InspectionEpisode>,
    hands: Option<UnitreeG1Dex3Episode>,
    fault: UnitreeG1WorkbenchFault,
    max_steps: u64,
    parked: bool,
    in_arm_window: bool,
    upright: bool,
    grasped: bool,
    placed: bool,
    step: u64,
    observation: UnitreeG1WorkbenchObservation,
    scenario_input_digest: u64,
    dimensions: Vec<BehaviorDimension>,
}

impl std::fmt::Debug for UnitreeG1WorkbenchMissionScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("UnitreeG1WorkbenchMissionScenario")
            .field("fault", &self.fault)
            .field("observation", &self.observation)
            .finish_non_exhaustive()
    }
}

impl UnitreeG1WorkbenchMissionScenario {
    /// Successful scripted walk then Dex3 pick-and-place.
    pub fn success(seed: u64) -> Result<Self, AssetError> {
        Self::new(seed, UnitreeG1WorkbenchFault::None)
    }

    /// Loads both plants required by the selected fault.
    pub fn new(seed: u64, fault: UnitreeG1WorkbenchFault) -> Result<Self, AssetError> {
        let _ = seed;
        let factory = unitree_g1_factory_scene_path();
        let dex3 = super::unitree_g1_dex3_scene_path();
        let scenario_input_digest = digest_two_scenes(&factory, &dex3)?;
        let walk = if matches!(fault, UnitreeG1WorkbenchFault::SkipApproach) {
            None
        } else {
            Some(UnitreeG1InspectionEpisode::new(
                UnitreeG1InspectionEpisodeConfig {
                    marker_names: vec![WORKBENCH_MARKER.into()],
                    max_steps: 300,
                    ..UnitreeG1InspectionEpisodeConfig::default()
                },
            )?)
        };
        let hands = if matches!(fault, UnitreeG1WorkbenchFault::SkipApproach) {
            Some(UnitreeG1Dex3Episode::new(
                UnitreeG1Dex3EpisodeConfig::default(),
            )?)
        } else {
            None
        };
        let mut scenario = Self {
            walk,
            hands,
            fault,
            max_steps: DEFAULT_MAX_STEPS,
            parked: false,
            in_arm_window: matches!(fault, UnitreeG1WorkbenchFault::SkipApproach),
            upright: true,
            grasped: false,
            placed: false,
            step: 0,
            observation: placeholder_observation(),
            scenario_input_digest,
            dimensions: fault_dimensions(fault),
        };
        scenario.observation = scenario.observe();
        Ok(scenario)
    }

    /// Latest combined observation.
    #[must_use]
    pub fn current_observation(&self) -> UnitreeG1WorkbenchObservation {
        self.observation
    }

    fn observe(&self) -> UnitreeG1WorkbenchObservation {
        let (workbench_distance_m, pelvis_y_m) = if let Some(walk) = &self.walk {
            let obs = walk.current_observation();
            (obs.marker_distance_m, obs.base_position_m[1])
        } else if let Some(hands) = &self.hands {
            let obs = hands.current_observation();
            (G1_WORKBENCH_ARM_WINDOW_M, obs.part_position_m[1])
        } else {
            (f64::INFINITY, 0.0)
        };
        let grasped = self.grasped
            || self
                .hands
                .as_ref()
                .is_some_and(|hands| hands.current_observation().was_grasped);
        let placed = self.placed
            || self
                .hands
                .as_ref()
                .is_some_and(|hands| hands.current_observation().placed);
        let mission_complete =
            self.parked && self.in_arm_window && grasped && placed && self.upright;
        UnitreeG1WorkbenchObservation {
            step: self.step,
            workbench_distance_m,
            pelvis_y_m,
            parked: self.parked,
            in_arm_window: self.in_arm_window,
            upright: self.upright,
            grasped,
            placed,
            mission_complete,
        }
    }

    fn active_sim(&self) -> Option<&UrdfSceneSim> {
        if let Some(hands) = &self.hands {
            return Some(hands.simulation());
        }
        self.walk
            .as_ref()
            .map(UnitreeG1InspectionEpisode::simulation)
    }

    fn deadline(&self) -> SimDuration {
        SimDuration::from_ticks(
            SimDuration::from_hertz(Hertz::new(CONTROL_HZ))
                .ticks()
                .saturating_mul(self.max_steps),
        )
    }
}

/// Portable TaskSpec for the workbench mission.
#[must_use]
pub fn unitree_g1_workbench_task_spec(max_episode_steps: u64) -> TaskSpec {
    TaskSpec::new(
        G1_WORKBENCH_MISSION_TASK_ID,
        1.0 / CONTROL_HZ,
        ObservationSpec::new(vec![
            TensorSpec::new("workbench_distance_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("pelvis_y_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("parked", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("in_arm_window", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("upright", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("grasped", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
            TensorSpec::new("placed", TensorDType::F64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 1.0)),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "advance",
            TensorDType::F64,
            vec![],
            "1",
        )
        .with_bounds(TensorBounds::broadcast(0.0, 1.0))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("approach_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("mission_complete", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("mission_complete", TerminationKind::Success),
                TerminationConditionSpec::new("fallen", TerminationKind::Failure),
                TerminationConditionSpec::new("dropped_part", TerminationKind::Failure),
            ],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

impl BehaviorScenario for UnitreeG1WorkbenchMissionScenario {
    type Observation = UnitreeG1WorkbenchObservation;

    fn fixed_delta(&self) -> SimDuration {
        SimDuration::from_hertz(Hertz::new(CONTROL_HZ))
    }

    fn initial_observation(&self) -> Self::Observation {
        self.observation
    }

    fn state_digest(&self, observation: &Self::Observation) -> u64 {
        if let Some(sim) = self.active_sim() {
            hash_physics_state(sim.world())
        } else {
            stable_behavior_digest(&observation.workbench_distance_m.to_bits().to_le_bytes())
        }
    }

    fn scenario_digest(&self) -> u64 {
        let mut bytes = b"g1_workbench_mission_v1".to_vec();
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.max_steps.to_le_bytes());
        bytes.push(match self.fault {
            UnitreeG1WorkbenchFault::None => 0,
            UnitreeG1WorkbenchFault::SkipApproach => 1,
            UnitreeG1WorkbenchFault::DropPart => 2,
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
                "stay_upright",
                |observation: &UnitreeG1WorkbenchObservation| observation.upright,
            )?
            .with_entities(["pelvis"])?,
            BehaviorContract::eventually(
                "park_within_0_5_m",
                deadline,
                |observation: &UnitreeG1WorkbenchObservation| observation.parked,
            )?
            .with_entities([WORKBENCH_MARKER])?,
            BehaviorContract::eventually(
                "arm_window_0_2_m",
                deadline,
                |observation: &UnitreeG1WorkbenchObservation| observation.in_arm_window,
            )?
            .with_entities(["dex3_inspection_part"])?,
            BehaviorContract::eventually(
                "grasped",
                deadline,
                |observation: &UnitreeG1WorkbenchObservation| observation.grasped,
            )?
            .with_entities(["dex3_inspection_part"])?,
            BehaviorContract::eventually(
                "placed",
                deadline,
                |observation: &UnitreeG1WorkbenchObservation| observation.placed,
            )?
            .with_entities(["dex3_place_zone"])?,
            BehaviorContract::eventually(
                "mission_complete",
                deadline,
                |observation: &UnitreeG1WorkbenchObservation| observation.mission_complete,
            )?
            .with_entities(["workbench"])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        if let Some(walk) = &mut self.walk {
            let step = walk.step(UnitreeG1InspectionAction { advance: true });
            self.step += 1;
            if step.observation.base_position_m[1] < G1_WORKBENCH_MIN_PELVIS_Y_M {
                self.upright = false;
            }
            if step.observation.marker_distance_m <= G1_WORKBENCH_PARK_RADIUS_M {
                self.parked = true;
            }
            if step.observation.marker_distance_m <= G1_WORKBENCH_ARM_WINDOW_M {
                self.in_arm_window = true;
            }
            if step.terminated && matches!(self.fault, UnitreeG1WorkbenchFault::None) {
                self.walk = None;
                self.in_arm_window = true;
                self.hands = Some(
                    UnitreeG1Dex3Episode::new(UnitreeG1Dex3EpisodeConfig::default())
                        .expect("load Dex3 workcell after approach"),
                );
            } else if step.terminated || step.truncated {
                self.walk = None;
            }
        } else if let Some(hands) = &mut self.hands {
            let step = hands.step(UnitreeG1Dex3Action { advance: true });
            self.step += 1;
            self.in_arm_window = true;
            if step.observation.was_grasped {
                self.grasped = true;
            }
            if step.observation.placed {
                self.placed = true;
            }
            if step.is_done() {
                self.hands = None;
            }
        } else {
            self.step += 1;
        }
        self.observation = self.observe();
        let done = self.observation.mission_complete
            || !self.observation.upright
            || self.step >= self.max_steps
            || (self.walk.is_none()
                && self.hands.is_none()
                && !matches!(self.fault, UnitreeG1WorkbenchFault::None));
        BehaviorScenarioStep {
            observation: self.observation,
            done,
        }
    }
}

fn placeholder_observation() -> UnitreeG1WorkbenchObservation {
    UnitreeG1WorkbenchObservation {
        step: 0,
        workbench_distance_m: 0.0,
        pelvis_y_m: 0.8,
        parked: false,
        in_arm_window: false,
        upright: true,
        grasped: false,
        placed: false,
        mission_complete: false,
    }
}

fn fault_dimensions(fault: UnitreeG1WorkbenchFault) -> Vec<BehaviorDimension> {
    vec![
        BehaviorDimension {
            name: "drop_part".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                UnitreeG1WorkbenchFault::DropPart
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
        BehaviorDimension {
            name: "skip_approach".to_string(),
            value: BehaviorDimensionValue::Boolean(matches!(
                fault,
                UnitreeG1WorkbenchFault::SkipApproach
            )),
            baseline: BehaviorDimensionValue::Boolean(false),
        },
    ]
}

fn digest_two_scenes(first: &Path, second: &Path) -> Result<u64, AssetError> {
    let mut bytes = b"rne_behavior_scene_inputs_v1".to_vec();
    for scene_path in [first, second] {
        let bundle = load_scene_bundle(scene_path)?;
        for path in scene_dependency_paths(&bundle) {
            let contents = fs::read(&path).map_err(|error| AssetError::Io {
                path: path.display().to_string(),
                message: error.to_string(),
            })?;
            bytes.extend_from_slice(&(contents.len() as u64).to_le_bytes());
            bytes.extend_from_slice(&contents);
        }
    }
    Ok(stable_behavior_digest(&bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{run_behavior_scenarios, BehaviorContractStatus, BehaviorSeedStatus};

    #[test]
    fn grove_park_and_arm_window_match_documented_radii() {
        assert_eq!(G1_WORKBENCH_PARK_RADIUS_M, 0.5);
        assert_eq!(G1_WORKBENCH_ARM_WINDOW_M, 0.2);
        unitree_g1_workbench_task_spec(DEFAULT_MAX_STEPS)
            .validate()
            .expect("task spec");
        let path = crate::asset_path::bundled_asset_path(Path::new(
            "tasks/g1_workbench_mission.task.json",
        ));
        let loaded: TaskSpec =
            serde_json::from_slice(&fs::read(&path).expect("committed task spec"))
                .expect("parse task spec");
        assert_eq!(unitree_g1_workbench_task_spec(DEFAULT_MAX_STEPS), loaded);
    }

    #[test]
    fn scripted_mission_walks_then_picks_and_places() {
        let report = run_behavior_scenarios(
            "g1_workbench_success",
            [1],
            UnitreeG1WorkbenchMissionScenario::success,
        );
        assert!(report.passed(), "{report:?}");
    }

    #[test]
    fn skipping_the_walk_fails_the_park_contract() {
        let report = run_behavior_scenarios("g1_workbench_skip_approach", [1], |seed| {
            UnitreeG1WorkbenchMissionScenario::new(seed, UnitreeG1WorkbenchFault::SkipApproach)
        });
        assert_eq!(report.seeds[0].status, BehaviorSeedStatus::Failed);
        let park = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "park_within_0_5_m")
            .expect("park contract");
        assert_eq!(park.status, BehaviorContractStatus::Failed);
    }
}
