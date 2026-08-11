use super::{
    UnitreeG1Dex3Action, UnitreeG1Dex3Episode, UnitreeG1Dex3EpisodeConfig, UnitreeG1Dex3Observation,
};
use crate::{
    stable_behavior_digest, BehaviorContract, BehaviorContractError, BehaviorDimension,
    BehaviorDimensionValue, BehaviorScenario, BehaviorScenarioStep, Episode,
};
use rne_assets::{load_scene_bundle, scene_dependency_paths, AssetError};
use rne_core::SimDuration;
use rne_math::Hertz;
use rne_physics::hash_physics_state;
use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

const PART_NAME: &str = "dex3_inspection_part";
const THUMB_NAME: &str = "right_hand_thumb_2_link";
const INDEX_NAME: &str = "right_hand_index_1_link";
const INACTIVE_PALM_NAME: &str = "left_hand_palm_link";
const TRAY_NAME: &str = "dex3_inspection_tray";
const PART_OFFSET_DIMENSIONS: [&str; 3] = ["part_offset_x_m", "part_offset_y_m", "part_offset_z_m"];
const INVALID_TRAY_DIMENSION: &str = "tray_on_inactive_palm";

/// Contract thresholds for the G1 + Dex3 behavior scenario.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1Dex3BehaviorConfig {
    /// Inclusive deadline for acquiring a contact-confirmed grasp.
    pub grasp_deadline: SimDuration,
    /// Maximum accepted payload displacement during one 60 Hz step.
    pub max_payload_step_m: f64,
    /// Consecutive dual-contact samples required by the behavior contract.
    pub required_dual_contact_steps: u32,
}

impl Default for UnitreeG1Dex3BehaviorConfig {
    fn default() -> Self {
        Self {
            grasp_deadline: SimDuration::from_ticks(5_000_000_000),
            max_payload_step_m: 0.01,
            required_dual_contact_steps: 3,
        }
    }
}

/// Headless randomized G1 + Dex3 acquisition task adapted to behavior contracts.
pub struct UnitreeG1Dex3BehaviorScenario {
    episode: UnitreeG1Dex3Episode,
    config: UnitreeG1Dex3BehaviorConfig,
    dimensions: Vec<BehaviorDimension>,
    invalid_tray_fixture: bool,
    scenario_input_digest: u64,
}

impl std::fmt::Debug for UnitreeG1Dex3BehaviorScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("UnitreeG1Dex3BehaviorScenario")
            .field("config", &self.config)
            .field("dimensions", &self.dimensions)
            .field("invalid_tray_fixture", &self.invalid_tray_fixture)
            .field("scenario_input_digest", &self.scenario_input_digest)
            .finish_non_exhaustive()
    }
}

impl UnitreeG1Dex3BehaviorScenario {
    /// Loads the deterministic randomized G1 + Dex3 task for one seed.
    pub fn new(seed: u64, config: UnitreeG1Dex3BehaviorConfig) -> Result<Self, AssetError> {
        Self::load(UnitreeG1Dex3EpisodeConfig::randomized(seed), config, false)
    }

    /// Reconstructs a scenario from stable replay or failure-case dimensions.
    pub fn from_dimensions(
        seed: u64,
        config: UnitreeG1Dex3BehaviorConfig,
        dimensions: &[BehaviorDimension],
    ) -> Result<Self, AssetError> {
        let (part_position_override_m, invalid_tray) = decode_dimensions(dimensions)?;
        let mut episode_config = UnitreeG1Dex3EpisodeConfig::randomized(seed);
        if let Some(offset_m) = part_position_override_m {
            episode_config.part_position_jitter_m = [0.0; 3];
            episode_config.part_position_override_m = Some(offset_m);
        }
        if invalid_tray {
            episode_config.max_steps = 2;
        }
        Self::load(episode_config, config, invalid_tray)
    }

    /// Loads the deterministic committed-failure layout used by Behavior CI tests.
    pub fn invalid_tray_fixture(seed: u64) -> Result<Self, AssetError> {
        let mut episode_config = UnitreeG1Dex3EpisodeConfig::randomized(seed);
        episode_config.max_steps = 2;
        Self::load(episode_config, UnitreeG1Dex3BehaviorConfig::default(), true)
    }

    fn load(
        episode_config: UnitreeG1Dex3EpisodeConfig,
        config: UnitreeG1Dex3BehaviorConfig,
        invalid_tray: bool,
    ) -> Result<Self, AssetError> {
        let scenario_input_digest = digest_scene_inputs(&episode_config.scene_path)?;
        let mut episode = UnitreeG1Dex3Episode::new(episode_config)?;
        if invalid_tray {
            place_tray_on_inactive_palm(&mut episode)?;
        }
        let dimensions = dimensions_from_episode(&episode, invalid_tray);
        Ok(Self {
            episode,
            config,
            dimensions,
            invalid_tray_fixture: invalid_tray,
            scenario_input_digest,
        })
    }
}

impl BehaviorScenario for UnitreeG1Dex3BehaviorScenario {
    type Observation = UnitreeG1Dex3Observation;

    fn fixed_delta(&self) -> SimDuration {
        SimDuration::from_hertz(Hertz::new(60.0))
    }

    fn initial_observation(&self) -> Self::Observation {
        self.episode.current_observation()
    }

    fn state_digest(&self, _observation: &Self::Observation) -> u64 {
        hash_physics_state(self.episode.simulation().world())
    }

    fn scenario_digest(&self) -> u64 {
        let observation = self.initial_observation();
        let mut bytes = b"unitree_g1_dex3_behavior_v2".to_vec();
        bytes.extend_from_slice(&self.config.grasp_deadline.ticks().to_le_bytes());
        bytes.extend_from_slice(&self.config.max_payload_step_m.to_bits().to_le_bytes());
        bytes.extend_from_slice(&self.config.required_dual_contact_steps.to_le_bytes());
        bytes.extend_from_slice(&self.episode.simulation().world_seed().to_le_bytes());
        bytes.extend_from_slice(&self.scenario_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.state_digest(&observation).to_le_bytes());
        for value in observation.part_position_offset_m {
            bytes.extend_from_slice(&value.to_bits().to_le_bytes());
        }
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
            bytes.push(0xff);
        }
        stable_behavior_digest(&bytes)
    }

    fn behavior_dimensions(&self) -> Vec<BehaviorDimension> {
        self.dimensions.clone()
    }

    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
        if self.invalid_tray_fixture {
            return Ok(vec![BehaviorContract::always(
                "no_inactive_hand_contact",
                |observation: &UnitreeG1Dex3Observation| {
                    !observation.inactive_hand_workcell_contact
                },
            )?
            .with_entities([INACTIVE_PALM_NAME, PART_NAME, TRAY_NAME])?]);
        }
        if !self.config.max_payload_step_m.is_finite() || self.config.max_payload_step_m <= 0.0 {
            return Err(BehaviorContractError::InvalidTolerance);
        }
        let initial = self.initial_observation();
        let required_dual_contact_steps = self.config.required_dual_contact_steps;
        let mut previous_grasped_for_contact = initial.grasped;
        let mut previous_grasped_for_streak = initial.grasped;
        let mut previous_position = None;
        let max_payload_step_m = self.config.max_payload_step_m;
        Ok(vec![
            BehaviorContract::always("finite_observation", observation_is_finite)?
                .with_entities([PART_NAME])?,
            BehaviorContract::always(
                "dual_contact_before_grasp",
                move |observation: &UnitreeG1Dex3Observation| {
                    let grasp_started = !previous_grasped_for_contact && observation.grasped;
                    previous_grasped_for_contact = observation.grasped;
                    !grasp_started || observation.dual_contact
                },
            )?
            .with_entities([THUMB_NAME, INDEX_NAME, PART_NAME])?,
            BehaviorContract::always(
                "stable_contact_before_grasp",
                move |observation: &UnitreeG1Dex3Observation| {
                    let grasp_started = !previous_grasped_for_streak && observation.grasped;
                    previous_grasped_for_streak = observation.grasped;
                    !grasp_started
                        || observation.stable_contact_steps >= required_dual_contact_steps
                },
            )?
            .with_entities([THUMB_NAME, INDEX_NAME, PART_NAME])?,
            BehaviorContract::consecutive(
                "stable_contact_for_3_steps",
                self.config.required_dual_contact_steps,
                |observation: &UnitreeG1Dex3Observation| observation.dual_contact,
            )?
            .with_entities([THUMB_NAME, INDEX_NAME, PART_NAME])?,
            BehaviorContract::always(
                "no_inactive_hand_contact",
                |observation: &UnitreeG1Dex3Observation| {
                    !observation.inactive_hand_workcell_contact
                },
            )?
            .with_entities([INACTIVE_PALM_NAME, PART_NAME, TRAY_NAME])?,
            BehaviorContract::always(
                "no_tray_contact_before_place",
                |observation: &UnitreeG1Dex3Observation| !observation.working_hand_tray_contact,
            )?
            .with_entities(["right_hand_palm_link", TRAY_NAME])?,
            BehaviorContract::eventually(
                "grasp_within_5_seconds",
                self.config.grasp_deadline,
                |observation: &UnitreeG1Dex3Observation| observation.was_grasped,
            )?
            .with_entities([THUMB_NAME, INDEX_NAME, PART_NAME])?,
            BehaviorContract::always(
                "payload_never_teleports",
                move |observation: &UnitreeG1Dex3Observation| {
                    let current = observation.part_position_m;
                    let valid = previous_position.is_none_or(|previous: [f64; 3]| {
                        current
                            .into_iter()
                            .zip(previous)
                            .map(|(current, previous)| (current - previous).powi(2))
                            .sum::<f64>()
                            .sqrt()
                            <= max_payload_step_m
                    });
                    previous_position = Some(current);
                    valid
                },
            )?
            .with_entities([PART_NAME])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        let step = self.episode.step(UnitreeG1Dex3Action { advance: true });
        BehaviorScenarioStep {
            observation: step.observation,
            done: step.is_done(),
        }
    }
}

fn dimensions_from_episode(
    episode: &UnitreeG1Dex3Episode,
    invalid_tray: bool,
) -> Vec<BehaviorDimension> {
    let offset = episode.current_observation().part_position_offset_m;
    let mut dimensions = PART_OFFSET_DIMENSIONS
        .into_iter()
        .zip(offset)
        .map(|(name, value)| BehaviorDimension {
            name: name.to_string(),
            value: BehaviorDimensionValue::Number(value),
            baseline: BehaviorDimensionValue::Number(0.0),
        })
        .collect::<Vec<_>>();
    dimensions.push(BehaviorDimension {
        name: INVALID_TRAY_DIMENSION.to_string(),
        value: BehaviorDimensionValue::Boolean(invalid_tray),
        baseline: BehaviorDimensionValue::Boolean(false),
    });
    dimensions
}

fn decode_dimensions(
    dimensions: &[BehaviorDimension],
) -> Result<(Option<[f64; 3]>, bool), AssetError> {
    let mut part_offset = [None; 3];
    let mut invalid_tray = false;
    let mut names = BTreeSet::new();
    for dimension in dimensions {
        if !names.insert(dimension.name.as_str()) {
            return Err(invalid_dimensions(format!(
                "duplicate dimension `{}`",
                dimension.name
            )));
        }
        if let Some(index) = PART_OFFSET_DIMENSIONS
            .iter()
            .position(|name| *name == dimension.name)
        {
            let (BehaviorDimensionValue::Number(value), BehaviorDimensionValue::Number(baseline)) =
                (&dimension.value, &dimension.baseline)
            else {
                return Err(invalid_dimensions(format!(
                    "dimension `{}` must be numeric",
                    dimension.name
                )));
            };
            if !value.is_finite() || *baseline != 0.0 {
                return Err(invalid_dimensions(format!(
                    "dimension `{}` must be finite with a zero baseline",
                    dimension.name
                )));
            }
            part_offset[index] = Some(*value);
        } else if dimension.name == INVALID_TRAY_DIMENSION {
            let (BehaviorDimensionValue::Boolean(value), BehaviorDimensionValue::Boolean(false)) =
                (&dimension.value, &dimension.baseline)
            else {
                return Err(invalid_dimensions(format!(
                    "dimension `{INVALID_TRAY_DIMENSION}` must be boolean with a false baseline"
                )));
            };
            invalid_tray = *value;
        } else {
            return Err(invalid_dimensions(format!(
                "unknown G1 + Dex3 behavior dimension `{}`",
                dimension.name
            )));
        }
    }

    let present_offsets = part_offset.iter().filter(|value| value.is_some()).count();
    let part_offset = match present_offsets {
        0 => None,
        3 => Some(part_offset.map(|value| value.expect("all offsets present"))),
        _ => {
            return Err(invalid_dimensions(
                "part offset dimensions must provide x, y, and z together",
            ));
        }
    };
    Ok((part_offset, invalid_tray))
}

fn place_tray_on_inactive_palm(episode: &mut UnitreeG1Dex3Episode) -> Result<(), AssetError> {
    let left_palm = episode
        .simulation()
        .named_translation_m(INACTIVE_PALM_NAME)
        .ok_or_else(|| invalid_dimensions("missing inactive palm"))?;
    if episode
        .simulation_mut()
        .set_named_body_translation_m(TRAY_NAME, [left_palm.0, left_palm.1, left_palm.2])
    {
        Ok(())
    } else {
        Err(invalid_dimensions("could not move the inspection tray"))
    }
}

fn invalid_dimensions(message: impl Into<String>) -> AssetError {
    AssetError::Invalid {
        path: "unitree_g1_dex3_behavior".to_string(),
        message: message.into(),
    }
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

fn observation_is_finite(observation: &UnitreeG1Dex3Observation) -> bool {
    observation
        .part_position_m
        .into_iter()
        .chain(observation.part_position_offset_m)
        .chain([
            observation.max_part_height_m,
            observation.part_speed_m_s,
            observation.place_distance_m,
            observation.pinch_gap_m,
            observation.contact_span_m,
            observation.contact_center_error_m,
            observation.contact_opposition,
        ])
        .all(f64::is_finite)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        minimize_behavior_failure, run_behavior_scenarios, run_behavior_scenarios_with_replays,
        verify_behavior_replay, BehaviorContractStatus, BehaviorFailureCase, BehaviorReplayError,
    };

    #[test]
    fn invalid_behavior_tolerance_is_rejected() {
        let scenario = UnitreeG1Dex3BehaviorScenario::new(
            0,
            UnitreeG1Dex3BehaviorConfig {
                max_payload_step_m: f64::NAN,
                ..Default::default()
            },
        )
        .expect("scenario");
        let mut non_finite_observation = scenario.initial_observation();
        non_finite_observation.part_speed_m_s = f64::NAN;
        assert!(!observation_is_finite(&non_finite_observation));
        assert_eq!(
            scenario.contracts().expect_err("invalid tolerance"),
            BehaviorContractError::InvalidTolerance
        );
    }

    #[test]
    fn randomized_g1_dex3_behavior_contracts_pass_headlessly() {
        let report = run_behavior_scenarios("unitree_g1_dex3_acquire", [0], |seed| {
            UnitreeG1Dex3BehaviorScenario::new(seed, UnitreeG1Dex3BehaviorConfig::default())
        });
        assert!(report.passed(), "{report:#?}");
    }

    #[test]
    fn invalid_tray_layout_reports_inactive_hand_contact() {
        let report = run_behavior_scenarios("unitree_g1_dex3_invalid_tray", [0], |seed| {
            UnitreeG1Dex3BehaviorScenario::invalid_tray_fixture(seed)
        });
        assert!(!report.passed());
        let failure = report.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_inactive_hand_contact")
            .expect("inactive-hand contract");
        assert_eq!(failure.status, BehaviorContractStatus::Failed);
        assert!(failure.violation.is_some());
    }

    #[test]
    fn committed_failure_case_replays_identically_headlessly() {
        let failure_case = BehaviorFailureCase::from_json(include_str!(
            "../../../tests/fixtures/unitree_g1_dex3_invalid_tray.behavior-case.json"
        ))
        .expect("committed failure case");
        let run = run_behavior_scenarios_with_replays(
            failure_case.scenario.clone(),
            [failure_case.seed],
            |seed| {
                UnitreeG1Dex3BehaviorScenario::from_dimensions(
                    seed,
                    UnitreeG1Dex3BehaviorConfig::default(),
                    &failure_case.dimensions,
                )
            },
        )
        .expect("failure run");
        assert!(!run.report.passed());
        let replay = run.failure_replays.first().expect("failure replay");
        assert_eq!(replay.failure.contract.name, failure_case.expected_contract);
        let persisted_replay = crate::BehaviorReplayArtifact::from_json(
            &replay.to_json_pretty().expect("serialized replay"),
        )
        .expect("persisted replay");

        let verification = verify_behavior_replay(&persisted_replay, |seed, dimensions| {
            UnitreeG1Dex3BehaviorScenario::from_dimensions(
                seed,
                UnitreeG1Dex3BehaviorConfig::default(),
                dimensions,
            )
        })
        .expect("identical headless replay");
        assert_eq!(verification.contract, failure_case.expected_contract);
        assert_eq!(verification.step, persisted_replay.failure.violation.step);
        assert_eq!(
            verification.state_digest,
            persisted_replay.failure.violation.state_digest
        );
    }

    #[test]
    fn randomized_g1_failure_minimizes_to_the_required_tray_dimension() {
        let original_run = run_behavior_scenarios_with_replays(
            "unitree_g1_dex3_invalid_tray",
            [0],
            UnitreeG1Dex3BehaviorScenario::invalid_tray_fixture,
        )
        .expect("original failure run");
        let original = original_run
            .failure_replays
            .first()
            .expect("original replay");
        let minimized = minimize_behavior_failure(original, |dimensions| {
            let candidate = run_behavior_scenarios_with_replays(
                original.scenario.clone(),
                [original.seed],
                |seed| {
                    UnitreeG1Dex3BehaviorScenario::from_dimensions(
                        seed,
                        UnitreeG1Dex3BehaviorConfig::default(),
                        dimensions,
                    )
                },
            )?;
            Ok::<_, BehaviorReplayError>(candidate.failure_replays.into_iter().next())
        })
        .expect("minimized failure");

        assert_eq!(minimized.active_dimensions_before, 3);
        assert_eq!(minimized.active_dimensions_after, 1);
        assert_eq!(minimized.attempts, 3);
        assert_eq!(
            minimized.artifact.failure.contract.name,
            "no_inactive_hand_contact"
        );
        assert!(minimized.artifact.dimensions.iter().all(|dimension| {
            dimension.name == INVALID_TRAY_DIMENSION || !dimension.is_active()
        }));
        verify_behavior_replay(&minimized.artifact, |seed, dimensions| {
            UnitreeG1Dex3BehaviorScenario::from_dimensions(
                seed,
                UnitreeG1Dex3BehaviorConfig::default(),
                dimensions,
            )
        })
        .expect("minimized replay verification");
    }

    #[test]
    fn partial_part_override_is_rejected() {
        let dimensions =
            [BehaviorDimension::number("part_offset_x_m", 0.01, 0.0).expect("dimension")];
        let error = UnitreeG1Dex3BehaviorScenario::from_dimensions(
            0,
            UnitreeG1Dex3BehaviorConfig::default(),
            &dimensions,
        )
        .expect_err("partial offset");
        assert!(error.to_string().contains("x, y, and z"));
    }
}
