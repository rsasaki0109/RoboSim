use super::{
    UnitreeG1Dex3Action, UnitreeG1Dex3Episode, UnitreeG1Dex3EpisodeConfig, UnitreeG1Dex3Observation,
};
use crate::{
    BehaviorContract, BehaviorContractError, BehaviorScenario, BehaviorScenarioStep, Episode,
};
use rne_assets::AssetError;
use rne_core::SimDuration;
use rne_math::Hertz;

const PART_NAME: &str = "dex3_inspection_part";
const THUMB_NAME: &str = "right_hand_thumb_2_link";
const INDEX_NAME: &str = "right_hand_index_1_link";
const INACTIVE_PALM_NAME: &str = "left_hand_palm_link";
const TRAY_NAME: &str = "dex3_inspection_tray";

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
}

impl std::fmt::Debug for UnitreeG1Dex3BehaviorScenario {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("UnitreeG1Dex3BehaviorScenario")
            .field("config", &self.config)
            .finish_non_exhaustive()
    }
}

impl UnitreeG1Dex3BehaviorScenario {
    /// Loads the deterministic randomized G1 + Dex3 task for one seed.
    pub fn new(seed: u64, config: UnitreeG1Dex3BehaviorConfig) -> Result<Self, AssetError> {
        Ok(Self {
            episode: UnitreeG1Dex3Episode::new(UnitreeG1Dex3EpisodeConfig::randomized(seed))?,
            config,
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

    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
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
    use crate::{run_behavior_scenarios, BehaviorContractStatus};

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
            let mut episode_config = UnitreeG1Dex3EpisodeConfig::randomized(seed);
            episode_config.max_steps = 2;
            let mut episode = UnitreeG1Dex3Episode::new(episode_config)?;
            let left_palm = episode
                .simulation()
                .named_translation_m(INACTIVE_PALM_NAME)
                .expect("left palm");
            assert!(episode
                .simulation_mut()
                .set_named_body_translation_m(TRAY_NAME, [left_palm.0, left_palm.1, left_palm.2]));
            Ok::<_, AssetError>(UnitreeG1Dex3BehaviorScenario {
                episode,
                config: UnitreeG1Dex3BehaviorConfig::default(),
            })
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
}
