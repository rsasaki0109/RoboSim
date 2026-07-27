//! Deterministic, backend-neutral robot behavior contracts and reports.

use rne_core::{SimClock, SimDuration, SimTime};
use serde::Serialize;
use std::collections::BTreeSet;
use std::fmt;

/// Temporal form evaluated by a [`BehaviorContract`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum BehaviorContractKind {
    /// The predicate must be true at every evaluated step.
    Always,
    /// The predicate must become true no later than the deadline.
    Eventually {
        /// Inclusive deadline in simulation-time ticks.
        within_ticks: u64,
    },
    /// The predicate must be true for this many consecutive evaluated steps.
    Consecutive {
        /// Required length of the successful streak.
        steps: u32,
    },
}

/// Invalid behavior-contract configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BehaviorContractError {
    /// Contract names must contain a non-whitespace character.
    EmptyName,
    /// Entity names must contain a non-whitespace character.
    EmptyEntity,
    /// Consecutive contracts require at least one step.
    ZeroConsecutiveSteps,
    /// A numeric contract tolerance must be finite and greater than zero.
    InvalidTolerance,
}

impl fmt::Display for BehaviorContractError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyName => formatter.write_str("behavior contract name must not be empty"),
            Self::EmptyEntity => formatter.write_str("behavior contract entity must not be empty"),
            Self::ZeroConsecutiveSteps => {
                formatter.write_str("consecutive behavior contract steps must be greater than zero")
            }
            Self::InvalidTolerance => {
                formatter.write_str("behavior contract tolerance must be finite and positive")
            }
        }
    }
}

impl std::error::Error for BehaviorContractError {}

/// Result state of one behavior contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BehaviorContractStatus {
    /// The contract was satisfied.
    Passed,
    /// The contract was violated.
    Failed,
}

/// First observed violation of a behavior contract.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BehaviorViolation {
    /// Zero-based evaluated scenario step.
    pub step: u64,
    /// Simulation timestamp represented as stable integer ticks.
    pub sim_time_ticks: u64,
    /// Entities relevant to the failed contract.
    pub entities: Vec<String>,
    /// Human-readable failure explanation.
    pub message: String,
}

/// Final result for one behavior contract.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BehaviorContractResult {
    /// Stable contract name.
    pub name: String,
    /// Temporal form that was evaluated.
    pub kind: BehaviorContractKind,
    /// Final pass/fail state.
    pub status: BehaviorContractStatus,
    /// First violation, present only when the contract failed.
    pub violation: Option<BehaviorViolation>,
}

type BehaviorPredicate<O> = Box<dyn FnMut(&O) -> bool + Send + 'static>;

/// Stateful typed predicate with an `Always`, `Eventually`, or `Consecutive` temporal form.
///
/// Predicates receive task-owned observations and never receive physics-backend handles.
pub struct BehaviorContract<O> {
    name: String,
    kind: BehaviorContractKind,
    entities: Vec<String>,
    predicate: BehaviorPredicate<O>,
    streak: u32,
    resolved: Option<BehaviorContractResult>,
}

impl<O> fmt::Debug for BehaviorContract<O> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("BehaviorContract")
            .field("name", &self.name)
            .field("kind", &self.kind)
            .field("entities", &self.entities)
            .field("streak", &self.streak)
            .field("resolved", &self.resolved)
            .finish_non_exhaustive()
    }
}

impl<O> BehaviorContract<O> {
    /// Creates an `Always` contract.
    pub fn always(
        name: impl Into<String>,
        predicate: impl FnMut(&O) -> bool + Send + 'static,
    ) -> Result<Self, BehaviorContractError> {
        Self::new(name, BehaviorContractKind::Always, predicate)
    }

    /// Creates an `Eventually` contract with an inclusive simulation-time deadline.
    pub fn eventually(
        name: impl Into<String>,
        within: SimDuration,
        predicate: impl FnMut(&O) -> bool + Send + 'static,
    ) -> Result<Self, BehaviorContractError> {
        Self::new(
            name,
            BehaviorContractKind::Eventually {
                within_ticks: within.ticks(),
            },
            predicate,
        )
    }

    /// Creates a `Consecutive` contract.
    pub fn consecutive(
        name: impl Into<String>,
        steps: u32,
        predicate: impl FnMut(&O) -> bool + Send + 'static,
    ) -> Result<Self, BehaviorContractError> {
        if steps == 0 {
            return Err(BehaviorContractError::ZeroConsecutiveSteps);
        }
        Self::new(name, BehaviorContractKind::Consecutive { steps }, predicate)
    }

    fn new(
        name: impl Into<String>,
        kind: BehaviorContractKind,
        predicate: impl FnMut(&O) -> bool + Send + 'static,
    ) -> Result<Self, BehaviorContractError> {
        let name = name.into();
        if name.trim().is_empty() {
            return Err(BehaviorContractError::EmptyName);
        }
        Ok(Self {
            name,
            kind,
            entities: Vec::new(),
            predicate: Box::new(predicate),
            streak: 0,
            resolved: None,
        })
    }

    /// Attaches stable entity names to violation diagnostics.
    pub fn with_entities(
        mut self,
        entities: impl IntoIterator<Item = impl Into<String>>,
    ) -> Result<Self, BehaviorContractError> {
        let entities = entities.into_iter().map(Into::into).collect::<Vec<_>>();
        if entities.iter().any(|entity| entity.trim().is_empty()) {
            return Err(BehaviorContractError::EmptyEntity);
        }
        self.entities = entities;
        Ok(self)
    }

    fn evaluate(&mut self, step: u64, sim_time: SimTime, observation: &O) {
        if self.resolved.is_some() {
            return;
        }
        let satisfied = (self.predicate)(observation);
        match self.kind {
            BehaviorContractKind::Always if !satisfied => {
                self.fail(step, sim_time, "predicate was false");
            }
            BehaviorContractKind::Eventually { .. } if satisfied => {
                self.pass();
            }
            BehaviorContractKind::Eventually { within_ticks }
                if sim_time.ticks() >= within_ticks =>
            {
                self.fail(
                    step,
                    sim_time,
                    "deadline elapsed before predicate became true",
                );
            }
            BehaviorContractKind::Consecutive { steps } => {
                self.streak = if satisfied {
                    self.streak.saturating_add(1).min(steps)
                } else {
                    0
                };
                if self.streak >= steps {
                    self.pass();
                }
            }
            BehaviorContractKind::Always | BehaviorContractKind::Eventually { .. } => {}
        }
    }

    fn finish(&mut self, step: u64, sim_time: SimTime) {
        if self.resolved.is_some() {
            return;
        }
        match self.kind {
            BehaviorContractKind::Always => self.pass(),
            BehaviorContractKind::Eventually { .. } => {
                self.fail(
                    step,
                    sim_time,
                    "scenario ended before predicate became true",
                );
            }
            BehaviorContractKind::Consecutive { steps } => self.fail(
                step,
                sim_time,
                format!(
                    "scenario ended with a consecutive streak of {}, required {steps}",
                    self.streak
                ),
            ),
        }
    }

    fn pass(&mut self) {
        self.resolved = Some(BehaviorContractResult {
            name: self.name.clone(),
            kind: self.kind,
            status: BehaviorContractStatus::Passed,
            violation: None,
        });
    }

    fn fail(&mut self, step: u64, sim_time: SimTime, message: impl Into<String>) {
        self.resolved = Some(BehaviorContractResult {
            name: self.name.clone(),
            kind: self.kind,
            status: BehaviorContractStatus::Failed,
            violation: Some(BehaviorViolation {
                step,
                sim_time_ticks: sim_time.ticks(),
                entities: self.entities.clone(),
                message: message.into(),
            }),
        });
    }

    fn into_result(self) -> BehaviorContractResult {
        self.resolved
            .expect("behavior contract must be finished before reporting")
    }
}

/// One headless scenario transition consumed by the behavior runner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BehaviorScenarioStep<O> {
    /// Typed observation after the transition.
    pub observation: O,
    /// Whether the scenario has ended.
    pub done: bool,
}

/// Task adapter driven by the deterministic multi-seed behavior runner.
pub trait BehaviorScenario {
    /// Task-owned observation inspected by typed predicates.
    type Observation;

    /// Fixed simulation duration represented by one call to [`Self::advance`].
    fn fixed_delta(&self) -> SimDuration;

    /// Observation at scenario step zero.
    fn initial_observation(&self) -> Self::Observation;

    /// Fresh stateful contracts for this scenario.
    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError>;

    /// Advances the scenario by exactly one fixed simulation step.
    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation>;
}

/// Aggregate state of one seeded scenario run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BehaviorSeedStatus {
    /// Every contract passed.
    Passed,
    /// At least one contract failed, or scenario setup failed.
    Failed,
}

/// Report for one deterministic seed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BehaviorSeedReport {
    /// Scenario seed.
    pub seed: u64,
    /// Final aggregate state.
    pub status: BehaviorSeedStatus,
    /// Last evaluated step.
    pub steps: u64,
    /// Final simulation-time ticks.
    pub sim_time_ticks: u64,
    /// Stable contract results in declaration order.
    pub contracts: Vec<BehaviorContractResult>,
    /// Scenario creation or contract setup error.
    pub setup_error: Option<String>,
}

/// Stable multi-seed behavior report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BehaviorReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Stable scenario name.
    pub scenario: String,
    /// Seed reports in ascending numeric order.
    pub seeds: Vec<BehaviorSeedReport>,
}

impl BehaviorReport {
    /// Current serialized report schema version.
    pub const SCHEMA_VERSION: u32 = 1;

    /// Returns true when every seed and contract passed.
    pub fn passed(&self) -> bool {
        self.seeds
            .iter()
            .all(|seed| seed.status == BehaviorSeedStatus::Passed)
    }

    /// Serializes a human-readable JSON report.
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Serializes a JUnit XML test suite suitable for CI annotations.
    pub fn to_junit_xml(&self) -> String {
        let tests = self
            .seeds
            .iter()
            .map(|seed| {
                seed.contracts
                    .len()
                    .max(usize::from(seed.setup_error.is_some()))
            })
            .sum::<usize>();
        let failures = self
            .seeds
            .iter()
            .map(|seed| {
                seed.contracts
                    .iter()
                    .filter(|contract| contract.status == BehaviorContractStatus::Failed)
                    .count()
                    + usize::from(seed.setup_error.is_some())
            })
            .sum::<usize>();
        let mut xml = format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<testsuite name=\"{}\" tests=\"{tests}\" failures=\"{failures}\">\n",
            escape_xml(&self.scenario)
        );
        for seed in &self.seeds {
            if let Some(error) = &seed.setup_error {
                xml.push_str(&format!(
                    "  <testcase classname=\"{}\" name=\"seed_{}_setup\"><failure message=\"{}\"/></testcase>\n",
                    escape_xml(&self.scenario),
                    seed.seed,
                    escape_xml(error)
                ));
            }
            for contract in &seed.contracts {
                xml.push_str(&format!(
                    "  <testcase classname=\"{}\" name=\"seed_{}_{}\"",
                    escape_xml(&self.scenario),
                    seed.seed,
                    escape_xml(&contract.name)
                ));
                if let Some(violation) = &contract.violation {
                    xml.push_str(&format!(
                        "><failure message=\"{} at step {}\"/></testcase>\n",
                        escape_xml(&violation.message),
                        violation.step
                    ));
                } else {
                    xml.push_str("/>\n");
                }
            }
        }
        xml.push_str("</testsuite>\n");
        xml
    }
}

/// Runs a scenario factory for a deterministic, ascending set of unique seeds.
pub fn run_behavior_scenarios<S, E>(
    scenario: impl Into<String>,
    seeds: impl IntoIterator<Item = u64>,
    mut factory: impl FnMut(u64) -> Result<S, E>,
) -> BehaviorReport
where
    S: BehaviorScenario,
    E: fmt::Display,
{
    let scenario = scenario.into();
    let ordered_seeds = seeds.into_iter().collect::<BTreeSet<_>>();
    let reports = ordered_seeds
        .into_iter()
        .map(|seed| match factory(seed) {
            Ok(mut scenario) => run_one_seed(seed, &mut scenario),
            Err(error) => setup_failure(seed, error.to_string()),
        })
        .collect();
    BehaviorReport {
        schema_version: BehaviorReport::SCHEMA_VERSION,
        scenario,
        seeds: reports,
    }
}

fn run_one_seed<S: BehaviorScenario>(seed: u64, scenario: &mut S) -> BehaviorSeedReport {
    let mut contracts = match scenario.contracts() {
        Ok(contracts) => contracts,
        Err(error) => return setup_failure(seed, error.to_string()),
    };
    let fixed_delta = scenario.fixed_delta();
    let mut clock = SimClock::new(fixed_delta);
    let mut step = 0;
    let initial = scenario.initial_observation();
    for contract in &mut contracts {
        contract.evaluate(step, clock.sim_time(), &initial);
    }
    loop {
        let scenario_step = scenario.advance();
        step += 1;
        let _ = clock.advance(fixed_delta);
        for contract in &mut contracts {
            contract.evaluate(step, clock.sim_time(), &scenario_step.observation);
        }
        if scenario_step.done {
            break;
        }
    }
    for contract in &mut contracts {
        contract.finish(step, clock.sim_time());
    }
    let contracts = contracts
        .into_iter()
        .map(BehaviorContract::into_result)
        .collect::<Vec<_>>();
    let status = if contracts
        .iter()
        .all(|contract| contract.status == BehaviorContractStatus::Passed)
    {
        BehaviorSeedStatus::Passed
    } else {
        BehaviorSeedStatus::Failed
    };
    BehaviorSeedReport {
        seed,
        status,
        steps: step,
        sim_time_ticks: clock.sim_time().ticks(),
        contracts,
        setup_error: None,
    }
}

fn setup_failure(seed: u64, error: String) -> BehaviorSeedReport {
    BehaviorSeedReport {
        seed,
        status: BehaviorSeedStatus::Failed,
        steps: 0,
        sim_time_ticks: 0,
        contracts: Vec::new(),
        setup_error: Some(error),
    }
}

fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Hertz;

    #[derive(Clone, Copy, Debug, PartialEq)]
    struct Sample {
        value: i32,
    }

    struct Samples {
        values: Vec<i32>,
        index: usize,
    }

    impl BehaviorScenario for Samples {
        type Observation = Sample;

        fn fixed_delta(&self) -> SimDuration {
            SimDuration::from_hertz(Hertz::new(10.0))
        }

        fn initial_observation(&self) -> Self::Observation {
            Sample {
                value: self.values[0],
            }
        }

        fn contracts(
            &self,
        ) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
            Ok(vec![
                BehaviorContract::always("non_negative", |sample: &Sample| sample.value >= 0)?,
                BehaviorContract::eventually(
                    "reaches_two",
                    SimDuration::from_ticks(200_000_000),
                    |sample: &Sample| sample.value == 2,
                )?,
                BehaviorContract::consecutive("two_positive", 2, |sample: &Sample| {
                    sample.value > 0
                })?,
            ])
        }

        fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
            self.index += 1;
            BehaviorScenarioStep {
                observation: Sample {
                    value: self.values[self.index],
                },
                done: self.index + 1 == self.values.len(),
            }
        }
    }

    #[test]
    fn evaluators_cover_step_zero_deadline_and_consecutive_reset() {
        let report = run_behavior_scenarios("samples", [7], |_| {
            Ok::<_, &str>(Samples {
                values: vec![0, 1, 0, 1],
                index: 0,
            })
        });
        assert!(!report.passed());
        let results = &report.seeds[0].contracts;
        assert_eq!(results[0].status, BehaviorContractStatus::Passed);
        assert_eq!(results[1].status, BehaviorContractStatus::Failed);
        assert_eq!(results[1].violation.as_ref().expect("violation").step, 2);
        assert_eq!(results[2].status, BehaviorContractStatus::Failed);

        let mut zero_deadline =
            BehaviorContract::eventually("immediate", SimDuration::ZERO, |value: &bool| *value)
                .expect("valid contract");
        zero_deadline.evaluate(0, SimTime::ZERO, &false);
        assert_eq!(
            zero_deadline.into_result().status,
            BehaviorContractStatus::Failed
        );
    }

    #[test]
    fn invalid_contract_configuration_is_rejected() {
        assert!(matches!(
            BehaviorContract::<bool>::always(" ", |_| true),
            Err(BehaviorContractError::EmptyName)
        ));
        assert!(matches!(
            BehaviorContract::<bool>::consecutive("valid", 0, |_| true),
            Err(BehaviorContractError::ZeroConsecutiveSteps)
        ));
        assert!(matches!(
            BehaviorContract::<bool>::always("valid", |_| true)
                .expect("contract")
                .with_entities([""]),
            Err(BehaviorContractError::EmptyEntity)
        ));
    }

    #[test]
    fn first_violation_is_preserved() {
        let mut contract =
            BehaviorContract::always("first_failure", |value: &bool| *value).expect("contract");
        contract.evaluate(0, SimTime::ZERO, &false);
        contract.evaluate(1, SimTime::from_ticks(10), &false);
        contract.finish(2, SimTime::from_ticks(20));
        assert_eq!(
            contract
                .into_result()
                .violation
                .expect("first violation")
                .step,
            0
        );
    }

    #[test]
    fn seeds_and_contracts_have_stable_order() {
        let report = run_behavior_scenarios("ordered", [9, 2, 9, 4], |_| {
            Ok::<_, &str>(Samples {
                values: vec![0, 1, 2],
                index: 0,
            })
        });
        assert_eq!(
            report
                .seeds
                .iter()
                .map(|seed| seed.seed)
                .collect::<Vec<_>>(),
            vec![2, 4, 9]
        );
        assert_eq!(report.seeds[0].contracts[0].name, "non_negative");
        assert_eq!(report.seeds[0].contracts[1].name, "reaches_two");
        assert_eq!(report.seeds[0].contracts[2].name, "two_positive");
    }

    #[test]
    fn json_and_junit_reports_are_stable_and_escape_xml() {
        let report = run_behavior_scenarios("sample<&", [3], |_| {
            Ok::<_, &str>(Samples {
                values: vec![0, 1, 2],
                index: 0,
            })
        });
        let json = report.to_json_pretty().expect("JSON report");
        assert!(json.contains("\"schema_version\": 1"));
        assert!(json.contains("\"scenario\": \"sample<&\""));
        assert_eq!(
            report.to_junit_xml(),
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<testsuite name=\"sample&lt;&amp;\" tests=\"3\" failures=\"0\">\n  \
<testcase classname=\"sample&lt;&amp;\" name=\"seed_3_non_negative\"/>\n  \
<testcase classname=\"sample&lt;&amp;\" name=\"seed_3_reaches_two\"/>\n  \
<testcase classname=\"sample&lt;&amp;\" name=\"seed_3_two_positive\"/>\n\
</testsuite>\n"
        );
    }
}
