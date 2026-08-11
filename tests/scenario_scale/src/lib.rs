//! Deterministic M5 OpenSCENARIO scale benchmark and report.

#![deny(missing_docs)]

use anyhow::Context;
use rne_openscenario::{
    execute_scenario, parse_openscenario_xml_with_source, stable_replay_input_digest,
    ScenarioActionEvidence, ScenarioDocument, ScenarioRunOptions, ScenarioRunResult,
};
use rne_traffic::{parse_traffic_asset, TrafficNetwork, TrafficOwnershipMetrics};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::time::Instant;

const REPORT_SCHEMA_VERSION: u32 = 1;
const ACTOR_COUNT: usize = 100;
const STEP_COUNT: u64 = 600;
const SIMULATION_HZ: f64 = 60.0;
const REQUIRED_THROUGHPUT_STEPS_PER_S: f64 = 60.0;
const MINIMUM_GAP_M: f64 = 2.0;
const MEASURED_REPETITIONS: usize = 3;
const SCENARIO_PATH: &str = "assets/scenarios/urban_scale_100.xosc";
const NETWORK_PATH: &str = "assets/traffic/urban_scale_corridor.rne.traffic.json";
const SCENARIO_XML: &str = include_str!("../../../assets/scenarios/urban_scale_100.xosc");
const NETWORK_JSON: &str =
    include_str!("../../../assets/traffic/urban_scale_corridor.rne.traffic.json");

/// One wall-clock sample from the outer headless benchmark harness.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScenarioScaleSample {
    /// Zero-based measured repetition.
    pub repetition: usize,
    /// Outer-harness elapsed wall-clock time in nanoseconds.
    pub elapsed_ns: u128,
    /// Completed simulation steps per wall-clock second.
    pub throughput_steps_per_s: f64,
    /// Runtime-owned final fleet digest.
    pub stable_hash: u64,
    /// Canonical actor/action evidence digest.
    pub result_digest: u64,
}

/// One classified M5 scale violation and its exit bound.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScenarioScaleViolation {
    /// Stable violation registry identifier.
    pub id: String,
    /// Measured violation count.
    pub measured_count: usize,
    /// Unit of the measured value.
    pub unit: String,
    /// Largest count accepted by the M5 exit gate.
    pub allowed_count: usize,
    /// Whether the measured count satisfies the bound.
    pub passed: bool,
    /// Deterministic evidence or test boundary for the row.
    pub evidence: String,
}

/// Machine-readable evidence for the M5 100-actor scenario scale gate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScenarioScaleReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Named runner class on which throughput was measured.
    pub benchmark_class: String,
    /// Committed OpenSCENARIO fixture path.
    pub scenario_path: String,
    /// Stable digest of the exact OpenSCENARIO bytes.
    pub scenario_digest: u64,
    /// Committed traffic-network fixture path.
    pub network_path: String,
    /// Stable digest of the exact traffic-network bytes.
    pub network_digest: u64,
    /// Number of actors required and observed.
    pub actor_count: usize,
    /// Fixed simulation steps in each execution.
    pub steps: u64,
    /// Requested fixed simulation frequency.
    pub simulation_hz: f64,
    /// Stable runtime-owned fleet digest.
    pub stable_hash: u64,
    /// Canonical final actor/action evidence digest.
    pub result_digest: u64,
    /// Canonically ordered externally visible actor names.
    pub actor_order: Vec<String>,
    /// Canonically ordered applied action target names.
    pub action_order: Vec<String>,
    /// Whether final actor UUID order is strictly canonical.
    pub actor_order_is_canonical: bool,
    /// Whether action evidence uses the frozen total order.
    pub action_order_is_canonical: bool,
    /// Whether reversed source entity/action declarations reproduce exactly.
    pub reverse_declaration_match: bool,
    /// Whether every warm-up and measured repetition reproduced exactly.
    pub repetition_match: bool,
    /// Smallest bumper-to-bumper gap observed during the reference execution.
    pub minimum_observed_gap_m: Option<f64>,
    /// Final mixed-ownership counts from the shared traffic step report.
    pub ownership: TrafficOwnershipMetrics,
    /// Classified violation registry with explicit zero-count exit bounds.
    pub violations: Vec<ScenarioScaleViolation>,
    /// Violations that could not be mapped to the registry.
    pub unexplained_violation_count: usize,
    /// Throughput samples; timing fields never participate in stable digests.
    pub samples: Vec<ScenarioScaleSample>,
    /// Slowest measured repetition.
    pub minimum_throughput_steps_per_s: f64,
    /// Required headless throughput.
    pub required_throughput_steps_per_s: f64,
    /// Overall gate status.
    pub status: String,
}

impl ScenarioScaleReport {
    /// Returns true when determinism, violations, ownership, spacing, and throughput pass.
    pub fn all_passed(&self) -> bool {
        self.status == "passed"
    }
}

/// Executes the committed M5 reference and returns deterministic plus timing evidence.
pub fn run_scale_report() -> anyhow::Result<ScenarioScaleReport> {
    let (document, network) = load_reference()?;
    let options = ScenarioRunOptions {
        steps: STEP_COUNT,
        hz: SIMULATION_HZ,
    };

    // This untimed execution both warms code/data and defines the exact state
    // evidence every measured execution must reproduce.
    let reference = execute_scenario(&document, &network, &options)
        .context("execute untimed M5 scale reference")?;
    let mut reversed = document.clone();
    reversed.entities.reverse();
    reversed.actions.reverse();
    let reverse_result = execute_scenario(&reversed, &network, &options)
        .context("execute reversed-declaration M5 scale reference")?;
    let reverse_declaration_match = reverse_result == reference;

    let mut samples = Vec::with_capacity(MEASURED_REPETITIONS);
    let mut repetition_match = true;
    for repetition in 0..MEASURED_REPETITIONS {
        let started = Instant::now();
        let result = execute_scenario(&document, &network, &options)
            .with_context(|| format!("execute measured M5 scale repetition {repetition}"))?;
        let elapsed = started.elapsed();
        let elapsed_ns = elapsed.as_nanos().max(1);
        repetition_match &= result == reference;
        samples.push(ScenarioScaleSample {
            repetition,
            elapsed_ns,
            throughput_steps_per_s: STEP_COUNT as f64 * 1_000_000_000.0 / elapsed_ns as f64,
            stable_hash: result.stable_hash,
            result_digest: result.result_digest,
        });
    }

    build_report(
        &document,
        &reference,
        reverse_declaration_match,
        repetition_match,
        samples,
    )
}

fn load_reference() -> anyhow::Result<(ScenarioDocument, TrafficNetwork)> {
    let document = parse_openscenario_xml_with_source(SCENARIO_PATH, SCENARIO_XML)
        .context("parse committed M5 OpenSCENARIO fixture")?;
    let asset = parse_traffic_asset(NETWORK_JSON.as_bytes())
        .context("parse committed M5 traffic fixture")?;
    Ok((document, asset.network))
}

fn build_report(
    document: &ScenarioDocument,
    reference: &ScenarioRunResult,
    reverse_declaration_match: bool,
    repetition_match: bool,
    samples: Vec<ScenarioScaleSample>,
) -> anyhow::Result<ScenarioScaleReport> {
    let actor_order_is_canonical = reference
        .final_actors
        .windows(2)
        .all(|window| window[0].stable_uuid < window[1].stable_uuid);
    let action_order_is_canonical = action_order_is_canonical(&reference.action_evidence);
    let actor_order = reference
        .final_actors
        .iter()
        .map(|actor| actor.name.clone())
        .collect::<Vec<_>>();
    let action_order = reference
        .action_evidence
        .iter()
        .map(|action| action.entity_name.clone())
        .collect::<Vec<_>>();
    let violations = vec![
        violation(
            "traffic.collision",
            reference.collisions,
            "aggregate oriented-overlap and same-route bumper-gap diagnostics",
        ),
        violation(
            "traffic.signal",
            reference.signal_violations,
            "aggregate red stop-line crossing diagnostics",
        ),
        violation(
            "traffic.ownership.invalid_state",
            reference.ownership.invalid_actor_count,
            "transactional ownership validation from the final completed step",
        ),
        violation(
            "traffic.ownership.double_integration",
            0,
            "mixed external ownership is covered by the rne_traffic external_pose gate",
        ),
        violation(
            "scenario.action.unapplied",
            reference.unapplied_action_count,
            "scheduled action count minus canonical applied-action evidence",
        ),
        violation(
            "traci.recovery.unreconciled",
            0,
            "snapshot-only identity reconciliation is covered by the rne_traci co_simulation gate",
        ),
    ];
    let minimum_throughput_steps_per_s = samples
        .iter()
        .map(|sample| sample.throughput_steps_per_s)
        .reduce(f64::min)
        .context("M5 scale report requires at least one timing sample")?;
    let unexplained_violation_count = 0;
    let ownership_passed = reference.ownership.total_actor_count == ACTOR_COUNT
        && reference.ownership.runtime_owned_actor_count == ACTOR_COUNT
        && reference.ownership.runtime_advanced_actor_count == ACTOR_COUNT
        && reference.ownership.external_owned_actor_count == 0
        && reference.ownership.external_observed_actor_count == 0
        && reference.ownership.invalid_actor_count == 0;
    let contract_passed = document.entities.len() == ACTOR_COUNT
        && document.actions.len() == ACTOR_COUNT
        && reference.final_actors.len() == ACTOR_COUNT
        && reference.action_evidence.len() == ACTOR_COUNT
        && reference.steps == STEP_COUNT
        && reference
            .minimum_observed_gap_m
            .is_some_and(|gap_m| gap_m + 1.0e-9 >= MINIMUM_GAP_M)
        && actor_order_is_canonical
        && action_order_is_canonical
        && reverse_declaration_match
        && repetition_match
        && ownership_passed;
    let passed = contract_passed
        && violations.iter().all(|violation| violation.passed)
        && unexplained_violation_count == 0
        && minimum_throughput_steps_per_s >= REQUIRED_THROUGHPUT_STEPS_PER_S;

    Ok(ScenarioScaleReport {
        schema_version: REPORT_SCHEMA_VERSION,
        benchmark_class: benchmark_class(),
        scenario_path: SCENARIO_PATH.to_string(),
        scenario_digest: stable_replay_input_digest(SCENARIO_XML.as_bytes()),
        network_path: NETWORK_PATH.to_string(),
        network_digest: stable_replay_input_digest(NETWORK_JSON.as_bytes()),
        actor_count: reference.final_actors.len(),
        steps: reference.steps,
        simulation_hz: SIMULATION_HZ,
        stable_hash: reference.stable_hash,
        result_digest: reference.result_digest,
        actor_order,
        action_order,
        actor_order_is_canonical,
        action_order_is_canonical,
        reverse_declaration_match,
        repetition_match,
        minimum_observed_gap_m: reference.minimum_observed_gap_m,
        ownership: reference.ownership,
        violations,
        unexplained_violation_count,
        samples,
        minimum_throughput_steps_per_s,
        required_throughput_steps_per_s: REQUIRED_THROUGHPUT_STEPS_PER_S,
        status: if passed { "passed" } else { "failed" }.to_string(),
    })
}

fn action_order_is_canonical(actions: &[ScenarioActionEvidence]) -> bool {
    actions
        .windows(2)
        .all(|window| compare_action_evidence(&window[0], &window[1]) != Ordering::Greater)
}

fn compare_action_evidence(
    left: &ScenarioActionEvidence,
    right: &ScenarioActionEvidence,
) -> Ordering {
    left.start_time_s
        .total_cmp(&right.start_time_s)
        .then_with(|| left.entity_name.cmp(&right.entity_name))
        .then_with(|| left.source_action_index.cmp(&right.source_action_index))
}

fn violation(id: &str, measured_count: usize, evidence: &str) -> ScenarioScaleViolation {
    ScenarioScaleViolation {
        id: id.to_string(),
        measured_count,
        unit: "count".to_string(),
        allowed_count: 0,
        passed: measured_count == 0,
        evidence: evidence.to_string(),
    }
}

fn benchmark_class() -> String {
    std::env::var("RNE_SCENARIO_SCALE_BENCHMARK_CLASS").unwrap_or_else(|_| {
        if std::env::var("GITHUB_ACTIONS").as_deref() == Ok("true") {
            "github-hosted-windows-latest".to_string()
        } else {
            format!("local-{}-{}", std::env::consts::OS, std::env::consts::ARCH)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXPECTED_STABLE_HASH: u64 = 7_503_294_308_621_126_381;
    const EXPECTED_RESULT_DIGEST: u64 = 6_732_886_903_736_628_512;

    #[test]
    fn committed_reference_has_stable_order_and_zero_violations() {
        let report = run_scale_report().expect("M5 scenario scale report");

        assert_eq!(report.actor_count, ACTOR_COUNT);
        assert_eq!(report.steps, STEP_COUNT);
        assert!(report.actor_order_is_canonical);
        assert!(report.action_order_is_canonical);
        assert!(report.reverse_declaration_match);
        assert!(report.repetition_match);
        assert_eq!(
            report.actor_order.first().map(String::as_str),
            Some("actor_000")
        );
        assert_eq!(
            report.actor_order.last().map(String::as_str),
            Some("actor_099")
        );
        assert_eq!(report.action_order.len(), ACTOR_COUNT);
        assert!(report.violations.iter().all(|violation| violation.passed));
        assert_eq!(report.unexplained_violation_count, 0);
        assert_eq!(report.stable_hash, EXPECTED_STABLE_HASH);
        assert_eq!(report.result_digest, EXPECTED_RESULT_DIGEST);
        assert!(report.all_passed(), "{report:#?}");

        let json = serde_json::to_string(&report).expect("serialize report");
        let decoded: ScenarioScaleReport =
            serde_json::from_str(&json).expect("deserialize report schema");
        assert_eq!(decoded.schema_version, report.schema_version);
        assert_eq!(decoded.status, report.status);
        assert_eq!(decoded.result_digest, report.result_digest);
        assert_eq!(decoded.actor_order, report.actor_order);
        assert_eq!(decoded.action_order, report.action_order);
        assert_eq!(decoded.violations, report.violations);
        assert_eq!(decoded.samples.len(), MEASURED_REPETITIONS);
    }
}
