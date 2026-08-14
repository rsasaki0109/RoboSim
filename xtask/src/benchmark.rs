//! Deterministic aggregation of the existing physics and scenario benchmarks.
//!
//! The benchmark command is an evidence boundary. It consumes the reports
//! emitted by `physics-conformance` and `scenario-scale`; it does not execute
//! either benchmark implementation itself. Volatile timing samples are kept
//! in a separate optional report so the stable report can be compared byte for
//! byte across hosts and repetitions.

use anyhow::{Context, Result};
use rne_core::{DeterminismContract, DeterminismScope};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

/// Stable schema version for the aggregated benchmark report.
pub(crate) const BENCHMARK_REPORT_SCHEMA_VERSION: u32 = 1;
/// Stable artifact discriminator for the aggregated benchmark report.
pub(crate) const BENCHMARK_REPORT_KIND: &str = "rne_benchmark_report";
const TIMINGS_REPORT_KIND: &str = "rne_benchmark_timings";
const DEFAULT_REPORT_PATH: &str = "artifacts/benchmarks/report.json";
const DEFAULT_TIMINGS_PATH: &str = "artifacts/benchmarks/timings.json";
const NANOS_PER_SECOND: f64 = 1_000_000_000.0;
// SimDuration::from_hertz(Hertz::new(60.0)) uses integer nanosecond division.
const DEFAULT_FIXED_DELTA_TICKS: u64 = 16_666_666;
const PHYSICS_EVIDENCE: &[&str] = &[
    "docs/PLAN_PHYSICS_CONFORMANCE.md",
    "tests/physics_conformance/src/lib.rs",
];
const SCENARIO_EVIDENCE: &[&str] = &[
    "assets/scenarios/urban_scale_100.xosc",
    "assets/traffic/urban_scale_corridor.rne.traffic.json",
    "docs/PLAN_SCENARIO_TRAFFIC_SCALE.md",
    "tests/scenario_scale/src/lib.rs",
];

#[derive(Debug, Clone)]
struct BenchmarkOptions {
    physics_report: PathBuf,
    scenario_report: PathBuf,
    output: PathBuf,
    timings_output: Option<PathBuf>,
    generate_missing: bool,
}

/// Stable, timing-free aggregate of benchmark evidence.
#[derive(Debug, Clone, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BenchmarkReport {
    kind: String,
    schema_version: u32,
    engine_version: String,
    cases: Vec<BenchmarkCase>,
    evidence: Vec<BenchmarkEvidence>,
    content_digest: String,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BenchmarkCase {
    id: String,
    source: String,
    backend: String,
    seed: Option<u64>,
    steps: u64,
    fixed_delta_ticks: u64,
    #[serde(rename = "fixed_delta_s")]
    fixed_delta_s: f64,
    state_digest: String,
    result_digest: String,
    determinism_contract: DeterminismContract,
    evidence_refs: Vec<String>,
    passed: bool,
}

#[derive(Debug, Clone, Deserialize, Eq, PartialEq, Ord, PartialOrd, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BenchmarkEvidence {
    path: String,
    sha256: String,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct TimingsReport {
    kind: &'static str,
    schema_version: u32,
    benchmark_class: Option<String>,
    samples: Vec<TimingSample>,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct TimingSample {
    repetition: u64,
    elapsed_ns: u128,
    throughput_steps_per_s: f64,
    stable_hash: String,
    result_digest: String,
}

/// Runs the existing benchmark producers when necessary and writes the stable aggregate.
pub(crate) fn benchmark(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = super::workspace_root()?;
    let options = parse_options(args, &root)?;

    ensure_input_report(
        &root,
        &options.physics_report,
        "physics-conformance",
        options.generate_missing,
    )?;
    ensure_input_report(
        &root,
        &options.scenario_report,
        "scenario-scale",
        options.generate_missing,
    )?;

    let physics = read_json(&options.physics_report)?;
    let scenario = read_json(&options.scenario_report)?;
    let report = build_report_from_inputs(&physics, &scenario, &root)?;
    validate_report(&report)?;
    write_json(&options.output, &report)?;

    if let Some(timings_output) = options.timings_output {
        let timings = build_timings_report(&scenario)?;
        write_json(&timings_output, &timings)?;
    }

    println!(
        "benchmark report ok: cases={} output={}",
        report.cases.len(),
        options.output.display()
    );
    Ok(())
}

fn parse_options(args: &mut impl Iterator<Item = String>, root: &Path) -> Result<BenchmarkOptions> {
    let mut physics_report = root.join("artifacts/physics-conformance/report.json");
    let mut scenario_report = root.join("artifacts/scenario-scale/report.json");
    let mut output = root.join(DEFAULT_REPORT_PATH);
    let mut timings_output = None;
    let mut generate_missing = true;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--physics-report" | "--physics" => {
                physics_report = absolute_from(
                    root,
                    PathBuf::from(
                        args.next()
                            .ok_or_else(|| anyhow::anyhow!("{argument} requires a path"))?,
                    ),
                );
            }
            "--scenario-report" | "--scenario" => {
                scenario_report = absolute_from(
                    root,
                    PathBuf::from(
                        args.next()
                            .ok_or_else(|| anyhow::anyhow!("{argument} requires a path"))?,
                    ),
                );
            }
            "--output" | "--json" => {
                output = absolute_from(
                    root,
                    PathBuf::from(
                        args.next()
                            .ok_or_else(|| anyhow::anyhow!("{argument} requires a path"))?,
                    ),
                );
            }
            "--timings" => timings_output = Some(root.join(DEFAULT_TIMINGS_PATH)),
            "--timings-output" => {
                timings_output = Some(absolute_from(
                    root,
                    PathBuf::from(
                        args.next()
                            .ok_or_else(|| anyhow::anyhow!("--timings-output requires a path"))?,
                    ),
                ));
            }
            "--no-generate" => generate_missing = false,
            other => anyhow::bail!("unknown benchmark argument: {other}"),
        }
    }

    Ok(BenchmarkOptions {
        physics_report,
        scenario_report,
        output,
        timings_output,
        generate_missing,
    })
}

fn ensure_input_report(
    root: &Path,
    report_path: &Path,
    command: &str,
    generate_missing: bool,
) -> Result<()> {
    if report_path.is_file() {
        return Ok(());
    }
    anyhow::ensure!(
        generate_missing,
        "benchmark input report {} is missing (omit --no-generate to create it)",
        report_path.display()
    );
    if let Some(parent) = report_path.parent() {
        fs::create_dir_all(parent)?;
    }
    let status = Command::new("cargo")
        .current_dir(root)
        .args([
            "run", "--locked", "-q", "-p", "xtask", "--", command, "--json",
        ])
        .arg(report_path)
        .status()
        .with_context(|| format!("run xtask {command} producer"))?;
    anyhow::ensure!(
        status.success(),
        "xtask {command} producer failed with status {status}"
    );
    anyhow::ensure!(
        report_path.is_file(),
        "xtask {command} producer did not write {}",
        report_path.display()
    );
    Ok(())
}

fn read_json(path: &Path) -> Result<Value> {
    let bytes =
        fs::read(path).with_context(|| format!("read benchmark input {}", path.display()))?;
    serde_json::from_slice(&bytes)
        .with_context(|| format!("parse benchmark input {}", path.display()))
}

fn build_report_from_inputs(
    physics: &Value,
    scenario: &Value,
    root: &Path,
) -> Result<BenchmarkReport> {
    validate_source_reports(physics, scenario)?;
    let physics_evidence = load_evidence(root, PHYSICS_EVIDENCE)?;
    let scenario_evidence = load_evidence(root, SCENARIO_EVIDENCE)?;

    let mut cases = Vec::new();
    let physics_cases = physics
        .get("cases")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("physics report omitted cases"))?;
    for source_case in physics_cases {
        cases.push(build_physics_case(source_case, &physics_evidence)?);
    }
    cases.push(build_scenario_case(scenario, &scenario_evidence)?);
    cases.sort_by(|left, right| left.id.cmp(&right.id));

    let mut evidence = physics_evidence;
    evidence.extend(scenario_evidence);
    evidence.sort();
    evidence.dedup();

    let report = BenchmarkReport {
        kind: BENCHMARK_REPORT_KIND.to_string(),
        schema_version: BENCHMARK_REPORT_SCHEMA_VERSION,
        engine_version: super::RELEASE_VERSION.to_string(),
        cases,
        evidence,
        content_digest: String::new(),
    };
    finalize_report(report)
}

fn validate_source_reports(physics: &Value, scenario: &Value) -> Result<()> {
    anyhow::ensure!(
        physics.get("schema_version").and_then(Value::as_u64) == Some(1),
        "physics benchmark report must use schema_version 1"
    );
    anyhow::ensure!(
        physics.get("all_passed").and_then(Value::as_bool) == Some(true),
        "physics benchmark report did not pass"
    );
    anyhow::ensure!(
        scenario.get("schema_version").and_then(Value::as_u64) == Some(1),
        "scenario benchmark report must use schema_version 1"
    );
    anyhow::ensure!(
        scenario.get("status").and_then(Value::as_str) == Some("passed"),
        "scenario benchmark report did not pass"
    );
    Ok(())
}

fn load_evidence(root: &Path, paths: &[&str]) -> Result<Vec<BenchmarkEvidence>> {
    paths
        .iter()
        .map(|path| {
            validate_relative_path(path)?;
            let absolute = root.join(path);
            let bytes = fs::read(&absolute)
                .with_context(|| format!("read benchmark evidence {}", absolute.display()))?;
            Ok(BenchmarkEvidence {
                path: (*path).to_string(),
                sha256: sha256_digest(&bytes),
            })
        })
        .collect()
}

/// Returns the fixed-step duration as the exact nanosecond ticks used by
/// `SimDuration`, plus its derived seconds projection. Source producers may
/// provide ticks directly; seconds are rounded to the nearest nanosecond and
/// hertz uses the same rounded-rate integer division as `SimDuration::from_hertz`.
fn fixed_delta(value: &Value) -> Result<(u64, f64)> {
    if let Some(raw) = value.get("fixed_delta_ticks") {
        if !raw.is_null() {
            let ticks =
                parse_u64(raw).ok_or_else(|| anyhow::anyhow!("fixed_delta_ticks must be a u64"))?;
            return fixed_delta_from_ticks(ticks);
        }
    }
    for field in ["fixed_delta_s", "fixed_delta"] {
        if let Some(raw) = value.get(field) {
            if raw.is_null() {
                continue;
            }
            let seconds =
                parse_f64(raw).ok_or_else(|| anyhow::anyhow!("{field} must be a finite number"))?;
            return fixed_delta_from_seconds(seconds);
        }
    }
    if let Some(raw) = value.get("simulation_hz") {
        if !raw.is_null() {
            let hz = parse_f64(raw)
                .ok_or_else(|| anyhow::anyhow!("simulation_hz must be a finite number"))?;
            return fixed_delta_from_hz(hz);
        }
    }
    fixed_delta_from_ticks(DEFAULT_FIXED_DELTA_TICKS)
}

fn fixed_delta_from_ticks(ticks: u64) -> Result<(u64, f64)> {
    anyhow::ensure!(ticks > 0, "fixed delta ticks must be positive");
    Ok((ticks, ticks as f64 / NANOS_PER_SECOND))
}

fn fixed_delta_from_seconds(seconds: f64) -> Result<(u64, f64)> {
    anyhow::ensure!(
        seconds.is_finite() && seconds > 0.0,
        "fixed delta seconds must be finite and positive"
    );
    let rounded_ticks = (seconds * NANOS_PER_SECOND).round();
    anyhow::ensure!(
        rounded_ticks.is_finite() && rounded_ticks > 0.0 && rounded_ticks <= u64::MAX as f64,
        "fixed delta seconds cannot be represented as nanosecond ticks"
    );
    fixed_delta_from_ticks(rounded_ticks as u64)
}

fn fixed_delta_from_hz(hz: f64) -> Result<(u64, f64)> {
    anyhow::ensure!(
        hz.is_finite() && hz > 0.0,
        "simulation_hz must be finite and positive"
    );
    let rounded_hz = hz.round().max(1.0) as u64;
    anyhow::ensure!(rounded_hz > 0, "simulation_hz is too large");
    fixed_delta_from_ticks(1_000_000_000 / rounded_hz)
}

fn build_physics_case(
    source_case: &Value,
    evidence: &[BenchmarkEvidence],
) -> Result<BenchmarkCase> {
    let source_id = required_string(source_case, "id")?;
    let backend = required_string(source_case, "backend")?;
    let id = format!("physics/{source_id}");
    let steps = source_case
        .get("steps")
        .and_then(Value::as_u64)
        .unwrap_or_else(|| physics_default_steps(&source_id));
    let (fixed_delta_ticks, fixed_delta_s) = fixed_delta(source_case)?;
    let state_digest = source_digest(source_case, &["state_digest", "snapshot_hash"])?
        .unwrap_or_else(|| sha256_digest(&canonical_json_bytes(&stable_source_value(source_case))));
    let result_digest = source_digest(source_case, &["result_digest"])?
        .unwrap_or_else(|| sha256_digest(&canonical_json_bytes(&stable_source_value(source_case))));
    build_case(
        id,
        "physics-conformance".to_string(),
        backend,
        source_case.get("seed").and_then(parse_u64),
        steps,
        fixed_delta_ticks,
        fixed_delta_s,
        state_digest,
        result_digest,
        source_case
            .get("passed")
            .and_then(Value::as_bool)
            .unwrap_or(false),
        evidence,
    )
}

fn build_scenario_case(scenario: &Value, evidence: &[BenchmarkEvidence]) -> Result<BenchmarkCase> {
    let steps = required_u64(scenario, "steps")?;
    let (fixed_delta_ticks, fixed_delta_s) = fixed_delta(scenario)?;
    let state_digest = source_digest(scenario, &["state_digest", "stable_hash"])?
        .ok_or_else(|| anyhow::anyhow!("scenario report omitted stable_hash/state_digest"))?;
    let result_digest = source_digest(scenario, &["result_digest"])?
        .ok_or_else(|| anyhow::anyhow!("scenario report omitted result_digest"))?;
    build_case(
        "scenario/urban_scale_100".to_string(),
        "scenario-scale".to_string(),
        "native_traffic".to_string(),
        scenario.get("seed").and_then(parse_u64),
        steps,
        fixed_delta_ticks,
        fixed_delta_s,
        state_digest,
        result_digest,
        scenario.get("status").and_then(Value::as_str) == Some("passed"),
        evidence,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_case(
    id: String,
    source: String,
    backend: String,
    seed: Option<u64>,
    steps: u64,
    fixed_delta_ticks: u64,
    fixed_delta_s: f64,
    state_digest: String,
    result_digest: String,
    passed: bool,
    evidence: &[BenchmarkEvidence],
) -> Result<BenchmarkCase> {
    anyhow::ensure!(steps > 0, "benchmark case {id} must have steps");
    anyhow::ensure!(
        fixed_delta_ticks > 0,
        "benchmark case {id} must have positive fixed delta ticks"
    );
    anyhow::ensure!(
        fixed_delta_s.to_bits() == (fixed_delta_ticks as f64 / NANOS_PER_SECOND).to_bits(),
        "benchmark case {id} fixed_delta_s must be derived from nanosecond ticks"
    );
    validate_digest(&state_digest)?;
    validate_digest(&result_digest)?;
    let scope = DeterminismScope::new(
        format!("benchmark.{id}"),
        ["state_digest", "result_digest"],
        0,
        steps,
    )
    .map_err(|error| anyhow::anyhow!("build determinism scope for {id}: {error}"))?;
    let determinism_contract = DeterminismContract::exact(id.clone(), scope)
        .map_err(|error| anyhow::anyhow!("build determinism contract for {id}: {error}"))?;
    Ok(BenchmarkCase {
        id,
        source,
        backend,
        seed,
        steps,
        fixed_delta_ticks,
        fixed_delta_s,
        state_digest,
        result_digest,
        determinism_contract,
        evidence_refs: evidence.iter().map(|item| item.path.clone()).collect(),
        passed,
    })
}

fn build_timings_report(scenario: &Value) -> Result<TimingsReport> {
    let mut samples = scenario
        .get("samples")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("scenario benchmark report omitted samples"))?
        .iter()
        .map(|sample| {
            Ok(TimingSample {
                repetition: required_u64(sample, "repetition")?,
                elapsed_ns: sample
                    .get("elapsed_ns")
                    .and_then(Value::as_u64)
                    .ok_or_else(|| anyhow::anyhow!("timing sample omitted elapsed_ns"))?
                    as u128,
                throughput_steps_per_s: sample
                    .get("throughput_steps_per_s")
                    .and_then(Value::as_f64)
                    .ok_or_else(|| {
                        anyhow::anyhow!("timing sample omitted throughput_steps_per_s")
                    })?,
                stable_hash: source_digest(sample, &["stable_hash"])?
                    .ok_or_else(|| anyhow::anyhow!("timing sample omitted stable_hash"))?,
                result_digest: source_digest(sample, &["result_digest"])?
                    .ok_or_else(|| anyhow::anyhow!("timing sample omitted result_digest"))?,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    samples.sort_by_key(|sample| sample.repetition);
    Ok(TimingsReport {
        kind: TIMINGS_REPORT_KIND,
        schema_version: BENCHMARK_REPORT_SCHEMA_VERSION,
        benchmark_class: scenario
            .get("benchmark_class")
            .and_then(Value::as_str)
            .map(str::to_string),
        samples,
    })
}

fn finalize_report(mut report: BenchmarkReport) -> Result<BenchmarkReport> {
    report.content_digest = content_digest(&report)?;
    Ok(report)
}

fn validate_report(report: &BenchmarkReport) -> Result<()> {
    anyhow::ensure!(
        report.kind == BENCHMARK_REPORT_KIND,
        "benchmark report kind mismatch"
    );
    anyhow::ensure!(
        report.schema_version == BENCHMARK_REPORT_SCHEMA_VERSION,
        "benchmark report schema_version must be {BENCHMARK_REPORT_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        report.engine_version == super::RELEASE_VERSION,
        "benchmark report engine_version must be {}",
        super::RELEASE_VERSION
    );
    let case_ids = report
        .cases
        .iter()
        .map(|case| case.id.as_str())
        .collect::<Vec<_>>();
    anyhow::ensure!(
        case_ids.windows(2).all(|window| window[0] < window[1]),
        "benchmark cases must be strictly sorted by id"
    );
    let evidence_paths = report
        .evidence
        .iter()
        .map(|item| item.path.as_str())
        .collect::<Vec<_>>();
    anyhow::ensure!(
        evidence_paths
            .windows(2)
            .all(|window| window[0] < window[1]),
        "benchmark evidence must be strictly sorted by path"
    );
    let evidence_set = report
        .evidence
        .iter()
        .map(|item| item.path.as_str())
        .collect::<BTreeSet<_>>();
    for evidence in &report.evidence {
        validate_relative_path(&evidence.path)?;
        validate_sha256_digest(&evidence.sha256)?;
    }
    for case in &report.cases {
        anyhow::ensure!(
            !case.id.trim().is_empty(),
            "benchmark case id must not be empty"
        );
        anyhow::ensure!(
            !case.source.trim().is_empty(),
            "benchmark case source must not be empty"
        );
        anyhow::ensure!(
            !case.backend.trim().is_empty(),
            "benchmark case {} backend must not be empty",
            case.id
        );
        anyhow::ensure!(case.passed, "benchmark case {} did not pass", case.id);
        anyhow::ensure!(case.steps > 0, "benchmark case {} has zero steps", case.id);
        anyhow::ensure!(
            case.fixed_delta_ticks > 0,
            "benchmark case {} has invalid fixed delta ticks",
            case.id
        );
        anyhow::ensure!(
            case.fixed_delta_s.to_bits()
                == (case.fixed_delta_ticks as f64 / NANOS_PER_SECOND).to_bits(),
            "benchmark case {} fixed_delta_s does not match fixed_delta_ticks",
            case.id
        );
        validate_digest(&case.state_digest)?;
        validate_digest(&case.result_digest)?;
        case.determinism_contract
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid contract for {}: {error}", case.id))?;
        anyhow::ensure!(
            !case.evidence_refs.is_empty(),
            "benchmark case {} must reference evidence",
            case.id
        );
        anyhow::ensure!(
            case.evidence_refs
                .windows(2)
                .all(|window| window[0] < window[1]),
            "evidence refs for {} must be sorted",
            case.id
        );
        for reference in &case.evidence_refs {
            anyhow::ensure!(
                evidence_set.contains(reference.as_str()),
                "case {} references unknown evidence {}",
                case.id,
                reference
            );
        }
    }
    anyhow::ensure!(
        report.content_digest == content_digest(report)?,
        "benchmark content_digest does not match canonical content"
    );
    Ok(())
}

fn content_digest(report: &BenchmarkReport) -> Result<String> {
    let mut without_digest = serde_json::to_value(report)?;
    without_digest
        .as_object_mut()
        .expect("benchmark report serializes as an object")
        .remove("content_digest");
    Ok(sha256_digest(&canonical_json_bytes(&without_digest)))
}

fn source_digest(value: &Value, fields: &[&str]) -> Result<Option<String>> {
    for field in fields {
        if let Some(source) = value.get(*field) {
            if source.is_null() {
                continue;
            }
            return normalize_digest(source).map(Some);
        }
    }
    Ok(None)
}

/// Removes producer-only volatile fields before a missing source digest is
/// derived. This keeps fallback evidence stable if a producer adds timing or
/// host metadata without changing the deterministic case result.
fn stable_source_value(value: &Value) -> Value {
    match value {
        Value::Object(object) => {
            let mut stable = Map::new();
            for (key, value) in object {
                if matches!(
                    key.as_str(),
                    "absolute_path"
                        | "benchmark_class"
                        | "duration_ns"
                        | "elapsed_ns"
                        | "host"
                        | "host_data"
                        | "hostname"
                        | "machine"
                        | "network_path"
                        | "path"
                        | "platform"
                        | "samples"
                        | "scenario_path"
                        | "stderr"
                        | "stdout"
                        | "timestamp"
                        | "timestamps"
                        | "timings"
                        | "throughput_steps_per_s"
                ) {
                    continue;
                }
                stable.insert(key.clone(), stable_source_value(value));
            }
            Value::Object(stable)
        }
        Value::Array(values) => Value::Array(values.iter().map(stable_source_value).collect()),
        other => other.clone(),
    }
}

fn normalize_digest(value: &Value) -> Result<String> {
    if let Some(number) = value.as_u64() {
        return Ok(format!("0x{number:016x}"));
    }
    let Some(text) = value.as_str() else {
        anyhow::bail!("digest must be a u64 or string")
    };
    if let Some(hex) = text.strip_prefix("0x") {
        anyhow::ensure!(
            hex.len() == 16 && hex.chars().all(|character| character.is_ascii_hexdigit()),
            "0x digest must contain exactly 16 hexadecimal digits"
        );
        return Ok(format!("0x{}", hex.to_ascii_lowercase()));
    }
    if let Some(hex) = text.strip_prefix("sha256:") {
        anyhow::ensure!(
            hex.len() == 64 && hex.chars().all(|character| character.is_ascii_hexdigit()),
            "sha256 digest must contain exactly 64 hexadecimal digits"
        );
        return Ok(format!("sha256:{}", hex.to_ascii_lowercase()));
    }
    if let Ok(number) = text.parse::<u64>() {
        return Ok(format!("0x{number:016x}"));
    }
    anyhow::bail!("unsupported digest format")
}

fn validate_digest(digest: &str) -> Result<()> {
    let normalized = normalize_digest(&Value::String(digest.to_string()))?;
    anyhow::ensure!(
        normalized == digest,
        "digest must use canonical lowercase formatting"
    );
    Ok(())
}

fn validate_sha256_digest(digest: &str) -> Result<()> {
    anyhow::ensure!(
        digest.starts_with("sha256:")
            && digest.len() == "sha256:".len() + 64
            && digest["sha256:".len()..]
                .chars()
                .all(|character| !character.is_ascii_uppercase())
            && digest["sha256:".len()..]
                .chars()
                .all(|character| character.is_ascii_hexdigit()),
        "invalid SHA-256 digest"
    );
    Ok(())
}

fn required_string(value: &Value, field: &str) -> Result<String> {
    value
        .get(field)
        .and_then(Value::as_str)
        .filter(|text| !text.trim().is_empty())
        .map(str::to_string)
        .ok_or_else(|| anyhow::anyhow!("benchmark input omitted non-empty {field}"))
}

fn required_u64(value: &Value, field: &str) -> Result<u64> {
    value
        .get(field)
        .and_then(parse_u64)
        .ok_or_else(|| anyhow::anyhow!("benchmark input omitted u64 {field}"))
}

fn parse_u64(value: &Value) -> Option<u64> {
    value
        .as_u64()
        .or_else(|| value.as_str().and_then(|text| text.parse::<u64>().ok()))
}

fn parse_f64(value: &Value) -> Option<f64> {
    value
        .as_f64()
        .or_else(|| value.as_str().and_then(|text| text.parse::<f64>().ok()))
}

fn physics_default_steps(case_id: &str) -> u64 {
    if case_id.contains("raycast") {
        1
    } else if case_id.contains("articulation") || case_id.contains("contact") {
        180
    } else {
        60
    }
}

fn canonical_json_bytes(value: &Value) -> Vec<u8> {
    serde_json::to_vec(&canonical_value(value)).expect("JSON values are serializable")
}

fn canonical_value(value: &Value) -> Value {
    match value {
        Value::Object(object) => {
            let mut entries = object.iter().collect::<Vec<_>>();
            entries.sort_by(|left, right| left.0.cmp(right.0));
            let mut canonical = Map::new();
            for (key, value) in entries {
                canonical.insert(key.clone(), canonical_value(value));
            }
            Value::Object(canonical)
        }
        Value::Array(values) => Value::Array(values.iter().map(canonical_value).collect()),
        other => other.clone(),
    }
}

fn sha256_digest(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

fn validate_relative_path(path: &str) -> Result<()> {
    let path_object = Path::new(path);
    anyhow::ensure!(!path_object.is_absolute(), "path must be relative: {path}");
    anyhow::ensure!(!path.contains('\\'), "path must use '/' separators: {path}");
    anyhow::ensure!(
        path_object.components().all(|component| {
            !matches!(
                component,
                Component::ParentDir | Component::RootDir | Component::Prefix(_)
            )
        }),
        "path must not escape the repository: {path}"
    );
    Ok(())
}

fn absolute_from(root: &Path, path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn write_json<T: Serialize>(path: &Path, value: &T) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create benchmark output directory {}", parent.display()))?;
    }
    let mut json = serde_json::to_vec_pretty(value)?;
    json.push(b'\n');
    fs::write(path, json).with_context(|| format!("write benchmark output {}", path.display()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::tempdir;

    fn fixture_root() -> tempfile::TempDir {
        let root = tempdir().expect("fixture root");
        for path in PHYSICS_EVIDENCE.iter().chain(SCENARIO_EVIDENCE) {
            let path = root.path().join(path);
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).expect("fixture parent");
            }
            fs::write(&path, format!("fixture:{}\n", path.display())).expect("fixture evidence");
        }
        root
    }

    fn physics_fixture() -> Value {
        serde_json::json!({
            "schema_version": 1,
            "all_passed": true,
            "cases": [
                {
                    "id": "z.case",
                    "backend": "analytic",
                    "passed": true,
                    "snapshot_hash": 7,
                    "steps": 4,
                    "fixed_delta": 0.25
                },
                {
                    "id": "a.case",
                    "backend": "rapier",
                    "passed": true,
                    "snapshot_hash": 8,
                    "steps": 4,
                    "fixed_delta": 0.25
                }
            ]
        })
    }

    fn scenario_fixture(samples: Value, benchmark_class: &str) -> Value {
        serde_json::json!({
            "schema_version": 1,
            "status": "passed",
            "benchmark_class": benchmark_class,
            "steps": 4,
            "simulation_hz": 4.0,
            "stable_hash": 11,
            "result_digest": 12,
            "samples": samples
        })
    }

    #[test]
    fn report_is_byte_identical_for_reordered_inputs() {
        let root = fixture_root();
        let first = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(serde_json::json!([]), "host-a"),
            root.path(),
        )
        .expect("aggregate first report");
        let mut physics = physics_fixture();
        physics["cases"].as_array_mut().unwrap().reverse();
        let second = build_report_from_inputs(
            &physics,
            &scenario_fixture(serde_json::json!([]), "host-b"),
            root.path(),
        )
        .expect("aggregate second report");
        assert_eq!(
            serde_json::to_vec(&first).unwrap(),
            serde_json::to_vec(&second).unwrap()
        );
    }

    #[test]
    fn timing_projection_does_not_change_stable_report() {
        let root = fixture_root();
        let first = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(
                serde_json::json!([{
                    "repetition": 0,
                    "elapsed_ns": 10,
                    "throughput_steps_per_s": 400.0,
                    "stable_hash": 11,
                    "result_digest": 12
                }]),
                "host-a",
            ),
            root.path(),
        )
        .unwrap();
        let second = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(
                serde_json::json!([{
                    "repetition": 0,
                    "elapsed_ns": 999,
                    "throughput_steps_per_s": 4.0,
                    "stable_hash": 11,
                    "result_digest": 12
                }]),
                "host-b",
            ),
            root.path(),
        )
        .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn physics_digest_fallback_ignores_volatile_source_fields() {
        let root = fixture_root();
        let mut first_physics = physics_fixture();
        first_physics["cases"][0]["snapshot_hash"] = Value::Null;
        first_physics["cases"][0]["elapsed_ns"] = serde_json::json!(10);
        first_physics["cases"][0]["host"] = Value::String("host-a".to_string());
        let first = build_report_from_inputs(
            &first_physics,
            &scenario_fixture(serde_json::json!([]), "host-a"),
            root.path(),
        )
        .unwrap();

        first_physics["cases"][0]["elapsed_ns"] = serde_json::json!(999);
        first_physics["cases"][0]["host"] = Value::String("host-b".to_string());
        let second = build_report_from_inputs(
            &first_physics,
            &scenario_fixture(serde_json::json!([]), "host-b"),
            root.path(),
        )
        .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn report_has_relative_hashed_evidence() {
        let root = fixture_root();
        let report = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap();
        assert!(
            report
                .evidence
                .iter()
                .all(|item| !Path::new(&item.path).is_absolute()
                    && item.sha256.starts_with("sha256:"))
        );
    }

    #[test]
    fn content_digest_excludes_itself_and_validates() {
        let root = fixture_root();
        let report = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap();
        validate_report(&report).expect("canonical digest validates");
        let mut changed = report.clone();
        changed.content_digest =
            "sha256:0000000000000000000000000000000000000000000000000000000000000000".to_string();
        assert_eq!(
            content_digest(&report).unwrap(),
            content_digest(&changed).unwrap()
        );
        assert!(validate_report(&changed).is_err());

        let mut uppercase = report.clone();
        uppercase.cases[0].result_digest = format!(
            "sha256:{}",
            uppercase.cases[0].result_digest["sha256:".len()..].to_ascii_uppercase()
        );
        assert!(validate_report(&uppercase).is_err());
    }

    #[test]
    fn schema_and_kind_are_stable_and_unknown_fields_reject() {
        let root = fixture_root();
        let report = build_report_from_inputs(
            &physics_fixture(),
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap();
        assert_eq!(report.kind, BENCHMARK_REPORT_KIND);
        assert_eq!(report.schema_version, BENCHMARK_REPORT_SCHEMA_VERSION);
        let mut json = serde_json::to_value(report).unwrap();
        json["unexpected"] = Value::Bool(true);
        assert!(serde_json::from_value::<BenchmarkReport>(json).is_err());
    }

    #[test]
    fn invalid_source_digest_is_rejected() {
        let root = fixture_root();
        let mut physics = physics_fixture();
        physics["cases"][0]["snapshot_hash"] = Value::String("not-a-digest".to_string());
        let error = build_report_from_inputs(
            &physics,
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap_err();
        assert!(error.to_string().contains("digest"));
    }

    #[test]
    fn physics_default_steps_match_conformance_semantics() {
        assert_eq!(physics_default_steps("rigid_body.free_fall"), 60);
        assert_eq!(physics_default_steps("articulation.revolute_limit"), 180);
        assert_eq!(physics_default_steps("contact_force.resting_impulse"), 180);
        assert_eq!(physics_default_steps("raycast_batch.ordered_hits"), 1);
        assert_eq!(physics_default_steps("analytic_vs_rapier.free_fall"), 60);
    }

    #[test]
    fn fixed_delta_uses_sim_duration_nanosecond_semantics() {
        assert_eq!(
            fixed_delta(&serde_json::json!({"simulation_hz": 60.0})).unwrap(),
            (16_666_666, 0.016666666)
        );
        assert_eq!(
            fixed_delta(&serde_json::json!({"fixed_delta_ticks": 12_345})).unwrap(),
            (12_345, 0.000012345)
        );
        assert_eq!(
            fixed_delta(&serde_json::json!({
                "fixed_delta_ticks": 12_345,
                "fixed_delta_s": 0.25,
                "simulation_hz": 4.0
            }))
            .unwrap(),
            (12_345, 0.000012345)
        );
        assert_eq!(
            fixed_delta(&serde_json::json!({"fixed_delta_s": 0.25})).unwrap(),
            (250_000_000, 0.25)
        );
    }

    #[test]
    fn physics_result_fallback_includes_stable_case_semantics() {
        let root = fixture_root();
        let mut first_physics = physics_fixture();
        first_physics["cases"][0]["snapshot_hash"] = Value::Null;
        first_physics["cases"][0]["state_digest"] = Value::String("0x0000000000000007".to_string());
        first_physics["cases"][0]["metrics"] = serde_json::json!({"position_m": 1.0});
        let first = build_report_from_inputs(
            &first_physics,
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap();

        first_physics["cases"][0]["metrics"] = serde_json::json!({"position_m": 2.0});
        let second = build_report_from_inputs(
            &first_physics,
            &scenario_fixture(serde_json::json!([]), "host"),
            root.path(),
        )
        .unwrap();
        let first_digest = first
            .cases
            .iter()
            .find(|case| case.id == "physics/z.case")
            .expect("z case in first report")
            .result_digest
            .clone();
        let second_digest = second
            .cases
            .iter()
            .find(|case| case.id == "physics/z.case")
            .expect("z case in second report")
            .result_digest
            .clone();
        assert_ne!(
            first_digest, second_digest,
            "fallback result digest must include deterministic metrics"
        );
    }
}
