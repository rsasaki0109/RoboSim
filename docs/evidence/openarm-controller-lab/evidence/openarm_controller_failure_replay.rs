//! Converts the diagnosed OpenArm PID baseline failure into a portable replay.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
};
use serde::Deserialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

const JOINT_INDEX: usize = 4;

#[derive(Debug, Deserialize)]
struct RapierTrace {
    kind: String,
    schema_version: u32,
    backend_id: String,
    controller_id: String,
    fixed_delta_ticks: u64,
    initial_state_digest: u64,
    observations: Vec<TraceObservation>,
}

#[derive(Debug, Deserialize)]
struct TraceObservation {
    step: u64,
    sim_time_ticks: u64,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
    joint_position_target_rad: Vec<f64>,
    physics_hash: u64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm controller failure replay failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let mut report_path = None;
    let mut trace_path = None;
    let mut output_path = None;
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--report" => report_path = Some(required_path(&mut arguments, "--report")?),
            "--trace" => trace_path = Some(required_path(&mut arguments, "--trace")?),
            "--output" => output_path = Some(required_path(&mut arguments, "--output")?),
            other => bail!("unknown argument {other:?}"),
        }
    }
    let report_path = report_path.context("--report is required")?;
    let trace_path = trace_path.context("--trace is required")?;
    let output_path = output_path.context("--output is required")?;
    let report_bytes =
        fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?;
    let trace_bytes =
        fs::read(&trace_path).with_context(|| format!("read {}", trace_path.display()))?;
    let report: Value = serde_json::from_slice(&report_bytes)
        .with_context(|| format!("parse {}", report_path.display()))?;
    let trace: RapierTrace = serde_json::from_slice(&trace_bytes)
        .with_context(|| format!("parse {}", trace_path.display()))?;
    validate_inputs(&report, &trace, &sha256(&trace_bytes))?;
    let failure = &report["baseline_first_failure"];
    let failure_step = required_u64(failure, "first_violation_step")?;
    let failure_index = usize::try_from(failure_step - 1)?;
    let failure_observation = trace
        .observations
        .get(failure_index)
        .context("controller failure step exceeds the Rapier trace")?;
    let requirement_id = required_str(failure, "id")?;
    let descriptor = BehaviorContractDescriptor {
        name: requirement_id.to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec!["rne_rapier".to_string(), "openarm_right_joint5".to_string()],
    };
    let report_sha256 = sha256(&report_bytes);
    let trace_sha256 = sha256(&trace_bytes);
    let mut frames = Vec::with_capacity(failure_index + 2);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "backend_id": trace.backend_id,
            "controller_id": trace.controller_id,
            "report_sha256": report_sha256,
            "trace_sha256": trace_sha256,
            "requirement_id": requirement_id,
            "classification": "non_gating_baseline",
            "contract_status": "initial"
        }),
        state_digest: trace.initial_state_digest,
    });
    for observation in &trace.observations[..=failure_index] {
        let failed = observation.step == failure_step;
        frames.push(BehaviorReplayFrame {
            step: observation.step,
            sim_time_ticks: observation.sim_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "joint5_position_rad": observation.joint_position_rad[JOINT_INDEX],
                "joint5_velocity_rad_s": observation.joint_velocity_rad_s[JOINT_INDEX],
                "joint5_target_rad": observation.joint_position_target_rad[JOINT_INDEX],
                "requirement_id": requirement_id,
                "settling_deadline_step": failure_step,
                "settling_band_rad": required_f64(failure, "settling_band_rad")?,
                "contract_status": if failed { "failed" } else { "pending" }
            }),
            state_digest: observation.physics_hash,
        });
    }
    let maximum_s = required_f64(failure, "maximum")?;
    let target_rad = required_f64(failure, "target_rad")?;
    let position_rad = required_f64(failure, "position_at_violation_rad")?;
    let band_rad = required_f64(failure, "settling_band_rad")?;
    let message = format!(
        "PID joint 5 had not settled within {maximum_s:.6} s by step {failure_step}; target={target_rad:.9} rad, position={position_rad:.9} rad, band=+/-{band_rad:.9} rad"
    );
    let scenario = required_str(&report, "suite_id")?.to_string();
    let replay = BehaviorReplayArtifact::new(
        scenario,
        digest_u64(&report_bytes),
        20260824,
        trace.fixed_delta_ticks,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor.clone(),
            violation: BehaviorViolation {
                step: failure_step,
                sim_time_ticks: failure_observation.sim_time_ticks,
                state_digest: failure_observation.physics_hash,
                entities: descriptor.entities.clone(),
                message,
            },
        },
    )?;
    replay.write_json(&output_path)?;
    println!(
        "OpenArm controller failure replay: requirement={requirement_id} step={failure_step} report_sha256={report_sha256}"
    );
    Ok(())
}

fn validate_inputs(report: &Value, trace: &RapierTrace, trace_sha256: &str) -> Result<()> {
    anyhow::ensure!(
        report["kind"] == "rne_openarm_controller_comparison_report"
            && report["schema_version"] == 1
            && report["status"] == "passed"
            && report["first_failed_requirement"].is_null(),
        "report is not a supported passing controller comparison"
    );
    let failure = &report["baseline_first_failure"];
    anyhow::ensure!(
        failure["status"] == "failed"
            && failure["classification"] == "non_gating_baseline"
            && required_str(failure, "id")?.starts_with("controller.pid.")
            && required_u64(failure, "first_violation_step")?
                >= required_u64(failure, "settling_deadline_step")?,
        "controller report has no valid PID baseline failure"
    );
    anyhow::ensure!(
        trace.kind == "rne_openarm_backend_trace"
            && trace.schema_version == 1
            && trace.backend_id == "rne_rapier",
        "trace is not a supported Rapier controller trace"
    );
    let backend = report["backend_results"]
        .as_array()
        .context("controller report has no backend results")?
        .iter()
        .find(|item| item["role"] == "pid" && item["backend_id"] == "rne_rapier")
        .context("controller report has no PID Rapier result")?;
    anyhow::ensure!(
        required_str(backend, "trace_sha256")? == trace_sha256,
        "Rapier PID trace digest differs from the report"
    );
    anyhow::ensure!(
        !trace.observations.is_empty()
            && trace
                .observations
                .iter()
                .enumerate()
                .all(|(index, observation)| {
                    observation.step == index as u64 + 1
                        && observation.sim_time_ticks == observation.step * trace.fixed_delta_ticks
                        && observation.joint_position_rad.len() == 9
                        && observation.joint_velocity_rad_s.len() == 9
                        && observation.joint_position_target_rad.len() == 9
                }),
        "Rapier PID observations are not contiguous nine-joint evidence"
    );
    Ok(())
}

fn required_path(arguments: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    arguments
        .next()
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}

fn required_str<'a>(value: &'a Value, field: &str) -> Result<&'a str> {
    value[field]
        .as_str()
        .with_context(|| format!("missing string field {field}"))
}

fn required_u64(value: &Value, field: &str) -> Result<u64> {
    value[field]
        .as_u64()
        .with_context(|| format!("missing integer field {field}"))
}

fn required_f64(value: &Value, field: &str) -> Result<f64> {
    value[field]
        .as_f64()
        .filter(|number| number.is_finite())
        .with_context(|| format!("missing finite field {field}"))
}

fn digest_u64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(Sha256::digest(bytes)[..8].try_into().expect("eight bytes"))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
