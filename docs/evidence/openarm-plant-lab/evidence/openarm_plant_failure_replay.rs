//! Converts a diagnosed OpenArm plant requirement failure into a portable replay.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
};
use serde::Deserialize;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Deserialize)]
struct PlantReport {
    kind: String,
    schema_version: u32,
    status: String,
    experiment_id: String,
    experiment_contract: ExperimentContract,
    first_failed_requirement: Option<FailedRequirement>,
    backends: Vec<BackendEvidence>,
}

#[derive(Debug, Deserialize)]
struct ExperimentContract {
    fixed_delta_ticks: u64,
}

#[derive(Debug, Deserialize)]
struct FailedRequirement {
    id: String,
    gate: String,
    unit: String,
    observed: Option<f64>,
    maximum: f64,
    first_violation_step: Option<u64>,
    settling_deadline_step: u64,
    settling_band_rad: f64,
    target_rad: f64,
    position_at_deadline_rad: f64,
}

#[derive(Debug, Deserialize)]
struct BackendEvidence {
    backend_id: String,
    trace_sha256: String,
}

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
        eprintln!("OpenArm plant failure replay failed: {error:#}");
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
    let report: PlantReport = serde_json::from_slice(&report_bytes)
        .with_context(|| format!("parse {}", report_path.display()))?;
    let trace: RapierTrace = serde_json::from_slice(&trace_bytes)
        .with_context(|| format!("parse {}", trace_path.display()))?;
    let report_sha256 = sha256(&report_bytes);
    let trace_sha256 = sha256(&trace_bytes);
    let failure = validate_inputs(&report, &trace, &trace_sha256)?;
    let failure_step = failure
        .first_violation_step
        .context("first failed plant requirement has no first_violation_step")?;
    let failure_index = usize::try_from(failure_step - 1)?;
    let failure_observation = trace
        .observations
        .get(failure_index)
        .context("plant failure step exceeds the Rapier trace")?;
    let descriptor = BehaviorContractDescriptor {
        name: failure.id.clone(),
        kind: BehaviorContractKind::Always,
        entities: vec!["rne_rapier".to_string(), "openarm_right_joint5".to_string()],
    };
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
            "requirement_id": failure.id,
            "gate": failure.gate,
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
                "joint_position_rad": observation.joint_position_rad,
                "joint_velocity_rad_s": observation.joint_velocity_rad_s,
                "joint_position_target_rad": observation.joint_position_target_rad,
                "requirement_id": failure.id,
                "settling_deadline_step": failure.settling_deadline_step,
                "settling_band_rad": failure.settling_band_rad,
                "target_rad": failure.target_rad,
                "position_at_deadline_rad": failure.position_at_deadline_rad,
                "contract_status": if failed { "failed" } else { "pending" }
            }),
            state_digest: observation.physics_hash,
        });
    }
    let message = format!(
        "joint 5 had not settled within {:.6} {} by step {}; target={:.9} rad, position={:.9} rad, band=+/-{:.9} rad",
        failure.maximum,
        failure.unit,
        failure_step,
        failure.target_rad,
        failure.position_at_deadline_rad,
        failure.settling_band_rad
    );
    let replay = BehaviorReplayArtifact::new(
        report.experiment_id.clone(),
        digest_u64(&report_bytes),
        20260824,
        report.experiment_contract.fixed_delta_ticks,
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
        "OpenArm plant failure replay: requirement={} step={} report_sha256={report_sha256}",
        failure.id, failure_step
    );
    Ok(())
}

fn validate_inputs<'a>(
    report: &'a PlantReport,
    trace: &RapierTrace,
    trace_sha256: &str,
) -> Result<&'a FailedRequirement> {
    anyhow::ensure!(
        report.kind == "rne_openarm_plant_lab_report"
            && report.schema_version == 1
            && report.status == "needs_tuning",
        "report is not a supported failing OpenArm plant report"
    );
    anyhow::ensure!(
        trace.kind == "rne_openarm_backend_trace"
            && trace.schema_version == 1
            && trace.backend_id == "rne_rapier"
            && trace.fixed_delta_ticks == report.experiment_contract.fixed_delta_ticks,
        "trace is not the report's supported Rapier plant trace"
    );
    let backend = report
        .backends
        .iter()
        .find(|backend| backend.backend_id == "rne_rapier")
        .context("report has no Rapier backend evidence")?;
    anyhow::ensure!(
        backend.trace_sha256 == trace_sha256,
        "Rapier trace digest differs from the report"
    );
    let failure = report
        .first_failed_requirement
        .as_ref()
        .context("plant report has no failed requirement")?;
    anyhow::ensure!(
        failure.gate == "closed_loop_performance"
            && failure.id.ends_with(".rne_rapier")
            && failure.observed.is_none()
            && failure.maximum.is_finite()
            && failure.maximum >= 0.0
            && failure.settling_band_rad.is_finite()
            && failure.settling_band_rad >= 0.0
            && failure.target_rad.is_finite()
            && failure.position_at_deadline_rad.is_finite()
            && failure.first_violation_step == Some(failure.settling_deadline_step),
        "first failure is not a valid Rapier settling-time failure"
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
        "Rapier observations are not contiguous nine-joint fixed-step evidence"
    );
    Ok(failure)
}

fn required_path(arguments: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    arguments
        .next()
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}

fn digest_u64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(Sha256::digest(bytes)[..8].try_into().expect("eight bytes"))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
