//! Converts the first OpenArm joint-loss envelope failure into a portable replay.

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

const JOINT_LOSS_PERFORMANCE_REQUIREMENT: &str = "joint_loss.maximum_controlled_joint_rmse_rad";
const COULOMB_PERFORMANCE_REQUIREMENT: &str = "coulomb.maximum_controlled_joint_rmse_rad";
const RMSE_RECOMPUTE_TOLERANCE_RAD: f64 = 1.0e-12;

#[derive(Debug, Deserialize)]
struct JointLossReport {
    kind: String,
    schema_version: u32,
    status: String,
    experiment_id: String,
    controlled_joint: String,
    outcomes: Vec<Outcome>,
}

#[derive(Debug, Deserialize)]
struct Outcome {
    backend_id: String,
    case_id: String,
    plant_viscous_damping_nm_s_per_rad: f64,
    #[serde(default)]
    plant_coulomb_friction_nm: Option<f64>,
    #[serde(default)]
    plant_coulomb_transition_velocity_rad_s: Option<f64>,
    status: String,
    trace_sha256: String,
    metrics: Metrics,
    checks: Vec<Check>,
}

#[derive(Debug, Deserialize)]
struct Metrics {
    sample_count: usize,
    tracking_rmse_rad: f64,
}

#[derive(Debug, Deserialize)]
struct Check {
    requirement_id: String,
    unit: String,
    observed: serde_json::Value,
    maximum: Option<f64>,
    status: String,
}

#[derive(Debug, Deserialize)]
struct BackendTrace {
    kind: String,
    schema_version: u32,
    backend_id: String,
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
    joint_reference_position_rad: Vec<f64>,
    physics_hash: u64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm joint-loss failure replay failed: {error:#}");
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
    let report: JointLossReport = serde_json::from_slice(&report_bytes)
        .with_context(|| format!("parse {}", report_path.display()))?;
    let trace: BackendTrace = serde_json::from_slice(&trace_bytes)
        .with_context(|| format!("parse {}", trace_path.display()))?;
    let report_sha256 = sha256(&report_bytes);
    let trace_sha256 = sha256(&trace_bytes);
    let (outcome, requirement) = validate_inputs(&report, &trace, &trace_sha256)?;
    let joint_index = 4;
    let final_observation = trace
        .observations
        .last()
        .context("joint-loss trace has no observations")?;
    let descriptor = BehaviorContractDescriptor {
        name: requirement.requirement_id.clone(),
        kind: BehaviorContractKind::Always,
        entities: vec![outcome.backend_id.clone(), report.controlled_joint.clone()],
    };
    let mut squared_error_sum = 0.0;
    let mut frames = Vec::with_capacity(trace.observations.len() + 1);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "backend_id": trace.backend_id,
            "case_id": outcome.case_id,
            "plant_viscous_damping_nm_s_per_rad": outcome.plant_viscous_damping_nm_s_per_rad,
            "plant_coulomb_friction_nm": outcome.plant_coulomb_friction_nm,
            "plant_coulomb_transition_velocity_rad_s": outcome.plant_coulomb_transition_velocity_rad_s,
            "report_sha256": report_sha256,
            "trace_sha256": trace_sha256,
            "requirement_id": requirement.requirement_id,
            "contract_status": "pending"
        }),
        state_digest: trace.initial_state_digest,
    });
    for observation in &trace.observations {
        let error = observation.joint_reference_position_rad[joint_index]
            - observation.joint_position_rad[joint_index];
        squared_error_sum += error * error;
        let running_rmse = (squared_error_sum / observation.step as f64).sqrt();
        let failed = observation.step == final_observation.step;
        frames.push(BehaviorReplayFrame {
            step: observation.step,
            sim_time_ticks: observation.sim_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "joint": report.controlled_joint,
                "joint_position_rad": observation.joint_position_rad[joint_index],
                "joint_velocity_rad_s": observation.joint_velocity_rad_s[joint_index],
                "joint_reference_position_rad": observation.joint_reference_position_rad[joint_index],
                "running_tracking_rmse_rad": running_rmse,
                "maximum_tracking_rmse_rad": requirement.maximum,
                "contract_status": if failed { "failed" } else { "pending" }
            }),
            state_digest: observation.physics_hash,
        });
    }
    let maximum = requirement
        .maximum
        .context("failed RMSE requirement has no maximum")?;
    let loss_parameters = if let (Some(friction), Some(transition)) = (
        outcome.plant_coulomb_friction_nm,
        outcome.plant_coulomb_transition_velocity_rad_s,
    ) {
        format!(
            "plant Coulomb friction {friction:.6} N*m and transition velocity {transition:.6} rad/s"
        )
    } else {
        format!(
            "plant damping {:.6} N*m*s/rad",
            outcome.plant_viscous_damping_nm_s_per_rad
        )
    };
    let message = format!(
        "joint 5 episode RMSE {:.9} {} exceeded {:.9} {} at step {} with {}",
        outcome.metrics.tracking_rmse_rad,
        requirement.unit,
        maximum,
        requirement.unit,
        final_observation.step,
        loss_parameters
    );
    let replay = BehaviorReplayArtifact::new(
        report.experiment_id.clone(),
        digest_u64(&report_bytes),
        20260824,
        trace.fixed_delta_ticks,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor.clone(),
            violation: BehaviorViolation {
                step: final_observation.step,
                sim_time_ticks: final_observation.sim_time_ticks,
                state_digest: final_observation.physics_hash,
                entities: descriptor.entities.clone(),
                message,
            },
        },
    )?;
    replay.write_json(&output_path)?;
    println!(
        "OpenArm joint-loss failure replay: backend={} damping={} friction={:?} transition={:?} requirement={} step={} report_sha256={report_sha256}",
        outcome.backend_id,
        outcome.plant_viscous_damping_nm_s_per_rad,
        outcome.plant_coulomb_friction_nm,
        outcome.plant_coulomb_transition_velocity_rad_s,
        requirement.requirement_id,
        final_observation.step
    );
    Ok(())
}

fn validate_inputs<'a>(
    report: &'a JointLossReport,
    trace: &BackendTrace,
    trace_sha256: &str,
) -> Result<(&'a Outcome, &'a Check)> {
    let performance_requirement = match report.kind.as_str() {
        "rne_openarm_joint_loss_report" => JOINT_LOSS_PERFORMANCE_REQUIREMENT,
        "rne_openarm_coulomb_friction_report" => COULOMB_PERFORMANCE_REQUIREMENT,
        _ => bail!("report is not a supported OpenArm joint-loss envelope report"),
    };
    anyhow::ensure!(
        report.schema_version == 1 && matches!(report.status.as_str(), "passed" | "needs_tuning"),
        "report is not a supported OpenArm joint-loss envelope report"
    );
    let outcome = report
        .outcomes
        .iter()
        .filter(|outcome| {
            ((report.status == "passed" && outcome.status == "expected_boundary_failure")
                || (report.status == "needs_tuning" && outcome.status == "failed"))
                && outcome.checks.iter().any(|check| {
                    check.requirement_id == performance_requirement && check.status == "failed"
                })
        })
        .min_by(|left, right| {
            loss_parameter(left)
                .total_cmp(&loss_parameter(right))
                .then_with(|| {
                    replay_backend_rank(&left.backend_id)
                        .cmp(&replay_backend_rank(&right.backend_id))
                })
                .then_with(|| left.backend_id.cmp(&right.backend_id))
        })
        .context("joint-loss report has no failed performance outcome")?;
    let requirement = outcome
        .checks
        .iter()
        .find(|check| check.requirement_id == performance_requirement)
        .context("failed outcome has no RMSE requirement")?;
    anyhow::ensure!(
        trace.kind == "rne_openarm_backend_trace"
            && trace.schema_version == 1
            && trace.backend_id == outcome.backend_id
            && outcome.trace_sha256 == trace_sha256,
        "trace is not the report's first joint-loss failure"
    );
    anyhow::ensure!(
        outcome.metrics.sample_count == trace.observations.len()
            && outcome.metrics.tracking_rmse_rad.is_finite()
            && requirement.observed.as_f64() == Some(outcome.metrics.tracking_rmse_rad)
            && requirement.maximum.is_some_and(f64::is_finite),
        "failed RMSE evidence is inconsistent"
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
                        && observation.joint_reference_position_rad.len() == 9
                }),
        "joint-loss observations are not contiguous nine-joint fixed-step evidence"
    );
    let recomputed_rmse = (trace
        .observations
        .iter()
        .map(|observation| {
            let error =
                observation.joint_reference_position_rad[4] - observation.joint_position_rad[4];
            error * error
        })
        .sum::<f64>()
        / trace.observations.len() as f64)
        .sqrt();
    anyhow::ensure!(
        (recomputed_rmse - outcome.metrics.tracking_rmse_rad).abs()
            <= RMSE_RECOMPUTE_TOLERANCE_RAD,
        "trace RMSE {recomputed_rmse:.15} differs from report RMSE {:.15} beyond {RMSE_RECOMPUTE_TOLERANCE_RAD:.1e} rad",
        outcome.metrics.tracking_rmse_rad
    );
    Ok((outcome, requirement))
}

fn loss_parameter(outcome: &Outcome) -> f64 {
    outcome
        .plant_coulomb_friction_nm
        .unwrap_or(outcome.plant_viscous_damping_nm_s_per_rad)
}

fn replay_backend_rank(backend_id: &str) -> u8 {
    match backend_id {
        "rne_rapier" => 0,
        "mujoco_native" => 1,
        _ => 2,
    }
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
