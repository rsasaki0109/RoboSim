//! Converts the first recorded/shadow deviation into a portable behavior replay.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
};
use rne_hardware_gateway::recorded::RecordedShadowReport;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("recorded/shadow failure replay failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let mut report_path = None;
    let mut output_path = None;
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--report" => report_path = Some(required_path(&mut args, "--report")?),
            "--output" => output_path = Some(required_path(&mut args, "--output")?),
            other => bail!("unknown argument {other:?}"),
        }
    }
    let report_path = report_path.context("--report is required")?;
    let output_path = output_path.context("--output is required")?;
    let report_bytes =
        fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?;
    let report: RecordedShadowReport = serde_json::from_slice(&report_bytes)
        .with_context(|| format!("parse {}", report_path.display()))?;
    anyhow::ensure!(
        report.kind == "rne_recorded_shadow_report"
            && report.schema_version == 1
            && report.summary.status == "failed",
        "report is not an unexpected recorded/shadow contract failure"
    );
    let (failure_index, violation) = report
        .comparison
        .samples
        .iter()
        .enumerate()
        .find_map(|(index, sample)| {
            sample
                .first_violation
                .as_ref()
                .map(|violation| (index, violation))
        })
        .context("failed report has no numeric first violation")?;
    let failed_sample = &report.comparison.samples[failure_index];
    let fixed_delta_ticks = report
        .comparison
        .samples
        .windows(2)
        .next()
        .map(|pair| pair[1].simulation_time_ticks - pair[0].simulation_time_ticks)
        .context("report needs two samples to derive the fixed step")?;
    anyhow::ensure!(
        fixed_delta_ticks > 0
            && report.comparison.samples.windows(2).all(|pair| {
                pair[1].simulation_time_ticks - pair[0].simulation_time_ticks == fixed_delta_ticks
            }),
        "shadow report simulation clock is not fixed-step"
    );
    let contract_name = format!(
        "recorded_shadow.{}.{}.{}",
        report.mode_name(),
        violation.tensor_name,
        violation.tensor_element
    );
    let descriptor = BehaviorContractDescriptor {
        name: contract_name,
        kind: BehaviorContractKind::Always,
        entities: vec![
            report.task_id.clone(),
            report.controller_id.clone(),
            violation.tensor_name.clone(),
        ],
    };
    let report_sha256 = sha256(&report_bytes);
    let mut frames = Vec::with_capacity(failure_index + 2);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "experiment_id": report.experiment_id,
            "task_id": report.task_id,
            "task_sha256": report.task_sha256,
            "controller_id": report.controller_id,
            "controller_sha256": report.controller_sha256,
            "requirements_sha256": report.requirements_sha256,
            "session_sha256": report.session_sha256,
            "report_sha256": report_sha256,
            "contract_status": "pending"
        }),
        state_digest: digest_u64(&report_bytes),
    });
    for sample in &report.comparison.samples[..=failure_index] {
        let failed = sample.hardware_sequence == failed_sample.hardware_sequence;
        frames.push(BehaviorReplayFrame {
            step: sample.simulation_step,
            sim_time_ticks: sample.simulation_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "hardware_sequence": sample.hardware_sequence,
                "hardware_received_at_ms": sample.hardware_received_at_ms,
                "recorded_values": sample.hardware_values,
                "simulation_values": sample.simulation_values,
                "max_absolute_error": sample.max_absolute_error,
                "first_violation": sample.first_violation,
                "contract_status": if failed { "failed" } else { "pending" }
            }),
            state_digest: digest_u64(&serde_json::to_vec(sample)?),
        });
    }
    let message = format!(
        "recorded/shadow {}[{}] differed by {:.9} {} at observation sequence {}; tolerance {:.9} {}",
        violation.tensor_name,
        violation.tensor_element,
        violation.absolute_error,
        violation.unit,
        failed_sample.hardware_sequence,
        violation.absolute_tolerance,
        violation.unit
    );
    let replay = BehaviorReplayArtifact::new(
        report.experiment_id,
        digest_u64(&report_bytes),
        20260826,
        fixed_delta_ticks,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor.clone(),
            violation: BehaviorViolation {
                step: failed_sample.simulation_step,
                sim_time_ticks: failed_sample.simulation_time_ticks,
                state_digest: digest_u64(&serde_json::to_vec(failed_sample)?),
                entities: descriptor.entities.clone(),
                message,
            },
        },
    )?;
    replay.write_json(&output_path)?;
    println!(
        "recorded/shadow failure replay: sequence={} tensor={} element={} report_sha256={} -> {}",
        failed_sample.hardware_sequence,
        violation.tensor_name,
        violation.tensor_element,
        report_sha256,
        output_path.display()
    );
    Ok(())
}

trait ModeName {
    fn mode_name(&self) -> &'static str;
}

impl ModeName for RecordedShadowReport {
    fn mode_name(&self) -> &'static str {
        match self.mode {
            rne_hardware_gateway::HardwareMode::Playback => "playback",
            rne_hardware_gateway::HardwareMode::Shadow => "shadow",
            rne_hardware_gateway::HardwareMode::Hil => "hil",
            rne_hardware_gateway::HardwareMode::Live => "live",
        }
    }
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn digest_u64(bytes: &[u8]) -> u64 {
    let digest = Sha256::digest(bytes);
    u64::from_le_bytes(digest[..8].try_into().expect("SHA-256 has eight bytes"))
}

fn required_path(args: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    args.next()
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}
