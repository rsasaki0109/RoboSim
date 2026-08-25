//! Executes one content-addressed recorded playback or shadow session.

use anyhow::{bail, Context, Result};
use rne_ai::TaskSpec;
use rne_hardware_gateway::recorded::{evaluate_recorded_shadow_session, RecordedShadowSession};
use rne_hardware_gateway::HardwareMode;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("recorded/shadow session failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let mut task_path = None;
    let mut controller_path = None;
    let mut session_path = None;
    let mut output_path = None;
    let mut mode = None;
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--task" => task_path = Some(required_path(&mut args, "--task")?),
            "--controller" => controller_path = Some(required_path(&mut args, "--controller")?),
            "--session" => session_path = Some(required_path(&mut args, "--session")?),
            "--output" => output_path = Some(required_path(&mut args, "--output")?),
            "--mode" => {
                mode = Some(match required_string(&mut args, "--mode")?.as_str() {
                    "playback" => HardwareMode::Playback,
                    "shadow" => HardwareMode::Shadow,
                    other => bail!("unsupported non-actuating mode {other:?}"),
                })
            }
            other => bail!("unknown argument {other:?}"),
        }
    }
    let task_path = task_path.context("--task is required")?;
    let controller_path = controller_path.context("--controller is required")?;
    let session_path = session_path.context("--session is required")?;
    let output_path = output_path.context("--output is required")?;
    let mode = mode.context("--mode is required")?;
    let task_bytes = read(&task_path)?;
    let controller_bytes = read(&controller_path)?;
    let session_bytes = read(&session_path)?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes)
        .with_context(|| format!("parse TaskSpec {}", task_path.display()))?;
    let controller: Value = serde_json::from_slice(&controller_bytes)
        .with_context(|| format!("parse controller {}", controller_path.display()))?;
    let session: RecordedShadowSession = serde_json::from_slice(&session_bytes)
        .with_context(|| format!("parse session {}", session_path.display()))?;
    let controller_id = controller
        .get("controller_id")
        .and_then(Value::as_str)
        .context("controller artifact has no controller_id")?;
    let controller_task_id = controller
        .get("task_id")
        .and_then(Value::as_str)
        .context("controller artifact has no task_id")?;
    anyhow::ensure!(
        session.task_sha256 == sha256(&task_bytes)
            && session.controller_sha256 == sha256(&controller_bytes)
            && session.controller_id == controller_id
            && session.task_id == task.task_id
            && controller_task_id == task.task_id,
        "session, TaskSpec, and controller content bindings differ"
    );
    let report = evaluate_recorded_shadow_session(task, session, sha256(&session_bytes), mode)?;
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create output directory {}", parent.display()))?;
    }
    let json = serde_json::to_string_pretty(&report)?;
    fs::write(&output_path, format!("{json}\n"))
        .with_context(|| format!("write {}", output_path.display()))?;
    println!(
        "recorded/shadow report: mode={:?} status={} samples={} violations={} drops={} output={}",
        mode,
        report.summary.status,
        report.summary.accepted_samples,
        report.comparison.summary.violating_elements,
        report.summary.dropped_observations,
        output_path.display()
    );
    Ok(())
}

fn read(path: &PathBuf) -> Result<Vec<u8>> {
    fs::read(path).with_context(|| format!("read {}", path.display()))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn required_path(args: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    Ok(PathBuf::from(required_string(args, option)?))
}

fn required_string(args: &mut impl Iterator<Item = String>, option: &str) -> Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}
