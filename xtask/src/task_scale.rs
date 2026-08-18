//! Portable-task CPU reference scaling evidence.
//!
//! Wall-clock time is read only in this outer harness. The reference episode
//! advances exclusively through fixed simulation-time arithmetic.

use anyhow::{Context, Result};
use rne_ai::{
    ActionSpec, Episode, EpisodeStep, ObservationSpec, PortableBatchConfig, PortableBatchRunner,
    ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType, TensorSpec,
    TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::{DeterminismContract, DeterminismScope};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub(crate) const TASK_SCALE_REPORT_SCHEMA_VERSION: u32 = 1;
const TASK_SCALE_REPORT_KIND: &str = "rne_task_scale_report";
const DEFAULT_OUTPUT: &str = "artifacts/task-scale/report.json";
const DEFAULT_WARMUP_STEPS: u64 = 32;
const DEFAULT_MEASURED_STEPS: u64 = 256;
const BATCH_WIDTHS: [usize; 4] = [1, 16, 256, 4096];
const ROOT_SEED: u64 = 0x524e_452d_5343_414c;

#[derive(Clone, Debug)]
struct Options {
    output: PathBuf,
    warmup_steps: u64,
    measured_steps: u64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct TaskScaleReport {
    kind: String,
    schema_version: u32,
    engine_version: String,
    task_spec: TaskSpec,
    backend: String,
    precision: String,
    hardware: HardwareMetadata,
    warmup_steps: u64,
    measured_steps: u64,
    determinism_contract: DeterminismContract,
    samples: Vec<TaskScaleSample>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct HardwareMetadata {
    operating_system: String,
    architecture: String,
    cpu: String,
    logical_cpus: usize,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct TaskScaleSample {
    batch_size: usize,
    transitions: u64,
    elapsed_ns: u128,
    throughput_transitions_per_s: f64,
    lane_zero_replay_digest: u64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct ReferenceObservation {
    position_m: f64,
    velocity_m_s: f64,
}

#[derive(Clone, Debug)]
struct ReferenceEpisode {
    seed: u64,
    position_m: f64,
    velocity_m_s: f64,
    step: u64,
}

impl ReferenceEpisode {
    fn new(seed: u64) -> Self {
        Self {
            seed,
            position_m: 0.0,
            velocity_m_s: 0.0,
            step: 0,
        }
    }

    fn observation(&self) -> ReferenceObservation {
        ReferenceObservation {
            position_m: self.position_m,
            velocity_m_s: self.velocity_m_s,
        }
    }
}

impl Episode for ReferenceEpisode {
    type Observation = ReferenceObservation;
    type Action = f64;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        let signed = (self.seed >> 11) as f64 / ((1_u64 << 53) as f64) * 2.0 - 1.0;
        self.position_m = signed;
        self.velocity_m_s = 0.0;
        self.step = 0;
        EpisodeStep {
            observation: self.observation(),
            reward: -self.position_m.abs(),
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, acceleration_m_s2: Self::Action) -> EpisodeStep<Self::Observation> {
        const CONTROL_STEP_S: f64 = 0.01;
        self.velocity_m_s += acceleration_m_s2 * CONTROL_STEP_S;
        self.position_m += self.velocity_m_s * CONTROL_STEP_S;
        self.step += 1;
        EpisodeStep {
            observation: self.observation(),
            reward: -self.position_m.abs(),
            terminated: self.position_m.abs() <= 1.0e-6,
            truncated: self.step >= 1024,
        }
    }

    fn episode_index(&self) -> u32 {
        0
    }

    fn step_in_episode(&self) -> u64 {
        self.step
    }
}

pub(crate) fn task_scale(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = super::workspace_root()?;
    let options = parse_options(args, &root)?;
    let report = run_report(&BATCH_WIDTHS, options.warmup_steps, options.measured_steps)?;
    validate_report(&report, &BATCH_WIDTHS)?;
    write_report(&options.output, &report)?;
    println!(
        "task scale report ok: task={} batches={:?} output={}",
        report.task_spec.task_id,
        BATCH_WIDTHS,
        options.output.display()
    );
    Ok(())
}

fn parse_options(args: &mut impl Iterator<Item = String>, root: &Path) -> Result<Options> {
    let mut options = Options {
        output: root.join(DEFAULT_OUTPUT),
        warmup_steps: DEFAULT_WARMUP_STEPS,
        measured_steps: DEFAULT_MEASURED_STEPS,
    };
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--json" | "--output" => {
                let path = args
                    .next()
                    .ok_or_else(|| anyhow::anyhow!("{argument} requires a path"))?;
                let path = PathBuf::from(path);
                options.output = if path.is_absolute() {
                    path
                } else {
                    root.join(path)
                };
            }
            "--warmup-steps" => {
                options.warmup_steps = parse_positive(&argument, args.next())?;
            }
            "--measured-steps" => {
                options.measured_steps = parse_positive(&argument, args.next())?;
            }
            other => anyhow::bail!("unknown task-scale argument: {other}"),
        }
    }
    Ok(options)
}

fn parse_positive(argument: &str, value: Option<String>) -> Result<u64> {
    let value = value.ok_or_else(|| anyhow::anyhow!("{argument} requires a positive integer"))?;
    let parsed = value
        .parse::<u64>()
        .with_context(|| format!("invalid {argument} value {value:?}"))?;
    anyhow::ensure!(parsed > 0, "{argument} must be greater than zero");
    Ok(parsed)
}

fn run_report(widths: &[usize], warmup_steps: u64, measured_steps: u64) -> Result<TaskScaleReport> {
    anyhow::ensure!(!widths.is_empty(), "at least one batch width is required");
    let task_spec = reference_task_spec();
    task_spec.validate()?;
    let mut samples = Vec::with_capacity(widths.len());
    let mut lane_zero_digest = None;
    for &batch_size in widths {
        anyhow::ensure!(batch_size > 0, "batch width must be positive");
        run_iterations(task_spec.clone(), batch_size, warmup_steps)?;

        let started = Instant::now();
        let digest = run_iterations(task_spec.clone(), batch_size, measured_steps)?;
        let elapsed_ns = started.elapsed().as_nanos().max(1);
        let transitions = measured_steps
            .checked_mul(u64::try_from(batch_size)?)
            .context("transition count overflow")?;
        if let Some(expected) = lane_zero_digest {
            anyhow::ensure!(
                digest == expected,
                "batch width {batch_size} changed lane-zero replay digest"
            );
        } else {
            lane_zero_digest = Some(digest);
        }
        samples.push(TaskScaleSample {
            batch_size,
            transitions,
            elapsed_ns,
            throughput_transitions_per_s: transitions as f64 * 1_000_000_000.0 / elapsed_ns as f64,
            lane_zero_replay_digest: digest,
        });
    }
    let determinism_contract = DeterminismContract::exact(
        "task_scale_lane_zero_batch_width_invariance",
        DeterminismScope::new(
            "task/rne.reference.kinematic.v1/lane/0",
            [
                "episode_seed",
                "observation",
                "reward",
                "termination",
                "replay_digest",
            ],
            0,
            measured_steps + 1,
        )?,
    )?;
    Ok(TaskScaleReport {
        kind: TASK_SCALE_REPORT_KIND.to_string(),
        schema_version: TASK_SCALE_REPORT_SCHEMA_VERSION,
        engine_version: env!("CARGO_PKG_VERSION").to_string(),
        task_spec,
        backend: "rne_cpu_reference".to_string(),
        precision: "f64".to_string(),
        hardware: hardware_metadata(),
        warmup_steps,
        measured_steps,
        determinism_contract,
        samples,
    })
}

fn run_iterations(task_spec: TaskSpec, batch_size: usize, steps: u64) -> Result<u64> {
    let mut runner = PortableBatchRunner::from_task_spec(
        task_spec,
        PortableBatchConfig {
            num_envs: batch_size,
            seed: ROOT_SEED,
            auto_reset: false,
        },
        ReferenceEpisode::new,
    )?;
    runner.reset();
    let actions = vec![0.25; batch_size];
    for _ in 0..steps {
        runner.step(&actions);
    }
    runner
        .lane_replay_digest(0)
        .context("reference runner omitted lane zero")
}

fn reference_task_spec() -> TaskSpec {
    TaskSpec::new(
        "rne.reference.kinematic.v1",
        0.01,
        ObservationSpec::new(vec![
            TensorSpec::new("position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("velocity_m_s", TensorDType::F64, vec![], "m/s"),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "acceleration_m_s2",
            TensorDType::F64,
            vec![],
            "m/s^2",
        )
        .with_bounds(TensorBounds::broadcast(-1.0, 1.0))]),
        RewardSpec::weighted_sum(vec![RewardTermSpec::new("position_error_m", -1.0, "m")]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "origin_reached",
                TerminationKind::Success,
            )],
            Some(1024),
        ),
        ResetSpec::splitmix64(true),
    )
}

fn validate_report(report: &TaskScaleReport, widths: &[usize]) -> Result<()> {
    anyhow::ensure!(
        report.kind == TASK_SCALE_REPORT_KIND,
        "task-scale kind mismatch"
    );
    anyhow::ensure!(
        report.schema_version == TASK_SCALE_REPORT_SCHEMA_VERSION,
        "task-scale schema mismatch"
    );
    report.task_spec.validate()?;
    report.determinism_contract.validate()?;
    anyhow::ensure!(report.warmup_steps > 0, "warmup must be positive");
    anyhow::ensure!(report.measured_steps > 0, "measured steps must be positive");
    anyhow::ensure!(
        report
            .samples
            .iter()
            .map(|sample| sample.batch_size)
            .eq(widths.iter().copied()),
        "task-scale batch widths are not canonical"
    );
    let expected_digest = report
        .samples
        .first()
        .context("task-scale samples must not be empty")?
        .lane_zero_replay_digest;
    for sample in &report.samples {
        anyhow::ensure!(sample.elapsed_ns > 0, "elapsed_ns must be positive");
        anyhow::ensure!(
            sample.throughput_transitions_per_s.is_finite()
                && sample.throughput_transitions_per_s > 0.0,
            "throughput must be finite and positive"
        );
        anyhow::ensure!(
            sample.lane_zero_replay_digest == expected_digest,
            "lane-zero digest changed with batch width"
        );
    }
    Ok(())
}

fn hardware_metadata() -> HardwareMetadata {
    HardwareMetadata {
        operating_system: std::env::consts::OS.to_string(),
        architecture: std::env::consts::ARCH.to_string(),
        cpu: cpu_name(),
        logical_cpus: std::thread::available_parallelism()
            .map(usize::from)
            .unwrap_or(1),
    }
}

fn cpu_name() -> String {
    if let Ok(cpu) = std::env::var("PROCESSOR_IDENTIFIER") {
        if !cpu.trim().is_empty() {
            return cpu;
        }
    }
    if let Ok(cpuinfo) = fs::read_to_string("/proc/cpuinfo") {
        if let Some(name) = cpuinfo.lines().find_map(|line| {
            let (key, value) = line.split_once(':')?;
            (key.trim() == "model name").then(|| value.trim().to_string())
        }) {
            return name;
        }
    }
    "unknown".to_string()
}

fn write_report(path: &Path, report: &TaskScaleReport) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create task-scale directory {}", parent.display()))?;
    }
    let mut json = serde_json::to_string_pretty(report)?;
    json.push('\n');
    fs::write(path, json).with_context(|| format!("write task-scale report {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_scale_run_keeps_lane_zero_exact() {
        let report = run_report(&[1, 4, 32], 2, 8).expect("small task-scale report");
        validate_report(&report, &[1, 4, 32]).expect("valid report");
        assert!(report
            .samples
            .windows(2)
            .all(|pair| pair[0].lane_zero_replay_digest == pair[1].lane_zero_replay_digest));
    }

    #[test]
    fn report_v1_schema_matches_committed_golden() {
        let golden = include_str!("../../tests/golden/evidence/task-scale-report-v1.json");
        let report: TaskScaleReport = serde_json::from_str(golden).expect("parse golden report");
        validate_report(&report, &[1]).expect("valid golden report");
        assert_eq!(
            serde_json::to_string_pretty(&report).expect("serialize golden report"),
            golden.trim_end()
        );
    }
}
