//! Standalone conformance runner for external fixed-step simulator processes.

use super::wire::{
    SimulatorAdapterFrame, SimulatorAdapterPayload, SimulatorHostFrame, SimulatorHostPayload,
    SimulatorRejectionCode, SimulatorWireCodec, SIMULATOR_WIRE_SCHEMA_VERSION,
};
use super::{SimulatorArtifactRole, SimulatorRuntimeArtifact, SimulatorRuntimeManifest};
use crate::{GatewayConfig, HardwareGateway};
use rne_ai::TaskSpec;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::ffi::OsString;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Stable kind for external simulator-adapter conformance reports.
pub const SIMULATOR_ADAPTER_CONFORMANCE_REPORT_KIND: &str =
    "rne_simulator_adapter_conformance_report";

/// Current external simulator-adapter conformance report schema.
pub const SIMULATOR_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION: u32 = 1;

const MAX_RESPONSE_TIMEOUT_MS: u64 = 60_000;
const NS_PER_SECOND: f64 = 1_000_000_000.0;
const CHECK_IDS: [&str; 10] = [
    "open_identity",
    "task_binding",
    "fixed_delta_binding",
    "reset_origin",
    "bounded_step",
    "fixed_step_progression",
    "deterministic_replay",
    "action_sequence_rejection",
    "session_isolation",
    "width_rejection",
];

/// Process launch and content-addressed runtime configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SimulatorAdapterConformanceConfig {
    /// Executable used to launch each fresh adapter process.
    pub program: PathBuf,
    /// Exact arguments supplied to every adapter process.
    pub arguments: Vec<OsString>,
    /// Adapter implementation file whose bytes identify the tested subject.
    pub subject: PathBuf,
    /// Runtime manifest beside its world, robot model, and adapter config files.
    pub runtime_manifest: PathBuf,
    /// Maximum wait for one response and clean process exit.
    pub response_timeout_ms: u64,
}

impl SimulatorAdapterConformanceConfig {
    /// Creates a configuration using the executable as its implementation subject.
    pub fn new(program: impl Into<PathBuf>, runtime_manifest: impl Into<PathBuf>) -> Self {
        let program = program.into();
        Self {
            subject: program.clone(),
            program,
            arguments: Vec::new(),
            runtime_manifest: runtime_manifest.into(),
            response_timeout_ms: 5_000,
        }
    }

    fn validate(&self) -> Result<(), SimulatorAdapterConformanceError> {
        if self.program.as_os_str().is_empty() {
            return Err(SimulatorAdapterConformanceError::InvalidConfig(
                "adapter program is empty".to_string(),
            ));
        }
        if self.subject.as_os_str().is_empty() {
            return Err(SimulatorAdapterConformanceError::InvalidConfig(
                "adapter subject is empty".to_string(),
            ));
        }
        if self.runtime_manifest.as_os_str().is_empty() {
            return Err(SimulatorAdapterConformanceError::InvalidConfig(
                "runtime manifest is empty".to_string(),
            ));
        }
        if !(1..=MAX_RESPONSE_TIMEOUT_MS).contains(&self.response_timeout_ms) {
            return Err(SimulatorAdapterConformanceError::InvalidConfig(format!(
                "response_timeout_ms must be within 1..={MAX_RESPONSE_TIMEOUT_MS}"
            )));
        }
        Ok(())
    }
}

/// Content-addressed adapter, launch arguments, TaskSpec, and simulator runtime.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorAdapterConformanceSubject {
    /// Adapter implementation file name.
    pub adapter_file: String,
    /// SHA-256 of the exact adapter implementation bytes.
    pub adapter_sha256: String,
    /// Exact adapter implementation size.
    pub adapter_size_bytes: u64,
    /// Launcher file name, such as a binary or interpreter.
    pub launcher_file: String,
    /// SHA-256 of normalized, length-delimited adapter arguments.
    pub arguments_sha256: String,
    /// Number of normalized adapter arguments.
    pub argument_count: usize,
    /// TaskSpec file name.
    pub task_file: String,
    /// SHA-256 of the exact TaskSpec bytes.
    pub task_sha256: String,
    /// Runtime manifest file name.
    pub runtime_manifest_file: String,
    /// SHA-256 of the exact runtime manifest bytes.
    pub runtime_manifest_sha256: String,
    /// Exact runtime manifest size.
    pub runtime_manifest_size_bytes: u64,
    /// Rehashed canonical world, robot model, and adapter configuration files.
    pub runtime_artifacts: Vec<SimulatorRuntimeArtifact>,
}

/// Runtime identity negotiated from a successful simulator handshake.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorAdapterConformanceIdentity {
    /// Stable simulator family.
    pub simulator_id: String,
    /// Exact simulator runtime version.
    pub simulator_version: String,
    /// Stable adapter identity.
    pub adapter_id: String,
    /// Bound portable TaskSpec identity.
    pub task_id: String,
    /// Simulator process protocol schema exercised by the runner.
    pub wire_schema_version: u32,
    /// Flattened TaskSpec observation width.
    pub observation_width: usize,
    /// Flattened TaskSpec action width.
    pub action_width: usize,
    /// Exact simulation-time ticks advanced per action.
    pub fixed_delta_ticks: u64,
}

/// One canonical external simulator-adapter verdict.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorAdapterConformanceCheck {
    /// Stable check identifier.
    pub id: String,
    /// `passed`, `failed`, or `not_run`.
    pub status: String,
    /// Bounded diagnostic associated with the verdict.
    pub detail: String,
}

/// Portable deterministic external simulator-adapter conformance report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorAdapterConformanceReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Stable report discriminator.
    pub kind: String,
    /// Aggregate `passed` or `failed` verdict.
    pub status: String,
    /// Content-addressed inputs and launch contract tested by this run.
    pub subject: SimulatorAdapterConformanceSubject,
    /// Negotiated simulator identity, absent when canonical open failed.
    pub adapter: Option<SimulatorAdapterConformanceIdentity>,
    /// Checks in canonical conformance order.
    pub checks: Vec<SimulatorAdapterConformanceCheck>,
}

impl SimulatorAdapterConformanceReport {
    /// Returns true only when identity exists and every canonical check passed.
    pub fn passed(&self) -> bool {
        self.status == "passed"
            && self.adapter.is_some()
            && self.checks.iter().all(|check| check.status == "passed")
    }

    /// Validates schema, canonical checks, hashes, identity, and aggregate status.
    pub fn validate(&self) -> Result<(), SimulatorAdapterConformanceError> {
        if self.schema_version != SIMULATOR_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "unsupported report schema".to_string(),
            ));
        }
        if self.kind != SIMULATOR_ADAPTER_CONFORMANCE_REPORT_KIND {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "report kind drifted".to_string(),
            ));
        }
        if self
            .checks
            .iter()
            .map(|check| check.id.as_str())
            .ne(CHECK_IDS)
        {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "check registry is not canonical".to_string(),
            ));
        }
        if self.checks.iter().any(|check| {
            !matches!(check.status.as_str(), "passed" | "failed" | "not_run")
                || check.detail.len() > 512
        }) {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "check status or diagnostic is invalid".to_string(),
            ));
        }
        for digest in [
            self.subject.adapter_sha256.as_str(),
            self.subject.arguments_sha256.as_str(),
            self.subject.task_sha256.as_str(),
            self.subject.runtime_manifest_sha256.as_str(),
        ] {
            if !super::is_sha256_hex(digest) {
                return Err(SimulatorAdapterConformanceError::InvalidReport(
                    "subject digest is not lowercase SHA-256 hex".to_string(),
                ));
            }
        }
        let expected_roles = [
            SimulatorArtifactRole::World,
            SimulatorArtifactRole::RobotModel,
            SimulatorArtifactRole::AdapterConfig,
        ];
        if self.subject.runtime_artifacts.len() != expected_roles.len()
            || self
                .subject
                .runtime_artifacts
                .iter()
                .map(|artifact| artifact.role)
                .ne(expected_roles)
            || self.subject.runtime_artifacts.iter().any(|artifact| {
                artifact.file.trim().is_empty()
                    || artifact.file.contains('/')
                    || artifact.file.contains('\\')
                    || matches!(artifact.file.as_str(), "." | "..")
                    || artifact.size_bytes == 0
                    || !super::is_sha256_hex(&artifact.sha256)
            })
        {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "runtime artifact binding is invalid".to_string(),
            ));
        }
        let expected =
            if self.adapter.is_some() && self.checks.iter().all(|check| check.status == "passed") {
                "passed"
            } else {
                "failed"
            };
        if self.status != expected {
            return Err(SimulatorAdapterConformanceError::InvalidReport(
                "aggregate status does not match checks".to_string(),
            ));
        }
        if let Some(adapter) = &self.adapter {
            if adapter.wire_schema_version != SIMULATOR_WIRE_SCHEMA_VERSION
                || adapter.observation_width == 0
                || adapter.action_width == 0
                || adapter.fixed_delta_ticks == 0
            {
                return Err(SimulatorAdapterConformanceError::InvalidReport(
                    "adapter identity is invalid".to_string(),
                ));
            }
        }
        Ok(())
    }

    /// Serializes a validated report as stable pretty JSON with trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, SimulatorAdapterConformanceError> {
        self.validate()?;
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }

    fn new(subject: SimulatorAdapterConformanceSubject) -> Self {
        Self {
            schema_version: SIMULATOR_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION,
            kind: SIMULATOR_ADAPTER_CONFORMANCE_REPORT_KIND.to_string(),
            status: "failed".to_string(),
            subject,
            adapter: None,
            checks: CHECK_IDS
                .iter()
                .map(|id| SimulatorAdapterConformanceCheck {
                    id: (*id).to_string(),
                    status: "not_run".to_string(),
                    detail: String::new(),
                })
                .collect(),
        }
    }

    fn verdict(&mut self, id: &str, result: Result<String, String>) {
        let check = self
            .checks
            .iter_mut()
            .find(|check| check.id == id)
            .expect("canonical simulator adapter conformance check");
        match result {
            Ok(detail) => {
                check.status = "passed".to_string();
                check.detail = detail.chars().take(512).collect();
            }
            Err(detail) => {
                check.status = "failed".to_string();
                check.detail = detail.chars().take(512).collect();
            }
        }
        self.status =
            if self.adapter.is_some() && self.checks.iter().all(|check| check.status == "passed") {
                "passed"
            } else {
                "failed"
            }
            .to_string();
    }
}

/// Failure reading conformance inputs, validating configuration, or report shape.
#[derive(Debug, thiserror::Error)]
pub enum SimulatorAdapterConformanceError {
    /// A content-addressed input could not be read.
    #[error("read simulator adapter conformance input {path}: {message}")]
    Read {
        /// Input path.
        path: String,
        /// Operating-system diagnostic.
        message: String,
    },
    /// Process launch or timeout configuration is invalid.
    #[error("invalid simulator adapter conformance config: {0}")]
    InvalidConfig(String),
    /// TaskSpec cannot bind the fixed-step simulator contract.
    #[error("invalid simulator adapter TaskSpec: {0}")]
    InvalidTask(String),
    /// Runtime manifest or one bound file is invalid.
    #[error("invalid simulator runtime manifest: {0}")]
    InvalidRuntime(String),
    /// Runner produced or received an invalid report.
    #[error("invalid simulator adapter conformance report: {0}")]
    InvalidReport(String),
    /// JSON parsing or serialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Runs the published process-level external simulator conformance catalog.
///
/// Every check launches a fresh adapter process. Protocol and semantic failures
/// become a valid failed report; malformed trusted inputs remain errors.
pub fn run_simulator_adapter_conformance(
    task_path: &Path,
    config: &SimulatorAdapterConformanceConfig,
) -> Result<SimulatorAdapterConformanceReport, SimulatorAdapterConformanceError> {
    config.validate()?;
    let task_bytes = read_input(task_path)?;
    let task_spec: TaskSpec = serde_json::from_slice(&task_bytes)
        .map_err(|error| SimulatorAdapterConformanceError::InvalidTask(error.to_string()))?;
    task_spec
        .validate()
        .map_err(|error| SimulatorAdapterConformanceError::InvalidTask(error.to_string()))?;
    let task = ConformanceTask::new(task_spec, sha256_hex(&task_bytes))?;
    let adapter_bytes = read_input(&config.subject)?;
    let runtime_bytes = read_input(&config.runtime_manifest)?;
    let runtime: SimulatorRuntimeManifest = serde_json::from_slice(&runtime_bytes)
        .map_err(|error| SimulatorAdapterConformanceError::InvalidRuntime(error.to_string()))?;
    runtime
        .validate()
        .map_err(|error| SimulatorAdapterConformanceError::InvalidRuntime(error.to_string()))?;
    verify_runtime_artifacts(config, &runtime)?;
    if runtime.fixed_delta_ticks != task.fixed_delta_ticks {
        return Err(SimulatorAdapterConformanceError::InvalidRuntime(format!(
            "manifest fixed_delta_ticks {} differs from TaskSpec {}",
            runtime.fixed_delta_ticks, task.fixed_delta_ticks
        )));
    }
    let arguments = normalized_arguments(config)?;
    let subject = SimulatorAdapterConformanceSubject {
        adapter_file: file_name(&config.subject),
        adapter_sha256: sha256_hex(&adapter_bytes),
        adapter_size_bytes: adapter_bytes.len() as u64,
        launcher_file: file_name(&config.program),
        arguments_sha256: sha256_hex(&serde_json::to_vec(&arguments)?),
        argument_count: arguments.len(),
        task_file: file_name(task_path),
        task_sha256: task.task_sha256.clone(),
        runtime_manifest_file: file_name(&config.runtime_manifest),
        runtime_manifest_sha256: sha256_hex(&runtime_bytes),
        runtime_manifest_size_bytes: runtime_bytes.len() as u64,
        runtime_artifacts: runtime.artifacts.clone(),
    };
    let mut report = SimulatorAdapterConformanceReport::new(subject);

    match open_identity_case(config, &task, &runtime) {
        Ok((identity, detail)) => {
            report.adapter = Some(identity);
            report.verdict("open_identity", Ok(detail));
        }
        Err(error) => report.verdict("open_identity", Err(error)),
    }
    let identity = report.adapter.clone();
    let identity = identity.as_ref();
    report.verdict("task_binding", task_binding_case(config, &task, identity));
    report.verdict(
        "fixed_delta_binding",
        fixed_delta_binding_case(config, &task, identity),
    );
    report.verdict("reset_origin", reset_origin_case(config, &task, identity));
    report.verdict("bounded_step", bounded_step_case(config, &task, identity));
    report.verdict(
        "fixed_step_progression",
        fixed_step_progression_case(config, &task, identity),
    );
    report.verdict(
        "deterministic_replay",
        deterministic_replay_case(config, &task, identity),
    );
    report.verdict(
        "action_sequence_rejection",
        action_sequence_rejection_case(config, &task, identity),
    );
    report.verdict(
        "session_isolation",
        session_isolation_case(config, &task, identity),
    );
    report.verdict(
        "width_rejection",
        width_rejection_case(config, &task, identity),
    );
    report.validate()?;
    Ok(report)
}

#[derive(Clone, Debug)]
struct ConformanceTask {
    task_id: String,
    task_sha256: String,
    observation_width: usize,
    action_width: usize,
    fixed_delta_ticks: u64,
    bounded_action: Vec<f64>,
}

impl ConformanceTask {
    fn new(spec: TaskSpec, task_sha256: String) -> Result<Self, SimulatorAdapterConformanceError> {
        let gateway = HardwareGateway::new(spec.clone(), GatewayConfig::default())
            .map_err(|error| SimulatorAdapterConformanceError::InvalidTask(error.to_string()))?;
        let fixed_delta = spec.control_step_s * NS_PER_SECOND;
        if !fixed_delta.is_finite() || fixed_delta < 1.0 || fixed_delta > u64::MAX as f64 {
            return Err(SimulatorAdapterConformanceError::InvalidTask(
                "control_step_s cannot be represented as positive nanosecond ticks".to_string(),
            ));
        }
        let fixed_delta_ticks = fixed_delta.round() as u64;
        let bounded_action = gateway
            .action_limits()
            .iter()
            .map(|limit| 0.25 * limit.lower + 0.75 * limit.upper)
            .collect();
        Ok(Self {
            task_id: spec.task_id,
            task_sha256,
            observation_width: gateway.observation_width(),
            action_width: gateway.action_width(),
            fixed_delta_ticks,
            bounded_action,
        })
    }
}

fn open_identity_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    runtime: &SimulatorRuntimeManifest,
) -> Result<(SimulatorAdapterConformanceIdentity, String), String> {
    let mut process = AdapterProcess::spawn(config)?;
    let identity = open(&mut process, task, "rne.simulator.open.v1", 1, None)?;
    if identity.simulator_id != runtime.simulator_id
        || identity.simulator_version != runtime.simulator_version
    {
        return Err("handshake simulator identity differs from runtime manifest".to_string());
    }
    close(process, "rne.simulator.open.v1", 2)?;
    Ok((
        identity.clone(),
        format!(
            "simulator_id={} simulator_version={} adapter_id={}",
            identity.simulator_id, identity.simulator_version, identity.adapter_id
        ),
    ))
}

fn task_binding_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.task-binding.v1";
    let mut process = AdapterProcess::spawn(config)?;
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        1,
        SimulatorHostPayload::Open {
            task_id: task.task_id.clone(),
            task_sha256: "f".repeat(64),
            observation_width: task.observation_width,
            action_width: task.action_width,
            fixed_delta_ticks: task.fixed_delta_ticks,
        },
    ))?;
    expect_rejection(response, SimulatorRejectionCode::TaskMismatch)?;
    open(&mut process, task, session, 2, expected)?;
    close(process, session, 3)?;
    Ok("wrong TaskSpec digest rejected before correct binding".to_string())
}

fn fixed_delta_binding_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.fixed-delta.v1";
    let mut process = AdapterProcess::spawn(config)?;
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        1,
        SimulatorHostPayload::Open {
            task_id: task.task_id.clone(),
            task_sha256: task.task_sha256.clone(),
            observation_width: task.observation_width,
            action_width: task.action_width,
            fixed_delta_ticks: task.fixed_delta_ticks + 1,
        },
    ))?;
    expect_rejection(response, SimulatorRejectionCode::FixedDeltaMismatch)?;
    open(&mut process, task, session, 2, expected)?;
    close(process, session, 3)?;
    Ok("one-tick fixed-delta drift rejected before correct binding".to_string())
}

fn reset_origin_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.reset.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    let reset = reset(&mut process, session, 2, 7)?;
    if reset.values.len() != task.observation_width {
        return Err("reset observation width differs from TaskSpec".to_string());
    }
    close(process, session, 3)?;
    Ok("seeded reset returned the TaskSpec observation at step=0 time=0".to_string())
}

fn bounded_step_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.bounded-step.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    reset(&mut process, session, 2, 7)?;
    let step = step(&mut process, session, 3, 0, task.bounded_action.clone())?;
    validate_step(&step, task, 0, 1)?;
    close(process, session, 4)?;
    Ok("one TaskSpec-bounded action advanced exactly one simulator step".to_string())
}

fn fixed_step_progression_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.progression.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    reset(&mut process, session, 2, 11)?;
    for index in 0..3 {
        let response = step(
            &mut process,
            session,
            index + 3,
            index,
            task.bounded_action.clone(),
        )?;
        validate_step(&response, task, index, index + 1)?;
    }
    close(process, session, 6)?;
    Ok(format!(
        "three actions reached step=3 sim_time_ticks={}",
        task.fixed_delta_ticks * 3
    ))
}

fn deterministic_replay_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let first = deterministic_trace(config, task, expected, "rne.simulator.repeat-a.v1")?;
    let second = deterministic_trace(config, task, expected, "rne.simulator.repeat-b.v1")?;
    if first != second {
        return Err("fresh same-runtime seeded executions differed".to_string());
    }
    Ok("two fresh seeded executions produced identical observations and state digests".to_string())
}

fn deterministic_trace(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
    session: &str,
) -> Result<Vec<(Vec<u64>, u64)>, String> {
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    let reset = reset(&mut process, session, 2, 29)?;
    let mut trace = vec![(
        reset.values.iter().map(|value| value.to_bits()).collect(),
        reset.state_digest,
    )];
    for index in 0..2 {
        let response = step(
            &mut process,
            session,
            index + 3,
            index,
            task.bounded_action.clone(),
        )?;
        trace.push((
            response
                .values
                .iter()
                .map(|value| value.to_bits())
                .collect(),
            response.state_digest,
        ));
    }
    close(process, session, 5)?;
    Ok(trace)
}

fn action_sequence_rejection_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.action-sequence.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    reset(&mut process, session, 2, 31)?;
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        3,
        SimulatorHostPayload::Step {
            action_sequence: 1,
            values: task.bounded_action.clone(),
        },
    ))?;
    expect_rejection(response, SimulatorRejectionCode::ActionSequenceMismatch)?;
    let valid = step(&mut process, session, 4, 0, task.bounded_action.clone())?;
    validate_step(&valid, task, 0, 1)?;
    close(process, session, 5)?;
    Ok("skipped action sequence rejected without advancing simulator state".to_string())
}

fn session_isolation_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.session-a.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    let response = process.exchange(SimulatorHostFrame::new(
        "rne.simulator.session-b.v1",
        2,
        SimulatorHostPayload::Reset { seed: 1 },
    ))?;
    expect_rejection(response, SimulatorRejectionCode::SessionMismatch)?;
    reset(&mut process, session, 3, 1)?;
    close(process, session, 4)?;
    Ok("cross-session request rejected without corrupting bound session".to_string())
}

fn width_rejection_case(
    config: &SimulatorAdapterConformanceConfig,
    task: &ConformanceTask,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<String, String> {
    let session = "rne.simulator.width.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(&mut process, task, session, 1, expected)?;
    reset(&mut process, session, 2, 37)?;
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        3,
        SimulatorHostPayload::Step {
            action_sequence: 0,
            values: vec![0.0; task.action_width + 1],
        },
    ))?;
    expect_rejection(response, SimulatorRejectionCode::WidthMismatch)?;
    let valid = step(&mut process, session, 4, 0, task.bounded_action.clone())?;
    validate_step(&valid, task, 0, 1)?;
    close(process, session, 5)?;
    Ok("wrong-width action rejected before simulator advancement".to_string())
}

fn open(
    process: &mut AdapterProcess,
    task: &ConformanceTask,
    session: &str,
    sequence: u64,
    expected: Option<&SimulatorAdapterConformanceIdentity>,
) -> Result<SimulatorAdapterConformanceIdentity, String> {
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        sequence,
        SimulatorHostPayload::Open {
            task_id: task.task_id.clone(),
            task_sha256: task.task_sha256.clone(),
            observation_width: task.observation_width,
            action_width: task.action_width,
            fixed_delta_ticks: task.fixed_delta_ticks,
        },
    ))?;
    let SimulatorAdapterPayload::Ready {
        simulator_id,
        simulator_version,
        adapter_id,
        task_id,
        task_sha256,
        observation_width,
        action_width,
        fixed_delta_ticks,
    } = response.payload
    else {
        return Err(format!("open did not return ready: {:?}", response.payload));
    };
    if task_id != task.task_id
        || task_sha256 != task.task_sha256
        || observation_width != task.observation_width
        || action_width != task.action_width
        || fixed_delta_ticks != task.fixed_delta_ticks
    {
        return Err("ready response changed TaskSpec or timing binding".to_string());
    }
    let identity = SimulatorAdapterConformanceIdentity {
        simulator_id,
        simulator_version,
        adapter_id,
        task_id,
        wire_schema_version: SIMULATOR_WIRE_SCHEMA_VERSION,
        observation_width,
        action_width,
        fixed_delta_ticks,
    };
    if expected.is_some_and(|expected| expected != &identity) {
        return Err("simulator identity changed between fresh conformance cases".to_string());
    }
    Ok(identity)
}

#[derive(Clone, Debug, PartialEq)]
struct ResetEvidence {
    values: Vec<f64>,
    state_digest: u64,
}

fn reset(
    process: &mut AdapterProcess,
    session: &str,
    sequence: u64,
    seed: u64,
) -> Result<ResetEvidence, String> {
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        sequence,
        SimulatorHostPayload::Reset { seed },
    ))?;
    let SimulatorAdapterPayload::ResetComplete {
        seed: actual_seed,
        values,
        state_digest,
    } = response.payload
    else {
        return Err(format!("reset did not complete: {:?}", response.payload));
    };
    if actual_seed != seed {
        return Err("adapter changed the reset seed".to_string());
    }
    Ok(ResetEvidence {
        values,
        state_digest,
    })
}

#[derive(Clone, Debug, PartialEq)]
struct StepEvidence {
    action_sequence: u64,
    step: u64,
    sim_time_ticks: u64,
    values: Vec<f64>,
    terminated: bool,
    truncated: bool,
    state_digest: u64,
}

fn step(
    process: &mut AdapterProcess,
    session: &str,
    request_sequence: u64,
    action_sequence: u64,
    values: Vec<f64>,
) -> Result<StepEvidence, String> {
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        request_sequence,
        SimulatorHostPayload::Step {
            action_sequence,
            values,
        },
    ))?;
    let SimulatorAdapterPayload::Stepped {
        action_sequence,
        step,
        sim_time_ticks,
        values,
        terminated,
        truncated,
        state_digest,
    } = response.payload
    else {
        return Err(format!(
            "step did not return observation: {:?}",
            response.payload
        ));
    };
    Ok(StepEvidence {
        action_sequence,
        step,
        sim_time_ticks,
        values,
        terminated,
        truncated,
        state_digest,
    })
}

fn validate_step(
    response: &StepEvidence,
    task: &ConformanceTask,
    action_sequence: u64,
    step: u64,
) -> Result<(), String> {
    let expected_sim_time_ticks = task.fixed_delta_ticks.checked_mul(step);
    if response.action_sequence != action_sequence
        || response.step != step
        || Some(response.sim_time_ticks) != expected_sim_time_ticks
        || response.values.len() != task.observation_width
        || response.terminated
        || response.truncated
    {
        return Err(
            "step response violated sequence, time, width, or terminal contract".to_string(),
        );
    }
    Ok(())
}

fn expect_rejection(
    response: SimulatorAdapterFrame,
    expected: SimulatorRejectionCode,
) -> Result<(), String> {
    if response.payload == (SimulatorAdapterPayload::Rejected { code: expected }) {
        Ok(())
    } else {
        Err(format!(
            "expected rejection {expected:?}, got {:?}",
            response.payload
        ))
    }
}

fn close(mut process: AdapterProcess, session: &str, sequence: u64) -> Result<(), String> {
    let response = process.exchange(SimulatorHostFrame::new(
        session,
        sequence,
        SimulatorHostPayload::Close,
    ))?;
    if response.payload != SimulatorAdapterPayload::Closed {
        return Err(format!(
            "close was not acknowledged: {:?}",
            response.payload
        ));
    }
    process.finish()
}

struct AdapterProcess {
    child: Child,
    stdin: Option<ChildStdin>,
    lines: Receiver<Result<Option<Vec<u8>>, String>>,
    reader: Option<JoinHandle<()>>,
    codec: SimulatorWireCodec,
    timeout: Duration,
    finished: bool,
}

impl AdapterProcess {
    fn spawn(config: &SimulatorAdapterConformanceConfig) -> Result<Self, String> {
        let mut child = Command::new(&config.program)
            .args(&config.arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|error| format!("could not spawn simulator adapter: {error}"))?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| "adapter stdin was not piped".to_string())?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| "adapter stdout was not piped".to_string())?;
        let codec = SimulatorWireCodec::default();
        let (sender, lines) = mpsc::sync_channel(1);
        let reader = thread::spawn(move || {
            let mut stdout = std::io::BufReader::new(stdout);
            loop {
                let result = codec
                    .read_line(&mut stdout)
                    .map_err(|error| error.to_string());
                let terminal = !matches!(result, Ok(Some(_)));
                if sender.send(result).is_err() || terminal {
                    break;
                }
            }
        });
        Ok(Self {
            child,
            stdin: Some(stdin),
            lines,
            reader: Some(reader),
            codec,
            timeout: Duration::from_millis(config.response_timeout_ms),
            finished: false,
        })
    }

    fn exchange(&mut self, request: SimulatorHostFrame) -> Result<SimulatorAdapterFrame, String> {
        let encoded = self
            .codec
            .encode_host(&request)
            .map_err(|error| format!("could not encode host frame: {error}"))?;
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| "adapter stdin is closed".to_string())?;
        stdin
            .write_all(&encoded)
            .and_then(|()| stdin.flush())
            .map_err(|error| format!("could not write adapter request: {error}"))?;
        let line = match self.lines.recv_timeout(self.timeout) {
            Ok(Ok(Some(line))) => line,
            Ok(Ok(None)) => return Err("adapter exited before responding".to_string()),
            Ok(Err(error)) => return Err(format!("could not read adapter response: {error}")),
            Err(RecvTimeoutError::Timeout) => {
                return Err(format!(
                    "adapter response exceeded {} ms",
                    self.timeout.as_millis()
                ));
            }
            Err(RecvTimeoutError::Disconnected) => {
                return Err("adapter response reader stopped".to_string());
            }
        };
        let response = self
            .codec
            .decode_adapter(&line)
            .map_err(|error| format!("invalid adapter response: {error}"))?;
        if response.session_id != request.session_id
            || response.request_sequence != request.sequence
        {
            return Err("adapter response did not correlate to request".to_string());
        }
        Ok(response)
    }

    fn finish(mut self) -> Result<(), String> {
        self.stdin.take();
        let started = Instant::now();
        let status = loop {
            match self.child.try_wait() {
                Ok(Some(status)) => break status,
                Ok(None) if started.elapsed() < self.timeout => {
                    thread::sleep(Duration::from_millis(5))
                }
                Ok(None) => {
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    self.finished = true;
                    self.join_reader();
                    return Err("adapter did not exit after close".to_string());
                }
                Err(error) => return Err(format!("could not wait for adapter process: {error}")),
            }
        };
        self.finished = true;
        self.join_reader();
        if status.success() {
            Ok(())
        } else {
            Err(format!("adapter exited with status {status}"))
        }
    }

    fn join_reader(&mut self) {
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

impl Drop for AdapterProcess {
    fn drop(&mut self) {
        self.stdin.take();
        if !self.finished {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
        self.join_reader();
    }
}

fn verify_runtime_artifacts(
    config: &SimulatorAdapterConformanceConfig,
    runtime: &SimulatorRuntimeManifest,
) -> Result<(), SimulatorAdapterConformanceError> {
    let root = config
        .runtime_manifest
        .parent()
        .unwrap_or_else(|| Path::new("."));
    for artifact in &runtime.artifacts {
        let path = root.join(&artifact.file);
        let bytes = read_input(&path)?;
        if bytes.len() as u64 != artifact.size_bytes || sha256_hex(&bytes) != artifact.sha256 {
            return Err(SimulatorAdapterConformanceError::InvalidRuntime(format!(
                "runtime artifact {} size or digest mismatch",
                artifact.file
            )));
        }
    }
    Ok(())
}

fn normalized_arguments(
    config: &SimulatorAdapterConformanceConfig,
) -> Result<Vec<String>, SimulatorAdapterConformanceError> {
    let subject = config.subject.to_str().ok_or_else(|| {
        SimulatorAdapterConformanceError::InvalidConfig(
            "adapter subject path must be valid Unicode".to_string(),
        )
    })?;
    let runtime = config.runtime_manifest.to_str().ok_or_else(|| {
        SimulatorAdapterConformanceError::InvalidConfig(
            "runtime manifest path must be valid Unicode".to_string(),
        )
    })?;
    config
        .arguments
        .iter()
        .enumerate()
        .map(|(index, argument)| {
            let argument = argument.to_str().ok_or_else(|| {
                SimulatorAdapterConformanceError::InvalidConfig(format!(
                    "adapter argument {index} is not valid Unicode"
                ))
            })?;
            Ok(if argument == subject {
                "<adapter-subject>".to_string()
            } else if argument == runtime {
                "<runtime-manifest>".to_string()
            } else {
                argument.to_string()
            })
        })
        .collect()
}

fn read_input(path: &Path) -> Result<Vec<u8>, SimulatorAdapterConformanceError> {
    fs::read(path).map_err(|error| SimulatorAdapterConformanceError::Read {
        path: path.display().to_string(),
        message: error.to_string(),
    })
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("unknown")
        .to_string()
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_validation_rejects_noncanonical_checks() {
        let subject = SimulatorAdapterConformanceSubject {
            adapter_file: "adapter".to_string(),
            adapter_sha256: "0".repeat(64),
            adapter_size_bytes: 1,
            launcher_file: "adapter".to_string(),
            arguments_sha256: "1".repeat(64),
            argument_count: 0,
            task_file: "task.json".to_string(),
            task_sha256: "2".repeat(64),
            runtime_manifest_file: "runtime.json".to_string(),
            runtime_manifest_sha256: "3".repeat(64),
            runtime_manifest_size_bytes: 1,
            runtime_artifacts: [
                super::super::SimulatorArtifactRole::World,
                super::super::SimulatorArtifactRole::RobotModel,
                super::super::SimulatorArtifactRole::AdapterConfig,
            ]
            .into_iter()
            .map(|role| SimulatorRuntimeArtifact {
                role,
                file: format!("{role:?}.bin"),
                size_bytes: 1,
                sha256: "4".repeat(64),
            })
            .collect(),
        };
        let mut report = SimulatorAdapterConformanceReport::new(subject);
        report.validate().unwrap();
        report.checks.swap(0, 1);
        assert!(matches!(
            report.validate(),
            Err(SimulatorAdapterConformanceError::InvalidReport(_))
        ));
        report.checks.swap(0, 1);
        report.subject.runtime_artifacts.swap(0, 1);
        assert!(matches!(
            report.validate(),
            Err(SimulatorAdapterConformanceError::InvalidReport(_))
        ));
    }
}
