//! Standalone conformance runner for external hardware adapter processes.
//!
//! The runner speaks only the versioned JSON Lines protocol from [`crate::wire`].
//! It launches a fresh process for each case, bounds every response wait, and
//! kills only the child it launched when a case times out or violates the
//! protocol. The tested adapter must be configured for a sandbox or mock device:
//! full conformance deliberately exercises HIL authority after the caller opts in.

use crate::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareWireCodec, HostWireFrame, HostWirePayload,
    WireRejectionCode, HARDWARE_WIRE_SCHEMA_VERSION,
};
use crate::{ActuationFrame, GatewayConfig, HardwareGateway, HardwareMode, SafetyReason};
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

/// Stable kind identifier for external hardware-adapter conformance reports.
pub const HARDWARE_ADAPTER_CONFORMANCE_REPORT_KIND: &str =
    "rne_hardware_adapter_conformance_report";
/// Current external hardware-adapter conformance report schema.
pub const HARDWARE_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION: u32 = 1;

const MAX_RESPONSE_TIMEOUT_MS: u64 = 60_000;
const CHECK_IDS: [&str; 9] = [
    "open_identity",
    "task_binding",
    "observation_stream",
    "bounded_actuation",
    "safe_stop",
    "shadow_authority",
    "sequence_rejection",
    "session_isolation",
    "width_rejection",
];

/// Process launch and safety configuration for one adapter conformance run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HardwareAdapterConformanceConfig {
    /// Executable used to launch each fresh adapter process.
    pub program: PathBuf,
    /// Exact arguments supplied to every adapter process.
    pub arguments: Vec<OsString>,
    /// Adapter implementation file whose bytes identify the conformance subject.
    ///
    /// This may differ from `program` for interpreted adapters. For example,
    /// `program` can be Python while this path points at the adapter script.
    pub subject: PathBuf,
    /// Maximum wait for one response and for clean process termination.
    pub response_timeout_ms: u64,
    /// Explicit authorization to send one TaskSpec-bounded HIL action.
    ///
    /// Set this only when `program` is connected to a sandbox or mock device.
    pub allow_hil: bool,
}

impl HardwareAdapterConformanceConfig {
    /// Creates a configuration that treats the executable itself as the subject.
    ///
    /// HIL remains disabled until the caller explicitly sets [`Self::allow_hil`].
    pub fn new(program: impl Into<PathBuf>) -> Self {
        let program = program.into();
        Self {
            subject: program.clone(),
            program,
            arguments: Vec::new(),
            response_timeout_ms: 5_000,
            allow_hil: false,
        }
    }

    fn validate(&self) -> Result<(), HardwareAdapterConformanceError> {
        if self.program.as_os_str().is_empty() {
            return Err(HardwareAdapterConformanceError::InvalidConfig(
                "adapter program is empty".to_string(),
            ));
        }
        if self.subject.as_os_str().is_empty() {
            return Err(HardwareAdapterConformanceError::InvalidConfig(
                "adapter subject is empty".to_string(),
            ));
        }
        if !(1..=MAX_RESPONSE_TIMEOUT_MS).contains(&self.response_timeout_ms) {
            return Err(HardwareAdapterConformanceError::InvalidConfig(format!(
                "response_timeout_ms must be within 1..={MAX_RESPONSE_TIMEOUT_MS}"
            )));
        }
        if !self.allow_hil {
            return Err(HardwareAdapterConformanceError::InvalidConfig(
                "full conformance requires explicit HIL authorization for a sandbox or mock device"
                    .to_string(),
            ));
        }
        Ok(())
    }
}

/// Content-addressed adapter implementation, launch contract, and TaskSpec.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareAdapterConformanceSubject {
    /// Adapter implementation file name without a machine-specific parent path.
    pub adapter_file: String,
    /// SHA-256 digest of the adapter implementation bytes.
    pub adapter_sha256: String,
    /// Adapter implementation size in bytes.
    pub adapter_size_bytes: u64,
    /// Launcher file name, such as a native executable or Python interpreter.
    pub launcher_file: String,
    /// SHA-256 of normalized, length-delimited process arguments.
    pub arguments_sha256: String,
    /// Number of process arguments included in `arguments_sha256`.
    pub argument_count: usize,
    /// TaskSpec file name without a machine-specific parent path.
    pub task_file: String,
    /// SHA-256 digest of the exact TaskSpec bytes.
    pub task_sha256: String,
}

/// Adapter identity negotiated from a successful protocol-v1 handshake.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareAdapterConformanceIdentity {
    /// Stable device or sandbox identity returned by the adapter.
    pub device_id: String,
    /// Portable task identity accepted by the adapter.
    pub task_id: String,
    /// Hardware process protocol schema exercised by the runner.
    pub wire_schema_version: u32,
    /// Flattened TaskSpec observation width accepted by the adapter.
    pub observation_width: usize,
    /// Flattened TaskSpec action width accepted by the adapter.
    pub action_width: usize,
}

/// One canonical external hardware-adapter verdict.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareAdapterConformanceCheck {
    /// Stable check identifier.
    pub id: String,
    /// `passed`, `failed`, or `not_run`.
    pub status: String,
    /// Bounded diagnostic associated with the verdict.
    pub detail: String,
}

/// Portable, deterministic external hardware-adapter conformance report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareAdapterConformanceReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Stable report discriminator.
    pub kind: String,
    /// Aggregate `passed` or `failed` verdict.
    pub status: String,
    /// Content-addressed inputs and launch contract tested by this run.
    pub subject: HardwareAdapterConformanceSubject,
    /// Negotiated adapter identity, absent when the canonical open failed.
    pub adapter: Option<HardwareAdapterConformanceIdentity>,
    /// Checks in canonical conformance order.
    pub checks: Vec<HardwareAdapterConformanceCheck>,
}

impl HardwareAdapterConformanceReport {
    /// Returns true only when the adapter identity exists and every check passed.
    pub fn passed(&self) -> bool {
        self.status == "passed"
            && self.adapter.is_some()
            && self.checks.iter().all(|check| check.status == "passed")
    }

    /// Validates schema, check ordering, digests, diagnostics, and aggregate status.
    pub fn validate(&self) -> Result<(), HardwareAdapterConformanceError> {
        if self.schema_version != HARDWARE_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION {
            return Err(HardwareAdapterConformanceError::InvalidReport(format!(
                "expected schema {}, got {}",
                HARDWARE_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION, self.schema_version
            )));
        }
        if self.kind != HARDWARE_ADAPTER_CONFORMANCE_REPORT_KIND {
            return Err(HardwareAdapterConformanceError::InvalidReport(
                "report kind drifted".to_string(),
            ));
        }
        if self
            .checks
            .iter()
            .map(|check| check.id.as_str())
            .ne(CHECK_IDS)
        {
            return Err(HardwareAdapterConformanceError::InvalidReport(
                "check registry is not canonical".to_string(),
            ));
        }
        if self.checks.iter().any(|check| {
            !matches!(check.status.as_str(), "passed" | "failed" | "not_run")
                || check.detail.len() > 512
        }) {
            return Err(HardwareAdapterConformanceError::InvalidReport(
                "check status or diagnostic is invalid".to_string(),
            ));
        }
        for digest in [
            self.subject.adapter_sha256.as_str(),
            self.subject.arguments_sha256.as_str(),
            self.subject.task_sha256.as_str(),
        ] {
            if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
                return Err(HardwareAdapterConformanceError::InvalidReport(
                    "subject digest is not SHA-256 hex".to_string(),
                ));
            }
        }
        let expected =
            if self.adapter.is_some() && self.checks.iter().all(|check| check.status == "passed") {
                "passed"
            } else {
                "failed"
            };
        if self.status != expected {
            return Err(HardwareAdapterConformanceError::InvalidReport(
                "aggregate status does not match checks".to_string(),
            ));
        }
        if let Some(adapter) = &self.adapter {
            if adapter.wire_schema_version != HARDWARE_WIRE_SCHEMA_VERSION
                || adapter.device_id.trim().is_empty()
                || adapter.task_id.trim().is_empty()
                || adapter.observation_width == 0
                || adapter.action_width == 0
            {
                return Err(HardwareAdapterConformanceError::InvalidReport(
                    "adapter identity is invalid".to_string(),
                ));
            }
        }
        Ok(())
    }

    /// Serializes a validated report as stable pretty JSON with a trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, HardwareAdapterConformanceError> {
        self.validate()?;
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }

    fn new(subject: HardwareAdapterConformanceSubject) -> Self {
        Self {
            schema_version: HARDWARE_ADAPTER_CONFORMANCE_REPORT_SCHEMA_VERSION,
            kind: HARDWARE_ADAPTER_CONFORMANCE_REPORT_KIND.to_string(),
            status: "failed".to_string(),
            subject,
            adapter: None,
            checks: CHECK_IDS
                .iter()
                .map(|id| HardwareAdapterConformanceCheck {
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
            .expect("canonical hardware adapter conformance check");
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

/// Failure reading conformance inputs, validating configuration, or serializing a report.
#[derive(Debug, thiserror::Error)]
pub enum HardwareAdapterConformanceError {
    /// A content-addressed input could not be read.
    #[error("read hardware adapter conformance input {path}: {message}")]
    Read {
        /// Input path.
        path: String,
        /// Operating-system diagnostic.
        message: String,
    },
    /// The launch or HIL authorization configuration is invalid.
    #[error("invalid hardware adapter conformance config: {0}")]
    InvalidConfig(String),
    /// The supplied TaskSpec cannot bind the hardware gateway contract.
    #[error("invalid hardware adapter TaskSpec: {0}")]
    InvalidTask(String),
    /// The runner produced or received an invalid report shape.
    #[error("invalid hardware adapter conformance report: {0}")]
    InvalidReport(String),
    /// JSON parsing or serialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Runs the published process-level adapter conformance catalog.
///
/// Each case starts a new child process. Process, protocol, timeout, and semantic
/// failures become a valid failed report. Input and report-shape failures are
/// returned as errors. `config.allow_hil` must be explicitly true because the
/// bounded-actuation case sends one normal action derived from TaskSpec limits.
pub fn run_hardware_adapter_conformance(
    task_path: &Path,
    config: &HardwareAdapterConformanceConfig,
) -> Result<HardwareAdapterConformanceReport, HardwareAdapterConformanceError> {
    config.validate()?;
    let task_bytes = read_input(task_path)?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes)
        .map_err(|error| HardwareAdapterConformanceError::InvalidTask(error.to_string()))?;
    let task = ConformanceTask::new(task)?;
    let adapter_bytes = read_input(&config.subject)?;
    let arguments = normalized_arguments(config)?;
    let arguments_bytes = serde_json::to_vec(&arguments)?;
    let subject = HardwareAdapterConformanceSubject {
        adapter_file: file_name(&config.subject),
        adapter_sha256: sha256_hex(&adapter_bytes),
        adapter_size_bytes: u64::try_from(adapter_bytes.len()).unwrap_or(u64::MAX),
        launcher_file: file_name(&config.program),
        arguments_sha256: sha256_hex(&arguments_bytes),
        argument_count: arguments.len(),
        task_file: file_name(task_path),
        task_sha256: sha256_hex(&task_bytes),
    };
    let mut report = HardwareAdapterConformanceReport::new(subject);

    match open_identity_case(config, &task) {
        Ok((identity, detail)) => {
            report.adapter = Some(identity);
            report.verdict("open_identity", Ok(detail));
        }
        Err(error) => report.verdict("open_identity", Err(error)),
    }
    let expected_device = report
        .adapter
        .as_ref()
        .map(|adapter| adapter.device_id.clone());
    report.verdict(
        "task_binding",
        task_binding_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "observation_stream",
        observation_stream_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "bounded_actuation",
        bounded_actuation_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "safe_stop",
        safe_stop_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "shadow_authority",
        shadow_authority_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "sequence_rejection",
        sequence_rejection_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "session_isolation",
        session_isolation_case(config, &task, expected_device.as_deref()),
    );
    report.verdict(
        "width_rejection",
        width_rejection_case(config, &task, expected_device.as_deref()),
    );
    report.validate()?;
    Ok(report)
}

#[derive(Clone, Debug)]
struct ConformanceTask {
    spec: TaskSpec,
    task_id: String,
    observation_width: usize,
    action_width: usize,
    bounded_action: Vec<f64>,
}

impl ConformanceTask {
    fn new(spec: TaskSpec) -> Result<Self, HardwareAdapterConformanceError> {
        let gateway = HardwareGateway::new(spec.clone(), GatewayConfig::default())
            .map_err(|error| HardwareAdapterConformanceError::InvalidTask(error.to_string()))?;
        let bounded_action = gateway
            .action_limits()
            .iter()
            .map(|limit| 0.5 * limit.lower + 0.5 * limit.upper)
            .collect();
        Ok(Self {
            task_id: spec.task_id.clone(),
            observation_width: gateway.observation_width(),
            action_width: gateway.action_width(),
            spec,
            bounded_action,
        })
    }

    fn gateway(&self, mode: HardwareMode) -> Result<HardwareGateway, String> {
        HardwareGateway::new(
            self.spec.clone(),
            GatewayConfig {
                mode,
                ..GatewayConfig::default()
            },
        )
        .map_err(|error| error.to_string())
    }
}

fn open_identity_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
) -> Result<(HardwareAdapterConformanceIdentity, String), String> {
    let mut process = AdapterProcess::spawn(config)?;
    let identity = open(
        &mut process,
        task,
        HardwareMode::Hil,
        "rne.conformance.open.v1",
        1,
        None,
    )?;
    send_safe_stop(&mut process, task, "rne.conformance.open.v1", 2)?;
    close(process, "rne.conformance.open.v1", 3)?;
    let detail = format!(
        "device_id={} wire_schema={} task binding accepted",
        identity.device_id, identity.wire_schema_version
    );
    Ok((identity, detail))
}

fn task_binding_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.task-binding.v1";
    let mut process = AdapterProcess::spawn(config)?;
    let wrong_width = task
        .action_width
        .checked_add(1)
        .ok_or_else(|| "action width cannot be perturbed".to_string())?;
    let response = process.exchange(HostWireFrame::new(
        session,
        1,
        HostWirePayload::Open {
            task_id: task.task_id.clone(),
            mode: HardwareMode::Hil,
            observation_width: task.observation_width,
            action_width: wrong_width,
        },
    ))?;
    expect_rejection(response, WireRejectionCode::WidthMismatch)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        2,
        expected_device,
    )?;
    send_safe_stop(&mut process, task, session, 3)?;
    close(process, session, 4)?;
    Ok("wrong TaskSpec width rejected before authority; correct binding recovered".to_string())
}

fn observation_stream_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.observation.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    let mut gateway = task.gateway(HardwareMode::Hil)?;
    gateway.connect(0).map_err(|error| error.to_string())?;
    let first = poll_observation(&mut process, session, 2)?;
    gateway
        .ingest_observation(1, first.0, first.1)
        .map_err(|error| error.to_string())?;
    let second = poll_observation(&mut process, session, 3)?;
    gateway
        .ingest_observation(2, second.0, second.1)
        .map_err(|error| error.to_string())?;
    if second.0 <= first.0 {
        return Err("observation sequence did not increase".to_string());
    }
    send_safe_stop(&mut process, task, session, 4)?;
    close(process, session, 5)?;
    Ok("two TaskSpec-shaped observations passed dtype and sequence validation".to_string())
}

fn bounded_actuation_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.actuation.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    let mut gateway = task.gateway(HardwareMode::Hil)?;
    gateway.connect(0).map_err(|error| error.to_string())?;
    let observation = poll_observation(&mut process, session, 2)?;
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .map_err(|error| error.to_string())?;
    gateway.arm(1).map_err(|error| error.to_string())?;
    gateway
        .submit_action(2, 0, observation.0, task.bounded_action.clone())
        .map_err(|error| error.to_string())?;
    let frame = gateway
        .poll_actuation(2)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "gateway did not queue the bounded HIL action".to_string())?;
    let response = process.exchange(HostWireFrame::new(
        session,
        3,
        HostWirePayload::Actuate { frame },
    ))?;
    if response.payload
        != (DeviceWirePayload::ActuationAccepted {
            action_sequence: Some(0),
            safety_stop: false,
        })
    {
        return Err(format!(
            "bounded action was not accepted: {:?}",
            response.payload
        ));
    }
    gateway.disarm(3).map_err(|error| error.to_string())?;
    let stop = gateway
        .poll_actuation(3)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "gateway did not queue the disarm stop".to_string())?;
    send_stop_frame(&mut process, session, 4, stop)?;
    close(process, session, 5)?;
    Ok("TaskSpec-bounded HIL action accepted and followed by a confirmed stop".to_string())
}

fn safe_stop_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.safe-stop.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    send_safe_stop(&mut process, task, session, 2)?;
    close(process, session, 3)?;
    Ok("adapter confirmed an explicit zero-output safety frame".to_string())
}

fn shadow_authority_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.shadow.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Shadow,
        session,
        1,
        expected_device,
    )?;
    let _ = poll_observation(&mut process, session, 2)?;
    let response = process.exchange(HostWireFrame::new(
        session,
        3,
        HostWirePayload::Actuate {
            frame: ActuationFrame {
                action_sequence: Some(0),
                queued_at_ms: 0,
                values: task.bounded_action.clone(),
                safety_stop: false,
                reason: None,
            },
        },
    ))?;
    expect_rejection(response, WireRejectionCode::AuthorityDenied)?;
    send_safe_stop(&mut process, task, session, 4)?;
    close(process, session, 5)?;
    Ok("shadow mode rejected authority while still accepting a safety stop".to_string())
}

fn sequence_rejection_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.sequence.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    let response = process.exchange(HostWireFrame::new(
        session,
        1,
        HostWirePayload::PollObservation,
    ))?;
    expect_rejection(response, WireRejectionCode::NonMonotonicSequence)?;
    send_safe_stop(&mut process, task, session, 2)?;
    close(process, session, 3)?;
    Ok("duplicate request sequence rejected without losing safe-stop control".to_string())
}

fn session_isolation_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.session-a.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    let response = process.exchange(HostWireFrame::new(
        "rne.conformance.session-b.v1",
        2,
        HostWirePayload::PollObservation,
    ))?;
    expect_rejection(response, WireRejectionCode::SessionMismatch)?;
    send_safe_stop(&mut process, task, session, 3)?;
    close(process, session, 4)?;
    Ok("cross-session request rejected without losing safe-stop control".to_string())
}

fn width_rejection_case(
    config: &HardwareAdapterConformanceConfig,
    task: &ConformanceTask,
    expected_device: Option<&str>,
) -> Result<String, String> {
    let session = "rne.conformance.width.v1";
    let mut process = AdapterProcess::spawn(config)?;
    open(
        &mut process,
        task,
        HardwareMode::Hil,
        session,
        1,
        expected_device,
    )?;
    let _ = poll_observation(&mut process, session, 2)?;
    let response = process.exchange(HostWireFrame::new(
        session,
        3,
        HostWirePayload::Actuate {
            frame: ActuationFrame {
                action_sequence: Some(0),
                queued_at_ms: 0,
                values: vec![0.0; task.action_width.saturating_add(1)],
                safety_stop: false,
                reason: None,
            },
        },
    ))?;
    expect_rejection(response, WireRejectionCode::WidthMismatch)?;
    send_safe_stop(&mut process, task, session, 4)?;
    close(process, session, 5)?;
    Ok("wrong-width action rejected before device actuation".to_string())
}

fn open(
    process: &mut AdapterProcess,
    task: &ConformanceTask,
    mode: HardwareMode,
    session: &str,
    sequence: u64,
    expected_device: Option<&str>,
) -> Result<HardwareAdapterConformanceIdentity, String> {
    let response = process.exchange(HostWireFrame::new(
        session,
        sequence,
        HostWirePayload::Open {
            task_id: task.task_id.clone(),
            mode,
            observation_width: task.observation_width,
            action_width: task.action_width,
        },
    ))?;
    let DeviceWirePayload::Ready {
        device_id,
        task_id,
        observation_width,
        action_width,
    } = response.payload
    else {
        return Err(format!("open did not return ready: {:?}", response.payload));
    };
    if task_id != task.task_id
        || observation_width != task.observation_width
        || action_width != task.action_width
    {
        return Err("ready response changed the TaskSpec binding".to_string());
    }
    if expected_device.is_some_and(|expected| expected != device_id) {
        return Err(format!(
            "device identity changed between cases: expected {expected_device:?}, got {device_id:?}"
        ));
    }
    Ok(HardwareAdapterConformanceIdentity {
        device_id,
        task_id,
        wire_schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
        observation_width,
        action_width,
    })
}

fn poll_observation(
    process: &mut AdapterProcess,
    session: &str,
    sequence: u64,
) -> Result<(u64, Vec<f64>), String> {
    let response = process.exchange(HostWireFrame::new(
        session,
        sequence,
        HostWirePayload::PollObservation,
    ))?;
    let DeviceWirePayload::Observation { sequence, values } = response.payload else {
        return Err(format!(
            "poll did not return observation: {:?}",
            response.payload
        ));
    };
    Ok((sequence, values))
}

fn send_safe_stop(
    process: &mut AdapterProcess,
    task: &ConformanceTask,
    session: &str,
    sequence: u64,
) -> Result<(), String> {
    send_stop_frame(
        process,
        session,
        sequence,
        ActuationFrame {
            action_sequence: None,
            queued_at_ms: 0,
            values: vec![0.0; task.action_width],
            safety_stop: true,
            reason: Some(SafetyReason::ManualDisarm),
        },
    )
}

fn send_stop_frame(
    process: &mut AdapterProcess,
    session: &str,
    sequence: u64,
    frame: ActuationFrame,
) -> Result<(), String> {
    let response = process.exchange(HostWireFrame::new(
        session,
        sequence,
        HostWirePayload::Actuate { frame },
    ))?;
    if response.payload
        != (DeviceWirePayload::ActuationAccepted {
            action_sequence: None,
            safety_stop: true,
        })
    {
        return Err(format!(
            "adapter did not confirm safe stop: {:?}",
            response.payload
        ));
    }
    Ok(())
}

fn expect_rejection(response: DeviceWireFrame, expected: WireRejectionCode) -> Result<(), String> {
    if response.payload == (DeviceWirePayload::Rejected { code: expected }) {
        Ok(())
    } else {
        Err(format!(
            "expected rejection {expected:?}, got {:?}",
            response.payload
        ))
    }
}

fn close(mut process: AdapterProcess, session: &str, sequence: u64) -> Result<(), String> {
    let response = process.exchange(HostWireFrame::new(
        session,
        sequence,
        HostWirePayload::Close,
    ))?;
    if response.payload != DeviceWirePayload::Closed {
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
    codec: HardwareWireCodec,
    timeout: Duration,
    finished: bool,
}

impl AdapterProcess {
    fn spawn(config: &HardwareAdapterConformanceConfig) -> Result<Self, String> {
        let mut child = Command::new(&config.program)
            .args(&config.arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|error| format!("could not spawn adapter process: {error}"))?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| "adapter stdin was not piped".to_string())?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| "adapter stdout was not piped".to_string())?;
        let codec = HardwareWireCodec::default();
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

    fn exchange(&mut self, request: HostWireFrame) -> Result<DeviceWireFrame, String> {
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
            .decode_device(&line)
            .map_err(|error| format!("invalid adapter response: {error}"))?;
        if response.session_id != request.session_id
            || response.request_sequence != request.sequence
        {
            return Err("adapter response did not correlate to the request".to_string());
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
                    thread::sleep(Duration::from_millis(5));
                }
                Ok(None) => {
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    self.finished = true;
                    self.join_reader();
                    return Err(format!(
                        "adapter did not exit within {} ms after close",
                        self.timeout.as_millis()
                    ));
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
            let started = Instant::now();
            loop {
                match self.child.try_wait() {
                    Ok(Some(_)) => break,
                    Err(_) => {
                        let _ = self.child.kill();
                        let _ = self.child.wait();
                        break;
                    }
                    Ok(None) if started.elapsed() < self.timeout => {
                        thread::sleep(Duration::from_millis(5));
                    }
                    Ok(None) => {
                        let _ = self.child.kill();
                        let _ = self.child.wait();
                        break;
                    }
                }
            }
        }
        self.join_reader();
    }
}

fn normalized_arguments(
    config: &HardwareAdapterConformanceConfig,
) -> Result<Vec<String>, HardwareAdapterConformanceError> {
    let subject = config.subject.to_str().ok_or_else(|| {
        HardwareAdapterConformanceError::InvalidConfig(
            "adapter subject path must be valid Unicode".to_string(),
        )
    })?;
    config
        .arguments
        .iter()
        .enumerate()
        .map(|(index, argument)| {
            let argument = argument.to_str().ok_or_else(|| {
                HardwareAdapterConformanceError::InvalidConfig(format!(
                    "adapter argument {index} is not valid Unicode"
                ))
            })?;
            Ok(if argument == subject {
                "<adapter-subject>".to_string()
            } else {
                argument.to_string()
            })
        })
        .collect()
}

fn read_input(path: &Path) -> Result<Vec<u8>, HardwareAdapterConformanceError> {
    fs::read(path).map_err(|error| HardwareAdapterConformanceError::Read {
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
    fn report_validation_rejects_noncanonical_checks_and_digest() {
        let subject = HardwareAdapterConformanceSubject {
            adapter_file: "adapter".to_string(),
            adapter_sha256: "0".repeat(64),
            adapter_size_bytes: 1,
            launcher_file: "adapter".to_string(),
            arguments_sha256: "1".repeat(64),
            argument_count: 0,
            task_file: "task.json".to_string(),
            task_sha256: "2".repeat(64),
        };
        let mut report = HardwareAdapterConformanceReport::new(subject);
        assert!(report.validate().is_ok());
        report.checks.swap(0, 1);
        assert!(matches!(
            report.validate(),
            Err(HardwareAdapterConformanceError::InvalidReport(_))
        ));
        report.checks.swap(0, 1);
        report.subject.task_sha256 = "bad".to_string();
        assert!(matches!(
            report.validate(),
            Err(HardwareAdapterConformanceError::InvalidReport(_))
        ));
    }

    #[test]
    fn hil_authorization_is_explicit_and_timeout_is_bounded() {
        let mut config = HardwareAdapterConformanceConfig::new("adapter");
        assert!(matches!(
            config.validate(),
            Err(HardwareAdapterConformanceError::InvalidConfig(_))
        ));
        config.allow_hil = true;
        config.response_timeout_ms = MAX_RESPONSE_TIMEOUT_MS + 1;
        assert!(matches!(
            config.validate(),
            Err(HardwareAdapterConformanceError::InvalidConfig(_))
        ));
    }
}
