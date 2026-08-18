//! Standalone conformance runner for external accelerator JSONL processes.

use super::conformance::task_spec_sha256;
use super::protocol::{validate_request_envelope, validate_response_envelope};
use super::{
    AcceleratorCapabilityReport, AcceleratorManifest, AcceleratorProtocolFrame,
    AcceleratorProtocolTranscript, AcceleratorRuntimeContract, ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
};
use rne_ai::{TaskSpec, TASK_SPEC_SCHEMA_VERSION};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::ffi::OsString;
use std::fs;
use std::io::{BufRead, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Stable kind identifier for process-level accelerator conformance reports.
pub const ACCELERATOR_PROCESS_CONFORMANCE_REPORT_KIND: &str =
    "rne_accelerator_process_conformance_report";
/// Current process-level accelerator conformance report schema.
pub const ACCELERATOR_PROCESS_CONFORMANCE_REPORT_SCHEMA_VERSION: u32 = 1;

const ROOT_SEED: u64 = 42;
const BATCH_WIDTH: usize = 1;
const SESSION_ID: &str = "contract";
const MAX_RESPONSE_TIMEOUT_MS: u64 = 60_000;
const MAX_CONTRACT_BYTES: u64 = 1024 * 1024;
const MAX_SUBJECT_BYTES: u64 = 64 * 1024 * 1024;
const MAX_PROTOCOL_LINE_BYTES: usize = 16 * 1024 * 1024;
const CHECK_IDS: [&str; 11] = [
    "spawn",
    "probe",
    "create",
    "reset_lanes",
    "step",
    "checkpoint",
    "restore",
    "close",
    "unsupported_operation",
    "shutdown",
    "transcript_binding",
];
const OPERATIONS: [&str; 9] = [
    "probe",
    "create",
    "reset_lanes",
    "step",
    "checkpoint",
    "restore",
    "close",
    "unsupported_v1_fixture",
    "shutdown",
];

/// Process launch configuration for one accelerator conformance run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AcceleratorProcessConformanceConfig {
    /// Executable used to launch the adapter.
    pub program: PathBuf,
    /// Exact arguments supplied without a shell.
    pub arguments: Vec<OsString>,
    /// Adapter implementation artifact whose bytes identify the tested subject.
    pub subject: PathBuf,
    /// Maximum wait for one response and clean shutdown.
    pub response_timeout_ms: u64,
}

impl AcceleratorProcessConformanceConfig {
    /// Creates a launch configuration that treats the executable as its subject.
    pub fn new(program: impl Into<PathBuf>) -> Self {
        let program = program.into();
        Self {
            subject: program.clone(),
            program,
            arguments: Vec::new(),
            response_timeout_ms: 5_000,
        }
    }

    fn validate(&self) -> Result<(), AcceleratorProcessConformanceError> {
        if self.program.as_os_str().is_empty() {
            return Err(AcceleratorProcessConformanceError::InvalidConfig(
                "adapter program is empty".to_string(),
            ));
        }
        if self.subject.as_os_str().is_empty() {
            return Err(AcceleratorProcessConformanceError::InvalidConfig(
                "adapter subject is empty".to_string(),
            ));
        }
        if !(1..=MAX_RESPONSE_TIMEOUT_MS).contains(&self.response_timeout_ms) {
            return Err(AcceleratorProcessConformanceError::InvalidConfig(format!(
                "response_timeout_ms must be within 1..={MAX_RESPONSE_TIMEOUT_MS}"
            )));
        }
        Ok(())
    }
}

/// Content-addressed implementation, launch contract, and portable inputs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorProcessConformanceSubject {
    /// Adapter implementation file name without a machine-specific parent path.
    pub adapter_file: String,
    /// SHA-256 digest of the adapter implementation bytes.
    pub adapter_sha256: String,
    /// Adapter implementation size in bytes.
    pub adapter_size_bytes: u64,
    /// Launcher file name without a machine-specific parent path.
    pub launcher_file: String,
    /// SHA-256 of the normalized argument array.
    pub arguments_sha256: String,
    /// Number of launcher arguments included in the digest.
    pub argument_count: usize,
    /// Selected manifest file name.
    pub manifest_file: String,
    /// SHA-256 of the exact selected manifest bytes.
    pub manifest_sha256: String,
    /// Runtime-contract file name.
    pub runtime_file: String,
    /// SHA-256 of the exact runtime-contract bytes.
    pub runtime_sha256: String,
    /// TaskSpec file name.
    pub task_file: String,
    /// SHA-256 of the exact TaskSpec bytes.
    pub task_sha256: String,
}

/// One canonical accelerator-process conformance verdict.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorProcessConformanceCheck {
    /// Stable check identifier.
    pub id: String,
    /// `passed`, `failed`, or `not_run`.
    pub status: String,
    /// Bounded diagnostic associated with the verdict.
    pub detail: String,
}

/// Portable process-level accelerator protocol conformance report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorProcessConformanceReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Aggregate `passed` or `failed` verdict.
    pub status: String,
    /// Content-addressed tested subject and portable inputs.
    pub subject: AcceleratorProcessConformanceSubject,
    /// Stable adapter identifier from the selected manifest.
    pub adapter_id: String,
    /// Stable task identifier from the bound TaskSpec.
    pub task_id: String,
    /// Bound TaskSpec schema.
    pub task_spec_schema: u32,
    /// Canonical TaskSpec digest used by the protocol transcript.
    pub task_spec_sha256: String,
    /// Exercised accelerator protocol schema.
    pub protocol_schema: u32,
    /// Root seed used by the deterministic lifecycle.
    pub root_seed: u64,
    /// Batch width used by the deterministic lifecycle.
    pub batch_width: usize,
    /// Canonically ordered conformance verdicts.
    pub checks: Vec<AcceleratorProcessConformanceCheck>,
    /// Valid request/response envelopes observed before success or first failure.
    pub frames: Vec<AcceleratorProtocolFrame>,
}

impl AcceleratorProcessConformanceReport {
    /// Returns true only when every check passed and all nine exchanges exist.
    pub fn passed(&self) -> bool {
        self.status == "passed"
            && self.frames.len() == OPERATIONS.len()
            && self.checks.iter().all(|check| check.status == "passed")
    }

    /// Validates schema, subject digests, check order, and partial transcript order.
    pub fn validate(&self) -> Result<(), AcceleratorProcessConformanceError> {
        if self.kind != ACCELERATOR_PROCESS_CONFORMANCE_REPORT_KIND
            || self.schema_version != ACCELERATOR_PROCESS_CONFORMANCE_REPORT_SCHEMA_VERSION
        {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "process conformance report kind or schema mismatch".to_string(),
            ));
        }
        if self.adapter_id.trim().is_empty()
            || self.task_id.trim().is_empty()
            || self.task_spec_schema != TASK_SPEC_SCHEMA_VERSION
            || self.protocol_schema != ACCELERATOR_PROTOCOL_SCHEMA_VERSION
            || self.batch_width != BATCH_WIDTH
        {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "process conformance binding is invalid".to_string(),
            ));
        }
        for digest in [
            self.subject.adapter_sha256.as_str(),
            self.subject.arguments_sha256.as_str(),
            self.subject.manifest_sha256.as_str(),
            self.subject.runtime_sha256.as_str(),
            self.subject.task_sha256.as_str(),
            self.task_spec_sha256.as_str(),
        ] {
            validate_sha256(digest)?;
        }
        if self.subject.adapter_file.is_empty()
            || self.subject.launcher_file.is_empty()
            || self.subject.manifest_file.is_empty()
            || self.subject.runtime_file.is_empty()
            || self.subject.task_file.is_empty()
            || self.subject.adapter_size_bytes == 0
        {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "process conformance subject is incomplete".to_string(),
            ));
        }
        if self
            .checks
            .iter()
            .map(|check| check.id.as_str())
            .ne(CHECK_IDS)
        {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "process conformance check registry is not canonical".to_string(),
            ));
        }
        let mut failure_seen = false;
        for check in &self.checks {
            if !matches!(check.status.as_str(), "passed" | "failed" | "not_run")
                || check.detail.chars().count() > 512
            {
                return Err(AcceleratorProcessConformanceError::InvalidReport(
                    "process conformance check status or detail is invalid".to_string(),
                ));
            }
            match check.status.as_str() {
                "passed" if failure_seen => {
                    return Err(AcceleratorProcessConformanceError::InvalidReport(
                        "a conformance check passed after an earlier failure".to_string(),
                    ));
                }
                "failed" if failure_seen => {
                    return Err(AcceleratorProcessConformanceError::InvalidReport(
                        "more than one conformance check failed".to_string(),
                    ));
                }
                "failed" => failure_seen = true,
                "not_run" if !failure_seen => {
                    return Err(AcceleratorProcessConformanceError::InvalidReport(
                        "a conformance check was skipped before any failure".to_string(),
                    ));
                }
                _ => {}
            }
        }
        let passed_operations = self.checks[1..10]
            .iter()
            .filter(|check| check.status == "passed")
            .count();
        if self.frames.len() != passed_operations || self.frames.len() > OPERATIONS.len() {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "observed frame count differs from passed operation checks".to_string(),
            ));
        }
        for (index, frame) in self.frames.iter().enumerate() {
            validate_request_envelope(
                &frame.request,
                index as u64,
                OPERATIONS[index],
                self.protocol_schema,
            )?;
            validate_response_envelope(
                &frame.response,
                index as u64,
                index != 7,
                self.protocol_schema,
            )?;
        }
        let expected_status = if !failure_seen
            && self.frames.len() == OPERATIONS.len()
            && self.checks.iter().all(|check| check.status == "passed")
        {
            "passed"
        } else {
            "failed"
        };
        if self.status != expected_status {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "aggregate process conformance status mismatch".to_string(),
            ));
        }
        Ok(())
    }

    /// Serializes a validated report as stable pretty JSON with a trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, AcceleratorProcessConformanceError> {
        self.validate()?;
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }

    /// Binds a passing report and its complete transcript to selected contracts.
    pub fn validate_against(
        &self,
        manifest: &AcceleratorManifest,
        runtime: &AcceleratorRuntimeContract,
        task: &TaskSpec,
    ) -> Result<(), AcceleratorProcessConformanceError> {
        self.validate()?;
        if !self.passed()
            || self.adapter_id != manifest.id
            || self.task_id != task.task_id
            || self.task_spec_sha256 != task_spec_sha256(task)?
            || self.protocol_schema != manifest.protocol_schema
            || !manifest.supported_batch_widths.contains(&self.batch_width)
        {
            return Err(AcceleratorProcessConformanceError::InvalidReport(
                "process conformance report differs from selected contracts".to_string(),
            ));
        }
        self.transcript()
            .validate_against(manifest, runtime, task)?;
        Ok(())
    }

    fn new(
        subject: AcceleratorProcessConformanceSubject,
        manifest: &AcceleratorManifest,
        task: &TaskSpec,
    ) -> Result<Self, AcceleratorProcessConformanceError> {
        Ok(Self {
            kind: ACCELERATOR_PROCESS_CONFORMANCE_REPORT_KIND.to_string(),
            schema_version: ACCELERATOR_PROCESS_CONFORMANCE_REPORT_SCHEMA_VERSION,
            status: "failed".to_string(),
            subject,
            adapter_id: manifest.id.clone(),
            task_id: task.task_id.clone(),
            task_spec_schema: TASK_SPEC_SCHEMA_VERSION,
            task_spec_sha256: task_spec_sha256(task)?,
            protocol_schema: ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            root_seed: ROOT_SEED,
            batch_width: BATCH_WIDTH,
            checks: CHECK_IDS
                .iter()
                .map(|id| AcceleratorProcessConformanceCheck {
                    id: (*id).to_string(),
                    status: "not_run".to_string(),
                    detail: String::new(),
                })
                .collect(),
            frames: Vec::new(),
        })
    }

    fn verdict(&mut self, id: &str, result: Result<&str, String>) {
        let check = self
            .checks
            .iter_mut()
            .find(|check| check.id == id)
            .expect("canonical accelerator process conformance check");
        match result {
            Ok(detail) => {
                check.status = "passed".to_string();
                check.detail = detail.to_string();
            }
            Err(detail) => {
                check.status = "failed".to_string();
                check.detail = detail.chars().take(512).collect();
            }
        }
        self.status = if self.checks.iter().all(|check| check.status == "passed") {
            "passed"
        } else {
            "failed"
        }
        .to_string();
    }

    fn transcript(&self) -> AcceleratorProtocolTranscript {
        AcceleratorProtocolTranscript {
            kind: super::ACCELERATOR_PROTOCOL_TRANSCRIPT_KIND.to_string(),
            schema_version: super::ACCELERATOR_PROTOCOL_TRANSCRIPT_SCHEMA_VERSION,
            protocol_schema: self.protocol_schema,
            adapter_id: self.adapter_id.clone(),
            task_id: self.task_id.clone(),
            task_spec_schema: self.task_spec_schema,
            task_spec_sha256: self.task_spec_sha256.clone(),
            root_seed: self.root_seed,
            batch_width: self.batch_width,
            frames: self.frames.clone(),
        }
    }
}

/// Failure reading inputs, validating configuration, or serializing a report.
#[derive(Debug, thiserror::Error)]
pub enum AcceleratorProcessConformanceError {
    /// A bounded conformance input could not be read.
    #[error("read accelerator conformance input {path}: {message}")]
    Read {
        /// Input path.
        path: String,
        /// Operating-system or bound diagnostic.
        message: String,
    },
    /// Process launch configuration is invalid.
    #[error("invalid accelerator conformance config: {0}")]
    InvalidConfig(String),
    /// Manifest, runtime contract, or TaskSpec input is invalid.
    #[error("invalid accelerator conformance contract: {0}")]
    InvalidContract(String),
    /// Report fields or aggregate verdict are inconsistent.
    #[error("invalid accelerator conformance report: {0}")]
    InvalidReport(String),
    /// An embedded accelerator contract failed validation.
    #[error(transparent)]
    Contract(#[from] super::AcceleratorContractError),
    /// JSON parsing or serialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Runs the complete deterministic protocol-v1 lifecycle against one fresh process.
///
/// Input and report-shape failures return an error. Spawn, timeout, protocol, and
/// semantic failures return a valid failed report with later checks marked `not_run`.
pub fn run_accelerator_process_conformance(
    manifest_path: &Path,
    runtime_path: &Path,
    task_path: &Path,
    config: &AcceleratorProcessConformanceConfig,
) -> Result<AcceleratorProcessConformanceReport, AcceleratorProcessConformanceError> {
    config.validate()?;
    let manifest_bytes = read_input(manifest_path, MAX_CONTRACT_BYTES)?;
    let runtime_bytes = read_input(runtime_path, MAX_CONTRACT_BYTES)?;
    let task_bytes = read_input(task_path, MAX_CONTRACT_BYTES)?;
    let subject_bytes = read_input(&config.subject, MAX_SUBJECT_BYTES)?;
    let manifest_text = std::str::from_utf8(&manifest_bytes).map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "manifest is not UTF-8: {error}"
        ))
    })?;
    let runtime_text = std::str::from_utf8(&runtime_bytes).map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "runtime contract is not UTF-8: {error}"
        ))
    })?;
    let manifest: AcceleratorManifest = toml::from_str(manifest_text).map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "parse accelerator manifest: {error}"
        ))
    })?;
    let runtime: AcceleratorRuntimeContract = toml::from_str(runtime_text).map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "parse accelerator runtime contract: {error}"
        ))
    })?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes).map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "parse accelerator TaskSpec: {error}"
        ))
    })?;
    manifest.validate()?;
    runtime.validate()?;
    task.validate().map_err(|error| {
        AcceleratorProcessConformanceError::InvalidContract(format!(
            "validate accelerator TaskSpec: {error}"
        ))
    })?;
    if !manifest.supported_batch_widths.contains(&BATCH_WIDTH) {
        return Err(AcceleratorProcessConformanceError::InvalidContract(
            "manifest does not advertise batch width one".to_string(),
        ));
    }
    let normalized_arguments = normalized_arguments(config)?;
    let subject = AcceleratorProcessConformanceSubject {
        adapter_file: file_name(&config.subject),
        adapter_sha256: sha256_hex(&subject_bytes),
        adapter_size_bytes: subject_bytes.len() as u64,
        launcher_file: file_name(&config.program),
        arguments_sha256: sha256_hex(&serde_json::to_vec(&normalized_arguments)?),
        argument_count: normalized_arguments.len(),
        manifest_file: file_name(manifest_path),
        manifest_sha256: sha256_hex(&manifest_bytes),
        runtime_file: file_name(runtime_path),
        runtime_sha256: sha256_hex(&runtime_bytes),
        task_file: file_name(task_path),
        task_sha256: sha256_hex(&task_bytes),
    };
    let mut report = AcceleratorProcessConformanceReport::new(subject, &manifest, &task)?;
    let mut process = match AdapterProcess::spawn(config) {
        Ok(process) => {
            report.verdict(
                "spawn",
                Ok("fresh adapter process launched without a shell"),
            );
            process
        }
        Err(error) => {
            report.verdict("spawn", Err(error));
            report.validate()?;
            return Ok(report);
        }
    };
    let task_value = serde_json::to_value(&task)?;
    let mut checkpoint = None;
    for index in 0..OPERATIONS.len() {
        let request = request(index, &task_value, checkpoint.as_ref());
        let success = index != 7;
        let response = match process.exchange(&request, index as u64, success) {
            Ok(response) => response,
            Err(error) => {
                report.verdict(CHECK_IDS[index + 1], Err(error));
                report.validate()?;
                return Ok(report);
            }
        };
        if index == 0 {
            let result = response.get("result").cloned().ok_or_else(|| {
                AcceleratorProcessConformanceError::InvalidReport(
                    "probe response omitted result after envelope validation".to_string(),
                )
            })?;
            let capability: AcceleratorCapabilityReport = match serde_json::from_value(result) {
                Ok(capability) => capability,
                Err(error) => {
                    report.verdict("probe", Err(format!("invalid capability report: {error}")));
                    report.validate()?;
                    return Ok(report);
                }
            };
            if let Err(error) = capability.validate_against(&manifest, &runtime, &task) {
                report.verdict("probe", Err(format!("capability binding failed: {error}")));
                report.validate()?;
                return Ok(report);
            }
        }
        if index == 4 {
            checkpoint = response.get("result").cloned();
            if checkpoint.is_none() {
                report.verdict(
                    "checkpoint",
                    Err("checkpoint response omitted result".to_string()),
                );
                report.validate()?;
                return Ok(report);
            }
        }
        if index == 8 {
            if let Err(error) = process.finish() {
                report.verdict("shutdown", Err(error));
                report.validate()?;
                return Ok(report);
            }
        }
        report
            .frames
            .push(AcceleratorProtocolFrame { request, response });
        report.verdict(CHECK_IDS[index + 1], Ok(operation_detail(index)));
    }
    match report
        .transcript()
        .validate_against(&manifest, &runtime, &task)
    {
        Ok(()) => report.verdict(
            "transcript_binding",
            Ok("all exchanges bind to the manifest, runtime contract, TaskSpec, and checkpoint"),
        ),
        Err(error) => report.verdict("transcript_binding", Err(error.to_string())),
    }
    report.validate()?;
    Ok(report)
}

fn request(index: usize, task: &Value, checkpoint: Option<&Value>) -> Value {
    let base = || {
        json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": index as u64,
            "operation": OPERATIONS[index],
        })
    };
    match index {
        1 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 1,
            "operation": "create",
            "session_id": SESSION_ID,
            "task_spec": task,
            "root_seed": ROOT_SEED,
            "batch_width": BATCH_WIDTH,
            "auto_reset": false,
        }),
        2 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 2,
            "operation": "reset_lanes",
            "session_id": SESSION_ID,
            "lane_ids": [0],
        }),
        3 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 3,
            "operation": "step",
            "session_id": SESSION_ID,
            "actions": [[0.0]],
        }),
        4 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 4,
            "operation": "checkpoint",
            "session_id": SESSION_ID,
        }),
        5 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 5,
            "operation": "restore",
            "session_id": SESSION_ID,
            "checkpoint": checkpoint.expect("checkpoint response precedes restore"),
        }),
        6 => json!({
            "kind": "rne_accelerator_request",
            "schema_version": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "request_id": 6,
            "operation": "close",
            "session_id": SESSION_ID,
        }),
        _ => base(),
    }
}

fn operation_detail(index: usize) -> &'static str {
    [
        "capability report is bound to selected contracts",
        "session created with exact TaskSpec, seed, width, and reset mode",
        "lane zero reset through its next deterministic episode seed",
        "one exact action produced a finite correlated step",
        "portable checkpoint returned after reset and step",
        "exact checkpoint restored the checkpointed lane state",
        "session closed explicitly",
        "unsupported operation failed with a stable protocol error",
        "adapter acknowledged shutdown and exited successfully",
    ][index]
}

struct AdapterProcess {
    child: Child,
    stdin: Option<ChildStdin>,
    lines: Receiver<Result<Option<Vec<u8>>, String>>,
    reader: Option<JoinHandle<()>>,
    timeout: Duration,
    finished: bool,
}

impl AdapterProcess {
    fn spawn(config: &AcceleratorProcessConformanceConfig) -> Result<Self, String> {
        let mut child = Command::new(&config.program)
            .args(&config.arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|error| format!("could not spawn accelerator process: {error}"))?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| "accelerator stdin was not piped".to_string())?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| "accelerator stdout was not piped".to_string())?;
        let (sender, lines) = mpsc::sync_channel(1);
        let reader = thread::spawn(move || {
            let mut stdout = std::io::BufReader::new(stdout);
            loop {
                let result = read_bounded_line(&mut stdout);
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
            timeout: Duration::from_millis(config.response_timeout_ms),
            finished: false,
        })
    }

    fn exchange(
        &mut self,
        request: &Value,
        request_id: u64,
        success: bool,
    ) -> Result<Value, String> {
        let mut encoded = serde_json::to_vec(request)
            .map_err(|error| format!("could not encode accelerator request: {error}"))?;
        encoded.push(b'\n');
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| "accelerator stdin is closed".to_string())?;
        stdin
            .write_all(&encoded)
            .and_then(|()| stdin.flush())
            .map_err(|error| format!("could not write accelerator request: {error}"))?;
        let line = match self.lines.recv_timeout(self.timeout) {
            Ok(Ok(Some(line))) => line,
            Ok(Ok(None)) => return Err("accelerator exited before responding".to_string()),
            Ok(Err(error)) => return Err(format!("could not read accelerator response: {error}")),
            Err(RecvTimeoutError::Timeout) => {
                return Err(format!(
                    "accelerator response exceeded {} ms",
                    self.timeout.as_millis()
                ));
            }
            Err(RecvTimeoutError::Disconnected) => {
                return Err("accelerator response reader stopped".to_string());
            }
        };
        let response: Value = serde_json::from_slice(&line)
            .map_err(|error| format!("accelerator response is not strict JSON: {error}"))?;
        validate_response_envelope(
            &response,
            request_id,
            success,
            ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
        )
        .map_err(|error| format!("invalid accelerator response envelope: {error}"))?;
        Ok(response)
    }

    fn finish(&mut self) -> Result<(), String> {
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
                    let _ = self.drain_stdout_after_exit();
                    self.join_reader();
                    return Err(format!(
                        "accelerator did not exit within {} ms after shutdown",
                        self.timeout.as_millis()
                    ));
                }
                Err(error) => {
                    return Err(format!("could not wait for accelerator process: {error}"));
                }
            }
        };
        self.finished = true;
        let stdout_result = self.drain_stdout_after_exit();
        self.join_reader();
        if !status.success() {
            return Err(format!("accelerator exited with status {status}"));
        }
        stdout_result
    }

    fn drain_stdout_after_exit(&mut self) -> Result<(), String> {
        let mut extra_lines = 0_usize;
        loop {
            match self.lines.recv_timeout(self.timeout) {
                Ok(Ok(Some(_))) => extra_lines = extra_lines.saturating_add(1),
                Ok(Ok(None)) | Err(RecvTimeoutError::Disconnected) => break,
                Ok(Err(error)) => {
                    return Err(format!("could not finish accelerator stdout: {error}"));
                }
                Err(RecvTimeoutError::Timeout) => {
                    return Err(format!(
                        "accelerator stdout did not close within {} ms",
                        self.timeout.as_millis()
                    ));
                }
            }
        }
        if extra_lines == 0 {
            Ok(())
        } else {
            Err(format!(
                "accelerator emitted {extra_lines} unexpected stdout line(s) after shutdown"
            ))
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
            let _ = self.drain_stdout_after_exit();
        }
        self.join_reader();
    }
}

fn read_bounded_line(reader: &mut impl BufRead) -> Result<Option<Vec<u8>>, String> {
    let mut line = Vec::new();
    loop {
        let available = reader
            .fill_buf()
            .map_err(|error| format!("read accelerator stdout: {error}"))?;
        if available.is_empty() {
            return if line.is_empty() {
                Ok(None)
            } else {
                Err("accelerator response ended without a newline".to_string())
            };
        }
        let take = available
            .iter()
            .position(|byte| *byte == b'\n')
            .map_or(available.len(), |index| index + 1);
        if line.len().saturating_add(take) > MAX_PROTOCOL_LINE_BYTES {
            return Err(format!(
                "accelerator response exceeds {MAX_PROTOCOL_LINE_BYTES} bytes"
            ));
        }
        line.extend_from_slice(&available[..take]);
        reader.consume(take);
        if line.last() == Some(&b'\n') {
            line.pop();
            if line.last() == Some(&b'\r') {
                line.pop();
            }
            return Ok(Some(line));
        }
    }
}

fn normalized_arguments(
    config: &AcceleratorProcessConformanceConfig,
) -> Result<Vec<String>, AcceleratorProcessConformanceError> {
    let subject = config.subject.to_str().ok_or_else(|| {
        AcceleratorProcessConformanceError::InvalidConfig(
            "adapter subject path must be valid Unicode".to_string(),
        )
    })?;
    config
        .arguments
        .iter()
        .enumerate()
        .map(|(index, argument)| {
            let argument = argument.to_str().ok_or_else(|| {
                AcceleratorProcessConformanceError::InvalidConfig(format!(
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

fn read_input(path: &Path, maximum: u64) -> Result<Vec<u8>, AcceleratorProcessConformanceError> {
    let metadata =
        fs::symlink_metadata(path).map_err(|error| AcceleratorProcessConformanceError::Read {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    if !metadata.file_type().is_file() || metadata.len() == 0 || metadata.len() > maximum {
        return Err(AcceleratorProcessConformanceError::Read {
            path: path.display().to_string(),
            message: format!("input must be a non-empty regular file at or below {maximum} bytes"),
        });
    }
    let file = fs::File::open(path).map_err(|error| AcceleratorProcessConformanceError::Read {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    file.take(maximum + 1)
        .read_to_end(&mut bytes)
        .map_err(|error| AcceleratorProcessConformanceError::Read {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    if bytes.len() as u64 != metadata.len() {
        return Err(AcceleratorProcessConformanceError::Read {
            path: path.display().to_string(),
            message: "input changed while it was being read".to_string(),
        });
    }
    Ok(bytes)
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

fn validate_sha256(value: &str) -> Result<(), AcceleratorProcessConformanceError> {
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        Ok(())
    } else {
        Err(AcceleratorProcessConformanceError::InvalidReport(
            "process conformance digest is not lowercase SHA-256 hex".to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject() -> AcceleratorProcessConformanceSubject {
        AcceleratorProcessConformanceSubject {
            adapter_file: "adapter.py".to_string(),
            adapter_sha256: "0".repeat(64),
            adapter_size_bytes: 1,
            launcher_file: "python".to_string(),
            arguments_sha256: "1".repeat(64),
            argument_count: 1,
            manifest_file: "accelerator.toml".to_string(),
            manifest_sha256: "2".repeat(64),
            runtime_file: "runtime.toml".to_string(),
            runtime_sha256: "3".repeat(64),
            task_file: "task.json".to_string(),
            task_sha256: "4".repeat(64),
        }
    }

    #[test]
    fn timeout_and_unknown_report_fields_fail_closed() {
        let mut config = AcceleratorProcessConformanceConfig::new("adapter");
        config.response_timeout_ms = 0;
        assert!(config.validate().is_err());
        config.response_timeout_ms = MAX_RESPONSE_TIMEOUT_MS + 1;
        assert!(config.validate().is_err());

        let value = json!({
            "kind": ACCELERATOR_PROCESS_CONFORMANCE_REPORT_KIND,
            "schema_version": ACCELERATOR_PROCESS_CONFORMANCE_REPORT_SCHEMA_VERSION,
            "status": "failed",
            "subject": subject(),
            "adapter_id": "adapter",
            "task_id": "task",
            "task_spec_schema": TASK_SPEC_SCHEMA_VERSION,
            "task_spec_sha256": "5".repeat(64),
            "protocol_schema": ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "root_seed": ROOT_SEED,
            "batch_width": BATCH_WIDTH,
            "checks": [],
            "frames": [],
            "unknown": true,
        });
        assert!(serde_json::from_value::<AcceleratorProcessConformanceReport>(value).is_err());
    }
}
