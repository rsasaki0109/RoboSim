//! Content-addressed recorded-playback and live-shadow session evidence.
//!
//! This module binds the existing fail-closed gateway and ordered shadow
//! comparator to explicit clock, latency, drop, calibration, TaskSpec, and
//! controller provenance. It remains transport-neutral and never reads a wall
//! clock or grants actuator authority.

use crate::shadow::{
    ShadowComparator, ShadowComparisonConfig, ShadowComparisonError, ShadowComparisonReport,
    ShadowTensorTolerance,
};
use crate::{
    CommandDisposition, GatewayBuildError, GatewayConfig, GatewayError, GatewayEvidence,
    HardwareGateway, HardwareMode,
};
use rne_ai::{TaskSpec, TaskSpecValidationError};
use serde::{Deserialize, Serialize};

/// Schema version for recorded/shadow session inputs and reports.
pub const RECORDED_SHADOW_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for a bounded recorded/shadow session input.
pub const RECORDED_SHADOW_SESSION_KIND: &str = "rne_recorded_shadow_session";

/// Stable discriminator for a completed recorded/shadow report.
pub const RECORDED_SHADOW_REPORT_KIND: &str = "rne_recorded_shadow_report";

/// One machine-independent content-addressed input artifact.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedArtifactBinding {
    /// Unique semantic role inside the session.
    pub role: String,
    /// Stable artifact kind or backend identifier.
    pub kind: String,
    /// File name without a machine-specific parent directory.
    pub file_name: String,
    /// Lowercase SHA-256 of the exact artifact bytes.
    pub sha256: String,
}

/// Content-addressed calibration used to normalize one or more observations.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationBinding {
    /// Stable calibration role, such as `joint_state`.
    pub role: String,
    /// Calibration model discriminator.
    pub kind: String,
    /// Lowercase SHA-256 of the exact calibration artifact bytes.
    pub sha256: String,
}

/// Explicit timing, unit, drop, memory, and calibration contract.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedStreamContract {
    /// Named source of capture timestamps.
    pub clock_source: String,
    /// Number of nanoseconds represented by one timestamp tick.
    pub tick_period_ns: u64,
    /// Required nominal capture-to-availability latency.
    pub nominal_latency_ticks: u64,
    /// Inclusive maximum capture-to-availability latency.
    pub maximum_latency_ticks: u64,
    /// Policy for sequence gaps; v1 requires explicit counted gaps.
    pub drop_policy: String,
    /// Maximum number of paired samples retained in the report.
    pub sample_capacity: usize,
    /// TaskSpec tensor names and units in exact observation order.
    pub tensor_units: Vec<RecordedTensorUnit>,
    /// Content-addressed calibrations applied before values enter the session.
    pub calibrations: Vec<CalibrationBinding>,
}

/// One TaskSpec observation tensor and its declared SI unit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedTensorUnit {
    /// Tensor name in TaskSpec order.
    pub tensor_name: String,
    /// Unit declared by the TaskSpec.
    pub unit: String,
}

/// One controller decision paired to a timestamped recorded observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedShadowFrame {
    /// Strictly increasing source observation sequence.
    pub observation_sequence: u64,
    /// Explicit number of unavailable source sequences before this frame.
    pub dropped_sequences_before: u64,
    /// Source capture timestamp in the declared clock ticks.
    pub captured_at_ticks: u64,
    /// Host-available timestamp in the same clock domain.
    pub available_at_ticks: u64,
    /// Deterministic simulation step paired with this source observation.
    pub simulation_step: u64,
    /// Paired SimClock time in nanosecond ticks.
    pub simulation_time_ticks: u64,
    /// Calibrated source values in flattened TaskSpec order.
    pub recorded_values: Vec<f64>,
    /// Simulation values in flattened TaskSpec order.
    pub simulation_values: Vec<f64>,
    /// Strictly increasing controller action sequence.
    pub action_sequence: u64,
    /// Timestamp when the controller action reached the gateway boundary.
    pub action_submitted_at_ticks: u64,
    /// Controller action in flattened TaskSpec order.
    pub action_values: Vec<f64>,
}

/// One bounded session input shared by playback and shadow authority modes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedShadowSession {
    /// Stable input discriminator.
    pub kind: String,
    /// Input schema version.
    pub schema_version: u32,
    /// Stable experiment identifier owning the declared gates.
    pub experiment_id: String,
    /// SHA-256 of the exact predeclared requirements artifact.
    pub requirements_sha256: String,
    /// Exact TaskSpec identifier.
    pub task_id: String,
    /// Lowercase SHA-256 of the exact TaskSpec artifact.
    pub task_sha256: String,
    /// Exact controller identifier.
    pub controller_id: String,
    /// Lowercase SHA-256 of the exact controller artifact.
    pub controller_sha256: String,
    /// Content-addressed actions, traces, models, and configurations.
    pub sources: Vec<RecordedArtifactBinding>,
    /// Count of controller bootstrap actions preceding the first observation.
    pub bootstrap_action_count: u64,
    /// Explicit stream contract.
    pub stream: RecordedStreamContract,
    /// One absolute tolerance per TaskSpec observation tensor.
    pub tolerances: Vec<ShadowTensorTolerance>,
    /// Ordered observation/action pairs.
    pub frames: Vec<RecordedShadowFrame>,
    /// Optional injected transport disconnect after this observation sequence.
    pub disconnect_after_observation_sequence: Option<u64>,
}

impl RecordedShadowSession {
    /// Rebinds an untrusted session envelope to the supplied portable TaskSpec.
    pub fn validate_against(&self, task: &TaskSpec) -> Result<(), RecordedShadowError> {
        validate_session(task, self)
    }
}

/// Aggregate timing, drop, authority, and terminal outcome evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedShadowSummary {
    /// Number of observation/action pairs accepted before termination.
    pub accepted_samples: usize,
    /// Number of explicit source sequence drops.
    pub dropped_observations: u64,
    /// Number of valid actions suppressed by non-actuating authority.
    pub suppressed_actions: usize,
    /// Maximum observed capture-to-availability latency.
    pub maximum_observed_latency_ticks: u64,
    /// Whether every numeric comparison stayed inside tolerance.
    pub comparison_passed: bool,
    /// Whether the transport disconnected at the declared injected sequence.
    pub transport_failure_observed: bool,
    /// Whether any actuator write was emitted.
    pub actuator_writes_emitted: bool,
    /// Stable terminal classification.
    pub status: String,
}

/// Versioned, self-contained evidence for a recorded or shadow session.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedShadowReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// SHA-256 of the exact session input artifact.
    pub session_sha256: String,
    /// Stable experiment identifier owning the declared gates.
    pub experiment_id: String,
    /// SHA-256 of the exact predeclared requirements artifact.
    pub requirements_sha256: String,
    /// Exact TaskSpec identifier.
    pub task_id: String,
    /// SHA-256 of the exact TaskSpec artifact.
    pub task_sha256: String,
    /// Exact controller identifier.
    pub controller_id: String,
    /// SHA-256 of the exact controller artifact.
    pub controller_sha256: String,
    /// Content-addressed actions, traces, models, and configurations.
    pub sources: Vec<RecordedArtifactBinding>,
    /// Non-actuating gateway mode used for this execution.
    pub mode: HardwareMode,
    /// Input stream contract retained without weakening.
    pub stream: RecordedStreamContract,
    /// Controller bootstrap actions preceding source observations.
    pub bootstrap_action_count: u64,
    /// Ordered numeric comparison report.
    pub comparison: ShadowComparisonReport,
    /// Fail-closed gateway audit evidence.
    pub gateway: GatewayEvidence,
    /// Aggregate verdict.
    pub summary: RecordedShadowSummary,
}

/// Evaluates one bounded session through playback or shadow authority.
pub fn evaluate_recorded_shadow_session(
    task: TaskSpec,
    session: RecordedShadowSession,
    session_sha256: String,
    mode: HardwareMode,
) -> Result<RecordedShadowReport, RecordedShadowError> {
    if !matches!(mode, HardwareMode::Playback | HardwareMode::Shadow) {
        return Err(RecordedShadowError::Invalid(
            "recorded/shadow evaluation requires playback or shadow mode",
        ));
    }
    if !is_sha256(&session_sha256) {
        return Err(RecordedShadowError::Invalid(
            "session artifact digest is invalid",
        ));
    }
    validate_session(&task, &session)?;
    let event_capacity = session
        .frames
        .len()
        .checked_mul(3)
        .and_then(|value| value.checked_add(4))
        .ok_or(RecordedShadowError::Invalid("event capacity overflow"))?;
    let timing_limit_ms = ticks_to_ms_ceil(
        session.stream.maximum_latency_ticks,
        session.stream.tick_period_ns,
    )?
    .max(1);
    let mut gateway = HardwareGateway::new(
        task.clone(),
        GatewayConfig {
            mode,
            max_observation_age_ms: timing_limit_ms,
            command_deadline_ms: timing_limit_ms,
            max_command_age_ms: timing_limit_ms,
            observation_capacity: session.stream.sample_capacity,
            actuation_capacity: 1,
            event_capacity,
        },
    )?;
    let first = session.frames.first().ok_or(RecordedShadowError::Invalid(
        "recorded/shadow session must contain at least one frame",
    ))?;
    gateway.connect(ticks_to_ms_ceil(
        first.available_at_ticks,
        session.stream.tick_period_ns,
    )?)?;
    let mut comparator = ShadowComparator::new(
        task,
        ShadowComparisonConfig {
            sample_capacity: session.stream.sample_capacity,
            tensors: session.tolerances.clone(),
        },
    )?;
    let mut dropped_observations = 0_u64;
    let mut suppressed_actions = 0_usize;
    let mut maximum_observed_latency_ticks = 0_u64;
    let mut transport_failure_observed = false;
    let mut actuator_writes_emitted = false;
    for frame in &session.frames {
        let latency = frame.available_at_ticks - frame.captured_at_ticks;
        maximum_observed_latency_ticks = maximum_observed_latency_ticks.max(latency);
        dropped_observations = dropped_observations
            .checked_add(frame.dropped_sequences_before)
            .ok_or(RecordedShadowError::Invalid("drop count overflow"))?;
        let available_ms =
            ticks_to_ms_ceil(frame.available_at_ticks, session.stream.tick_period_ns)?;
        gateway.ingest_observation(
            available_ms,
            frame.observation_sequence,
            frame.recorded_values.clone(),
        )?;
        comparator.compare(
            crate::HardwareObservation {
                sequence: frame.observation_sequence,
                received_at_ms: available_ms,
                values: frame.recorded_values.clone(),
            },
            frame.simulation_step,
            frame.simulation_time_ticks,
            frame.simulation_values.clone(),
        )?;
        let submitted_ms = ticks_to_ms_ceil(
            frame.action_submitted_at_ticks,
            session.stream.tick_period_ns,
        )?;
        if gateway.submit_action(
            submitted_ms,
            frame.action_sequence,
            frame.observation_sequence,
            frame.action_values.clone(),
        )? != CommandDisposition::Suppressed
        {
            return Err(RecordedShadowError::Invalid(
                "non-actuating session unexpectedly queued an action",
            ));
        }
        suppressed_actions += 1;
        actuator_writes_emitted |= gateway.poll_actuation(submitted_ms)?.is_some();
        if session.disconnect_after_observation_sequence == Some(frame.observation_sequence) {
            gateway.disconnect(submitted_ms)?;
            transport_failure_observed = true;
            break;
        }
    }
    let comparison = comparator.finish()?;
    if !transport_failure_observed {
        let last_ms = session
            .frames
            .last()
            .map(|frame| {
                ticks_to_ms_ceil(
                    frame.action_submitted_at_ticks,
                    session.stream.tick_period_ns,
                )
            })
            .transpose()?
            .ok_or(RecordedShadowError::Invalid("session has no terminal time"))?;
        gateway.close_cleanly(last_ms)?;
    }
    let expected_transport_failure = session.disconnect_after_observation_sequence.is_some();
    let status = if expected_transport_failure
        && transport_failure_observed
        && !actuator_writes_emitted
    {
        "failed_as_expected"
    } else if !expected_transport_failure && comparison.summary.passed && !actuator_writes_emitted {
        "passed"
    } else {
        "failed"
    };
    let accepted_samples = comparison.summary.compared_samples;
    let comparison_passed = comparison.summary.passed;
    Ok(RecordedShadowReport {
        kind: RECORDED_SHADOW_REPORT_KIND.to_string(),
        schema_version: RECORDED_SHADOW_SCHEMA_VERSION,
        session_sha256,
        experiment_id: session.experiment_id,
        requirements_sha256: session.requirements_sha256,
        task_id: session.task_id,
        task_sha256: session.task_sha256,
        controller_id: session.controller_id,
        controller_sha256: session.controller_sha256,
        sources: session.sources,
        mode,
        stream: session.stream,
        bootstrap_action_count: session.bootstrap_action_count,
        comparison,
        gateway: gateway.take_evidence(),
        summary: RecordedShadowSummary {
            accepted_samples,
            dropped_observations,
            suppressed_actions,
            maximum_observed_latency_ticks,
            comparison_passed,
            transport_failure_observed,
            actuator_writes_emitted,
            status: status.to_string(),
        },
    })
}

fn validate_session(
    task: &TaskSpec,
    session: &RecordedShadowSession,
) -> Result<(), RecordedShadowError> {
    task.validate()?;
    if session.kind != RECORDED_SHADOW_SESSION_KIND
        || session.schema_version != RECORDED_SHADOW_SCHEMA_VERSION
        || session.experiment_id.is_empty()
        || !is_sha256(&session.requirements_sha256)
        || session.task_id != task.task_id
        || !is_sha256(&session.task_sha256)
        || !is_sha256(&session.controller_sha256)
    {
        return Err(RecordedShadowError::Invalid(
            "session kind, schema, identity, or digest is invalid",
        ));
    }
    if session.controller_id.is_empty()
        || session.sources.is_empty()
        || session.sources.iter().any(|binding| {
            binding.role.is_empty()
                || binding.kind.is_empty()
                || binding.file_name.is_empty()
                || binding.file_name.contains('/')
                || binding.file_name.contains('\\')
                || !is_sha256(&binding.sha256)
        })
        || {
            let mut roles: Vec<_> = session
                .sources
                .iter()
                .map(|binding| binding.role.as_str())
                .collect();
            roles.sort_unstable();
            roles.windows(2).any(|pair| pair[0] == pair[1])
        }
        || session.stream.clock_source.is_empty()
        || session.stream.tick_period_ns == 0
        || session.stream.nominal_latency_ticks > session.stream.maximum_latency_ticks
        || session.stream.drop_policy != "explicit_sequence_gap_v1"
        || session.stream.sample_capacity == 0
        || session.frames.is_empty()
        || session.frames.len() > session.stream.sample_capacity
        || session.stream.calibrations.is_empty()
        || session.stream.calibrations.iter().any(|binding| {
            binding.role.is_empty() || binding.kind.is_empty() || !is_sha256(&binding.sha256)
        })
    {
        return Err(RecordedShadowError::Invalid("invalid stream contract"));
    }
    let expected_units: Vec<_> = task
        .observation
        .tensors
        .iter()
        .map(|tensor| RecordedTensorUnit {
            tensor_name: tensor.name.clone(),
            unit: tensor.unit.clone(),
        })
        .collect();
    if session.stream.tensor_units != expected_units {
        return Err(RecordedShadowError::Invalid(
            "stream tensor units differ from TaskSpec order",
        ));
    }
    let mut previous_observation: Option<u64> = None;
    let mut previous_capture: Option<u64> = None;
    let mut previous_available: Option<u64> = None;
    let mut previous_action: Option<u64> = None;
    for frame in &session.frames {
        if frame.available_at_ticks < frame.captured_at_ticks
            || frame.available_at_ticks - frame.captured_at_ticks
                > session.stream.maximum_latency_ticks
            || frame.action_submitted_at_ticks < frame.available_at_ticks
        {
            return Err(RecordedShadowError::Invalid(
                "frame violates latency or action timing contract",
            ));
        }
        if let Some(previous) = previous_observation {
            let expected = previous
                .checked_add(1)
                .and_then(|value| value.checked_add(frame.dropped_sequences_before))
                .ok_or(RecordedShadowError::Invalid(
                    "observation sequence overflow",
                ))?;
            if frame.observation_sequence != expected {
                return Err(RecordedShadowError::Invalid(
                    "observation gap differs from explicit drop count",
                ));
            }
        } else if frame.dropped_sequences_before != 0 {
            return Err(RecordedShadowError::Invalid(
                "first frame cannot declare a preceding drop",
            ));
        }
        if previous_capture.is_some_and(|value| frame.captured_at_ticks <= value)
            || previous_available.is_some_and(|value| frame.available_at_ticks <= value)
            || previous_action.is_some_and(|value| frame.action_sequence <= value)
        {
            return Err(RecordedShadowError::Invalid(
                "frame sequence or timestamp is not strictly increasing",
            ));
        }
        previous_observation = Some(frame.observation_sequence);
        previous_capture = Some(frame.captured_at_ticks);
        previous_available = Some(frame.available_at_ticks);
        previous_action = Some(frame.action_sequence);
    }
    if session
        .disconnect_after_observation_sequence
        .is_some_and(|sequence| {
            !session
                .frames
                .iter()
                .any(|frame| frame.observation_sequence == sequence)
        })
    {
        return Err(RecordedShadowError::Invalid(
            "disconnect sequence is absent from frames",
        ));
    }
    Ok(())
}

fn ticks_to_ms_ceil(ticks: u64, tick_period_ns: u64) -> Result<u64, RecordedShadowError> {
    ticks
        .checked_mul(tick_period_ns)
        .and_then(|ns| ns.checked_add(999_999))
        .map(|ns| ns / 1_000_000)
        .ok_or(RecordedShadowError::Invalid(
            "timestamp conversion overflow",
        ))
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

/// Failure validating or executing a recorded/shadow session.
#[derive(Debug, thiserror::Error)]
pub enum RecordedShadowError {
    /// The portable TaskSpec is invalid.
    #[error(transparent)]
    Task(#[from] TaskSpecValidationError),
    /// The TaskSpec or gateway configuration is invalid.
    #[error(transparent)]
    GatewayBuild(#[from] GatewayBuildError),
    /// The gateway rejected a runtime operation.
    #[error(transparent)]
    Gateway(#[from] GatewayError),
    /// The ordered numeric comparator rejected the session.
    #[error(transparent)]
    Comparison(#[from] ShadowComparisonError),
    /// The session envelope violates a structural or provenance invariant.
    #[error("invalid recorded/shadow session: {0}")]
    Invalid(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ai::{diff_drive_goal_task_spec, DiffDriveRewardConfig};

    fn session(disconnect: Option<u64>) -> (TaskSpec, RecordedShadowSession) {
        let task = diff_drive_goal_task_spec(180, DiffDriveRewardConfig::default());
        let tensor_units = task
            .observation
            .tensors
            .iter()
            .map(|tensor| RecordedTensorUnit {
                tensor_name: tensor.name.clone(),
                unit: tensor.unit.clone(),
            })
            .collect();
        let tolerances = task
            .observation
            .tensors
            .iter()
            .map(|tensor| ShadowTensorTolerance {
                tensor_name: tensor.name.clone(),
                absolute_tolerance: if matches!(
                    tensor.dtype,
                    rne_ai::TensorDType::F32 | rne_ai::TensorDType::F64
                ) {
                    0.01
                } else {
                    0.0
                },
            })
            .collect();
        let frame = |sequence, capture, value: f64| {
            let mut values = vec![0.0; 9];
            values[0] = value;
            RecordedShadowFrame {
                observation_sequence: sequence,
                dropped_sequences_before: 0,
                captured_at_ticks: capture,
                available_at_ticks: capture + 1_000_000,
                simulation_step: sequence,
                simulation_time_ticks: capture,
                recorded_values: values.clone(),
                simulation_values: values,
                action_sequence: sequence,
                action_submitted_at_ticks: capture + 1_000_000,
                action_values: vec![0.0; 2],
            }
        };
        let session = RecordedShadowSession {
            kind: RECORDED_SHADOW_SESSION_KIND.to_string(),
            schema_version: RECORDED_SHADOW_SCHEMA_VERSION,
            experiment_id: "rne.recorded.test.v1".to_string(),
            requirements_sha256: "e".repeat(64),
            task_id: task.task_id.clone(),
            task_sha256: "a".repeat(64),
            controller_id: "rne.controller.test.v1".to_string(),
            controller_sha256: "b".repeat(64),
            sources: vec![RecordedArtifactBinding {
                role: "recorded_trace".to_string(),
                kind: "test_trace".to_string(),
                file_name: "recorded.json".to_string(),
                sha256: "f".repeat(64),
            }],
            bootstrap_action_count: 0,
            stream: RecordedStreamContract {
                clock_source: "device_monotonic".to_string(),
                tick_period_ns: 1,
                nominal_latency_ticks: 1_000_000,
                maximum_latency_ticks: 2_000_000,
                drop_policy: "explicit_sequence_gap_v1".to_string(),
                sample_capacity: 2,
                tensor_units,
                calibrations: vec![CalibrationBinding {
                    role: "base_state".to_string(),
                    kind: "identity_si_v1".to_string(),
                    sha256: "c".repeat(64),
                }],
            },
            tolerances,
            frames: vec![frame(1, 1_000_000, 0.0), frame(2, 3_000_000, 0.001)],
            disconnect_after_observation_sequence: disconnect,
        };
        (task, session)
    }

    #[test]
    fn playback_suppresses_every_action_and_retains_provenance() {
        let (task, session) = session(None);
        let report =
            evaluate_recorded_shadow_session(task, session, "d".repeat(64), HardwareMode::Playback)
                .unwrap();
        assert_eq!(report.summary.status, "passed");
        assert_eq!(report.summary.accepted_samples, 2);
        assert_eq!(report.summary.suppressed_actions, 2);
        assert!(!report.summary.actuator_writes_emitted);
        assert_eq!(
            report.gateway.final_snapshot.connection_state,
            crate::GatewayConnectionState::Disconnected
        );
    }

    #[test]
    fn injected_shadow_disconnect_is_bounded_and_non_actuating() {
        let (task, mut session) = session(Some(1));
        session.frames.truncate(1);
        session.stream.sample_capacity = 1;
        let report =
            evaluate_recorded_shadow_session(task, session, "d".repeat(64), HardwareMode::Shadow)
                .unwrap();
        assert_eq!(report.summary.status, "failed_as_expected");
        assert!(report.summary.transport_failure_observed);
        assert!(!report.summary.actuator_writes_emitted);
    }

    #[test]
    fn excessive_recorded_latency_is_rejected_before_gateway_execution() {
        let (task, mut session) = session(None);
        session.frames[0].available_at_ticks += 2_000_000;
        assert!(matches!(
            evaluate_recorded_shadow_session(task, session, "d".repeat(64), HardwareMode::Playback),
            Err(RecordedShadowError::Invalid(
                "frame violates latency or action timing contract"
            ))
        ));
    }
}
