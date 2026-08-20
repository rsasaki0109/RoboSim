//! Profile-bound host runner and complete LeKiwi session evidence.
//!
//! The runner owns protocol ordering and gateway decisions, but not a clock or
//! transport implementation. Hosts inject monotonic ticks and a bounded
//! request/response transport, keeping wall-clock and process concerns outside
//! the reusable state machine.

use crate::{
    lekiwi_reference_profile_v1, LeKiwiProfileError, LeKiwiReferenceProfile,
    LEKIWI_DEVICE_BRIDGE_SCHEMA_VERSION, LEKIWI_MOCK_DEVICE_ID, LEKIWI_PHYSICAL_DEVICE_ID_PREFIX,
};
use rne_hardware_gateway::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareSessionEvidence, HardwareSessionEvidenceError,
    HardwareWireTraceEntry, HardwareWireTraceError, HardwareWireTraceOutcome,
    HardwareWireTraceRecorder, HostWireFrame, HostWirePayload,
};
use rne_hardware_gateway::{
    CommandDisposition, GatewayBuildError, GatewayConfig, GatewayConnectionState, GatewayError,
    HardwareGateway, HardwareMode, SafetyReason,
};
use serde::{Deserialize, Serialize};
use std::fmt;

/// Schema version for a complete profile-bound LeKiwi session artifact.
pub const LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`LeKiwiReferenceSessionEvidence`].
pub const LEKIWI_REFERENCE_SESSION_KIND: &str = "rne_lekiwi_reference_session";

/// Hard bound on observations retained by one runner invocation.
pub const MAX_LEKIWI_REFERENCE_SESSION_SAMPLES: usize = 100_000;

/// Source of monotonic host milliseconds used by the hardware gateway.
///
/// Implementations may read a host monotonic clock. Deterministic tests can
/// instead supply explicit ticks. Values must never decrease.
pub trait LeKiwiMonotonicClock {
    /// Returns the current monotonic host tick in milliseconds.
    fn now_ms(&mut self) -> u64;
}

/// One bounded request/response transport to a LeKiwi device bridge.
///
/// Implementations must place their own finite I/O timeout around every
/// exchange. The physical bridge has an independent watchdog, but that does
/// not make an unbounded host read acceptable.
pub trait LeKiwiWireTransport {
    /// Sends one request and returns its single correlated device response.
    fn exchange(
        &mut self,
        request: &HostWireFrame,
    ) -> Result<DeviceWireFrame, LeKiwiTransportError>;
}

/// Transport-neutral host I/O failure.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{message}")]
pub struct LeKiwiTransportError {
    message: String,
}

impl LeKiwiTransportError {
    /// Creates a transport failure without exposing vendor-specific types.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

/// Bounded construction parameters for one reference session.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LeKiwiReferenceSessionConfig {
    /// Stable identity copied into every wire frame and evidence artifact.
    pub session_id: String,
    /// Shadow, HIL, or live authority requested from the device bridge.
    pub mode: HardwareMode,
    /// Maximum observation/action cycles allowed before close.
    pub sample_capacity: usize,
}

impl LeKiwiReferenceSessionConfig {
    /// Creates a profile-bound session configuration.
    pub fn new(session_id: impl Into<String>, mode: HardwareMode, sample_capacity: usize) -> Self {
        Self {
            session_id: session_id.into(),
            mode,
            sample_capacity,
        }
    }
}

/// One accepted observation passed to a host controller.
#[derive(Clone, Debug, PartialEq)]
pub struct LeKiwiReferenceObservation {
    /// Connection-local observation sequence emitted by the device.
    pub sequence: u64,
    /// TaskSpec-ordered normalized observation values.
    pub values: Vec<f64>,
}

/// One accepted device observation and its gateway command decision.
#[derive(Clone, Debug, PartialEq)]
pub struct LeKiwiReferenceSample {
    /// Connection-local observation sequence emitted by the device.
    pub observation_sequence: u64,
    /// TaskSpec-ordered normalized observation values.
    pub observation_values: Vec<f64>,
    /// Whether the action was queued or deliberately suppressed.
    pub command_disposition: CommandDisposition,
}

/// Result of one poll/action cycle.
#[derive(Clone, Debug, PartialEq)]
pub enum LeKiwiReferenceSampleOutcome {
    /// The session remains open after one accepted sample.
    Sample(LeKiwiReferenceSample),
    /// A device or gateway safety condition ended the session with evidence.
    Terminal(Box<LeKiwiReferenceSessionEvidence>),
}

/// Self-contained reference profile plus correlated session evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiReferenceSessionEvidence {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Device bridge contract version used for this exchange.
    pub device_bridge_schema_version: u32,
    /// Device identity returned by the bridge ready handshake.
    ///
    /// The companion bridge uses distinct mock and physical prefixes so mock
    /// evidence cannot be presented as a physical-device run.
    pub device_id: String,
    /// Exact reference selection, upstream pin, units, and safety limits.
    pub profile: LeKiwiReferenceProfile,
    /// Exact wire exchange and correlated gateway decisions.
    pub session: HardwareSessionEvidence,
}

impl LeKiwiReferenceSessionEvidence {
    /// Validates the outer schema, exact profile, open dimensions, and all
    /// nested wire/gateway terminal invariants.
    pub fn validate(&self) -> Result<(), LeKiwiReferenceSessionEvidenceError> {
        if self.kind != LEKIWI_REFERENCE_SESSION_KIND {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidKind {
                actual: self.kind.clone(),
            });
        }
        if self.schema_version != LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION {
            return Err(
                LeKiwiReferenceSessionEvidenceError::UnsupportedSchemaVersion {
                    actual: self.schema_version,
                },
            );
        }
        if self.device_bridge_schema_version != LEKIWI_DEVICE_BRIDGE_SCHEMA_VERSION {
            return Err(
                LeKiwiReferenceSessionEvidenceError::UnsupportedBridgeSchemaVersion {
                    actual: self.device_bridge_schema_version,
                },
            );
        }
        self.profile.validate()?;
        if !self.profile.supported_modes.contains(&self.session.mode) {
            return Err(LeKiwiReferenceSessionEvidenceError::UnsupportedMode(
                self.session.mode,
            ));
        }
        self.session.validate_against(&self.profile.task)?;
        let Some(HardwareWireTraceEntry::Host { frame }) = self.session.wire_trace.entries.first()
        else {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidOpenContract);
        };
        let HostWirePayload::Open {
            task_id,
            mode,
            observation_width,
            action_width,
        } = &frame.payload
        else {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidOpenContract);
        };
        if task_id != &self.profile.task.task_id
            || *mode != self.session.mode
            || *observation_width != self.profile.observation_bindings.len()
            || *action_width != self.profile.action_bindings.len()
        {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidOpenContract);
        }
        let Some(HardwareWireTraceEntry::Device { frame }) = self.session.wire_trace.entries.get(1)
        else {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidOpenContract);
        };
        let DeviceWirePayload::Ready { device_id, .. } = &frame.payload else {
            return Err(LeKiwiReferenceSessionEvidenceError::InvalidOpenContract);
        };
        if device_id != &self.device_id {
            return Err(LeKiwiReferenceSessionEvidenceError::DeviceIdentityMismatch);
        }
        validate_device_identity(device_id)?;
        Ok(())
    }
}

/// Failure validating a complete reference-session artifact.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LeKiwiReferenceSessionEvidenceError {
    /// The artifact discriminator is not the v1 reference-session kind.
    #[error("invalid LeKiwi reference-session kind {actual:?}")]
    InvalidKind {
        /// Rejected discriminator.
        actual: String,
    },
    /// The outer evidence schema is unsupported.
    #[error("unsupported LeKiwi reference-session schema {actual}")]
    UnsupportedSchemaVersion {
        /// Rejected schema version.
        actual: u32,
    },
    /// The declared Python bridge contract is unsupported.
    #[error("unsupported LeKiwi device-bridge schema {actual}")]
    UnsupportedBridgeSchemaVersion {
        /// Rejected bridge schema version.
        actual: u32,
    },
    /// The exact embedded reference profile is invalid.
    #[error(transparent)]
    Profile(#[from] LeKiwiProfileError),
    /// The session requested authority outside the selected profile.
    #[error("LeKiwi reference profile does not support mode {0:?}")]
    UnsupportedMode(HardwareMode),
    /// The nested session evidence failed replay or terminal validation.
    #[error(transparent)]
    Session(#[from] HardwareSessionEvidenceError),
    /// Serialized nested evidence fields differ from their reconstructed form.
    #[error("invalid LeKiwi nested session evidence envelope")]
    InvalidSessionEnvelope,
    /// The open frame does not match the exact profile task and dimensions.
    #[error("LeKiwi wire open does not match the reference profile")]
    InvalidOpenContract,
    /// The promoted device identity differs from the ready response.
    #[error("LeKiwi reference-session device identity does not match ready handshake")]
    DeviceIdentityMismatch,
    /// The Ready identity is neither the exact mock nor a named physical unit.
    #[error("unsupported LeKiwi device identity {actual:?}")]
    UnsupportedDeviceIdentity {
        /// Rejected Ready identity.
        actual: String,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RunnerState {
    Created,
    Open,
    Finished,
    Failed,
}

/// Stateful host runner for the exact LeKiwi + SO-101 v1 profile.
#[derive(Debug)]
pub struct LeKiwiReferenceSessionRunner<T, C> {
    transport: T,
    clock: C,
    profile: LeKiwiReferenceProfile,
    gateway: Option<HardwareGateway>,
    recorder: Option<HardwareWireTraceRecorder>,
    session_id: String,
    mode: HardwareMode,
    device_id: Option<String>,
    sample_capacity: usize,
    samples: usize,
    next_request_sequence: u64,
    next_action_sequence: u64,
    state: RunnerState,
}

impl<T, C> LeKiwiReferenceSessionRunner<T, C>
where
    T: LeKiwiWireTransport,
    C: LeKiwiMonotonicClock,
{
    /// Builds a bounded runner without opening device authority.
    pub fn new(
        transport: T,
        clock: C,
        config: LeKiwiReferenceSessionConfig,
    ) -> Result<Self, LeKiwiReferenceSessionError> {
        let profile = lekiwi_reference_profile_v1();
        profile.validate()?;
        if !profile.supported_modes.contains(&config.mode) {
            return Err(LeKiwiReferenceSessionError::UnsupportedMode(config.mode));
        }
        if config.sample_capacity == 0
            || config.sample_capacity > MAX_LEKIWI_REFERENCE_SESSION_SAMPLES
        {
            return Err(LeKiwiReferenceSessionError::InvalidSampleCapacity {
                actual: config.sample_capacity,
                maximum: MAX_LEKIWI_REFERENCE_SESSION_SAMPLES,
            });
        }
        let trace_capacity = trace_capacity(config.mode, config.sample_capacity)?;
        let event_capacity = config
            .sample_capacity
            .checked_mul(4)
            .and_then(|count| count.checked_add(16))
            .ok_or(LeKiwiReferenceSessionError::CapacityOverflow)?;
        let gateway_config = GatewayConfig {
            mode: config.mode,
            max_observation_age_ms: profile.safety.max_observation_age_ms,
            command_deadline_ms: profile.safety.command_deadline_ms,
            max_command_age_ms: profile.safety.max_command_age_ms,
            observation_capacity: config.sample_capacity.min(4),
            actuation_capacity: 2,
            event_capacity,
        };
        let gateway = HardwareGateway::new(profile.task.clone(), gateway_config)?;
        let recorder = HardwareWireTraceRecorder::new(
            config.session_id.clone(),
            profile.task.task_id.clone(),
            trace_capacity,
        )?;
        Ok(Self {
            transport,
            clock,
            profile,
            gateway: Some(gateway),
            recorder: Some(recorder),
            session_id: config.session_id,
            mode: config.mode,
            device_id: None,
            sample_capacity: config.sample_capacity,
            samples: 0,
            next_request_sequence: 1,
            next_action_sequence: 1,
            state: RunnerState::Created,
        })
    }

    /// Opens the bridge and connects the gateway after an exact ready response.
    pub fn open(&mut self) -> Result<(), LeKiwiReferenceSessionError> {
        if self.state != RunnerState::Created {
            return Err(LeKiwiReferenceSessionError::InvalidState {
                expected: "created",
                actual: self.state_name(),
            });
        }
        let response = self.exchange(HostWirePayload::Open {
            task_id: self.profile.task.task_id.clone(),
            mode: self.mode,
            observation_width: self.profile.observation_bindings.len(),
            action_width: self.profile.action_bindings.len(),
        })?;
        let DeviceWirePayload::Ready {
            device_id,
            task_id,
            observation_width,
            action_width,
        } = &response.payload
        else {
            return self.fail_unexpected("ready", &response.payload);
        };
        if task_id != &self.profile.task.task_id
            || *observation_width != self.profile.observation_bindings.len()
            || *action_width != self.profile.action_bindings.len()
        {
            return self.fail_unexpected("matching_ready", &response.payload);
        }
        if let Err(error) = validate_device_identity(device_id) {
            self.state = RunnerState::Failed;
            return Err(error.into());
        }
        self.device_id = Some(device_id.clone());
        let now_ms = self.clock.now_ms();
        self.gateway_mut()?.connect(now_ms)?;
        self.state = RunnerState::Open;
        Ok(())
    }

    /// Polls one observation and submits a precomputed TaskSpec-ordered action.
    ///
    /// Use [`Self::sample_with_controller`] when the action depends on the
    /// current observation or controller compute time must count toward the
    /// command deadline.
    pub fn sample(
        &mut self,
        action_values: Vec<f64>,
    ) -> Result<LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionError> {
        self.sample_with_controller(|_| action_values)
    }

    /// Polls one observation, passes it to a host controller, and delivers the
    /// returned action only when the selected mode grants authority.
    ///
    /// The injected clock is read after the controller returns, so controller
    /// computation contributes to the observation-to-command deadline. An
    /// invalid controller result in HIL/live trips a typed fail-closed stop.
    pub fn sample_with_controller<F>(
        &mut self,
        controller: F,
    ) -> Result<LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionError>
    where
        F: FnOnce(&LeKiwiReferenceObservation) -> Vec<f64>,
    {
        self.require_open()?;
        if let Some(evidence) = self.check_gateway_safety()? {
            return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
        }
        if self.samples == self.sample_capacity {
            return Err(LeKiwiReferenceSessionError::SampleCapacityExceeded {
                capacity: self.sample_capacity,
            });
        }
        let response = self.exchange(HostWirePayload::PollObservation)?;
        if let Some(evidence) = self.finish_from_terminal_payload(&response.payload)? {
            return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
        }
        let DeviceWirePayload::Observation { sequence, values } = response.payload else {
            return self.fail_unexpected("observation", &response.payload);
        };
        let now_ms = self.clock.now_ms();
        self.gateway_mut()?
            .ingest_observation(now_ms, sequence, values.clone())?;

        if can_actuate(self.mode)
            && self.gateway_ref()?.connection_state() == GatewayConnectionState::Connected
        {
            let now_ms = self.clock.now_ms();
            self.gateway_mut()?.arm(now_ms)?;
        }

        let observation = LeKiwiReferenceObservation {
            sequence,
            values: values.clone(),
        };
        let action_values = controller(&observation);
        if action_values.len() != self.profile.action_bindings.len() {
            if can_actuate(self.mode) {
                let evidence = self.finish_controller_fault()?;
                return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
            }
            return Err(LeKiwiReferenceSessionError::ActionWidth {
                expected: self.profile.action_bindings.len(),
                actual: action_values.len(),
            });
        }

        let action_sequence = self.next_action_sequence;
        self.next_action_sequence = self
            .next_action_sequence
            .checked_add(1)
            .ok_or(LeKiwiReferenceSessionError::SequenceOverflow)?;
        let now_ms = self.clock.now_ms();
        let disposition = match self.gateway_mut()?.submit_action(
            now_ms,
            action_sequence,
            sequence,
            action_values,
        ) {
            Ok(disposition) => disposition,
            Err(error) => {
                if can_actuate(self.mode) {
                    if let Some(reason) = self.gateway_ref()?.snapshot().safety_latch {
                        let evidence = self.finish_gateway_safety(reason)?;
                        return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
                    }
                    let evidence = self.finish_controller_fault()?;
                    return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
                }
                return Err(error.into());
            }
        };

        if disposition == CommandDisposition::Queued {
            let now_ms = self.clock.now_ms();
            let frame = self
                .gateway_mut()?
                .poll_actuation(now_ms)?
                .ok_or(LeKiwiReferenceSessionError::MissingActuation)?;
            if frame.safety_stop {
                let reason = frame
                    .reason
                    .ok_or(LeKiwiReferenceSessionError::MissingSafetyReason)?;
                if let Some(evidence) = self.exchange_actuation(frame)? {
                    return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
                }
                let evidence =
                    self.finalize(HardwareWireTraceOutcome::GatewaySafetyStopped { reason })?;
                return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
            }
            if let Some(evidence) = self.exchange_actuation(frame)? {
                return Ok(LeKiwiReferenceSampleOutcome::Terminal(Box::new(evidence)));
            }
        }

        self.samples += 1;
        Ok(LeKiwiReferenceSampleOutcome::Sample(
            LeKiwiReferenceSample {
                observation_sequence: sequence,
                observation_values: observation.values,
                command_disposition: disposition,
            },
        ))
    }

    /// Relinquishes authority, confirms the device-side zero stop, completes
    /// the close exchange, and returns validated session evidence.
    pub fn close(&mut self) -> Result<LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionError> {
        self.require_open()?;
        if let Some(evidence) = self.check_gateway_safety()? {
            return Ok(evidence);
        }
        if can_actuate(self.mode) {
            let now_ms = self.clock.now_ms();
            self.gateway_mut()?.disarm(now_ms)?;
            let frame = self
                .gateway_mut()?
                .poll_actuation(now_ms)?
                .ok_or(LeKiwiReferenceSessionError::MissingActuation)?;
            if let Some(evidence) = self.exchange_actuation(frame)? {
                return Ok(evidence);
            }
        }
        let response = self.exchange(HostWirePayload::Close)?;
        if let Some(evidence) = self.finish_from_terminal_payload(&response.payload)? {
            return Ok(evidence);
        }
        if !matches!(response.payload, DeviceWirePayload::Closed) {
            return self.fail_unexpected("closed", &response.payload);
        }
        let now_ms = self.clock.now_ms();
        self.gateway_mut()?.close_cleanly(now_ms)?;
        self.finalize(HardwareWireTraceOutcome::Completed)
    }

    /// Asserts an operator emergency stop, confirms the device-side zero stop,
    /// and returns terminal session evidence.
    pub fn emergency_stop(
        &mut self,
    ) -> Result<LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionError> {
        self.require_open()?;
        if let Some(evidence) = self.check_gateway_safety()? {
            return Ok(evidence);
        }
        let now_ms = self.clock.now_ms();
        self.gateway_mut()?.emergency_stop(now_ms)?;
        self.finish_gateway_safety(SafetyReason::EmergencyStop)
    }

    /// Returns shared access to the injected transport.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Returns mutable access to the injected transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    fn exchange(
        &mut self,
        payload: HostWirePayload,
    ) -> Result<DeviceWireFrame, LeKiwiReferenceSessionError> {
        let sequence = self.next_request_sequence;
        self.next_request_sequence = self
            .next_request_sequence
            .checked_add(1)
            .ok_or(LeKiwiReferenceSessionError::SequenceOverflow)?;
        let request = HostWireFrame::new(self.session_id.clone(), sequence, payload);
        if let Err(error) = self.recorder_mut()?.record_host(request.clone()) {
            self.state = RunnerState::Failed;
            return Err(error.into());
        }
        let response = match self.transport.exchange(&request) {
            Ok(response) => response,
            Err(error) => {
                self.state = RunnerState::Failed;
                return Err(error.into());
            }
        };
        if let Err(error) = self.recorder_mut()?.record_device(response.clone()) {
            self.state = RunnerState::Failed;
            return Err(error.into());
        }
        Ok(response)
    }

    fn exchange_actuation(
        &mut self,
        frame: rne_hardware_gateway::ActuationFrame,
    ) -> Result<Option<LeKiwiReferenceSessionEvidence>, LeKiwiReferenceSessionError> {
        let expected_action_sequence = frame.action_sequence;
        let expected_safety_stop = frame.safety_stop;
        let response = self.exchange(HostWirePayload::Actuate { frame })?;
        if let Some(evidence) = self.finish_from_terminal_payload(&response.payload)? {
            return Ok(Some(evidence));
        }
        let accepted_matches = matches!(
            response.payload,
            DeviceWirePayload::ActuationAccepted {
                action_sequence,
                safety_stop,
            } if action_sequence == expected_action_sequence
                && safety_stop == expected_safety_stop
        );
        if !accepted_matches {
            return self.fail_unexpected("actuation_accepted", &response.payload);
        }
        Ok(None)
    }

    fn finish_gateway_safety(
        &mut self,
        reason: SafetyReason,
    ) -> Result<LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionError> {
        let now_ms = self.clock.now_ms();
        let frame = self
            .gateway_mut()?
            .poll_actuation(now_ms)?
            .ok_or(LeKiwiReferenceSessionError::MissingActuation)?;
        if !frame.safety_stop || frame.reason != Some(reason) {
            self.state = RunnerState::Failed;
            return Err(LeKiwiReferenceSessionError::InvalidSafetyActuation);
        }
        if let Some(evidence) = self.exchange_actuation(frame)? {
            return Ok(evidence);
        }
        self.finalize(HardwareWireTraceOutcome::GatewaySafetyStopped { reason })
    }

    fn finish_controller_fault(
        &mut self,
    ) -> Result<LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionError> {
        let now_ms = self.clock.now_ms();
        self.gateway_mut()?.controller_fault(now_ms)?;
        self.finish_gateway_safety(SafetyReason::ControllerFault)
    }

    fn check_gateway_safety(
        &mut self,
    ) -> Result<Option<LeKiwiReferenceSessionEvidence>, LeKiwiReferenceSessionError> {
        if !can_actuate(self.mode) {
            return Ok(None);
        }
        let now_ms = self.clock.now_ms();
        let tick_error = self.gateway_mut()?.tick(now_ms).err();
        if let Some(reason) = self.gateway_ref()?.snapshot().safety_latch {
            return self.finish_gateway_safety(reason).map(Some);
        }
        if let Some(error) = tick_error {
            return Err(error.into());
        }
        Ok(None)
    }

    fn finish_from_terminal_payload(
        &mut self,
        payload: &DeviceWirePayload,
    ) -> Result<Option<LeKiwiReferenceSessionEvidence>, LeKiwiReferenceSessionError> {
        match *payload {
            DeviceWirePayload::SafetySignal {
                reason,
                safe_stop_applied: true,
            } => {
                let now_ms = self.clock.now_ms();
                self.gateway_mut()?.device_safety_signal(reason, now_ms)?;
                self.finalize(HardwareWireTraceOutcome::SafetyStopped { reason })
                    .map(Some)
            }
            DeviceWirePayload::Disconnected {
                reason,
                safe_stop_applied: true,
            } => {
                let now_ms = self.clock.now_ms();
                self.gateway_mut()?.disconnect(now_ms)?;
                self.finalize(HardwareWireTraceOutcome::Disconnected { reason })
                    .map(Some)
            }
            _ => Ok(None),
        }
    }

    fn finalize(
        &mut self,
        outcome: HardwareWireTraceOutcome,
    ) -> Result<LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionError> {
        let recorder = self
            .recorder
            .take()
            .ok_or(LeKiwiReferenceSessionError::RunnerUnavailable)?;
        let mut gateway = self
            .gateway
            .take()
            .ok_or(LeKiwiReferenceSessionError::RunnerUnavailable)?;
        self.state = RunnerState::Finished;
        let trace = recorder.finish(outcome)?;
        let session = HardwareSessionEvidence::new(trace, gateway.take_evidence())?;
        let evidence = LeKiwiReferenceSessionEvidence {
            kind: LEKIWI_REFERENCE_SESSION_KIND.to_string(),
            schema_version: LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
            device_bridge_schema_version: LEKIWI_DEVICE_BRIDGE_SCHEMA_VERSION,
            device_id: self
                .device_id
                .clone()
                .ok_or(LeKiwiReferenceSessionError::MissingDeviceIdentity)?,
            profile: self.profile.clone(),
            session,
        };
        evidence.validate()?;
        Ok(evidence)
    }

    fn require_open(&self) -> Result<(), LeKiwiReferenceSessionError> {
        if self.state == RunnerState::Open {
            Ok(())
        } else {
            Err(LeKiwiReferenceSessionError::InvalidState {
                expected: "open",
                actual: self.state_name(),
            })
        }
    }

    fn gateway_ref(&self) -> Result<&HardwareGateway, LeKiwiReferenceSessionError> {
        self.gateway
            .as_ref()
            .ok_or(LeKiwiReferenceSessionError::RunnerUnavailable)
    }

    fn gateway_mut(&mut self) -> Result<&mut HardwareGateway, LeKiwiReferenceSessionError> {
        self.gateway
            .as_mut()
            .ok_or(LeKiwiReferenceSessionError::RunnerUnavailable)
    }

    fn recorder_mut(
        &mut self,
    ) -> Result<&mut HardwareWireTraceRecorder, LeKiwiReferenceSessionError> {
        self.recorder
            .as_mut()
            .ok_or(LeKiwiReferenceSessionError::RunnerUnavailable)
    }

    fn fail_unexpected<R>(
        &mut self,
        expected: &'static str,
        payload: &DeviceWirePayload,
    ) -> Result<R, LeKiwiReferenceSessionError> {
        self.state = RunnerState::Failed;
        Err(LeKiwiReferenceSessionError::UnexpectedResponse {
            expected,
            actual: payload_name(payload),
        })
    }

    fn state_name(&self) -> &'static str {
        match self.state {
            RunnerState::Created => "created",
            RunnerState::Open => "open",
            RunnerState::Finished => "finished",
            RunnerState::Failed => "failed",
        }
    }
}

/// Failure building, running, or finalizing a reference session.
#[derive(Debug, thiserror::Error)]
pub enum LeKiwiReferenceSessionError {
    /// The exact built-in profile did not validate.
    #[error(transparent)]
    Profile(#[from] LeKiwiProfileError),
    /// Playback or another authority mode is outside the physical profile.
    #[error("LeKiwi reference runner does not support mode {0:?}")]
    UnsupportedMode(HardwareMode),
    /// The requested sample bound is zero or above the hard ceiling.
    #[error("LeKiwi sample capacity {actual} must be in 1..={maximum}")]
    InvalidSampleCapacity {
        /// Requested number of samples.
        actual: usize,
        /// Hard runner ceiling.
        maximum: usize,
    },
    /// A derived trace or event capacity overflowed the host index type.
    #[error("LeKiwi reference-session capacity overflow")]
    CapacityOverflow,
    /// The caller attempted to exceed the declared sample bound.
    #[error("LeKiwi reference-session sample capacity {capacity} exceeded")]
    SampleCapacityExceeded {
        /// Declared session bound.
        capacity: usize,
    },
    /// A supplied action did not have the reference profile width.
    #[error("LeKiwi action width must be {expected}, got {actual}")]
    ActionWidth {
        /// Required flattened action width.
        expected: usize,
        /// Supplied flattened action width.
        actual: usize,
    },
    /// A request or action sequence could not be incremented.
    #[error("LeKiwi reference-session sequence overflow")]
    SequenceOverflow,
    /// An operation was attempted in the wrong lifecycle state.
    #[error("LeKiwi runner state must be {expected}, got {actual}")]
    InvalidState {
        /// Required lifecycle state.
        expected: &'static str,
        /// Actual lifecycle state.
        actual: &'static str,
    },
    /// A complete runner component had already been consumed.
    #[error("LeKiwi reference runner is no longer available")]
    RunnerUnavailable,
    /// The device returned a valid but contextually unexpected response.
    #[error("expected LeKiwi {expected} response, got {actual}")]
    UnexpectedResponse {
        /// Expected response payload.
        expected: &'static str,
        /// Actual response payload.
        actual: &'static str,
    },
    /// A queued gateway action disappeared before transport delivery.
    #[error("LeKiwi gateway did not yield its queued actuation")]
    MissingActuation,
    /// A gateway safety frame omitted its typed stop reason.
    #[error("LeKiwi gateway safety frame omitted its reason")]
    MissingSafetyReason,
    /// A gateway safety frame differed from the latched condition.
    #[error("LeKiwi gateway produced an inconsistent safety actuation")]
    InvalidSafetyActuation,
    /// A successful ready handshake did not yield a device identity.
    #[error("LeKiwi reference session is missing its device identity")]
    MissingDeviceIdentity,
    /// The TaskSpec-bound gateway could not be constructed.
    #[error(transparent)]
    GatewayBuild(#[from] GatewayBuildError),
    /// A gateway transition rejected the operation.
    #[error(transparent)]
    Gateway(#[from] GatewayError),
    /// The bounded wire recorder rejected a frame or terminal outcome.
    #[error(transparent)]
    Trace(#[from] HardwareWireTraceError),
    /// Transport I/O failed or timed out.
    #[error(transparent)]
    Transport(#[from] LeKiwiTransportError),
    /// Gateway and wire terminal evidence did not correlate.
    #[error(transparent)]
    SessionEvidence(#[from] HardwareSessionEvidenceError),
    /// The completed outer reference artifact did not validate.
    #[error(transparent)]
    ReferenceEvidence(#[from] LeKiwiReferenceSessionEvidenceError),
}

fn trace_capacity(
    mode: HardwareMode,
    sample_capacity: usize,
) -> Result<usize, LeKiwiReferenceSessionError> {
    let entries_per_sample = if can_actuate(mode) { 4 } else { 2 };
    let terminal_entries = if can_actuate(mode) { 6 } else { 4 };
    sample_capacity
        .checked_mul(entries_per_sample)
        .and_then(|entries| entries.checked_add(terminal_entries))
        .ok_or(LeKiwiReferenceSessionError::CapacityOverflow)
}

fn can_actuate(mode: HardwareMode) -> bool {
    matches!(mode, HardwareMode::Hil | HardwareMode::Live)
}

fn payload_name(payload: &DeviceWirePayload) -> &'static str {
    match payload {
        DeviceWirePayload::Ready { .. } => "ready",
        DeviceWirePayload::Observation { .. } => "observation",
        DeviceWirePayload::ActuationAccepted { .. } => "actuation_accepted",
        DeviceWirePayload::SafetySignal { .. } => "safety_signal",
        DeviceWirePayload::Disconnected { .. } => "disconnected",
        DeviceWirePayload::Closed => "closed",
        DeviceWirePayload::Rejected { .. } => "rejected",
    }
}

fn validate_device_identity(device_id: &str) -> Result<(), LeKiwiReferenceSessionEvidenceError> {
    let physical_id = device_id.strip_prefix(LEKIWI_PHYSICAL_DEVICE_ID_PREFIX);
    if device_id == LEKIWI_MOCK_DEVICE_ID
        || physical_id.is_some_and(|identity| !identity.trim().is_empty())
    {
        Ok(())
    } else {
        Err(
            LeKiwiReferenceSessionEvidenceError::UnsupportedDeviceIdentity {
                actual: device_id.to_string(),
            },
        )
    }
}

impl fmt::Display for LeKiwiReferenceSessionConfig {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{} {:?} ({} samples)",
            self.session_id, self.mode, self.sample_capacity
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_hardware_gateway::wire::{
        HardwareWireTraceOutcome, WireDisconnectReason, WireRejectionCode,
    };
    use rne_hardware_gateway::GatewayConnectionState;

    #[derive(Debug, Default)]
    struct StepClock {
        next_ms: u64,
    }

    impl LeKiwiMonotonicClock for StepClock {
        fn now_ms(&mut self) -> u64 {
            let now_ms = self.next_ms;
            self.next_ms += 1;
            now_ms
        }
    }

    #[derive(Debug, Default)]
    struct MockTransport {
        next_observation_sequence: u64,
        terminal_poll: Option<DeviceWirePayload>,
    }

    impl MockTransport {
        fn with_terminal_poll(payload: DeviceWirePayload) -> Self {
            Self {
                next_observation_sequence: 0,
                terminal_poll: Some(payload),
            }
        }
    }

    impl LeKiwiWireTransport for MockTransport {
        fn exchange(
            &mut self,
            request: &HostWireFrame,
        ) -> Result<DeviceWireFrame, LeKiwiTransportError> {
            let payload = match &request.payload {
                HostWirePayload::Open {
                    task_id,
                    observation_width,
                    action_width,
                    ..
                } => DeviceWirePayload::Ready {
                    device_id: LEKIWI_MOCK_DEVICE_ID.to_string(),
                    task_id: task_id.clone(),
                    observation_width: *observation_width,
                    action_width: *action_width,
                },
                HostWirePayload::PollObservation => {
                    if let Some(payload) = self.terminal_poll.take() {
                        payload
                    } else {
                        self.next_observation_sequence += 1;
                        DeviceWirePayload::Observation {
                            sequence: self.next_observation_sequence,
                            values: vec![0.0; 9],
                        }
                    }
                }
                HostWirePayload::Actuate { frame } => DeviceWirePayload::ActuationAccepted {
                    action_sequence: frame.action_sequence,
                    safety_stop: frame.safety_stop,
                },
                HostWirePayload::Close => DeviceWirePayload::Closed,
            };
            Ok(DeviceWireFrame::new(
                request.session_id.clone(),
                request.sequence,
                payload,
            ))
        }
    }

    fn runner(
        mode: HardwareMode,
        sample_capacity: usize,
    ) -> LeKiwiReferenceSessionRunner<MockTransport, StepClock> {
        LeKiwiReferenceSessionRunner::new(
            MockTransport::default(),
            StepClock::default(),
            LeKiwiReferenceSessionConfig::new("rne.lekiwi.test.session", mode, sample_capacity),
        )
        .unwrap()
    }

    #[test]
    fn shadow_session_completes_without_an_actuation_frame() {
        let mut runner = runner(HardwareMode::Shadow, 2);
        runner.open().unwrap();
        for _ in 0..2 {
            let LeKiwiReferenceSampleOutcome::Sample(sample) = runner.sample(vec![0.0; 3]).unwrap()
            else {
                panic!("shadow session ended early");
            };
            assert_eq!(sample.command_disposition, CommandDisposition::Suppressed);
        }
        let evidence = runner.close().unwrap();
        evidence.validate().unwrap();
        assert_eq!(
            evidence.session.wire_trace.outcome,
            HardwareWireTraceOutcome::Completed
        );
        assert_eq!(evidence.session.wire_trace.entries.len(), 8);
        assert_eq!(
            evidence.session.gateway.final_snapshot.connection_state,
            GatewayConnectionState::Disconnected
        );
        assert_eq!(evidence.session.gateway.final_snapshot.safety_latch, None);
        assert!(!evidence.session.wire_trace.entries.iter().any(|entry| {
            matches!(
                entry,
                HardwareWireTraceEntry::Host {
                    frame: HostWireFrame {
                        payload: HostWirePayload::Actuate { .. },
                        ..
                    }
                }
            )
        }));
    }

    #[test]
    fn live_session_stops_before_clean_close() {
        let mut runner = runner(HardwareMode::Live, 2);
        runner.open().unwrap();
        for action in [vec![0.02, 0.0, 0.0], vec![0.0, -0.02, 0.1]] {
            assert!(matches!(
                runner.sample(action).unwrap(),
                LeKiwiReferenceSampleOutcome::Sample(_)
            ));
        }
        let evidence = runner.close().unwrap();
        evidence.validate().unwrap();
        assert_eq!(evidence.session.wire_trace.entries.len(), 14);
        assert_eq!(
            evidence.session.gateway.final_snapshot.connection_state,
            GatewayConnectionState::Disconnected
        );
        assert_eq!(evidence.session.gateway.final_snapshot.safety_latch, None);
        assert!(evidence.session.gateway.events.iter().any(|event| matches!(
            event,
            rne_hardware_gateway::GatewayEvent::ActuationDelivered {
                action_sequence: None,
                safety_stop: true,
            }
        )));
    }

    #[test]
    fn limit_trip_delivers_gateway_stop_and_returns_terminal_evidence() {
        let mut runner = runner(HardwareMode::Live, 1);
        runner.open().unwrap();
        let LeKiwiReferenceSampleOutcome::Terminal(evidence) =
            runner.sample(vec![0.100_001, 0.0, 0.0]).unwrap()
        else {
            panic!("over-limit command did not terminate");
        };
        evidence.validate().unwrap();
        assert_eq!(
            evidence.session.wire_trace.outcome,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::ActuatorLimit
            }
        );
        assert_eq!(
            evidence.session.gateway.final_snapshot.safety_latch,
            Some(SafetyReason::ActuatorLimit)
        );
        assert_eq!(evidence.session.gateway.final_snapshot.queued_actuations, 0);
    }

    #[test]
    fn invalid_controller_output_and_emergency_stop_are_typed_terminals() {
        let mut controller_fault = runner(HardwareMode::Live, 1);
        controller_fault.open().unwrap();
        let LeKiwiReferenceSampleOutcome::Terminal(controller_evidence) = controller_fault
            .sample_with_controller(|observation| {
                assert_eq!(observation.sequence, 1);
                assert_eq!(observation.values.len(), 9);
                vec![0.0; 2]
            })
            .unwrap()
        else {
            panic!("invalid controller output did not terminate");
        };
        assert_eq!(
            controller_evidence.session.wire_trace.outcome,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::ControllerFault
            }
        );

        let mut emergency = runner(HardwareMode::Hil, 1);
        emergency.open().unwrap();
        let emergency_evidence = emergency.emergency_stop().unwrap();
        assert_eq!(
            emergency_evidence.session.wire_trace.outcome,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::EmergencyStop
            }
        );
    }

    #[test]
    fn device_safety_and_shadow_disconnect_are_correlated() {
        let mut safety_runner = LeKiwiReferenceSessionRunner::new(
            MockTransport::with_terminal_poll(DeviceWirePayload::SafetySignal {
                reason: SafetyReason::CommandStale,
                safe_stop_applied: true,
            }),
            StepClock::default(),
            LeKiwiReferenceSessionConfig::new("rne.lekiwi.safety", HardwareMode::Live, 1),
        )
        .unwrap();
        safety_runner.open().unwrap();
        let LeKiwiReferenceSampleOutcome::Terminal(safety) =
            safety_runner.sample(vec![0.0; 3]).unwrap()
        else {
            panic!("device safety did not terminate");
        };
        assert_eq!(
            safety.session.wire_trace.outcome,
            HardwareWireTraceOutcome::SafetyStopped {
                reason: SafetyReason::CommandStale
            }
        );

        let mut disconnect_runner = LeKiwiReferenceSessionRunner::new(
            MockTransport::with_terminal_poll(DeviceWirePayload::Disconnected {
                reason: WireDisconnectReason::InjectedFault,
                safe_stop_applied: true,
            }),
            StepClock::default(),
            LeKiwiReferenceSessionConfig::new("rne.lekiwi.disconnect", HardwareMode::Shadow, 1),
        )
        .unwrap();
        disconnect_runner.open().unwrap();
        let LeKiwiReferenceSampleOutcome::Terminal(disconnected) =
            disconnect_runner.sample(vec![0.0; 3]).unwrap()
        else {
            panic!("device disconnect did not terminate");
        };
        disconnected.validate().unwrap();
        assert_eq!(
            disconnected.session.gateway.final_snapshot.connection_state,
            GatewayConnectionState::Disconnected
        );
        assert_eq!(
            disconnected.session.gateway.final_snapshot.safety_latch,
            None
        );
    }

    #[test]
    fn tampered_outer_contract_and_rejected_open_fail() {
        let mut runner = runner(HardwareMode::Shadow, 1);
        runner.open().unwrap();
        assert!(matches!(
            runner.sample(vec![0.0; 3]).unwrap(),
            LeKiwiReferenceSampleOutcome::Sample(_)
        ));
        let mut evidence = runner.close().unwrap();
        evidence.device_bridge_schema_version += 1;
        assert!(matches!(
            evidence.validate(),
            Err(LeKiwiReferenceSessionEvidenceError::UnsupportedBridgeSchemaVersion { .. })
        ));

        let mut rejected = LeKiwiReferenceSessionRunner::new(
            MockTransport::with_terminal_poll(DeviceWirePayload::Rejected {
                code: WireRejectionCode::UnsupportedMode,
            }),
            StepClock::default(),
            LeKiwiReferenceSessionConfig::new("rne.lekiwi.rejected", HardwareMode::Shadow, 1),
        )
        .unwrap();
        rejected.open().unwrap();
        assert!(matches!(
            rejected.sample(vec![0.0; 3]),
            Err(LeKiwiReferenceSessionError::UnexpectedResponse {
                actual: "rejected",
                ..
            })
        ));
    }
}
