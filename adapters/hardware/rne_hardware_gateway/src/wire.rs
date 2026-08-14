//! Versioned, bounded process protocol for hardware adapter processes.
//!
//! The protocol deliberately carries RNE-owned values only. Vendor SDK types,
//! sockets, clocks, and process handles stay in the transport implementation.
//! Frames use one canonical JSON value followed by `\n`; the codec rejects
//! oversized or multi-line input before deserializing it.

use crate::{
    ActuationFrame, GatewayConnectionState, GatewayEvidence, HardwareMode, SafetyReason,
    HARDWARE_GATEWAY_EVIDENCE_KIND, HARDWARE_GATEWAY_SCHEMA_VERSION,
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::io::BufRead;

/// Schema version shared by host frames, device frames, and wire traces.
pub const HARDWARE_WIRE_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for host-to-device frames.
pub const HARDWARE_HOST_FRAME_KIND: &str = "rne_hardware_host_frame";

/// Stable discriminator for device-to-host frames.
pub const HARDWARE_DEVICE_FRAME_KIND: &str = "rne_hardware_device_frame";

/// Stable discriminator for a replayable wire trace.
pub const HARDWARE_WIRE_TRACE_KIND: &str = "rne_hardware_wire_trace";

/// Stable discriminator for gateway state plus its exact process exchange.
pub const HARDWARE_SESSION_EVIDENCE_KIND: &str = "rne_hardware_session_evidence";

/// Default encoded frame bound, including the terminating newline.
pub const DEFAULT_MAX_HARDWARE_WIRE_FRAME_BYTES: usize = 64 * 1024;

/// Hard ceiling accepted when constructing a codec.
pub const MAX_HARDWARE_WIRE_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// Host command carried inside a [`HostWireFrame`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum HostWirePayload {
    /// Opens one TaskSpec-shaped device session.
    Open {
        /// Portable task identity.
        task_id: String,
        /// Authority requested by the host.
        mode: HardwareMode,
        /// Flattened observation width.
        observation_width: usize,
        /// Flattened action width.
        action_width: usize,
    },
    /// Requests the next device observation.
    PollObservation,
    /// Delivers one gateway-validated actuation frame.
    Actuate {
        /// Command or fail-closed stop produced by the gateway.
        frame: ActuationFrame,
    },
    /// Closes the current device session without granting further authority.
    Close,
}

/// One versioned host-to-device process frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HostWireFrame {
    /// Stable frame discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Caller-selected session identity.
    pub session_id: String,
    /// Strictly increasing host request sequence.
    pub sequence: u64,
    /// Typed host command.
    pub payload: HostWirePayload,
}

impl HostWireFrame {
    /// Creates a v1 host frame. The codec validates it before encoding.
    pub fn new(session_id: impl Into<String>, sequence: u64, payload: HostWirePayload) -> Self {
        Self {
            kind: HARDWARE_HOST_FRAME_KIND.to_string(),
            schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
            session_id: session_id.into(),
            sequence,
            payload,
        }
    }

    /// Validates schema, identity, dimensions, and numeric payload invariants.
    pub fn validate(&self) -> Result<(), HardwareWireError> {
        validate_envelope(
            &self.kind,
            HARDWARE_HOST_FRAME_KIND,
            self.schema_version,
            &self.session_id,
        )?;
        match &self.payload {
            HostWirePayload::Open {
                task_id,
                observation_width,
                action_width,
                ..
            } => {
                validate_identifier("task_id", task_id)?;
                validate_width("observation_width", *observation_width)?;
                validate_width("action_width", *action_width)?;
            }
            HostWirePayload::Actuate { frame } => validate_actuation(frame)?,
            HostWirePayload::PollObservation | HostWirePayload::Close => {}
        }
        Ok(())
    }
}

/// Transport-neutral reason a device process ended authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WireDisconnectReason {
    /// The remote process or physical transport closed normally.
    PeerClosed,
    /// An I/O or device transport failed.
    TransportFault,
    /// A deterministic mock intentionally injected a disconnect.
    InjectedFault,
    /// The peer detected a protocol invariant violation.
    ProtocolViolation,
}

/// Stable device rejection codes without vendor-specific error values.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WireRejectionCode {
    /// A command required an open session.
    NotOpen,
    /// A second open command targeted an active process.
    AlreadyOpen,
    /// A request sequence did not increase.
    NonMonotonicSequence,
    /// A frame used a different session identity.
    SessionMismatch,
    /// A payload width differed from the open-session contract.
    WidthMismatch,
    /// A numeric payload contained NaN or infinity.
    NonFiniteValue,
    /// The device has already entered a terminal state.
    TerminalState,
}

/// Device response carried inside a [`DeviceWireFrame`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum DeviceWirePayload {
    /// Confirms the task and flattened widths accepted by the device process.
    Ready {
        /// Stable device or mock identity.
        device_id: String,
        /// Accepted portable task identity.
        task_id: String,
        /// Accepted flattened observation width.
        observation_width: usize,
        /// Accepted flattened action width.
        action_width: usize,
    },
    /// Returns one connection-local device observation.
    Observation {
        /// Strictly increasing observation sequence.
        sequence: u64,
        /// Flattened TaskSpec-order values.
        values: Vec<f64>,
    },
    /// Confirms that the device applied an actuation or safe stop.
    ActuationAccepted {
        /// Source action sequence, absent for a gateway-generated stop.
        action_sequence: Option<u64>,
        /// True when a fail-closed stop was applied.
        safety_stop: bool,
    },
    /// Reports a device-side safety assertion and whether zero output was applied.
    SafetySignal {
        /// Typed gateway safety reason.
        reason: SafetyReason,
        /// True when the device independently applied a safe stop.
        safe_stop_applied: bool,
    },
    /// Reports that transport authority ended.
    Disconnected {
        /// Transport-neutral disconnect classification.
        reason: WireDisconnectReason,
        /// True when the device watchdog applied a safe stop.
        safe_stop_applied: bool,
    },
    /// Confirms an explicit close command.
    Closed,
    /// Rejects a request using a stable, machine-readable code.
    Rejected {
        /// First failed protocol invariant.
        code: WireRejectionCode,
    },
}

/// One versioned device-to-host process frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeviceWireFrame {
    /// Stable frame discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Session identity echoed from the request.
    pub session_id: String,
    /// Host request sequence answered by this response.
    pub request_sequence: u64,
    /// Typed device response.
    pub payload: DeviceWirePayload,
}

impl DeviceWireFrame {
    /// Creates a v1 response frame. The codec validates it before encoding.
    pub fn new(
        session_id: impl Into<String>,
        request_sequence: u64,
        payload: DeviceWirePayload,
    ) -> Self {
        Self {
            kind: HARDWARE_DEVICE_FRAME_KIND.to_string(),
            schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
            session_id: session_id.into(),
            request_sequence,
            payload,
        }
    }

    /// Validates schema, identity, dimensions, and numeric payload invariants.
    pub fn validate(&self) -> Result<(), HardwareWireError> {
        validate_envelope(
            &self.kind,
            HARDWARE_DEVICE_FRAME_KIND,
            self.schema_version,
            &self.session_id,
        )?;
        match &self.payload {
            DeviceWirePayload::Ready {
                device_id,
                task_id,
                observation_width,
                action_width,
            } => {
                validate_identifier("device_id", device_id)?;
                validate_identifier("task_id", task_id)?;
                validate_width("observation_width", *observation_width)?;
                validate_width("action_width", *action_width)?;
            }
            DeviceWirePayload::Observation { values, .. } => {
                validate_width("observation.values", values.len())?;
                validate_finite("observation", values)?;
            }
            DeviceWirePayload::ActuationAccepted {
                action_sequence,
                safety_stop,
            } if (*safety_stop && action_sequence.is_some())
                || (!*safety_stop && action_sequence.is_none()) =>
            {
                return Err(HardwareWireError::InvalidActuation);
            }
            DeviceWirePayload::SafetySignal {
                safe_stop_applied, ..
            }
            | DeviceWirePayload::Disconnected {
                safe_stop_applied, ..
            } if !safe_stop_applied => {
                return Err(HardwareWireError::UnsafeTerminalResponse);
            }
            DeviceWirePayload::ActuationAccepted { .. }
            | DeviceWirePayload::SafetySignal { .. }
            | DeviceWirePayload::Disconnected { .. }
            | DeviceWirePayload::Closed
            | DeviceWirePayload::Rejected { .. } => {}
        }
        Ok(())
    }
}

/// Maximum-size JSON Lines codec for hardware process frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HardwareWireCodec {
    max_frame_bytes: usize,
}

impl HardwareWireCodec {
    /// Creates a codec with a non-zero bound no larger than the hard ceiling.
    pub fn new(max_frame_bytes: usize) -> Result<Self, HardwareWireError> {
        if !(1..=MAX_HARDWARE_WIRE_FRAME_BYTES).contains(&max_frame_bytes) {
            return Err(HardwareWireError::InvalidFrameBound {
                requested: max_frame_bytes,
                maximum: MAX_HARDWARE_WIRE_FRAME_BYTES,
            });
        }
        Ok(Self { max_frame_bytes })
    }

    /// Returns the encoded frame bound, including its newline.
    pub fn max_frame_bytes(self) -> usize {
        self.max_frame_bytes
    }

    /// Encodes one validated host frame as a bounded JSON line.
    pub fn encode_host(&self, frame: &HostWireFrame) -> Result<Vec<u8>, HardwareWireError> {
        frame.validate()?;
        self.encode(frame)
    }

    /// Decodes and validates one bounded host JSON line.
    pub fn decode_host(&self, line: &[u8]) -> Result<HostWireFrame, HardwareWireError> {
        let frame: HostWireFrame = self.decode(line)?;
        frame.validate()?;
        Ok(frame)
    }

    /// Encodes one validated device frame as a bounded JSON line.
    pub fn encode_device(&self, frame: &DeviceWireFrame) -> Result<Vec<u8>, HardwareWireError> {
        frame.validate()?;
        self.encode(frame)
    }

    /// Decodes and validates one bounded device JSON line.
    pub fn decode_device(&self, line: &[u8]) -> Result<DeviceWireFrame, HardwareWireError> {
        let frame: DeviceWireFrame = self.decode(line)?;
        frame.validate()?;
        Ok(frame)
    }

    /// Reads at most one bounded line without allowing unbounded allocation.
    ///
    /// Returns `Ok(None)` only for EOF before any bytes were read. EOF in the
    /// middle of a frame is rejected.
    pub fn read_line<R: BufRead>(
        &self,
        reader: &mut R,
    ) -> Result<Option<Vec<u8>>, HardwareWireError> {
        let mut line = Vec::new();
        loop {
            let available = reader.fill_buf()?;
            if available.is_empty() {
                return if line.is_empty() {
                    Ok(None)
                } else {
                    Err(HardwareWireError::MissingNewline)
                };
            }
            let consumed = available
                .iter()
                .position(|byte| *byte == b'\n')
                .map_or(available.len(), |index| index + 1);
            let attempted = line.len().saturating_add(consumed);
            if attempted > self.max_frame_bytes {
                return Err(HardwareWireError::FrameTooLarge {
                    actual: attempted,
                    maximum: self.max_frame_bytes,
                });
            }
            line.extend_from_slice(&available[..consumed]);
            reader.consume(consumed);
            if line.last() == Some(&b'\n') {
                return Ok(Some(line));
            }
        }
    }

    fn encode<T: Serialize>(&self, frame: &T) -> Result<Vec<u8>, HardwareWireError> {
        let mut bytes = serde_json::to_vec(frame)
            .map_err(|error| HardwareWireError::Json(error.to_string()))?;
        bytes.push(b'\n');
        if bytes.len() > self.max_frame_bytes {
            return Err(HardwareWireError::FrameTooLarge {
                actual: bytes.len(),
                maximum: self.max_frame_bytes,
            });
        }
        Ok(bytes)
    }

    fn decode<T: DeserializeOwned>(&self, line: &[u8]) -> Result<T, HardwareWireError> {
        if line.len() > self.max_frame_bytes {
            return Err(HardwareWireError::FrameTooLarge {
                actual: line.len(),
                maximum: self.max_frame_bytes,
            });
        }
        if line.last() != Some(&b'\n') {
            return Err(HardwareWireError::MissingNewline);
        }
        let body = &line[..line.len() - 1];
        if body.is_empty() {
            return Err(HardwareWireError::EmptyFrame);
        }
        if body.iter().any(|byte| matches!(byte, b'\n' | b'\r')) {
            return Err(HardwareWireError::MultipleLines);
        }
        serde_json::from_slice(body).map_err(|error| HardwareWireError::Json(error.to_string()))
    }
}

impl Default for HardwareWireCodec {
    fn default() -> Self {
        Self {
            max_frame_bytes: DEFAULT_MAX_HARDWARE_WIRE_FRAME_BYTES,
        }
    }
}

/// Terminal classification for a recorded wire exchange.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum HardwareWireTraceOutcome {
    /// The host and device completed an explicit close exchange.
    Completed,
    /// The device ended authority and applied its safe stop.
    Disconnected {
        /// Transport-neutral reason.
        reason: WireDisconnectReason,
    },
    /// The device asserted a typed safety condition and applied its safe stop.
    SafetyStopped {
        /// Typed safety condition.
        reason: SafetyReason,
    },
    /// The gateway initiated a stop and the device acknowledged zero output.
    GatewaySafetyStopped {
        /// Gateway safety condition.
        reason: SafetyReason,
    },
}

/// One typed direction entry in a hardware wire trace.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "direction", rename_all = "snake_case", deny_unknown_fields)]
pub enum HardwareWireTraceEntry {
    /// Host-to-device request.
    Host {
        /// Exact validated host frame.
        frame: HostWireFrame,
    },
    /// Device-to-host response.
    Device {
        /// Exact validated device frame.
        frame: DeviceWireFrame,
    },
}

/// Complete bounded, replayable trace of one process protocol session.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareWireTrace {
    /// Stable trace discriminator.
    pub kind: String,
    /// Trace schema version.
    pub schema_version: u32,
    /// Session identity shared by all frames.
    pub session_id: String,
    /// Task identity accepted during open.
    pub task_id: String,
    /// Alternating host/device frames in exact exchange order.
    pub entries: Vec<HardwareWireTraceEntry>,
    /// Terminal session classification.
    pub outcome: HardwareWireTraceOutcome,
}

impl HardwareWireTrace {
    /// Validates the trace envelope, every frame, request correlation, open
    /// handshake, and terminal response.
    pub fn validate(&self) -> Result<(), HardwareWireTraceError> {
        if self.kind != HARDWARE_WIRE_TRACE_KIND {
            return Err(HardwareWireTraceError::InvalidKind {
                actual: self.kind.clone(),
            });
        }
        if self.schema_version != HARDWARE_WIRE_SCHEMA_VERSION {
            return Err(HardwareWireTraceError::UnsupportedSchemaVersion {
                actual: self.schema_version,
            });
        }
        let mut recorder = HardwareWireTraceRecorder::new(
            self.session_id.clone(),
            self.task_id.clone(),
            self.entries.len().max(1),
        )?;
        for entry in &self.entries {
            match entry {
                HardwareWireTraceEntry::Host { frame } => {
                    recorder.record_host(frame.clone())?;
                }
                HardwareWireTraceEntry::Device { frame } => {
                    recorder.record_device(frame.clone())?;
                }
            }
        }
        recorder.finish(self.outcome)?;
        Ok(())
    }
}

impl HardwareWireTraceEntry {
    fn host_frame(&self) -> Option<&HostWireFrame> {
        match self {
            Self::Host { frame } => Some(frame),
            Self::Device { .. } => None,
        }
    }
}

/// Correlated gateway decisions and process frames for one hardware session.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareSessionEvidence {
    /// Stable evidence discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Process session identity.
    pub session_id: String,
    /// Bound portable task identity.
    pub task_id: String,
    /// Authority mode used by the gateway.
    pub mode: HardwareMode,
    /// Exact bounded host/device exchange.
    pub wire_trace: HardwareWireTrace,
    /// Gateway decisions and final fail-closed status.
    pub gateway: GatewayEvidence,
}

impl HardwareSessionEvidence {
    /// Correlates a complete wire trace with gateway evidence and validates the
    /// terminal safety outcome.
    pub fn new(
        wire_trace: HardwareWireTrace,
        gateway: GatewayEvidence,
    ) -> Result<Self, HardwareSessionEvidenceError> {
        wire_trace
            .validate()
            .map_err(|error| HardwareSessionEvidenceError::InvalidWireTrace {
                reason: error.to_string(),
            })?;
        if gateway.kind != HARDWARE_GATEWAY_EVIDENCE_KIND
            || gateway.schema_version != HARDWARE_GATEWAY_SCHEMA_VERSION
            || gateway.task_id != gateway.final_snapshot.task_id
            || gateway.mode != gateway.final_snapshot.mode
        {
            return Err(HardwareSessionEvidenceError::InvalidGatewayContract);
        }
        if wire_trace.task_id != gateway.task_id {
            return Err(HardwareSessionEvidenceError::TaskMismatch {
                wire: wire_trace.task_id.clone(),
                gateway: gateway.task_id.clone(),
            });
        }
        let open = wire_trace
            .entries
            .first()
            .and_then(HardwareWireTraceEntry::host_frame)
            .expect("validated trace starts with a host frame");
        let HostWirePayload::Open { mode, .. } = &open.payload else {
            unreachable!("validated trace starts with an open frame");
        };
        if *mode != gateway.mode {
            return Err(HardwareSessionEvidenceError::ModeMismatch {
                wire: *mode,
                gateway: gateway.mode,
            });
        }
        match wire_trace.outcome {
            HardwareWireTraceOutcome::Completed => {}
            HardwareWireTraceOutcome::Disconnected { .. } => {
                if gateway.final_snapshot.connection_state != GatewayConnectionState::Disconnected
                    || gateway.final_snapshot.safety_latch != Some(SafetyReason::Disconnected)
                {
                    return Err(HardwareSessionEvidenceError::TerminalSafetyMismatch);
                }
            }
            HardwareWireTraceOutcome::SafetyStopped { reason } => {
                if gateway.final_snapshot.safety_latch != Some(reason) {
                    return Err(HardwareSessionEvidenceError::TerminalSafetyMismatch);
                }
            }
            HardwareWireTraceOutcome::GatewaySafetyStopped { reason } => {
                if gateway.final_snapshot.safety_latch != Some(reason) {
                    return Err(HardwareSessionEvidenceError::TerminalSafetyMismatch);
                }
            }
        }
        Ok(Self {
            kind: HARDWARE_SESSION_EVIDENCE_KIND.to_string(),
            schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
            session_id: wire_trace.session_id.clone(),
            task_id: wire_trace.task_id.clone(),
            mode: gateway.mode,
            wire_trace,
            gateway,
        })
    }
}

/// Failure correlating process frames with gateway safety evidence.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum HardwareSessionEvidenceError {
    /// The process trace failed its complete replay validation.
    #[error("invalid hardware wire trace: {reason}")]
    InvalidWireTrace {
        /// First failed trace invariant.
        reason: String,
    },
    /// The gateway evidence kind or schema version is unsupported.
    #[error("invalid hardware gateway evidence contract")]
    InvalidGatewayContract,
    /// Wire and gateway artifacts name different tasks.
    #[error("hardware evidence task mismatch: wire {wire:?}, gateway {gateway:?}")]
    TaskMismatch {
        /// Wire trace task identity.
        wire: String,
        /// Gateway evidence task identity.
        gateway: String,
    },
    /// Wire and gateway artifacts use different authority modes.
    #[error("hardware evidence mode mismatch: wire {wire:?}, gateway {gateway:?}")]
    ModeMismatch {
        /// Mode requested during the wire open handshake.
        wire: HardwareMode,
        /// Gateway authority mode.
        gateway: HardwareMode,
    },
    /// Terminal wire and gateway safety states disagree.
    #[error("hardware terminal wire outcome does not match gateway safety state")]
    TerminalSafetyMismatch,
}

/// Bounded recorder that refuses to drop replay-relevant wire frames.
#[derive(Debug)]
pub struct HardwareWireTraceRecorder {
    session_id: String,
    task_id: String,
    capacity: usize,
    entries: Vec<HardwareWireTraceEntry>,
    last_host_sequence: Option<u64>,
    pending_request_sequence: Option<u64>,
}

impl HardwareWireTraceRecorder {
    /// Creates a recorder with a non-zero entry bound.
    pub fn new(
        session_id: impl Into<String>,
        task_id: impl Into<String>,
        capacity: usize,
    ) -> Result<Self, HardwareWireTraceError> {
        let session_id = session_id.into();
        let task_id = task_id.into();
        if session_id.trim().is_empty() {
            return Err(HardwareWireTraceError::EmptyIdentifier {
                field: "session_id",
            });
        }
        if task_id.trim().is_empty() {
            return Err(HardwareWireTraceError::EmptyIdentifier { field: "task_id" });
        }
        if capacity == 0 {
            return Err(HardwareWireTraceError::ZeroCapacity);
        }
        Ok(Self {
            session_id,
            task_id,
            capacity,
            entries: Vec::with_capacity(capacity),
            last_host_sequence: None,
            pending_request_sequence: None,
        })
    }

    /// Records one request and reserves its matching response position.
    pub fn record_host(&mut self, frame: HostWireFrame) -> Result<(), HardwareWireTraceError> {
        frame
            .validate()
            .map_err(|error| HardwareWireTraceError::InvalidFrame {
                reason: error.to_string(),
            })?;
        self.require_capacity()?;
        self.require_session(&frame.session_id)?;
        if let Some(pending) = self.pending_request_sequence {
            return Err(HardwareWireTraceError::ResponsePending { sequence: pending });
        }
        if let Some(previous) = self.last_host_sequence {
            if frame.sequence <= previous {
                return Err(HardwareWireTraceError::NonMonotonicHostSequence {
                    previous,
                    actual: frame.sequence,
                });
            }
        }
        self.last_host_sequence = Some(frame.sequence);
        self.pending_request_sequence = Some(frame.sequence);
        self.entries.push(HardwareWireTraceEntry::Host { frame });
        Ok(())
    }

    /// Records the response correlated to the most recent request.
    pub fn record_device(&mut self, frame: DeviceWireFrame) -> Result<(), HardwareWireTraceError> {
        frame
            .validate()
            .map_err(|error| HardwareWireTraceError::InvalidFrame {
                reason: error.to_string(),
            })?;
        self.require_capacity()?;
        self.require_session(&frame.session_id)?;
        let pending = self
            .pending_request_sequence
            .ok_or(HardwareWireTraceError::UnexpectedResponse)?;
        if frame.request_sequence != pending {
            return Err(HardwareWireTraceError::ResponseSequenceMismatch {
                expected: pending,
                actual: frame.request_sequence,
            });
        }
        self.pending_request_sequence = None;
        self.entries.push(HardwareWireTraceEntry::Device { frame });
        Ok(())
    }

    /// Finishes only after every request has a correlated response.
    pub fn finish(
        self,
        outcome: HardwareWireTraceOutcome,
    ) -> Result<HardwareWireTrace, HardwareWireTraceError> {
        if let Some(sequence) = self.pending_request_sequence {
            return Err(HardwareWireTraceError::ResponsePending { sequence });
        }
        if self.entries.is_empty() {
            return Err(HardwareWireTraceError::EmptyTrace);
        }
        validate_trace_semantics(&self.entries, &self.task_id, outcome)?;
        Ok(HardwareWireTrace {
            kind: HARDWARE_WIRE_TRACE_KIND.to_string(),
            schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
            session_id: self.session_id,
            task_id: self.task_id,
            entries: self.entries,
            outcome,
        })
    }

    fn require_capacity(&self) -> Result<(), HardwareWireTraceError> {
        if self.entries.len() == self.capacity {
            Err(HardwareWireTraceError::CapacityExceeded {
                capacity: self.capacity,
            })
        } else {
            Ok(())
        }
    }

    fn require_session(&self, actual: &str) -> Result<(), HardwareWireTraceError> {
        if actual == self.session_id {
            Ok(())
        } else {
            Err(HardwareWireTraceError::SessionMismatch {
                expected: self.session_id.clone(),
                actual: actual.to_string(),
            })
        }
    }
}

/// Failure encoding, decoding, or validating a wire frame.
#[derive(Debug, thiserror::Error)]
pub enum HardwareWireError {
    /// The configured byte bound is zero or exceeds the hard ceiling.
    #[error("hardware wire frame bound {requested} must be in 1..={maximum}")]
    InvalidFrameBound {
        /// Requested encoded byte bound.
        requested: usize,
        /// Protocol hard ceiling.
        maximum: usize,
    },
    /// The encoded or received frame exceeded its configured byte bound.
    #[error("hardware wire frame has at least {actual} bytes, maximum is {maximum}")]
    FrameTooLarge {
        /// Observed or minimum attempted size.
        actual: usize,
        /// Configured maximum size.
        maximum: usize,
    },
    /// A frame ended without its canonical newline.
    #[error("hardware wire frame must end with a newline")]
    MissingNewline,
    /// A line contained no JSON body.
    #[error("hardware wire frame must not be empty")]
    EmptyFrame,
    /// A frame contained embedded line separators.
    #[error("hardware wire frame must contain exactly one JSON line")]
    MultipleLines,
    /// JSON could not be encoded or decoded.
    #[error("invalid hardware wire JSON: {0}")]
    Json(String),
    /// The frame kind is not valid for its direction.
    #[error("invalid hardware wire kind: expected {expected:?}, got {actual:?}")]
    InvalidKind {
        /// Required direction discriminator.
        expected: &'static str,
        /// Received discriminator.
        actual: String,
    },
    /// The peer uses an unsupported schema version.
    #[error("unsupported hardware wire schema: expected {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Supported schema version.
        expected: u32,
        /// Received schema version.
        actual: u32,
    },
    /// A required identity is empty.
    #[error("hardware wire {field} must not be empty")]
    EmptyIdentifier {
        /// Invalid field name.
        field: &'static str,
    },
    /// A flattened space has zero width.
    #[error("hardware wire {field} must be greater than zero")]
    ZeroWidth {
        /// Invalid width field.
        field: &'static str,
    },
    /// A numeric vector contains NaN or infinity.
    #[error("hardware wire {space} value {index} must be finite")]
    NonFiniteValue {
        /// Observation or actuation space.
        space: &'static str,
        /// Flattened value index.
        index: usize,
    },
    /// An actuation frame combines action and safety fields inconsistently.
    #[error("hardware wire actuation frame has inconsistent safety fields")]
    InvalidActuation,
    /// A terminal device response failed to confirm an independent safe stop.
    #[error("terminal hardware response must confirm a device-side safe stop")]
    UnsafeTerminalResponse,
    /// Reading the bounded transport failed.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// Failure retaining an exact request/response trace.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum HardwareWireTraceError {
    /// The trace discriminator is unsupported.
    #[error("invalid hardware wire trace kind {actual:?}")]
    InvalidKind {
        /// Received trace kind.
        actual: String,
    },
    /// The trace schema version is unsupported.
    #[error("unsupported hardware wire trace schema {actual}")]
    UnsupportedSchemaVersion {
        /// Received schema version.
        actual: u32,
    },
    /// A required trace identifier is empty.
    #[error("hardware wire trace {field} must not be empty")]
    EmptyIdentifier {
        /// Invalid identifier field.
        field: &'static str,
    },
    /// The trace cannot retain any entries.
    #[error("hardware wire trace capacity must be greater than zero")]
    ZeroCapacity,
    /// A retained frame is invalid protocol data.
    #[error("invalid hardware wire trace frame: {reason}")]
    InvalidFrame {
        /// First failed frame invariant.
        reason: String,
    },
    /// The trace is full and refuses to discard replay evidence.
    #[error("hardware wire trace capacity {capacity} exceeded")]
    CapacityExceeded {
        /// Configured entry capacity.
        capacity: usize,
    },
    /// A frame belongs to a different session.
    #[error("hardware wire session mismatch: expected {expected:?}, got {actual:?}")]
    SessionMismatch {
        /// Recorder session.
        expected: String,
        /// Frame session.
        actual: String,
    },
    /// A second request arrived before the first response.
    #[error("hardware wire request {sequence} is still awaiting a response")]
    ResponsePending {
        /// Pending host sequence.
        sequence: u64,
    },
    /// A host request sequence did not increase.
    #[error("hardware host sequence {actual} must be greater than {previous}")]
    NonMonotonicHostSequence {
        /// Previous host sequence.
        previous: u64,
        /// Rejected host sequence.
        actual: u64,
    },
    /// A response arrived without a pending request.
    #[error("hardware device response has no pending host request")]
    UnexpectedResponse,
    /// A response answered a different host sequence.
    #[error("hardware device response targets {actual}, expected {expected}")]
    ResponseSequenceMismatch {
        /// Pending host sequence.
        expected: u64,
        /// Response request sequence.
        actual: u64,
    },
    /// No exchange was recorded.
    #[error("hardware wire trace must contain at least one exchange")]
    EmptyTrace,
    /// The first exchange is not one matching open/ready handshake.
    #[error("hardware wire trace must begin with a matching open/ready handshake")]
    InvalidHandshake,
    /// The final device response does not prove the declared trace outcome.
    #[error("hardware wire trace terminal response does not match its outcome")]
    OutcomeMismatch,
}

fn validate_trace_semantics(
    entries: &[HardwareWireTraceEntry],
    task_id: &str,
    outcome: HardwareWireTraceOutcome,
) -> Result<(), HardwareWireTraceError> {
    let handshake_matches = match entries {
        [HardwareWireTraceEntry::Host {
            frame:
                HostWireFrame {
                    payload:
                        HostWirePayload::Open {
                            task_id: opened_task,
                            observation_width: opened_observation_width,
                            action_width: opened_action_width,
                            ..
                        },
                    ..
                },
        }, HardwareWireTraceEntry::Device {
            frame:
                DeviceWireFrame {
                    payload:
                        DeviceWirePayload::Ready {
                            task_id: ready_task,
                            observation_width: ready_observation_width,
                            action_width: ready_action_width,
                            ..
                        },
                    ..
                },
        }, ..] => {
            opened_task == task_id
                && ready_task == task_id
                && opened_observation_width == ready_observation_width
                && opened_action_width == ready_action_width
        }
        _ => false,
    };
    if !handshake_matches {
        return Err(HardwareWireTraceError::InvalidHandshake);
    }

    let terminal_matches = match (outcome, entries.last()) {
        (
            HardwareWireTraceOutcome::Completed,
            Some(HardwareWireTraceEntry::Device {
                frame:
                    DeviceWireFrame {
                        payload: DeviceWirePayload::Closed,
                        ..
                    },
            }),
        ) => true,
        (
            HardwareWireTraceOutcome::Disconnected { reason: expected },
            Some(HardwareWireTraceEntry::Device {
                frame:
                    DeviceWireFrame {
                        payload:
                            DeviceWirePayload::Disconnected {
                                reason,
                                safe_stop_applied: true,
                            },
                        ..
                    },
            }),
        ) => expected == *reason,
        (
            HardwareWireTraceOutcome::SafetyStopped { reason: expected },
            Some(HardwareWireTraceEntry::Device {
                frame:
                    DeviceWireFrame {
                        payload:
                            DeviceWirePayload::SafetySignal {
                                reason,
                                safe_stop_applied: true,
                            },
                        ..
                    },
            }),
        ) => expected == *reason,
        (
            HardwareWireTraceOutcome::GatewaySafetyStopped { .. },
            Some(HardwareWireTraceEntry::Device {
                frame:
                    DeviceWireFrame {
                        payload:
                            DeviceWirePayload::ActuationAccepted {
                                action_sequence: None,
                                safety_stop: true,
                            },
                        ..
                    },
            }),
        ) => true,
        _ => false,
    };
    if !terminal_matches {
        return Err(HardwareWireTraceError::OutcomeMismatch);
    }
    Ok(())
}

fn validate_envelope(
    kind: &str,
    expected_kind: &'static str,
    schema_version: u32,
    session_id: &str,
) -> Result<(), HardwareWireError> {
    if kind != expected_kind {
        return Err(HardwareWireError::InvalidKind {
            expected: expected_kind,
            actual: kind.to_string(),
        });
    }
    if schema_version != HARDWARE_WIRE_SCHEMA_VERSION {
        return Err(HardwareWireError::UnsupportedSchemaVersion {
            expected: HARDWARE_WIRE_SCHEMA_VERSION,
            actual: schema_version,
        });
    }
    validate_identifier("session_id", session_id)
}

fn validate_identifier(field: &'static str, value: &str) -> Result<(), HardwareWireError> {
    if value.trim().is_empty() {
        Err(HardwareWireError::EmptyIdentifier { field })
    } else {
        Ok(())
    }
}

fn validate_width(field: &'static str, width: usize) -> Result<(), HardwareWireError> {
    if width == 0 {
        Err(HardwareWireError::ZeroWidth { field })
    } else {
        Ok(())
    }
}

fn validate_finite(space: &'static str, values: &[f64]) -> Result<(), HardwareWireError> {
    if let Some(index) = values.iter().position(|value| !value.is_finite()) {
        Err(HardwareWireError::NonFiniteValue { space, index })
    } else {
        Ok(())
    }
}

fn validate_actuation(frame: &ActuationFrame) -> Result<(), HardwareWireError> {
    validate_width("actuation.values", frame.values.len())?;
    validate_finite("actuation", &frame.values)?;
    let consistent = if frame.safety_stop {
        frame.action_sequence.is_none() && frame.reason.is_some()
    } else {
        frame.action_sequence.is_some() && frame.reason.is_none()
    };
    if consistent {
        Ok(())
    } else {
        Err(HardwareWireError::InvalidActuation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufReader, Cursor};

    fn open() -> HostWireFrame {
        HostWireFrame::new(
            "session-1",
            1,
            HostWirePayload::Open {
                task_id: "rne.test.task.v1".into(),
                mode: HardwareMode::Hil,
                observation_width: 3,
                action_width: 2,
            },
        )
    }

    #[test]
    fn codec_round_trip_is_one_bounded_line() {
        let codec = HardwareWireCodec::default();
        let bytes = codec.encode_host(&open()).unwrap();
        assert_eq!(bytes.last(), Some(&b'\n'));
        assert_eq!(codec.decode_host(&bytes).unwrap(), open());
    }

    #[test]
    fn codec_rejects_unknown_fields_and_multiple_lines() {
        let codec = HardwareWireCodec::default();
        let unknown = b"{\"kind\":\"rne_hardware_host_frame\",\"schema_version\":1,\"session_id\":\"s\",\"sequence\":1,\"payload\":{\"type\":\"poll_observation\"},\"extra\":true}\n";
        assert!(matches!(
            codec.decode_host(unknown),
            Err(HardwareWireError::Json(_))
        ));
        let two = b"{}\n{}\n";
        assert!(matches!(
            codec.decode_host(two),
            Err(HardwareWireError::MultipleLines)
        ));
    }

    #[test]
    fn bounded_reader_rejects_before_unbounded_allocation() {
        let codec = HardwareWireCodec::new(8).unwrap();
        let mut reader = BufReader::with_capacity(4, Cursor::new(b"123456789\n"));
        assert!(matches!(
            codec.read_line(&mut reader),
            Err(HardwareWireError::FrameTooLarge { .. })
        ));
    }

    #[test]
    fn terminal_response_must_confirm_device_stop() {
        let frame = DeviceWireFrame::new(
            "session-1",
            2,
            DeviceWirePayload::Disconnected {
                reason: WireDisconnectReason::TransportFault,
                safe_stop_applied: false,
            },
        );
        assert!(matches!(
            frame.validate(),
            Err(HardwareWireError::UnsafeTerminalResponse)
        ));
    }

    #[test]
    fn trace_refuses_overwrite_and_uncorrelated_responses() {
        let mut recorder =
            HardwareWireTraceRecorder::new("session-1", "rne.test.task.v1", 2).unwrap();
        let host = open();
        recorder.record_host(host.clone()).unwrap();
        let wrong = DeviceWireFrame::new(
            "session-1",
            9,
            DeviceWirePayload::Rejected {
                code: WireRejectionCode::NotOpen,
            },
        );
        assert_eq!(
            recorder.record_device(wrong),
            Err(HardwareWireTraceError::ResponseSequenceMismatch {
                expected: 1,
                actual: 9,
            })
        );
        let ready = DeviceWireFrame::new(
            "session-1",
            1,
            DeviceWirePayload::Ready {
                device_id: "mock-1".into(),
                task_id: "rne.test.task.v1".into(),
                observation_width: 3,
                action_width: 2,
            },
        );
        recorder.record_device(ready).unwrap();
        assert_eq!(
            recorder.record_host(HostWireFrame::new("session-1", 2, HostWirePayload::Close,)),
            Err(HardwareWireTraceError::CapacityExceeded { capacity: 2 })
        );
    }
}
