//! Bounded JSON Lines protocol for fixed-step external simulator processes.

use super::{is_sha256_hex, valid_identifier};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::io::BufRead;

/// Current external-simulator process protocol schema.
pub const SIMULATOR_WIRE_SCHEMA_VERSION: u32 = 1;

/// Stable host-frame discriminator.
pub const SIMULATOR_HOST_FRAME_KIND: &str = "rne_simulator_host_frame";

/// Stable adapter-frame discriminator.
pub const SIMULATOR_ADAPTER_FRAME_KIND: &str = "rne_simulator_adapter_frame";

/// Default maximum encoded frame size, including its newline.
pub const DEFAULT_MAX_SIMULATOR_WIRE_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// Hard maximum encoded frame size.
pub const MAX_SIMULATOR_WIRE_FRAME_BYTES: usize = 64 * 1024 * 1024;

/// Host command sent to an external simulator adapter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum SimulatorHostPayload {
    /// Binds one TaskSpec and fixed-step execution session.
    Open {
        /// Stable TaskSpec identity.
        task_id: String,
        /// SHA-256 of the exact TaskSpec bytes.
        task_sha256: String,
        /// Flattened observation width.
        observation_width: usize,
        /// Flattened action width.
        action_width: usize,
        /// Simulation-time ticks advanced by every accepted action.
        fixed_delta_ticks: u64,
    },
    /// Resets the bound world to a deterministic episode seed.
    Reset {
        /// Exact reset seed owned by the caller.
        seed: u64,
    },
    /// Applies one TaskSpec-ordered action and advances exactly one fixed step.
    Step {
        /// Strictly increasing action sequence within the reset episode.
        action_sequence: u64,
        /// Flattened action values in TaskSpec order.
        values: Vec<f64>,
    },
    /// Closes the current session and releases simulator resources.
    Close,
}

/// One versioned host-to-adapter frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorHostFrame {
    /// Stable frame discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Caller-selected session identity.
    pub session_id: String,
    /// Strictly increasing request sequence.
    pub sequence: u64,
    /// Typed host command.
    pub payload: SimulatorHostPayload,
}

impl SimulatorHostFrame {
    /// Creates a protocol-v1 host frame.
    pub fn new(
        session_id: impl Into<String>,
        sequence: u64,
        payload: SimulatorHostPayload,
    ) -> Self {
        Self {
            kind: SIMULATOR_HOST_FRAME_KIND.to_string(),
            schema_version: SIMULATOR_WIRE_SCHEMA_VERSION,
            session_id: session_id.into(),
            sequence,
            payload,
        }
    }

    /// Validates envelope, fixed-step, digest, width, and numeric invariants.
    pub fn validate(&self) -> Result<(), SimulatorWireError> {
        validate_envelope(
            &self.kind,
            SIMULATOR_HOST_FRAME_KIND,
            self.schema_version,
            &self.session_id,
        )?;
        match &self.payload {
            SimulatorHostPayload::Open {
                task_id,
                task_sha256,
                observation_width,
                action_width,
                fixed_delta_ticks,
            } => {
                if !valid_identifier(task_id) {
                    return Err(SimulatorWireError::InvalidIdentifier("task_id"));
                }
                if !is_sha256_hex(task_sha256) {
                    return Err(SimulatorWireError::InvalidDigest);
                }
                validate_width(*observation_width)?;
                validate_width(*action_width)?;
                if *fixed_delta_ticks == 0 {
                    return Err(SimulatorWireError::InvalidFixedDelta);
                }
            }
            SimulatorHostPayload::Step { values, .. } => {
                validate_width(values.len())?;
                validate_finite(values)?;
            }
            SimulatorHostPayload::Reset { .. } | SimulatorHostPayload::Close => {}
        }
        Ok(())
    }
}

/// Stable adapter-side protocol rejection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimulatorRejectionCode {
    /// Command requires an open TaskSpec session.
    NotOpen,
    /// Open was requested twice without closing.
    AlreadyOpen,
    /// Request sequence did not strictly increase.
    NonMonotonicSequence,
    /// Request targeted a different session.
    SessionMismatch,
    /// TaskSpec identity or digest differs from adapter configuration.
    TaskMismatch,
    /// Observation or action width differs from the TaskSpec mapping.
    WidthMismatch,
    /// Fixed simulation delta differs from the adapter configuration.
    FixedDeltaMismatch,
    /// Step was requested before reset.
    ResetRequired,
    /// Action sequence did not strictly increase from zero.
    ActionSequenceMismatch,
    /// Action contains a non-finite value.
    NonFiniteValue,
    /// Session already reached a terminal state.
    TerminalState,
}

/// Adapter response produced after a host request.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum SimulatorAdapterPayload {
    /// Confirms the exact TaskSpec and runtime identity accepted by the adapter.
    Ready {
        /// Stable simulator family.
        simulator_id: String,
        /// Exact simulator runtime version.
        simulator_version: String,
        /// Stable adapter implementation identity.
        adapter_id: String,
        /// Accepted TaskSpec identity.
        task_id: String,
        /// Accepted TaskSpec digest.
        task_sha256: String,
        /// Accepted flattened observation width.
        observation_width: usize,
        /// Accepted flattened action width.
        action_width: usize,
        /// Accepted simulation-time ticks per step.
        fixed_delta_ticks: u64,
    },
    /// Returns the initial observation at step and simulation time zero.
    ResetComplete {
        /// Seed applied to the reset world.
        seed: u64,
        /// Initial observation values in TaskSpec order.
        values: Vec<f64>,
        /// Backend-specific stable digest for same-runtime determinism checks.
        state_digest: u64,
    },
    /// Returns the observation reached after exactly one action step.
    Stepped {
        /// Accepted action sequence.
        action_sequence: u64,
        /// One-based step reached by this response.
        step: u64,
        /// Exact accumulated simulation-time ticks.
        sim_time_ticks: u64,
        /// Observation values in TaskSpec order.
        values: Vec<f64>,
        /// Task-owned terminal flag.
        terminated: bool,
        /// Task-owned truncation flag.
        truncated: bool,
        /// Backend-specific stable digest for same-runtime determinism checks.
        state_digest: u64,
    },
    /// Confirms explicit session close.
    Closed,
    /// Rejects the request before changing simulator state.
    Rejected {
        /// First violated protocol invariant.
        code: SimulatorRejectionCode,
    },
}

/// One versioned adapter-to-host frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorAdapterFrame {
    /// Stable frame discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Session identity correlated to the request.
    pub session_id: String,
    /// Host request sequence answered by this response.
    pub request_sequence: u64,
    /// Typed adapter response.
    pub payload: SimulatorAdapterPayload,
}

impl SimulatorAdapterFrame {
    /// Creates a protocol-v1 adapter frame.
    pub fn new(
        session_id: impl Into<String>,
        request_sequence: u64,
        payload: SimulatorAdapterPayload,
    ) -> Self {
        Self {
            kind: SIMULATOR_ADAPTER_FRAME_KIND.to_string(),
            schema_version: SIMULATOR_WIRE_SCHEMA_VERSION,
            session_id: session_id.into(),
            request_sequence,
            payload,
        }
    }

    /// Validates envelope, identity, widths, digests, and numeric payloads.
    pub fn validate(&self) -> Result<(), SimulatorWireError> {
        validate_envelope(
            &self.kind,
            SIMULATOR_ADAPTER_FRAME_KIND,
            self.schema_version,
            &self.session_id,
        )?;
        match &self.payload {
            SimulatorAdapterPayload::Ready {
                simulator_id,
                simulator_version,
                adapter_id,
                task_id,
                task_sha256,
                observation_width,
                action_width,
                fixed_delta_ticks,
            } => {
                for (field, value) in [
                    ("simulator_id", simulator_id.as_str()),
                    ("simulator_version", simulator_version.as_str()),
                    ("adapter_id", adapter_id.as_str()),
                    ("task_id", task_id.as_str()),
                ] {
                    if !valid_identifier(value) {
                        return Err(SimulatorWireError::InvalidIdentifier(field));
                    }
                }
                if !is_sha256_hex(task_sha256) {
                    return Err(SimulatorWireError::InvalidDigest);
                }
                validate_width(*observation_width)?;
                validate_width(*action_width)?;
                if *fixed_delta_ticks == 0 {
                    return Err(SimulatorWireError::InvalidFixedDelta);
                }
            }
            SimulatorAdapterPayload::ResetComplete { values, .. }
            | SimulatorAdapterPayload::Stepped { values, .. } => {
                validate_width(values.len())?;
                validate_finite(values)?;
            }
            SimulatorAdapterPayload::Closed | SimulatorAdapterPayload::Rejected { .. } => {}
        }
        Ok(())
    }
}

/// Maximum-size JSON Lines codec for simulator protocol frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SimulatorWireCodec {
    max_frame_bytes: usize,
}

impl SimulatorWireCodec {
    /// Creates a codec with a non-zero bound no larger than the hard ceiling.
    pub fn new(max_frame_bytes: usize) -> Result<Self, SimulatorWireError> {
        if !(1..=MAX_SIMULATOR_WIRE_FRAME_BYTES).contains(&max_frame_bytes) {
            return Err(SimulatorWireError::InvalidFrameBound {
                requested: max_frame_bytes,
                maximum: MAX_SIMULATOR_WIRE_FRAME_BYTES,
            });
        }
        Ok(Self { max_frame_bytes })
    }

    /// Returns the encoded frame bound, including its newline.
    pub fn max_frame_bytes(self) -> usize {
        self.max_frame_bytes
    }

    /// Encodes a validated host frame.
    pub fn encode_host(&self, frame: &SimulatorHostFrame) -> Result<Vec<u8>, SimulatorWireError> {
        frame.validate()?;
        self.encode(frame)
    }

    /// Decodes and validates one host frame.
    pub fn decode_host(&self, line: &[u8]) -> Result<SimulatorHostFrame, SimulatorWireError> {
        let frame: SimulatorHostFrame = self.decode(line)?;
        frame.validate()?;
        Ok(frame)
    }

    /// Encodes a validated adapter frame.
    pub fn encode_adapter(
        &self,
        frame: &SimulatorAdapterFrame,
    ) -> Result<Vec<u8>, SimulatorWireError> {
        frame.validate()?;
        self.encode(frame)
    }

    /// Decodes and validates one adapter frame.
    pub fn decode_adapter(&self, line: &[u8]) -> Result<SimulatorAdapterFrame, SimulatorWireError> {
        let frame: SimulatorAdapterFrame = self.decode(line)?;
        frame.validate()?;
        Ok(frame)
    }

    /// Reads at most one bounded newline-terminated frame.
    pub fn read_line<R: BufRead>(
        &self,
        reader: &mut R,
    ) -> Result<Option<Vec<u8>>, SimulatorWireError> {
        let mut line = Vec::new();
        loop {
            let available = reader.fill_buf()?;
            if available.is_empty() {
                return if line.is_empty() {
                    Ok(None)
                } else {
                    Err(SimulatorWireError::MissingNewline)
                };
            }
            let consumed = available
                .iter()
                .position(|byte| *byte == b'\n')
                .map_or(available.len(), |index| index + 1);
            let attempted = line.len().saturating_add(consumed);
            if attempted > self.max_frame_bytes {
                return Err(SimulatorWireError::FrameTooLarge {
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

    fn encode<T: Serialize>(&self, frame: &T) -> Result<Vec<u8>, SimulatorWireError> {
        let mut bytes = serde_json::to_vec(frame)?;
        bytes.push(b'\n');
        if bytes.len() > self.max_frame_bytes {
            return Err(SimulatorWireError::FrameTooLarge {
                actual: bytes.len(),
                maximum: self.max_frame_bytes,
            });
        }
        Ok(bytes)
    }

    fn decode<T: DeserializeOwned>(&self, line: &[u8]) -> Result<T, SimulatorWireError> {
        if line.len() > self.max_frame_bytes {
            return Err(SimulatorWireError::FrameTooLarge {
                actual: line.len(),
                maximum: self.max_frame_bytes,
            });
        }
        if line.last() != Some(&b'\n') {
            return Err(SimulatorWireError::MissingNewline);
        }
        let body = &line[..line.len() - 1];
        if body.is_empty() {
            return Err(SimulatorWireError::EmptyFrame);
        }
        if body.iter().any(|byte| matches!(byte, b'\n' | b'\r')) {
            return Err(SimulatorWireError::MultipleLines);
        }
        Ok(serde_json::from_slice(body)?)
    }
}

impl Default for SimulatorWireCodec {
    fn default() -> Self {
        Self {
            max_frame_bytes: DEFAULT_MAX_SIMULATOR_WIRE_FRAME_BYTES,
        }
    }
}

/// Invalid simulator process frame or bounded codec operation.
#[derive(Debug, thiserror::Error)]
pub enum SimulatorWireError {
    /// Frame kind does not match its direction.
    #[error("invalid external simulator frame kind")]
    InvalidKind,
    /// Frame schema is unsupported.
    #[error("unsupported external simulator wire schema {0}")]
    UnsupportedSchemaVersion(u32),
    /// A stable identity field is malformed.
    #[error("invalid external simulator identifier {0}")]
    InvalidIdentifier(&'static str),
    /// TaskSpec digest is not lowercase SHA-256 hex.
    #[error("invalid external simulator TaskSpec digest")]
    InvalidDigest,
    /// Tensor width is zero or exceeds the protocol bound.
    #[error("invalid external simulator tensor width")]
    InvalidWidth,
    /// Fixed step duration is zero.
    #[error("invalid external simulator fixed delta")]
    InvalidFixedDelta,
    /// Numeric payload includes NaN or infinity.
    #[error("external simulator payload contains non-finite values")]
    NonFiniteValue,
    /// Requested codec bound is outside the hard limits.
    #[error("invalid simulator frame bound {requested}; maximum is {maximum}")]
    InvalidFrameBound {
        /// Requested bound.
        requested: usize,
        /// Hard maximum.
        maximum: usize,
    },
    /// Encoded frame exceeds the configured bound.
    #[error("simulator frame has {actual} bytes; maximum is {maximum}")]
    FrameTooLarge {
        /// Observed or attempted size.
        actual: usize,
        /// Configured maximum.
        maximum: usize,
    },
    /// Frame did not end with a newline.
    #[error("external simulator frame is missing its newline")]
    MissingNewline,
    /// Empty JSON line is not a frame.
    #[error("external simulator frame is empty")]
    EmptyFrame,
    /// One codec call contained multiple lines.
    #[error("external simulator frame contains multiple lines")]
    MultipleLines,
    /// JSON shape or field set is invalid.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// Bounded input read failed.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

fn validate_envelope(
    kind: &str,
    expected_kind: &str,
    schema_version: u32,
    session_id: &str,
) -> Result<(), SimulatorWireError> {
    if kind != expected_kind {
        return Err(SimulatorWireError::InvalidKind);
    }
    if schema_version != SIMULATOR_WIRE_SCHEMA_VERSION {
        return Err(SimulatorWireError::UnsupportedSchemaVersion(schema_version));
    }
    if !valid_identifier(session_id) {
        return Err(SimulatorWireError::InvalidIdentifier("session_id"));
    }
    Ok(())
}

fn validate_width(width: usize) -> Result<(), SimulatorWireError> {
    if width == 0 || width > MAX_SIMULATOR_WIRE_FRAME_BYTES / 8 {
        Err(SimulatorWireError::InvalidWidth)
    } else {
        Ok(())
    }
}

fn validate_finite(values: &[f64]) -> Result<(), SimulatorWireError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(SimulatorWireError::NonFiniteValue)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn open() -> SimulatorHostFrame {
        SimulatorHostFrame::new(
            "session-v1",
            1,
            SimulatorHostPayload::Open {
                task_id: "task-v1".to_string(),
                task_sha256: "0".repeat(64),
                observation_width: 3,
                action_width: 2,
                fixed_delta_ticks: 16_666_666,
            },
        )
    }

    #[test]
    fn codec_round_trips_one_bounded_line() {
        let codec = SimulatorWireCodec::default();
        let encoded = codec.encode_host(&open()).unwrap();
        assert_eq!(codec.decode_host(&encoded).unwrap(), open());
        let mut cursor = Cursor::new(encoded.clone());
        assert_eq!(codec.read_line(&mut cursor).unwrap(), Some(encoded));
        assert_eq!(codec.read_line(&mut cursor).unwrap(), None);
    }

    #[test]
    fn unknown_fields_and_non_finite_payloads_fail_closed() {
        let codec = SimulatorWireCodec::default();
        let mut value = serde_json::to_value(open()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("future".to_string(), serde_json::json!(true));
        let mut line = serde_json::to_vec(&value).unwrap();
        line.push(b'\n');
        assert!(matches!(
            codec.decode_host(&line),
            Err(SimulatorWireError::Json(_))
        ));
        let step = SimulatorHostFrame::new(
            "session-v1",
            2,
            SimulatorHostPayload::Step {
                action_sequence: 0,
                values: vec![f64::NAN],
            },
        );
        assert!(matches!(
            codec.encode_host(&step),
            Err(SimulatorWireError::NonFiniteValue)
        ));
    }
}
