//! Versioned framed transport for live runner frontends.
//!
//! The wire contract is deliberately independent of sockets and renderers. It
//! can be exercised headlessly and reused by native, browser, and adapter
//! frontends without exposing external ecosystem types through RNE core crates.

use crate::{ImageDepth, ImageRgb8, PointCloud};
use rne_core::control::{ControlCommand, RunnerControlState};
use rne_math::Vec3;
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use thiserror::Error;

/// Four-byte marker at the start of every transport frame.
pub const TRANSPORT_MAGIC: [u8; 4] = *b"RNEF";
/// Size of the fixed little-endian transport header.
pub const TRANSPORT_HEADER_BYTES: usize = 32;
/// Current production transport protocol major version.
pub const TRANSPORT_PROTOCOL_MAJOR: u16 = 1;
/// Current production transport protocol minor version.
pub const TRANSPORT_PROTOCOL_MINOR: u16 = 0;
/// Absolute payload safety limit used by the reference implementation.
pub const TRANSPORT_MAX_PAYLOAD_BYTES: usize = 32 * 1024 * 1024;
/// Maximum UTF-8 rejection detail carried on the wire.
pub const TRANSPORT_MAX_REJECT_MESSAGE_BYTES: usize = 1024;

/// A message carried by the framed frontend transport.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TransportMessageKind {
    /// Client version, capability, and queue-budget offer.
    ClientHello = 1,
    /// Server's selected protocol and effective limits.
    ServerHello = 2,
    /// Explicit negotiation rejection.
    Reject = 3,
    /// Runner-control command.
    ControlCommand = 4,
    /// Runner-control command acknowledgement.
    ControlAck = 5,
    /// Compact per-step status metadata.
    Status = 6,
    /// Lossless RGBA8 sensor image.
    ImageRgb8 = 7,
    /// Little-endian linear-depth image in metres.
    ImageDepthF32 = 8,
    /// LiDAR point cloud with optional aligned attributes.
    LidarPointCloud = 9,
    /// Notice that one or more latest-only messages were dropped.
    Gap = 10,
}

impl TryFrom<u16> for TransportMessageKind {
    type Error = TransportError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::ClientHello),
            2 => Ok(Self::ServerHello),
            3 => Ok(Self::Reject),
            4 => Ok(Self::ControlCommand),
            5 => Ok(Self::ControlAck),
            6 => Ok(Self::Status),
            7 => Ok(Self::ImageRgb8),
            8 => Ok(Self::ImageDepthF32),
            9 => Ok(Self::LidarPointCloud),
            10 => Ok(Self::Gap),
            other => Err(TransportError::UnknownMessageKind(other)),
        }
    }
}

/// Failure while validating, encoding, or decoding transport data.
#[derive(Debug, Error)]
pub enum TransportError {
    /// Underlying reader or writer failed.
    #[error("transport I/O failed: {0}")]
    Io(#[from] io::Error),
    /// Frame magic did not identify an RNE transport frame.
    #[error("invalid transport magic")]
    InvalidMagic,
    /// Message kind is not defined by this implementation.
    #[error("unknown transport message kind {0}")]
    UnknownMessageKind(u16),
    /// Payload exceeds the configured safety limit.
    #[error("transport payload is {actual} bytes, limit is {limit} bytes")]
    PayloadTooLarge {
        /// Payload bytes requested or declared.
        actual: usize,
        /// Maximum accepted payload bytes.
        limit: usize,
    },
    /// Input ended before a complete value was available.
    #[error("truncated transport payload")]
    Truncated,
    /// Bytes remained after decoding the expected payload.
    #[error("transport payload contains trailing bytes")]
    TrailingBytes,
    /// A named field violated its wire invariant.
    #[error("invalid transport field `{0}`")]
    InvalidField(&'static str),
    /// A UTF-8 field was malformed.
    #[error("transport text is not valid UTF-8")]
    InvalidUtf8,
}

/// One complete framed transport message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransportFrame {
    /// Protocol major version used to encode this frame.
    pub protocol_major: u16,
    /// Protocol minor version used to encode this frame.
    pub protocol_minor: u16,
    /// Message discriminator.
    pub kind: TransportMessageKind,
    /// Message-specific flags.
    pub flags: u16,
    /// Monotonic sequence within the run session.
    pub sequence: u64,
    /// Stable identifier for one runner process/session.
    pub session_id: u64,
    /// Message-specific payload bytes.
    pub payload: Vec<u8>,
}

impl TransportFrame {
    /// Creates a protocol-v1 frame.
    pub fn new(
        kind: TransportMessageKind,
        sequence: u64,
        session_id: u64,
        payload: Vec<u8>,
    ) -> Self {
        Self {
            protocol_major: TRANSPORT_PROTOCOL_MAJOR,
            protocol_minor: TRANSPORT_PROTOCOL_MINOR,
            kind,
            flags: 0,
            sequence,
            session_id,
            payload,
        }
    }

    /// Returns header plus payload bytes retained by an egress queue.
    pub fn encoded_len(&self) -> usize {
        TRANSPORT_HEADER_BYTES.saturating_add(self.payload.len())
    }

    /// Encodes this frame into one byte vector.
    pub fn encode(&self) -> Result<Vec<u8>, TransportError> {
        let mut bytes = Vec::with_capacity(self.encoded_len());
        self.write_to(&mut bytes)?;
        Ok(bytes)
    }

    /// Writes this frame without platform-dependent padding or endianness.
    pub fn write_to<W: Write>(&self, writer: &mut W) -> Result<(), TransportError> {
        validate_payload_len(self.payload.len(), TRANSPORT_MAX_PAYLOAD_BYTES)?;
        let payload_len = u32::try_from(self.payload.len())
            .map_err(|_| TransportError::InvalidField("payload_len"))?;
        let mut header = [0_u8; TRANSPORT_HEADER_BYTES];
        header[0..4].copy_from_slice(&TRANSPORT_MAGIC);
        header[4..6].copy_from_slice(&self.protocol_major.to_le_bytes());
        header[6..8].copy_from_slice(&self.protocol_minor.to_le_bytes());
        header[8..10].copy_from_slice(&(self.kind as u16).to_le_bytes());
        header[10..12].copy_from_slice(&self.flags.to_le_bytes());
        header[12..16].copy_from_slice(&payload_len.to_le_bytes());
        header[16..24].copy_from_slice(&self.sequence.to_le_bytes());
        header[24..32].copy_from_slice(&self.session_id.to_le_bytes());
        writer.write_all(&header)?;
        writer.write_all(&self.payload)?;
        Ok(())
    }

    /// Decodes exactly one frame from a complete byte slice.
    pub fn decode(bytes: &[u8], max_payload_bytes: usize) -> Result<Self, TransportError> {
        if bytes.len() < TRANSPORT_HEADER_BYTES {
            return Err(TransportError::Truncated);
        }
        let fields = decode_header(&bytes[..TRANSPORT_HEADER_BYTES], max_payload_bytes)?;
        let expected_len = TRANSPORT_HEADER_BYTES
            .checked_add(fields.payload_len)
            .ok_or(TransportError::InvalidField("payload_len"))?;
        if bytes.len() < expected_len {
            return Err(TransportError::Truncated);
        }
        if bytes.len() != expected_len {
            return Err(TransportError::TrailingBytes);
        }
        Ok(fields.into_frame(bytes[TRANSPORT_HEADER_BYTES..].to_vec()))
    }

    /// Reads one frame, returning `None` only for a clean EOF before a header.
    ///
    /// The declared payload length is checked before allocation.
    pub fn read_from<R: Read>(
        reader: &mut R,
        max_payload_bytes: usize,
    ) -> Result<Option<Self>, TransportError> {
        let mut header = [0_u8; TRANSPORT_HEADER_BYTES];
        loop {
            match reader.read(&mut header[..1]) {
                Ok(0) => return Ok(None),
                Ok(1) => break,
                Ok(_) => unreachable!("one-byte read returned more than one byte"),
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) => return Err(error.into()),
            }
        }
        reader
            .read_exact(&mut header[1..])
            .map_err(map_read_exact_error)?;
        let fields = decode_header(&header, max_payload_bytes)?;
        let mut payload = vec![0_u8; fields.payload_len];
        reader
            .read_exact(&mut payload)
            .map_err(map_read_exact_error)?;
        Ok(Some(fields.into_frame(payload)))
    }
}

fn map_read_exact_error(error: io::Error) -> TransportError {
    if error.kind() == io::ErrorKind::UnexpectedEof {
        TransportError::Truncated
    } else {
        TransportError::Io(error)
    }
}

fn validate_payload_len(actual: usize, limit: usize) -> Result<(), TransportError> {
    if actual > limit {
        Err(TransportError::PayloadTooLarge { actual, limit })
    } else {
        Ok(())
    }
}

struct HeaderFields {
    protocol_major: u16,
    protocol_minor: u16,
    kind: TransportMessageKind,
    flags: u16,
    payload_len: usize,
    sequence: u64,
    session_id: u64,
}

impl HeaderFields {
    fn into_frame(self, payload: Vec<u8>) -> TransportFrame {
        TransportFrame {
            protocol_major: self.protocol_major,
            protocol_minor: self.protocol_minor,
            kind: self.kind,
            flags: self.flags,
            sequence: self.sequence,
            session_id: self.session_id,
            payload,
        }
    }
}

fn decode_header(header: &[u8], max_payload_bytes: usize) -> Result<HeaderFields, TransportError> {
    if header.len() != TRANSPORT_HEADER_BYTES {
        return Err(TransportError::Truncated);
    }
    if header[0..4] != TRANSPORT_MAGIC {
        return Err(TransportError::InvalidMagic);
    }
    let protocol_major = u16::from_le_bytes([header[4], header[5]]);
    let protocol_minor = u16::from_le_bytes([header[6], header[7]]);
    let kind = TransportMessageKind::try_from(u16::from_le_bytes([header[8], header[9]]))?;
    let flags = u16::from_le_bytes([header[10], header[11]]);
    let payload_len = u32::from_le_bytes([header[12], header[13], header[14], header[15]]) as usize;
    validate_payload_len(payload_len, max_payload_bytes)?;
    let sequence = u64::from_le_bytes(header[16..24].try_into().expect("fixed header slice"));
    let session_id = u64::from_le_bytes(header[24..32].try_into().expect("fixed header slice"));
    Ok(HeaderFields {
        protocol_major,
        protocol_minor,
        kind,
        flags,
        payload_len,
        sequence,
        session_id,
    })
}

/// Capability bits exchanged during protocol negotiation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransportCapabilities(u64);

impl TransportCapabilities {
    /// Runner-control command and acknowledgement messages.
    pub const CONTROL: Self = Self(1 << 0);
    /// Compact status metadata messages.
    pub const STATUS: Self = Self(1 << 1);
    /// Lossless RGBA8 image messages.
    pub const IMAGE_RGB8: Self = Self(1 << 2);
    /// Linear-depth f32 image messages.
    pub const IMAGE_DEPTH_F32: Self = Self(1 << 3);
    /// LiDAR point-cloud messages.
    pub const LIDAR_POINT_CLOUD: Self = Self(1 << 4);
    /// Gap notices and latest-only reconnect semantics.
    pub const RESUME_LATEST: Self = Self(1 << 5);
    /// All capabilities known by protocol v1.
    pub const ALL_V1: Self = Self((1 << 6) - 1);

    /// Creates a capability set from raw bits, preserving unknown future bits.
    pub const fn from_bits(bits: u64) -> Self {
        Self(bits)
    }

    /// Returns the raw wire representation.
    pub const fn bits(self) -> u64 {
        self.0
    }

    /// Returns true when every bit in `required` is present.
    pub const fn contains(self, required: Self) -> bool {
        self.0 & required.0 == required.0
    }

    /// Returns the intersection of two capability sets.
    pub const fn intersection(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    /// Returns the union of two capability sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// First payload sent by a frontend client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClientHello {
    /// Oldest protocol major accepted by the client.
    pub min_protocol_major: u16,
    /// Newest protocol major accepted by the client.
    pub max_protocol_major: u16,
    /// Capabilities understood by the client.
    pub capabilities: TransportCapabilities,
    /// Capabilities without which the client cannot operate.
    pub required_capabilities: TransportCapabilities,
    /// Largest payload the client will allocate.
    pub max_payload_bytes: u32,
    /// Largest number of queued outbound frames the client permits.
    pub queue_frame_limit: u32,
    /// Largest queued outbound byte budget the client permits.
    pub queue_byte_limit: u32,
    /// Last transport sequence consumed from this session, when reconnecting.
    pub resume_after_sequence: Option<u64>,
}

impl ClientHello {
    /// Encodes the fixed protocol-v1 hello payload.
    pub fn encode_payload(self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(40);
        push_u16(&mut bytes, self.min_protocol_major);
        push_u16(&mut bytes, self.max_protocol_major);
        push_u64(&mut bytes, self.capabilities.bits());
        push_u64(&mut bytes, self.required_capabilities.bits());
        push_u32(&mut bytes, self.max_payload_bytes);
        push_u32(&mut bytes, self.queue_frame_limit);
        push_u32(&mut bytes, self.queue_byte_limit);
        push_u64(&mut bytes, self.resume_after_sequence.unwrap_or(u64::MAX));
        bytes
    }

    /// Decodes a protocol-v1 hello payload.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let hello = Self {
            min_protocol_major: decoder.u16()?,
            max_protocol_major: decoder.u16()?,
            capabilities: TransportCapabilities::from_bits(decoder.u64()?),
            required_capabilities: TransportCapabilities::from_bits(decoder.u64()?),
            max_payload_bytes: decoder.u32()?,
            queue_frame_limit: decoder.u32()?,
            queue_byte_limit: decoder.u32()?,
            resume_after_sequence: match decoder.u64()? {
                u64::MAX => None,
                sequence => Some(sequence),
            },
        };
        decoder.finish()?;
        Ok(hello)
    }
}

/// Server-side range and safety limits used for negotiation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NegotiationPolicy {
    /// Oldest protocol major implemented by the server.
    pub min_protocol_major: u16,
    /// Newest protocol major implemented by the server.
    pub max_protocol_major: u16,
    /// Capabilities implemented by the server.
    pub capabilities: TransportCapabilities,
    /// Server payload safety limit.
    pub max_payload_bytes: u32,
    /// Server queue frame limit.
    pub queue_frame_limit: u32,
    /// Server queue byte limit.
    pub queue_byte_limit: u32,
}

impl Default for NegotiationPolicy {
    fn default() -> Self {
        Self {
            min_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
            max_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
            capabilities: TransportCapabilities::ALL_V1,
            max_payload_bytes: TRANSPORT_MAX_PAYLOAD_BYTES as u32,
            queue_frame_limit: 32,
            queue_byte_limit: 64 * 1024 * 1024,
        }
    }
}

/// Effective protocol and limits selected for one connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NegotiatedTransport {
    /// Selected protocol major.
    pub protocol_major: u16,
    /// Selected protocol minor.
    pub protocol_minor: u16,
    /// Mutually supported capabilities.
    pub capabilities: TransportCapabilities,
    /// Effective maximum payload bytes.
    pub max_payload_bytes: u32,
    /// Effective egress frame limit.
    pub queue_frame_limit: u32,
    /// Effective egress byte limit.
    pub queue_byte_limit: u32,
    /// Client's resume cursor, if this is a reconnect.
    pub resume_after_sequence: Option<u64>,
}

/// Stable wire code explaining a rejected hello.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NegotiationRejectCode {
    /// Client range was internally invalid.
    InvalidVersionRange = 1,
    /// Client and server had no common protocol major.
    UnsupportedVersion = 2,
    /// Client required capability bits it did not advertise.
    InvalidCapabilities = 3,
    /// A required client capability is unavailable.
    RequiredCapabilityUnavailable = 4,
    /// One or more offered limits were zero or unusably small.
    InvalidLimits = 5,
}

impl TryFrom<u16> for NegotiationRejectCode {
    type Error = TransportError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::InvalidVersionRange),
            2 => Ok(Self::UnsupportedVersion),
            3 => Ok(Self::InvalidCapabilities),
            4 => Ok(Self::RequiredCapabilityUnavailable),
            5 => Ok(Self::InvalidLimits),
            _ => Err(TransportError::InvalidField("reject_code")),
        }
    }
}

/// Explicit hello rejection suitable for a bounded wire response.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NegotiationReject {
    /// Stable machine-readable reason.
    pub code: NegotiationRejectCode,
    /// Human-readable bounded detail.
    pub message: String,
}

impl NegotiationReject {
    fn new(code: NegotiationRejectCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }

    /// Encodes this rejection with a bounded UTF-8 detail.
    pub fn encode_payload(&self) -> Vec<u8> {
        let message = truncate_utf8(
            &self.message,
            TRANSPORT_MAX_REJECT_MESSAGE_BYTES.min(u16::MAX as usize),
        );
        let mut bytes = Vec::with_capacity(4 + message.len());
        push_u16(&mut bytes, self.code as u16);
        push_u16(&mut bytes, message.len() as u16);
        bytes.extend_from_slice(message.as_bytes());
        bytes
    }

    /// Decodes a bounded rejection payload.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let code = NegotiationRejectCode::try_from(decoder.u16()?)?;
        let len = decoder.u16()? as usize;
        if len > TRANSPORT_MAX_REJECT_MESSAGE_BYTES {
            return Err(TransportError::InvalidField("reject_message_len"));
        }
        let message = std::str::from_utf8(decoder.take(len)?)
            .map_err(|_| TransportError::InvalidUtf8)?
            .to_string();
        decoder.finish()?;
        Ok(Self { code, message })
    }
}

/// Negotiates one client offer against server policy.
pub fn negotiate_transport(
    client: ClientHello,
    policy: NegotiationPolicy,
) -> Result<NegotiatedTransport, NegotiationReject> {
    if client.min_protocol_major == 0
        || client.min_protocol_major > client.max_protocol_major
        || policy.min_protocol_major == 0
        || policy.min_protocol_major > policy.max_protocol_major
    {
        return Err(NegotiationReject::new(
            NegotiationRejectCode::InvalidVersionRange,
            "invalid protocol version range",
        ));
    }
    let overlap_min = client.min_protocol_major.max(policy.min_protocol_major);
    let overlap_max = client.max_protocol_major.min(policy.max_protocol_major);
    if overlap_min > overlap_max {
        return Err(NegotiationReject::new(
            NegotiationRejectCode::UnsupportedVersion,
            "no common protocol major version",
        ));
    }
    if !client.capabilities.contains(client.required_capabilities) {
        return Err(NegotiationReject::new(
            NegotiationRejectCode::InvalidCapabilities,
            "required capabilities were not advertised",
        ));
    }
    let capabilities = client.capabilities.intersection(policy.capabilities);
    if !capabilities.contains(client.required_capabilities) {
        return Err(NegotiationReject::new(
            NegotiationRejectCode::RequiredCapabilityUnavailable,
            "a required capability is unavailable",
        ));
    }
    if client.max_payload_bytes == 0
        || client.queue_frame_limit == 0
        || client.queue_byte_limit <= TRANSPORT_HEADER_BYTES as u32
        || policy.max_payload_bytes == 0
        || policy.queue_frame_limit == 0
        || policy.queue_byte_limit <= TRANSPORT_HEADER_BYTES as u32
    {
        return Err(NegotiationReject::new(
            NegotiationRejectCode::InvalidLimits,
            "payload and queue limits must be non-zero",
        ));
    }
    let queue_byte_limit = client.queue_byte_limit.min(policy.queue_byte_limit);
    let queue_payload_limit = queue_byte_limit - TRANSPORT_HEADER_BYTES as u32;
    Ok(NegotiatedTransport {
        protocol_major: overlap_max,
        protocol_minor: if overlap_max == TRANSPORT_PROTOCOL_MAJOR {
            TRANSPORT_PROTOCOL_MINOR
        } else {
            0
        },
        capabilities,
        max_payload_bytes: client
            .max_payload_bytes
            .min(policy.max_payload_bytes)
            .min(queue_payload_limit),
        queue_frame_limit: client.queue_frame_limit.min(policy.queue_frame_limit),
        queue_byte_limit,
        resume_after_sequence: client.resume_after_sequence,
    })
}

/// Successful negotiation response.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServerHello {
    /// Effective negotiated values.
    pub negotiated: NegotiatedTransport,
    /// Number of prior connections accepted for this run session.
    pub reconnect_generation: u32,
    /// Latest transport sequence assigned by the server.
    pub current_sequence: u64,
    /// Cumulative latest-only messages dropped in this run session.
    pub dropped_messages: u64,
}

impl ServerHello {
    /// Encodes the fixed successful-negotiation payload.
    pub fn encode_payload(self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(52);
        push_u16(&mut bytes, self.negotiated.protocol_major);
        push_u16(&mut bytes, self.negotiated.protocol_minor);
        push_u64(&mut bytes, self.negotiated.capabilities.bits());
        push_u32(&mut bytes, self.negotiated.max_payload_bytes);
        push_u32(&mut bytes, self.negotiated.queue_frame_limit);
        push_u32(&mut bytes, self.negotiated.queue_byte_limit);
        push_u32(&mut bytes, self.reconnect_generation);
        push_u64(&mut bytes, self.current_sequence);
        push_u64(&mut bytes, self.dropped_messages);
        push_u64(
            &mut bytes,
            self.negotiated.resume_after_sequence.unwrap_or(u64::MAX),
        );
        bytes
    }

    /// Decodes a successful-negotiation payload.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let protocol_major = decoder.u16()?;
        let protocol_minor = decoder.u16()?;
        let capabilities = TransportCapabilities::from_bits(decoder.u64()?);
        let max_payload_bytes = decoder.u32()?;
        let queue_frame_limit = decoder.u32()?;
        let queue_byte_limit = decoder.u32()?;
        let reconnect_generation = decoder.u32()?;
        let current_sequence = decoder.u64()?;
        let dropped_messages = decoder.u64()?;
        let resume_after_sequence = match decoder.u64()? {
            u64::MAX => None,
            sequence => Some(sequence),
        };
        let negotiated = NegotiatedTransport {
            protocol_major,
            protocol_minor,
            capabilities,
            max_payload_bytes,
            queue_frame_limit,
            queue_byte_limit,
            resume_after_sequence,
        };
        let result = Self {
            negotiated,
            reconnect_generation,
            current_sequence,
            dropped_messages,
        };
        decoder.finish()?;
        Ok(result)
    }
}

/// Metadata common to every typed sensor payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorFrameMetadata {
    /// Stable DataBus stream id.
    pub stream_id: u64,
    /// Monotonic sequence within the sensor stream.
    pub sensor_sequence: u64,
    /// Capture timestamp in simulation nanosecond ticks.
    pub capture_ticks: u64,
    /// Availability timestamp in simulation nanosecond ticks.
    pub available_ticks: u64,
}

/// Encodes an RGBA8 image with its DataBus metadata.
pub fn encode_image_rgb8(
    metadata: SensorFrameMetadata,
    image: &ImageRgb8,
) -> Result<Vec<u8>, TransportError> {
    let pixel_count = checked_image_elements(image.width, image.height)?;
    let expected = pixel_count
        .checked_mul(4)
        .ok_or(TransportError::InvalidField("rgba8_len"))?;
    if image.rgba8.len() != expected {
        return Err(TransportError::InvalidField("rgba8_len"));
    }
    let payload_len = 44_usize
        .checked_add(expected)
        .ok_or(TransportError::InvalidField("rgba8_len"))?;
    validate_payload_len(payload_len, TRANSPORT_MAX_PAYLOAD_BYTES)?;
    let mut bytes = Vec::with_capacity(payload_len);
    encode_sensor_metadata(&mut bytes, metadata);
    push_u32(&mut bytes, image.width);
    push_u32(&mut bytes, image.height);
    push_u32(&mut bytes, expected as u32);
    bytes.extend_from_slice(&image.rgba8);
    Ok(bytes)
}

/// Decodes an RGBA8 image and DataBus metadata.
pub fn decode_image_rgb8(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, ImageRgb8), TransportError> {
    let mut decoder = Decoder::new(payload);
    let metadata = decode_sensor_metadata(&mut decoder)?;
    let width = decoder.u32()?;
    let height = decoder.u32()?;
    let declared_len = decoder.u32()? as usize;
    let expected = checked_image_elements(width, height)?
        .checked_mul(4)
        .ok_or(TransportError::InvalidField("rgba8_len"))?;
    if declared_len != expected {
        return Err(TransportError::InvalidField("rgba8_len"));
    }
    let rgba8 = decoder.take(expected)?.to_vec();
    decoder.finish()?;
    Ok((metadata, ImageRgb8::from_rgba8(width, height, rgba8)))
}

/// Encodes a linear-depth f32 image with its DataBus metadata.
pub fn encode_image_depth(
    metadata: SensorFrameMetadata,
    image: &ImageDepth,
) -> Result<Vec<u8>, TransportError> {
    let element_count = checked_image_elements(image.width, image.height)?;
    if image.depth_m.len() != element_count || image.depth_m.iter().any(|depth| !depth.is_finite())
    {
        return Err(TransportError::InvalidField("depth_m"));
    }
    let data_bytes = element_count
        .checked_mul(std::mem::size_of::<f32>())
        .ok_or(TransportError::InvalidField("depth_len"))?;
    let payload_len = 44_usize
        .checked_add(data_bytes)
        .ok_or(TransportError::InvalidField("depth_len"))?;
    validate_payload_len(payload_len, TRANSPORT_MAX_PAYLOAD_BYTES)?;
    let mut bytes = Vec::with_capacity(payload_len);
    encode_sensor_metadata(&mut bytes, metadata);
    push_u32(&mut bytes, image.width);
    push_u32(&mut bytes, image.height);
    push_u32(&mut bytes, element_count as u32);
    for depth in &image.depth_m {
        bytes.extend_from_slice(&depth.to_bits().to_le_bytes());
    }
    Ok(bytes)
}

/// Decodes a linear-depth f32 image and DataBus metadata.
pub fn decode_image_depth(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, ImageDepth), TransportError> {
    let mut decoder = Decoder::new(payload);
    let metadata = decode_sensor_metadata(&mut decoder)?;
    let width = decoder.u32()?;
    let height = decoder.u32()?;
    let declared_elements = decoder.u32()? as usize;
    let expected = checked_image_elements(width, height)?;
    if declared_elements != expected || expected > decoder.remaining() / std::mem::size_of::<f32>()
    {
        return Err(TransportError::InvalidField("depth_len"));
    }
    let mut depth_m = Vec::with_capacity(expected);
    for _ in 0..expected {
        let depth = f32::from_bits(decoder.u32()?);
        if !depth.is_finite() {
            return Err(TransportError::InvalidField("depth_m"));
        }
        depth_m.push(depth);
    }
    decoder.finish()?;
    Ok((metadata, ImageDepth::new(width, height, depth_m)))
}

const LIDAR_INTENSITY: u32 = 1 << 0;
const LIDAR_RAY_INDEX: u32 = 1 << 1;
const LIDAR_RETURN_INDEX: u32 = 1 << 2;
const LIDAR_CHANNEL_INDEX: u32 = 1 << 3;
const LIDAR_TIMESTAMP: u32 = 1 << 4;
const LIDAR_KNOWN_ATTRIBUTES: u32 = (1 << 5) - 1;

/// Encodes a LiDAR cloud with aligned optional attributes and DataBus metadata.
pub fn encode_lidar_point_cloud(
    metadata: SensorFrameMetadata,
    cloud: &PointCloud,
) -> Result<Vec<u8>, TransportError> {
    let count = cloud.points_m.len();
    if count > u32::MAX as usize || !cloud.attributes_are_aligned() {
        return Err(TransportError::InvalidField("lidar_attributes"));
    }
    if cloud.points_m.iter().any(|point| !point.is_finite()) {
        return Err(TransportError::InvalidField("points_m"));
    }
    if cloud
        .intensities
        .iter()
        .any(|value| !value.is_finite() || !(0.0..=1.0).contains(value))
    {
        return Err(TransportError::InvalidField("intensities"));
    }
    if cloud
        .timestamps_s
        .iter()
        .any(|value| !value.is_finite() || *value < 0.0)
    {
        return Err(TransportError::InvalidField("timestamps_s"));
    }
    let mut mask = 0_u32;
    if !cloud.intensities.is_empty() {
        mask |= LIDAR_INTENSITY;
    }
    if !cloud.ray_indices.is_empty() {
        mask |= LIDAR_RAY_INDEX;
    }
    if !cloud.return_indices.is_empty() {
        mask |= LIDAR_RETURN_INDEX;
    }
    if !cloud.channel_indices.is_empty() {
        mask |= LIDAR_CHANNEL_INDEX;
    }
    if !cloud.timestamps_s.is_empty() {
        mask |= LIDAR_TIMESTAMP;
    }
    let mut bytes = Vec::new();
    encode_sensor_metadata(&mut bytes, metadata);
    push_u32(&mut bytes, count as u32);
    push_u32(&mut bytes, mask);
    for point in &cloud.points_m {
        push_f64(&mut bytes, point.x);
        push_f64(&mut bytes, point.y);
        push_f64(&mut bytes, point.z);
    }
    for value in &cloud.intensities {
        push_f32(&mut bytes, *value);
    }
    for value in &cloud.ray_indices {
        push_u32(&mut bytes, *value);
    }
    bytes.extend_from_slice(&cloud.return_indices);
    for value in &cloud.channel_indices {
        push_u16(&mut bytes, *value);
    }
    for value in &cloud.timestamps_s {
        push_f64(&mut bytes, *value);
    }
    validate_payload_len(bytes.len(), TRANSPORT_MAX_PAYLOAD_BYTES)?;
    Ok(bytes)
}

/// Decodes a LiDAR cloud and DataBus metadata.
pub fn decode_lidar_point_cloud(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, PointCloud), TransportError> {
    let mut decoder = Decoder::new(payload);
    let metadata = decode_sensor_metadata(&mut decoder)?;
    let count = decoder.u32()? as usize;
    let mask = decoder.u32()?;
    if mask & !LIDAR_KNOWN_ATTRIBUTES != 0 || count > decoder.remaining() / 24 {
        return Err(TransportError::InvalidField("lidar_attributes"));
    }
    let mut cloud = PointCloud::new();
    cloud.points_m.reserve(count);
    for _ in 0..count {
        let point = Vec3::new(decoder.f64()?, decoder.f64()?, decoder.f64()?);
        if !point.is_finite() {
            return Err(TransportError::InvalidField("points_m"));
        }
        cloud.points_m.push(point);
    }
    if mask & LIDAR_INTENSITY != 0 {
        cloud.intensities.reserve(count);
        for _ in 0..count {
            let value = decoder.f32()?;
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err(TransportError::InvalidField("intensities"));
            }
            cloud.intensities.push(value);
        }
    }
    if mask & LIDAR_RAY_INDEX != 0 {
        cloud.ray_indices.reserve(count);
        for _ in 0..count {
            cloud.ray_indices.push(decoder.u32()?);
        }
    }
    if mask & LIDAR_RETURN_INDEX != 0 {
        cloud.return_indices.extend_from_slice(decoder.take(count)?);
    }
    if mask & LIDAR_CHANNEL_INDEX != 0 {
        cloud.channel_indices.reserve(count);
        for _ in 0..count {
            cloud.channel_indices.push(decoder.u16()?);
        }
    }
    if mask & LIDAR_TIMESTAMP != 0 {
        cloud.timestamps_s.reserve(count);
        for _ in 0..count {
            let value = decoder.f64()?;
            if !value.is_finite() || value < 0.0 {
                return Err(TransportError::InvalidField("timestamps_s"));
            }
            cloud.timestamps_s.push(value);
        }
    }
    decoder.finish()?;
    Ok((metadata, cloud))
}

fn checked_image_elements(width: u32, height: u32) -> Result<usize, TransportError> {
    if width == 0 || height == 0 {
        return Err(TransportError::InvalidField("image_dimensions"));
    }
    (width as usize)
        .checked_mul(height as usize)
        .ok_or(TransportError::InvalidField("image_dimensions"))
}

fn encode_sensor_metadata(bytes: &mut Vec<u8>, metadata: SensorFrameMetadata) {
    push_u64(bytes, metadata.stream_id);
    push_u64(bytes, metadata.sensor_sequence);
    push_u64(bytes, metadata.capture_ticks);
    push_u64(bytes, metadata.available_ticks);
}

fn decode_sensor_metadata(
    decoder: &mut Decoder<'_>,
) -> Result<SensorFrameMetadata, TransportError> {
    let metadata = SensorFrameMetadata {
        stream_id: decoder.u64()?,
        sensor_sequence: decoder.u64()?,
        capture_ticks: decoder.u64()?,
        available_ticks: decoder.u64()?,
    };
    if metadata.available_ticks < metadata.capture_ticks {
        return Err(TransportError::InvalidField("available_ticks"));
    }
    Ok(metadata)
}

/// Compact status metadata carried separately from bulk sensor frames.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StatusMessage {
    /// Completed fixed-step count.
    pub step: u64,
    /// Current simulation time in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Runner pause/advance state.
    pub state: RunnerControlState,
    /// Compact JSON state without base64 bulk sensor payloads.
    pub snapshot_json: Vec<u8>,
}

impl StatusMessage {
    /// Encodes status metadata.
    pub fn encode_payload(&self) -> Result<Vec<u8>, TransportError> {
        validate_payload_len(self.snapshot_json.len(), TRANSPORT_MAX_PAYLOAD_BYTES - 24)?;
        let json_len = u32::try_from(self.snapshot_json.len())
            .map_err(|_| TransportError::InvalidField("snapshot_json_len"))?;
        let mut bytes = Vec::with_capacity(24 + self.snapshot_json.len());
        push_u64(&mut bytes, self.step);
        push_u64(&mut bytes, self.sim_time_ticks);
        bytes.push(match self.state {
            RunnerControlState::Running => 0,
            RunnerControlState::Paused => 1,
        });
        bytes.extend_from_slice(&[0, 0, 0]);
        push_u32(&mut bytes, json_len);
        bytes.extend_from_slice(&self.snapshot_json);
        Ok(bytes)
    }

    /// Decodes status metadata.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let step = decoder.u64()?;
        let sim_time_ticks = decoder.u64()?;
        let state = match decoder.u8()? {
            0 => RunnerControlState::Running,
            1 => RunnerControlState::Paused,
            _ => return Err(TransportError::InvalidField("runner_state")),
        };
        if decoder.take(3)? != [0, 0, 0] {
            return Err(TransportError::InvalidField("status_reserved"));
        }
        let len = decoder.u32()? as usize;
        let snapshot_json = decoder.take(len)?.to_vec();
        std::str::from_utf8(&snapshot_json).map_err(|_| TransportError::InvalidUtf8)?;
        decoder.finish()?;
        Ok(Self {
            step,
            sim_time_ticks,
            state,
            snapshot_json,
        })
    }
}

/// Encodes one transport-neutral runner-control command.
pub fn encode_control_command(command: ControlCommand) -> Vec<u8> {
    let (kind, frames) = match command {
        ControlCommand::Pause => (1, 0),
        ControlCommand::Resume => (2, 0),
        ControlCommand::Step { frames } => (3, frames),
        ControlCommand::Reset => (4, 0),
        ControlCommand::Quit => (5, 0),
    };
    let mut bytes = Vec::with_capacity(16);
    bytes.push(kind);
    bytes.extend_from_slice(&[0; 7]);
    push_u64(&mut bytes, frames);
    bytes
}

/// Decodes one transport-neutral runner-control command.
pub fn decode_control_command(payload: &[u8]) -> Result<ControlCommand, TransportError> {
    let mut decoder = Decoder::new(payload);
    let kind = decoder.u8()?;
    if decoder.take(7)? != [0; 7] {
        return Err(TransportError::InvalidField("control_reserved"));
    }
    let frames = decoder.u64()?;
    decoder.finish()?;
    match (kind, frames) {
        (1, 0) => Ok(ControlCommand::Pause),
        (2, 0) => Ok(ControlCommand::Resume),
        (3, frames) => Ok(ControlCommand::Step { frames }),
        (4, 0) => Ok(ControlCommand::Reset),
        (5, 0) => Ok(ControlCommand::Quit),
        _ => Err(TransportError::InvalidField("control_command")),
    }
}

/// Applied state acknowledgement for a queued control command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlAck {
    /// Transport sequence of the accepted command.
    pub command_sequence: u64,
    /// Runner state after accepting the command.
    pub state: RunnerControlState,
}

impl ControlAck {
    /// Encodes this acknowledgement.
    pub fn encode_payload(self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(16);
        push_u64(&mut bytes, self.command_sequence);
        bytes.push(match self.state {
            RunnerControlState::Running => 0,
            RunnerControlState::Paused => 1,
        });
        bytes.extend_from_slice(&[0; 7]);
        bytes
    }

    /// Decodes an acknowledgement.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let command_sequence = decoder.u64()?;
        let state = match decoder.u8()? {
            0 => RunnerControlState::Running,
            1 => RunnerControlState::Paused,
            _ => return Err(TransportError::InvalidField("runner_state")),
        };
        if decoder.take(7)? != [0; 7] {
            return Err(TransportError::InvalidField("control_ack_reserved"));
        }
        decoder.finish()?;
        Ok(Self {
            command_sequence,
            state,
        })
    }
}

/// Cumulative message-loss notice for latest-only delivery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GapNotice {
    /// First transport sequence known to be unavailable.
    pub first_missing_sequence: u64,
    /// Last transport sequence known to be unavailable.
    pub last_missing_sequence: u64,
    /// Cumulative latest-only messages dropped in the session.
    pub total_dropped_messages: u64,
}

impl GapNotice {
    /// Encodes this gap notice.
    pub fn encode_payload(self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(24);
        push_u64(&mut bytes, self.first_missing_sequence);
        push_u64(&mut bytes, self.last_missing_sequence);
        push_u64(&mut bytes, self.total_dropped_messages);
        bytes
    }

    /// Decodes a gap notice.
    pub fn decode_payload(payload: &[u8]) -> Result<Self, TransportError> {
        let mut decoder = Decoder::new(payload);
        let notice = Self {
            first_missing_sequence: decoder.u64()?,
            last_missing_sequence: decoder.u64()?,
            total_dropped_messages: decoder.u64()?,
        };
        decoder.finish()?;
        if notice.first_missing_sequence > notice.last_missing_sequence {
            return Err(TransportError::InvalidField("gap_sequence_range"));
        }
        Ok(notice)
    }
}

/// Stable key used to replace stale latest-only messages.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EgressKey {
    /// Compact runner status.
    Status,
    /// Latest payload for one stable sensor stream and wire kind.
    Sensor {
        /// DataBus stream id.
        stream_id: u64,
        /// Typed sensor payload kind.
        kind: TransportMessageKind,
    },
    /// Latest cumulative gap notice.
    Gap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeliveryClass {
    Reliable,
    LatestOnly(EgressKey),
}

#[derive(Clone, Debug)]
struct QueuedFrame {
    frame: TransportFrame,
    class: DeliveryClass,
}

/// Invalid bounded queue configuration.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[error("egress queue limits must allow at least one transport header")]
pub struct InvalidQueueLimits;

/// A reliable frame could not fit without dropping another reliable frame.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[error("reliable frame cannot fit in bounded egress queue")]
pub struct EgressQueueFull {
    /// Encoded bytes required by the rejected frame.
    pub frame_bytes: usize,
    /// Frames already queued.
    pub queued_frames: usize,
    /// Bytes already queued.
    pub queued_bytes: usize,
}

/// Result of enqueueing a latest-only message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatestPushOutcome {
    /// Message was queued without evicting another message.
    Enqueued,
    /// Message was queued after replacing or evicting `dropped` stale messages.
    EnqueuedAfterDrops {
        /// Number of stale messages removed by this push.
        dropped: u64,
    },
    /// Incoming message itself could not fit and was dropped.
    Dropped,
}

/// Sequence range removed by the most recent latest-only queue push.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DroppedSequenceRange {
    /// Smallest sequence removed by the push.
    pub first_sequence: u64,
    /// Largest sequence removed by the push.
    pub last_sequence: u64,
    /// Number of messages removed, which may exceed the numeric span when
    /// transport sequences for reliable messages occur between drops.
    pub count: u64,
}

/// Non-blocking frame+byte bounded egress queue.
///
/// Reliable messages are never evicted. Latest-only messages replace older
/// messages with the same key and may evict the oldest other latest-only
/// message. Every eviction is counted deterministically.
#[derive(Debug)]
pub struct BoundedEgressQueue {
    frames: VecDeque<QueuedFrame>,
    max_frames: usize,
    max_bytes: usize,
    queued_bytes: usize,
    dropped_messages: u64,
    last_dropped_range: Option<DroppedSequenceRange>,
}

impl BoundedEgressQueue {
    /// Creates a queue with explicit frame and encoded-byte limits.
    pub fn new(max_frames: usize, max_bytes: usize) -> Result<Self, InvalidQueueLimits> {
        if max_frames == 0 || max_bytes < TRANSPORT_HEADER_BYTES {
            return Err(InvalidQueueLimits);
        }
        Ok(Self {
            frames: VecDeque::new(),
            max_frames,
            max_bytes,
            queued_bytes: 0,
            dropped_messages: 0,
            last_dropped_range: None,
        })
    }

    /// Pushes a reliable message or returns without mutating the queue.
    pub fn push_reliable(&mut self, frame: TransportFrame) -> Result<(), EgressQueueFull> {
        let frame_bytes = frame.encoded_len();
        if !self.can_fit(frame_bytes) {
            return Err(EgressQueueFull {
                frame_bytes,
                queued_frames: self.frames.len(),
                queued_bytes: self.queued_bytes,
            });
        }
        self.queued_bytes += frame_bytes;
        self.frames.push_back(QueuedFrame {
            frame,
            class: DeliveryClass::Reliable,
        });
        Ok(())
    }

    /// Pushes a latest-only message without blocking.
    pub fn push_latest(&mut self, key: EgressKey, frame: TransportFrame) -> LatestPushOutcome {
        self.last_dropped_range = None;
        let frame_bytes = frame.encoded_len();
        if frame_bytes > self.max_bytes {
            self.record_drop(frame.sequence);
            return LatestPushOutcome::Dropped;
        }
        let mut dropped = 0_u64;
        if let Some(index) = self.frames.iter().position(
            |queued| matches!(queued.class, DeliveryClass::LatestOnly(existing) if existing == key),
        ) {
            let sequence = self.remove(index).expect("matched queue index exists");
            self.record_drop(sequence);
            dropped += 1;
        }
        while !self.can_fit(frame_bytes) {
            let Some(index) = self
                .frames
                .iter()
                .position(|queued| matches!(queued.class, DeliveryClass::LatestOnly(_)))
            else {
                self.record_drop(frame.sequence);
                return LatestPushOutcome::Dropped;
            };
            let sequence = self.remove(index).expect("matched queue index exists");
            self.record_drop(sequence);
            dropped += 1;
        }
        self.queued_bytes += frame_bytes;
        self.frames.push_back(QueuedFrame {
            frame,
            class: DeliveryClass::LatestOnly(key),
        });
        if dropped == 0 {
            LatestPushOutcome::Enqueued
        } else {
            LatestPushOutcome::EnqueuedAfterDrops { dropped }
        }
    }

    /// Removes and returns the oldest queued frame.
    pub fn pop_front(&mut self) -> Option<TransportFrame> {
        let queued = self.frames.pop_front()?;
        self.queued_bytes = self.queued_bytes.saturating_sub(queued.frame.encoded_len());
        Some(queued.frame)
    }

    /// Number of queued frames.
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// Returns true when no frame is queued.
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Encoded bytes currently retained.
    pub fn queued_bytes(&self) -> usize {
        self.queued_bytes
    }

    /// Cumulative latest-only messages dropped or replaced.
    pub fn dropped_messages(&self) -> u64 {
        self.dropped_messages
    }

    /// Returns and clears the sequence range dropped by the latest push.
    pub fn take_last_dropped_range(&mut self) -> Option<DroppedSequenceRange> {
        self.last_dropped_range.take()
    }

    /// Configured maximum frame count.
    pub fn max_frames(&self) -> usize {
        self.max_frames
    }

    /// Configured maximum encoded-byte budget.
    pub fn max_bytes(&self) -> usize {
        self.max_bytes
    }

    fn can_fit(&self, frame_bytes: usize) -> bool {
        self.frames.len() < self.max_frames
            && self
                .queued_bytes
                .checked_add(frame_bytes)
                .is_some_and(|bytes| bytes <= self.max_bytes)
    }

    fn remove(&mut self, index: usize) -> Option<u64> {
        if let Some(queued) = self.frames.remove(index) {
            self.queued_bytes = self.queued_bytes.saturating_sub(queued.frame.encoded_len());
            Some(queued.frame.sequence)
        } else {
            None
        }
    }

    fn record_drop(&mut self, sequence: u64) {
        self.dropped_messages = self.dropped_messages.saturating_add(1);
        match &mut self.last_dropped_range {
            Some(range) => {
                range.first_sequence = range.first_sequence.min(sequence);
                range.last_sequence = range.last_sequence.max(sequence);
                range.count = range.count.saturating_add(1);
            }
            None => {
                self.last_dropped_range = Some(DroppedSequenceRange {
                    first_sequence: sequence,
                    last_sequence: sequence,
                    count: 1,
                });
            }
        }
    }
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_f32(bytes: &mut Vec<u8>, value: f32) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

fn push_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

fn truncate_utf8(text: &str, max_bytes: usize) -> &str {
    let mut end = text.len().min(max_bytes);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    &text[..end]
}

struct Decoder<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Decoder<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.offset)
    }

    fn take(&mut self, len: usize) -> Result<&'a [u8], TransportError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or(TransportError::Truncated)?;
        let value = self
            .bytes
            .get(self.offset..end)
            .ok_or(TransportError::Truncated)?;
        self.offset = end;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, TransportError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, TransportError> {
        Ok(u16::from_le_bytes(
            self.take(2)?.try_into().expect("two-byte slice"),
        ))
    }

    fn u32(&mut self) -> Result<u32, TransportError> {
        Ok(u32::from_le_bytes(
            self.take(4)?.try_into().expect("four-byte slice"),
        ))
    }

    fn u64(&mut self) -> Result<u64, TransportError> {
        Ok(u64::from_le_bytes(
            self.take(8)?.try_into().expect("eight-byte slice"),
        ))
    }

    fn f32(&mut self) -> Result<f32, TransportError> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn f64(&mut self) -> Result<f64, TransportError> {
        Ok(f64::from_bits(self.u64()?))
    }

    fn finish(self) -> Result<(), TransportError> {
        if self.offset == self.bytes.len() {
            Ok(())
        } else {
            Err(TransportError::TrailingBytes)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metadata() -> SensorFrameMetadata {
        SensorFrameMetadata {
            stream_id: 7,
            sensor_sequence: 11,
            capture_ticks: 20,
            available_ticks: 25,
        }
    }

    #[test]
    fn frame_header_has_platform_independent_golden_bytes() {
        let frame = TransportFrame {
            protocol_major: 1,
            protocol_minor: 2,
            kind: TransportMessageKind::Status,
            flags: 0x0304,
            sequence: 0x0102_0304_0506_0708,
            session_id: 0x1112_1314_1516_1718,
            payload: vec![0xaa, 0xbb],
        };
        let bytes = frame.encode().expect("encode");
        assert_eq!(
            &bytes[..TRANSPORT_HEADER_BYTES],
            &[
                0x52, 0x4e, 0x45, 0x46, 0x01, 0x00, 0x02, 0x00, 0x06, 0x00, 0x04, 0x03, 0x02, 0x00,
                0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x18, 0x17, 0x16, 0x15,
                0x14, 0x13, 0x12, 0x11,
            ]
        );
        assert_eq!(TransportFrame::decode(&bytes, 16).unwrap(), frame);
    }

    #[test]
    fn reader_rejects_declared_length_before_allocating() {
        let frame = TransportFrame::new(TransportMessageKind::Status, 1, 2, vec![0; 8]);
        let mut bytes = frame.encode().unwrap();
        bytes[12..16].copy_from_slice(&1024_u32.to_le_bytes());
        let error = TransportFrame::read_from(&mut bytes.as_slice(), 16).unwrap_err();
        assert!(matches!(
            error,
            TransportError::PayloadTooLarge {
                actual: 1024,
                limit: 16
            }
        ));
    }

    #[test]
    fn clean_eof_is_distinct_from_truncation() {
        assert!(TransportFrame::read_from(&mut [].as_slice(), 16)
            .unwrap()
            .is_none());
        assert!(matches!(
            TransportFrame::read_from(&mut [b'R'].as_slice(), 16),
            Err(TransportError::Truncated)
        ));
    }

    #[test]
    fn client_and_server_hello_round_trip() {
        let client = ClientHello {
            min_protocol_major: 1,
            max_protocol_major: 2,
            capabilities: TransportCapabilities::ALL_V1,
            required_capabilities: TransportCapabilities::CONTROL,
            max_payload_bytes: 4096,
            queue_frame_limit: 8,
            queue_byte_limit: 16_384,
            resume_after_sequence: Some(55),
        };
        assert_eq!(
            ClientHello::decode_payload(&client.encode_payload()).unwrap(),
            client
        );
        let negotiated = negotiate_transport(client, NegotiationPolicy::default()).unwrap();
        assert_eq!(negotiated.protocol_major, 1);
        assert_eq!(negotiated.max_payload_bytes, 4096);
        assert_eq!(negotiated.queue_frame_limit, 8);
        assert_eq!(negotiated.resume_after_sequence, Some(55));

        let server = ServerHello {
            negotiated,
            reconnect_generation: 2,
            current_sequence: 99,
            dropped_messages: 4,
        };
        assert_eq!(
            ServerHello::decode_payload(&server.encode_payload()).unwrap(),
            server
        );
    }

    #[test]
    fn negotiation_rejects_incompatible_or_missing_capabilities() {
        let incompatible = ClientHello {
            min_protocol_major: 2,
            max_protocol_major: 3,
            capabilities: TransportCapabilities::ALL_V1,
            required_capabilities: TransportCapabilities::CONTROL,
            max_payload_bytes: 1024,
            queue_frame_limit: 2,
            queue_byte_limit: 2048,
            resume_after_sequence: None,
        };
        assert_eq!(
            negotiate_transport(incompatible, NegotiationPolicy::default())
                .unwrap_err()
                .code,
            NegotiationRejectCode::UnsupportedVersion
        );

        let missing = ClientHello {
            min_protocol_major: 1,
            max_protocol_major: 1,
            capabilities: TransportCapabilities::CONTROL,
            required_capabilities: TransportCapabilities::IMAGE_RGB8,
            ..incompatible
        };
        assert_eq!(
            negotiate_transport(missing, NegotiationPolicy::default())
                .unwrap_err()
                .code,
            NegotiationRejectCode::InvalidCapabilities
        );
    }

    #[test]
    fn rejection_text_is_bounded_at_utf8_boundary() {
        let rejection =
            NegotiationReject::new(NegotiationRejectCode::UnsupportedVersion, "界".repeat(1000));
        let payload = rejection.encode_payload();
        let decoded = NegotiationReject::decode_payload(&payload).unwrap();
        assert!(decoded.message.len() <= TRANSPORT_MAX_REJECT_MESSAGE_BYTES);
        assert!(decoded.message.is_char_boundary(decoded.message.len()));
    }

    #[test]
    fn rgba8_round_trip_is_lossless() {
        let image = ImageRgb8::from_rgba8(2, 1, vec![1, 2, 3, 4, 5, 6, 7, 8]);
        let payload = encode_image_rgb8(metadata(), &image).unwrap();
        assert_eq!(decode_image_rgb8(&payload).unwrap(), (metadata(), image));
    }

    #[test]
    fn depth_round_trip_preserves_f32_bits() {
        let image = ImageDepth::new(2, 1, vec![1.25, 9.5]);
        let payload = encode_image_depth(metadata(), &image).unwrap();
        let (decoded_metadata, decoded) = decode_image_depth(&payload).unwrap();
        assert_eq!(decoded_metadata, metadata());
        assert_eq!(decoded.width, image.width);
        assert_eq!(decoded.height, image.height);
        assert_eq!(
            decoded
                .depth_m
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            image
                .depth_m
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn lidar_round_trip_preserves_all_parallel_arrays() {
        let mut cloud = PointCloud::new();
        cloud.push_return(Vec3::new(1.0, 2.0, 3.0), 0.5, 7, 1, 2, 0.01);
        cloud.push_return(Vec3::new(4.0, 5.0, 6.0), 0.75, 8, 2, 3, 0.02);
        let payload = encode_lidar_point_cloud(metadata(), &cloud).unwrap();
        assert_eq!(
            decode_lidar_point_cloud(&payload).unwrap(),
            (metadata(), cloud)
        );
    }

    #[test]
    fn malformed_sensor_lengths_and_non_finite_values_are_rejected() {
        let invalid_rgb = ImageRgb8::from_rgba8(2, 2, vec![0; 4]);
        assert!(matches!(
            encode_image_rgb8(metadata(), &invalid_rgb),
            Err(TransportError::InvalidField("rgba8_len"))
        ));
        let invalid_depth = ImageDepth::new(1, 1, vec![f32::NAN]);
        assert!(matches!(
            encode_image_depth(metadata(), &invalid_depth),
            Err(TransportError::InvalidField("depth_m"))
        ));
        let mut invalid_cloud = PointCloud::new();
        invalid_cloud.points_m.push(Vec3::new(f64::NAN, 0.0, 0.0));
        assert!(matches!(
            encode_lidar_point_cloud(metadata(), &invalid_cloud),
            Err(TransportError::InvalidField("points_m"))
        ));
    }

    #[test]
    fn control_status_ack_and_gap_payloads_round_trip() {
        let command = ControlCommand::Step { frames: 17 };
        assert_eq!(
            decode_control_command(&encode_control_command(command)).unwrap(),
            command
        );
        let status = StatusMessage {
            step: 3,
            sim_time_ticks: 50,
            state: RunnerControlState::Paused,
            snapshot_json: br#"{"base":[1,2,3]}"#.to_vec(),
        };
        assert_eq!(
            StatusMessage::decode_payload(&status.encode_payload().unwrap()).unwrap(),
            status
        );
        let ack = ControlAck {
            command_sequence: 9,
            state: RunnerControlState::Running,
        };
        assert_eq!(
            ControlAck::decode_payload(&ack.encode_payload()).unwrap(),
            ack
        );
        let gap = GapNotice {
            first_missing_sequence: 10,
            last_missing_sequence: 12,
            total_dropped_messages: 5,
        };
        assert_eq!(
            GapNotice::decode_payload(&gap.encode_payload()).unwrap(),
            gap
        );
    }

    fn tiny_frame(sequence: u64, payload_bytes: usize) -> TransportFrame {
        TransportFrame::new(
            TransportMessageKind::Status,
            sequence,
            1,
            vec![0; payload_bytes],
        )
    }

    #[test]
    fn latest_only_queue_replaces_stale_key_with_fixed_bounds() {
        let frame_bytes = tiny_frame(1, 8).encoded_len();
        let mut queue = BoundedEgressQueue::new(2, frame_bytes * 2).unwrap();
        assert_eq!(
            queue.push_latest(EgressKey::Status, tiny_frame(1, 8)),
            LatestPushOutcome::Enqueued
        );
        assert_eq!(
            queue.push_latest(EgressKey::Status, tiny_frame(2, 8)),
            LatestPushOutcome::EnqueuedAfterDrops { dropped: 1 }
        );
        assert_eq!(queue.len(), 1);
        assert_eq!(queue.queued_bytes(), frame_bytes);
        assert_eq!(queue.dropped_messages(), 1);
        assert_eq!(queue.pop_front().unwrap().sequence, 2);
    }

    #[test]
    fn reliable_frames_are_never_evicted_by_latest_only_frames() {
        let frame_bytes = tiny_frame(1, 8).encoded_len();
        let mut queue = BoundedEgressQueue::new(1, frame_bytes).unwrap();
        queue.push_reliable(tiny_frame(1, 8)).unwrap();
        assert_eq!(
            queue.push_latest(EgressKey::Status, tiny_frame(2, 8)),
            LatestPushOutcome::Dropped
        );
        assert_eq!(queue.len(), 1);
        assert_eq!(queue.dropped_messages(), 1);
        assert_eq!(queue.pop_front().unwrap().sequence, 1);
    }

    #[test]
    fn reliable_push_fails_without_mutating_full_queue() {
        let frame_bytes = tiny_frame(1, 8).encoded_len();
        let mut queue = BoundedEgressQueue::new(1, frame_bytes).unwrap();
        queue.push_reliable(tiny_frame(1, 8)).unwrap();
        assert!(queue.push_reliable(tiny_frame(2, 8)).is_err());
        assert_eq!(queue.len(), 1);
        assert_eq!(queue.dropped_messages(), 0);
    }
}
