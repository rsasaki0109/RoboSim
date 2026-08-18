//! Versioned binary payloads used by dataset streams beyond image and LiDAR codecs.

use crate::transport::{SensorFrameMetadata, TransportError, TRANSPORT_MAX_PAYLOAD_BYTES};
use crate::{ImuSample, PoseSample};
use serde::{Deserialize, Serialize};

/// Shared schema version for dataset-native run payload encodings.
pub const DATASET_PAYLOAD_SCHEMA_VERSION: u32 = 1;
/// Dataset IMU payload encoding name.
pub const DATASET_IMU_ENCODING: &str = "rne.dataset.imu.v1";
/// Dataset planar transform payload encoding name.
pub const DATASET_TRANSFORM_ENCODING: &str = "rne.dataset.pose2d.v1";
/// Dataset flat action payload encoding name.
pub const DATASET_ACTION_ENCODING: &str = "rne.dataset.action_f64.v1";
/// Dataset task outcome payload encoding name.
pub const DATASET_TASK_OUTCOME_ENCODING: &str = "rne.dataset.task_outcome.v1";
/// Dataset ground-truth annotation payload encoding name.
pub const DATASET_ANNOTATION_ENCODING: &str = "rne.dataset.ground_truth_f64.v1";

const METADATA_BYTES: usize = 32;
const MAX_VALUES: usize = (TRANSPORT_MAX_PAYLOAD_BYTES - METADATA_BYTES - 16) / 8;

/// Flat action sample in the order declared by the run's ActionSpec.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetActionSample {
    /// Finite action values in semantic ActionSpec row-major order.
    pub values: Vec<f64>,
}

/// Episode transition outcome captured alongside actions and observations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetTaskOutcomeSample {
    /// Deterministic episode index within the lane or run.
    pub episode_index: u64,
    /// Step index within the episode.
    pub step_in_episode: u64,
    /// Scalar reward for this transition.
    pub reward: f64,
    /// Cumulative reward after this transition.
    pub cumulative_reward: f64,
    /// Whether the task termination condition fired.
    pub terminated: bool,
    /// Whether the horizon or external truncation condition fired.
    pub truncated: bool,
    /// Optional semantic success verdict when the task defines one.
    pub success: Option<bool>,
}

/// Numeric ground-truth annotation with stable class and instance identities.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetGroundTruthAnnotation {
    /// Dataset-local class identifier bound by the stream manifest.
    pub class_id: u32,
    /// Stable instance or track identifier.
    pub instance_id: u64,
    /// Finite values in the order and units declared by stream fields.
    pub values: Vec<f64>,
}

/// Encodes an IMU sample with embedded stream/timestamp metadata.
pub fn encode_dataset_imu(
    metadata: SensorFrameMetadata,
    sample: &ImuSample,
) -> Result<Vec<u8>, TransportError> {
    if !sample.angular_velocity_rad_s.is_finite() || !sample.linear_acceleration_m_s2.is_finite() {
        return Err(TransportError::InvalidField("imu_sample"));
    }
    let mut bytes = Vec::with_capacity(METADATA_BYTES + 48);
    encode_metadata(&mut bytes, metadata)?;
    for value in [
        sample.angular_velocity_rad_s.x,
        sample.angular_velocity_rad_s.y,
        sample.angular_velocity_rad_s.z,
        sample.linear_acceleration_m_s2.x,
        sample.linear_acceleration_m_s2.y,
        sample.linear_acceleration_m_s2.z,
    ] {
        push_f64(&mut bytes, value);
    }
    Ok(bytes)
}

/// Decodes and validates an IMU sample and embedded metadata.
pub fn decode_dataset_imu(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, ImuSample), TransportError> {
    let mut decoder = Decoder::new(payload)?;
    let metadata = decoder.metadata()?;
    let angular_velocity_rad_s =
        rne_math::Vec3::new(decoder.f64()?, decoder.f64()?, decoder.f64()?);
    let linear_acceleration_m_s2 =
        rne_math::Vec3::new(decoder.f64()?, decoder.f64()?, decoder.f64()?);
    decoder.finish()?;
    if !angular_velocity_rad_s.is_finite() || !linear_acceleration_m_s2.is_finite() {
        return Err(TransportError::InvalidField("imu_sample"));
    }
    Ok((
        metadata,
        ImuSample {
            angular_velocity_rad_s,
            linear_acceleration_m_s2,
        },
    ))
}

/// Encodes a planar pose/transform sample with embedded metadata.
pub fn encode_dataset_transform(
    metadata: SensorFrameMetadata,
    sample: &PoseSample,
) -> Result<Vec<u8>, TransportError> {
    if !sample.position_m.is_finite() || !sample.yaw_rad.is_finite() {
        return Err(TransportError::InvalidField("transform_sample"));
    }
    let mut bytes = Vec::with_capacity(METADATA_BYTES + 32);
    encode_metadata(&mut bytes, metadata)?;
    for value in [
        sample.position_m.x,
        sample.position_m.y,
        sample.position_m.z,
        sample.yaw_rad,
    ] {
        push_f64(&mut bytes, value);
    }
    Ok(bytes)
}

/// Decodes and validates a planar pose/transform sample and metadata.
pub fn decode_dataset_transform(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, PoseSample), TransportError> {
    let mut decoder = Decoder::new(payload)?;
    let metadata = decoder.metadata()?;
    let position_m = rne_math::Vec3::new(decoder.f64()?, decoder.f64()?, decoder.f64()?);
    let yaw_rad = decoder.f64()?;
    decoder.finish()?;
    if !position_m.is_finite() || !yaw_rad.is_finite() {
        return Err(TransportError::InvalidField("transform_sample"));
    }
    Ok((
        metadata,
        PoseSample {
            position_m,
            yaw_rad,
        },
    ))
}

/// Encodes a finite flat action sample with embedded metadata.
pub fn encode_dataset_action(
    metadata: SensorFrameMetadata,
    sample: &DatasetActionSample,
) -> Result<Vec<u8>, TransportError> {
    validate_values("action_values", &sample.values)?;
    let count = u32::try_from(sample.values.len())
        .map_err(|_| TransportError::InvalidField("action_values"))?;
    let mut bytes = Vec::with_capacity(METADATA_BYTES + 4 + sample.values.len() * 8);
    encode_metadata(&mut bytes, metadata)?;
    push_u32(&mut bytes, count);
    for value in &sample.values {
        push_f64(&mut bytes, *value);
    }
    Ok(bytes)
}

/// Decodes and validates a flat action sample and metadata.
pub fn decode_dataset_action(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, DatasetActionSample), TransportError> {
    let mut decoder = Decoder::new(payload)?;
    let metadata = decoder.metadata()?;
    let values = decoder.f64_values("action_values")?;
    decoder.finish()?;
    Ok((metadata, DatasetActionSample { values }))
}

/// Encodes one finite task outcome with embedded metadata.
pub fn encode_dataset_task_outcome(
    metadata: SensorFrameMetadata,
    sample: &DatasetTaskOutcomeSample,
) -> Result<Vec<u8>, TransportError> {
    if !sample.reward.is_finite() || !sample.cumulative_reward.is_finite() {
        return Err(TransportError::InvalidField("task_outcome"));
    }
    let mut bytes = Vec::with_capacity(METADATA_BYTES + 40);
    encode_metadata(&mut bytes, metadata)?;
    push_u64(&mut bytes, sample.episode_index);
    push_u64(&mut bytes, sample.step_in_episode);
    push_f64(&mut bytes, sample.reward);
    push_f64(&mut bytes, sample.cumulative_reward);
    let mut flags = u8::from(sample.terminated) | (u8::from(sample.truncated) << 1);
    if let Some(success) = sample.success {
        flags |= 1 << 2;
        flags |= u8::from(success) << 3;
    }
    bytes.push(flags);
    bytes.extend_from_slice(&[0_u8; 7]);
    Ok(bytes)
}

/// Decodes and validates one task outcome and embedded metadata.
pub fn decode_dataset_task_outcome(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, DatasetTaskOutcomeSample), TransportError> {
    let mut decoder = Decoder::new(payload)?;
    let metadata = decoder.metadata()?;
    let episode_index = decoder.u64()?;
    let step_in_episode = decoder.u64()?;
    let reward = decoder.f64()?;
    let cumulative_reward = decoder.f64()?;
    let flags = decoder.u8()?;
    if flags & !0x0f != 0 || decoder.take(7)?.iter().any(|byte| *byte != 0) {
        return Err(TransportError::InvalidField("task_outcome_flags"));
    }
    decoder.finish()?;
    if !reward.is_finite() || !cumulative_reward.is_finite() {
        return Err(TransportError::InvalidField("task_outcome"));
    }
    let success = if flags & (1 << 2) == 0 {
        if flags & (1 << 3) != 0 {
            return Err(TransportError::InvalidField("task_outcome_flags"));
        }
        None
    } else {
        Some(flags & (1 << 3) != 0)
    };
    Ok((
        metadata,
        DatasetTaskOutcomeSample {
            episode_index,
            step_in_episode,
            reward,
            cumulative_reward,
            terminated: flags & 1 != 0,
            truncated: flags & (1 << 1) != 0,
            success,
        },
    ))
}

/// Encodes one numeric ground-truth annotation with embedded metadata.
pub fn encode_dataset_annotation(
    metadata: SensorFrameMetadata,
    sample: &DatasetGroundTruthAnnotation,
) -> Result<Vec<u8>, TransportError> {
    validate_values("annotation_values", &sample.values)?;
    let count = u32::try_from(sample.values.len())
        .map_err(|_| TransportError::InvalidField("annotation_values"))?;
    let mut bytes = Vec::with_capacity(METADATA_BYTES + 16 + sample.values.len() * 8);
    encode_metadata(&mut bytes, metadata)?;
    push_u32(&mut bytes, sample.class_id);
    push_u64(&mut bytes, sample.instance_id);
    push_u32(&mut bytes, count);
    for value in &sample.values {
        push_f64(&mut bytes, *value);
    }
    Ok(bytes)
}

/// Decodes and validates one numeric ground-truth annotation and metadata.
pub fn decode_dataset_annotation(
    payload: &[u8],
) -> Result<(SensorFrameMetadata, DatasetGroundTruthAnnotation), TransportError> {
    let mut decoder = Decoder::new(payload)?;
    let metadata = decoder.metadata()?;
    let class_id = decoder.u32()?;
    let instance_id = decoder.u64()?;
    let values = decoder.f64_values("annotation_values")?;
    decoder.finish()?;
    Ok((
        metadata,
        DatasetGroundTruthAnnotation {
            class_id,
            instance_id,
            values,
        },
    ))
}

fn validate_values(field: &'static str, values: &[f64]) -> Result<(), TransportError> {
    if values.is_empty()
        || values.len() > MAX_VALUES
        || values.iter().any(|value| !value.is_finite())
    {
        return Err(TransportError::InvalidField(field));
    }
    Ok(())
}

fn encode_metadata(
    bytes: &mut Vec<u8>,
    metadata: SensorFrameMetadata,
) -> Result<(), TransportError> {
    if metadata.available_ticks < metadata.capture_ticks {
        return Err(TransportError::InvalidField("available_ticks"));
    }
    push_u64(bytes, metadata.stream_id);
    push_u64(bytes, metadata.sensor_sequence);
    push_u64(bytes, metadata.capture_ticks);
    push_u64(bytes, metadata.available_ticks);
    Ok(())
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn push_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

struct Decoder<'a> {
    bytes: &'a [u8],
    cursor: usize,
}

impl<'a> Decoder<'a> {
    fn new(bytes: &'a [u8]) -> Result<Self, TransportError> {
        if bytes.len() > TRANSPORT_MAX_PAYLOAD_BYTES {
            return Err(TransportError::PayloadTooLarge {
                actual: bytes.len(),
                limit: TRANSPORT_MAX_PAYLOAD_BYTES,
            });
        }
        Ok(Self { bytes, cursor: 0 })
    }

    fn metadata(&mut self) -> Result<SensorFrameMetadata, TransportError> {
        let metadata = SensorFrameMetadata {
            stream_id: self.u64()?,
            sensor_sequence: self.u64()?,
            capture_ticks: self.u64()?,
            available_ticks: self.u64()?,
        };
        if metadata.available_ticks < metadata.capture_ticks {
            return Err(TransportError::InvalidField("available_ticks"));
        }
        Ok(metadata)
    }

    fn u8(&mut self) -> Result<u8, TransportError> {
        Ok(self.take(1)?[0])
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

    fn f64(&mut self) -> Result<f64, TransportError> {
        Ok(f64::from_bits(self.u64()?))
    }

    fn f64_values(&mut self, field: &'static str) -> Result<Vec<f64>, TransportError> {
        let count = self.u32()? as usize;
        if count == 0 || count > MAX_VALUES || count > self.remaining() / 8 {
            return Err(TransportError::InvalidField(field));
        }
        let mut values = Vec::with_capacity(count);
        for _ in 0..count {
            values.push(self.f64()?);
        }
        validate_values(field, &values)?;
        Ok(values)
    }

    fn take(&mut self, count: usize) -> Result<&'a [u8], TransportError> {
        let end = self
            .cursor
            .checked_add(count)
            .ok_or(TransportError::Truncated)?;
        let result = self
            .bytes
            .get(self.cursor..end)
            .ok_or(TransportError::Truncated)?;
        self.cursor = end;
        Ok(result)
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.cursor)
    }

    fn finish(self) -> Result<(), TransportError> {
        if self.cursor == self.bytes.len() {
            Ok(())
        } else {
            Err(TransportError::TrailingBytes)
        }
    }
}
