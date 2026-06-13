//! Timestamped simulation frames.

use crate::StreamId;
use rne_core::{SimDuration, SimTime};
use rne_ecs::Entity;
use serde::{Deserialize, Serialize};

/// Marker trait for frame payload types published on the DataBus.
pub trait FramePayload: Clone + Send + Sync + 'static {}

impl FramePayload for crate::payloads::ImuSample {}
impl FramePayload for crate::payloads::PointCloud {}
impl FramePayload for crate::payloads::WheelEncoderSample {}
impl FramePayload for crate::payloads::JointState {}
impl FramePayload for crate::payloads::ImageRgb8 {}

/// Timestamped typed frame published on the DataBus.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame<T: FramePayload> {
    /// Stream identifier.
    pub stream_id: StreamId,
    /// Source entity.
    pub entity: Entity,
    /// Monotonic sequence number within the stream.
    pub sequence: u64,
    /// Simulation time when the sample was captured.
    pub sim_time: SimTime,
    /// Simulation time when the sample was captured before latency.
    pub capture_time: SimTime,
    /// Simulation time when the sample becomes available to consumers.
    pub available_time: SimTime,
    /// Typed payload.
    pub payload: T,
}

impl<T: FramePayload> Frame<T> {
    /// Creates a new frame with identical capture and available timestamps.
    pub fn new(
        stream_id: StreamId,
        entity: Entity,
        sequence: u64,
        sim_time: SimTime,
        payload: T,
    ) -> Self {
        Self {
            stream_id,
            entity,
            sequence,
            sim_time,
            capture_time: sim_time,
            available_time: sim_time,
            payload,
        }
    }

    /// Applies output latency to the frame.
    pub fn with_latency(mut self, latency: SimDuration) -> Self {
        self.available_time = self.capture_time + latency;
        self.sim_time = self.available_time;
        self
    }
}

/// Erased frame metadata for logging and replay.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrameHeader {
    /// Stream identifier.
    pub stream_id: StreamId,
    /// Source entity index.
    pub entity_index: u32,
    /// Sequence number.
    pub sequence: u64,
    /// Capture timestamp ticks.
    pub capture_ticks: u64,
    /// Available timestamp ticks.
    pub available_ticks: u64,
}
