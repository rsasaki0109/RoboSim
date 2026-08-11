//! In-memory typed publish/subscribe bus.

use crate::frame::{Frame, FramePayload};
use crate::StreamId;
use rne_core::SimTime;
use std::any::{Any, TypeId};
use std::collections::{HashMap, VecDeque};
use std::num::NonZeroUsize;
use thiserror::Error;

/// DataBus publish/subscribe error.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum DataBusError {
    /// Stream does not exist.
    #[error("stream not found")]
    StreamNotFound,
    /// Payload type mismatch for stream.
    #[error("payload type mismatch")]
    TypeMismatch,
    /// A bounded bus requires at least one retained frame per stream.
    #[error("per-stream frame capacity must be greater than zero")]
    InvalidCapacity,
}

/// Cursor for reading frames from a stream in order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SubscriptionCursor {
    next_sequence: u64,
}

impl SubscriptionCursor {
    /// Creates a cursor starting at the given sequence.
    pub const fn at(sequence: u64) -> Self {
        Self {
            next_sequence: sequence,
        }
    }
}

/// Backend-agnostic DataBus interface.
pub trait DataBus {
    /// Publishes a typed frame.
    fn publish<T: FramePayload>(&mut self, frame: Frame<T>);

    /// Returns the latest frame for a stream, if any.
    ///
    /// This ignores [`Frame::available_time`] and therefore sees data the consumer
    /// could not physically have yet. Use it for logging and offline analysis; a
    /// controller in the loop must use [`Self::latest_available`] instead.
    fn latest<T: FramePayload>(&self, stream: StreamId) -> Option<Frame<T>>;

    /// Returns the newest frame whose [`Frame::available_time`] is at or before `now`.
    ///
    /// This is the read a real system performs: a sensor frame exists from its capture
    /// instant but cannot influence a controller until transport and processing latency
    /// have elapsed. Returns `None` when nothing has arrived yet.
    fn latest_available<T: FramePayload>(&self, stream: StreamId, now: SimTime)
        -> Option<Frame<T>>;

    /// Reads the next frame after the cursor for a stream.
    fn next<T: FramePayload>(
        &self,
        stream: StreamId,
        cursor: &mut SubscriptionCursor,
    ) -> Option<Frame<T>>;
}

struct TypedStream {
    type_id: TypeId,
    frames: VecDeque<Box<dyn Any + Send + Sync>>,
    dropped_frames: u64,
}

/// In-memory typed DataBus for simulation and tests.
#[derive(Default)]
pub struct InMemoryDataBus {
    streams: HashMap<StreamId, TypedStream>,
    capacity_per_stream: Option<NonZeroUsize>,
}

impl InMemoryDataBus {
    /// Creates an empty bus.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a bus retaining at most `capacity` newest frames per stream.
    ///
    /// Publishing never blocks. When a stream reaches the limit, its oldest
    /// frame is discarded before the new frame is retained. Subscription
    /// cursors that fall behind resume at the oldest retained sequence.
    pub fn with_capacity_per_stream(capacity: usize) -> Result<Self, DataBusError> {
        let capacity_per_stream =
            NonZeroUsize::new(capacity).ok_or(DataBusError::InvalidCapacity)?;
        Ok(Self {
            streams: HashMap::new(),
            capacity_per_stream: Some(capacity_per_stream),
        })
    }

    /// Returns the number of frames stored for a stream.
    pub fn frame_count(&self, stream: StreamId) -> usize {
        self.streams
            .get(&stream)
            .map(|s| s.frames.len())
            .unwrap_or(0)
    }

    /// Returns the cumulative frames evicted by bounded retention for a stream.
    pub fn dropped_frame_count(&self, stream: StreamId) -> u64 {
        self.streams
            .get(&stream)
            .map(|state| state.dropped_frames)
            .unwrap_or(0)
    }

    fn stream_mut<T: FramePayload>(&mut self, stream: StreamId) -> &mut TypedStream {
        let type_id = TypeId::of::<T>();
        self.streams.entry(stream).or_insert_with(|| TypedStream {
            type_id,
            frames: VecDeque::new(),
            dropped_frames: 0,
        })
    }

    fn stream<T: FramePayload>(&self, stream: StreamId) -> Result<&TypedStream, DataBusError> {
        let stream_state = self
            .streams
            .get(&stream)
            .ok_or(DataBusError::StreamNotFound)?;
        if stream_state.type_id != TypeId::of::<T>() {
            return Err(DataBusError::TypeMismatch);
        }
        Ok(stream_state)
    }
}

impl DataBus for InMemoryDataBus {
    fn publish<T: FramePayload>(&mut self, frame: Frame<T>) {
        let capacity = self.capacity_per_stream;
        let stream = self.stream_mut::<T>(frame.stream_id);
        debug_assert_eq!(stream.type_id, TypeId::of::<T>());
        if capacity.is_some_and(|capacity| stream.frames.len() == capacity.get()) {
            stream.frames.pop_front();
            stream.dropped_frames = stream.dropped_frames.saturating_add(1);
        }
        stream.frames.push_back(Box::new(frame));
    }

    fn latest<T: FramePayload>(&self, stream: StreamId) -> Option<Frame<T>> {
        let stream_state = self.stream::<T>(stream).ok()?;
        stream_state
            .frames
            .back()?
            .downcast_ref::<Frame<T>>()
            .cloned()
    }

    fn latest_available<T: FramePayload>(
        &self,
        stream: StreamId,
        now: SimTime,
    ) -> Option<Frame<T>> {
        let stream_state = self.stream::<T>(stream).ok()?;
        // Frames are published in order, so scan from the newest end for the first
        // one that has arrived.
        stream_state
            .frames
            .iter()
            .rev()
            .filter_map(|frame| frame.downcast_ref::<Frame<T>>())
            .find(|frame| frame.available_time <= now)
            .cloned()
    }

    fn next<T: FramePayload>(
        &self,
        stream: StreamId,
        cursor: &mut SubscriptionCursor,
    ) -> Option<Frame<T>> {
        let stream_state = self.stream::<T>(stream).ok()?;
        let frame = stream_state
            .frames
            .iter()
            .filter_map(|frame| frame.downcast_ref::<Frame<T>>())
            .find(|frame| frame.sequence >= cursor.next_sequence)
            .cloned()?;
        cursor.next_sequence = frame.sequence.saturating_add(1);
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::payloads::ImuSample;
    use rne_core::SimTime;
    use rne_math::Seconds;

    #[test]
    fn publish_subscribe_order() {
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "source");
        let mut bus = InMemoryDataBus::new();
        let stream = StreamId::new(1);

        for sequence in 0..3 {
            bus.publish(Frame::new(
                stream,
                entity,
                sequence,
                SimTime::from_seconds(Seconds::new(sequence as f64 * 0.1)),
                ImuSample::default(),
            ));
        }

        let mut cursor = SubscriptionCursor::default();
        let first = bus.next::<ImuSample>(stream, &mut cursor).unwrap();
        let second = bus.next::<ImuSample>(stream, &mut cursor).unwrap();
        assert_eq!(first.sequence, 0);
        assert_eq!(second.sequence, 1);
        assert_eq!(bus.frame_count(stream), 3);
    }

    #[test]
    fn timestamp_preserved() {
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "source");
        let mut bus = InMemoryDataBus::new();
        let stream = StreamId::new(7);
        let time = SimTime::from_seconds(Seconds::new(0.25));
        bus.publish(Frame::new(stream, entity, 0, time, ImuSample::default()));

        let latest = bus.latest::<ImuSample>(stream).unwrap();
        assert_eq!(latest.sim_time, time);
    }

    #[test]
    fn latest_available_respects_frame_latency() {
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "source");
        let mut bus = InMemoryDataBus::new();
        let stream = StreamId::new(9);
        let latency = rne_core::SimDuration::from_seconds(Seconds::new(0.1));
        for sequence in 0..3_u64 {
            let capture = SimTime::from_seconds(Seconds::new(sequence as f64 * 0.1));
            bus.publish(
                Frame::new(stream, entity, sequence, capture, ImuSample::default())
                    .with_latency(latency),
            );
        }

        // Before anything has arrived there is no readable frame, even though
        // `latest` already sees one.
        let early = SimTime::from_seconds(Seconds::new(0.05));
        assert!(bus.latest_available::<ImuSample>(stream, early).is_none());
        assert!(bus.latest::<ImuSample>(stream).is_some());

        // At 0.25 s the frames captured at 0.0 and 0.1 have arrived; the newest
        // arrived frame wins, not the newest published one.
        let mid = SimTime::from_seconds(Seconds::new(0.25));
        let frame = bus.latest_available::<ImuSample>(stream, mid).unwrap();
        assert_eq!(frame.sequence, 1);

        // Far in the future every frame has arrived.
        let late = SimTime::from_seconds(Seconds::new(10.0));
        let frame = bus.latest_available::<ImuSample>(stream, late).unwrap();
        assert_eq!(frame.sequence, 2);
    }

    #[test]
    fn bounded_retention_keeps_latest_frames_and_counts_evictions() {
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "source");
        let stream = StreamId::new(11);
        let mut bus = InMemoryDataBus::with_capacity_per_stream(2).unwrap();
        for sequence in 0..5 {
            bus.publish(Frame::new(
                stream,
                entity,
                sequence,
                SimTime::from_ticks(sequence),
                ImuSample::default(),
            ));
        }

        assert_eq!(bus.frame_count(stream), 2);
        assert_eq!(bus.dropped_frame_count(stream), 3);
        assert_eq!(bus.latest::<ImuSample>(stream).unwrap().sequence, 4);
    }

    #[test]
    fn lagging_cursor_resumes_at_oldest_retained_sequence() {
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "source");
        let stream = StreamId::new(12);
        let mut bus = InMemoryDataBus::with_capacity_per_stream(2).unwrap();
        for sequence in 0..4 {
            bus.publish(Frame::new(
                stream,
                entity,
                sequence,
                SimTime::from_ticks(sequence),
                ImuSample::default(),
            ));
        }

        let mut cursor = SubscriptionCursor::default();
        assert_eq!(
            bus.next::<ImuSample>(stream, &mut cursor).unwrap().sequence,
            2
        );
        assert_eq!(
            bus.next::<ImuSample>(stream, &mut cursor).unwrap().sequence,
            3
        );
        assert!(bus.next::<ImuSample>(stream, &mut cursor).is_none());
    }

    #[test]
    fn bounded_retention_rejects_zero_capacity() {
        assert!(matches!(
            InMemoryDataBus::with_capacity_per_stream(0),
            Err(DataBusError::InvalidCapacity)
        ));
    }
}
