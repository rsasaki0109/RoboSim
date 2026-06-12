//! In-memory typed publish/subscribe bus.

use crate::frame::{Frame, FramePayload};
use crate::StreamId;
use std::any::{Any, TypeId};
use std::collections::HashMap;
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
    fn latest<T: FramePayload>(&self, stream: StreamId) -> Option<Frame<T>>;

    /// Reads the next frame after the cursor for a stream.
    fn next<T: FramePayload>(
        &self,
        stream: StreamId,
        cursor: &mut SubscriptionCursor,
    ) -> Option<Frame<T>>;
}

struct TypedStream {
    type_id: TypeId,
    frames: Vec<Box<dyn Any + Send + Sync>>,
}

/// In-memory typed DataBus for simulation and tests.
#[derive(Default)]
pub struct InMemoryDataBus {
    streams: HashMap<StreamId, TypedStream>,
}

impl InMemoryDataBus {
    /// Creates an empty bus.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the number of frames stored for a stream.
    pub fn frame_count(&self, stream: StreamId) -> usize {
        self.streams
            .get(&stream)
            .map(|s| s.frames.len())
            .unwrap_or(0)
    }

    fn stream_mut<T: FramePayload>(&mut self, stream: StreamId) -> &mut TypedStream {
        let type_id = TypeId::of::<T>();
        self.streams.entry(stream).or_insert_with(|| TypedStream {
            type_id,
            frames: Vec::new(),
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
        let stream = self.stream_mut::<T>(frame.stream_id);
        debug_assert_eq!(stream.type_id, TypeId::of::<T>());
        stream.frames.push(Box::new(frame));
    }

    fn latest<T: FramePayload>(&self, stream: StreamId) -> Option<Frame<T>> {
        let stream_state = self.stream::<T>(stream).ok()?;
        stream_state
            .frames
            .last()?
            .downcast_ref::<Frame<T>>()
            .cloned()
    }

    fn next<T: FramePayload>(
        &self,
        stream: StreamId,
        cursor: &mut SubscriptionCursor,
    ) -> Option<Frame<T>> {
        let stream_state = self.stream::<T>(stream).ok()?;
        let index = cursor.next_sequence as usize;
        let frame = stream_state
            .frames
            .get(index)?
            .downcast_ref::<Frame<T>>()
            .cloned()?;
        cursor.next_sequence += 1;
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
}
