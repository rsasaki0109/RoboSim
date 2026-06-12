//! Stream identifiers for typed DataBus channels.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Identifier for a typed data stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct StreamId(pub u64);

impl StreamId {
    /// Creates a stream id from a raw value.
    pub const fn new(id: u64) -> Self {
        Self(id)
    }
}

impl fmt::Display for StreamId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "stream:{}", self.0)
    }
}
