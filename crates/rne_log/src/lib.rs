//! Simulation log record and replay for Robot Native Engine.

#![deny(missing_docs)]

pub mod record;
pub mod replay;

pub use record::{
    frame_header, LogRecord, ReplayCompatibility, ReplayCompatibilityError, ReplayHeader,
    ReplayRandomSnapshot, ReplayRandomSnapshotError, ReplayRngState, SimulationLog,
    REPLAY_LOG_FORMAT_VERSION, REPLAY_RANDOM_SNAPSHOT_VERSION,
};
pub use replay::{replay_commands, replay_commands_checked};
