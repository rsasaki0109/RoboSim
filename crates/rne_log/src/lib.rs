//! Simulation log record and replay for Robot Native Engine.

#![deny(missing_docs)]

pub mod artifact;
pub mod record;
pub mod replay;

pub use artifact::{
    ReplayAction, ReplayArtifact, ReplayArtifactError, ReplayClock, ReplayContact,
    ReplayControllerKind, ReplayFailureKind, ReplayFinalReport, ReplayFrame, ReplayJointState,
    ReplayObservation, ReplaySensorPayload, ReplaySensorPayloadData, ReplaySensorStream,
    REPLAY_ARTIFACT_VERSION,
};

pub use record::{
    frame_header, LogRecord, ReplayCompatibility, ReplayCompatibilityError, ReplayHeader,
    ReplayRandomSnapshot, ReplayRandomSnapshotError, ReplayRngState, SimulationLog,
    REPLAY_LOG_FORMAT_VERSION, REPLAY_RANDOM_SNAPSHOT_VERSION,
};
pub use replay::{replay_commands, replay_commands_checked};
