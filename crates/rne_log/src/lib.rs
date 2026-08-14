//! Simulation log record and replay for Robot Native Engine.

#![deny(missing_docs)]

pub mod artifact;
pub mod capsule;
pub mod record;
pub mod replay;

pub use artifact::{
    ReplayAction, ReplayArtifact, ReplayArtifactError, ReplayClock, ReplayContact,
    ReplayControllerKind, ReplayFailureKind, ReplayFinalReport, ReplayFrame, ReplayJointPosition,
    ReplayJointState, ReplayJointVelocity, ReplayObservation, ReplayRobotJointVelocity,
    ReplaySensorPayload, ReplaySensorPayloadData, ReplaySensorStream, REPLAY_ARTIFACT_VERSION,
};
pub use capsule::{
    normalize_relative_path, ArtifactPathError, ArtifactRef, BackendMetadata, BuildMetadata,
    FailureCapsule, FailureCapsuleError, FailureMetadata, MinimizationMetadata, RunMetadata,
    FAILURE_CAPSULE_KIND, FAILURE_CAPSULE_SCHEMA_VERSION, FAILURE_CAPSULE_VERSION,
};

pub use record::{
    frame_header, LogRecord, ReplayCompatibility, ReplayCompatibilityError, ReplayHeader,
    ReplayRandomSnapshot, ReplayRandomSnapshotError, ReplayRngState, SimulationLog,
    REPLAY_LOG_FORMAT_VERSION, REPLAY_RANDOM_SNAPSHOT_VERSION,
};
pub use replay::{replay_commands, replay_commands_checked};
