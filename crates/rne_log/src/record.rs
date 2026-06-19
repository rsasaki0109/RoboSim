//! Log recording.

use rne_core::{SimDuration, SimTime, DETERMINISTIC_RNG_VERSION, KEYED_RANDOM_VERSION};
use rne_data::{FrameHeader, ImuSample, PointCloud, StreamId, WheelEncoderSample};
use rne_robot::ActuatorCommandEntry;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;
use thiserror::Error;

/// Replay log schema version.
pub const REPLAY_LOG_FORMAT_VERSION: u32 = 1;

/// Replay random snapshot schema version.
pub const REPLAY_RANDOM_SNAPSHOT_VERSION: u32 = 1;

/// Log serialization error.
#[derive(Debug, Error)]
pub enum LogError {
    /// IO failure.
    #[error(transparent)]
    Io(#[from] io::Error),
    /// JSON failure.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Replay compatibility validation failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReplayCompatibilityError {
    /// The log does not contain replay metadata.
    #[error("replay log is missing a replay header")]
    MissingHeader,
    /// A deterministic compatibility field does not match the expected value.
    #[error("replay header mismatch for {field}: expected {expected}, got {actual}")]
    Mismatch {
        /// Field name that did not match.
        field: &'static str,
        /// Expected value.
        expected: String,
        /// Actual value from the replay header.
        actual: String,
    },
}

/// Replay random snapshot validation failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReplayRandomSnapshotError {
    /// The snapshot payload schema is not supported by this engine.
    #[error(
        "unsupported replay random snapshot schema version: expected {expected}, got {actual}"
    )]
    UnsupportedSchemaVersion {
        /// Expected snapshot schema version.
        expected: u32,
        /// Actual snapshot schema version.
        actual: u32,
    },
    /// A deterministic snapshot field does not match the current replay target.
    #[error("replay random snapshot mismatch for {field}: expected {expected}, got {actual}")]
    Mismatch {
        /// Field name that did not match.
        field: &'static str,
        /// Expected value.
        expected: String,
        /// Actual value from the replay random snapshot.
        actual: String,
    },
    /// A named RNG state required by the replay target was not recorded.
    #[error("replay random snapshot is missing RNG state {name}")]
    MissingRngState {
        /// Missing producer-defined stream name.
        name: String,
    },
}

/// Determinism fields that must match before replaying a log.
///
/// The producer engine version is recorded in [`ReplayHeader`] for audit and
/// debugging, but compatibility is determined by the explicit format,
/// algorithm, stream derivation, world seed, and timestep versions here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplayCompatibility {
    /// Expected replay log schema version.
    pub log_format_version: u32,
    /// Expected root world seed.
    pub world_seed: u64,
    /// Expected deterministic RNG algorithm version.
    pub rng_algorithm_version: u32,
    /// Expected stateless keyed random algorithm version.
    pub keyed_random_version: u32,
    /// Expected world random stream derivation version.
    pub stream_derivation_version: u32,
    /// Expected fixed simulation timestep in nanosecond ticks.
    pub fixed_delta_ticks: u64,
}

impl ReplayCompatibility {
    /// Creates expected replay compatibility metadata for the current engine.
    pub fn current(
        world_seed: u64,
        stream_derivation_version: u32,
        fixed_delta: SimDuration,
    ) -> Self {
        Self {
            log_format_version: REPLAY_LOG_FORMAT_VERSION,
            world_seed,
            rng_algorithm_version: DETERMINISTIC_RNG_VERSION,
            keyed_random_version: KEYED_RANDOM_VERSION,
            stream_derivation_version,
            fixed_delta_ticks: fixed_delta.ticks(),
        }
    }

    fn validate_actual(&self, actual: Self) -> Result<(), ReplayCompatibilityError> {
        check_match(
            "log_format_version",
            self.log_format_version,
            actual.log_format_version,
        )?;
        check_match("world_seed", self.world_seed, actual.world_seed)?;
        check_match(
            "rng_algorithm_version",
            self.rng_algorithm_version,
            actual.rng_algorithm_version,
        )?;
        check_match(
            "keyed_random_version",
            self.keyed_random_version,
            actual.keyed_random_version,
        )?;
        check_match(
            "stream_derivation_version",
            self.stream_derivation_version,
            actual.stream_derivation_version,
        )?;
        check_match(
            "fixed_delta_ticks",
            self.fixed_delta_ticks,
            actual.fixed_delta_ticks,
        )?;
        Ok(())
    }
}

/// Replay metadata required to validate deterministic playback compatibility.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplayHeader {
    /// Version of the JSONL replay log schema.
    pub log_format_version: u32,
    /// Engine version that produced the log.
    pub engine_version: String,
    /// Root world seed used for deterministic random behavior.
    pub world_seed: u64,
    /// Deterministic RNG algorithm version used by the run.
    pub rng_algorithm_version: u32,
    /// Stateless keyed random algorithm version used by the run.
    pub keyed_random_version: u32,
    /// World random stream derivation version used by the run.
    pub stream_derivation_version: u32,
    /// Fixed simulation timestep in nanosecond ticks.
    pub fixed_delta_ticks: u64,
}

impl ReplayHeader {
    /// Creates replay metadata for the current engine and RNG algorithm.
    pub fn new(world_seed: u64, stream_derivation_version: u32, fixed_delta: SimDuration) -> Self {
        Self {
            log_format_version: REPLAY_LOG_FORMAT_VERSION,
            engine_version: env!("CARGO_PKG_VERSION").to_string(),
            world_seed,
            rng_algorithm_version: DETERMINISTIC_RNG_VERSION,
            keyed_random_version: KEYED_RANDOM_VERSION,
            stream_derivation_version,
            fixed_delta_ticks: fixed_delta.ticks(),
        }
    }

    /// Returns the deterministic compatibility fields for this header.
    pub const fn compatibility(&self) -> ReplayCompatibility {
        ReplayCompatibility {
            log_format_version: self.log_format_version,
            world_seed: self.world_seed,
            rng_algorithm_version: self.rng_algorithm_version,
            keyed_random_version: self.keyed_random_version,
            stream_derivation_version: self.stream_derivation_version,
            fixed_delta_ticks: self.fixed_delta_ticks,
        }
    }

    /// Validates this header against expected deterministic compatibility fields.
    pub fn validate_compatibility(
        &self,
        expected: &ReplayCompatibility,
    ) -> Result<(), ReplayCompatibilityError> {
        expected.validate_actual(self.compatibility())
    }
}

/// Named deterministic RNG state stored in a replay random snapshot.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplayRngState {
    /// Stable producer-defined stream name.
    pub name: String,
    /// Internal RNG state value needed to continue the stream.
    pub state: u64,
}

impl ReplayRngState {
    /// Creates a named RNG state entry.
    pub fn new(name: impl Into<String>, state: u64) -> Self {
        Self {
            name: name.into(),
            state,
        }
    }
}

/// Replay checkpoint for deterministic random state.
///
/// This is not a complete physics or ECS world snapshot. It captures the root
/// world random stream plus producer-owned deterministic RNG streams that must
/// be restored alongside a separately persisted simulation state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplayRandomSnapshot {
    /// Version of this snapshot payload schema.
    pub schema_version: u32,
    /// Simulation time of the snapshot in nanosecond ticks.
    pub sim_ticks: u64,
    /// Producer-defined monotonic snapshot sequence.
    pub sequence: u64,
    /// Root world seed used to derive deterministic streams.
    pub world_seed: u64,
    /// Current internal state of the main world random stream.
    pub world_main_rng_state: u64,
    /// Additional named RNG states owned by sensors, agents, or episodes.
    pub rng_states: Vec<ReplayRngState>,
}

impl ReplayRandomSnapshot {
    /// Creates an empty replay random snapshot at the given simulation time.
    pub fn new(
        sim_time: SimTime,
        sequence: u64,
        world_seed: u64,
        world_main_rng_state: u64,
    ) -> Self {
        Self {
            schema_version: REPLAY_RANDOM_SNAPSHOT_VERSION,
            sim_ticks: sim_time.ticks(),
            sequence,
            world_seed,
            world_main_rng_state,
            rng_states: Vec::new(),
        }
    }

    /// Appends one named RNG state and returns the updated snapshot.
    pub fn with_rng_state(mut self, rng_state: ReplayRngState) -> Self {
        self.rng_states.push(rng_state);
        self
    }

    /// Appends one named RNG state.
    pub fn push_rng_state(&mut self, rng_state: ReplayRngState) {
        self.rng_states.push(rng_state);
    }

    /// Validates that this snapshot uses the current payload schema.
    pub fn validate_current_schema(&self) -> Result<(), ReplayRandomSnapshotError> {
        if self.schema_version == REPLAY_RANDOM_SNAPSHOT_VERSION {
            Ok(())
        } else {
            Err(ReplayRandomSnapshotError::UnsupportedSchemaVersion {
                expected: REPLAY_RANDOM_SNAPSHOT_VERSION,
                actual: self.schema_version,
            })
        }
    }

    /// Validates that this snapshot was produced for the expected world seed.
    pub fn validate_world_seed(
        &self,
        expected_world_seed: u64,
    ) -> Result<(), ReplayRandomSnapshotError> {
        check_snapshot_match("world_seed", expected_world_seed, self.world_seed)
    }

    /// Returns the state for a named RNG stream, if present.
    pub fn rng_state(&self, name: &str) -> Option<u64> {
        self.rng_states
            .iter()
            .find(|state| state.name == name)
            .map(|state| state.state)
    }

    /// Returns the state for a named RNG stream, or an error if it is absent.
    pub fn require_rng_state(&self, name: &str) -> Result<u64, ReplayRandomSnapshotError> {
        self.rng_state(name)
            .ok_or_else(|| ReplayRandomSnapshotError::MissingRngState {
                name: name.to_string(),
            })
    }
}

/// One JSONL log record.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum LogRecord {
    /// Replay compatibility metadata.
    ReplayHeader {
        /// Replay header payload.
        header: ReplayHeader,
    },
    /// Replay random state checkpoint.
    ReplayRandomSnapshot {
        /// Replay random snapshot payload.
        snapshot: ReplayRandomSnapshot,
    },
    /// Actuator command entry.
    ActuatorCommand {
        /// Simulation time ticks.
        sim_ticks: u64,
        /// Command sequence.
        sequence: u64,
        /// Wheel actuator entity index.
        wheel_index: Option<u32>,
        /// Wheel velocity in radians per second.
        velocity_rad_s: Option<f64>,
    },
    /// IMU frame metadata and payload.
    ImuFrame {
        /// Frame header.
        header: FrameHeader,
        /// Payload.
        payload: ImuSample,
    },
    /// LiDAR frame metadata and payload.
    LidarFrame {
        /// Frame header.
        header: FrameHeader,
        /// Payload.
        payload: PointCloud,
    },
    /// Wheel encoder frame metadata and payload.
    WheelEncoderFrame {
        /// Frame header.
        header: FrameHeader,
        /// Payload.
        payload: WheelEncoderSample,
    },
}

/// Append-only simulation log writer.
#[derive(Debug, Default)]
pub struct SimulationLog {
    header: Option<ReplayHeader>,
    records: Vec<LogRecord>,
}

impl SimulationLog {
    /// Creates an empty log.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates an empty log with replay metadata.
    pub fn new_with_header(header: ReplayHeader) -> Self {
        Self {
            header: Some(header),
            records: Vec::new(),
        }
    }

    /// Returns replay metadata when the log was created with a header.
    pub fn header(&self) -> Option<&ReplayHeader> {
        self.header.as_ref()
    }

    /// Replaces replay metadata for this log.
    pub fn set_header(&mut self, header: ReplayHeader) {
        self.header = Some(header);
    }

    /// Validates this log's replay header before deterministic playback.
    pub fn validate_compatibility(
        &self,
        expected: &ReplayCompatibility,
    ) -> Result<(), ReplayCompatibilityError> {
        let header = self
            .header()
            .ok_or(ReplayCompatibilityError::MissingHeader)?;
        header.validate_compatibility(expected)
    }

    /// Records an actuator command entry.
    pub fn record_actuator_command(&mut self, entry: &ActuatorCommandEntry) {
        let (wheel_index, velocity_rad_s) = match &entry.command {
            rne_robot::ActuatorCommand::WheelVelocity {
                wheel,
                velocity_rad_s,
            } => (Some(wheel.index()), Some(*velocity_rad_s)),
            _ => (None, None),
        };

        self.records.push(LogRecord::ActuatorCommand {
            sim_ticks: entry.sim_time.ticks(),
            sequence: entry.sequence,
            wheel_index,
            velocity_rad_s,
        });
    }

    /// Records a replay random state snapshot.
    pub fn record_random_snapshot(&mut self, snapshot: ReplayRandomSnapshot) {
        self.records
            .push(LogRecord::ReplayRandomSnapshot { snapshot });
    }

    /// Records an IMU frame.
    pub fn record_imu(&mut self, header: FrameHeader, payload: ImuSample) {
        self.records.push(LogRecord::ImuFrame { header, payload });
    }

    /// Records a LiDAR frame.
    pub fn record_lidar(&mut self, header: FrameHeader, payload: PointCloud) {
        self.records.push(LogRecord::LidarFrame { header, payload });
    }

    /// Records a wheel encoder frame.
    pub fn record_wheel_encoder(&mut self, header: FrameHeader, payload: WheelEncoderSample) {
        self.records
            .push(LogRecord::WheelEncoderFrame { header, payload });
    }

    /// Returns all stored records.
    pub fn records(&self) -> &[LogRecord] {
        &self.records
    }

    /// Returns replay random state snapshots in record order.
    pub fn random_snapshots(&self) -> impl Iterator<Item = &ReplayRandomSnapshot> {
        self.records.iter().filter_map(|record| match record {
            LogRecord::ReplayRandomSnapshot { snapshot } => Some(snapshot),
            _ => None,
        })
    }

    /// Returns the latest replay random state snapshot, if one was recorded.
    pub fn latest_random_snapshot(&self) -> Option<&ReplayRandomSnapshot> {
        self.records.iter().rev().find_map(|record| match record {
            LogRecord::ReplayRandomSnapshot { snapshot } => Some(snapshot),
            _ => None,
        })
    }

    /// Writes the log to a JSONL file.
    pub fn write_jsonl(&self, path: impl AsRef<Path>) -> Result<(), LogError> {
        let mut file = File::create(path)?;
        if let Some(header) = &self.header {
            serde_json::to_writer(
                &mut file,
                &LogRecord::ReplayHeader {
                    header: header.clone(),
                },
            )?;
            file.write_all(b"\n")?;
        }
        for record in &self.records {
            serde_json::to_writer(&mut file, record)?;
            file.write_all(b"\n")?;
        }
        Ok(())
    }

    /// Loads a log from a JSONL file.
    pub fn read_jsonl(path: impl AsRef<Path>) -> Result<Self, LogError> {
        let content = std::fs::read_to_string(path)?;
        let mut header = None;
        let mut records = Vec::new();
        for line in content.lines().filter(|line| !line.trim().is_empty()) {
            let record = serde_json::from_str(line)?;
            match record {
                LogRecord::ReplayHeader { header: parsed } => header = Some(parsed),
                other => records.push(other),
            }
        }
        Ok(Self { header, records })
    }
}

fn check_match<T>(
    field: &'static str,
    expected: T,
    actual: T,
) -> Result<(), ReplayCompatibilityError>
where
    T: Eq + ToString,
{
    if expected == actual {
        Ok(())
    } else {
        Err(ReplayCompatibilityError::Mismatch {
            field,
            expected: expected.to_string(),
            actual: actual.to_string(),
        })
    }
}

fn check_snapshot_match<T>(
    field: &'static str,
    expected: T,
    actual: T,
) -> Result<(), ReplayRandomSnapshotError>
where
    T: Eq + ToString,
{
    if expected == actual {
        Ok(())
    } else {
        Err(ReplayRandomSnapshotError::Mismatch {
            field,
            expected: expected.to_string(),
            actual: actual.to_string(),
        })
    }
}

/// Builds a frame header from stream metadata.
pub fn frame_header(
    stream_id: StreamId,
    entity_index: u32,
    sequence: u64,
    capture_time: SimTime,
    available_time: SimTime,
) -> FrameHeader {
    FrameHeader {
        stream_id,
        entity_index,
        sequence,
        capture_ticks: capture_time.ticks(),
        available_ticks: available_time.ticks(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn replay_header_roundtrips_jsonl() {
        let header = ReplayHeader::new(42, 1, SimDuration::from_ticks(16_666_666));
        let log = SimulationLog::new_with_header(header.clone());

        let file = NamedTempFile::new().unwrap();
        log.write_jsonl(file.path()).unwrap();
        let loaded = SimulationLog::read_jsonl(file.path()).unwrap();

        assert_eq!(loaded.header(), Some(&header));
        assert!(loaded.records().is_empty());
    }

    #[test]
    fn replay_header_log_record_json_is_stable() {
        let header = ReplayHeader::new(42, 1, SimDuration::from_ticks(16_666_666));
        let json = serde_json::to_string(&LogRecord::ReplayHeader { header }).unwrap();
        let expected = format!(
            concat!(
                r#"{{"kind":"replay_header","header":{{"#,
                r#""log_format_version":1,"#,
                r#""engine_version":"{}","#,
                r#""world_seed":42,"#,
                r#""rng_algorithm_version":1,"#,
                r#""keyed_random_version":1,"#,
                r#""stream_derivation_version":1,"#,
                r#""fixed_delta_ticks":16666666"#,
                r#"}}}}"#
            ),
            env!("CARGO_PKG_VERSION")
        );

        assert_eq!(json, expected);
    }

    #[test]
    fn replay_header_is_not_returned_as_payload_record() {
        let header = ReplayHeader::new(7, 1, SimDuration::from_ticks(1));
        let content = serde_json::to_string(&LogRecord::ReplayHeader { header }).unwrap();
        let file = NamedTempFile::new().unwrap();
        std::fs::write(file.path(), format!("{content}\n")).unwrap();

        let loaded = SimulationLog::read_jsonl(file.path()).unwrap();

        assert!(loaded.header().is_some());
        assert!(loaded.records().is_empty());
    }

    #[test]
    fn replay_header_validates_expected_compatibility() {
        let header = ReplayHeader::new(42, 1, SimDuration::from_ticks(16_666_666));
        let expected = ReplayCompatibility::current(42, 1, SimDuration::from_ticks(16_666_666));

        header.validate_compatibility(&expected).unwrap();
    }

    #[test]
    fn replay_header_rejects_timestep_mismatch() {
        let header = ReplayHeader::new(42, 1, SimDuration::from_ticks(16_666_666));
        let expected = ReplayCompatibility::current(42, 1, SimDuration::from_ticks(1));

        let error = header.validate_compatibility(&expected).unwrap_err();

        assert!(matches!(
            error,
            ReplayCompatibilityError::Mismatch {
                field: "fixed_delta_ticks",
                ..
            }
        ));
    }

    #[test]
    fn simulation_log_requires_header_for_compatibility_validation() {
        let log = SimulationLog::new();
        let expected = ReplayCompatibility::current(0, 1, SimDuration::from_ticks(1));

        assert_eq!(
            log.validate_compatibility(&expected),
            Err(ReplayCompatibilityError::MissingHeader)
        );
    }

    #[test]
    fn replay_random_snapshot_roundtrips_jsonl() {
        let snapshot = ReplayRandomSnapshot::new(SimTime::from_ticks(120), 3, 42, 99)
            .with_rng_state(ReplayRngState::new("episode", 7));
        let mut log = SimulationLog::new();
        log.record_random_snapshot(snapshot.clone());

        let file = NamedTempFile::new().unwrap();
        log.write_jsonl(file.path()).unwrap();
        let loaded = SimulationLog::read_jsonl(file.path()).unwrap();

        assert_eq!(loaded.latest_random_snapshot(), Some(&snapshot));
        assert_eq!(loaded.random_snapshots().count(), 1);
    }

    #[test]
    fn replay_random_snapshot_log_record_json_is_stable() {
        let snapshot = ReplayRandomSnapshot::new(SimTime::from_ticks(120), 3, 42, 99)
            .with_rng_state(ReplayRngState::new("episode", 7));
        let json = serde_json::to_string(&LogRecord::ReplayRandomSnapshot { snapshot }).unwrap();

        assert_eq!(
            json,
            concat!(
                r#"{"kind":"replay_random_snapshot","snapshot":{"#,
                r#""schema_version":1,"#,
                r#""sim_ticks":120,"#,
                r#""sequence":3,"#,
                r#""world_seed":42,"#,
                r#""world_main_rng_state":99,"#,
                r#""rng_states":[{"name":"episode","state":7}]"#,
                r#"}}"#
            )
        );
    }

    #[test]
    fn replay_random_snapshot_finds_named_rng_state() {
        let snapshot = ReplayRandomSnapshot::new(SimTime::from_ticks(1), 0, 5, 5)
            .with_rng_state(ReplayRngState::new("episode", 11));

        assert_eq!(snapshot.rng_state("episode"), Some(11));
        assert_eq!(snapshot.require_rng_state("episode"), Ok(11));
        assert_eq!(snapshot.rng_state("sensor"), None);
        assert_eq!(
            snapshot.require_rng_state("sensor"),
            Err(ReplayRandomSnapshotError::MissingRngState {
                name: "sensor".to_string()
            })
        );
    }

    #[test]
    fn replay_random_snapshot_validates_schema_and_seed() {
        let mut snapshot = ReplayRandomSnapshot::new(SimTime::from_ticks(1), 0, 5, 5);

        snapshot.validate_current_schema().unwrap();
        snapshot.validate_world_seed(5).unwrap();

        snapshot.schema_version += 1;

        assert_eq!(
            snapshot.validate_current_schema(),
            Err(ReplayRandomSnapshotError::UnsupportedSchemaVersion {
                expected: REPLAY_RANDOM_SNAPSHOT_VERSION,
                actual: REPLAY_RANDOM_SNAPSHOT_VERSION + 1,
            })
        );
        assert!(matches!(
            snapshot.validate_world_seed(6),
            Err(ReplayRandomSnapshotError::Mismatch {
                field: "world_seed",
                ..
            })
        ));
    }
}
