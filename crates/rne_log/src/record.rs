//! Log recording.

use rne_core::SimTime;
use rne_data::{FrameHeader, ImuSample, PointCloud, StreamId, WheelEncoderSample};
use rne_robot::ActuatorCommandEntry;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;
use thiserror::Error;

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

/// One JSONL log record.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum LogRecord {
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
    records: Vec<LogRecord>,
}

impl SimulationLog {
    /// Creates an empty log.
    pub fn new() -> Self {
        Self::default()
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

    /// Writes the log to a JSONL file.
    pub fn write_jsonl(&self, path: impl AsRef<Path>) -> Result<(), LogError> {
        let mut file = File::create(path)?;
        for record in &self.records {
            serde_json::to_writer(&mut file, record)?;
            file.write_all(b"\n")?;
        }
        Ok(())
    }

    /// Loads a log from a JSONL file.
    pub fn read_jsonl(path: impl AsRef<Path>) -> Result<Self, LogError> {
        let content = std::fs::read_to_string(path)?;
        let mut records = Vec::new();
        for line in content.lines().filter(|line| !line.trim().is_empty()) {
            records.push(serde_json::from_str(line)?);
        }
        Ok(Self { records })
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
