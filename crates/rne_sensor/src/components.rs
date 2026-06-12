//! Sensor ECS components.

use crate::{CameraSpec, ImuSpec, LidarSpec, WheelEncoderSpec};
use bevy_ecs::prelude::Component;
use rne_core::SimDuration;
use rne_data::StreamId;

/// Sensor type specification.
#[derive(Clone, Debug, PartialEq)]
pub enum SensorKind {
    /// Inertial measurement unit.
    Imu(ImuSpec),
    /// 2D scanning LiDAR.
    Lidar(LidarSpec),
    /// RGB camera.
    Camera(CameraSpec),
    /// Wheel encoder.
    WheelEncoder(WheelEncoderSpec),
}

/// Sensor entity configuration.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Sensor {
    /// Sensor kind and parameters.
    pub kind: SensorKind,
    /// Update rate in hertz.
    pub update_rate_hz: f64,
    /// Output latency in simulation nanosecond ticks.
    pub latency_ticks: u64,
    /// Internal coordinate frame id.
    pub frame_id: u32,
    /// Whether sampling is enabled.
    pub enabled: bool,
    /// DataBus stream id.
    pub stream_id: StreamId,
}

impl Sensor {
    /// Sample period derived from update rate.
    pub fn period(&self) -> SimDuration {
        SimDuration::from_hertz(rne_math::Hertz::new(self.update_rate_hz))
    }

    /// Output latency as a simulation duration.
    pub fn latency(&self) -> SimDuration {
        SimDuration::from_ticks(self.latency_ticks)
    }
}

/// Runtime sensor sampling state.
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub struct SensorState {
    /// Last published sequence number.
    pub last_sequence: u64,
    /// Simulation ticks of the last sample.
    pub last_sample_ticks: u64,
    /// Total emitted frames.
    pub frame_count: u64,
}
