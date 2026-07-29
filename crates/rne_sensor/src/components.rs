//! Sensor ECS components.

use crate::{CameraSpec, ImuSpec, LidarSpec, WheelEncoderSpec};
use bevy_ecs::prelude::Component;
use rne_core::SimDuration;
use rne_data::StreamId;
use serde::{Deserialize, Serialize};

/// Non-visual optical properties used by physics-aware LiDAR sampling.
///
/// This component is deliberately independent of render and physics materials:
/// importers or applications may attach it to any raycast-hit entity without
/// changing how that entity looks or how contact response is calculated.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LidarMaterial {
    /// Fraction of incident laser energy reflected toward the environment.
    pub reflectivity: f64,
    /// Fraction of incident laser energy transmitted to later surfaces.
    pub transmissivity: f64,
    /// Surface roughness in `[0, 1]`; rough surfaces have a broader angular response.
    pub roughness: f64,
}

impl LidarMaterial {
    /// Creates a material with values clamped to the physical `[0, 1]` interval.
    pub fn new(reflectivity: f64, transmissivity: f64, roughness: f64) -> Self {
        Self {
            reflectivity: reflectivity.clamp(0.0, 1.0),
            transmissivity: transmissivity.clamp(0.0, 1.0),
            roughness: roughness.clamp(0.0, 1.0),
        }
    }

    /// Clear architectural glass with a weak first return and strong transmission.
    pub fn clear_glass() -> Self {
        Self::new(0.12, 0.82, 0.05)
    }

    /// Dry asphalt with low reflectivity and high roughness.
    pub fn dry_asphalt() -> Self {
        Self::new(0.18, 0.0, 0.9)
    }

    /// Diffuse concrete with moderate reflectivity.
    pub fn concrete() -> Self {
        Self::new(0.45, 0.0, 0.75)
    }

    /// Painted metal with a strong, comparatively smooth return.
    pub fn painted_metal() -> Self {
        Self::new(0.72, 0.0, 0.25)
    }
}

impl Default for LidarMaterial {
    fn default() -> Self {
        Self::new(0.5, 0.0, 0.5)
    }
}

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
