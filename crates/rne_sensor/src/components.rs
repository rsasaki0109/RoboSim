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
    /// Multiplier applied to the diffuse return for retroreflective sheeting.
    ///
    /// Diffuse surfaces use `1.0`. Corner-cube sheeting used on road signs and
    /// licence plates returns one to two orders of magnitude more energy toward the
    /// emitter, which is what drives detector saturation and blooming in real scans.
    /// The gain only applies near normal entrance angles; see
    /// [`crate::lidar`] for the entrance-angle falloff.
    #[serde(default = "unit_gain")]
    pub retroreflective_gain: f64,
}

fn unit_gain() -> f64 {
    1.0
}

impl LidarMaterial {
    /// Creates a diffuse material with values clamped to the physical `[0, 1]` interval.
    pub fn new(reflectivity: f64, transmissivity: f64, roughness: f64) -> Self {
        Self {
            reflectivity: reflectivity.clamp(0.0, 1.0),
            transmissivity: transmissivity.clamp(0.0, 1.0),
            roughness: roughness.clamp(0.0, 1.0),
            retroreflective_gain: 1.0,
        }
    }

    /// Returns this material with a retroreflective gain of at least `1.0`.
    pub fn with_retroreflective_gain(mut self, gain: f64) -> Self {
        self.retroreflective_gain = if gain.is_finite() { gain.max(1.0) } else { 1.0 };
        self
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

    /// Retroreflective road-sign sheeting that saturates the detector near normal incidence.
    pub fn retroreflective_sign() -> Self {
        Self::new(0.85, 0.0, 0.1).with_retroreflective_gain(60.0)
    }

    /// Retroreflective licence-plate sheeting with a narrower, weaker lobe than signage.
    pub fn licence_plate() -> Self {
        Self::new(0.7, 0.0, 0.15).with_retroreflective_gain(25.0)
    }
}

impl Default for LidarMaterial {
    fn default() -> Self {
        Self::new(0.5, 0.0, 0.5)
    }
}

/// Sensor type specification.
///
/// [`LidarSpec`] is much larger than the other specs because it carries the full
/// physical scan, beam, noise and weather model. [`Sensor`] is cloned once per
/// sensor per sample tick, so copying that padding is cheaper than the heap
/// indirection boxing the variant would add to that hot path.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum SensorKind {
    /// Inertial measurement unit.
    Imu(ImuSpec),
    /// Scanning LiDAR, single-plane or multi-channel.
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
