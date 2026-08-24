//! Sensor ECS components.

use crate::{CameraSpec, ImuSpec, LidarSpec, WheelEncoderSpec};
use bevy_ecs::prelude::Component;
use rne_core::SimDuration;
use rne_data::{JointFeedback, StreamId};
use rne_ecs::Entity;
use rne_math::Vec3;
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

/// Evolving IMU error state carried between samples.
///
/// Bias instability and rate random walk are time-correlated processes, so they cannot
/// be derived from a sample index alone. This component holds them on the sensor entity,
/// together with the previous body velocity needed to derive proper acceleration.
/// Replaying the same sample sequence reproduces the same drift.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ImuState {
    /// Gyroscope Gauss-Markov bias in radians per second.
    pub gyro_bias_rad_s: Vec3,
    /// Accelerometer Gauss-Markov bias in meters per second squared.
    pub accel_bias_m_s2: Vec3,
    /// Accumulated gyroscope rate random walk in radians per second.
    pub gyro_rate_walk_rad_s: Vec3,
    /// Accumulated accelerometer rate random walk in meters per second squared.
    pub accel_rate_walk_m_s2: Vec3,
    /// World-frame linear velocity at the previous sample, in meters per second.
    pub previous_linear_velocity_m_s: Vec3,
    /// Simulation ticks of the previous sample.
    pub previous_sample_ticks: u64,
    /// Whether a previous sample has been taken.
    pub initialized: bool,
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

/// One named joint included in a [`JointFeedbackSensor`] stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointFeedbackChannelSpec {
    /// Stable joint name serialized into every sample.
    pub name: String,
    /// ECS entity carrying backend-neutral joint state and actuation components.
    pub joint_entity: Entity,
}

/// Deterministic fault injected after sampling and before output latency.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum JointFeedbackFault {
    /// No injected fault.
    #[default]
    None,
    /// Drop exactly one attempted sequence, producing an observable sequence gap.
    DropSequence {
        /// One-based attempted sequence to drop.
        sequence: u64,
    },
    /// Hold the last emitted values from this attempted sequence onward.
    StuckFromSequence {
        /// One-based first attempted sequence that reuses the previous values.
        sequence: u64,
    },
}

/// Typed joint and actuator feedback sensor configuration.
///
/// This is separate from [`SensorKind`] so adding the control-oriented stream
/// does not break exhaustive matches over the compatibility-stable sensor enum.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct JointFeedbackSensor {
    /// Update rate in hertz.
    pub update_rate_hz: f64,
    /// Exact sampling period in simulation ticks, overriding `update_rate_hz`.
    ///
    /// Use this when a TaskSpec declares an integer control period that cannot
    /// be represented exactly by conversion from hertz.
    pub sample_period_ticks: Option<u64>,
    /// First scheduled capture time in simulation nanosecond ticks.
    pub phase_offset_ticks: u64,
    /// Capture-to-availability latency in simulation nanosecond ticks.
    pub latency_ticks: u64,
    /// Whether sampling is enabled.
    pub enabled: bool,
    /// DataBus stream id.
    pub stream_id: StreamId,
    /// Joint channels in externally visible contract order.
    pub channels: Vec<JointFeedbackChannelSpec>,
    /// Optional deterministic fault applied in the documented processing order.
    pub fault: JointFeedbackFault,
}

impl JointFeedbackSensor {
    /// Sample period derived from the declared update rate.
    pub fn period(&self) -> SimDuration {
        self.sample_period_ticks.map_or_else(
            || SimDuration::from_hertz(rne_math::Hertz::new(self.update_rate_hz)),
            SimDuration::from_ticks,
        )
    }

    /// Returns true when rate, channel identities, and fault sequence are valid.
    pub fn is_valid(&self) -> bool {
        if !self.update_rate_hz.is_finite()
            || self.update_rate_hz <= 0.0
            || self.sample_period_ticks == Some(0)
            || self.channels.is_empty()
        {
            return false;
        }
        let mut names = std::collections::BTreeSet::new();
        let mut entities = std::collections::BTreeSet::new();
        if self.channels.iter().any(|channel| {
            channel.name.is_empty()
                || !names.insert(channel.name.as_str())
                || !entities.insert(channel.joint_entity.index())
        }) {
            return false;
        }
        match self.fault {
            JointFeedbackFault::None => true,
            JointFeedbackFault::DropSequence { sequence } => sequence > 0,
            JointFeedbackFault::StuckFromSequence { sequence } => sequence > 1,
        }
    }
}

/// Runtime state for a [`JointFeedbackSensor`].
#[derive(Component, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct JointFeedbackSensorState {
    /// Number of scheduled sample attempts, including injected drops.
    pub attempted_sequence: u64,
    /// Number of frames actually emitted.
    pub emitted_frames: u64,
    /// Last emitted payload used by deterministic stuck-value injection.
    pub last_emitted: Option<JointFeedback>,
}
