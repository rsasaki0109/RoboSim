//! Common sensor frame payloads.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// IMU sample payload.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
    /// Linear acceleration in meters per second squared.
    pub linear_acceleration_m_s2: Vec3,
}

/// LiDAR point cloud payload.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointCloud {
    /// Points in the sensor frame, meters.
    pub points_m: Vec<Vec3>,
}

impl PointCloud {
    /// Creates an empty point cloud.
    pub fn new() -> Self {
        Self {
            points_m: Vec::new(),
        }
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new()
    }
}

/// Wheel encoder sample payload.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WheelEncoderSample {
    /// Wheel position in radians.
    pub position_rad: f64,
    /// Wheel velocity in radians per second.
    pub velocity_rad_s: f64,
}

/// Articulated joint positions and velocities published on the DataBus.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    /// Joint names matching `positions_rad` / `velocities_rad_s` order.
    pub names: Vec<String>,
    /// Joint positions in radians, in actuation order.
    pub positions_rad: Vec<f64>,
    /// Joint velocities in radians per second, in actuation order.
    pub velocities_rad_s: Vec<f64>,
}

/// RGBA8 camera image payload.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ImageRgb8 {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// RGBA8 pixel data in row-major order.
    pub rgba8: Vec<u8>,
}

impl ImageRgb8 {
    /// Creates an image payload from RGBA8 bytes.
    pub fn from_rgba8(width: u32, height: u32, rgba8: Vec<u8>) -> Self {
        Self {
            width,
            height,
            rgba8,
        }
    }
}
