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

/// Linear depth image payload in meters (row-major).
///
/// Headless camera sensors publish probe-derived depth (see `scene_depth_probe` in
/// `rne_render`). Values are noiseless and deterministic for a given scene pose.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImageDepth {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Row-major linear depth values in meters.
    pub depth_m: Vec<f32>,
}

impl ImageDepth {
    /// Creates a depth image payload from raw values.
    pub fn new(width: u32, height: u32, depth_m: Vec<f32>) -> Self {
        Self {
            width,
            height,
            depth_m,
        }
    }

    /// Returns the center-pixel depth in meters when the buffer is non-empty.
    pub fn center_depth_m(&self) -> f32 {
        if self.depth_m.is_empty() {
            return 0.0;
        }
        let center = (self.height / 2 * self.width + self.width / 2) as usize;
        self.depth_m.get(center).copied().unwrap_or(self.depth_m[0])
    }

    /// Returns the minimum finite depth in the buffer.
    pub fn min_depth_m(&self) -> f32 {
        self.depth_m
            .iter()
            .copied()
            .filter(|depth| depth.is_finite())
            .fold(f32::INFINITY, f32::min)
    }

    /// Returns a stable FNV-1a hash of depth values for determinism tests.
    pub fn hash_depth(&self) -> u64 {
        hash_depth_f32(&self.depth_m)
    }
}

/// Computes a stable FNV-1a hash over depth values bit patterns.
///
/// Keep in sync with the duplicate in `rne_render::depth::hash_depth_f32` (render
/// cannot depend on `rne_data`).
pub fn hash_depth_f32(values: &[f32]) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for value in values {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}
