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
    /// Points in the world frame, meters.
    pub points_m: Vec<Vec3>,
    /// Normalized return intensity for each point in `[0, 1]`.
    ///
    /// Legacy point clouds may leave this empty. Otherwise its length matches
    /// [`Self::points_m`].
    #[serde(default)]
    pub intensities: Vec<f32>,
    /// Source ray index for each point.
    ///
    /// Legacy point clouds may leave this empty. Otherwise its length matches
    /// [`Self::points_m`].
    #[serde(default)]
    pub ray_indices: Vec<u32>,
    /// One-based return index within the source ray.
    ///
    /// Legacy point clouds may leave this empty. Otherwise its length matches
    /// [`Self::points_m`].
    #[serde(default)]
    pub return_indices: Vec<u8>,
    /// Zero-based elevation channel (ring) index for each point.
    ///
    /// Single-plane scanners report `0` for every point. Legacy point clouds may
    /// leave this empty; otherwise its length matches [`Self::points_m`].
    #[serde(default)]
    pub channel_indices: Vec<u16>,
    /// Emission time of each point relative to the start of the scan, in seconds.
    ///
    /// A spinning scanner emits one azimuth column at a time, so points in the same
    /// cloud are captured at different instants. Consumers that need motion-corrected
    /// geometry must use these offsets. Legacy point clouds may leave this empty;
    /// otherwise its length matches [`Self::points_m`].
    #[serde(default)]
    pub timestamps_s: Vec<f64>,
}

impl PointCloud {
    /// Creates an empty point cloud.
    pub fn new() -> Self {
        Self {
            points_m: Vec::new(),
            intensities: Vec::new(),
            ray_indices: Vec::new(),
            return_indices: Vec::new(),
            channel_indices: Vec::new(),
            timestamps_s: Vec::new(),
        }
    }

    /// Appends one LiDAR return while preserving parallel-array invariants.
    pub fn push_return(
        &mut self,
        point_m: Vec3,
        intensity: f32,
        ray_index: u32,
        return_index: u8,
        channel_index: u16,
        timestamp_s: f64,
    ) {
        self.points_m.push(point_m);
        self.intensities.push(intensity);
        self.ray_indices.push(ray_index);
        self.return_indices.push(return_index);
        self.channel_indices.push(channel_index);
        self.timestamps_s.push(timestamp_s);
    }

    /// Returns true when all optional LiDAR attributes are absent or aligned.
    pub fn attributes_are_aligned(&self) -> bool {
        let len = self.points_m.len();
        [
            self.intensities.len(),
            self.ray_indices.len(),
            self.return_indices.len(),
            self.channel_indices.len(),
            self.timestamps_s.len(),
        ]
        .into_iter()
        .all(|attribute_len| attribute_len == 0 || attribute_len == len)
    }

    /// Returns the scan duration implied by the per-point timestamps, in seconds.
    ///
    /// Returns `0.0` when the cloud carries no timestamps.
    pub fn scan_duration_s(&self) -> f64 {
        let min = self
            .timestamps_s
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .timestamps_s
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        if min.is_finite() && max.is_finite() {
            max - min
        } else {
            0.0
        }
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new()
    }
}

/// Localization pose estimate payload.
///
/// Published by localization or ground-truth-with-latency sources so controllers can
/// consume pose through the DataBus — and therefore through
/// [`crate::Frame::available_time`] — instead of reading simulator state directly.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PoseSample {
    /// Position in the world frame, meters.
    pub position_m: Vec3,
    /// Heading about the world up axis in radians.
    pub yaw_rad: f64,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_cloud_return_attributes_stay_aligned() {
        let mut cloud = PointCloud::new();
        cloud.push_return(Vec3::new(1.0, 2.0, 3.0), 0.75, 4, 2, 9, 0.004);

        assert!(cloud.attributes_are_aligned());
        assert_eq!(cloud.points_m.len(), 1);
        assert_eq!(cloud.intensities, vec![0.75]);
        assert_eq!(cloud.ray_indices, vec![4]);
        assert_eq!(cloud.return_indices, vec![2]);
        assert_eq!(cloud.channel_indices, vec![9]);
        assert_eq!(cloud.timestamps_s, vec![0.004]);
    }

    #[test]
    fn legacy_point_cloud_without_attributes_is_valid() {
        let cloud = PointCloud {
            points_m: vec![Vec3::X],
            ..PointCloud::new()
        };

        assert!(cloud.attributes_are_aligned());
    }

    #[test]
    fn legacy_serialized_cloud_deserializes_without_new_attributes() {
        let legacy = r#"{"points_m":[[1.0,2.0,3.0]],"intensities":[0.5],"ray_indices":[0],"return_indices":[1]}"#;
        let cloud: PointCloud = serde_json::from_str(legacy).expect("legacy point cloud");

        assert_eq!(cloud.points_m.len(), 1);
        assert!(cloud.channel_indices.is_empty());
        assert!(cloud.timestamps_s.is_empty());
        assert!(cloud.attributes_are_aligned());
        assert_eq!(cloud.scan_duration_s(), 0.0);
    }

    #[test]
    fn scan_duration_spans_first_and_last_emission() {
        let mut cloud = PointCloud::new();
        cloud.push_return(Vec3::X, 0.4, 0, 1, 0, 0.0);
        cloud.push_return(Vec3::Y, 0.4, 1, 1, 0, 0.05);
        cloud.push_return(Vec3::Z, 0.4, 2, 1, 0, 0.1);

        assert!((cloud.scan_duration_s() - 0.1).abs() < 1e-12);
    }
}
