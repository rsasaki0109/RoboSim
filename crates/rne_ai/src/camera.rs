//! Wrist camera helpers for mobile manipulator simulation.

use rne_assets::WristCameraMountSpawned;
use rne_data::{ImageDepth, ImageRgb8, StreamId};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_sensor::{Sensor, CAMERA_DEPTH_STREAM_OFFSET};
use rne_world::world_transform_of;

const WRIST_CAMERA_STREAM_BASE: u32 = 400;

/// A wrist camera entity tracked relative to an arm link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WristCameraMount {
    /// Parent link the camera follows.
    pub parent_link: Entity,
    /// Camera sensor entity.
    pub camera: Entity,
    /// Mount offset from the parent link origin in meters.
    pub offset_m: Vec3,
}

/// Deterministic target estimate extracted from a wrist linear-depth frame.
///
/// The headless renderer does not provide semantic labels, so this contract uses
/// the nearest finite positive depth sample as a conservative obstacle/target
/// estimate. Ties are resolved by choosing the pixel closest to the image center,
/// which keeps the result stable when a probe fills an entire frame with one depth.
/// The returned offsets are in the camera frame: +X is image-right, +Y is up, and
/// the optical axis is local -Z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WristRgbdTargetEstimate {
    /// Depth-frame width in pixels.
    pub width_px: u32,
    /// Depth-frame height in pixels.
    pub height_px: u32,
    /// Selected target pixel horizontal coordinate.
    pub pixel_u_px: u32,
    /// Selected target pixel vertical coordinate.
    pub pixel_v_px: u32,
    /// Selected linear depth in meters.
    pub depth_m: f64,
    /// Center-pixel depth in meters.
    pub center_depth_m: f64,
    /// Minimum finite depth in the frame in meters.
    pub min_depth_m: f64,
    /// Camera-frame horizontal target offset in meters.
    pub offset_x_m: f64,
    /// Camera-frame vertical target offset in meters.
    pub offset_y_m: f64,
}

impl WristRgbdTargetEstimate {
    /// Estimates a target from a depth frame and vertical field of view.
    ///
    /// Returns `None` when the frame has no pixels, has inconsistent dimensions,
    /// or contains no finite positive depth. `fov_y_rad` is clamped to a safe
    /// pinhole range so malformed asset values cannot produce non-finite offsets.
    pub fn from_depth(frame: &ImageDepth, fov_y_rad: f64) -> Option<Self> {
        let pixel_count = usize::try_from(frame.width)
            .ok()?
            .checked_mul(usize::try_from(frame.height).ok()?)?;
        if pixel_count == 0 || frame.depth_m.len() < pixel_count {
            return None;
        }

        let center_u = (f64::from(frame.width) - 1.0) * 0.5;
        let center_v = (f64::from(frame.height) - 1.0) * 0.5;
        let mut selected: Option<(usize, f32, f64)> = None;
        let mut min_depth_m = f32::INFINITY;
        for (index, depth) in frame.depth_m.iter().take(pixel_count).copied().enumerate() {
            if !depth.is_finite() || depth <= 0.0 {
                continue;
            }
            min_depth_m = min_depth_m.min(depth);
            let u = index % frame.width as usize;
            let v = index / frame.width as usize;
            let center_distance =
                (u as f64 - center_u).mul_add(u as f64 - center_u, (v as f64 - center_v).powi(2));
            let should_select = selected.is_none_or(|(_, current_depth, current_distance)| {
                depth < current_depth
                    || (depth == current_depth && center_distance < current_distance)
            });
            if should_select {
                selected = Some((index, depth, center_distance));
            }
        }
        let (index, depth, _) = selected?;
        let fov_y_rad = if fov_y_rad.is_finite() {
            fov_y_rad.clamp(1.0e-3, std::f64::consts::PI - 1.0e-3)
        } else {
            std::f64::consts::FRAC_PI_4
        };
        let height_px = frame.height.max(1);
        let focal_y_px = f64::from(height_px) * 0.5 / (fov_y_rad * 0.5).tan();
        let focal_x_px = focal_y_px * f64::from(frame.width.max(1)) / f64::from(height_px);
        let pixel_u_px = (index % frame.width as usize) as u32;
        let pixel_v_px = (index / frame.width as usize) as u32;
        let depth_m = f64::from(depth);
        let offset_x_m = (f64::from(pixel_u_px) - center_u) * depth_m / focal_x_px;
        let offset_y_m = -(f64::from(pixel_v_px) - center_v) * depth_m / focal_y_px;
        Some(Self {
            width_px: frame.width,
            height_px: frame.height,
            pixel_u_px,
            pixel_v_px,
            depth_m,
            center_depth_m: f64::from(frame.center_depth_m()),
            min_depth_m: f64::from(min_depth_m),
            offset_x_m,
            offset_y_m,
        })
    }
}

impl From<WristCameraMountSpawned> for WristCameraMount {
    fn from(mount: WristCameraMountSpawned) -> Self {
        Self {
            parent_link: mount.parent_link,
            camera: mount.camera,
            offset_m: mount.mount_offset_m,
        }
    }
}

/// Returns the DataBus stream id for a robot wrist camera.
pub fn wrist_camera_stream_for_index(index: usize) -> StreamId {
    StreamId::new(WRIST_CAMERA_STREAM_BASE as u64 + index as u64)
}

/// Returns the paired depth stream id for a wrist camera RGB stream.
pub fn wrist_camera_depth_stream(rgb_stream: StreamId) -> StreamId {
    StreamId::new(rgb_stream.0 + CAMERA_DEPTH_STREAM_OFFSET)
}

/// Copies the parent link pose onto a free-floating camera mount entity.
pub fn sync_wrist_camera_mount(
    world: &mut World,
    parent_link: Entity,
    camera: Entity,
    offset_m: Vec3,
) {
    let parent = world_transform_of(world, parent_link);
    if let Some(mut camera_tf) = world.get_mut::<rne_world::Transform3>(camera) {
        camera_tf.translation = parent.translation + parent.rotation * offset_m;
        camera_tf.rotation = parent.rotation;
    }
}

/// Syncs every tracked wrist camera mount before sensor sampling.
pub fn sync_wrist_camera_mounts(world: &mut World, mounts: &[WristCameraMount]) {
    for mount in mounts {
        sync_wrist_camera_mount(world, mount.parent_link, mount.camera, mount.offset_m);
    }
}

/// Collects wrist camera mounts from asset spawn metadata.
pub fn wrist_camera_mounts_from_spawned(
    spawned: &[WristCameraMountSpawned],
) -> Vec<WristCameraMount> {
    spawned
        .iter()
        .copied()
        .map(WristCameraMount::from)
        .collect()
}

/// Returns the expected RGBA8 pixel count for the wrist camera when present.
pub fn wrist_camera_pixel_count(world: &World, mount: &WristCameraMount) -> Option<usize> {
    let sensor = world.get::<Sensor>(mount.camera)?;
    let rne_sensor::SensorKind::Camera(spec) = sensor.kind else {
        return None;
    };
    Some((spec.width * spec.height * 4) as usize)
}

/// Returns true when an image payload matches the configured camera dimensions.
pub fn wrist_camera_image_valid(image: &ImageRgb8, expected_pixels: usize) -> bool {
    !image.rgba8.is_empty() && image.rgba8.len() == expected_pixels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rgbd_target_estimate_selects_center_on_equal_depth_tie() {
        let frame = ImageDepth::new(3, 3, vec![1.0; 9]);
        let estimate = WristRgbdTargetEstimate::from_depth(&frame, 1.0).unwrap();
        assert_eq!((estimate.pixel_u_px, estimate.pixel_v_px), (1, 1));
        assert_eq!(estimate.offset_x_m, 0.0);
        assert_eq!(estimate.offset_y_m, 0.0);
    }

    #[test]
    fn rgbd_target_estimate_projects_off_center_depth() {
        let mut depth = vec![2.0; 12];
        depth[1] = 1.0;
        let estimate = WristRgbdTargetEstimate::from_depth(
            &ImageDepth::new(4, 3, depth),
            std::f64::consts::FRAC_PI_2,
        )
        .unwrap();
        assert_eq!((estimate.pixel_u_px, estimate.pixel_v_px), (1, 0));
        assert!(estimate.offset_x_m < 0.0);
        assert!(estimate.offset_y_m > 0.0);
        assert_eq!(estimate.min_depth_m, 1.0);
    }

    #[test]
    fn rgbd_target_estimate_rejects_invalid_frame() {
        assert!(WristRgbdTargetEstimate::from_depth(
            &ImageDepth::new(2, 2, vec![f32::NAN; 4]),
            1.0,
        )
        .is_none());
    }
}
