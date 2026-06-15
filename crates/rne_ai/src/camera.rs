//! Wrist camera helpers for mobile manipulator simulation.

use rne_assets::WristCameraMountSpawned;
use rne_data::{ImageRgb8, StreamId};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_sensor::Sensor;
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
