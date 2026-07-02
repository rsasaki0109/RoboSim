//! Depth buffer output from a camera pass.

use crate::{Camera, RenderScene};
use rne_math::{Transform3, Vec3};

/// Linear view-space depth in meters produced by a render pass.
#[derive(Clone, Debug, PartialEq)]
pub struct DepthFrame {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Row-major linear depth values in meters.
    pub depth_m: Vec<f32>,
}

impl DepthFrame {
    /// Creates a depth frame from raw values.
    pub fn new(width: u32, height: u32, depth_m: Vec<f32>) -> Self {
        Self {
            width,
            height,
            depth_m,
        }
    }

    /// Returns a stable hash of the depth buffer for determinism tests.
    pub fn hash_depth(&self) -> u64 {
        hash_depth_f32(&self.depth_m)
    }
}

/// Computes a stable FNV-1a hash over depth values bit patterns.
///
/// Keep in sync with `rne_data::payloads::hash_depth_f32`.
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

/// Deterministic center and minimum depth from scene item transforms.
///
/// Uses a cone test in the camera forward direction. Suitable for headless
/// camera sensors without a GPU rasterizer.
pub fn scene_depth_probe(camera: &Camera, view: &Transform3, scene: &RenderScene) -> (f32, f32) {
    let forward = (view.rotation * -Vec3::Z).normalize();
    let mut min_m = camera.far_m as f32;
    let mut center_m = camera.far_m as f32;
    let half_fov = camera.fov_y_rad * 0.5;
    let center_dot = (half_fov * 1.5).cos();

    for item in &scene.items {
        let delta = item.transform.translation - view.translation;
        let dist = delta.length() as f32;
        if dist < 0.02 {
            continue;
        }
        let dir = delta.normalize();
        let dot = forward.dot(dir);
        if dot <= 0.05 {
            continue;
        }
        min_m = min_m.min(dist);
        if dot >= center_dot {
            center_m = center_m.min(dist);
        }
    }

    (center_m, min_m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::RenderScene;
    use crate::VisualShape;
    use rne_math::Quat;
    use rne_world::Transform3 as WorldTransform3;

    #[test]
    fn depth_hash_is_stable() {
        let frame = DepthFrame::new(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(frame.hash_depth(), frame.hash_depth());
    }

    #[test]
    fn scene_depth_probe_reports_nearest_item() {
        let camera = Camera::new(64, 48, std::f64::consts::FRAC_PI_4);
        let view = Transform3::from_translation_rotation(Vec3::new(0.0, 0.6, 0.0), Quat::IDENTITY);
        let mut scene = RenderScene::new();
        scene.items.push(RenderScene::item_from_visual(
            WorldTransform3::from_translation_rotation(Vec3::new(0.0, 0.58, -0.8), Quat::IDENTITY),
            VisualShape::Box {
                size_m: Vec3::new(0.06, 0.06, 0.06),
            },
            [1.0, 0.0, 0.0, 1.0],
            WorldTransform3::IDENTITY,
        ));
        let (center, min) = scene_depth_probe(&camera, &view, &scene);
        assert!(center < camera.far_m as f32);
        assert!((center - min).abs() < f32::EPSILON);
    }
}
