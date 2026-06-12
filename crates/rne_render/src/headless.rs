//! Deterministic headless render backend for CI and camera sensors.

use crate::backend::{RenderBackend, RenderError};
use crate::camera::Camera;
use crate::depth::DepthFrame;
use crate::image::{ImageFrame, RenderTarget};
use crate::pass::CameraPassOutput;
use crate::scene::RenderScene;
use rne_core::SimTime;
use rne_math::Transform3;

/// CPU-only renderer that produces deterministic placeholder images.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HeadlessRenderBackend;

impl HeadlessRenderBackend {
    /// Creates a headless renderer.
    pub fn new() -> Self {
        Self
    }

    /// Builds a deterministic RGBA8 image from camera parameters and a seed.
    pub fn placeholder_image(camera: &Camera, seed: u64) -> ImageFrame {
        let target = camera.render_target();
        let mut rgba8 = vec![0_u8; target.rgba8_len()];

        for y in 0..target.height {
            for x in 0..target.width {
                let i = ((y * target.width + x) * 4) as usize;
                let pixel_seed = seed
                    .wrapping_add(u64::from(x))
                    .wrapping_mul(0x9E3779B97F4A7C15)
                    .wrapping_add(u64::from(y).wrapping_mul(0xBF58476D1CE4E5B9));

                rgba8[i] = ((pixel_seed >> 16) & 0xFF) as u8;
                rgba8[i + 1] = ((pixel_seed >> 8) & 0xFF) as u8;
                rgba8[i + 2] = (pixel_seed & 0xFF) as u8;
                rgba8[i + 3] = 255;
            }
        }

        ImageFrame::from_rgba8(target.width, target.height, rgba8)
    }

    /// Builds a deterministic image that mixes pose and simulation time.
    pub fn camera_image(
        camera: &Camera,
        view: &Transform3,
        sim_time: SimTime,
        seed: u64,
    ) -> ImageFrame {
        let pose_seed = seed
            .wrapping_add(sim_time.ticks())
            .wrapping_add(view.translation.x.to_bits())
            .wrapping_add(view.translation.z.to_bits());
        Self::placeholder_image(camera, pose_seed)
    }
}

impl RenderBackend for HeadlessRenderBackend {
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        let mut rgba8 = vec![0_u8; target.rgba8_len()];
        let r = (clear_color[0].clamp(0.0, 1.0) * 255.0) as u8;
        let g = (clear_color[1].clamp(0.0, 1.0) * 255.0) as u8;
        let b = (clear_color[2].clamp(0.0, 1.0) * 255.0) as u8;
        let a = (clear_color[3].clamp(0.0, 1.0) * 255.0) as u8;

        for chunk in rgba8.chunks_exact_mut(4) {
            chunk.copy_from_slice(&[r, g, b, a]);
        }

        Ok(ImageFrame::from_rgba8(target.width, target.height, rgba8))
    }

    fn render_camera(
        &mut self,
        camera: &Camera,
        view: &Transform3,
        clear_color: [f32; 4],
        sim_time: SimTime,
        seed: u64,
    ) -> Result<ImageFrame, RenderError> {
        let _ = clear_color;
        Ok(Self::camera_image(camera, view, sim_time, seed))
    }

    fn render_scene_camera(
        &mut self,
        camera: &Camera,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<CameraPassOutput, RenderError> {
        let color = self.render_clear(camera.render_target(), clear_color)?;
        let mut depth_m = vec![camera.far_m as f32; (camera.width * camera.height) as usize];

        if let Some(item) = scene.items.first() {
            let distance = (view.translation - item.transform.translation).length() as f32;
            let center = (camera.height / 2 * camera.width + camera.width / 2) as usize;
            if center < depth_m.len() {
                depth_m[center] = distance;
            }
        }

        Ok(CameraPassOutput {
            color,
            depth: DepthFrame::new(camera.width, camera.height, depth_m),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hash_rgba8;
    use rne_math::{Quat, Vec3};

    #[test]
    fn headless_camera_is_deterministic() {
        let camera = Camera::new(32, 24, std::f64::consts::FRAC_PI_4);
        let view = Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 2.0), Quat::IDENTITY);
        let sim_time = SimTime::from_ticks(42);

        let first = HeadlessRenderBackend::camera_image(&camera, &view, sim_time, 7);
        let second = HeadlessRenderBackend::camera_image(&camera, &view, sim_time, 7);

        assert_eq!(first, second);
        assert_eq!(first.hash_pixels(), second.hash_pixels());
        assert_ne!(first.hash_pixels(), hash_rgba8(&[]));
    }

    #[test]
    fn clear_color_is_uniform() {
        let mut backend = HeadlessRenderBackend::new();
        let frame = backend
            .render_clear(RenderTarget::new(4, 4), [0.2, 0.4, 0.6, 1.0])
            .unwrap();

        assert_eq!(frame.rgba8.len(), 4 * 4 * 4);
        assert!(frame.rgba8.starts_with(&[51, 102, 153, 255]));
    }
}
