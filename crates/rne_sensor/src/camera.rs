//! RGB camera sensor specification and sampling.

use rne_core::SimTime;
use rne_data::ImageRgb8;
use rne_render::{Camera, RenderBackend};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// RGB camera parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraSpec {
    /// Output width in pixels.
    pub width: u32,
    /// Output height in pixels.
    pub height: u32,
    /// Vertical field of view in radians.
    pub fov_y_rad: f64,
    /// Deterministic render seed.
    pub seed: u64,
}

impl Default for CameraSpec {
    fn default() -> Self {
        Self {
            width: 64,
            height: 48,
            fov_y_rad: std::f64::consts::FRAC_PI_4,
            seed: 0,
        }
    }
}

/// Samples an RGB camera attached to the given entity transform.
pub fn sample_camera<R: RenderBackend + ?Sized>(
    render: &mut R,
    transform: &Transform3,
    spec: &CameraSpec,
    sim_time: SimTime,
) -> ImageRgb8 {
    let camera = Camera::new(spec.width, spec.height, spec.fov_y_rad);
    let view = rne_math::Transform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    };

    let frame = render
        .render_camera(&camera, &view, [0.05, 0.08, 0.12, 1.0], sim_time, spec.seed)
        .expect("camera render");

    ImageRgb8::from_rgba8(frame.width, frame.height, frame.rgba8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_render::HeadlessRenderBackend;

    #[test]
    fn camera_sensor_returns_image_payload() {
        let mut backend = HeadlessRenderBackend::new();
        let spec = CameraSpec {
            width: 16,
            height: 12,
            ..CameraSpec::default()
        };
        let image = sample_camera(
            &mut backend,
            &Transform3::default(),
            &spec,
            SimTime::from_ticks(10),
        );

        assert_eq!(image.width, 16);
        assert_eq!(image.height, 12);
        assert_eq!(image.rgba8.len(), 16 * 12 * 4);
    }
}
