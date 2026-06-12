//! Camera parameters for rendering and camera sensors.

use crate::RenderTarget;
use rne_math::{Mat4, Transform3, Vec3};

/// Pinhole camera parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    /// Output width in pixels.
    pub width: u32,
    /// Output height in pixels.
    pub height: u32,
    /// Vertical field of view in radians.
    pub fov_y_rad: f64,
    /// Near clip plane in meters.
    pub near_m: f64,
    /// Far clip plane in meters.
    pub far_m: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            width: 64,
            height: 48,
            fov_y_rad: std::f64::consts::FRAC_PI_4,
            near_m: 0.1,
            far_m: 100.0,
        }
    }
}

impl Camera {
    /// Creates a camera with the given resolution and vertical field of view.
    pub fn new(width: u32, height: u32, fov_y_rad: f64) -> Self {
        Self {
            width,
            height,
            fov_y_rad,
            ..Self::default()
        }
    }

    /// Returns the render target for this camera.
    pub fn render_target(&self) -> RenderTarget {
        RenderTarget::new(self.width, self.height)
    }

    /// Builds a right-handed perspective projection matrix.
    pub fn projection_matrix(&self) -> Mat4 {
        let aspect = self.width as f64 / self.height.max(1) as f64;
        Mat4::perspective_rh(self.fov_y_rad, aspect, self.near_m, self.far_m)
    }

    /// Builds a view matrix from a camera world transform looking down local -Z.
    pub fn view_matrix(camera_world: &Transform3) -> Mat4 {
        let eye = camera_world.translation;
        let forward = camera_world.rotation * -Vec3::Z;
        let target = eye + forward;
        let up = camera_world.rotation * Vec3::Y;
        Mat4::look_at_rh(eye, target, up)
    }

    /// Returns the combined view-projection matrix for a camera pose.
    pub fn view_projection(&self, camera_world: &Transform3) -> Mat4 {
        self.projection_matrix() * Self::view_matrix(camera_world)
    }
}
