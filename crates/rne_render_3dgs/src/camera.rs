//! Camera adapter from RNE pinhole cameras to `wgpu-3dgs-viewer`.

use glam::{Mat4, UVec2};
use rne_math::Transform3;
use rne_render::Camera;
use wgpu_3dgs_viewer::CameraTrait;

/// Bridges RNE world-space cameras to the splat viewer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RneSplatCamera {
    view: Mat4,
    projection: Mat4,
}

impl RneSplatCamera {
    /// Builds a splat camera from RNE camera parameters and pose.
    pub fn from_rne(camera: &Camera, view: &Transform3) -> Self {
        let view_matrix = Camera::view_matrix(view).as_mat4();
        let projection = camera.projection_matrix().as_mat4();
        Self {
            view: view_matrix,
            projection,
        }
    }

    /// Updates the splat viewer camera buffers.
    pub fn upload(&self, viewer: &mut wgpu_3dgs_viewer::Viewer, queue: &wgpu::Queue, size: UVec2) {
        viewer.update_camera(queue, self, size);
    }
}

impl CameraTrait for RneSplatCamera {
    fn view(&self) -> Mat4 {
        self.view
    }

    fn projection(&self, _aspect_ratio: f32) -> Mat4 {
        self.projection
    }
}
