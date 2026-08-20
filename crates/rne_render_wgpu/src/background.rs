//! Optional background render pass hook for hybrid mesh compositing.

use rne_math::Transform3;
use rne_render::{Camera, RenderError};

/// Renders a visual-only background into an existing GPU color target.
pub trait BackgroundRenderPass {
    /// Draws the background into `color_view` before the mesh foreground pass.
    fn render_background(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        color_view: &wgpu::TextureView,
        camera: &Camera,
        view: &Transform3,
        clear_color: [f32; 4],
    ) -> Result<(), RenderError>;
}
