//! Render backend trait.

use crate::{Camera, ImageFrame, RenderTarget};
use rne_core::SimTime;
use rne_math::Transform3;
use thiserror::Error;

/// Render backend error.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RenderError {
    /// No compatible GPU adapter was found.
    #[error("no compatible GPU adapter")]
    NoAdapter,
    /// The backend failed to initialize.
    #[error("backend initialization failed: {0}")]
    InitFailed(String),
    /// Rendering failed.
    #[error("render failed: {0}")]
    RenderFailed(String),
}

/// Backend-agnostic rendering interface.
pub trait RenderBackend {
    /// Clears a render target to a solid color.
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError>;

    /// Renders a camera view. MVP backends may ignore scene geometry.
    fn render_camera(
        &mut self,
        camera: &Camera,
        view: &Transform3,
        clear_color: [f32; 4],
        sim_time: SimTime,
        seed: u64,
    ) -> Result<ImageFrame, RenderError> {
        let _ = (view, sim_time, seed);
        self.render_clear(camera.render_target(), clear_color)
    }
}
