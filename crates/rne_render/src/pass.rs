//! Combined outputs from a camera render pass.

use crate::{DepthFrame, ImageFrame};

/// Color and depth outputs from one camera render.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraPassOutput {
    /// RGBA8 color image.
    pub color: ImageFrame,
    /// Linear depth image in meters.
    pub depth: DepthFrame,
}
