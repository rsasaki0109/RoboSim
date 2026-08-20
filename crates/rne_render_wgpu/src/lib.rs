//! wgpu render backend for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod background;
pub mod camera;
mod environment_filter;
mod primitive;
pub mod taa;

#[cfg(feature = "viewer")]
mod overlay;
#[cfg(feature = "viewer")]
pub mod viewer;

pub use backend::WgpuRenderBackend;
pub use background::BackgroundRenderPass;
pub use camera::CameraOrbit;
pub use taa::TaaSettings;
#[cfg(feature = "viewer")]
pub use viewer::{InteractiveViewer, ViewerError};
