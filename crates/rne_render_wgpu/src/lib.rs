//! wgpu render backend for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod camera;
mod primitive;
pub mod taa;

#[cfg(feature = "viewer")]
mod overlay;
#[cfg(feature = "viewer")]
pub mod viewer;

pub use backend::WgpuRenderBackend;
pub use camera::CameraOrbit;
pub use taa::TaaSettings;
#[cfg(feature = "viewer")]
pub use viewer::{InteractiveViewer, ViewerError};
