//! Render backend traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod camera;
pub mod headless;
pub mod image;

pub use backend::{RenderBackend, RenderError};
pub use camera::Camera;
pub use headless::HeadlessRenderBackend;
pub use image::{hash_rgba8, ImageFrame, RenderTarget};
