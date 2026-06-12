//! Render backend traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod camera;
pub mod depth;
pub mod headless;
pub mod image;
pub mod pass;
pub mod scene;
pub mod visual;

pub use backend::{RenderBackend, RenderError};
pub use camera::Camera;
pub use depth::{hash_depth_f32, DepthFrame};
pub use headless::HeadlessRenderBackend;
pub use image::{hash_rgba8, ImageFrame, RenderTarget};
pub use pass::CameraPassOutput;
pub use scene::{RenderScene, RenderSceneItem};
pub use visual::{Visual, VisualShape};
