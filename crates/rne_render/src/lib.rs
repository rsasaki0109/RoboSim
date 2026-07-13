//! Render backend traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod camera;
pub mod depth;
pub mod headless;
pub mod image;
pub mod lidar;
pub mod mesh;
pub mod mesh_cache;
pub mod pass;
pub mod path;
pub mod scene;
pub mod visual;

pub use backend::{RenderBackend, RenderError};
pub use camera::Camera;
pub use depth::{hash_depth_f32, scene_depth_probe, DepthFrame};
pub use headless::HeadlessRenderBackend;
pub use image::{hash_rgba8, ImageFrame, RenderTarget};
pub use mesh::{load_mesh, load_stl, load_stl_bytes, MeshLoadError, TriangleMesh};
pub use mesh_cache::MeshRenderCache;
pub use pass::CameraPassOutput;
pub use path::resolve_package_uri;
pub use scene::{RenderScene, RenderSceneItem};
pub use visual::{LinkVisuals, Visual, VisualShape};
