//! 3D Gaussian splat rendering adapter for RNE hybrid captures.

#![deny(missing_docs)]

mod camera;
mod depth;
mod splat;
mod validation;

pub use depth::{
    composite_mesh_and_splat_depth, splat_proxy_depth_from_gaussians, splat_proxy_depth_from_ply,
};
pub use splat::{
    load_gaussian_splat_background, render_hybrid_scene_camera, GaussianSplatBackground,
    GaussianSplatError,
};
pub use validation::{
    validate_registered_splat_depth, RegisteredSplatDepthLandmark, RegisteredSplatDepthReport,
    RegisteredSplatDepthTolerances,
};
