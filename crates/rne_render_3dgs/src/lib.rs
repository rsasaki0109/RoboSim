//! 3D Gaussian splat rendering adapter for RNE hybrid captures.

#![deny(missing_docs)]

mod camera;
mod splat;

pub use splat::{
    load_gaussian_splat_background, render_hybrid_scene_camera, GaussianSplatBackground,
    GaussianSplatError,
};
