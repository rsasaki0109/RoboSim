//! Render backend traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod animation;
pub mod backend;
pub mod camera;
pub mod depth;
pub mod environment;
pub mod gaussian_splat;
pub mod gaussian_splat_validation;
pub mod headless;
pub mod image;
pub mod lidar;
pub mod material;
pub mod mesh;
pub mod mesh_cache;
pub mod pass;
pub mod path;
pub mod scene;
pub mod visual;

pub use animation::{
    AnimationChannel, AnimationClip, AnimationInterpolation, AnimationProperty,
    AnimationSampleError, GltfAnimationPlayer, GltfNode, GltfSceneAsset, GltfScenePart, GltfSkin,
    GltfSkinJoint, SkinWeights, SkinningData,
};
pub use backend::{RenderBackend, RenderError};
pub use camera::Camera;
pub use depth::{hash_depth_f32, scene_depth_probe, DepthFrame};
pub use environment::{EnvironmentLighting, EnvironmentMap, EnvironmentMapError};
pub use gaussian_splat::{
    load_gaussian_splat_manifest, load_gaussian_splat_manifest_with_override,
    tsukuba_confirmation_splat_manifest_path, tsukuba_kenkyugakuen_splat_manifest_path,
    validate_gaussian_splat_manifest, validate_gaussian_splat_manifest_with_override,
    GaussianSplatCaptureReport, GaussianSplatEnvironment, GaussianSplatError, HybridRenderScene,
    GAUSSIAN_SPLAT_RENDERER_ID_V1,
};
pub use gaussian_splat_validation::{
    audit_gaussian_splat_validation_fixture, compare_registered_gaussian_splat_observations,
    gaussian_splat_observation_tolerances, require_qualifying_gaussian_splat_fixture,
    GaussianSplatObservationMetrics, GaussianSplatObservationTolerances,
    GaussianSplatValidationAudit, GaussianSplatValidationError,
};
pub use headless::HeadlessRenderBackend;
pub use image::{hash_rgba8, ImageFrame, RenderTarget};
pub use material::PbrMaterial;
pub use mesh::{
    load_gltf_scene, load_mesh, load_mesh_parts, load_stl, load_stl_bytes, LoadedMeshPart,
    MeshLoadError, TriangleMesh,
};
pub use mesh_cache::MeshRenderCache;
pub use pass::CameraPassOutput;
pub use path::resolve_package_uri;
pub use scene::{RenderScene, RenderSceneItem};
pub use visual::{LinkVisuals, Visual, VisualShape};
