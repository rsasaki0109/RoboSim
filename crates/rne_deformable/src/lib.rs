//! Deterministic backend-neutral cable and cloth dynamics.

#![deny(missing_docs)]

pub mod collision;
pub mod components;
pub mod resources;
pub mod systems;

pub use collision::DeformableCollider;
pub use components::{
    CableSegment, CableSpec, ClothSpec, ConstraintKind, DeformableBody, DeformableKind,
    DeformableMaterial, DeformableSurfaceMesh, DeformableVisual, DistanceConstraint, Particle,
    PinConstraint, TriangleTopology,
};
pub use resources::{DeformableSolverConfig, DeformableStepError};
pub use systems::{build_cable, build_cloth, step_deformable, step_deformable_world};
