//! Backend-neutral collision primitives used by deformable particles.

use rne_math::Transform3;
use rne_physics::ColliderShape;

/// Fixed or kinematically sampled collider used for one-way particle contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeformableCollider {
    /// Collider shape without backend-specific handles.
    pub shape: ColliderShape,
    /// World-space collider pose.
    pub world_transform: Transform3,
    /// Coulomb friction coefficient.
    pub friction: f64,
}
