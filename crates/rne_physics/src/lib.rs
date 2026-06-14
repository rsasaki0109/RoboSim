//! Physics backend traits and ECS components for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod components;
pub mod events;
pub mod hash;

pub use backend::{
    PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
};
pub use components::{
    Collider, ColliderShape, FixedJointDesc, JointMotor, PhysicsMaterial, PrismaticJointDesc,
    RevoluteJointDesc, RigidBody, RigidBodyType,
};
pub use events::{ContactEvent, RaycastHit, RaycastQuery};
pub use hash::hash_physics_state;
