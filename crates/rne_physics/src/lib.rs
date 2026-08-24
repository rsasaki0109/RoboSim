//! Physics backend traits and ECS components for Robot Native Engine.

#![deny(missing_docs)]

pub mod backend;
pub mod components;
pub mod events;
pub mod hash;
pub mod snapshot;

pub use backend::{
    require_capabilities, PhysicsBackend, PhysicsBackendManifest, PhysicsBackendManifestError,
    PhysicsBackendRepeatability, PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
    PHYSICS_BACKEND_MANIFEST_SCHEMA_VERSION, PHYSICS_CONFORMANCE_REPORT_SCHEMA_VERSION,
    PHYSICS_TOLERANCE_REGISTRY_VERSION,
};
pub use components::{
    Collider, ColliderShape, CollisionGroups, FixedJointDesc, JointActuation, JointMotor,
    JointMotorGainModel, JointState, MultibodyLink, PhysicsMaterial, PrismaticJointDesc,
    RevoluteJointDesc, RigidBody, RigidBodyInertia, RigidBodyType,
};
pub use events::{ContactEvent, RaycastHit, RaycastQuery};
pub use hash::hash_physics_state;
pub use snapshot::{
    capture_physics_snapshot, PhysicsBodySnapshot, PhysicsContactSnapshot, PhysicsSnapshot,
    PhysicsSnapshotError, PHYSICS_SNAPSHOT_SCHEMA_VERSION,
};
