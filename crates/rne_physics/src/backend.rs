//! Physics backend trait and world identifiers.

use crate::{ContactEvent, RaycastHit, RaycastQuery};
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::Vec3;
use thiserror::Error;

/// Identifier for a backend-owned physics world instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PhysicsWorldId(pub u32);

impl PhysicsWorldId {
    /// Default physics world identifier.
    pub const DEFAULT: Self = Self(0);
}

/// Initial configuration for a physics world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsWorldDesc {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// Constraint solver iterations per step. `0` uses the backend default; a higher
    /// value stabilizes stiff articulated chains (several jointed links) at extra cost.
    pub solver_iterations: usize,
}

impl Default for PhysicsWorldDesc {
    fn default() -> Self {
        Self {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            solver_iterations: 0,
        }
    }
}

/// Optional physics backend capabilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsCapability {
    /// Supports rigid body simulation.
    RigidBody,
    /// Supports articulated bodies.
    Articulation,
    /// Supports GPU rigid body simulation.
    GpuRigidBody,
    /// Supports deterministic stepping.
    DeterministicStep,
    /// Supports soft bodies.
    SoftBody,
    /// Supports contact force reporting.
    ContactForce,
    /// Supports batched raycasts.
    RaycastBatch,
}

/// Physics backend error type.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum PhysicsError {
    /// Requested physics world does not exist.
    #[error("physics world not found")]
    WorldNotFound,
    /// Backend failed to initialize.
    #[error("physics backend initialization failed")]
    InitializationFailed,
}

/// Backend-agnostic physics simulation interface.
pub trait PhysicsBackend: Send + Sync + 'static {
    /// Opaque rigid body handle type.
    type BodyHandle: Copy + Send + Sync + std::fmt::Debug;
    /// Opaque collider handle type.
    type ColliderHandle: Copy + Send + Sync + std::fmt::Debug;

    /// Creates a new physics world and returns its identifier.
    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError>;

    /// Synchronizes ECS state into the physics world.
    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError>;

    /// Advances the physics simulation by one fixed step.
    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError>;

    /// Synchronizes physics state back into ECS transforms.
    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError>;

    /// Executes a raycast query.
    fn raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError>;

    /// Returns contact events from the last simulation step.
    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError>;

    /// Returns supported capabilities for this backend.
    fn capabilities(&self) -> &[PhysicsCapability];
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::{Collider, RigidBody};
    use rne_ecs::spawn_named;
    use rne_world::Transform3;

    struct MockBackend {
        worlds: Vec<PhysicsWorldDesc>,
        contacts: Vec<ContactEvent>,
    }

    impl MockBackend {
        fn new() -> Self {
            Self {
                worlds: Vec::new(),
                contacts: Vec::new(),
            }
        }
    }

    impl PhysicsBackend for MockBackend {
        type BodyHandle = u32;
        type ColliderHandle = u32;

        fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
            self.worlds.push(desc);
            Ok(PhysicsWorldId(self.worlds.len() as u32 - 1))
        }

        fn sync_from_ecs(
            &mut self,
            world: &mut World,
            _physics_world: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            let _count = world
                .query::<(&RigidBody, &Collider, &Transform3)>()
                .iter(world)
                .count();
            Ok(())
        }

        fn step(
            &mut self,
            _physics_world: PhysicsWorldId,
            _dt: SimDuration,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn sync_to_ecs(
            &mut self,
            _world: &mut World,
            _physics_world: PhysicsWorldId,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn raycast(
            &self,
            _physics_world: PhysicsWorldId,
            _query: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            Ok(Vec::new())
        }

        fn contacts(
            &self,
            _physics_world: PhysicsWorldId,
        ) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&self.contacts)
        }

        fn capabilities(&self) -> &[PhysicsCapability] {
            &[PhysicsCapability::RigidBody]
        }
    }

    #[test]
    fn mock_backend_registers_world_and_syncs_entities() {
        let mut backend = MockBackend::new();
        let world_id = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("world");
        assert_eq!(world_id, PhysicsWorldId(0));

        let mut world = World::new();
        let entity = spawn_named(&mut world, "cube");
        world.entity_mut(entity).insert((
            RigidBody::default(),
            Collider::default(),
            Transform3::default(),
        ));

        backend
            .sync_from_ecs(&mut world, world_id)
            .expect("sync from ecs");
        backend
            .step(
                world_id,
                SimDuration::from_hertz(rne_math::Hertz::new(60.0)),
            )
            .expect("step");
        backend
            .sync_to_ecs(&mut world, world_id)
            .expect("sync to ecs");
    }
}
