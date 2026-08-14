//! Deterministic analytic physics backend for Robot Native Engine.
//!
//! [`AnalyticBackend`] is a second open physics backend that implements the
//! backend-neutral [`PhysicsBackend`] trait with collision-free dynamics: dynamic
//! rigid bodies integrate linear velocity and position under gravity with an
//! explicit semi-implicit Euler step. There are no contacts, joints, or
//! collisions, which makes it a fast, deterministic backend for planning and
//! policy iteration where contact response is not needed.
//!
//! The backend declares [`PhysicsCapability::RigidBody`],
//! [`PhysicsCapability::KinematicBody`], and
//! [`PhysicsCapability::DeterministicStep`]; runs that require articulation or
//! contact force must negotiate a different backend.

#![deny(missing_docs)]

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{
    ContactEvent, PhysicsBackend, PhysicsBackendManifest, PhysicsBackendRepeatability,
    PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId, RaycastHit, RaycastQuery,
    RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use std::collections::HashMap;

/// Backend-agnostic capability set provided by the analytic backend.
const CAPABILITIES: &[PhysicsCapability] = &[
    PhysicsCapability::RigidBody,
    PhysicsCapability::DeterministicStep,
    PhysicsCapability::KinematicBody,
];

/// One integrated dynamic body inside an [`AnalyticWorld`].
#[derive(Clone, Copy, Debug)]
struct AnalyticBody {
    /// ECS entity whose transform this body drives.
    entity: Entity,
    /// Motion type; only [`RigidBodyType::Dynamic`] bodies integrate.
    body_type: RigidBodyType,
    /// Integrated world position in metres.
    position: Vec3,
    /// Integrated linear velocity in metres per second.
    velocity: Vec3,
}

/// Per-world analytic state.
#[derive(Clone, Debug, Default)]
struct AnalyticWorld {
    /// Gravity vector in metres per second squared.
    gravity_m_s2: Vec3,
    /// Integrated dynamic bodies, sorted by entity index.
    bodies: Vec<AnalyticBody>,
}

impl AnalyticWorld {
    fn step(&mut self, dt_s: f64) {
        for body in &mut self.bodies {
            if body.body_type != RigidBodyType::Dynamic {
                continue;
            }
            body.velocity += self.gravity_m_s2 * dt_s;
            body.position += body.velocity * dt_s;
        }
    }
}

/// Deterministic, collision-free rigid body backend.
///
/// The backend holds integrated body state between steps and writes transforms
/// back to ECS on [`PhysicsBackend::sync_to_ecs`]. Contacts and raycasts always
/// return empty, and the backend does not model articulation.
#[derive(Default)]
pub struct AnalyticBackend {
    worlds: HashMap<PhysicsWorldId, AnalyticWorld>,
    next_world_id: u32,
    contacts: Vec<ContactEvent>,
}

impl AnalyticBackend {
    /// Creates an analytic backend with no worlds.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the versioned conformance manifest for this backend.
    pub fn manifest() -> PhysicsBackendManifest {
        PhysicsBackendManifest::new(
            "analytic",
            env!("CARGO_PKG_VERSION"),
            "rne_analytic",
            "semi_implicit_euler_v1",
            CAPABILITIES.iter().copied(),
            PhysicsBackendRepeatability::SameRuntimeExact,
        )
        .expect("the built-in analytic backend manifest is valid")
    }

    fn world(&self, physics_world: PhysicsWorldId) -> Result<&AnalyticWorld, PhysicsError> {
        self.worlds
            .get(&physics_world)
            .ok_or(PhysicsError::WorldNotFound)
    }

    fn world_mut(
        &mut self,
        physics_world: PhysicsWorldId,
    ) -> Result<&mut AnalyticWorld, PhysicsError> {
        self.worlds
            .get_mut(&physics_world)
            .ok_or(PhysicsError::WorldNotFound)
    }
}

impl PhysicsBackend for AnalyticBackend {
    type BodyHandle = ();
    type ColliderHandle = ();

    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        let id = PhysicsWorldId(self.next_world_id);
        self.next_world_id += 1;
        self.worlds.insert(
            id,
            AnalyticWorld {
                gravity_m_s2: desc.gravity_m_s2,
                bodies: Vec::new(),
            },
        );
        Ok(id)
    }

    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let mut bodies = world
            .iter_entities()
            .filter_map(|entity_ref| {
                let entity = entity_ref.id();
                let body = world.get::<RigidBody>(entity)?;
                let transform = world.get::<Transform3>(entity)?;
                Some((entity.index(), entity, *body, transform.translation))
            })
            .collect::<Vec<_>>();
        bodies.sort_unstable_by_key(|(index, _, _, _)| *index);
        self.world_mut(physics_world)?.bodies = bodies
            .into_iter()
            .map(|(_, entity, body, position)| AnalyticBody {
                entity,
                body_type: body.body_type,
                position,
                velocity: body.linear_velocity_m_s,
            })
            .collect();
        Ok(())
    }

    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError> {
        let world_state = self.world_mut(physics_world)?;
        let dt_s = dt.as_seconds().value();
        world_state.step(dt_s);
        self.contacts.clear();
        Ok(())
    }

    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let bodies = self.world(physics_world)?.bodies.clone();
        for body in &bodies {
            if let Some(mut transform) = world.get_mut::<Transform3>(body.entity) {
                transform.translation = body.position;
            }
            if let Some(mut rigid_body) = world.get_mut::<RigidBody>(body.entity) {
                rigid_body.linear_velocity_m_s = body.velocity;
            }
        }
        Ok(())
    }

    fn raycast(
        &self,
        _physics_world: PhysicsWorldId,
        _query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        Ok(Vec::new())
    }

    fn contacts(&self, _physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&self.contacts)
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        CAPABILITIES
    }
}

/// Steps the analytic backend for a fixed simulation duration.
///
/// The [`PhysicsBackend::step`] method integrates each world's dynamic bodies;
/// callers sync from ECS once up front and call this helper per step (which
/// syncs the integrated transforms back to ECS without re-syncing velocities).
pub fn step_physics(
    backend: &mut AnalyticBackend,
    world: &mut World,
    physics_world: PhysicsWorldId,
    dt: SimDuration,
) -> Result<(), PhysicsError> {
    backend.step(physics_world, dt)?;
    backend.sync_to_ecs(world, physics_world)
}
