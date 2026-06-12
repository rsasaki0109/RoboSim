//! Rapier backend implementation.

use crate::convert::{
    body_type_to_rapier, isometry_to_transform, shape_to_shared, transform_to_isometry,
    vec3_from_point, vec3_from_rapier, vec3_to_point, vec3_to_rapier,
};
use rapier3d::na::Vector3;
use rapier3d::pipeline::{PhysicsPipeline, QueryPipeline};
use rapier3d::prelude::*;
use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{
    Collider, ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
    PhysicsWorldId, RaycastHit, RaycastQuery, RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use std::collections::HashMap;

/// Rapier-backed physics simulation.
pub struct RapierBackend {
    worlds: HashMap<PhysicsWorldId, RapierWorldState>,
    next_world_id: u32,
    capabilities: Vec<PhysicsCapability>,
}

struct RapierWorldState {
    gravity: Vector3<f32>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhaseMultiSap,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    entity_to_body: HashMap<Entity, RigidBodyHandle>,
    body_to_entity: HashMap<RigidBodyHandle, Entity>,
    collider_to_entity: HashMap<ColliderHandle, Entity>,
    contacts: Vec<ContactEvent>,
}

impl RapierBackend {
    /// Creates a new Rapier backend with default capabilities.
    pub fn new() -> Self {
        Self {
            worlds: HashMap::new(),
            next_world_id: 0,
            capabilities: vec![
                PhysicsCapability::RigidBody,
                PhysicsCapability::RaycastBatch,
            ],
        }
    }

    fn world_mut(&mut self, id: PhysicsWorldId) -> Result<&mut RapierWorldState, PhysicsError> {
        self.worlds.get_mut(&id).ok_or(PhysicsError::WorldNotFound)
    }

    fn world(&self, id: PhysicsWorldId) -> Result<&RapierWorldState, PhysicsError> {
        self.worlds.get(&id).ok_or(PhysicsError::WorldNotFound)
    }
}

impl Default for RapierBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBackend for RapierBackend {
    type BodyHandle = RigidBodyHandle;
    type ColliderHandle = ColliderHandle;

    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        let id = PhysicsWorldId(self.next_world_id);
        self.next_world_id += 1;

        self.worlds.insert(
            id,
            RapierWorldState {
                gravity: vec3_to_rapier(desc.gravity_m_s2),
                integration_parameters: IntegrationParameters::default(),
                physics_pipeline: PhysicsPipeline::new(),
                island_manager: IslandManager::new(),
                broad_phase: BroadPhaseMultiSap::new(),
                narrow_phase: NarrowPhase::new(),
                bodies: RigidBodySet::new(),
                colliders: ColliderSet::new(),
                impulse_joints: ImpulseJointSet::new(),
                multibody_joints: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),
                query_pipeline: QueryPipeline::new(),
                entity_to_body: HashMap::new(),
                body_to_entity: HashMap::new(),
                collider_to_entity: HashMap::new(),
                contacts: Vec::new(),
            },
        );

        Ok(id)
    }

    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let state = self.world_mut(physics_world)?;

        for entity_ref in world.iter_entities() {
            let entity = entity_ref.id();
            let Some(transform) = world.get::<Transform3>(entity) else {
                continue;
            };
            let Some(rigid_body) = world.get::<RigidBody>(entity) else {
                continue;
            };
            let Some(collider) = world.get::<Collider>(entity) else {
                continue;
            };

            let isometry = transform_to_isometry(transform);

            if let Some(body_handle) = state.entity_to_body.get(&entity).copied() {
                if let Some(body) = state.bodies.get_mut(body_handle) {
                    body.set_position(isometry, true);
                    if rigid_body.body_type == RigidBodyType::Kinematic {
                        body.set_linvel(vec3_to_rapier(rigid_body.linear_velocity_m_s), true);
                    }
                }
                continue;
            }

            let mut builder = RigidBodyBuilder::new(body_type_to_rapier(rigid_body.body_type))
                .position(isometry)
                .additional_mass(rigid_body.mass_kg as f32);

            if rigid_body.body_type == RigidBodyType::Dynamic {
                builder = builder
                    .linvel(vec3_to_rapier(rigid_body.linear_velocity_m_s))
                    .angvel(vec3_to_rapier(rigid_body.angular_velocity_rad_s));
            }

            let body_handle = state.bodies.insert(builder.build());
            let collider_handle = state.colliders.insert_with_parent(
                ColliderBuilder::new(shape_to_shared(collider.shape))
                    .friction(collider.material.friction)
                    .restitution(collider.material.restitution)
                    .build(),
                body_handle,
                &mut state.bodies,
            );

            state.entity_to_body.insert(entity, body_handle);
            state.body_to_entity.insert(body_handle, entity);
            state.collider_to_entity.insert(collider_handle, entity);
        }

        Ok(())
    }

    fn step(&mut self, physics_world: PhysicsWorldId, dt: SimDuration) -> Result<(), PhysicsError> {
        let state = self.world_mut(physics_world)?;
        state.integration_parameters.dt = dt.as_seconds().value() as f32;
        state.contacts.clear();

        state.physics_pipeline.step(
            &state.gravity,
            &state.integration_parameters,
            &mut state.island_manager,
            &mut state.broad_phase,
            &mut state.narrow_phase,
            &mut state.bodies,
            &mut state.colliders,
            &mut state.impulse_joints,
            &mut state.multibody_joints,
            &mut state.ccd_solver,
            Some(&mut state.query_pipeline),
            &(),
            &(),
        );

        state.query_pipeline.update(&state.colliders);

        for contact_pair in state.narrow_phase.contact_pairs() {
            if !contact_pair.has_any_active_contact {
                continue;
            }
            let Some(entity_a) = state
                .collider_to_entity
                .get(&contact_pair.collider1)
                .copied()
            else {
                continue;
            };
            let Some(entity_b) = state
                .collider_to_entity
                .get(&contact_pair.collider2)
                .copied()
            else {
                continue;
            };
            let normal = contact_pair
                .manifolds
                .first()
                .map(|manifold| vec3_from_rapier(manifold.local_n1))
                .unwrap_or(Vec3::Y);

            state.contacts.push(ContactEvent {
                entity_a,
                entity_b,
                normal,
            });
        }

        Ok(())
    }

    fn sync_to_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        let state = self.world(physics_world)?;

        for (entity, body_handle) in &state.entity_to_body {
            let Some(body) = state.bodies.get(*body_handle) else {
                continue;
            };
            if body.body_type() != rapier3d::prelude::RigidBodyType::Dynamic {
                continue;
            }

            if let Some(mut transform) = world.get_mut::<Transform3>(*entity) {
                let updated = isometry_to_transform(body.position());
                transform.translation = updated.translation;
                transform.rotation = updated.rotation;
            }
        }

        Ok(())
    }

    fn raycast(
        &self,
        physics_world: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let state = self.world(physics_world)?;
        let origin = vec3_to_point(query.origin_m);
        let direction = vec3_to_rapier(query.direction);
        if direction.norm_squared() <= f32::EPSILON {
            return Ok(Vec::new());
        }

        let ray = Ray::new(origin, direction.normalize());
        let filter = QueryFilter::default();
        let Some((collider_handle, intersection)) = state.query_pipeline.cast_ray_and_get_normal(
            &state.bodies,
            &state.colliders,
            &ray,
            query.max_distance_m as f32,
            true,
            filter,
        ) else {
            return Ok(Vec::new());
        };

        let Some(entity) = state.collider_to_entity.get(&collider_handle).copied() else {
            return Ok(Vec::new());
        };

        Ok(vec![RaycastHit {
            entity,
            point_m: vec3_from_point(ray.point_at(intersection.time_of_impact)),
            normal: vec3_from_rapier(intersection.normal),
            distance_m: intersection.time_of_impact as f64,
        }])
    }

    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&self.world(physics_world)?.contacts)
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        &self.capabilities
    }
}

/// Runs one full physics update: sync from ECS, step, sync to ECS.
pub fn step_physics(
    backend: &mut RapierBackend,
    world: &mut World,
    physics_world: PhysicsWorldId,
    dt: SimDuration,
) -> Result<(), PhysicsError> {
    backend.sync_from_ecs(world, physics_world)?;
    backend.step(physics_world, dt)?;
    backend.sync_to_ecs(world, physics_world)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::spawn_named;
    use rne_math::Quat;
    use rne_physics::{hash_physics_state, ColliderShape};

    fn fixed_step() -> SimDuration {
        SimDuration::from_hertz(rne_math::Hertz::new(60.0))
    }

    fn setup_world() -> (RapierBackend, PhysicsWorldId, World, Entity, Entity) {
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");

        let mut world = World::new();
        let ground = spawn_named(&mut world, "ground");
        world.entity_mut(ground).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(10.0, 0.5, 10.0),
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        ));

        let cube = spawn_named(&mut world, "cube");
        world.entity_mut(cube).insert((
            RigidBody::default(),
            Collider::cuboid(Vec3::splat(0.5)),
            Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
        ));

        (backend, physics_world, world, ground, cube)
    }

    #[test]
    fn falling_cube_moves_downward() {
        let (mut backend, physics_world, mut world, _, cube) = setup_world();
        let dt = fixed_step();

        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        for _ in 0..30 {
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
        }

        let y = world
            .get::<Transform3>(cube)
            .expect("cube transform")
            .translation
            .y;
        assert!(y < 5.0, "cube should fall from initial height, y={y}");
        assert!(y > 0.0, "cube should rest above ground, y={y}");
    }

    #[test]
    fn raycast_hits_ground() {
        let (mut backend, physics_world, mut world, ground, _) = setup_world();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, fixed_step()).unwrap();

        let hits = backend
            .raycast(
                physics_world,
                RaycastQuery::downward(Vec3::new(3.0, 10.0, 0.0), 20.0),
            )
            .expect("raycast");

        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity, ground);
        assert_relative_eq!(hits[0].point_m.y, 0.0, epsilon = 0.1);
    }

    #[test]
    fn deterministic_1000_step_hash() {
        let (mut backend, physics_world, mut world, _, _) = setup_world();
        let dt = fixed_step();

        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        for _ in 0..1000 {
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
        }

        let hash_a = hash_physics_state(&world);

        let (mut backend, physics_world, mut world, _, _) = setup_world();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        for _ in 0..1000 {
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
        }

        let hash_b = hash_physics_state(&world);
        assert_eq!(hash_a, hash_b, "physics replay should be deterministic");
        assert_ne!(hash_a, 0, "hash should reflect simulated state");
    }
}
