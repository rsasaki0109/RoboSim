//! Rapier backend implementation.

use crate::convert::{
    body_type_to_rapier, isometry_to_transform, quat_to_rapier, shape_to_shared,
    transform_to_isometry, vec3_from_point, vec3_from_rapier, vec3_to_point, vec3_to_rapier,
};
use rapier3d::na::{Translation3, Unit, UnitQuaternion, Vector3};
use rapier3d::pipeline::{PhysicsPipeline, QueryPipeline};
use rapier3d::prelude::*;
use rne_core::SimDuration;
use rne_ecs::Parent;
use rne_ecs::{Entity, World};
use rne_math::Transform3 as MathTransform3;
use rne_math::Vec3;
use rne_physics::{
    Collider, ContactEvent, FixedJointDesc, JointMotor, MultibodyLink, PhysicsBackend,
    PhysicsCapability, PhysicsError, PhysicsWorldDesc, PhysicsWorldId, PrismaticJointDesc,
    RaycastHit, RaycastQuery, RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_world::{world_transform_of, Transform3};
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
    entity_to_collider: HashMap<Entity, ColliderHandle>,
    collider_to_entity: HashMap<ColliderHandle, Entity>,
    entity_to_joint: HashMap<Entity, ImpulseJointHandle>,
    entity_to_multibody_joint: HashMap<Entity, MultibodyJointHandle>,
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
                PhysicsCapability::Articulation,
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
                // A higher solver-iteration count (set per world) keeps stiff articulated
                // chains — e.g. the lift robot's multi-link arm — stable instead of swinging
                // chaotically; `0` keeps Rapier's default so existing robots are unchanged.
                integration_parameters: match std::num::NonZeroUsize::new(desc.solver_iterations) {
                    Some(iterations) => IntegrationParameters {
                        num_solver_iterations: iterations,
                        ..IntegrationParameters::default()
                    },
                    None => IntegrationParameters::default(),
                },
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
                entity_to_collider: HashMap::new(),
                collider_to_entity: HashMap::new(),
                entity_to_joint: HashMap::new(),
                entity_to_multibody_joint: HashMap::new(),
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

        // Iterate in a stable entity order so Rapier handle assignment and the
        // resulting solver order are deterministic regardless of ECS archetype
        // layout (see AGENTS.md determinism requirements).
        for entity in sorted_entities(world) {
            let transform = world_transform_of(world, entity);
            let Some(rigid_body) = world.get::<RigidBody>(entity) else {
                continue;
            };
            let collider = world.get::<Collider>(entity);
            if collider.is_none() && world.get::<MultibodyLink>(entity).is_none() {
                continue;
            }

            let isometry = transform_to_isometry(&transform);

            if let Some(body_handle) = state.entity_to_body.get(&entity).copied() {
                if let Some(body) = state.bodies.get_mut(body_handle) {
                    body.set_position(isometry, true);
                    if rigid_body.body_type != RigidBodyType::Fixed {
                        body.set_linvel(vec3_to_rapier(rigid_body.linear_velocity_m_s), true);
                        body.set_angvel(vec3_to_rapier(rigid_body.angular_velocity_rad_s), true);
                    }
                }
                sync_entity_collider(world, state, entity, body_handle, collider);
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
            state.entity_to_body.insert(entity, body_handle);
            state.body_to_entity.insert(body_handle, entity);
            if let Some(collider) = collider {
                let collider_handle = state.colliders.insert_with_parent(
                    ColliderBuilder::new(shape_to_shared(collider.shape))
                        .position(transform_to_isometry(&collider.local_offset))
                        .friction(collider.material.friction)
                        .restitution(collider.material.restitution)
                        .sensor(collider.sensor)
                        .collision_groups({
                            let groups = world
                                .get::<rne_physics::CollisionGroups>(entity)
                                .copied()
                                .unwrap_or_default();
                            InteractionGroups::new(
                                Group::from_bits_truncate(groups.memberships),
                                Group::from_bits_truncate(groups.filter),
                            )
                        })
                        .build(),
                    body_handle,
                    &mut state.bodies,
                );
                state.entity_to_collider.insert(entity, collider_handle);
                state.collider_to_entity.insert(collider_handle, entity);
            }
        }

        sync_joints_from_ecs(world, state)?;

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
            let impulse = contact_pair.total_impulse_magnitude();

            state.contacts.push(ContactEvent {
                entity_a,
                entity_b,
                normal,
                impulse,
            });
        }

        for (collider_a, collider_b, intersecting) in state.narrow_phase.intersection_pairs() {
            if !intersecting {
                continue;
            }
            let Some(entity_a) = state.collider_to_entity.get(&collider_a).copied() else {
                continue;
            };
            let Some(entity_b) = state.collider_to_entity.get(&collider_b).copied() else {
                continue;
            };
            state.contacts.push(ContactEvent {
                entity_a,
                entity_b,
                normal: Vec3::ZERO,
                impulse: 0.0,
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

        // Write back in a stable parent-before-child order (entities are created
        // root-first, so ascending id keeps a parent's transform fresh before a
        // child reads it for its local-frame conversion). This avoids the
        // order-dependent drift that HashMap iteration would introduce.
        let mut bodies: Vec<(Entity, RigidBodyHandle)> = state
            .entity_to_body
            .iter()
            .map(|(entity, handle)| (*entity, *handle))
            .collect();
        bodies.sort_unstable_by_key(|(entity, _)| *entity);

        for (entity, body_handle) in bodies {
            let Some(body) = state.bodies.get(body_handle) else {
                continue;
            };
            if body.body_type() != rapier3d::prelude::RigidBodyType::Dynamic {
                continue;
            }

            let world_tf = isometry_to_transform(body.position());
            let parent_entity = world.get::<Parent>(entity).map(|parent| parent.0);
            let local_tf = if let Some(parent_entity) = parent_entity {
                let parent_world = world_transform_of(world, parent_entity);
                world_to_local_transform(&parent_world, &world_tf)
            } else {
                world_tf
            };
            if let Some(mut transform) = world.get_mut::<Transform3>(entity) {
                *transform = local_tf;
            }
            if let Some(mut rigid_body) = world.get_mut::<RigidBody>(entity) {
                rigid_body.linear_velocity_m_s = vec3_from_rapier(*body.linvel());
                rigid_body.angular_velocity_rad_s = vec3_from_rapier(*body.angvel());
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

fn sync_entity_collider(
    world: &World,
    state: &mut RapierWorldState,
    entity: Entity,
    body_handle: RigidBodyHandle,
    collider: Option<&Collider>,
) {
    let existing = state.entity_to_collider.get(&entity).copied();
    match (existing, collider) {
        (None, Some(collider)) => {
            let handle = state.colliders.insert_with_parent(
                collider_builder(world, entity, collider).build(),
                body_handle,
                &mut state.bodies,
            );
            state.entity_to_collider.insert(entity, handle);
            state.collider_to_entity.insert(handle, entity);
        }
        (Some(handle), Some(collider)) => {
            if let Some(existing) = state.colliders.get_mut(handle) {
                existing.set_sensor(collider.sensor);
                existing.set_collision_groups(interaction_groups(world, entity));
            }
        }
        (Some(handle), None) => {
            state
                .colliders
                .remove(handle, &mut state.island_manager, &mut state.bodies, true);
            state.collider_to_entity.remove(&handle);
            state.entity_to_collider.remove(&entity);
        }
        (None, None) => {}
    }
}

fn collider_builder(world: &World, entity: Entity, collider: &Collider) -> ColliderBuilder {
    ColliderBuilder::new(shape_to_shared(collider.shape))
        .position(transform_to_isometry(&collider.local_offset))
        .friction(collider.material.friction)
        .restitution(collider.material.restitution)
        .sensor(collider.sensor)
        .collision_groups(interaction_groups(world, entity))
}

fn interaction_groups(world: &World, entity: Entity) -> InteractionGroups {
    let groups = world
        .get::<rne_physics::CollisionGroups>(entity)
        .copied()
        .unwrap_or_default();
    InteractionGroups::new(
        Group::from_bits_truncate(groups.memberships),
        Group::from_bits_truncate(groups.filter),
    )
}

/// Maximum torque a revolute joint motor may apply.
const REVOLUTE_MOTOR_MAX_FORCE: f32 = 50.0;
/// Maximum force a prismatic joint motor may apply. Higher than the revolute cap
/// so a vertical lift can hold a multi-link arm against gravity (~60 N) with the
/// motor gain still in a stable range.
const PRISMATIC_MOTOR_MAX_FORCE: f32 = 150.0;

/// Maximum motor force for a wired joint, selected by its driven axis.
fn motor_max_force(axis: JointAxis) -> f32 {
    match axis {
        JointAxis::LinX | JointAxis::LinY | JointAxis::LinZ => PRISMATIC_MOTOR_MAX_FORCE,
        _ => REVOLUTE_MOTOR_MAX_FORCE,
    }
}

/// Driven axis of a wired joint, selecting the motor degree of freedom.
fn motor_axis_for_entity(world: &World, entity: Entity) -> Option<JointAxis> {
    if world.get::<RevoluteJointDesc>(entity).is_some() {
        Some(JointAxis::AngX)
    } else if world.get::<PrismaticJointDesc>(entity).is_some() {
        Some(JointAxis::LinX)
    } else {
        None
    }
}

/// Returns true when the entity still carries any joint description component.
fn has_joint_desc(world: &World, entity: Entity) -> bool {
    world.get::<RevoluteJointDesc>(entity).is_some()
        || world.get::<PrismaticJointDesc>(entity).is_some()
        || world.get::<FixedJointDesc>(entity).is_some()
}

fn normalized_axis(axis: Vec3) -> Unit<Vector3<f32>> {
    let axis = vec3_to_rapier(axis);
    if axis.norm_squared() <= f32::EPSILON {
        Vector3::y_axis()
    } else {
        Unit::new_normalize(axis)
    }
}

/// Returns all live entities sorted by id for deterministic backend iteration.
fn sorted_entities(world: &World) -> Vec<Entity> {
    let mut entities: Vec<Entity> = world.iter_entities().map(|entity| entity.id()).collect();
    entities.sort_unstable();
    entities
}

fn sync_joints_from_ecs(world: &World, state: &mut RapierWorldState) -> Result<(), PhysicsError> {
    // Release wired joints whose description component was removed (e.g. a grasp
    // weld dropped when the gripper opens). Drop in a stable order for determinism.
    let mut detached: Vec<Entity> = state
        .entity_to_joint
        .keys()
        .copied()
        .filter(|entity| !has_joint_desc(world, *entity))
        .collect();
    detached.sort_unstable();
    for entity in detached {
        if let Some(handle) = state.entity_to_joint.remove(&entity) {
            state.impulse_joints.remove(handle, true);
        }
    }
    let mut detached_multibody: Vec<Entity> = state
        .entity_to_multibody_joint
        .keys()
        .copied()
        .filter(|entity| !has_joint_desc(world, *entity))
        .collect();
    detached_multibody.sort_unstable();
    for entity in detached_multibody {
        if let Some(handle) = state.entity_to_multibody_joint.remove(&entity) {
            state.multibody_joints.remove(handle, true);
        }
    }

    for entity in sorted_entities(world) {
        if state.entity_to_joint.contains_key(&entity)
            || state.entity_to_multibody_joint.contains_key(&entity)
        {
            continue;
        }

        let (parent, joint, motor_axis) = if let Some(desc) = world.get::<RevoluteJointDesc>(entity)
        {
            let mut builder = RevoluteJointBuilder::new(normalized_axis(desc.axis))
                .local_anchor1(vec3_to_point(desc.anchor_parent_m))
                .local_anchor2(vec3_to_point(desc.anchor_child_m));
            if let (Some(lower_rad), Some(upper_rad)) = (desc.lower_rad, desc.upper_rad) {
                builder = builder.limits([lower_rad as f32, upper_rad as f32]);
            }
            let joint = builder.build();
            (
                desc.parent,
                GenericJoint::from(joint),
                Some(JointAxis::AngX),
            )
        } else if let Some(desc) = world.get::<PrismaticJointDesc>(entity) {
            let joint = PrismaticJointBuilder::new(normalized_axis(desc.axis))
                .local_anchor1(vec3_to_point(desc.anchor_parent_m))
                .local_anchor2(vec3_to_point(desc.anchor_child_m))
                .build();
            (
                desc.parent,
                GenericJoint::from(joint),
                Some(JointAxis::LinX),
            )
        } else if let Some(desc) = world.get::<FixedJointDesc>(entity) {
            // Lock the child at its current relative pose: parent frame carries the
            // relative rotation, child frame is the identity at its anchor.
            let frame1 = Isometry::from_parts(
                Translation3::from(vec3_to_rapier(desc.anchor_parent_m)),
                quat_to_rapier(desc.relative_rotation),
            );
            let frame2 = Isometry::from_parts(
                Translation3::from(vec3_to_rapier(desc.anchor_child_m)),
                UnitQuaternion::identity(),
            );
            let joint = FixedJointBuilder::new()
                .local_frame1(frame1)
                .local_frame2(frame2)
                .build();
            (desc.parent, GenericJoint::from(joint), None)
        } else {
            continue;
        };

        let Some(parent_body) = state.entity_to_body.get(&parent).copied() else {
            continue;
        };
        let Some(child_body) = state.entity_to_body.get(&entity).copied() else {
            continue;
        };

        if world.get::<MultibodyLink>(entity).is_some() {
            let Some(handle) = state
                .multibody_joints
                .insert(parent_body, child_body, joint, true)
            else {
                continue;
            };
            if let (Some(motor_axis), Some((multibody, link_id))) =
                (motor_axis, state.multibody_joints.get_mut(handle))
            {
                if let Some(link) = multibody.link_mut(link_id) {
                    link.joint
                        .data
                        .set_motor_max_force(motor_axis, motor_max_force(motor_axis));
                }
            }
            state.entity_to_multibody_joint.insert(entity, handle);
            continue;
        }

        let handle = state
            .impulse_joints
            .insert(parent_body, child_body, joint, true);
        if let (Some(motor_axis), Some(joint)) = (motor_axis, state.impulse_joints.get_mut(handle))
        {
            joint
                .data
                .set_motor_max_force(motor_axis, motor_max_force(motor_axis));
        }
        state.entity_to_joint.insert(entity, handle);
    }

    Ok(())
}

fn apply_joint_motors(world: &World, state: &mut RapierWorldState) {
    for (entity, joint_handle) in &state.entity_to_joint {
        let Some(motor) = world.get::<JointMotor>(*entity) else {
            continue;
        };
        let Some(axis) = motor_axis_for_entity(world, *entity) else {
            continue;
        };
        let Some(joint) = state.impulse_joints.get_mut(*joint_handle) else {
            continue;
        };
        // With zero stiffness this is exactly a velocity motor (set_motor_velocity);
        // a positive stiffness adds a position spring that holds a load without drift.
        joint.data.set_motor(
            axis,
            motor.target_position as f32,
            motor.velocity_rad_s as f32,
            motor.stiffness as f32,
            motor.gain as f32,
        );
        // A per-motor force override takes precedence over the per-joint-type cap.
        if motor.max_force > 0.0 {
            joint.data.set_motor_max_force(axis, motor.max_force as f32);
        }
    }
    for (entity, joint_handle) in &state.entity_to_multibody_joint {
        let Some(motor) = world.get::<JointMotor>(*entity) else {
            continue;
        };
        let Some(axis) = motor_axis_for_entity(world, *entity) else {
            continue;
        };
        let Some((multibody, link_id)) = state.multibody_joints.get_mut(*joint_handle) else {
            continue;
        };
        let Some(link) = multibody.link_mut(link_id) else {
            continue;
        };
        link.joint.data.set_motor(
            axis,
            motor.target_position as f32,
            motor.velocity_rad_s as f32,
            motor.stiffness as f32,
            motor.gain as f32,
        );
        if motor.max_force > 0.0 {
            link.joint
                .data
                .set_motor_max_force(axis, motor.max_force as f32);
        }
    }
}

fn world_to_local_transform(parent_world: &Transform3, world_tf: &Transform3) -> Transform3 {
    let parent = to_math_transform(parent_world);
    let world = to_math_transform(world_tf);
    let local = parent.inverse().mul_transform(&world);
    from_math_transform(local)
}

fn to_math_transform(transform: &Transform3) -> MathTransform3 {
    MathTransform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

fn from_math_transform(transform: MathTransform3) -> Transform3 {
    Transform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
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
    {
        let state = backend.world_mut(physics_world)?;
        apply_joint_motors(world, state);
    }
    backend.step(physics_world, dt)?;
    backend.sync_to_ecs(world, physics_world)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::spawn_named;
    use rne_math::Quat;
    use rne_physics::{hash_physics_state, ColliderShape, CollisionGroups};

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
    fn collision_groups_disable_same_group_contacts() {
        let (mut backend, physics_world, mut world, ground, cube) = setup_world();
        let groups = CollisionGroups::without_self_collision(1);
        world.entity_mut(ground).insert(groups);
        world.entity_mut(cube).insert(groups);

        for _ in 0..90 {
            step_physics(&mut backend, &mut world, physics_world, fixed_step()).unwrap();
        }

        let y = world
            .get::<Transform3>(cube)
            .expect("cube transform")
            .translation
            .y;
        assert!(
            y < 0.0,
            "same-group filtering should let cube pass through ground, y={y}"
        );
    }

    #[test]
    fn runtime_sensor_and_collision_group_updates_report_force_free_overlap() {
        let (mut backend, physics_world, mut world, _, cube) = setup_world();
        let sensor = spawn_named(&mut world, "sensor");
        world.entity_mut(sensor).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::splat(0.5),
                },
                sensor: false,
                ..Collider::default()
            },
            Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
        ));
        let filtered = CollisionGroups::without_self_collision(1);
        world.entity_mut(sensor).insert(filtered);
        world.entity_mut(cube).insert(filtered);

        step_physics(&mut backend, &mut world, physics_world, fixed_step()).unwrap();
        assert!(backend.contacts(physics_world).unwrap().is_empty());

        world.entity_mut(sensor).insert(CollisionGroups::default());
        world.entity_mut(cube).insert(CollisionGroups::default());
        world
            .get_mut::<Collider>(sensor)
            .expect("sensor collider")
            .sensor = true;
        step_physics(&mut backend, &mut world, physics_world, fixed_step()).unwrap();

        let overlap = backend
            .contacts(physics_world)
            .unwrap()
            .iter()
            .find(|contact| {
                (contact.entity_a == sensor && contact.entity_b == cube)
                    || (contact.entity_a == cube && contact.entity_b == sensor)
            })
            .expect("sensor overlap event");
        assert_eq!(overlap.impulse, 0.0);
        assert_eq!(overlap.normal, Vec3::ZERO);
        let cube_transform = world.get::<Transform3>(cube).expect("cube transform");
        assert_eq!(cube_transform.translation.x, 0.0);
        assert_eq!(cube_transform.translation.z, 0.0);
        assert!(cube_transform.translation.y > 4.99);
    }

    #[test]
    fn resting_contact_impulse_matches_steady_state_weight() {
        // A box resting on the ground plane needs the ground's contact impulse to
        // balance gravity each step: impulse ≈ weight * dt = m * g * dt on average.
        // Verifies ContactEvent::impulse carries real solver data (not just a
        // placeholder zero) and is in the right ballpark once the cube has
        // settled. A single step's impulse is noisy (the TGS soft solver's bias
        // term over/under-corrects tick to tick even at rest), so this averages
        // over a settled window instead of asserting on one step.
        let (mut backend, physics_world, mut world, ground, cube) = setup_world();
        let dt = fixed_step();

        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        for _ in 0..240 {
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
            backend.sync_from_ecs(&mut world, physics_world).unwrap();
        }

        let mut samples = Vec::new();
        for _ in 0..60 {
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
            backend.sync_from_ecs(&mut world, physics_world).unwrap();

            let contacts = backend.contacts(physics_world).unwrap();
            let impulse = contacts
                .iter()
                .find(|contact| {
                    (contact.entity_a == ground && contact.entity_b == cube)
                        || (contact.entity_a == cube && contact.entity_b == ground)
                })
                .map(|contact| contact.impulse as f64)
                .expect("cube should be resting in contact with the ground");
            samples.push(impulse);
        }

        let mean_impulse = samples.iter().sum::<f64>() / samples.len() as f64;
        // `setup_world`'s cube has RigidBody::default().mass_kg == 1.0, but Rapier
        // treats that as ADDITIONAL mass on top of the mass its 1 m^3 collider
        // contributes at the engine's default density of 1.0 kg/m^3 (see
        // RigidBodyBuilder::additional_mass docs), so the body's real total mass
        // is 2.0 kg, not 1.0 kg.
        let total_mass_kg = 2.0;
        let expected_impulse = total_mass_kg * 9.81 * dt.as_seconds().value();
        assert!(
            mean_impulse > 0.0,
            "resting contact should carry a nonzero impulse"
        );
        assert!(
            (mean_impulse - expected_impulse).abs() < 0.5 * expected_impulse,
            "mean resting contact impulse {mean_impulse} should approximate steady-state weight*dt {expected_impulse}"
        );
    }

    #[test]
    fn sync_to_ecs_writes_dynamic_body_velocity() {
        let (mut backend, physics_world, mut world, _, cube) = setup_world();

        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, fixed_step()).unwrap();
        backend.sync_to_ecs(&mut world, physics_world).unwrap();

        let body = world.get::<RigidBody>(cube).expect("cube body");
        assert!(
            body.linear_velocity_m_s.y < 0.0,
            "falling cube should have downward velocity, got {:?}",
            body.linear_velocity_m_s
        );
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
    fn collider_can_be_added_to_and_removed_from_existing_multibody_link() {
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");
        let mut world = World::new();
        let link = spawn_named(&mut world, "tool_link");
        world.entity_mut(link).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            MultibodyLink,
            Transform3::from_translation_rotation(Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY),
        ));
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, fixed_step()).unwrap();
        assert!(backend
            .raycast(
                physics_world,
                RaycastQuery::downward(Vec3::new(0.0, 2.0, 0.0), 2.0),
            )
            .unwrap()
            .is_empty());

        world
            .entity_mut(link)
            .insert(Collider::cuboid(Vec3::splat(0.1)));
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, fixed_step()).unwrap();
        let hits = backend
            .raycast(
                physics_world,
                RaycastQuery::downward(Vec3::new(0.0, 2.0, 0.0), 2.0),
            )
            .unwrap();
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].entity, link);

        world.entity_mut(link).remove::<Collider>();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        backend.step(physics_world, fixed_step()).unwrap();
        assert!(backend
            .raycast(
                physics_world,
                RaycastQuery::downward(Vec3::new(0.0, 2.0, 0.0), 2.0),
            )
            .unwrap()
            .is_empty());
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

    #[test]
    fn fixed_joint_welds_then_releases_body() {
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");

        let mut world = World::new();
        // Fixed anchor in mid-air with a small collider away from the cube.
        let anchor = spawn_named(&mut world, "anchor");
        world.entity_mut(anchor).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::splat(0.05)),
            Transform3::from_translation_rotation(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
        ));

        // Dynamic cube beside the anchor, welded so it cannot fall.
        let cube = spawn_named(&mut world, "cube");
        world.entity_mut(cube).insert((
            RigidBody::default(),
            Collider::cuboid(Vec3::splat(0.1)),
            Transform3::from_translation_rotation(Vec3::new(0.5, 5.0, 0.0), Quat::IDENTITY),
            FixedJointDesc {
                parent: anchor,
                anchor_parent_m: Vec3::new(0.5, 0.0, 0.0),
                anchor_child_m: Vec3::ZERO,
                relative_rotation: Quat::IDENTITY,
            },
        ));

        let dt = fixed_step();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        for _ in 0..120 {
            backend.sync_from_ecs(&mut world, physics_world).unwrap();
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
        }

        let welded_y = world.get::<Transform3>(cube).unwrap().translation.y;
        assert!(
            welded_y > 4.9,
            "welded cube should hang from the anchor, y={welded_y}"
        );

        // Release the weld and let it fall.
        world.entity_mut(cube).remove::<FixedJointDesc>();
        for _ in 0..120 {
            backend.sync_from_ecs(&mut world, physics_world).unwrap();
            backend.step(physics_world, dt).unwrap();
            backend.sync_to_ecs(&mut world, physics_world).unwrap();
        }

        let released_y = world.get::<Transform3>(cube).unwrap().translation.y;
        assert!(
            released_y < welded_y - 0.5,
            "released cube should fall once the weld is removed, y={released_y}"
        );
    }

    /// Runs a 1 kg mass suspended from a fixed anchor by a vertical prismatic
    /// joint whose motor commands an upward velocity, and returns the mass's
    /// final height. Higher `gain` lets the motor track that target more
    /// stiffly against gravity, up to the backend force cap.
    fn lift_displacement(gain: f64, multibody: bool) -> f64 {
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");

        let mut world = World::new();
        let anchor = spawn_named(&mut world, "anchor");
        world.entity_mut(anchor).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::splat(0.05)),
            Transform3::from_translation_rotation(Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY),
        ));

        let mass = spawn_named(&mut world, "mass");
        world.entity_mut(mass).insert((
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::splat(0.1)),
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.5, 0.0), Quat::IDENTITY),
            PrismaticJointDesc {
                parent: anchor,
                axis: Vec3::new(0.0, 1.0, 0.0),
                anchor_parent_m: Vec3::new(0.0, -2.5, 0.0),
                anchor_child_m: Vec3::ZERO,
            },
            JointMotor {
                velocity_rad_s: 1.0,
                gain,
                ..JointMotor::default()
            },
        ));
        if multibody {
            world.entity_mut(anchor).insert(MultibodyLink);
            world.entity_mut(mass).insert(MultibodyLink);
        }

        let dt = fixed_step();
        for _ in 0..120 {
            step_physics(&mut backend, &mut world, physics_world, dt).unwrap();
        }

        world.get::<Transform3>(mass).unwrap().translation.y - 0.5
    }

    #[test]
    fn motor_gain_lifts_mass_against_gravity() {
        // A unit gain produces a force too weak to hold ~9.81 N of weight, so the
        // mass sags below where a strong gain (force-capped above gravity) lifts it.
        let weak = lift_displacement(1.0, false);
        let strong = lift_displacement(40.0, false);

        assert!(
            strong > weak + 0.2,
            "higher gain should lift the mass higher: weak={weak}, strong={strong}"
        );
        assert!(
            strong > 0.5,
            "high-gain motor should raise the mass against gravity, displacement={strong}"
        );
    }

    #[test]
    fn multibody_motor_lifts_mass_against_gravity() {
        let displacement = lift_displacement(40.0, true);
        assert!(
            displacement > 0.5,
            "multibody motor should lift its child, displacement={displacement}"
        );
    }
}
