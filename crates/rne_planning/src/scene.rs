//! Planning scene: robot model, collision state, and kinematics solvers.

use crate::error::PlanningError;
use crate::group::PlanningGroup;
use bevy_ecs::prelude::World;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_robot::{
    ColliderShape, CollisionPrimitive, CollisionWorld, KinematicModel, KinematicsSolverRegistry,
    PathCollisionConfig, SelfCollisionChecker, Transform3, VoxelGridObject,
};

/// The environment a planner reasons about.
///
/// `PlanningScene` is the `MoveIt` `PlanningScene` analogue: it owns the robot's
/// kinematic model, its self-collision checker, a world of static collision
/// objects, and the kinematics solvers used to resolve pose goals. It is
/// backend-neutral and never requires a physics engine.
#[derive(Debug)]
pub struct PlanningScene {
    self_checker: SelfCollisionChecker,
    collision_world: CollisionWorld,
    solvers: KinematicsSolverRegistry,
    groups: Vec<PlanningGroup>,
    workspace_bounds: Option<(Vec3, Vec3)>,
}

impl PlanningScene {
    /// Builds a scene from a robot in the ECS world.
    pub fn from_world(world: &World, robot: Entity) -> Result<Self, PlanningError> {
        let self_checker = SelfCollisionChecker::from_robot(world, robot)?;
        Ok(Self {
            self_checker,
            collision_world: CollisionWorld::new(),
            solvers: KinematicsSolverRegistry::with_builtins(),
            groups: Vec::new(),
            workspace_bounds: None,
        })
    }

    /// Sets the workspace box used by the workspace-bounds adapters.
    pub fn set_workspace_bounds(&mut self, min: Vec3, max: Vec3) {
        self.workspace_bounds = Some((min, max));
    }

    /// Sets the workspace box, consuming and returning the scene.
    pub fn with_workspace_bounds(mut self, min: Vec3, max: Vec3) -> Self {
        self.set_workspace_bounds(min, max);
        self
    }

    /// Workspace box as `(min, max)`, if one is set.
    pub fn workspace_bounds(&self) -> Option<(Vec3, Vec3)> {
        self.workspace_bounds
    }

    /// Registers a planning group, replacing any group with the same name.
    pub fn add_group(&mut self, group: PlanningGroup) {
        self.groups
            .retain(|existing| existing.name() != group.name());
        self.groups.push(group);
    }

    /// Registers a planning group, consuming and returning the scene.
    pub fn with_group(mut self, group: PlanningGroup) -> Self {
        self.add_group(group);
        self
    }

    /// Planning groups in registration order.
    pub fn groups(&self) -> &[PlanningGroup] {
        &self.groups
    }

    /// Looks up a planning group by name.
    pub fn group(&self, name: &str) -> Option<&PlanningGroup> {
        self.groups.iter().find(|group| group.name() == name)
    }

    /// Replaces the collision world, consuming and returning the scene.
    pub fn with_collision_world(mut self, collision_world: CollisionWorld) -> Self {
        self.collision_world = collision_world;
        self
    }

    /// The robot's kinematic model.
    pub fn model(&self) -> &KinematicModel {
        self.self_checker.model()
    }

    /// The robot's self-collision checker.
    pub fn self_checker(&self) -> &SelfCollisionChecker {
        &self.self_checker
    }

    /// Attaches a grasped or mounted collision body to a link.
    ///
    /// This is the `MoveIt` `AttachedBody` analogue; the body is checked against
    /// other links (except `touch_links`) and world objects.
    pub fn attach_body(
        &mut self,
        name: impl Into<String>,
        link: Entity,
        shape: ColliderShape,
        local_offset: Transform3,
        touch_links: Vec<Entity>,
    ) -> Result<(), PlanningError> {
        self.self_checker
            .attach_body(name, link, shape, local_offset, touch_links)?;
        Ok(())
    }

    /// Removes an attached body by name, returning whether it existed.
    pub fn detach_body(&mut self, name: &str) -> bool {
        self.self_checker.detach_body(name)
    }

    /// Attached bodies in insertion order.
    pub fn attached_bodies(&self) -> &[rne_robot::AttachedBody] {
        self.self_checker.attached_bodies()
    }

    /// Static world collision objects.
    pub fn collision_world(&self) -> &CollisionWorld {
        &self.collision_world
    }

    /// Adds or replaces a named world collision object.
    pub fn add_collision_object(
        &mut self,
        name: impl Into<String>,
        primitive: CollisionPrimitive,
    ) -> usize {
        self.collision_world.add_named_object(name, primitive)
    }

    /// Removes a named world collision object, returning whether it existed.
    pub fn remove_collision_object(&mut self, name: &str) -> bool {
        self.collision_world.remove_object(name)
    }

    /// Adds or replaces a named triangle-mesh collision object.
    pub fn add_mesh_collision_object(
        &mut self,
        name: impl Into<String>,
        vertices: Vec<Vec3>,
        triangles: Vec<[usize; 3]>,
    ) -> Result<usize, PlanningError> {
        Ok(self
            .collision_world
            .add_mesh_object(name, vertices, triangles)?)
    }

    /// Removes a named mesh collision object, returning whether it existed.
    pub fn remove_mesh_collision_object(&mut self, name: &str) -> bool {
        self.collision_world.remove_mesh_object(name)
    }

    /// Builds and adds a voxel occupancy map covering `points`.
    pub fn add_occupancy_map(
        &mut self,
        name: impl Into<String>,
        resolution_m: f64,
        padding_m: f64,
        points: &[Vec3],
    ) -> Result<usize, PlanningError> {
        let grid = VoxelGridObject::from_points(name, resolution_m, padding_m, points)?;
        Ok(self.collision_world.add_voxel_grid(grid))
    }

    /// Removes a named voxel occupancy map, returning whether it existed.
    pub fn remove_occupancy_map(&mut self, name: &str) -> bool {
        self.collision_world.remove_voxel_grid(name)
    }

    /// Registered kinematics solvers.
    pub fn solvers(&self) -> &KinematicsSolverRegistry {
        &self.solvers
    }

    /// Whether the line of sight from `from` to `to` is clear of world and
    /// robot geometry.
    ///
    /// Links in `excluded_links` (for example the sensor link) are ignored.
    pub fn line_of_sight_clear(
        &self,
        q: &[f64],
        from: Vec3,
        to: Vec3,
        excluded_links: &[Entity],
    ) -> Result<bool, PlanningError> {
        if self.collision_world.segment_blocked(from, to) {
            return Ok(false);
        }
        Ok(!self
            .self_checker
            .segment_blocked(q, from, to, excluded_links)?)
    }

    /// Whether a configuration is free of self and world collision.
    pub fn is_state_valid(&self, q: &[f64]) -> Result<bool, PlanningError> {
        if self.self_checker.check(q)?.is_colliding() {
            return Ok(false);
        }
        if self
            .collision_world
            .check(&self.self_checker, q)?
            .is_colliding()
        {
            return Ok(false);
        }
        Ok(true)
    }

    /// Whether the interpolated motion between two configurations is valid.
    ///
    /// Sampling is deterministic and shared with the planners.
    pub fn is_motion_valid(
        &self,
        start: &[f64],
        goal: &[f64],
        steps: usize,
    ) -> Result<bool, PlanningError> {
        self.is_motion_valid_with(start, goal, steps, None)
    }

    /// Whether a motion is collision free and satisfies an optional path
    /// constraint at every sampled configuration.
    pub fn is_motion_valid_with(
        &self,
        start: &[f64],
        goal: &[f64],
        steps: usize,
        path_constraint: Option<&crate::constraints::PathConstraint>,
    ) -> Result<bool, PlanningError> {
        let steps = steps.max(1);
        let path = self
            .self_checker
            .check_path(start, goal, &PathCollisionConfig::new(steps))?;
        if !path.is_valid() {
            return Ok(false);
        }

        let dof = self.self_checker.model().dof();
        if start.len() != dof || goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: start.len().min(goal.len()),
                expected: dof,
            });
        }
        let mut q = vec![0.0; dof];
        for step in 0..=steps {
            let interpolation = step as f64 / steps as f64;
            for (index, value) in q.iter_mut().enumerate() {
                *value = start[index] + (goal[index] - start[index]) * interpolation;
            }
            if self
                .collision_world
                .check(&self.self_checker, &q)?
                .is_colliding()
            {
                return Ok(false);
            }
            if let Some(constraint) = path_constraint {
                if !constraint.is_satisfied(self, &q)? {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
}
