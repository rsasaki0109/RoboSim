//! Articulated tree model derived from the robot link/joint graph.

use crate::spatial::SpatialVec;
use crate::SpatialInertia;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{RigidBody, RigidBodyInertia};
use rne_robot::components::Inertial;
use rne_robot::{Joint, JointKind, KinematicModel, KinematicsError};
use std::collections::HashMap;
use thiserror::Error;

/// Error returned while building or evaluating an articulated model.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum DynamicsError {
    /// The underlying kinematic model could not be derived.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
    /// The supplied configuration or velocity vector has the wrong length.
    #[error("expected {expected} values but received {provided}")]
    DimensionMismatch {
        /// Number of values supplied.
        provided: usize,
        /// Number of values the model requires.
        expected: usize,
    },
    /// A configuration, velocity, acceleration, or torque vector was not finite.
    #[error("dynamics input contains a non-finite value")]
    NonFiniteInput,
    /// Gravity was not a finite vector.
    #[error("gravity must be a finite vector")]
    InvalidGravity,
    /// The mass matrix could not be inverted because it is singular.
    #[error("mass matrix is singular and cannot be inverted")]
    SingularMassMatrix,
    /// An internal invariant was violated: a degree of freedom had no owner.
    #[error("internal error: degree of freedom {0} has no owner link")]
    MissingDofOwner(usize),
}

#[derive(Clone, Debug)]
pub(crate) struct ArticulatedLink {
    pub(crate) entity: Entity,
    pub(crate) parent: Option<usize>,
    pub(crate) inertia: SpatialInertia,
    pub(crate) joint: Option<ArticulatedJoint>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ArticulatedJoint {
    pub(crate) dof: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct DofSpec {
    pub(crate) link: usize,
    pub(crate) s: SpatialVec,
}

/// A floating- or fixed-base articulated tree with link spatial inertias.
///
/// The model is a plain value derived from the scene. It owns a
/// [`KinematicModel`] for link ordering and forward kinematics and adds the
/// inertial and joint-motion data needed by the dynamics algorithms. It never
/// depends on a physics backend, a renderer, or wall-clock time.
#[derive(Clone, Debug)]
pub struct ArticulatedModel {
    pub(crate) robot: Entity,
    pub(crate) gravity_m_s2: Vec3,
    pub(crate) base_dof: usize,
    pub(crate) nv: usize,
    pub(crate) links: Vec<ArticulatedLink>,
    pub(crate) dofs: Vec<DofSpec>,
    pub(crate) link_dofs: Vec<Vec<usize>>,
    pub(crate) kinematic: KinematicModel,
}

impl ArticulatedModel {
    /// Builds an articulated model with the RNE default gravity `(0, -9.81, 0)`
    /// in a Y-up world.
    pub fn from_robot(world: &World, robot: Entity) -> Result<Self, DynamicsError> {
        Self::from_robot_with_gravity(world, robot, Vec3::new(0.0, -9.81, 0.0))
    }

    /// Builds an articulated model with an explicit gravity vector.
    pub(crate) fn from_robot_with_gravity(
        world: &World,
        robot: Entity,
        gravity_m_s2: Vec3,
    ) -> Result<Self, DynamicsError> {
        if !gravity_m_s2.is_finite() {
            return Err(DynamicsError::InvalidGravity);
        }
        let kinematic = KinematicModel::from_robot(world, robot)?;
        let base_dof = kinematic.base_dof();
        let nv = kinematic.dof();
        let link_count = kinematic.link_count();

        let mut joint_by_child: HashMap<Entity, (Entity, JointKind, Vec3)> = HashMap::new();
        for entity_ref in world.iter_entities() {
            let Some(joint) = entity_ref.get::<Joint>() else {
                continue;
            };
            if joint.robot == robot {
                joint_by_child.insert(joint.child_link, (entity_ref.id(), joint.kind, joint.axis));
            }
        }

        let mut dofs: Vec<Option<DofSpec>> = vec![None; nv];
        let mut link_dofs: Vec<Vec<usize>> = vec![Vec::new(); link_count];
        for (dof, slot) in dofs.iter_mut().enumerate().take(base_dof) {
            let mut s = [0.0; 6];
            s[dof] = 1.0;
            *slot = Some(DofSpec { link: 0, s });
            link_dofs[0].push(dof);
        }

        let mut links: Vec<ArticulatedLink> = Vec::with_capacity(link_count);
        for (index, link_dof) in link_dofs.iter_mut().enumerate() {
            let entity = kinematic
                .link_entity(index)
                .ok_or(DynamicsError::MissingDofOwner(index))?;
            let parent = kinematic.link_parent(index);
            let inertia = link_spatial_inertia(world, entity);
            let joint = joint_by_child
                .get(&entity)
                .map(|(joint_entity, kind, axis)| {
                    let dof = kinematic
                        .dof_index_of_joint(*joint_entity)
                        .map(|joint_dof| base_dof + joint_dof);
                    if let Some(global_dof) = dof {
                        if let Some(s) = joint_motion_subspace(*kind, *axis) {
                            dofs[global_dof] = Some(DofSpec { link: index, s });
                            link_dof.push(global_dof);
                        }
                    }
                    ArticulatedJoint { dof }
                });
            links.push(ArticulatedLink {
                entity,
                parent,
                inertia,
                joint,
            });
        }

        let mut resolved = Vec::with_capacity(nv);
        for (index, slot) in dofs.into_iter().enumerate() {
            resolved.push(slot.ok_or(DynamicsError::MissingDofOwner(index))?);
        }

        Ok(Self {
            robot,
            gravity_m_s2,
            base_dof,
            nv,
            links,
            dofs: resolved,
            link_dofs,
            kinematic,
        })
    }

    /// Owning robot entity.
    pub fn robot(&self) -> Entity {
        self.robot
    }

    /// Gravity vector in the world frame, in m/s².
    pub fn gravity_m_s2(&self) -> Vec3 {
        self.gravity_m_s2
    }

    /// Number of floating-base degrees of freedom (0 or 6).
    pub fn base_dof(&self) -> usize {
        self.base_dof
    }

    /// Total number of generalized velocity coordinates.
    pub fn nv(&self) -> usize {
        self.nv
    }

    /// Number of links in topological order.
    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    /// Entity of the link at topological index `index`.
    pub fn link_entity(&self, index: usize) -> Option<Entity> {
        self.links.get(index).map(|link| link.entity)
    }

    /// Topological index of a link's parent, if any.
    pub fn link_parent(&self, index: usize) -> Option<usize> {
        self.links.get(index).and_then(|link| link.parent)
    }

    /// Spatial inertia of the link at topological index `index`.
    pub fn link_inertia(&self, index: usize) -> Option<&SpatialInertia> {
        self.links.get(index).map(|link| &link.inertia)
    }

    /// Underlying kinematic model used for link ordering and forward kinematics.
    pub fn kinematic(&self) -> &KinematicModel {
        &self.kinematic
    }
}

fn link_spatial_inertia(world: &World, entity: Entity) -> SpatialInertia {
    let mass_kg = world
        .get::<RigidBody>(entity)
        .map(|body| body.mass_kg)
        .unwrap_or(0.0);
    if let Some(inertia) = world.get::<RigidBodyInertia>(entity) {
        return SpatialInertia::new(
            mass_kg,
            inertia.center_of_mass_local_m,
            [
                [inertia.ixx_kg_m2, inertia.ixy_kg_m2, inertia.ixz_kg_m2],
                [inertia.ixy_kg_m2, inertia.iyy_kg_m2, inertia.iyz_kg_m2],
                [inertia.ixz_kg_m2, inertia.iyz_kg_m2, inertia.izz_kg_m2],
            ],
        );
    }
    let center_of_mass_m = world
        .get::<Inertial>(entity)
        .map(|inertial| inertial.center_of_mass_m)
        .unwrap_or(Vec3::ZERO);
    SpatialInertia::point_mass(mass_kg, center_of_mass_m)
}

fn joint_motion_subspace(kind: JointKind, axis: Vec3) -> Option<SpatialVec> {
    let normalized = axis.normalize_or_zero();
    let axis = if normalized.length_squared() <= f64::EPSILON {
        Vec3::Y
    } else {
        normalized
    };
    match kind {
        JointKind::Fixed => None,
        JointKind::Revolute | JointKind::Continuous => {
            Some([0.0, 0.0, 0.0, axis.x, axis.y, axis.z])
        }
        JointKind::Prismatic => Some([axis.x, axis.y, axis.z, 0.0, 0.0, 0.0]),
    }
}
