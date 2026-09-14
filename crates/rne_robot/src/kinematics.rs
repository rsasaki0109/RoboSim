//! Generic forward and inverse kinematics for articulated robots.
//!
//! This module derives a kinematic model directly from the [`crate::Link`] and
//! [`crate::Joint`] ECS graph so that any robot — not just a hand-written body —
//! exposes forward kinematics, a geometric Jacobian, and a damped least-squares
//! inverse kinematics solver. The design follows the body-model separation used
//! by Choreonoid: the model is a plain value derived from the scene and can be
//! evaluated without a physics backend, a renderer, or wall-clock time.
//!
//! # Conventions
//!
//! A joint connects a parent link to a child link. The child link's local
//! [`Transform3`] is the *joint origin*, i.e. the child pose relative to the
//! parent at zero displacement. Joint displacement is applied after the origin:
//!
//! * revolute / continuous: `child = parent * origin * R(axis, q)`
//! * prismatic: `child = parent * origin * T(axis * q)`
//! * fixed: `child = parent * origin`
//!
//! `axis` is expressed in the joint frame (the child frame at zero
//! displacement), which is the URDF convention produced by `rne_urdf_import`.

use crate::components::{
    FloatingBase, Joint, JointKind, JointLimits, Link, MimicJoint, PassiveJoint, Robot,
};
use bevy_ecs::prelude::World;
use rne_ecs::{Entity, Name};
use rne_math::{Pose3, Quat, Vec3};
use rne_world::Transform3;
use std::collections::HashMap;
use thiserror::Error;

/// Error returned while building or evaluating a kinematic model.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum KinematicsError {
    /// The requested entity does not have a [`Robot`] component.
    #[error("entity {0:?} is not a robot")]
    MissingRobot(Entity),
    /// The robot has no link entities.
    #[error("robot {0:?} has no links")]
    NoLinks(Entity),
    /// The robot base link is not among the robot's links.
    #[error("robot base link {0:?} is not part of the robot")]
    MissingBaseLink(Entity),
    /// The base link is the child of another link, which implies a cycle.
    #[error("robot base link {0:?} is not a root link")]
    BaseLinkNotRoot(Entity),
    /// Two joints declare the same child link.
    #[error("link {0:?} is the child of more than one joint")]
    MultipleParent(Entity),
    /// A link cannot be reached from the base link.
    #[error("link {0:?} is disconnected from the robot base")]
    DisconnectedLink(Entity),
    /// The joint count does not match the model degrees of freedom.
    #[error("expected {expected} joint values but received {provided}")]
    JointCountMismatch {
        /// Number of values supplied.
        provided: usize,
        /// Number of movable joints in the model.
        expected: usize,
    },
    /// A target link entity is not part of the model.
    #[error("link {0:?} is not part of the kinematic model")]
    UnknownLink(Entity),
    /// The end-effector link is not descended from any movable joint.
    #[error("link {0:?} is not driven by any movable joint")]
    NoMovableChain(Entity),
    /// Inverse kinematics did not converge within the iteration budget.
    #[error("inverse kinematics did not converge after {iterations} iterations")]
    NotConverged {
        /// Number of iterations performed.
        iterations: usize,
    },
    /// An input value was not finite.
    #[error("kinematic input contains a non-finite value")]
    NonFiniteInput,
    /// A movable joint entity was missing its [`Joint`] component.
    #[error("joint entity {0:?} is missing its Joint component")]
    MissingJoint(Entity),
    /// A joint name is not part of the model degrees of freedom.
    #[error("joint {0:?} is not a movable joint of the model")]
    UnknownJoint(String),
    /// A kinematics solver name was empty.
    #[error("kinematics solver name must not be empty")]
    InvalidSolverName,
    /// Two registered kinematics solvers declared the same name.
    #[error("kinematics solver {0:?} is already registered")]
    DuplicateSolver(String),
    /// A mimic joint references a source that is not an independent joint.
    #[error("mimic joint {joint:?} references invalid source {source_joint:?}")]
    InvalidMimicJoint {
        /// The mimic joint entity.
        joint: Entity,
        /// The referenced source joint entity.
        source_joint: Entity,
    },
    /// Inverse kinematics options disabled both position and orientation.
    #[error("inverse kinematics must solve position, orientation, or both")]
    EmptyIkObjective,
    /// `base` is not an ancestor of `tip` in the link tree.
    #[error("link {tip:?} is not descended from {base:?}")]
    ChainNotFound {
        /// Requested chain base link.
        base: Entity,
        /// Requested chain tip link.
        tip: Entity,
    },
}

#[derive(Clone, Debug)]
struct ModelLink {
    entity: Entity,
    name: String,
    parent: Option<usize>,
    joint: Option<usize>,
    local: Transform3,
}

#[derive(Clone, Copy, Debug)]
struct MimicSpec {
    source_dof: usize,
    multiplier: f64,
    offset: f64,
}

#[derive(Clone, Debug)]
struct ModelJoint {
    entity: Entity,
    name: String,
    kind: JointKind,
    axis: Vec3,
    limits: JointLimits,
    parent_link: usize,
    child_link: usize,
    dof: Option<usize>,
    mimic: Option<MimicSpec>,
}

/// A kinematic model derived from a robot's link and joint graph.
///
/// Links are stored in topological order (parents before children) so forward
/// kinematics is a single pass. Movable joints define the degrees of freedom in
/// a deterministic order matching [`Self::movable_joint_entities`].
#[derive(Clone, Debug)]
pub struct KinematicModel {
    robot: Entity,
    base_link: usize,
    links: Vec<ModelLink>,
    joints: Vec<ModelJoint>,
    dof: Vec<usize>,
    passive_joints: Vec<usize>,
    base_dof: usize,
    entity_to_link: HashMap<Entity, usize>,
}

/// Translation sampling range for a floating base, in meters.
pub const FLOATING_BASE_TRANSLATION_LIMIT_M: f64 = 10.0;

const FLOATING_BASE_DOF_NAMES: [&str; 6] = [
    "base_x",
    "base_y",
    "base_z",
    "base_roll",
    "base_pitch",
    "base_yaw",
];

impl KinematicModel {
    /// Builds a model for the given robot entity.
    pub fn from_robot(world: &World, robot: Entity) -> Result<Self, KinematicsError> {
        let robot_component = world
            .get::<Robot>(robot)
            .ok_or(KinematicsError::MissingRobot(robot))?;
        let base_entity = robot_component.base_link;
        let base_dof = if world.get::<FloatingBase>(base_entity).is_some() {
            6
        } else {
            0
        };

        let mut link_records: Vec<(Entity, String, Transform3)> = Vec::new();
        for entity_ref in world.iter_entities() {
            let Some(link) = entity_ref.get::<Link>() else {
                continue;
            };
            if link.robot != robot {
                continue;
            }
            let transform = entity_ref
                .get::<Transform3>()
                .copied()
                .unwrap_or(Transform3::IDENTITY);
            link_records.push((entity_ref.id(), link.name.clone(), transform));
        }
        if link_records.is_empty() {
            return Err(KinematicsError::NoLinks(robot));
        }
        link_records.sort_by(|a, b| a.1.cmp(&b.1).then_with(|| a.0.index().cmp(&b.0.index())));

        let mut old_index: HashMap<Entity, usize> = HashMap::new();
        for (index, (entity, _, _)) in link_records.iter().enumerate() {
            old_index.insert(*entity, index);
        }
        let base_old = *old_index
            .get(&base_entity)
            .ok_or(KinematicsError::MissingBaseLink(base_entity))?;

        struct RawMimic {
            source: Entity,
            multiplier: f64,
            offset: f64,
        }

        struct RawJoint {
            entity: Entity,
            name: String,
            parent: usize,
            child: usize,
            kind: JointKind,
            axis: Vec3,
            limits: JointLimits,
            mimic: Option<RawMimic>,
            passive: bool,
        }

        let mut raw_joints: Vec<RawJoint> = Vec::new();
        let mut old_parent: Vec<Option<usize>> = vec![None; link_records.len()];
        let mut old_joint_index: Vec<Option<usize>> = vec![None; link_records.len()];
        for entity_ref in world.iter_entities() {
            let Some(joint) = entity_ref.get::<Joint>() else {
                continue;
            };
            if joint.robot != robot {
                continue;
            }
            let parent = *old_index
                .get(&joint.parent_link)
                .ok_or(KinematicsError::MissingBaseLink(joint.parent_link))?;
            let child = *old_index
                .get(&joint.child_link)
                .ok_or(KinematicsError::MissingBaseLink(joint.child_link))?;
            if old_parent[child].is_some() {
                return Err(KinematicsError::MultipleParent(joint.child_link));
            }
            old_parent[child] = Some(parent);
            let name = entity_ref
                .get::<Name>()
                .map(|name| name.0.clone())
                .unwrap_or_else(|| format!("joint_{}", entity_ref.id().index()));
            let mimic = entity_ref.get::<MimicJoint>().map(|mimic| RawMimic {
                source: mimic.source,
                multiplier: mimic.multiplier,
                offset: mimic.offset,
            });
            let passive = entity_ref.get::<PassiveJoint>().is_some();
            old_joint_index[child] = Some(raw_joints.len());
            raw_joints.push(RawJoint {
                entity: entity_ref.id(),
                name,
                parent,
                child,
                kind: joint.kind,
                axis: joint.axis,
                limits: joint.limits,
                mimic,
                passive,
            });
        }

        if old_parent[base_old].is_some() {
            return Err(KinematicsError::BaseLinkNotRoot(base_entity));
        }

        let mut children: Vec<Vec<usize>> = vec![Vec::new(); link_records.len()];
        for joint in &raw_joints {
            children[joint.parent].push(joint.child);
        }
        for list in &mut children {
            list.sort_unstable();
        }

        let mut order: Vec<usize> = Vec::with_capacity(link_records.len());
        let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        queue.push_back(base_old);
        let mut visited = vec![false; link_records.len()];
        visited[base_old] = true;
        while let Some(node) = queue.pop_front() {
            order.push(node);
            for &child in &children[node] {
                if !visited[child] {
                    visited[child] = true;
                    queue.push_back(child);
                }
            }
        }
        if order.len() != link_records.len() {
            let disconnected = (0..link_records.len())
                .find(|&index| !visited[index])
                .map(|index| link_records[index].0)
                .unwrap_or(base_entity);
            return Err(KinematicsError::DisconnectedLink(disconnected));
        }

        let mut new_index = vec![0usize; link_records.len()];
        for (position, &old) in order.iter().enumerate() {
            new_index[old] = position;
        }

        let mut links: Vec<ModelLink> = Vec::with_capacity(order.len());
        for &old in &order {
            let (entity, name, local) = &link_records[old];
            links.push(ModelLink {
                entity: *entity,
                name: name.clone(),
                parent: old_parent[old].map(|parent| new_index[parent]),
                joint: None,
                local: *local,
            });
        }

        raw_joints.sort_by_key(|joint| new_index[joint.child]);
        let mut joints: Vec<ModelJoint> = Vec::with_capacity(raw_joints.len());
        let mut dof: Vec<usize> = Vec::new();
        let mut passive_joints: Vec<usize> = Vec::new();
        let mut mimic_sources: Vec<(usize, RawMimic)> = Vec::new();
        for joint in raw_joints {
            let child_link = new_index[joint.child];
            let joint_index = joints.len();
            let independent = joint.kind != JointKind::Fixed && joint.mimic.is_none();
            let dof_index = if independent {
                let index = dof.len();
                dof.push(joint_index);
                Some(index)
            } else {
                None
            };
            if joint.passive {
                passive_joints.push(joint_index);
            }
            if let Some(mimic) = joint.mimic {
                mimic_sources.push((joint_index, mimic));
            }
            links[child_link].joint = Some(joint_index);
            joints.push(ModelJoint {
                entity: joint.entity,
                name: joint.name,
                kind: joint.kind,
                axis: joint.axis,
                limits: joint.limits,
                parent_link: new_index[joint.parent],
                child_link,
                dof: dof_index,
                mimic: None,
            });
        }

        if !mimic_sources.is_empty() {
            let entity_to_joint: HashMap<Entity, usize> = joints
                .iter()
                .enumerate()
                .map(|(index, joint)| (joint.entity, index))
                .collect();
            for (joint_index, mimic) in &mimic_sources {
                let source_joint = entity_to_joint.get(&mimic.source).copied().ok_or(
                    KinematicsError::InvalidMimicJoint {
                        joint: joints[*joint_index].entity,
                        source_joint: mimic.source,
                    },
                )?;
                let Some(source_dof) = joints[source_joint].dof else {
                    return Err(KinematicsError::InvalidMimicJoint {
                        joint: joints[*joint_index].entity,
                        source_joint: mimic.source,
                    });
                };
                if !mimic.multiplier.is_finite() || !mimic.offset.is_finite() {
                    return Err(KinematicsError::NonFiniteInput);
                }
                joints[*joint_index].mimic = Some(MimicSpec {
                    source_dof,
                    multiplier: mimic.multiplier,
                    offset: mimic.offset,
                });
            }
        }

        let mut entity_to_link = HashMap::new();
        for (index, link) in links.iter().enumerate() {
            entity_to_link.insert(link.entity, index);
        }

        Ok(Self {
            robot,
            base_link: new_index[base_old],
            links,
            joints,
            dof,
            passive_joints,
            base_dof,
            entity_to_link,
        })
    }

    /// Owning robot entity.
    pub fn robot(&self) -> Entity {
        self.robot
    }

    /// Base link entity.
    pub fn base_link(&self) -> Entity {
        self.links[self.base_link].entity
    }

    /// Number of links in the model.
    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    /// Number of movable degrees of freedom, including a floating base.
    pub fn dof(&self) -> usize {
        self.base_dof + self.dof.len()
    }

    /// Number of degrees of freedom contributed by the floating base (0 or 6).
    pub fn base_dof(&self) -> usize {
        self.base_dof
    }

    /// Movable degree-of-freedom names, base first, then joints.
    pub fn movable_dof_names(&self) -> Vec<String> {
        let mut names: Vec<String> = FLOATING_BASE_DOF_NAMES[..self.base_dof]
            .iter()
            .map(|name| (*name).to_string())
            .collect();
        names.extend(self.movable_joint_names().into_iter().map(str::to_string));
        names
    }

    /// Entity of the link at topological index `index`.
    pub fn link_entity(&self, index: usize) -> Option<Entity> {
        self.links.get(index).map(|link| link.entity)
    }

    /// Name of the link at topological index `index`.
    pub fn link_name(&self, index: usize) -> Option<&str> {
        self.links.get(index).map(|link| link.name.as_str())
    }

    /// Looks up the topological index of a link entity.
    pub fn link_index(&self, entity: Entity) -> Option<usize> {
        self.entity_to_link.get(&entity).copied()
    }

    /// Topological index of a link's parent, if it has one.
    pub fn link_parent(&self, index: usize) -> Option<usize> {
        self.links.get(index).and_then(|link| link.parent)
    }

    /// Joint entities from `base_link` to `tip_link`, in base-to-tip order.
    ///
    /// Both fixed and movable joints are included. Returns
    /// [`KinematicsError::ChainNotFound`] when `base_link` is not an ancestor of
    /// `tip_link`.
    pub fn chain_joints(
        &self,
        base_link: Entity,
        tip_link: Entity,
    ) -> Result<Vec<Entity>, KinematicsError> {
        let base = self
            .link_index(base_link)
            .ok_or(KinematicsError::UnknownLink(base_link))?;
        let mut index = self
            .link_index(tip_link)
            .ok_or(KinematicsError::UnknownLink(tip_link))?;
        let mut joints = Vec::new();
        while index != base {
            let Some(joint_index) = self.links[index].joint else {
                return Err(KinematicsError::ChainNotFound {
                    base: base_link,
                    tip: tip_link,
                });
            };
            joints.push(self.joints[joint_index].entity);
            index = self.joints[joint_index].parent_link;
        }
        joints.reverse();
        Ok(joints)
    }

    /// Degree-of-freedom index of a joint entity, if it is independent.
    pub fn dof_index_of_joint(&self, joint: Entity) -> Option<usize> {
        self.joints
            .iter()
            .find(|candidate| candidate.entity == joint)
            .and_then(|candidate| candidate.dof)
    }

    /// Link entity with the given name.
    pub fn link_entity_by_name(&self, name: &str) -> Option<Entity> {
        self.links
            .iter()
            .find(|link| link.name == name)
            .map(|link| link.entity)
    }

    /// Joint entity with the given name.
    pub fn joint_entity_by_name(&self, name: &str) -> Option<Entity> {
        self.joints
            .iter()
            .find(|joint| joint.name == name)
            .map(|joint| joint.entity)
    }

    /// Parent link entity of a joint.
    pub fn joint_parent_link(&self, joint: Entity) -> Option<Entity> {
        self.joints
            .iter()
            .find(|candidate| candidate.entity == joint)
            .and_then(|candidate| self.links.get(candidate.parent_link))
            .map(|link| link.entity)
    }

    /// Child link entity of a joint.
    pub fn joint_child_link(&self, joint: Entity) -> Option<Entity> {
        self.joints
            .iter()
            .find(|candidate| candidate.entity == joint)
            .and_then(|candidate| self.links.get(candidate.child_link))
            .map(|link| link.entity)
    }

    /// Movable joint entities in degree-of-freedom order.
    pub fn movable_joint_entities(&self) -> Vec<Entity> {
        self.dof
            .iter()
            .map(|&index| self.joints[index].entity)
            .collect()
    }

    /// Movable joint names in degree-of-freedom order.
    pub fn movable_joint_names(&self) -> Vec<&str> {
        self.dof
            .iter()
            .map(|&index| self.joints[index].name.as_str())
            .collect()
    }

    /// Joint limits in degree-of-freedom order, including a floating base.
    pub fn joint_limits(&self) -> Vec<JointLimits> {
        let mut limits = Vec::with_capacity(self.dof());
        for index in 0..self.base_dof {
            limits.push(base_joint_limit(index));
        }
        limits.extend(self.dof.iter().map(|&index| self.joints[index].limits));
        limits
    }

    /// Passive joint entities in deterministic joint order.
    pub fn passive_joint_entities(&self) -> Vec<Entity> {
        self.passive_joints
            .iter()
            .map(|&index| self.joints[index].entity)
            .collect()
    }

    /// Mimic joint entities in deterministic joint order.
    pub fn mimic_joint_entities(&self) -> Vec<Entity> {
        self.joints
            .iter()
            .filter(|joint| joint.mimic.is_some())
            .map(|joint| joint.entity)
            .collect()
    }

    /// Whether a joint entity is a mimic joint.
    pub fn is_mimic_joint(&self, joint: Entity) -> bool {
        self.joints
            .iter()
            .any(|candidate| candidate.entity == joint && candidate.mimic.is_some())
    }

    /// Computes forward kinematics for a joint vector, using the model's stored
    /// root transform.
    pub fn forward_kinematics(&self, q: &[f64]) -> Result<ForwardKinematics, KinematicsError> {
        self.forward_kinematics_with_base(q, None)
    }

    /// Computes forward kinematics with an optional override for the root pose.
    ///
    /// `base_pose` replaces the stored root link transform when provided, which
    /// is useful for evaluating a floating base without mutating the world.
    pub fn forward_kinematics_with_base(
        &self,
        q: &[f64],
        base_pose: Option<&Transform3>,
    ) -> Result<ForwardKinematics, KinematicsError> {
        if q.len() != self.dof() {
            return Err(KinematicsError::JointCountMismatch {
                provided: q.len(),
                expected: self.dof(),
            });
        }
        if q.iter().any(|value| !value.is_finite()) {
            return Err(KinematicsError::NonFiniteInput);
        }

        let floating_base = self.base_dof == 6;
        let mut transforms = vec![Transform3::IDENTITY; self.links.len()];
        for (index, link) in self.links.iter().enumerate() {
            match link.parent {
                Some(parent) => {
                    let joint_index = link
                        .joint
                        .expect("every non-root link is connected by a joint");
                    let joint = &self.joints[joint_index];
                    let displacement = match joint.dof {
                        Some(dof) => q[self.base_dof + dof],
                        None => match &joint.mimic {
                            Some(mimic) => {
                                mimic.multiplier * q[self.base_dof + mimic.source_dof]
                                    + mimic.offset
                            }
                            None => 0.0,
                        },
                    };
                    let motion = joint_motion(joint, displacement);
                    transforms[index] = transforms[parent]
                        .mul_transform(&link.local)
                        .mul_transform(&motion);
                }
                None => {
                    transforms[index] = match base_pose {
                        Some(pose) => *pose,
                        None if floating_base => {
                            floating_base_transform(q).mul_transform(&link.local)
                        }
                        None => link.local,
                    };
                }
            }
        }

        Ok(ForwardKinematics {
            links: self.links.iter().map(|link| link.entity).collect(),
            transforms,
        })
    }

    /// Computes the geometric Jacobian of `target_link` about `point_local`.
    ///
    /// The returned matrix has 6 rows (linear xyz then angular xyz) and one
    /// column per movable joint. Only joints on the chain from the base to the
    /// target contribute.
    pub fn jacobian(
        &self,
        q: &[f64],
        target_link: Entity,
        point_local: Vec3,
    ) -> Result<Jacobian, KinematicsError> {
        let state = self.forward_kinematics(q)?;
        let target = self
            .link_index(target_link)
            .ok_or(KinematicsError::UnknownLink(target_link))?;

        let mut matrix = Jacobian::zeros(6, self.dof());
        let point_world = transform_point(&state.transforms[target], point_local);

        if self.base_dof == 6 {
            let base_origin = Vec3::new(q[0], q[1], q[2]);
            let yaw = Quat::from_rotation_z(q[5]);
            let roll_axis = (yaw * Quat::from_rotation_y(q[4]) * Vec3::X).normalize_or_zero();
            let pitch_axis = (yaw * Vec3::Y).normalize_or_zero();
            let r = point_world - base_origin;
            matrix.add(0, 0, 1.0);
            matrix.add(1, 1, 1.0);
            matrix.add(2, 2, 1.0);
            for (base_dof, axis) in [(3usize, roll_axis), (4, pitch_axis), (5, Vec3::Z)] {
                let linear = axis.cross(r);
                matrix.add(0, base_dof, linear.x);
                matrix.add(1, base_dof, linear.y);
                matrix.add(2, base_dof, linear.z);
                matrix.add(3, base_dof, axis.x);
                matrix.add(4, base_dof, axis.y);
                matrix.add(5, base_dof, axis.z);
            }
        }

        let mut index = target;
        while let Some(joint_index) = self.links[index].joint {
            let joint = &self.joints[joint_index];
            let (joint_dof, coefficient) = if let Some(dof) = joint.dof {
                (dof, 1.0)
            } else if let Some(mimic) = &joint.mimic {
                (mimic.source_dof, mimic.multiplier)
            } else {
                index = joint.parent_link;
                continue;
            };
            let dof = self.base_dof + joint_dof;
            let zero_frame = state.transforms[joint.parent_link]
                .mul_transform(&self.links[joint.child_link].local);
            let axis = world_axis(&zero_frame, joint.axis);
            let origin = zero_frame.translation;
            match joint.kind {
                JointKind::Prismatic => {
                    matrix.add(0, dof, axis.x * coefficient);
                    matrix.add(1, dof, axis.y * coefficient);
                    matrix.add(2, dof, axis.z * coefficient);
                }
                _ => {
                    let r = point_world - origin;
                    let linear = axis.cross(r);
                    matrix.add(0, dof, linear.x * coefficient);
                    matrix.add(1, dof, linear.y * coefficient);
                    matrix.add(2, dof, linear.z * coefficient);
                    matrix.add(3, dof, axis.x * coefficient);
                    matrix.add(4, dof, axis.y * coefficient);
                    matrix.add(5, dof, axis.z * coefficient);
                }
            }
            index = joint.parent_link;
        }

        Ok(matrix)
    }

    /// Manipulability of the chain to `end_link` at `q`.
    ///
    /// This is the MoveIt kinematics-metric analogue `sqrt(det(J J^T))`. It is
    /// zero at a kinematic singularity and grows with the distance from one.
    pub fn manipulability(&self, q: &[f64], end_link: Entity) -> Result<f64, KinematicsError> {
        let jacobian = self.jacobian(q, end_link, Vec3::ZERO)?;
        let mut normal = [[0.0_f64; 6]; 6];
        for (row, normal_row) in normal.iter_mut().enumerate() {
            for (column, cell) in normal_row.iter_mut().enumerate() {
                *cell = (0..jacobian.cols())
                    .map(|index| jacobian.get(row, index) * jacobian.get(column, index))
                    .sum();
            }
        }
        Ok(determinant6(&mut normal).max(0.0).sqrt())
    }

    /// Minimum signed distance from `q` to the nearest finite joint limit.
    ///
    /// Returns [`f64::INFINITY`] when no joint has a finite limit. Continuous
    /// joints are ignored.
    pub fn joint_limit_distance(&self, q: &[f64]) -> f64 {
        let mut minimum = f64::INFINITY;
        for (dof, &value) in q.iter().enumerate() {
            if dof < self.base_dof {
                let limit = base_joint_limit(dof);
                if limit.lower.is_finite() {
                    minimum = minimum.min(value - limit.lower);
                }
                if limit.upper.is_finite() {
                    minimum = minimum.min(limit.upper - value);
                }
                continue;
            }
            let joint = &self.joints[self.dof[dof - self.base_dof]];
            if joint.kind == JointKind::Continuous {
                continue;
            }
            if joint.limits.lower.is_finite() {
                minimum = minimum.min(value - joint.limits.lower);
            }
            if joint.limits.upper.is_finite() {
                minimum = minimum.min(joint.limits.upper - value);
            }
        }
        minimum
    }

    /// Samples a configuration within the model's joint limits.
    ///
    /// Inactive joints (when `active` is set) keep their `base` value. Used by
    /// [`KinematicsSolver::search_position_ik`]; callers normally do not need it.
    pub(crate) fn random_configuration(
        &self,
        base: &[f64],
        active: Option<&[bool]>,
        rng: &mut SplitMix64,
    ) -> Vec<f64> {
        let mut configuration = base.to_vec();
        for (dof, value) in configuration.iter_mut().enumerate() {
            if active.is_some_and(|mask| !mask[dof]) {
                continue;
            }
            if dof < self.base_dof {
                *value = match dof {
                    0..=2 => (rng.next_f64() * 2.0 - 1.0) * FLOATING_BASE_TRANSLATION_LIMIT_M,
                    _ => -std::f64::consts::PI + 2.0 * std::f64::consts::PI * rng.next_f64(),
                };
                continue;
            }
            let joint = &self.joints[self.dof[dof - self.base_dof]];
            match joint.kind {
                JointKind::Fixed => *value = 0.0,
                JointKind::Continuous => {
                    *value = -std::f64::consts::PI + 2.0 * std::f64::consts::PI * rng.next_f64();
                }
                JointKind::Revolute | JointKind::Prismatic => {
                    let lower = if joint.limits.lower.is_finite() {
                        joint.limits.lower
                    } else {
                        -std::f64::consts::PI
                    };
                    let upper = if joint.limits.upper.is_finite() {
                        joint.limits.upper
                    } else {
                        std::f64::consts::PI
                    };
                    *value = if upper <= lower {
                        lower
                    } else {
                        lower + (upper - lower) * rng.next_f64()
                    };
                }
            }
        }
        configuration
    }

    /// Solves inverse kinematics for a full pose target.
    ///
    /// Uses damped least squares with joint-limit clamping. When
    /// [`IkOptions::solve_orientation`] is false, only the position rows are
    /// driven.
    pub fn inverse_kinematics(
        &self,
        target: &Pose3,
        end_link: Entity,
        initial: &[f64],
        options: &IkOptions,
    ) -> Result<IkSolution, KinematicsError> {
        self.inverse_kinematics_masked(target, end_link, initial, options, None)
    }

    /// Solves inverse kinematics while only moving active degrees of freedom.
    ///
    /// `active` must have one entry per degree of freedom. Inactive joints keep
    /// their initial value. This is the group-scoped solver used when planning a
    /// single chain; an all-true mask behaves like [`Self::inverse_kinematics`].
    pub fn inverse_kinematics_active(
        &self,
        target: &Pose3,
        end_link: Entity,
        initial: &[f64],
        options: &IkOptions,
        active: &[bool],
    ) -> Result<IkSolution, KinematicsError> {
        if active.len() != self.dof() {
            return Err(KinematicsError::JointCountMismatch {
                provided: active.len(),
                expected: self.dof(),
            });
        }
        self.inverse_kinematics_masked(target, end_link, initial, options, Some(active))
    }

    fn inverse_kinematics_masked(
        &self,
        target: &Pose3,
        end_link: Entity,
        initial: &[f64],
        options: &IkOptions,
        active: Option<&[bool]>,
    ) -> Result<IkSolution, KinematicsError> {
        if initial.len() != self.dof() {
            return Err(KinematicsError::JointCountMismatch {
                provided: initial.len(),
                expected: self.dof(),
            });
        }
        if !target.translation.is_finite() || !target.rotation.is_finite() {
            return Err(KinematicsError::NonFiniteInput);
        }
        let target_index = self
            .link_index(end_link)
            .ok_or(KinematicsError::UnknownLink(end_link))?;
        if !self.has_movable_ancestor(target_index) {
            return Err(KinematicsError::NoMovableChain(end_link));
        }
        let active_columns: Option<Vec<usize>> = active.map(|mask| {
            mask.iter()
                .enumerate()
                .filter_map(|(index, &enabled)| enabled.then_some(index))
                .collect()
        });
        if let Some(columns) = &active_columns {
            if columns.is_empty() {
                return Err(KinematicsError::NoMovableChain(end_link));
            }
        }

        let mut q = initial.to_vec();
        self.clamp_to_limits(&mut q, true);

        for iteration in 0..options.max_iterations {
            let state = self.forward_kinematics(&q)?;
            let current = &state.transforms[target_index];
            let position_error = target.translation - current.translation;
            let orientation_error = orientation_error(current.rotation, target.rotation);
            let position_length = position_error.length();
            let orientation_length = orientation_error.length();

            let position_ok =
                !options.solve_position || position_length <= options.position_tolerance_m;
            let orientation_ok = !options.solve_orientation
                || orientation_length <= options.orientation_tolerance_rad;
            if position_ok && orientation_ok {
                return Ok(IkSolution {
                    joint_positions: q,
                    iterations: iteration,
                    position_error_m: position_length,
                    orientation_error_rad: orientation_length,
                });
            }

            let rows = solver_rows(options);
            if rows.is_empty() {
                return Err(KinematicsError::EmptyIkObjective);
            }
            let matrix = self.jacobian(&q, end_link, Vec3::ZERO)?;
            let mut error = [0.0_f64; 6];
            error[0] = position_error.x;
            error[1] = position_error.y;
            error[2] = position_error.z;
            error[3] = orientation_error.x;
            error[4] = orientation_error.y;
            error[5] = orientation_error.z;
            let delta = match &active_columns {
                None => damped_least_squares(&matrix, &error, &rows, options)?,
                Some(columns) => {
                    let mut reduced = Jacobian::zeros(6, columns.len());
                    for (new_column, &old_column) in columns.iter().enumerate() {
                        for row in 0..6 {
                            reduced.add(row, new_column, matrix.get(row, old_column));
                        }
                    }
                    let reduced_delta = damped_least_squares(&reduced, &error, &rows, options)?;
                    let mut full = vec![0.0; self.dof()];
                    for (new_column, &old_column) in columns.iter().enumerate() {
                        full[old_column] = reduced_delta[new_column];
                    }
                    full
                }
            };
            for (value, step) in q.iter_mut().zip(delta.iter()) {
                *value += step;
            }
            self.clamp_to_limits(&mut q, false);
        }

        Err(KinematicsError::NotConverged {
            iterations: options.max_iterations,
        })
    }

    fn solve_analytic_two_link(
        &self,
        target: &Pose3,
        end_link: Entity,
        initial: &[f64],
        options: &IkOptions,
    ) -> Option<IkSolution> {
        if self.base_dof > 0 {
            return None;
        }
        let chain = self
            .chain_joints(self.links[self.base_link].entity, end_link)
            .ok()?;
        let movable: Vec<usize> = chain
            .iter()
            .filter_map(|&entity| self.joints.iter().position(|joint| joint.entity == entity))
            .filter(|&index| {
                self.joints[index].dof.is_some()
                    && matches!(
                        self.joints[index].kind,
                        JointKind::Revolute | JointKind::Continuous
                    )
            })
            .collect();
        if movable.len() != 2 {
            return None;
        }
        let zero = vec![0.0; self.dof()];
        let state0 = self.forward_kinematics(&zero).ok()?;
        let target_index = self.link_index(end_link)?;
        let end_zero = state0.transforms[target_index].translation;
        let first = &self.joints[movable[0]];
        let second = &self.joints[movable[1]];
        let frame1 =
            state0.transforms[first.parent_link].mul_transform(&self.links[first.child_link].local);
        let frame2 = state0.transforms[second.parent_link]
            .mul_transform(&self.links[second.child_link].local);
        let first_origin = frame1.translation;
        let second_origin = frame2.translation;
        let first_axis = (frame1.rotation * first.axis.normalize_or_zero()).normalize_or_zero();
        let second_axis = (frame2.rotation * second.axis.normalize_or_zero()).normalize_or_zero();
        if first_axis.cross(second_axis).length() > 1.0e-6 {
            return None;
        }
        let first_link = second_origin - first_origin;
        let first_length = first_link.length();
        let u = first_link.normalize_or_zero();
        if first_length <= 1.0e-9 || u.length_squared() < 0.5 {
            return None;
        }
        let v = first_axis.cross(u);
        let second_link = end_zero - second_origin;
        let second_length = second_link.length();
        if second_length <= 1.0e-9 {
            return None;
        }
        let beta = second_link.dot(v).atan2(second_link.dot(u));

        let delta = target.translation - first_origin;
        let px = delta.dot(u);
        let py = delta.dot(v);
        let radius = (px * px + py * py).sqrt();
        let cos_second =
            (radius * radius - first_length * first_length - second_length * second_length)
                / (2.0 * first_length * second_length);
        if !(-1.0..=1.0).contains(&cos_second) {
            return None;
        }
        let second_magnitude = cos_second.acos();

        for sign in [1.0_f64, -1.0] {
            let second_angle = sign * second_magnitude;
            let reference = beta + second_angle;
            let first_angle = py.atan2(px)
                - (second_length * reference.sin())
                    .atan2(first_length + second_length * reference.cos());
            let mut q = initial.to_vec();
            q[first.dof.expect("movable")] = first_angle;
            q[second.dof.expect("movable")] = second_angle;
            self.clamp_to_limits(&mut q, false);
            let Ok(state) = self.forward_kinematics(&q) else {
                continue;
            };
            let transform = &state.transforms[target_index];
            let position_error = (target.translation - transform.translation).length();
            let orientation_error = orientation_error(transform.rotation, target.rotation).length();
            let position_ok =
                !options.solve_position || position_error <= options.position_tolerance_m;
            let orientation_ok = !options.solve_orientation
                || orientation_error <= options.orientation_tolerance_rad;
            if position_ok && orientation_ok {
                return Some(IkSolution {
                    joint_positions: q,
                    iterations: 0,
                    position_error_m: position_error,
                    orientation_error_rad: orientation_error,
                });
            }
        }
        None
    }

    fn has_movable_ancestor(&self, mut index: usize) -> bool {
        if self.base_dof > 0 {
            return true;
        }
        while let Some(joint_index) = self.links[index].joint {
            let joint = &self.joints[joint_index];
            if joint.dof.is_some() || joint.mimic.is_some() {
                return true;
            }
            index = joint.parent_link;
        }
        false
    }

    fn clamp_to_limits(&self, q: &mut [f64], round: bool) {
        for (dof, value) in q.iter_mut().enumerate() {
            if dof < self.base_dof {
                let limit = base_joint_limit(dof);
                *value = value.clamp(limit.lower, limit.upper);
                if round && !value.is_finite() {
                    *value = 0.0;
                }
                continue;
            }
            let joint = &self.joints[self.dof[dof - self.base_dof]];
            match joint.kind {
                JointKind::Fixed => *value = 0.0,
                JointKind::Continuous => {}
                JointKind::Revolute | JointKind::Prismatic => {
                    let lower = if joint.limits.lower.is_finite() {
                        joint.limits.lower
                    } else {
                        f64::NEG_INFINITY
                    };
                    let upper = if joint.limits.upper.is_finite() {
                        joint.limits.upper
                    } else {
                        f64::INFINITY
                    };
                    *value = value.clamp(lower, upper);
                }
            }
            if round && !value.is_finite() {
                *value = 0.0;
            }
        }
    }
}

/// Forward kinematics result aligned with the model's topological link order.
#[derive(Clone, Debug, PartialEq)]
pub struct ForwardKinematics {
    links: Vec<Entity>,
    transforms: Vec<Transform3>,
}

impl ForwardKinematics {
    /// World transform for a link entity.
    pub fn link_transform(&self, entity: Entity) -> Option<&Transform3> {
        self.links
            .iter()
            .position(|candidate| *candidate == entity)
            .map(|index| &self.transforms[index])
    }

    /// World transform at a topological link index.
    pub fn transform_at(&self, index: usize) -> Option<&Transform3> {
        self.transforms.get(index)
    }

    /// Link entities in the order of [`Self::transforms`].
    pub fn links(&self) -> &[Entity] {
        &self.links
    }

    /// All link world transforms.
    pub fn transforms(&self) -> &[Transform3] {
        &self.transforms
    }
}

/// A dense matrix produced by [`KinematicModel::jacobian`].
#[derive(Clone, Debug, PartialEq)]
pub struct Jacobian {
    rows: usize,
    cols: usize,
    data: Vec<Vec<f64>>,
}

impl Jacobian {
    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![vec![0.0; cols]; rows],
        }
    }

    /// Number of rows.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Number of columns.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Reads an entry.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Row-major view of the matrix.
    pub fn rows_slice(&self) -> &[Vec<f64>] {
        &self.data
    }

    fn add(&mut self, row: usize, col: usize, value: f64) {
        self.data[row][col] += value;
    }
}

/// Inverse kinematics options.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkOptions {
    /// Maximum solver iterations.
    pub max_iterations: usize,
    /// Position convergence tolerance in meters.
    pub position_tolerance_m: f64,
    /// Orientation convergence tolerance in radians.
    pub orientation_tolerance_rad: f64,
    /// Damped least-squares regularization.
    pub damping: f64,
    /// Step scale applied to each joint update.
    pub step_size: f64,
    /// When true, drive the position rows of the Jacobian.
    pub solve_position: bool,
    /// When true, drive the orientation rows of the Jacobian.
    pub solve_orientation: bool,
}

impl Default for IkOptions {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            position_tolerance_m: 1.0e-4,
            orientation_tolerance_rad: 1.0e-3,
            damping: 1.0e-2,
            step_size: 1.0,
            solve_position: true,
            solve_orientation: true,
        }
    }
}

/// Inverse kinematics result.
#[derive(Clone, Debug, PartialEq)]
pub struct IkSolution {
    /// Solved joint positions in degree-of-freedom order.
    pub joint_positions: Vec<f64>,
    /// Iterations performed.
    pub iterations: usize,
    /// Residual position error in meters.
    pub position_error_m: f64,
    /// Residual orientation error in radians.
    pub orientation_error_rad: f64,
}

/// A full-pose inverse kinematics query for a single end link.
///
/// This mirrors MoveIt's `KinematicsBase` request shape: an end link, the
/// desired pose, a seed configuration, and solver options. The seed is always
/// expressed in the model's degree-of-freedom order.
#[derive(Clone, Debug, PartialEq)]
pub struct IkRequest {
    /// End link to drive.
    pub end_link: Entity,
    /// Desired pose of `end_link` in the model base frame.
    pub target: Pose3,
    /// Initial joint positions used as the solver seed, in DoF order.
    pub seed: Vec<f64>,
    /// Solver options.
    pub options: IkOptions,
    /// Optional active degree-of-freedom mask.
    ///
    /// When set, only the enabled joints may move and inactive joints keep their
    /// seed value. This lets a planner scope a solver to a planning group.
    pub active_dof: Option<Vec<bool>>,
}

impl IkRequest {
    /// Creates an inverse kinematics request that may move every joint.
    pub fn new(end_link: Entity, target: Pose3, seed: Vec<f64>, options: IkOptions) -> Self {
        Self {
            end_link,
            target,
            seed,
            options,
            active_dof: None,
        }
    }

    /// Restricts the solver to the given active degree-of-freedom mask.
    pub fn with_active_dof(mut self, active_dof: Vec<bool>) -> Self {
        self.active_dof = Some(active_dof);
        self
    }
}

/// Boundary implemented by swappable inverse kinematics solvers.
///
/// The kinematic model is passed in on every call rather than owned by the
/// solver, so one solver instance can serve any robot. Implementations must be
/// deterministic for a given model and request.
pub trait KinematicsSolver: Send + Sync + std::fmt::Debug {
    /// Solver name used for registry lookup.
    fn name(&self) -> &str;

    /// Solves inverse kinematics for `request` against `model`.
    fn solve(
        &self,
        model: &KinematicModel,
        request: &IkRequest,
    ) -> Result<IkSolution, KinematicsError>;

    /// Solves inverse kinematics with seeded random restarts.
    ///
    /// This is the MoveIt `searchPositionIK` analogue. The first attempt uses
    /// the request seed; later attempts sample configurations within the model's
    /// joint limits from a deterministic generator keyed by `seed`. An active
    /// mask is preserved, so only active joints are randomized and inactive
    /// joints keep their request seed value.
    fn search_position_ik(
        &self,
        model: &KinematicModel,
        request: &IkRequest,
        restarts: usize,
        seed: u64,
    ) -> Result<IkSolution, KinematicsError> {
        let mut rng = SplitMix64::new(seed);
        let mut attempt_request = request.clone();
        for attempt in 0..=restarts {
            if attempt > 0 {
                attempt_request.seed = model.random_configuration(
                    &request.seed,
                    request.active_dof.as_deref(),
                    &mut rng,
                );
            }
            if let Ok(solution) = self.solve(model, &attempt_request) {
                return Ok(solution);
            }
        }
        Err(KinematicsError::NotConverged {
            iterations: request.options.max_iterations,
        })
    }

    /// Solves inverse kinematics using the current state as the seed.
    ///
    /// The default builds an [`IkRequest`] from the state positions and
    /// delegates to [`Self::solve`].
    fn solve_from_state(
        &self,
        state: &RobotState,
        end_link: Entity,
        target: &Pose3,
        options: &IkOptions,
    ) -> Result<IkSolution, KinematicsError> {
        let request = IkRequest::new(end_link, *target, state.positions().to_vec(), *options);
        self.solve(state.model(), &request)
    }
}

/// Name of the built-in damped least-squares solver.
pub const DAMPED_LEAST_SQUARES_SOLVER: &str = "damped_least_squares";

/// Built-in damped least-squares inverse kinematics solver.
///
/// This wraps [`KinematicModel::inverse_kinematics`] behind the
/// [`KinematicsSolver`] boundary so it can be selected by name alongside other
/// solvers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DampedLeastSquaresSolver;

impl DampedLeastSquaresSolver {
    /// Creates the built-in damped least-squares solver.
    pub fn new() -> Self {
        Self
    }
}

impl KinematicsSolver for DampedLeastSquaresSolver {
    fn name(&self) -> &str {
        DAMPED_LEAST_SQUARES_SOLVER
    }

    fn solve(
        &self,
        model: &KinematicModel,
        request: &IkRequest,
    ) -> Result<IkSolution, KinematicsError> {
        match &request.active_dof {
            Some(active) => model.inverse_kinematics_active(
                &request.target,
                request.end_link,
                &request.seed,
                &request.options,
                active,
            ),
            None => model.inverse_kinematics(
                &request.target,
                request.end_link,
                &request.seed,
                &request.options,
            ),
        }
    }
}

/// Name of the built-in Jacobian-transpose solver.
pub const JACOBIAN_TRANSPOSE_SOLVER: &str = "jacobian_transpose";

/// Built-in Jacobian-transpose inverse kinematics solver.
///
/// A first-order `dq = gain * J^T e` solver with a backtracking gain. It is
/// slower than damped least squares but stable and easy to reason about, and
/// gives the solver registry a second selectable implementation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JacobianTransposeSolver;

impl JacobianTransposeSolver {
    /// Creates the built-in Jacobian-transpose solver.
    pub fn new() -> Self {
        Self
    }
}

impl KinematicsSolver for JacobianTransposeSolver {
    fn name(&self) -> &str {
        JACOBIAN_TRANSPOSE_SOLVER
    }

    fn solve(
        &self,
        model: &KinematicModel,
        request: &IkRequest,
    ) -> Result<IkSolution, KinematicsError> {
        if request.seed.len() != model.dof() {
            return Err(KinematicsError::JointCountMismatch {
                provided: request.seed.len(),
                expected: model.dof(),
            });
        }
        if !request.target.translation.is_finite() || !request.target.rotation.is_finite() {
            return Err(KinematicsError::NonFiniteInput);
        }
        let target_index = model
            .link_index(request.end_link)
            .ok_or(KinematicsError::UnknownLink(request.end_link))?;
        if !model.has_movable_ancestor(target_index) {
            return Err(KinematicsError::NoMovableChain(request.end_link));
        }
        let rows = solver_rows(&request.options);
        if rows.is_empty() {
            return Err(KinematicsError::EmptyIkObjective);
        }
        let limits = model.joint_limits();
        let mut q = request.seed.clone();
        clamp_to_joint_limits(&mut q, &limits);

        for iteration in 0..request.options.max_iterations {
            let current = model.forward_kinematics(&q)?;
            let transform = current
                .link_transform(request.end_link)
                .ok_or(KinematicsError::UnknownLink(request.end_link))?;
            let position_error = request.target.translation - transform.translation;
            let orientation = orientation_error(transform.rotation, request.target.rotation);
            let position_length = position_error.length();
            let orientation_length = orientation.length();
            let position_ok = !request.options.solve_position
                || position_length <= request.options.position_tolerance_m;
            let orientation_ok = !request.options.solve_orientation
                || orientation_length <= request.options.orientation_tolerance_rad;
            if position_ok && orientation_ok {
                return Ok(IkSolution {
                    joint_positions: q,
                    iterations: iteration,
                    position_error_m: position_length,
                    orientation_error_rad: orientation_length,
                });
            }

            let matrix = model.jacobian(&q, request.end_link, Vec3::ZERO)?;
            let mut error = [0.0_f64; 6];
            error[0] = position_error.x;
            error[1] = position_error.y;
            error[2] = position_error.z;
            error[3] = orientation.x;
            error[4] = orientation.y;
            error[5] = orientation.z;
            let mut delta = vec![0.0; model.dof()];
            for (dof, value) in delta.iter_mut().enumerate() {
                if request.active_dof.as_ref().is_some_and(|mask| !mask[dof]) {
                    continue;
                }
                *value = rows
                    .iter()
                    .map(|&row| matrix.get(row, dof) * error[row])
                    .sum();
            }
            if delta.iter().all(|value| value.abs() <= 1.0e-15) {
                return Err(KinematicsError::NotConverged {
                    iterations: iteration,
                });
            }

            let metric = |position: f64, orientation: f64| {
                let position = if request.options.solve_position {
                    position
                } else {
                    0.0
                };
                let orientation = if request.options.solve_orientation {
                    orientation
                } else {
                    0.0
                };
                position + orientation
            };
            let current_metric = metric(position_length, orientation_length);
            let mut gain = request.options.step_size;
            let mut accepted = false;
            for _ in 0..20 {
                let mut candidate = q.clone();
                for (value, step) in candidate.iter_mut().zip(delta.iter()) {
                    *value += gain * step;
                }
                clamp_to_joint_limits(&mut candidate, &limits);
                let candidate_forward = model.forward_kinematics(&candidate)?;
                let candidate_transform = candidate_forward
                    .link_transform(request.end_link)
                    .ok_or(KinematicsError::UnknownLink(request.end_link))?;
                let candidate_position =
                    (request.target.translation - candidate_transform.translation).length();
                let candidate_orientation =
                    orientation_error(candidate_transform.rotation, request.target.rotation)
                        .length();
                if metric(candidate_position, candidate_orientation) < current_metric {
                    q = candidate;
                    accepted = true;
                    break;
                }
                gain *= 0.5;
            }
            if !accepted {
                return Err(KinematicsError::NotConverged {
                    iterations: iteration,
                });
            }
        }

        Err(KinematicsError::NotConverged {
            iterations: request.options.max_iterations,
        })
    }
}

/// Name of the built-in analytic two-link solver.
pub const ANALYTIC_TWO_LINK_SOLVER: &str = "analytic_two_link";

/// Built-in closed-form solver for a planar two-revolute chain.
///
/// Detects a chain of exactly two revolute joints with parallel axes and the
/// end link off the second joint, then solves both elbow configurations in
/// closed form. This is the RNE reference to a MoveIt/IKFast analytic
/// kinematics plugin; it returns `NotConverged` for any other structure.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AnalyticTwoLinkSolver;

impl AnalyticTwoLinkSolver {
    /// Creates the built-in analytic two-link solver.
    pub fn new() -> Self {
        Self
    }
}

impl KinematicsSolver for AnalyticTwoLinkSolver {
    fn name(&self) -> &str {
        ANALYTIC_TWO_LINK_SOLVER
    }

    fn solve(
        &self,
        model: &KinematicModel,
        request: &IkRequest,
    ) -> Result<IkSolution, KinematicsError> {
        if request.seed.len() != model.dof() {
            return Err(KinematicsError::JointCountMismatch {
                provided: request.seed.len(),
                expected: model.dof(),
            });
        }
        if !request.target.translation.is_finite() || !request.target.rotation.is_finite() {
            return Err(KinematicsError::NonFiniteInput);
        }
        model
            .solve_analytic_two_link(
                &request.target,
                request.end_link,
                &request.seed,
                &request.options,
            )
            .ok_or(KinematicsError::NotConverged { iterations: 0 })
    }
}

/// Ordered registry of kinematics solvers addressable by name.
///
/// Registration preserves insertion order so [`Self::names`] is deterministic.
#[derive(Debug, Default)]
pub struct KinematicsSolverRegistry {
    solvers: Vec<Box<dyn KinematicsSolver>>,
}

impl KinematicsSolverRegistry {
    /// Creates an empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a registry preloaded with the built-in solvers.
    pub fn with_builtins() -> Self {
        let mut registry = Self::new();
        registry
            .register(Box::new(DampedLeastSquaresSolver::new()))
            .expect("built-in kinematics solver names are unique");
        registry
            .register(Box::new(JacobianTransposeSolver::new()))
            .expect("built-in kinematics solver names are unique");
        registry
            .register(Box::new(AnalyticTwoLinkSolver::new()))
            .expect("built-in kinematics solver names are unique");
        registry
    }

    /// Registers a solver, rejecting empty or duplicate names.
    pub fn register(&mut self, solver: Box<dyn KinematicsSolver>) -> Result<(), KinematicsError> {
        if solver.name().trim().is_empty() {
            return Err(KinematicsError::InvalidSolverName);
        }
        if self
            .solvers
            .iter()
            .any(|existing| existing.name() == solver.name())
        {
            return Err(KinematicsError::DuplicateSolver(solver.name().to_string()));
        }
        self.solvers.push(solver);
        Ok(())
    }

    /// Looks up a solver by name.
    pub fn get(&self, name: &str) -> Option<&dyn KinematicsSolver> {
        self.solvers
            .iter()
            .find(|solver| solver.name() == name)
            .map(|solver| solver.as_ref())
    }

    /// Registered solver names in insertion order.
    pub fn names(&self) -> Vec<&str> {
        self.solvers.iter().map(|solver| solver.name()).collect()
    }

    /// Number of registered solvers.
    pub fn len(&self) -> usize {
        self.solvers.len()
    }

    /// Whether no solver is registered.
    pub fn is_empty(&self) -> bool {
        self.solvers.is_empty()
    }
}

/// Named joint state for a robot, analogous to MoveIt's `RobotState`.
///
/// The state owns a snapshot of the kinematic model plus the current joint
/// positions and velocities in the model's degree-of-freedom order. Named
/// accessors make it convenient to seed inverse kinematics and to write results
/// back into the ECS.
#[derive(Clone, Debug)]
pub struct RobotState {
    model: KinematicModel,
    names: Vec<String>,
    positions: Vec<f64>,
    velocities: Vec<f64>,
}

impl RobotState {
    /// Reads the current joint positions and velocities from the world.
    pub fn from_world(world: &World, robot: Entity) -> Result<Self, KinematicsError> {
        let model = KinematicModel::from_robot(world, robot)?;
        let names = model.movable_dof_names();
        let mut positions = vec![0.0; model.base_dof()];
        let mut velocities = vec![0.0; model.base_dof()];
        for joint_entity in model.movable_joint_entities() {
            let joint = world
                .get::<Joint>(joint_entity)
                .ok_or(KinematicsError::MissingJoint(joint_entity))?;
            positions.push(joint.position);
            velocities.push(joint.velocity);
        }
        Ok(Self {
            model,
            names,
            positions,
            velocities,
        })
    }

    /// The kinematic model backing this state.
    pub fn model(&self) -> &KinematicModel {
        &self.model
    }

    /// Movable joint names in degree-of-freedom order.
    pub fn joint_names(&self) -> &[String] {
        &self.names
    }

    /// Joint positions in degree-of-freedom order.
    pub fn positions(&self) -> &[f64] {
        &self.positions
    }

    /// Joint velocities in degree-of-freedom order.
    pub fn velocities(&self) -> &[f64] {
        &self.velocities
    }

    /// Current position of a named joint.
    pub fn position(&self, name: &str) -> Option<f64> {
        self.index_of(name).map(|index| self.positions[index])
    }

    /// Sets a named joint position.
    pub fn set_position(&mut self, name: &str, value: f64) -> Result<(), KinematicsError> {
        if !value.is_finite() {
            return Err(KinematicsError::NonFiniteInput);
        }
        let index = self
            .index_of(name)
            .ok_or_else(|| KinematicsError::UnknownJoint(name.to_string()))?;
        self.positions[index] = value;
        Ok(())
    }

    /// Replaces all joint positions in degree-of-freedom order.
    pub fn set_positions(&mut self, positions: &[f64]) -> Result<(), KinematicsError> {
        if positions.len() != self.positions.len() {
            return Err(KinematicsError::JointCountMismatch {
                provided: positions.len(),
                expected: self.positions.len(),
            });
        }
        if positions.iter().any(|value| !value.is_finite()) {
            return Err(KinematicsError::NonFiniteInput);
        }
        self.positions.copy_from_slice(positions);
        Ok(())
    }

    /// Sets a named joint velocity.
    pub fn set_velocity(&mut self, name: &str, value: f64) -> Result<(), KinematicsError> {
        if !value.is_finite() {
            return Err(KinematicsError::NonFiniteInput);
        }
        let index = self
            .index_of(name)
            .ok_or_else(|| KinematicsError::UnknownJoint(name.to_string()))?;
        self.velocities[index] = value;
        Ok(())
    }

    /// Computes forward kinematics for the current state.
    pub fn forward_kinematics(&self) -> Result<ForwardKinematics, KinematicsError> {
        self.model.forward_kinematics(&self.positions)
    }

    /// Solves inverse kinematics using `solver` and the current state as seed.
    pub fn solve_ik(
        &self,
        solver: &dyn KinematicsSolver,
        end_link: Entity,
        target: &Pose3,
        options: &IkOptions,
    ) -> Result<IkSolution, KinematicsError> {
        solver.solve_from_state(self, end_link, target, options)
    }

    fn index_of(&self, name: &str) -> Option<usize> {
        self.names.iter().position(|candidate| candidate == name)
    }
}

fn floating_base_transform(q: &[f64]) -> Transform3 {
    let translation = Vec3::new(q[0], q[1], q[2]);
    let rotation =
        Quat::from_rotation_z(q[5]) * Quat::from_rotation_y(q[4]) * Quat::from_rotation_x(q[3]);
    Transform3::from_translation_rotation(translation, rotation)
}

fn base_joint_limit(index: usize) -> JointLimits {
    if index < 3 {
        JointLimits {
            lower: -FLOATING_BASE_TRANSLATION_LIMIT_M,
            upper: FLOATING_BASE_TRANSLATION_LIMIT_M,
            ..JointLimits::default()
        }
    } else {
        JointLimits::default()
    }
}

fn joint_motion(joint: &ModelJoint, displacement: f64) -> Transform3 {
    match joint.kind {
        JointKind::Fixed => Transform3::IDENTITY,
        JointKind::Revolute | JointKind::Continuous => {
            let axis = normalize_axis(joint.axis);
            Transform3::from_translation_rotation(
                Vec3::ZERO,
                Quat::from_axis_angle(axis, displacement),
            )
        }
        JointKind::Prismatic => {
            let axis = normalize_axis(joint.axis);
            Transform3::from_translation_rotation(axis * displacement, Quat::IDENTITY)
        }
    }
}

fn transform_point(transform: &Transform3, point: Vec3) -> Vec3 {
    transform.translation + transform.rotation * (transform.scale * point)
}

fn world_axis(frame: &Transform3, axis: Vec3) -> Vec3 {
    (frame.rotation * normalize_axis(axis)).normalize_or_zero()
}

fn normalize_axis(axis: Vec3) -> Vec3 {
    let normalized = axis.normalize_or_zero();
    if normalized.length_squared() <= f64::EPSILON {
        Vec3::Y
    } else {
        normalized
    }
}

fn orientation_error(current: Quat, target: Quat) -> Vec3 {
    let mut delta = target * current.conjugate();
    if delta.w < 0.0 {
        delta = -delta;
    }
    let vector = Vec3::new(delta.x, delta.y, delta.z);
    let sin_half = vector.length();
    if sin_half <= 1.0e-12 {
        2.0 * vector
    } else {
        let angle = 2.0 * sin_half.atan2(delta.w);
        vector / sin_half * angle
    }
}

/// Deterministic SplitMix64 generator used for seeded IK restarts.
#[derive(Clone, Debug)]
pub(crate) struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    pub(crate) fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1_u64 << 53) as f64
    }
}

#[allow(clippy::needless_range_loop)]
fn determinant6(matrix: &mut [[f64; 6]; 6]) -> f64 {
    let mut determinant = 1.0;
    for column in 0..6 {
        let mut pivot = column;
        for row in (column + 1)..6 {
            if matrix[row][column].abs() > matrix[pivot][column].abs() {
                pivot = row;
            }
        }
        if matrix[pivot][column].abs() < 1.0e-18 {
            return 0.0;
        }
        if pivot != column {
            matrix.swap(pivot, column);
            determinant = -determinant;
        }
        let pivot_row = matrix[column];
        let diagonal = pivot_row[column];
        determinant *= diagonal;
        for row in (column + 1)..6 {
            let factor = matrix[row][column] / diagonal;
            for (target, &source) in matrix[row].iter_mut().zip(&pivot_row).skip(column) {
                *target -= factor * source;
            }
        }
    }
    determinant
}

fn clamp_to_joint_limits(positions: &mut [f64], limits: &[JointLimits]) {
    for (value, limit) in positions.iter_mut().zip(limits) {
        let lower = if limit.lower.is_finite() {
            limit.lower
        } else {
            f64::NEG_INFINITY
        };
        let upper = if limit.upper.is_finite() {
            limit.upper
        } else {
            f64::INFINITY
        };
        *value = value.clamp(lower, upper);
    }
}

fn solver_rows(options: &IkOptions) -> Vec<usize> {
    let mut rows = Vec::with_capacity(6);
    if options.solve_position {
        rows.extend([0, 1, 2]);
    }
    if options.solve_orientation {
        rows.extend([3, 4, 5]);
    }
    rows
}

fn damped_least_squares(
    matrix: &Jacobian,
    error: &[f64; 6],
    rows: &[usize],
    options: &IkOptions,
) -> Result<Vec<f64>, KinematicsError> {
    if matrix.cols() == 0 {
        return Ok(Vec::new());
    }
    let active_rows = rows.len();
    let all_rows = matrix.rows_slice();
    let mut normal = vec![vec![0.0; active_rows]; active_rows];
    for (r, row) in normal.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = all_rows[rows[r]]
                .iter()
                .zip(&all_rows[rows[c]])
                .map(|(a, b)| a * b)
                .sum();
        }
    }
    let regularization = options.damping * options.damping;
    for (r, row) in normal.iter_mut().enumerate() {
        row[r] += regularization;
    }
    let mut rhs: Vec<f64> = rows.iter().map(|&row| error[row]).collect();
    if !solve_linear(&mut normal, &mut rhs) {
        return Err(KinematicsError::NonFiniteInput);
    }
    let mut delta = vec![0.0; matrix.cols()];
    for (col, value) in delta.iter_mut().enumerate() {
        *value = options.step_size
            * rows
                .iter()
                .zip(&rhs)
                .map(|(&row, y)| all_rows[row][col] * y)
                .sum::<f64>();
    }
    if delta.iter().any(|value| !value.is_finite()) {
        return Err(KinematicsError::NonFiniteInput);
    }
    Ok(delta)
}

fn solve_linear(matrix: &mut [Vec<f64>], rhs: &mut [f64]) -> bool {
    let n = rhs.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &b| matrix[a][col].abs().total_cmp(&matrix[b][col].abs()))
            .unwrap_or(col);
        if matrix[pivot][col].abs() < 1.0e-12 {
            return false;
        }
        matrix.swap(col, pivot);
        rhs.swap(col, pivot);
        let pivot_row = matrix[col].clone();
        let diagonal = pivot_row[col];
        for row in (col + 1)..n {
            let factor = matrix[row][col] / diagonal;
            for (target, source) in matrix[row].iter_mut().zip(&pivot_row).skip(col) {
                *target -= factor * source;
            }
            rhs[row] -= factor * rhs[col];
        }
    }
    let mut solution = vec![0.0; n];
    for row in (0..n).rev() {
        let known: f64 = matrix[row][(row + 1)..]
            .iter()
            .zip(&solution[(row + 1)..])
            .map(|(coefficient, value)| coefficient * value)
            .sum();
        solution[row] = (rhs[row] - known) / matrix[row][row];
    }
    for (value, out) in solution.iter().zip(rhs.iter_mut()) {
        *out = *value;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::{spawn_named, World};
    use std::f64::consts::FRAC_PI_2;

    fn planar_arm_world() -> (World, Entity, Entity, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "arm");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let ee = spawn_named(&mut world, "ee");
        let tip = spawn_named(&mut world, "tip");

        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(ee).insert((
            Link {
                robot,
                name: "ee".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        ));
        world.entity_mut(tip).insert((
            Link {
                robot,
                name: "tip".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "arm".into(),
            base_link: base,
        });

        let joint1 = spawn_named(&mut world, "joint1");
        world.entity_mut(joint1).insert(Joint {
            robot,
            parent_link: base,
            child_link: link1,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
        let joint2 = spawn_named(&mut world, "joint2");
        world.entity_mut(joint2).insert(Joint {
            robot,
            parent_link: link1,
            child_link: ee,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
        let joint3 = spawn_named(&mut world, "joint3");
        world.entity_mut(joint3).insert(Joint {
            robot,
            parent_link: ee,
            child_link: tip,
            kind: JointKind::Fixed,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });

        (world, robot, ee, tip)
    }

    fn planar_arm() -> (KinematicModel, Entity, Entity) {
        let (world, robot, ee, tip) = planar_arm_world();
        (KinematicModel::from_robot(&world, robot).unwrap(), ee, tip)
    }

    #[test]
    fn forward_kinematics_matches_planar_arm() {
        let (model, ee, tip) = planar_arm();
        assert_eq!(model.dof(), 2);
        let q = [FRAC_PI_2, 0.0];
        let state = model.forward_kinematics(&q).unwrap();
        let end = state.link_transform(ee).unwrap().translation;
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(end.y, 1.0, epsilon = 1e-9);
        assert_relative_eq!(end.z, 0.0, epsilon = 1e-9);
        let tip_position = state.link_transform(tip).unwrap().translation;
        assert_relative_eq!(tip_position.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(tip_position.y, 2.0, epsilon = 1e-9);
    }

    #[test]
    fn jacobian_matches_analytic_planar_velocity() {
        let (model, ee, tip) = planar_arm();
        let q = [0.3, -0.4];
        let jacobian = model.jacobian(&q, tip, Vec3::ZERO).unwrap();
        let state = model.forward_kinematics(&q).unwrap();
        let tip_position = state.link_transform(tip).unwrap().translation;
        let joint1_origin = Vec3::ZERO;
        let joint2_origin = state.link_transform(ee).unwrap().translation;

        let expected_col1 = Vec3::Z.cross(tip_position - joint1_origin);
        let expected_col2 = Vec3::Z.cross(tip_position - joint2_origin);
        assert_relative_eq!(jacobian.get(0, 0), expected_col1.x, epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(1, 0), expected_col1.y, epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(0, 1), expected_col2.x, epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(1, 1), expected_col2.y, epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(5, 0), 1.0, epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(5, 1), 1.0, epsilon = 1e-9);
    }

    #[test]
    fn inverse_kinematics_reaches_position() {
        let (model, _ee, tip) = planar_arm();
        let target = Pose3 {
            translation: Vec3::new(1.2, 0.5, 0.0),
            rotation: Quat::IDENTITY,
        };
        let options = IkOptions {
            solve_orientation: false,
            ..IkOptions::default()
        };
        let solution = model
            .inverse_kinematics(&target, tip, &[0.2, 0.2], &options)
            .unwrap();
        let state = model.forward_kinematics(&solution.joint_positions).unwrap();
        let end = state.link_transform(tip).unwrap().translation;
        assert!((end - target.translation).length() < 1.0e-3, "end={end:?}");
    }

    #[test]
    fn prismatic_joint_translates_along_axis() {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "slider");
        let base = spawn_named(&mut world, "base");
        let slider = spawn_named(&mut world, "slider_link");
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(slider).insert((
            Link {
                robot,
                name: "slider_link".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "slider".into(),
            base_link: base,
        });
        let joint = spawn_named(&mut world, "slide");
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: slider,
            kind: JointKind::Prismatic,
            limits: JointLimits::default(),
            axis: Vec3::X,
            position: 0.0,
            velocity: 0.0,
        });

        let model = KinematicModel::from_robot(&world, robot).unwrap();
        let state = model.forward_kinematics(&[0.25]).unwrap();
        let position = state.link_transform(slider).unwrap().translation;
        assert_relative_eq!(position.x, 0.25, epsilon = 1e-12);
        assert_relative_eq!(position.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn solver_registry_exposes_builtin_and_rejects_duplicates() {
        let mut registry = KinematicsSolverRegistry::with_builtins();
        assert_eq!(
            registry.names(),
            vec![
                DAMPED_LEAST_SQUARES_SOLVER,
                JACOBIAN_TRANSPOSE_SOLVER,
                ANALYTIC_TWO_LINK_SOLVER
            ]
        );
        assert!(registry.get(DAMPED_LEAST_SQUARES_SOLVER).is_some());
        assert!(registry.get(JACOBIAN_TRANSPOSE_SOLVER).is_some());
        assert!(registry.get(ANALYTIC_TWO_LINK_SOLVER).is_some());
        assert!(registry.get("missing").is_none());
        assert_eq!(registry.len(), 3);

        assert_eq!(
            registry.register(Box::new(DampedLeastSquaresSolver::new())),
            Err(KinematicsError::DuplicateSolver(
                DAMPED_LEAST_SQUARES_SOLVER.to_string()
            ))
        );
    }

    #[test]
    fn damped_least_squares_solver_solves_request() {
        let (model, _ee, tip) = planar_arm();
        let solver = DampedLeastSquaresSolver::new();
        let request = IkRequest::new(
            tip,
            Pose3 {
                translation: Vec3::new(1.2, 0.5, 0.0),
                rotation: Quat::IDENTITY,
            },
            vec![0.2, 0.2],
            IkOptions {
                solve_orientation: false,
                ..IkOptions::default()
            },
        );
        let solution = solver.solve(&model, &request).unwrap();
        let state = model.forward_kinematics(&solution.joint_positions).unwrap();
        let reached = state.link_transform(tip).unwrap().translation;
        assert!((reached - request.target.translation).length() < 1.0e-3);
    }

    #[test]
    fn robot_state_reads_and_writes_named_joints() {
        let (world, robot, ee, _tip) = planar_arm_world();
        let solver = DampedLeastSquaresSolver::new();
        let mut state = RobotState::from_world(&world, robot).unwrap();
        assert_eq!(
            state
                .joint_names()
                .iter()
                .map(String::as_str)
                .collect::<Vec<_>>(),
            vec!["joint1", "joint2"]
        );
        assert_eq!(state.positions(), [0.0, 0.0]);

        state.set_position("joint1", FRAC_PI_2).unwrap();
        let fk = state.forward_kinematics().unwrap();
        let end = fk.link_transform(ee).unwrap().translation;
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(end.y, 1.0, epsilon = 1e-9);

        let target = Pose3 {
            translation: Vec3::new(0.6, 0.8, 0.0),
            rotation: Quat::IDENTITY,
        };
        let solution = state
            .solve_ik(
                &solver,
                ee,
                &target,
                &IkOptions {
                    solve_orientation: false,
                    ..IkOptions::default()
                },
            )
            .unwrap();
        let reached = state
            .model()
            .forward_kinematics(&solution.joint_positions)
            .unwrap()
            .link_transform(ee)
            .unwrap()
            .translation;
        assert!((reached - target.translation).length() < 1.0e-3);
    }

    #[test]
    fn robot_state_rejects_unknown_joint() {
        let (world, robot, _ee, _tip) = planar_arm_world();
        let mut state = RobotState::from_world(&world, robot).unwrap();
        assert_eq!(
            state.set_position("missing", 0.0),
            Err(KinematicsError::UnknownJoint("missing".to_string()))
        );
        assert!(state.position("missing").is_none());
        assert_eq!(state.position("joint1"), Some(0.0));
    }

    fn mimic_arm(source_is_joint: bool) -> (World, Entity, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "mimic_robot");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let finger = spawn_named(&mut world, "finger");

        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(finger).insert((
            Link {
                robot,
                name: "finger".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "mimic".into(),
            base_link: base,
        });

        let joint1 = spawn_named(&mut world, "joint1");
        world.entity_mut(joint1).insert(Joint {
            robot,
            parent_link: base,
            child_link: link1,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
        let finger_joint = spawn_named(&mut world, "finger_joint");
        world.entity_mut(finger_joint).insert((
            Joint {
                robot,
                parent_link: link1,
                child_link: finger,
                kind: JointKind::Revolute,
                limits: JointLimits::default(),
                axis: Vec3::Z,
                position: 0.0,
                velocity: 0.0,
            },
            MimicJoint::new(if source_is_joint { joint1 } else { link1 }, 2.0, 0.1),
        ));

        (world, robot, finger)
    }

    #[test]
    fn mimic_joint_is_derived_and_not_a_dof() {
        let (world, robot, finger) = mimic_arm(true);
        let model = KinematicModel::from_robot(&world, robot).unwrap();
        assert_eq!(model.dof(), 1);
        assert_eq!(model.movable_joint_names(), vec!["joint1"]);
        assert_eq!(model.mimic_joint_entities().len(), 1);

        let state = model.forward_kinematics(&[0.5]).unwrap();
        let transform = state.link_transform(finger).unwrap();
        let expected = Quat::from_rotation_z(1.6);
        assert_relative_eq!(transform.rotation.x, expected.x, epsilon = 1e-9);
        assert_relative_eq!(transform.rotation.y, expected.y, epsilon = 1e-9);
        assert_relative_eq!(transform.rotation.z, expected.z, epsilon = 1e-9);
        assert_relative_eq!(transform.rotation.w, expected.w, epsilon = 1e-9);
    }

    #[test]
    fn mimic_joint_contributes_to_jacobian() {
        let (world, robot, finger) = mimic_arm(true);
        let model = KinematicModel::from_robot(&world, robot).unwrap();
        let jacobian = model.jacobian(&[0.5], finger, Vec3::ZERO).unwrap();
        assert_relative_eq!(jacobian.get(0, 0), -0.5_f64.sin(), epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(1, 0), 0.5_f64.cos(), epsilon = 1e-9);
        assert_relative_eq!(jacobian.get(5, 0), 3.0, epsilon = 1e-9);
    }

    #[test]
    fn mimic_joint_rejects_invalid_source() {
        let (world, robot, _finger) = mimic_arm(false);
        assert!(matches!(
            KinematicModel::from_robot(&world, robot),
            Err(KinematicsError::InvalidMimicJoint { .. })
        ));
    }

    #[test]
    fn chain_joints_lists_base_to_tip() {
        let (world, robot, _ee, tip) = planar_arm_world();
        let model = KinematicModel::from_robot(&world, robot).unwrap();
        let base = model.base_link();
        let chain = model.chain_joints(base, tip).unwrap();
        assert_eq!(chain.len(), 3);
        assert_eq!(model.dof_index_of_joint(chain[0]), Some(0));
        assert_eq!(model.dof_index_of_joint(chain[1]), Some(1));
        assert_eq!(model.dof_index_of_joint(chain[2]), None);
        assert!(matches!(
            model.chain_joints(tip, base),
            Err(KinematicsError::ChainNotFound { .. })
        ));
    }

    #[test]
    fn active_ik_holds_inactive_joints() {
        let (world, robot, _ee, tip) = planar_arm_world();
        let model = KinematicModel::from_robot(&world, robot).unwrap();
        let desired = [0.8, 0.2];
        let target = {
            let state = model.forward_kinematics(&desired).unwrap();
            Pose3 {
                translation: state.link_transform(tip).unwrap().translation,
                rotation: Quat::IDENTITY,
            }
        };
        let options = IkOptions {
            solve_orientation: false,
            ..IkOptions::default()
        };
        let solution = model
            .inverse_kinematics_active(&target, tip, &[0.2, 0.2], &options, &[true, false])
            .unwrap();
        assert_relative_eq!(solution.joint_positions[1], 0.2, epsilon = 1e-12);
        let state = model.forward_kinematics(&solution.joint_positions).unwrap();
        let reached = state.link_transform(tip).unwrap().translation;
        assert!((reached - target.translation).length() < 1.0e-3);
    }

    #[test]
    fn jacobian_transpose_solver_reaches_position() {
        let (model, _ee, tip) = planar_arm();
        let target = {
            let state = model.forward_kinematics(&[0.9, 0.4]).unwrap();
            Pose3 {
                translation: state.link_transform(tip).unwrap().translation,
                rotation: Quat::IDENTITY,
            }
        };
        let options = IkOptions {
            solve_orientation: false,
            max_iterations: 5_000,
            ..IkOptions::default()
        };
        let request = IkRequest::new(tip, target, vec![0.2, 0.2], options);
        let solution = JacobianTransposeSolver::new()
            .solve(&model, &request)
            .unwrap();
        let reached = model
            .forward_kinematics(&solution.joint_positions)
            .unwrap()
            .link_transform(tip)
            .unwrap()
            .translation;
        assert!((reached - target.translation).length() < 1.0e-3);
    }

    #[test]
    fn analytic_two_link_solver_reaches_position_exactly() {
        let (model, _ee, tip) = planar_arm();
        let target = {
            let state = model.forward_kinematics(&[0.7, 0.5]).unwrap();
            Pose3 {
                translation: state.link_transform(tip).unwrap().translation,
                rotation: Quat::IDENTITY,
            }
        };
        let options = IkOptions {
            solve_orientation: false,
            ..IkOptions::default()
        };
        let request = IkRequest::new(tip, target, vec![0.2, 0.2], options);
        let solution = AnalyticTwoLinkSolver::new()
            .solve(&model, &request)
            .unwrap();
        assert_eq!(solution.iterations, 0);
        let reached = model
            .forward_kinematics(&solution.joint_positions)
            .unwrap()
            .link_transform(tip)
            .unwrap()
            .translation;
        assert!((reached - target.translation).length() < 1.0e-6);
    }

    #[test]
    fn search_position_ik_finds_reachable_target_deterministically() {
        let (model, _ee, tip) = planar_arm();
        let target = {
            let state = model.forward_kinematics(&[0.8, 0.2]).unwrap();
            let transform = state.link_transform(tip).unwrap();
            Pose3 {
                translation: transform.translation,
                rotation: Quat::IDENTITY,
            }
        };
        let options = IkOptions {
            solve_orientation: false,
            ..IkOptions::default()
        };
        let request = IkRequest::new(tip, target, vec![2.5, -2.5], options);
        let solver = DampedLeastSquaresSolver::new();
        let first = solver.search_position_ik(&model, &request, 20, 7).unwrap();
        let second = solver.search_position_ik(&model, &request, 20, 7).unwrap();
        assert_eq!(first.joint_positions, second.joint_positions);
        let reached = model
            .forward_kinematics(&first.joint_positions)
            .unwrap()
            .link_transform(tip)
            .unwrap()
            .translation;
        assert!((reached - target.translation).length() < 1.0e-3);
    }

    #[test]
    fn manipulability_is_non_negative_and_finite() {
        let (model, _ee, tip) = planar_arm();
        let manipulability = model.manipulability(&[0.5, 1.5], tip).unwrap();
        assert!(manipulability.is_finite());
        assert!(manipulability >= 0.0);
    }

    #[test]
    fn joint_limit_distance_measures_nearest_bound() {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "bounded");
        let base = spawn_named(&mut world, "base");
        let slider = spawn_named(&mut world, "slider");
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(slider).insert((
            Link {
                robot,
                name: "slider".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "bounded".into(),
            base_link: base,
        });
        let joint = spawn_named(&mut world, "slide");
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: slider,
            kind: JointKind::Prismatic,
            limits: JointLimits {
                lower: -1.0,
                upper: 1.0,
                ..JointLimits::default()
            },
            axis: Vec3::X,
            position: 0.0,
            velocity: 0.0,
        });

        let model = KinematicModel::from_robot(&world, robot).unwrap();
        assert_relative_eq!(model.joint_limit_distance(&[0.25]), 0.75, epsilon = 1e-12);
        assert_relative_eq!(model.joint_limit_distance(&[-0.9]), 0.1, epsilon = 1e-12);
    }

    #[test]
    fn floating_base_adds_six_dof_and_moves_the_root() {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "mobile");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            FloatingBase,
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "mobile".into(),
            base_link: base,
        });
        let joint = spawn_named(&mut world, "joint1");
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: link1,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });

        let model = KinematicModel::from_robot(&world, robot).unwrap();
        assert_eq!(model.dof(), 7);
        assert_eq!(model.base_dof(), 6);
        assert_eq!(model.movable_dof_names()[0], "base_x");
        assert_eq!(model.movable_dof_names()[6], "joint1");
        assert_eq!(model.joint_limits().len(), 7);

        let state = model
            .forward_kinematics(&[1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            .unwrap();
        let link = state.link_transform(link1).unwrap().translation;
        assert_relative_eq!(link.x, 2.0, epsilon = 1e-12);
        assert_relative_eq!(link.y, 2.0, epsilon = 1e-12);

        let state = model
            .forward_kinematics(&[0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2, 0.0])
            .unwrap();
        let link = state.link_transform(link1).unwrap().translation;
        assert_relative_eq!(link.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(link.y, 1.0, epsilon = 1e-9);

        let jacobian = model.jacobian(&[0.0; 7], link1, Vec3::ZERO).unwrap();
        assert_relative_eq!(jacobian.get(0, 0), 1.0, epsilon = 1e-12);
        assert_relative_eq!(jacobian.get(1, 1), 1.0, epsilon = 1e-12);
        assert_relative_eq!(jacobian.get(2, 2), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn passive_joint_keeps_its_dof() {
        let (world, robot, _link1, _tip) = planar_arm_world();
        let joint2 = *KinematicModel::from_robot(&world, robot)
            .unwrap()
            .movable_joint_entities()
            .last()
            .unwrap();
        let mut world = world;
        world.entity_mut(joint2).insert(PassiveJoint);

        let model = KinematicModel::from_robot(&world, robot).unwrap();
        assert_eq!(model.dof(), 2);
        assert_eq!(model.passive_joint_entities(), vec![joint2]);
    }
}
