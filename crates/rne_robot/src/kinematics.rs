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

use crate::components::{Joint, JointKind, JointLimits, Link, Robot};
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
}

#[derive(Clone, Debug)]
struct ModelLink {
    entity: Entity,
    name: String,
    parent: Option<usize>,
    joint: Option<usize>,
    local: Transform3,
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
    entity_to_link: HashMap<Entity, usize>,
}

impl KinematicModel {
    /// Builds a model for the given robot entity.
    pub fn from_robot(world: &World, robot: Entity) -> Result<Self, KinematicsError> {
        let robot_component = world
            .get::<Robot>(robot)
            .ok_or(KinematicsError::MissingRobot(robot))?;
        let base_entity = robot_component.base_link;

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

        struct RawJoint {
            entity: Entity,
            name: String,
            parent: usize,
            child: usize,
            kind: JointKind,
            axis: Vec3,
            limits: JointLimits,
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
            old_joint_index[child] = Some(raw_joints.len());
            raw_joints.push(RawJoint {
                entity: entity_ref.id(),
                name,
                parent,
                child,
                kind: joint.kind,
                axis: joint.axis,
                limits: joint.limits,
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
        for joint in raw_joints {
            let child_link = new_index[joint.child];
            let joint_index = joints.len();
            let dof_index = if joint.kind == JointKind::Fixed {
                None
            } else {
                let index = dof.len();
                dof.push(joint_index);
                Some(index)
            };
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
            });
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

    /// Number of movable degrees of freedom.
    pub fn dof(&self) -> usize {
        self.dof.len()
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

    /// Joint limits in degree-of-freedom order.
    pub fn joint_limits(&self) -> Vec<JointLimits> {
        self.dof
            .iter()
            .map(|&index| self.joints[index].limits)
            .collect()
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
        if q.len() != self.dof.len() {
            return Err(KinematicsError::JointCountMismatch {
                provided: q.len(),
                expected: self.dof.len(),
            });
        }
        if q.iter().any(|value| !value.is_finite()) {
            return Err(KinematicsError::NonFiniteInput);
        }

        let mut transforms = vec![Transform3::IDENTITY; self.links.len()];
        for (index, link) in self.links.iter().enumerate() {
            match link.parent {
                Some(parent) => {
                    let joint_index = link
                        .joint
                        .expect("every non-root link is connected by a joint");
                    let joint = &self.joints[joint_index];
                    let displacement = joint.dof.map(|dof| q[dof]).unwrap_or(0.0);
                    let motion = joint_motion(joint, displacement);
                    transforms[index] = transforms[parent]
                        .mul_transform(&link.local)
                        .mul_transform(&motion);
                }
                None => {
                    transforms[index] = base_pose.copied().unwrap_or(link.local);
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

        let mut matrix = Jacobian::zeros(6, self.dof.len());
        let point_world = transform_point(&state.transforms[target], point_local);

        let mut index = target;
        while let Some(joint_index) = self.links[index].joint {
            let joint = &self.joints[joint_index];
            if let Some(dof) = joint.dof {
                let zero_frame = state.transforms[joint.parent_link]
                    .mul_transform(&self.links[joint.child_link].local);
                let axis = world_axis(&zero_frame, joint.axis);
                let origin = zero_frame.translation;
                match joint.kind {
                    JointKind::Prismatic => {
                        matrix.set(0, dof, axis.x);
                        matrix.set(1, dof, axis.y);
                        matrix.set(2, dof, axis.z);
                    }
                    _ => {
                        let r = point_world - origin;
                        let linear = axis.cross(r);
                        matrix.set(0, dof, linear.x);
                        matrix.set(1, dof, linear.y);
                        matrix.set(2, dof, linear.z);
                        matrix.set(3, dof, axis.x);
                        matrix.set(4, dof, axis.y);
                        matrix.set(5, dof, axis.z);
                    }
                }
            }
            index = joint.parent_link;
        }

        Ok(matrix)
    }

    /// Solves inverse kinematics for a full pose target.
    ///
    /// Uses damped least squares with joint-limit clamping. When
    /// [`IkOptions::solve_position_only`] is set, only the position rows are
    /// driven.
    pub fn inverse_kinematics(
        &self,
        target: &Pose3,
        end_link: Entity,
        initial: &[f64],
        options: &IkOptions,
    ) -> Result<IkSolution, KinematicsError> {
        if initial.len() != self.dof.len() {
            return Err(KinematicsError::JointCountMismatch {
                provided: initial.len(),
                expected: self.dof.len(),
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

        let mut q = initial.to_vec();
        self.clamp_to_limits(&mut q, true);

        for iteration in 0..options.max_iterations {
            let state = self.forward_kinematics(&q)?;
            let current = &state.transforms[target_index];
            let position_error = target.translation - current.translation;
            let orientation_error = orientation_error(current.rotation, target.rotation);
            let position_length = position_error.length();
            let orientation_length = orientation_error.length();

            let position_ok = position_length <= options.position_tolerance_m;
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

            let matrix = self.jacobian(&q, end_link, Vec3::ZERO)?;
            let mut error = vec![0.0; 6];
            error[0] = position_error.x;
            error[1] = position_error.y;
            error[2] = position_error.z;
            if options.solve_orientation {
                error[3] = orientation_error.x;
                error[4] = orientation_error.y;
                error[5] = orientation_error.z;
            }
            let active_rows = if options.solve_orientation { 6 } else { 3 };
            let delta = damped_least_squares(&matrix, &error, active_rows, options)?;
            for (value, step) in q.iter_mut().zip(delta.iter()) {
                *value += step;
            }
            self.clamp_to_limits(&mut q, false);
        }

        Err(KinematicsError::NotConverged {
            iterations: options.max_iterations,
        })
    }

    fn has_movable_ancestor(&self, mut index: usize) -> bool {
        while let Some(joint_index) = self.links[index].joint {
            let joint = &self.joints[joint_index];
            if joint.dof.is_some() {
                return true;
            }
            index = joint.parent_link;
        }
        false
    }

    fn clamp_to_limits(&self, q: &mut [f64], round: bool) {
        for (dof, value) in q.iter_mut().enumerate() {
            let joint = &self.joints[self.dof[dof]];
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

    fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row][col] = value;
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
    /// When true, drive position and orientation; otherwise position only.
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

fn damped_least_squares(
    matrix: &Jacobian,
    error: &[f64],
    active_rows: usize,
    options: &IkOptions,
) -> Result<Vec<f64>, KinematicsError> {
    if matrix.cols() == 0 {
        return Ok(Vec::new());
    }
    let mut normal = vec![vec![0.0; active_rows]; active_rows];
    let rows = matrix.rows_slice();
    for (r, row) in normal.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = rows[r].iter().zip(&rows[c]).map(|(a, b)| a * b).sum();
        }
    }
    let regularization = options.damping * options.damping;
    for (r, row) in normal.iter_mut().enumerate() {
        row[r] += regularization;
    }
    let mut rhs: Vec<f64> = error[..active_rows].to_vec();
    if !solve_linear(&mut normal, &mut rhs) {
        return Err(KinematicsError::NonFiniteInput);
    }
    let mut delta = vec![0.0; matrix.cols()];
    for (col, value) in delta.iter_mut().enumerate() {
        *value = options.step_size
            * rows
                .iter()
                .zip(&rhs)
                .map(|(row, y)| row[col] * y)
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

    fn planar_arm() -> (KinematicModel, Entity, Entity) {
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

        let model = KinematicModel::from_robot(&world, robot).unwrap();
        (model, ee, tip)
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
}
