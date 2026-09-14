//! The weighted inverse-dynamics whole-body controller.

use crate::contact::{ContactPoint, FrictionCone};
use rne_dynamics::{
    center_of_mass, com_jacobian, link_motions, mass_matrix, non_linear_effects, ArticulatedModel,
    DenseMatrix, DynamicsError,
};
use rne_ecs::Entity;
use rne_math::Vec3;
use thiserror::Error;

/// Error returned while building or solving a whole-body control problem.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum WbcError {
    /// The underlying dynamics evaluation failed.
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
    /// The underlying kinematics evaluation failed.
    #[error(transparent)]
    Kinematics(#[from] rne_robot::KinematicsError),
    /// Whole-body control currently requires the six floating-base degrees of
    /// freedom.
    #[error("whole-body control requires a floating-base model")]
    RequiresFloatingBase,
    /// No contact points were provided.
    #[error("whole-body control requires at least one contact point")]
    NoContacts,
    /// A contact referenced a link that is not part of the model.
    #[error("contact link {0:?} is not part of the articulated model")]
    UnknownContactLink(Entity),
    /// Torque limits did not match the number of actuated joints.
    #[error("expected {expected} torque limits but received {provided}")]
    TorqueLimitDimension {
        /// Number of actuated joints.
        expected: usize,
        /// Number of limits provided.
        provided: usize,
    },
    /// A posture task did not match the number of actuated joints.
    #[error("posture task expects {expected} joints but the model has {provided}")]
    PostureDimension {
        /// Number of actuated joints.
        expected: usize,
        /// Number of joint targets provided.
        provided: usize,
    },
    /// A contact point or task value was invalid.
    #[error("whole-body control input is invalid: {0}")]
    InvalidInput(&'static str),
    /// The weighted normal equations were singular.
    #[error("weighted normal equations are singular")]
    SingularSystem,
}

/// Weights for the whole-body control least-squares problem.
#[derive(Clone, Debug, PartialEq)]
pub struct WholeBodyConfig {
    /// Weight on the floating-base equations of motion.
    pub dynamics_weight: f64,
    /// Weight on the contact no-slip rows.
    pub contact_weight: f64,
    /// Weight on the center-of-mass task.
    pub com_weight: f64,
    /// Weight on the posture task.
    pub posture_weight: f64,
    /// Tikhonov weight on the contact forces (pulls them toward zero).
    pub force_regularization: f64,
    /// Tikhonov weight on the joint accelerations (pulls them toward zero).
    pub acceleration_regularization: f64,
    /// Small diagonal added to the normal equations for numerical rank.
    pub solver_regularization: f64,
    /// Optional symmetric joint torque limit per actuated joint, in N·m.
    pub torque_limits_nm: Option<Vec<f64>>,
}

impl Default for WholeBodyConfig {
    fn default() -> Self {
        Self {
            dynamics_weight: 1.0e6,
            contact_weight: 1.0e6,
            com_weight: 1.0e2,
            posture_weight: 1.0,
            force_regularization: 1.0e-4,
            acceleration_regularization: 1.0e-4,
            solver_regularization: 1.0e-9,
            torque_limits_nm: None,
        }
    }
}

/// Center-of-mass tracking task.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ComTask {
    /// Desired center-of-mass position in world coordinates, in meters.
    pub desired_position_m: Vec3,
    /// Desired center-of-mass velocity in world coordinates, in m/s.
    pub desired_velocity_m_s: Vec3,
    /// Feed-forward center-of-mass acceleration in world coordinates, in m/s².
    pub desired_acceleration_m_s2: Vec3,
    /// Position feedback gain in inverse seconds squared.
    pub position_gain_s_inv2: f64,
    /// Velocity feedback gain in inverse seconds.
    pub velocity_gain_s_inv: f64,
}

impl ComTask {
    /// Holds the center of mass at `position_m` with critical-style gains.
    pub fn hold(position_m: Vec3) -> Self {
        Self {
            desired_position_m: position_m,
            desired_velocity_m_s: Vec3::ZERO,
            desired_acceleration_m_s2: Vec3::ZERO,
            position_gain_s_inv2: 100.0,
            velocity_gain_s_inv: 20.0,
        }
    }
}

/// Joint-space posture task pulling the actuated joints toward a target.
#[derive(Clone, Debug, PartialEq)]
pub struct PostureTask {
    /// Desired actuated joint positions in degrees of freedom order, in rad or m.
    pub desired_joint_positions: Vec<f64>,
    /// Position feedback gain in inverse seconds squared.
    pub position_gain_s_inv2: f64,
    /// Velocity feedback gain in inverse seconds.
    pub velocity_gain_s_inv: f64,
}

/// Result of one whole-body control solve.
#[derive(Clone, Debug, PartialEq)]
pub struct WholeBodySolution {
    /// Solved generalized accelerations, in degrees of freedom order.
    pub joint_acceleration: Vec<f64>,
    /// Friction-projected world-frame contact forces, in newtons.
    pub contact_forces_world_n: Vec<Vec3>,
    /// Required actuated joint torques, in N·m or N.
    pub joint_torque_nm: Vec<f64>,
    /// Residual of the floating-base equations of motion, in N and N·m.
    pub base_wrench_residual: [f64; 6],
    /// Point-acceleration residual at each contact, in m/s².
    pub contact_acceleration_residual_m_s2: Vec<f64>,
    /// Realized center-of-mass acceleration, in m/s².
    pub com_acceleration_m_s2: Vec3,
    /// Whether any joint torque was clipped by the configured limit.
    pub torque_saturated: bool,
}

/// Weighted inverse-dynamics whole-body controller.
#[derive(Clone, Debug, PartialEq)]
pub struct WholeBodyController {
    config: WholeBodyConfig,
}

impl WholeBodyController {
    /// Creates a controller with the given weights.
    pub fn new(config: WholeBodyConfig) -> Self {
        Self { config }
    }

    /// The controller weights.
    pub fn config(&self) -> &WholeBodyConfig {
        &self.config
    }

    /// Solves for joint accelerations, contact forces, and joint torques.
    ///
    /// `contacts` are the active point contacts. `com_task` and `posture_task`
    /// are optional; at least one task plus the dynamics and contact rows make
    /// the problem well-posed.
    #[allow(clippy::needless_range_loop)]
    pub fn solve(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[ContactPoint],
        com_task: Option<&ComTask>,
        posture_task: Option<&PostureTask>,
    ) -> Result<WholeBodySolution, WbcError> {
        if model.base_dof() != 6 {
            return Err(WbcError::RequiresFloatingBase);
        }
        if contacts.is_empty() {
            return Err(WbcError::NoContacts);
        }
        if q.iter().chain(qd).any(|value| !value.is_finite()) {
            return Err(WbcError::InvalidInput("configuration is not finite"));
        }

        let nv = model.nv();
        let nj = nv - model.base_dof();
        if let Some(limits) = &self.config.torque_limits_nm {
            if limits.len() != nj {
                return Err(WbcError::TorqueLimitDimension {
                    expected: nj,
                    provided: limits.len(),
                });
            }
        }
        if let Some(posture) = posture_task {
            if posture.desired_joint_positions.len() != nj {
                return Err(WbcError::PostureDimension {
                    expected: nj,
                    provided: posture.desired_joint_positions.len(),
                });
            }
        }
        if contacts.iter().any(|contact| !contact.is_valid()) {
            return Err(WbcError::InvalidInput("a contact point is invalid"));
        }

        let mass = mass_matrix(model, q)?;
        let bias = non_linear_effects(model, q, qd)?;
        let motions = link_motions(model, q, qd)?;
        let com = center_of_mass(model, q)?;
        let com_jacobian = com_jacobian(model, q)?;

        // Contact Jacobians, point bias accelerations, and cones.
        let mut contact_jacobians: Vec<Vec<Vec<f64>>> = Vec::with_capacity(contacts.len());
        let mut contact_bias: Vec<Vec3> = Vec::with_capacity(contacts.len());
        let mut cones: Vec<FrictionCone> = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let link_index = model
                .kinematic()
                .link_index(contact.link)
                .ok_or(WbcError::UnknownContactLink(contact.link))?;
            let jacobian =
                rne_dynamics::frame_jacobian(model, q, contact.link, contact.point_local_m)?;
            let mut rows = vec![vec![0.0; nv]; 3];
            for (row, values) in rows.iter_mut().enumerate() {
                for (column, value) in values.iter_mut().enumerate() {
                    *value = jacobian.get(row, column);
                }
            }
            contact_jacobians.push(rows);
            contact_bias
                .push(motions[link_index].point_bias_acceleration_m_s2(contact.point_local_m));
            cones.push(FrictionCone::from_contact(contact));
        }

        let cols = nv + 3 * contacts.len();
        let mut rows: Vec<Vec<f64>> = Vec::new();
        let mut rhs: Vec<f64> = Vec::new();

        // Floating-base equations of motion: M_base qdd - sum J_c^T f = -h_base.
        let dynamics_scale = self.config.dynamics_weight.sqrt();
        for row in 0..6 {
            let mut coefficients = vec![0.0; cols];
            for column in 0..nv {
                coefficients[column] = mass.get(row, column);
            }
            for (contact, jacobian) in contact_jacobians.iter().enumerate() {
                for component in 0..3 {
                    coefficients[nv + 3 * contact + component] = -jacobian[component][row];
                }
            }
            push_row(
                &mut rows,
                &mut rhs,
                coefficients,
                -bias[row],
                dynamics_scale,
            );
        }

        // Contact no-slip: J_c qdd = -bias_c.
        let contact_scale = self.config.contact_weight.sqrt();
        for (contact, jacobian) in contact_jacobians.iter().enumerate() {
            for component in 0..3 {
                let mut coefficients = vec![0.0; cols];
                coefficients[..nv].copy_from_slice(&jacobian[component]);
                let target = -contact_bias[contact].to_array()[component];
                push_row(&mut rows, &mut rhs, coefficients, target, contact_scale);
            }
        }

        // Center-of-mass task.
        if let Some(task) = com_task {
            let com_velocity = mat_vec(&com_jacobian, qd);
            let desired = task.desired_acceleration_m_s2
                + (task.desired_position_m - com) * task.position_gain_s_inv2
                + (task.desired_velocity_m_s - com_velocity) * task.velocity_gain_s_inv;
            let com_bias = com_bias_acceleration(model, &motions);
            let desired = desired.to_array();
            let com_bias = com_bias.to_array();
            let scale = self.config.com_weight.sqrt();
            for component in 0..3 {
                let mut coefficients = vec![0.0; cols];
                for column in 0..nv {
                    coefficients[column] = com_jacobian.get(component, column);
                }
                let target = desired[component] - com_bias[component];
                push_row(&mut rows, &mut rhs, coefficients, target, scale);
            }
        }

        // Joint posture task.
        if let Some(task) = posture_task {
            let scale = self.config.posture_weight.sqrt();
            for joint in 0..nj {
                let dof = model.base_dof() + joint;
                let target = task.position_gain_s_inv2
                    * (task.desired_joint_positions[joint] - q[dof])
                    - task.velocity_gain_s_inv * qd[dof];
                let mut coefficients = vec![0.0; cols];
                coefficients[dof] = 1.0;
                push_row(&mut rows, &mut rhs, coefficients, target, scale);
            }
        }

        // Tikhonov regularization.
        let force_scale = self.config.force_regularization.sqrt();
        for contact in 0..contacts.len() {
            for component in 0..3 {
                let mut coefficients = vec![0.0; cols];
                coefficients[nv + 3 * contact + component] = 1.0;
                push_row(&mut rows, &mut rhs, coefficients, 0.0, force_scale);
            }
        }
        let acceleration_scale = self.config.acceleration_regularization.sqrt();
        for column in 0..nv {
            let mut coefficients = vec![0.0; cols];
            coefficients[column] = 1.0;
            push_row(&mut rows, &mut rhs, coefficients, 0.0, acceleration_scale);
        }

        let solution = solve_least_squares(&rows, &rhs, cols, self.config.solver_regularization)
            .ok_or(WbcError::SingularSystem)?;
        let joint_acceleration = solution[..nv].to_vec();
        let mut contact_forces_world_n = Vec::with_capacity(contacts.len());
        for (contact, cone) in cones.iter().enumerate() {
            let force = Vec3::new(
                solution[nv + 3 * contact],
                solution[nv + 3 * contact + 1],
                solution[nv + 3 * contact + 2],
            );
            contact_forces_world_n.push(cone.project(force));
        }

        // Recover the generalized force S^T tau = M qdd + h - sum J_c^T f.
        let mut generalized: Vec<f64> = (0..nv)
            .map(|row| {
                (0..nv)
                    .map(|column| mass.get(row, column) * joint_acceleration[column])
                    .sum::<f64>()
                    + bias[row]
            })
            .collect();
        for (contact, jacobian) in contact_jacobians.iter().enumerate() {
            let force = contact_forces_world_n[contact];
            for row in 0..nv {
                generalized[row] -= jacobian[0][row] * force.x
                    + jacobian[1][row] * force.y
                    + jacobian[2][row] * force.z;
            }
        }
        let mut base_wrench_residual = [0.0; 6];
        base_wrench_residual.copy_from_slice(&generalized[..6]);
        let mut joint_torque_nm = generalized[model.base_dof()..].to_vec();
        let mut torque_saturated = false;
        if let Some(limits) = &self.config.torque_limits_nm {
            for (torque, limit) in joint_torque_nm.iter_mut().zip(limits) {
                let limited = torque.clamp(-limit, *limit);
                if (limited - *torque).abs() > 1.0e-12 {
                    torque_saturated = true;
                }
                *torque = limited;
            }
        }

        let mut contact_acceleration_residual_m_s2 = Vec::with_capacity(contacts.len());
        for (contact, jacobian) in contact_jacobians.iter().enumerate() {
            let mut squared = 0.0;
            for component in 0..3 {
                let acceleration = (0..nv)
                    .map(|column| jacobian[component][column] * joint_acceleration[column])
                    .sum::<f64>()
                    + contact_bias[contact].to_array()[component];
                squared += acceleration * acceleration;
            }
            contact_acceleration_residual_m_s2.push(squared.sqrt());
        }

        let com_acceleration_m_s2_tmp =
            mat_vec(&com_jacobian, &joint_acceleration) + com_bias_acceleration(model, &motions);

        Ok(WholeBodySolution {
            joint_acceleration,
            contact_forces_world_n,
            joint_torque_nm,
            base_wrench_residual,
            contact_acceleration_residual_m_s2,
            com_acceleration_m_s2: com_acceleration_m_s2_tmp,
            torque_saturated,
        })
    }
}

fn com_bias_acceleration(model: &ArticulatedModel, motions: &[rne_dynamics::LinkMotion]) -> Vec3 {
    let mut total_mass = 0.0;
    let mut weighted = Vec3::ZERO;
    for (index, motion) in motions.iter().enumerate() {
        let Some(inertia) = model.link_inertia(index) else {
            continue;
        };
        if inertia.mass_kg == 0.0 {
            continue;
        }
        weighted += motion.point_bias_acceleration_m_s2(inertia.center_of_mass_m) * inertia.mass_kg;
        total_mass += inertia.mass_kg;
    }
    if total_mass > 0.0 {
        weighted / total_mass
    } else {
        Vec3::ZERO
    }
}

fn mat_vec(jacobian: &DenseMatrix, vector: &[f64]) -> Vec3 {
    let mut out = Vec3::ZERO;
    for row in 0..3 {
        let value: f64 = (0..jacobian.cols())
            .map(|column| jacobian.get(row, column) * vector[column])
            .sum();
        match row {
            0 => out.x = value,
            1 => out.y = value,
            _ => out.z = value,
        }
    }
    out
}

fn push_row(
    rows: &mut Vec<Vec<f64>>,
    rhs: &mut Vec<f64>,
    coefficients: Vec<f64>,
    target: f64,
    scale: f64,
) {
    rows.push(
        coefficients
            .into_iter()
            .map(|value| value * scale)
            .collect(),
    );
    rhs.push(target * scale);
}

fn solve_least_squares(
    rows: &[Vec<f64>],
    rhs: &[f64],
    cols: usize,
    regularization: f64,
) -> Option<Vec<f64>> {
    // Column equilibration keeps the normal equations well-conditioned even when
    // the mass matrix mixes base inertias and tiny link inertias.
    let mut column_norm = vec![0.0; cols];
    for row in rows {
        for (column, value) in row.iter().enumerate() {
            column_norm[column] += value * value;
        }
    }
    let column_scale: Vec<f64> = column_norm
        .iter()
        .map(|value| {
            let norm = value.sqrt();
            if norm > 1.0e-12 {
                norm
            } else {
                1.0
            }
        })
        .collect();

    let mut normal = DenseMatrix::zeros(cols, cols);
    let mut normal_rhs = vec![0.0; cols];
    for (row, target) in rows.iter().zip(rhs) {
        let scaled: Vec<f64> = row
            .iter()
            .zip(&column_scale)
            .map(|(value, scale)| value * scale)
            .collect();
        for i in 0..cols {
            if scaled[i] == 0.0 {
                continue;
            }
            normal_rhs[i] += scaled[i] * target;
            for j in i..cols {
                normal.set(i, j, normal.get(i, j) + scaled[i] * scaled[j]);
            }
        }
    }
    for i in 0..cols {
        for j in 0..i {
            normal.set(i, j, normal.get(j, i));
        }
        normal.set(i, i, normal.get(i, i) + regularization);
    }
    let scaled_solution = normal.solve(&normal_rhs)?;
    Some(
        scaled_solution
            .iter()
            .zip(&column_scale)
            .map(|(value, scale)| value * scale)
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::{spawn_named, World};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Link, Robot, RobotId};
    use rne_world::Transform3;

    fn floating_body() -> (World, rne_dynamics::ArticulatedModel, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "body");
        let base = spawn_named(&mut world, "base");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "floating_body".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            FloatingBase,
            RigidBody {
                mass_kg: 3.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::ZERO,
                ixx_kg_m2: 0.1,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.1,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.1,
            },
        ));
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        (world, model, base)
    }

    #[test]
    fn standing_body_supports_its_weight() {
        let (_world, model, base) = floating_body();
        let contacts = vec![
            ContactPoint::new(base, Vec3::new(0.1, -0.2, 0.0), 0.8),
            ContactPoint::new(base, Vec3::new(-0.1, -0.2, 0.0), 0.8),
        ];
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        let solution = controller
            .solve(
                &model,
                &[0.0; 6],
                &[0.0; 6],
                &contacts,
                Some(&ComTask::hold(Vec3::ZERO)),
                None,
            )
            .expect("solve");

        let total_vertical: f64 = solution
            .contact_forces_world_n
            .iter()
            .map(|force| force.y)
            .sum();
        assert_relative_eq!(total_vertical, 3.0 * 9.81, epsilon = 0.5);
        assert!(solution
            .contact_forces_world_n
            .iter()
            .all(|force| force.y > 0.0));
        let residual = solution
            .base_wrench_residual
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        assert!(residual < 0.5, "base wrench residual {residual}");
        assert!(solution
            .contact_acceleration_residual_m_s2
            .iter()
            .all(|value| *value < 1.0e-3));
        assert!(solution.com_acceleration_m_s2.length() < 1.0e-2);
        assert!(!solution.torque_saturated);
    }

    #[test]
    fn solve_is_deterministic() {
        let (_world, model, base) = floating_body();
        let contacts = vec![ContactPoint::new(base, Vec3::new(0.1, -0.2, 0.0), 0.8)];
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        let first = controller
            .solve(
                &model,
                &[0.0; 6],
                &[0.0; 6],
                &contacts,
                Some(&ComTask::hold(Vec3::ZERO)),
                None,
            )
            .expect("solve");
        let second = controller
            .solve(
                &model,
                &[0.0; 6],
                &[0.0; 6],
                &contacts,
                Some(&ComTask::hold(Vec3::ZERO)),
                None,
            )
            .expect("solve");
        assert_eq!(first, second);
    }

    #[test]
    fn rejects_fixed_base_and_empty_contacts() {
        let (_world, model, base) = floating_body();
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        assert_eq!(
            controller.solve(&model, &[0.0; 6], &[0.0; 6], &[], None, None),
            Err(WbcError::NoContacts)
        );
        let contacts = vec![ContactPoint::new(base, Vec3::ZERO, 0.8)];
        let bad_limits = WholeBodyConfig {
            torque_limits_nm: Some(vec![1.0]),
            ..WholeBodyConfig::default()
        };
        let controller = WholeBodyController::new(bad_limits);
        assert!(matches!(
            controller.solve(&model, &[0.0; 6], &[0.0; 6], &contacts, None, None),
            Err(WbcError::TorqueLimitDimension { .. })
        ));
    }
}
