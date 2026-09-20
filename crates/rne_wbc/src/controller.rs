//! The weighted inverse-dynamics whole-body controller.

use crate::contact::{ContactPoint, FrictionCone};
use rne_dynamics::{
    center_of_mass, centroidal_momentum, centroidal_momentum_bias, centroidal_momentum_matrix,
    com_jacobian, link_motions, mass_matrix, non_linear_effects, ArticulatedModel, DenseMatrix,
    DynamicsError,
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
    ///
    /// Retained for API compatibility. An empty contact set is now valid and
    /// means the flight phase: the base is unactuated and only the joints and
    /// tasks act, so the solver no longer returns this error.
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
    /// Contact compliance in meters per newton: the no-slip row is relaxed to
    /// `J qdd + bias = compliance * f`, so a contact point accelerates slightly
    /// under load. A value of zero (the default) enforces rigid point contacts.
    /// A small positive value lets the solver match a soft/penalty plant such as
    /// Rapier's.
    pub contact_compliance: f64,
    /// Weight on the center-of-mass task.
    pub com_weight: f64,
    /// Weight on the posture task.
    pub posture_weight: f64,
    /// Weight on the base-attitude task.
    pub angular_weight: f64,
    /// Weight on an optional feed-forward joint torque reference.
    pub torque_reference_weight: f64,
    /// Tikhonov weight on the contact forces (pulls them toward zero).
    pub force_regularization: f64,
    /// Tikhonov weight on the joint accelerations (pulls them toward zero).
    pub acceleration_regularization: f64,
    /// Small diagonal added to the normal equations for numerical rank.
    pub solver_regularization: f64,
    /// Optional symmetric joint torque limit per actuated joint, in N·m.
    pub torque_limits_nm: Option<Vec<f64>>,
    /// When true, the torque limits are enforced *inside* the solve (a box on
    /// the joint torques) so the controller returns the best feasible
    /// compromise. When false (the default) the unconstrained solution is
    /// clipped afterward, preserving the historical behavior.
    pub enforce_torque_limits: bool,
}

impl Default for WholeBodyConfig {
    fn default() -> Self {
        Self {
            dynamics_weight: 1.0e6,
            contact_weight: 1.0e6,
            contact_compliance: 0.0,
            com_weight: 1.0e2,
            posture_weight: 1.0,
            angular_weight: 1.0e4,
            torque_reference_weight: 1.0e3,
            force_regularization: 1.0e-4,
            acceleration_regularization: 1.0e-4,
            solver_regularization: 1.0e-9,
            torque_limits_nm: None,
            enforce_torque_limits: false,
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
    /// Optional desired joint velocities, for an acceleration-level tracking law.
    pub desired_joint_velocities: Option<Vec<f64>>,
    /// Optional desired joint accelerations, for an acceleration-level tracking
    /// law. Supplying a reference trajectory's acceleration keeps the task in
    /// the solve's null space instead of letting it drift.
    pub desired_joint_accelerations: Option<Vec<f64>>,
    /// Position feedback gain in inverse seconds squared.
    pub position_gain_s_inv2: f64,
    /// Velocity feedback gain in inverse seconds.
    pub velocity_gain_s_inv: f64,
}

/// Base-orientation task commanding the floating-base angular acceleration.
///
/// The three rows set the base angular acceleration in the base body frame, so
/// a leveling controller can drive the body upright during a maneuver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseAttitudeTask {
    /// Desired base angular acceleration in the base body frame, in rad/s².
    pub desired_angular_acceleration_rad_s2: Vec3,
}

/// Whole-body angular-momentum task at the acceleration level.
///
/// The rows command the angular part of the centroidal momentum rate,
/// `Ldot = A(q) qdd + c(q, qd)`, so a flight-phase controller can build or
/// arrest a spin with no contacts. The linear part of the momentum is left
/// free.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentroidalMomentumTask {
    /// Desired angular momentum about the whole-body center of mass, in
    /// kg·m²/s.
    pub desired_angular_momentum_world_kg_m2_s: Vec3,
    /// Momentum feedback gain in inverse seconds.
    pub momentum_gain_s_inv: f64,
    /// Feed-forward angular momentum rate, in N·m.
    pub desired_angular_momentum_rate_world_nm: Vec3,
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
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::needless_range_loop)]
    pub fn solve(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[ContactPoint],
        com_task: Option<&ComTask>,
        base_attitude_task: Option<&BaseAttitudeTask>,
        posture_task: Option<&PostureTask>,
    ) -> Result<WholeBodySolution, WbcError> {
        self.solve_with_torque_reference(
            model,
            q,
            qd,
            contacts,
            com_task,
            base_attitude_task,
            posture_task,
            None,
        )
    }

    /// [`Self::solve`] with an optional per-joint feed-forward torque
    /// reference, such as a trajectory-plan torque.
    #[allow(clippy::too_many_arguments)]
    pub fn solve_with_torque_reference(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[ContactPoint],
        com_task: Option<&ComTask>,
        base_attitude_task: Option<&BaseAttitudeTask>,
        posture_task: Option<&PostureTask>,
        torque_reference_nm: Option<&[f64]>,
    ) -> Result<WholeBodySolution, WbcError> {
        self.solve_inner(
            model,
            q,
            qd,
            contacts,
            com_task,
            base_attitude_task,
            posture_task,
            torque_reference_nm,
            None,
        )
    }

    /// [`Self::solve`] with a whole-body angular-momentum task.
    ///
    /// The momentum task lets a flight-phase controller shape the spin without
    /// contacts, using [`rne_dynamics::centroidal_momentum`].
    #[allow(clippy::too_many_arguments)]
    pub fn solve_with_centroidal_momentum(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[ContactPoint],
        com_task: Option<&ComTask>,
        base_attitude_task: Option<&BaseAttitudeTask>,
        posture_task: Option<&PostureTask>,
        momentum_task: Option<&CentroidalMomentumTask>,
    ) -> Result<WholeBodySolution, WbcError> {
        self.solve_inner(
            model,
            q,
            qd,
            contacts,
            com_task,
            base_attitude_task,
            posture_task,
            None,
            momentum_task,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::needless_range_loop)]
    fn solve_inner(
        &self,
        model: &ArticulatedModel,
        q: &[f64],
        qd: &[f64],
        contacts: &[ContactPoint],
        com_task: Option<&ComTask>,
        base_attitude_task: Option<&BaseAttitudeTask>,
        posture_task: Option<&PostureTask>,
        torque_reference_nm: Option<&[f64]>,
        momentum_task: Option<&CentroidalMomentumTask>,
    ) -> Result<WholeBodySolution, WbcError> {
        if model.base_dof() != 6 {
            return Err(WbcError::RequiresFloatingBase);
        }
        // Empty contacts are allowed: this is the flight phase, where the base
        // is unactuated and only the actuated joints and any tasks act. The
        // contact rows are simply absent.
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
                // Soft contacts: `J qdd + bias = compliance * f`.
                coefficients[nv + 3 * contact + component] = -self.config.contact_compliance;
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

        // Base attitude task: command the base angular acceleration.
        if let Some(task) = base_attitude_task {
            let scale = self.config.angular_weight.sqrt();
            let desired = task.desired_angular_acceleration_rad_s2.to_array();
            for component in 0..3 {
                let mut coefficients = vec![0.0; cols];
                coefficients[3 + component] = 1.0;
                push_row(&mut rows, &mut rhs, coefficients, desired[component], scale);
            }
        }

        // Whole-body angular-momentum task: `Ldot = A qdd + c`.
        if let Some(task) = momentum_task {
            let momentum_matrix = centroidal_momentum_matrix(model, q)?;
            let momentum_bias = centroidal_momentum_bias(model, q, qd)?;
            let momentum = centroidal_momentum(model, q, qd)?;
            let scale = self.config.angular_weight.sqrt();
            for component in 0..3 {
                let row = 3 + component;
                let desired = task.desired_angular_momentum_rate_world_nm.to_array()[component]
                    + task.momentum_gain_s_inv
                        * (task.desired_angular_momentum_world_kg_m2_s.to_array()[component]
                            - momentum[row]);
                let mut coefficients = vec![0.0; cols];
                for column in 0..nv {
                    coefficients[column] = momentum_matrix.get(row, column);
                }
                push_row(
                    &mut rows,
                    &mut rhs,
                    coefficients,
                    desired - momentum_bias[row],
                    scale,
                );
            }
        }

        // Joint posture task.
        if let Some(task) = posture_task {
            let scale = self.config.posture_weight.sqrt();
            for joint in 0..nj {
                let dof = model.base_dof() + joint;
                let acceleration = task
                    .desired_joint_accelerations
                    .as_ref()
                    .map(|values| values[joint])
                    .unwrap_or(0.0);
                let velocity_target = task
                    .desired_joint_velocities
                    .as_ref()
                    .map(|values| values[joint])
                    .unwrap_or(0.0);
                let target = acceleration
                    + task.position_gain_s_inv2 * (task.desired_joint_positions[joint] - q[dof])
                    + task.velocity_gain_s_inv * (velocity_target - qd[dof]);
                let mut coefficients = vec![0.0; cols];
                coefficients[dof] = 1.0;
                push_row(&mut rows, &mut rhs, coefficients, target, scale);
            }
        }

        // Feed-forward joint torque reference, for example a trajectory-plan
        // torque. `tau_j = (M qdd + h - J^T f)_j`, so the row is
        // `M_j qdd - J^T_j f = reference_j - h_j`.
        if let Some(reference) = torque_reference_nm {
            assert_eq!(reference.len(), nj);
            let scale = self.config.torque_reference_weight.sqrt();
            for joint in 0..nj {
                let row = model.base_dof() + joint;
                let mut coefficients = vec![0.0; cols];
                for column in 0..nv {
                    coefficients[column] = mass.get(row, column);
                }
                for (contact, jacobian) in contact_jacobians.iter().enumerate() {
                    for component in 0..3 {
                        coefficients[nv + 3 * contact + component] -= jacobian[component][row];
                    }
                }
                push_row(
                    &mut rows,
                    &mut rhs,
                    coefficients,
                    reference[joint] - bias[row],
                    scale,
                );
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

        // Solve. With actuated joint torque limits the torques join the
        // unknowns and are box-constrained inside the solve, so the solver
        // returns the best *feasible* compromise instead of clipping the
        // unconstrained torques afterward (which distorts the whole motion).
        let nj = nv - model.base_dof();
        let (solution, torque_from_solve) = match &self.config.torque_limits_nm {
            Some(limits) if self.config.enforce_torque_limits && limits.len() == nj => {
                let cols_total = cols + nj;
                let mut extended: Vec<Vec<f64>> = rows
                    .iter()
                    .map(|row| {
                        let mut padded = row.clone();
                        padded.resize(cols_total, 0.0);
                        padded
                    })
                    .collect();
                let mut extended_rhs = rhs.clone();
                // tau_j = (M qdd + h - J^T f)_j  ->  M_j qdd - J^T_j f - tau_j = -h_j
                // Weighted like the dynamics rows so the box actually bounds the
                // joint torques instead of a decoupled slack variable.
                let tau_scale = self.config.dynamics_weight.sqrt();
                for j in 0..nj {
                    let r = model.base_dof() + j;
                    let mut row = vec![0.0; cols_total];
                    for column in 0..nv {
                        row[column] = mass.get(r, column) * tau_scale;
                    }
                    for (contact, jacobian) in contact_jacobians.iter().enumerate() {
                        for component in 0..3 {
                            row[nv + 3 * contact + component] -= jacobian[component][r] * tau_scale;
                        }
                    }
                    row[cols + j] = -tau_scale;
                    extended.push(row);
                    extended_rhs.push(-bias[r] * tau_scale);
                }
                let solution = solve_box_least_squares(
                    &extended,
                    &extended_rhs,
                    cols_total,
                    cols,
                    limits,
                    self.config.solver_regularization,
                )
                .ok_or(WbcError::SingularSystem)?;
                (solution, true)
            }
            _ => (
                solve_least_squares(&rows, &rhs, cols, self.config.solver_regularization)
                    .ok_or(WbcError::SingularSystem)?,
                false,
            ),
        };
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
        let mut joint_torque_nm = if torque_from_solve {
            solution[cols..cols + nj].to_vec()
        } else {
            generalized[model.base_dof()..].to_vec()
        };
        let mut torque_saturated = false;
        if let Some(limits) = &self.config.torque_limits_nm {
            for (torque, limit) in joint_torque_nm.iter_mut().zip(limits) {
                if !torque_from_solve {
                    let limited = torque.clamp(-limit, *limit);
                    if (limited - *torque).abs() > 1.0e-12 {
                        torque_saturated = true;
                    }
                    *torque = limited;
                } else if torque.abs() >= *limit - 1.0e-9 {
                    torque_saturated = true;
                }
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

/// Box-constrained linear least squares solved by projected gradient.
///
/// Minimizes `||A x - b||^2 + regularization * ||x||^2` subject to
/// `-limits[i] <= x[box_start + i] <= limits[i]`. The projection just clamps the
/// constrained coordinates, and the step size is set from a power-iteration
/// estimate of the Lipschitz constant, so the method is deterministic and needs
/// no external solver.
#[allow(clippy::needless_range_loop)]
fn solve_box_least_squares(
    rows: &[Vec<f64>],
    rhs: &[f64],
    cols: usize,
    box_start: usize,
    limits: &[f64],
    regularization: f64,
) -> Option<Vec<f64>> {
    // Coordinate-descent sweeps. Deterministic and fixed so replays match.
    const ITERATIONS: usize = 400;
    // Column equilibration: divide each column by its norm so the scaled system
    // has unit-norm columns, which is what makes the coordinate descent below
    // converge quickly. (`A' = A D` with `D = diag(1 / norm)`, so the recovered
    // variables are `x = D x'`.)
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

    // Normal equations H = A'^T A' + reg I and gradient g = A'^T b (in scaled x).
    let mut h = vec![vec![0.0; cols]; cols];
    let mut g = vec![0.0; cols];
    for (row, target) in rows.iter().zip(rhs) {
        let scaled: Vec<f64> = row
            .iter()
            .zip(&column_scale)
            .map(|(value, scale)| value / scale)
            .collect();
        for i in 0..cols {
            if scaled[i] == 0.0 {
                continue;
            }
            g[i] += scaled[i] * target;
            for j in i..cols {
                h[i][j] += scaled[i] * scaled[j];
            }
        }
    }
    for i in 0..cols {
        for j in 0..i {
            h[i][j] = h[j][i];
        }
        h[i][i] += regularization;
    }

    // Scaled-space bounds for the boxed coordinates.
    let bounds: Vec<f64> = (0..limits.len())
        .map(|i| limits[i].abs().max(0.0) * column_scale[box_start + i])
        .collect();

    // Projected coordinate descent (projected Gauss-Seidel) on the SPD normal
    // equations `H x = g`. Each coordinate takes its exact one-dimensional
    // minimizer
    //
    //     x_i <- (g_i - sum_{j != i} H_ij x_j) / H_ii,  clipped to its bound,
    //
    // which converges monotonically for a symmetric positive-definite `H`. A
    // plain steepest-descent step with `1 / lambda_max` stalls on the
    // ill-conditioned systems a whole-body solve produces (the joint-torque
    // variables in particular), which is how `tau` used to come back at zero.
    let mut x = vec![0.0; cols];
    for _ in 0..ITERATIONS {
        for i in 0..cols {
            let mut residual = g[i];
            for (j, value) in x.iter().enumerate() {
                if j != i {
                    residual -= h[i][j] * value;
                }
            }
            let diagonal = h[i][i];
            if diagonal.abs() <= 1.0e-18 {
                continue;
            }
            let mut value = residual / diagonal;
            if i >= box_start {
                let bound = bounds[i - box_start];
                value = value.clamp(-bound, bound);
            }
            x[i] = value;
        }
    }

    Some(
        x.iter()
            .zip(&column_scale)
            .map(|(value, scale)| value / scale)
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::{spawn_named, World};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};
    use rne_world::Transform3;

    #[test]
    fn box_solver_converges_across_a_wide_scale_spread() {
        // Columns separated by three orders of magnitude, like the joint-torque
        // variables next to the acceleration variables. The old steepest-descent
        // step stalled here and left the boxed coordinate at zero.
        let rows = vec![vec![1.0e-3, 0.0], vec![0.0, 1.0], vec![1.0e-3, 1.0]];
        let rhs = vec![2.0e-3, 3.0, 4.0];
        let solution = solve_box_least_squares(&rows, &rhs, 2, 1, &[1.0], 1.0e-12).unwrap();
        // The box sits on x[1]: the unconstrained optimum there is far above 1,
        // so it clips to 1, and x[0] must then reach its own clipped optimum.
        assert_relative_eq!(solution[1], 1.0, epsilon = 1.0e-9);
        assert_relative_eq!(solution[0], 1501.0, epsilon = 1.0e-3);
        let residual = |x: &[f64]| {
            rows.iter()
                .zip(&rhs)
                .map(|(row, target)| (row[0] * x[0] + row[1] * x[1] - target).powi(2))
                .sum::<f64>()
        };
        assert!(
            residual(&solution) < 0.5 * residual(&[0.0, 0.0]),
            "residual {}",
            residual(&solution)
        );
    }

    #[test]
    fn box_solver_respects_and_releases_the_box() {
        // Independent rows: unconstrained x = (3, 10); the box on x[1] clips it.
        let rows = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let rhs = vec![3.0, 10.0];
        let clipped = solve_box_least_squares(&rows, &rhs, 2, 1, &[2.0], 1.0e-9).unwrap();
        assert_relative_eq!(clipped[0], 3.0, epsilon = 1.0e-6);
        assert_relative_eq!(clipped[1], 2.0, epsilon = 1.0e-6);

        let released = solve_box_least_squares(&rows, &rhs, 2, 1, &[100.0], 1.0e-9).unwrap();
        assert_relative_eq!(released[0], 3.0, epsilon = 1.0e-6);
        assert_relative_eq!(released[1], 10.0, epsilon = 1.0e-6);
    }

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

    fn floating_arm() -> (World, rne_dynamics::ArticulatedModel, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "arm");
        let base = spawn_named(&mut world, "base");
        let link = spawn_named(&mut world, "link");
        let joint = spawn_named(&mut world, "joint");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "floating_arm".into(),
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
        world.entity_mut(link).insert((
            Link {
                robot,
                name: "link".into(),
            },
            Transform3::from_translation_rotation(
                Vec3::new(0.3, 0.0, 0.0),
                rne_math::Quat::IDENTITY,
            ),
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::new(0.2, 0.0, 0.0),
                ixx_kg_m2: 0.02,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.02,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.02,
            },
        ));
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: link,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        (world, model, base)
    }

    #[test]
    fn flight_phase_tracks_posture_with_a_free_base() {
        let (_world, model, _base) = floating_arm();
        assert_eq!(model.nv(), 7);
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        let posture = PostureTask {
            desired_joint_positions: vec![0.5],
            desired_joint_velocities: Some(vec![0.0]),
            desired_joint_accelerations: None,
            position_gain_s_inv2: 25.0,
            velocity_gain_s_inv: 10.0,
        };
        let solution = controller
            .solve(
                &model,
                &[0.0; 7],
                &[0.0; 7],
                &[],
                None,
                None,
                Some(&posture),
            )
            .expect("flight solve");
        // With no contacts the base is unactuated and only gravity acts on it,
        // but the joint is driven toward the posture target.
        assert!(solution.contact_forces_world_n.is_empty());
        assert!(solution
            .joint_acceleration
            .iter()
            .all(|value| value.is_finite()));
        assert!(
            solution.joint_acceleration[6] > 0.0,
            "joint did not accelerate toward the target: {}",
            solution.joint_acceleration[6]
        );
    }

    #[test]
    fn momentum_task_tracks_a_commanded_angular_rate() {
        // Ground reaction is needed to change the angular momentum, so the task
        // is exercised with contacts, as in a push or a landing.
        let (_world, model, base) = floating_body();
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        let q = [0.0; 6];
        let qd = [0.0; 6];
        let contacts = vec![
            ContactPoint::new(base, Vec3::new(0.1, -0.2, 0.0), 0.8),
            ContactPoint::new(base, Vec3::new(-0.1, -0.2, 0.0), 0.8),
        ];
        let desired = Vec3::new(0.0, 0.0, 1.0);
        let task = CentroidalMomentumTask {
            desired_angular_momentum_world_kg_m2_s: Vec3::ZERO,
            momentum_gain_s_inv: 0.0,
            desired_angular_momentum_rate_world_nm: desired,
        };
        let without = controller
            .solve_with_centroidal_momentum(&model, &q, &qd, &contacts, None, None, None, None)
            .expect("solve");
        let with = controller
            .solve_with_centroidal_momentum(
                &model,
                &q,
                &qd,
                &contacts,
                None,
                None,
                None,
                Some(&task),
            )
            .expect("solve");

        let matrix = centroidal_momentum_matrix(&model, &q).expect("matrix");
        let bias = centroidal_momentum_bias(&model, &q, &qd).expect("bias");
        let angular_rate = |solution: &WholeBodySolution| {
            let rate = matrix.mul_vec(&solution.joint_acceleration);
            Vec3::new(rate[3] + bias[3], rate[4] + bias[4], rate[5] + bias[5])
        };
        let without_error = (angular_rate(&without) - desired).length();
        let with_error = (angular_rate(&with) - desired).length();
        assert!(
            with_error < without_error,
            "momentum task did not approach the target: {with_error} vs {without_error}"
        );
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
                None,
            )
            .expect("solve");
        assert_eq!(first, second);
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn solution_matches_constrained_forward_dynamics_with_base_velocity() {
        let (_world, model, base) = floating_body();
        let contacts = vec![
            ContactPoint::new(base, Vec3::new(0.1, -0.2, 0.1), 0.8),
            ContactPoint::new(base, Vec3::new(-0.1, -0.2, 0.1), 0.8),
            ContactPoint::new(base, Vec3::new(0.0, -0.2, -0.1), 0.8),
        ];
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        // A nonzero base twist exercises the Coriolis/bias terms.
        let q = [0.05, -0.02, 0.03, 0.01, -0.02, 0.015];
        let qd = [0.1, -0.05, 0.02, 0.03, -0.02, 0.04];
        // No task competes with the dynamics/contact rows, so the weighted least
        // squares must reproduce the exact constrained KKT solution.
        let solution = controller
            .solve(&model, &q, &qd, &contacts, None, None, None)
            .expect("solve");

        let specs: Vec<rne_dynamics::ContactSpec> = contacts
            .iter()
            .map(|contact| rne_dynamics::ContactSpec {
                link: contact.link,
                point_local_m: contact.point_local_m,
            })
            .collect();
        let (qdd, _forces) = rne_dynamics::constrained_forward_dynamics(
            &model,
            &q,
            &qd,
            &vec![0.0; model.nv()],
            &specs,
        )
        .expect("constrained dynamics");

        // The contact constraints pin the acceleration, so the WBC solve must
        // reproduce the constrained-dynamics acceleration even with a nonzero
        // base twist. (The individual contact forces are not unique for an
        // over-constrained point set, so they are not compared here.)
        for index in 0..model.nv() {
            assert_relative_eq!(
                solution.joint_acceleration[index],
                qdd[index],
                epsilon = 1.0e-4,
                max_relative = 1.0e-4
            );
        }
    }

    #[test]
    fn contact_compliance_relaxes_the_no_slip_constraint() {
        let (_world, model, base) = floating_body();
        let contacts = vec![
            ContactPoint::new(base, Vec3::new(0.1, -0.2, 0.0), 0.8),
            ContactPoint::new(base, Vec3::new(-0.1, -0.2, 0.0), 0.8),
        ];
        let solve = |compliance: f64| {
            WholeBodyController::new(WholeBodyConfig {
                contact_compliance: compliance,
                ..WholeBodyConfig::default()
            })
            .solve(
                &model,
                &[0.0; 6],
                &[0.0; 6],
                &contacts,
                Some(&ComTask::hold(Vec3::ZERO)),
                None,
                None,
            )
            .expect("solve")
        };
        let rigid = solve(0.0);
        let soft = solve(1.0e-4);

        let worst = |solution: &WholeBodySolution| {
            solution
                .contact_acceleration_residual_m_s2
                .iter()
                .fold(0.0_f64, |value, residual| value.max(residual.abs()))
        };
        assert!(worst(&rigid) < 1.0e-3, "rigid residual {}", worst(&rigid));
        assert!(
            worst(&soft) > 10.0 * worst(&rigid).max(1.0e-9),
            "compliance should allow a contact acceleration: rigid {} soft {}",
            worst(&rigid),
            worst(&soft)
        );
        let total = |solution: &WholeBodySolution| {
            solution
                .contact_forces_world_n
                .iter()
                .map(|force| force.y)
                .sum::<f64>()
        };
        assert_relative_eq!(total(&rigid), total(&soft), epsilon = 1.0);
    }

    #[test]
    fn handles_empty_contacts_and_rejects_bad_limits() {
        let (_world, model, base) = floating_body();
        let controller = WholeBodyController::new(WholeBodyConfig::default());
        // Flight phase: no contacts is valid and yields a finite free-base
        // solution rather than an error.
        let solution = controller
            .solve(&model, &[0.0; 6], &[0.0; 6], &[], None, None, None)
            .expect("flight-phase solve");
        assert!(solution.contact_forces_world_n.is_empty());
        assert!(solution
            .joint_acceleration
            .iter()
            .all(|value| value.is_finite()));
        let contacts = vec![ContactPoint::new(base, Vec3::ZERO, 0.8)];
        let bad_limits = WholeBodyConfig {
            torque_limits_nm: Some(vec![1.0]),
            ..WholeBodyConfig::default()
        };
        let controller = WholeBodyController::new(bad_limits);
        assert!(matches!(
            controller.solve(&model, &[0.0; 6], &[0.0; 6], &contacts, None, None, None),
            Err(WbcError::TorqueLimitDimension { .. })
        ));
    }
}
