//! Compliant contact-implicit articulated dynamics.
//!
//! [`ContactImplicitArticulatedDynamics`] keeps the set of candidate contact
//! points fixed but decides at every step which of them actually push, from the
//! point penetrations. A point above the ground contributes no force and one
//! that has penetrated pushes back with a smooth normal force and a Coulomb
//! friction force. An optimizer built on this model therefore discovers its own
//! contact schedule instead of being handed one, which is the point of a
//! contact-implicit formulation.

use crate::ddp::{DiscreteDynamics, DynamicsDerivatives, OcError};
use rne_dynamics::{
    frame_jacobian, integrate_configuration, link_motions, mass_matrix, mass_matrix_gradient,
    non_linear_effects, non_linear_effects_gradient, ArticulatedModel, ContactSpec,
};
use rne_math::Vec3;

/// Tangential-speed regularization for the smooth Coulomb friction force.
const FRICTION_REGULARIZATION_M_S: f64 = 1.0e-3;

/// A compliant, penalty-based normal contact law on a ground plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompliantContactModel {
    /// Ground plane height along the world up axis, in meters.
    pub ground_height_m: f64,
    /// Normal stiffness in newtons per meter of penetration.
    pub stiffness_n_m: f64,
    /// Normal damping in newton-seconds per meter of approach speed.
    pub damping_n_s_m: f64,
    /// Penetration exponent (1 = linear, 2 = quadratic).
    pub penetration_exponent: f64,
    /// Coulomb friction coefficient.
    pub friction: f64,
    /// Normal force clamp in newtons, for numerical stability.
    pub max_normal_force_n: f64,
}

impl Default for CompliantContactModel {
    fn default() -> Self {
        Self {
            ground_height_m: 0.0,
            stiffness_n_m: 1.0e4,
            damping_n_s_m: 1.0e2,
            penetration_exponent: 2.0,
            friction: 0.8,
            max_normal_force_n: 1.0e5,
        }
    }
}

impl CompliantContactModel {
    /// Normal force in newtons for a penetration depth and approach speed.
    ///
    /// Returns zero when the point is not penetrating; the force is clamped to
    /// `max_normal_force_n`.
    pub fn normal_force_n(&self, penetration_m: f64, approach_speed_m_s: f64) -> f64 {
        if penetration_m <= 0.0 {
            return 0.0;
        }
        let elastic = self.stiffness_n_m * penetration_m.powf(self.penetration_exponent);
        let damping = self.damping_n_s_m * approach_speed_m_s.max(0.0);
        (elastic + damping).clamp(0.0, self.max_normal_force_n)
    }
}

/// Compliant contact-implicit dynamics over a fixed set of candidate points.
#[derive(Clone, Debug)]
pub struct ContactImplicitArticulatedDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
    /// Candidate contact points; only those that penetrate push back.
    pub contacts: Vec<ContactSpec>,
    /// Compliant normal/friction law.
    pub contact_model: CompliantContactModel,
    /// Number of explicit sub-steps per call to [`step_state`](Self::step_state).
    ///
    /// The compliant contact law is stiff, so a single explicit step is only
    /// stable for a soft stiffness. Splitting the step lets a physically stiff
    /// law stay stable without changing the outer time step.
    pub substeps: usize,
}

impl<'a> ContactImplicitArticulatedDynamics<'a> {
    /// Creates a contact-implicit dynamics model.
    pub fn new(
        model: &'a ArticulatedModel,
        step_time_s: f64,
        contacts: Vec<ContactSpec>,
        contact_model: CompliantContactModel,
    ) -> Self {
        Self {
            model,
            step_time_s,
            contacts,
            contact_model,
            substeps: 1,
        }
    }

    /// Sets the number of internal explicit sub-steps (at least one).
    #[must_use]
    pub fn with_substeps(mut self, substeps: usize) -> Self {
        self.substeps = substeps.max(1);
        self
    }

    fn generalized_contact_force(
        &self,
        q: &[f64],
        qd: &[f64],
    ) -> Result<Vec<f64>, rne_dynamics::DynamicsError> {
        let nv = self.model.nv();
        let mut generalized = vec![0.0; nv];
        let motions = link_motions(self.model, q, qd)?;
        for spec in &self.contacts {
            let Some(link_index) = self.model.kinematic().link_index(spec.link) else {
                continue;
            };
            let motion = motions[link_index];
            let offset = motion.world_transform.rotation * spec.point_local_m;
            let point_world = motion.world_transform.translation + offset;
            let penetration = self.contact_model.ground_height_m - point_world.y;
            let jacobian = frame_jacobian(self.model, q, spec.link, spec.point_local_m)?;
            let point_velocity = Vec3::new(
                (0..nv).map(|col| jacobian.get(0, col) * qd[col]).sum(),
                (0..nv).map(|col| jacobian.get(1, col) * qd[col]).sum(),
                (0..nv).map(|col| jacobian.get(2, col) * qd[col]).sum(),
            );
            let normal_force = self
                .contact_model
                .normal_force_n(penetration, -point_velocity.y);
            if normal_force <= 0.0 {
                continue;
            }
            let tangential = point_velocity - Vec3::Y * point_velocity.y;
            let tangential_speed = tangential.length();
            let friction_direction = -tangential
                / (tangential_speed * tangential_speed
                    + FRICTION_REGULARIZATION_M_S * FRICTION_REGULARIZATION_M_S)
                    .sqrt();
            let friction_force = friction_direction * (self.contact_model.friction * normal_force);
            let force_world = Vec3::Y * normal_force + friction_force;
            for (dof, accumulated) in generalized.iter_mut().enumerate() {
                *accumulated += jacobian.get(0, dof) * force_world.x
                    + jacobian.get(1, dof) * force_world.y
                    + jacobian.get(2, dof) * force_world.z;
            }
        }
        Ok(generalized)
    }

    fn step_state(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        let nv = self.model.nv();
        let base = self.model.base_dof();
        if state.len() != 2 * nv || control.len() != nv - base {
            return Err(OcError::Dimension(
                "contact-implicit articulated state or control",
            ));
        }
        let mut q = state[..nv].to_vec();
        let mut qd = state[nv..].to_vec();
        let mut generalized = vec![0.0; nv];
        generalized[base..].copy_from_slice(control);
        let dt = self.step_time_s / self.substeps.max(1) as f64;
        for _ in 0..self.substeps.max(1) {
            let bias =
                non_linear_effects(self.model, &q, &qd).map_err(|_| OcError::Dynamics)?;
            let contact = self
                .generalized_contact_force(&q, &qd)
                .map_err(|_| OcError::Dynamics)?;
            let mut forces = generalized.clone();
            for index in 0..nv {
                forces[index] -= bias[index];
                forces[index] += contact[index];
            }
            let mass = mass_matrix(self.model, &q).map_err(|_| OcError::Dynamics)?;
            let acceleration = mass.solve(&forces).ok_or(OcError::Dynamics)?;
            // Integrate the configuration through the floating-base chart, not
            // with the raw twist, so a rotated base does not drift its world
            // position.
            q = integrate_configuration(self.model, &q, &qd, &acceleration, dt)
                .map_err(|_| OcError::Dynamics)?;
            for index in 0..nv {
                qd[index] += acceleration[index] * dt;
            }
        }
        let mut next = vec![0.0; 2 * nv];
        next[..nv].copy_from_slice(&q);
        next[nv..].copy_from_slice(&qd);
        Ok(next)
    }

    /// Semi-analytical `(df/dx, df/du)` for a single step.
    ///
    /// The smooth forward dynamics use the analytic gradient; the compliant
    /// contact force is differentiated by central differences of the force
    /// alone (one `frame_jacobian` per candidate point), which is far cheaper
    /// than differencing the whole step. The floating-base chart Jacobian of the
    /// configuration step is a small central difference of that map, as in
    /// `ArticulatedDynamics`. Returns `None` when sub-stepping is enabled, since
    /// the composed step Jacobian is not assembled here.
    #[allow(clippy::needless_range_loop)]
    fn analytic_derivatives_inner(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Result<DynamicsDerivatives, OcError> {
        let nv = self.model.nv();
        let base = self.model.base_dof();
        let nx = 2 * nv;
        let nu = nv - base;
        if state.len() != nx || control.len() != nu {
            return Err(OcError::Dimension(
                "contact-implicit articulated state or control",
            ));
        }
        let (q, qd) = state.split_at(nv);
        let mut torque = vec![0.0; nv];
        torque[base..].copy_from_slice(control);

        let mass = mass_matrix(self.model, q).map_err(|_| OcError::Dynamics)?;
        let bias = non_linear_effects(self.model, q, qd).map_err(|_| OcError::Dynamics)?;
        let contact = self
            .generalized_contact_force(q, qd)
            .map_err(|_| OcError::Dynamics)?;
        let mut rhs = torque.clone();
        for index in 0..nv {
            rhs[index] += contact[index] - bias[index];
        }
        let acceleration = mass.solve(&rhs).ok_or(OcError::Dynamics)?;

        let mass_gradient = mass_matrix_gradient(self.model, q).map_err(|_| OcError::Dynamics)?;
        let bias_gradient =
            non_linear_effects_gradient(self.model, q, qd).map_err(|_| OcError::Dynamics)?;

        let epsilon = 1.0e-6;
        let mut contact_q = vec![vec![0.0; nv]; nv];
        let mut contact_qd = vec![vec![0.0; nv]; nv];
        for k in 0..nv {
            let mut plus = q.to_vec();
            plus[k] += epsilon;
            let mut minus = q.to_vec();
            minus[k] -= epsilon;
            let force_plus = self
                .generalized_contact_force(&plus, qd)
                .map_err(|_| OcError::Dynamics)?;
            let force_minus = self
                .generalized_contact_force(&minus, qd)
                .map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                contact_q[k][row] = (force_plus[row] - force_minus[row]) / (2.0 * epsilon);
            }

            let mut plus = qd.to_vec();
            plus[k] += epsilon;
            let mut minus = qd.to_vec();
            minus[k] -= epsilon;
            let force_plus = self
                .generalized_contact_force(q, &plus)
                .map_err(|_| OcError::Dynamics)?;
            let force_minus = self
                .generalized_contact_force(q, &minus)
                .map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                contact_qd[k][row] = (force_plus[row] - force_minus[row]) / (2.0 * epsilon);
            }
        }

        let mut dqdd_dq = vec![vec![0.0; nv]; nv];
        let mut dqdd_dqd = vec![vec![0.0; nv]; nv];
        let mut dqdd_dtau = vec![vec![0.0; nv]; nv];
        for k in 0..nv {
            let mut derivative = vec![0.0; nv];
            for row in 0..nv {
                let mut product = 0.0;
                for column in 0..nv {
                    product += mass_gradient[k].get(row, column) * acceleration[column];
                }
                derivative[row] =
                    -bias_gradient.with_respect_to_q[k][row] + contact_q[k][row] - product;
            }
            let column = mass.solve(&derivative).ok_or(OcError::Dynamics)?;
            dqdd_dq[k].copy_from_slice(&column[..nv]);

            let derivative_qd: Vec<f64> = (0..nv)
                .map(|row| -bias_gradient.with_respect_to_qd[k][row] + contact_qd[k][row])
                .collect();
            let column = mass.solve(&derivative_qd).ok_or(OcError::Dynamics)?;
            dqdd_dqd[k].copy_from_slice(&column[..nv]);
        }
        for j in 0..nv {
            let mut basis = vec![0.0; nv];
            basis[j] = 1.0;
            let column = mass.solve(&basis).ok_or(OcError::Dynamics)?;
            dqdd_dtau[j].copy_from_slice(&column[..nv]);
        }

        let dt = self.step_time_s;
        let integrate_at = |q_value: &[f64], qd_value: &[f64], qdd_value: &[f64]| {
            integrate_configuration(self.model, q_value, qd_value, qdd_value, dt)
        };
        let mut fx = vec![vec![0.0; nx]; nx];
        let mut fu = vec![vec![0.0; nu]; nx];
        for k in 0..nv {
            let mut q_shift = q.to_vec();
            q_shift[k] += epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] += dqdd_dq[k][row] * epsilon;
            }
            let plus = integrate_at(&q_shift, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut q_shift = q.to_vec();
            q_shift[k] -= epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= dqdd_dq[k][row] * epsilon;
            }
            let minus = integrate_at(&q_shift, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                fx[row][k] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }
        for k in 0..nv {
            let mut qd_shift = qd.to_vec();
            qd_shift[k] += epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] += dqdd_dqd[k][row] * epsilon;
            }
            let plus = integrate_at(q, &qd_shift, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut qd_shift = qd.to_vec();
            qd_shift[k] -= epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= dqdd_dqd[k][row] * epsilon;
            }
            let minus = integrate_at(q, &qd_shift, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                fx[row][nv + k] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }
        for j in 0..nu {
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] += dqdd_dtau[base + j][row] * epsilon;
            }
            let plus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= dqdd_dtau[base + j][row] * epsilon;
            }
            let minus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                fu[row][j] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }
        for k in 0..nv {
            for row in 0..nv {
                fx[nv + row][k] = dqdd_dq[k][row] * dt;
                fx[nv + row][nv + k] =
                    if row == k { 1.0 } else { 0.0 } + dqdd_dqd[k][row] * dt;
            }
        }
        for j in 0..nu {
            for row in 0..nv {
                fu[nv + row][j] = dqdd_dtau[base + j][row] * dt;
            }
        }
        Ok(DynamicsDerivatives { fx, fu })
    }
}

impl DiscreteDynamics for ContactImplicitArticulatedDynamics<'_> {
    fn state_dim(&self) -> usize {
        2 * self.model.nv()
    }

    fn control_dim(&self) -> usize {
        self.model.nv() - self.model.base_dof()
    }

    fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        self.step_state(state, control)
    }

    fn analytic_derivatives(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Option<Result<DynamicsDerivatives, OcError>> {
        if self.substeps > 1 {
            return None;
        }
        Some(self.analytic_derivatives_inner(state, control))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_dynamics::ArticulatedModel;
    use rne_ecs::{spawn_named, World};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Link, Robot, RobotId};
    use rne_world::Transform3;

    fn floating_body(mass_kg: f64) -> (World, ArticulatedModel, rne_ecs::Entity) {
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
                mass_kg,
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

    fn corner_contacts(base: rne_ecs::Entity) -> Vec<ContactSpec> {
        vec![
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.1, 0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(-0.1, -0.1, 0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.0, -0.1, -0.1),
            },
        ]
    }

    #[test]
    fn a_body_above_the_ground_falls_freely() {
        let (_world, model, base) = floating_body(3.0);
        let dynamics = ContactImplicitArticulatedDynamics::new(
            &model,
            0.005,
            corner_contacts(base),
            CompliantContactModel {
                ground_height_m: -1.0,
                ..CompliantContactModel::default()
            },
        );
        let mut state = vec![0.0; 12];
        state[1] = 0.5;
        for _ in 0..20 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        // No candidate point reached the ground, so gravity acted freely.
        assert!(state[7] < -0.1, "base velocity {}", state[7]);
        assert!(state[1] < 0.5, "base height {}", state[1]);
    }

    #[test]
    fn a_falling_body_is_caught_by_its_candidate_points() {
        let (_world, model, base) = floating_body(3.0);
        let dynamics = ContactImplicitArticulatedDynamics::new(
            &model,
            0.002,
            corner_contacts(base),
            CompliantContactModel {
                stiffness_n_m: 5.0e4,
                damping_n_s_m: 3.0e2,
                ..CompliantContactModel::default()
            },
        );
        // Start with the contact points at the ground; gravity presses them in.
        let mut state = vec![0.0; 12];
        state[1] = 0.1;
        for _ in 0..1500 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        // The compliance holds the body up instead of letting it fall through.
        assert!(state[1] > -0.2, "base fell through: {}", state[1]);
        assert!(state[7].abs() < 0.5, "body never settled: {}", state[7]);
    }

    #[test]
    fn a_rotated_free_body_does_not_drift_its_world_position() {
        let (_world, model, base) = floating_body(3.0);
        let dynamics = ContactImplicitArticulatedDynamics::new(
            &model,
            0.005,
            corner_contacts(base),
            CompliantContactModel {
                ground_height_m: -100.0,
                ..CompliantContactModel::default()
            },
        );
        let mut state = vec![0.0; 12];
        state[1] = 5.0;
        // A quarter-turn yaw. Integrating the raw body twist would leak the
        // gravity acceleration into the world x/z coordinates; the chart
        // integration keeps them fixed and only the world height drops.
        state[5] = std::f64::consts::FRAC_PI_2;
        for _ in 0..20 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        assert!(state[0].abs() < 1.0e-9, "x drifted: {}", state[0]);
        assert!(state[2].abs() < 1.0e-9, "z drifted: {}", state[2]);
        assert!(state[1] < 5.0, "did not fall: {}", state[1]);
    }

    #[test]
    fn a_point_above_the_ground_exerts_no_force() {
        let model = CompliantContactModel::default();
        assert_eq!(model.normal_force_n(-0.01, 0.0), 0.0);
        assert!(model.normal_force_n(0.01, 0.0) > 0.0);
        // The force is clamped for stability.
        let clamped = CompliantContactModel {
            max_normal_force_n: 10.0,
            ..model
        };
        assert_eq!(clamped.normal_force_n(10.0, 0.0), 10.0);
    }

    #[test]
    fn analytic_derivatives_match_finite_difference() {
        let (_world, model, base) = floating_body(3.0);
        let dynamics = ContactImplicitArticulatedDynamics::new(
            &model,
            0.005,
            corner_contacts(base),
            CompliantContactModel {
                stiffness_n_m: 5.0e4,
                damping_n_s_m: 3.0e2,
                ..CompliantContactModel::default()
            },
        );
        let mut state = vec![0.0; 12];
        // Put the contact points just below the ground so the contact force is
        // active and its gradient is non-zero.
        state[1] = 0.05;
        state[7] = -0.2;
        let control: Vec<f64> = Vec::new();
        let analytic = dynamics
            .analytic_derivatives(&state, &control)
            .expect("analytic")
            .expect("some");
        let finite = crate::ddp::dynamics_derivatives(&dynamics, 0, &state, &control, 1.0e-6)
            .expect("finite");
        let nx = 2 * model.nv();
        // The friction regularization makes some entries large (the tangential
        // force derivative is `friction * Fn / reg`), so compare relatively.
        let mut max_relative = 0.0_f64;
        for row in 0..nx {
            for column in 0..nx {
                let scale = analytic.fx[row][column]
                    .abs()
                    .max(finite.fx[row][column].abs())
                    .max(1.0);
                max_relative =
                    max_relative.max((analytic.fx[row][column] - finite.fx[row][column]).abs() / scale);
            }
        }
        assert!(max_relative < 1.0e-4, "fx relative error {max_relative}");
        assert_eq!(analytic.fu.len(), nx);
        assert!(analytic.fu.iter().all(|row| row.is_empty()));
    }

    #[test]
    fn sub_steps_keep_a_stiff_contact_stable() {
        let (_world, model, base) = floating_body(3.0);
        let stiff = CompliantContactModel {
            stiffness_n_m: 1.0e6,
            damping_n_s_m: 1.0e3,
            ..CompliantContactModel::default()
        };
        let single = ContactImplicitArticulatedDynamics::new(
            &model,
            0.01,
            corner_contacts(base),
            stiff,
        );
        let sub = ContactImplicitArticulatedDynamics::new(
            &model,
            0.01,
            corner_contacts(base),
            stiff,
        )
        .with_substeps(64);
        let mut state = vec![0.0; 12];
        state[1] = 0.1;
        let mut single_state = state.clone();
        let mut single_diverged = false;
        for _ in 0..500 {
            match single.step(&single_state, &[]) {
                Ok(next) => single_state = next,
                Err(_) => {
                    single_diverged = true;
                    break;
                }
            }
        }
        for _ in 0..500 {
            state = sub.step(&state, &[]).expect("step");
        }
        assert!(
            single_diverged || !single_state[1].is_finite() || single_state[1] < -1.0,
            "single step unexpectedly stable: {}",
            single_state[1]
        );
        assert!(state[1].is_finite());
        assert!(state[1] > -0.05, "sub-stepped body fell through: {}", state[1]);
    }
}
