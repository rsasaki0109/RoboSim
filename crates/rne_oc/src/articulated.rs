//! Discrete dynamics for a floating- or fixed-base articulated model.

use crate::ddp::{DiscreteDynamics, DynamicsDerivatives, DynamicsHessian, OcError};
use rne_dynamics::{
    forward_dynamics, forward_dynamics_gradient, integrate_configuration, ArticulatedModel,
};

/// Semi-implicit Euler dynamics for an [`ArticulatedModel`].
///
/// The state is `[q, qd]` and the control is the actuated-joint torque,
/// excluding the floating-base rows. Each call evaluates the native forward
/// dynamics and integrates one step.
pub struct ArticulatedDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
}

impl<'a> ArticulatedDynamics<'a> {
    /// Creates an articulated dynamics model.
    pub fn new(model: &'a ArticulatedModel, step_time_s: f64) -> Self {
        Self { model, step_time_s }
    }
}

impl DiscreteDynamics for ArticulatedDynamics<'_> {
    fn state_dim(&self) -> usize {
        2 * self.model.nv()
    }

    fn control_dim(&self) -> usize {
        self.model.nv() - self.model.base_dof()
    }

    fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        let nv = self.model.nv();
        let base = self.model.base_dof();
        if state.len() != 2 * nv || control.len() != nv - base {
            return Err(OcError::Dimension("articulated state or control"));
        }
        let (q, qd) = state.split_at(nv);
        let mut torque = vec![0.0; nv];
        torque[base..].copy_from_slice(control);
        let acceleration =
            forward_dynamics(self.model, q, qd, &torque).map_err(|_| OcError::Dynamics)?;

        let dt = self.step_time_s;
        let mut next = vec![0.0; 2 * nv];
        next[..nv].copy_from_slice(
            &integrate_configuration(self.model, q, qd, &acceleration, dt)
                .map_err(|_| OcError::Dynamics)?,
        );
        for index in 0..nv {
            next[nv + index] = qd[index] + acceleration[index] * dt;
        }
        Ok(next)
    }

    fn analytic_derivatives(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Option<Result<DynamicsDerivatives, OcError>> {
        Some(self.analytic_derivatives_inner(state, control))
    }

    fn analytic_hessian(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Option<Result<DynamicsHessian, OcError>> {
        Some(self.analytic_hessian_inner(state, control))
    }
}

impl ArticulatedDynamics<'_> {
    /// Builds `(df/dx, df/du)` from the analytical forward-dynamics gradient.
    ///
    /// The state layout is `[q, qd]` and the control is the actuated-joint
    /// torque. `d(qdd)/d(q, qd, tau)` is analytic; the configuration step maps
    /// `qdd` through `integrate_configuration`, whose chart Jacobian for a
    /// floating base is taken by a small central difference of that map alone
    /// (it does not re-solve the dynamics).
    #[allow(clippy::needless_range_loop)]
    fn analytic_derivatives_inner(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Result<DynamicsDerivatives, OcError> {
        let nv = self.model.nv();
        let base = self.model.base_dof();
        if state.len() != 2 * nv || control.len() != nv - base {
            return Err(OcError::Dimension("articulated state or control"));
        }
        let (q, qd) = state.split_at(nv);
        let mut torque = vec![0.0; nv];
        torque[base..].copy_from_slice(control);

        let gradient =
            forward_dynamics_gradient(self.model, q, qd, &torque).map_err(|_| OcError::Dynamics)?;
        let acceleration =
            forward_dynamics(self.model, q, qd, &torque).map_err(|_| OcError::Dynamics)?;

        let nx = 2 * nv;
        let nu = nv - base;
        let mut fx = vec![vec![0.0; nx]; nx];
        let mut fu = vec![vec![0.0; nu]; nx];

        // Position rows: `q_next = integrate_configuration(q, qd, qdd, dt)`.
        // The joint block is exact (`q + qd dt + 0.5 qdd dt^2`); the floating
        // base chart derivative is a small central difference of the map.
        let dt = self.step_time_s;
        let epsilon = 1.0e-7;
        let integrate_at = |q_value: &[f64], qd_value: &[f64], qdd_value: &[f64]| {
            integrate_configuration(self.model, q_value, qd_value, qdd_value, dt)
        };

        for k in 0..nv {
            let mut q_shift = q.to_vec();
            q_shift[k] += epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] += gradient.with_respect_to_q[k][row] * epsilon;
            }
            let plus = integrate_at(&q_shift, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut q_shift = q.to_vec();
            q_shift[k] -= epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= gradient.with_respect_to_q[k][row] * epsilon;
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
                qdd_shift[row] += gradient.with_respect_to_qd[k][row] * epsilon;
            }
            let plus = integrate_at(q, &qd_shift, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut qd_shift = qd.to_vec();
            qd_shift[k] -= epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= gradient.with_respect_to_qd[k][row] * epsilon;
            }
            let minus = integrate_at(q, &qd_shift, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                fx[row][nv + k] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }
        for j in 0..nu {
            let mut tau_shift = torque.clone();
            tau_shift[base + j] += epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] += gradient.with_respect_to_control[base + j][row] * epsilon;
            }
            let plus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            let mut tau_shift = torque.clone();
            tau_shift[base + j] -= epsilon;
            let mut qdd_shift = acceleration.clone();
            for row in 0..nv {
                qdd_shift[row] -= gradient.with_respect_to_control[base + j][row] * epsilon;
            }
            let minus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
            for row in 0..nv {
                fu[row][j] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }

        // Velocity rows: `qd_next = qd + qdd dt`, entirely analytic.
        for k in 0..nv {
            for row in 0..nv {
                fx[nv + row][k] = gradient.with_respect_to_q[k][row] * dt;
                fx[nv + row][nv + k] =
                    if row == k { 1.0 } else { 0.0 } + gradient.with_respect_to_qd[k][row] * dt;
            }
        }
        for j in 0..nu {
            for row in 0..nv {
                fu[nv + row][j] = gradient.with_respect_to_control[base + j][row] * dt;
            }
        }

        Ok(DynamicsDerivatives { fx, fu })
    }

    /// Builds `(d²f/dx², d²f/dxdu, d²f/du²)` by differentiating the analytical
    /// first derivatives at the same point.
    ///
    /// The first-derivative map is the one the solver uses, so differentiating
    /// it keeps the second derivatives consistent with the Jacobians even where
    /// the floating-base chart Jacobian is itself a small central difference.
    #[allow(clippy::needless_range_loop)]
    fn analytic_hessian_inner(
        &self,
        state: &[f64],
        control: &[f64],
    ) -> Result<DynamicsHessian, OcError> {
        let nv = self.model.nv();
        let nx = 2 * nv;
        let nu = self.model.nv() - self.model.base_dof();
        let epsilon = 1.0e-6;
        let shift = |base: &[f64], index: usize, delta: f64| {
            let mut value = base.to_vec();
            value[index] += delta;
            value
        };
        let mut fxx = vec![vec![vec![0.0; nx]; nx]; nx];
        let mut fxu = vec![vec![vec![0.0; nu]; nx]; nx];
        let mut fuu = vec![vec![vec![0.0; nu]; nu]; nx];

        for j in 0..nx {
            let plus = self.analytic_derivatives_inner(&shift(state, j, epsilon), control)?;
            let minus = self.analytic_derivatives_inner(&shift(state, j, -epsilon), control)?;
            for a in 0..nx {
                for i in 0..nx {
                    fxx[a][i][j] = (plus.fx[a][i] - minus.fx[a][i]) / (2.0 * epsilon);
                }
                for control_index in 0..nu {
                    fxu[a][j][control_index] =
                        (plus.fu[a][control_index] - minus.fu[a][control_index]) / (2.0 * epsilon);
                }
            }
        }
        for j in 0..nu {
            let plus = self.analytic_derivatives_inner(state, &shift(control, j, epsilon))?;
            let minus = self.analytic_derivatives_inner(state, &shift(control, j, -epsilon))?;
            for a in 0..nx {
                for i in 0..nu {
                    fuu[a][i][j] = (plus.fu[a][i] - minus.fu[a][i]) / (2.0 * epsilon);
                }
            }
        }

        Ok(DynamicsHessian { fxx, fxu, fuu })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ddp::{solve, DdpConfig, QuadraticCost};
    use rne_dynamics::ArticulatedModel;
    use rne_ecs::{spawn_named, World};
    use rne_math::Vec3;
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{Joint, JointKind, JointLimits, Link, Robot, RobotId};
    use rne_world::Transform3;

    fn pendulum_model() -> ArticulatedModel {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link = spawn_named(&mut world, "link");
        let joint = spawn_named(&mut world, "joint");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "pendulum".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: 0.0,
                ..RigidBody::default()
            },
        ));
        world.entity_mut(link).insert((
            Link {
                robot,
                name: "link".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::new(1.0, 0.0, 0.0),
                ixx_kg_m2: 0.0,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.0,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.0,
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
        ArticulatedModel::from_robot(&world, robot).expect("model")
    }

    fn floating_two_link_model() -> ArticulatedModel {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let link2 = spawn_named(&mut world, "link2");
        let joint1 = spawn_named(&mut world, "joint1");
        let joint2 = spawn_named(&mut world, "joint2");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "floating_two_link".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            rne_robot::FloatingBase,
            RigidBody {
                mass_kg: 2.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::new(0.1, 0.0, 0.0),
                ixx_kg_m2: 0.1,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.1,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.1,
            },
        ));
        for (entity, offset) in [(link1, Vec3::ZERO), (link2, Vec3::new(0.4, 0.0, 0.0))] {
            world.entity_mut(entity).insert((
                Link {
                    robot,
                    name: "link".into(),
                },
                Transform3::from_translation_rotation(offset, rne_math::Quat::IDENTITY),
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
        }
        for (entity, parent, child) in [(joint1, base, link1), (joint2, link1, link2)] {
            world.entity_mut(entity).insert(Joint {
                robot,
                parent_link: parent,
                child_link: child,
                kind: JointKind::Revolute,
                limits: JointLimits::default(),
                axis: Vec3::Z,
                position: 0.0,
                velocity: 0.0,
            });
        }
        ArticulatedModel::from_robot(&world, robot).expect("model")
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn analytic_derivatives_match_finite_difference() {
        use crate::ddp::dynamics_derivatives;
        let model = floating_two_link_model();
        let dynamics = ArticulatedDynamics::new(&model, 0.02);
        let nv = model.nv();
        let state: Vec<f64> = vec![
            0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7, 0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3,
        ];
        let control = vec![0.35, -0.2];
        let analytic = dynamics
            .analytic_derivatives(&state, &control)
            .expect("analytic")
            .expect("some");
        let finite = dynamics_derivatives(&dynamics, 0, &state, &control, 1.0e-6).expect("fd");

        let mut max_fx = 0.0_f64;
        for row in 0..2 * nv {
            for column in 0..2 * nv {
                max_fx = max_fx.max((analytic.fx[row][column] - finite.fx[row][column]).abs());
            }
        }
        let mut max_fu = 0.0_f64;
        for row in 0..2 * nv {
            for column in 0..nv - model.base_dof() {
                max_fu = max_fu.max((analytic.fu[row][column] - finite.fu[row][column]).abs());
            }
        }
        assert!(max_fx < 1.0e-4, "fx error {max_fx}");
        assert!(max_fu < 1.0e-4, "fu error {max_fu}");
    }

    #[test]
    fn exact_hessian_converges_on_a_floating_chain() {
        use crate::ddp::QuadraticCost;
        use crate::multiple_shooting::{
            max_defect, solve_multiple_shooting, MultipleShootingConfig,
        };
        let model = floating_two_link_model();
        let dynamics = ArticulatedDynamics::new(&model, 0.02);
        let nx = 2 * model.nv();
        let nu = model.nv() - model.base_dof();
        let horizon = 20;
        let mut states = Vec::new();
        for k in 0..=horizon {
            let t = k as f64 / horizon as f64;
            let mut state = vec![0.0; nx];
            state[1] = 0.5;
            state[6] = 0.4 * t;
            state[7] = -0.3 * t;
            states.push(state);
        }
        let controls = vec![vec![0.0; nu]; horizon];
        let cost = QuadraticCost::new(vec![0.0; nx], vec![0.001; nu], vec![0.0; nx]);
        let run = |use_exact_hessian: bool| {
            let config = MultipleShootingConfig {
                max_iterations: 120,
                tolerance: 1.0e-6,
                use_exact_hessian,
                ..MultipleShootingConfig::default()
            };
            solve_multiple_shooting(&dynamics, &cost, &states, &controls, &config).expect("solve")
        };
        let gauss_newton = run(false);
        let newton = run(true);
        let gn_defect = max_defect(&dynamics, &gauss_newton.states, &gauss_newton.controls);
        let newton_defect = max_defect(&dynamics, &newton.states, &newton.controls);
        assert!(gn_defect < 1.0e-2, "gauss-newton defect {gn_defect}");
        assert!(newton_defect < 1.0e-2, "newton defect {newton_defect}");
        assert!(
            newton_defect <= gn_defect * 1.5,
            "newton {newton_defect} worse than gauss-newton {gn_defect}"
        );
    }

    #[test]
    fn pendulum_swings_up_under_ddp() {
        let model = pendulum_model();
        let dynamics = ArticulatedDynamics::new(&model, 0.02);
        let horizon = 150;
        let target = std::f64::consts::FRAC_PI_2;
        let hanging = -std::f64::consts::FRAC_PI_2;
        let mut cost = QuadraticCost::new(vec![0.0, 0.0], vec![0.002], vec![200.0, 20.0]);
        cost.state_reference = vec![target, 0.0];
        cost.running_scale = 0.02;
        let states = vec![vec![hanging, 0.0]; horizon + 1];
        let controls = vec![vec![0.0]; horizon];
        let config = DdpConfig {
            max_iterations: 300,
            tolerance: 1.0e-10,
            ..DdpConfig::default()
        };
        let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
        let final_angle = solution.states[horizon][0];
        let final_rate = solution.states[horizon][1];
        assert!(
            (final_angle - target).abs() < 0.3,
            "angle {final_angle}, rate {final_rate}, cost {}",
            solution.cost
        );
    }
}
