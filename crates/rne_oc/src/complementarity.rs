//! Velocity-level complementarity contact dynamics.
//!
//! [`ComplementarityContactDynamics`] resolves hard point contacts with a
//! sequential-impulse (projected Gauss-Seidel) solver instead of a compliant
//! penalty. At each step the post-contact velocity satisfies the contact
//! complementarity conditions to a solver tolerance, so the points do not
//! penetrate and the force is a true constraint impulse rather than a spring
//! force proportional to penetration.
//!
//! The model is intentionally differentiable only through its smooth dynamics;
//! the contact solve is piecewise smooth, which is what a shooting optimizer
//! must contend with for any hard-contact formulation.

use crate::ddp::{DiscreteDynamics, OcError};
use rne_dynamics::{
    frame_jacobian, integrate_configuration, link_motions, mass_matrix, non_linear_effects,
    ArticulatedModel, ContactSpec,
};

/// Hard point contacts solved as a velocity-level complementarity problem.
#[derive(Clone, Debug)]
pub struct ComplementarityContactDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
    /// Candidate contact points; each is clamped against the ground plane.
    pub contacts: Vec<ContactSpec>,
    /// Coulomb friction coefficient.
    pub friction: f64,
    /// Sequential-impulse sweeps per step.
    pub iterations: usize,
    /// Baumgarte position-correction factor that pushes out existing penetration.
    pub baumgarte: f64,
    /// Ground plane height along the world up axis, in meters.
    pub ground_height_m: f64,
    /// Number of impulse sub-steps per call to [`DiscreteDynamics::step`].
    ///
    /// A smaller internal step resolves the contact transition more sharply, at
    /// the cost of more dynamics and Jacobian evaluations per step.
    pub substeps: usize,
}

impl<'a> ComplementarityContactDynamics<'a> {
    /// Creates a hard-contact complementarity dynamics model.
    pub fn new(
        model: &'a ArticulatedModel,
        step_time_s: f64,
        contacts: Vec<ContactSpec>,
        friction: f64,
    ) -> Self {
        Self {
            model,
            step_time_s,
            contacts,
            friction,
            iterations: 40,
            baumgarte: 0.2,
            ground_height_m: 0.0,
            substeps: 1,
        }
    }

    /// Sets the number of impulse sub-steps per step and returns `self`.
    #[must_use]
    pub fn with_substeps(mut self, substeps: usize) -> Self {
        self.substeps = substeps.max(1);
        self
    }

    /// Solves the contact impulses and returns the post-contact velocity.
    ///
    /// `linear_jacobians[i]` holds the `3 x nv` world linear Jacobian of contact
    /// `i`, and `minv_jt[i][c]` is `M^-1 J_i^T e_c`. The sweep is a projected
    /// Gauss-Seidel over the accumulated normal impulse with a clamped tangential
    /// impulse, which converges to the Coulomb solution for well-posed contacts.
    #[allow(clippy::needless_range_loop)]
    fn solve_impulses(
        &self,
        q: &[f64],
        qd_free: &[f64],
        penetration: &[f64],
        mass: &rne_dynamics::DenseMatrix,
        dt: f64,
    ) -> Result<Vec<f64>, OcError> {
        let nv = self.model.nv();
        let nc = self.contacts.len();
        let mut minv_jt: Vec<Vec<Vec<f64>>> = Vec::with_capacity(nc);
        let mut jacobians: Vec<Vec<[f64; 3]>> = Vec::with_capacity(nc);
        for spec in &self.contacts {
            let jac = frame_jacobian(self.model, q, spec.link, spec.point_local_m)
                .map_err(|_| OcError::Dynamics)?;
            let mut rows = vec![[0.0; 3]; nv];
            for column in 0..nv {
                rows[column] = [jac.get(0, column), jac.get(1, column), jac.get(2, column)];
            }
            jacobians.push(rows);
            let mut solved = vec![vec![0.0; nv]; 3];
            for component in 0..3 {
                let basis: Vec<f64> = (0..nv).map(|row| jac.get(component, row)).collect();
                let column = mass.solve(&basis).ok_or(OcError::Dynamics)?;
                solved[component].copy_from_slice(&column[..nv]);
            }
            minv_jt.push(solved);
        }

        let mut qd = qd_free.to_vec();
        let mut lambda = vec![0.0_f64; nc];
        // A point only pushes if it is at the surface or will reach it within
        // this step; a point above the ground contributes nothing.
        let active: Vec<bool> = (0..nc)
            .map(|i| {
                let mut vn_free = 0.0;
                for dof in 0..nv {
                    vn_free += jacobians[i][dof][1] * qd_free[dof];
                }
                penetration[i] - vn_free * dt >= -1.0e-9
            })
            .collect();
        for _ in 0..self.iterations.max(1) {
            for i in 0..nc {
                if !active[i] {
                    continue;
                }
                // Normal direction is world up (component 1). The Baumgarte
                // term lowers the target velocity so existing penetration is
                // pushed out; without it a resting body would keep the residual
                // penetration it started with.
                let mut vn = -self.baumgarte * penetration[i].max(0.0) / dt;
                for dof in 0..nv {
                    vn += jacobians[i][dof][1] * qd[dof];
                }
                let mut denom = 0.0;
                for dof in 0..nv {
                    denom += jacobians[i][dof][1] * minv_jt[i][1][dof];
                }
                if denom <= 1.0e-12 {
                    continue;
                }
                let delta = (-vn / denom).max(-lambda[i]);
                lambda[i] += delta;
                for dof in 0..nv {
                    qd[dof] += minv_jt[i][1][dof] * delta;
                }

                // Clamped tangential impulse against the accumulated normal one.
                let mut tangential = [0.0_f64; 2];
                for dof in 0..nv {
                    tangential[0] += jacobians[i][dof][0] * qd[dof];
                    tangential[1] += jacobians[i][dof][2] * qd[dof];
                }
                let mut effective = [[0.0_f64; 2]; 2];
                for (a, component_a) in [0_usize, 2].iter().enumerate() {
                    for (b, component_b) in [0_usize, 2].iter().enumerate() {
                        for dof in 0..nv {
                            effective[a][b] +=
                                jacobians[i][dof][*component_a] * minv_jt[i][*component_b][dof];
                        }
                    }
                }
                let determinant =
                    effective[0][0] * effective[1][1] - effective[0][1] * effective[1][0];
                if determinant.abs() < 1.0e-12 {
                    continue;
                }
                let impulse = [
                    -(effective[1][1] * tangential[0] - effective[0][1] * tangential[1])
                        / determinant,
                    -(-effective[1][0] * tangential[0] + effective[0][0] * tangential[1])
                        / determinant,
                ];
                let magnitude = (impulse[0] * impulse[0] + impulse[1] * impulse[1]).sqrt();
                let limit = self.friction * lambda[i];
                let scale = if magnitude > limit && magnitude > 0.0 {
                    limit / magnitude
                } else {
                    1.0
                };
                let impulse = [impulse[0] * scale, impulse[1] * scale];
                for dof in 0..nv {
                    qd[dof] += minv_jt[i][0][dof] * impulse[0] + minv_jt[i][2][dof] * impulse[1];
                }
            }
        }
        Ok(qd)
    }

    /// One impulse sub-step: free acceleration, contact solve, then a
    /// semi-implicit configuration update with the post-contact velocity.
    #[allow(clippy::needless_range_loop)]
    fn substep(
        &self,
        q: &[f64],
        qd: &[f64],
        torque: &[f64],
        dt: f64,
    ) -> Result<(Vec<f64>, Vec<f64>), OcError> {
        let nv = self.model.nv();
        let mass = mass_matrix(self.model, q).map_err(|_| OcError::Dynamics)?;
        let bias = non_linear_effects(self.model, q, qd).map_err(|_| OcError::Dynamics)?;
        let rhs: Vec<f64> = (0..nv).map(|i| torque[i] - bias[i]).collect();
        let free_acceleration = mass.solve(&rhs).ok_or(OcError::Dynamics)?;
        let qd_free: Vec<f64> = (0..nv).map(|i| qd[i] + free_acceleration[i] * dt).collect();

        let motions = link_motions(self.model, q, qd).map_err(|_| OcError::Dynamics)?;
        let mut penetration = vec![0.0; self.contacts.len()];
        for (index, spec) in self.contacts.iter().enumerate() {
            if let Some(link_index) = self.model.kinematic().link_index(spec.link) {
                let motion = motions[link_index];
                let offset = motion.world_transform.rotation * spec.point_local_m;
                let point = motion.world_transform.translation + offset;
                penetration[index] = self.ground_height_m - point.y;
            }
        }

        let qd_next = self.solve_impulses(q, &qd_free, &penetration, &mass, dt)?;
        let q_next = integrate_configuration(self.model, q, &qd_next, &vec![0.0; nv], dt)
            .map_err(|_| OcError::Dynamics)?;
        Ok((q_next, qd_next))
    }
}

impl DiscreteDynamics for ComplementarityContactDynamics<'_> {
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
            return Err(OcError::Dimension("complementarity state or control"));
        }
        let mut torque = vec![0.0; nv];
        torque[base..].copy_from_slice(control);
        let mut q = state[..nv].to_vec();
        let mut qd = state[nv..].to_vec();
        let dt = self.step_time_s / self.substeps.max(1) as f64;
        for _ in 0..self.substeps.max(1) {
            let (q_next, qd_next) = self.substep(&q, &qd, &torque, dt)?;
            q = q_next;
            qd = qd_next;
        }
        let mut next = vec![0.0; 2 * nv];
        next[..nv].copy_from_slice(&q);
        next[nv..].copy_from_slice(&qd);
        Ok(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::{spawn_named, World};
    use rne_math::Vec3;
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
    fn a_falling_body_is_caught_without_penetration() {
        let (_world, model, base) = floating_body(3.0);
        let dynamics =
            ComplementarityContactDynamics::new(&model, 0.005, corner_contacts(base), 0.8);
        let mut state = vec![0.0; 12];
        state[1] = 0.05;
        state[7] = -1.0;
        for _ in 0..400 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        // The hard contact holds the body up: the points stop at the surface
        // instead of sinking by a stiffness-dependent penetration.
        assert!(state[1].is_finite());
        assert!(state[1] > 0.09, "body sank to {}", state[1]);
        assert!(state[7].abs() < 0.2, "body never stopped: {}", state[7]);
    }

    #[test]
    fn a_body_above_the_ground_falls_freely() {
        let (_world, model, base) = floating_body(3.0);
        let mut dynamics =
            ComplementarityContactDynamics::new(&model, 0.005, corner_contacts(base), 0.8);
        dynamics.ground_height_m = -1.0;
        let mut state = vec![0.0; 12];
        state[1] = 0.5;
        for _ in 0..20 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        assert!(state[7] < -0.1, "base velocity {}", state[7]);
        assert!(state[1] < 0.5, "base height {}", state[1]);
    }
}
