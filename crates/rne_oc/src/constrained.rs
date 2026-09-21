//! Contact-constrained articulated dynamics and contact sequences.

use crate::ddp::{DiscreteDynamics, DynamicsDerivatives, OcError, ShootingDynamics};
use rne_dynamics::{
    constrained_forward_dynamics, constrained_forward_dynamics_gradient, impulse_velocity,
    impulse_velocity_gradient, integrate_configuration, ArticulatedModel, ContactSpec,
};

fn integrate(
    model: &ArticulatedModel,
    step_time_s: f64,
    state: &[f64],
    control: &[f64],
    contacts: &[ContactSpec],
) -> Result<Vec<f64>, OcError> {
    let nv = model.nv();
    let base = model.base_dof();
    if state.len() != 2 * nv || control.len() != nv - base {
        return Err(OcError::Dimension(
            "constrained articulated state or control",
        ));
    }
    let (q, qd) = state.split_at(nv);
    let mut torque = vec![0.0; nv];
    torque[base..].copy_from_slice(control);
    let (acceleration, _) = constrained_forward_dynamics(model, q, qd, &torque, contacts)
        .map_err(|_| OcError::Dynamics)?;
    let dt = step_time_s;
    let mut next = vec![0.0; 2 * nv];
    next[..nv].copy_from_slice(
        &integrate_configuration(model, q, qd, &acceleration, dt).map_err(|_| OcError::Dynamics)?,
    );
    for index in 0..nv {
        next[nv + index] = qd[index] + acceleration[index] * dt;
    }
    Ok(next)
}

/// Analytical `(df/dx, df/du)` of one [`integrate`] step.
///
/// The constrained forward dynamics are differentiated analytically through
/// [`constrained_forward_dynamics_gradient`]; the floating-base chart Jacobian
/// of the configuration update is a small central difference of that map, as in
/// `ArticulatedDynamics`.
#[allow(clippy::needless_range_loop)]
fn integrate_derivatives(
    model: &ArticulatedModel,
    step_time_s: f64,
    state: &[f64],
    control: &[f64],
    contacts: &[ContactSpec],
) -> Result<DynamicsDerivatives, OcError> {
    let nv = model.nv();
    let base = model.base_dof();
    let nx = 2 * nv;
    let nu = nv - base;
    let (q, qd) = state.split_at(nv);
    let mut torque = vec![0.0; nv];
    torque[base..].copy_from_slice(control);

    let (acceleration, _) = constrained_forward_dynamics(model, q, qd, &torque, contacts)
        .map_err(|_| OcError::Dynamics)?;
    let gradient = constrained_forward_dynamics_gradient(model, q, qd, &torque, contacts)
        .map_err(|_| OcError::Dynamics)?;

    let epsilon = 1.0e-7;
    let dt = step_time_s;
    let integrate_at = |q_value: &[f64], qd_value: &[f64], qdd_value: &[f64]| {
        integrate_configuration(model, q_value, qd_value, qdd_value, dt)
    };
    let mut fx = vec![vec![0.0; nx]; nx];
    let mut fu = vec![vec![0.0; nu]; nx];
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
        let mut qdd_shift = acceleration.clone();
        for row in 0..nv {
            qdd_shift[row] += gradient.with_respect_to_control[base + j][row] * epsilon;
        }
        let plus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
        let mut qdd_shift = acceleration.clone();
        for row in 0..nv {
            qdd_shift[row] -= gradient.with_respect_to_control[base + j][row] * epsilon;
        }
        let minus = integrate_at(q, qd, &qdd_shift).map_err(|_| OcError::Dynamics)?;
        for row in 0..nu {
            fu[row][j] = (plus[row] - minus[row]) / (2.0 * epsilon);
        }
    }
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

/// Semi-implicit Euler over the contact-constrained articulated dynamics.
///
/// The active contact set is constant across the horizon, so the points neither
/// accelerate nor separate.
pub struct ConstrainedArticulatedDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
    /// Active contacts.
    pub contacts: Vec<ContactSpec>,
}

impl<'a> ConstrainedArticulatedDynamics<'a> {
    /// Creates a contact-constrained dynamics model.
    pub fn new(model: &'a ArticulatedModel, step_time_s: f64, contacts: Vec<ContactSpec>) -> Self {
        Self {
            model,
            step_time_s,
            contacts,
        }
    }
}

impl DiscreteDynamics for ConstrainedArticulatedDynamics<'_> {
    fn state_dim(&self) -> usize {
        2 * self.model.nv()
    }

    fn control_dim(&self) -> usize {
        self.model.nv() - self.model.base_dof()
    }

    fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        integrate(self.model, self.step_time_s, state, control, &self.contacts)
    }
}

/// One phase of a contact sequence.
#[derive(Clone, Debug)]
pub struct ContactPhase {
    /// Contacts active during the phase.
    pub contacts: Vec<ContactSpec>,
    /// Number of steps in the phase.
    pub steps: usize,
}

/// A node-dependent dynamics model over a contact sequence.
///
/// Each node uses the active contacts of its phase. When a node adds contacts,
/// the impulsive velocity reset projects the pre-impact velocity onto the new
/// contact set by default. This is the physically correct hard impact, but it
/// makes the transition non-smooth, which trajectory optimization handles
/// poorly. [`Self::new_without_impact`] disables the reset so the regularized
/// constrained dynamics at the next node absorbs the contact instead, which is
/// a common smoothing choice for planning.
pub struct ContactSequenceDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
    contacts_per_node: Vec<Vec<ContactSpec>>,
    reset_per_node: Vec<Option<Vec<ContactSpec>>>,
    impact_reset: bool,
    substeps: usize,
}

impl<'a> ContactSequenceDynamics<'a> {
    /// Expands a phase list into a per-node contact schedule.
    pub fn new(model: &'a ArticulatedModel, step_time_s: f64, phases: &[ContactPhase]) -> Self {
        Self::build(model, step_time_s, phases, true, 1)
    }

    /// Like [`Self::new`] but integrates each step with `substeps` inner steps,
    /// which stabilizes stiff contact modes at the cost of more dynamics
    /// evaluations per step.
    pub fn new_with_substeps(
        model: &'a ArticulatedModel,
        step_time_s: f64,
        phases: &[ContactPhase],
        substeps: usize,
    ) -> Self {
        Self::build(model, step_time_s, phases, true, substeps.max(1))
    }

    /// Like [`Self::new`] but without the impulsive velocity reset at contact
    /// additions, so the transition is smooth for a trajectory optimizer.
    pub fn new_without_impact(
        model: &'a ArticulatedModel,
        step_time_s: f64,
        phases: &[ContactPhase],
    ) -> Self {
        Self::build(model, step_time_s, phases, false, 1)
    }

    fn build(
        model: &'a ArticulatedModel,
        step_time_s: f64,
        phases: &[ContactPhase],
        impact_reset: bool,
        substeps: usize,
    ) -> Self {
        let mut contacts_per_node = Vec::new();
        for phase in phases {
            for _ in 0..phase.steps {
                contacts_per_node.push(phase.contacts.clone());
            }
        }
        let mut reset_per_node: Vec<Option<Vec<ContactSpec>>> = vec![None; contacts_per_node.len()];
        for node in 0..contacts_per_node.len().saturating_sub(1) {
            let current = &contacts_per_node[node];
            let next = &contacts_per_node[node + 1];
            let adds = next.iter().any(|contact| !current.contains(contact));
            if adds && !next.is_empty() {
                reset_per_node[node] = Some(next.clone());
            }
        }
        Self {
            model,
            step_time_s,
            contacts_per_node,
            reset_per_node,
            impact_reset,
            substeps,
        }
    }

    /// Sets the number of inner integration steps per step and returns `self`,
    /// so it composes with [`Self::new_without_impact`].
    pub fn with_substeps(mut self, substeps: usize) -> Self {
        self.substeps = substeps.max(1);
        self
    }

    /// Number of steps in the sequence.
    pub fn node_count(&self) -> usize {
        self.contacts_per_node.len()
    }
}

impl ShootingDynamics for ContactSequenceDynamics<'_> {
    fn state_dim(&self) -> usize {
        2 * self.model.nv()
    }

    fn control_dim(&self) -> usize {
        self.model.nv() - self.model.base_dof()
    }

    fn step_at(&self, node: usize, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        let contacts = self
            .contacts_per_node
            .get(node)
            .ok_or(OcError::Dimension("contact sequence node"))?;
        // Sub-stepping holds the control over the step but integrates the
        // constrained dynamics at a finer resolution, which stabilizes the
        // stiff contact modes a single semi-implicit Euler step cannot.
        let mut next = state.to_vec();
        let sub_dt = self.step_time_s / self.substeps as f64;
        for _ in 0..self.substeps {
            next = integrate(self.model, sub_dt, &next, control, contacts)?;
        }
        if self.impact_reset {
            if let Some(reset) = &self.reset_per_node[node] {
                let nv = self.model.nv();
                let (q, qd) = next.split_at(nv);
                let (post_impact, _) =
                    impulse_velocity(self.model, q, qd, reset).map_err(|_| OcError::Dynamics)?;
                next[nv..].copy_from_slice(&post_impact);
            }
        }
        Ok(next)
    }

    /// Analytical Jacobians with an exact impulse reset.
    ///
    /// The smooth integration is central-differenced and the contact-addition
    /// reset is composed on top through
    /// [`rne_dynamics::impulse_velocity_gradient`]. Differencing across the
    /// reset is what makes the transition non-smooth, so this keeps the
    /// Jacobian exact at an impact node.
    #[allow(clippy::needless_range_loop)]
    fn analytic_derivatives(
        &self,
        node: usize,
        state: &[f64],
        control: &[f64],
    ) -> Option<Result<DynamicsDerivatives, OcError>> {
        let contacts = self.contacts_per_node.get(node)?;
        let nv = self.model.nv();
        let nx = 2 * nv;
        let nu = nv - self.model.base_dof();
        if state.len() != nx || control.len() != nu {
            return Some(Err(OcError::Dimension("contact sequence state or control")));
        }
        if self.substeps > 1 {
            // The composed sub-stepped Jacobian is not assembled here.
            return None;
        }
        let DynamicsDerivatives { mut fx, mut fu } =
            match integrate_derivatives(self.model, self.step_time_s, state, control, contacts) {
                Ok(value) => value,
                Err(_) => return Some(Err(OcError::Dynamics)),
            };
        if !self.impact_reset {
            return Some(Ok(DynamicsDerivatives { fx, fu }));
        }
        if let Some(reset) = &self.reset_per_node[node] {
            let next = integrate(self.model, self.step_time_s, state, control, contacts).ok()?;
            let (q, _) = next.split_at(nv);
            let reset_jacobian = impulse_velocity_gradient(self.model, q, reset).ok()?;
            // The reset leaves positions and applies `P` to the velocity rows.
            // Read the original velocity rows before overwriting them.
            let velocity_rows: Vec<Vec<f64>> = (0..nv)
                .map(|row| (0..nx).map(|column| fx[nv + row][column]).collect())
                .collect();
            for column in 0..nx {
                for row in 0..nv {
                    let value: f64 = (0..nv)
                        .map(|index| reset_jacobian.get(row, index) * velocity_rows[index][column])
                        .sum();
                    fx[nv + row][column] = value;
                }
            }
            let velocity_rows_u: Vec<Vec<f64>> = (0..nv)
                .map(|row| (0..nu).map(|column| fu[nv + row][column]).collect())
                .collect();
            for column in 0..nu {
                for row in 0..nv {
                    let value: f64 = (0..nv)
                        .map(|index| {
                            reset_jacobian.get(row, index) * velocity_rows_u[index][column]
                        })
                        .sum();
                    fu[nv + row][column] = value;
                }
            }
        }
        Some(Ok(DynamicsDerivatives { fx, fu }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_dynamics::ArticulatedModel;
    use rne_ecs::{spawn_named, World};
    use rne_math::Vec3;
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Link, Robot, RobotId};
    use rne_world::Transform3;

    fn floating_body() -> (World, ArticulatedModel, rne_ecs::Entity) {
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

    fn contacts(base: rne_ecs::Entity) -> Vec<ContactSpec> {
        vec![
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.2, 0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.2, -0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(-0.1, -0.2, 0.0),
            },
        ]
    }

    #[test]
    fn constrained_body_stays_put_on_its_contacts() {
        let (_world, model, base) = floating_body();
        let dynamics = ConstrainedArticulatedDynamics::new(&model, 0.01, contacts(base));
        let mut state = vec![0.0; 12];
        for _ in 0..50 {
            state = dynamics.step(&state, &[]).expect("step");
        }
        // The base has not fallen.
        assert!(state[1].abs() < 1.0e-6, "base y {}", state[1]);
        assert!(state.iter().all(|value| value.abs() < 1.0e-6));
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn contact_sequence_applies_an_impact_reset() {
        let (_world, model, base) = floating_body();
        let phases = [
            ContactPhase {
                contacts: Vec::new(),
                steps: 5,
            },
            ContactPhase {
                contacts: contacts(base),
                steps: 5,
            },
        ];
        let dynamics = ContactSequenceDynamics::new(&model, 0.01, &phases);
        let mut state = vec![0.0; 12];
        state[1] = 0.2;
        state[7] = -2.0;
        for node in 0..4 {
            state = dynamics.step_at(node, &state, &[]).expect("free step");
        }
        let incoming = state[7];
        state = dynamics.step_at(4, &state, &[]).expect("impact step");
        assert!(state[7] > incoming, "impact did not arrest the fall");
        let qd = &state[6..];
        for spec in contacts(base) {
            let jac =
                rne_dynamics::frame_jacobian(&model, &state[..6], spec.link, spec.point_local_m)
                    .expect("jacobian");
            for row in 0..3 {
                let velocity: f64 = (0..6).map(|column| jac.get(row, column) * qd[column]).sum();
                assert!(velocity.abs() < 1.0e-6, "contact velocity {velocity}");
            }
        }
    }

    #[test]
    fn substepping_keeps_a_contact_body_stable() {
        let (_world, model, base) = floating_body();
        let phases = [ContactPhase {
            contacts: contacts(base),
            steps: 50,
        }];
        let fine = ContactSequenceDynamics::new_with_substeps(&model, 0.02, &phases, 5);
        let mut state = vec![0.0; 12];
        for node in 0..50 {
            state = fine.step_at(node, &state, &[]).expect("step");
        }
        // The body stays on its contacts under the finer integration.
        assert!(state.iter().all(|value| value.abs() < 1.0e-6), "{state:?}");
    }

    #[test]
    fn without_impact_the_transition_stays_free() {
        let (_world, model, base) = floating_body();
        let phases = [
            ContactPhase {
                contacts: Vec::new(),
                steps: 5,
            },
            ContactPhase {
                contacts: contacts(base),
                steps: 5,
            },
        ];
        let with_impact = ContactSequenceDynamics::new(&model, 0.01, &phases);
        let without_impact = ContactSequenceDynamics::new_without_impact(&model, 0.01, &phases);
        let mut state = vec![0.0; 12];
        state[1] = 0.2;
        state[7] = -2.0;
        for node in 0..4 {
            state = without_impact
                .step_at(node, &state, &[])
                .expect("free step");
        }
        let with = with_impact.step_at(4, &state, &[]).expect("impact step");
        let without = without_impact.step_at(4, &state, &[]).expect("smooth step");
        // The hard impact arrests the downward velocity; the smooth transition
        // leaves it free so the constrained node can absorb it over the step.
        assert!(
            with[7] > without[7],
            "reset {} did not arrest more than smooth {}",
            with[7],
            without[7]
        );
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn impact_node_jacobian_matches_finite_difference() {
        use crate::ddp::dynamics_derivatives;
        let (_world, model, base) = floating_body();
        let phases = [
            ContactPhase {
                contacts: Vec::new(),
                steps: 5,
            },
            ContactPhase {
                contacts: contacts(base),
                steps: 5,
            },
        ];
        let dynamics = ContactSequenceDynamics::new(&model, 0.01, &phases);
        // Node 4 is the last free node; its step applies the impact reset.
        let mut state = vec![0.0; 12];
        state[1] = 0.2;
        state[7] = -2.0;
        for node in 0..4 {
            state = dynamics.step_at(node, &state, &[]).expect("free step");
        }
        let analytic = dynamics
            .analytic_derivatives(4, &state, &[])
            .expect("analytic")
            .expect("some");
        let finite = dynamics_derivatives(&dynamics, 4, &state, &[], 1.0e-7).expect("fd");
        let mut max_fx = 0.0_f64;
        for row in 0..12 {
            for column in 0..12 {
                max_fx = max_fx.max((analytic.fx[row][column] - finite.fx[row][column]).abs());
            }
        }
        assert!(max_fx < 1.0e-3, "impact fx error {max_fx}");
    }

    #[test]
    fn contact_sequence_switches_between_held_and_free() {
        let (_world, model, base) = floating_body();
        let phases = [
            ContactPhase {
                contacts: contacts(base),
                steps: 20,
            },
            ContactPhase {
                contacts: Vec::new(),
                steps: 20,
            },
        ];
        let dynamics = ContactSequenceDynamics::new(&model, 0.01, &phases);
        assert_eq!(dynamics.node_count(), 40);
        let mut state = vec![0.0; 12];
        for node in 0..20 {
            state = dynamics.step_at(node, &state, &[]).expect("held step");
        }
        assert!(
            state[1].abs() < 1.0e-6,
            "base moved while held: {}",
            state[1]
        );
        for node in 20..40 {
            state = dynamics.step_at(node, &state, &[]).expect("free step");
        }
        assert!(
            state[1] < -0.01,
            "base did not fall once free: {}",
            state[1]
        );
    }
}
