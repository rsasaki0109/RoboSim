//! Contact-constrained articulated dynamics and contact sequences.

use crate::ddp::{DiscreteDynamics, OcError, ShootingDynamics};
use rne_dynamics::{constrained_forward_dynamics, ArticulatedModel, ContactSpec};

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
    for index in 0..nv {
        next[index] = q[index] + qd[index] * dt + 0.5 * acceleration[index] * dt * dt;
        next[nv + index] = qd[index] + acceleration[index] * dt;
    }
    Ok(next)
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
/// Each node uses the active contacts of its phase. Impact reset maps at phase
/// boundaries are not applied yet, so a sequence must be initialized with a
/// configuration whose velocities are consistent with the new contact set.
pub struct ContactSequenceDynamics<'a> {
    /// Model to integrate.
    pub model: &'a ArticulatedModel,
    /// Integration step in seconds.
    pub step_time_s: f64,
    contacts_per_node: Vec<Vec<ContactSpec>>,
}

impl<'a> ContactSequenceDynamics<'a> {
    /// Expands a phase list into a per-node contact schedule.
    pub fn new(model: &'a ArticulatedModel, step_time_s: f64, phases: &[ContactPhase]) -> Self {
        let mut contacts_per_node = Vec::new();
        for phase in phases {
            for _ in 0..phase.steps {
                contacts_per_node.push(phase.contacts.clone());
            }
        }
        Self {
            model,
            step_time_s,
            contacts_per_node,
        }
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
        integrate(self.model, self.step_time_s, state, control, contacts)
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
