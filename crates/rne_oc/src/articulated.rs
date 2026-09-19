//! Discrete dynamics for a floating- or fixed-base articulated model.

use crate::ddp::{DiscreteDynamics, OcError};
use rne_dynamics::{forward_dynamics, integrate_configuration, ArticulatedModel};

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
