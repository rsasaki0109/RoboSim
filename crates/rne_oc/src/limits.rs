//! Actuator-limit cost wrapper for the native optimal-control solver.
//!
//! The DDP solver constrains controls with a box ([`crate::DdpConfig`]
//! `control_lower`/`control_upper`) but has no state box. A joint velocity
//! limit is a state bound, so this module enforces it as a soft hinge penalty:
//! inside the limit the cost is untouched, and beyond it the penalty grows
//! quadratically. The penalty is differentiable and its Hessian is diagonal,
//! so it drops straight into the existing DDP backward pass.
//!
//! A plan that stays inside the joint speed limits of the real machine is more
//! likely to transfer to a plant with actuator bandwidth limits than a plan
//! that exploits the optimizer's unbounded joint speeds.

use crate::ddp::{CostDerivatives, CostModel, TerminalDerivatives};

/// Wraps a cost model and adds a hinge penalty on joint velocity limit
/// violations.
///
/// The wrapped state is laid out as `2 * nv` values: the first `nv` are the
/// generalized coordinates and the second `nv` are the generalized velocities.
/// Joint dof `d` therefore has velocity `state[nv + joint_offset + d]`, with
/// `joint_offset` typically `6` to skip a floating base.
#[derive(Clone, Debug)]
pub struct ActuatorLimitCost<C> {
    /// Wrapped running and terminal cost.
    pub inner: C,
    /// Per-dof joint velocity limits in radians per second (or m/s for
    /// prismatic dofs), in generalized-coordinate order.
    pub velocity_limits_rad_s: Vec<f64>,
    /// Penalty weight applied to the squared velocity excess beyond a limit.
    pub velocity_weight: f64,
    /// Number of generalized coordinates; the state has `2 * nv` entries.
    pub nv: usize,
    /// Index of the first joint dof inside the generalized coordinates.
    pub joint_offset: usize,
}

impl<C> ActuatorLimitCost<C> {
    /// Creates the wrapper.
    pub fn new(
        inner: C,
        velocity_limits_rad_s: Vec<f64>,
        velocity_weight: f64,
        nv: usize,
        joint_offset: usize,
    ) -> Self {
        Self {
            inner,
            velocity_limits_rad_s,
            velocity_weight,
            nv,
            joint_offset,
        }
    }

    /// Absolute state indices of the joint velocities paired with their limits.
    fn velocity_slots(&self) -> impl Iterator<Item = (usize, f64)> + '_ {
        let base = self.nv + self.joint_offset;
        self.velocity_limits_rad_s
            .iter()
            .enumerate()
            .map(move |(dof, limit)| (base + dof, *limit))
    }

    /// Soft hinge penalty `0.5 * w * max(0, |v| - limit)^2` over all dofs.
    fn penalty(&self, state: &[f64]) -> f64 {
        self.velocity_slots()
            .map(|(index, limit)| {
                let excess = state[index].abs() - limit;
                if excess > 0.0 {
                    0.5 * self.velocity_weight * excess * excess
                } else {
                    0.0
                }
            })
            .sum()
    }

    /// Adds the penalty gradient and diagonal Hessian into existing
    /// derivatives.
    fn accumulate(&self, state: &[f64], lx: &mut [f64], lxx: &mut [Vec<f64>]) {
        for (index, limit) in self.velocity_slots() {
            let value = state[index];
            let excess = value.abs() - limit;
            if excess > 0.0 {
                lx[index] += self.velocity_weight * excess * value.signum();
                lxx[index][index] += self.velocity_weight;
            }
        }
    }
}

impl<C: CostModel> CostModel for ActuatorLimitCost<C> {
    fn running(&self, node: usize, state: &[f64], control: &[f64]) -> f64 {
        self.inner.running(node, state, control) + self.penalty(state)
    }

    fn terminal(&self, state: &[f64]) -> f64 {
        self.inner.terminal(state) + self.penalty(state)
    }

    fn running_derivatives(&self, node: usize, state: &[f64], control: &[f64]) -> CostDerivatives {
        let mut derivatives = self.inner.running_derivatives(node, state, control);
        self.accumulate(state, &mut derivatives.lx, &mut derivatives.lxx);
        derivatives
    }

    fn terminal_derivatives(&self, state: &[f64]) -> TerminalDerivatives {
        let mut derivatives = self.inner.terminal_derivatives(state);
        self.accumulate(state, &mut derivatives.lx, &mut derivatives.lxx);
        derivatives
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ddp::QuadraticCost;

    fn inner(state_dim: usize, control_dim: usize) -> QuadraticCost {
        QuadraticCost::new(
            vec![0.0; state_dim],
            vec![0.0; control_dim],
            vec![0.0; state_dim],
        )
    }

    #[test]
    fn inside_limit_is_free() {
        let cost = ActuatorLimitCost::new(inner(16, 4), vec![5.0, 5.0], 10.0, 8, 6);
        let mut state = vec![0.0; 16];
        state[8 + 6] = 4.0;
        state[8 + 7] = -4.9;
        assert_eq!(cost.running(0, &state, &[0.0; 4]), 0.0);
    }

    #[test]
    fn violation_penalizes_and_gradients() {
        let cost = ActuatorLimitCost::new(inner(16, 4), vec![5.0], 2.0, 8, 6);
        let mut state = vec![0.0; 16];
        state[8 + 6] = 8.0;
        let expected = 0.5 * 2.0 * 3.0 * 3.0;
        assert!((cost.running(0, &state, &[0.0; 4]) - expected).abs() < 1.0e-12);
        let derivatives = cost.running_derivatives(0, &state, &[0.0; 4]);
        assert!((derivatives.lx[8 + 6] - 2.0 * 3.0).abs() < 1.0e-12);
        assert!((derivatives.lxx[8 + 6][8 + 6] - 2.0).abs() < 1.0e-12);
        assert_eq!(derivatives.lxx[8 + 7][8 + 7], 0.0);
    }
}
