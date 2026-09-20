//! Direct multiple shooting over a discrete trajectory.
//!
//! Unlike single-shooting DDP, the whole state trajectory is a decision
//! variable and the dynamics enters as a defect constraint
//! `x_{k+1} - f(x_k, u_k) = 0`. A Gauss-Seidel sweep drives the defects down:
//! each node's state and control are corrected by a local Gauss-Newton step
//! while its neighbours are held fixed, so the initial state trajectory does
//! not have to be a rollout. This is what makes a non-smooth contact transition
//! tractable: an infeasible warm start is repaired by moving the states, not by
//! replaying unstable controls.

use crate::ddp::{
    dynamics_derivatives, CostModel, DdpSolution, DynamicsDerivatives, OcError, ShootingDynamics,
};
use crate::matrix::{invert, mat_vec};

/// Configuration for the multiple-shooting solver.
#[derive(Clone, Debug, PartialEq)]
pub struct MultipleShootingConfig {
    /// Outer sweeps over the horizon.
    pub max_iterations: usize,
    /// Stop when the largest defect falls below this.
    pub tolerance: f64,
    /// Initial defect penalty.
    pub initial_penalty: f64,
    /// Multiplier applied to the penalty each sweep.
    pub penalty_growth: f64,
    /// Largest penalty.
    pub max_penalty: f64,
    /// Levenberg-Marquardt damping on the local Gauss-Newton step.
    pub regularization: f64,
    /// Central-difference step when a model has no analytic derivatives.
    pub epsilon: f64,
    /// Optional per-control lower bounds.
    pub control_lower: Option<Vec<f64>>,
    /// Optional per-control upper bounds.
    pub control_upper: Option<Vec<f64>>,
}

impl Default for MultipleShootingConfig {
    fn default() -> Self {
        Self {
            max_iterations: 1500,
            tolerance: 1.0e-6,
            initial_penalty: 1.0e2,
            penalty_growth: 1.5,
            max_penalty: 1.0e9,
            regularization: 1.0e-6,
            epsilon: 1.0e-6,
            control_lower: None,
            control_upper: None,
        }
    }
}

fn clamp_control(control: &[f64], config: &MultipleShootingConfig) -> Vec<f64> {
    match (&config.control_lower, &config.control_upper) {
        (Some(lower), Some(upper)) => control
            .iter()
            .enumerate()
            .map(|(index, value)| {
                let low = lower.get(index).copied().unwrap_or(f64::NEG_INFINITY);
                let high = upper.get(index).copied().unwrap_or(f64::INFINITY);
                value.clamp(low, high)
            })
            .collect(),
        _ => control.to_vec(),
    }
}

fn derivatives_at(
    dynamics: &dyn ShootingDynamics,
    node: usize,
    state: &[f64],
    control: &[f64],
    epsilon: f64,
) -> Result<DynamicsDerivatives, OcError> {
    match dynamics.analytic_derivatives(node, state, control) {
        Some(result) => result,
        None => dynamics_derivatives(dynamics, node, state, control, epsilon),
    }
}

/// Largest single-state defect across the horizon for a trajectory.
pub fn max_defect(
    dynamics: &dyn ShootingDynamics,
    states: &[Vec<f64>],
    controls: &[Vec<f64>],
) -> f64 {
    let horizon = controls.len();
    let mut maximum = 0.0_f64;
    for k in 0..horizon {
        if let Ok(predicted) = dynamics.step_at(k, &states[k], &controls[k]) {
            for (a, b) in states[k + 1].iter().zip(&predicted) {
                maximum = maximum.max((a - b).abs());
            }
        }
    }
    maximum
}

#[allow(clippy::too_many_arguments, clippy::needless_range_loop)]
fn local_step(
    dynamics: &dyn ShootingDynamics,
    cost: &dyn CostModel,
    node: usize,
    states: &[Vec<f64>],
    controls: &[Vec<f64>],
    penalty: f64,
    regularization: f64,
    epsilon: f64,
) -> Result<(Vec<f64>, Vec<f64>), OcError> {
    let nx = dynamics.state_dim();
    let nu = dynamics.control_dim();
    let x = &states[node];
    let u = &controls[node];
    let previous = &states[node - 1];
    let previous_u = &controls[node - 1];
    let next = &states[node + 1];

    let derivatives = derivatives_at(dynamics, node, x, u, epsilon)?;
    let predicted = dynamics.step_at(node, x, u)?;
    let previous_predicted = dynamics.step_at(node - 1, previous, previous_u)?;
    // `r_prev` is affine in the node state (derivative identity); `r_cur`
    // depends on the node state and control through `f`.
    let r_prev: Vec<f64> = (0..nx).map(|i| x[i] - previous_predicted[i]).collect();
    let r_cur: Vec<f64> = (0..nx).map(|i| next[i] - predicted[i]).collect();
    let running = cost.running_derivatives(node, x, u);

    let mut gradient = vec![0.0; nx + nu];
    for i in 0..nx {
        let mut fx_t_r = 0.0;
        for row in 0..nx {
            fx_t_r += derivatives.fx[row][i] * r_cur[row];
        }
        gradient[i] = running.lx[i] + penalty * (r_prev[i] - fx_t_r);
    }
    for j in 0..nu {
        let mut fu_t_r = 0.0;
        for row in 0..nx {
            fu_t_r += derivatives.fu[row][j] * r_cur[row];
        }
        gradient[nx + j] = running.lu[j] - penalty * fu_t_r;
    }

    // Gauss-Newton Hessian of the local penalty plus the cost Hessian.
    let mut hessian = vec![vec![0.0; nx + nu]; nx + nu];
    for i in 0..nx {
        for j in 0..nx {
            let mut fx_fx = 0.0;
            for row in 0..nx {
                fx_fx += derivatives.fx[row][i] * derivatives.fx[row][j];
            }
            hessian[i][j] = running.lxx[i][j] + penalty * (if i == j { 1.0 } else { 0.0 } + fx_fx);
        }
    }
    for j in 0..nu {
        for l in 0..nu {
            let mut fu_fu = 0.0;
            for row in 0..nx {
                fu_fu += derivatives.fu[row][j] * derivatives.fu[row][l];
            }
            hessian[nx + j][nx + l] = running.luu[j][l] + penalty * fu_fu;
        }
    }
    for i in 0..nx {
        for j in 0..nu {
            let mut fu_fx = 0.0;
            for row in 0..nx {
                fu_fx += derivatives.fu[row][j] * derivatives.fx[row][i];
            }
            let value = running.lux[j][i] + penalty * fu_fx;
            hessian[nx + j][i] = value;
            hessian[i][nx + j] = value;
        }
    }
    for (index, row) in hessian.iter_mut().enumerate() {
        row[index] += regularization;
    }

    let inverse = invert(&hessian).ok_or(OcError::Singular)?;
    let step = mat_vec(&inverse, &gradient);
    let mut delta_x = vec![0.0; nx];
    let mut delta_u = vec![0.0; nu];
    for i in 0..nx {
        delta_x[i] = -step[i];
    }
    for j in 0..nu {
        delta_u[j] = -step[nx + j];
    }
    Ok((delta_x, delta_u))
}

/// Solves the shooting problem by direct multiple shooting.
///
/// Returns a [`DdpSolution`] whose `cost` is the true trajectory cost (without
/// the defect penalty) and whose states satisfy the dynamics to the residual
/// defect size.
#[allow(clippy::too_many_arguments)]
pub fn solve_multiple_shooting(
    dynamics: &dyn ShootingDynamics,
    cost: &dyn CostModel,
    initial_states: &[Vec<f64>],
    initial_controls: &[Vec<f64>],
    config: &MultipleShootingConfig,
) -> Result<DdpSolution, OcError> {
    let horizon = initial_controls.len();
    if horizon == 0 {
        return Err(OcError::EmptyHorizon);
    }
    if initial_states.len() != horizon + 1 {
        return Err(OcError::Dimension("initial state trajectory"));
    }
    let nx = dynamics.state_dim();
    let nu = dynamics.control_dim();
    if initial_states.iter().any(|state| state.len() != nx)
        || initial_controls.iter().any(|control| control.len() != nu)
    {
        return Err(OcError::Dimension("multiple-shooting state or control"));
    }

    let mut xs = initial_states.to_vec();
    let mut us = initial_controls.to_vec();
    let mut penalty = config.initial_penalty;
    let mut converged = false;
    let mut iterations = 0;

    for iteration in 0..config.max_iterations {
        iterations = iteration + 1;
        // Gauss-Seidel sweep: correct each node with its neighbours fixed.
        for node in 1..horizon {
            let (delta_x, delta_u) = match local_step(
                dynamics,
                cost,
                node,
                &xs,
                &us,
                penalty,
                config.regularization,
                config.epsilon,
            ) {
                Ok(step) => step,
                Err(_) => continue,
            };
            // Reject a step that leaves the node outside a sane range: the
            // local Gauss-Newton can overshoot when the controls are bounded
            // and the defects cannot be met exactly.
            let mut trial_x = xs[node].clone();
            for i in 0..nx {
                trial_x[i] += delta_x[i];
            }
            if trial_x
                .iter()
                .all(|value| value.is_finite() && value.abs() < 1.0e6)
            {
                xs[node] = trial_x;
                for j in 0..nu {
                    us[node][j] += delta_u[j];
                }
                us[node] = clamp_control(&us[node], config);
            }
        }
        let defect = max_defect(dynamics, &xs, &us);
        if defect < config.tolerance {
            converged = true;
            break;
        }
        // Raise the penalty periodically so the defects keep shrinking.
        if iteration % 4 == 3 {
            penalty = (penalty * config.penalty_growth).min(config.max_penalty);
        }
    }

    let running: f64 = (0..horizon).map(|k| cost.running(k, &xs[k], &us[k])).sum();
    let final_cost = running + cost.terminal(&xs[horizon]);
    Ok(DdpSolution {
        states: xs,
        controls: us,
        cost: final_cost,
        iterations,
        converged,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ddp::{DiscreteDynamics, QuadraticCost};

    struct DoubleIntegrator {
        dt: f64,
    }

    impl DiscreteDynamics for DoubleIntegrator {
        fn state_dim(&self) -> usize {
            2
        }

        fn control_dim(&self) -> usize {
            1
        }

        fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
            Ok(vec![
                state[0] + state[1] * self.dt + 0.5 * control[0] * self.dt * self.dt,
                state[1] + control[0] * self.dt,
            ])
        }
    }

    #[test]
    fn repairs_an_infeasible_warm_start() {
        let dynamics = DoubleIntegrator { dt: 0.1 };
        let mut cost = QuadraticCost::new(vec![0.01, 0.01], vec![0.001], vec![200.0, 20.0]);
        cost.state_reference = vec![1.0, 0.0];
        cost.running_scale = 0.1;
        let horizon = 40;
        // A deliberately inconsistent warm start that is not a rollout.
        let mut states = Vec::new();
        for k in 0..=horizon {
            let t = k as f64 / horizon as f64;
            states.push(vec![t, 0.5]);
        }
        let controls = vec![vec![0.0]; horizon];
        let config = MultipleShootingConfig::default();
        let solution =
            solve_multiple_shooting(&dynamics, &cost, &states, &controls, &config).expect("solve");
        let defect = max_defect(&dynamics, &solution.states, &solution.controls);
        assert!(defect < 1.0e-3, "defect {defect}");
    }
}
