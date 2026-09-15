//! Differential dynamic programming over a discrete shooting problem.

use crate::matrix::{
    add_diagonal, identity, invert, mat_add, mat_mul, mat_scale, mat_sub, mat_transpose, mat_vec,
    symmetrize,
};
use thiserror::Error;

/// Error returned by the optimal-control solvers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum OcError {
    /// A vector or matrix had an inconsistent dimension.
    #[error("dimension mismatch in {0}")]
    Dimension(&'static str),
    /// The discrete dynamics could not be evaluated.
    #[error("dynamics evaluation failed")]
    Dynamics,
    /// The regularized control Hessian was singular.
    #[error("the regularized control Hessian is singular")]
    Singular,
    /// The solver needs at least one control node.
    #[error("the shooting problem needs at least one node")]
    EmptyHorizon,
}

/// A discrete-time dynamics model `x_{k+1} = f(x_k, u_k)`.
pub trait DiscreteDynamics {
    /// State dimension.
    fn state_dim(&self) -> usize;
    /// Control dimension.
    fn control_dim(&self) -> usize;
    /// Advances the state by one step.
    fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError>;
}

/// A node-dependent dynamics model for a shooting problem.
///
/// Time-invariant [`DiscreteDynamics`] implement this through a blanket impl;
/// contact sequences implement it directly so the active contact set can change
/// along the horizon.
pub trait ShootingDynamics {
    /// State dimension.
    fn state_dim(&self) -> usize;
    /// Control dimension.
    fn control_dim(&self) -> usize;
    /// Advances the state at node `node` by one step.
    fn step_at(&self, node: usize, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError>;
}

impl<T: DiscreteDynamics + ?Sized> ShootingDynamics for T {
    fn state_dim(&self) -> usize {
        DiscreteDynamics::state_dim(self)
    }

    fn control_dim(&self) -> usize {
        DiscreteDynamics::control_dim(self)
    }

    fn step_at(&self, _node: usize, state: &[f64], control: &[f64]) -> Result<Vec<f64>, OcError> {
        self.step(state, control)
    }
}

/// Derivatives of a running cost.
#[derive(Clone, Debug)]
pub struct CostDerivatives {
    /// Gradient with respect to the state.
    pub lx: Vec<f64>,
    /// Gradient with respect to the control.
    pub lu: Vec<f64>,
    /// State Hessian.
    pub lxx: Vec<Vec<f64>>,
    /// Cross Hessian (control rows, state columns).
    pub lux: Vec<Vec<f64>>,
    /// Control Hessian.
    pub luu: Vec<Vec<f64>>,
}

/// Derivatives of a terminal cost.
#[derive(Clone, Debug)]
pub struct TerminalDerivatives {
    /// Gradient with respect to the state.
    pub lx: Vec<f64>,
    /// State Hessian.
    pub lxx: Vec<Vec<f64>>,
}

/// A running and terminal cost model.
pub trait CostModel {
    /// Running cost `l_k(x, u)` at node `node`.
    fn running(&self, node: usize, state: &[f64], control: &[f64]) -> f64;
    /// Terminal cost `lf(x)`.
    fn terminal(&self, state: &[f64]) -> f64;
    /// Derivatives of the running cost at node `node`.
    fn running_derivatives(&self, node: usize, state: &[f64], control: &[f64]) -> CostDerivatives;
    /// Derivatives of the terminal cost.
    fn terminal_derivatives(&self, state: &[f64]) -> TerminalDerivatives;
}

/// Diagonal quadratic running and terminal cost.
#[derive(Clone, Debug)]
pub struct QuadraticCost {
    /// Running state weights (diagonal).
    pub state_weights: Vec<f64>,
    /// Running control weights (diagonal).
    pub control_weights: Vec<f64>,
    /// Terminal state weights (diagonal).
    pub terminal_weights: Vec<f64>,
    /// Desired state.
    pub state_reference: Vec<f64>,
    /// Desired control.
    pub control_reference: Vec<f64>,
    /// Multiplier on the running cost, such as the integration step.
    pub running_scale: f64,
}

impl QuadraticCost {
    /// Creates a diagonal quadratic cost; references default to zero.
    pub fn new(
        state_weights: Vec<f64>,
        control_weights: Vec<f64>,
        terminal_weights: Vec<f64>,
    ) -> Self {
        let state_reference = vec![0.0; state_weights.len()];
        let control_reference = vec![0.0; control_weights.len()];
        Self {
            state_weights,
            control_weights,
            terminal_weights,
            state_reference,
            control_reference,
            running_scale: 1.0,
        }
    }
}

impl CostModel for QuadraticCost {
    fn running(&self, _node: usize, state: &[f64], control: &[f64]) -> f64 {
        let state_cost: f64 = self
            .state_weights
            .iter()
            .zip(state)
            .zip(&self.state_reference)
            .map(|((weight, value), reference)| weight * (value - reference).powi(2))
            .sum();
        let control_cost: f64 = self
            .control_weights
            .iter()
            .zip(control)
            .zip(&self.control_reference)
            .map(|((weight, value), reference)| weight * (value - reference).powi(2))
            .sum();
        0.5 * self.running_scale * (state_cost + control_cost)
    }

    fn terminal(&self, state: &[f64]) -> f64 {
        let state_cost: f64 = self
            .terminal_weights
            .iter()
            .zip(state)
            .zip(&self.state_reference)
            .map(|((weight, value), reference)| weight * (value - reference).powi(2))
            .sum();
        0.5 * state_cost
    }

    fn running_derivatives(&self, _node: usize, state: &[f64], control: &[f64]) -> CostDerivatives {
        let lx = self
            .state_weights
            .iter()
            .zip(state)
            .zip(&self.state_reference)
            .map(|((weight, value), reference)| self.running_scale * weight * (value - reference))
            .collect();
        let lu = self
            .control_weights
            .iter()
            .zip(control)
            .zip(&self.control_reference)
            .map(|((weight, value), reference)| self.running_scale * weight * (value - reference))
            .collect();
        CostDerivatives {
            lx,
            lu,
            lxx: diagonal(&self.state_weights, self.running_scale),
            lux: vec![vec![0.0; state.len()]; control.len()],
            luu: diagonal(&self.control_weights, self.running_scale),
        }
    }

    fn terminal_derivatives(&self, state: &[f64]) -> TerminalDerivatives {
        let lx = self
            .terminal_weights
            .iter()
            .zip(state)
            .zip(&self.state_reference)
            .map(|((weight, value), reference)| weight * (value - reference))
            .collect();
        TerminalDerivatives {
            lx,
            lxx: diagonal(&self.terminal_weights, 1.0),
        }
    }
}

/// A per-node schedule of quadratic running costs with one terminal cost.
///
/// This is the Crocoddyl action-model analogue for a fixed contact sequence:
/// each phase can pull toward a different reference, for example a crouch
/// during a loading phase and a target apex during flight.
#[derive(Clone, Debug)]
pub struct PhaseCostSchedule {
    /// Running cost per node.
    pub running: Vec<QuadraticCost>,
    /// Terminal cost.
    pub terminal: QuadraticCost,
}

impl PhaseCostSchedule {
    fn running_cost(&self, node: usize) -> &QuadraticCost {
        let index = node.min(self.running.len().saturating_sub(1));
        &self.running[index]
    }
}

impl CostModel for PhaseCostSchedule {
    fn running(&self, node: usize, state: &[f64], control: &[f64]) -> f64 {
        self.running_cost(node).running(node, state, control)
    }

    fn terminal(&self, state: &[f64]) -> f64 {
        self.terminal.terminal(state)
    }

    fn running_derivatives(&self, node: usize, state: &[f64], control: &[f64]) -> CostDerivatives {
        self.running_cost(node)
            .running_derivatives(node, state, control)
    }

    fn terminal_derivatives(&self, state: &[f64]) -> TerminalDerivatives {
        self.terminal.terminal_derivatives(state)
    }
}

fn diagonal(weights: &[f64], scale: f64) -> Vec<Vec<f64>> {
    let mut out = identity(weights.len());
    for (index, weight) in weights.iter().enumerate() {
        out[index][index] = weight * scale;
    }
    out
}

/// Derivatives of the discrete dynamics: `fx = d f / d x`, `fu = d f / d u`.
#[derive(Clone, Debug)]
pub struct DynamicsDerivatives {
    /// State Jacobian (state rows, state columns).
    pub fx: Vec<Vec<f64>>,
    /// Control Jacobian (state rows, control columns).
    pub fu: Vec<Vec<f64>>,
}

/// Central-difference derivatives of the dynamics.
pub fn dynamics_derivatives(
    dynamics: &dyn ShootingDynamics,
    node: usize,
    state: &[f64],
    control: &[f64],
    epsilon: f64,
) -> Result<DynamicsDerivatives, OcError> {
    let nx = dynamics.state_dim();
    let nu = dynamics.control_dim();
    let mut fx = vec![vec![0.0; nx]; nx];
    let mut fu = vec![vec![0.0; nu]; nx];
    for column in 0..nx {
        let mut plus = state.to_vec();
        let mut minus = state.to_vec();
        plus[column] += epsilon;
        minus[column] -= epsilon;
        let f_plus = dynamics.step_at(node, &plus, control)?;
        let f_minus = dynamics.step_at(node, &minus, control)?;
        for row in 0..nx {
            fx[row][column] = (f_plus[row] - f_minus[row]) / (2.0 * epsilon);
        }
    }
    for column in 0..nu {
        let mut plus = control.to_vec();
        let mut minus = control.to_vec();
        plus[column] += epsilon;
        minus[column] -= epsilon;
        let f_plus = dynamics.step_at(node, state, &plus)?;
        let f_minus = dynamics.step_at(node, state, &minus)?;
        for row in 0..nx {
            fu[row][column] = (f_plus[row] - f_minus[row]) / (2.0 * epsilon);
        }
    }
    Ok(DynamicsDerivatives { fx, fu })
}

/// Solver configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DdpConfig {
    /// Maximum number of DDP iterations.
    pub max_iterations: usize,
    /// Convergence tolerance on the cost decrease.
    pub tolerance: f64,
    /// Initial Levenberg-Marquardt regularization.
    pub initial_regularization: f64,
    /// Minimum regularization.
    pub min_regularization: f64,
    /// Maximum regularization before giving up.
    pub max_regularization: f64,
    /// Multiplicative regularization update.
    pub regularization_factor: f64,
    /// Number of line-search halvings.
    pub line_search_steps: usize,
    /// Central-difference step.
    pub epsilon: f64,
    /// Keep dynamics gaps open in the forward pass (FDDP-style infeasible warm start).
    pub keep_gaps_open: bool,
}

impl Default for DdpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1.0e-8,
            initial_regularization: 1.0e-6,
            min_regularization: 1.0e-8,
            max_regularization: 1.0e8,
            regularization_factor: 2.0,
            line_search_steps: 12,
            epsilon: 1.0e-6,
            keep_gaps_open: false,
        }
    }
}

/// Result of a DDP solve.
#[derive(Clone, Debug, PartialEq)]
pub struct DdpSolution {
    /// State trajectory, length `horizon + 1`.
    pub states: Vec<Vec<f64>>,
    /// Control trajectory, length `horizon`.
    pub controls: Vec<Vec<f64>>,
    /// Final trajectory cost.
    pub cost: f64,
    /// Number of iterations executed.
    pub iterations: usize,
    /// Whether the cost decrease fell below the tolerance.
    pub converged: bool,
}

fn vec_add(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
}

fn vec_sub(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x - y).collect()
}

fn vec_scale(a: &[f64], factor: f64) -> Vec<f64> {
    a.iter().map(|value| value * factor).collect()
}

fn vec_neg(a: &[f64]) -> Vec<f64> {
    a.iter().map(|value| -value).collect()
}

/// Solves the shooting problem with differential dynamic programming.
pub fn solve(
    dynamics: &dyn ShootingDynamics,
    cost: &dyn CostModel,
    initial_states: &[Vec<f64>],
    initial_controls: &[Vec<f64>],
    config: &DdpConfig,
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

    let trajectory_cost = |states: &[Vec<f64>], controls: &[Vec<f64>]| -> f64 {
        let running: f64 = (0..horizon)
            .map(|k| cost.running(k, &states[k], &controls[k]))
            .sum();
        running + cost.terminal(&states[horizon])
    };

    let mut states = initial_states.to_vec();
    let mut controls = initial_controls.to_vec();
    let mut regularization = config.initial_regularization;
    let mut current_cost = trajectory_cost(&states, &controls);
    let mut converged = false;
    let mut iterations = 0;

    for iteration in 0..config.max_iterations {
        iterations = iteration + 1;

        let mut value_x = cost.terminal_derivatives(&states[horizon]).lx;
        let mut value_xx = cost.terminal_derivatives(&states[horizon]).lxx;
        let mut feedforward = vec![vec![0.0; nu]; horizon];
        let mut feedback = vec![vec![vec![0.0; nx]; nu]; horizon];
        let mut failed = false;

        for k in (0..horizon).rev() {
            let derivatives =
                match dynamics_derivatives(dynamics, k, &states[k], &controls[k], config.epsilon) {
                    Ok(derivatives) => derivatives,
                    Err(_) => {
                        failed = true;
                        break;
                    }
                };
            let fx = derivatives.fx;
            let fu = derivatives.fu;
            let derivative = cost.running_derivatives(k, &states[k], &controls[k]);

            let fx_t = mat_transpose(&fx);
            let fu_t = mat_transpose(&fu);
            let qx = vec_add(&derivative.lx, &mat_vec(&fx_t, &value_x));
            let qu = vec_add(&derivative.lu, &mat_vec(&fu_t, &value_x));
            let qxx = mat_add(&derivative.lxx, &mat_mul(&fx_t, &mat_mul(&value_xx, &fx)));
            let qux = mat_add(&derivative.lux, &mat_mul(&fu_t, &mat_mul(&value_xx, &fx)));
            let mut quu = mat_add(&derivative.luu, &mat_mul(&fu_t, &mat_mul(&value_xx, &fu)));
            add_diagonal(&mut quu, regularization);

            let Some(quu_inverse) = invert(&quu) else {
                failed = true;
                break;
            };
            let k_gain = vec_neg(&mat_vec(&quu_inverse, &qu));
            let k_feedback = mat_scale(&mat_mul(&quu_inverse, &qux), -1.0);

            let qux_t = mat_transpose(&qux);
            let value_x_new = vec_add(&qx, &mat_vec(&qux_t, &k_gain));
            let value_xx_new = symmetrize(&mat_sub(
                &qxx,
                &mat_mul(&qux_t, &mat_mul(&quu_inverse, &qux)),
            ));

            feedforward[k] = k_gain;
            feedback[k] = k_feedback;
            value_x = value_x_new;
            value_xx = value_xx_new;
        }

        if failed {
            regularization *= config.regularization_factor;
            if regularization > config.max_regularization {
                break;
            }
            continue;
        }

        let gaps: Vec<Vec<f64>> = if config.keep_gaps_open {
            let mut gaps = Vec::with_capacity(horizon);
            let mut ok = true;
            for k in 0..horizon {
                match dynamics.step_at(k, &states[k], &controls[k]) {
                    Ok(predicted) => gaps.push(vec_sub(&states[k + 1], &predicted)),
                    Err(_) => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok {
                gaps
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        };

        let mut improved = false;
        for step in 0..config.line_search_steps {
            let alpha = 0.5_f64.powi(step as i32);
            let mut candidate_states = vec![vec![0.0; nx]; horizon + 1];
            let mut candidate_controls = vec![vec![0.0; nu]; horizon];
            candidate_states[0] = states[0].clone();
            let mut candidate_cost = 0.0;
            let mut valid = true;
            for k in 0..horizon {
                let dx = vec_sub(&candidate_states[k], &states[k]);
                let du = vec_add(
                    &vec_scale(&feedforward[k], alpha),
                    &mat_vec(&feedback[k], &dx),
                );
                let u = vec_add(&controls[k], &du);
                let mut next = match dynamics.step_at(k, &candidate_states[k], &u) {
                    Ok(next) => next,
                    Err(_) => {
                        valid = false;
                        break;
                    }
                };
                if config.keep_gaps_open && !gaps.is_empty() {
                    next = vec_add(&next, &vec_scale(&gaps[k], 1.0 - alpha));
                }
                candidate_cost += cost.running(k, &candidate_states[k], &u);
                candidate_controls[k] = u;
                candidate_states[k + 1] = next;
            }
            if !valid {
                continue;
            }
            candidate_cost += cost.terminal(&candidate_states[horizon]);

            if candidate_cost.is_finite() && candidate_cost < current_cost {
                let decrease = current_cost - candidate_cost;
                states = candidate_states;
                controls = candidate_controls;
                current_cost = candidate_cost;
                improved = true;
                regularization = (regularization * 0.5).max(config.min_regularization);
                if decrease < config.tolerance {
                    converged = true;
                }
                break;
            }
        }

        if !improved {
            regularization *= config.regularization_factor;
            if regularization > config.max_regularization {
                break;
            }
        }
        if converged {
            break;
        }
    }

    // FDDP uses opened gaps as a warm start. Project the final controls onto a
    // feasible rollout and polish with standard DDP so the returned trajectory
    // satisfies the dynamics exactly at a local optimum.
    if config.keep_gaps_open {
        let mut feasible = vec![vec![0.0; nx]; horizon + 1];
        feasible[0] = states[0].clone();
        let mut ok = true;
        for k in 0..horizon {
            match dynamics.step_at(k, &feasible[k], &controls[k]) {
                Ok(next) => feasible[k + 1] = next,
                Err(_) => {
                    ok = false;
                    break;
                }
            }
        }
        if ok {
            let mut polish = *config;
            polish.keep_gaps_open = false;
            if let Ok(solution) = solve(dynamics, cost, &feasible, &controls, &polish) {
                return Ok(solution);
            }
        }
    }

    Ok(DdpSolution {
        states,
        controls,
        cost: current_cost,
        iterations,
        converged,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

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
            let (position, velocity) = (state[0], state[1]);
            let acceleration = control[0];
            Ok(vec![
                position + velocity * self.dt + 0.5 * acceleration * self.dt * self.dt,
                velocity + acceleration * self.dt,
            ])
        }
    }

    #[test]
    fn double_integrator_reaches_the_target() {
        let dt = 0.1;
        let horizon = 60;
        let dynamics = DoubleIntegrator { dt };
        let mut cost = QuadraticCost::new(vec![0.01, 0.01], vec![0.001], vec![200.0, 20.0]);
        cost.state_reference = vec![1.0, 0.0];
        cost.running_scale = dt;
        let states = vec![vec![0.0, 0.0]; horizon + 1];
        let controls = vec![vec![0.0]; horizon];
        let config = DdpConfig {
            max_iterations: 200,
            ..DdpConfig::default()
        };
        let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
        let final_position = solution.states[horizon][0];
        let final_velocity = solution.states[horizon][1];
        assert!(
            (final_position - 1.0).abs() < 0.05,
            "position {final_position}"
        );
        assert!(final_velocity.abs() < 0.05, "velocity {final_velocity}");
        assert!(solution.cost < 0.01, "cost {}", solution.cost);
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn fddp_converges_from_an_infeasible_initial_trajectory() {
        let dynamics = DoubleIntegrator { dt: 0.1 };
        let mut cost = QuadraticCost::new(vec![0.01, 0.01], vec![0.001], vec![200.0, 20.0]);
        cost.state_reference = vec![1.0, 0.0];
        cost.running_scale = 0.1;
        let horizon = 60;
        let mut states = Vec::new();
        for k in 0..=horizon {
            let t = k as f64 / horizon as f64;
            states.push(vec![t, 0.0]);
        }
        let controls = vec![vec![0.0]; horizon];
        let config = DdpConfig {
            max_iterations: 300,
            keep_gaps_open: true,
            ..DdpConfig::default()
        };
        let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
        let final_position = solution.states[horizon][0];
        assert!(
            (final_position - 1.0).abs() < 0.05,
            "position {final_position}"
        );
        let mut max_gap: f64 = 0.0;
        for k in 0..horizon {
            let predicted = dynamics
                .step(&solution.states[k], &solution.controls[k])
                .expect("step");
            for index in 0..2 {
                max_gap = max_gap.max((solution.states[k + 1][index] - predicted[index]).abs());
            }
        }
        assert!(max_gap < 1.0e-3, "unclosed gap {max_gap}");
    }

    #[test]
    fn phase_schedule_dispatches_per_node_references() {
        let unit = |reference: f64| {
            let mut cost = QuadraticCost::new(vec![1.0, 0.0], vec![0.0], vec![0.0]);
            cost.state_reference = vec![reference, 0.0];
            cost
        };
        let running: Vec<QuadraticCost> = (0..10)
            .map(|node| unit(if node < 5 { -1.0 } else { 1.0 }))
            .collect();
        let terminal = QuadraticCost::new(vec![0.0, 0.0], vec![0.0], vec![0.0]);
        let schedule = PhaseCostSchedule { running, terminal };
        let state = [1.0, 0.0];
        // Node 0 targets -1, node 9 targets +1.
        assert!((schedule.running(0, &state, &[]) - 2.0).abs() < 1.0e-12);
        assert!((schedule.running(9, &state, &[]) - 0.0).abs() < 1.0e-12);
    }

    #[test]
    fn double_integrator_is_deterministic() {
        let dynamics = DoubleIntegrator { dt: 0.1 };
        let mut cost = QuadraticCost::new(vec![0.01, 0.01], vec![0.001], vec![200.0, 20.0]);
        cost.state_reference = vec![1.0, 0.0];
        let states = vec![vec![0.0, 0.0]; 31];
        let controls = vec![vec![0.0]; 30];
        let config = DdpConfig::default();
        let first = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
        let second = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
        assert_eq!(first.states, second.states);
        assert_eq!(first.controls, second.controls);
    }
}
