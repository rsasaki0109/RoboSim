//! 2D pose graph with Gauss-Newton optimization.
//!
//! A pose graph stores robot poses as nodes connected by relative-pose
//! constraints. Sequential constraints come from odometry; loop-closure
//! constraints come from scan matching. Gauss-Newton optimization distributes
//! loop-closure error across the graph to remove accumulated drift.
//!
//! The implementation mirrors the information-form treatment used by
//! `g2o` / `GTSAM` at a scale appropriate for the engine: anchor node zero,
//! linearize each constraint, and solve the reduced sparse normal equations by
//! Cholesky.

use rne_nav::Pose2d;
use serde::{Deserialize, Serialize};

/// A constraint between two pose nodes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseGraphEdge {
    /// Source node index.
    pub from: usize,
    /// Target node index.
    pub to: usize,
    /// Measured `from → to` relative pose.
    pub measurement: Pose2d,
    /// Diagonal information (inverse covariance) for `(x, y, yaw)`.
    pub information: (f64, f64, f64),
    /// Whether this edge closes a loop.
    pub loop_closure: bool,
}

impl PoseGraphEdge {
    /// Creates an odometry edge with unit information.
    pub fn odometry(from: usize, to: usize, measurement: Pose2d) -> Self {
        Self {
            from,
            to,
            measurement,
            information: (1.0, 1.0, 1.0),
            loop_closure: false,
        }
    }

    /// Creates a loop-closure edge with the given information weights.
    pub fn loop_closure(
        from: usize,
        to: usize,
        measurement: Pose2d,
        information: (f64, f64, f64),
    ) -> Self {
        Self {
            from,
            to,
            measurement,
            information,
            loop_closure: true,
        }
    }
}

/// Error returned by pose-graph construction or optimization.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PoseGraphError {
    /// The graph has no nodes.
    #[error("pose graph has no nodes")]
    EmptyGraph,
    /// An edge references a node index outside the graph.
    #[error("edge references node {0} which does not exist")]
    UnknownNode(usize),
    /// The graph is disconnected from the anchor node.
    #[error("pose graph is disconnected from the anchor")]
    Disconnected,
    /// A pose or measurement contained a non-finite value.
    #[error("pose graph input must be finite")]
    NonFinite,
    /// The normal equations were singular even after damping.
    #[error("pose graph normal equations are singular")]
    SingularSystem,
    /// The iteration budget was too small to run.
    #[error("pose graph optimization requires at least one iteration")]
    InvalidIterations,
}

/// A 2D pose graph.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseGraph {
    nodes: Vec<Pose2d>,
    edges: Vec<PoseGraphEdge>,
}

impl PoseGraph {
    /// Creates an empty graph.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }

    /// Appends a node and returns its index.
    pub fn add_node(&mut self, pose: Pose2d) -> usize {
        self.nodes.push(pose);
        self.nodes.len() - 1
    }

    /// Appends an edge.
    pub fn add_edge(&mut self, edge: PoseGraphEdge) {
        self.edges.push(edge);
    }

    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of edges.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Pose of a node.
    pub fn node(&self, index: usize) -> Option<Pose2d> {
        self.nodes.get(index).copied()
    }

    /// All node poses.
    pub fn nodes(&self) -> &[Pose2d] {
        &self.nodes
    }

    /// All edges.
    pub fn edges(&self) -> &[PoseGraphEdge] {
        &self.edges
    }

    /// Residual `(ex, ey, eθ)` of an edge under the current node poses.
    ///
    /// Small residuals indicate a consistent constraint; a large residual after
    /// optimization flags a likely outlier (for example a bad loop closure).
    pub fn edge_residual(&self, index: usize) -> Option<(f64, f64, f64)> {
        let edge = self.edges.get(index)?;
        Some(residual(&self.nodes, edge))
    }

    /// Removes and returns the edge at `index`.
    ///
    /// Removing an edge shifts the indices of later edges, so callers that track
    /// loop-closure edge indices should only remove the most recent edge.
    pub fn remove_edge(&mut self, index: usize) -> Option<PoseGraphEdge> {
        if index < self.edges.len() {
            Some(self.edges.remove(index))
        } else {
            None
        }
    }

    /// Replaces a node pose.
    pub fn set_node(&mut self, index: usize, pose: Pose2d) -> bool {
        if let Some(slot) = self.nodes.get_mut(index) {
            *slot = pose;
            true
        } else {
            false
        }
    }

    /// Returns the indices of nodes reachable from `start`, sorted ascending.
    pub fn component(&self, start: usize) -> Vec<usize> {
        if start >= self.nodes.len() {
            return Vec::new();
        }
        let mut visited = vec![false; self.nodes.len()];
        let mut stack = vec![start];
        visited[start] = true;
        while let Some(node) = stack.pop() {
            for edge in &self.edges {
                let neighbour = if edge.from == node {
                    Some(edge.to)
                } else if edge.to == node {
                    Some(edge.from)
                } else {
                    None
                };
                if let Some(neighbour) = neighbour {
                    if !visited[neighbour] {
                        visited[neighbour] = true;
                        stack.push(neighbour);
                    }
                }
            }
        }
        (0..self.nodes.len())
            .filter(|node| visited[*node])
            .collect()
    }

    /// Optimizes the graph, anchoring `anchor` and iterating Gauss-Newton.
    pub fn optimize(
        &mut self,
        iterations: usize,
        damping: f64,
        anchor: usize,
    ) -> Result<f64, PoseGraphError> {
        self.optimize_inner(iterations, damping, anchor, None)
    }

    /// Optimizes the graph with a Huber robust kernel on every edge.
    ///
    /// Edges whose residual norm exceeds `huber_delta` are down-weighted by
    /// `huber_delta / residual`, which keeps a single grossly wrong loop closure
    /// from dragging the whole trajectory.
    pub fn optimize_robust(
        &mut self,
        iterations: usize,
        damping: f64,
        anchor: usize,
        huber_delta: f64,
    ) -> Result<f64, PoseGraphError> {
        if !huber_delta.is_finite() || huber_delta <= 0.0 {
            return Err(PoseGraphError::NonFinite);
        }
        self.optimize_inner(iterations, damping, anchor, Some(huber_delta))
    }

    fn optimize_inner(
        &mut self,
        iterations: usize,
        damping: f64,
        anchor: usize,
        huber_delta: Option<f64>,
    ) -> Result<f64, PoseGraphError> {
        if self.nodes.is_empty() {
            return Err(PoseGraphError::EmptyGraph);
        }
        if iterations == 0 {
            return Err(PoseGraphError::InvalidIterations);
        }
        if anchor >= self.nodes.len() {
            return Err(PoseGraphError::UnknownNode(anchor));
        }
        for node in &self.nodes {
            if !node.is_finite() {
                return Err(PoseGraphError::NonFinite);
            }
        }
        for edge in &self.edges {
            if edge.from >= self.nodes.len() {
                return Err(PoseGraphError::UnknownNode(edge.from));
            }
            if edge.to >= self.nodes.len() {
                return Err(PoseGraphError::UnknownNode(edge.to));
            }
            if !edge.measurement.is_finite()
                || !edge.information.0.is_finite()
                || !edge.information.1.is_finite()
                || !edge.information.2.is_finite()
            {
                return Err(PoseGraphError::NonFinite);
            }
        }

        // Fixed nodes: the anchor and every node disconnected from it.
        let fixed: Vec<bool> = {
            let component = self.component(anchor);
            let mut fixed = vec![true; self.nodes.len()];
            for node in component {
                fixed[node] = false;
            }
            fixed[anchor] = true;
            fixed
        };
        let active: Vec<usize> = (0..self.nodes.len()).filter(|node| !fixed[*node]).collect();
        if active.is_empty() {
            return Ok(0.0);
        }
        let mut free_index = vec![None; self.nodes.len()];
        for (position, node) in active.iter().enumerate() {
            free_index[*node] = Some(position);
        }

        let mut final_error = 0.0;
        for _ in 0..iterations {
            let n = active.len();
            let size = n * 3;
            let mut h = vec![0.0; size * size];
            let mut b = vec![0.0; n * 3];
            let mut total_error = 0.0;

            for edge in &self.edges {
                if fixed[edge.from] && fixed[edge.to] {
                    continue;
                }
                let error = residual(&self.nodes, edge);
                total_error += information_quadratic(edge, &error);
                let (jacobian_from, jacobian_to) = edge_jacobians(&self.nodes, edge);
                let information = match huber_delta {
                    None => edge.information,
                    Some(delta) => {
                        let rho = information_quadratic(edge, &error).sqrt();
                        let weight = if rho <= delta || rho <= f64::EPSILON {
                            1.0
                        } else {
                            delta / rho
                        };
                        (
                            edge.information.0 * weight,
                            edge.information.1 * weight,
                            edge.information.2 * weight,
                        )
                    }
                };
                accumulate(
                    &mut h,
                    &mut b,
                    size,
                    &free_index,
                    edge.from,
                    jacobian_from,
                    error,
                    information,
                );
                if edge.to != edge.from {
                    accumulate(
                        &mut h,
                        &mut b,
                        size,
                        &free_index,
                        edge.to,
                        jacobian_to,
                        error,
                        information,
                    );
                }
            }
            add_damping(&mut h, n, damping);
            let delta = solve_cholesky(&h, &b, n).ok_or(PoseGraphError::SingularSystem)?;
            for (position, node) in active.iter().enumerate() {
                let pose = &mut self.nodes[*node];
                pose.x_m -= delta[position * 3];
                pose.y_m -= delta[position * 3 + 1];
                pose.yaw_rad = wrap_angle(pose.yaw_rad - delta[position * 3 + 2]);
            }
            final_error = total_error;
            if delta.iter().all(|value| value.abs() < 1.0e-10) {
                break;
            }
        }

        Ok(final_error)
    }
}

impl Default for PoseGraph {
    fn default() -> Self {
        Self::new()
    }
}

fn residual(nodes: &[Pose2d], edge: &PoseGraphEdge) -> (f64, f64, f64) {
    let predicted = nodes[edge.from].inverse().compose(nodes[edge.to]);
    let error = edge.measurement.inverse().compose(predicted);
    (error.x_m, error.y_m, wrap_angle(error.yaw_rad))
}

/// Returns the Jacobians of the residual with respect to `from` and `to`.
///
/// The residual is `e = m⁻¹ ∘ q_from⁻¹ ∘ q_to` and perturbations are additive
/// in the world frame `(x, y, yaw)`. Written as `J[row][axis]` with rows
/// `(e_x, e_y, e_θ)` and axes `(x, y, θ)`, and `θ = q_from.yaw + m.yaw`:
///
/// * `J_from = [[-cosθ, -sinθ, d·sinθ], [sinθ, -cosθ, -d·cosθ], [0, 0, -1]]`
/// * `J_to   = [[ cosθ,  sinθ, 0],        [-sinθ, cosθ, 0],       [0, 0, 1]]`
///
/// where `d = q_from - q_to`.
fn edge_jacobians(nodes: &[Pose2d], edge: &PoseGraphEdge) -> ([[f64; 3]; 3], [[f64; 3]; 3]) {
    let from = nodes[edge.from];
    let to = nodes[edge.to];
    let theta = from.yaw_rad + edge.measurement.yaw_rad;
    let (sin_t, cos_t) = theta.sin_cos();
    let dx = from.x_m - to.x_m;
    let dy = from.y_m - to.y_m;

    let jacobian_from = [
        [-cos_t, -sin_t, dx * sin_t - dy * cos_t],
        [sin_t, -cos_t, dx * cos_t + dy * sin_t],
        [0.0, 0.0, -1.0],
    ];
    let jacobian_to = [[cos_t, sin_t, 0.0], [-sin_t, cos_t, 0.0], [0.0, 0.0, 1.0]];
    (jacobian_from, jacobian_to)
}

/// Applies `Jᵀ Ω J` to `h` and `Jᵀ Ω e` to `b` for one endpoint.
#[allow(clippy::too_many_arguments)]
fn accumulate(
    h: &mut [f64],
    b: &mut [f64],
    size: usize,
    free_index: &[Option<usize>],
    node: usize,
    jacobian: [[f64; 3]; 3],
    error: (f64, f64, f64),
    information: (f64, f64, f64),
) {
    let Some(position) = free_index[node] else {
        return;
    };
    let error = [error.0, error.1, error.2];
    let weights = [information.0, information.1, information.2];
    let base = position * 3;
    for a in 0..3 {
        for c in 0..3 {
            let mut sum = 0.0;
            for (row, weight) in weights.iter().enumerate() {
                sum += jacobian[row][a] * weight * jacobian[row][c];
            }
            h[(base + a) * size + base + c] += sum;
        }
        let mut sum = 0.0;
        for (row, weight) in weights.iter().enumerate() {
            sum += jacobian[row][a] * weight * error[row];
        }
        b[base + a] += sum;
    }
}
fn information_quadratic(edge: &PoseGraphEdge, error: &(f64, f64, f64)) -> f64 {
    let values = [error.0, error.1, error.2];
    let weights = [edge.information.0, edge.information.1, edge.information.2];
    values
        .iter()
        .zip(weights)
        .map(|(value, weight)| value * value * weight)
        .sum()
}

fn add_damping(h: &mut [f64], n: usize, damping: f64) {
    let diagonal = damping.max(0.0).max(1.0e-9);
    let size = n * 3;
    for i in 0..size {
        h[i * size + i] += diagonal;
    }
}

/// Solves `H x = b` for a symmetric positive-definite `H` via Cholesky.
fn solve_cholesky(h: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let size = n * 3;
    let mut lower = vec![0.0; size * size];
    for row in 0..size {
        for col in 0..=row {
            let mut sum = h[row * size + col];
            for k in 0..col {
                sum -= lower[row * size + k] * lower[col * size + k];
            }
            if row == col {
                if sum <= 1.0e-18 {
                    return None;
                }
                lower[row * size + col] = sum.sqrt();
            } else {
                lower[row * size + col] = sum / lower[col * size + col];
            }
        }
    }

    let mut y = vec![0.0; size];
    for row in 0..size {
        let mut sum = b[row];
        for k in 0..row {
            sum -= lower[row * size + k] * y[k];
        }
        y[row] = sum / lower[row * size + row];
    }

    let mut x = vec![0.0; size];
    for row in (0..size).rev() {
        let mut sum = y[row];
        for k in (row + 1)..size {
            sum -= lower[k * size + row] * x[k];
        }
        x[row] = sum / lower[row * size + row];
    }
    if x.iter().all(|value| value.is_finite()) {
        Some(x)
    } else {
        None
    }
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Finite-difference check of `edge_jacobians` against `residual`.
    #[test]
    fn jacobians_match_finite_difference() {
        let eps = 1.0e-6;
        let from = Pose2d::new(0.3, -0.2, 0.7);
        let to = Pose2d::new(1.1, 0.9, -0.4);
        let measurement = Pose2d::new(0.6, 0.5, 0.2);
        let nodes = [from, to];
        let edge = PoseGraphEdge {
            from: 0,
            to: 1,
            measurement,
            information: (1.0, 1.0, 1.0),
            loop_closure: false,
        };
        let (jac_from, jac_to) = edge_jacobians(&nodes, &edge);

        for axis in 0..3 {
            for sign in [-1.0_f64, 1.0] {
                let delta = sign * eps;
                let base = residual(&nodes, &edge);
                let base_values = [base.0, base.1, base.2];

                let mut perturbed_from = from;
                match axis {
                    0 => perturbed_from.x_m += delta,
                    1 => perturbed_from.y_m += delta,
                    _ => perturbed_from.yaw_rad += delta,
                }
                let from_result = residual(&[perturbed_from, to], &edge);
                let from_values = [from_result.0, from_result.1, from_result.2];
                let numeric_from = [
                    (from_values[0] - base_values[0]) / delta,
                    (from_values[1] - base_values[1]) / delta,
                    wrap_angle(from_values[2] - base_values[2]) / delta,
                ];

                let mut perturbed_to = to;
                match axis {
                    0 => perturbed_to.x_m += delta,
                    1 => perturbed_to.y_m += delta,
                    _ => perturbed_to.yaw_rad += delta,
                }
                let to_result = residual(&[from, perturbed_to], &edge);
                let to_values = [to_result.0, to_result.1, to_result.2];
                let numeric_to = [
                    (to_values[0] - base_values[0]) / delta,
                    (to_values[1] - base_values[1]) / delta,
                    wrap_angle(to_values[2] - base_values[2]) / delta,
                ];

                for row in 0..3 {
                    assert_relative_eq!(jac_from[row][axis], numeric_from[row], epsilon = 1e-4);
                    assert_relative_eq!(jac_to[row][axis], numeric_to[row], epsilon = 1e-4);
                }
            }
        }
    }

    #[test]
    fn optimizes_a_small_square_loop() {
        // Truth: unit square. Odometry drifts, a loop closure closes the gap.
        let truth = [
            Pose2d::new(0.0, 0.0, 0.0),
            Pose2d::new(1.0, 0.0, 0.0),
            Pose2d::new(1.0, 1.0, std::f64::consts::FRAC_PI_2),
            Pose2d::new(0.0, 1.0, std::f64::consts::PI),
        ];
        let mut graph = PoseGraph::new();
        // Chain nodes with slightly drifted poses.
        graph.add_node(Pose2d::new(0.0, 0.0, 0.0));
        graph.add_node(Pose2d::new(1.05, 0.02, 0.01));
        graph.add_node(Pose2d::new(1.08, 1.03, 1.55));
        graph.add_node(Pose2d::new(-0.05, 1.02, 3.12));

        // Odometry measurements taken from truth (consistent constraints).
        for index in 0..3 {
            let measurement = truth[index].inverse().compose(truth[index + 1]);
            graph.add_edge(PoseGraphEdge::odometry(index, index + 1, measurement));
        }
        // Loop closure from the last node back to the first.
        let closure = truth[3].inverse().compose(truth[0]);
        graph.add_edge(PoseGraphEdge::loop_closure(3, 0, closure, (1.0, 1.0, 1.0)));

        let before: f64 = graph
            .nodes()
            .iter()
            .zip(&truth)
            .map(|(node, truth)| (node.x_m - truth.x_m).hypot(node.y_m - truth.y_m))
            .sum();
        let error = graph.optimize(50, 1.0e-3, 0).unwrap();
        let after: f64 = graph
            .nodes()
            .iter()
            .zip(&truth)
            .map(|(node, truth)| (node.x_m - truth.x_m).hypot(node.y_m - truth.y_m))
            .sum();

        assert!(after < before * 0.2, "before={before} after={after}");
        assert!(error < 0.05, "residual error={error}");
        assert_relative_eq!(graph.node(0).unwrap().x_m, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn rejects_unknown_node_edges() {
        let mut graph = PoseGraph::new();
        graph.add_node(Pose2d::IDENTITY);
        graph.add_edge(PoseGraphEdge::odometry(0, 5, Pose2d::IDENTITY));
        assert_eq!(
            graph.optimize(10, 1.0e-3, 0),
            Err(PoseGraphError::UnknownNode(5))
        );
    }

    #[test]
    fn empty_graph_is_rejected() {
        let mut graph = PoseGraph::new();
        assert_eq!(
            graph.optimize(10, 1.0e-3, 0),
            Err(PoseGraphError::EmptyGraph)
        );
    }

    #[test]
    fn robust_optimization_rejects_a_gross_outlier() {
        let truth = [
            Pose2d::new(0.0, 0.0, 0.0),
            Pose2d::new(1.0, 0.0, 0.0),
            Pose2d::new(2.0, 0.0, 0.0),
        ];
        let build = || {
            let mut graph = PoseGraph::new();
            graph.add_node(truth[0]);
            graph.add_node(truth[1]);
            graph.add_node(truth[2]);
            graph.add_edge(PoseGraphEdge::odometry(
                0,
                1,
                truth[0].inverse().compose(truth[1]),
            ));
            graph.add_edge(PoseGraphEdge::odometry(
                1,
                2,
                truth[1].inverse().compose(truth[2]),
            ));
            // A grossly wrong loop closure from node 2 back to node 0.
            graph.add_edge(PoseGraphEdge::loop_closure(
                2,
                0,
                Pose2d::new(-5.0, 3.0, 1.0),
                (1.0, 1.0, 1.0),
            ));
            graph
        };
        let error = |graph: &PoseGraph| -> f64 {
            graph
                .nodes()
                .iter()
                .zip(&truth)
                .map(|(node, truth)| (node.x_m - truth.x_m).hypot(node.y_m - truth.y_m))
                .sum()
        };

        let mut plain = build();
        plain.optimize(50, 1.0e-3, 0).unwrap();
        let mut robust = build();
        robust.optimize_robust(50, 1.0e-3, 0, 0.25).unwrap();

        assert!(
            error(&robust) < error(&plain) * 0.5,
            "robust={} plain={}",
            error(&robust),
            error(&plain)
        );
    }

    #[test]
    fn edge_residual_flags_inconsistent_constraints() {
        let mut graph = PoseGraph::new();
        graph.add_node(Pose2d::new(0.0, 0.0, 0.0));
        graph.add_node(Pose2d::new(1.0, 0.0, 0.0));
        graph.add_edge(PoseGraphEdge::odometry(0, 1, Pose2d::new(1.0, 0.0, 0.0)));
        let consistent = graph.edge_residual(0).unwrap();
        assert!(consistent.0.hypot(consistent.1) < 1.0e-9);

        graph.add_edge(PoseGraphEdge::loop_closure(
            1,
            0,
            Pose2d::new(5.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
        ));
        let outlier = graph.edge_residual(1).unwrap();
        assert!(outlier.0.hypot(outlier.1) > 1.0);

        assert!(graph.remove_edge(1).is_some());
        assert!(graph.edge_residual(1).is_none());
    }
}
