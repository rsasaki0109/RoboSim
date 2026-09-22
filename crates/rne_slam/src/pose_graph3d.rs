//! 6-DoF pose graph with Gauss-Newton optimization on SE(3).
//!
//! Nodes are rigid poses; edges are measured relative poses with diagonal
//! information. Sequential edges come from preintegrated LiDAR-inertial
//! odometry, loop-closure edges from scan matching. Optimization distributes
//! loop-closure error across the graph.
//!
//! Residuals are the SE(3) log of the measurement mismatch; Jacobians are
//! central finite differences in each node's right-perturbation tangent space.
//! Numerical Jacobians trade a little speed for correctness and are sufficient
//! at the graph sizes this crate targets; analytic SE(3) Jacobians are a later
//! optimization. The solver is a dense Cholesky on the reduced normal equations,
//! mirroring the 2D [`crate::pose_graph`] module.

use crate::se3::Se3;
use serde::{Deserialize, Serialize};

/// Tangent dimension of an SE(3) node (rotation-first).
pub const POSE3D_DIM: usize = 6;

/// A relative-pose constraint between two 3D nodes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseGraph3dEdge {
    /// Source node index.
    pub from: usize,
    /// Target node index.
    pub to: usize,
    /// Measured `from -> to` relative pose.
    pub measurement: Se3,
    /// Diagonal information (inverse covariance) for `(rot(3), trans(3))`.
    pub information: [f64; POSE3D_DIM],
    /// Whether this edge closes a loop.
    pub loop_closure: bool,
}

impl PoseGraph3dEdge {
    /// Creates an odometry edge with unit information.
    pub fn odometry(from: usize, to: usize, measurement: Se3) -> Self {
        Self {
            from,
            to,
            measurement,
            information: [1.0; POSE3D_DIM],
            loop_closure: false,
        }
    }

    /// Creates a loop-closure edge with the given information weights.
    pub fn loop_closure(
        from: usize,
        to: usize,
        measurement: Se3,
        information: [f64; POSE3D_DIM],
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

/// Error returned by 3D pose-graph construction or optimization.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PoseGraph3dError {
    /// The graph has no nodes.
    #[error("3D pose graph has no nodes")]
    EmptyGraph,
    /// An edge references a node index outside the graph.
    #[error("edge references node {0} which does not exist")]
    UnknownNode(usize),
    /// A pose or measurement contained a non-finite value.
    #[error("3D pose graph input must be finite")]
    NonFinite,
    /// The normal equations were singular even after damping.
    #[error("3D pose graph normal equations are singular")]
    SingularSystem,
    /// The iteration budget was too small to run.
    #[error("3D pose graph optimization requires at least one iteration")]
    InvalidIterations,
}

/// A 3D pose graph.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseGraph3d {
    nodes: Vec<Se3>,
    edges: Vec<PoseGraph3dEdge>,
}

impl PoseGraph3d {
    /// Creates an empty graph.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }

    /// Appends a node and returns its index.
    pub fn add_node(&mut self, pose: Se3) -> usize {
        self.nodes.push(pose);
        self.nodes.len() - 1
    }

    /// Appends an edge.
    pub fn add_edge(&mut self, edge: PoseGraph3dEdge) {
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
    pub fn node(&self, index: usize) -> Option<Se3> {
        self.nodes.get(index).copied()
    }

    /// Overwrites a node pose.
    pub fn set_node(&mut self, index: usize, pose: Se3) {
        if let Some(node) = self.nodes.get_mut(index) {
            *node = pose;
        }
    }

    /// All nodes.
    pub fn nodes(&self) -> &[Se3] {
        &self.nodes
    }

    /// All edges.
    pub fn edges(&self) -> &[PoseGraph3dEdge] {
        &self.edges
    }

    /// Optimizes the graph, anchoring `anchor` and iterating Gauss-Newton.
    ///
    /// Returns the final weighted sum-of-squares of the residuals.
    pub fn optimize(
        &mut self,
        iterations: usize,
        damping: f64,
        anchor: usize,
    ) -> Result<f64, PoseGraph3dError> {
        if self.nodes.is_empty() {
            return Err(PoseGraph3dError::EmptyGraph);
        }
        if iterations == 0 {
            return Err(PoseGraph3dError::InvalidIterations);
        }
        if anchor >= self.nodes.len() {
            return Err(PoseGraph3dError::UnknownNode(anchor));
        }
        if self.nodes.iter().any(|node| !node.is_finite()) {
            return Err(PoseGraph3dError::NonFinite);
        }
        for edge in &self.edges {
            if edge.from >= self.nodes.len() {
                return Err(PoseGraph3dError::UnknownNode(edge.from));
            }
            if edge.to >= self.nodes.len() {
                return Err(PoseGraph3dError::UnknownNode(edge.to));
            }
            if !edge.measurement.is_finite()
                || edge.information.iter().any(|value| !value.is_finite())
            {
                return Err(PoseGraph3dError::NonFinite);
            }
        }

        let fixed = self.fixed_mask(anchor);
        let active: Vec<usize> = (0..self.nodes.len()).filter(|node| !fixed[*node]).collect();
        if active.is_empty() {
            return Ok(0.0);
        }
        let mut free_index = vec![None; self.nodes.len()];
        for (position, node) in active.iter().enumerate() {
            free_index[*node] = Some(position);
        }

        let size = active.len() * POSE3D_DIM;
        let mut final_cost = 0.0;
        for _ in 0..iterations {
            let mut h = vec![0.0; size * size];
            let mut b = vec![0.0; size];
            let mut cost = 0.0;

            for edge in &self.edges {
                if fixed[edge.from] && fixed[edge.to] {
                    continue;
                }
                let from = self.nodes[edge.from];
                let to = self.nodes[edge.to];
                let error = residual_pair(from, to, edge.measurement);
                cost += weighted_quadratic(&error, &edge.information);
                let jacobian_from = numeric_jacobian_from(from, to, edge.measurement);
                let jacobian_to = numeric_jacobian_to(from, to, edge.measurement);
                accumulate(
                    &mut h,
                    &mut b,
                    size,
                    &free_index,
                    edge.from,
                    &jacobian_from,
                    &error,
                    &edge.information,
                );
                if edge.to != edge.from {
                    accumulate(
                        &mut h,
                        &mut b,
                        size,
                        &free_index,
                        edge.to,
                        &jacobian_to,
                        &error,
                        &edge.information,
                    );
                }
            }

            let diagonal = damping.max(0.0).max(1.0e-9);
            for i in 0..size {
                h[i * size + i] += diagonal;
            }
            let delta = solve_cholesky(&h, &b, size).ok_or(PoseGraph3dError::SingularSystem)?;
            let mut max_step = 0.0_f64;
            for (position, node) in active.iter().enumerate() {
                let mut xi = [0.0; POSE3D_DIM];
                for axis in 0..POSE3D_DIM {
                    xi[axis] = delta[position * POSE3D_DIM + axis];
                    max_step = max_step.max(xi[axis].abs());
                }
                self.nodes[*node] = self.nodes[*node].compose(Se3::exp(xi));
            }
            final_cost = cost;
            if max_step < 1.0e-10 {
                break;
            }
        }

        Ok(final_cost)
    }

    fn fixed_mask(&self, anchor: usize) -> Vec<bool> {
        let mut visited = vec![false; self.nodes.len()];
        let mut stack = vec![anchor];
        visited[anchor] = true;
        while let Some(node) = stack.pop() {
            for edge in &self.edges {
                let neighbor = if edge.from == node {
                    Some(edge.to)
                } else if edge.to == node {
                    Some(edge.from)
                } else {
                    None
                };
                if let Some(neighbor) = neighbor {
                    if !visited[neighbor] {
                        visited[neighbor] = true;
                        stack.push(neighbor);
                    }
                }
            }
        }
        // The anchor itself is fixed; every node disconnected from it is too.
        let mut fixed: Vec<bool> = visited.iter().map(|connected| !connected).collect();
        fixed[anchor] = true;
        fixed
    }
}

impl Default for PoseGraph3d {
    fn default() -> Self {
        Self::new()
    }
}

fn residual_pair(from: Se3, to: Se3, measurement: Se3) -> [f64; POSE3D_DIM] {
    measurement
        .inverse()
        .compose(from.inverse().compose(to))
        .log()
}

fn perturb(pose: Se3, axis: usize, delta: f64) -> Se3 {
    let mut xi = [0.0; POSE3D_DIM];
    xi[axis] = delta;
    pose.compose(Se3::exp(xi))
}

fn numeric_jacobian_from(from: Se3, to: Se3, measurement: Se3) -> [[f64; POSE3D_DIM]; POSE3D_DIM] {
    numeric_jacobian(from, to, measurement, true)
}

fn numeric_jacobian_to(from: Se3, to: Se3, measurement: Se3) -> [[f64; POSE3D_DIM]; POSE3D_DIM] {
    numeric_jacobian(from, to, measurement, false)
}

fn numeric_jacobian(
    from: Se3,
    to: Se3,
    measurement: Se3,
    vary_from: bool,
) -> [[f64; POSE3D_DIM]; POSE3D_DIM] {
    const EPS: f64 = 1.0e-6;
    let mut jacobian = [[0.0; POSE3D_DIM]; POSE3D_DIM];
    for axis in 0..POSE3D_DIM {
        let plus = if vary_from {
            residual_pair(perturb(from, axis, EPS), to, measurement)
        } else {
            residual_pair(from, perturb(to, axis, EPS), measurement)
        };
        let minus = if vary_from {
            residual_pair(perturb(from, axis, -EPS), to, measurement)
        } else {
            residual_pair(from, perturb(to, axis, -EPS), measurement)
        };
        for (row, jacobian_row) in jacobian.iter_mut().enumerate() {
            jacobian_row[axis] = (plus[row] - minus[row]) / (2.0 * EPS);
        }
    }
    jacobian
}

fn weighted_quadratic(error: &[f64; POSE3D_DIM], information: &[f64; POSE3D_DIM]) -> f64 {
    error
        .iter()
        .zip(information.iter())
        .map(|(value, weight)| value * value * weight)
        .sum()
}

#[allow(clippy::too_many_arguments)]
fn accumulate(
    h: &mut [f64],
    b: &mut [f64],
    size: usize,
    free_index: &[Option<usize>],
    node: usize,
    jacobian: &[[f64; POSE3D_DIM]; POSE3D_DIM],
    error: &[f64; POSE3D_DIM],
    information: &[f64; POSE3D_DIM],
) {
    let Some(position) = free_index[node] else {
        return;
    };
    let base = position * POSE3D_DIM;
    for a in 0..POSE3D_DIM {
        for c in 0..POSE3D_DIM {
            let mut sum = 0.0;
            for row in 0..POSE3D_DIM {
                sum += jacobian[row][a] * information[row] * jacobian[row][c];
            }
            h[(base + a) * size + base + c] += sum;
        }
        let mut sum = 0.0;
        for row in 0..POSE3D_DIM {
            sum += jacobian[row][a] * information[row] * error[row];
        }
        // Gauss-Newton right-hand side is `-J^T W e`.
        b[base + a] -= sum;
    }
}

/// Solves `H x = b` for a symmetric positive-definite `H` via Cholesky.
fn solve_cholesky(h: &[f64], b: &[f64], size: usize) -> Option<Vec<f64>> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::{Quat, Vec3};

    fn yaw(angle: f64) -> Quat {
        Quat::from_axis_angle(Vec3::Y, angle)
    }

    fn pose(x: f64, z: f64, angle: f64) -> Se3 {
        Se3::new(yaw(angle), Vec3::new(x, 0.0, z))
    }

    #[test]
    fn single_edge_converges_to_the_measurement() {
        let mut graph = PoseGraph3d::new();
        graph.add_node(Se3::IDENTITY);
        // Start the second node far from the measurement.
        graph.add_node(pose(3.0, -2.0, 0.9));
        let measurement = pose(1.0, 0.5, 0.3);
        graph.add_edge(PoseGraph3dEdge::odometry(0, 1, measurement));

        let before = graph.node(1).unwrap().inverse().compose(measurement).log();
        let before_cost: f64 = before.iter().map(|value| value * value).sum();
        graph.optimize(50, 1.0e-6, 0).expect("optimize");
        let after = graph.node(1).unwrap().inverse().compose(measurement).log();
        let after_cost: f64 = after.iter().map(|value| value * value).sum();
        assert!(
            after_cost < before_cost * 1.0e-6,
            "cost {before_cost} -> {after_cost}"
        );
        // The optimal node 1 equals the measurement from node 0.
        let expected = graph.node(0).unwrap().compose(measurement);
        let actual = graph.node(1).unwrap();
        assert_relative_eq!(actual.translation.x, expected.translation.x, epsilon = 1e-6);
        assert_relative_eq!(actual.translation.z, expected.translation.z, epsilon = 1e-6);
    }

    #[test]
    fn loop_closure_removes_drift() {
        // Truth: unit triangle in the XZ plane with yaw turns.
        let truth = [
            pose(0.0, 0.0, 0.0),
            pose(1.0, 0.0, 0.0),
            pose(1.0, 1.0, std::f64::consts::FRAC_PI_2),
            pose(0.0, 1.0, std::f64::consts::PI),
        ];
        let mut graph = PoseGraph3d::new();
        // Drifted initial poses.
        graph.add_node(pose(0.0, 0.0, 0.0));
        graph.add_node(pose(1.1, 0.05, 0.02));
        graph.add_node(pose(1.15, 1.08, 1.62));
        graph.add_node(pose(-0.08, 1.05, 3.20));

        for index in 0..3 {
            let measurement = truth[index].inverse().compose(truth[index + 1]);
            graph.add_edge(PoseGraph3dEdge::odometry(index, index + 1, measurement));
        }
        let closure = truth[3].inverse().compose(truth[0]);
        graph.add_edge(PoseGraph3dEdge::loop_closure(3, 0, closure, [1.0; 6]));

        let before: f64 = graph
            .nodes()
            .iter()
            .zip(&truth)
            .map(|(node, truth)| (node.translation - truth.translation).length())
            .sum();
        graph.optimize(80, 1.0e-4, 0).expect("optimize");
        let after: f64 = graph
            .nodes()
            .iter()
            .zip(&truth)
            .map(|(node, truth)| (node.translation - truth.translation).length())
            .sum();
        assert!(after < before * 0.25, "drift {before} -> {after}");
        for (node, truth) in graph.nodes().iter().zip(&truth) {
            assert_relative_eq!(node.translation.z, truth.translation.z, epsilon = 0.05);
        }
    }

    #[test]
    fn invalid_graphs_are_rejected() {
        let mut empty = PoseGraph3d::new();
        assert!(matches!(
            empty.optimize(1, 1e-6, 0),
            Err(PoseGraph3dError::EmptyGraph)
        ));

        let mut graph = PoseGraph3d::new();
        graph.add_node(Se3::IDENTITY);
        graph.add_edge(PoseGraph3dEdge::odometry(0, 5, Se3::IDENTITY));
        assert!(matches!(
            graph.optimize(1, 1e-6, 0),
            Err(PoseGraph3dError::UnknownNode(5))
        ));
    }
}
