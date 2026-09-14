//! Built-in motion planners.
//!
//! `JointInterpolationPlanner` is the MoveIt `JointInterpolation` analogue: it
//! interpolates directly to a resolved goal and rejects the plan if the straight
//! line collides. `RrtConnectPlanner` grows a deterministic, seeded tree from
//! the start and connects a goal tree to it.

use crate::constraints::{GoalConstraint, PathConstraint};
use crate::error::PlanningError;
use crate::group::PlanningGroup;
use crate::planner::MotionPlanner;
use crate::request::{MotionPlanRequest, MotionPlanResponse, PlanningOptions};
use crate::rng::DeterministicRng;
use crate::scene::PlanningScene;
use crate::trajectory::{RobotTrajectory, TrajectoryPoint};
use rne_math::{Pose3, Quat, Vec3};
use rne_robot::{IkRequest, JointLimits, DAMPED_LEAST_SQUARES_SOLVER};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Name of the built-in joint interpolation planner.
pub const JOINT_INTERPOLATION_PLANNER: &str = "joint_interpolation";
/// Name of the built-in bidirectional RRT-Connect planner.
pub const RRT_CONNECT_PLANNER: &str = "rrt_connect";

/// MoveIt `JointInterpolation` style planner.
///
/// Resolves the goal, verifies that the straight joint-space line is
/// collision-free, and returns a uniformly timed trajectory. There is no
/// sampling, so a plan either succeeds immediately or fails.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JointInterpolationPlanner;

impl JointInterpolationPlanner {
    /// Creates the built-in joint interpolation planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for JointInterpolationPlanner {
    fn name(&self) -> &str {
        JOINT_INTERPOLATION_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }

        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }
        if !scene.is_motion_valid_with(
            &request.start,
            &goal,
            request.options.collision_check_steps,
            request.path_constraint.as_ref(),
        )? {
            return Err(PlanningError::NoPath);
        }

        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: 0,
            trajectory: interpolate_trajectory(&request.start, &goal, &request.options, &active),
        })
    }
}

/// Deterministic RRT-Connect style planner.
///
/// The planner samples joint configurations within the model's joint limits,
/// extends a start tree, and greedily connects a goal tree to each newly added
/// node. Sampling uses an explicit seed from [`PlanningOptions::seed`], so the
/// same scene and request always produce the same trajectory.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RrtConnectPlanner;

impl RrtConnectPlanner {
    /// Creates the built-in RRT-Connect planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for RrtConnectPlanner {
    fn name(&self) -> &str {
        RRT_CONNECT_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }

        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed);
        let mut start_tree = Tree::new(request.start.clone());
        let mut goal_tree = Tree::new(goal.clone());
        let step = request.options.step_size;
        let check_steps = request.options.collision_check_steps;
        let connect_budget = request.options.max_iterations.max(1);

        for iteration in 0..request.options.max_iterations {
            let sample = if rng.next_f64() < request.options.goal_bias {
                goal.clone()
            } else {
                sample_configuration(&limits, &active, &request.start, &mut rng)
            };

            let Some(new_index) = extend_tree(
                scene,
                &mut start_tree,
                &sample,
                step,
                check_steps,
                &active,
                request.path_constraint.as_ref(),
            )?
            else {
                continue;
            };
            let target = start_tree.nodes[new_index].clone();
            if let Some(connect_index) = connect_tree(
                scene,
                &mut goal_tree,
                &target,
                step,
                check_steps,
                connect_budget,
                &active,
                request.path_constraint.as_ref(),
            )? {
                let mut waypoints = start_tree.path_to_root(new_index);
                waypoints.reverse();
                for config in goal_tree.path_to_root(connect_index) {
                    if waypoints.last() != Some(&config) {
                        waypoints.push(config);
                    }
                }
                let trajectory = RobotTrajectory::from_waypoints(
                    &waypoints,
                    request.options.waypoint_duration_s,
                )?;
                return Ok(MotionPlanResponse {
                    planner: self.name().to_string(),
                    iterations: iteration + 1,
                    trajectory,
                });
            }
        }

        Err(PlanningError::IterationLimit)
    }
}

/// Name of the built-in asymptotically optimal RRT* planner.
pub const RRT_STAR_PLANNER: &str = "rrt_star";

/// Deterministic RRT* style planner with parent selection and rewiring.
///
/// Each sampled node chooses the cheapest collision-free parent within
/// `neighbor_radius`, then rewires nearby nodes when routing through the new
/// node is cheaper. The best goal connection found is returned after the
/// iteration budget, so the result is at least as good as the connections
/// explored. Sampling uses the explicit [`PlanningOptions::seed`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RrtStarPlanner;

impl RrtStarPlanner {
    /// Creates the built-in RRT* planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for RrtStarPlanner {
    fn name(&self) -> &str {
        RRT_STAR_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }

        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed);
        let mut tree = StarTree::new(request.start.clone());
        let step = request.options.step_size;
        let check_steps = request.options.collision_check_steps;
        let radius = if request.options.neighbor_radius > 0.0 {
            request.options.neighbor_radius
        } else {
            step * 3.0
        };
        let mut best: Option<(f64, usize, usize)> = None;

        for iteration in 0..request.options.max_iterations {
            let sample = if rng.next_f64() < request.options.goal_bias {
                goal.clone()
            } else {
                sample_configuration(&limits, &active, &request.start, &mut rng)
            };
            let nearest = tree.nearest(&sample);
            let new_config = steer(&tree.nodes[nearest], &sample, step, &active);
            if !scene.is_motion_valid_with(
                &tree.nodes[nearest],
                &new_config,
                check_steps,
                request.path_constraint.as_ref(),
            )? {
                continue;
            }

            let neighbors = tree.neighbors(&new_config, radius);
            let mut parent = nearest;
            let mut parent_cost =
                tree.costs[nearest] + configuration_distance(&tree.nodes[nearest], &new_config);
            for &candidate in &neighbors {
                let candidate_cost = tree.costs[candidate]
                    + configuration_distance(&tree.nodes[candidate], &new_config);
                if candidate_cost + 1.0e-12 < parent_cost
                    && scene.is_motion_valid_with(
                        &tree.nodes[candidate],
                        &new_config,
                        check_steps,
                        request.path_constraint.as_ref(),
                    )?
                {
                    parent_cost = candidate_cost;
                    parent = candidate;
                }
            }
            let new_index = tree.add(new_config, parent);

            for &neighbor in &neighbors {
                if neighbor == parent {
                    continue;
                }
                let rewired_cost = tree.costs[new_index]
                    + configuration_distance(&tree.nodes[new_index], &tree.nodes[neighbor]);
                if rewired_cost + 1.0e-12 < tree.costs[neighbor]
                    && scene.is_motion_valid_with(
                        &tree.nodes[new_index],
                        &tree.nodes[neighbor],
                        check_steps,
                        request.path_constraint.as_ref(),
                    )?
                {
                    tree.set_parent(neighbor, new_index);
                }
            }

            let goal_distance = configuration_distance(&tree.nodes[new_index], &goal);
            if goal_distance <= step * 1.5
                && scene.is_motion_valid_with(
                    &tree.nodes[new_index],
                    &goal,
                    check_steps,
                    request.path_constraint.as_ref(),
                )?
            {
                let goal_cost = tree.costs[new_index] + goal_distance;
                if best.is_none_or(|(cost, _, _)| goal_cost < cost) {
                    best = Some((goal_cost, new_index, iteration));
                }
            }
        }

        let Some((_, best_index, best_iteration)) = best else {
            return Err(PlanningError::IterationLimit);
        };
        let mut waypoints = tree.path_to_root(best_index);
        waypoints.reverse();
        if waypoints.last() != Some(&goal) {
            waypoints.push(goal);
        }
        let trajectory =
            RobotTrajectory::from_waypoints(&waypoints, request.options.waypoint_duration_s)?;
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: best_iteration + 1,
            trajectory,
        })
    }
}

/// Name of the built-in informed RRT* planner.
pub const INFORMED_RRT_STAR_PLANNER: &str = "informed_rrt_star";

/// Informed RRT* style planner.
///
/// Starts as RRT* and, once a solution is found, restricts sampling to the
/// ellipsoid of configurations whose path could still improve the current best
/// cost (the OMPL informed-sampling idea). Deterministic for a given seed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InformedRrtStarPlanner;

impl InformedRrtStarPlanner {
    /// Creates the built-in informed RRT* planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for InformedRrtStarPlanner {
    fn name(&self) -> &str {
        INFORMED_RRT_STAR_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }
        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed ^ 0x1F12_3F4A_5B6C_7D8E);
        let mut tree = StarTree::new(request.start.clone());
        let step = request.options.step_size;
        let check_steps = request.options.collision_check_steps;
        let radius = if request.options.neighbor_radius > 0.0 {
            request.options.neighbor_radius
        } else {
            step * 3.0
        };
        let mut best: Option<(f64, usize, usize)> = None;

        for iteration in 0..request.options.max_iterations {
            let sample = if rng.next_f64() < request.options.goal_bias {
                goal.clone()
            } else if let Some((best_cost, _, _)) = best {
                informed_configuration(&request.start, &goal, &active, &limits, best_cost, &mut rng)
            } else {
                sample_configuration(&limits, &active, &request.start, &mut rng)
            };
            let nearest = tree.nearest(&sample);
            let new_config = steer(&tree.nodes[nearest], &sample, step, &active);
            let path = request.path_constraint.as_ref();
            if !scene.is_motion_valid_with(&tree.nodes[nearest], &new_config, check_steps, path)? {
                continue;
            }

            let neighbors = tree.neighbors(&new_config, radius);
            let mut parent = nearest;
            let mut parent_cost =
                tree.costs[nearest] + configuration_distance(&tree.nodes[nearest], &new_config);
            for &candidate in &neighbors {
                let candidate_cost = tree.costs[candidate]
                    + configuration_distance(&tree.nodes[candidate], &new_config);
                if candidate_cost + 1.0e-12 < parent_cost
                    && scene.is_motion_valid_with(
                        &tree.nodes[candidate],
                        &new_config,
                        check_steps,
                        path,
                    )?
                {
                    parent_cost = candidate_cost;
                    parent = candidate;
                }
            }
            let new_index = tree.add(new_config, parent);

            for &neighbor in &neighbors {
                if neighbor == parent {
                    continue;
                }
                let rewired_cost = tree.costs[new_index]
                    + configuration_distance(&tree.nodes[new_index], &tree.nodes[neighbor]);
                if rewired_cost + 1.0e-12 < tree.costs[neighbor]
                    && scene.is_motion_valid_with(
                        &tree.nodes[new_index],
                        &tree.nodes[neighbor],
                        check_steps,
                        path,
                    )?
                {
                    tree.set_parent(neighbor, new_index);
                }
            }

            let goal_distance = configuration_distance(&tree.nodes[new_index], &goal);
            if goal_distance <= step * 1.5
                && scene.is_motion_valid_with(&tree.nodes[new_index], &goal, check_steps, path)?
            {
                let goal_cost = tree.costs[new_index] + goal_distance;
                if best.is_none_or(|(cost, _, _)| goal_cost < cost) {
                    best = Some((goal_cost, new_index, iteration));
                }
            }
        }

        let Some((_, best_index, best_iteration)) = best else {
            return Err(PlanningError::IterationLimit);
        };
        let mut waypoints = tree.path_to_root(best_index);
        waypoints.reverse();
        if waypoints.last() != Some(&goal) {
            waypoints.push(goal);
        }
        let trajectory =
            RobotTrajectory::from_waypoints(&waypoints, request.options.waypoint_duration_s)?;
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: best_iteration + 1,
            trajectory,
        })
    }
}

/// Samples inside the informed ellipsoid when a solution cost is known.
fn informed_configuration(
    start: &[f64],
    goal: &[f64],
    active: &[usize],
    limits: &[JointLimits],
    best_cost: f64,
    rng: &mut DeterministicRng,
) -> Vec<f64> {
    let dimension = active.len();
    let sub_start: Vec<f64> = active.iter().map(|&dof| start[dof]).collect();
    let sub_goal: Vec<f64> = active.iter().map(|&dof| goal[dof]).collect();
    let minimum = configuration_distance(&sub_start, &sub_goal);
    if dimension == 0 || !best_cost.is_finite() || best_cost <= minimum || minimum <= 1.0e-9 {
        return sample_configuration(limits, active, start, rng);
    }

    let center: Vec<f64> = sub_start
        .iter()
        .zip(&sub_goal)
        .map(|(a, b)| 0.5 * (a + b))
        .collect();
    let first_axis: Vec<f64> = sub_start
        .iter()
        .zip(&sub_goal)
        .map(|(a, b)| (b - a) / minimum)
        .collect();
    let basis = orthonormal_basis(&first_axis, dimension);
    let major = 0.5 * best_cost;
    let minor = (0.25 * (best_cost * best_cost - minimum * minimum))
        .max(0.0)
        .sqrt();

    let mut direction = vec![0.0; dimension];
    let mut norm = 0.0;
    for value in direction.iter_mut() {
        *value = rng.next_f64() * 2.0 - 1.0;
        norm += *value * *value;
    }
    norm = norm.sqrt();
    let radius = rng.next_f64().powf(1.0 / dimension as f64);
    let scale = |component: f64| {
        if norm > 1.0e-12 {
            component / norm
        } else {
            0.0
        }
    };

    let mut sub = center.clone();
    let major_component = major * radius * scale(direction[0]);
    for (value, axis) in sub.iter_mut().zip(&basis[0]) {
        *value += axis * major_component;
    }
    for index in 1..dimension {
        let component = minor * radius * scale(direction[index]);
        for (value, axis) in sub.iter_mut().zip(&basis[index]) {
            *value += axis * component;
        }
    }

    let mut configuration = start.to_vec();
    for (slot, &dof) in active.iter().enumerate() {
        configuration[dof] = clamp_joint(sub[slot], &limits[dof]);
    }
    configuration
}

fn orthonormal_basis(first_axis: &[f64], dimension: usize) -> Vec<Vec<f64>> {
    let mut basis = Vec::with_capacity(dimension);
    let mut first = first_axis.to_vec();
    let norm = first.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm > 1.0e-12 {
        for value in &mut first {
            *value /= norm;
        }
    }
    basis.push(first);
    for axis in 0..dimension {
        if basis.len() == dimension {
            break;
        }
        let mut candidate = vec![0.0; dimension];
        candidate[axis] = 1.0;
        for existing in &basis {
            let projection: f64 = candidate.iter().zip(existing).map(|(a, b)| a * b).sum();
            for (value, basis_value) in candidate.iter_mut().zip(existing) {
                *value -= projection * basis_value;
            }
        }
        let norm = candidate
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if norm > 1.0e-9 {
            for value in &mut candidate {
                *value /= norm;
            }
            basis.push(candidate);
        }
    }
    basis
}

/// Name of the built-in batch informed roadmap planner.
pub const BIT_STAR_PLANNER: &str = "bit_star";

/// Batch-informal sampling planner (BIT*-style).
///
/// Unlike a single-shot roadmap, this samples in batches, connects each batch to
/// the growing roadmap with collision-checked edges, and runs A* after every
/// batch. Once a solution exists, sampling is restricted to the informed
/// ellipsoid. This is the RNE reference to BIT*-style batch informed planning;
/// it is deterministic for a given seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BitStarPlanner {
    /// Samples drawn per batch.
    pub batch_size: usize,
}

impl Default for BitStarPlanner {
    fn default() -> Self {
        Self { batch_size: 25 }
    }
}

impl BitStarPlanner {
    /// Creates the built-in batch informed planner.
    pub fn new() -> Self {
        Self::default()
    }
}

impl MotionPlanner for BitStarPlanner {
    fn name(&self) -> &str {
        BIT_STAR_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }
        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed ^ 0x2C9A_11E5_7B3D_40F1);
        let check_steps = request.options.collision_check_steps;
        let path = request.path_constraint.as_ref();
        let radius = if request.options.neighbor_radius > 0.0 {
            request.options.neighbor_radius
        } else {
            request.options.step_size * 5.0
        };
        let neighbor_cap = request.options.roadmap_neighbors.max(1);
        let batch_size = self.batch_size.max(1);
        let total_samples = request.options.max_iterations.max(2);

        let mut nodes: Vec<Vec<f64>> = vec![request.start.clone(), goal.clone()];
        let mut adjacency: Vec<Vec<(usize, f64)>> = vec![Vec::new(), Vec::new()];
        let mut best: Option<(f64, Vec<usize>)> = None;
        let mut sampled = 0usize;

        while sampled < total_samples {
            let batch = batch_size.min(total_samples - sampled);
            let mut new_indices = Vec::with_capacity(batch);
            for _ in 0..batch {
                let sample = if let Some((cost, _)) = &best {
                    if rng.next_f64() < request.options.goal_bias {
                        goal.clone()
                    } else {
                        informed_configuration(
                            &request.start,
                            &goal,
                            &active,
                            &limits,
                            *cost,
                            &mut rng,
                        )
                    }
                } else {
                    sample_configuration(&limits, &active, &request.start, &mut rng)
                };
                sampled += 1;
                if sample == request.start || sample == goal {
                    continue;
                }
                if scene.is_state_valid(&sample)? {
                    nodes.push(sample);
                    new_indices.push(nodes.len() - 1);
                }
            }
            for _ in 0..new_indices.len() {
                adjacency.push(Vec::new());
            }

            for &node in &new_indices {
                let mut candidates: Vec<(usize, f64)> = (0..nodes.len())
                    .filter(|&other| other != node)
                    .map(|other| (other, configuration_distance(&nodes[node], &nodes[other])))
                    .filter(|(_, distance)| *distance <= radius)
                    .collect();
                candidates.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
                for (other, distance) in candidates.into_iter().take(neighbor_cap) {
                    if other == node
                        || adjacency[node]
                            .iter()
                            .any(|&(existing, _)| existing == other)
                    {
                        continue;
                    }
                    if scene.is_motion_valid_with(&nodes[node], &nodes[other], check_steps, path)? {
                        adjacency[node].push((other, distance));
                        adjacency[other].push((node, distance));
                    }
                }
            }

            if let Some((cost, indices)) = astar_path(&nodes, &adjacency) {
                if best.as_ref().is_none_or(|(current, _)| cost < *current) {
                    best = Some((cost, indices));
                }
            }
        }

        let Some((_, indices)) = best else {
            return Err(PlanningError::NoPath);
        };
        let waypoints: Vec<Vec<f64>> = indices.iter().map(|&index| nodes[index].clone()).collect();
        let trajectory =
            RobotTrajectory::from_waypoints(&waypoints, request.options.waypoint_duration_s)?;
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: nodes.len(),
            trajectory,
        })
    }
}

fn astar_path(nodes: &[Vec<f64>], adjacency: &[Vec<(usize, f64)>]) -> Option<(f64, Vec<usize>)> {
    if nodes.len() < 2 {
        return None;
    }
    let goal = 1usize;
    let heuristic = |index: usize| configuration_distance(&nodes[index], &nodes[goal]);
    let mut cost = vec![f64::INFINITY; nodes.len()];
    let mut previous: Vec<Option<usize>> = vec![None; nodes.len()];
    let mut closed = vec![false; nodes.len()];
    cost[0] = 0.0;
    let mut queue = BinaryHeap::new();
    queue.push(QueueNode {
        cost: heuristic(0),
        index: 0,
    });
    while let Some(node) = queue.pop() {
        if closed[node.index] {
            continue;
        }
        closed[node.index] = true;
        if node.index == goal {
            let mut indices = vec![goal];
            let mut current = goal;
            while let Some(parent) = previous[current] {
                indices.push(parent);
                current = parent;
            }
            indices.reverse();
            return Some((cost[goal], indices));
        }
        for &(neighbor, weight) in &adjacency[node.index] {
            if closed[neighbor] {
                continue;
            }
            let tentative = cost[node.index] + weight;
            if tentative < cost[neighbor] {
                cost[neighbor] = tentative;
                previous[neighbor] = Some(node.index);
                queue.push(QueueNode {
                    cost: tentative + heuristic(neighbor),
                    index: neighbor,
                });
            }
        }
    }
    None
}

#[derive(Clone, Debug)]
struct StarTree {
    nodes: Vec<Vec<f64>>,
    parents: Vec<Option<usize>>,
    children: Vec<Vec<usize>>,
    costs: Vec<f64>,
}

impl StarTree {
    fn new(root: Vec<f64>) -> Self {
        Self {
            nodes: vec![root],
            parents: vec![None],
            children: vec![Vec::new()],
            costs: vec![0.0],
        }
    }

    fn nearest(&self, target: &[f64]) -> usize {
        let mut best = 0;
        let mut best_distance = f64::INFINITY;
        for (index, node) in self.nodes.iter().enumerate() {
            let distance = configuration_distance(node, target);
            if distance < best_distance {
                best_distance = distance;
                best = index;
            }
        }
        best
    }

    fn neighbors(&self, target: &[f64], radius: f64) -> Vec<usize> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| configuration_distance(node, target) <= radius)
            .map(|(index, _)| index)
            .collect()
    }

    fn add(&mut self, config: Vec<f64>, parent: usize) -> usize {
        let cost = self.costs[parent] + configuration_distance(&self.nodes[parent], &config);
        let index = self.nodes.len();
        self.nodes.push(config);
        self.parents.push(Some(parent));
        self.children.push(Vec::new());
        self.costs.push(cost);
        self.children[parent].push(index);
        index
    }

    fn set_parent(&mut self, node: usize, new_parent: usize) {
        if let Some(old_parent) = self.parents[node] {
            self.children[old_parent].retain(|&child| child != node);
        }
        self.parents[node] = Some(new_parent);
        self.children[new_parent].push(node);
        self.propagate(new_parent);
    }

    fn propagate(&mut self, root: usize) {
        let mut stack = vec![root];
        while let Some(node) = stack.pop() {
            let base = self.costs[node];
            let children = self.children[node].clone();
            for child in children {
                self.costs[child] =
                    base + configuration_distance(&self.nodes[node], &self.nodes[child]);
                stack.push(child);
            }
        }
    }

    fn path_to_root(&self, mut index: usize) -> Vec<Vec<f64>> {
        let mut path = vec![self.nodes[index].clone()];
        while let Some(parent) = self.parents[index] {
            path.push(self.nodes[parent].clone());
            index = parent;
        }
        path
    }
}

/// Name of the built-in hybrid planner.
pub const HYBRID_PLANNER: &str = "hybrid";

/// Hybrid planner: a global RRT-Connect plan refined by CHOMP optimization.
///
/// This mirrors MoveIt's hybrid planning, where a sampling planner finds a
/// feasible path and a trajectory optimizer smooths it. The result is
/// deterministic because both stages are.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HybridPlanner;

impl HybridPlanner {
    /// Creates the built-in hybrid planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for HybridPlanner {
    fn name(&self) -> &str {
        HYBRID_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        let base = RrtConnectPlanner::new().plan(scene, request)?;
        let optimized = crate::optimize::optimize_trajectory(
            scene,
            request,
            &base.trajectory,
            &crate::optimize::ChompOptions::default(),
        )?;
        let feasible = crate::optimize::trajectory_is_feasible(scene, request, &optimized)?;
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: base.iterations,
            trajectory: if feasible { optimized } else { base.trajectory },
        })
    }
}

/// Name of the built-in STOMP planner.
pub const STOMP_PLANNER: &str = "stomp";

/// Stochastic trajectory optimization planner.
///
/// This is the RNE reference to MoveIt's STOMP: each iteration draws noisy
/// rollouts around the current trajectory, evaluates the smoothness plus
/// obstacle cost, and moves to the cost-weighted average while keeping the
/// endpoints fixed. The best trajectory seen is returned. Sampling uses the
/// explicit [`PlanningOptions::seed`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StompPlanner {
    /// Optimization iterations.
    pub iterations: usize,
    /// Noisy rollouts per iteration.
    pub rollouts: usize,
    /// Per-waypoint joint noise magnitude.
    pub noise: f64,
    /// Softmax temperature for rollout weighting.
    pub temperature: f64,
}

impl Default for StompPlanner {
    fn default() -> Self {
        Self {
            iterations: 30,
            rollouts: 20,
            noise: 0.1,
            temperature: 0.05,
        }
    }
}

impl StompPlanner {
    /// Creates the built-in STOMP planner with default options.
    pub fn new() -> Self {
        Self::default()
    }
}

impl MotionPlanner for StompPlanner {
    fn name(&self) -> &str {
        STOMP_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }
        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let count = request.options.waypoint_count.max(2);
        let mut trajectory: Vec<Vec<f64>> = (0..count)
            .map(|index| {
                let interpolation = index as f64 / (count - 1) as f64;
                let mut positions = request.start.clone();
                for &dof_index in &active {
                    positions[dof_index] = request.start[dof_index]
                        + (goal[dof_index] - request.start[dof_index]) * interpolation;
                }
                positions
            })
            .collect();

        let cost_options = crate::optimize::ChompOptions {
            smoothness_weight: 1.0,
            obstacle_weight: 1.0,
            safe_distance_m: 0.05,
            ..crate::optimize::ChompOptions::default()
        };
        let mut best = trajectory.clone();
        let mut best_cost = crate::optimize::trajectory_cost(scene, &trajectory, &cost_options)?;
        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed ^ 0x51ED_270B_9C6A_1F3D);
        let temperature = self.temperature.max(1.0e-9);

        for _ in 0..self.iterations {
            let mut rollouts = Vec::with_capacity(self.rollouts.max(1));
            let mut costs = Vec::with_capacity(self.rollouts.max(1));
            for _ in 0..self.rollouts.max(1) {
                let mut noisy = trajectory.clone();
                for (index, waypoint) in noisy
                    .iter_mut()
                    .enumerate()
                    .skip(1)
                    .take(count.saturating_sub(2))
                {
                    let window = (std::f64::consts::PI * index as f64 / (count - 1) as f64)
                        .sin()
                        .abs();
                    for &dof_index in &active {
                        let delta = (rng.next_f64() * 2.0 - 1.0) * self.noise * window;
                        waypoint[dof_index] =
                            clamp_joint(waypoint[dof_index] + delta, &limits[dof_index]);
                    }
                }
                costs.push(crate::optimize::trajectory_cost(
                    scene,
                    &noisy,
                    &cost_options,
                )?);
                rollouts.push(noisy);
            }

            let minimum = costs.iter().copied().fold(f64::INFINITY, f64::min);
            let mut weights: Vec<f64> = costs
                .iter()
                .map(|cost| (-(cost - minimum) / temperature).exp())
                .collect();
            let total: f64 = weights.iter().sum();
            if total <= 0.0 || !total.is_finite() {
                continue;
            }
            for weight in &mut weights {
                *weight /= total;
            }

            let mut averaged = trajectory.clone();
            for (index, waypoint) in averaged
                .iter_mut()
                .enumerate()
                .skip(1)
                .take(count.saturating_sub(2))
            {
                for &dof_index in &active {
                    waypoint[dof_index] = rollouts
                        .iter()
                        .zip(&weights)
                        .map(|(rollout, weight)| weight * rollout[index][dof_index])
                        .sum();
                }
            }
            let averaged_cost = crate::optimize::trajectory_cost(scene, &averaged, &cost_options)?;
            if averaged_cost < best_cost {
                best_cost = averaged_cost;
                best = averaged.clone();
            }
            trajectory = averaged;
        }

        let trajectory =
            RobotTrajectory::from_waypoints(&best, request.options.waypoint_duration_s)?;
        if !crate::optimize::trajectory_is_feasible(scene, request, &trajectory)? {
            return Err(PlanningError::NoPath);
        }
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: self.iterations,
            trajectory,
        })
    }
}

fn clamp_joint(value: f64, limit: &JointLimits) -> f64 {
    let lower = if limit.lower.is_finite() {
        limit.lower
    } else {
        f64::NEG_INFINITY
    };
    let upper = if limit.upper.is_finite() {
        limit.upper
    } else {
        f64::INFINITY
    };
    value.clamp(lower, upper)
}

/// Name of the built-in probabilistic roadmap planner.
pub const PRM_PLANNER: &str = "prm";

/// Deterministic probabilistic roadmap planner.
///
/// Samples collision-free configurations, connects each node to its nearest
/// neighbours with collision-checked straight-line edges (MoveIt's local
/// planner), then searches the roadmap with Dijkstra. Sampling uses the
/// explicit [`PlanningOptions::seed`]; `max_iterations` bounds the number of
/// samples and [`PlanningOptions::roadmap_neighbors`] the edges per node.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PrmPlanner;

impl PrmPlanner {
    /// Creates the built-in probabilistic roadmap planner.
    pub fn new() -> Self {
        Self
    }
}

impl MotionPlanner for PrmPlanner {
    fn name(&self) -> &str {
        PRM_PLANNER
    }

    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        request.validate()?;
        let dof = scene.model().dof();
        if request.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }

        let group = resolve_group(scene, request)?;
        let active = active_indices(dof, group);
        let goal = resolve_goal(scene, request, group)?;
        if goal.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }
        if !scene.is_state_valid(&goal)? {
            return Err(PlanningError::GoalInCollision);
        }

        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed);
        let check_steps = request.options.collision_check_steps;
        let path = request.path_constraint.as_ref();
        let radius = if request.options.neighbor_radius > 0.0 {
            request.options.neighbor_radius
        } else {
            request.options.step_size * 5.0
        };

        let mut nodes: Vec<Vec<f64>> = vec![request.start.clone(), goal.clone()];
        let sample_budget = request.options.max_iterations.max(2);
        for _ in 2..sample_budget {
            let sample = sample_configuration(&limits, &active, &request.start, &mut rng);
            if sample == request.start || sample == goal {
                continue;
            }
            if scene.is_state_valid(&sample)? {
                nodes.push(sample);
            }
        }

        let neighbor_count = request
            .options
            .roadmap_neighbors
            .min(nodes.len().saturating_sub(1))
            .max(1);
        let mut adjacency: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nodes.len()];
        for node in 0..nodes.len() {
            let mut candidates: Vec<(usize, f64)> = (0..nodes.len())
                .filter(|&other| other != node)
                .map(|other| (other, configuration_distance(&nodes[node], &nodes[other])))
                .filter(|(_, distance)| *distance <= radius)
                .collect();
            candidates.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
            for (other, distance) in candidates.into_iter().take(neighbor_count) {
                if other <= node {
                    continue;
                }
                if scene.is_motion_valid_with(&nodes[node], &nodes[other], check_steps, path)? {
                    adjacency[node].push((other, distance));
                    adjacency[other].push((node, distance));
                }
            }
        }

        let mut distance = vec![f64::INFINITY; nodes.len()];
        let mut previous: Vec<Option<usize>> = vec![None; nodes.len()];
        distance[0] = 0.0;
        let mut queue = BinaryHeap::new();
        queue.push(QueueNode {
            cost: 0.0,
            index: 0,
        });
        while let Some(current) = queue.pop() {
            if current.cost > distance[current.index] {
                continue;
            }
            if current.index == 1 {
                break;
            }
            for &(neighbor, weight) in &adjacency[current.index] {
                let tentative = current.cost + weight;
                if tentative < distance[neighbor] {
                    distance[neighbor] = tentative;
                    previous[neighbor] = Some(current.index);
                    queue.push(QueueNode {
                        cost: tentative,
                        index: neighbor,
                    });
                }
            }
        }
        if !distance[1].is_finite() {
            return Err(PlanningError::NoPath);
        }

        let mut indices = vec![1usize];
        let mut current = 1usize;
        while let Some(step) = previous[current] {
            indices.push(step);
            current = step;
        }
        indices.reverse();
        let waypoints: Vec<Vec<f64>> = indices.iter().map(|&index| nodes[index].clone()).collect();
        let trajectory =
            RobotTrajectory::from_waypoints(&waypoints, request.options.waypoint_duration_s)?;
        Ok(MotionPlanResponse {
            planner: self.name().to_string(),
            iterations: nodes.len(),
            trajectory,
        })
    }
}

#[derive(PartialEq)]
struct QueueNode {
    cost: f64,
    index: usize,
}

impl Eq for QueueNode {}

impl Ord for QueueNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.index.cmp(&self.index))
    }
}

impl PartialOrd for QueueNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn resolve_group<'a>(
    scene: &'a PlanningScene,
    request: &MotionPlanRequest,
) -> Result<Option<&'a PlanningGroup>, PlanningError> {
    match &request.group {
        None => Ok(None),
        Some(name) => scene
            .group(name)
            .map(Some)
            .ok_or_else(|| PlanningError::UnknownGroup(name.clone())),
    }
}

fn active_indices(dof: usize, group: Option<&PlanningGroup>) -> Vec<usize> {
    match group {
        Some(group) => group.dof_indices().to_vec(),
        None => (0..dof).collect(),
    }
}

fn resolve_goal(
    scene: &PlanningScene,
    request: &MotionPlanRequest,
    group: Option<&PlanningGroup>,
) -> Result<Vec<f64>, PlanningError> {
    match &request.goal {
        GoalConstraint::Joint { positions } => {
            let dof = scene.model().dof();
            if positions.len() == dof {
                Ok(positions.clone())
            } else if let Some(group) = group {
                if positions.len() == group.dof() {
                    Ok(group.embed(&request.start, positions))
                } else {
                    Err(PlanningError::JointCountMismatch {
                        provided: positions.len(),
                        expected: group.dof(),
                    })
                }
            } else {
                Err(PlanningError::JointCountMismatch {
                    provided: positions.len(),
                    expected: dof,
                })
            }
        }
        GoalConstraint::Pose { end_link, target } => {
            solve_pose_goal(scene, request, group, *end_link, *target, true, true)
        }
        GoalConstraint::Position { end_link, target } => {
            let target = Pose3 {
                translation: *target,
                rotation: Quat::IDENTITY,
            };
            solve_pose_goal(scene, request, group, *end_link, target, true, false)
        }
        GoalConstraint::Orientation { end_link, target } => {
            let target = Pose3 {
                translation: Vec3::ZERO,
                rotation: *target,
            };
            solve_pose_goal(scene, request, group, *end_link, target, false, true)
        }
    }
}

fn solve_pose_goal(
    scene: &PlanningScene,
    request: &MotionPlanRequest,
    group: Option<&PlanningGroup>,
    end_link: rne_ecs::Entity,
    target: Pose3,
    solve_position: bool,
    solve_orientation: bool,
) -> Result<Vec<f64>, PlanningError> {
    let solver_name = request
        .options
        .solver
        .as_deref()
        .unwrap_or(DAMPED_LEAST_SQUARES_SOLVER);
    let solver = scene
        .solvers()
        .get(solver_name)
        .ok_or_else(|| PlanningError::UnknownSolver(solver_name.to_string()))?;
    let mut options = request.options.ik_options;
    options.solve_position = solve_position;
    options.solve_orientation = solve_orientation;
    let mut ik_request = IkRequest::new(end_link, target, request.start.clone(), options);
    if let Some(group) = group {
        ik_request = ik_request.with_active_dof(group.active_mask(scene.model().dof()));
    }
    let result = if request.options.ik_restarts > 0 {
        solver.search_position_ik(
            scene.model(),
            &ik_request,
            request.options.ik_restarts,
            request.options.seed,
        )
    } else {
        solver.solve(scene.model(), &ik_request)
    };
    match result {
        Ok(solution) => Ok(solution.joint_positions),
        Err(_) => Err(PlanningError::GoalUnreachable),
    }
}

fn interpolate_trajectory(
    start: &[f64],
    goal: &[f64],
    options: &PlanningOptions,
    active: &[usize],
) -> RobotTrajectory {
    let count = options.waypoint_count.max(2);
    let mut points = Vec::with_capacity(count);
    for index in 0..count {
        let interpolation = index as f64 / (count - 1) as f64;
        let mut positions = start.to_vec();
        for &dof in active {
            positions[dof] = start[dof] + (goal[dof] - start[dof]) * interpolation;
        }
        points.push(TrajectoryPoint {
            positions,
            time_from_start_s: index as f64 * options.waypoint_duration_s,
        });
    }
    RobotTrajectory::new(points).expect("interpolated trajectory is always valid")
}

fn sample_configuration(
    limits: &[JointLimits],
    active: &[usize],
    base: &[f64],
    rng: &mut DeterministicRng,
) -> Vec<f64> {
    let mut configuration = base.to_vec();
    for &dof in active {
        let limit = limits[dof];
        let lower = if limit.lower.is_finite() {
            limit.lower
        } else {
            -std::f64::consts::PI
        };
        let upper = if limit.upper.is_finite() {
            limit.upper
        } else {
            std::f64::consts::PI
        };
        configuration[dof] = if upper <= lower {
            lower
        } else {
            lower + (upper - lower) * rng.next_f64()
        };
    }
    configuration
}

fn configuration_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(left, right)| (left - right) * (left - right))
        .sum::<f64>()
        .sqrt()
}

fn steer(from: &[f64], to: &[f64], step: f64, active: &[usize]) -> Vec<f64> {
    let distance = configuration_distance(from, to);
    if distance <= step || distance <= f64::EPSILON {
        return to.to_vec();
    }
    let scale = step / distance;
    let mut result = from.to_vec();
    for &dof in active {
        result[dof] = from[dof] + (to[dof] - from[dof]) * scale;
    }
    result
}

fn extend_tree(
    scene: &PlanningScene,
    tree: &mut Tree,
    target: &[f64],
    step: f64,
    check_steps: usize,
    active: &[usize],
    path: Option<&PathConstraint>,
) -> Result<Option<usize>, PlanningError> {
    let nearest = tree.nearest(target);
    let from = tree.nodes[nearest].clone();
    let new_config = steer(&from, target, step, active);
    if !scene.is_motion_valid_with(&from, &new_config, check_steps, path)? {
        return Ok(None);
    }
    Ok(Some(tree.add(new_config, nearest)))
}

#[allow(clippy::too_many_arguments)]
fn connect_tree(
    scene: &PlanningScene,
    tree: &mut Tree,
    target: &[f64],
    step: f64,
    check_steps: usize,
    budget: usize,
    active: &[usize],
    path: Option<&PathConstraint>,
) -> Result<Option<usize>, PlanningError> {
    for _ in 0..budget {
        let Some(index) = extend_tree(scene, tree, target, step, check_steps, active, path)? else {
            return Ok(None);
        };
        if configuration_distance(&tree.nodes[index], target) <= 1.0e-9 {
            return Ok(Some(index));
        }
    }
    Ok(None)
}

#[derive(Clone, Debug)]
struct Tree {
    nodes: Vec<Vec<f64>>,
    parents: Vec<Option<usize>>,
}

impl Tree {
    fn new(root: Vec<f64>) -> Self {
        Self {
            nodes: vec![root],
            parents: vec![None],
        }
    }

    fn nearest(&self, target: &[f64]) -> usize {
        let mut best = 0;
        let mut best_distance = f64::INFINITY;
        for (index, node) in self.nodes.iter().enumerate() {
            let distance = configuration_distance(node, target);
            if distance < best_distance {
                best_distance = distance;
                best = index;
            }
        }
        best
    }

    fn add(&mut self, config: Vec<f64>, parent: usize) -> usize {
        self.nodes.push(config);
        self.parents.push(Some(parent));
        self.nodes.len() - 1
    }

    fn path_to_root(&self, mut index: usize) -> Vec<Vec<f64>> {
        let mut path = vec![self.nodes[index].clone()];
        while let Some(parent) = self.parents[index] {
            path.push(self.nodes[parent].clone());
            index = parent;
        }
        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::request::MotionPlanRequest;
    use crate::test_support::{arm_world, position_only_ik};
    use approx::assert_relative_eq;
    use rne_math::Vec3;
    use rne_robot::{CollisionPrimitive, CollisionWorld, CollisionWorldObject};

    #[test]
    fn planner_registry_lists_builtins_and_rejects_duplicates() {
        let mut registry = crate::planner::PlannerRegistry::with_builtins();
        assert_eq!(
            registry.names(),
            vec![
                JOINT_INTERPOLATION_PLANNER,
                RRT_CONNECT_PLANNER,
                RRT_STAR_PLANNER,
                PRM_PLANNER,
                HYBRID_PLANNER,
                STOMP_PLANNER,
                INFORMED_RRT_STAR_PLANNER,
                BIT_STAR_PLANNER
            ]
        );
        assert!(matches!(
            registry.register(Box::new(RrtConnectPlanner::new())),
            Err(PlanningError::DuplicatePlanner(name)) if name == RRT_CONNECT_PLANNER
        ));
    }

    #[test]
    fn joint_interpolation_plans_between_joint_goals() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.3, 0.2],
            GoalConstraint::joint(vec![0.8, -0.4]),
            PlanningOptions {
                waypoint_count: 5,
                ..PlanningOptions::default()
            },
        );

        let response = JointInterpolationPlanner::new()
            .plan(&scene, &request)
            .unwrap();
        assert_eq!(response.planner, JOINT_INTERPOLATION_PLANNER);
        assert_eq!(response.iterations, 0);
        assert_eq!(response.trajectory.len(), 5);
        assert_eq!(
            response.trajectory.start_positions().unwrap(),
            request.start.as_slice()
        );
        let goal = response.trajectory.goal_positions().unwrap();
        assert!((goal[0] - 0.8).abs() < 1.0e-12);
        assert!((goal[1] + 0.4).abs() < 1.0e-12);
    }

    #[test]
    fn joint_interpolation_resolves_position_goal() {
        let (world, robot, ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        // A position that is exactly reachable by a nearby configuration, so the
        // solver converges without unwrapping joint angles.
        let desired = [0.6, 0.4];
        let target = scene
            .model()
            .forward_kinematics(&desired)
            .unwrap()
            .link_transform(ee)
            .unwrap()
            .translation;
        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::position(ee, target),
            PlanningOptions {
                ik_options: position_only_ik(),
                ..PlanningOptions::default()
            },
        );

        let response = JointInterpolationPlanner::new()
            .plan(&scene, &request)
            .unwrap();
        let fk = scene
            .model()
            .forward_kinematics(response.trajectory.goal_positions().unwrap())
            .unwrap();
        let reached = fk.link_transform(ee).unwrap().translation;
        assert!((reached - target).length() < 5.0e-3);
    }

    #[test]
    fn rrt_connect_finds_path_and_is_deterministic() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            step_size: 0.5,
            seed: 7,
            max_iterations: 5_000,
            waypoint_count: 20,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let planner = RrtConnectPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.planner, RRT_CONNECT_PLANNER);
        assert_eq!(first.trajectory, second.trajectory);
        assert_eq!(
            first.trajectory.start_positions().unwrap(),
            request.start.as_slice()
        );
        let goal = first.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn reports_start_and_goal_collisions() {
        let (world, robot, _ee) = arm_world();

        let blocking_base = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
            CollisionPrimitive::Sphere {
                center_m: Vec3::ZERO,
                radius_m: 0.2,
            },
        )]);
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_collision_world(blocking_base);
        let request = MotionPlanRequest::new(
            vec![0.3, 0.2],
            GoalConstraint::joint(vec![0.5, 0.5]),
            PlanningOptions::default(),
        );
        assert!(matches!(
            JointInterpolationPlanner::new().plan(&scene, &request),
            Err(PlanningError::StartInCollision)
        ));

        let blocking_goal = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
            CollisionPrimitive::Sphere {
                center_m: Vec3::new(2.0, 0.0, 0.0),
                radius_m: 0.2,
            },
        )]);
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_collision_world(blocking_goal);
        let request = MotionPlanRequest::new(
            vec![0.3, 0.2],
            GoalConstraint::joint(vec![0.0, 0.0]),
            PlanningOptions::default(),
        );
        assert!(matches!(
            JointInterpolationPlanner::new().plan(&scene, &request),
            Err(PlanningError::GoalInCollision)
        ));
    }

    #[test]
    fn rrt_star_finds_path_and_is_deterministic() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            step_size: 0.4,
            goal_bias: 0.2,
            seed: 11,
            max_iterations: 2_000,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let planner = RrtStarPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.planner, RRT_STAR_PLANNER);
        assert_eq!(first.trajectory, second.trajectory);
        let goal = first.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn group_scoped_planning_holds_inactive_joints() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let group = {
            let model = scene.model();
            let joint1 = model.movable_joint_entities()[0];
            PlanningGroup::joints(model, "shoulder", &[joint1], model.base_link(), tool).unwrap()
        };
        let scene = scene.with_group(group);

        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::joint(vec![0.8]),
            PlanningOptions {
                waypoint_count: 5,
                ..PlanningOptions::default()
            },
        )
        .with_group("shoulder");

        let response = JointInterpolationPlanner::new()
            .plan(&scene, &request)
            .unwrap();
        let start = response.trajectory.start_positions().unwrap();
        let goal = response.trajectory.goal_positions().unwrap();
        assert_relative_eq!(start[1], 0.2, epsilon = 1.0e-12);
        assert_relative_eq!(goal[1], 0.2, epsilon = 1.0e-12);
        assert_relative_eq!(goal[0], 0.8, epsilon = 1.0e-12);
    }

    #[test]
    fn group_scoped_pose_goal_uses_only_group_joints() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let target = scene
            .model()
            .forward_kinematics(&[0.8, 0.2])
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        let group = {
            let model = scene.model();
            let joint1 = model.movable_joint_entities()[0];
            PlanningGroup::joints(model, "shoulder", &[joint1], model.base_link(), tool).unwrap()
        };
        let scene = scene.with_group(group);

        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::position(tool, target),
            PlanningOptions {
                ik_options: position_only_ik(),
                ..PlanningOptions::default()
            },
        )
        .with_group("shoulder");

        let response = JointInterpolationPlanner::new()
            .plan(&scene, &request)
            .unwrap();
        let goal = response.trajectory.goal_positions().unwrap();
        assert_relative_eq!(goal[1], 0.2, epsilon = 1.0e-9);
        let reached = scene
            .model()
            .forward_kinematics(goal)
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        assert!((reached - target).length() < 1.0e-3);
    }

    #[test]
    fn pose_goal_with_restarts_is_deterministic() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let target = scene
            .model()
            .forward_kinematics(&[0.8, 0.2])
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        let request = MotionPlanRequest::new(
            vec![0.3, 0.2],
            GoalConstraint::position(tool, target),
            PlanningOptions {
                ik_options: position_only_ik(),
                ik_restarts: 50,
                seed: 3,
                ..PlanningOptions::default()
            },
        );

        let planner = RrtConnectPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.trajectory, second.trajectory);
        let reached = scene
            .model()
            .forward_kinematics(first.trajectory.goal_positions().unwrap())
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        assert!((reached - target).length() < 1.0e-3);
    }

    #[test]
    fn orientation_goal_is_resolved() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let target = scene
            .model()
            .forward_kinematics(&[0.8, 0.2])
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .rotation;
        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::orientation(tool, target),
            PlanningOptions {
                ik_restarts: 50,
                seed: 5,
                ..PlanningOptions::default()
            },
        );

        let response = RrtConnectPlanner::new().plan(&scene, &request).unwrap();
        let goal_rotation = scene
            .model()
            .forward_kinematics(response.trajectory.goal_positions().unwrap())
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .rotation;
        let delta = goal_rotation * target.conjugate();
        let angle = 2.0 * delta.w.abs().min(1.0).acos();
        assert!(angle < 1.0e-3, "orientation error={angle}");
    }

    #[test]
    fn path_constraint_is_enforced_along_motion() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let upright = Quat::IDENTITY;

        let valid = MotionPlanRequest::new(
            vec![0.1, -0.1],
            GoalConstraint::joint(vec![0.2, -0.2]),
            PlanningOptions::default(),
        )
        .with_path_constraint(PathConstraint::Orientation {
            end_link: tool,
            target: upright,
            tolerance_rad: 0.1,
        });
        assert!(JointInterpolationPlanner::new()
            .plan(&scene, &valid)
            .is_ok());

        let violating = MotionPlanRequest::new(
            vec![0.1, -0.1],
            GoalConstraint::joint(vec![0.5, 0.5]),
            PlanningOptions::default(),
        )
        .with_path_constraint(PathConstraint::Orientation {
            end_link: tool,
            target: upright,
            tolerance_rad: 0.1,
        });
        assert!(matches!(
            JointInterpolationPlanner::new().plan(&scene, &violating),
            Err(PlanningError::NoPath)
        ));
    }

    #[test]
    fn scene_collision_objects_change_validity() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
        scene.add_collision_object(
            "block",
            CollisionPrimitive::Sphere {
                center_m: Vec3::new(2.0, 0.0, 0.0),
                radius_m: 0.2,
            },
        );
        assert!(!scene.is_state_valid(&[0.0, 0.0]).unwrap());
        assert!(scene.remove_collision_object("block"));
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
    }

    #[test]
    fn scene_mesh_objects_change_validity() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
        // A floor mesh at z = 0 cuts through the link spheres at z = 0.
        scene
            .add_mesh_collision_object(
                "floor",
                vec![
                    Vec3::new(-1.0, -1.0, 0.0),
                    Vec3::new(1.0, -1.0, 0.0),
                    Vec3::new(1.0, 1.0, 0.0),
                    Vec3::new(-1.0, 1.0, 0.0),
                ],
                vec![[0, 1, 2], [0, 2, 3]],
            )
            .unwrap();
        assert!(!scene.is_state_valid(&[0.0, 0.0]).unwrap());
        assert!(scene.remove_mesh_collision_object("floor"));
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
    }

    #[test]
    fn optimizer_planners_return_feasible_trajectories() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        scene.add_collision_object(
            "obstacle",
            CollisionPrimitive::Sphere {
                center_m: Vec3::new(1.5, 1.0, 0.0),
                radius_m: 0.18,
            },
        );
        let options = PlanningOptions {
            seed: 7,
            step_size: 0.4,
            goal_bias: 0.2,
            max_iterations: 4_000,
            collision_check_steps: 8,
            waypoint_count: 20,
            neighbor_radius: 1.5,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::joint(vec![1.4, -1.0]),
            options,
        );

        // Straight joint interpolation is blocked by the obstacle.
        assert!(JointInterpolationPlanner::new()
            .plan(&scene, &request)
            .is_err());

        // Hybrid refines the collision-free RRT-Connect plan and stays feasible.
        let hybrid = HybridPlanner::new().plan(&scene, &request).unwrap();
        assert!(
            crate::optimize::trajectory_is_feasible(&scene, &request, &hybrid.trajectory).unwrap()
        );

        // STOMP either returns a feasible trajectory or reports no path.
        match StompPlanner::new().plan(&scene, &request) {
            Ok(response) => {
                assert!(crate::optimize::trajectory_is_feasible(
                    &scene,
                    &request,
                    &response.trajectory
                )
                .unwrap());
            }
            Err(PlanningError::NoPath) => {}
            Err(other) => panic!("unexpected error: {other}"),
        }
    }

    #[test]
    fn scene_occupancy_map_changes_validity() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
        // The elbow link sits at (1, 0, 0) at q = 0; occupy that voxel.
        scene
            .add_occupancy_map("cloud", 0.1, 0.1, &[Vec3::new(1.0, 0.0, 0.0)])
            .unwrap();
        assert!(!scene.is_state_valid(&[0.0, 0.0]).unwrap());
        assert!(scene.remove_occupancy_map("cloud"));
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
    }

    #[test]
    fn visibility_path_constraint_checks_angle_and_occlusion() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let visible = PathConstraint::Visibility {
            sensor_link: tool,
            target: Vec3::new(3.0, 0.0, 0.0),
            tolerance_rad: 0.2,
        };
        assert!(visible.is_satisfied(&scene, &[0.0, 0.0]).unwrap());

        let behind = PathConstraint::Visibility {
            sensor_link: tool,
            target: Vec3::new(1.0, 0.0, 0.0),
            tolerance_rad: 0.2,
        };
        assert!(!behind.is_satisfied(&scene, &[0.0, 0.0]).unwrap());

        let blocked = scene.with_collision_world(CollisionWorld::with_objects(vec![
            CollisionWorldObject::new(CollisionPrimitive::Sphere {
                center_m: Vec3::new(2.5, 0.0, 0.0),
                radius_m: 0.1,
            }),
        ]));
        assert!(!visible.is_satisfied(&blocked, &[0.0, 0.0]).unwrap());
    }

    #[test]
    fn unknown_group_is_reported() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.2, 0.2],
            GoalConstraint::joint(vec![0.5, 0.5]),
            PlanningOptions::default(),
        )
        .with_group("missing");
        assert!(matches!(
            JointInterpolationPlanner::new().plan(&scene, &request),
            Err(PlanningError::UnknownGroup(name)) if name == "missing"
        ));
    }

    #[test]
    fn hybrid_plans_then_optimizes() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            step_size: 0.5,
            seed: 4,
            max_iterations: 5_000,
            waypoint_count: 20,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let response = HybridPlanner::new().plan(&scene, &request).unwrap();
        assert_eq!(response.planner, HYBRID_PLANNER);
        let goal = response.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);

        let base = RrtConnectPlanner::new().plan(&scene, &request).unwrap();
        let positions = |trajectory: &RobotTrajectory| {
            trajectory
                .points()
                .iter()
                .map(|point| point.positions.clone())
                .collect::<Vec<_>>()
        };
        let chomp = crate::optimize::ChompOptions::default();
        let hybrid_cost =
            crate::optimize::trajectory_cost(&scene, &positions(&response.trajectory), &chomp)
                .unwrap();
        let base_cost =
            crate::optimize::trajectory_cost(&scene, &positions(&base.trajectory), &chomp).unwrap();
        assert!(hybrid_cost <= base_cost + 1.0e-12);
    }

    #[test]
    fn stomp_optimizes_and_keeps_endpoints() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.6, 0.4]),
            PlanningOptions {
                waypoint_count: 15,
                seed: 2,
                ..PlanningOptions::default()
            },
        );
        let planner = StompPlanner::new();
        let response = planner.plan(&scene, &request).unwrap();
        let again = planner.plan(&scene, &request).unwrap();
        assert_eq!(response.planner, STOMP_PLANNER);
        assert_eq!(response.trajectory, again.trajectory);
        assert_eq!(
            response.trajectory.start_positions().unwrap(),
            request.start.as_slice()
        );
        let goal = response.trajectory.goal_positions().unwrap();
        assert!((goal[0] - 0.6).abs() < 1.0e-9);
        assert!((goal[1] - 0.4).abs() < 1.0e-9);
    }

    #[test]
    fn informed_rrt_star_finds_path_and_is_deterministic() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            step_size: 0.4,
            goal_bias: 0.2,
            seed: 21,
            max_iterations: 3_000,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let planner = InformedRrtStarPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.planner, INFORMED_RRT_STAR_PLANNER);
        assert_eq!(first.trajectory, second.trajectory);
        let goal = first.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn bit_star_finds_path_and_is_deterministic() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            seed: 33,
            max_iterations: 600,
            neighbor_radius: 2.0,
            roadmap_neighbors: 8,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let planner = BitStarPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.planner, BIT_STAR_PLANNER);
        assert_eq!(first.trajectory, second.trajectory);
        let goal = first.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn prm_finds_path_and_is_deterministic() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let options = PlanningOptions {
            seed: 9,
            max_iterations: 800,
            neighbor_radius: 2.0,
            roadmap_neighbors: 10,
            ..PlanningOptions::default()
        };
        let request = MotionPlanRequest::new(
            vec![1.0, 1.0],
            GoalConstraint::joint(vec![-1.0, -1.0]),
            options,
        );

        let planner = PrmPlanner::new();
        let first = planner.plan(&scene, &request).unwrap();
        let second = planner.plan(&scene, &request).unwrap();
        assert_eq!(first.planner, PRM_PLANNER);
        assert_eq!(first.trajectory, second.trajectory);
        let goal = first.trajectory.goal_positions().unwrap();
        assert!((goal[0] + 1.0).abs() < 1.0e-9);
        assert!((goal[1] + 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn attached_body_invalidates_state() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        let link1 = {
            let model = scene.model();
            (0..model.link_count())
                .find(|&index| model.link_name(index) == Some("link1"))
                .and_then(|index| model.link_entity(index))
                .unwrap()
        };
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
        scene
            .attach_body(
                "payload",
                link1,
                rne_robot::ColliderShape::Sphere { radius_m: 0.2 },
                rne_robot::Transform3::from_translation_rotation(
                    Vec3::new(1.0, 0.0, 0.0),
                    Quat::IDENTITY,
                ),
                Vec::new(),
            )
            .unwrap();
        assert!(!scene.is_state_valid(&[0.0, 0.0]).unwrap());
        assert_eq!(scene.attached_bodies().len(), 1);
        assert!(scene.detach_body("payload"));
        assert!(scene.is_state_valid(&[0.0, 0.0]).unwrap());
    }

    #[test]
    fn pipeline_dispatches_to_selected_planner() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.2, 0.1],
            GoalConstraint::joint(vec![0.6, 0.4]),
            PlanningOptions::default(),
        );

        let mut pipeline = crate::pipeline::PlanningPipeline::with_builtins();
        pipeline.set_planner(JOINT_INTERPOLATION_PLANNER).unwrap();
        assert_eq!(
            pipeline.adapter_names(),
            vec![
                crate::adapters::FIX_START_STATE_BOUNDS_ADAPTER,
                crate::adapters::FIX_WORKSPACE_BOUNDS_ADAPTER,
                crate::adapters::FIX_START_STATE_COLLISION_ADAPTER,
                crate::adapters::SIMPLIFY_TRAJECTORY_ADAPTER,
                crate::adapters::TIME_PARAMETERIZATION_ADAPTER
            ]
        );
        let response = pipeline.plan(&scene, &request).unwrap();
        assert_eq!(response.planner, JOINT_INTERPOLATION_PLANNER);
        assert!(matches!(
            pipeline.set_planner("missing"),
            Err(PlanningError::UnknownPlanner(_))
        ));
    }

    #[test]
    fn pipeline_clamps_start_state_and_respects_velocity_limits() {
        let (world, robot, _ee) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![4.0, 0.5],
            GoalConstraint::joint(vec![0.5, 0.5]),
            PlanningOptions {
                waypoint_count: 6,
                ..PlanningOptions::default()
            },
        );

        let mut pipeline = crate::pipeline::PlanningPipeline::with_builtins();
        pipeline.set_planner(JOINT_INTERPOLATION_PLANNER).unwrap();
        let response = pipeline.plan(&scene, &request).unwrap();

        let start = response.trajectory.start_positions().unwrap();
        assert!((start[0] - std::f64::consts::PI).abs() < 1.0e-12);
        assert!((start[1] - 0.5).abs() < 1.0e-12);

        for window in response.trajectory.points().windows(2) {
            let dt = window[1].time_from_start_s - window[0].time_from_start_s;
            if dt <= 0.0 {
                continue;
            }
            for index in 0..2 {
                let velocity = (window[1].positions[index] - window[0].positions[index]).abs() / dt;
                assert!(velocity <= 1.0 + 1.0e-9, "velocity={velocity}");
            }
        }
    }
}
