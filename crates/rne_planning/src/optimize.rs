//! CHOMP-inspired trajectory optimization.
//!
//! Given a seeded trajectory, the optimizer lowers a cost that combines a
//! smoothness term (squared accelerations) with an obstacle term (squared
//! clearance violation against robot self-collision and world objects). It uses
//! deterministic finite-difference gradients and a backtracking step, keeping
//! the endpoints fixed. This is the RNE reference to `MoveIt`'s CHOMP planner.

use crate::constraints::PathConstraint;
use crate::error::PlanningError;
use crate::request::MotionPlanRequest;
use crate::scene::PlanningScene;
use crate::trajectory::RobotTrajectory;

/// Trajectory optimization options.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChompOptions {
    /// Gradient descent iterations.
    pub iterations: usize,
    /// Initial gradient step size.
    pub step_size: f64,
    /// Weight on the smoothness term.
    pub smoothness_weight: f64,
    /// Weight on the obstacle term.
    pub obstacle_weight: f64,
    /// Clearance below which the obstacle penalty applies, in meters.
    pub safe_distance_m: f64,
    /// Finite-difference step for gradients.
    pub finite_difference: f64,
}

impl Default for ChompOptions {
    fn default() -> Self {
        Self {
            iterations: 100,
            step_size: 0.1,
            smoothness_weight: 1.0,
            obstacle_weight: 1.0,
            safe_distance_m: 0.05,
            finite_difference: 1.0e-4,
        }
    }
}

/// Optimizes a trajectory in place, returning a new one with fixed endpoints.
pub fn optimize_trajectory(
    scene: &PlanningScene,
    request: &MotionPlanRequest,
    trajectory: &RobotTrajectory,
    options: &ChompOptions,
) -> Result<RobotTrajectory, PlanningError> {
    request.validate()?;
    if !options.step_size.is_finite()
        || options.step_size <= 0.0
        || !options.finite_difference.is_finite()
        || options.finite_difference <= 0.0
        || !options.safe_distance_m.is_finite()
        || options.safe_distance_m < 0.0
    {
        return Err(PlanningError::Invalid(
            "trajectory optimizer options must be finite and positive".to_string(),
        ));
    }

    let mut points: Vec<Vec<f64>> = trajectory
        .points()
        .iter()
        .map(|point| point.positions.clone())
        .collect();
    if points.len() < 3 {
        return Ok(trajectory.clone());
    }
    let dof = points[0].len();
    let active: Vec<usize> = match &request.group {
        Some(name) => scene
            .group(name)
            .ok_or_else(|| PlanningError::UnknownGroup(name.clone()))?
            .dof_indices()
            .to_vec(),
        None => (0..dof).collect(),
    };
    let limits = scene.model().joint_limits();

    let collision_check_steps = request.options.collision_check_steps;
    let path_constraint = request.path_constraint.as_ref();
    let original = points.clone();
    let feasible_at_start =
        points_feasible(scene, &points, collision_check_steps, path_constraint)?;
    let mut current = trajectory_cost(scene, &points, options)?;
    let mut feasible_best: Option<(f64, Vec<Vec<f64>>)> =
        feasible_at_start.then(|| (current, points.clone()));
    let mut step = options.step_size;

    for _ in 0..options.iterations {
        let mut candidate = points.clone();
        for index in 1..points.len() - 1 {
            for &dof_index in &active {
                let original = points[index][dof_index];
                points[index][dof_index] = original + options.finite_difference;
                let plus = trajectory_cost(scene, &points, options)?;
                points[index][dof_index] = original - options.finite_difference;
                let minus = trajectory_cost(scene, &points, options)?;
                points[index][dof_index] = original;
                let gradient = (plus - minus) / (2.0 * options.finite_difference);
                candidate[index][dof_index] =
                    clamp_to_limit(original - step * gradient, limits.get(dof_index).copied());
            }
        }

        let candidate_cost = trajectory_cost(scene, &candidate, options)?;
        let candidate_feasible =
            points_feasible(scene, &candidate, collision_check_steps, path_constraint)?;
        if candidate_feasible
            && feasible_best
                .as_ref()
                .is_none_or(|(best_cost, _)| candidate_cost < *best_cost)
        {
            feasible_best = Some((candidate_cost, candidate.clone()));
        }

        // Only replace the working trajectory with a feasible candidate when the
        // seed is already feasible, so the result never becomes less feasible.
        let accept = if feasible_at_start {
            candidate_feasible && candidate_cost + 1.0e-12 < current
        } else {
            candidate_cost + 1.0e-12 < current
        };
        if accept {
            points = candidate;
            current = candidate_cost;
        } else {
            step *= 0.5;
            if step < 1.0e-6 {
                break;
            }
        }
    }

    let result = if feasible_at_start {
        points
    } else {
        feasible_best.map(|(_, best)| best).unwrap_or(original)
    };
    RobotTrajectory::from_waypoints(&result, request.options.waypoint_duration_s)
}

/// Whether every segment of a trajectory is collision free and satisfies the
/// request's path constraint.
pub fn trajectory_is_feasible(
    scene: &PlanningScene,
    request: &MotionPlanRequest,
    trajectory: &RobotTrajectory,
) -> Result<bool, PlanningError> {
    let points: Vec<Vec<f64>> = trajectory
        .points()
        .iter()
        .map(|point| point.positions.clone())
        .collect();
    points_feasible(
        scene,
        &points,
        request.options.collision_check_steps,
        request.path_constraint.as_ref(),
    )
}

fn points_feasible(
    scene: &PlanningScene,
    points: &[Vec<f64>],
    collision_check_steps: usize,
    path_constraint: Option<&PathConstraint>,
) -> Result<bool, PlanningError> {
    for window in points.windows(2) {
        if !scene.is_motion_valid_with(
            &window[0],
            &window[1],
            collision_check_steps,
            path_constraint,
        )? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Cost of a trajectory under the given options.
pub fn trajectory_cost(
    scene: &PlanningScene,
    points: &[Vec<f64>],
    options: &ChompOptions,
) -> Result<f64, PlanningError> {
    if points.len() < 2 {
        return Ok(0.0);
    }
    let mut smoothness = 0.0;
    for index in 1..points.len() - 1 {
        for ((&next, &current), &previous) in points[index + 1]
            .iter()
            .zip(&points[index])
            .zip(&points[index - 1])
        {
            let acceleration = next - 2.0 * current + previous;
            smoothness += acceleration * acceleration;
        }
    }

    let mut obstacle = 0.0;
    for waypoint in points {
        let clearance = minimum_clearance(scene, waypoint)?;
        if clearance < options.safe_distance_m {
            let violation = options.safe_distance_m - clearance;
            obstacle += violation * violation;
        }
    }

    Ok(options.smoothness_weight * smoothness + options.obstacle_weight * obstacle)
}

fn minimum_clearance(scene: &PlanningScene, q: &[f64]) -> Result<f64, PlanningError> {
    let world = scene
        .collision_world()
        .distance(scene.self_checker(), q)?
        .min_distance_m()
        .unwrap_or(f64::INFINITY);
    let self_distance = scene
        .self_checker()
        .distance(q)?
        .min_distance_m()
        .unwrap_or(f64::INFINITY);
    Ok(world.min(self_distance))
}

fn clamp_to_limit(value: f64, limit: Option<rne_robot::JointLimits>) -> f64 {
    let Some(limit) = limit else {
        return value;
    };
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::GoalConstraint;
    use crate::request::{MotionPlanRequest, PlanningOptions};
    use crate::test_support::arm_world;
    use rne_math::Vec3;
    use rne_robot::{CollisionPrimitive, CollisionWorld, CollisionWorldObject};

    #[test]
    fn optimization_reduces_obstacle_cost_and_keeps_endpoints() {
        let (world, robot, tool) = arm_world();
        // Tool at q = [0.3, 0.0] is on the radius-2 circle. Place the obstacle
        // just outside it: the straight path stays feasible but within the safe
        // distance, so the optimizer can trade smoothness for clearance.
        let obstacle_pose = scene_pose(&world, robot, tool);
        let radial = obstacle_pose.normalize_or_zero();
        let obstacle = CollisionPrimitive::Sphere {
            center_m: obstacle_pose + radial * 0.15,
            radius_m: 0.05,
        };
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_collision_world(CollisionWorld::with_objects(vec![
                CollisionWorldObject::new(obstacle),
            ]));

        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.6, 0.0]),
            PlanningOptions {
                waypoint_count: 21,
                ..PlanningOptions::default()
            },
        );
        let initial = RobotTrajectory::from_waypoints(
            &(0..21)
                .map(|index| {
                    let t = index as f64 / 20.0;
                    vec![0.6 * t, 0.0]
                })
                .collect::<Vec<_>>(),
            0.05,
        )
        .unwrap();

        let options = ChompOptions {
            iterations: 50,
            safe_distance_m: 0.1,
            ..ChompOptions::default()
        };
        assert!(trajectory_is_feasible(&scene, &request, &initial).unwrap());
        let initial_cost = trajectory_cost(&scene, &positions(&initial), &options).unwrap();
        let optimized = optimize_trajectory(&scene, &request, &initial, &options).unwrap();
        let optimized_cost = trajectory_cost(&scene, &positions(&optimized), &options).unwrap();

        assert!(
            optimized_cost < initial_cost,
            "{optimized_cost} !< {initial_cost}"
        );
        assert!(trajectory_is_feasible(&scene, &request, &optimized).unwrap());
        assert_eq!(
            optimized.start_positions().unwrap(),
            initial.start_positions().unwrap()
        );
        assert_eq!(
            optimized.goal_positions().unwrap(),
            initial.goal_positions().unwrap()
        );
    }

    #[test]
    fn optimization_preserves_feasibility() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.5, 0.4]),
            PlanningOptions {
                waypoint_count: 15,
                ..PlanningOptions::default()
            },
        );
        let initial = RobotTrajectory::from_waypoints(
            &(0..15)
                .map(|index| {
                    let t = index as f64 / 14.0;
                    vec![0.5 * t, 0.4 * t]
                })
                .collect::<Vec<_>>(),
            0.05,
        )
        .unwrap();
        assert!(trajectory_is_feasible(&scene, &request, &initial).unwrap());
        let optimized =
            optimize_trajectory(&scene, &request, &initial, &ChompOptions::default()).unwrap();
        assert!(trajectory_is_feasible(&scene, &request, &optimized).unwrap());
    }

    #[test]
    fn optimization_is_deterministic() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.5, 0.4]),
            PlanningOptions {
                waypoint_count: 11,
                ..PlanningOptions::default()
            },
        );
        let initial = RobotTrajectory::from_waypoints(
            &(0..11)
                .map(|index| {
                    let t = index as f64 / 10.0;
                    vec![0.5 * t, 0.4 * t]
                })
                .collect::<Vec<_>>(),
            0.05,
        )
        .unwrap();
        let options = ChompOptions::default();
        let first = optimize_trajectory(&scene, &request, &initial, &options).unwrap();
        let second = optimize_trajectory(&scene, &request, &initial, &options).unwrap();
        assert_eq!(first, second);
    }

    fn scene_pose(
        world: &bevy_ecs::prelude::World,
        robot: rne_ecs::Entity,
        tool: rne_ecs::Entity,
    ) -> Vec3 {
        let scene = PlanningScene::from_world(world, robot).unwrap();
        scene
            .model()
            .forward_kinematics(&[0.3, 0.0])
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation
    }

    fn positions(trajectory: &RobotTrajectory) -> Vec<Vec<f64>> {
        trajectory
            .points()
            .iter()
            .map(|point| point.positions.clone())
            .collect()
    }
}
