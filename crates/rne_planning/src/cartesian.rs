//! Cartesian waypoint path planning.
//!
//! This is the RNE analogue of `MoveIt`'s `CartesianInterpolator` and the Pilz
//! `LIN` motion: a sequence of end-link poses is interpolated in Cartesian
//! space, each interpolated pose is solved with inverse kinematics seeded from
//! the previous configuration, and the resulting joint path is collision
//! checked. Planning may stop early and report the completed fraction, matching
//! `MoveIt`'s `computeCartesianPath` return shape.

use crate::error::PlanningError;
use crate::request::PlanningOptions;
use crate::scene::PlanningScene;
use crate::trajectory::RobotTrajectory;
use rne_ecs::Entity;
use rne_math::{Pose3, Vec3};
use rne_robot::{IkRequest, KinematicsError, DAMPED_LEAST_SQUARES_SOLVER};

/// A Cartesian waypoint path request.
#[derive(Clone, Debug, PartialEq)]
pub struct CartesianPathRequest {
    /// Start joint positions in degree-of-freedom order.
    pub start: Vec<f64>,
    /// End link to follow the waypoints.
    pub end_link: Entity,
    /// Absolute end-link poses to reach in order.
    pub waypoints: Vec<Pose3>,
    /// Maximum translation between adjacent interpolated poses, in meters.
    pub max_translation_step_m: f64,
    /// Maximum rotation between adjacent interpolated poses, in radians.
    pub max_rotation_step_rad: f64,
    /// When true, solve full pose; otherwise solve position only.
    pub solve_orientation: bool,
    /// Optional planning group name scoping which joints may move.
    pub group: Option<String>,
    /// Planner options used for the kinematics solver and collision checking.
    pub options: PlanningOptions,
}

impl CartesianPathRequest {
    /// Creates a request with a 1 cm / 0.1 rad interpolation resolution.
    pub fn new(
        start: Vec<f64>,
        end_link: Entity,
        waypoints: Vec<Pose3>,
        options: PlanningOptions,
    ) -> Self {
        Self {
            start,
            end_link,
            waypoints,
            max_translation_step_m: 0.01,
            max_rotation_step_rad: 0.1,
            solve_orientation: true,
            group: None,
            options,
        }
    }

    /// Validates the request against the model degrees of freedom.
    pub fn validate(&self, dof: usize) -> Result<(), PlanningError> {
        if self.start.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: self.start.len(),
                expected: dof,
            });
        }
        if self.start.iter().any(|value| !value.is_finite()) {
            return Err(PlanningError::Invalid(
                "start state must be finite".to_string(),
            ));
        }
        if self.waypoints.is_empty() {
            return Err(PlanningError::Invalid(
                "cartesian path requires at least one waypoint".to_string(),
            ));
        }
        for waypoint in &self.waypoints {
            if !waypoint.translation.is_finite() || !waypoint.rotation.is_finite() {
                return Err(PlanningError::Invalid(
                    "cartesian waypoints must be finite".to_string(),
                ));
            }
        }
        if !self.max_translation_step_m.is_finite() || self.max_translation_step_m <= 0.0 {
            return Err(PlanningError::Invalid(
                "max translation step must be finite and positive".to_string(),
            ));
        }
        if !self.max_rotation_step_rad.is_finite() || self.max_rotation_step_rad <= 0.0 {
            return Err(PlanningError::Invalid(
                "max rotation step must be finite and positive".to_string(),
            ));
        }
        self.options.validate()
    }
}

/// Result of a Cartesian path query.
#[derive(Clone, Debug, PartialEq)]
pub struct CartesianPathResult {
    /// The planned joint trajectory, possibly shorter than the full path.
    pub trajectory: RobotTrajectory,
    /// Fraction of the waypoint path completed, in `[0, 1]`.
    pub fraction: f64,
    /// Number of Cartesian waypoints fully reached.
    pub completed_waypoints: usize,
}

impl CartesianPathResult {
    /// Whether every waypoint was reached.
    pub fn is_complete(&self) -> bool {
        self.fraction >= 1.0 - 1.0e-9
    }
}

/// Planner that follows Cartesian waypoints with inverse kinematics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CartesianPathPlanner;

impl CartesianPathPlanner {
    /// Creates the Cartesian path planner.
    pub fn new() -> Self {
        Self
    }

    /// Plans a collision-free Cartesian path, stopping at the first failure.
    pub fn plan(
        &self,
        scene: &PlanningScene,
        request: &CartesianPathRequest,
    ) -> Result<CartesianPathResult, PlanningError> {
        let dof = scene.model().dof();
        request.validate(dof)?;
        if !scene.is_state_valid(&request.start)? {
            return Err(PlanningError::StartInCollision);
        }

        let solver_name = request
            .options
            .solver
            .as_deref()
            .unwrap_or(DAMPED_LEAST_SQUARES_SOLVER);
        let solver = scene
            .solvers()
            .get(solver_name)
            .ok_or_else(|| PlanningError::UnknownSolver(solver_name.to_string()))?;
        let active_mask = match &request.group {
            None => None,
            Some(name) => {
                let group = scene
                    .group(name)
                    .ok_or_else(|| PlanningError::UnknownGroup(name.clone()))?;
                Some(group.active_mask(dof))
            }
        };

        let mut configs = vec![request.start.clone()];
        let mut current = request.start.clone();
        let mut previous_pose = end_pose(scene, &current, request.end_link)?;
        let total = request.waypoints.len();
        let mut completed = 0usize;
        let mut partial = 0.0_f64;

        'waypoints: for waypoint in &request.waypoints {
            let translation = previous_pose.translation.distance(waypoint.translation);
            let rotation = previous_pose
                .rotation
                .angle_between(waypoint.rotation)
                .abs();
            let segments = (translation / request.max_translation_step_m)
                .ceil()
                .max((rotation / request.max_rotation_step_rad).ceil())
                .max(1.0) as usize;

            let mut last_ok = 0usize;
            for step in 1..=segments {
                let interpolation = step as f64 / segments as f64;
                let target = interpolate_pose(&previous_pose, waypoint, interpolation);
                let mut ik_options = request.options.ik_options;
                ik_options.solve_orientation = request.solve_orientation;
                let mut ik_request =
                    IkRequest::new(request.end_link, target, current.clone(), ik_options);
                if let Some(mask) = &active_mask {
                    ik_request = ik_request.with_active_dof(mask.clone());
                }
                let Ok(solution) = solver.solve(scene.model(), &ik_request) else {
                    break;
                };
                if !scene.is_motion_valid(
                    &current,
                    &solution.joint_positions,
                    request.options.collision_check_steps,
                )? {
                    break;
                }
                current = solution.joint_positions;
                configs.push(current.clone());
                last_ok = step;
            }

            if last_ok == segments {
                completed += 1;
                previous_pose = *waypoint;
            } else {
                partial = last_ok as f64 / segments as f64;
                break 'waypoints;
            }
        }

        let fraction = (completed as f64 + partial) / total as f64;
        let trajectory =
            RobotTrajectory::from_waypoints(&configs, request.options.waypoint_duration_s)?;
        Ok(CartesianPathResult {
            trajectory,
            fraction,
            completed_waypoints: completed,
        })
    }
}

/// Convenience wrapper that runs [`CartesianPathPlanner`].
pub fn compute_cartesian_path(
    scene: &PlanningScene,
    request: &CartesianPathRequest,
) -> Result<CartesianPathResult, PlanningError> {
    CartesianPathPlanner::new().plan(scene, request)
}

fn end_pose(
    scene: &PlanningScene,
    positions: &[f64],
    end_link: Entity,
) -> Result<Pose3, PlanningError> {
    let forward = scene.model().forward_kinematics(positions)?;
    let transform = forward
        .link_transform(end_link)
        .ok_or(PlanningError::Kinematics(KinematicsError::UnknownLink(
            end_link,
        )))?;
    Ok(Pose3 {
        translation: transform.translation,
        rotation: transform.rotation,
    })
}

fn interpolate_pose(from: &Pose3, to: &Pose3, interpolation: f64) -> Pose3 {
    Pose3 {
        translation: from.translation.lerp(to.translation, interpolation),
        rotation: from.rotation.slerp(to.rotation, interpolation).normalize(),
    }
}

/// Builds end-link waypoints along the circular arc through three poses.
///
/// This is the RNE analogue of the Pilz `CIRC` motion: the arc passes from
/// `start` through `via` to `goal`, and orientation is slerped from start to
/// goal with the same parameter as the arc angle. The returned list excludes
/// `start` and ends at `goal`, so it can be passed directly to
/// [`CartesianPathRequest::waypoints`]. Collinear points have no circle and
/// return [`PlanningError::Invalid`].
pub fn circular_waypoints(
    start: &Pose3,
    via: &Pose3,
    goal: &Pose3,
    max_translation_step_m: f64,
    max_rotation_step_rad: f64,
) -> Result<Vec<Pose3>, PlanningError> {
    let p0 = start.translation;
    let a = via.translation - p0;
    let b = goal.translation - p0;
    let normal = a.cross(b);
    if normal.length_squared() <= 1.0e-18 {
        return Err(PlanningError::Invalid(
            "circular waypoints require three non-collinear points".to_string(),
        ));
    }
    let normal_squared = normal.length_squared();
    let center = p0
        + (a.length_squared() * b - b.length_squared() * a).cross(normal) / (2.0 * normal_squared);
    let radius = (p0 - center).length();
    if radius <= 1.0e-12 {
        return Err(PlanningError::Invalid(
            "circular waypoints need a non-degenerate circle".to_string(),
        ));
    }

    let normal_unit = normal.normalize_or_zero();
    let u = (p0 - center).normalize_or_zero();
    let v = normal_unit.cross(u);

    let angle_of = |point: Vec3| {
        let delta = point - center;
        let mut angle = delta.dot(v).atan2(delta.dot(u));
        if angle < 0.0 {
            angle += std::f64::consts::TAU;
        }
        angle
    };
    let angle_via = angle_of(via.translation);
    let angle_goal = angle_of(goal.translation);
    let sweep = if angle_via <= angle_goal {
        angle_goal
    } else {
        angle_goal - std::f64::consts::TAU
    };

    let rotation = (sweep.abs() / max_rotation_step_rad.max(f64::MIN_POSITIVE)).ceil();
    let translation = (sweep.abs() * radius / max_translation_step_m.max(f64::MIN_POSITIVE)).ceil();
    let steps = rotation.max(translation).max(1.0) as usize;

    let mut waypoints = Vec::with_capacity(steps);
    for step in 1..=steps {
        let fraction = step as f64 / steps as f64;
        let angle = sweep * fraction;
        let translation = center + (u * (radius * angle.cos())) + (v * (radius * angle.sin()));
        let orientation = start.rotation.slerp(goal.rotation, fraction).normalize();
        waypoints.push(Pose3 {
            translation,
            rotation: orientation,
        });
    }
    Ok(waypoints)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::arm_world;
    use rne_math::Quat;

    fn pose_for(scene: &PlanningScene, end_link: Entity, q: &[f64]) -> Pose3 {
        let forward = scene.model().forward_kinematics(q).unwrap();
        let transform = forward.link_transform(end_link).unwrap();
        Pose3 {
            translation: transform.translation,
            rotation: transform.rotation,
        }
    }

    #[test]
    fn follows_reachable_cartesian_positions() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let waypoints = vec![
            pose_for(&scene, tool, &[0.5, 0.3]),
            pose_for(&scene, tool, &[0.7, 0.45]),
        ];
        let request = CartesianPathRequest {
            start: vec![0.2, 0.2],
            end_link: tool,
            waypoints: waypoints.clone(),
            max_translation_step_m: 0.02,
            max_rotation_step_rad: 0.05,
            solve_orientation: false,
            group: None,
            options: PlanningOptions::default(),
        };

        let result = CartesianPathPlanner::new().plan(&scene, &request).unwrap();
        assert!(result.is_complete(), "fraction={}", result.fraction);
        assert_eq!(result.completed_waypoints, 2);
        assert!(result.trajectory.len() >= 3);

        let forward = scene
            .model()
            .forward_kinematics(result.trajectory.goal_positions().unwrap())
            .unwrap();
        let reached = forward.link_transform(tool).unwrap().translation;
        assert!((reached - waypoints[1].translation).length() < 1.0e-3);
    }

    #[test]
    fn full_pose_cartesian_reports_partial_for_nonredundant_arm() {
        // A straight Cartesian line between two reachable poses of a 2R arm
        // generally leaves the reachable pose manifold, so full-pose motion
        // stops early and reports the completed fraction instead of failing.
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = CartesianPathRequest {
            start: vec![0.2, 0.2],
            end_link: tool,
            waypoints: vec![pose_for(&scene, tool, &[0.5, 0.3])],
            max_translation_step_m: 0.02,
            max_rotation_step_rad: 0.05,
            solve_orientation: true,
            group: None,
            options: PlanningOptions::default(),
        };

        let result = CartesianPathPlanner::new().plan(&scene, &request).unwrap();
        assert!(result.fraction < 1.0);
        assert_eq!(result.completed_waypoints, 0);
    }

    #[test]
    fn reports_partial_fraction_for_unreachable_waypoint() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request = CartesianPathRequest {
            start: vec![0.2, 0.2],
            end_link: tool,
            waypoints: vec![Pose3 {
                translation: Vec3::new(5.0, 0.0, 0.0),
                rotation: Quat::IDENTITY,
            }],
            max_translation_step_m: 0.05,
            max_rotation_step_rad: 0.1,
            solve_orientation: false,
            group: None,
            options: PlanningOptions::default(),
        };

        let result = CartesianPathPlanner::new().plan(&scene, &request).unwrap();
        assert!(result.fraction < 1.0);
        assert_eq!(result.completed_waypoints, 0);
        assert_eq!(result.trajectory.len(), 1);
    }

    #[test]
    fn circular_waypoints_trace_an_arc_through_the_via_pose() {
        let start = Pose3 {
            translation: Vec3::new(1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        let via = Pose3 {
            translation: Vec3::new(0.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        let goal = Pose3 {
            translation: Vec3::new(-1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        let waypoints = circular_waypoints(&start, &via, &goal, 0.05, 0.1).unwrap();
        assert!(!waypoints.is_empty());
        for waypoint in &waypoints {
            assert!((waypoint.translation.length() - 1.0).abs() < 1.0e-9);
        }
        assert!(waypoints
            .iter()
            .any(|waypoint| (waypoint.translation - via.translation).length() < 0.1));
        assert!((waypoints.last().unwrap().translation - goal.translation).length() < 1.0e-9);
    }

    #[test]
    fn circular_waypoints_reject_collinear_points() {
        let start = Pose3 {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let via = Pose3 {
            translation: Vec3::new(1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        let goal = Pose3 {
            translation: Vec3::new(2.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        assert!(matches!(
            circular_waypoints(&start, &via, &goal, 0.05, 0.1),
            Err(PlanningError::Invalid(_))
        ));
    }

    #[test]
    fn circular_cartesian_path_reaches_the_goal() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let start = pose_for(&scene, tool, &[0.0, 0.0]);
        let via = pose_for(&scene, tool, &[0.5, 0.0]);
        let goal = pose_for(&scene, tool, &[1.0, 0.0]);
        let waypoints = circular_waypoints(&start, &via, &goal, 0.05, 0.1).unwrap();
        let request = CartesianPathRequest {
            start: vec![0.0, 0.0],
            end_link: tool,
            waypoints,
            max_translation_step_m: 0.05,
            max_rotation_step_rad: 0.1,
            solve_orientation: false,
            group: None,
            options: PlanningOptions::default(),
        };
        let result = CartesianPathPlanner::new().plan(&scene, &request).unwrap();
        assert!(result.is_complete(), "fraction={}", result.fraction);
        let reached = scene
            .model()
            .forward_kinematics(result.trajectory.goal_positions().unwrap())
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        assert!((reached - goal.translation).length() < 1.0e-3);
    }

    #[test]
    fn rejects_empty_waypoints() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let request =
            CartesianPathRequest::new(vec![0.0, 0.0], tool, Vec::new(), PlanningOptions::default());
        assert!(matches!(
            CartesianPathPlanner::new().plan(&scene, &request),
            Err(PlanningError::Invalid(_))
        ));
    }
}
