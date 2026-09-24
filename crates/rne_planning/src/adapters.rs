//! Planning request adapters and trajectory time parameterization.
//!
//! This is the RNE analogue of `MoveIt`'s `PlanningRequestAdapter` chain. An
//! adapter may transform a request before planning (`adapt_request`) or a
//! response afterwards (`adapt_response`). `FixStartStateBounds` clamps the
//! start state to joint limits and `AddTimeParameterization` retimes a
//! trajectory to respect joint velocity limits.

use crate::constraints::GoalConstraint;
use crate::error::PlanningError;
use crate::request::{MotionPlanRequest, MotionPlanResponse};
use crate::rng::DeterministicRng;
use crate::scene::PlanningScene;
use crate::trajectory::{RobotTrajectory, TrajectoryPoint};
use rne_robot::JointLimits;

/// Name of the built-in start-state bounds adapter.
pub const FIX_START_STATE_BOUNDS_ADAPTER: &str = "fix_start_state_bounds";
/// Name of the built-in workspace-bounds fix adapter.
pub const FIX_WORKSPACE_BOUNDS_ADAPTER: &str = "fix_workspace_bounds";
/// Name of the built-in workspace-bounds validation adapter.
pub const VALIDATE_WORKSPACE_BOUNDS_ADAPTER: &str = "validate_workspace_bounds";
/// Name of the built-in start-state collision adapter.
pub const FIX_START_STATE_COLLISION_ADAPTER: &str = "fix_start_state_collision";
/// Name of the built-in trajectory simplification adapter.
pub const SIMPLIFY_TRAJECTORY_ADAPTER: &str = "simplify_trajectory";
/// Name of the built-in time parameterization adapter.
pub const TIME_PARAMETERIZATION_ADAPTER: &str = "add_time_parameterization";

/// Boundary implemented by planning request adapters.
///
/// Adapters are applied in registration order before planning and again after
/// planning. Implementations must be deterministic for a given scene and
/// request.
pub trait PlanningRequestAdapter: Send + Sync + std::fmt::Debug {
    /// Adapter name used for diagnostics.
    fn name(&self) -> &str;

    /// Transforms a request before it reaches the planner.
    fn adapt_request(
        &self,
        _scene: &PlanningScene,
        request: MotionPlanRequest,
    ) -> Result<MotionPlanRequest, PlanningError> {
        Ok(request)
    }

    /// Transforms a response after the planner returns.
    fn adapt_response(
        &self,
        _scene: &PlanningScene,
        _request: &MotionPlanRequest,
        response: MotionPlanResponse,
    ) -> Result<MotionPlanResponse, PlanningError> {
        Ok(response)
    }
}

/// Clamps the start state into the model's joint limits.
///
/// This mirrors `MoveIt`'s `FixStartStateBounds`: a start configuration slightly
/// outside the declared limits is projected back onto the bounds instead of
/// rejecting the request.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FixStartStateBounds;

impl FixStartStateBounds {
    /// Creates the start-state bounds adapter.
    pub fn new() -> Self {
        Self
    }
}

impl PlanningRequestAdapter for FixStartStateBounds {
    fn name(&self) -> &str {
        FIX_START_STATE_BOUNDS_ADAPTER
    }

    fn adapt_request(
        &self,
        scene: &PlanningScene,
        mut request: MotionPlanRequest,
    ) -> Result<MotionPlanRequest, PlanningError> {
        let limits = scene.model().joint_limits();
        if request.start.len() != limits.len() {
            return Err(PlanningError::JointCountMismatch {
                provided: request.start.len(),
                expected: limits.len(),
            });
        }
        clamp_positions(&mut request.start, &limits);
        Ok(request)
    }
}

/// Retimes a trajectory to respect joint velocity limits.
///
/// This mirrors `MoveIt`'s `AddTimeParameterization` (time-optimal for piecewise
/// constant velocity; acceleration limits are not modeled because the robot
/// model only carries a maximum velocity). The requested
/// [`crate::PlanningOptions::velocity_scaling_factor`] scales every limit. When
/// the model declares no finite velocity limit, the trajectory is returned
/// unchanged.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AddTimeParameterization;

impl AddTimeParameterization {
    /// Creates the time parameterization adapter.
    pub fn new() -> Self {
        Self
    }
}

impl PlanningRequestAdapter for AddTimeParameterization {
    fn name(&self) -> &str {
        TIME_PARAMETERIZATION_ADAPTER
    }

    fn adapt_response(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
        mut response: MotionPlanResponse,
    ) -> Result<MotionPlanResponse, PlanningError> {
        let limits = scene.model().joint_limits();
        response.trajectory = time_parameterize_with_acceleration(
            &response.trajectory,
            &limits,
            &request.options.acceleration_limits,
            request.options.velocity_scaling_factor,
        )?;
        Ok(response)
    }
}

/// Clamps a goal position into the scene workspace box.
///
/// This mirrors `MoveIt`'s `FixWorkspaceBounds`. It only acts on position and
/// pose goals and does nothing when the scene has no workspace bounds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FixWorkspaceBounds;

impl FixWorkspaceBounds {
    /// Creates the workspace-bounds fix adapter.
    pub fn new() -> Self {
        Self
    }
}

impl PlanningRequestAdapter for FixWorkspaceBounds {
    fn name(&self) -> &str {
        FIX_WORKSPACE_BOUNDS_ADAPTER
    }

    fn adapt_request(
        &self,
        scene: &PlanningScene,
        mut request: MotionPlanRequest,
    ) -> Result<MotionPlanRequest, PlanningError> {
        let Some((min, max)) = scene.workspace_bounds() else {
            return Ok(request);
        };
        match &mut request.goal {
            GoalConstraint::Position { target, .. } => {
                *target = clamp_point(*target, min, max);
            }
            GoalConstraint::Pose { target, .. } => {
                target.translation = clamp_point(target.translation, min, max);
            }
            GoalConstraint::Joint { .. } | GoalConstraint::Orientation { .. } => {}
        }
        Ok(request)
    }
}

/// Rejects a goal position outside the scene workspace box.
///
/// This mirrors `MoveIt`'s `ValidateWorkspaceBounds`; it does nothing when the
/// scene has no workspace bounds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ValidateWorkspaceBounds;

impl ValidateWorkspaceBounds {
    /// Creates the workspace-bounds validation adapter.
    pub fn new() -> Self {
        Self
    }
}

impl PlanningRequestAdapter for ValidateWorkspaceBounds {
    fn name(&self) -> &str {
        VALIDATE_WORKSPACE_BOUNDS_ADAPTER
    }

    fn adapt_request(
        &self,
        scene: &PlanningScene,
        request: MotionPlanRequest,
    ) -> Result<MotionPlanRequest, PlanningError> {
        let Some((min, max)) = scene.workspace_bounds() else {
            return Ok(request);
        };
        let target = match &request.goal {
            GoalConstraint::Position { target, .. } => Some(*target),
            GoalConstraint::Pose { target, .. } => Some(target.translation),
            GoalConstraint::Joint { .. } | GoalConstraint::Orientation { .. } => None,
        };
        if let Some(target) = target {
            if !within_bounds(target, min, max) {
                return Err(PlanningError::Invalid(
                    "goal position is outside the workspace bounds".to_string(),
                ));
            }
        }
        Ok(request)
    }
}

fn clamp_point(value: rne_math::Vec3, min: rne_math::Vec3, max: rne_math::Vec3) -> rne_math::Vec3 {
    rne_math::Vec3::new(
        value.x.clamp(min.x, max.x),
        value.y.clamp(min.y, max.y),
        value.z.clamp(min.z, max.z),
    )
}

fn within_bounds(value: rne_math::Vec3, min: rne_math::Vec3, max: rne_math::Vec3) -> bool {
    value.x >= min.x
        && value.x <= max.x
        && value.y >= min.y
        && value.y <= max.y
        && value.z >= min.z
        && value.z <= max.z
}

/// Moves a colliding start state to a nearby collision-free one.
///
/// This mirrors `MoveIt`'s `FixStartStateCollision`: when the start state is in
/// collision, the adapter draws seeded per-joint perturbations up to
/// `perturbation` (radians or meters) and keeps the first valid state. If none
/// is found the request is returned unchanged, so the planner still reports
/// `StartInCollision`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixStartStateCollision {
    /// Number of perturbation attempts.
    pub attempts: usize,
    /// Maximum per-joint perturbation in radians or meters.
    pub perturbation: f64,
}

impl Default for FixStartStateCollision {
    fn default() -> Self {
        Self {
            attempts: 20,
            perturbation: 0.1,
        }
    }
}

impl FixStartStateCollision {
    /// Creates a start-state collision adapter.
    pub fn new(attempts: usize, perturbation: f64) -> Self {
        Self {
            attempts,
            perturbation,
        }
    }
}

impl PlanningRequestAdapter for FixStartStateCollision {
    fn name(&self) -> &str {
        FIX_START_STATE_COLLISION_ADAPTER
    }

    fn adapt_request(
        &self,
        scene: &PlanningScene,
        mut request: MotionPlanRequest,
    ) -> Result<MotionPlanRequest, PlanningError> {
        if scene.is_state_valid(&request.start)? {
            return Ok(request);
        }
        let dof = scene.model().dof();
        if request.start.len() != dof
            || self.attempts == 0
            || !self.perturbation.is_finite()
            || self.perturbation <= 0.0
        {
            return Ok(request);
        }
        let limits = scene.model().joint_limits();
        let mut rng = DeterministicRng::new(request.options.seed ^ 0x5DEE_CE66_D1CE_F00D);
        for _ in 0..self.attempts {
            let mut candidate = request.start.clone();
            for (index, value) in candidate.iter_mut().enumerate() {
                let limit = limits[index];
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
                let delta = (rng.next_f64() * 2.0 - 1.0) * self.perturbation;
                *value = (*value + delta).clamp(lower, upper);
            }
            if scene.is_state_valid(&candidate)? {
                request.start = candidate;
                return Ok(request);
            }
        }
        Ok(request)
    }
}

/// Removes redundant waypoints whose shortcut is collision free.
///
/// This mirrors a `MoveIt` trajectory simplification response adapter. It greedily
/// keeps the farthest directly reachable waypoint and re-times the result with
/// `waypoint_duration_s`; a following time-parameterization adapter can retime it
/// again.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SimplifyTrajectory;

impl SimplifyTrajectory {
    /// Creates the trajectory simplification adapter.
    pub fn new() -> Self {
        Self
    }
}

impl PlanningRequestAdapter for SimplifyTrajectory {
    fn name(&self) -> &str {
        SIMPLIFY_TRAJECTORY_ADAPTER
    }

    fn adapt_response(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
        mut response: MotionPlanResponse,
    ) -> Result<MotionPlanResponse, PlanningError> {
        let points = response.trajectory.points().to_vec();
        if points.len() < 3 {
            return Ok(response);
        }
        let steps = request.options.collision_check_steps;
        let path = request.path_constraint.as_ref();
        let mut kept = vec![0usize];
        let mut current = 0usize;
        while current + 1 < points.len() {
            let mut next = points.len() - 1;
            while next > current + 1
                && !scene.is_motion_valid_with(
                    &points[current].positions,
                    &points[next].positions,
                    steps,
                    path,
                )?
            {
                next -= 1;
            }
            kept.push(next);
            current = next;
        }
        if kept.len() == points.len() {
            return Ok(response);
        }
        let waypoints: Vec<Vec<f64>> = kept
            .iter()
            .map(|&index| points[index].positions.clone())
            .collect();
        response.trajectory =
            RobotTrajectory::from_waypoints(&waypoints, request.options.waypoint_duration_s)?;
        Ok(response)
    }
}

fn clamp_positions(positions: &mut [f64], limits: &[JointLimits]) {
    for (value, limit) in positions.iter_mut().zip(limits) {
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
        *value = value.clamp(lower, upper);
    }
}

/// Retimes a trajectory with the given joint velocity limits and scaling.
///
/// Acceleration is left unconstrained; use
/// [`time_parameterize_with_acceleration`] for trapezoidal timing.
pub fn time_parameterize(
    trajectory: &RobotTrajectory,
    limits: &[JointLimits],
    velocity_scaling_factor: f64,
) -> Result<RobotTrajectory, PlanningError> {
    time_parameterize_with_acceleration(trajectory, limits, &[], velocity_scaling_factor)
}

/// Retimes a trajectory with joint velocity and acceleration limits.
///
/// Each segment is timed with a stop-and-go trapezoidal profile per joint when
/// an acceleration limit is present, otherwise with constant velocity. An empty
/// `acceleration_limits` slice leaves acceleration unconstrained. The maximum
/// per-joint segment time is used, so every joint stays within its limits. When
/// no finite velocity or acceleration limit is declared, the input timing is
/// preserved.
pub fn time_parameterize_with_acceleration(
    trajectory: &RobotTrajectory,
    limits: &[JointLimits],
    acceleration_limits: &[f64],
    velocity_scaling_factor: f64,
) -> Result<RobotTrajectory, PlanningError> {
    let points = trajectory.points();
    if points.len() < 2 {
        return Ok(trajectory.clone());
    }
    let dof = points[0].positions.len();
    if dof != limits.len() {
        return Err(PlanningError::JointCountMismatch {
            provided: dof,
            expected: limits.len(),
        });
    }
    if !acceleration_limits.is_empty() && acceleration_limits.len() != dof {
        return Err(PlanningError::JointCountMismatch {
            provided: acceleration_limits.len(),
            expected: dof,
        });
    }
    let any_velocity = limits
        .iter()
        .any(|limit| limit.max_velocity.is_finite() && limit.max_velocity > 0.0);
    let any_acceleration = acceleration_limits
        .iter()
        .any(|limit| limit.is_finite() && *limit > 0.0);
    if !any_velocity && !any_acceleration {
        return Ok(trajectory.clone());
    }

    let mut retimed = Vec::with_capacity(points.len());
    let mut time = points[0].time_from_start_s;
    retimed.push(points[0].clone());
    for window in points.windows(2) {
        let mut required_s = 0.0_f64;
        for (index, limit) in limits.iter().enumerate() {
            let max_velocity = finite_limit(limit.max_velocity) * velocity_scaling_factor;
            let max_acceleration = acceleration_limits
                .get(index)
                .map_or(f64::INFINITY, |limit| {
                    finite_limit(*limit) * velocity_scaling_factor
                });
            let delta = (window[1].positions[index] - window[0].positions[index]).abs();
            let segment_s = trapezoid_time(delta, max_velocity, max_acceleration);
            if segment_s > required_s {
                required_s = segment_s;
            }
        }
        time += required_s;
        retimed.push(TrajectoryPoint {
            positions: window[1].positions.clone(),
            time_from_start_s: time,
        });
    }
    RobotTrajectory::new(retimed)
}

fn finite_limit(value: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        f64::INFINITY
    }
}

/// Time for a rest-to-rest trapezoidal move over `distance`.
///
/// Infinite velocity or acceleration degenerates to constant velocity or
/// triangular profile respectively. Zero distance takes zero time.
fn trapezoid_time(distance: f64, max_velocity: f64, max_acceleration: f64) -> f64 {
    if distance <= 0.0 {
        return 0.0;
    }
    if max_acceleration.is_infinite() {
        return if max_velocity.is_infinite() {
            0.0
        } else {
            distance / max_velocity
        };
    }
    if max_velocity.is_infinite() {
        return 2.0 * (distance / max_acceleration).sqrt();
    }
    let accel_distance = max_velocity * max_velocity / (2.0 * max_acceleration);
    if distance >= 2.0 * accel_distance {
        2.0 * max_velocity / max_acceleration + (distance - 2.0 * accel_distance) / max_velocity
    } else {
        2.0 * (distance / max_acceleration).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::GoalConstraint;
    use crate::request::PlanningOptions;
    use crate::test_support::arm_world;
    use rne_math::Vec3;
    use rne_robot::{CollisionPrimitive, CollisionWorld, CollisionWorldObject};

    fn limits(max_velocity: &[f64]) -> Vec<JointLimits> {
        max_velocity
            .iter()
            .map(|&velocity| JointLimits {
                max_velocity: velocity,
                ..JointLimits::default()
            })
            .collect()
    }

    #[test]
    fn time_parameterization_respects_velocity_limits() {
        let trajectory =
            RobotTrajectory::from_waypoints(&[vec![0.0, 0.0], vec![1.0, 0.0], vec![1.0, 0.5]], 0.1)
                .unwrap();
        let retimed = time_parameterize(&trajectory, &limits(&[1.0, 2.0]), 1.0).unwrap();
        assert_eq!(retimed.len(), 3);
        assert!((retimed.points()[1].time_from_start_s - 1.0).abs() < 1.0e-12);
        assert!((retimed.points()[2].time_from_start_s - 1.25).abs() < 1.0e-12);
        assert!((retimed.duration_s() - 1.25).abs() < 1.0e-12);
    }

    #[test]
    fn velocity_scaling_slows_the_trajectory() {
        let trajectory = RobotTrajectory::from_waypoints(&[vec![0.0], vec![1.0]], 0.01).unwrap();
        let full = time_parameterize(&trajectory, &limits(&[1.0]), 1.0).unwrap();
        let half = time_parameterize(&trajectory, &limits(&[1.0]), 0.5).unwrap();
        assert!((full.duration_s() - 1.0).abs() < 1.0e-12);
        assert!((half.duration_s() - 2.0).abs() < 1.0e-12);
    }

    #[test]
    fn missing_velocity_limits_keep_original_timing() {
        let trajectory = RobotTrajectory::from_waypoints(&[vec![0.0], vec![3.0]], 0.25).unwrap();
        let retimed = time_parameterize(&trajectory, &limits(&[f64::INFINITY]), 1.0).unwrap();
        assert!((retimed.duration_s() - 0.25).abs() < 1.0e-12);
    }

    #[test]
    fn clamp_positions_respects_finite_bounds() {
        let mut positions = vec![5.0, -5.0, 0.5];
        clamp_positions(
            &mut positions,
            &[
                JointLimits::default(),
                JointLimits::default(),
                JointLimits::default(),
            ],
        );
        assert_eq!(positions, vec![5.0, -5.0, 0.5]);

        let bounded = JointLimits {
            lower: -1.0,
            upper: 1.0,
            ..JointLimits::default()
        };
        let mut positions = vec![5.0, -5.0];
        clamp_positions(&mut positions, &[bounded, bounded]);
        assert_eq!(positions, vec![1.0, -1.0]);
    }

    #[test]
    fn acceleration_limits_add_trapezoidal_timing() {
        // distance 10, vmax 1, amax 4: t = 2*1/4 + (10 - 0.25)/1 = 10.25
        let trajectory = RobotTrajectory::from_waypoints(&[vec![0.0], vec![10.0]], 0.01).unwrap();
        let retimed =
            time_parameterize_with_acceleration(&trajectory, &limits(&[1.0]), &[4.0], 1.0).unwrap();
        assert!((retimed.duration_s() - 10.25).abs() < 1.0e-9);
    }

    #[test]
    fn short_moves_use_a_triangular_profile() {
        // distance 1, vmax 10, amax 4: t = 2*sqrt(1/4) = 1
        let trajectory = RobotTrajectory::from_waypoints(&[vec![0.0], vec![1.0]], 0.01).unwrap();
        let retimed =
            time_parameterize_with_acceleration(&trajectory, &limits(&[10.0]), &[4.0], 1.0)
                .unwrap();
        assert!((retimed.duration_s() - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn acceleration_scaling_slows_the_move() {
        // vmax 1, amax 4 at scaling 0.5 -> v 0.5, a 2: t = 0.5 + (10 - 0.125)/0.5 = 20.25
        let trajectory = RobotTrajectory::from_waypoints(&[vec![0.0], vec![10.0]], 0.01).unwrap();
        let retimed =
            time_parameterize_with_acceleration(&trajectory, &limits(&[1.0]), &[4.0], 0.5).unwrap();
        assert!((retimed.duration_s() - 20.25).abs() < 1.0e-9);
    }

    #[test]
    fn fix_start_state_collision_finds_free_state() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_collision_world(CollisionWorld::with_objects(vec![
                CollisionWorldObject::new(CollisionPrimitive::Sphere {
                    center_m: Vec3::new(2.0, 0.0, 0.0),
                    radius_m: 0.2,
                }),
            ]));
        assert!(!scene.is_state_valid(&[0.0, 0.0]).unwrap());

        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.5, 0.5]),
            PlanningOptions::default(),
        );
        let adapted = FixStartStateCollision::new(50, 0.5)
            .adapt_request(&scene, request)
            .unwrap();
        assert_ne!(adapted.start, vec![0.0, 0.0]);
        assert!(scene.is_state_valid(&adapted.start).unwrap());
    }

    #[test]
    fn simplify_trajectory_removes_redundant_waypoints() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let waypoints: Vec<Vec<f64>> = (0..5).map(|index| vec![0.1 * index as f64, 0.0]).collect();
        let trajectory = RobotTrajectory::from_waypoints(&waypoints, 0.1).unwrap();
        let response = MotionPlanResponse {
            planner: "test".to_string(),
            iterations: 0,
            trajectory,
        };
        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::joint(vec![0.4, 0.0]),
            PlanningOptions::default(),
        );
        let simplified = SimplifyTrajectory::new()
            .adapt_response(&scene, &request, response)
            .unwrap();
        assert_eq!(simplified.trajectory.len(), 2);
    }

    #[test]
    fn fix_workspace_bounds_clamps_goal_position() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_workspace_bounds(Vec3::splat(-1.0), Vec3::splat(1.0));
        let request = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::position(tool, Vec3::new(5.0, 0.0, 0.0)),
            PlanningOptions::default(),
        );
        let adapted = FixWorkspaceBounds::new()
            .adapt_request(&scene, request)
            .unwrap();
        match adapted.goal {
            GoalConstraint::Position { target, .. } => {
                assert_eq!(target, Vec3::new(1.0, 0.0, 0.0));
            }
            other => panic!("expected position goal, got {other:?}"),
        }
    }

    #[test]
    fn validate_workspace_bounds_rejects_outside_only() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot)
            .unwrap()
            .with_workspace_bounds(Vec3::splat(-1.0), Vec3::splat(1.0));
        let outside = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::position(tool, Vec3::new(5.0, 0.0, 0.0)),
            PlanningOptions::default(),
        );
        assert!(ValidateWorkspaceBounds::new()
            .adapt_request(&scene, outside)
            .is_err());
        let inside = MotionPlanRequest::new(
            vec![0.0, 0.0],
            GoalConstraint::position(tool, Vec3::new(0.5, 0.0, 0.0)),
            PlanningOptions::default(),
        );
        assert!(ValidateWorkspaceBounds::new()
            .adapt_request(&scene, inside)
            .is_ok());
    }

    #[test]
    fn mismatched_acceleration_length_is_rejected() {
        let trajectory =
            RobotTrajectory::from_waypoints(&[vec![0.0, 0.0], vec![1.0, 1.0]], 0.01).unwrap();
        assert!(matches!(
            time_parameterize_with_acceleration(&trajectory, &limits(&[1.0, 1.0]), &[4.0], 1.0),
            Err(PlanningError::JointCountMismatch { .. })
        ));
    }
}
