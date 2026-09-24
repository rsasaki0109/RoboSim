//! Motion plan requests, options, and responses.

use crate::constraints::{GoalConstraint, PathConstraint};
use crate::error::PlanningError;
use crate::trajectory::RobotTrajectory;
use rne_robot::IkOptions;

/// Tunable planner options shared across planners.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanningOptions {
    /// Maximum iterations or samples before a planner gives up.
    pub max_iterations: usize,
    /// Maximum joint-space extension per tree step, in radians or meters.
    pub step_size: f64,
    /// Probability in `[0, 1]` of sampling the goal directly.
    pub goal_bias: f64,
    /// Interpolation segments used by collision checking along a motion.
    pub collision_check_steps: usize,
    /// Deterministic random seed for sampling planners.
    pub seed: u64,
    /// Rewiring radius for optimal planners; `0.0` selects three times the step.
    pub neighbor_radius: f64,
    /// Nearest neighbours each roadmap node connects to (PRM).
    pub roadmap_neighbors: usize,
    /// Number of waypoints interpolated between two joint configurations.
    pub waypoint_count: usize,
    /// Time between adjacent interpolated waypoints in seconds.
    pub waypoint_duration_s: f64,
    /// Velocity scaling factor in `(0, 1]` applied by time parameterization.
    pub velocity_scaling_factor: f64,
    /// Per-joint acceleration limits in rad/s^2 or m/s^2, in `DoF` order.
    ///
    /// An empty vector leaves acceleration unconstrained and yields
    /// piecewise-constant velocity timing. A non-empty vector must match the
    /// model degrees of freedom and enables trapezoidal timing.
    pub acceleration_limits: Vec<f64>,
    /// Inverse kinematics options used to resolve pose goals.
    pub ik_options: IkOptions,
    /// Seeded random restarts for pose-goal inverse kinematics.
    ///
    /// Zero performs a single solve from the start seed. A positive value uses
    /// [`rne_robot::KinematicsSolver::search_position_ik`] with `seed`.
    pub ik_restarts: usize,
    /// Optional kinematics solver name; `None` selects the built-in solver.
    pub solver: Option<String>,
}

impl Default for PlanningOptions {
    fn default() -> Self {
        Self {
            max_iterations: 10_000,
            step_size: 0.2,
            goal_bias: 0.1,
            collision_check_steps: 4,
            seed: 0,
            neighbor_radius: 0.0,
            roadmap_neighbors: 10,
            waypoint_count: 50,
            waypoint_duration_s: 0.1,
            velocity_scaling_factor: 1.0,
            acceleration_limits: Vec::new(),
            ik_options: IkOptions::default(),
            ik_restarts: 0,
            solver: None,
        }
    }
}

impl PlanningOptions {
    /// Validates option ranges and finiteness.
    pub fn validate(&self) -> Result<(), PlanningError> {
        if !self.step_size.is_finite() || self.step_size <= 0.0 {
            return Err(PlanningError::Invalid(
                "step size must be finite and positive".to_string(),
            ));
        }
        if !self.goal_bias.is_finite() || !(0.0..=1.0).contains(&self.goal_bias) {
            return Err(PlanningError::Invalid(
                "goal bias must be finite and within [0, 1]".to_string(),
            ));
        }
        if self.collision_check_steps == 0 {
            return Err(PlanningError::Invalid(
                "collision check steps must be at least one".to_string(),
            ));
        }
        if self.max_iterations == 0 {
            return Err(PlanningError::Invalid(
                "max iterations must be at least one".to_string(),
            ));
        }
        if self.waypoint_count < 2 {
            return Err(PlanningError::Invalid(
                "waypoint count must be at least two".to_string(),
            ));
        }
        if !self.waypoint_duration_s.is_finite() || self.waypoint_duration_s <= 0.0 {
            return Err(PlanningError::Invalid(
                "waypoint duration must be finite and positive".to_string(),
            ));
        }
        if !self.velocity_scaling_factor.is_finite()
            || self.velocity_scaling_factor <= 0.0
            || self.velocity_scaling_factor > 1.0
        {
            return Err(PlanningError::Invalid(
                "velocity scaling factor must be within (0, 1]".to_string(),
            ));
        }
        if !self.neighbor_radius.is_finite() || self.neighbor_radius < 0.0 {
            return Err(PlanningError::Invalid(
                "neighbor radius must be finite and non-negative".to_string(),
            ));
        }
        if self.roadmap_neighbors == 0 {
            return Err(PlanningError::Invalid(
                "roadmap neighbors must be at least one".to_string(),
            ));
        }
        if self
            .acceleration_limits
            .iter()
            .any(|limit| !limit.is_finite() || *limit < 0.0)
        {
            return Err(PlanningError::Invalid(
                "acceleration limits must be finite and non-negative".to_string(),
            ));
        }
        if let Some(solver) = &self.solver {
            if solver.trim().is_empty() {
                return Err(PlanningError::Invalid(
                    "solver name must not be empty".to_string(),
                ));
            }
        }
        Ok(())
    }
}

/// A request to plan a collision-free joint-space motion.
#[derive(Clone, Debug, PartialEq)]
pub struct MotionPlanRequest {
    /// Start joint positions in degree-of-freedom order.
    pub start: Vec<f64>,
    /// Goal constraint for the motion.
    pub goal: GoalConstraint,
    /// Optional planning group name defined in the scene.
    pub group: Option<String>,
    /// Optional constraint that must hold along the whole motion.
    pub path_constraint: Option<PathConstraint>,
    /// Planner options.
    pub options: PlanningOptions,
}

impl MotionPlanRequest {
    /// Creates a motion plan request over every joint.
    pub fn new(start: Vec<f64>, goal: GoalConstraint, options: PlanningOptions) -> Self {
        Self {
            start,
            goal,
            group: None,
            path_constraint: None,
            options,
        }
    }

    /// Scopes the request to a planning group.
    pub fn with_group(mut self, group: impl Into<String>) -> Self {
        self.group = Some(group.into());
        self
    }

    /// Adds a constraint that must hold along the whole motion.
    pub fn with_path_constraint(mut self, constraint: PathConstraint) -> Self {
        self.path_constraint = Some(constraint);
        self
    }

    /// Validates the request against its own constraints.
    ///
    /// The scene's degree of freedom is checked by the planner.
    pub fn validate(&self) -> Result<(), PlanningError> {
        if self.start.is_empty() {
            return Err(PlanningError::EmptyRequest);
        }
        if self.start.iter().any(|value| !value.is_finite()) {
            return Err(PlanningError::Invalid(
                "start state must be finite".to_string(),
            ));
        }
        if let Some(group) = &self.group {
            if group.trim().is_empty() {
                return Err(PlanningError::Invalid(
                    "group name must not be empty".to_string(),
                ));
            }
        }
        if let GoalConstraint::Joint { positions } = &self.goal {
            if positions.is_empty() {
                return Err(PlanningError::Invalid(
                    "joint goal must not be empty".to_string(),
                ));
            }
            if positions.iter().any(|value| !value.is_finite()) {
                return Err(PlanningError::Invalid(
                    "joint goal must be finite".to_string(),
                ));
            }
        }
        match &self.path_constraint {
            Some(PathConstraint::Orientation { tolerance_rad, .. })
                if !tolerance_rad.is_finite() || *tolerance_rad < 0.0 =>
            {
                return Err(PlanningError::Invalid(
                    "orientation tolerance must be finite and non-negative".to_string(),
                ));
            }
            Some(PathConstraint::Position { tolerance_m, .. })
                if !tolerance_m.is_finite() || *tolerance_m < 0.0 =>
            {
                return Err(PlanningError::Invalid(
                    "position tolerance must be finite and non-negative".to_string(),
                ));
            }
            Some(PathConstraint::Visibility { tolerance_rad, .. })
                if !tolerance_rad.is_finite() || *tolerance_rad < 0.0 =>
            {
                return Err(PlanningError::Invalid(
                    "visibility tolerance must be finite and non-negative".to_string(),
                ));
            }
            _ => {}
        }
        self.options.validate()
    }
}

/// A successful motion plan.
#[derive(Clone, Debug, PartialEq)]
pub struct MotionPlanResponse {
    /// Name of the planner that produced the trajectory.
    pub planner: String,
    /// Iterations or samples consumed by the planner.
    pub iterations: usize,
    /// The planned trajectory.
    pub trajectory: RobotTrajectory,
}
