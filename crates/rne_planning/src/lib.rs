//! Backend-neutral joint-space motion planning for Robot Native Engine.
//!
//! `rne_planning` provides a native, MoveIt-inspired planning stack without any
//! `MoveIt`, ROS, physics-backend, or renderer dependency:
//!
//! - [`PlanningScene`] combines the robot's [`rne_robot::KinematicModel`], its
//!   self-collision checker, static world collision objects, and the kinematics
//!   solvers used to resolve pose goals. [`PlanningGroup`] names a chain or
//!   joint subset so planning, sampling, and inverse kinematics are scoped to
//!   those joints while the rest hold their start configuration.
//! - [`MotionPlanRequest`] / [`MotionPlanResponse`] carry start state, goal
//!   constraints, options, and the resulting [`RobotTrajectory`].
//! - [`MotionPlanner`] is the swappable planner boundary. Built-ins are
//!   [`JointInterpolationPlanner`], [`RrtConnectPlanner`], [`RrtStarPlanner`],
//!   [`PrmPlanner`], [`HybridPlanner`] (global plan plus CHOMP refinement),
//!   [`StompPlanner`], [`InformedRrtStarPlanner`], and [`BitStarPlanner`]
//!   (batch informed), addressed by name through [`PlannerRegistry`] and
//!   dispatched by [`PlanningPipeline`].
//! - [`GoalConstraint`] supports joint, pose, position, and orientation goals;
//!   [`PathConstraint`] must hold at every configuration along a motion
//!   (orientation, position, or visibility), and [`ConstraintSampler`] draws
//!   configurations satisfying a goal constraint.
//! - [`parse_srdf`] and [`PlanningScene::apply_srdf`] load planning groups from
//!   a `MoveIt` SRDF document.
//! - [`CartesianPathPlanner`] follows end-link waypoints with inverse
//!   kinematics and reports the completed fraction; [`circular_waypoints`]
//!   builds a circular arc (Pilz `CIRC`) for it.
//! - [`optimize_trajectory`] is a CHOMP-inspired trajectory optimizer that trades
//!   smoothness against obstacle clearance.
//! - [`PlanningRequestAdapter`] is the pre/post-processing boundary.
//!   [`FixStartStateBounds`] clamps the start state, [`FixWorkspaceBounds`] and
//!   [`ValidateWorkspaceBounds`] handle the scene workspace box,
//!   [`FixStartStateCollision`] moves the start out of collision,
//!   [`SimplifyTrajectory`] removes redundant waypoints, and
//!   [`AddTimeParameterization`] retimes trajectories to joint velocity and
//!   acceleration limits.
//!
//! All sampling planners use an explicit seed from [`PlanningOptions::seed`],
//! so a scene and request are reproducible across runs.

#![deny(missing_docs)]

mod adapters;
mod cartesian;
mod constraints;
mod error;
mod group;
mod optimize;
mod pipeline;
mod planner;
mod planners;
mod request;
mod rng;
mod sampler;
mod scene;
mod srdf;
mod trajectory;

#[cfg(test)]
mod test_support;

pub use adapters::{
    time_parameterize, time_parameterize_with_acceleration, AddTimeParameterization,
    FixStartStateBounds, FixStartStateCollision, FixWorkspaceBounds, PlanningRequestAdapter,
    SimplifyTrajectory, ValidateWorkspaceBounds, FIX_START_STATE_BOUNDS_ADAPTER,
    FIX_START_STATE_COLLISION_ADAPTER, FIX_WORKSPACE_BOUNDS_ADAPTER, SIMPLIFY_TRAJECTORY_ADAPTER,
    TIME_PARAMETERIZATION_ADAPTER, VALIDATE_WORKSPACE_BOUNDS_ADAPTER,
};
pub use cartesian::{
    circular_waypoints, compute_cartesian_path, CartesianPathPlanner, CartesianPathRequest,
    CartesianPathResult,
};
pub use constraints::{GoalConstraint, PathConstraint};
pub use error::PlanningError;
pub use group::PlanningGroup;
pub use optimize::{optimize_trajectory, trajectory_cost, trajectory_is_feasible, ChompOptions};
pub use pipeline::PlanningPipeline;
pub use planner::{MotionPlanner, PlannerRegistry};
pub use planners::{
    BitStarPlanner, HybridPlanner, InformedRrtStarPlanner, JointInterpolationPlanner, PrmPlanner,
    RrtConnectPlanner, RrtStarPlanner, StompPlanner, BIT_STAR_PLANNER, HYBRID_PLANNER,
    INFORMED_RRT_STAR_PLANNER, JOINT_INTERPOLATION_PLANNER, PRM_PLANNER, RRT_CONNECT_PLANNER,
    RRT_STAR_PLANNER, STOMP_PLANNER,
};
pub use request::{MotionPlanRequest, MotionPlanResponse, PlanningOptions};
pub use sampler::ConstraintSampler;
pub use scene::PlanningScene;
pub use srdf::{parse_srdf, SrdfDocument, SrdfError, SrdfGroup};
pub use trajectory::{RobotTrajectory, TrajectoryPoint};
