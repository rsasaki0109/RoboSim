//! Errors returned across the motion planning pipeline.

use rne_robot::KinematicsError;
use thiserror::Error;

/// Error returned while validating or executing a motion plan.
#[derive(Debug, Error)]
pub enum PlanningError {
    /// The request carried an empty start state.
    #[error("motion plan request has an empty start state")]
    EmptyRequest,
    /// A joint vector did not match the model degrees of freedom.
    #[error("expected {expected} joint values but received {provided}")]
    JointCountMismatch {
        /// Number of values supplied.
        provided: usize,
        /// Number of movable joints in the model.
        expected: usize,
    },
    /// A planning option or trajectory value was invalid.
    #[error("invalid planning value: {0}")]
    Invalid(String),
    /// The start configuration is in collision.
    #[error("start configuration is in collision")]
    StartInCollision,
    /// The goal configuration is in collision.
    #[error("goal configuration is in collision")]
    GoalInCollision,
    /// The goal pose could not be reached by inverse kinematics.
    #[error("goal constraint is unreachable")]
    GoalUnreachable,
    /// No collision-free path connected the start and goal.
    #[error("no collision-free path found")]
    NoPath,
    /// The planner exhausted its iteration budget.
    #[error("planner exceeded its iteration limit")]
    IterationLimit,
    /// A requested kinematics solver is not registered.
    #[error("kinematics solver {0:?} is not registered")]
    UnknownSolver(String),
    /// A requested planner is not registered.
    #[error("motion planner {0:?} is not registered")]
    UnknownPlanner(String),
    /// A requested planning group is not defined in the scene.
    #[error("planning group {0:?} is not defined")]
    UnknownGroup(String),
    /// A planner name was empty.
    #[error("motion planner name must not be empty")]
    InvalidPlannerName,
    /// A planner with the same name was already registered.
    #[error("motion planner {0:?} is already registered")]
    DuplicatePlanner(String),
    /// The kinematic model could not be evaluated.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
}
