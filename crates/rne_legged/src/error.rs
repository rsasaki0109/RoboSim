//! Errors returned by the legged template planners.

use thiserror::Error;

/// Error returned while building or evaluating a walking pattern.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum LeggedError {
    /// The LIPM height or gravity was not positive and finite.
    #[error("invalid LIPM parameters: height {com_height_m} m, gravity {gravity_m_s2} m/s²")]
    InvalidLimp {
        /// Center-of-mass height in meters.
        com_height_m: f64,
        /// Gravitational acceleration magnitude in meters per second squared.
        gravity_m_s2: f64,
    },
    /// A request value was out of range or non-finite.
    #[error("invalid walking request: {0}")]
    InvalidRequest(&'static str),
    /// A walking plan must contain at least one phase.
    #[error("walking plan must contain at least one footstep phase")]
    EmptyPlan,
    /// The preview controller needs at least one preview sample.
    #[error("preview controller requires at least one preview step")]
    EmptyPreview,
    /// A generated trajectory contained a non-finite value.
    #[error("walking trajectory contains a non-finite value")]
    NonFiniteTrajectory,
}
