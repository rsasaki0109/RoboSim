//! Solver resources and errors.

use thiserror::Error;

/// Fixed-step XPBD solver configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeformableSolverConfig {
    /// Number of equal substeps per simulation tick.
    pub substeps: u32,
    /// Sequential constraint iterations per substep.
    pub constraint_iterations: u32,
}

impl Default for DeformableSolverConfig {
    fn default() -> Self {
        Self {
            substeps: 4,
            constraint_iterations: 8,
        }
    }
}

/// Invalid input detected before a deformable simulation step.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum DeformableStepError {
    /// Simulation duration must be finite and positive.
    #[error("step duration must be finite and positive")]
    InvalidDuration,
    /// Solver counts must both be positive.
    #[error("substeps and constraint iterations must be positive")]
    InvalidSolverCounts,
    /// Particle state or material contains an invalid value.
    #[error("invalid deformable state: {0}")]
    InvalidState(String),
}
