//! Diagnostic progress at fallible execution boundaries.
//!
//! These values are observations, not a solver checkpoint or replay proof. A
//! backend may mutate before returning an error; the active stage does not
//! establish an exact failure timestamp or a post-failure state digest.

use serde::{Deserialize, Serialize};

/// Operation entered most recently, before its potentially fallible work.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionStage {
    /// Selecting actions from available actor observations, before batch execution.
    ActionSelection,
    /// Batch preflight, lane execution and learning projection as one call boundary.
    /// Per-lane diagnostics provide finer stages; this does not prove physics began.
    BatchExecution,
    /// Applying the pending force/torque command.
    ApplyWrench,
    /// Advancing the physics backend, which may partially mutate on failure.
    Physics,
    /// Copying completed backend state into the ECS.
    Synchronize,
    /// Reading solved contacts and evaluating the drive force path.
    Drive,
    /// Sampling and queuing sensor outputs.
    Sensors,
    /// Computing an estimate from available sensor frames.
    Estimation,
    /// Computing evaluator-only reward or termination information.
    Evaluation,
    /// Validating/projecting a completed interval into a learning transition.
    LearningProjection,
    /// Updating a learner after physical execution.
    Learning,
    /// Capturing or encoding evidence after execution.
    Evidence,
}

/// Diagnostic counters with explicitly different completion boundaries.
///
/// This is not a versioned artifact envelope. Its owner must bind lane/episode,
/// requested action, backend, build and pre-attempt evidence when packaging it.
/// No physical state hash is implied, especially when `active_stage` is present.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecutionProgress {
    /// Successfully returned outer control intervals in the current episode.
    pub completed_intervals: u64,
    /// Local simulation time at the last successful outer return (zero at reset).
    /// This is not the time when an error occurred.
    pub last_successful_time_ticks: u64,
    /// Completed physics/ECS/drive updates, possibly ahead of sensor processing.
    /// A failed backend call may have progressed further internally.
    pub completed_drive_ticks: u64,
    /// Entered stage not yet followed by a successful outer return.
    pub active_stage: Option<ExecutionStage>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_attempt_can_fail_without_a_completed_interval() {
        let progress = ExecutionProgress {
            active_stage: Some(ExecutionStage::Physics),
            ..Default::default()
        };
        let bytes = serde_json::to_vec(&progress).unwrap();
        assert_eq!(
            serde_json::from_slice::<ExecutionProgress>(&bytes).unwrap(),
            progress
        );
        assert!(serde_json::from_str::<ExecutionProgress>(
            r#"{"completed_intervals":0,"last_successful_time_ticks":0,"completed_drive_ticks":0,"active_stage":"physics","state_digest":0}"#
        ).is_err());
    }
}
