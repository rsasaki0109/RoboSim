//! Deterministic simulation-tick scheduling for the flagship LeKiwi boundary.
//!
//! The scheduler owns no wall clock. It accepts one exact, zero-based flagship
//! sequence at a time and emits only phase-zero even sequences to the 30 Hz
//! physical boundary. Every source action is projected and validated before a
//! decision is recorded, including actions deliberately suppressed by rate
//! conversion.

use crate::flagship_projection::{
    project_flagship_action_to_lekiwi, FlagshipLeKiwiActionProjection,
    FlagshipLeKiwiProjectionError,
};
use rne_ai::FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS;
use serde::{Deserialize, Serialize};

/// Flagship controller period in integer nanosecond simulation ticks.
pub const FLAGSHIP_CONTROLLER_PERIOD_TICKS: u64 = FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS;

/// LeKiwi write period derived from exactly two flagship controller ticks.
pub const FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS: u64 = 33_333_334;

/// Number of flagship controller decisions per LeKiwi write slot.
pub const FLAGSHIP_LEKIWI_DECIMATION: u64 = 2;

const _: () = assert!(FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS >= 1_000_000_000 / 30);

/// Schema version for one flagship-to-LeKiwi rate decision.
pub const FLAGSHIP_LEKIWI_RATE_DECISION_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`FlagshipLeKiwiRateDecision`].
pub const FLAGSHIP_LEKIWI_RATE_DECISION_KIND: &str = "rne_flagship_lekiwi_rate_decision";

/// Whether one validated parent action receives physical write authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlagshipLeKiwiRateDisposition {
    /// Phase-zero action selected for the physical write slot.
    Emit,
    /// Validated intermediate controller action deliberately denied authority.
    SuppressBetweenPhysicalTicks,
}

/// One deterministic, evidence-bearing 60-to-30 Hz scheduling decision.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiRateDecision {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Rate-decision schema version.
    pub schema_version: u32,
    /// Exact zero-based flagship action sequence.
    pub parent_sequence: u64,
    /// Exact parent controller period in integer nanosecond ticks.
    pub parent_period_ticks: u64,
    /// Exact physical write period in integer nanosecond ticks.
    pub physical_period_ticks: u64,
    /// Zero-based physical slot containing this parent sequence.
    pub physical_slot: u64,
    /// Physical write sequence when emitted; absent when suppressed.
    pub physical_sequence: Option<u64>,
    /// Explicit authority decision.
    pub disposition: FlagshipLeKiwiRateDisposition,
    /// Fully validated action projection, retained even when suppressed.
    pub projection: FlagshipLeKiwiActionProjection,
}

/// Stateful exact-sequence scheduler for the flagship LeKiwi rate boundary.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FlagshipLeKiwiRateScheduler {
    expected_parent_sequence: u64,
}

impl FlagshipLeKiwiRateScheduler {
    /// Creates a scheduler synchronized to parent sequence zero and phase zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the only parent sequence currently accepted by [`Self::ingest`].
    pub fn expected_parent_sequence(&self) -> u64 {
        self.expected_parent_sequence
    }

    /// Validates and schedules one complete flagship controller action.
    ///
    /// Duplicate, missing, out-of-order, invalid, or overflowing source actions
    /// fail without advancing scheduler state.
    pub fn ingest(
        &mut self,
        parent_sequence: u64,
        parent_action: &[f64],
    ) -> Result<FlagshipLeKiwiRateDecision, FlagshipLeKiwiRateError> {
        if parent_sequence != self.expected_parent_sequence {
            return Err(FlagshipLeKiwiRateError::UnexpectedParentSequence {
                expected: self.expected_parent_sequence,
                actual: parent_sequence,
            });
        }
        let next_parent_sequence = parent_sequence
            .checked_add(1)
            .ok_or(FlagshipLeKiwiRateError::SequenceOverflow)?;
        let projection = project_flagship_action_to_lekiwi(parent_action)?;
        let physical_slot = parent_sequence / FLAGSHIP_LEKIWI_DECIMATION;
        let emitted = parent_sequence.is_multiple_of(FLAGSHIP_LEKIWI_DECIMATION);

        let decision = FlagshipLeKiwiRateDecision {
            kind: FLAGSHIP_LEKIWI_RATE_DECISION_KIND.to_string(),
            schema_version: FLAGSHIP_LEKIWI_RATE_DECISION_SCHEMA_VERSION,
            parent_sequence,
            parent_period_ticks: FLAGSHIP_CONTROLLER_PERIOD_TICKS,
            physical_period_ticks: FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS,
            physical_slot,
            physical_sequence: emitted.then_some(physical_slot),
            disposition: if emitted {
                FlagshipLeKiwiRateDisposition::Emit
            } else {
                FlagshipLeKiwiRateDisposition::SuppressBetweenPhysicalTicks
            },
            projection,
        };
        self.expected_parent_sequence = next_parent_sequence;
        Ok(decision)
    }
}

/// Failure at the deterministic flagship-to-LeKiwi rate boundary.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FlagshipLeKiwiRateError {
    /// The input was duplicated, missing, or out of order.
    #[error("flagship parent sequence must be {expected}, got {actual}")]
    UnexpectedParentSequence {
        /// Required next sequence.
        expected: u64,
        /// Supplied sequence.
        actual: u64,
    },
    /// The zero-based source sequence cannot be advanced safely.
    #[error("flagship parent sequence overflow")]
    SequenceOverflow,
    /// The source action or its physical projection failed closed.
    #[error(transparent)]
    Projection(#[from] FlagshipLeKiwiProjectionError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn action(speed_rad_s: f64) -> Vec<f64> {
        vec![speed_rad_s, speed_rad_s, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

    #[test]
    fn phase_zero_emits_even_sequences_and_records_suppression() {
        let mut scheduler = FlagshipLeKiwiRateScheduler::new();
        let zero = scheduler.ingest(0, &action(0.5)).unwrap();
        let one = scheduler.ingest(1, &action(0.6)).unwrap();
        let two = scheduler.ingest(2, &action(0.7)).unwrap();

        assert_eq!(zero.disposition, FlagshipLeKiwiRateDisposition::Emit);
        assert_eq!(zero.physical_sequence, Some(0));
        assert_eq!(one.physical_slot, 0);
        assert_eq!(
            one.disposition,
            FlagshipLeKiwiRateDisposition::SuppressBetweenPhysicalTicks
        );
        assert_eq!(one.physical_sequence, None);
        assert_eq!(two.physical_sequence, Some(1));
        assert_eq!(scheduler.expected_parent_sequence(), 3);
    }

    #[test]
    fn duplicate_gap_and_invalid_projection_do_not_advance_state() {
        let mut scheduler = FlagshipLeKiwiRateScheduler::new();
        scheduler.ingest(0, &action(0.0)).unwrap();
        assert!(matches!(
            scheduler.ingest(0, &action(0.0)),
            Err(FlagshipLeKiwiRateError::UnexpectedParentSequence { .. })
        ));
        assert!(matches!(
            scheduler.ingest(2, &action(0.0)),
            Err(FlagshipLeKiwiRateError::UnexpectedParentSequence { .. })
        ));
        assert_eq!(scheduler.expected_parent_sequence(), 1);

        assert!(matches!(
            scheduler.ingest(1, &action(2.0)),
            Err(FlagshipLeKiwiRateError::Projection(_))
        ));
        assert_eq!(scheduler.expected_parent_sequence(), 1);
        scheduler.ingest(1, &action(0.0)).unwrap();
        assert_eq!(scheduler.expected_parent_sequence(), 2);
    }

    #[test]
    fn periods_are_integer_exact() {
        assert_eq!(
            FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS,
            FLAGSHIP_CONTROLLER_PERIOD_TICKS * FLAGSHIP_LEKIWI_DECIMATION
        );
    }

    #[test]
    fn sequence_overflow_fails_without_advancing_state() {
        let mut scheduler = FlagshipLeKiwiRateScheduler {
            expected_parent_sequence: u64::MAX,
        };
        assert_eq!(
            scheduler.ingest(u64::MAX, &action(0.0)),
            Err(FlagshipLeKiwiRateError::SequenceOverflow)
        );
        assert_eq!(scheduler.expected_parent_sequence(), u64::MAX);
    }
}
