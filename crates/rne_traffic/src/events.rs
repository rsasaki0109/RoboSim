//! Traffic runtime events.

use bevy_ecs::prelude::Event;
use rne_core::SimTime;

/// Reports completion of one deterministic traffic step.
#[derive(Clone, Copy, Debug, Event, PartialEq, Eq)]
pub struct TrafficStepCompleted {
    /// Monotonic traffic step index after the completed step.
    pub step_index: u64,
    /// Simulation timestamp associated with the completed step.
    pub sim_time: SimTime,
}
