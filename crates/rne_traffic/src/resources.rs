//! Traffic runtime resources.

use bevy_ecs::prelude::Resource;

/// Per-world deterministic traffic runtime state.
#[derive(Clone, Debug, Default, PartialEq, Eq, Resource)]
pub struct TrafficRuntime {
    step_index: u64,
}

impl TrafficRuntime {
    /// Returns the number of completed traffic steps.
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    pub(crate) fn advance(&mut self) -> u64 {
        self.step_index = self.step_index.saturating_add(1);
        self.step_index
    }
}
