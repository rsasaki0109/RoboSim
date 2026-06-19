//! Episode lifecycle: reset, step, reward, and termination.

/// Why an episode ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminationReason {
    /// Goal or success condition reached.
    Success,
    /// Step budget exhausted.
    Truncated,
    /// Episode still running.
    None,
}

impl TerminationReason {
    /// Returns true when the episode has ended.
    pub fn is_done(self) -> bool {
        !matches!(self, Self::None)
    }
}

/// Result of a single environment step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EpisodeStep<O> {
    /// Environment observation after the step.
    pub observation: O,
    /// Scalar reward for the transition.
    pub reward: f64,
    /// True when a terminal success condition was met.
    pub terminated: bool,
    /// True when the step budget was exhausted.
    pub truncated: bool,
}

impl<O> EpisodeStep<O> {
    /// Returns true when the episode has ended for any reason.
    pub fn is_done(self) -> bool {
        self.terminated || self.truncated
    }

    /// Returns the termination reason for this step.
    pub fn termination(self) -> TerminationReason {
        if self.terminated {
            TerminationReason::Success
        } else if self.truncated {
            TerminationReason::Truncated
        } else {
            TerminationReason::None
        }
    }
}

/// Snapshot of an episode-owned deterministic RNG position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EpisodeRandomSnapshot {
    /// Internal state of the episode RNG.
    pub rng_state: u64,
}

impl EpisodeRandomSnapshot {
    /// Creates an RNG snapshot from an internal state value.
    pub const fn new(rng_state: u64) -> Self {
        Self { rng_state }
    }
}

/// Reset/step interface for reinforcement learning episodes.
pub trait Episode {
    /// Observation type returned after each step.
    type Observation;
    /// Action type accepted by the environment.
    type Action;

    /// Resets the environment and returns the initial step result.
    fn reset(&mut self) -> EpisodeStep<Self::Observation>;

    /// Applies an action and advances the simulation by one tick.
    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation>;

    /// Zero-based index of the current episode since construction.
    fn episode_index(&self) -> u32;

    /// Number of completed steps in the current episode.
    fn step_in_episode(&self) -> u64;
}
