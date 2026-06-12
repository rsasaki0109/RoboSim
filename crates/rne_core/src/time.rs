//! Simulation time types and clock.

use rne_math::{Hertz, Seconds};
use std::fmt;

/// Simulation timestamp in seconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash)]
pub struct SimTime {
    ticks: u64,
}

impl SimTime {
    /// Zero simulation time.
    pub const ZERO: Self = Self { ticks: 0 };

    /// Creates simulation time from seconds.
    pub fn from_seconds(seconds: Seconds) -> Self {
        Self {
            ticks: (seconds.value() * 1_000_000_000.0).round() as u64,
        }
    }

    /// Returns the time as seconds.
    pub fn as_seconds(self) -> Seconds {
        Seconds::new(self.ticks as f64 / 1_000_000_000.0)
    }

    /// Returns raw nanosecond ticks.
    pub const fn ticks(self) -> u64 {
        self.ticks
    }

    /// Creates simulation time from raw nanosecond ticks.
    pub const fn from_ticks(ticks: u64) -> Self {
        Self { ticks }
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_seconds())
    }
}

impl std::ops::Add<SimDuration> for SimTime {
    type Output = Self;

    fn add(self, rhs: SimDuration) -> Self::Output {
        Self {
            ticks: self.ticks.saturating_add(rhs.ticks),
        }
    }
}

/// Simulation duration in seconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Eq, Ord, Hash)]
pub struct SimDuration {
    ticks: u64,
}

impl SimDuration {
    /// Zero duration.
    pub const ZERO: Self = Self { ticks: 0 };

    /// Creates a duration from seconds.
    pub fn from_seconds(seconds: Seconds) -> Self {
        Self {
            ticks: (seconds.value() * 1_000_000_000.0).round() as u64,
        }
    }

    /// Creates a fixed-step duration from an update rate in hertz.
    pub fn from_hertz(hz: Hertz) -> Self {
        let hz_value = hz.value().round().max(1.0) as u64;
        Self {
            ticks: 1_000_000_000 / hz_value,
        }
    }

    /// Creates a duration from raw nanosecond ticks.
    pub const fn from_ticks(ticks: u64) -> Self {
        Self { ticks }
    }

    /// Returns the duration as seconds.
    pub fn as_seconds(self) -> Seconds {
        Seconds::new(self.ticks as f64 / 1_000_000_000.0)
    }

    /// Returns raw nanosecond ticks.
    pub const fn ticks(self) -> u64 {
        self.ticks
    }
}

impl fmt::Display for SimDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_seconds())
    }
}

/// Fixed-step simulation clock driven only by explicit deltas.
#[derive(Clone, Debug)]
pub struct SimClock {
    sim_time: SimTime,
    fixed_delta: SimDuration,
    time_scale: f64,
    paused: bool,
    accumulator_ticks: u64,
}

impl SimClock {
    /// Creates a clock with the given fixed step size.
    pub fn new(fixed_delta: SimDuration) -> Self {
        Self {
            sim_time: SimTime::ZERO,
            fixed_delta,
            time_scale: 1.0,
            paused: false,
            accumulator_ticks: 0,
        }
    }

    /// Current simulation time.
    pub fn sim_time(&self) -> SimTime {
        self.sim_time
    }

    /// Fixed simulation step size.
    pub fn fixed_delta(&self) -> SimDuration {
        self.fixed_delta
    }

    /// Current time scale multiplier.
    pub fn time_scale(&self) -> f64 {
        self.time_scale
    }

    /// Whether the clock is paused.
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Sets the time scale multiplier.
    pub fn set_time_scale(&mut self, scale: f64) {
        self.time_scale = scale.max(0.0);
    }

    /// Pauses simulation time advancement.
    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resumes simulation time advancement.
    pub fn resume(&mut self) {
        self.paused = false;
    }

    /// Resets simulation time and accumulator.
    pub fn reset(&mut self) {
        self.sim_time = SimTime::ZERO;
        self.accumulator_ticks = 0;
    }

    /// Advances the clock using an explicit wall-independent delta.
    ///
    /// Returns the number of fixed simulation steps to execute.
    pub fn advance(&mut self, delta: SimDuration) -> u32 {
        if self.paused || self.time_scale == 0.0 {
            return 0;
        }

        let scaled_delta_ticks = (delta.ticks() as f64 * self.time_scale).round() as u64;
        self.accumulator_ticks = self.accumulator_ticks.saturating_add(scaled_delta_ticks);

        let fixed_ticks = self.fixed_delta.ticks();
        if fixed_ticks == 0 {
            return 0;
        }

        let mut steps = 0_u32;
        while self.accumulator_ticks >= fixed_ticks {
            self.accumulator_ticks -= fixed_ticks;
            self.sim_time = self.sim_time + self.fixed_delta;
            steps += 1;
        }

        steps
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn step_60_hz() -> SimDuration {
        SimDuration::from_hertz(Hertz::new(60.0))
    }

    #[test]
    fn fixed_step_60_hz() {
        let fixed = step_60_hz();
        let mut clock = SimClock::new(fixed);
        let steps = clock.advance(SimDuration {
            ticks: fixed.ticks() * 60,
        });

        assert_eq!(steps, 60);
        assert_relative_eq!(
            clock.sim_time().as_seconds().value(),
            fixed.as_seconds().value() * 60.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn pause_and_resume() {
        let fixed = step_60_hz();
        let mut clock = SimClock::new(fixed);
        clock.pause();
        assert_eq!(
            clock.advance(SimDuration {
                ticks: fixed.ticks() * 60
            }),
            0
        );
        clock.resume();
        assert_eq!(
            clock.advance(SimDuration {
                ticks: fixed.ticks() * 60
            }),
            60
        );
    }

    #[test]
    fn time_scale() {
        let fixed = step_60_hz();
        let mut clock = SimClock::new(fixed);
        clock.set_time_scale(0.5);
        let steps = clock.advance(SimDuration {
            ticks: fixed.ticks() * 60,
        });

        assert_eq!(steps, 30);
        assert_relative_eq!(
            clock.sim_time().as_seconds().value(),
            fixed.as_seconds().value() * 30.0,
            epsilon = 1e-9
        );
    }
}
