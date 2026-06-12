//! Application builder and plugin registration.

use crate::{Schedule, SimClock, SimDuration};

/// Plugin trait for extending the engine.
pub trait Plugin: Send + Sync + 'static {
    /// Human-readable plugin name.
    fn name(&self) -> &'static str;

    /// Registers systems with the application schedule.
    fn build(&self, schedule: &mut Schedule);
}

/// Builder for configuring an application before it is finalized.
#[derive(Default)]
pub struct AppBuilder {
    plugins: Vec<Box<dyn Plugin>>,
    schedule: Schedule,
    fixed_delta: SimDuration,
}

impl AppBuilder {
    /// Creates a new application builder with the default 60 Hz fixed step.
    pub fn new() -> Self {
        Self {
            fixed_delta: SimDuration::from_hertz(rne_math::Hertz::new(60.0)),
            ..Self::default()
        }
    }

    /// Sets the fixed simulation step size.
    pub fn with_fixed_delta(mut self, fixed_delta: SimDuration) -> Self {
        self.fixed_delta = fixed_delta;
        self
    }

    /// Registers a plugin.
    pub fn add_plugin<P: Plugin>(mut self, plugin: P) -> Self {
        plugin.build(&mut self.schedule);
        self.plugins.push(Box::new(plugin));
        self
    }

    /// Mutable access to the schedule during plugin registration.
    pub fn schedule_mut(&mut self) -> &mut Schedule {
        &mut self.schedule
    }

    /// Builds the runnable application.
    pub fn build(self) -> RneApp {
        RneApp {
            clock: SimClock::new(self.fixed_delta),
            schedule: self.schedule,
            plugins: self.plugins,
        }
    }
}

/// Main simulation application container.
pub struct RneApp {
    clock: SimClock,
    schedule: Schedule,
    #[allow(dead_code)]
    plugins: Vec<Box<dyn Plugin>>,
}

impl RneApp {
    /// Creates a new application with default settings.
    pub fn new() -> Self {
        AppBuilder::new().build()
    }

    /// Registers a plugin after construction.
    pub fn add_plugin<P: Plugin>(&mut self, plugin: P) -> &mut Self {
        plugin.build(&mut self.schedule);
        self.plugins.push(Box::new(plugin));
        self
    }

    /// Returns a shared reference to the simulation clock.
    pub fn clock(&self) -> &SimClock {
        &self.clock
    }

    /// Returns a mutable reference to the simulation clock.
    pub fn clock_mut(&mut self) -> &mut SimClock {
        &mut self.clock
    }

    /// Returns the schedule.
    pub fn schedule(&self) -> &Schedule {
        &self.schedule
    }

    /// Advances simulation time and runs one fixed step batch.
    pub fn step(&mut self, delta: SimDuration) {
        let steps = self.clock.advance(delta);
        for _ in 0..steps {
            self.schedule.run();
        }
    }
}

impl Default for RneApp {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schedule::{SchedulePhase, SystemId};
    use std::sync::atomic::{AtomicUsize, Ordering};

    static PLUGIN_BUILDS: AtomicUsize = AtomicUsize::new(0);

    struct TestPlugin;

    impl Plugin for TestPlugin {
        fn name(&self) -> &'static str {
            "test_plugin"
        }

        fn build(&self, schedule: &mut Schedule) {
            PLUGIN_BUILDS.fetch_add(1, Ordering::SeqCst);
            schedule.add_system(SchedulePhase::PreUpdate, SystemId::new("test"), || {});
        }
    }

    #[test]
    fn plugin_registration() {
        let before = PLUGIN_BUILDS.load(Ordering::SeqCst);
        let _app = AppBuilder::new().add_plugin(TestPlugin).build();
        assert_eq!(PLUGIN_BUILDS.load(Ordering::SeqCst), before + 1);
    }
}
