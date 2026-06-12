//! Schedule phases and system ordering.

use std::fmt;

/// Ordered phases in the simulation pipeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SchedulePhase {
    /// Startup initialization.
    Startup,
    /// Asset loading.
    AssetLoad,
    /// Pre-update hooks.
    PreUpdate,
    /// AI observation.
    AIObserve,
    /// AI action selection.
    AIAct,
    /// Control input application.
    Control,
    /// Pre-physics synchronization.
    PrePhysics,
    /// Fixed physics step.
    PhysicsFixedStep,
    /// Post-physics synchronization.
    PostPhysics,
    /// Sensor sampling.
    SensorSample,
    /// Data recording.
    DataRecord,
    /// Render extraction.
    RenderExtract,
    /// Render submission.
    RenderSubmit,
    /// Post-update hooks.
    PostUpdate,
    /// Cleanup.
    Cleanup,
}

impl SchedulePhase {
    /// Returns all phases in execution order.
    pub const fn all() -> &'static [Self] {
        &[
            Self::Startup,
            Self::AssetLoad,
            Self::PreUpdate,
            Self::AIObserve,
            Self::AIAct,
            Self::Control,
            Self::PrePhysics,
            Self::PhysicsFixedStep,
            Self::PostPhysics,
            Self::SensorSample,
            Self::DataRecord,
            Self::RenderExtract,
            Self::RenderSubmit,
            Self::PostUpdate,
            Self::Cleanup,
        ]
    }
}

/// Identifier for a registered system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SystemId {
    name: &'static str,
}

impl SystemId {
    /// Creates a system identifier from a static name.
    pub const fn new(name: &'static str) -> Self {
        Self { name }
    }

    /// Returns the system name.
    pub const fn name(self) -> &'static str {
        self.name
    }
}

impl fmt::Display for SystemId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name)
    }
}

/// Function pointer system for early bootstrap.
pub type SystemFn = fn();

#[derive(Clone, Copy, Debug)]
struct ScheduledSystem {
    id: SystemId,
    system: SystemFn,
}

/// Ordered collection of systems grouped by phase.
#[derive(Default)]
pub struct Schedule {
    systems: Vec<(SchedulePhase, ScheduledSystem)>,
}

impl Schedule {
    /// Creates an empty schedule.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers a system in the given phase.
    pub fn add_system(&mut self, phase: SchedulePhase, id: SystemId, system: SystemFn) {
        self.systems.push((phase, ScheduledSystem { id, system }));
    }

    /// Returns registered systems in phase order.
    pub fn systems(&self) -> impl Iterator<Item = (SchedulePhase, SystemId)> + '_ {
        let mut ordered: Vec<_> = self
            .systems
            .iter()
            .map(|(phase, system)| (*phase, system.id))
            .collect();
        ordered.sort_by_key(|(phase, _)| *phase);
        ordered.into_iter()
    }

    /// Runs all systems in phase order.
    pub fn run(&self) {
        let mut ordered = self.systems.clone();
        ordered.sort_by_key(|(phase, _)| *phase);

        for (_, system) in ordered {
            (system.system)();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    thread_local! {
        static ORDER: RefCell<Vec<&'static str>> = const { RefCell::new(Vec::new()) };
    }

    fn record(name: &'static str) {
        ORDER.with(|order| order.borrow_mut().push(name));
    }

    fn system_a() {
        record("a");
    }

    fn system_b() {
        record("b");
    }

    #[test]
    fn phase_ordering() {
        ORDER.with(|order| order.borrow_mut().clear());

        let mut schedule = Schedule::new();
        schedule.add_system(SchedulePhase::PostUpdate, SystemId::new("b"), system_b);
        schedule.add_system(SchedulePhase::PreUpdate, SystemId::new("a"), system_a);
        schedule.run();

        ORDER.with(|order| {
            assert_eq!(order.borrow().as_slice(), &["a", "b"]);
        });
    }
}
