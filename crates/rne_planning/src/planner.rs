//! Planner boundary and registry.

use crate::error::PlanningError;
use crate::planners::{
    BitStarPlanner, HybridPlanner, InformedRrtStarPlanner, JointInterpolationPlanner, PrmPlanner,
    RrtConnectPlanner, RrtStarPlanner, StompPlanner,
};
use crate::request::{MotionPlanRequest, MotionPlanResponse};
use crate::scene::PlanningScene;

/// Boundary implemented by swappable motion planners.
///
/// This is the RNE analogue of MoveIt's `PlannerInterface`: a planner receives a
/// scene and a request and returns a validated trajectory or a structured error.
/// Implementations must be deterministic for a given scene and request.
pub trait MotionPlanner: Send + Sync + std::fmt::Debug {
    /// Planner name used for registry lookup.
    fn name(&self) -> &str;

    /// Plans a collision-free motion.
    fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError>;
}

/// Ordered registry of motion planners addressable by name.
///
/// Registration preserves insertion order so [`Self::names`] is deterministic.
#[derive(Debug, Default)]
pub struct PlannerRegistry {
    planners: Vec<Box<dyn MotionPlanner>>,
}

impl PlannerRegistry {
    /// Creates an empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a registry preloaded with the built-in planners.
    pub fn with_builtins() -> Self {
        let mut registry = Self::new();
        registry
            .register(Box::new(JointInterpolationPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(RrtConnectPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(RrtStarPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(PrmPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(HybridPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(StompPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(InformedRrtStarPlanner::new()))
            .expect("built-in planner names are unique");
        registry
            .register(Box::new(BitStarPlanner::new()))
            .expect("built-in planner names are unique");
        registry
    }

    /// Registers a planner, rejecting empty or duplicate names.
    pub fn register(&mut self, planner: Box<dyn MotionPlanner>) -> Result<(), PlanningError> {
        if planner.name().trim().is_empty() {
            return Err(PlanningError::InvalidPlannerName);
        }
        if self
            .planners
            .iter()
            .any(|existing| existing.name() == planner.name())
        {
            return Err(PlanningError::DuplicatePlanner(planner.name().to_string()));
        }
        self.planners.push(planner);
        Ok(())
    }

    /// Looks up a planner by name.
    pub fn get(&self, name: &str) -> Option<&dyn MotionPlanner> {
        self.planners
            .iter()
            .find(|planner| planner.name() == name)
            .map(|planner| planner.as_ref())
    }

    /// Registered planner names in insertion order.
    pub fn names(&self) -> Vec<&str> {
        self.planners.iter().map(|planner| planner.name()).collect()
    }

    /// Number of registered planners.
    pub fn len(&self) -> usize {
        self.planners.len()
    }

    /// Whether no planner is registered.
    pub fn is_empty(&self) -> bool {
        self.planners.is_empty()
    }
}
