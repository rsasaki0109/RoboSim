//! Planning pipeline that selects a registered planner and runs adapters.

use crate::adapters::{
    AddTimeParameterization, FixStartStateBounds, FixStartStateCollision, FixWorkspaceBounds,
    PlanningRequestAdapter, SimplifyTrajectory,
};
use crate::error::PlanningError;
use crate::planner::PlannerRegistry;
use crate::planners::RRT_CONNECT_PLANNER;
use crate::request::{MotionPlanRequest, MotionPlanResponse};
use crate::scene::PlanningScene;

/// A configured planning pipeline.
///
/// This is the RNE analogue of `MoveIt`'s `PlanningPipeline`: it owns a planner
/// registry and an ordered adapter chain, exposes the registered planner names,
/// and dispatches a validated request to the selected planner. Adapters run
/// before planning and again after planning.
#[derive(Debug)]
pub struct PlanningPipeline {
    registry: PlannerRegistry,
    planner_name: String,
    adapters: Vec<Box<dyn PlanningRequestAdapter>>,
}

impl PlanningPipeline {
    /// Creates a pipeline that dispatches to the named planner with no adapters.
    pub fn new(
        registry: PlannerRegistry,
        planner_name: impl Into<String>,
    ) -> Result<Self, PlanningError> {
        let planner_name = planner_name.into();
        if registry.get(&planner_name).is_none() {
            return Err(PlanningError::UnknownPlanner(planner_name));
        }
        Ok(Self {
            registry,
            planner_name,
            adapters: Vec::new(),
        })
    }

    /// Creates a pipeline preloaded with the built-in planners and adapters.
    ///
    /// The default planner is [`RRT_CONNECT_PLANNER`] and the default adapters
    /// run [`FixStartStateBounds`], [`FixWorkspaceBounds`],
    /// [`FixStartStateCollision`], [`SimplifyTrajectory`], then
    /// [`AddTimeParameterization`].
    pub fn with_builtins() -> Self {
        Self {
            registry: PlannerRegistry::with_builtins(),
            planner_name: RRT_CONNECT_PLANNER.to_string(),
            adapters: vec![
                Box::new(FixStartStateBounds::new()),
                Box::new(FixWorkspaceBounds::new()),
                Box::new(FixStartStateCollision::new(20, 0.1)),
                Box::new(SimplifyTrajectory::new()),
                Box::new(AddTimeParameterization::new()),
            ],
        }
    }

    /// Registered planner names in deterministic order.
    pub fn planner_names(&self) -> Vec<&str> {
        self.registry.names()
    }

    /// Adapter names in application order.
    pub fn adapter_names(&self) -> Vec<&str> {
        self.adapters.iter().map(|adapter| adapter.name()).collect()
    }

    /// Selects a registered planner by name.
    pub fn set_planner(&mut self, name: impl Into<String>) -> Result<(), PlanningError> {
        let name = name.into();
        if self.registry.get(&name).is_none() {
            return Err(PlanningError::UnknownPlanner(name));
        }
        self.planner_name = name;
        Ok(())
    }

    /// Validates the request, runs the adapter chain, then runs the planner.
    pub fn plan(
        &self,
        scene: &PlanningScene,
        request: &MotionPlanRequest,
    ) -> Result<MotionPlanResponse, PlanningError> {
        let mut request = request.clone();
        for adapter in &self.adapters {
            request = adapter.adapt_request(scene, request)?;
        }
        request.validate()?;

        let planner = self
            .registry
            .get(&self.planner_name)
            .ok_or_else(|| PlanningError::UnknownPlanner(self.planner_name.clone()))?;
        let mut response = planner.plan(scene, &request)?;

        for adapter in &self.adapters {
            response = adapter.adapt_response(scene, &request, response)?;
        }
        Ok(response)
    }
}
