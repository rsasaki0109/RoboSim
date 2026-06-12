//! Agents that live in the same ECS world as the simulation.

use super::components::{Agent, AgentKind, AgentTarget, AttachedPolicy};
use super::diff_drive::DiffDrivePolicySource;
use crate::action::DiffDriveAction;
use crate::env::DiffDriveEpisode;
use crate::env::DiffDriveSim;
use crate::observation::DiffDriveObservation;
use crate::policy::Policy;
use bevy_ecs::prelude::Component;
use rne_ecs::{spawn_named, Entity};

/// Policy-driven agent state stored on an entity in [`DiffDriveSim::world`].
#[derive(Component)]
pub struct SharedDiffDriveAgentState {
    policy: Option<Box<dyn DiffDrivePolicySource>>,
    last_observation: DiffDriveObservation,
}

impl SharedDiffDriveAgentState {
    /// Creates agent state with the given observation snapshot.
    pub fn new(last_observation: DiffDriveObservation) -> Self {
        Self {
            policy: None,
            last_observation,
        }
    }

    /// Attaches a policy used on the next step.
    pub fn attach_policy<P>(&mut self, policy: P)
    where
        P: Policy<DiffDriveEpisode> + Send + Sync + 'static,
    {
        self.policy = Some(Box::new(policy));
    }

    /// Returns the latest observation seen by this agent.
    pub fn last_observation(&self) -> DiffDriveObservation {
        self.last_observation
    }

    /// Returns true when a policy has been attached.
    pub fn has_policy(&self) -> bool {
        self.policy.is_some()
    }
}

/// Spawns an agent entity in the simulation ECS world.
pub fn spawn_shared_diff_drive_agent(
    sim: &mut DiffDriveSim,
    name: impl Into<String>,
    kind: AgentKind,
) -> Entity {
    let robot = sim.robot().robot;
    let observation = sim.observe_robot(robot);
    let entity = spawn_named(sim.world_mut(), name);
    sim.world_mut().entity_mut(entity).insert((
        Agent { kind },
        AgentTarget { robot: Some(robot) },
        SharedDiffDriveAgentState::new(observation),
    ));
    entity
}

/// Attaches a policy to a shared-world agent entity.
pub fn attach_shared_diff_drive_policy<P>(sim: &mut DiffDriveSim, agent: Entity, policy: P)
where
    P: Policy<DiffDriveEpisode> + Send + Sync + 'static,
{
    let robot = sim
        .world()
        .get::<AgentTarget>(agent)
        .and_then(|target| target.robot)
        .unwrap_or_else(|| sim.robot().robot);
    let observation = sim.observe_robot(robot);

    let mut entity = sim
        .world_mut()
        .get_entity_mut(agent)
        .expect("shared-world agent entity must exist");
    let mut state = entity
        .get_mut::<SharedDiffDriveAgentState>()
        .expect("entity must have SharedDiffDriveAgentState");
    state.attach_policy(policy);
    state.last_observation = observation;
    entity.insert(AttachedPolicy);
}

/// Refreshes an agent observation from the live simulation world.
pub fn observe_shared_diff_drive_agent(
    sim: &mut DiffDriveSim,
    agent: Entity,
) -> DiffDriveObservation {
    let robot = sim
        .world()
        .get::<AgentTarget>(agent)
        .and_then(|target| target.robot)
        .unwrap_or_else(|| sim.robot().robot);
    let observation = sim.observe_robot(robot);
    sim.world_mut()
        .get_mut::<SharedDiffDriveAgentState>(agent)
        .expect("shared-world agent must have SharedDiffDriveAgentState")
        .last_observation = observation;
    observation
}

/// Applies the agent policy and advances the shared simulation by one tick.
pub fn step_shared_diff_drive_agent(sim: &mut DiffDriveSim, agent: Entity) -> DiffDriveObservation {
    let action = {
        let world = sim.world_mut();
        let mut state = world
            .get_mut::<SharedDiffDriveAgentState>(agent)
            .expect("shared-world agent must have SharedDiffDriveAgentState");
        let observation = state.last_observation;
        let policy = state
            .policy
            .as_mut()
            .expect("shared-world agent stepped without an attached policy");
        policy.act(&observation)
    };

    step_shared_diff_drive_action(sim, agent, action)
}

/// Applies an explicit action for the agent's robot and advances the simulation.
pub fn step_shared_diff_drive_action(
    sim: &mut DiffDriveSim,
    agent: Entity,
    action: DiffDriveAction,
) -> DiffDriveObservation {
    let observation = sim.step_action(action);
    sim.world_mut()
        .get_mut::<SharedDiffDriveAgentState>(agent)
        .expect("shared-world agent must have SharedDiffDriveAgentState")
        .last_observation = observation;
    observation
}

/// Steps every shared-world agent in the simulation (last action wins per robot).
pub fn step_shared_diff_drive_agents(sim: &mut DiffDriveSim) {
    let agents: Vec<Entity> = {
        let world = sim.world_mut();
        let mut query = world.query::<(Entity, &SharedDiffDriveAgentState)>();
        query
            .iter(world)
            .filter(|(_, state)| state.has_policy())
            .map(|(entity, _)| entity)
            .collect()
    };

    for agent in agents {
        let _ = step_shared_diff_drive_agent(sim, agent);
    }
}
