//! Agent stepping helpers.

use super::components::AgentGoal;
use super::diff_drive::DiffDriveAgentState;
use crate::episode::EpisodeStep;
use crate::observation::DiffDriveObservation;
use rne_ecs::{Entity, World};

/// Resets one diff-drive agent and returns the initial step.
pub fn reset_diff_drive_agent(
    world: &mut World,
    agent: Entity,
) -> EpisodeStep<DiffDriveObservation> {
    let step = world
        .get_mut::<DiffDriveAgentState>(agent)
        .expect("diff-drive agent must have DiffDriveAgentState")
        .reset();
    let goal_x_m = world
        .get::<DiffDriveAgentState>(agent)
        .expect("diff-drive agent must have DiffDriveAgentState")
        .episode()
        .goal_x_m();
    world.entity_mut(agent).insert(AgentGoal { goal_x_m });
    step
}

/// Steps one diff-drive agent using its attached policy.
pub fn step_diff_drive_agent(
    world: &mut World,
    agent: Entity,
) -> EpisodeStep<DiffDriveObservation> {
    world
        .get_mut::<DiffDriveAgentState>(agent)
        .expect("diff-drive agent must have DiffDriveAgentState")
        .step()
}

/// Steps every diff-drive agent in the world.
pub fn step_diff_drive_agents(world: &mut World) {
    let mut query = world.query::<&mut DiffDriveAgentState>();
    for mut state in query.iter_mut(world) {
        if !state.last_step().is_done() {
            state.step();
        }
    }
}
