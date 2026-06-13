//! Agent spawn helpers.

use super::components::{Agent, AgentKind, AgentTarget};
use super::diff_drive::DiffDriveAgentState;
use crate::DiffDriveEpisodeConfig;
use rne_ecs::{spawn_named, Entity, World};

/// Spawns a diff-drive agent entity with episode state but no policy yet.
pub fn spawn_diff_drive_agent(
    world: &mut World,
    name: impl Into<String>,
    config: DiffDriveEpisodeConfig,
    kind: AgentKind,
) -> Entity {
    let entity = spawn_named(world, name);
    world.entity_mut(entity).insert((
        Agent { kind },
        AgentTarget::default(),
        DiffDriveAgentState::new(config),
    ));
    entity
}
