//! Agent entity components.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;

/// How an agent produces actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AgentKind {
    /// Closed-loop policy (Rust, Python, or external process).
    Policy,
    /// Human teleoperation input.
    Teleop,
    /// External controller connected over IPC or network.
    External,
}

/// Marks an entity as an agent controller.
#[derive(Component, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Agent {
    /// Controller category.
    pub kind: AgentKind,
}

/// Optional robot target when agent and robot share the same ECS world.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct AgentTarget {
    /// Controlled robot entity, when known.
    pub robot: Option<Entity>,
}

/// Marker set after a policy has been attached to the agent.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct AttachedPolicy;
