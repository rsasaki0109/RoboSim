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

/// Optional goal position for goal-relative observations.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct AgentGoal {
    /// Target base X position in meters.
    pub goal_x_m: f64,
}

impl AgentGoal {
    /// Creates a forward goal at the given X coordinate.
    pub fn at_x(goal_x_m: f64) -> Self {
        Self { goal_x_m }
    }
}

/// Marker set after a policy has been attached to the agent.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct AttachedPolicy;
