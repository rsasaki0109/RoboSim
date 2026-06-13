//! Agent entities and policy attachment.

mod components;
mod diff_drive;
mod spawn;
mod systems;

pub use components::{Agent, AgentKind, AgentTarget, AttachedPolicy};
pub use diff_drive::{attach_diff_drive_policy, DiffDriveAgentState, DiffDrivePolicySource};
pub use spawn::spawn_diff_drive_agent;
pub use systems::{reset_diff_drive_agent, step_diff_drive_agent, step_diff_drive_agents};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ConstantVelocityPolicy, DiffDriveEpisodeConfig};
    use rne_ecs::World;

    #[test]
    fn diff_drive_agent_reaches_goal_with_attached_policy() {
        let mut world = World::new();
        let agent = spawn_diff_drive_agent(
            &mut world,
            "forward_agent",
            DiffDriveEpisodeConfig {
                goal_x_m: 1.5,
                max_steps: 300,
                ..DiffDriveEpisodeConfig::default()
            },
            AgentKind::Policy,
        );
        attach_diff_drive_policy(&mut world, agent, ConstantVelocityPolicy::new(6.0));

        let mut step = reset_diff_drive_agent(&mut world, agent);
        while !step.is_done() {
            step = step_diff_drive_agent(&mut world, agent);
        }

        assert!(step.terminated, "expected goal success");
        assert!(step.observation.base_x_m >= 1.5);
    }

    #[test]
    fn policy_can_be_swapped_after_spawn() {
        let mut world = World::new();
        let agent = spawn_diff_drive_agent(
            &mut world,
            "swap_agent",
            DiffDriveEpisodeConfig {
                goal_x_m: 1.5,
                max_steps: 300,
                ..DiffDriveEpisodeConfig::default()
            },
            AgentKind::Policy,
        );

        attach_diff_drive_policy(&mut world, agent, ConstantVelocityPolicy::new(1.0));
        let _ = reset_diff_drive_agent(&mut world, agent);
        for _ in 0..50 {
            let _ = step_diff_drive_agent(&mut world, agent);
        }
        let slow_x = world
            .get::<DiffDriveAgentState>(agent)
            .expect("agent state")
            .last_step()
            .observation
            .base_x_m;

        attach_diff_drive_policy(&mut world, agent, ConstantVelocityPolicy::new(8.0));
        let _ = reset_diff_drive_agent(&mut world, agent);
        for _ in 0..50 {
            let _ = step_diff_drive_agent(&mut world, agent);
        }
        let fast_x = world
            .get::<DiffDriveAgentState>(agent)
            .expect("agent state")
            .last_step()
            .observation
            .base_x_m;

        assert!(
            fast_x > slow_x,
            "faster policy should advance farther: slow={slow_x}, fast={fast_x}"
        );
    }
}
