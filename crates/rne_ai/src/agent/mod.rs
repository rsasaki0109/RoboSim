//! Agent entities and policy attachment.

mod components;
mod diff_drive;
mod shared;
mod spawn;
mod systems;

pub use components::{Agent, AgentGoal, AgentKind, AgentTarget, AttachedPolicy};
pub use diff_drive::{
    attach_diff_drive_policy, attach_goal_conditioned_policy, DiffDriveAgentState,
    DiffDrivePolicySource,
};
pub use shared::{
    attach_shared_diff_drive_policy, attach_shared_goal_conditioned_policy,
    observe_shared_diff_drive_agent, spawn_shared_diff_drive_agent,
    spawn_shared_diff_drive_agent_for_robot, step_shared_diff_drive_action,
    step_shared_diff_drive_agent, step_shared_diff_drive_agents, SharedDiffDriveAgentState,
};
pub use spawn::spawn_diff_drive_agent;
pub use systems::{reset_diff_drive_agent, step_diff_drive_agent, step_diff_drive_agents};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ConstantVelocityPolicy, DiffDriveEpisodeConfig, DiffDriveSim};
    use rne_ecs::World;

    #[test]
    fn diff_drive_agent_reaches_goal_with_goal_seeking_policy() {
        let mut world = World::new();
        let agent = spawn_diff_drive_agent(
            &mut world,
            "goal_agent",
            DiffDriveEpisodeConfig {
                goal_x_m: 1.5,
                max_steps: 300,
                ..DiffDriveEpisodeConfig::default()
            },
            AgentKind::Policy,
        );
        attach_goal_conditioned_policy(&mut world, agent, crate::GoalSeekingPolicy::new(6.0, 0.05));

        let mut step = reset_diff_drive_agent(&mut world, agent);
        assert_eq!(step.observation.goal_delta_x_m, Some(1.5));

        while !step.is_done() {
            step = step_diff_drive_agent(&mut world, agent);
        }

        assert!(step.terminated, "expected goal success");
        assert!(step.observation.base_x_m >= 1.5);
    }

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

    #[test]
    fn shared_world_agent_drives_simulation_forward() {
        let mut sim = crate::DiffDriveSim::new();
        let agent = spawn_shared_diff_drive_agent(&mut sim, "shared_agent", AgentKind::Policy);
        attach_shared_diff_drive_policy(&mut sim, agent, ConstantVelocityPolicy::new(6.0));

        let mut final_x = 0.0;
        for _ in 0..300 {
            let obs = step_shared_diff_drive_agent(&mut sim, agent);
            final_x = obs.base_x_m;
        }

        assert!(final_x > 0.8, "expected forward motion, got x={final_x}");
        assert!(sim.world().get::<Agent>(agent).is_some());
        assert!(sim
            .world()
            .get::<SharedDiffDriveAgentState>(agent)
            .is_some());
    }

    #[test]
    fn multi_robot_shared_agents_advance_in_one_tick() {
        use rne_math::Vec3;
        use rne_robot::{DiffDriveConfig, DiffDriveDriveMode};

        let mut sim = DiffDriveSim::with_robot_configs(&[
            DiffDriveConfig {
                model_name: "robot_a".into(),
                initial_translation_m: Vec3::new(0.0, 0.25, -1.0),
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
            DiffDriveConfig {
                model_name: "robot_b".into(),
                initial_translation_m: Vec3::new(0.0, 0.25, 1.0),
                drive_mode: DiffDriveDriveMode::JointDriven,
                ..DiffDriveConfig::default()
            },
        ]);
        let robot_a = sim.robots()[0].robot;
        let robot_b = sim.robots()[1].robot;

        let agent_a = spawn_shared_diff_drive_agent_for_robot(
            &mut sim,
            "agent_a",
            AgentKind::Policy,
            robot_a,
            Some(1.5),
        );
        let agent_b = spawn_shared_diff_drive_agent_for_robot(
            &mut sim,
            "agent_b",
            AgentKind::Policy,
            robot_b,
            Some(1.0),
        );
        attach_shared_diff_drive_policy(&mut sim, agent_a, ConstantVelocityPolicy::new(6.0));
        attach_shared_diff_drive_policy(&mut sim, agent_b, ConstantVelocityPolicy::new(4.0));

        let mut final_a = 0.0;
        let mut final_b = 0.0;
        for _ in 0..300 {
            step_shared_diff_drive_agents(&mut sim);
            final_a = sim
                .world()
                .get::<SharedDiffDriveAgentState>(agent_a)
                .expect("agent_a state")
                .last_observation()
                .base_x_m;
            final_b = sim
                .world()
                .get::<SharedDiffDriveAgentState>(agent_b)
                .expect("agent_b state")
                .last_observation()
                .base_x_m;
        }

        assert!(
            final_a > 0.5,
            "agent A robot should move forward, x={final_a}"
        );
        assert!(
            final_b > 0.5,
            "agent B robot should move forward, x={final_b}"
        );
        let obs_a = sim
            .world()
            .get::<SharedDiffDriveAgentState>(agent_a)
            .expect("agent_a state")
            .last_observation();
        assert!(obs_a.goal_delta_x_m.is_some());
        assert!(obs_a.left_wheel_velocity_rad_s > 5.0);
    }
}
