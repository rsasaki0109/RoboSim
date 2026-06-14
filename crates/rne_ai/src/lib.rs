//! Episode API, rewards, and policy traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod action;
pub mod agent;
pub mod domain_randomization;
pub mod env;
pub mod episode;
pub mod goal;
pub mod lidar;
pub mod multi_robot;
pub mod observation;
pub mod policy;
pub mod render;
pub mod reward;
pub mod rng;

pub use action::{DiffDriveAction, MobileManipulatorAction};
pub use agent::{
    attach_diff_drive_policy, attach_goal_conditioned_policy, attach_shared_diff_drive_policy,
    attach_shared_goal_conditioned_policy, observe_shared_diff_drive_agent, reset_diff_drive_agent,
    spawn_diff_drive_agent, spawn_shared_diff_drive_agent, spawn_shared_diff_drive_agent_for_robot,
    step_diff_drive_agent, step_diff_drive_agents, step_shared_diff_drive_agent,
    step_shared_diff_drive_agents, Agent, AgentGoal, AgentKind, AgentTarget, AttachedPolicy,
    DiffDriveAgentState, SharedDiffDriveAgentState,
};
pub use domain_randomization::DiffDriveDomainRandomization;
pub use env::{
    mm_mobile_twist_to_wheel_velocities, wheel_command_to_motor_rad_s, DiffDriveEpisode,
    DiffDriveEpisodeConfig, DiffDriveSim, MobileManipulatorSim, VectorizedDiffDriveConfig,
    VectorizedDiffDriveEnv, VectorizedDiffDriveStep,
};
pub use episode::{Episode, EpisodeStep, TerminationReason};
pub use goal::{
    goal_x_from_observation, GoalConditionedAdapter, GoalConditionedPolicy, GoalCurriculum,
    GoalCurriculumConfig, GoalCurriculumStage, GoalSeekingPolicy, GoalTaskSet,
};
pub use lidar::{lidar_mounts_from_spawned, lidar_stream_for_index, sync_lidar_mounts, LidarMount};
pub use multi_robot::{
    head_on_collision_configs, head_on_collision_sim, inter_robot_contacts, last_contacts,
    nearest_peer_observation, robot_separation_m, robots_in_contact, PeerObservation,
};
pub use observation::{DiffDriveObservation, MobileManipulatorObservation};
pub use policy::{ConstantVelocityPolicy, Policy};
pub use render::{
    append_lidar_overlay, build_diff_drive_render_scene, build_visual_render_scene,
    LidarOverlayStats,
};
pub use reward::DiffDriveRewardConfig;
pub use rng::DeterministicRng;
