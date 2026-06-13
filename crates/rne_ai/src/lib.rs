//! Episode API, rewards, and policy traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod action;
pub mod agent;
pub mod domain_randomization;
pub mod env;
pub mod episode;
pub mod observation;
pub mod policy;
pub mod reward;
pub mod rng;

pub use action::DiffDriveAction;
pub use agent::{
    attach_diff_drive_policy, attach_shared_diff_drive_policy, observe_shared_diff_drive_agent,
    reset_diff_drive_agent, spawn_diff_drive_agent, spawn_shared_diff_drive_agent,
    step_diff_drive_agent, step_diff_drive_agents, step_shared_diff_drive_agent,
    step_shared_diff_drive_agents, Agent, AgentKind, AgentTarget, AttachedPolicy,
    DiffDriveAgentState, SharedDiffDriveAgentState,
};
pub use domain_randomization::DiffDriveDomainRandomization;
pub use env::{
    DiffDriveEpisode, DiffDriveEpisodeConfig, DiffDriveSim, VectorizedDiffDriveConfig,
    VectorizedDiffDriveEnv, VectorizedDiffDriveStep,
};
pub use episode::{Episode, EpisodeStep, TerminationReason};
pub use observation::DiffDriveObservation;
pub use policy::{ConstantVelocityPolicy, Policy};
pub use reward::DiffDriveRewardConfig;
pub use rng::DeterministicRng;
