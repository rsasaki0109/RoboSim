//! Episode API, rewards, and policy traits for Robot Native Engine.

#![deny(missing_docs)]

pub mod action;
pub mod agent;
pub mod camera;
pub mod domain_randomization;
pub mod env;
pub mod episode;
pub mod goal;
pub mod grasp;
pub mod joint_trajectory;
pub mod lidar;
pub mod mm_lift_kinematics;
pub mod multi_robot;
pub mod observation;
pub mod policy;
pub mod reach;
pub mod render;
pub mod reward;
pub mod rng;
pub mod transport;

pub use action::{DiffDriveAction, MobileManipulatorAction};
pub use agent::{
    attach_diff_drive_policy, attach_goal_conditioned_policy, attach_shared_diff_drive_policy,
    attach_shared_goal_conditioned_policy, observe_shared_diff_drive_agent, reset_diff_drive_agent,
    spawn_diff_drive_agent, spawn_shared_diff_drive_agent, spawn_shared_diff_drive_agent_for_robot,
    step_diff_drive_agent, step_diff_drive_agents, step_shared_diff_drive_agent,
    step_shared_diff_drive_agents, Agent, AgentGoal, AgentKind, AgentTarget, AttachedPolicy,
    DiffDriveAgentState, SharedDiffDriveAgentState,
};
pub use camera::{
    sync_wrist_camera_mount, sync_wrist_camera_mounts, wrist_camera_image_valid,
    wrist_camera_mounts_from_spawned, wrist_camera_pixel_count, wrist_camera_stream_for_index,
    WristCameraMount,
};
pub use domain_randomization::DiffDriveDomainRandomization;
pub use env::{
    mm_lift_pick_scene_path, mm_lift_scene_path, mm_minimal_grasp_scene_path,
    mm_minimal_scene_path, mm_minimal_transport_scene_path, mm_mobile_scene_path,
    mm_mobile_twist_to_wheel_velocities, wheel_command_to_motor_rad_s, DiffDriveEpisode,
    DiffDriveEpisodeConfig, DiffDriveEpisodeSnapshot, DiffDriveEpisodeSnapshotError, DiffDriveSim,
    MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    MobileManipulatorEpisodeProgressSnapshot, MobileManipulatorEpisodeSnapshot,
    MobileManipulatorEpisodeSnapshotError, MobileManipulatorFixedJointSnapshot,
    MobileManipulatorFrameSnapshot, MobileManipulatorJointMotorSnapshot,
    MobileManipulatorRigidBodySnapshot, MobileManipulatorSensorStateSnapshot, MobileManipulatorSim,
    MobileManipulatorSimSnapshot, MobileManipulatorSimSnapshotError,
    MobileManipulatorTransformSnapshot, VectorizedDiffDriveConfig, VectorizedDiffDriveEnv,
    VectorizedDiffDriveSnapshot, VectorizedDiffDriveSnapshotError, VectorizedDiffDriveStep,
    VectorizedMobileManipulatorConfig, VectorizedMobileManipulatorEnv,
    VectorizedMobileManipulatorSnapshot, VectorizedMobileManipulatorSnapshotError,
    VectorizedMobileManipulatorStep,
};
pub use episode::{Episode, EpisodeRandomSnapshot, EpisodeStep, TerminationReason};
pub use goal::{
    goal_x_from_observation, GoalConditionedAdapter, GoalConditionedPolicy, GoalCurriculum,
    GoalCurriculumConfig, GoalCurriculumSnapshot, GoalCurriculumSnapshotError, GoalCurriculumStage,
    GoalSeekingPolicy, GoalTaskSet,
};
pub use grasp::{finger_contacts_named, sim_contacts_named, FINGER_LINK_NAMES};
pub use joint_trajectory::{
    hold_lift_joint_action, joint_tracking_action, JointTrackingLimits, JointTrajectory,
};
pub use lidar::{lidar_mounts_from_spawned, lidar_stream_for_index, sync_lidar_mounts, LidarMount};
pub use mm_lift_kinematics::{
    MmLiftGripperTarget, MmLiftIkError, MmLiftJointTarget, MmLiftKinematics,
};
pub use multi_robot::{
    head_on_collision_configs, head_on_collision_sim, inter_robot_contacts, last_contacts,
    nearest_peer_observation, robot_separation_m, robots_in_contact, PeerObservation,
};
pub use observation::{DiffDriveObservation, MobileManipulatorObservation};
pub use policy::{ConstantVelocityPolicy, LiftPickPlacePolicy, Policy};
pub use reach::{
    ee_distance_to_target_m, reach_action_joint_proportional, reach_action_proportional,
    JointReachTarget, ReachCurriculum, ReachCurriculumConfig, ReachCurriculumSnapshot,
    ReachCurriculumSnapshotError, ReachCurriculumStage, ReachRandomization, ReachTarget,
};
pub use render::{
    append_lidar_overlay, build_diff_drive_render_scene, build_visual_render_scene,
    LidarOverlayStats,
};
pub use reward::{DiffDriveRewardConfig, MobileManipulatorRewardConfig, MobileManipulatorTask};
pub use rng::DeterministicRng;
pub use transport::{
    body_moved_at_least_m, body_within_zone_m, displacement_m, had_finger_contact,
    named_translation_m, TRANSPORT_SUCCESS_M,
};
