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
pub mod mm_minimal_kinematics;
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
    cart_minimal_scene_path, humanoid_scene_path, lekiwi_scene_path, lekiwi_so101_scene_path,
    lekiwi_twist_to_wheel_velocities, lekiwi_wheel_command_to_motor_rad_s, mm_lift_pick_scene_path,
    mm_lift_scene_path, mm_minimal_clutter_scene_path, mm_minimal_grasp_scene_path,
    mm_minimal_scene_path, mm_minimal_transport_scene_path, mm_mobile_clutter_scene_path,
    mm_mobile_scene_path, mm_mobile_twist_to_wheel_velocities, quadruped_scene_path,
    quadruped_trot_targets, so101_scene_path, unitree_g1_dynamic_scene_path,
    unitree_g1_factory_scene_path, unitree_g1_gait_targets, unitree_g1_inspection_targets,
    unitree_g1_parts_pick_place_scene_path, unitree_g1_scene_path, unitree_go2_dynamic_scene_path,
    unitree_go2_scene_path, unitree_go2_trot_targets, wheel_command_to_motor_rad_s,
    ClutterPickConfig, DiffDriveEpisode, DiffDriveEpisodeConfig, DiffDriveEpisodeSnapshot,
    DiffDriveEpisodeSnapshotError, DiffDriveSim, GraspMode, HumanoidAction, HumanoidEpisode,
    HumanoidEpisodeConfig, HumanoidObservation, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, MobileManipulatorEpisodeProgressSnapshot,
    MobileManipulatorEpisodeSnapshot, MobileManipulatorEpisodeSnapshotError,
    MobileManipulatorFixedJointSnapshot, MobileManipulatorFrameSnapshot,
    MobileManipulatorJointMotorSnapshot, MobileManipulatorRigidBodySnapshot,
    MobileManipulatorSensorStateSnapshot, MobileManipulatorSim, MobileManipulatorSimSnapshot,
    MobileManipulatorSimSnapshotError, MobileManipulatorTransformSnapshot, QuadrupedAction,
    QuadrupedEpisode, QuadrupedEpisodeConfig, QuadrupedObservation, UnitreeG1Action,
    UnitreeG1Episode, UnitreeG1EpisodeConfig, UnitreeG1GaitAction, UnitreeG1GaitCommand,
    UnitreeG1GaitEpisode, UnitreeG1GaitEpisodeConfig, UnitreeG1GaitObservation,
    UnitreeG1InspectionAction, UnitreeG1InspectionEpisode, UnitreeG1InspectionEpisodeConfig,
    UnitreeG1InspectionObservation, UnitreeG1Observation, UnitreeG1PartsAction,
    UnitreeG1PartsEpisode, UnitreeG1PartsEpisodeConfig, UnitreeG1PartsObservation,
    UnitreeG1PartsPhase, UnitreeGo2Action, UnitreeGo2Episode, UnitreeGo2EpisodeConfig,
    UnitreeGo2GaitCommand, UnitreeGo2Observation, UrdfArmAction, UrdfCartAction,
    UrdfJointPositionTarget, UrdfKiwiAction, UrdfSceneObservation, UrdfSceneSim,
    VectorizedDiffDriveConfig, VectorizedDiffDriveEnv, VectorizedDiffDriveSnapshot,
    VectorizedDiffDriveSnapshotError, VectorizedDiffDriveStep, VectorizedMobileManipulatorConfig,
    VectorizedMobileManipulatorEnv, VectorizedMobileManipulatorSnapshot,
    VectorizedMobileManipulatorSnapshotError, VectorizedMobileManipulatorStep,
    LEKIWI_DRIVE_WHEEL_LINKS, LEKIWI_WHEEL_AZIMUTH_RAD, LEKIWI_WHEEL_JOINT_SIGN,
    LEKIWI_WHEEL_PIVOT_RADIUS_M, LEKIWI_WHEEL_RADIUS_M, QUADRUPED_FOOT_LINKS,
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
pub use mm_minimal_kinematics::{
    mm_minimal_clutter_place_target, mm_mobile_clutter_place_target, MmMinimalGripperTarget,
    MmMinimalIkError, MmMinimalJointTarget, MmMinimalKinematics, MM_MINIMAL_CLUTTER_PLACE_X_M,
    MM_MINIMAL_CLUTTER_PLACE_Y_M, MM_MINIMAL_CLUTTER_PLACE_Z_M, MM_MOBILE_CLUTTER_PLACE_X_M,
    MM_MOBILE_CLUTTER_PLACE_Y_M, MM_MOBILE_CLUTTER_PLACE_Z_M,
};
pub use multi_robot::{
    head_on_collision_configs, head_on_collision_sim, inter_robot_contacts, last_contacts,
    nearest_peer_observation, robot_separation_m, robots_in_contact, PeerObservation,
};
pub use observation::{DiffDriveObservation, MobileManipulatorObservation};
pub use policy::{
    ConstantVelocityPolicy, IkClutterPickPlacePolicy, IkLiftPickPlacePolicy,
    IkMobileClutterPickPlacePolicy, LiftPickPlacePolicy, Policy, VisuomotorReachPolicy,
};
pub use reach::{
    ee_distance_to_target_m, reach_action_joint_proportional, reach_action_proportional,
    JointReachTarget, ReachCurriculum, ReachCurriculumConfig, ReachCurriculumSnapshot,
    ReachCurriculumSnapshotError, ReachCurriculumStage, ReachRandomization, ReachTarget,
};
pub use render::{
    append_lidar_overlay, append_task_marker_overlay, build_diff_drive_render_scene,
    build_visual_render_scene, LidarOverlayStats, TaskMarkerOverlayStats,
};
pub use reward::{DiffDriveRewardConfig, MobileManipulatorRewardConfig, MobileManipulatorTask};
pub use rng::DeterministicRng;
pub use transport::{
    body_moved_at_least_m, body_within_zone_m, displacement_m, had_finger_contact,
    named_translation_m, TRANSPORT_SUCCESS_M,
};
