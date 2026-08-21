//! Built-in environment implementations.

pub mod diff_drive;
pub mod mobile_manipulator;
pub mod office_agv_delivery;
pub mod office_agv_desk_place;
pub mod office_agv_shared_aisle;
pub mod ssl_small_pitch;
pub mod tsukuba_confirmation;
pub mod tsukuba_full_run;
pub mod urdf_scene;

pub use diff_drive::DiffDriveSim;
pub use diff_drive::{
    diff_drive_goal_task_spec, DiffDriveEpisode, DiffDriveEpisodeConfig, DiffDriveEpisodeSnapshot,
    DiffDriveEpisodeSnapshotError, VectorizedDiffDriveConfig, VectorizedDiffDriveEnv,
    VectorizedDiffDriveSnapshot, VectorizedDiffDriveSnapshotError, VectorizedDiffDriveStep,
};
pub use mobile_manipulator::{
    mm_lift_pick_scene_path, mm_lift_scene_path, mm_minimal_clutter_scene_path,
    mm_minimal_grasp_scene_path, mm_minimal_scene_path, mm_minimal_transport_scene_path,
    mm_mobile_clutter_scene_path, mm_mobile_lift_pick_place_scene_path, mm_mobile_lift_scene_path,
    mm_mobile_scene_path, mm_mobile_twist_to_wheel_velocities, wheel_command_to_motor_rad_s,
    ClutterPickConfig, GraspMode, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    MobileManipulatorEpisodeProgressSnapshot, MobileManipulatorEpisodeSnapshot,
    MobileManipulatorEpisodeSnapshotError, MobileManipulatorFixedJointSnapshot,
    MobileManipulatorFrameSnapshot, MobileManipulatorJointMotorSnapshot,
    MobileManipulatorPhysicsFactory, MobileManipulatorRigidBodySnapshot,
    MobileManipulatorSensorStateSnapshot, MobileManipulatorSim, MobileManipulatorSimSnapshot,
    MobileManipulatorSimSnapshotError, MobileManipulatorTransformSnapshot,
    VectorizedMobileManipulatorConfig, VectorizedMobileManipulatorEnv,
    VectorizedMobileManipulatorSnapshot, VectorizedMobileManipulatorSnapshotError,
    VectorizedMobileManipulatorStep, MM_MOBILE_TRACK_WIDTH_M, MM_MOBILE_WHEEL_JOINT_SIGN,
    MM_MOBILE_WHEEL_RADIUS_M, MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION,
    MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
};
pub use office_agv_delivery::{
    evaluate_office_desk_delivery_stop, office_agv_delivery_scene_path,
    office_agv_delivery_task_spec, OfficeAgvDeliveryCourse, OfficeAgvDeliveryFault,
    OfficeAgvDeliveryObservation, OfficeAgvDeliveryScenario, OfficePlanarAabb, OfficeStopJudgement,
    OFFICE_AGV_DELIVERY_TASK_ID, OFFICE_DELIVERY_DESK_NAME, OFFICE_DESK_DELIVERY_BEFORE_M,
    OFFICE_PICKUP_DOCK_NAME,
};
pub use office_agv_desk_place::{
    evaluate_office_desk_place, office_agv_desk_place_task_path, office_agv_desk_place_task_spec,
    OfficeAgvDeskPlaceCourse, OfficeAgvDeskPlaceFault, OfficeAgvDeskPlaceObservation,
    OfficeAgvDeskPlaceScenario, OFFICE_AGV_DESK_PLACE_TASK_ID, OFFICE_CARGO_NAME,
};
pub use office_agv_shared_aisle::{
    evaluate_office_shared_aisle_block, office_agv_shared_aisle_task_path,
    office_agv_shared_aisle_task_spec, OfficeAgvSharedAisleCourse, OfficeAgvSharedAisleFault,
    OfficeAgvSharedAisleObservation, OfficeAgvSharedAisleScenario, OFFICE_AGV_SHARED_AISLE_TASK_ID,
    OFFICE_ONCOMING_AGV_NAME,
};
pub use ssl_small_pitch::{
    evaluate_ssl_ball_region, ssl_small_pitch_scene_path, ssl_small_pitch_task_spec, SslBallRegion,
    SslSmallPitch, SslSmallPitchFault, SslSmallPitchObservation, SslSmallPitchScenario,
    SSL_BALL_NAME, SSL_BALL_RADIUS_M, SSL_DIV_B_FIELD_LENGTH_M, SSL_DIV_B_FIELD_WIDTH_M,
    SSL_GOAL_DEPTH_M, SSL_GOAL_WIDTH_M, SSL_MAX_BALL_SPEED_M_S, SSL_ROBOT_MAX_RADIUS_M,
    SSL_SMALL_PITCH_TASK_ID,
};
pub use tsukuba_confirmation::{
    evaluate_tsukuba_road_edge_stop, evaluate_tsukuba_stop_line, tsukuba_confirmation_scene_path,
    tsukuba_confirmation_task_spec, TsukubaConfirmationCourse, TsukubaConfirmationFault,
    TsukubaConfirmationObservation, TsukubaConfirmationScenario, TsukubaPlanarAabb,
    TsukubaStopJudgement, TSUKUBA_CONFIRMATION_TASK_ID, TSUKUBA_GREEN_CONE_NAME,
    TSUKUBA_ROAD_EDGE_BEFORE_M, TSUKUBA_STOP_LINE_AFTER_M, TSUKUBA_STOP_LINE_BEFORE_M,
};
pub use tsukuba_full_run::{
    tsukuba_full_run_scene_path, tsukuba_full_run_task_spec, TsukubaFullRunCourse,
    TsukubaFullRunFault, TsukubaFullRunObservation, TsukubaFullRunScenario,
    TSUKUBA_FULL_RUN_TASK_ID,
};
pub use urdf_scene::{
    cart_minimal_scene_path, humanoid_scene_path, lekiwi_scene_path, lekiwi_so101_scene_path,
    lekiwi_twist_to_wheel_velocities, lekiwi_wheel_command_to_motor_rad_s, quadruped_scene_path,
    quadruped_trot_targets, run_unitree_g1_commanded_gait,
    run_unitree_g1_commanded_gait_with_policy, so101_scene_path,
    step_unitree_g1_hybrid_joint_targets, step_unitree_g1_inspection, unitree_g1_dex3_pick_targets,
    unitree_g1_dex3_pick_targets_with_carry, unitree_g1_dex3_scene_path,
    unitree_g1_dynamic_scene_path, unitree_g1_factory_scene_path, unitree_g1_gait_targets,
    unitree_g1_gait_targets_for_velocity, unitree_g1_gait_targets_for_velocity_with_yaw_stride,
    unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase, unitree_g1_gait_task_spec,
    unitree_g1_inspection_targets, unitree_g1_parts_pick_place_scene_path, unitree_g1_scene_path,
    unitree_g1_workbench_task_spec, unitree_go2_dynamic_scene_path, unitree_go2_scene_path,
    unitree_go2_scheduled_targets, unitree_go2_task_spec, unitree_go2_terrain_scene_path,
    unitree_go2_trot_targets, unitree_go2_trot_targets_with_overlay, HumanoidAction,
    HumanoidEpisode, HumanoidEpisodeConfig, HumanoidObservation, QuadrupedAction, QuadrupedEpisode,
    QuadrupedEpisodeConfig, QuadrupedObservation, UnitreeG1Action, UnitreeG1CommandedGaitConfig,
    UnitreeG1CommandedGaitOutcome, UnitreeG1CommandedTorquePolicy, UnitreeG1Dex3Action,
    UnitreeG1Dex3BehaviorConfig, UnitreeG1Dex3BehaviorScenario, UnitreeG1Dex3Episode,
    UnitreeG1Dex3EpisodeConfig, UnitreeG1Dex3HandCommand, UnitreeG1Dex3Observation,
    UnitreeG1Dex3Phase, UnitreeG1Episode, UnitreeG1EpisodeConfig, UnitreeG1GaitAction,
    UnitreeG1GaitCommand, UnitreeG1GaitEpisode, UnitreeG1GaitEpisodeConfig,
    UnitreeG1GaitObservation, UnitreeG1InspectionAction, UnitreeG1InspectionEpisode,
    UnitreeG1InspectionEpisodeConfig, UnitreeG1InspectionObservation, UnitreeG1Observation,
    UnitreeG1PartsAction, UnitreeG1PartsEpisode, UnitreeG1PartsEpisodeConfig,
    UnitreeG1PartsObservation, UnitreeG1PartsPhase, UnitreeG1TorqueOverlay,
    UnitreeG1TorquePolicyInput, UnitreeG1VelocityCommand, UnitreeG1VelocityPolicyInput,
    UnitreeG1WorkbenchFault, UnitreeG1WorkbenchMissionConfig, UnitreeG1WorkbenchMissionScenario,
    UnitreeG1WorkbenchObservation, UnitreeGo2Action, UnitreeGo2Episode, UnitreeGo2EpisodeConfig,
    UnitreeGo2GaitCommand, UnitreeGo2GaitOverlay, UnitreeGo2GaitSchedule, UnitreeGo2LegSchedule,
    UnitreeGo2Observation, UnitreeGo2PureTorquePolicy, UnitreeGo2Push,
    UnitreeGo2TerrainObservation, UnitreeGo2TorqueOverlay, UnitreeGo2TorquePolicy,
    UnitreeGo2VelocityCommand, UnitreeGo2VelocityPolicyConfig, UnitreeGo2VelocityPolicyInput,
    UrdfArmAction, UrdfCartAction, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfKiwiAction,
    UrdfSceneObservation, UrdfSceneSim, VectorizedUnitreeG1GaitCheckpoint,
    VectorizedUnitreeG1GaitConfig, VectorizedUnitreeG1GaitEnv, VectorizedUnitreeG1GaitStep,
    VectorizedUnitreeGo2GaitCheckpoint, VectorizedUnitreeGo2GaitConfig,
    VectorizedUnitreeGo2GaitEnv, VectorizedUnitreeGo2GaitStep, G1_WORKBENCH_ARM_WINDOW_M,
    G1_WORKBENCH_MIN_PELVIS_Y_M, G1_WORKBENCH_MISSION_TASK_ID, G1_WORKBENCH_PARK_RADIUS_M,
    LEKIWI_DRIVE_WHEEL_LINKS, LEKIWI_WHEEL_AZIMUTH_RAD, LEKIWI_WHEEL_JOINT_SIGN,
    LEKIWI_WHEEL_PIVOT_RADIUS_M, LEKIWI_WHEEL_RADIUS_M, QUADRUPED_FOOT_LINKS,
    UNITREE_G1_HEADING_ENVELOPE_STEPS_V02, UNITREE_G1_HEADING_ENVELOPE_STEPS_V021,
    UNITREE_G1_HEADING_TARGET_CLAMP_RAD, UNITREE_G1_LEARNED_STRIDE_OVERLAY_SCALE,
    UNITREE_G1_POSITION_DAMPING, UNITREE_G1_POSITION_STIFFNESS, UNITREE_G1_SIM_DT_S,
    UNITREE_G1_SPEED_LIMIT_RAD_S, UNITREE_G1_TORQUE_LIMIT_NM, UNITREE_G1_TORQUE_LINKS,
    UNITREE_G1_TORQUE_PD_DAMPING, UNITREE_G1_TORQUE_PD_STIFFNESS, UNITREE_GO2_POLICY_FEATURES,
    UNITREE_GO2_PURE_TORQUE_PHASE_BINS,
};
