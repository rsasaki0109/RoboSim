//! Robot, link, joint, and actuator framework for Robot Native Engine.

#![deny(missing_docs)]

pub mod actuator;
pub mod commands;
pub mod components;
pub mod controller_io;
pub mod device;
pub mod diff_drive;
pub mod joint;
pub mod kinematics;
pub mod motion;
pub mod ros2_control;
pub mod self_collision;
pub mod systems;

pub use actuator::{ActuatorLimits, ActuatorTarget, ControlMode};
pub use commands::{ActuatorCommand, ActuatorCommandBuffer, ActuatorCommandEntry};
pub use components::{
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState,
    CorneringStiffnessLoadSensitivity, DcMotorCompletedTelemetry, DcMotorFailureMode, DcMotorSpec,
    DcMotorState, Device, DeviceKind, DrivenAxle, FloatingBase, FourWheelVehicleSpec, Joint,
    JointKind, JointLimits, LateralLoadTransferSpec, Link, LinkDevices, LongitudinalDrivePathState,
    LongitudinalLoadTransferSpec, LongitudinalMobilityPlantSpec, LongitudinalMobilityPlantState,
    MimicJoint, MultirotorFlight, PassiveCasterSpec, PassiveJoint, PwmMotorCommandFrontendSpec,
    PwmMotorCommandPolarity, RigidRoadPatchSpec, RigidRoadProfileSpec, Robot, RobotId,
    SteeringActuatorFailureMode, SteeringActuatorSpec, SteeringActuatorState, SuspensionStrutSpec,
    TransmissionSpec, VehicleDynamics, WheelAssemblySpec, WheelStationSpec, WheelSteeringState,
};
pub use controller_io::{
    apply_controller_output, build_controller_io, step_controller, Controller,
    ControllerApplyReport, ControllerCommand, ControllerIoError, ControllerIoFrame,
    ControllerJointState, ControllerOutput,
};
pub use device::{
    attach_device, detach_device, device_link, devices_of_kind, devices_of_link, spawn_device,
    DeviceError,
};
pub use diff_drive::{
    spawn_diff_drive_robot, DiffDriveComponent, DiffDriveConfig, DiffDriveDriveMode,
    DiffDriveSpawned, DifferentialDrive,
};
pub use joint::validate_joint_limits;
pub use kinematics::{
    AnalyticTwoLinkSolver, DampedLeastSquaresSolver, ForwardKinematics, IkOptions, IkRequest,
    IkSolution, Jacobian, JacobianTransposeSolver, KinematicModel, KinematicsError,
    KinematicsSolver, KinematicsSolverRegistry, RobotState, ANALYTIC_TWO_LINK_SOLVER,
    DAMPED_LEAST_SQUARES_SOLVER, JACOBIAN_TRANSPOSE_SOLVER,
};
pub use motion::{
    body_motion_from_world, BodyMotion, BodyMotionSample, JointInterpolation, JointKeyframe,
    JointTrack, MotionError,
};
pub use rne_physics::ColliderShape;
pub use rne_world::Transform3;
pub use ros2_control::{
    DiffDriveCommand, DiffDriveControllerConfig, DiffDriveWheelController, JointCommand,
    JointTrajectory, JointTrajectoryPoint, Ros2ControlError,
};
pub use self_collision::{
    check_self_collisions, segment_intersects_primitive, signed_distance,
    signed_distance_primitive_mesh, signed_distance_primitive_voxels, AllowedCollisionMatrix,
    AttachedBody, CollisionPairDistance, CollisionPrimitive, CollisionWorld, CollisionWorldObject,
    MeshCollisionObject, PathCollisionConfig, PathCollisionReport, PathCollisionSample,
    SelfCollisionChecker, SelfCollisionDistanceReport, SelfCollisionPair, SelfCollisionReport,
    VoxelGridObject, WorldCollisionDistance, WorldCollisionDistanceReport, WorldCollisionPair,
    WorldCollisionReport,
};
pub use systems::{
    ackermann_kinematics, aggregate_wheel_contact_patch, apply_actuator_commands,
    combined_slip_tire_wrench, command_ackermann_drive, command_multirotor,
    differential_drive_kinematics, evaluate_combined_slip_tire,
    evaluate_combined_slip_tire_steady_force, evaluate_dc_motor, evaluate_longitudinal_drive_path,
    evaluate_longitudinal_mobility_plant, evaluate_pwm_motor_command, evaluate_steering_actuator,
    evaluate_suspension_strut, evaluate_transmission, fit_suspension_training_runs,
    identify_combined_slip_tire_steady, identify_steering_actuator_first_order,
    identify_suspension_strut, identify_suspension_strut_runs,
    identify_suspension_strut_runs_report, identify_tire_load_sensitivity,
    identify_tire_relaxation_length, multirotor_flight, pure_pursuit_steering,
    resolve_wheel_station_frame, rigid_road_patch_geometry, sample_rigid_road_profile,
    suspension_training_influence, sync_all_joint_motors_from_actuators,
    sync_joint_motors_from_actuators, vehicle_dynamics, wheel_rolling_resistance_torque_nm,
    AckermannCommandResult, CombinedSlipTireEvaluation, CombinedSlipTireIdentificationResult,
    CombinedSlipTireInput, CommandApplyResult, DcMotorEvaluation, LongitudinalDrivePathEvaluation,
    LongitudinalDrivePathInput, LongitudinalMobilityPlantEvaluation, MobilityPlantEvaluationError,
    MultirotorCommandResult, PwmMotorCommandEvaluation, RigidRoadPatchGeometry,
    RigidRoadSurfaceSample, SteeringActuatorEvaluation, SteeringActuatorIdentificationError,
    SteeringActuatorIdentificationResult, SteeringActuatorIdentificationSample,
    SteeringActuatorIdentificationSpec, SuspensionAcquisitionInfluence, SuspensionForceSample,
    SuspensionIdentificationError, SuspensionIdentificationResult, SuspensionIdentificationRun,
    SuspensionIdentificationSpec, SuspensionRunIdentificationReport, SuspensionRunResidual,
    SuspensionTrainingCoefficients, SuspensionTrainingInfluence, TireConditionResidual,
    TireForceIdentificationSample, TireIdentificationError, TireIdentificationRun,
    TireIdentificationSpec, TireLoadSensitivityIdentificationResult,
    TireLoadSensitivityIdentificationSpec, TireRelaxationAxis, TireRelaxationConditionResidual,
    TireRelaxationIdentificationError, TireRelaxationIdentificationResult,
    TireRelaxationIdentificationRun, TireRelaxationIdentificationSample,
    TireRelaxationIdentificationSpec, TransmissionEvaluation, WheelContactPatch, WheelStationFrame,
};
