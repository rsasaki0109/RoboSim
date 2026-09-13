//! Robot, link, joint, and actuator framework for Robot Native Engine.

#![deny(missing_docs)]

pub mod actuator;
pub mod commands;
pub mod components;
pub mod diff_drive;
pub mod joint;
pub mod systems;

pub use actuator::{ActuatorLimits, ActuatorTarget, ControlMode};
pub use commands::{ActuatorCommand, ActuatorCommandBuffer, ActuatorCommandEntry};
pub use components::{
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState,
    DcMotorCompletedTelemetry, DcMotorFailureMode, DcMotorSpec, DcMotorState, Joint, JointKind,
    JointLimits, Link, LongitudinalDrivePathState, LongitudinalMobilityPlantSpec,
    LongitudinalMobilityPlantState, MultirotorFlight, PassiveCasterSpec,
    PwmMotorCommandFrontendSpec, PwmMotorCommandPolarity, RigidRoadPatchSpec, RigidRoadProfileSpec,
    Robot, RobotId, SteeringActuatorFailureMode, SteeringActuatorSpec, SteeringActuatorState,
    SuspensionStrutSpec, TransmissionSpec, VehicleDynamics, WheelAssemblySpec, WheelStationSpec,
    WheelSteeringState,
};
pub use diff_drive::{
    spawn_diff_drive_robot, DiffDriveComponent, DiffDriveConfig, DiffDriveDriveMode,
    DiffDriveSpawned, DifferentialDrive,
};
pub use joint::validate_joint_limits;
pub use systems::{
    ackermann_kinematics, aggregate_wheel_contact_patch, apply_actuator_commands,
    combined_slip_tire_wrench, command_ackermann_drive, command_multirotor,
    differential_drive_kinematics, evaluate_combined_slip_tire,
    evaluate_combined_slip_tire_steady_force, evaluate_dc_motor, evaluate_longitudinal_drive_path,
    evaluate_longitudinal_mobility_plant, evaluate_pwm_motor_command, evaluate_steering_actuator,
    evaluate_suspension_strut, evaluate_transmission, fit_suspension_training_runs,
    identify_combined_slip_tire_steady, identify_steering_actuator_first_order,
    identify_suspension_strut, identify_suspension_strut_runs,
    identify_suspension_strut_runs_report, identify_tire_relaxation_length, multirotor_flight,
    pure_pursuit_steering, resolve_wheel_station_frame, rigid_road_patch_geometry,
    sample_rigid_road_profile, suspension_training_influence, sync_all_joint_motors_from_actuators,
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
    TireIdentificationSpec, TireRelaxationAxis, TireRelaxationConditionResidual,
    TireRelaxationIdentificationError, TireRelaxationIdentificationResult,
    TireRelaxationIdentificationRun, TireRelaxationIdentificationSample,
    TireRelaxationIdentificationSpec, TransmissionEvaluation, WheelContactPatch, WheelStationFrame,
};
