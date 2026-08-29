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
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState, DcMotorFailureMode,
    DcMotorSpec, DcMotorState, Joint, JointKind, JointLimits, Link, MultirotorFlight, Robot,
    RobotId, TransmissionSpec, VehicleDynamics, WheelAssemblySpec,
};
pub use diff_drive::{
    spawn_diff_drive_robot, DiffDriveComponent, DiffDriveConfig, DiffDriveDriveMode,
    DiffDriveSpawned, DifferentialDrive,
};
pub use joint::validate_joint_limits;
pub use systems::{
    ackermann_kinematics, aggregate_wheel_contact_patch, apply_actuator_commands,
    combined_slip_tire_wrench, command_ackermann_drive, command_multirotor,
    differential_drive_kinematics, evaluate_combined_slip_tire, evaluate_dc_motor,
    evaluate_transmission, multirotor_flight, pure_pursuit_steering,
    sync_all_joint_motors_from_actuators, sync_joint_motors_from_actuators, vehicle_dynamics,
    wheel_rolling_resistance_torque_nm, AckermannCommandResult, CombinedSlipTireEvaluation,
    CombinedSlipTireInput, CommandApplyResult, DcMotorEvaluation, MobilityPlantEvaluationError,
    MultirotorCommandResult, TransmissionEvaluation, WheelContactPatch,
};
