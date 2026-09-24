//! Robot control systems.
//!
//! This module is split into cohesive submodules; every item keeps its
//! original `crate::systems::*` path via the re-exports below, so the
//! `AGENTS.md` convention "all new systems go in systems.rs" still applies to
//! this directory (new systems go in the submodule that matches their
//! subject, or in `actuation.rs` if none fits).

use crate::actuator::ControlMode;
use crate::commands::{ActuatorCommand, ActuatorCommandBuffer};
use crate::components::{
    AckermannDrive, Actuator, CombinedSlipTireSpec, CombinedSlipTireState,
    CorneringStiffnessLoadSensitivity, DcMotorCompletedTelemetry, DcMotorFailureMode, DcMotorSpec,
    DcMotorState, DrivenAxle, FourWheelVehicleSpec, Joint, JointKind, LongitudinalDrivePathState,
    LongitudinalLoadTransferSpec, LongitudinalMobilityPlantSpec, LongitudinalMobilityPlantState,
    MultirotorFlight, PwmMotorCommandFrontendSpec, PwmMotorCommandPolarity, RigidRoadPatchSpec,
    RigidRoadProfileSpec, SteeringActuatorFailureMode, SteeringActuatorSpec, SteeringActuatorState,
    SuspensionStrutSpec, TransmissionSpec, VehicleDynamics, WheelAssemblySpec, WheelStationSpec,
};
use crate::diff_drive::DifferentialDrive;
use crate::joint::{validate_joint_position, validate_joint_velocity, JointValidationError};
use bevy_ecs::prelude::{Entity, World};
use rne_core::SimDuration;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, ContactPointSample, ExternalBodyWrench, JointActuation, JointMotor,
    RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

mod actuation;
mod actuator_eval;
mod identification;
mod types;
mod vehicle_dynamics;

pub use actuation::*;
pub use actuator_eval::*;
pub use identification::*;
pub use types::*;
pub use vehicle_dynamics::*;

#[cfg(test)]
mod tests;
