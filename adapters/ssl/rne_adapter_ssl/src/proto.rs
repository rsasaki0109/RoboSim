//! Hand-written prost messages matching ssl-simulation-protocol field tags.
//!
//! Generated from the official `.proto` files without requiring `protoc` in CI.
//! Unknown fields (for example full `SimulatorConfig`) are skipped on decode.

use prost::Message;

/// Official `RobotControl` wrapper.
#[derive(Clone, PartialEq, Message)]
pub struct RobotControl {
    /// Per-robot commands.
    #[prost(message, repeated, tag = "1")]
    pub robot_commands: Vec<RobotCommand>,
}

/// Official `RobotCommand`.
#[derive(Clone, PartialEq, Message)]
pub struct RobotCommand {
    /// Robot id.
    #[prost(uint32, tag = "1")]
    pub id: u32,
    /// Optional movement command.
    #[prost(message, optional, tag = "2")]
    pub move_command: Option<RobotMoveCommand>,
    /// Absolute kick speed in meters per second.
    #[prost(float, optional, tag = "3")]
    pub kick_speed: Option<f32>,
    /// Kick angle in degrees.
    #[prost(float, optional, tag = "4")]
    pub kick_angle: Option<f32>,
    /// Dribbler speed in rpm.
    #[prost(float, optional, tag = "5")]
    pub dribbler_speed: Option<f32>,
}

/// Official `RobotMoveCommand` oneof wrapper.
#[derive(Clone, PartialEq, Message)]
pub struct RobotMoveCommand {
    /// Movement variant.
    #[prost(oneof = "robot_move_command::Command", tags = "1, 2, 3")]
    pub command: Option<robot_move_command::Command>,
}

/// Nested oneof variants for [`RobotMoveCommand`].
pub mod robot_move_command {
    /// Movement command payload.
    #[derive(Clone, PartialEq, ::prost::Oneof)]
    pub enum Command {
        /// Wheel velocities in meters per second.
        #[prost(message, tag = "1")]
        WheelVelocity(super::MoveWheelVelocity),
        /// Local-frame velocity.
        #[prost(message, tag = "2")]
        LocalVelocity(super::MoveLocalVelocity),
        /// Global-frame velocity.
        #[prost(message, tag = "3")]
        GlobalVelocity(super::MoveGlobalVelocity),
    }
}

/// Official `MoveWheelVelocity`.
#[derive(Clone, PartialEq, Message)]
pub struct MoveWheelVelocity {
    /// Front-right wheel velocity in meters per second.
    #[prost(float, tag = "1")]
    pub front_right: f32,
    /// Back-right wheel velocity in meters per second.
    #[prost(float, tag = "2")]
    pub back_right: f32,
    /// Back-left wheel velocity in meters per second.
    #[prost(float, tag = "3")]
    pub back_left: f32,
    /// Front-left wheel velocity in meters per second.
    #[prost(float, tag = "4")]
    pub front_left: f32,
}

/// Official `MoveLocalVelocity`.
#[derive(Clone, PartialEq, Message)]
pub struct MoveLocalVelocity {
    /// Forward velocity in meters per second.
    #[prost(float, tag = "1")]
    pub forward: f32,
    /// Leftward velocity in meters per second.
    #[prost(float, tag = "2")]
    pub left: f32,
    /// Counter-clockwise angular velocity in radians per second.
    #[prost(float, tag = "3")]
    pub angular: f32,
}

/// Official `MoveGlobalVelocity`.
#[derive(Clone, PartialEq, Message)]
pub struct MoveGlobalVelocity {
    /// Field X velocity in meters per second.
    #[prost(float, tag = "1")]
    pub x: f32,
    /// Field Y velocity in meters per second.
    #[prost(float, tag = "2")]
    pub y: f32,
    /// Counter-clockwise angular velocity in radians per second.
    #[prost(float, tag = "3")]
    pub angular: f32,
}

/// Official `SimulatorError`.
#[derive(Clone, PartialEq, Message)]
pub struct SimulatorError {
    /// Machine-readable error code.
    #[prost(string, optional, tag = "1")]
    pub code: Option<String>,
    /// Human-readable message.
    #[prost(string, optional, tag = "2")]
    pub message: Option<String>,
}

/// Official `RobotFeedback` without the optional `Any` custom payload.
#[derive(Clone, PartialEq, Message)]
pub struct RobotFeedback {
    /// Robot id.
    #[prost(uint32, tag = "1")]
    pub id: u32,
    /// Whether the dribbler currently contacts the ball.
    #[prost(bool, optional, tag = "2")]
    pub dribbler_ball_contact: Option<bool>,
}

/// Official `RobotControlResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct RobotControlResponse {
    /// Protocol / capability errors.
    #[prost(message, repeated, tag = "1")]
    pub errors: Vec<SimulatorError>,
    /// Per-robot feedback.
    #[prost(message, repeated, tag = "2")]
    pub feedback: Vec<RobotFeedback>,
}

/// Official `TeleportBall`.
#[derive(Clone, PartialEq, Message)]
pub struct TeleportBall {
    /// X in meters.
    #[prost(float, optional, tag = "1")]
    pub x: Option<f32>,
    /// Y in meters.
    #[prost(float, optional, tag = "2")]
    pub y: Option<f32>,
    /// Z in meters.
    #[prost(float, optional, tag = "3")]
    pub z: Option<f32>,
    /// Velocity X in meters per second.
    #[prost(float, optional, tag = "4")]
    pub vx: Option<f32>,
    /// Velocity Y in meters per second.
    #[prost(float, optional, tag = "5")]
    pub vy: Option<f32>,
    /// Velocity Z in meters per second.
    #[prost(float, optional, tag = "6")]
    pub vz: Option<f32>,
    /// Teleport safely around robots.
    #[prost(bool, optional, tag = "7")]
    pub teleport_safely: Option<bool>,
    /// Force rolling angular velocity.
    #[prost(bool, optional, tag = "8")]
    pub roll: Option<bool>,
    /// Apply force instead of an instant teleport.
    #[prost(bool, optional, tag = "9")]
    pub by_force: Option<bool>,
}

/// Spike-sized `SimulatorControl` (ball teleport + speed; robot teleport later).
#[derive(Clone, PartialEq, Message)]
pub struct SimulatorControl {
    /// Optional ball teleport.
    #[prost(message, optional, tag = "1")]
    pub teleport_ball: Option<TeleportBall>,
    /// Simulation speed multiplier.
    #[prost(float, optional, tag = "3")]
    pub simulation_speed: Option<f32>,
}

/// Spike-sized `SimulatorCommand` (control only; config later).
#[derive(Clone, PartialEq, Message)]
pub struct SimulatorCommand {
    /// Control payload.
    #[prost(message, optional, tag = "1")]
    pub control: Option<SimulatorControl>,
}

/// Official `SimulatorResponse`.
#[derive(Clone, PartialEq, Message)]
pub struct SimulatorResponse {
    /// Protocol / capability errors.
    #[prost(message, repeated, tag = "1")]
    pub errors: Vec<SimulatorError>,
}
