//! Typed parsing of SSL `RobotControl` messages.

use crate::proto::{
    robot_move_command, RobotCommand, RobotControl, RobotControlResponse, RobotFeedback,
};
use crate::SslUdpError;
use prost::Message;

/// Team color that owns a robot-control UDP port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SslTeam {
    /// Blue team (default port 10301).
    Blue,
    /// Yellow team (default port 10302).
    Yellow,
}

/// Movement command after protobuf decode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SslMoveCommand {
    /// Four wheel speeds in meters per second.
    WheelVelocity {
        /// Front-right wheel.
        front_right_m_s: f32,
        /// Back-right wheel.
        back_right_m_s: f32,
        /// Back-left wheel.
        back_left_m_s: f32,
        /// Front-left wheel.
        front_left_m_s: f32,
    },
    /// Robot-local velocity.
    LocalVelocity {
        /// Forward speed in meters per second.
        forward_m_s: f32,
        /// Leftward speed in meters per second.
        left_m_s: f32,
        /// Yaw rate in radians per second.
        angular_rad_s: f32,
    },
    /// Field-frame velocity.
    GlobalVelocity {
        /// Field X speed in meters per second.
        x_m_s: f32,
        /// Field Y speed in meters per second.
        y_m_s: f32,
        /// Yaw rate in radians per second.
        angular_rad_s: f32,
    },
}

/// One robot command after protobuf decode.
#[derive(Clone, Debug, PartialEq)]
pub struct SslParsedRobotCommand {
    /// Robot id.
    pub id: u32,
    /// Optional movement.
    pub move_command: Option<SslMoveCommand>,
    /// Kick speed in meters per second.
    pub kick_speed_m_s: Option<f32>,
    /// Kick angle in degrees.
    pub kick_angle_deg: Option<f32>,
    /// Dribbler speed in rpm.
    pub dribbler_speed_rpm: Option<f32>,
}

/// Full `RobotControl` payload after protobuf decode.
#[derive(Clone, Debug, PartialEq)]
pub struct SslParsedRobotControl {
    /// Team that owns the socket.
    pub team: SslTeam,
    /// Commands in message order.
    pub commands: Vec<SslParsedRobotCommand>,
}

/// Decode raw UDP bytes into the official prost `RobotControl` message.
pub fn decode_robot_control(bytes: &[u8]) -> Result<RobotControl, SslUdpError> {
    RobotControl::decode(bytes).map_err(SslUdpError::Decode)
}

/// Encode an official prost `RobotControl` message.
pub fn encode_robot_control(control: &RobotControl) -> Vec<u8> {
    control.encode_to_vec()
}

/// Decode and map a `RobotControl` datagram into adapter types.
pub fn parse_robot_control(
    team: SslTeam,
    bytes: &[u8],
) -> Result<SslParsedRobotControl, SslUdpError> {
    let control = decode_robot_control(bytes)?;
    Ok(SslParsedRobotControl {
        team,
        commands: control
            .robot_commands
            .into_iter()
            .map(parse_robot_command)
            .collect(),
    })
}

fn parse_robot_command(command: RobotCommand) -> SslParsedRobotCommand {
    let move_command = command.move_command.and_then(|move_command| {
        move_command.command.map(|command| match command {
            robot_move_command::Command::WheelVelocity(wheels) => SslMoveCommand::WheelVelocity {
                front_right_m_s: wheels.front_right,
                back_right_m_s: wheels.back_right,
                back_left_m_s: wheels.back_left,
                front_left_m_s: wheels.front_left,
            },
            robot_move_command::Command::LocalVelocity(local) => SslMoveCommand::LocalVelocity {
                forward_m_s: local.forward,
                left_m_s: local.left,
                angular_rad_s: local.angular,
            },
            robot_move_command::Command::GlobalVelocity(global) => SslMoveCommand::GlobalVelocity {
                x_m_s: global.x,
                y_m_s: global.y,
                angular_rad_s: global.angular,
            },
        })
    });
    SslParsedRobotCommand {
        id: command.id,
        move_command,
        kick_speed_m_s: command.kick_speed,
        kick_angle_deg: command.kick_angle,
        dribbler_speed_rpm: command.dribbler_speed,
    }
}

/// Encode a `RobotControlResponse` with per-robot feedback and no errors.
pub fn encode_robot_control_response(feedback: &[RobotFeedback]) -> Vec<u8> {
    RobotControlResponse {
        errors: Vec::new(),
        feedback: feedback.to_vec(),
    }
    .encode_to_vec()
}
