//! RoboCup SSL simulation-protocol adapter.
//!
//! Speaks the official UDP ports from
//! <https://github.com/RoboCup-SSL/ssl-simulation-protocol> without pulling
//! protobuf into core crates. This is a wire spike: it does not run physics
//! or replace the headless geometry scorer in `rne_ai::ssl_small_pitch`.

#![deny(missing_docs)]

mod ports;
/// Wire message types matching ssl-simulation-protocol field tags.
pub mod proto;
mod robot_control;
mod udp;

pub use ports::{
    SslSimulationPorts, SSL_ROBOT_CONTROL_BLUE_PORT, SSL_ROBOT_CONTROL_YELLOW_PORT,
    SSL_SIM_CONTROL_PORT,
};
pub use robot_control::{
    decode_robot_control, encode_robot_control, encode_robot_control_response, parse_robot_control,
    SslMoveCommand, SslParsedRobotCommand, SslParsedRobotControl, SslTeam,
};
pub use udp::{
    bind_ssl_ports, serve_robot_control_once, serve_simulator_control_once, SslBoundPorts,
    SslUdpError,
};

use prost::Message;

/// Decode a `SimulatorCommand` datagram (port 10300).
pub fn decode_simulator_command(bytes: &[u8]) -> Result<proto::SimulatorCommand, SslUdpError> {
    proto::SimulatorCommand::decode(bytes).map_err(SslUdpError::Decode)
}

/// Encode a `SimulatorCommand` datagram.
pub fn encode_simulator_command(command: &proto::SimulatorCommand) -> Vec<u8> {
    command.encode_to_vec()
}

/// Build an empty successful `SimulatorResponse`.
#[must_use]
pub fn empty_simulator_response() -> proto::SimulatorResponse {
    proto::SimulatorResponse { errors: Vec::new() }
}

/// Build a `SimulatorResponse` that reports an unsupported feature.
#[must_use]
pub fn unsupported_simulator_response(code: &str, message: &str) -> proto::SimulatorResponse {
    proto::SimulatorResponse {
        errors: vec![proto::SimulatorError {
            code: Some(code.to_string()),
            message: Some(message.to_string()),
        }],
    }
}

/// Encode a `SimulatorResponse` datagram.
pub fn encode_simulator_response(response: &proto::SimulatorResponse) -> Vec<u8> {
    response.encode_to_vec()
}

/// Decode a `SimulatorResponse` datagram.
pub fn decode_simulator_response(bytes: &[u8]) -> Result<proto::SimulatorResponse, SslUdpError> {
    proto::SimulatorResponse::decode(bytes).map_err(SslUdpError::Decode)
}

/// Construct a ball-teleport `SimulatorCommand` used by the spike smoke test.
#[must_use]
pub fn teleport_ball_command(x_m: f32, y_m: f32, z_m: f32) -> proto::SimulatorCommand {
    proto::SimulatorCommand {
        control: Some(proto::SimulatorControl {
            teleport_ball: Some(proto::TeleportBall {
                x: Some(x_m),
                y: Some(y_m),
                z: Some(z_m),
                vx: None,
                vy: None,
                vz: None,
                teleport_safely: None,
                roll: None,
                by_force: None,
            }),
            simulation_speed: None,
        }),
    }
}

pub use proto::{
    MoveGlobalVelocity, MoveLocalVelocity, MoveWheelVelocity, RobotCommand, RobotControl,
    RobotControlResponse, RobotFeedback, RobotMoveCommand, SimulatorCommand, SimulatorControl,
    SimulatorError, SimulatorResponse, TeleportBall,
};
