//! UDP helpers for the SSL simulation-protocol ports.

use crate::ports::SslSimulationPorts;
use crate::proto::{RobotFeedback, SimulatorCommand};
use crate::robot_control::{
    encode_robot_control_response, parse_robot_control, SslParsedRobotControl, SslTeam,
};
use crate::{
    decode_simulator_command, empty_simulator_response, encode_simulator_response,
    unsupported_simulator_response,
};
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

/// Errors raised while binding or exchanging SSL UDP datagrams.
#[derive(Debug, thiserror::Error)]
pub enum SslUdpError {
    /// OS socket failure.
    #[error("ssl udp io: {0}")]
    Io(#[from] io::Error),
    /// Protobuf decode failure.
    #[error("ssl protobuf decode: {0}")]
    Decode(#[from] prost::DecodeError),
}

/// Bound SSL simulation sockets.
#[derive(Debug)]
pub struct SslBoundPorts {
    /// Simulation-control socket (10300 by default).
    pub control: UdpSocket,
    /// Blue robot-control socket (10301 by default).
    pub blue: UdpSocket,
    /// Yellow robot-control socket (10302 by default).
    pub yellow: UdpSocket,
}

impl SslBoundPorts {
    /// Local addresses actually bound (useful when ports were ephemeral).
    pub fn local_ports(&self) -> Result<SslSimulationPorts, SslUdpError> {
        Ok(SslSimulationPorts {
            control: self.control.local_addr()?.port(),
            blue: self.blue.local_addr()?.port(),
            yellow: self.yellow.local_addr()?.port(),
        })
    }
}

/// Bind the three SSL simulation-protocol sockets.
pub fn bind_ssl_ports(ports: SslSimulationPorts) -> Result<SslBoundPorts, SslUdpError> {
    let control = UdpSocket::bind(("127.0.0.1", ports.control))?;
    let blue = UdpSocket::bind(("127.0.0.1", ports.blue))?;
    let yellow = UdpSocket::bind(("127.0.0.1", ports.yellow))?;
    for socket in [&control, &blue, &yellow] {
        socket.set_read_timeout(Some(Duration::from_secs(2)))?;
        socket.set_write_timeout(Some(Duration::from_secs(2)))?;
    }
    Ok(SslBoundPorts {
        control,
        blue,
        yellow,
    })
}

/// Serve one robot-control datagram and reply with feedback for each command.
pub fn serve_robot_control_once(
    socket: &UdpSocket,
    team: SslTeam,
) -> Result<(SslParsedRobotControl, SocketAddr), SslUdpError> {
    let mut buffer = [0_u8; 65_507];
    let (len, peer) = socket.recv_from(&mut buffer)?;
    let parsed = parse_robot_control(team, &buffer[..len])?;
    let feedback: Vec<RobotFeedback> = parsed
        .commands
        .iter()
        .map(|command| RobotFeedback {
            id: command.id,
            dribbler_ball_contact: Some(false),
        })
        .collect();
    let response = encode_robot_control_response(&feedback);
    socket.send_to(&response, peer)?;
    Ok((parsed, peer))
}

/// Serve one simulation-control datagram.
///
/// Ball teleports are acknowledged with an empty response. Any other control
/// payload returns an unsupported-feature error so callers know the spike
/// surface.
pub fn serve_simulator_control_once(
    socket: &UdpSocket,
) -> Result<(SimulatorCommand, SocketAddr), SslUdpError> {
    let mut buffer = [0_u8; 65_507];
    let (len, peer) = socket.recv_from(&mut buffer)?;
    let command = decode_simulator_command(&buffer[..len])?;
    let response = match command.control.as_ref() {
        Some(control) if control.teleport_ball.is_some() => empty_simulator_response(),
        Some(_) => unsupported_simulator_response(
            "UNSUPPORTED",
            "rne_adapter_ssl spike only acknowledges teleport_ball",
        ),
        None => unsupported_simulator_response(
            "UNSUPPORTED",
            "rne_adapter_ssl spike requires SimulatorControl.teleport_ball",
        ),
    };
    socket.send_to(&encode_simulator_response(&response), peer)?;
    Ok((command, peer))
}
