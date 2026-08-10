//! Minimal TraCI client for live SUMO co-simulation.
//!
//! TraCI (Traffic Control Interface) is SUMO's TCP protocol for live
//! co-simulation. This crate implements the small subset a co-simulation
//! adapter needs to drive and observe a running SUMO process:
//!
//! - [`TraciClient::get_version`] (command `0x00`)
//! - [`TraciClient::simulation_step`] (command `0x02`)
//! - [`TraciClient::vehicle_ids`] and [`TraciClient::vehicle_position`]
//!   (domain command `0xa4`)
//! - [`TraciClient::set_vehicle_speed_m_s`] (vehicle state command `0xc4`)
//! - [`TraciClient::close`] (command `0x7f`)
//!
//! The wire format follows SUMO's reference `traci` implementation: every TCP
//! message starts with a 4-byte big-endian length that includes the length
//! field itself, followed by commands. Each command is
//! `<1-byte length><command id><payload>`, extended to
//! `<0x00><4-byte length><command id><payload>` when it exceeds 254 bytes.
//! Integers and doubles are big-endian; strings are a 4-byte length followed
//! by UTF-8 bytes. Every command is acknowledged by a status response
//! `<command id><result><description>` where `0x00` means success.

#![deny(missing_docs)]

pub mod co_simulation;

pub use co_simulation::CoSimulation;

use std::io::{BufReader, Read, Write};
use std::net::TcpStream;

const CMD_GET_VERSION: u8 = 0x00;
const CMD_SIM_STEP: u8 = 0x02;
const CMD_CLOSE: u8 = 0x7f;
const CMD_GET_VEHICLE_VARIABLE: u8 = 0xa4;
const CMD_SET_VEHICLE_VARIABLE: u8 = 0xc4;
const RESPONSE_GET_VEHICLE_VARIABLE: u8 = 0xb4;

const VAR_ID_LIST: u8 = 0x00;
const VAR_POSITION: u8 = 0x42;
const VAR_SPEED: u8 = 0x40;

const TYPE_STRING_LIST: u8 = 0x0e;
const TYPE_POSITION_2D: u8 = 0x01;
const TYPE_DOUBLE: u8 = 0x0b;

/// TraCI protocol or connection failure.
#[derive(Debug, thiserror::Error)]
pub enum TraciError {
    /// The connection could not be established or was closed.
    #[error("TraCI connection error: {0}")]
    Io(#[from] std::io::Error),
    /// The response did not match the command that was sent.
    #[error("TraCI protocol error: {0}")]
    Protocol(String),
    /// The requested command is not implemented by the server.
    #[error("TraCI command not implemented: {0}")]
    NotImplemented(String),
    /// The server rejected the command.
    #[error("TraCI command failed: {0}")]
    Command(String),
    /// The server sent a value with an unexpected type.
    #[error("TraCI unexpected value type {0:#x} for {1}")]
    UnexpectedValue(u8, &'static str),
    /// A caller supplied a value outside this adapter's supported range.
    #[error("invalid TraCI argument: {0}")]
    InvalidArgument(String),
}

/// A connected TraCI client bound to one SUMO process.
pub struct TraciClient {
    reader: BufReader<TcpStream>,
    stream: TcpStream,
}

impl TraciClient {
    /// Connects to a SUMO process listening on `host:port`.
    pub fn connect(host: &str, port: u16) -> Result<Self, TraciError> {
        let stream = TcpStream::connect((host, port))?;
        stream.set_nodelay(true)?;
        let reader = BufReader::new(stream.try_clone()?);
        Ok(Self { reader, stream })
    }

    /// Reads the TraCI API version and a human-readable SUMO version string.
    pub fn get_version(&mut self) -> Result<(u32, String), TraciError> {
        self.send_command(CMD_GET_VERSION, &[])?;
        let mut message = self.read_response()?;
        self.read_status(&mut message, CMD_GET_VERSION)?;
        message.read_command_length()?;
        let response = message.read_u8()?;
        if response != CMD_GET_VERSION {
            return Err(TraciError::Protocol(format!(
                "expected version response {CMD_GET_VERSION:#x}, got {response:#x}"
            )));
        }
        let api_version = message.read_u32()?;
        let name = message.read_string()?;
        Ok((api_version, name))
    }

    /// Advances SUMO by one simulation step.
    pub fn simulation_step(&mut self) -> Result<(), TraciError> {
        let mut payload = Vec::with_capacity(8);
        payload.extend_from_slice(&0.0_f64.to_be_bytes());
        self.send_command(CMD_SIM_STEP, &payload)?;
        let mut message = self.read_response()?;
        self.read_status(&mut message, CMD_SIM_STEP)?;
        let _subscription_count = message.read_u32()?;
        Ok(())
    }

    /// Lists the vehicle ids currently present in the simulation.
    pub fn vehicle_ids(&mut self) -> Result<Vec<String>, TraciError> {
        let mut message = self.get_vehicle_variable(VAR_ID_LIST, "")?;
        let value_type = message.read_u8()?;
        if value_type != TYPE_STRING_LIST {
            return Err(TraciError::UnexpectedValue(value_type, "vehicle id list"));
        }
        let count = message.read_u32()?;
        let mut ids = Vec::with_capacity(count as usize);
        for _ in 0..count {
            ids.push(message.read_string()?);
        }
        Ok(ids)
    }

    /// Reads a vehicle's `(x, y)` position in SUMO's network coordinates.
    pub fn vehicle_position(&mut self, vehicle_id: &str) -> Result<[f64; 2], TraciError> {
        let mut message = self.get_vehicle_variable(VAR_POSITION, vehicle_id)?;
        let value_type = message.read_u8()?;
        if value_type != TYPE_POSITION_2D {
            return Err(TraciError::UnexpectedValue(value_type, "vehicle position"));
        }
        let x = message.read_f64()?;
        let y = message.read_f64()?;
        Ok([x, y])
    }

    /// Reads a vehicle's position in the RNE Y-up frame `[x, 0, -y]`.
    ///
    /// SUMO uses `x` = east and `y` = north, matching the coordinate frame
    /// [`crate`] shares with `rne_sumo`: east maps to RNE X and north to
    /// negative Z, with Y up.
    pub fn vehicle_position_rne(&mut self, vehicle_id: &str) -> Result<[f64; 3], TraciError> {
        let [x, y] = self.vehicle_position(vehicle_id)?;
        Ok([x, 0.0, -y])
    }

    /// Explicitly asks SUMO to set one vehicle's target speed in m/s.
    ///
    /// The command is sent only when this method is called. SUMO remains the
    /// motion authority and applies its configured safety and speed-mode rules;
    /// RNE's traffic runtime does not integrate the mirrored actor. Passing
    /// `-1.0` restores SUMO's original vehicle-type speed behavior, as defined
    /// by the TraCI vehicle state API.
    pub fn set_vehicle_speed_m_s(
        &mut self,
        vehicle_id: &str,
        speed_m_s: f64,
    ) -> Result<(), TraciError> {
        if !speed_m_s.is_finite() || speed_m_s < -1.0 {
            return Err(TraciError::InvalidArgument(format!(
                "vehicle speed must be finite and at least -1.0 m/s, got {speed_m_s}"
            )));
        }
        let mut payload = Vec::with_capacity(1 + 4 + vehicle_id.len() + 1 + 8);
        payload.push(VAR_SPEED);
        payload.extend_from_slice(&(vehicle_id.len() as u32).to_be_bytes());
        payload.extend_from_slice(vehicle_id.as_bytes());
        payload.push(TYPE_DOUBLE);
        payload.extend_from_slice(&speed_m_s.to_be_bytes());
        self.send_command(CMD_SET_VEHICLE_VARIABLE, &payload)?;
        let mut message = self.read_response()?;
        self.read_status(&mut message, CMD_SET_VEHICLE_VARIABLE)
    }

    /// Tells SUMO to close the connection and shut down.
    pub fn close(&mut self) -> Result<(), TraciError> {
        self.send_command(CMD_CLOSE, &[])?;
        let mut message = self.read_response()?;
        self.read_status(&mut message, CMD_CLOSE)?;
        Ok(())
    }

    fn get_vehicle_variable(
        &mut self,
        variable: u8,
        vehicle_id: &str,
    ) -> Result<TraciMessage, TraciError> {
        let mut payload = Vec::with_capacity(5 + vehicle_id.len());
        payload.push(variable);
        payload.extend_from_slice(&(vehicle_id.len() as u32).to_be_bytes());
        payload.extend_from_slice(vehicle_id.as_bytes());
        self.send_command(CMD_GET_VEHICLE_VARIABLE, &payload)?;
        let mut message = self.read_response()?;
        self.read_status(&mut message, CMD_GET_VEHICLE_VARIABLE)?;
        message.read_command_length()?;
        let response = message.read_u8()?;
        if response != RESPONSE_GET_VEHICLE_VARIABLE {
            return Err(TraciError::Protocol(format!(
                "expected vehicle response {RESPONSE_GET_VEHICLE_VARIABLE:#x}, got {response:#x}"
            )));
        }
        let _variable = message.read_u8()?;
        let _object_id = message.read_string()?;
        Ok(message)
    }

    fn send_command(&mut self, command_id: u8, payload: &[u8]) -> Result<(), TraciError> {
        let mut command = Vec::with_capacity(7 + payload.len());
        // The command length counts the length byte itself, the command id,
        // and the payload (SUMO's reference `traci` uses `payload + 2`).
        let command_length = 2 + payload.len();
        if command_length <= 0xff {
            command.push(command_length as u8);
            command.push(command_id);
        } else {
            command.push(0x00);
            command.extend_from_slice(&((command_length + 4) as u32).to_be_bytes());
            command.push(command_id);
        }
        command.extend_from_slice(payload);
        let mut message = Vec::with_capacity(4 + command.len());
        message.extend_from_slice(&((command.len() as u32) + 4).to_be_bytes());
        message.extend_from_slice(&command);
        self.stream.write_all(&message)?;
        Ok(())
    }

    fn read_response(&mut self) -> Result<TraciMessage, TraciError> {
        let mut length_bytes = [0_u8; 4];
        self.reader.read_exact(&mut length_bytes)?;
        let length = u32::from_be_bytes(length_bytes) as usize;
        if length < 4 {
            return Err(TraciError::Protocol(format!(
                "invalid TraCI message length {length}"
            )));
        }
        let mut body = vec![0_u8; length - 4];
        self.reader.read_exact(&mut body)?;
        Ok(TraciMessage { body, position: 0 })
    }

    fn read_status(&mut self, message: &mut TraciMessage, expected: u8) -> Result<(), TraciError> {
        message.read_command_length()?;
        let command = message.read_u8()?;
        let result = message.read_u8()?;
        let description = message.read_string()?;
        if command != expected {
            return Err(TraciError::Protocol(format!(
                "expected status for command {expected:#x}, got {command:#x}"
            )));
        }
        match result {
            0x00 => Ok(()),
            0x01 => Err(TraciError::NotImplemented(description)),
            _ => Err(TraciError::Command(description)),
        }
    }
}

/// A received TraCI message being decoded.
struct TraciMessage {
    body: Vec<u8>,
    position: usize,
}

impl TraciMessage {
    fn read_u8(&mut self) -> Result<u8, TraciError> {
        let value = *self
            .body
            .get(self.position)
            .ok_or_else(|| TraciError::Protocol("truncated message".into()))?;
        self.position += 1;
        Ok(value)
    }

    fn read_u32(&mut self) -> Result<u32, TraciError> {
        let bytes = self
            .body
            .get(self.position..self.position + 4)
            .ok_or_else(|| TraciError::Protocol("truncated message".into()))?;
        self.position += 4;
        Ok(u32::from_be_bytes(bytes.try_into().expect("four bytes")))
    }

    fn read_f64(&mut self) -> Result<f64, TraciError> {
        let bytes = self
            .body
            .get(self.position..self.position + 8)
            .ok_or_else(|| TraciError::Protocol("truncated message".into()))?;
        self.position += 8;
        Ok(f64::from_be_bytes(bytes.try_into().expect("eight bytes")))
    }

    fn read_string(&mut self) -> Result<String, TraciError> {
        let length = self.read_u32()? as usize;
        let bytes = self
            .body
            .get(self.position..self.position + length)
            .ok_or_else(|| TraciError::Protocol("truncated message".into()))?;
        self.position += length;
        String::from_utf8(bytes.to_vec())
            .map_err(|_| TraciError::Protocol("response string is not UTF-8".into()))
    }

    /// Skips a command's 1-byte or 4-byte extended length prefix.
    fn read_command_length(&mut self) -> Result<(), TraciError> {
        let first = self.read_u8()?;
        if first == 0x00 {
            self.read_u32()?;
        }
        Ok(())
    }
}
