//! Official SSL simulation-protocol default UDP ports.

/// Simulation control port (`SimulatorCommand` / `SimulatorResponse`).
pub const SSL_SIM_CONTROL_PORT: u16 = 10_300;
/// Blue-team robot control port (`RobotControl` / `RobotControlResponse`).
pub const SSL_ROBOT_CONTROL_BLUE_PORT: u16 = 10_301;
/// Yellow-team robot control port (`RobotControl` / `RobotControlResponse`).
pub const SSL_ROBOT_CONTROL_YELLOW_PORT: u16 = 10_302;

/// Port triple used by an SSL simulator endpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SslSimulationPorts {
    /// Port 10300 by default.
    pub control: u16,
    /// Port 10301 by default.
    pub blue: u16,
    /// Port 10302 by default.
    pub yellow: u16,
}

impl Default for SslSimulationPorts {
    fn default() -> Self {
        Self {
            control: SSL_SIM_CONTROL_PORT,
            blue: SSL_ROBOT_CONTROL_BLUE_PORT,
            yellow: SSL_ROBOT_CONTROL_YELLOW_PORT,
        }
    }
}

impl SslSimulationPorts {
    /// Bind helpers that ask the OS for free ports (tests / smoke).
    #[must_use]
    pub const fn ephemeral() -> Self {
        Self {
            control: 0,
            blue: 0,
            yellow: 0,
        }
    }
}
