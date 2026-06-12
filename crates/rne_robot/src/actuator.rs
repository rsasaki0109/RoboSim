//! Actuator targets, modes, and limits.

use serde::{Deserialize, Serialize};

/// Actuator control mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlMode {
    /// Velocity control.
    #[default]
    Velocity,
    /// Position control.
    Position,
    /// Effort/torque control.
    Effort,
}

/// Commanded actuator setpoint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ActuatorTarget {
    /// Target velocity in radians per second.
    pub velocity_rad_s: f64,
    /// Target position in radians.
    pub position_rad: f64,
    /// Target effort in newton-meters.
    pub effort_nm: f64,
}

/// Actuator saturation and safety limits.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorLimits {
    /// Minimum velocity in radians per second.
    pub min_velocity_rad_s: f64,
    /// Maximum velocity in radians per second.
    pub max_velocity_rad_s: f64,
    /// Minimum position in radians.
    pub min_position_rad: f64,
    /// Maximum position in radians.
    pub max_position_rad: f64,
    /// Maximum effort in newton-meters.
    pub max_effort_nm: f64,
}

impl Default for ActuatorLimits {
    fn default() -> Self {
        Self {
            min_velocity_rad_s: -20.0,
            max_velocity_rad_s: 20.0,
            min_position_rad: -f64::INFINITY,
            max_position_rad: f64::INFINITY,
            max_effort_nm: 100.0,
        }
    }
}

impl ActuatorLimits {
    /// Clamps a velocity command to actuator limits.
    pub fn clamp_velocity(&self, velocity_rad_s: f64) -> f64 {
        velocity_rad_s.clamp(self.min_velocity_rad_s, self.max_velocity_rad_s)
    }

    /// Clamps a position command to actuator limits.
    pub fn clamp_position(&self, position_rad: f64) -> f64 {
        position_rad.clamp(self.min_position_rad, self.max_position_rad)
    }
}
