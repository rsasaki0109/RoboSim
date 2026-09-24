//! Mobile-base drive actuators with explicit limits and failure behavior.
//!
//! A [`MobileBase`] couples a drive geometry ([`DriveKind`]) with [`DriveLimits`]
//! and a rate limiter. Every command is validated, clamped to the configured
//! velocity and acceleration limits, and reported through [`DriveFault`]:
//!
//! * [`DriveFault::None`] — the command was applied without clamping.
//! * [`DriveFault::Saturated`] — the command was clamped to a velocity,
//!   steering, or acceleration limit.
//! * [`DriveFault::Disabled`] — the base is disabled and emits a hard-stop
//!   zero command.
//!
//! Non-finite commands or non-positive time steps are rejected with a
//! [`DriveError`]; a disabled or saturated base never produces a non-finite
//! wheel setpoint, so a fault always degrades to a safe stop.

use crate::control::VelocityCommand2d;
use serde::{Deserialize, Serialize};

/// Velocity and acceleration limits for a mobile base.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DriveLimits {
    /// Maximum absolute forward velocity in meters per second.
    pub max_linear_m_s: f64,
    /// Maximum absolute yaw rate in radians per second.
    pub max_angular_rad_s: f64,
    /// Maximum linear acceleration in meters per second squared.
    pub max_linear_accel_m_s2: f64,
    /// Maximum angular acceleration in radians per second squared.
    pub max_angular_accel_rad_s2: f64,
}

impl Default for DriveLimits {
    fn default() -> Self {
        Self {
            max_linear_m_s: 1.0,
            max_angular_rad_s: 2.0,
            max_linear_accel_m_s2: 2.0,
            max_angular_accel_rad_s2: 4.0,
        }
    }
}

impl DriveLimits {
    /// Whether every limit is finite and positive.
    pub fn is_valid(&self) -> bool {
        self.max_linear_m_s.is_finite()
            && self.max_linear_m_s > 0.0
            && self.max_angular_rad_s.is_finite()
            && self.max_angular_rad_s > 0.0
            && self.max_linear_accel_m_s2.is_finite()
            && self.max_linear_accel_m_s2 > 0.0
            && self.max_angular_accel_rad_s2.is_finite()
            && self.max_angular_accel_rad_s2 > 0.0
    }
}

/// Differential-drive geometry.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DifferentialDrive {
    /// Drive wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Distance between the left and right wheel contact patches in meters.
    pub track_width_m: f64,
}

impl DifferentialDrive {
    /// Converts a body command into `(left, right)` wheel angular rates.
    pub(crate) fn wheel_speeds(&self, command: VelocityCommand2d) -> WheelSpeeds {
        let half_track = 0.5 * self.track_width_m;
        let left = (command.linear_m_s - command.angular_rad_s * half_track) / self.wheel_radius_m;
        let right = (command.linear_m_s + command.angular_rad_s * half_track) / self.wheel_radius_m;
        WheelSpeeds {
            left_rad_s: left,
            right_rad_s: right,
        }
    }

    /// Recovers the body command from `(left, right)` wheel angular rates.
    pub fn body_command(&self, speeds: WheelSpeeds) -> VelocityCommand2d {
        let linear = self.wheel_radius_m * 0.5 * (speeds.left_rad_s + speeds.right_rad_s);
        let angular =
            self.wheel_radius_m * (speeds.right_rad_s - speeds.left_rad_s) / self.track_width_m;
        VelocityCommand2d::new(linear, angular)
    }

    fn is_valid(&self) -> bool {
        self.wheel_radius_m.is_finite()
            && self.wheel_radius_m > 0.0
            && self.track_width_m.is_finite()
            && self.track_width_m > 0.0
    }
}

/// Angular rates for the two wheels of a differential drive.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WheelSpeeds {
    /// Left wheel angular rate in radians per second.
    pub left_rad_s: f64,
    /// Right wheel angular rate in radians per second.
    pub right_rad_s: f64,
}

/// Ackermann (bicycle) steering geometry.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AckermannDrive {
    /// Distance between the front and rear axles in meters.
    pub wheelbase_m: f64,
    /// Drive wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Maximum absolute steering angle in radians.
    pub max_steering_rad: f64,
}

impl AckermannDrive {
    fn is_valid(&self) -> bool {
        self.wheelbase_m.is_finite()
            && self.wheelbase_m > 0.0
            && self.wheel_radius_m.is_finite()
            && self.wheel_radius_m > 0.0
            && self.max_steering_rad.is_finite()
            && self.max_steering_rad > 0.0
            && self.max_steering_rad < std::f64::consts::FRAC_PI_2
    }
}

/// Mecanum (four-wheel omnidirectional) geometry.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MecanumDrive {
    /// Drive wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Half the distance between the front and rear axles in meters.
    pub half_length_m: f64,
    /// Half the distance between the left and right wheels in meters.
    pub half_width_m: f64,
}

impl MecanumDrive {
    /// Converts a body command into `(front-left, front-right, rear-left,
    /// rear-right)` wheel angular rates.
    pub(crate) fn wheel_speeds(&self, command: VelocityCommand2d) -> [f64; 4] {
        let lever = self.half_length_m + self.half_width_m;
        let rotation = command.angular_rad_s * lever;
        let linear = command.linear_m_s;
        let r = self.wheel_radius_m;
        [
            (linear - rotation) / r,
            (linear + rotation) / r,
            (linear + rotation) / r,
            (linear - rotation) / r,
        ]
    }

    fn is_valid(&self) -> bool {
        self.wheel_radius_m.is_finite()
            && self.wheel_radius_m > 0.0
            && self.half_length_m.is_finite()
            && self.half_length_m >= 0.0
            && self.half_width_m.is_finite()
            && self.half_width_m >= 0.0
            && (self.half_length_m + self.half_width_m) > 0.0
    }
}

/// Drive geometry variants supported by a [`MobileBase`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DriveKind {
    /// Differential drive.
    Differential(DifferentialDrive),
    /// Ackermann steering.
    Ackermann(AckermannDrive),
    /// Four-wheel mecanum.
    Mecanum(MecanumDrive),
}

impl DriveKind {
    /// The zero (hard-stop) actuation for this drive.
    pub(crate) fn zero_actuation(&self) -> DriveActuation {
        match self {
            Self::Differential(_) => DriveActuation::Differential {
                left_rad_s: 0.0,
                right_rad_s: 0.0,
            },
            Self::Ackermann(_) => DriveActuation::Ackermann {
                wheel_rad_s: 0.0,
                steer_rad: 0.0,
            },
            Self::Mecanum(_) => DriveActuation::Mecanum {
                fl_rad_s: 0.0,
                fr_rad_s: 0.0,
                rl_rad_s: 0.0,
                rr_rad_s: 0.0,
            },
        }
    }

    fn is_valid(&self) -> bool {
        match self {
            Self::Differential(drive) => drive.is_valid(),
            Self::Ackermann(drive) => drive.is_valid(),
            Self::Mecanum(drive) => drive.is_valid(),
        }
    }
}

/// Wheel and steering setpoints produced by a drive.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DriveActuation {
    /// Differential wheel rates.
    Differential {
        /// Left wheel angular rate in radians per second.
        left_rad_s: f64,
        /// Right wheel angular rate in radians per second.
        right_rad_s: f64,
    },
    /// Ackermann wheel rate and steering angle.
    Ackermann {
        /// Drive wheel angular rate in radians per second.
        wheel_rad_s: f64,
        /// Front steering angle in radians.
        steer_rad: f64,
    },
    /// Mecanum wheel rates.
    Mecanum {
        /// Front-left wheel angular rate in radians per second.
        fl_rad_s: f64,
        /// Front-right wheel angular rate in radians per second.
        fr_rad_s: f64,
        /// Rear-left wheel angular rate in radians per second.
        rl_rad_s: f64,
        /// Rear-right wheel angular rate in radians per second.
        rr_rad_s: f64,
    },
}

/// Fault state reported after each actuator command.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DriveFault {
    /// No clamping occurred.
    #[default]
    None,
    /// The command was clamped to a velocity, steering, or acceleration limit.
    Saturated,
    /// The base is disabled; a hard-stop zero command was emitted.
    Disabled,
}

/// Result of a single actuator command.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DriveOutput {
    /// The command actually applied after limiting.
    pub command: VelocityCommand2d,
    /// Wheel and steering setpoints for [`DriveOutput::command`].
    pub actuation: DriveActuation,
    /// Fault reported for this command.
    pub fault: DriveFault,
}

/// Errors raised by drive actuation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DriveError {
    /// The drive geometry is degenerate or non-finite.
    #[error("invalid drive geometry")]
    InvalidGeometry,
    /// The drive or acceleration limits are degenerate or non-finite.
    #[error("invalid drive limits")]
    InvalidLimits,
    /// The requested command contained a non-finite component.
    #[error("non-finite velocity command")]
    NonFiniteCommand,
    /// The time step was not finite and strictly positive.
    #[error("time step must be finite and positive")]
    NonPositiveTime,
}

/// A mobile base with drive geometry, velocity/acceleration limits, and fault
/// handling.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MobileBase {
    /// Drive geometry.
    pub drive: DriveKind,
    /// Velocity and acceleration limits.
    pub limits: DriveLimits,
    applied: VelocityCommand2d,
    disabled: bool,
}

impl MobileBase {
    /// Creates a base after validating its geometry and limits.
    pub fn new(drive: DriveKind, limits: DriveLimits) -> Result<Self, DriveError> {
        if !limits.is_valid() {
            return Err(DriveError::InvalidLimits);
        }
        if !drive.is_valid() {
            return Err(DriveError::InvalidGeometry);
        }
        Ok(Self {
            drive,
            limits,
            applied: VelocityCommand2d::ZERO,
            disabled: false,
        })
    }

    /// Enables or disables the base.
    ///
    /// Disabling resets the applied command to zero so re-enabling restarts the
    /// acceleration ramp from rest.
    pub fn set_disabled(&mut self, disabled: bool) {
        self.disabled = disabled;
        if disabled {
            self.applied = VelocityCommand2d::ZERO;
        }
    }

    /// Applies a body command over `dt_s`, clamping to velocity, acceleration,
    /// and steering limits.
    ///
    /// The returned [`DriveOutput`] always contains finite setpoints. A disabled
    /// base emits [`DriveFault::Disabled`] and a zero command.
    pub fn command(
        &mut self,
        desired: VelocityCommand2d,
        dt_s: f64,
    ) -> Result<DriveOutput, DriveError> {
        if !desired.linear_m_s.is_finite() || !desired.angular_rad_s.is_finite() {
            return Err(DriveError::NonFiniteCommand);
        }
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(DriveError::NonPositiveTime);
        }

        if self.disabled {
            return Ok(DriveOutput {
                command: VelocityCommand2d::ZERO,
                actuation: self.drive.zero_actuation(),
                fault: DriveFault::Disabled,
            });
        }

        let mut fault = DriveFault::None;
        let mut target = desired;
        if target.linear_m_s.abs() > self.limits.max_linear_m_s {
            target.linear_m_s = target
                .linear_m_s
                .clamp(-self.limits.max_linear_m_s, self.limits.max_linear_m_s);
            fault = DriveFault::Saturated;
        }
        if target.angular_rad_s.abs() > self.limits.max_angular_rad_s {
            target.angular_rad_s = target.angular_rad_s.clamp(
                -self.limits.max_angular_rad_s,
                self.limits.max_angular_rad_s,
            );
            fault = DriveFault::Saturated;
        }

        let max_linear_step = self.limits.max_linear_accel_m_s2 * dt_s;
        let max_angular_step = self.limits.max_angular_accel_rad_s2 * dt_s;
        let linear_step =
            (target.linear_m_s - self.applied.linear_m_s).clamp(-max_linear_step, max_linear_step);
        let angular_step = (target.angular_rad_s - self.applied.angular_rad_s)
            .clamp(-max_angular_step, max_angular_step);
        if (linear_step - (target.linear_m_s - self.applied.linear_m_s)).abs() > 0.0
            || (angular_step - (target.angular_rad_s - self.applied.angular_rad_s)).abs() > 0.0
        {
            fault = DriveFault::Saturated;
        }

        self.applied = VelocityCommand2d::new(
            self.applied.linear_m_s + linear_step,
            self.applied.angular_rad_s + angular_step,
        );

        let (actuation, steer_saturated) = self.actuate(self.applied);
        if steer_saturated {
            fault = DriveFault::Saturated;
        }

        Ok(DriveOutput {
            command: self.applied,
            actuation,
            fault,
        })
    }

    fn actuate(&self, command: VelocityCommand2d) -> (DriveActuation, bool) {
        match self.drive {
            DriveKind::Differential(drive) => {
                let speeds = drive.wheel_speeds(command);
                (
                    DriveActuation::Differential {
                        left_rad_s: speeds.left_rad_s,
                        right_rad_s: speeds.right_rad_s,
                    },
                    false,
                )
            }
            DriveKind::Ackermann(drive) => {
                let steer = if command.linear_m_s.abs() > 1.0e-6 {
                    (drive.wheelbase_m * command.angular_rad_s / command.linear_m_s).atan()
                } else {
                    0.0
                };
                let clamped = steer.clamp(-drive.max_steering_rad, drive.max_steering_rad);
                let saturated = (clamped - steer).abs() > 1.0e-12;
                (
                    DriveActuation::Ackermann {
                        wheel_rad_s: command.linear_m_s / drive.wheel_radius_m,
                        steer_rad: clamped,
                    },
                    saturated,
                )
            }
            DriveKind::Mecanum(drive) => {
                let speeds = drive.wheel_speeds(command);
                (
                    DriveActuation::Mecanum {
                        fl_rad_s: speeds[0],
                        fr_rad_s: speeds[1],
                        rl_rad_s: speeds[2],
                        rr_rad_s: speeds[3],
                    },
                    false,
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn base(drive: DriveKind) -> MobileBase {
        MobileBase::new(drive, DriveLimits::default()).unwrap()
    }

    #[test]
    fn differential_straight_is_equal_and_spin_is_opposite() {
        let drive = DifferentialDrive {
            wheel_radius_m: 0.1,
            track_width_m: 0.4,
        };
        let straight = drive.wheel_speeds(VelocityCommand2d::new(1.0, 0.0));
        assert_relative_eq!(straight.left_rad_s, 10.0, epsilon = 1e-12);
        assert_relative_eq!(straight.right_rad_s, 10.0, epsilon = 1e-12);

        let spin = drive.wheel_speeds(VelocityCommand2d::new(0.0, 1.0));
        assert_relative_eq!(spin.left_rad_s, -2.0, epsilon = 1e-12);
        assert_relative_eq!(spin.right_rad_s, 2.0, epsilon = 1e-12);

        let round_trip = drive.body_command(straight);
        assert_relative_eq!(round_trip.linear_m_s, 1.0, epsilon = 1e-12);
        assert_relative_eq!(round_trip.angular_rad_s, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn limit_clamps_velocity_and_reports_saturation() {
        let mut base = base(DriveKind::Differential(DifferentialDrive {
            wheel_radius_m: 0.1,
            track_width_m: 0.4,
        }));
        let limits = DriveLimits {
            max_linear_m_s: 0.5,
            max_linear_accel_m_s2: 100.0,
            ..DriveLimits::default()
        };
        let mut base2 = MobileBase::new(base.drive, limits).unwrap();
        let out = base2
            .command(VelocityCommand2d::new(5.0, 0.0), 0.1)
            .unwrap();
        assert_eq!(out.fault, DriveFault::Saturated);
        assert_relative_eq!(out.command.linear_m_s, 0.5, epsilon = 1e-12);

        let slow = base
            .command(VelocityCommand2d::new(0.1, 0.0), 0.05)
            .unwrap();
        assert_eq!(slow.fault, DriveFault::None);
        assert!(slow.command.linear_m_s > 0.0);
    }

    #[test]
    fn ackermann_saturates_steering() {
        let base = base(DriveKind::Ackermann(AckermannDrive {
            wheelbase_m: 2.0,
            wheel_radius_m: 0.3,
            max_steering_rad: 0.3,
        }));
        let limits = DriveLimits {
            max_angular_rad_s: 10.0,
            max_angular_accel_rad_s2: 1000.0,
            ..DriveLimits::default()
        };
        let mut base = MobileBase::new(base.drive, limits).unwrap();
        let out = base
            .command(VelocityCommand2d::new(1.0, 10.0), 1.0)
            .unwrap();
        assert_eq!(out.fault, DriveFault::Saturated);
        match out.actuation {
            DriveActuation::Ackermann { steer_rad, .. } => {
                assert_relative_eq!(steer_rad, 0.3, epsilon = 1e-12);
            }
            other => panic!("unexpected actuation {other:?}"),
        }
    }

    #[test]
    fn mecanum_rotation_spins_opposite_corners() {
        let drive = MecanumDrive {
            wheel_radius_m: 0.1,
            half_length_m: 0.2,
            half_width_m: 0.15,
        };
        let speeds = drive.wheel_speeds(VelocityCommand2d::new(0.0, 1.0));
        assert!(speeds[0] < 0.0);
        assert!(speeds[1] > 0.0);
        assert!(speeds[2] > 0.0);
        assert!(speeds[3] < 0.0);
    }

    #[test]
    fn disabled_base_hard_stops() {
        let mut base = base(DriveKind::Differential(DifferentialDrive {
            wheel_radius_m: 0.1,
            track_width_m: 0.4,
        }));
        base.command(VelocityCommand2d::new(0.5, 0.0), 1.0).unwrap();
        base.set_disabled(true);
        let out = base.command(VelocityCommand2d::new(0.5, 0.0), 1.0).unwrap();
        assert_eq!(out.fault, DriveFault::Disabled);
        assert_eq!(out.command, VelocityCommand2d::ZERO);
        assert_eq!(out.actuation, base.drive.zero_actuation());
    }

    #[test]
    fn rejects_non_finite_command_and_bad_time() {
        let mut base = base(DriveKind::Differential(DifferentialDrive {
            wheel_radius_m: 0.1,
            track_width_m: 0.4,
        }));
        assert_eq!(
            base.command(VelocityCommand2d::new(f64::NAN, 0.0), 0.1),
            Err(DriveError::NonFiniteCommand)
        );
        assert_eq!(
            base.command(VelocityCommand2d::ZERO, 0.0),
            Err(DriveError::NonPositiveTime)
        );
    }

    #[test]
    fn rejects_degenerate_geometry_and_limits() {
        let bad_drive = DriveKind::Differential(DifferentialDrive {
            wheel_radius_m: 0.0,
            track_width_m: 0.4,
        });
        assert_eq!(
            MobileBase::new(bad_drive, DriveLimits::default()),
            Err(DriveError::InvalidGeometry)
        );
        let bad_limits = DriveLimits {
            max_linear_m_s: 0.0,
            ..DriveLimits::default()
        };
        assert_eq!(
            MobileBase::new(
                DriveKind::Differential(DifferentialDrive {
                    wheel_radius_m: 0.1,
                    track_width_m: 0.4,
                }),
                bad_limits
            ),
            Err(DriveError::InvalidLimits)
        );
    }
}
