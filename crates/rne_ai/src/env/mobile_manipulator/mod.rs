//! Headless mobile manipulator simulation (fixed-base arm phase).

mod drive;
mod sim;

pub use drive::{
    mm_mobile_twist_to_wheel_velocities, wheel_command_to_motor_rad_s, MM_MOBILE_TRACK_WIDTH_M,
    MM_MOBILE_WHEEL_JOINT_SIGN, MM_MOBILE_WHEEL_RADIUS_M,
};
pub use sim::MobileManipulatorSim;
