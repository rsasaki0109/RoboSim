//! Spatial math and explicit units for Robot Native Engine.

#![deny(missing_docs)]

pub mod transform;
pub mod units;

pub use glam::{DMat4, DQuat, DVec3};
pub use transform::{y_up_euler_rad, yaw_rad, Pose3, Transform3, Velocity3};
pub use units::{Hertz, Meters, Radians, Seconds};

/// Three-dimensional vector using double precision.
pub type Vec3 = glam::DVec3;

/// Unit quaternion using double precision.
pub type Quat = glam::DQuat;

/// Four-by-four matrix using double precision.
pub type Mat4 = glam::DMat4;
