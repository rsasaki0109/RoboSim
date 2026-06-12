//! Sensor framework for Robot Native Engine.

#![deny(missing_docs)]

pub mod camera;
pub mod components;
pub mod imu;
pub mod lidar;
pub mod noise;
pub mod systems;
pub mod wheel_encoder;

pub use camera::{sample_camera, CameraSpec};
pub use components::{Sensor, SensorKind, SensorState};
pub use imu::{sample_imu, ImuSpec};
pub use lidar::{sample_lidar, LidarSpec};
pub use noise::NoiseModel;
pub use systems::{sample_sensors, SensorSampleContext, SensorSampler};
pub use wheel_encoder::{sample_wheel_encoder, WheelEncoderSpec};
