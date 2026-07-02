//! Sensor framework for Robot Native Engine.

#![deny(missing_docs)]

pub mod camera;
pub mod components;
pub mod imu;
pub mod lidar;
pub mod noise;
pub mod systems;
pub mod wheel_encoder;

pub use camera::{sample_camera, sample_camera_rgbd, CameraRgbdSample, CameraSpec};
pub use components::{Sensor, SensorKind, SensorState};
pub use imu::{sample_imu, sample_imu_keyed, ImuSpec};
pub use lidar::{sample_lidar, LidarSpec};
pub use noise::{NoiseModel, SensorNoiseKey};
pub use systems::{sample_sensors, SensorSampleContext, SensorSampler, CAMERA_DEPTH_STREAM_OFFSET};
pub use wheel_encoder::{sample_wheel_encoder, WheelEncoderSpec};
