//! Sensor framework for Robot Native Engine.

#![deny(missing_docs)]

pub mod camera;
pub mod components;
pub mod imu;
pub mod lidar;
pub mod noise;
pub mod systems;
pub mod wheel_encoder;

pub use camera::{
    sample_camera, sample_camera_rgbd, sample_camera_rgbd_keyed, sample_camera_rgbd_swept,
    CameraDistortion, CameraRgbdSample, CameraSpec, CameraSweep,
};
pub use components::{
    ImuFeedbackFault, ImuFeedbackSensor, ImuFeedbackSensorState, ImuMount, ImuState,
    JointFeedbackChannelSpec, JointFeedbackFault, JointFeedbackSensor, JointFeedbackSensorState,
    LidarMaterial, Sensor, SensorKind, SensorState,
};
pub use imu::{
    sample_imu, sample_imu_keyed, sample_imu_stateful, sample_imu_stateful_diagnostic,
    ImuAxisErrors, ImuDiagnosticSample, ImuSampleError, ImuSpec, ImuTruth, GRAVITY_M_S2,
};
pub use lidar::{
    sample_lidar, sample_lidar_at_entity, sample_lidar_at_entity_keyed, sample_lidar_keyed,
    sample_lidar_swept, LidarAtmosphere, LidarDomainRandomization, LidarFailureBehavior, LidarSpec,
    LidarSweep, RANGE_REFERENCE_M,
};
pub use noise::{NoiseModel, SensorNoiseKey};
pub use systems::{
    sample_imu_feedback_sensors, sample_joint_feedback_sensors, sample_sensors, ImuFeedbackError,
    JointFeedbackError, SensorSampleContext, SensorSampler, CAMERA_DEPTH_STREAM_OFFSET,
};
pub use wheel_encoder::{sample_wheel_encoder, WheelEncoderSpec};
