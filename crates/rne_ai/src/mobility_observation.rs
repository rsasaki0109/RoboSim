//! Controller-visible mobility observations assembled only from DataBus frames.

use crate::DiffDriveObservation;
use rne_core::SimTime;
use rne_data::{
    DataBus, Frame, FramePayload, ImuSample, PointCloud, PoseSample, StreamId, WheelEncoderSample,
};
use thiserror::Error;

/// DataBus streams required to build one differential-drive actor observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DiffDriveActorStreams {
    /// Localization or estimator pose stream.
    pub localization: StreamId,
    /// Measured left-wheel encoder stream.
    pub left_wheel_encoder: StreamId,
    /// Measured right-wheel encoder stream.
    pub right_wheel_encoder: StreamId,
    /// IMU stream.
    pub imu: StreamId,
    /// LiDAR point-cloud stream.
    pub lidar: StreamId,
}

/// Timing and sequence metadata retained beside one actor-visible payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActorFrameMetadata {
    /// DataBus stream identifier.
    pub stream_id: StreamId,
    /// Source sequence number.
    pub sequence: u64,
    /// Simulation ticks at which the source captured the measurement.
    pub capture_ticks: u64,
    /// Simulation ticks at which the controller could first consume the frame.
    pub available_ticks: u64,
    /// Measurement age at the controller decision time in simulation ticks.
    pub age_ticks: u64,
}

/// Differential-drive actor observation with complete input-frame provenance.
///
/// The numerical observation contains measurements and task-provided goal data.
/// It contains no actuator command, ECS transform, rigid-body state, contact data,
/// or other privileged simulator truth.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveActorObservationFrame {
    /// Values exposed to the policy.
    pub observation: DiffDriveObservation,
    /// Controller decision time in simulation ticks.
    pub controller_time_ticks: u64,
    /// Localization input metadata.
    pub localization: ActorFrameMetadata,
    /// Left encoder input metadata.
    pub left_wheel_encoder: ActorFrameMetadata,
    /// Right encoder input metadata.
    pub right_wheel_encoder: ActorFrameMetadata,
    /// IMU input metadata.
    pub imu: ActorFrameMetadata,
    /// LiDAR input metadata.
    pub lidar: ActorFrameMetadata,
}

/// Schema domain used by [`stable_diff_drive_actor_observation_digest`].
pub const DIFF_DRIVE_ACTOR_OBSERVATION_DIGEST_SCHEMA: &str = "rne.diff_drive.actor_observation.v1";

/// Computes a stable FNV-1a digest of exactly one actor-visible observation frame.
///
/// The canonical byte stream includes every policy-visible value and the stream,
/// sequence, capture, availability, and age metadata for every required input. Integer
/// fields use little-endian `u64`; floating-point fields use their exact IEEE-754 bits;
/// optional values include an explicit presence byte. This digest deliberately excludes
/// privileged simulator state and is suitable for deterministic replay evidence.
pub fn stable_diff_drive_actor_observation_digest(frame: &DiffDriveActorObservationFrame) -> u64 {
    let mut bytes = Vec::with_capacity(320);
    bytes.extend_from_slice(DIFF_DRIVE_ACTOR_OBSERVATION_DIGEST_SCHEMA.as_bytes());
    bytes.push(0);
    push_u64(&mut bytes, frame.controller_time_ticks);
    push_observation(&mut bytes, &frame.observation);
    for metadata in [
        frame.localization,
        frame.left_wheel_encoder,
        frame.right_wheel_encoder,
        frame.imu,
        frame.lidar,
    ] {
        push_metadata(&mut bytes, metadata);
    }
    stable_fnv1a(&bytes)
}

/// Failure to assemble a complete actor observation at a decision time.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum DiffDriveActorObservationError {
    /// No frame of the required payload type was available by the decision time.
    #[error("no available {payload} frame on stream {stream_id}")]
    MissingAvailableFrame {
        /// Human-readable payload role.
        payload: &'static str,
        /// Required stream identifier.
        stream_id: u64,
    },
}

/// Builds a differential-drive policy input exclusively from available DataBus frames.
///
/// `goal_x_m` is task data rather than simulator state. Frames captured in the
/// future or still inside their declared latency window are invisible because
/// this function always uses [`DataBus::latest_available`].
pub fn diff_drive_actor_observation(
    bus: &impl DataBus,
    streams: DiffDriveActorStreams,
    controller_time: SimTime,
    goal_x_m: Option<f64>,
) -> Result<DiffDriveActorObservationFrame, DiffDriveActorObservationError> {
    let pose =
        required_frame::<PoseSample>(bus, streams.localization, controller_time, "localization")?;
    let left = required_frame::<WheelEncoderSample>(
        bus,
        streams.left_wheel_encoder,
        controller_time,
        "left wheel encoder",
    )?;
    let right = required_frame::<WheelEncoderSample>(
        bus,
        streams.right_wheel_encoder,
        controller_time,
        "right wheel encoder",
    )?;
    let imu = required_frame::<ImuSample>(bus, streams.imu, controller_time, "IMU")?;
    let lidar = required_frame::<PointCloud>(bus, streams.lidar, controller_time, "LiDAR")?;

    let controller_time_ticks = controller_time.ticks();
    Ok(DiffDriveActorObservationFrame {
        observation: DiffDriveObservation {
            base_x_m: pose.payload.position_m.x,
            base_y_m: pose.payload.position_m.y,
            base_z_m: pose.payload.position_m.z,
            base_yaw_rad: pose.payload.yaw_rad,
            left_wheel_velocity_rad_s: left.payload.velocity_rad_s,
            right_wheel_velocity_rad_s: right.payload.velocity_rad_s,
            imu_ay_m_s2: imu.payload.linear_acceleration_m_s2.y,
            lidar_points: lidar.payload.points_m.len(),
            goal_delta_x_m: goal_x_m.map(|goal| goal - pose.payload.position_m.x),
            peer_delta_x_m: None,
            peer_delta_z_m: None,
            peer_separation_m: None,
        },
        controller_time_ticks,
        localization: metadata(&pose, controller_time_ticks),
        left_wheel_encoder: metadata(&left, controller_time_ticks),
        right_wheel_encoder: metadata(&right, controller_time_ticks),
        imu: metadata(&imu, controller_time_ticks),
        lidar: metadata(&lidar, controller_time_ticks),
    })
}

fn required_frame<T: FramePayload>(
    bus: &impl DataBus,
    stream: StreamId,
    controller_time: SimTime,
    payload: &'static str,
) -> Result<Frame<T>, DiffDriveActorObservationError> {
    bus.latest_available::<T>(stream, controller_time).ok_or(
        DiffDriveActorObservationError::MissingAvailableFrame {
            payload,
            stream_id: stream.0,
        },
    )
}

fn metadata<T: FramePayload>(frame: &Frame<T>, controller_time_ticks: u64) -> ActorFrameMetadata {
    ActorFrameMetadata {
        stream_id: frame.stream_id,
        sequence: frame.sequence,
        capture_ticks: frame.capture_time.ticks(),
        available_ticks: frame.available_time.ticks(),
        age_ticks: controller_time_ticks.saturating_sub(frame.capture_time.ticks()),
    }
}

fn push_observation(bytes: &mut Vec<u8>, observation: &DiffDriveObservation) {
    for value in [
        observation.base_x_m,
        observation.base_y_m,
        observation.base_z_m,
        observation.base_yaw_rad,
        observation.left_wheel_velocity_rad_s,
        observation.right_wheel_velocity_rad_s,
        observation.imu_ay_m_s2,
    ] {
        push_u64(bytes, value.to_bits());
    }
    push_u64(bytes, observation.lidar_points as u64);
    for value in [
        observation.goal_delta_x_m,
        observation.peer_delta_x_m,
        observation.peer_delta_z_m,
        observation.peer_separation_m,
    ] {
        match value {
            Some(value) => {
                bytes.push(1);
                push_u64(bytes, value.to_bits());
            }
            None => bytes.push(0),
        }
    }
}

fn push_metadata(bytes: &mut Vec<u8>, metadata: ActorFrameMetadata) {
    for value in [
        metadata.stream_id.0,
        metadata.sequence,
        metadata.capture_ticks,
        metadata.available_ticks,
        metadata.age_ticks,
    ] {
        push_u64(bytes, value);
    }
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn stable_fnv1a(bytes: &[u8]) -> u64 {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    digest
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_core::SimDuration;
    use rne_data::{Frame, InMemoryDataBus};
    use rne_ecs::Entity;
    use rne_math::Vec3;

    const STREAMS: DiffDriveActorStreams = DiffDriveActorStreams {
        localization: StreamId(10),
        left_wheel_encoder: StreamId(11),
        right_wheel_encoder: StreamId(12),
        imu: StreamId(13),
        lidar: StreamId(14),
    };

    fn publish<T: FramePayload>(
        bus: &mut InMemoryDataBus,
        stream: StreamId,
        sequence: u64,
        capture_ticks: u64,
        latency_ticks: u64,
        payload: T,
    ) {
        bus.publish(
            Frame::new(
                stream,
                Entity::PLACEHOLDER,
                sequence,
                SimTime::from_ticks(capture_ticks),
                payload,
            )
            .with_latency(SimDuration::from_ticks(latency_ticks)),
        );
    }

    fn publish_complete_set(bus: &mut InMemoryDataBus, capture_ticks: u64) {
        publish(
            bus,
            STREAMS.localization,
            1,
            capture_ticks,
            2,
            PoseSample {
                position_m: Vec3::new(1.0, 0.25, -0.5),
                yaw_rad: 0.2,
            },
        );
        publish(
            bus,
            STREAMS.left_wheel_encoder,
            2,
            capture_ticks,
            2,
            WheelEncoderSample {
                position_rad: 0.3,
                velocity_rad_s: 0.0,
            },
        );
        publish(
            bus,
            STREAMS.right_wheel_encoder,
            3,
            capture_ticks,
            2,
            WheelEncoderSample {
                position_rad: 0.4,
                velocity_rad_s: 0.5,
            },
        );
        publish(
            bus,
            STREAMS.imu,
            4,
            capture_ticks,
            2,
            ImuSample {
                linear_acceleration_m_s2: Vec3::new(0.0, 9.7, 0.0),
                ..ImuSample::default()
            },
        );
        publish(
            bus,
            STREAMS.lidar,
            5,
            capture_ticks,
            2,
            PointCloud {
                points_m: vec![Vec3::X, Vec3::Y],
                ..PointCloud::default()
            },
        );
    }

    #[test]
    fn actor_observation_uses_measurements_and_retains_timing() {
        let mut bus = InMemoryDataBus::new();
        publish_complete_set(&mut bus, 10);

        let frame = diff_drive_actor_observation(&bus, STREAMS, SimTime::from_ticks(12), Some(3.0))
            .expect("complete observation");

        assert_eq!(frame.observation.base_x_m, 1.0);
        assert_eq!(frame.observation.left_wheel_velocity_rad_s, 0.0);
        assert_eq!(frame.observation.right_wheel_velocity_rad_s, 0.5);
        assert_eq!(frame.observation.goal_delta_x_m, Some(2.0));
        assert_eq!(frame.observation.lidar_points, 2);
        assert_eq!(frame.localization.capture_ticks, 10);
        assert_eq!(frame.localization.available_ticks, 12);
        assert_eq!(frame.localization.age_ticks, 2);
    }

    #[test]
    fn unavailable_newer_frames_cannot_leak_future_truth_or_command_like_values() {
        let mut bus = InMemoryDataBus::new();
        publish_complete_set(&mut bus, 10);
        publish(
            &mut bus,
            STREAMS.localization,
            20,
            20,
            20,
            PoseSample {
                position_m: Vec3::splat(99.0),
                yaw_rad: 9.0,
            },
        );
        publish(
            &mut bus,
            STREAMS.left_wheel_encoder,
            21,
            20,
            20,
            WheelEncoderSample {
                position_rad: 10.0,
                velocity_rad_s: 6.0,
            },
        );

        let frame = diff_drive_actor_observation(&bus, STREAMS, SimTime::from_ticks(25), None)
            .expect("older available observation");

        assert_eq!(frame.observation.base_x_m, 1.0);
        assert_eq!(frame.observation.left_wheel_velocity_rad_s, 0.0);
        assert_eq!(frame.localization.sequence, 1);
        assert_eq!(frame.left_wheel_encoder.sequence, 2);
    }

    #[test]
    fn incomplete_sensor_set_fails_closed() {
        let bus = InMemoryDataBus::new();

        assert_eq!(
            diff_drive_actor_observation(&bus, STREAMS, SimTime::ZERO, None),
            Err(DiffDriveActorObservationError::MissingAvailableFrame {
                payload: "localization",
                stream_id: 10,
            })
        );
    }

    #[test]
    fn actor_observation_digest_is_canonical_and_sensitive_to_actor_input() {
        let mut bus = InMemoryDataBus::new();
        publish_complete_set(&mut bus, 10);
        let frame = diff_drive_actor_observation(&bus, STREAMS, SimTime::from_ticks(12), Some(2.0))
            .unwrap();
        let digest = stable_diff_drive_actor_observation_digest(&frame);

        assert_eq!(digest, 10_906_356_641_843_652_636);
        assert_eq!(digest, stable_diff_drive_actor_observation_digest(&frame));

        let changed_goal =
            diff_drive_actor_observation(&bus, STREAMS, SimTime::from_ticks(12), Some(2.5))
                .unwrap();
        assert_ne!(
            digest,
            stable_diff_drive_actor_observation_digest(&changed_goal)
        );
    }
}
