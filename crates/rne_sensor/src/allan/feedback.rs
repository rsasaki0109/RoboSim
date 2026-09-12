//! Strict offline projection of available IMU feedback into six scalar statistics.

use super::{overlapping_allan_deviation, AllanError, AllanPoint, AllanSample, MAX_ALLAN_SAMPLES};
use rne_core::{SimDuration, SimTime};
use rne_data::{Frame, ImuFeedback, ImuFeedbackStatus, StreamId};
use rne_ecs::Entity;
use thiserror::Error;

/// Six-axis statistics and the exact source segment identity, not a calibration certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct ImuAllanStatistics {
    /// Single source stream used by this segment.
    pub stream_id: StreamId,
    /// Single source sensor entity, including generation.
    pub entity: Entity,
    /// First accepted sequence (a segment may begin after startup).
    pub first_sequence: u64,
    /// Last accepted sequence, with no internal omissions.
    pub last_sequence: u64,
    /// First capture in simulation ticks.
    pub first_capture_ticks: u64,
    /// Last capture in simulation ticks.
    pub last_capture_ticks: u64,
    /// X/Y/Z gyro deviations in rad/s (variances in (rad/s)^2).
    pub gyro: [Vec<AllanPoint>; 3],
    /// X/Y/Z specific-force deviations in m/s^2 (variances in (m/s^2)^2).
    pub accel: [Vec<AllanPoint>; 3],
}

/// A source contract or statistical failure, with no partial accepted output.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ImuAllanError {
    /// An invalid frame is not silently skipped, sorted, relabelled or repaired.
    #[error("IMU Allan frame {index}: {reason}")]
    Frame {
        /// Zero-based position in the supplied capture-ordered segment.
        index: usize,
        /// Specific failed contract condition.
        reason: &'static str,
    },
    /// Bounded scalar-statistic validation or arithmetic failure.
    #[error(transparent)]
    Statistic(#[from] AllanError),
}

/// Analyze a complete, capture-ordered segment of already available IMU frames.
///
/// Require one entity/stream, a supported payload schema, positive contiguous
/// sequences, finite six-axis data, nominal status and no saturation flags.
/// Scheduled and actual captures must both advance by the specified period;
/// scheduled+phase_error must equal capture. The Frame compatibility timestamp
/// must equal availability, which must lie between capture and `observed_until`.
/// Availability may vary or reorder; it never replaces capture in the statistic.
/// The first sequence need not be one, allowing explicitly chosen startup exclusion.
/// No motion/stationarity or physical-calibration claim follows from acceptance.
pub fn analyze_imu_feedback(
    frames: &[Frame<ImuFeedback>],
    sample_period: SimDuration,
    observed_until: SimTime,
    factors: &[usize],
) -> Result<ImuAllanStatistics, ImuAllanError> {
    if !(2..=MAX_ALLAN_SAMPLES).contains(&frames.len()) {
        return Err(AllanError::Size.into());
    }
    if sample_period.ticks() == 0 {
        return Err(AllanError::Period.into());
    }
    let first = &frames[0];
    for (index, frame) in frames.iter().enumerate() {
        let fail = |reason| ImuAllanError::Frame { index, reason };
        if frame.stream_id != first.stream_id || frame.entity != first.entity {
            return Err(fail("mixed sensor identity"));
        }
        let p = &frame.payload;
        if p.schema_version != ImuFeedback::SCHEMA_VERSION {
            return Err(fail("unsupported schema"));
        }
        if frame.sequence == 0
            || (index > 0 && frames[index - 1].sequence.checked_add(1) != Some(frame.sequence))
        {
            return Err(fail("noncontiguous sequence"));
        }
        if frame.available_time < frame.capture_time
            || frame.available_time > observed_until
            || frame.sim_time != frame.available_time
        {
            return Err(fail("invalid or not-yet-available frame time"));
        }
        if p.scheduled_capture_ticks
            .checked_add(p.sample_phase_error_ticks)
            != Some(frame.capture_time.ticks())
        {
            return Err(fail("inconsistent capture phase"));
        }
        if index > 0
            && (frames[index - 1]
                .payload
                .scheduled_capture_ticks
                .checked_add(sample_period.ticks())
                != Some(p.scheduled_capture_ticks)
                || frames[index - 1]
                    .capture_time
                    .ticks()
                    .checked_add(sample_period.ticks())
                    != Some(frame.capture_time.ticks()))
        {
            return Err(fail("nonuniform capture or schedule"));
        }
        if p.status != ImuFeedbackStatus::Nominal
            || p.gyro_saturated.into_iter().any(|v| v)
            || p.accel_saturated.into_iter().any(|v| v)
        {
            return Err(fail("stuck or saturated measurement"));
        }
        if !p.angular_velocity_rad_s.is_finite() || !p.specific_force_m_s2.is_finite() {
            return Err(fail("nonfinite measurement"));
        }
    }
    let mut channels: [Vec<AllanPoint>; 6] = std::array::from_fn(|_| Vec::new());
    for (axis, channel) in channels.iter_mut().enumerate() {
        let samples: Vec<_> = frames
            .iter()
            .map(|frame| {
                let g = frame.payload.angular_velocity_rad_s;
                let a = frame.payload.specific_force_m_s2;
                AllanSample {
                    capture_ticks: frame.capture_time.ticks(),
                    value: [g.x, g.y, g.z, a.x, a.y, a.z][axis],
                }
            })
            .collect();
        *channel = overlapping_allan_deviation(&samples, sample_period, factors)?;
    }
    let [gx, gy, gz, ax, ay, az] = channels;
    let last = &frames[frames.len() - 1];
    Ok(ImuAllanStatistics {
        stream_id: first.stream_id,
        entity: first.entity,
        first_sequence: first.sequence,
        last_sequence: last.sequence,
        first_capture_ticks: first.capture_time.ticks(),
        last_capture_ticks: last.capture_time.ticks(),
        gyro: [gx, gy, gz],
        accel: [ax, ay, az],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        sample_imu_feedback_sensors, ImuFeedbackFault, ImuFeedbackSensor, ImuMount, ImuSpec,
    };
    use rne_data::{DataBus, InMemoryDataBus};
    use rne_ecs::{spawn_named, World};
    use rne_physics::RigidBody;
    use rne_world::{Transform3, WorldRandom};

    fn observed(fault: ImuFeedbackFault) -> Vec<Frame<ImuFeedback>> {
        observed_with_spec(fault, ImuSpec::default())
    }

    fn observed_with_spec(fault: ImuFeedbackFault, spec: ImuSpec) -> Vec<Frame<ImuFeedback>> {
        let mut world = World::new();
        world.insert_resource(WorldRandom::new(321));
        let body = spawn_named(&mut world, "body");
        world
            .entity_mut(body)
            .insert((RigidBody::default(), Transform3::IDENTITY));
        let sensor = spawn_named(&mut world, "imu");
        let stream = StreamId::new(91);
        world.entity_mut(sensor).insert((
            ImuMount {
                body_entity: body,
                body_from_sensor: Transform3::IDENTITY,
            },
            ImuFeedbackSensor {
                spec,
                update_rate_hz: 100.0,
                sample_period_ticks: Some(10_000_000),
                phase_offset_ticks: 0,
                latency_ticks: 3_000_000,
                enabled: true,
                stream_id: stream,
                fault,
            },
        ));
        let mut bus = InMemoryDataBus::new();
        let mut frames = Vec::new();
        for index in 0..8 {
            world
                .get_mut::<RigidBody>(body)
                .unwrap()
                .angular_velocity_rad_s
                .x = index as f64 * 0.1;
            let capture = SimTime::from_ticks(index * 10_000_000);
            let count = sample_imu_feedback_sensors(&mut world, capture, &mut bus).unwrap();
            if count > 0 {
                let available = SimTime::from_ticks(capture.ticks() + 3_000_000);
                let f = bus
                    .latest_available::<ImuFeedback>(stream, available)
                    .unwrap();
                frames.push(f.clone());
            }
        }
        frames
    }
    fn analyze(frames: &[Frame<ImuFeedback>]) -> Result<ImuAllanStatistics, ImuAllanError> {
        analyze_imu_feedback(
            frames,
            SimDuration::from_ticks(10_000_000),
            SimTime::from_ticks(100_000_000),
            &[1, 2],
        )
    }
    #[test]
    fn actual_frontend_latency_does_not_change_capture_statistics() {
        let frames = observed(ImuFeedbackFault::None);
        let a = analyze(&frames).unwrap();
        assert_eq!(a.first_sequence, 1);
        assert_eq!(a.last_sequence, 8);
        assert!((a.gyro[0][0].variance - 0.005).abs() < 1e-12);
        assert!((a.gyro[0][1].variance - 0.02).abs() < 1e-12);
        assert!(a.accel.iter().flatten().all(|p| p.variance == 0.0));
        let mut delayed = frames.clone();
        for (i, f) in delayed.iter_mut().enumerate() {
            f.available_time = SimTime::from_ticks(100_000_000 - i as u64);
            f.sim_time = f.available_time;
        }
        assert_eq!(a, analyze(&delayed).unwrap());
        assert!(analyze_imu_feedback(
            &frames,
            SimDuration::from_ticks(10_000_000),
            SimTime::from_ticks(72_999_999),
            &[1]
        )
        .is_err());
        let tail = analyze(&frames[2..]).unwrap();
        assert_eq!(tail.first_sequence, 3);
    }
    #[test]
    fn actual_frontend_clipping_is_not_accepted_as_noise_evidence() {
        let frames = observed_with_spec(
            ImuFeedbackFault::None,
            ImuSpec {
                gyro_range_rad_s: 0.25,
                ..ImuSpec::default()
            },
        );
        assert!(analyze_imu_feedback(
            &frames[..3],
            SimDuration::from_ticks(10_000_000),
            SimTime::from_ticks(100_000_000),
            &[1],
        )
        .is_ok());
        assert_eq!(frames[3].payload.angular_velocity_rad_s.x, 0.25);
        assert!(frames[3].payload.gyro_saturated[0]);
        assert!(matches!(
            analyze(&frames),
            Err(ImuAllanError::Frame { index: 3, .. })
        ));
        let frames = observed_with_spec(
            ImuFeedbackFault::None,
            ImuSpec {
                accel_range_m_s2: 1.0,
                ..ImuSpec::default()
            },
        );
        assert!(frames[0].payload.accel_saturated.into_iter().any(|v| v));
        assert!(matches!(
            analyze(&frames),
            Err(ImuAllanError::Frame { index: 0, .. })
        ));
    }

    #[test]
    fn actual_frontend_faults_and_corrupt_metadata_are_rejected() {
        assert!(analyze(&observed(ImuFeedbackFault::DropSequence { sequence: 3 })).is_err());
        assert!(analyze(&observed(ImuFeedbackFault::StuckFromSequence {
            sequence: 3
        }))
        .is_err());
        let original = observed(ImuFeedbackFault::None);
        for case in 0..10 {
            let mut frames = original.clone();
            let f = &mut frames[3];
            match case {
                0 => f.stream_id = StreamId::new(1),
                1 => f.sequence += 1,
                2 => f.payload.schema_version += 1,
                3 => f.payload.sample_phase_error_ticks += 1,
                4 => f.payload.gyro_saturated[0] = true,
                5 => f.payload.specific_force_m_s2.x = f64::NAN,
                6 => f.available_time = SimTime::ZERO,
                7 => f.sim_time = SimTime::ZERO,
                8 => {
                    f.payload.scheduled_capture_ticks += 1;
                    f.capture_time = SimTime::from_ticks(f.capture_time.ticks() + 1);
                }
                _ => f.entity = Entity::PLACEHOLDER,
            }
            assert!(analyze(&frames).is_err(), "case={case}");
        }
        assert_eq!(original, observed(ImuFeedbackFault::None));
    }
}
