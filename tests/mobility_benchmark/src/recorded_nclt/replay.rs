//! Source-time DataBus replay; configured delays are experiments, not measured latency.

use super::{read_imu_samples, read_wheel_samples, NcltImuSample, NcltSeries, NcltWheelSample};
use anyhow::{ensure, Context, Result};
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, FramePayload, InMemoryDataBus, StreamId};
use rne_ecs::Entity;
use serde::Serialize;
use std::io::Read;

impl FramePayload for NcltWheelSample {}
impl FramePayload for NcltImuSample {}

const WHEELS: StreamId = StreamId::new(1);
const IMU: StreamId = StreamId::new(2);

/// Explicit replay transport policy. These delays are not source sensor calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct NcltReplayPolicy {
    /// Constant wheel delivery delay, bounded to one second.
    pub wheel_delay_us: u64,
    /// Constant IMU delivery delay, bounded to one second.
    pub imu_delay_us: u64,
}

/// Latest delivered source observations, with no pose truth or reconstructed counts.
#[derive(Clone, Debug, PartialEq)]
pub struct NcltReplayObservation {
    /// Current replay decision time, in a shared relative simulation clock.
    pub decision_time: SimTime,
    /// Latest available wheel speeds; absent before the first delivery.
    pub wheels: Option<Frame<NcltWheelSample>>,
    /// Latest available filtered IMU; absent before the first delivery.
    pub imu: Option<Frame<NcltImuSample>>,
}

/// Replay constructed from bounded source bytes, not caller-modified parsed structs.
///
/// Frame capture times are source-time scheduling surrogates, not certified physical
/// capture times. Payloads retain Unix timestamps, source axes and filtering. No
/// noise is added; unknown source uncertainty and sensor status stay unknown.
/// Reconstruct with `new` for reset. The bus retains one latest frame per stream;
/// use `next_delivery_time` to visit every distinct delivery boundary.
pub struct NcltReplay {
    wheels: NcltSeries<NcltWheelSample>,
    imu: NcltSeries<NcltImuSample>,
    policy: NcltReplayPolicy,
    origin_us: u64,
    wheel_cursor: usize,
    imu_cursor: usize,
    now: SimTime,
    entity: Entity,
    bus: InMemoryDataBus,
}

impl std::fmt::Debug for NcltReplay {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NcltReplay")
            .field("policy", &self.policy)
            .field("origin_us", &self.origin_us)
            .field("wheel_cursor", &self.wheel_cursor)
            .field("imu_cursor", &self.imu_cursor)
            .field("now", &self.now)
            .finish_non_exhaustive()
    }
}

impl NcltReplay {
    /// Validate both complete inputs and timestamp ranges before creating any bus frames.
    pub fn new(
        wheels: impl Read,
        imu: impl Read,
        policy: NcltReplayPolicy,
        entity: Entity,
    ) -> Result<Self> {
        ensure!(
            policy.wheel_delay_us <= 1_000_000 && policy.imu_delay_us <= 1_000_000,
            "replay delay exceeds one second"
        );
        let wheels = read_wheel_samples(wheels)?;
        let imu = read_imu_samples(imu)?;
        let origin_us = wheels.samples[0]
            .timestamp_us
            .min(imu.samples[0].timestamp_us);
        // Readers establish nonempty, sorted input; checking each final timestamp
        // proves every earlier conversion and delay addition safe as well.
        delivery_ticks(
            wheels.samples.last().unwrap().timestamp_us,
            origin_us,
            policy.wheel_delay_us,
        )?;
        delivery_ticks(
            imu.samples.last().unwrap().timestamp_us,
            origin_us,
            policy.imu_delay_us,
        )?;
        Ok(Self {
            wheels,
            imu,
            policy,
            origin_us,
            wheel_cursor: 0,
            imu_cursor: 0,
            now: SimTime::ZERO,
            entity,
            bus: InMemoryDataBus::with_capacity_per_stream(1)?,
        })
    }

    /// Shared Unix-microsecond origin chosen as the earliest sample across both streams.
    pub fn origin_us(&self) -> u64 {
        self.origin_us
    }

    /// Immutable transport policy used by this replay.
    pub fn policy(&self) -> NcltReplayPolicy {
        self.policy
    }

    /// Exact input digests in wheel, IMU order; no publisher authenticity is implied.
    pub fn source_sha256(&self) -> (&str, &str) {
        (&self.wheels.source_sha256, &self.imu.source_sha256)
    }

    /// Next delivery boundary; ties publish wheels before IMU in `advance_to`.
    pub fn next_delivery_time(&self) -> Option<SimTime> {
        let wheel = self
            .wheels
            .samples
            .get(self.wheel_cursor)
            .map(|s| self.time(s.timestamp_us, self.policy.wheel_delay_us));
        let imu = self
            .imu
            .samples
            .get(self.imu_cursor)
            .map(|s| self.time(s.timestamp_us, self.policy.imu_delay_us));
        match (wheel, imu) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// Deliver all samples due by `now`. Backward calls fail before modifying state.
    ///
    /// Equal-time events are delivered together; a large jump intentionally retains
    /// only the latest delivered frame of each stream. No future sample is exposed.
    pub fn advance_to(&mut self, now: SimTime) -> Result<NcltReplayObservation> {
        ensure!(now >= self.now, "replay clock cannot move backward");
        while let Some(next) = self.next_delivery_time() {
            if next > now {
                break;
            }
            if let Some(sample) = self.wheels.samples.get(self.wheel_cursor) {
                if self.time(sample.timestamp_us, self.policy.wheel_delay_us) == next {
                    self.bus.publish(
                        Frame::new(
                            WHEELS,
                            self.entity,
                            self.wheel_cursor as u64,
                            self.time(sample.timestamp_us, 0),
                            sample.clone(),
                        )
                        .with_latency(SimDuration::from_ticks(self.policy.wheel_delay_us * 1000)),
                    );
                    self.wheel_cursor += 1;
                }
            }
            if let Some(sample) = self.imu.samples.get(self.imu_cursor) {
                if self.time(sample.timestamp_us, self.policy.imu_delay_us) == next {
                    self.bus.publish(
                        Frame::new(
                            IMU,
                            self.entity,
                            self.imu_cursor as u64,
                            self.time(sample.timestamp_us, 0),
                            sample.clone(),
                        )
                        .with_latency(SimDuration::from_ticks(self.policy.imu_delay_us * 1000)),
                    );
                    self.imu_cursor += 1;
                }
            }
        }
        self.now = now;
        Ok(self.observation())
    }

    /// Read only latest available source frames at the current replay decision time.
    pub fn observation(&self) -> NcltReplayObservation {
        NcltReplayObservation {
            decision_time: self.now,
            wheels: self.bus.latest_available(WHEELS, self.now),
            imu: self.bus.latest_available(IMU, self.now),
        }
    }

    fn time(&self, timestamp_us: u64, delay_us: u64) -> SimTime {
        SimTime::from_ticks(
            delivery_ticks(timestamp_us, self.origin_us, delay_us)
                .expect("constructor checked sorted source ranges"),
        )
    }
}

fn delivery_ticks(timestamp_us: u64, origin_us: u64, delay_us: u64) -> Result<u64> {
    timestamp_us
        .checked_sub(origin_us)
        .and_then(|t| t.checked_add(delay_us))
        .and_then(|t| t.checked_mul(1000))
        .context("replay time overflow")
}

#[cfg(test)]
mod tests {
    use super::*;
    const W: &[u8] = b"1350000000000010,1,2\n1350000000000030,3,4\n";
    const I: &[u8] = b"1350000000000000,1,2,3,4,5,6,7,8,9\n1350000000000020,9,8,7,6,5,4,3,2,1\n";
    fn entity() -> Entity {
        rne_ecs::spawn_named(&mut rne_ecs::World::new(), "replay")
    }
    fn replay() -> NcltReplay {
        NcltReplay::new(
            W,
            I,
            NcltReplayPolicy {
                wheel_delay_us: 5,
                imu_delay_us: 15,
            },
            entity(),
        )
        .unwrap()
    }

    #[test]
    fn common_origin_delay_ties_and_no_future_leakage() {
        let mut r = replay();
        assert_eq!(r.origin_us(), 1350000000000000);
        assert_eq!(r.next_delivery_time(), Some(SimTime::from_ticks(15000)));
        let early = r.advance_to(SimTime::from_ticks(14999)).unwrap();
        assert!(early.wheels.is_none() && early.imu.is_none());
        let first = r.advance_to(SimTime::from_ticks(15000)).unwrap();
        let w = first.wheels.unwrap();
        let i = first.imu.unwrap();
        assert_eq!(w.capture_time.ticks(), 10000);
        assert_eq!(i.capture_time.ticks(), 0);
        assert_eq!(w.available_time.ticks(), 15000);
        assert_eq!(i.available_time.ticks(), 15000);
        assert_eq!(w.payload.left_speed_m_s, 1.0);
        assert_eq!(i.payload.angular_velocity_rad_s, [7.0, 8.0, 9.0]);
        assert_eq!(
            r.advance_to(SimTime::from_ticks(34999))
                .unwrap()
                .wheels
                .unwrap()
                .sequence,
            0
        );
        assert_eq!(
            r.advance_to(SimTime::from_ticks(35000))
                .unwrap()
                .wheels
                .unwrap()
                .sequence,
            1
        );
        assert!(r.next_delivery_time().is_none());
    }

    #[test]
    fn reset_reconstruction_jump_and_backward_rejection() {
        let mut a = replay();
        let mut b = replay();
        while let Some(t) = a.next_delivery_time() {
            assert_eq!(a.advance_to(t).unwrap(), b.advance_to(t).unwrap());
        }
        let mut jumped = replay();
        assert_eq!(
            a.observation(),
            jumped.advance_to(SimTime::from_ticks(35000)).unwrap()
        );
        let before = a.observation();
        assert!(a.advance_to(SimTime::ZERO).is_err());
        assert_eq!(a.observation(), before);
        assert_eq!(a.source_sha256(), b.source_sha256());
    }

    #[test]
    fn staggered_streams_remain_missing_until_delivery_and_age_after_end() {
        let mut r = NcltReplay::new(
            W,
            I,
            NcltReplayPolicy {
                wheel_delay_us: 0,
                imu_delay_us: 0,
            },
            entity(),
        )
        .unwrap();
        let first = r.advance_to(SimTime::ZERO).unwrap();
        assert!(first.wheels.is_none());
        assert_eq!(first.imu.as_ref().unwrap().sequence, 0);
        assert_eq!(first, r.advance_to(SimTime::ZERO).unwrap());
        let next = r.advance_to(SimTime::from_ticks(10000)).unwrap();
        assert_eq!(next.wheels.unwrap().sequence, 0);
        assert_eq!(next.imu.unwrap().sequence, 0);
        let late = r.advance_to(SimTime::from_ticks(1_000_000)).unwrap();
        assert_eq!(late.wheels.unwrap().capture_time.ticks(), 30000);
        assert_eq!(late.imu.unwrap().capture_time.ticks(), 20000);
        assert!(r.next_delivery_time().is_none());
    }

    #[test]
    fn invalid_source_or_policy_never_creates_a_replay() {
        let p = NcltReplayPolicy {
            wheel_delay_us: 0,
            imu_delay_us: 0,
        };
        assert!(NcltReplay::new(W, &b""[..], p, entity()).is_err());
        assert!(NcltReplay::new(
            W,
            I,
            NcltReplayPolicy {
                wheel_delay_us: 1_000_001,
                ..p
            },
            entity()
        )
        .is_err());
        assert!(delivery_ticks(0, 1, 0).is_err());
        assert!(delivery_ticks(u64::MAX, 0, 1).is_err());
        assert!(delivery_ticks(u64::MAX / 1000 + 1, 0, 0).is_err());
    }
}
