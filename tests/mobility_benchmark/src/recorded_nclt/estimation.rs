//! Fixed-lag planar wheel-speed/gyro integration of delivered source observations.
//!
//! This is an uncalibrated no-slip planar baseline, not a full 3D or statistical
//! filter. Unknown motion splits local segments; no pose truth is consumed.

use super::{
    replay::{NcltReplayObservation, NcltReplayPolicy},
    NcltImuSample, NcltWheelSample,
};
use anyhow::{ensure, Context, Result};
use rne_data::{Frame, FramePayload};
use serde::Serialize;
use std::collections::VecDeque;

#[cfg(test)]
mod tests;

/// Explicit assumptions and hold limits; no values are silently calibrated.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct VelocityEstimatorConfig {
    /// Maximum wheel hold age, strictly positive and at most ten seconds.
    pub wheel_hold_us: u64,
    /// Maximum IMU hold age, strictly positive and at most ten seconds.
    pub imu_hold_us: u64,
    /// Assumed wheel-speed scale, finite and strictly positive.
    pub wheel_scale: f64,
    /// Assumed source gyro-z bias in radians per second.
    pub gyro_z_bias_rad_s: f64,
    /// Initial x/y position for every local segment, in source planar axes.
    pub segment_origin_m: [f64; 2],
    /// Initial source-axis heading for every local segment, in radians.
    pub segment_heading_rad: f64,
    /// Maximum pending samples per stream, from one through 4096.
    pub queue_capacity: usize,
}

/// A local planar pose; it must not be interpreted as a global joined trajectory.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct VelocitySegmentPose {
    /// One-based local segment identity.
    pub segment_id: u64,
    /// Source planar x/y position in meters.
    pub position_m: [f64; 2],
    /// Heading about source down axis, in radians.
    pub heading_rad: f64,
}

/// Timestamped integration and coverage evidence; absent pose means no fresh pair.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct VelocityEstimate {
    /// Decision time in simulation nanosecond ticks.
    pub decision_time_ticks: u64,
    /// Completed source horizon; absent before the configured output lag elapses.
    pub estimate_time_ticks: Option<u64>,
    /// Declared maximum replay delay, not a measured capture delay.
    pub output_lag_ticks: u64,
    /// Current local pose only when both held inputs are fresh.
    pub pose: Option<VelocitySegmentPose>,
    /// Integrated source-time duration across all local segments.
    pub integrated_ticks: u64,
    /// Source-time duration with missing or expired observations.
    pub unobserved_ticks: u64,
    /// Age of the held wheel input at the estimate horizon.
    pub wheel_age_ticks: Option<u64>,
    /// Age of the held IMU input at the estimate horizon.
    pub imu_age_ticks: Option<u64>,
    /// Always false: no calibrated covariance is available in this baseline.
    pub uncertainty_calibrated: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Held {
    capture: u64,
    value: f64,
}

/// Transactional, bounded fixed-lag estimator of delivered NCLT source samples.
///
/// Call at every replay delivery boundary and finally at the last source time
/// plus maximum delay. Duplicate unchanged observations are harmless. Sequence
/// gaps, changed duplicates, malformed frames and backward clocks are rejected.
#[derive(Clone, Debug, PartialEq)]
pub struct MeasuredVelocityEstimator {
    config: VelocityEstimatorConfig,
    policy: NcltReplayPolicy,
    origin_us: u64,
    last_decision: u64,
    cursor: u64,
    wheels: VecDeque<Held>,
    imu: VecDeque<Held>,
    last_wheel: Option<Frame<NcltWheelSample>>,
    last_imu: Option<Frame<NcltImuSample>>,
    wheel: Option<Held>,
    gyro: Option<Held>,
    segment: Option<VelocitySegmentPose>,
    segment_count: u64,
    integrated: u64,
    unobserved: u64,
}

impl MeasuredVelocityEstimator {
    /// Construct with explicit replay timing and model assumptions; no source arrays are read.
    pub fn new(
        origin_us: u64,
        policy: NcltReplayPolicy,
        config: VelocityEstimatorConfig,
    ) -> Result<Self> {
        ensure!(
            (1_000_000_000_000_000..10_000_000_000_000_000).contains(&origin_us),
            "invalid source origin"
        );
        ensure!(
            policy.wheel_delay_us <= 1_000_000 && policy.imu_delay_us <= 1_000_000,
            "invalid replay delays"
        );
        ensure!(
            (1..=10_000_000).contains(&config.wheel_hold_us)
                && (1..=10_000_000).contains(&config.imu_hold_us),
            "invalid hold duration"
        );
        ensure!(
            (1..=4096).contains(&config.queue_capacity),
            "invalid queue capacity"
        );
        ensure!(
            config.wheel_scale.is_finite()
                && config.wheel_scale > 0.0
                && config.gyro_z_bias_rad_s.is_finite()
                && config.segment_heading_rad.is_finite()
                && config.segment_origin_m.iter().all(|v| v.is_finite()),
            "non-finite or invalid model assumption"
        );
        Ok(Self {
            config,
            policy,
            origin_us,
            last_decision: 0,
            cursor: 0,
            wheels: VecDeque::new(),
            imu: VecDeque::new(),
            last_wheel: None,
            last_imu: None,
            wheel: None,
            gyro: None,
            segment: None,
            segment_count: 0,
            integrated: 0,
            unobserved: 0,
        })
    }

    /// Process only delivered observations. Any returned error leaves the estimator unchanged.
    pub fn update(&mut self, observation: &NcltReplayObservation) -> Result<VelocityEstimate> {
        let mut candidate = self.clone();
        let result = candidate.update_inner(observation)?;
        *self = candidate;
        Ok(result)
    }

    fn update_inner(&mut self, o: &NcltReplayObservation) -> Result<VelocityEstimate> {
        let decision = o.decision_time.ticks();
        ensure!(decision >= self.last_decision, "backward decision time");
        if admit(
            &o.wheels,
            &self.last_wheel,
            self.origin_us,
            self.policy.wheel_delay_us,
            decision,
            1,
            |s| s.timestamp_us,
        )? {
            let f = o.wheels.as_ref().unwrap();
            ensure!(
                f.payload.left_speed_m_s.is_finite() && f.payload.right_speed_m_s.is_finite(),
                "non-finite wheel speed"
            );
            let value = (0.5 * f.payload.left_speed_m_s + 0.5 * f.payload.right_speed_m_s)
                * self.config.wheel_scale;
            ensure!(value.is_finite(), "scaled speed overflow");
            ensure!(
                self.wheels.len() < self.config.queue_capacity,
                "wheel queue exhausted"
            );
            ensure!(
                f.capture_time.ticks() >= self.cursor,
                "late wheel capture beyond lag bound"
            );
            self.wheels.push_back(Held {
                capture: f.capture_time.ticks(),
                value,
            });
            self.last_wheel = Some(f.clone());
        }
        if admit(
            &o.imu,
            &self.last_imu,
            self.origin_us,
            self.policy.imu_delay_us,
            decision,
            2,
            |s| s.timestamp_us,
        )? {
            let f = o.imu.as_ref().unwrap();
            ensure!(
                f.payload
                    .angular_velocity_rad_s
                    .iter()
                    .chain(f.payload.acceleration_m_s2.iter())
                    .chain(f.payload.magnetic_field_gauss.iter())
                    .all(|v| v.is_finite()),
                "non-finite IMU"
            );
            let value = f.payload.angular_velocity_rad_s[2] - self.config.gyro_z_bias_rad_s;
            ensure!(value.is_finite(), "gyro correction overflow");
            ensure!(
                self.imu.len() < self.config.queue_capacity,
                "IMU queue exhausted"
            );
            ensure!(
                f.capture_time.ticks() >= self.cursor,
                "late IMU capture beyond lag bound"
            );
            self.imu.push_back(Held {
                capture: f.capture_time.ticks(),
                value,
            });
            self.last_imu = Some(f.clone());
        }
        if let (Some(w), Some(i)) = (&self.last_wheel, &self.last_imu) {
            ensure!(w.entity == i.entity, "mixed source entities");
        }
        let lag = self.policy.wheel_delay_us.max(self.policy.imu_delay_us) * 1000;
        let horizon = decision.checked_sub(lag);
        if let Some(h) = horizon {
            loop {
                let next = self
                    .wheels
                    .front()
                    .map(|s| s.capture)
                    .into_iter()
                    .chain(self.imu.front().map(|s| s.capture))
                    .min();
                let Some(t) = next.filter(|t| *t <= h) else {
                    break;
                };
                self.integrate_to(t)?;
                if self.wheels.front().is_some_and(|s| s.capture == t) {
                    self.wheel = self.wheels.pop_front();
                }
                if self.imu.front().is_some_and(|s| s.capture == t) {
                    self.gyro = self.imu.pop_front();
                }
            }
            self.integrate_to(h)?;
            if self.fresh_until().is_some_and(|end| end > self.cursor) {
                self.start_segment();
            }
        }
        self.last_decision = decision;
        let fresh = horizon.is_some() && self.fresh_until().is_some_and(|end| end > self.cursor);
        Ok(VelocityEstimate {
            decision_time_ticks: decision,
            estimate_time_ticks: horizon,
            output_lag_ticks: lag,
            pose: if fresh { self.segment } else { None },
            integrated_ticks: self.integrated,
            unobserved_ticks: self.unobserved,
            wheel_age_ticks: self.wheel.map(|s| self.cursor - s.capture),
            imu_age_ticks: self.gyro.map(|s| self.cursor - s.capture),
            uncertainty_calibrated: false,
        })
    }

    fn fresh_until(&self) -> Option<u64> {
        let w = self.wheel?;
        let i = self.gyro?;
        Some(
            w.capture
                .saturating_add(self.config.wheel_hold_us * 1000)
                .min(i.capture.saturating_add(self.config.imu_hold_us * 1000)),
        )
    }

    fn start_segment(&mut self) {
        if self.segment.is_none() {
            self.segment_count += 1;
            self.segment = Some(VelocitySegmentPose {
                segment_id: self.segment_count,
                position_m: self.config.segment_origin_m,
                heading_rad: self.config.segment_heading_rad,
            });
        }
    }

    fn integrate_to(&mut self, end: u64) -> Result<()> {
        ensure!(end >= self.cursor, "source clock moved backward");
        if end == self.cursor {
            return Ok(());
        }
        let valid_end = self
            .fresh_until()
            .unwrap_or(self.cursor)
            .min(end)
            .max(self.cursor);
        if valid_end > self.cursor {
            self.start_segment();
            let dt = (valid_end - self.cursor) as f64 * 1e-9;
            let pose = self.segment.as_mut().unwrap();
            integrate_pose(
                pose,
                self.wheel.unwrap().value,
                self.gyro.unwrap().value,
                dt,
            )?;
            self.integrated += valid_end - self.cursor;
        }
        if valid_end < end {
            self.unobserved += end - valid_end;
            self.segment = None;
        }
        self.cursor = end;
        Ok(())
    }
}

fn admit<T: FramePayload + PartialEq>(
    incoming: &Option<Frame<T>>,
    previous: &Option<Frame<T>>,
    origin_us: u64,
    delay_us: u64,
    decision: u64,
    stream: u64,
    timestamp: impl Fn(&T) -> u64,
) -> Result<bool> {
    let Some(f) = incoming else {
        ensure!(previous.is_none(), "previously seen stream disappeared");
        return Ok(false);
    };
    let capture = timestamp(&f.payload)
        .checked_sub(origin_us)
        .and_then(|t| t.checked_mul(1000))
        .context("invalid source timestamp")?;
    let available = capture
        .checked_add(delay_us * 1000)
        .context("availability overflow")?;
    ensure!(
        f.stream_id.0 == stream
            && f.capture_time.ticks() == capture
            && f.available_time.ticks() == available
            && f.sim_time.ticks() == available
            && available <= decision,
        "invalid or unavailable frame"
    );
    if let Some(p) = previous {
        if f.sequence == p.sequence {
            ensure!(f == p, "changed duplicate frame");
            return Ok(false);
        }
        ensure!(
            p.sequence.checked_add(1) == Some(f.sequence)
                && f.capture_time > p.capture_time
                && f.entity == p.entity,
            "sequence gap or source change"
        );
    } else {
        ensure!(f.sequence == 0, "first sequence must be zero");
    }
    Ok(true)
}

fn integrate_pose(
    pose: &mut VelocitySegmentPose,
    speed: f64,
    yaw_rate: f64,
    dt: f64,
) -> Result<()> {
    let angle = yaw_rate * dt;
    let half = angle * 0.5;
    let sinc = if half.abs() < 1e-6 {
        1.0 - half * half / 6.0
    } else {
        half.sin() / half
    };
    let distance = speed * dt * sinc;
    let mid = pose.heading_rad + half;
    pose.position_m[0] += distance * mid.cos();
    pose.position_m[1] += distance * mid.sin();
    pose.heading_rad = (pose.heading_rad + angle + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    ensure!(
        pose.position_m.iter().all(|v| v.is_finite()) && pose.heading_rad.is_finite(),
        "pose integration overflow"
    );
    Ok(())
}
