//! Sensor-only planar odometry for Ackermann vehicles.
//!
//! The estimator consumes four wheel encoders, two front steering encoders, and a
//! mounted IMU exclusively through availability-time-aware DataBus reads. It does
//! not accept commands, ECS state, physics handles, or privileged vehicle truth.

use rne_core::SimTime;
use rne_data::{
    DataBus, Frame, ImuFeedback, ImuFeedbackStatus, IncrementalEncoderFeedback,
    IncrementalEncoderStatus, PoseSample, StreamId,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use thiserror::Error;

/// Four wheel and two steering encoder streams consumed by Ackermann odometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AckermannEncoderStreams {
    /// Front-left wheel rotation encoder.
    pub front_left_wheel: StreamId,
    /// Rear-left wheel rotation encoder.
    pub rear_left_wheel: StreamId,
    /// Front-right wheel rotation encoder.
    pub front_right_wheel: StreamId,
    /// Rear-right wheel rotation encoder.
    pub rear_right_wheel: StreamId,
    /// Front-left absolute steering encoder.
    pub front_left_steering: StreamId,
    /// Front-right absolute steering encoder.
    pub front_right_steering: StreamId,
    /// Mounted IMU stream whose z axis is vehicle yaw.
    pub imu: StreamId,
}

impl AckermannEncoderStreams {
    fn wheel_streams(self) -> [StreamId; 4] {
        [
            self.front_left_wheel,
            self.rear_left_wheel,
            self.front_right_wheel,
            self.rear_right_wheel,
        ]
    }

    fn steering_streams(self) -> [StreamId; 2] {
        [self.front_left_steering, self.front_right_steering]
    }
}

/// Geometry, calibration, synchronization, and uncertainty contract.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AckermannImuOdometryConfig {
    /// Effective rolling radius shared by the four wheel encoders, in meters.
    pub wheel_radius_m: f64,
    /// Rear-to-front axle distance, in meters.
    pub wheelbase_m: f64,
    /// Left-to-right wheel-center distance, in meters.
    pub track_width_m: f64,
    /// Decoded counts per mechanical revolution for each wheel encoder.
    pub wheel_counts_per_revolution: [u32; 4],
    /// Signed finite counter width for each wheel encoder.
    pub wheel_counter_bits: [u8; 4],
    /// Per-wheel count direction, restricted to `-1` or `1`.
    pub wheel_direction: [i8; 4],
    /// Largest physically plausible wheel count change per accepted update.
    pub max_abs_wheel_delta_counts: u64,
    /// Mounted-IMU yaw-axis direction, restricted to `-1.0` or `1.0`.
    pub gyro_z_direction: f64,
    /// Calibrated yaw-rate bias removed from the IMU observation, in rad/s.
    pub gyro_z_bias_rad_s: f64,
    /// Complementary-filter weight assigned to the IMU yaw increment.
    pub gyro_yaw_weight: f64,
    /// IMU weight used when wheel/steering and IMU yaw increments disagree.
    pub disagreement_gyro_yaw_weight: f64,
    /// Absolute yaw-increment disagreement that changes estimator health, in radians.
    pub disagreement_threshold_rad: f64,
    /// Maximum permitted curvature difference reconstructed from the two steering sensors.
    pub steering_curvature_disagreement_per_m: f64,
    /// One-sigma longitudinal distance uncertainty per update, in meters.
    pub wheel_distance_std_m: f64,
    /// One-sigma steering-derived yaw increment uncertainty, in radians.
    pub encoder_yaw_std_rad: f64,
    /// One-sigma calibrated gyro rate uncertainty, in rad/s.
    pub gyro_rate_std_rad_s: f64,
    /// Maximum capture-time separation among all seven inputs, in ticks.
    pub max_input_skew_ticks: u64,
    /// Maximum age of the oldest input at the estimator decision, in ticks.
    pub max_frame_age_ticks: u64,
}

impl AckermannImuOdometryConfig {
    /// Validates geometry, calibration, counter, fusion, and timing invariants.
    pub fn validate(self) -> Result<(), AckermannImuOdometryError> {
        for (name, value) in [
            ("wheel_radius_m", self.wheel_radius_m),
            ("wheelbase_m", self.wheelbase_m),
            ("track_width_m", self.track_width_m),
            (
                "disagreement_threshold_rad",
                self.disagreement_threshold_rad,
            ),
            (
                "steering_curvature_disagreement_per_m",
                self.steering_curvature_disagreement_per_m,
            ),
            ("wheel_distance_std_m", self.wheel_distance_std_m),
            ("encoder_yaw_std_rad", self.encoder_yaw_std_rad),
            ("gyro_rate_std_rad_s", self.gyro_rate_std_rad_s),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(AckermannImuOdometryError::InvalidConfig(name));
            }
        }
        if !self.gyro_z_bias_rad_s.is_finite() {
            return Err(AckermannImuOdometryError::InvalidConfig(
                "gyro_z_bias_rad_s",
            ));
        }
        if !matches!(self.gyro_z_direction, -1.0 | 1.0) {
            return Err(AckermannImuOdometryError::InvalidConfig("gyro_z_direction"));
        }
        if !(0.0..=1.0).contains(&self.gyro_yaw_weight)
            || !(0.0..=1.0).contains(&self.disagreement_gyro_yaw_weight)
        {
            return Err(AckermannImuOdometryError::InvalidConfig("gyro_yaw_weight"));
        }
        if self.wheel_counts_per_revolution.contains(&0) {
            return Err(AckermannImuOdometryError::InvalidConfig(
                "wheel_counts_per_revolution",
            ));
        }
        if self
            .wheel_counter_bits
            .iter()
            .any(|bits| !(2..=63).contains(bits))
        {
            return Err(AckermannImuOdometryError::InvalidConfig(
                "wheel_counter_bits",
            ));
        }
        if self
            .wheel_direction
            .iter()
            .any(|direction| !matches!(direction, -1 | 1))
        {
            return Err(AckermannImuOdometryError::InvalidConfig("wheel_direction"));
        }
        let smallest_half_range = self
            .wheel_counter_bits
            .iter()
            .map(|bits| 1_u64 << (bits - 1))
            .min()
            .expect("four counters");
        if self.max_abs_wheel_delta_counts == 0
            || self.max_abs_wheel_delta_counts >= smallest_half_range
        {
            return Err(AckermannImuOdometryError::InvalidConfig(
                "max_abs_wheel_delta_counts",
            ));
        }
        Ok(())
    }
}

/// Health classification attached to each accepted estimate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AckermannImuOdometryHealth {
    /// First synchronized set established the counter baseline.
    #[default]
    Initializing,
    /// All inputs advanced normally and agreed within declared bounds.
    Nominal,
    /// At least one sensor sequence exposed missing input frames.
    InputSequenceGap,
    /// IMU range clipping caused the update to use steering/wheel yaw only.
    ImuSaturated,
    /// Left and right steering sensors imply incompatible curvatures.
    SteeringDisagreement,
    /// Steering/wheel and IMU yaw increments disagree beyond the configured limit.
    WheelImuDisagreement,
}

/// Sensor provenance retained with every Ackermann estimate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AckermannImuOdometryProvenance {
    /// Latest accepted wheel encoder sequences in FL, RL, FR, RR order.
    pub wheel_sequences: [u64; 4],
    /// Latest accepted steering encoder sequences in FL, FR order.
    pub steering_sequences: [u64; 2],
    /// Latest accepted IMU sequence.
    pub imu_sequence: u64,
    /// Newest capture timestamp among the synchronized inputs.
    pub capture_ticks: u64,
    /// Estimator decision timestamp.
    pub decision_ticks: u64,
    /// Age of the oldest synchronized input at the decision.
    pub max_age_ticks: u64,
    /// Total missing sequence count since the preceding accepted update.
    pub skipped_sequences: u64,
}

/// One sensor-only planar Ackermann state estimate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AckermannImuOdometryEstimate {
    /// Estimated planar pose; `position_m.x/y` are forward/lateral world coordinates.
    pub pose: PoseSample,
    /// Estimated rear-axle-center forward speed, in m/s.
    pub linear_velocity_m_s: f64,
    /// Fused estimated yaw rate, in rad/s.
    pub angular_velocity_rad_s: f64,
    /// Steering-derived path curvature, in 1/m.
    pub curvature_per_m: f64,
    /// Steering/wheel yaw increment for this update, in radians.
    pub encoder_delta_yaw_rad: f64,
    /// IMU yaw increment for this update, in radians.
    pub gyro_delta_yaw_rad: f64,
    /// Wrapped encoder-minus-IMU yaw increment, in radians.
    pub yaw_innovation_rad: f64,
    /// Current estimator health.
    pub health: AckermannImuOdometryHealth,
    /// Exact DataBus input provenance.
    pub provenance: AckermannImuOdometryProvenance,
    /// Planar `[x, y, yaw]` covariance.
    pub pose_covariance: [[f64; 3]; 3],
}

/// Failure to produce a valid sensor-only Ackermann odometry update.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum AckermannImuOdometryError {
    /// A configuration field violated its documented invariant.
    #[error("invalid Ackermann/IMU odometry configuration field: {0}")]
    InvalidConfig(&'static str),
    /// A required frame had not arrived by the decision time.
    #[error("no available {payload} frame on stream {stream_id}")]
    MissingAvailableFrame {
        /// Human-readable input role.
        payload: &'static str,
        /// Required stream identifier.
        stream_id: u64,
    },
    /// Input capture timestamps exceed the synchronization bound.
    #[error("input capture skew {observed_ticks} ticks exceeds {maximum_ticks} ticks")]
    InputSkew {
        /// Observed newest-minus-oldest capture time.
        observed_ticks: u64,
        /// Configured maximum capture skew.
        maximum_ticks: u64,
    },
    /// The oldest input is too old for this decision.
    #[error("input age {observed_ticks} ticks exceeds {maximum_ticks} ticks")]
    StaleInput {
        /// Observed age.
        observed_ticks: u64,
        /// Configured maximum age.
        maximum_ticks: u64,
    },
    /// At least one required stream did not advance.
    #[error("no new synchronized Ackermann sensor set is available")]
    NoNewSensorSet,
    /// Capture time did not advance.
    #[error("capture time did not advance")]
    NonAdvancingCaptureTime,
    /// A sensor reported a held value.
    #[error("{sensor} reported stuck-value status")]
    StuckValue {
        /// Stable sensor role.
        sensor: &'static str,
    },
    /// A finite wheel or steering encoder counter saturated.
    #[error("{sensor} encoder counter is saturated")]
    EncoderSaturated {
        /// Stable sensor role.
        sensor: &'static str,
    },
    /// A frame contained a non-finite observation.
    #[error("non-finite {field} observation")]
    NonFiniteObservation {
        /// Measurement field.
        field: &'static str,
    },
    /// A modular wheel count change exceeded the physical bound.
    #[error("{sensor} count delta {delta_counts} exceeds limit {maximum_counts}")]
    ImplausibleCounterDelta {
        /// Stable sensor role.
        sensor: &'static str,
        /// Reconstructed signed count change.
        delta_counts: i64,
        /// Configured absolute limit.
        maximum_counts: u64,
    },
    /// Steering geometry reached a singular or non-physical denominator.
    #[error("steering geometry is singular")]
    SingularSteeringGeometry,
}

#[derive(Clone, Copy, Debug)]
struct AcceptedInputs {
    wheel_raw_counts: [i64; 4],
    wheel_sequences: [u64; 4],
    steering_sequences: [u64; 2],
    imu_sequence: u64,
    capture_ticks: u64,
    oldest_capture_ticks: u64,
}

/// Deterministic Ackermann estimator with a strictly DataBus-only update boundary.
#[derive(Clone, Debug)]
pub struct AckermannImuOdometry {
    config: AckermannImuOdometryConfig,
    pose: PoseSample,
    pose_covariance: [[f64; 3]; 3],
    previous: Option<AcceptedInputs>,
}

impl AckermannImuOdometry {
    /// Creates an estimator at the supplied initial estimated pose.
    pub fn new(
        config: AckermannImuOdometryConfig,
        initial_pose: PoseSample,
    ) -> Result<Self, AckermannImuOdometryError> {
        config.validate()?;
        if !initial_pose.position_m.x.is_finite()
            || !initial_pose.position_m.y.is_finite()
            || !initial_pose.yaw_rad.is_finite()
        {
            return Err(AckermannImuOdometryError::InvalidConfig("initial_pose"));
        }
        Ok(Self {
            config,
            pose: initial_pose,
            pose_covariance: [[0.0; 3]; 3],
            previous: None,
        })
    }

    /// Returns the latest estimate without exposing simulation state.
    pub const fn pose(&self) -> PoseSample {
        self.pose
    }

    /// Incorporates the newest synchronized sensor frames available at `decision_time`.
    ///
    /// No command, ECS world, rigid-body state, physics backend, or truth value is
    /// accepted by this API. Calibration, quantization, faults, and transport timing
    /// must already be represented by the supplied DataBus frames.
    pub fn update(
        &mut self,
        bus: &impl DataBus,
        streams: AckermannEncoderStreams,
        decision_time: SimTime,
    ) -> Result<AckermannImuOdometryEstimate, AckermannImuOdometryError> {
        let wheel_roles = [
            "front-left wheel",
            "rear-left wheel",
            "front-right wheel",
            "rear-right wheel",
        ];
        let steering_roles = ["front-left steering", "front-right steering"];
        let wheel_frames = std::array::from_fn(|index| {
            required::<IncrementalEncoderFeedback>(
                bus,
                streams.wheel_streams()[index],
                decision_time,
                wheel_roles[index],
            )
        });
        let wheel_frames = collect_array(wheel_frames)?;
        let steering_frames = std::array::from_fn(|index| {
            required::<IncrementalEncoderFeedback>(
                bus,
                streams.steering_streams()[index],
                decision_time,
                steering_roles[index],
            )
        });
        let steering_frames = collect_pair(steering_frames)?;
        let imu = required::<ImuFeedback>(bus, streams.imu, decision_time, "IMU")?;

        for (frame, role) in wheel_frames.iter().zip(wheel_roles) {
            validate_encoder_status(frame.payload.status, role)?;
            if !frame.payload.position_rad.is_finite() {
                return Err(AckermannImuOdometryError::NonFiniteObservation {
                    field: "wheel encoder position",
                });
            }
        }
        for (frame, role) in steering_frames.iter().zip(steering_roles) {
            validate_encoder_status(frame.payload.status, role)?;
            if !frame.payload.position_rad.is_finite() {
                return Err(AckermannImuOdometryError::NonFiniteObservation {
                    field: "steering encoder position",
                });
            }
        }
        if imu.payload.status == ImuFeedbackStatus::StuckValue {
            return Err(AckermannImuOdometryError::StuckValue { sensor: "IMU" });
        }
        if !imu.payload.angular_velocity_rad_s.z.is_finite() {
            return Err(AckermannImuOdometryError::NonFiniteObservation {
                field: "IMU yaw rate",
            });
        }

        let mut captures = [0_u64; 7];
        for (target, frame) in captures[..4].iter_mut().zip(&wheel_frames) {
            *target = frame.capture_time.ticks();
        }
        for (target, frame) in captures[4..6].iter_mut().zip(&steering_frames) {
            *target = frame.capture_time.ticks();
        }
        captures[6] = imu.capture_time.ticks();
        let oldest_capture_ticks = *captures.iter().min().expect("seven captures");
        let capture_ticks = *captures.iter().max().expect("seven captures");
        let skew_ticks = capture_ticks - oldest_capture_ticks;
        if skew_ticks > self.config.max_input_skew_ticks {
            return Err(AckermannImuOdometryError::InputSkew {
                observed_ticks: skew_ticks,
                maximum_ticks: self.config.max_input_skew_ticks,
            });
        }
        let max_age_ticks = decision_time.ticks().saturating_sub(oldest_capture_ticks);
        if max_age_ticks > self.config.max_frame_age_ticks {
            return Err(AckermannImuOdometryError::StaleInput {
                observed_ticks: max_age_ticks,
                maximum_ticks: self.config.max_frame_age_ticks,
            });
        }

        let accepted = AcceptedInputs {
            wheel_raw_counts: wheel_frames.each_ref().map(|frame| frame.payload.raw_count),
            wheel_sequences: wheel_frames.each_ref().map(|frame| frame.sequence),
            steering_sequences: steering_frames.each_ref().map(|frame| frame.sequence),
            imu_sequence: imu.sequence,
            capture_ticks,
            oldest_capture_ticks,
        };
        let steering_rad = steering_frames
            .each_ref()
            .map(|frame| frame.payload.position_rad);
        let (curvature_per_m, steering_disagreement_per_m) =
            steering_curvature(self.config, steering_rad)?;

        let Some(previous) = self.previous else {
            self.previous = Some(accepted);
            return Ok(self.estimate(
                AckermannImuOdometryHealth::Initializing,
                accepted,
                decision_time,
                0,
                0.0,
                0.0,
                curvature_per_m,
                0.0,
                0.0,
            ));
        };
        if accepted
            .wheel_sequences
            .iter()
            .zip(previous.wheel_sequences)
            .any(|(current, prior)| *current <= prior)
            || accepted
                .steering_sequences
                .iter()
                .zip(previous.steering_sequences)
                .any(|(current, prior)| *current <= prior)
            || accepted.imu_sequence <= previous.imu_sequence
        {
            return Err(AckermannImuOdometryError::NoNewSensorSet);
        }
        let dt_ticks = capture_ticks
            .checked_sub(previous.capture_ticks)
            .filter(|ticks| *ticks > 0)
            .ok_or(AckermannImuOdometryError::NonAdvancingCaptureTime)?;
        let dt_s = dt_ticks as f64 / 1_000_000_000.0;

        let mut wheel_distance_m = [0.0; 4];
        for index in 0..4 {
            let counts = modular_counter_delta(
                previous.wheel_raw_counts[index],
                accepted.wheel_raw_counts[index],
                self.config.wheel_counter_bits[index],
            ) * i64::from(self.config.wheel_direction[index]);
            if counts.unsigned_abs() > self.config.max_abs_wheel_delta_counts {
                return Err(AckermannImuOdometryError::ImplausibleCounterDelta {
                    sensor: wheel_roles[index],
                    delta_counts: counts,
                    maximum_counts: self.config.max_abs_wheel_delta_counts,
                });
            }
            wheel_distance_m[index] = counts as f64 * TAU * self.config.wheel_radius_m
                / f64::from(self.config.wheel_counts_per_revolution[index]);
        }
        let center_distance_m =
            rear_axle_distance(self.config, wheel_distance_m, steering_rad, curvature_per_m)?;
        let encoder_delta_yaw_rad = center_distance_m * curvature_per_m;
        let gyro_rate_rad_s = self.config.gyro_z_direction
            * (imu.payload.angular_velocity_rad_s.z - self.config.gyro_z_bias_rad_s);
        let gyro_delta_yaw_rad = gyro_rate_rad_s * dt_s;
        let yaw_innovation_rad = wrap_angle(encoder_delta_yaw_rad - gyro_delta_yaw_rad);
        let skipped_sequences = sequence_gaps(previous, accepted);
        let (health, gyro_weight) = if imu.payload.status == ImuFeedbackStatus::Saturated {
            (AckermannImuOdometryHealth::ImuSaturated, 0.0)
        } else if steering_disagreement_per_m > self.config.steering_curvature_disagreement_per_m {
            (
                AckermannImuOdometryHealth::SteeringDisagreement,
                self.config.disagreement_gyro_yaw_weight,
            )
        } else if yaw_innovation_rad.abs() > self.config.disagreement_threshold_rad {
            (
                AckermannImuOdometryHealth::WheelImuDisagreement,
                self.config.disagreement_gyro_yaw_weight,
            )
        } else if skipped_sequences > 0 {
            (
                AckermannImuOdometryHealth::InputSequenceGap,
                self.config.gyro_yaw_weight,
            )
        } else {
            (
                AckermannImuOdometryHealth::Nominal,
                self.config.gyro_yaw_weight,
            )
        };
        let delta_yaw_rad =
            encoder_delta_yaw_rad * (1.0 - gyro_weight) + gyro_delta_yaw_rad * gyro_weight;
        let midpoint_yaw_rad = self.pose.yaw_rad + 0.5 * delta_yaw_rad;
        self.pose.position_m.x += center_distance_m * midpoint_yaw_rad.cos();
        self.pose.position_m.y += center_distance_m * midpoint_yaw_rad.sin();
        self.pose.yaw_rad = wrap_angle(self.pose.yaw_rad + delta_yaw_rad);
        let yaw_variance = (1.0 - gyro_weight).powi(2) * self.config.encoder_yaw_std_rad.powi(2)
            + gyro_weight.powi(2) * (self.config.gyro_rate_std_rad_s * dt_s).powi(2);
        self.pose_covariance[0][0] += self.config.wheel_distance_std_m.powi(2);
        self.pose_covariance[1][1] += self.config.wheel_distance_std_m.powi(2);
        self.pose_covariance[2][2] += yaw_variance;
        self.previous = Some(accepted);

        Ok(self.estimate(
            health,
            accepted,
            decision_time,
            skipped_sequences,
            center_distance_m / dt_s,
            delta_yaw_rad / dt_s,
            curvature_per_m,
            encoder_delta_yaw_rad,
            gyro_delta_yaw_rad,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn estimate(
        &self,
        health: AckermannImuOdometryHealth,
        accepted: AcceptedInputs,
        decision_time: SimTime,
        skipped_sequences: u64,
        linear_velocity_m_s: f64,
        angular_velocity_rad_s: f64,
        curvature_per_m: f64,
        encoder_delta_yaw_rad: f64,
        gyro_delta_yaw_rad: f64,
    ) -> AckermannImuOdometryEstimate {
        AckermannImuOdometryEstimate {
            pose: self.pose,
            linear_velocity_m_s,
            angular_velocity_rad_s,
            curvature_per_m,
            encoder_delta_yaw_rad,
            gyro_delta_yaw_rad,
            yaw_innovation_rad: wrap_angle(encoder_delta_yaw_rad - gyro_delta_yaw_rad),
            health,
            provenance: AckermannImuOdometryProvenance {
                wheel_sequences: accepted.wheel_sequences,
                steering_sequences: accepted.steering_sequences,
                imu_sequence: accepted.imu_sequence,
                capture_ticks: accepted.capture_ticks,
                decision_ticks: decision_time.ticks(),
                max_age_ticks: decision_time
                    .ticks()
                    .saturating_sub(accepted.oldest_capture_ticks),
                skipped_sequences,
            },
            pose_covariance: self.pose_covariance,
        }
    }
}

fn steering_curvature(
    config: AckermannImuOdometryConfig,
    steering_rad: [f64; 2],
) -> Result<(f64, f64), AckermannImuOdometryError> {
    let half_track_m = 0.5 * config.track_width_m;
    let tangents = steering_rad.map(f64::tan);
    let denominators = [
        config.wheelbase_m - half_track_m * tangents[0],
        config.wheelbase_m + half_track_m * tangents[1],
    ];
    if denominators
        .iter()
        .any(|value| !value.is_finite() || value.abs() < 1.0e-9)
    {
        return Err(AckermannImuOdometryError::SingularSteeringGeometry);
    }
    let curvatures = [tangents[0] / denominators[0], tangents[1] / denominators[1]];
    if curvatures.iter().any(|value| !value.is_finite()) {
        return Err(AckermannImuOdometryError::SingularSteeringGeometry);
    }
    Ok((
        0.5 * (curvatures[0] + curvatures[1]),
        (curvatures[0] - curvatures[1]).abs(),
    ))
}

fn rear_axle_distance(
    config: AckermannImuOdometryConfig,
    wheel_distance_m: [f64; 4],
    steering_rad: [f64; 2],
    curvature_per_m: f64,
) -> Result<f64, AckermannImuOdometryError> {
    let half_track_m = 0.5 * config.track_width_m;
    let factors = [
        steering_rad[0].cos() * (1.0 + curvature_per_m * half_track_m)
            + curvature_per_m * config.wheelbase_m * steering_rad[0].sin(),
        1.0 + curvature_per_m * half_track_m,
        steering_rad[1].cos() * (1.0 - curvature_per_m * half_track_m)
            + curvature_per_m * config.wheelbase_m * steering_rad[1].sin(),
        1.0 - curvature_per_m * half_track_m,
    ];
    if factors
        .iter()
        .any(|value| !value.is_finite() || value.abs() < 1.0e-6)
    {
        return Err(AckermannImuOdometryError::SingularSteeringGeometry);
    }
    Ok(wheel_distance_m
        .iter()
        .zip(factors)
        .map(|(distance, factor)| distance / factor)
        .sum::<f64>()
        / 4.0)
}

fn required<T: rne_data::FramePayload>(
    bus: &impl DataBus,
    stream: StreamId,
    now: SimTime,
    payload: &'static str,
) -> Result<Frame<T>, AckermannImuOdometryError> {
    bus.latest_available::<T>(stream, now)
        .ok_or(AckermannImuOdometryError::MissingAvailableFrame {
            payload,
            stream_id: stream.0,
        })
}

fn collect_array<T, E>(values: [Result<T, E>; 4]) -> Result<[T; 4], E> {
    let [a, b, c, d] = values;
    Ok([a?, b?, c?, d?])
}

fn collect_pair<T, E>(values: [Result<T, E>; 2]) -> Result<[T; 2], E> {
    let [a, b] = values;
    Ok([a?, b?])
}

fn validate_encoder_status(
    status: IncrementalEncoderStatus,
    sensor: &'static str,
) -> Result<(), AckermannImuOdometryError> {
    match status {
        IncrementalEncoderStatus::Initializing | IncrementalEncoderStatus::Nominal => Ok(()),
        IncrementalEncoderStatus::CounterSaturated => {
            Err(AckermannImuOdometryError::EncoderSaturated { sensor })
        }
        IncrementalEncoderStatus::StuckValue => {
            Err(AckermannImuOdometryError::StuckValue { sensor })
        }
    }
}

fn modular_counter_delta(previous: i64, current: i64, bits: u8) -> i64 {
    let modulus = 1_i128 << bits;
    let half = modulus / 2;
    let mut delta = i128::from(current) - i128::from(previous);
    if delta >= half {
        delta -= modulus;
    } else if delta < -half {
        delta += modulus;
    }
    delta as i64
}

fn sequence_gap(previous: u64, current: u64) -> u64 {
    current.saturating_sub(previous).saturating_sub(1)
}

fn sequence_gaps(previous: AcceptedInputs, current: AcceptedInputs) -> u64 {
    previous
        .wheel_sequences
        .iter()
        .zip(current.wheel_sequences)
        .map(|(prior, next)| sequence_gap(*prior, next))
        .chain(
            previous
                .steering_sequences
                .iter()
                .zip(current.steering_sequences)
                .map(|(prior, next)| sequence_gap(*prior, next)),
        )
        .chain(std::iter::once(sequence_gap(
            previous.imu_sequence,
            current.imu_sequence,
        )))
        .sum()
}

fn wrap_angle(angle_rad: f64) -> f64 {
    (angle_rad + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use rne_core::SimDuration;
    use rne_data::{Frame, InMemoryDataBus};
    use rne_ecs::Entity;
    use rne_math::Vec3;

    const STREAMS: AckermannEncoderStreams = AckermannEncoderStreams {
        front_left_wheel: StreamId(101),
        rear_left_wheel: StreamId(102),
        front_right_wheel: StreamId(103),
        rear_right_wheel: StreamId(104),
        front_left_steering: StreamId(105),
        front_right_steering: StreamId(106),
        imu: StreamId(107),
    };

    fn config() -> AckermannImuOdometryConfig {
        AckermannImuOdometryConfig {
            wheel_radius_m: 0.25,
            wheelbase_m: 2.0,
            track_width_m: 1.2,
            wheel_counts_per_revolution: [1_000; 4],
            wheel_counter_bits: [16; 4],
            wheel_direction: [1; 4],
            max_abs_wheel_delta_counts: 10_000,
            gyro_z_direction: 1.0,
            gyro_z_bias_rad_s: 0.01,
            gyro_yaw_weight: 0.5,
            disagreement_gyro_yaw_weight: 0.9,
            disagreement_threshold_rad: 0.1,
            steering_curvature_disagreement_per_m: 0.02,
            wheel_distance_std_m: 0.001,
            encoder_yaw_std_rad: 0.002,
            gyro_rate_std_rad_s: 0.01,
            max_input_skew_ticks: 0,
            max_frame_age_ticks: 5_000_000,
        }
    }

    fn publish_set(
        bus: &mut InMemoryDataBus,
        sequence: u64,
        capture_ticks: u64,
        latency_ticks: u64,
        wheel_counts: [i64; 4],
        steering_rad: [f64; 2],
        gyro_rad_s: f64,
    ) {
        for (stream, count) in STREAMS.wheel_streams().into_iter().zip(wheel_counts) {
            publish_encoder(
                bus,
                stream,
                sequence,
                capture_ticks,
                latency_ticks,
                count,
                0.0,
            );
        }
        for (stream, position) in STREAMS.steering_streams().into_iter().zip(steering_rad) {
            publish_encoder(
                bus,
                stream,
                sequence,
                capture_ticks,
                latency_ticks,
                0,
                position,
            );
        }
        bus.publish(
            Frame::new(
                STREAMS.imu,
                Entity::PLACEHOLDER,
                sequence,
                SimTime::from_ticks(capture_ticks),
                ImuFeedback {
                    schema_version: ImuFeedback::SCHEMA_VERSION,
                    angular_velocity_rad_s: Vec3::new(0.0, 0.0, gyro_rad_s),
                    ..ImuFeedback::default()
                },
            )
            .with_latency(SimDuration::from_ticks(latency_ticks)),
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn publish_encoder(
        bus: &mut InMemoryDataBus,
        stream: StreamId,
        sequence: u64,
        capture_ticks: u64,
        latency_ticks: u64,
        raw_count: i64,
        position_rad: f64,
    ) {
        bus.publish(
            Frame::new(
                stream,
                Entity::PLACEHOLDER,
                sequence,
                SimTime::from_ticks(capture_ticks),
                IncrementalEncoderFeedback {
                    schema_version: IncrementalEncoderFeedback::SCHEMA_VERSION,
                    status: if sequence == 1 {
                        IncrementalEncoderStatus::Initializing
                    } else {
                        IncrementalEncoderStatus::Nominal
                    },
                    raw_count,
                    position_rad,
                    ..IncrementalEncoderFeedback::default()
                },
            )
            .with_latency(SimDuration::from_ticks(latency_ticks)),
        );
    }

    #[test]
    fn config_rejects_nonphysical_geometry_and_counter_limits() {
        let mut invalid = config();
        invalid.wheelbase_m = 0.0;
        assert_eq!(
            invalid.validate(),
            Err(AckermannImuOdometryError::InvalidConfig("wheelbase_m"))
        );
        let mut invalid = config();
        invalid.max_abs_wheel_delta_counts = 1_u64 << 15;
        assert_eq!(
            invalid.validate(),
            Err(AckermannImuOdometryError::InvalidConfig(
                "max_abs_wheel_delta_counts"
            ))
        );
    }

    #[test]
    fn straight_motion_is_reconstructed_from_quantized_counts() {
        let mut bus = InMemoryDataBus::new();
        let mut estimator = AckermannImuOdometry::new(config(), PoseSample::default()).unwrap();
        publish_set(&mut bus, 1, 0, 2, [0; 4], [0.0; 2], 0.01);
        let initial = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(2))
            .unwrap();
        assert_eq!(initial.health, AckermannImuOdometryHealth::Initializing);
        publish_set(&mut bus, 2, 10_000_000, 2, [100; 4], [0.0; 2], 0.01);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_002))
            .unwrap();
        assert_eq!(estimate.health, AckermannImuOdometryHealth::Nominal);
        assert_abs_diff_eq!(estimate.pose.position_m.x, 0.05 * PI, epsilon = 1.0e-12);
        assert_abs_diff_eq!(estimate.pose.position_m.y, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(estimate.angular_velocity_rad_s, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn ackermann_geometry_and_imu_produce_a_turn_without_truth_input() {
        let mut bus = InMemoryDataBus::new();
        let mut estimator = AckermannImuOdometry::new(config(), PoseSample::default()).unwrap();
        let curvature = 0.2;
        let half_track = 0.5 * config().track_width_m;
        let steering = [
            (config().wheelbase_m * curvature / (1.0 + curvature * half_track)).atan(),
            (config().wheelbase_m * curvature / (1.0 - curvature * half_track)).atan(),
        ];
        publish_set(&mut bus, 1, 0, 0, [0; 4], steering, 0.01 + 0.2);
        estimator.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        let center_distance_m = 0.10;
        let wheel_distances = [
            center_distance_m
                * (steering[0].cos() * (1.0 + curvature * half_track)
                    + curvature * config().wheelbase_m * steering[0].sin()),
            center_distance_m * (1.0 + curvature * half_track),
            center_distance_m
                * (steering[1].cos() * (1.0 - curvature * half_track)
                    + curvature * config().wheelbase_m * steering[1].sin()),
            center_distance_m * (1.0 - curvature * half_track),
        ];
        let counts = wheel_distances
            .map(|distance| (distance * 1_000.0 / (TAU * config().wheel_radius_m)).round() as i64);
        publish_set(&mut bus, 2, 100_000_000, 0, counts, steering, 0.01 + 0.2);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(100_000_000))
            .unwrap();
        assert_abs_diff_eq!(estimate.curvature_per_m, curvature, epsilon = 1.0e-12);
        assert!(estimate.pose.position_m.x > 0.09);
        assert!(estimate.pose.position_m.y > 0.0);
        assert_abs_diff_eq!(estimate.angular_velocity_rad_s, 0.2, epsilon = 0.01);
    }

    #[test]
    fn availability_time_and_stuck_status_fail_closed() {
        let mut bus = InMemoryDataBus::new();
        let mut estimator = AckermannImuOdometry::new(config(), PoseSample::default()).unwrap();
        publish_set(&mut bus, 1, 10, 5, [0; 4], [0.0; 2], 0.01);
        assert!(matches!(
            estimator.update(&bus, STREAMS, SimTime::from_ticks(14)),
            Err(AckermannImuOdometryError::MissingAvailableFrame { .. })
        ));
        estimator
            .update(&bus, STREAMS, SimTime::from_ticks(15))
            .unwrap();
        let stuck = IncrementalEncoderFeedback {
            schema_version: IncrementalEncoderFeedback::SCHEMA_VERSION,
            status: IncrementalEncoderStatus::StuckValue,
            ..IncrementalEncoderFeedback::default()
        };
        bus.publish(Frame::new(
            STREAMS.front_left_steering,
            Entity::PLACEHOLDER,
            2,
            SimTime::from_ticks(20),
            stuck,
        ));
        assert_eq!(
            estimator.update(&bus, STREAMS, SimTime::from_ticks(20)),
            Err(AckermannImuOdometryError::StuckValue {
                sensor: "front-left steering"
            })
        );
    }

    #[test]
    fn identical_databus_history_produces_identical_estimates() {
        let mut first_bus = InMemoryDataBus::new();
        let mut second_bus = InMemoryDataBus::new();
        let mut first = AckermannImuOdometry::new(config(), PoseSample::default()).unwrap();
        let mut second = AckermannImuOdometry::new(config(), PoseSample::default()).unwrap();
        for sequence in 1..=8 {
            let ticks = (sequence - 1) * 10_000_000;
            let counts = [sequence as i64 * 7; 4];
            for bus in [&mut first_bus, &mut second_bus] {
                publish_set(bus, sequence, ticks, 3, counts, [0.08, 0.09], 0.05);
            }
            let now = SimTime::from_ticks(ticks + 3);
            assert_eq!(
                first.update(&first_bus, STREAMS, now),
                second.update(&second_bus, STREAMS, now)
            );
        }
    }
}
