//! Sensor-only planar odometry from incremental wheel encoders and a raw IMU.

use crate::task::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_core::SimTime;
use rne_data::{
    DataBus, Frame, ImuFeedback, ImuFeedbackStatus, IncrementalEncoderFeedback,
    IncrementalEncoderStatus, PoseSample, StreamId,
};
use std::f64::consts::{PI, TAU};
use thiserror::Error;

/// Stable task identity for the sensor-only differential-drive baseline.
pub const WHEEL_IMU_SENSOR_ONLY_TASK_ID: &str = "rne.mobility.wheel_imu_sensor_only.v1";

/// DataBus streams consumed by [`WheelImuOdometry`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WheelImuOdometryStreams {
    /// Left incremental-encoder stream.
    pub left_encoder: StreamId,
    /// Right incremental-encoder stream.
    pub right_encoder: StreamId,
    /// Raw IMU-feedback stream.
    pub imu: StreamId,
}

/// Physical calibration and timing limits for sensor-only odometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelImuOdometryConfig {
    /// Radius of each driven wheel in meters.
    pub wheel_radius_m: f64,
    /// Lateral distance between wheel contact centers in meters.
    pub track_width_m: f64,
    /// Left encoder counts per mechanical revolution.
    pub left_counts_per_revolution: u32,
    /// Right encoder counts per mechanical revolution.
    pub right_counts_per_revolution: u32,
    /// Width of the signed left hardware counter in bits, from 2 through 63.
    pub left_counter_bits: u8,
    /// Width of the signed right hardware counter in bits, from 2 through 63.
    pub right_counter_bits: u8,
    /// Left encoder direction, exactly `-1` or `1`.
    pub left_direction: i8,
    /// Right encoder direction, exactly `-1` or `1`.
    pub right_direction: i8,
    /// IMU yaw-axis direction, exactly `-1.0` or `1.0`.
    pub gyro_z_direction: f64,
    /// Largest plausible absolute wheel-count change per accepted update.
    ///
    /// This must be below half of both counter ranges so a long sequence gap
    /// cannot silently alias physical motion into the opposite direction.
    pub max_abs_wheel_delta_counts: u64,
    /// Calibrated stationary gyroscope bias in radians per second.
    pub gyro_z_bias_rad_s: f64,
    /// One-standard-deviation center-distance uncertainty per update, in meters.
    pub wheel_distance_std_m: f64,
    /// One-standard-deviation encoder yaw-increment uncertainty per update, in radians.
    pub encoder_yaw_std_rad: f64,
    /// One-standard-deviation gyroscope yaw-rate uncertainty, in radians per second.
    pub gyro_rate_std_rad_s: f64,
    /// Normal fusion weight assigned to the gyro yaw increment in `[0, 1]`.
    pub gyro_yaw_weight: f64,
    /// Gyro weight used after wheel/gyro disagreement in `[gyro_yaw_weight, 1]`.
    pub disagreement_gyro_yaw_weight: f64,
    /// Absolute encoder/gyro yaw-increment disagreement that flags likely slip.
    pub disagreement_threshold_rad: f64,
    /// Maximum capture-time separation accepted among the three inputs.
    pub max_input_skew_ticks: u64,
    /// Maximum measurement age at the estimator decision time.
    pub max_frame_age_ticks: u64,
}

impl WheelImuOdometryConfig {
    /// Validates physical, calibration, and timing invariants.
    pub fn validate(self) -> Result<(), WheelImuOdometryError> {
        if !self.wheel_radius_m.is_finite() || self.wheel_radius_m <= 0.0 {
            return Err(WheelImuOdometryError::InvalidConfig("wheel_radius_m"));
        }
        if !self.track_width_m.is_finite() || self.track_width_m <= 0.0 {
            return Err(WheelImuOdometryError::InvalidConfig("track_width_m"));
        }
        if self.left_counts_per_revolution == 0 {
            return Err(WheelImuOdometryError::InvalidConfig(
                "left_counts_per_revolution",
            ));
        }
        if self.right_counts_per_revolution == 0 {
            return Err(WheelImuOdometryError::InvalidConfig(
                "right_counts_per_revolution",
            ));
        }
        if !(2..=63).contains(&self.left_counter_bits) {
            return Err(WheelImuOdometryError::InvalidConfig("left_counter_bits"));
        }
        if !(2..=63).contains(&self.right_counter_bits) {
            return Err(WheelImuOdometryError::InvalidConfig("right_counter_bits"));
        }
        if !matches!(self.left_direction, -1 | 1) {
            return Err(WheelImuOdometryError::InvalidConfig("left_direction"));
        }
        if !matches!(self.right_direction, -1 | 1) {
            return Err(WheelImuOdometryError::InvalidConfig("right_direction"));
        }
        if !matches!(self.gyro_z_direction, -1.0 | 1.0) {
            return Err(WheelImuOdometryError::InvalidConfig("gyro_z_direction"));
        }
        let left_half_range = 1_u64 << (self.left_counter_bits - 1);
        let right_half_range = 1_u64 << (self.right_counter_bits - 1);
        if self.max_abs_wheel_delta_counts == 0
            || self.max_abs_wheel_delta_counts >= left_half_range
            || self.max_abs_wheel_delta_counts >= right_half_range
        {
            return Err(WheelImuOdometryError::InvalidConfig(
                "max_abs_wheel_delta_counts",
            ));
        }
        for (name, value) in [
            ("gyro_z_bias_rad_s", self.gyro_z_bias_rad_s),
            ("wheel_distance_std_m", self.wheel_distance_std_m),
            ("encoder_yaw_std_rad", self.encoder_yaw_std_rad),
            ("gyro_rate_std_rad_s", self.gyro_rate_std_rad_s),
            ("gyro_yaw_weight", self.gyro_yaw_weight),
            (
                "disagreement_gyro_yaw_weight",
                self.disagreement_gyro_yaw_weight,
            ),
            (
                "disagreement_threshold_rad",
                self.disagreement_threshold_rad,
            ),
        ] {
            if !value.is_finite() {
                return Err(WheelImuOdometryError::InvalidConfig(name));
            }
        }
        if !(0.0..=1.0).contains(&self.gyro_yaw_weight) {
            return Err(WheelImuOdometryError::InvalidConfig("gyro_yaw_weight"));
        }
        if !(self.gyro_yaw_weight..=1.0).contains(&self.disagreement_gyro_yaw_weight) {
            return Err(WheelImuOdometryError::InvalidConfig(
                "disagreement_gyro_yaw_weight",
            ));
        }
        if self.disagreement_threshold_rad < 0.0 {
            return Err(WheelImuOdometryError::InvalidConfig(
                "disagreement_threshold_rad",
            ));
        }
        if self.wheel_distance_std_m < 0.0
            || self.encoder_yaw_std_rad < 0.0
            || self.gyro_rate_std_rad_s < 0.0
        {
            return Err(WheelImuOdometryError::InvalidConfig(
                "measurement uncertainty",
            ));
        }
        Ok(())
    }
}

/// Health classification attached to each odometry estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WheelImuOdometryHealth {
    /// First synchronized sample establishes counter and time baselines.
    Initializing,
    /// All inputs were nominal and mutually consistent.
    Nominal,
    /// At least one source sequence skipped while retained counts remained usable.
    InputSequenceGap,
    /// The gyro was saturated, so the update used encoder yaw only.
    ImuSaturated,
    /// Wheel and gyro increments disagreed beyond the configured slip threshold.
    WheelImuDisagreement,
}

/// Input provenance retained with one sensor-only odometry estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WheelImuOdometryProvenance {
    /// Left encoder sequence.
    pub left_sequence: u64,
    /// Right encoder sequence.
    pub right_sequence: u64,
    /// IMU sequence.
    pub imu_sequence: u64,
    /// Effective capture time, the newest capture among synchronized inputs.
    pub capture_ticks: u64,
    /// Estimator decision time.
    pub decision_ticks: u64,
    /// Maximum input age at the decision time.
    pub max_age_ticks: u64,
    /// Number of sequences skipped since the preceding accepted update.
    pub skipped_sequences: u64,
}

/// Planar pose, twist, fusion evidence, and exact source provenance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelImuOdometryEstimate {
    /// Estimated localization pose, never simulator truth.
    pub pose: PoseSample,
    /// Estimated forward velocity in meters per second.
    pub linear_velocity_m_s: f64,
    /// Estimated yaw rate in radians per second.
    pub angular_velocity_rad_s: f64,
    /// Encoder-only yaw increment for this update.
    pub encoder_delta_yaw_rad: f64,
    /// Bias-corrected gyro yaw increment for this update.
    pub gyro_delta_yaw_rad: f64,
    /// Wrapped encoder-minus-gyro yaw innovation.
    pub yaw_innovation_rad: f64,
    /// Sample health classification.
    pub health: WheelImuOdometryHealth,
    /// Input frame timing and sequence evidence.
    pub provenance: WheelImuOdometryProvenance,
    /// Propagated covariance of `[x_m, y_m, yaw_rad]` in row-major order.
    pub pose_covariance: [[f64; 3]; 3],
}

/// Policy-visible numerical observation derived only from an odometry estimate and task goal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelImuActorObservation {
    /// Estimated planar position in meters.
    pub estimated_position_m: [f64; 2],
    /// Estimated yaw in radians.
    pub estimated_yaw_rad: f64,
    /// Estimated forward speed in meters per second.
    pub estimated_linear_velocity_m_s: f64,
    /// Estimated yaw rate in radians per second.
    pub estimated_angular_velocity_rad_s: f64,
    /// Estimated planar position covariance block in square meters.
    pub position_covariance_m2: [[f64; 2]; 2],
    /// Estimated yaw variance in square radians.
    pub yaw_variance_rad2: f64,
    /// Wrapped encoder-minus-gyro yaw innovation in radians.
    pub yaw_innovation_rad: f64,
    /// Maximum source-frame age at the decision time, in seconds.
    pub max_input_age_s: f64,
    /// Total source sequence numbers skipped since the preceding update.
    pub skipped_sequences: u64,
    /// Stable integer code for [`WheelImuOdometryHealth`].
    pub health_code: u8,
    /// Task goal minus estimated position in meters.
    pub goal_delta_m: [f64; 2],
}

impl WheelImuActorObservation {
    /// Builds a policy observation without accepting simulator truth.
    pub fn from_estimate(estimate: WheelImuOdometryEstimate, goal_position_m: [f64; 2]) -> Self {
        Self {
            estimated_position_m: [estimate.pose.position_m.x, estimate.pose.position_m.y],
            estimated_yaw_rad: estimate.pose.yaw_rad,
            estimated_linear_velocity_m_s: estimate.linear_velocity_m_s,
            estimated_angular_velocity_rad_s: estimate.angular_velocity_rad_s,
            position_covariance_m2: [
                [
                    estimate.pose_covariance[0][0],
                    estimate.pose_covariance[0][1],
                ],
                [
                    estimate.pose_covariance[1][0],
                    estimate.pose_covariance[1][1],
                ],
            ],
            yaw_variance_rad2: estimate.pose_covariance[2][2],
            yaw_innovation_rad: estimate.yaw_innovation_rad,
            max_input_age_s: estimate.provenance.max_age_ticks as f64 / 1_000_000_000.0,
            skipped_sequences: estimate.provenance.skipped_sequences,
            health_code: health_code(estimate.health),
            goal_delta_m: [
                goal_position_m[0] - estimate.pose.position_m.x,
                goal_position_m[1] - estimate.pose.position_m.y,
            ],
        }
    }
}

/// Returns the portable actor contract for sensor-only mobile goal reaching.
///
/// Observation tensors contain only [`WheelImuActorObservation`] values. Ground
/// truth may be used by the runtime to score progress and termination, but it has
/// no tensor in this actor-visible contract. Actions are bounded motor terminal
/// voltages rather than privileged velocity realization.
pub fn wheel_imu_sensor_only_task_spec(
    max_episode_steps: u64,
    max_motor_voltage_v: f64,
) -> TaskSpec {
    TaskSpec::new(
        WHEEL_IMU_SENSOR_ONLY_TASK_ID,
        1.0 / 100.0,
        ObservationSpec::new(vec![
            TensorSpec::new("estimated_position_m", TensorDType::F64, vec![2], "m"),
            TensorSpec::new("estimated_yaw_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new(
                "estimated_linear_velocity_m_s",
                TensorDType::F64,
                vec![],
                "m/s",
            ),
            TensorSpec::new(
                "estimated_angular_velocity_rad_s",
                TensorDType::F64,
                vec![],
                "rad/s",
            ),
            TensorSpec::new(
                "position_covariance_m2",
                TensorDType::F64,
                vec![2, 2],
                "m^2",
            ),
            TensorSpec::new("yaw_variance_rad2", TensorDType::F64, vec![], "rad^2"),
            TensorSpec::new("yaw_innovation_rad", TensorDType::F64, vec![], "rad"),
            TensorSpec::new("max_input_age_s", TensorDType::F64, vec![], "s")
                .with_bounds(TensorBounds::broadcast(0.0, f64::MAX)),
            TensorSpec::new("skipped_sequences", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("health_code", TensorDType::U8, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 4.0)),
            TensorSpec::new("goal_delta_m", TensorDType::F64, vec![2], "m"),
        ]),
        ActionSpec::new(vec![TensorSpec::new(
            "motor_terminal_voltage_v",
            TensorDType::F64,
            vec![2],
            "V",
        )
        .with_bounds(TensorBounds::broadcast(
            -max_motor_voltage_v,
            max_motor_voltage_v,
        ))]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("truth_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("truth_goal_reached", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![TerminationConditionSpec::new(
                "truth_goal_reached",
                TerminationKind::Success,
            )],
            Some(max_episode_steps),
        ),
        ResetSpec::splitmix64(true),
    )
}

/// Failure to produce a valid sensor-only odometry update.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum WheelImuOdometryError {
    /// A configuration field violated its documented invariant.
    #[error("invalid wheel/IMU odometry configuration field: {0}")]
    InvalidConfig(&'static str),
    /// A required input had not arrived by the estimator decision time.
    #[error("no available {payload} frame on stream {stream_id}")]
    MissingAvailableFrame {
        /// Human-readable input role.
        payload: &'static str,
        /// Required stream identifier.
        stream_id: u64,
    },
    /// The newest available inputs were not synchronized within the configured bound.
    #[error("input capture skew {observed_ticks} ticks exceeds {maximum_ticks} ticks")]
    InputSkew {
        /// Observed newest-minus-oldest capture time.
        observed_ticks: u64,
        /// Configured maximum capture skew.
        maximum_ticks: u64,
    },
    /// An input was older than the configured estimator limit.
    #[error("input age {observed_ticks} ticks exceeds {maximum_ticks} ticks")]
    StaleInput {
        /// Oldest measurement age.
        observed_ticks: u64,
        /// Configured maximum age.
        maximum_ticks: u64,
    },
    /// A previously accepted encoder pair was offered again.
    #[error("no new synchronized encoder pair is available")]
    NoNewEncoderPair,
    /// Capture time did not advance, so velocity integration is undefined.
    #[error("capture time did not advance")]
    NonAdvancingCaptureTime,
    /// A sensor reported a held value that must not be integrated as fresh motion.
    #[error("{sensor} reported stuck-value status")]
    StuckValue {
        /// Sensor role.
        sensor: &'static str,
    },
    /// A saturated encoder counter cannot provide continuing displacement evidence.
    #[error("{sensor} encoder counter is saturated")]
    EncoderSaturated {
        /// Encoder role.
        sensor: &'static str,
    },
    /// A frame contained a non-finite numerical observation.
    #[error("non-finite {field} observation")]
    NonFiniteObservation {
        /// Measurement field.
        field: &'static str,
    },
    /// A modular count change exceeded the configured physically plausible bound.
    #[error("{sensor} count delta {delta_counts} exceeds limit {maximum_counts}")]
    ImplausibleCounterDelta {
        /// Encoder role.
        sensor: &'static str,
        /// Reconstructed signed count change.
        delta_counts: i64,
        /// Configured absolute limit.
        maximum_counts: u64,
    },
}

#[derive(Clone, Copy, Debug)]
struct AcceptedInputs {
    left_raw_count: i64,
    right_raw_count: i64,
    left_sequence: u64,
    right_sequence: u64,
    imu_sequence: u64,
    capture_ticks: u64,
    oldest_capture_ticks: u64,
}

/// Deterministic planar estimator whose public update boundary accepts only DataBus frames.
#[derive(Clone, Debug)]
pub struct WheelImuOdometry {
    config: WheelImuOdometryConfig,
    pose: PoseSample,
    pose_covariance: [[f64; 3]; 3],
    previous: Option<AcceptedInputs>,
}

impl WheelImuOdometry {
    /// Creates an estimator at the supplied initial estimate.
    pub fn new(
        config: WheelImuOdometryConfig,
        initial_pose: PoseSample,
    ) -> Result<Self, WheelImuOdometryError> {
        config.validate()?;
        if !initial_pose.position_m.x.is_finite()
            || !initial_pose.position_m.y.is_finite()
            || !initial_pose.yaw_rad.is_finite()
        {
            return Err(WheelImuOdometryError::InvalidConfig("initial_pose"));
        }
        Ok(Self {
            config,
            pose: initial_pose,
            pose_covariance: [[0.0; 3]; 3],
            previous: None,
        })
    }

    /// Returns the current estimated pose without reading simulation state.
    pub const fn pose(&self) -> PoseSample {
        self.pose
    }

    /// Incorporates the newest synchronized frames available at `decision_time`.
    ///
    /// The signature intentionally contains no ECS world, transform, rigid-body state,
    /// command, or ground-truth input. Transport latency is respected through
    /// [`DataBus::latest_available`].
    pub fn update(
        &mut self,
        bus: &impl DataBus,
        streams: WheelImuOdometryStreams,
        decision_time: SimTime,
    ) -> Result<WheelImuOdometryEstimate, WheelImuOdometryError> {
        let left = required::<IncrementalEncoderFeedback>(
            bus,
            streams.left_encoder,
            decision_time,
            "left encoder",
        )?;
        let right = required::<IncrementalEncoderFeedback>(
            bus,
            streams.right_encoder,
            decision_time,
            "right encoder",
        )?;
        let imu = required::<ImuFeedback>(bus, streams.imu, decision_time, "IMU")?;
        validate_encoder_status(left.payload.status, "left")?;
        validate_encoder_status(right.payload.status, "right")?;
        if imu.payload.status == ImuFeedbackStatus::StuckValue {
            return Err(WheelImuOdometryError::StuckValue { sensor: "IMU" });
        }
        for (field, value) in [
            ("left encoder position", left.payload.position_rad),
            ("right encoder position", right.payload.position_rad),
            ("IMU yaw rate", imu.payload.angular_velocity_rad_s.z),
        ] {
            if !value.is_finite() {
                return Err(WheelImuOdometryError::NonFiniteObservation { field });
            }
        }

        let captures = [
            left.capture_time.ticks(),
            right.capture_time.ticks(),
            imu.capture_time.ticks(),
        ];
        let oldest_capture = *captures.iter().min().expect("three captures");
        let capture_ticks = *captures.iter().max().expect("three captures");
        let skew_ticks = capture_ticks - oldest_capture;
        if skew_ticks > self.config.max_input_skew_ticks {
            return Err(WheelImuOdometryError::InputSkew {
                observed_ticks: skew_ticks,
                maximum_ticks: self.config.max_input_skew_ticks,
            });
        }
        let max_age_ticks = decision_time.ticks().saturating_sub(oldest_capture);
        if max_age_ticks > self.config.max_frame_age_ticks {
            return Err(WheelImuOdometryError::StaleInput {
                observed_ticks: max_age_ticks,
                maximum_ticks: self.config.max_frame_age_ticks,
            });
        }

        let accepted = AcceptedInputs {
            left_raw_count: left.payload.raw_count,
            right_raw_count: right.payload.raw_count,
            left_sequence: left.sequence,
            right_sequence: right.sequence,
            imu_sequence: imu.sequence,
            capture_ticks,
            oldest_capture_ticks: oldest_capture,
        };
        let Some(previous) = self.previous else {
            self.previous = Some(accepted);
            return Ok(self.estimate(
                WheelImuOdometryHealth::Initializing,
                accepted,
                decision_time,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
            ));
        };
        if left.sequence <= previous.left_sequence || right.sequence <= previous.right_sequence {
            return Err(WheelImuOdometryError::NoNewEncoderPair);
        }
        let dt_ticks = capture_ticks
            .checked_sub(previous.capture_ticks)
            .filter(|ticks| *ticks > 0)
            .ok_or(WheelImuOdometryError::NonAdvancingCaptureTime)?;
        let dt_s = dt_ticks as f64 / 1_000_000_000.0;

        let left_counts = modular_counter_delta(
            previous.left_raw_count,
            accepted.left_raw_count,
            self.config.left_counter_bits,
        ) * i64::from(self.config.left_direction);
        let right_counts = modular_counter_delta(
            previous.right_raw_count,
            accepted.right_raw_count,
            self.config.right_counter_bits,
        ) * i64::from(self.config.right_direction);
        validate_counter_delta(left_counts, self.config.max_abs_wheel_delta_counts, "left")?;
        validate_counter_delta(
            right_counts,
            self.config.max_abs_wheel_delta_counts,
            "right",
        )?;
        let left_distance_m = counts_to_distance_m(
            left_counts,
            self.config.left_counts_per_revolution,
            self.config.wheel_radius_m,
        );
        let right_distance_m = counts_to_distance_m(
            right_counts,
            self.config.right_counts_per_revolution,
            self.config.wheel_radius_m,
        );
        let center_distance_m = 0.5 * (left_distance_m + right_distance_m);
        let encoder_delta_yaw_rad =
            (right_distance_m - left_distance_m) / self.config.track_width_m;
        let gyro_rate_rad_s = self.config.gyro_z_direction
            * (imu.payload.angular_velocity_rad_s.z - self.config.gyro_z_bias_rad_s);
        let gyro_delta_yaw_rad = gyro_rate_rad_s * dt_s;
        let yaw_innovation_rad = wrap_angle(encoder_delta_yaw_rad - gyro_delta_yaw_rad);

        let skipped_sequences = sequence_gap(previous.left_sequence, left.sequence)
            + sequence_gap(previous.right_sequence, right.sequence)
            + sequence_gap(previous.imu_sequence, imu.sequence);
        let (health, gyro_weight) = if imu.payload.status == ImuFeedbackStatus::Saturated {
            (WheelImuOdometryHealth::ImuSaturated, 0.0)
        } else if yaw_innovation_rad.abs() > self.config.disagreement_threshold_rad {
            (
                WheelImuOdometryHealth::WheelImuDisagreement,
                self.config.disagreement_gyro_yaw_weight,
            )
        } else if skipped_sequences > 0 {
            (
                WheelImuOdometryHealth::InputSequenceGap,
                self.config.gyro_yaw_weight,
            )
        } else {
            (WheelImuOdometryHealth::Nominal, self.config.gyro_yaw_weight)
        };
        let delta_yaw_rad =
            encoder_delta_yaw_rad * (1.0 - gyro_weight) + gyro_delta_yaw_rad * gyro_weight;
        let midpoint_yaw_rad = self.pose.yaw_rad + 0.5 * delta_yaw_rad;
        self.pose.position_m.x += center_distance_m * midpoint_yaw_rad.cos();
        self.pose.position_m.y += center_distance_m * midpoint_yaw_rad.sin();
        self.pose.yaw_rad = wrap_angle(self.pose.yaw_rad + delta_yaw_rad);
        let yaw_variance = (1.0 - gyro_weight).powi(2) * self.config.encoder_yaw_std_rad.powi(2)
            + gyro_weight.powi(2) * (self.config.gyro_rate_std_rad_s * dt_s).powi(2);
        self.pose_covariance = propagate_pose_covariance(
            self.pose_covariance,
            midpoint_yaw_rad,
            center_distance_m,
            self.config.wheel_distance_std_m.powi(2),
            yaw_variance,
        );
        self.previous = Some(accepted);

        Ok(self.estimate(
            health,
            accepted,
            decision_time,
            skipped_sequences,
            center_distance_m / dt_s,
            delta_yaw_rad / dt_s,
            encoder_delta_yaw_rad,
            gyro_delta_yaw_rad,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn estimate(
        &self,
        health: WheelImuOdometryHealth,
        accepted: AcceptedInputs,
        decision_time: SimTime,
        skipped_sequences: u64,
        linear_velocity_m_s: f64,
        angular_velocity_rad_s: f64,
        encoder_delta_yaw_rad: f64,
        gyro_delta_yaw_rad: f64,
    ) -> WheelImuOdometryEstimate {
        WheelImuOdometryEstimate {
            pose: self.pose,
            linear_velocity_m_s,
            angular_velocity_rad_s,
            encoder_delta_yaw_rad,
            gyro_delta_yaw_rad,
            yaw_innovation_rad: wrap_angle(encoder_delta_yaw_rad - gyro_delta_yaw_rad),
            health,
            provenance: WheelImuOdometryProvenance {
                left_sequence: accepted.left_sequence,
                right_sequence: accepted.right_sequence,
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

fn required<T: rne_data::FramePayload>(
    bus: &impl DataBus,
    stream: StreamId,
    now: SimTime,
    payload: &'static str,
) -> Result<Frame<T>, WheelImuOdometryError> {
    bus.latest_available::<T>(stream, now)
        .ok_or(WheelImuOdometryError::MissingAvailableFrame {
            payload,
            stream_id: stream.0,
        })
}

fn validate_encoder_status(
    status: IncrementalEncoderStatus,
    sensor: &'static str,
) -> Result<(), WheelImuOdometryError> {
    match status {
        IncrementalEncoderStatus::Initializing | IncrementalEncoderStatus::Nominal => Ok(()),
        IncrementalEncoderStatus::CounterSaturated => {
            Err(WheelImuOdometryError::EncoderSaturated { sensor })
        }
        IncrementalEncoderStatus::StuckValue => Err(WheelImuOdometryError::StuckValue { sensor }),
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

fn validate_counter_delta(
    delta_counts: i64,
    maximum_counts: u64,
    sensor: &'static str,
) -> Result<(), WheelImuOdometryError> {
    if delta_counts.unsigned_abs() > maximum_counts {
        Err(WheelImuOdometryError::ImplausibleCounterDelta {
            sensor,
            delta_counts,
            maximum_counts,
        })
    } else {
        Ok(())
    }
}

fn counts_to_distance_m(counts: i64, counts_per_revolution: u32, wheel_radius_m: f64) -> f64 {
    counts as f64 * TAU * wheel_radius_m / f64::from(counts_per_revolution)
}

fn sequence_gap(previous: u64, current: u64) -> u64 {
    current.saturating_sub(previous).saturating_sub(1)
}

const fn health_code(health: WheelImuOdometryHealth) -> u8 {
    match health {
        WheelImuOdometryHealth::Initializing => 0,
        WheelImuOdometryHealth::Nominal => 1,
        WheelImuOdometryHealth::InputSequenceGap => 2,
        WheelImuOdometryHealth::ImuSaturated => 3,
        WheelImuOdometryHealth::WheelImuDisagreement => 4,
    }
}

fn wrap_angle(angle_rad: f64) -> f64 {
    (angle_rad + PI).rem_euclid(TAU) - PI
}

fn propagate_pose_covariance(
    covariance: [[f64; 3]; 3],
    midpoint_yaw_rad: f64,
    distance_m: f64,
    distance_variance_m2: f64,
    yaw_variance_rad2: f64,
) -> [[f64; 3]; 3] {
    let sin_yaw = midpoint_yaw_rad.sin();
    let cos_yaw = midpoint_yaw_rad.cos();
    let f = [
        [1.0, 0.0, -distance_m * sin_yaw],
        [0.0, 1.0, distance_m * cos_yaw],
        [0.0, 0.0, 1.0],
    ];
    let g = [
        [cos_yaw, -0.5 * distance_m * sin_yaw],
        [sin_yaw, 0.5 * distance_m * cos_yaw],
        [0.0, 1.0],
    ];
    let mut propagated = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            for first in 0..3 {
                for second in 0..3 {
                    propagated[row][column] +=
                        f[row][first] * covariance[first][second] * f[column][second];
                }
            }
            propagated[row][column] += g[row][0] * distance_variance_m2 * g[column][0]
                + g[row][1] * yaw_variance_rad2 * g[column][1];
        }
    }
    propagated
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_core::SimDuration;
    use rne_data::{Frame, InMemoryDataBus};
    use rne_ecs::{Entity, World};
    use rne_math::Vec3;
    use rne_world::Transform3 as WorldTransform3;

    const STREAMS: WheelImuOdometryStreams = WheelImuOdometryStreams {
        left_encoder: StreamId(41),
        right_encoder: StreamId(42),
        imu: StreamId(43),
    };

    fn config() -> WheelImuOdometryConfig {
        WheelImuOdometryConfig {
            wheel_radius_m: 0.1,
            track_width_m: 0.5,
            left_counts_per_revolution: 100,
            right_counts_per_revolution: 100,
            left_counter_bits: 8,
            right_counter_bits: 8,
            left_direction: 1,
            right_direction: 1,
            gyro_z_direction: 1.0,
            max_abs_wheel_delta_counts: 120,
            gyro_z_bias_rad_s: 0.0,
            wheel_distance_std_m: 0.001,
            encoder_yaw_std_rad: 0.01,
            gyro_rate_std_rad_s: 0.02,
            gyro_yaw_weight: 0.5,
            disagreement_gyro_yaw_weight: 0.9,
            disagreement_threshold_rad: 0.2,
            max_input_skew_ticks: 0,
            max_frame_age_ticks: 20_000_000,
        }
    }

    fn publish_set(
        bus: &mut InMemoryDataBus,
        sequence: u64,
        capture_ticks: u64,
        latency_ticks: u64,
        left_count: i64,
        right_count: i64,
        gyro_z_rad_s: f64,
    ) {
        for (stream, count) in [
            (STREAMS.left_encoder, left_count),
            (STREAMS.right_encoder, right_count),
        ] {
            bus.publish(
                Frame::new(
                    stream,
                    Entity::PLACEHOLDER,
                    sequence,
                    SimTime::from_ticks(capture_ticks),
                    IncrementalEncoderFeedback {
                        status: if sequence == 1 {
                            IncrementalEncoderStatus::Initializing
                        } else {
                            IncrementalEncoderStatus::Nominal
                        },
                        raw_count: count,
                        position_rad: count as f64 * TAU / 100.0,
                        ..IncrementalEncoderFeedback::default()
                    },
                )
                .with_latency(SimDuration::from_ticks(latency_ticks)),
            );
        }
        bus.publish(
            Frame::new(
                STREAMS.imu,
                Entity::PLACEHOLDER,
                sequence,
                SimTime::from_ticks(capture_ticks),
                ImuFeedback {
                    angular_velocity_rad_s: Vec3::new(0.0, 0.0, gyro_z_rad_s),
                    ..ImuFeedback::default()
                },
            )
            .with_latency(SimDuration::from_ticks(latency_ticks)),
        );
    }

    #[test]
    fn straight_motion_uses_quantized_counts_and_capture_time() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 1, 0, 0, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        assert_eq!(
            estimator
                .update(&bus, STREAMS, SimTime::from_ticks(1))
                .unwrap()
                .health,
            WheelImuOdometryHealth::Initializing
        );
        publish_set(&mut bus, 2, 10_000_000, 5, 10, 10, 0.0);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_005))
            .unwrap();

        assert!((estimate.pose.position_m.x - 0.02 * PI).abs() < 1.0e-12);
        assert_eq!(estimate.pose.position_m.y, 0.0);
        assert!((estimate.linear_velocity_m_s - 2.0 * PI).abs() < 1.0e-12);
        assert_eq!(estimate.health, WheelImuOdometryHealth::Nominal);
        assert!(estimate.pose_covariance[0][0] > 0.0);
        assert!(estimate.pose_covariance[2][2] > 0.0);
        assert_eq!(estimate.provenance.capture_ticks, 10_000_000);
        assert_eq!(estimate.provenance.decision_ticks, 10_000_005);
    }

    #[test]
    fn gyro_changes_heading_without_access_to_truth_orientation() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 0, 0, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        estimator.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        publish_set(&mut bus, 2, 100_000_000, 0, 0, 0, 1.0);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(100_000_000))
            .unwrap();

        assert!((estimate.pose.yaw_rad - 0.05).abs() < 1.0e-12);
        assert!((estimate.gyro_delta_yaw_rad - 0.1).abs() < 1.0e-12);
    }

    #[test]
    fn finite_counter_wrap_is_reconstructed() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 125, 125, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        estimator.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        publish_set(&mut bus, 2, 10_000_000, 0, -126, -126, 0.0);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_000))
            .unwrap();

        assert!((estimate.pose.position_m.x - 0.005 * TAU).abs() < 1.0e-12);
    }

    #[test]
    fn disagreement_is_explicit_and_adaptively_trusts_gyro() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 0, 0, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        estimator.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        publish_set(&mut bus, 2, 100_000_000, 0, -100, 100, 0.0);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(100_000_000))
            .unwrap();

        assert_eq!(
            estimate.health,
            WheelImuOdometryHealth::WheelImuDisagreement
        );
        assert!(estimate.pose.yaw_rad.abs() < estimate.encoder_delta_yaw_rad.abs());
    }

    #[test]
    fn latency_stale_skew_and_sequence_gaps_are_observable() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 10, 0, 0, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        assert!(matches!(
            estimator.update(&bus, STREAMS, SimTime::from_ticks(9)),
            Err(WheelImuOdometryError::MissingAvailableFrame { .. })
        ));
        estimator
            .update(&bus, STREAMS, SimTime::from_ticks(10))
            .unwrap();
        publish_set(&mut bus, 4, 10_000_000, 0, 1, 1, 0.0);
        let estimate = estimator
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_000))
            .unwrap();
        assert_eq!(estimate.health, WheelImuOdometryHealth::InputSequenceGap);
        assert_eq!(estimate.provenance.skipped_sequences, 6);
    }

    #[test]
    fn changing_ecs_truth_cannot_change_a_frame_only_estimate() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 0, 0, 0.0);
        publish_set(&mut bus, 2, 10_000_000, 0, 4, 6, 0.1);
        let mut first = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        first.update(&bus, STREAMS, SimTime::ZERO).unwrap();

        let mut world = World::new();
        let body = world.spawn(WorldTransform3::IDENTITY).id();
        let before = first
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_000))
            .unwrap();
        world.entity_mut(body).insert(WorldTransform3 {
            translation: Vec3::splat(999.0),
            ..WorldTransform3::IDENTITY
        });

        let mut second = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        second.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        let after = second
            .update(&bus, STREAMS, SimTime::from_ticks(10_000_000))
            .unwrap();
        assert_eq!(before, after);
    }

    #[test]
    fn stuck_and_saturated_inputs_fail_closed() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 0, 0, 0.0);
        bus.publish(Frame::new(
            STREAMS.left_encoder,
            Entity::PLACEHOLDER,
            2,
            SimTime::ZERO,
            IncrementalEncoderFeedback {
                status: IncrementalEncoderStatus::CounterSaturated,
                ..IncrementalEncoderFeedback::default()
            },
        ));
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        assert_eq!(
            estimator.update(&bus, STREAMS, SimTime::ZERO),
            Err(WheelImuOdometryError::EncoderSaturated { sensor: "left" })
        );
    }

    #[test]
    fn ambiguous_large_counter_change_fails_closed() {
        let mut bus = InMemoryDataBus::new();
        publish_set(&mut bus, 1, 0, 0, 0, 0, 0.0);
        let mut estimator = WheelImuOdometry::new(config(), PoseSample::default()).unwrap();
        estimator.update(&bus, STREAMS, SimTime::ZERO).unwrap();
        publish_set(&mut bus, 2, 10_000_000, 0, 121, 1, 0.0);

        assert_eq!(
            estimator.update(&bus, STREAMS, SimTime::from_ticks(10_000_000)),
            Err(WheelImuOdometryError::ImplausibleCounterDelta {
                sensor: "left",
                delta_counts: 121,
                maximum_counts: 120,
            })
        );
    }

    #[test]
    fn sensor_only_task_and_actor_observation_expose_no_truth_tensor() {
        let spec = wheel_imu_sensor_only_task_spec(500, 24.0);
        spec.validate().unwrap();
        assert_eq!(spec.task_id, WHEEL_IMU_SENSOR_ONLY_TASK_ID);
        assert!(spec
            .observation
            .tensors
            .iter()
            .all(|tensor| !tensor.name.contains("truth")));
        assert_eq!(
            spec.action.tensors[0].bounds.as_ref().unwrap().lower,
            vec![-24.0]
        );

        let estimate = WheelImuOdometryEstimate {
            pose: PoseSample {
                position_m: Vec3::new(1.0, 2.0, 0.0),
                yaw_rad: 0.5,
            },
            linear_velocity_m_s: 0.3,
            angular_velocity_rad_s: 0.2,
            encoder_delta_yaw_rad: 0.02,
            gyro_delta_yaw_rad: 0.01,
            yaw_innovation_rad: 0.01,
            health: WheelImuOdometryHealth::Nominal,
            provenance: WheelImuOdometryProvenance {
                left_sequence: 2,
                right_sequence: 2,
                imu_sequence: 4,
                capture_ticks: 10,
                decision_ticks: 15,
                max_age_ticks: 7,
                skipped_sequences: 1,
            },
            pose_covariance: [[1.0, 0.1, 0.0], [0.1, 2.0, 0.0], [0.0, 0.0, 0.3]],
        };
        let actor = WheelImuActorObservation::from_estimate(estimate, [4.0, 6.0]);
        assert_eq!(actor.estimated_position_m, [1.0, 2.0]);
        assert_eq!(actor.goal_delta_m, [3.0, 4.0]);
        assert_eq!(actor.health_code, 1);
        assert_eq!(actor.position_covariance_m2, [[1.0, 0.1], [0.1, 2.0]]);
    }
}
