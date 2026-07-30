//! Physics-aware IMU sensor specification and sampling.
//!
//! An inertial measurement unit does not report world-frame velocity. A gyroscope
//! reports angular rate about its own axes, and an accelerometer reports **specific
//! force** — the non-gravitational acceleration it experiences, expressed in the sensor
//! body frame:
//!
//! ```text
//! omega_body = R^T * omega_world
//! f_body     = R^T * (a_world - g_world)
//! ```
//!
//! A device at rest therefore reads `+9.81 m/s^2` along its up axis, not zero and not
//! `-9.81`: the mounting surface pushes it up against gravity.
//!
//! # Error model
//!
//! Errors follow the decomposition used for Allan-variance characterization of MEMS
//! parts (see IEEE Std 952 and the MEMS stochastic-error literature). For a true
//! measurement `x` over a sample interval `dt`:
//!
//! ```text
//! measured = M x + b_turn_on + b_gm + b_rrw + n_white
//! ```
//!
//! * `M` — scale-factor and axis-misalignment matrix, small-angle.
//! * `b_turn_on` — fixed turn-on bias, constant for a run.
//! * `b_gm` — bias instability as a first-order Gauss-Markov process with correlation
//!   time `tau`, the flat minimum of an Allan deviation curve.
//! * `b_rrw` — rate random walk, the rising tail of an Allan deviation curve.
//! * `n_white` — angle/velocity random walk, white noise whose standard deviation over
//!   an interval scales as `sigma / sqrt(dt)`.
//!
//! The output is then saturated to the configured measurement range and quantized to the
//! configured resolution, in that order, matching how a real part clips before its
//! analog-to-digital converter rounds.
//!
//! # Determinism and state
//!
//! `b_gm` and `b_rrw` evolve over time, so they need state. [`ImuState`] carries that
//! state on the sensor entity, and every random draw comes from a disjoint slot of a
//! [`SensorNoiseKey`]-derived stream. Replaying the same sample sequence therefore
//! reproduces the same drift exactly. No wall-clock time is read: `dt` comes from the
//! difference between explicit [`SimTime`] samples.
//!
//! [`sample_imu`] and [`sample_imu_keyed`] remain stateless. They model a device with no
//! proper acceleration and no bias evolution, which is the right answer for a static
//! probe and keeps existing callers working.

use crate::components::ImuState;
use crate::noise::{gaussian_pair, NoiseModel, SensorNoiseKey};
use rne_core::{mix64, KeyedRandom, SimDuration, SimTime};
use rne_data::ImuSample;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::RigidBody;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

const IMU_RANDOM_DOMAIN_V1: u64 = 0x3155_4D49_5F45_4E52;
/// Random slots for the physical error model, disjoint from [`NoiseModel`] slots 0..3.
const SLOT_BASE: u64 = 1_024;
const SLOT_GYRO_WHITE: u64 = SLOT_BASE;
const SLOT_ACCEL_WHITE: u64 = SLOT_BASE + 8;
const SLOT_GYRO_BIAS: u64 = SLOT_BASE + 16;
const SLOT_ACCEL_BIAS: u64 = SLOT_BASE + 24;
const SLOT_GYRO_RATE_WALK: u64 = SLOT_BASE + 32;
const SLOT_ACCEL_RATE_WALK: u64 = SLOT_BASE + 40;

/// Standard gravity vector in the world frame.
pub const GRAVITY_M_S2: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Allan-variance error parameters for one inertial triad.
///
/// Units follow the measured quantity: `rad/s` for a gyroscope, `m/s^2` for an
/// accelerometer. All values default to zero, which is an ideal sensor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuAxisErrors {
    /// White-noise density in `unit / sqrt(Hz)`; angle or velocity random walk.
    pub random_walk: f64,
    /// Standard deviation of the Gauss-Markov bias in `unit`; bias instability.
    pub bias_instability: f64,
    /// Correlation time of the Gauss-Markov bias in seconds.
    pub bias_correlation_time_s: f64,
    /// Rate random walk coefficient in `unit / s^1.5`.
    pub rate_random_walk: f64,
    /// Fixed turn-on bias per axis in `unit`.
    pub turn_on_bias: Vec3,
    /// Fractional scale-factor error per axis; `0.01` is one percent.
    pub scale_factor_error: Vec3,
    /// Small-angle axis misalignment in radians about each axis.
    pub misalignment_rad: Vec3,
}

impl ImuAxisErrors {
    /// Returns true when this triad introduces no error at all.
    pub fn is_ideal(&self) -> bool {
        self.random_walk <= 0.0
            && self.bias_instability <= 0.0
            && self.rate_random_walk <= 0.0
            && self.turn_on_bias == Vec3::ZERO
            && self.scale_factor_error == Vec3::ZERO
            && self.misalignment_rad == Vec3::ZERO
    }

    /// Applies the small-angle scale-factor and misalignment matrix.
    ///
    /// ```text
    ///     | 1 + sx    -mz      my  |
    /// M = |   mz    1 + sy    -mx  |
    ///     |  -my      mx    1 + sz |
    /// ```
    pub fn apply_scale_misalignment(&self, value: Vec3) -> Vec3 {
        let s = self.scale_factor_error;
        let m = self.misalignment_rad;
        Vec3::new(
            (1.0 + s.x) * value.x - m.z * value.y + m.y * value.z,
            m.z * value.x + (1.0 + s.y) * value.y - m.x * value.z,
            -m.y * value.x + m.x * value.y + (1.0 + s.z) * value.z,
        )
    }
}

/// IMU sensor parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ImuSpec {
    /// Legacy additive noise model, applied after the physical error model.
    pub noise: NoiseModel,
    /// Deterministic noise seed.
    pub seed: u64,
    /// Gyroscope error parameters in `rad/s`.
    pub gyro: ImuAxisErrors,
    /// Accelerometer error parameters in `m/s^2`.
    pub accel: ImuAxisErrors,
    /// Gyroscope measurement range in `rad/s`; `0.0` disables saturation.
    pub gyro_range_rad_s: f64,
    /// Accelerometer measurement range in `m/s^2`; `0.0` disables saturation.
    pub accel_range_m_s2: f64,
    /// Gyroscope quantization step in `rad/s`; `0.0` disables quantization.
    pub gyro_resolution_rad_s: f64,
    /// Accelerometer quantization step in `m/s^2`; `0.0` disables quantization.
    pub accel_resolution_m_s2: f64,
}

impl ImuSpec {
    /// Returns true when no physical error term would modify a measurement.
    pub fn is_ideal(&self) -> bool {
        self.gyro.is_ideal()
            && self.accel.is_ideal()
            && self.gyro_range_rad_s <= 0.0
            && self.accel_range_m_s2 <= 0.0
            && self.gyro_resolution_rad_s <= 0.0
            && self.accel_resolution_m_s2 <= 0.0
    }
}

/// Samples an IMU attached to the given entity.
///
/// This stateless entry point models a device with no proper acceleration and no bias
/// evolution: the accelerometer reports the reaction to gravity in the body frame. Use
/// [`sample_imu_stateful`] for the full drift model.
pub fn sample_imu(world: &World, entity: Entity, spec: &ImuSpec) -> ImuSample {
    let (angular, linear) = static_measurement(world, entity);
    let (angular, linear) = spec.noise.apply_imu(
        angular,
        linear,
        spec.seed.wrapping_add(entity.index() as u64),
    );

    ImuSample {
        angular_velocity_rad_s: angular,
        linear_acceleration_m_s2: linear,
    }
}

/// Samples an IMU attached to the given entity using a stateless noise key.
pub fn sample_imu_keyed(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
) -> ImuSample {
    let (angular, linear) = static_measurement(world, entity);
    let (angular, linear) = spec.noise.apply_imu_keyed(angular, linear, noise_key);

    ImuSample {
        angular_velocity_rad_s: angular,
        linear_acceleration_m_s2: linear,
    }
}

/// Samples an IMU with the full physical error model.
///
/// `state` carries the bias processes and the previous body velocity between samples and
/// is advanced in place. `sim_time` must be the current sample time; the interval since
/// the previous sample drives both the derived proper acceleration and the noise scaling.
pub fn sample_imu_stateful(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    state: &mut ImuState,
) -> ImuSample {
    let rotation = world
        .get::<Transform3>(entity)
        .map(|transform| transform.rotation)
        .unwrap_or_default();
    let (angular_world, linear_velocity_world) = world
        .get::<RigidBody>(entity)
        .map(|body| (body.angular_velocity_rad_s, body.linear_velocity_m_s))
        .unwrap_or((Vec3::ZERO, Vec3::ZERO));

    let dt_s = state.interval_since(sim_time).as_seconds().value();
    // Proper acceleration is the derivative of world velocity; without a previous
    // sample the device is assumed to be in steady motion.
    let acceleration_world = if state.initialized && dt_s > 0.0 {
        (linear_velocity_world - state.previous_linear_velocity_m_s) / dt_s
    } else {
        Vec3::ZERO
    };

    let inverse = rotation.inverse();
    let true_angular = inverse * angular_world;
    // Specific force: what an accelerometer feels is acceleration minus gravity.
    let true_specific_force = inverse * (acceleration_world - GRAVITY_M_S2);

    let random = imu_random(noise_key);
    let effective_dt_s = if dt_s > 0.0 { dt_s } else { 0.0 };

    let angular = apply_axis_errors(
        true_angular,
        &spec.gyro,
        &random,
        noise_key,
        effective_dt_s,
        AxisSlots {
            white: SLOT_GYRO_WHITE,
            bias: SLOT_GYRO_BIAS,
            rate_walk: SLOT_GYRO_RATE_WALK,
        },
        &mut state.gyro_bias_rad_s,
        &mut state.gyro_rate_walk_rad_s,
    );
    let linear = apply_axis_errors(
        true_specific_force,
        &spec.accel,
        &random,
        noise_key,
        effective_dt_s,
        AxisSlots {
            white: SLOT_ACCEL_WHITE,
            bias: SLOT_ACCEL_BIAS,
            rate_walk: SLOT_ACCEL_RATE_WALK,
        },
        &mut state.accel_bias_m_s2,
        &mut state.accel_rate_walk_m_s2,
    );

    let angular = quantize(
        saturate(angular, spec.gyro_range_rad_s),
        spec.gyro_resolution_rad_s,
    );
    let linear = quantize(
        saturate(linear, spec.accel_range_m_s2),
        spec.accel_resolution_m_s2,
    );

    let (angular, linear) = spec.noise.apply_imu_keyed(angular, linear, noise_key);

    state.previous_linear_velocity_m_s = linear_velocity_world;
    state.previous_sample_ticks = sim_time.ticks();
    state.initialized = true;

    ImuSample {
        angular_velocity_rad_s: angular,
        linear_acceleration_m_s2: linear,
    }
}

/// Random slot assignments for one inertial triad.
struct AxisSlots {
    white: u64,
    bias: u64,
    rate_walk: u64,
}

#[allow(clippy::too_many_arguments)]
fn apply_axis_errors(
    truth: Vec3,
    errors: &ImuAxisErrors,
    random: &KeyedRandom,
    key: SensorNoiseKey,
    dt_s: f64,
    slots: AxisSlots,
    bias: &mut Vec3,
    rate_walk: &mut Vec3,
) -> Vec3 {
    if errors.is_ideal() {
        return truth;
    }

    advance_gauss_markov(errors, random, key, dt_s, slots.bias, bias);
    advance_rate_random_walk(errors, random, key, dt_s, slots.rate_walk, rate_walk);

    let mut measured =
        errors.apply_scale_misalignment(truth) + errors.turn_on_bias + *bias + *rate_walk;

    if errors.random_walk > 0.0 {
        // White-noise density integrates down over a longer sample interval.
        let sigma = if dt_s > 0.0 {
            errors.random_walk / dt_s.sqrt()
        } else {
            errors.random_walk
        };
        measured += gaussian_vec3(random, key, slots.white) * sigma;
    }
    measured
}

/// Advances the first-order Gauss-Markov bias toward its stationary distribution.
fn advance_gauss_markov(
    errors: &ImuAxisErrors,
    random: &KeyedRandom,
    key: SensorNoiseKey,
    dt_s: f64,
    slot: u64,
    bias: &mut Vec3,
) {
    let sigma = errors.bias_instability;
    if sigma <= 0.0 {
        return;
    }
    let tau = errors.bias_correlation_time_s;
    if tau <= 0.0 || dt_s <= 0.0 {
        // Without a correlation time the bias is a fresh draw each sample.
        *bias = gaussian_vec3(random, key, slot) * sigma;
        return;
    }
    let decay = (-dt_s / tau).exp();
    let driving = sigma * (1.0 - decay * decay).max(0.0).sqrt();
    *bias = *bias * decay + gaussian_vec3(random, key, slot) * driving;
}

/// Integrates the rate random walk, whose increments scale with `sqrt(dt)`.
fn advance_rate_random_walk(
    errors: &ImuAxisErrors,
    random: &KeyedRandom,
    key: SensorNoiseKey,
    dt_s: f64,
    slot: u64,
    rate_walk: &mut Vec3,
) {
    let coefficient = errors.rate_random_walk;
    if coefficient <= 0.0 || dt_s <= 0.0 {
        return;
    }
    *rate_walk += gaussian_vec3(random, key, slot) * (coefficient * dt_s.sqrt());
}

/// Returns the body-frame measurement of a device with no proper acceleration.
fn static_measurement(world: &World, entity: Entity) -> (Vec3, Vec3) {
    let rotation = world
        .get::<Transform3>(entity)
        .map(|transform| transform.rotation)
        .unwrap_or_default();
    let angular_world = world
        .get::<RigidBody>(entity)
        .map(|body| body.angular_velocity_rad_s)
        .unwrap_or(Vec3::ZERO);
    let inverse = rotation.inverse();
    (inverse * angular_world, inverse * -GRAVITY_M_S2)
}

fn saturate(value: Vec3, range: f64) -> Vec3 {
    if range <= 0.0 {
        return value;
    }
    Vec3::new(
        value.x.clamp(-range, range),
        value.y.clamp(-range, range),
        value.z.clamp(-range, range),
    )
}

fn quantize(value: Vec3, step: f64) -> Vec3 {
    if step <= 0.0 {
        return value;
    }
    Vec3::new(
        (value.x / step).round() * step,
        (value.y / step).round() * step,
        (value.z / step).round() * step,
    )
}

fn imu_random(key: SensorNoiseKey) -> KeyedRandom {
    KeyedRandom::new(key.root_seed, IMU_RANDOM_DOMAIN_V1 ^ mix64(key.sensor_seed))
}

/// Draws three independent standard normals from consecutive slot pairs.
fn gaussian_vec3(random: &KeyedRandom, key: SensorNoiseKey, slot: u64) -> Vec3 {
    let (x, y) = gaussian_pair(random, key, slot);
    let (z, _) = gaussian_pair(random, key, slot + 4);
    Vec3::new(x, y, z)
}

impl ImuState {
    /// Returns the interval since the previous sample.
    fn interval_since(&self, sim_time: SimTime) -> SimDuration {
        SimDuration::from_ticks(self.previous_sample_ticks.abs_diff(sim_time.ticks()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::spawn_named;
    use rne_math::{Quat, Seconds};

    fn imu_world(
        rotation: Quat,
        linear_velocity_m_s: Vec3,
        angular_rad_s: Vec3,
    ) -> (World, Entity) {
        let mut world = World::new();
        let sensor = spawn_named(&mut world, "imu");
        world.entity_mut(sensor).insert((
            Transform3::from_translation_rotation(Vec3::ZERO, rotation),
            RigidBody {
                linear_velocity_m_s,
                angular_velocity_rad_s: angular_rad_s,
                ..RigidBody::default()
            },
        ));
        (world, sensor)
    }

    fn at(seconds: f64) -> SimTime {
        SimTime::from_seconds(Seconds::new(seconds))
    }

    #[test]
    fn static_imu_reports_upward_specific_force() {
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);

        let sample = sample_imu(&world, sensor, &ImuSpec::default());

        // A device at rest is pushed up against gravity and reads +1 g, not -1 g.
        assert_relative_eq!(sample.linear_acceleration_m_s2.y, 9.81, epsilon = 1e-9);
        assert_relative_eq!(sample.linear_acceleration_m_s2.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(sample.linear_acceleration_m_s2.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn measurements_are_reported_in_the_body_frame() {
        // Rolled 90 degrees about +Z, the body +X axis points along world up, so the
        // accelerometer reads its +1 g on X instead of Y.
        let rotation = Quat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let (world, sensor) = imu_world(rotation, Vec3::ZERO, Vec3::new(0.0, 0.7, 0.0));

        let sample = sample_imu(&world, sensor, &ImuSpec::default());

        assert_relative_eq!(sample.linear_acceleration_m_s2.x, 9.81, epsilon = 1e-9);
        assert_relative_eq!(sample.linear_acceleration_m_s2.y, 0.0, epsilon = 1e-9);
        // Angular rate rotates into the body frame the same way.
        assert_relative_eq!(sample.angular_velocity_rad_s.x, 0.7, epsilon = 1e-9);
        assert_relative_eq!(sample.angular_velocity_rad_s.y, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn accelerometer_reports_proper_acceleration_not_velocity() {
        let spec = ImuSpec::default();
        let key = SensorNoiseKey::new(1, 2, 3, 4);
        let mut state = ImuState::default();

        // Steady 5 m/s: no proper acceleration, so only gravity is felt.
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO);
        sample_imu_stateful(&world, sensor, &spec, key, at(0.0), &mut state);
        let steady = sample_imu_stateful(&world, sensor, &spec, key, at(0.1), &mut state);
        assert_relative_eq!(steady.linear_acceleration_m_s2.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(steady.linear_acceleration_m_s2.y, 9.81, epsilon = 1e-9);

        // Accelerating from 5 to 7 m/s over 0.1 s is 20 m/s^2 along +X.
        let (faster, sensor_faster) =
            imu_world(Quat::IDENTITY, Vec3::new(7.0, 0.0, 0.0), Vec3::ZERO);
        let accelerating =
            sample_imu_stateful(&faster, sensor_faster, &spec, key, at(0.2), &mut state);
        assert_relative_eq!(
            accelerating.linear_acceleration_m_s2.x,
            20.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn ideal_spec_leaves_the_measurement_untouched() {
        let spec = ImuSpec::default();
        assert!(spec.is_ideal());

        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::new(0.3, -0.2, 0.1));
        let mut state = ImuState::default();
        let sample = sample_imu_stateful(
            &world,
            sensor,
            &spec,
            SensorNoiseKey::new(9, 9, 9, 9),
            at(0.0),
            &mut state,
        );

        assert_relative_eq!(sample.angular_velocity_rad_s.x, 0.3, epsilon = 1e-12);
        assert_relative_eq!(sample.angular_velocity_rad_s.y, -0.2, epsilon = 1e-12);
        assert_relative_eq!(sample.linear_acceleration_m_s2.y, 9.81, epsilon = 1e-12);
    }

    #[test]
    fn scale_factor_and_misalignment_form_a_small_angle_matrix() {
        let errors = ImuAxisErrors {
            scale_factor_error: Vec3::new(0.01, 0.02, -0.01),
            misalignment_rad: Vec3::new(0.001, -0.002, 0.003),
            ..ImuAxisErrors::default()
        };

        // A pure +X input leaks into the other axes through misalignment.
        let measured = errors.apply_scale_misalignment(Vec3::X);
        assert_relative_eq!(measured.x, 1.01, epsilon = 1e-12);
        assert_relative_eq!(measured.y, 0.003, epsilon = 1e-12);
        assert_relative_eq!(measured.z, 0.002, epsilon = 1e-12);

        // An ideal triad is the identity.
        let ideal = ImuAxisErrors::default();
        assert_eq!(
            ideal.apply_scale_misalignment(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0)
        );
    }

    #[test]
    fn turn_on_bias_offsets_every_sample_identically() {
        let spec = ImuSpec {
            gyro: ImuAxisErrors {
                turn_on_bias: Vec3::new(0.01, -0.02, 0.03),
                ..ImuAxisErrors::default()
            },
            ..ImuSpec::default()
        };
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        let key = SensorNoiseKey::new(4, 4, 4, 4);
        let mut state = ImuState::default();

        let first = sample_imu_stateful(&world, sensor, &spec, key, at(0.0), &mut state);
        let second = sample_imu_stateful(&world, sensor, &spec, key, at(0.5), &mut state);

        assert_relative_eq!(first.angular_velocity_rad_s.x, 0.01, epsilon = 1e-12);
        assert_eq!(first.angular_velocity_rad_s, second.angular_velocity_rad_s);
    }

    #[test]
    fn white_noise_scales_down_with_a_longer_sample_interval() {
        let spec = ImuSpec {
            gyro: ImuAxisErrors {
                random_walk: 0.01,
                ..ImuAxisErrors::default()
            },
            ..ImuSpec::default()
        };
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);

        let deviation = |dt: f64| {
            let mut total = 0.0;
            for index in 0..400_u64 {
                let mut state = ImuState {
                    previous_sample_ticks: 0,
                    initialized: true,
                    ..ImuState::default()
                };
                let sample = sample_imu_stateful(
                    &world,
                    sensor,
                    &spec,
                    SensorNoiseKey::new(7, 7, 7, index),
                    at(dt),
                    &mut state,
                );
                total += sample.angular_velocity_rad_s.x.abs();
            }
            total / 400.0
        };

        // Quadrupling the interval halves the noise, because sigma goes as 1/sqrt(dt).
        let fast = deviation(0.01);
        let slow = deviation(0.04);
        assert!(slow < fast);
        assert_relative_eq!(slow, fast * 0.5, max_relative = 0.15);
    }

    #[test]
    fn gauss_markov_bias_decays_toward_zero_without_new_driving_noise() {
        let errors = ImuAxisErrors {
            bias_instability: 0.02,
            bias_correlation_time_s: 10.0,
            ..ImuAxisErrors::default()
        };
        let random = imu_random(SensorNoiseKey::new(1, 1, 1, 1));
        let mut bias = Vec3::new(0.05, 0.05, 0.05);

        // With the driving term removed the process relaxes by exp(-dt / tau).
        let decayed = bias * (-1.0_f64 / 10.0).exp();
        advance_gauss_markov(
            &ImuAxisErrors {
                bias_instability: 1e-18,
                ..errors
            },
            &random,
            SensorNoiseKey::new(1, 1, 1, 1),
            1.0,
            SLOT_GYRO_BIAS,
            &mut bias,
        );
        assert_relative_eq!(bias.x, decayed.x, max_relative = 1e-6);
    }

    #[test]
    fn bias_instability_stays_bounded_while_rate_random_walk_grows() {
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);

        let run = |spec: ImuSpec| {
            let mut state = ImuState::default();
            let mut maximum = 0.0_f64;
            for index in 1..600_u64 {
                let sample = sample_imu_stateful(
                    &world,
                    sensor,
                    &spec,
                    SensorNoiseKey::new(3, 3, 3, index),
                    at(index as f64 * 0.01),
                    &mut state,
                );
                maximum = maximum.max(sample.angular_velocity_rad_s.x.abs());
            }
            (maximum, state)
        };

        let (bounded, _) = run(ImuSpec {
            gyro: ImuAxisErrors {
                bias_instability: 0.002,
                bias_correlation_time_s: 1.0,
                ..ImuAxisErrors::default()
            },
            ..ImuSpec::default()
        });
        let (walking, walk_state) = run(ImuSpec {
            gyro: ImuAxisErrors {
                rate_random_walk: 0.02,
                ..ImuAxisErrors::default()
            },
            ..ImuSpec::default()
        });

        // A stationary Gauss-Markov bias stays near its standard deviation.
        assert!(bounded < 0.02, "bias instability drifted to {bounded}");
        // A rate random walk accumulates without bound.
        assert!(walking > bounded);
        assert!(walk_state.gyro_rate_walk_rad_s.x.abs() > 0.0);
    }

    #[test]
    fn saturation_clips_and_quantization_rounds() {
        assert_eq!(
            saturate(Vec3::new(5.0, -5.0, 0.5), 2.0),
            Vec3::new(2.0, -2.0, 0.5)
        );
        assert_eq!(
            saturate(Vec3::new(5.0, -5.0, 0.5), 0.0),
            Vec3::new(5.0, -5.0, 0.5)
        );

        let quantized = quantize(Vec3::new(0.123, -0.077, 0.0), 0.05);
        assert_relative_eq!(quantized.x, 0.1, epsilon = 1e-12);
        assert_relative_eq!(quantized.y, -0.1, epsilon = 1e-12);
        assert_eq!(quantize(Vec3::new(0.123, 0.0, 0.0), 0.0).x, 0.123);
    }

    #[test]
    fn saturation_applies_before_quantization() {
        let spec = ImuSpec {
            accel_range_m_s2: 4.0,
            accel_resolution_m_s2: 0.5,
            ..ImuSpec::default()
        };
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        let mut state = ImuState::default();

        let sample = sample_imu_stateful(
            &world,
            sensor,
            &spec,
            SensorNoiseKey::new(2, 2, 2, 2),
            at(0.0),
            &mut state,
        );

        // 9.81 clips to the 4 m/s^2 range, which is already on the quantization grid.
        assert_relative_eq!(sample.linear_acceleration_m_s2.y, 4.0, epsilon = 1e-12);
    }

    #[test]
    fn drift_is_repeatable_for_the_same_key_sequence() {
        let spec = ImuSpec {
            gyro: ImuAxisErrors {
                random_walk: 0.005,
                bias_instability: 0.001,
                bias_correlation_time_s: 5.0,
                rate_random_walk: 0.0005,
                ..ImuAxisErrors::default()
            },
            accel: ImuAxisErrors {
                random_walk: 0.02,
                bias_instability: 0.01,
                bias_correlation_time_s: 20.0,
                ..ImuAxisErrors::default()
            },
            ..ImuSpec::default()
        };
        let (world, sensor) = imu_world(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO);

        let run = || {
            let mut state = ImuState::default();
            let mut samples = Vec::new();
            for index in 1..50_u64 {
                samples.push(sample_imu_stateful(
                    &world,
                    sensor,
                    &spec,
                    SensorNoiseKey::new(21, 22, 23, index),
                    at(index as f64 * 0.01),
                    &mut state,
                ));
            }
            (samples, state)
        };

        let (first, first_state) = run();
        let (second, second_state) = run();
        assert_eq!(first, second);
        assert_eq!(first_state, second_state);
        // The run actually drifted rather than sitting on the truth.
        assert!(first
            .iter()
            .any(|sample| (sample.linear_acceleration_m_s2.y - 9.81).abs() > 1e-6));
    }
}
