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

use crate::components::{ImuKinematicState, ImuMount, ImuState};
use crate::noise::{gaussian_pair, NoiseModel, SensorNoiseKey};
use rne_core::{mix64, KeyedRandom, SimDuration, SimTime};
use rne_data::ImuSample;
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::RigidBody;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

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
    /// Returns true when every parameter is finite and stochastic scales are non-negative.
    pub fn is_valid(&self) -> bool {
        self.random_walk.is_finite()
            && self.random_walk >= 0.0
            && self.bias_instability.is_finite()
            && self.bias_instability >= 0.0
            && self.bias_correlation_time_s.is_finite()
            && self.bias_correlation_time_s >= 0.0
            && self.rate_random_walk.is_finite()
            && self.rate_random_walk >= 0.0
            && self.turn_on_bias.is_finite()
            && self.scale_factor_error.is_finite()
            && self.misalignment_rad.is_finite()
    }

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
    /// Returns true when every physical/noise parameter is finite and unit-valid.
    pub fn is_valid(&self) -> bool {
        self.gyro.is_valid()
            && self.accel.is_valid()
            && self.gyro_range_rad_s.is_finite()
            && self.gyro_range_rad_s >= 0.0
            && self.accel_range_m_s2.is_finite()
            && self.accel_range_m_s2 >= 0.0
            && self.gyro_resolution_rad_s.is_finite()
            && self.gyro_resolution_rad_s >= 0.0
            && self.accel_resolution_m_s2.is_finite()
            && self.accel_resolution_m_s2 >= 0.0
            && self.noise.angular_stddev_rad_s.is_finite()
            && self.noise.angular_stddev_rad_s >= 0.0
            && self.noise.linear_stddev_m_s2.is_finite()
            && self.noise.linear_stddev_m_s2 >= 0.0
            && self.noise.linear_bias_m_s2.is_finite()
    }

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

/// Ground-truth kinematics at the physical IMU location.
///
/// This is diagnostic evidence, not an IMU measurement. In particular, the
/// world orientation is retained for validation and estimator scoring rather
/// than being inserted into the raw gyroscope/accelerometer payload.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuTruth {
    /// Sensor orientation as the rotation from sensor frame into world frame.
    pub world_from_sensor: Quat,
    /// True sensor-frame angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
    /// True sensor-frame specific force in meters per second squared.
    pub specific_force_m_s2: Vec3,
}

/// Raw IMU sample accompanied by separately labeled validation diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuDiagnosticSample {
    /// Raw gyroscope and accelerometer output after the declared error pipeline.
    pub measurement: ImuSample,
    /// Ground truth at the mounted sensor location.
    pub truth: ImuTruth,
    /// Per-axis gyroscope saturation flags before quantization.
    pub gyro_saturated: [bool; 3],
    /// Per-axis accelerometer saturation flags before quantization.
    pub accel_saturated: [bool; 3],
}

/// Error returned before sampling an explicitly mounted IMU.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum ImuSampleError {
    /// The body-to-sensor transform is non-finite, scaled, or non-unit.
    #[error("IMU mount transform is invalid")]
    InvalidMount,
    /// The declared mount body has no world transform.
    #[error("IMU mount body has no Transform3")]
    MissingBodyTransform,
    /// The declared mount body has no rigid-body kinematics.
    #[error("IMU mount body has no RigidBody")]
    MissingRigidBody,
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
    sample_imu_stateful_impl(
        world,
        entity,
        Transform3::IDENTITY,
        spec,
        noise_key,
        sim_time,
        ImuStatefulStates {
            error_state: state,
            kinematic_state: None,
        },
    )
    .measurement
}

/// Samples an IMU while retaining angular history for offset-mount acceleration.
///
/// This is the lever-arm-aware counterpart to [`sample_imu_stateful`]. The
/// separate kinematic state preserves the compatibility-stable [`ImuState`]
/// layout while allowing deterministic angular acceleration to span samples.
pub fn sample_imu_stateful_with_kinematics(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    state: &mut ImuState,
    kinematic_state: &mut ImuKinematicState,
) -> ImuSample {
    sample_imu_stateful_impl(
        world,
        entity,
        Transform3::IDENTITY,
        spec,
        noise_key,
        sim_time,
        ImuStatefulStates {
            error_state: state,
            kinematic_state: Some(kinematic_state),
        },
    )
    .measurement
}

/// Samples a stateful IMU and retains separately labeled truth/saturation evidence.
///
/// When the sensor entity carries an [`ImuMount`], body kinematics are resolved
/// at the declared sensor pose. Otherwise the sensor entity itself remains the
/// rigid body and an identity mount preserves the legacy behavior.
pub fn sample_imu_stateful_diagnostic(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    state: &mut ImuState,
) -> Result<ImuDiagnosticSample, ImuSampleError> {
    sample_imu_stateful_diagnostic_impl(world, entity, spec, noise_key, sim_time, state, None)
}

/// Samples a mounted IMU with diagnostic evidence and angular history.
///
/// Use this entry point for a non-zero mount translation when tangential
/// acceleration from changing angular velocity must be represented.
pub fn sample_imu_stateful_diagnostic_with_kinematics(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    state: &mut ImuState,
    kinematic_state: &mut ImuKinematicState,
) -> Result<ImuDiagnosticSample, ImuSampleError> {
    sample_imu_stateful_diagnostic_impl(
        world,
        entity,
        spec,
        noise_key,
        sim_time,
        state,
        Some(kinematic_state),
    )
}

fn sample_imu_stateful_diagnostic_impl(
    world: &World,
    entity: Entity,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    state: &mut ImuState,
    kinematic_state: Option<&mut ImuKinematicState>,
) -> Result<ImuDiagnosticSample, ImuSampleError> {
    let mount = world.get::<ImuMount>(entity).copied().unwrap_or(ImuMount {
        body_entity: entity,
        body_from_sensor: Transform3::IDENTITY,
    });
    if !mount.is_valid() {
        return Err(ImuSampleError::InvalidMount);
    }
    if world.get::<Transform3>(mount.body_entity).is_none() {
        return Err(ImuSampleError::MissingBodyTransform);
    }
    if world.get::<RigidBody>(mount.body_entity).is_none() {
        return Err(ImuSampleError::MissingRigidBody);
    }
    Ok(sample_imu_stateful_impl(
        world,
        mount.body_entity,
        mount.body_from_sensor,
        spec,
        noise_key,
        sim_time,
        ImuStatefulStates {
            error_state: state,
            kinematic_state,
        },
    ))
}

struct ImuStatefulStates<'a> {
    error_state: &'a mut ImuState,
    kinematic_state: Option<&'a mut ImuKinematicState>,
}

fn sample_imu_stateful_impl(
    world: &World,
    body_entity: Entity,
    body_from_sensor: Transform3,
    spec: &ImuSpec,
    noise_key: SensorNoiseKey,
    sim_time: SimTime,
    states: ImuStatefulStates<'_>,
) -> ImuDiagnosticSample {
    let ImuStatefulStates {
        error_state: state,
        kinematic_state,
    } = states;
    let world_from_body = world
        .get::<Transform3>(body_entity)
        .copied()
        .unwrap_or(Transform3::IDENTITY);
    let world_from_sensor = world_from_body.mul_transform(&body_from_sensor);
    let (angular_world, linear_velocity_world) = world
        .get::<RigidBody>(body_entity)
        .map(|body| (body.angular_velocity_rad_s, body.linear_velocity_m_s))
        .unwrap_or((Vec3::ZERO, Vec3::ZERO));

    let dt_s = state.interval_since(sim_time).as_seconds().value();
    // Body-origin acceleration and angular acceleration come from explicit
    // fixed-step velocity differences. Without a previous sample the device is
    // assumed to be in steady motion.
    let body_acceleration_world = if state.initialized && dt_s > 0.0 {
        (linear_velocity_world - state.previous_linear_velocity_m_s) / dt_s
    } else {
        Vec3::ZERO
    };
    let angular_acceleration_world = if let Some(kinematic_state) = kinematic_state.as_ref() {
        if kinematic_state.initialized && dt_s > 0.0 {
            (angular_world - kinematic_state.previous_angular_velocity_rad_s) / dt_s
        } else {
            Vec3::ZERO
        }
    } else {
        Vec3::ZERO
    };
    let lever_arm_world = world_from_body.rotation * body_from_sensor.translation;
    let sensor_acceleration_world = body_acceleration_world
        + angular_acceleration_world.cross(lever_arm_world)
        + angular_world.cross(angular_world.cross(lever_arm_world));

    let inverse = world_from_sensor.rotation.inverse();
    let true_angular = inverse * angular_world;
    // Specific force: what an accelerometer feels is acceleration minus gravity.
    let true_specific_force = inverse * (sensor_acceleration_world - GRAVITY_M_S2);

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

    let gyro_saturated = saturated_axes(angular, spec.gyro_range_rad_s);
    let accel_saturated = saturated_axes(linear, spec.accel_range_m_s2);
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
    if let Some(kinematic_state) = kinematic_state {
        kinematic_state.previous_angular_velocity_rad_s = angular_world;
        kinematic_state.initialized = true;
    }

    ImuDiagnosticSample {
        measurement: ImuSample {
            angular_velocity_rad_s: angular,
            linear_acceleration_m_s2: linear,
        },
        truth: ImuTruth {
            world_from_sensor: world_from_sensor.rotation,
            angular_velocity_rad_s: true_angular,
            specific_force_m_s2: true_specific_force,
        },
        gyro_saturated,
        accel_saturated,
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

fn saturated_axes(value: Vec3, range: f64) -> [bool; 3] {
    if range <= 0.0 {
        return [false; 3];
    }
    [
        value.x.abs() > range,
        value.y.abs() > range,
        value.z.abs() > range,
    ]
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

    fn mounted_imu_world(body: RigidBody, body_from_sensor: Transform3) -> (World, Entity, Entity) {
        let mut world = World::new();
        let body_entity = spawn_named(&mut world, "body");
        world
            .entity_mut(body_entity)
            .insert((Transform3::IDENTITY, body));
        let sensor = spawn_named(&mut world, "mounted_imu");
        world.entity_mut(sensor).insert(ImuMount {
            body_entity,
            body_from_sensor,
        });
        (world, body_entity, sensor)
    }

    fn assert_vec3_close(actual: Vec3, expected: Vec3) {
        assert_relative_eq!(actual.x, expected.x, epsilon = 1.0e-12);
        assert_relative_eq!(actual.y, expected.y, epsilon = 1.0e-12);
        assert_relative_eq!(actual.z, expected.z, epsilon = 1.0e-12);
    }

    #[test]
    fn mounted_truth_uses_the_sensor_axes_without_claiming_orientation_as_measurement() {
        let body = RigidBody {
            angular_velocity_rad_s: Vec3::X,
            ..RigidBody::default()
        };
        let body_from_sensor = Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        );
        let (world, _, sensor) = mounted_imu_world(body, body_from_sensor);
        let sample = sample_imu_stateful_diagnostic(
            &world,
            sensor,
            &ImuSpec::default(),
            SensorNoiseKey::new(1, 2, 3, 1),
            at(0.0),
            &mut ImuState::default(),
        )
        .unwrap();

        assert_vec3_close(sample.truth.angular_velocity_rad_s, Vec3::NEG_Y);
        assert_vec3_close(sample.truth.specific_force_m_s2, Vec3::new(9.81, 0.0, 0.0));
        assert_vec3_close(sample.measurement.angular_velocity_rad_s, Vec3::NEG_Y);
    }

    #[test]
    fn offset_mount_includes_centripetal_and_tangential_acceleration() {
        let body = RigidBody {
            angular_velocity_rad_s: Vec3::new(0.0, 0.0, 2.0),
            ..RigidBody::default()
        };
        let mount = Transform3::from_translation_rotation(Vec3::new(0.5, 0.0, 0.0), Quat::IDENTITY);
        let (mut world, body_entity, sensor) = mounted_imu_world(RigidBody::default(), mount);
        let mut state = ImuState::default();
        let mut kinematic_state = ImuKinematicState::default();
        sample_imu_stateful_diagnostic_with_kinematics(
            &world,
            sensor,
            &ImuSpec::default(),
            SensorNoiseKey::new(1, 2, 3, 1),
            at(0.0),
            &mut state,
            &mut kinematic_state,
        )
        .unwrap();
        world.entity_mut(body_entity).insert(body);
        let sample = sample_imu_stateful_diagnostic_with_kinematics(
            &world,
            sensor,
            &ImuSpec::default(),
            SensorNoiseKey::new(1, 2, 3, 2),
            at(0.5),
            &mut state,
            &mut kinematic_state,
        )
        .unwrap();

        // alpha x r = +2 m/s² Y and omega x (omega x r) = -2 m/s² X.
        assert_vec3_close(
            sample.truth.specific_force_m_s2,
            Vec3::new(-2.0, 11.81, 0.0),
        );
    }

    #[test]
    fn diagnostic_reports_per_axis_saturation_after_mount_kinematics() {
        let body = RigidBody {
            angular_velocity_rad_s: Vec3::new(0.0, 0.0, 2.0),
            ..RigidBody::default()
        };
        let mount = Transform3::from_translation_rotation(Vec3::new(0.5, 0.0, 0.0), Quat::IDENTITY);
        let (world, _, sensor) = mounted_imu_world(body, mount);
        let spec = ImuSpec {
            gyro_range_rad_s: 1.0,
            accel_range_m_s2: 5.0,
            ..ImuSpec::default()
        };
        let sample = sample_imu_stateful_diagnostic(
            &world,
            sensor,
            &spec,
            SensorNoiseKey::new(1, 2, 3, 1),
            at(0.0),
            &mut ImuState::default(),
        )
        .unwrap();

        assert_eq!(sample.gyro_saturated, [false, false, true]);
        assert_eq!(sample.accel_saturated, [false, true, false]);
        assert_eq!(sample.measurement.angular_velocity_rad_s.z, 1.0);
        assert_eq!(sample.measurement.linear_acceleration_m_s2.y, 5.0);
    }

    #[test]
    fn invalid_mount_fails_before_sampling() {
        let (mut world, _, sensor) = mounted_imu_world(RigidBody::default(), Transform3::IDENTITY);
        world
            .get_mut::<ImuMount>(sensor)
            .unwrap()
            .body_from_sensor
            .scale = Vec3::splat(2.0);
        let error = sample_imu_stateful_diagnostic(
            &world,
            sensor,
            &ImuSpec::default(),
            SensorNoiseKey::new(1, 2, 3, 1),
            at(0.0),
            &mut ImuState::default(),
        )
        .unwrap_err();
        assert_eq!(error, ImuSampleError::InvalidMount);
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
