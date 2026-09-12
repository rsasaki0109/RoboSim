//! Fixed-seed statistical checks of actual IMU outputs, not a replacement generator.

use crate::{
    allan::{overlapping_allan_deviation, AllanSample},
    sample_imu_stateful, ImuAxisErrors, ImuSpec, ImuState, SensorNoiseKey,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, World};
use rne_physics::RigidBody;
use rne_world::Transform3;

const FACTORS: [usize; 4] = [1, 4, 16, 64];
const SEEDS: [u64; 4] = [11, 29, 47, 83];
const SAMPLES: usize = 8192;
const SCALE: f64 = 0.02;
const CORRELATION_S: f64 = 0.2;
// Frozen before executing these stochastic checks. This is a regression envelope,
// not a confidence level or a physical calibration acceptance bound.
const MAX_RELATIVE_ERROR: f64 = 0.25;

fn gm_variance(m: usize, dt: f64) -> f64 {
    // Independent covariance-matrix oracle for two adjacent averages. It uses
    // the stationary AR(1) covariance, not the production prefix-sum algorithm.
    let mut sum = 0.0;
    for i in 0..2 * m {
        for j in 0..2 * m {
            let sign = if (i < m) == (j < m) { 1.0 } else { -1.0 };
            sum += sign * (-(i.abs_diff(j) as f64) * dt / CORRELATION_S).exp();
        }
    }
    SCALE * SCALE * sum / (2.0 * (m * m) as f64)
}

#[test]
fn allan_actual_imu_white_walk_and_markov_match_discrete_expectations() {
    let mut world = World::new();
    let sensor = spawn_named(&mut world, "allan_imu");
    world
        .entity_mut(sensor)
        .insert((Transform3::IDENTITY, RigidBody::default()));
    for period_ticks in [10_000_000, 40_000_000] {
        let period = SimDuration::from_ticks(period_ticks);
        let dt = period.as_seconds().value();
        for kind in 0..3 {
            let errors = match kind {
                0 => ImuAxisErrors {
                    random_walk: SCALE,
                    ..Default::default()
                },
                1 => ImuAxisErrors {
                    rate_random_walk: SCALE,
                    ..Default::default()
                },
                _ => ImuAxisErrors {
                    bias_instability: SCALE,
                    bias_correlation_time_s: CORRELATION_S,
                    ..Default::default()
                },
            };
            let spec = ImuSpec {
                gyro: errors,
                accel: errors,
                ..Default::default()
            };
            let mut variances = [[0.0; 4]; 6];
            for seed in SEEDS {
                let mut state = ImuState::default();
                let mut channels: [Vec<AllanSample>; 6] =
                    std::array::from_fn(|_| Vec::with_capacity(SAMPLES));
                // Sample at zero to initialize GM stationarily. Exclude the
                // startup output: its white-noise dt=0 fallback is not density-scaled.
                for index in 0..=SAMPLES {
                    let ticks = index as u64 * period_ticks;
                    let output = sample_imu_stateful(
                        &world,
                        sensor,
                        &spec,
                        SensorNoiseKey::new(seed, 7, 3, index as u64),
                        SimTime::from_ticks(ticks),
                        &mut state,
                    );
                    if index == 0 {
                        continue;
                    }
                    let g = output.angular_velocity_rad_s;
                    let a = output.linear_acceleration_m_s2;
                    for (channel, value) in channels.iter_mut().zip([g.x, g.y, g.z, a.x, a.y, a.z])
                    {
                        channel.push(AllanSample {
                            capture_ticks: ticks,
                            value,
                        });
                    }
                }
                for (axis, channel) in channels.iter().enumerate() {
                    for (index, point) in overlapping_allan_deviation(channel, period, &FACTORS)
                        .unwrap()
                        .iter()
                        .enumerate()
                    {
                        variances[axis][index] += point.variance / SEEDS.len() as f64;
                        assert_eq!(point.pair_count, SAMPLES - 2 * FACTORS[index] + 1);
                    }
                }
            }
            for (axis, values) in variances.iter().enumerate() {
                for (index, &m) in FACTORS.iter().enumerate() {
                    let expected = match kind {
                        0 => SCALE * SCALE / (m as f64 * dt),
                        1 => SCALE * SCALE * dt * (2.0 * (m * m) as f64 + 1.0) / (6.0 * m as f64),
                        _ => gm_variance(m, dt),
                    };
                    let relative = (values[index] / expected - 1.0).abs();
                    assert!(relative<MAX_RELATIVE_ERROR,
                        "kind={kind} dt={dt} axis={axis} m={m} observed={} expected={expected} relative={relative}",values[index]);
                }
            }
        }
    }
}
