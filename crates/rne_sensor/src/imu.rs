//! IMU sensor specification and sampling.

use crate::noise::{NoiseModel, SensorNoiseKey};
use rne_data::ImuSample;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::RigidBody;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// IMU sensor parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuSpec {
    /// Optional noise model.
    pub noise: NoiseModel,
    /// Deterministic noise seed.
    pub seed: u64,
}

/// Samples an IMU attached to the given entity.
pub fn sample_imu(world: &World, entity: Entity, spec: &ImuSpec) -> ImuSample {
    let (angular, linear) = sample_imu_raw(world, entity);
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
    let (angular, linear) = sample_imu_raw(world, entity);
    let (angular, linear) = spec.noise.apply_imu_keyed(angular, linear, noise_key);

    ImuSample {
        angular_velocity_rad_s: angular,
        linear_acceleration_m_s2: linear,
    }
}

fn sample_imu_raw(world: &World, entity: Entity) -> (Vec3, Vec3) {
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let (angular, mut linear) = world
        .get::<RigidBody>(entity)
        .map(|body| (body.angular_velocity_rad_s, body.linear_velocity_m_s))
        .unwrap_or((Vec3::ZERO, Vec3::ZERO));

    if world.get::<Transform3>(entity).is_some() {
        linear += gravity;
    }

    (angular, linear)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::spawn_named;

    #[test]
    fn static_imu_reports_gravity() {
        let mut world = World::new();
        let sensor = spawn_named(&mut world, "imu");
        world
            .entity_mut(sensor)
            .insert((Transform3::default(), RigidBody::default()));

        let sample = sample_imu(&world, sensor, &ImuSpec::default());
        assert!((sample.linear_acceleration_m_s2.y + 9.81).abs() < 1e-6);
    }
}
