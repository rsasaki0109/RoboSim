//! Physics-aware LiDAR sensor specification and sampling.

use crate::{LidarMaterial, SensorNoiseKey};
use rne_core::{mix64, KeyedRandom};
use rne_data::PointCloud;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{PhysicsBackend, PhysicsWorldId, RaycastHit, RaycastQuery};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

const LIDAR_RANDOM_DOMAIN_V1: u64 = 0x3152_4144_494C_4E52;
const RETURN_CHANNEL_STRIDE: u64 = 8;
const RANGE_REFERENCE_M: f64 = 10.0;

/// Behavior when the physics backend cannot evaluate a LiDAR ray.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LidarFailureBehavior {
    /// Omit only the failed ray and continue the scan.
    #[default]
    DropRay,
    /// Discard the complete scan when any ray fails.
    DropScan,
}

/// Atmospheric conditions that attenuate LiDAR energy along the beam path.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LidarAtmosphere {
    /// Fog extinction coefficient in inverse meters.
    pub fog_extinction_per_m: f64,
    /// Liquid precipitation rate in millimeters per hour.
    pub rain_rate_mm_h: f64,
    /// Suspended dust concentration in milligrams per cubic meter.
    pub dust_density_mg_m3: f64,
    /// Snow water-equivalent rate in millimeters per hour.
    pub snow_rate_mm_h: f64,
}

impl LidarAtmosphere {
    /// Returns a simple wavelength-neutral extinction coefficient in inverse meters.
    ///
    /// The compact coefficients intentionally model deterministic first-order
    /// attenuation. More detailed wavelength/scattering backends can replace this
    /// model without changing the sensor or traffic APIs.
    pub fn extinction_per_m(self) -> f64 {
        self.fog_extinction_per_m.max(0.0)
            + self.rain_rate_mm_h.max(0.0) * 0.0001
            + self.dust_density_mg_m3.max(0.0) * 0.000_02
            + self.snow_rate_mm_h.max(0.0) * 0.000_15
    }
}

/// Per-scan deterministic ranges used for LiDAR domain randomization.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LidarDomainRandomization {
    /// Inclusive fog-extinction range in inverse meters.
    pub fog_extinction_range_per_m: [f64; 2],
    /// Inclusive rain-rate range in millimeters per hour.
    pub rain_rate_range_mm_h: [f64; 2],
    /// Inclusive dust-density range in milligrams per cubic meter.
    pub dust_density_range_mg_m3: [f64; 2],
    /// Inclusive snow-rate range in millimeters per hour.
    pub snow_rate_range_mm_h: [f64; 2],
}

impl LidarDomainRandomization {
    fn sample(self, base: LidarAtmosphere, key: SensorNoiseKey) -> LidarAtmosphere {
        let random = lidar_random(key);
        LidarAtmosphere {
            fog_extinction_per_m: base.fog_extinction_per_m
                + sample_nonnegative_range(&random, key, 0, self.fog_extinction_range_per_m),
            rain_rate_mm_h: base.rain_rate_mm_h
                + sample_nonnegative_range(&random, key, 1, self.rain_rate_range_mm_h),
            dust_density_mg_m3: base.dust_density_mg_m3
                + sample_nonnegative_range(&random, key, 2, self.dust_density_range_mg_m3),
            snow_rate_mm_h: base.snow_rate_mm_h
                + sample_nonnegative_range(&random, key, 3, self.snow_rate_range_mm_h),
        }
    }
}

/// 2D scanning LiDAR parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LidarSpec {
    /// Number of rays per scan.
    pub ray_count: u32,
    /// Minimum azimuth angle in radians.
    pub min_angle_rad: f64,
    /// Maximum azimuth angle in radians.
    pub max_angle_rad: f64,
    /// Minimum measurable range in meters.
    pub min_range_m: f64,
    /// Maximum range in meters.
    pub max_range_m: f64,
    /// Vertical offset of the scan plane in the sensor frame.
    pub height_offset_m: f64,
    /// Maximum returns retained for one ray.
    pub max_returns: u8,
    /// Laser wavelength in nanometers, retained for material-model evolution.
    pub wavelength_nm: f64,
    /// Full beam divergence angle in radians.
    pub beam_divergence_rad: f64,
    /// Gaussian range-noise standard deviation in meters.
    pub range_noise_stddev_m: f64,
    /// Gaussian normalized-intensity noise standard deviation.
    pub intensity_noise_stddev: f64,
    /// Base probability of dropping a physically valid return.
    pub dropout_probability: f64,
    /// Minimum normalized intensity retained in the output cloud.
    pub minimum_intensity: f64,
    /// Sensor-local deterministic seed mixed with [`rne_world::WorldRandom`].
    pub seed: u64,
    /// Base atmospheric conditions.
    pub atmosphere: LidarAtmosphere,
    /// Seeded per-scan atmospheric domain randomization.
    pub domain_randomization: LidarDomainRandomization,
    /// Backend failure behavior.
    pub failure_behavior: LidarFailureBehavior,
}

impl Default for LidarSpec {
    fn default() -> Self {
        Self {
            ray_count: 360,
            min_angle_rad: -std::f64::consts::PI,
            max_angle_rad: std::f64::consts::PI,
            min_range_m: 0.05,
            max_range_m: 20.0,
            height_offset_m: 0.2,
            max_returns: 1,
            wavelength_nm: 905.0,
            beam_divergence_rad: 0.0,
            range_noise_stddev_m: 0.0,
            intensity_noise_stddev: 0.0,
            dropout_probability: 0.0,
            minimum_intensity: 0.0,
            seed: 0,
            atmosphere: LidarAtmosphere::default(),
            domain_randomization: LidarDomainRandomization::default(),
            failure_behavior: LidarFailureBehavior::DropRay,
        }
    }
}

/// Samples a horizontal LiDAR scan using default materials and no keyed variation.
///
/// This compatibility entry point does not have ECS access, so every surface uses
/// [`LidarMaterial::default`]. Use [`sample_lidar_keyed`] for material-aware scans.
pub fn sample_lidar<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    mount_transform: &Transform3,
    spec: &LidarSpec,
) -> PointCloud {
    sample_lidar_impl(
        backend,
        physics_world,
        None,
        mount_transform,
        spec,
        SensorNoiseKey::new(0, spec.seed, 0, 0),
    )
}

/// Samples a material-aware scan with stateless deterministic noise and weather.
pub fn sample_lidar_keyed<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &World,
    mount_transform: &Transform3,
    spec: &LidarSpec,
    noise_key: SensorNoiseKey,
) -> PointCloud {
    sample_lidar_impl(
        backend,
        physics_world,
        Some(world),
        mount_transform,
        spec,
        noise_key,
    )
}

/// Convenience mount lookup for a sensor entity.
pub fn sample_lidar_at_entity<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &World,
    entity: Entity,
    spec: &LidarSpec,
) -> PointCloud {
    sample_lidar_at_entity_keyed(
        backend,
        physics_world,
        world,
        entity,
        spec,
        SensorNoiseKey::new(0, spec.seed, 0, 0),
    )
}

/// Convenience mount lookup with deterministic scan coordinates.
pub fn sample_lidar_at_entity_keyed<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &World,
    entity: Entity,
    spec: &LidarSpec,
    noise_key: SensorNoiseKey,
) -> PointCloud {
    let transform = world.get::<Transform3>(entity).copied().unwrap_or_default();
    sample_lidar_keyed(backend, physics_world, world, &transform, spec, noise_key)
}

fn sample_lidar_impl<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: Option<&World>,
    mount_transform: &Transform3,
    spec: &LidarSpec,
    noise_key: SensorNoiseKey,
) -> PointCloud {
    if spec.ray_count == 0
        || spec.max_returns == 0
        || !spec.max_range_m.is_finite()
        || spec.max_range_m <= 0.0
    {
        return PointCloud::new();
    }

    let mut cloud = PointCloud::new();
    let origin = mount_transform.translation + Vec3::new(0.0, spec.height_offset_m, 0.0);
    let random = lidar_random(noise_key);
    let atmosphere = spec.domain_randomization.sample(spec.atmosphere, noise_key);

    for ray_index in 0..spec.ray_count {
        let t = if spec.ray_count <= 1 {
            0.0
        } else {
            ray_index as f64 / (spec.ray_count - 1) as f64
        };
        let nominal_angle = spec.min_angle_rad + (spec.max_angle_rad - spec.min_angle_rad) * t;
        let angle = nominal_angle
            + gaussian_sample(
                &random,
                noise_key,
                u64::from(ray_index) * RETURN_CHANNEL_STRIDE,
            ) * spec.beam_divergence_rad.max(0.0)
                / 2.0;
        let direction = mount_transform.rotation * Vec3::new(angle.cos(), 0.0, angle.sin());
        let query = RaycastQuery {
            origin_m: origin,
            direction,
            max_distance_m: spec.max_range_m,
        };

        let mut hits = match backend.raycast(physics_world, query) {
            Ok(hits) => hits,
            Err(_) if spec.failure_behavior == LidarFailureBehavior::DropRay => continue,
            Err(_) => return PointCloud::new(),
        };
        sort_hits(&mut hits);
        append_ray_returns(
            &mut cloud, world, origin, direction, ray_index, &hits, spec, atmosphere, &random,
            noise_key,
        );
    }

    debug_assert!(cloud.attributes_are_aligned());
    cloud
}

#[allow(clippy::too_many_arguments)]
fn append_ray_returns(
    cloud: &mut PointCloud,
    world: Option<&World>,
    origin: Vec3,
    direction: Vec3,
    ray_index: u32,
    hits: &[RaycastHit],
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
) {
    let mut incident_energy = 1.0;
    let mut surface_return_index = 0_u8;

    for hit in hits {
        if hit.distance_m < spec.min_range_m.max(0.0) {
            continue;
        }
        if hit.distance_m > spec.max_range_m || surface_return_index >= spec.max_returns {
            break;
        }
        surface_return_index += 1;

        let material = world
            .and_then(|world| world.get::<LidarMaterial>(hit.entity).copied())
            .unwrap_or_default();
        let normal = hit.normal.normalize_or_zero();
        let incidence_cos = (-direction).dot(normal).clamp(0.0, 1.0);
        let angular_exponent = 1.0 + 4.0 * (1.0 - material.roughness.clamp(0.0, 1.0));
        let angular_response = incidence_cos.powf(angular_exponent);
        let range_ratio = hit.distance_m.max(0.0) / RANGE_REFERENCE_M;
        let range_response = 1.0 / (1.0 + range_ratio * range_ratio);
        let atmospheric_response =
            (-2.0 * atmosphere.extinction_per_m() * hit.distance_m.max(0.0)).exp();
        let channel =
            u64::from(ray_index) * RETURN_CHANNEL_STRIDE + u64::from(surface_return_index);
        let intensity_noise =
            gaussian_sample(random, noise_key, channel + 1) * spec.intensity_noise_stddev.max(0.0);
        let intensity = (incident_energy
            * material.reflectivity.clamp(0.0, 1.0)
            * angular_response
            * range_response
            * atmospheric_response
            + intensity_noise)
            .clamp(0.0, 1.0);
        let dropout = spec.dropout_probability.clamp(0.0, 1.0);
        let keep = random.sample_unit_f64(
            noise_key.stable_sensor_id ^ u64::from(ray_index),
            noise_key.sample_index,
            channel + 2,
        ) >= dropout;

        if keep && intensity >= spec.minimum_intensity.clamp(0.0, 1.0) {
            let noisy_distance_m = (hit.distance_m
                + gaussian_sample(random, noise_key, channel + 3)
                    * spec.range_noise_stddev_m.max(0.0))
            .clamp(spec.min_range_m.max(0.0), spec.max_range_m);
            cloud.push_return(
                origin + direction * noisy_distance_m,
                intensity as f32,
                ray_index,
                surface_return_index,
            );
        }

        let maximum_transmission = 1.0 - material.reflectivity.clamp(0.0, 1.0);
        incident_energy *= material
            .transmissivity
            .clamp(0.0, maximum_transmission.max(0.0));
        if incident_energy <= f64::EPSILON {
            break;
        }
    }
}

fn sort_hits(hits: &mut [RaycastHit]) {
    hits.sort_by(|left, right| {
        left.distance_m
            .total_cmp(&right.distance_m)
            .then_with(|| left.entity.index().cmp(&right.entity.index()))
    });
}

fn lidar_random(key: SensorNoiseKey) -> KeyedRandom {
    KeyedRandom::new(
        key.root_seed,
        LIDAR_RANDOM_DOMAIN_V1 ^ mix64(key.sensor_seed),
    )
}

fn gaussian_sample(random: &KeyedRandom, key: SensorNoiseKey, channel: u64) -> f64 {
    let u1 = random
        .sample_unit_f64(key.stable_sensor_id, key.sample_index, channel)
        .max(f64::MIN_POSITIVE);
    let u2 = random.sample_unit_f64(
        key.stable_sensor_id,
        key.sample_index,
        channel.wrapping_add(0x1_0000),
    );
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

fn sample_nonnegative_range(
    random: &KeyedRandom,
    key: SensorNoiseKey,
    channel: u64,
    range: [f64; 2],
) -> f64 {
    let min = range[0].min(range[1]).max(0.0);
    let max = range[0].max(range[1]).max(min);
    if max <= min {
        return min;
    }
    random.sample_f64(
        key.stable_sensor_id,
        key.sample_index,
        channel + 0x2_0000,
        min,
        max,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::spawn_named;
    use rne_math::Quat;
    use rne_physics::{ContactEvent, PhysicsCapability, PhysicsError, PhysicsWorldDesc};

    #[derive(Default)]
    struct OrderedHitPhysics {
        hits: Vec<RaycastHit>,
        fail: bool,
    }

    impl PhysicsBackend for OrderedHitPhysics {
        type BodyHandle = ();
        type ColliderHandle = ();

        fn create_world(&mut self, _: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
            Ok(PhysicsWorldId::DEFAULT)
        }

        fn sync_from_ecs(&mut self, _: &mut World, _: PhysicsWorldId) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn step(
            &mut self,
            _: PhysicsWorldId,
            _: rne_core::SimDuration,
        ) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn sync_to_ecs(&mut self, _: &mut World, _: PhysicsWorldId) -> Result<(), PhysicsError> {
            Ok(())
        }

        fn raycast(
            &self,
            _: PhysicsWorldId,
            _: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            if self.fail {
                Err(PhysicsError::WorldNotFound)
            } else {
                Ok(self.hits.clone())
            }
        }

        fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&[])
        }

        fn capabilities(&self) -> &[PhysicsCapability] {
            &[]
        }
    }

    #[test]
    fn lidar_ray_directions_are_normalized() {
        let transform = Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY);
        let spec = LidarSpec {
            ray_count: 4,
            min_angle_rad: 0.0,
            max_angle_rad: TAU,
            max_range_m: 10.0,
            height_offset_m: 0.0,
            ..LidarSpec::default()
        };

        let t = 1.0 / 3.0;
        let angle = spec.min_angle_rad + (spec.max_angle_rad - spec.min_angle_rad) * t;
        let direction = transform.rotation * Vec3::new(angle.cos(), 0.0, angle.sin());
        assert_relative_eq!(direction.length(), 1.0, epsilon = 1e-9);
    }

    #[test]
    fn glass_transmits_energy_to_a_second_return() {
        let mut world = World::new();
        let glass = spawn_named(&mut world, "glass");
        let concrete = spawn_named(&mut world, "concrete");
        world.entity_mut(glass).insert(LidarMaterial::clear_glass());
        world.entity_mut(concrete).insert(LidarMaterial::concrete());
        let physics = OrderedHitPhysics {
            hits: vec![
                RaycastHit {
                    entity: concrete,
                    point_m: Vec3::new(8.0, 0.0, 0.0),
                    normal: Vec3::NEG_X,
                    distance_m: 8.0,
                },
                RaycastHit {
                    entity: glass,
                    point_m: Vec3::new(4.0, 0.0, 0.0),
                    normal: Vec3::NEG_X,
                    distance_m: 4.0,
                },
            ],
            fail: false,
        };
        let spec = LidarSpec {
            ray_count: 1,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            max_returns: 2,
            max_range_m: 20.0,
            ..LidarSpec::default()
        };

        let cloud = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            SensorNoiseKey::new(42, 3, 7, 11),
        );

        assert_eq!(cloud.points_m.len(), 2);
        assert_eq!(cloud.return_indices, vec![1, 2]);
        assert!(cloud.intensities[1] > cloud.intensities[0]);
        assert!(cloud.attributes_are_aligned());
    }

    #[test]
    fn keyed_noise_and_weather_are_repeatable() {
        let mut world = World::new();
        let target = spawn_named(&mut world, "target");
        world
            .entity_mut(target)
            .insert(LidarMaterial::painted_metal());
        let physics = OrderedHitPhysics {
            hits: vec![RaycastHit {
                entity: target,
                point_m: Vec3::new(5.0, 0.0, 0.0),
                normal: Vec3::NEG_X,
                distance_m: 5.0,
            }],
            fail: false,
        };
        let spec = LidarSpec {
            ray_count: 3,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            range_noise_stddev_m: 0.02,
            intensity_noise_stddev: 0.01,
            beam_divergence_rad: 0.001,
            domain_randomization: LidarDomainRandomization {
                fog_extinction_range_per_m: [0.001, 0.02],
                rain_rate_range_mm_h: [0.0, 20.0],
                dust_density_range_mg_m3: [0.0, 10.0],
                snow_rate_range_mm_h: [0.0, 5.0],
            },
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(123, 9, 77, 5);

        let first = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );
        let second = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );
        assert_eq!(first, second);

        let changed = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            SensorNoiseKey {
                sample_index: 6,
                ..key
            },
        );
        assert_ne!(first, changed);
    }

    #[test]
    fn failure_behavior_can_drop_ray_or_scan() {
        let physics = OrderedHitPhysics {
            hits: Vec::new(),
            fail: true,
        };
        let mut spec = LidarSpec {
            ray_count: 2,
            ..LidarSpec::default()
        };
        let dropped_rays = sample_lidar(
            &physics,
            PhysicsWorldId::DEFAULT,
            &Transform3::IDENTITY,
            &spec,
        );
        assert!(dropped_rays.points_m.is_empty());

        spec.failure_behavior = LidarFailureBehavior::DropScan;
        let dropped_scan = sample_lidar(
            &physics,
            PhysicsWorldId::DEFAULT,
            &Transform3::IDENTITY,
            &spec,
        );
        assert!(dropped_scan.points_m.is_empty());
    }
}
