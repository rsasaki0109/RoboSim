//! Physics-aware LiDAR sensor specification and sampling.
//!
//! The model is renderer-independent and deterministic. Every stochastic effect is
//! drawn from an explicit [`SensorNoiseKey`] slot, so a given key always reproduces
//! the same scan.
//!
//! # Radiometry
//!
//! Returned energy follows the standard single-scattering LiDAR equation reduced to
//! the terms a simulator can evaluate from geometry and material properties:
//!
//! ```text
//! I = rho * f(theta) * G_retro(theta) * (r_ref / r)^2 * overlap(r) * exp(-2 * sigma * r)
//! ```
//!
//! * `rho` — [`LidarMaterial::reflectivity`].
//! * `f(theta)` — angular response; smooth surfaces fall off faster than rough ones.
//! * `G_retro` — retroreflective gain with an entrance-angle falloff.
//! * `(r_ref / r)^2` — inverse-square spreading, normalized so a Lambertian unit
//!   reflector at [`RANGE_REFERENCE_M`] under normal incidence returns `1.0`.
//! * `overlap(r)` — the transmitter/receiver geometric form factor, which suppresses
//!   very close returns. The crossover range is the configured minimum range.
//! * `exp(-2 * sigma * r)` — two-way atmospheric extinction.
//!
//! Energy above [`LidarSpec::saturation_intensity`] is clipped, and the clipped
//! excess optionally blooms into neighbouring azimuth columns.
//!
//! # Scan geometry
//!
//! A scan is a grid of `ray_count` azimuth columns by [`LidarSpec::channel_count`]
//! elevation channels. Columns are emitted sequentially over
//! [`LidarSpec::rotation_period_s`], so a moving sensor produces a motion-distorted
//! cloud: each column is cast from the interpolated pose of the [`LidarSweep`] at its
//! own emission time, and every point carries that time in
//! [`rne_data::PointCloud::timestamps_s`].

use crate::{LidarMaterial, SensorNoiseKey};
use rne_core::{mix64, KeyedRandom};
use rne_data::PointCloud;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{PhysicsBackend, PhysicsWorldId, RaycastHit, RaycastQuery};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

const LIDAR_RANDOM_DOMAIN_V1: u64 = 0x3152_4144_494C_4E52;

/// Range at which a Lambertian unit reflector under normal incidence returns `1.0`.
pub const RANGE_REFERENCE_M: f64 = 10.0;

/// Random slots reserved for each ray.
const RAY_SLOT_STRIDE: u64 = 256;
/// Ray-local slot for the beam pointing jitter (consumes two slots).
const SLOT_POINTING_JITTER: u64 = 0;
/// Ray-local slot for the beam footprint sampling phase.
const SLOT_FOOTPRINT_PHASE: u64 = 2;
/// Ray-local slot deciding whether an aerosol backscatter return occurs.
const SLOT_BACKSCATTER_EVENT: u64 = 3;
/// Ray-local slot selecting the aerosol backscatter range.
const SLOT_BACKSCATTER_RANGE: u64 = 4;
/// First ray-local slot of the per-return block.
const SLOT_RETURN_BASE: u64 = 16;
/// Ray-local slots consumed by one return.
const SLOT_RETURN_STRIDE: u64 = 8;
/// Return-local slot for intensity noise (consumes two slots).
const SLOT_RETURN_INTENSITY_NOISE: u64 = 0;
/// Return-local slot for the detection dropout draw.
const SLOT_RETURN_DROPOUT: u64 = 2;
/// Return-local slot for range noise (consumes two slots).
const SLOT_RETURN_RANGE_NOISE: u64 = 3;
/// Return-local slot for the solar noise floor (consumes two slots).
const SLOT_RETURN_SOLAR_NOISE: u64 = 5;
/// Scan-level slots live above every ray-local slot.
const SCAN_SLOT_BASE: u64 = 1 << 60;

/// Entrance-angle exponent of retroreflective sheeting.
const RETRO_ENTRANCE_EXPONENT: f64 = 8.0;
/// Intensity retained by a return whose footprint straddles a depth discontinuity.
const MIXED_PIXEL_INTENSITY_PENALTY: f64 = 0.45;
/// Fraction of the extinction coefficient that scatters energy back to the detector.
const BACKSCATTER_ALBEDO: f64 = 0.35;
/// Discrete occlusion coefficient per millimeter per hour of rain, per meter.
const RAIN_OCCLUSION_PER_MM_H_M: f64 = 0.000_28;
/// Discrete occlusion coefficient per millimeter per hour of snow, per meter.
const SNOW_OCCLUSION_PER_MM_H_M: f64 = 0.000_65;
/// Golden angle used to spread beam footprint samples.
const GOLDEN_ANGLE_RAD: f64 = PI * (3.0 - 2.236_067_977_499_79);

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

    /// Returns the per-meter probability coefficient of a single pulse being blocked.
    ///
    /// [`Self::extinction_per_m`] is the ensemble-average attenuation. Rain and snow
    /// additionally consist of particles large enough to occlude a whole pulse, which
    /// shows up as isolated missing returns rather than as a uniform intensity loss.
    /// Fog and dust particles are too small for that and are excluded here.
    pub fn occlusion_per_m(self) -> f64 {
        self.rain_rate_mm_h.max(0.0) * RAIN_OCCLUSION_PER_MM_H_M
            + self.snow_rate_mm_h.max(0.0) * SNOW_OCCLUSION_PER_MM_H_M
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

/// Sensor pose at the start and end of one scan revolution.
///
/// A spinning LiDAR does not capture a scan instantaneously. Casting every azimuth
/// column from the sensor pose interpolated across the sweep reproduces the motion
/// distortion that real point clouds exhibit while the platform moves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarSweep {
    /// Sensor pose when the first azimuth column is emitted.
    pub start: Transform3,
    /// Sensor pose when the scan completes.
    pub end: Transform3,
}

impl LidarSweep {
    /// Creates a sweep between two sensor poses.
    pub fn new(start: Transform3, end: Transform3) -> Self {
        Self { start, end }
    }

    /// Creates a sweep for a sensor that does not move during the scan.
    pub fn stationary(pose: Transform3) -> Self {
        Self {
            start: pose,
            end: pose,
        }
    }

    /// Returns true when the sensor pose is identical across the sweep.
    pub fn is_stationary(&self) -> bool {
        self.start == self.end
    }

    /// Returns the interpolated sensor pose at `fraction` through the sweep.
    pub fn pose_at(&self, fraction: f64) -> Transform3 {
        if self.is_stationary() {
            return self.start;
        }
        let fraction = fraction.clamp(0.0, 1.0);
        Transform3 {
            translation: self.start.translation.lerp(self.end.translation, fraction),
            rotation: self.start.rotation.slerp(self.end.rotation, fraction),
            scale: self.start.scale.lerp(self.end.scale, fraction),
        }
    }
}

/// Scanning LiDAR parameters.
///
/// Defaults describe a single-plane 360-degree scanner with no noise, which keeps
/// legacy 2D configurations behaving as before. Set [`Self::channel_count`] and the
/// elevation limits to model a multi-channel 3D sensor.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LidarSpec {
    /// Number of azimuth columns per scan.
    pub ray_count: u32,
    /// Minimum azimuth angle in radians.
    pub min_angle_rad: f64,
    /// Maximum azimuth angle in radians.
    pub max_angle_rad: f64,
    /// Number of elevation channels (rings) per azimuth column.
    pub channel_count: u16,
    /// Minimum elevation angle in radians; negative points below the scan axis.
    pub min_elevation_rad: f64,
    /// Maximum elevation angle in radians.
    pub max_elevation_rad: f64,
    /// Time to sweep every azimuth column, in seconds.
    ///
    /// `0.0` models an instantaneous scan: every point shares timestamp `0.0` and no
    /// motion distortion is produced even for a moving [`LidarSweep`].
    pub rotation_period_s: f64,
    /// Minimum measurable range in meters, also used as the near-field crossover range.
    pub min_range_m: f64,
    /// Maximum range in meters.
    pub max_range_m: f64,
    /// Vertical offset of the scan origin in the world frame.
    pub height_offset_m: f64,
    /// Maximum returns retained for one ray.
    pub max_returns: u8,
    /// Laser wavelength in nanometers, retained for material-model evolution.
    pub wavelength_nm: f64,
    /// Full beam divergence angle in radians.
    pub beam_divergence_rad: f64,
    /// Number of sub-samples used to integrate the beam footprint.
    ///
    /// `1` keeps the cheap model in which divergence is a Gaussian pointing jitter.
    /// Values above `1` cast additional rays across the divergence cone, which lets
    /// footprints that straddle a depth discontinuity produce mixed-pixel returns at
    /// a blended range — the artifact that puts stray points on object silhouettes.
    pub beam_sample_count: u8,
    /// Footprint depth spread above which a return is treated as a mixed pixel.
    pub mixed_pixel_threshold_m: f64,
    /// Gaussian range-noise standard deviation in meters.
    pub range_noise_stddev_m: f64,
    /// Gaussian normalized-intensity noise standard deviation.
    pub intensity_noise_stddev: f64,
    /// Additive ambient noise floor caused by background solar illumination.
    pub solar_noise_floor: f64,
    /// Base probability of dropping a physically valid return.
    pub dropout_probability: f64,
    /// Normalized intensity at which the detector saturates.
    pub saturation_intensity: f64,
    /// Fraction of clipped excess energy that blooms into neighbouring columns.
    pub bloom_gain: f64,
    /// Azimuth-column radius over which saturated returns bloom.
    pub bloom_column_radius: u16,
    /// Scale factor for aerosol backscatter returns; `0.0` disables them.
    pub backscatter_probability_scale: f64,
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
            min_angle_rad: -PI,
            max_angle_rad: PI,
            channel_count: 1,
            min_elevation_rad: 0.0,
            max_elevation_rad: 0.0,
            rotation_period_s: 0.0,
            min_range_m: 0.05,
            max_range_m: 20.0,
            height_offset_m: 0.2,
            max_returns: 1,
            wavelength_nm: 905.0,
            beam_divergence_rad: 0.0,
            beam_sample_count: 1,
            mixed_pixel_threshold_m: 0.25,
            range_noise_stddev_m: 0.0,
            intensity_noise_stddev: 0.0,
            solar_noise_floor: 0.0,
            dropout_probability: 0.0,
            saturation_intensity: 1.0,
            bloom_gain: 0.0,
            bloom_column_radius: 0,
            backscatter_probability_scale: 0.0,
            minimum_intensity: 0.0,
            seed: 0,
            atmosphere: LidarAtmosphere::default(),
            domain_randomization: LidarDomainRandomization::default(),
            failure_behavior: LidarFailureBehavior::DropRay,
        }
    }
}

impl LidarSpec {
    /// Returns the number of elevation channels, at least one.
    pub fn effective_channel_count(&self) -> u16 {
        self.channel_count.max(1)
    }

    /// Returns the elevation angle of `channel` in radians.
    pub fn channel_elevation_rad(&self, channel: u16) -> f64 {
        let channels = self.effective_channel_count();
        if channels <= 1 {
            return self.min_elevation_rad;
        }
        let t = f64::from(channel.min(channels - 1)) / f64::from(channels - 1);
        self.min_elevation_rad + (self.max_elevation_rad - self.min_elevation_rad) * t
    }

    /// Returns the emission time of azimuth column `column` relative to scan start.
    pub fn column_time_s(&self, column: u32) -> f64 {
        if self.ray_count == 0 || self.rotation_period_s <= 0.0 {
            return 0.0;
        }
        self.rotation_period_s * (f64::from(column) / f64::from(self.ray_count))
    }

    /// Returns the total number of rays cast per scan.
    pub fn rays_per_scan(&self) -> u64 {
        u64::from(self.ray_count) * u64::from(self.effective_channel_count())
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
        &LidarSweep::stationary(*mount_transform),
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
        &LidarSweep::stationary(*mount_transform),
        spec,
        noise_key,
    )
}

/// Samples a scan whose azimuth columns are cast from a moving sensor pose.
///
/// Use this when the platform moves appreciably within
/// [`LidarSpec::rotation_period_s`]: the resulting cloud carries the same motion
/// distortion a real spinning scanner produces, and every point records its emission
/// time so downstream code can undistort it.
pub fn sample_lidar_swept<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &World,
    sweep: &LidarSweep,
    spec: &LidarSpec,
    noise_key: SensorNoiseKey,
) -> PointCloud {
    sample_lidar_impl(backend, physics_world, Some(world), sweep, spec, noise_key)
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

/// One detected return before bloom, thresholding and clamping are applied.
#[derive(Clone, Copy, Debug)]
struct PendingReturn {
    point_m: Vec3,
    raw_intensity: f64,
    column: u32,
    channel: u16,
    return_index: u8,
    timestamp_s: f64,
}

/// Geometry of one ray within the scan grid.
#[derive(Clone, Copy, Debug)]
struct RayGeometry {
    origin_m: Vec3,
    direction: Vec3,
    column: u32,
    channel: u16,
    ordinal: u64,
    timestamp_s: f64,
}

fn sample_lidar_impl<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: Option<&World>,
    sweep: &LidarSweep,
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

    let random = lidar_random(noise_key);
    let atmosphere = spec.domain_randomization.sample(spec.atmosphere, noise_key);
    let channels = spec.effective_channel_count();
    let mut pending: Vec<PendingReturn> = Vec::new();

    for column in 0..spec.ray_count {
        let sweep_fraction = f64::from(column) / f64::from(spec.ray_count);
        let pose = sweep.pose_at(sweep_fraction);
        let origin_m = pose.translation + Vec3::new(0.0, spec.height_offset_m, 0.0);
        let azimuth = column_azimuth_rad(spec, column);
        let timestamp_s = spec.column_time_s(column);

        for channel in 0..channels {
            let ordinal = u64::from(column) * u64::from(channels) + u64::from(channel);
            let direction =
                ray_direction(spec, &pose, azimuth, channel, &random, noise_key, ordinal);
            let Some(direction) = direction else {
                continue;
            };
            let ray = RayGeometry {
                origin_m,
                direction,
                column,
                channel,
                ordinal,
                timestamp_s,
            };

            match evaluate_ray(
                backend,
                physics_world,
                world,
                &ray,
                spec,
                atmosphere,
                &random,
                noise_key,
            ) {
                Ok(returns) => pending.extend(returns),
                Err(RayFailure::Skip) => continue,
                Err(RayFailure::Scan) => return PointCloud::new(),
            }
        }
    }

    apply_bloom(&mut pending, spec);

    let mut cloud = PointCloud::new();
    let minimum_intensity = spec.minimum_intensity.clamp(0.0, 1.0);
    let saturation = saturation_intensity(spec);
    for entry in pending {
        let intensity = entry.raw_intensity.clamp(0.0, saturation);
        if intensity < minimum_intensity {
            continue;
        }
        cloud.push_return(
            entry.point_m,
            intensity as f32,
            entry.column,
            entry.return_index,
            entry.channel,
            entry.timestamp_s,
        );
    }

    debug_assert!(cloud.attributes_are_aligned());
    cloud
}

/// Why a ray produced no returns.
enum RayFailure {
    /// Skip this ray and continue the scan.
    Skip,
    /// Abandon the whole scan.
    Scan,
}

#[allow(clippy::too_many_arguments)]
fn evaluate_ray<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: Option<&World>,
    ray: &RayGeometry,
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
) -> Result<Vec<PendingReturn>, RayFailure> {
    let mut hits = match backend.raycast(physics_world, raycast_query(ray, spec)) {
        Ok(hits) => hits,
        Err(_) if spec.failure_behavior == LidarFailureBehavior::DropRay => {
            return Err(RayFailure::Skip)
        }
        Err(_) => return Err(RayFailure::Scan),
    };
    sort_hits(&mut hits);

    let footprint = sample_beam_footprint(backend, physics_world, ray, spec, random, noise_key)?;
    let mut returns = surface_returns(
        world, ray, &hits, footprint, spec, atmosphere, random, noise_key,
    );

    if let Some(backscatter) =
        backscatter_return(ray, &returns, spec, atmosphere, random, noise_key)
    {
        returns.push(backscatter);
    }

    returns.sort_by(|left, right| left.range_m.total_cmp(&right.range_m));
    returns.truncate(usize::from(spec.max_returns));

    Ok(returns
        .into_iter()
        .enumerate()
        .map(|(index, candidate)| PendingReturn {
            point_m: ray.origin_m + ray.direction * candidate.range_m,
            raw_intensity: candidate.intensity,
            column: ray.column,
            channel: ray.channel,
            return_index: (index + 1) as u8,
            timestamp_s: ray.timestamp_s,
        })
        .collect())
}

/// A return candidate before it is ordered and indexed within its ray.
#[derive(Clone, Copy, Debug)]
struct ReturnCandidate {
    range_m: f64,
    intensity: f64,
}

/// Aggregated result of integrating the beam footprint.
#[derive(Clone, Copy, Debug, Default)]
struct BeamFootprint {
    /// Energy-weighted mean range across footprint samples, when available.
    blended_range_m: Option<f64>,
    /// Fraction of footprint samples that returned energy.
    fill_fraction: f64,
    /// True when the footprint straddles a depth discontinuity.
    mixed: bool,
}

fn raycast_query(ray: &RayGeometry, spec: &LidarSpec) -> RaycastQuery {
    RaycastQuery {
        origin_m: ray.origin_m,
        direction: ray.direction,
        max_distance_m: spec.max_range_m,
    }
}

fn column_azimuth_rad(spec: &LidarSpec, column: u32) -> f64 {
    let t = if spec.ray_count <= 1 {
        0.0
    } else {
        f64::from(column) / f64::from(spec.ray_count - 1)
    };
    spec.min_angle_rad + (spec.max_angle_rad - spec.min_angle_rad) * t
}

fn ray_direction(
    spec: &LidarSpec,
    pose: &Transform3,
    azimuth: f64,
    channel: u16,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
    ordinal: u64,
) -> Option<Vec3> {
    let elevation = spec.channel_elevation_rad(channel);
    let (azimuth, elevation) = if spec.beam_sample_count <= 1 {
        // Cheap model: divergence acts as an uncorrelated pointing jitter.
        let half_divergence = spec.beam_divergence_rad.max(0.0) / 2.0;
        let jitter = gaussian_pair(random, noise_key, ray_slot(ordinal, SLOT_POINTING_JITTER));
        (
            azimuth + jitter.0 * half_divergence,
            elevation + jitter.1 * half_divergence,
        )
    } else {
        (azimuth, elevation)
    };

    let local = Vec3::new(
        elevation.cos() * azimuth.cos(),
        elevation.sin(),
        elevation.cos() * azimuth.sin(),
    );
    let direction = pose.rotation * local;
    let direction = direction.normalize_or_zero();
    (direction != Vec3::ZERO).then_some(direction)
}

/// Integrates the beam footprint by casting sub-rays across the divergence cone.
fn sample_beam_footprint<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    ray: &RayGeometry,
    spec: &LidarSpec,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
) -> Result<BeamFootprint, RayFailure> {
    let samples = u32::from(spec.beam_sample_count);
    let half_divergence = spec.beam_divergence_rad.max(0.0) / 2.0;
    if samples <= 1 || half_divergence <= 0.0 {
        return Ok(BeamFootprint::default());
    }

    let (right, up) = beam_basis(ray.direction);
    let phase0 = TAU
        * random.sample_unit_f64(
            noise_key.stable_sensor_id,
            noise_key.sample_index,
            ray_slot(ray.ordinal, SLOT_FOOTPRINT_PHASE),
        );
    let min_range = spec.min_range_m.max(0.0);

    let mut ranges: Vec<f64> = Vec::with_capacity(samples as usize);
    for sample in 0..samples {
        let direction = if sample == 0 {
            ray.direction
        } else {
            let radial = (f64::from(sample) / f64::from(samples - 1)).sqrt() * half_divergence;
            let phase = phase0 + f64::from(sample) * GOLDEN_ANGLE_RAD;
            // Small-angle offset: divergence is milliradian-scale in practice.
            let offset = right * (radial * phase.cos()) + up * (radial * phase.sin());
            (ray.direction + offset).normalize_or_zero()
        };
        if direction == Vec3::ZERO {
            continue;
        }

        let hits = backend.raycast(
            physics_world,
            RaycastQuery {
                origin_m: ray.origin_m,
                direction,
                max_distance_m: spec.max_range_m,
            },
        );
        let hits = match hits {
            Ok(hits) => hits,
            Err(_) if spec.failure_behavior == LidarFailureBehavior::DropRay => {
                return Err(RayFailure::Skip)
            }
            Err(_) => return Err(RayFailure::Scan),
        };
        if let Some(nearest) = hits
            .iter()
            .map(|hit| hit.distance_m)
            .filter(|distance| *distance >= min_range && *distance <= spec.max_range_m)
            .fold(None, |best: Option<f64>, distance| {
                Some(best.map_or(distance, |best| best.min(distance)))
            })
        {
            ranges.push(nearest);
        }
    }

    if ranges.is_empty() {
        return Ok(BeamFootprint {
            blended_range_m: None,
            fill_fraction: 0.0,
            mixed: false,
        });
    }

    // Weight by returned energy so the nearer surface dominates a mixed pixel.
    let mut weight_sum = 0.0;
    let mut weighted_range = 0.0;
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for range in &ranges {
        let weight = 1.0 / range.max(f64::MIN_POSITIVE).powi(2);
        weight_sum += weight;
        weighted_range += weight * range;
        minimum = minimum.min(*range);
        maximum = maximum.max(*range);
    }

    Ok(BeamFootprint {
        blended_range_m: Some(weighted_range / weight_sum),
        fill_fraction: ranges.len() as f64 / f64::from(samples),
        mixed: (maximum - minimum) > spec.mixed_pixel_threshold_m.max(0.0),
    })
}

/// Returns an orthonormal basis spanning the plane perpendicular to `axis`.
fn beam_basis(axis: Vec3) -> (Vec3, Vec3) {
    let helper = if axis.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let right = axis.cross(helper).normalize_or_zero();
    let right = if right == Vec3::ZERO { Vec3::X } else { right };
    (right, axis.cross(right).normalize_or_zero())
}

#[allow(clippy::too_many_arguments)]
fn surface_returns(
    world: Option<&World>,
    ray: &RayGeometry,
    hits: &[RaycastHit],
    footprint: BeamFootprint,
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
) -> Vec<ReturnCandidate> {
    let mut candidates = Vec::new();
    let mut incident_energy = 1.0;
    let mut surface_return_index = 0_u8;
    let min_range = spec.min_range_m.max(0.0);

    for hit in hits {
        if hit.distance_m < min_range {
            continue;
        }
        if hit.distance_m > spec.max_range_m || surface_return_index >= spec.max_returns {
            break;
        }
        surface_return_index += 1;
        let is_first_return = surface_return_index == 1;
        let slot = return_slot(ray.ordinal, surface_return_index);

        let material = world
            .and_then(|world| world.get::<LidarMaterial>(hit.entity).copied())
            .unwrap_or_default();

        // A mixed pixel reports the energy-weighted blend of the surfaces its
        // footprint covers, not the range of the axial hit.
        let geometric_range_m = if is_first_return {
            footprint.blended_range_m.unwrap_or(hit.distance_m)
        } else {
            hit.distance_m
        };

        let mut intensity = surface_intensity(
            incident_energy,
            material,
            hit,
            ray.direction,
            geometric_range_m,
            spec,
            atmosphere,
        );
        if is_first_return {
            if footprint.fill_fraction > 0.0 {
                intensity *= footprint.fill_fraction;
            }
            if footprint.mixed {
                intensity *= MIXED_PIXEL_INTENSITY_PENALTY;
            }
        }

        intensity += gaussian_pair(random, noise_key, slot + SLOT_RETURN_INTENSITY_NOISE).0
            * spec.intensity_noise_stddev.max(0.0);
        intensity += gaussian_pair(random, noise_key, slot + SLOT_RETURN_SOLAR_NOISE)
            .0
            .abs()
            * spec.solar_noise_floor.max(0.0);

        if intensity > 0.0
            && detects_pulse(geometric_range_m, spec, atmosphere, random, noise_key, slot)
        {
            let noisy_range_m = (geometric_range_m
                + gaussian_pair(random, noise_key, slot + SLOT_RETURN_RANGE_NOISE).0
                    * spec.range_noise_stddev_m.max(0.0))
            .clamp(min_range, spec.max_range_m);
            candidates.push(ReturnCandidate {
                range_m: noisy_range_m,
                intensity,
            });
        }

        let maximum_transmission = 1.0 - material.reflectivity.clamp(0.0, 1.0);
        incident_energy *= material
            .transmissivity
            .clamp(0.0, maximum_transmission.max(0.0));
        if incident_energy <= f64::EPSILON {
            break;
        }
    }

    candidates
}

fn surface_intensity(
    incident_energy: f64,
    material: LidarMaterial,
    hit: &RaycastHit,
    direction: Vec3,
    range_m: f64,
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
) -> f64 {
    let normal = hit.normal.normalize_or_zero();
    let incidence_cos = (-direction).dot(normal).clamp(0.0, 1.0);
    let angular_exponent = 1.0 + 4.0 * (1.0 - material.roughness.clamp(0.0, 1.0));
    let angular_response = incidence_cos.powf(angular_exponent);
    let retro_gain = if material.retroreflective_gain.is_finite() {
        material.retroreflective_gain.max(1.0)
    } else {
        1.0
    };
    let retro_response = 1.0 + (retro_gain - 1.0) * incidence_cos.powf(RETRO_ENTRANCE_EXPONENT);
    let atmospheric_response = (-2.0 * atmosphere.extinction_per_m() * range_m.max(0.0)).exp();

    incident_energy
        * material.reflectivity.clamp(0.0, 1.0)
        * angular_response
        * retro_response
        * range_response(range_m, spec)
        * atmospheric_response
}

/// Inverse-square spreading combined with the transmit/receive overlap form factor.
fn range_response(range_m: f64, spec: &LidarSpec) -> f64 {
    let range_m = range_m.max(f64::MIN_POSITIVE);
    let crossover_m = spec.min_range_m.max(1e-3);
    let overlap = 1.0 - (-(range_m / crossover_m).powi(2)).exp();
    (RANGE_REFERENCE_M / range_m).powi(2) * overlap
}

/// Returns false when a discrete precipitation particle or a dropout swallows the pulse.
fn detects_pulse(
    range_m: f64,
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
    slot: u64,
) -> bool {
    let base_dropout = spec.dropout_probability.clamp(0.0, 1.0);
    let occlusion = 1.0 - (-atmosphere.occlusion_per_m() * range_m.max(0.0)).exp();
    let drop_probability = (base_dropout + (1.0 - base_dropout) * occlusion).clamp(0.0, 1.0);
    let draw = random.sample_unit_f64(
        noise_key.stable_sensor_id,
        noise_key.sample_index,
        slot + SLOT_RETURN_DROPOUT,
    );
    draw >= drop_probability
}

/// Returns an aerosol backscatter return sampled along the free path of the beam.
fn backscatter_return(
    ray: &RayGeometry,
    surface: &[ReturnCandidate],
    spec: &LidarSpec,
    atmosphere: LidarAtmosphere,
    random: &KeyedRandom,
    noise_key: SensorNoiseKey,
) -> Option<ReturnCandidate> {
    let scale = spec.backscatter_probability_scale.max(0.0);
    let sigma = atmosphere.extinction_per_m();
    if scale <= 0.0 || sigma <= 0.0 {
        return None;
    }

    // Particles beyond the first hard surface are not illuminated.
    let reach_m = surface
        .iter()
        .map(|candidate| candidate.range_m)
        .fold(spec.max_range_m, f64::min)
        .clamp(spec.min_range_m.max(0.0), spec.max_range_m);
    let optical_depth = (1.0 - (-sigma * reach_m).exp()).clamp(0.0, 1.0);
    let probability = (scale * optical_depth).clamp(0.0, 1.0);

    let event = random.sample_unit_f64(
        noise_key.stable_sensor_id,
        noise_key.sample_index,
        ray_slot(ray.ordinal, SLOT_BACKSCATTER_EVENT),
    );
    if event >= probability {
        return None;
    }

    // Invert the truncated exponential free-path distribution.
    let uniform = random.sample_unit_f64(
        noise_key.stable_sensor_id,
        noise_key.sample_index,
        ray_slot(ray.ordinal, SLOT_BACKSCATTER_RANGE),
    );
    let range_m = (-(1.0 - uniform * optical_depth).max(f64::MIN_POSITIVE).ln() / sigma)
        .clamp(spec.min_range_m.max(0.0), reach_m);

    let intensity =
        BACKSCATTER_ALBEDO * sigma * range_response(range_m, spec) * (-2.0 * sigma * range_m).exp();
    (intensity > 0.0).then_some(ReturnCandidate { range_m, intensity })
}

/// Spreads clipped detector energy into neighbouring azimuth columns.
fn apply_bloom(pending: &mut [PendingReturn], spec: &LidarSpec) {
    let gain = spec.bloom_gain.max(0.0);
    let radius = i64::from(spec.bloom_column_radius);
    if gain <= 0.0 || radius <= 0 || pending.is_empty() {
        return;
    }
    let saturation = saturation_intensity(spec);

    let mut index_by_cell: HashMap<(u32, u16), Vec<usize>> = HashMap::new();
    for (index, entry) in pending.iter().enumerate() {
        index_by_cell
            .entry((entry.column, entry.channel))
            .or_default()
            .push(index);
    }

    // Collect first so the bloom of one return never feeds the bloom of another.
    let sources = pending
        .iter()
        .filter(|entry| entry.raw_intensity > saturation)
        .map(|entry| {
            (
                entry.column,
                entry.channel,
                entry.raw_intensity - saturation,
            )
        })
        .collect::<Vec<_>>();

    let mut contributions = vec![0.0_f64; pending.len()];
    for (column, channel, excess) in sources {
        for offset in -radius..=radius {
            if offset == 0 {
                continue;
            }
            let Ok(neighbour) = u32::try_from(i64::from(column) + offset) else {
                continue;
            };
            let Some(indices) = index_by_cell.get(&(neighbour, channel)) else {
                continue;
            };
            let falloff = gain * excess / (1.0 + offset.unsigned_abs() as f64);
            for index in indices {
                contributions[*index] += falloff;
            }
        }
    }

    for (entry, contribution) in pending.iter_mut().zip(contributions) {
        entry.raw_intensity += contribution;
    }
}

fn saturation_intensity(spec: &LidarSpec) -> f64 {
    if spec.saturation_intensity.is_finite() {
        spec.saturation_intensity.clamp(f64::MIN_POSITIVE, 1.0)
    } else {
        1.0
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

fn ray_slot(ordinal: u64, offset: u64) -> u64 {
    ordinal.wrapping_mul(RAY_SLOT_STRIDE).wrapping_add(offset) % SCAN_SLOT_BASE
}

fn return_slot(ordinal: u64, return_index: u8) -> u64 {
    ray_slot(
        ordinal,
        SLOT_RETURN_BASE + u64::from(return_index.saturating_sub(1)) * SLOT_RETURN_STRIDE,
    )
}

/// Draws two independent standard normals from one Box-Muller pair.
fn gaussian_pair(random: &KeyedRandom, key: SensorNoiseKey, slot: u64) -> (f64, f64) {
    let u1 = random
        .sample_unit_f64(key.stable_sensor_id, key.sample_index, slot)
        .max(f64::MIN_POSITIVE);
    let u2 = random.sample_unit_f64(key.stable_sensor_id, key.sample_index, slot + 1);
    let magnitude = (-2.0 * u1.ln()).sqrt();
    (magnitude * (TAU * u2).cos(), magnitude * (TAU * u2).sin())
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
        SCAN_SLOT_BASE + channel,
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

    /// Returns a plane at `plane_x` that every ray hits perpendicularly.
    struct WallPhysics {
        entity: Entity,
        plane_x: f64,
    }

    impl PhysicsBackend for WallPhysics {
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
            query: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            if query.direction.x <= 1e-9 {
                return Ok(Vec::new());
            }
            let distance = (self.plane_x - query.origin_m.x) / query.direction.x;
            if distance <= 0.0 || distance > query.max_distance_m {
                return Ok(Vec::new());
            }
            Ok(vec![RaycastHit {
                entity: self.entity,
                point_m: query.origin_m + query.direction * distance,
                normal: Vec3::NEG_X,
                distance_m: distance,
            }])
        }
        fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&[])
        }
        fn capabilities(&self) -> &[PhysicsCapability] {
            &[]
        }
    }

    /// Returns a near surface only for sub-rays offset above the beam axis.
    struct EdgePhysics {
        entity: Entity,
        near_m: f64,
        far_m: f64,
    }

    impl PhysicsBackend for EdgePhysics {
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
            query: RaycastQuery,
        ) -> Result<Vec<RaycastHit>, PhysicsError> {
            let distance = if query.direction.y >= 0.0 {
                self.near_m
            } else {
                self.far_m
            };
            Ok(vec![RaycastHit {
                entity: self.entity,
                point_m: query.origin_m + query.direction * distance,
                normal: -query.direction,
                distance_m: distance,
            }])
        }
        fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
            Ok(&[])
        }
        fn capabilities(&self) -> &[PhysicsCapability] {
            &[]
        }
    }

    fn diffuse_world(material: LidarMaterial) -> (World, Entity) {
        let mut world = World::new();
        let entity = spawn_named(&mut world, "surface");
        world.entity_mut(entity).insert(material);
        (world, entity)
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

        let direction = ray_direction(
            &spec,
            &transform,
            column_azimuth_rad(&spec, 1),
            0,
            &lidar_random(SensorNoiseKey::new(0, 0, 0, 0)),
            SensorNoiseKey::new(0, 0, 0, 0),
            1,
        )
        .expect("direction");
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
        assert_eq!(cloud.channel_indices, vec![0, 0]);
        assert!(cloud.attributes_are_aligned());
    }

    #[test]
    fn keyed_noise_and_weather_are_repeatable() {
        let (world, target) = diffuse_world(LidarMaterial::painted_metal());
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
            solar_noise_floor: 0.004,
            backscatter_probability_scale: 0.4,
            domain_randomization: LidarDomainRandomization {
                fog_extinction_range_per_m: [0.001, 0.02],
                rain_rate_range_mm_h: [0.0, 20.0],
                dust_density_range_mg_m3: [0.0, 10.0],
                snow_rate_range_mm_h: [0.0, 5.0],
            },
            max_returns: 2,
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

    #[test]
    fn multi_channel_scan_covers_every_ring() {
        let (world, wall) = diffuse_world(LidarMaterial::concrete());
        let physics = WallPhysics {
            entity: wall,
            plane_x: 12.0,
        };
        let spec = LidarSpec {
            ray_count: 4,
            min_angle_rad: -0.2,
            max_angle_rad: 0.2,
            channel_count: 8,
            min_elevation_rad: -0.2,
            max_elevation_rad: 0.2,
            min_range_m: 0.5,
            max_range_m: 40.0,
            height_offset_m: 0.0,
            ..LidarSpec::default()
        };

        let cloud = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            SensorNoiseKey::new(5, 5, 5, 5),
        );

        assert_eq!(cloud.points_m.len(), 32);
        assert_eq!(spec.rays_per_scan(), 32);
        let rings = cloud
            .channel_indices
            .iter()
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(rings.len(), 8);
        // Elevation channels must reach above and below the scan axis.
        let heights = cloud.points_m.iter().map(|point| point.y);
        assert!(heights.clone().fold(f64::INFINITY, f64::min) < -1.0);
        assert!(heights.fold(f64::NEG_INFINITY, f64::max) > 1.0);
    }

    #[test]
    fn elevation_table_spans_the_configured_vertical_field_of_view() {
        let spec = LidarSpec {
            channel_count: 3,
            min_elevation_rad: -0.5,
            max_elevation_rad: 0.5,
            ..LidarSpec::default()
        };

        assert_relative_eq!(spec.channel_elevation_rad(0), -0.5, epsilon = 1e-12);
        assert_relative_eq!(spec.channel_elevation_rad(1), 0.0, epsilon = 1e-12);
        assert_relative_eq!(spec.channel_elevation_rad(2), 0.5, epsilon = 1e-12);
        // Out-of-range channels clamp instead of extrapolating.
        assert_relative_eq!(spec.channel_elevation_rad(9), 0.5, epsilon = 1e-12);
    }

    #[test]
    fn sweeping_the_sensor_distorts_the_cloud_and_stamps_emission_times() {
        let (world, wall) = diffuse_world(LidarMaterial::concrete());
        let physics = WallPhysics {
            entity: wall,
            plane_x: 15.0,
        };
        let spec = LidarSpec {
            ray_count: 8,
            min_angle_rad: -0.3,
            max_angle_rad: 0.3,
            min_range_m: 0.5,
            max_range_m: 60.0,
            height_offset_m: 0.0,
            rotation_period_s: 0.1,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(11, 2, 3, 4);
        let start = Transform3::IDENTITY;
        let end = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 5.0), Quat::IDENTITY);

        let stationary = sample_lidar_swept(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &LidarSweep::stationary(start),
            &spec,
            key,
        );
        let swept = sample_lidar_swept(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &LidarSweep::new(start, end),
            &spec,
            key,
        );

        assert_eq!(stationary.points_m.len(), swept.points_m.len());
        assert_ne!(stationary.points_m, swept.points_m);

        // Emission times advance monotonically across the sweep and span one period.
        assert_relative_eq!(swept.timestamps_s[0], 0.0, epsilon = 1e-12);
        assert!(swept.timestamps_s.windows(2).all(|pair| pair[1] >= pair[0]));
        assert_relative_eq!(swept.scan_duration_s(), 0.0875, epsilon = 1e-12);

        // A stationary sweep must reproduce the plain keyed sample exactly.
        let plain = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &start,
            &spec,
            key,
        );
        assert_eq!(plain, stationary);
    }

    #[test]
    fn intensity_follows_inverse_square_range_falloff() {
        let (world, wall) = diffuse_world(LidarMaterial::new(1.0, 0.0, 1.0));
        let spec = LidarSpec {
            ray_count: 1,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            min_range_m: 0.1,
            max_range_m: 100.0,
            height_offset_m: 0.0,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(1, 1, 1, 1);

        let reference = sample_lidar_keyed(
            &physics_wall(wall, RANGE_REFERENCE_M),
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );
        let doubled = sample_lidar_keyed(
            &physics_wall(wall, RANGE_REFERENCE_M * 2.0),
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );

        // A unit Lambertian reflector at the reference range returns full scale.
        assert_relative_eq!(f64::from(reference.intensities[0]), 1.0, epsilon = 1e-6);
        // Doubling the range quarters the returned energy.
        assert_relative_eq!(f64::from(doubled.intensities[0]), 0.25, epsilon = 1e-6);
    }

    fn physics_wall(entity: Entity, plane_x: f64) -> WallPhysics {
        WallPhysics { entity, plane_x }
    }

    #[test]
    fn near_field_overlap_suppresses_returns_inside_the_crossover_range() {
        let (world, wall) = diffuse_world(LidarMaterial::new(1.0, 0.0, 1.0));
        let spec = LidarSpec {
            ray_count: 1,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            min_range_m: 2.0,
            max_range_m: 100.0,
            height_offset_m: 0.0,
            ..LidarSpec::default()
        };

        let close = sample_lidar_keyed(
            &physics_wall(wall, 2.0),
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            SensorNoiseKey::new(2, 2, 2, 2),
        );

        // Without the overlap term the inverse-square law alone would return 25.0.
        let expected = 25.0 * (1.0 - (-1.0_f64).exp());
        assert_relative_eq!(
            f64::from(close.intensities[0]),
            expected.min(1.0),
            epsilon = 1e-6
        );
    }

    #[test]
    fn retroreflective_sheeting_saturates_and_blooms_into_neighbours() {
        let mut world = World::new();
        let sign = spawn_named(&mut world, "sign");
        world
            .entity_mut(sign)
            .insert(LidarMaterial::retroreflective_sign());
        let physics = WallPhysics {
            entity: sign,
            plane_x: 30.0,
        };
        let base = LidarSpec {
            ray_count: 5,
            min_angle_rad: -0.05,
            max_angle_rad: 0.05,
            min_range_m: 0.5,
            max_range_m: 80.0,
            height_offset_m: 0.0,
            saturation_intensity: 0.9,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(7, 7, 7, 7);

        let without_bloom = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &base,
            key,
        );
        // The retroreflective lobe drives the detector into saturation.
        assert!(without_bloom
            .intensities
            .iter()
            .all(|intensity| (*intensity - 0.9).abs() < 1e-6));

        // A diffuse surface of the same reflectivity stays well below saturation.
        let mut diffuse_world = World::new();
        let plate = spawn_named(&mut diffuse_world, "plate");
        diffuse_world
            .entity_mut(plate)
            .insert(LidarMaterial::new(0.85, 0.0, 0.1));
        let diffuse = sample_lidar_keyed(
            &WallPhysics {
                entity: plate,
                plane_x: 30.0,
            },
            PhysicsWorldId::DEFAULT,
            &diffuse_world,
            &Transform3::IDENTITY,
            &base,
            key,
        );
        assert!(diffuse.intensities.iter().all(|intensity| *intensity < 0.2));
    }

    #[test]
    fn bloom_lifts_dim_neighbours_of_a_saturated_column() {
        let mut world = World::new();
        let sign = spawn_named(&mut world, "sign");
        // Only the middle column sees the retroreflector; neighbours see concrete.
        world
            .entity_mut(sign)
            .insert(LidarMaterial::retroreflective_sign());

        struct SplitPhysics {
            sign: Entity,
            plain: Entity,
        }
        impl PhysicsBackend for SplitPhysics {
            type BodyHandle = ();
            type ColliderHandle = ();
            fn create_world(
                &mut self,
                _: PhysicsWorldDesc,
            ) -> Result<PhysicsWorldId, PhysicsError> {
                Ok(PhysicsWorldId::DEFAULT)
            }
            fn sync_from_ecs(
                &mut self,
                _: &mut World,
                _: PhysicsWorldId,
            ) -> Result<(), PhysicsError> {
                Ok(())
            }
            fn step(
                &mut self,
                _: PhysicsWorldId,
                _: rne_core::SimDuration,
            ) -> Result<(), PhysicsError> {
                Ok(())
            }
            fn sync_to_ecs(
                &mut self,
                _: &mut World,
                _: PhysicsWorldId,
            ) -> Result<(), PhysicsError> {
                Ok(())
            }
            fn raycast(
                &self,
                _: PhysicsWorldId,
                query: RaycastQuery,
            ) -> Result<Vec<RaycastHit>, PhysicsError> {
                let entity = if query.direction.z.abs() < 1e-6 {
                    self.sign
                } else {
                    self.plain
                };
                Ok(vec![RaycastHit {
                    entity,
                    point_m: query.origin_m + query.direction * 30.0,
                    normal: -query.direction,
                    distance_m: 30.0,
                }])
            }
            fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
                Ok(&[])
            }
            fn capabilities(&self) -> &[PhysicsCapability] {
                &[]
            }
        }

        let plain = spawn_named(&mut world, "concrete");
        world.entity_mut(plain).insert(LidarMaterial::concrete());
        let physics = SplitPhysics { sign, plain };
        let spec = LidarSpec {
            ray_count: 3,
            min_angle_rad: -0.02,
            max_angle_rad: 0.02,
            min_range_m: 0.5,
            max_range_m: 80.0,
            height_offset_m: 0.0,
            saturation_intensity: 0.9,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(8, 8, 8, 8);

        let plain_scan = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );
        let bloomed = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &LidarSpec {
                bloom_gain: 0.05,
                bloom_column_radius: 1,
                ..spec
            },
            key,
        );

        assert_eq!(plain_scan.points_m.len(), 3);
        assert_eq!(bloomed.points_m.len(), 3);
        // The saturated middle column is unchanged; its dim neighbours brighten.
        assert!(bloomed.intensities[0] > plain_scan.intensities[0]);
        assert!(bloomed.intensities[2] > plain_scan.intensities[2]);
        assert_relative_eq!(
            f64::from(bloomed.intensities[1]),
            f64::from(plain_scan.intensities[1]),
            epsilon = 1e-9
        );
    }

    #[test]
    fn beam_footprint_straddling_an_edge_reports_a_mixed_pixel() {
        let (world, edge) = diffuse_world(LidarMaterial::concrete());
        let physics = EdgePhysics {
            entity: edge,
            near_m: 10.0,
            far_m: 30.0,
        };
        let axial = LidarSpec {
            ray_count: 1,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            min_range_m: 0.5,
            max_range_m: 80.0,
            height_offset_m: 0.0,
            beam_divergence_rad: 0.02,
            mixed_pixel_threshold_m: 0.25,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(3, 3, 3, 3);

        let single_sample = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &axial,
            key,
        );
        let integrated = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &LidarSpec {
                beam_sample_count: 16,
                ..axial
            },
            key,
        );

        let axial_range = single_sample.points_m[0].length();
        let mixed_range = integrated.points_m[0].length();
        // The blended range sits between the two surfaces the footprint covers.
        assert!(mixed_range > 10.0 + 1e-6);
        assert!(mixed_range < 30.0 - 1e-6);
        assert!((mixed_range - axial_range).abs() > 0.25);
        // Splitting the footprint across an edge costs the return energy.
        assert!(integrated.intensities[0] < single_sample.intensities[0]);
    }

    #[test]
    fn fog_backscatter_adds_returns_in_front_of_the_surface() {
        let (world, wall) = diffuse_world(LidarMaterial::concrete());
        let physics = WallPhysics {
            entity: wall,
            plane_x: 60.0,
        };
        let clear = LidarSpec {
            ray_count: 64,
            min_angle_rad: -0.4,
            max_angle_rad: 0.4,
            min_range_m: 0.5,
            max_range_m: 80.0,
            height_offset_m: 0.0,
            max_returns: 2,
            ..LidarSpec::default()
        };
        let foggy = LidarSpec {
            atmosphere: LidarAtmosphere {
                fog_extinction_per_m: 0.05,
                ..LidarAtmosphere::default()
            },
            backscatter_probability_scale: 1.0,
            ..clear
        };
        let key = SensorNoiseKey::new(4, 4, 4, 4);

        let clear_scan = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &clear,
            key,
        );
        let foggy_scan = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &foggy,
            key,
        );

        // Aerosol returns land in front of the wall, so the scan gains near points.
        let clear_near = clear_scan
            .points_m
            .iter()
            .filter(|point| point.length() < 55.0)
            .count();
        let foggy_near = foggy_scan
            .points_m
            .iter()
            .filter(|point| point.length() < 55.0)
            .count();
        assert_eq!(clear_near, 0);
        assert!(foggy_near > 0);
        assert!(foggy_scan.return_indices.contains(&2));
    }

    #[test]
    fn rain_occlusion_drops_distant_returns() {
        let (world, wall) = diffuse_world(LidarMaterial::concrete());
        let physics = WallPhysics {
            entity: wall,
            plane_x: 70.0,
        };
        let spec = LidarSpec {
            ray_count: 128,
            min_angle_rad: -0.4,
            max_angle_rad: 0.4,
            min_range_m: 0.5,
            max_range_m: 90.0,
            height_offset_m: 0.0,
            minimum_intensity: 0.0,
            ..LidarSpec::default()
        };
        let key = SensorNoiseKey::new(6, 6, 6, 6);

        let dry = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &spec,
            key,
        );
        let downpour = sample_lidar_keyed(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &Transform3::IDENTITY,
            &LidarSpec {
                atmosphere: LidarAtmosphere {
                    rain_rate_mm_h: 40.0,
                    ..LidarAtmosphere::default()
                },
                ..spec
            },
            key,
        );

        assert_eq!(dry.points_m.len(), 128);
        assert!(downpour.points_m.len() < dry.points_m.len());
    }

    #[test]
    fn occlusion_ignores_fog_and_dust_but_not_precipitation() {
        let fog = LidarAtmosphere {
            fog_extinction_per_m: 0.2,
            dust_density_mg_m3: 500.0,
            ..LidarAtmosphere::default()
        };
        let rain = LidarAtmosphere {
            rain_rate_mm_h: 20.0,
            ..LidarAtmosphere::default()
        };

        assert!(fog.extinction_per_m() > 0.0);
        assert_relative_eq!(fog.occlusion_per_m(), 0.0, epsilon = 1e-15);
        assert!(rain.occlusion_per_m() > 0.0);
    }
}
