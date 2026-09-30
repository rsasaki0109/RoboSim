//! Livox Mid-360 scan pattern, sensor preset, and rig occlusion, fitted to real scans.
//!
//! The Mid-360 is a non-repetitive scanner: it does not fire on an azimuth/channel
//! grid, so [`LidarSpec`]'s grid cannot describe it. Livox does not publish the scan
//! pattern. This module reproduces it from real `livox_ros_driver2` recordings of a
//! Go2-mounted unit (see `docs/LIVOX_MID360.md` and
//! `tools/prepare_livox_mid360_model.py`):
//!
//! * four emitter lines fire in turn every [`LIVOX_MID360_POINT_PERIOD_S`], so one
//!   firing of all four lines takes [`LIVOX_MID360_FIRING_PERIOD_S`];
//! * a fast rotor turns about 181 times per second, and a second element nods the
//!   beams through the −7°..52° elevation band with a period slightly longer than a
//!   frame, which is what makes successive frames land on new directions;
//! * each line's direction is a smooth function of those two phases, stored as a
//!   91-term Fourier series per line and component.
//!
//! On recordings the fit never saw, the generated directions match the measured
//! returns with a median error of about 0.1°, below the 0.15° angular precision on
//! the Livox datasheet.
//!
//! # Coordinate frames
//!
//! [`LivoxMid360Pattern::direction`] returns directions in the Livox sensor frame
//! (`x` forward, `y` left, `z` up, as published by the driver). [`LidarRay`] uses the
//! engine's sensor convention (`+X` forward, `+Y` up, `+Z` right), so a Livox
//! direction `(x, y, z)` becomes the local direction `(x, z, -y)`.
//!
//! # Timestamp, latency, and noise behavior
//!
//! * **Timestamps.** Point `n` of a frame is emitted `n * 5 µs` after the frame's
//!   first firing, and every returned point carries that offset in
//!   `PointCloud.timestamps_s`. Frames are whole 96-point driver packets, so they
//!   hold 208 or 209 packets and span 99.8–100.3 ms at the 10 Hz frame rate.
//! * **Latency.** The pattern adds none; frame latency is the owning `Sensor`'s
//!   `latency_ticks`, as for every other `LiDAR`.
//! * **Noise.** Range noise, pointing jitter, detection, and dropout are the
//!   physics-aware model in [`crate::lidar`] configured by [`livox_mid360_spec`].
//!   Rig occlusion draws come from a disjoint keyed stream, so a scan replays
//!   exactly for a given [`SensorNoiseKey`].

use crate::lidar::{sample_lidar_pattern_swept, LidarRay, LidarSpec, LidarSweep};
use crate::livox_mid360_coefficients::{
    COEFFICIENTS, HARMONICS, NOD_RAD_PER_FIRING, ROTOR_RAD_PER_FIRING,
};
use crate::SensorNoiseKey;
use rne_core::{mix64, KeyedRandom};
use rne_data::PointCloud;
use rne_ecs::World;
use rne_math::Vec3;
use rne_physics::{PhysicsBackend, PhysicsWorldId};
use serde::{Deserialize, Serialize};
use std::ops::Range;

/// Interval between consecutive points (one emitter line firing), in seconds.
pub const LIVOX_MID360_POINT_PERIOD_S: f64 = 5.0e-6;
/// Number of emitter lines fired in turn.
pub const LIVOX_MID360_LINE_COUNT: u16 = 4;
/// Interval between consecutive firings of all lines, in seconds (200,000 points/s).
pub const LIVOX_MID360_FIRING_PERIOD_S: f64 =
    LIVOX_MID360_POINT_PERIOD_S * LIVOX_MID360_LINE_COUNT as f64;
/// Points per driver packet; frames always hold whole packets.
pub const LIVOX_MID360_POINTS_PER_PACKET: u64 = 96;
/// Nominal frame period at the typical 10 Hz frame rate, in seconds.
pub const LIVOX_MID360_FRAME_PERIOD_S: f64 = 0.1;
/// Lowest elevation of the vertical field of view, in radians (−7°).
pub const LIVOX_MID360_MIN_ELEVATION_RAD: f64 = -7.0 * std::f64::consts::PI / 180.0;
/// Highest elevation of the vertical field of view, in radians (52°).
pub const LIVOX_MID360_MAX_ELEVATION_RAD: f64 = 52.0 * std::f64::consts::PI / 180.0;

const FIRINGS_PER_PACKET: u64 = LIVOX_MID360_POINTS_PER_PACKET / LIVOX_MID360_LINE_COUNT as u64;
/// Packets per frame as a ratio: 0.1 s / (24 firings * 20 µs) = 625 / 3.
const PACKETS_PER_FRAME_NUM: u64 = 625;
const PACKETS_PER_FRAME_DEN: u64 = 3;
const RIG_RANDOM_DOMAIN_V1: u64 = 0x4D49_4433_3630_5247;

/// Physics-aware settings of a Livox Mid-360.
///
/// Values come from the Livox datasheet unless noted:
///
/// * 905 nm, 0.1 m close-proximity blind zone, 70 m range at 80 % reflectivity;
/// * first return only, which is the configuration behind the 200,000 points/s rate;
/// * the detection threshold puts a 10 % Lambertian target at 40 m — the datasheet's
///   other range figure — at the 50 % detection point;
/// * range noise of 1 cm (1σ), the robust spread measured on a real floor within 2 m,
///   which also contains floor roughness, so it is an upper bound; the datasheet
///   quotes ≤ 2 cm at 10 m;
/// * pointing jitter of 0.09° (1σ), the residual of the fitted pattern against real
///   returns, again an upper bound on the true jitter.
///
/// The grid fields describe a 4-ring, 5,000-column approximation used only when the
/// spec is sampled on the grid; [`sample_livox_mid360`] ignores them.
pub fn livox_mid360_spec() -> LidarSpec {
    LidarSpec {
        ray_count: 5_000,
        min_angle_rad: -std::f64::consts::PI,
        max_angle_rad: std::f64::consts::PI,
        channel_count: LIVOX_MID360_LINE_COUNT,
        min_elevation_rad: LIVOX_MID360_MIN_ELEVATION_RAD,
        max_elevation_rad: LIVOX_MID360_MAX_ELEVATION_RAD,
        rotation_period_s: LIVOX_MID360_FRAME_PERIOD_S,
        min_range_m: 0.1,
        max_range_m: 70.0,
        height_offset_m: 0.0,
        max_returns: 1,
        wavelength_nm: 905.0,
        beam_divergence_rad: 2.0 * 0.09_f64.to_radians(),
        beam_sample_count: 1,
        range_noise_stddev_m: 0.01,
        detection_threshold_intensity: 0.1 * (crate::RANGE_REFERENCE_M / 40.0).powi(2),
        detection_sharpness: 3.0,
        ..LidarSpec::default()
    }
}

/// Deterministic Mid-360 firing pattern.
///
/// The pattern is indexed by a global firing counter that runs continuously across
/// frames, exactly as the real scanner never stops between frames. The phase offsets
/// shift where the counter starts; real units wander by about 1° within ±2° over
/// tens of seconds, which moves points along the scan path without changing
/// coverage, so the default leaves them at zero.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LivoxMid360Pattern {
    rotor_phase_offset_rad: f64,
    nod_phase_offset_rad: f64,
}

impl LivoxMid360Pattern {
    /// Creates the pattern with zero phase offsets.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the pattern with its rotor and nod phases shifted, in radians.
    pub fn with_phase_offsets_rad(self, rotor_rad: f64, nod_rad: f64) -> Self {
        Self {
            rotor_phase_offset_rad: rotor_rad,
            nod_phase_offset_rad: nod_rad,
        }
    }

    /// Returns the unit direction of `line` at global firing `firing`, in the Livox
    /// sensor frame (`x` forward, `y` left, `z` up).
    ///
    /// Lines beyond [`LIVOX_MID360_LINE_COUNT`] wrap.
    pub fn direction(&self, firing: u64, line: u16) -> Vec3 {
        let line = usize::from(line % LIVOX_MID360_LINE_COUNT);
        let firing = firing as f64;
        let rotor = ROTOR_RAD_PER_FIRING * firing + self.rotor_phase_offset_rad;
        let nod = NOD_RAD_PER_FIRING * firing + self.nod_phase_offset_rad;
        let coefficients = &COEFFICIENTS[line];
        let mut rotor_frame = [0.0_f64; 3];
        let mut column = 0;
        for &(rotor_harmonic, nod_harmonic) in &HARMONICS {
            if (rotor_harmonic, nod_harmonic) == (0, 0) {
                for (component, value) in rotor_frame.iter_mut().enumerate() {
                    *value += coefficients[component][column];
                }
                column += 1;
                continue;
            }
            let phase = f64::from(rotor_harmonic) * rotor + f64::from(nod_harmonic) * nod;
            let (sin, cos) = phase.sin_cos();
            for (component, value) in rotor_frame.iter_mut().enumerate() {
                *value += coefficients[component][column] * cos
                    + coefficients[component][column + 1] * sin;
            }
            column += 2;
        }
        let (sin, cos) = rotor.sin_cos();
        Vec3::new(
            cos * rotor_frame[0] - sin * rotor_frame[1],
            sin * rotor_frame[0] + cos * rotor_frame[1],
            rotor_frame[2],
        )
        .normalize_or_zero()
    }

    /// Returns the global firings that make up frame `frame_index`.
    ///
    /// A frame is the whole packets emitted within its 100 ms window, so frames
    /// alternate between 208 and 209 packets (4,992 or 5,016 firings), as recorded.
    pub fn frame_firings(frame_index: u64) -> Range<u64> {
        let start = frame_start_packet(frame_index) * FIRINGS_PER_PACKET;
        let end = frame_start_packet(frame_index.saturating_add(1)) * FIRINGS_PER_PACKET;
        start..end
    }

    /// Returns every ray of frame `frame_index` in emission order, in the engine's
    /// sensor convention.
    ///
    /// `LidarRay::column` is the firing index within the frame, `LidarRay::channel`
    /// the emitter line, and `LidarRay::time_s` the emission offset from the frame's
    /// first firing.
    pub fn frame_rays(&self, frame_index: u64) -> Vec<LidarRay> {
        let firings = Self::frame_firings(frame_index);
        let start = firings.start;
        let mut rays = Vec::with_capacity(
            ((firings.end - firings.start) * u64::from(LIVOX_MID360_LINE_COUNT)) as usize,
        );
        for firing in firings {
            let column = (firing - start) as u32;
            for line in 0..LIVOX_MID360_LINE_COUNT {
                let direction = self.direction(firing, line);
                let (azimuth_rad, elevation_rad) = livox_to_local_angles(direction);
                rays.push(LidarRay {
                    azimuth_rad,
                    elevation_rad,
                    time_s: f64::from(column) * LIVOX_MID360_FIRING_PERIOD_S
                        + f64::from(line) * LIVOX_MID360_POINT_PERIOD_S,
                    channel: line,
                    column,
                });
            }
        }
        rays
    }
}

fn frame_start_packet(frame_index: u64) -> u64 {
    frame_index
        .saturating_mul(PACKETS_PER_FRAME_NUM)
        .div_ceil(PACKETS_PER_FRAME_DEN)
}

/// Converts a Livox-frame unit direction to the engine's ray azimuth and elevation.
fn livox_to_local_angles(direction: Vec3) -> (f64, f64) {
    // Livox (x, y, z) is local (x, z, -y): local azimuth runs from +X toward +Z (right).
    let elevation_rad = direction.z.clamp(-1.0, 1.0).asin();
    let azimuth_rad = (-direction.y).atan2(direction.x);
    (azimuth_rad, elevation_rad)
}

/// Converts an engine ray back to a Livox-frame unit direction.
fn local_angles_to_livox(azimuth_rad: f64, elevation_rad: f64) -> Vec3 {
    let (sin_el, cos_el) = elevation_rad.sin_cos();
    let (sin_az, cos_az) = azimuth_rad.sin_cos();
    Vec3::new(cos_el * cos_az, -cos_el * sin_az, sin_el)
}

/// Measured occlusion of a `LiDAR` by the robot and mount it is attached to.
///
/// Mounting hardware close to the window sits inside the sensor's blind zone, so it
/// removes returns without producing points; body panels further away return points
/// at a fixed range. Both are fixed in the sensor frame. The table is binned in
/// Livox-frame azimuth `atan2(y, x)` and elevation `asin(z)`; bins without a cell
/// pass every ray.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LidarRigOcclusion {
    /// Azimuth of the first bin's lower edge, in degrees.
    pub min_azimuth_deg: f64,
    /// Azimuth bin width, in degrees.
    pub azimuth_bin_deg: f64,
    /// Number of azimuth bins.
    pub azimuth_bins: u32,
    /// Elevation of the first bin's lower edge, in degrees.
    pub min_elevation_deg: f64,
    /// Elevation bin width, in degrees.
    pub elevation_bin_deg: f64,
    /// Number of elevation bins.
    pub elevation_bins: u32,
    /// Normalized intensity reported for self returns.
    #[serde(default = "default_self_return_intensity")]
    pub self_return_intensity: f64,
    /// Bins that block or reflect some of their rays.
    pub cells: Vec<LidarRigOcclusionCell>,
}

/// One bin of a [`LidarRigOcclusion`] table.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LidarRigOcclusionCell {
    /// Azimuth bin index.
    pub azimuth_bin: u32,
    /// Elevation bin index.
    pub elevation_bin: u32,
    /// Probability that a ray in this bin produces no return.
    pub block_probability: f64,
    /// Probability that a ray in this bin returns from the robot itself.
    pub self_return_probability: f64,
    /// Range of self returns in this bin, in meters.
    pub self_return_range_m: f64,
}

/// Normalized self-return intensity: the recorded median of 1 on the driver's
/// 0–255 reflectivity scale.
fn default_self_return_intensity() -> f64 {
    1.0 / 255.0
}

/// What the rig does to one ray.
#[derive(Clone, Copy, Debug, PartialEq)]
enum RigOutcome {
    Pass,
    Blocked,
    SelfReturn(f64),
}

impl LidarRigOcclusion {
    fn dense_index(&self) -> Vec<Option<LidarRigOcclusionCell>> {
        let mut dense = vec![None; (self.azimuth_bins as usize) * (self.elevation_bins as usize)];
        for cell in &self.cells {
            if cell.azimuth_bin < self.azimuth_bins && cell.elevation_bin < self.elevation_bins {
                dense[(cell.azimuth_bin * self.elevation_bins + cell.elevation_bin) as usize] =
                    Some(*cell);
            }
        }
        dense
    }

    fn bin(&self, direction: Vec3) -> Option<usize> {
        if self.azimuth_bin_deg <= 0.0 || self.elevation_bin_deg <= 0.0 {
            return None;
        }
        let azimuth_deg = direction.y.atan2(direction.x).to_degrees();
        let elevation_deg = direction.z.clamp(-1.0, 1.0).asin().to_degrees();
        let azimuth = ((azimuth_deg - self.min_azimuth_deg) / self.azimuth_bin_deg).floor();
        let elevation = ((elevation_deg - self.min_elevation_deg) / self.elevation_bin_deg).floor();
        if azimuth < 0.0 || elevation < 0.0 {
            return None;
        }
        let (azimuth, elevation) = (azimuth as u32, elevation as u32);
        (azimuth < self.azimuth_bins && elevation < self.elevation_bins)
            .then(|| (azimuth * self.elevation_bins + elevation) as usize)
    }
}

/// Samples one Mid-360 frame: pattern, rig occlusion, and physics-aware returns.
///
/// Rays the rig blocks produce nothing; rays it reflects produce a self return at the
/// measured range; every other ray is cast through [`sample_lidar_pattern_swept`]. The
/// sweep spans the frame, so a moving platform distorts the cloud as the real sensor
/// does. `frame_index` selects the frame of the continuous firing stream and should
/// advance by one per call.
// Each argument is an independent input the caller already owns; bundling them would
// only relocate the arity.
#[allow(clippy::too_many_arguments)]
pub fn sample_livox_mid360<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &World,
    sweep: &LidarSweep,
    spec: &LidarSpec,
    pattern: &LivoxMid360Pattern,
    frame_index: u64,
    rig: Option<&LidarRigOcclusion>,
    noise_key: SensorNoiseKey,
) -> PointCloud {
    let rays = pattern.frame_rays(frame_index);
    let Some(rig) = rig else {
        return sample_lidar_pattern_swept(
            backend,
            physics_world,
            world,
            sweep,
            spec,
            &rays,
            noise_key,
        );
    };

    let dense = rig.dense_index();
    let random = KeyedRandom::new(
        noise_key.root_seed,
        RIG_RANDOM_DOMAIN_V1 ^ mix64(noise_key.sensor_seed),
    );
    let mut cast = Vec::with_capacity(rays.len());
    let mut self_returns = Vec::new();
    for (index, ray) in rays.iter().enumerate() {
        let direction = local_angles_to_livox(ray.azimuth_rad, ray.elevation_rad);
        let outcome = match rig.bin(direction).and_then(|bin| dense[bin]) {
            None => RigOutcome::Pass,
            Some(cell) => {
                let draw = random.sample_unit_f64(
                    noise_key.stable_sensor_id,
                    noise_key.sample_index,
                    index as u64,
                );
                if draw < cell.block_probability {
                    RigOutcome::Blocked
                } else if draw < cell.block_probability + cell.self_return_probability {
                    RigOutcome::SelfReturn(cell.self_return_range_m)
                } else {
                    RigOutcome::Pass
                }
            }
        };
        match outcome {
            RigOutcome::Pass => cast.push(*ray),
            RigOutcome::Blocked => {}
            RigOutcome::SelfReturn(range_m) => self_returns.push((*ray, range_m)),
        }
    }

    let mut cloud =
        sample_lidar_pattern_swept(backend, physics_world, world, sweep, spec, &cast, noise_key);
    let period_s = spec.rotation_period_s;
    for (ray, range_m) in self_returns {
        let fraction = if period_s > 0.0 {
            (ray.time_s / period_s).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let pose = sweep.pose_at(fraction);
        let (sin_el, cos_el) = ray.elevation_rad.sin_cos();
        let (sin_az, cos_az) = ray.azimuth_rad.sin_cos();
        let local = Vec3::new(cos_el * cos_az, sin_el, cos_el * sin_az);
        let origin_m = pose.translation + Vec3::new(0.0, spec.height_offset_m, 0.0);
        cloud.push_return(
            origin_m + (pose.rotation * local) * range_m,
            rig.self_return_intensity as f32,
            ray.column,
            1,
            ray.channel,
            if period_s > 0.0 { ray.time_s } else { 0.0 },
        );
    }
    cloud
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_hold_whole_packets_at_ten_hertz() {
        let mut firings = 0;
        for frame in 0..30 {
            let range = LivoxMid360Pattern::frame_firings(frame);
            let len = range.end - range.start;
            assert_eq!(len % FIRINGS_PER_PACKET, 0);
            assert!(len == 208 * FIRINGS_PER_PACKET || len == 209 * FIRINGS_PER_PACKET);
            if frame > 0 {
                assert_eq!(
                    range.start,
                    LivoxMid360Pattern::frame_firings(frame - 1).end
                );
            }
            firings += len;
        }
        // 30 frames = 3 s at 20 µs per firing.
        assert_eq!(firings, 150_000);
    }

    #[test]
    fn rays_stay_inside_the_field_of_view_and_emission_window() {
        let rays = LivoxMid360Pattern::new().frame_rays(7);
        let margin_rad = 1.5_f64.to_radians();
        for ray in &rays {
            assert!(ray.elevation_rad >= LIVOX_MID360_MIN_ELEVATION_RAD - margin_rad);
            assert!(ray.elevation_rad <= LIVOX_MID360_MAX_ELEVATION_RAD + margin_rad);
            assert!(ray.time_s >= 0.0 && ray.time_s < 0.101);
            assert!(ray.channel < LIVOX_MID360_LINE_COUNT);
        }
        assert_eq!(rays.len() % LIVOX_MID360_POINTS_PER_PACKET as usize, 0);
    }

    #[test]
    fn local_angle_conversion_round_trips() {
        let pattern = LivoxMid360Pattern::new();
        for firing in [0_u64, 17, 5_000, 123_456] {
            for line in 0..LIVOX_MID360_LINE_COUNT {
                let direction = pattern.direction(firing, line);
                let (azimuth, elevation) = livox_to_local_angles(direction);
                let back = local_angles_to_livox(azimuth, elevation);
                assert!((back - direction).length() < 1e-12);
            }
        }
    }

    #[test]
    fn successive_frames_cover_new_directions() {
        // Non-repetitive scanning: coverage of the 360° x 59° field of view in 1° cells
        // keeps growing frame after frame. Measured: 61.9 % after one frame, 97.6 %
        // after ten.
        let pattern = LivoxMid360Pattern::new();
        let mut cells = std::collections::HashSet::new();
        let mut covered = Vec::new();
        for frame in 0..10 {
            for ray in pattern.frame_rays(frame) {
                cells.insert((
                    ray.azimuth_rad.to_degrees().floor() as i32,
                    ray.elevation_rad.to_degrees().floor() as i32,
                ));
            }
            covered.push(cells.len() as f64 / (360.0 * 59.0));
        }
        assert!(
            covered.windows(2).all(|pair| pair[1] > pair[0]),
            "{covered:?}"
        );
        assert!(covered[0] > 0.58 && covered[0] < 0.66, "{covered:?}");
        assert!(covered[9] > 0.95, "{covered:?}");
    }
}
