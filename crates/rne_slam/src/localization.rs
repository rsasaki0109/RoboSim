//! Deterministic Monte Carlo localization (AMCL) against a known map.
//!
//! A particle cloud is predicted from the odometry delta, weighted by a
//! likelihood field built from a prior occupancy map, and resampled when the
//! effective sample size drops. Every random draw comes from a seeded
//! [`DeterministicRng`], so a recorded sequence reproduces the same estimate.

use crate::likelihood::{LikelihoodConfig, LikelihoodField};
use crate::scan_match::scan_points_2d;
use rne_core::DeterministicRng;
use rne_math::Vec3;
use rne_nav::{GridError, LaserScan2d, OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

/// Monte Carlo localization configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AmclConfig {
    /// Number of particles.
    pub particle_count: usize,
    /// Seed for the internal deterministic RNG.
    pub seed: u64,
    /// Likelihood field settings for the prior map.
    pub likelihood: LikelihoodConfig,
    /// Maximum number of beams used per update.
    pub max_beams: usize,
    /// Translation noise scale per meter of motion.
    pub translation_noise_m: f64,
    /// Rotation noise scale per radian of motion.
    pub rotation_noise_rad: f64,
    /// Initial particle translation standard deviation in meters.
    pub initial_translation_std_m: f64,
    /// Initial particle yaw standard deviation in radians.
    pub initial_rotation_std_rad: f64,
    /// Minimum number of in-field beams required to update weights.
    pub min_valid_beams: usize,
}

impl Default for AmclConfig {
    fn default() -> Self {
        Self {
            particle_count: 500,
            seed: 0x5EED_1234,
            likelihood: LikelihoodConfig::default(),
            max_beams: 90,
            translation_noise_m: 0.10,
            rotation_noise_rad: 0.05,
            initial_translation_std_m: 0.25,
            initial_rotation_std_rad: 0.15,
            min_valid_beams: 10,
        }
    }
}

/// A weighted pose sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Particle {
    /// Sampled pose.
    pub pose: Pose2d,
    /// Normalized weight.
    pub weight: f64,
}

/// Result of one localization update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AmclUpdate {
    /// Weighted-mean pose estimate.
    pub estimate: Pose2d,
    /// Effective sample size `1 / Σw²`.
    pub effective_particles: f64,
    /// Mean likelihood per valid beam of the best particle.
    pub mean_score: f64,
}

/// Deterministic particle-filter localization over a known occupancy map.
#[derive(Clone, Debug)]
pub struct Amcl {
    config: AmclConfig,
    rng: DeterministicRng,
    particles: Vec<Particle>,
    field: LikelihoodField,
    previous_odom: Option<Pose2d>,
    estimate: Pose2d,
}

impl Amcl {
    /// Creates a localization filter from a prior map and initial belief.
    pub fn new(
        map: &OccupancyGrid,
        initial_pose: Pose2d,
        config: AmclConfig,
    ) -> Result<Self, GridError> {
        if config.particle_count == 0 {
            return Err(GridError::InvalidSize);
        }
        let field = LikelihoodField::from_occupancy(map, &config.likelihood)?;
        let mut rng = DeterministicRng::new(config.seed);
        let particles = (0..config.particle_count)
            .map(|_| Particle {
                pose: Pose2d::new(
                    initial_pose.x_m + gaussian(&mut rng) * config.initial_translation_std_m,
                    initial_pose.y_m + gaussian(&mut rng) * config.initial_translation_std_m,
                    initial_pose.yaw_rad + gaussian(&mut rng) * config.initial_rotation_std_rad,
                ),
                weight: 1.0 / config.particle_count as f64,
            })
            .collect();
        Ok(Self {
            config,
            rng,
            particles,
            field,
            previous_odom: None,
            estimate: initial_pose,
        })
    }

    /// Current particle cloud.
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    /// Current weighted-mean estimate.
    pub fn estimate(&self) -> Pose2d {
        self.estimate
    }

    /// Predicts from odometry, weights against the scan, and resamples.
    pub fn update(
        &mut self,
        scan: &LaserScan2d,
        odom_pose: Pose2d,
        sensor_from_base: Pose2d,
    ) -> AmclUpdate {
        let delta = match self.previous_odom {
            Some(previous) => previous.inverse().compose(odom_pose),
            None => Pose2d::IDENTITY,
        };
        let translation = delta.x_m.hypot(delta.y_m);
        let translation_std = self.config.translation_noise_m * translation.sqrt() + 1.0e-4;
        let rotation_std = self.config.rotation_noise_rad * delta.yaw_rad.abs().sqrt() + 1.0e-5;

        for particle in &mut self.particles {
            let sampled = Pose2d::new(
                delta.x_m + gaussian(&mut self.rng) * translation_std,
                delta.y_m + gaussian(&mut self.rng) * translation_std,
                delta.yaw_rad + gaussian(&mut self.rng) * rotation_std,
            );
            particle.pose = particle.pose.compose(sampled);
        }

        let points = scan_points_2d(scan, self.config.max_beams);
        let field = &self.field;
        let min_valid_beams = self.config.min_valid_beams;
        let mut total_weight = 0.0;
        let mut best_score = 0.0_f64;
        for particle in &mut self.particles {
            let score = particle_score(
                field,
                particle.pose,
                sensor_from_base,
                &points,
                min_valid_beams,
            );
            particle.weight = score.max(0.0);
            total_weight += particle.weight;
            best_score = best_score.max(score);
        }
        if total_weight > 0.0 {
            for particle in &mut self.particles {
                particle.weight /= total_weight;
            }
        } else {
            let uniform = 1.0 / self.particles.len() as f64;
            for particle in &mut self.particles {
                particle.weight = uniform;
            }
        }

        let sum_squares: f64 = self.particles.iter().map(|p| p.weight * p.weight).sum();
        let effective = if sum_squares > 0.0 {
            1.0 / sum_squares
        } else {
            0.0
        };
        self.estimate = weighted_mean(&self.particles);
        if effective < self.particles.len() as f64 * 0.5 {
            self.resample();
        }
        self.previous_odom = Some(odom_pose);

        AmclUpdate {
            estimate: self.estimate,
            effective_particles: effective,
            mean_score: best_score,
        }
    }

    fn resample(&mut self) {
        let count = self.particles.len();
        let offset = self.rng.uniform_f64(0.0, 1.0 / count as f64);
        let mut cumulative = 0.0;
        let mut index = 0;
        let mut resampled = Vec::with_capacity(count);
        for _ in 0..count {
            let target = offset + cumulative;
            while index + 1 < count {
                let next = cumulative + self.particles[index].weight;
                if target < next {
                    break;
                }
                cumulative = next;
                index += 1;
            }
            resampled.push(self.particles[index]);
            cumulative += 1.0 / count as f64;
        }
        let uniform = 1.0 / count as f64;
        for particle in &mut resampled {
            particle.weight = uniform;
        }
        self.particles = resampled;
    }
}

fn particle_score(
    field: &LikelihoodField,
    pose: Pose2d,
    sensor_from_base: Pose2d,
    points: &[Vec3],
    min_valid_beams: usize,
) -> f64 {
    let sensor_pose = pose.compose(sensor_from_base);
    let mut total = 0.0;
    let mut valid = 0;
    for point in points {
        let world = sensor_pose.transform_point(*point);
        if field.world_to_grid(world).is_some() {
            total += field.score(world);
            valid += 1;
        }
    }
    if valid < min_valid_beams {
        0.0
    } else {
        total / valid as f64
    }
}

fn weighted_mean(particles: &[Particle]) -> Pose2d {
    let mut x = 0.0;
    let mut y = 0.0;
    let mut sin = 0.0;
    let mut cos = 0.0;
    let mut total = 0.0;
    for particle in particles {
        let weight = particle.weight.max(0.0);
        total += weight;
        x += particle.pose.x_m * weight;
        y += particle.pose.y_m * weight;
        sin += particle.pose.yaw_rad.sin() * weight;
        cos += particle.pose.yaw_rad.cos() * weight;
    }
    if total <= 0.0 {
        return particles
            .first()
            .map(|particle| particle.pose)
            .unwrap_or(Pose2d::IDENTITY);
    }
    Pose2d::new(x / total, y / total, sin.atan2(cos))
}

/// Samples a standard normal via the Box-Muller transform.
fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::GridCoord;
    use std::f64::consts::TAU;

    fn room_map() -> OccupancyGrid {
        let resolution = 0.05;
        let mut grid = OccupancyGrid::new(
            (12.0 / resolution) as usize,
            (8.0 / resolution) as usize,
            resolution,
            Pose2d::new(-6.0, -4.0, 0.0),
        )
        .unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        for (min_x, min_y, max_x, max_y) in [
            (-5.0, -3.0, -5.0, 3.0),
            (5.0, -3.0, 5.0, 3.0),
            (-5.0, -3.0, 5.0, -3.0),
            (-5.0, 3.0, 5.0, 3.0),
        ] {
            for step in 0..=200 {
                let t = step as f64 / 200.0;
                let world = Vec3::new(
                    min_x + (max_x - min_x) * t,
                    min_y + (max_y - min_y) * t,
                    0.0,
                );
                if let Some(coord) = grid.world_to_grid(world) {
                    grid.reset(coord);
                    grid.mark_occupied(coord);
                }
            }
        }
        grid
    }

    fn room_scan(x_m: f64, y_m: f64) -> LaserScan2d {
        let beams = 360;
        let mut ranges = Vec::with_capacity(beams);
        for beam in 0..beams {
            let angle = TAU * beam as f64 / beams as f64;
            ranges.push(ray_to_wall(x_m, y_m, angle));
        }
        LaserScan2d {
            time_s: 0.0,
            frame: rne_nav::FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 30.0,
            ranges_m: ranges,
        }
    }

    fn ray_to_wall(x: f64, y: f64, angle: f64) -> f64 {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for (bound, position, direction) in
            [(5.0, x, dx), (-5.0, x, dx), (3.0, y, dy), (-3.0, y, dy)]
        {
            if direction.abs() > 1.0e-9 {
                let t = (bound - position) / direction;
                if t > 0.0 {
                    best = best.min(t);
                }
            }
        }
        best
    }

    fn run_localization() -> (Amcl, Pose2d) {
        let map = room_map();
        let config = AmclConfig::default();
        let initial = Pose2d::new(-2.0, 0.5, 0.1);
        let mut amcl = Amcl::new(&map, Pose2d::new(-2.1, 0.6, 0.05), config).unwrap();
        let mut truth = initial;
        let mut odom = Pose2d::new(-2.1, 0.6, 0.05);
        for _ in 0..15 {
            truth.x_m += 0.1;
            odom.x_m += 0.1;
            odom.y_m += 0.005;
            let scan = room_scan(truth.x_m, truth.y_m);
            amcl.update(&scan, odom, Pose2d::IDENTITY);
        }
        (amcl, truth)
    }

    #[test]
    fn converges_near_truth() {
        let (amcl, truth) = run_localization();
        let estimate = amcl.estimate();
        assert!(
            (estimate.x_m - truth.x_m).hypot(estimate.y_m - truth.y_m) < 0.25,
            "estimate={estimate:?} truth={truth:?}"
        );
    }

    #[test]
    fn localization_is_deterministic() {
        let (first, _) = run_localization();
        let (second, _) = run_localization();
        assert_eq!(
            first.estimate().x_m.to_bits(),
            second.estimate().x_m.to_bits()
        );
        assert_eq!(
            first.estimate().y_m.to_bits(),
            second.estimate().y_m.to_bits()
        );
        assert_eq!(
            first.estimate().yaw_rad.to_bits(),
            second.estimate().yaw_rad.to_bits()
        );
    }
}
