//! LiDAR sensor specification and sampling.

use rne_data::PointCloud;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::{PhysicsBackend, PhysicsWorldId, RaycastQuery};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// 2D scanning LiDAR parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LidarSpec {
    /// Number of rays per scan.
    pub ray_count: u32,
    /// Minimum azimuth angle in radians.
    pub min_angle_rad: f64,
    /// Maximum azimuth angle in radians.
    pub max_angle_rad: f64,
    /// Maximum range in meters.
    pub max_range_m: f64,
    /// Vertical offset of the scan plane in the sensor frame.
    pub height_offset_m: f64,
}

impl Default for LidarSpec {
    fn default() -> Self {
        Self {
            ray_count: 360,
            min_angle_rad: -std::f64::consts::PI,
            max_angle_rad: std::f64::consts::PI,
            max_range_m: 20.0,
            height_offset_m: 0.2,
        }
    }
}

/// Samples a horizontal LiDAR scan using the physics raycast backend.
pub fn sample_lidar<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    mount_transform: &Transform3,
    spec: &LidarSpec,
) -> PointCloud {
    let mut points_m = Vec::with_capacity(spec.ray_count as usize);
    let origin = mount_transform.translation + Vec3::new(0.0, spec.height_offset_m, 0.0);

    for ray_index in 0..spec.ray_count {
        let t = if spec.ray_count <= 1 {
            0.0
        } else {
            ray_index as f64 / (spec.ray_count - 1) as f64
        };
        let angle = spec.min_angle_rad + (spec.max_angle_rad - spec.min_angle_rad) * t;
        let direction = mount_transform.rotation * Vec3::new(angle.cos(), 0.0, angle.sin());

        let query = RaycastQuery {
            origin_m: origin,
            direction,
            max_distance_m: spec.max_range_m,
        };

        if let Ok(hits) = backend.raycast(physics_world, query) {
            if let Some(hit) = hits.first() {
                points_m.push(hit.point_m);
            }
        }
    }

    PointCloud { points_m }
}

/// Convenience mount lookup for a sensor entity.
pub fn sample_lidar_at_entity<B: PhysicsBackend>(
    backend: &B,
    physics_world: PhysicsWorldId,
    world: &rne_ecs::World,
    entity: Entity,
    spec: &LidarSpec,
) -> PointCloud {
    let transform = world.get::<Transform3>(entity).copied().unwrap_or_default();
    sample_lidar(backend, physics_world, &transform, spec)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Quat;
    use std::f64::consts::TAU;

    #[test]
    fn lidar_ray_directions_are_normalized() {
        let transform = Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY);
        let spec = LidarSpec {
            ray_count: 4,
            min_angle_rad: 0.0,
            max_angle_rad: TAU,
            max_range_m: 10.0,
            height_offset_m: 0.0,
        };

        // Without backend, verify angle stepping logic only.
        let t = 1.0 / 3.0;
        let angle = spec.min_angle_rad + (spec.max_angle_rad - spec.min_angle_rad) * t;
        let direction = transform.rotation * Vec3::new(angle.cos(), 0.0, angle.sin());
        assert_relative_eq!(direction.length(), 1.0, epsilon = 1e-9);
    }
}
