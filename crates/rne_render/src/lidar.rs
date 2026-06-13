//! LiDAR hit visualization helpers.

use crate::scene::{RenderScene, RenderSceneItem};
use crate::visual::VisualShape;
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};

/// Default marker radius for LiDAR hit spheres in meters.
pub const DEFAULT_LIDAR_POINT_RADIUS_M: f64 = 0.04;

impl RenderScene {
    /// Appends one sphere marker per LiDAR hit using [`DEFAULT_LIDAR_POINT_RADIUS_M`].
    pub fn append_lidar_points(&mut self, points_m: &[Vec3], color_rgba: [f32; 4]) {
        self.append_lidar_points_sized(points_m, DEFAULT_LIDAR_POINT_RADIUS_M, color_rgba);
    }

    /// Appends one sphere marker per LiDAR hit with the given radius.
    pub fn append_lidar_points_sized(
        &mut self,
        points_m: &[Vec3],
        point_radius_m: f64,
        color_rgba: [f32; 4],
    ) {
        let diameter = point_radius_m * 2.0;
        let scale = Vec3::splat(diameter);
        for point in points_m {
            self.items.push(RenderSceneItem {
                transform: MathTransform3 {
                    translation: *point,
                    rotation: Quat::IDENTITY,
                    scale,
                },
                shape: VisualShape::Sphere {
                    radius_m: point_radius_m,
                },
                color_rgba,
                mesh: None,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Vec3;

    #[test]
    fn append_lidar_points_adds_sphere_items() {
        let points = [Vec3::new(1.0, 0.2, 0.0), Vec3::new(2.0, 0.2, 0.5)];
        let mut scene = RenderScene::new();
        scene.append_lidar_points(&points, [0.1, 0.9, 0.2, 1.0]);
        assert_eq!(scene.items.len(), 2);
        assert!(matches!(scene.items[0].shape, VisualShape::Sphere { .. }));
    }
}
