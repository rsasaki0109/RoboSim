//! Planar poses used by maps, scans, and the transform tree.

use rne_math::{Quat, Transform3, Vec3};
use serde::{Deserialize, Serialize};

/// A rigid 2D pose `(x, y, yaw)` in meters and radians.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose2d {
    /// Translation along world x in meters.
    pub x_m: f64,
    /// Translation along world y in meters.
    pub y_m: f64,
    /// Counter-clockwise yaw about world z in radians.
    pub yaw_rad: f64,
}

impl Pose2d {
    /// Identity pose.
    pub const IDENTITY: Self = Self {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };

    /// Creates a pose from components.
    pub fn new(x_m: f64, y_m: f64, yaw_rad: f64) -> Self {
        Self { x_m, y_m, yaw_rad }
    }

    /// Creates a pose from a translation vector and yaw; `z` is ignored.
    pub fn from_translation_yaw(translation_m: Vec3, yaw_rad: f64) -> Self {
        Self {
            x_m: translation_m.x,
            y_m: translation_m.y,
            yaw_rad,
        }
    }

    /// Converts the pose to a [`Transform3`] with no scale.
    pub fn to_transform3(self) -> Transform3 {
        Transform3::from_translation_rotation(
            Vec3::new(self.x_m, self.y_m, 0.0),
            Quat::from_rotation_z(self.yaw_rad),
        )
    }

    /// Composes this pose with a child pose (`self * child`).
    pub fn compose(self, child: Self) -> Self {
        let (sin, cos) = self.yaw_rad.sin_cos();
        Self {
            x_m: self.x_m + cos * child.x_m - sin * child.y_m,
            y_m: self.y_m + sin * child.x_m + cos * child.y_m,
            yaw_rad: self.yaw_rad + child.yaw_rad,
        }
    }

    /// Returns the inverse pose.
    pub fn inverse(self) -> Self {
        let (sin, cos) = self.yaw_rad.sin_cos();
        Self {
            x_m: -(cos * self.x_m + sin * self.y_m),
            y_m: -(-sin * self.x_m + cos * self.y_m),
            yaw_rad: -self.yaw_rad,
        }
    }

    /// Transforms a point from this pose's frame into the world frame.
    pub fn transform_point(self, point_m: Vec3) -> Vec3 {
        let (sin, cos) = self.yaw_rad.sin_cos();
        Vec3::new(
            self.x_m + cos * point_m.x - sin * point_m.y,
            self.y_m + sin * point_m.x + cos * point_m.y,
            point_m.z,
        )
    }

    /// Transforms a point from the world frame into this pose's frame.
    pub fn inverse_transform_point(self, point_m: Vec3) -> Vec3 {
        self.inverse().transform_point(point_m)
    }

    /// Whether every component is finite.
    pub fn is_finite(self) -> bool {
        self.x_m.is_finite() && self.y_m.is_finite() && self.yaw_rad.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn inverse_round_trips_a_point() {
        let pose = Pose2d::new(1.5, -2.0, 0.7);
        let point = Vec3::new(0.3, 0.9, 0.0);
        let world = pose.transform_point(point);
        let back = pose.inverse_transform_point(world);
        assert_relative_eq!(back.x, point.x, epsilon = 1e-12);
        assert_relative_eq!(back.y, point.y, epsilon = 1e-12);
    }

    #[test]
    fn compose_matches_transform3() {
        let a = Pose2d::new(1.0, 2.0, 0.5);
        let b = Pose2d::new(-0.4, 0.6, -0.2);
        let composed = a.to_transform3().mul_transform(&b.to_transform3());
        let expected = a.compose(b).to_transform3();
        assert_relative_eq!(
            composed.translation.x,
            expected.translation.x,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            composed.translation.y,
            expected.translation.y,
            epsilon = 1e-12
        );
        assert_relative_eq!(composed.rotation.z, expected.rotation.z, epsilon = 1e-12);
        assert_relative_eq!(composed.rotation.w, expected.rotation.w, epsilon = 1e-12);
    }
}
