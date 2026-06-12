//! Transforms and spatial poses.

use crate::{Mat4, Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Local transform with translation, rotation, and scale.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    /// Translation in meters.
    pub translation: Vec3,
    /// Rotation as a unit quaternion.
    pub rotation: Quat,
    /// Non-uniform scale.
    pub scale: Vec3,
}

impl Transform3 {
    /// Identity transform.
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3::ONE,
    };

    /// Creates a transform from translation and rotation.
    pub fn from_translation_rotation(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
            scale: Vec3::ONE,
        }
    }

    /// Converts the transform to a 4x4 matrix.
    pub fn to_matrix(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(self.scale, self.rotation, self.translation)
    }

    /// Composes this transform with another (`self * rhs`).
    pub fn mul_transform(&self, rhs: &Self) -> Self {
        Self {
            translation: self.translation + self.rotation * (self.scale * rhs.translation),
            rotation: self.rotation * rhs.rotation,
            scale: self.scale * rhs.scale,
        }
    }

    /// Returns the inverse transform.
    pub fn inverse(&self) -> Self {
        let inv_rotation = self.rotation.conjugate();
        let inv_scale = Vec3::new(1.0 / self.scale.x, 1.0 / self.scale.y, 1.0 / self.scale.z);
        let inv_translation = inv_rotation * (-(inv_scale * self.translation));

        Self {
            translation: inv_translation,
            rotation: inv_rotation,
            scale: inv_scale,
        }
    }

    /// Transforms a point by this transform.
    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.translation + self.rotation * (self.scale * point)
    }
}

/// Position and orientation without scale.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose3 {
    /// Position in meters.
    pub translation: Vec3,
    /// Orientation as a unit quaternion.
    pub rotation: Quat,
}

impl Pose3 {
    /// Identity pose.
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    /// Converts the pose to a transform with unit scale.
    pub fn to_transform(&self) -> Transform3 {
        Transform3::from_translation_rotation(self.translation, self.rotation)
    }
}

/// Linear and angular velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Velocity3 {
    /// Linear velocity in meters per second.
    pub linear_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_rad_s: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn transform_compose() {
        let a = Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY);
        let b = Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY);
        let composed = a.mul_transform(&b);

        assert_relative_eq!(composed.translation.x, 1.0);
        assert_relative_eq!(composed.translation.y, 2.0);
        assert_relative_eq!(composed.translation.z, 0.0);
    }

    #[test]
    fn transform_inverse() {
        let transform = Transform3::from_translation_rotation(
            Vec3::new(3.0, -1.0, 2.0),
            Quat::from_rotation_y(0.5),
        );
        let identity = transform.mul_transform(&transform.inverse());

        assert_relative_eq!(identity.translation.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.translation.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.translation.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.rotation.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.rotation.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.rotation.z, 0.0, epsilon = 1e-10);
        assert_relative_eq!(identity.rotation.w, 1.0, epsilon = 1e-10);
    }
}
