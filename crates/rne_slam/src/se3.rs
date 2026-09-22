//! Rigid-body SE(3) utilities for 6-DoF estimation.
//!
//! Backend-neutral helpers built on `rne_math`'s `Quat`/`Vec3`: twist/blend
//! exponential and logarithmic maps, rotations, and the SO(3) left Jacobian and
//! its inverse. The tangent vector convention is rotation-first,
//! `xi = [phi_x, phi_y, phi_z, rho_x, rho_y, rho_z]`, matching
//! `se3_log`'s output so perturbation loops are consistent.

use rne_math::{Quat, Transform3, Vec3};
use serde::{Deserialize, Serialize};

/// A rigid transform: rotation followed by translation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Se3 {
    /// Unit rotation.
    pub rotation: Quat,
    /// Translation in meters.
    pub translation: Vec3,
}

impl Default for Se3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Se3 {
    /// Identity transform.
    pub const IDENTITY: Self = Self {
        rotation: Quat::IDENTITY,
        translation: Vec3::ZERO,
    };

    /// Creates a transform from a rotation and translation.
    pub fn new(rotation: Quat, translation: Vec3) -> Self {
        Self {
            rotation: rotation.normalize(),
            translation,
        }
    }

    /// Converts from a world [`Transform3`], ignoring scale.
    pub fn from_transform3(transform: &Transform3) -> Self {
        Self::new(transform.rotation, transform.translation)
    }

    /// Converts to a unit-scale [`Transform3`].
    pub fn to_transform3(self) -> Transform3 {
        Transform3 {
            translation: self.translation,
            rotation: self.rotation,
            scale: Vec3::ONE,
        }
    }

    /// Returns `self * other`.
    pub fn compose(self, other: Self) -> Self {
        Self {
            rotation: (self.rotation * other.rotation).normalize(),
            translation: self.translation + self.rotation.mul_vec3(other.translation),
        }
    }

    /// Returns the inverse transform.
    pub fn inverse(self) -> Self {
        let rotation = self.rotation.conjugate();
        Self {
            rotation,
            translation: rotation.mul_vec3(-self.translation),
        }
    }

    /// Transforms a point from this frame into the parent frame.
    pub fn transform_point(self, point: Vec3) -> Vec3 {
        self.rotation.mul_vec3(point) + self.translation
    }

    /// Exponential map from a tangent vector (rotation-first).
    pub fn exp(xi: [f64; 6]) -> Self {
        let phi = Vec3::new(xi[0], xi[1], xi[2]);
        let rho = Vec3::new(xi[3], xi[4], xi[5]);
        let rotation = so3_exp(phi);
        let jacobian = so3_left_jacobian(phi);
        Self {
            rotation,
            translation: mat3_vec(jacobian, rho),
        }
    }

    /// Logarithmic map to a tangent vector (rotation-first).
    pub fn log(self) -> [f64; 6] {
        let phi = so3_log(self.rotation);
        let rho = mat3_vec(so3_left_jacobian_inverse(phi), self.translation);
        [phi.x, phi.y, phi.z, rho.x, rho.y, rho.z]
    }

    /// True when the rotation is normalized and every value is finite.
    pub fn is_finite(self) -> bool {
        self.translation.is_finite()
            && self.rotation.x.is_finite()
            && self.rotation.y.is_finite()
            && self.rotation.z.is_finite()
            && self.rotation.w.is_finite()
            && (self.rotation.length_squared() - 1.0).abs() < 1.0e-6
    }
}

/// SO(3) exponential map: rotation vector (radians) to unit quaternion.
pub fn so3_exp(phi: Vec3) -> Quat {
    let theta = phi.length();
    if theta < 1.0e-12 {
        // First-order approximation keeps near-zero twists exact and stable.
        return Quat::from_xyzw(0.5 * phi.x, 0.5 * phi.y, 0.5 * phi.z, 1.0).normalize();
    }
    Quat::from_axis_angle(phi / theta, theta)
}

/// SO(3) logarithmic map: unit quaternion to rotation vector (radians).
pub fn so3_log(quaternion: Quat) -> Vec3 {
    let quaternion = if quaternion.w < 0.0 {
        -quaternion
    } else {
        quaternion
    };
    let vector = Vec3::new(quaternion.x, quaternion.y, quaternion.z);
    let norm = vector.length();
    if norm < 1.0e-12 {
        return 2.0 * vector;
    }
    let angle = 2.0 * norm.atan2(quaternion.w);
    vector / norm * angle
}

/// SO(3) left Jacobian `J = I + A*hat(phi) + B*hat(phi)^2`.
pub fn so3_left_jacobian(phi: Vec3) -> [[f64; 3]; 3] {
    let theta = phi.length();
    let (a, b) = if theta < 1.0e-8 {
        (0.5, 1.0 / 6.0)
    } else {
        (
            (1.0 - theta.cos()) / (theta * theta),
            (theta - theta.sin()) / (theta * theta * theta),
        )
    };
    let omega = skew(phi);
    let omega2 = mat3_mul(omega, omega);
    mat3_add(
        mat3_identity(),
        mat3_add(mat3_scale(omega, a), mat3_scale(omega2, b)),
    )
}

/// Inverse SO(3) left Jacobian `J^{-1} = I - 0.5*hat(phi) + C*hat(phi)^2`.
pub fn so3_left_jacobian_inverse(phi: Vec3) -> [[f64; 3]; 3] {
    let theta = phi.length();
    let c = if theta < 1.0e-6 {
        1.0 / 12.0
    } else {
        1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin())
    };
    let omega = skew(phi);
    let omega2 = mat3_mul(omega, omega);
    mat3_add(
        mat3_identity(),
        mat3_add(mat3_scale(omega, -0.5), mat3_scale(omega2, c)),
    )
}

/// Skew-symmetric matrix of a vector.
pub fn skew(vector: Vec3) -> [[f64; 3]; 3] {
    [
        [0.0, -vector.z, vector.y],
        [vector.z, 0.0, -vector.x],
        [-vector.y, vector.x, 0.0],
    ]
}

/// Identity 3x3 matrix.
pub fn mat3_identity() -> [[f64; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

/// Matrix product.
pub fn mat3_mul(left: [[f64; 3]; 3], right: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = (0..3).map(|k| left[row][k] * right[k][col]).sum();
        }
    }
    out
}

/// Elementwise sum.
pub fn mat3_add(left: [[f64; 3]; 3], right: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = left[row][col] + right[row][col];
        }
    }
    out
}

/// Scalar product.
pub fn mat3_scale(matrix: [[f64; 3]; 3], scale: f64) -> [[f64; 3]; 3] {
    let mut out = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = matrix[row][col] * scale;
        }
    }
    out
}

/// Matrix-vector product.
pub fn mat3_vec(matrix: [[f64; 3]; 3], vector: Vec3) -> Vec3 {
    Vec3::new(
        matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn exp_log_round_trip() {
        let xi = [0.3, -0.7, 1.2, 0.5, -0.2, 0.9];
        let transform = Se3::exp(xi);
        assert!(transform.is_finite());
        let recovered = transform.log();
        for (a, b) in xi.iter().zip(recovered.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-9);
        }
    }

    #[test]
    fn compose_inverse_is_identity() {
        let transform = Se3::new(
            Quat::from_axis_angle(Vec3::new(0.2, 0.5, 0.8).normalize(), 0.9),
            Vec3::new(1.0, -2.0, 3.0),
        );
        let identity = transform.compose(transform.inverse());
        assert_relative_eq!(identity.translation.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(identity.translation.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(identity.translation.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn small_twist_is_first_order() {
        let xi = [1e-9, 0.0, 0.0, 2e-9, 0.0, 0.0];
        let transform = Se3::exp(xi);
        assert_relative_eq!(transform.translation.x, 2e-9, epsilon = 1e-15);
    }
}
