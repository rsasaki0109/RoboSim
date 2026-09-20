//! Spatial vector algebra for articulated-body dynamics.
//!
//! The implementation follows the Featherstone formulation with the vector
//! ordering used elsewhere in RNE: motion `[linear; angular]` and force
//! `[force; torque]`. All matrices are row-major fixed-size arrays so the
//! arithmetic is deterministic and allocation-free at the leaf level.

use rne_math::{Quat, Vec3};
use rne_world::Transform3;

/// Row-major `3x3` matrix.
pub type Mat3 = [[f64; 3]; 3];

/// Row-major `6x6` matrix.
pub type Mat6 = [[f64; 6]; 6];

/// Spatial motion or force vector ordered `[x, y, z, rx, ry, rz]`.
pub type SpatialVec = [f64; 6];

/// Identity `3x3` matrix.
pub fn mat3_identity() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

/// Zero `3x3` matrix.
pub fn mat3_zero() -> Mat3 {
    [[0.0; 3]; 3]
}

/// Skew-symmetric cross-product matrix `[v]x`.
pub fn skew(v: Vec3) -> Mat3 {
    [[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]]
}

/// Converts a rotation quaternion to a row-major `3x3` rotation matrix.
pub fn rotation_matrix(rotation: Quat) -> Mat3 {
    let q = rotation.normalize();
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Multiplies a `3x3` matrix by a vector.
pub fn mat3_mul_vec(m: &Mat3, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

/// Multiplies two `3x3` matrices.
pub fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = mat3_zero();
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, cell) in out_row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

/// Transposes a `3x3` matrix.
pub fn mat3_transpose(m: &Mat3) -> Mat3 {
    let mut out = mat3_zero();
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = m[column][row];
        }
    }
    out
}

/// Scales a `3x3` matrix.
pub fn mat3_scale(m: &Mat3, factor: f64) -> Mat3 {
    let mut out = *m;
    for row in out.iter_mut() {
        for value in row.iter_mut() {
            *value *= factor;
        }
    }
    out
}

/// Adds two `3x3` matrices.
pub fn mat3_add(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = *a;
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] += b[row][column];
        }
    }
    out
}

/// Identity `6x6` matrix.
pub fn mat6_identity() -> Mat6 {
    let mut out = mat6_zero();
    for (index, row) in out.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    out
}

/// Zero `6x6` matrix.
pub fn mat6_zero() -> Mat6 {
    [[0.0; 6]; 6]
}

/// Multiplies a `6x6` matrix by a spatial vector.
pub fn mat6_mul_vec(m: &Mat6, v: &SpatialVec) -> SpatialVec {
    let mut out = [0.0; 6];
    for (row, value) in out.iter_mut().enumerate() {
        *value = (0..6).map(|column| m[row][column] * v[column]).sum();
    }
    out
}

/// Multiplies a transposed `6x6` matrix by a spatial vector.
pub fn mat6_transpose_mul_vec(m: &Mat6, v: &SpatialVec) -> SpatialVec {
    let mut out = [0.0; 6];
    for (row, value) in out.iter_mut().enumerate() {
        *value = (0..6).map(|column| m[column][row] * v[column]).sum();
    }
    out
}

/// Multiplies two `6x6` matrices.
pub fn mat6_mul(a: &Mat6, b: &Mat6) -> Mat6 {
    let mut out = mat6_zero();
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, cell) in out_row.iter_mut().enumerate() {
            *cell = (0..6).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

/// Adds two `6x6` matrices.
pub fn mat6_add(a: &Mat6, b: &Mat6) -> Mat6 {
    let mut out = *a;
    for row in 0..6 {
        for column in 0..6 {
            out[row][column] += b[row][column];
        }
    }
    out
}

/// Adds two spatial vectors.
pub fn add6(a: &SpatialVec, b: &SpatialVec) -> SpatialVec {
    let mut out = [0.0; 6];
    for (index, value) in out.iter_mut().enumerate() {
        *value = a[index] + b[index];
    }
    out
}

/// Scales a spatial vector.
pub fn scale6(v: &SpatialVec, factor: f64) -> SpatialVec {
    let mut out = [0.0; 6];
    for (index, value) in out.iter_mut().enumerate() {
        *value = v[index] * factor;
    }
    out
}

/// Dot product of two spatial vectors.
pub fn dot6(a: &SpatialVec, b: &SpatialVec) -> f64 {
    (0..6).map(|index| a[index] * b[index]).sum()
}

/// Inverts a rigid transform with unit scale.
pub fn inverse_transform(transform: &Transform3) -> Transform3 {
    let rotation = transform.rotation.conjugate();
    let translation = rotation * -transform.translation;
    Transform3::from_translation_rotation(translation, rotation)
}

/// Transforms a point by a rigid transform.
pub fn transform_point(transform: &Transform3, point: Vec3) -> Vec3 {
    transform.translation + transform.rotation * point
}

/// Spatial motion transform from frame `B` to frame `A`.
///
/// `a_from_b` is the pose of `B` expressed in `A` (`x_A = R * x_B + p`). The
/// returned matrix maps a motion vector expressed in `B` to the equivalent
/// motion vector referred to the origin of `A`.
pub fn motion_transform(a_from_b: &Transform3) -> Mat6 {
    let r = rotation_matrix(a_from_b.rotation);
    let pr = mat3_mul(&skew(a_from_b.translation), &r);
    let mut out = mat6_zero();
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = r[row][column];
            out[row][column + 3] = pr[row][column];
            out[row + 3][column + 3] = r[row][column];
        }
    }
    out
}

/// World-frame first-order perturbation of a rigid transform.
///
/// `translation` is the derivative of the transform translation and
/// `angular_velocity` is the derivative of the rotation such that
/// `d(rotation_matrix) = skew(angular_velocity) * rotation_matrix`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransformPerturbation {
    /// Derivative of the translation, in the parent frame.
    pub translation: Vec3,
    /// World-frame angular velocity of the rotation.
    pub angular_velocity: Vec3,
}

impl TransformPerturbation {
    /// Zero perturbation.
    pub const ZERO: Self = Self {
        translation: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
    };
}

/// Builds the world-frame perturbation of `pose` from a body-frame twist.
pub fn perturbation_from_body_twist(
    pose: &Transform3,
    twist: &SpatialVec,
) -> TransformPerturbation {
    let rotation = pose.rotation;
    TransformPerturbation {
        translation: rotation * Vec3::new(twist[0], twist[1], twist[2]),
        angular_velocity: rotation * Vec3::new(twist[3], twist[4], twist[5]),
    }
}

/// Derivative of [`motion_transform`] under a world-frame perturbation.
///
/// The blocks of `Ad(a_from_b)` are `R`, `skew(p) R`, and `R`. Given
/// `dR = skew(omega) R` and `dp`, the derivative is assembled block-wise.
pub fn motion_transform_derivative(
    a_from_b: &Transform3,
    perturbation: &TransformPerturbation,
) -> Mat6 {
    let r = rotation_matrix(a_from_b.rotation);
    let omega = perturbation.angular_velocity;
    let d_r = {
        let s = skew(omega);
        mat3_mul(&s, &r)
    };
    // d(skew(p) R) = skew(dp) R + skew(p) dR.
    let d_pr = {
        let left = mat3_mul(&skew(perturbation.translation), &r);
        let right = mat3_mul(&skew(a_from_b.translation), &d_r);
        mat3_add(&left, &right)
    };
    let mut out = mat6_zero();
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = d_r[row][column];
            out[row][column + 3] = d_pr[row][column];
            out[row + 3][column + 3] = d_r[row][column];
        }
    }
    out
}

/// Spatial motion cross product `v x m`.
pub fn cross_motion(v: &SpatialVec, m: &SpatialVec) -> SpatialVec {
    let vl = Vec3::new(v[0], v[1], v[2]);
    let w = Vec3::new(v[3], v[4], v[5]);
    let ml = Vec3::new(m[0], m[1], m[2]);
    let wl = Vec3::new(m[3], m[4], m[5]);
    let linear = w.cross(ml) + vl.cross(wl);
    let angular = w.cross(wl);
    [
        linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
    ]
}

/// Spatial force cross product `v x* f`.
///
/// This is the dual of [`cross_motion`] and satisfies
/// `dot(cross_motion(v, m), f) + dot(m, cross_force(v, f)) == 0` for every
/// motion vector `v`, motion vector `m`, and force vector `f`.
pub fn cross_force(v: &SpatialVec, f: &SpatialVec) -> SpatialVec {
    let vl = Vec3::new(v[0], v[1], v[2]);
    let w = Vec3::new(v[3], v[4], v[5]);
    let fl = Vec3::new(f[0], f[1], f[2]);
    let torque = Vec3::new(f[3], f[4], f[5]);
    let force = w.cross(fl);
    let moment = w.cross(torque) + vl.cross(fl);
    [force.x, force.y, force.z, moment.x, moment.y, moment.z]
}

/// Spatial inertia of a rigid body expressed about a frame origin.
///
/// The rotational inertia is the body-frame tensor about the center of mass,
/// matching [`rne_physics::RigidBodyInertia`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpatialInertia {
    /// Mass in kilograms.
    pub mass_kg: f64,
    /// Center of mass in the frame, in meters.
    pub center_of_mass_m: Vec3,
    /// Rotational inertia about the center of mass, in kg·m².
    pub inertia_about_com_kg_m2: Mat3,
}

impl SpatialInertia {
    /// Zero inertia.
    pub const ZERO: Self = Self {
        mass_kg: 0.0,
        center_of_mass_m: Vec3::ZERO,
        inertia_about_com_kg_m2: [[0.0; 3]; 3],
    };

    /// Creates a spatial inertia from mass, center of mass, and tensor.
    pub fn new(mass_kg: f64, center_of_mass_m: Vec3, inertia_about_com_kg_m2: Mat3) -> Self {
        Self {
            mass_kg,
            center_of_mass_m,
            inertia_about_com_kg_m2,
        }
    }

    /// Creates a point mass with no rotational inertia.
    pub fn point_mass(mass_kg: f64, center_of_mass_m: Vec3) -> Self {
        Self::new(mass_kg, center_of_mass_m, mat3_zero())
    }

    /// Builds the `6x6` spatial inertia matrix ordered `[force; torque]` for a
    /// motion vector ordered `[linear; angular]`.
    pub fn matrix(&self) -> Mat6 {
        let c = skew(self.center_of_mass_m);
        let cx_cx = mat3_mul(&c, &c);
        let mass = self.mass_kg;
        let top_right = mat3_scale(&c, -mass);
        let bottom_left = mat3_scale(&c, mass);
        let bottom_right = {
            let mut out = self.inertia_about_com_kg_m2;
            let correction = mat3_scale(&cx_cx, mass);
            for row in 0..3 {
                for column in 0..3 {
                    out[row][column] -= correction[row][column];
                }
            }
            out
        };
        let mut out = mat6_zero();
        for row in 0..3 {
            for column in 0..3 {
                out[row][column] = if row == column { mass } else { 0.0 };
                out[row][column + 3] = top_right[row][column];
                out[row + 3][column] = bottom_left[row][column];
                out[row + 3][column + 3] = bottom_right[row][column];
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cross_products_are_dual() {
        let v = [0.3, -0.4, 0.7, 0.2, 0.5, -0.1];
        let m = [-0.6, 0.2, 0.4, 0.1, -0.3, 0.8];
        let f = [0.5, 0.9, -0.2, -0.4, 0.6, 0.3];
        let left = dot6(&cross_motion(&v, &m), &f);
        let right = dot6(&m, &cross_force(&v, &f));
        assert!((left + right).abs() < 1.0e-12);
    }

    #[test]
    fn motion_transform_round_trips_rotation() {
        let t = Transform3::from_translation_rotation(
            Vec3::new(0.3, -0.2, 0.5),
            Quat::from_rotation_z(0.4),
        );
        let forward = motion_transform(&t);
        let backward = motion_transform(&inverse_transform(&t));
        let motion = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let round_trip = mat6_mul_vec(&backward, &mat6_mul_vec(&forward, &motion));
        for index in 0..6 {
            assert!((round_trip[index] - motion[index]).abs() < 1.0e-12);
        }
    }

    #[test]
    fn point_mass_matrix_matches_hand_derivation() {
        let inertia = SpatialInertia::point_mass(2.0, Vec3::new(1.0, 0.0, 0.0));
        let m = inertia.matrix();
        assert!((m[0][0] - 2.0).abs() < 1.0e-12);
        assert!((m[4][4] - 2.0).abs() < 1.0e-12);
        assert!((m[5][5] - 2.0).abs() < 1.0e-12);
        // Point mass at +x: angular momentum about z for linear velocity in y is
        // m * x = 2.0, i.e. I[1][5] = m * c_x.
        assert!((m[1][5] - 2.0).abs() < 1.0e-12);
    }
}
