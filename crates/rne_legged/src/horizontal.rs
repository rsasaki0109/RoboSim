//! Two-dimensional horizontal geometry for walking patterns.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

/// A horizontal position or displacement in the world `X`/`Z` plane.
///
/// The world is Y-up, so the vertical component is not stored here. Use
/// [`Horizontal::to_world`] to lift a horizontal value to a world
/// [`rne_math::Vec3`] at a chosen height.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Horizontal {
    /// World `X` component in meters.
    pub x_m: f64,
    /// World `Z` component in meters.
    pub z_m: f64,
}

impl Horizontal {
    /// The origin.
    pub const ZERO: Self = Self { x_m: 0.0, z_m: 0.0 };

    /// Creates a horizontal value from `X` and `Z` components in meters.
    pub fn new(x_m: f64, z_m: f64) -> Self {
        Self { x_m, z_m }
    }

    /// Extracts the horizontal components of a world position.
    pub fn from_world(position_m: Vec3) -> Self {
        Self::new(position_m.x, position_m.z)
    }

    /// Lifts the horizontal value to a world position at the given height.
    pub fn to_world(self, height_m: f64) -> Vec3 {
        Vec3::new(self.x_m, height_m, self.z_m)
    }

    /// Uniform scale.
    pub fn scale(self, factor: f64) -> Self {
        Self::new(self.x_m * factor, self.z_m * factor)
    }

    /// Dot product.
    pub fn dot(self, other: Self) -> f64 {
        self.x_m * other.x_m + self.z_m * other.z_m
    }

    /// Euclidean length.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Returns a unit vector, or [`Horizontal::ZERO`] when the length is zero.
    pub fn normalize_or_zero(self) -> Self {
        let length = self.norm();
        if length <= f64::EPSILON {
            Self::ZERO
        } else {
            self.scale(1.0 / length)
        }
    }

    /// Returns the vector rotated 90 degrees counter-clockwise in the `X`/`Z`
    /// plane.
    pub fn perpendicular_left(self) -> Self {
        Self::new(-self.z_m, self.x_m)
    }

    /// Linear interpolation from `self` to `other` by `t`.
    pub fn lerp(self, other: Self, t: f64) -> Self {
        self.scale(1.0 - t).add(other.scale(t))
    }

    /// Whether both components are finite.
    pub fn is_finite(self) -> bool {
        self.x_m.is_finite() && self.z_m.is_finite()
    }
}

impl Add for Horizontal {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x_m + rhs.x_m, self.z_m + rhs.z_m)
    }
}

impl Sub for Horizontal {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x_m - rhs.x_m, self.z_m - rhs.z_m)
    }
}

impl Mul<f64> for Horizontal {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        self.scale(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perpendicular_is_orthogonal_and_unit() {
        let direction = Horizontal::new(1.0, 0.0);
        let left = direction.perpendicular_left();
        assert!((direction.dot(left)).abs() < 1.0e-12);
        assert!((left.norm() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn world_round_trip_preserves_horizontal_components() {
        let world = Vec3::new(1.5, 0.3, -2.0);
        let horizontal = Horizontal::from_world(world);
        let lifted = horizontal.to_world(0.3);
        assert!((lifted.x - 1.5).abs() < 1.0e-12);
        assert!((lifted.y - 0.3).abs() < 1.0e-12);
        assert!((lifted.z + 2.0).abs() < 1.0e-12);
    }
}
