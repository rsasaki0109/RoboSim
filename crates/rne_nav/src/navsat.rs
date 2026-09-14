//! Geographic (NavSat) to local map projection.
//!
//! [`NavSatTransform`] is the RNE analogue of `robot_localization`'s
//! `navsat_transform_node`: it fixes an ENU local frame at a datum and converts
//! WGS-84 latitude/longitude/altitude to and from the planar map frame using an
//! equirectangular approximation. The approximation is accurate for the local,
//! small-area use a robot map needs and keeps the conversion deterministic and
//! dependency-free.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Mean Earth radius in meters (WGS-84 authalic radius).
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Errors raised by the NavSat transform.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum NavSatError {
    /// A configuration value was non-finite or out of range.
    #[error("invalid NavSat configuration")]
    InvalidConfig,
    /// A geographic or map input was non-finite.
    #[error("non-finite NavSat input")]
    NonFiniteInput,
}

/// A local ENU frame fixed at a geographic datum.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NavSatTransform {
    /// Latitude of the datum in radians.
    pub datum_latitude_rad: f64,
    /// Longitude of the datum in radians.
    pub datum_longitude_rad: f64,
    /// Altitude of the datum in meters.
    pub datum_altitude_m: f64,
    /// Rotation of the map frame relative to ENU, in radians.
    pub datum_yaw_rad: f64,
    /// Earth radius used for the projection, in meters.
    pub earth_radius_m: f64,
}

impl Default for NavSatTransform {
    fn default() -> Self {
        Self {
            datum_latitude_rad: 0.0,
            datum_longitude_rad: 0.0,
            datum_altitude_m: 0.0,
            datum_yaw_rad: 0.0,
            earth_radius_m: EARTH_RADIUS_M,
        }
    }
}

impl NavSatTransform {
    /// Creates a transform after validating the datum.
    pub fn new(
        datum_latitude_rad: f64,
        datum_longitude_rad: f64,
        datum_altitude_m: f64,
    ) -> Result<Self, NavSatError> {
        let transform = Self {
            datum_latitude_rad,
            datum_longitude_rad,
            datum_altitude_m,
            ..Self::default()
        };
        if !transform.is_valid() {
            return Err(NavSatError::InvalidConfig);
        }
        Ok(transform)
    }

    /// Whether the datum is finite and within valid ranges.
    pub fn is_valid(&self) -> bool {
        self.datum_latitude_rad.is_finite()
            && self.datum_latitude_rad.abs() <= std::f64::consts::FRAC_PI_2
            && self.datum_longitude_rad.is_finite()
            && self.datum_longitude_rad.abs() <= std::f64::consts::PI
            && self.datum_altitude_m.is_finite()
            && self.datum_yaw_rad.is_finite()
            && self.earth_radius_m.is_finite()
            && self.earth_radius_m > 0.0
    }

    /// Projects latitude/longitude/altitude (radians, radians, meters) to map
    /// `(east, north, up)` meters.
    pub fn datum_to_map(
        &self,
        latitude_rad: f64,
        longitude_rad: f64,
        altitude_m: f64,
    ) -> Result<Vec3, NavSatError> {
        if !latitude_rad.is_finite() || !longitude_rad.is_finite() || !altitude_m.is_finite() {
            return Err(NavSatError::NonFiniteInput);
        }
        let east = self.earth_radius_m
            * (longitude_rad - self.datum_longitude_rad)
            * self.datum_latitude_rad.cos();
        let north = self.earth_radius_m * (latitude_rad - self.datum_latitude_rad);
        let up = altitude_m - self.datum_altitude_m;
        let (sin, cos) = self.datum_yaw_rad.sin_cos();
        Ok(Vec3::new(
            cos * east - sin * north,
            sin * east + cos * north,
            up,
        ))
    }

    /// Inverts [`NavSatTransform::datum_to_map`].
    pub fn map_to_datum(&self, point_m: Vec3) -> Result<(f64, f64, f64), NavSatError> {
        if !point_m.is_finite() {
            return Err(NavSatError::NonFiniteInput);
        }
        let (sin, cos) = self.datum_yaw_rad.sin_cos();
        let east = cos * point_m.x + sin * point_m.y;
        let north = -sin * point_m.x + cos * point_m.y;
        let latitude_rad = self.datum_latitude_rad + north / self.earth_radius_m;
        let longitude_rad = self.datum_longitude_rad
            + east / (self.earth_radius_m * self.datum_latitude_rad.cos()).max(f64::MIN_POSITIVE);
        let altitude_m = self.datum_altitude_m + point_m.z;
        Ok((latitude_rad, longitude_rad, altitude_m))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn datum_maps_to_origin() {
        let transform = NavSatTransform::new(0.6, -2.1, 42.0).unwrap();
        let point = transform.datum_to_map(0.6, -2.1, 42.0).unwrap();
        assert_relative_eq!(point.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(point.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(point.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn known_offset_is_east_and_north() {
        let transform = NavSatTransform::new(0.0, 0.0, 0.0).unwrap();
        // 1e-5 rad latitude ~ 63.7 m north; 1e-5 rad longitude at the equator
        // ~ 63.7 m east.
        let point = transform.datum_to_map(1.0e-5, 1.0e-5, 3.0).unwrap();
        assert_relative_eq!(point.y, EARTH_RADIUS_M * 1.0e-5, epsilon = 1e-6);
        assert_relative_eq!(point.x, EARTH_RADIUS_M * 1.0e-5, epsilon = 1e-6);
        assert_relative_eq!(point.z, 3.0, epsilon = 1e-9);
    }

    #[test]
    fn round_trips_through_the_datum() {
        let mut transform = NavSatTransform::new(0.5, 2.0, 100.0).unwrap();
        transform.datum_yaw_rad = 0.7;
        let (lat, lon, alt) = (0.5001, 2.0002, 105.0);
        let map = transform.datum_to_map(lat, lon, alt).unwrap();
        let (back_lat, back_lon, back_alt) = transform.map_to_datum(map).unwrap();
        assert_relative_eq!(back_lat, lat, epsilon = 1e-12);
        assert_relative_eq!(back_lon, lon, epsilon = 1e-12);
        assert_relative_eq!(back_alt, alt, epsilon = 1e-12);
    }

    #[test]
    fn rejects_invalid_config_and_input() {
        assert_eq!(
            NavSatTransform::new(2.0, 0.0, 0.0),
            Err(NavSatError::InvalidConfig)
        );
        let transform = NavSatTransform::default();
        assert_eq!(
            transform.datum_to_map(f64::NAN, 0.0, 0.0),
            Err(NavSatError::NonFiniteInput)
        );
    }
}
