//! Explicit unit newtypes to prevent mixing physical quantities.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

macro_rules! unit_newtype {
    ($name:ident, $suffix:literal, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(f64);

        impl $name {
            /// Zero value.
            pub const ZERO: Self = Self(0.0);

            /// Creates a new value from the raw quantity.
            pub const fn new(value: f64) -> Self {
                Self(value)
            }

            /// Returns the raw quantity.
            pub const fn value(self) -> f64 {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}{}", self.0, $suffix)
            }
        }

        impl Add for $name {
            type Output = Self;

            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }

        impl AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl Sub for $name {
            type Output = Self;

            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }

        impl SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl Mul<f64> for $name {
            type Output = Self;

            fn mul(self, rhs: f64) -> Self::Output {
                Self(self.0 * rhs)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;

            fn div(self, rhs: f64) -> Self::Output {
                Self(self.0 / rhs)
            }
        }
    };
}

unit_newtype!(Meters, " m", "Length in meters.");
unit_newtype!(Radians, " rad", "Angle in radians.");
unit_newtype!(Seconds, " s", "Time in seconds.");
unit_newtype!(Hertz, " Hz", "Frequency in hertz.");

impl Mul<Seconds> for Hertz {
    type Output = f64;

    fn mul(self, rhs: Seconds) -> Self::Output {
        self.0 * rhs.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units_compile_and_display() {
        let length = Meters::new(1.5);
        let angle = Radians::new(std::f64::consts::FRAC_PI_2);
        let duration = Seconds::new(0.5);
        let rate = Hertz::new(60.0);

        assert_eq!(length.to_string(), "1.5 m");
        assert_eq!(angle.to_string(), "1.5707963267948966 rad");
        assert_eq!(duration.to_string(), "0.5 s");
        assert_eq!(rate.to_string(), "60 Hz");
        assert!((rate * duration - 30.0).abs() < f64::EPSILON);
    }
}
