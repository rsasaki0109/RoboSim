//! Shared physical environment inputs to sensor sampling.

use bevy_ecs::prelude::Resource;
use rne_math::Vec3;

/// World-frame gravity used to convert kinematic acceleration into specific force.
///
/// Set this to the same gravity as the physics world before sampling IMUs.
/// Absence preserves the legacy [`crate::GRAVITY_M_S2`] default. This is simulation
/// environment truth, not a controller-visible observation or calibration estimate.
#[derive(Clone, Copy, Debug, Resource)]
pub struct SensorGravity {
    gravity_m_s2: Vec3,
}

impl SensorGravity {
    /// Constructs a finite gravity vector; zero gravity is permitted.
    pub fn new(gravity_m_s2: Vec3) -> Option<Self> {
        gravity_m_s2.is_finite().then_some(Self { gravity_m_s2 })
    }

    /// Returns world-frame gravity in meters per second squared.
    pub fn gravity_m_s2(self) -> Vec3 {
        self.gravity_m_s2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gravity_rejects_non_finite_values_but_allows_free_fall() {
        assert!(SensorGravity::new(Vec3::new(f64::NAN, 0.0, 0.0)).is_none());
        assert!(SensorGravity::new(Vec3::new(0.0, f64::INFINITY, 0.0)).is_none());
        assert_eq!(
            SensorGravity::new(Vec3::ZERO).unwrap().gravity_m_s2(),
            Vec3::ZERO
        );
    }
}
