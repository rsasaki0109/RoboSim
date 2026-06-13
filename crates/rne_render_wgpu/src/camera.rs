//! Orbit camera helpers for headless and interactive rendering.

use rne_math::{Quat, Transform3, Vec3};

/// Orbit camera around a world-space focus point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraOrbit {
    /// Horizontal orbit angle in radians.
    pub yaw_rad: f64,
    /// Vertical orbit angle in radians.
    pub pitch_rad: f64,
    /// Distance from the focus point in meters.
    pub distance_m: f64,
    /// Point the camera looks at.
    pub focus: Vec3,
}

impl Default for CameraOrbit {
    fn default() -> Self {
        Self {
            yaw_rad: 0.0,
            pitch_rad: 0.55,
            distance_m: 4.0,
            focus: Vec3::ZERO,
        }
    }
}

impl CameraOrbit {
    /// Builds a camera world transform looking at the focus point.
    pub fn camera_transform(&self) -> Transform3 {
        let pitch = self.pitch_rad.clamp(0.15, 1.45);
        let yaw = self.yaw_rad;
        let horizontal = self.distance_m * pitch.sin();
        let eye = Vec3::new(
            self.focus.x + horizontal * yaw.sin(),
            self.focus.y + self.distance_m * pitch.cos(),
            self.focus.z + horizontal * yaw.cos(),
        );
        let forward = (self.focus - eye).normalize_or_zero();
        let rotation = if forward.length_squared() > f64::EPSILON {
            Quat::from_rotation_arc(-Vec3::Z, forward)
        } else {
            Quat::IDENTITY
        };

        Transform3::from_translation_rotation(eye, rotation)
    }
}
