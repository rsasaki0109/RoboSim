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
        let rotation = (Quat::from_rotation_y(yaw)
            * Quat::from_rotation_x(pitch - std::f64::consts::FRAC_PI_2))
        .normalize();

        Transform3::from_translation_rotation(eye, rotation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orbit_camera_looks_at_focus_without_roll() {
        let orbit = CameraOrbit {
            focus: Vec3::new(3.0, 2.0, -4.0),
            yaw_rad: 1.1,
            pitch_rad: 1.0,
            distance_m: 12.0,
        };
        let transform = orbit.camera_transform();
        let expected_forward = (orbit.focus - transform.translation).normalize();
        let actual_forward = transform.rotation * -Vec3::Z;
        assert!((expected_forward - actual_forward).length() < 1.0e-12);
        assert!((transform.rotation * Vec3::X).y.abs() < 1.0e-12);
        assert!((transform.rotation * Vec3::Y).dot(Vec3::Y) > 0.0);
    }
}
