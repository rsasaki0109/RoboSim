//! Navigation ECS components.

use bevy_ecs::prelude::Component;
use rne_math::Vec3;

/// A planar navigation goal expressed in the map frame.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct NavGoal {
    /// Target position in meters; `z` is ignored by planar planners.
    pub target_m: Vec3,
    /// Optional desired final yaw in radians.
    pub yaw_rad: Option<f64>,
    /// Position tolerance in meters for goal completion.
    pub tolerance_m: f64,
}

impl NavGoal {
    /// Creates a position-only goal with a default tolerance.
    pub fn position(target_m: Vec3) -> Self {
        Self {
            target_m,
            yaw_rad: None,
            tolerance_m: 0.1,
        }
    }

    /// Creates a goal with a desired final yaw.
    pub fn with_yaw(target_m: Vec3, yaw_rad: f64, tolerance_m: f64) -> Self {
        Self {
            target_m,
            yaw_rad: Some(yaw_rad),
            tolerance_m,
        }
    }
}
