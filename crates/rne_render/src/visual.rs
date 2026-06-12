//! Scene visual primitives attached to entities.

use bevy_ecs::prelude::Component;
use rne_math::{Quat, Vec3};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// Visual primitive shape.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum VisualShape {
    /// Axis-aligned box with full extents in meters.
    Box {
        /// Full size in meters.
        size_m: Vec3,
    },
    /// Sphere with radius in meters.
    Sphere {
        /// Radius in meters.
        radius_m: f64,
    },
    /// Cylinder aligned with the local Z axis.
    Cylinder {
        /// Radius in meters.
        radius_m: f64,
        /// Full length in meters.
        length_m: f64,
    },
    /// External mesh asset path.
    Mesh {
        /// Mesh file path from the URDF package.
        path: String,
        /// Non-uniform scale.
        scale: Vec3,
    },
}

/// Visual description attached to a link or prop entity.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Visual {
    /// Primitive or mesh shape.
    pub shape: VisualShape,
    /// Pose relative to the entity transform.
    pub local_offset: Transform3,
    /// RGBA color in linear space.
    pub color_rgba: [f32; 4],
}

impl Visual {
    /// Creates a solid-color visual with identity local offset.
    pub fn new(shape: VisualShape, color_rgba: [f32; 4]) -> Self {
        Self {
            shape,
            local_offset: Transform3::IDENTITY,
            color_rgba,
        }
    }

    /// Returns a copy with the given local offset.
    pub fn with_local_offset(mut self, translation: Vec3, rotation: Quat) -> Self {
        self.local_offset = Transform3::from_translation_rotation(translation, rotation);
        self
    }
}
