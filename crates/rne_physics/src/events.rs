//! Physics events and queries.

use rne_ecs::Entity;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Contact event between two colliders.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactEvent {
    /// First colliding entity.
    pub entity_a: Entity,
    /// Second colliding entity.
    pub entity_b: Entity,
    /// Contact normal pointing from A to B.
    pub normal: Vec3,
    /// Accumulated normal-impulse magnitude for this contact pair over the last
    /// physics step, summed across every manifold and contact point between the
    /// two colliders (units: N·s, i.e. force integrated over the fixed step —
    /// Rapier applies impulses rather than forces internally). Zero when the
    /// backend does not populate it (e.g. a contact pair with no active solver
    /// contact this step). Useful for distinguishing a light graze from a
    /// load-bearing contact (see friction-based grasping) without needing raw
    /// per-point solver data.
    pub impulse: f32,
}

/// Raycast query definition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaycastQuery {
    /// Ray origin in meters.
    pub origin_m: Vec3,
    /// Unit ray direction.
    pub direction: Vec3,
    /// Maximum ray length in meters.
    pub max_distance_m: f64,
}

impl RaycastQuery {
    /// Creates a downward gravity-aligned ray used for ground checks.
    pub fn downward(origin_m: Vec3, max_distance_m: f64) -> Self {
        Self {
            origin_m,
            direction: Vec3::NEG_Y,
            max_distance_m,
        }
    }
}

/// Result of a successful raycast hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RaycastHit {
    /// Hit entity.
    pub entity: Entity,
    /// Hit point in meters.
    pub point_m: Vec3,
    /// Surface normal at the hit point.
    pub normal: Vec3,
    /// Distance from the ray origin in meters.
    pub distance_m: f64,
}
