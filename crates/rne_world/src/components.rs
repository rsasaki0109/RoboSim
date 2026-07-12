//! World-level ECS components.

use crate::WorldRandom;
use bevy_ecs::prelude::Component;
use rne_ecs::EntityUuid;
use rne_math::{Mat4, Quat, Transform3 as MathTransform3, Vec3};
use serde::{Deserialize, Serialize};

/// Root world configuration entity.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldEntity {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// Deterministic random seed.
    pub seed: u64,
    /// Simulation time scale multiplier.
    pub time_scale: f64,
}

impl Default for WorldEntity {
    fn default() -> Self {
        Self {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            seed: 0,
            time_scale: 1.0,
        }
    }
}

/// Gravity resource attached to a world entity.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Gravity {
    /// Gravity vector in meters per second squared.
    pub vector_m_s2: Vec3,
}

/// Named semantic location in a world used by tasks, policies, and episodes.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TaskMarker {
    /// Application-defined marker kind, such as `inspection` or `drop_zone`.
    pub kind: String,
    /// Success or interaction radius around the marker in meters.
    pub radius_m: f64,
}

impl Default for Gravity {
    fn default() -> Self {
        Self {
            vector_m_s2: Vec3::new(0.0, -9.81, 0.0),
        }
    }
}

/// Local spatial transform component.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    /// Translation in meters.
    pub translation: Vec3,
    /// Rotation as a unit quaternion.
    pub rotation: Quat,
    /// Non-uniform scale.
    pub scale: Vec3,
}

impl Default for Transform3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform3 {
    /// Identity transform.
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3::ONE,
    };

    /// Creates a transform from translation and rotation.
    pub fn from_translation_rotation(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
            scale: Vec3::ONE,
        }
    }

    /// Converts the transform to a 4x4 matrix.
    pub fn to_matrix(&self) -> Mat4 {
        MathTransform3 {
            translation: self.translation,
            rotation: self.rotation,
            scale: self.scale,
        }
        .to_matrix()
    }

    /// Composes this transform with a local child transform.
    pub fn mul_transform(&self, local: &Self) -> Self {
        let parent = MathTransform3 {
            translation: self.translation,
            rotation: self.rotation,
            scale: self.scale,
        };
        let child = MathTransform3 {
            translation: local.translation,
            rotation: local.rotation,
            scale: local.scale,
        };
        let composed = parent.mul_transform(&child);
        Self {
            translation: composed.translation,
            rotation: composed.rotation,
            scale: composed.scale,
        }
    }
}

/// Cached global transform matrix.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GlobalTransform3 {
    /// World-space transform matrix.
    pub matrix: Mat4,
}

/// Spawns a world root entity with default configuration.
pub fn spawn_world(world: &mut bevy_ecs::world::World) -> bevy_ecs::entity::Entity {
    if !world.contains_resource::<WorldRandom>() {
        world.insert_resource(WorldRandom::default());
    }
    world
        .spawn((
            EntityUuid::default(),
            WorldEntity::default(),
            Gravity::default(),
        ))
        .id()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_ecs::world::World;

    #[test]
    fn spawn_world_inserts_default_random_when_missing() {
        let mut world = World::new();

        spawn_world(&mut world);

        assert_eq!(world.resource::<WorldRandom>().seed(), 0);
    }

    #[test]
    fn spawn_world_preserves_existing_random_state() {
        let mut world = World::new();
        world.insert_resource(WorldRandom::new(7));
        let consumed = world.resource_mut::<WorldRandom>().next_u64();

        spawn_world(&mut world);

        let next_after_spawn = world.resource_mut::<WorldRandom>().next_u64();
        let mut expected = WorldRandom::new(7);
        assert_eq!(expected.next_u64(), consumed);
        assert_eq!(expected.next_u64(), next_after_spawn);
    }
}
