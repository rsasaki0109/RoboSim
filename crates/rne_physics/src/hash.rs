//! Deterministic physics state hashing helpers.

use crate::components::{RigidBody, RigidBodyType};
use rne_ecs::Entity;
use rne_ecs::World;
use rne_world::Transform3;
use std::collections::BTreeMap;
use std::hash::{Hash, Hasher};

/// Hashes dynamic rigid body transforms for determinism tests.
///
/// Entities are sorted by stable index before hashing to ensure stable ordering.
pub fn hash_physics_state(world: &World) -> u64 {
    let mut entries: BTreeMap<u32, (f64, f64, f64)> = BTreeMap::new();

    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        let Some(rigid_body) = world.get::<RigidBody>(entity) else {
            continue;
        };
        if rigid_body.body_type == RigidBodyType::Fixed {
            continue;
        }
        let Some(transform) = world.get::<Transform3>(entity) else {
            continue;
        };
        entries.insert(
            entity.index(),
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
        );
    }

    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for (entity_index, (x, y, z)) in entries {
        entity_index.hash(&mut hasher);
        quantize(x).hash(&mut hasher);
        quantize(y).hash(&mut hasher);
        quantize(z).hash(&mut hasher);
    }

    hasher.finish()
}

fn quantize(value: f64) -> i64 {
    (value * 1_000_000.0).round() as i64
}

/// Returns the translation of an entity for test assertions.
pub fn entity_translation(world: &World, entity: Entity) -> Option<rne_math::Vec3> {
    world
        .get::<Transform3>(entity)
        .map(|transform| transform.translation)
}
