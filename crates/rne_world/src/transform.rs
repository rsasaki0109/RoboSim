//! World transform helpers.

use bevy_ecs::prelude::World;
use rne_ecs::{Entity, Parent};

use crate::Transform3;

/// Returns the composed world transform for an entity in a parent/child hierarchy.
pub fn world_transform_of(world: &World, entity: Entity) -> Transform3 {
    let local = world.get::<Transform3>(entity).copied().unwrap_or_default();
    let Some(parent) = world.get::<Parent>(entity) else {
        return local;
    };
    world_transform_of(world, parent.0).mul_transform(&local)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_ecs::world::World as EcsWorld;
    use rne_ecs::{spawn_named, Parent};
    use rne_math::{Quat, Vec3};

    #[test]
    fn world_transform_composes_parent_chain() {
        let mut world = EcsWorld::new();
        let parent = spawn_named(&mut world, "parent");
        let child = spawn_named(&mut world, "child");
        world.entity_mut(parent).insert(Transform3::from_translation_rotation(
            Vec3::new(1.0, 0.0, 0.0),
            Quat::IDENTITY,
        ));
        world.entity_mut(child).insert((
            Parent(parent),
            Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
        ));

        let global = world_transform_of(&world, child);
        assert!((global.translation.x - 1.0).abs() < 1e-10);
        assert!((global.translation.y - 2.0).abs() < 1e-10);
    }
}
