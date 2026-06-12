//! Entity spawning helpers.

use crate::{EntityUuid, Name};
use bevy_ecs::prelude::{Entity, World};

/// Spawns an entity with a stable UUID and name.
pub fn spawn_named(world: &mut World, name: impl Into<String>) -> Entity {
    world.spawn((EntityUuid::new_v4(), Name::new(name))).id()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Children, Parent};

    #[test]
    fn spawn_entity_with_name() {
        let mut world = World::new();
        let entity = spawn_named(&mut world, "root");
        let name = world.get::<Name>(entity).expect("name component");
        assert_eq!(name.0, "root");
    }

    #[test]
    fn parent_children_relationship() {
        let mut world = World::new();
        let parent = spawn_named(&mut world, "parent");
        let child = spawn_named(&mut world, "child");

        world
            .entity_mut(parent)
            .insert(Children(Default::default()));
        world.entity_mut(child).insert(Parent(parent));
        world
            .entity_mut(parent)
            .get_mut::<Children>()
            .expect("children")
            .0
            .push(child);

        let children = world.get::<Children>(parent).expect("children");
        assert_eq!(children.0.as_slice(), &[child]);
        assert_eq!(world.get::<Parent>(child).expect("parent").0, parent);
    }
}
