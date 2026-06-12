//! World systems.

use crate::{GlobalTransform3, Transform3};
use bevy_ecs::prelude::{Entity, Query, Without};
use rne_ecs::{Children, Parent};
use rne_math::Mat4;
use std::collections::{HashMap, HashSet, VecDeque};

/// Propagates local transforms through the parent/child hierarchy.
pub fn propagate_transforms(
    roots: Query<(Entity, &Transform3), Without<Parent>>,
    children: Query<&Children>,
    mut transforms: Query<(&Transform3, &mut GlobalTransform3)>,
    parents: Query<&Parent>,
) {
    let mut global_by_entity: HashMap<Entity, Mat4> = HashMap::new();

    for (entity, local) in roots.iter() {
        let global = local.to_matrix();
        if let Ok((_, mut global_transform)) = transforms.get_mut(entity) {
            global_transform.matrix = global;
        }
        global_by_entity.insert(entity, global);
    }

    let mut queue: VecDeque<Entity> = global_by_entity.keys().copied().collect();
    let mut visited = HashSet::new();

    while let Some(parent) = queue.pop_front() {
        if !visited.insert(parent) {
            continue;
        }

        let Some(parent_global) = global_by_entity.get(&parent).copied() else {
            continue;
        };

        let Ok(parent_children) = children.get(parent) else {
            continue;
        };

        for child in parent_children.0.iter().copied() {
            let Ok((local, mut global)) = transforms.get_mut(child) else {
                continue;
            };
            let child_global = parent_global * local.to_matrix();
            global.matrix = child_global;
            global_by_entity.insert(child, child_global);
            queue.push_back(child);

            if let Ok(child_parent) = parents.get(child) {
                debug_assert_eq!(child_parent.0, parent);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_ecs::world::World;
    use rne_ecs::{spawn_named, Children, Parent};
    use rne_math::{Quat, Vec3};

    #[test]
    fn parent_transform_propagation() {
        let mut world = World::new();

        let parent = spawn_named(&mut world, "parent");
        let child = spawn_named(&mut world, "child");

        world.entity_mut(parent).insert((
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
            GlobalTransform3::default(),
            Children(Default::default()),
        ));
        world.entity_mut(child).insert((
            Parent(parent),
            Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
            GlobalTransform3::default(),
        ));
        world
            .entity_mut(parent)
            .get_mut::<Children>()
            .unwrap()
            .0
            .push(child);

        let mut schedule = bevy_ecs::schedule::Schedule::default();
        schedule.add_systems(propagate_transforms);
        schedule.run(&mut world);

        let child_global = world.get::<GlobalTransform3>(child).unwrap();
        let point = child_global.matrix.transform_point3(Vec3::ZERO);
        assert!((point.x - 1.0).abs() < 1e-10);
        assert!((point.y - 2.0).abs() < 1e-10);
    }
}
