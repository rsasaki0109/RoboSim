//! Deterministic physics state hashing helpers.

use crate::components::{JointState, RigidBody, RigidBodyType};
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

/// Hashes articulated physics state with a portable, versioned byte contract.
///
/// Version 2 includes every non-fixed rigid body's translation, orientation,
/// linear velocity, and angular velocity, plus every revolute or prismatic
/// [`JointState`] coordinate and velocity. Entities are ordered by ECS index,
/// component kinds carry explicit tags, all SI values are quantized to
/// `1e-6`, and the resulting little-endian bytes use FNV-1a 64-bit. This makes
/// the digest suitable for exact replay comparison across machines running the
/// same RNE build. It is not a cross-backend equivalence claim.
pub fn hash_physics_state_v2(world: &World) -> u64 {
    let mut entries: BTreeMap<(u32, u8), Vec<i64>> = BTreeMap::new();

    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if let (Some(rigid_body), Some(transform)) = (
            world.get::<RigidBody>(entity),
            world.get::<Transform3>(entity),
        ) {
            if rigid_body.body_type != RigidBodyType::Fixed {
                entries.insert(
                    (entity.index(), 1),
                    vec![
                        quantize(transform.translation.x),
                        quantize(transform.translation.y),
                        quantize(transform.translation.z),
                        quantize(transform.rotation.x),
                        quantize(transform.rotation.y),
                        quantize(transform.rotation.z),
                        quantize(transform.rotation.w),
                        quantize(rigid_body.linear_velocity_m_s.x),
                        quantize(rigid_body.linear_velocity_m_s.y),
                        quantize(rigid_body.linear_velocity_m_s.z),
                        quantize(rigid_body.angular_velocity_rad_s.x),
                        quantize(rigid_body.angular_velocity_rad_s.y),
                        quantize(rigid_body.angular_velocity_rad_s.z),
                    ],
                );
            }
        }
        match world.get::<JointState>(entity).copied() {
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => {
                entries.insert(
                    (entity.index(), 2),
                    vec![quantize(position_rad), quantize(velocity_rad_s)],
                );
            }
            Some(JointState::Prismatic {
                position_m,
                velocity_m_s,
            }) => {
                entries.insert(
                    (entity.index(), 3),
                    vec![quantize(position_m), quantize(velocity_m_s)],
                );
            }
            Some(JointState::Fixed) | None => {}
        }
    }

    let mut hash = FNV1A_OFFSET_BASIS;
    for ((entity_index, component_tag), values) in entries {
        fnv1a_update(&mut hash, &entity_index.to_le_bytes());
        fnv1a_update(&mut hash, &[component_tag]);
        fnv1a_update(&mut hash, &(values.len() as u32).to_le_bytes());
        for value in values {
            fnv1a_update(&mut hash, &value.to_le_bytes());
        }
    }
    hash
}

const FNV1A_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV1A_PRIME: u64 = 0x100000001b3;

fn fnv1a_update(hash: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *hash ^= u64::from(*byte);
        *hash = hash.wrapping_mul(FNV1A_PRIME);
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::{Quat, Vec3};

    #[test]
    fn v2_changes_when_revolute_joint_state_changes() {
        let mut world = World::new();
        let entity = world
            .spawn(JointState::Revolute {
                position_rad: 0.1,
                velocity_rad_s: 0.2,
            })
            .id();
        let before = hash_physics_state_v2(&world);
        world.entity_mut(entity).insert(JointState::Revolute {
            position_rad: 0.11,
            velocity_rad_s: 0.2,
        });
        assert_ne!(hash_physics_state_v2(&world), before);
    }

    #[test]
    fn v2_changes_when_rigid_body_orientation_or_velocity_changes() {
        let mut world = World::new();
        let entity = world
            .spawn((RigidBody::default(), Transform3::IDENTITY))
            .id();
        let initial = hash_physics_state_v2(&world);
        world.entity_mut(entity).insert(Transform3 {
            rotation: Quat::from_rotation_z(0.1),
            ..Transform3::IDENTITY
        });
        let rotated = hash_physics_state_v2(&world);
        assert_ne!(rotated, initial);
        world.entity_mut(entity).insert(RigidBody {
            linear_velocity_m_s: Vec3::new(0.1, 0.0, 0.0),
            ..RigidBody::default()
        });
        assert_ne!(hash_physics_state_v2(&world), rotated);
    }

    #[test]
    fn v2_ignores_sub_quantization_joint_noise() {
        let mut world = World::new();
        let entity = world
            .spawn(JointState::Prismatic {
                position_m: 0.1,
                velocity_m_s: 0.2,
            })
            .id();
        let initial = hash_physics_state_v2(&world);
        world.entity_mut(entity).insert(JointState::Prismatic {
            position_m: 0.1 + 0.4e-6,
            velocity_m_s: 0.2,
        });
        assert_eq!(hash_physics_state_v2(&world), initial);
    }
}
