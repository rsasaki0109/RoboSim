//! Canonical, backend-neutral snapshots of observable physics state.

use crate::{ContactEvent, RigidBody, RigidBodyType};
use rne_ecs::{Name, World};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Schema version for [`PhysicsSnapshot`].
pub const PHYSICS_SNAPSHOT_SCHEMA_VERSION: u16 = 1;

const FNV1A64_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV1A64_PRIME: u64 = 0x00000100000001b3;

/// Observable rigid-body state at a completed simulation step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsBodySnapshot {
    /// ECS entity index used as the stable identity within this world instance.
    pub entity_index: u32,
    /// Optional semantic entity name.
    pub name: Option<String>,
    /// Motion type.
    pub body_type: RigidBodyType,
    /// Configured mass in kilograms.
    pub mass_kg: f64,
    /// World translation in metres.
    pub translation_m: [f64; 3],
    /// Canonical unit-quaternion components in x/y/z/w order.
    pub rotation_xyzw: [f64; 4],
    /// World linear velocity in metres per second.
    pub linear_velocity_m_s: [f64; 3],
    /// World angular velocity in radians per second.
    pub angular_velocity_rad_s: [f64; 3],
}

/// Canonical contact-pair evidence from the last completed physics step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsContactSnapshot {
    /// Lower canonical ECS entity index.
    pub entity_a_index: u32,
    /// Higher canonical ECS entity index.
    pub entity_b_index: u32,
    /// Contact normal oriented from canonical entity A to B.
    pub normal_a_to_b: [f64; 3],
    /// Accumulated normal impulse in newton-seconds.
    pub normal_impulse_n_s: f64,
}

/// Versioned physics state used for deterministic evidence and conformance reports.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsSnapshot {
    /// Snapshot schema version.
    pub schema_version: u16,
    /// Completed fixed-step index.
    pub step: u64,
    /// Simulation timestamp in nanosecond ticks.
    pub sim_time_ticks: u64,
    /// Rigid bodies ordered by entity index.
    pub bodies: Vec<PhysicsBodySnapshot>,
    /// Canonical contacts in deterministic pair/value order.
    pub contacts: Vec<PhysicsContactSnapshot>,
}

impl PhysicsSnapshot {
    /// Computes a frozen FNV-1a 64-bit digest over canonical little-endian fields.
    ///
    /// This digest is suitable for exact repeat executions of one deterministic
    /// backend. It does not imply bit equivalence between unlike solvers or
    /// platforms for contact-rich state.
    pub fn stable_hash(&self) -> u64 {
        let mut hash = FNV1A64_OFFSET_BASIS;
        hash_bytes(&mut hash, &self.schema_version.to_le_bytes());
        hash_bytes(&mut hash, &self.step.to_le_bytes());
        hash_bytes(&mut hash, &self.sim_time_ticks.to_le_bytes());
        hash_bytes(&mut hash, &(self.bodies.len() as u64).to_le_bytes());
        for body in &self.bodies {
            hash_bytes(&mut hash, &body.entity_index.to_le_bytes());
            match &body.name {
                Some(name) => {
                    hash_byte(&mut hash, 1);
                    hash_bytes(&mut hash, &(name.len() as u64).to_le_bytes());
                    hash_bytes(&mut hash, name.as_bytes());
                }
                None => hash_byte(&mut hash, 0),
            }
            hash_byte(&mut hash, body_type_code(body.body_type));
            hash_f64(&mut hash, body.mass_kg);
            hash_f64_slice(&mut hash, &body.translation_m);
            hash_f64_slice(&mut hash, &body.rotation_xyzw);
            hash_f64_slice(&mut hash, &body.linear_velocity_m_s);
            hash_f64_slice(&mut hash, &body.angular_velocity_rad_s);
        }
        hash_bytes(&mut hash, &(self.contacts.len() as u64).to_le_bytes());
        for contact in &self.contacts {
            hash_bytes(&mut hash, &contact.entity_a_index.to_le_bytes());
            hash_bytes(&mut hash, &contact.entity_b_index.to_le_bytes());
            hash_f64_slice(&mut hash, &contact.normal_a_to_b);
            hash_f64(&mut hash, contact.normal_impulse_n_s);
        }
        hash
    }
}

/// Snapshot capture failure.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum PhysicsSnapshotError {
    /// One body field was NaN or infinite.
    #[error("entity {entity_index} has non-finite physics field {field}")]
    NonFiniteBody {
        /// ECS entity index.
        entity_index: u32,
        /// Invalid field name.
        field: &'static str,
    },
    /// One contact field was NaN or infinite.
    #[error("contact {entity_a_index}<->{entity_b_index} has non-finite physics field {field}")]
    NonFiniteContact {
        /// First ECS entity index as reported by the backend.
        entity_a_index: u32,
        /// Second ECS entity index as reported by the backend.
        entity_b_index: u32,
        /// Invalid field name.
        field: &'static str,
    },
}

/// Captures a canonical snapshot from ECS state and last-step contacts.
pub fn capture_physics_snapshot(
    world: &World,
    contacts: &[ContactEvent],
    step: u64,
    sim_time_ticks: u64,
) -> Result<PhysicsSnapshot, PhysicsSnapshotError> {
    let mut bodies = Vec::new();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        let Some(body) = world.get::<RigidBody>(entity) else {
            continue;
        };
        let Some(transform) = world.get::<Transform3>(entity) else {
            continue;
        };
        validate_body(entity.index(), body, transform)?;
        let rotation_xyzw = canonical_quaternion([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]);
        bodies.push(PhysicsBodySnapshot {
            entity_index: entity.index(),
            name: world.get::<Name>(entity).map(|name| name.0.clone()),
            body_type: body.body_type,
            mass_kg: canonical_zero(body.mass_kg),
            translation_m: vec3_array(transform.translation),
            rotation_xyzw,
            linear_velocity_m_s: vec3_array(body.linear_velocity_m_s),
            angular_velocity_rad_s: vec3_array(body.angular_velocity_rad_s),
        });
    }
    bodies.sort_unstable_by_key(|body| body.entity_index);

    let mut canonical_contacts = contacts
        .iter()
        .map(canonical_contact)
        .collect::<Result<Vec<_>, _>>()?;
    canonical_contacts.sort_by(|left, right| {
        left.entity_a_index
            .cmp(&right.entity_a_index)
            .then_with(|| left.entity_b_index.cmp(&right.entity_b_index))
            .then_with(|| compare_f64_arrays(&left.normal_a_to_b, &right.normal_a_to_b))
            .then_with(|| left.normal_impulse_n_s.total_cmp(&right.normal_impulse_n_s))
    });

    Ok(PhysicsSnapshot {
        schema_version: PHYSICS_SNAPSHOT_SCHEMA_VERSION,
        step,
        sim_time_ticks,
        bodies,
        contacts: canonical_contacts,
    })
}

fn validate_body(
    entity_index: u32,
    body: &RigidBody,
    transform: &Transform3,
) -> Result<(), PhysicsSnapshotError> {
    let fields = [
        ("mass_kg", body.mass_kg),
        ("translation_m.x", transform.translation.x),
        ("translation_m.y", transform.translation.y),
        ("translation_m.z", transform.translation.z),
        ("rotation.x", transform.rotation.x),
        ("rotation.y", transform.rotation.y),
        ("rotation.z", transform.rotation.z),
        ("rotation.w", transform.rotation.w),
        ("linear_velocity_m_s.x", body.linear_velocity_m_s.x),
        ("linear_velocity_m_s.y", body.linear_velocity_m_s.y),
        ("linear_velocity_m_s.z", body.linear_velocity_m_s.z),
        ("angular_velocity_rad_s.x", body.angular_velocity_rad_s.x),
        ("angular_velocity_rad_s.y", body.angular_velocity_rad_s.y),
        ("angular_velocity_rad_s.z", body.angular_velocity_rad_s.z),
    ];
    if let Some((field, _)) = fields.into_iter().find(|(_, value)| !value.is_finite()) {
        return Err(PhysicsSnapshotError::NonFiniteBody {
            entity_index,
            field,
        });
    }
    Ok(())
}

fn canonical_contact(
    contact: &ContactEvent,
) -> Result<PhysicsContactSnapshot, PhysicsSnapshotError> {
    let a = contact.entity_a.index();
    let b = contact.entity_b.index();
    let values = [
        ("normal.x", contact.normal.x),
        ("normal.y", contact.normal.y),
        ("normal.z", contact.normal.z),
        ("normal_impulse_n_s", contact.impulse as f64),
    ];
    if let Some((field, _)) = values.into_iter().find(|(_, value)| !value.is_finite()) {
        return Err(PhysicsSnapshotError::NonFiniteContact {
            entity_a_index: a,
            entity_b_index: b,
            field,
        });
    }
    let (entity_a_index, entity_b_index, normal) = if a <= b {
        (a, b, contact.normal)
    } else {
        (b, a, -contact.normal)
    };
    Ok(PhysicsContactSnapshot {
        entity_a_index,
        entity_b_index,
        normal_a_to_b: vec3_array(normal),
        normal_impulse_n_s: canonical_zero(contact.impulse as f64),
    })
}

fn canonical_quaternion(mut value: [f64; 4]) -> [f64; 4] {
    let negate = value[3] < 0.0
        || (value[3] == 0.0
            && (value[2] < 0.0
                || (value[2] == 0.0 && (value[1] < 0.0 || (value[1] == 0.0 && value[0] < 0.0)))));
    if negate {
        for component in &mut value {
            *component = -*component;
        }
    }
    for component in &mut value {
        *component = canonical_zero(*component);
    }
    value
}

fn vec3_array(value: rne_math::Vec3) -> [f64; 3] {
    [
        canonical_zero(value.x),
        canonical_zero(value.y),
        canonical_zero(value.z),
    ]
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

fn compare_f64_arrays<const N: usize>(left: &[f64; N], right: &[f64; N]) -> std::cmp::Ordering {
    for (left, right) in left.iter().zip(right) {
        let ordering = left.total_cmp(right);
        if ordering != std::cmp::Ordering::Equal {
            return ordering;
        }
    }
    std::cmp::Ordering::Equal
}

fn body_type_code(body_type: RigidBodyType) -> u8 {
    match body_type {
        RigidBodyType::Dynamic => 0,
        RigidBodyType::Fixed => 1,
        RigidBodyType::Kinematic => 2,
    }
}

fn hash_f64_slice(hash: &mut u64, values: &[f64]) {
    for value in values {
        hash_f64(hash, *value);
    }
}

fn hash_f64(hash: &mut u64, value: f64) {
    hash_bytes(hash, &canonical_zero(value).to_bits().to_le_bytes());
}

fn hash_bytes(hash: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        hash_byte(hash, *byte);
    }
}

fn hash_byte(hash: &mut u64, byte: u8) {
    *hash ^= u64::from(byte);
    *hash = hash.wrapping_mul(FNV1A64_PRIME);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RigidBody;
    use rne_ecs::{spawn_named, World};
    use rne_math::{Quat, Vec3};
    use rne_world::Transform3;

    fn fixture() -> (World, rne_ecs::Entity, rne_ecs::Entity) {
        let mut world = World::new();
        let first = spawn_named(&mut world, "first");
        let second = spawn_named(&mut world, "second");
        world.entity_mut(first).insert((
            RigidBody {
                mass_kg: 2.0,
                linear_velocity_m_s: Vec3::new(1.0, 2.0, 3.0),
                ..RigidBody::default()
            },
            Transform3::from_translation_rotation(
                Vec3::new(4.0, 5.0, 6.0),
                Quat::from_rotation_y(0.25),
            ),
        ));
        world.entity_mut(second).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Transform3::default(),
        ));
        (world, first, second)
    }

    #[test]
    fn contact_order_and_quaternion_sign_are_canonical() {
        let (mut world, first, second) = fixture();
        let contact = ContactEvent {
            entity_a: second,
            entity_b: first,
            normal: Vec3::Y,
            impulse: 2.5,
        };
        let snapshot = capture_physics_snapshot(&world, &[contact], 7, 42).unwrap();

        world.get_mut::<Transform3>(first).unwrap().rotation =
            -world.get::<Transform3>(first).unwrap().rotation;
        let reversed = ContactEvent {
            entity_a: first,
            entity_b: second,
            normal: -Vec3::Y,
            impulse: 2.5,
        };
        let equivalent = capture_physics_snapshot(&world, &[reversed], 7, 42).unwrap();

        assert_eq!(snapshot, equivalent);
        assert_eq!(snapshot.contacts[0].normal_a_to_b, [0.0, -1.0, 0.0]);
        assert_eq!(snapshot.stable_hash(), equivalent.stable_hash());
    }

    #[test]
    fn stable_hash_has_a_frozen_golden_value() {
        let (world, first, second) = fixture();
        let snapshot = capture_physics_snapshot(
            &world,
            &[ContactEvent {
                entity_a: first,
                entity_b: second,
                normal: Vec3::NEG_Y,
                impulse: 1.25,
            }],
            3,
            50_000_000,
        )
        .unwrap();
        assert_eq!(snapshot.stable_hash(), 5_027_773_066_177_644_859);
    }

    #[test]
    fn non_finite_state_is_rejected() {
        let (mut world, first, _) = fixture();
        world
            .get_mut::<RigidBody>(first)
            .unwrap()
            .linear_velocity_m_s
            .x = f64::NAN;
        assert!(matches!(
            capture_physics_snapshot(&world, &[], 0, 0),
            Err(PhysicsSnapshotError::NonFiniteBody {
                field: "linear_velocity_m_s.x",
                ..
            })
        ));
    }
}
