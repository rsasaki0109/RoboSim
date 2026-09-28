//! The warehouse the robot maps day after day: fixed walls, racks and a
//! column, and pallets that move between days.
//!
//! World coordinates are the physics world's `(x, z)`; the SLAM plane maps
//! `x -> x` and `z -> y`, so a rectangle here is a rectangle on the map.

use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_world::Transform3;

/// An axis-aligned footprint: centre `(x, z)` and half extents, in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Footprint {
    pub(crate) x: f64,
    pub(crate) z: f64,
    pub(crate) half_x: f64,
    pub(crate) half_z: f64,
    pub(crate) height: f64,
}

impl Footprint {
    const fn new(x: f64, z: f64, half_x: f64, half_z: f64, height: f64) -> Self {
        Self {
            x,
            z,
            half_x,
            half_z,
            height,
        }
    }

    pub(crate) fn contains(&self, x: f64, z: f64, margin: f64) -> bool {
        (x - self.x).abs() <= self.half_x + margin && (z - self.z).abs() <= self.half_z + margin
    }
}

/// Walls, racks, a column and the office block: everything that never moves.
pub(crate) const STATIC: [Footprint; 8] = [
    Footprint::new(0.0, 4.6, 7.2, 0.1, 2.4),    // north wall
    Footprint::new(0.0, -4.6, 7.2, 0.1, 2.4),   // south wall
    Footprint::new(7.1, 0.0, 0.1, 4.6, 2.4),    // east wall
    Footprint::new(-7.1, 0.0, 0.1, 4.6, 2.4),   // west wall
    Footprint::new(-3.0, -3.7, 2.2, 0.45, 2.0), // long rack
    Footprint::new(3.4, -3.7, 1.4, 0.45, 2.0),  // short rack
    Footprint::new(0.0, 1.05, 0.2, 0.2, 2.4),   // column
    Footprint::new(5.9, 3.5, 1.2, 1.0, 2.4),    // office block
];

/// Every place a pallet can stand. The robot's loop never enters one.
pub(crate) const SLOTS: [(f64, f64); 9] = [
    (-3.0, -0.2),
    (-1.6, -0.2),
    (1.4, -0.2),
    (2.8, -0.2),
    (-5.0, 3.3),
    (-3.4, 3.3),
    (-1.8, 3.3),
    (0.2, 3.3),
    (2.0, 3.3),
];
pub(crate) const PALLET_HALF_M: f64 = 0.45;
pub(crate) const PALLET_HEIGHT_M: f64 = 1.2;

/// Which slots hold a pallet on each day.
pub(crate) const DAYS: [&[usize]; 4] = [
    &[0, 1, 2, 4, 6, 7],
    // A pallet leaves the staging area, one arrives at the east end, and one
    // on the north side is moved two slots along.
    &[1, 2, 3, 4, 7, 8],
    &[1, 3, 4, 5, 7, 8],
    &[0, 3, 5, 7, 8],
];

/// Each day's route: where the robot starts, then the loop corners it drives
/// to, overlapping its start so the day closes its own loop. The robot starts
/// facing its first corner.
pub(crate) const ROUTES: [&[(f64, f64)]; 4] = [
    &[
        (-5.0, -2.0),
        (5.0, -2.0),
        (5.0, 1.9),
        (-5.0, 1.9),
        (-5.0, -2.0),
        (-2.0, -2.0),
    ],
    &[
        (5.0, 1.9),
        (-5.0, 1.9),
        (-5.0, -2.0),
        (5.0, -2.0),
        (5.0, 1.9),
        (2.0, 1.9),
    ],
    &[
        (0.0, -2.0),
        (5.0, -2.0),
        (5.0, 1.9),
        (-5.0, 1.9),
        (-5.0, -2.0),
        (3.0, -2.0),
    ],
    &[
        (-5.0, 1.9),
        (-5.0, -2.0),
        (5.0, -2.0),
        (5.0, 1.9),
        (-5.0, 1.9),
        (-5.0, -1.0),
    ],
];

pub(crate) fn pallets(day: usize) -> Vec<Footprint> {
    DAYS[day]
        .iter()
        .map(|slot| {
            let (x, z) = SLOTS[*slot];
            Footprint::new(x, z, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M)
        })
        .collect()
}

/// Spawns one fixed box per footprint and returns the entities.
pub(crate) fn spawn(world: &mut World, footprints: &[Footprint], name: &str) -> Vec<Entity> {
    footprints
        .iter()
        .map(|footprint| {
            let entity = spawn_named(world, name);
            world.entity_mut(entity).insert((
                RigidBody {
                    body_type: RigidBodyType::Fixed,
                    ..RigidBody::default()
                },
                Collider {
                    shape: ColliderShape::Cuboid {
                        half_extents_m: Vec3::new(
                            footprint.half_x,
                            0.5 * footprint.height,
                            footprint.half_z,
                        ),
                    },
                    ..Collider::default()
                },
                Transform3::from_translation_rotation(
                    Vec3::new(footprint.x, 0.5 * footprint.height, footprint.z),
                    Quat::IDENTITY,
                ),
            ));
            entity
        })
        .collect()
}
