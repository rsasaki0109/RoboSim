//! Rigid bodies settling on a sampled heightfield instead of an infinite plane.
//!
//! Every legged and wheeled demo in this repository has run on a flat
//! `ColliderShape::Plane`. `HeightfieldCollider` replaces that ground with a
//! sampled terrain patch, so contact happens at the sampled surface and a body
//! that rolls off the patch edge falls rather than finding more floor.
//!
//! The terrain here is a 12 m x 12 m patch combining a constant ramp along the
//! local X axis with a ripple along Z, which is asymmetric in both axes: a
//! transposed or mis-scaled grid moves every reported contact height.
//!
//! Run headlessly with:
//!
//! ```text
//! cargo run -p terrain_heightfield --example 116_terrain_heightfield
//! ```

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, HeightfieldCollider, PhysicsBackend,
    PhysicsWorldDesc, RaycastQuery, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;

/// Sample counts along the patch's local X and Z axes.
const ROWS: u32 = 33;
const COLUMNS: u32 = 33;
/// Total horizontal extents in meters; the Y component keeps heights in meters.
const EXTENT_M: Vec3 = Vec3::new(12.0, 1.0, 12.0);
/// Rise of the constant ramp across the full X extent, in meters.
const RAMP_RISE_M: f64 = 1.2;
/// Peak-to-zero amplitude of the ripple along Z, in meters.
const RIPPLE_AMPLITUDE_M: f64 = 0.08;
/// Number of full ripple periods across the Z extent.
const RIPPLE_PERIODS: f64 = 3.0;
const BALL_RADIUS_M: f64 = 0.15;

/// Analytic terrain height in meters at one local ground position.
///
/// The example builds its samples from this function, so the same expression
/// is also the expected contact height the run checks itself against.
fn terrain_height_m(x_m: f64, z_m: f64) -> f64 {
    let ramp = RAMP_RISE_M * (x_m / EXTENT_M.x + 0.5);
    let ripple =
        RIPPLE_AMPLITUDE_M * (RIPPLE_PERIODS * std::f64::consts::TAU * (z_m / EXTENT_M.z)).sin();
    ramp + ripple
}

/// Builds the terrain patch by sampling [`terrain_height_m`] on the grid.
fn build_terrain() -> HeightfieldCollider {
    let mut field = HeightfieldCollider::flat(ROWS, COLUMNS, EXTENT_M);
    for row in 0..ROWS {
        // Samples span the full extent inclusive of both edges.
        let x_m = EXTENT_M.x * (f64::from(row) / f64::from(ROWS - 1) - 0.5);
        for column in 0..COLUMNS {
            let z_m = EXTENT_M.z * (f64::from(column) / f64::from(COLUMNS - 1) - 0.5);
            *field
                .height_mut(row, column)
                .expect("sample inside the declared grid") = terrain_height_m(x_m, z_m);
        }
    }
    field
}

fn spawn_ball(world: &mut World, name: &'static str, position_m: Vec3) -> Entity {
    let ball = spawn_named(world, name);
    world.entity_mut(ball).insert((
        RigidBody::default(),
        Collider {
            shape: ColliderShape::Sphere {
                radius_m: BALL_RADIUS_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(position_m, Quat::IDENTITY),
    ));
    ball
}

fn main() {
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");

    let mut world = World::new();
    let terrain_field = build_terrain();
    assert!(
        terrain_field.is_valid(),
        "terrain patch must be accepted by a backend"
    );
    let terrain = spawn_named(&mut world, "terrain");
    world.entity_mut(terrain).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        // The companion stays a bounding approximation for consumers that do
        // not read the heightfield.
        Collider::cuboid(Vec3::new(EXTENT_M.x / 2.0, RAMP_RISE_M, EXTENT_M.z / 2.0)),
        terrain_field,
        Transform3::IDENTITY,
    ));

    let balls = [
        ("uphill", Vec3::new(-3.0, 3.0, -2.5)),
        ("crest", Vec3::new(0.0, 3.0, 0.0)),
        ("downhill", Vec3::new(3.0, 3.0, 2.5)),
    ]
    .map(|(name, position_m)| (name, spawn_ball(&mut world, name, position_m)));

    // Probe the surface before stepping: the query pipeline reports the sampled
    // terrain, not the bounding companion.
    println!("sampled terrain surface");
    for (x_m, z_m) in [(-4.0, 0.0), (0.0, 0.0), (4.0, 0.0), (0.0, 2.0)] {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("sync terrain");
        let hits = backend
            .raycast(
                physics_world,
                RaycastQuery::downward(Vec3::new(x_m, 5.0, z_m), 10.0),
            )
            .expect("terrain raycast");
        // The dropped bodies are still overhead, so select the terrain hit.
        let hit = hits
            .iter()
            .find(|hit| hit.entity == terrain)
            .expect("terrain under the probe");
        let expected_m = terrain_height_m(x_m, z_m);
        println!(
            "  ({x_m:+.1}, {z_m:+.1}) -> y = {:.4} m (expected {expected_m:.4} m)",
            hit.point_m.y
        );
        assert!(
            (hit.point_m.y - expected_m).abs() < 1e-3,
            "terrain contact height drifted from the sampled grid"
        );
    }

    let dt = SimDuration::from_hertz(Hertz::new(60.0));
    for _ in 0..240 {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");
    }

    println!("bodies after 4 s");
    for (name, ball) in balls {
        let center_m = world
            .get::<Transform3>(ball)
            .expect("ball transform")
            .translation;
        let surface_m = terrain_height_m(center_m.x, center_m.z);
        let clearance_m = center_m.y - surface_m;
        println!(
            "  {name}: x = {:+.3} m, y = {:.3} m, z = {:+.3} m, above terrain = {clearance_m:.3} m",
            center_m.x, center_m.y, center_m.z
        );
        assert!(
            center_m.x.abs() < EXTENT_M.x / 2.0 && center_m.z.abs() < EXTENT_M.z / 2.0,
            "{name} rolled off the patch, so its rest pose is not a terrain contact"
        );
        assert!(
            clearance_m > 0.0,
            "{name} sank through the sampled terrain surface"
        );
        assert!(
            clearance_m < 4.0 * BALL_RADIUS_M,
            "{name} never reached the terrain"
        );
    }

    println!("state hash = {:016x}", hash_physics_state(&world));
}
