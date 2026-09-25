//! A wheeled robot boarding an elevator and riding between floors, headless.
//!
//! Multi-floor operation is what separates an indoor service robot from a
//! single-floor one, and the part a simulator has to get right is the boarding
//! contract: the robot may only cross the threshold while the car is at its
//! floor with the doors fully open, and once aboard it must be carried by the
//! car rather than left behind.
//!
//! `rne_nav::Elevator` owns the timing as a pure state machine; this example
//! attaches it to physics. The car and door leaves are kinematic bodies whose
//! poses follow the state machine each step, and the rider is an ordinary
//! dynamic body. Riding needs no special support: normal contact carries it.
//!
//! No renderer is involved. Run with:
//!
//! ```text
//! cargo run --release -p elevator_ride --example 118_elevator_ride
//! ```

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{Elevator, ElevatorSpec, ElevatorState};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;

/// Physics rate, in hertz.
const PHYSICS_HZ: f64 = 120.0;
/// Car platform half extents in meters.
const CAR_HALF_EXTENTS_M: Vec3 = Vec3::new(0.9, 0.05, 0.9);
/// Door leaf half extents in meters.
const DOOR_HALF_EXTENTS_M: Vec3 = Vec3::new(0.05, 1.0, 0.45);
/// Rider half extents in meters, standing on the car.
const RIDER_HALF_EXTENTS_M: Vec3 = Vec3::new(0.25, 0.2, 0.25);
/// Rider mass in kilograms, in the range of a small service robot.
const RIDER_MASS_KG: f64 = 30.0;
/// Seconds simulated before the ride is scored.
const MAX_SECONDS: f64 = 30.0;

fn spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: vec![0.0, 3.5, 7.0],
        car_speed_m_s: 1.0,
        car_acceleration_m_s2: 0.8,
        door_travel_m: 0.6,
        door_speed_m_s: 0.6,
        door_hold_s: 2.0,
    }
}

struct Shaft {
    car: Entity,
    left_door: Entity,
    right_door: Entity,
    rider: Entity,
}

fn spawn_shaft(world: &mut World, start_height_m: f64) -> Shaft {
    let car = spawn_named(world, "elevator_car");
    world.entity_mut(car).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: CAR_HALF_EXTENTS_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, start_height_m, 0.0), Quat::IDENTITY),
    ));

    let mut door = |name: &'static str| {
        let entity = spawn_named(world, name);
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: DOOR_HALF_EXTENTS_M,
                },
                ..Collider::default()
            },
            Transform3::IDENTITY,
        ));
        entity
    };
    let left_door = door("elevator_door_left");
    let right_door = door("elevator_door_right");

    let rider = spawn_named(world, "rider");
    world.entity_mut(rider).insert((
        RigidBody {
            mass_kg: RIDER_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: RIDER_HALF_EXTENTS_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(
                0.0,
                start_height_m + CAR_HALF_EXTENTS_M.y + RIDER_HALF_EXTENTS_M.y + 0.02,
                0.0,
            ),
            Quat::IDENTITY,
        ),
    ));

    Shaft {
        car,
        left_door,
        right_door,
        rider,
    }
}

/// Writes the state machine's car height and door opening onto the bodies.
fn apply_elevator(world: &mut World, shaft: &Shaft, elevator: &Elevator) {
    let car_height_m = elevator.car_height_m();
    if let Some(mut transform) = world.get_mut::<Transform3>(shaft.car) {
        transform.translation.y = car_height_m;
    }
    // Leaves part from the doorway centre by the opening amount.
    let door_centre_y_m = car_height_m + DOOR_HALF_EXTENTS_M.y;
    let opening_m = elevator.door_opening_m();
    let doorway_x_m = CAR_HALF_EXTENTS_M.x - DOOR_HALF_EXTENTS_M.x;
    for (entity, sign) in [(shaft.left_door, -1.0), (shaft.right_door, 1.0)] {
        if let Some(mut transform) = world.get_mut::<Transform3>(entity) {
            transform.translation = Vec3::new(
                sign * (doorway_x_m + opening_m),
                door_centre_y_m,
                CAR_HALF_EXTENTS_M.z,
            );
        }
    }
}

fn rider_height_m(world: &World, shaft: &Shaft) -> f64 {
    world
        .get::<Transform3>(shaft.rider)
        .expect("rider transform")
        .translation
        .y
}

fn main() {
    let spec = spec();
    spec.validate().expect("elevator specification");
    let start_floor = 0;
    let target_floor = 2;

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let shaft = spawn_shaft(&mut world, spec.floor_heights_m[start_floor]);
    let mut elevator = Elevator::new(spec.clone(), start_floor).expect("elevator");
    apply_elevator(&mut world, &shaft, &elevator);

    let dt = SimDuration::from_hertz(Hertz::new(PHYSICS_HZ));
    let dt_s = 1.0 / PHYSICS_HZ;

    // Let the rider settle onto the car before anything moves.
    for _ in 0..(PHYSICS_HZ as usize) {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("settle");
    }
    let settled_height_m = rider_height_m(&world, &shaft);
    let settled_clearance_m = settled_height_m - elevator.car_height_m();
    println!(
        "settled: rider y = {settled_height_m:.4} m, clearance above car = {settled_clearance_m:.4} m"
    );

    elevator.call(target_floor).expect("call the target floor");

    let mut boarded_at_wrong_time = false;
    let mut doors_open_while_moving = false;
    let mut max_clearance_error_m: f64 = 0.0;
    let mut arrived = false;
    let steps = (MAX_SECONDS * PHYSICS_HZ) as usize;

    for _ in 0..steps {
        elevator.update(dt_s).expect("elevator update");
        apply_elevator(&mut world, &shaft, &elevator);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");

        // The car must never travel with its doors open.
        if matches!(elevator.state(), ElevatorState::Moving { .. })
            && elevator.door_opening_m() > 0.0
        {
            doors_open_while_moving = true;
        }
        // The threshold must only be crossable at a stopped, fully open floor.
        if elevator.is_boardable(target_floor)
            && (elevator.car_height_m() - spec.floor_heights_m[target_floor]).abs() > 1.0e-9
        {
            boarded_at_wrong_time = true;
        }
        // The rider must stay on the car for the whole ride.
        let clearance_m = rider_height_m(&world, &shaft) - elevator.car_height_m();
        max_clearance_error_m =
            max_clearance_error_m.max((clearance_m - settled_clearance_m).abs());

        if elevator.is_boardable(target_floor) {
            arrived = true;
            break;
        }
    }

    let rider_y_m = rider_height_m(&world, &shaft);
    println!(
        "arrived: car y = {:.4} m, rider y = {rider_y_m:.4} m, worst ride clearance error = {max_clearance_error_m:.4} m",
        elevator.car_height_m()
    );

    assert!(arrived, "the elevator never reached the target floor");
    assert!(
        !doors_open_while_moving,
        "the car travelled with its doors open"
    );
    assert!(
        !boarded_at_wrong_time,
        "the elevator reported a boardable floor while the car was elsewhere"
    );
    // Carried, not left behind: the rider climbed the full shaft height.
    let climbed_m = rider_y_m - settled_height_m;
    let shaft_height_m = spec.floor_heights_m[target_floor] - spec.floor_heights_m[start_floor];
    assert!(
        (climbed_m - shaft_height_m).abs() < 0.05,
        "the rider climbed {climbed_m:.4} m of a {shaft_height_m:.4} m shaft"
    );
    // Riding is exact: the car is commanded as a kinematic body, so the solver
    // knows its velocity and the rider is carried rather than repeatedly caught
    // by penetration resolution. Teleporting the car instead leaves the rider
    // trailing it by 7 cm for the whole ascent.
    assert!(
        max_clearance_error_m < 1.0e-3,
        "the rider did not stay seated on the car during the ride: \
         worst clearance error {max_clearance_error_m:.4} m"
    );

    println!("state hash = {:016x}", hash_physics_state(&world));
}
