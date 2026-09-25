//! Pressing an elevator call button through solved contact forces, headless.
//!
//! A robot that shares a building with people has to operate the building's own
//! controls. Summoning the car by calling an API proves nothing; reaching a
//! 2 cm target with enough force, and no more, is the thing that has to work.
//!
//! `rne_nav::CallButton` owns the button as a force-sensitive device: it accepts
//! only contacts on its face, sums their force, actuates with hysteresis, and
//! reports an edge, so one physical press produces exactly one elevator call
//! however long the presser rests on it. `rne_nav::Elevator` owns the car.
//!
//! The presser here is a dynamic body pushed by a known external wrench rather
//! than an arm. That keeps the contract under test the *button*, not a
//! manipulator controller, while the contact forces are still solved by the
//! physics backend exactly as an arm's would be. A kinematic presser would not
//! work: a kinematic body against a fixed body is a pair the solver never
//! resolves, because neither can move, so no contact force is produced at all.
//! Driving this with the SO-101 arm is the next step and is currently blocked.
//!
//! No renderer is involved. Run with:
//!
//! ```text
//! cargo run --release -p elevator_call_button --example 120_elevator_call_button
//! ```

use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{ButtonContact, CallButton, CallButtonSpec, CallButtonState, Elevator, ElevatorSpec};
use rne_physics::{
    Collider, ColliderShape, ExternalBodyWrench, PhysicsBackend, PhysicsWorldDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;

/// Physics rate, in hertz.
const PHYSICS_HZ: f64 = 240.0;
/// Button face centre in world meters, at panel height beside the door.
const BUTTON_CENTER_M: Vec3 = Vec3::new(0.0, 1.1, 0.0);
/// Half extents of the button body in meters: a 4 cm square plunger face.
const BUTTON_HALF_EXTENTS_M: Vec3 = Vec3::new(0.02, 0.02, 0.01);
/// Half extents of the presser in meters, about a fingertip.
const PRESSER_HALF_EXTENTS_M: Vec3 = Vec3::new(0.008, 0.008, 0.008);
/// Presser mass in kilograms.
const PRESSER_MASS_KG: f64 = 1.0;
/// Force driving the presser onto the face, in newtons.
///
/// Above the button's actuation threshold and well below anything that would
/// drive the fingertip through the panel.
const PRESS_FORCE_N: f64 = 3.0;
/// How far in front of the face the presser starts, in meters.
///
/// Short, so the fingertip arrives slowly: a free body accelerated over a long
/// approach lands with an impact force many times the load it then holds.
const APPROACH_M: f64 = 0.002;
/// Floor this button calls.
const CALL_FLOOR: usize = 2;
/// Seconds of each phase.
const APPROACH_S: f64 = 2.0;
const HOLD_S: f64 = 1.5;
const RETRACT_S: f64 = 2.0;

fn elevator_spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: vec![0.0, 3.5, 7.0],
        car_speed_m_s: 1.0,
        car_acceleration_m_s2: 0.8,
        door_travel_m: 0.6,
        door_speed_m_s: 0.6,
        door_hold_s: 2.0,
    }
}

/// Presser face position for one phase progress, along the button normal.
///
/// The button face normal points at `-Z`, so the presser travels in `+Z`.
fn presser_z_m(offset_from_face_m: f64) -> f64 {
    BUTTON_CENTER_M.z - offset_from_face_m - PRESSER_HALF_EXTENTS_M.z
}

fn main() {
    let spec = elevator_spec();
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsWorldDesc::default()
        })
        .expect("physics world");
    let mut world = World::new();

    // The button body is part of the building: fixed, and the arm cannot push
    // it out of the way. Its face is the side the presser approaches.
    let button_body = spawn_named(&mut world, "call_button");
    world.entity_mut(button_body).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: BUTTON_HALF_EXTENTS_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            BUTTON_CENTER_M + Vec3::new(0.0, 0.0, BUTTON_HALF_EXTENTS_M.z),
            Quat::IDENTITY,
        ),
    ));

    let presser = spawn_named(&mut world, "presser");
    world.entity_mut(presser).insert((
        RigidBody {
            mass_kg: PRESSER_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: PRESSER_HALF_EXTENTS_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(
                BUTTON_CENTER_M.x,
                BUTTON_CENTER_M.y,
                presser_z_m(APPROACH_M),
            ),
            Quat::IDENTITY,
        ),
    ));

    let mut button = CallButton::new(CallButtonSpec {
        center_world_m: BUTTON_CENTER_M,
        normal_world: Vec3::new(0.0, 0.0, -1.0),
        radius_m: 0.02,
        travel_m: 0.006,
        press_force_n: 2.0,
        release_force_n: 0.5,
        floor: CALL_FLOOR,
    })
    .expect("call button");
    let mut elevator = Elevator::new(spec.clone(), 0).expect("elevator");

    let dt = SimDuration::from_hertz(Hertz::new(PHYSICS_HZ));
    let dt_s = 1.0 / PHYSICS_HZ;

    let mut peak_force_n: f64 = 0.0;
    let mut hold_force_sum_n = 0.0;
    let mut hold_samples = 0usize;
    let mut pressed_at_s = None;
    let mut released_at_s = None;
    let mut elapsed_s = 0.0;
    let total_s = APPROACH_S + HOLD_S + RETRACT_S;

    while elapsed_s < total_s {
        // Push onto the face, hold, then withdraw. The presser is driven by a
        // force rather than a commanded pose, so the load the button reads is
        // the load the solver actually resolved.
        let drive_n = if elapsed_s < APPROACH_S + HOLD_S {
            PRESS_FORCE_N
        } else {
            -PRESS_FORCE_N
        };

        // The wrench must land between the synchronization that creates the
        // bodies and the step that consumes the forces.
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("sync");
        let presser_m = world
            .get::<Transform3>(presser)
            .expect("presser transform")
            .translation;
        backend
            .apply_external_body_wrench(
                physics_world,
                ExternalBodyWrench {
                    entity: presser,
                    point_world_m: presser_m,
                    force_world_n: Vec3::new(0.0, 0.0, drive_n),
                    torque_world_nm: Vec3::ZERO,
                },
            )
            .expect("drive the presser");
        backend.step(physics_world, dt).expect("step");
        backend
            .sync_to_ecs(&mut world, physics_world)
            .expect("readback");

        let contacts: Vec<ButtonContact> = backend
            .contact_points(physics_world)
            .expect("contact points")
            .iter()
            .filter(|sample| sample.entity_a == button_body || sample.entity_b == button_body)
            .map(|sample| ButtonContact {
                point_world_m: sample.point_world_m,
                normal_force_n: sample.normal_force_n,
            })
            .collect();
        let state = button.update(&contacts);
        peak_force_n = peak_force_n.max(button.applied_force_n());
        // Sample the settled load over the second half of the hold, past the
        // impact transient of the fingertip arriving.
        if elapsed_s > APPROACH_S + HOLD_S / 2.0 && elapsed_s < APPROACH_S + HOLD_S {
            hold_force_sum_n += button.applied_force_n();
            hold_samples += 1;
        }

        if button.just_pressed() {
            pressed_at_s = Some(elapsed_s);
            elevator.call(button.spec().floor).expect("call the floor");
            println!(
                "pressed at {elapsed_s:.2} s with {:.2} N; elevator called to floor {CALL_FLOOR}",
                button.applied_force_n()
            );
        }
        if released_at_s.is_none() && pressed_at_s.is_some() && state == CallButtonState::Released {
            released_at_s = Some(elapsed_s);
        }

        elapsed_s += dt_s;
    }

    let hold_force_n = hold_force_sum_n / hold_samples.max(1) as f64;
    println!(
        "face load: {peak_force_n:.2} N peak on impact, {hold_force_n:.2} N settled; {} press(es)",
        button.press_count()
    );

    let pressed_at_s = pressed_at_s.expect("the presser never actuated the button");
    assert!(
        pressed_at_s > 0.0,
        "the button actuated before the presser reached it"
    );
    assert_eq!(
        button.press_count(),
        1,
        "holding the button down must summon the car once, not repeatedly"
    );
    assert!(
        peak_force_n >= button.spec().press_force_n,
        "the recorded force never reached the actuation threshold"
    );
    // The settled load tracks the drive force, so the button is reading the load
    // the solver resolved rather than an impact artefact. It sits slightly above
    // the drive because Rapier's contact constraint also carries a penetration
    // recovery term, which is a push the button genuinely receives.
    assert!(
        hold_force_n >= PRESS_FORCE_N,
        "settled face load {hold_force_n:.2} N fell below the {PRESS_FORCE_N:.2} N drive"
    );
    assert!(
        hold_force_n < PRESS_FORCE_N + 1.0,
        "settled face load {hold_force_n:.2} N is far above the {PRESS_FORCE_N:.2} N drive"
    );
    let released_at_s = released_at_s.expect("the button never released when the presser withdrew");
    assert!(
        released_at_s > pressed_at_s,
        "the button released before it was pressed"
    );
    println!("released at {released_at_s:.2} s when the presser withdrew");
    assert_eq!(elevator.pending_calls(), &[CALL_FLOOR]);

    // The car answers the call.
    let mut arrived_after_s = None;
    for step in 0..20_000 {
        elevator.update(dt_s).expect("elevator update");
        if elevator.is_boardable(CALL_FLOOR) {
            arrived_after_s = Some(step as f64 * dt_s);
            break;
        }
    }
    let arrived_after_s = arrived_after_s.expect("the car never answered the call");
    println!(
        "car arrived at floor {CALL_FLOOR} ({:.2} m) after {arrived_after_s:.2} s, doors open",
        elevator.car_height_m()
    );
    assert!((elevator.car_height_m() - spec.floor_heights_m[CALL_FLOOR]).abs() < 1.0e-9);
}
