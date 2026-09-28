//! Two forklift AGVs hand one case across two floors through the lift.
//!
//! Example 123 has one truck ride the lift with its load. A site with a truck
//! on every floor does not do that: the lift is the vertical conveyor between
//! them. Here the ground-floor truck takes the case off the goods-in stand,
//! calls the car, turns round and sets the case on a stand inside the car,
//! then backs out. The car goes up with nothing in it but the case, and the
//! upper-floor truck, which has been waiting at the landing, forks it out of
//! the car, turns round and sets it down on the outbound bay.
//!
//! What is physical and what is commanded:
//!
//! * Each mast is a real prismatic joint with a position servo, and the case
//!   is an ordinary dynamic body from start to finish: lifted because tines are
//!   under it, set down because they come back down. The in-car stand is two
//!   kinematic legs that move with the car, so the case rides up on contact.
//! * The car and door leaves are `rne_nav::Elevator` state; the lobby button is
//!   `rne_nav::CallButton` reading solved contact force from the truck's body.
//! * The doors are held by a light-curtain check: while any truck body or the
//!   case overlaps the doorway, the mission calls `Elevator::hold_doors`. That
//!   check reads the bodies' poses; it is not a simulated sensor.
//! * The chassis velocity **and heading** are commanded. The drive wheels are
//!   not modelled, so turning round is a commanded yaw rate, limited so that
//!   an unstrapped case stays on the tines.
//! * Sending the car up once the ground-floor truck is clear is a call made by
//!   the mission, standing in for the site's dispatch system.
//!
//! ```text
//! cargo run --release -p warehouse_relay --example 125_warehouse_relay -- --smoke
//! cargo run --release -p warehouse_relay --example 125_warehouse_relay
//! ```

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Transform3 as MathTransform, Vec3};
use rne_nav::{ButtonContact, CallButton, CallButtonSpec, Elevator, ElevatorSpec, ElevatorState};
use rne_physics::{
    Collider, ColliderShape, CommandedKinematicPose, JointMotor, JointMotorGainModel,
    PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, PrismaticJointDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, ImageFrame, MeshRenderCache, PbrMaterial,
    RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

/// Deck half extents per floor, in meters.
const DECK_HALF_M: [Vec3; 2] = [Vec3::new(3.9, 0.06, 1.5), Vec3::new(1.65, 0.06, 1.5)];
/// Deck centre on the world x axis per floor, in meters.
const DECK_X_M: [f64; 2] = [SHAFT_X_M - 4.8, SHAFT_X_M - 2.55];

/// Physics rate, as in example 123: the tines carry an unstrapped case
/// through contact, which wants small steps.
const PHYSICS_HZ: f64 = 240.0;
/// Floor heights served by the lift, in meters.
const FLOOR_HEIGHTS_M: [f64; 2] = [0.0, 3.2];
/// Car platform half extents, in meters.
const CAR_HALF_M: Vec3 = Vec3::new(0.95, 0.06, 0.95);
/// Door leaf half extents, in meters.
const DOOR_HALF_M: Vec3 = Vec3::new(0.05, 1.15, 0.46);
/// Shaft centre on the world x axis, in meters.
const SHAFT_X_M: f64 = 0.0;
/// The doorway plane: the car's landing edge, in meters.
const DOORWAY_X_M: f64 = SHAFT_X_M - CAR_HALF_M.x;

/// Chassis half extents, in meters.
const CHASSIS_HALF_M: Vec3 = Vec3::new(0.52, 0.26, 0.30);
/// Chassis mass, in kilograms.
const CHASSIS_MASS_KG: f64 = 320.0;
/// Fork carriage (the tines) half extents, in meters.
const FORK_HALF_M: Vec3 = Vec3::new(0.34, 0.025, 0.14);
/// Fork carriage mass, in kilograms.
const FORK_MASS_KG: f64 = 8.0;
/// Where the tines sit ahead of the chassis centre along the truck's own x,
/// in meters. Negative: the forks are at the truck's local -x end.
const FORK_REACH_M: f64 = -0.95;
/// Mast travel limits and set points, in meters of joint travel.
const FORK_GROUND_M: f64 = -0.30;
const FORK_ENGAGE_M: f64 = 0.0;
const FORK_CARRY_M: f64 = 0.30;
/// Mast anchor height above the chassis centre, in meters.
const MAST_ANCHOR_Y_M: f64 = 0.015;
/// Backrest half extents and mass.
const BACKREST_HALF_M: Vec3 = Vec3::new(0.025, 0.21, 0.24);
const BACKREST_MASS_KG: f64 = 4.0;
/// Where a case sits on engaged tines, along the truck's own x, in meters:
/// deep, against the backrest, which is where example 123 found a load stays
/// put when the truck changes speed.
const CASE_LOCAL_X_M: f64 = FORK_REACH_M + (FORK_HALF_M.x - CASE_HALF_M.x) + 0.02;

/// Case half extents and mass.
const CASE_HALF_M: Vec3 = Vec3::new(0.19, 0.16, 0.26);
const CASE_MASS_KG: f64 = 14.0;

/// Stand geometry shared by all three stands: two legs with a gap the tines
/// fit between.
const STAND_TOP_Y_M: f64 = 0.34;
const STAND_LEG_HALF_M: Vec3 = Vec3::new(0.10, STAND_TOP_Y_M * 0.5, 0.07);
const STAND_LEG_Z_M: f64 = 0.23;

/// Goods-in stand on the ground floor, in meters.
const STAND_X_M: f64 = -4.30;
/// In-car stand centre on the world x axis, in meters.
///
/// Far enough in that the tines' tips stay 0.29 m short of the car's back
/// wall, and near enough the door that a truck reaching in keeps its chassis
/// mostly on the landing.
const CAR_STAND_X_M: f64 = SHAFT_X_M + 0.15;
/// Outbound stand on the upper floor, centred under where the case sits when
/// the truck is at `BAY_X_M`.
const BAY_X_M: f64 = SHAFT_X_M - 2.35;
const OUTBOUND_X_M: f64 = BAY_X_M + CASE_LOCAL_X_M;

/// Truck headings, in radians about +y. At 0 the forks face -x (back into the
/// building); at pi they face +x, into the car.
const FORKS_OUT_RAD: f64 = 0.0;
const FORKS_IN_RAD: f64 = std::f64::consts::PI;

/// Ground-floor truck stops, in meters of chassis x.
const A_ENGAGED_X_M: f64 = STAND_X_M - CASE_LOCAL_X_M;
const A_APPROACH_X_M: f64 = A_ENGAGED_X_M + CASE_HALF_M.x + 0.46;
/// Press stop: the chassis's side on the button face, its nose short of the
/// closed door leaves. Example 123 stops 0.15 m further in, which only works
/// because its diagonal approach has already pressed the button and opened
/// the doors by the time it gets there.
const PRESS_X_M: f64 = DOORWAY_X_M - DOOR_HALF_M.x - CHASSIS_HALF_M.x - 0.03;
const BUTTON_CENTER_M: Vec3 = Vec3::new(-1.25, 0.56, CHASSIS_HALF_M.z + 0.02);
const BUTTON_NORMAL: Vec3 = Vec3::new(0.0, 0.0, -1.0);
const PRESS_Z_M: f64 = BUTTON_CENTER_M.z - CHASSIS_HALF_M.z + 0.015;
const BUTTON_HALF_M: Vec3 = Vec3::new(0.05, 0.05, 0.03);
/// Where the ground-floor truck turns round, in meters.
///
/// The swept circle is the tine tips, 1.29 m from the chassis centre. Here it
/// clears the goods-in stand behind and the button stanchion ahead, and the
/// truck turns through +z, the open side of the aisle: the racking is on -z.
const A_TURN_X_M: f64 = -2.75;
/// Where a truck stands with the case over the in-car stand.
const CAR_LOAD_X_M: f64 = CAR_STAND_X_M + CASE_LOCAL_X_M;
/// Chassis x a truck must be behind for its tine tips to be clear of the
/// doorway plane, with a margin.
const CLEAR_OF_DOORWAY_X_M: f64 = DOORWAY_X_M + FORK_REACH_M - FORK_HALF_M.x - 0.10;

/// Upper-floor truck: waits and turns here, tine tips clear of the doorway.
const B_TURN_X_M: f64 = -2.45;
/// Where it ends up after backing off the delivered case: tine tips 0.12 m
/// past the case's face, tail short of the door leaves.
///
/// Example 123's stop leaves the tips 8 cm under the case, which is set down
/// but not let go of.
const B_WITHDRAW_X_M: f64 = OUTBOUND_X_M + CASE_HALF_M.x - FORK_REACH_M + FORK_HALF_M.x + 0.12;

/// Drive speeds and limits.
const DRIVE_M_S: f64 = 0.62;
const CREEP_M_S: f64 = 0.22;
const DRIVE_ACCEL_M_S2: f64 = 0.5;
/// Turning rate and its rate limit, in rad/s and rad/s^2.
///
/// Slow enough that the case, 0.78 m out on the tines and held only by
/// friction, feels 0.1 m/s^2 of centripetal and tangential acceleration.
const TURN_RATE_RAD_S: f64 = 0.35;
const TURN_ACCEL_RAD_S2: f64 = 0.25;
/// Seconds the relay is allowed before it is declared stuck.
const MAX_SECONDS: f64 = 200.0;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.30, 0.34, 0.40, 1.0];
const FRAME_COUNT: usize = 150;

fn elevator_spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: FLOOR_HEIGHTS_M.to_vec(),
        car_speed_m_s: 1.1,
        car_acceleration_m_s2: 0.9,
        door_travel_m: 0.56,
        door_speed_m_s: 0.55,
        // Short: the light curtain holds the doors for as long as a truck is
        // in the doorway, so the dwell only has to cover the approach.
        door_hold_s: 5.0,
    }
}

/// One forklift: the three bodies physics solves, and its drive command.
struct Truck {
    chassis: Entity,
    fork: Entity,
    backrest: Entity,
    /// Commanded heading and its rate, in radians and rad/s.
    yaw_rad: f64,
    yaw_rate_rad_s: f64,
    /// Commanded world-frame velocity, in m/s.
    velocity_x_m_s: f64,
    velocity_z_m_s: f64,
}

/// Where a truck is going this step.
#[derive(Clone, Copy)]
struct DriveGoal {
    x_m: f64,
    z_m: f64,
    yaw_rad: f64,
    speed_m_s: f64,
    mast_m: f64,
}

/// Spawns a truck with its chassis centre at `position` and heading `yaw_rad`.
///
/// The fork and backrest start where their joints put them: spawning them
/// unrotated under a rotated chassis would have the solver snap them round on
/// the first step.
fn spawn_truck(world: &mut World, name: &str, position: Vec3, yaw_rad: f64) -> Truck {
    let rotation = Quat::from_rotation_y(yaw_rad);
    let chassis = spawn_named(world, format!("{name}_chassis"));
    world.entity_mut(chassis).insert((
        RigidBody {
            mass_kg: CHASSIS_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: CHASSIS_HALF_M,
            },
            material: rolling_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(position, rotation),
    ));

    let fork_anchor = Vec3::new(FORK_REACH_M, MAST_ANCHOR_Y_M, 0.0);
    let fork = spawn_named(world, format!("{name}_fork"));
    world.entity_mut(fork).insert((
        RigidBody {
            mass_kg: FORK_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: FORK_HALF_M,
            },
            material: high_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(position + rotation * fork_anchor, rotation),
        PrismaticJointDesc {
            parent: chassis,
            axis: Vec3::Y,
            anchor_parent_m: fork_anchor,
            anchor_child_m: Vec3::ZERO,
            relative_rotation: Quat::IDENTITY,
            lower_m: Some(FORK_GROUND_M),
            upper_m: Some(FORK_CARRY_M + 0.05),
        },
        // Example 123's mast: force-based and capped so the loaded tines
        // cannot torque the truck onto its nose.
        JointMotorGainModel::ForceBased,
        JointMotor {
            target_position: FORK_ENGAGE_M,
            stiffness: 6_000.0,
            gain: 600.0,
            max_force: 700.0,
            ..JointMotor::default()
        },
    ));

    // The backrest rides its own rail rather than being welded to the fork;
    // example 123 measured a welded chain losing most of the mast's authority.
    let backrest_anchor =
        fork_anchor + Vec3::new(FORK_HALF_M.x + BACKREST_HALF_M.x, BACKREST_HALF_M.y, 0.0);
    let backrest = spawn_named(world, format!("{name}_backrest"));
    world.entity_mut(backrest).insert((
        RigidBody {
            mass_kg: BACKREST_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: BACKREST_HALF_M,
            },
            material: high_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(position + rotation * backrest_anchor, rotation),
        PrismaticJointDesc {
            parent: chassis,
            axis: Vec3::Y,
            anchor_parent_m: backrest_anchor,
            anchor_child_m: Vec3::ZERO,
            relative_rotation: Quat::IDENTITY,
            lower_m: Some(FORK_GROUND_M),
            upper_m: Some(FORK_CARRY_M + 0.05),
        },
        JointMotorGainModel::ForceBased,
        JointMotor {
            target_position: FORK_ENGAGE_M,
            stiffness: 6_000.0,
            gain: 600.0,
            max_force: 400.0,
            ..JointMotor::default()
        },
    ));

    Truck {
        chassis,
        fork,
        backrest,
        yaw_rad,
        yaw_rate_rad_s: 0.0,
        velocity_x_m_s: 0.0,
        velocity_z_m_s: 0.0,
    }
}

/// Grip so a case parked on the tines stays there when the truck accelerates.
fn high_friction() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: 1.4,
        restitution: 0.0,
    }
}

/// The chassis base: the drive wheels are not modelled, so it slides.
fn rolling_friction() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: 0.04,
        restitution: 0.0,
    }
}

/// Decks and the car floor.
///
/// The solver averages the two materials in a contact, so a slippery chassis
/// on a grippy deck is not slippery: at the stands' 1.4 it came to 0.72, and
/// at that the base's stiction swallowed every drive or turn command below
/// about 0.03 m/s or 0.13 rad/s in a single step -- the truck parked 3 cm short
/// of its stops and 0.03 rad off its heading. Nothing that carries the case
/// touches the deck, so the deck can be the slippery side: 0.06 combined.
fn deck_friction() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: 0.08,
        restitution: 0.0,
    }
}

fn fixed_body() -> RigidBody {
    RigidBody {
        body_type: RigidBodyType::Fixed,
        ..RigidBody::default()
    }
}

/// Spawns a two-legged stand on a fixed deck.
fn spawn_stand(world: &mut World, name: &str, center_x_m: f64, deck_y_m: f64) {
    for (suffix, sign) in [("near", -1.0), ("far", 1.0)] {
        let leg = spawn_named(world, format!("{name}_leg_{suffix}"));
        world.entity_mut(leg).insert((
            fixed_body(),
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: STAND_LEG_HALF_M,
                },
                material: high_friction(),
                ..Collider::default()
            },
            Transform3::from_translation_rotation(
                Vec3::new(
                    center_x_m,
                    deck_y_m + STAND_LEG_HALF_M.y,
                    sign * STAND_LEG_Z_M,
                ),
                Quat::IDENTITY,
            ),
        ));
    }
}

struct Site {
    car: Entity,
    left_door: Entity,
    right_door: Entity,
    /// The in-car stand's two legs, which ride with the car.
    car_legs: [Entity; 2],
    button: Entity,
    case: Entity,
    lower: Truck,
    upper: Truck,
}

/// Chassis centre height above a floor, in meters.
fn chassis_height_m(floor: usize) -> f64 {
    FLOOR_HEIGHTS_M[floor] + CAR_HALF_M.y + CHASSIS_HALF_M.y + 0.01
}

fn spawn_site(world: &mut World) -> Site {
    for (index, height_m) in FLOOR_HEIGHTS_M.iter().enumerate() {
        let slab = spawn_named(world, if index == 0 { "deck_1f" } else { "deck_2f" });
        world.entity_mut(slab).insert((
            fixed_body(),
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: DECK_HALF_M[index],
                },
                material: deck_friction(),
                ..Collider::default()
            },
            Transform3::from_translation_rotation(
                Vec3::new(DECK_X_M[index], *height_m, 0.0),
                Quat::IDENTITY,
            ),
        ));
    }
    spawn_stand(world, "goods_in", STAND_X_M, CAR_HALF_M.y);
    spawn_stand(
        world,
        "outbound",
        OUTBOUND_X_M,
        FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y,
    );

    let car = spawn_named(world, "elevator_car");
    world.entity_mut(car).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        // It carries the in-car stand's load, so its motion is a command the
        // solver sees as a velocity.
        CommandedKinematicPose,
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: CAR_HALF_M,
            },
            material: deck_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(SHAFT_X_M, FLOOR_HEIGHTS_M[0], 0.0),
            Quat::IDENTITY,
        ),
    ));
    let mut kinematic = |name: String, half: Vec3, commanded: bool| {
        let entity = spawn_named(world, name);
        let mut entity_mut = world.entity_mut(entity);
        entity_mut.insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: half,
                },
                material: high_friction(),
                ..Collider::default()
            },
            Transform3::IDENTITY,
        ));
        if commanded {
            // These carry the case up the shaft: the contact that lifts it
            // needs their velocity, not just a new position each step.
            entity_mut.insert(CommandedKinematicPose);
        }
        entity
    };
    let left_door = kinematic("door_left".into(), DOOR_HALF_M, false);
    let right_door = kinematic("door_right".into(), DOOR_HALF_M, false);
    let car_legs = [
        kinematic("car_stand_leg_near".into(), STAND_LEG_HALF_M, true),
        kinematic("car_stand_leg_far".into(), STAND_LEG_HALF_M, true),
    ];

    let button = spawn_named(world, "call_button");
    world.entity_mut(button).insert((
        fixed_body(),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: BUTTON_HALF_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
            Quat::IDENTITY,
        ),
    ));

    let lower = spawn_truck(
        world,
        "truck_1f",
        Vec3::new(A_APPROACH_X_M, chassis_height_m(0), 0.0),
        FORKS_OUT_RAD,
    );
    let upper = spawn_truck(
        world,
        "truck_2f",
        Vec3::new(B_TURN_X_M, chassis_height_m(1), 0.0),
        FORKS_IN_RAD,
    );

    let case = spawn_named(world, "inbound_case");
    world.entity_mut(case).insert((
        RigidBody {
            mass_kg: CASE_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: CASE_HALF_M,
            },
            material: high_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(
                STAND_X_M,
                CAR_HALF_M.y + STAND_TOP_Y_M + CASE_HALF_M.y + 0.01,
                0.0,
            ),
            Quat::IDENTITY,
        ),
    ));

    Site {
        car,
        left_door,
        right_door,
        car_legs,
        button,
        case,
        lower,
        upper,
    }
}

/// Writes the elevator's car height and door opening onto the car, its doors
/// and the stand that rides in it.
fn apply_elevator(world: &mut World, site: &Site, elevator: &Elevator) {
    let car_height_m = elevator.car_height_m();
    let set = |world: &mut World, entity: Entity, translation: Vec3| {
        if let Some(mut transform) = world.get_mut::<Transform3>(entity) {
            transform.translation = translation;
        }
    };
    set(world, site.car, Vec3::new(SHAFT_X_M, car_height_m, 0.0));
    let opening_m = elevator.door_opening_m();
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for (entity, sign) in [(site.left_door, -1.0), (site.right_door, 1.0)] {
        set(
            world,
            entity,
            Vec3::new(
                DOORWAY_X_M,
                car_height_m + DOOR_HALF_M.y,
                sign * (doorway_z_m + opening_m),
            ),
        );
    }
    for (entity, sign) in [(site.car_legs[0], -1.0), (site.car_legs[1], 1.0)] {
        set(
            world,
            entity,
            Vec3::new(
                CAR_STAND_X_M,
                car_height_m + CAR_HALF_M.y + STAND_LEG_HALF_M.y,
                sign * STAND_LEG_Z_M,
            ),
        );
    }
}

fn pose(world: &World, entity: Entity) -> (Vec3, Quat) {
    world
        .get::<Transform3>(entity)
        .map_or((Vec3::ZERO, Quat::IDENTITY), |t| {
            (t.translation, t.rotation)
        })
}

/// Writes this step's velocity, heading and mast target onto a truck.
///
/// Translation and turning are separate: the truck turns only once it has
/// arrived, so the tines sweep a circle about a known point instead of a
/// curve through whatever is nearby.
fn drive_truck(world: &mut World, truck: &mut Truck, goal: DriveGoal, dt_s: f64) {
    let (chassis, rotation) = pose(world, truck.chassis);
    let yaw_now_rad = yaw_of(rotation);
    let heading_error = goal.yaw_rad - truck.yaw_rad;
    // Hold station until the body, not just the command, faces the goal.
    let turning = heading_error.abs() > 1.0e-3 || wrap_rad(goal.yaw_rad - yaw_now_rad).abs() > 0.01;

    // Rate-limited commands throughout: the case is not strapped down, and a
    // velocity that changes inside one step leaves it behind.
    let ramp =
        |current: f64, target: f64, limit: f64| current + (target - current).clamp(-limit, limit);
    // Proportional near the goal, and never faster than can still stop there
    // at the acceleration limit: the ramp alone overshot a stop by 0.2 m.
    let gain = |error_m: f64| {
        let stoppable_m_s = (2.0 * 0.8 * DRIVE_ACCEL_M_S2 * error_m.abs()).sqrt();
        let speed_m_s = (error_m / 0.25).clamp(-1.0, 1.0) * goal.speed_m_s.min(stoppable_m_s);
        // A floor near the goal: below a few cm/s the chassis's contact
        // stiction swallows the command and the truck parks 3 cm short.
        if error_m.abs() > 0.004 {
            error_m.signum() * speed_m_s.abs().max(0.03)
        } else {
            0.0
        }
    };
    let (target_x, target_z) = if turning {
        (0.0, 0.0)
    } else {
        (gain(goal.x_m - chassis.x), gain(goal.z_m - chassis.z))
    };
    let step_m_s = DRIVE_ACCEL_M_S2 * dt_s;
    truck.velocity_x_m_s = ramp(truck.velocity_x_m_s, target_x, step_m_s);
    truck.velocity_z_m_s = ramp(truck.velocity_z_m_s, target_z, step_m_s);

    // Turn with a trapezoidal rate: brake in time to stop on the heading.
    let braking_rad = truck.yaw_rate_rad_s.powi(2) / (2.0 * TURN_ACCEL_RAD_S2);
    let target_rate = if heading_error.abs() <= braking_rad + 1.0e-4 {
        0.0
    } else {
        TURN_RATE_RAD_S * heading_error.signum()
    };
    truck.yaw_rate_rad_s = ramp(truck.yaw_rate_rad_s, target_rate, TURN_ACCEL_RAD_S2 * dt_s);
    truck.yaw_rad += truck.yaw_rate_rad_s * dt_s;
    if (goal.yaw_rad - truck.yaw_rad).abs() < 2.0e-3 && truck.yaw_rate_rad_s.abs() < 0.01 {
        truck.yaw_rad = goal.yaw_rad;
        truck.yaw_rate_rad_s = 0.0;
    }

    // The heading is commanded as a rate the solver integrates, plus a small
    // pull toward the commanded yaw. Writing the rotation itself each step
    // jerked the jointed fork round a step at a time, and an unstrapped case
    // slid off the tines 1.4 rad into the first turn.
    let yaw_error_rad = wrap_rad(truck.yaw_rad - yaw_now_rad);
    // With a floor, for the same reason as the drive's: at a few hundredths of
    // a rad/s the base's stiction holds the chassis 0.03 rad off heading.
    let correction_rad_s = if yaw_error_rad.abs() > 0.002 {
        yaw_error_rad.signum() * (3.0 * yaw_error_rad.abs()).clamp(0.06, 0.3)
    } else {
        0.0
    };
    if let Some(mut body) = world.get_mut::<RigidBody>(truck.chassis) {
        body.linear_velocity_m_s.x = truck.velocity_x_m_s;
        body.linear_velocity_m_s.z = truck.velocity_z_m_s;
        body.angular_velocity_rad_s = Vec3::new(0.0, truck.yaw_rate_rad_s + correction_rad_s, 0.0);
    }
    for carriage in [truck.fork, truck.backrest] {
        if let Some(mut motor) = world.get_mut::<JointMotor>(carriage) {
            motor.target_position = goal.mast_m;
        }
    }
}

/// Heading of a truck body, in radians about +y, with the same zero as the
/// commanded yaw.
fn yaw_of(rotation: Quat) -> f64 {
    let facing = rotation * Vec3::X;
    (-facing.z).atan2(facing.x)
}

/// An angle wrapped into [-pi, pi).
fn wrap_rad(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Whether a truck has finished turning to `yaw_rad`: commanded there and
/// actually there.
fn heading_reached(world: &World, truck: &Truck, yaw_rad: f64) -> bool {
    truck.yaw_rad == yaw_rad
        && wrap_rad(yaw_rad - yaw_of(pose(world, truck.chassis).1)).abs() < 0.01
}

/// World-axis half extents of a box after rotation: the box's footprint.
fn world_half_extents(rotation: Quat, half: Vec3) -> Vec3 {
    let axes = [rotation * Vec3::X, rotation * Vec3::Y, rotation * Vec3::Z];
    let along = |pick: fn(Vec3) -> f64| {
        pick(axes[0]).abs() * half.x + pick(axes[1]).abs() * half.y + pick(axes[2]).abs() * half.z
    };
    Vec3::new(along(|v| v.x), along(|v| v.y), along(|v| v.z))
}

/// Whether anything a door leaf could close on is in the car's doorway.
///
/// The doorway is the slab the door leaves slide through, 2 cm deeper on
/// each side, as wide as the open doors and 2.3 m tall above the car floor.
/// A truck standing at the landing with its nose short of the leaves is not in
/// it; a tine or bumper crossing the threshold is.
fn doorway_occupied(world: &World, site: &Site, car_height_m: f64) -> bool {
    let mut boxes = vec![(site.case, CASE_HALF_M)];
    for truck in [&site.lower, &site.upper] {
        boxes.push((truck.chassis, CHASSIS_HALF_M));
        boxes.push((truck.fork, FORK_HALF_M));
        boxes.push((truck.backrest, BACKREST_HALF_M));
    }
    let floor_y_m = car_height_m + CAR_HALF_M.y;
    boxes.into_iter().any(|(entity, half)| {
        let (center, rotation) = pose(world, entity);
        let extent = world_half_extents(rotation, half);
        (center.x - DOORWAY_X_M).abs() < extent.x + DOOR_HALF_M.x + 0.02
            && center.z.abs() < extent.z + 0.6
            && center.y + extent.y > floor_y_m
            && center.y - extent.y < floor_y_m + 2.3
    })
}

/// Feeds the panel's solved contacts to the button and reports a fresh press.
fn read_call_button(
    backend: &mut RapierBackend,
    physics_world: rne_physics::PhysicsWorldId,
    panel: Entity,
    button: &mut CallButton,
) -> bool {
    let contacts: Vec<ButtonContact> = backend
        .contact_points(physics_world)
        .expect("contact points")
        .iter()
        .filter(|sample| sample.entity_a == panel || sample.entity_b == panel)
        .map(|sample| ButtonContact {
            point_world_m: sample.point_world_m,
            normal_force_n: sample.normal_force_n,
        })
        .collect();
    button.update(&contacts);
    button.just_pressed()
}

/// What the ground-floor truck is doing, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Lower {
    Approach,
    Engage,
    Lift,
    Haul,
    Press,
    BackOff,
    TurnRound,
    WaitForDoors,
    LoadCar,
    SetDown,
    Withdraw,
    Parked,
}

/// What the upper-floor truck is doing, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Upper {
    Waiting,
    Unload,
    Lift,
    BackOut,
    TurnRound,
    Place,
    SetDown,
    Withdraw,
    Done,
}

impl Lower {
    fn label(self) -> &'static str {
        match self {
            Lower::Approach => "1F: lining up on goods-in",
            Lower::Engage => "1F: tines under the case",
            Lower::Lift => "1F: lifting",
            Lower::Haul => "1F: hauling to the lift",
            Lower::Press => "1F: pressing the call button",
            Lower::BackOff => "1F: backing off to turn",
            Lower::TurnRound => "1F: turning round",
            Lower::WaitForDoors => "1F: waiting for the doors",
            Lower::LoadCar => "1F: reaching into the car",
            Lower::SetDown => "1F: setting the case on the car stand",
            Lower::Withdraw => "1F: backing out of the car",
            Lower::Parked => "1F: clear",
        }
    }

    fn goal(self) -> DriveGoal {
        let (x_m, z_m, yaw_rad, speed_m_s, mast_m) = match self {
            Lower::Approach => (A_APPROACH_X_M, 0.0, FORKS_OUT_RAD, DRIVE_M_S, FORK_ENGAGE_M),
            Lower::Engage => (A_ENGAGED_X_M, 0.0, FORKS_OUT_RAD, CREEP_M_S, FORK_ENGAGE_M),
            Lower::Lift => (A_ENGAGED_X_M, 0.0, FORKS_OUT_RAD, CREEP_M_S, FORK_CARRY_M),
            // Along the aisle first, then a sidestep onto the button. Arriving
            // diagonally put the chassis's front corner into the button body.
            Lower::Haul => (PRESS_X_M, 0.0, FORKS_OUT_RAD, DRIVE_M_S, FORK_CARRY_M),
            Lower::Press => (PRESS_X_M, PRESS_Z_M, FORKS_OUT_RAD, CREEP_M_S, FORK_CARRY_M),
            Lower::BackOff => (A_TURN_X_M, 0.0, FORKS_OUT_RAD, DRIVE_M_S, FORK_CARRY_M),
            Lower::TurnRound | Lower::WaitForDoors => {
                (A_TURN_X_M, 0.0, FORKS_IN_RAD, DRIVE_M_S, FORK_CARRY_M)
            }
            Lower::LoadCar => (CAR_LOAD_X_M, 0.0, FORKS_IN_RAD, CREEP_M_S, FORK_CARRY_M),
            // Below the seating height, so the case transfers to the legs and
            // the tines come out from under it.
            Lower::SetDown => (
                CAR_LOAD_X_M,
                0.0,
                FORKS_IN_RAD,
                CREEP_M_S,
                FORK_ENGAGE_M - 0.05,
            ),
            Lower::Withdraw | Lower::Parked => (
                A_TURN_X_M,
                0.0,
                FORKS_IN_RAD,
                CREEP_M_S,
                FORK_ENGAGE_M - 0.05,
            ),
        };
        DriveGoal {
            x_m,
            z_m,
            yaw_rad,
            speed_m_s,
            mast_m,
        }
    }
}

impl Upper {
    fn label(self) -> &'static str {
        match self {
            Upper::Waiting => "2F: waiting at the landing",
            Upper::Unload => "2F: tines under the case in the car",
            Upper::Lift => "2F: lifting it off the car stand",
            Upper::BackOut => "2F: backing out of the car",
            Upper::TurnRound => "2F: turning round",
            Upper::Place => "2F: over the outbound bay",
            Upper::SetDown => "2F: setting the case down",
            Upper::Withdraw => "2F: backing clear",
            Upper::Done => "2F: delivered",
        }
    }

    fn goal(self) -> DriveGoal {
        let (x_m, yaw_rad, speed_m_s, mast_m) = match self {
            Upper::Waiting => (B_TURN_X_M, FORKS_IN_RAD, DRIVE_M_S, FORK_ENGAGE_M),
            Upper::Unload => (CAR_LOAD_X_M, FORKS_IN_RAD, CREEP_M_S, FORK_ENGAGE_M),
            Upper::Lift => (CAR_LOAD_X_M, FORKS_IN_RAD, CREEP_M_S, FORK_CARRY_M),
            Upper::BackOut => (B_TURN_X_M, FORKS_IN_RAD, CREEP_M_S, FORK_CARRY_M),
            Upper::TurnRound => (B_TURN_X_M, FORKS_OUT_RAD, DRIVE_M_S, FORK_CARRY_M),
            Upper::Place => (BAY_X_M, FORKS_OUT_RAD, CREEP_M_S, FORK_CARRY_M),
            Upper::SetDown => (BAY_X_M, FORKS_OUT_RAD, CREEP_M_S, FORK_ENGAGE_M - 0.05),
            Upper::Withdraw | Upper::Done => (
                B_WITHDRAW_X_M,
                FORKS_OUT_RAD,
                CREEP_M_S,
                FORK_ENGAGE_M - 0.05,
            ),
        };
        DriveGoal {
            x_m,
            z_m: 0.0,
            yaw_rad,
            speed_m_s,
            mast_m,
        }
    }
}

/// Height of a case resting on a stand whose deck is at `deck_y_m`.
fn seated_case_y_m(deck_y_m: f64) -> f64 {
    deck_y_m + STAND_TOP_Y_M + CASE_HALF_M.y
}

/// Everything a frame needs about one truck.
#[derive(Clone, Copy)]
struct TruckPose {
    chassis: (Vec3, Quat),
    fork: (Vec3, Quat),
    backrest: (Vec3, Quat),
}

fn truck_pose(world: &World, truck: &Truck) -> TruckPose {
    TruckPose {
        chassis: pose(world, truck.chassis),
        fork: pose(world, truck.fork),
        backrest: pose(world, truck.backrest),
    }
}

struct Frame {
    car_y_m: f64,
    door_opening_m: f64,
    button_lit: bool,
    lower: TruckPose,
    upper: TruckPose,
    case: (Vec3, Quat),
}

/// What the run measured, for its own report and gates.
struct Relay {
    frames: Vec<Frame>,
    order: Vec<&'static str>,
    presses: u32,
    lower_lift_m: f64,
    upper_lift_m: f64,
    /// Worst slip of the case across each truck's tines while it carried it.
    lower_slip_m: f64,
    upper_slip_m: f64,
    /// Steps on which a truck or the case was in the doorway while the doors
    /// were not fully open.
    door_conflicts: u32,
    /// Steps on which the car moved with a truck body inside its footprint.
    rides_with_a_truck: u32,
    /// Whether the case was on the car while it travelled.
    case_rode_the_car: bool,
    lower_highest_y_m: f64,
    upper_lowest_y_m: f64,
    case_final: Vec3,
    upper_final_x_m: f64,
    finished_step: usize,
}

/// Tracks how far the case creeps across a truck's tines while it is carried.
fn track_slip(first: &mut Option<Vec3>, worst: &mut f64, case: Vec3, fork: (Vec3, Quat)) {
    // In the fork's own frame, so turning round is not counted as slip.
    let offset = fork.1.conjugate() * (case - fork.0);
    match first {
        None => *first = Some(offset),
        Some(start) => {
            let slip = Vec3::new(offset.x - start.x, 0.0, offset.z - start.z).length();
            *worst = worst.max(slip);
        }
    }
}

/// The measurements a run accumulates step by step.
struct Tally {
    lower_lift_m: f64,
    upper_lift_m: f64,
    lower_slip_m: f64,
    upper_slip_m: f64,
    lower_first: Option<Vec3>,
    upper_first: Option<Vec3>,
    door_conflicts: u32,
    rides_with_a_truck: u32,
    case_rode_the_car: bool,
    lower_highest_y_m: f64,
    upper_lowest_y_m: f64,
}

impl Tally {
    fn new() -> Self {
        Self {
            lower_lift_m: 0.0,
            upper_lift_m: 0.0,
            lower_slip_m: 0.0,
            upper_slip_m: 0.0,
            lower_first: None,
            upper_first: None,
            door_conflicts: 0,
            rides_with_a_truck: 0,
            case_rode_the_car: false,
            lower_highest_y_m: f64::MIN,
            upper_lowest_y_m: f64::MAX,
        }
    }

    fn record(
        &mut self,
        lower: Lower,
        upper: Upper,
        elevator: &Elevator,
        case: Vec3,
        lower_pose: &TruckPose,
        upper_pose: &TruckPose,
    ) {
        let car_deck_y_m = elevator.car_height_m() + CAR_HALF_M.y;
        self.lower_highest_y_m = self.lower_highest_y_m.max(lower_pose.chassis.0.y);
        self.upper_lowest_y_m = self.upper_lowest_y_m.min(upper_pose.chassis.0.y);

        if matches!(elevator.state(), ElevatorState::Moving { .. }) {
            let inside = |pose: &TruckPose| {
                pose.chassis.0.x + CHASSIS_HALF_M.x > DOORWAY_X_M
                    && (pose.chassis.0.y - elevator.car_height_m()).abs() < 1.0
            };
            if inside(lower_pose) || inside(upper_pose) {
                self.rides_with_a_truck += 1;
            }
            if (case.y - seated_case_y_m(car_deck_y_m)).abs() < 0.03
                && (case.x - CAR_STAND_X_M).abs() < 0.2
            {
                self.case_rode_the_car = true;
            }
        }

        if matches!(
            lower,
            Lower::Haul
                | Lower::Press
                | Lower::BackOff
                | Lower::TurnRound
                | Lower::WaitForDoors
                | Lower::LoadCar
        ) {
            track_slip(
                &mut self.lower_first,
                &mut self.lower_slip_m,
                case,
                lower_pose.fork,
            );
        }
        if matches!(lower, Lower::Lift | Lower::Haul) {
            self.lower_lift_m = self
                .lower_lift_m
                .max(case.y - seated_case_y_m(CAR_HALF_M.y));
        }
        if matches!(upper, Upper::BackOut | Upper::TurnRound | Upper::Place) {
            track_slip(
                &mut self.upper_first,
                &mut self.upper_slip_m,
                case,
                upper_pose.fork,
            );
        }
        if matches!(upper, Upper::Lift | Upper::BackOut) {
            self.upper_lift_m = self
                .upper_lift_m
                .max(case.y - seated_case_y_m(car_deck_y_m));
        }
    }
}

/// The ground-floor truck's next phase.
fn advance_lower(
    phase: Lower,
    world: &World,
    site: &Site,
    case: Vec3,
    presses: u32,
    elevator: &Elevator,
) -> Lower {
    let (chassis, _) = pose(world, site.lower.chassis);
    let car_deck_y_m = elevator.car_height_m() + CAR_HALF_M.y;
    match phase {
        Lower::Approach if (chassis.x - A_APPROACH_X_M).abs() < 0.06 => Lower::Engage,
        Lower::Engage if (chassis.x - A_ENGAGED_X_M).abs() < 0.04 => Lower::Lift,
        Lower::Lift if case.y > seated_case_y_m(CAR_HALF_M.y) + 0.12 => Lower::Haul,
        // Stopped, not just arrived: the sidestep onto the button starts from
        // rest.
        Lower::Haul
            if (chassis.x - PRESS_X_M).abs() < 0.02 && site.lower.velocity_x_m_s.abs() < 0.02 =>
        {
            Lower::Press
        }
        Lower::Press if presses > 0 => Lower::BackOff,
        Lower::BackOff if (chassis.x - A_TURN_X_M).abs() < 0.05 && chassis.z.abs() < 0.05 => {
            Lower::TurnRound
        }
        Lower::TurnRound if heading_reached(world, &site.lower, FORKS_IN_RAD) => {
            Lower::WaitForDoors
        }
        Lower::WaitForDoors if elevator.is_boardable(0) => Lower::LoadCar,
        Lower::LoadCar if (chassis.x - CAR_LOAD_X_M).abs() < 0.03 => Lower::SetDown,
        // Down when the case rests on the car stand rather than on the tines.
        Lower::SetDown if case.y < seated_case_y_m(car_deck_y_m) + 0.03 => Lower::Withdraw,
        Lower::Withdraw if chassis.x < CLEAR_OF_DOORWAY_X_M => Lower::Parked,
        other => other,
    }
}

/// The upper-floor truck's next phase.
fn advance_upper(
    phase: Upper,
    world: &World,
    site: &Site,
    case: Vec3,
    dispatched: bool,
    elevator: &Elevator,
) -> Upper {
    let (chassis, _) = pose(world, site.upper.chassis);
    let car_deck_y_m = elevator.car_height_m() + CAR_HALF_M.y;
    match phase {
        Upper::Waiting if dispatched && elevator.is_boardable(1) => Upper::Unload,
        Upper::Unload if (chassis.x - CAR_LOAD_X_M).abs() < 0.03 => Upper::Lift,
        Upper::Lift if case.y > seated_case_y_m(car_deck_y_m) + 0.12 => Upper::BackOut,
        Upper::BackOut if (chassis.x - B_TURN_X_M).abs() < 0.05 => Upper::TurnRound,
        Upper::TurnRound if heading_reached(world, &site.upper, FORKS_OUT_RAD) => Upper::Place,
        Upper::Place if (chassis.x - BAY_X_M).abs() < 0.03 => Upper::SetDown,
        Upper::SetDown if case.y < seated_case_y_m(FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y) + 0.03 => {
            Upper::Withdraw
        }
        Upper::Withdraw if chassis.x > B_WITHDRAW_X_M - 0.05 => Upper::Done,
        other => other,
    }
}

/// Runs the relay. With `sample_every`, captures a frame every that many steps.
fn run_relay(sample_every: Option<usize>, trace: bool) -> Relay {
    let spec = elevator_spec();
    spec.validate().expect("elevator specification");
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let mut site = spawn_site(&mut world);
    let mut elevator = Elevator::new(spec.clone(), 0).expect("elevator");
    apply_elevator(&mut world, &site, &elevator);
    let mut button = CallButton::new(CallButtonSpec {
        center_world_m: BUTTON_CENTER_M,
        normal_world: BUTTON_NORMAL,
        radius_m: 0.10,
        travel_m: 0.02,
        press_force_n: 2.0,
        release_force_n: 1.0,
        floor: 0,
    })
    .expect("call button");

    let dt = SimDuration::from_hertz(Hertz::new(PHYSICS_HZ));
    let dt_s = 1.0 / PHYSICS_HZ;
    let steps = (MAX_SECONDS * PHYSICS_HZ) as usize;
    for _ in 0..(PHYSICS_HZ as usize / 2) {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("settle");
    }

    let mut lower = Lower::Approach;
    let mut upper = Upper::Waiting;
    let mut order = vec![lower.label(), upper.label()];
    let mut presses = 0_u32;
    let mut dispatched = false;
    let mut tally = Tally::new();
    let mut frames = Vec::new();
    let mut finished_step = steps;

    for step in 0..steps {
        drive_truck(&mut world, &mut site.lower, lower.goal(), dt_s);
        drive_truck(&mut world, &mut site.upper, upper.goal(), dt_s);

        if read_call_button(&mut backend, physics_world, site.button, &mut button) {
            presses += 1;
            elevator.call(0).expect("summon the car to the lobby");
        }
        // Doors lost to the dwell while the loaded truck turned round are
        // what an operator presses the button again for. The call is
        // idempotent and only made while the car is parked here.
        if matches!(lower, Lower::WaitForDoors) && !elevator.is_boardable(0) && !dispatched {
            elevator.call(0).expect("summon the car again");
        }
        let occupied = doorway_occupied(&world, &site, elevator.car_height_m());
        if occupied {
            elevator.hold_doors();
            if elevator.door_opening_m() < spec.door_travel_m - 1.0e-9 {
                tally.door_conflicts += 1;
            }
        }

        elevator.update(dt_s).expect("elevator update");
        apply_elevator(&mut world, &site, &elevator);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");

        let (case, _) = pose(&world, site.case);
        let lower_pose = truck_pose(&world, &site.lower);
        let upper_pose = truck_pose(&world, &site.upper);
        tally.record(lower, upper, &elevator, case, &lower_pose, &upper_pose);

        lower = advance_lower(lower, &world, &site, case, presses, &elevator);
        // Clear of the doorway: send the car up with the case alone.
        if matches!(lower, Lower::Parked) && !dispatched {
            elevator.call(1).expect("dispatch the car to 2F");
            dispatched = true;
        }
        upper = advance_upper(upper, &world, &site, case, dispatched, &elevator);

        for label in [lower.label(), upper.label()] {
            if !order.contains(&label) {
                order.push(label);
            }
        }
        if trace && step % 240 == 0 {
            eprintln!(
                "t={:6.2} {:40} {:36} car={:+.2} doors={:.2} case=({:+.3},{:+.3},{:+.3}) 1F=({:+.3},{:+.3} yaw {:+.2}) 2F=({:+.3},{:+.3} yaw {:+.2})",
                step as f64 / PHYSICS_HZ,
                lower.label(),
                upper.label(),
                elevator.car_height_m(),
                elevator.door_opening_m(),
                case.x,
                case.y,
                case.z,
                lower_pose.chassis.0.x,
                lower_pose.chassis.0.z,
                site.lower.yaw_rad,
                upper_pose.chassis.0.x,
                upper_pose.chassis.0.z,
                site.upper.yaw_rad,
            );
        }
        if let Some(every) = sample_every {
            if step % every == 0 && frames.len() < FRAME_COUNT {
                frames.push(Frame {
                    car_y_m: elevator.car_height_m(),
                    door_opening_m: elevator.door_opening_m(),
                    button_lit: presses > 0 && !dispatched,
                    lower: lower_pose,
                    upper: upper_pose,
                    case: pose(&world, site.case),
                });
            }
        }
        if matches!(upper, Upper::Done) && finished_step == steps {
            finished_step = step;
        }
        let captured = sample_every.is_none_or(|_| frames.len() >= FRAME_COUNT);
        if finished_step < steps && captured {
            break;
        }
    }

    assert_eq!(
        upper,
        Upper::Done,
        "the relay did not finish: 1F {lower:?}, 2F {upper:?}"
    );
    Relay {
        frames,
        order,
        presses,
        lower_lift_m: tally.lower_lift_m,
        upper_lift_m: tally.upper_lift_m,
        lower_slip_m: tally.lower_slip_m,
        upper_slip_m: tally.upper_slip_m,
        door_conflicts: tally.door_conflicts,
        rides_with_a_truck: tally.rides_with_a_truck,
        case_rode_the_car: tally.case_rode_the_car,
        lower_highest_y_m: tally.lower_highest_y_m,
        upper_lowest_y_m: tally.upper_lowest_y_m,
        case_final: pose(&world, site.case).0,
        upper_final_x_m: pose(&world, site.upper.chassis).0.x,
        finished_step,
    }
}

/// Equirectangular environment map width and height, in pixels.
///
/// Small on purpose: it is only ever used as a light source, and the backend
/// prefilters it into irradiance and specular mips before anything samples it.
const ENVIRONMENT_W: u32 = 128;
const ENVIRONMENT_H: u32 = 64;

/// Builds the light this warehouse is lit by.
///
/// Without one, every surface facing away from the single directional light
/// renders black -- the truck came out a silhouette and no amount of tuning its
/// colours fixed it, because the problem was that there was nothing else to
/// light it. This is a synthesised interior rather than a photograph: a bright
/// ceiling carrying the fixture bands, mid-tone walls and a dark floor, which
/// is what a warehouse actually is from a surface's point of view.
fn warehouse_environment() -> EnvironmentLighting {
    let mut rgba32f = Vec::with_capacity((ENVIRONMENT_W * ENVIRONMENT_H * 4) as usize);
    for row in 0..ENVIRONMENT_H {
        // 0 at the zenith, 1 at the nadir.
        let down = (row as f32 + 0.5) / ENVIRONMENT_H as f32;
        for column in 0..ENVIRONMENT_W {
            let around = (column as f32 + 0.5) / ENVIRONMENT_W as f32;
            let (r, g, b) = if down < 0.30 {
                // Ceiling. Four fixture runs, bright enough to read as sources.
                let bands = (around * std::f32::consts::TAU * 4.0).sin().max(0.0);
                let fixture = bands.powf(18.0);
                let base = 0.75 + 0.85 * (1.0 - down / 0.30);
                (
                    base + 4.5 * fixture,
                    base + 4.5 * fixture,
                    base * 1.03 + 4.3 * fixture,
                )
            } else if down < 0.62 {
                // Walls, falling off toward the floor.
                let t = (down - 0.30) / 0.32;
                let level = 0.52 - 0.26 * t;
                (level * 0.94, level * 0.97, level)
            } else {
                // Floor bounce: dim, and slightly warm from the concrete.
                (0.15, 0.145, 0.135)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    let map = EnvironmentMap::from_rgba32f(ENVIRONMENT_W, ENVIRONMENT_H, rgba32f)
        .expect("warehouse environment map");
    EnvironmentLighting {
        map: Some(Arc::new(map)),
        intensity: 1.0,
        // Above the defaults (0.35 / 0.25): this map is the scene's fill light,
        // not a subtle tint on top of one. Not far above, though -- at 0.95 the
        // decks blew out to white.
        diffuse_strength: 0.72,
        specular_strength: 0.38,
        rotation_rad: 0.0,
    }
}

/// The PBR maps the warehouse surfaces are built from.
struct Surfaces {
    concrete: Arc<ImageFrame>,
    concrete_normal: Arc<ImageFrame>,
    concrete_roughness: Arc<ImageFrame>,
}

fn surfaces() -> &'static Surfaces {
    static SURFACES: OnceLock<Surfaces> = OnceLock::new();
    SURFACES.get_or_init(|| {
        // Shared with the photoreal test bay rather than copied: a warehouse
        // deck and a test-bay floor are the same poured concrete.
        let assets = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("examples directory")
            .join("63_g1_stride_gif/assets/photoreal_test_bay");
        Surfaces {
            concrete: load_texture(&assets.join("concrete_floor_basecolor.png")),
            concrete_normal: load_texture(&assets.join("concrete_floor_normal.png")),
            concrete_roughness: load_texture(&assets.join("concrete_floor_roughness.png")),
        }
    })
}

fn load_texture(path: &Path) -> Arc<ImageFrame> {
    let rgba = image::open(path)
        .unwrap_or_else(|error| panic!("load render texture {}: {error}", path.display()))
        .into_rgba8();
    Arc::new(ImageFrame::from_rgba8(
        rgba.width(),
        rgba.height(),
        rgba.into_raw(),
    ))
}

/// A horizontal slab of poured concrete, tiled one texture repeat per 1.5 m.
fn push_concrete_deck(scene: &mut RenderScene, center: Vec3, half: Vec3) {
    let surfaces = surfaces();
    let (half_x, half_z) = (half.x as f32, half.z as f32);
    let (repeat_x, repeat_z) = ((half.x / 0.75) as f32, (half.z / 0.75) as f32);
    let mesh = TriangleMesh {
        positions: vec![
            [-half_x, 0.0, -half_z],
            [half_x, 0.0, -half_z],
            [half_x, 0.0, half_z],
            [-half_x, 0.0, half_z],
        ],
        normals: vec![[0.0, 1.0, 0.0]; 4],
        texcoords: vec![
            [0.0, 0.0],
            [repeat_x, 0.0],
            [repeat_x, repeat_z],
            [0.0, repeat_z],
        ],
        // Counter-clockwise seen from above. The obvious order winds the other
        // way, which puts the outward normal at -Y and lets back-face culling
        // delete the floor.
        indices: vec![0, 2, 1, 0, 3, 2],
        skinning: None,
    };
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: center + Vec3::new(0.0, half.y, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: [1.0; 4],
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&surfaces.concrete)),
        material: PbrMaterial::new([1.0; 4], 0.92, 0.0, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&surfaces.concrete_normal)),
            Some(Arc::clone(&surfaces.concrete_roughness)),
        ),
    });
    // The slab's own edge, so the deck reads as a thickness rather than a
    // sheet of paper when the camera is nearly level with it. Dropped a few
    // millimetres: its top face is otherwise coplanar with the textured quad
    // and wins the depth test about half the time, which hides the concrete.
    push_pbr(
        scene,
        center - Vec3::new(0.0, 0.006, 0.0),
        half,
        [0.42, 0.43, 0.45, 1.0],
        0.95,
        0.0,
        [0.0; 3],
    );
}

/// A box with real surface parameters rather than the default material.
fn push_pbr(
    scene: &mut RenderScene,
    translation: Vec3,
    half: Vec3,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    push_pbr_rotated(
        scene,
        translation,
        Quat::IDENTITY,
        half,
        color,
        roughness,
        metallic,
        emissive,
    );
}

/// Painted steel: the colour the caller asked for, with a sheen.
///
/// Metallic stays low. There is no image-based lighting in this shot, so a
/// surface with nothing to reflect renders as its reflections -- that is, as
/// black. At 0.65 the whole truck came out a silhouette.
fn push_steel(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    half: Vec3,
    color: [f32; 4],
) {
    push_pbr_rotated(
        scene,
        translation,
        rotation,
        half,
        color,
        0.45,
        0.08,
        [0.0; 3],
    );
}

/// A box with real surface parameters, at any orientation.
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn push_pbr_rotated(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    half: Vec3,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    // Through `item_from_visual`, which folds the box size into the transform;
    // a hand-built item with unit scale draws every box as a 1 m cube.
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(translation, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    scene.items.push(item);
}

/// A stringer pallet drawn under a case, turned with it.
fn push_pallet(scene: &mut RenderScene, case: (Vec3, Quat)) {
    const WOOD: [f32; 4] = [0.60, 0.45, 0.28, 1.0];
    const BOARD_HALF_Y: f64 = 0.012;
    const BLOCK_HALF_Y: f64 = 0.045;
    let (center, rotation) = case;
    let base = center - rotation * Vec3::new(0.0, CASE_HALF_M.y + 0.114, 0.0);
    for offset in [-0.20, 0.0, 0.20] {
        push_pbr_rotated(
            scene,
            base + rotation * Vec3::new(0.0, BLOCK_HALF_Y, offset),
            rotation,
            Vec3::new(0.055, BLOCK_HALF_Y, 0.055),
            [0.50, 0.37, 0.22, 1.0],
            0.95,
            0.0,
            [0.0; 3],
        );
    }
    for offset in [-0.21, 0.0, 0.21] {
        push_pbr_rotated(
            scene,
            base + rotation * Vec3::new(0.0, 2.0 * BLOCK_HALF_Y + BOARD_HALF_Y, offset),
            rotation,
            Vec3::new(0.20, BOARD_HALF_Y, 0.055),
            WOOD,
            0.9,
            0.0,
            [0.0; 3],
        );
    }
}

/// Scanned CC0 props from Poly Haven, fetched by
/// `tools/prepare_polyhaven_warehouse.py` (licence and hashes in
/// `assets/props/polyhaven_warehouse/manifest.json`).
fn props_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_warehouse")
}

/// A scanned prop. `path` is relative to [`props_root`]; the prop's own origin
/// is its base, so `base` is where it stands.
fn push_prop(scene: &mut RenderScene, path: &str, base: Vec3, yaw_rad: f64, scale: Vec3) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(base, Quat::from_rotation_y(yaw_rad)),
        VisualShape::Mesh {
            path: path.to_string(),
            scale,
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

const CARDBOARD_BOX: &str = "cardboard_box_01/cardboard_box_01_1k.gltf";
const WOODEN_CRATE: &str = "wooden_crate_01/wooden_crate_01_1k.gltf";

/// A cylinder with its axis along `axis` (a unit vector in world space).
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn push_cylinder(
    scene: &mut RenderScene,
    center: Vec3,
    axis: Vec3,
    radius_m: f64,
    length_m: f64,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center, Quat::from_rotation_arc(Vec3::Z, axis)),
        VisualShape::Cylinder { radius_m, length_m },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    scene.items.push(item);
}

/// An autonomous forklift, drawn around the three bodies physics solves.
///
/// Chassis, fork carriage and backrest are the physical shapes; everything
/// else -- frame, counterweight, mast, chains, cylinders, wheels, sensors and
/// lights -- is placed from their poses so it can never disagree with them.
/// It is an AGV rather than a ride-on truck, so there is no seat or overhead
/// guard: a sensor tower with a LiDAR, corner safety scanners, a status light
/// strip and the blue spot such trucks project on the floor ahead of the forks.
/// Colours shared by the truck's parts.
const TRUCK_FRAME: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
const TRUCK_STEEL: [f32; 4] = [0.46, 0.48, 0.52, 1.0];
const TRUCK_CHROME: [f32; 4] = [0.78, 0.80, 0.84, 1.0];
const TRUCK_TYRE: [f32; 4] = [0.07, 0.07, 0.08, 1.0];
const TRUCK_RUBBER: [f32; 4] = [0.10, 0.10, 0.11, 1.0];
const TRUCK_TINE: [f32; 4] = [0.22, 0.23, 0.25, 1.0];
const TRUCK_HAZARD: [f32; 4] = [0.95, 0.78, 0.10, 1.0];

/// A truck's chassis frame: where its parts are placed from.
struct TruckFrame {
    chassis: Vec3,
    rotation: Quat,
    x_axis: Vec3,
    y_axis: Vec3,
    z_axis: Vec3,
    floor_y: f64,
}

impl TruckFrame {
    fn of(truck: &TruckPose) -> Self {
        let (chassis, rotation) = truck.chassis;
        Self {
            chassis,
            rotation,
            x_axis: rotation * Vec3::X,
            y_axis: rotation * Vec3::Y,
            z_axis: rotation * Vec3::Z,
            floor_y: chassis.y - CHASSIS_HALF_M.y - 0.07,
        }
    }

    fn at(&self, offset: Vec3) -> Vec3 {
        self.chassis + self.rotation * offset
    }
}

fn push_truck(scene: &mut RenderScene, truck: &TruckPose, bodywork: [f32; 4]) {
    push_truck_body(scene, truck, bodywork);
    push_truck_mast(scene, truck, bodywork);
    push_truck_load_handling(scene, truck, bodywork);
}

/// Frame, body, counterweight, wheels, sensors and lights.
#[allow(clippy::too_many_lines)] // One part list; splitting it further only scatters it.
fn push_truck_body(scene: &mut RenderScene, truck: &TruckPose, bodywork: [f32; 4]) {
    let frame = TruckFrame::of(truck);
    let at = |offset: Vec3| frame.at(offset);
    let rotation = frame.rotation;
    let x_axis = frame.x_axis;
    let y_axis = frame.y_axis;
    let z_axis = frame.z_axis;
    let shade = |factor: f32| {
        [
            bodywork[0] * factor,
            bodywork[1] * factor,
            bodywork[2] * factor,
            1.0,
        ]
    };
    // Frame, body and the rounded counterweight at the rear.
    push_pbr_rotated(
        scene,
        at(Vec3::new(0.0, -0.19, 0.0)),
        rotation,
        Vec3::new(0.50, 0.07, 0.29),
        TRUCK_FRAME,
        0.7,
        0.2,
        [0.0; 3],
    );
    push_pbr_rotated(
        scene,
        at(Vec3::new(-0.04, 0.03, 0.0)),
        rotation,
        Vec3::new(0.36, 0.15, 0.30),
        bodywork,
        0.38,
        0.08,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        at(Vec3::new(0.28, -0.01, 0.0)),
        z_axis,
        0.24,
        0.60,
        shade(0.82),
        0.42,
        0.08,
        [0.0; 3],
    );
    push_pbr_rotated(
        scene,
        at(Vec3::new(0.26, 0.19, 0.0)),
        rotation,
        Vec3::new(0.24, 0.01, 0.28),
        shade(0.82),
        0.42,
        0.08,
        [0.0; 3],
    );
    // Yellow-and-black hazard band across the counterweight's face.
    for stripe in 0..6 {
        let z = -0.25 + f64::from(stripe) * 0.1;
        let color = if stripe % 2 == 0 {
            TRUCK_HAZARD
        } else {
            TRUCK_FRAME
        };
        push_pbr_rotated(
            scene,
            at(Vec3::new(0.525, -0.08, z)),
            rotation,
            Vec3::new(0.005, 0.03, 0.05),
            color,
            0.6,
            0.0,
            [0.0; 3],
        );
    }
    push_pbr_rotated(
        scene,
        at(Vec3::new(-0.04, 0.185, 0.0)),
        rotation,
        Vec3::new(0.34, 0.006, 0.27),
        TRUCK_RUBBER,
        0.95,
        0.0,
        [0.0; 3],
    );
    // Status light strips along both flanks.
    for side in [-1.0, 1.0] {
        push_pbr_rotated(
            scene,
            at(Vec3::new(-0.04, 0.12, side * 0.302)),
            rotation,
            Vec3::new(0.30, 0.012, 0.004),
            [0.2, 0.9, 0.5, 1.0],
            0.3,
            0.0,
            [0.25, 1.1, 0.55],
        );
    }
    // Wheels: drive wheels under the counterweight, load wheels under the mast.
    for (x, radius) in [(0.30, 0.105), (-0.36, 0.085)] {
        for side in [-1.0, 1.0] {
            let center = at(Vec3::new(x, -0.26 + radius - 0.035, side * 0.29));
            push_cylinder(
                scene, center, z_axis, radius, 0.08, TRUCK_TYRE, 0.9, 0.0, [0.0; 3],
            );
            push_cylinder(
                scene,
                center + z_axis * (side * 0.004),
                z_axis,
                radius * 0.55,
                0.082,
                TRUCK_STEEL,
                0.35,
                0.6,
                [0.0; 3],
            );
        }
    }
    // Corner safety scanners, front and rear.
    for (x, face) in [(-0.50, -1.0), (0.52, 1.0)] {
        for side in [-1.0, 1.0] {
            let center = at(Vec3::new(x, -0.12, side * 0.24));
            push_pbr_rotated(
                scene,
                center,
                rotation,
                Vec3::new(0.035, 0.035, 0.045),
                TRUCK_FRAME,
                0.5,
                0.1,
                [0.0; 3],
            );
            push_pbr_rotated(
                scene,
                center + x_axis * (face * 0.036),
                rotation,
                Vec3::new(0.002, 0.018, 0.03),
                TRUCK_HAZARD,
                0.4,
                0.0,
                [0.35, 0.28, 0.02],
            );
        }
    }
    // Sensor tower: post, LiDAR puck with its lit ring, amber beacon.
    push_cylinder(
        scene,
        at(Vec3::new(0.34, 0.45, 0.0)),
        y_axis,
        0.028,
        0.52,
        TRUCK_STEEL,
        0.35,
        0.6,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        at(Vec3::new(0.34, 0.74, 0.0)),
        y_axis,
        0.062,
        0.07,
        TRUCK_FRAME,
        0.4,
        0.3,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        at(Vec3::new(0.34, 0.735, 0.0)),
        y_axis,
        0.064,
        0.012,
        [0.3, 0.8, 1.0, 1.0],
        0.3,
        0.0,
        [0.25, 0.8, 1.1],
    );
    push_cylinder(
        scene,
        at(Vec3::new(0.34, 0.80, 0.0)),
        y_axis,
        0.035,
        0.05,
        [1.0, 0.62, 0.1, 1.0],
        0.3,
        0.0,
        [1.2, 0.65, 0.08],
    );
}

/// Mast channels, top tie, lift cylinder, chains and tilt cylinders.
#[allow(clippy::too_many_lines)] // One part list; splitting it further only scatters it.
fn push_truck_mast(scene: &mut RenderScene, truck: &TruckPose, _bodywork: [f32; 4]) {
    let frame = TruckFrame::of(truck);
    let at = |offset: Vec3| frame.at(offset);
    let rotation = frame.rotation;
    let x_axis = frame.x_axis;
    let y_axis = frame.y_axis;
    // Mast: outer and inner channels, top tie, lift cylinder, chains.
    let mast_x = FORK_REACH_M + FORK_HALF_M.x + 0.05;
    for side in [-1.0, 1.0] {
        push_pbr_rotated(
            scene,
            at(Vec3::new(mast_x, 0.38, side * 0.20)),
            rotation,
            Vec3::new(0.04, 0.64, 0.02),
            TRUCK_STEEL,
            0.45,
            0.55,
            [0.0; 3],
        );
        push_pbr_rotated(
            scene,
            at(Vec3::new(mast_x - 0.005, 0.38, side * 0.175)),
            rotation,
            Vec3::new(0.035, 0.64, 0.006),
            TRUCK_FRAME,
            0.6,
            0.3,
            [0.0; 3],
        );
        push_pbr_rotated(
            scene,
            at(Vec3::new(mast_x - 0.03, 0.34, side * 0.15)),
            rotation,
            Vec3::new(0.02, 0.60, 0.014),
            TRUCK_CHROME,
            0.25,
            0.8,
            [0.0; 3],
        );
        push_pbr_rotated(
            scene,
            at(Vec3::new(mast_x - 0.045, 0.34, side * 0.075)),
            rotation,
            Vec3::new(0.006, 0.55, 0.012),
            TRUCK_FRAME,
            0.8,
            0.4,
            [0.0; 3],
        );
        // Tilt cylinders from the body to the mast.
        push_cylinder(
            scene,
            at(Vec3::new(-0.46, 0.08, side * 0.24)),
            x_axis,
            0.022,
            0.16,
            TRUCK_CHROME,
            0.25,
            0.8,
            [0.0; 3],
        );
    }
    push_pbr_rotated(
        scene,
        at(Vec3::new(mast_x, 1.02, 0.0)),
        rotation,
        Vec3::new(0.045, 0.03, 0.22),
        TRUCK_STEEL,
        0.45,
        0.55,
        [0.0; 3],
    );
    push_pbr_rotated(
        scene,
        at(Vec3::new(mast_x, -0.20, 0.0)),
        rotation,
        Vec3::new(0.045, 0.03, 0.22),
        TRUCK_STEEL,
        0.45,
        0.55,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        at(Vec3::new(mast_x + 0.02, 0.30, 0.0)),
        y_axis,
        0.032,
        0.95,
        TRUCK_CHROME,
        0.2,
        0.85,
        [0.0; 3],
    );
}

/// Load backrest, tines and the warning spot ahead of them.
#[allow(clippy::too_many_lines)] // One part list; splitting it further only scatters it.
fn push_truck_load_handling(scene: &mut RenderScene, truck: &TruckPose, _bodywork: [f32; 4]) {
    let frame = TruckFrame::of(truck);
    let x_axis = frame.x_axis;
    // Carriage: a lattice load backrest where the physical backrest is.
    let (backrest, backrest_rotation) = truck.backrest;
    let rest = |offset: Vec3| backrest + backrest_rotation * offset;
    for bar in 0..5 {
        let z = -0.20 + f64::from(bar) * 0.1;
        push_pbr_rotated(
            scene,
            rest(Vec3::new(0.0, 0.0, z)),
            backrest_rotation,
            Vec3::new(0.012, BACKREST_HALF_M.y, 0.012),
            TRUCK_TINE,
            0.5,
            0.4,
            [0.0; 3],
        );
    }
    for y in [-BACKREST_HALF_M.y, BACKREST_HALF_M.y, 0.0] {
        push_pbr_rotated(
            scene,
            rest(Vec3::new(0.0, y, 0.0)),
            backrest_rotation,
            Vec3::new(0.018, 0.016, BACKREST_HALF_M.z),
            TRUCK_TINE,
            0.5,
            0.4,
            [0.0; 3],
        );
    }
    // L-shaped tines: the physical blade plus its shank up the carriage.
    let (fork, fork_rotation) = truck.fork;
    for side in [-1.0, 1.0] {
        push_pbr_rotated(
            scene,
            fork + fork_rotation * Vec3::new(0.0, 0.0, side * 0.085),
            fork_rotation,
            Vec3::new(FORK_HALF_M.x, FORK_HALF_M.y, 0.045),
            TRUCK_TINE,
            0.45,
            0.5,
            [0.0; 3],
        );
        push_pbr_rotated(
            scene,
            fork + fork_rotation * Vec3::new(FORK_HALF_M.x - 0.02, 0.18, side * 0.085),
            fork_rotation,
            Vec3::new(0.02, 0.18, 0.045),
            TRUCK_TINE,
            0.45,
            0.5,
            [0.0; 3],
        );
    }
    // The blue warning spot on the floor ahead of the forks.
    let spot = frame.chassis + x_axis * -1.75;
    push_cylinder(
        scene,
        Vec3::new(spot.x, frame.floor_y + 0.0025, spot.z),
        Vec3::Y,
        0.12,
        0.002,
        [0.2, 0.45, 1.0, 1.0],
        0.3,
        0.0,
        [0.25, 0.55, 1.4],
    );
}

/// A PBR texture set: colour, OpenGL normal and roughness maps.
struct TextureSet {
    color: Arc<ImageFrame>,
    normal: Arc<ImageFrame>,
    roughness: Arc<ImageFrame>,
}

fn texture_set(name: &str) -> TextureSet {
    let root = props_root().join("textures");
    TextureSet {
        color: load_texture(&root.join(format!("{name}_diff_1k.jpg"))),
        normal: load_texture(&root.join(format!("{name}_nor_gl_1k.jpg"))),
        roughness: load_texture(&root.join(format!("{name}_rough_1k.jpg"))),
    }
}

/// Profiled steel cladding for the warehouse walls.
fn wall_textures() -> &'static TextureSet {
    static WALLS: OnceLock<TextureSet> = OnceLock::new();
    WALLS.get_or_init(|| texture_set("box_profile_metal_sheet"))
}

/// A textured rectangle facing `u x v`, spanning `+-u` and `+-v` about
/// `center`, one texture repeat every `repeat_m` meters.
fn push_textured_panel(
    scene: &mut RenderScene,
    center: Vec3,
    u: Vec3,
    v: Vec3,
    textures: &TextureSet,
    repeat_m: f64,
    tint: [f32; 4],
) {
    let (repeat_u, repeat_v) = (
        (2.0 * u.length() / repeat_m) as f32,
        (2.0 * v.length() / repeat_m) as f32,
    );
    let normal = u.cross(v).normalize();
    let corner = |a: f64, b: f64| {
        let point = u * a + v * b;
        [point.x as f32, point.y as f32, point.z as f32]
    };
    let mesh = TriangleMesh {
        positions: vec![
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ],
        normals: vec![[normal.x as f32, normal.y as f32, normal.z as f32]; 4],
        texcoords: vec![
            [0.0, repeat_v],
            [repeat_u, repeat_v],
            [repeat_u, 0.0],
            [0.0, 0.0],
        ],
        // Counter-clockwise seen from the side `normal` points to.
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: tint,
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&textures.color)),
        material: PbrMaterial::new(tint, 0.7, 0.25, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&textures.normal)),
            Some(Arc::clone(&textures.roughness)),
        ),
    });
}

/// One stand leg, drawn as the welded frame it stands for: two square posts,
/// cross bracing, and a top plate with a hazard-striped edge. The physics leg
/// is the solid box this frame fills.
fn push_stand_leg(scene: &mut RenderScene, center: Vec3) {
    const POST: [f32; 4] = [0.30, 0.33, 0.38, 1.0];
    const PLATE: [f32; 4] = [0.52, 0.54, 0.58, 1.0];
    const HAZARD: [f32; 4] = [0.95, 0.78, 0.10, 1.0];
    const DARK: [f32; 4] = [0.12, 0.12, 0.13, 1.0];
    let half = STAND_LEG_HALF_M;
    for dx in [-1.0, 1.0] {
        push_pbr(
            scene,
            center + Vec3::new(dx * (half.x - 0.02), -0.01, 0.0),
            Vec3::new(0.02, half.y - 0.01, half.z - 0.01),
            POST,
            0.5,
            0.4,
            [0.0; 3],
        );
    }
    for y in [-0.6, 0.1] {
        push_pbr(
            scene,
            center + Vec3::new(0.0, y * half.y, 0.0),
            Vec3::new(half.x - 0.02, 0.012, half.z - 0.02),
            POST,
            0.5,
            0.4,
            [0.0; 3],
        );
    }
    push_pbr(
        scene,
        center + Vec3::new(0.0, half.y - 0.008, 0.0),
        Vec3::new(half.x, 0.008, half.z),
        PLATE,
        0.45,
        0.5,
        [0.0; 3],
    );
    for stripe in 0..5 {
        let x = -half.x + 0.02 + f64::from(stripe) * (2.0 * half.x - 0.04) / 4.0;
        let color = if stripe % 2 == 0 { HAZARD } else { DARK };
        for dz in [-1.0, 1.0] {
            push_pbr(
                scene,
                center + Vec3::new(x, half.y - 0.03, dz * (half.z + 0.001)),
                Vec3::new(0.02, 0.02, 0.001),
                color,
                0.6,
                0.0,
                [0.0; 3],
            );
        }
    }
}

/// The car's cab: back and side walls in brushed steel, a handrail, and a lit
/// ceiling panel. The open side faces the camera, as the shaft's does.
fn push_car_interior(scene: &mut RenderScene, car_y_m: f64, handrail: bool) {
    const PANEL: [f32; 4] = [0.66, 0.68, 0.72, 1.0];
    const TRIM: [f32; 4] = [0.30, 0.31, 0.34, 1.0];
    let floor = car_y_m + CAR_HALF_M.y;
    let wall_half_y = 1.1;
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M + CAR_HALF_M.x - 0.02, floor + wall_half_y, 0.0),
        Vec3::new(0.02, wall_half_y, CAR_HALF_M.z),
        PANEL,
        0.3,
        0.75,
        [0.0; 3],
    );
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M + 0.04, floor + wall_half_y, -CAR_HALF_M.z + 0.02),
        Vec3::new(CAR_HALF_M.x - 0.04, wall_half_y, 0.02),
        PANEL,
        0.3,
        0.75,
        [0.0; 3],
    );
    for panel in 0..3 {
        let z = -0.6 + f64::from(panel) * 0.6;
        push_pbr(
            scene,
            Vec3::new(
                SHAFT_X_M + CAR_HALF_M.x - 0.041,
                floor + wall_half_y,
                z + 0.3,
            ),
            Vec3::new(0.001, wall_half_y - 0.02, 0.004),
            TRIM,
            0.5,
            0.4,
            [0.0; 3],
        );
    }
    if handrail {
    push_cylinder(
        scene,
        Vec3::new(SHAFT_X_M + CAR_HALF_M.x - 0.09, floor + 0.9, 0.0),
        Vec3::Z,
        0.018,
        1.6,
        [0.82, 0.84, 0.88, 1.0],
        0.2,
        0.9,
        [0.0; 3],
    );
    }
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M + 0.05, floor + 2.2, -0.2),
        Vec3::new(0.45, 0.01, 0.35),
        [0.95, 0.96, 1.0, 1.0],
        0.3,
        0.0,
        [0.8, 0.82, 0.88],
    );
    // Hazard edge along the threshold.
    push_pbr(
        scene,
        Vec3::new(DOORWAY_X_M + 0.03, floor + 0.001, 0.0),
        Vec3::new(0.03, 0.001, CAR_HALF_M.z - 0.05),
        [0.95, 0.78, 0.10, 1.0],
        0.6,
        0.0,
        [0.0; 3],
    );
}

/// Warehouse dressing from the scanned CC0 props, kept clear of both trucks'
/// routes and turning circles.
fn push_floor_props(scene: &mut RenderScene) {
    let ground = CAR_HALF_M.y;
    let upper = FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y;
    // Goods-in end of the ground floor.
    push_prop(
        scene,
        "hand_truck/hand_truck_1k.gltf",
        Vec3::new(-6.3, ground, 1.05),
        -0.5,
        Vec3::ONE,
    );
    push_prop(
        scene,
        "industrial_storage_cart/industrial_storage_cart_1k.gltf",
        Vec3::new(-7.9, ground, 0.75),
        1.57,
        Vec3::ONE,
    );
    push_prop(
        scene,
        CARDBOARD_BOX,
        Vec3::new(-7.75, ground + 0.72, 0.62),
        0.2,
        Vec3::ONE,
    );
    push_prop(
        scene,
        "WetFloorSign_01/WetFloorSign_01_1k.gltf",
        Vec3::new(-5.4, ground, 1.25),
        0.4,
        Vec3::ONE,
    );
    // By the lift: extinguisher on the floor, the distribution board on the
    // back wall.
    push_prop(
        scene,
        "korean_fire_extinguisher_01/korean_fire_extinguisher_01_1k.gltf",
        Vec3::new(-1.45, ground, -1.12),
        0.3,
        Vec3::ONE,
    );
    push_prop(
        scene,
        "power_box_01/power_box_01_1k.gltf",
        Vec3::new(-2.0, 1.35, -1.24),
        0.0,
        Vec3::ONE,
    );
    // Upper floor: the shipping door the delivered case is bound for, and a
    // shelf unit of boxed stock by the lift.
    push_prop(
        scene,
        "rollershutter_door/rollershutter_door_1k.gltf",
        Vec3::new(-3.3, upper, -1.28),
        0.0,
        Vec3::ONE,
    );
    push_prop(
        scene,
        "steel_frame_shelves_02/steel_frame_shelves_02_1k.gltf",
        Vec3::new(-1.45, upper, -1.05),
        0.0,
        Vec3::new(1.0, 0.72, 1.0),
    );
    for (level, y) in [0.0_f64, 0.54, 1.08].iter().enumerate() {
        push_prop(
            scene,
            CARDBOARD_BOX,
            Vec3::new(-1.45, upper + 0.03 + y * 0.72, -1.05),
            0.1 * level as f64,
            Vec3::new(0.8, 0.8, 0.8),
        );
    }
}

fn append_site(scene: &mut RenderScene, frame: &Frame) {
    const CAR: [f32; 4] = [0.74, 0.77, 0.82, 1.0];
    const DOOR: [f32; 4] = [0.80, 0.84, 0.90, 1.0];
    const BUTTON_IDLE: [f32; 4] = [0.45, 0.47, 0.52, 1.0];
    const BUTTON_LIT: [f32; 4] = [0.99, 0.74, 0.20, 1.0];
    /// The two trucks are the same machine; the colour says which floor's.
    const ORANGE: [f32; 4] = [0.95, 0.56, 0.06, 1.0];
    const BLUE: [f32; 4] = [0.16, 0.45, 0.86, 1.0];

    push_building(scene);
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M, frame.car_y_m, 0.0),
        CAR_HALF_M,
        CAR,
        0.55,
        0.08,
        [0.0; 3],
    );
    for sign in [-1.0, 1.0] {
        push_stand_leg(
            scene,
            Vec3::new(
                CAR_STAND_X_M,
                frame.car_y_m + CAR_HALF_M.y + STAND_LEG_HALF_M.y,
                sign * STAND_LEG_Z_M,
            ),
        );
    }
    push_car_interior(scene, frame.car_y_m, true);
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for sign in [-1.0, 1.0] {
        push_pbr(
            scene,
            Vec3::new(
                DOORWAY_X_M,
                frame.car_y_m + DOOR_HALF_M.y,
                sign * (doorway_z_m + frame.door_opening_m),
            ),
            DOOR_HALF_M,
            DOOR,
            0.35,
            0.55,
            [0.0; 3],
        );
        // A narrow vision panel in each leaf.
        push_pbr(
            scene,
            Vec3::new(
                DOORWAY_X_M - DOOR_HALF_M.x - 0.001,
                frame.car_y_m + 1.45,
                sign * (doorway_z_m + frame.door_opening_m),
            ),
            Vec3::new(0.001, 0.35, 0.06),
            [0.10, 0.14, 0.18, 1.0],
            0.05,
            0.2,
            [0.0; 3],
        );
    }
    // Drawn larger than the collider so its lit state is visible at this scale.
    push_pbr(
        scene,
        BUTTON_CENTER_M - BUTTON_NORMAL * 0.04,
        Vec3::new(0.09, 0.15, 0.02),
        [0.32, 0.34, 0.39, 1.0],
        0.5,
        0.05,
        [0.0; 3],
    );
    push_pbr(
        scene,
        BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
        Vec3::new(0.055, 0.055, BUTTON_HALF_M.z),
        if frame.button_lit {
            BUTTON_LIT
        } else {
            BUTTON_IDLE
        },
        0.4,
        0.1,
        if frame.button_lit {
            [0.55, 0.38, 0.05]
        } else {
            [0.0; 3]
        },
    );

    push_truck(scene, &frame.lower, ORANGE);
    push_truck(scene, &frame.upper, BLUE);
    push_pallet(scene, frame.case);
    // The scanned box is 0.388 x 0.341 x 0.516 m against the 0.38 x 0.32 x
    // 0.52 m physics case; its origin is its base, 0.033 m off-centre in z.
    let (case, case_rotation) = frame.case;
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            case + case_rotation * Vec3::new(0.0, -CASE_HALF_M.y, -0.033),
            case_rotation,
        ),
        VisualShape::Mesh {
            path: CARDBOARD_BOX.to_string(),
            scale: Vec3::new(0.38 / 0.388, 0.32 / 0.341, 0.52 / 0.516),
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let trace = std::env::args().any(|argument| argument == "--trace");
    let relay = run_relay(None, trace);

    println!(
        "relay: {} press(es), finished at {:.1} s; 1F lifted {:.3} m, slip {:.4} m; 2F lifted {:.3} m off the car stand, slip {:.4} m",
        relay.presses,
        relay.finished_step as f64 / PHYSICS_HZ,
        relay.lower_lift_m,
        relay.lower_slip_m,
        relay.upper_lift_m,
        relay.upper_slip_m,
    );
    println!(
        "doors: {} conflict step(s); car moved with a truck aboard on {} step(s); case rode the car: {}",
        relay.door_conflicts, relay.rides_with_a_truck, relay.case_rode_the_car
    );
    println!(
        "case delivered at ({:.3}, {:.3}, {:.3}); 2F truck backed off to x={:.2}",
        relay.case_final.x, relay.case_final.y, relay.case_final.z, relay.upper_final_x_m
    );
    println!("sequence: {}", relay.order.join(" -> "));

    assert!(
        relay.presses >= 1,
        "the 1F truck never actuated the call button"
    );
    assert!(
        relay.lower_lift_m > 0.12 && relay.upper_lift_m > 0.12,
        "both trucks must lift the case clear: {:.3} / {:.3} m",
        relay.lower_lift_m,
        relay.upper_lift_m
    );
    // The handover is the point: each truck stays on its own floor and the
    // case goes up in the car without either of them.
    assert!(
        relay.lower_highest_y_m < FLOOR_HEIGHTS_M[1] * 0.5
            && relay.upper_lowest_y_m > FLOOR_HEIGHTS_M[1] * 0.5,
        "a truck left its floor"
    );
    assert!(relay.case_rode_the_car, "the case never rode the car");
    assert_eq!(
        relay.rides_with_a_truck, 0,
        "the car moved with a truck in it"
    );
    assert_eq!(
        relay.door_conflicts, 0,
        "a truck or the case was in the doorway while the doors were not fully open"
    );
    let outbound_y_m = seated_case_y_m(FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y);
    assert!(
        (relay.case_final.y - outbound_y_m).abs() < 0.03
            && (relay.case_final.x - OUTBOUND_X_M).abs() < 0.08
            && relay.case_final.z.abs() < 0.08,
        "the case is not seated on the outbound stand"
    );
    // Clear of what it delivered, or "delivered" means "still on the tines".
    assert!(
        relay.upper_final_x_m + FORK_REACH_M - FORK_HALF_M.x > relay.case_final.x + CASE_HALF_M.x,
        "the 2F truck did not back its tines out of the delivered case"
    );

    if smoke {
        println!("smoke ok: the relay completes headlessly");
        return;
    }

    // Second pass, sampled so the capture spans exactly the relay.
    let every = (relay.finished_step / FRAME_COUNT).max(1) + 1;
    let captured = run_relay(Some(every), false);
    assert_eq!(
        captured.finished_step, relay.finished_step,
        "the capture pass must replay the measured relay"
    );

    let frames_dir =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-warehouse-relay-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    backend.set_environment(warehouse_environment());
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let orbit = CameraOrbit {
        focus: Vec3::new(SHAFT_X_M - 1.95, 1.95, 0.0),
        yaw_rad: 0.46,
        pitch_rad: 1.16,
        distance_m: 8.3,
    };
    let still: Option<usize> = std::env::args()
        .skip_while(|argument| argument != "--still")
        .nth(1)
        .and_then(|value| value.parse().ok());
    for (index, frame) in captured.frames.iter().enumerate() {
        if still.is_some_and(|wanted| wanted != index) {
            continue;
        }
        let mut scene = RenderScene::default();
        append_site(&mut scene, frame);
        let root = props_root();
        mesh_cache
            .resolve_scene(&mut scene, &[root.as_path()])
            .expect("resolve scene meshes");
        // `--closeup` looks at the ground-floor truck, for checking the model.
        let view = if std::env::args().any(|argument| argument == "--closeup") {
            CameraOrbit {
                focus: frame.lower.chassis.0 + Vec3::new(-0.3, 0.2, 0.0),
                yaw_rad: 0.35,
                pitch_rad: 1.2,
                distance_m: 2.4,
            }
        } else {
            orbit
        };
        let output = backend
            .render_scene_camera(&camera, &view.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render relay frame");
        write_png(
            &frames_dir.join(format!("frame-{index:03}.png")),
            &output.color.rgba8,
        )
        .expect("write frame");
    }
    if let Some(index) = still {
        println!(
            "wrote still {}",
            frames_dir.join(format!("frame-{index:03}.png")).display()
        );
        return;
    }
    let media_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");
    let gif_path = media_dir.join("warehouse-relay.gif");
    build_gif(&frames_dir, &gif_path).expect("build gif");
    let poster =
        image::open(frames_dir.join(format!("frame-{:03}.png", captured.frames.len() / 2)))
            .expect("read poster frame");
    poster
        .save(media_dir.join("warehouse-relay.png"))
        .expect("write poster");
    println!("wrote {}", gif_path.display());
}

/// The parts of the site that never move: shaft, decks, racking, fixtures and
/// the two stands.
fn push_building(scene: &mut RenderScene) {
    const SHAFT: [f32; 4] = [0.33, 0.36, 0.42, 1.0];
    const RACK_BEAM: [f32; 4] = [0.92, 0.55, 0.10, 1.0];
    const RACK_UPRIGHT: [f32; 4] = [0.16, 0.36, 0.60, 1.0];
    const BAY: [f32; 4] = [0.14, 0.70, 0.40, 1.0];
    const HAZARD: [f32; 4] = [0.92, 0.78, 0.12, 1.0];
    /// Racking runs along the back of the aisle. On the camera's side it stands
    /// between the lens and the job and hides the whole thing.
    const RACK_Z_M: f64 = -0.98;

    // Shaft walls, so the car reads as travelling inside something rather than
    // floating. Back plus one side; the open side is the camera's cutaway.
    let shaft_half_height_m = FLOOR_HEIGHTS_M[1] * 0.5 + 0.9;
    let shaft_mid_y_m = shaft_half_height_m - 0.6;
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M + 1.02, shaft_mid_y_m, 0.0),
        Vec3::new(0.06, shaft_half_height_m, 0.98),
        SHAFT,
        0.88,
        0.03,
        [0.0; 3],
    );
    push_pbr(
        scene,
        Vec3::new(SHAFT_X_M, shaft_mid_y_m, -1.12),
        Vec3::new(1.05, shaft_half_height_m, 0.06),
        SHAFT,
        0.88,
        0.03,
        [0.0; 3],
    );

    for (index, height_m) in FLOOR_HEIGHTS_M.iter().enumerate() {
        push_pbr(
            scene,
            Vec3::new(DECK_X_M[index], height_m + 1.05, -1.30),
            Vec3::new(DECK_HALF_M[index].x - 0.6, 1.05, 0.05),
            SHAFT,
            0.9,
            0.05,
            [0.0; 3],
        );
        push_textured_panel(
            scene,
            Vec3::new(DECK_X_M[index], height_m + 1.05, -1.245),
            Vec3::new(DECK_HALF_M[index].x - 0.6, 0.0, 0.0),
            Vec3::new(0.0, 1.05, 0.0),
            wall_textures(),
            1.6,
            [1.0, 1.0, 1.0, 1.0],
        );
        push_concrete_deck(
            scene,
            Vec3::new(DECK_X_M[index], *height_m, 0.0),
            DECK_HALF_M[index],
        );
        // Hazard striping across the lift threshold, which is where the deck
        // stops being a floor and starts being a hole.
        for stripe in 0..5 {
            push_pbr(
                scene,
                Vec3::new(
                    SHAFT_X_M - 1.16,
                    height_m + 2.0 * CAR_HALF_M.y + 0.003,
                    -0.6 + f64::from(stripe) * 0.30,
                ),
                Vec3::new(0.10, 0.003, 0.10),
                HAZARD,
                0.7,
                0.0,
                [0.0; 3],
            );
        }
    }

    push_racking(scene, RACK_BEAM, RACK_UPRIGHT, RACK_Z_M);

    // Ceiling fixtures: scanned fluorescent battens, plus the emissive tube
    // each one holds so the shot has light sources in it.
    for bay in 0..4 {
        let at = Vec3::new(SHAFT_X_M - 6.6 + f64::from(bay) * 1.7, 2.84, -0.45);
        push_prop(
            scene,
            "mounted_fluorescent_lights/mounted_fluorescent_lights_1k.gltf",
            at,
            0.0,
            Vec3::ONE,
        );
        push_pbr(
            scene,
            at - Vec3::new(0.0, 0.03, 0.0),
            Vec3::new(0.44, 0.012, 0.02),
            [0.97, 0.98, 1.0, 1.0],
            0.35,
            0.0,
            [0.9, 0.92, 0.97],
        );
    }

    push_floor_props(scene);

    for (x_m, deck_y_m) in [
        (STAND_X_M, CAR_HALF_M.y),
        (OUTBOUND_X_M, FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y),
    ] {
        for sign in [-1.0, 1.0] {
            push_stand_leg(
                scene,
                Vec3::new(x_m, deck_y_m + STAND_LEG_HALF_M.y, sign * STAND_LEG_Z_M),
            );
        }
    }
    push_pbr(
        scene,
        Vec3::new(
            OUTBOUND_X_M,
            FLOOR_HEIGHTS_M[1] + 2.0 * CAR_HALF_M.y + 0.004,
            0.0,
        ),
        Vec3::new(0.40, 0.004, 0.40),
        BAY,
        0.8,
        0.0,
        [0.0; 3],
    );
}

/// Racking down the ground-floor aisle: uprights, beam levels and stock.
/// Dressing -- the trucks' routes never cross it.
fn push_racking(
    scene: &mut RenderScene,
    rack_beam: [f32; 4],
    rack_upright: [f32; 4],
    rack_z_m: f64,
) {
    // Racking down the ground-floor aisle: uprights, bracing and beam levels,
    // with pallets on them. Dressing -- the truck's route never crosses it.
    for bay in 0..4 {
        let x_m = SHAFT_X_M - 7.4 + f64::from(bay) * 1.62;
        for level in 0..3 {
            let y_m = 0.52 + f64::from(level) * 0.88;
            push_steel(
                scene,
                Vec3::new(x_m, y_m, rack_z_m),
                Quat::IDENTITY,
                Vec3::new(0.76, 0.045, 0.05),
                rack_beam,
            );
            // Stock: every level but the top holds goods, scanned boxes on
            // pallets or crates, varied by bay so the aisle does not repeat.
            if level < 2 {
                let top = y_m + 0.045;
                match (bay + level) % 3 {
                    0 => {
                        push_pallet(
                            scene,
                            (
                                Vec3::new(x_m, top + CASE_HALF_M.y + 0.114, rack_z_m),
                                Quat::IDENTITY,
                            ),
                        );
                        for (dx, yaw) in [(-0.22, 0.05), (0.22, -0.08)] {
                            push_prop(
                                scene,
                                CARDBOARD_BOX,
                                Vec3::new(x_m + dx, top + 0.114, rack_z_m),
                                yaw,
                                Vec3::ONE,
                            );
                        }
                        push_prop(
                            scene,
                            CARDBOARD_BOX,
                            Vec3::new(x_m - 0.05, top + 0.455, rack_z_m),
                            1.62,
                            Vec3::ONE,
                        );
                    }
                    1 => {
                        push_prop(
                            scene,
                            WOODEN_CRATE,
                            Vec3::new(x_m, top, rack_z_m + 0.02),
                            0.0,
                            Vec3::ONE,
                        );
                        push_prop(
                            scene,
                            WOODEN_CRATE,
                            Vec3::new(x_m + 0.1, top + 0.34, rack_z_m),
                            0.06,
                            Vec3::new(0.8, 0.8, 0.8),
                        );
                    }
                    _ => {
                        push_pallet(
                            scene,
                            (
                                Vec3::new(x_m, top + CASE_HALF_M.y + 0.114, rack_z_m),
                                Quat::IDENTITY,
                            ),
                        );
                        push_prop(
                            scene,
                            CARDBOARD_BOX,
                            Vec3::new(x_m + 0.15, top + 0.114, rack_z_m),
                            1.57,
                            Vec3::ONE,
                        );
                    }
                }
            }
        }
        for side in [-1.0, 1.0] {
            push_steel(
                scene,
                Vec3::new(x_m + side * 0.76, 1.32, rack_z_m),
                Quat::IDENTITY,
                Vec3::new(0.05, 1.32, 0.05),
                rack_upright,
            );
        }
    }
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = png::Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

/// Encodes the frames with `tools/encode_gif.py`: a scanned-texture scene
/// through ffmpeg's encoder came out at 7-13 MB, frame-differenced at under 1.
fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/encode_gif.py");
    let status = std::process::Command::new("python3")
        .arg(script)
        .arg(frames_dir)
        .arg(gif_path)
        .args(["--fps", "12", "--colors", "192"])
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("encode_gif.py failed to build the gif"));
    }
    Ok(())
}
