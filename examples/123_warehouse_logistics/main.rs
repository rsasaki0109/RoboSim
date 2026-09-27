//! A forklift AGV runs one inbound-to-delivery job across two floors: it takes
//! a case off the goods-in stand, carries it down the racking aisle, calls the
//! lift, rides up and sets the case down on the outbound bay.
//!
//! Every part of this exists somewhere in the repository and none of it had a
//! picture as one job. Example 122 draws the lift; examples 118 to 120 prove
//! the devices; the manipulation examples pick things up on one floor. What a
//! logistics site actually looks like is the whole chain end to end, so this
//! runs it as a single mission.
//!
//! Nothing is staged:
//!
//! * The mast is a real prismatic joint between the chassis and the fork, with
//!   a position servo. The case's weight is carried through that joint, so the
//!   fork has to hold it up rather than the case being pinned to the carriage.
//! * The case is an ordinary dynamic body throughout. It is lifted because the
//!   tines are underneath it and it is delivered because they come back down --
//!   there is no attach step, no weld and no body-type switch.
//! * The car and door leaves are kinematic bodies driven by `rne_nav::Elevator`,
//!   and the button is `rne_nav::CallButton` reading solved contact force.
//!
//! The chassis is velocity-controlled, which is the one thing here that is a
//! command rather than a consequence: its drive wheels are not modelled. The
//! load it carries is therefore not felt as rolling resistance.
//!
//! ```text
//! cargo run --release -p warehouse_logistics --example 123_warehouse_logistics -- --smoke
//! cargo run --release -p warehouse_logistics --example 123_warehouse_logistics
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
///
/// The upper deck is short on purpose: run it the length of the lower one and
/// it hangs over the goods-in stand, hiding the pick from the only camera this
/// runs from.
const DECK_HALF_M: [Vec3; 2] = [Vec3::new(3.9, 0.06, 1.5), Vec3::new(1.65, 0.06, 1.5)];
/// Deck centre on the world x axis per floor, in meters.
const DECK_X_M: [f64; 2] = [SHAFT_X_M - 4.8, SHAFT_X_M - 2.55];

/// Physics rate. The car carries the truck through contact and the tines carry
/// the case the same way, which needs the solver to see platform velocity
/// rather than a teleport.
const PHYSICS_HZ: f64 = 240.0;
/// Floor heights, in meters.
const FLOOR_HEIGHTS_M: [f64; 2] = [0.0, 3.2];
/// Car platform half extents, in meters.
const CAR_HALF_M: Vec3 = Vec3::new(0.95, 0.06, 0.95);
/// Door leaf half extents, in meters.
const DOOR_HALF_M: Vec3 = Vec3::new(0.05, 1.15, 0.46);
/// Shaft centre on the world x axis, in meters.
const SHAFT_X_M: f64 = 0.0;

/// Truck chassis half extents, in meters.
///
/// Long and low. The counterweight of a real truck is what keeps it down when
/// the load is out in front; here the same job is done by putting the mass in a
/// body wide enough not to pitch over on the first acceleration.
const CHASSIS_HALF_M: Vec3 = Vec3::new(0.52, 0.26, 0.30);
/// Truck chassis mass, in kilograms.
///
/// A counterweight, which is what lets anything carry a load out in front of
/// its own wheels.
const CHASSIS_MASS_KG: f64 = 320.0;
/// Fork carriage half extents, in meters: a flat slab standing in for the tines.
///
/// Narrower in z than the gap between the stand's legs, because that gap is
/// what the tines have to pass through to get under the load.
const FORK_HALF_M: Vec3 = Vec3::new(0.34, 0.025, 0.14);
/// Fork mass, in kilograms.
const FORK_MASS_KG: f64 = 8.0;
/// Fork reach from the chassis centre, in meters.
///
/// Negative: the tines lead along -x, the direction the goods-in stand is in.
/// Two clearances set this, and both were measured after getting it wrong:
/// at `ENGAGED_X_M` the chassis front face must stop short of the stand's legs
/// (it clears by 0.16 m), and the backrest must sit ahead of the chassis rather
/// than inside it. At -0.88 m the backrest overlapped the chassis by 5 mm and
/// the solver pushed the two apart with 1.3 kN for the whole run.
const FORK_REACH_M: f64 = -0.95;
/// Mast travel, in meters of joint displacement.
///
/// Zero is the height the tines enter the stand at. The mast has to reach
/// below that to set a load down on a bare deck, and above it to carry.
const FORK_GROUND_M: f64 = -0.30;
const FORK_ENGAGE_M: f64 = 0.0;
const FORK_CARRY_M: f64 = 0.30;
/// Height of the joint anchor above the chassis centre, in meters.
///
/// Chosen so that at `FORK_ENGAGE_M` the top of the tines sits just under the
/// load, and at `FORK_GROUND_M` it sits just above the deck.
const MAST_ANCHOR_Y_M: f64 = 0.015;

/// Load backrest half extents, in meters.
///
/// A real truck has one for the reason this one needs one: on the tines a case
/// has nothing behind it, and the first time the truck accelerates away from
/// the load the case slides off the open end.
const BACKREST_HALF_M: Vec3 = Vec3::new(0.025, 0.21, 0.24);
/// Backrest mass, in kilograms.
const BACKREST_MASS_KG: f64 = 4.0;

/// Case half extents, in meters.
///
/// Wide enough in z to bridge the stand's two legs with room to spare, so it
/// rests on them rather than balancing on their inner edges.
const CASE_HALF_M: Vec3 = Vec3::new(0.19, 0.16, 0.26);
/// Case mass, in kilograms.
const CASE_MASS_KG: f64 = 14.0;
/// Goods-in stand: where the case starts, in meters.
const STAND_X_M: f64 = -4.30;
/// Height of the stand's legs, in meters.
const STAND_TOP_Y_M: f64 = 0.34;
/// Half extents of one stand leg, in meters.
const STAND_LEG_HALF_M: Vec3 = Vec3::new(0.10, STAND_TOP_Y_M * 0.5, 0.07);
/// Centre of each leg along z, in meters. The gap between them is what the
/// tines drive into.
const STAND_LEG_Z_M: f64 = 0.23;

/// Where the truck lines up before running the tines into the stand, in meters.
const APPROACH_X_M: f64 = ENGAGED_X_M + CASE_HALF_M.x + 0.46;
/// Where the truck stops with the tines under the case, in meters.
/// Deep enough that the case ends up against the backrest rather than out on
/// the tines: a load parked near the tip slides off it the first time the truck
/// slows down, which is what happened at every shallower value tried.
const ENGAGED_X_M: f64 = STAND_X_M - FORK_REACH_M - (FORK_HALF_M.x - CASE_HALF_M.x) - 0.02;

/// Where the truck stands inside the car, in meters.
///
/// Not the shaft centreline: the truck is 0.52 m of chassis behind its centre
/// and 1.29 m of mast and tines ahead of it, so centring the chassis leaves the
/// tines hanging out of the car. They then catch the upper floor slab on the
/// way up, which drove the mast 0.14 m through its own lower limit and dropped
/// the case. Centring the whole assembly leaves 45 mm at each end.
const BOARD_X_M: f64 = SHAFT_X_M + 0.5 * ((-FORK_REACH_M + FORK_HALF_M.x) - CHASSIS_HALF_M.x);

/// Where the truck stops to press the button, in meters.
const PRESS_X_M: f64 = -1.45;
/// Button face centre, in meters: on a stanchion beside the doorway.
///
/// Set so the truck reaches it with a 5 cm sidestep rather than a lane change.
/// Every centimetre of sideways excursion is lateral acceleration applied to a
/// case balanced on a flat fork with nothing on its sides, and the case is not
/// strapped down; a 0.5 m reach across the aisle threw it off every run.
const BUTTON_CENTER_M: Vec3 = Vec3::new(-1.25, 0.56, CHASSIS_HALF_M.z + 0.02);
/// Outward normal of the button face.
const BUTTON_NORMAL: Vec3 = Vec3::new(0.0, 0.0, -1.0);
/// Where the truck drives to reach the panel, in meters.
///
/// Slightly past the face: a controller that stops flush leaves no standing
/// error, so the contact carries no force and the button never actuates. Only
/// slightly, though. A 320 kg chassis under velocity control does not stop when
/// it touches something, it keeps going: at 50 mm of overshoot it drove 24 kN
/// into the panel, wedged, and never reached the doorway.
const PRESS_Z_M: f64 = BUTTON_CENTER_M.z - CHASSIS_HALF_M.z + 0.015;
/// Button body half extents, in meters.
const BUTTON_HALF_M: Vec3 = Vec3::new(0.05, 0.05, 0.03);

/// Where the truck sets the case down on the upper floor, in meters.
const BAY_X_M: f64 = SHAFT_X_M - 2.35;
/// Outbound stand centre on the upper floor, in meters: under the tines when
/// the truck is at `BAY_X_M`.
///
/// It has legs for the same reason the goods-in stand does. Lowering a flat
/// tine to the deck leaves the case still sitting on the tine, 0.046 m up,
/// because the tine itself is in the way -- so backing out drags the load
/// instead of leaving it. The case has to come to rest on something the tines
/// can slide out from between.
const OUTBOUND_X_M: f64 = BAY_X_M + FORK_REACH_M;
/// Top of the outbound stand's legs, in world meters.
const OUTBOUND_TOP_Y_M: f64 = FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y + STAND_TOP_Y_M;
/// Where the truck ends up after backing off the delivered case, in meters.
///
/// Far enough back that the tines are out of the outbound stand, and no
/// further: at -1.25 m the truck reversed into the doorway and spent the rest
/// of the run pushing 4 kN into a closed door leaf.
const WITHDRAW_X_M: f64 = SHAFT_X_M - 1.75;

/// Drive speed, in meters per second.
const DRIVE_M_S: f64 = 0.62;
/// Creep speed for entering and leaving a load, in meters per second.
const CREEP_M_S: f64 = 0.22;
/// Drive acceleration limit, in meters per second squared.
///
/// The case rides on a flat fork with nothing holding it, so the truck can only
/// change speed as fast as friction can take the case with it.
const DRIVE_ACCEL_M_S2: f64 = 0.5;
/// Seconds the mission is allowed before it is declared stuck.
const MAX_SECONDS: f64 = 95.0;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.30, 0.34, 0.40, 1.0];
const FRAME_COUNT: usize = 120;

fn elevator_spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: FLOOR_HEIGHTS_M.to_vec(),
        car_speed_m_s: 1.1,
        car_acceleration_m_s2: 0.9,
        door_travel_m: 0.56,
        door_speed_m_s: 0.55,
        door_hold_s: 8.0,
    }
}

/// Grip so a case parked on the tines stays there when the truck accelerates.
fn high_friction() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: 1.4,
        restitution: 0.0,
    }
}

/// Rolling contact for the chassis.
///
/// The drive wheels are not modelled, so the chassis slides on its own base.
/// At deck friction it loses the whole commanded speed to Coulomb drag inside
/// a single 240 Hz step and never gets anywhere; a wheeled vehicle's
/// resistance is low, and this is the value that says so.
///
/// It is not what the contact ends up with: the solver averages the two
/// materials, so against this site's 1.4 decks the chassis slides at 0.72.
/// That is enough for the drive speeds used here; example 125, which needs
/// slow creeps and turns, gives its decks a low friction instead.
fn rolling_friction() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: 0.04,
        restitution: 0.0,
    }
}

/// Bodies the mission drives or reads.
struct Site {
    car: Entity,
    left_door: Entity,
    right_door: Entity,
    chassis: Entity,
    fork: Entity,
    backrest: Entity,
    case: Entity,
    button: Entity,
}

/// Spawns one two-legged stand: a pair of fixed legs with a gap between them.
///
/// The gap is the whole point. A load resting on a solid plinth cannot be
/// forked, only pushed, and a load lowered onto a bare deck is still sitting on
/// the tines that put it there.
fn spawn_stand(world: &mut World, name: &str, center_x_m: f64, deck_y_m: f64) {
    for (suffix, sign) in [("near", -1.0), ("far", 1.0)] {
        let leg = spawn_named(world, format!("{name}_leg_{suffix}"));
        world.entity_mut(leg).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
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

/// Spawns the car, its two door leaves and the call button.
fn spawn_lift(world: &mut World) -> (Entity, Entity, Entity, Entity) {
    let car = spawn_named(world, "elevator_car");
    world.entity_mut(car).insert((
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
        // The car carries its rider, so its pose is a command: the solver needs
        // its velocity to resolve the contact that does the carrying.
        CommandedKinematicPose,
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: CAR_HALF_M,
            },
            material: high_friction(),
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(SHAFT_X_M, FLOOR_HEIGHTS_M[0], 0.0),
            Quat::IDENTITY,
        ),
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
                    half_extents_m: DOOR_HALF_M,
                },
                ..Collider::default()
            },
            Transform3::IDENTITY,
        ));
        entity
    };
    let left_door = door("door_left");
    let right_door = door("door_right");

    // The button is part of the building: fixed, so the truck cannot push it
    // out of the way, and its face is the side the truck arrives from.
    let button = spawn_named(world, "call_button");
    world.entity_mut(button).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: BUTTON_HALF_M,
            },
            ..Collider::default()
        },
        // `BUTTON_CENTER_M` is the *face*, which is what `CallButtonSpec`
        // describes, so the body sits half its thickness behind it.
        Transform3::from_translation_rotation(
            BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
            Quat::IDENTITY,
        ),
    ));

    (car, left_door, right_door, button)
}

/// Spawns the four bodies the truck and its load are made of: chassis, fork
/// carriage, backrest and the case itself.
fn spawn_truck_bodies(world: &mut World) -> (Entity, Entity, Entity, Entity) {
    let chassis_y_m = CAR_HALF_M.y + CHASSIS_HALF_M.y + 0.01;
    let chassis = spawn_named(world, "forklift_chassis");
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
        Transform3::from_translation_rotation(
            Vec3::new(APPROACH_X_M, chassis_y_m, 0.0),
            Quat::IDENTITY,
        ),
    ));

    // The mast: one prismatic degree of freedom along the chassis's own +y,
    // with a position servo stiff enough to hold a loaded fork against gravity.
    let fork = spawn_named(world, "fork_carriage");
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
        Transform3::from_translation_rotation(
            Vec3::new(
                APPROACH_X_M + FORK_REACH_M,
                chassis_y_m + MAST_ANCHOR_Y_M,
                0.0,
            ),
            Quat::IDENTITY,
        ),
        PrismaticJointDesc {
            parent: chassis,
            axis: Vec3::Y,
            anchor_parent_m: Vec3::new(FORK_REACH_M, MAST_ANCHOR_Y_M, 0.0),
            anchor_child_m: Vec3::ZERO,
            relative_rotation: Quat::IDENTITY,
            lower_m: Some(FORK_GROUND_M),
            upper_m: Some(FORK_CARRY_M + 0.05),
        },
        // Force-based, so the gains are newtons rather than an acceleration,
        // and sized to the load rather than to whatever holds position
        // fastest. The tines act a `FORK_REACH_M` lever ahead of the chassis,
        // so an oversized mast does not hold the load better -- it torques the
        // truck onto its nose. At `FORK_REACH_M` the 700 N cap is 560 N.m
        // against the chassis's 1632 N.m of restoring moment, which is the
        // margin that keeps all four wheels down while the mast is working.
        JointMotorGainModel::ForceBased,
        JointMotor {
            target_position: FORK_ENGAGE_M,
            stiffness: 6_000.0,
            gain: 600.0,
            max_force: 700.0,
            ..JointMotor::default()
        },
    ));

    // The backrest, welded to the carriage: the tines' closed end.
    let backrest = spawn_named(world, "load_backrest");
    let backrest_offset = Vec3::new(FORK_HALF_M.x + BACKREST_HALF_M.x, BACKREST_HALF_M.y, 0.0);
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
        Transform3::from_translation_rotation(
            Vec3::new(
                APPROACH_X_M + FORK_REACH_M,
                chassis_y_m + MAST_ANCHOR_Y_M,
                0.0,
            ) + backrest_offset,
            Quat::IDENTITY,
        ),
        // Its own mast rail rather than a weld to the carriage. Chaining a
        // fixed joint onto the prismatic one leaves the solver a two-joint
        // chain hanging off a body whose velocity is overwritten every step,
        // and the mast loses most of its authority to it: measured, the lift
        // slowed from 0.16 m in 25 s to 0.07 m in 50 s. Two independent joints
        // to the same parent, driven to the same target, hold the same shape
        // without the chain.
        PrismaticJointDesc {
            parent: chassis,
            axis: Vec3::Y,
            anchor_parent_m: Vec3::new(FORK_REACH_M, MAST_ANCHOR_Y_M, 0.0) + backrest_offset,
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

    // The outbound stand, the goods-in stand's twin on the upper floor.
    spawn_stand(
        world,
        "outbound",
        OUTBOUND_X_M,
        FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y,
    );

    // The case is an ordinary dynamic body from here to the outbound bay.
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

    (chassis, fork, backrest, case)
}

fn spawn_site(world: &mut World) -> Site {
    // Warehouse decks, one per served floor, stopping short of the shaft so the
    // camera sees into it.
    for (index, height_m) in FLOOR_HEIGHTS_M.iter().enumerate() {
        let slab = spawn_named(world, if index == 0 { "deck_1f" } else { "deck_2f" });
        world.entity_mut(slab).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: DECK_HALF_M[index],
                },
                material: high_friction(),
                ..Collider::default()
            },
            // Flush with the car platform's top rather than 6 cm below it. A
            // step at the doorway is a trip hazard for a long truck with a
            // loaded fork out front, and it put the truck on its roof.
            Transform3::from_translation_rotation(
                Vec3::new(DECK_X_M[index], *height_m, 0.0),
                Quat::IDENTITY,
            ),
        ));
    }

    // Goods-in stand: two legs with an open middle, which is the only reason a
    // fork can get under anything. A solid plinth would have to be shoved.
    spawn_stand(world, "goods_in", STAND_X_M, CAR_HALF_M.y);

    let (car, left_door, right_door, button) = spawn_lift(world);

    let (chassis, fork, backrest, case) = spawn_truck_bodies(world);

    Site {
        car,
        left_door,
        right_door,
        chassis,
        fork,
        backrest,
        case,
        button,
    }
}

/// Writes the state machine's car height and door opening onto the bodies.
fn apply_elevator(world: &mut World, site: &Site, elevator: &Elevator) {
    let car_height_m = elevator.car_height_m();
    if let Some(mut transform) = world.get_mut::<Transform3>(site.car) {
        transform.translation.y = car_height_m;
    }
    let opening_m = elevator.door_opening_m();
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for (entity, sign) in [(site.left_door, -1.0), (site.right_door, 1.0)] {
        if let Some(mut transform) = world.get_mut::<Transform3>(entity) {
            transform.translation = Vec3::new(
                SHAFT_X_M - CAR_HALF_M.x,
                car_height_m + DOOR_HALF_M.y,
                sign * (doorway_z_m + opening_m),
            );
        }
    }
}

fn translation(world: &World, entity: Entity) -> Vec3 {
    world
        .get::<Transform3>(entity)
        .map_or(Vec3::ZERO, |t| t.translation)
}

/// What the truck is doing, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Approach,
    Engage,
    Lift,
    Haul,
    Press,
    WaitForDoors,
    Board,
    Ride,
    DriveOut,
    Place,
    Lower,
    Withdraw,
    Done,
}

impl Phase {
    fn label(self) -> &'static str {
        match self {
            Phase::Approach => "lining up on the goods-in stand",
            Phase::Engage => "running the tines under the case",
            Phase::Lift => "lifting the case off the stand",
            Phase::Haul => "hauling to the lift",
            Phase::Press => "pressing the call button",
            Phase::WaitForDoors => "waiting for the doors",
            Phase::Board => "boarding with the load",
            Phase::Ride => "riding to 2F",
            Phase::DriveOut => "driving out on 2F",
            Phase::Place => "positioning over the outbound bay",
            Phase::Lower => "setting the case down",
            Phase::Withdraw => "backing clear of the delivered case",
            Phase::Done => "delivered",
        }
    }

    /// Mast target for this phase, in meters of joint travel.
    fn mast_target_m(self) -> f64 {
        match self {
            Phase::Approach | Phase::Engage => FORK_ENGAGE_M,
            // Below the seating height, so the case transfers to the stand's
            // legs and the tines come out from under it.
            Phase::Lower | Phase::Withdraw => FORK_ENGAGE_M - 0.05,
            Phase::Done => FORK_GROUND_M,
            _ => FORK_CARRY_M,
        }
    }
}

/// One captured frame: every pose the renderer needs.
struct Frame {
    car_y_m: f64,
    door_opening_m: f64,
    chassis: Vec3,
    fork: Vec3,
    backrest: Vec3,
    case: Vec3,
    phase: Phase,
}

struct Mission {
    frames: Vec<Frame>,
    /// Each phase in the order it was entered, for the run's own report.
    order: Vec<Phase>,
    presses: u32,
    /// Worst slip of the case across the tines while it was being carried.
    carry_slip_m: f64,
    /// How high the case was lifted above the stand deck.
    lift_height_m: f64,
    delivered_floor: usize,
    /// Where the case ended up, so the run can check it was actually delivered.
    case_final: Vec3,
}

/// Writes this step's drive command and mast target onto the truck.
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn drive_truck(
    world: &mut World,
    site: &Site,
    phase: Phase,
    chassis: Vec3,
    dt_s: f64,
    commanded_x_m_s: &mut f64,
    commanded_z_m_s: &mut f64,
) {
    // Drive command for this phase, along x and z only; gravity owns y.
    let (target_x_m, target_z_m, speed_m_s) = match phase {
        Phase::Approach => (APPROACH_X_M, 0.0, DRIVE_M_S),
        Phase::Engage => (ENGAGED_X_M, 0.0, CREEP_M_S),
        // Hold station while the mast does the work, so the case comes
        // straight up off the stand instead of being dragged over its lip.
        Phase::Lift => (ENGAGED_X_M, 0.0, CREEP_M_S),
        Phase::Haul | Phase::Press => (PRESS_X_M, PRESS_Z_M, DRIVE_M_S),
        // Back off the panel decisively once the call is registered. A slow
        // withdrawal lets the contact chatter across the button's release
        // threshold and register a second press.
        Phase::WaitForDoors => (PRESS_X_M, 0.0, DRIVE_M_S),
        Phase::Board | Phase::Ride => (BOARD_X_M, 0.0, DRIVE_M_S),
        Phase::DriveOut | Phase::Place => (BAY_X_M, 0.0, DRIVE_M_S),
        Phase::Lower => (BAY_X_M, 0.0, CREEP_M_S),
        Phase::Withdraw | Phase::Done => (WITHDRAW_X_M, 0.0, CREEP_M_S),
    };
    // Tight enough that the drive saturates for most of each leg instead of
    // creeping in asymptotically; the mission has eleven legs to get through.
    let gain = |error_m: f64| (error_m / 0.25).clamp(-1.0, 1.0) * speed_m_s;
    // The command itself is rate-limited, not the measured velocity. The
    // case is not strapped to anything, and a velocity written straight
    // onto the body changes inside one step -- an unbounded acceleration
    // that leaves the case behind where the tines were. Ramping the
    // *command* keeps the acceleration the case has to follow at
    // `DRIVE_ACCEL_M_S2`, which needs 0.08 of its weight in friction.
    // Ramping the measured velocity instead does not work: at this mass
    // each increment is smaller than the contact's stiction and the solver
    // arrests it before the next step.
    let ramp = |current: f64, target: f64| {
        let step_m_s = DRIVE_ACCEL_M_S2 * dt_s;
        current + (target - current).clamp(-step_m_s, step_m_s)
    };
    // During the ride the wheels are stopped and nothing is commanded: the
    // car carries the truck through ordinary contact, and overriding its
    // velocity every step would be staging the thing under test.
    if !matches!(phase, Phase::Ride) {
        // Line up before entering. The doorway is only as wide as the car
        // and the truck arrives off-centre from leaning on the panel; a
        // truck that corrects its offset while driving forward wedges a
        // corner against a door leaf and stops there with the load up.
        let lining_up = matches!(phase, Phase::Board) && (target_z_m - chassis.z).abs() > 0.12;
        let forward_m_s = if lining_up {
            0.0
        } else {
            gain(target_x_m - chassis.x)
        };
        *commanded_x_m_s = ramp(*commanded_x_m_s, forward_m_s);
        *commanded_z_m_s = ramp(*commanded_z_m_s, gain(target_z_m - chassis.z));
        if let Some(mut body) = world.get_mut::<RigidBody>(site.chassis) {
            body.linear_velocity_m_s.x = *commanded_x_m_s;
            body.linear_velocity_m_s.z = *commanded_z_m_s;
            // Heading is commanded too. The drive wheels are not modelled,
            // so nothing steers the truck back once something knocks it
            // round -- and something does: brushing the panel torqued the
            // chassis 17 degrees, which put its front corner into the panel
            // body and wedged it there under 26 kN. A real base holds the
            // heading it is given.
            body.angular_velocity_rad_s = Vec3::ZERO;
        }
        if let Some(mut transform) = world.get_mut::<Transform3>(site.chassis) {
            transform.rotation = Quat::IDENTITY;
        }
    }
    for carriage in [site.fork, site.backrest] {
        if let Some(mut motor) = world.get_mut::<JointMotor>(carriage) {
            motor.target_position = phase.mast_target_m();
        }
    }
}

/// Returns the phase the mission is in after this step.
fn advance_phase(
    phase: Phase,
    chassis: Vec3,
    case: Vec3,
    presses: u32,
    elevator: &mut Elevator,
) -> Phase {
    match phase {
        Phase::Approach if (chassis.x - APPROACH_X_M).abs() < 0.06 => Phase::Engage,
        Phase::Engage if (chassis.x - ENGAGED_X_M).abs() < 0.04 => Phase::Lift,
        // Clear of the stand deck by a margin the solver cannot produce by
        // jitter, so the haul starts only once the load is genuinely up.
        Phase::Lift if case.y > CAR_HALF_M.y + STAND_TOP_Y_M + CASE_HALF_M.y + 0.12 => Phase::Haul,
        Phase::Haul
            if (chassis.x - PRESS_X_M).abs() < 0.25 && (chassis.z - PRESS_Z_M).abs() < 0.25 =>
        {
            Phase::Press
        }
        Phase::Press if presses > 0 => Phase::WaitForDoors,
        // Lined up on the doorway rather than exactly on its centreline:
        // the truck arrives having just leaned on the panel, and holding
        // out for the centreline outlasts the door hold.
        Phase::WaitForDoors if elevator.is_boardable(0) && chassis.z.abs() < 0.14 => Phase::Board,
        Phase::Board if (chassis.x - BOARD_X_M).abs() < 0.06 => {
            // Aboard: choose the destination, which is a separate act from
            // summoning the car.
            elevator.call(1).expect("select the upper floor");
            Phase::Ride
        }
        Phase::Ride if elevator.is_boardable(1) => Phase::DriveOut,
        Phase::DriveOut if (chassis.x - BAY_X_M).abs() < 0.12 => Phase::Place,
        Phase::Place if (chassis.x - BAY_X_M).abs() < 0.05 => Phase::Lower,
        // The case is down when it is resting on the outbound stand rather
        // than on the tines.
        Phase::Lower if case.y < OUTBOUND_TOP_Y_M + CASE_HALF_M.y + 0.03 => Phase::Withdraw,
        Phase::Withdraw if chassis.x > WITHDRAW_X_M - 0.08 => Phase::Done,
        other => other,
    }
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

fn run_mission(capture: bool, trace: bool) -> Mission {
    let spec = elevator_spec();
    spec.validate().expect("elevator specification");

    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    let mut world = World::new();
    let site = spawn_site(&mut world);
    let mut elevator = Elevator::new(spec.clone(), 0).expect("elevator");
    apply_elevator(&mut world, &site, &elevator);

    let mut button = CallButton::new(CallButtonSpec {
        center_world_m: BUTTON_CENTER_M,
        normal_world: BUTTON_NORMAL,
        radius_m: 0.10,
        travel_m: 0.02,
        press_force_n: 2.0,
        release_force_n: 1.0,
        // A lobby button summons the car to the floor it is on. Choosing a
        // destination is what the truck does once it is aboard.
        floor: 0,
    })
    .expect("call button");

    let dt = SimDuration::from_hertz(Hertz::new(PHYSICS_HZ));
    let dt_s = 1.0 / PHYSICS_HZ;
    let steps = (MAX_SECONDS * PHYSICS_HZ) as usize;
    let sample_every = (steps / FRAME_COUNT).max(1);

    // Settle before anything is commanded.
    for _ in 0..(PHYSICS_HZ as usize / 2) {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("settle");
    }

    let mut phase = Phase::Approach;
    let mut presses = 0_u32;
    let mut carry_slip_m: f64 = 0.0;
    let mut lift_height_m: f64 = 0.0;
    let mut carried_offset: Option<Vec3> = None;
    let mut frames = Vec::new();
    let mut order = vec![phase];
    let mut commanded_x_m_s = 0.0_f64;
    let mut commanded_z_m_s = 0.0_f64;

    for step in 0..steps {
        let chassis = translation(&world, site.chassis);

        drive_truck(
            &mut world,
            &site,
            phase,
            chassis,
            dt_s,
            &mut commanded_x_m_s,
            &mut commanded_z_m_s,
        );

        if read_call_button(&mut backend, physics_world, site.button, &mut button) {
            presses += 1;
            elevator.call(0).expect("summon the car to the lobby");
        }
        // A door hold is finite and a loaded truck lines up before it enters.
        // Losing the doors halfway through boarding leaves the truck nosed
        // against a closed leaf, which is what an operator presses the button
        // again for, so the mission does. The call is idempotent.
        if matches!(phase, Phase::WaitForDoors | Phase::Board) && !elevator.is_boardable(0) {
            elevator.call(0).expect("summon the car again");
        }

        elevator.update(dt_s).expect("elevator update");
        apply_elevator(&mut world, &site, &elevator);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");

        let chassis = translation(&world, site.chassis);
        let fork = translation(&world, site.fork);
        let case = translation(&world, site.case);

        // Once the load is off the stand, track how far it creeps across the
        // tines. This is the measurement that says the carry is real rather
        // than the case being glued on.
        let carrying = matches!(
            phase,
            Phase::Haul | Phase::Press | Phase::WaitForDoors | Phase::Board | Phase::Ride
        );
        if carrying {
            let offset = case - fork;
            match carried_offset {
                None => carried_offset = Some(offset),
                Some(first) => {
                    let slip = Vec3::new(offset.x - first.x, 0.0, offset.z - first.z).length();
                    carry_slip_m = carry_slip_m.max(slip);
                }
            }
        }
        // Measured while the load is still over the goods-in stand: once the
        // truck rides the lift, height above that stand is the building's, not
        // the mast's.
        if matches!(phase, Phase::Lift | Phase::Haul) {
            lift_height_m =
                lift_height_m.max(case.y - (CAR_HALF_M.y + STAND_TOP_Y_M + CASE_HALF_M.y));
        }

        phase = advance_phase(phase, chassis, case, presses, &mut elevator);

        if trace && step % 120 == 0 {
            eprintln!(
                "t={:5.2} {:24} chassis=({:+.3},{:+.3},{:+.3}) mast={:+.3} case=({:+.3},{:+.3},{:+.3})",
                step as f64 / PHYSICS_HZ,
                phase.label(),
                chassis.x,
                chassis.y,
                chassis.z,
                fork.y - (chassis.y + MAST_ANCHOR_Y_M),
                case.x,
                case.y,
                case.z
            );
        }
        if order.last() != Some(&phase) {
            order.push(phase);
        }

        if capture && step % sample_every == 0 && frames.len() < FRAME_COUNT {
            frames.push(Frame {
                car_y_m: elevator.car_height_m(),
                door_opening_m: elevator.door_opening_m(),
                chassis,
                fork,
                backrest: translation(&world, site.backrest),
                case,
                phase,
            });
        }
        if matches!(phase, Phase::Done) && !capture {
            break;
        }
        if matches!(phase, Phase::Done) && frames.len() >= FRAME_COUNT {
            break;
        }
    }

    let case_final = translation(&world, site.case);
    let delivered_floor = FLOOR_HEIGHTS_M
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (case_final.y - *a)
                .abs()
                .partial_cmp(&(case_final.y - *b).abs())
                .expect("finite")
        })
        .map_or(0, |(index, _)| index);

    assert_eq!(phase, Phase::Done, "the delivery did not finish");
    assert!(
        !matches!(elevator.state(), ElevatorState::Moving { .. })
            || elevator.door_opening_m() == 0.0,
        "the car travelled with its doors open"
    );
    Mission {
        frames,
        order,
        presses,
        carry_slip_m,
        lift_height_m,
        delivered_floor,
        case_final,
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
    // Built through `item_from_visual` rather than by hand: it folds the
    // shape's size into the transform, and an item assembled directly with a
    // unit scale draws every box as a 1 m cube whatever `size_m` says.
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(translation, Quat::IDENTITY),
        VisualShape::Box { size_m: half * 2.0 },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    scene.items.push(item);
}

/// Painted steel: the colour the caller asked for, with a sheen.
///
/// Metallic stays low. There is no image-based lighting in this shot, so a
/// surface with nothing to reflect renders as its reflections -- that is, as
/// black. At 0.65 the whole truck came out a silhouette.
fn push_steel(scene: &mut RenderScene, translation: Vec3, half: Vec3, color: [f32; 4]) {
    push_pbr(scene, translation, half, color, 0.45, 0.08, [0.0; 3]);
}

/// A stringer pallet: three deck boards on three blocks, drawn under the case.
///
/// The physics case is one box; this is what a box of that size is actually
/// carried on, and without it the load looks like it is floating on the tines.
fn push_pallet(scene: &mut RenderScene, center: Vec3) {
    const WOOD: [f32; 4] = [0.60, 0.45, 0.28, 1.0];
    const BOARD_HALF_Y: f64 = 0.012;
    const BLOCK_HALF_Y: f64 = 0.045;
    for offset in [-0.20, 0.0, 0.20] {
        push_pbr(
            scene,
            center + Vec3::new(0.0, BLOCK_HALF_Y, offset),
            Vec3::new(0.055, BLOCK_HALF_Y, 0.055),
            [0.50, 0.37, 0.22, 1.0],
            0.95,
            0.0,
            [0.0; 3],
        );
    }
    for offset in [-0.21, 0.0, 0.21] {
        push_pbr(
            scene,
            center + Vec3::new(0.0, 2.0 * BLOCK_HALF_Y + BOARD_HALF_Y, offset),
            Vec3::new(0.20, BOARD_HALF_Y, 0.055),
            WOOD,
            0.9,
            0.0,
            [0.0; 3],
        );
    }
}

/// The truck: counterweight body, mast rails, tines, wheels and guard.
///
/// Only the chassis, carriage and backrest exist in physics. The rest is what
/// those three shapes are part of, and it is drawn from their poses so it can
/// never disagree with them.
fn push_truck(scene: &mut RenderScene, frame: &Frame) {
    const BODYWORK: [f32; 4] = [0.95, 0.56, 0.06, 1.0];
    const DARK: [f32; 4] = [0.30, 0.32, 0.37, 1.0];
    const TYRE: [f32; 4] = [0.20, 0.20, 0.22, 1.0];

    let chassis = frame.chassis;
    // Counterweight at the back, cab deck in front of it.
    push_pbr(
        scene,
        chassis + Vec3::new(CHASSIS_HALF_M.x - 0.20, 0.03, 0.0),
        Vec3::new(0.20, CHASSIS_HALF_M.y - 0.02, CHASSIS_HALF_M.z),
        [0.88, 0.50, 0.05, 1.0],
        0.5,
        0.05,
        [0.0; 3],
    );
    push_pbr(
        scene,
        chassis,
        CHASSIS_HALF_M,
        BODYWORK,
        0.45,
        0.05,
        [0.0; 3],
    );
    // Overhead guard: four posts and a roof, the thing that makes a forklift
    // read as a forklift from any angle.
    for (dx, dz) in [
        (CHASSIS_HALF_M.x - 0.06, CHASSIS_HALF_M.z - 0.05),
        (CHASSIS_HALF_M.x - 0.06, -(CHASSIS_HALF_M.z - 0.05)),
        (-(CHASSIS_HALF_M.x - 0.10), CHASSIS_HALF_M.z - 0.05),
        (-(CHASSIS_HALF_M.x - 0.10), -(CHASSIS_HALF_M.z - 0.05)),
    ] {
        push_steel(
            scene,
            chassis + Vec3::new(dx, CHASSIS_HALF_M.y + 0.40, dz),
            Vec3::new(0.022, 0.40, 0.022),
            DARK,
        );
    }
    push_steel(
        scene,
        chassis + Vec3::new(0.0, CHASSIS_HALF_M.y + 0.81, 0.0),
        Vec3::new(CHASSIS_HALF_M.x - 0.04, 0.018, CHASSIS_HALF_M.z - 0.03),
        DARK,
    );
    // Wheels, as blocks rather than cylinders: at this scale the cylinder
    // tessellation reads as a lump rather than a wheel, and four lumps under
    // the chassis read as one.
    for (dx, dz) in [
        (CHASSIS_HALF_M.x - 0.16, CHASSIS_HALF_M.z + 0.005),
        (CHASSIS_HALF_M.x - 0.16, -(CHASSIS_HALF_M.z + 0.005)),
        (-(CHASSIS_HALF_M.x - 0.14), CHASSIS_HALF_M.z + 0.005),
        (-(CHASSIS_HALF_M.x - 0.14), -(CHASSIS_HALF_M.z + 0.005)),
    ] {
        push_pbr(
            scene,
            chassis + Vec3::new(dx, -CHASSIS_HALF_M.y + 0.005, dz),
            Vec3::new(0.105, 0.105, 0.035),
            TYRE,
            0.95,
            0.0,
            [0.0; 3],
        );
    }

    // Mast: two rails from the chassis front up past the carriage.
    let mast_x_m = chassis.x + FORK_REACH_M + FORK_HALF_M.x + 0.04;
    for sign in [-1.0, 1.0] {
        push_steel(
            scene,
            Vec3::new(
                mast_x_m,
                chassis.y + MAST_ANCHOR_Y_M + 0.44,
                chassis.z + sign * (FORK_HALF_M.z + 0.05),
            ),
            Vec3::new(0.035, 0.52, 0.030),
            DARK,
        );
    }
    // Two tines rather than one slab, on the carriage the physics solves.
    for sign in [-1.0, 1.0] {
        push_steel(
            scene,
            frame.fork + Vec3::new(0.0, 0.0, sign * 0.085),
            Vec3::new(FORK_HALF_M.x, FORK_HALF_M.y, 0.045),
            [0.72, 0.74, 0.78, 1.0],
        );
    }
    push_steel(scene, frame.backrest, BACKREST_HALF_M, DARK);
}

/// The parts of the site that never move: shaft, decks, racking, fixtures and
/// the two stands.
fn push_building(scene: &mut RenderScene) {
    const SHAFT: [f32; 4] = [0.33, 0.36, 0.42, 1.0];
    const CASE: [f32; 4] = [0.72, 0.55, 0.34, 1.0];
    const RACK_BEAM: [f32; 4] = [0.92, 0.55, 0.10, 1.0];
    const RACK_UPRIGHT: [f32; 4] = [0.16, 0.36, 0.60, 1.0];
    const STAND: [f32; 4] = [0.40, 0.43, 0.48, 1.0];
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

    // Racking down the ground-floor aisle: uprights, bracing and beam levels,
    // with pallets on them. Dressing -- the truck's route never crosses it.
    for bay in 0..4 {
        let x_m = SHAFT_X_M - 7.4 + f64::from(bay) * 1.62;
        for level in 0..3 {
            let y_m = 0.52 + f64::from(level) * 0.88;
            push_steel(
                scene,
                Vec3::new(x_m, y_m, RACK_Z_M),
                Vec3::new(0.76, 0.045, 0.05),
                RACK_BEAM,
            );
            if level < 2 && bay % 2 == 0 {
                push_pallet(scene, Vec3::new(x_m, y_m + 0.045, RACK_Z_M));
                push_pbr(
                    scene,
                    Vec3::new(x_m, y_m + 0.30, RACK_Z_M),
                    Vec3::new(0.24, 0.19, 0.24),
                    CASE,
                    0.9,
                    0.0,
                    [0.0; 3],
                );
            }
        }
        for side in [-1.0, 1.0] {
            push_steel(
                scene,
                Vec3::new(x_m + side * 0.76, 1.32, RACK_Z_M),
                Vec3::new(0.05, 1.32, 0.05),
                RACK_UPRIGHT,
            );
        }
    }

    // Ceiling fixtures, emissive so the shot has a light source in it.
    for bay in 0..4 {
        push_pbr(
            scene,
            Vec3::new(SHAFT_X_M - 6.6 + f64::from(bay) * 1.7, 2.78, -0.45),
            Vec3::new(0.52, 0.04, 0.10),
            [0.97, 0.98, 1.0, 1.0],
            0.35,
            0.0,
            [0.85, 0.87, 0.92],
        );
    }

    for (x_m, deck_y_m) in [
        (STAND_X_M, CAR_HALF_M.y),
        (OUTBOUND_X_M, FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y),
    ] {
        for sign in [-1.0, 1.0] {
            push_steel(
                scene,
                Vec3::new(x_m, deck_y_m + STAND_LEG_HALF_M.y, sign * STAND_LEG_Z_M),
                STAND_LEG_HALF_M,
                STAND,
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

fn append_site(scene: &mut RenderScene, frame: &Frame) {
    const CAR: [f32; 4] = [0.74, 0.77, 0.82, 1.0];
    const DOOR: [f32; 4] = [0.80, 0.84, 0.90, 1.0];
    const BUTTON_IDLE: [f32; 4] = [0.45, 0.47, 0.52, 1.0];
    const BUTTON_LIT: [f32; 4] = [0.99, 0.74, 0.20, 1.0];
    const CASE: [f32; 4] = [0.72, 0.55, 0.34, 1.0];

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
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for sign in [-1.0, 1.0] {
        push_pbr(
            scene,
            Vec3::new(
                SHAFT_X_M - CAR_HALF_M.x,
                frame.car_y_m + DOOR_HALF_M.y,
                sign * (doorway_z_m + frame.door_opening_m),
            ),
            DOOR_HALF_M,
            DOOR,
            0.45,
            0.08,
            [0.0; 3],
        );
    }

    let lit = !matches!(frame.phase, Phase::Approach | Phase::Engage | Phase::Lift);
    // Drawn larger than the collider on purpose: the physical button is 10 cm
    // across and would be three pixels at this scale, so the state it reports
    // would be invisible. The collider, and therefore the press, is unchanged.
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
        if lit { BUTTON_LIT } else { BUTTON_IDLE },
        0.4,
        0.1,
        if lit { [0.55, 0.38, 0.05] } else { [0.0; 3] },
    );

    push_truck(scene, frame);
    push_pallet(
        scene,
        frame.case - Vec3::new(0.0, CASE_HALF_M.y + 0.114, 0.0),
    );
    push_pbr(scene, frame.case, CASE_HALF_M, CASE, 0.92, 0.0, [0.0; 3]);
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = png::Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "12",
            "-i",
        ])
        .arg(frames_dir.join("frame-%03d.png"))
        .args([
            "-vf",
            "split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4",
        ])
        .arg(gif_path)
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("ffmpeg failed to build the gif"));
    }
    Ok(())
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let trace = std::env::args().any(|argument| argument == "--trace");
    let mission = run_mission(!smoke, trace);

    println!(
        "mission: {} press(es), lifted {:.3} m off the stand, worst carry slip {:.4} m, case delivered to floor {} at ({:.2}, {:.2}, {:.2})",
        mission.presses,
        mission.lift_height_m,
        mission.carry_slip_m,
        mission.delivered_floor,
        mission.case_final.x,
        mission.case_final.y,
        mission.case_final.z,
    );
    println!(
        "sequence: {}",
        mission
            .order
            .iter()
            .map(|phase| phase.label())
            .collect::<Vec<_>>()
            .join(" -> ")
    );

    assert!(
        mission.presses >= 1,
        "the truck never actuated the call button"
    );
    assert!(
        mission.lift_height_m > 0.12,
        "the case never came off the stand: {} m",
        mission.lift_height_m
    );
    assert_eq!(
        mission.delivered_floor, 1,
        "the case must end up on the upper floor"
    );
    // The truck has to be clear of what it delivered, or "delivered" only
    // means "still on the tines".
    assert!(
        mission.case_final.x < WITHDRAW_X_M,
        "the truck did not back off the case it set down"
    );

    if smoke {
        println!("smoke ok: the job completes headlessly");
        return;
    }

    let frames_dir =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-warehouse-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    backend.set_environment(warehouse_environment());
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    // A fixed viewpoint, deliberately: the subject is one route across two
    // floors, and an orbiting camera both fights the eye and destroys
    // inter-frame compression.
    let orbit = CameraOrbit {
        focus: Vec3::new(SHAFT_X_M - 1.95, 1.95, 0.0),
        yaw_rad: 0.46,
        // Larger pitch is nearer horizontal. At 1.44 the two decks were seen
        // edge-on and read as lines; a three-quarter view puts the load on a
        // surface the eye can see.
        pitch_rad: 1.16,
        // 10.6 m framed the job as a grey postage stamp in an empty room; 7.6 m
        // cut the upper floor off the top, which is where the delivery happens.
        distance_m: 8.3,
    };

    for (index, frame) in mission.frames.iter().enumerate() {
        let mut scene = RenderScene::default();
        append_site(&mut scene, frame);
        mesh_cache
            .resolve_scene(&mut scene, &[])
            .expect("resolve scene meshes");
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render warehouse frame");
        write_png(
            &frames_dir.join(format!("frame-{index:03}.png")),
            &output.color.rgba8,
        )
        .expect("write frame");
    }

    let media_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");
    let gif_path = media_dir.join("warehouse-logistics.gif");
    build_gif(&frames_dir, &gif_path).expect("build gif");
    println!("wrote {}", gif_path.display());
}
