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
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{ButtonContact, CallButton, CallButtonSpec, Elevator, ElevatorSpec, ElevatorState};
use rne_physics::{
    Collider, ColliderShape, CommandedKinematicPose, JointMotor, JointMotorGainModel,
    PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, PrismaticJointDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{Camera, MeshRenderCache, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

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
/// Long enough that the chassis stops clear of the stand's legs while the tines
/// are fully under the load: at `ENGAGED_X_M` the chassis front face is 0.09 m
/// short of the legs, and a shorter reach simply jammed the truck against them.
const FORK_REACH_M: f64 = -0.88;
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
/// error, so the contact carries no force and the button never actuates.
const PRESS_Z_M: f64 = BUTTON_CENTER_M.z - CHASSIS_HALF_M.z + 0.05;
/// Button body half extents, in meters.
const BUTTON_HALF_M: Vec3 = Vec3::new(0.05, 0.05, 0.03);

/// Where the truck sets the case down on the upper floor, in meters.
const BAY_X_M: f64 = SHAFT_X_M - 2.35;
/// Where the truck ends up after backing off the delivered case, in meters.
const WITHDRAW_X_M: f64 = SHAFT_X_M - 1.25;

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
const CLEAR_COLOR: [f32; 4] = [0.11, 0.13, 0.17, 1.0];
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
                    half_extents_m: Vec3::new(3.9, 0.06, 1.5),
                },
                material: high_friction(),
                ..Collider::default()
            },
            // Flush with the car platform's top rather than 6 cm below it. A
            // step at the doorway is a trip hazard for a long truck with a
            // loaded fork out front, and it put the truck on its roof.
            Transform3::from_translation_rotation(
                Vec3::new(SHAFT_X_M - 4.8, *height_m, 0.0),
                Quat::IDENTITY,
            ),
        ));
    }

    // Goods-in stand: two legs with an open middle, which is the only reason a
    // fork can get under anything. A solid plinth would have to be shoved.
    for (index, sign) in [(0, -1.0), (1, 1.0)] {
        let leg = spawn_named(
            world,
            if index == 0 {
                "goods_in_leg_near"
            } else {
                "goods_in_leg_far"
            },
        );
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
                    STAND_X_M,
                    CAR_HALF_M.y + STAND_LEG_HALF_M.y,
                    sign * STAND_LEG_Z_M,
                ),
                Quat::IDENTITY,
            ),
        ));
    }

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
            Phase::Lower | Phase::Withdraw | Phase::Done => FORK_GROUND_M,
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

fn run_mission(capture: bool) -> Mission {
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
            Phase::Board | Phase::Ride => (SHAFT_X_M, 0.0, DRIVE_M_S),
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
            commanded_x_m_s = ramp(commanded_x_m_s, forward_m_s);
            commanded_z_m_s = ramp(commanded_z_m_s, gain(target_z_m - chassis.z));
            if let Some(mut body) = world.get_mut::<RigidBody>(site.chassis) {
                body.linear_velocity_m_s.x = commanded_x_m_s;
                body.linear_velocity_m_s.z = commanded_z_m_s;
            }
        }
        for carriage in [site.fork, site.backrest] {
            if let Some(mut motor) = world.get_mut::<JointMotor>(carriage) {
                motor.target_position = phase.mast_target_m();
            }
        }

        // The button reads the solved contact between the truck and the panel.
        let contacts: Vec<ButtonContact> = backend
            .contact_points(physics_world)
            .expect("contact points")
            .iter()
            .filter(|sample| sample.entity_a == site.button || sample.entity_b == site.button)
            .map(|sample| ButtonContact {
                point_world_m: sample.point_world_m,
                normal_force_n: sample.normal_force_n,
            })
            .collect();
        button.update(&contacts);
        if button.just_pressed() {
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
        lift_height_m = lift_height_m.max(case.y - (CAR_HALF_M.y + STAND_TOP_Y_M + CASE_HALF_M.y));

        phase = match phase {
            Phase::Approach if (chassis.x - APPROACH_X_M).abs() < 0.06 => Phase::Engage,
            Phase::Engage if (chassis.x - ENGAGED_X_M).abs() < 0.04 => Phase::Lift,
            // Clear of the stand deck by a margin the solver cannot produce by
            // jitter, so the haul starts only once the load is genuinely up.
            Phase::Lift if case.y > CAR_HALF_M.y + STAND_TOP_Y_M + CASE_HALF_M.y + 0.12 => {
                Phase::Haul
            }
            Phase::Haul
                if (chassis.x - PRESS_X_M).abs() < 0.25 && (chassis.z - PRESS_Z_M).abs() < 0.25 =>
            {
                Phase::Press
            }
            Phase::Press if presses > 0 => Phase::WaitForDoors,
            // Lined up on the doorway rather than exactly on its centreline:
            // the truck arrives having just leaned on the panel, and holding
            // out for the centreline outlasts the door hold.
            Phase::WaitForDoors if elevator.is_boardable(0) && chassis.z.abs() < 0.14 => {
                Phase::Board
            }
            Phase::Board if (chassis.x - SHAFT_X_M).abs() < 0.10 => {
                // Aboard: choose the destination, which is a separate act from
                // summoning the car.
                elevator.call(1).expect("select the upper floor");
                Phase::Ride
            }
            Phase::Ride if elevator.is_boardable(1) => Phase::DriveOut,
            Phase::DriveOut if (chassis.x - BAY_X_M).abs() < 0.12 => Phase::Place,
            Phase::Place if (chassis.x - BAY_X_M).abs() < 0.05 => Phase::Lower,
            // The case is down when it is resting on the upper deck rather than
            // on the tines.
            Phase::Lower if case.y < FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y + CASE_HALF_M.y + 0.05 => {
                Phase::Withdraw
            }
            Phase::Withdraw if chassis.x > WITHDRAW_X_M - 0.08 => Phase::Done,
            other => other,
        };

        if std::env::var("TRACE").is_ok() && step % 120 == 0 {
            eprintln!(
                "t={:5.2} phase={:?} z={:+.3} board={} presses={} vx={:+.4} chassis=({:+.3},{:+.3}) pitch={:+.3} fork=({:+.3},{:+.3}) disp={:+.3} case=({:+.3},{:+.3}) target_x={:+.3}",
                step as f64 / PHYSICS_HZ, phase, chassis.z, elevator.is_boardable(0), presses,
                world.get::<RigidBody>(site.chassis).map_or(0.0, |b| b.linear_velocity_m_s.x),
                chassis.x, chassis.y,
                world.get::<Transform3>(site.chassis).map_or(0.0, |t| 2.0 * f64::atan2(t.rotation.z, t.rotation.w)),
                fork.x, fork.y, fork.y - (chassis.y + MAST_ANCHOR_Y_M),
                case.x, case.y, target_x_m
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

fn append_site(scene: &mut RenderScene, frame: &Frame) {
    const DECK: [f32; 4] = [0.50, 0.53, 0.58, 1.0];
    const SHAFT: [f32; 4] = [0.26, 0.29, 0.35, 1.0];
    const CAR: [f32; 4] = [0.76, 0.79, 0.85, 1.0];
    const DOOR: [f32; 4] = [0.82, 0.86, 0.92, 1.0];
    const BUTTON_IDLE: [f32; 4] = [0.55, 0.57, 0.62, 1.0];
    const BUTTON_LIT: [f32; 4] = [0.98, 0.72, 0.18, 1.0];
    const TRUCK: [f32; 4] = [0.96, 0.62, 0.09, 1.0];
    const MAST: [f32; 4] = [0.20, 0.22, 0.27, 1.0];
    const CASE: [f32; 4] = [0.74, 0.56, 0.33, 1.0];
    const RACK: [f32; 4] = [0.33, 0.37, 0.44, 1.0];
    const STAND: [f32; 4] = [0.42, 0.45, 0.51, 1.0];
    const BAY: [f32; 4] = [0.16, 0.72, 0.42, 1.0];

    let mut push = |translation: Vec3, half: Vec3, color: [f32; 4]| {
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(translation, Quat::IDENTITY),
            VisualShape::Box { size_m: half * 2.0 },
            color,
            Transform3::IDENTITY,
        ));
    };

    // Shaft walls, so the car reads as travelling inside something rather than
    // floating. Back plus one side; the open side is the camera's cutaway.
    let shaft_half_height_m = FLOOR_HEIGHTS_M[1] * 0.5 + 1.2;
    let shaft_mid_y_m = shaft_half_height_m - 0.6;
    push(
        Vec3::new(SHAFT_X_M + 1.05, shaft_mid_y_m, 0.0),
        Vec3::new(0.06, shaft_half_height_m, 1.15),
        SHAFT,
    );
    push(
        Vec3::new(SHAFT_X_M, shaft_mid_y_m, -1.12),
        Vec3::new(1.05, shaft_half_height_m, 0.06),
        SHAFT,
    );
    // Back wall of the aisle, one per floor, and the panel it carries.
    for height_m in FLOOR_HEIGHTS_M {
        push(
            Vec3::new(SHAFT_X_M - 4.4, height_m + 1.25, -1.18),
            Vec3::new(4.3, 1.25, 0.05),
            SHAFT,
        );
        push(
            Vec3::new(SHAFT_X_M - 4.8, height_m, 0.0),
            Vec3::new(3.9, 0.06, 1.5),
            DECK,
        );
    }
    // Racking down the ground-floor aisle: uprights and two beam levels. This
    // is dressing, not physics -- the truck's route never crosses it.
    for bay in 0..4 {
        let x_m = SHAFT_X_M - 7.6 + f64::from(bay) * 1.55;
        for level in 0..2 {
            push(
                Vec3::new(x_m, 0.62 + f64::from(level) * 0.92, -0.95),
                Vec3::new(0.70, 0.04, 0.16),
                RACK,
            );
        }
        for side in [-1.0, 1.0] {
            push(
                Vec3::new(x_m + side * 0.70, 0.86, -0.95),
                Vec3::new(0.05, 0.86, 0.16),
                RACK,
            );
        }
    }
    // Outbound bay marking on the upper deck.
    push(
        Vec3::new(
            BAY_X_M + FORK_REACH_M,
            FLOOR_HEIGHTS_M[1] + CAR_HALF_M.y + 0.005,
            0.0,
        ),
        Vec3::new(0.34, 0.005, 0.30),
        BAY,
    );

    push(
        Vec3::new(STAND_X_M, CAR_HALF_M.y + STAND_LEG_HALF_M.y, -STAND_LEG_Z_M),
        STAND_LEG_HALF_M,
        STAND,
    );
    push(
        Vec3::new(STAND_X_M, CAR_HALF_M.y + STAND_LEG_HALF_M.y, STAND_LEG_Z_M),
        STAND_LEG_HALF_M,
        STAND,
    );
    push(Vec3::new(SHAFT_X_M, frame.car_y_m, 0.0), CAR_HALF_M, CAR);
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for sign in [-1.0, 1.0] {
        push(
            Vec3::new(
                SHAFT_X_M - CAR_HALF_M.x,
                frame.car_y_m + DOOR_HALF_M.y,
                sign * (doorway_z_m + frame.door_opening_m),
            ),
            DOOR_HALF_M,
            DOOR,
        );
    }
    let lit = !matches!(frame.phase, Phase::Approach | Phase::Engage | Phase::Lift);
    // Drawn larger than the collider on purpose: the physical button is 10 cm
    // across and would be three pixels at this scale, so the state it reports
    // would be invisible. The collider, and therefore the press, is unchanged.
    push(
        BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
        Vec3::new(0.10, 0.10, BUTTON_HALF_M.z),
        if lit { BUTTON_LIT } else { BUTTON_IDLE },
    );

    push(frame.chassis, CHASSIS_HALF_M, TRUCK);
    // Mast uprights, drawn between the chassis and the carriage so the lift
    // reads as a mechanism rather than a floating slab.
    let mast_x_m = frame.chassis.x - CHASSIS_HALF_M.x - 0.05;
    let mast_half_height_m = 0.40;
    push(
        Vec3::new(
            mast_x_m,
            frame.chassis.y + MAST_ANCHOR_Y_M + mast_half_height_m,
            0.0,
        ),
        Vec3::new(0.04, mast_half_height_m, FORK_HALF_M.z),
        MAST,
    );
    push(frame.fork, FORK_HALF_M, MAST);
    push(frame.backrest, BACKREST_HALF_M, MAST);
    push(frame.case, CASE_HALF_M, CASE);
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
    let mission = run_mission(!smoke);

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
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    // A fixed viewpoint, deliberately: the subject is one route across two
    // floors, and an orbiting camera both fights the eye and destroys
    // inter-frame compression.
    let orbit = CameraOrbit {
        focus: Vec3::new(SHAFT_X_M - 2.30, 1.55, 0.0),
        yaw_rad: 0.34,
        pitch_rad: 1.36,
        distance_m: 10.6,
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
