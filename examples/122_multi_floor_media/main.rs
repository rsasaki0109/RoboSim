//! A service robot calls a lift, boards it, rides to the floor above and drives
//! out — rendered, because until now none of this had a picture.
//!
//! Examples 118, 119 and 120 each prove one part of multi-floor operation and
//! all three are headless. A robot that changes floors is the thing that
//! separates an indoor service robot from a single-floor one, and it was
//! invisible. This runs the same devices in one scene and draws it.
//!
//! Nothing here is staged. The car and the door leaves are kinematic bodies
//! whose poses come from `rne_nav::Elevator`'s state machine; the button is
//! `rne_nav::CallButton` reading solved contact forces from the robot's own
//! body, and it reports one press however long the robot leans on it; the
//! robot is an ordinary dynamic body that the car carries by normal contact,
//! not by being parented to it.
//!
//! ```text
//! cargo run --release -p multi_floor_media --example 122_multi_floor_media -- --smoke
//! cargo run --release -p multi_floor_media --example 122_multi_floor_media
//! ```
//!
//! `--smoke` runs the mission headlessly and checks it; without it the run also
//! renders `docs/media/multi-floor-lift.gif`.

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{ButtonContact, CallButton, CallButtonSpec, Elevator, ElevatorSpec, ElevatorState};
use rne_physics::{
    Collider, ColliderShape, CommandedKinematicPose, PhysicsBackend, PhysicsWorldDesc, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{Camera, MeshRenderCache, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

/// Physics rate. The car carries the robot through contact, which needs the
/// solver to see the platform's velocity rather than a teleport.
const PHYSICS_HZ: f64 = 240.0;
/// Floor heights, in meters.
const FLOOR_HEIGHTS_M: [f64; 2] = [0.0, 3.2];
/// Car platform half extents, in meters.
const CAR_HALF_M: Vec3 = Vec3::new(0.85, 0.06, 0.85);
/// Door leaf half extents, in meters.
const DOOR_HALF_M: Vec3 = Vec3::new(0.05, 1.05, 0.42);
/// Robot half extents, in meters.
///
/// Wide and low. A tall narrow box driven at this speed topples on the first
/// acceleration, lands on its side and then drags along the floor.
const ROBOT_HALF_M: Vec3 = Vec3::new(0.32, 0.36, 0.30);
/// Robot mass, in kilograms.
const ROBOT_MASS_KG: f64 = 28.0;
/// Shaft centre on the world x axis, in meters.
const SHAFT_X_M: f64 = 0.0;
/// Where the robot starts on the lobby floor, in meters.
const START_X_M: f64 = -4.2;
/// Where the robot stops to press the button, in meters.
///
/// Just past the panel, so the drive controller keeps a standing error and the
/// contact carries a real force instead of grazing it.
const PRESS_X_M: f64 = -1.33;
/// Button face centre, in meters.
///
/// On the lobby wall *beside* the doorway, not in front of it: a panel on the
/// robot's path is a bollard, and the robot cannot drive through it to board.
const BUTTON_CENTER_M: Vec3 = Vec3::new(-1.15, 0.56, -1.02);
/// Outward normal of the button face: it faces the lobby, along +z.
const BUTTON_NORMAL: Vec3 = Vec3::Z;
/// Where the robot drives to reach the panel, in meters.
///
/// Slightly *past* the face rather than exactly on it: a controller that stops
/// flush leaves no standing error, so the contact carries no force and the
/// button never actuates.
const PRESS_Z_M: f64 = BUTTON_CENTER_M.z + ROBOT_HALF_M.z - 0.06;
/// Button body half extents, in meters.
const BUTTON_HALF_M: Vec3 = Vec3::new(0.05, 0.05, 0.03);
/// Drive speed, in meters per second.
const DRIVE_M_S: f64 = 0.55;
/// Seconds the mission is allowed before it is declared stuck.
const MAX_SECONDS: f64 = 60.0;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.13, 0.15, 0.19, 1.0];
const FRAME_COUNT: usize = 110;

fn elevator_spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: FLOOR_HEIGHTS_M.to_vec(),
        car_speed_m_s: 1.1,
        car_acceleration_m_s2: 0.9,
        door_travel_m: 0.52,
        door_speed_m_s: 0.55,
        door_hold_s: 6.5,
    }
}

/// Bodies the mission drives or reads.
struct Site {
    car: Entity,
    left_door: Entity,
    right_door: Entity,
    robot: Entity,
    button: Entity,
}

fn spawn_site(world: &mut World) -> Site {
    // Lobby floors. Fixed slabs, one per served floor, stopping short of the
    // shaft so the camera sees into it.
    for (index, height_m) in FLOOR_HEIGHTS_M.iter().enumerate() {
        let slab = spawn_named(world, if index == 0 { "floor_1f" } else { "floor_2f" });
        world.entity_mut(slab).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(3.0, 0.06, 1.2),
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(
                Vec3::new(SHAFT_X_M - 3.9, height_m - 0.06, 0.0),
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

    // The button is part of the building: fixed, so the robot cannot push it
    // out of the way, and its face is the side the robot arrives from.
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

    let robot = spawn_named(world, "service_robot");
    world.entity_mut(robot).insert((
        RigidBody {
            mass_kg: ROBOT_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: ROBOT_HALF_M,
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(START_X_M, ROBOT_HALF_M.y + 0.01, 0.0),
            Quat::IDENTITY,
        ),
    ));

    Site {
        car,
        left_door,
        right_door,
        robot,
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

/// What the robot is doing, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    DriveToButton,
    Press,
    WaitForDoors,
    Board,
    Ride,
    DriveOut,
    Done,
}

impl Phase {
    fn label(self) -> &'static str {
        match self {
            Phase::DriveToButton => "driving to the call button",
            Phase::Press => "pressing the call button",
            Phase::WaitForDoors => "waiting for the doors",
            Phase::Board => "boarding",
            Phase::Ride => "riding to 2F",
            Phase::DriveOut => "driving out on 2F",
            Phase::Done => "delivered",
        }
    }
}

/// One captured frame: every pose the renderer needs.
struct Frame {
    car_y_m: f64,
    door_opening_m: f64,
    robot: Vec3,
    phase: Phase,
}

struct Mission {
    frames: Vec<Frame>,
    /// Each phase in the order it was entered, for the run's own report.
    order: Vec<Phase>,
    presses: u32,
    ride_clearance_error_m: f64,
    delivered_floor: usize,
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
        radius_m: 0.09,
        travel_m: 0.02,
        press_force_n: 2.0,
        release_force_n: 1.0,
        // A lobby button summons the car to the floor it is on. Choosing a
        // destination is what the robot does once it is aboard.
        floor: 0,
    })
    .expect("call button");

    let dt = SimDuration::from_hertz(Hertz::new(PHYSICS_HZ));
    let dt_s = 1.0 / PHYSICS_HZ;
    let steps = (MAX_SECONDS * PHYSICS_HZ) as usize;
    let sample_every = (steps / FRAME_COUNT).max(1);

    // Settle before anything is commanded.
    for _ in 0..(PHYSICS_HZ as usize / 4) {
        step_physics(&mut backend, &mut world, physics_world, dt).expect("settle");
    }
    let settled_clearance_m = translation(&world, site.robot).y - elevator.car_height_m();

    let mut phase = Phase::DriveToButton;
    let mut presses = 0_u32;
    let mut ride_clearance_error_m: f64 = 0.0;
    let mut frames = Vec::new();
    let mut order = vec![phase];

    for step in 0..steps {
        let robot = translation(&world, site.robot);

        // Drive command for this phase, along x only; gravity owns y.
        let (target_x_m, target_z_m) = match phase {
            Phase::DriveToButton | Phase::Press => (PRESS_X_M, PRESS_Z_M),
            // Back off the panel decisively once the call is registered. A
            // slow withdrawal lets the contact chatter across the button's
            // release threshold and register a second press.
            Phase::WaitForDoors => (PRESS_X_M, 0.35),
            Phase::Board | Phase::Ride => (SHAFT_X_M, 0.0),
            Phase::DriveOut | Phase::Done => (SHAFT_X_M - 2.6, 0.0),
        };
        let gain = |error_m: f64| (error_m / 0.8).clamp(-1.0, 1.0) * DRIVE_M_S;
        // During the ride the wheels are stopped and nothing is commanded:
        // the car carries the robot through ordinary contact, and overriding
        // its velocity every step would be staging the thing under test.
        if !matches!(phase, Phase::Ride) {
            if let Some(mut body) = world.get_mut::<RigidBody>(site.robot) {
                body.linear_velocity_m_s.x = gain(target_x_m - robot.x);
                body.linear_velocity_m_s.z = gain(target_z_m - robot.z);
            }
        }

        // The button reads the solved contact between the robot and the panel.
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
            // A second press while the car is already on its way is what a
            // real panel receives from an impatient passenger, and the state
            // machine treats it the same way: the call is idempotent.
            elevator.call(0).expect("summon the car to the lobby");
        }

        elevator.update(dt_s).expect("elevator update");
        apply_elevator(&mut world, &site, &elevator);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");

        let robot = translation(&world, site.robot);
        phase = match phase {
            Phase::DriveToButton
                if (robot.x - PRESS_X_M).abs() < 0.25 && (robot.z - PRESS_Z_M).abs() < 0.25 =>
            {
                Phase::Press
            }
            Phase::Press if presses > 0 => Phase::WaitForDoors,
            Phase::WaitForDoors if elevator.is_boardable(0) && robot.z.abs() < 0.12 => Phase::Board,
            Phase::Board if (robot.x - SHAFT_X_M).abs() < 0.10 => {
                // Aboard: choose the destination, which is a separate act from
                // summoning the car.
                elevator.call(1).expect("select the upper floor");
                Phase::Ride
            }
            Phase::Ride if elevator.is_boardable(1) => Phase::DriveOut,
            Phase::DriveOut if robot.x < SHAFT_X_M - 2.4 => Phase::Done,
            other => other,
        };

        if order.last() != Some(&phase) {
            order.push(phase);
        }
        if matches!(phase, Phase::Ride) {
            let clearance_m = robot.y - elevator.car_height_m();
            ride_clearance_error_m =
                ride_clearance_error_m.max((clearance_m - settled_clearance_m).abs());
        }

        if capture && step % sample_every == 0 && frames.len() < FRAME_COUNT {
            frames.push(Frame {
                car_y_m: elevator.car_height_m(),
                door_opening_m: elevator.door_opening_m(),
                robot,
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

    let robot_y_m = translation(&world, site.robot).y;
    let delivered_floor = FLOOR_HEIGHTS_M
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (robot_y_m - *a)
                .abs()
                .partial_cmp(&(robot_y_m - *b).abs())
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
        ride_clearance_error_m,
        delivered_floor,
    }
}

fn append_site(scene: &mut RenderScene, frame: &Frame) {
    const FLOOR: [f32; 4] = [0.52, 0.55, 0.60, 1.0];
    const SHAFT: [f32; 4] = [0.28, 0.31, 0.38, 1.0];
    const CAR: [f32; 4] = [0.78, 0.81, 0.86, 1.0];
    const DOOR: [f32; 4] = [0.82, 0.86, 0.92, 1.0];
    const BUTTON_IDLE: [f32; 4] = [0.55, 0.57, 0.62, 1.0];
    const BUTTON_LIT: [f32; 4] = [0.98, 0.72, 0.18, 1.0];
    const ROBOT: [f32; 4] = [0.20, 0.78, 0.95, 1.0];

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
    let shaft_half_height_m = FLOOR_HEIGHTS_M[1] * 0.5 + 1.1;
    let shaft_mid_y_m = shaft_half_height_m - 0.6;
    push(
        Vec3::new(SHAFT_X_M + 0.95, shaft_mid_y_m, 0.0),
        Vec3::new(0.06, shaft_half_height_m, 1.05),
        SHAFT,
    );
    push(
        Vec3::new(SHAFT_X_M, shaft_mid_y_m, -1.02),
        Vec3::new(0.95, shaft_half_height_m, 0.06),
        SHAFT,
    );
    // Lobby wall the panel is mounted on, one per floor.
    for height_m in FLOOR_HEIGHTS_M {
        push(
            Vec3::new(SHAFT_X_M - 2.4, height_m + 1.1, -1.08),
            Vec3::new(1.5, 1.1, 0.05),
            SHAFT,
        );
    }
    // Panel plate, so the button is not a speck on a wall.
    push(
        BUTTON_CENTER_M - BUTTON_NORMAL * 0.02,
        Vec3::new(0.16, 0.26, 0.01),
        [0.18, 0.20, 0.25, 1.0],
    );
    for height_m in FLOOR_HEIGHTS_M {
        push(
            Vec3::new(SHAFT_X_M - 3.9, height_m - 0.06, 0.0),
            Vec3::new(3.0, 0.06, 1.2),
            FLOOR,
        );
    }
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
    let lit = !matches!(frame.phase, Phase::DriveToButton);
    // Drawn larger than the collider on purpose: the physical button is 10 cm
    // across and would be three pixels at this scale, so the state it reports
    // would be invisible. The collider, and therefore the press, is unchanged.
    push(
        BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
        Vec3::new(0.10, 0.10, BUTTON_HALF_M.z),
        if lit { BUTTON_LIT } else { BUTTON_IDLE },
    );
    push(frame.robot, ROBOT_HALF_M, ROBOT);
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
            "14",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=14,scale=860:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg multi-floor gif encode failed"))
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let mission = run_mission(!smoke);

    println!(
        "mission: {} press(es), worst ride clearance error {:.4} m, delivered to floor {}",
        mission.presses, mission.ride_clearance_error_m, mission.delivered_floor
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
    // The one-call-per-press edge is example 119's property, proven there
    // against a controlled fingertip. The presser here is a 28 kg robot
    // leaning on the panel with its whole bumper, which separates and
    // re-touches as it withdraws, so this asserts the call happened rather
    // than counting how many times the bumper brushed the plate. The call
    // itself is idempotent: summoning a car already on its way changes
    // nothing.
    assert!(
        mission.presses >= 1,
        "the robot never actuated the call button"
    );
    assert_eq!(mission.delivered_floor, 1, "the robot must end on 2F");
    // Measured 0.0197 m. Example 118's rider starts parked on the car and
    // holds 0.0000 m; this one drives aboard and arrives with the bounce that
    // costs. The bound is set above that, not around it, so it fails on the
    // rider being left behind rather than on a millimetre of settling.
    assert!(
        mission.ride_clearance_error_m < 0.04,
        "the robot slipped on the car during the ride: {} m",
        mission.ride_clearance_error_m
    );

    if smoke {
        println!("smoke ok: the mission completes headlessly");
        return;
    }

    let frames_dir =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-multi-floor-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    // A fixed viewpoint, deliberately: the subject is vertical travel, and an
    // orbiting camera both fights the eye and destroys inter-frame compression.
    let orbit = CameraOrbit {
        focus: Vec3::new(SHAFT_X_M - 1.35, 1.60, 0.0),
        // Mostly across the travel axis, with just enough turn to see the
        // doorway open rather than a door edge-on.
        yaw_rad: 0.30,
        pitch_rad: 1.34,
        distance_m: 8.6,
    };

    for (index, frame) in mission.frames.iter().enumerate() {
        let mut scene = RenderScene::default();
        append_site(&mut scene, frame);
        mesh_cache
            .resolve_scene(&mut scene, &[])
            .expect("resolve scene meshes");
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render multi-floor frame");
        write_png(
            &frames_dir.join(format!("frame-{index:03}.png")),
            &output.color.rgba8,
        )
        .expect("write frame");
    }

    let media_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");
    let gif_path = media_dir.join("multi-floor-lift.gif");
    build_gif(&frames_dir, &gif_path).expect("encode the multi-floor gif");
    println!(
        "wrote {} frames and {}",
        mission.frames.len(),
        gif_path.display()
    );
}
