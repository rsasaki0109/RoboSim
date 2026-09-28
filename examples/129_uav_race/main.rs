//! Four racing drones fly three laps of a gate course at night.
//!
//! Each drone is a `MultirotorFlight` body: a position loop that commands a
//! bounded velocity, a velocity loop that commands a bounded acceleration,
//! and an attitude that tilts with that acceleration up to the drone's
//! limit. It is flown by chasing a point on the course a speed-dependent
//! distance ahead, so the position loop asks for the speed its own profile
//! allows at that point of the lap. The four fly the same course in four
//! lanes, one per corner of the gate opening, so a faster drone can pass a
//! slower one without either leaving its lane.
//!
//! Every gate crossing is checked against the opening: where the drone
//! crossed the gate's plane, and how much room it had to the frame.
//!
//! ```text
//! cargo run --release -p uav_race --example 129_uav_race -- --smoke
//! cargo run --release -p uav_race --example 129_uav_race
//! ```

mod course;
mod render;

use course::{Course, GATES, GATE_OPENING_M};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::RigidBody;
use rne_robot::{command_multirotor, multirotor_flight, MultirotorFlight};
use rne_world::Transform3;
use std::path::PathBuf;

const SIM_HZ: f64 = 200.0;
/// Rendering samples per second.
pub(crate) const RECORD_HZ: f64 = 12.0;
const LAPS: usize = 3;
/// Radius of a drone, props included.
pub(crate) const DRONE_RADIUS_M: f64 = 0.22;
/// Each drone's lane: its offset (left, up) from the course line.
const LANES: [(f64, f64); 4] = [(0.4, 0.4), (-0.4, 0.4), (0.4, -0.4), (-0.4, -0.4)];

/// One drone: its performance and its colours.
#[derive(Clone, Copy)]
pub(crate) struct Pilot {
    pub(crate) name: &'static str,
    accel_m_s2: f64,
    tilt_rad: f64,
    top_m_s: f64,
    pub(crate) frame: [f32; 4],
    pub(crate) led: [f32; 3],
}

/// Slowest first: the start order.
pub(crate) const PILOTS: [Pilot; 4] = [
    Pilot {
        name: "cyan",
        accel_m_s2: 15.0,
        tilt_rad: 0.98,
        top_m_s: 19.0,
        frame: [0.10, 0.70, 0.85, 1.0],
        led: [0.1, 0.9, 1.0],
    },
    Pilot {
        name: "magenta",
        accel_m_s2: 17.0,
        tilt_rad: 1.03,
        top_m_s: 21.0,
        frame: [0.85, 0.15, 0.65, 1.0],
        led: [1.0, 0.15, 0.8],
    },
    Pilot {
        name: "lime",
        accel_m_s2: 19.0,
        tilt_rad: 1.08,
        top_m_s: 23.0,
        frame: [0.45, 0.85, 0.15, 1.0],
        led: [0.5, 1.0, 0.1],
    },
    Pilot {
        name: "orange",
        accel_m_s2: 21.0,
        tilt_rad: 1.13,
        top_m_s: 25.0,
        frame: [0.95, 0.45, 0.08, 1.0],
        led: [1.0, 0.5, 0.05],
    },
];

struct Drone {
    entity: Entity,
    pilot: Pilot,
    lane: (f64, f64),
    profile: Vec<f64>,
    index: usize,
    progress_m: f64,
    previous_position: Vec3,
    /// Gates crossed so far, in course order.
    gates_passed: usize,
    gates_missed: usize,
    min_clearance_m: f64,
    lap_times_s: Vec<f64>,
    finished: bool,
    /// When this drone may leave its pad.
    start_s: f64,
    pad: Vec3,
}

/// One recorded moment, for rendering.
pub(crate) struct Moment {
    pub(crate) time_s: f64,
    pub(crate) drones: Vec<Transform3>,
}

struct Report {
    finishing_order: Vec<&'static str>,
    /// Passes on the course: time, passing drone, passed drone.
    passes: Vec<(f64, usize, usize)>,
    lap_times: Vec<(&'static str, Vec<f64>)>,
    overtakes: usize,
    gates_passed: usize,
    gates_missed: usize,
    min_gate_clearance_m: f64,
    min_separation_m: f64,
    top_speed_m_s: f64,
    moments: Vec<Moment>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let course = Course::build();
    let report = race(&course, !smoke);
    print_report(&report, &course);
    check(&report);
    if smoke {
        println!("smoke ok");
        return;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let gif = root.join("docs/media/uav-race.gif");
    render::render_race(
        &course,
        &report.moments,
        &report.passes,
        &root.join("target/rne-uav-race-frames"),
        &gif,
    )
    .expect("render the race");
    println!("wrote {}", gif.display());
}

fn flight(pilot: &Pilot) -> MultirotorFlight {
    MultirotorFlight {
        max_horizontal_speed_m_s: pilot.top_m_s,
        max_climb_speed_m_s: 12.0,
        max_acceleration_m_s2: pilot.accel_m_s2,
        max_yaw_rate_rad_s: 6.0,
        max_tilt_rad: pilot.tilt_rad,
        position_gain_s_inv: 6.0,
        velocity_gain_s_inv: 12.0,
        attitude_response_s: 0.05,
        ..MultirotorFlight::default()
    }
}

/// Yaw that points the body's +x along `direction`.
fn heading_yaw(direction: Vec3) -> f64 {
    (-direction.z).atan2(direction.x)
}

fn spawn(world: &mut World, course: &Course) -> Vec<Drone> {
    let n = course.len();
    // Start pads 18 m before the first gate, one per lane, on the ground:
    // from 7 m the drones were still climbing into their lanes at the gate.
    let start = (course.gate_index[0] + n - 180) % n;
    PILOTS
        .iter()
        .zip(LANES)
        .map(|(pilot, lane)| {
            // A staggered grid under the lanes: two lanes share each side and
            // differ only in height, so the upper lane's pad sits 1.5 m behind
            // the lower lane's. Pads at the lanes' own offsets put two drones
            // on one pad; pads in a row across had drones crossing each
            // other's climb.
            let behind = if lane.1 > 0.0 { 15 } else { 0 };
            let beside = course.at(start + n - behind, (lane.0, 0.0));
            let pad = Vec3::new(beside.x, 0.12, beside.z);
            let entity = spawn_named(world, pilot.name);
            let mut body = flight(pilot);
            body.yaw_rad = heading_yaw(course.tangent[start]);
            body.target_position_m = pad;
            body.target_yaw_rad = body.yaw_rad;
            world.entity_mut(entity).insert((
                body,
                Transform3::from_translation_rotation(pad, Quat::from_rotation_y(body.yaw_rad)),
                RigidBody::default(),
            ));
            let lateral = 0.75 * pilot.accel_m_s2.min(9.81 * pilot.tilt_rad.tan());
            Drone {
                entity,
                pilot: *pilot,
                lane,
                profile: course.speed_profile(lateral, 0.6 * pilot.accel_m_s2, pilot.top_m_s),
                index: start,
                progress_m: -(course.gate_index[0] as f64 - start as f64).rem_euclid(n as f64)
                    * 0.1,
                previous_position: pad,
                gates_passed: 0,
                gates_missed: 0,
                min_clearance_m: f64::INFINITY,
                lap_times_s: Vec::new(),
                finished: false,
                start_s: 0.0,
                pad,
            }
        })
        .collect()
}

/// A pursuit start: the slowest drone leaves first, and each faster one
/// waits 80 % of the time its profile says it would gain over the race, so
/// the faster drones have to catch and pass on the course.
fn handicap(drones: &mut [Drone], course: &Course) {
    let race_time =
        |drone: &Drone| LAPS as f64 * drone.profile.iter().map(|v| 0.1 / v.max(1.0)).sum::<f64>();
    let slowest = drones.iter().map(race_time).fold(0.0, f64::max);
    for drone in drones.iter_mut() {
        drone.start_s = 0.8 * (slowest - race_time(drone));
    }
    let _ = course;
}

fn race(course: &Course, record: bool) -> Report {
    let mut world = World::new();
    let mut drones = spawn(&mut world, course);
    handicap(&mut drones, course);
    let dt_s = 1.0 / SIM_HZ;
    let dt = SimDuration::from_seconds(Seconds::new(dt_s));
    let n = course.len();
    let lap_m = course.length_m();
    let (mut min_separation_m, mut top_speed_m_s) = (f64::INFINITY, 0.0_f64);
    let mut moments = Vec::new();
    let mut passes = Vec::new();
    // Which of each pair is ahead on the course, once both have left.
    let mut ahead: Vec<Vec<Option<bool>>> = vec![vec![None; drones.len()]; drones.len()];
    let record_every = (SIM_HZ / RECORD_HZ).round() as usize;
    let mut step = 0_usize;
    while drones.iter().any(|drone| !drone.finished) {
        assert!(step < (300.0 * SIM_HZ) as usize, "the race never finished");
        let time_s = step as f64 * dt_s;
        for drone in &mut drones {
            let position = world
                .get::<Transform3>(drone.entity)
                .expect("pose")
                .translation;
            let index = course.project(position, drone.index, 300);
            let advanced = ((index + n - drone.index) % n) as f64;
            let advanced = if advanced > n as f64 / 2.0 {
                advanced - n as f64
            } else {
                advanced
            };
            drone.index = index;
            drone.progress_m += advanced * 0.1;
            check_gates(drone, course, position);
            drone.previous_position = position;
            if !drone.finished && drone.progress_m >= lap_m * (drone.lap_times_s.len() + 1) as f64 {
                let previous: f64 = drone.lap_times_s.iter().sum::<f64>() + drone.start_s;
                drone.lap_times_s.push(time_s - previous);
                drone.finished = drone.lap_times_s.len() >= LAPS;
            }
            // Chase a point ahead: the position loop's gain turns the
            // distance into the commanded speed, so the lookahead is the
            // profile speed divided by that gain.
            let flight = world.get::<MultirotorFlight>(drone.entity).expect("flight");
            let speed = if drone.finished {
                4.0
            } else {
                drone.profile[index]
            };
            let lookahead = ((speed / flight.position_gain_s_inv) / 0.1)
                .round()
                .max(10.0) as usize;
            let target = if time_s < drone.start_s {
                drone.pad
            } else {
                course.at(index + lookahead, drone.lane)
            };
            let yaw = heading_yaw(course.tangent[(index + lookahead / 2) % n]);
            command_multirotor(&mut world, drone.entity, target, yaw);
        }
        multirotor_flight(&mut world, dt);
        for a in 0..drones.len() {
            for b in 0..drones.len() {
                let flying = |d: &Drone| time_s >= d.start_s && !d.finished;
                if a == b || !flying(&drones[a]) || !flying(&drones[b]) {
                    continue;
                }
                // A metre either way settles who is ahead; level is no change.
                let lead = drones[a].progress_m - drones[b].progress_m;
                let now = if lead > 1.0 {
                    Some(true)
                } else if lead < -1.0 {
                    Some(false)
                } else {
                    ahead[a][b]
                };
                if ahead[a][b] == Some(false) && now == Some(true) {
                    passes.push((time_s, a, b));
                }
                ahead[a][b] = now;
            }
        }
        for (i, drone) in drones.iter().enumerate() {
            let a = world
                .get::<Transform3>(drone.entity)
                .expect("a")
                .translation;
            let flight = world.get::<MultirotorFlight>(drone.entity).expect("flight");
            top_speed_m_s = top_speed_m_s.max(flight.velocity_m_s.length());
            for other in drones.iter().skip(i + 1) {
                let b = world
                    .get::<Transform3>(other.entity)
                    .expect("b")
                    .translation;
                // Airborne only: the start pads sit side by side.
                if a.y > 1.0 && b.y > 1.0 {
                    min_separation_m = min_separation_m.min((a - b).length());
                }
            }
        }
        if record && step.is_multiple_of(record_every) {
            moments.push(Moment {
                time_s,
                drones: drones
                    .iter()
                    .map(|drone| *world.get::<Transform3>(drone.entity).expect("pose"))
                    .collect(),
            });
        }
        step += 1;
    }
    let mut report = finish(drones, min_separation_m, top_speed_m_s, moments);
    report.passes = passes;
    report
}

/// Checks whether the drone crossed the plane of the gate it is flying
/// toward this step, and if so where: inside the opening with room for its
/// props, or not.
fn check_gates(drone: &mut Drone, course: &Course, position: Vec3) {
    let gate = drone.gates_passed % GATES.len();
    let (x, y, z) = GATES[gate];
    let centre = Vec3::new(x, y, z);
    let index = course.gate_index[gate];
    let normal = course.tangent[index];
    let before = (drone.previous_position - centre).dot(normal);
    let after = (position - centre).dot(normal);
    if before < 0.0 && after >= 0.0 && (position - centre).length() < 6.0 {
        let t = before / (before - after);
        let crossing = drone.previous_position + (position - drone.previous_position) * t;
        let offset = crossing - centre;
        let side = offset.dot(course.left[index]).abs();
        let height = offset.dot(course.up[index]).abs();
        let clearance = 0.5 * GATE_OPENING_M - side.max(height) - DRONE_RADIUS_M;
        drone.min_clearance_m = drone.min_clearance_m.min(clearance);
        if clearance < 0.0 {
            drone.gates_missed += 1;
        }
        drone.gates_passed += 1;
    }
}

fn finish(
    drones: Vec<Drone>,
    min_separation_m: f64,
    top_speed_m_s: f64,
    moments: Vec<Moment>,
) -> Report {
    // Finishing order is who crosses the line first: start offset included.
    let total = |drone: &Drone| drone.start_s + drone.lap_times_s.iter().sum::<f64>();
    let mut order: Vec<usize> = (0..drones.len()).collect();
    order.sort_by(|a, b| total(&drones[*a]).total_cmp(&total(&drones[*b])));
    let rank: Vec<usize> = (0..drones.len())
        .map(|i| order.iter().position(|o| *o == i).expect("ranked"))
        .collect();
    // Net passes: pairs that finished in the opposite order to the start.
    let overtakes = (0..drones.len())
        .flat_map(|a| (a + 1..drones.len()).map(move |b| (a, b)))
        .filter(|(a, b)| rank[*a] > rank[*b])
        .count();
    Report {
        passes: Vec::new(),
        finishing_order: order.iter().map(|i| drones[*i].pilot.name).collect(),
        lap_times: drones
            .iter()
            .map(|drone| (drone.pilot.name, drone.lap_times_s.clone()))
            .collect(),
        overtakes,
        gates_passed: drones.iter().map(|drone| drone.gates_passed).sum(),
        gates_missed: drones.iter().map(|drone| drone.gates_missed).sum(),
        min_gate_clearance_m: drones
            .iter()
            .map(|drone| drone.min_clearance_m)
            .fold(f64::INFINITY, f64::min),
        min_separation_m,
        top_speed_m_s,
        moments,
    }
}

fn print_report(report: &Report, course: &Course) {
    println!(
        "lap length {:.0} m, {} gates",
        course.length_m(),
        GATES.len()
    );
    for (name, laps) in &report.lap_times {
        println!(
            "{name:>8}: {}",
            laps.iter()
                .map(|t| format!("{t:.2} s"))
                .collect::<Vec<_>>()
                .join("  ")
        );
    }
    println!("finish: {}", report.finishing_order.join(" > "));
    for (time_s, a, b) in &report.passes {
        println!(
            "  {time_s:6.2} s  {} passes {}",
            PILOTS[*a].name, PILOTS[*b].name
        );
    }
    println!(
        "gates {} crossed, {} outside the opening; tightest clearance to a frame {:.2} m",
        report.gates_passed, report.gates_missed, report.min_gate_clearance_m
    );
    println!(
        "net passes {}, closest two drones {:.2} m apart, top speed {:.1} m/s",
        report.overtakes, report.min_separation_m, report.top_speed_m_s
    );
}

fn check(report: &Report) {
    assert_eq!(report.gates_missed, 0, "a drone flew outside a gate");
    assert!(
        report.gates_passed >= PILOTS.len() * GATES.len() * LAPS,
        "a gate was skipped"
    );
    // Lanes are 0.8 m apart; the closest two drones came was 0.80 m.
    assert!(
        report.min_separation_m > 2.0 * DRONE_RADIUS_M + 0.2,
        "two drones came within {:.2} m",
        report.min_separation_m
    );
    // The tightest crossing left 0.56 m between props and frame.
    assert!(report.min_gate_clearance_m > 0.3, "a drone grazed a gate");
    // The pursuit start makes the faster drones catch and pass: all six
    // pairs changed places on the course.
    assert!(
        report.passes.len() >= 4,
        "only {} passes",
        report.passes.len()
    );
    assert_eq!(
        report.finishing_order[0], "orange",
        "the fastest drone did not win"
    );
}
