//! Four open-wheel cars race two laps of a circuit on the tire-limited dynamic
//! bicycle model.
//!
//! Each car follows the minimum-curvature racing line with pure pursuit at the
//! speed its own grip and power allow, and passes a slower car by moving onto
//! an offset line beside it. The grid is in reverse order of pace, so the
//! faster cars have to overtake. `VehicleDynamics` decides whether a car makes
//! a corner: the speed profile asks for a share of the grip, and a car that
//! asks for too much runs wide.
//!
//! ```text
//! cargo run --release -p car_race --example 128_car_race -- --smoke
//! cargo run --release -p car_race --example 128_car_race
//! ```

mod render;
mod track;

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::RigidBody;
use rne_robot::{pure_pursuit_steering, vehicle_dynamics, AckermannDrive, VehicleDynamics};
use rne_world::Transform3;
use std::path::PathBuf;
use track::{grip_m_s2, RacingLine, Track, WIDTH_M};

const SIM_HZ: f64 = 200.0;
const LAPS: usize = 3;
/// Car length and width, for spacing and contact checks.
pub(crate) const CAR_HALF_LENGTH_M: f64 = 2.6;
pub(crate) const CAR_HALF_WIDTH_M: f64 = 1.0;
/// Friction coefficient of the slicks on this asphalt, downforce folded in.
const FRICTION: f64 = 1.7;
/// Seconds after the start before anyone starts a pass: the grid launches
/// nose to tail and every car is "closing" on the one ahead.
const START_SETTLE_S: f64 = 4.0;
/// How far off the racing line a passing car drives.
const PASSING_OFFSET_M: f64 = 3.6;

/// One driver and car: how much of the grip it uses, how hard it accelerates,
/// and its livery.
#[derive(Clone, Copy)]
pub(crate) struct Entry {
    pub name: &'static str,
    grip_use: f64,
    accel_m_s2: f64,
    pub livery: [f32; 4],
    pub accent: [f32; 4],
}

/// Slowest first: the grid order.
const ENTRIES: [Entry; 4] = [
    Entry {
        name: "yellow",
        grip_use: 0.76,
        accel_m_s2: 6.5,
        livery: [0.98, 0.78, 0.08, 1.0],
        accent: [0.10, 0.10, 0.12, 1.0],
    },
    Entry {
        name: "green",
        grip_use: 0.81,
        accel_m_s2: 7.3,
        livery: [0.05, 0.45, 0.30, 1.0],
        accent: [0.85, 0.85, 0.80, 1.0],
    },
    Entry {
        name: "blue",
        grip_use: 0.845,
        accel_m_s2: 8.1,
        livery: [0.10, 0.22, 0.62, 1.0],
        accent: [0.95, 0.30, 0.12, 1.0],
    },
    Entry {
        name: "red",
        grip_use: 0.875,
        accel_m_s2: 9.0,
        livery: [0.82, 0.06, 0.08, 1.0],
        accent: [0.95, 0.95, 0.95, 1.0],
    },
];

/// A car in the race.
struct Car {
    entity: Entity,
    entry: Entry,
    speed_profile: Vec<f64>,
    /// Speed profiles of the passing lines to the left and right of the
    /// racing line, from their own curvature.
    passing_profiles: [Vec<f64>; 2],
    /// Nearest centre-line sample, carried from step to step.
    index: usize,
    /// Distance raced, in meters: laps times lap length plus progress.
    progress_m: f64,
    /// Current and wanted offset from the racing line.
    shift_m: f64,
    shift_target_m: f64,
    /// The car being passed while a pass is on, and when the pass began.
    passing: Option<(usize, f64)>,
    /// Whether a car alongside leaves this one no room on the track.
    squeezed: bool,
    /// No new pass before this time, after one was given up.
    cooldown_until_s: f64,
    /// Passes this car completed: when, and on which car.
    passes_made: Vec<(f64, usize)>,
    lap_times_s: Vec<f64>,
    finished: bool,
}

/// One sampled moment of the race, for rendering.
pub(crate) struct Moment {
    pub time_s: f64,
    pub cars: Vec<CarPose>,
}

#[derive(Clone, Copy)]
pub(crate) struct CarPose {
    pub transform: Transform3,
    pub steering_rad: f64,
    pub speed_m_s: f64,
    pub wheel_angle_rad: f64,
    pub braking: bool,
}

struct Report {
    finishing_order: Vec<&'static str>,
    /// Completed passes: time, passing car, passed car (grid slots).
    passes: Vec<(f64, usize, usize)>,
    lap_times: Vec<(&'static str, Vec<f64>)>,
    overtakes: usize,
    min_gap_m: f64,
    max_lateral_g: f64,
    max_off_line_m: f64,
    max_track_excursion_m: f64,
    saturated_share: f64,
    moments: Vec<Moment>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let track = Track::build();
    let line = RacingLine::minimum_curvature(&track, 1.4);
    let report = race(&track, &line, !smoke);
    print_report(&report, &track);
    check(&report);
    if smoke {
        println!("smoke ok");
        return;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let gif = root.join("docs/media/car-race.gif");
    render::render_race(
        &track,
        &line,
        &report.moments,
        &report.passes,
        &ENTRIES,
        &root.join("target/rne-car-race-frames"),
        &gif,
    )
    .expect("render the race");
    println!("wrote {}", gif.display());
}

fn race_drive(entry: &Entry) -> AckermannDrive {
    AckermannDrive {
        wheelbase_m: 3.4,
        max_speed_m_s: 80.0,
        max_steering_rad: 0.45,
        max_acceleration_m_s2: entry.accel_m_s2,
        max_deceleration_m_s2: 16.0,
        max_steering_rate_rad_s: 2.5,
        ..AckermannDrive::default()
    }
}

fn race_dynamics() -> VehicleDynamics {
    VehicleDynamics {
        mass_kg: 800.0,
        yaw_inertia_kg_m2: 1_000.0,
        // Set up with understeer, as race cars are: at the limit the front
        // lets go first and the car runs wide instead of spinning. The
        // understeer gradient m/L (b/Cf - a/Cr) is 1.3e-3 rad per m/s^2. The
        // first setup (axles 1.8/1.6 m, rear 130 kN/rad) was near neutral at
        // 1.6e-4, and a car rejoining its line after a pass spun.
        front_axle_m: 1.6,
        rear_axle_m: 1.8,
        center_of_mass_height_m: 0.3,
        front_cornering_stiffness_n_rad: 110_000.0,
        rear_cornering_stiffness_n_rad: 150_000.0,
        friction_coefficient: FRICTION,
        ..VehicleDynamics::default()
    }
}

/// The racing line moved `sign * PASSING_OFFSET_M` sideways, kept on the track.
fn passing_line(track: &Track, line: &RacingLine, sign: f64) -> RacingLine {
    let limit = 0.5 * WIDTH_M - CAR_HALF_WIDTH_M - 0.3;
    let offsets = line
        .offset
        .iter()
        .map(|offset| (offset + sign * PASSING_OFFSET_M).clamp(-limit, limit))
        .collect();
    RacingLine::from_offsets(track, offsets)
}

fn spawn_grid(world: &mut World, track: &Track, line: &RacingLine) -> Vec<Car> {
    let n = track.len();
    ENTRIES
        .iter()
        .enumerate()
        .map(|(slot, entry)| {
            // Staggered two-wide grid behind the line, pole on the left.
            let back = 10 + 9 * slot;
            let index = (n - back) % n;
            let side = if slot % 2 == 0 { 2.6 } else { -2.6 };
            let (x, z) = track.at(index, side);
            let (ax, az) = track.at(index + 5, side);
            let heading =
                Quat::from_rotation_arc(Vec3::X, Vec3::new(ax - x, 0.0, az - z).normalize());
            let entity = spawn_named(world, entry.name);
            world.entity_mut(entity).insert((
                race_drive(entry),
                race_dynamics(),
                Transform3::from_translation_rotation(Vec3::new(x, 0.0, z), heading),
                RigidBody::default(),
            ));
            let grip = grip_m_s2(FRICTION) * entry.grip_use;
            let profile = |racing: &RacingLine| {
                racing.speed_profile(grip, entry.accel_m_s2, 16.0 * entry.grip_use, 78.0)
            };
            // Off the racing line, with less grip in hand.
            let passing_profile = |racing: &RacingLine| {
                racing.speed_profile(0.9 * grip, entry.accel_m_s2, 14.0 * entry.grip_use, 78.0)
            };
            Car {
                entity,
                entry: *entry,
                speed_profile: profile(line),
                passing_profiles: [
                    passing_profile(&passing_line(track, line, 1.0)),
                    passing_profile(&passing_line(track, line, -1.0)),
                ],
                index,
                progress_m: -(back as f64),
                shift_m: side - line.offset[index],
                shift_target_m: 0.0,
                passing: None,
                squeezed: false,
                cooldown_until_s: 0.0,
                passes_made: Vec::new(),
                lap_times_s: Vec::new(),
                finished: false,
            }
        })
        .collect()
}

fn race(track: &Track, line: &RacingLine, record: bool) -> Report {
    let mut world = World::new();
    let mut cars = spawn_grid(&mut world, track, line);
    let dt_s = 1.0 / SIM_HZ;
    let dt = SimDuration::from_seconds(Seconds::new(dt_s));
    let lap = track.length_m;
    let n = track.len();
    let mut moments = Vec::new();
    let (mut min_gap_m, mut max_lateral_g, mut max_off_line_m, mut max_excursion_m) =
        (f64::INFINITY, 0.0_f64, 0.0_f64, 0.0_f64);
    let (mut saturated, mut samples) = (0_usize, 0_usize);
    let mut wheel_angle = vec![0.0; cars.len()];
    let mut step = 0_usize;
    while cars.iter().any(|car| !car.finished) {
        assert!(step < (240.0 * SIM_HZ) as usize, "the race never finished");
        let time_s = step as f64 * dt_s;
        let states: Vec<(Transform3, f64, f64)> = cars
            .iter()
            .map(|car| {
                let transform = *world.get::<Transform3>(car.entity).expect("transform");
                let speed = world
                    .get::<AckermannDrive>(car.entity)
                    .expect("drive")
                    .speed_m_s;
                (transform, speed, car.progress_m)
            })
            .collect();
        for (i, car) in cars.iter_mut().enumerate() {
            let (transform, _, _) = states[i];
            let position = transform.translation;
            let (index, lateral) = track.project(position.x, position.z, car.index, 40);
            let advanced = ((index + n - car.index) % n) as f64;
            let advanced = if advanced > n as f64 / 2.0 {
                advanced - n as f64
            } else {
                advanced
            };
            car.index = index;
            car.progress_m += advanced;
            max_excursion_m = max_excursion_m.max(lateral.abs() + CAR_HALF_WIDTH_M - 0.5 * WIDTH_M);
            max_off_line_m = max_off_line_m.max((lateral - line.offset[index] - car.shift_m).abs());
            if !car.finished && car.progress_m >= lap * (car.lap_times_s.len() + 1) as f64 {
                let previous: f64 = car.lap_times_s.iter().sum();
                car.lap_times_s.push(time_s - previous);
                car.finished = car.lap_times_s.len() >= LAPS;
            }
            plan_pass(car, i, &states, track, line, time_s);
            let (steering, target_speed, tow) = command(car, i, &states, track, line, index);
            let mut drive = world.get_mut::<AckermannDrive>(car.entity).expect("drive");
            drive.target_steering_rad =
                steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
            drive.target_speed_m_s = target_speed;
            drive.max_acceleration_m_s2 = car.entry.accel_m_s2 * (1.0 + 0.4 * tow);
        }
        vehicle_dynamics(&mut world, dt);

        // Measurements.
        for (i, car) in cars.iter().enumerate() {
            let dynamics = world.get::<VehicleDynamics>(car.entity).expect("dynamics");
            let speed = world
                .get::<AckermannDrive>(car.entity)
                .expect("drive")
                .speed_m_s;
            max_lateral_g = max_lateral_g.max((speed * dynamics.yaw_rate_rad_s).abs() / 9.81);
            if dynamics.front_saturated || dynamics.rear_saturated {
                saturated += 1;
            }
            samples += 1;
            wheel_angle[i] += speed * dt_s / 0.33;
            for other in cars.iter().skip(i + 1) {
                let a = world.get::<Transform3>(car.entity).expect("a").translation;
                let b = world
                    .get::<Transform3>(other.entity)
                    .expect("b")
                    .translation;
                min_gap_m = min_gap_m.min((a - b).length());
            }
        }
        if record && step.is_multiple_of(SIM_HZ as usize / 10) {
            moments.push(snapshot(&world, &cars, &wheel_angle, time_s));
        }
        step += 1;
    }
    let mut finishing: Vec<&Car> = cars.iter().collect();
    finishing.sort_by(|a, b| {
        let ta: f64 = a.lap_times_s.iter().sum();
        let tb: f64 = b.lap_times_s.iter().sum();
        ta.total_cmp(&tb)
    });
    // Net passes: pairs that finished in the opposite order to the grid.
    let finish_rank: Vec<usize> = cars
        .iter()
        .map(|car| {
            finishing
                .iter()
                .position(|other| other.entry.name == car.entry.name)
                .expect("finished")
        })
        .collect();
    let overtakes = (0..cars.len())
        .flat_map(|a| (a + 1..cars.len()).map(move |b| (a, b)))
        .filter(|(a, b)| finish_rank[*a] > finish_rank[*b])
        .count();
    let mut passes: Vec<(f64, usize, usize)> = cars
        .iter()
        .enumerate()
        .flat_map(|(slot, car)| {
            car.passes_made
                .iter()
                .map(move |(t, other)| (*t, slot, *other))
        })
        .collect();
    passes.sort_by(|a, b| a.0.total_cmp(&b.0));
    Report {
        passes,
        finishing_order: finishing.iter().map(|car| car.entry.name).collect(),
        lap_times: cars
            .iter()
            .map(|car| (car.entry.name, car.lap_times_s.clone()))
            .collect(),
        overtakes,
        min_gap_m,
        max_lateral_g,
        max_off_line_m,
        max_track_excursion_m: max_excursion_m,
        saturated_share: saturated as f64 / samples.max(1) as f64,
        moments,
    }
}

/// Steering and speed for one car this step, and how deep it is in a tow.
fn command(
    car: &Car,
    me: usize,
    states: &[(Transform3, f64, f64)],
    track: &Track,
    line: &RacingLine,
    index: usize,
) -> (f64, f64, f64) {
    let n = track.len();
    let (transform, speed, _) = states[me];
    let lookahead = (0.45 * speed).clamp(7.0, 30.0) as usize;
    let ahead = (index + lookahead) % n;
    // The passing offset rides on the racing line, which itself runs
    // to the edges; the sum is clamped to the track, or a car passing
    // on the outside of a corner aims past the kerb.
    let edge = 0.5 * WIDTH_M - CAR_HALF_WIDTH_M - 0.3;
    let target_offset = (line.offset[ahead] + car.shift_m).clamp(-edge, edge);
    let squeezed = car.squeezed;
    let (tx, tz) = track.at(ahead, target_offset);
    let steering = pure_pursuit_steering(
        &transform,
        Vec3::new(tx, 0.0, tz),
        race_drive(&car.entry).wheelbase_m,
        lookahead as f64,
    );
    // Blend toward the passing line's own profile as the car moves
    // across to it.
    let across = (car.shift_m.abs() / PASSING_OFFSET_M).min(1.0);
    let side = usize::from(car.shift_m < 0.0);
    let mut target_speed =
        car.speed_profile[index] * (1.0 - across) + car.passing_profiles[side][index] * across;
    // The tow: close behind another car in the same lane, drag drops,
    // so the car accelerates harder and, where the next 80 m does not
    // brake, can run a little past its own profile.
    let tow = slipstream(car, me, states, track);
    if flat_out_ahead(&car.speed_profile, index, 80) {
        target_speed += 4.0 * tow;
    }
    target_speed = target_speed.min(follow_speed(car, me, states, track, speed));
    if squeezed {
        // No room beside the car ahead: lift and drop in behind it.
        target_speed = target_speed.min((speed - 3.0).max(8.0));
    }
    target_speed = target_speed.max(0.0);
    if car.finished {
        target_speed = target_speed.min(25.0);
    }
    (steering, target_speed, tow)
}

/// Every car's pose at `time_s`, for rendering.
fn snapshot(world: &World, cars: &[Car], wheel_angle: &[f64], time_s: f64) -> Moment {
    Moment {
        time_s,
        cars: cars
            .iter()
            .zip(wheel_angle)
            .map(|(car, wheel_angle_rad)| {
                let drive = world.get::<AckermannDrive>(car.entity).expect("drive");
                CarPose {
                    transform: *world.get::<Transform3>(car.entity).expect("transform"),
                    steering_rad: drive.steering_rad,
                    speed_m_s: drive.speed_m_s,
                    wheel_angle_rad: *wheel_angle_rad,
                    braking: drive.target_speed_m_s < drive.speed_m_s - 0.5,
                }
            })
            .collect(),
    }
}

/// Arc distance from `from` forward to `to` around the lap, in meters.
fn gap_ahead(from_m: f64, to_m: f64) -> f64 {
    to_m - from_m
}

/// Starts, holds and ends a pass. A car that is closing on the car ahead by
/// more than 1 m/s, or is held up within 15 m of a car slower than its own
/// speed profile allows, moves onto a line beside it, on the side with more room
/// over the next stretch, and stays there until it is 8 m clear, or gives up
/// when the other car pulls away or six seconds bring no gain. The car being
/// passed keeps the racing line.
fn plan_pass(
    car: &mut Car,
    me: usize,
    states: &[(Transform3, f64, f64)],
    track: &Track,
    line: &RacingLine,
    time_s: f64,
) {
    let (_, my_speed, my_progress) = states[me];
    let n = track.len();
    let limit = 0.5 * WIDTH_M - CAR_HALF_WIDTH_M - 0.3;
    match car.passing {
        Some((other, began_s)) => {
            let ahead_of_me = states[other].2 - my_progress;
            // Done once 8 m clear; abandoned if the other car pulls away or
            // six seconds bring no gain.
            let done = ahead_of_me < -8.0;
            let lost = ahead_of_me > 30.0 || (time_s - began_s > 6.0 && ahead_of_me > 0.0);
            if lost {
                car.cooldown_until_s = time_s + 3.0;
            }
            if done {
                car.passes_made.push((time_s, other));
            }
            if done || lost {
                car.passing = None;
                car.shift_target_m = 0.0;
            }
        }
        None if time_s > START_SETTLE_S.max(car.cooldown_until_s) => {
            let closing = states
                .iter()
                .enumerate()
                .find(|(other, (_, speed, progress))| {
                    let gap = gap_ahead(my_progress, *progress);
                    // Closing, or held up: close behind a car slower than this
                    // car could be going here.
                    let closing = my_speed > speed + 1.0;
                    let held_up = gap < 15.0 && car.speed_profile[car.index] > speed + 1.5;
                    let towing = gap < 20.0;
                    *other != me
                        && (0.0..30.0).contains(&gap)
                        && (closing || held_up || towing)
                        && flat_out_ahead(&car.speed_profile, car.index, 120)
                });
            if let Some((other, (transform, _, _))) = closing {
                let (_, their_lateral) = track.project(
                    transform.translation.x,
                    transform.translation.z,
                    car.index,
                    60,
                );
                let room = |sign: f64| {
                    (0..80)
                        .map(|k| {
                            let i = (car.index + k) % n;
                            limit - (line.offset[i] + sign * PASSING_OFFSET_M).abs()
                        })
                        .fold(f64::INFINITY, f64::min)
                };
                // Away from where the other car is, unless that side has no room.
                let away = if their_lateral < line.offset[car.index] {
                    1.0
                } else {
                    -1.0
                };
                let sign = if room(away) >= room(-away) - 1.0 {
                    away
                } else {
                    -away
                };
                car.passing = Some((other, time_s));
                car.shift_target_m = sign * PASSING_OFFSET_M;
            }
        }
        None => {}
    }
    // Back onto the racing line before braking for the next corner: passes
    // are made on the straights.
    // Back onto the racing line before braking for the next corner: passes
    // are made on the straights, unless the pass is already alongside, in
    // which case it keeps its line through the corner.
    let alongside = car
        .passing
        .is_some_and(|(other, _)| states[other].2 - my_progress < 1.0);
    if car.shift_target_m != 0.0 && !alongside && !flat_out_ahead(&car.speed_profile, car.index, 70)
    {
        car.passing = None;
        car.shift_target_m = 0.0;
        car.cooldown_until_s = time_s + 3.0;
    }
    // The car behind is responsible: leave a car's width to any car alongside
    // and ahead, in shift space so the move is rate limited like any other.
    // The car ahead keeps its line.
    let base = line.offset[car.index % n];
    let mut goal = car.shift_target_m;
    car.squeezed = false;
    for (other, (transform, _, progress)) in states.iter().enumerate() {
        let ahead = progress - my_progress;
        if other == me || !(0.0..=2.0 * CAR_HALF_LENGTH_M + 1.0).contains(&ahead) {
            continue;
        }
        let (_, lateral) = track.project(
            transform.translation.x,
            transform.translation.z,
            car.index,
            60,
        );
        let theirs = lateral - base;
        let side = if car.shift_m >= theirs { 1.0 } else { -1.0 };
        let keep = theirs + side * SIDE_BY_SIDE_M;
        if (side > 0.0 && goal < keep) || (side < 0.0 && goal > keep) {
            goal = keep;
        }
    }
    if goal > limit - base || goal < -limit - base {
        // No room: give the pass up and fall in behind.
        car.squeezed = true;
        if car.passing.is_some() {
            car.cooldown_until_s = time_s + 3.0;
        }
        car.passing = None;
        car.shift_target_m = 0.0;
        goal = goal.clamp(-limit - base, limit - base);
    }
    // Move across at no more than 1.5 m per second.
    let rate = 1.5 / SIM_HZ;
    car.shift_m += (goal - car.shift_m).clamp(-rate, rate);
    car.shift_m = car.shift_m.clamp(-limit - base, limit - base);
}

/// Whether the profile holds up (never drops more than 0.5 m/s below its
/// current value) over the next `meters`: no braking zone ahead.
fn flat_out_ahead(profile: &[f64], index: usize, meters: usize) -> bool {
    let n = profile.len();
    (0..meters).all(|k| profile[(index + k) % n] >= profile[index] - 0.5)
}

/// A car's width and a margin between two cars side by side.
const SIDE_BY_SIDE_M: f64 = 2.0 * CAR_HALF_WIDTH_M + 0.8;

/// How deep in another car's tow this car is: 1 right behind it, falling to 0
/// at 30 m, and 0 unless the two are in the same lane.
fn slipstream(car: &Car, me: usize, states: &[(Transform3, f64, f64)], track: &Track) -> f64 {
    let (my_transform, _, my_progress) = states[me];
    let (_, my_lateral) = track.project(
        my_transform.translation.x,
        my_transform.translation.z,
        car.index,
        40,
    );
    states
        .iter()
        .enumerate()
        .filter(|(other, _)| *other != me)
        .filter_map(|(_, (transform, _, progress))| {
            let gap = gap_ahead(my_progress, *progress);
            if !(0.0..30.0).contains(&gap) {
                return None;
            }
            let (_, lateral) = track.project(
                transform.translation.x,
                transform.translation.z,
                car.index,
                60,
            );
            ((lateral - my_lateral).abs() < 1.5).then_some(1.0 - gap / 30.0)
        })
        .fold(0.0, f64::max)
}

/// The speed that keeps a car from running into one directly ahead in its
/// lane: no faster than that car once within two seconds of it.
fn follow_speed(
    car: &Car,
    me: usize,
    states: &[(Transform3, f64, f64)],
    track: &Track,
    my_speed: f64,
) -> f64 {
    let (my_transform, _, my_progress) = states[me];
    let (_, my_lateral) = track.project(
        my_transform.translation.x,
        my_transform.translation.z,
        car.index,
        40,
    );
    let mut limit = f64::INFINITY;
    for (other, (transform, speed, progress)) in states.iter().enumerate() {
        if other == me {
            continue;
        }
        let gap = gap_ahead(my_progress, *progress) - 2.0 * CAR_HALF_LENGTH_M;
        if !(-2.0 * CAR_HALF_LENGTH_M..2.0 * my_speed.max(10.0)).contains(&gap) {
            continue;
        }
        let (_, lateral) = track.project(
            transform.translation.x,
            transform.translation.z,
            car.index,
            60,
        );
        if (lateral - my_lateral).abs() < 2.0 * CAR_HALF_WIDTH_M + 0.6 {
            // Behind: close the gap no faster than half a meter per second
            // per meter. Overlapping: drop back, rather than sit alongside a
            // car that has nowhere to go.
            limit = if gap < 0.0 {
                limit.min(speed - 2.0)
            } else {
                limit.min(speed + 0.5 * gap)
            };
        }
    }
    limit
}

fn print_report(report: &Report, track: &Track) {
    println!("lap length {:.0} m", track.length_m);
    for (name, laps) in &report.lap_times {
        println!(
            "{name:>7}: {}",
            laps.iter()
                .map(|t| format!("{t:.2} s"))
                .collect::<Vec<_>>()
                .join("  ")
        );
    }
    println!("finish: {}", report.finishing_order.join(" > "));
    for (time_s, passer, passed) in &report.passes {
        println!(
            "  {time_s:6.1} s  {} passes {}",
            ENTRIES[*passer].name, ENTRIES[*passed].name
        );
    }
    println!(
        "overtakes {}, closest cars {:.2} m apart, peak lateral {:.2} g, tires saturated {:.1} % of car-steps",
        report.overtakes,
        report.min_gap_m,
        report.max_lateral_g,
        100.0 * report.saturated_share
    );
    println!(
        "worst line error {:.2} m, worst excursion past the track edge {:.2} m",
        report.max_off_line_m, report.max_track_excursion_m
    );
}

fn check(report: &Report) {
    // Side by side, 2.47 m apart centre to centre on a 2 m wide car: the
    // closest two cars came was half a meter of air.
    assert!(
        report.min_gap_m > 2.0 * CAR_HALF_WIDTH_M + 0.2,
        "two cars came within {:.2} m",
        report.min_gap_m
    );
    assert!(
        report.max_track_excursion_m < 0.1,
        "a car ran {:.2} m past the edge",
        report.max_track_excursion_m
    );
    // The grid is in reverse order of pace; the fastest car has to pass the
    // other three to win, and did.
    assert_eq!(
        report.finishing_order[0], "red",
        "the fastest car did not win"
    );
    assert!(report.overtakes >= 3, "only {} passes", report.overtakes);
}
