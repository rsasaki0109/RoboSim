//! Searches for a G1 torque overlay that walks the robot *forwards*.
//!
//! `UnitreeG1TorqueOverlay::LEARNED_STRIDE`, found by example 62, walks the G1
//! backwards. Measured: over 800 ticks the knees bulge 0.028 m toward the
//! pelvis's local +x -- the way the robot faces -- while the body travels
//! 0.345 m toward -x. The search that found it scored each window as
//! `hypot(dx, dz)`, a distance with no direction, and named it
//! `window_a_forward_m`, so walking backwards scored exactly as well as
//! walking forwards and the optimiser took the one it found first.
//!
//! This is the same CEM search -- same 48-coefficient contact-gated overlay,
//! same anti-cheat minimum window, same fall, tilt and straightness
//! penalties, same median of three ulp-perturbed replays -- with one change to
//! what it rewards: displacement *along the way the robot faces*, signed.
//!
//! The scripted stepper under the overlay also moves its stance foot forward
//! beneath the pelvis, which is the stepping pattern of a backward walk.
//! `--forward-gait` reflects the hip-pitch stride term (and the ankle term that
//! follows it) so the swing leg reaches forward instead. The library gait is
//! left alone: every pinned G1 locomotion result depends on it bit for bit.
//!
//! Rewarding displacement along the facing was not enough on its own. Each
//! search found the next hole by walking through it: the first went forwards
//! but spun 3 rad in 24 s, because the straightness penalty read
//! `base_relative_yaw_rad`, whose "yaw" is about a horizontal axis on this
//! z-up robot; the second went straight but travelled 0.86 rad off its
//! heading, because nothing penalised sideways travel. Heading is now measured
//! from the pelvis's facing and sideways travel costs `SIDEWAYS_WEIGHT` per
//! meter. Each search warm-starts from the previous one's result, pinned below
//! as a seed, so any of them can be re-run.
//!
//! ```text
//! # reproduce the pinned FORWARD_STRIDE (fourth search)
//! cargo run --release -p g1_forward_stride --example 124_g1_forward_stride -- --train --warm-start-less-crabbing
//! # verify it
//! cargo run --release -p g1_forward_stride --example 124_g1_forward_stride
//! ```

use std::fs;
use std::path::{Path, PathBuf};

use rne_ai::{
    unitree_g1_dynamic_scene_path, unitree_g1_gait_targets, UnitreeG1GaitCommand,
    UnitreeG1TorqueOverlay, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::Vec3;

const ROLLOUT_STEPS: u64 = 1440;
const WINDOW_START_STEP: u64 = 480;
const WINDOW_SPLIT_STEP: u64 = 960;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 300.0;
const KD: f64 = 10.0;
const TORQUE_LIMIT_NM: f64 = 88.0;
const SPEED_LIMIT_RAD_S: f64 = 30.0;
const WALK_STRIDE_RAD: f64 = 0.065;
const WALK_FOOT_LIFT_RAD: f64 = 0.12;
const WALK_CYCLE_STEPS: u64 = 100;
/// Required median forward progress per 8 s window, in meters.
///
/// Set from what the walk is for, not from what the search reached: the
/// factory route's legs are about 0.5 m, and 0.10 m per 8 s window crosses one
/// in 40 s. The pinned overlay makes 0.14-0.16.
const FORWARD_MIN_WINDOW_M: f64 = 0.10;
const DIM: usize = 48;
const POPULATION: usize = 64;
const ELITE: usize = 16;
const ITERATIONS: usize = 30;

/// Result of the first search (signed forward score, heading penalty still
/// read from the broken `base_relative_yaw_rad`): 0.25/0.25 m per window along
/// the facing, but spinning about 3 rad in 24 s. Seed for the second.
const SPINNING_SEED: [[f64; 6]; 8] = [
    [
        -0.282483298543,
        -0.138803127254,
        9.559762479574,
        -0.611454617571,
        -6.384224651209,
        -5.339325506617,
    ],
    [
        0.164267736604,
        2.306333777881,
        -9.136442336041,
        12.361063890635,
        0.660317592696,
        -0.601797655937,
    ],
    [
        9.566075873155,
        2.696004602724,
        -6.633537875681,
        -6.175352700414,
        0.769534508975,
        -1.473102033854,
    ],
    [
        9.736447925172,
        1.325536175554,
        -2.755775701590,
        -19.643302978489,
        -11.515167274416,
        -4.971192667878,
    ],
    [
        -2.694418041493,
        0.734362553691,
        -9.979894078838,
        -5.028506900787,
        -0.697245023420,
        -4.526602370015,
    ],
    [
        0.881720650781,
        2.391067103931,
        -4.829789676319,
        20.000000000000,
        -0.094510124490,
        6.585424084838,
    ],
    [
        -4.304775500135,
        -4.954366060668,
        -6.557005609108,
        -5.281440044147,
        -2.899341554896,
        0.213841807057,
    ],
    [
        1.315039041044,
        9.308092146447,
        0.428761849514,
        6.063331234848,
        -6.307624854466,
        -2.736818288185,
    ],
];

/// Result of the second search (heading penalty fixed, warm-started from the
/// first forward overlay): straight, not spinning, 0.13/0.14 m per window
/// along the facing -- but travelling 0.86 rad off it. Seed for the third.
const STRAIGHT_CRABBING_SEED: [[f64; 6]; 8] = [
    [
        -1.012241809375,
        0.796869402204,
        8.679138921689,
        1.072395616439,
        -5.133332404561,
        -7.758830199553,
    ],
    [
        -3.470700261057,
        1.836146957869,
        -10.514986415576,
        14.811683892068,
        0.706085371996,
        -3.133883980814,
    ],
    [
        4.753798722790,
        3.388514288237,
        -6.676103164336,
        -7.058671958342,
        2.371135213291,
        -0.561307245913,
    ],
    [
        12.510272561033,
        3.165044426247,
        -3.229483169591,
        -20.000000000000,
        -7.522887681845,
        -4.272908436453,
    ],
    [
        -2.156193658713,
        0.885269946308,
        -10.084747468315,
        -7.483792130630,
        -1.736650447504,
        -6.164877540389,
    ],
    [
        -1.016897841101,
        0.132707239738,
        -4.435508650369,
        14.911212067474,
        -1.549213701235,
        2.968374212542,
    ],
    [
        -5.836861524254,
        -4.018371218867,
        -7.756189379583,
        -5.118502386467,
        -3.331529793870,
        -0.785201646623,
    ],
    [
        1.886791477177,
        7.839329864889,
        0.806792995485,
        4.253605163907,
        -6.028850711458,
        -3.695553129224,
    ],
];

/// Weight on sideways travel in the score, per meter.
///
/// The third search ran at 1.0 and cut the crab from 0.86 rad to about 0.35
/// rad off the heading; the fourth runs at 3.0 from that result.
const SIDEWAYS_WEIGHT: f64 = 3.0;

/// Result of the third search (sideways penalty 1.0): 0.16/0.16 m per window,
/// no spin, travelling about 0.35 rad off the facing. Seed for the fourth.
const LESS_CRABBING_SEED: [[f64; 6]; 8] = [
    [
        -1.882880525299,
        1.992197366386,
        8.293742075089,
        2.770450430598,
        -4.117914361783,
        -8.721742023768,
    ],
    [
        -4.671106021978,
        2.856314674711,
        -11.538849964258,
        17.514363231937,
        -0.071142475716,
        -2.394734273058,
    ],
    [
        5.206269762353,
        1.812959324383,
        -6.031081056836,
        -9.421788640098,
        4.036912358833,
        -1.018602071227,
    ],
    [
        13.003866001307,
        2.941171139375,
        -4.183702883493,
        -15.328477759678,
        -5.071961799571,
        -3.352985023258,
    ],
    [
        -1.783835457427,
        0.461483002331,
        -13.245585751314,
        -6.089573907573,
        -0.797149069245,
        -10.194464319175,
    ],
    [
        -1.069373222966,
        -0.863871124671,
        -3.324954611194,
        16.235025433476,
        -7.481594485950,
        1.431354617756,
    ],
    [
        -5.070846738543,
        -5.295777697678,
        -8.020882111606,
        -4.117275953209,
        -3.253165251080,
        -1.233871255531,
    ],
    [
        1.686636247860,
        8.178205642463,
        -1.448559248091,
        3.878324268419,
        -7.942832002936,
        -6.499345504450,
    ],
];

const TORQUE_LINKS: [&str; 8] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
];

/// Whether `--forward-gait` was passed.
fn forward_gait() -> bool {
    std::env::args().any(|argument| argument == "--forward-gait")
}

/// The stepper's targets, reflected so the swing leg reaches forward when
/// `forward` is set.
///
/// The library gait sets `hip_pitch = -0.18 + stride * wave` and ankle pitch to
/// `-0.18 - 0.45 * stride * wave - ...`. On the G1 a more negative hip pitch
/// puts the thigh forward, so as `wave` falls from +1 to -1 through stance the
/// planted foot travels forward under the pelvis -- a backward step.
/// Reflecting the stride term about -0.18 reverses that.
fn gait_targets(
    step: u64,
    command: UnitreeG1GaitCommand,
    forward: bool,
) -> Vec<UrdfJointPositionTarget<'static>> {
    let mut targets = unitree_g1_gait_targets(step, command).to_vec();
    if forward {
        for (hip, ankle) in [
            ("left_hip_pitch_link", "left_ankle_pitch_link"),
            ("right_hip_pitch_link", "right_ankle_pitch_link"),
        ] {
            let stride_term = targets
                .iter()
                .find(|target| target.link_name == hip)
                .expect("hip pitch target")
                .position
                + 0.18;
            for target in &mut targets {
                if target.link_name == hip {
                    target.position = -0.18 - stride_term;
                } else if target.link_name == ankle {
                    target.position += 0.9 * stride_term;
                }
            }
        }
    }
    targets
}

fn walk_command() -> UnitreeG1GaitCommand {
    UnitreeG1GaitCommand {
        stride_rad: WALK_STRIDE_RAD,
        foot_lift_rad: WALK_FOOT_LIFT_RAD,
        cycle_steps: WALK_CYCLE_STEPS,
    }
}

#[derive(Clone, Copy, Debug)]
struct RolloutOutcome {
    window_a_forward_m: f64,
    window_b_forward_m: f64,
    total_forward_m: f64,
    total_yaw_rad: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
    score: f64,
}

fn settle(sim: &mut UrdfSceneSim, cycle_steps: u64) {
    sim.configure_position_motors(220.0, 24.0, TORQUE_LIMIT_NM);
    let stand = unitree_g1_gait_targets(
        0,
        UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps,
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
}

fn rollout(
    command: UnitreeG1GaitCommand,
    overlay: &UnitreeG1TorqueOverlay,
    steps: u64,
) -> RolloutOutcome {
    let forward = forward_gait();
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("load dynamic G1");
    settle(&mut sim, command.cycle_steps);
    let up_body_reference = {
        let pose = sim.named_transform("pelvis").expect("pelvis pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let true_tilt = |sim: &UrdfSceneSim| {
        let pose = sim.named_transform("pelvis").expect("pelvis pose");
        let up = (pose.rotation * up_body_reference).normalize_or_zero();
        up.y.clamp(-1.0, 1.0).acos()
    };
    // The way the robot faces, in the ground plane. The pelvis frame is z-up
    // (the URDF's), and the knees bend toward its local +x, so +x is the front.
    let facing = |sim: &UrdfSceneSim| {
        let forward = sim.named_transform("pelvis").expect("pelvis pose").rotation * Vec3::X;
        let flat = Vec3::new(forward.x, 0.0, forward.z);
        let length = flat.length().max(1.0e-9);
        [flat.x / length, flat.z / length]
    };
    let start = sim.observe();
    let mut window_start_position = [start.base_x_m, start.base_z_m];
    let mut window_facing = facing(&sim);
    let mut sideways_m = 0.0_f64;
    let mut window_a_forward_m = 0.0;
    let mut window_b_forward_m = 0.0;
    // Heading about the world vertical, from the facing direction. Not
    // `base_relative_yaw_rad`: that is `y_up_euler_rad(q_ref^-1 * q)`, the
    // relative rotation in the *robot's* frame, and the G1's frame is z-up, so
    // its "yaw" is a rotation about a horizontal axis. Measured on the first
    // forward overlay this search found, the body turned 3.0 rad in 24 s while
    // that field reported +0.05 rad -- so the straightness penalty never fired
    // and the search was free to find a walk that spins.
    let heading = |sim: &UrdfSceneSim| {
        let [x, z] = facing(sim);
        x.atan2(z)
    };
    let mut previous_yaw = heading(&sim);
    let mut total_yaw_rad = 0.0;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad = 0.0_f64;
    let cycle = command.cycle_steps;
    for step in 0..steps {
        let targets = gait_targets(step, command, forward);
        let servo: Vec<UrdfJointPositionTarget<'_>> = targets
            .iter()
            .filter(|target| !TORQUE_LINKS.contains(&target.link_name))
            .copied()
            .collect();
        sim.set_joint_position_targets(&servo);
        let stance = [
            sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0,
            sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0,
        ];
        let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
        let feed_forward = overlay.torques_nm(two_cycle_phase, stance);
        let torques: Vec<UrdfJointTorqueTarget<'_>> = TORQUE_LINKS
            .iter()
            .enumerate()
            .map(|(index, link_name)| {
                let target_position = targets
                    .iter()
                    .find(|target| target.link_name == *link_name)
                    .expect("torque link in gait targets")
                    .position;
                let q = sim.named_joint_position(link_name).expect("joint position");
                let qd = sim.named_joint_velocity(link_name).expect("joint velocity");
                UrdfJointTorqueTarget {
                    link_name,
                    torque_nm: (KP * (target_position - q) - KD * qd + feed_forward[index])
                        .clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                    max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                }
            })
            .collect();
        sim.step_joint_torques(&torques);
        let observed = sim.observe();
        if !observed.base_y_m.is_finite() {
            // A solver blow-up is an automatic worst score.
            return RolloutOutcome {
                window_a_forward_m: 0.0,
                window_b_forward_m: 0.0,
                total_forward_m: 0.0,
                total_yaw_rad: 0.0,
                min_height_m: -1.0,
                max_tilt_rad: 10.0,
                score: -100.0,
            };
        }
        let current_yaw = heading(&sim);
        let mut yaw_delta = current_yaw - previous_yaw;
        while yaw_delta > std::f64::consts::PI {
            yaw_delta -= 2.0 * std::f64::consts::PI;
        }
        while yaw_delta < -std::f64::consts::PI {
            yaw_delta += 2.0 * std::f64::consts::PI;
        }
        total_yaw_rad += yaw_delta;
        previous_yaw = current_yaw;
        // Signed: progress along the way the robot faced when the window
        // opened. A backward walk scores negative, which is the whole point.
        let along = |from: [f64; 2], toward: [f64; 2]| {
            (observed.base_x_m - from[0]) * toward[0] + (observed.base_z_m - from[1]) * toward[1]
        };
        if step + 1 == WINDOW_START_STEP {
            window_start_position = [observed.base_x_m, observed.base_z_m];
            window_facing = facing(&sim);
        } else if step + 1 == WINDOW_SPLIT_STEP {
            window_a_forward_m = along(window_start_position, window_facing);
            sideways_m += along(window_start_position, [window_facing[1], -window_facing[0]]).abs();
            window_start_position = [observed.base_x_m, observed.base_z_m];
            window_facing = facing(&sim);
        } else if step + 1 == steps {
            window_b_forward_m = along(window_start_position, window_facing);
            sideways_m += along(window_start_position, [window_facing[1], -window_facing[0]]).abs();
        }
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(true_tilt(&sim));
    }
    let end = sim.observe();
    let total_forward_m = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
    let _ = window_facing;
    // Anti-cheat transport score: minimum straight-line window displacement,
    // with fall/crouch penalties tuned to the humanoid's 0.80 m stance and a
    // straightness penalty.
    // Sideways travel is penalised as well as turning. Scoring only the
    // component along the facing let a walk that crabs 49 degrees off its
    // heading keep 65% of its credit: dead straight, no spin, and diagonal.
    let score = 2.0 * window_a_forward_m.min(window_b_forward_m)
        - SIDEWAYS_WEIGHT * sideways_m
        - if max_tilt_rad > 0.5 { 5.0 } else { 0.0 }
        - 20.0 * (0.60 - min_height_m).max(0.0)
        - 0.5 * total_yaw_rad.abs();
    RolloutOutcome {
        window_a_forward_m,
        window_b_forward_m,
        total_forward_m,
        total_yaw_rad,
        min_height_m,
        max_tilt_rad,
        score,
    }
}

fn overlay_from(params: &[f64; DIM]) -> UnitreeG1TorqueOverlay {
    let mut coefficients = [[0.0; 6]; 8];
    for joint in 0..8 {
        coefficients[joint][0] = params[joint * 6].clamp(-30.0, 30.0);
        for term in 1..6 {
            coefficients[joint][term] = params[joint * 6 + term].clamp(-20.0, 20.0);
        }
    }
    UnitreeG1TorqueOverlay { coefficients }
}

/// Verification ensemble: three ulp-perturbed replays — the chaos-floor
/// discipline from the Go2 campaign.
fn ensemble_outcomes(
    command: UnitreeG1GaitCommand,
    overlay: UnitreeG1TorqueOverlay,
) -> Vec<Option<RolloutOutcome>> {
    [0.0_f64, 1.0e-9, 3.0e-9]
        .iter()
        .map(|perturbation| {
            let mut member = overlay;
            member.coefficients[0][0] += *perturbation;
            // Wild candidates can blow the solver up inside a step (the
            // humanoid explosion mode); a panicking rollout deterministically
            // becomes a fall instead of killing the search.
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                rollout(command, &member, ROLLOUT_STEPS)
            }))
            .ok()
        })
        .collect()
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(|a, b| a.partial_cmp(b).expect("finite ensemble metric"));
    values[values.len() / 2]
}

fn ensemble_score_for(command: UnitreeG1GaitCommand, overlay: UnitreeG1TorqueOverlay) -> f64 {
    // Keep the original resumable CEM stream (0/1/2e-9) stable. The
    // verification and command-sweep ensemble above uses the pinned
    // cross-platform bar (0/1/3e-9).
    let mut scores: Vec<f64> = (0..3)
        .map(|k| {
            let mut member = overlay;
            member.coefficients[0][0] += k as f64 * 1.0e-9;
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                rollout(command, &member, ROLLOUT_STEPS).score
            }))
            .unwrap_or(-100.0)
        })
        .collect();
    scores.sort_by(|a, b| a.partial_cmp(b).expect("finite scores"));
    scores[1]
}

fn ensemble_score(params: &[f64; DIM]) -> f64 {
    ensemble_score_for(walk_command(), overlay_from(params))
}

fn gaussian(rng: &mut rne_ai::DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

const ITERATIONS_PER_RUN: usize = ITERATIONS;
const PARALLEL_ROLLOUTS: usize = 8;

type TrainState = (usize, [f64; DIM], [f64; DIM], (f64, [f64; DIM]));

fn state_path() -> PathBuf {
    // One resumable CEM state per stepper, so the two searches never mix.
    let name = if forward_gait() {
        "g1_forward_stride_v2_cem_state_forward_gait.txt"
    } else {
        "g1_forward_stride_v4_cem_state.txt"
    };
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../target")
        .join(name)
}

fn load_state(path: &Path) -> Option<TrainState> {
    let text = fs::read_to_string(path).ok()?;
    let values: Vec<f64> = text
        .split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    if values.len() != 2 + 3 * DIM {
        return None;
    }
    let mut mean = [0.0; DIM];
    let mut sigma = [0.0; DIM];
    let mut best_params = [0.0; DIM];
    mean.copy_from_slice(&values[1..1 + DIM]);
    sigma.copy_from_slice(&values[1 + DIM..1 + 2 * DIM]);
    best_params.copy_from_slice(&values[2 + 2 * DIM..2 + 3 * DIM]);
    Some((
        values[0] as usize,
        mean,
        sigma,
        (values[1 + 2 * DIM], best_params),
    ))
}

fn save_state(path: &Path, state: &TrainState) {
    let mut text = format!("{}\n", state.0);
    for value in state.1.iter().chain(state.2.iter()) {
        text.push_str(&format!("{value:.12}\n"));
    }
    text.push_str(&format!("{:.12}\n", state.3 .0));
    for value in state.3 .1.iter() {
        text.push_str(&format!("{value:.12}\n"));
    }
    fs::write(path, text).expect("write CEM state");
}

/// Where a fresh search starts: from zero, or from a pinned earlier result.
fn initial_state() -> TrainState {
    if std::env::args().any(|argument| argument == "--warm-start-less-crabbing") {
        let mut mean = [0.0; DIM];
        for (joint, row) in LESS_CRABBING_SEED.iter().enumerate() {
            for (term, value) in row.iter().enumerate() {
                mean[joint * 6 + term] = *value;
            }
        }
        (0, mean, [1.0; DIM], (f64::MIN, mean))
    } else if std::env::args().any(|argument| argument == "--warm-start-straight") {
        // The second search's result: straight and not spinning, but
        // travelling 0.86 rad off the way it faces.
        let mut mean = [0.0; DIM];
        for (joint, row) in STRAIGHT_CRABBING_SEED.iter().enumerate() {
            for (term, value) in row.iter().enumerate() {
                mean[joint * 6 + term] = *value;
            }
        }
        (0, mean, [1.5; DIM], (f64::MIN, mean))
    } else if std::env::args().any(|argument| argument == "--warm-start") {
        // Start from the first forward overlay, which covers ground
        // the right way but spins about 3 rad in 24 s. From zero, with
        // the heading penalty measured properly, the search stalled at
        // +0.03 m per window by iteration 14: standing nearly still is
        // the cheapest way to avoid turning. Starting from a walk that
        // already goes forwards leaves it only the spin to remove.
        let mut mean = [0.0; DIM];
        for (joint, row) in SPINNING_SEED.iter().enumerate() {
            for (term, value) in row.iter().enumerate() {
                mean[joint * 6 + term] = *value;
            }
        }
        (0, mean, [2.0; DIM], (f64::MIN, mean))
    } else {
        (0, [0.0; DIM], [5.0; DIM], (f64::MIN, [0.0; DIM]))
    }
}

fn train() {
    // Solver panics from wild candidates are expected and scored, not fatal;
    // silence the default hook so the log stays readable.
    std::panic::set_hook(Box::new(|_| {}));
    let path = state_path();
    let (start_iteration, mut mean, mut sigma, mut best) =
        load_state(&path).unwrap_or_else(initial_state);
    let end_iteration = (start_iteration + ITERATIONS_PER_RUN).min(ITERATIONS);
    for iteration in start_iteration..end_iteration {
        // Sequential sampling from a per-iteration seed keeps the search
        // deterministic and resumable; only the physics rollouts parallelize.
        let mut rng = rne_ai::DeterministicRng::new(42 + iteration as u64);
        let population: Vec<[f64; DIM]> = (0..POPULATION)
            .map(|_| {
                let mut params = [0.0_f64; DIM];
                for (value, (m, s)) in params.iter_mut().zip(mean.iter().zip(sigma.iter())) {
                    *value = m + s * gaussian(&mut rng);
                }
                params
            })
            .collect();
        let mut scored: Vec<(f64, [f64; DIM])> = Vec::with_capacity(POPULATION);
        for chunk in population.chunks(PARALLEL_ROLLOUTS) {
            let scores = std::thread::scope(|scope| {
                let handles: Vec<_> = chunk
                    .iter()
                    .map(|params| scope.spawn(move || ensemble_score(params)))
                    .collect();
                handles
                    .into_iter()
                    .map(|handle| handle.join().expect("rollout thread"))
                    .collect::<Vec<_>>()
            });
            for (score, params) in scores.into_iter().zip(chunk.iter()) {
                scored.push((score, *params));
            }
        }
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).expect("finite scores"));
        if scored[0].0 > best.0 {
            best = scored[0];
        }
        for dimension in 0..DIM {
            let elite_mean = scored[..ELITE]
                .iter()
                .map(|(_, params)| params[dimension])
                .sum::<f64>()
                / ELITE as f64;
            let elite_variance = scored[..ELITE]
                .iter()
                .map(|(_, params)| (params[dimension] - elite_mean).powi(2))
                .sum::<f64>()
                / ELITE as f64;
            mean[dimension] = elite_mean;
            sigma[dimension] = elite_variance.sqrt().max(0.5);
        }
        // The per-iteration probe is a single replay, and a single replay of a
        // chaotic candidate can blow the solver up where the median of three
        // did not. Unprotected, that panic took the whole search down at
        // iteration 28 of the fourth run -- and because the search is
        // deterministic, resuming would have crashed at the same place.
        let probe = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            rollout(walk_command(), &overlay_from(&scored[0].1), ROLLOUT_STEPS)
        }))
        .unwrap_or(RolloutOutcome {
            window_a_forward_m: f64::NAN,
            window_b_forward_m: f64::NAN,
            total_forward_m: f64::NAN,
            total_yaw_rad: f64::NAN,
            min_height_m: f64::NAN,
            max_tilt_rad: f64::NAN,
            score: f64::NAN,
        });
        println!(
            "iter {iteration:2}: best median score {:.3} (windows {:.2}/{:.2} m total {:.2} m yaw {:+.2} minH {:.3} maxTilt {:.2})",
            scored[0].0,
            probe.window_a_forward_m,
            probe.window_b_forward_m,
            probe.total_forward_m,
            probe.total_yaw_rad,
            probe.min_height_m,
            probe.max_tilt_rad
        );
        save_state(&path, &(iteration + 1, mean, sigma, best));
    }
    if end_iteration < ITERATIONS {
        println!("checkpointed at iteration {end_iteration}/{ITERATIONS}; run again to continue");
        return;
    }
    for k in 0..3 {
        let mut overlay = overlay_from(&best.1);
        overlay.coefficients[0][0] += k as f64 * 1.0e-9;
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            rollout(walk_command(), &overlay, ROLLOUT_STEPS)
        }))
        .unwrap_or(RolloutOutcome {
            window_a_forward_m: f64::NAN,
            window_b_forward_m: f64::NAN,
            total_forward_m: f64::NAN,
            total_yaw_rad: f64::NAN,
            min_height_m: f64::NAN,
            max_tilt_rad: f64::NAN,
            score: f64::NAN,
        });
        println!(
            "ensemble member {k}: windows {:.2}/{:.2} m total {:.2} m minH {:.3} tilt {:.2}",
            outcome.window_a_forward_m,
            outcome.window_b_forward_m,
            outcome.total_forward_m,
            outcome.min_height_m,
            outcome.max_tilt_rad
        );
    }
    let final_outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        rollout(walk_command(), &overlay_from(&best.1), ROLLOUT_STEPS)
    }))
    .unwrap_or(RolloutOutcome {
        window_a_forward_m: f64::NAN,
        window_b_forward_m: f64::NAN,
        total_forward_m: f64::NAN,
        total_yaw_rad: f64::NAN,
        min_height_m: f64::NAN,
        max_tilt_rad: f64::NAN,
        score: f64::NAN,
    });
    println!(
        "final best: median score {:.3} total {:.2} m over 24 s ({:.3} m/s)",
        best.0,
        final_outcome.total_forward_m,
        final_outcome.total_forward_m / 24.0
    );
    // Full 12-decimal precision: contact-gated rollouts diverge under
    // 6-decimal rounding, so the pinned constant must reproduce these digits.
    let overlay = overlay_from(&best.1);
    println!("coefficients: [");
    for row in overlay.coefficients {
        let cells: Vec<String> = row.iter().map(|value| format!("{value:.12}")).collect();
        println!("    [{}],", cells.join(", "));
    }
    println!("],");
}

fn main() {
    if std::env::args().any(|argument| argument == "--train") {
        train();
        return;
    }

    // Default: replay the pinned forward walk against the backward one, each
    // as the median of three ulp-perturbed replays, and require that it goes
    // the way the robot faces.
    let median_windows = |overlay: UnitreeG1TorqueOverlay| {
        let members = ensemble_outcomes(walk_command(), overlay);
        for (index, member) in members.iter().enumerate() {
            match member {
                Some(outcome) => println!(
                    "  member {index}: windows {:+.2}/{:+.2} m minH {:.3} tilt {:.2}",
                    outcome.window_a_forward_m,
                    outcome.window_b_forward_m,
                    outcome.min_height_m,
                    outcome.max_tilt_rad
                ),
                None => println!("  member {index}: SOLVER PANIC (scored as a fall)"),
            }
        }
        let mut windows: Vec<f64> = members
            .iter()
            .map(|member| {
                member
                    .as_ref()
                    .map_or(f64::MIN, |o| o.window_a_forward_m.min(o.window_b_forward_m))
            })
            .collect();
        let mut heights: Vec<f64> = members
            .iter()
            .map(|member| member.as_ref().map_or(-1.0, |o| o.min_height_m))
            .collect();
        (
            median(std::mem::take(&mut windows)),
            median(std::mem::take(&mut heights)),
        )
    };
    println!("LEARNED_STRIDE (example 62):");
    let (backward, _) = median_windows(UnitreeG1TorqueOverlay::LEARNED_STRIDE);
    println!("FORWARD_STRIDE (this search):");
    let (forward, height) = median_windows(UnitreeG1TorqueOverlay::FORWARD_STRIDE);
    println!(
        "median signed window: LEARNED_STRIDE {backward:+.2} m, FORWARD_STRIDE {forward:+.2} m"
    );
    assert!(
        backward < 0.0,
        "the example-62 overlay is expected to walk backwards; if it no longer does, this \
         example's premise is stale: {backward:+.2} m"
    );
    assert!(
        forward > FORWARD_MIN_WINDOW_M,
        "the forward walk must cover ground the way the robot faces: {forward:+.2} m"
    );
    assert!(
        height > 0.70,
        "the forward walk fell: minimum height {height:.3} m"
    );
}
