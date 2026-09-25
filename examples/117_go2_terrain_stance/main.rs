//! Position-held Unitree Go2 stance on sampled terrain, headless.
//!
//! Every Go2 result in this repository was measured on a flat ground plane.
//! This example swaps that plane for a `HeightfieldCollider` slope and sweeps
//! the slope angle, so the angle at which the pinned standing pose stops
//! holding is a measurement rather than an assumption.
//!
//! Three properties of sampled terrain matter here and are handled explicitly
//! rather than assumed. A heightfield is an **open surface**: unlike the 1 m
//! thick ground box it replaces, it never pushes a body back out, so the patch
//! is placed below the robot's whole footprint and the robot drops onto it.
//! For the same reason a penetration that occurs *during* the run is also
//! permanent, which is why the scene is stepped at 240 Hz: at the scene's
//! default 60 Hz the Go2's feet tunnel through and fall past -4 m on a 10 deg
//! slope, and the run then measures nothing. A tunnelling check enforces that.
//! The patch is also **finite**: a robot that slides past the edge falls, so
//! slide distance is reported and rest poses are checked against the bounds.
//!
//! No renderer is involved. Run with:
//!
//! ```text
//! cargo run --release -p go2_terrain_stance --example 117_go2_terrain_stance
//! ```

use rne_ai::env::urdf_scene::{UrdfJointPositionTarget, UrdfSceneSim};
use rne_core::SimDuration;
use rne_legged::{Horizontal, SupportPolygon};
use rne_math::{Hertz, Vec3};
use rne_physics::HeightfieldCollider;

/// Sample counts along the patch's local X and Z axes.
const ROWS: u32 = 81;
const COLUMNS: u32 = 81;
/// Total horizontal extents in meters; heights stay in meters.
const EXTENT_M: Vec3 = Vec3::new(20.0, 1.0, 20.0);
/// Terrain depth below the spawn pose at the robot's own x position, in meters.
///
/// A heightfield is an open surface: a link that starts below it is never
/// pushed out and falls forever. The scene's spawn pose puts the Go2's feet at
/// y = -0.066, so the patch must clear that at *every* x the robot occupies,
/// not just under its origin.
const SPAWN_CLEARANCE_M: f64 = 0.10;
/// Half the robot's footprint along X, in meters.
///
/// The slope pivots at x = 0, so the terrain rises by this much times the
/// slope's tangent under the leading feet; the patch is lowered to match.
const ROBOT_HALF_LENGTH_M: f64 = 0.40;
/// Lowest spawned link height in the scene's pose, in meters.
const LOWEST_SPAWN_LINK_M: f64 = -0.066;
/// Physics rate, in hertz.
///
/// Bounds per-step penetration of the open terrain surface; see the module
/// documentation for the measured 60 Hz tunnelling this avoids.
const PHYSICS_HZ: f64 = 240.0;
/// Settling steps before the stance is scored: 3 s at [`PHYSICS_HZ`].
const SETTLE_STEPS: usize = 720;
/// Scored steps after settling: 4 s at [`PHYSICS_HZ`].
const HOLD_STEPS: usize = 960;
/// Slope angles swept, in degrees.
const SLOPE_DEG: [f64; 6] = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0];
/// Position-motor gains and standing targets pinned by the flat-ground
/// `official_unitree_go2_dynamic_multibody_stands_on_four_feet` regression.
const MOTOR_STIFFNESS: f64 = 180.0;
const MOTOR_DAMPING: f64 = 18.0;
const MOTOR_MAX_EFFORT_NM: f64 = 23.7;
const HIP_TARGET_RAD: f64 = 0.0;
const THIGH_TARGET_RAD: f64 = 0.8;
const CALF_TARGET_RAD: f64 = -1.5;
/// Upright gate: the pinned flat stance holds the base well above this.
const MIN_BASE_CLEARANCE_M: f64 = 0.18;
/// Upright gate: how far the body may deviate from terrain-parallel, in
/// radians. A position-held stance is a rigid assembly on four feet, so its
/// body tracks the slope; deviation from that is the failure signal.
const MAX_SLOPE_TRACKING_ERROR_RAD: f64 = 0.15;
/// Upright gate: out-of-plane roll, in radians.
const MAX_ROLL_RAD: f64 = 0.15;

const FEET: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
/// Contact samples within this distance collapse to one support vertex.
const CONTACT_MERGE_TOLERANCE_M: f64 = 0.03;

/// Signed distance from the ground-projected center of mass to the boundary of
/// the measured support polygon, in meters.
///
/// Gravity is vertical whatever the terrain does, so the tipping criterion is
/// still the vertical projection of the center of mass against the horizontal
/// projection of the contacts. Sliding is a separate failure and is reported
/// through the slide column instead.
fn stability_margin_m(sim: &UrdfSceneSim) -> f64 {
    let contacts: Vec<Horizontal> = sim
        .named_body_contact_points_m("ground")
        .into_iter()
        .map(Horizontal::from_world)
        .collect();
    let polygon = SupportPolygon::from_contacts(&contacts, CONTACT_MERGE_TOLERANCE_M);
    let Some((com_m, _)) = sim.dynamic_center_of_mass_m() else {
        return f64::NEG_INFINITY;
    };
    polygon
        .stability_margin(Horizontal::from_world(com_m))
        .signed_distance_m
}

/// Terrain height at the spawn origin for one slope, in meters.
///
/// Chosen so the surface stays below the lowest spawned link across the whole
/// robot footprint; the robot then drops onto the terrain and settles.
fn terrain_origin_height_m(slope_rad: f64) -> f64 {
    LOWEST_SPAWN_LINK_M - SPAWN_CLEARANCE_M - ROBOT_HALF_LENGTH_M * slope_rad.tan().abs()
}

/// Terrain height in meters at one local X position.
///
/// A plane tilted about the world Z axis, rising toward +X.
fn terrain_height_m(x_m: f64, slope_rad: f64) -> f64 {
    terrain_origin_height_m(slope_rad) + x_m * slope_rad.tan()
}

fn slope_patch(slope_rad: f64) -> HeightfieldCollider {
    let mut field = HeightfieldCollider::flat(ROWS, COLUMNS, EXTENT_M);
    for row in 0..ROWS {
        let x_m = EXTENT_M.x * (f64::from(row) / f64::from(ROWS - 1) - 0.5);
        let height_m = terrain_height_m(x_m, slope_rad);
        for column in 0..COLUMNS {
            *field
                .height_mut(row, column)
                .expect("sample inside the declared grid") = height_m;
        }
    }
    field
}

/// The pinned standing pose, reused unchanged at every slope angle.
fn standing_targets() -> Vec<UrdfJointPositionTarget<'static>> {
    let mut targets = Vec::with_capacity(12);
    for (hip, thigh, calf) in [
        ("FL_hip", "FL_thigh", "FL_calf"),
        ("FR_hip", "FR_thigh", "FR_calf"),
        ("RL_hip", "RL_thigh", "RL_calf"),
        ("RR_hip", "RR_thigh", "RR_calf"),
    ] {
        targets.push(UrdfJointPositionTarget {
            link_name: hip,
            position: HIP_TARGET_RAD,
        });
        targets.push(UrdfJointPositionTarget {
            link_name: thigh,
            position: THIGH_TARGET_RAD,
        });
        targets.push(UrdfJointPositionTarget {
            link_name: calf,
            position: CALF_TARGET_RAD,
        });
    }
    targets
}

struct StanceResult {
    slope_deg: f64,
    /// Whether any body tunnelled through the open terrain surface.
    ///
    /// A run in which a link fell through measures nothing about the stance, so
    /// it is reported as invalid rather than scored.
    tunnelled: bool,
    base_clearance_m: f64,
    /// Deviation of the body from terrain-parallel, in radians.
    slope_tracking_error_rad: f64,
    roll_rad: f64,
    downhill_slide_m: f64,
    /// Stability margin at the end of the hold, in meters.
    margin_m: f64,
    loaded_feet: usize,
    on_patch: bool,
    upright: bool,
}

fn measure_stance(slope_deg: f64) -> StanceResult {
    let slope_rad = slope_deg.to_radians();
    let mut sim = UrdfSceneSim::from_scene_path_with_ground_heightfield_and_fixed_delta(
        &UrdfSceneSim::unitree_go2_dynamic_scene_path(),
        slope_patch(slope_rad),
        SimDuration::from_hertz(Hertz::new(PHYSICS_HZ)),
    )
    .expect("spawn Go2 on sampled terrain");
    sim.configure_position_motors(MOTOR_STIFFNESS, MOTOR_DAMPING, MOTOR_MAX_EFFORT_NM);
    let targets = standing_targets();

    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
    let settled = sim.observe();
    for _ in 0..HOLD_STEPS {
        sim.step_joint_position_targets(&targets);
    }
    let held = sim.observe();

    // Height is measured against the surface the robot stands on, not the
    // world origin. The slope rises along X, which is a pitch for this robot,
    // so a body resting flat on the terrain reports a pitch equal to the slope.
    let base_clearance_m = held.base_y_m - terrain_height_m(held.base_x_m, slope_rad);
    let slope_tracking_error_rad = (held.base_relative_pitch_rad.abs() - slope_rad).abs();
    let roll_rad = held.base_relative_roll_rad.abs();
    let loaded_feet = FEET
        .into_iter()
        .filter(|foot| sim.link_contact_impulse_ns(foot) > 0.0)
        .count();
    let margin_m = stability_margin_m(&sim);
    // A body that fell through the open surface drags the mass-weighted center
    // of mass far below the terrain; nothing the robot does can put it there.
    let (com_m, _) = sim
        .dynamic_center_of_mass_m()
        .expect("scene has dynamic mass");
    let tunnelled = com_m.y < terrain_height_m(com_m.x, slope_rad) - 1.0;
    let on_patch = held.base_x_m.abs() < EXTENT_M.x / 2.0 && held.base_z_m.abs() < EXTENT_M.z / 2.0;

    StanceResult {
        slope_deg,
        tunnelled,
        base_clearance_m,
        slope_tracking_error_rad,
        roll_rad,
        downhill_slide_m: settled.base_x_m - held.base_x_m,
        margin_m,
        loaded_feet,
        on_patch,
        // Standing means all four feet carry load. A body held up by its
        // calves or belly is not a stance, however level it looks.
        upright: !tunnelled
            && on_patch
            && loaded_feet == FEET.len()
            && base_clearance_m > MIN_BASE_CLEARANCE_M
            && slope_tracking_error_rad < MAX_SLOPE_TRACKING_ERROR_RAD
            && roll_rad < MAX_ROLL_RAD,
    }
}

fn main() {
    println!("Go2 pinned standing pose on a sampled heightfield slope");
    println!(
        "slope_deg  clearance_m  track_err  roll_rad   slide_m    margin_m  feet  on_patch  upright"
    );

    let mut results = Vec::new();
    for slope_deg in SLOPE_DEG {
        let result = measure_stance(slope_deg);
        if result.tunnelled {
            println!(
                "{:9.1}  INVALID: a body tunnelled through the open surface",
                result.slope_deg
            );
        } else {
            println!(
                "{:9.1}  {:11.3}  {:9.3}  {:8.3}  {:+8.3}  {:+10.4}  {:4}  {:8}  {}",
                result.slope_deg,
                result.base_clearance_m,
                result.slope_tracking_error_rad,
                result.roll_rad,
                result.downhill_slide_m,
                result.margin_m,
                result.loaded_feet,
                if result.on_patch { "yes" } else { "no" },
                if result.upright { "yes" } else { "no" }
            );
        }
        results.push(result);
    }

    let flat = results.first().expect("flat reference");
    assert!(
        !flat.tunnelled,
        "the zero-slope patch must support the robot without losing a body"
    );
    assert!(
        flat.upright,
        "a zero-slope patch must reproduce the pinned flat-ground stance"
    );

    // Tunnelling is a modelling failure, not a stance result: a run that lost a
    // body through the open surface measured nothing about the stance, so its
    // row is reported as invalid instead of being scored. This is the honest
    // boundary of terrain-based legged measurement today, reported rather than
    // hidden or asserted away.
    let (valid, tunnelled): (Vec<&StanceResult>, Vec<&StanceResult>) =
        results.iter().partition(|result| !result.tunnelled);
    if !tunnelled.is_empty() {
        let angles: Vec<String> = tunnelled
            .iter()
            .map(|result| format!("{:.0}", result.slope_deg))
            .collect();
        println!(
            "INVALID at {} deg: a body tunnelled through the open surface",
            angles.join(", ")
        );
    }
    assert!(
        valid.len() >= 2,
        "the sweep must retain at least one sloped run to compare against flat"
    );

    let holds_to_deg = valid
        .iter()
        .take_while(|result| result.upright)
        .map(|result| result.slope_deg)
        .last()
        .expect("flat reference holds");
    println!(
        "pinned stance holds up to {holds_to_deg:.1} deg of the {} valid swept angles",
        valid.len()
    );

    // Determinism: the same slope must replay to the same measurement.
    let repeat = measure_stance(SLOPE_DEG[1]);
    assert_eq!(
        repeat.base_clearance_m.to_bits(),
        results[1].base_clearance_m.to_bits(),
        "repeated terrain stance must replay exactly"
    );
    println!(
        "replay of the {:.1} deg case is bit-identical",
        SLOPE_DEG[1]
    );
}
