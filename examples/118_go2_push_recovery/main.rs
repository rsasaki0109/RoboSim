//! Unitree Go2 push recovery under a real external wrench, headless.
//!
//! Disturbance work in this repository used `tilt_named_body_rad`, which
//! re-poses a root body: the contact configuration changes, but nothing is
//! resisted by inertia or actuation because no force enters the solver.
//! `UrdfSceneSim::apply_named_link_wrench` pushes the trunk with an actual
//! world-frame force, so the disturbance is opposed by contact friction, the
//! robot's inertia, and the position motors, exactly as a shove would be.
//!
//! The sweep finds the force at which the pinned standing pose stops
//! recovering. No renderer is involved. Run with:
//!
//! ```text
//! cargo run --release -p go2_push_recovery --example 118_go2_push_recovery
//! ```

use rne_ai::env::urdf_scene::{UrdfJointPositionTarget, UrdfSceneSim};
use rne_legged::{Horizontal, SupportPolygon};
use rne_math::Vec3;

/// Settling steps before the push, at the scene's 60 Hz step.
const SETTLE_STEPS: usize = 180;
/// Steps over which the push force is held.
const PUSH_STEPS: usize = 12;
/// Recovery steps scored after the push ends.
const RECOVERY_STEPS: usize = 240;
/// Lateral push forces swept, in newtons.
const PUSH_FORCE_N: [f64; 6] = [0.0, 40.0, 80.0, 120.0, 160.0, 200.0];
/// Height above the trunk origin at which the push is applied, in meters.
///
/// A shove lands on the body, not at its center of mass, so the offset induces
/// the accompanying moment.
const PUSH_HEIGHT_OFFSET_M: f64 = 0.06;
/// Position-motor gains and standing targets pinned by the flat-ground
/// `official_unitree_go2_dynamic_multibody_stands_on_four_feet` regression.
const MOTOR_STIFFNESS: f64 = 180.0;
const MOTOR_DAMPING: f64 = 18.0;
const MOTOR_MAX_EFFORT_NM: f64 = 23.7;
const HIP_TARGET_RAD: f64 = 0.0;
const THIGH_TARGET_RAD: f64 = 0.8;
const CALF_TARGET_RAD: f64 = -1.5;
/// Recovery gate: minimum trunk height, in meters.
const MIN_RECOVERED_HEIGHT_M: f64 = 0.18;
/// Recovery gate: maximum residual tilt, in radians.
const MAX_RECOVERED_TILT_RAD: f64 = 0.20;

const TRUNK_LINK: &str = "base";
const FEET: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
/// Contact samples within this distance collapse to one support vertex, so the
/// several samples a solver reports per foot do not become their own polygon.
const CONTACT_MERGE_TOLERANCE_M: f64 = 0.03;

/// Signed distance from the ground-projected center of mass to the boundary of
/// the measured support polygon, in meters.
///
/// Positive while the projection is supported. This is the mechanism behind a
/// fall: support is lost when the margin crosses zero, before the body tilts.
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

/// The pinned standing pose, reused unchanged at every push magnitude.
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

struct PushResult {
    force_n: f64,
    peak_tilt_rad: f64,
    residual_tilt_rad: f64,
    lateral_displacement_m: f64,
    /// Smallest stability margin reached during the push and recovery.
    min_margin_m: f64,
    recovered_feet: usize,
    recovered: bool,
}

fn tilt_rad(observation: &rne_ai::env::urdf_scene::UrdfSceneObservation) -> f64 {
    observation
        .base_relative_roll_rad
        .abs()
        .max(observation.base_relative_pitch_rad.abs())
}

fn measure_push(force_n: f64) -> PushResult {
    let mut sim = UrdfSceneSim::from_scene_path(&UrdfSceneSim::unitree_go2_dynamic_scene_path())
        .expect("spawn dynamic Go2");
    sim.configure_position_motors(MOTOR_STIFFNESS, MOTOR_DAMPING, MOTOR_MAX_EFFORT_NM);
    let targets = standing_targets();

    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
    let settled = sim.observe();

    // Lateral shove along +Z, held for a bounded window.
    let mut peak_tilt_rad = tilt_rad(&settled);
    let mut min_margin_m = stability_margin_m(&sim);
    for _ in 0..PUSH_STEPS {
        if force_n != 0.0 {
            let trunk_m = sim
                .named_link_position_m(TRUNK_LINK)
                .expect("Go2 trunk link");
            let applied = sim.apply_named_link_wrench(
                TRUNK_LINK,
                trunk_m + Vec3::new(0.0, PUSH_HEIGHT_OFFSET_M, 0.0),
                Vec3::new(0.0, 0.0, force_n),
                Vec3::ZERO,
            );
            assert!(applied, "the trunk link must accept an external wrench");
        }
        sim.step_joint_position_targets(&targets);
        peak_tilt_rad = peak_tilt_rad.max(tilt_rad(&sim.observe()));
        min_margin_m = min_margin_m.min(stability_margin_m(&sim));
    }

    for _ in 0..RECOVERY_STEPS {
        sim.step_joint_position_targets(&targets);
        peak_tilt_rad = peak_tilt_rad.max(tilt_rad(&sim.observe()));
        min_margin_m = min_margin_m.min(stability_margin_m(&sim));
    }
    let recovered = sim.observe();
    let recovered_feet = FEET
        .into_iter()
        .filter(|foot| sim.link_contact_impulse_ns(foot) > 0.0)
        .count();
    let residual_tilt_rad = tilt_rad(&recovered);

    PushResult {
        force_n,
        peak_tilt_rad,
        residual_tilt_rad,
        lateral_displacement_m: recovered.base_z_m - settled.base_z_m,
        min_margin_m,
        recovered_feet,
        recovered: recovered.base_y_m > MIN_RECOVERED_HEIGHT_M
            && residual_tilt_rad < MAX_RECOVERED_TILT_RAD
            && recovered_feet == FEET.len(),
    }
}

fn main() {
    println!("Go2 pinned stance under a lateral trunk wrench");
    println!("force_n  peak_tilt  residual_tilt  lateral_m  min_margin_m  feet  recovered");

    let mut results = Vec::new();
    for force_n in PUSH_FORCE_N {
        let result = measure_push(force_n);
        println!(
            "{:7.0}  {:9.3}  {:13.3}  {:+9.3}  {:+12.4}  {:4}  {}",
            result.force_n,
            result.peak_tilt_rad,
            result.residual_tilt_rad,
            result.lateral_displacement_m,
            result.min_margin_m,
            result.recovered_feet,
            if result.recovered { "yes" } else { "no" }
        );
        results.push(result);
    }

    let undisturbed = results.first().expect("zero-force reference");
    assert!(
        undisturbed.recovered,
        "the zero-force case must reproduce the pinned standing pose"
    );

    // The wrench must actually disturb the plant: a re-posing hack would leave
    // the peak tilt indistinguishable from the undisturbed run.
    let strongest = results.last().expect("swept force");
    assert!(
        strongest.peak_tilt_rad > undisturbed.peak_tilt_rad + 0.02,
        "the swept wrench never disturbed the stance: {:.4} vs {:.4} rad",
        strongest.peak_tilt_rad,
        undisturbed.peak_tilt_rad
    );
    assert!(
        !strongest.recovered,
        "the sweep must reach a force the pinned stance cannot recover from"
    );
    // Response must be ordered in the force while the robot is still on its
    // feet. Past the tipping point the body rotates through large angles and
    // settles in whatever orientation it lands in, so tilt stops being an
    // ordered measure of push strength and is only a fall indicator.
    let recovering: Vec<&PushResult> = results
        .iter()
        .take_while(|result| result.recovered)
        .collect();
    assert!(
        recovering.len() >= 2,
        "the sweep must contain at least two recovering cases to order"
    );
    assert!(
        recovering
            .windows(2)
            .all(|pair| pair[1].peak_tilt_rad >= pair[0].peak_tilt_rad - 1e-9),
        "peak tilt must not decrease with push force while the stance holds"
    );

    let recovers_to_n = results
        .iter()
        .take_while(|result| result.recovered)
        .map(|result| result.force_n)
        .last()
        .expect("zero-force reference recovers");
    println!("pinned stance recovers up to {recovers_to_n:.0} N of the swept forces");

    // The stability margin must explain the boundary rather than restate it:
    // a stance that recovers never loses support, and one that falls does.
    for result in &results {
        if result.recovered {
            assert!(
                result.min_margin_m > 0.0,
                "{} N recovered while reporting lost support: margin {:.4} m",
                result.force_n,
                result.min_margin_m
            );
        } else {
            assert!(
                result.min_margin_m <= 0.0,
                "{} N fell while support was never lost: margin {:.4} m",
                result.force_n,
                result.min_margin_m
            );
        }
    }

    let repeat = measure_push(PUSH_FORCE_N[1]);
    assert_eq!(
        repeat.peak_tilt_rad.to_bits(),
        results[1].peak_tilt_rad.to_bits(),
        "repeated push must replay exactly"
    );
    println!(
        "replay of the {:.0} N case is bit-identical",
        PUSH_FORCE_N[1]
    );
}
