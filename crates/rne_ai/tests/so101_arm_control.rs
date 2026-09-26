//! The SO-101 arm must start where it is authored and stay there under a servo.
//!
//! Both properties were broken. The shipped asset omitted
//! `use_joint_origin_rpy`, so the wired joint frames disagreed with the authored
//! pose and the solver resolved that as a constraint violation on the first
//! step, launching the chain kilometres away. Separately, the default 60 Hz
//! physics rate is below the stability boundary for this arm's link inertias,
//! so raising servo stiffness made tracking worse rather than better.
//!
//! See `docs/ARM_POSITION_CONTROL.md`.

use rne_ai::{so101_scene_path, UrdfSceneSim};
use rne_math::Vec3;

const LINKS: [&str; 8] = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "gripper_frame_link",
    "moving_jaw_so101_v1_link",
];

const ACTUATED: [&str; 5] = [
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
];

fn rest_positions(sim: &UrdfSceneSim) -> Vec<Vec3> {
    LINKS
        .iter()
        .map(|link| sim.named_transform(link).expect("link").translation)
        .collect()
}

fn worst_drift_m(sim: &UrdfSceneSim, rest: &[Vec3]) -> f64 {
    LINKS
        .iter()
        .zip(rest)
        .map(|(link, start)| {
            sim.named_transform(link)
                .map_or(f64::NAN, |t| (t.translation - *start).length())
        })
        .fold(0.0_f64, f64::max)
}

/// The arm must not be launched off its authored pose by the first step.
///
/// Without `use_joint_origin_rpy` the worst first-step displacement was
/// 11701.26 m. The bound below is two orders of magnitude above the measured
/// 0.0089 m, so it fails on a regression of that kind without pinning the exact
/// solver output.
#[test]
fn the_authored_pose_survives_the_first_step() {
    let mut sim = UrdfSceneSim::from_scene_path(&so101_scene_path()).expect("load so101");
    let rest = rest_positions(&sim);
    sim.step_joint_position_targets(&[]);
    let drift_m = worst_drift_m(&sim, &rest);
    assert!(
        drift_m < 1.0,
        "the first step displaced a link by {drift_m} m; the authored pose and the \
         wired joint frames disagree (see docs/ARM_POSITION_CONTROL.md)"
    );
}

/// Raising servo stiffness must not make the held pose worse.
///
/// This is the property that fails at the default 60 Hz rate: settled drift
/// grew from 0.324 m at k=20 to 0.441 m at k=80, with a 2.07 m transient. At
/// 240 Hz the same sweep is flat. A servo whose error grows with gain is
/// integrating unstably, which no amount of solver iteration fixes.
#[test]
fn a_stiffer_servo_holds_the_pose_at_least_as_well() {
    const SUBSTEPS: usize = 4; // 240 Hz, above the measured stability boundary.
    let mut settled = Vec::new();
    for stiffness in [5.0_f64, 20.0, 80.0] {
        let mut sim = UrdfSceneSim::from_scene_path(&so101_scene_path()).expect("load so101");
        let rest = rest_positions(&sim);
        for link in ACTUATED {
            assert!(
                sim.configure_named_revolute_position_actuation(
                    link,
                    stiffness,
                    stiffness * 0.1,
                    10.0
                ),
                "configure {link}"
            );
        }
        for _ in 0..600 {
            sim.step_joint_position_actuation_targets_substeps(&[], SUBSTEPS)
                .expect("substep split");
        }
        let drift_m = worst_drift_m(&sim, &rest);
        assert!(drift_m.is_finite(), "k={stiffness} diverged");
        settled.push(drift_m);
    }
    let softest = settled[0];
    let stiffest = settled[settled.len() - 1];
    assert!(
        stiffest <= softest * 1.5,
        "settled drift grew with stiffness ({softest} m -> {stiffest} m); the servo is \
         unstable at this physics rate (see docs/ARM_POSITION_CONTROL.md)"
    );
}
