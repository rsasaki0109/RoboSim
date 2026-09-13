//! SO101 mobile clutter navigate-and-place E2E: diff-drive approach on
//! `mm_mobile_so101_clutter`, then jaw-pocket pick-and-place toward the ground
//! target.
//!
//! `--smoke` asserts the policy reaches and grasps `clutter_cube_a` (the
//! moving jaw closes on the cube in the tip/anvil pocket and the weld latches).
//! Carrying the captured payload all the way to the place target is still being
//! tuned: the light single-jaw arm loses the swung payload during the carry
//! turn, so the run reports `placed=false`.

use rne_ai::{
    mm_mobile_so101_clutter_scene_path, so101_mobile_clutter_place_target, Episode,
    MobileManipulatorEpisode, MobileManipulatorEpisodeConfig, MobileManipulatorRewardConfig,
    MobileManipulatorTask, Policy, So101MobileClutterPickPlacePolicy,
};

/// Pocket-to-object horizontal distance (m) the reach must achieve.
const REACH_TOLERANCE_M: f64 = 0.05;

fn so101_clutter_place_config() -> MobileManipulatorEpisodeConfig {
    let target = so101_mobile_clutter_place_target();
    MobileManipulatorEpisodeConfig {
        max_steps: 2600,
        scene_path: mm_mobile_so101_clutter_scene_path(),
        task: MobileManipulatorTask::Place {
            object_name: "clutter_cube_a".into(),
            target,
            place_tolerance_m: 0.12,
        },
        reward: MobileManipulatorRewardConfig::default(),
        reach_randomization: None,
        reach_curriculum: None,
        clutter_pick: None,
        rng_seed: 0,
    }
}

/// Runs the policy and returns `(grasped_ever, placed, min_pocket_error_m)`.
fn run_so101_clutter(episode: &mut MobileManipulatorEpisode) -> (bool, bool, f64) {
    let mut policy = So101MobileClutterPickPlacePolicy::new();
    let total_steps = policy.total_steps();
    let mut step = episode.reset();
    let mut grasped = false;
    let mut min_pocket_error_m = f64::INFINITY;
    for _ in 0..total_steps {
        step = episode.step(policy.act(&step.observation));
        if episode.simulation().is_grasping() {
            grasped = true;
        }
        // Before the grasp the episode reports the pick object pose; after it
        // zeroes that pose, so only measure the reach while it is available.
        if step.observation.pick_object_y_m != 0.0 {
            let error_m = (step.observation.pick_object_x_m - step.observation.gripper_pocket_x_m)
                .hypot(step.observation.pick_object_z_m - step.observation.gripper_pocket_z_m);
            min_pocket_error_m = min_pocket_error_m.min(error_m);
        }
    }
    (grasped, step.terminated, min_pocket_error_m)
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut episode = MobileManipulatorEpisode::new(so101_clutter_place_config());
    let (grasped, placed, min_pocket_error_m) = run_so101_clutter(&mut episode);

    if smoke {
        if grasped && min_pocket_error_m <= REACH_TOLERANCE_M {
            println!(
                "so101 clutter smoke ok: reached and grasped cube_a (pocket error {min_pocket_error_m:.3} m; placed={placed}), steps={}",
                episode.step_in_episode()
            );
            return;
        }
        eprintln!(
            "smoke failed: grasped={grasped} min pocket error {min_pocket_error_m:.3} m (placed={placed}) steps={}",
            episode.step_in_episode()
        );
        std::process::exit(1);
    }

    println!(
        "so101 clutter: reached={:.3} m grasped={grasped} placed={placed} reward={:.2} steps={}",
        min_pocket_error_m,
        episode.total_reward(),
        episode.step_in_episode()
    );
}
