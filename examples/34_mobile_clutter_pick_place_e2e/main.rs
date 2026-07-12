//! Mobile clutter navigate-and-place E2E: diff-drive approach on `mm_mobile_clutter`,
//! then IK arm pick-and-place toward the ground target.
//!
//! `--smoke` asserts the policy grasps `clutter_cube_a`; the full run also places it
//! on the ground target and terminates the episode.

use rne_ai::{
    mm_mobile_clutter_place_target, mm_mobile_clutter_scene_path, Episode, GraspMode,
    IkMobileClutterPickPlacePolicy, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    MobileManipulatorRewardConfig, MobileManipulatorTask, Policy,
};

fn mobile_clutter_place_config() -> MobileManipulatorEpisodeConfig {
    let target = mm_mobile_clutter_place_target();
    MobileManipulatorEpisodeConfig {
        max_steps: 1600,
        scene_path: mm_mobile_clutter_scene_path(),
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

fn run_mobile_clutter(episode: &mut MobileManipulatorEpisode) -> (bool, bool) {
    let mut policy = IkMobileClutterPickPlacePolicy::new();
    let total_steps = policy.total_steps();
    let mut step = episode.reset();
    episode.set_grasp_mode(GraspMode::Friction);
    let mut grasped = false;
    for _ in 0..total_steps {
        step = episode.step(policy.act(&step.observation));
        if episode.simulation().is_grasping() {
            grasped = true;
        }
    }
    (grasped, step.terminated)
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut episode = MobileManipulatorEpisode::new(mobile_clutter_place_config());
    let (grasped, placed) = run_mobile_clutter(&mut episode);

    if smoke {
        if grasped {
            println!(
                "mobile clutter smoke ok: grasped cube_a (placed={placed}), steps={}",
                episode.step_in_episode()
            );
            return;
        }
        eprintln!(
            "smoke failed: grasped={grasped} placed={placed} steps={}",
            episode.step_in_episode()
        );
        std::process::exit(1);
    }

    println!(
        "mobile clutter: grasped={grasped} placed={placed} reward={:.2} steps={}",
        episode.total_reward(),
        episode.step_in_episode()
    );
}
