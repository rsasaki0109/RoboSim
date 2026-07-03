//! Fixed-base clutter pick-and-place E2E: IK-assisted approach on the `mm_minimal_clutter`
//! table, then IK carry toward the fixed-base ground place target.
//!
//! `--smoke` asserts grasp and place on the center cube (`clutter_cube_b`).

use rne_ai::{
    mm_minimal_clutter_place_target, mm_minimal_clutter_scene_path, Episode,
    IkClutterPickPlacePolicy, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    MobileManipulatorRewardConfig, MobileManipulatorTask, Policy,
};

fn clutter_center_place_config() -> MobileManipulatorEpisodeConfig {
    let target = mm_minimal_clutter_place_target();
    MobileManipulatorEpisodeConfig {
        max_steps: 960,
        scene_path: mm_minimal_clutter_scene_path(),
        task: MobileManipulatorTask::Place {
            object_name: "clutter_cube_b".into(),
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

fn run_ik_clutter(episode: &mut MobileManipulatorEpisode) -> (bool, bool) {
    let mut policy = IkClutterPickPlacePolicy::new();
    let total_steps = policy.total_steps();
    let mut step = episode.reset();
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
    let mut episode = MobileManipulatorEpisode::new(clutter_center_place_config());
    let (grasped, placed) = run_ik_clutter(&mut episode);

    if smoke {
        if grasped && placed {
            println!(
                "clutter pick-place smoke ok: grasped and placed center cube, steps={}",
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
        "clutter pick-place: grasped={grasped} placed={placed} reward={:.2} steps={}",
        episode.total_reward(),
        episode.step_in_episode()
    );
}
