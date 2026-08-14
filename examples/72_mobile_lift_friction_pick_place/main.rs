//! Headless flagship demo for an engine-native lift-capable mobile manipulator.
//!
//! The rollout uses a dynamic `mm_mobile_lift` URDF, a free rigid-body cube, a
//! force-limited parallel-jaw friction grasp (no weld joint), and the wrist RGB-D
//! sensor. It completes navigate → approach → grasp → lift → transport → place
//! with fixed-step deterministic policy control.

use rne_ai::{
    Episode, GraspMode, IkMobileLiftPickPlacePolicy, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, Policy,
};

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut episode =
        MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::mobile_lift_pick_place());
    let mut policy = IkMobileLiftPickPlacePolicy::new();
    let mut step = episode.reset();
    episode.set_grasp_mode(GraspMode::Friction);
    let resting_y = episode
        .simulation()
        .named_translation_m("mobile_lift_cube")
        .expect("mobile lift cube")
        .1;
    let mut grasped = false;
    let mut max_cube_y = resting_y;

    for _ in 0..policy.total_steps() {
        step = episode.step(policy.act(&step.observation));
        grasped |= episode.simulation().is_grasping();
        max_cube_y = max_cube_y.max(
            episode
                .simulation()
                .named_translation_m("mobile_lift_cube")
                .expect("mobile lift cube")
                .1,
        );
        if step.is_done() {
            break;
        }
    }

    let placed = episode
        .simulation()
        .named_translation_m("mobile_lift_cube")
        .expect("mobile lift cube");
    let success = step.terminated && grasped && max_cube_y > resting_y + 0.12;

    if smoke {
        if !success {
            eprintln!(
                "mobile-lift smoke failed: phase={:?} failure={:?} terminated={} grasped={} max_y={:.3} final=({:.3},{:.3},{:.3})",
                policy.phase(),
                policy.failure_class(&step.observation),
                step.terminated,
                grasped,
                max_cube_y,
                placed.0,
                placed.1,
                placed.2,
            );
            std::process::exit(1);
        }
        println!(
            "mobile-lift friction smoke ok: phase={:?} cube_clearance={:.3} m placed=({:.3},{:.3},{:.3}) rgbd={} px depth={:.3} m",
            policy.phase(),
            max_cube_y - resting_y,
            placed.0,
            placed.1,
            placed.2,
            step.observation.wrist_camera_pixels,
            step.observation.wrist_depth_min_m,
        );
        return;
    }

    println!(
        "mobile-lift friction rollout: phase={:?} success={} failure={:?} grasped={} clearance={:.3} m cube=({:.3},{:.3},{:.3})",
        policy.phase(),
        success,
        policy.failure_class(&step.observation),
        grasped,
        max_cube_y - resting_y,
        placed.0,
        placed.1,
        placed.2,
    );
}
