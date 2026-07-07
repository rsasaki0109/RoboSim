//! Mobile manipulator pick-and-place episode on the clutter table: IK-assisted
//! approach to `clutter_cube_b`, grasp with contact-triggered welding, carry to the
//! fixed-base place target, and release.
//!
//! The gripper uses contact-triggered welding (see `rne_physics::FixedJointDesc`):
//! both finger links must touch the cube before it welds to the end-effector;
//! opening releases it at the target.

use rne_ai::{
    Episode, IkClutterPickPlacePolicy, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    Policy,
};

/// Drives the clutter IK pick-place policy until the episode terminates.
fn run_place(episode: &mut MobileManipulatorEpisode) -> bool {
    let mut policy = IkClutterPickPlacePolicy::new();
    let mut step = episode.reset();
    for _ in 0..policy.total_steps() {
        step = episode.step(policy.act(&step.observation));
        if step.terminated {
            return true;
        }
    }
    false
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::place());
    let placed = run_place(&mut episode);

    if smoke {
        if placed {
            println!(
                "place smoke ok: cube placed at target, reward={:.2} steps={}",
                episode.total_reward(),
                episode.step_in_episode()
            );
            return;
        }
        eprintln!("smoke failed: pick-and-place episode did not place the cube at the target");
        std::process::exit(1);
    }

    println!(
        "pick-and-place episode: placed={placed} reward={:.2} grasping={}",
        episode.total_reward(),
        episode.simulation().is_grasping()
    );
}
