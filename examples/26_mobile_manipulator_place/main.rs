//! Mobile manipulator pick-and-place episode: grasp a cube, carry it, and set it
//! down at a target location.
//!
//! The gripper uses contact-triggered welding (see `rne_physics::FixedJointDesc`):
//! closing onto the cube attaches it to the end-effector; opening releases it.

use rne_ai::{
    Episode, MobileManipulatorAction, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
};

/// Scripted pick-and-place rollout. Returns the terminating step when it succeeds.
fn run_place(episode: &mut MobileManipulatorEpisode) -> bool {
    let close = MobileManipulatorAction {
        gripper_velocity_rad_s: -2.5,
        ..MobileManipulatorAction::default()
    };
    let carry = MobileManipulatorAction {
        gripper_velocity_rad_s: -2.0,
        shoulder_velocity_rad_s: 0.6,
        ..MobileManipulatorAction::default()
    };
    let hold = MobileManipulatorAction {
        gripper_velocity_rad_s: -2.0,
        ..MobileManipulatorAction::default()
    };
    let open = MobileManipulatorAction {
        gripper_velocity_rad_s: 3.0,
        ..MobileManipulatorAction::default()
    };

    // Close the gripper until the cube is grasped (welded to the end-effector).
    for _ in 0..30 {
        episode.step(close);
        if episode.simulation().is_grasping() {
            break;
        }
    }
    // Carry the cube along the arm sweep, then settle the arm before release.
    // 60 steps at 0.6 rad/s: the sweep the place() target was derived from under
    // the stable arm dynamics.
    for _ in 0..60 {
        episode.step(carry);
    }
    for _ in 0..30 {
        episode.step(hold);
    }
    // Open the gripper to release the cube and let it settle at the target.
    for _ in 0..150 {
        let step = episode.step(open);
        if step.terminated {
            return true;
        }
    }
    false
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::place());
    let _ = episode.reset();
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
