//! Mobile manipulator inspect + transport episodes with reward/termination.

use rne_ai::{
    Episode, MobileManipulatorAction, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
};

fn run_inspect_smoke() -> bool {
    let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::inspect());
    let _ = episode.reset();
    for _ in 0..180 {
        let step = episode.step(MobileManipulatorAction {
            shoulder_velocity_rad_s: 2.0,
            ..MobileManipulatorAction::default()
        });
        if step.terminated {
            println!(
                "inspect smoke ok: wrist_camera pixels={} reward={:.2}",
                step.observation.wrist_camera_pixels,
                episode.total_reward()
            );
            return true;
        }
    }
    false
}

fn run_transport_smoke() -> bool {
    let close = MobileManipulatorAction {
        gripper_velocity_rad_s: -2.5,
        ..MobileManipulatorAction::default()
    };
    let transport = MobileManipulatorAction {
        gripper_velocity_rad_s: -2.0,
        shoulder_velocity_rad_s: 4.0,
        ..MobileManipulatorAction::default()
    };

    for attempt in 1..=3 {
        let mut episode =
            MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::transport());
        let _ = episode.reset();
        for _ in 0..120 {
            episode.step(close);
        }
        for _ in 0..720 {
            let step = episode.step(transport);
            if step.terminated {
                println!(
                    "transport smoke ok: attempt={attempt} reward={:.2} steps={}",
                    episode.total_reward(),
                    episode.step_in_episode()
                );
                return true;
            }
        }
    }
    false
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    if smoke {
        if !run_inspect_smoke() {
            eprintln!("smoke failed: inspect episode did not terminate");
            std::process::exit(1);
        }
        if !run_transport_smoke() {
            eprintln!("smoke failed: transport episode did not reach drop zone");
            std::process::exit(1);
        }
        return;
    }

    let mut episode = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::inspect());
    let initial = episode.reset();
    println!(
        "inspect episode: wrist_camera pixels={}",
        initial.observation.wrist_camera_pixels
    );
}
