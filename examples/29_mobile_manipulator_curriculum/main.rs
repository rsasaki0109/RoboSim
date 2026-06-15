//! Goal-conditioned reach with an easy-to-hard curriculum.
//!
//! Runs a fixed goal-conditioned policy across episodes; the curriculum widens the target
//! region as the policy succeeds, so the active stage climbs from 0 to the final stage.

use rne_ai::{
    Episode, MobileManipulatorAction, MobileManipulatorEpisode, MobileManipulatorEpisodeConfig,
    MobileManipulatorObservation,
};

const MAX_EPISODES: usize = 30;
const EPISODE_STEPS: usize = 500;

/// Goal-conditioned proportional policy using only the observation's goal offset.
fn policy(obs: &MobileManipulatorObservation) -> MobileManipulatorAction {
    MobileManipulatorAction {
        shoulder_velocity_rad_s: (2.5 * obs.target_dx_m - 0.5 * obs.target_dy_m).clamp(-6.0, 6.0),
        elbow_velocity_rad_s: (1.5 * obs.target_dx_m + 3.0 * obs.target_dz_m).clamp(-6.0, 6.0),
        ..MobileManipulatorAction::default()
    }
}

/// Runs episodes until the curriculum reaches its final stage; returns (final_stage, solved).
fn run_curriculum(episode: &mut MobileManipulatorEpisode, final_stage: usize) -> (usize, usize) {
    let mut solved = 0;
    let mut step = episode.reset();
    for _ in 0..MAX_EPISODES {
        let mut obs = step.observation;
        for _ in 0..EPISODE_STEPS {
            let result = episode.step(policy(&obs));
            obs = result.observation;
            if result.terminated {
                solved += 1;
                break;
            }
            if result.truncated {
                break;
            }
        }
        let stage = episode.curriculum_stage().unwrap_or(0);
        if stage >= final_stage {
            return (stage, solved);
        }
        step = episode.reset();
    }
    (episode.curriculum_stage().unwrap_or(0), solved)
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut episode =
        MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::reach_curriculum(5));
    let final_stage = 2;
    let (stage, solved) = run_curriculum(&mut episode, final_stage);

    if smoke {
        if stage >= final_stage {
            println!("curriculum smoke ok: reached stage {stage} after {solved} solved episodes");
            return;
        }
        eprintln!("smoke failed: curriculum stuck at stage {stage} ({solved} solved)");
        std::process::exit(1);
    }

    println!("curriculum: final stage={stage}, solved episodes={solved}");
}
