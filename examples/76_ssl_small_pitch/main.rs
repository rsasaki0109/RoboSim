//! Headless RoboCup SSL Division B 2v2 analog.
//!
//! Scores official field geometry (goal mouth, out-of-bounds, 6.5 m/s ball
//! cap). This is not grSim and does not bind the SSL protobuf ports.

use rne_ai::{
    run_behavior_scenarios, ssl_small_pitch_task_spec, BehaviorContractStatus, BehaviorSeedStatus,
    SslSmallPitchFault, SslSmallPitchScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let fail_out = std::env::args().any(|argument| argument == "--fail-out");

    ssl_small_pitch_task_spec(2_000)
        .validate()
        .expect("ssl small-pitch TaskSpec");

    if !fail_out {
        let success = run_behavior_scenarios(
            "ssl_small_pitch_success",
            [1],
            SslSmallPitchScenario::success,
        );
        println!(
            "success: passed={} steps={}",
            success.passed(),
            success.seeds[0].steps
        );
        if !success.passed() {
            eprintln!("{success:?}");
            std::process::exit(1);
        }
    }

    if smoke || fail_out {
        let failure = run_behavior_scenarios("ssl_small_pitch_drive_out", [1], |seed| {
            SslSmallPitchScenario::new(seed, SslSmallPitchFault::DriveOut)
        });
        let in_play = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "ball_in_play_or_goal")
            .expect("in-play contract");
        println!(
            "drive-out: status={:?} in_play={:?} steps={}",
            failure.seeds[0].status, in_play.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || in_play.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
