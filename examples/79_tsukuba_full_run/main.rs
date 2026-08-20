//! Headless Tsukuba Challenge 2026 shortened full-run analog.
//!
//! Scores three official stop-line boxes, timed pedestrian-signal waits, and
//! no roadway entry. This is not the 2.2 km city loop.

use rne_ai::{
    run_behavior_scenarios, tsukuba_full_run_task_spec, BehaviorContractStatus,
    BehaviorSeedStatus, TsukubaFullRunFault, TsukubaFullRunScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let skip_stops = std::env::args().any(|argument| argument == "--skip-stops");

    tsukuba_full_run_task_spec(2_400)
        .validate()
        .expect("tsukuba full-run TaskSpec");

    if !skip_stops {
        let success = run_behavior_scenarios(
            "tsukuba_full_run_success",
            [1],
            TsukubaFullRunScenario::success,
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

    if smoke || skip_stops {
        let failure = run_behavior_scenarios("tsukuba_full_run_skip_stops", [1], |seed| {
            TsukubaFullRunScenario::new(seed, TsukubaFullRunFault::SkipStopLines)
        });
        let stop = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "first_stop_line_stop")
            .expect("first stop contract");
        println!(
            "skip-stops: status={:?} stop={:?} steps={}",
            failure.seeds[0].status,
            stop.status,
            failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || stop.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
