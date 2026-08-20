//! Headless office AGV desk-place mission analog.
//!
//! Shared-aisle delivery plus kinematic cargo unload into the desk place box.
//! `--smoke` proves success and a skip-place Failure Capsule contract.

use rne_ai::{
    office_agv_desk_place_task_spec, run_behavior_scenarios, BehaviorContractStatus,
    BehaviorSeedStatus, OfficeAgvDeskPlaceFault, OfficeAgvDeskPlaceScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let fail_skip_place = std::env::args().any(|argument| argument == "--fail-skip-place");

    office_agv_desk_place_task_spec(2_400)
        .validate()
        .expect("office AGV desk-place TaskSpec");

    if !fail_skip_place {
        let success = run_behavior_scenarios(
            "office_agv_desk_place_success",
            [1],
            OfficeAgvDeskPlaceScenario::success,
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

    if smoke || fail_skip_place {
        let failure = run_behavior_scenarios("office_agv_desk_place_skip", [1], |seed| {
            OfficeAgvDeskPlaceScenario::new(seed, OfficeAgvDeskPlaceFault::SkipPlace)
        });
        let place = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "desk_place_complete")
            .expect("desk place contract");
        println!(
            "skip-place: status={:?} place={:?} steps={}",
            failure.seeds[0].status, place.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || place.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
