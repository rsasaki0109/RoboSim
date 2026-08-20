//! Headless office AGV dock-to-desk delivery analog.
//!
//! Scores a short analytic corridor: stop on the pickup dock, then stop in the
//! delivery box in front of the desk, without leaving the aisle. `--smoke`
//! proves a successful scripted run and a skip-dock Failure Capsule contract.

use rne_ai::{
    office_agv_delivery_task_spec, run_behavior_scenarios, BehaviorContractStatus,
    BehaviorSeedStatus, OfficeAgvDeliveryFault, OfficeAgvDeliveryScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let fail_skip_dock = std::env::args().any(|argument| argument == "--fail-skip-dock");

    office_agv_delivery_task_spec(1_200)
        .validate()
        .expect("office AGV delivery TaskSpec");

    if !fail_skip_dock {
        let success = run_behavior_scenarios(
            "office_agv_delivery_success",
            [1],
            OfficeAgvDeliveryScenario::success,
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

    if smoke || fail_skip_dock {
        let failure = run_behavior_scenarios("office_agv_delivery_skip_dock", [1], |seed| {
            OfficeAgvDeliveryScenario::new(seed, OfficeAgvDeliveryFault::SkipDock)
        });
        let without_pickup = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_desk_without_pickup")
            .expect("desk without pickup contract");
        println!(
            "skip-dock: status={:?} without_pickup={:?} steps={}",
            failure.seeds[0].status, without_pickup.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || without_pickup.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
