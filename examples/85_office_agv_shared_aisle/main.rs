//! Headless office AGV shared-aisle delivery analog.
//!
//! Extends dock-to-desk scoring with a kinematic oncoming AGV: yield while the
//! shared segment is occupied, then finish delivery. `--smoke` proves success
//! and an ignore-yield collision Failure Capsule contract.

use rne_ai::{
    office_agv_shared_aisle_task_spec, run_behavior_scenarios, BehaviorContractStatus,
    BehaviorSeedStatus, OfficeAgvSharedAisleFault, OfficeAgvSharedAisleScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let fail_ignore_yield = std::env::args().any(|argument| argument == "--fail-ignore-yield");

    office_agv_shared_aisle_task_spec(2_000)
        .validate()
        .expect("office AGV shared-aisle TaskSpec");

    if !fail_ignore_yield {
        let success = run_behavior_scenarios(
            "office_agv_shared_aisle_success",
            [1],
            OfficeAgvSharedAisleScenario::success,
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

    if smoke || fail_ignore_yield {
        let failure = run_behavior_scenarios("office_agv_shared_aisle_ignore_yield", [1], |seed| {
            OfficeAgvSharedAisleScenario::new(seed, OfficeAgvSharedAisleFault::IgnoreYield)
        });
        let contact = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_other_agv_contact")
            .expect("other AGV contact contract");
        println!(
            "ignore-yield: status={:?} contact={:?} steps={}",
            failure.seeds[0].status, contact.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || contact.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
