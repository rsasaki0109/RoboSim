//! Headless Grove-G1 style workbench mission: park, then Dex3 pick and place.
//!
//! Walks the dynamic G1 into the 0.5 m park radius, then runs the pelvis-pinned
//! Dex3 workcell. This is not Nav2 or MoveIt.

use rne_ai::{
    run_behavior_scenarios, unitree_g1_workbench_task_spec, BehaviorContractStatus,
    BehaviorSeedStatus, UnitreeG1WorkbenchFault, UnitreeG1WorkbenchMissionScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let skip_approach = std::env::args().any(|argument| argument == "--skip-approach");

    unitree_g1_workbench_task_spec(800)
        .validate()
        .expect("g1 workbench TaskSpec");

    if !skip_approach {
        let success = run_behavior_scenarios(
            "g1_workbench_success",
            [1],
            UnitreeG1WorkbenchMissionScenario::success,
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

    if smoke || skip_approach {
        let failure = run_behavior_scenarios("g1_workbench_skip_approach", [1], |seed| {
            UnitreeG1WorkbenchMissionScenario::new(seed, UnitreeG1WorkbenchFault::SkipApproach)
        });
        let park = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "park_within_0_5_m")
            .expect("park contract");
        println!(
            "skip-approach: status={:?} park={:?} steps={}",
            failure.seeds[0].status, park.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || park.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
