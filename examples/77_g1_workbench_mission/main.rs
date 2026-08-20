//! Headless Grove-G1 style workbench mission v3: park, arm window, Dex3 carry, place.
//!
//! Walks the dynamic G1 into the 0.5 m park, closes to the 0.2 m arm window,
//! then runs the pelvis-pinned Dex3 workcell with an explicit horizontal carry
//! before place. This is not Nav2 or MoveIt.

use rne_ai::{
    run_behavior_scenarios, unitree_g1_workbench_task_spec, BehaviorContractStatus,
    BehaviorSeedStatus, UnitreeG1WorkbenchFault, UnitreeG1WorkbenchMissionScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let skip_approach = std::env::args().any(|argument| argument == "--skip-approach");
    let drop_part = std::env::args().any(|argument| argument == "--drop-part");
    let skip_carry = std::env::args().any(|argument| argument == "--skip-carry");

    unitree_g1_workbench_task_spec(840)
        .validate()
        .expect("g1 workbench TaskSpec");

    if !skip_approach && !drop_part && !skip_carry {
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

    if smoke || drop_part {
        let failure = run_behavior_scenarios("g1_workbench_drop_part", [1], |seed| {
            UnitreeG1WorkbenchMissionScenario::new(seed, UnitreeG1WorkbenchFault::DropPart)
        });
        let park = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "park_within_0_5_m")
            .expect("park contract");
        let grasped = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "grasped")
            .expect("grasped contract");
        println!(
            "drop-part: status={:?} park={:?} grasped={:?} steps={}",
            failure.seeds[0].status, park.status, grasped.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || park.status != BehaviorContractStatus::Passed
            || grasped.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }

    if smoke || skip_carry {
        let failure = run_behavior_scenarios("g1_workbench_skip_carry", [1], |seed| {
            UnitreeG1WorkbenchMissionScenario::new(seed, UnitreeG1WorkbenchFault::SkipCarry)
        });
        let park = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "park_within_0_5_m")
            .expect("park contract");
        let grasped = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "grasped")
            .expect("grasped contract");
        let carry = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "carry_before_place")
            .expect("carry contract");
        println!(
            "skip-carry: status={:?} park={:?} grasped={:?} carry={:?} steps={}",
            failure.seeds[0].status,
            park.status,
            grasped.status,
            carry.status,
            failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || park.status != BehaviorContractStatus::Passed
            || grasped.status != BehaviorContractStatus::Passed
            || carry.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
