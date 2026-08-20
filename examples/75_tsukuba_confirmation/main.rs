//! Headless Tsukuba Challenge 2026 confirmation-run analog.
//!
//! Scores the official geometric checklist: two road-edge stops, no green-cone
//! contact, e-stop to zero speed, and no roadway entry. `--smoke` proves a
//! successful scripted run and a cone-hit Failure Capsule contract.

use rne_ai::{
    run_behavior_scenarios, tsukuba_confirmation_task_spec, BehaviorContractStatus,
    BehaviorSeedStatus, TsukubaConfirmationFault, TsukubaConfirmationScenario,
};

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let fail_cone = std::env::args().any(|argument| argument == "--fail-cone");

    tsukuba_confirmation_task_spec(1_200)
        .validate()
        .expect("tsukuba confirmation TaskSpec");

    if !fail_cone {
        let success = run_behavior_scenarios(
            "tsukuba_confirmation_success",
            [1],
            TsukubaConfirmationScenario::success,
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

    if smoke || fail_cone {
        let failure = run_behavior_scenarios("tsukuba_confirmation_hit_cone", [1], |seed| {
            TsukubaConfirmationScenario::new(seed, TsukubaConfirmationFault::HitCone)
        });
        let cone = failure.seeds[0]
            .contracts
            .iter()
            .find(|contract| contract.name == "no_cone_contact")
            .expect("cone contract");
        println!(
            "cone-hit: status={:?} cone={:?} steps={}",
            failure.seeds[0].status, cone.status, failure.seeds[0].steps
        );
        if failure.seeds[0].status != BehaviorSeedStatus::Failed
            || cone.status != BehaviorContractStatus::Failed
        {
            eprintln!("{failure:?}");
            std::process::exit(1);
        }
    }
}
