use rne_ai::TaskSpec;
use rne_hardware_lekiwi::{
    lekiwi_base_task_spec, lekiwi_reference_profile_v1, LeKiwiReferenceProfile,
};

const PROFILE_GOLDEN: &str = include_str!("fixtures/lekiwi-reference-profile-v1.json");
const TASK_GOLDEN: &str = include_str!("fixtures/lekiwi_so101_base.task.json");

#[test]
fn reference_profile_matches_committed_golden() {
    let profile = lekiwi_reference_profile_v1();
    profile.validate().unwrap();
    let actual = format!("{}\n", serde_json::to_string_pretty(&profile).unwrap());
    assert_eq!(actual, PROFILE_GOLDEN);

    let decoded: LeKiwiReferenceProfile = serde_json::from_str(PROFILE_GOLDEN).unwrap();
    decoded.validate().unwrap();
    assert_eq!(decoded, profile);
    assert_workspace_copy(
        "tests/golden/hardware/lekiwi-reference-profile-v1.json",
        PROFILE_GOLDEN,
    );
}

#[test]
fn base_task_matches_committed_golden() {
    let task = lekiwi_base_task_spec();
    task.validate().unwrap();
    let actual = format!("{}\n", serde_json::to_string_pretty(&task).unwrap());
    assert_eq!(actual, TASK_GOLDEN);

    let decoded: TaskSpec = serde_json::from_str(TASK_GOLDEN).unwrap();
    decoded.validate().unwrap();
    assert_eq!(decoded, task);
    assert_workspace_copy("assets/tasks/lekiwi_so101_base.task.json", TASK_GOLDEN);
}

fn assert_workspace_copy(relative_path: &str, expected: &str) {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../..")
        .join(relative_path);
    if path.exists() {
        assert_eq!(std::fs::read_to_string(path).unwrap(), expected);
    }
}
