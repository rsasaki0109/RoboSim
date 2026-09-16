//! Process-level combined suspension-and-tire application coverage; fixtures are not physical evidence.

#[cfg(feature = "mujoco")]
use rne_mobility_benchmark::identified_suspension_tire::IdentifiedSuspensionTireEvidence;
use rne_mobility_benchmark::identified_suspension_tire::IdentifiedSuspensionTireTrace;
use rne_mobility_benchmark::identified_tire_backend::IdentifiedTireProfileEvidence;
use rne_mobility_benchmark::suspension_identification::SuspensionIdentificationDataset;
use std::path::Path;
use std::{fs, process::Command};

fn run_fixture(backend: &str, output: &Path) {
    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", backend, "--output"])
        .arg(output)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(output.is_file());
}

#[test]
fn identified_suspension_tire_rapier_cli_binds_both_identified_specs() {
    let root = std::env::temp_dir().join(format!(
        "rne-identified-suspension-tire-cli-{}",
        std::process::id()
    ));
    fs::create_dir(&root).unwrap();
    let dataset_path = root.join("suspension-dataset.json");
    let profile_path = root.join("tire-profile.json");
    let trace_path = root.join("rapier.json");

    run_fixture("suspension-identification-fixture", &dataset_path);
    run_fixture("identified-tire-profile-fixture", &profile_path);

    let execute = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "identified-suspension-tire-rapier", "--input"])
        .arg(&dataset_path)
        .arg("--tire-profile")
        .arg(&profile_path)
        .arg("--output")
        .arg(&trace_path)
        .output()
        .unwrap();
    assert!(
        execute.status.success(),
        "{}",
        String::from_utf8_lossy(&execute.stderr)
    );

    let dataset: SuspensionIdentificationDataset =
        serde_json::from_slice(&fs::read(&dataset_path).unwrap()).unwrap();
    let profile: IdentifiedTireProfileEvidence =
        serde_json::from_slice(&fs::read(&profile_path).unwrap()).unwrap();
    let evidence: IdentifiedSuspensionTireTrace =
        serde_json::from_slice(&fs::read(&trace_path).unwrap()).unwrap();
    evidence.validate().unwrap();
    dataset.validate().unwrap();
    profile.validate().unwrap();
    assert_eq!(evidence.applied_tire_spec, profile.tire_spec);
    assert_eq!(evidence.trace.wheel_plant_spec.tire, profile.tire_spec);
    assert!(!evidence.physical_measurement);
    fs::remove_dir_all(root).unwrap();
}

#[cfg(feature = "mujoco")]
#[test]
fn identified_suspension_tire_compare_cli_passes_cross_backend() {
    let root = std::env::temp_dir().join(format!(
        "rne-identified-suspension-tire-compare-cli-{}",
        std::process::id()
    ));
    fs::create_dir(&root).unwrap();
    let dataset_path = root.join("suspension-dataset.json");
    let profile_path = root.join("tire-profile.json");
    let compare_path = root.join("compare.json");

    run_fixture("suspension-identification-fixture", &dataset_path);
    run_fixture("identified-tire-profile-fixture", &profile_path);

    let compare = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "identified-suspension-tire-compare", "--input"])
        .arg(&dataset_path)
        .arg("--tire-profile")
        .arg(&profile_path)
        .arg("--output")
        .arg(&compare_path)
        .output()
        .unwrap();
    assert!(
        compare.status.success(),
        "{}",
        String::from_utf8_lossy(&compare.stderr)
    );

    let evidence: IdentifiedSuspensionTireEvidence =
        serde_json::from_slice(&fs::read(&compare_path).unwrap()).unwrap();
    evidence.validate().unwrap();
    assert!(evidence.passed, "{:#?}", evidence.road_comparison.metrics);
    assert!(!evidence.physical_measurement);
    fs::remove_dir_all(root).unwrap();
}
