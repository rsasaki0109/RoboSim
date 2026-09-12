//! Process-level tire artifact coverage; the fixture is not physical evidence.

use rne_mobility_benchmark::tire_identification::{
    TireIdentificationDataset, TireIdentificationEvidence,
};
use std::{fs, process::Command};

#[test]
fn tire_cli_emits_a_bound_dataset_and_identification_result() {
    let root = std::env::temp_dir().join(format!("rne-tire-cli-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let dataset_path = root.join("dataset.json");
    let evidence_path = root.join("evidence.json");

    let fixture = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "tire-identification-fixture", "--output"])
        .arg(&dataset_path)
        .output()
        .unwrap();
    assert!(
        fixture.status.success(),
        "{}",
        String::from_utf8_lossy(&fixture.stderr)
    );
    let identify = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "tire-identification", "--input"])
        .arg(&dataset_path)
        .arg("--output")
        .arg(&evidence_path)
        .output()
        .unwrap();
    assert!(
        identify.status.success(),
        "{}",
        String::from_utf8_lossy(&identify.stderr)
    );

    let dataset: TireIdentificationDataset =
        serde_json::from_slice(&fs::read(&dataset_path).unwrap()).unwrap();
    let evidence: TireIdentificationEvidence =
        serde_json::from_slice(&fs::read(&evidence_path).unwrap()).unwrap();
    dataset.validate().unwrap();
    evidence.validate(&dataset).unwrap();
    assert_eq!(evidence.dataset_content_sha256, dataset.content_sha256);

    fs::remove_dir_all(root).unwrap();
}
