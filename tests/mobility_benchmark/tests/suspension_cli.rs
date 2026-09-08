//! Process-level regression coverage; synthetic inputs are not physical evidence.

use rne_mobility_benchmark::suspension_identification::{
    suspension_identification_spec, synthetic_suspension_identification_dataset,
};
use rne_mobility_benchmark::suspension_runs::{SuspensionRunInput, SuspensionRunRequest};
use std::{fs, process::Command};

#[test]
fn whole_run_cli_generates_and_reverifies_all_diagnostic_envelopes() {
    let root = std::env::temp_dir().join(format!("rne-suspension-cli-{}", std::process::id()));
    // Exclusive creation: never overwrite or remove another run's directory.
    fs::create_dir(&root).unwrap();
    let first = synthetic_suspension_identification_dataset().unwrap();
    let mut second = first.clone();
    second.dataset_id = "synthetic.cli.holdout".into();
    second.samples[0].force_n += 1.0;
    second.seal().unwrap();
    let request = SuspensionRunRequest {
        kind: "rne_suspension_run_request".into(),
        schema_version: 1,
        spec: suspension_identification_spec(),
        training: vec![SuspensionRunInput {
            acquisition_id: 1,
            dataset: first,
        }],
        holdout: vec![SuspensionRunInput {
            acquisition_id: 2,
            dataset: second,
        }],
    };
    let input = root.join("input.json");
    fs::write(&input, serde_json::to_vec(&request).unwrap()).unwrap();
    for (generate, verify) in [
        ("suspension-run-identification", "suspension-run-verify"),
        ("suspension-excitation", "suspension-excitation-verify"),
        ("suspension-timing", "suspension-timing-verify"),
    ] {
        let output = root.join(format!("{generate}.json"));
        let replay = root.join(format!("{verify}.json"));
        let mut command = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"));
        command
            .args(["--backend", generate, "--input"])
            .arg(&input)
            .arg("--output")
            .arg(&output);
        if generate == "suspension-timing" {
            command.args(["--interval-tolerance-s", "0.000000001"]);
        }
        let result = command.output().unwrap();
        assert!(
            result.status.success(),
            "{generate}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
        let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
            .args(["--backend", verify, "--input"])
            .arg(&output)
            .arg("--output")
            .arg(&replay)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{verify}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
        assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
    }
    fs::remove_dir_all(&root).unwrap();
}
