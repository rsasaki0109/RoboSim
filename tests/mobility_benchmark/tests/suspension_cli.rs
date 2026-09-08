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
        ("suspension-influence", "suspension-influence-verify"),
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
    use rne_mobility_benchmark::suspension_acquisition::*;
    use rne_mobility_benchmark::suspension_identification::SuspensionDatasetSourceKind;
    use rne_mobility_benchmark::suspension_runs::SuspensionAcquiredRunRequest;
    use sha2::{Digest, Sha256};
    let file_ref = |name: &str| {
        let bytes = format!("test-only evidence: {name}");
        fs::write(root.join(name), &bytes).unwrap();
        SuspensionEvidenceFileRef {
            path: name.into(),
            size_bytes: bytes.len() as u64,
            sha256: format!("sha256:{:x}", Sha256::digest(bytes.as_bytes())),
        }
    };
    let mut runs = request;
    let mut manifests = Vec::new();
    for run in runs.training.iter_mut().chain(&mut runs.holdout) {
        run.dataset.source_kind = SuspensionDatasetSourceKind::RecordedBench;
        run.dataset.source_description =
            "test-only synthetic fixture, not a physical measurement".into();
        run.dataset.seal().unwrap();
        let calibration = file_ref("calibration.txt");
        let mut manifest = SuspensionPhysicalAcquisitionManifest {
            kind: SUSPENSION_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: SUSPENSION_ACQUISITION_SCHEMA_VERSION,
            capture_id: format!("test.capture.{}", run.acquisition_id),
            source_kind: SuspensionDatasetSourceKind::RecordedBench,
            dataset_content_digest: run.dataset.content_digest.clone(),
            vehicle_id: "test.vehicle".into(),
            strut_id: "front.left".into(),
            data_logger_id: "test.logger".into(),
            logger_software: "test-only".into(),
            clock_kind: SuspensionCaptureClockKind::SharedHardware,
            all_channels_synchronized: true,
            maximum_timestamp_uncertainty_s: 0.00001,
            raw_capture_format: SuspensionRawCaptureFormat::Csv,
            raw_capture: file_ref(&format!("raw-{}.csv", run.acquisition_id)),
            acquisition_procedure: file_ref("procedure.txt"),
            signals: [
                SuspensionSignalKind::Position,
                SuspensionSignalKind::Velocity,
                SuspensionSignalKind::Force,
            ]
            .into_iter()
            .map(|kind| SuspensionSignalEvidence {
                signal: kind,
                sensor_id: format!("test.{kind:?}").to_ascii_lowercase(),
                unit: match kind {
                    SuspensionSignalKind::Position => "m",
                    SuspensionSignalKind::Velocity => "m/s",
                    SuspensionSignalKind::Force => "N",
                }
                .into(),
                origin: SuspensionSignalOrigin::Measured,
                positive_along_strut_axis: true,
                sample_rate_hz: 200.0,
                resolution_si: 0.000001,
                expanded_uncertainty_si: 0.00001,
                calibration_kind: SuspensionCalibrationKind::Iso17025,
                calibration_artifact: calibration.clone(),
            })
            .collect(),
            rne_commit: "a".repeat(40),
            content_sha256: String::new(),
        };
        manifest.seal().unwrap();
        manifests.push(manifest);
    }
    let holdout = manifests.split_off(1);
    let acquired = SuspensionAcquiredRunRequest {
        runs,
        training: manifests,
        holdout,
    };
    let acquired_input = root.join("acquired.json");
    fs::write(&acquired_input, serde_json::to_vec(&acquired).unwrap()).unwrap();
    let output = root.join("acquired-evidence.json");
    let replay = root.join("acquired-replay.json");
    for (mode, source, destination) in [
        ("suspension-acquired", &acquired_input, &output),
        ("suspension-acquired-verify", &output, &replay),
    ] {
        let mut command = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"));
        command
            .args(["--backend", mode, "--input"])
            .arg(source)
            .arg("--evidence-root")
            .arg(&root)
            .arg("--output")
            .arg(destination);
        if mode == "suspension-acquired" {
            command.args(["--interval-tolerance-s", "0.000000001"]);
        }
        let result = command.output().unwrap();
        assert!(
            result.status.success(),
            "{mode}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
    }
    assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
    fs::write(root.join("raw-2.csv"), b"tampered").unwrap();
    let rejected_output = root.join("rejected.json");
    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "suspension-acquired-verify", "--input"])
        .arg(&output)
        .arg("--evidence-root")
        .arg(&root)
        .arg("--output")
        .arg(&rejected_output)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("file size mismatch"));
    assert!(!rejected_output.exists());
    fs::remove_dir_all(&root).unwrap();
}
