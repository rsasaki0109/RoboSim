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
    use rne_mobility_benchmark::suspension_uncertainty::*;
    let uncertainty_request = SuspensionUncertaintyRequest {
        errors: SuspensionAcquiredErrorRequest {
            kind: "rne_suspension_acquired_error_request".into(),
            schema_version: 1,
            acquisitions: acquired.clone(),
            model: SuspensionErrorModel {
                kind: "rne_suspension_additive_error_model".into(),
                schema_version: 1,
                seed: 42,
                draws: 8,
                factors: vec![SuspensionErrorFactor {
                    factor_id: 1,
                    distribution: SuspensionErrorDistribution::Normal,
                    scope: SuspensionErrorScope::SharedTraining,
                    position_loading_m: 0.0,
                    velocity_loading_m_s: 0.0,
                    force_loading_n: 1e8,
                }],
            },
            calibration: vec![SuspensionFactorCalibrationBinding {
                factor_id: 1,
                acquisition_id: 1,
                signal: SuspensionSignalKind::Force,
                calibration_artifact: acquired.training[0].signals[2].calibration_artifact.clone(),
                interpretation:
                    "Test-only deliberately inconsistent error budget; no physical qualification."
                        .into(),
            }],
        },
        interpretations: [
            SuspensionSignalKind::Position,
            SuspensionSignalKind::Velocity,
            SuspensionSignalKind::Force,
        ]
        .into_iter()
        .map(|signal| SuspensionCoverageInterpretation {
            acquisition_id: 1,
            signal,
            coverage_factor: 2.0,
            absolute_tolerance_si: 1e-12,
        })
        .collect(),
    };
    let uncertainty_input = root.join("uncertainty-input.json");
    let uncertainty_output = root.join("uncertainty-output.json");
    let uncertainty_replay = root.join("uncertainty-replay.json");
    fs::write(
        &uncertainty_input,
        serde_json::to_vec(&uncertainty_request).unwrap(),
    )
    .unwrap();
    for (mode, source, destination) in [
        (
            "suspension-uncertainty",
            &uncertainty_input,
            &uncertainty_output,
        ),
        (
            "suspension-uncertainty-verify",
            &uncertainty_output,
            &uncertainty_replay,
        ),
    ] {
        let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
            .args(["--backend", mode, "--input"])
            .arg(source)
            .arg("--evidence-root")
            .arg(&root)
            .arg("--output")
            .arg(destination)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{mode}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
    }
    let bytes = fs::read(&uncertainty_output).unwrap();
    for (option, value) in [("--interval-tolerance-s", "0.001"), ("--seed", "123")] {
        let destination = root.join(format!("unexpected-{}.json", &option[2..]));
        let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
            .args(["--backend", "suspension-uncertainty", "--input"])
            .arg(&uncertainty_input)
            .arg("--evidence-root")
            .arg(&root)
            .arg("--output")
            .arg(&destination)
            .args([option, value])
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert!(String::from_utf8_lossy(&result.stderr).contains("assumptions must be embedded"));
        assert!(!destination.exists());
    }
    assert_eq!(bytes, fs::read(&uncertainty_replay).unwrap());
    let uncertainty = decode_suspension_uncertainty(&bytes, &root).unwrap();
    assert!(uncertainty.budget.iter().all(|entry| !entry.matched));
    assert!(uncertainty
        .propagation
        .draws
        .iter()
        .any(|draw| draw.is_err()));
    let mut forged = uncertainty.clone();
    forged.propagation.draws.clear();
    let forged_input = root.join("uncertainty-forged.json");
    fs::write(&forged_input, serde_json::to_vec(&forged).unwrap()).unwrap();
    let rejected_uncertainty = root.join("uncertainty-rejected.json");
    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "suspension-uncertainty-verify", "--input"])
        .arg(&forged_input)
        .arg("--evidence-root")
        .arg(&root)
        .arg("--output")
        .arg(&rejected_uncertainty)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("replay mismatch"));
    assert!(!rejected_uncertainty.exists());
    {
        use rne_mobility_benchmark::suspension_derivative::{
            SuspensionDerivativeBinding, SuspensionDerivativeOperator,
            SuspensionDerivedErrorRequest,
        };
        let mut errors = uncertainty_request.errors.clone();
        let operator = SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;
        let dataset = &mut errors.acquisitions.runs.training[0].dataset;
        let times: Vec<_> = dataset.samples.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = dataset.samples.iter().map(|s| s.position_m).collect();
        for (sample, velocity) in dataset
            .samples
            .iter_mut()
            .zip(operator.reconstruct(&times, &positions).unwrap())
        {
            sample.velocity_m_s = velocity;
        }
        dataset.seal().unwrap();
        let manifest = &mut errors.acquisitions.training[0];
        manifest.dataset_content_digest = dataset.content_digest.clone();
        manifest.signals[1].origin = SuspensionSignalOrigin::Derived;
        manifest.signals[1].calibration_kind = SuspensionCalibrationKind::DerivedSignalProcedure;
        manifest.seal().unwrap();
        let bindings = vec![SuspensionDerivativeBinding {
            capture_id: manifest.capture_id.clone(),
            procedure: manifest.signals[1].calibration_artifact.clone(),
            operator,
            absolute_tolerance_m_s: 0.0,
        }];
        let derived = SuspensionDerivedErrorRequest { errors, bindings };
        {
            use rne_mobility_benchmark::suspension_affine_acquisition::*;
            use rne_mobility_benchmark::suspension_derivative::{
                SuspensionAffineCorrection, SuspensionAffineErrorModel, SuspensionAffineFactor,
            };
            use rne_mobility_benchmark::suspension_uncertainty::{
                SuspensionErrorDistribution, SuspensionErrorScope,
            };
            use SuspensionAffineDomain::{Force, Position, Timebase};
            let mut affine = SuspensionAcquiredAffineRequest {
                kind: "rne_suspension_acquired_affine_request".into(),
                schema_version: 1,
                acquisitions: derived.errors.acquisitions.clone(),
                derivatives: derived.bindings.clone(),
                model: SuspensionAffineErrorModel {
                    kind: "rne_suspension_affine_error_model".into(),
                    schema_version: 1,
                    seed: 42,
                    draws: 8,
                    nominal: vec![(
                        SuspensionAffineCorrection {
                            time_reference_s: 0.0,
                            time_scale: 1.0,
                            time_offset_s: 0.0,
                            position_scale: 1.0,
                            position_offset_m: 0.0,
                            force_scale: 1.0,
                            force_offset_n: 0.0,
                        },
                        operator,
                    )],
                    factors: vec![SuspensionAffineFactor {
                        factor_id: 7,
                        distribution: SuspensionErrorDistribution::Normal,
                        scope: SuspensionErrorScope::SharedTraining,
                        time_scale_loading: 100.0,
                        time_offset_loading_s: 0.0,
                        position_scale_loading: 0.0,
                        position_offset_loading_m: 0.0,
                        force_scale_loading: 0.0,
                        force_offset_loading_n: 0.0,
                    }],
                },
                calibration: vec![],
            };
            let manifest = &affine.acquisitions.training[0];
            let clock_bytes =
                fs::read(root.join(&manifest.signals[0].calibration_artifact.path)).unwrap();
            fs::write(root.join("affine-clock.txt"), &clock_bytes).unwrap();
            for (factor_id, domain) in [
                (None, Timebase),
                (None, Position),
                (None, Force),
                (Some(7), Timebase),
            ] {
                let signal = &manifest.signals[if domain == Force { 2 } else { 0 }];
                let mut artifact = signal.calibration_artifact.clone();
                if domain == Timebase {
                    artifact.path = "affine-clock.txt".into();
                }
                affine.calibration.push(SuspensionAffineCalibrationBinding {
                    factor_id,
                    domain,
                    acquisition_id: affine.acquisitions.runs.training[0].acquisition_id,
                    capture_id: manifest.capture_id.clone(),
                    instrument_id: if domain == Timebase {
                        "test-clock".into()
                    } else {
                        signal.sensor_id.clone()
                    },
                    calibration_artifact: artifact,
                    interpretation: "Synthetic test declaration, not calibration evidence.".into(),
                });
            }
            let input = root.join("affine-input.json");
            let output = root.join("affine-output.json");
            let replay = root.join("affine-replay.json");
            fs::write(&input, serde_json::to_vec(&affine).unwrap()).unwrap();
            let invoke = |mode: &str,
                          source: &std::path::Path,
                          destination: &std::path::Path,
                          extra: &[&str]| {
                Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
                    .args(["--backend", mode, "--input"])
                    .arg(source)
                    .arg("--evidence-root")
                    .arg(&root)
                    .arg("--output")
                    .arg(destination)
                    .args(extra)
                    .output()
                    .unwrap()
            };
            for (mode, source, destination) in [
                ("suspension-affine-errors", &input, &output),
                ("suspension-affine-errors-verify", &output, &replay),
            ] {
                let result = invoke(mode, source, destination, &[]);
                assert!(
                    result.status.success(),
                    "{mode}: {}",
                    String::from_utf8_lossy(&result.stderr)
                );
            }
            assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
            let evidence: SuspensionAffineEvidence =
                serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
            assert_eq!(evidence.propagation.draws.len(), 8);
            assert!(evidence.propagation.draws.iter().any(Result::is_err));
            let rejected = root.join("affine-rejected.json");
            for (mode, source) in [
                ("suspension-affine-errors", &input),
                ("suspension-affine-errors-verify", &output),
            ] {
                for extra in [["--seed", "123"], ["--interval-tolerance-s", "0.001"]] {
                    let result = invoke(mode, source, &rejected, &extra);
                    assert!(!result.status.success());
                    assert!(String::from_utf8_lossy(&result.stderr)
                        .contains("assumptions must be embedded"));
                    assert!(!rejected.exists());
                }
            }
            let mut forged = evidence;
            forged.propagation.draws.clear();
            let forged_path = root.join("affine-forged.json");
            fs::write(&forged_path, serde_json::to_vec(&forged).unwrap()).unwrap();
            let result = invoke(
                "suspension-affine-errors-verify",
                &forged_path,
                &rejected,
                &[],
            );
            assert!(!result.status.success());
            assert!(String::from_utf8_lossy(&result.stderr).contains("replay mismatch"));
            assert!(!rejected.exists());
            fs::write(root.join("affine-clock.txt"), b"tampered").unwrap();
            let result = invoke("suspension-affine-errors-verify", &output, &rejected, &[]);
            assert!(!result.status.success());
            assert!(!rejected.exists());
            fs::write(root.join("affine-clock.txt"), clock_bytes).unwrap();
            use rne_mobility_benchmark::suspension_sampling::{
                SuspensionTimingErrorModel, SuspensionTimingFactor, SuspensionTimingScope,
            };
            use rne_mobility_benchmark::suspension_timestamp_acquisition::*;
            let timestamp = SuspensionTimestampRequest {
                acquisitions: affine.acquisitions.clone(),
                derivatives: affine.derivatives.clone(),
                draws: 8,
                model: SuspensionTimingErrorModel {
                    kind: "rne_suspension_timing_error_model".into(),
                    schema_version: 1,
                    seed: 42,
                    factors: vec![SuspensionTimingFactor {
                        factor_id: 7,
                        distribution: SuspensionErrorDistribution::Normal,
                        scope: SuspensionTimingScope::IndependentSamples,
                        physical_loading_s: 0.0,
                        reported_loading_s: 10.0,
                    }],
                },
                clocks: affine
                    .calibration
                    .iter()
                    .filter(|b| b.domain == Timebase)
                    .cloned()
                    .collect(),
            };
            let input = root.join("timestamp-input.json");
            {
                use rne_mobility_benchmark::suspension_robust::*;
                let request = SuspensionAcquiredHuberRequest {
                    acquisitions: affine.acquisitions.clone(),
                    robust_spec: SuspensionHuberSpec {
                        delta_n: 10.0,
                        maximum_iterations: 100,
                        prediction_tolerance_n: 1e-7,
                    },
                };
                let source = root.join("huber-input.json");
                let output = root.join("huber-output.json");
                let replay = root.join("huber-replay.json");
                let rejected = root.join("huber-rejected.json");
                fs::write(&source, serde_json::to_vec(&request).unwrap()).unwrap();
                for (mode, input, destination) in [
                    ("suspension-huber", &source, &output),
                    ("suspension-huber-verify", &output, &replay),
                ] {
                    let result = invoke(mode, input, destination, &[]);
                    assert!(
                        result.status.success(),
                        "{}",
                        String::from_utf8_lossy(&result.stderr)
                    );
                    let result = invoke(mode, input, &rejected, &["--seed", "123"]);
                    assert!(!result.status.success());
                    assert!(String::from_utf8_lossy(&result.stderr)
                        .contains("assumptions must be embedded"));
                    assert!(!rejected.exists());
                }
                assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
                let mut forged: SuspensionAcquiredHuberEvidence =
                    serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
                forged.evaluation.iterate.weights.clear();
                let forged_path = root.join("huber-forged.json");
                fs::write(&forged_path, serde_json::to_vec(&forged).unwrap()).unwrap();
                assert!(
                    !invoke("suspension-huber-verify", &forged_path, &rejected, &[])
                        .status
                        .success()
                );
                assert!(!rejected.exists());
            }
            let output = root.join("timestamp-output.json");
            let replay = root.join("timestamp-replay.json");
            fs::write(&input, serde_json::to_vec(&timestamp).unwrap()).unwrap();
            for (mode, source, destination) in [
                ("suspension-timestamp-errors", &input, &output),
                ("suspension-timestamp-errors-verify", &output, &replay),
            ] {
                let result = invoke(mode, source, destination, &[]);
                assert!(
                    result.status.success(),
                    "{mode}: {}",
                    String::from_utf8_lossy(&result.stderr)
                );
            }
            assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
            let evidence: SuspensionTimestampEvidence =
                serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
            assert_eq!(evidence.propagation.draws.len(), 8);
            assert!(evidence.propagation.draws.iter().all(Result::is_err));
            let rejected = root.join("timestamp-rejected.json");
            for (mode, source) in [
                ("suspension-timestamp-errors", &input),
                ("suspension-timestamp-errors-verify", &output),
            ] {
                for extra in [["--seed", "123"], ["--interval-tolerance-s", "0.001"]] {
                    let result = invoke(mode, source, &rejected, &extra);
                    assert!(!result.status.success());
                    assert!(String::from_utf8_lossy(&result.stderr)
                        .contains("assumptions must be embedded"));
                    assert!(!rejected.exists());
                }
            }
            let mut forged = evidence;
            forged.propagation.draws.clear();
            let forged_path = root.join("timestamp-forged.json");
            fs::write(&forged_path, serde_json::to_vec(&forged).unwrap()).unwrap();
            let result = invoke(
                "suspension-timestamp-errors-verify",
                &forged_path,
                &rejected,
                &[],
            );
            assert!(!result.status.success());
            assert!(String::from_utf8_lossy(&result.stderr).contains("replay mismatch"));
            assert!(!rejected.exists());
            let clock_bytes = fs::read(root.join("affine-clock.txt")).unwrap();
            fs::write(root.join("affine-clock.txt"), b"tampered").unwrap();
            let result = invoke(
                "suspension-timestamp-errors-verify",
                &output,
                &rejected,
                &[],
            );
            assert!(!result.status.success());
            assert!(!rejected.exists());
            fs::write(root.join("affine-clock.txt"), clock_bytes).unwrap();
        }
        let input = root.join("derived-input.json");
        let output = root.join("derived-output.json");
        let replay = root.join("derived-replay.json");
        fs::write(&input, serde_json::to_vec(&derived).unwrap()).unwrap();
        for (mode, source, destination) in [
            ("suspension-derived-errors", &input, &output),
            ("suspension-derived-errors-verify", &output, &replay),
        ] {
            let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
                .args(["--backend", mode, "--input"])
                .arg(source)
                .arg("--evidence-root")
                .arg(&root)
                .arg("--output")
                .arg(destination)
                .output()
                .unwrap();
            assert!(
                result.status.success(),
                "{mode}: {}",
                String::from_utf8_lossy(&result.stderr)
            );
        }
        assert_eq!(fs::read(&output).unwrap(), fs::read(&replay).unwrap());
        let rejected_options = root.join("derived-rejected-options.json");
        for (mode, source) in [
            ("suspension-derived-errors", &input),
            ("suspension-derived-errors-verify", &output),
        ] {
            for (option, value) in [("--seed", "123"), ("--interval-tolerance-s", "0.001")] {
                let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
                    .args(["--backend", mode, "--input"])
                    .arg(source)
                    .arg("--evidence-root")
                    .arg(&root)
                    .arg("--output")
                    .arg(&rejected_options)
                    .args([option, value])
                    .output()
                    .unwrap();
                assert!(!result.status.success());
                assert!(String::from_utf8_lossy(&result.stderr)
                    .contains("assumptions must be embedded"));
                assert!(!rejected_options.exists());
            }
        }
        let mut forged: serde_json::Value =
            serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
        forged["propagation"]["draws"] = serde_json::json!([]);
        let bad_input = root.join("derived-forged.json");
        let rejected = root.join("derived-rejected.json");
        fs::write(&bad_input, serde_json::to_vec(&forged).unwrap()).unwrap();
        let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
            .args(["--backend", "suspension-derived-errors-verify", "--input"])
            .arg(&bad_input)
            .arg("--evidence-root")
            .arg(&root)
            .arg("--output")
            .arg(&rejected)
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert!(String::from_utf8_lossy(&result.stderr).contains("replay mismatch"));
        assert!(!rejected.exists());
    }
    fs::write(root.join("raw-2.csv"), b"tampered").unwrap();
    let rejected_derived = root.join("derived-rejected-source.json");
    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "suspension-derived-errors-verify", "--input"])
        .arg(root.join("derived-output.json"))
        .arg("--evidence-root")
        .arg(&root)
        .arg("--output")
        .arg(&rejected_derived)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("file size mismatch"));
    assert!(!rejected_derived.exists());
    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "suspension-uncertainty-verify", "--input"])
        .arg(&uncertainty_output)
        .arg("--evidence-root")
        .arg(&root)
        .arg("--output")
        .arg(&rejected_uncertainty)
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("file size mismatch"));
    assert!(!rejected_uncertainty.exists());
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
