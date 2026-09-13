//! Process-level tire artifact coverage; the fixture is not physical evidence.

use rne_mobility_benchmark::identified_tire_backend::{
    IdentifiedTireBackendTrace, IdentifiedTireProfileEvidence,
};
use rne_mobility_benchmark::tire_acquisition::{
    RoadFrictionEvidenceKind, TireCalibrationKind, TireCaptureClockKind, TireEvidenceFileRef,
    TirePhysicalAcquisitionManifest, TirePhysicalQualificationEvidence, TireRawCaptureFormat,
    TireRunAcquisitionEvidence, TireSignalEvidence, TireSignalKind, TireSignalOrigin,
    TIRE_ACQUISITION_MANIFEST_KIND, TIRE_ACQUISITION_SCHEMA_VERSION,
};
use rne_mobility_benchmark::tire_identification::{
    synthetic_tire_identification_dataset, TireDatasetSourceKind, TireIdentificationDataset,
    TireIdentificationEvidence,
};
use rne_mobility_benchmark::tire_relaxation::{TireRelaxationDataset, TireRelaxationEvidence};
use sha2::{Digest, Sha256};
use std::path::Path;
use std::{fs, process::Command};

fn file_ref(root: &Path, name: &str, bytes: &[u8]) -> TireEvidenceFileRef {
    fs::write(root.join(name), bytes).unwrap();
    TireEvidenceFileRef {
        path: name.into(),
        size_bytes: bytes.len() as u64,
        sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
    }
}

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

#[test]
fn tire_relaxation_cli_emits_a_bound_non_physical_result() {
    let root = std::env::temp_dir().join(format!("rne-tire-relaxation-cli-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let dataset_path = root.join("transient.json");
    let evidence_path = root.join("transient-evidence.json");

    let fixture = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "tire-relaxation-fixture", "--output"])
        .arg(&dataset_path)
        .output()
        .unwrap();
    assert!(
        fixture.status.success(),
        "{}",
        String::from_utf8_lossy(&fixture.stderr)
    );
    let identify = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "tire-relaxation-identification", "--input"])
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

    let dataset: TireRelaxationDataset =
        serde_json::from_slice(&fs::read(&dataset_path).unwrap()).unwrap();
    let evidence: TireRelaxationEvidence =
        serde_json::from_slice(&fs::read(&evidence_path).unwrap()).unwrap();
    evidence.validate(&dataset).unwrap();
    assert!(!evidence.recorded_source_claim);
    assert!((evidence.result.relaxation_length_m - 0.35).abs() < 1.0e-12);
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn identified_tire_cli_binds_profile_to_rapier_plant() {
    let root = std::env::temp_dir().join(format!("rne-identified-tire-cli-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let profile_path = root.join("profile.json");
    let trace_path = root.join("rapier.json");

    let fixture = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "identified-tire-profile-fixture", "--output"])
        .arg(&profile_path)
        .output()
        .unwrap();
    assert!(
        fixture.status.success(),
        "{}",
        String::from_utf8_lossy(&fixture.stderr)
    );
    let execute = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "identified-tire-rapier", "--input"])
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

    let profile: IdentifiedTireProfileEvidence =
        serde_json::from_slice(&fs::read(profile_path).unwrap()).unwrap();
    let evidence: IdentifiedTireBackendTrace =
        serde_json::from_slice(&fs::read(trace_path).unwrap()).unwrap();
    profile.validate().unwrap();
    evidence.validate().unwrap();
    assert_eq!(evidence.trace.plant.tire, profile.tire_spec);
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn tire_acquisition_cli_verifies_physical_bindings_and_external_files() {
    let root =
        std::env::temp_dir().join(format!("rne-tire-acquisition-cli-{}", std::process::id()));
    fs::create_dir(&root).unwrap();
    let mut dataset = synthetic_tire_identification_dataset().unwrap();
    dataset.source_kind = TireDatasetSourceKind::RecordedBench;
    dataset.source_description = "process test only; not real physical evidence".into();
    dataset.seal().unwrap();
    let raw = file_ref(&root, "capture.mcap", b"process-test raw capture");
    let road = file_ref(&root, "road.txt", b"process-test road evidence");
    let calibration = file_ref(&root, "calibration.txt", b"process-test calibration");
    let procedure = file_ref(&root, "procedure.txt", b"process-test procedure");
    let signals = || {
        [
            TireSignalKind::LongitudinalSlipRatio,
            TireSignalKind::LateralSlipTangent,
            TireSignalKind::NormalLoad,
            TireSignalKind::LongitudinalForce,
            TireSignalKind::LateralForce,
        ]
        .into_iter()
        .map(|signal| TireSignalEvidence {
            signal,
            sensor_id: format!("sensor.{signal:?}"),
            unit: signal.unit().into(),
            convention: signal.convention().into(),
            origin: TireSignalOrigin::Measured,
            source_sample_rate_hz: if signal == TireSignalKind::LateralSlipTangent {
                400.0
            } else {
                100.0
            },
            converted_sample_rate_hz: 100.0,
            resolution_si: 1.0e-6,
            expanded_uncertainty_si: 1.0e-4,
            calibration_kind: TireCalibrationKind::Iso17025,
            calibration_artifact: calibration.clone(),
        })
        .collect()
    };
    let mut runs = dataset
        .training_runs
        .iter()
        .chain(&dataset.holdout_runs)
        .map(|run| TireRunAcquisitionEvidence {
            acquisition_id: run.acquisition_id,
            condition_id: run.condition_id,
            road_friction_scale: run.road_friction_scale,
            tire_id: "tire.test.serial_1".into(),
            clock_kind: TireCaptureClockKind::SharedHardware,
            all_channels_synchronized: true,
            maximum_timestamp_uncertainty_s: 0.0001,
            raw_capture_format: TireRawCaptureFormat::Mcap,
            raw_capture: raw.clone(),
            raw_segment_id: format!("segment.{}", run.acquisition_id),
            road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
            road_friction_artifact: road.clone(),
            signals: signals(),
        })
        .collect::<Vec<_>>();
    runs.sort_by_key(|run| run.acquisition_id);
    let mut manifest = TirePhysicalAcquisitionManifest {
        kind: TIRE_ACQUISITION_MANIFEST_KIND.into(),
        schema_version: TIRE_ACQUISITION_SCHEMA_VERSION,
        capture_id: "campaign.process.test".into(),
        source_kind: TireDatasetSourceKind::RecordedBench,
        dataset_content_sha256: dataset.content_sha256.clone(),
        vehicle_id: "rig.process.test".into(),
        data_logger_id: "logger.process.test".into(),
        logger_software: "process test logger 1.0".into(),
        acquisition_procedure: procedure,
        runs,
        rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
        content_sha256: String::new(),
    };
    manifest.seal().unwrap();
    let dataset_path = root.join("dataset.json");
    let manifest_path = root.join("manifest.json");
    let output_path = root.join("verified.json");
    fs::write(&dataset_path, serde_json::to_vec(&dataset).unwrap()).unwrap();
    fs::write(&manifest_path, serde_json::to_vec(&manifest).unwrap()).unwrap();

    let result = Command::new(env!("CARGO_BIN_EXE_rne-mobility-benchmark"))
        .args(["--backend", "tire-acquisition-verify", "--input"])
        .arg(&dataset_path)
        .arg("--acquisition-manifest")
        .arg(&manifest_path)
        .arg("--evidence-root")
        .arg(&root)
        .arg("--output")
        .arg(&output_path)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let verified: TirePhysicalQualificationEvidence =
        serde_json::from_slice(&fs::read(output_path).unwrap()).unwrap();
    assert!(verified.physical_measurement);
    verified.validate(&dataset, &manifest, &root).unwrap();
    fs::remove_dir_all(root).unwrap();
}
