use rne_ai::TaskSpec;
use rne_hardware_gateway::simulator::conformance::{
    run_simulator_adapter_conformance, SimulatorAdapterConformanceConfig,
    SimulatorAdapterConformanceReport,
};
use rne_hardware_gateway::simulator::SimulatorRuntimeManifest;
use rne_hardware_gateway::{GatewayConfig, HardwareGateway};
use sha2::{Digest, Sha256};
use std::ffi::OsString;
use std::path::{Path, PathBuf};

fn fixture(relative: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/simulator")
        .join(relative)
}

fn task_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets/tasks/diff_drive_goal.task.json")
}

fn gazebo_fixture(relative: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../simulator/rne_gazebo_harmonic")
        .join(relative)
}

fn config() -> SimulatorAdapterConformanceConfig {
    let task_path = task_path();
    let bytes = std::fs::read(&task_path).unwrap();
    let task: TaskSpec = serde_json::from_slice(&bytes).unwrap();
    let gateway = HardwareGateway::new(task.clone(), GatewayConfig::default()).unwrap();
    let task_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-simulator-mock-adapter"));
    let mut config = SimulatorAdapterConformanceConfig::new(&program, fixture("runtime.json"));
    config.arguments = [
        "--simulator-id".to_string(),
        "gazebo_sim_fixture".to_string(),
        "--simulator-version".to_string(),
        "8.9.0".to_string(),
        "--task-id".to_string(),
        task.task_id,
        "--task-sha256".to_string(),
        task_sha256,
        "--observation-width".to_string(),
        gateway.observation_width().to_string(),
        "--action-width".to_string(),
        gateway.action_width().to_string(),
        "--fixed-delta-ticks".to_string(),
        "16666667".to_string(),
    ]
    .into_iter()
    .map(OsString::from)
    .collect();
    config
}

#[test]
fn process_adapter_passes_complete_fixed_step_catalog() {
    let report = run_simulator_adapter_conformance(&task_path(), &config()).unwrap();
    assert!(report.passed(), "{report:#?}");
    assert_eq!(report.checks.len(), 10);
    assert_eq!(
        report.adapter.as_ref().unwrap().simulator_id,
        "gazebo_sim_fixture"
    );
    assert_eq!(report.subject.runtime_artifacts.len(), 3);
}

#[test]
fn fresh_process_conformance_reports_are_byte_identical() {
    let first = run_simulator_adapter_conformance(&task_path(), &config())
        .unwrap()
        .to_json_pretty()
        .unwrap();
    let second = run_simulator_adapter_conformance(&task_path(), &config())
        .unwrap()
        .to_json_pretty()
        .unwrap();
    assert_eq!(first, second);
}

#[test]
fn repository_gazebo_openarm_fixture_is_hash_bound_and_task_compatible() {
    let manifest_bytes = std::fs::read(gazebo_fixture("runtime.json")).unwrap();
    let manifest: SimulatorRuntimeManifest = serde_json::from_slice(&manifest_bytes).unwrap();
    manifest.validate().unwrap();
    assert_eq!(manifest.simulator_id, "gazebo_sim");
    assert_eq!(manifest.fixed_delta_ticks, 16_666_667);
    for artifact in &manifest.artifacts {
        let bytes = std::fs::read(gazebo_fixture(&artifact.file)).unwrap();
        assert_eq!(bytes.len() as u64, artifact.size_bytes);
        assert_eq!(format!("{:x}", Sha256::digest(&bytes)), artifact.sha256);
    }

    let task_bytes =
        std::fs::read(gazebo_fixture("openarm_right_joint_tracking.task.json")).unwrap();
    let task: TaskSpec = serde_json::from_slice(&task_bytes).unwrap();
    task.validate().unwrap();
    let gateway = HardwareGateway::new(task, GatewayConfig::default()).unwrap();
    assert_eq!(gateway.observation_width(), 18);
    assert_eq!(gateway.action_width(), 9);
    assert!(gazebo_fixture("rne_gazebo_harmonic_adapter.py").is_file());
    assert!(gazebo_fixture("adapter.manifest.toml").is_file());

    let report: SimulatorAdapterConformanceReport =
        serde_json::from_slice(&std::fs::read(gazebo_fixture("conformance.report.json")).unwrap())
            .unwrap();
    report.validate().unwrap();
    assert!(report.passed());
    for (file, expected) in [
        (
            "rne_gazebo_harmonic_adapter.py",
            report.subject.adapter_sha256.as_str(),
        ),
        (
            "openarm_right_joint_tracking.task.json",
            report.subject.task_sha256.as_str(),
        ),
        (
            "runtime.json",
            report.subject.runtime_manifest_sha256.as_str(),
        ),
    ] {
        let bytes = std::fs::read(gazebo_fixture(file)).unwrap();
        assert_eq!(format!("{:x}", Sha256::digest(bytes)), expected);
    }
}
