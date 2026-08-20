use rne_hardware_gateway::conformance::{
    run_hardware_adapter_conformance, HardwareAdapterConformanceConfig,
};
use std::ffi::OsString;
use std::path::PathBuf;

fn task_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../assets/tasks/diff_drive_goal.task.json")
}

fn bound_mock_config() -> HardwareAdapterConformanceConfig {
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-hardware-mock-device"));
    let mut config = HardwareAdapterConformanceConfig::new(program);
    config.arguments = [
        "--device-id",
        "rne-external-mock-v1",
        "--expected-task-id",
        "rne.diff_drive.sensor_goal.v1",
        "--observation-width",
        "9",
        "--action-width",
        "2",
    ]
    .into_iter()
    .map(OsString::from)
    .collect();
    config.allow_hil = true;
    config
}

#[test]
fn external_mock_process_passes_exact_repeatable_catalog() {
    let first = run_hardware_adapter_conformance(&task_path(), &bound_mock_config())
        .expect("run first conformance");
    let second = run_hardware_adapter_conformance(&task_path(), &bound_mock_config())
        .expect("run repeated conformance");
    assert!(first.passed(), "checks: {:#?}", first.checks);
    assert_eq!(first, second);
    assert_eq!(first.adapter.unwrap().device_id, "rne-external-mock-v1");
}

#[test]
fn adapter_without_a_fixed_task_binding_gets_a_failed_report() {
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-hardware-mock-device"));
    let mut config = HardwareAdapterConformanceConfig::new(program);
    config.allow_hil = true;
    let report = run_hardware_adapter_conformance(&task_path(), &config)
        .expect("semantic failure still produces a report");
    assert!(!report.passed());
    assert_eq!(report.status, "failed");
    let binding = report
        .checks
        .iter()
        .find(|check| check.id == "task_binding")
        .expect("task binding check");
    assert_eq!(binding.status, "failed");
}
