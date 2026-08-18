use rne_accelerator_contract::{
    run_accelerator_process_conformance, AcceleratorProcessConformanceConfig,
    AcceleratorProcessConformanceReport,
};
use std::ffi::OsString;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

fn contracts() -> (PathBuf, PathBuf, PathBuf, PathBuf) {
    let root = root();
    (
        root.join("adapters/mjx/accelerator.toml"),
        root.join("adapters/mjx/runtime.toml"),
        root.join("adapters/mjx/fixtures/free-fall-task-spec-v1.json"),
        root.join("tests/golden/accelerators/protocol-transcript-v1.json"),
    )
}

#[test]
fn standalone_process_runner_passes_the_complete_installed_mock_lifecycle() {
    let (manifest, runtime, task, transcript) = contracts();
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-accelerator-protocol-mock"));
    let mut config = AcceleratorProcessConformanceConfig::new(&program);
    config.arguments = vec![OsString::from("--transcript"), transcript.into_os_string()];
    let report = run_accelerator_process_conformance(&manifest, &runtime, &task, &config).unwrap();
    report.validate().unwrap();
    assert!(report.passed());
    assert_eq!(report.frames.len(), 9);
    assert!(report.checks.iter().all(|check| check.status == "passed"));

    let json = report.to_json_pretty().unwrap();
    let decoded: AcceleratorProcessConformanceReport = serde_json::from_str(&json).unwrap();
    assert_eq!(decoded, report);
}

#[test]
fn timeout_kills_only_the_launched_child_and_emits_a_valid_failed_report() {
    let (manifest, runtime, task, _) = contracts();
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-accelerator-protocol-mock"));
    let mut config = AcceleratorProcessConformanceConfig::new(&program);
    config.arguments = vec![OsString::from("--stall")];
    config.response_timeout_ms = 25;
    let report = run_accelerator_process_conformance(&manifest, &runtime, &task, &config).unwrap();
    report.validate().unwrap();
    assert!(!report.passed());
    assert_eq!(report.checks[0].status, "passed");
    assert_eq!(report.checks[1].status, "failed");
    assert!(report.checks[1].detail.contains("exceeded 25 ms"));
    assert!(report.checks[2..]
        .iter()
        .all(|check| check.status == "not_run"));
    assert!(report.frames.is_empty());
}

#[test]
fn stdout_after_shutdown_is_rejected_without_blocking_the_reader_join() {
    let (manifest, runtime, task, transcript) = contracts();
    let program = PathBuf::from(env!("CARGO_BIN_EXE_rne-accelerator-protocol-mock"));
    let mut config = AcceleratorProcessConformanceConfig::new(&program);
    config.arguments = vec![
        OsString::from("--transcript"),
        transcript.into_os_string(),
        OsString::from("--extra-output"),
    ];
    config.response_timeout_ms = 100;
    let report = run_accelerator_process_conformance(&manifest, &runtime, &task, &config).unwrap();
    report.validate().unwrap();
    assert!(!report.passed());
    assert_eq!(report.frames.len(), 8);
    assert_eq!(report.checks[9].status, "failed");
    assert!(report.checks[9].detail.contains("unexpected stdout"));
    assert_eq!(report.checks[10].status, "not_run");
}

#[test]
fn missing_program_is_a_reported_spawn_failure_not_an_input_alias() {
    let (manifest, runtime, task, _) = contracts();
    let subject = PathBuf::from(env!("CARGO_BIN_EXE_rne-accelerator-protocol-mock"));
    let mut config = AcceleratorProcessConformanceConfig::new(
        root().join("definitely-missing-accelerator-process"),
    );
    config.subject = subject;
    let report = run_accelerator_process_conformance(&manifest, &runtime, &task, &config).unwrap();
    report.validate().unwrap();
    assert_eq!(report.checks[0].status, "failed");
    assert!(report.checks[0].detail.contains("could not spawn"));
    assert!(report.frames.is_empty());
}
