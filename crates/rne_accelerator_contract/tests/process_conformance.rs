use rne_accelerator_contract::{
    run_accelerator_process_conformance, scaffold_accelerator_adapter,
    AcceleratorProcessConformanceConfig, AcceleratorProcessConformanceReport,
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

fn mock_program() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_rne-accelerator-protocol-mock"))
}

/// Content-addressed subject for mock launcher tests.
///
/// The mock debug binary is the process launcher, not the hashed adapter
/// artifact. Debug builds can exceed the 64 MiB subject limit (~280 MiB on CI).
fn mock_subject(transcript: &Path) -> PathBuf {
    transcript.to_path_buf()
}

#[test]
fn standalone_process_runner_passes_the_complete_installed_mock_lifecycle() {
    let (manifest, runtime, task, transcript) = contracts();
    let mut config = AcceleratorProcessConformanceConfig::new(mock_program());
    config.subject = mock_subject(&transcript);
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
    let (manifest, runtime, task, transcript) = contracts();
    let mut config = AcceleratorProcessConformanceConfig::new(mock_program());
    config.subject = mock_subject(&transcript);
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
    let mut config = AcceleratorProcessConformanceConfig::new(mock_program());
    config.subject = mock_subject(&transcript);
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
    let (manifest, runtime, task, transcript) = contracts();
    let mut config = AcceleratorProcessConformanceConfig::new(
        root().join("definitely-missing-accelerator-process"),
    );
    config.subject = mock_subject(&transcript);
    let report = run_accelerator_process_conformance(&manifest, &runtime, &task, &config).unwrap();
    report.validate().unwrap();
    assert_eq!(report.checks[0].status, "failed");
    assert!(report.checks[0].detail.contains("could not spawn"));
    assert!(report.frames.is_empty());
}

#[test]
fn generated_accelerator_scaffold_passes_the_standalone_process_runner() {
    let parent = std::env::temp_dir().join(format!(
        "rne-accelerator-scaffold-process-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&parent);
    let directory = scaffold_accelerator_adapter("scaffold_process", &parent).unwrap();
    let adapter = directory.join("adapter.py");
    let mut config = AcceleratorProcessConformanceConfig::new(python_command());
    config.arguments = vec![adapter.clone().into_os_string()];
    config.subject = adapter;
    let report = run_accelerator_process_conformance(
        &directory.join("accelerator.toml"),
        &directory.join("runtime.toml"),
        &directory.join("task.json"),
        &config,
    )
    .unwrap();
    assert!(report.passed(), "scaffold checks: {:#?}", report.checks);
    let _ = std::fs::remove_dir_all(&parent);
}

fn python_command() -> OsString {
    if let Some(value) = std::env::var_os("PYTHON") {
        return value;
    }
    for candidate in ["python3", "python"] {
        if std::process::Command::new(candidate)
            .arg("--version")
            .output()
            .is_ok_and(|output| output.status.success())
        {
            return OsString::from(candidate);
        }
    }
    panic!("accelerator scaffold conformance requires python3 or python");
}
