use rne_hardware_gateway::conformance::{
    run_hardware_adapter_conformance, HardwareAdapterConformanceConfig,
};
use std::ffi::OsString;
use std::path::PathBuf;
use std::process::{Command, Stdio};

#[test]
fn python_mock_passes_the_published_external_adapter_catalog() {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let script = crate_dir.join("python/rne_hardware_lekiwi_device.py");
    let task = crate_dir.join("tests/fixtures/lekiwi_so101_base.task.json");
    let mut config = HardwareAdapterConformanceConfig::new(python_command());
    config.subject = script.clone();
    config.arguments = vec![script.into_os_string(), OsString::from("--mock")];
    config.response_timeout_ms = 5_000;
    config.allow_hil = true;

    let report = run_hardware_adapter_conformance(&task, &config)
        .expect("run LeKiwi Python adapter conformance");
    assert!(report.passed(), "checks: {:#?}", report.checks);
    assert_eq!(
        report.adapter.expect("adapter identity").device_id,
        "rne.lekiwi_so101.mock.v1"
    );
}

fn python_command() -> PathBuf {
    let candidates = std::env::var_os("PYTHON")
        .map(PathBuf::from)
        .into_iter()
        .chain([PathBuf::from("python3"), PathBuf::from("python")]);
    for candidate in candidates {
        let status = Command::new(&candidate)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        if status.is_ok_and(|status| status.success()) {
            return candidate;
        }
    }
    panic!("Python 3 is required to test LeKiwi adapter conformance");
}
