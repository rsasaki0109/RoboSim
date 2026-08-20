//! Process-level tests for standalone controller-plugin conformance.

use serde_json::Value;
use std::path::{Path, PathBuf};
use std::process::Command;

const BIN: &str = env!("CARGO_BIN_EXE_rne-asset");

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn target_dir() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| workspace_root().join("target"))
}

fn plugin_library_name() -> &'static str {
    if cfg!(target_os = "windows") {
        "rne_plugin_example_velocity_servo.dll"
    } else if cfg!(target_os = "macos") {
        "librne_plugin_example_velocity_servo.dylib"
    } else {
        "librne_plugin_example_velocity_servo.so"
    }
}

fn build_plugin_library() -> PathBuf {
    let status = Command::new("cargo")
        .args([
            "build",
            "--locked",
            "-p",
            "rne_plugin_example_velocity_servo",
        ])
        .current_dir(workspace_root())
        .status()
        .expect("build reference controller plugin");
    assert!(status.success(), "reference plugin build failed");
    let library = target_dir().join("debug").join(plugin_library_name());
    assert!(library.is_file(), "missing plugin at {}", library.display());
    library
}

fn run_check(library: &Path, manifest: &Path, report: &Path) -> std::process::Output {
    Command::new(BIN)
        .args(["plugin", "check", "--library"])
        .arg(library)
        .arg("--manifest")
        .arg(manifest)
        .arg("--output")
        .arg(report)
        .output()
        .expect("run controller-plugin conformance")
}

fn read_report(path: &Path) -> Value {
    serde_json::from_slice(&std::fs::read(path).expect("read conformance report"))
        .expect("parse conformance report")
}

#[test]
fn plugin_check_persists_pass_and_semantic_failure_reports() {
    let root = workspace_root();
    let library = build_plugin_library();
    let report_root = std::env::temp_dir().join(format!("rne-plugin-cli-{}", std::process::id()));
    std::fs::create_dir_all(&report_root).expect("create report directory");
    let passed_report = report_root.join("passed.json");
    let failed_report = report_root.join("failed.json");

    let passed = run_check(
        &library,
        &root.join("crates/rne_plugin_example_velocity_servo/rne-plugin.json"),
        &passed_report,
    );
    assert!(
        passed.status.success(),
        "passing CLI failed: {}",
        String::from_utf8_lossy(&passed.stderr)
    );
    assert_eq!(read_report(&passed_report)["status"], "passed");

    let failed = run_check(
        &library,
        &root.join("crates/rne_plugin_legacy_v2_fixture/rne-plugin.json"),
        &failed_report,
    );
    assert!(!failed.status.success(), "mismatched manifest must fail");
    let report = read_report(&failed_report);
    assert_eq!(report["status"], "failed");
    assert_eq!(report["checks"][0]["id"], "manifest_identity");
    assert_eq!(report["checks"][0]["status"], "failed");

    std::fs::remove_dir_all(report_root).expect("remove report directory");
}
