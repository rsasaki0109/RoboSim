//! Process-level SUMO co-simulation tests for the `rne-asset` binary.

use std::path::PathBuf;
use std::process::{Command, Stdio};

const BIN: &str = env!("CARGO_BIN_EXE_rne-asset");

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn sumo_available() -> bool {
    Command::new("sumo")
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

/// The `co-sim` command mirrors SUMO vehicles and verifies determinism.
#[test]
fn co_sim_command_reports_mirrored_vehicles_deterministically() {
    if !sumo_available() {
        eprintln!(
            "skipping co-sim test: `sumo` is not on PATH (install with `pip install eclipse-sumo`)"
        );
        return;
    }
    let root = manifest_dir().join("../..");
    let net = root.join("assets/networks/minimal_cross.net.xml");
    let routes = root.join("assets/networks/sumo_cross_flow.rou.xml");
    let output = Command::new(BIN)
        .arg("co-sim")
        .arg(&net)
        .arg("--routes")
        .arg(&routes)
        .arg("--steps")
        .arg("5")
        .arg("--determinism-check")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("run co-sim");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "rne-asset co-sim failed:\nstdout:\n{stdout}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        stdout.contains("final_actors=1"),
        "the fixture vehicle must be mirrored, got:\n{stdout}"
    );
    assert!(
        stdout.contains("determinism: identical co-simulation outcome"),
        "the co-simulation must be deterministic, got:\n{stdout}"
    );
}
