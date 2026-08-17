use serde_json::Value;
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_TEMP_ID: AtomicU64 = AtomicU64::new(0);

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new() -> Self {
        let base = std::env::temp_dir();
        for _ in 0..100 {
            let id = NEXT_TEMP_ID.fetch_add(1, Ordering::Relaxed);
            let path = base.join(format!(
                "rne-asset-failure-capsule-{}-{id}",
                std::process::id()
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("create temporary test directory: {error}"),
            }
        }
        panic!("could not allocate a unique temporary test directory")
    }

    fn path(&self) -> &std::path::Path {
        &self.0
    }
}

impl Drop for TestDirectory {
    fn drop(&mut self) {
        if self.0.starts_with(std::env::temp_dir()) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("workspace root")
}

#[test]
fn installed_cli_creates_and_verifies_a_failure_capsule() {
    let root = workspace_root();
    let output = TestDirectory::new();
    let installed_root = output.path().join("extracted-release");
    fs::create_dir(&installed_root).expect("create extracted release root");
    fs::copy(root.join("Cargo.lock"), installed_root.join("Cargo.lock"))
        .expect("stage release lockfile");
    let capsule = output.path().join("external-project-capsule");
    let replay = root.join("tests/golden/replays/behavior-replay-v1.json");
    let task = root.join("assets/tasks/diff_drive_goal.task.json");
    let cli = env!("CARGO_BIN_EXE_rne-asset");

    let create = Command::new(cli)
        .current_dir(&installed_root)
        .args([
            "failure-capsule",
            "create",
            "--replay",
            replay.to_str().expect("UTF-8 replay path"),
            "--evidence",
            task.to_str().expect("UTF-8 task path"),
            "--output",
            capsule.to_str().expect("UTF-8 capsule path"),
            "--backend",
            "external-test",
            "--backend-version",
            "1.0",
        ])
        .output()
        .expect("create Failure Capsule");
    assert!(
        create.status.success(),
        "create stderr: {}",
        String::from_utf8_lossy(&create.stderr)
    );

    let manifest: Value = serde_json::from_slice(
        &fs::read(capsule.join("capsule.json")).expect("read capsule manifest"),
    )
    .expect("parse capsule manifest");
    assert_eq!(manifest["kind"], "rne_failure_capsule");
    assert_eq!(manifest["backend"]["name"], "external-test");
    assert_eq!(
        manifest["build"]["git_commit"], "unknown",
        "an extracted archive must not invent source-control provenance"
    );
    assert_eq!(
        manifest["artifacts"].as_array().map(Vec::len),
        Some(2),
        "replay and TaskSpec must both be retained"
    );

    let verify = Command::new(cli)
        .current_dir(&installed_root)
        .args([
            "failure-capsule",
            "verify",
            capsule.to_str().expect("UTF-8 capsule path"),
        ])
        .output()
        .expect("verify Failure Capsule");
    assert!(
        verify.status.success(),
        "verify stderr: {}",
        String::from_utf8_lossy(&verify.stderr)
    );

    let repeated_create = Command::new(cli)
        .current_dir(&installed_root)
        .args([
            "failure-capsule",
            "create",
            "--replay",
            replay.to_str().expect("UTF-8 replay path"),
            "--output",
            capsule.to_str().expect("UTF-8 capsule path"),
        ])
        .output()
        .expect("repeat Failure Capsule creation");
    assert!(
        !repeated_create.status.success(),
        "existing evidence directory must never be overwritten"
    );
}
