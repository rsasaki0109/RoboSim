//! Process-level runner-control tests for the `rne-asset` binary.

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

const BIN: &str = env!("CARGO_BIN_EXE_rne-asset");

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn run_control_script(script: &[u8], replay: &std::path::Path) -> std::process::Output {
    let _ = std::fs::remove_file(replay);
    let mut child = Command::new(BIN)
        .arg("run")
        .arg(manifest_dir().join("../../assets/runs/mm_minimal_joint_velocity.rne.run.toml"))
        .arg("--control-stdin")
        .arg("--replay-out")
        .arg(replay)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn rne-asset");

    {
        let mut stdin = child.stdin.take().expect("child stdin");
        stdin.write_all(script).expect("write control script");
    }

    let output = child.wait_with_output().expect("wait for rne-asset");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "rne-asset failed:\nstdout:\n{stdout}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    output
}

/// `pause`, `step 3`, `quit` piped on stdin must drive exactly three frames and
/// write a replay artifact for the partial episode.
#[test]
fn control_stdin_pause_step_quit_produces_exactly_the_requested_frames() {
    let replay = manifest_dir().join("../../target/runs/control_step_3.rne-replay");
    run_control_script(b"pause\nstep 3\nquit\n", &replay);

    let artifact =
        rne_log::ReplayArtifact::read_json(&replay).expect("read control replay artifact");
    assert_eq!(
        artifact.frames.len(),
        3,
        "pause + step 3 + quit must produce exactly 3 frames"
    );
    assert_eq!(artifact.clock.steps, 3);
}

/// `quit` alone must end a paused run without advancing past the current frame.
#[test]
fn control_stdin_quit_without_steps_writes_an_empty_replay() {
    let replay = manifest_dir().join("../../target/runs/control_quit.rne-replay");
    run_control_script(b"pause\nquit\n", &replay);

    let artifact =
        rne_log::ReplayArtifact::read_json(&replay).expect("read control replay artifact");
    assert_eq!(
        artifact.frames.len(),
        0,
        "pause + quit without stepping must run no frames"
    );
}

/// `reset` must restart the episode from the initial conditions before the
/// remaining scripted steps run.
#[test]
fn control_stdin_reset_restarts_the_episode() {
    let replay = manifest_dir().join("../../target/runs/control_reset.rne-replay");
    run_control_script(b"pause\nstep 5\nreset\nstep 4\nquit\n", &replay);

    let artifact =
        rne_log::ReplayArtifact::read_json(&replay).expect("read control replay artifact");
    assert_eq!(
        artifact.frames.len(),
        4,
        "only the post-reset steps are reported"
    );
    assert_eq!(artifact.clock.steps, 4);
}
