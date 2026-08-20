//! Process-level runner-control tests for the `rne-asset` binary.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};

use rne_core::control::ControlCommand;
use rne_data::transport::{
    decode_image_depth, decode_image_rgb8, decode_lidar_point_cloud, encode_control_command,
    ClientHello, ControlAck, ServerHello, StatusMessage, TransportCapabilities, TransportFrame,
    TransportMessageKind, TRANSPORT_MAX_PAYLOAD_BYTES,
};

const BIN: &str = env!("CARGO_BIN_EXE_rne-asset");

struct KillOnDropChild(Child);

impl KillOnDropChild {
    fn wait(&mut self) -> std::io::Result<std::process::ExitStatus> {
        self.0.wait()
    }

    fn try_wait(&mut self) -> std::io::Result<Option<std::process::ExitStatus>> {
        self.0.try_wait()
    }

    fn wait_bounded(&mut self) -> std::io::Result<std::process::ExitStatus> {
        for _ in 0..300 {
            if let Some(status) = self.try_wait()? {
                return Ok(status);
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        Err(std::io::Error::new(
            std::io::ErrorKind::TimedOut,
            "runner did not exit within 15 seconds",
        ))
    }
}

impl Drop for KillOnDropChild {
    fn drop(&mut self) {
        if self.0.try_wait().ok().flatten().is_none() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

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

/// Spawns the runner with a TCP control endpoint and returns the child, its
/// stdout reader, and the bound control port.
fn spawn_tcp_control(
    replay: &std::path::Path,
) -> (Child, BufReader<std::process::ChildStdout>, u16) {
    spawn_tcp_control_for(
        manifest_dir().join("../../assets/runs/mm_minimal_joint_velocity.rne.run.toml"),
        replay,
    )
}

fn spawn_tcp_control_for(
    manifest: PathBuf,
    replay: &std::path::Path,
) -> (Child, BufReader<std::process::ChildStdout>, u16) {
    spawn_tcp_control_for_options(manifest, replay, false)
}

fn spawn_tcp_control_for_options(
    manifest: PathBuf,
    replay: &std::path::Path,
    control_camera_full_resolution: bool,
) -> (Child, BufReader<std::process::ChildStdout>, u16) {
    let _ = std::fs::remove_file(replay);
    let mut command = Command::new(BIN);
    command
        .arg("run")
        .arg(manifest)
        .arg("--control-port")
        .arg("0")
        .arg("--replay-out")
        .arg(replay);
    if control_camera_full_resolution {
        command.arg("--control-camera-full-resolution");
    }
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn rne-asset");

    let stdout = child.stdout.take().expect("child stdout");
    let mut stdout = BufReader::new(stdout);
    let mut port = None;
    let mut line = String::new();
    while port.is_none() {
        line.clear();
        if stdout.read_line(&mut line).expect("read child stdout") == 0 {
            break;
        }
        if let Some(rest) = line.split_once("127.0.0.1:").map(|(_, rest)| rest) {
            let digits: String = rest
                .chars()
                .take_while(|character| character.is_ascii_digit())
                .collect();
            port = digits.parse::<u16>().ok();
        }
    }
    (
        child,
        stdout,
        port.expect("control port from runner stdout"),
    )
}

fn spawn_binary_frontend_for(
    manifest: PathBuf,
    replay: &std::path::Path,
) -> (KillOnDropChild, BufReader<std::process::ChildStdout>, u16) {
    let _ = std::fs::remove_file(replay);
    let mut child = Command::new(BIN)
        .arg("run")
        .arg(manifest)
        .arg("--frontend-port")
        .arg("0")
        .arg("--replay-out")
        .arg(replay)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn binary frontend runner");
    let stdout = child.stdout.take().expect("binary frontend stdout");
    let mut stdout = BufReader::new(stdout);
    let mut port = None;
    let mut line = String::new();
    while port.is_none() {
        line.clear();
        if stdout.read_line(&mut line).expect("read frontend stdout") == 0 {
            break;
        }
        if let Some(rest) = line.split_once("127.0.0.1:").map(|(_, rest)| rest) {
            let digits: String = rest
                .chars()
                .take_while(|character| character.is_ascii_digit())
                .collect();
            port = digits.parse::<u16>().ok();
        }
    }
    (
        KillOnDropChild(child),
        stdout,
        port.expect("binary frontend port from runner stdout"),
    )
}

fn connect_binary_frontend(port: u16) -> (TcpStream, u64, ServerHello) {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect binary frontend");
    stream
        .set_read_timeout(Some(std::time::Duration::from_secs(10)))
        .expect("set binary frontend read timeout");
    let hello = ClientHello {
        min_protocol_major: 1,
        max_protocol_major: 1,
        capabilities: TransportCapabilities::ALL_V1,
        required_capabilities: TransportCapabilities::CONTROL.union(TransportCapabilities::STATUS),
        max_payload_bytes: TRANSPORT_MAX_PAYLOAD_BYTES as u32,
        queue_frame_limit: 16,
        queue_byte_limit: 64 * 1024 * 1024,
        resume_after_sequence: None,
    };
    TransportFrame::new(
        TransportMessageKind::ClientHello,
        1,
        0,
        hello.encode_payload(),
    )
    .write_to(&mut stream)
    .expect("write binary ClientHello");
    let response = TransportFrame::read_from(&mut stream, TRANSPORT_MAX_PAYLOAD_BYTES)
        .expect("read binary ServerHello")
        .expect("binary ServerHello frame");
    assert_eq!(response.kind, TransportMessageKind::ServerHello);
    let server = ServerHello::decode_payload(&response.payload).expect("decode ServerHello");
    (stream, response.session_id, server)
}

fn write_binary_command(
    stream: &mut TcpStream,
    session_id: u64,
    sequence: u64,
    command: ControlCommand,
) {
    TransportFrame::new(
        TransportMessageKind::ControlCommand,
        sequence,
        session_id,
        encode_control_command(command),
    )
    .write_to(stream)
    .expect("write binary control command");
}

#[test]
fn binary_frontend_streams_lossless_rgbd_with_sim_timestamps() {
    let manifest = manifest_dir().join("../../assets/runs/mm_minimal_joint_velocity.rne.run.toml");
    let replay = manifest_dir().join("../../target/runs/frontend_binary_rgbd.rne-replay");
    let (mut child, _stdout, port) = spawn_binary_frontend_for(manifest, &replay);
    let (mut stream, session_id, server) = connect_binary_frontend(port);
    assert!(server
        .negotiated
        .capabilities
        .contains(TransportCapabilities::IMAGE_RGB8));
    assert!(server
        .negotiated
        .capabilities
        .contains(TransportCapabilities::IMAGE_DEPTH_F32));
    write_binary_command(
        &mut stream,
        session_id,
        10,
        ControlCommand::Step { frames: 1 },
    );

    let mut ack = false;
    let mut status = false;
    let mut rgb = false;
    let mut depth = false;
    for _ in 0..16 {
        let frame = TransportFrame::read_from(&mut stream, TRANSPORT_MAX_PAYLOAD_BYTES)
            .expect("read RGB-D frontend frame")
            .unwrap_or_else(|| {
                panic!(
                    "RGB-D frontend closed early: ack={ack} status={status} rgb={rgb} depth={depth} child={:?}",
                    child.try_wait()
                )
            });
        match frame.kind {
            TransportMessageKind::ControlAck => {
                let value = ControlAck::decode_payload(&frame.payload).expect("decode ack");
                ack |= value.command_sequence == 10;
            }
            TransportMessageKind::Status => {
                let value = StatusMessage::decode_payload(&frame.payload).expect("decode status");
                assert_eq!(value.step, 1);
                assert!(value.sim_time_ticks > 0);
                let json = std::str::from_utf8(&value.snapshot_json).expect("status UTF-8");
                assert!(json.contains("\"sensors\""));
                assert!(!json.contains("rgba8_base64"));
                assert!(!json.contains("depth_f32_le_base64"));
                status = true;
            }
            TransportMessageKind::ImageRgb8 => {
                let (metadata, image) = decode_image_rgb8(&frame.payload).expect("decode RGB8");
                assert_eq!((image.width, image.height), (64, 48));
                assert_eq!(image.rgba8.len(), 64 * 48 * 4);
                assert!(metadata.available_ticks >= metadata.capture_ticks);
                rgb = true;
            }
            TransportMessageKind::ImageDepthF32 => {
                let (metadata, image) = decode_image_depth(&frame.payload).expect("decode depth");
                assert_eq!((image.width, image.height), (64, 48));
                assert_eq!(image.depth_m.len(), 64 * 48);
                assert!(image.depth_m.iter().all(|value| value.is_finite()));
                assert!(metadata.available_ticks >= metadata.capture_ticks);
                depth = true;
            }
            TransportMessageKind::Gap => {}
            other => panic!("unexpected RGB-D frontend frame: {other:?}"),
        }
        if ack && status && rgb && depth {
            break;
        }
    }
    assert!(
        ack && status && rgb && depth,
        "missing RGB-D frontend frames"
    );

    write_binary_command(&mut stream, session_id, 11, ControlCommand::Quit);
    assert!(
        child
            .wait()
            .expect("wait for RGB-D frontend runner")
            .success(),
        "RGB-D frontend runner must exit successfully"
    );
    let artifact = rne_log::ReplayArtifact::read_json(&replay).expect("read RGB-D replay");
    assert_eq!(artifact.frames.len(), 1);
}

#[test]
fn binary_frontend_streams_aligned_lidar_payload() {
    let manifest =
        manifest_dir().join("../../assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml");
    let replay = manifest_dir().join("../../target/runs/frontend_binary_lidar.rne-replay");
    let (mut child, _stdout, port) = spawn_binary_frontend_for(manifest, &replay);
    let (mut stream, session_id, _server) = connect_binary_frontend(port);
    write_binary_command(
        &mut stream,
        session_id,
        20,
        ControlCommand::Step { frames: 1 },
    );

    let mut lidar = None;
    for _ in 0..16 {
        let frame = TransportFrame::read_from(&mut stream, TRANSPORT_MAX_PAYLOAD_BYTES)
            .expect("read LiDAR frontend frame")
            .expect("LiDAR frontend frame");
        if frame.kind == TransportMessageKind::LidarPointCloud {
            lidar = Some(decode_lidar_point_cloud(&frame.payload).expect("decode LiDAR"));
            break;
        }
    }
    let (metadata, cloud) = lidar.expect("binary LiDAR payload");
    assert!(metadata.available_ticks >= metadata.capture_ticks);
    assert!(!cloud.points_m.is_empty());
    assert!(cloud.attributes_are_aligned());
    assert!(cloud.points_m.iter().all(|point| point.is_finite()));

    write_binary_command(&mut stream, session_id, 21, ControlCommand::Quit);
    assert!(
        child
            .wait()
            .expect("wait for LiDAR frontend runner")
            .success(),
        "LiDAR frontend runner must exit successfully"
    );
    let artifact = rne_log::ReplayArtifact::read_json(&replay).expect("read LiDAR replay");
    assert_eq!(artifact.frames.len(), 1);
}

#[test]
fn binary_frontend_unread_client_does_not_stall_simulation() {
    let manifest = manifest_dir().join("../../assets/runs/mm_minimal_joint_velocity.rne.run.toml");
    let replay = manifest_dir().join("../../target/runs/frontend_binary_slow_client.rne-replay");
    let (mut child, _stdout, port) = spawn_binary_frontend_for(manifest, &replay);
    let (mut stream, session_id, _server) = connect_binary_frontend(port);
    write_binary_command(&mut stream, session_id, 30, ControlCommand::Resume);

    // Deliberately retain the socket without reading the acknowledgement,
    // status, RGB, or depth frames. Socket I/O occurs off the simulation
    // thread, and both queue dimensions are bounded.
    let status = child
        .wait_bounded()
        .expect("slow frontend must not stall runner");
    assert!(status.success(), "runner must finish with unread frontend");
    let artifact = rne_log::ReplayArtifact::read_json(&replay).expect("read slow-client replay");
    assert_eq!(artifact.frames.len(), 120);
    drop(stream);
}

/// The opt-in full-resolution TCP path must carry the source RGB-D dimensions
/// and exact little-endian payload lengths through a real runner process.
#[test]
fn control_tcp_full_resolution_camera_and_depth_snapshot() {
    let manifest = manifest_dir().join("../../assets/runs/mm_minimal_joint_velocity.rne.run.toml");
    let replay = manifest_dir().join("../../target/runs/control_tcp_full_rgbd.rne-replay");
    let (mut child, _stdout, port) = spawn_tcp_control_for_options(manifest, &replay, true);

    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect RGB-D control");
    let mut reader = BufReader::new(stream.try_clone().expect("clone RGB-D control stream"));
    let mut ready = String::new();
    reader.read_line(&mut ready).expect("read RGB-D ready");
    assert_eq!(ready.trim(), "ready paused protocol=1");

    stream.write_all(b"step 1\n").expect("write RGB-D step");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read RGB-D ack");
    assert_eq!(ack.trim(), "ok paused");

    let mut status = String::new();
    reader.read_line(&mut status).expect("read RGB-D status");
    assert!(
        status.starts_with("status step=1 "),
        "unexpected status: {status}"
    );
    let snapshot = status
        .split_once(" snapshot=")
        .map(|(_, snapshot)| snapshot.trim())
        .expect("snapshot field");
    let value: serde_json::Value = serde_json::from_str(snapshot).expect("snapshot JSON");
    let camera = value["sensors"]
        .as_array()
        .expect("sensor array")
        .iter()
        .find(|stream| stream["kind"] == "camera")
        .and_then(|stream| stream.get("camera"))
        .expect("camera payload");
    assert_eq!(camera["width"], serde_json::json!(64));
    assert_eq!(camera["height"], serde_json::json!(48));
    assert_eq!(
        base64::decode(camera["rgba8_base64"].as_str().expect("RGB base64"))
            .expect("decode RGB")
            .len(),
        64 * 48 * 4
    );
    assert_eq!(camera["depth_width"], serde_json::json!(64));
    assert_eq!(camera["depth_height"], serde_json::json!(48));
    assert_eq!(
        base64::decode(
            camera["depth_f32_le_base64"]
                .as_str()
                .expect("depth base64")
        )
        .expect("decode depth")
        .len(),
        64 * 48 * 4
    );

    stream.write_all(b"quit\n").expect("write RGB-D quit");
    let mut quit_ack = String::new();
    match reader.read_line(&mut quit_ack) {
        Ok(_) => assert_eq!(quit_ack.trim(), "ok paused"),
        // Windows CI occasionally resets the control socket after the runner
        // has already accepted quit and begun teardown.
        Err(error)
            if matches!(
                error.kind(),
                std::io::ErrorKind::ConnectionReset | std::io::ErrorKind::UnexpectedEof
            ) => {}
        Err(error) => panic!("read RGB-D quit ack: {error}"),
    }
    assert!(
        child.wait().expect("wait for RGB-D runner").success(),
        "RGB-D runner must exit successfully"
    );
    let artifact = rne_log::ReplayArtifact::read_json(&replay).expect("read RGB-D replay");
    assert_eq!(artifact.frames.len(), 1);
}

/// `--control-port` serves pause/step/quit over TCP with live status replies and
/// drives exactly the requested frames.
#[test]
fn control_tcp_step_and_quit_produce_the_requested_frames() {
    let replay = manifest_dir().join("../../target/runs/control_tcp_step_3.rne-replay");
    let (mut child, _stdout, port) = spawn_tcp_control(&replay);

    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect control");
    let mut reader = BufReader::new(stream.try_clone().expect("clone control stream"));

    let mut ready = String::new();
    reader.read_line(&mut ready).expect("read ready");
    assert_eq!(ready.trim(), "ready paused protocol=1");

    stream.write_all(b"step 3\n").expect("write step");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read ack");
    assert_eq!(ack.trim(), "ok paused");

    for expected_step in 1..=3 {
        let mut status = String::new();
        reader.read_line(&mut status).expect("read status");
        assert!(
            status.starts_with(&format!("status step={expected_step} ")),
            "got unexpected status line: {status}"
        );
        assert!(
            status.contains("\"base\"")
                && status.contains("\"base_yaw_rad\"")
                && status.contains("\"joints\"")
                && status.contains("\"sensors\""),
            "status must stream a live observation snapshot, got: {status}"
        );
    }

    stream.write_all(b"quit\n").expect("write quit");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read quit ack");
    assert_eq!(ack.trim(), "ok paused");

    let status = child.wait().expect("wait for rne-asset");
    assert!(status.success(), "rne-asset must exit successfully");

    let artifact =
        rne_log::ReplayArtifact::read_json(&replay).expect("read control replay artifact");
    assert_eq!(
        artifact.frames.len(),
        3,
        "TCP step 3 + quit must produce exactly 3 frames"
    );
    assert_eq!(artifact.clock.steps, 3);
}

/// Scenario manifests use the same paused TCP runner contract and stream traffic snapshots.
#[test]
fn control_tcp_scenario_step_and_quit_streams_traffic_status() {
    let manifest = manifest_dir().join("../../assets/runs/scenario_speed.rne.run.toml");
    let replay = manifest_dir().join("../../target/runs/control_tcp_scenario.rne-replay");
    let (mut child, _stdout, port) = spawn_tcp_control_for(manifest, &replay);

    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect scenario control");
    let mut reader = BufReader::new(stream.try_clone().expect("clone scenario control stream"));
    let mut ready = String::new();
    reader.read_line(&mut ready).expect("read scenario ready");
    assert_eq!(ready.trim(), "ready paused protocol=1");

    stream.write_all(b"step 3\n").expect("write scenario step");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read scenario ack");
    assert_eq!(ack.trim(), "ok paused");
    for expected_step in 1..=3 {
        let mut status = String::new();
        reader.read_line(&mut status).expect("read scenario status");
        assert!(
            status.starts_with(&format!("status step={expected_step} ")),
            "got unexpected scenario status line: {status}"
        );
        assert!(
            status.contains("\"positions_m\"") && status.contains("\"stable_hash\""),
            "scenario status must stream traffic state, got: {status}"
        );
    }

    stream.write_all(b"quit\n").expect("write scenario quit");
    let mut ack = String::new();
    reader.read_line(&mut ack).expect("read scenario quit ack");
    assert_eq!(ack.trim(), "ok paused");
    assert!(
        child.wait().expect("wait for scenario runner").success(),
        "scenario runner must exit successfully"
    );
    let artifact = rne_openscenario::ScenarioReplayArtifact::read_json(&replay)
        .expect("read scenario control artifact");
    assert!(artifact.replayable);
    assert_eq!(
        artifact.control_commands,
        vec![ControlCommand::Step { frames: 3 }, ControlCommand::Quit,]
    );
    assert_eq!(artifact.executed_steps, 3);

    let replay_output = Command::new(BIN)
        .arg("replay")
        .arg(&replay)
        .output()
        .expect("replay scenario control artifact");
    assert!(
        replay_output.status.success(),
        "scenario control replay failed:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&replay_output.stdout),
        String::from_utf8_lossy(&replay_output.stderr)
    );
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

/// The runner starts paused awaiting the first command, so a script without a
/// leading `pause` advances exactly the requested steps regardless of timing.
#[test]
fn control_stdin_starts_paused_and_advances_only_on_command() {
    let replay = manifest_dir().join("../../target/runs/control_start_paused.rne-replay");
    run_control_script(b"step 2\nquit\n", &replay);

    let artifact =
        rne_log::ReplayArtifact::read_json(&replay).expect("read control replay artifact");
    assert_eq!(
        artifact.frames.len(),
        2,
        "the runner must not advance before the first command"
    );
    assert_eq!(artifact.clock.steps, 2);
}
