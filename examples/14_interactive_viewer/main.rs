//! Interactive viewer with keyboard teleop for diff-drive scenes and mobile manipulators.
//!
//! Controls (diff-drive scene):
//! - W / S: drive forward / backward
//! - A / D: turn left / right
//!
//! Controls (`--manipulator` / `--manipulator-mobile` / `--manipulator-lift`):
//! - Q / E: shoulder down / up
//! - Z / X: elbow down / up
//! - C / V: gripper close / open
//! - R / F: lift up / down (lift variant only)
//! - W / S / A / D: base drive (mobile variant only)
//!
//! Shared:
//! - Left / Right: orbit camera
//! - Up / Down: zoom camera
//! - L: toggle LiDAR hit overlay (diff-drive scenes only)
//! - M: toggle semantic task-marker rings
//! - P: toggle camera PiP (remote camera or manipulator profiles)
//! - D: toggle remote GPU depth PiP (`--control-camera-full-resolution`)
//! - Escape: quit
//!
//! Remote runner frontend (diff-drive and URDF profiles):
//! - Space: pause / resume
//! - N: advance one fixed step
//! - T: advance ten fixed steps
//! - R: reset the remote episode
//!
//! Usage:
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- \
//!     --connect 127.0.0.1:9000 assets/scenes/mesh_diff_drive.rne.scene.toml
//!
//! Usage:
//!   cargo run -p interactive_viewer --example 14_interactive_viewer
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- assets/scenes/mesh_diff_drive.rne.scene.toml
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator-mobile
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator-lift
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --so101
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --cart
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --lekiwi
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --lekiwi-so101
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --urdf assets/scenes/unitree_g1_factory.rne.scene.toml
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke

use rne_ai::{
    append_lidar_overlay, append_task_marker_overlay, build_diff_drive_render_scene,
    build_visual_render_scene, cart_minimal_scene_path, lekiwi_scene_path, lekiwi_so101_scene_path,
    mm_lift_scene_path, mm_minimal_scene_path, mm_mobile_scene_path, so101_scene_path,
    DiffDriveAction, DiffDriveSim, MobileManipulatorAction, MobileManipulatorSim, UrdfArmAction,
    UrdfCartAction, UrdfKiwiAction, UrdfSceneSim,
};
use rne_assets::AssetHotReloader;
use rne_math::{Quat, Vec3};
use rne_render::{hash_depth_f32, hash_rgba8, Camera, MeshRenderCache, RenderBackend, VisualShape};
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, WgpuRenderBackend};
use rne_world::Transform3;
use serde::Deserialize;
use std::collections::HashSet;
use std::env;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Duration;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];
const DRIVE_SPEED_RAD_S: f64 = 5.0;
const TURN_DELTA_RAD_S: f64 = 3.0;
const ARM_SPEED_RAD_S: f64 = 2.5;
const GRIPPER_SPEED_RAD_S: f64 = 2.0;
const LIFT_SPEED_M_S: f64 = 0.3;
const REMOTE_LIDAR_COLOR: [f32; 4] = [0.95, 0.80, 0.15, 0.9];

#[derive(Clone, Debug, Deserialize)]
struct RemoteSnapshot {
    #[serde(default)]
    base: Option<[f64; 3]>,
    #[serde(default)]
    base_yaw_rad: Option<f64>,
    #[serde(default)]
    positions_m: Option<Vec<[f64; 3]>>,
    #[serde(default)]
    joints: Option<RemoteJointState>,
    #[serde(default)]
    sensors: Vec<RemoteSensorStream>,
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteJointState {
    names: Vec<String>,
    positions_rad: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteSensorStream {
    #[allow(dead_code)]
    stream_id: u64,
    #[allow(dead_code)]
    kind: String,
    #[allow(dead_code)]
    sequence: u64,
    /// Stable digest of the latest typed payload.
    #[allow(dead_code)]
    #[serde(default)]
    payload_hash: u64,
    /// Bounded RGB-D camera preview.
    #[serde(default)]
    camera: Option<RemoteCameraPreview>,
    /// Bounded world-frame LiDAR preview.
    #[serde(default)]
    lidar: Option<RemoteLidarPreview>,
    /// Latest IMU sample, if present.
    #[allow(dead_code)]
    #[serde(default)]
    imu: Option<RemoteImuSample>,
    /// Latest wheel-encoder sample, if present.
    #[allow(dead_code)]
    #[serde(default)]
    wheel_encoder: Option<RemoteWheelEncoderSample>,
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteCameraPreview {
    width: u32,
    height: u32,
    rgba8_base64: String,
    #[allow(dead_code)]
    #[serde(default)]
    depth_center_m: Option<f32>,
    #[allow(dead_code)]
    #[serde(default)]
    depth_hash: Option<u64>,
    #[allow(dead_code)]
    #[serde(default)]
    depth_width: Option<u32>,
    #[allow(dead_code)]
    #[serde(default)]
    depth_height: Option<u32>,
    #[serde(default)]
    depth_f32_le_base64: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteLidarPreview {
    #[allow(dead_code)]
    #[serde(default)]
    point_count: usize,
    points_m: Vec<[f64; 3]>,
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteImuSample {
    #[allow(dead_code)]
    angular_velocity_rad_s: [f64; 3],
    #[allow(dead_code)]
    linear_acceleration_m_s2: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
struct RemoteWheelEncoderSample {
    #[allow(dead_code)]
    position_rad: f64,
    #[allow(dead_code)]
    velocity_rad_s: f64,
}

impl RemoteSnapshot {
    fn camera_pip(&self) -> Option<(Vec<u8>, u32, u32)> {
        self.sensors
            .iter()
            .filter_map(|stream| stream.camera.as_ref())
            .find_map(|camera| {
                let expected_len = (camera.width as usize)
                    .checked_mul(camera.height as usize)?
                    .checked_mul(4)?;
                let rgba8 = base64::decode(&camera.rgba8_base64).ok()?;
                if camera.width == 0 || camera.height == 0 || rgba8.len() != expected_len {
                    return None;
                }
                Some((rgba8, camera.width, camera.height))
            })
    }

    fn lidar_points(&self) -> Vec<Vec3> {
        self.sensors
            .iter()
            .filter_map(|stream| stream.lidar.as_ref())
            .flat_map(|lidar| lidar.points_m.iter().copied())
            .filter(|point| point.iter().all(|value| value.is_finite()))
            .map(|point| Vec3::new(point[0], point[1], point[2]))
            .collect()
    }

    fn depth_pip(&self) -> Option<(Vec<f32>, u32, u32)> {
        self.sensors
            .iter()
            .filter_map(|stream| stream.camera.as_ref())
            .find_map(|camera| {
                let width = camera.depth_width?;
                let height = camera.depth_height?;
                let expected_len = (width as usize).checked_mul(height as usize)?;
                let bytes = base64::decode(camera.depth_f32_le_base64.as_ref()?).ok()?;
                let expected_byte_len = expected_len.checked_mul(std::mem::size_of::<f32>())?;
                if width == 0 || height == 0 || bytes.len() != expected_byte_len {
                    return None;
                }
                let depth = bytes
                    .chunks_exact(4)
                    .map(|chunk| {
                        let value = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                        if value.is_finite() && value >= 0.0 {
                            value
                        } else {
                            0.0
                        }
                    })
                    .collect::<Vec<_>>();
                Some((depth, width, height))
            })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RemoteRunnerState {
    Paused,
    Running,
}

impl RemoteRunnerState {
    fn as_str(self) -> &'static str {
        match self {
            Self::Paused => "paused",
            Self::Running => "running",
        }
    }
}

#[derive(Clone, Debug)]
struct RemoteStatus {
    step: u64,
    sim_time_s: f64,
    state: RemoteRunnerState,
    snapshot: RemoteSnapshot,
}

enum RemoteEvent {
    Status(RemoteStatus),
    Disconnected,
}

/// Parses one `status ... snapshot=...` line from the runner control protocol.
fn parse_remote_status(line: &str) -> Option<RemoteStatus> {
    let line = line.strip_prefix("status ")?;
    let (fields, snapshot_json) = line.split_once(" snapshot=")?;
    let mut step = None;
    let mut sim_time_s = None;
    let mut state = None;
    for field in fields.split_whitespace() {
        let (key, value) = field.split_once('=')?;
        match key {
            "step" => step = value.parse::<u64>().ok(),
            "t" => sim_time_s = value.parse::<f64>().ok(),
            "state" => {
                state = match value {
                    "paused" => Some(RemoteRunnerState::Paused),
                    "running" => Some(RemoteRunnerState::Running),
                    _ => None,
                }
            }
            _ => {}
        }
    }
    Some(RemoteStatus {
        step: step?,
        sim_time_s: sim_time_s?,
        state: state?,
        snapshot: serde_json::from_str(snapshot_json).ok()?,
    })
}

/// Small native client for the line-oriented runner frontend protocol.
struct RemoteControlClient {
    writer: BufWriter<TcpStream>,
    receiver: mpsc::Receiver<RemoteEvent>,
    latest: Option<RemoteStatus>,
    connected: bool,
}

impl RemoteControlClient {
    fn connect(address: &str) -> Result<Self, String> {
        let mut last_error = None;
        let stream = address
            .to_socket_addrs()
            .map_err(|error| format!("resolve runner address {address}: {error}"))?
            .find_map(|socket_address| {
                match TcpStream::connect_timeout(&socket_address, Duration::from_secs(5)) {
                    Ok(stream) => Some(stream),
                    Err(error) => {
                        last_error = Some(error.to_string());
                        None
                    }
                }
            })
            .ok_or_else(|| {
                format!(
                    "connect runner at {address}: {}",
                    last_error.unwrap_or_else(|| "no address resolved".to_string())
                )
            })?;
        let read_stream = stream
            .try_clone()
            .map_err(|error| format!("clone runner stream: {error}"))?;
        let mut reader = BufReader::new(read_stream);
        let mut ready = String::new();
        reader
            .read_line(&mut ready)
            .map_err(|error| format!("read runner handshake: {error}"))?;
        if !ready.starts_with("ready ") {
            return Err(format!("unexpected runner handshake: {}", ready.trim()));
        }

        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("rne-viewer-runner-reader".into())
            .spawn(move || {
                for line in reader.lines() {
                    let Ok(line) = line else { break };
                    if let Some(status) = parse_remote_status(&line) {
                        if sender.send(RemoteEvent::Status(status)).is_err() {
                            return;
                        }
                    }
                }
                let _ = sender.send(RemoteEvent::Disconnected);
            })
            .map_err(|error| format!("spawn runner reader: {error}"))?;

        Ok(Self {
            writer: BufWriter::new(stream),
            receiver,
            latest: None,
            connected: true,
        })
    }

    fn poll(&mut self) {
        loop {
            match self.receiver.try_recv() {
                Ok(RemoteEvent::Status(status)) => self.latest = Some(status),
                Ok(RemoteEvent::Disconnected) => {
                    self.connected = false;
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.connected = false;
                    break;
                }
            }
        }
    }

    fn latest(&self) -> Option<&RemoteStatus> {
        self.latest.as_ref()
    }

    fn state(&self) -> RemoteRunnerState {
        self.latest
            .as_ref()
            .map(|status| status.state)
            .unwrap_or(RemoteRunnerState::Paused)
    }

    fn state_label(&self) -> &'static str {
        if !self.connected {
            "disconnected"
        } else if self.latest.is_none() {
            "waiting"
        } else {
            self.state().as_str()
        }
    }

    fn send(&mut self, command: &str) -> Result<(), String> {
        if !self.connected {
            return Err("runner connection is closed".to_string());
        }
        if let Err(error) = writeln!(self.writer, "{command}").and_then(|_| self.writer.flush()) {
            self.connected = false;
            return Err(format!("send runner command `{command}`: {error}"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
enum ViewerProfile {
    DiffDriveScene(PathBuf),
    ManipulatorFixed(PathBuf),
    ManipulatorMobile(PathBuf),
    ManipulatorLift(PathBuf),
    So101(PathBuf),
    Cart(PathBuf),
    LeKiwi(PathBuf),
    LeKiwiSo101(PathBuf),
    Urdf(PathBuf),
}

enum ViewerSim {
    DiffDrive(Box<DiffDriveSim>),
    Manipulator(Box<MobileManipulatorSim>),
    UrdfScene(Box<UrdfSceneSim>),
}

impl ViewerSim {
    fn step(&mut self, keys: &HashSet<KeyCode>) {
        match self {
            Self::DiffDrive(sim) => {
                sim.step_action(teleop_diff_drive(keys));
            }
            Self::Manipulator(sim) => {
                sim.step(teleop_manipulator(keys, sim.mobile_base()));
            }
            Self::UrdfScene(sim) => {
                if sim.is_kiwi_drive() && sim.has_arm() {
                    sim.step_kiwi_and_arm(teleop_urdf_kiwi(keys), teleop_urdf_arm(keys));
                } else if sim.is_kiwi_drive() {
                    sim.step_kiwi(teleop_urdf_kiwi(keys));
                } else if sim.left_wheel().is_some() {
                    sim.step_cart(teleop_urdf_cart(keys));
                } else {
                    sim.step_arm(teleop_urdf_arm(keys));
                }
            }
        }
    }

    fn apply_remote_snapshot(&mut self, snapshot: &RemoteSnapshot) -> Result<(), String> {
        let base = snapshot.base.or_else(|| {
            snapshot
                .positions_m
                .as_ref()
                .and_then(|positions| positions.first().copied())
        });
        match self {
            Self::DiffDrive(sim) => {
                let Some(base) = base else {
                    return Ok(());
                };
                let base_link = sim.robot().base_link;
                let mut transform = sim
                    .world_mut()
                    .get_mut::<Transform3>(base_link)
                    .ok_or_else(|| "remote viewer base link has no Transform3".to_string())?;
                transform.translation = Vec3::new(base[0], base[1], base[2]);
                if let Some(yaw_rad) = snapshot.base_yaw_rad {
                    transform.rotation = Quat::from_rotation_y(yaw_rad);
                }
                Ok(())
            }
            Self::UrdfScene(sim) => {
                let (names, positions) = snapshot
                    .joints
                    .as_ref()
                    .map(|joints| (joints.names.as_slice(), joints.positions_rad.as_slice()))
                    .unwrap_or((&[], &[]));
                sim.apply_render_projection(base, snapshot.base_yaw_rad, names, positions);
                Ok(())
            }
            Self::Manipulator(_) => Err(
                "remote runner frontend supports diff-drive and generic URDF profiles; use --urdf for articulated snapshots"
                    .into(),
            ),
        }
    }

    fn focus(&self) -> Vec3 {
        match self {
            Self::DiffDrive(sim) => {
                let obs = sim.observe();
                Vec3::new(obs.base_x_m, 0.25, obs.base_z_m)
            }
            Self::Manipulator(sim) => {
                let obs = sim.observe();
                Vec3::new(obs.ee_x_m, obs.ee_y_m, obs.ee_z_m)
            }
            Self::UrdfScene(sim) => {
                let obs = sim.observe();
                Vec3::new(obs.base_x_m, obs.base_y_m + 0.25, obs.base_z_m)
            }
        }
    }

    fn hud_line(&self) -> String {
        match self {
            Self::DiffDrive(sim) => {
                let obs = sim.observe();
                format!(
                    "base=({:.2}, {:.2}, {:.2}) yaw={:.2} rad",
                    obs.base_x_m, obs.base_y_m, obs.base_z_m, obs.base_yaw_rad
                )
            }
            Self::Manipulator(sim) => {
                let obs = sim.observe();
                format!(
                    "ee=({:.2}, {:.2}, {:.2}) shoulder={:.2} rad elbow={:.2} rad base=({:.2}, {:.2})",
                    obs.ee_x_m,
                    obs.ee_y_m,
                    obs.ee_z_m,
                    obs.shoulder_position_rad,
                    obs.elbow_position_rad,
                    obs.base_x_m,
                    obs.base_z_m
                )
            }
            Self::UrdfScene(sim) => {
                let obs = sim.observe();
                format!(
                    "base=({:.2}, {:.2}, {:.2}) yaw={:.2} rad joints={}",
                    obs.base_x_m,
                    obs.base_y_m,
                    obs.base_z_m,
                    obs.base_yaw_rad,
                    obs.actuated_joint_count
                )
            }
        }
    }

    fn wrist_camera_pip(&self) -> Option<(Vec<u8>, u32, u32)> {
        match self {
            Self::Manipulator(sim) => sim
                .latest_wrist_camera()
                .map(|image| (image.rgba8, image.width, image.height)),
            Self::DiffDrive(_) => None,
            Self::UrdfScene(_) => None,
        }
    }

    fn wrist_camera_enabled(&self) -> bool {
        matches!(self, Self::Manipulator(sim) if sim.wrist_camera_enabled())
    }

    fn build_scene(
        &self,
        show_lidar: bool,
        show_task_markers: bool,
        remote_lidar_points: Option<&[Vec3]>,
    ) -> rne_render::RenderScene {
        let mut scene = match self {
            Self::DiffDrive(sim) => build_diff_drive_render_scene(sim.world(), sim.robots()),
            Self::Manipulator(sim) => build_visual_render_scene(sim.world()),
            Self::UrdfScene(sim) => build_visual_render_scene(sim.world()),
        };
        if show_lidar {
            if let Some(points) = remote_lidar_points {
                scene.append_lidar_points(points, REMOTE_LIDAR_COLOR);
            } else if let Self::DiffDrive(sim) = self {
                append_lidar_overlay(&mut scene, sim.world(), sim.data_bus());
            }
        }
        if show_task_markers {
            match self {
                Self::DiffDrive(sim) => {
                    append_task_marker_overlay(&mut scene, sim.world());
                }
                Self::Manipulator(sim) => {
                    append_task_marker_overlay(&mut scene, sim.world());
                }
                Self::UrdfScene(sim) => {
                    append_task_marker_overlay(&mut scene, sim.world());
                }
            }
        }
        scene
    }

    fn mesh_roots(&self) -> Vec<PathBuf> {
        match self {
            Self::DiffDrive(sim) => sim.mesh_package_roots().to_vec(),
            Self::Manipulator(_) => Vec::new(),
            Self::UrdfScene(sim) => sim.mesh_package_roots().to_vec(),
        }
    }

    fn reload_scene(&mut self, scene_path: &Path) -> Result<(), String> {
        match self {
            Self::DiffDrive(sim) => sim
                .reload_scene()
                .map_err(|error| format!("reload scene: {error}")),
            Self::Manipulator(sim) => {
                **sim = MobileManipulatorSim::from_scene_path(scene_path)
                    .map_err(|error| format!("reload manipulator scene: {error}"))?;
                Ok(())
            }
            Self::UrdfScene(sim) => {
                **sim = UrdfSceneSim::from_scene_path(scene_path)
                    .map_err(|error| format!("reload URDF scene: {error}"))?;
                Ok(())
            }
        }
    }

    fn world_seed(&self) -> u64 {
        match self {
            Self::DiffDrive(sim) => sim.world_seed(),
            Self::Manipulator(_) => 0,
            Self::UrdfScene(sim) => sim.world_seed(),
        }
    }

    fn smoke_base_x(&self) -> f64 {
        match self {
            Self::DiffDrive(sim) => sim.observe().base_x_m,
            Self::Manipulator(_) => 0.0,
            Self::UrdfScene(sim) => sim.observe().base_x_m,
        }
    }

    fn smoke_lidar_hits(&self) -> usize {
        match self {
            Self::DiffDrive(sim) => {
                let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
                append_lidar_overlay(&mut scene, sim.world(), sim.data_bus()).hit_markers
            }
            Self::Manipulator(_) => 0,
            Self::UrdfScene(_) => 0,
        }
    }
}

fn main() {
    let smoke = env::args().any(|arg| arg == "--smoke") || env::var("RNE_VIEWER_SMOKE").is_ok();
    let profile = viewer_profile_from_args();

    if smoke || env::var("RNE_SKIP_GPU").is_ok() {
        run_smoke(smoke, &profile);
        return;
    }

    let event_loop = EventLoop::new().expect("create event loop");
    let remote_addr = viewer_remote_addr_from_args();
    let mut app = App::new(profile, remote_addr);
    event_loop.run_app(&mut app).expect("run viewer");
}

fn default_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml")
}

fn viewer_remote_addr_from_args() -> Option<String> {
    let args: Vec<String> = env::args().skip(1).collect();
    args.windows(2)
        .find(|pair| pair[0] == "--connect")
        .map(|pair| pair[1].clone())
        .or_else(|| {
            args.iter()
                .find_map(|arg| arg.strip_prefix("--connect=").map(str::to_string))
        })
        .or_else(|| env::var("RNE_VIEWER_CONNECT").ok())
}

fn viewer_profile_from_args() -> ViewerProfile {
    let args: Vec<String> = env::args().skip(1).collect();
    let scene_arg = args
        .iter()
        .find(|arg| !arg.starts_with('-') && arg.ends_with(".scene.toml"))
        .map(PathBuf::from);
    if args.iter().any(|arg| arg == "--manipulator-mobile") {
        return ViewerProfile::ManipulatorMobile(scene_arg.unwrap_or_else(mm_mobile_scene_path));
    }
    if args.iter().any(|arg| arg == "--manipulator-lift") {
        return ViewerProfile::ManipulatorLift(scene_arg.unwrap_or_else(mm_lift_scene_path));
    }
    if args.iter().any(|arg| arg == "--manipulator") {
        return ViewerProfile::ManipulatorFixed(scene_arg.unwrap_or_else(mm_minimal_scene_path));
    }
    if args.iter().any(|arg| arg == "--so101") {
        return ViewerProfile::So101(scene_arg.unwrap_or_else(so101_scene_path));
    }
    if args.iter().any(|arg| arg == "--cart") {
        return ViewerProfile::Cart(scene_arg.unwrap_or_else(cart_minimal_scene_path));
    }
    if args.iter().any(|arg| arg == "--lekiwi") {
        return ViewerProfile::LeKiwi(scene_arg.unwrap_or_else(lekiwi_scene_path));
    }
    if args.iter().any(|arg| arg == "--lekiwi-so101") {
        return ViewerProfile::LeKiwiSo101(scene_arg.unwrap_or_else(lekiwi_so101_scene_path));
    }
    if args.iter().any(|arg| arg == "--urdf") {
        return ViewerProfile::Urdf(
            scene_arg.unwrap_or_else(|| panic!("--urdf requires a .scene.toml path")),
        );
    }
    let scene_path = scene_arg.unwrap_or_else(default_scene_path);
    ViewerProfile::DiffDriveScene(scene_path)
}

fn load_sim(profile: &ViewerProfile) -> Result<ViewerSim, String> {
    match profile {
        ViewerProfile::DiffDriveScene(path) => DiffDriveSim::from_scene_path(path)
            .map(|sim| ViewerSim::DiffDrive(Box::new(sim)))
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorFixed(path) => MobileManipulatorSim::from_scene_path(path)
            .map(|sim| ViewerSim::Manipulator(Box::new(sim)))
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorMobile(path) => MobileManipulatorSim::from_scene_path(path)
            .map(|sim| ViewerSim::Manipulator(Box::new(sim)))
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorLift(path) => MobileManipulatorSim::from_scene_path(path)
            .map(|sim| ViewerSim::Manipulator(Box::new(sim)))
            .map_err(|error| error.to_string()),
        ViewerProfile::So101(path)
        | ViewerProfile::Cart(path)
        | ViewerProfile::LeKiwi(path)
        | ViewerProfile::LeKiwiSo101(path)
        | ViewerProfile::Urdf(path) => UrdfSceneSim::from_scene_path(path)
            .map(|sim| ViewerSim::UrdfScene(Box::new(sim)))
            .map_err(|error| error.to_string()),
    }
}

fn profile_label(profile: &ViewerProfile) -> String {
    match profile {
        ViewerProfile::DiffDriveScene(path) => path.display().to_string(),
        ViewerProfile::ManipulatorFixed(path) => format!("mm_minimal ({})", path.display()),
        ViewerProfile::ManipulatorMobile(path) => format!("mm_mobile ({})", path.display()),
        ViewerProfile::ManipulatorLift(path) => format!("mm_lift ({})", path.display()),
        ViewerProfile::So101(path) => format!("so101 ({})", path.display()),
        ViewerProfile::Cart(path) => format!("cart_minimal ({})", path.display()),
        ViewerProfile::LeKiwi(path) => format!("lekiwi ({})", path.display()),
        ViewerProfile::LeKiwiSo101(path) => format!("lekiwi_so101 ({})", path.display()),
        ViewerProfile::Urdf(path) => format!("urdf ({})", path.display()),
    }
}

fn profile_scene_path(profile: &ViewerProfile) -> &Path {
    match profile {
        ViewerProfile::DiffDriveScene(path)
        | ViewerProfile::ManipulatorFixed(path)
        | ViewerProfile::ManipulatorMobile(path)
        | ViewerProfile::ManipulatorLift(path)
        | ViewerProfile::So101(path)
        | ViewerProfile::Cart(path)
        | ViewerProfile::LeKiwi(path)
        | ViewerProfile::LeKiwiSo101(path)
        | ViewerProfile::Urdf(path) => path,
    }
}

fn run_smoke(explicit: bool, profile: &ViewerProfile) {
    if env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping interactive viewer smoke");
        return;
    }

    let mut sim = load_sim(profile).expect("load viewer simulation");
    let initial_urdf_pose = match &sim {
        ViewerSim::UrdfScene(sim) => Some(sim.observe()),
        _ => None,
    };
    for _ in 0..60 {
        sim.step(&smoke_keys(profile));
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };

    let mut scene = sim.build_scene(
        matches!(profile, ViewerProfile::DiffDriveScene(_)),
        true,
        None,
    );
    let mesh_items = count_mesh_items(&scene);
    let mut mesh_cache = MeshRenderCache::new();
    let mesh_roots = sim.mesh_roots();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    mesh_cache
        .resolve_scene(&mut scene, &mesh_root_refs)
        .expect("resolve mesh assets");

    let orbit = CameraOrbit {
        focus: sim.focus(),
        yaw_rad: -0.09,
        pitch_rad: 0.52,
        distance_m: 3.6,
    };
    let camera = Camera::new(640, 360, std::f64::consts::FRAC_PI_4);
    let view = orbit.camera_transform();

    let output = backend
        .render_scene_camera(&camera, &view, &scene, CLEAR_COLOR)
        .expect("smoke render");

    let lidar_hits = sim.smoke_lidar_hits();
    println!(
        "interactive viewer smoke{}: profile={} seed={} items={} mesh_items={} lidar_hits={} color_hash={:#018x} depth_hash={:#018x} base_x={:.2} m hud={}",
        if explicit { "" } else { " (RNE_SKIP_GPU fallback)" },
        profile_label(profile),
        sim.world_seed(),
        scene.items.len(),
        mesh_items,
        lidar_hits,
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        sim.smoke_base_x(),
        sim.hud_line()
    );

    if scene.items.is_empty() {
        std::process::exit(1);
    }

    match profile {
        ViewerProfile::DiffDriveScene(_) => {
            if sim.smoke_base_x() <= 0.0 {
                std::process::exit(1);
            }
            if lidar_hits < 4 {
                eprintln!("interactive viewer smoke expected lidar hits, got {lidar_hits}");
                std::process::exit(1);
            }
        }
        ViewerProfile::ManipulatorFixed(_) => {
            let obs = match &sim {
                ViewerSim::Manipulator(sim) => sim.observe(),
                _ => unreachable!(),
            };
            if obs.joint_state_count < 4 {
                std::process::exit(1);
            }
            if !sim.wrist_camera_enabled() || obs.wrist_camera_pixels < 64 * 48 * 4 {
                eprintln!(
                    "interactive viewer smoke expected wrist camera pixels, got {}",
                    obs.wrist_camera_pixels
                );
                std::process::exit(1);
            }
        }
        ViewerProfile::ManipulatorMobile(_) => {
            let obs = match &sim {
                ViewerSim::Manipulator(sim) => sim.observe(),
                _ => unreachable!(),
            };
            if obs.base_x_m.abs() <= 0.05 && obs.base_z_m.abs() <= 0.05 {
                std::process::exit(1);
            }
        }
        ViewerProfile::ManipulatorLift(_) => {
            let obs = match &sim {
                ViewerSim::Manipulator(sim) => sim.observe(),
                _ => unreachable!(),
            };
            // The lift robot has 5 actuated joints (lift + shoulder + elbow + 2 fingers).
            if obs.joint_state_count < 5 {
                eprintln!(
                    "interactive viewer smoke expected 5 lift joints, got {}",
                    obs.joint_state_count
                );
                std::process::exit(1);
            }
        }
        ViewerProfile::So101(_) => {
            let obs = match &sim {
                ViewerSim::UrdfScene(sim) => sim.observe(),
                _ => unreachable!(),
            };
            if obs.actuated_joint_count < 5 {
                eprintln!(
                    "interactive viewer smoke expected SO-101 actuated joints, got {}",
                    obs.actuated_joint_count
                );
                std::process::exit(1);
            }
            if mesh_items < 5 {
                eprintln!(
                    "interactive viewer smoke expected SO-101 mesh visuals, got {mesh_items}"
                );
                std::process::exit(1);
            }
        }
        ViewerProfile::Cart(_) => {
            if sim.smoke_base_x().abs() <= 0.02 {
                eprintln!(
                    "interactive viewer smoke expected cart displacement, base_x={}",
                    sim.smoke_base_x()
                );
                std::process::exit(1);
            }
        }
        ViewerProfile::LeKiwi(_) => {
            let obs = match &sim {
                ViewerSim::UrdfScene(sim) => sim.observe(),
                _ => unreachable!(),
            };
            if obs.actuated_joint_count < 3 {
                eprintln!(
                    "interactive viewer smoke expected 3 lekiwi wheel joints, got {}",
                    obs.actuated_joint_count
                );
                std::process::exit(1);
            }
            if mesh_items < 3 {
                eprintln!(
                    "interactive viewer smoke expected lekiwi mesh visuals, got {mesh_items}"
                );
                std::process::exit(1);
            }
            if let Some(initial) = initial_urdf_pose {
                let dx_m = obs.base_x_m - initial.base_x_m;
                let dz_m = obs.base_z_m - initial.base_z_m;
                let planar_m = (dx_m * dx_m + dz_m * dz_m).sqrt();
                if planar_m <= 0.02 {
                    eprintln!(
                        "interactive viewer smoke expected lekiwi displacement, planar={planar_m:.4} m"
                    );
                    std::process::exit(1);
                }
            }
        }
        ViewerProfile::LeKiwiSo101(_) => {
            let obs = match &sim {
                ViewerSim::UrdfScene(sim) => sim.observe(),
                _ => unreachable!(),
            };
            if obs.actuated_joint_count < 8 {
                eprintln!(
                    "interactive viewer smoke expected lekiwi_so101 actuated joints, got {}",
                    obs.actuated_joint_count
                );
                std::process::exit(1);
            }
            if mesh_items < 8 {
                eprintln!(
                    "interactive viewer smoke expected lekiwi_so101 mesh visuals, got {mesh_items}"
                );
                std::process::exit(1);
            }
            if let Some(initial) = initial_urdf_pose {
                let dx_m = obs.base_x_m - initial.base_x_m;
                let dz_m = obs.base_z_m - initial.base_z_m;
                let planar_m = (dx_m * dx_m + dz_m * dz_m).sqrt();
                if !(0.02..=2.0).contains(&planar_m) {
                    eprintln!(
                        "interactive viewer smoke expected lekiwi_so101 displacement in (0.02, 2.0] m, planar={planar_m:.4} m"
                    );
                    std::process::exit(1);
                }
            }
        }
        ViewerProfile::Urdf(_) => {}
    }

    let center = (output.color.height / 2 * output.color.width + output.color.width / 2) as usize;
    let center_depth = output.depth.depth_m[center];
    if center_depth >= camera.far_m as f32 {
        eprintln!("interactive viewer smoke render invalid (center_depth={center_depth:.2} m)");
        std::process::exit(1);
    }
}

fn smoke_keys(profile: &ViewerProfile) -> HashSet<KeyCode> {
    let mut keys = HashSet::new();
    match profile {
        ViewerProfile::DiffDriveScene(_) | ViewerProfile::ManipulatorMobile(_) => {
            keys.insert(KeyCode::KeyW);
        }
        ViewerProfile::ManipulatorFixed(_) => {
            keys.insert(KeyCode::KeyQ);
        }
        ViewerProfile::ManipulatorLift(_) => {
            keys.insert(KeyCode::KeyR);
        }
        ViewerProfile::So101(_) => {
            keys.insert(KeyCode::KeyQ);
        }
        ViewerProfile::Cart(_) => {
            keys.insert(KeyCode::KeyW);
        }
        ViewerProfile::LeKiwi(_) => {
            keys.insert(KeyCode::KeyW);
        }
        ViewerProfile::LeKiwiSo101(_) => {
            keys.insert(KeyCode::KeyW);
            keys.insert(KeyCode::KeyQ);
        }
        ViewerProfile::Urdf(_) => {}
    }
    keys
}

struct App {
    profile: ViewerProfile,
    remote_addr: Option<String>,
    window: Option<Arc<Window>>,
    viewer: Option<InteractiveViewer>,
    sim: Option<ViewerSim>,
    remote: Option<RemoteControlClient>,
    hot_reloader: Option<AssetHotReloader>,
    mesh_cache: MeshRenderCache,
    reload_count: u32,
    reload_pending: bool,
    last_reload_error: Option<String>,
    orbit: CameraOrbit,
    pressed: HashSet<KeyCode>,
    show_lidar: bool,
    show_task_markers: bool,
    show_wrist_camera: bool,
    show_depth_pip: bool,
    last_hud: String,
}

impl App {
    fn new(profile: ViewerProfile, remote_addr: Option<String>) -> Self {
        Self {
            profile,
            remote_addr,
            window: None,
            viewer: None,
            sim: None,
            remote: None,
            hot_reloader: None,
            mesh_cache: MeshRenderCache::new(),
            reload_count: 0,
            reload_pending: false,
            last_reload_error: None,
            orbit: CameraOrbit::default(),
            pressed: HashSet::new(),
            show_lidar: true,
            show_task_markers: true,
            show_wrist_camera: true,
            show_depth_pip: false,
            last_hud: String::new(),
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let title = format!("RNE Interactive Viewer — {}", profile_label(&self.profile));
        let window = Arc::new(
            event_loop
                .create_window(
                    Window::default_attributes()
                        .with_title(title)
                        .with_inner_size(winit::dpi::LogicalSize::new(960, 720)),
                )
                .expect("create window"),
        );

        let viewer = match InteractiveViewer::new(window.clone()) {
            Ok(viewer) => viewer,
            Err(error) => {
                eprintln!("viewer init failed: {error}");
                event_loop.exit();
                return;
            }
        };

        let sim = match load_sim(&self.profile) {
            Ok(sim) => sim,
            Err(error) => {
                eprintln!("failed to load viewer profile: {error}");
                event_loop.exit();
                return;
            }
        };

        let remote = if let Some(address) = self.remote_addr.as_deref() {
            if matches!(
                &self.profile,
                ViewerProfile::ManipulatorFixed(_)
                    | ViewerProfile::ManipulatorMobile(_)
                    | ViewerProfile::ManipulatorLift(_)
            ) {
                eprintln!("--connect requires a diff-drive or generic URDF scene profile");
                event_loop.exit();
                return;
            }
            match RemoteControlClient::connect(address) {
                Ok(remote) => {
                    println!("connected to runner frontend at {address}");
                    Some(remote)
                }
                Err(error) => {
                    eprintln!("runner frontend connection failed: {error}");
                    event_loop.exit();
                    return;
                }
            }
        } else {
            None
        };

        let hot_reloader = match AssetHotReloader::load(profile_scene_path(&self.profile)) {
            Ok(reloader) => Some(reloader),
            Err(error) => {
                eprintln!("failed to watch scene dependencies: {error}");
                event_loop.exit();
                return;
            }
        };

        self.orbit.focus = sim.focus();
        self.mesh_cache.clear();
        println!(
            "loaded {} (seed={}, mesh_roots={})",
            profile_label(&self.profile),
            sim.world_seed(),
            sim.mesh_roots().len()
        );

        self.window = Some(window);
        self.viewer = Some(viewer);
        self.sim = Some(sim);
        self.remote = remote;
        self.hot_reloader = hot_reloader;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(viewer) = self.viewer.as_mut() {
                    viewer.resize(size.width, size.height);
                }
            }
            WindowEvent::KeyboardInput { event, .. } => self.handle_key(event),
            WindowEvent::RedrawRequested => {
                if let Err(error) = self.frame() {
                    eprintln!("render error: {error}");
                    event_loop.exit();
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

impl App {
    fn handle_key(&mut self, event: KeyEvent) {
        let physical = match event.physical_key {
            PhysicalKey::Code(code) => code,
            _ => return,
        };

        match event.state {
            ElementState::Pressed => {
                if physical == KeyCode::Escape {
                    if let Some(remote) = self.remote.as_mut() {
                        let _ = remote.send("quit");
                    }
                    std::process::exit(0);
                }
                if physical == KeyCode::KeyL
                    && matches!(self.profile, ViewerProfile::DiffDriveScene(_))
                {
                    self.show_lidar = !self.show_lidar;
                    println!(
                        "lidar overlay {}",
                        if self.show_lidar {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
                }
                if physical == KeyCode::KeyP
                    && (self.remote.is_some()
                        || matches!(
                            self.profile,
                            ViewerProfile::ManipulatorFixed(_)
                                | ViewerProfile::ManipulatorMobile(_)
                                | ViewerProfile::ManipulatorLift(_)
                        ))
                {
                    self.show_wrist_camera = !self.show_wrist_camera;
                    println!(
                        "camera pip {}",
                        if self.show_wrist_camera {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
                }
                if physical == KeyCode::KeyD && self.remote.is_some() {
                    self.show_depth_pip = !self.show_depth_pip;
                    println!(
                        "depth pip {}",
                        if self.show_depth_pip {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
                }
                if physical == KeyCode::KeyM {
                    self.show_task_markers = !self.show_task_markers;
                    println!(
                        "task marker overlay {}",
                        if self.show_task_markers {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
                }
                if let Some(remote) = self.remote.as_mut() {
                    let command = match physical {
                        KeyCode::Space => Some(if remote.state() == RemoteRunnerState::Paused {
                            "resume"
                        } else {
                            "pause"
                        }),
                        KeyCode::KeyN => Some("step 1"),
                        KeyCode::KeyT => Some("step 10"),
                        KeyCode::KeyR => Some("reset"),
                        _ => None,
                    };
                    if let Some(command) = command {
                        if let Err(error) = remote.send(command) {
                            eprintln!("{error}");
                        }
                        return;
                    }
                }
                self.pressed.insert(physical);
            }
            ElementState::Released => {
                self.pressed.remove(&physical);
            }
        }
    }

    fn frame(&mut self) -> Result<(), String> {
        self.apply_camera_input();
        self.poll_hot_reload()?;

        let remote_status = if let Some(remote) = self.remote.as_mut() {
            remote.poll();
            remote.latest().cloned()
        } else {
            None
        };

        let sim = self.sim.as_mut().ok_or("simulation not ready")?;
        let remote_lidar_points = remote_status
            .as_ref()
            .map(|status| status.snapshot.lidar_points());
        if let Some(status) = &remote_status {
            sim.apply_remote_snapshot(&status.snapshot)?;
        } else if self.remote.is_none() {
            sim.step(&self.pressed);
        }
        self.orbit.focus = sim.focus();

        let hud = if let Some(status) = &remote_status {
            let joint_count = status
                .snapshot
                .joints
                .as_ref()
                .map_or(0, |joints| joints.positions_rad.len());
            format!(
                "remote step={} t={:.3} state={} joints={} sensors={} {}",
                status.step,
                status.sim_time_s,
                status.state.as_str(),
                joint_count,
                status.snapshot.sensors.len(),
                sim.hud_line()
            )
        } else if let Some(remote) = self.remote.as_ref() {
            format!("remote {} {}", remote.state_label(), sim.hud_line())
        } else {
            sim.hud_line()
        };
        if hud != self.last_hud {
            if let Some(window) = &self.window {
                window.set_title(&format!(
                    "RNE Interactive Viewer — {} | {}",
                    profile_label(&self.profile),
                    hud
                ));
            }
            self.last_hud = hud;
        }

        let mut scene = sim.build_scene(
            self.show_lidar,
            self.show_task_markers,
            remote_lidar_points.as_deref(),
        );
        let mesh_roots = sim.mesh_roots();
        let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
        self.mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .map_err(|error| error.to_string())?;

        let view = self.orbit.camera_transform();
        let viewer = self.viewer.as_mut().ok_or("viewer not ready")?;
        let pip = if self.show_wrist_camera {
            if self.remote.is_some() {
                remote_status
                    .as_ref()
                    .and_then(|status| status.snapshot.camera_pip())
            } else {
                sim.wrist_camera_pip()
            }
        } else {
            None
        };
        let depth_pip = if self.show_depth_pip {
            remote_status
                .as_ref()
                .and_then(|status| status.snapshot.depth_pip())
        } else {
            None
        };
        viewer
            .render_with_pip_and_depth(&view, &scene, CLEAR_COLOR, pip, depth_pip)
            .map_err(|error| error.to_string())
    }

    fn poll_hot_reload(&mut self) -> Result<(), String> {
        if !self.reload_pending {
            let poll_result = match self.hot_reloader.as_mut() {
                Some(reloader) => reloader.poll(),
                None => return Ok(()),
            };
            match poll_result {
                Ok(true) => self.reload_pending = true,
                Ok(false) => return Ok(()),
                Err(error) => {
                    self.report_reload_error(format!("asset reload rejected: {error}"));
                    return Ok(());
                }
            }
        }

        let reload_result = self
            .sim
            .as_mut()
            .ok_or("simulation not ready")?
            .reload_scene(profile_scene_path(&self.profile));
        if let Err(error) = reload_result {
            self.report_reload_error(error);
            return Ok(());
        }
        self.reload_pending = false;
        self.last_reload_error = None;
        self.reload_count += 1;
        self.mesh_cache.clear();
        let sim = self.sim.as_ref().ok_or("simulation not ready")?;
        self.orbit.focus = sim.focus();
        println!(
            "reloaded scene (#{}) seed={} mesh_roots={}",
            self.reload_count,
            sim.world_seed(),
            sim.mesh_roots().len()
        );
        Ok(())
    }

    fn report_reload_error(&mut self, error: String) {
        if self.last_reload_error.as_deref() != Some(&error) {
            eprintln!("{error}; keeping the last valid World and waiting for another save");
            self.last_reload_error = Some(error);
        }
    }

    fn apply_camera_input(&mut self) {
        if self.pressed.contains(&KeyCode::ArrowLeft) {
            self.orbit.yaw_rad -= 0.04;
        }
        if self.pressed.contains(&KeyCode::ArrowRight) {
            self.orbit.yaw_rad += 0.04;
        }
        if self.pressed.contains(&KeyCode::ArrowUp) {
            self.orbit.distance_m = (self.orbit.distance_m - 0.08).max(1.5);
        }
        if self.pressed.contains(&KeyCode::ArrowDown) {
            self.orbit.distance_m = (self.orbit.distance_m + 0.08).min(12.0);
        }
    }
}

fn teleop_diff_drive(keys: &HashSet<KeyCode>) -> DiffDriveAction {
    let forward = keys.contains(&KeyCode::KeyW);
    let backward = keys.contains(&KeyCode::KeyS);
    let left = keys.contains(&KeyCode::KeyA);
    let right = keys.contains(&KeyCode::KeyD);

    let mut linear = 0.0;
    if forward {
        linear += DRIVE_SPEED_RAD_S;
    }
    if backward {
        linear -= DRIVE_SPEED_RAD_S * 0.6;
    }

    let mut turn = 0.0;
    if left {
        turn -= TURN_DELTA_RAD_S;
    }
    if right {
        turn += TURN_DELTA_RAD_S;
    }

    DiffDriveAction {
        left_velocity_rad_s: linear - turn,
        right_velocity_rad_s: linear + turn,
    }
}

fn teleop_manipulator(keys: &HashSet<KeyCode>, mobile_base: bool) -> MobileManipulatorAction {
    let mut action = MobileManipulatorAction::default();
    if keys.contains(&KeyCode::KeyQ) {
        action.shoulder_velocity_rad_s += ARM_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyE) {
        action.shoulder_velocity_rad_s -= ARM_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyZ) {
        action.elbow_velocity_rad_s += ARM_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyX) {
        action.elbow_velocity_rad_s -= ARM_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyC) {
        action.gripper_velocity_rad_s -= GRIPPER_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyV) {
        action.gripper_velocity_rad_s += GRIPPER_SPEED_RAD_S;
    }
    // Vertical lift (lift robot only; ignored by robots without a lift joint).
    if keys.contains(&KeyCode::KeyR) {
        action.lift_velocity_m_s += LIFT_SPEED_M_S;
    }
    if keys.contains(&KeyCode::KeyF) {
        action.lift_velocity_m_s -= LIFT_SPEED_M_S;
    }

    if mobile_base {
        let drive = teleop_diff_drive(keys);
        action.left_wheel_velocity_rad_s = drive.left_velocity_rad_s;
        action.right_wheel_velocity_rad_s = drive.right_velocity_rad_s;
    }

    action
}

fn teleop_urdf_cart(keys: &HashSet<KeyCode>) -> UrdfCartAction {
    let drive = teleop_diff_drive(keys);
    UrdfCartAction {
        left_velocity_rad_s: drive.left_velocity_rad_s,
        right_velocity_rad_s: drive.right_velocity_rad_s,
    }
}

fn teleop_urdf_arm(keys: &HashSet<KeyCode>) -> UrdfArmAction {
    let mut shoulder_pan_velocity_rad_s = 0.0;
    if keys.contains(&KeyCode::KeyQ) {
        shoulder_pan_velocity_rad_s += ARM_SPEED_RAD_S;
    }
    if keys.contains(&KeyCode::KeyE) {
        shoulder_pan_velocity_rad_s -= ARM_SPEED_RAD_S;
    }
    UrdfArmAction {
        shoulder_pan_velocity_rad_s,
    }
}

const KIWI_DRIVE_SPEED_M_S: f64 = 0.35;
const KIWI_TURN_RAD_S: f64 = 0.45;

fn teleop_urdf_kiwi(keys: &HashSet<KeyCode>) -> UrdfKiwiAction {
    let mut vx_m_s = 0.0;
    let mut vz_m_s = 0.0;
    let mut wz_rad_s = 0.0;
    if keys.contains(&KeyCode::KeyW) {
        vx_m_s += KIWI_DRIVE_SPEED_M_S;
    }
    if keys.contains(&KeyCode::KeyS) {
        vx_m_s -= KIWI_DRIVE_SPEED_M_S;
    }
    if keys.contains(&KeyCode::KeyA) {
        vz_m_s += KIWI_DRIVE_SPEED_M_S;
    }
    if keys.contains(&KeyCode::KeyD) {
        vz_m_s -= KIWI_DRIVE_SPEED_M_S;
    }
    if keys.contains(&KeyCode::KeyQ) {
        wz_rad_s += KIWI_TURN_RAD_S;
    }
    if keys.contains(&KeyCode::KeyE) {
        wz_rad_s -= KIWI_TURN_RAD_S;
    }
    UrdfKiwiAction {
        vx_m_s,
        vz_m_s,
        wz_rad_s,
    }
}

fn count_mesh_items(scene: &rne_render::RenderScene) -> usize {
    scene
        .items
        .iter()
        .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_scene_path_exists() {
        assert!(default_scene_path().is_file());
    }

    #[test]
    fn remote_status_parses_pose_and_sensor_summary() {
        let status = parse_remote_status(
            r#"status step=7 t=0.116667 state=paused snapshot={"base":[1.0,0.25,2.0],"base_yaw_rad":0.5,"joints":null,"sensors":[{"stream_id":1,"kind":"imu","sequence":7}]}"#,
        )
        .expect("parse remote status");
        assert_eq!(status.step, 7);
        assert_eq!(status.state, RemoteRunnerState::Paused);
        assert_eq!(status.snapshot.base, Some([1.0, 0.25, 2.0]));
        assert_eq!(status.snapshot.base_yaw_rad, Some(0.5));
        assert_eq!(status.snapshot.sensors.len(), 1);
    }

    #[test]
    fn remote_status_decodes_camera_and_lidar_previews() {
        let status = parse_remote_status(
            r#"status step=8 t=0.133333 state=paused snapshot={"base":[1.0,0.25,2.0],"base_yaw_rad":0.5,"joints":null,"sensors":[{"stream_id":2,"kind":"camera","sequence":8,"payload_hash":11,"camera":{"width":1,"height":1,"rgba8_base64":"AQIDBA==","depth_center_m":1.5,"depth_hash":12,"depth_width":2,"depth_height":1,"depth_f32_le_base64":"AACAPwAAAEA="}},{"stream_id":3,"kind":"lidar","sequence":8,"payload_hash":13,"lidar":{"point_count":1,"points_m":[[2.0,0.3,1.0]]}}]}"#,
        )
        .expect("parse sensor previews");
        assert_eq!(status.snapshot.camera_pip(), Some((vec![1, 2, 3, 4], 1, 1)));
        assert_eq!(
            status.snapshot.lidar_points(),
            vec![Vec3::new(2.0, 0.3, 1.0)]
        );
        assert_eq!(status.snapshot.depth_pip(), Some((vec![1.0, 2.0], 2, 1)));
    }

    #[test]
    fn remote_status_parses_scenario_traffic_positions() {
        let status = parse_remote_status(
            r#"status step=3 t=0.050000 state=paused snapshot={"positions_m":[[4.0,0.0,2.0]],"signal_violations":0,"collisions":0,"stable_hash":42,"average_speed_m_s":3.0}"#,
        )
        .expect("parse scenario status");
        assert_eq!(status.snapshot.base, None);
        assert_eq!(status.snapshot.positions_m, Some(vec![[4.0, 0.0, 2.0]]));
    }

    #[test]
    fn remote_status_ignores_command_ack_lines() {
        assert!(parse_remote_status("ok paused").is_none());
        assert!(parse_remote_status("ready paused").is_none());
    }

    #[test]
    fn mesh_scene_loads_visuals() {
        let scene_path = default_scene_path();
        let sim = DiffDriveSim::from_scene_path(&scene_path).expect("load scene");
        assert!(!sim.mesh_package_roots().is_empty());
        let scene = build_diff_drive_render_scene(sim.world(), sim.robots());
        assert!(count_mesh_items(&scene) >= 1);
        let cylinder_items = scene
            .items
            .iter()
            .filter(|item| matches!(item.shape, VisualShape::Cylinder { .. }))
            .count();
        assert!(
            cylinder_items >= 2,
            "expected wheel cylinder visuals, got {cylinder_items}"
        );
        assert!(
            scene.items.len() >= 4,
            "expected base + wheels + ground plane items"
        );
    }

    #[test]
    fn manipulator_visual_scene_has_links() {
        let sim = MobileManipulatorSim::from_scene_path(&mm_minimal_scene_path())
            .expect("load mm_minimal scene");
        let scene = build_visual_render_scene(sim.world());
        assert!(
            scene.items.len() >= 6,
            "expected base + arm + gripper links + ground, got {}",
            scene.items.len()
        );
    }

    #[test]
    fn urdf_scene_can_be_rebuilt_for_hot_reload() {
        let scene_path = cart_minimal_scene_path();
        let profile = ViewerProfile::Urdf(scene_path.clone());
        assert_eq!(profile_scene_path(&profile), scene_path);
        let mut sim = load_sim(&profile).expect("load cart URDF scene");
        let seed = sim.world_seed();
        sim.reload_scene(&scene_path)
            .expect("reload cart URDF scene");
        assert_eq!(sim.world_seed(), seed);
        assert!(matches!(sim, ViewerSim::UrdfScene(_)));
    }
}
