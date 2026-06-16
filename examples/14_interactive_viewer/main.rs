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
//! - P: toggle wrist camera PiP (manipulator profiles only)
//! - Escape: quit
//!
//! Usage:
//!   cargo run -p interactive_viewer --example 14_interactive_viewer
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- assets/scenes/mesh_diff_drive.rne.scene.toml
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator-mobile
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --manipulator-lift
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke

use rne_ai::{
    append_lidar_overlay, build_diff_drive_render_scene, build_visual_render_scene,
    mm_lift_scene_path, mm_minimal_scene_path, mm_mobile_scene_path, DiffDriveAction, DiffDriveSim,
    MobileManipulatorAction, MobileManipulatorSim,
};
use rne_assets::AssetHotReloader;
use rne_math::Vec3;
use rne_render::{hash_depth_f32, hash_rgba8, Camera, MeshRenderCache, RenderBackend, VisualShape};
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, WgpuRenderBackend};
use std::collections::HashSet;
use std::env;
use std::path::{Path, PathBuf};
use std::sync::Arc;
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

#[derive(Clone, Debug)]
enum ViewerProfile {
    DiffDriveScene(PathBuf),
    ManipulatorFixed(PathBuf),
    ManipulatorMobile(PathBuf),
    ManipulatorLift(PathBuf),
}

enum ViewerSim {
    DiffDrive(DiffDriveSim),
    Manipulator(MobileManipulatorSim),
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
        }
    }

    fn wrist_camera_pip(&self) -> Option<(Vec<u8>, u32, u32)> {
        match self {
            Self::Manipulator(sim) => sim
                .latest_wrist_camera()
                .map(|image| (image.rgba8, image.width, image.height)),
            Self::DiffDrive(_) => None,
        }
    }

    fn wrist_camera_enabled(&self) -> bool {
        matches!(self, Self::Manipulator(sim) if sim.wrist_camera_enabled())
    }

    fn build_scene(&self, show_lidar: bool) -> rne_render::RenderScene {
        match self {
            Self::DiffDrive(sim) => {
                let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
                if show_lidar {
                    append_lidar_overlay(&mut scene, sim.world(), sim.data_bus());
                }
                scene
            }
            Self::Manipulator(sim) => build_visual_render_scene(sim.world()),
        }
    }

    fn mesh_roots(&self) -> Vec<PathBuf> {
        match self {
            Self::DiffDrive(sim) => sim.mesh_package_roots().to_vec(),
            Self::Manipulator(_) => Vec::new(),
        }
    }

    fn supports_hot_reload(&self) -> bool {
        matches!(self, Self::DiffDrive(_))
    }

    fn reload_scene(&mut self, scene_path: &Path) -> Result<(), String> {
        match self {
            Self::DiffDrive(sim) => sim
                .reload_scene()
                .map_err(|error| format!("reload scene: {error}")),
            Self::Manipulator(_) => {
                let _ = scene_path;
                Ok(())
            }
        }
    }

    fn world_seed(&self) -> u64 {
        match self {
            Self::DiffDrive(sim) => sim.world_seed(),
            Self::Manipulator(_) => 0,
        }
    }

    fn smoke_base_x(&self) -> f64 {
        match self {
            Self::DiffDrive(sim) => sim.observe().base_x_m,
            Self::Manipulator(sim) => sim.observe().base_x_m,
        }
    }

    fn smoke_lidar_hits(&self) -> usize {
        match self {
            Self::DiffDrive(sim) => {
                let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
                append_lidar_overlay(&mut scene, sim.world(), sim.data_bus()).hit_markers
            }
            Self::Manipulator(_) => 0,
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
    let mut app = App::new(profile);
    event_loop.run_app(&mut app).expect("run viewer");
}

fn default_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml")
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
    let scene_path = scene_arg.unwrap_or_else(default_scene_path);
    ViewerProfile::DiffDriveScene(scene_path)
}

fn load_sim(profile: &ViewerProfile) -> Result<ViewerSim, String> {
    match profile {
        ViewerProfile::DiffDriveScene(path) => DiffDriveSim::from_scene_path(path)
            .map(ViewerSim::DiffDrive)
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorFixed(path) => MobileManipulatorSim::from_scene_path(path)
            .map(ViewerSim::Manipulator)
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorMobile(path) => MobileManipulatorSim::from_scene_path(path)
            .map(ViewerSim::Manipulator)
            .map_err(|error| error.to_string()),
        ViewerProfile::ManipulatorLift(path) => MobileManipulatorSim::from_scene_path(path)
            .map(ViewerSim::Manipulator)
            .map_err(|error| error.to_string()),
    }
}

fn profile_label(profile: &ViewerProfile) -> String {
    match profile {
        ViewerProfile::DiffDriveScene(path) => path.display().to_string(),
        ViewerProfile::ManipulatorFixed(path) => format!("mm_minimal ({})", path.display()),
        ViewerProfile::ManipulatorMobile(path) => format!("mm_mobile ({})", path.display()),
        ViewerProfile::ManipulatorLift(path) => format!("mm_lift ({})", path.display()),
    }
}

fn run_smoke(explicit: bool, profile: &ViewerProfile) {
    if env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping interactive viewer smoke");
        return;
    }

    let mut sim = load_sim(profile).expect("load viewer simulation");
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

    let mut scene = sim.build_scene(matches!(profile, ViewerProfile::DiffDriveScene(_)));
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
    }
    keys
}

struct App {
    profile: ViewerProfile,
    window: Option<Arc<Window>>,
    viewer: Option<InteractiveViewer>,
    sim: Option<ViewerSim>,
    hot_reloader: Option<AssetHotReloader>,
    mesh_cache: MeshRenderCache,
    reload_count: u32,
    orbit: CameraOrbit,
    pressed: HashSet<KeyCode>,
    show_lidar: bool,
    show_wrist_camera: bool,
    last_hud: String,
}

impl App {
    fn new(profile: ViewerProfile) -> Self {
        Self {
            profile,
            window: None,
            viewer: None,
            sim: None,
            hot_reloader: None,
            mesh_cache: MeshRenderCache::new(),
            reload_count: 0,
            orbit: CameraOrbit::default(),
            pressed: HashSet::new(),
            show_lidar: true,
            show_wrist_camera: true,
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

        let hot_reloader = if let ViewerProfile::DiffDriveScene(path) = &self.profile {
            match AssetHotReloader::load(path) {
                Ok(reloader) => Some(reloader),
                Err(error) => {
                    eprintln!("failed to watch scene dependencies: {error}");
                    event_loop.exit();
                    return;
                }
            }
        } else {
            None
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
                    && matches!(
                        self.profile,
                        ViewerProfile::ManipulatorFixed(_)
                            | ViewerProfile::ManipulatorMobile(_)
                            | ViewerProfile::ManipulatorLift(_)
                    )
                {
                    self.show_wrist_camera = !self.show_wrist_camera;
                    println!(
                        "wrist camera pip {}",
                        if self.show_wrist_camera {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
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

        let sim = self.sim.as_mut().ok_or("simulation not ready")?;
        sim.step(&self.pressed);
        self.orbit.focus = sim.focus();

        let hud = sim.hud_line();
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

        let mut scene = sim.build_scene(self.show_lidar);
        let mesh_roots = sim.mesh_roots();
        let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
        self.mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .map_err(|error| error.to_string())?;

        let view = self.orbit.camera_transform();
        let viewer = self.viewer.as_mut().ok_or("viewer not ready")?;
        let pip = if self.show_wrist_camera {
            sim.wrist_camera_pip()
        } else {
            None
        };
        viewer
            .render_with_pip(&view, &scene, CLEAR_COLOR, pip)
            .map_err(|error| error.to_string())
    }

    fn poll_hot_reload(&mut self) -> Result<(), String> {
        let Some(reloader) = self.hot_reloader.as_mut() else {
            return Ok(());
        };
        if !reloader.poll().map_err(|error| error.to_string())? {
            return Ok(());
        }

        let sim = self.sim.as_mut().ok_or("simulation not ready")?;
        if !sim.supports_hot_reload() {
            return Ok(());
        }
        if let ViewerProfile::DiffDriveScene(path) = &self.profile {
            sim.reload_scene(path)?;
        }
        self.reload_count += 1;
        self.mesh_cache.clear();
        self.orbit.focus = sim.focus();
        println!(
            "reloaded scene (#{}) seed={} mesh_roots={}",
            self.reload_count,
            sim.world_seed(),
            sim.mesh_roots().len()
        );
        Ok(())
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
}
