//! Interactive diff-drive viewer with keyboard teleop, scene assets, URDF mesh visuals, and hot reload.
//!
//! Controls:
//! - W / S: drive forward / backward
//! - A / D: turn left / right
//! - Left / Right: orbit camera
//! - Up / Down: zoom camera
//! - L: toggle LiDAR hit overlay
//! - Escape: quit
//!
//! Usage:
//!   cargo run -p interactive_viewer --example 14_interactive_viewer
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- assets/scenes/mesh_diff_drive.rne.scene.toml
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke
//!
//! Edit the scene or referenced robot files while running; the viewer reloads automatically.

use rne_ai::{append_lidar_overlay, build_diff_drive_render_scene, DiffDriveAction, DiffDriveSim};
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

fn main() {
    let smoke = env::args().any(|arg| arg == "--smoke") || env::var("RNE_VIEWER_SMOKE").is_ok();
    let scene_path = scene_path_from_args();

    if smoke || env::var("RNE_SKIP_GPU").is_ok() {
        run_smoke(smoke, &scene_path);
        return;
    }

    let event_loop = EventLoop::new().expect("create event loop");
    let mut app = App::new(scene_path);
    event_loop.run_app(&mut app).expect("run viewer");
}

fn default_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml")
}

fn scene_path_from_args() -> PathBuf {
    env::args()
        .skip(1)
        .find(|arg| !arg.starts_with('-'))
        .map(PathBuf::from)
        .unwrap_or_else(default_scene_path)
}

fn run_smoke(explicit: bool, scene_path: &Path) {
    if env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping interactive viewer smoke");
        return;
    }

    let mut sim = DiffDriveSim::from_scene_path(scene_path).expect("load scene");
    sim.enable_lidar_demo();
    for _ in 0..60 {
        sim.step_action(DiffDriveAction::forward(DRIVE_SPEED_RAD_S));
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };

    let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
    let lidar_stats = append_lidar_overlay(&mut scene, sim.world(), sim.data_bus());
    let mesh_items = count_mesh_items(&scene);
    let mut mesh_cache = MeshRenderCache::new();
    let mesh_roots = mesh_roots_for_sim(&sim);
    mesh_cache
        .resolve_scene(&mut scene, &mesh_roots)
        .expect("resolve mesh assets");

    let orbit = CameraOrbit {
        focus: robot_focus(&sim),
        yaw_rad: -0.09,
        pitch_rad: 0.52,
        distance_m: 3.6,
    };
    let camera = Camera::new(640, 360, std::f64::consts::FRAC_PI_4);
    let view = orbit.camera_transform();

    let output = backend
        .render_scene_camera(&camera, &view, &scene, CLEAR_COLOR)
        .expect("smoke render");

    println!(
        "interactive viewer smoke{}: scene={} seed={} items={} mesh_items={} lidar_hits={} color_hash={:#018x} depth_hash={:#018x} base_x={:.2} m",
        if explicit { "" } else { " (RNE_SKIP_GPU fallback)" },
        scene_path.display(),
        sim.world_seed(),
        scene.items.len(),
        mesh_items,
        lidar_stats.hit_markers,
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        sim.observe().base_x_m
    );

    if scene.items.is_empty() || sim.observe().base_x_m <= 0.0 {
        std::process::exit(1);
    }
    if lidar_stats.hit_markers < 4 {
        eprintln!(
            "interactive viewer smoke expected lidar hits, got {}",
            lidar_stats.hit_markers
        );
        std::process::exit(1);
    }

    let center = (output.color.height / 2 * output.color.width + output.color.width / 2) as usize;
    let center_depth = output.depth.depth_m[center];
    if center_depth >= camera.far_m as f32 {
        eprintln!("interactive viewer smoke render invalid (center_depth={center_depth:.2} m)");
        std::process::exit(1);
    }
}

struct App {
    scene_path: PathBuf,
    window: Option<Arc<Window>>,
    viewer: Option<InteractiveViewer>,
    sim: Option<DiffDriveSim>,
    hot_reloader: Option<AssetHotReloader>,
    mesh_cache: MeshRenderCache,
    reload_count: u32,
    orbit: CameraOrbit,
    pressed: HashSet<KeyCode>,
    show_lidar: bool,
}

impl App {
    fn new(scene_path: PathBuf) -> Self {
        Self {
            scene_path,
            window: None,
            viewer: None,
            sim: None,
            hot_reloader: None,
            mesh_cache: MeshRenderCache::new(),
            reload_count: 0,
            orbit: CameraOrbit::default(),
            pressed: HashSet::new(),
            show_lidar: true,
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let title = format!(
            "RNE Interactive Viewer — {}",
            self.scene_path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("scene")
        );
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

        let mut sim = match DiffDriveSim::from_scene_path(&self.scene_path) {
            Ok(sim) => sim,
            Err(error) => {
                eprintln!(
                    "failed to load scene {}: {error}",
                    self.scene_path.display()
                );
                event_loop.exit();
                return;
            }
        };
        sim.enable_lidar_demo();
        let hot_reloader = match AssetHotReloader::load(&self.scene_path) {
            Ok(reloader) => reloader,
            Err(error) => {
                eprintln!(
                    "failed to watch scene dependencies for {}: {error}",
                    self.scene_path.display()
                );
                event_loop.exit();
                return;
            }
        };

        self.orbit.focus = robot_focus(&sim);
        self.mesh_cache.clear();
        println!(
            "loaded scene {} (seed={}, robots={}, mesh_roots={}, lidar=on)",
            self.scene_path.display(),
            sim.world_seed(),
            sim.robots().len(),
            sim.mesh_package_roots().len()
        );

        self.window = Some(window);
        self.viewer = Some(viewer);
        self.sim = Some(sim);
        self.hot_reloader = Some(hot_reloader);
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
                if physical == KeyCode::KeyL {
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

        let action = teleop_action(&self.pressed);
        let sim = self.sim.as_mut().ok_or("simulation not ready")?;
        self.orbit.focus = robot_focus(sim);
        sim.step_action(action);

        let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
        if self.show_lidar {
            append_lidar_overlay(&mut scene, sim.world(), sim.data_bus());
        }
        let mesh_roots = mesh_roots_for_sim(sim);
        self.mesh_cache
            .resolve_scene(&mut scene, &mesh_roots)
            .map_err(|error| error.to_string())?;

        let view = self.orbit.camera_transform();
        let viewer = self.viewer.as_mut().ok_or("viewer not ready")?;
        viewer
            .render(&view, &scene, CLEAR_COLOR)
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
        sim.reload_scene()
            .map_err(|error| format!("reload scene: {error}"))?;
        self.reload_count += 1;
        self.mesh_cache.clear();
        self.orbit.focus = robot_focus(sim);
        println!(
            "reloaded scene {} (#{}) seed={} mesh_roots={}",
            self.scene_path.display(),
            self.reload_count,
            sim.world_seed(),
            sim.mesh_package_roots().len()
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

fn teleop_action(keys: &HashSet<KeyCode>) -> DiffDriveAction {
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

fn robot_focus(sim: &DiffDriveSim) -> Vec3 {
    let obs = sim.observe();
    Vec3::new(obs.base_x_m, 0.25, obs.base_z_m)
}

fn mesh_roots_for_sim(sim: &DiffDriveSim) -> Vec<&Path> {
    sim.mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect()
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
}
