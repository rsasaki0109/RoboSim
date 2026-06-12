//! Interactive diff-drive viewer with keyboard teleop, scene assets, and hot reload.
//!
//! Controls:
//! - W / S: drive forward / backward
//! - A / D: turn left / right
//! - Left / Right: orbit camera
//! - Up / Down: zoom camera
//! - Escape: quit
//!
//! Usage:
//!   cargo run -p interactive_viewer --example 14_interactive_viewer
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- assets/scenes/episode_diff_drive.rne.scene.toml
//!   cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke
//!
//! Edit the scene or referenced robot files while running; the viewer reloads automatically.

use rne_ai::{DiffDriveAction, DiffDriveSim};
use rne_assets::AssetHotReloader;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::Collider;
use rne_render::{
    hash_depth_f32, hash_rgba8, Camera, RenderBackend, RenderScene, Visual, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, WgpuRenderBackend};
use rne_robot::DiffDriveSpawned;
use rne_world::Transform3 as WorldTransform3;
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

    if smoke || env::var("RNE_SKIP_GPU").is_ok() {
        run_smoke(smoke, &default_scene_path());
        return;
    }

    let scene_path = scene_path_from_args();
    let event_loop = EventLoop::new().expect("create event loop");
    let mut app = App::new(scene_path);
    event_loop.run_app(&mut app).expect("run viewer");
}

fn default_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/episode_diff_drive.rne.scene.toml")
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

    let scene = build_diff_drive_render_scene(sim.world(), sim.robot());
    let orbit = CameraOrbit {
        focus: robot_focus(&sim),
        ..CameraOrbit::default()
    };
    let camera = Camera::new(320, 240, std::f64::consts::FRAC_PI_4);
    let view = orbit.camera_transform();

    let output = backend
        .render_scene_camera(&camera, &view, &scene, CLEAR_COLOR)
        .expect("smoke render");

    println!(
        "interactive viewer smoke{}: scene={} seed={} items={} color_hash={:#018x} depth_hash={:#018x} base_x={:.2} m",
        if explicit { "" } else { " (RNE_SKIP_GPU fallback)" },
        scene_path.display(),
        sim.world_seed(),
        scene.items.len(),
        hash_rgba8(&output.color.rgba8),
        hash_depth_f32(&output.depth.depth_m),
        sim.observe().base_x_m
    );

    if scene.items.is_empty() || sim.observe().base_x_m <= 0.0 {
        std::process::exit(1);
    }
}

struct App {
    scene_path: PathBuf,
    window: Option<Arc<Window>>,
    viewer: Option<InteractiveViewer>,
    sim: Option<DiffDriveSim>,
    hot_reloader: Option<AssetHotReloader>,
    reload_count: u32,
    orbit: CameraOrbit,
    pressed: HashSet<KeyCode>,
}

impl App {
    fn new(scene_path: PathBuf) -> Self {
        Self {
            scene_path,
            window: None,
            viewer: None,
            sim: None,
            hot_reloader: None,
            reload_count: 0,
            orbit: CameraOrbit::default(),
            pressed: HashSet::new(),
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

        let sim = match DiffDriveSim::from_scene_path(&self.scene_path) {
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
        println!(
            "loaded scene {} (seed={}, robots={})",
            self.scene_path.display(),
            sim.world_seed(),
            sim.robots().len()
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

        let scene = build_diff_drive_render_scene(sim.world(), sim.robot());
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
        self.orbit.focus = robot_focus(sim);
        println!(
            "reloaded scene {} (#{}) seed={}",
            self.scene_path.display(),
            self.reload_count,
            sim.world_seed()
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

fn build_diff_drive_render_scene(world: &World, robot: &DiffDriveSpawned) -> RenderScene {
    let drive = robot.drive;
    let mut scene = RenderScene::new();

    scene.items.push(render_item(
        world,
        robot.base_link,
        VisualShape::Box {
            size_m: base_size_m(world, robot.base_link),
        },
        [0.35, 0.55, 0.95, 1.0],
    ));

    for wheel in [robot.left_wheel, robot.right_wheel] {
        if wheel == robot.base_link {
            continue;
        }
        scene.items.push(render_item(
            world,
            wheel,
            VisualShape::Cylinder {
                radius_m: drive.wheel_radius_m,
                length_m: drive.wheel_radius_m * 0.6,
            },
            [0.2, 0.2, 0.2, 1.0],
        ));
    }

    scene.items.push(RenderScene::item_from_visual(
        WorldTransform3::from_translation_rotation(Vec3::new(0.0, -0.01, 0.0), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(40.0, 0.02, 40.0),
        },
        [0.25, 0.28, 0.32, 1.0],
        WorldTransform3::IDENTITY,
    ));

    scene
}

fn render_item(
    world: &World,
    entity: rne_ecs::Entity,
    shape: VisualShape,
    color_rgba: [f32; 4],
) -> rne_render::RenderSceneItem {
    let world_transform = world
        .get::<WorldTransform3>(entity)
        .copied()
        .unwrap_or_default();
    let local_offset = world
        .get::<Visual>(entity)
        .map(|visual| visual.local_offset)
        .unwrap_or_default();
    RenderScene::item_from_visual(world_transform, shape, color_rgba, local_offset)
}

fn base_size_m(world: &World, base_link: rne_ecs::Entity) -> Vec3 {
    world
        .get::<Collider>(base_link)
        .and_then(|collider| match collider.shape {
            rne_physics::ColliderShape::Cuboid { half_extents_m } => Some(half_extents_m * 2.0),
            _ => None,
        })
        .unwrap_or_else(|| Vec3::new(0.5, 0.3, 0.4))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_scene_path_exists() {
        assert!(default_scene_path().is_file());
    }
}
