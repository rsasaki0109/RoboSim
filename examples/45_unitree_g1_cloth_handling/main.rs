//! Runs and captures deterministic Unitree G1 + Dex3 cloth handling.

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dex3_pick_targets, UnitreeG1Dex3HandCommand, UrdfSceneSim,
};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::fs;
use std::path::{Path, PathBuf};

const CLOTH_NAME: &str = "dex3_handling_cloth";
const PALM_NAME: &str = "right_hand_palm_link";
const THUMB_SENSOR_NAME: &str = "right_dex3_cloth_thumb_probe";
const INDEX_SENSOR_NAME: &str = "right_dex3_cloth_index_probe";
const LEFT_PALM_NAME: &str = "left_hand_palm_link";
const SETTLE_STEPS: u64 = 4;
const APPROACH_STEPS: u64 = 16;
const CLOSE_STEPS: u64 = 40;
const PINCH_STEPS: u64 = 24;
const LIFT_STEPS: u64 = 80;
const HOLD_STEPS: u64 = 20;
const OPEN_STEPS: u64 = 16;
const DROP_STEPS: u64 = 60;
const LIFT_START: u64 = APPROACH_STEPS + CLOSE_STEPS + PINCH_STEPS;
const RELEASE_STEP: u64 = LIFT_START + LIFT_STEPS + HOLD_STEPS;
const TOTAL_STEPS: u64 = RELEASE_STEP + OPEN_STEPS + DROP_STEPS;
const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const FRAME_COUNT: usize = 64;
const STEPS_PER_FRAME: usize = 4;
const CLEAR_COLOR: [f32; 4] = [0.025, 0.04, 0.07, 1.0];

fn main() {
    if std::env::args().any(|argument| argument == "--gif") {
        render_gif();
    } else {
        run_headless();
    }
}

struct ClothHandlingDemo {
    sim: UrdfSceneSim,
    step: u64,
    was_attached: bool,
    released: bool,
    initial_center_m: Vec3,
    max_center_y_m: f64,
    initial_left_palm_m: Vec3,
    max_left_palm_drift_m: f64,
}

impl ClothHandlingDemo {
    fn new() -> Self {
        let scene_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/unitree_g1_cloth_handling.rne.scene.toml");
        let mut sim = UrdfSceneSim::from_scene_path_with_fixed_links(
            &scene_path,
            &[
                "left_shoulder_pitch_link",
                "left_shoulder_roll_link",
                "left_shoulder_yaw_link",
                "left_elbow_link",
                "left_wrist_roll_link",
                "left_wrist_pitch_link",
                "left_wrist_yaw_link",
                "left_hand_palm_link",
                "left_hand_thumb_0_link",
                "left_hand_thumb_1_link",
                "left_hand_thumb_2_link",
                "left_hand_middle_0_link",
                "left_hand_middle_1_link",
                "left_hand_index_0_link",
                "left_hand_index_1_link",
            ],
        )
        .expect("load G1 cloth scene with inactive left arm fixed");
        assert!(sim.configure_deformable_solver(2, 4));
        configure_dex3(&mut sim);
        for _ in 0..SETTLE_STEPS {
            sim.step_joint_position_targets(&unitree_g1_dex3_pick_targets(
                0.0,
                0.0,
                UnitreeG1Dex3HandCommand { closure: 0.0 },
            ));
        }
        let initial_center_m = cloth_center_m(&sim);
        let initial_left_palm_m = named_position_m(&sim, LEFT_PALM_NAME);
        Self {
            sim,
            step: 0,
            was_attached: false,
            released: false,
            initial_center_m,
            max_center_y_m: initial_center_m.y,
            initial_left_palm_m,
            max_left_palm_drift_m: 0.0,
        }
    }

    fn advance(&mut self) {
        if self.step == RELEASE_STEP {
            self.released = self.sim.release_named_deformable(CLOTH_NAME);
        }
        let (approach, lift, closure) = command_at_step(self.step);
        self.sim
            .step_joint_position_targets(&unitree_g1_dex3_pick_targets(
                approach,
                lift,
                UnitreeG1Dex3HandCommand { closure },
            ));
        if !self.was_attached && closure >= 0.72 {
            self.was_attached = self
                .sim
                .try_attach_named_deformable_on_named_collider_contacts(
                    CLOTH_NAME,
                    PALM_NAME,
                    &[THUMB_SENSOR_NAME, INDEX_SENSOR_NAME],
                )
                .expect("valid Dex3 cloth contact grasp");
        }
        self.step += 1;
        self.max_center_y_m = self.max_center_y_m.max(cloth_center_m(&self.sim).y);
        self.max_left_palm_drift_m = self
            .max_left_palm_drift_m
            .max(named_position_m(&self.sim, LEFT_PALM_NAME).distance(self.initial_left_palm_m));
    }

    fn finish_assertions(&self) {
        let final_center = cloth_center_m(&self.sim);
        assert!(self.was_attached, "Dex3 probes must acquire the cloth");
        assert!(self.released, "script must release the acquired cloth");
        assert!(
            self.max_center_y_m >= self.initial_center_m.y + 0.08,
            "cloth must lift visibly: initial={:.3}m max={:.3}m",
            self.initial_center_m.y,
            self.max_center_y_m
        );
        assert!(
            final_center.y <= self.max_center_y_m - 0.04,
            "released cloth must descend: final={:.3}m max={:.3}m",
            final_center.y,
            self.max_center_y_m
        );
        assert!(
            self.max_left_palm_drift_m <= 0.06,
            "inactive left hand must remain still: drift={:.4}m",
            self.max_left_palm_drift_m
        );
    }
}

fn configure_dex3(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    for (name, max_force_nm) in [
        ("right_hand_thumb_0_link", 2.45),
        ("right_hand_thumb_1_link", 1.4),
        ("right_hand_thumb_2_link", 1.4),
        ("right_hand_middle_0_link", 1.4),
        ("right_hand_middle_1_link", 1.4),
        ("right_hand_index_0_link", 1.4),
        ("right_hand_index_1_link", 1.4),
    ] {
        assert!(sim.configure_named_position_motor(name, 40.0, 4.0, max_force_nm));
    }
    assert!(sim.add_named_child_box_sensor(
        "right_hand_thumb_2_link",
        THUMB_SENSOR_NAME,
        [0.026, 0.050, 0.026],
        [0.0, 0.026, 0.0],
    ));
    assert!(sim.add_named_child_box_sensor(
        "right_hand_index_1_link",
        INDEX_SENSOR_NAME,
        [0.050, 0.026, 0.026],
        [0.026, 0.0, 0.0],
    ));
}

fn command_at_step(step: u64) -> (f64, f64, f64) {
    if step < APPROACH_STEPS {
        ((step + 1) as f64 / APPROACH_STEPS as f64, 0.0, 0.0)
    } else if step < APPROACH_STEPS + CLOSE_STEPS {
        (
            1.0,
            0.0,
            (step - APPROACH_STEPS + 1) as f64 / CLOSE_STEPS as f64,
        )
    } else if step < LIFT_START {
        (1.0, 0.0, 1.0)
    } else if step < LIFT_START + LIFT_STEPS {
        (1.0, (step - LIFT_START + 1) as f64 / LIFT_STEPS as f64, 1.0)
    } else if step < RELEASE_STEP {
        (1.0, 1.0, 1.0)
    } else {
        (
            1.0,
            1.0,
            1.0 - ((step - RELEASE_STEP + 1) as f64 / OPEN_STEPS as f64).clamp(0.0, 1.0),
        )
    }
}

fn cloth_center_m(sim: &UrdfSceneSim) -> Vec3 {
    let body = sim
        .named_deformable_body(CLOTH_NAME)
        .expect("named cloth body");
    body.particles
        .iter()
        .map(|particle| particle.position_m)
        .sum::<Vec3>()
        / body.particles.len() as f64
}

fn named_position_m(sim: &UrdfSceneSim, name: &str) -> Vec3 {
    let (x, y, z) = sim.named_translation_m(name).expect("named scene position");
    Vec3::new(x, y, z)
}

fn run_headless() {
    let mut first = ClothHandlingDemo::new();
    let mut replay = ClothHandlingDemo::new();
    while first.step < TOTAL_STEPS {
        first.advance();
        replay.advance();
        assert_eq!(
            first
                .sim
                .named_deformable_body(CLOTH_NAME)
                .expect("first cloth"),
            replay
                .sim
                .named_deformable_body(CLOTH_NAME)
                .expect("replay cloth"),
            "cloth replay diverged at step {}",
            first.step
        );
    }
    first.finish_assertions();
    replay.finish_assertions();
    let body = first
        .sim
        .named_deformable_body(CLOTH_NAME)
        .expect("cloth body");
    println!(
        "G1 cloth handling complete: particles={} attached={} released={} max_height={:.3}m hash={:#018x}",
        body.particles.len(),
        first.was_attached,
        first.released,
        first.max_center_y_m,
        body.stable_state_hash(),
    );
}

fn render_gif() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-cloth-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 cloth frame directory");
    let mut demo = ClothHandlingDemo::new();
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots = demo.sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for _ in 0..STEPS_PER_FRAME {
            demo.advance();
        }
        let mut scene = build_visual_render_scene(demo.sim.world());
        scene.items.retain(|item| {
            !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
        });
        append_checker_floor(&mut scene, 0.12);
        append_cloth(
            &mut scene,
            &demo.sim,
            demo.sim.named_deformable_is_attached(CLOTH_NAME),
        );
        append_probe_markers(&mut scene, &demo.sim);
        mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .expect("resolve official G1 Dex3 meshes");
        let orbit = CameraOrbit {
            focus: Vec3::new(0.27, 0.93, 0.18),
            yaw_rad: 0.78,
            pitch_rad: 1.16,
            distance_m: 0.98,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render G1 cloth frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write G1 cloth frame");
    }
    demo.finish_assertions();
    let gif_path = media_dir.join("unitree-g1-cloth.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 cloth gif");
    build_poster(&gif_path, &media_dir.join("unitree-g1-cloth.png"))
        .expect("extract G1 cloth poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!("rendered G1 cloth media to {}", gif_path.display());
}

fn append_cloth(scene: &mut RenderScene, sim: &UrdfSceneSim, attached: bool) {
    let surface = sim
        .named_deformable_body(CLOTH_NAME)
        .expect("cloth body")
        .cloth_surface_mesh()
        .expect("cloth surface");
    scene.items.push(RenderScene::item_from_dynamic_mesh(
        TriangleMesh {
            positions: surface.positions,
            normals: surface.normals,
            indices: surface.indices,
        },
        if attached {
            [0.10, 0.72, 1.0, 1.0]
        } else {
            [0.08, 0.58, 0.92, 1.0]
        },
    ));
}

fn append_probe_markers(scene: &mut RenderScene, sim: &UrdfSceneSim) {
    for (name, color) in [
        (THUMB_SENSOR_NAME, [1.0, 0.30, 0.08, 1.0]),
        (INDEX_SENSOR_NAME, [0.10, 0.90, 0.35, 1.0]),
    ] {
        let (x, y, z) = sim.named_translation_m(name).expect("Dex3 cloth probe");
        scene.items.push(RenderSceneItem {
            transform: Transform3 {
                translation: Vec3::new(x, y, z),
                rotation: Quat::IDENTITY,
                scale: Vec3::splat(0.011),
            },
            shape: VisualShape::Sphere { radius_m: 1.0 },
            color_rgba: color,
            mesh: None,
        });
    }
}

fn append_checker_floor(scene: &mut RenderScene, tile_m: f64) {
    for row in -6..=6 {
        for column in -6..=6 {
            scene.items.push(RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(column as f64 * tile_m, -0.008, row as f64 * tile_m),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: if (row + column) & 1 == 0 {
                    [0.10, 0.14, 0.20, 1.0]
                } else {
                    [0.045, 0.065, 0.10, 1.0]
                },
                mesh: None,
            });
        }
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 cloth gif encode failed"))
}

fn build_poster(gif_path: &Path, poster_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            &gif_path.to_string_lossy(),
            "-vf",
            "select=eq(n\\,32)",
            "-frames:v",
            "1",
            "-update",
            "1",
            &poster_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 cloth poster extraction failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
