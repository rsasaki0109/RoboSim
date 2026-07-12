//! Renders the official Unitree G1 23-DoF URDF moving under RNE physics.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_inspection_targets,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const FRAME_COUNT: usize = 36;
const STEPS_PER_FRAME: u64 = 3;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 frame directory");

    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("load dynamic G1");
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let stand = standing_targets();
    for _ in 0..120 {
        sim.step_joint_position_targets(&stand);
    }
    let start = sim.observe();

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_inspection_targets(step));
        }
        let mut scene = build_visual_render_scene(sim.world());
        scene.items.retain(|item| {
            !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
        });
        append_checker_floor(&mut scene, start.base_x_m, start.base_z_m, 0.10);
        append_inspection_station(&mut scene, start.base_x_m, start.base_z_m);
        mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .expect("resolve official G1 meshes");
        if frame == 0 {
            let meshes = scene
                .items
                .iter()
                .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
                .count();
            assert!(
                meshes >= 20,
                "expected official G1 mesh visuals, got {meshes}"
            );
        }
        let orbit = CameraOrbit {
            focus: Vec3::new(start.base_x_m, start.base_y_m + 0.05, start.base_z_m),
            yaw_rad: -0.72,
            pitch_rad: 1.25,
            distance_m: 1.75,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render G1 frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write G1 frame");
    }

    let gif_path = media_dir.join("unitree-g1.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 gif");
    image::open(frames_dir.join("frame-030.png"))
        .expect("read poster frame")
        .save(media_dir.join("unitree-g1.png"))
        .expect("write G1 poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered official Unitree G1 media to {}",
        gif_path.display()
    );
}

fn append_inspection_station(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64) {
    append_box(
        scene,
        Vec3::new(center_x_m + 1.00, 0.59, center_z_m - 0.30),
        Vec3::new(0.22, 1.18, 0.22),
        [0.12, 0.16, 0.22, 1.0],
    );
    append_box(
        scene,
        Vec3::new(center_x_m + 0.95, 1.08, center_z_m - 0.30),
        Vec3::new(0.025, 0.25, 0.16),
        [0.06, 0.30, 0.38, 1.0],
    );
    append_box(
        scene,
        Vec3::new(center_x_m + 0.93, 1.11, center_z_m - 0.30),
        Vec3::new(0.018, 0.055, 0.055),
        [0.10, 0.95, 0.45, 1.0],
    );
}

fn append_box(scene: &mut RenderScene, translation: Vec3, size_m: Vec3, color: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: rne_math::Quat::IDENTITY,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
    });
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    for row in -6..=6 {
        for column in -6..=6 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(
                        center_x_m + column as f64 * tile_m,
                        -0.008,
                        center_z_m + row as f64 * tile_m,
                    ),
                    rotation: rne_math::Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: color,
                mesh: None,
            });
        }
    }
}

fn standing_targets() -> [UrdfJointPositionTarget<'static>; 6] {
    [
        target("left_hip_pitch_link", -0.18),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
    ]
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y", "-framerate", "12", "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
