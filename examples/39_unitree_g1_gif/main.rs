//! Renders the official Unitree G1 23-DoF URDF moving under RNE physics.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_scene_path, UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_math::Vec3;
use rne_render::{Camera, MeshRenderCache, RenderBackend, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const FRAME_COUNT: usize = 36;
const STEPS_PER_FRAME: u64 = 5;
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

    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_scene_path()).expect("load G1");
    sim.configure_position_motors(55.0, 10.0, 88.0);
    for _ in 0..120 {
        sim.step_joint_position_targets(&g1_targets(0));
    }

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            sim.step_joint_position_targets(&g1_targets(frame as u64 * STEPS_PER_FRAME + substep));
        }
        let mut scene = build_visual_render_scene(sim.world());
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
        let obs = sim.observe();
        let orbit = CameraOrbit {
            focus: Vec3::new(obs.base_x_m, obs.base_y_m + 0.05, obs.base_z_m),
            yaw_rad: -0.72 + frame as f64 * 0.004,
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
    image::open(frames_dir.join("frame-009.png"))
        .expect("read poster frame")
        .save(media_dir.join("unitree-g1.png"))
        .expect("write G1 poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered official Unitree G1 media to {}",
        gif_path.display()
    );
}

fn g1_targets(step: u64) -> [UrdfJointPositionTarget<'static>; 23] {
    let phase = step as f64 * std::f64::consts::TAU / 90.0;
    let sway = phase.sin();
    let wave = (phase + 0.5).sin();
    [
        target("left_hip_pitch_link", -0.18 + 0.04 * sway),
        target("left_hip_roll_link", 0.03 * sway),
        target("left_hip_yaw_link", 0.0),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("left_ankle_roll_link", 0.0),
        target("right_hip_pitch_link", -0.18 - 0.04 * sway),
        target("right_hip_roll_link", 0.03 * sway),
        target("right_hip_yaw_link", 0.0),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
        target("right_ankle_roll_link", 0.0),
        target("torso_link", 0.10 * sway),
        target("left_shoulder_pitch_link", 0.18 * wave),
        target("left_shoulder_roll_link", 0.28 + 0.12 * sway),
        target("left_shoulder_yaw_link", 0.0),
        target("left_elbow_link", 0.42 + 0.20 * wave),
        target("left_wrist_roll_rubber_hand", 0.25 * sway),
        target("right_shoulder_pitch_link", -0.18 * wave),
        target("right_shoulder_roll_link", -0.28 + 0.12 * sway),
        target("right_shoulder_yaw_link", 0.0),
        target("right_elbow_link", 0.42 - 0.20 * wave),
        target("right_wrist_roll_rubber_hand", -0.25 * sway),
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
