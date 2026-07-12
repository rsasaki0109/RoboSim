//! Renders the official Unitree Go2 URDF stepping under RNE physics to README media.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_dynamic_scene_path, unitree_go2_trot_targets,
    UnitreeGo2GaitCommand, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 360;
const FRAME_COUNT: usize = 36;
const STEPS_PER_FRAME: u64 = 3;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-go2-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create Go2 frame directory");

    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(180.0, 18.0, 23.7);
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: 90,
        },
    );
    for _ in 0..120 {
        sim.step_joint_position_targets(&stand);
    }
    let start = sim.observe();
    println!("Go2 settled observation: {start:?}");

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_go2_trot_targets(
                step,
                UnitreeGo2GaitCommand::default(),
            ));
        }
        let mut scene = build_visual_render_scene(sim.world());
        scene
            .items
            .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
        append_checker_floor(&mut scene, start.base_x_m, start.base_z_m, 0.12);
        mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .expect("resolve official Go2 meshes");
        if frame == 0 {
            let meshes = scene
                .items
                .iter()
                .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
                .count();
            assert!(
                meshes >= 13,
                "expected official Go2 mesh visuals, got {meshes}"
            );
        }
        let orbit = CameraOrbit {
            focus: Vec3::new(start.base_x_m, start.base_y_m, start.base_z_m),
            yaw_rad: -0.82,
            pitch_rad: 1.05,
            distance_m: 1.10,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render Go2 frame");
        if frame == 0 {
            let unique = output
                .color
                .rgba8
                .chunks_exact(4)
                .collect::<std::collections::HashSet<_>>()
                .len();
            assert!(unique > 2, "Go2 frame should contain rendered geometry");
        }
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write Go2 frame");
    }

    let gif_path = media_dir.join("unitree-go2.gif");
    build_gif(&frames_dir, &gif_path).expect("encode Go2 gif");
    let poster = image::open(frames_dir.join("frame-009.png")).expect("read poster frame");
    poster
        .save(media_dir.join("unitree-go2.png"))
        .expect("write Go2 poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered official Unitree Go2 media to {}",
        gif_path.display()
    );
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

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg Go2 gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
