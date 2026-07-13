//! Runs or renders the fixed-base Unitree G1 contact-grasp parts handling task.

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    append_task_marker_overlay, build_visual_render_scene, Episode, UnitreeG1PartsAction,
    UnitreeG1PartsEpisode,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::fs;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const FRAME_COUNT: usize = 60;
const STEPS_PER_FRAME: usize = 4;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

fn main() {
    if std::env::args().any(|arg| arg == "--gif") {
        render_gif();
    } else {
        run_headless();
    }
}

fn run_headless() {
    let mut episode =
        UnitreeG1PartsEpisode::new(Default::default()).expect("load fixed-base G1 parts episode");
    let mut total_reward = 0.0;
    loop {
        let step = episode.step(UnitreeG1PartsAction { advance: true });
        total_reward += step.reward;
        if episode.step_in_episode() == 1
            || episode.step_in_episode().is_multiple_of(30)
            || step.is_done()
        {
            println!(
                "step {:3}: phase={:?} height={:.3} m max={:.3} m speed={:.3} m/s place_error={:.3} m contact={} grasped={} reward={:.3}",
                episode.step_in_episode(),
                step.observation.phase,
                step.observation.part_height_m,
                step.observation.max_part_height_m,
                step.observation.part_speed_m_s,
                step.observation.place_distance_m,
                step.observation.hand_contact,
                step.observation.grasped,
                step.reward,
            );
        }
        if step.is_done() {
            assert!(step.terminated, "parts task must succeed before truncation");
            println!(
                "G1 parts pick-and-place complete: total_reward={total_reward:.3}, placed={}",
                step.observation.placed
            );
            break;
        }
    }
}

fn render_gif() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-parts-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 parts frame directory");

    let mut episode = UnitreeG1PartsEpisode::new(Default::default()).expect("parts episode");
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots = episode.simulation().mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        let mut step = None;
        for _ in 0..STEPS_PER_FRAME {
            step = Some(episode.step(UnitreeG1PartsAction { advance: true }));
        }
        let mut scene = build_visual_render_scene(episode.simulation().world());
        scene.items.retain(|item| {
            !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
        });
        append_checker_floor(&mut scene, 0.12);
        append_task_marker_overlay(&mut scene, episode.simulation().world());
        mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .expect("resolve official G1 and OBJ rack meshes");
        let orbit = CameraOrbit {
            focus: Vec3::new(0.16, 0.82, 0.16),
            yaw_rad: -0.72,
            pitch_rad: 1.22,
            distance_m: 1.75,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render G1 parts frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write G1 parts frame");
        if frame + 1 == FRAME_COUNT {
            assert!(step.expect("episode step").terminated);
        }
    }

    let gif_path = media_dir.join("unitree-g1-parts.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 parts gif");
    image::open(frames_dir.join("frame-035.png"))
        .expect("read G1 parts poster")
        .save(media_dir.join("unitree-g1-parts.png"))
        .expect("write G1 parts poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!("rendered G1 parts media to {}", gif_path.display());
}

fn append_checker_floor(scene: &mut RenderScene, tile_m: f64) {
    for row in -6..=6 {
        for column in -6..=6 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(column as f64 * tile_m, -0.008, row as f64 * tile_m),
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
            "fps=12,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 parts gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
