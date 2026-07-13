//! Runs or renders the articulated Unitree G1 29-DoF + Dex3 pick-and-place task.

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{build_visual_render_scene, Episode, UnitreeG1Dex3Action, UnitreeG1Dex3Episode};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::fs;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const INSET_WIDTH: u32 = 300;
const INSET_HEIGHT: u32 = 225;
const INSET_BORDER: u32 = 4;
const FRAME_COUNT: usize = 58;
const STEPS_PER_FRAME: usize = 4;
const CLEAR_COLOR: [f32; 4] = [0.025, 0.04, 0.07, 1.0];

fn main() {
    if std::env::args().any(|arg| arg == "--gif") {
        render_gif();
        return;
    }
    run_headless();
}

fn run_headless() {
    let mut episode = UnitreeG1Dex3Episode::new(Default::default()).expect("load G1 Dex3 episode");
    let mut total_reward = 0.0;
    let mut last_phase = None;
    loop {
        let step = episode.step(UnitreeG1Dex3Action { advance: true });
        total_reward += step.reward;
        if last_phase != Some(step.observation.phase) || step.is_done() {
            println!(
                "step {:3}: phase={:?} height={:.3}m gap={:.3}m span={:.3}m center={:.3}m opposition={:.2} stable={} dual={} grasped={} placed={}",
                episode.step_in_episode(),
                step.observation.phase,
                step.observation.part_position_m[1],
                step.observation.pinch_gap_m,
                step.observation.contact_span_m,
                step.observation.contact_center_error_m,
                step.observation.contact_opposition,
                step.observation.stable_contact_steps,
                step.observation.dual_contact,
                step.observation.grasped,
                step.observation.placed,
            );
            last_phase = Some(step.observation.phase);
        }
        if step.is_done() {
            assert!(step.terminated, "Dex3 task must succeed before truncation");
            println!(
                "G1 Dex3 pick-and-place complete: total_reward={total_reward:.3}, placed={}",
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
    let frames_dir = media_dir.join("unitree-g1-dex3-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 Dex3 frame directory");

    let mut episode = UnitreeG1Dex3Episode::new(Default::default()).expect("G1 Dex3 episode");
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let inset_camera = Camera::new(INSET_WIDTH, INSET_HEIGHT, std::f64::consts::FRAC_PI_6);
    let mesh_roots = episode.simulation().mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        let mut step = None;
        for _ in 0..STEPS_PER_FRAME {
            step = Some(episode.step(UnitreeG1Dex3Action { advance: true }));
        }
        let mut scene = build_visual_render_scene(episode.simulation().world());
        scene.items.retain(|item| {
            !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
        });
        append_checker_floor(&mut scene, 0.12);
        mesh_cache
            .resolve_scene(&mut scene, &mesh_root_refs)
            .expect("resolve official G1 Dex3 meshes");
        let overview_orbit = CameraOrbit {
            focus: Vec3::new(0.25, 0.90, 0.18),
            yaw_rad: 0.78,
            pitch_rad: 1.18,
            distance_m: 1.25,
        };
        let mut output = backend
            .render_scene_camera(
                &camera,
                &overview_orbit.camera_transform(),
                &scene,
                CLEAR_COLOR,
            )
            .expect("render G1 Dex3 frame");
        let observation = step.as_ref().expect("episode step").observation;
        let palm = episode
            .simulation()
            .named_translation_m("right_hand_palm_link")
            .expect("right Dex3 palm");
        let part = observation.part_position_m;
        let follow_part = if observation.grasped { 0.55 } else { 0.65 };
        let inset_focus = Vec3::new(
            palm.0 * (1.0 - follow_part) + part[0] * follow_part,
            palm.1 * (1.0 - follow_part) + part[1] * follow_part,
            palm.2 * (1.0 - follow_part) + part[2] * follow_part,
        );
        let inset_orbit = CameraOrbit {
            focus: inset_focus,
            yaw_rad: 0.78,
            pitch_rad: 1.12,
            distance_m: 0.42,
        };
        let inset_camera_transform = inset_orbit.camera_transform();
        let marker_offset_m =
            (inset_camera_transform.translation - inset_focus).normalize_or_zero() * 0.018;
        let mut inset_scene = scene.clone();
        inset_scene.items.retain(|item| {
            matches!(item.shape, VisualShape::Mesh { .. })
                || (item.transform.scale.x <= 0.05
                    && item.transform.scale.y <= 0.05
                    && item.transform.scale.z <= 0.05)
        });
        append_contact_markers(
            &mut inset_scene,
            episode.simulation(),
            observation.dual_contact,
            marker_offset_m,
        );
        let inset = backend
            .render_scene_camera(
                &inset_camera,
                &inset_camera_transform,
                &inset_scene,
                CLEAR_COLOR,
            )
            .expect("render G1 Dex3 contact inset");
        composite_inset(
            &mut output.color.rgba8,
            output.color.width,
            output.color.height,
            &inset.color.rgba8,
            inset.color.width,
            inset.color.height,
            if observation.grasped {
                [40, 245, 105, 255]
            } else {
                [20, 205, 225, 255]
            },
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write G1 Dex3 frame");
        if frame + 1 == FRAME_COUNT {
            assert!(step.expect("episode step").terminated);
        }
    }

    let gif_path = media_dir.join("unitree-g1-dex3.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 Dex3 gif");
    build_poster(&gif_path, &media_dir.join("unitree-g1-dex3.png"))
        .expect("extract G1 Dex3 poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!("rendered G1 Dex3 media to {}", gif_path.display());
}

fn append_contact_markers(
    scene: &mut RenderScene,
    sim: &rne_ai::UrdfSceneSim,
    dual_contact: bool,
    marker_offset_m: Vec3,
) {
    for (name, idle_color) in [
        ("right_dex3_thumb_contact_sensor", [1.0, 0.28, 0.06, 1.0]),
        ("right_dex3_index_contact_sensor", [0.05, 0.62, 1.0, 1.0]),
    ] {
        let (x, y, z) = sim
            .named_translation_m(name)
            .expect("configured Dex3 contact sensor");
        scene.items.push(RenderSceneItem {
            transform: Transform3 {
                translation: Vec3::new(x, y, z) + marker_offset_m,
                rotation: rne_math::Quat::IDENTITY,
                scale: Vec3::splat(if dual_contact { 0.012 } else { 0.010 }),
            },
            shape: VisualShape::Sphere { radius_m: 1.0 },
            color_rgba: if dual_contact {
                [0.15, 1.0, 0.35, 1.0]
            } else {
                idle_color
            },
            mesh: None,
        });
    }
}

fn composite_inset(
    base: &mut [u8],
    base_width: u32,
    base_height: u32,
    inset: &[u8],
    inset_width: u32,
    inset_height: u32,
    border_rgba: [u8; 4],
) {
    let x0 = base_width - inset_width - INSET_BORDER * 3;
    let y0 = INSET_BORDER * 3;
    for y in y0 - INSET_BORDER..y0 + inset_height + INSET_BORDER {
        for x in x0 - INSET_BORDER..x0 + inset_width + INSET_BORDER {
            let index = ((y * base_width + x) * 4) as usize;
            base[index..index + 4].copy_from_slice(&border_rgba);
        }
    }
    for y in 0..inset_height {
        let base_start = (((y0 + y) * base_width + x0) * 4) as usize;
        let inset_start = (y * inset_width * 4) as usize;
        let byte_count = (inset_width * 4) as usize;
        base[base_start..base_start + byte_count]
            .copy_from_slice(&inset[inset_start..inset_start + byte_count]);
    }
    debug_assert_eq!(base.len(), (base_width * base_height * 4) as usize);
}

fn append_checker_floor(scene: &mut RenderScene, tile_m: f64) {
    for row in -6..=6 {
        for column in -6..=6 {
            let color = if (row + column) & 1 == 0 {
                [0.10, 0.14, 0.20, 1.0]
            } else {
                [0.045, 0.065, 0.10, 1.0]
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
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 Dex3 gif encode failed"))
}

fn build_poster(gif_path: &Path, poster_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            &gif_path.to_string_lossy(),
            "-vf",
            "select=eq(n\\,40)",
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
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 Dex3 poster extraction failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
