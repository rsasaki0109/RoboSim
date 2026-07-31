//! Renders the Go2 fall-versus-save comparison to README media.
//!
//! Both panels run the identical torque-limited trot under the identical
//! sustained flank push. The left robot is open loop, capsizes, and ends flat on
//! its side; the right robot feeds the measured lean back through two channels —
//! hip abduction plus differential leg-length extension — and rides the push out
//! standing. The weak motors are what makes the comparison honest: at the stiff
//! 23.7 N*m limit the scripted gait is passively stable and no controller is
//! needed (see `docs/DISTURBANCE_INJECTION.md`). The same numbers are pinned by
//! the `sustained_push_topples_weak_motor_trot_and_two_channel_feedback_saves_it`
//! test.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, Episode, UnitreeGo2Action, UnitreeGo2Episode,
    UnitreeGo2EpisodeConfig, UnitreeGo2Observation, UnitreeGo2Push, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const PANEL_WIDTH: u32 = 480;
const PANEL_HEIGHT: u32 = 360;
const BANNER_HEIGHT: u32 = 8;
const FRAME_COUNT: usize = 72;
const STEPS_PER_FRAME: u64 = 5;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

// The empirically probed fall/save boundary; the acceptance test
// `sustained_push_topples_weak_motor_trot_and_two_channel_feedback_saves_it`
// pins the same numbers, so this GIF cannot silently drift away from the tested
// physics.
const MOTOR_MAX_FORCE_N: f64 = 8.0;
const PUSH_STEP: u64 = 150;
const PUSH_TOTAL_TILT_RAD: f64 = 1.8;
const PUSH_DURATION_STEPS: u64 = 20;
const HIP_P_GAIN: f64 = 1.6;
const HIP_D_GAIN: f64 = 6.0;
const EXTENSION_P_GAIN: f64 = 2.5;
const EXTENSION_D_GAIN: f64 = 5.0;

fn episode() -> UnitreeGo2Episode {
    UnitreeGo2Episode::new(UnitreeGo2EpisodeConfig {
        max_steps: FRAME_COUNT as u64 * STEPS_PER_FRAME + 1,
        push: Some(UnitreeGo2Push {
            step: PUSH_STEP,
            roll_tilt_rad: PUSH_TOTAL_TILT_RAD,
            duration_steps: PUSH_DURATION_STEPS,
        }),
        motor_max_force_n: MOTOR_MAX_FORCE_N,
        ..Default::default()
    })
    .expect("load weak-motor Go2 episode")
}

fn feedback(previous_lean: &mut f64, observation: &UnitreeGo2Observation) -> UnitreeGo2Action {
    let lean = observation.base_relative_pitch_rad;
    let lean_rate = lean - *previous_lean;
    *previous_lean = lean;
    UnitreeGo2Action {
        roll_correction_rad: HIP_P_GAIN * lean + HIP_D_GAIN * lean_rate,
        lateral_extension_rad: -(EXTENSION_P_GAIN * lean + EXTENSION_D_GAIN * lean_rate),
        ..UnitreeGo2Action::default()
    }
}

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("go2-fall-vs-save-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create fall-vs-save frame directory");

    let mut open_episode = episode();
    let mut saved_episode = episode();
    let open_start = open_episode.sim().observe();
    let saved_start = saved_episode.sim().observe();
    let focus_open = Vec3::new(
        open_start.base_x_m,
        open_start.base_y_m,
        open_start.base_z_m,
    );
    let focus_saved = Vec3::new(
        saved_start.base_x_m,
        saved_start.base_y_m,
        saved_start.base_z_m,
    );
    let mut open_observation: Option<UnitreeGo2Observation> = None;
    let mut saved_observation: Option<UnitreeGo2Observation> = None;
    let mut saved_lean = 0.0_f64;

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = open_episode.sim().mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for _ in 0..STEPS_PER_FRAME {
            open_observation = Some(open_episode.step(UnitreeGo2Action::default()).observation);
            let action = match &saved_observation {
                Some(observation) => feedback(&mut saved_lean, observation),
                None => UnitreeGo2Action::default(),
            };
            saved_observation = Some(saved_episode.step(action).observation);
        }
        let left = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            open_episode.sim(),
            focus_open,
        );
        let right = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            saved_episode.sim(),
            focus_saved,
        );
        if frame == 0 {
            let unique = left
                .chunks_exact(4)
                .collect::<std::collections::HashSet<_>>()
                .len();
            assert!(unique > 2, "fall-vs-save frame should contain geometry");
        }
        let composite = composite_side_by_side(&left, &right);
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &composite,
            PANEL_WIDTH * 2,
            PANEL_HEIGHT + BANNER_HEIGHT,
        )
        .expect("write fall-vs-save frame");
    }

    // The GIF must show the physics the acceptance test pins: open loop flat on
    // its side, feedback braced on its feet.
    let tilt = |observation: &UnitreeGo2Observation| {
        observation
            .base_relative_pitch_rad
            .hypot(observation.base_relative_roll_rad)
    };
    let open_end = open_observation.expect("open-loop panel stepped");
    let saved_end = saved_observation.expect("feedback panel stepped");
    assert!(
        tilt(&open_end) > 1.3,
        "open-loop panel should end flat on its side, tilt {:.2}",
        tilt(&open_end)
    );
    assert!(
        tilt(&saved_end) < 0.55 && saved_end.base_y_m > 0.2,
        "feedback panel should end standing, tilt {:.2} height {:.3}",
        tilt(&saved_end),
        saved_end.base_y_m
    );

    let gif_path = media_dir.join("go2-fall-vs-save.gif");
    build_gif(&frames_dir, &gif_path).expect("encode fall-vs-save gif");
    let poster = image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read poster frame");
    poster
        .save(media_dir.join("go2-fall-vs-save.png"))
        .expect("write fall-vs-save poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!("rendered fall-vs-save media to {}", gif_path.display());
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    sim: &UrdfSceneSim,
    focus: Vec3,
) -> Vec<u8> {
    let mut scene = build_visual_render_scene(sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_checker_floor(&mut scene, focus.x, focus.z, 0.12);
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official Go2 meshes");
    // Low chase view along the body axis: the lateral fall reads as a silhouette
    // change instead of being flattened by a top-down camera.
    let orbit = CameraOrbit {
        focus: Vec3::new(focus.x, focus.y + 0.04, focus.z),
        yaw_rad: -1.6,
        pitch_rad: 1.35,
        distance_m: 1.6,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render fall-vs-save panel");
    output.color.rgba8
}

/// Joins the two panels with a colored banner: red over the open-loop fall, green
/// over the feedback save.
fn composite_side_by_side(left: &[u8], right: &[u8]) -> Vec<u8> {
    let width = (PANEL_WIDTH * 2) as usize;
    let height = (PANEL_HEIGHT + BANNER_HEIGHT) as usize;
    let mut composite = vec![0_u8; width * height * 4];
    for y in 0..BANNER_HEIGHT as usize {
        for x in 0..width {
            let offset = (y * width + x) * 4;
            let color = if x < PANEL_WIDTH as usize {
                [196, 60, 48, 255]
            } else {
                [56, 168, 82, 255]
            };
            composite[offset..offset + 4].copy_from_slice(&color);
        }
    }
    for y in 0..PANEL_HEIGHT as usize {
        let row = y + BANNER_HEIGHT as usize;
        let panel_row = y * PANEL_WIDTH as usize * 4;
        let panel_bytes = PANEL_WIDTH as usize * 4;
        let left_offset = (row * width) * 4;
        composite[left_offset..left_offset + panel_bytes]
            .copy_from_slice(&left[panel_row..panel_row + panel_bytes]);
        let right_offset = left_offset + panel_bytes;
        composite[right_offset..right_offset + panel_bytes]
            .copy_from_slice(&right[panel_row..panel_row + panel_bytes]);
    }
    composite
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
                base_color_texture: None,
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
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg fall-vs-save gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
