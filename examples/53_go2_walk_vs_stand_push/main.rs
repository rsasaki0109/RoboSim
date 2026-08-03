//! Renders the Go2 motion-is-stability comparison to README media.
//!
//! Both panels take the identical sustained 1.8 rad flank push on identical
//! 8 N*m torque-limited motors, with **no balance controller on either side**.
//! The left robot trots slowly in place and capsizes; the right robot walks at
//! ~0.17 m/s and shrugs the push off — cyclic foot replanting is itself a
//! stabilizer (see `docs/GO2_LOCOMOTION.md`). The same numbers are pinned by
//! the `walking_trot_shrugs_off_the_push_that_topples_the_slow_trot` test.

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
const FRAME_COUNT: usize = 84;
const STEPS_PER_FRAME: u64 = 5;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

// The measured boundary: this push topples the slow trot and is shrugged off by
// the walking trot, both open loop on 8 N*m motors. The push step is a multiple
// of both cycle lengths — the topple is gait-phase dependent, and this phase
// matches the acceptance test's.
const MOTOR_MAX_FORCE_N: f64 = 8.0;
const PUSH_STEP: u64 = 180;
const PUSH_TOTAL_TILT_RAD: f64 = 1.8;
const PUSH_DURATION_STEPS: u64 = 20;
const SLOW_CYCLE_STEPS: u64 = 90;
const WALK_CYCLE_STEPS: u64 = 45;
const WALK_STRIDE_RAD: f64 = 0.24;

fn episode(cycle_steps: u64) -> UnitreeGo2Episode {
    UnitreeGo2Episode::new(UnitreeGo2EpisodeConfig {
        max_steps: FRAME_COUNT as u64 * STEPS_PER_FRAME + 1,
        cycle_steps,
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

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("go2-walk-vs-stand-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create walk-vs-stand frame directory");

    let mut slow_episode = episode(SLOW_CYCLE_STEPS);
    let mut walk_episode = episode(WALK_CYCLE_STEPS);

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = slow_episode.sim().mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    let walk_action = UnitreeGo2Action {
        stride_rad: WALK_STRIDE_RAD,
        ..UnitreeGo2Action::default()
    };
    let mut slow_observation: Option<UnitreeGo2Observation> = None;
    let mut walk_observation: Option<UnitreeGo2Observation> = None;

    for frame in 0..FRAME_COUNT {
        for _ in 0..STEPS_PER_FRAME {
            slow_observation = Some(slow_episode.step(UnitreeGo2Action::default()).observation);
            walk_observation = Some(walk_episode.step(walk_action).observation);
        }
        let left = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            slow_episode.sim(),
        );
        let right = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            walk_episode.sim(),
        );
        if frame == 0 {
            let unique = left
                .chunks_exact(4)
                .collect::<std::collections::HashSet<_>>()
                .len();
            assert!(unique > 2, "walk-vs-stand frame should contain geometry");
        }
        let composite = composite_side_by_side(&left, &right);
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &composite,
            PANEL_WIDTH * 2,
            PANEL_HEIGHT + BANNER_HEIGHT,
        )
        .expect("write walk-vs-stand frame");
    }

    // The GIF must show the measured physics: slow trot down, walking trot
    // upright and still covering ground.
    let tilt = |observation: &UnitreeGo2Observation| {
        observation
            .base_relative_pitch_rad
            .hypot(observation.base_relative_roll_rad)
    };
    let slow_end = slow_observation.expect("slow panel stepped");
    let walk_end = walk_observation.expect("walking panel stepped");
    assert!(
        tilt(&slow_end) > 1.3,
        "slow-trot panel should end flat on its side, tilt {:.2}",
        tilt(&slow_end)
    );
    assert!(
        tilt(&walk_end) < 0.3 && walk_end.base_y_m > 0.2,
        "walking panel should end upright, tilt {:.2} height {:.3}",
        tilt(&walk_end),
        walk_end.base_y_m
    );

    let gif_path = media_dir.join("go2-walk-vs-stand-push.gif");
    build_gif(&frames_dir, &gif_path).expect("encode walk-vs-stand gif");
    let poster = image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read poster frame");
    poster
        .save(media_dir.join("go2-walk-vs-stand-push.png"))
        .expect("write walk-vs-stand poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!("rendered walk-vs-stand media to {}", gif_path.display());
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    sim: &UrdfSceneSim,
) -> Vec<u8> {
    let observed = sim.observe();
    let mut scene = build_visual_render_scene(sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_checker_floor(&mut scene, observed.base_x_m, observed.base_z_m, 0.12);
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official Go2 meshes");
    // Low follow-cam: tracks the base so the walking robot stays in frame and
    // the lateral fall reads as a silhouette change.
    let orbit = CameraOrbit {
        focus: Vec3::new(
            observed.base_x_m,
            observed.base_y_m + 0.04,
            observed.base_z_m,
        ),
        yaw_rad: -1.6,
        pitch_rad: 1.35,
        distance_m: 1.6,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render walk-vs-stand panel");
    output.color.rgba8
}

/// Joins the two panels with a colored banner: red over the toppling slow trot,
/// green over the walking trot that shrugs the push off.
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
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -8..=8 {
        for column in -8..=8 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(
                        snap(center_x_m) + column as f64 * tile_m,
                        -0.008,
                        snap(center_z_m) + row as f64 * tile_m,
                    ),
                    rotation: rne_math::Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: color,
                mesh: None,
                base_color_texture: None,
                material: Default::default(),
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
        .ok_or_else(|| std::io::Error::other("ffmpeg walk-vs-stand gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
