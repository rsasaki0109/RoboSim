//! Learns a Go2 turning gait that the scripted trot cannot express.
//!
//! `docs/GO2_LOCOMOTION.md` measures six hand-designed steering mechanisms and
//! shows none of them yaws this 3-DoF-per-leg platform. This example runs a
//! deterministic, resumable, parallel cross-entropy search over the Fourier
//! joint-offset overlay (`UnitreeGo2GaitOverlay`, 60 coefficients including
//! half-frequency symmetry-breaking terms) whose anti-cheat objective — the
//! minimum yaw over two disjoint late windows — is itself a measured result:
//! naive objectives were gamed twice by bounded-twist physics. The default mode
//! replays the pinned [`UnitreeGo2GaitOverlay::LEARNED_TURN`] headlessly and
//! verifies the sustained turn; `--train` reproduces the search (seed 42);
//! `--gif` additionally renders the gait. The same pinned overlay is verified
//! by the `learned_overlay_turns_the_walking_trot` test in `rne_ai`.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_dynamic_scene_path,
    unitree_go2_trot_targets_with_overlay, DeterministicRng, UnitreeGo2GaitCommand,
    UnitreeGo2GaitOverlay, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 400;
const FRAME_COUNT: usize = 84;
const STEPS_PER_FRAME: u64 = 5;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];

const ROLLOUT_STEPS: u64 = 1440;
/// The rollout's yaw is scored as the *minimum* over two disjoint late windows
/// (steps 480..960 and 960..1440). A bounded elastic twist scores zero, and a
/// slow yaw oscillation — which defeated a single-window objective by hiding
/// its reversal beyond the rollout horizon — goes negative in one of the two
/// windows, so only genuinely sustained rotation wins.
const WINDOW_SPLIT_STEP: u64 = 960;
const WINDOW_START_STEP: u64 = 480;
const SETTLE_STEPS: u64 = 120;
const DIM: usize = 60;
const POPULATION: usize = 64;
const ELITE: usize = 16;
const ITERATIONS: usize = 30;

/// Base walking command the overlay is learned on: the fast stable walk from
/// the motion-is-stability measurements.
fn walk_command() -> UnitreeGo2GaitCommand {
    UnitreeGo2GaitCommand {
        stride_rad: 0.24,
        cycle_steps: 45,
        ..UnitreeGo2GaitCommand::default()
    }
}

/// The pinned result of `--train` (seed 42) lives in the library as
/// [`UnitreeGo2GaitOverlay::LEARNED_TURN`] so the acceptance test and this
/// renderer share one source of truth. Regenerate with `--train` and update the
/// constant there.
fn learned_overlay() -> UnitreeGo2GaitOverlay {
    UnitreeGo2GaitOverlay::LEARNED_TURN
}

struct RolloutOutcome {
    total_yaw_rad: f64,
    window_a_yaw_rad: f64,
    window_b_yaw_rad: f64,
    forward_m: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
    score: f64,
}

fn rollout(overlay: &UnitreeGo2GaitOverlay, steps: u64) -> RolloutOutcome {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(180.0, 18.0, 23.7);
    let stand = unitree_go2_trot_targets_with_overlay(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            ..walk_command()
        },
        &UnitreeGo2GaitOverlay::ZERO,
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let start = sim.observe();
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut total_yaw_rad = 0.0;
    let mut window_a_yaw_rad = 0.0;
    let mut window_b_yaw_rad = 0.0;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad = 0.0_f64;
    for step in 0..steps {
        sim.step_joint_position_targets(&unitree_go2_trot_targets_with_overlay(
            step,
            walk_command(),
            overlay,
        ));
        let observed = sim.observe();
        let mut delta = observed.base_relative_yaw_rad - previous_yaw;
        while delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        total_yaw_rad += delta;
        if (WINDOW_START_STEP..WINDOW_SPLIT_STEP).contains(&step) {
            window_a_yaw_rad += delta;
        } else if step >= WINDOW_SPLIT_STEP {
            window_b_yaw_rad += delta;
        }
        previous_yaw = observed.base_relative_yaw_rad;
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(
            observed
                .base_relative_pitch_rad
                .hypot(observed.base_relative_roll_rad),
        );
    }
    let end = sim.observe();
    let forward_m = (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m);
    // Score the *minimum* of the two late windows: elastic twists score zero,
    // slow oscillations go negative in one window, and only sustained rotation
    // survives. Falling or crouching is heavily penalized so the search cannot
    // cheat by tumbling.
    let score = 2.0 * window_a_yaw_rad.min(window_b_yaw_rad)
        - if max_tilt_rad > 0.8 { 5.0 } else { 0.0 }
        - 20.0 * (0.19 - min_height_m).max(0.0);
    RolloutOutcome {
        total_yaw_rad,
        window_a_yaw_rad,
        window_b_yaw_rad,
        forward_m,
        min_height_m,
        max_tilt_rad,
        score,
    }
}

fn overlay_from(params: &[f64; DIM]) -> UnitreeGo2GaitOverlay {
    let mut coefficients = [[0.0; 5]; 12];
    for joint in 0..12 {
        coefficients[joint][0] = params[joint * 5].clamp(-0.3, 0.3);
        for harmonic in 1..5 {
            coefficients[joint][harmonic] = params[joint * 5 + harmonic].clamp(-0.35, 0.35);
        }
    }
    UnitreeGo2GaitOverlay { coefficients }
}

fn gaussian(rng: &mut DeterministicRng) -> f64 {
    let u1 = rng.uniform_f64(1.0e-12, 1.0);
    let u2 = rng.uniform_f64(0.0, 1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Iterations per `--train` invocation; the search checkpoints its state so a
/// long run can be split across invocations without losing determinism (each
/// iteration re-seeds its own RNG from the iteration index).
const ITERATIONS_PER_RUN: usize = 6;
const PARALLEL_ROLLOUTS: usize = 16;

type TrainState = (usize, [f64; DIM], [f64; DIM], (f64, [f64; DIM]));

fn state_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/go2_turn_cem_state.txt")
}

fn load_state(path: &Path) -> Option<TrainState> {
    let text = fs::read_to_string(path).ok()?;
    let values: Vec<f64> = text
        .split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    if values.len() != 2 + 3 * DIM {
        return None;
    }
    let mut mean = [0.0; DIM];
    let mut sigma = [0.0; DIM];
    let mut best_params = [0.0; DIM];
    mean.copy_from_slice(&values[1..1 + DIM]);
    sigma.copy_from_slice(&values[1 + DIM..1 + 2 * DIM]);
    best_params.copy_from_slice(&values[2 + 2 * DIM..2 + 3 * DIM]);
    Some((
        values[0] as usize,
        mean,
        sigma,
        (values[1 + 2 * DIM], best_params),
    ))
}

fn save_state(path: &Path, state: &TrainState) {
    let mut text = format!("{}\n", state.0);
    for value in state.1.iter().chain(state.2.iter()) {
        text.push_str(&format!("{value:.12}\n"));
    }
    text.push_str(&format!("{:.12}\n", state.3 .0));
    for value in state.3 .1.iter() {
        text.push_str(&format!("{value:.12}\n"));
    }
    fs::write(path, text).expect("write CEM state");
}

fn train() {
    let path = state_path();
    let (start_iteration, mut mean, mut sigma, mut best) =
        load_state(&path).unwrap_or((0, [0.0; DIM], [0.12; DIM], (f64::MIN, [0.0; DIM])));
    let end_iteration = (start_iteration + ITERATIONS_PER_RUN).min(ITERATIONS);
    for iteration in start_iteration..end_iteration {
        // Sequential sampling from a per-iteration seed keeps the search
        // deterministic and resumable; only the physics rollouts parallelize.
        let mut rng = DeterministicRng::new(42 + iteration as u64);
        let population: Vec<[f64; DIM]> = (0..POPULATION)
            .map(|_| {
                let mut params = [0.0_f64; DIM];
                for (value, (m, s)) in params.iter_mut().zip(mean.iter().zip(sigma.iter())) {
                    *value = m + s * gaussian(&mut rng);
                }
                params
            })
            .collect();
        let mut scored: Vec<(f64, [f64; DIM])> = Vec::with_capacity(POPULATION);
        for chunk in population.chunks(PARALLEL_ROLLOUTS) {
            let scores = std::thread::scope(|scope| {
                let handles: Vec<_> = chunk
                    .iter()
                    .map(|params| {
                        scope.spawn(move || rollout(&overlay_from(params), ROLLOUT_STEPS).score)
                    })
                    .collect();
                handles
                    .into_iter()
                    .map(|handle| handle.join().expect("rollout thread"))
                    .collect::<Vec<_>>()
            });
            for (score, params) in scores.into_iter().zip(chunk.iter()) {
                scored.push((score, *params));
            }
        }
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).expect("finite scores"));
        if scored[0].0 > best.0 {
            best = scored[0];
        }
        for dimension in 0..DIM {
            let elite_mean = scored[..ELITE]
                .iter()
                .map(|(_, params)| params[dimension])
                .sum::<f64>()
                / ELITE as f64;
            let elite_variance = scored[..ELITE]
                .iter()
                .map(|(_, params)| (params[dimension] - elite_mean).powi(2))
                .sum::<f64>()
                / ELITE as f64;
            mean[dimension] = elite_mean;
            sigma[dimension] = elite_variance.sqrt().max(0.02);
        }
        let probe = rollout(&overlay_from(&scored[0].1), ROLLOUT_STEPS);
        println!(
            "iter {iteration:2}: best score {:.3} (windows {:+.3}/{:+.3} totalYaw {:+.3} fwd {:.2} minH {:.3} maxTilt {:.2})",
            scored[0].0,
            probe.window_a_yaw_rad,
            probe.window_b_yaw_rad,
            probe.total_yaw_rad,
            probe.forward_m,
            probe.min_height_m,
            probe.max_tilt_rad
        );
        save_state(&path, &(iteration + 1, mean, sigma, best));
    }
    if end_iteration < ITERATIONS {
        println!("checkpointed at iteration {end_iteration}/{ITERATIONS}; run --train again");
        return;
    }
    let final_outcome = rollout(&overlay_from(&best.1), ROLLOUT_STEPS);
    println!(
        "final best: score {:.3} windows {:+.3}/{:+.3} rad per 8 s ({:.3} rad/s sustained), totalYaw {:+.3}",
        best.0,
        final_outcome.window_a_yaw_rad,
        final_outcome.window_b_yaw_rad,
        final_outcome.window_a_yaw_rad.min(final_outcome.window_b_yaw_rad) / 8.0,
        final_outcome.total_yaw_rad
    );
    let overlay = overlay_from(&best.1);
    println!("coefficients: [");
    for coefficient in overlay.coefficients {
        println!(
            "    [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}],",
            coefficient[0], coefficient[1], coefficient[2], coefficient[3], coefficient[4]
        );
    }
    println!("],");
}

fn main() {
    if std::env::args().any(|argument| argument == "--train") {
        train();
        return;
    }

    // Replay the pinned learned gait and prove the turn is sustained (positive
    // in both late windows). This runs headless and is the example's default.
    let outcome = rollout(&learned_overlay(), ROLLOUT_STEPS);
    assert!(
        outcome.window_a_yaw_rad > 0.12 && outcome.window_b_yaw_rad > 0.12,
        "learned gait should keep turning, windows {:+.3}/{:+.3}",
        outcome.window_a_yaw_rad,
        outcome.window_b_yaw_rad
    );
    assert!(
        outcome.max_tilt_rad < 0.8 && outcome.min_height_m > 0.15,
        "learned gait should stay upright, tilt {:.2} height {:.3}",
        outcome.max_tilt_rad,
        outcome.min_height_m
    );
    println!(
        "learned turn verified: windows {:+.3}/{:+.3} rad per 8 s ({:.3} rad/s sustained), totalYaw {:+.3}, maxTilt {:.2}",
        outcome.window_a_yaw_rad,
        outcome.window_b_yaw_rad,
        outcome.window_a_yaw_rad.min(outcome.window_b_yaw_rad) / 8.0,
        outcome.total_yaw_rad,
        outcome.max_tilt_rad
    );
    if !std::env::args().any(|argument| argument == "--gif") {
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("go2-learned-turn-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create learned-turn frame directory");

    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    sim.configure_position_motors(180.0, 18.0, 23.7);
    let stand = unitree_go2_trot_targets_with_overlay(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            ..walk_command()
        },
        &UnitreeGo2GaitOverlay::ZERO,
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    // Walk through the wind-up transient off camera so the GIF shows the
    // sustained turn.
    let overlay = learned_overlay();
    for step in 0..WINDOW_START_STEP {
        sim.step_joint_position_targets(&unitree_go2_trot_targets_with_overlay(
            step,
            walk_command(),
            &overlay,
        ));
    }
    let start = sim.observe();

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = WINDOW_START_STEP + frame as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_go2_trot_targets_with_overlay(
                step,
                walk_command(),
                &overlay,
            ));
        }
        let observed = sim.observe();
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
            assert!(meshes >= 13, "expected Go2 mesh visuals, got {meshes}");
        }
        // Fixed elevated camera on the start point so the turning arc is the
        // visible story; the robot stays within a body length or two of it.
        let orbit = CameraOrbit {
            focus: Vec3::new(
                (start.base_x_m + observed.base_x_m) / 2.0,
                0.12,
                (start.base_z_m + observed.base_z_m) / 2.0,
            ),
            yaw_rad: -1.1,
            pitch_rad: 1.15,
            distance_m: 2.1,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render learned-turn frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write learned-turn frame");
    }

    let gif_path = media_dir.join("go2-learned-turn.gif");
    build_gif(&frames_dir, &gif_path).expect("encode learned-turn gif");
    let poster = image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read poster frame");
    poster
        .save(media_dir.join("go2-learned-turn.png"))
        .expect("write learned-turn poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered learned-turn media to {} (total yaw {:+.3} rad)",
        gif_path.display(),
        outcome.total_yaw_rad
    );
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    for row in -10..=10 {
        for column in -10..=10 {
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
            "fps=12,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg learned-turn gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
