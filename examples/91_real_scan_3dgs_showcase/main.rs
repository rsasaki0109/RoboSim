//! README capture made from a real CC0 Scaniverse Gaussian-splat dataset.
//!
//! ```text
//! cargo run --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --smoke
//! cargo run --release --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --capture
//! ```

use anyhow::{Context, Result};
use png::{BitDepth, ColorType, Encoder};
use rne_math::Vec3;
use rne_render::{
    hash_rgba8, validate_gaussian_splat_manifest_with_override, Camera, HybridRenderScene,
    RenderScene,
};
use rne_render_3dgs::{
    load_gaussian_splat_background, render_hybrid_scene_camera, splat_proxy_depth_from_ply,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 48;
const POSTER_FRAME: usize = 24;
const FOV_Y_RAD: f64 = 0.72;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.045, 0.055, 1.0];
const PLY_BYTES: u64 = 14_644_690;
const PLY_SHA256: &str = "ac0cee7f06f2cebf9d912bf211bc87cd8f3229a0ebd59e0389daadf530389298";
const SMOKE_COMMAND: &str =
    "cargo run --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --smoke";
const CAPTURE_COMMAND: &str = "cargo run --release --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --capture";

#[derive(Clone, Debug, Serialize)]
struct Metadata {
    kind: &'static str,
    schema_version: u32,
    environment_id: String,
    subject: &'static str,
    dataset: DatasetEvidence,
    simulation: DeterminismEvidence,
    #[serde(skip_serializing_if = "Option::is_none")]
    capture: Option<CaptureEvidence>,
    camera: CameraEvidence,
    provenance: Vec<&'static str>,
    reproduce_smoke: &'static str,
    reproduce_capture: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct DatasetEvidence {
    capture_method: &'static str,
    real_capture: bool,
    synthetic_splats_added: bool,
    upstream_gaussians: u64,
    derivative_gaussians: u64,
    selection: &'static str,
    ply_bytes: u64,
    ply_sha256: String,
    standin: bool,
}

#[derive(Clone, Debug, Serialize)]
struct DeterminismEvidence {
    scenario: &'static str,
    steps: u64,
    deterministic_digest: u64,
    replay_digest: u64,
    replay_match: bool,
    outcome: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct CaptureEvidence {
    gpu_rendered: bool,
    width_px: u32,
    height_px: u32,
    frame_count: usize,
    frame_pattern: String,
    gif_path: String,
    gif_bytes: u64,
    gif_sha256: String,
    poster_path: String,
    poster_bytes: u64,
    poster_sha256: String,
    poster_frame: usize,
    sampled_sim_steps: Vec<u64>,
    sampled_phases: Vec<&'static str>,
    unique_render_hashes: usize,
    duplicate_adjacent_frames: usize,
    ffmpeg_command: String,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct CameraEvidence {
    fov_y_rad: f64,
    focus_m: [f64; 3],
    yaw_start_rad: f64,
    yaw_end_rad: f64,
    pitch_rad: f64,
    distance_m: f64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("real-scan 3DGS showcase failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let smoke = args.iter().any(|argument| argument == "--smoke");
    let capture = args.iter().any(|argument| argument == "--capture");
    let probe = args.iter().any(|argument| argument == "--probe");
    anyhow::ensure!(
        smoke || capture || probe,
        "choose --smoke, --probe, or --capture"
    );

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest_path = repo_root
        .join("assets/environments/wakufactory_sakura_3dgs/wakufactory_sakura.rne.splat.toml");
    let ply_override = args
        .windows(2)
        .find(|window| window[0] == "--ply")
        .map(|window| PathBuf::from(&window[1]));
    let environment =
        validate_gaussian_splat_manifest_with_override(&manifest_path, ply_override.as_deref())
            .context("validate real-capture splat manifest")?;
    anyhow::ensure!(
        !environment.standin,
        "real showcase must not use a stand-in"
    );
    let ply_sha256 = sha256_file(&environment.ply_path);
    if ply_override.is_none() {
        anyhow::ensure!(
            fs::metadata(&environment.ply_path)?.len() == PLY_BYTES,
            "real-capture PLY byte count changed"
        );
        anyhow::ensure!(ply_sha256 == PLY_SHA256, "real-capture PLY SHA-256 changed");
    } else {
        anyhow::ensure!(
            probe,
            "--ply is a local probe override and requires --probe"
        );
    }

    let deterministic_digest = deterministic_evidence(&environment)?;
    let replay_digest = deterministic_evidence(&environment)?;
    anyhow::ensure!(
        deterministic_digest == replay_digest,
        "real-scan camera/depth evidence did not replay exactly"
    );

    let capture_evidence = if capture {
        Some(render_capture(&repo_root, &environment)?)
    } else if probe {
        render_probe(&repo_root, &environment)?;
        None
    } else {
        None
    };
    let metadata = Metadata {
        kind: "rne_real_capture_3dgs_showcase_metadata",
        schema_version: 1,
        environment_id: environment.environment_id,
        subject: "WakuFactory Sakura real-world Scaniverse capture",
        dataset: DatasetEvidence {
            capture_method: "iPad Pro (3rd generation) + Scaniverse",
            real_capture: true,
            synthetic_splats_added: false,
            upstream_gaussians: 236_178,
            derivative_gaussians: 59_045,
            selection: "every fourth upstream Gaussian record, copied byte-for-byte",
            ply_bytes: PLY_BYTES,
            ply_sha256,
            standin: false,
        },
        simulation: DeterminismEvidence {
            scenario: "fixed 48-step orbit through the committed real-capture splat cloud",
            steps: FRAME_COUNT as u64,
            deterministic_digest,
            replay_digest,
            replay_match: true,
            outcome: "real_capture_loaded=true; deterministic_camera_replay=true",
        },
        capture: capture_evidence,
        camera: camera_evidence(),
        provenance: vec![
            "assets/environments/wakufactory_sakura_3dgs/PROVENANCE.md",
            "assets/environments/wakufactory_sakura_3dgs/wakufactory_sakura.rne.splat.toml",
            "tools/prepare_wakufactory_sakura_3dgs.py",
            "examples/91_real_scan_3dgs_showcase/main.rs",
        ],
        reproduce_smoke: SMOKE_COMMAND,
        reproduce_capture: CAPTURE_COMMAND,
    };
    let metadata_json = serde_json::to_string_pretty(&metadata)?;
    if capture {
        fs::write(
            repo_root.join("docs/media/showcase-real-3dgs.json"),
            &metadata_json,
        )?;
    } else {
        let out_dir = target_dir(&repo_root).join("rne-showcase-real-3dgs");
        fs::create_dir_all(&out_dir)?;
        fs::write(out_dir.join("smoke.json"), &metadata_json)?;
    }
    println!(
        "real 3DGS {}: environment={} ply_bytes={} sha256={} digest={:#018x}",
        if capture {
            "capture"
        } else if probe {
            "probe"
        } else {
            "smoke"
        },
        metadata.environment_id,
        metadata.dataset.ply_bytes,
        metadata.dataset.ply_sha256,
        deterministic_digest,
    );
    Ok(())
}

fn deterministic_evidence(environment: &rne_render::GaussianSplatEnvironment) -> Result<u64> {
    let camera = Camera::new(160, 90, FOV_Y_RAD);
    let mut digest = 0xcbf29ce484222325_u64;
    for index in [0_usize, FRAME_COUNT / 2, FRAME_COUNT - 1] {
        let orbit = orbit(index);
        let depth = splat_proxy_depth_from_ply(
            &environment.ply_path,
            &camera,
            &orbit.camera_transform(),
            &environment.transform,
        )?;
        digest ^= depth.hash_depth();
        digest = digest.wrapping_mul(0x100000001b3);
        digest ^= index as u64;
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(digest)
}

fn render_probe(
    repo_root: &Path,
    environment: &rne_render::GaussianSplatEnvironment,
) -> Result<()> {
    let out_dir = target_dir(repo_root).join("rne-showcase-real-3dgs");
    fs::create_dir_all(&out_dir)?;
    for index in [0, 12, POSTER_FRAME, 36, FRAME_COUNT - 1] {
        let rgba8 = render_frame(environment, index)?;
        let output = out_dir.join(format!("probe-{index:02}.png"));
        write_png(&output, WIDTH, HEIGHT, &rgba8)?;
        println!("wrote {}", output.display());
    }
    Ok(())
}

fn render_capture(
    repo_root: &Path,
    environment: &rne_render::GaussianSplatEnvironment,
) -> Result<CaptureEvidence> {
    let out_dir = target_dir(repo_root).join("rne-showcase-real-3dgs");
    if out_dir.exists() {
        fs::remove_dir_all(&out_dir)?;
    }
    fs::create_dir_all(&out_dir)?;
    let mut backend = WgpuRenderBackend::new().context("initialize wgpu real-scan renderer")?;
    let mut background = load_gaussian_splat_background(backend.device(), environment)
        .context("load real Scaniverse Gaussian splats")?;
    let camera = Camera::new(WIDTH, HEIGHT, FOV_Y_RAD);
    let hybrid = HybridRenderScene::new(environment.clone(), RenderScene::new());
    let mut hashes = Vec::with_capacity(FRAME_COUNT);
    for index in 0..FRAME_COUNT {
        let output = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &camera,
            &orbit(index).camera_transform(),
            &hybrid,
            CLEAR_COLOR,
        )
        .with_context(|| format!("render real-scan frame {index}"))?;
        hashes.push(hash_rgba8(&output.color.rgba8));
        write_png(
            &out_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &output.color.rgba8,
        )?;
    }
    let unique_render_hashes = hashes.iter().copied().collect::<BTreeSet<_>>().len();
    let duplicate_adjacent_frames = hashes.windows(2).filter(|pair| pair[0] == pair[1]).count();
    anyhow::ensure!(
        unique_render_hashes >= FRAME_COUNT.saturating_sub(1),
        "real-scan capture has only {unique_render_hashes}/{FRAME_COUNT} unique frames"
    );
    anyhow::ensure!(
        duplicate_adjacent_frames <= 1,
        "real-scan capture has {duplicate_adjacent_frames} adjacent duplicates"
    );

    let media_dir = repo_root.join("docs/media");
    let gif_path = media_dir.join("showcase-real-3dgs.gif");
    let poster_path = media_dir.join("showcase-real-3dgs.png");
    fs::copy(
        out_dir.join(format!("frame-{POSTER_FRAME:03}.png")),
        &poster_path,
    )?;
    encode_gif(&out_dir, &gif_path)?;
    let gif_bytes = fs::metadata(&gif_path)?.len();
    anyhow::ensure!(gif_bytes >= 100_000, "real-scan GIF is unexpectedly small");
    anyhow::ensure!(gif_bytes <= 5_000_000, "real-scan GIF exceeds 5 MB");
    Ok(CaptureEvidence {
        gpu_rendered: true,
        width_px: WIDTH,
        height_px: HEIGHT,
        frame_count: FRAME_COUNT,
        frame_pattern: "target/rne-showcase-real-3dgs/frame-%03d.png".into(),
        gif_path: "docs/media/showcase-real-3dgs.gif".into(),
        gif_bytes,
        gif_sha256: sha256_file(&gif_path),
        poster_path: "docs/media/showcase-real-3dgs.png".into(),
        poster_bytes: fs::metadata(&poster_path)?.len(),
        poster_sha256: sha256_file(&poster_path),
        poster_frame: POSTER_FRAME,
        sampled_sim_steps: (0..FRAME_COUNT as u64).collect(),
        sampled_phases: vec!["real-scan-orbit"; FRAME_COUNT],
        unique_render_hashes,
        duplicate_adjacent_frames,
        ffmpeg_command: ffmpeg_command(),
    })
}

fn render_frame(
    environment: &rne_render::GaussianSplatEnvironment,
    index: usize,
) -> Result<Vec<u8>> {
    let mut backend = WgpuRenderBackend::new().context("initialize wgpu real-scan probe")?;
    let mut background = load_gaussian_splat_background(backend.device(), environment)?;
    let camera = Camera::new(WIDTH, HEIGHT, FOV_Y_RAD);
    let hybrid = HybridRenderScene::new(environment.clone(), RenderScene::new());
    let output = render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        &camera,
        &orbit(index).camera_transform(),
        &hybrid,
        CLEAR_COLOR,
    )?;
    Ok(output.color.rgba8)
}

fn orbit(index: usize) -> CameraOrbit {
    let t = index as f64 / (FRAME_COUNT - 1) as f64;
    let eased = 0.5 - 0.5 * (std::f64::consts::TAU * t).cos();
    CameraOrbit {
        focus: Vec3::new(-0.13, 0.015, 0.015),
        yaw_rad: std::f64::consts::PI - 0.20 + 0.40 * eased,
        pitch_rad: 1.18,
        distance_m: 0.72,
    }
}

fn camera_evidence() -> CameraEvidence {
    CameraEvidence {
        fov_y_rad: FOV_Y_RAD,
        focus_m: [-0.13, 0.015, 0.015],
        yaw_start_rad: std::f64::consts::PI - 0.20,
        yaw_end_rad: std::f64::consts::PI + 0.20,
        pitch_rad: 1.18,
        distance_m: 0.72,
    }
}

fn encode_gif(frames_dir: &Path, gif_path: &Path) -> Result<()> {
    let input = frames_dir.join("frame-%03d.png");
    let filter = "fps=10,scale=960:540:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=32:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3";
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "10",
            "-i",
            &input.to_string_lossy(),
            "-frames:v",
            &FRAME_COUNT.to_string(),
            "-vf",
            filter,
            "-loop",
            "0",
            &gif_path.to_string_lossy(),
        ])
        .status()
        .context("spawn ffmpeg")?;
    anyhow::ensure!(status.success(), "ffmpeg GIF encode failed");
    Ok(())
}

fn ffmpeg_command() -> String {
    "ffmpeg -y -loglevel error -framerate 10 -i target/rne-showcase-real-3dgs/frame-%03d.png -frames:v 48 -vf 'fps=10,scale=960:540:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=32:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3' -loop 0 docs/media/showcase-real-3dgs.gif".into()
}

fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

fn sha256_file(path: &Path) -> String {
    Sha256::digest(fs::read(path).expect("read hash input"))
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) -> Result<()> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header()?;
    png_writer.write_image_data(rgba8)?;
    Ok(())
}
