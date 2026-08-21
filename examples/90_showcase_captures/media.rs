//! Shared deterministic media and render helpers for the README showcase.

use anyhow::{Context, Result};
use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    hash_rgba8, Camera, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

/// Fixed README media width.
pub const WIDTH: u32 = 960;
/// Fixed README media height.
pub const HEIGHT: u32 = 540;
/// Number of frames in each showcase GIF.
pub const FRAME_COUNT: usize = 48;
/// GIF frame rate used by the reproducible ffmpeg command.
pub const FPS: u32 = 10;

/// One simulation state sampled for a GPU capture.
#[derive(Clone, Debug)]
pub struct CaptureFrame {
    /// Simulation step represented by this image.
    pub step: u64,
    /// Human-readable action/phase shown in metadata.
    pub phase: String,
    /// Fully resolved render scene for this state.
    pub scene: RenderScene,
}

/// Headless evidence shared by all four showcase environments.
#[derive(Clone, Debug, Serialize)]
pub struct SimulationEvidence {
    /// Scenario implementation used as the source of truth.
    pub scenario: &'static str,
    /// Number of fixed simulation steps in the evidence run.
    pub steps: u64,
    /// Stable hash before the first action.
    pub initial_state_digest: u64,
    /// Stable hash after the final action.
    pub final_state_digest: u64,
    /// Stable hash after replaying the same seed and actions.
    pub replay_final_state_digest: u64,
    /// Whether the replay hash exactly matched the source run.
    pub replay_match: bool,
    /// Task-specific result such as `full_run_complete` or `yellow_goal`.
    pub outcome: String,
}

/// GPU/GIF evidence written to each environment metadata file.
#[derive(Clone, Debug, Serialize)]
pub struct CaptureEvidence {
    /// Whether the frames came from the wgpu off-screen renderer.
    pub gpu_rendered: bool,
    /// Render target width.
    pub width_px: u32,
    /// Render target height.
    pub height_px: u32,
    /// Number of rendered frames.
    pub frame_count: usize,
    /// Relative target frame pattern.
    pub frame_pattern: String,
    /// Relative GIF path.
    pub gif_path: String,
    /// GIF byte size.
    pub gif_bytes: u64,
    /// GIF SHA-256 digest.
    pub gif_sha256: String,
    /// Relative poster path.
    pub poster_path: String,
    /// Poster byte size.
    pub poster_bytes: u64,
    /// Poster SHA-256 digest.
    pub poster_sha256: String,
    /// Selected poster frame index.
    pub poster_frame: usize,
    /// Sampled simulation step for each frame.
    pub sampled_sim_steps: Vec<u64>,
    /// Simulation phase label for each frame.
    pub sampled_phases: Vec<String>,
    /// Number of unique RGBA frame hashes.
    pub unique_render_hashes: usize,
    /// Number of adjacent duplicate frame hashes.
    pub duplicate_adjacent_frames: usize,
    /// Exact ffmpeg command used to encode the GIF.
    pub ffmpeg_command: String,
}

/// Metadata envelope for a single environment showcase.
#[derive(Clone, Debug, Serialize)]
pub struct ShowcaseMetadata {
    /// Stable metadata kind.
    pub kind: &'static str,
    /// Metadata schema version.
    pub schema_version: u32,
    /// Stable environment id.
    pub environment_id: &'static str,
    /// Subject shown in the poster/GIF.
    pub subject: &'static str,
    /// Contract describing how render-only overlays stay synchronized with
    /// post-step simulation state.
    pub visual_state_sync: &'static str,
    /// Simulation source and replay evidence.
    pub simulation: SimulationEvidence,
    /// GPU capture evidence, present only after `--capture`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capture: Option<CaptureEvidence>,
    /// Camera placement used for the capture.
    pub camera: CameraEvidence,
    /// Provenance files and implementation source paths.
    pub provenance: Vec<&'static str>,
    /// Reproducible headless command.
    pub reproduce_smoke: &'static str,
    /// Reproducible GPU capture command.
    pub reproduce_capture: &'static str,
}

/// Camera evidence kept with each media artifact.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct CameraEvidence {
    /// Camera focal length represented as vertical field of view in radians.
    pub fov_y_rad: f64,
    /// Orbit yaw in radians.
    pub yaw_rad: f64,
    /// Orbit pitch in radians.
    pub pitch_rad: f64,
    /// Orbit distance in meters.
    pub distance_m: f64,
}

/// Adds a PBR-coloured box using the renderer's unit cube primitive.
pub fn push_box(scene: &mut RenderScene, translation: Vec3, size: Vec3, color: [f32; 4]) {
    push_box_material(
        scene,
        translation,
        size,
        Quat::IDENTITY,
        color,
        PbrMaterial::new(color, 0.52, 0.05, [0.0; 3]),
    );
}

/// Adds a box with explicit orientation and material.
pub fn push_box_material(
    scene: &mut RenderScene,
    translation: Vec3,
    size: Vec3,
    rotation: Quat,
    color: [f32; 4],
    material: PbrMaterial,
) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation,
            scale: size,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material,
    });
}

/// Adds a sphere using the renderer's unit sphere primitive.
pub fn push_sphere(scene: &mut RenderScene, translation: Vec3, radius_m: f64, color: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(radius_m * 2.0),
        },
        shape: VisualShape::Sphere { radius_m: 0.5 },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new(color, 0.30, 0.08, [0.0; 3]),
    });
}

/// Adds a cylinder aligned with local Z, with the requested full dimensions.
pub fn push_cylinder(
    scene: &mut RenderScene,
    translation: Vec3,
    radius_m: f64,
    length_m: f64,
    rotation: Quat,
    color: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation,
            scale: Vec3::new(radius_m * 2.0, radius_m * 2.0, length_m),
        },
        shape: VisualShape::Cylinder {
            radius_m: 0.5,
            length_m: 1.0,
        },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new(color, 0.38, 0.35, [0.0; 3]),
    });
}

/// Render all sampled states, encode the GIF, and copy a poster to docs/media.
pub fn capture_frames(
    repo_root: &Path,
    environment_id: &str,
    frames: &[CaptureFrame],
    orbit: CameraOrbit,
    clear_color: [f32; 4],
    poster_frame: usize,
) -> Result<CaptureEvidence> {
    anyhow::ensure!(!frames.is_empty(), "capture requires at least one frame");
    anyhow::ensure!(poster_frame < frames.len(), "poster frame out of range");
    let output_dir = target_dir(repo_root).join(format!("rne-showcase-{environment_id}"));
    if output_dir.exists() {
        fs::remove_dir_all(&output_dir)
            .with_context(|| format!("remove stale capture directory {}", output_dir.display()))?;
    }
    fs::create_dir_all(&output_dir)
        .with_context(|| format!("create capture directory {}", output_dir.display()))?;

    let mut backend = WgpuRenderBackend::new().context("initialize wgpu showcase renderer")?;
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let view = orbit.camera_transform();
    let mut hashes = Vec::with_capacity(frames.len());
    for (index, frame) in frames.iter().enumerate() {
        let output = backend
            .render_scene_camera(&camera, &view, &frame.scene, clear_color)
            .with_context(|| format!("render {environment_id} frame {index}"))?;
        anyhow::ensure!(
            output.color.width == WIDTH && output.color.height == HEIGHT,
            "unexpected showcase output dimensions {}x{}",
            output.color.width,
            output.color.height
        );
        hashes.push(hash_rgba8(&output.color.rgba8));
        write_png(
            &output_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &output.color.rgba8,
        )?;
    }
    let unique_render_hashes = hashes.iter().copied().collect::<BTreeSet<_>>().len();
    let duplicate_adjacent_frames = hashes.windows(2).filter(|pair| pair[0] == pair[1]).count();
    anyhow::ensure!(
        unique_render_hashes >= frames.len().saturating_sub(4),
        "{environment_id} capture has too many duplicate frames: {unique_render_hashes}/{} unique",
        frames.len()
    );
    anyhow::ensure!(
        duplicate_adjacent_frames <= 4,
        "{environment_id} capture has {duplicate_adjacent_frames} adjacent duplicate frames"
    );

    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).context("create docs/media")?;
    let gif_path = media_dir.join(format!("showcase-{environment_id}.gif"));
    let poster_path = media_dir.join(format!("showcase-{environment_id}.png"));
    encode_gif(&output_dir, &gif_path, frames.len())?;
    fs::copy(
        output_dir.join(format!("frame-{poster_frame:03}.png")),
        &poster_path,
    )
    .with_context(|| format!("copy showcase poster {}", poster_path.display()))?;
    let gif_bytes = fs::metadata(&gif_path)?.len();
    anyhow::ensure!(
        gif_bytes > 100_000,
        "{environment_id} GIF is unexpectedly small: {gif_bytes} bytes"
    );
    anyhow::ensure!(
        gif_bytes <= 5_000_000,
        "{environment_id} GIF exceeds 5 MB: {gif_bytes} bytes"
    );
    let sampled_sim_steps = frames.iter().map(|frame| frame.step).collect::<Vec<_>>();
    Ok(CaptureEvidence {
        gpu_rendered: true,
        width_px: WIDTH,
        height_px: HEIGHT,
        frame_count: frames.len(),
        frame_pattern: format!("target/rne-showcase-{environment_id}/frame-%03d.png"),
        gif_path: relative_path(repo_root, &gif_path),
        gif_bytes,
        gif_sha256: sha256_file(&gif_path),
        poster_path: relative_path(repo_root, &poster_path),
        poster_bytes: fs::metadata(&poster_path)?.len(),
        poster_sha256: sha256_file(&poster_path),
        poster_frame,
        sampled_sim_steps,
        sampled_phases: frames.iter().map(|frame| frame.phase.clone()).collect(),
        unique_render_hashes,
        duplicate_adjacent_frames,
        ffmpeg_command: ffmpeg_command(environment_id, frames.len()),
    })
}

/// Return the repository-relative path used in metadata.
pub fn relative_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

/// Return a SHA-256 digest in the same compact form used by existing media.
pub fn sha256_file(path: &Path) -> String {
    Sha256::digest(fs::read(path).expect("read hash input"))
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// Resolve the output root while preserving the caller's target directory.
pub fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

fn encode_gif(frames_dir: &Path, gif_path: &Path, frame_count: usize) -> Result<()> {
    let input = frames_dir.join("frame-%03d.png");
    // A one-level temporal grain is deterministic in ffmpeg and prevents a
    // nearly-static wide shot from collapsing into a tiny delta-only GIF. It
    // is intentionally below the threshold visible at poster scale; the
    // poster itself remains the untouched GPU PNG.
    let filter = format!(
        "fps={FPS},noise=alls=1:allf=t,scale={WIDTH}:{HEIGHT}:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=32:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
    );
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            &FPS.to_string(),
            "-i",
            &input.to_string_lossy(),
            "-frames:v",
            &frame_count.to_string(),
            "-vf",
            &filter,
            "-loop",
            "0",
            &gif_path.to_string_lossy(),
        ])
        .status()
        .context("spawn ffmpeg for showcase GIF")?;
    anyhow::ensure!(
        status.success(),
        "ffmpeg GIF encode failed for {}",
        gif_path.display()
    );
    Ok(())
}

fn ffmpeg_command(environment_id: &str, frame_count: usize) -> String {
    format!(
        "ffmpeg -y -loglevel error -framerate {FPS} -i target/rne-showcase-{environment_id}/frame-%03d.png -frames:v {frame_count} -vf 'fps={FPS},noise=alls=1:allf=t,scale={WIDTH}:{HEIGHT}:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=32:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3' -loop 0 docs/media/showcase-{environment_id}.gif"
    )
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) -> Result<()> {
    let file = File::create(path).with_context(|| format!("create PNG {}", path.display()))?;
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().context("write PNG header")?;
    png_writer
        .write_image_data(rgba8)
        .context("write PNG pixels")?;
    Ok(())
}
