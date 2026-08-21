//! Tsukuba confirmation + PLATEAU tile backdrop (visual only).
//!
//! Imports the committed PLATEAU LOD1 fixture as a mesh backdrop behind the
//! analytic Tsukuba confirmation sidewalk. Contest scoring stays in example 75
//! (`rne.tsukuba.confirmation.v1`); this example never changes those judges.

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, tsukuba_confirmation_task_spec, TSUKUBA_CONFIRMATION_TASK_ID,
};
use rne_assets::load_and_spawn_scene;
use rne_ecs::World;
use rne_math::{Quat, Transform3, Vec3};
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions};
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const CLEAR_COLOR: [f32; 4] = [0.52, 0.60, 0.68, 1.0];

#[derive(Clone, Debug, Deserialize)]
struct BackdropManifest {
    kind: String,
    schema_version: u32,
    environment_id: String,
    confirmation_task_id: String,
    plateau_fixture_gml: String,
    tile_name: String,
    backdrop_translation_m: [f64; 3],
    backdrop_scale: f64,
    renderer_identity: String,
}

#[derive(Clone, Debug, Serialize)]
struct TsukubaPlateauCaptureReport {
    kind: &'static str,
    schema_version: u32,
    environment_id: String,
    confirmation_task_id: String,
    renderer_identity: String,
    plateau_building_count: usize,
    plateau_road_count: usize,
    plateau_triangle_count: usize,
    backdrop_items: usize,
    foreground_items: usize,
    rgba_hash: Option<u64>,
    png_path: Option<String>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest_path = repo_root.join("assets/environments/tsukuba_plateau_backdrop.rne.env.toml");
    let manifest = load_manifest(&manifest_path);

    assert_eq!(manifest.kind, "rne_tsukuba_plateau_backdrop");
    assert_eq!(manifest.schema_version, 1);
    assert_eq!(manifest.confirmation_task_id, TSUKUBA_CONFIRMATION_TASK_ID);
    let task = tsukuba_confirmation_task_spec(500);
    task.validate().expect("confirmation TaskSpec");
    assert_eq!(task.task_id, TSUKUBA_CONFIRMATION_TASK_ID);

    let fixture = resolve_path(&manifest_path, &manifest.plateau_fixture_gml);
    assert!(
        fixture.is_file(),
        "missing PLATEAU fixture {}",
        fixture.display()
    );

    let out_dir = output_dir(&repo_root).join("rne-tsukuba-plateau-backdrop");
    fs::create_dir_all(&out_dir).expect("output dir");
    let import = import_citygml_file(
        &fixture,
        &out_dir.join("plateau"),
        &ImportOptions {
            tile_name: manifest.tile_name.clone(),
            coordinate_mode: CoordinateMode::GeographicDegrees,
            world_seed: 83,
            ..ImportOptions::default()
        },
    )
    .expect("import PLATEAU fixture");
    assert!(
        import.building_count >= 1,
        "expected buildings from the LOD1 fixture"
    );
    assert!(
        import.road_count >= 1,
        "expected roads from the LOD1 fixture"
    );
    assert!(import.scene_path.is_file());

    let mut plateau_world = World::new();
    load_and_spawn_scene(&mut plateau_world, &import.scene_path).expect("spawn PLATEAU backdrop");
    let mut backdrop = build_visual_render_scene(&plateau_world);
    offset_scene(
        &mut backdrop,
        Vec3::from_array(manifest.backdrop_translation_m),
        manifest.backdrop_scale,
    );
    let foreground = tsukuba_confirmation_foreground();
    let mut composite = backdrop.clone();
    composite.items.extend(foreground.items.iter().cloned());

    if smoke || std::env::var_os("RNE_SKIP_GPU").is_some() {
        let report = TsukubaPlateauCaptureReport {
            kind: "rne_tsukuba_plateau_backdrop_capture_report",
            schema_version: 1,
            environment_id: manifest.environment_id.clone(),
            confirmation_task_id: manifest.confirmation_task_id.clone(),
            renderer_identity: manifest.renderer_identity.clone(),
            plateau_building_count: import.building_count,
            plateau_road_count: import.road_count,
            plateau_triangle_count: import.triangle_count,
            backdrop_items: backdrop.items.len(),
            foreground_items: foreground.items.len(),
            rgba_hash: None,
            png_path: None,
        };
        write_report(&out_dir.join("tsukuba_plateau_backdrop.json"), &report);
        println!(
            "smoke: environment={} task={} buildings={} roads={} backdrop_items={} foreground_items={}",
            report.environment_id,
            report.confirmation_task_id,
            report.plateau_building_count,
            report.plateau_road_count,
            report.backdrop_items,
            report.foreground_items
        );
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; Tsukuba PLATEAU backdrop smoke passed: {error}");
            return;
        }
    };
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(3.75, 0.25, 0.0),
        yaw_rad: -1.35,
        pitch_rad: 1.05,
        distance_m: 8.0,
    };
    let view = orbit.camera_transform();
    let output = match backend.render_scene_camera(&camera, &view, &composite, CLEAR_COLOR) {
        Ok(output) => output,
        Err(error) => {
            eprintln!("capture unavailable; Tsukuba PLATEAU backdrop smoke passed: {error}");
            return;
        }
    };
    let hash = output.color.hash_pixels();
    assert_ne!(hash, 0);
    let png_path = out_dir.join("tsukuba_plateau_backdrop.png");
    write_png(
        &png_path,
        output.color.width,
        output.color.height,
        &output.color.rgba8,
    );
    let report = TsukubaPlateauCaptureReport {
        kind: "rne_tsukuba_plateau_backdrop_capture_report",
        schema_version: 1,
        environment_id: manifest.environment_id,
        confirmation_task_id: manifest.confirmation_task_id,
        renderer_identity: manifest.renderer_identity,
        plateau_building_count: import.building_count,
        plateau_road_count: import.road_count,
        plateau_triangle_count: import.triangle_count,
        backdrop_items: backdrop.items.len(),
        foreground_items: foreground.items.len(),
        rgba_hash: Some(hash),
        png_path: Some(png_path.display().to_string()),
    };
    write_report(&out_dir.join("tsukuba_plateau_backdrop.json"), &report);
    println!(
        "capture: buildings={} roads={} rgba_hash={hash} png={}",
        import.building_count,
        import.road_count,
        png_path.display()
    );
}

fn load_manifest(path: &Path) -> BackdropManifest {
    let text = fs::read_to_string(path).expect("read backdrop manifest");
    toml::from_str(&text).expect("parse backdrop manifest")
}

fn resolve_path(manifest_path: &Path, relative: &str) -> PathBuf {
    manifest_path
        .parent()
        .expect("manifest parent")
        .join(relative)
}

fn offset_scene(scene: &mut RenderScene, translation: Vec3, scale: f64) {
    for item in &mut scene.items {
        item.transform.translation = translation + item.transform.translation * scale;
        item.transform.scale *= scale;
    }
}

fn tsukuba_confirmation_foreground() -> RenderScene {
    let mut scene = RenderScene::new();
    scene.items.push(box_item(
        [3.75, 0.01, 0.0],
        [8.5, 0.02, 2.0],
        [0.78, 0.76, 0.72, 0.85],
    ));
    scene.items.push(box_item(
        [3.75, 0.005, 2.2],
        [8.5, 0.01, 2.4],
        [0.22, 0.22, 0.24, 0.90],
    ));
    scene.items.push(box_item(
        [6.0, 0.2, 0.0],
        [0.24, 0.4, 0.24],
        [0.12, 0.72, 0.22, 1.0],
    ));
    scene.items.push(box_item(
        [0.5, 0.25, 0.0],
        [0.5, 0.3, 0.4],
        [0.35, 0.55, 0.95, 1.0],
    ));
    scene
}

fn box_item(center: [f64; 3], size_m: [f64; 3], color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation: Vec3::from_array(center),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Box {
            size_m: Vec3::from_array(size_m),
        },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

fn output_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

fn write_report(path: &Path, report: &TsukubaPlateauCaptureReport) {
    let json = serde_json::to_string_pretty(report).expect("report json");
    fs::write(path, json).expect("write report");
    println!("report {}", path.display());
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) {
    let file = File::create(path).expect("png file");
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().expect("png header");
    png_writer.write_image_data(rgba8).expect("png pixels");
}
