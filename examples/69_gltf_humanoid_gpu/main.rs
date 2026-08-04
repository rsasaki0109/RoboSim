//! Loads a real rigged humanoid glTF asset and validates the GPU skinning path.
//!
//! The default mode renders two deterministic animation frames to
//! `target/rne-gltf-humanoid`. `--smoke` loads the same asset, samples the
//! animation and GPU payloads, and exits without initializing a GPU.

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Vec3};
use rne_render::{
    hash_depth_f32, hash_rgba8, load_gltf_scene, Camera, GltfAnimationPlayer, GltfSceneAsset,
    RenderBackend, RenderScene, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const ASSET_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/fixtures/rigged_figure/RiggedFigure.glb"
);
const WIDTH: u32 = 640;
const HEIGHT: u32 = 640;
const CLEAR_COLOR: [f32; 4] = [0.018, 0.024, 0.038, 1.0];

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke")
        || std::env::var_os("RNE_SKIP_GPU").is_some();
    let asset_path = Path::new(ASSET_PATH);
    let asset = load_gltf_scene(asset_path).expect("load Rigged Figure GLB");
    let mut player = GltfAnimationPlayer::new(Some(0));
    player.advance(0.45);
    validate_asset(&asset, &player);

    if smoke {
        println!(
            "Rigged Figure smoke: nodes={} parts={} skins={} animations={} skinned_parts={} gpu_parts={}",
            asset.nodes.len(),
            asset.parts.len(),
            asset.skins.len(),
            asset.animations.len(),
            asset
                .parts
                .iter()
                .filter(|part| part.skin_index.is_some())
                .count(),
            gpu_part_count(&asset, &player),
        );
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; Rigged Figure asset smoke passed: {error}");
            return;
        }
    };
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 0.72, 0.0),
        yaw_rad: -0.35,
        pitch_rad: 1.25,
        distance_m: 2.35,
    };
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../..")
                .join("target")
        });
    let output_dir = target_dir.join("rne-gltf-humanoid");
    fs::create_dir_all(&output_dir).expect("create glTF humanoid output directory");

    for frame in 0..2 {
        let scene = scene_for(&asset, &player);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render Rigged Figure frame");
        let frame_path = output_dir.join(format!("frame-{frame:03}.png"));
        let min_depth_m = output
            .depth
            .depth_m
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        assert!(
            min_depth_m < camera.far_m as f32,
            "scene did not reach the camera"
        );
        write_png(
            &frame_path,
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write Rigged Figure frame");
        println!(
            "Rigged Figure frame={frame} parts={} color_hash={:#018x} depth_hash={:#018x} min_depth_m={min_depth_m:.3} path={}",
            scene.items.len(),
            hash_rgba8(&output.color.rgba8),
            hash_depth_f32(&output.depth.depth_m),
            frame_path.display()
        );
        player.advance(1.0 / 30.0);
    }
}

fn validate_asset(asset: &GltfSceneAsset, player: &GltfAnimationPlayer) {
    assert!(!asset.nodes.is_empty(), "Rigged Figure has no nodes");
    assert!(!asset.parts.is_empty(), "Rigged Figure has no mesh parts");
    assert!(!asset.skins.is_empty(), "Rigged Figure has no skins");
    assert!(
        !asset.animations.is_empty(),
        "Rigged Figure has no animations"
    );
    assert!(
        asset.parts.iter().any(|part| part.skin_index.is_some()),
        "Rigged Figure has no skinned mesh part"
    );
    assert!(player.time_s > 0.0);

    for part_index in 0..asset.parts.len() {
        let mesh = player
            .sample_part_for_gpu(asset, part_index)
            .expect("sample Rigged Figure GPU payload");
        assert_eq!(mesh.positions.len(), mesh.normals.len());
        assert!(!mesh.indices.is_empty());
        assert!(mesh
            .positions
            .iter()
            .flatten()
            .all(|value| value.is_finite()));
        if let Some(skinning) = mesh.skinning {
            assert!(!skinning.joint_matrices.is_empty());
            assert_eq!(skinning.joints.len(), mesh.positions.len());
            assert_eq!(skinning.weights.len(), mesh.positions.len());
        }
    }
}

fn gpu_part_count(asset: &GltfSceneAsset, player: &GltfAnimationPlayer) -> usize {
    (0..asset.parts.len())
        .filter(|part_index| {
            player
                .sample_part_for_gpu(asset, *part_index)
                .expect("sample Rigged Figure GPU payload")
                .skinning
                .is_some()
        })
        .count()
}

fn scene_for(asset: &GltfSceneAsset, player: &GltfAnimationPlayer) -> RenderScene {
    let mut scene = RenderScene::new();
    for part_index in 0..asset.parts.len() {
        let part = &asset.parts[part_index];
        let mesh = player
            .sample_part_for_gpu(asset, part_index)
            .expect("sample Rigged Figure render mesh");
        let mut item = RenderScene::item_from_dynamic_mesh(mesh, [1.0; 4]);
        item.base_color_texture = part.render_part.base_color_texture.clone().map(Arc::new);
        item.material = part.render_part.material.clone();
        if let Some(base_color_rgba) = part.render_part.base_color_rgba {
            item.color_rgba = [1.0; 4];
            item.material.base_color_rgba = base_color_rgba;
        }
        scene.items.push(item);
    }

    scene.items.push(RenderScene::item_from_visual(
        rne_world::Transform3::from_translation_rotation(
            Vec3::new(0.0, -0.02, 0.0),
            Quat::IDENTITY,
        ),
        VisualShape::Box {
            size_m: Vec3::new(3.0, 0.04, 3.0),
        },
        [0.12, 0.15, 0.18, 1.0],
        rne_world::Transform3::IDENTITY,
    ));
    scene
}

fn write_png(path: &Path, rgba8: &[u8], width: u32, height: u32) -> Result<(), String> {
    let file = File::create(path).map_err(|error| error.to_string())?;
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().map_err(|error| error.to_string())?;
    png_writer
        .write_image_data(rgba8)
        .map_err(|error| error.to_string())
}
