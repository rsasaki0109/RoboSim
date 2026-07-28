//! Imports a synthetic PLATEAU tile and renders drone and car traversal GIFs.

use png::{BitDepth, ColorType, Encoder};
use rne_assets::{load_scene_bundle, mesh_package_roots, spawn_scene_bundle, SpawnSceneOptions};
use rne_ecs::World;
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};
use rne_plateau::{import_citygml_file, ImportOptions};
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, Visual, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 720;
const HEIGHT: u32 = 405;
const DRONE_FRAME_COUNT: usize = 48;
const CAR_FRAME_COUNT: usize = 96;
const CLEAR_COLOR: [f32; 4] = [0.025, 0.045, 0.075, 1.0];

fn main() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let input = repo_root.join("crates/rne_plateau/tests/fixtures/plateau_lod1_minimal.gml");
    let generated_dir = repo_root.join("target/plateau-drone-demo");
    let result = import_citygml_file(
        &input,
        &generated_dir,
        &ImportOptions {
            tile_name: "plateau-drone-city".into(),
            world_seed: 46,
            ..ImportOptions::default()
        },
    )
    .expect("import synthetic PLATEAU tile");
    let bundle = load_scene_bundle(&result.scene_path).expect("load generated PLATEAU scene");
    let mut world = World::new();
    spawn_scene_bundle(&mut world, &bundle, None, SpawnSceneOptions::default())
        .expect("spawn generated PLATEAU scene headlessly");
    assert_eq!(result.building_count, 2);
    assert_eq!(bundle.scene.objects.len(), 2);
    println!(
        "PLATEAU tile ready: buildings={} triangles={} scene={}",
        result.building_count,
        result.triangle_count,
        result.scene_path.display()
    );

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless PLATEAU import completed");
        return;
    }
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable after successful headless import: {error}");
            return;
        }
    };
    let media_dir = repo_root.join("docs/media");
    let frames_dir = generated_dir.join("frames");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old PLATEAU frames");
    }
    fs::create_dir_all(&frames_dir).expect("create PLATEAU frames");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut city_scene = render_scene_from_world(&mut world);
    let mesh_roots = mesh_package_roots(&bundle);
    let root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    city_scene
        .resolve_mesh_assets_with_roots(&root_refs)
        .expect("resolve generated PLATEAU meshes");
    append_city_ground(&mut city_scene);

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 8.5, 0.0),
        yaw_rad: -0.80,
        pitch_rad: 0.91,
        distance_m: 42.0,
    };
    for frame in 0..DRONE_FRAME_COUNT {
        let progress = frame as f64 / (DRONE_FRAME_COUNT - 1) as f64;
        let drone_position = drone_position(progress);
        let mut scene = city_scene.clone();
        append_flight_path(&mut scene, progress);
        append_traffic(&mut scene, progress);
        append_drone(&mut scene, drone_position, progress);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU drone frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write PLATEAU drone frame");
    }

    let gif_path = media_dir.join("plateau-drone.gif");
    build_gif(&frames_dir, &gif_path).expect("encode PLATEAU drone GIF");
    image::open(frames_dir.join("frame-040.png"))
        .expect("read PLATEAU poster frame")
        .save(media_dir.join("plateau-drone.png"))
        .expect("write PLATEAU poster");
    fs::remove_dir_all(&frames_dir).expect("remove PLATEAU frame directory");

    fs::create_dir_all(&frames_dir).expect("create PLATEAU car frames");
    for frame in 0..CAR_FRAME_COUNT {
        let progress = frame as f64 / (CAR_FRAME_COUNT - 1) as f64;
        let car_position = northbound_car_position(progress);
        let mut scene = city_scene.clone();
        append_traffic(&mut scene, progress);
        let car_camera = CameraOrbit {
            focus: car_position + Vec3::new(0.0, 0.45, 0.0),
            yaw_rad: 2.85,
            pitch_rad: 1.15,
            distance_m: 12.5,
        };
        let output = backend
            .render_scene_camera(&camera, &car_camera.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render PLATEAU car frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write PLATEAU car frame");
    }
    let car_gif_path = media_dir.join("plateau-car.gif");
    build_gif(&frames_dir, &car_gif_path).expect("encode PLATEAU car GIF");
    image::open(frames_dir.join("frame-032.png"))
        .expect("read PLATEAU car poster frame")
        .save(media_dir.join("plateau-car.png"))
        .expect("write PLATEAU car poster");
    fs::remove_dir_all(&frames_dir).expect("remove PLATEAU car frame directory");
    println!(
        "rendered PLATEAU drone and car media to {} and {}",
        gif_path.display(),
        car_gif_path.display()
    );
}

fn render_scene_from_world(world: &mut World) -> RenderScene {
    let mut scene = RenderScene::new();
    let mut query = world.query::<(&Transform3, &Visual)>();
    for (transform, visual) in query.iter(world) {
        scene.items.push(RenderScene::item_from_visual(
            *transform,
            visual.shape.clone(),
            visual.color_rgba,
            visual.local_offset,
        ));
    }
    scene
}

fn append_city_ground(scene: &mut RenderScene) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation: Vec3::new(0.0, -0.08, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(46.0, 0.12, 36.0),
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: [0.09, 0.12, 0.14, 1.0],
        mesh: None,
    });
    push_box(
        scene,
        Vec3::new(0.0, 0.01, 0.0),
        Quat::IDENTITY,
        Vec3::new(6.0, 0.04, 36.0),
        [0.055, 0.065, 0.075, 1.0],
    );
    for offset in [-2.75, 2.75] {
        scene.items.push(RenderSceneItem {
            transform: MathTransform3 {
                translation: Vec3::new(offset, 0.01, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::new(0.12, 0.035, 34.0),
            },
            shape: VisualShape::Box { size_m: Vec3::ONE },
            color_rgba: [0.85, 0.76, 0.30, 1.0],
            mesh: None,
        });
    }
    for segment in -8..=8 {
        push_box(
            scene,
            Vec3::new(0.0, 0.035, segment as f64 * 2.0),
            Quat::IDENTITY,
            Vec3::new(0.10, 0.04, 1.0),
            [0.78, 0.82, 0.84, 1.0],
        );
    }
}

fn drone_position(progress: f64) -> Vec3 {
    let x = -20.0 + 40.0 * progress;
    let z = 8.5 - 17.0 * progress;
    let y = 14.5 + (progress * std::f64::consts::TAU).sin();
    Vec3::new(x, y, z)
}

fn append_flight_path(scene: &mut RenderScene, progress: f64) {
    let visible_markers = (progress * 20.0).floor() as usize;
    for marker in 0..=visible_markers {
        let marker_progress = marker as f64 / 20.0;
        let position = drone_position(marker_progress) - Vec3::new(0.0, 0.8, 0.0);
        push_box(
            scene,
            position,
            Quat::IDENTITY,
            Vec3::splat(0.24),
            [0.10, 0.70, 0.88, 0.75],
        );
    }
}

fn append_traffic(scene: &mut RenderScene, progress: f64) {
    let northbound = northbound_car_position(progress);
    let southbound = southbound_car_position(progress);
    append_car(scene, northbound, 0.0, [0.92, 0.22, 0.10, 1.0]);
    append_car(
        scene,
        southbound,
        std::f64::consts::PI,
        [0.12, 0.48, 0.92, 1.0],
    );
}

fn northbound_car_position(progress: f64) -> Vec3 {
    Vec3::new(-1.35, 0.48, -17.0 + 34.0 * progress)
}

fn southbound_car_position(progress: f64) -> Vec3 {
    let south_progress = (progress + 0.32).fract();
    Vec3::new(1.35, 0.48, 17.0 - 34.0 * south_progress)
}

fn append_car(scene: &mut RenderScene, center: Vec3, yaw_rad: f64, color_rgba: [f32; 4]) {
    let rotation = Quat::from_rotation_y(yaw_rad);
    push_box(
        scene,
        center,
        rotation,
        Vec3::new(1.4, 0.55, 2.8),
        color_rgba,
    );
    push_box(
        scene,
        center + Vec3::new(0.0, 0.43, -0.12),
        rotation,
        Vec3::new(1.08, 0.48, 1.30),
        [0.20, 0.29, 0.36, 1.0],
    );
    for (x, z) in [(-0.74, -0.90), (0.74, -0.90), (-0.74, 0.90), (0.74, 0.90)] {
        let offset = rotation * Vec3::new(x, -0.23, z);
        push_box(
            scene,
            center + offset,
            rotation,
            Vec3::new(0.22, 0.38, 0.52),
            [0.015, 0.02, 0.025, 1.0],
        );
    }
}

fn append_drone(scene: &mut RenderScene, center: Vec3, progress: f64) {
    let yaw = -0.4 + progress * 0.8;
    push_box(
        scene,
        center,
        Quat::from_rotation_y(yaw),
        Vec3::new(2.8, 0.70, 1.9),
        [0.09, 0.16, 0.22, 1.0],
    );
    for diagonal in [-1.0, 1.0] {
        push_box(
            scene,
            center + Vec3::new(0.0, 0.05, 0.0),
            Quat::from_rotation_y(yaw + diagonal * std::f64::consts::FRAC_PI_4),
            Vec3::new(5.6, 0.20, 0.20),
            [0.18, 0.26, 0.32, 1.0],
        );
    }
    let rotor_spin = progress * std::f64::consts::TAU * 10.0;
    for (x, z) in [(-2.0, -1.35), (-2.0, 1.35), (2.0, -1.35), (2.0, 1.35)] {
        let local = Quat::from_rotation_y(yaw) * Vec3::new(x, 0.18, z);
        push_box(
            scene,
            center + local,
            Quat::from_rotation_y(rotor_spin),
            Vec3::new(2.1, 0.08, 0.15),
            [0.20, 0.85, 0.95, 1.0],
        );
    }
    push_box(
        scene,
        center + Vec3::new(0.0, -0.42, 0.45),
        Quat::IDENTITY,
        Vec3::new(0.75, 0.55, 0.70),
        [0.92, 0.38, 0.12, 1.0],
    );
}

fn push_box(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    size_m: Vec3,
    color_rgba: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba,
        mesh: None,
    });
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
        .ok_or_else(|| std::io::Error::other("ffmpeg PLATEAU GIF encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drone_and_traffic_paths_are_deterministic_and_clear_buildings() {
        assert_eq!(drone_position(0.5), drone_position(0.5));
        for step in 0..=100 {
            let progress = step as f64 / 100.0;
            let northbound = northbound_car_position(progress);
            let southbound = southbound_car_position(progress);
            for car in [northbound, southbound] {
                assert!(car.x.abs() + 0.7 < 3.0);
                assert!(car.z >= -17.0 && car.z <= 17.0);
            }
        }
    }
}
