//! Imports a synthetic PLATEAU tile and renders drone and car traversal GIFs.

use png::{BitDepth, ColorType, Encoder};
use rne_assets::{load_scene_bundle, mesh_package_roots, spawn_scene_bundle, SpawnSceneOptions};
use rne_core::{SimClock, SimDuration};
use rne_ecs::{spawn_named, World};
use rne_math::{Hertz, Quat, Transform3 as MathTransform3, Vec3};
use rne_plateau::{import_citygml_file, ImportOptions, ImportedLane};
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, Visual, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    ackermann_kinematics, command_ackermann_drive, pure_pursuit_steering, AckermannDrive,
};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 720;
const HEIGHT: u32 = 405;
const DRONE_FRAME_COUNT: usize = 48;
const CAR_FRAME_COUNT: usize = 96;
const RENDER_HZ: usize = 12;
const SIM_HZ: usize = 60;
const SIM_STEPS_PER_FRAME: usize = SIM_HZ / RENDER_HZ;
const CLEAR_COLOR: [f32; 4] = [0.025, 0.045, 0.075, 1.0];

#[derive(Clone, Copy, Debug, PartialEq)]
struct VehicleFrame {
    transform: Transform3,
    speed_m_s: f64,
    steering_rad: f64,
    wheel_rotation_rad: f64,
}

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
    assert_eq!(result.road_count, 1);
    assert_eq!(result.lane_count, 2);
    assert_eq!(bundle.scene.objects.len(), 3);
    let (primary_traffic, opposing_traffic) =
        simulate_two_way_traffic(&result.lanes, CAR_FRAME_COUNT);
    println!(
        "PLATEAU tile ready: buildings={} roads={} lanes={} triangles={} scene={}",
        result.building_count,
        result.road_count,
        result.lane_count,
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
        let traffic_index = frame * (CAR_FRAME_COUNT - 1) / (DRONE_FRAME_COUNT - 1);
        let mut scene = city_scene.clone();
        append_flight_path(&mut scene, progress);
        append_traffic(
            &mut scene,
            primary_traffic[traffic_index],
            opposing_traffic[traffic_index],
        );
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
        let primary = primary_traffic[frame];
        let mut scene = city_scene.clone();
        append_traffic(&mut scene, primary, opposing_traffic[frame]);
        let car_camera = follow_camera(primary);
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

fn simulate_two_way_traffic(
    lanes: &[ImportedLane],
    frame_count: usize,
) -> (Vec<VehicleFrame>, Vec<VehicleFrame>) {
    assert_eq!(lanes.len(), 2, "example requires one derived two-way road");
    let mut ordered: Vec<&ImportedLane> = lanes.iter().collect();
    ordered.sort_by(|left, right| {
        let left_delta = left.centerline_m[1][2] - left.centerline_m[0][2];
        let right_delta = right.centerline_m[1][2] - right.centerline_m[0][2];
        right_delta.total_cmp(&left_delta)
    });
    (
        simulate_lane_vehicle(ordered[0], frame_count, 0.0),
        simulate_lane_vehicle(ordered[1], frame_count, 0.9),
    )
}

fn simulate_lane_vehicle(
    lane: &ImportedLane,
    frame_count: usize,
    start_delay_s: f64,
) -> Vec<VehicleFrame> {
    let fixed_delta = SimDuration::from_hertz(Hertz::new(SIM_HZ as f64));
    let mut clock = SimClock::new(fixed_delta);
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, format!("vehicle_{}", lane.lane_id));
    let start = Vec3::from_array(lane.centerline_m[0]) + Vec3::new(0.0, 0.65, 0.0);
    let end = Vec3::from_array(lane.centerline_m[1]) + Vec3::new(0.0, 0.65, 0.0);
    let direction = (end - start).normalize_or_zero();
    let yaw_rad = -direction.z.atan2(direction.x);
    let drive = AckermannDrive {
        max_speed_m_s: 7.0,
        max_acceleration_m_s2: 2.2,
        max_deceleration_m_s2: 4.5,
        max_steering_rate_rad_s: 0.7,
        ..AckermannDrive::default()
    };
    world.entity_mut(vehicle).insert((
        Transform3::from_translation_rotation(start, Quat::from_rotation_y(yaw_rad)),
        drive,
    ));
    let mut wheel_rotation_rad = 0.0;
    let mut frames = Vec::with_capacity(frame_count);
    for _ in 0..frame_count {
        let transform = *world.get::<Transform3>(vehicle).expect("vehicle transform");
        let drive = world
            .get::<AckermannDrive>(vehicle)
            .expect("Ackermann drive");
        frames.push(VehicleFrame {
            transform,
            speed_m_s: drive.speed_m_s,
            steering_rad: drive.steering_rad,
            wheel_rotation_rad,
        });
        for _ in 0..SIM_STEPS_PER_FRAME {
            let transform = *world.get::<Transform3>(vehicle).expect("vehicle transform");
            let drive = world
                .get::<AckermannDrive>(vehicle)
                .expect("Ackermann drive");
            let remaining_m = (end - transform.translation).dot(direction).max(0.0);
            let stopping_speed_m_s = (2.0 * drive.max_deceleration_m_s2 * remaining_m).sqrt();
            let target_speed_m_s = if clock.sim_time().as_seconds().value() < start_delay_s {
                0.0
            } else {
                6.0_f64.min(stopping_speed_m_s)
            };
            let steering_rad = pure_pursuit_steering(&transform, end, drive.wheelbase_m, 6.0);
            let _ = command_ackermann_drive(&mut world, vehicle, target_speed_m_s, steering_rad);
            assert_eq!(clock.advance(fixed_delta), 1);
            ackermann_kinematics(&mut world, clock.fixed_delta());
            let passed_endpoint = {
                let transform = world.get::<Transform3>(vehicle).expect("vehicle transform");
                (end - transform.translation).dot(direction) <= 0.0
            };
            if passed_endpoint {
                let mut transform = world
                    .get_mut::<Transform3>(vehicle)
                    .expect("vehicle transform");
                transform.translation.x = end.x;
                transform.translation.z = end.z;
                let mut drive = world
                    .get_mut::<AckermannDrive>(vehicle)
                    .expect("Ackermann drive");
                drive.speed_m_s = 0.0;
                drive.target_speed_m_s = 0.0;
                drive.steering_rad = 0.0;
                drive.target_steering_rad = 0.0;
            }
            let speed_m_s = world
                .get::<AckermannDrive>(vehicle)
                .expect("Ackermann drive")
                .speed_m_s;
            wheel_rotation_rad += speed_m_s * fixed_delta.as_seconds().value() / 0.36;
        }
    }
    frames
}

fn follow_camera(vehicle: VehicleFrame) -> CameraOrbit {
    let forward = vehicle.transform.rotation * Vec3::X;
    let right = vehicle.transform.rotation * Vec3::Z;
    let eye_direction = (-forward + right * 0.14).normalize_or_zero();
    CameraOrbit {
        focus: vehicle.transform.translation + forward * 4.0 + Vec3::new(0.0, 0.55, 0.0),
        yaw_rad: eye_direction.x.atan2(eye_direction.z),
        pitch_rad: 1.28,
        distance_m: 18.0,
    }
}

fn append_traffic(scene: &mut RenderScene, primary: VehicleFrame, opposing: VehicleFrame) {
    append_car(scene, primary, [0.84, 0.12, 0.045, 1.0]);
    append_car(scene, opposing, [0.045, 0.24, 0.72, 1.0]);
}

fn append_car(scene: &mut RenderScene, vehicle: VehicleFrame, color_rgba: [f32; 4]) {
    let center = vehicle.transform.translation;
    let rotation = vehicle.transform.rotation;
    push_box(
        scene,
        center,
        rotation,
        Vec3::new(4.35, 0.52, 1.82),
        color_rgba,
    );
    push_box(
        scene,
        center + rotation * Vec3::new(-0.15, 0.48, 0.0),
        rotation,
        Vec3::new(1.95, 0.65, 1.58),
        [0.12, 0.20, 0.27, 1.0],
    );
    push_box(
        scene,
        center + rotation * Vec3::new(0.82, 0.50, 0.0),
        rotation * Quat::from_rotation_z(-0.68),
        Vec3::new(0.08, 0.78, 1.50),
        [0.16, 0.26, 0.34, 1.0],
    );
    push_box(
        scene,
        center + rotation * Vec3::new(-1.12, 0.48, 0.0),
        rotation * Quat::from_rotation_z(0.64),
        Vec3::new(0.08, 0.72, 1.48),
        [0.14, 0.23, 0.30, 1.0],
    );
    for (x, color) in [
        (2.20, [0.98, 0.86, 0.42, 1.0]),
        (-2.20, [0.90, 0.04, 0.025, 1.0]),
    ] {
        for z in [-0.58, 0.58] {
            push_box(
                scene,
                center + rotation * Vec3::new(x, -0.02, z),
                rotation,
                Vec3::new(0.08, 0.18, 0.34),
                color,
            );
        }
    }
    for (x, z, steerable) in [
        (-1.34, -0.96, false),
        (-1.34, 0.96, false),
        (1.34, -0.96, true),
        (1.34, 0.96, true),
    ] {
        let wheel_rotation = rotation
            * Quat::from_rotation_y(if steerable { vehicle.steering_rad } else { 0.0 })
            * Quat::from_rotation_z(vehicle.wheel_rotation_rad);
        push_cylinder(
            scene,
            center + rotation * Vec3::new(x, -0.32, z),
            wheel_rotation,
            0.36,
            0.24,
            [0.012, 0.016, 0.020, 1.0],
        );
        push_box(
            scene,
            center + rotation * Vec3::new(x, -0.32, z),
            wheel_rotation,
            Vec3::new(0.58, 0.07, 0.26),
            [0.58, 0.61, 0.64, 1.0],
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

fn push_cylinder(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    radius_m: f64,
    length_m: f64,
    color_rgba: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Cylinder { radius_m, length_m },
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
    fn simclock_traffic_is_deterministic_and_stays_in_derived_lanes() {
        assert_eq!(drone_position(0.5), drone_position(0.5));
        let lanes = vec![
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-0".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[-1.5, 0.05, -17.0], [-1.5, 0.05, 17.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisPositive,
            },
            ImportedLane {
                lane_id: "road-main/surface-0000/lane-1".into(),
                road_source_id: "road-main".into(),
                centerline_m: [[1.5, 0.05, 17.0], [1.5, 0.05, -17.0]],
                width_m: 3.0,
                travel_direction: rne_plateau::LaneTravelDirection::PrincipalAxisNegative,
            },
        ];
        let first = simulate_two_way_traffic(&lanes, CAR_FRAME_COUNT);
        let second = simulate_two_way_traffic(&lanes, CAR_FRAME_COUNT);
        assert_eq!(first, second);
        assert!(first.0.last().unwrap().transform.translation.z > 12.0);
        for (lane, frames) in [(&lanes[0], &first.0), (&lanes[1], &first.1)] {
            let lane_x = lane.centerline_m[0][0];
            for frame in frames {
                assert!((frame.transform.translation.x - lane_x).abs() < 0.05);
                assert!(frame.transform.translation.z >= -17.05);
                assert!(frame.transform.translation.z <= 17.05);
            }
        }
    }
}
