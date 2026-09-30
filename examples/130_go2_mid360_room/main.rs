//! Walks the welded Go2 around a room while an upside-down Livox Mid-360 on its
//! back scans at 10 Hz, with the sensor model fitted to real Go2 recordings.
//!
//! * **Robot.** `unitree_go2_jump` on `rne_ai::UnitreeGo2ModelTrot`, steering toward
//!   waypoints around a table: the yaw-rate command turns the heading error to the
//!   next waypoint, as a path follower would.
//! * **Sensor.** `rne_sensor::sample_livox_mid360` with the Mid-360's non-repetitive
//!   pattern, the measured Go2 rig occlusion, and near-range blanking. Each frame is
//!   swept over the 0.1 s the robot moved during it, so the cloud carries the gait's
//!   motion distortion. The scene's raycasts skip the robot's own links; its body
//!   returns come from the measured rig table (`docs/LIVOX_MID360.md`).
//!
//! The gate checks the walk (every waypoint reached, clearance to furniture) and the
//! sensor against the recordings: while walking, the per-band no-return fractions
//! of the floor-facing elevations stay near the recorded ones.
//!
//! ```text
//! cargo run --release -p go2_mid360_room --example 130_go2_mid360_room -- --smoke
//! cargo run --release -p go2_mid360_room --example 130_go2_mid360_room
//! ```

use std::collections::{HashSet, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_mid360_mount, UnitreeGo2ModelTrot,
    UnitreeGo2TrotCommand, UrdfSceneSim,
};
use rne_data::PointCloud;
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_sensor::{
    livox_mid360_spec, LidarRigOcclusion, LidarSpec, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
};
use rne_world::Transform3 as WorldTransform3;

/// Waypoints `[x, z]` around the table, in meters; the loop ends near the start.
const WAYPOINTS: [[f64; 2]; 5] = [[3.0, 0.0], [3.0, 2.8], [-0.8, 2.8], [-0.8, 0.0], [0.6, 0.0]];
const WAYPOINT_RADIUS_M: f64 = 0.35;
const FORWARD_SPEED_M_S: f64 = 0.25;
/// Slower walking while the heading error is large, so corners stay tight.
const TURNING_SPEED_M_S: f64 = 0.12;
const TURNING_ERROR_RAD: f64 = 0.6;
const HEADING_GAIN_PER_S: f64 = 1.5;
const YAW_RATE_LIMIT_RAD_S: f64 = 0.5;
const MAX_DURATION_S: f64 = 110.0;
/// Control steps per Mid-360 frame: 0.1 s at 500 Hz.
const STEPS_PER_LIDAR_FRAME: u64 = 50;

/// Scene furniture and walls as `(centre [x, z], half extents [x, z])`.
const OBSTACLES: [([f64; 2], [f64; 2]); 8] = [
    ([4.6, 1.4], [0.05, 2.55]),
    ([-2.4, 1.4], [0.05, 2.55]),
    ([1.1, -1.1], [3.55, 0.05]),
    ([1.1, 3.9], [3.55, 0.05]),
    ([1.1, 1.4], [1.0, 0.5]),
    ([4.25, 2.8], [0.25, 0.7]),
    ([-1.9, -0.6], [0.3, 0.3]),
    ([3.9, -0.55], [0.25, 0.25]),
];

/// Recorded no-return fraction per 4° Livox-elevation band from 24° (EIL_Box and
/// EIL_Mask2, both walking); the model must fall between them within the tolerance.
const RECORDED_NO_RETURN: [[f64; 2]; 7] = [
    [0.338, 0.489],
    [0.343, 0.521],
    [0.410, 0.553],
    [0.533, 0.594],
    [0.673, 0.709],
    [0.772, 0.794],
    [0.991, 0.995],
];

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
/// A GIF frame every this many Mid-360 frames (0.8 s).
const LIDAR_FRAMES_PER_GIF_FRAME: u64 = 8;
/// Mid-360 frames drawn together: one 0.1 s frame, as the Livox viewer shows it.
const DRAWN_LIDAR_FRAMES: usize = 1;
const COLORMAP_BUCKETS: usize = 24;

fn repo_path(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

struct Run {
    sim: UrdfSceneSim,
    trot: UnitreeGo2ModelTrot,
    spec: LidarSpec,
    rig: LidarRigOcclusion,
    pattern: LivoxMid360Pattern,
    frame_index: u64,
    frame_start_pose: WorldTransform3,
    waypoint: usize,
    recent: VecDeque<PointCloud>,
    trail: Vec<[f64; 2]>,
    band_slots: [u64; 7],
    band_empty: [u64; 7],
    returns: u64,
    min_clearance_m: f64,
    min_height_m: f64,
}

impl Run {
    fn new() -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&repo_path(
            "assets/scenes/unitree_go2_mid360_room.rne.scene.toml",
        ))
        .expect("load Go2 room");
        let trot = UnitreeGo2ModelTrot::stand_up(&mut sim);
        let rig: LidarRigOcclusion = serde_json::from_str(
            &fs::read_to_string(repo_path(
                "assets/sensors/livox_mid360/go2_rig_occlusion.json",
            ))
            .expect("read rig table"),
        )
        .expect("parse rig table");
        let frame_start_pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        Self {
            sim,
            trot,
            spec: livox_mid360_spec(),
            rig,
            pattern: LivoxMid360Pattern::new(),
            frame_index: 0,
            frame_start_pose,
            waypoint: 0,
            recent: VecDeque::new(),
            trail: Vec::new(),
            band_slots: [0; 7],
            band_empty: [0; 7],
            returns: 0,
            min_clearance_m: f64::MAX,
            min_height_m: f64::MAX,
        }
    }

    fn done(&self) -> bool {
        self.waypoint >= WAYPOINTS.len() || self.trot.time_s() >= MAX_DURATION_S
    }

    /// Advances one Mid-360 frame: 50 control steps, then the frame's scan.
    fn step_frame(&mut self) {
        for _ in 0..STEPS_PER_LIDAR_FRAME {
            self.step_control();
        }
        let end_pose = self
            .sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let sweep = LidarSweep::new(self.frame_start_pose, end_pose);
        let cloud = self.sim.sample_livox_mid360(
            &sweep,
            &self.spec,
            &self.pattern,
            self.frame_index,
            Some(&self.rig),
            SensorNoiseKey::new(self.sim.world_seed(), self.spec.seed, 1, self.frame_index),
        );
        self.record_bands(&cloud);
        self.returns += cloud.points_m.len() as u64;
        self.recent.push_back(cloud);
        while self.recent.len() > DRAWN_LIDAR_FRAMES {
            self.recent.pop_front();
        }
        self.frame_start_pose = end_pose;
        self.frame_index += 1;
    }

    fn step_control(&mut self) {
        let observed = self.sim.observe();
        let position = [observed.base_x_m, observed.base_z_m];
        if let Some(target) = WAYPOINTS.get(self.waypoint) {
            if (target[0] - position[0]).hypot(target[1] - position[1]) < WAYPOINT_RADIUS_M {
                self.waypoint += 1;
            }
        }
        let command = match WAYPOINTS.get(self.waypoint) {
            None => UnitreeGo2TrotCommand::default(),
            Some(target) => {
                let base = self.sim.named_transform("base").expect("base pose");
                let facing = base.rotation * Vec3::X;
                // Heading is measured from +x toward -z (a left turn about +y).
                let yaw = (-facing.z).atan2(facing.x);
                let desired = (-(target[1] - position[1])).atan2(target[0] - position[0]);
                let error = wrap_angle(desired - yaw);
                UnitreeGo2TrotCommand {
                    forward_speed_m_s: if error.abs() > TURNING_ERROR_RAD {
                        TURNING_SPEED_M_S
                    } else {
                        FORWARD_SPEED_M_S
                    },
                    yaw_rate_rad_s: (HEADING_GAIN_PER_S * error)
                        .clamp(-YAW_RATE_LIMIT_RAD_S, YAW_RATE_LIMIT_RAD_S),
                }
            }
        };
        self.trot.step(&mut self.sim, command);
        if self.trot.steps().is_multiple_of(125) {
            self.trail.push(position);
        }
        let after = self.sim.observe();
        self.min_height_m = self.min_height_m.min(after.base_y_m);
        self.min_clearance_m = self
            .min_clearance_m
            .min(clearance_m([after.base_x_m, after.base_z_m]));
    }

    fn record_bands(&mut self, cloud: &PointCloud) {
        let returned: HashSet<(u32, u16)> = cloud
            .ray_indices
            .iter()
            .copied()
            .zip(cloud.channel_indices.iter().copied())
            .collect();
        for ray in self.pattern.frame_rays(self.frame_index) {
            let band = ((ray.elevation_rad.to_degrees() - 24.0) / 4.0).floor();
            if !(0.0..7.0).contains(&band) {
                continue;
            }
            let band = band as usize;
            self.band_slots[band] += 1;
            if !returned.contains(&(ray.column, ray.channel)) {
                self.band_empty[band] += 1;
            }
        }
    }

    fn band_no_return(&self) -> Vec<f64> {
        (0..7)
            .map(|band| self.band_empty[band] as f64 / self.band_slots[band].max(1) as f64)
            .collect()
    }
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Distance from the base centre to the nearest wall or piece of furniture.
fn clearance_m(position: [f64; 2]) -> f64 {
    OBSTACLES
        .iter()
        .map(|(center, half)| {
            let dx = ((position[0] - center[0]).abs() - half[0]).max(0.0);
            let dz = ((position[1] - center[1]).abs() - half[1]).max(0.0);
            dx.hypot(dz)
        })
        .fold(f64::MAX, f64::min)
}

fn run(mut on_frame: impl FnMut(&Run)) -> Run {
    let mut run = Run::new();
    while !run.done() {
        run.step_frame();
        on_frame(&run);
    }
    run
}

fn report_and_gate(run: &Run) {
    let bands = run.band_no_return();
    println!(
        "walked the loop in {:.1} s: waypoints {}/{}, clearance >= {:.2} m, lowest body {:.3} m",
        run.trot.time_s(),
        run.waypoint.min(WAYPOINTS.len()),
        WAYPOINTS.len(),
        run.min_clearance_m,
        run.min_height_m
    );
    println!(
        "Mid-360: {} frames, {:.0} returns per frame; floor-band no-return {}",
        run.frame_index,
        run.returns as f64 / run.frame_index.max(1) as f64,
        bands
            .iter()
            .zip(RECORDED_NO_RETURN)
            .enumerate()
            .map(|(index, (model, recorded))| format!(
                "{}°: {model:.3} (rec {:.3}-{:.3})",
                24 + 4 * index,
                recorded[0],
                recorded[1]
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    assert_eq!(run.waypoint, WAYPOINTS.len(), "the loop was not completed");
    assert!(
        run.min_height_m > 0.2,
        "the walk sagged: {:.3} m",
        run.min_height_m
    );
    assert!(
        run.min_clearance_m > 0.35,
        "came within {:.2} m of an obstacle",
        run.min_clearance_m
    );
    for (index, (model, recorded)) in bands.iter().zip(RECORDED_NO_RETURN).enumerate() {
        assert!(
            *model > recorded[0] - 0.06 && *model < recorded[1] + 0.06,
            "band {}°: no-return {model:.3} outside recorded {:.3}-{:.3}",
            24 + 4 * index,
            recorded[0],
            recorded[1]
        );
    }
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke") {
        let run = run(|_| {});
        report_and_gate(&run);
        println!("smoke ok: the Go2 walked the room loop with a recording-matched Mid-360");
        return;
    }
    render_media();
}

fn render_media() {
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let frames_dir = repo_path("target/rne-go2-mid360-room-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut frame = 0_usize;
    let run = run(|run| {
        if !run.frame_index.is_multiple_of(LIDAR_FRAMES_PER_GIF_FRAME) {
            return;
        }
        let rgba = render_frame(&mut backend, &camera, &mut mesh_cache, run);
        write_png(&frames_dir.join(format!("frame-{frame:03}.png")), &rgba).expect("write frame");
        frame += 1;
    });
    report_and_gate(&run);

    let media_dir = repo_path("docs/media");
    let gif_path = media_dir.join("go2-mid360-room.gif");
    build_gif(&frames_dir, &gif_path).expect("encode gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", frame / 3)))
        .expect("read poster frame")
        .save(media_dir.join("go2-mid360-room.png"))
        .expect("write poster");
    println!("wrote {} ({frame} frames)", gif_path.display());
}

fn render_frame(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    run: &Run,
) -> Vec<u8> {
    // One fixed view over the room: the background stays still between frames, so the
    // GIF spends its bytes on the robot and the returns.
    let view = CameraOrbit {
        focus: Vec3::new(1.1, 0.0, 1.5),
        yaw_rad: -2.3,
        pitch_rad: 0.78,
        distance_m: 5.3,
    };
    let mut scene = build_visual_render_scene(run.sim.world());
    // Darker walls and furniture, so the returns on them read.
    for item in &mut scene.items {
        if matches!(item.shape, VisualShape::Box { .. }) {
            for channel in &mut item.color_rgba[..3] {
                *channel *= 0.45;
            }
        }
    }
    // Replace the ground plane with a dark floor the points read against.
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { size_m } if size_m.x >= 20.0));
    scene.items.push(box_item(
        Vec3::new(1.1, -0.005, 1.4),
        Vec3::new(7.0, 0.01, 5.0),
        [0.07, 0.09, 0.12, 1.0],
    ));
    for position in &run.trail {
        scene.items.push(box_item(
            Vec3::new(position[0], 0.004, position[1]),
            Vec3::new(0.05, 0.004, 0.05),
            [0.98, 0.84, 0.25, 1.0],
        ));
    }
    append_height_colored_points(&mut scene, run.recent.iter());
    // The Mid-360 itself: 65 x 65 x 60 mm, hanging upside down from its mount.
    let mount = run
        .sim
        .named_mount_transform("base", &unitree_go2_mid360_mount())
        .expect("mount pose");
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation: mount.translation,
            // The cylinder's axis is local z; the sensor's up axis is local y.
            rotation: mount.rotation * Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            // The renderer scales a unit cylinder, as it does a unit box.
            scale: Vec3::new(0.065, 0.065, 0.06),
        },
        shape: VisualShape::Cylinder {
            radius_m: 0.5,
            length_m: 1.0,
        },
        color_rgba: [0.12, 0.12, 0.13, 1.0],
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
    let roots: Vec<&Path> = run
        .sim
        .mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect();
    mesh_cache
        .resolve_scene(&mut scene, &roots)
        .expect("resolve official Go2 meshes");
    backend
        .render_scene_camera(camera, &view.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render frame")
        .color
        .rgba8
}

fn box_item(translation: Vec3, scale: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

/// Draws returns as small markers coloured by height, turbo from floor to 1 m.
fn append_height_colored_points<'a>(
    scene: &mut RenderScene,
    clouds: impl Iterator<Item = &'a PointCloud>,
) {
    let mut buckets: Vec<PointMesh> = (0..COLORMAP_BUCKETS)
        .map(|_| PointMesh::default())
        .collect();
    for cloud in clouds {
        for point in &cloud.points_m {
            let t = (point.y / 1.0).clamp(0.0, 1.0);
            let bucket =
                ((t * (COLORMAP_BUCKETS - 1) as f64).round() as usize).min(COLORMAP_BUCKETS - 1);
            buckets[bucket].add_marker(*point, 0.024);
        }
    }
    for (bucket, mesh) in buckets.into_iter().enumerate() {
        if mesh.indices.is_empty() {
            continue;
        }
        let t = bucket as f64 / (COLORMAP_BUCKETS - 1) as f64;
        scene.items.push(RenderSceneItem {
            transform: Transform3::IDENTITY,
            shape: VisualShape::DynamicMesh,
            color_rgba: turbo_colormap(0.1 + 0.85 * t),
            mesh: Some(Arc::new(TriangleMesh {
                positions: mesh.positions,
                normals: mesh.normals,
                texcoords: mesh.texcoords,
                indices: mesh.indices,
                skinning: None,
            })),
            base_color_texture: None,
            material: Default::default(),
        });
    }
}

#[derive(Default)]
struct PointMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl PointMesh {
    /// Three crossed quads, so the marker reads from any view direction.
    fn add_marker(&mut self, center: Vec3, radius_m: f64) {
        let x = Vec3::X * radius_m;
        let y = Vec3::Y * radius_m;
        let z = Vec3::Z * radius_m;
        self.add_quad(center - x, center + y, center - y, center + x);
        self.add_quad(center - z, center + y, center - y, center + z);
        self.add_quad(center - x, center + z, center - z, center + x);
    }

    fn add_quad(&mut self, first: Vec3, second: Vec3, third: Vec3, fourth: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend([first, second, third, fourth].map(|p| [p.x as f32, p.y as f32, p.z as f32]));
        self.normals.extend([[0.0, 1.0, 0.0]; 4]);
        self.texcoords.extend([[0.0, 0.0]; 4]);
        self.indices
            .extend([base, base + 1, base + 2, base + 2, base + 1, base + 3]);
    }
}

/// Google's Turbo colormap, polynomial approximation.
fn turbo_colormap(t: f64) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let polynomial = |c: [f64; 6]| -> f32 {
        (c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))).clamp(0.0, 1.0) as f32
    };
    [
        polynomial([
            0.135_721_38,
            4.615_392_60,
            -42.660_322_58,
            132.131_082_34,
            -152.942_393_96,
            59.286_379_43,
        ]),
        polynomial([
            0.091_402_61,
            2.194_188_39,
            4.842_966_58,
            -14.185_033_33,
            4.277_298_57,
            2.829_566_04,
        ]),
        polynomial([
            0.106_673_30,
            12.641_946_08,
            -60.582_048_36,
            110.362_767_71,
            -89.903_109_12,
            27.348_249_73,
        ]),
        1.0,
    ]
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-framerate", "8", "-i"])
        .arg(frames_dir.join("frame-%03d.png"))
        .args([
            "-vf",
            "scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle",
        ])
        .arg(gif_path)
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}
