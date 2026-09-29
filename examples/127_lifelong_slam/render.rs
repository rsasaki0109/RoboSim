//! Draws the week: each day the robot drives its loop over the map it woke up
//! with, then the map updates and the cells that changed are highlighted.
//!
//! Everything map-related is drawn from the lifelong map itself, in its own
//! frame, placed in the world by the first day's start pose, so any error in
//! the map shows as an offset from the building rather than being hidden.

use crate::warehouse::{self, Footprint, STATIC};
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_nav::{GridCoord, LaserScan2d, OccupancyGrid, Pose2d};
use rne_render::{
    grid_mesh, Camera, EnvironmentLighting, EnvironmentMap, GridMeshSpec, ImageFrame,
    MeshRenderCache, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh,
    VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const DRIVE_FRAMES: usize = 26;
const UPDATE_FRAMES: usize = 10;
const CLEAR_COLOR: [f32; 4] = [0.20, 0.22, 0.26, 1.0];

/// What one day contributes to the picture.
pub(crate) struct DayVisual {
    pub day: usize,
    /// The day's true path, one pose per scan.
    pub path: Vec<Pose2d>,
    /// The LiDAR scan at each pose of `path`, in the base frame.
    pub live: Vec<LaserScan2d>,
    /// The map the robot woke up with; `None` on the first day.
    pub prior: Option<OccupancyGrid>,
    /// The map after the day was merged and pruned.
    pub after: OccupancyGrid,
    /// Cells the day turned occupied or free, in the map frame.
    pub appeared: Vec<GridCoord>,
    pub vanished: Vec<GridCoord>,
}

/// Renders the week to `frames_dir` and encodes `gif_path`.
pub(crate) fn render_week(
    days: &[DayVisual],
    world_from_map: Pose2d,
    frames_dir: &Path,
    gif_path: &Path,
) -> std::io::Result<()> {
    let _ = fs::remove_dir_all(frames_dir);
    fs::create_dir_all(frames_dir)?;
    let mut backend = WgpuRenderBackend::new().map_err(std::io::Error::other)?;
    backend.set_environment(environment());
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(0.0, 1.2, -1.0),
        yaw_rad: 0.55,
        pitch_rad: 0.9,
        distance_m: 16.5,
    };
    let mut cache = MeshRenderCache::new();
    let props = props_root();
    let mut index = 0;
    for visual in days {
        for frame in 0..DRIVE_FRAMES + UPDATE_FRAMES {
            let mut scene = RenderScene::default();
            push_building(&mut scene, visual.day);
            let (map, highlight) = if frame < DRIVE_FRAMES {
                (visual.prior.as_ref(), false)
            } else {
                (Some(&visual.after), true)
            };
            if let Some(map) = map {
                push_map(&mut scene, map, world_from_map);
            }
            if highlight {
                push_changes(&mut scene, &visual.after, visual, world_from_map);
            }
            let robot = visual.path[if frame < DRIVE_FRAMES {
                (frame * (visual.path.len() - 1)) / (DRIVE_FRAMES - 1)
            } else {
                visual.path.len() - 1
            }];
            push_map_screen(
                &mut scene,
                map,
                highlight.then_some(visual),
                world_from_map,
                robot,
            );
            let step = if frame < DRIVE_FRAMES {
                (frame * (visual.path.len() - 1)) / (DRIVE_FRAMES - 1)
            } else {
                visual.path.len() - 1
            };
            push_drive(&mut scene, visual, step);
            cache
                .resolve_scene(&mut scene, &[props.as_path()])
                .map_err(std::io::Error::other)?;
            let output = backend
                .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
                .map_err(std::io::Error::other)?;
            write_png(
                &frames_dir.join(format!("frame-{index:03}.png")),
                &output.color.rgba8,
            )?;
            index += 1;
        }
    }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/encode_gif.py");
    let status = std::process::Command::new("python3")
        .arg(script)
        .arg(frames_dir)
        .arg(gif_path)
        .args(["--fps", "10", "--colors", "192"])
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("encode_gif.py failed"));
    }
    Ok(())
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = png::Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}

fn props_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_warehouse")
}

/// A synthesised warehouse interior to light the scene by.
fn environment() -> EnvironmentLighting {
    const W: u32 = 128;
    const H: u32 = 64;
    let mut rgba32f = Vec::with_capacity((W * H * 4) as usize);
    for row in 0..H {
        let down = (row as f32 + 0.5) / H as f32;
        for column in 0..W {
            let around = (column as f32 + 0.5) / W as f32;
            let (r, g, b) = if down < 0.30 {
                let bands = (around * std::f32::consts::TAU * 4.0).sin().max(0.0);
                let base = 0.75 + 0.85 * (1.0 - down / 0.30);
                let fixture = 4.5 * bands.powf(18.0);
                (base + fixture, base + fixture, base * 1.03 + fixture)
            } else if down < 0.62 {
                let level = 0.52 - 0.26 * (down - 0.30) / 0.32;
                (level * 0.94, level * 0.97, level)
            } else {
                (0.15, 0.145, 0.135)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(W, H, rgba32f).expect("environment map"),
        )),
        intensity: 1.0,
        diffuse_strength: 0.72,
        specular_strength: 0.38,
        rotation_rad: 0.0,
    }
}

fn push_box(scene: &mut RenderScene, center: Vec3, half: Vec3, color: [f32; 4], rough: f32) {
    push_box_rotated(scene, center, Quat::IDENTITY, half, color, rough, [0.0; 3]);
}

fn push_box_rotated(
    scene: &mut RenderScene,
    center: Vec3,
    rotation: Quat,
    half: Vec3,
    color: [f32; 4],
    rough: f32,
    emissive: [f32; 3],
) {
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, rough, 0.05, emissive);
    scene.items.push(item);
}

fn push_sphere(scene: &mut RenderScene, center: Vec3, radius: f64, color: [f32; 4], glow: bool) {
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center, Quat::IDENTITY),
        VisualShape::Sphere { radius_m: radius },
        color,
        Transform3::IDENTITY,
    );
    let emissive = if glow {
        [color[0], color[1], color[2]]
    } else {
        [0.0; 3]
    };
    item.material = PbrMaterial::new(color, 0.4, 0.0, emissive);
    scene.items.push(item);
}

fn push_prop(scene: &mut RenderScene, path: &str, base: Vec3, yaw: f64, scale: Vec3) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(base, Quat::from_rotation_y(yaw)),
        VisualShape::Mesh {
            path: path.to_string(),
            scale,
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

struct Concrete {
    color: Arc<ImageFrame>,
    normal: Arc<ImageFrame>,
    roughness: Arc<ImageFrame>,
}

fn concrete() -> &'static Concrete {
    static CONCRETE: OnceLock<Concrete> = OnceLock::new();
    CONCRETE.get_or_init(|| {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../63_g1_stride_gif/assets/photoreal_test_bay");
        let load = |name: &str| {
            let rgba = image::open(root.join(name))
                .expect("load concrete texture")
                .into_rgba8();
            Arc::new(ImageFrame::from_rgba8(
                rgba.width(),
                rgba.height(),
                rgba.into_raw(),
            ))
        };
        Concrete {
            color: load("concrete_floor_basecolor.png"),
            normal: load("concrete_floor_normal.png"),
            roughness: load("concrete_floor_roughness.png"),
        }
    })
}

/// The concrete floor, one texture repeat per 1.5 m.
fn push_floor(scene: &mut RenderScene) {
    let (half_x, half_z) = (7.2_f32, 4.6_f32);
    let (repeat_x, repeat_z) = (half_x / 0.75, half_z / 0.75);
    let mesh = TriangleMesh {
        positions: vec![
            [-half_x, 0.0, -half_z],
            [half_x, 0.0, -half_z],
            [half_x, 0.0, half_z],
            [-half_x, 0.0, half_z],
        ],
        normals: vec![[0.0, 1.0, 0.0]; 4],
        texcoords: vec![
            [0.0, 0.0],
            [repeat_x, 0.0],
            [repeat_x, repeat_z],
            [0.0, repeat_z],
        ],
        // Counter-clockwise seen from above.
        indices: vec![0, 2, 1, 0, 3, 2],
        skinning: None,
    };
    let surfaces = concrete();
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: [1.0; 4],
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&surfaces.color)),
        material: PbrMaterial::new([1.0; 4], 0.92, 0.0, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&surfaces.normal)),
            Some(Arc::clone(&surfaces.roughness)),
        ),
    });
}

/// Walls, racks, the column, the office block, the day's pallets, and a row
/// of lamps on the north wall counting the days.
fn push_building(scene: &mut RenderScene, day: usize) {
    const WALL: [f32; 4] = [0.62, 0.64, 0.67, 1.0];
    push_floor(scene);
    for (index, footprint) in STATIC.iter().enumerate() {
        match index {
            // The north and east walls stand between the camera and the floor:
            // drawn as a low curb so they frame it without hiding it.
            0 | 2 => push_box(
                scene,
                Vec3::new(footprint.x, 0.15, footprint.z),
                Vec3::new(footprint.half_x, 0.15, footprint.half_z),
                WALL,
                0.9,
            ),
            1 | 3 => push_box(
                scene,
                Vec3::new(footprint.x, 0.5 * footprint.height, footprint.z),
                Vec3::new(footprint.half_x, 0.5 * footprint.height, footprint.half_z),
                WALL,
                0.9,
            ),
            4 | 5 => push_rack(scene, footprint),
            6 => push_box(
                scene,
                Vec3::new(footprint.x, 0.5 * footprint.height, footprint.z),
                Vec3::new(footprint.half_x, 0.5 * footprint.height, footprint.half_z),
                [0.85, 0.72, 0.18, 1.0],
                0.6,
            ),
            _ => push_office(scene, footprint),
        }
    }
    for pallet in warehouse::pallets(day) {
        push_pallet(scene, &pallet);
    }
    for lamp in 0..warehouse::DAYS.len() {
        let lit = lamp <= day;
        let color = if lit {
            [0.25, 1.0, 0.45, 1.0]
        } else {
            [0.12, 0.13, 0.15, 1.0]
        };
        push_box_rotated(
            scene,
            Vec3::new(3.6 + 0.8 * lamp as f64, 2.1, -4.48),
            Quat::IDENTITY,
            Vec3::new(0.28, 0.18, 0.02),
            color,
            0.3,
            if lit { [0.25, 1.0, 0.45] } else { [0.0; 3] },
        );
    }
}

/// Pallet racking: blue uprights, orange beams, boxed stock on each level.
fn push_rack(scene: &mut RenderScene, footprint: &Footprint) {
    const UPRIGHT: [f32; 4] = [0.16, 0.36, 0.60, 1.0];
    const BEAM: [f32; 4] = [0.92, 0.55, 0.10, 1.0];
    let bays = (2.0 * footprint.half_x / 1.1).round().max(1.0) as usize;
    let bay = 2.0 * footprint.half_x / bays as f64;
    for post in 0..=bays {
        let x = footprint.x - footprint.half_x + post as f64 * bay;
        for side in [-1.0, 1.0] {
            push_box(
                scene,
                Vec3::new(
                    x,
                    0.5 * footprint.height,
                    footprint.z + side * (footprint.half_z - 0.04),
                ),
                Vec3::new(0.04, 0.5 * footprint.height, 0.04),
                UPRIGHT,
                0.5,
            );
        }
    }
    for level in [0.15, 0.85, 1.55] {
        for side in [-1.0, 1.0] {
            push_box(
                scene,
                Vec3::new(
                    footprint.x,
                    level,
                    footprint.z + side * (footprint.half_z - 0.04),
                ),
                Vec3::new(footprint.half_x, 0.05, 0.03),
                BEAM,
                0.5,
            );
        }
        for slot in 0..bays {
            let x = footprint.x - footprint.half_x + (slot as f64 + 0.5) * bay;
            push_prop(
                scene,
                "cardboard_box_01/cardboard_box_01_1k.gltf",
                Vec3::new(x, level + 0.05, footprint.z),
                0.1 * slot as f64,
                Vec3::new(1.4, 1.2, 1.2),
            );
        }
    }
}

/// The office block, cut away to waist height like the near walls, with a
/// glazed band on the side facing the floor.
fn push_office(scene: &mut RenderScene, footprint: &Footprint) {
    push_box(
        scene,
        Vec3::new(footprint.x, 0.45, footprint.z),
        Vec3::new(footprint.half_x, 0.45, footprint.half_z),
        [0.78, 0.76, 0.70, 1.0],
        0.8,
    );
    push_box_rotated(
        scene,
        Vec3::new(footprint.x - footprint.half_x - 0.005, 0.6, footprint.z),
        Quat::IDENTITY,
        Vec3::new(0.01, 0.2, footprint.half_z - 0.15),
        [0.35, 0.55, 0.70, 1.0],
        0.15,
        [0.05, 0.08, 0.10],
    );
}

/// A wooden pallet stacked with three layers of four cartons.
fn push_pallet(scene: &mut RenderScene, pallet: &Footprint) {
    push_box(
        scene,
        Vec3::new(pallet.x, 0.07, pallet.z),
        Vec3::new(pallet.half_x, 0.07, pallet.half_z),
        [0.60, 0.45, 0.28, 1.0],
        0.9,
    );
    for layer in 0..3 {
        for (dx, dz) in [(-0.22, -0.22), (0.22, -0.22), (-0.22, 0.22), (0.22, 0.22)] {
            push_prop(
                scene,
                "cardboard_box_01/cardboard_box_01_1k.gltf",
                Vec3::new(pallet.x + dx, 0.14 + 0.35 * f64::from(layer), pallet.z + dz),
                0.05 * f64::from(layer),
                Vec3::new(1.1, 1.0, 0.82),
            );
        }
    }
}

fn map_item(mesh: TriangleMesh, color: [f32; 4], world_from_map: Pose2d) -> RenderSceneItem {
    let mut item = RenderScene::item_from_dynamic_mesh(mesh, color);
    item.transform.translation = Vec3::new(world_from_map.x_m, 0.0, world_from_map.y_m);
    item.transform.rotation = Quat::from_rotation_y(-world_from_map.yaw_rad);
    item.material = PbrMaterial::new(color, 0.6, 0.0, [0.0; 3]);
    item
}

fn map_spec(grid: &OccupancyGrid, height_m: f64, fill: f64) -> GridMeshSpec {
    GridMeshSpec {
        columns: grid.width(),
        rows: grid.height(),
        cell_size_m: grid.resolution_m(),
        origin_m: Vec3::new(grid.origin().x_m, 0.0, grid.origin().y_m),
        height_m,
        fill,
    }
}

/// The lifelong map's occupied cells, as dark tiles on the floor.
fn push_map(scene: &mut RenderScene, grid: &OccupancyGrid, world_from_map: Pose2d) {
    let mesh = grid_mesh(&map_spec(grid, 0.02, 1.0), |column, row| {
        grid.is_occupied(GridCoord {
            x: column as isize,
            y: row as isize,
        })
    });
    if !mesh.indices.is_empty() {
        scene
            .items
            .push(map_item(mesh, [0.05, 0.06, 0.08, 1.0], world_from_map));
    }
}

/// The day's changes: green where the map gained an obstacle, red where it
/// lost one.
fn push_changes(
    scene: &mut RenderScene,
    grid: &OccupancyGrid,
    visual: &DayVisual,
    world_from_map: Pose2d,
) {
    for (cells, color) in [
        (&visual.appeared, [0.20, 0.95, 0.40, 1.0]),
        (&visual.vanished, [1.0, 0.22, 0.18, 1.0]),
    ] {
        if cells.is_empty() {
            continue;
        }
        let set: std::collections::HashSet<(isize, isize)> =
            cells.iter().map(|coord| (coord.x, coord.y)).collect();
        let mesh = grid_mesh(&map_spec(grid, 0.03, 1.0), |column, row| {
            set.contains(&(column as isize, row as isize))
        });
        let mut item = map_item(mesh, color, world_from_map);
        item.material = PbrMaterial::new(color, 0.4, 0.0, [color[0], color[1], color[2]]);
        scene.items.push(item);
    }
}

/// The robot at `step` of its path, its trail so far, and the scan its LiDAR
/// returns there: a beam from the LiDAR head to every return, and the return
/// itself marked where it struck.
fn push_drive(scene: &mut RenderScene, visual: &DayVisual, step: usize) {
    const TRAIL: [f32; 4] = [0.15, 0.80, 0.95, 1.0];
    for pose in visual.path[..=step].iter().step_by(5) {
        push_sphere(
            scene,
            Vec3::new(pose.x_m, 0.05, pose.y_m),
            0.04,
            TRAIL,
            false,
        );
    }
    let pose = visual.path[step];
    push_robot(scene, pose);
    push_scan(scene, pose, &visual.live[step]);
}

/// Height of the drawn scan plane: the LiDAR head on the robot.
const SCAN_HEIGHT_M: f64 = 0.46;
/// Every how many of the scan's 360 beams is drawn as a ray.
const RAY_STRIDE: usize = 6;
/// Distance from the LiDAR at which a drawn ray starts: just outside the
/// robot's body.
const RAY_START_M: f64 = 0.38;
/// Adjacent returns closer than this are joined into the scan's outline.
const OUTLINE_GAP_M: f64 = 0.35;

fn push_scan(scene: &mut RenderScene, pose: Pose2d, scan: &LaserScan2d) {
    const RAY: [f32; 4] = [0.30, 0.80, 1.0, 1.0];
    const OUTLINE: [f32; 4] = [1.0, 0.22, 0.10, 1.0];
    const HIT: [f32; 4] = [1.0, 0.85, 0.20, 1.0];
    let world = |x: f64, y: f64| {
        let point = pose.transform_point(Vec3::new(x, y, 0.0));
        Vec3::new(point.x, SCAN_HEIGHT_M, point.y)
    };
    let returns: Vec<Option<Vec3>> = scan
        .ranges_m
        .iter()
        .enumerate()
        .map(|(beam, range)| {
            (range.is_finite() && *range <= scan.range_max_m).then(|| {
                let angle = scan.angle_min_rad + scan.angle_increment_rad * beam as f64;
                world(range * angle.cos(), range * angle.sin())
            })
        })
        .collect();
    let mut rays = SolidMesh::default();
    let mut outline = SolidMesh::default();
    let mut hits = SolidMesh::default();
    for (beam, hit) in returns.iter().enumerate() {
        let Some(hit) = *hit else { continue };
        if beam.is_multiple_of(RAY_STRIDE) {
            // Rays start at the edge of the robot's body so they do not
            // paint over it.
            let angle = scan.angle_min_rad + scan.angle_increment_rad * beam as f64;
            let start = world(RAY_START_M * angle.cos(), RAY_START_M * angle.sin());
            if (hit - start).dot(start - world(0.0, 0.0)) > 0.0 {
                rays.ribbon(start, hit, 0.018);
            }
            hits.cube(hit, 0.07);
        }
        if let Some(next) = returns[(beam + 1) % returns.len()] {
            if (next - hit).length() < OUTLINE_GAP_M {
                outline.bar(hit, next, 0.05);
            }
        }
    }
    for (mesh, color, glow) in [(rays, RAY, 0.55), (outline, OUTLINE, 1.0), (hits, HIT, 1.0)] {
        if let Some(mesh) = mesh.build() {
            let mut item = RenderScene::item_from_dynamic_mesh(mesh, color);
            let emissive = [color[0] * glow, color[1] * glow, color[2] * glow];
            item.material = PbrMaterial::new(color, 0.5, 0.0, emissive);
            scene.items.push(item);
        }
    }
}

/// Quads in the world frame gathered into one mesh, so a scan is a few
/// draws rather than hundreds of scene items.
#[derive(Default)]
struct SolidMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
}

impl SolidMesh {
    fn quad(&mut self, corners: [Vec3; 4], normal: Vec3) {
        let base = self.positions.len() as u32;
        for corner in corners {
            self.positions
                .push([corner.x as f32, corner.y as f32, corner.z as f32]);
            self.normals
                .push([normal.x as f32, normal.y as f32, normal.z as f32]);
        }
        let facing = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
        if facing.dot(normal) >= 0.0 {
            self.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        } else {
            self.indices
                .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
        }
    }

    /// A thin strip from `from` to `to`, lying in the scan plane.
    fn ribbon(&mut self, from: Vec3, to: Vec3, width_m: f64) {
        let along = to - from;
        let side = Vec3::new(-along.z, 0.0, along.x).normalize_or_zero() * (width_m * 0.5);
        self.quad([from - side, to - side, to + side, from + side], Vec3::Y);
    }

    /// A square bar from `from` to `to`.
    fn bar(&mut self, from: Vec3, to: Vec3, size_m: f64) {
        let along = to - from;
        let side = Vec3::new(-along.z, 0.0, along.x).normalize_or_zero() * (size_m * 0.5);
        let up = Vec3::Y * (size_m * 0.5);
        self.quad(
            [
                from - side + up,
                to - side + up,
                to + side + up,
                from + side + up,
            ],
            Vec3::Y,
        );
        self.quad(
            [
                from - side - up,
                to - side - up,
                to - side + up,
                from - side + up,
            ],
            -side,
        );
        self.quad(
            [
                from + side - up,
                to + side - up,
                to + side + up,
                from + side + up,
            ],
            side,
        );
    }

    /// A cube centered on `at`.
    fn cube(&mut self, at: Vec3, size_m: f64) {
        let h = size_m * 0.5;
        for normal in [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z] {
            let (u, v) = if normal.y.abs() > 0.5 {
                (Vec3::X, Vec3::Z)
            } else if normal.x.abs() > 0.5 {
                (Vec3::Y, Vec3::Z)
            } else {
                (Vec3::X, Vec3::Y)
            };
            let c = at + normal * h;
            self.quad(
                [
                    c - u * h - v * h,
                    c + u * h - v * h,
                    c + u * h + v * h,
                    c - u * h + v * h,
                ],
                normal,
            );
        }
    }

    fn build(self) -> Option<TriangleMesh> {
        (!self.indices.is_empty()).then(|| TriangleMesh {
            texcoords: vec![[0.0, 0.0]; self.positions.len()],
            positions: self.positions,
            normals: self.normals,
            indices: self.indices,
            skinning: None,
        })
    }
}

/// A small AGV: body, bumper, LiDAR puck and a status light, facing its
/// heading.
fn push_robot(scene: &mut RenderScene, pose: Pose2d) {
    let rotation = Quat::from_rotation_y(-pose.yaw_rad);
    let at =
        |x: f64, y: f64, z: f64| Vec3::new(pose.x_m, 0.0, pose.y_m) + rotation * Vec3::new(x, y, z);
    push_box_rotated(
        scene,
        at(0.0, 0.2, 0.0),
        rotation,
        Vec3::new(0.32, 0.14, 0.24),
        [0.95, 0.55, 0.10, 1.0],
        0.45,
        [0.0; 3],
    );
    push_box_rotated(
        scene,
        at(0.34, 0.12, 0.0),
        rotation,
        Vec3::new(0.03, 0.06, 0.24),
        [0.08, 0.08, 0.09, 1.0],
        0.8,
        [0.0; 3],
    );
    push_box_rotated(
        scene,
        at(0.05, 0.42, 0.0),
        rotation,
        Vec3::new(0.07, 0.08, 0.07),
        [0.10, 0.10, 0.12, 1.0],
        0.4,
        [0.0; 3],
    );
    push_box_rotated(
        scene,
        at(0.05, 0.505, 0.0),
        rotation,
        Vec3::new(0.071, 0.006, 0.071),
        [0.20, 0.75, 1.0, 1.0],
        0.3,
        [0.20, 0.75, 1.0],
    );
}

/// World extent the map screen shows: the building plus a margin.
const SCREEN_WORLD: (f64, f64, f64, f64) = (-7.5, 7.5, -4.9, 4.9);
const SCREEN_PX_M: f64 = 0.1;

/// The lifelong map on a display board hung above the far wall, drawn top
/// down in world orientation: occupied black, free light, unknown grey, the
/// day's appeared cells green and vanished cells red, the robot a blue dot.
/// Map cells are looked up through the map's own frame, so a registration
/// error would show as the map sitting off the building's outline.
fn push_map_screen(
    scene: &mut RenderScene,
    map: Option<&OccupancyGrid>,
    changes: Option<&DayVisual>,
    world_from_map: Pose2d,
    robot: Pose2d,
) {
    let (x0, x1, z0, z1) = SCREEN_WORLD;
    let width = ((x1 - x0) / SCREEN_PX_M).round() as u32;
    let height = ((z1 - z0) / SCREEN_PX_M).round() as u32;
    let changed = |coords: &[GridCoord]| -> std::collections::HashSet<(isize, isize)> {
        coords.iter().map(|coord| (coord.x, coord.y)).collect()
    };
    let appeared = changes
        .map(|visual| changed(&visual.appeared))
        .unwrap_or_default();
    let vanished = changes
        .map(|visual| changed(&visual.vanished))
        .unwrap_or_default();
    let map_from_world = world_from_map.inverse();
    let mut rgba = Vec::with_capacity((width * height * 4) as usize);
    // The board hangs on the south wall facing the camera, so it shows the
    // floor the way the camera sees it: south (-z) at the top. With the quad's
    // texture coordinates, that puts north in the image's first row; checked
    // against the day-1 change squares, which land on their slots.
    for row in 0..height {
        let z = z1 - (f64::from(row) + 0.5) * SCREEN_PX_M;
        for column in 0..width {
            let x = x0 + (f64::from(column) + 0.5) * SCREEN_PX_M;
            // Each pixel covers four map cells; it shows the most
            // significant of them, so one-cell walls survive the downscale.
            let color = map
                .map(|grid| {
                    let mut rank = 0;
                    for (dx, dz) in [
                        (-0.025, -0.025),
                        (0.025, -0.025),
                        (-0.025, 0.025),
                        (0.025, 0.025),
                    ] {
                        let local = map_from_world.transform_point(Vec3::new(x + dx, z + dz, 0.0));
                        let Some(coord) = grid.world_to_grid(local) else {
                            continue;
                        };
                        let key = (coord.x, coord.y);
                        let cell = if appeared.contains(&key) {
                            5
                        } else if vanished.contains(&key) {
                            4
                        } else if grid.is_occupied(coord) {
                            3
                        } else if grid.is_known(coord) {
                            2
                        } else {
                            1
                        };
                        rank = rank.max(cell);
                    }
                    match rank {
                        5 => [40, 230, 90],
                        4 => [240, 50, 40],
                        3 => [18, 20, 24],
                        2 => [225, 228, 232],
                        _ => [120, 124, 130],
                    }
                })
                .unwrap_or([120, 124, 130]);
            let near_robot = (x - robot.x_m).hypot(z - robot.y_m) < 0.3;
            let color = if near_robot { [30, 140, 255] } else { color };
            rgba.extend_from_slice(&[color[0], color[1], color[2], 255]);
        }
    }
    let image = Arc::new(ImageFrame::from_rgba8(width, height, rgba));
    // The board: 1 m of screen per 2.2 m of warehouse, above the south wall.
    let scale = 1.0 / 2.6;
    let (half_w, half_h) = (0.5 * (x1 - x0) * scale, 0.5 * (z1 - z0) * scale);
    let centre = Vec3::new(-1.0, 2.4 + half_h + 0.1, -4.45);
    let mesh = TriangleMesh {
        positions: vec![
            [-half_w as f32, -half_h as f32, 0.0],
            [half_w as f32, -half_h as f32, 0.0],
            [half_w as f32, half_h as f32, 0.0],
            [-half_w as f32, half_h as f32, 0.0],
        ],
        normals: vec![[0.0, 0.0, 1.0]; 4],
        texcoords: vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    push_box(
        scene,
        centre - Vec3::new(0.0, 0.0, 0.03),
        Vec3::new(half_w + 0.08, half_h + 0.08, 0.025),
        [0.08, 0.08, 0.09, 1.0],
        0.5,
    );
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: centre,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: [1.0; 4],
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(image),
        material: PbrMaterial::new([1.0; 4], 0.9, 0.0, [0.12, 0.12, 0.12]),
    });
}
