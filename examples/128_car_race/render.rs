//! Draws the race: a circuit built around the sampled track, four open-wheel
//! cars built from their parts at their simulated poses, and a camera that
//! follows the closest battle.
//!
//! The cut is the start and a window around every completed pass; each frame
//! is the recorded state at that moment, nothing interpolated.

use crate::track::{RacingLine, Track, WIDTH_M};
use crate::{CarPose, Entry, Moment};
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, ImageFrame, MeshRenderCache, PbrMaterial,
    RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const WIDTH: u32 = 720;
const HEIGHT: u32 = 405;
/// Seconds of race shown around each pass, before and after it completes.
const PASS_WINDOW_S: (f64, f64) = (3.2, 0.8);
/// Seconds of the start shown.
const START_S: f64 = 5.0;
/// Props are drawn only within this distance of the camera's focus.
const PROP_RANGE_M: f64 = 110.0;

pub(crate) fn render_race(
    track: &Track,
    _line: &RacingLine,
    moments: &[Moment],
    passes: &[(f64, usize, usize)],
    entries: &[Entry],
    frames_dir: &Path,
    gif_path: &Path,
) -> std::io::Result<()> {
    let _ = fs::remove_dir_all(frames_dir);
    fs::create_dir_all(frames_dir)?;
    let mut backend = WgpuRenderBackend::new().map_err(std::io::Error::other)?;
    backend.set_environment(sky());
    let camera = Camera::new(WIDTH, HEIGHT, 0.9);
    let mut cache = MeshRenderCache::new();
    let props =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_racing");
    let circuit = Circuit::build(track);

    let selected = cut(moments, passes, track);
    // `--still N` renders frame N alone, for looking at the models.
    let still: Option<usize> = std::env::args()
        .skip_while(|argument| argument != "--still")
        .nth(1)
        .and_then(|value| value.parse().ok());
    for (index, (moment, shot)) in selected.iter().enumerate() {
        if still.is_some_and(|wanted| wanted != index) {
            continue;
        }
        let mut scene = RenderScene::default();
        circuit.push(&mut scene, shot.focus);
        for (car, entry) in moment.cars.iter().zip(entries) {
            push_car(&mut scene, car, entry, moment.time_s);
        }
        cache
            .resolve_scene(&mut scene, &[props.as_path()])
            .map_err(std::io::Error::other)?;
        let orbit = shot.orbit();
        let output = backend
            .render_scene_camera(
                &camera,
                &orbit.camera_transform(),
                &scene,
                [0.55, 0.72, 0.92, 1.0],
            )
            .map_err(std::io::Error::other)?;
        write_png(
            &frames_dir.join(format!("frame-{index:03}.png")),
            &output.color.rgba8,
        )?;
    }
    if still.is_some() {
        return Ok(());
    }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/encode_gif.py");
    let status = std::process::Command::new("python3")
        .arg(script)
        .arg(frames_dir)
        .arg(gif_path)
        .args(["--fps", "12", "--colors", "112"])
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("encode_gif.py failed"));
    }
    Ok(())
}

/// A trackside camera post: the track point it looks at and the direction
/// the cars come from.
#[derive(Clone, Copy)]
struct Shot {
    focus: (f64, f64),
    /// Direction the cars travel through the shot, as an angle in (x, z).
    heading: f64,
}

impl Shot {
    fn at(track: &Track, index: usize) -> Self {
        let n = track.len();
        let (x0, z0) = track.center[index % n];
        let (x1, z1) = track.center[(index + 3) % n];
        Self {
            focus: (x0, z0),
            heading: (z1 - z0).atan2(x1 - x0),
        }
    }

    /// The post stands down the track from its focus and off to one side,
    /// raised, so the cars run toward it and past.
    fn orbit(&self) -> CameraOrbit {
        // This orbit puts the camera along (sin yaw, cos yaw) in (x, z) from
        // its focus; ahead of cars heading (cos h, sin h) is atan2(cos h, sin h).
        CameraOrbit {
            focus: Vec3::new(self.focus.0, 1.0, self.focus.1),
            yaw_rad: self.heading.cos().atan2(self.heading.sin()) + 0.75,
            pitch_rad: 1.2,
            distance_m: 18.0,
        }
    }
}

/// Spacing of the camera posts around the lap, in track samples (meters).
const POST_SPACING: usize = 40;

/// The start, then a window around each completed pass, in race order. Each
/// frame is filmed from the camera post the battle is approaching; the
/// director cuts to the next post once the cars run past. A camera that moved
/// with the cars repainted every pixel of grass and asphalt each frame and the
/// GIF ran to 70 MB; posts keep the scenery still between cuts.
fn cut<'a>(
    moments: &'a [Moment],
    passes: &[(f64, usize, usize)],
    track: &Track,
) -> Vec<(&'a Moment, Shot)> {
    let n = track.len();
    let mut frames = Vec::new();
    let mut film = |window: (f64, f64), cars: &dyn Fn(&Moment) -> Vec<usize>| {
        let mut post: Option<usize> = None;
        for moment in moments
            .iter()
            .filter(|moment| (window.0..=window.1).contains(&moment.time_s))
        {
            let followed = cars(moment);
            let middle = followed
                .iter()
                .map(|car| moment.cars[*car].transform.translation)
                .fold(Vec3::ZERO, |sum, p| sum + p)
                / followed.len() as f64;
            let (index, _) = track.project(middle.x, middle.z, 0, n / 2);
            // Cut when the cars have run 4 m past the current post's focus.
            let passed = post.is_none_or(|current| {
                let ahead = (index + n - current) % n;
                ahead > 4 && ahead < n / 2
            });
            if passed {
                let next = (index / POST_SPACING + 1) * POST_SPACING;
                post = Some(next % n);
            }
            frames.push((moment, Shot::at(track, post.expect("post"))));
        }
    };
    film((0.0, START_S), &|moment| {
        // The leading pair at the start.
        let mut order: Vec<usize> = (0..moment.cars.len()).collect();
        order.sort_by(|a, b| {
            let pa = moment.cars[*a].transform.translation;
            let pb = moment.cars[*b].transform.translation;
            pb.x.total_cmp(&pa.x)
        });
        order.into_iter().take(2).collect()
    });
    for (time_s, passer, passed) in passes {
        let window = (
            (time_s - PASS_WINDOW_S.0).max(START_S + 0.05),
            time_s + PASS_WINDOW_S.1,
        );
        film(window, &|_| vec![*passer, *passed]);
    }
    frames
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

/// A clear afternoon sky: bright near the horizon, deep blue overhead, a warm
/// sun low in one direction, and green ground bounce.
fn sky() -> EnvironmentLighting {
    const W: u32 = 128;
    const H: u32 = 64;
    let mut rgba32f = Vec::with_capacity((W * H * 4) as usize);
    for row in 0..H {
        let down = (row as f32 + 0.5) / H as f32;
        for column in 0..W {
            let around = (column as f32 + 0.5) / W as f32 * std::f32::consts::TAU;
            let (r, g, b) = if down < 0.5 {
                let up = 1.0 - down / 0.5;
                let sun = ((around - 1.0).cos().max(0.0)
                    * (1.0 - (down - 0.35).abs() * 4.0).max(0.0))
                .powf(12.0)
                    * 8.0;
                (
                    0.95 - 0.55 * up + sun,
                    1.05 - 0.40 * up + sun * 0.95,
                    1.20 - 0.10 * up + sun * 0.8,
                )
            } else {
                (0.22, 0.30, 0.16)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(W, H, rgba32f).expect("sky"),
        )),
        intensity: 1.0,
        diffuse_strength: 0.75,
        specular_strength: 0.45,
        rotation_rad: 0.0,
    }
}

// ---------------------------------------------------------------- primitives

fn material_item(
    transform: Transform3,
    shape: VisualShape,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) -> RenderSceneItem {
    let mut item = RenderScene::item_from_visual(transform, shape, color, Transform3::IDENTITY);
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    item
}

/// Surface finishes used across the cars and the circuit.
#[derive(Clone, Copy)]
struct Finish {
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
}

impl Finish {
    const fn paint(color: [f32; 4]) -> Self {
        Self {
            color,
            roughness: 0.28,
            metallic: 0.15,
            emissive: [0.0; 3],
        }
    }
    const fn matte(color: [f32; 4]) -> Self {
        Self {
            color,
            roughness: 0.85,
            metallic: 0.0,
            emissive: [0.0; 3],
        }
    }
}

const CARBON: Finish = Finish {
    color: [0.07, 0.07, 0.08, 1.0],
    roughness: 0.35,
    metallic: 0.2,
    emissive: [0.0; 3],
};
const RUBBER: Finish = Finish::matte([0.05, 0.05, 0.055, 1.0]);
const ALLOY: Finish = Finish {
    color: [0.62, 0.64, 0.68, 1.0],
    roughness: 0.25,
    metallic: 0.85,
    emissive: [0.0; 3],
};
const TITANIUM: Finish = Finish {
    color: [0.20, 0.21, 0.23, 1.0],
    roughness: 0.3,
    metallic: 0.7,
    emissive: [0.0; 3],
};

fn push_box(scene: &mut RenderScene, center: Vec3, rotation: Quat, half: Vec3, finish: Finish) {
    scene.items.push(material_item(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    ));
}

fn push_cylinder(
    scene: &mut RenderScene,
    center: Vec3,
    axis: Vec3,
    radius_m: f64,
    length_m: f64,
    finish: Finish,
) {
    scene.items.push(material_item(
        Transform3::from_translation_rotation(
            center,
            Quat::from_rotation_arc(Vec3::Z, axis.normalize()),
        ),
        VisualShape::Cylinder { radius_m, length_m },
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    ));
}

/// A cylinder with its own orientation, for spinning parts.
fn push_cylinder_rotated(
    scene: &mut RenderScene,
    center: Vec3,
    rotation: Quat,
    radius_m: f64,
    length_m: f64,
    finish: Finish,
) {
    scene.items.push(material_item(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Cylinder { radius_m, length_m },
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    ));
}

fn push_sphere(scene: &mut RenderScene, center: Vec3, radius_m: f64, finish: Finish) {
    scene.items.push(material_item(
        Transform3::from_translation_rotation(center, Quat::IDENTITY),
        VisualShape::Sphere { radius_m },
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    ));
}

fn push_prop(scene: &mut RenderScene, path: &str, base: Vec3, rotation: Quat) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(base, rotation),
        VisualShape::Mesh {
            path: path.to_string(),
            scale: Vec3::ONE,
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

fn mesh_item(
    mesh: TriangleMesh,
    finish: Finish,
    texture: Option<&Arc<ImageFrame>>,
    maps: Option<(&Arc<ImageFrame>, &Arc<ImageFrame>)>,
) -> RenderSceneItem {
    let mut material = PbrMaterial::new(
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    );
    if let Some((normal, roughness)) = maps {
        material =
            material.with_texture_maps(Some(Arc::clone(normal)), Some(Arc::clone(roughness)));
    }
    RenderSceneItem {
        transform: MathTransform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: finish.color,
        mesh: Some(Arc::new(mesh)),
        base_color_texture: texture.map(Arc::clone),
        material,
    }
}

// ------------------------------------------------------------------- circuit

/// Quads collected into one mesh, wound to face up.
#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    /// A quad with corners in order around its edge, facing +y.
    fn quad(&mut self, corners: [Vec3; 4], uv: [[f32; 2]; 4]) {
        let base = self.positions.len() as u32;
        for corner in corners {
            self.positions
                .push([corner.x as f32, corner.y as f32, corner.z as f32]);
        }
        self.texcoords.extend_from_slice(&uv);
        // Counter-clockwise seen from above means (b - a) x (c - a) points +y.
        let up = (corners[1] - corners[0]).cross(corners[2] - corners[0]).y > 0.0;
        if up {
            self.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        } else {
            self.indices
                .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
        }
    }

    fn build(self) -> Option<TriangleMesh> {
        (!self.indices.is_empty()).then(|| TriangleMesh {
            normals: vec![[0.0, 1.0, 0.0]; self.positions.len()],
            positions: self.positions,
            texcoords: self.texcoords,
            indices: self.indices,
            skinning: None,
        })
    }
}

struct Textures {
    color: Arc<ImageFrame>,
    normal: Arc<ImageFrame>,
    roughness: Arc<ImageFrame>,
}

fn load_textures(name: &str) -> Textures {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/props/polyhaven_racing/textures");
    let load = |suffix: &str| {
        let rgba = image::open(root.join(format!("{name}_{suffix}_1k.jpg")))
            .expect("load racing texture")
            .into_rgba8();
        Arc::new(ImageFrame::from_rgba8(
            rgba.width(),
            rgba.height(),
            rgba.into_raw(),
        ))
    };
    Textures {
        color: load("diff"),
        normal: load("nor_gl"),
        roughness: load("rough"),
    }
}

/// Everything that does not move, built once.
struct Circuit {
    items: Vec<RenderSceneItem>,
    /// Props placed along the circuit, drawn when near the camera.
    props: Vec<(&'static str, Vec3, Quat)>,
}

impl Circuit {
    fn build(track: &Track) -> Self {
        let asphalt = load_textures("asphalt_track");
        let grass = load_textures("leafy_grass");
        let mut items = Vec::new();
        items.push(ground(&grass));
        items.push(surface(track, &asphalt));
        items.extend(markings(track));
        items.extend(start_line(track));
        items.extend(grandstand());
        let props = barriers(track);
        Self { items, props }
    }

    fn push(&self, scene: &mut RenderScene, focus: (f64, f64)) {
        scene.items.extend(self.items.iter().cloned());
        for (path, base, rotation) in &self.props {
            if (base.x - focus.0).hypot(base.z - focus.1) < PROP_RANGE_M {
                push_prop(scene, path, *base, *rotation);
            }
        }
        push_gantry(scene);
    }
}

fn ground(grass: &Textures) -> RenderSceneItem {
    let (x0, x1, z0, z1) = (-320.0, 320.0, -160.0, 260.0);
    let mut mesh = QuadMesh::default();
    let repeat = 6.0;
    mesh.quad(
        [
            Vec3::new(x0, 0.0, z0),
            Vec3::new(x1, 0.0, z0),
            Vec3::new(x1, 0.0, z1),
            Vec3::new(x0, 0.0, z1),
        ],
        [
            [0.0, 0.0],
            [((x1 - x0) / repeat) as f32, 0.0],
            [((x1 - x0) / repeat) as f32, ((z1 - z0) / repeat) as f32],
            [0.0, ((z1 - z0) / repeat) as f32],
        ],
    );
    mesh_item(
        mesh.build().expect("ground"),
        Finish::matte([0.85, 0.95, 0.8, 1.0]),
        Some(&grass.color),
        Some((&grass.normal, &grass.roughness)),
    )
}

/// The asphalt ribbon, one texture repeat every 8 m along and across.
fn surface(track: &Track, asphalt: &Textures) -> RenderSceneItem {
    let n = track.len();
    let half = 0.5 * WIDTH_M + 0.4;
    let mut mesh = QuadMesh::default();
    for i in 0..n {
        let j = (i + 1) % n;
        let corner = |index: usize, side: f64| {
            let (x, z) = track.at(index, side);
            Vec3::new(x, 0.02, z)
        };
        let (v0, v1) = ((i as f64 / 8.0) as f32, ((i + 1) as f64 / 8.0) as f32);
        let across = (2.0 * half / 8.0) as f32;
        mesh.quad(
            [
                corner(i, -half),
                corner(j, -half),
                corner(j, half),
                corner(i, half),
            ],
            [[0.0, v0], [0.0, v1], [across, v1], [across, v0]],
        );
    }
    mesh_item(
        mesh.build().expect("surface"),
        Finish::matte([0.9, 0.9, 0.9, 1.0]),
        Some(&asphalt.color),
        Some((&asphalt.normal, &asphalt.roughness)),
    )
}

/// White edge lines, and red-and-white kerbs on both edges wherever the
/// track turns tighter than a 90 m radius.
fn markings(track: &Track) -> Vec<RenderSceneItem> {
    let n = track.len();
    let edge = 0.5 * WIDTH_M;
    let mut white = QuadMesh::default();
    let mut red = QuadMesh::default();
    let strip = |mesh: &mut QuadMesh, i: usize, from: f64, to: f64, y: f64| {
        let j = (i + 1) % n;
        let p = |index: usize, side: f64| {
            let (x, z) = track.at(index, side);
            Vec3::new(x, y, z)
        };
        mesh.quad([p(i, from), p(j, from), p(j, to), p(i, to)], [[0.0; 2]; 4]);
    };
    for i in 0..n {
        for sign in [-1.0, 1.0] {
            strip(
                &mut white,
                i,
                sign * (edge - 0.25),
                sign * (edge - 0.1),
                0.028,
            );
        }
        let turning = (-12..=12).any(|k| {
            let index = (i as isize + k).rem_euclid(n as isize) as usize;
            track.curvature[index].abs() > 1.0 / 90.0
        });
        if turning {
            let mesh = if (i / 3) % 2 == 0 {
                &mut red
            } else {
                &mut white
            };
            for sign in [-1.0, 1.0] {
                strip(mesh, i, sign * (edge - 0.1), sign * (edge + 1.3), 0.035);
            }
        }
    }
    let mut items = Vec::new();
    if let Some(mesh) = white.build() {
        items.push(mesh_item(
            mesh,
            Finish::matte([0.93, 0.93, 0.92, 1.0]),
            None,
            None,
        ));
    }
    if let Some(mesh) = red.build() {
        items.push(mesh_item(
            mesh,
            Finish::matte([0.78, 0.08, 0.07, 1.0]),
            None,
            None,
        ));
    }
    items
}

/// The chequered start/finish line across the track at sample zero.
fn start_line(track: &Track) -> Vec<RenderSceneItem> {
    let n = track.len();
    let mut black = QuadMesh::default();
    let mut white = QuadMesh::default();
    let squares = 16;
    let size = WIDTH_M / squares as f64;
    for row in 0..2 {
        for column in 0..squares {
            let from = -0.5 * WIDTH_M + column as f64 * size;
            let to = from + size;
            let i = row % n;
            let j = (row + 1) % n;
            let p = |index: usize, side: f64| {
                let (x, z) = track.at(index, side);
                Vec3::new(x, 0.036, z)
            };
            let mesh = if (row + column) % 2 == 0 {
                &mut black
            } else {
                &mut white
            };
            mesh.quad([p(i, from), p(j, from), p(j, to), p(i, to)], [[0.0; 2]; 4]);
        }
    }
    vec![
        mesh_item(
            black.build().expect("black"),
            Finish::matte([0.05, 0.05, 0.05, 1.0]),
            None,
            None,
        ),
        mesh_item(
            white.build().expect("white"),
            Finish::matte([0.95, 0.95, 0.95, 1.0]),
            None,
            None,
        ),
    ]
}

/// The start/finish gantry over the line: two posts, a beam, and a row of
/// start lights.
fn push_gantry(scene: &mut RenderScene) {
    let steel = Finish::paint([0.18, 0.2, 0.24, 1.0]);
    for z in [-7.5, 7.5] {
        push_box(
            scene,
            Vec3::new(0.0, 3.0, z),
            Quat::IDENTITY,
            Vec3::new(0.2, 3.0, 0.2),
            steel,
        );
    }
    push_box(
        scene,
        Vec3::new(0.0, 6.2, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.35, 0.35, 7.7),
        steel,
    );
    push_box(
        scene,
        Vec3::new(-0.36, 6.2, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.02, 0.28, 5.5),
        Finish::paint([0.92, 0.92, 0.92, 1.0]),
    );
    for light in 0..5 {
        push_sphere(
            scene,
            Vec3::new(-0.4, 5.75, -2.0 + light as f64),
            0.14,
            Finish {
                color: [0.25, 0.02, 0.02, 1.0],
                roughness: 0.3,
                metallic: 0.0,
                emissive: [0.0; 3],
            },
        );
    }
}

/// A grandstand along the outside of the main straight: stepped tiers with a
/// crowd, a roof on pillars.
fn grandstand() -> Vec<RenderSceneItem> {
    let (x0, x1) = (-70.0, 70.0);
    let mut items = Vec::new();
    let concrete = Finish::matte([0.66, 0.66, 0.64, 1.0]);
    for tier in 0..8 {
        let z = -22.0 - 1.2 * tier as f64;
        let height = 1.0 + 0.8 * tier as f64;
        items.push(material_item(
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.5 * height, z), Quat::IDENTITY),
            VisualShape::Box {
                size_m: Vec3::new(x1 - x0, height, 1.2),
            },
            concrete.color,
            concrete.roughness,
            0.0,
            [0.0; 3],
        ));
    }
    // The crowd: one small box per seat, in six shirt colours, as six meshes.
    let shirts = [
        [0.85, 0.15, 0.12, 1.0],
        [0.12, 0.30, 0.75, 1.0],
        [0.95, 0.85, 0.20, 1.0],
        [0.92, 0.92, 0.90, 1.0],
        [0.15, 0.55, 0.30, 1.0],
        [0.20, 0.20, 0.22, 1.0],
    ];
    let mut crowd: Vec<QuadMesh> = (0..shirts.len()).map(|_| QuadMesh::default()).collect();
    let mut seed = 0x2545_f491_u32;
    for tier in 0..8 {
        let z = -21.7 - 1.2 * tier as f64;
        let y = 1.0 + 0.8 * tier as f64 + 0.35;
        let mut x = x0 + 0.5;
        while x < x1 - 0.5 {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            if seed % 10 < 8 {
                let mesh = &mut crowd[(seed / 10) as usize % shirts.len()];
                // Front face and top of a seated spectator.
                let (hx, hz) = (0.22, 0.2);
                mesh.quad(
                    [
                        Vec3::new(x - hx, y + 0.3, z - hz),
                        Vec3::new(x + hx, y + 0.3, z - hz),
                        Vec3::new(x + hx, y + 0.3, z + hz),
                        Vec3::new(x - hx, y + 0.3, z + hz),
                    ],
                    [[0.0; 2]; 4],
                );
                mesh.quad(
                    [
                        Vec3::new(x - hx, y - 0.3, z + hz),
                        Vec3::new(x + hx, y - 0.3, z + hz),
                        Vec3::new(x + hx, y + 0.3, z + hz),
                        Vec3::new(x - hx, y + 0.3, z + hz),
                    ],
                    [[0.0; 2]; 4],
                );
            }
            x += 0.6;
        }
    }
    for (mesh, shirt) in crowd.into_iter().zip(shirts) {
        if let Some(mut built) = mesh.build() {
            // The vertical faces face +z, toward the track.
            let count = built.positions.len();
            for (index, normal) in built.normals.iter_mut().enumerate() {
                if index % 8 >= 4 && index < count {
                    *normal = [0.0, 0.0, 1.0];
                }
            }
            items.push(mesh_item(built, Finish::matte(shirt), None, None));
        }
    }
    // Roof on pillars.
    items.push(material_item(
        Transform3::from_translation_rotation(
            Vec3::new(0.0, 9.6, -26.5),
            Quat::from_rotation_x(-0.08),
        ),
        VisualShape::Box {
            size_m: Vec3::new(x1 - x0 + 4.0, 0.3, 12.0),
        },
        [0.85, 0.87, 0.9, 1.0],
        0.4,
        0.5,
        [0.0; 3],
    ));
    let mut x = x0;
    while x <= x1 {
        items.push(material_item(
            Transform3::from_translation_rotation(Vec3::new(x, 4.8, -31.8), Quat::IDENTITY),
            VisualShape::Box {
                size_m: Vec3::new(0.4, 9.6, 0.4),
            },
            [0.3, 0.32, 0.36, 1.0],
            0.5,
            0.5,
            [0.0; 3],
        ));
        x += 20.0;
    }
    // Pit wall along the outside of the straight.
    items.push(material_item(
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.55, -8.2), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(290.0, 1.1, 0.5),
        },
        [0.9, 0.9, 0.9, 1.0],
        0.8,
        0.0,
        [0.0; 3],
    ));
    items
}

/// Scanned tyre walls on the outside of the tight corners and concrete
/// barriers along the inside of the main straight.
fn barriers(track: &Track) -> Vec<(&'static str, Vec3, Quat)> {
    let n = track.len();
    let mut props = Vec::new();
    let tyre_lying = Quat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let mut last = None;
    for i in 0..n {
        let kappa = track.curvature[i];
        if kappa.abs() < 1.0 / 45.0 {
            continue;
        }
        // Outside of the corner, beyond the kerb and a strip of grass.
        let side = -kappa.signum() * (0.5 * WIDTH_M + 7.0);
        let (x, z) = track.at(i, side);
        if last.is_some_and(|(lx, lz): (f64, f64)| (x - lx).hypot(z - lz) < 0.62) {
            continue;
        }
        last = Some((x, z));
        for layer in 0..2 {
            props.push((
                "old_tyre/old_tyre_1k.gltf",
                Vec3::new(x, 0.083 + 0.166 * f64::from(layer), z),
                tyre_lying,
            ));
        }
    }
    // Concrete barriers on the infield side of the main straight.
    let mut x = -140.0;
    while x < 140.0 {
        props.push((
            "concrete_road_barrier/concrete_road_barrier_1k.gltf",
            Vec3::new(x, 0.0, 8.6),
            Quat::IDENTITY,
        ));
        x += 1.55;
    }
    props
}

// ----------------------------------------------------------------------- car

/// An open-wheel racing car drawn around its simulated pose: the car's own
/// frame has +x forward, +y up and +z to its right.
fn push_car(scene: &mut RenderScene, car: &CarPose, entry: &Entry, time_s: f64) {
    let rotation = car.transform.rotation;
    let origin = car.transform.translation;
    let at = |x: f64, y: f64, z: f64| origin + rotation * Vec3::new(x, y, z);
    let paint = Finish::paint(entry.livery);
    let accent = Finish::paint(entry.accent);
    let part = |scene: &mut RenderScene, x: f64, y: f64, z: f64, half: Vec3, finish: Finish| {
        push_box(scene, at(x, y, z), rotation, half, finish);
    };
    let tilted = |scene: &mut RenderScene,
                  x: f64,
                  y: f64,
                  z: f64,
                  pitch: f64,
                  half: Vec3,
                  finish: Finish| {
        push_box(
            scene,
            at(x, y, z),
            rotation * Quat::from_rotation_z(pitch),
            half,
            finish,
        );
    };

    // Floor and plank.
    part(scene, 0.0, 0.07, 0.0, Vec3::new(2.05, 0.02, 0.72), CARBON);
    part(
        scene,
        0.0,
        0.05,
        0.0,
        Vec3::new(1.9, 0.015, 0.14),
        Finish::matte([0.55, 0.45, 0.30, 1.0]),
    );
    // Survival cell, stepping down into the nose.
    part(scene, 0.25, 0.36, 0.0, Vec3::new(0.95, 0.22, 0.34), paint);
    tilted(
        scene,
        1.45,
        0.33,
        0.0,
        -0.10,
        Vec3::new(0.35, 0.16, 0.26),
        paint,
    );
    tilted(
        scene,
        2.0,
        0.26,
        0.0,
        -0.14,
        Vec3::new(0.3, 0.11, 0.17),
        paint,
    );
    tilted(
        scene,
        2.42,
        0.2,
        0.0,
        -0.18,
        Vec3::new(0.18, 0.07, 0.1),
        accent,
    );
    // Cockpit, driver and halo.
    part(scene, 0.05, 0.585, 0.0, Vec3::new(0.42, 0.02, 0.24), CARBON);
    push_sphere(
        scene,
        at(-0.05, 0.72, 0.0),
        0.15,
        Finish::paint(entry.accent),
    );
    push_box(
        scene,
        at(0.02, 0.74, 0.0),
        rotation,
        Vec3::new(0.08, 0.035, 0.13),
        Finish {
            color: [0.05, 0.06, 0.08, 1.0],
            roughness: 0.1,
            metallic: 0.6,
            emissive: [0.0; 3],
        },
    );
    push_cylinder(
        scene,
        at(0.42, 0.72, 0.0),
        rotation * Vec3::new(0.45, 1.0, 0.0),
        0.025,
        0.36,
        TITANIUM,
    );
    for side in [-1.0, 1.0] {
        push_cylinder(
            scene,
            at(0.12, 0.86, side * 0.2),
            rotation * Vec3::new(1.0, 0.0, side * 0.25),
            0.025,
            0.62,
            TITANIUM,
        );
    }
    // Sidepods with their radiator inlets.
    for side in [-1.0, 1.0] {
        part(
            scene,
            -0.15,
            0.3,
            side * 0.56,
            Vec3::new(0.78, 0.2, 0.22),
            paint,
        );
        part(
            scene,
            0.64,
            0.34,
            side * 0.56,
            Vec3::new(0.02, 0.13, 0.19),
            CARBON,
        );
        part(
            scene,
            -0.15,
            0.51,
            side * 0.56,
            Vec3::new(0.6, 0.012, 0.2),
            accent,
        );
    }
    // Engine cover rising into the airbox, and the shark fin.
    tilted(
        scene,
        -0.95,
        0.47,
        0.0,
        0.06,
        Vec3::new(0.8, 0.2, 0.2),
        paint,
    );
    part(scene, -0.3, 0.86, 0.0, Vec3::new(0.24, 0.16, 0.12), paint);
    part(scene, -0.12, 0.88, 0.0, Vec3::new(0.02, 0.1, 0.09), CARBON);
    tilted(
        scene,
        -1.25,
        0.8,
        0.0,
        0.18,
        Vec3::new(0.55, 0.14, 0.012),
        accent,
    );
    push_aero(scene, car, entry, time_s);
    push_wheels(scene, car, &at, rotation);
}

/// Wings, rain light, diffuser and mirrors.
fn push_aero(scene: &mut RenderScene, car: &CarPose, entry: &Entry, time_s: f64) {
    let rotation = car.transform.rotation;
    let origin = car.transform.translation;
    let at = |x: f64, y: f64, z: f64| origin + rotation * Vec3::new(x, y, z);
    let paint = Finish::paint(entry.livery);
    let accent = Finish::paint(entry.accent);
    let part = |scene: &mut RenderScene, x: f64, y: f64, z: f64, half: Vec3, finish: Finish| {
        push_box(scene, at(x, y, z), rotation, half, finish);
    };
    let tilted = |scene: &mut RenderScene,
                  x: f64,
                  y: f64,
                  z: f64,
                  pitch: f64,
                  half: Vec3,
                  finish: Finish| {
        push_box(
            scene,
            at(x, y, z),
            rotation * Quat::from_rotation_z(pitch),
            half,
            finish,
        );
    };

    // Front wing: two elements between endplates.
    part(scene, 2.55, 0.11, 0.0, Vec3::new(0.3, 0.018, 0.92), paint);
    tilted(
        scene,
        2.35,
        0.19,
        0.0,
        0.35,
        Vec3::new(0.14, 0.012, 0.9),
        accent,
    );
    for side in [-1.0, 1.0] {
        part(
            scene,
            2.45,
            0.17,
            side * 0.95,
            Vec3::new(0.38, 0.1, 0.015),
            CARBON,
        );
    }
    // Rear wing: main plane and flap between endplates, on a pylon, and the
    // rain light that burns when the car brakes.
    part(scene, -2.35, 0.9, 0.0, Vec3::new(0.22, 0.02, 0.52), paint);
    tilted(
        scene,
        -2.52,
        1.0,
        0.0,
        0.45,
        Vec3::new(0.12, 0.015, 0.5),
        accent,
    );
    for side in [-1.0, 1.0] {
        part(
            scene,
            -2.4,
            0.78,
            side * 0.53,
            Vec3::new(0.32, 0.3, 0.015),
            paint,
        );
    }
    part(scene, -2.2, 0.55, 0.0, Vec3::new(0.05, 0.3, 0.03), CARBON);
    let rain = if car.braking || (time_s * 6.0) as i64 % 2 == 0 && car.speed_m_s < 1.0 {
        [1.0, 0.1, 0.08]
    } else {
        [0.12, 0.0, 0.0]
    };
    part(
        scene,
        -2.5,
        0.33,
        0.0,
        Vec3::new(0.02, 0.04, 0.08),
        Finish {
            color: [0.9, 0.05, 0.05, 1.0],
            roughness: 0.3,
            metallic: 0.0,
            emissive: rain,
        },
    );
    // Diffuser.
    tilted(
        scene,
        -2.2,
        0.16,
        0.0,
        -0.25,
        Vec3::new(0.3, 0.015, 0.6),
        CARBON,
    );
    // Mirrors.
    for side in [-1.0, 1.0] {
        part(
            scene,
            0.55,
            0.66,
            side * 0.46,
            Vec3::new(0.05, 0.035, 0.07),
            paint,
        );
        push_cylinder(
            scene,
            at(0.55, 0.62, side * 0.4),
            rotation * Vec3::new(0.0, 0.3, side),
            0.01,
            0.14,
            CARBON,
        );
    }
}

/// Four wheels on their wishbones: black tyres with a coloured compound band,
/// alloy rims with spokes turning at the car's rolling speed, fronts steered.
fn push_wheels(
    scene: &mut RenderScene,
    car: &CarPose,
    at: &dyn Fn(f64, f64, f64) -> Vec3,
    rotation: Quat,
) {
    const RADIUS: f64 = 0.36;
    let band = Finish::paint([0.95, 0.82, 0.10, 1.0]);
    for (x, width, steered) in [(1.6, 0.3, true), (-1.8, 0.4, false)] {
        for side in [-1.0, 1.0] {
            let track_half = 0.8 + 0.5 * width;
            let hub = at(x, RADIUS, side * track_half);
            let steer = if steered {
                Quat::from_rotation_y(-car.steering_rad)
            } else {
                Quat::IDENTITY
            };
            let spin = Quat::from_rotation_z(-car.wheel_angle_rad);
            // The wheel's axle is its local z.
            let wheel = rotation * steer * spin;
            push_cylinder_rotated(scene, hub, wheel, RADIUS, width, RUBBER);
            push_cylinder_rotated(scene, hub, wheel, RADIUS - 0.05, width + 0.004, band);
            push_cylinder_rotated(scene, hub, wheel, RADIUS - 0.08, width + 0.008, RUBBER);
            push_cylinder_rotated(scene, hub, wheel, RADIUS - 0.12, width + 0.012, ALLOY);
            for spoke in 0..3 {
                let angle = f64::from(spoke) * std::f64::consts::PI / 3.0;
                push_box(
                    scene,
                    hub + wheel * Vec3::new(0.0, 0.0, side * (0.5 * width + 0.008)),
                    wheel * Quat::from_rotation_z(angle),
                    Vec3::new(0.2, 0.018, 0.01),
                    TITANIUM,
                );
            }
            // Wishbones from the chassis to the upright.
            for (height, reach) in [(0.28, 0.0), (0.46, 0.0)] {
                let from = at(x + reach, height, side * 0.3);
                let to = hub + Vec3::new(0.0, height - RADIUS, 0.0);
                let middle = (from + to) * 0.5;
                push_cylinder(
                    scene,
                    middle,
                    to - from,
                    0.018,
                    (to - from).length(),
                    CARBON,
                );
            }
        }
    }
}
