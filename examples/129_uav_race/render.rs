//! Draws the heat at night: LED gates on stands, lit start pads, floodlight
//! masts around a dark field, and four racing quads built from their parts,
//! each trailing its last second of flight in its own colour.
//!
//! The cut: a close camera on the pads for the staggered start, a wide shot
//! of the course for the chase, and trackside cameras at the gates where the
//! final-lap passes happened. Every frame is a recorded moment.

use crate::course::{Course, GATES, GATE_OPENING_M};
use crate::{Moment, PILOTS, RECORD_HZ};
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_render::{
    Camera, EnvironmentLighting, EnvironmentMap, PbrMaterial, RenderBackend, RenderScene,
    RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const WIDTH: u32 = 720;
const HEIGHT: u32 = 405;
/// Seconds of flight trailed behind each drone.
const TRAIL_S: f64 = 0.8;
/// LED colour of each gate, in course order.
const GATE_COLOURS: [[f32; 3]; 4] = [
    [1.0, 0.25, 0.1],
    [0.1, 0.6, 1.0],
    [0.9, 0.2, 1.0],
    [0.2, 1.0, 0.4],
];

/// Where a shot's camera looks from.
#[derive(Clone, Copy)]
struct Shot {
    focus: Vec3,
    yaw_rad: f64,
    pitch_rad: f64,
    distance_m: f64,
}

impl Shot {
    fn orbit(&self) -> CameraOrbit {
        CameraOrbit {
            focus: self.focus,
            yaw_rad: self.yaw_rad,
            pitch_rad: self.pitch_rad,
            distance_m: self.distance_m,
        }
    }
}

pub(crate) fn render_race(
    course: &Course,
    moments: &[Moment],
    passes: &[(f64, usize, usize)],
    frames_dir: &Path,
    gif_path: &Path,
) -> std::io::Result<()> {
    let _ = fs::remove_dir_all(frames_dir);
    fs::create_dir_all(frames_dir)?;
    let mut backend = WgpuRenderBackend::new().map_err(std::io::Error::other)?;
    backend.set_environment(night());
    let camera = Camera::new(WIDTH, HEIGHT, 0.85);
    let still: Option<usize> = std::env::args()
        .skip_while(|argument| argument != "--still")
        .nth(1)
        .and_then(|value| value.parse().ok());
    let arena = arena(course);
    for (frame, (index, shot)) in cut(course, moments, passes).into_iter().enumerate() {
        if still.is_some_and(|wanted| wanted != frame) {
            continue;
        }
        let moment = &moments[index];
        let mut scene = RenderScene::default();
        scene.items.extend(arena.iter().cloned());
        let trail_from = index.saturating_sub((TRAIL_S * RECORD_HZ) as usize);
        for (drone, pilot) in PILOTS.iter().enumerate() {
            let trail: Vec<Vec3> = moments[trail_from..=index]
                .iter()
                .map(|past| past.drones[drone].translation)
                .collect();
            push_trail(&mut scene, &trail, pilot.led);
            push_quad(
                &mut scene,
                &moment.drones[drone],
                pilot.frame,
                pilot.led,
                moment.time_s,
            );
        }
        let output = backend
            .render_scene_camera(
                &camera,
                &shot.orbit().camera_transform(),
                &scene,
                [0.015, 0.02, 0.04, 1.0],
            )
            .map_err(std::io::Error::other)?;
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
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
        .args(["--fps", "12", "--colors", "128"])
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("encode_gif.py failed"));
    }
    Ok(())
}

/// A camera behind `focus`, looking along `direction`. The orbit puts the
/// camera along (sin yaw, cos yaw) in (x, z) from its focus, so behind a
/// direction (dx, dz) is atan2(-dx, -dz).
fn behind(focus: Vec3, direction: Vec3, distance_m: f64, pitch_rad: f64) -> Shot {
    Shot {
        focus,
        yaw_rad: (-direction.x).atan2(-direction.z) + 0.3,
        pitch_rad,
        distance_m,
    }
}

/// The edit: the launch from behind the pads, a chase camera on the leader
/// through the first lap, a wide shot of the course at triple speed, and a
/// chase camera on the closest pair through the final-lap passes.
fn cut(course: &Course, moments: &[Moment], passes: &[(f64, usize, usize)]) -> Vec<(usize, Shot)> {
    let first_pass = passes.first().map_or(f64::INFINITY, |pass| pass.0);
    let last_pass = passes.last().map_or(0.0, |pass| pass.0);
    let direction_at = |position: Vec3| {
        let index = course.project(position, 0, course.len() / 2);
        course.tangent[(index + 15) % course.len()]
    };
    let pads = {
        let start = moments[0]
            .drones
            .iter()
            .fold(Vec3::ZERO, |sum, d| sum + d.translation)
            * 0.25;
        behind(
            start + Vec3::new(0.0, 0.6, 0.0),
            direction_at(start),
            6.0,
            1.3,
        )
    };
    let wide = Shot {
        focus: Vec3::new(7.0, 3.0, 16.0),
        yaw_rad: -0.55,
        pitch_rad: 0.95,
        distance_m: 44.0,
    };
    let mut frames = Vec::new();
    let mut eased: Option<Vec3> = None;
    for (index, moment) in moments.iter().enumerate() {
        let t = moment.time_s;
        let shot = if t <= 5.5 {
            Some(pads)
        } else if t <= 10.0 {
            // The leader: the first drone away.
            let leader = moment.drones[0].translation;
            Some(chase(&mut eased, leader, direction_at(leader)))
        } else if t < first_pass - 3.0 {
            eased = None;
            (index % 3 == 0).then_some(wide)
        } else if t <= last_pass + 1.0 {
            let pair = closest_pair(moment);
            Some(chase(&mut eased, pair, direction_at(pair)))
        } else {
            None
        };
        if let Some(shot) = shot {
            frames.push((index, shot));
        }
    }
    frames
}

/// A chase camera 4 m behind `target`, eased so it follows smoothly.
fn chase(eased: &mut Option<Vec3>, target: Vec3, direction: Vec3) -> Shot {
    let focus = match eased {
        Some(previous) => *previous + (target - *previous) * 0.5,
        None => target,
    };
    *eased = Some(focus);
    behind(focus, direction, 4.0, 1.28)
}

/// The middle of the two drones closest to each other.
fn closest_pair(moment: &Moment) -> Vec3 {
    let mut best = (f64::INFINITY, Vec3::ZERO);
    for a in 0..moment.drones.len() {
        for b in a + 1..moment.drones.len() {
            let (pa, pb) = (moment.drones[a].translation, moment.drones[b].translation);
            let distance = (pa - pb).length();
            if distance < best.0 {
                best = (distance, (pa + pb) * 0.5);
            }
        }
    }
    best.1
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

/// A night sky: near black overhead, a faint glow along the horizon from the
/// city, and dark ground.
fn night() -> EnvironmentLighting {
    const W: u32 = 64;
    const H: u32 = 32;
    let mut rgba32f = Vec::with_capacity((W * H * 4) as usize);
    for row in 0..H {
        let down = (row as f32 + 0.5) / H as f32;
        for _ in 0..W {
            let glow = (1.0 - (down - 0.5).abs() * 6.0).max(0.0);
            let (r, g, b) = if down < 0.5 {
                (0.03 + 0.25 * glow, 0.04 + 0.18 * glow, 0.08 + 0.12 * glow)
            } else {
                (0.03, 0.035, 0.04)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    EnvironmentLighting {
        map: Some(Arc::new(
            EnvironmentMap::from_rgba32f(W, H, rgba32f).expect("night"),
        )),
        intensity: 1.0,
        diffuse_strength: 0.9,
        specular_strength: 0.6,
        rotation_rad: 0.0,
    }
}

// ---------------------------------------------------------------- primitives

#[derive(Clone, Copy)]
struct Finish {
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
}

const fn matte(color: [f32; 4]) -> Finish {
    Finish {
        color,
        roughness: 0.8,
        metallic: 0.0,
        emissive: [0.0; 3],
    }
}

const fn glow(light: [f32; 3]) -> Finish {
    Finish {
        color: [light[0], light[1], light[2], 1.0],
        roughness: 0.3,
        metallic: 0.0,
        emissive: light,
    }
}

const CARBON: Finish = Finish {
    color: [0.06, 0.06, 0.07, 1.0],
    roughness: 0.35,
    metallic: 0.25,
    emissive: [0.0; 3],
};

fn item(transform: Transform3, shape: VisualShape, finish: Finish) -> RenderSceneItem {
    let mut item =
        RenderScene::item_from_visual(transform, shape, finish.color, Transform3::IDENTITY);
    item.material = PbrMaterial::new(
        finish.color,
        finish.roughness,
        finish.metallic,
        finish.emissive,
    );
    item
}

fn cuboid(center: Vec3, rotation: Quat, half: Vec3, finish: Finish) -> RenderSceneItem {
    item(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        finish,
    )
}

fn cylinder(
    center: Vec3,
    rotation: Quat,
    radius_m: f64,
    length_m: f64,
    finish: Finish,
) -> RenderSceneItem {
    item(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Cylinder { radius_m, length_m },
        finish,
    )
}

// --------------------------------------------------------------------- arena

/// Everything that does not move: the field, the gates on their stands, the
/// start pads, the course markers and the floodlight masts.
fn arena(course: &Course) -> Vec<RenderSceneItem> {
    let mut items = Vec::new();
    items.push(field());
    items.extend(field_grid());
    for (gate, (x, y, z)) in GATES.iter().enumerate() {
        let direction = course.tangent[course.gate_index[gate]];
        push_gate(
            &mut items,
            Vec3::new(*x, *y, *z),
            direction,
            GATE_COLOURS[gate % GATE_COLOURS.len()],
        );
    }
    // Ground markers under the course line every 3 m, so its shape reads
    // from the wide shot.
    for index in (0..course.len()).step_by(30) {
        let point = course.points[index];
        items.push(cuboid(
            Vec3::new(point.x, 0.02, point.z),
            Quat::IDENTITY,
            Vec3::new(0.12, 0.01, 0.12),
            glow([0.25, 0.25, 0.3]),
        ));
    }
    // Floodlight masts around the field.
    for (x, z) in [
        (-30.0, -14.0),
        (44.0, -14.0),
        (44.0, 46.0),
        (-30.0, 46.0),
        (7.0, 50.0),
    ] {
        items.push(cuboid(
            Vec3::new(x, 7.0, z),
            Quat::IDENTITY,
            Vec3::new(0.2, 7.0, 0.2),
            matte([0.3, 0.3, 0.32, 1.0]),
        ));
        items.push(cuboid(
            Vec3::new(x, 14.2, z),
            Quat::IDENTITY,
            Vec3::new(1.2, 0.4, 0.3),
            glow([1.0, 0.95, 0.85]),
        ));
    }
    // Start pads: one per drone, lit at the edge.
    items
}

/// Faint lines every 4 m across the field, for a sense of speed.
fn field_grid() -> Vec<RenderSceneItem> {
    let mut items = Vec::new();
    let line = glow([0.06, 0.08, 0.07]);
    let mut x = -40.0;
    while x <= 56.0 {
        items.push(cuboid(
            Vec3::new(x, 0.005, 18.0),
            Quat::IDENTITY,
            Vec3::new(0.02, 0.004, 38.0),
            line,
        ));
        x += 4.0;
    }
    let mut z = -20.0;
    while z <= 56.0 {
        items.push(cuboid(
            Vec3::new(8.0, 0.005, z),
            Quat::IDENTITY,
            Vec3::new(48.0, 0.004, 0.02),
            line,
        ));
        z += 4.0;
    }
    items
}

/// A dark synthetic turf field.
fn field() -> RenderSceneItem {
    let (x0, x1, z0, z1) = (-40.0_f32, 56.0_f32, -20.0_f32, 56.0_f32);
    let mesh = TriangleMesh {
        positions: vec![[x0, 0.0, z0], [x1, 0.0, z0], [x1, 0.0, z1], [x0, 0.0, z1]],
        normals: vec![[0.0, 1.0, 0.0]; 4],
        texcoords: vec![[0.0; 2]; 4],
        // Counter-clockwise seen from above.
        indices: vec![0, 2, 1, 0, 3, 2],
        skinning: None,
    };
    let colour = [0.05, 0.09, 0.06, 1.0];
    RenderSceneItem {
        transform: MathTransform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: colour,
        mesh: Some(Arc::new(mesh)),
        base_color_texture: None,
        material: PbrMaterial::new(colour, 0.9, 0.0, [0.0; 3]),
    }
}

/// A square gate: a padded frame with LED strips on its inner edges, facing
/// along `direction`, on a stand down to the ground with a weighted base.
fn push_gate(items: &mut Vec<RenderSceneItem>, centre: Vec3, direction: Vec3, light: [f32; 3]) {
    let facing = Vec3::new(direction.x, 0.0, direction.z).normalize();
    // The frame lies in the plane across `facing`: its local x is `facing`.
    let rotation = Quat::from_rotation_arc(Vec3::X, facing);
    let half = 0.5 * GATE_OPENING_M;
    let bar = 0.09;
    let padding = matte([0.12, 0.12, 0.14, 1.0]);
    for (offset, extent) in [
        (
            Vec3::new(0.0, half + bar, 0.0),
            Vec3::new(bar, bar, half + 2.0 * bar),
        ),
        (
            Vec3::new(0.0, -half - bar, 0.0),
            Vec3::new(bar, bar, half + 2.0 * bar),
        ),
        (Vec3::new(0.0, 0.0, half + bar), Vec3::new(bar, half, bar)),
        (Vec3::new(0.0, 0.0, -half - bar), Vec3::new(bar, half, bar)),
    ] {
        items.push(cuboid(
            centre + rotation * offset,
            rotation,
            extent,
            padding,
        ));
        // The LED strip along the inner face of the bar.
        let inward = -offset.normalize();
        items.push(cuboid(
            centre + rotation * (offset + inward * (bar + 0.01)),
            rotation,
            if offset.y.abs() > 0.0 {
                Vec3::new(0.035, 0.03, half)
            } else {
                Vec3::new(0.035, half, 0.03)
            },
            glow([light[0] * 1.6, light[1] * 1.6, light[2] * 1.6]),
        ));
    }
    // Stand: a post from the bottom bar to a weighted base.
    let bottom = centre.y - half - 2.0 * bar;
    if bottom > 0.05 {
        let post = Vec3::new(centre.x, 0.5 * bottom, centre.z);
        items.push(cuboid(
            post,
            rotation,
            Vec3::new(0.06, 0.5 * bottom, 0.06),
            matte([0.35, 0.36, 0.4, 1.0]),
        ));
    }
    items.push(cuboid(
        Vec3::new(centre.x, 0.08, centre.z),
        rotation,
        Vec3::new(0.5, 0.08, 0.5),
        matte([0.2, 0.2, 0.22, 1.0]),
    ));
    items.push(cuboid(
        Vec3::new(centre.x, 0.165, centre.z),
        rotation,
        Vec3::new(0.52, 0.006, 0.52),
        glow([light[0] * 0.6, light[1] * 0.6, light[2] * 0.6]),
    ));
}

// ---------------------------------------------------------------------- drone

/// The last stretch of a drone's flight as a thin glowing line, fading
/// toward the oldest end.
fn push_trail(scene: &mut RenderScene, trail: &[Vec3], light: [f32; 3]) {
    let count = trail.len();
    for (age, pair) in trail.windows(2).enumerate() {
        let span = pair[1] - pair[0];
        if span.length() < 1e-3 {
            continue;
        }
        let fresh = ((age + 1) as f64 / count as f64) as f32;
        scene.items.push(cylinder(
            (pair[0] + pair[1]) * 0.5,
            Quat::from_rotation_arc(Vec3::Z, span.normalize()),
            0.018,
            span.length(),
            glow([light[0] * fresh, light[1] * fresh, light[2] * fresh]),
        ));
    }
}

/// A 7-inch racing quad drawn at its simulated pose: carbon X frame and
/// stack, motor bells, three-blade props with a spinning disc, a tilted FPV
/// camera, battery on top, antennas, and LED strips under the arms. The body
/// frame is +x forward, +y up.
fn push_quad(
    scene: &mut RenderScene,
    pose: &Transform3,
    frame: [f32; 4],
    light: [f32; 3],
    time_s: f64,
) {
    let rotation = pose.rotation;
    let origin = pose.translation;
    let at = |x: f64, y: f64, z: f64| origin + rotation * Vec3::new(x, y, z);
    let push = |scene: &mut RenderScene, item: RenderSceneItem| scene.items.push(item);
    // Frame: two crossed arms and a centre plate stack.
    for angle in [std::f64::consts::FRAC_PI_4, -std::f64::consts::FRAC_PI_4] {
        push(
            scene,
            cuboid(
                at(0.0, 0.0, 0.0),
                rotation * Quat::from_rotation_y(angle),
                Vec3::new(0.2, 0.006, 0.018),
                CARBON,
            ),
        );
    }
    push(
        scene,
        cuboid(
            at(0.0, 0.0, 0.0),
            rotation,
            Vec3::new(0.06, 0.004, 0.045),
            CARBON,
        ),
    );
    push(
        scene,
        cuboid(
            at(0.0, 0.035, 0.0),
            rotation,
            Vec3::new(0.055, 0.003, 0.04),
            CARBON,
        ),
    );
    for (x, z) in [
        (0.045, 0.03),
        (0.045, -0.03),
        (-0.045, 0.03),
        (-0.045, -0.03),
    ] {
        push(
            scene,
            cylinder(
                at(x, 0.018, z),
                rotation * Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                0.004,
                0.035,
                matte([0.7, 0.7, 0.72, 1.0]),
            ),
        );
    }
    // Canopy in the team colour, and the battery strapped on top.
    push(
        scene,
        cuboid(
            at(0.01, 0.052, 0.0),
            rotation,
            Vec3::new(0.05, 0.014, 0.035),
            Finish {
                color: frame,
                roughness: 0.4,
                metallic: 0.05,
                emissive: [0.0; 3],
            },
        ),
    );
    push(
        scene,
        cuboid(
            at(-0.01, 0.085, 0.0),
            rotation,
            Vec3::new(0.055, 0.018, 0.025),
            matte([0.12, 0.12, 0.13, 1.0]),
        ),
    );
    push(
        scene,
        cuboid(
            at(-0.01, 0.085, 0.0),
            rotation,
            Vec3::new(0.006, 0.019, 0.026),
            matte([0.8, 0.1, 0.1, 1.0]),
        ),
    );
    // FPV camera tilted up 30 degrees, the way racers fly fast.
    let camera_tilt = rotation * Quat::from_rotation_z(0.52);
    push(
        scene,
        cuboid(
            at(0.075, 0.02, 0.0),
            camera_tilt,
            Vec3::new(0.012, 0.012, 0.012),
            matte([0.1, 0.1, 0.1, 1.0]),
        ),
    );
    push(
        scene,
        cylinder(
            at(0.09, 0.028, 0.0),
            camera_tilt * Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            0.007,
            0.012,
            Finish {
                color: [0.05, 0.05, 0.08, 1.0],
                roughness: 0.05,
                metallic: 0.7,
                emissive: [0.0; 3],
            },
        ),
    );
    // Antennas trailing up and back.
    for side in [-1.0, 1.0] {
        push(
            scene,
            cylinder(
                at(-0.07, 0.07, side * 0.02),
                rotation
                    * Quat::from_rotation_arc(
                        Vec3::Z,
                        Vec3::new(-0.5, 1.0, side * 0.3).normalize(),
                    ),
                0.003,
                0.08,
                matte([0.15, 0.15, 0.15, 1.0]),
            ),
        );
    }
    push_rotors(scene, pose, frame, light, time_s);
}

/// Motors, three-blade props over a blur disc, and the arm LED strips.
fn push_rotors(
    scene: &mut RenderScene,
    pose: &Transform3,
    frame: [f32; 4],
    light: [f32; 3],
    time_s: f64,
) {
    let rotation = pose.rotation;
    let origin = pose.translation;
    let at = |x: f64, y: f64, z: f64| origin + rotation * Vec3::new(x, y, z);
    let push = |scene: &mut RenderScene, item: RenderSceneItem| scene.items.push(item);
    // Motors, props and the LED strip under each arm.
    let spin = time_s * 180.0;
    for (index, (x, z)) in [(0.13, 0.13), (0.13, -0.13), (-0.13, 0.13), (-0.13, -0.13)]
        .into_iter()
        .enumerate()
    {
        let direction = if index % 3 == 0 { 1.0 } else { -1.0 };
        let upright = rotation * Quat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        push(
            scene,
            cylinder(
                at(x, 0.018, z),
                upright,
                0.016,
                0.026,
                Finish {
                    color: frame,
                    roughness: 0.3,
                    metallic: 0.6,
                    emissive: [0.0; 3],
                },
            ),
        );
        push(
            scene,
            cylinder(
                at(x, 0.034, z),
                upright,
                0.006,
                0.008,
                matte([0.75, 0.75, 0.78, 1.0]),
            ),
        );
        // A faint disc where the blades are a blur, and three blades at
        // this instant.
        push(
            scene,
            cylinder(
                at(x, 0.037, z),
                upright,
                0.089,
                0.002,
                Finish {
                    color: [0.14, 0.14, 0.16, 1.0],
                    roughness: 0.6,
                    metallic: 0.0,
                    emissive: [0.01, 0.01, 0.012],
                },
            ),
        );
        for blade in 0..3 {
            let angle = direction * spin + f64::from(blade) * std::f64::consts::TAU / 3.0;
            let yaw = Quat::from_rotation_y(angle);
            push(
                scene,
                cuboid(
                    at(x, 0.04, z) + rotation * (yaw * Vec3::new(0.045, 0.0, 0.0)),
                    rotation * yaw,
                    Vec3::new(0.045, 0.002, 0.009),
                    matte([0.9, 0.9, 0.92, 1.0]),
                ),
            );
        }
        // LED strips along the arm, under it and on top.
        let along = rotation * Quat::from_rotation_y(-(z / x).atan());
        for y in [-0.009, 0.009] {
            push(
                scene,
                cuboid(
                    at(x * 0.55, y, z * 0.55),
                    along,
                    Vec3::new(0.06, 0.003, 0.007),
                    glow(light),
                ),
            );
        }
    }
}
