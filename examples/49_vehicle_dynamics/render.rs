//! Draws the comparison: a sweeper circuit with kerbs, runoff and tyre walls,
//! both cars built from their parts at their recorded poses, and the trail
//! each has left. Every pose, steering angle and speed on screen is a recorded
//! sample of the run; the scenery is render-only.

use crate::{ComparisonRun, ComparisonSample};
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_render::{
    EnvironmentLighting, EnvironmentMap, ImageFrame, PbrMaterial, RenderScene, RenderSceneItem,
    TriangleMesh, VisualShape,
};
use rne_render_wgpu::CameraOrbit;
use rne_robot::VehicleDynamics;
use rne_world::Transform3;
use std::f64::consts::PI;
use std::path::PathBuf;
use std::sync::Arc;

/// Half width of the road, meters.
const ROAD_HALF_M: f64 = 3.5;
/// Width of the kerb strip outside each road edge on the sweeper.
const KERB_M: f64 = 1.3;
/// The sweeper: centre and radius of the course's arc.
const ARC_CENTER: (f64, f64) = (40.0, -18.0);
const ARC_RADIUS_M: f64 = 18.0;
/// The runoff area's outer radius around the sweeper, and how far below the
/// exit straight it reaches.
const RUNOFF_OUTER_M: f64 = 36.0;
const RUNOFF_EXIT_Z_M: f64 = -56.0;
/// Tyre walls stand this far past the runoff's edge.
const WALL_GAP_M: f64 = 2.5;
/// Wheel radius of the drawn cars.
const WHEEL_RADIUS_M: f64 = 0.34;
const TRACK_HALF_M: f64 = 0.8;
/// Ends of the straights, past the course's own start and finish, so the road
/// runs out of shot rather than stopping in the grass.
const STRAIGHT_FROM_X_M: f64 = -34.0;

pub(crate) const KINEMATIC_PAINT: [f32; 4] = [0.10, 0.78, 0.52, 1.0];
pub(crate) const DYNAMIC_PAINT: [f32; 4] = [1.0, 0.50, 0.08, 1.0];
pub(crate) const SATURATED: [f32; 4] = [0.95, 0.16, 0.24, 1.0];
/// The tyre colour; tests count wheels by it.
pub(crate) const TYRE: [f32; 4] = [0.045, 0.045, 0.05, 1.0];

/// A fixed post outside the sweeper, raised, looking back up the approach:
/// the cars come from the far end, turn in front of it, and the dynamic car
/// runs wide toward it.
pub(crate) fn camera() -> CameraOrbit {
    // The camera sits along (sin yaw, cos yaw) in (x, z) from its focus.
    CameraOrbit {
        focus: Vec3::new(38.0, 0.0, -24.0),
        yaw_rad: 1.62,
        pitch_rad: 1.0,
        distance_m: 58.0,
    }
}

/// Everything that does not move, built once.
pub(crate) struct Circuit {
    items: Vec<RenderSceneItem>,
    props: Vec<(&'static str, Vec3, Quat)>,
}

impl Circuit {
    pub(crate) fn build() -> Self {
        let asphalt = load_textures("asphalt_track");
        let grass = load_textures("leafy_grass");
        let centre = centreline();
        let mut items = vec![ground(&grass), runoff(&asphalt), road(&centre, &asphalt)];
        items.extend(markings(&centre));
        items.extend(start_line());
        items.extend(brake_boards());
        items.extend(treeline());
        Self {
            items,
            props: barriers(),
        }
    }

    /// The props alone, without loading textures, for headless checks.
    pub(crate) fn build_props_only() -> Self {
        Self {
            items: Vec::new(),
            props: barriers(),
        }
    }

    /// The directory the prop paths are relative to.
    pub(crate) fn props_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_racing")
    }

    /// The circuit, the trails up to `frame`, and both cars at `frame`.
    pub(crate) fn scene(&self, run: &ComparisonRun, frame: usize) -> RenderScene {
        let mut scene = RenderScene::default();
        scene.items.extend(self.items.iter().cloned());
        for (path, base, rotation) in &self.props {
            push_prop(&mut scene, path, *base, *rotation);
        }
        let visible = frame.min(run.samples.len().saturating_sub(1));
        let shown = &run.samples[..=visible];
        scene.items.extend(trails(shown));
        let rolled = rolled_distances(shown);
        if let (Some(now), Some(before)) = (shown.last(), shown.iter().rev().nth(1)) {
            push_car(
                &mut scene,
                &CarState {
                    transform: now.kinematic_transform,
                    steering_rad: now.kinematic_steering_rad,
                    rolled_m: rolled.0,
                    braking: now.kinematic_speed_m_s < before.kinematic_speed_m_s - 0.05,
                },
                KINEMATIC_PAINT,
            );
            push_car(
                &mut scene,
                &CarState {
                    transform: now.dynamic_transform,
                    steering_rad: now.dynamic_steering_rad,
                    rolled_m: rolled.1,
                    braking: now.dynamic_speed_m_s < before.dynamic_speed_m_s - 0.05,
                },
                DYNAMIC_PAINT,
            );
        } else if let Some(now) = shown.last() {
            for (transform, paint) in [
                (now.kinematic_transform, KINEMATIC_PAINT),
                (now.dynamic_transform, DYNAMIC_PAINT),
            ] {
                push_car(
                    &mut scene,
                    &CarState {
                        transform,
                        steering_rad: 0.0,
                        rolled_m: 0.0,
                        braking: false,
                    },
                    paint,
                );
            }
        }
        scene
    }

    /// Every tyre-wall position, for the clearance check.
    pub(crate) fn wall_positions(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.props.iter().map(|(_, base, _)| *base)
    }
}

/// Distance each car has travelled over the shown samples, for wheel spin.
fn rolled_distances(samples: &[ComparisonSample]) -> (f64, f64) {
    samples.windows(2).fold((0.0, 0.0), |(k, d), pair| {
        (
            k + (pair[1].kinematic_transform.translation - pair[0].kinematic_transform.translation)
                .length(),
            d + (pair[1].dynamic_transform.translation - pair[0].dynamic_transform.translation)
                .length(),
        )
    })
}

/// The course's centreline, densely: the approach along +x, the left-hand
/// sweeper, and the exit straight back along -x. The course waypoints sample
/// the same three pieces every 5 m.
fn centreline() -> Vec<Vec3> {
    let mut points = Vec::new();
    let mut x = STRAIGHT_FROM_X_M;
    while x < ARC_CENTER.0 {
        points.push(Vec3::new(x, 0.0, 0.0));
        x += 1.0;
    }
    for step in 0..=90 {
        let angle = f64::from(step) / 90.0 * PI;
        points.push(Vec3::new(
            ARC_CENTER.0 + ARC_RADIUS_M * angle.sin(),
            0.0,
            ARC_CENTER.1 + ARC_RADIUS_M * angle.cos(),
        ));
    }
    let mut x = ARC_CENTER.0 - 1.0;
    while x >= STRAIGHT_FROM_X_M {
        points.push(Vec3::new(x, 0.0, 2.0 * ARC_CENTER.1));
        x -= 1.0;
    }
    points
}

/// The unit vector to the left of travel at each centreline point.
fn lefts(points: &[Vec3]) -> Vec<Vec3> {
    let n = points.len();
    (0..n)
        .map(|i| {
            let ahead = points[(i + 1).min(n - 1)] - points[i.saturating_sub(1)];
            Vec3::new(ahead.z, 0.0, -ahead.x).normalize()
        })
        .collect()
}

fn on_arc(point: Vec3) -> bool {
    point.x > ARC_CENTER.0 + 1e-6
}

// ------------------------------------------------------------------- circuit

fn ground(grass: &Textures) -> RenderSceneItem {
    let (x0, x1, z0, z1) = (-160.0, 200.0, -200.0, 140.0);
    let repeat = 5.0;
    let mut mesh = QuadMesh::default();
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

/// Paved runoff, painted blue, outside the sweeper and below the exit straight, where a car
/// that runs wide ends up: an annulus around the arc and a strip along the
/// exit.
fn runoff(asphalt: &Textures) -> RenderSceneItem {
    let mut mesh = QuadMesh::default();
    let (cx, cz) = ARC_CENTER;
    let inner = ARC_RADIUS_M + ROAD_HALF_M;
    let uv = |p: Vec3| [(p.x / 9.0) as f32, (p.z / 9.0) as f32];
    let y = 0.012;
    let steps = 60;
    for step in 0..steps {
        let a0 = f64::from(step) / f64::from(steps) * PI;
        let a1 = f64::from(step + 1) / f64::from(steps) * PI;
        let at = |angle: f64, radius: f64| {
            Vec3::new(cx + radius * angle.sin(), y, cz + radius * angle.cos())
        };
        let corners = [
            at(a0, inner),
            at(a1, inner),
            at(a1, RUNOFF_OUTER_M),
            at(a0, RUNOFF_OUTER_M),
        ];
        mesh.quad(corners, corners.map(uv));
    }
    // The strip below the exit straight, out to where the annulus ends.
    let exit_edge = 2.0 * cz - ROAD_HALF_M;
    let corners = [
        Vec3::new(STRAIGHT_FROM_X_M, y, exit_edge),
        Vec3::new(cx, y, exit_edge),
        Vec3::new(cx, y, RUNOFF_EXIT_Z_M),
        Vec3::new(STRAIGHT_FROM_X_M, y, RUNOFF_EXIT_Z_M),
    ];
    mesh.quad(corners, corners.map(uv));
    mesh_item(
        mesh.build().expect("runoff"),
        Finish::matte([0.22, 0.34, 0.56, 1.0]),
        None,
        Some((&asphalt.normal, &asphalt.roughness)),
    )
}

/// The asphalt ribbon, one texture repeat every 8 m.
fn road(centre: &[Vec3], asphalt: &Textures) -> RenderSceneItem {
    let left = lefts(centre);
    let mut mesh = QuadMesh::default();
    let mut along = 0.0;
    let across = (2.0 * ROAD_HALF_M / 8.0) as f32;
    for i in 0..centre.len() - 1 {
        let step = (centre[i + 1] - centre[i]).length();
        let (v0, v1) = ((along / 8.0) as f32, ((along + step) / 8.0) as f32);
        along += step;
        let at = |index: usize, side: f64| centre[index] + left[index] * side + Vec3::Y * 0.02;
        mesh.quad(
            [
                at(i, -ROAD_HALF_M),
                at(i + 1, -ROAD_HALF_M),
                at(i + 1, ROAD_HALF_M),
                at(i, ROAD_HALF_M),
            ],
            [[0.0, v0], [0.0, v1], [across, v1], [across, v0]],
        );
    }
    mesh_item(
        mesh.build().expect("road"),
        Finish::matte([0.55, 0.55, 0.57, 1.0]),
        Some(&asphalt.color),
        Some((&asphalt.normal, &asphalt.roughness)),
    )
}

/// White edge lines along the whole road, a dashed centre line on the
/// straights, and red-and-white kerbs on both edges of the sweeper.
fn markings(centre: &[Vec3]) -> Vec<RenderSceneItem> {
    let left = lefts(centre);
    let mut white = QuadMesh::default();
    let mut red = QuadMesh::default();
    let strip = |mesh: &mut QuadMesh, i: usize, from: f64, to: f64, y: f64| {
        let p = |index: usize, side: f64| centre[index] + left[index] * side + Vec3::Y * y;
        mesh.quad(
            [p(i, from), p(i + 1, from), p(i + 1, to), p(i, to)],
            [[0.0; 2]; 4],
        );
    };
    let mut arc_index = 0;
    for i in 0..centre.len() - 1 {
        for sign in [-1.0, 1.0] {
            strip(
                &mut white,
                i,
                sign * (ROAD_HALF_M - 0.3),
                sign * (ROAD_HALF_M - 0.15),
                0.028,
            );
        }
        if on_arc(centre[i]) && on_arc(centre[i + 1]) {
            let mesh = if (arc_index / 2) % 2 == 0 {
                &mut red
            } else {
                &mut white
            };
            arc_index += 1;
            for sign in [-1.0, 1.0] {
                strip(
                    mesh,
                    i,
                    sign * ROAD_HALF_M,
                    sign * (ROAD_HALF_M + KERB_M),
                    0.034,
                );
            }
        } else if i % 6 < 3 {
            strip(&mut white, i, -0.08, 0.08, 0.028);
        }
    }
    let mut items = Vec::new();
    for (mesh, colour) in [
        (white, [0.93, 0.93, 0.92, 1.0]),
        (red, [0.80, 0.08, 0.07, 1.0]),
    ] {
        if let Some(mesh) = mesh.build() {
            items.push(mesh_item(mesh, Finish::matte(colour), None, None));
        }
    }
    items
}

/// A chequered line across the approach where both cars start, at x = 0.
fn start_line() -> Vec<RenderSceneItem> {
    let mut black = QuadMesh::default();
    let mut white = QuadMesh::default();
    let squares = 14;
    let size = 2.0 * ROAD_HALF_M / f64::from(squares);
    for row in 0..2 {
        for column in 0..squares {
            let mesh = if (row + column) % 2 == 0 {
                &mut black
            } else {
                &mut white
            };
            let x = -0.5 * size + f64::from(row) * size - size;
            let z = -ROAD_HALF_M + f64::from(column) * size;
            mesh.quad(
                [
                    Vec3::new(x, 0.03, z),
                    Vec3::new(x + size, 0.03, z),
                    Vec3::new(x + size, 0.03, z + size),
                    Vec3::new(x, 0.03, z + size),
                ],
                [[0.0; 2]; 4],
            );
        }
    }
    [(black, [0.05, 0.05, 0.05, 1.0]), (white, [0.95; 4])]
        .into_iter()
        .filter_map(|(mesh, colour)| {
            mesh.build()
                .map(|mesh| mesh_item(mesh, Finish::matte(colour), None, None))
        })
        .collect()
}

/// Three countdown boards outside the approach before the sweeper: three,
/// two and one bars.
fn brake_boards() -> Vec<RenderSceneItem> {
    let mut items = Vec::new();
    let mut push = |center: Vec3, half: Vec3, finish: Finish| {
        items.push(material_item(
            Transform3::from_translation_rotation(center, Quat::IDENTITY),
            VisualShape::Box { size_m: half * 2.0 },
            finish,
        ));
    };
    for (bars, x) in [(3, 22.0), (2, 28.0), (1, 34.0)] {
        let z = ROAD_HALF_M + 2.6;
        push(Vec3::new(x, 0.55, z), Vec3::new(0.04, 0.55, 0.04), TITANIUM);
        push(
            Vec3::new(x - 0.05, 1.45, z),
            Vec3::new(0.03, 0.42, 0.62),
            Finish::matte([0.95, 0.95, 0.93, 1.0]),
        );
        for bar in 0..bars {
            push(
                Vec3::new(x - 0.085, 1.22 + 0.2 * f64::from(bar), z),
                Vec3::new(0.005, 0.06, 0.5),
                Finish::matte([0.06, 0.06, 0.07, 1.0]),
            );
        }
    }
    items
}

/// A line of trees across the far end of the approach, where the ground
/// would otherwise meet the sky as a flat band.
fn treeline() -> Vec<RenderSceneItem> {
    let mut items = Vec::new();
    let bark = Finish::matte([0.24, 0.17, 0.11, 1.0]);
    let leaves = [
        Finish::matte([0.12, 0.30, 0.12, 1.0]),
        Finish::matte([0.17, 0.38, 0.15, 1.0]),
        Finish::matte([0.10, 0.25, 0.11, 1.0]),
    ];
    for index in 0..40 {
        let k = f64::from(index);
        // A fixed scatter, so every frame draws the same trees.
        let jitter = ((k * 12.9898).sin() * 43_758.545).fract();
        let x = -36.0 - 8.0 * jitter;
        let z = -80.0 + k * 3.4;
        let height = 5.0 + 3.0 * jitter;
        items.push(material_item(
            Transform3::from_translation_rotation(
                Vec3::new(x, 0.5 * height, z),
                Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            ),
            VisualShape::Cylinder {
                radius_m: 0.25,
                length_m: height,
            },
            bark,
        ));
        items.push(material_item(
            Transform3::from_translation_rotation(
                Vec3::new(x, 0.8 * height + 0.8, z),
                Quat::IDENTITY,
            ),
            VisualShape::Sphere {
                radius_m: 2.6 + jitter,
            },
            leaves[index as usize % leaves.len()],
        ));
    }
    items
}

/// Tyre walls two high past the runoff around the sweeper and below the exit,
/// and concrete barriers along the infield side of the approach.
fn barriers() -> Vec<(&'static str, Vec3, Quat)> {
    let mut props = Vec::new();
    let lying = Quat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let mut tyres = |x: f64, z: f64| {
        for layer in 0..2 {
            props.push((
                "old_tyre/old_tyre_1k.gltf",
                Vec3::new(x, 0.083 + 0.166 * f64::from(layer), z),
                lying,
            ));
        }
    };
    let radius = RUNOFF_OUTER_M + WALL_GAP_M;
    let count = (PI * radius / 0.62).floor() as usize;
    for index in 0..=count {
        let angle = index as f64 / count as f64 * PI;
        tyres(
            ARC_CENTER.0 + radius * angle.sin(),
            ARC_CENTER.1 + radius * angle.cos(),
        );
    }
    let mut x = ARC_CENTER.0 - 0.62;
    while x > STRAIGHT_FROM_X_M {
        tyres(x, RUNOFF_EXIT_Z_M - WALL_GAP_M);
        x -= 0.62;
    }
    let mut x = -24.0;
    while x < 30.0 {
        props.push((
            "concrete_road_barrier/concrete_road_barrier_1k.gltf",
            Vec3::new(x, 0.0, -(ROAD_HALF_M + 2.2)),
            Quat::IDENTITY,
        ));
        x += 1.55;
    }
    props
}

/// Each car's trail on the road, as a ribbon through its recorded positions:
/// green for the kinematic car; orange for the dynamic car, red wherever its
/// front axle was beyond its grip.
fn trails(samples: &[ComparisonSample]) -> Vec<RenderSceneItem> {
    let mut kinematic = QuadMesh::default();
    let mut dynamic = QuadMesh::default();
    let mut saturated = QuadMesh::default();
    let ribbon = |mesh: &mut QuadMesh, a: Vec3, b: Vec3, y: f64| {
        let along = b - a;
        if along.length() < 1e-3 {
            return;
        }
        let side = Vec3::new(along.z, 0.0, -along.x).normalize() * 0.16;
        let lift = Vec3::Y * y;
        mesh.quad(
            [
                a - side + lift,
                b - side + lift,
                b + side + lift,
                a + side + lift,
            ],
            [[0.0; 2]; 4],
        );
    };
    for pair in samples.windows(2) {
        ribbon(
            &mut kinematic,
            pair[0].kinematic_transform.translation,
            pair[1].kinematic_transform.translation,
            0.05,
        );
        ribbon(
            if pair[1].front_saturated {
                &mut saturated
            } else {
                &mut dynamic
            },
            pair[0].dynamic_transform.translation,
            pair[1].dynamic_transform.translation,
            0.055,
        );
    }
    [
        (kinematic, KINEMATIC_PAINT),
        (dynamic, DYNAMIC_PAINT),
        (saturated, SATURATED),
    ]
    .into_iter()
    .filter_map(|(mesh, colour)| {
        mesh.build().map(|mesh| {
            mesh_item(
                mesh,
                Finish {
                    emissive: [colour[0] * 0.35, colour[1] * 0.35, colour[2] * 0.35],
                    ..Finish::matte(colour)
                },
                None,
                None,
            )
        })
    })
    .collect()
}

// ----------------------------------------------------------------------- car

struct CarState {
    transform: Transform3,
    steering_rad: f64,
    rolled_m: f64,
    braking: bool,
}

/// A GT coupé drawn around the car's recorded pose. The car's own frame has
/// +x forward from its centre of mass, +y up and z across; the axles are
/// where [`VehicleDynamics::default`] puts them.
fn push_car(scene: &mut RenderScene, car: &CarState, paint: [f32; 4]) {
    let rotation = car.transform.rotation;
    let base = car.transform.translation;
    let at = |x: f64, y: f64, z: f64| base + rotation * Vec3::new(x, y, z);
    let pitched = |angle: f64| rotation * Quat::from_rotation_z(angle);
    let body = Finish::paint(paint);
    let glass = Finish {
        color: [0.05, 0.08, 0.11, 1.0],
        roughness: 0.05,
        metallic: 0.3,
        emissive: [0.0; 3],
    };
    let dark = Finish::matte([0.03, 0.03, 0.035, 1.0]);
    let level = rotation;
    let front = VehicleDynamics::default().front_axle_m;
    let rear = -VehicleDynamics::default().rear_axle_m;

    // Lower body, sills and the tub between the arches.
    push_box(
        scene,
        at(-0.1, 0.52, 0.0),
        level,
        Vec3::new(2.05, 0.2, 0.9),
        body,
    );
    push_box(
        scene,
        at(-0.1, 0.3, 0.0),
        level,
        Vec3::new(1.95, 0.06, 0.86),
        dark,
    );
    // Nose: a bonnet that slopes down to a low bumper, a splitter and grille.
    push_box(
        scene,
        at(front + 0.25, 0.7, 0.0),
        pitched(-0.09),
        Vec3::new(0.78, 0.045, 0.86),
        body,
    );
    push_box(
        scene,
        at(front + 0.9, 0.5, 0.0),
        pitched(-0.25),
        Vec3::new(0.2, 0.16, 0.88),
        body,
    );
    push_box(
        scene,
        at(front + 1.02, 0.22, 0.0),
        level,
        Vec3::new(0.18, 0.018, 0.9),
        CARBON,
    );
    push_box(
        scene,
        at(front + 1.07, 0.44, 0.0),
        level,
        Vec3::new(0.02, 0.07, 0.5),
        dark,
    );
    // Arches: fenders over each wheel, wider at the rear haunches.
    for (x, half_x, top, reach) in [(front, 0.52, 0.76, 0.9), (rear, 0.6, 0.8, 0.94)] {
        for side in [-1.0, 1.0] {
            push_box(
                scene,
                at(x, top - 0.1, side * (reach - 0.1)),
                level,
                Vec3::new(half_x, 0.1, 0.1),
                body,
            );
        }
    }
    // Greenhouse: tinted glass, a raked windscreen and rear screen, and a
    // painted roof.
    push_box(
        scene,
        at(-0.35, 0.92, 0.0),
        level,
        Vec3::new(0.8, 0.2, 0.72),
        glass,
    );
    push_box(
        scene,
        at(0.62, 0.93, 0.0),
        pitched(0.98),
        Vec3::new(0.02, 0.36, 0.7),
        glass,
    );
    push_box(
        scene,
        at(-1.33, 0.93, 0.0),
        pitched(-0.9),
        Vec3::new(0.02, 0.34, 0.68),
        glass,
    );
    push_box(
        scene,
        at(-0.4, 1.14, 0.0),
        level,
        Vec3::new(0.66, 0.035, 0.68),
        body,
    );
    // Rear deck, bumper, diffuser, exhausts.
    push_box(
        scene,
        at(rear - 0.3, 0.76, 0.0),
        level,
        Vec3::new(0.52, 0.05, 0.9),
        body,
    );
    push_box(
        scene,
        at(rear - 0.72, 0.52, 0.0),
        level,
        Vec3::new(0.12, 0.2, 0.9),
        body,
    );
    push_box(
        scene,
        at(rear - 0.74, 0.26, 0.0),
        level,
        Vec3::new(0.14, 0.05, 0.78),
        CARBON,
    );
    push_trim(scene, car, paint, front, rear);
    push_wheels(scene, car, &at, front, rear);
}

/// Stripe, skirts, roundels, mirrors, rear wing, lights and exhausts.
fn push_trim(scene: &mut RenderScene, car: &CarState, paint: [f32; 4], front: f64, rear: f64) {
    let rotation = car.transform.rotation;
    let base = car.transform.translation;
    let at = |x: f64, y: f64, z: f64| base + rotation * Vec3::new(x, y, z);
    let pitched = |angle: f64| rotation * Quat::from_rotation_z(angle);
    let body = Finish::paint(paint);
    let level = rotation;
    let trim = Finish::paint([0.95, 0.95, 0.93, 1.0]);
    // Racing stripe over bonnet, roof and deck.
    for (x, y, half_x, angle) in [
        (front + 0.25, 0.748, 0.78, -0.09),
        (-0.4, 1.178, 0.66, 0.0),
        (rear - 0.3, 0.812, 0.52, 0.0),
    ] {
        push_box(
            scene,
            at(x, y, 0.0),
            pitched(angle),
            Vec3::new(half_x, 0.004, 0.14),
            trim,
        );
    }
    // Side skirts, door roundels and mirrors.
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            at(-0.1, 0.27, side * 0.9),
            level,
            Vec3::new(1.2, 0.05, 0.03),
            CARBON,
        );
        push_cylinder_rotated(
            scene,
            at(-0.2, 0.55, side * 0.905),
            rotation,
            0.19,
            0.012,
            trim,
        );
        push_box(
            scene,
            at(0.42, 0.92, side * 0.88),
            rotation,
            Vec3::new(0.07, 0.05, 0.1),
            body,
        );
    }
    // Rear wing on two uprights, with end plates.
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            at(rear - 0.5, 0.94, side * 0.5),
            rotation,
            Vec3::new(0.05, 0.14, 0.018),
            CARBON,
        );
        push_box(
            scene,
            at(rear - 0.55, 1.1, side * 0.86),
            rotation,
            Vec3::new(0.2, 0.09, 0.01),
            CARBON,
        );
    }
    push_box(
        scene,
        at(rear - 0.55, 1.1, 0.0),
        pitched(0.12),
        Vec3::new(0.19, 0.015, 0.86),
        CARBON,
    );
    // Lights: headlamps, and a tail bar that brightens when the recorded
    // speed falls.
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            at(front + 0.93, 0.64, side * 0.62),
            rotation * Quat::from_rotation_y(side * 0.35),
            Vec3::new(0.05, 0.045, 0.2),
            Finish {
                emissive: [1.6, 1.55, 1.35],
                ..Finish::paint([1.0, 0.98, 0.9, 1.0])
            },
        );
    }
    let tail = if car.braking { 3.0 } else { 0.9 };
    push_box(
        scene,
        at(rear - 0.85, 0.68, 0.0),
        rotation,
        Vec3::new(0.02, 0.035, 0.8),
        Finish {
            emissive: [tail, 0.05 * tail, 0.04 * tail],
            ..Finish::paint([0.8, 0.05, 0.04, 1.0])
        },
    );
    for side in [-1.0, 1.0] {
        push_cylinder(
            scene,
            at(rear - 0.86, 0.3, side * 0.4),
            rotation * Vec3::X,
            0.05,
            0.14,
            TITANIUM,
        );
    }
}

/// Wheels that spin with the distance the car has covered and, at the front,
/// turn with its recorded steering angle: tyre, alloy rim, five spokes, hub,
/// brake disc, and a caliper that steers but does not spin.
fn push_wheels(
    scene: &mut RenderScene,
    car: &CarState,
    at: &dyn Fn(f64, f64, f64) -> Vec3,
    front: f64,
    rear: f64,
) {
    let rotation = car.transform.rotation;
    let caliper = Finish::paint([0.85, 0.1, 0.08, 1.0]);
    let spin = Quat::from_rotation_z(-car.rolled_m / WHEEL_RADIUS_M);
    for (x, steered) in [(front, true), (rear, false)] {
        for side in [-1.0, 1.0] {
            let hub = at(x, WHEEL_RADIUS_M, side * TRACK_HALF_M);
            let steer = if steered {
                Quat::from_rotation_y(car.steering_rad)
            } else {
                Quat::IDENTITY
            };
            // The wheel's axle is its local z.
            let upright = rotation * steer;
            let wheel = upright * spin;
            let outward = |depth: f64| upright * Vec3::new(0.0, 0.0, side * depth);
            scene.items.push(material_item(
                Transform3::from_translation_rotation(hub, wheel),
                VisualShape::Cylinder {
                    radius_m: WHEEL_RADIUS_M,
                    length_m: 0.26,
                },
                Finish::matte(TYRE),
            ));
            push_cylinder_rotated(scene, hub + outward(0.004), wheel, 0.235, 0.262, ALLOY);
            push_cylinder_rotated(scene, hub + outward(0.012), wheel, 0.2, 0.25, DARK_METAL);
            for spoke in 0..5 {
                let angle = f64::from(spoke) * 2.0 * PI / 5.0;
                let direction = wheel * Quat::from_rotation_z(angle);
                push_box(
                    scene,
                    hub + direction * Vec3::new(0.11, 0.0, 0.0) + outward(0.128),
                    direction,
                    Vec3::new(0.1, 0.022, 0.012),
                    ALLOY,
                );
            }
            push_cylinder_rotated(scene, hub + outward(0.135), wheel, 0.045, 0.02, TITANIUM);
            push_cylinder_rotated(scene, hub + outward(0.03), wheel, 0.18, 0.025, TITANIUM);
            push_box(
                scene,
                hub + upright * Vec3::new(-0.13, 0.08, side * 0.06),
                upright * Quat::from_rotation_z(0.55),
                Vec3::new(0.045, 0.07, 0.035),
                caliper,
            );
        }
    }
}

// ---------------------------------------------------------------- materials

/// A clear afternoon sky: bright near the horizon, deep blue overhead, a warm
/// sun low in one direction, and green ground bounce.
pub(crate) fn sky() -> EnvironmentLighting {
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
            roughness: 0.22,
            metallic: 0.25,
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
const ALLOY: Finish = Finish {
    color: [0.72, 0.73, 0.76, 1.0],
    roughness: 0.22,
    metallic: 0.9,
    emissive: [0.0; 3],
};
const DARK_METAL: Finish = Finish {
    color: [0.12, 0.12, 0.13, 1.0],
    roughness: 0.4,
    metallic: 0.6,
    emissive: [0.0; 3],
};
const TITANIUM: Finish = Finish {
    color: [0.36, 0.37, 0.4, 1.0],
    roughness: 0.3,
    metallic: 0.8,
    emissive: [0.0; 3],
};

fn material_item(transform: Transform3, shape: VisualShape, finish: Finish) -> RenderSceneItem {
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

fn push_box(scene: &mut RenderScene, center: Vec3, rotation: Quat, half: Vec3, finish: Finish) {
    scene.items.push(material_item(
        Transform3::from_translation_rotation(center, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        finish,
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
    push_cylinder_rotated(
        scene,
        center,
        Quat::from_rotation_arc(Vec3::Z, axis.normalize()),
        radius_m,
        length_m,
        finish,
    );
}

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
        finish,
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

/// Quads collected into one mesh, wound to face up.
#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    /// A quad with corners in order around its edge.
    fn quad(&mut self, corners: [Vec3; 4], uv: [[f32; 2]; 4]) {
        let base = self.positions.len() as u32;
        for corner in corners {
            self.positions
                .push([corner.x as f32, corner.y as f32, corner.z as f32]);
        }
        self.texcoords.extend_from_slice(&uv);
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
    let root = Circuit::props_root().join("textures");
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
