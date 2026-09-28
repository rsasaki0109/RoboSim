//! Render-only dressing for the lift scene: the building, the lift's machinery
//! and fittings, and the service robot, all placed from the poses physics and
//! `rne_nav::Elevator` report. None of it has a collider; the bodies the
//! mission solves are the slabs, car, door leaves, button and robot box in
//! `main.rs`, and everything here is drawn on or around them.

use super::{
    Frame, Phase, BUTTON_CENTER_M, BUTTON_HALF_M, BUTTON_NORMAL, CAR_HALF_M, DOOR_HALF_M,
    FLOOR_HEIGHTS_M, ROBOT_HALF_M, SHAFT_X_M,
};
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_render::{
    EnvironmentLighting, EnvironmentMap, ImageFrame, PbrMaterial, RenderScene, RenderSceneItem,
    TriangleMesh, VisualShape,
};
use rne_world::Transform3;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

/// Top of the shaft, where the traction machine sits.
const SHAFT_TOP_Y_M: f64 = 5.6;
/// Inside height of the car, floor to ceiling.
const CAB_HEIGHT_M: f64 = 2.25;
/// Lobby slab extents, matching the colliders `main.rs` spawns.
const SLAB_CENTER_X_M: f64 = SHAFT_X_M - 3.9;
const SLAB_HALF_M: Vec3 = Vec3::new(3.0, 0.06, 1.2);
/// Plane of the lobby's back wall, just behind the call button's plate.
const LOBBY_WALL_Z_M: f64 = -1.08;

const LAMP_OFF: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
const LAMP_AMBER: [f32; 3] = [1.0, 0.62, 0.12];
const LAMP_GREEN: [f32; 3] = [0.25, 1.0, 0.45];

/// Scanned CC0 props from Poly Haven, fetched by
/// `tools/prepare_polyhaven_warehouse.py` (licence and hashes in
/// `assets/props/polyhaven_warehouse/manifest.json`).
pub(crate) fn props_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/props/polyhaven_warehouse")
}

const CARDBOARD_BOX: &str = "cardboard_box_01/cardboard_box_01_1k.gltf";

/// Equirectangular environment map width and height, in pixels.
const ENVIRONMENT_W: u32 = 128;
const ENVIRONMENT_H: u32 = 64;

/// A synthesised lobby interior to light the scene by: a warm, bright ceiling
/// with light bands, pale walls and a dim floor. Without it every face turned
/// from the single directional light renders black.
pub(crate) fn lobby_environment() -> EnvironmentLighting {
    let mut rgba32f = Vec::with_capacity((ENVIRONMENT_W * ENVIRONMENT_H * 4) as usize);
    for row in 0..ENVIRONMENT_H {
        let down = (row as f32 + 0.5) / ENVIRONMENT_H as f32;
        for column in 0..ENVIRONMENT_W {
            let around = (column as f32 + 0.5) / ENVIRONMENT_W as f32;
            let (r, g, b) = if down < 0.30 {
                let bands = (around * std::f32::consts::TAU * 3.0).sin().max(0.0);
                let fixture = bands.powf(16.0);
                let base = 0.80 + 0.80 * (1.0 - down / 0.30);
                (
                    base * 1.04 + 4.2 * fixture,
                    base + 4.0 * fixture,
                    base * 0.94 + 3.6 * fixture,
                )
            } else if down < 0.62 {
                let t = (down - 0.30) / 0.32;
                let level = 0.58 - 0.26 * t;
                (level, level * 0.97, level * 0.92)
            } else {
                (0.17, 0.16, 0.15)
            };
            rgba32f.extend_from_slice(&[r, g, b, 1.0]);
        }
    }
    let map = EnvironmentMap::from_rgba32f(ENVIRONMENT_W, ENVIRONMENT_H, rgba32f)
        .expect("lobby environment map");
    EnvironmentLighting {
        map: Some(Arc::new(map)),
        intensity: 1.0,
        diffuse_strength: 0.72,
        specular_strength: 0.38,
        rotation_rad: 0.0,
    }
}

struct TextureSet {
    color: Arc<ImageFrame>,
    normal: Arc<ImageFrame>,
    roughness: Arc<ImageFrame>,
}

fn load_texture(path: &Path) -> Arc<ImageFrame> {
    let rgba = image::open(path)
        .unwrap_or_else(|error| panic!("load render texture {}: {error}", path.display()))
        .into_rgba8();
    Arc::new(ImageFrame::from_rgba8(
        rgba.width(),
        rgba.height(),
        rgba.into_raw(),
    ))
}

/// Poured concrete, shared with the photoreal test bay.
fn concrete() -> &'static TextureSet {
    static CONCRETE: OnceLock<TextureSet> = OnceLock::new();
    CONCRETE.get_or_init(|| {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../63_g1_stride_gif/assets/photoreal_test_bay");
        TextureSet {
            color: load_texture(&root.join("concrete_floor_basecolor.png")),
            normal: load_texture(&root.join("concrete_floor_normal.png")),
            roughness: load_texture(&root.join("concrete_floor_roughness.png")),
        }
    })
}

/// A textured rectangle facing `u x v`, spanning `+-u` and `+-v` about
/// `center`, one texture repeat every `repeat_m` meters.
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn push_textured_panel(
    scene: &mut RenderScene,
    center: Vec3,
    u: Vec3,
    v: Vec3,
    textures: &TextureSet,
    repeat_m: f64,
    tint: [f32; 4],
    metallic: f32,
) {
    let (repeat_u, repeat_v) = (
        (2.0 * u.length() / repeat_m) as f32,
        (2.0 * v.length() / repeat_m) as f32,
    );
    let normal = u.cross(v).normalize();
    let corner = |a: f64, b: f64| {
        let point = u * a + v * b;
        [point.x as f32, point.y as f32, point.z as f32]
    };
    let mesh = TriangleMesh {
        positions: vec![
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ],
        normals: vec![[normal.x as f32, normal.y as f32, normal.z as f32]; 4],
        texcoords: vec![
            [0.0, repeat_v],
            [repeat_u, repeat_v],
            [repeat_u, 0.0],
            [0.0, 0.0],
        ],
        // Counter-clockwise seen from the side `normal` points to.
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: tint,
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&textures.color)),
        material: PbrMaterial::new(tint, 0.8, metallic, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&textures.normal)),
            Some(Arc::clone(&textures.roughness)),
        ),
    });
}

/// A box with real surface parameters, at any orientation.
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn push_box(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    half: Vec3,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    // Through `item_from_visual`, which folds the box size into the transform.
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(translation, rotation),
        VisualShape::Box { size_m: half * 2.0 },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    scene.items.push(item);
}

/// An axis-aligned, non-emissive box.
fn push_plain(
    scene: &mut RenderScene,
    translation: Vec3,
    half: Vec3,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
) {
    push_box(
        scene,
        translation,
        Quat::IDENTITY,
        half,
        color,
        roughness,
        metallic,
        [0.0; 3],
    );
}

/// A lamp: dark when off, glowing `emissive` when on.
fn push_lamp(scene: &mut RenderScene, translation: Vec3, half: Vec3, lit: Option<[f32; 3]>) {
    match lit {
        Some(glow) => push_box(
            scene,
            translation,
            Quat::IDENTITY,
            half,
            [glow[0], glow[1], glow[2], 1.0],
            0.3,
            0.0,
            glow,
        ),
        None => push_plain(scene, translation, half, LAMP_OFF, 0.4, 0.1),
    }
}

/// A cylinder with its axis along `axis` (a unit vector in world space).
#[allow(clippy::too_many_arguments)] // Each argument is an independent quantity.
fn push_cylinder(
    scene: &mut RenderScene,
    center: Vec3,
    axis: Vec3,
    radius_m: f64,
    length_m: f64,
    color: [f32; 4],
    roughness: f32,
    metallic: f32,
    emissive: [f32; 3],
) {
    let mut item = RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center, Quat::from_rotation_arc(Vec3::Z, axis)),
        VisualShape::Cylinder { radius_m, length_m },
        color,
        Transform3::IDENTITY,
    );
    item.material = PbrMaterial::new(color, roughness, metallic, emissive);
    scene.items.push(item);
}

/// A scanned prop standing at `base`.
fn push_prop(scene: &mut RenderScene, path: &str, base: Vec3, rotation: Quat, scale: Vec3) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(base, rotation),
        VisualShape::Mesh {
            path: path.to_string(),
            scale,
        },
        [1.0; 4],
        Transform3::IDENTITY,
    ));
}

/// Draws one frame of the scene.
pub(crate) fn append_site(scene: &mut RenderScene, frame: &Frame) {
    push_shaft(scene, frame.car_y_m);
    for (floor, height_m) in FLOOR_HEIGHTS_M.iter().enumerate() {
        push_lobby(scene, floor, *height_m, frame);
    }
    push_car(scene, frame);
    push_robot(scene, frame);
}

/// The shaft: clad walls, guide rails, the traction machine at the top, hoist
/// ropes to the car, and the counterweight that travels the other way.
fn push_shaft(scene: &mut RenderScene, car_y_m: f64) {
    const RAIL: [f32; 4] = [0.30, 0.32, 0.35, 1.0];
    let bottom_y_m = -0.7;
    let half_height_m = 0.5 * (SHAFT_TOP_Y_M - bottom_y_m);
    let mid_y_m = bottom_y_m + half_height_m;
    // Back and far side, poured concrete.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + 1.0, mid_y_m, 0.0),
        Vec3::new(0.05, half_height_m, 1.1),
        [0.30, 0.32, 0.36, 1.0],
        0.9,
        0.0,
    );
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + 0.05, mid_y_m, -1.12),
        Vec3::new(1.0, half_height_m, 0.05),
        [0.30, 0.32, 0.36, 1.0],
        0.9,
        0.0,
    );
    push_textured_panel(
        scene,
        Vec3::new(SHAFT_X_M + 0.945, mid_y_m, 0.0),
        Vec3::new(0.0, 0.0, 1.05),
        Vec3::new(0.0, half_height_m, 0.0),
        concrete(),
        1.6,
        [0.78, 0.80, 0.84, 1.0],
        0.0,
    );
    push_textured_panel(
        scene,
        Vec3::new(SHAFT_X_M + 0.05, mid_y_m, -1.065),
        Vec3::new(0.95, 0.0, 0.0),
        Vec3::new(0.0, half_height_m, 0.0),
        concrete(),
        1.6,
        [0.78, 0.80, 0.84, 1.0],
        0.0,
    );
    // Car guide rails on the far wall, counterweight rails on the back.
    for z in [-0.55, 0.55] {
        push_plain(
            scene,
            Vec3::new(SHAFT_X_M + 0.92, mid_y_m, z),
            Vec3::new(0.025, half_height_m, 0.035),
            RAIL,
            0.35,
            0.7,
        );
    }
    for x in [0.25, 0.75] {
        push_plain(
            scene,
            Vec3::new(SHAFT_X_M + x, mid_y_m, -1.03),
            Vec3::new(0.025, half_height_m, 0.02),
            RAIL,
            0.35,
            0.7,
        );
    }
    push_hoist(scene, car_y_m);
}

/// One landing: floor slab, back wall with skirting and a wayfinding band,
/// the landing's door frame, the hall panel and position indicator, ceiling
/// lights and props.
fn push_lobby(scene: &mut RenderScene, floor: usize, height_m: f64, frame: &Frame) {
    const WALL: [f32; 4] = [0.80, 0.77, 0.71, 1.0];
    const SKIRT: [f32; 4] = [0.20, 0.21, 0.23, 1.0];
    const BAND: [f32; 4] = [0.10, 0.52, 0.62, 1.0];
    const FRAME: [f32; 4] = [0.70, 0.72, 0.76, 1.0];
    const HAZARD: [f32; 4] = [0.95, 0.78, 0.10, 1.0];

    // Slab: textured top, and its own edge so it reads as a thickness.
    let slab_center = Vec3::new(SLAB_CENTER_X_M, height_m - SLAB_HALF_M.y, 0.0);
    push_textured_panel(
        scene,
        Vec3::new(SLAB_CENTER_X_M, height_m + 0.001, 0.0),
        Vec3::new(SLAB_HALF_M.x, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -SLAB_HALF_M.z),
        concrete(),
        1.5,
        [0.95, 0.93, 0.90, 1.0],
        0.0,
    );
    push_plain(
        scene,
        slab_center - Vec3::new(0.0, 0.004, 0.0),
        SLAB_HALF_M,
        [0.46, 0.47, 0.49, 1.0],
        0.95,
        0.0,
    );
    // Threshold: hazard stripes where the floor gives way to the shaft.
    for stripe in 0..6 {
        push_plain(
            scene,
            Vec3::new(
                SHAFT_X_M - 1.0,
                height_m + 0.003,
                -0.75 + f64::from(stripe) * 0.30,
            ),
            Vec3::new(0.08, 0.003, 0.08),
            HAZARD,
            0.7,
            0.0,
        );
    }

    // Back wall of the lobby.
    let wall_center_x = SHAFT_X_M - 3.4;
    let wall_half_x = 2.5;
    push_plain(
        scene,
        Vec3::new(wall_center_x, height_m + 1.25, LOBBY_WALL_Z_M - 0.05),
        Vec3::new(wall_half_x, 1.25, 0.05),
        WALL,
        0.85,
        0.0,
    );
    push_plain(
        scene,
        Vec3::new(wall_center_x, height_m + 0.06, LOBBY_WALL_Z_M + 0.01),
        Vec3::new(wall_half_x, 0.06, 0.01),
        SKIRT,
        0.5,
        0.1,
    );
    push_plain(
        scene,
        Vec3::new(wall_center_x, height_m + 1.0, LOBBY_WALL_Z_M + 0.005),
        Vec3::new(wall_half_x, 0.05, 0.005),
        BAND,
        0.6,
        0.0,
    );

    // Landing door frame: jambs and a header around the car's doorway.
    let jamb_x = SHAFT_X_M - CAR_HALF_M.x - 0.12;
    for z in [-0.93, 0.93] {
        push_plain(
            scene,
            Vec3::new(jamb_x, height_m + 1.15, z),
            Vec3::new(0.06, 1.15, 0.07),
            FRAME,
            0.3,
            0.7,
        );
    }
    push_plain(
        scene,
        Vec3::new(jamb_x, height_m + 2.38, 0.0),
        Vec3::new(0.06, 0.08, 1.0),
        FRAME,
        0.3,
        0.7,
    );

    push_hall_fixtures(scene, floor, height_m, frame);

    // Ceiling battens: on the ground floor under the upper slab, and on the
    // upper floor at the same height above it.
    for bay in 0..3 {
        let at = Vec3::new(
            SHAFT_X_M - 5.6 + f64::from(bay) * 1.7,
            height_m + 2.9,
            -0.35,
        );
        push_prop(
            scene,
            "mounted_fluorescent_lights/mounted_fluorescent_lights_1k.gltf",
            at,
            Quat::IDENTITY,
            Vec3::ONE,
        );
        push_box(
            scene,
            at - Vec3::new(0.0, 0.03, 0.0),
            Quat::IDENTITY,
            Vec3::new(0.44, 0.012, 0.02),
            [0.97, 0.98, 1.0, 1.0],
            0.35,
            0.0,
            [0.9, 0.92, 0.97],
        );
    }

    if floor == 0 {
        push_ground_props(scene, height_m);
    } else {
        push_upper_props(scene, height_m);
    }
}

fn push_ground_props(scene: &mut RenderScene, floor_y_m: f64) {
    push_prop(
        scene,
        "korean_fire_extinguisher_01/korean_fire_extinguisher_01_1k.gltf",
        Vec3::new(SHAFT_X_M - 2.15, floor_y_m, -0.98),
        Quat::from_rotation_y(0.3),
        Vec3::ONE,
    );
    push_prop(
        scene,
        "power_box_01/power_box_01_1k.gltf",
        Vec3::new(SHAFT_X_M - 3.2, floor_y_m + 1.3, LOBBY_WALL_Z_M - 0.04),
        Quat::IDENTITY,
        Vec3::ONE,
    );
    push_prop(
        scene,
        "WetFloorSign_01/WetFloorSign_01_1k.gltf",
        Vec3::new(SHAFT_X_M - 5.9, floor_y_m, 0.85),
        Quat::from_rotation_y(0.5),
        Vec3::ONE,
    );
    push_prop(
        scene,
        "hand_truck/hand_truck_1k.gltf",
        Vec3::new(SHAFT_X_M - 6.3, floor_y_m, -0.75),
        Quat::from_rotation_y(-0.4),
        Vec3::ONE,
    );
}

fn push_upper_props(scene: &mut RenderScene, floor_y_m: f64) {
    push_prop(
        scene,
        "steel_frame_shelves_02/steel_frame_shelves_02_1k.gltf",
        Vec3::new(SHAFT_X_M - 3.6, floor_y_m, -0.92),
        Quat::IDENTITY,
        Vec3::new(1.0, 0.72, 1.0),
    );
    for (level, y) in [0.0_f64, 0.54, 1.08].iter().enumerate() {
        push_prop(
            scene,
            CARDBOARD_BOX,
            Vec3::new(SHAFT_X_M - 3.6, floor_y_m + 0.03 + y * 0.72, -0.92),
            Quat::from_rotation_y(0.1 * level as f64),
            Vec3::new(0.8, 0.8, 0.8),
        );
    }
    // Goods waiting for the robot's next run, stacked at the far end.
    for (x, z, y, yaw) in [
        (-6.25, -0.75, 0.0, 0.1),
        (-6.25, -0.75, 0.34, -0.2),
        (-6.25, -0.2, 0.0, 0.05),
        (-5.7, -0.8, 0.0, 0.3),
    ] {
        push_prop(
            scene,
            CARDBOARD_BOX,
            Vec3::new(SHAFT_X_M + x, floor_y_m + y, z),
            Quat::from_rotation_y(yaw),
            Vec3::ONE,
        );
    }
    push_prop(
        scene,
        "korean_fire_extinguisher_01/korean_fire_extinguisher_01_1k.gltf",
        Vec3::new(SHAFT_X_M - 1.6, floor_y_m, -0.98),
        Quat::from_rotation_y(-0.2),
        Vec3::ONE,
    );
}

fn nearest_floor(car_y_m: f64) -> usize {
    FLOOR_HEIGHTS_M
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (car_y_m - *a)
                .abs()
                .partial_cmp(&(car_y_m - *b).abs())
                .expect("finite")
        })
        .map_or(0, |(index, _)| index)
}

/// The car: platform, cab walls in brushed steel with a handrail, a lit
/// ceiling, the operating panel, the door leaves, and the crosshead the ropes
/// hang from. The side facing the camera is left open, as the shaft's is.
fn push_car(scene: &mut RenderScene, frame: &Frame) {
    const PANEL: [f32; 4] = [0.66, 0.68, 0.72, 1.0];
    const TRIM: [f32; 4] = [0.30, 0.31, 0.34, 1.0];
    const DOOR: [f32; 4] = [0.74, 0.77, 0.82, 1.0];
    let car_y = frame.car_y_m;
    let floor = car_y + CAR_HALF_M.y;
    let wall_half_y = 0.5 * CAB_HEIGHT_M;

    // Platform: the collider's slab, with a dark rubber floor on top.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M, car_y, 0.0),
        CAR_HALF_M,
        [0.30, 0.31, 0.33, 1.0],
        0.6,
        0.5,
    );
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M, floor + 0.002, 0.0),
        Vec3::new(CAR_HALF_M.x - 0.02, 0.002, CAR_HALF_M.z - 0.02),
        [0.12, 0.12, 0.13, 1.0],
        0.9,
        0.0,
    );
    // Sill in brushed aluminium along the doorway.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M - CAR_HALF_M.x + 0.04, floor + 0.004, 0.0),
        Vec3::new(0.04, 0.004, CAR_HALF_M.z),
        [0.80, 0.82, 0.85, 1.0],
        0.3,
        0.8,
    );
    // Far and back walls, with panel seams.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + CAR_HALF_M.x - 0.02, floor + wall_half_y, 0.0),
        Vec3::new(0.02, wall_half_y, CAR_HALF_M.z),
        PANEL,
        0.3,
        0.75,
    );
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + 0.04, floor + wall_half_y, -CAR_HALF_M.z + 0.02),
        Vec3::new(CAR_HALF_M.x - 0.04, wall_half_y, 0.02),
        PANEL,
        0.3,
        0.75,
    );
    for z in [-0.28, 0.28] {
        push_plain(
            scene,
            Vec3::new(SHAFT_X_M + CAR_HALF_M.x - 0.041, floor + wall_half_y, z),
            Vec3::new(0.001, wall_half_y - 0.02, 0.004),
            TRIM,
            0.5,
            0.4,
        );
    }
    push_cylinder(
        scene,
        Vec3::new(SHAFT_X_M + 0.05, floor + 0.9, -CAR_HALF_M.z + 0.09),
        Vec3::X,
        0.018,
        1.4,
        [0.82, 0.84, 0.88, 1.0],
        0.2,
        0.9,
        [0.0; 3],
    );
    // Ceiling: a canopy frame and a lit panel under it.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M, floor + CAB_HEIGHT_M + 0.02, -0.2),
        Vec3::new(CAR_HALF_M.x, 0.02, CAR_HALF_M.z - 0.2),
        [0.40, 0.41, 0.44, 1.0],
        0.5,
        0.4,
    );
    push_box(
        scene,
        Vec3::new(SHAFT_X_M + 0.05, floor + CAB_HEIGHT_M - 0.01, -0.25),
        Quat::IDENTITY,
        Vec3::new(0.5, 0.01, 0.4),
        [0.95, 0.96, 1.0, 1.0],
        0.3,
        0.0,
        [0.85, 0.86, 0.90],
    );
    // Crosshead the ropes hang from.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M, floor + CAB_HEIGHT_M + 0.12, 0.0),
        Vec3::new(0.08, 0.08, CAR_HALF_M.z + 0.05),
        [0.85, 0.66, 0.12, 1.0],
        0.5,
        0.3,
    );
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M - 0.14, floor + CAB_HEIGHT_M + 0.16, -0.35),
        Vec3::new(0.06, 0.04, 0.1),
        [0.25, 0.26, 0.28, 1.0],
        0.5,
        0.4,
    );
    // Car operating panel on the far wall: a destination button per floor,
    // lit for the floor the robot chose once it is aboard.
    let cop = Vec3::new(SHAFT_X_M + CAR_HALF_M.x - 0.045, floor + 1.15, 0.55);
    push_plain(
        scene,
        cop,
        Vec3::new(0.006, 0.28, 0.1),
        [0.20, 0.21, 0.23, 1.0],
        0.3,
        0.6,
    );
    let destination = matches!(frame.phase, Phase::Ride);
    for (index, dy) in [(0_usize, -0.08), (1, 0.08)] {
        push_lamp(
            scene,
            cop + Vec3::new(-0.008, dy, 0.0),
            Vec3::new(0.004, 0.035, 0.035),
            (destination && index == 1).then_some(LAMP_AMBER),
        );
    }
    // Door leaves: the kinematic bodies' shapes, in brushed steel with a
    // rubber safety edge on the meeting stile.
    let doorway_z_m = CAR_HALF_M.z - DOOR_HALF_M.z;
    for sign in [-1.0, 1.0] {
        let center = Vec3::new(
            SHAFT_X_M - CAR_HALF_M.x,
            car_y + DOOR_HALF_M.y,
            sign * (doorway_z_m + frame.door_opening_m),
        );
        push_plain(scene, center, DOOR_HALF_M, DOOR, 0.28, 0.85);
        push_plain(
            scene,
            center - Vec3::new(0.0, 0.0, sign * (DOOR_HALF_M.z - 0.01)),
            Vec3::new(DOOR_HALF_M.x + 0.004, DOOR_HALF_M.y - 0.05, 0.012),
            [0.10, 0.10, 0.11, 1.0],
            0.8,
            0.0,
        );
    }
}

/// A delivery robot drawn on the robot's collision box, turned with it: drive
/// wheels and casters, a bumper with an LED status band, a white shell with a
/// parcel on top, a LiDAR, and a tilted face screen at the front (+x).
fn push_robot(scene: &mut RenderScene, frame: &Frame) {
    const SHELL: [f32; 4] = [0.92, 0.93, 0.95, 1.0];
    const BASE: [f32; 4] = [0.22, 0.24, 0.28, 1.0];
    const RUBBER: [f32; 4] = [0.07, 0.07, 0.08, 1.0];
    const ACCENT: [f32; 4] = [0.10, 0.55, 0.78, 1.0];
    let (center, rotation) = (frame.robot, frame.robot_rotation);
    let at = |local: Vec3| center + rotation * local;
    let half = ROBOT_HALF_M;
    let status = match frame.phase {
        Phase::Press | Phase::WaitForDoors => LAMP_AMBER,
        Phase::Done => LAMP_GREEN,
        _ => [0.20, 0.75, 1.0],
    };

    push_robot_wheels(scene, frame);
    // Base, bumper and status band.
    push_box(
        scene,
        at(Vec3::new(0.0, -half.y + 0.14, 0.0)),
        rotation,
        Vec3::new(half.x - 0.02, 0.07, half.z - 0.05),
        BASE,
        0.6,
        0.2,
        [0.0; 3],
    );
    push_box(
        scene,
        at(Vec3::new(0.0, -half.y + 0.11, 0.0)),
        rotation,
        Vec3::new(half.x, 0.035, half.z - 0.035),
        RUBBER,
        0.85,
        0.0,
        [0.0; 3],
    );
    push_box(
        scene,
        at(Vec3::new(0.0, -half.y + 0.2, 0.0)),
        rotation,
        Vec3::new(half.x - 0.015, 0.008, half.z - 0.045),
        [status[0], status[1], status[2], 1.0],
        0.3,
        0.0,
        status,
    );
    // Shell and compartment door.
    let shell_bottom = -half.y + 0.21;
    let shell_half_y = 0.5 * (half.y - 0.02 - shell_bottom);
    push_box(
        scene,
        at(Vec3::new(-0.02, shell_bottom + shell_half_y, 0.0)),
        rotation,
        Vec3::new(half.x - 0.05, shell_half_y, half.z - 0.05),
        SHELL,
        0.35,
        0.05,
        [0.0; 3],
    );
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            at(Vec3::new(
                -0.04,
                shell_bottom + shell_half_y,
                side * (half.z - 0.049),
            )),
            rotation,
            Vec3::new(half.x - 0.14, shell_half_y - 0.04, 0.002),
            [0.80, 0.82, 0.85, 1.0],
            0.4,
            0.1,
            [0.0; 3],
        );
        push_box(
            scene,
            at(Vec3::new(
                -0.04,
                shell_bottom + shell_half_y * 1.6,
                side * (half.z - 0.046),
            )),
            rotation,
            Vec3::new(half.x - 0.14, 0.012, 0.002),
            ACCENT,
            0.5,
            0.0,
            [0.0; 3],
        );
    }
    // Face: a screen leaned back at the front, two glowing eyes on it.
    let lean = Quat::from_rotation_z(0.35);
    let face = Vec3::new(half.x - 0.08, half.y - 0.1, 0.0);
    push_box(
        scene,
        at(face),
        rotation * lean,
        Vec3::new(0.03, 0.09, 0.2),
        [0.08, 0.09, 0.11, 1.0],
        0.2,
        0.3,
        [0.0; 3],
    );
    for side in [-1.0, 1.0] {
        push_box(
            scene,
            at(face + lean * Vec3::new(0.031, 0.01, side * 0.07)),
            rotation * lean,
            Vec3::new(0.002, 0.025, 0.02),
            [status[0], status[1], status[2], 1.0],
            0.3,
            0.0,
            status,
        );
    }
    // LiDAR on a short mast at the back, and the parcel being delivered.
    push_cylinder(
        scene,
        at(Vec3::new(-half.x + 0.1, half.y + 0.02, 0.0)),
        rotation * Vec3::Y,
        0.05,
        0.05,
        [0.12, 0.12, 0.14, 1.0],
        0.4,
        0.3,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        at(Vec3::new(-half.x + 0.1, half.y + 0.047, 0.0)),
        rotation * Vec3::Y,
        0.052,
        0.006,
        [0.20, 0.75, 1.0, 1.0],
        0.3,
        0.0,
        [0.20, 0.75, 1.0],
    );
    push_prop(
        scene,
        CARDBOARD_BOX,
        at(Vec3::new(-0.02, half.y - 0.02, 0.0)),
        rotation * Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
        Vec3::new(0.5, 0.5, 0.5),
    );
}

/// The traction machine, hoist ropes, counterweight and pit buffer.
fn push_hoist(scene: &mut RenderScene, car_y_m: f64) {
    const ROPE: [f32; 4] = [0.20, 0.20, 0.21, 1.0];
    const MACHINE: [f32; 4] = [0.18, 0.36, 0.30, 1.0];
    let bottom_y_m = -0.7;
    // Machine: bedplate, motor, and the traction sheave the ropes wrap.
    let top = SHAFT_TOP_Y_M - 0.35;
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + 0.2, top - 0.12, -0.35),
        Vec3::new(0.7, 0.04, 0.45),
        [0.24, 0.25, 0.27, 1.0],
        0.6,
        0.4,
    );
    push_cylinder(
        scene,
        Vec3::new(SHAFT_X_M + 0.55, top + 0.1, -0.35),
        Vec3::Z,
        0.17,
        0.42,
        MACHINE,
        0.45,
        0.2,
        [0.0; 3],
    );
    push_cylinder(
        scene,
        Vec3::new(SHAFT_X_M + 0.1, top + 0.08, -0.35),
        Vec3::Z,
        0.24,
        0.14,
        [0.55, 0.57, 0.60, 1.0],
        0.3,
        0.8,
        [0.0; 3],
    );
    // Hoist ropes from the car's crosshead up to the sheave, and from the
    // sheave's far side down to the counterweight. The counterweight moves
    // opposite the car over the same travel.
    let crosshead_y_m = car_y_m + CAR_HALF_M.y + CAB_HEIGHT_M + 0.18;
    let counterweight_y_m = FLOOR_HEIGHTS_M[1] + 1.4 - (car_y_m - FLOOR_HEIGHTS_M[0]);
    let rope_top_y_m = top + 0.08;
    for dz in [-0.06, -0.02, 0.02, 0.06] {
        let car_rope = rope_top_y_m - crosshead_y_m;
        push_cylinder(
            scene,
            Vec3::new(SHAFT_X_M - 0.14, crosshead_y_m + car_rope * 0.5, -0.35 + dz),
            Vec3::Y,
            0.008,
            car_rope,
            ROPE,
            0.6,
            0.3,
            [0.0; 3],
        );
        let weight_rope = rope_top_y_m - (counterweight_y_m + 0.7);
        push_cylinder(
            scene,
            Vec3::new(
                SHAFT_X_M + 0.5 + dz,
                counterweight_y_m + 0.7 + weight_rope * 0.5,
                -0.95,
            ),
            Vec3::Y,
            0.008,
            weight_rope,
            ROPE,
            0.6,
            0.3,
            [0.0; 3],
        );
    }
    // Counterweight: a frame of stacked filler weights between its rails.
    push_plain(
        scene,
        Vec3::new(SHAFT_X_M + 0.5, counterweight_y_m, -0.97),
        Vec3::new(0.24, 0.7, 0.06),
        [0.22, 0.23, 0.25, 1.0],
        0.5,
        0.5,
    );
    for block in 0..7 {
        push_plain(
            scene,
            Vec3::new(
                SHAFT_X_M + 0.5,
                counterweight_y_m - 0.6 + f64::from(block) * 0.2,
                -0.95,
            ),
            Vec3::new(0.21, 0.085, 0.065),
            [0.40, 0.42, 0.45, 1.0],
            0.7,
            0.3,
        );
    }
    // Pit buffer under the car.
    push_cylinder(
        scene,
        Vec3::new(SHAFT_X_M, bottom_y_m + 0.18, 0.0),
        Vec3::Y,
        0.09,
        0.36,
        [0.85, 0.70, 0.12, 1.0],
        0.5,
        0.1,
        [0.0; 3],
    );
}

/// The landing's position indicator and hall call panel.
fn push_hall_fixtures(scene: &mut RenderScene, floor: usize, height_m: f64, frame: &Frame) {
    // Position indicator above the hall panel: a lamp per floor, lit for the
    // floor the car is nearest, and a direction lamp while it travels.
    let indicator = Vec3::new(BUTTON_CENTER_M.x, height_m + 1.95, LOBBY_WALL_Z_M + 0.012);
    push_plain(
        scene,
        indicator,
        Vec3::new(0.16, 0.16, 0.01),
        [0.08, 0.08, 0.09, 1.0],
        0.3,
        0.2,
    );
    let car_floor = nearest_floor(frame.car_y_m);
    for (index, dx) in [(0_usize, -0.07), (1, 0.07)] {
        push_lamp(
            scene,
            indicator + Vec3::new(dx, 0.05, 0.012),
            Vec3::new(0.045, 0.045, 0.004),
            (car_floor == index).then_some(LAMP_GREEN),
        );
    }
    let travelling = frame.car_velocity_m_s.abs() > 0.02;
    push_lamp(
        scene,
        indicator + Vec3::new(0.0, -0.08, 0.012),
        Vec3::new(0.10, 0.025, 0.004),
        travelling.then_some(LAMP_AMBER),
    );

    // Hall panel: only the lobby floor's is the physical button; the upper
    // landing gets a matching panel with no call on it.
    let panel = if floor == 0 {
        BUTTON_CENTER_M - BUTTON_NORMAL * 0.02
    } else {
        Vec3::new(
            BUTTON_CENTER_M.x,
            height_m + BUTTON_CENTER_M.y,
            LOBBY_WALL_Z_M + 0.03,
        )
    };
    push_plain(
        scene,
        panel,
        Vec3::new(0.14, 0.24, 0.012),
        [0.62, 0.64, 0.68, 1.0],
        0.25,
        0.85,
    );
    if floor == 0 {
        let lit = matches!(frame.phase, Phase::Press | Phase::WaitForDoors);
        // Drawn larger than the collider on purpose: the physical button is
        // 10 cm across and would be three pixels at this scale. The collider,
        // and therefore the press, is unchanged.
        push_cylinder(
            scene,
            BUTTON_CENTER_M - BUTTON_NORMAL * BUTTON_HALF_M.z,
            BUTTON_NORMAL,
            0.085,
            2.0 * BUTTON_HALF_M.z,
            if lit {
                [1.0, 0.70, 0.20, 1.0]
            } else {
                [0.30, 0.31, 0.34, 1.0]
            },
            0.3,
            0.2,
            if lit { LAMP_AMBER } else { [0.0; 3] },
        );
    } else {
        push_cylinder(
            scene,
            panel + Vec3::new(0.0, 0.0, 0.02),
            Vec3::Z,
            0.06,
            0.02,
            [0.30, 0.31, 0.34, 1.0],
            0.3,
            0.2,
            [0.0; 3],
        );
    }
}

/// The robot's drive wheels and casters.
fn push_robot_wheels(scene: &mut RenderScene, frame: &Frame) {
    const RUBBER: [f32; 4] = [0.07, 0.07, 0.08, 1.0];
    let (center, rotation) = (frame.robot, frame.robot_rotation);
    let at = |local: Vec3| center + rotation * local;
    let half = ROBOT_HALF_M;
    // Wheels: two drive wheels mid-length, four casters at the corners.
    let axle = rotation * Vec3::Z;
    for side in [-1.0, 1.0] {
        push_cylinder(
            scene,
            at(Vec3::new(0.0, -half.y + 0.075, side * (half.z - 0.03))),
            axle,
            0.075,
            0.05,
            RUBBER,
            0.9,
            0.0,
            [0.0; 3],
        );
        push_cylinder(
            scene,
            at(Vec3::new(0.0, -half.y + 0.075, side * (half.z - 0.001))),
            axle,
            0.035,
            0.01,
            [0.60, 0.62, 0.66, 1.0],
            0.3,
            0.8,
            [0.0; 3],
        );
        for end in [-1.0, 1.0] {
            push_cylinder(
                scene,
                at(Vec3::new(
                    end * (half.x - 0.07),
                    -half.y + 0.035,
                    side * (half.z - 0.08),
                )),
                axle,
                0.035,
                0.03,
                RUBBER,
                0.9,
                0.0,
                [0.0; 3],
            );
        }
    }
}
