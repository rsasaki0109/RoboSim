//! Interior dressing and obstacle geometry shared by the Go2 indoor examples.
//!
//! [`Interior::decorate`] replaces each static scene object's plain render box
//! with a detailed, textured model *inside the same collision envelope*: what the
//! robot, its Mid-360, and the planner see is exactly what is drawn, only with
//! more detail. Floors and a rug are flat overlays. Textures are generated
//! procedurally and deterministically, so captures replay byte for byte.
//!
//! [`static_obstacles`] reads the scene's fixed colliders, so clearance checks
//! follow the scene file instead of a hand-copied list.

use std::sync::Arc;

use rne_ecs::{Name, World};
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_render::{
    ImageFrame, PbrMaterial, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_robot::Link;
use rne_world::{world_transform_of, Transform3 as WorldTransform3};

/// A fixed scene collider seen from above.
#[derive(Clone, Debug, PartialEq)]
pub struct StaticObject {
    /// Scene object name.
    pub name: String,
    /// World pose of the collider.
    pub pose: WorldTransform3,
    /// Collider shape.
    pub shape: ColliderShape,
}

impl StaticObject {
    /// Planar distance from `[x, z]` to the object's footprint, in meters.
    pub fn clearance_m(&self, position: [f64; 2]) -> f64 {
        let local = self.pose.rotation.inverse()
            * Vec3::new(
                position[0] - self.pose.translation.x,
                0.0,
                position[1] - self.pose.translation.z,
            );
        match self.shape {
            ColliderShape::Cuboid { half_extents_m } => {
                let dx = (local.x.abs() - half_extents_m.x).max(0.0);
                let dz = (local.z.abs() - half_extents_m.z).max(0.0);
                dx.hypot(dz)
            }
            ColliderShape::Capsule { radius_m, .. } | ColliderShape::Sphere { radius_m } => {
                (local.x.hypot(local.z) - radius_m).max(0.0)
            }
            _ => f64::MAX,
        }
    }
}

/// Colliders whose top is at most this high are the floor, not obstacles.
const FLOOR_TOP_M: f64 = 0.02;

/// Every fixed, non-robot collider standing above the floor, sorted by name.
pub fn static_obstacles(world: &World) -> Vec<StaticObject> {
    let mut objects: Vec<StaticObject> = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let id = entity_ref.id();
            let name = world.get::<Name>(id)?;
            let collider = world.get::<Collider>(id)?;
            let body = world.get::<RigidBody>(id)?;
            if body.body_type != RigidBodyType::Fixed
                || world.get::<Link>(id).is_some()
                || matches!(collider.shape, ColliderShape::Plane { .. })
            {
                return None;
            }
            let pose = world_transform_of(world, id).mul_transform(&WorldTransform3 {
                translation: collider.local_offset.translation,
                rotation: collider.local_offset.rotation,
                scale: Vec3::ONE,
            });
            let top_m = match collider.shape {
                ColliderShape::Cuboid { half_extents_m } => pose.translation.y + half_extents_m.y,
                _ => f64::MAX,
            };
            (top_m > FLOOR_TOP_M).then(|| StaticObject {
                name: name.0.clone(),
                pose,
                shape: collider.shape.clone(),
            })
        })
        .collect();
    objects.sort_by(|a, b| a.name.cmp(&b.name));
    objects
}

/// Planar distance from `[x, z]` to the nearest static obstacle, in meters.
pub fn clearance_m(obstacles: &[StaticObject], position: [f64; 2]) -> f64 {
    obstacles
        .iter()
        .map(|object| object.clearance_m(position))
        .fold(f64::MAX, f64::min)
}

/// A floor finish laid over a rectangle of the ground, in world `x`/`z`.
#[derive(Clone, Copy, Debug)]
pub struct FloorPatch {
    /// Minimum `[x, z]` corner.
    pub min: [f64; 2],
    /// Maximum `[x, z]` corner.
    pub max: [f64; 2],
    /// Finish.
    pub finish: Finish,
}

/// Floor finishes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Finish {
    /// Oak planks.
    Wood,
    /// Large grey tiles.
    Tile,
    /// Patterned rug.
    Rug,
}

/// The two-room layout's floors and rug.
pub fn two_room_floors() -> Vec<FloorPatch> {
    vec![
        FloorPatch {
            min: [-2.4, -2.4],
            max: [2.35, 2.4],
            finish: Finish::Wood,
        },
        FloorPatch {
            min: [2.45, -2.4],
            max: [7.0, 2.4],
            finish: Finish::Tile,
        },
        FloorPatch {
            min: [-1.1, 0.35],
            max: [1.3, 1.45],
            finish: Finish::Rug,
        },
    ]
}

/// Procedural textures, generated once.
pub struct Interior {
    wood_floor: Arc<ImageFrame>,
    tile_floor: Arc<ImageFrame>,
    rug: Arc<ImageFrame>,
    plaster: Arc<ImageFrame>,
    oak: Arc<ImageFrame>,
    books: Arc<ImageFrame>,
    crate_wood: Arc<ImageFrame>,
    cardboard: Arc<ImageFrame>,
    stone: Arc<ImageFrame>,
    fabric: Arc<ImageFrame>,
    cabinet_front: Arc<ImageFrame>,
    floors: Vec<FloorPatch>,
}

impl std::fmt::Debug for Interior {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Interior")
            .field("floors", &self.floors)
            .finish_non_exhaustive()
    }
}

/// Rendering options.
#[derive(Clone, Copy, Debug, Default)]
pub struct DecorOptions {
    /// Draw walls no taller than this, for cutaway views.
    pub wall_height_cap_m: Option<f64>,
}

impl Interior {
    /// Generates the textures for `floors`.
    pub fn new(floors: Vec<FloorPatch>) -> Self {
        Self {
            wood_floor: Arc::new(wood_floor_texture()),
            tile_floor: Arc::new(tile_texture()),
            rug: Arc::new(rug_texture()),
            plaster: Arc::new(plaster_texture()),
            oak: Arc::new(oak_texture()),
            books: Arc::new(books_texture()),
            crate_wood: Arc::new(crate_texture()),
            cardboard: Arc::new(cardboard_texture()),
            stone: Arc::new(stone_texture()),
            fabric: Arc::new(fabric_texture()),
            cabinet_front: Arc::new(cabinet_front_texture()),
            floors,
        }
    }

    /// Replaces the static objects' plain boxes (and the ground plane) in `scene`
    /// with detailed models, and lays the floors.
    pub fn decorate(&self, scene: &mut RenderScene, world: &World, options: DecorOptions) {
        let objects = static_obstacles(world);
        scene.items.retain(|item| {
            let plain = matches!(
                item.shape,
                VisualShape::Box { .. } | VisualShape::Cylinder { .. } | VisualShape::Sphere { .. }
            );
            if !plain {
                return true;
            }
            if matches!(item.shape, VisualShape::Box { size_m } if size_m.x >= 20.0) {
                return false;
            }
            !objects.iter().any(|object| {
                (object.pose.translation - item.transform.translation).length() < 1.0e-3
            })
        });
        // A dark surround beyond the rooms.
        let mut surround = MeshBuilder::default();
        surround.add_box(
            Vec3::new(2.3, -0.012, 0.0),
            Vec3::new(8.0, 0.01, 5.0),
            Quat::IDENTITY,
            1.0,
        );
        push(
            scene,
            "decor://surround",
            surround,
            None,
            [0.05, 0.06, 0.07, 1.0],
            0.9,
        );
        for (index, patch) in self.floors.iter().enumerate() {
            let (texture, tile_m, y, roughness) = match patch.finish {
                Finish::Wood => (&self.wood_floor, 1.6, 0.0005, 0.55),
                Finish::Tile => (&self.tile_floor, 1.2, 0.0005, 0.35),
                Finish::Rug => (&self.rug, 0.0, 0.003, 0.95),
            };
            let mut mesh = MeshBuilder::default();
            mesh.add_floor(patch.min, patch.max, y, tile_m);
            push(
                scene,
                &format!("decor://floor-{index}"),
                mesh,
                Some(texture),
                [1.0, 1.0, 1.0, 1.0],
                roughness,
            );
        }
        for object in &objects {
            self.decorate_object(scene, object, options);
        }
    }

    fn decorate_object(
        &self,
        scene: &mut RenderScene,
        object: &StaticObject,
        options: DecorOptions,
    ) {
        let name = object.name.as_str();
        let key = format!("decor://{name}");
        let center = object.pose.translation;
        let rotation = object.pose.rotation;
        match object.shape {
            ColliderShape::Cuboid { half_extents_m } => {
                let half = half_extents_m;
                if name.starts_with("wall_") || name.starts_with("partition_") {
                    self.wall(scene, &key, center, half, rotation, options);
                } else if name == "table" {
                    self.island(scene, &key, center, half, rotation);
                } else if name == "shelf" {
                    self.bookshelf(scene, &key, center, half, rotation);
                } else if name == "counter" {
                    self.counter(scene, &key, center, half, rotation);
                } else if name.starts_with("sofa_") {
                    self.sofa_part(scene, &key, center, half, rotation);
                } else if name.starts_with("crate_") {
                    let cardboard = name.ends_with("_b");
                    let mut mesh = MeshBuilder::default();
                    mesh.add_box(center, half, rotation, if cardboard { 0.6 } else { 0.5 });
                    let texture = if cardboard {
                        &self.cardboard
                    } else {
                        &self.crate_wood
                    };
                    push(scene, &key, mesh, Some(texture), [1.0; 4], 0.8);
                } else {
                    let mut mesh = MeshBuilder::default();
                    mesh.add_box(center, half, rotation, 1.0);
                    push(scene, &key, mesh, None, [0.6, 0.6, 0.62, 1.0], 0.7);
                }
            }
            ColliderShape::Capsule {
                half_height_m,
                radius_m,
            } if name.starts_with("plant_") => {
                self.plant(scene, &key, center, half_height_m + radius_m, radius_m);
            }
            _ => {}
        }
    }

    /// Plaster wall with a dark baseboard and a light top cap.
    fn wall(
        &self,
        scene: &mut RenderScene,
        key: &str,
        center: Vec3,
        half: Vec3,
        rotation: Quat,
        options: DecorOptions,
    ) {
        let full_height = 2.0 * half.y;
        let height = options
            .wall_height_cap_m
            .map_or(full_height, |cap| cap.min(full_height));
        let bottom = center.y - half.y;
        let body_center = Vec3::new(center.x, bottom + height * 0.5, center.z);
        let mut body = MeshBuilder::default();
        body.add_box(
            body_center,
            Vec3::new(half.x, height * 0.5, half.z),
            rotation,
            1.2,
        );
        push(
            scene,
            key,
            body,
            Some(&self.plaster),
            [0.93, 0.92, 0.89, 1.0],
            0.85,
        );
        // Baseboard: a 9 cm band flush with both faces.
        let board = 0.09_f64.min(height);
        let mut base = MeshBuilder::default();
        base.add_box(
            Vec3::new(center.x, bottom + board * 0.5, center.z),
            Vec3::new(half.x + 0.002, board * 0.5, half.z + 0.002),
            rotation,
            0.5,
        );
        push(
            scene,
            &format!("{key}-base"),
            base,
            Some(&self.oak),
            [0.45, 0.33, 0.24, 1.0],
            0.5,
        );
        if height > 0.2 {
            let mut cap = MeshBuilder::default();
            cap.add_box(
                Vec3::new(center.x, bottom + height - 0.015, center.z),
                Vec3::new(half.x + 0.002, 0.015, half.z + 0.002),
                rotation,
                1.0,
            );
            push(
                scene,
                &format!("{key}-cap"),
                cap,
                None,
                [0.97, 0.97, 0.95, 1.0],
                0.4,
            );
        }
    }

    /// Kitchen island: cabinet body with door and drawer fronts under a stone top.
    fn island(&self, scene: &mut RenderScene, key: &str, center: Vec3, half: Vec3, rotation: Quat) {
        let top_t = 0.04;
        let body_half = Vec3::new(half.x - 0.03, half.y - top_t * 0.5, half.z - 0.03);
        let body_center = center - Vec3::Y * (top_t * 0.5);
        let mut body = MeshBuilder::default();
        body.add_box_faces_uv(body_center, body_half, rotation);
        push(scene, key, body, Some(&self.cabinet_front), [1.0; 4], 0.6);
        let mut top = MeshBuilder::default();
        top.add_box(
            center + Vec3::Y * (half.y - top_t * 0.5),
            Vec3::new(half.x, top_t * 0.5, half.z),
            rotation,
            1.0,
        );
        push(
            scene,
            &format!("{key}-top"),
            top,
            Some(&self.stone),
            [1.0; 4],
            0.25,
        );
        // Plinth, recessed.
        let mut plinth = MeshBuilder::default();
        plinth.add_box(
            center - Vec3::Y * (half.y - 0.04),
            Vec3::new(half.x - 0.07, 0.04, half.z - 0.07),
            rotation,
            1.0,
        );
        push(
            scene,
            &format!("{key}-plinth"),
            plinth,
            None,
            [0.12, 0.12, 0.13, 1.0],
            0.7,
        );
    }

    /// Bookshelf: oak carcass, shelves of books on the front.
    fn bookshelf(
        &self,
        scene: &mut RenderScene,
        key: &str,
        center: Vec3,
        half: Vec3,
        rotation: Quat,
    ) {
        let mut carcass = MeshBuilder::default();
        carcass.add_box(center, half, rotation, 0.8);
        push(
            scene,
            key,
            carcass,
            Some(&self.oak),
            [0.85, 0.72, 0.58, 1.0],
            0.55,
        );
        // The book faces sit 2 mm proud of the long faces, inside a 3 cm frame.
        for side in [-1.0, 1.0] {
            let mut face = MeshBuilder::default();
            let normal = rotation * Vec3::new(0.0, 0.0, side);
            let right = rotation * Vec3::X;
            face.add_textured_quad(
                center + normal * (half.z + 0.002),
                right * (half.x - 0.03),
                Vec3::Y * (half.y - 0.03),
                normal,
            );
            push(
                scene,
                &format!("{key}-books{side}"),
                face,
                Some(&self.books),
                [1.0; 4],
                0.7,
            );
        }
    }

    /// Reception counter: stone top over a slatted oak front.
    fn counter(
        &self,
        scene: &mut RenderScene,
        key: &str,
        center: Vec3,
        half: Vec3,
        rotation: Quat,
    ) {
        let top_t = 0.05;
        let mut body = MeshBuilder::default();
        body.add_box(
            center - Vec3::Y * (top_t * 0.5),
            Vec3::new(half.x - 0.02, half.y - top_t * 0.5, half.z - 0.02),
            rotation,
            0.5,
        );
        push(
            scene,
            key,
            body,
            Some(&self.crate_wood),
            [0.95, 0.85, 0.72, 1.0],
            0.6,
        );
        let mut top = MeshBuilder::default();
        top.add_box(
            center + Vec3::Y * (half.y - top_t * 0.5),
            Vec3::new(half.x, top_t * 0.5, half.z),
            rotation,
            1.0,
        );
        push(
            scene,
            &format!("{key}-top"),
            top,
            Some(&self.stone),
            [0.9, 0.9, 0.92, 1.0],
            0.25,
        );
    }

    /// Sofa seat, back, and arms: upholstered boxes with a seam.
    fn sofa_part(
        &self,
        scene: &mut RenderScene,
        key: &str,
        center: Vec3,
        half: Vec3,
        rotation: Quat,
    ) {
        let mut mesh = MeshBuilder::default();
        mesh.add_box(
            center,
            Vec3::new(half.x - 0.01, half.y - 0.01, half.z - 0.01),
            rotation,
            0.35,
        );
        push(scene, key, mesh, Some(&self.fabric), [1.0; 4], 0.95);
        // Legs' shadow line: a dark band at the bottom.
        let mut band = MeshBuilder::default();
        band.add_box(
            center - Vec3::Y * (half.y - 0.03),
            Vec3::new(half.x, 0.03, half.z),
            rotation,
            1.0,
        );
        push(
            scene,
            &format!("{key}-band"),
            band,
            None,
            [0.18, 0.15, 0.13, 1.0],
            0.7,
        );
    }

    /// Potted plant inside its capsule envelope: pot, soil, and leaf clusters.
    fn plant(
        &self,
        scene: &mut RenderScene,
        key: &str,
        center: Vec3,
        half_span_m: f64,
        radius_m: f64,
    ) {
        let bottom = center.y - half_span_m;
        let pot_h = 0.38;
        let pot_r = radius_m * 0.75;
        push_shape(
            scene,
            Vec3::new(center.x, bottom + pot_h * 0.5, center.z),
            Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            Vec3::new(pot_r * 2.0, pot_r * 2.0, pot_h),
            VisualShape::Cylinder {
                radius_m: 0.5,
                length_m: 1.0,
            },
            [0.62, 0.34, 0.22, 1.0],
            0.8,
        );
        let leaves = [[0.06, 0.62, 0.35], [0.30, 0.52, 0.34], [0.20, 0.44, 0.24]];
        for index in 0..9_u64 {
            let h = hash(index * 31 + key.len() as u64);
            let angle = (h % 628) as f64 / 100.0;
            let reach = radius_m * (0.25 + 0.45 * ((h >> 12) % 100) as f64 / 100.0);
            let lift = bottom
                + pot_h
                + 0.12
                + (2.0 * half_span_m - pot_h - 0.3) * ((h >> 24) % 100) as f64 / 100.0;
            let size = 0.13 + 0.06 * ((h >> 36) % 100) as f64 / 100.0;
            let color = leaves[(h >> 44) as usize % leaves.len()];
            push_shape(
                scene,
                Vec3::new(
                    center.x + reach * angle.cos(),
                    lift,
                    center.z + reach * angle.sin(),
                ),
                Quat::IDENTITY,
                Vec3::splat(size * 2.0),
                VisualShape::Sphere { radius_m: 0.5 },
                [color[0] as f32, color[1] as f32, color[2] as f32, 1.0],
                0.9,
            );
        }
    }
}

fn push(
    scene: &mut RenderScene,
    key: &str,
    mesh: MeshBuilder,
    texture: Option<&Arc<ImageFrame>>,
    color: [f32; 4],
    roughness: f32,
) {
    if mesh.indices.is_empty() {
        return;
    }
    scene.items.push(RenderSceneItem {
        transform: MathTransform3::IDENTITY,
        shape: VisualShape::Mesh {
            path: key.to_string(),
            scale: Vec3::ONE,
        },
        color_rgba: color,
        mesh: Some(Arc::new(TriangleMesh {
            positions: mesh.positions,
            normals: mesh.normals,
            texcoords: mesh.texcoords,
            indices: mesh.indices,
            skinning: None,
        })),
        base_color_texture: texture.cloned(),
        material: PbrMaterial::new(color, roughness, 0.0, [0.0; 3]),
    });
}

fn push_shape(
    scene: &mut RenderScene,
    translation: Vec3,
    rotation: Quat,
    scale: Vec3,
    shape: VisualShape,
    color: [f32; 4],
    roughness: f32,
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation,
            scale,
        },
        shape,
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::new(color, roughness, 0.0, [0.0; 3]),
    });
}

#[derive(Default)]
struct MeshBuilder {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl MeshBuilder {
    fn vertex(&mut self, position: Vec3, normal: Vec3, uv: [f64; 2]) -> u32 {
        let index = self.positions.len() as u32;
        self.positions
            .push([position.x as f32, position.y as f32, position.z as f32]);
        self.normals
            .push([normal.x as f32, normal.y as f32, normal.z as f32]);
        self.texcoords.push([uv[0] as f32, uv[1] as f32]);
        index
    }

    /// A quad centred at `center` spanning `±right` and `±up`, facing `normal`,
    /// with UVs in world meters divided by `tile_m` (or 0..1 when `tile_m` is 0).
    fn quad(&mut self, center: Vec3, right: Vec3, up: Vec3, normal: Vec3, tile_m: f64) {
        let (u, v) = if tile_m > 0.0 {
            (right.length() * 2.0 / tile_m, up.length() * 2.0 / tile_m)
        } else {
            (1.0, 1.0)
        };
        let corners = [
            (center - right - up, [0.0, v]),
            (center + right - up, [u, v]),
            (center + right + up, [u, 0.0]),
            (center - right + up, [0.0, 0.0]),
        ];
        let base = self.vertex(corners[0].0, normal, corners[0].1);
        for (position, uv) in &corners[1..] {
            self.vertex(*position, normal, *uv);
        }
        // Wind counter-clockwise seen from the normal side.
        let facing = (right.cross(up)).dot(normal) >= 0.0;
        if facing {
            self.indices
                .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        } else {
            self.indices
                .extend([base, base + 2, base + 1, base, base + 3, base + 2]);
        }
    }

    fn add_textured_quad(&mut self, center: Vec3, right: Vec3, up: Vec3, normal: Vec3) {
        self.quad(center, right, up, normal, 0.0);
    }

    /// A box with every face UV-mapped in meters divided by `tile_m`.
    fn add_box(&mut self, center: Vec3, half: Vec3, rotation: Quat, tile_m: f64) {
        for (normal, right, up) in box_faces(half) {
            self.quad(
                center + rotation * normal,
                rotation * right,
                rotation * up,
                rotation * normal.normalize(),
                tile_m,
            );
        }
    }

    /// A box whose side faces each map the whole texture once (fronts), top and
    /// bottom tiled.
    fn add_box_faces_uv(&mut self, center: Vec3, half: Vec3, rotation: Quat) {
        for (normal, right, up) in box_faces(half) {
            let side = normal.y.abs() < 1.0e-9;
            self.quad(
                center + rotation * normal,
                rotation * right,
                rotation * up,
                rotation * normal.normalize(),
                if side { 0.0 } else { 1.0 },
            );
        }
    }

    fn add_floor(&mut self, min: [f64; 2], max: [f64; 2], y: f64, tile_m: f64) {
        let center = Vec3::new((min[0] + max[0]) * 0.5, y, (min[1] + max[1]) * 0.5);
        self.quad(
            center,
            Vec3::X * ((max[0] - min[0]) * 0.5),
            Vec3::Z * (-(max[1] - min[1]) * 0.5),
            Vec3::Y,
            tile_m,
        );
    }
}

/// `(offset to face centre, half right, half up)` for the six faces of a box.
fn box_faces(half: Vec3) -> [(Vec3, Vec3, Vec3); 6] {
    [
        (Vec3::X * half.x, Vec3::Z * -half.z, Vec3::Y * half.y),
        (Vec3::X * -half.x, Vec3::Z * half.z, Vec3::Y * half.y),
        (Vec3::Z * half.z, Vec3::X * half.x, Vec3::Y * half.y),
        (Vec3::Z * -half.z, Vec3::X * -half.x, Vec3::Y * half.y),
        (Vec3::Y * half.y, Vec3::X * half.x, Vec3::Z * -half.z),
        (Vec3::Y * -half.y, Vec3::X * half.x, Vec3::Z * half.z),
    ]
}

fn hash(value: u64) -> u64 {
    let mut x = value ^ 0x9e37_79b9_7f4a_7c15;
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// Smooth value noise in `[0, 1]` at a lattice of `cells` per texture.
fn noise(x: f64, y: f64, seed: u64) -> f64 {
    let (xi, yi) = (x.floor() as i64, y.floor() as i64);
    let (xf, yf) = (x - x.floor(), y - y.floor());
    let corner = |dx: i64, dy: i64| {
        let key = ((xi + dx) as u64).wrapping_mul(73_856_093)
            ^ ((yi + dy) as u64).wrapping_mul(19_349_663)
            ^ seed;
        (hash(key) % 10_000) as f64 / 10_000.0
    };
    let smooth = |t: f64| t * t * (3.0 - 2.0 * t);
    let (sx, sy) = (smooth(xf), smooth(yf));
    let top = corner(0, 0) + (corner(1, 0) - corner(0, 0)) * sx;
    let bottom = corner(0, 1) + (corner(1, 1) - corner(0, 1)) * sx;
    top + (bottom - top) * sy
}

fn image(size: u32, mut pixel: impl FnMut(f64, f64) -> [f64; 3]) -> ImageFrame {
    let mut rgba = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let color = pixel(
                f64::from(x) / f64::from(size),
                f64::from(y) / f64::from(size),
            );
            for channel in color {
                rgba.push((channel.clamp(0.0, 1.0) * 255.0).round() as u8);
            }
            rgba.push(255);
        }
    }
    ImageFrame::from_rgba8(size, size, rgba)
}

fn mix(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

fn scale(color: [f64; 3], factor: f64) -> [f64; 3] {
    [color[0] * factor, color[1] * factor, color[2] * factor]
}

/// Oak planks, 8 per tile, staggered joints and grain.
fn wood_floor_texture() -> ImageFrame {
    image(512, |u, v| {
        let rows = 8.0;
        let row = (v * rows).floor();
        let within = v * rows - row;
        let offset = (hash(row as u64 + 11) % 100) as f64 / 100.0;
        let along = (u + offset).rem_euclid(1.0);
        let plank_tone = (hash(row as u64 * 7 + (along * 2.0).floor() as u64) % 100) as f64 / 100.0;
        let grain = 0.5
            + 0.5 * ((along * 90.0 + noise(along * 12.0, within * 3.0, row as u64) * 6.0).sin());
        let base = mix([0.55, 0.39, 0.25], [0.70, 0.52, 0.34], plank_tone);
        let mut color = mix(base, scale(base, 0.82), grain * 0.35);
        let seam = within < 0.035 || (along * 2.0).fract() < 0.006;
        if seam {
            color = scale(color, 0.55);
        }
        color
    })
}

/// 60 cm grey tiles with grout.
fn tile_texture() -> ImageFrame {
    image(512, |u, v| {
        let tiles = 2.0;
        let (tu, tv) = ((u * tiles).fract(), (v * tiles).fract());
        let grout = tu < 0.012 || tv < 0.012;
        if grout {
            return [0.46, 0.46, 0.45];
        }
        let cell = ((u * tiles).floor() as u64) * 13 + (v * tiles).floor() as u64;
        let tone = 0.74 + 0.05 * (hash(cell) % 100) as f64 / 100.0;
        let speck = noise(u * 80.0, v * 80.0, cell) * 0.05;
        [tone + speck, tone + speck, tone + speck + 0.01]
    })
}

/// A deep red rug with a cream border and a lattice.
fn rug_texture() -> ImageFrame {
    image(512, |u, v| {
        let edge = u.min(v).min(1.0 - u).min(1.0 - v);
        let weave = 0.92 + 0.08 * noise(u * 200.0, v * 200.0, 5);
        if edge < 0.035 {
            return scale([0.86, 0.80, 0.66], weave);
        }
        if edge < 0.07 {
            return scale([0.22, 0.24, 0.40], weave);
        }
        let lattice = ((u * 10.0).fract() - 0.5).abs() + ((v * 5.0).fract() - 0.5).abs();
        let base = if lattice < 0.18 {
            [0.84, 0.66, 0.34]
        } else {
            [0.52, 0.12, 0.13]
        };
        scale(base, weave)
    })
}

fn plaster_texture() -> ImageFrame {
    image(256, |u, v| {
        let n = noise(u * 24.0, v * 24.0, 3) * 0.06 + noise(u * 90.0, v * 90.0, 4) * 0.03;
        [0.90 + n, 0.89 + n, 0.86 + n]
    })
}

fn oak_texture() -> ImageFrame {
    image(256, |u, v| {
        let grain = 0.5 + 0.5 * ((v * 60.0 + noise(u * 3.0, v * 8.0, 9) * 8.0).sin());
        mix([0.66, 0.49, 0.32], [0.54, 0.38, 0.24], grain * 0.6)
    })
}

/// Four shelves of book spines inside a frame.
fn books_texture() -> ImageFrame {
    const PALETTE: [[f64; 3]; 8] = [
        [0.55, 0.12, 0.12],
        [0.13, 0.25, 0.45],
        [0.18, 0.38, 0.24],
        [0.78, 0.62, 0.30],
        [0.30, 0.22, 0.40],
        [0.85, 0.82, 0.74],
        [0.20, 0.20, 0.22],
        [0.62, 0.30, 0.16],
    ];
    image(512, |u, v| {
        let shelves = 4.0;
        let shelf = (v * shelves).floor();
        let within = v * shelves - shelf;
        if within > 0.93 {
            return [0.60, 0.45, 0.30];
        }
        // Book widths vary; walk across the shelf in hashed steps.
        let slot = (u * 36.0 + (hash(shelf as u64) % 7) as f64 * 0.3).floor() as u64;
        let book = hash(slot * 17 + shelf as u64 * 1000);
        let height = 0.55 + 0.35 * ((book >> 8) % 100) as f64 / 100.0;
        if 0.93 - within > height {
            return [0.20, 0.15, 0.11];
        }
        let color = PALETTE[(book % PALETTE.len() as u64) as usize];
        let band = ((0.93 - within) / height - 0.2).abs() < 0.03;
        if band {
            scale(color, 1.4)
        } else {
            color
        }
    })
}

/// Horizontal pine slats with gaps.
fn crate_texture() -> ImageFrame {
    image(256, |u, v| {
        let slats = 5.0;
        let within = (v * slats).fract();
        if within < 0.07 {
            return [0.20, 0.14, 0.09];
        }
        let grain = noise(u * 30.0, v * 6.0, (v * slats) as u64) * 0.12;
        [0.74 + grain, 0.58 + grain, 0.38 + grain]
    })
}

/// Cardboard with a band of packing tape.
fn cardboard_texture() -> ImageFrame {
    image(256, |u, v| {
        if (u - 0.5).abs() < 0.08 {
            return [0.78, 0.66, 0.44];
        }
        let fiber = noise(u * 60.0, v * 60.0, 12) * 0.06;
        [0.66 + fiber, 0.50 + fiber, 0.32 + fiber]
    })
}

fn stone_texture() -> ImageFrame {
    image(256, |u, v| {
        let vein = (noise(u * 5.0, v * 5.0, 21) * 10.0).sin().abs();
        let speck = noise(u * 120.0, v * 120.0, 22) * 0.08;
        let t = 0.82 + speck - if vein < 0.08 { 0.15 } else { 0.0 };
        [t, t, t + 0.01]
    })
}

fn fabric_texture() -> ImageFrame {
    image(256, |u, v| {
        let weave = (((u * 160.0).sin() * (v * 160.0).sin()) * 0.5 + 0.5) * 0.06;
        let tone = noise(u * 10.0, v * 10.0, 31) * 0.05;
        [
            0.28 + weave + tone,
            0.38 + weave + tone,
            0.46 + weave + tone,
        ]
    })
}

/// Two drawers over two doors, with pull handles, for one cabinet face.
fn cabinet_front_texture() -> ImageFrame {
    image(512, |u, v| {
        let base = [0.93, 0.92, 0.88];
        let line = |a: f64| (a.fract() < 0.008) || (a.fract() > 0.992);
        let drawer_row = v < 0.3;
        let columns = u * 4.0;
        let rows = if drawer_row { v / 0.3 } else { (v - 0.3) / 0.7 };
        if line(columns) || (v - 0.3).abs() < 0.004 || line(rows) && v > 0.3 {
            return [0.55, 0.54, 0.52];
        }
        let cx = columns.fract();
        let handle = if drawer_row {
            (cx - 0.5).abs() < 0.18 && (v / 0.3 - 0.35).abs() < 0.05
        } else {
            (cx - 0.88).abs() < 0.025 && (rows - 0.25).abs() < 0.12
        };
        if handle {
            return [0.30, 0.30, 0.32];
        }
        base
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object(shape: ColliderShape, translation: Vec3, yaw_rad: f64) -> StaticObject {
        StaticObject {
            name: "test".to_string(),
            pose: WorldTransform3::from_translation_rotation(
                translation,
                Quat::from_rotation_y(yaw_rad),
            ),
            shape,
        }
    }

    #[test]
    fn clearance_measures_box_and_capsule_footprints() {
        let wall = object(
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(1.0, 0.5, 0.05),
            },
            Vec3::new(0.0, 0.5, 0.0),
            0.0,
        );
        assert!((wall.clearance_m([0.0, 0.55]) - 0.5).abs() < 1e-12);
        assert!((wall.clearance_m([1.3, 0.05]) - 0.3).abs() < 1e-12);
        assert_eq!(wall.clearance_m([0.2, 0.0]), 0.0);
        // A quarter turn swaps the footprint's axes.
        let turned = object(
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(1.0, 0.5, 0.05),
            },
            Vec3::new(0.0, 0.5, 0.0),
            std::f64::consts::FRAC_PI_2,
        );
        assert!((turned.clearance_m([0.55, 0.0]) - 0.5).abs() < 1e-9);
        let plant = object(
            ColliderShape::Capsule {
                half_height_m: 0.3,
                radius_m: 0.25,
            },
            Vec3::new(2.0, 0.55, 0.0),
            0.0,
        );
        assert!((plant.clearance_m([2.0, 1.0]) - 0.75).abs() < 1e-12);
        assert!(
            (clearance_m(&[wall, plant], [2.0, 0.6]) - 0.35).abs() < 1e-12,
            "nearest of the two"
        );
    }

    #[test]
    fn textures_are_deterministic() {
        assert_eq!(wood_floor_texture().rgba8, wood_floor_texture().rgba8);
        assert_eq!(books_texture().hash_pixels(), books_texture().hash_pixels());
    }
}
