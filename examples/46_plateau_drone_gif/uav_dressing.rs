//! Render-only dressing for the UAV view: facade textures on the PLATEAU
//! building volumes and a detailed quadrotor. The buildings keep the exact
//! boxes the PLATEAU import produced; only their surfaces are drawn with
//! windows. The quadrotor is drawn at the simulated body pose.

use super::{push_box, push_cylinder, push_sphere, uav_camera_transform, Footprint, UavFrame};
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};
use rne_render::{
    ImageFrame, PbrMaterial, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
use std::sync::Arc;

/// Width of one facade bay and height of one storey, in meters: one repeat
/// of a facade texture.
const BAY_M: f64 = 2.4;
const STOREY_M: f64 = 3.2;
const TEXTURE_W: u32 = 96;
const TEXTURE_H: u32 = 128;

/// Draws the PLATEAU LOD1 building meshes with windowed facades and roofs.
/// The imported geometry is kept exactly: every triangle is moved to world
/// space unchanged and given texture coordinates, walls one texture repeat
/// per bay and storey counted from the ground, roofs a planar mapping.
/// Buildings that already carry a texture, such as the LOD2 mesh, are left
/// alone.
pub(crate) fn apply_facades(scene: &mut RenderScene, footprints: &[Footprint]) {
    let is_lod1_building = |item: &RenderSceneItem| {
        matches!(&item.shape, VisualShape::Mesh { path, .. } if path.contains("plateau_building_"))
            && item.base_color_texture.is_none()
            && item.mesh.is_some()
    };
    let mut walls: [MeshBuilder; 3] = Default::default();
    let mut roofs = MeshBuilder::default();
    for (index, item) in scene
        .items
        .iter()
        .filter(|item| is_lod1_building(item))
        .enumerate()
    {
        let mesh = item.mesh.as_ref().expect("building mesh resolved");
        let transform = item.transform;
        let world = mesh
            .positions
            .iter()
            .map(|&[x, y, z]| {
                transform.translation
                    + transform.rotation
                        * Vec3::new(
                            f64::from(x) * transform.scale.x,
                            f64::from(y) * transform.scale.y,
                            f64::from(z) * transform.scale.z,
                        )
            })
            .collect::<Vec<_>>();
        let (ground, top) = world
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), point| {
                (low.min(point.y), high.max(point.y))
            });
        // Houses stay residential; taller blocks mix glass and concrete
        // offices with apartment fronts.
        let style = if top - ground < 7.0 {
            2
        } else {
            (hash(index) % 3) as usize
        };
        for triangle in mesh.indices.chunks_exact(3) {
            let corners = [
                world[triangle[0] as usize],
                world[triangle[1] as usize],
                world[triangle[2] as usize],
            ];
            let normal = (corners[1] - corners[0])
                .cross(corners[2] - corners[0])
                .normalize_or_zero();
            if normal.y.abs() < 0.3 {
                let along = Vec3::Y.cross(normal).normalize_or_zero();
                let uv = corners.map(|point| {
                    [
                        (point.dot(along) / BAY_M) as f32,
                        (1.0 - (point.y - ground) / STOREY_M) as f32,
                    ]
                });
                walls[style].triangle(corners, normal, uv);
            } else if normal.y > 0.0 {
                let uv = corners.map(|point| [(point.x / 6.0) as f32, (point.z / 6.0) as f32]);
                roofs.triangle(corners, normal, uv);
            }
        }
    }
    scene
        .items
        .retain(|item| !is_lod1_building(item) && !is_box_detail(item, footprints));

    let textures = [glass_facade(), concrete_facade(), residential_facade()];
    for ((builder, texture), path) in walls.into_iter().zip(textures).zip([
        "uav-facade-glass",
        "uav-facade-concrete",
        "uav-facade-residential",
    ]) {
        if let Some(mesh) = builder.build() {
            scene
                .items
                .push(textured_item(path, mesh, texture, [1.0; 4], 0.55));
        }
    }
    if let Some(mesh) = roofs.build() {
        scene.items.push(textured_item(
            "uav-roof",
            mesh,
            roof_texture(),
            [1.0; 4],
            0.9,
        ));
    }
}

/// The glass strips and roof caps `append_plateau_building_details` lays
/// on a dozen buildings. Both follow the collision box rather than the
/// building outline, so they float beside rotated or irregular buildings;
/// the textured facades and roofs replace them.
fn is_box_detail(item: &RenderSceneItem, footprints: &[Footprint]) -> bool {
    let VisualShape::Box { size_m } = item.shape else {
        return false;
    };
    let size = Vec3::new(
        size_m.x * item.transform.scale.x,
        size_m.y * item.transform.scale.y,
        size_m.z * item.transform.scale.z,
    );
    let at = item.transform.translation;
    let strip = (size.x - 0.055).abs() < 1e-9 || (size.z - 0.055).abs() < 1e-9;
    footprints.iter().any(|building| {
        let around = (building.min_x_m - 0.1..=building.max_x_m + 0.1).contains(&at.x)
            && (building.min_z_m - 0.1..=building.max_z_m + 0.1).contains(&at.z);
        let cap = (size.y - 0.24).abs() < 1e-9 && (at.y - building.max_y_m - 0.12).abs() < 1e-9;
        around && (strip || cap)
    })
}

fn hash(value: usize) -> u64 {
    let mut x = value as u64 ^ 0x9e37_79b9_7f4a_7c15;
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

fn textured_item(
    path: &str,
    mesh: TriangleMesh,
    texture: Arc<ImageFrame>,
    color: [f32; 4],
    roughness: f32,
) -> RenderSceneItem {
    RenderSceneItem {
        transform: MathTransform3::IDENTITY,
        shape: VisualShape::Mesh {
            path: path.to_string(),
            scale: Vec3::ONE,
        },
        color_rgba: color,
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(texture),
        material: PbrMaterial::new(color, roughness, 0.05, [0.0; 3]),
    }
}

#[derive(Default)]
struct MeshBuilder {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl MeshBuilder {
    fn triangle(&mut self, corners: [Vec3; 3], normal: Vec3, uv: [[f32; 2]; 3]) {
        let base = self.positions.len() as u32;
        for (corner, uv) in corners.iter().zip(uv) {
            self.positions
                .push([corner.x as f32, corner.y as f32, corner.z as f32]);
            self.normals
                .push([normal.x as f32, normal.y as f32, normal.z as f32]);
            self.texcoords.push(uv);
        }
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }

    fn build(self) -> Option<TriangleMesh> {
        (!self.indices.is_empty()).then_some(TriangleMesh {
            positions: self.positions,
            normals: self.normals,
            texcoords: self.texcoords,
            indices: self.indices,
            skinning: None,
        })
    }
}

fn texture(pixel: impl Fn(f32, f32) -> [f32; 3]) -> Arc<ImageFrame> {
    let mut rgba = Vec::with_capacity((TEXTURE_W * TEXTURE_H * 4) as usize);
    for y in 0..TEXTURE_H {
        for x in 0..TEXTURE_W {
            let u = (x as f32 + 0.5) / TEXTURE_W as f32;
            let v = (y as f32 + 0.5) / TEXTURE_H as f32;
            let [r, g, b] = pixel(u, v);
            rgba.extend_from_slice(&[
                (r.clamp(0.0, 1.0) * 255.0) as u8,
                (g.clamp(0.0, 1.0) * 255.0) as u8,
                (b.clamp(0.0, 1.0) * 255.0) as u8,
                255,
            ]);
        }
    }
    Arc::new(ImageFrame::from_rgba8(TEXTURE_W, TEXTURE_H, rgba))
}

// The renderer samples these without mipmaps, so every feature is kept at
// least a tenth of a bay wide: thinner lines shimmer from frame to frame on
// distant buildings.

/// A curtain wall: two tinted glass panes per bay between aluminium
/// mullions, and a dark spandrel band at each floor slab. The texture's top
/// row is the top of a storey.
fn glass_facade() -> Arc<ImageFrame> {
    texture(|u, v| {
        let mullion = (u * 2.0).fract() < 0.1;
        let spandrel = v > 0.8;
        if mullion {
            [0.46, 0.49, 0.52]
        } else if spandrel {
            [0.24, 0.26, 0.29]
        } else {
            // Sky reflected in the upper glass, darker below.
            let sky = 1.0 - v;
            [0.20 + 0.14 * sky, 0.28 + 0.16 * sky, 0.36 + 0.18 * sky]
        }
    })
}

/// A concrete office front: one window with a sill per bay.
fn concrete_facade() -> Arc<ImageFrame> {
    texture(|u, v| {
        let window = (0.15..0.85).contains(&u) && (0.16..0.66).contains(&v);
        let sill = (0.12..0.88).contains(&u) && (0.66..0.76).contains(&v);
        if window {
            let shade = 0.1 * (0.66 - v);
            [0.16 + shade, 0.20 + shade, 0.25 + shade]
        } else if sill {
            [0.60, 0.61, 0.62]
        } else {
            [0.70, 0.69, 0.66]
        }
    })
}

/// A residential front: a curtained window over a balcony per bay, warm
/// render.
fn residential_facade() -> Arc<ImageFrame> {
    texture(|u, v| {
        let window = (0.12..0.88).contains(&u) && (0.14..0.66).contains(&v);
        let slab = (0.66..0.76).contains(&v);
        let balcony = (0.76..0.96).contains(&v);
        if balcony {
            [0.42, 0.43, 0.44]
        } else if slab {
            [0.84, 0.82, 0.78]
        } else if window {
            let curtain = (u - 0.5).abs() > 0.26;
            if curtain {
                [0.62, 0.58, 0.52]
            } else {
                [0.18, 0.21, 0.24]
            }
        } else {
            [0.80, 0.73, 0.62]
        }
    })
}

/// A membrane roof laid in 6 m sheets with wide, low-contrast laps.
fn roof_texture() -> Arc<ImageFrame> {
    texture(|u, v| {
        let seam = u.fract() < 0.1 || v.fract() < 0.1;
        if seam {
            [0.41, 0.41, 0.41]
        } else {
            [0.46, 0.46, 0.45]
        }
    })
}

/// A heavy-lift quadrotor drawn at the simulated body pose: a shelled body
/// with its battery, four folding arms with motors and spinning two-blade
/// propellers, navigation lights, a GNSS mast, landing skids and the camera
/// gimbal the onboard RGB-D camera looks from.
pub(crate) fn append_detailed_quadrotor(scene: &mut RenderScene, frame: UavFrame) {
    const SHELL: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
    const TOP: [f32; 4] = [0.70, 0.72, 0.74, 1.0];
    const ACCENT: [f32; 4] = [0.92, 0.42, 0.08, 1.0];
    const CARBON: [f32; 4] = [0.07, 0.07, 0.08, 1.0];
    const METAL: [f32; 4] = [0.46, 0.48, 0.52, 1.0];
    const BLADE: [f32; 4] = [0.10, 0.11, 0.12, 1.0];
    let center = frame.transform.translation;
    let rotation = frame.transform.rotation;
    let at = |local: Vec3| center + rotation * local;
    let vertical = rotation * Quat::from_rotation_x(-FRAC_PI_2);

    // Body: a lower shell, a lighter top cover, a nose and the battery.
    push_box(
        scene,
        at(Vec3::new(0.0, -0.02, 0.0)),
        rotation,
        Vec3::new(0.62, 0.16, 0.40),
        SHELL,
    );
    push_box(
        scene,
        at(Vec3::new(-0.02, 0.08, 0.0)),
        rotation,
        Vec3::new(0.52, 0.06, 0.34),
        TOP,
    );
    push_box(
        scene,
        at(Vec3::new(0.32, 0.0, 0.0)),
        rotation,
        Vec3::new(0.10, 0.12, 0.30),
        SHELL,
    );
    push_box(
        scene,
        at(Vec3::new(-0.08, 0.12, 0.0)),
        rotation,
        Vec3::new(0.30, 0.05, 0.16),
        ACCENT,
    );
    // GNSS mast with its disc antenna.
    push_cylinder(
        scene,
        at(Vec3::new(-0.2, 0.2, 0.0)),
        vertical,
        0.012,
        0.18,
        CARBON,
    );
    push_cylinder(
        scene,
        at(Vec3::new(-0.2, 0.3, 0.0)),
        vertical,
        0.06,
        0.02,
        TOP,
    );

    // Arms, motors and propellers at the four corners.
    for (index, arm_yaw) in [FRAC_PI_4, -FRAC_PI_4, PI - FRAC_PI_4, -(PI - FRAC_PI_4)]
        .into_iter()
        .enumerate()
    {
        let direction = Vec3::new(arm_yaw.cos(), 0.0, -arm_yaw.sin());
        let hub_local = direction * 0.877 + Vec3::new(0.0, 0.04, 0.0);
        let arm_rotation = rotation * Quat::from_rotation_arc(Vec3::Z, direction);
        push_cylinder(
            scene,
            at(direction * 0.48),
            arm_rotation,
            0.028,
            0.8,
            CARBON,
        );
        // Folding hinge collar near the body.
        push_cylinder(scene, at(direction * 0.2), arm_rotation, 0.04, 0.06, METAL);
        let hub = at(hub_local);
        push_cylinder(scene, hub, vertical, 0.07, 0.1, SHELL);
        push_cylinder(
            scene,
            hub + rotation * Vec3::new(0.0, 0.06, 0.0),
            vertical,
            0.075,
            0.02,
            METAL,
        );
        push_cylinder(
            scene,
            hub + rotation * Vec3::new(0.0, 0.08, 0.0),
            vertical,
            0.022,
            0.03,
            METAL,
        );
        let spin = if index % 2 == 0 {
            frame.rotor_angle_rad
        } else {
            -frame.rotor_angle_rad
        };
        let blade_rotation = rotation * Quat::from_rotation_y(spin);
        for half in [0.0, PI] {
            let blade = blade_rotation * Quat::from_rotation_y(half);
            push_box(
                scene,
                hub + rotation * Vec3::new(0.0, 0.085, 0.0) + blade * Vec3::new(0.22, 0.0, 0.0),
                blade * Quat::from_rotation_x(0.12),
                Vec3::new(0.44, 0.012, 0.06),
                BLADE,
            );
        }
        // Navigation lights under the motors: green starboard front, red
        // port front, white aft.
        let light = match index {
            0 => [0.1, 1.0, 0.35, 1.0],
            1 => [1.0, 0.1, 0.06, 1.0],
            _ => [1.0, 1.0, 1.0, 1.0],
        };
        push_sphere(
            scene,
            hub + rotation * Vec3::new(0.0, -0.07, 0.0),
            0.028,
            light,
        );
        if let Some(last) = scene.items.last_mut() {
            last.material = PbrMaterial::new(light, 0.2, 0.0, [light[0], light[1], light[2]]);
        }
    }

    append_undercarriage(scene, frame);
}

/// Landing skids and the camera gimbal under the body.
fn append_undercarriage(scene: &mut RenderScene, frame: UavFrame) {
    const SHELL: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
    const CARBON: [f32; 4] = [0.07, 0.07, 0.08, 1.0];
    const METAL: [f32; 4] = [0.46, 0.48, 0.52, 1.0];
    let center = frame.transform.translation;
    let rotation = frame.transform.rotation;
    let at = |local: Vec3| center + rotation * local;

    // Landing skids: two struts each side down to a tube along the body.
    for side in [-1.0, 1.0] {
        for x in [-0.16, 0.16] {
            push_cylinder(
                scene,
                at(Vec3::new(x, -0.2, side * 0.22)),
                rotation * Quat::from_rotation_x(-FRAC_PI_2 + side * 0.25),
                0.014,
                0.3,
                CARBON,
            );
        }
        push_cylinder(
            scene,
            at(Vec3::new(0.0, -0.34, side * 0.26)),
            rotation * Quat::from_rotation_y(FRAC_PI_2),
            0.016,
            0.62,
            CARBON,
        );
    }

    // Gimbal: a yoke under the nose holding the camera the onboard RGB-D
    // sensor renders from.
    let camera = uav_camera_transform(frame);
    push_box(
        scene,
        at(Vec3::new(0.26, -0.12, 0.0)),
        rotation,
        Vec3::new(0.05, 0.1, 0.12),
        METAL,
    );
    push_box(
        scene,
        camera.translation,
        camera.rotation,
        Vec3::new(0.1, 0.08, 0.1),
        SHELL,
    );
    push_cylinder(
        scene,
        camera.translation + camera.rotation * Vec3::NEG_Z * 0.06,
        camera.rotation,
        0.03,
        0.04,
        [0.12, 0.18, 0.26, 1.0],
    );
}
