//! Render scene helpers for diff-drive simulation.

use rne_data::{DataBus, InMemoryDataBus, PointCloud};
use rne_ecs::{Entity, Parent, World};
use rne_math::{yaw_rad, Quat, Vec3};
use rne_physics::Collider;
use rne_render::{RenderScene, Visual, VisualShape};
use rne_robot::DiffDriveSpawned;
use rne_sensor::{Sensor, SensorKind};
use rne_world::{world_transform_of, Transform3 as WorldTransform3};

const GROUND_COLOR: [f32; 4] = [0.25, 0.28, 0.32, 1.0];
const BASE_COLOR: [f32; 4] = [0.35, 0.55, 0.95, 1.0];
const WHEEL_COLOR: [f32; 4] = [0.2, 0.2, 0.2, 1.0];
const LIDAR_HIT_COLOR: [f32; 4] = [0.15, 0.95, 0.35, 0.85];
const LIDAR_MOUNT_COLOR: [f32; 4] = [0.95, 0.35, 0.25, 1.0];

/// Summary of LiDAR markers appended to a render scene.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LidarOverlayStats {
    /// Number of ray hit markers drawn.
    pub hit_markers: usize,
    /// Number of sensor mount markers drawn.
    pub mount_markers: usize,
}

impl LidarOverlayStats {
    /// Total number of overlay spheres added to the scene.
    pub fn total_markers(&self) -> usize {
        self.hit_markers + self.mount_markers
    }
}

/// Appends LiDAR hit and mount markers from the latest DataBus frames.
pub fn append_lidar_overlay(
    scene: &mut RenderScene,
    world: &World,
    data_bus: &InMemoryDataBus,
) -> LidarOverlayStats {
    let mut stats = LidarOverlayStats::default();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        let Some(sensor) = world.get::<Sensor>(entity) else {
            continue;
        };
        let SensorKind::Lidar(_) = sensor.kind else {
            continue;
        };
        let Some(frame) = data_bus.latest::<PointCloud>(sensor.stream_id) else {
            continue;
        };
        let hits = frame.payload.points_m.len();
        if hits > 0 {
            scene.append_lidar_points(&frame.payload.points_m, LIDAR_HIT_COLOR);
            stats.hit_markers += hits;
        }

        if let Some(mount) = world
            .get::<WorldTransform3>(entity)
            .map(|transform| transform.translation)
        {
            scene.append_lidar_points_sized(std::slice::from_ref(&mount), 0.06, LIDAR_MOUNT_COLOR);
            stats.mount_markers += 1;
        }
    }

    stats
}

/// Builds a render scene for one or more diff-drive robots and an optional ground plane.
pub fn build_diff_drive_render_scene(world: &World, robots: &[DiffDriveSpawned]) -> RenderScene {
    let mut scene = RenderScene::new();
    for robot in robots {
        append_robot_items(&mut scene, world, robot);
    }
    append_ground_plane(&mut scene);
    scene
}

/// Builds a render scene from all entities that carry visuals or colliders.
pub fn build_visual_render_scene(world: &World) -> RenderScene {
    let mut scene = RenderScene::new();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world.get::<Visual>(entity).is_some() || world.get::<Collider>(entity).is_some() {
            append_entity_visual(&mut scene, world, entity, false, None);
        }
    }
    scene
}

fn append_robot_items(scene: &mut RenderScene, world: &World, robot: &DiffDriveSpawned) {
    for (entity, yaw_only) in [
        (robot.base_link, true),
        (robot.left_wheel, false),
        (robot.right_wheel, false),
    ] {
        if !yaw_only && entity == robot.base_link {
            continue;
        }
        let (fallback_shape, fallback_color) = fallback_visual_for_link(world, robot, entity);
        append_entity_visual(
            scene,
            world,
            entity,
            yaw_only,
            Some((fallback_shape, fallback_color)),
        );
    }
}

fn fallback_visual_for_link(
    world: &World,
    robot: &DiffDriveSpawned,
    entity: Entity,
) -> (VisualShape, [f32; 4]) {
    if entity == robot.base_link {
        return (
            VisualShape::Box {
                size_m: base_size_m(world, robot.base_link),
            },
            BASE_COLOR,
        );
    }

    if entity == robot.left_wheel || entity == robot.right_wheel {
        return (
            VisualShape::Cylinder {
                radius_m: robot.drive.wheel_radius_m,
                length_m: robot.drive.wheel_radius_m * 0.6,
            },
            WHEEL_COLOR,
        );
    }

    (
        VisualShape::Box {
            size_m: Vec3::new(0.1, 0.1, 0.1),
        },
        BASE_COLOR,
    )
}

fn append_entity_visual(
    scene: &mut RenderScene,
    world: &World,
    entity: Entity,
    yaw_only: bool,
    fallback: Option<(VisualShape, [f32; 4])>,
) {
    let world_transform = link_render_transform(world, entity, yaw_only);

    if let Some(visual) = world.get::<Visual>(entity) {
        scene.items.push(RenderScene::item_from_visual(
            world_transform,
            visual.shape.clone(),
            visual.color_rgba,
            visual.local_offset,
        ));
        return;
    }

    let Some((fallback_shape, fallback_color)) = fallback else {
        if let Some(collider) = world.get::<Collider>(entity) {
            if let Some((shape, color)) = collider_fallback_visual(collider) {
                scene.items.push(RenderScene::item_from_visual(
                    world_transform,
                    shape,
                    color,
                    WorldTransform3::IDENTITY,
                ));
            }
        }
        return;
    };

    scene.items.push(RenderScene::item_from_visual(
        world_transform,
        fallback_shape,
        fallback_color,
        WorldTransform3::IDENTITY,
    ));
}

fn collider_fallback_visual(collider: &Collider) -> Option<(VisualShape, [f32; 4])> {
    match collider.shape {
        rne_physics::ColliderShape::Cuboid { half_extents_m } => Some((
            VisualShape::Box {
                size_m: half_extents_m * 2.0,
            },
            BASE_COLOR,
        )),
        rne_physics::ColliderShape::Sphere { radius_m } => {
            Some((VisualShape::Sphere { radius_m }, BASE_COLOR))
        }
        _ => None,
    }
}

/// Resolves a link transform for rendering, composing parent chains when present.
fn link_render_transform(world: &World, entity: Entity, yaw_only: bool) -> WorldTransform3 {
    let world_tf = if world.get::<Parent>(entity).is_some() {
        world_transform_of(world, entity)
    } else {
        world
            .get::<WorldTransform3>(entity)
            .copied()
            .unwrap_or_default()
    };

    if yaw_only {
        WorldTransform3::from_translation_rotation(
            world_tf.translation,
            Quat::from_rotation_y(yaw_rad(world_tf.rotation)),
        )
    } else {
        world_tf
    }
}

fn append_ground_plane(scene: &mut RenderScene) {
    scene.items.push(RenderScene::item_from_visual(
        WorldTransform3::from_translation_rotation(Vec3::new(0.0, -0.01, 0.0), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(40.0, 0.02, 40.0),
        },
        GROUND_COLOR,
        WorldTransform3::IDENTITY,
    ));
}

fn base_size_m(world: &World, base_link: Entity) -> Vec3 {
    world
        .get::<Collider>(base_link)
        .and_then(|collider| match collider.shape {
            rne_physics::ColliderShape::Cuboid { half_extents_m } => Some(half_extents_m * 2.0),
            _ => None,
        })
        .unwrap_or_else(|| Vec3::new(0.5, 0.3, 0.4))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DiffDriveSim;
    use std::path::PathBuf;

    fn mesh_scene_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml")
    }

    #[test]
    fn mesh_scene_includes_all_link_visuals() {
        let scene_path = mesh_scene_path();
        assert!(
            scene_path.is_file(),
            "missing mesh scene fixture at {}",
            scene_path.display()
        );

        let sim = DiffDriveSim::from_scene_path(&scene_path).expect("load mesh scene");
        let scene = build_diff_drive_render_scene(sim.world(), sim.robots());
        let robot_items = scene.items.len().saturating_sub(1);
        assert!(
            robot_items >= 3,
            "expected base + wheel visuals, got {robot_items} robot items"
        );

        let mesh_items = scene
            .items
            .iter()
            .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
            .count();
        let cylinder_items = scene
            .items
            .iter()
            .filter(|item| matches!(item.shape, VisualShape::Cylinder { .. }))
            .count();
        assert!(mesh_items >= 1, "expected base mesh visual");
        assert!(
            cylinder_items >= 2,
            "expected cylinder wheel visuals, got {cylinder_items}"
        );
    }
}
