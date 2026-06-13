//! Render scene helpers for diff-drive simulation.

use rne_ecs::World;
use rne_math::{yaw_rad, Quat, Vec3};
use rne_physics::Collider;
use rne_render::{RenderScene, Visual, VisualShape};
use rne_robot::DiffDriveSpawned;
use rne_world::Transform3 as WorldTransform3;

const GROUND_COLOR: [f32; 4] = [0.25, 0.28, 0.32, 1.0];
const BASE_COLOR: [f32; 4] = [0.35, 0.55, 0.95, 1.0];
const WHEEL_COLOR: [f32; 4] = [0.2, 0.2, 0.2, 1.0];

/// Builds a render scene for one or more diff-drive robots and an optional ground plane.
pub fn build_diff_drive_render_scene(world: &World, robots: &[DiffDriveSpawned]) -> RenderScene {
    let mut scene = RenderScene::new();
    for robot in robots {
        append_robot_items(&mut scene, world, robot);
    }
    append_ground_plane(&mut scene);
    scene
}

fn append_robot_items(scene: &mut RenderScene, world: &World, robot: &DiffDriveSpawned) {
    let drive = robot.drive;
    append_link_item(
        scene,
        world,
        robot.base_link,
        VisualShape::Box {
            size_m: base_size_m(world, robot.base_link),
        },
        BASE_COLOR,
    );

    for wheel in [robot.left_wheel, robot.right_wheel] {
        if wheel == robot.base_link {
            continue;
        }
        append_link_item(
            scene,
            world,
            wheel,
            VisualShape::Cylinder {
                radius_m: drive.wheel_radius_m,
                length_m: drive.wheel_radius_m * 0.6,
            },
            WHEEL_COLOR,
        );
    }
}

fn append_link_item(
    scene: &mut RenderScene,
    world: &World,
    entity: rne_ecs::Entity,
    fallback_shape: VisualShape,
    fallback_color: [f32; 4],
) {
    let world_transform = render_transform(world, entity);

    if let Some(visual) = world.get::<Visual>(entity) {
        scene.items.push(RenderScene::item_from_visual(
            world_transform,
            visual.shape.clone(),
            visual.color_rgba,
            visual.local_offset,
        ));
        return;
    }

    let local_offset = WorldTransform3::IDENTITY;
    scene.items.push(RenderScene::item_from_visual(
        world_transform,
        fallback_shape,
        fallback_color,
        local_offset,
    ));
}

/// Builds a render transform using world translation and yaw-only rotation.
fn render_transform(world: &World, entity: rne_ecs::Entity) -> WorldTransform3 {
    let transform = world
        .get::<WorldTransform3>(entity)
        .copied()
        .unwrap_or_default();
    WorldTransform3::from_translation_rotation(
        transform.translation,
        Quat::from_rotation_y(yaw_rad(transform.rotation)),
    )
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

fn base_size_m(world: &World, base_link: rne_ecs::Entity) -> Vec3 {
    world
        .get::<Collider>(base_link)
        .and_then(|collider| match collider.shape {
            rne_physics::ColliderShape::Cuboid { half_extents_m } => Some(half_extents_m * 2.0),
            _ => None,
        })
        .unwrap_or_else(|| Vec3::new(0.5, 0.3, 0.4))
}
