//! Spawn ECS entities from parsed assets.

use crate::error::AssetError;
use crate::robot::{RobotAsset, RobotKind};
use crate::scene::SceneAsset;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_robot::{spawn_diff_drive_robot, DiffDriveSpawned, Link};
use rne_urdf_import::{attach_urdf_visuals, parse_urdf_file};
use rne_world::{spawn_world, Gravity, Transform3, WorldEntity};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Result of spawning a robot asset into the ECS world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpawnedRobot {
    /// Robot root entity.
    pub robot: Entity,
    /// Base link entity.
    pub base_link: Entity,
}

/// Result of spawning a scene asset into the ECS world.
#[derive(Clone, Debug, PartialEq)]
pub struct SpawnedScene {
    /// World root entity.
    pub world: Entity,
    /// Spawned robots keyed by model name.
    pub robots: Vec<(String, SpawnedRobot)>,
}

/// Spawns entities described by a robot asset.
pub fn spawn_robot_asset(
    world: &mut World,
    asset_path: &Path,
    asset: &RobotAsset,
) -> Result<SpawnedRobot, AssetError> {
    match asset.kind {
        RobotKind::DiffDrive => {
            let section = asset
                .diff_drive
                .as_ref()
                .ok_or_else(|| AssetError::invalid("robot", "missing diff_drive section"))?;
            let spawned = spawn_diff_drive_robot(world, &section.to_config(&asset.model_name));
            if let Some(visuals) = &asset.visuals {
                attach_diff_drive_visuals(world, asset_path, visuals, &spawned)?;
            }
            Ok(SpawnedRobot {
                robot: spawned.robot,
                base_link: spawned.base_link,
            })
        }
        RobotKind::Urdf => Err(AssetError::UnsupportedRobotKind {
            kind: "urdf".into(),
        }),
    }
}

/// Spawns a scene and its referenced diff-drive robots.
pub fn spawn_scene(
    world: &mut World,
    scene: &SceneAsset,
    robots: &[(PathBuf, RobotAsset)],
) -> Result<SpawnedScene, AssetError> {
    if robots.len() != scene.robots.len() {
        return Err(AssetError::invalid(
            "scene",
            format!(
                "expected {} robot assets, got {}",
                scene.robots.len(),
                robots.len()
            ),
        ));
    }

    let world_entity = spawn_world(world);
    world.entity_mut(world_entity).insert((
        WorldEntity {
            gravity_m_s2: vec3_from_array(scene.world.gravity_m_s2),
            seed: scene.world.seed,
            ..WorldEntity::default()
        },
        Gravity {
            vector_m_s2: vec3_from_array(scene.world.gravity_m_s2),
        },
    ));

    if scene.ground.enabled {
        spawn_ground_plane(world);
    }

    let mut spawned_robots = Vec::new();
    for (index, (robot_path, robot_asset)) in robots.iter().enumerate() {
        let spawned =
            spawn_robot_asset(world, robot_path, robot_asset).map_err(|error| match error {
                AssetError::UnsupportedRobotKind { kind } => AssetError::invalid(
                    scene.robots[index].path.clone(),
                    format!("robot #{index} kind `{kind}` is not supported by spawn_scene"),
                ),
                other => other,
            })?;
        spawned_robots.push((robot_asset.model_name.clone(), spawned));
    }

    Ok(SpawnedScene {
        world: world_entity,
        robots: spawned_robots,
    })
}

/// Loads and spawns a scene file and its referenced robot assets.
pub fn load_and_spawn_scene(
    world: &mut World,
    scene_path: &Path,
) -> Result<SpawnedScene, AssetError> {
    let scene = crate::scene::load_scene_asset(scene_path)?;
    let robots = crate::scene::load_scene_robots(scene_path, &scene)?;
    spawn_scene(world, &scene, &robots)
}

/// Spawns a fixed ground plane collider used by built-in scenes.
pub fn spawn_ground_plane(world: &mut World) -> Entity {
    let ground = spawn_named(world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(20.0, 0.5, 20.0),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    ground
}

/// Convenience wrapper returning full diff-drive spawn details.
pub fn spawn_diff_drive_from_asset(
    world: &mut World,
    asset: &RobotAsset,
) -> Result<DiffDriveSpawned, AssetError> {
    let section = asset
        .diff_drive
        .as_ref()
        .ok_or_else(|| AssetError::invalid("robot", "missing diff_drive section"))?;
    Ok(spawn_diff_drive_robot(
        world,
        &section.to_config(&asset.model_name),
    ))
}

fn vec3_from_array(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

fn attach_diff_drive_visuals(
    world: &mut World,
    asset_path: &Path,
    visuals: &crate::robot::VisualsRobotAsset,
    spawned: &DiffDriveSpawned,
) -> Result<(), AssetError> {
    let base_dir = asset_path.parent().unwrap_or_else(|| Path::new("."));
    let urdf_path = visuals.resolve_urdf_path(base_dir);
    let urdf = parse_urdf_file(&urdf_path).map_err(|error| {
        AssetError::invalid(
            asset_path.display().to_string(),
            format!(
                "visuals urdf parse failed ({}): {error}",
                urdf_path.display()
            ),
        )
    })?;

    let links = collect_robot_links(world, spawned.robot);
    let attached = attach_urdf_visuals(world, &urdf, &links, [0.7, 0.7, 0.75, 1.0]);
    if attached == 0 {
        return Err(AssetError::invalid(
            asset_path.display().to_string(),
            format!(
                "visuals urdf attached no matching links: {}",
                urdf_path.display()
            ),
        ));
    }
    Ok(())
}

fn collect_robot_links(world: &mut World, robot: Entity) -> HashMap<String, Entity> {
    let mut links = HashMap::new();
    let mut query = world.query::<(Entity, &Link)>();
    for (entity, link) in query.iter(world) {
        if link.robot == robot {
            links.insert(link.name.clone(), entity);
        }
    }
    links
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawn_scene_from_fixture() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, &scene_path).unwrap();

        assert!(world.get::<WorldEntity>(spawned.world).is_some());
        assert_eq!(spawned.robots.len(), 1);
        assert!(world
            .get::<RigidBody>(spawned.robots[0].1.base_link)
            .is_some());
    }

    #[test]
    fn diff_drive_visuals_attach_from_urdf() {
        let robot_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/mesh_diff_drive.rne.robot.toml");
        let asset = crate::robot::load_robot_asset(&robot_path).unwrap();
        let mut world = World::new();
        let spawned = spawn_robot_asset(&mut world, &robot_path, &asset).unwrap();
        assert!(world.get::<rne_render::Visual>(spawned.base_link).is_some());
    }

    #[test]
    fn urdf_robot_asset_cannot_spawn_in_core_loader() {
        let robot_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/diff_drive_urdf.rne.robot.toml");
        let asset = crate::robot::load_robot_asset(&robot_path).unwrap();
        let mut world = World::new();
        let error = spawn_robot_asset(&mut world, &robot_path, &asset).unwrap_err();
        assert!(matches!(error, AssetError::UnsupportedRobotKind { .. }));
    }
}
