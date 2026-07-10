//! Spawn ECS entities from parsed assets.

use crate::error::AssetError;
use crate::robot::{LidarRobotAsset, RobotAsset, RobotKind};
use crate::scene::{ObstacleBodyType, SceneAsset, SceneObstacleAsset};
use rne_data::StreamId;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, PhysicsMaterial, RigidBody, RigidBodyType};
use rne_robot::{spawn_diff_drive_robot, DiffDriveSpawned, Link};
use rne_sensor::{Sensor, SensorKind, SensorState};
use rne_urdf_import::{
    attach_urdf_articulation, attach_urdf_visuals, parse_urdf, parse_urdf_file,
    spawn_urdf_robot_with_config, UrdfRobot,
};
use rne_world::{spawn_world, world_transform_of, Gravity, Transform3, WorldEntity, WorldRandom};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Optional embedded URDF XML keyed by resolved robot-relative path.
pub type UrdfSourceMap<'a> = HashMap<PathBuf, &'a str>;

/// Controls how [`spawn_scene`] wires physics articulation for URDF robots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpawnSceneOptions {
    /// When false, URDF joints are spawned without Rapier articulation wiring.
    pub wire_articulation: bool,
}

impl Default for SpawnSceneOptions {
    /// Headless simulation and examples expect wired articulation unless a viewer
    /// opts out (see the web viewer's kinematic preview mode).
    fn default() -> Self {
        Self {
            wire_articulation: true,
        }
    }
}

const LIDAR_STREAM_BASE: u32 = 200;
const WRIST_CAMERA_STREAM_BASE: u32 = 400;

/// LiDAR mount spawned with a robot or scene.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarMountSpawned {
    /// Robot base link the LiDAR follows.
    pub base_link: Entity,
    /// LiDAR sensor entity.
    pub lidar: Entity,
    /// Mount offset from the base link origin in meters.
    pub mount_offset_m: Vec3,
}

/// Wrist camera mount spawned with a URDF robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WristCameraMountSpawned {
    /// Parent link the camera follows.
    pub parent_link: Entity,
    /// Camera sensor entity.
    pub camera: Entity,
    /// Mount offset from the parent link origin in meters.
    pub mount_offset_m: Vec3,
}

/// Optional sensors spawned alongside a robot asset.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RobotSensorMounts {
    /// LiDAR mount when configured.
    pub lidar: Option<LidarMountSpawned>,
    /// Wrist camera mount when configured.
    pub wrist_camera: Option<WristCameraMountSpawned>,
}

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
    /// LiDAR mounts spawned from robot assets.
    pub lidar_mounts: Vec<LidarMountSpawned>,
    /// Wrist camera mounts spawned from robot assets.
    pub wrist_camera_mounts: Vec<WristCameraMountSpawned>,
}

/// Spawns entities described by a robot asset.
pub fn spawn_robot_asset(
    world: &mut World,
    asset_path: &Path,
    asset: &RobotAsset,
    lidar_stream_index: Option<usize>,
) -> Result<(SpawnedRobot, RobotSensorMounts), AssetError> {
    spawn_robot_asset_with_sources(world, asset_path, asset, lidar_stream_index, None, true)
}

/// Spawns a robot asset using optional embedded URDF XML.
pub fn spawn_robot_asset_with_sources(
    world: &mut World,
    asset_path: &Path,
    asset: &RobotAsset,
    lidar_stream_index: Option<usize>,
    urdf_sources: Option<&UrdfSourceMap<'_>>,
    wire_articulation: bool,
) -> Result<(SpawnedRobot, RobotSensorMounts), AssetError> {
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
            let lidar_mount = asset.lidar.as_ref().and_then(|config| {
                config.enabled.then(|| {
                    spawn_robot_lidar(
                        world,
                        spawned.base_link,
                        config,
                        lidar_stream_index.unwrap_or(0),
                    )
                })
            });
            Ok((
                SpawnedRobot {
                    robot: spawned.robot,
                    base_link: spawned.base_link,
                },
                RobotSensorMounts {
                    lidar: lidar_mount,
                    wrist_camera: None,
                },
            ))
        }
        RobotKind::Urdf => {
            let section = asset
                .urdf
                .as_ref()
                .ok_or_else(|| AssetError::invalid("robot", "missing urdf section"))?;
            let base_dir = asset_path.parent().unwrap_or_else(|| Path::new("."));
            let urdf_path = section.resolve_path(base_dir);
            let urdf = load_urdf_robot(&urdf_path, urdf_sources).map_err(|error| {
                AssetError::invalid(
                    asset_path.display().to_string(),
                    format!("urdf parse failed ({}): {error}", urdf_path.display()),
                )
            })?;

            let mut spawn_config = section.to_spawn_config();
            if let Some(parent) = urdf_path.parent() {
                spawn_config.mesh_assets_root = Some(parent.to_path_buf());
            }

            let spawned =
                spawn_urdf_robot_with_config(world, &urdf, spawn_config).map_err(|error| {
                    AssetError::invalid(
                        asset_path.display().to_string(),
                        format!("urdf spawn failed: {error}"),
                    )
                })?;

            if wire_articulation && section.articulation {
                attach_urdf_articulation(world, &urdf, &spawned, section.to_articulation_config())
                    .map_err(|error| {
                        AssetError::invalid(
                            asset_path.display().to_string(),
                            format!("urdf articulation failed: {error}"),
                        )
                    })?;
            }

            world
                .entity_mut(spawned.base_link)
                .insert(Transform3::from_translation_rotation(
                    vec3_from_array(section.initial_translation_m),
                    quat_from_rpy(section.initial_rotation_rpy),
                ));

            let wrist_camera = asset.wrist_camera.as_ref().and_then(|config| {
                config.enabled.then(|| {
                    spawn_wrist_camera(
                        world,
                        &spawned.links,
                        config,
                        lidar_stream_index.unwrap_or(0),
                    )
                })
            });

            Ok((
                SpawnedRobot {
                    robot: spawned.robot,
                    base_link: spawned.base_link,
                },
                RobotSensorMounts {
                    lidar: None,
                    wrist_camera,
                },
            ))
        }
    }
}

/// Spawns a scene and its referenced diff-drive robots.
pub fn spawn_scene(
    world: &mut World,
    scene: &SceneAsset,
    robots: &[(PathBuf, RobotAsset)],
) -> Result<SpawnedScene, AssetError> {
    spawn_scene_with_sources(world, scene, robots, None, SpawnSceneOptions::default())
}

/// Spawns a parsed scene using optional embedded URDF sources.
pub fn spawn_scene_with_sources(
    world: &mut World,
    scene: &SceneAsset,
    robots: &[(PathBuf, RobotAsset)],
    urdf_sources: Option<&UrdfSourceMap<'_>>,
    options: SpawnSceneOptions,
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
    world.resource_mut::<WorldRandom>().reset(scene.world.seed);

    if scene.ground.enabled {
        spawn_ground_plane(world);
    }

    for obstacle in &scene.obstacles {
        spawn_scene_obstacle(world, obstacle);
    }

    let mut spawned_robots = Vec::new();
    let mut lidar_mounts = Vec::new();
    let mut wrist_camera_mounts = Vec::new();
    for (index, (robot_path, robot_asset)) in robots.iter().enumerate() {
        let (spawned, mounts) = spawn_robot_asset_with_sources(
            world,
            robot_path,
            robot_asset,
            Some(index),
            urdf_sources,
            options.wire_articulation,
        )
        .map_err(|error| match error {
            AssetError::UnsupportedRobotKind { kind } => AssetError::invalid(
                scene.robots[index].path.clone(),
                format!("robot #{index} kind `{kind}` is not supported by spawn_scene"),
            ),
            other => other,
        })?;
        if let Some(mount) = mounts.lidar {
            lidar_mounts.push(mount);
        }
        if let Some(mount) = mounts.wrist_camera {
            wrist_camera_mounts.push(mount);
        }
        spawned_robots.push((robot_asset.model_name.clone(), spawned));
    }

    Ok(SpawnedScene {
        world: world_entity,
        robots: spawned_robots,
        lidar_mounts,
        wrist_camera_mounts,
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

/// Spawns a parsed scene bundle with optional embedded URDF sources.
pub fn spawn_scene_bundle(
    world: &mut World,
    bundle: &crate::pipeline::SceneAssetBundle,
    urdf_sources: Option<&UrdfSourceMap<'_>>,
    options: SpawnSceneOptions,
) -> Result<SpawnedScene, AssetError> {
    spawn_scene_with_sources(world, &bundle.scene, &bundle.robots, urdf_sources, options)
}

fn load_urdf_robot(
    urdf_path: &Path,
    urdf_sources: Option<&UrdfSourceMap<'_>>,
) -> Result<UrdfRobot, rne_urdf_import::UrdfParseError> {
    if let Some(sources) = urdf_sources {
        if let Some(xml) = sources.get(urdf_path) {
            return parse_urdf(xml);
        }
    }
    parse_urdf_file(urdf_path)
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

fn spawn_scene_obstacle(world: &mut World, obstacle: &SceneObstacleAsset) -> Entity {
    let entity = spawn_named(world, &obstacle.name);
    let body_type = match obstacle.body_type {
        ObstacleBodyType::Fixed => RigidBodyType::Fixed,
        ObstacleBodyType::Dynamic => RigidBodyType::Dynamic,
    };
    world.entity_mut(entity).insert((
        RigidBody {
            body_type,
            mass_kg: obstacle.mass_kg,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: vec3_from_array(obstacle.half_extents_m),
            },
            material: PhysicsMaterial {
                friction: obstacle
                    .friction
                    .unwrap_or_else(|| PhysicsMaterial::default().friction),
                ..PhysicsMaterial::default()
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            vec3_from_array(obstacle.translation_m),
            Quat::IDENTITY,
        ),
    ));
    entity
}

fn sync_sensor_mount(world: &mut World, parent_link: Entity, sensor: Entity, offset_m: Vec3) {
    let parent = world_transform_of(world, parent_link);
    if let Some(mut sensor_tf) = world.get_mut::<Transform3>(sensor) {
        sensor_tf.translation = parent.translation + parent.rotation * offset_m;
        sensor_tf.rotation = parent.rotation;
    }
}

fn sync_lidar_mount(world: &mut World, base_link: Entity, lidar: Entity, offset_m: Vec3) {
    sync_sensor_mount(world, base_link, lidar, offset_m);
}

fn spawn_wrist_camera(
    world: &mut World,
    links: &HashMap<String, Entity>,
    config: &crate::robot::WristCameraRobotAsset,
    stream_index: usize,
) -> WristCameraMountSpawned {
    let parent_link = *links
        .get(&config.mount_link)
        .unwrap_or_else(|| panic!("missing wrist camera mount link `{}`", config.mount_link));
    let offset_m = config.mount_offset();
    let camera = spawn_named(world, "wrist_camera");
    world.entity_mut(camera).insert((
        Sensor {
            kind: SensorKind::Camera(config.to_spec()),
            update_rate_hz: config.update_rate_hz,
            latency_ticks: 0,
            frame_id: 12,
            enabled: true,
            stream_id: StreamId::new(WRIST_CAMERA_STREAM_BASE as u64 + stream_index as u64),
        },
        SensorState::default(),
        Transform3::IDENTITY,
    ));
    sync_sensor_mount(world, parent_link, camera, offset_m);
    WristCameraMountSpawned {
        parent_link,
        camera,
        mount_offset_m: offset_m,
    }
}

fn spawn_robot_lidar(
    world: &mut World,
    base_link: Entity,
    config: &LidarRobotAsset,
    stream_index: usize,
) -> LidarMountSpawned {
    let offset_m = config.mount_offset();
    let lidar = spawn_named(world, "lidar");
    world.entity_mut(lidar).insert((
        Sensor {
            kind: SensorKind::Lidar(config.to_spec()),
            update_rate_hz: config.update_rate_hz,
            latency_ticks: 0,
            frame_id: 11,
            enabled: true,
            stream_id: StreamId::new(LIDAR_STREAM_BASE as u64 + stream_index as u64),
        },
        SensorState::default(),
        Transform3::IDENTITY,
    ));
    sync_lidar_mount(world, base_link, lidar, offset_m);
    LidarMountSpawned {
        base_link,
        lidar,
        mount_offset_m: offset_m,
    }
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

fn quat_from_rpy(rpy: [f64; 3]) -> Quat {
    Quat::from_rotation_z(rpy[2]) * Quat::from_rotation_y(rpy[1]) * Quat::from_rotation_x(rpy[0])
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
    use super::collect_robot_links;
    use crate::{load_and_spawn_scene, spawn_robot_asset};
    use rne_ecs::World;
    use rne_physics::RigidBody;
    use rne_robot::Link;
    use rne_sensor::Sensor;
    use rne_world::{WorldEntity, WorldRandom};
    use std::path::Path;

    #[test]
    fn spawn_scene_from_fixture() {
        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/episode_diff_drive.rne.scene.toml");
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, &scene_path).unwrap();

        assert!(world.get::<WorldEntity>(spawned.world).is_some());
        assert_eq!(world.resource::<WorldRandom>().seed(), 42);
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
        let (spawned, _) = spawn_robot_asset(&mut world, &robot_path, &asset, None).unwrap();
        assert!(world.get::<rne_render::Visual>(spawned.base_link).is_some());
    }

    #[test]
    fn spawn_scene_with_lidar_and_obstacle() {
        let scene_text = r#"
[world]
seed = 7

[ground]
enabled = true

[[robots]]
path = "diff_drive.rne.robot.toml"

[[obstacles]]
name = "front_wall"
translation_m = [0.0, 1.0, 8.0]
half_extents_m = [8.0, 1.0, 0.25]
"#;
        let robot_text = r#"
kind = "diff_drive"
model_name = "diff_drive"

[diff_drive]

[lidar]
ray_count = 64
"#;
        let dir = std::env::temp_dir().join(format!("rne_assets_lidar_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let robot_path = dir.join("diff_drive.rne.robot.toml");
        let scene_path = dir.join("scene.rne.scene.toml");
        std::fs::write(&robot_path, robot_text).unwrap();
        std::fs::write(&scene_path, scene_text).unwrap();

        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, &scene_path).unwrap();
        assert_eq!(spawned.lidar_mounts.len(), 1);
        assert!(world.get::<Sensor>(spawned.lidar_mounts[0].lidar).is_some());
        let mut names = world.query::<&rne_ecs::Name>();
        assert!(names.iter(&world).any(|name| name.0 == "front_wall"));

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn obstacle_friction_field_overrides_collider_material() {
        use crate::scene::{parse_scene_asset, ObstacleBodyType};
        use rne_physics::{Collider, PhysicsMaterial};

        let text = r#"
[[obstacles]]
name = "slick_cube"
translation_m = [0.5, 0.4, 0.0]
half_extents_m = [0.03, 0.03, 0.03]
body_type = "dynamic"
mass_kg = 0.05
friction = 0.02
"#;
        let scene = parse_scene_asset(text, Path::new("scene.toml")).unwrap();
        assert_eq!(scene.obstacles[0].body_type, ObstacleBodyType::Dynamic);
        assert_eq!(scene.obstacles[0].friction, Some(0.02));

        let mut world = World::new();
        let entity = super::spawn_scene_obstacle(&mut world, &scene.obstacles[0]);
        let collider = world.get::<Collider>(entity).expect("collider");
        assert!((collider.material.friction - 0.02).abs() < f32::EPSILON);

        // An obstacle with no `friction` key keeps the engine default.
        let default_text = r#"
[[obstacles]]
name = "plain_cube"
translation_m = [0.0, 0.4, 0.0]
half_extents_m = [0.03, 0.03, 0.03]
"#;
        let default_scene = parse_scene_asset(default_text, Path::new("scene.toml")).unwrap();
        assert_eq!(default_scene.obstacles[0].friction, None);
        let default_entity = super::spawn_scene_obstacle(&mut world, &default_scene.obstacles[0]);
        let default_collider = world.get::<Collider>(default_entity).expect("collider");
        assert!(
            (default_collider.material.friction - PhysicsMaterial::default().friction).abs()
                < f32::EPSILON
        );
    }

    #[test]
    fn urdf_robot_asset_spawns_without_articulation() {
        let robot_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/diff_drive_urdf.rne.robot.toml");
        let asset = crate::robot::load_robot_asset(&robot_path).unwrap();
        let mut world = World::new();
        let (spawned, mounts) = spawn_robot_asset(&mut world, &robot_path, &asset, None).unwrap();
        assert!(mounts.lidar.is_none());
        assert!(mounts.wrist_camera.is_none());
        assert!(world.get::<RigidBody>(spawned.base_link).is_some());
        assert!(world.get::<Link>(spawned.base_link).is_some());
    }

    #[test]
    fn urdf_robot_asset_spawns_with_articulation() {
        let robot_path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mm_mobile.rne.robot.toml");
        let asset = crate::robot::load_robot_asset(&robot_path).unwrap();
        let mut world = World::new();
        let (spawned, _) = spawn_robot_asset(&mut world, &robot_path, &asset, None).unwrap();
        let links = collect_robot_links(&mut world, spawned.robot);
        let left_wheel = links["left_wheel"];
        assert!(world.get::<rne_physics::JointMotor>(left_wheel).is_some());
    }
}
