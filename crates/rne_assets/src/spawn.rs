//! Spawn ECS entities from parsed assets.

use crate::error::AssetError;
use crate::robot::{LidarRobotAsset, RobotAsset, RobotKind};
use crate::scene::{
    ObstacleBodyType, SceneAsset, SceneCollisionAsset, SceneDeformableAsset,
    SceneDeformableMaterialAsset, SceneObjectAsset, SceneObstacleAsset, SceneTaskMarkerAsset,
    SceneVisualAsset,
};
use rne_data::StreamId;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, PhysicsMaterial, RigidBody, RigidBodyType};
use rne_render::{Visual, VisualShape};
use rne_robot::{spawn_diff_drive_robot, DiffDriveSpawned, Link};
use rne_sensor::{Sensor, SensorKind, SensorState};
use rne_urdf_import::{
    attach_urdf_articulation, attach_urdf_visuals, parse_urdf, parse_urdf_file,
    spawn_urdf_robot_with_config, UrdfRobot,
};
use rne_world::{
    spawn_world, world_transform_of, Gravity, TaskMarker, Transform3, WorldEntity, WorldRandom,
};
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
    /// Cable and cloth entities in scene declaration order.
    pub deformables: Vec<Entity>,
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
    for object in &scene.objects {
        spawn_scene_object(world, object);
    }
    let deformables = scene
        .deformables
        .iter()
        .map(|asset| spawn_scene_deformable(world, asset))
        .collect::<Result<Vec<_>, _>>()?;
    for marker in &scene.task_markers {
        spawn_scene_task_marker(world, marker);
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
        deformables,
    })
}

fn spawn_scene_deformable(
    world: &mut World,
    asset: &SceneDeformableAsset,
) -> Result<Entity, AssetError> {
    use rne_deformable::{build_cable, build_cloth, CableSpec, ClothSpec};
    let (body, color_rgba) = match asset {
        SceneDeformableAsset::Cable {
            start_m,
            end_m,
            particle_count,
            total_mass_kg,
            pin_start,
            pin_end,
            material,
            color_rgba,
            ..
        } => (
            build_cable(CableSpec {
                start_m: vec3_from_array(*start_m),
                end_m: vec3_from_array(*end_m),
                particle_count: *particle_count,
                total_mass_kg: *total_mass_kg,
                pin_start: *pin_start,
                pin_end: *pin_end,
                material: deformable_material(*material),
            }),
            *color_rgba,
        ),
        SceneDeformableAsset::Cloth {
            origin_m,
            width_direction_m,
            height_direction_m,
            columns,
            rows,
            total_mass_kg,
            pin_top_edge,
            material,
            color_rgba,
            ..
        } => (
            build_cloth(ClothSpec {
                origin_m: vec3_from_array(*origin_m),
                width_direction_m: vec3_from_array(*width_direction_m),
                height_direction_m: vec3_from_array(*height_direction_m),
                columns: *columns,
                rows: *rows,
                total_mass_kg: *total_mass_kg,
                pin_top_edge: *pin_top_edge,
                material: deformable_material(*material),
            }),
            *color_rgba,
        ),
    };
    let body = body.map_err(|error| AssetError::invalid(asset.name(), error.to_string()))?;
    let entity = spawn_named(world, asset.name());
    world
        .entity_mut(entity)
        .insert((body, rne_deformable::DeformableVisual { color_rgba }));
    Ok(entity)
}

fn deformable_material(asset: SceneDeformableMaterialAsset) -> rne_deformable::DeformableMaterial {
    rne_deformable::DeformableMaterial {
        collision_radius_m: asset.collision_radius_m,
        structural_compliance_m_n: asset.structural_compliance_m_n,
        shear_compliance_m_n: asset.shear_compliance_m_n,
        bending_compliance_m_n: asset.bending_compliance_m_n,
        velocity_retention_per_s: asset.velocity_retention_per_s,
        self_collision: asset.self_collision,
    }
}

fn spawn_scene_task_marker(world: &mut World, marker: &SceneTaskMarkerAsset) -> Entity {
    let entity = spawn_named(world, &marker.name);
    world.entity_mut(entity).insert((
        TaskMarker {
            kind: marker.kind.clone(),
            radius_m: marker.radius_m,
        },
        Transform3::from_translation_rotation(
            vec3_from_array(marker.translation_m),
            Quat::IDENTITY,
        ),
    ));
    entity
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

fn spawn_scene_object(world: &mut World, object: &SceneObjectAsset) -> Entity {
    let entity = spawn_named(world, &object.name);
    let body_type = match object.body_type {
        ObstacleBodyType::Fixed => RigidBodyType::Fixed,
        ObstacleBodyType::Dynamic => RigidBodyType::Dynamic,
    };
    let rotation = quat_from_rpy(object.rotation_rpy_rad);
    world.entity_mut(entity).insert((
        RigidBody {
            body_type,
            mass_kg: object.mass_kg,
            ..RigidBody::default()
        },
        Transform3::from_translation_rotation(vec3_from_array(object.translation_m), rotation),
    ));

    if let Some(collision) = object.collision {
        let shape = match collision {
            SceneCollisionAsset::Box { size_m } => ColliderShape::Cuboid {
                half_extents_m: vec3_from_array(size_m) * 0.5,
            },
            SceneCollisionAsset::Sphere { radius_m } => ColliderShape::Sphere { radius_m },
            SceneCollisionAsset::Capsule {
                half_height_m,
                radius_m,
            } => ColliderShape::Capsule {
                half_height_m,
                radius_m,
            },
        };
        let defaults = PhysicsMaterial::default();
        world.entity_mut(entity).insert(Collider {
            shape,
            material: PhysicsMaterial {
                friction: object.friction.unwrap_or(defaults.friction),
                restitution: object.restitution.unwrap_or(defaults.restitution),
            },
            ..Collider::default()
        });
    }

    if let Some(visual) = &object.visual {
        let (shape, color_rgba) = match visual {
            SceneVisualAsset::Box { size_m, color_rgba } => (
                VisualShape::Box {
                    size_m: vec3_from_array(*size_m),
                },
                *color_rgba,
            ),
            SceneVisualAsset::Sphere {
                radius_m,
                color_rgba,
            } => (
                VisualShape::Sphere {
                    radius_m: *radius_m,
                },
                *color_rgba,
            ),
            SceneVisualAsset::Cylinder {
                radius_m,
                length_m,
                color_rgba,
            } => (
                VisualShape::Cylinder {
                    radius_m: *radius_m,
                    length_m: *length_m,
                },
                *color_rgba,
            ),
            SceneVisualAsset::Mesh {
                path,
                scale,
                color_rgba,
            } => (
                VisualShape::Mesh {
                    path: path.clone(),
                    scale: vec3_from_array(*scale),
                },
                *color_rgba,
            ),
        };
        world
            .entity_mut(entity)
            .insert(Visual::new(shape, color_rgba));
    }
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
    use rne_math::{Quat, Vec3};
    use rne_physics::RigidBody;
    use rne_robot::Link;
    use rne_sensor::Sensor;
    use rne_world::{Transform3, WorldEntity, WorldRandom};
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
        assert!(spawned.deformables.is_empty());
        assert!(world
            .get::<RigidBody>(spawned.robots[0].1.base_link)
            .is_some());
    }

    #[test]
    fn cable_scene_spawns_and_replays_headlessly() {
        use rne_deformable::{step_deformable_world, DeformableBody, DeformableSolverConfig};

        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/deformable_cable.rne.scene.toml");
        let mut first = World::new();
        let mut second = World::new();
        let first_spawned =
            load_and_spawn_scene(&mut first, &scene_path).expect("first cable scene");
        let second_spawned =
            load_and_spawn_scene(&mut second, &scene_path).expect("second cable scene");
        assert_eq!(first_spawned.deformables.len(), 1);
        assert_eq!(second_spawned.deformables.len(), 1);

        for _ in 0..180 {
            for world in [&mut first, &mut second] {
                step_deformable_world(
                    world,
                    Vec3::new(0.0, -9.81, 0.0),
                    1.0 / 60.0,
                    DeformableSolverConfig::default(),
                )
                .expect("headless cable step");
            }
        }
        let first_body = first
            .get::<DeformableBody>(first_spawned.deformables[0])
            .expect("first cable");
        let second_body = second
            .get::<DeformableBody>(second_spawned.deformables[0])
            .expect("second cable");
        assert_eq!(
            first_body.stable_state_hash(),
            second_body.stable_state_hash()
        );
        assert_eq!(first_body, second_body);
        let obstacle_center = Vec3::new(0.0, 0.58, 0.0);
        assert!(first_body.particles.iter().all(|particle| {
            particle.position_m.distance(obstacle_center)
                >= 0.16 + first_body.material.collision_radius_m - 1.0e-8
        }));
    }

    #[test]
    fn cloth_scene_drapes_over_box_and_replays_headlessly() {
        use rne_deformable::{step_deformable_world, DeformableBody, DeformableSolverConfig};

        let scene_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/deformable_cloth.rne.scene.toml");
        let mut first = World::new();
        let mut second = World::new();
        let first_spawned =
            load_and_spawn_scene(&mut first, &scene_path).expect("first cloth scene");
        let second_spawned =
            load_and_spawn_scene(&mut second, &scene_path).expect("second cloth scene");
        for _ in 0..300 {
            for world in [&mut first, &mut second] {
                step_deformable_world(
                    world,
                    Vec3::new(0.0, -9.81, 0.0),
                    1.0 / 60.0,
                    DeformableSolverConfig::default(),
                )
                .expect("headless cloth step");
            }
        }
        let first_body = first
            .get::<DeformableBody>(first_spawned.deformables[0])
            .expect("first cloth");
        let second_body = second
            .get::<DeformableBody>(second_spawned.deformables[0])
            .expect("second cloth");
        assert!(first_body.material.self_collision);
        assert_eq!(first_body, second_body);
        assert_eq!(
            first_body.stable_state_hash(),
            second_body.stable_state_hash()
        );
        let center = Vec3::new(0.0, 0.36, 0.0);
        let expanded =
            Vec3::new(0.26, 0.36, 0.21) + Vec3::splat(first_body.material.collision_radius_m);
        assert!(first_body.particles.iter().all(|particle| {
            let local = particle.position_m - center;
            local.x.abs() >= expanded.x - 1.0e-8
                || local.y.abs() >= expanded.y - 1.0e-8
                || local.z.abs() >= expanded.z - 1.0e-8
        }));
        let mesh = first_body.cloth_surface_mesh().expect("cloth mesh");
        assert_eq!(mesh.positions.len(), first_body.particles.len());
        assert!(mesh
            .normals
            .iter()
            .all(|normal| normal.iter().all(|value| value.is_finite())));
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
    fn environment_object_spawns_visual_collision_and_material() {
        use crate::scene::parse_scene_asset;
        use rne_physics::Collider;
        use rne_render::{Visual, VisualShape};

        let scene = parse_scene_asset(
            r#"
[[objects]]
name = "workbench"
translation_m = [1.0, 0.5, -0.25]
rotation_rpy_rad = [0.0, 0.5, 0.0]
body_type = "fixed"
friction = 0.8
restitution = 0.1
visual = { shape = "box", size_m = [1.0, 1.0, 0.5], color_rgba = [0.1, 0.2, 0.3, 1.0] }
collision = { shape = "box", size_m = [1.0, 1.0, 0.5] }
"#,
            Path::new("scene.toml"),
        )
        .unwrap();
        let mut world = World::new();
        let entity = super::spawn_scene_object(&mut world, &scene.objects[0]);
        let visual = world.get::<Visual>(entity).expect("visual");
        assert!(matches!(visual.shape, VisualShape::Box { .. }));
        let collider = world.get::<Collider>(entity).expect("collider");
        assert!((collider.material.friction - 0.8).abs() < f32::EPSILON);
        assert!((collider.material.restitution - 0.1).abs() < f32::EPSILON);
        let transform = world.get::<Transform3>(entity).expect("transform");
        assert_eq!(transform.translation, Vec3::new(1.0, 0.5, -0.25));
        assert_ne!(transform.rotation, Quat::IDENTITY);
    }

    #[test]
    fn rounded_environment_object_spawns_cylinder_visual_and_capsule_collider() {
        use crate::scene::parse_scene_asset;
        use rne_physics::{Collider, ColliderShape};
        use rne_render::{Visual, VisualShape};

        let scene = parse_scene_asset(
            r#"
[[objects]]
name = "safety_post"
visual = { shape = "cylinder", radius_m = 0.08, length_m = 0.9 }
collision = { shape = "capsule", half_height_m = 0.37, radius_m = 0.08 }
"#,
            Path::new("scene.toml"),
        )
        .unwrap();
        let mut world = World::new();
        let entity = super::spawn_scene_object(&mut world, &scene.objects[0]);
        assert!(matches!(
            world.get::<Visual>(entity).expect("visual").shape,
            VisualShape::Cylinder { .. }
        ));
        assert_eq!(
            world.get::<Collider>(entity).expect("collider").shape,
            ColliderShape::Capsule {
                half_height_m: 0.37,
                radius_m: 0.08,
            }
        );
    }

    #[test]
    fn task_marker_spawns_semantic_world_location() {
        let marker = crate::scene::SceneTaskMarkerAsset {
            name: "inspection_a".into(),
            kind: "inspection".into(),
            translation_m: [0.8, 0.0, -0.3],
            radius_m: 0.4,
        };
        let mut world = World::new();
        let entity = super::spawn_scene_task_marker(&mut world, &marker);
        let component = world
            .get::<rne_world::TaskMarker>(entity)
            .expect("task marker");
        assert_eq!(component.kind, "inspection");
        assert_eq!(component.radius_m, 0.4);
        assert_eq!(
            world
                .get::<Transform3>(entity)
                .expect("transform")
                .translation,
            Vec3::new(0.8, 0.0, -0.3)
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
