//! LiDAR sensor helpers for diff-drive simulation and rendering.

use rne_assets::LidarMountSpawned;
use rne_data::StreamId;
use rne_ecs::{Entity, World};
use rne_math::Vec3;

const LIDAR_STREAM_BASE: u32 = 200;

/// A LiDAR sensor entity tracked relative to a robot base link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarMount {
    /// Robot base link the LiDAR follows.
    pub base_link: Entity,
    /// LiDAR sensor entity.
    pub lidar: Entity,
    /// Mount offset from the base link origin in meters.
    pub offset_m: Vec3,
}

impl From<LidarMountSpawned> for LidarMount {
    fn from(mount: LidarMountSpawned) -> Self {
        Self {
            base_link: mount.base_link,
            lidar: mount.lidar,
            offset_m: mount.mount_offset_m,
        }
    }
}

/// Returns the DataBus stream id for a robot's LiDAR sensor.
pub fn lidar_stream_for_index(index: usize) -> StreamId {
    StreamId::new(LIDAR_STREAM_BASE as u64 + index as u64)
}

/// Copies the robot base pose onto a free-floating LiDAR mount entity.
pub fn sync_lidar_mount(world: &mut World, base_link: Entity, lidar: Entity, offset_m: Vec3) {
    let Some(base) = world.get::<rne_world::Transform3>(base_link).copied() else {
        return;
    };
    if let Some(mut lidar_tf) = world.get_mut::<rne_world::Transform3>(lidar) {
        lidar_tf.translation = base.translation + base.rotation * offset_m;
        lidar_tf.rotation = base.rotation;
    }
}

/// Syncs every tracked LiDAR mount before sensor sampling.
pub fn sync_lidar_mounts(world: &mut World, mounts: &[LidarMount]) {
    for mount in mounts {
        sync_lidar_mount(world, mount.base_link, mount.lidar, mount.offset_m);
    }
}

/// Collects LiDAR mounts for the given robots from asset spawn metadata.
pub fn lidar_mounts_from_spawned(spawned: &[LidarMountSpawned]) -> Vec<LidarMount> {
    spawned.iter().copied().map(LidarMount::from).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_assets::{load_and_spawn_scene, parse_robot_asset, spawn_robot_asset};
    use rne_ecs::World;
    use rne_robot::{spawn_diff_drive_robot, DiffDriveConfig};
    use rne_sensor::Sensor;
    use std::path::Path;

    #[test]
    fn scene_asset_spawns_lidar_mount() {
        let scene_text = r#"
[ground]
enabled = true

[[robots]]
path = "robot.rne.robot.toml"
"#;
        let robot_text = r#"
kind = "diff_drive"
model_name = "diff_drive"

[diff_drive]

[lidar]
"#;
        let dir = std::env::temp_dir().join(format!("rne_ai_lidar_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("robot.rne.robot.toml"), robot_text).unwrap();
        let scene_path = dir.join("scene.rne.scene.toml");
        std::fs::write(&scene_path, scene_text).unwrap();

        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, &scene_path).unwrap();
        let mounts = lidar_mounts_from_spawned(&spawned.lidar_mounts);
        assert_eq!(mounts.len(), 1);
        assert!(world.get::<Sensor>(mounts[0].lidar).is_some());

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn sync_lidar_mount_follows_base_motion() {
        let mut world = World::new();
        let robot = spawn_diff_drive_robot(
            &mut world,
            &DiffDriveConfig {
                initial_translation_m: Vec3::new(1.0, 0.25, 0.0),
                ..DiffDriveConfig::default()
            },
        );
        let (_, mount) = spawn_robot_asset(
            &mut world,
            Path::new("robot.toml"),
            &parse_robot_asset(
                r#"
kind = "diff_drive"
model_name = "diff_drive"
[diff_drive]
[lidar]
"#,
                Path::new("robot.toml"),
            )
            .unwrap(),
            Some(0),
        )
        .unwrap();
        let mount = mount.lidar.expect("lidar mount");
        let offset = mount.mount_offset_m;
        sync_lidar_mount(&mut world, robot.base_link, mount.lidar, offset);
        let lidar_y = world
            .get::<rne_world::Transform3>(mount.lidar)
            .unwrap()
            .translation
            .y;
        assert!((lidar_y - 0.45).abs() < 1e-6);
    }
}
