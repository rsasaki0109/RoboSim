//! LiDAR sensor helpers for diff-drive simulation and rendering.

use rne_data::StreamId;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_robot::DiffDriveSpawned;
use rne_sensor::{LidarSpec, Sensor, SensorKind, SensorState};
use rne_world::Transform3;

const LIDAR_STREAM_BASE: u32 = 200;

/// Returns the DataBus stream id for a robot's LiDAR sensor.
pub fn lidar_stream_for_index(index: usize) -> StreamId {
    StreamId::new(LIDAR_STREAM_BASE as u64 + index as u64)
}

/// Spawns a fixed wall obstacle useful for LiDAR demo scenes.
pub fn spawn_lidar_demo_wall(world: &mut World) {
    let wall = spawn_named(world, "lidar_demo_wall");
    world.entity_mut(wall).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(8.0, 1.0, 0.25),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, 1.0, 8.0), Quat::IDENTITY),
    ));
}

/// Attaches a horizontal LiDAR sensor for the given robot base link.
pub fn attach_lidar_sensor(world: &mut World, base_link: Entity, stream_id: StreamId) -> Entity {
    let lidar = spawn_named(world, "lidar");
    world.entity_mut(lidar).insert((
        Sensor {
            kind: SensorKind::Lidar(LidarSpec {
                ray_count: 120,
                max_range_m: 15.0,
                ..LidarSpec::default()
            }),
            update_rate_hz: 10.0,
            latency_ticks: 0,
            frame_id: 11,
            enabled: true,
            stream_id,
        },
        SensorState::default(),
        Transform3::IDENTITY,
    ));
    sync_lidar_mount(world, base_link, lidar);
    lidar
}

/// Copies the robot base pose onto a free-floating LiDAR mount entity.
pub fn sync_lidar_mount(world: &mut World, base_link: Entity, lidar: Entity) {
    let Some(base) = world.get::<Transform3>(base_link).copied() else {
        return;
    };
    if let Some(mut lidar_tf) = world.get_mut::<Transform3>(lidar) {
        lidar_tf.translation = base.translation + base.rotation * Vec3::new(0.0, 0.2, 0.0);
        lidar_tf.rotation = base.rotation;
    }
}

/// Syncs every tracked LiDAR mount before sensor sampling.
pub fn sync_lidar_mounts(world: &mut World, mounts: &[(Entity, Entity)]) {
    for &(base_link, lidar) in mounts {
        sync_lidar_mount(world, base_link, lidar);
    }
}

/// Registers demo obstacles and LiDAR sensors, then syncs them into physics.
pub fn enable_lidar_demo<B: PhysicsBackend>(
    world: &mut World,
    robots: &[DiffDriveSpawned],
    backend: &mut B,
    physics_world: PhysicsWorldId,
    mounts: &mut Vec<(Entity, Entity)>,
) {
    if !mounts.is_empty() {
        return;
    }

    spawn_lidar_demo_wall(world);
    for (index, robot) in robots.iter().enumerate() {
        let lidar = attach_lidar_sensor(world, robot.base_link, lidar_stream_for_index(index));
        mounts.push((robot.base_link, lidar));
    }

    backend
        .sync_from_ecs(world, physics_world)
        .expect("sync lidar demo into physics");
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::Name;
    use rne_physics::PhysicsWorldDesc;
    use rne_physics_rapier::RapierBackend;
    use rne_robot::{spawn_diff_drive_robot, DiffDriveConfig, DiffDriveSpawned};

    fn spawn_robot(world: &mut World) -> DiffDriveSpawned {
        spawn_diff_drive_robot(
            world,
            &DiffDriveConfig {
                initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
                ..DiffDriveConfig::default()
            },
        )
    }

    #[test]
    fn enable_lidar_demo_attaches_sensor_and_wall() {
        let mut world = World::new();
        let robot = spawn_robot(&mut world);
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .expect("physics world");
        backend.sync_from_ecs(&mut world, physics_world).unwrap();

        let mut mounts = Vec::new();
        enable_lidar_demo(
            &mut world,
            std::slice::from_ref(&robot),
            &mut backend,
            physics_world,
            &mut mounts,
        );

        assert_eq!(mounts.len(), 1);
        let (_, lidar) = mounts[0];
        assert!(world.get::<Sensor>(lidar).is_some());
        let mut names = world.query::<&Name>();
        assert!(names.iter(&world).any(|name| name.0 == "lidar_demo_wall"));
    }
}
