use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_kinematic_traffic, KinematicTrafficConfig, KinematicTrafficError, TrafficActor,
    TrafficId, TrafficOwnershipMetrics, TrafficPose, TrafficPoseSource, TrafficRoute,
    TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime,
};
use uuid::Uuid;

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

#[test]
fn external_pose_actors_are_left_to_their_adapter() {
    let route_id = id("route:runtime");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        false,
    )
    .expect("route");
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route).expect("insert route");

    let mut world = World::new();
    let external_pose = [42.0, 0.0, -7.0];
    let external = world
        .spawn((
            TrafficActor::motor_vehicle(),
            TrafficPoseSource::External,
            EntityUuid(Uuid::from_u128(1)),
            TrafficPose {
                position_m: external_pose,
                yaw_rad: 0.25,
            },
        ))
        .id();
    let runtime_actor = world
        .spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(2)),
            TrafficRouteFollower {
                route_id: route_id.clone(),
                distance_m: 0.0,
                speed_m_s: 0.0,
                desired_speed_m_s: 5.0,
                length_m: 4.0,
            },
            TrafficPose {
                position_m: [0.0, 0.0, 0.0],
                yaw_rad: 0.0,
            },
        ))
        .id();

    let mut runtime = TrafficRuntime::default();
    let report = advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(500_000_000),
        SimDuration::from_ticks(500_000_000),
        KinematicTrafficConfig::default(),
    )
    .expect("advance runtime-owned actor");

    assert_eq!(report.actor_count, 1);
    assert_eq!(
        report.ownership,
        TrafficOwnershipMetrics {
            total_actor_count: 2,
            runtime_owned_actor_count: 1,
            external_owned_actor_count: 1,
            runtime_advanced_actor_count: 1,
            external_observed_actor_count: 1,
            invalid_actor_count: 0,
        }
    );
    assert_eq!(runtime.step_index(), 1);
    assert_eq!(
        world
            .get::<TrafficPose>(external)
            .expect("external pose")
            .position_m,
        external_pose
    );
    assert!(
        world
            .get::<TrafficRouteFollower>(runtime_actor)
            .expect("runtime follower")
            .distance_m
            > 0.0
    );
}

fn mixed_world_digest(reverse_spawn_order: bool, external_x_m: f64) -> (u64, [f64; 3]) {
    let route_id = id("route:mixed");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        false,
    )
    .expect("route");
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route).expect("insert route");
    let mut world = World::new();
    let spawn_external = |world: &mut World| {
        world.spawn((
            TrafficActor::motor_vehicle(),
            TrafficPoseSource::External,
            EntityUuid(Uuid::from_u128(20)),
            TrafficPose {
                position_m: [external_x_m, 0.0, -2.0],
                yaw_rad: 0.5,
            },
        ));
    };
    let spawn_runtime = |world: &mut World| {
        world.spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(10)),
            TrafficRouteFollower {
                route_id: route_id.clone(),
                distance_m: 0.0,
                speed_m_s: 0.0,
                desired_speed_m_s: 5.0,
                length_m: 4.0,
            },
            TrafficPose {
                position_m: [0.0, 0.0, 0.0],
                yaw_rad: 0.0,
            },
        ));
    };
    if reverse_spawn_order {
        spawn_external(&mut world);
        spawn_runtime(&mut world);
    } else {
        spawn_runtime(&mut world);
        spawn_external(&mut world);
    }
    let mut runtime = TrafficRuntime::default();
    let report = advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(500_000_000),
        SimDuration::from_ticks(500_000_000),
        KinematicTrafficConfig::default(),
    )
    .expect("mixed step");
    let external_pose = world
        .iter_entities()
        .find_map(|entity| {
            let id = entity.get::<EntityUuid>()?;
            (id.0.as_u128() == 20).then(|| entity.get::<TrafficPose>().expect("pose").position_m)
        })
        .expect("external actor");
    (report.externally_visible_state_hash, external_pose)
}

#[test]
fn mixed_ownership_digest_is_spawn_order_independent_and_observes_external_pose() {
    let forward = mixed_world_digest(false, 42.0);
    let reverse = mixed_world_digest(true, 42.0);
    let moved_external = mixed_world_digest(false, 43.0);

    assert_eq!(forward, reverse);
    assert_eq!(forward.1, [42.0, 0.0, -2.0]);
    assert_ne!(forward.0, moved_external.0);
}

#[test]
fn invalid_external_pose_fails_before_runtime_mutation() {
    let route_id = id("route:transactional");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        false,
    )
    .expect("route");
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route).expect("insert route");
    let mut world = World::new();
    world.spawn((
        TrafficActor::motor_vehicle(),
        TrafficPoseSource::External,
        EntityUuid(Uuid::from_u128(1)),
        TrafficPose {
            position_m: [f64::NAN, 0.0, 0.0],
            yaw_rad: 0.0,
        },
    ));
    let runtime_actor = world
        .spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(2)),
            TrafficRouteFollower {
                route_id,
                distance_m: 3.0,
                speed_m_s: 1.0,
                desired_speed_m_s: 5.0,
                length_m: 4.0,
            },
            TrafficPose {
                position_m: [3.0, 0.0, 0.0],
                yaw_rad: 0.0,
            },
        ))
        .id();
    let mut runtime = TrafficRuntime::default();

    let error = advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(500_000_000),
        SimDuration::from_ticks(500_000_000),
        KinematicTrafficConfig::default(),
    )
    .expect_err("invalid external pose");

    assert_eq!(error, KinematicTrafficError::InvalidActorState { uuid: 1 });
    assert_eq!(runtime.step_index(), 0);
    assert_eq!(
        world
            .get::<TrafficRouteFollower>(runtime_actor)
            .expect("runtime follower")
            .distance_m,
        3.0
    );
}
