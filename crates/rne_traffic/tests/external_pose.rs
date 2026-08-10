use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_kinematic_traffic, KinematicTrafficConfig, TrafficActor, TrafficId, TrafficPose,
    TrafficPoseSource, TrafficRoute, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime,
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
