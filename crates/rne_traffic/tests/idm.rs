use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_kinematic_traffic, CarFollowingModel, IdmParams, KinematicTrafficConfig,
    KinematicTrafficError, TrafficActor, TrafficId, TrafficPose, TrafficRoute, TrafficRouteCatalog,
    TrafficRouteFollower, TrafficRuntime,
};
use uuid::Uuid;

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn idm_config() -> KinematicTrafficConfig {
    KinematicTrafficConfig {
        car_following: CarFollowingModel::Idm(IdmParams::default()),
        ..KinematicTrafficConfig::default()
    }
}

fn spawn_follower(
    world: &mut World,
    route_id: &TrafficId,
    uuid: u128,
    distance_m: f64,
    speed_m_s: f64,
    desired_speed_m_s: f64,
) -> bevy_ecs::entity::Entity {
    world
        .spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(uuid)),
            TrafficRouteFollower {
                route_id: route_id.clone(),
                distance_m,
                speed_m_s,
                desired_speed_m_s,
                length_m: 4.0,
            },
            TrafficPose {
                position_m: [distance_m, 0.0, 0.0],
                yaw_rad: 0.0,
            },
        ))
        .id()
}

fn straight_route() -> (TrafficId, TrafficRouteCatalog) {
    let route_id = id("route:idm");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
        false,
    )
    .expect("route");
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route).expect("insert route");
    (route_id, routes)
}

#[test]
fn idm_accelerates_a_lone_actor_in_free_flow() {
    let (route_id, routes) = straight_route();
    let mut world = World::new();
    let actor = spawn_follower(&mut world, &route_id, 1, 0.0, 0.0, 10.0);
    let mut runtime = TrafficRuntime::default();

    advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(100_000_000),
        SimDuration::from_ticks(100_000_000),
        idm_config(),
    )
    .expect("step");

    let speed = world.get::<TrafficRouteFollower>(actor).unwrap().speed_m_s;
    assert!(speed > 0.0, "free-flow IDM must accelerate, got {speed}");
    assert!(speed <= 10.0);
}

#[test]
fn idm_brakes_for_a_stopped_leader() {
    let (route_id, routes) = straight_route();
    let mut world = World::new();
    let follower = spawn_follower(&mut world, &route_id, 1, 0.0, 5.0, 10.0);
    // Leader 8 m ahead with zero speed: bumper gap is 8 - (2 + 2) = 4 m.
    spawn_follower(&mut world, &route_id, 2, 8.0, 0.0, 0.0);
    let mut runtime = TrafficRuntime::default();

    advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(100_000_000),
        SimDuration::from_ticks(100_000_000),
        idm_config(),
    )
    .expect("step");

    let speed = world
        .get::<TrafficRouteFollower>(follower)
        .unwrap()
        .speed_m_s;
    assert!(
        speed < 5.0,
        "IDM must brake for a close stopped leader, got {speed}"
    );
    assert!(speed >= 0.0);
}

#[test]
fn idm_replay_is_deterministic() {
    let run = || {
        let (route_id, routes) = straight_route();
        let mut world = World::new();
        let follower = spawn_follower(&mut world, &route_id, 1, 0.0, 6.0, 12.0);
        spawn_follower(&mut world, &route_id, 2, 12.0, 2.0, 8.0);
        let mut runtime = TrafficRuntime::default();
        for step in 1..=20 {
            let time = SimTime::from_ticks(step * 100_000_000);
            advance_kinematic_traffic(
                &mut world,
                &routes,
                &mut runtime,
                time,
                SimDuration::from_ticks(100_000_000),
                idm_config(),
            )
            .expect("step");
        }
        let follower = world.get::<TrafficRouteFollower>(follower).unwrap();
        (follower.speed_m_s, follower.distance_m)
    };
    assert_eq!(run(), run());
}

#[test]
fn invalid_idm_config_is_rejected() {
    let (_route_id, routes) = straight_route();
    let mut world = World::new();
    let mut runtime = TrafficRuntime::default();
    let config = KinematicTrafficConfig {
        car_following: CarFollowingModel::Idm(IdmParams {
            max_acceleration_m_s2: -1.0,
            ..IdmParams::default()
        }),
        ..KinematicTrafficConfig::default()
    };
    let error = advance_kinematic_traffic(
        &mut world,
        &routes,
        &mut runtime,
        SimTime::from_ticks(100_000_000),
        SimDuration::from_ticks(100_000_000),
        config,
    )
    .expect_err("invalid IDM params must be rejected");
    assert!(matches!(error, KinematicTrafficError::InvalidConfig { .. }));
}
