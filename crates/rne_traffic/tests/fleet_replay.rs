use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_kinematic_traffic, KinematicTrafficConfig, TrafficActor, TrafficId, TrafficPose,
    TrafficRoute, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime,
};
use uuid::Uuid;

const ACTOR_COUNT: usize = 100;
const STEP_COUNT: u64 = 600;

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn city_loop() -> TrafficRoute {
    TrafficRoute::new(
        id("route:city-loop"),
        vec![
            [0.0, 0.0, 0.0],
            [500.0, 0.0, 0.0],
            [500.0, 0.0, 500.0],
            [0.0, 0.0, 500.0],
        ],
        true,
    )
    .expect("city loop")
}

fn replay(reverse_spawn_order: bool) -> (u64, f64, World) {
    let mut catalog = TrafficRouteCatalog::default();
    catalog.insert(city_loop()).expect("insert route");
    let mut world = World::new();
    let indices: Vec<_> = if reverse_spawn_order {
        (0..ACTOR_COUNT).rev().collect()
    } else {
        (0..ACTOR_COUNT).collect()
    };
    for index in indices {
        let distance_m = index as f64 * 20.0;
        let sample = catalog
            .get(&id("route:city-loop"))
            .expect("route")
            .sample(distance_m);
        world.spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(index as u128 + 1)),
            TrafficRouteFollower {
                route_id: id("route:city-loop"),
                distance_m,
                speed_m_s: 0.0,
                desired_speed_m_s: 10.0 + (index % 5) as f64 * 0.25,
                length_m: 4.4,
            },
            TrafficPose {
                position_m: sample.position_m,
                yaw_rad: sample.yaw_rad,
            },
        ));
    }

    let delta = SimDuration::from_ticks(16_666_666);
    let mut runtime = TrafficRuntime::default();
    let mut final_step = None;
    for step in 1..=STEP_COUNT {
        final_step = Some(
            advance_kinematic_traffic(
                &mut world,
                &catalog,
                &mut runtime,
                SimTime::from_ticks(step * delta.ticks()),
                delta,
                KinematicTrafficConfig::default(),
            )
            .expect("traffic step"),
        );
    }
    let final_step = final_step.expect("at least one step");
    (
        final_step.stable_state_hash,
        final_step.minimum_observed_gap_m.expect("closed-loop gaps"),
        world,
    )
}

#[test]
fn one_hundred_vehicle_replay_is_spawn_order_independent() {
    let (forward_hash, forward_gap_m, forward_world) = replay(false);
    let (reverse_hash, reverse_gap_m, reverse_world) = replay(true);

    assert_eq!(forward_hash, reverse_hash);
    assert_eq!(forward_hash, 5_765_881_651_073_142_143);
    assert_eq!(forward_gap_m.to_bits(), reverse_gap_m.to_bits());
    assert!(forward_gap_m >= KinematicTrafficConfig::default().minimum_gap_m);
    assert_eq!(
        forward_world
            .iter_entities()
            .filter(|entity| entity.contains::<TrafficActor>())
            .count(),
        ACTOR_COUNT
    );
    assert_eq!(
        reverse_world
            .iter_entities()
            .filter(|entity| entity.contains::<TrafficActor>())
            .count(),
        ACTOR_COUNT
    );
}

#[test]
fn closed_route_samples_every_segment_and_wraps() {
    let route = city_loop();
    assert_eq!(route.total_length_m(), 2_000.0);
    assert_eq!(route.sample(0.0).position_m, [0.0, 0.0, 0.0]);
    assert_eq!(route.sample(750.0).position_m, [500.0, 0.0, 250.0]);
    assert_eq!(route.sample(1_750.0).position_m, [0.0, 0.0, 250.0]);
    assert_eq!(route.sample(2_050.0).position_m, [50.0, 0.0, 0.0]);
}
