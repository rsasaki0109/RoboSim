//! Deterministic traffic runtime systems.

use crate::{TrafficActor, TrafficRuntime, TrafficStepCompleted};
use bevy_ecs::prelude::{With, World};
use rne_core::SimTime;
use rne_ecs::EntityUuid;
use std::error::Error;
use std::fmt;

/// Indicates that externally visible traffic behavior cannot be ordered stably.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissingTrafficActorStableId {
    /// Number of traffic actors that do not have an [`EntityUuid`].
    pub actor_count: usize,
}

impl fmt::Display for MissingTrafficActorStableId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{} traffic actor(s) are missing EntityUuid",
            self.actor_count
        )
    }
}

impl Error for MissingTrafficActorStableId {}

/// Returns traffic actor UUIDs in canonical byte order.
///
/// Returns an error instead of silently accepting an actor without an
/// [`EntityUuid`], because ECS insertion order is not a stable external
/// identity.
pub fn traffic_actors_in_stable_order(
    world: &mut World,
) -> Result<Vec<EntityUuid>, MissingTrafficActorStableId> {
    let mut query = world.query_filtered::<Option<&EntityUuid>, With<TrafficActor>>();
    let mut missing_count = 0;
    let mut actor_ids = Vec::new();
    for id in query.iter(world) {
        if let Some(id) = id {
            actor_ids.push(*id);
        } else {
            missing_count += 1;
        }
    }
    if missing_count != 0 {
        return Err(MissingTrafficActorStableId {
            actor_count: missing_count,
        });
    }
    actor_ids.sort_unstable_by_key(|id| id.0.as_u128());
    Ok(actor_ids)
}

/// Advances the per-world traffic step counter using an explicit simulation time.
pub fn advance_traffic_step(
    runtime: &mut TrafficRuntime,
    sim_time: SimTime,
) -> TrafficStepCompleted {
    TrafficStepCompleted {
        step_index: runtime.advance(),
        sim_time,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_core::SimDuration;
    use uuid::Uuid;

    fn id(value: u128) -> EntityUuid {
        EntityUuid(Uuid::from_u128(value))
    }

    #[test]
    fn actor_order_is_independent_of_spawn_order() {
        let mut world = World::new();
        world.spawn((TrafficActor::motor_vehicle(), id(30)));
        world.spawn((TrafficActor::motor_vehicle(), id(10)));
        world.spawn((TrafficActor::motor_vehicle(), id(20)));

        assert_eq!(
            traffic_actors_in_stable_order(&mut world).expect("stable IDs"),
            vec![id(10), id(20), id(30)]
        );
    }

    #[test]
    fn actor_without_stable_id_is_rejected() {
        let mut world = World::new();
        world.spawn((TrafficActor::motor_vehicle(), id(10)));
        world.spawn(TrafficActor::motor_vehicle());

        assert_eq!(
            traffic_actors_in_stable_order(&mut world),
            Err(MissingTrafficActorStableId { actor_count: 1 })
        );
    }

    #[test]
    fn traffic_steps_use_explicit_simulation_time() {
        let mut runtime = TrafficRuntime::default();
        let sim_time = SimTime::ZERO + SimDuration::from_ticks(16_666_666);

        let event = advance_traffic_step(&mut runtime, sim_time);

        assert_eq!(event.step_index, 1);
        assert_eq!(event.sim_time, sim_time);
        assert_eq!(runtime.step_index(), 1);
    }
}
