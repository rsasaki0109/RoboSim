//! Deterministic traffic runtime systems.

use crate::{
    TrafficActor, TrafficPose, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime,
    TrafficStepCompleted,
};
use bevy_ecs::prelude::{Entity, With, World};
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;
use thiserror::Error;

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

/// Deterministic longitudinal-control parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicTrafficConfig {
    /// Maximum acceleration toward free-flow speed.
    pub max_acceleration_m_s2: f64,
    /// Maximum braking magnitude.
    pub max_braking_m_s2: f64,
    /// Minimum bumper-to-bumper gap maintained on a shared route.
    pub minimum_gap_m: f64,
    /// Speed-proportional desired headway.
    pub time_headway_s: f64,
}

impl Default for KinematicTrafficConfig {
    fn default() -> Self {
        Self {
            max_acceleration_m_s2: 2.0,
            max_braking_m_s2: 4.5,
            minimum_gap_m: 2.0,
            time_headway_s: 1.2,
        }
    }
}

/// Summary of one completed kinematic fleet step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicTrafficStep {
    /// Standard traffic step completion metadata.
    pub completed: TrafficStepCompleted,
    /// Number of actors advanced.
    pub actor_count: usize,
    /// Smallest bumper gap observed after the step, when a leader exists.
    pub minimum_observed_gap_m: Option<f64>,
    /// Stable hash of ordered route follower and pose state.
    pub stable_state_hash: u64,
}

/// Kinematic traffic step validation failure.
#[derive(Debug, Error, PartialEq)]
pub enum KinematicTrafficError {
    /// The fixed simulation delta was zero.
    #[error("kinematic traffic step requires a non-zero SimDuration")]
    ZeroDelta,
    /// A numeric configuration field was invalid.
    #[error("invalid kinematic traffic config `{field}`")]
    InvalidConfig {
        /// Invalid field.
        field: &'static str,
    },
    /// One or more traffic actors lacked required stable state.
    #[error("{actor_count} traffic actor(s) are missing EntityUuid, TrafficRouteFollower, or TrafficPose")]
    MissingActorState {
        /// Number of invalid actors.
        actor_count: usize,
    },
    /// Two traffic actors shared one stable UUID.
    #[error("duplicate traffic actor UUID `{uuid}`")]
    DuplicateActorId {
        /// Duplicate UUID encoded as a `u128`.
        uuid: u128,
    },
    /// An actor referred to an unknown route.
    #[error("traffic actor `{uuid}` references unknown route `{route_id}`")]
    MissingRoute {
        /// Stable actor UUID encoded as a `u128`.
        uuid: u128,
        /// Missing stable route ID.
        route_id: crate::TrafficId,
    },
    /// An actor's longitudinal state was invalid.
    #[error("traffic actor `{uuid}` has invalid kinematic state")]
    InvalidActorState {
        /// Stable actor UUID encoded as a `u128`.
        uuid: u128,
    },
}

#[derive(Clone, Debug)]
struct ActorSnapshot {
    entity: Entity,
    uuid: u128,
    follower: TrafficRouteFollower,
}

/// Advances all route followers in stable UUID order using explicit simulation time.
///
/// The system uses a deterministic car-following rule on each route and updates
/// [`TrafficPose`] without requiring rendering or a physics backend.
pub fn advance_kinematic_traffic(
    world: &mut World,
    routes: &TrafficRouteCatalog,
    runtime: &mut TrafficRuntime,
    sim_time: SimTime,
    delta: SimDuration,
    config: KinematicTrafficConfig,
) -> Result<KinematicTrafficStep, KinematicTrafficError> {
    validate_kinematic_config(config)?;
    if delta.ticks() == 0 {
        return Err(KinematicTrafficError::ZeroDelta);
    }
    let delta_s = delta.as_seconds().value();
    let mut query = world.query_filtered::<(
        Entity,
        Option<&EntityUuid>,
        Option<&TrafficRouteFollower>,
        Option<&TrafficPose>,
    ), With<TrafficActor>>();
    let mut missing_count = 0;
    let mut actors = Vec::new();
    for (entity, uuid, follower, pose) in query.iter(world) {
        match (uuid, follower, pose) {
            (Some(uuid), Some(follower), Some(_)) => actors.push(ActorSnapshot {
                entity,
                uuid: uuid.0.as_u128(),
                follower: follower.clone(),
            }),
            _ => missing_count += 1,
        }
    }
    if missing_count != 0 {
        return Err(KinematicTrafficError::MissingActorState {
            actor_count: missing_count,
        });
    }
    actors.sort_by_key(|actor| actor.uuid);
    let mut actor_ids = BTreeSet::new();
    for actor in &mut actors {
        if !actor_ids.insert(actor.uuid) {
            return Err(KinematicTrafficError::DuplicateActorId { uuid: actor.uuid });
        }
        validate_follower(actor)?;
        let route = routes.get(&actor.follower.route_id).ok_or_else(|| {
            KinematicTrafficError::MissingRoute {
                uuid: actor.uuid,
                route_id: actor.follower.route_id.clone(),
            }
        })?;
        actor.follower.distance_m = route.normalize_distance(actor.follower.distance_m);
    }

    let groups = route_groups(&actors);
    let mut updated = actors.clone();
    for indices in groups.values() {
        let leader_gaps = leader_gaps(indices, &actors, routes);
        for (position, actor_index) in indices.iter().copied().enumerate() {
            let actor = &actors[actor_index];
            let route = routes
                .get(&actor.follower.route_id)
                .expect("routes validated before stepping");
            let optional_gap_m = leader_gaps[position];
            let desired_gap_m =
                config.minimum_gap_m + actor.follower.speed_m_s * config.time_headway_s;
            let safe_speed_m_s = optional_gap_m
                .map(|gap_m| (gap_m - desired_gap_m).max(0.0) / delta_s)
                .unwrap_or(actor.follower.desired_speed_m_s);
            let target_speed_m_s = actor.follower.desired_speed_m_s.min(safe_speed_m_s);
            let speed_delta_m_s = if target_speed_m_s >= actor.follower.speed_m_s {
                config.max_acceleration_m_s2 * delta_s
            } else {
                -config.max_braking_m_s2 * delta_s
            };
            let new_speed_m_s = if speed_delta_m_s >= 0.0 {
                (actor.follower.speed_m_s + speed_delta_m_s).min(target_speed_m_s)
            } else {
                (actor.follower.speed_m_s + speed_delta_m_s).max(target_speed_m_s)
            }
            .max(0.0);
            let mut travel_m = (actor.follower.speed_m_s + new_speed_m_s) * 0.5 * delta_s;
            if let Some(gap_m) = optional_gap_m {
                travel_m = travel_m.min((gap_m - config.minimum_gap_m).max(0.0));
            }
            if !route.is_closed() {
                travel_m =
                    travel_m.min((route.total_length_m() - actor.follower.distance_m).max(0.0));
            }
            updated[actor_index].follower.speed_m_s =
                if travel_m <= 0.0 { 0.0 } else { new_speed_m_s };
            updated[actor_index].follower.distance_m =
                route.normalize_distance(actor.follower.distance_m + travel_m);
        }
    }

    for actor in &updated {
        let route = routes
            .get(&actor.follower.route_id)
            .expect("routes validated before mutation");
        let sample = route.sample(actor.follower.distance_m);
        *world
            .get_mut::<TrafficRouteFollower>(actor.entity)
            .expect("follower validated before mutation") = actor.follower.clone();
        *world
            .get_mut::<TrafficPose>(actor.entity)
            .expect("pose validated before mutation") = TrafficPose {
            position_m: sample.position_m,
            yaw_rad: sample.yaw_rad,
        };
    }

    let post_groups = route_groups(&updated);
    let minimum_observed_gap_m = post_groups
        .values()
        .flat_map(|indices| leader_gaps(indices, &updated, routes))
        .flatten()
        .reduce(f64::min);
    let completed = advance_traffic_step(runtime, sim_time);
    Ok(KinematicTrafficStep {
        completed,
        actor_count: updated.len(),
        minimum_observed_gap_m,
        stable_state_hash: stable_fleet_hash(runtime.step_index(), &updated, world),
    })
}

fn validate_kinematic_config(config: KinematicTrafficConfig) -> Result<(), KinematicTrafficError> {
    for (field, value, positive) in [
        ("max_acceleration_m_s2", config.max_acceleration_m_s2, true),
        ("max_braking_m_s2", config.max_braking_m_s2, true),
        ("minimum_gap_m", config.minimum_gap_m, false),
        ("time_headway_s", config.time_headway_s, false),
    ] {
        if !value.is_finite() || (positive && value <= 0.0) || (!positive && value < 0.0) {
            return Err(KinematicTrafficError::InvalidConfig { field });
        }
    }
    Ok(())
}

fn validate_follower(actor: &ActorSnapshot) -> Result<(), KinematicTrafficError> {
    let follower = &actor.follower;
    if !follower.distance_m.is_finite()
        || !follower.speed_m_s.is_finite()
        || follower.speed_m_s < 0.0
        || !follower.desired_speed_m_s.is_finite()
        || follower.desired_speed_m_s < 0.0
        || !follower.length_m.is_finite()
        || follower.length_m <= 0.0
    {
        return Err(KinematicTrafficError::InvalidActorState { uuid: actor.uuid });
    }
    Ok(())
}

fn route_groups(actors: &[ActorSnapshot]) -> BTreeMap<crate::TrafficId, Vec<usize>> {
    let mut groups = BTreeMap::<crate::TrafficId, Vec<usize>>::new();
    for (index, actor) in actors.iter().enumerate() {
        groups
            .entry(actor.follower.route_id.clone())
            .or_default()
            .push(index);
    }
    for indices in groups.values_mut() {
        indices.sort_by(|left, right| {
            actors[*left]
                .follower
                .distance_m
                .total_cmp(&actors[*right].follower.distance_m)
                .then_with(|| actors[*left].uuid.cmp(&actors[*right].uuid))
        });
    }
    groups
}

fn leader_gaps(
    indices: &[usize],
    actors: &[ActorSnapshot],
    routes: &TrafficRouteCatalog,
) -> Vec<Option<f64>> {
    if indices.len() < 2 {
        return vec![None; indices.len()];
    }
    let route = routes
        .get(&actors[indices[0]].follower.route_id)
        .expect("group route validated");
    indices
        .iter()
        .enumerate()
        .map(|(position, actor_index)| {
            let leader_position = position + 1;
            if leader_position == indices.len() && !route.is_closed() {
                return None;
            }
            let leader_index = indices[leader_position % indices.len()];
            let mut center_gap_m =
                actors[leader_index].follower.distance_m - actors[*actor_index].follower.distance_m;
            if center_gap_m <= 0.0 && route.is_closed() {
                center_gap_m += route.total_length_m();
            }
            Some(
                center_gap_m
                    - (actors[*actor_index].follower.length_m
                        + actors[leader_index].follower.length_m)
                        * 0.5,
            )
        })
        .collect()
}

fn stable_fleet_hash(step_index: u64, actors: &[ActorSnapshot], world: &World) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = OFFSET;
    let mut append = |bytes: &[u8]| {
        for byte in bytes {
            hash = (hash ^ u64::from(*byte)).wrapping_mul(PRIME);
        }
    };
    append(&step_index.to_le_bytes());
    for actor in actors {
        append(&actor.uuid.to_le_bytes());
        append(actor.follower.route_id.as_str().as_bytes());
        append(&actor.follower.distance_m.to_bits().to_le_bytes());
        append(&actor.follower.speed_m_s.to_bits().to_le_bytes());
        let pose = world
            .get::<TrafficPose>(actor.entity)
            .expect("pose updated before hashing");
        for coordinate in pose.position_m {
            append(&coordinate.to_bits().to_le_bytes());
        }
        append(&pose.yaw_rad.to_bits().to_le_bytes());
    }
    hash
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
