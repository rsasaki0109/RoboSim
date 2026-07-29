//! Deterministic traffic runtime systems.

use crate::{
    SignalAspect, TrafficActor, TrafficConflictControls, TrafficDeparture, TrafficPose,
    TrafficRoute, TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime, TrafficSignalControls,
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
    /// Half-width of the forward corridor used to find leaders on other routes.
    pub cross_route_headway_half_width_m: f64,
    /// Assumed body width for oriented collision diagnostics across routes.
    pub cross_route_vehicle_width_m: f64,
    /// Extra setback before a conflict movement for a vehicle without its reservation.
    pub conflict_stop_margin_m: f64,
}

impl Default for KinematicTrafficConfig {
    fn default() -> Self {
        Self {
            max_acceleration_m_s2: 2.0,
            max_braking_m_s2: 4.5,
            minimum_gap_m: 2.0,
            time_headway_s: 1.2,
            cross_route_headway_half_width_m: 1.8,
            cross_route_vehicle_width_m: 2.0,
            conflict_stop_margin_m: 2.0,
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
    /// Red stop-line crossings during this step.
    pub signal_violation_count: usize,
    /// Overlapping bumper pairs observed after this step.
    pub collision_count: usize,
    /// Number of junction conflict groups with an active owner.
    pub active_reservation_count: usize,
    /// Traffic-flow measurements after the completed step.
    pub flow: TrafficFlowMetrics,
}

/// Deterministic traffic-flow measurements.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrafficFlowMetrics {
    /// Mean speed of departed, unfinished actors.
    pub average_speed_m_s: f64,
    /// Departed, unfinished actors moving at no more than 0.1 m/s.
    pub waiting_actor_count: usize,
    /// Largest waiting-actor count on one runtime route.
    pub maximum_queue_length: usize,
    /// Cumulative number of actors that reached an open route endpoint.
    pub completed_trip_count: u64,
    /// Cumulative stopped time across all actors.
    pub cumulative_waiting_time_s: f64,
}

/// Mutable control resources consumed by one reserved traffic step.
#[derive(Debug)]
pub struct KinematicTrafficControls<'a> {
    signal_controls: &'a TrafficSignalControls,
    conflict_controls: &'a mut TrafficConflictControls,
}

impl<'a> KinematicTrafficControls<'a> {
    /// Borrows signal aspects and mutable junction reservation state.
    pub fn new(
        signal_controls: &'a TrafficSignalControls,
        conflict_controls: &'a mut TrafficConflictControls,
    ) -> Self {
        Self {
            signal_controls,
            conflict_controls,
        }
    }
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
    pose: TrafficPose,
    departure_time_s: Option<f64>,
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
    advance_traffic(
        world,
        routes,
        &TrafficSignalControls::default(),
        None,
        runtime,
        sim_time,
        delta,
        config,
    )
}

/// Advances route followers with deterministic red-signal stop-line control.
pub fn advance_controlled_kinematic_traffic(
    world: &mut World,
    routes: &TrafficRouteCatalog,
    controls: &TrafficSignalControls,
    runtime: &mut TrafficRuntime,
    sim_time: SimTime,
    delta: SimDuration,
    config: KinematicTrafficConfig,
) -> Result<KinematicTrafficStep, KinematicTrafficError> {
    advance_traffic(
        world, routes, controls, None, runtime, sim_time, delta, config,
    )
}

/// Advances route followers with red-signal and deterministic junction-reservation control.
pub fn advance_reserved_kinematic_traffic(
    world: &mut World,
    routes: &TrafficRouteCatalog,
    controls: KinematicTrafficControls<'_>,
    runtime: &mut TrafficRuntime,
    sim_time: SimTime,
    delta: SimDuration,
    config: KinematicTrafficConfig,
) -> Result<KinematicTrafficStep, KinematicTrafficError> {
    advance_traffic(
        world,
        routes,
        controls.signal_controls,
        Some(controls.conflict_controls),
        runtime,
        sim_time,
        delta,
        config,
    )
}

#[allow(clippy::too_many_arguments)]
fn advance_traffic(
    world: &mut World,
    routes: &TrafficRouteCatalog,
    controls: &TrafficSignalControls,
    mut conflict_controls: Option<&mut TrafficConflictControls>,
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
        Option<&TrafficDeparture>,
    ), With<TrafficActor>>();
    let mut missing_count = 0;
    let mut actors = Vec::new();
    for (entity, uuid, follower, pose, departure) in query.iter(world) {
        match (uuid, follower, pose) {
            (Some(uuid), Some(follower), Some(pose)) => actors.push(ActorSnapshot {
                entity,
                uuid: uuid.0.as_u128(),
                follower: follower.clone(),
                pose: *pose,
                departure_time_s: departure.map(|departure| departure.departure_time_s),
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
        if actor
            .departure_time_s
            .is_some_and(|time_s| !time_s.is_finite() || time_s < 0.0)
        {
            return Err(KinematicTrafficError::InvalidActorState { uuid: actor.uuid });
        }
        let route = routes.get(&actor.follower.route_id).ok_or_else(|| {
            KinematicTrafficError::MissingRoute {
                uuid: actor.uuid,
                route_id: actor.follower.route_id.clone(),
            }
        })?;
        actor.follower.distance_m = route.normalize_distance(actor.follower.distance_m);
    }
    if let Some(conflicts) = conflict_controls.as_deref_mut() {
        update_conflict_reservations(&actors, controls, conflicts, sim_time);
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
            let spatial_gap_m = cross_route_leader_gap_m(actor_index, &actors, config);
            let optional_gap_m = [optional_gap_m, spatial_gap_m]
                .into_iter()
                .flatten()
                .reduce(f64::min);
            let desired_gap_m =
                config.minimum_gap_m + actor.follower.speed_m_s * config.time_headway_s;
            let headway_speed_m_s = optional_gap_m
                .map(|gap_m| (gap_m - desired_gap_m).max(0.0) / delta_s)
                .unwrap_or(actor.follower.desired_speed_m_s);
            let signal_limit_m = red_signal_limit_m(actor, route, controls);
            let conflict_limit_m = conflict_controls.as_deref().and_then(|conflicts| {
                conflict_stop_limit_m(actor, route, conflicts, config.conflict_stop_margin_m)
            });
            let control_limit_m = [signal_limit_m, conflict_limit_m]
                .into_iter()
                .flatten()
                .reduce(f64::min);
            let control_speed_m_s = control_limit_m
                .map(|distance_m| (2.0 * config.max_braking_m_s2 * distance_m).sqrt())
                .unwrap_or(actor.follower.desired_speed_m_s);
            let departed = actor
                .departure_time_s
                .is_none_or(|departure_time_s| sim_time.as_seconds().value() >= departure_time_s);
            let target_speed_m_s = actor
                .follower
                .desired_speed_m_s
                .min(headway_speed_m_s)
                .min(control_speed_m_s)
                * if departed { 1.0 } else { 0.0 };
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
            if let Some(control_limit_m) = control_limit_m {
                travel_m = travel_m.min(control_limit_m);
            }
            if !departed {
                travel_m = 0.0;
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
    let collision_count = post_groups
        .values()
        .flat_map(|indices| leader_gaps(indices, &updated, routes))
        .flatten()
        .filter(|gap_m| *gap_m < 0.0)
        .count()
        + cross_route_collision_count(&updated, world, config);
    let flow = record_flow_metrics(runtime, &updated, routes, sim_time, delta);
    let completed = advance_traffic_step(runtime, sim_time);
    Ok(KinematicTrafficStep {
        completed,
        actor_count: updated.len(),
        minimum_observed_gap_m,
        stable_state_hash: stable_fleet_hash(runtime.step_index(), &updated, world),
        signal_violation_count: 0,
        collision_count,
        active_reservation_count: conflict_controls
            .as_deref()
            .map_or(0, TrafficConflictControls::reservation_count),
        flow,
    })
}

fn record_flow_metrics(
    runtime: &mut TrafficRuntime,
    actors: &[ActorSnapshot],
    routes: &TrafficRouteCatalog,
    sim_time: SimTime,
    delta: SimDuration,
) -> TrafficFlowMetrics {
    let mut active_speed_sum_m_s = 0.0;
    let mut active_actor_count = 0;
    let mut waiting_actor_count = 0;
    let mut queues = BTreeMap::<crate::TrafficId, usize>::new();
    for actor in actors {
        let route = routes
            .get(&actor.follower.route_id)
            .expect("route validated before flow metrics");
        let departed = actor
            .departure_time_s
            .is_none_or(|departure_time_s| sim_time.as_seconds().value() >= departure_time_s);
        let completed =
            !route.is_closed() && actor.follower.distance_m >= route.total_length_m() - 1.0e-9;
        let waiting = departed && !completed && actor.follower.speed_m_s <= 0.1;
        runtime.record_actor_step(
            actor.uuid,
            &actor.follower.route_id,
            if waiting { delta.ticks() } else { 0 },
            completed,
        );
        if departed && !completed {
            active_speed_sum_m_s += actor.follower.speed_m_s;
            active_actor_count += 1;
        }
        if waiting {
            waiting_actor_count += 1;
            *queues.entry(actor.follower.route_id.clone()).or_default() += 1;
        }
    }
    TrafficFlowMetrics {
        average_speed_m_s: if active_actor_count == 0 {
            0.0
        } else {
            active_speed_sum_m_s / active_actor_count as f64
        },
        waiting_actor_count,
        maximum_queue_length: queues.values().copied().max().unwrap_or(0),
        completed_trip_count: runtime.completed_trip_count(),
        cumulative_waiting_time_s: runtime.cumulative_waiting_ticks() as f64 / 1_000_000_000.0,
    }
}

fn red_signal_limit_m(
    actor: &ActorSnapshot,
    route: &TrafficRoute,
    controls: &TrafficSignalControls,
) -> Option<f64> {
    controls
        .iter()
        .filter(|control| {
            control.route_id == actor.follower.route_id && control.aspect == SignalAspect::Red
        })
        .filter_map(|control| {
            let stop_center_m =
                route.normalize_distance(control.stop_distance_m - actor.follower.length_m * 0.5);
            if route.is_closed() {
                Some((stop_center_m - actor.follower.distance_m).rem_euclid(route.total_length_m()))
            } else if stop_center_m >= actor.follower.distance_m {
                Some(stop_center_m - actor.follower.distance_m)
            } else {
                None
            }
        })
        .reduce(f64::min)
}

fn update_conflict_reservations(
    actors: &[ActorSnapshot],
    signals: &TrafficSignalControls,
    conflicts: &mut TrafficConflictControls,
    sim_time: SimTime,
) {
    let request_distance_m = conflicts.request_distance_m();
    let sim_time_s = sim_time.as_seconds().value();
    for group_id in conflicts.group_ids() {
        let retained_owner = conflicts.owner(&group_id).filter(|owner| {
            let Some(actor) = actors.iter().find(|actor| actor.uuid == *owner) else {
                return false;
            };
            if !actor_has_departed(actor, sim_time_s) {
                return false;
            }
            conflicts.iter().any(|control| {
                control.conflict_group_id == group_id
                    && control.route_id == actor.follower.route_id
                    && actor.follower.distance_m - actor.follower.length_m * 0.5
                        <= control.exit_distance_m
                    && (actor.follower.distance_m + actor.follower.length_m * 0.5
                        >= control.entry_distance_m
                        || !red_signal_blocks_reservation(actor, control, signals))
            })
        });
        if retained_owner.is_some() {
            continue;
        }
        let candidate = conflicts
            .iter()
            .filter(|control| control.conflict_group_id == group_id)
            .flat_map(|control| {
                actors
                    .iter()
                    .filter(move |actor| actor.follower.route_id == control.route_id)
                    .filter(move |actor| actor_has_departed(actor, sim_time_s))
                    .filter(move |actor| !red_signal_blocks_reservation(actor, control, signals))
                    .filter_map(move |actor| {
                        let front_distance_m =
                            actor.follower.distance_m + actor.follower.length_m * 0.5;
                        let rear_distance_m =
                            actor.follower.distance_m - actor.follower.length_m * 0.5;
                        if rear_distance_m > control.exit_distance_m {
                            return None;
                        }
                        let remaining_m = (control.entry_distance_m - front_distance_m).max(0.0);
                        if remaining_m > request_distance_m {
                            return None;
                        }
                        let arrival_s = remaining_m / actor.follower.speed_m_s.max(0.5);
                        Some((control.priority, arrival_s, actor.uuid))
                    })
            })
            .min_by(|left, right| {
                left.0
                    .cmp(&right.0)
                    .then_with(|| left.1.total_cmp(&right.1))
                    .then_with(|| left.2.cmp(&right.2))
            })
            .map(|candidate| candidate.2);
        conflicts.set_owner(group_id, candidate);
    }
}

fn actor_has_departed(actor: &ActorSnapshot, sim_time_s: f64) -> bool {
    actor
        .departure_time_s
        .is_none_or(|departure_time_s| sim_time_s >= departure_time_s)
}

fn red_signal_blocks_reservation(
    actor: &ActorSnapshot,
    conflict: &crate::TrafficConflictControl,
    signals: &TrafficSignalControls,
) -> bool {
    let actor_front_m = actor.follower.distance_m + actor.follower.length_m * 0.5;
    signals.iter().any(|signal| {
        signal.route_id == actor.follower.route_id
            && signal.aspect == SignalAspect::Red
            && signal.stop_distance_m + 1.0e-9 >= actor_front_m
            && signal.stop_distance_m <= conflict.entry_distance_m + 1.0e-9
    })
}

fn conflict_stop_limit_m(
    actor: &ActorSnapshot,
    route: &TrafficRoute,
    controls: &TrafficConflictControls,
    stop_margin_m: f64,
) -> Option<f64> {
    controls
        .iter()
        .filter(|control| control.route_id == actor.follower.route_id)
        .filter(|control| controls.owner(&control.conflict_group_id) != Some(actor.uuid))
        .filter_map(|control| {
            let stop_center_m = route.normalize_distance(
                control.entry_distance_m - actor.follower.length_m * 0.5 - stop_margin_m,
            );
            (stop_center_m >= actor.follower.distance_m)
                .then_some(stop_center_m - actor.follower.distance_m)
        })
        .reduce(f64::min)
}

fn cross_route_leader_gap_m(
    actor_index: usize,
    actors: &[ActorSnapshot],
    config: KinematicTrafficConfig,
) -> Option<f64> {
    let actor = &actors[actor_index];
    let forward = [actor.pose.yaw_rad.cos(), -actor.pose.yaw_rad.sin()];
    let right = [-forward[1], forward[0]];
    actors
        .iter()
        .enumerate()
        .filter(|(index, candidate)| {
            *index != actor_index && candidate.follower.route_id != actor.follower.route_id
        })
        .filter_map(|(_, candidate)| {
            let delta = [
                candidate.pose.position_m[0] - actor.pose.position_m[0],
                candidate.pose.position_m[2] - actor.pose.position_m[2],
            ];
            let longitudinal_m = delta[0] * forward[0] + delta[1] * forward[1];
            let lateral_m = (delta[0] * right[0] + delta[1] * right[1]).abs();
            (longitudinal_m > 0.0 && lateral_m <= config.cross_route_headway_half_width_m)
                .then_some(
                    longitudinal_m - (actor.follower.length_m + candidate.follower.length_m) * 0.5,
                )
        })
        .reduce(f64::min)
}

fn cross_route_collision_count(
    actors: &[ActorSnapshot],
    world: &World,
    config: KinematicTrafficConfig,
) -> usize {
    let mut count = 0;
    for left in 0..actors.len() {
        for right in (left + 1)..actors.len() {
            if actors[left].follower.route_id == actors[right].follower.route_id {
                continue;
            }
            let left_pose = world
                .get::<TrafficPose>(actors[left].entity)
                .expect("pose updated before collision diagnostics");
            let right_pose = world
                .get::<TrafficPose>(actors[right].entity)
                .expect("pose updated before collision diagnostics");
            if oriented_vehicle_rectangles_overlap(
                &actors[left],
                left_pose,
                &actors[right],
                right_pose,
                config.cross_route_vehicle_width_m,
            ) {
                count += 1;
            }
        }
    }
    count
}

fn oriented_vehicle_rectangles_overlap(
    left: &ActorSnapshot,
    left_pose: &TrafficPose,
    right: &ActorSnapshot,
    right_pose: &TrafficPose,
    vehicle_width_m: f64,
) -> bool {
    let left_forward = [left_pose.yaw_rad.cos(), -left_pose.yaw_rad.sin()];
    let left_right = [-left_forward[1], left_forward[0]];
    let right_forward = [right_pose.yaw_rad.cos(), -right_pose.yaw_rad.sin()];
    let right_right = [-right_forward[1], right_forward[0]];
    let delta = [
        right_pose.position_m[0] - left_pose.position_m[0],
        right_pose.position_m[2] - left_pose.position_m[2],
    ];
    [left_forward, left_right, right_forward, right_right]
        .into_iter()
        .all(|axis| {
            let center_distance_m = dot2(delta, axis).abs();
            let left_radius_m = left.follower.length_m * 0.5 * dot2(left_forward, axis).abs()
                + vehicle_width_m * 0.5 * dot2(left_right, axis).abs();
            let right_radius_m = right.follower.length_m * 0.5 * dot2(right_forward, axis).abs()
                + vehicle_width_m * 0.5 * dot2(right_right, axis).abs();
            center_distance_m < left_radius_m + right_radius_m - 1.0e-9
        })
}

fn dot2(left: [f64; 2], right: [f64; 2]) -> f64 {
    left[0] * right[0] + left[1] * right[1]
}

fn validate_kinematic_config(config: KinematicTrafficConfig) -> Result<(), KinematicTrafficError> {
    for (field, value, positive) in [
        ("max_acceleration_m_s2", config.max_acceleration_m_s2, true),
        ("max_braking_m_s2", config.max_braking_m_s2, true),
        ("minimum_gap_m", config.minimum_gap_m, false),
        ("time_headway_s", config.time_headway_s, false),
        (
            "cross_route_headway_half_width_m",
            config.cross_route_headway_half_width_m,
            true,
        ),
        (
            "cross_route_vehicle_width_m",
            config.cross_route_vehicle_width_m,
            true,
        ),
        (
            "conflict_stop_margin_m",
            config.conflict_stop_margin_m,
            false,
        ),
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
