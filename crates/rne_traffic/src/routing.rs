//! Deterministic shortest-path routing over traffic schema v1.

use crate::{TrafficActorKind, TrafficAsset, TrafficConnection, TrafficId, TrafficNetwork};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

/// Stable lane and connection sequence returned by the route planner.
#[derive(Clone, Debug, PartialEq)]
pub struct LaneRoute {
    /// Ordered directed lane IDs, including start and goal.
    pub lane_ids: Vec<TrafficId>,
    /// Ordered connection IDs between adjacent lanes.
    pub connection_ids: Vec<TrafficId>,
    /// Total traversed centerline and connection distance.
    pub distance_m: f64,
}

/// Deterministic route-planning failure.
#[derive(Debug, Error, PartialEq)]
pub enum RoutingError {
    /// The source network failed schema validation.
    #[error("invalid traffic network: {0}")]
    InvalidNetwork(String),
    /// A requested lane ID does not exist.
    #[error("unknown route endpoint lane `{lane_id}`")]
    UnknownLane {
        /// Missing lane ID.
        lane_id: TrafficId,
    },
    /// A requested endpoint does not admit the actor class.
    #[error("lane `{lane_id}` does not allow {actor:?}")]
    ActorNotAllowed {
        /// Incompatible lane ID.
        lane_id: TrafficId,
        /// Requested actor class.
        actor: TrafficActorKind,
    },
    /// No compatible directed path exists.
    #[error("no route from `{start_lane_id}` to `{goal_lane_id}` for {actor:?}")]
    NoRoute {
        /// Start lane ID.
        start_lane_id: TrafficId,
        /// Goal lane ID.
        goal_lane_id: TrafficId,
        /// Requested actor class.
        actor: TrafficActorKind,
    },
}

#[derive(Clone, Debug)]
struct Candidate {
    distance_m: f64,
    lane_ids: Vec<TrafficId>,
    connection_ids: Vec<TrafficId>,
}

/// Finds the shortest actor-compatible directed lane route.
///
/// Equal-distance alternatives are resolved by the lexicographic lane-ID
/// sequence and then connection-ID sequence, independent of asset array order.
pub fn shortest_lane_route(
    network: &TrafficNetwork,
    start_lane_id: &TrafficId,
    goal_lane_id: &TrafficId,
    actor: TrafficActorKind,
) -> Result<LaneRoute, RoutingError> {
    TrafficAsset::new(network.clone())
        .validate()
        .map_err(|error| RoutingError::InvalidNetwork(error.to_string()))?;
    let lanes = network
        .lanes
        .iter()
        .map(|lane| (lane.id.clone(), lane))
        .collect::<BTreeMap<_, _>>();
    for lane_id in [start_lane_id, goal_lane_id] {
        let lane = lanes
            .get(lane_id)
            .ok_or_else(|| RoutingError::UnknownLane {
                lane_id: lane_id.clone(),
            })?;
        if !lane.allowed_actors.contains(&actor) {
            return Err(RoutingError::ActorNotAllowed {
                lane_id: lane_id.clone(),
                actor,
            });
        }
    }
    let mut outgoing = BTreeMap::<TrafficId, Vec<&TrafficConnection>>::new();
    for connection in &network.connections {
        outgoing
            .entry(connection.incoming_lane_id.clone())
            .or_default()
            .push(connection);
    }
    for connections in outgoing.values_mut() {
        connections.sort_by(|left, right| left.id.cmp(&right.id));
    }

    let start = Candidate {
        distance_m: lane_length_m(lanes[start_lane_id].centerline_m.as_slice()),
        lane_ids: vec![start_lane_id.clone()],
        connection_ids: Vec::new(),
    };
    let mut best = BTreeMap::from([(start_lane_id.clone(), start)]);
    let mut unsettled = BTreeSet::from([start_lane_id.clone()]);
    while !unsettled.is_empty() {
        let current_id = unsettled
            .iter()
            .min_by(|left, right| candidate_order(&best[*left], &best[*right]))
            .expect("nonempty unsettled set")
            .clone();
        unsettled.remove(&current_id);
        let current = best[&current_id].clone();
        if &current_id == goal_lane_id {
            return Ok(LaneRoute {
                lane_ids: current.lane_ids,
                connection_ids: current.connection_ids,
                distance_m: current.distance_m,
            });
        }
        for connection in outgoing.get(&current_id).into_iter().flatten() {
            let next_lane = lanes[&connection.outgoing_lane_id];
            if !next_lane.allowed_actors.contains(&actor) {
                continue;
            }
            let mut candidate = current.clone();
            candidate.distance_m +=
                lane_length_m(&connection.path_m) + lane_length_m(&next_lane.centerline_m);
            candidate.lane_ids.push(next_lane.id.clone());
            candidate.connection_ids.push(connection.id.clone());
            let replace = best
                .get(&next_lane.id)
                .is_none_or(|known| candidate_order(&candidate, known).is_lt());
            if replace {
                best.insert(next_lane.id.clone(), candidate);
                unsettled.insert(next_lane.id.clone());
            }
        }
    }
    Err(RoutingError::NoRoute {
        start_lane_id: start_lane_id.clone(),
        goal_lane_id: goal_lane_id.clone(),
        actor,
    })
}

fn candidate_order(left: &Candidate, right: &Candidate) -> std::cmp::Ordering {
    left.distance_m
        .total_cmp(&right.distance_m)
        .then_with(|| left.lane_ids.cmp(&right.lane_ids))
        .then_with(|| left.connection_ids.cmp(&right.connection_ids))
}

fn lane_length_m(points: &[[f64; 3]]) -> f64 {
    points
        .windows(2)
        .map(|pair| {
            (pair[1][0] - pair[0][0])
                .hypot(pair[1][1] - pair[0][1])
                .hypot(pair[1][2] - pair[0][2])
        })
        .sum()
}
