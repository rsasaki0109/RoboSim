//! Traffic runtime resources.

use crate::{MovementKind, SignalAspect, TrafficAsset, TrafficId, TrafficNetwork};
use bevy_ecs::prelude::Resource;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const ROUTE_EPSILON_M: f64 = 1.0e-9;

/// Per-world deterministic traffic runtime state.
#[derive(Clone, Debug, Default, PartialEq, Eq, Resource)]
pub struct TrafficRuntime {
    step_index: u64,
    actor_metrics: BTreeMap<u128, TrafficActorRuntimeMetrics>,
    completed_trip_count: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct TrafficActorRuntimeMetrics {
    route_id: TrafficId,
    waiting_ticks: u64,
    completed: bool,
}

impl TrafficRuntime {
    /// Returns the number of completed traffic steps.
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    pub(crate) fn advance(&mut self) -> u64 {
        self.step_index = self.step_index.saturating_add(1);
        self.step_index
    }

    pub(crate) fn record_actor_step(
        &mut self,
        uuid: u128,
        route_id: &TrafficId,
        waiting_ticks: u64,
        completed: bool,
    ) {
        let metrics =
            self.actor_metrics
                .entry(uuid)
                .or_insert_with(|| TrafficActorRuntimeMetrics {
                    route_id: route_id.clone(),
                    waiting_ticks: 0,
                    completed: false,
                });
        if metrics.route_id != *route_id {
            *metrics = TrafficActorRuntimeMetrics {
                route_id: route_id.clone(),
                waiting_ticks: 0,
                completed: false,
            };
        }
        metrics.waiting_ticks = metrics.waiting_ticks.saturating_add(waiting_ticks);
        if completed && !metrics.completed {
            metrics.completed = true;
            self.completed_trip_count = self.completed_trip_count.saturating_add(1);
        }
    }

    pub(crate) fn completed_trip_count(&self) -> u64 {
        self.completed_trip_count
    }

    pub(crate) fn cumulative_waiting_ticks(&self) -> u64 {
        self.actor_metrics
            .values()
            .map(|metrics| metrics.waiting_ticks)
            .fold(0_u64, u64::saturating_add)
    }
}

/// One source-network movement embedded in a runtime route.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficRouteMovement {
    /// Source connection traversed by this movement.
    pub connection_id: TrafficId,
    /// Route distance at the beginning of the connection path.
    pub entry_distance_m: f64,
    /// Route distance at the end of the connection path.
    pub exit_distance_m: f64,
}

/// One deterministic polyline route shared by traffic actors.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficRoute {
    id: TrafficId,
    path_m: Vec<[f64; 3]>,
    cumulative_distance_m: Vec<f64>,
    total_length_m: f64,
    closed: bool,
    movements: Vec<TrafficRouteMovement>,
}

impl TrafficRoute {
    /// Validates and constructs a route.
    pub fn new(
        id: TrafficId,
        path_m: Vec<[f64; 3]>,
        closed: bool,
    ) -> Result<Self, TrafficRouteError> {
        if path_m.len() < 2 {
            return Err(TrafficRouteError::TooFewPoints { route_id: id });
        }
        if path_m.iter().flatten().any(|value| !value.is_finite()) {
            return Err(TrafficRouteError::NonFiniteGeometry { route_id: id });
        }
        let mut cumulative_distance_m = Vec::with_capacity(path_m.len());
        cumulative_distance_m.push(0.0);
        for points in path_m.windows(2) {
            let length_m = distance(points[0], points[1]);
            let next_m = cumulative_distance_m.last().copied().unwrap_or(0.0) + length_m;
            cumulative_distance_m.push(next_m);
        }
        let closing_length_m = if closed {
            distance(*path_m.last().expect("nonempty path"), path_m[0])
        } else {
            0.0
        };
        let total_length_m =
            cumulative_distance_m.last().copied().unwrap_or(0.0) + closing_length_m;
        if total_length_m <= ROUTE_EPSILON_M {
            return Err(TrafficRouteError::DegenerateGeometry { route_id: id });
        }
        Ok(Self {
            id,
            path_m,
            cumulative_distance_m,
            total_length_m,
            closed,
            movements: Vec::new(),
        })
    }

    /// Returns the stable route identifier.
    pub fn id(&self) -> &TrafficId {
        &self.id
    }

    /// Returns the directed route points.
    pub fn path_m(&self) -> &[[f64; 3]] {
        &self.path_m
    }

    /// Returns total polyline length in meters.
    pub fn total_length_m(&self) -> f64 {
        self.total_length_m
    }

    /// Returns whether distance wraps from the final point to the first.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Returns source-network movements and their route-distance spans.
    pub fn movements(&self) -> &[TrafficRouteMovement] {
        &self.movements
    }

    /// Samples position and heading at one route distance.
    pub fn sample(&self, distance_m: f64) -> TrafficRouteSample {
        let distance_m = self.normalize_distance(distance_m);
        let last_path_distance_m = *self
            .cumulative_distance_m
            .last()
            .expect("validated route distances");
        if self.closed && distance_m > last_path_distance_m {
            return sample_segment(
                *self.path_m.last().expect("validated route"),
                self.path_m[0],
                distance_m - last_path_distance_m,
                self.total_length_m - last_path_distance_m,
            );
        }
        let segment_index = self
            .cumulative_distance_m
            .partition_point(|value| *value <= distance_m)
            .saturating_sub(1)
            .min(self.path_m.len() - 2);
        sample_segment(
            self.path_m[segment_index],
            self.path_m[segment_index + 1],
            distance_m - self.cumulative_distance_m[segment_index],
            self.cumulative_distance_m[segment_index + 1]
                - self.cumulative_distance_m[segment_index],
        )
    }

    pub(crate) fn normalize_distance(&self, distance_m: f64) -> f64 {
        if self.closed {
            distance_m.rem_euclid(self.total_length_m)
        } else {
            distance_m.clamp(0.0, self.total_length_m)
        }
    }

    pub(crate) fn with_movements(mut self, movements: Vec<TrafficRouteMovement>) -> Self {
        self.movements = movements;
        self
    }
}

/// Position and heading sampled from a [`TrafficRoute`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrafficRouteSample {
    /// Interpolated position in frame meters.
    pub position_m: [f64; 3],
    /// Heading around the positive Y axis in radians.
    pub yaw_rad: f64,
}

/// Validated routes available to traffic runtime systems.
#[derive(Clone, Debug, Default, PartialEq, Resource)]
pub struct TrafficRouteCatalog {
    routes: BTreeMap<TrafficId, TrafficRoute>,
}

impl TrafficRouteCatalog {
    /// Inserts a route, rejecting a duplicate stable ID.
    pub fn insert(&mut self, route: TrafficRoute) -> Result<(), TrafficRouteError> {
        if self.routes.contains_key(route.id()) {
            return Err(TrafficRouteError::DuplicateId {
                route_id: route.id().clone(),
            });
        }
        self.routes.insert(route.id().clone(), route);
        Ok(())
    }

    /// Resolves one route by stable ID.
    pub fn get(&self, route_id: &TrafficId) -> Option<&TrafficRoute> {
        self.routes.get(route_id)
    }

    /// Returns the number of routes.
    pub fn len(&self) -> usize {
        self.routes.len()
    }

    /// Returns whether no routes are registered.
    pub fn is_empty(&self) -> bool {
        self.routes.is_empty()
    }

    /// Iterates routes in stable ID order.
    pub fn iter(&self) -> impl Iterator<Item = (&TrafficId, &TrafficRoute)> {
        self.routes.iter()
    }
}

/// Invalid route geometry or catalog operation.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrafficRouteError {
    /// A route cannot define a segment.
    #[error("traffic route `{route_id}` requires at least two points")]
    TooFewPoints {
        /// Invalid route ID.
        route_id: TrafficId,
    },
    /// A route contains NaN or infinity.
    #[error("traffic route `{route_id}` contains non-finite geometry")]
    NonFiniteGeometry {
        /// Invalid route ID.
        route_id: TrafficId,
    },
    /// A route has zero total length.
    #[error("traffic route `{route_id}` has zero total length")]
    DegenerateGeometry {
        /// Invalid route ID.
        route_id: TrafficId,
    },
    /// A catalog already contains the route ID.
    #[error("duplicate traffic route ID `{route_id}`")]
    DuplicateId {
        /// Duplicate route ID.
        route_id: TrafficId,
    },
}

/// One signal-controlled stop position on a runtime route.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficSignalControl {
    /// Stable signal-control identifier.
    pub id: TrafficId,
    /// Route affected by this control.
    pub route_id: TrafficId,
    /// Longitudinal stop-line distance along the route.
    pub stop_distance_m: f64,
    /// Current signal aspect.
    pub aspect: SignalAspect,
}

/// Runtime signal controls keyed by stable ID.
#[derive(Clone, Debug, Default, PartialEq, Resource)]
pub struct TrafficSignalControls {
    controls: BTreeMap<TrafficId, TrafficSignalControl>,
}

impl TrafficSignalControls {
    /// Inserts a validated control.
    pub fn insert(
        &mut self,
        control: TrafficSignalControl,
    ) -> Result<(), TrafficSignalControlError> {
        if !control.stop_distance_m.is_finite() || control.stop_distance_m < 0.0 {
            return Err(TrafficSignalControlError::InvalidStopDistance {
                control_id: control.id,
            });
        }
        if self.controls.contains_key(&control.id) {
            return Err(TrafficSignalControlError::DuplicateId {
                control_id: control.id,
            });
        }
        self.controls.insert(control.id.clone(), control);
        Ok(())
    }

    /// Updates one control aspect.
    pub fn set_aspect(
        &mut self,
        control_id: &TrafficId,
        aspect: SignalAspect,
    ) -> Result<(), TrafficSignalControlError> {
        let control = self.controls.get_mut(control_id).ok_or_else(|| {
            TrafficSignalControlError::UnknownId {
                control_id: control_id.clone(),
            }
        })?;
        control.aspect = aspect;
        Ok(())
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &TrafficSignalControl> {
        self.controls.values()
    }
}

/// Invalid runtime signal-control operation.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrafficSignalControlError {
    /// Stop distance was negative or non-finite.
    #[error("traffic signal control `{control_id}` has an invalid stop distance")]
    InvalidStopDistance {
        /// Invalid control ID.
        control_id: TrafficId,
    },
    /// The stable control ID was already registered.
    #[error("duplicate traffic signal control ID `{control_id}`")]
    DuplicateId {
        /// Duplicate control ID.
        control_id: TrafficId,
    },
    /// The stable control ID was not registered.
    #[error("unknown traffic signal control ID `{control_id}`")]
    UnknownId {
        /// Missing control ID.
        control_id: TrafficId,
    },
}

/// One route's controlled traversal through a conflicting junction.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficConflictControl {
    /// Stable representative of the connection-conflict component.
    pub conflict_group_id: TrafficId,
    /// Runtime route containing the movement.
    pub route_id: TrafficId,
    /// Source connection traversed by the route.
    pub connection_id: TrafficId,
    /// Route distance where the controlled movement begins.
    pub entry_distance_m: f64,
    /// Route distance where the controlled movement has cleared.
    pub exit_distance_m: f64,
    /// Stable priority; lower values are considered first.
    pub priority: u32,
}

/// Deterministic reservations for conflicting route movements.
#[derive(Clone, Debug, PartialEq, Resource)]
pub struct TrafficConflictControls {
    request_distance_m: f64,
    controls: BTreeMap<(TrafficId, TrafficId), TrafficConflictControl>,
    reservations: BTreeMap<TrafficId, u128>,
}

impl TrafficConflictControls {
    /// Builds conservative conflict-component groups from route movement spans
    /// and symmetric connection conflicts in a validated network.
    pub fn from_network_routes(
        network: &TrafficNetwork,
        routes: &TrafficRouteCatalog,
        request_distance_m: f64,
    ) -> Result<Self, TrafficConflictControlError> {
        if !request_distance_m.is_finite() || request_distance_m <= 0.0 {
            return Err(TrafficConflictControlError::InvalidRequestDistance);
        }
        TrafficAsset::new(network.clone())
            .validate()
            .map_err(|error| TrafficConflictControlError::InvalidNetwork {
                message: error.to_string(),
            })?;
        let connections = network
            .connections
            .iter()
            .map(|connection| (connection.id.clone(), connection))
            .collect::<BTreeMap<_, _>>();
        let conflict_groups = connection_conflict_groups(&connections);
        let mut controls = BTreeMap::<(TrafficId, TrafficId), TrafficConflictControl>::new();
        for (route_id, route) in routes.iter() {
            if route.is_closed() && !route.movements().is_empty() {
                return Err(TrafficConflictControlError::ClosedRoute {
                    route_id: route_id.clone(),
                });
            }
            for movement in route.movements() {
                let connection = connections.get(&movement.connection_id).ok_or_else(|| {
                    TrafficConflictControlError::UnknownConnection {
                        connection_id: movement.connection_id.clone(),
                    }
                })?;
                let Some(group_id) = conflict_groups.get(&connection.id).cloned() else {
                    continue;
                };
                let control = TrafficConflictControl {
                    conflict_group_id: group_id.clone(),
                    route_id: route_id.clone(),
                    connection_id: connection.id.clone(),
                    entry_distance_m: movement.entry_distance_m,
                    exit_distance_m: movement.exit_distance_m,
                    priority: movement_priority(connection.movement),
                };
                let key = (group_id, route_id.clone());
                if let Some(existing) = controls.get_mut(&key) {
                    existing.entry_distance_m =
                        existing.entry_distance_m.min(control.entry_distance_m);
                    existing.exit_distance_m =
                        existing.exit_distance_m.max(control.exit_distance_m);
                    existing.priority = existing.priority.max(control.priority);
                    existing.connection_id =
                        existing.connection_id.clone().min(control.connection_id);
                } else {
                    controls.insert(key, control);
                }
            }
        }
        Ok(Self {
            request_distance_m,
            controls,
            reservations: BTreeMap::new(),
        })
    }

    /// Returns the maximum upstream distance at which a reservation is requested.
    pub fn request_distance_m(&self) -> f64 {
        self.request_distance_m
    }

    /// Returns the stable UUID currently owning a conflict group.
    pub fn owner(&self, conflict_group_id: &TrafficId) -> Option<u128> {
        self.reservations.get(conflict_group_id).copied()
    }

    /// Returns the number of controlled route/junction pairs.
    pub fn len(&self) -> usize {
        self.controls.len()
    }

    /// Returns whether no route movement requires conflict control.
    pub fn is_empty(&self) -> bool {
        self.controls.is_empty()
    }

    /// Iterates configured route movements in stable group/route order.
    pub fn iter(&self) -> impl Iterator<Item = &TrafficConflictControl> {
        self.controls.values()
    }

    pub(crate) fn group_ids(&self) -> Vec<TrafficId> {
        self.controls
            .keys()
            .map(|(group_id, _)| group_id.clone())
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    pub(crate) fn set_owner(&mut self, group_id: TrafficId, owner: Option<u128>) {
        if let Some(owner) = owner {
            self.reservations.insert(group_id, owner);
        } else {
            self.reservations.remove(&group_id);
        }
    }

    pub(crate) fn reservation_count(&self) -> usize {
        self.reservations.len()
    }
}

/// Invalid route conflict-control configuration.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrafficConflictControlError {
    /// Reservation requests require a positive finite lookahead.
    #[error("traffic conflict request distance must be finite and greater than zero")]
    InvalidRequestDistance,
    /// The source traffic network is invalid.
    #[error("invalid traffic network for conflict control: {message}")]
    InvalidNetwork {
        /// Schema validation detail.
        message: String,
    },
    /// A materialized route refers to a missing source connection.
    #[error("runtime route refers to unknown connection `{connection_id}`")]
    UnknownConnection {
        /// Missing connection ID.
        connection_id: TrafficId,
    },
    /// Closed routes require lap-aware movement spans, which are not yet supported.
    #[error("closed route `{route_id}` cannot contain conflict-controlled movements")]
    ClosedRoute {
        /// Unsupported closed route.
        route_id: TrafficId,
    },
}

fn movement_priority(movement: MovementKind) -> u32 {
    match movement {
        MovementKind::Straight => 0,
        MovementKind::Right => 1,
        MovementKind::Left => 2,
        MovementKind::Merge | MovementKind::Split => 3,
        MovementKind::UTurn => 4,
    }
}

fn connection_conflict_groups(
    connections: &BTreeMap<TrafficId, &crate::TrafficConnection>,
) -> BTreeMap<TrafficId, TrafficId> {
    let controlled_junctions = connections
        .values()
        .filter_map(|connection| connection.junction_id.clone())
        .collect::<BTreeSet<_>>();
    let junction_connections = connections.values().fold(
        BTreeMap::<TrafficId, Vec<TrafficId>>::new(),
        |mut grouped, connection| {
            if let Some(junction_id) = &connection.junction_id {
                if controlled_junctions.contains(junction_id) {
                    grouped
                        .entry(junction_id.clone())
                        .or_default()
                        .push(connection.id.clone());
                }
            }
            grouped
        },
    );
    let mut unvisited = connections
        .values()
        .filter(|connection| {
            !connection.conflict_connection_ids.is_empty() || connection.junction_id.is_some()
        })
        .map(|connection| connection.id.clone())
        .collect::<BTreeSet<_>>();
    let mut groups = BTreeMap::new();
    while let Some(first) = unvisited.iter().next().cloned() {
        let mut pending = vec![first];
        let mut component = BTreeSet::new();
        while let Some(connection_id) = pending.pop() {
            if !component.insert(connection_id.clone()) {
                continue;
            }
            unvisited.remove(&connection_id);
            let connection = connections
                .get(&connection_id)
                .expect("validated conflict connection exists");
            pending.extend(connection.conflict_connection_ids.iter().cloned());
            if let Some(junction_id) = &connection.junction_id {
                if let Some(siblings) = junction_connections.get(junction_id) {
                    pending.extend(siblings.iter().cloned());
                }
            }
        }
        let representative = component
            .first()
            .expect("conflict component contains its seed")
            .clone();
        groups.extend(
            component
                .into_iter()
                .map(|connection_id| (connection_id, representative.clone())),
        );
    }
    groups
}

fn distance(left: [f64; 3], right: [f64; 3]) -> f64 {
    (right[0] - left[0])
        .hypot(right[1] - left[1])
        .hypot(right[2] - left[2])
}

fn sample_segment(
    start: [f64; 3],
    end: [f64; 3],
    local_distance_m: f64,
    segment_length_m: f64,
) -> TrafficRouteSample {
    let t = if segment_length_m <= ROUTE_EPSILON_M {
        0.0
    } else {
        (local_distance_m / segment_length_m).clamp(0.0, 1.0)
    };
    let position_m = [
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    ];
    TrafficRouteSample {
        position_m,
        yaw_rad: -(end[2] - start[2]).atan2(end[0] - start[0]),
    }
}
