//! Traffic runtime resources.

use crate::TrafficId;
use bevy_ecs::prelude::Resource;
use std::collections::BTreeMap;
use thiserror::Error;

const ROUTE_EPSILON_M: f64 = 1.0e-9;

/// Per-world deterministic traffic runtime state.
#[derive(Clone, Debug, Default, PartialEq, Eq, Resource)]
pub struct TrafficRuntime {
    step_index: u64,
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
}

/// One deterministic polyline route shared by traffic actors.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficRoute {
    id: TrafficId,
    path_m: Vec<[f64; 3]>,
    cumulative_distance_m: Vec<f64>,
    total_length_m: f64,
    closed: bool,
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
