//! Multi-floor building maps and cross-floor route planning.
//!
//! Every map in this crate is a single horizontal surface, which is the right
//! model for one floor and the wrong model for a building. An indoor service
//! robot that cannot leave its floor is a fundamentally different product from
//! one that can, and the difference is not a bigger grid: floors do not share a
//! coordinate plane, and moving between them happens only at specific places —
//! an elevator threshold, the foot of a staircase — through a device that takes
//! time and may refuse.
//!
//! A [`BuildingMap`] is therefore a set of [`Floor`]s, each with its own
//! costmap, joined by [`FloorTransition`]s that name where a robot may cross
//! and what crossing costs. [`plan_building_route`] searches that graph,
//! delegating every within-floor leg to the ordinary 2D planner, so a route is
//! a sequence of drives and crossings rather than one impossible path.
//!
//! The planner is deterministic: nodes are expanded in cost order with ties
//! broken by index, so the same building and endpoints always produce the same
//! route.

use crate::costmap::Costmap;
use crate::path::Path2d;
use crate::planner::{plan_path, GlobalPlannerConfig, PlanError};
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BinaryHeap;

/// Identifier of one floor within a building.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct FloorId(pub usize);

/// One navigable floor of a building.
///
/// The costmap is expressed in that floor's own horizontal frame; `elevation_m`
/// places the walking surface in the world so a caller can put the robot there.
#[derive(Clone, Debug)]
pub struct Floor {
    /// Identifier, unique within the building.
    pub id: FloorId,
    /// Human-readable name, such as `"1F"`.
    pub name: String,
    /// Height of the walking surface in world meters.
    pub elevation_m: f64,
    /// Navigable costmap for this floor.
    pub costmap: Costmap,
}

/// How a robot crosses between two floors.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransitionKind {
    /// A car the robot rides, which must be summoned and boarded.
    Elevator,
    /// Steps, which most wheeled robots cannot use.
    Stairs,
    /// A continuous slope.
    Ramp,
}

/// A place where a robot may cross between two floors.
///
/// The two poses are the points a robot must reach to use it: where it boards
/// on [`Self::from`], and where it ends up on [`Self::to`]. `cost_s` is the
/// expected traversal time, which for an elevator includes summoning and
/// waiting, so the planner can prefer a nearby ramp over a distant lift.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloorTransition {
    /// Human-readable name, such as `"north lift"`.
    pub name: String,
    /// How the crossing is made.
    pub kind: TransitionKind,
    /// Floor the robot leaves.
    pub from: FloorId,
    /// Floor the robot arrives on.
    pub to: FloorId,
    /// Boarding point in the `from` floor's frame, in meters.
    pub from_point_m: Vec3,
    /// Alighting point in the `to` floor's frame, in meters.
    pub to_point_m: Vec3,
    /// Expected traversal cost in seconds.
    pub cost_s: f64,
    /// Whether the crossing may also be made in the reverse direction.
    pub bidirectional: bool,
}

/// A position on a named floor.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloorPosition {
    /// Floor the position is on.
    pub floor: FloorId,
    /// Position in that floor's frame, in meters.
    pub point_m: Vec3,
}

impl FloorPosition {
    /// Creates a floor position.
    pub fn new(floor: FloorId, point_m: Vec3) -> Self {
        Self { floor, point_m }
    }
}

/// Error raised when building or searching a multi-floor map.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum BuildingError {
    /// Two floors declared the same identifier.
    #[error("floor {0:?} is declared more than once")]
    DuplicateFloor(FloorId),
    /// A transition or query named a floor the building does not have.
    #[error("floor {0:?} is not part of this building")]
    UnknownFloor(FloorId),
    /// A transition joined a floor to itself.
    #[error("transition `{0}` joins a floor to itself")]
    SelfTransition(String),
    /// A cost or coordinate was not finite, or a cost was negative.
    #[error("building costs and coordinates must be finite, with non-negative costs")]
    NonFinite,
    /// A transition endpoint was outside its floor's costmap.
    #[error("transition `{0}` has an endpoint outside its floor")]
    TransitionOutsideFloor(String),
    /// No sequence of drives and crossings reaches the goal.
    #[error("no route reaches the goal floor")]
    Unreachable,
    /// A within-floor leg could not be planned.
    #[error(transparent)]
    Plan(#[from] PlanError),
}

/// A set of floors joined by crossings.
#[derive(Clone, Debug)]
pub struct BuildingMap {
    floors: Vec<Floor>,
    transitions: Vec<FloorTransition>,
}

impl BuildingMap {
    /// Builds a validated building map.
    ///
    /// Rejects duplicate floors, transitions naming unknown floors, transitions
    /// that join a floor to itself, non-finite or negative costs, and endpoints
    /// that fall outside the floor they claim to be on. An endpoint outside its
    /// floor would otherwise produce a route the robot cannot start.
    pub fn new(
        floors: Vec<Floor>,
        transitions: Vec<FloorTransition>,
    ) -> Result<Self, BuildingError> {
        for (index, floor) in floors.iter().enumerate() {
            if floors[..index].iter().any(|other| other.id == floor.id) {
                return Err(BuildingError::DuplicateFloor(floor.id));
            }
            if !floor.elevation_m.is_finite() {
                return Err(BuildingError::NonFinite);
            }
        }
        let map = Self {
            floors,
            transitions,
        };
        for transition in &map.transitions {
            let from = map
                .floor(transition.from)
                .ok_or(BuildingError::UnknownFloor(transition.from))?;
            let to = map
                .floor(transition.to)
                .ok_or(BuildingError::UnknownFloor(transition.to))?;
            if transition.from == transition.to {
                return Err(BuildingError::SelfTransition(transition.name.clone()));
            }
            if !transition.cost_s.is_finite()
                || transition.cost_s < 0.0
                || !transition.from_point_m.is_finite()
                || !transition.to_point_m.is_finite()
            {
                return Err(BuildingError::NonFinite);
            }
            if from
                .costmap
                .world_to_grid(transition.from_point_m)
                .is_none()
                || to.costmap.world_to_grid(transition.to_point_m).is_none()
            {
                return Err(BuildingError::TransitionOutsideFloor(
                    transition.name.clone(),
                ));
            }
        }
        Ok(map)
    }

    /// Returns the floors.
    pub fn floors(&self) -> &[Floor] {
        &self.floors
    }

    /// Returns the transitions.
    pub fn transitions(&self) -> &[FloorTransition] {
        &self.transitions
    }

    /// Returns one floor by identifier.
    pub fn floor(&self, id: FloorId) -> Option<&Floor> {
        self.floors.iter().find(|floor| floor.id == id)
    }
}

/// One step of a cross-floor route.
#[derive(Clone, Debug, PartialEq)]
pub enum RouteLeg {
    /// Driving across one floor.
    Drive {
        /// Floor being driven across.
        floor: FloorId,
        /// Planned path in that floor's frame.
        path: Path2d,
    },
    /// Crossing between floors.
    Cross {
        /// Index into [`BuildingMap::transitions`].
        transition: usize,
        /// Floor being left.
        from: FloorId,
        /// Floor being reached.
        to: FloorId,
    },
}

/// A planned route through a building.
#[derive(Clone, Debug, PartialEq)]
pub struct BuildingRoute {
    /// Drives and crossings in order.
    pub legs: Vec<RouteLeg>,
    /// Summed drive length in meters plus crossing cost in seconds.
    ///
    /// The two are added with [`RouteCosts::drive_cost_per_meter`] applied to
    /// the driving part, so the caller decides how a metre of floor compares
    /// with a second of waiting.
    pub total_cost: f64,
}

/// How driving distance trades off against crossing time.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteCosts {
    /// Cost charged per meter driven.
    pub drive_cost_per_meter: f64,
}

impl Default for RouteCosts {
    fn default() -> Self {
        // One metre of clear floor costs about as much as a second of waiting,
        // which makes a lift worth walking to only when it saves real distance.
        Self {
            drive_cost_per_meter: 1.0,
        }
    }
}

/// A searchable point: the start, the goal, or a transition endpoint.
#[derive(Clone, Copy, Debug)]
struct Node {
    floor: FloorId,
    point_m: Vec3,
}

/// Dijkstra queue entry ordered by cost, with a deterministic tie-break.
#[derive(Clone, Copy, Debug, PartialEq)]
struct QueueEntry {
    cost: f64,
    node: usize,
}

impl Eq for QueueEntry {}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reversed so the binary heap pops the cheapest first; equal costs fall
        // back to the lower node index so expansion order is deterministic.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Plans a route between two positions that may be on different floors.
///
/// Within-floor legs are planned by [`plan_path`] on that floor's costmap, so
/// obstacles, inflation and unknown-cell policy behave exactly as they do for
/// single-floor navigation. Crossings are taken whole: a route either uses a
/// transition or it does not, because a robot cannot stop halfway up a lift.
///
/// A leg that cannot be planned removes that edge rather than failing the
/// search, so a blocked corridor makes the planner prefer another staircase
/// instead of reporting the whole building unreachable.
pub fn plan_building_route(
    map: &BuildingMap,
    start: FloorPosition,
    goal: FloorPosition,
    costs: RouteCosts,
    planner: &GlobalPlannerConfig,
) -> Result<BuildingRoute, BuildingError> {
    if !costs.drive_cost_per_meter.is_finite() || costs.drive_cost_per_meter < 0.0 {
        return Err(BuildingError::NonFinite);
    }
    if !start.point_m.is_finite() || !goal.point_m.is_finite() {
        return Err(BuildingError::NonFinite);
    }
    map.floor(start.floor)
        .ok_or(BuildingError::UnknownFloor(start.floor))?;
    map.floor(goal.floor)
        .ok_or(BuildingError::UnknownFloor(goal.floor))?;

    // Node 0 is the start, node 1 the goal, then one node per usable transition
    // endpoint in declaration order.
    let mut nodes = vec![
        Node {
            floor: start.floor,
            point_m: start.point_m,
        },
        Node {
            floor: goal.floor,
            point_m: goal.point_m,
        },
    ];
    // (transition index, forward) -> (entry node, exit node)
    let mut crossings: Vec<(usize, bool, usize, usize)> = Vec::new();
    for (index, transition) in map.transitions().iter().enumerate() {
        let entry = nodes.len();
        nodes.push(Node {
            floor: transition.from,
            point_m: transition.from_point_m,
        });
        let exit = nodes.len();
        nodes.push(Node {
            floor: transition.to,
            point_m: transition.to_point_m,
        });
        crossings.push((index, true, entry, exit));
        if transition.bidirectional {
            crossings.push((index, false, exit, entry));
        }
    }

    // Drive edges between every pair of nodes sharing a floor, planned once.
    let mut edges: Vec<Vec<(usize, f64, Option<usize>)>> = vec![Vec::new(); nodes.len()];
    for from in 0..nodes.len() {
        for to in 0..nodes.len() {
            if from == to || nodes[from].floor != nodes[to].floor {
                continue;
            }
            let floor = map
                .floor(nodes[from].floor)
                .ok_or(BuildingError::UnknownFloor(nodes[from].floor))?;
            match plan_path(
                &floor.costmap,
                nodes[from].point_m,
                nodes[to].point_m,
                planner,
            ) {
                Ok(path) => {
                    let cost = path.length_m() * costs.drive_cost_per_meter;
                    edges[from].push((to, cost, None));
                }
                // A blocked leg is a missing edge, not a failed search.
                Err(_) => continue,
            }
        }
    }
    for (index, _forward, entry, exit) in &crossings {
        edges[*entry].push((*exit, map.transitions()[*index].cost_s, Some(*index)));
    }

    // Dijkstra.
    let mut best = vec![f64::INFINITY; nodes.len()];
    let mut previous: Vec<Option<(usize, Option<usize>)>> = vec![None; nodes.len()];
    let mut heap = BinaryHeap::new();
    best[0] = 0.0;
    heap.push(QueueEntry { cost: 0.0, node: 0 });
    while let Some(QueueEntry { cost, node }) = heap.pop() {
        if cost > best[node] {
            continue;
        }
        if node == 1 {
            break;
        }
        for (next, edge_cost, transition) in &edges[node] {
            let candidate = cost + edge_cost;
            if candidate < best[*next] {
                best[*next] = candidate;
                previous[*next] = Some((node, *transition));
                heap.push(QueueEntry {
                    cost: candidate,
                    node: *next,
                });
            }
        }
    }
    if !best[1].is_finite() {
        return Err(BuildingError::Unreachable);
    }

    // Walk the predecessors back and rebuild the legs in order.
    let mut reversed: Vec<(usize, usize, Option<usize>)> = Vec::new();
    let mut cursor = 1usize;
    while let Some((previous_node, transition)) = previous[cursor] {
        reversed.push((previous_node, cursor, transition));
        cursor = previous_node;
    }
    reversed.reverse();

    let mut legs = Vec::with_capacity(reversed.len());
    for (from, to, transition) in reversed {
        match transition {
            Some(index) => legs.push(RouteLeg::Cross {
                transition: index,
                from: nodes[from].floor,
                to: nodes[to].floor,
            }),
            None => {
                let floor = map
                    .floor(nodes[from].floor)
                    .ok_or(BuildingError::UnknownFloor(nodes[from].floor))?;
                let path = plan_path(
                    &floor.costmap,
                    nodes[from].point_m,
                    nodes[to].point_m,
                    planner,
                )?;
                legs.push(RouteLeg::Drive {
                    floor: nodes[from].floor,
                    path,
                });
            }
        }
    }

    Ok(BuildingRoute {
        legs,
        total_cost: best[1],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::CostmapConfig;
    use crate::grid::OccupancyGrid;
    use crate::pose2d::Pose2d;

    /// An open floor with no obstacles, 10 m x 10 m at 0.25 m resolution.
    ///
    /// Every cell is marked free: a freshly created grid is entirely unknown,
    /// and the default planner refuses to route through unknown space.
    fn open_floor(id: usize, name: &str, elevation_m: f64) -> Floor {
        let mut grid = OccupancyGrid::new(40, 40, 0.25, Pose2d::new(0.0, 0.0, 0.0)).expect("grid");
        for y in 0..40 {
            for x in 0..40 {
                let coord = crate::grid::GridCoord { x, y };
                for _ in 0..8 {
                    grid.mark_free(coord);
                }
            }
        }
        Floor {
            id: FloorId(id),
            name: name.to_string(),
            elevation_m,
            costmap: Costmap::from_occupancy(&grid, &CostmapConfig::default()).expect("costmap"),
        }
    }

    fn lift(name: &str, from: usize, to: usize, cost_s: f64) -> FloorTransition {
        FloorTransition {
            name: name.to_string(),
            kind: TransitionKind::Elevator,
            from: FloorId(from),
            to: FloorId(to),
            from_point_m: Vec3::new(1.0, 1.0, 0.0),
            to_point_m: Vec3::new(1.0, 1.0, 0.0),
            cost_s,
            bidirectional: true,
        }
    }

    fn floors_visited(route: &BuildingRoute) -> Vec<FloorId> {
        route
            .legs
            .iter()
            .filter_map(|leg| match leg {
                RouteLeg::Cross { to, .. } => Some(*to),
                RouteLeg::Drive { .. } => None,
            })
            .collect()
    }

    #[test]
    fn a_building_rejects_duplicate_floors_bad_transitions_and_stray_endpoints() {
        assert_eq!(
            BuildingMap::new(
                vec![open_floor(0, "1F", 0.0), open_floor(0, "1F", 3.5)],
                vec![]
            )
            .unwrap_err(),
            BuildingError::DuplicateFloor(FloorId(0))
        );
        let floors = || vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)];
        assert_eq!(
            BuildingMap::new(floors(), vec![lift("lift", 0, 9, 20.0)]).unwrap_err(),
            BuildingError::UnknownFloor(FloorId(9))
        );
        assert_eq!(
            BuildingMap::new(floors(), vec![lift("lift", 0, 0, 20.0)]).unwrap_err(),
            BuildingError::SelfTransition("lift".to_string())
        );
        assert_eq!(
            BuildingMap::new(floors(), vec![lift("lift", 0, 1, -1.0)]).unwrap_err(),
            BuildingError::NonFinite
        );
        let mut stray = lift("lift", 0, 1, 20.0);
        stray.to_point_m = Vec3::new(500.0, 500.0, 0.0);
        assert_eq!(
            BuildingMap::new(floors(), vec![stray]).unwrap_err(),
            BuildingError::TransitionOutsideFloor("lift".to_string())
        );
    }

    #[test]
    fn a_same_floor_route_never_takes_a_lift() {
        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![lift("lift", 0, 1, 20.0)],
        )
        .expect("building");
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(2.0, 2.0, 0.0)),
            FloorPosition::new(FloorId(0), Vec3::new(8.0, 8.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");
        assert_eq!(route.legs.len(), 1);
        assert!(matches!(
            route.legs[0],
            RouteLeg::Drive {
                floor: FloorId(0),
                ..
            }
        ));
        assert!(floors_visited(&route).is_empty());
    }

    #[test]
    fn a_cross_floor_route_drives_to_the_lift_crosses_and_drives_on() {
        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![lift("lift", 0, 1, 20.0)],
        )
        .expect("building");
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(8.0, 8.0, 0.0)),
            FloorPosition::new(FloorId(1), Vec3::new(8.0, 8.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");

        assert_eq!(route.legs.len(), 3, "drive, cross, drive: {:?}", route.legs);
        match &route.legs[0] {
            RouteLeg::Drive { floor, path } => {
                assert_eq!(*floor, FloorId(0));
                assert!(path.length_m() > 0.0);
            }
            leg => panic!("expected a drive first, got {leg:?}"),
        }
        assert_eq!(
            route.legs[1],
            RouteLeg::Cross {
                transition: 0,
                from: FloorId(0),
                to: FloorId(1),
            }
        );
        match &route.legs[2] {
            RouteLeg::Drive { floor, .. } => assert_eq!(*floor, FloorId(1)),
            leg => panic!("expected a drive last, got {leg:?}"),
        }
        // Cost is both drives plus the wait, so it exceeds either alone.
        assert!(route.total_cost > 20.0);

        // The reverse route works because the lift is bidirectional.
        let back = plan_building_route(
            &map,
            FloorPosition::new(FloorId(1), Vec3::new(8.0, 8.0, 0.0)),
            FloorPosition::new(FloorId(0), Vec3::new(8.0, 8.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("reverse route");
        assert_eq!(floors_visited(&back), vec![FloorId(0)]);
    }

    #[test]
    fn a_one_way_crossing_is_not_traversed_backwards() {
        let mut one_way = lift("escalator", 0, 1, 5.0);
        one_way.bidirectional = false;
        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![one_way],
        )
        .expect("building");

        assert!(plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
            FloorPosition::new(FloorId(1), Vec3::new(5.0, 5.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .is_ok());
        assert_eq!(
            plan_building_route(
                &map,
                FloorPosition::new(FloorId(1), Vec3::new(5.0, 5.0, 0.0)),
                FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
                RouteCosts::default(),
                &GlobalPlannerConfig::default(),
            )
            .unwrap_err(),
            BuildingError::Unreachable
        );
    }

    #[test]
    fn the_cheaper_of_two_crossings_is_chosen_and_the_choice_follows_its_cost() {
        let mut near_slow = lift("near lift", 0, 1, 120.0);
        near_slow.from_point_m = Vec3::new(1.0, 1.0, 0.0);
        near_slow.to_point_m = Vec3::new(1.0, 1.0, 0.0);
        let mut far_fast = lift("far ramp", 0, 1, 1.0);
        far_fast.kind = TransitionKind::Ramp;
        far_fast.from_point_m = Vec3::new(9.0, 9.0, 0.0);
        far_fast.to_point_m = Vec3::new(9.0, 9.0, 0.0);

        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![near_slow.clone(), far_fast.clone()],
        )
        .expect("building");
        // Starting beside the slow lift, the fast ramp is still worth the walk.
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(1.0, 1.0, 0.0)),
            FloorPosition::new(FloorId(1), Vec3::new(9.0, 9.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");
        assert!(
            matches!(
                route
                    .legs
                    .iter()
                    .find(|leg| matches!(leg, RouteLeg::Cross { .. })),
                Some(RouteLeg::Cross { transition: 1, .. })
            ),
            "expected the ramp: {:?}",
            route.legs
        );

        // Make the near lift quick and it wins instead: the choice is the cost,
        // not the declaration order.
        let mut near_fast = near_slow;
        near_fast.cost_s = 1.0;
        let mut far_slow = far_fast;
        far_slow.cost_s = 120.0;
        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![near_fast, far_slow],
        )
        .expect("building");
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(1.0, 1.0, 0.0)),
            FloorPosition::new(FloorId(1), Vec3::new(9.0, 9.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");
        assert!(
            matches!(
                route
                    .legs
                    .iter()
                    .find(|leg| matches!(leg, RouteLeg::Cross { .. })),
                Some(RouteLeg::Cross { transition: 0, .. })
            ),
            "expected the near lift: {:?}",
            route.legs
        );
    }

    #[test]
    fn a_three_floor_building_chains_crossings() {
        let map = BuildingMap::new(
            vec![
                open_floor(0, "1F", 0.0),
                open_floor(1, "2F", 3.5),
                open_floor(2, "3F", 7.0),
            ],
            vec![
                lift("lower lift", 0, 1, 10.0),
                lift("upper lift", 1, 2, 10.0),
            ],
        )
        .expect("building");
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
            FloorPosition::new(FloorId(2), Vec3::new(5.0, 5.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");
        assert_eq!(floors_visited(&route), vec![FloorId(1), FloorId(2)]);

        // Planning is deterministic for the same building and endpoints.
        let repeat = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
            FloorPosition::new(FloorId(2), Vec3::new(5.0, 5.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("repeat");
        assert_eq!(repeat, route);
    }

    #[test]
    fn a_floor_with_no_crossing_is_unreachable() {
        let map = BuildingMap::new(
            vec![open_floor(0, "1F", 0.0), open_floor(1, "2F", 3.5)],
            vec![],
        )
        .expect("building");
        assert_eq!(
            plan_building_route(
                &map,
                FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
                FloorPosition::new(FloorId(1), Vec3::new(5.0, 5.0, 0.0)),
                RouteCosts::default(),
                &GlobalPlannerConfig::default(),
            )
            .unwrap_err(),
            BuildingError::Unreachable
        );
        assert_eq!(
            plan_building_route(
                &map,
                FloorPosition::new(FloorId(0), Vec3::new(5.0, 5.0, 0.0)),
                FloorPosition::new(FloorId(7), Vec3::new(5.0, 5.0, 0.0)),
                RouteCosts::default(),
                &GlobalPlannerConfig::default(),
            )
            .unwrap_err(),
            BuildingError::UnknownFloor(FloorId(7))
        );
    }
}
