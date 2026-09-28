//! A lifelong occupancy map that favours what was seen most recently.
//!
//! Fusing every visit's scans into one log-odds grid treats all evidence as
//! equally current. A pallet mapped on Monday and moved on Tuesday then stays
//! in Monday's spot: Tuesday's free-space rays only cancel as much evidence as
//! Monday's returns built up, and a place scanned on many earlier visits takes
//! as many later ones to clear. The map describes the building's history, not
//! its present.
//!
//! [`build_recency_map`] instead folds sessions in order and, before adding a
//! session's evidence to a cell that session observed, scales what the cell
//! already held by [`RecencyMapConfig::retain`]. With `retain` below one, a
//! single clear visit outweighs any number of older contrary ones, while cells
//! the new session did not see keep their previous state untouched: not
//! looking at a place is not evidence that it changed.
//!
//! The map is rebuilt from keyframe scans at the lifelong graph's *current*
//! estimates, so every re-optimization, merge or prune is reflected in the
//! geometry rather than frozen into a grid integrated along the way. Between
//! consecutive sessions it reports which cells turned occupied and which
//! turned free.

use crate::session::{LifelongPoseGraph, SessionId};
use rne_nav::{
    integrate_scan, GridCoord, GridError, LaserScan2d, OccupancyGrid, Pose2d, ScanIntegrationConfig,
};
use serde::{Deserialize, Serialize};

/// A keyframe scan attached to one node of a lifelong pose graph.
#[derive(Clone, Debug, PartialEq)]
pub struct MapKeyframe {
    /// Node of the lifelong graph that recorded the scan.
    pub node: usize,
    /// The scan, in the sensor frame.
    pub scan: LaserScan2d,
    /// Sensor pose relative to the robot base.
    pub sensor_from_base: Pose2d,
}

/// Settings for [`build_recency_map`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecencyMapConfig {
    /// Per-scan occupancy integration.
    pub scan: ScanIntegrationConfig,
    /// Factor applied to a cell's accumulated evidence before a later session
    /// that observed the cell adds its own; in `[0, 1]`. One is ordinary
    /// fusion; zero keeps only the latest observing session.
    pub retain: f64,
    /// Probability at or above which a cell counts as occupied.
    pub occupied_probability: f64,
    /// Probability at or below which a cell counts as free.
    pub free_probability: f64,
    /// A cell is reported as vanished only when nothing within this distance
    /// is still occupied, and as appeared only when nothing within it was
    /// occupied before, in meters. Two sessions registered a few centimetres
    /// apart shift every wall by a cell; without this, each such shift reads
    /// as a wall vanishing next to a wall appearing.
    pub change_clearance_m: f64,
}

impl Default for RecencyMapConfig {
    fn default() -> Self {
        Self {
            scan: ScanIntegrationConfig::default(),
            retain: 0.25,
            occupied_probability: 0.65,
            free_probability: 0.35,
            change_clearance_m: 0.15,
        }
    }
}

impl RecencyMapConfig {
    fn is_valid(&self) -> bool {
        (0.0..=1.0).contains(&self.retain)
            && self.free_probability.is_finite()
            && self.occupied_probability.is_finite()
            && 0.0 < self.free_probability
            && self.free_probability < self.occupied_probability
            && self.occupied_probability < 1.0
            && self.change_clearance_m.is_finite()
            && self.change_clearance_m >= 0.0
    }
}

/// Cells whose state a session changed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionChanges {
    /// The session that changed them.
    pub session: SessionId,
    /// Cells that were free before this session and are occupied after it.
    pub appeared: Vec<GridCoord>,
    /// Cells that were occupied before this session and are free after it.
    pub vanished: Vec<GridCoord>,
    /// Cells this session observed at all.
    pub observed: usize,
}

/// The map [`build_recency_map`] produced and the changes along the way.
#[derive(Clone, Debug, PartialEq)]
pub struct RecencyMap {
    /// The fused map after the last session.
    pub grid: OccupancyGrid,
    /// For each session after the first, the cells it changed.
    pub changes: Vec<SessionChanges>,
}

/// Error raised while building a recency map.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RecencyMapError {
    /// A threshold or the retain factor was out of range.
    #[error("recency map configuration is out of range")]
    InvalidConfig,
    /// A keyframe referenced a node the graph does not have.
    #[error("keyframe references node {0} which does not exist")]
    UnknownNode(usize),
    /// Scan integration failed.
    #[error(transparent)]
    Grid(#[from] GridError),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CellState {
    Unknown,
    Free,
    Occupied,
}

/// Rebuilds the lifelong map from keyframes at the graph's current estimates.
///
/// `template` supplies the grid's size, resolution and origin; its contents are
/// ignored. Keyframes may be given in any order: they are grouped by the
/// session of their node and folded session by session, and within a session
/// in node order, so the result is deterministic.
pub fn build_recency_map(
    template: &OccupancyGrid,
    graph: &LifelongPoseGraph,
    keyframes: &[MapKeyframe],
    config: &RecencyMapConfig,
) -> Result<RecencyMap, RecencyMapError> {
    if !config.is_valid() {
        return Err(RecencyMapError::InvalidConfig);
    }
    let blank = || {
        OccupancyGrid::new(
            template.width(),
            template.height(),
            template.resolution_m(),
            template.origin(),
        )
    };
    let cells = template.width() * template.height();
    let mut sorted: Vec<&MapKeyframe> = keyframes.iter().collect();
    for keyframe in &sorted {
        if graph.node_session(keyframe.node).is_none() {
            return Err(RecencyMapError::UnknownNode(keyframe.node));
        }
    }
    sorted.sort_by_key(|keyframe| (graph.node_session(keyframe.node), keyframe.node));

    let occupied_log_odds = log_odds(config.occupied_probability);
    let free_log_odds = log_odds(config.free_probability);
    let state_of = |evidence: f64| {
        if evidence >= occupied_log_odds {
            CellState::Occupied
        } else if evidence <= free_log_odds {
            CellState::Free
        } else {
            CellState::Unknown
        }
    };

    let mut fused = vec![0.0_f64; cells];
    let mut changes = Vec::new();
    let mut first = true;
    let mut start = 0;
    while start < sorted.len() {
        let session = graph.node_session(sorted[start].node);
        let end = start
            + sorted[start..]
                .iter()
                .take_while(|keyframe| graph.node_session(keyframe.node) == session)
                .count();
        let mut evidence = blank()?;
        for keyframe in &sorted[start..end] {
            let base = graph
                .graph()
                .node(keyframe.node)
                .ok_or(RecencyMapError::UnknownNode(keyframe.node))?;
            integrate_scan(
                &mut evidence,
                &keyframe.scan,
                base.compose(keyframe.sensor_from_base),
                &config.scan,
            )?;
        }
        let mut report = SessionChanges {
            // Every keyframe's node was checked against the graph above.
            session: session.expect("keyframe node has a session"),
            appeared: Vec::new(),
            vanished: Vec::new(),
            observed: 0,
        };
        let before: Vec<CellState> = fused.iter().map(|value| state_of(*value)).collect();
        let mut candidates = Vec::new();
        for (index, raw) in evidence.log_odds().iter().enumerate() {
            if *raw == 0 {
                continue;
            }
            report.observed += 1;
            fused[index] = fused[index] * config.retain + stored_to_log_odds(*raw);
            let after = state_of(fused[index]);
            if before[index] != after {
                candidates.push((index, before[index], after));
            }
        }
        let after: Vec<CellState> = fused.iter().map(|value| state_of(*value)).collect();
        let reach = (config.change_clearance_m / template.resolution_m()).round() as isize;
        let width = template.width() as isize;
        let height = template.height() as isize;
        let any_occupied_near = |states: &[CellState], index: usize| {
            let (x, y) = ((index as isize) % width, (index as isize) / width);
            (-reach..=reach).any(|dy| {
                (-reach..=reach).any(|dx| {
                    let (nx, ny) = (x + dx, y + dy);
                    (dx, dy) != (0, 0)
                        && (0..width).contains(&nx)
                        && (0..height).contains(&ny)
                        && states[(ny * width + nx) as usize] == CellState::Occupied
                })
            })
        };
        for (index, was, now) in candidates {
            let coord = GridCoord {
                x: (index % template.width()) as isize,
                y: (index / template.width()) as isize,
            };
            match (was, now) {
                (CellState::Free, CellState::Occupied) if !any_occupied_near(&before, index) => {
                    report.appeared.push(coord);
                }
                (CellState::Occupied, CellState::Free) if !any_occupied_near(&after, index) => {
                    report.vanished.push(coord);
                }
                _ => {}
            }
        }
        if !first {
            changes.push(report);
        }
        first = false;
        start = end;
    }

    let grid = OccupancyGrid::from_log_odds(
        template.width(),
        template.height(),
        template.resolution_m(),
        template.origin(),
        fused
            .iter()
            .map(|value| log_odds_to_stored(*value))
            .collect(),
    )?;
    Ok(RecencyMap { grid, changes })
}

fn log_odds(probability: f64) -> f64 {
    (probability / (1.0 - probability)).ln()
}

/// The grid stores log-odds as fixed-point `i16` at this scale (`rne_nav`'s
/// crate-private `LOG_ODDS_SCALE`); a test pins the two together through the
/// grid's public `probability`.
const GRID_LOG_ODDS_SCALE: f64 = 1000.0;

fn stored_to_log_odds(stored: i16) -> f64 {
    f64::from(stored) / GRID_LOG_ODDS_SCALE
}

fn log_odds_to_stored(value: f64) -> i16 {
    (value * GRID_LOG_ODDS_SCALE).round().clamp(
        f64::from(rne_nav::grid::MIN_LOG_ODDS),
        f64::from(rne_nav::grid::MAX_LOG_ODDS),
    ) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose_graph::PoseGraph;
    use crate::session::{MergeOptions, SessionConstraint};
    use rne_math::Vec3;

    const KEYFRAMES_PER_VISIT: usize = 4;

    /// A narrow fan of beams straight ahead that all return at `range_m`.
    fn fan(range_m: f64) -> LaserScan2d {
        let beams = 9;
        LaserScan2d {
            time_s: 0.0,
            frame: rne_nav::FrameId::new("laser"),
            angle_min_rad: -0.04,
            angle_increment_rad: 0.01,
            range_min_m: 0.05,
            range_max_m: 10.0,
            ranges_m: vec![range_m; beams],
        }
    }

    fn template() -> OccupancyGrid {
        OccupancyGrid::new(80, 40, 0.1, Pose2d::new(-1.0, -2.0, 0.0)).expect("grid")
    }

    /// A graph of one node per visit at the origin, every visit a separate
    /// session, with each visit's keyframes all taken from that node.
    fn visits(ranges_m: &[f64]) -> (LifelongPoseGraph, Vec<MapKeyframe>) {
        let single = || {
            let mut graph = PoseGraph::new();
            graph.add_node(Pose2d::IDENTITY);
            graph
        };
        let mut lifelong = LifelongPoseGraph::from_first_session(single());
        for _ in 1..ranges_m.len() {
            lifelong
                .merge_session(
                    &single(),
                    &[SessionConstraint::new(
                        0,
                        0,
                        Pose2d::IDENTITY,
                        (100.0, 100.0, 100.0),
                    )],
                    MergeOptions::default(),
                )
                .expect("merge");
        }
        let keyframes = ranges_m
            .iter()
            .enumerate()
            .flat_map(|(node, range_m)| {
                (0..KEYFRAMES_PER_VISIT).map(move |_| MapKeyframe {
                    node,
                    scan: fan(*range_m),
                    sensor_from_base: Pose2d::IDENTITY,
                })
            })
            .collect();
        (lifelong, keyframes)
    }

    fn state_at(grid: &OccupancyGrid, x_m: f64) -> f64 {
        let coord = grid
            .world_to_grid(Vec3::new(x_m, 0.0, 0.0))
            .expect("in grid");
        grid.probability(coord).expect("probability")
    }

    #[test]
    fn the_fixed_point_scale_matches_the_grid() {
        let grid =
            OccupancyGrid::from_log_odds(1, 1, 0.1, Pose2d::IDENTITY, vec![850]).expect("grid");
        let expected = 1.0 / (1.0 + (-stored_to_log_odds(850)).exp());
        let actual = grid.probability(GridCoord { x: 0, y: 0 }).expect("cell");
        assert!((actual - expected).abs() < 1e-6, "{actual} vs {expected}");
    }

    #[test]
    fn a_moved_object_leaves_the_recency_map_but_haunts_plain_fusion() {
        // Visit 0 sees an object at 2 m; visit 1 sees through to a wall at 5 m.
        let (graph, keyframes) = visits(&[2.0, 5.0]);
        let plain = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig {
                retain: 1.0,
                ..RecencyMapConfig::default()
            },
        )
        .expect("plain");
        let recency = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig::default(),
        )
        .expect("recency");
        assert!(
            state_at(&plain.grid, 2.0) >= 0.65,
            "plain fusion keeps the ghost"
        );
        assert!(
            state_at(&recency.grid, 2.0) <= 0.35,
            "recency map clears it"
        );
        let changes = &recency.changes[0];
        assert_eq!(changes.session, SessionId(1));
        assert!(!changes.vanished.is_empty());
        assert!(changes
            .vanished
            .iter()
            .all(|coord| (recency.grid.grid_to_world(*coord).x - 2.0).abs() < 0.15));
    }

    #[test]
    fn an_object_placed_in_seen_free_space_is_reported_as_appeared() {
        let (graph, keyframes) = visits(&[5.0, 2.0]);
        let map = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig::default(),
        )
        .expect("map");
        assert!(state_at(&map.grid, 2.0) >= 0.65);
        assert!(map.changes[0]
            .appeared
            .iter()
            .any(|coord| { (map.grid.grid_to_world(*coord).x - 2.0).abs() < 0.1 }));
    }

    #[test]
    fn a_wall_registered_one_cell_off_is_not_a_change() {
        // The same wall, seen 5 cm further away the second time.
        let (graph, keyframes) = visits(&[2.0, 2.05]);
        let map = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig::default(),
        )
        .expect("map");
        assert!(map.changes[0].appeared.is_empty(), "{:?}", map.changes[0]);
        assert!(map.changes[0].vanished.is_empty(), "{:?}", map.changes[0]);
        let unfiltered = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig {
                change_clearance_m: 0.0,
                ..RecencyMapConfig::default()
            },
        )
        .expect("map");
        assert!(
            !unfiltered.changes[0].appeared.is_empty()
                || !unfiltered.changes[0].vanished.is_empty(),
            "without the clearance the shift reads as a change"
        );
    }

    #[test]
    fn a_place_the_new_session_did_not_see_keeps_its_state() {
        let (graph, mut keyframes) = visits(&[2.0, 2.0]);
        // The second visit looks the other way: nothing it sees overlaps.
        for keyframe in keyframes.iter_mut().filter(|keyframe| keyframe.node == 1) {
            keyframe.sensor_from_base = Pose2d::new(0.0, 0.0, std::f64::consts::PI);
        }
        let map = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig::default(),
        )
        .expect("map");
        assert!(state_at(&map.grid, 2.0) >= 0.65);
        assert!(map.changes[0].vanished.is_empty());
    }

    #[test]
    fn keyframes_follow_the_graph_after_it_moves() {
        let (mut graph, keyframes) = visits(&[2.0]);
        let mut moved = graph.graph().clone();
        moved.set_node(0, Pose2d::new(1.0, 0.0, 0.0));
        graph = LifelongPoseGraph::from_first_session(moved);
        let map = build_recency_map(
            &template(),
            &graph,
            &keyframes,
            &RecencyMapConfig::default(),
        )
        .expect("map");
        assert!(
            state_at(&map.grid, 3.0) >= 0.65,
            "the return moves with its node"
        );
        assert!(state_at(&map.grid, 2.0) < 0.65);
    }

    #[test]
    fn bad_configuration_and_unknown_nodes_are_rejected() {
        let (graph, mut keyframes) = visits(&[2.0]);
        let bad = RecencyMapConfig {
            retain: 1.5,
            ..RecencyMapConfig::default()
        };
        assert_eq!(
            build_recency_map(&template(), &graph, &keyframes, &bad),
            Err(RecencyMapError::InvalidConfig)
        );
        keyframes[0].node = 7;
        assert_eq!(
            build_recency_map(
                &template(),
                &graph,
                &keyframes,
                &RecencyMapConfig::default()
            ),
            Err(RecencyMapError::UnknownNode(7))
        );
    }
}
