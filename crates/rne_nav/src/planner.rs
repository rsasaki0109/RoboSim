//! Grid-based global path planner (A* / Dijkstra) over a [`Costmap`].

use crate::costmap::{Costmap, COST_LETHAL, COST_NO_INFORMATION};
use crate::grid::GridCoord;
use crate::path::Path2d;
use rne_math::Vec3;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::SQRT_2;
use thiserror::Error;

const NEIGHBOURS: [(isize, isize); 8] = [
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
];

/// Global planner configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GlobalPlannerConfig {
    /// Whether unknown cells are traversable.
    pub allow_unknown: bool,
    /// Multiplier applied to normalized cell cost when relaxing an edge.
    pub cost_weight: f64,
    /// Heuristic weight; `1.0` is A*, `0.0` degrades to Dijkstra.
    pub heuristic_weight: f64,
    /// Maximum number of expanded nodes before giving up.
    pub max_iterations: usize,
}

impl Default for GlobalPlannerConfig {
    fn default() -> Self {
        Self {
            allow_unknown: false,
            cost_weight: 0.5,
            heuristic_weight: 1.0,
            max_iterations: 1_000_000,
        }
    }
}

/// Error returned by the global planner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum PlanError {
    /// The start position is outside the costmap.
    #[error("start position is outside the costmap")]
    StartOutsideMap,
    /// The goal position is outside the costmap.
    #[error("goal position is outside the costmap")]
    GoalOutsideMap,
    /// The start cell is not traversable.
    #[error("start cell is not traversable")]
    StartBlocked,
    /// The goal cell is not traversable.
    #[error("goal cell is not traversable")]
    GoalBlocked,
    /// No path connects the start and goal cells.
    #[error("no path found between start and goal")]
    NoPath,
    /// The node expansion limit was reached.
    #[error("global planner exceeded its iteration limit")]
    IterationLimit,
    /// The planner configuration contained a non-finite value.
    #[error("global planner configuration must be finite")]
    NonFiniteConfig,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct OpenNode {
    f: f64,
    g: f64,
    coord: GridCoord,
}

impl Eq for OpenNode {}

impl Ord for OpenNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed so `BinaryHeap` behaves as a min-heap on `f`, with
        // deterministic tie-breaking on `g` and then coordinate.
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| other.g.total_cmp(&self.g))
            .then_with(|| other.coord.cmp(&self.coord))
    }
}

impl PartialOrd for OpenNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Plans a path between two world positions on a costmap.
pub fn plan_path(
    costmap: &Costmap,
    start_m: Vec3,
    goal_m: Vec3,
    config: &GlobalPlannerConfig,
) -> Result<Path2d, PlanError> {
    if !config.cost_weight.is_finite()
        || !config.heuristic_weight.is_finite()
        || config.cost_weight < 0.0
        || config.heuristic_weight < 0.0
    {
        return Err(PlanError::NonFiniteConfig);
    }

    let start = costmap
        .world_to_grid(start_m)
        .ok_or(PlanError::StartOutsideMap)?;
    let goal = costmap
        .world_to_grid(goal_m)
        .ok_or(PlanError::GoalOutsideMap)?;
    if !traversable(costmap, start, config) {
        return Err(PlanError::StartBlocked);
    }
    if !traversable(costmap, goal, config) {
        return Err(PlanError::GoalBlocked);
    }
    if start == goal {
        return Ok(Path2d::from_points(&[start_m, goal_m]));
    }

    let width = costmap.width();
    let height = costmap.height();
    let resolution = costmap.resolution_m();
    let cell_count = width * height;
    let index = |coord: GridCoord| coord.y as usize * width + coord.x as usize;

    let mut g_score = vec![f64::INFINITY; cell_count];
    let mut came_from = vec![None; cell_count];
    let mut closed = vec![false; cell_count];
    g_score[index(start)] = 0.0;

    let mut open = BinaryHeap::new();
    open.push(OpenNode {
        f: heuristic(start, goal, resolution, config.heuristic_weight),
        g: 0.0,
        coord: start,
    });

    let mut expanded = 0_usize;
    while let Some(node) = open.pop() {
        if node.coord == goal {
            return Ok(reconstruct(
                costmap, &came_from, start, goal, start_m, goal_m,
            ));
        }
        let node_index = index(node.coord);
        if closed[node_index] {
            continue;
        }
        closed[node_index] = true;
        expanded += 1;
        if expanded > config.max_iterations {
            return Err(PlanError::IterationLimit);
        }

        for (dx, dy) in NEIGHBOURS {
            let neighbour = GridCoord {
                x: node.coord.x + dx,
                y: node.coord.y + dy,
            };
            if !costmap.contains(neighbour) || !traversable(costmap, neighbour, config) {
                continue;
            }
            let neighbour_index = index(neighbour);
            if closed[neighbour_index] {
                continue;
            }
            let base = if dx != 0 && dy != 0 {
                resolution * SQRT_2
            } else {
                resolution
            };
            let cost = costmap.cost_at(neighbour).unwrap_or(COST_LETHAL);
            let step = base * (1.0 + config.cost_weight * cost as f64 / COST_LETHAL as f64);
            let tentative = g_score[node_index] + step;
            if tentative < g_score[neighbour_index] {
                g_score[neighbour_index] = tentative;
                came_from[neighbour_index] = Some(node.coord);
                open.push(OpenNode {
                    f: tentative + heuristic(neighbour, goal, resolution, config.heuristic_weight),
                    g: tentative,
                    coord: neighbour,
                });
            }
        }
    }

    Err(PlanError::NoPath)
}

fn traversable(costmap: &Costmap, coord: GridCoord, config: &GlobalPlannerConfig) -> bool {
    match costmap.cost_at(coord) {
        Some(COST_LETHAL) => false,
        Some(COST_NO_INFORMATION) => config.allow_unknown,
        Some(_) => true,
        None => false,
    }
}

fn heuristic(coord: GridCoord, goal: GridCoord, resolution: f64, weight: f64) -> f64 {
    let dx = (coord.x - goal.x).unsigned_abs() as f64;
    let dy = (coord.y - goal.y).unsigned_abs() as f64;
    let octile = (dx + dy) + (SQRT_2 - 2.0) * dx.min(dy);
    octile * resolution * weight
}

fn reconstruct(
    costmap: &Costmap,
    came_from: &[Option<GridCoord>],
    start: GridCoord,
    goal: GridCoord,
    start_m: Vec3,
    goal_m: Vec3,
) -> Path2d {
    let width = costmap.width();
    let index = |coord: GridCoord| coord.y as usize * width + coord.x as usize;

    let mut cells: Vec<GridCoord> = Vec::new();
    let mut current = goal;
    while current != start {
        cells.push(current);
        match came_from[index(current)] {
            Some(previous) => current = previous,
            None => break,
        }
    }
    cells.push(start);
    cells.reverse();

    let mut points: Vec<Vec3> = cells
        .iter()
        .map(|coord| costmap.grid_to_world(*coord))
        .collect();
    if let Some(first) = points.first_mut() {
        *first = start_m;
    }
    if let Some(last) = points.last_mut() {
        *last = goal_m;
    }
    Path2d::from_points(&points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::CostmapConfig;
    use crate::grid::OccupancyGrid;
    use crate::pose2d::Pose2d;

    fn open_costmap() -> Costmap {
        let mut grid = OccupancyGrid::new(40, 40, 0.1, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap()
    }

    #[test]
    fn plans_through_open_space() {
        let costmap = open_costmap();
        let path = plan_path(
            &costmap,
            Vec3::new(-1.5, 0.0, 0.0),
            Vec3::new(1.5, 0.0, 0.0),
            &GlobalPlannerConfig::default(),
        )
        .unwrap();
        assert!(path.len() >= 2);
        assert!((path.goal().unwrap().x_m - 1.5).abs() < 1e-9);
        assert!(path.length_m() < 3.5, "length={}", path.length_m());
    }

    #[test]
    fn routes_around_a_wall() {
        let mut grid = OccupancyGrid::new(40, 40, 0.1, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        // A vertical wall at x = 0 that blocks the straight line at y = 0.
        for y in 5..26 {
            let coord = GridCoord { x: 20, y };
            grid.reset(coord);
            grid.mark_occupied(coord);
        }
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        let path = plan_path(
            &costmap,
            Vec3::new(-1.5, 0.0, 0.0),
            Vec3::new(1.5, 0.0, 0.0),
            &GlobalPlannerConfig::default(),
        )
        .unwrap();
        // It must leave the straight line to pass around the wall.
        assert!(path.length_m() > 3.1, "length={}", path.length_m());
    }

    #[test]
    fn reports_no_path_when_walled_off() {
        let mut grid = OccupancyGrid::new(20, 20, 0.1, Pose2d::new(-1.0, -1.0, 0.0)).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
                if x == 10 {
                    grid.reset(coord);
                    grid.mark_occupied(coord);
                }
            }
        }
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        assert_eq!(
            plan_path(
                &costmap,
                Vec3::new(-0.8, 0.0, 0.0),
                Vec3::new(0.8, 0.0, 0.0),
                &GlobalPlannerConfig::default(),
            ),
            Err(PlanError::NoPath)
        );
    }

    #[test]
    fn rejects_blocked_goal() {
        let mut grid = OccupancyGrid::new(10, 10, 0.5, Pose2d::IDENTITY).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        let blocked = GridCoord { x: 5, y: 5 };
        grid.reset(blocked);
        grid.mark_occupied(blocked);
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        assert_eq!(
            plan_path(
                &costmap,
                Vec3::new(1.25, 1.25, 0.0),
                Vec3::new(2.75, 2.75, 0.0),
                &GlobalPlannerConfig::default(),
            ),
            Err(PlanError::GoalBlocked)
        );
    }
}
