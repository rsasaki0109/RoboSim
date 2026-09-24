//! Online 2D SLAM front-end: odometry prediction, scan matching, and mapping.

use crate::likelihood::{LikelihoodConfig, LikelihoodField};
use crate::pose_graph::{PoseGraph, PoseGraphEdge};
use crate::scan_match::{scan_points_2d, ScanMatchConfig, ScanMatcher};
use rne_nav::{
    integrate_scan, GridError, LaserScan2d, OccupancyGrid, Pose2d, ScanIntegrationConfig,
    ScanIntegrationReport,
};
use serde::{Deserialize, Serialize};

/// Online SLAM configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SlamConfig {
    /// Occupancy integration settings.
    pub scan: ScanIntegrationConfig,
    /// Likelihood field settings.
    pub likelihood: LikelihoodConfig,
    /// Scan matcher settings.
    pub matcher: ScanMatchConfig,
    /// Maximum number of beams used for matching.
    pub max_beams: usize,
    /// Number of most recent poses retained for loop-closure search.
    pub loop_candidates: usize,
    /// Minimum node separation before a pose is considered for loop closure.
    pub loop_min_separation: usize,
    /// Score above which a historical pose is accepted as a loop closure.
    pub loop_min_score: f64,
    /// Maximum distance in meters between the matched closure and the
    /// odometry-predicted pose before the closure is rejected as an outlier.
    pub loop_max_translation_residual_m: f64,
    /// Maximum yaw error in radians for the same consistency gate.
    pub loop_max_rotation_residual_rad: f64,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            scan: ScanIntegrationConfig::default(),
            likelihood: LikelihoodConfig::default(),
            matcher: ScanMatchConfig::default(),
            max_beams: 180,
            loop_candidates: 8,
            loop_min_separation: 10,
            loop_min_score: 0.55,
            loop_max_translation_residual_m: 0.75,
            loop_max_rotation_residual_rad: 0.5,
        }
    }
}

/// Outcome of processing one scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlamUpdate {
    /// Corrected base pose in the map frame.
    pub pose: Pose2d,
    /// Whether scan matching was applied.
    pub matched: bool,
    /// Mean likelihood of the accepted match, or zero when unmatched.
    pub match_score: f64,
    /// Number of loop closures added on this update.
    pub loop_closures: usize,
    /// Occupancy integration report.
    pub report: ScanIntegrationReport,
}

/// Deterministic online 2D SLAM estimator with loop closure.
///
/// Each scan is predicted from the odometry delta, refined against a
/// likelihood field built from the map so far, then integrated into the map.
/// When the pose revisits an earlier location, a loop-closure edge is added and
/// the pose graph is re-optimized. No wall-clock time or random state is used.
#[derive(Clone, Debug)]
pub struct Slam2d {
    grid: OccupancyGrid,
    matcher: ScanMatcher,
    config: SlamConfig,
    pose: Pose2d,
    previous_odom: Option<Pose2d>,
    scans_processed: usize,
    graph: PoseGraph,
    loop_edges: Vec<usize>,
}

impl Slam2d {
    /// Creates an estimator over a starting map and pose.
    pub fn new(grid: OccupancyGrid, config: SlamConfig) -> Self {
        Self {
            grid,
            matcher: ScanMatcher::new(config.matcher),
            config,
            pose: Pose2d::IDENTITY,
            previous_odom: None,
            scans_processed: 0,
            graph: PoseGraph::new(),
            loop_edges: Vec::new(),
        }
    }

    /// Current estimated base pose in the map frame.
    pub fn pose(&self) -> Pose2d {
        self.pose
    }

    /// Overrides the estimated pose.
    pub fn set_pose(&mut self, pose: Pose2d) {
        self.pose = pose;
    }

    /// Current occupancy grid.
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }

    /// Number of scans processed.
    pub fn scans_processed(&self) -> usize {
        self.scans_processed
    }

    /// The underlying pose graph.
    pub fn graph(&self) -> &PoseGraph {
        &self.graph
    }

    /// Indices of edges added as loop closures, sorted ascending.
    pub fn loop_edge_indices(&self) -> &[usize] {
        &self.loop_edges
    }

    /// Processes a scan with its odometry pose and sensor mounting transform.
    pub fn process(
        &mut self,
        scan: &LaserScan2d,
        odom_pose: Pose2d,
        sensor_from_base: Pose2d,
    ) -> Result<SlamUpdate, GridError> {
        let predicted = match self.previous_odom {
            Some(previous) => self.pose.compose(previous.inverse().compose(odom_pose)),
            // The first scan seeds the map at the odometry pose.
            None => odom_pose,
        };

        let points = scan_points_2d(scan, self.config.max_beams);
        let mut matched = false;
        let mut match_score = 0.0;
        let mut corrected = predicted;
        if self.scans_processed > 0 && points.len() >= self.config.matcher.min_valid_beams {
            let field = LikelihoodField::from_occupancy(&self.grid, &self.config.likelihood)?;
            if let Some(result) =
                self.matcher
                    .match_scan(&field, &points, sensor_from_base, predicted)
            {
                corrected = result.pose;
                match_score = result.score;
                matched = true;
            }
        }

        let sensor_pose_world = corrected.compose(sensor_from_base);
        let report = integrate_scan(&mut self.grid, scan, sensor_pose_world, &self.config.scan)?;

        let node = self.graph.add_node(corrected);
        if let Some(previous_odom) = self.previous_odom {
            let measurement = previous_odom.inverse().compose(odom_pose);
            self.graph
                .add_edge(PoseGraphEdge::odometry(node - 1, node, measurement));
        }

        let mut loop_closures = 0;
        if matched {
            loop_closures = self.try_loop_closure(&points, sensor_from_base, predicted)?;
        }

        self.pose = corrected;
        self.previous_odom = Some(odom_pose);
        self.scans_processed += 1;

        Ok(SlamUpdate {
            pose: corrected,
            matched,
            match_score,
            loop_closures,
            report,
        })
    }

    fn try_loop_closure(
        &mut self,
        points: &[rne_math::Vec3],
        sensor_from_base: Pose2d,
        predicted: Pose2d,
    ) -> Result<usize, GridError> {
        let count = self.graph.node_count();
        if count == 0 {
            return Ok(0);
        }
        let current = count - 1;
        let min_separation = self.config.loop_min_separation;
        if current < min_separation {
            return Ok(0);
        }

        let field = LikelihoodField::from_occupancy(&self.grid, &self.config.likelihood)?;
        let current_pose = self.pose;

        // Rank historical nodes by spatial proximity to the current pose, then
        // try the closest candidates first. This finds genuine revisits instead
        // of only the most recent poses.
        let mut candidates: Vec<(usize, f64)> = (0..current.saturating_sub(min_separation))
            .filter_map(|node| {
                self.graph.node(node).map(|pose| {
                    let distance = (pose.x_m - current_pose.x_m).hypot(pose.y_m - current_pose.y_m);
                    (node, distance)
                })
            })
            .collect();
        candidates.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        candidates.truncate(self.config.loop_candidates);

        let mut best: Option<(usize, Pose2d, f64)> = None;
        for (candidate, _) in candidates {
            let Some(candidate_pose) = self.graph.node(candidate) else {
                continue;
            };
            let Some(result) =
                self.matcher
                    .match_scan(&field, points, sensor_from_base, candidate_pose)
            else {
                continue;
            };
            // Reject implausible jumps: the closure must stay near the current pose.
            if (result.pose.x_m - current_pose.x_m).hypot(result.pose.y_m - current_pose.y_m)
                > self.config.matcher.linear_window_m * 2.0
            {
                continue;
            }
            let replace = best
                .as_ref()
                .map(|(_, _, score)| result.score > *score)
                .unwrap_or(true);
            if replace {
                best = Some((candidate, result.pose, result.score));
            }
        }

        let Some((candidate, pose, score)) = best else {
            return Ok(0);
        };
        if score < self.config.loop_min_score {
            return Ok(0);
        }
        // Robust gate: a scan match that lands far from the odometry prediction
        // is an outlier even if it scores highly against the map.
        if !closure_consistent(
            predicted,
            pose,
            self.config.loop_max_translation_residual_m,
            self.config.loop_max_rotation_residual_rad,
        ) {
            return Ok(0);
        }

        let measurement = self.graph.node(candidate).unwrap().inverse().compose(pose);
        let information = (1.0, 1.0, 1.0);
        self.graph.add_edge(PoseGraphEdge::loop_closure(
            candidate,
            current,
            measurement,
            information,
        ));
        let edge_index = self.graph.edge_count() - 1;
        self.loop_edges.push(edge_index);
        self.graph
            .optimize(20, 1.0e-3, 0)
            .map_err(|_| GridError::NonFinite)?;

        // Robust back-end: if the optimized loop edge still has a large residual
        // it is inconsistent with the rest of the graph, so roll it back.
        if let Some(residual) = self.graph.edge_residual(edge_index) {
            let translation = residual.0.hypot(residual.1);
            let rotation = residual.2.abs();
            if translation > self.config.loop_max_translation_residual_m
                || rotation > self.config.loop_max_rotation_residual_rad
            {
                self.graph.remove_edge(edge_index);
                self.loop_edges.pop();
                let _ = self.graph.optimize(20, 1.0e-3, 0);
                return Ok(0);
            }
        }

        // Keep the scan-matched pose authoritative; the graph is refined in the
        // background so that loop closure does not yank the live trajectory.
        Ok(1)
    }
}

/// Whether a matched loop-closure pose is consistent with the odometry
/// prediction within the given translation and rotation bounds.
pub fn closure_consistent(
    predicted: Pose2d,
    matched: Pose2d,
    max_translation_m: f64,
    max_rotation_rad: f64,
) -> bool {
    let translation = (matched.x_m - predicted.x_m).hypot(matched.y_m - predicted.y_m);
    let rotation = wrap_angle(matched.yaw_rad - predicted.yaw_rad).abs();
    translation <= max_translation_m && rotation <= max_rotation_rad
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::{FrameId, GridCoord};
    use std::f64::consts::TAU;

    fn empty_room_grid() -> OccupancyGrid {
        OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap()
    }

    fn room_scan(x_m: f64, y_m: f64) -> LaserScan2d {
        let beams = 360;
        let mut ranges = Vec::with_capacity(beams);
        for beam in 0..beams {
            let angle = TAU * beam as f64 / beams as f64;
            ranges.push(ray_to_wall(x_m, y_m, angle));
        }
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 30.0,
            ranges_m: ranges,
        }
    }

    fn ray_to_wall(x: f64, y: f64, angle: f64) -> f64 {
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for (bound, position, direction) in
            [(5.0, x, dx), (-5.0, x, dx), (3.0, y, dy), (-3.0, y, dy)]
        {
            if direction.abs() > 1.0e-9 {
                let t = (bound - position) / direction;
                if t > 0.0 {
                    best = best.min(t);
                }
            }
        }
        best
    }

    /// Scan for a room with asymmetric pillars so position is observable.
    fn landmark_scan(x_m: f64, y_m: f64) -> LaserScan2d {
        let beams = 360;
        let mut ranges = Vec::with_capacity(beams);
        for beam in 0..beams {
            let angle = TAU * beam as f64 / beams as f64;
            let wall = ray_to_wall(x_m, y_m, angle);
            let pillar = ray_to_pillar(x_m, y_m, angle);
            ranges.push(wall.min(pillar));
        }
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 30.0,
            ranges_m: ranges,
        }
    }

    fn ray_to_pillar(x: f64, y: f64, angle: f64) -> f64 {
        // Three pillars at distinct positions break the room's symmetry.
        let pillars = [
            (2.0_f64, 1.2_f64, 0.25_f64),
            (-1.0, -1.6, 0.3),
            (3.5, -1.0, 0.2),
        ];
        let (dx, dy) = (angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for (px, py, radius) in pillars {
            let fx = px - x;
            let fy = py - y;
            let projection = fx * dx + fy * dy;
            if projection <= 0.0 {
                continue;
            }
            let closest = fx.hypot(fy);
            let perpendicular = (closest * closest - projection * projection)
                .max(0.0)
                .sqrt();
            if perpendicular <= radius {
                let entry = projection
                    - (radius * radius - perpendicular * perpendicular)
                        .max(0.0)
                        .sqrt();
                if entry > 0.0 {
                    best = best.min(entry);
                }
            }
        }
        best
    }

    #[test]
    fn slam_tracks_through_odometry_drift() {
        let mut slam = Slam2d::new(empty_room_grid(), SlamConfig::default());
        let mut true_pose = Pose2d::new(-3.0, 0.0, 0.0);
        let mut odom_pose = Pose2d::new(-3.0, 0.0, 0.0);
        let mut last_update = None;

        for step in 0..12 {
            let scan = room_scan(true_pose.x_m, true_pose.y_m);
            let update = slam
                .process(&scan, odom_pose, Pose2d::IDENTITY)
                .expect("slam update");
            last_update = Some(update);
            true_pose.x_m += 0.5;
            // Odometry drifts in y and yaw; the scan match must correct it.
            odom_pose.x_m += 0.5;
            odom_pose.y_m += 0.02 * (step + 1) as f64;
            odom_pose.yaw_rad += 0.01 * (step + 1) as f64;
        }

        let update = last_update.unwrap();
        assert!(update.matched, "expected at least one scan match");
        let true_final_x = -3.0 + 11.0 * 0.5;
        assert!(
            (update.pose.x_m - true_final_x).abs() < 0.2,
            "pose={:?}",
            update.pose
        );
        assert!(
            (update.pose.y_m - 0.0).abs() < 0.15,
            "drift not corrected: {:?}",
            update.pose
        );

        let occupied = (0..slam.grid().height())
            .flat_map(|y| (0..slam.grid().width()).map(move |x| (x, y)))
            .filter(|(x, y)| {
                slam.grid().is_occupied(GridCoord {
                    x: *x as isize,
                    y: *y as isize,
                })
            })
            .count();
        assert!(
            occupied > 50,
            "map should contain walls, occupied={occupied}"
        );
    }

    #[test]
    fn slam_adds_odometry_edges_and_keeps_scan_matched_pose() {
        let mut slam = Slam2d::new(empty_room_grid(), SlamConfig::default());
        let mut true_pose = Pose2d::new(-3.0, 0.0, 0.0);
        let mut odom_pose = true_pose;
        for _ in 0..12 {
            let scan = landmark_scan(true_pose.x_m, true_pose.y_m);
            slam.process(&scan, odom_pose, Pose2d::IDENTITY)
                .expect("slam update");
            // Advance after the scan so the final estimate corresponds to the
            // pose at which the last scan was taken.
            let last_x = true_pose.x_m;
            true_pose.x_m += 0.5;
            odom_pose.x_m += 0.5;
            odom_pose.yaw_rad += 0.01;
            let _ = last_x;
        }
        // One node per processed scan; one odometry edge per transition.
        assert_eq!(slam.graph().node_count(), 12);
        assert_eq!(slam.graph().edge_count(), 11);
        assert!(slam.loop_edge_indices().is_empty());
        // The final pose stays near the last scan position despite yaw drift.
        let last_scan_x = -3.0 + 11.0 * 0.5;
        assert!(
            (slam.pose().x_m - last_scan_x).abs() < 0.2,
            "pose={:?}",
            slam.pose()
        );
        assert!(slam.pose().y_m.abs() < 0.15, "pose={:?}", slam.pose());
    }

    #[test]
    fn slam_closes_a_loop_on_an_exact_revisit() {
        let mut slam = Slam2d::new(empty_room_grid(), SlamConfig::default());
        // Outbound along x with exact odometry, then return to the start.
        let mut x = -3.0_f64;
        let mut odom = Pose2d::new(-3.0, 0.0, 0.0);
        for _ in 0..16 {
            let scan = landmark_scan(x, 0.0);
            slam.process(&scan, odom, Pose2d::IDENTITY).expect("update");
            x += 0.25;
            odom.x_m += 0.25;
        }
        // Reverse back to the start.
        let mut closing = 0;
        for _ in 0..16 {
            x -= 0.25;
            odom.x_m -= 0.25;
            let scan = landmark_scan(x, 0.0);
            let update = slam.process(&scan, odom, Pose2d::IDENTITY).expect("update");
            closing += update.loop_closures;
        }
        assert!(closing > 0, "expected a loop closure on the revisit");
        assert_eq!(slam.loop_edge_indices().len(), closing);
        // The estimate is pulled back toward the true start.
        assert!(
            (slam.pose().x_m - x).abs() < 0.3,
            "pose={:?} truth_x={x}",
            slam.pose()
        );
    }

    #[test]
    fn closure_gate_rejects_inconsistent_matches() {
        let predicted = Pose2d::new(1.0, 0.5, 0.2);
        assert!(closure_consistent(
            predicted,
            Pose2d::new(1.1, 0.55, 0.25),
            0.75,
            0.5
        ));
        assert!(!closure_consistent(
            predicted,
            Pose2d::new(3.0, 0.0, 0.2),
            0.75,
            0.5
        ));
        assert!(!closure_consistent(
            predicted,
            Pose2d::new(1.0, 0.5, 1.2),
            0.75,
            0.5
        ));
    }
}
