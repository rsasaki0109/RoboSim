//! Multi-session ("lifelong") pose-graph mapping.
//!
//! A robot that works in one building maps it many times. Each visit produces
//! its own trajectory and its own map, and the useful question is not "what did
//! this run see" but "what does the robot know about this place after every run
//! so far".
//!
//! [`combine_graphs`](crate::combine_graphs) concatenates two graphs: the nodes
//! and edges of both end up in one container, but nothing connects them, so the
//! result is two disconnected trajectories that happen to share a struct. That
//! is two maps, not a lifelong map.
//!
//! This module adds the missing piece. A later session is expressed in its own
//! frame, so merging it means
//!
//! 1. finding the rigid transform that puts it in the prior session's frame,
//! 2. adding **inter-session constraints** between nodes that observed the same
//!    place in different sessions, and
//! 3. re-optimizing so both trajectories share one consistent estimate.
//!
//! Step 2 is what makes the result a single map: without at least one such
//! constraint the merged graph stays disconnected and the optimizer cannot
//! relate the sessions at all.
//!
//! Every node keeps the session it came from, because later lifelong work —
//! pruning old nodes, decaying stale structure, reporting per-session drift —
//! needs to know which visit contributed what.

use crate::likelihood::LikelihoodConfig;
use crate::pose_graph::{PoseGraph, PoseGraphEdge, PoseGraphError};
use crate::relocalize::{GlobalRelocalizer, RelocalizationConfig, RelocalizationError};
use rne_math::Vec3;
use rne_nav::{OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};

/// Identifier of one mapping session.
///
/// Sessions are numbered in the order they are merged, starting at zero for the
/// graph a [`LifelongPoseGraph`] is seeded with.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SessionId(pub usize);

/// A correspondence between a node in the prior map and a node in a new session.
///
/// This is the lifelong equivalent of a loop closure: instead of relating two
/// poses within one run, it relates a pose from an earlier visit to a pose from
/// the current one. `measurement` is the observed `prior → session` relative
/// pose, normally produced by matching the new session's scan against the prior
/// map.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SessionConstraint {
    /// Node index in the existing lifelong graph.
    pub prior_node: usize,
    /// Node index within the session being merged, before re-indexing.
    pub session_node: usize,
    /// Measured relative pose from the prior node to the session node.
    pub measurement: Pose2d,
    /// Diagonal information (inverse covariance) for `(x, y, yaw)`.
    pub information: (f64, f64, f64),
}

impl SessionConstraint {
    /// Creates a constraint with the given information weights.
    pub fn new(
        prior_node: usize,
        session_node: usize,
        measurement: Pose2d,
        information: (f64, f64, f64),
    ) -> Self {
        Self {
            prior_node,
            session_node,
            measurement,
            information,
        }
    }

    /// Returns whether the measurement and information are usable.
    pub fn is_valid(&self) -> bool {
        self.measurement.is_finite()
            && [self.information.0, self.information.1, self.information.2]
                .into_iter()
                .all(|weight| weight.is_finite() && weight > 0.0)
    }
}

/// Error returned when merging a session into a lifelong map.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SessionError {
    /// The session being merged has no nodes.
    #[error("session graph has no nodes")]
    EmptySession,
    /// No inter-session constraint was supplied, so the merge cannot connect.
    #[error("merging a session requires at least one inter-session constraint")]
    NoConstraints,
    /// A constraint referenced a node outside the prior graph.
    #[error("constraint references prior node {0} which does not exist")]
    UnknownPriorNode(usize),
    /// A constraint referenced a node outside the session graph.
    #[error("constraint references session node {0} which does not exist")]
    UnknownSessionNode(usize),
    /// A constraint measurement or information weight was not usable.
    #[error("inter-session constraint must be finite with positive information")]
    InvalidConstraint,
    /// Optimization of the merged graph failed.
    #[error(transparent)]
    Optimization(#[from] PoseGraphError),
}

/// A pose graph accumulated across mapping sessions.
///
/// The graph itself is an ordinary [`PoseGraph`]; this type adds the session
/// each node came from and the merge operation that keeps the graph connected.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LifelongPoseGraph {
    graph: PoseGraph,
    node_sessions: Vec<SessionId>,
    session_count: usize,
}

impl LifelongPoseGraph {
    /// Seeds a lifelong map from the first session's graph.
    ///
    /// Every node belongs to session zero.
    pub fn from_first_session(graph: PoseGraph) -> Self {
        let node_sessions = vec![SessionId(0); graph.node_count()];
        Self {
            graph,
            node_sessions,
            session_count: 1,
        }
    }

    /// Returns the accumulated pose graph.
    pub fn graph(&self) -> &PoseGraph {
        &self.graph
    }

    /// Consumes the map and returns the accumulated pose graph.
    pub fn into_graph(self) -> PoseGraph {
        self.graph
    }

    /// Returns how many sessions have been merged, including the first.
    pub fn session_count(&self) -> usize {
        self.session_count
    }

    /// Returns the session each node came from, indexed by node.
    pub fn node_sessions(&self) -> &[SessionId] {
        &self.node_sessions
    }

    /// Returns the session one node came from, or `None` when out of range.
    pub fn node_session(&self, node: usize) -> Option<SessionId> {
        self.node_sessions.get(node).copied()
    }

    /// Returns the node indices contributed by one session, ascending.
    pub fn session_nodes(&self, session: SessionId) -> Vec<usize> {
        self.node_sessions
            .iter()
            .enumerate()
            .filter_map(|(node, owner)| (*owner == session).then_some(node))
            .collect()
    }

    /// Returns whether every node is reachable from node zero.
    ///
    /// A lifelong map that is not connected is a set of independent maps: the
    /// optimizer cannot express one session's correction in another's frame.
    pub fn is_connected(&self) -> bool {
        self.graph.node_count() > 0 && self.graph.component(0).len() == self.graph.node_count()
    }

    /// Merges a session into the map and re-optimizes.
    ///
    /// The session's poses are first rigidly moved into the map frame using the
    /// first supplied constraint, so the optimizer starts from a sensible
    /// linearization point rather than from two overlapping trajectories. Every
    /// constraint then becomes an inter-session edge, and the merged graph is
    /// optimized with a Huber kernel so one bad correspondence cannot drag both
    /// trajectories.
    ///
    /// Returns the final mean squared error reported by the optimizer.
    pub fn merge_session(
        &mut self,
        session: &PoseGraph,
        constraints: &[SessionConstraint],
        options: MergeOptions,
    ) -> Result<f64, SessionError> {
        if session.node_count() == 0 {
            return Err(SessionError::EmptySession);
        }
        if constraints.is_empty() {
            return Err(SessionError::NoConstraints);
        }
        for constraint in constraints {
            if constraint.prior_node >= self.graph.node_count() {
                return Err(SessionError::UnknownPriorNode(constraint.prior_node));
            }
            if constraint.session_node >= session.node_count() {
                return Err(SessionError::UnknownSessionNode(constraint.session_node));
            }
            if !constraint.is_valid() {
                return Err(SessionError::InvalidConstraint);
            }
        }

        // Rigid alignment from the first correspondence: the session node should
        // land where the prior node plus the measured offset says it is.
        let anchor = &constraints[0];
        let prior_pose = self
            .graph
            .node(anchor.prior_node)
            .ok_or(SessionError::UnknownPriorNode(anchor.prior_node))?;
        let session_pose = session
            .node(anchor.session_node)
            .ok_or(SessionError::UnknownSessionNode(anchor.session_node))?;
        let expected = prior_pose.compose(anchor.measurement);
        let map_from_session = expected.compose(session_pose.inverse());

        let offset = self.graph.node_count();
        let session_id = SessionId(self.session_count);
        for pose in session.nodes() {
            self.graph.add_node(map_from_session.compose(*pose));
            self.node_sessions.push(session_id);
        }
        // Intra-session edges are relative measurements, so they are unchanged
        // by the rigid alignment and only need re-indexing.
        for edge in session.edges() {
            self.graph.add_edge(PoseGraphEdge {
                from: edge.from + offset,
                to: edge.to + offset,
                ..*edge
            });
        }
        for constraint in constraints {
            self.graph.add_edge(PoseGraphEdge::loop_closure(
                constraint.prior_node,
                constraint.session_node + offset,
                constraint.measurement,
                constraint.information,
            ));
        }
        self.session_count += 1;

        let error = self.graph.optimize_robust(
            options.iterations,
            options.damping,
            options.anchor,
            options.huber_delta,
        )?;
        Ok(error)
    }
}

/// Optimization settings applied after a session merge.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MergeOptions {
    /// Gauss-Newton iterations.
    pub iterations: usize,
    /// Levenberg damping added to the normal equations.
    pub damping: f64,
    /// Node held fixed during optimization.
    pub anchor: usize,
    /// Huber threshold that down-weights grossly inconsistent correspondences.
    pub huber_delta: f64,
}

impl Default for MergeOptions {
    fn default() -> Self {
        Self {
            iterations: 20,
            damping: 1.0e-6,
            anchor: 0,
            huber_delta: 0.3,
        }
    }
}

/// One keyframe scan from the session being merged.
///
/// The points are in the sensor frame, exactly as the scan matcher and
/// relocalizer consume them, and `sensor_from_base` places the sensor on the
/// robot. `node` identifies which node of the session graph recorded the scan.
#[derive(Clone, Debug, PartialEq)]
pub struct SessionScan {
    /// Node index within the session graph.
    pub node: usize,
    /// Scan endpoints in the sensor frame, in meters.
    pub points_sensor_m: Vec<Vec3>,
    /// Sensor pose relative to the robot base.
    pub sensor_from_base: Pose2d,
}

/// Settings for discovering inter-session correspondences.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiscoveryConfig {
    /// Likelihood field built from the prior map.
    pub likelihood: LikelihoodConfig,
    /// Search settings for the global relocalizer.
    pub relocalization: RelocalizationConfig,
    /// Mean likelihood a recognition must reach to become a constraint.
    ///
    /// This is deliberately separate from the relocalizer's own `min_score`: a
    /// pose good enough to seed localization is not necessarily good enough to
    /// weld two sessions together permanently.
    pub min_score: f64,
    /// Maximum distance in meters from the recognized pose to the prior node it
    /// is attached to.
    ///
    /// A recognition far from every prior pose means the new session saw a part
    /// of the building the prior map only covers thinly; tying it to a distant
    /// node would fabricate a measurement.
    pub max_prior_distance_m: f64,
    /// Information weight applied at a score of 1.0, scaled linearly by score.
    pub information_scale: (f64, f64, f64),
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            likelihood: LikelihoodConfig::default(),
            relocalization: RelocalizationConfig::default(),
            min_score: 0.6,
            max_prior_distance_m: 2.0,
            information_scale: (100.0, 100.0, 100.0),
        }
    }
}

impl DiscoveryConfig {
    /// Returns whether every threshold is finite and positive.
    pub fn is_valid(&self) -> bool {
        self.min_score.is_finite()
            && self.min_score > 0.0
            && self.max_prior_distance_m.is_finite()
            && self.max_prior_distance_m > 0.0
            && [
                self.information_scale.0,
                self.information_scale.1,
                self.information_scale.2,
            ]
            .into_iter()
            .all(|weight| weight.is_finite() && weight > 0.0)
    }
}

/// One accepted recognition of a previously mapped place.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SessionRecognition {
    /// The constraint this recognition produced.
    pub constraint: SessionConstraint,
    /// Pose recognized in the prior map frame.
    pub recognized_pose: Pose2d,
    /// Mean likelihood of the scan at that pose.
    pub score: f64,
    /// Distance in meters from the recognized pose to the attached prior node.
    pub prior_distance_m: f64,
}

/// Error raised while discovering inter-session correspondences.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DiscoveryError {
    /// A threshold was non-finite or non-positive.
    #[error("discovery configuration must be finite and positive")]
    InvalidConfig,
    /// A scan referenced a node outside the session graph.
    #[error("scan references session node {0} which does not exist")]
    UnknownSessionNode(usize),
    /// Relocalization against the prior map failed.
    #[error(transparent)]
    Relocalization(#[from] RelocalizationError),
}

/// Discovers inter-session constraints by recognizing places in the prior map.
///
/// This is the step that turns "merge these sessions, here are the
/// correspondences" into "the robot came back and realized where it was". Each
/// supplied keyframe scan is relocalized against the prior map without using
/// the new session's own frame at all; a recognition is accepted only when it
/// scores above [`DiscoveryConfig::min_score`] and lands within
/// [`DiscoveryConfig::max_prior_distance_m`] of an existing node.
///
/// The measurement stored on the constraint is the prior node's pose composed
/// backwards with the recognized pose, so it is a relative measurement in
/// exactly the form [`LifelongPoseGraph::merge_session`] expects. Information is
/// scaled by the score, so a marginal recognition pulls less than a confident
/// one.
///
/// Scans are processed in the supplied order and ties between equidistant prior
/// nodes are broken by the lower node index, so the same inputs always produce
/// the same constraints. Returns an empty vector when nothing was recognized;
/// that is a normal outcome for a session in an unmapped part of the building,
/// and the caller should not merge on it.
pub fn discover_session_constraints(
    prior_map: &OccupancyGrid,
    lifelong: &LifelongPoseGraph,
    session: &PoseGraph,
    scans: &[SessionScan],
    config: &DiscoveryConfig,
) -> Result<Vec<SessionRecognition>, DiscoveryError> {
    if !config.is_valid() {
        return Err(DiscoveryError::InvalidConfig);
    }
    for scan in scans {
        if scan.node >= session.node_count() {
            return Err(DiscoveryError::UnknownSessionNode(scan.node));
        }
    }
    let relocalizer = GlobalRelocalizer::new(prior_map, config.likelihood, config.relocalization)?;

    let mut recognitions = Vec::new();
    for scan in scans {
        if scan.points_sensor_m.is_empty() {
            continue;
        }
        let Some(result) = relocalizer.relocalize(&scan.points_sensor_m, scan.sensor_from_base)?
        else {
            continue;
        };
        if result.score < config.min_score {
            continue;
        }

        let Some((prior_node, prior_pose, prior_distance_m)) =
            nearest_prior_node(lifelong, result.pose)
        else {
            continue;
        };
        if prior_distance_m > config.max_prior_distance_m {
            continue;
        }

        // Relative measurement from the prior pose to the recognized pose.
        let measurement = prior_pose.inverse().compose(result.pose);
        let weight = result.score.clamp(0.0, 1.0);
        let information = (
            config.information_scale.0 * weight,
            config.information_scale.1 * weight,
            config.information_scale.2 * weight,
        );
        let constraint = SessionConstraint::new(prior_node, scan.node, measurement, information);
        if !constraint.is_valid() {
            continue;
        }
        recognitions.push(SessionRecognition {
            constraint,
            recognized_pose: result.pose,
            score: result.score,
            prior_distance_m,
        });
    }
    Ok(recognitions)
}

/// Returns the existing node closest to a recognized pose, ties by lower index.
fn nearest_prior_node(
    lifelong: &LifelongPoseGraph,
    recognized: Pose2d,
) -> Option<(usize, Pose2d, f64)> {
    let mut best: Option<(usize, Pose2d, f64)> = None;
    for node in 0..lifelong.graph().node_count() {
        let pose = lifelong.graph().node(node)?;
        let distance_m =
            ((pose.x_m - recognized.x_m).powi(2) + (pose.y_m - recognized.y_m).powi(2)).sqrt();
        if best.is_none_or(|(_, _, current)| distance_m < current) {
            best = Some((node, pose, distance_m));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph_io::combine_graphs;
    use rne_nav::OccupancyGrid;

    /// Ground-truth poses of a straight corridor run, one node per meter.
    fn truth(node_count: usize) -> Vec<Pose2d> {
        (0..node_count)
            .map(|index| Pose2d::new(index as f64, 0.0, 0.0))
            .collect()
    }

    /// Builds a session whose odometry accumulates `drift_m` of error per step.
    ///
    /// Edges carry the *measured* (drifting) relative pose, which is what a real
    /// odometry chain records, so the optimizer sees a consistent problem.
    fn drifting_session(node_count: usize, drift_m: f64) -> PoseGraph {
        let mut graph = PoseGraph::new();
        let mut pose = Pose2d::new(0.0, 0.0, 0.0);
        graph.add_node(pose);
        for _ in 1..node_count {
            let step = Pose2d::new(1.0 + drift_m, 0.0, 0.0);
            let next = pose.compose(step);
            graph.add_node(next);
            graph.add_edge(PoseGraphEdge::odometry(
                graph.node_count() - 2,
                graph.node_count() - 1,
                step,
            ));
            pose = next;
        }
        graph
    }

    /// Mean distance from each node to its ground-truth pose, in meters.
    fn mean_error_m(poses: &[Pose2d], truth: &[Pose2d]) -> f64 {
        let total: f64 = poses
            .iter()
            .zip(truth)
            .map(|(estimate, truth)| {
                ((estimate.x_m - truth.x_m).powi(2) + (estimate.y_m - truth.y_m).powi(2)).sqrt()
            })
            .sum();
        total / poses.len() as f64
    }

    /// Wall points of a deliberately asymmetric room, in meters.
    ///
    /// The notch on the lower wall breaks the symmetry a plain rectangle would
    /// have, so a scan can identify *where* in the room it was taken.
    fn room_walls_m() -> Vec<Vec3> {
        let mut points = Vec::new();
        let mut push = |x: f64, y: f64| points.push(Vec3::new(x, y, 0.0));
        let step = 0.1;
        let mut x = 0.0;
        while x <= 6.0 + 1e-9 {
            push(x, 0.0);
            push(x, 3.0);
            x += step;
        }
        let mut y = 0.0;
        while y <= 3.0 + 1e-9 {
            push(0.0, y);
            push(6.0, y);
            y += step;
        }
        // Asymmetric notch: a stub wall partway along the room.
        let mut y = 0.0;
        while y <= 1.2 + 1e-9 {
            push(4.0, y);
            y += step;
        }
        points
    }

    fn room_map() -> OccupancyGrid {
        let mut grid =
            OccupancyGrid::new(70, 40, 0.1, Pose2d::new(-0.5, -0.5, 0.0)).expect("room grid");
        for point in room_walls_m() {
            if let Some(coord) = grid.world_to_grid(point) {
                // Several hits so the cell clears the occupied threshold.
                for _ in 0..8 {
                    grid.mark_occupied(coord);
                }
            }
        }
        grid
    }

    /// Simulates a scan of the room walls from one true robot pose.
    ///
    /// Points are expressed in the sensor frame. This keeps every wall point in
    /// range rather than modelling occlusion, which makes recognition easier
    /// than reality; the test therefore proves the wiring and the frame
    /// conventions, not field robustness.
    fn simulated_scan(true_pose: Pose2d, sensor_from_base: Pose2d, max_range_m: f64) -> Vec<Vec3> {
        let sensor_in_map = true_pose.compose(sensor_from_base);
        room_walls_m()
            .into_iter()
            .filter(|point| {
                let dx = point.x - true_pose.x_m;
                let dy = point.y - true_pose.y_m;
                (dx * dx + dy * dy).sqrt() <= max_range_m
            })
            .map(|point| sensor_in_map.inverse_transform_point(point))
            .collect()
    }

    #[test]
    fn a_revisit_is_recognized_in_the_prior_map_and_becomes_a_constraint() {
        let prior_map = room_map();
        let sensor_from_base = Pose2d::new(0.0, 0.0, 0.0);

        // First visit: three keyframes along the room, mapped without drift.
        let truth = [
            Pose2d::new(1.0, 1.5, 0.0),
            Pose2d::new(3.0, 1.5, 0.0),
            Pose2d::new(5.0, 1.5, 0.0),
        ];
        let mut first = PoseGraph::new();
        for pose in truth {
            first.add_node(pose);
        }
        for index in 1..truth.len() {
            let measurement = truth[index - 1].inverse().compose(truth[index]);
            first.add_edge(PoseGraphEdge::odometry(index - 1, index, measurement));
        }
        let lifelong = LifelongPoseGraph::from_first_session(first);

        // Second visit over the same route, recorded in its own frame: the
        // robot booted somewhere else and has no idea where it is.
        let session_frame = Pose2d::new(-20.0, 7.0, std::f64::consts::FRAC_PI_2);
        let mut second = PoseGraph::new();
        for pose in truth {
            second.add_node(session_frame.compose(pose));
        }
        for index in 1..truth.len() {
            let measurement = truth[index - 1].inverse().compose(truth[index]);
            second.add_edge(PoseGraphEdge::odometry(index - 1, index, measurement));
        }

        let scans: Vec<SessionScan> = truth
            .iter()
            .enumerate()
            .map(|(node, pose)| SessionScan {
                node,
                points_sensor_m: simulated_scan(*pose, sensor_from_base, 8.0),
                sensor_from_base,
            })
            .collect();

        let recognitions = discover_session_constraints(
            &prior_map,
            &lifelong,
            &second,
            &scans,
            &DiscoveryConfig::default(),
        )
        .expect("discovery runs");

        assert!(
            !recognitions.is_empty(),
            "the second visit must recognize the room it already mapped"
        );
        for recognition in &recognitions {
            let expected = truth[recognition.constraint.session_node];
            println!(
                "recognized node {} at ({:.3}, {:.3}) vs truth ({:.3}, {:.3}), score {:.3}, prior node {} at {:.3} m",
                recognition.constraint.session_node,
                recognition.recognized_pose.x_m,
                recognition.recognized_pose.y_m,
                expected.x_m,
                expected.y_m,
                recognition.score,
                recognition.constraint.prior_node,
                recognition.prior_distance_m
            );
        }
        for recognition in &recognitions {
            let expected = truth[recognition.constraint.session_node];
            assert!(
                (recognition.recognized_pose.x_m - expected.x_m).abs() < 0.3
                    && (recognition.recognized_pose.y_m - expected.y_m).abs() < 0.3,
                "recognized {:?} for a keyframe truly at {expected:?}",
                recognition.recognized_pose
            );
            assert!(recognition.score >= DiscoveryConfig::default().min_score);
            assert!(
                recognition.prior_distance_m <= DiscoveryConfig::default().max_prior_distance_m
            );
        }

        // Discovery is deterministic for the same inputs.
        let repeat = discover_session_constraints(
            &prior_map,
            &lifelong,
            &second,
            &scans,
            &DiscoveryConfig::default(),
        )
        .expect("discovery repeats");
        assert_eq!(repeat, recognitions);

        // The discovered constraints are usable directly by the merge, and the
        // session lands back on the route it actually drove.
        let constraints: Vec<SessionConstraint> = recognitions
            .iter()
            .map(|recognition| recognition.constraint)
            .collect();
        let mut merged = lifelong.clone();
        merged
            .merge_session(&second, &constraints, MergeOptions::default())
            .expect("merge the recognized session");
        assert!(merged.is_connected());

        for (index, node) in merged.session_nodes(SessionId(1)).into_iter().enumerate() {
            let pose = merged.graph().node(node).expect("merged node");
            let expected = truth[index];
            assert!(
                (pose.x_m - expected.x_m).abs() < 0.3 && (pose.y_m - expected.y_m).abs() < 0.3,
                "merged node {index} at {pose:?} should sit near {expected:?}"
            );
        }
    }

    #[test]
    fn discovery_rejects_bad_configuration_unknown_nodes_and_unrecognized_places() {
        let prior_map = room_map();
        let lifelong =
            LifelongPoseGraph::from_first_session(single_node_graph(Pose2d::new(1.0, 1.5, 0.0)));
        let mut session = PoseGraph::new();
        session.add_node(Pose2d::new(0.0, 0.0, 0.0));
        let sensor_from_base = Pose2d::new(0.0, 0.0, 0.0);
        let scan = SessionScan {
            node: 0,
            points_sensor_m: simulated_scan(Pose2d::new(1.0, 1.5, 0.0), sensor_from_base, 8.0),
            sensor_from_base,
        };

        let invalid = DiscoveryConfig {
            min_score: 0.0,
            ..DiscoveryConfig::default()
        };
        assert_eq!(
            discover_session_constraints(
                &prior_map,
                &lifelong,
                &session,
                std::slice::from_ref(&scan),
                &invalid
            ),
            Err(DiscoveryError::InvalidConfig)
        );

        let out_of_range = SessionScan {
            node: 9,
            ..scan.clone()
        };
        assert_eq!(
            discover_session_constraints(
                &prior_map,
                &lifelong,
                &session,
                &[out_of_range],
                &DiscoveryConfig::default()
            ),
            Err(DiscoveryError::UnknownSessionNode(9))
        );

        // An empty scan contributes nothing rather than failing the batch.
        let empty = SessionScan {
            node: 0,
            points_sensor_m: Vec::new(),
            sensor_from_base,
        };
        assert!(discover_session_constraints(
            &prior_map,
            &lifelong,
            &session,
            &[empty],
            &DiscoveryConfig::default()
        )
        .expect("empty scan is skipped")
        .is_empty());

        // A prior map whose only node is far from anything the scan matches
        // yields no constraint rather than a fabricated one.
        let distant =
            LifelongPoseGraph::from_first_session(single_node_graph(Pose2d::new(40.0, 40.0, 0.0)));
        assert!(discover_session_constraints(
            &prior_map,
            &distant,
            &session,
            &[scan],
            &DiscoveryConfig::default()
        )
        .expect("discovery runs")
        .is_empty());
    }

    fn single_node_graph(pose: Pose2d) -> PoseGraph {
        let mut graph = PoseGraph::new();
        graph.add_node(pose);
        graph
    }

    #[test]
    fn concatenating_sessions_leaves_a_silently_unrelated_second_map() {
        // This is the behaviour `merge_session` exists to replace. The combined
        // container holds both trajectories but relates them by nothing, and —
        // worse — optimizing it reports success, so a caller can believe two
        // sessions were merged when the second was never constrained at all.
        let node_count = 6;
        let first = drifting_session(node_count, 0.0);
        let second = drifting_session(node_count, 0.05);
        let mut combined = combine_graphs(&first, &second);
        assert_eq!(combined.node_count(), 2 * node_count);
        assert_eq!(
            combined.component(0).len(),
            node_count,
            "concatenation must leave the second session unreachable"
        );

        // `PoseGraphError::Disconnected` exists but is never raised: the
        // optimizer accepts the disconnected graph.
        let before: Vec<Pose2d> = (node_count..2 * node_count)
            .map(|node| combined.node(node).expect("second-session node"))
            .collect();
        combined
            .optimize(10, 1.0e-6, 0)
            .expect("the optimizer accepts a disconnected graph");
        let after: Vec<Pose2d> = (node_count..2 * node_count)
            .map(|node| combined.node(node).expect("second-session node"))
            .collect();
        assert_eq!(
            before, after,
            "optimizing the concatenation cannot correct the second session, \
             because nothing relates it to the first"
        );
        assert!(mean_error_m(&after, &truth(node_count)) > 0.1);
    }

    #[test]
    fn merging_a_session_connects_it_and_reduces_drift_against_truth() {
        let node_count = 6;
        let truth = truth(node_count);

        // A clean first visit, then a second visit whose odometry over-reports
        // each step by 5 cm and therefore ends 25 cm long.
        let first = drifting_session(node_count, 0.0);
        let second = drifting_session(node_count, 0.05);
        let uncorrected = second.nodes().to_vec();
        assert!(mean_error_m(&uncorrected, &truth) > 0.1);

        // The robot recognizes the corridor's start and end from the prior map.
        let constraints = [
            SessionConstraint::new(0, 0, Pose2d::new(0.0, 0.0, 0.0), (100.0, 100.0, 100.0)),
            SessionConstraint::new(
                node_count - 1,
                node_count - 1,
                Pose2d::new(0.0, 0.0, 0.0),
                (100.0, 100.0, 100.0),
            ),
        ];

        let mut lifelong = LifelongPoseGraph::from_first_session(first);
        assert!(lifelong.is_connected());
        assert_eq!(lifelong.session_count(), 1);

        lifelong
            .merge_session(&second, &constraints, MergeOptions::default())
            .expect("merge second session");

        assert_eq!(lifelong.session_count(), 2);
        assert_eq!(lifelong.graph().node_count(), 2 * node_count);
        assert!(
            lifelong.is_connected(),
            "a merged session must share one connected map"
        );

        // Provenance survives the merge.
        assert_eq!(lifelong.node_session(0), Some(SessionId(0)));
        assert_eq!(lifelong.node_session(node_count), Some(SessionId(1)));
        assert_eq!(lifelong.session_nodes(SessionId(1)).len(), node_count);

        let merged: Vec<Pose2d> = lifelong
            .session_nodes(SessionId(1))
            .into_iter()
            .map(|node| lifelong.graph().node(node).expect("merged node"))
            .collect();
        let before = mean_error_m(&uncorrected, &truth);
        let after = mean_error_m(&merged, &truth);
        println!("mean node error: {before:.4} m before merge, {after:.4} m after");
        assert!(
            after < before,
            "merging must pull the drifting session toward the prior map: \
             {before:.4} m before, {after:.4} m after"
        );
    }

    #[test]
    fn merge_rejects_empty_sessions_missing_constraints_and_bad_references() {
        let first = drifting_session(4, 0.0);
        let second = drifting_session(4, 0.0);
        let valid = SessionConstraint::new(0, 0, Pose2d::new(0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        let options = MergeOptions::default();

        let mut lifelong = LifelongPoseGraph::from_first_session(first.clone());
        assert_eq!(
            lifelong.merge_session(&PoseGraph::new(), &[valid], options),
            Err(SessionError::EmptySession)
        );
        assert_eq!(
            lifelong.merge_session(&second, &[], options),
            Err(SessionError::NoConstraints)
        );
        assert_eq!(
            lifelong.merge_session(
                &second,
                &[SessionConstraint::new(
                    99,
                    0,
                    Pose2d::new(0.0, 0.0, 0.0),
                    (1.0, 1.0, 1.0)
                )],
                options
            ),
            Err(SessionError::UnknownPriorNode(99))
        );
        assert_eq!(
            lifelong.merge_session(
                &second,
                &[SessionConstraint::new(
                    0,
                    99,
                    Pose2d::new(0.0, 0.0, 0.0),
                    (1.0, 1.0, 1.0)
                )],
                options
            ),
            Err(SessionError::UnknownSessionNode(99))
        );
        assert_eq!(
            lifelong.merge_session(
                &second,
                &[SessionConstraint::new(
                    0,
                    0,
                    Pose2d::new(f64::NAN, 0.0, 0.0),
                    (1.0, 1.0, 1.0)
                )],
                options
            ),
            Err(SessionError::InvalidConstraint)
        );
        assert_eq!(
            lifelong.merge_session(
                &second,
                &[SessionConstraint::new(
                    0,
                    0,
                    Pose2d::new(0.0, 0.0, 0.0),
                    (0.0, 1.0, 1.0)
                )],
                options
            ),
            Err(SessionError::InvalidConstraint)
        );

        // Every rejection left the map untouched.
        assert_eq!(lifelong.session_count(), 1);
        assert_eq!(lifelong.graph().node_count(), first.node_count());
    }

    #[test]
    fn a_session_recorded_in_its_own_frame_is_aligned_into_the_map_frame() {
        // The second visit starts in a different corner of the building and
        // therefore records its trajectory in a rotated, translated frame.
        let first = drifting_session(5, 0.0);
        let offset = Pose2d::new(10.0, -4.0, std::f64::consts::FRAC_PI_2);
        let mut second = PoseGraph::new();
        for pose in first.nodes() {
            second.add_node(offset.compose(*pose));
        }
        for edge in first.edges() {
            second.add_edge(*edge);
        }

        let mut lifelong = LifelongPoseGraph::from_first_session(first.clone());
        lifelong
            .merge_session(
                &second,
                &[SessionConstraint::new(
                    0,
                    0,
                    Pose2d::new(0.0, 0.0, 0.0),
                    (100.0, 100.0, 100.0),
                )],
                MergeOptions::default(),
            )
            .expect("merge offset session");

        // After alignment the two visits of the same corridor coincide.
        for (index, node) in lifelong.session_nodes(SessionId(1)).into_iter().enumerate() {
            let merged = lifelong.graph().node(node).expect("merged node");
            let original = first.node(index).expect("first-session node");
            assert!(
                (merged.x_m - original.x_m).abs() < 1.0e-6
                    && (merged.y_m - original.y_m).abs() < 1.0e-6,
                "node {index}: {merged:?} should coincide with {original:?}"
            );
        }
    }
}
