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

use crate::likelihood::{LikelihoodConfig, LikelihoodField};
use crate::pose_graph::{PoseGraph, PoseGraphEdge, PoseGraphError};
use crate::relocalize::{GlobalRelocalizer, RelocalizationConfig, RelocalizationError};
use crate::scan_match::{ScanMatchConfig, ScanMatcher};
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
        self.validate_merge(session, constraints)?;

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

impl LifelongPoseGraph {
    /// Merges a session with the existing map held fixed.
    ///
    /// [`Self::merge_session`] re-optimizes the whole graph, so every merge
    /// moves the existing map a little toward the new session: measured on a
    /// four-day warehouse run, 4 to 10 cm and up to 0.02 rad per merge. Over
    /// days the map frame creeps, and anything stored in map coordinates
    /// creeps with it. Here only the new session's nodes are optimized; each
    /// constraint acts as an absolute target for its session node, the prior
    /// node's current pose composed with the measurement. The existing nodes,
    /// and so the map frame, do not move. The session's edges and the
    /// inter-session edges are added to the graph exactly as `merge_session`
    /// adds them, so a later full optimization can still use them.
    ///
    /// Returns the final mean squared error of the session-only optimization.
    pub fn merge_session_onto_map(
        &mut self,
        session: &PoseGraph,
        constraints: &[SessionConstraint],
        options: MergeOptions,
    ) -> Result<f64, SessionError> {
        self.validate_merge(session, constraints)?;
        // Node 0 is the map origin; session node i is node i + 1.
        let mut local = PoseGraph::new();
        local.add_node(Pose2d::IDENTITY);
        let anchor = &constraints[0];
        let target = |constraint: &SessionConstraint| {
            self.graph
                .node(constraint.prior_node)
                .map(|prior| prior.compose(constraint.measurement))
                .ok_or(SessionError::UnknownPriorNode(constraint.prior_node))
        };
        let session_anchor = session
            .node(anchor.session_node)
            .ok_or(SessionError::UnknownSessionNode(anchor.session_node))?;
        let map_from_session = target(anchor)?.compose(session_anchor.inverse());
        for pose in session.nodes() {
            local.add_node(map_from_session.compose(*pose));
        }
        for edge in session.edges() {
            local.add_edge(PoseGraphEdge {
                from: edge.from + 1,
                to: edge.to + 1,
                ..*edge
            });
        }
        for constraint in constraints {
            local.add_edge(PoseGraphEdge::loop_closure(
                0,
                constraint.session_node + 1,
                target(constraint)?,
                constraint.information,
            ));
        }
        let error =
            local.optimize_robust(options.iterations, options.damping, 0, options.huber_delta)?;

        let offset = self.graph.node_count();
        let session_id = SessionId(self.session_count);
        for pose in &local.nodes()[1..] {
            self.graph.add_node(*pose);
            self.node_sessions.push(session_id);
        }
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
        Ok(error)
    }

    fn validate_merge(
        &self,
        session: &PoseGraph,
        constraints: &[SessionConstraint],
    ) -> Result<(), SessionError> {
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
        Ok(())
    }
}

/// Settings for pruning nodes a later session has superseded.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PruneConfig {
    /// A node is superseded when a node from a later session lies within this
    /// distance, in meters, and within [`Self::max_yaw_rad`] of its heading.
    pub radius_m: f64,
    /// Heading difference, in radians, within which a later node supersedes.
    pub max_yaw_rad: f64,
    /// Node that is never pruned: the optimizer's anchor.
    pub anchor: usize,
    /// The first this-many sessions are the map's reference and are never
    /// pruned. Pruning every older session replaces the map a new session is
    /// registered against with the previous session's copy of it, fitting
    /// error and all, so the map frame random-walks from day to day; keeping a
    /// reference holds it in place.
    pub reference_sessions: usize,
}

impl Default for PruneConfig {
    fn default() -> Self {
        Self {
            radius_m: 0.4,
            max_yaw_rad: 0.6,
            anchor: 0,
            reference_sessions: 0,
        }
    }
}

/// What [`LifelongPoseGraph::prune_superseded`] did.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PruneReport {
    /// Number of nodes removed.
    pub removed: usize,
    /// For every node before pruning, its index afterwards, or `None` when it
    /// was removed. Callers holding per-node data (keyframe scans) use this to
    /// drop and re-index theirs.
    pub index_map: Vec<Option<usize>>,
}

impl LifelongPoseGraph {
    /// Removes nodes that a later session has seen again from nearly the same
    /// pose, so the graph stops growing with every visit to the same place.
    ///
    /// A node is superseded when some node from a later session lies within
    /// [`PruneConfig::radius_m`] and [`PruneConfig::max_yaw_rad`] of its current
    /// estimate. Removing a node must not cut the graph, so its constraints are
    /// compounded onto a surviving neighbour first: the neighbour it shares an
    /// odometry edge with, or its lowest-indexed neighbour otherwise. Each
    /// re-attached edge's measurement is the neighbour-to-node measurement
    /// composed with the original one, and its information is the two edges'
    /// information combined in series (covariances add), so a compounded edge
    /// is never more certain than the path it replaces.
    ///
    /// Nodes are considered in index order, and a node counts as superseded
    /// only by nodes that are still present, so the result is deterministic
    /// and the latest session always survives intact. The anchor and the
    /// reference sessions are never removed. Estimates are left as they are; no re-optimization is run.
    pub fn prune_superseded(&mut self, config: PruneConfig) -> PruneReport {
        let count = self.graph.node_count();
        let mut alive = vec![true; count];
        let mut edges: Vec<PoseGraphEdge> = self.graph.edges().to_vec();
        let nodes = self.graph.nodes().to_vec();
        let superseded = |node: usize, alive: &[bool]| {
            let session = self.node_sessions[node];
            let pose = nodes[node];
            (0..count).any(|other| {
                alive[other]
                    && self.node_sessions[other] > session
                    && (nodes[other].x_m - pose.x_m).hypot(nodes[other].y_m - pose.y_m)
                        <= config.radius_m
                    && wrap_angle(nodes[other].yaw_rad - pose.yaw_rad).abs() <= config.max_yaw_rad
            })
        };
        for node in 0..count {
            let reference = self.node_sessions[node].0 < config.reference_sessions;
            if node == config.anchor || reference || !superseded(node, &alive) {
                continue;
            }
            edges = contract_node(&edges, node);
            alive[node] = false;
        }

        let mut index_map = Vec::with_capacity(count);
        let mut graph = PoseGraph::new();
        let mut node_sessions = Vec::new();
        for node in 0..count {
            if alive[node] {
                index_map.push(Some(graph.add_node(nodes[node])));
                node_sessions.push(self.node_sessions[node]);
            } else {
                index_map.push(None);
            }
        }
        for edge in edges {
            if let (Some(from), Some(to)) = (index_map[edge.from], index_map[edge.to]) {
                graph.add_edge(PoseGraphEdge { from, to, ..edge });
            }
        }
        let removed = alive.iter().filter(|kept| !**kept).count();
        self.graph = graph;
        self.node_sessions = node_sessions;
        PruneReport { removed, index_map }
    }
}

/// One edge of a node being contracted, seen from that node.
#[derive(Clone, Copy)]
struct Incident {
    neighbour: usize,
    /// Measurement from the contracted node to the neighbour.
    node_to_neighbour: Pose2d,
    information: (f64, f64, f64),
    loop_closure: bool,
}

/// Re-attaches every edge of `node` to one surviving neighbour and drops the
/// edges between them, leaving `node` isolated.
fn contract_node(edges: &[PoseGraphEdge], node: usize) -> Vec<PoseGraphEdge> {
    let incident: Vec<Incident> = edges
        .iter()
        .filter_map(|edge| {
            if edge.from == node && edge.to != node {
                Some(Incident {
                    neighbour: edge.to,
                    node_to_neighbour: edge.measurement,
                    information: edge.information,
                    loop_closure: edge.loop_closure,
                })
            } else if edge.to == node && edge.from != node {
                Some(Incident {
                    neighbour: edge.from,
                    node_to_neighbour: edge.measurement.inverse(),
                    information: edge.information,
                    loop_closure: edge.loop_closure,
                })
            } else {
                None
            }
        })
        .collect();
    let mut kept: Vec<PoseGraphEdge> = edges
        .iter()
        .filter(|edge| edge.from != node && edge.to != node)
        .copied()
        .collect();
    // Prefer a neighbour along the odometry chain; any neighbour otherwise.
    let survivor = incident
        .iter()
        .filter(|edge| !edge.loop_closure)
        .map(|edge| edge.neighbour)
        .min()
        .or_else(|| incident.iter().map(|edge| edge.neighbour).min());
    let Some(survivor) = survivor else {
        return kept;
    };
    // The link to the survivor with the strongest yaw information.
    let link = incident
        .iter()
        .filter(|edge| edge.neighbour == survivor)
        .copied()
        .reduce(|best, candidate| {
            if best.information.2 >= candidate.information.2 {
                best
            } else {
                candidate
            }
        })
        .expect("survivor is a neighbour");
    let survivor_to_node = link.node_to_neighbour.inverse();
    for edge in incident.iter().filter(|edge| edge.neighbour != survivor) {
        kept.push(PoseGraphEdge {
            from: survivor,
            to: edge.neighbour,
            measurement: survivor_to_node.compose(edge.node_to_neighbour),
            information: in_series(link.information, edge.information),
            loop_closure: edge.loop_closure,
        });
    }
    kept
}

/// Information of two measurements composed in series: covariances add.
fn in_series(a: (f64, f64, f64), b: (f64, f64, f64)) -> (f64, f64, f64) {
    let series = |a: f64, b: f64| 1.0 / (1.0 / a + 1.0 / b);
    (series(a.0, b.0), series(a.1, b.1), series(a.2, b.2))
}

fn wrap_angle(angle: f64) -> f64 {
    let wrapped = (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU);
    wrapped - std::f64::consts::PI
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

/// Settings for [`register_session_densely`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DenseRegistrationConfig {
    /// Likelihood field built from the prior map.
    pub likelihood: LikelihoodConfig,
    /// Local scan matcher; its windows bound how far a keyframe may sit from
    /// where the running correction predicts it.
    pub matcher: ScanMatchConfig,
    /// Mean likelihood a match must reach to become a constraint.
    pub min_score: f64,
    /// Maximum distance from the matched pose to the prior node it is attached
    /// to, in meters.
    pub max_prior_distance_m: f64,
    /// Information weight applied at a score of 1.0, scaled linearly by score.
    pub information_scale: (f64, f64, f64),
}

impl Default for DenseRegistrationConfig {
    fn default() -> Self {
        Self {
            likelihood: LikelihoodConfig::default(),
            matcher: ScanMatchConfig::default(),
            min_score: 0.6,
            max_prior_distance_m: 1.0,
            information_scale: (100.0, 100.0, 100.0),
        }
    }
}

/// Registers every keyframe of a session against the prior map, starting from
/// one global recognition.
///
/// [`discover_session_constraints`] searches the whole map for each scan it is
/// given, which is too expensive to do for every keyframe, and a handful of
/// recognitions cannot reconcile two trajectories that each carry their own
/// few-centimetre distortions: the maps they produce disagree locally, and
/// every disagreement reads as a change. This walks the session outward from
/// `seed` in both directions, predicting each keyframe from the previous
/// match's session-to-map correction and refining it with a local scan match,
/// so slowly varying distortion is tracked rather than accumulated. A match
/// that scores below [`DenseRegistrationConfig::min_score`] contributes no
/// constraint and does not update the correction.
///
/// The result is ordered by session node and has the same form as
/// [`discover_session_constraints`]'s, ready for
/// [`LifelongPoseGraph::merge_session`].
pub fn register_session_densely(
    prior_map: &OccupancyGrid,
    lifelong: &LifelongPoseGraph,
    session: &PoseGraph,
    scans: &[SessionScan],
    seed: &SessionRecognition,
    config: &DenseRegistrationConfig,
) -> Result<Vec<SessionRecognition>, DiscoveryError> {
    let valid_weights = [
        config.information_scale.0,
        config.information_scale.1,
        config.information_scale.2,
    ]
    .into_iter()
    .all(|weight| weight.is_finite() && weight > 0.0);
    if !(config.min_score.is_finite()
        && config.min_score > 0.0
        && config.max_prior_distance_m.is_finite()
        && config.max_prior_distance_m > 0.0
        && valid_weights)
    {
        return Err(DiscoveryError::InvalidConfig);
    }
    let seed_node = seed.constraint.session_node;
    let seed_pose = session
        .node(seed_node)
        .ok_or(DiscoveryError::UnknownSessionNode(seed_node))?;
    for scan in scans {
        if scan.node >= session.node_count() {
            return Err(DiscoveryError::UnknownSessionNode(scan.node));
        }
    }
    let field = LikelihoodField::from_occupancy(prior_map, &config.likelihood)
        .map_err(|_| DiscoveryError::InvalidConfig)?;
    let matcher = ScanMatcher::new(config.matcher);
    let mut ordered: Vec<&SessionScan> = scans.iter().collect();
    ordered.sort_by_key(|scan| scan.node);
    let split = ordered.partition_point(|scan| scan.node < seed_node);
    let seed_correction = seed.recognized_pose.compose(seed_pose.inverse());

    let mut recognitions = Vec::new();
    let forward = ordered[split..].iter();
    let backward = ordered[..split].iter().rev();
    for walk in [forward.collect::<Vec<_>>(), backward.collect::<Vec<_>>()] {
        let mut correction = seed_correction;
        for scan in walk {
            let session_pose = session
                .node(scan.node)
                .ok_or(DiscoveryError::UnknownSessionNode(scan.node))?;
            let Some(result) = matcher.match_scan(
                &field,
                &scan.points_sensor_m,
                scan.sensor_from_base,
                correction.compose(session_pose),
            ) else {
                continue;
            };
            if result.score < config.min_score {
                continue;
            }
            correction = result.pose.compose(session_pose.inverse());
            let Some((prior_node, prior_pose, prior_distance_m)) =
                nearest_prior_node(lifelong, result.pose)
            else {
                continue;
            };
            if prior_distance_m > config.max_prior_distance_m {
                continue;
            }
            let weight = result.score.clamp(0.0, 1.0);
            let constraint = SessionConstraint::new(
                prior_node,
                scan.node,
                prior_pose.inverse().compose(result.pose),
                (
                    config.information_scale.0 * weight,
                    config.information_scale.1 * weight,
                    config.information_scale.2 * weight,
                ),
            );
            if constraint.is_valid() {
                recognitions.push(SessionRecognition {
                    constraint,
                    recognized_pose: result.pose,
                    score: result.score,
                    prior_distance_m,
                });
            }
        }
    }
    recognitions.sort_by_key(|recognition| recognition.constraint.session_node);
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
    /// Two visits along the same corridor, the second merged against the first
    /// at its start and end.
    fn two_visits(node_count: usize) -> LifelongPoseGraph {
        let mut lifelong = LifelongPoseGraph::from_first_session(drifting_session(node_count, 0.0));
        let second = drifting_session(node_count, 0.0);
        let last = node_count - 1;
        lifelong
            .merge_session(
                &second,
                &[
                    SessionConstraint::new(0, 0, Pose2d::IDENTITY, (100.0, 100.0, 100.0)),
                    SessionConstraint::new(last, last, Pose2d::IDENTITY, (100.0, 100.0, 100.0)),
                ],
                MergeOptions::default(),
            )
            .expect("merge");
        lifelong
    }

    #[test]
    fn pruning_drops_the_superseded_visit_and_keeps_the_map_connected() {
        let mut lifelong = two_visits(6);
        assert_eq!(lifelong.graph().node_count(), 12);
        let report = lifelong.prune_superseded(PruneConfig::default());
        // Every first-visit node but the anchor sits under a second-visit node.
        assert_eq!(report.removed, 5);
        assert_eq!(lifelong.graph().node_count(), 7);
        assert!(lifelong.is_connected());
        assert_eq!(lifelong.session_nodes(SessionId(1)).len(), 6);
        assert_eq!(report.index_map[0], Some(0));
        assert!(report.index_map[1..6].iter().all(Option::is_none));
        assert_eq!(report.index_map[6], Some(1));
    }

    #[test]
    fn pruning_preserves_the_solution_it_compounds() {
        let mut lifelong = two_visits(6);
        let before: Vec<Pose2d> = lifelong.graph().nodes()[6..].to_vec();
        lifelong.prune_superseded(PruneConfig::default());
        let mut graph = lifelong.graph().clone();
        graph.optimize_robust(20, 1.0e-6, 0, 0.3).expect("optimize");
        // The surviving second visit does not move: compounded edges encode
        // the same relative geometry as the chains they replaced.
        let after = &graph.nodes()[1..];
        assert!(
            mean_error_m(after, &before) < 1.0e-6,
            "moved {}",
            mean_error_m(after, &before)
        );
    }

    #[test]
    fn pruning_keeps_nodes_no_later_session_revisited() {
        let mut lifelong = LifelongPoseGraph::from_first_session(drifting_session(6, 0.0));
        let mut far = PoseGraph::new();
        far.add_node(Pose2d::new(0.0, 0.0, 0.0));
        far.add_node(Pose2d::new(1.0, 0.0, 0.0));
        far.add_edge(PoseGraphEdge::odometry(0, 1, Pose2d::new(1.0, 0.0, 0.0)));
        // The second session is recognized 3 m to the side of the corridor.
        lifelong
            .merge_session(
                &far,
                &[SessionConstraint::new(
                    0,
                    0,
                    Pose2d::new(0.0, 3.0, 0.0),
                    (100.0, 100.0, 100.0),
                )],
                MergeOptions::default(),
            )
            .expect("merge");
        let report = lifelong.prune_superseded(PruneConfig::default());
        assert_eq!(report.removed, 0);
        assert_eq!(lifelong.graph().node_count(), 8);
    }

    #[test]
    fn a_compounded_edge_is_never_more_certain_than_its_parts() {
        let combined = in_series((100.0, 100.0, 50.0), (100.0, 25.0, 50.0));
        assert!((combined.0 - 50.0).abs() < 1e-9);
        assert!((combined.1 - 20.0).abs() < 1e-9);
        assert!((combined.2 - 25.0).abs() < 1e-9);
    }
    #[test]
    fn merging_onto_the_map_leaves_the_map_where_it_was() {
        let mut full = LifelongPoseGraph::from_first_session(drifting_session(6, 0.0));
        let mut fixed = full.clone();
        let prior: Vec<Pose2d> = full.graph().nodes().to_vec();
        // The second visit's odometry over-reports each step by 5 cm.
        let second = drifting_session(6, 0.05);
        let constraints = [
            SessionConstraint::new(0, 0, Pose2d::IDENTITY, (100.0, 100.0, 100.0)),
            SessionConstraint::new(5, 5, Pose2d::IDENTITY, (100.0, 100.0, 100.0)),
        ];
        full.merge_session(&second, &constraints, MergeOptions::default())
            .expect("merge");
        fixed
            .merge_session_onto_map(&second, &constraints, MergeOptions::default())
            .expect("merge onto map");
        // The full merge drags the first visit toward the second's drift...
        assert!(mean_error_m(&full.graph().nodes()[..6], &prior) > 1e-3);
        // ...merging onto the map does not move it at all,
        assert_eq!(&fixed.graph().nodes()[..6], prior.as_slice());
        // and fits the second visit to it instead.
        assert!(mean_error_m(&fixed.graph().nodes()[6..], &truth(6)) < 0.03);
        assert_eq!(fixed.graph().edge_count(), full.graph().edge_count());
        assert!(fixed.is_connected());
    }
    #[test]
    fn reference_sessions_are_never_pruned() {
        let mut lifelong = two_visits(6);
        let report = lifelong.prune_superseded(PruneConfig {
            reference_sessions: 1,
            ..PruneConfig::default()
        });
        assert_eq!(report.removed, 0);
        assert_eq!(lifelong.graph().node_count(), 12);
    }
}
