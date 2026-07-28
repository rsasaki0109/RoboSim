//! Deterministic lane topology construction.

use crate::{
    Accuracy, AccuracyClass, AuthorityClass, Junction, JunctionKind, Lane, MovementKind,
    Provenance, SourceReference, TrafficActorKind, TrafficAsset, TrafficConnection, TrafficId,
    TrafficNetwork,
};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use thiserror::Error;

const DIRECTION_EPSILON: f64 = 1.0e-9;

/// Parameters controlling deterministic endpoint clustering and turn generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TopologyBuildConfig {
    /// Maximum horizontal distance between endpoints assigned to one junction.
    pub endpoint_snap_m: f64,
    /// Maximum vertical endpoint separation considered at-grade.
    pub max_grade_separation_m: f64,
    /// Heading tolerance used to classify straight movements and merge arms.
    pub straight_tolerance_rad: f64,
    /// Angular distance from π classified as a U-turn.
    pub uturn_tolerance_rad: f64,
    /// Number of cubic Bézier segments in every generated connection path.
    pub turn_curve_segments: usize,
    /// Cubic handle length as a fraction of endpoint distance.
    pub turn_handle_scale: f64,
    /// Minimum cubic handle length in meters.
    pub min_turn_handle_m: f64,
    /// Horizontal path clearance used when detecting movement conflicts.
    pub conflict_clearance_m: f64,
    /// Whether U-turn connection candidates are emitted.
    pub allow_u_turns: bool,
}

impl Default for TopologyBuildConfig {
    fn default() -> Self {
        Self {
            endpoint_snap_m: 5.0,
            max_grade_separation_m: 1.0,
            straight_tolerance_rad: 20.0_f64.to_radians(),
            uturn_tolerance_rad: 20.0_f64.to_radians(),
            turn_curve_segments: 12,
            turn_handle_scale: 0.45,
            min_turn_handle_m: 1.0,
            conflict_clearance_m: 0.25,
            allow_u_turns: false,
        }
    }
}

/// Deterministic topology construction failure.
#[derive(Debug, Error)]
pub enum TopologyError {
    /// No source network was supplied.
    #[error("topology build requires at least one source network")]
    NoNetworks,
    /// Source networks contain no lanes.
    #[error("topology build requires at least one lane")]
    NoLanes,
    /// A configuration value is outside its supported range.
    #[error("invalid topology config `{field}`: {message}")]
    InvalidConfig {
        /// Configuration field.
        field: &'static str,
        /// Validation detail.
        message: &'static str,
    },
    /// An input traffic asset is invalid.
    #[error("invalid source network `{network_id}`: {message}")]
    InvalidInput {
        /// Stable source network ID.
        network_id: TrafficId,
        /// Schema validation detail.
        message: String,
    },
    /// Existing topology or signals would be overwritten.
    #[error("source network `{network_id}` already contains topology or signals")]
    ExistingTopology {
        /// Stable source network ID.
        network_id: TrafficId,
    },
    /// Source networks use different coordinate frames.
    #[error("source network `{network_id}` uses a different coordinate frame")]
    CoordinateFrameMismatch {
        /// Stable source network ID.
        network_id: TrafficId,
    },
    /// A lane ID is repeated across input networks.
    #[error("duplicate input lane ID `{lane_id}`")]
    DuplicateLane {
        /// Duplicated lane ID.
        lane_id: TrafficId,
    },
    /// A lane has no usable horizontal start or end heading.
    #[error("lane `{lane_id}` has degenerate endpoint geometry")]
    DegenerateLane {
        /// Invalid lane ID.
        lane_id: TrafficId,
    },
    /// A generated stable ID could not be represented.
    #[error("generated topology ID is invalid: {0}")]
    InvalidGeneratedId(String),
    /// The generated network violated traffic schema v1.
    #[error("generated topology is invalid: {0}")]
    InvalidOutput(String),
}

/// Counts describing one completed topology build.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TopologyBuildStats {
    /// Number of source networks.
    pub input_network_count: usize,
    /// Number of copied lanes.
    pub lane_count: usize,
    /// Number of generated junctions.
    pub junction_count: usize,
    /// Number of generated lane connections.
    pub connection_count: usize,
    /// Number of symmetric connection-conflict pairs.
    pub conflict_pair_count: usize,
    /// Number of generated tile-boundary junctions.
    pub tile_boundary_count: usize,
}

/// Generated network and deterministic construction statistics.
#[derive(Clone, Debug, PartialEq)]
pub struct TopologyBuildResult {
    /// Canonically ordered output network.
    pub network: TrafficNetwork,
    /// Build counts.
    pub stats: TopologyBuildStats,
}

/// Reusable deterministic topology builder.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TopologyBuilder {
    config: TopologyBuildConfig,
}

impl TopologyBuilder {
    /// Creates a builder after validating all numeric parameters.
    pub fn new(config: TopologyBuildConfig) -> Result<Self, TopologyError> {
        validate_config(config)?;
        Ok(Self { config })
    }

    /// Returns the active build configuration.
    pub const fn config(&self) -> TopologyBuildConfig {
        self.config
    }

    /// Merges lane-only networks and constructs deterministic junction topology.
    pub fn build(
        &self,
        output_network_id: TrafficId,
        networks: &[TrafficNetwork],
    ) -> Result<TopologyBuildResult, TopologyError> {
        build_impl(output_network_id, networks, self.config)
    }
}

/// Builds deterministic topology using one configuration.
pub fn build_traffic_topology(
    output_network_id: TrafficId,
    networks: &[TrafficNetwork],
    config: TopologyBuildConfig,
) -> Result<TopologyBuildResult, TopologyError> {
    TopologyBuilder::new(config)?.build(output_network_id, networks)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum EndpointRole {
    End,
    Start,
}

#[derive(Clone, Debug)]
struct LaneRecord {
    lane: Lane,
    source_network_id: TrafficId,
    start_heading: [f64; 2],
    end_heading: [f64; 2],
}

#[derive(Clone, Debug)]
struct Endpoint {
    lane_index: usize,
    lane_id: TrafficId,
    source_network_id: TrafficId,
    role: EndpointRole,
    position_m: [f64; 3],
    outward: [f64; 2],
}

#[derive(Clone, Debug)]
struct CandidateConnection {
    incoming_lane_index: usize,
    outgoing_lane_index: usize,
    movement: MovementKind,
}

fn build_impl(
    output_network_id: TrafficId,
    networks: &[TrafficNetwork],
    config: TopologyBuildConfig,
) -> Result<TopologyBuildResult, TopologyError> {
    if networks.is_empty() {
        return Err(TopologyError::NoNetworks);
    }
    let coordinate_frame = networks[0].coordinate_frame.clone();
    let mut lane_ids = BTreeSet::new();
    let mut lanes = Vec::new();
    for network in networks {
        TrafficAsset::new(network.clone())
            .validate()
            .map_err(|error| TopologyError::InvalidInput {
                network_id: network.id.clone(),
                message: error.to_string(),
            })?;
        if !network.junctions.is_empty()
            || !network.connections.is_empty()
            || !network.signals.is_empty()
        {
            return Err(TopologyError::ExistingTopology {
                network_id: network.id.clone(),
            });
        }
        if network.coordinate_frame != coordinate_frame {
            return Err(TopologyError::CoordinateFrameMismatch {
                network_id: network.id.clone(),
            });
        }
        for lane in &network.lanes {
            if !lane_ids.insert(lane.id.clone()) {
                return Err(TopologyError::DuplicateLane {
                    lane_id: lane.id.clone(),
                });
            }
            lanes.push(LaneRecord {
                start_heading: endpoint_heading(&lane.centerline_m, true).ok_or_else(|| {
                    TopologyError::DegenerateLane {
                        lane_id: lane.id.clone(),
                    }
                })?,
                end_heading: endpoint_heading(&lane.centerline_m, false).ok_or_else(|| {
                    TopologyError::DegenerateLane {
                        lane_id: lane.id.clone(),
                    }
                })?,
                lane: lane.clone(),
                source_network_id: network.id.clone(),
            });
        }
    }
    if lanes.is_empty() {
        return Err(TopologyError::NoLanes);
    }
    lanes.sort_by(|left, right| left.lane.id.cmp(&right.lane.id));

    let mut endpoints = Vec::with_capacity(lanes.len() * 2);
    for (lane_index, record) in lanes.iter().enumerate() {
        endpoints.push(Endpoint {
            lane_index,
            lane_id: record.lane.id.clone(),
            source_network_id: record.source_network_id.clone(),
            role: EndpointRole::End,
            position_m: *record.lane.centerline_m.last().expect("validated lane"),
            outward: [-record.end_heading[0], -record.end_heading[1]],
        });
        endpoints.push(Endpoint {
            lane_index,
            lane_id: record.lane.id.clone(),
            source_network_id: record.source_network_id.clone(),
            role: EndpointRole::Start,
            position_m: record.lane.centerline_m[0],
            outward: record.start_heading,
        });
    }
    endpoints.sort_by(|left, right| {
        left.lane_id
            .cmp(&right.lane_id)
            .then_with(|| left.role.cmp(&right.role))
    });

    let clusters = cluster_endpoints(&endpoints, config);
    let mut junctions = Vec::new();
    let mut connections = Vec::new();
    let mut tile_boundary_count = 0;
    for members in clusters.values() {
        let incoming: Vec<_> = members
            .iter()
            .copied()
            .filter(|index| endpoints[*index].role == EndpointRole::End)
            .collect();
        let outgoing: Vec<_> = members
            .iter()
            .copied()
            .filter(|index| endpoints[*index].role == EndpointRole::Start)
            .collect();
        if incoming.is_empty() || outgoing.is_empty() {
            continue;
        }
        let candidates = connection_candidates(&incoming, &outgoing, &endpoints, &lanes, config);
        if candidates.is_empty() {
            continue;
        }
        let junction_id = generated_junction_id(members, &endpoints)?;
        let kind = classify_junction(members, &incoming, &outgoing, &endpoints, config);
        if kind == JunctionKind::TileBoundary {
            tile_boundary_count += 1;
        }
        let center_m = mean_position(members, &endpoints);
        let sources = sources_for_members(members, &endpoints, &lanes);
        junctions.push(Junction {
            id: junction_id.clone(),
            provenance: topology_provenance(
                sources.clone(),
                config,
                "endpoint clustering and arm classification",
            ),
            kind,
            center_m,
        });
        for candidate in candidates {
            let incoming_lane = &lanes[candidate.incoming_lane_index].lane;
            let outgoing_lane = &lanes[candidate.outgoing_lane_index].lane;
            connections.push(TrafficConnection {
                id: generated_connection_id(&junction_id, &incoming_lane.id, &outgoing_lane.id)?,
                provenance: topology_provenance(
                    merge_sources(&incoming_lane.provenance, &outgoing_lane.provenance),
                    config,
                    "cubic turn curve from stable lane endpoint headings",
                ),
                incoming_lane_id: incoming_lane.id.clone(),
                outgoing_lane_id: outgoing_lane.id.clone(),
                junction_id: Some(junction_id.clone()),
                movement: candidate.movement,
                path_m: turn_curve(incoming_lane, outgoing_lane, config),
                conflict_connection_ids: Vec::new(),
                signal_group_id: None,
            });
        }
    }

    junctions.sort_by(|left, right| left.id.cmp(&right.id));
    connections.sort_by(|left, right| left.id.cmp(&right.id));
    let conflict_pair_count = populate_conflicts(&mut connections, config);
    let mut network_sources = networks
        .iter()
        .flat_map(|network| network.provenance.sources.iter().cloned())
        .collect::<Vec<_>>();
    if network_sources.is_empty() {
        network_sources.extend(networks.iter().map(|network| SourceReference {
            dataset: "RNE traffic network".into(),
            feature_id: Some(network.id.to_string()),
            uri: None,
        }));
    }
    network_sources.sort();
    network_sources.dedup();

    let output = TrafficAsset::new(TrafficNetwork {
        id: output_network_id,
        provenance: topology_provenance(
            network_sources,
            config,
            "deterministic multi-network topology construction",
        ),
        coordinate_frame,
        lanes: lanes.into_iter().map(|record| record.lane).collect(),
        junctions,
        connections,
        signals: Vec::new(),
    })
    .canonicalized();
    output
        .validate()
        .map_err(|error| TopologyError::InvalidOutput(error.to_string()))?;

    let stats = TopologyBuildStats {
        input_network_count: networks.len(),
        lane_count: output.network.lanes.len(),
        junction_count: output.network.junctions.len(),
        connection_count: output.network.connections.len(),
        conflict_pair_count,
        tile_boundary_count,
    };
    Ok(TopologyBuildResult {
        network: output.network,
        stats,
    })
}

fn validate_config(config: TopologyBuildConfig) -> Result<(), TopologyError> {
    validate_positive_config("endpoint_snap_m", config.endpoint_snap_m)?;
    validate_nonnegative_config("max_grade_separation_m", config.max_grade_separation_m)?;
    validate_angle_config("straight_tolerance_rad", config.straight_tolerance_rad)?;
    validate_angle_config("uturn_tolerance_rad", config.uturn_tolerance_rad)?;
    if config.turn_curve_segments < 2 {
        return Err(TopologyError::InvalidConfig {
            field: "turn_curve_segments",
            message: "must be at least 2",
        });
    }
    validate_positive_config("turn_handle_scale", config.turn_handle_scale)?;
    validate_nonnegative_config("min_turn_handle_m", config.min_turn_handle_m)?;
    validate_nonnegative_config("conflict_clearance_m", config.conflict_clearance_m)
}

fn validate_positive_config(field: &'static str, value: f64) -> Result<(), TopologyError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(TopologyError::InvalidConfig {
            field,
            message: "must be finite and greater than zero",
        })
    }
}

fn validate_nonnegative_config(field: &'static str, value: f64) -> Result<(), TopologyError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(TopologyError::InvalidConfig {
            field,
            message: "must be finite and non-negative",
        })
    }
}

fn validate_angle_config(field: &'static str, value: f64) -> Result<(), TopologyError> {
    if value.is_finite() && value > 0.0 && value < PI * 0.5 {
        Ok(())
    } else {
        Err(TopologyError::InvalidConfig {
            field,
            message: "must be finite and between zero and pi/2",
        })
    }
}

fn endpoint_heading(points: &[[f64; 3]], start: bool) -> Option<[f64; 2]> {
    if start {
        let origin = points[0];
        points
            .iter()
            .skip(1)
            .find_map(|point| normalize_horizontal([point[0] - origin[0], point[2] - origin[2]]))
    } else {
        let origin = *points.last()?;
        points
            .iter()
            .rev()
            .skip(1)
            .find_map(|point| normalize_horizontal([origin[0] - point[0], origin[2] - point[2]]))
    }
}

fn normalize_horizontal(vector: [f64; 2]) -> Option<[f64; 2]> {
    let length = vector[0].hypot(vector[1]);
    (length > DIRECTION_EPSILON).then_some([vector[0] / length, vector[1] / length])
}

fn cluster_endpoints(
    endpoints: &[Endpoint],
    config: TopologyBuildConfig,
) -> BTreeMap<usize, Vec<usize>> {
    let mut parents: Vec<_> = (0..endpoints.len()).collect();
    for left in 0..endpoints.len() {
        for right in (left + 1)..endpoints.len() {
            if endpoints_at_grade_and_near(&endpoints[left], &endpoints[right], config) {
                union_stable(&mut parents, left, right);
            }
        }
    }
    let mut clusters = BTreeMap::<usize, Vec<usize>>::new();
    for index in 0..endpoints.len() {
        let root = find_root(&mut parents, index);
        clusters.entry(root).or_default().push(index);
    }
    clusters
}

fn endpoints_at_grade_and_near(
    left: &Endpoint,
    right: &Endpoint,
    config: TopologyBuildConfig,
) -> bool {
    let horizontal =
        (left.position_m[0] - right.position_m[0]).hypot(left.position_m[2] - right.position_m[2]);
    let vertical = (left.position_m[1] - right.position_m[1]).abs();
    horizontal <= config.endpoint_snap_m && vertical <= config.max_grade_separation_m
}

fn find_root(parents: &mut [usize], index: usize) -> usize {
    if parents[index] != index {
        parents[index] = find_root(parents, parents[index]);
    }
    parents[index]
}

fn union_stable(parents: &mut [usize], left: usize, right: usize) {
    let left_root = find_root(parents, left);
    let right_root = find_root(parents, right);
    if left_root == right_root {
        return;
    }
    let root = left_root.min(right_root);
    let child = left_root.max(right_root);
    parents[child] = root;
}

fn connection_candidates(
    incoming: &[usize],
    outgoing: &[usize],
    endpoints: &[Endpoint],
    lanes: &[LaneRecord],
    config: TopologyBuildConfig,
) -> Vec<CandidateConnection> {
    let mut candidates = Vec::new();
    for incoming_endpoint in incoming {
        let incoming_index = endpoints[*incoming_endpoint].lane_index;
        for outgoing_endpoint in outgoing {
            let outgoing_index = endpoints[*outgoing_endpoint].lane_index;
            if incoming_index == outgoing_index
                || !actor_classes_overlap(
                    &lanes[incoming_index].lane.allowed_actors,
                    &lanes[outgoing_index].lane.allowed_actors,
                )
            {
                continue;
            }
            let movement = classify_movement(
                lanes[incoming_index].end_heading,
                lanes[outgoing_index].start_heading,
                config,
            );
            if movement == MovementKind::UTurn && !config.allow_u_turns {
                continue;
            }
            candidates.push(CandidateConnection {
                incoming_lane_index: incoming_index,
                outgoing_lane_index: outgoing_index,
                movement,
            });
        }
    }
    candidates.sort_by(|left, right| {
        lanes[left.incoming_lane_index]
            .lane
            .id
            .cmp(&lanes[right.incoming_lane_index].lane.id)
            .then_with(|| {
                lanes[left.outgoing_lane_index]
                    .lane
                    .id
                    .cmp(&lanes[right.outgoing_lane_index].lane.id)
            })
    });
    candidates
}

fn actor_classes_overlap(left: &[TrafficActorKind], right: &[TrafficActorKind]) -> bool {
    left.iter().any(|actor| right.contains(actor))
}

fn classify_movement(
    incoming: [f64; 2],
    outgoing: [f64; 2],
    config: TopologyBuildConfig,
) -> MovementKind {
    let dot = (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]).clamp(-1.0, 1.0);
    let cross_y = incoming[1] * outgoing[0] - incoming[0] * outgoing[1];
    let angle = cross_y.atan2(dot);
    if angle.abs() <= config.straight_tolerance_rad {
        MovementKind::Straight
    } else if PI - angle.abs() <= config.uturn_tolerance_rad {
        MovementKind::UTurn
    } else if angle > 0.0 {
        MovementKind::Left
    } else {
        MovementKind::Right
    }
}

fn classify_junction(
    members: &[usize],
    incoming: &[usize],
    outgoing: &[usize],
    endpoints: &[Endpoint],
    config: TopologyBuildConfig,
) -> JunctionKind {
    let source_networks: BTreeSet<_> = members
        .iter()
        .map(|index| endpoints[*index].source_network_id.clone())
        .collect();
    if source_networks.len() > 1 && incoming.len() == 1 && outgoing.len() == 1 {
        return JunctionKind::TileBoundary;
    }
    let mut arms: Vec<[f64; 2]> = Vec::new();
    for member in members {
        let direction = endpoints[*member].outward;
        if !arms
            .iter()
            .any(|known| angle_between(*known, direction).abs() <= config.straight_tolerance_rad)
        {
            arms.push(direction);
        }
    }
    if arms.len() >= 4 {
        JunctionKind::CrossIntersection
    } else if arms.len() == 3 {
        JunctionKind::TIntersection
    } else if incoming.len() > 1 && outgoing.len() == 1 {
        JunctionKind::Merge
    } else if incoming.len() == 1 && outgoing.len() > 1 {
        JunctionKind::Split
    } else {
        JunctionKind::Intersection
    }
}

fn angle_between(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dot = (left[0] * right[0] + left[1] * right[1]).clamp(-1.0, 1.0);
    let cross_y = left[1] * right[0] - left[0] * right[1];
    cross_y.atan2(dot)
}

fn mean_position(members: &[usize], endpoints: &[Endpoint]) -> [f64; 3] {
    let count = members.len() as f64;
    let mut sum = [0.0; 3];
    for member in members {
        for (axis, value) in sum.iter_mut().enumerate() {
            *value += endpoints[*member].position_m[axis];
        }
    }
    [sum[0] / count, sum[1] / count, sum[2] / count]
}

fn turn_curve(
    incoming_lane: &Lane,
    outgoing_lane: &Lane,
    config: TopologyBuildConfig,
) -> Vec<[f64; 3]> {
    let start = *incoming_lane
        .centerline_m
        .last()
        .expect("validated incoming lane");
    let end = outgoing_lane.centerline_m[0];
    let incoming_heading =
        endpoint_heading(&incoming_lane.centerline_m, false).expect("validated heading");
    let outgoing_heading =
        endpoint_heading(&outgoing_lane.centerline_m, true).expect("validated heading");
    let distance = (end[0] - start[0]).hypot(end[2] - start[2]);
    let handle = (distance * config.turn_handle_scale).max(config.min_turn_handle_m);
    let control_1 = [
        start[0] + incoming_heading[0] * handle,
        start[1] + (end[1] - start[1]) / 3.0,
        start[2] + incoming_heading[1] * handle,
    ];
    let control_2 = [
        end[0] - outgoing_heading[0] * handle,
        start[1] + (end[1] - start[1]) * 2.0 / 3.0,
        end[2] - outgoing_heading[1] * handle,
    ];
    (0..=config.turn_curve_segments)
        .map(|index| {
            let t = index as f64 / config.turn_curve_segments as f64;
            cubic_bezier(start, control_1, control_2, end, t)
        })
        .collect()
}

fn cubic_bezier(
    start: [f64; 3],
    control_1: [f64; 3],
    control_2: [f64; 3],
    end: [f64; 3],
    t: f64,
) -> [f64; 3] {
    let inverse = 1.0 - t;
    let weights = [
        inverse * inverse * inverse,
        3.0 * inverse * inverse * t,
        3.0 * inverse * t * t,
        t * t * t,
    ];
    let mut point = [0.0; 3];
    for axis in 0..3 {
        point[axis] = start[axis] * weights[0]
            + control_1[axis] * weights[1]
            + control_2[axis] * weights[2]
            + end[axis] * weights[3];
    }
    point
}

fn populate_conflicts(connections: &mut [TrafficConnection], config: TopologyBuildConfig) -> usize {
    let mut pairs = Vec::new();
    for left in 0..connections.len() {
        for right in (left + 1)..connections.len() {
            if connections[left].junction_id != connections[right].junction_id
                || connections[left].incoming_lane_id == connections[right].incoming_lane_id
                || connections[left].outgoing_lane_id == connections[right].outgoing_lane_id
            {
                continue;
            }
            if paths_conflict(
                &connections[left].path_m,
                &connections[right].path_m,
                config,
            ) {
                pairs.push((left, right));
            }
        }
    }
    for (left, right) in &pairs {
        let left_id = connections[*left].id.clone();
        let right_id = connections[*right].id.clone();
        connections[*left].conflict_connection_ids.push(right_id);
        connections[*right].conflict_connection_ids.push(left_id);
    }
    pairs.len()
}

fn paths_conflict(left: &[[f64; 3]], right: &[[f64; 3]], config: TopologyBuildConfig) -> bool {
    let left_y = left.iter().map(|point| point[1]).sum::<f64>() / left.len() as f64;
    let right_y = right.iter().map(|point| point[1]).sum::<f64>() / right.len() as f64;
    if (left_y - right_y).abs() > config.max_grade_separation_m {
        return false;
    }
    left.windows(2).any(|left_segment| {
        right.windows(2).any(|right_segment| {
            segment_distance_xz(
                left_segment[0],
                left_segment[1],
                right_segment[0],
                right_segment[1],
            ) <= config.conflict_clearance_m
        })
    })
}

fn segment_distance_xz(
    left_start: [f64; 3],
    left_end: [f64; 3],
    right_start: [f64; 3],
    right_end: [f64; 3],
) -> f64 {
    let a = [left_start[0], left_start[2]];
    let b = [left_end[0], left_end[2]];
    let c = [right_start[0], right_start[2]];
    let d = [right_end[0], right_end[2]];
    if segments_intersect(a, b, c, d) {
        return 0.0;
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}

fn segments_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let ab_c = orientation(a, b, c);
    let ab_d = orientation(a, b, d);
    let cd_a = orientation(c, d, a);
    let cd_b = orientation(c, d, b);
    (ab_c * ab_d <= 0.0 && cd_a * cd_b <= 0.0) && bounding_boxes_overlap(a, b, c, d)
}

fn orientation(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn bounding_boxes_overlap(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let left_min_x = a[0].min(b[0]);
    let left_max_x = a[0].max(b[0]);
    let left_min_y = a[1].min(b[1]);
    let left_max_y = a[1].max(b[1]);
    let right_min_x = c[0].min(d[0]);
    let right_max_x = c[0].max(d[0]);
    let right_min_y = c[1].min(d[1]);
    let right_max_y = c[1].max(d[1]);
    left_min_x <= right_max_x
        && right_min_x <= left_max_x
        && left_min_y <= right_max_y
        && right_min_y <= left_max_y
}

fn point_segment_distance(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> f64 {
    let segment = [end[0] - start[0], end[1] - start[1]];
    let length_squared = segment[0] * segment[0] + segment[1] * segment[1];
    if length_squared <= DIRECTION_EPSILON {
        return (point[0] - start[0]).hypot(point[1] - start[1]);
    }
    let projection = (((point[0] - start[0]) * segment[0] + (point[1] - start[1]) * segment[1])
        / length_squared)
        .clamp(0.0, 1.0);
    let closest = [
        start[0] + projection * segment[0],
        start[1] + projection * segment[1],
    ];
    (point[0] - closest[0]).hypot(point[1] - closest[1])
}

fn generated_junction_id(
    members: &[usize],
    endpoints: &[Endpoint],
) -> Result<TrafficId, TopologyError> {
    let mut identity = String::new();
    for member in members {
        let endpoint = &endpoints[*member];
        identity.push_str(endpoint.lane_id.as_str());
        identity.push(match endpoint.role {
            EndpointRole::End => 'E',
            EndpointRole::Start => 'S',
        });
        identity.push('\n');
    }
    generated_id("junction", &identity)
}

fn generated_connection_id(
    junction_id: &TrafficId,
    incoming_lane_id: &TrafficId,
    outgoing_lane_id: &TrafficId,
) -> Result<TrafficId, TopologyError> {
    generated_id(
        "connection",
        &format!("{junction_id}\n{incoming_lane_id}\n{outgoing_lane_id}"),
    )
}

fn generated_id(kind: &str, identity: &str) -> Result<TrafficId, TopologyError> {
    let hash = stable_hash_128(identity.as_bytes());
    TrafficId::new(format!("topology:{kind}-{hash:032x}"))
        .map_err(|error| TopologyError::InvalidGeneratedId(error.to_string()))
}

fn stable_hash_128(bytes: &[u8]) -> u128 {
    const OFFSET: u128 = 144_066_263_297_769_815_596_495_629_667_062_367_629;
    const PRIME: u128 = 309_485_009_821_345_068_724_781_371;
    bytes.iter().fold(OFFSET, |hash, byte| {
        (hash ^ u128::from(*byte)).wrapping_mul(PRIME)
    })
}

fn sources_for_members(
    members: &[usize],
    endpoints: &[Endpoint],
    lanes: &[LaneRecord],
) -> Vec<SourceReference> {
    let mut sources = members
        .iter()
        .flat_map(|member| {
            lanes[endpoints[*member].lane_index]
                .lane
                .provenance
                .sources
                .iter()
                .cloned()
        })
        .collect::<Vec<_>>();
    if sources.is_empty() {
        sources.extend(members.iter().map(|member| SourceReference {
            dataset: "RNE traffic lane".into(),
            feature_id: Some(endpoints[*member].lane_id.to_string()),
            uri: None,
        }));
    }
    sources.sort();
    sources.dedup();
    sources
}

fn merge_sources(left: &Provenance, right: &Provenance) -> Vec<SourceReference> {
    let mut sources = left
        .sources
        .iter()
        .chain(&right.sources)
        .cloned()
        .collect::<Vec<_>>();
    if sources.is_empty() {
        sources.push(SourceReference {
            dataset: "RNE traffic topology".into(),
            feature_id: None,
            uri: None,
        });
    }
    sources.sort();
    sources.dedup();
    sources
}

fn topology_provenance(
    sources: Vec<SourceReference>,
    config: TopologyBuildConfig,
    method: &'static str,
) -> Provenance {
    Provenance {
        authority: AuthorityClass::Derived,
        accuracy: Accuracy {
            class: AccuracyClass::Derived,
            horizontal_m: Some(config.endpoint_snap_m),
            vertical_m: Some(config.max_grade_separation_m),
        },
        sources,
        method: Some(method.into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_hash_has_fixed_vector() {
        assert_eq!(
            stable_hash_128(b"RNE traffic topology"),
            0xc6b4856e9b52653658782caefa46ef76
        );
    }

    #[test]
    fn movement_turn_sign_matches_y_up_coordinates() {
        let config = TopologyBuildConfig::default();
        assert_eq!(
            classify_movement([1.0, 0.0], [0.0, -1.0], config),
            MovementKind::Left
        );
        assert_eq!(
            classify_movement([1.0, 0.0], [0.0, 1.0], config),
            MovementKind::Right
        );
    }
}
