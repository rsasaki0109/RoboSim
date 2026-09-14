//! Versioned pose-graph serialization and multi-session combination.
//!
//! A [`PoseGraph`] is stored as versioned JSON carrying its nodes and edges, so
//! a mapping session can be saved and loaded bit-for-bit. [`combine_graphs`]
//! appends a second session's nodes and edges after the first, which is the
//! back-end primitive for multi-session mapping.

use crate::pose_graph::{PoseGraph, PoseGraphEdge};
use rne_nav::Pose2d;
use serde::{Deserialize, Serialize};
use std::path::Path;
use thiserror::Error;

/// Format tag written into every pose-graph file.
pub const RNE_POSE_GRAPH_FORMAT: &str = "rne.posegraph";
/// Current pose-graph format version.
pub const RNE_POSE_GRAPH_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct EdgeRecord {
    from: usize,
    to: usize,
    measurement: Pose2d,
    information: (f64, f64, f64),
    loop_closure: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct GraphFile {
    format: String,
    version: u32,
    nodes: Vec<Pose2d>,
    edges: Vec<EdgeRecord>,
}

/// Errors raised by pose-graph serialization.
#[derive(Debug, Error)]
pub enum GraphIoError {
    /// JSON (de)serialization failed.
    #[error("pose graph serialization failed: {0}")]
    Serialize(#[from] serde_json::Error),
    /// The file carried an unknown format tag.
    #[error("unsupported pose graph format {0:?}")]
    UnsupportedFormat(String),
    /// The file carried a newer or unknown version.
    #[error("unsupported pose graph version {0}")]
    UnsupportedVersion(u32),
    /// An edge referenced a node that does not exist.
    #[error("edge references node {0} which does not exist")]
    UnknownNode(usize),
    /// Reading or writing the graph file failed.
    #[error("pose graph file io failed: {0}")]
    Io(#[from] std::io::Error),
}

/// Serializes a pose graph to compact versioned JSON.
pub fn to_graph_json(graph: &PoseGraph) -> Result<String, GraphIoError> {
    let edges = graph
        .edges()
        .iter()
        .map(|edge| EdgeRecord {
            from: edge.from,
            to: edge.to,
            measurement: edge.measurement,
            information: edge.information,
            loop_closure: edge.loop_closure,
        })
        .collect();
    let file = GraphFile {
        format: RNE_POSE_GRAPH_FORMAT.to_string(),
        version: RNE_POSE_GRAPH_VERSION,
        nodes: graph.nodes().to_vec(),
        edges,
    };
    Ok(serde_json::to_string(&file)?)
}

/// Parses versioned JSON into a pose graph.
pub fn from_graph_json(text: &str) -> Result<PoseGraph, GraphIoError> {
    let file: GraphFile = serde_json::from_str(text)?;
    if file.format != RNE_POSE_GRAPH_FORMAT {
        return Err(GraphIoError::UnsupportedFormat(file.format));
    }
    if file.version != RNE_POSE_GRAPH_VERSION {
        return Err(GraphIoError::UnsupportedVersion(file.version));
    }
    let node_count = file.nodes.len();
    let mut graph = PoseGraph::new();
    for pose in file.nodes {
        graph.add_node(pose);
    }
    for edge in file.edges {
        if edge.from >= node_count || edge.to >= node_count {
            return Err(GraphIoError::UnknownNode(edge.from.max(edge.to)));
        }
        graph.add_edge(PoseGraphEdge {
            from: edge.from,
            to: edge.to,
            measurement: edge.measurement,
            information: edge.information,
            loop_closure: edge.loop_closure,
        });
    }
    Ok(graph)
}

/// Writes a pose graph to `path` in versioned JSON form.
pub fn save_graph(path: &Path, graph: &PoseGraph) -> Result<(), GraphIoError> {
    std::fs::write(path, to_graph_json(graph)?)?;
    Ok(())
}

/// Loads a pose graph from a versioned JSON file.
pub fn load_graph(path: &Path) -> Result<PoseGraph, GraphIoError> {
    let text = std::fs::read_to_string(path)?;
    from_graph_json(&text)
}

/// Appends `other`'s nodes and edges after `base`, offsetting edge indices.
pub fn combine_graphs(base: &PoseGraph, other: &PoseGraph) -> PoseGraph {
    let mut combined = PoseGraph::new();
    for pose in base.nodes() {
        combined.add_node(*pose);
    }
    for pose in other.nodes() {
        combined.add_node(*pose);
    }
    for edge in base.edges().iter().chain(other.edges()) {
        combined.add_edge(*edge);
    }
    let offset = base.node_count();
    let edge_count = base.edge_count();
    let edges: Vec<PoseGraphEdge> = combined
        .edges()
        .iter()
        .enumerate()
        .map(|(index, edge)| {
            if index < edge_count {
                *edge
            } else {
                PoseGraphEdge {
                    from: edge.from + offset,
                    to: edge.to + offset,
                    ..*edge
                }
            }
        })
        .collect();
    let nodes = combined.nodes().to_vec();
    let mut result = PoseGraph::new();
    for pose in nodes {
        result.add_node(pose);
    }
    for edge in edges {
        result.add_edge(edge);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_graph() -> PoseGraph {
        let mut graph = PoseGraph::new();
        let a = graph.add_node(Pose2d::new(0.0, 0.0, 0.0));
        let b = graph.add_node(Pose2d::new(1.0, 0.0, 0.1));
        let c = graph.add_node(Pose2d::new(2.0, 0.2, 0.2));
        graph.add_edge(PoseGraphEdge::odometry(a, b, Pose2d::new(1.0, 0.0, 0.1)));
        graph.add_edge(PoseGraphEdge::odometry(b, c, Pose2d::new(1.0, 0.2, 0.1)));
        graph.add_edge(PoseGraphEdge::loop_closure(
            a,
            c,
            Pose2d::new(2.0, 0.2, 0.2),
            (0.5, 0.5, 0.5),
        ));
        graph
    }

    #[test]
    fn empty_graph_golden_json_is_stable() {
        let graph = PoseGraph::new();
        assert_eq!(
            to_graph_json(&graph).unwrap(),
            "{\"format\":\"rne.posegraph\",\"version\":1,\"nodes\":[],\"edges\":[]}"
        );
    }

    #[test]
    fn round_trips_nodes_and_edges() {
        let graph = sample_graph();
        let restored = from_graph_json(&to_graph_json(&graph).unwrap()).unwrap();
        assert_eq!(restored.nodes(), graph.nodes());
        assert_eq!(restored.edges(), graph.edges());
    }

    #[test]
    fn combines_two_sessions_with_offset_indices() {
        let first = sample_graph();
        let second = sample_graph();
        let combined = combine_graphs(&first, &second);
        assert_eq!(
            combined.node_count(),
            first.node_count() + second.node_count()
        );
        assert_eq!(
            combined.edge_count(),
            first.edge_count() + second.edge_count()
        );
        let offset = first.node_count();
        let last = combined.edges().last().unwrap();
        assert!(last.from >= offset && last.to >= offset);
    }

    #[test]
    fn rejects_unknown_format_and_version() {
        let graph = sample_graph();
        let good = to_graph_json(&graph).unwrap();
        assert!(matches!(
            from_graph_json(&good.replace("\"rne.posegraph\"", "\"other\"")),
            Err(GraphIoError::UnsupportedFormat(_))
        ));
        assert!(matches!(
            from_graph_json(&good.replace("\"version\":1", "\"version\":2")),
            Err(GraphIoError::UnsupportedVersion(2))
        ));
    }
}
