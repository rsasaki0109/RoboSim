//! Internal coordinate frame graph.

use crate::Transform3;
use petgraph::graph::{NodeIndex, UnGraph};
use rne_math::Mat4;
use std::collections::HashMap;

/// Identifier for a coordinate frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameId(pub u32);

impl FrameId {
    /// World frame identifier.
    pub const WORLD: Self = Self(0);
}

/// Named coordinate frame node.
#[derive(Clone, Debug, PartialEq)]
pub struct FrameNode {
    /// Frame name.
    pub name: String,
}

/// Directed graph of coordinate frames and fixed transforms.
#[derive(Clone, Debug, Default)]
pub struct FrameGraph {
    graph: UnGraph<FrameNode, Transform3>,
    name_to_index: HashMap<String, NodeIndex>,
}

impl FrameGraph {
    /// Creates an empty frame graph with a world root frame.
    pub fn new() -> Self {
        let mut graph = Self::default();
        graph.add_frame(FrameId::WORLD, "world");
        graph
    }

    /// Adds a named frame and returns its graph index.
    pub fn add_frame(&mut self, id: FrameId, name: impl Into<String>) -> NodeIndex {
        let name = name.into();
        let index = self.graph.add_node(FrameNode { name: name.clone() });
        self.name_to_index.insert(name, index);
        let _ = id;
        index
    }

    /// Sets a fixed transform between parent and child frames.
    pub fn set_transform(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform3,
    ) -> Option<()> {
        let parent_index = *self.name_to_index.get(parent)?;
        let child_index = *self.name_to_index.get(child)?;
        self.graph.add_edge(parent_index, child_index, transform);
        Some(())
    }

    /// Computes the transform from `from` to `to` if connected.
    pub fn lookup_transform(&self, from: &str, to: &str) -> Option<Mat4> {
        let from_index = *self.name_to_index.get(from)?;
        let to_index = *self.name_to_index.get(to)?;

        if from_index == to_index {
            return Some(Mat4::IDENTITY);
        }

        let path = petgraph::algo::astar(
            &self.graph,
            from_index,
            |node| node == to_index,
            |_| 1,
            |_| 0,
        )?;

        let mut matrix = Mat4::IDENTITY;
        let mut current = path.1[0];
        for &next in &path.1[1..] {
            let edge = self
                .graph
                .find_edge(current, next)
                .or_else(|| self.graph.find_edge(next, current))?;
            let transform = if self
                .graph
                .edge_endpoints(edge)
                .is_some_and(|(a, _)| a == current)
            {
                self.graph[edge]
            } else {
                let math = rne_math::Transform3 {
                    translation: self.graph[edge].translation,
                    rotation: self.graph[edge].rotation,
                    scale: self.graph[edge].scale,
                };
                let inverse = math.inverse();
                Transform3 {
                    translation: inverse.translation,
                    rotation: inverse.rotation,
                    scale: inverse.scale,
                }
            };
            matrix = transform.to_matrix() * matrix;
            current = next;
        }

        Some(matrix)
    }

    /// Looks up a frame by name.
    pub fn frame(&self, name: &str) -> Option<&FrameNode> {
        self.name_to_index
            .get(name)
            .map(|index| &self.graph[*index])
    }

    /// Returns all static parent-to-child transforms in insertion order.
    pub fn edges(&self) -> Vec<FrameEdge> {
        self.graph
            .edge_indices()
            .filter_map(|edge| {
                let (parent_index, child_index) = self.graph.edge_endpoints(edge)?;
                Some(FrameEdge {
                    parent: self.graph[parent_index].name.clone(),
                    child: self.graph[child_index].name.clone(),
                    transform: self.graph[edge],
                })
            })
            .collect()
    }
}

/// Static transform edge in a frame graph.
#[derive(Clone, Debug, PartialEq)]
pub struct FrameEdge {
    /// Parent frame name.
    pub parent: String,
    /// Child frame name.
    pub child: String,
    /// Fixed transform from parent to child.
    pub transform: Transform3,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::{Quat, Vec3};

    #[test]
    fn frame_graph_lookup() {
        let mut graph = FrameGraph::new();
        graph.add_frame(FrameId(1), "base");
        graph.add_frame(FrameId(2), "sensor");
        graph
            .set_transform(
                "world",
                "base",
                Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
            )
            .unwrap();
        graph
            .set_transform(
                "base",
                "sensor",
                Transform3::from_translation_rotation(Vec3::new(0.0, 0.5, 0.0), Quat::IDENTITY),
            )
            .unwrap();

        let matrix = graph.lookup_transform("world", "sensor").unwrap();
        let point = matrix.transform_point3(Vec3::ZERO);
        assert_relative_eq!(point.x, 1.0);
        assert_relative_eq!(point.y, 0.5);
    }
}
