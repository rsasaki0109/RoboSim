//! Timestamped transform tree with shortest-path lookup and time interpolation.
//!
//! The buffer stores one edge per parent → child relationship. Lookups traverse
//! the tree in either direction and interpolate each edge with linear
//! translation and spherical rotation interpolation. This covers the usual
//! `map → odom → base_link → sensor` chains without depending on `tf2` or ROS.

use rne_math::Transform3;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fmt;
use thiserror::Error;

/// Name of a coordinate frame.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct FrameId(String);

impl FrameId {
    /// Creates a frame id.
    pub fn new(name: impl Into<String>) -> Self {
        Self(name.into())
    }

    /// Borrows the frame name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for FrameId {
    fn from(value: &str) -> Self {
        Self(value.to_string())
    }
}

impl From<String> for FrameId {
    fn from(value: String) -> Self {
        Self(value)
    }
}

impl fmt::Display for FrameId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

/// A transform sampled at a specific time.
#[derive(Clone, Debug, PartialEq)]
pub struct StampedTransform {
    /// Parent frame.
    pub parent: FrameId,
    /// Child frame.
    pub child: FrameId,
    /// Sample time in seconds.
    pub time_s: f64,
    /// Child pose in the parent frame.
    pub transform: Transform3,
}

impl StampedTransform {
    /// Creates a stamped transform.
    pub fn new(
        parent: impl Into<FrameId>,
        child: impl Into<FrameId>,
        time_s: f64,
        transform: Transform3,
    ) -> Self {
        Self {
            parent: parent.into(),
            child: child.into(),
            time_s,
            transform,
        }
    }
}

/// Error returned by transform-tree operations.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum TfError {
    /// A frame is not part of the tree.
    #[error("frame '{0}' is not part of the transform tree")]
    UnknownFrame(FrameId),
    /// No path connects the two frames.
    #[error("no transform path from '{from}' to '{to}'")]
    NoPath {
        /// Source frame.
        from: FrameId,
        /// Target frame.
        to: FrameId,
    },
    /// A frame already has a different parent.
    #[error("frame '{0}' already has a different parent")]
    MultipleParents(FrameId),
    /// A transform edge has no samples.
    #[error("transform edge '{parent}' -> '{child}' has no samples")]
    EmptyEdge {
        /// Parent frame.
        parent: FrameId,
        /// Child frame.
        child: FrameId,
    },
    /// The requested time lies outside the recorded range.
    #[error("transform '{parent}' -> '{child}' requires extrapolation at {time_s} s")]
    ExtrapolationRequired {
        /// Parent frame.
        parent: FrameId,
        /// Child frame.
        child: FrameId,
        /// Requested time.
        time_s: f64,
    },
    /// A sample time was not finite.
    #[error("transform sample time must be finite")]
    NonFiniteTime,
}

#[derive(Clone, Debug)]
struct Edge {
    parent: FrameId,
    child: FrameId,
    samples: Vec<StampedTransform>,
}

/// A transform tree buffer.
#[derive(Clone, Debug, Default)]
pub struct TfBuffer {
    edges: Vec<Edge>,
    index: HashMap<(FrameId, FrameId), usize>,
    child_parent: HashMap<FrameId, FrameId>,
    allow_extrapolation: bool,
}

impl TfBuffer {
    /// Creates an empty buffer that rejects extrapolation.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets whether lookups outside the recorded time range clamp to the
    /// nearest sample instead of returning an error.
    pub fn set_allow_extrapolation(&mut self, allow: bool) {
        self.allow_extrapolation = allow;
    }

    /// Number of transform edges.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// All frame ids in deterministic order.
    pub fn frames(&self) -> Vec<FrameId> {
        let mut frames: Vec<FrameId> = self
            .edges
            .iter()
            .flat_map(|edge| [edge.parent.clone(), edge.child.clone()])
            .collect();
        frames.sort();
        frames.dedup();
        frames
    }

    /// Returns the most recent sample of every edge, sorted by frame ids.
    ///
    /// Useful for publishing the whole tree as a static `TFMessage`.
    pub fn latest_transforms(&self) -> Vec<StampedTransform> {
        let mut transforms: Vec<StampedTransform> = self
            .edges
            .iter()
            .filter_map(|edge| edge.samples.last().cloned())
            .collect();
        transforms.sort_by(|a, b| a.parent.cmp(&b.parent).then_with(|| a.child.cmp(&b.child)));
        transforms
    }

    /// Inserts or replaces a sample on a parent → child edge.
    pub fn set_transform(&mut self, transform: StampedTransform) -> Result<(), TfError> {
        if !transform.time_s.is_finite() {
            return Err(TfError::NonFiniteTime);
        }
        if let Some(existing) = self.child_parent.get(&transform.child) {
            if existing != &transform.parent {
                return Err(TfError::MultipleParents(transform.child.clone()));
            }
        }
        let key = (transform.parent.clone(), transform.child.clone());
        if let Some(&edge_index) = self.index.get(&key) {
            let samples = &mut self.edges[edge_index].samples;
            match samples.binary_search_by(|sample| sample.time_s.total_cmp(&transform.time_s)) {
                Ok(position) => samples[position] = transform,
                Err(position) => samples.insert(position, transform),
            }
        } else {
            self.child_parent
                .insert(transform.child.clone(), transform.parent.clone());
            self.index.insert(key, self.edges.len());
            let parent = transform.parent.clone();
            let child = transform.child.clone();
            self.edges.push(Edge {
                parent,
                child,
                samples: vec![transform],
            });
        }
        Ok(())
    }

    /// Looks up the pose of `child` in `parent` at `time_s`, interpolating.
    pub fn lookup(
        &self,
        parent: &FrameId,
        child: &FrameId,
        time_s: f64,
    ) -> Result<Transform3, TfError> {
        if !time_s.is_finite() {
            return Err(TfError::NonFiniteTime);
        }
        if parent == child {
            return Ok(Transform3::IDENTITY);
        }
        let known = self.frames();
        if !known.contains(parent) {
            return Err(TfError::UnknownFrame(parent.clone()));
        }
        if !known.contains(child) {
            return Err(TfError::UnknownFrame(child.clone()));
        }

        let mut predecessors: HashMap<FrameId, Option<(FrameId, bool)>> = HashMap::new();
        let mut queue: VecDeque<FrameId> = VecDeque::new();
        queue.push_back(parent.clone());
        predecessors.insert(parent.clone(), None);
        while let Some(node) = queue.pop_front() {
            if node == *child {
                break;
            }
            for (neighbor, forward) in self.neighbors(&node) {
                if let std::collections::hash_map::Entry::Vacant(entry) =
                    predecessors.entry(neighbor.clone())
                {
                    entry.insert(Some((node.clone(), forward)));
                    queue.push_back(neighbor);
                }
            }
        }
        if !predecessors.contains_key(child) {
            return Err(TfError::NoPath {
                from: parent.clone(),
                to: child.clone(),
            });
        }

        let mut path: Vec<(FrameId, FrameId, bool)> = Vec::new();
        let mut current = child.clone();
        while let Some(Some((previous, forward))) = predecessors.get(&current).cloned() {
            path.push((previous.clone(), current.clone(), forward));
            current = previous;
        }
        path.reverse();

        let mut accumulated = Transform3::IDENTITY;
        for (previous, next, forward) in path {
            let (edge_parent, edge_child) = if forward {
                (previous, next)
            } else {
                (next, previous)
            };
            let edge =
                self.find_edge(&edge_parent, &edge_child)
                    .ok_or_else(|| TfError::NoPath {
                        from: parent.clone(),
                        to: child.clone(),
                    })?;
            let sample = self.sample_edge(edge, time_s)?;
            let step = if forward { sample } else { sample.inverse() };
            accumulated = accumulated.mul_transform(&step);
        }
        Ok(accumulated)
    }

    fn neighbors(&self, node: &FrameId) -> Vec<(FrameId, bool)> {
        let mut result: Vec<(FrameId, bool)> = Vec::new();
        for edge in &self.edges {
            if edge.parent == *node {
                result.push((edge.child.clone(), true));
            } else if edge.child == *node {
                result.push((edge.parent.clone(), false));
            }
        }
        result.sort_by(|a, b| a.0.cmp(&b.0));
        result.dedup_by(|a, b| a.0 == b.0);
        result
    }

    fn find_edge(&self, parent: &FrameId, child: &FrameId) -> Option<&Edge> {
        self.index
            .get(&(parent.clone(), child.clone()))
            .map(|index| &self.edges[*index])
    }

    fn sample_edge(&self, edge: &Edge, time_s: f64) -> Result<Transform3, TfError> {
        if edge.samples.is_empty() {
            return Err(TfError::EmptyEdge {
                parent: edge.parent.clone(),
                child: edge.child.clone(),
            });
        }
        if edge.samples.len() == 1 {
            return Ok(edge.samples[0].transform);
        }
        let first = &edge.samples[0];
        let last = &edge.samples[edge.samples.len() - 1];
        if time_s <= first.time_s {
            return if time_s == first.time_s || self.allow_extrapolation {
                Ok(first.transform)
            } else {
                Err(TfError::ExtrapolationRequired {
                    parent: edge.parent.clone(),
                    child: edge.child.clone(),
                    time_s,
                })
            };
        }
        if time_s >= last.time_s {
            return if time_s == last.time_s || self.allow_extrapolation {
                Ok(last.transform)
            } else {
                Err(TfError::ExtrapolationRequired {
                    parent: edge.parent.clone(),
                    child: edge.child.clone(),
                    time_s,
                })
            };
        }

        let upper = edge
            .samples
            .partition_point(|sample| sample.time_s <= time_s);
        let lower = &edge.samples[upper - 1];
        let upper = &edge.samples[upper];
        let span = upper.time_s - lower.time_s;
        let fraction = if span > 0.0 {
            (time_s - lower.time_s) / span
        } else {
            0.0
        };
        let translation = lower
            .transform
            .translation
            .lerp(upper.transform.translation, fraction);
        let rotation = lower
            .transform
            .rotation
            .slerp(upper.transform.rotation, fraction);
        Ok(Transform3::from_translation_rotation(translation, rotation))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::{Quat, Vec3};

    fn translation(x: f64, y: f64) -> Transform3 {
        Transform3::from_translation_rotation(Vec3::new(x, y, 0.0), Quat::IDENTITY)
    }

    #[test]
    fn static_chain_lookup_composes() {
        let mut buffer = TfBuffer::new();
        buffer
            .set_transform(StampedTransform::new(
                "map",
                "odom",
                0.0,
                translation(1.0, 0.0),
            ))
            .unwrap();
        buffer
            .set_transform(StampedTransform::new(
                "odom",
                "base_link",
                0.0,
                translation(0.0, 2.0),
            ))
            .unwrap();
        let base_in_map = buffer
            .lookup(&FrameId::new("map"), &FrameId::new("base_link"), 0.0)
            .unwrap();
        assert_relative_eq!(base_in_map.translation.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(base_in_map.translation.y, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn lookup_interpolates_between_samples() {
        let mut buffer = TfBuffer::new();
        buffer
            .set_transform(StampedTransform::new(
                "odom",
                "base_link",
                0.0,
                translation(0.0, 0.0),
            ))
            .unwrap();
        buffer
            .set_transform(StampedTransform::new(
                "odom",
                "base_link",
                2.0,
                translation(4.0, 0.0),
            ))
            .unwrap();
        let pose = buffer
            .lookup(&FrameId::new("odom"), &FrameId::new("base_link"), 1.0)
            .unwrap();
        assert_relative_eq!(pose.translation.x, 2.0, epsilon = 1e-12);
    }

    #[test]
    fn reverse_lookup_inverts() {
        let mut buffer = TfBuffer::new();
        buffer
            .set_transform(StampedTransform::new(
                "odom",
                "base_link",
                0.0,
                translation(3.0, 0.0),
            ))
            .unwrap();
        let odom_in_base = buffer
            .lookup(&FrameId::new("base_link"), &FrameId::new("odom"), 0.0)
            .unwrap();
        assert_relative_eq!(odom_in_base.translation.x, -3.0, epsilon = 1e-12);
    }

    #[test]
    fn rejects_multiple_parents_and_extrapolation() {
        let mut buffer = TfBuffer::new();
        buffer
            .set_transform(StampedTransform::new("a", "c", 0.0, translation(0.0, 0.0)))
            .unwrap();
        assert_eq!(
            buffer.set_transform(StampedTransform::new("b", "c", 0.0, translation(1.0, 0.0))),
            Err(TfError::MultipleParents(FrameId::new("c")))
        );

        buffer
            .set_transform(StampedTransform::new("a", "c", 2.0, translation(2.0, 0.0)))
            .unwrap();
        assert_eq!(
            buffer.lookup(&FrameId::new("a"), &FrameId::new("c"), 5.0),
            Err(TfError::ExtrapolationRequired {
                parent: FrameId::new("a"),
                child: FrameId::new("c"),
                time_s: 5.0,
            })
        );
    }

    #[test]
    fn unknown_frame_is_reported() {
        let buffer = TfBuffer::new();
        assert_eq!(
            buffer.lookup(&FrameId::new("map"), &FrameId::new("base_link"), 0.0),
            Err(TfError::UnknownFrame(FrameId::new("map")))
        );
    }
}
