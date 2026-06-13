//! World entity, scene index, and frame graph for Robot Native Engine.

#![deny(missing_docs)]

pub mod components;
pub mod frame_graph;
pub mod systems;
pub mod transform;

pub use components::{spawn_world, GlobalTransform3, Gravity, Transform3, WorldEntity};
pub use frame_graph::{FrameEdge, FrameGraph, FrameId, FrameNode};
pub use systems::propagate_transforms;
pub use transform::world_transform_of;
