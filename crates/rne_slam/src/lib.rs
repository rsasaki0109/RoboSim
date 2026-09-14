//! Deterministic backend-neutral 2D scan matching and online occupancy SLAM.
//!
//! `rne_slam` builds directly on `rne_nav`: it matches each [`rne_nav::LaserScan2d`]
//! against a likelihood field derived from the accumulated occupancy grid,
//! predicts from the odometry delta, and integrates the corrected scan back into
//! the map. Everything is ROS-free, renderer-free, and deterministic, so a
//! recorded scan/odometry sequence reproduces the same map and trajectory.
//!
//! Pose-graph optimization and loop closure are a later phase; this crate is the
//! online front-end.

#![deny(missing_docs)]

pub mod likelihood;
pub mod localization;
pub mod pose_graph;
pub mod resources;
pub mod scan_match;
pub mod slam;
pub mod systems;

pub use likelihood::{LikelihoodConfig, LikelihoodField};
pub use localization::{Amcl, AmclConfig, AmclUpdate, Particle};
pub use pose_graph::{PoseGraph, PoseGraphEdge, PoseGraphError};
pub use resources::{PendingSlamScans, SlamScanInput, SlamState};
pub use scan_match::{scan_points_2d, ScanMatchConfig, ScanMatchResult, ScanMatcher};
pub use slam::{closure_consistent, Slam2d, SlamConfig, SlamUpdate};
pub use systems::{slam_step, SlamStepReport};
