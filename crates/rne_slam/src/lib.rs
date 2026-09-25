//! Deterministic backend-neutral 2D scan matching and 3D LiDAR-inertial SLAM.
//!
//! `rne_slam` builds directly on `rne_nav`: it matches each [`rne_nav::LaserScan2d`]
//! against a likelihood field derived from the accumulated occupancy grid,
//! predicts from the odometry delta, and integrates the corrected scan back into
//! the map. The 3D side adds SE(3) utilities, IMU preintegration, and a 6-DoF
//! pose graph with Gauss-Newton optimization on the manifold. Everything is
//! ROS-free, renderer-free, and deterministic, so a recorded sequence reproduces
//! the same map and trajectory.

#![deny(missing_docs)]

pub mod floor_reacquire;
pub mod graph_io;
pub mod icp;
pub mod imu_preintegration;
pub mod likelihood;
pub mod lio;
pub mod lio_ekf;
pub mod lio_iekf;
pub mod lio_inertial;
pub mod localization;
pub mod odometry3d;
pub mod point_to_plane;
pub mod pose_graph;
pub mod pose_graph3d;
pub mod relocalize;
pub mod resources;
pub mod scan_match;
pub mod se3;
pub mod session;
pub mod slam;
pub mod slam3d;
pub mod systems;

pub use floor_reacquire::{
    reacquire_floor, FloorAmbiguity, FloorCandidate, FloorHypothesis, FloorIdentification,
    ReacquisitionConfig, ReacquisitionError,
};
pub use graph_io::{
    combine_graphs, from_graph_json, load_graph, save_graph, to_graph_json, GraphIoError,
    RNE_POSE_GRAPH_FORMAT, RNE_POSE_GRAPH_VERSION,
};
pub use icp::{Icp3d, IcpConfig, IcpError, IcpResult};
pub use imu_preintegration::{
    ImuBias, ImuPreintegrationError, ImuPreintegrator, ImuSample, PreintegratedDelta,
};
pub use likelihood::{LikelihoodConfig, LikelihoodField};
pub use lio::{LioConfig, LioError, LioOdometry, LioUpdate};
pub use lio_ekf::{LioEkf, LioEkfConfig, LioEkfError, LioEkfUpdate};
pub use lio_iekf::{LioIekf, LioIekfConfig, LioIekfError, LioIekfUpdate};
pub use lio_inertial::{LioInertialConfig, LioInertialEkf, LioInertialError, LioInertialUpdate};
pub use localization::{Amcl, AmclConfig, AmclUpdate, Particle};
pub use odometry3d::{voxel_downsample, IcpOdometry, IcpOdometryConfig, IcpOdometryUpdate};
pub use point_to_plane::{
    estimate_normals, IcpPointToPlane, PointToPlaneConfig, PointToPlaneError, PointToPlaneResult,
    VoxelPointIndex,
};
pub use pose_graph::{PoseGraph, PoseGraphEdge, PoseGraphError};
pub use pose_graph3d::{PoseGraph3d, PoseGraph3dEdge, PoseGraph3dError, POSE3D_DIM};
pub use relocalize::{
    GlobalRelocalizer, RelocalizationConfig, RelocalizationError, RelocalizationResult,
};
pub use resources::{PendingSlamScans, SlamScanInput, SlamState};
pub use scan_match::{scan_points_2d, ScanMatchConfig, ScanMatchResult, ScanMatcher};
pub use se3::{
    mat3_add, mat3_identity, mat3_mul, mat3_scale, mat3_vec, skew, so3_exp, so3_left_jacobian,
    so3_left_jacobian_inverse, so3_log, Se3,
};
pub use session::{
    discover_session_constraints, DiscoveryConfig, DiscoveryError, LifelongPoseGraph, MergeOptions,
    SessionConstraint, SessionError, SessionId, SessionRecognition, SessionScan,
};
pub use slam::{closure_consistent, Slam2d, SlamConfig, SlamUpdate};
pub use slam3d::{
    pose2d_to_transform3, pose3_to_pose2d, Slam3d, Slam3dConfig, Slam3dUpdate,
    SlamError as Slam3dError,
};
pub use systems::{slam_step, SlamStepReport};
