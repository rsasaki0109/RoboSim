//! Backend-neutral navigation foundations for Robot Native Engine.
//!
//! This crate is the ROS-free core that a SLAM or Nav2-style stack builds on:
//!
//! * [`OccupancyGrid`] — a deterministic 2D log-odds grid with world/grid
//!   projection and grid ray casting.
//! * [`Costmap`] — a planar cost surface derived from an occupancy grid with
//!   lethal, inscribed, and inflated costs.
//! * [`TfBuffer`] — a timestamped transform tree with shortest-path lookup and
//!   linear/slerp time interpolation for map → odom → base_link → sensor chains.
//! * [`LaserScan2d`] — a renderer/physics-independent 2D scan payload and its
//!   occupancy integration.
//!
//! The ECS glue lives in [`resources`] (a [`NavMap`] and [`TfTree`]) and
//! [`systems`]. Core crates must stay ROS-free; a ROS 2 adapter maps these
//! types to `nav_msgs`, `sensor_msgs`, and `tf2` without changing them.

#![deny(missing_docs)]

pub mod avoidance;
pub mod behavior_tree;
pub mod components;
pub mod control;
pub mod costmap;
pub mod drive;
pub mod dwa;
pub mod elevation;
pub mod fusion;
pub mod grid;
pub mod path;
pub mod planner;
pub mod points;
pub mod pose2d;
pub mod recovery;
pub mod resources;
pub mod scan;
pub mod systems;
pub mod tf;
pub mod tile;

pub use avoidance::{avoid_velocities, rollout_collides, AvoidanceConfig, CircularObstacle};
pub use behavior_tree::{Action, BtContext, BtNode, BtStatus, Condition, Selector, Sequence};
pub use components::NavGoal;
pub use control::{pure_pursuit_follow, FollowResult, PurePursuitConfig, VelocityCommand2d};
pub use costmap::{
    Costmap, CostmapConfig, COST_FREE, COST_INSCRIBED, COST_LETHAL, COST_NO_INFORMATION,
};
pub use drive::{
    AckermannDrive, DifferentialDrive, DriveActuation, DriveError, DriveFault, DriveKind,
    DriveLimits, DriveOutput, MecanumDrive, MobileBase, WheelSpeeds,
};
pub use dwa::{DwaConfig, DwaError, DwaOutcome, DwaPlanner};
pub use elevation::{ElevationCell, ElevationConfig, ElevationMap, ElevationReport};
pub use fusion::{wrap_angle, EkfConfig, EkfFusion, FusionError};
pub use grid::{GridCoord, GridError, OccupancyGrid};
pub use path::{ClosestPoint, Path2d, PathError};
pub use planner::{plan_path, GlobalPlannerConfig, PlanError};
pub use points::integrate_point_cloud;
pub use pose2d::Pose2d;
pub use recovery::{
    clear_costmap_around, RecoveryAction, RecoveryBehavior, RecoveryError, RecoveryOutcome,
    RecoverySequence, RecoveryStatus, CLEAR_LOG_ODDS,
};
pub use resources::{NavMap, PendingScan, PendingScans, TfTree};
pub use scan::{integrate_scan, LaserScan2d, ScanIntegrationConfig, ScanIntegrationReport};
pub use systems::integrate_pending_scans;
pub use tf::{FrameId, StampedTransform, TfBuffer, TfError};
pub use tile::TiledOccupancyGrid;
