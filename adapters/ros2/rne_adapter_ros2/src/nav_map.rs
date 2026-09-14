//! `nav_msgs/OccupancyGrid` and `nav_msgs/Path` mapping helpers.

use crate::messages::{RosHeader, RosMapMetaData, RosOccupancyGrid, RosPath, RosPoseStamped};
use crate::nav_odometry::to_ros_pose;
use rne_core::SimTime;
use rne_nav::{GridCoord, OccupancyGrid, Path2d};

/// Converts an occupancy grid into a `nav_msgs/OccupancyGrid` shape.
///
/// Cells use the ROS convention: `-1` unknown, `0` free, `100` occupied. Data
/// is row-major with x fastest, matching `OccupancyGrid`'s own layout. The
/// origin is the pose of cell `(0, 0)`'s center in the map frame.
pub fn to_ros_occupancy_grid(
    grid: &OccupancyGrid,
    sim_time: SimTime,
    frame_id: &str,
) -> RosOccupancyGrid {
    let width = grid.width();
    let height = grid.height();
    let mut data = Vec::with_capacity(width * height);
    for y in 0..height {
        for x in 0..width {
            data.push(grid.cell_value(GridCoord {
                x: x as isize,
                y: y as isize,
            }));
        }
    }

    RosOccupancyGrid {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        info: RosMapMetaData {
            map_load_time: crate::clock::to_ros_time(sim_time),
            resolution: grid.resolution_m() as f32,
            width: width as u32,
            height: height as u32,
            origin: to_ros_pose(grid.origin()),
        },
        data,
    }
}

/// Converts a planar path into a `nav_msgs/Path` shape.
pub fn to_ros_path(path: &Path2d, sim_time: SimTime, frame_id: &str) -> RosPath {
    let header = RosHeader {
        stamp: crate::clock::to_ros_time(sim_time),
        frame_id: frame_id.to_string(),
    };
    let poses = path
        .waypoints()
        .iter()
        .map(|waypoint| RosPoseStamped {
            header: header.clone(),
            pose: to_ros_pose(*waypoint),
        })
        .collect();
    RosPath { header, poses }
}

/// Converts a single planar pose into a `geometry_msgs/PoseStamped` shape.
pub fn to_ros_pose_stamped(
    pose: rne_nav::Pose2d,
    sim_time: SimTime,
    frame_id: &str,
) -> RosPoseStamped {
    RosPoseStamped {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        pose: to_ros_pose(pose),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::Pose2d;

    #[test]
    fn occupancy_grid_maps_metadata_and_values() {
        let mut grid = OccupancyGrid::new(4, 3, 0.5, Pose2d::new(-1.0, -1.0, 0.0)).unwrap();
        grid.mark_occupied(GridCoord { x: 1, y: 1 });
        let message = to_ros_occupancy_grid(&grid, SimTime::from_ticks(5), "map");
        assert_eq!(message.header.frame_id, "map");
        assert_eq!(message.info.width, 4);
        assert_eq!(message.info.height, 3);
        assert_eq!(message.info.resolution, 0.5);
        assert_eq!(message.info.origin.position.x, -1.0);
        assert_eq!(message.data.len(), 12);
        let (occupied_x, occupied_y, width) = (1_usize, 1_usize, message.info.width as usize);
        assert_eq!(message.data[occupied_y * width + occupied_x], 100);
        assert_eq!(message.data[0], -1);
    }

    #[test]
    fn path_maps_every_waypoint() {
        let path = Path2d::from_points(&[
            rne_math::Vec3::new(0.0, 0.0, 0.0),
            rne_math::Vec3::new(1.0, 0.0, 0.0),
            rne_math::Vec3::new(1.0, 1.0, 0.0),
        ]);
        let message = to_ros_path(&path, SimTime::ZERO, "map");
        assert_eq!(message.poses.len(), 3);
        assert_eq!(message.poses[2].pose.position.y, 1.0);
    }
}
