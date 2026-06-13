//! ROS2 `sensor_msgs/JointState` mapping.

use rne_core::SimTime;
use rne_data::JointState;

use crate::clock::to_ros_time;
use crate::messages::{RosHeader, RosJointState};

/// Maps a DataBus [`JointState`] payload to `sensor_msgs/JointState`.
pub fn to_ros_joint_state(state: &JointState, sim_time: SimTime, frame_id: &str) -> RosJointState {
    RosJointState {
        header: RosHeader {
            stamp: to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        names: state.names.clone(),
        positions: state.positions_rad.clone(),
        velocities: state.velocities_rad_s.clone(),
        efforts: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_state_preserves_names_and_positions() {
        let state = JointState {
            names: vec!["shoulder_joint".into(), "elbow_joint".into()],
            positions_rad: vec![0.1, -0.2],
            velocities_rad_s: vec![1.0, 0.5],
        };
        let ros = to_ros_joint_state(&state, SimTime::from_ticks(1_000_000_000), "base_link");
        assert_eq!(ros.names.len(), 2);
        assert_eq!(ros.positions[0], 0.1);
        assert_eq!(ros.velocities[1], 0.5);
        assert_eq!(ros.header.frame_id, "base_link");
    }
}
