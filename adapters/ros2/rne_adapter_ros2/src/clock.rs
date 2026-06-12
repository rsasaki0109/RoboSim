//! `/clock` mapping helpers.

use crate::messages::{RosClock, RosTime};
use rne_core::SimTime;

/// Converts RNE simulation time to a ROS `Clock` message.
pub fn to_ros_clock(sim_time: SimTime) -> RosClock {
    RosClock {
        clock: to_ros_time(sim_time),
    }
}

/// Converts RNE simulation time to a ROS time stamp.
pub fn to_ros_time(sim_time: SimTime) -> RosTime {
    let total_ns = sim_time.ticks();
    RosTime {
        sec: (total_ns / 1_000_000_000) as i32,
        nanosec: (total_ns % 1_000_000_000) as u32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_maps_nanosecond_ticks() {
        let clock = to_ros_clock(SimTime::from_ticks(1_500_000_000));
        assert_eq!(clock.clock.sec, 1);
        assert_eq!(clock.clock.nanosec, 500_000_000);
    }
}
