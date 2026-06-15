//! ROS2 `sensor_msgs/Image` mapping.

use rne_core::SimTime;
use rne_data::ImageRgb8;

use crate::clock::to_ros_time;
use crate::messages::{RosHeader, RosImage};

/// Maps a DataBus [`ImageRgb8`] payload to `sensor_msgs/Image`.
pub fn to_ros_image(image: &ImageRgb8, sim_time: SimTime, frame_id: &str) -> RosImage {
    let width = image.width;
    let height = image.height;
    let step = 4 * width;
    RosImage {
        header: RosHeader {
            stamp: to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height,
        width,
        encoding: "rgba8".to_string(),
        is_bigendian: false,
        step,
        data: image.rgba8.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn image_preserves_dimensions_and_payload() {
        let image = ImageRgb8::from_rgba8(8, 6, vec![1; 8 * 6 * 4]);
        let ros = to_ros_image(&image, SimTime::from_ticks(1_000_000_000), "wrist_camera");
        assert_eq!(ros.width, 8);
        assert_eq!(ros.height, 6);
        assert_eq!(ros.step, 32);
        assert_eq!(ros.encoding, "rgba8");
        assert_eq!(ros.data.len(), 8 * 6 * 4);
        assert_eq!(ros.header.frame_id, "wrist_camera");
    }
}
