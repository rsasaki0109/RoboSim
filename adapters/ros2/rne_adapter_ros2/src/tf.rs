//! TF mapping helpers.

use crate::messages::{
    RosHeader, RosQuaternion, RosTfMessage, RosTransform, RosTransformStamped, RosVector3,
};
use rne_core::SimTime;
use rne_math::{Mat4, Quat};
use rne_world::{FrameGraph, Transform3};

/// Converts a frame graph into a ROS TF message.
pub fn to_ros_tf_message(frame_graph: &FrameGraph, sim_time: SimTime) -> RosTfMessage {
    let transforms = frame_graph
        .edges()
        .into_iter()
        .map(|edge| to_ros_transform_stamped(&edge.parent, &edge.child, edge.transform, sim_time))
        .collect();

    RosTfMessage { transforms }
}

/// Converts a single fixed transform into `TransformStamped`.
pub fn to_ros_transform_stamped(
    parent_frame: &str,
    child_frame: &str,
    transform: Transform3,
    sim_time: SimTime,
) -> RosTransformStamped {
    RosTransformStamped {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: parent_frame.to_string(),
        },
        child_frame_id: child_frame.to_string(),
        transform: to_ros_transform(transform),
    }
}

/// Converts an RNE transform to a ROS transform message.
pub fn to_ros_transform(transform: Transform3) -> RosTransform {
    RosTransform {
        translation: RosVector3 {
            x: transform.translation.x,
            y: transform.translation.y,
            z: transform.translation.z,
        },
        rotation: to_ros_quaternion(transform.rotation),
    }
}

/// Converts a global matrix lookup into a ROS transform message.
pub fn to_ros_transform_from_matrix(matrix: Mat4) -> RosTransform {
    let (_scale, rotation, translation) = matrix.to_scale_rotation_translation();
    RosTransform {
        translation: RosVector3 {
            x: translation.x,
            y: translation.y,
            z: translation.z,
        },
        rotation: to_ros_quaternion(rotation),
    }
}

fn to_ros_quaternion(rotation: Quat) -> RosQuaternion {
    RosQuaternion {
        x: rotation.x,
        y: rotation.y,
        z: rotation.z,
        w: rotation.w,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Vec3;
    use rne_world::FrameId;

    #[test]
    fn tf_message_contains_frame_graph_edges() {
        let mut graph = FrameGraph::new();
        graph.add_frame(FrameId(1), "base_link");
        graph.add_frame(FrameId(2), "lidar");
        graph
            .set_transform(
                "world",
                "base_link",
                Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
            )
            .unwrap();
        graph
            .set_transform(
                "base_link",
                "lidar",
                Transform3::from_translation_rotation(Vec3::new(0.0, 0.2, 0.0), Quat::IDENTITY),
            )
            .unwrap();

        let tf = to_ros_tf_message(&graph, SimTime::from_ticks(10));
        assert_eq!(tf.transforms.len(), 2);
        assert!(tf
            .transforms
            .iter()
            .any(|transform| transform.child_frame_id == "lidar"));
    }

    #[test]
    fn transform_maps_translation() {
        let transform = to_ros_transform(Transform3::from_translation_rotation(
            Vec3::new(0.0, 1.5, -0.25),
            Quat::IDENTITY,
        ));
        assert_relative_eq!(transform.translation.y, 1.5);
    }
}
