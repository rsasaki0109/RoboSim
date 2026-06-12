//! ROS 2 bridge loop: headless diff-drive sim → `/clock`, `/points`, `/tf`.

use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rclrs::{Context, CreateBasicExecutor, Executor, Publisher, RclReturnCode, RclrsError, SpinOptions};
use rne_adapter_ros2::{
    to_ros_clock, to_ros_pointcloud2, to_ros_transform_stamped, RosTfMessage,
};
use rne_ai::{DiffDriveObservation, DiffDriveSim};
use rne_core::SimTime;
use rne_data::PointCloud;
use rne_math::{Quat, Vec3};
use rne_world::Transform3;

use crate::convert::{to_clock_message, to_pointcloud2_message, to_tf_message};

const SIM_DT_NS: u64 = 1_000_000_000 / 60;
const SIM_STEPS: usize = 180;

type ClockPublisher = Publisher<rosgraph_msgs::msg::Clock>;
type CloudPublisher = Publisher<sensor_msgs::msg::PointCloud2>;
type TfPublisher = Publisher<tf2_msgs::msg::TFMessage>;

/// Runs the native ROS 2 bridge until the smoke-test motion check passes.
pub fn run() -> Result<()> {
    let context = Context::default_from_env().context("failed to initialize rcl context")?;
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("rne_bridge")
        .context("failed to create ROS node")?;

    let clock_pub = node
        .create_publisher::<rosgraph_msgs::msg::Clock>("/clock")
        .context("failed to create /clock publisher")?;
    let cloud_pub = node
        .create_publisher::<sensor_msgs::msg::PointCloud2>("/points")
        .context("failed to create /points publisher")?;
    let tf_pub = node
        .create_publisher::<tf2_msgs::msg::TFMessage>("/tf")
        .context("failed to create /tf publisher")?;

    let mut sim = DiffDriveSim::new();
    let mut obs = sim.reset();
    let mut sim_ticks = 0_u64;

    eprintln!("Driving headless diff-drive via rne_ai");

    for step in 0..SIM_STEPS {
        obs = sim.step(6.0, 6.0);
        publish_frame(
            &clock_pub,
            &cloud_pub,
            &tf_pub,
            sim_ticks,
            &obs,
        )
        .with_context(|| format!("failed to publish frame at step {step}"))?;

        sim_ticks = sim_ticks.saturating_add(SIM_DT_NS);
        spin_once(&mut executor)?;

        if step % 60 == 59 {
            eprintln!("step {}: base_x={:.2} m", step + 1, obs.base_x_m);
        }
    }

    eprintln!("final base_x={:.2} m", obs.base_x_m);
    if obs.base_x_m < 1.0 {
        bail!("expected forward motion from diff-drive policy");
    }

    Ok(())
}

fn publish_frame(
    clock_pub: &ClockPublisher,
    cloud_pub: &CloudPublisher,
    tf_pub: &TfPublisher,
    sim_ticks: u64,
    obs: &DiffDriveObservation,
) -> Result<()> {
    let sim_time = SimTime::from_ticks(sim_ticks);
    let base = Vec3::new(obs.base_x_m, obs.base_y_m, obs.base_z_m);
    let distance = base.x.max(0.1) as f32;
    let points = vec![
        (distance, 0.0, 0.0),
        (distance, 0.5, 0.0),
        (distance, -0.5, 0.0),
    ];

    let clock = to_ros_clock(sim_time);
    clock_pub
        .publish(to_clock_message(&clock))
        .context("publish /clock")?;

    let cloud = to_ros_pointcloud2(
        &PointCloud {
            points_m: points
                .iter()
                .map(|(x, y, z)| Vec3::new(*x as f64, *y as f64, *z as f64))
                .collect(),
        },
        sim_time,
        "lidar",
    );
    cloud_pub
        .publish(to_pointcloud2_message(&cloud))
        .context("publish /points")?;

    let tf = make_tf_message(base, sim_time);
    tf_pub
        .publish(to_tf_message(&tf))
        .context("publish /tf")?;

    Ok(())
}

fn make_tf_message(base: Vec3, sim_time: SimTime) -> RosTfMessage {
    RosTfMessage {
        transforms: vec![
            to_ros_transform_stamped(
                "world",
                "base_link",
                Transform3::from_translation_rotation(base, Quat::IDENTITY),
                sim_time,
            ),
            to_ros_transform_stamped(
                "base_link",
                "lidar",
                Transform3::from_translation_rotation(Vec3::new(0.0, 0.2, 0.0), Quat::IDENTITY),
                sim_time,
            ),
        ],
    }
}

fn spin_once(executor: &mut Executor) -> Result<()> {
    match executor
        .spin(
            SpinOptions::spin_once().timeout(Duration::from_millis(10)),
        )
        .as_slice()
    {
        [] => Ok(()),
        [RclrsError::RclError {
            code: RclReturnCode::Timeout,
            ..
        }] => Ok(()),
        [error, ..] => Err(anyhow::anyhow!("executor spin_once failed: {error:?}")),
    }
}
