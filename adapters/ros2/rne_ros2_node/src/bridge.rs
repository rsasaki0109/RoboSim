//! ROS 2 bridge loop: headless sim → topics + simulation_interfaces control.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rclrs::{
    Context, CreateBasicExecutor, Executor, MandatoryParameter, Publisher, RclReturnCode,
    RclrsError, RequestedGoal, SpinOptions, Subscription, TerminatedGoal,
};
use rne_adapter_ros2::{
    pointcloud_to_laserscan, to_ros_clock, to_ros_image, to_ros_joint_state, to_ros_pointcloud2,
    to_ros_transform_stamped, RosTfMessage,
};
use rne_data::PointCloud;
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};
use rne_world::Transform3;
use simulation_interfaces::{
    action::{SimulateSteps, SimulateSteps_Feedback, SimulateSteps_Result},
    msg::SimulationState,
    srv::{
        GetSimulationState, GetSimulationState_Request, GetSimulationState_Response,
        ResetSimulation, ResetSimulation_Request, ResetSimulation_Response, SetSimulationState,
        SetSimulationState_Request, SetSimulationState_Response, StepSimulation,
        StepSimulation_Request, StepSimulation_Response,
    },
};

use crate::convert::{
    to_clock_message, to_image_message, to_joint_state_message, to_laserscan_message,
    to_pointcloud2_message, to_tf_message,
};
use crate::sim_control::{
    bridge_mode_from_env, BridgeFrame, BridgeMode, BridgeSim, BridgeSnapshot, StepFallback,
};

const SIM_STEPS: usize = 300;
const MIN_FORWARD_X_M: f64 = 0.8;
const MIN_LIDAR_HITS: usize = 8;
const MIN_MOBILE_BASE_MOTION_M: f64 = 0.15;
const MIN_MOBILE_JOINTS: usize = 4;
const MIN_WRIST_CAMERA_PIXELS: usize = 64 * 48 * 4;

type ClockPublisher = Publisher<rosgraph_msgs::msg::Clock>;
type CloudPublisher = Publisher<sensor_msgs::msg::PointCloud2>;
type ScanPublisher = Publisher<sensor_msgs::msg::LaserScan>;
type TfPublisher = Publisher<tf2_msgs::msg::TFMessage>;
type JointStatePublisher = Publisher<sensor_msgs::msg::JointState>;
type ImagePublisher = Publisher<sensor_msgs::msg::Image>;

struct BridgeLoop {
    sim: Mutex<BridgeSim>,
    clock_pub: ClockPublisher,
    cloud_pub: CloudPublisher,
    scan_pub: ScanPublisher,
    tf_pub: TfPublisher,
    joint_state_pub: JointStatePublisher,
    image_pub: Option<ImagePublisher>,
    wheel_velocity: MandatoryParameter<f64>,
    shoulder_velocity: MandatoryParameter<f64>,
    elbow_velocity: MandatoryParameter<f64>,
}

struct BridgeHandles {
    _reset: rclrs::Service<ResetSimulation>,
    _get_state: rclrs::Service<GetSimulationState>,
    _set_state: rclrs::Service<SetSimulationState>,
    _step: rclrs::Service<StepSimulation>,
    _simulate_steps: rclrs::ActionServer<SimulateSteps>,
    _cmd_vel: Option<Subscription<geometry_msgs::msg::Twist>>,
    _arm_joint_velocity: Option<Subscription<sensor_msgs::msg::JointState>>,
    _gripper_command: Option<Subscription<std_msgs::msg::Float64>>,
    _arm_joint_position: Option<Subscription<sensor_msgs::msg::JointState>>,
    _arm_joint_trajectory: Option<Subscription<trajectory_msgs::msg::JointTrajectory>>,
    _lift_command: Option<Subscription<std_msgs::msg::Float64>>,
}

impl BridgeLoop {
    fn new(
        sim: BridgeSim,
        clock_pub: ClockPublisher,
        cloud_pub: CloudPublisher,
        scan_pub: ScanPublisher,
        tf_pub: TfPublisher,
        joint_state_pub: JointStatePublisher,
        image_pub: Option<ImagePublisher>,
        wheel_velocity: MandatoryParameter<f64>,
        shoulder_velocity: MandatoryParameter<f64>,
        elbow_velocity: MandatoryParameter<f64>,
    ) -> Self {
        Self {
            sim: Mutex::new(sim),
            clock_pub,
            cloud_pub,
            scan_pub,
            tf_pub,
            joint_state_pub,
            image_pub,
            wheel_velocity,
            shoulder_velocity,
            elbow_velocity,
        }
    }

    fn mode(&self) -> BridgeMode {
        self.sim.lock().expect("bridge sim lock").mode()
    }

    fn fallback(&self) -> StepFallback {
        StepFallback {
            wheel_velocity_rad_s: self.wheel_velocity.get(),
            shoulder_velocity_rad_s: self.shoulder_velocity.get(),
            elbow_velocity_rad_s: self.elbow_velocity.get(),
        }
    }

    fn publish_current(&self) -> Result<()> {
        let sim = self.sim.lock().expect("bridge sim lock");
        publish_frame(
            &self.clock_pub,
            &self.cloud_pub,
            &self.scan_pub,
            &self.tf_pub,
            &self.joint_state_pub,
            self.image_pub.as_ref(),
            &sim.frame(),
        )
    }

    fn tick_playing(&self) -> Result<bool> {
        let mut sim = self.sim.lock().expect("bridge sim lock");
        if !sim.step_if_playing(self.fallback()) {
            return Ok(false);
        }
        let frame = sim.frame();
        drop(sim);
        publish_frame(
            &self.clock_pub,
            &self.cloud_pub,
            &self.scan_pub,
            &self.tf_pub,
            &self.joint_state_pub,
            self.image_pub.as_ref(),
            &frame,
        )?;
        Ok(true)
    }

    fn with_sim<T>(&self, f: impl FnOnce(&mut BridgeSim) -> T) -> T {
        let mut sim = self.sim.lock().expect("bridge sim lock");
        f(&mut sim)
    }
}

/// Runs the native ROS 2 bridge until the smoke-test motion check passes.
pub fn run() -> Result<()> {
    let context = Context::default_from_env().context("failed to initialize rcl context")?;
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("rne_bridge")
        .context("failed to create ROS node")?;

    let wheel_velocity = node
        .declare_parameter("wheel_velocity_rad_s")
        .default(6.0)
        .mandatory()
        .context("declare wheel_velocity_rad_s parameter")?;
    let shoulder_velocity = node
        .declare_parameter("shoulder_velocity_rad_s")
        .default(0.0)
        .mandatory()
        .context("declare shoulder_velocity_rad_s parameter")?;
    let elbow_velocity = node
        .declare_parameter("elbow_velocity_rad_s")
        .default(0.0)
        .mandatory()
        .context("declare elbow_velocity_rad_s parameter")?;

    let clock_pub = node
        .create_publisher::<rosgraph_msgs::msg::Clock>("/clock")
        .context("failed to create /clock publisher")?;
    let cloud_pub = node
        .create_publisher::<sensor_msgs::msg::PointCloud2>("/points")
        .context("failed to create /points publisher")?;
    let scan_pub = node
        .create_publisher::<sensor_msgs::msg::LaserScan>("/scan")
        .context("failed to create /scan publisher")?;
    let tf_pub = node
        .create_publisher::<tf2_msgs::msg::TFMessage>("/tf")
        .context("failed to create /tf publisher")?;
    let joint_state_pub = node
        .create_publisher::<sensor_msgs::msg::JointState>("/joint_states")
        .context("failed to create /joint_states publisher")?;

    let mode = bridge_mode_from_env();
    let image_pub = if mode == BridgeMode::MobileManipulator {
        Some(
            node.create_publisher::<sensor_msgs::msg::Image>("/camera/image_raw")
                .context("failed to create /camera/image_raw publisher")?,
        )
    } else {
        None
    };

    let bridge = Arc::new(BridgeLoop::new(
        BridgeSim::with_mode(mode),
        clock_pub,
        cloud_pub,
        scan_pub,
        tf_pub,
        joint_state_pub,
        image_pub,
        wheel_velocity,
        shoulder_velocity,
        elbow_velocity,
    ));

    let bridge_mode = bridge.mode();
    let _handles = register_services(&node, Arc::clone(&bridge))?;

    match mode {
        BridgeMode::DiffDrive => eprintln!("Driving headless diff-drive via rne_ai"),
        BridgeMode::MobileManipulator => {
            eprintln!("Driving headless mm_mobile via rne_ai (RNE_ROS2_MODE=mobile_manipulator)")
        }
    }

    let mut steps = 0_usize;
    let mut last_snapshot = bridge.with_sim(|sim| sim.snapshot());
    // Peak |base_x| over the drive: the mm_mobile base path curves, so the final
    // snapshot understates how far the command actually moved the base.
    let mut peak_abs_base_x_m = last_snapshot.base_x_m.abs();

    while steps < SIM_STEPS {
        if bridge.tick_playing()? {
            steps += 1;
            last_snapshot = bridge.with_sim(|sim| sim.snapshot());
            peak_abs_base_x_m = peak_abs_base_x_m.max(last_snapshot.base_x_m.abs());
            if steps % 60 == 0 {
                eprintln!("step {steps}: base_x={:.2} m", last_snapshot.base_x_m);
            }
        }
        spin_once(&mut executor)?;
    }

    eprintln!(
        "final base_x={:.2} m (peak |base_x|={:.2} m) joints={}",
        last_snapshot.base_x_m, peak_abs_base_x_m, last_snapshot.joint_count
    );
    verify_smoke(bridge_mode, &last_snapshot, peak_abs_base_x_m)?;

    hold_ros_graph_for_smoke(&bridge, &mut executor)?;

    Ok(())
}

fn verify_smoke(mode: BridgeMode, snapshot: &BridgeSnapshot, peak_abs_base_x_m: f64) -> Result<()> {
    match mode {
        BridgeMode::DiffDrive => {
            if snapshot.base_x_m < MIN_FORWARD_X_M {
                bail!("expected forward motion from diff-drive policy");
            }
            if snapshot.lidar_points < MIN_LIDAR_HITS {
                bail!(
                    "expected lidar hits from scene sim, got {}",
                    snapshot.lidar_points
                );
            }
            if snapshot.joint_count < 2 {
                bail!(
                    "expected wheel joints in /joint_states, got {}",
                    snapshot.joint_count
                );
            }
        }
        BridgeMode::MobileManipulator => {
            // The mm_mobile base path curves, so verify it moved at any point in the
            // drive (peak displacement) rather than from the final snapshot alone.
            if peak_abs_base_x_m < MIN_MOBILE_BASE_MOTION_M {
                bail!(
                    "expected mobile base motion, peak |base_x|={:.3} m",
                    peak_abs_base_x_m
                );
            }
            if snapshot.joint_count < MIN_MOBILE_JOINTS {
                bail!(
                    "expected {} joints in /joint_states, got {}",
                    MIN_MOBILE_JOINTS,
                    snapshot.joint_count
                );
            }
            if !snapshot.has_shoulder_joint {
                bail!("expected shoulder_joint in /joint_states");
            }
            if snapshot.wrist_camera_pixels < MIN_WRIST_CAMERA_PIXELS {
                bail!(
                    "expected wrist camera image on /camera/image_raw, got {} bytes",
                    snapshot.wrist_camera_pixels
                );
            }
        }
    }
    Ok(())
}

fn hold_ros_graph_for_smoke(bridge: &Arc<BridgeLoop>, executor: &mut Executor) -> Result<()> {
    let hold_secs = std::env::var("RNE_ROS2_HOLD_SECS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(0);
    if hold_secs == 0 {
        return Ok(());
    }

    eprintln!("holding ROS graph for {hold_secs}s (RNE_ROS2_HOLD_SECS)");
    bridge.publish_current()?;
    let deadline = std::time::Instant::now() + Duration::from_secs(hold_secs);
    while std::time::Instant::now() < deadline {
        let _ = bridge.tick_playing()?;
        bridge.publish_current()?;
        spin_once(executor)?;
    }
    Ok(())
}

fn register_services(node: &rclrs::Node, bridge: Arc<BridgeLoop>) -> Result<BridgeHandles> {
    let reset_bridge = Arc::clone(&bridge);
    let _reset = node
        .create_service::<ResetSimulation, _>(
            "/reset_simulation",
            move |request: ResetSimulation_Request| {
                let result = reset_bridge.with_sim(|sim| sim.reset(request.scope));
                ResetSimulation_Response { result }
            },
        )
        .context("create reset_simulation service")?;

    let get_bridge = Arc::clone(&bridge);
    let _get_state = node
        .create_service::<GetSimulationState, _>(
            "/get_simulation_state",
            move |_request: GetSimulationState_Request| GetSimulationState_Response {
                state: get_bridge.with_sim(|sim| sim.playback_state()),
                result: ok_result(),
            },
        )
        .context("create get_simulation_state service")?;

    let set_bridge = Arc::clone(&bridge);
    let _set_state = node
        .create_service::<SetSimulationState, _>(
            "/set_simulation_state",
            move |request: SetSimulationState_Request| {
                let result = set_bridge.with_sim(|sim| sim.set_playback(request.state.state));
                SetSimulationState_Response { result }
            },
        )
        .context("create set_simulation_state service")?;

    let step_bridge = Arc::clone(&bridge);
    let _step = node
        .create_service::<StepSimulation, _>(
            "/step_simulation",
            move |request: StepSimulation_Request| {
                let step_result = step_bridge
                    .with_sim(|sim| sim.step_while_paused(request.steps, step_bridge.fallback()));
                let publish_result = match step_result {
                    Ok(()) => step_bridge.publish_current(),
                    Err(result) => return StepSimulation_Response { result },
                };
                StepSimulation_Response {
                    result: match publish_result {
                        Ok(()) => ok_result(),
                        Err(error) => operation_failed(error.to_string()),
                    },
                }
            },
        )
        .context("create step_simulation service")?;

    let action_bridge = Arc::clone(&bridge);
    let action = node
        .create_action_server("/simulate_steps", move |handle| {
            simulate_steps_action(handle, Arc::clone(&action_bridge))
        })
        .context("create simulate_steps action server")?;

    let (
        _cmd_vel,
        _arm_joint_velocity,
        _gripper_command,
        _arm_joint_position,
        _arm_joint_trajectory,
        _lift_command,
    ) = if bridge.mode() == BridgeMode::MobileManipulator {
        register_mobile_subscribers(node, Arc::clone(&bridge))?
    } else {
        (None, None, None, None, None, None)
    };

    Ok(BridgeHandles {
        _reset,
        _get_state,
        _set_state,
        _step,
        _simulate_steps: action,
        _cmd_vel,
        _arm_joint_velocity,
        _gripper_command,
        _arm_joint_position,
        _arm_joint_trajectory,
        _lift_command,
    })
}

#[allow(clippy::type_complexity)]
fn register_mobile_subscribers(
    node: &rclrs::Node,
    bridge: Arc<BridgeLoop>,
) -> Result<(
    Option<Subscription<geometry_msgs::msg::Twist>>,
    Option<Subscription<sensor_msgs::msg::JointState>>,
    Option<Subscription<std_msgs::msg::Float64>>,
    Option<Subscription<sensor_msgs::msg::JointState>>,
    Option<Subscription<trajectory_msgs::msg::JointTrajectory>>,
    Option<Subscription<std_msgs::msg::Float64>>,
)> {
    let cmd_bridge = Arc::clone(&bridge);
    let cmd_vel = node
        .create_subscription("/cmd_vel", move |msg: geometry_msgs::msg::Twist| {
            cmd_bridge.with_sim(|sim| {
                sim.set_cmd_vel(msg.linear.x, msg.angular.z);
            });
        })
        .context("create /cmd_vel subscription")?;

    let arm_bridge = Arc::clone(&bridge);
    let arm_joint_velocity = node
        .create_subscription(
            "/arm_joint_velocity",
            move |msg: sensor_msgs::msg::JointState| {
                let (shoulder, elbow) = arm_velocities_from_joint_state(&msg);
                arm_bridge.with_sim(|sim| sim.set_arm_joint_velocities(shoulder, elbow));
            },
        )
        .context("create /arm_joint_velocity subscription")?;

    let gripper_bridge = Arc::clone(&bridge);
    let gripper_command = node
        .create_subscription("/gripper_command", move |msg: std_msgs::msg::Float64| {
            gripper_bridge.with_sim(|sim| sim.set_gripper_velocity(msg.data));
        })
        .context("create /gripper_command subscription")?;

    let lift_bridge = Arc::clone(&bridge);
    let lift_command = node
        .create_subscription("/lift_command", move |msg: std_msgs::msg::Float64| {
            lift_bridge.with_sim(|sim| sim.set_lift_velocity(msg.data));
        })
        .context("create /lift_command subscription")?;

    let position_bridge = Arc::clone(&bridge);
    let arm_joint_position = node
        .create_subscription(
            "/arm_joint_position",
            move |msg: sensor_msgs::msg::JointState| {
                if let Some((shoulder, elbow)) = arm_positions_from_joint_state(&msg) {
                    position_bridge.with_sim(|sim| sim.set_arm_joint_positions(shoulder, elbow));
                }
            },
        )
        .context("create /arm_joint_position subscription")?;

    let trajectory_bridge = Arc::clone(&bridge);
    let arm_joint_trajectory = node
        .create_subscription(
            "/arm_joint_trajectory",
            move |msg: trajectory_msgs::msg::JointTrajectory| {
                let waypoints = arm_trajectory_from_msg(&msg);
                if !waypoints.is_empty() {
                    trajectory_bridge.with_sim(|sim| sim.set_arm_trajectory(waypoints));
                }
            },
        )
        .context("create /arm_joint_trajectory subscription")?;

    Ok((
        Some(cmd_vel),
        Some(arm_joint_velocity),
        Some(gripper_command),
        Some(arm_joint_position),
        Some(arm_joint_trajectory),
        Some(lift_command),
    ))
}

fn arm_velocities_from_joint_state(msg: &sensor_msgs::msg::JointState) -> (f64, f64) {
    let mut shoulder = 0.0;
    let mut elbow = 0.0;
    for (name, velocity) in msg.name.iter().zip(msg.velocity.iter()) {
        match name.as_str() {
            "shoulder_joint" => shoulder = *velocity,
            "elbow_joint" => elbow = *velocity,
            _ => {}
        }
    }
    (shoulder, elbow)
}

/// Extracts (shoulder, elbow) position targets, if both are present in the message.
fn arm_positions_from_joint_state(msg: &sensor_msgs::msg::JointState) -> Option<(f64, f64)> {
    let mut shoulder = None;
    let mut elbow = None;
    for (name, position) in msg.name.iter().zip(msg.position.iter()) {
        match name.as_str() {
            "shoulder_joint" => shoulder = Some(*position),
            "elbow_joint" => elbow = Some(*position),
            _ => {}
        }
    }
    Some((shoulder?, elbow?))
}

/// Extracts ordered (shoulder, elbow) waypoints from a joint trajectory message.
fn arm_trajectory_from_msg(msg: &trajectory_msgs::msg::JointTrajectory) -> Vec<(f64, f64)> {
    let shoulder_idx = msg.joint_names.iter().position(|n| n == "shoulder_joint");
    let elbow_idx = msg.joint_names.iter().position(|n| n == "elbow_joint");
    let (Some(shoulder_idx), Some(elbow_idx)) = (shoulder_idx, elbow_idx) else {
        return Vec::new();
    };
    msg.points
        .iter()
        .filter_map(|point| {
            Some((
                *point.positions.get(shoulder_idx)?,
                *point.positions.get(elbow_idx)?,
            ))
        })
        .collect()
}

async fn simulate_steps_action(
    handle: RequestedGoal<SimulateSteps>,
    bridge: Arc<BridgeLoop>,
) -> TerminatedGoal {
    let steps = handle.goal().steps;
    let paused = bridge.with_sim(|sim| sim.playback() == SimulationState::STATE_PAUSED);
    if !paused {
        return handle.reject();
    }

    let executing = match handle.accept().begin() {
        rclrs::BeginAcceptedGoal::Execute(executing) => executing,
        rclrs::BeginAcceptedGoal::Cancel(cancelling) => {
            return cancelling.cancelled_with(SimulateSteps_Result {
                result: operation_failed("cancelled before execution"),
            });
        }
    };

    for completed in 1..=steps {
        let fallback = bridge.fallback();
        let step_result = bridge.with_sim(|sim| sim.step_while_paused(1, fallback));
        if let Err(result) = step_result {
            return executing.aborted_with(SimulateSteps_Result { result });
        }
        if let Err(error) = bridge.publish_current() {
            return executing.aborted_with(SimulateSteps_Result {
                result: operation_failed(error.to_string()),
            });
        }

        executing.publish_feedback(SimulateSteps_Feedback {
            completed_steps: completed,
            remaining_steps: steps.saturating_sub(completed),
        });
    }

    executing.succeeded_with(SimulateSteps_Result {
        result: ok_result(),
    })
}

fn publish_frame(
    clock_pub: &ClockPublisher,
    cloud_pub: &CloudPublisher,
    scan_pub: &ScanPublisher,
    tf_pub: &TfPublisher,
    joint_state_pub: &JointStatePublisher,
    image_pub: Option<&ImagePublisher>,
    frame: &BridgeFrame,
) -> Result<()> {
    let sim_time = rne_core::SimTime::from_ticks(frame.sim_ticks);
    let base = Transform3::from_translation_rotation(
        Vec3::new(frame.base_x_m, frame.base_y_m, frame.base_z_m),
        Quat::from_rotation_y(frame.base_yaw_rad),
    );

    let clock = to_ros_clock(sim_time);
    clock_pub
        .publish(to_clock_message(&clock))
        .context("publish /clock")?;

    let lidar_cloud = cloud_in_lidar_frame(&frame.lidar_cloud, frame.lidar_world.as_ref());
    let cloud = to_ros_pointcloud2(&lidar_cloud, sim_time, "lidar");
    cloud_pub
        .publish(to_pointcloud2_message(&cloud))
        .context("publish /points")?;

    if let (Some(lidar_world), Some(spec)) = (&frame.lidar_world, frame.lidar_spec) {
        let scan =
            pointcloud_to_laserscan(&frame.lidar_cloud, lidar_world, &spec, sim_time, "lidar");
        scan_pub
            .publish(to_laserscan_message(&scan))
            .context("publish /scan")?;
    }

    let tf = make_tf_message(base, frame.lidar_world, frame.ee_world_m, sim_time);
    tf_pub.publish(to_tf_message(&tf)).context("publish /tf")?;

    let joint_state = to_ros_joint_state(&frame.joint_state, sim_time, "base_link");
    joint_state_pub
        .publish(to_joint_state_message(&joint_state))
        .context("publish /joint_states")?;

    if let (Some(image_pub), Some(image)) = (image_pub, &frame.wrist_camera) {
        let ros_image = to_ros_image(image, sim_time, "wrist_camera");
        image_pub
            .publish(to_image_message(&ros_image))
            .context("publish /camera/image_raw")?;
    }

    Ok(())
}

fn cloud_in_lidar_frame(cloud: &PointCloud, lidar_world: Option<&Transform3>) -> PointCloud {
    let Some(lidar_world) = lidar_world else {
        return cloud.clone();
    };
    let inv = to_math_transform(lidar_world).inverse();
    PointCloud {
        points_m: cloud
            .points_m
            .iter()
            .map(|point| inv.transform_point(*point))
            .collect(),
    }
}

fn make_tf_message(
    base: Transform3,
    lidar_world: Option<Transform3>,
    ee_world_m: Option<Vec3>,
    sim_time: rne_core::SimTime,
) -> RosTfMessage {
    let lidar_relative = lidar_world
        .map(|lidar_world| {
            from_math_transform(
                to_math_transform(&base)
                    .inverse()
                    .mul_transform(&to_math_transform(&lidar_world)),
            )
        })
        .unwrap_or_else(|| {
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.2, 0.0), Quat::IDENTITY)
        });

    let mut transforms = vec![
        to_ros_transform_stamped("world", "base_link", base, sim_time),
        to_ros_transform_stamped("base_link", "lidar", lidar_relative, sim_time),
    ];

    // End-effector frame for manipulator modes, expressed relative to base_link.
    if let Some(ee_world_m) = ee_world_m {
        let ee_in_base = to_math_transform(&base)
            .inverse()
            .transform_point(ee_world_m);
        let ee_relative = Transform3::from_translation_rotation(ee_in_base, Quat::IDENTITY);
        transforms.push(to_ros_transform_stamped(
            "base_link",
            "ee_link",
            ee_relative,
            sim_time,
        ));
    }

    RosTfMessage { transforms }
}

fn to_math_transform(transform: &Transform3) -> MathTransform3 {
    MathTransform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

fn from_math_transform(transform: MathTransform3) -> Transform3 {
    Transform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

fn ok_result() -> simulation_interfaces::msg::Result {
    simulation_interfaces::msg::Result {
        result: simulation_interfaces::msg::Result::RESULT_OK,
        error_message: String::new(),
    }
}

fn operation_failed(message: impl Into<String>) -> simulation_interfaces::msg::Result {
    simulation_interfaces::msg::Result {
        result: simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
        error_message: message.into(),
    }
}

fn spin_once(executor: &mut Executor) -> Result<()> {
    match executor
        .spin(SpinOptions::spin_once().timeout(Duration::from_millis(10)))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_velocities_map_by_joint_name() {
        let msg = sensor_msgs::msg::JointState {
            name: vec!["elbow_joint".into(), "shoulder_joint".into()],
            velocity: vec![0.5, 1.5],
            ..Default::default()
        };
        let (shoulder, elbow) = arm_velocities_from_joint_state(&msg);
        assert!((shoulder - 1.5).abs() < f64::EPSILON);
        assert!((elbow - 0.5).abs() < f64::EPSILON);
    }
}
