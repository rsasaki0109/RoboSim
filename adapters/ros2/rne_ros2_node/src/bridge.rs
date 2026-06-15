//! ROS 2 bridge loop: headless sim → topics + simulation_interfaces control.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rclrs::{
    Context, CreateBasicExecutor, Executor, MandatoryParameter, Publisher, RclReturnCode,
    RclrsError, RequestedGoal, SpinOptions, Subscription, TerminatedGoal,
};
use rne_adapter_ros2::{
    pointcloud_to_laserscan, to_ros_clock, to_ros_joint_state, to_ros_pointcloud2,
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
    to_clock_message, to_joint_state_message, to_laserscan_message, to_pointcloud2_message,
    to_tf_message,
};
use crate::sim_control::{BridgeFrame, BridgeMode, BridgeSim, BridgeSnapshot, StepFallback};

const SIM_STEPS: usize = 300;
const MIN_FORWARD_X_M: f64 = 0.8;
const MIN_LIDAR_HITS: usize = 8;
const MIN_MOBILE_BASE_MOTION_M: f64 = 0.15;
const MIN_MOBILE_JOINTS: usize = 4;

type ClockPublisher = Publisher<rosgraph_msgs::msg::Clock>;
type CloudPublisher = Publisher<sensor_msgs::msg::PointCloud2>;
type ScanPublisher = Publisher<sensor_msgs::msg::LaserScan>;
type TfPublisher = Publisher<tf2_msgs::msg::TFMessage>;
type JointStatePublisher = Publisher<sensor_msgs::msg::JointState>;

struct BridgeLoop {
    sim: Mutex<BridgeSim>,
    clock_pub: ClockPublisher,
    cloud_pub: CloudPublisher,
    scan_pub: ScanPublisher,
    tf_pub: TfPublisher,
    joint_state_pub: JointStatePublisher,
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
}

impl BridgeLoop {
    fn new(
        sim: BridgeSim,
        clock_pub: ClockPublisher,
        cloud_pub: CloudPublisher,
        scan_pub: ScanPublisher,
        tf_pub: TfPublisher,
        joint_state_pub: JointStatePublisher,
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

    let bridge = Arc::new(BridgeLoop::new(
        BridgeSim::new(),
        clock_pub,
        cloud_pub,
        scan_pub,
        tf_pub,
        joint_state_pub,
        wheel_velocity,
        shoulder_velocity,
        elbow_velocity,
    ));

    let mode = bridge.mode();
    let _handles = register_services(&node, Arc::clone(&bridge))?;

    match mode {
        BridgeMode::DiffDrive => eprintln!("Driving headless diff-drive via rne_ai"),
        BridgeMode::MobileManipulator => {
            eprintln!("Driving headless mm_mobile via rne_ai (RNE_ROS2_MODE=mobile_manipulator)")
        }
    }

    let mut steps = 0_usize;
    let mut last_snapshot = bridge.with_sim(|sim| sim.snapshot());

    while steps < SIM_STEPS {
        if bridge.tick_playing()? {
            steps += 1;
            last_snapshot = bridge.with_sim(|sim| sim.snapshot());
            if steps % 60 == 0 {
                eprintln!("step {steps}: base_x={:.2} m", last_snapshot.base_x_m);
            }
        }
        spin_once(&mut executor)?;
    }

    eprintln!(
        "final base_x={:.2} m joints={}",
        last_snapshot.base_x_m, last_snapshot.joint_count
    );
    verify_smoke(mode, &last_snapshot)?;

    hold_ros_graph_for_smoke(&bridge, &mut executor)?;

    Ok(())
}

fn verify_smoke(mode: BridgeMode, snapshot: &BridgeSnapshot) -> Result<()> {
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
            if snapshot.base_x_m.abs() < MIN_MOBILE_BASE_MOTION_M {
                bail!(
                    "expected mobile base motion, |base_x|={:.3} m",
                    snapshot.base_x_m
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
    let deadline = std::time::Instant::now() + Duration::from_secs(hold_secs);
    while std::time::Instant::now() < deadline {
        let _ = bridge.tick_playing()?;
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

    let (_cmd_vel, _arm_joint_velocity) = if bridge.mode() == BridgeMode::MobileManipulator {
        register_mobile_subscribers(node, Arc::clone(&bridge))?
    } else {
        (None, None)
    };

    Ok(BridgeHandles {
        _reset,
        _get_state,
        _set_state,
        _step,
        _simulate_steps: action,
        _cmd_vel,
        _arm_joint_velocity,
    })
}

fn register_mobile_subscribers(
    node: &rclrs::Node,
    bridge: Arc<BridgeLoop>,
) -> Result<(
    Option<Subscription<geometry_msgs::msg::Twist>>,
    Option<Subscription<sensor_msgs::msg::JointState>>,
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

    Ok((Some(cmd_vel), Some(arm_joint_velocity)))
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

    let tf = make_tf_message(base, frame.lidar_world, sim_time);
    tf_pub.publish(to_tf_message(&tf)).context("publish /tf")?;

    let joint_state = to_ros_joint_state(&frame.joint_state, sim_time, "base_link");
    joint_state_pub
        .publish(to_joint_state_message(&joint_state))
        .context("publish /joint_states")?;

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

    RosTfMessage {
        transforms: vec![
            to_ros_transform_stamped("world", "base_link", base, sim_time),
            to_ros_transform_stamped("base_link", "lidar", lidar_relative, sim_time),
        ],
    }
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
