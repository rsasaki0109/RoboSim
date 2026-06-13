//! ROS 2 bridge loop: headless diff-drive sim → topics + simulation_interfaces control.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{bail, Context as _, Result};
use rclrs::{
    Context, CreateBasicExecutor, Executor, MandatoryParameter, Publisher, RclReturnCode,
    RclrsError, RequestedGoal, SpinOptions, TerminatedGoal,
};
use rne_adapter_ros2::{
    pointcloud_to_laserscan, to_ros_clock, to_ros_pointcloud2, to_ros_transform_stamped,
    RosTfMessage,
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

use crate::convert::{to_clock_message, to_laserscan_message, to_pointcloud2_message, to_tf_message};
use crate::sim_control::{BridgeFrame, BridgeSim};

const SIM_STEPS: usize = 300;
const MIN_FORWARD_X_M: f64 = 0.8;
const MIN_LIDAR_HITS: usize = 8;

type ClockPublisher = Publisher<rosgraph_msgs::msg::Clock>;
type CloudPublisher = Publisher<sensor_msgs::msg::PointCloud2>;
type ScanPublisher = Publisher<sensor_msgs::msg::LaserScan>;
type TfPublisher = Publisher<tf2_msgs::msg::TFMessage>;

struct BridgeLoop {
    sim: Mutex<BridgeSim>,
    clock_pub: ClockPublisher,
    cloud_pub: CloudPublisher,
    scan_pub: ScanPublisher,
    tf_pub: TfPublisher,
    wheel_velocity: MandatoryParameter<f64>,
}

impl BridgeLoop {
    fn new(
        sim: BridgeSim,
        clock_pub: ClockPublisher,
        cloud_pub: CloudPublisher,
        scan_pub: ScanPublisher,
        tf_pub: TfPublisher,
        wheel_velocity: MandatoryParameter<f64>,
    ) -> Self {
        Self {
            sim: Mutex::new(sim),
            clock_pub,
            cloud_pub,
            scan_pub,
            tf_pub,
            wheel_velocity,
        }
    }

    fn wheel_velocity(&self) -> f64 {
        self.wheel_velocity.get()
    }

    fn publish_current(&self) -> Result<()> {
        let sim = self.sim.lock().expect("bridge sim lock");
        publish_frame(
            &self.clock_pub,
            &self.cloud_pub,
            &self.scan_pub,
            &self.tf_pub,
            &sim.frame(),
        )
    }

    fn tick_playing(&self) -> Result<bool> {
        let mut sim = self.sim.lock().expect("bridge sim lock");
        if !sim.step_if_playing(self.wheel_velocity()) {
            return Ok(false);
        }
        let frame = sim.frame();
        drop(sim);
        publish_frame(
            &self.clock_pub,
            &self.cloud_pub,
            &self.scan_pub,
            &self.tf_pub,
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

    let bridge = Arc::new(BridgeLoop::new(
        BridgeSim::new(),
        clock_pub,
        cloud_pub,
        scan_pub,
        tf_pub,
        wheel_velocity,
    ));

    let _handles = register_services(&node, Arc::clone(&bridge))?;

    eprintln!("Driving headless diff-drive via rne_ai");

    let mut steps = 0_usize;
    let mut last_obs = bridge.with_sim(|sim| *sim.observation());

    while steps < SIM_STEPS {
        if bridge.tick_playing()? {
            steps += 1;
            last_obs = bridge.with_sim(|sim| *sim.observation());
            if steps % 60 == 0 {
                eprintln!("step {steps}: base_x={:.2} m", last_obs.base_x_m);
            }
        }
        spin_once(&mut executor)?;
    }

    eprintln!("final base_x={:.2} m", last_obs.base_x_m);
    if last_obs.base_x_m < MIN_FORWARD_X_M {
        bail!("expected forward motion from diff-drive policy");
    }
    if last_obs.lidar_points < MIN_LIDAR_HITS {
        bail!(
            "expected lidar hits from scene sim, got {}",
            last_obs.lidar_points
        );
    }

    hold_ros_graph_for_smoke(&bridge, &mut executor)?;

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

struct BridgeHandles {
    _reset: rclrs::Service<ResetSimulation>,
    _get_state: rclrs::Service<GetSimulationState>,
    _set_state: rclrs::Service<SetSimulationState>,
    _step: rclrs::Service<StepSimulation>,
    _simulate_steps: rclrs::ActionServer<SimulateSteps>,
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
                let wheel_velocity = step_bridge.wheel_velocity();
                let step_result = step_bridge
                    .with_sim(|sim| sim.step_while_paused(request.steps, wheel_velocity));
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

    let action = node
        .create_action_server("/simulate_steps", move |handle| {
            simulate_steps_action(handle, Arc::clone(&bridge))
        })
        .context("create simulate_steps action server")?;

    Ok(BridgeHandles {
        _reset,
        _get_state,
        _set_state,
        _step,
        _simulate_steps: action,
    })
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
        let wheel_velocity = bridge.wheel_velocity();
        let step_result = bridge.with_sim(|sim| sim.step_while_paused(1, wheel_velocity));
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
    frame: &BridgeFrame,
) -> Result<()> {
    let sim_time = rne_core::SimTime::from_ticks(frame.sim_ticks);
    let obs = &frame.obs;
    let base = Transform3::from_translation_rotation(
        Vec3::new(obs.base_x_m, obs.base_y_m, obs.base_z_m),
        Quat::from_rotation_y(obs.base_yaw_rad),
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
        let scan = pointcloud_to_laserscan(&frame.lidar_cloud, lidar_world, &spec, sim_time, "lidar");
        scan_pub
            .publish(to_laserscan_message(&scan))
            .context("publish /scan")?;
    }

    let tf = make_tf_message(base, frame.lidar_world, sim_time);
    tf_pub.publish(to_tf_message(&tf)).context("publish /tf")?;

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
