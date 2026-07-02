//! Headless bridge state for simulation_interfaces control.

use std::path::PathBuf;

use rne_ai::{
    DiffDriveObservation, DiffDriveSim, MobileManipulatorAction, MobileManipulatorObservation,
    MobileManipulatorSim, MmLiftJointTarget,
};
use rne_data::{ImageRgb8, JointState, PointCloud};
use rne_math::Vec3;
use rne_sensor::LidarSpec;
use rne_world::Transform3;
use simulation_interfaces::{
    msg::{Result as SimResult, SimulationState},
    srv::ResetSimulation_Request,
};

use crate::cmd_vel::mobile_action_from_twist_and_arm;

pub const SIM_DT_NS: u64 = 1_000_000_000 / 60;

/// Fallback joint velocities when no ROS command is active.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepFallback {
    /// Wheel angular velocity used when no `/cmd_vel` command is present.
    pub wheel_velocity_rad_s: f64,
    /// Shoulder velocity used when no arm command is present.
    pub shoulder_velocity_rad_s: f64,
    /// Elbow velocity used when no arm command is present.
    pub elbow_velocity_rad_s: f64,
}

/// Backend selected for the ROS bridge loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BridgeMode {
    /// Scene-based diff-drive robot with LiDAR (`mesh_diff_drive` by default).
    DiffDrive,
    /// Built-in diff-drive base + 2-DOF arm (`mm_mobile`).
    MobileManipulator,
    /// Fixed column + lift + 2-DOF arm (`mm_lift`).
    MmLift,
}

impl BridgeMode {
    /// Returns true when the mode exposes manipulator arm ROS subscriptions.
    pub fn has_manipulator_subscribers(self) -> bool {
        matches!(self, Self::MobileManipulator | Self::MmLift)
    }
}

/// Snapshot of simulation outputs published to ROS topics each frame.
#[derive(Clone, Debug)]
pub struct BridgeFrame {
    /// Simulation clock in nanosecond ticks.
    pub sim_ticks: u64,
    /// Base link X position in meters.
    pub base_x_m: f64,
    /// Base link Y position in meters.
    pub base_y_m: f64,
    /// Base link Z position in meters.
    pub base_z_m: f64,
    /// Base link yaw around world Y in radians.
    pub base_yaw_rad: f64,
    /// Latest LiDAR point count when available.
    pub lidar_points: usize,
    /// Latest LiDAR point cloud in world coordinates.
    pub lidar_cloud: PointCloud,
    /// World-space LiDAR mount transform when present.
    pub lidar_world: Option<Transform3>,
    /// LiDAR sensor specification when present.
    pub lidar_spec: Option<LidarSpec>,
    /// Latest joint state for `/joint_states`.
    pub joint_state: JointState,
    /// Latest wrist camera frame when configured.
    pub wrist_camera: Option<ImageRgb8>,
    /// World-frame end-effector position for the arm TF frame (manipulator only).
    pub ee_world_m: Option<Vec3>,
}

/// Lightweight observation for smoke checks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BridgeSnapshot {
    /// Base link X position in meters.
    pub base_x_m: f64,
    /// Latest LiDAR point count.
    pub lidar_points: usize,
    /// Number of joints in the latest `/joint_states` message.
    pub joint_count: usize,
    /// Whether `shoulder_joint` is present in joint names.
    pub has_shoulder_joint: bool,
    /// Whether `lift_joint` is present in joint names.
    pub has_lift_joint: bool,
    /// RGBA8 byte count in the latest wrist camera frame.
    pub wrist_camera_pixels: usize,
}

enum SimBackend {
    DiffDrive {
        sim: DiffDriveSim,
        obs: DiffDriveObservation,
    },
    MobileManipulator {
        sim: MobileManipulatorSim,
        obs: MobileManipulatorObservation,
        command: MobileManipulatorAction,
        /// Active (shoulder, elbow) position target driven by P-control.
        arm_target: Option<(f64, f64)>,
        /// Remaining (shoulder, elbow) trajectory waypoints to visit after `arm_target`.
        arm_trajectory: std::collections::VecDeque<(f64, f64)>,
        /// Active lift + arm joint target driven by direct position motors.
        lift_arm_target: Option<MmLiftJointTarget>,
        /// Remaining lift-arm trajectory waypoints after `lift_arm_target`.
        lift_arm_trajectory: std::collections::VecDeque<MmLiftJointTarget>,
    },
}

/// Proportional gain and velocity clamp for arm joint position control.
const ARM_POSITION_GAIN: f64 = 8.0;
const ARM_POSITION_MAX_VELOCITY_RAD_S: f64 = 4.0;
/// Joint-angle tolerance (rad) for considering a trajectory waypoint reached.
const ARM_WAYPOINT_TOLERANCE_RAD: f64 = 0.05;
/// Lift displacement tolerance (m) for lift-arm trajectory waypoints.
const LIFT_WAYPOINT_TOLERANCE_M: f64 = 0.04;

/// Returns the P-control velocity (rad/s) driving `current` toward `target`.
fn arm_velocity_toward(target: f64, current: f64) -> f64 {
    (ARM_POSITION_GAIN * (target - current)).clamp(
        -ARM_POSITION_MAX_VELOCITY_RAD_S,
        ARM_POSITION_MAX_VELOCITY_RAD_S,
    )
}

fn lift_arm_waypoint_reached(target: MmLiftJointTarget, obs: &MobileManipulatorObservation) -> bool {
    (target.lift_m - obs.lift_position_m).abs() < LIFT_WAYPOINT_TOLERANCE_M
        && (target.shoulder_rad - obs.shoulder_position_rad).abs() < ARM_WAYPOINT_TOLERANCE_RAD
        && (target.elbow_rad - obs.elbow_position_rad).abs() < ARM_WAYPOINT_TOLERANCE_RAD
}

/// Shared simulation playback and reset state for the ROS bridge loop.
pub struct BridgeSim {
    mode: BridgeMode,
    backend: SimBackend,
    sim_ticks: u64,
    playback: u8,
}

impl BridgeSim {
    /// Creates a bridge simulation in the playing state.
    pub fn new() -> Self {
        Self::with_mode(bridge_mode_from_env())
    }

    /// Creates a bridge simulation for an explicit backend mode.
    pub fn with_mode(mode: BridgeMode) -> Self {
        let backend = match mode {
            BridgeMode::DiffDrive => {
                let scene_path = default_ros2_scene_path();
                let mut sim = DiffDriveSim::from_scene_path(&scene_path).unwrap_or_else(|err| {
                    panic!("load ROS 2 scene {}: {err}", scene_path.display())
                });
                let obs = sim.reset();
                SimBackend::DiffDrive { sim, obs }
            }
            BridgeMode::MobileManipulator => {
                let scene_path = default_mobile_manipulator_scene_path();
                let mut sim =
                    MobileManipulatorSim::from_scene_path(&scene_path).unwrap_or_else(|err| {
                        panic!(
                            "load ROS 2 mobile manipulator scene {}: {err}",
                            scene_path.display()
                        )
                    });
                let obs = sim.reset();
                SimBackend::MobileManipulator {
                    sim,
                    obs,
                    command: MobileManipulatorAction::default(),
                    arm_target: None,
                    arm_trajectory: std::collections::VecDeque::new(),
                    lift_arm_target: None,
                    lift_arm_trajectory: std::collections::VecDeque::new(),
                }
            }
            BridgeMode::MmLift => {
                let scene_path = default_mm_lift_scene_path();
                let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).unwrap_or_else(
                    |err| {
                        panic!(
                            "load ROS 2 mm_lift scene {}: {err}",
                            scene_path.display()
                        )
                    },
                );
                let obs = sim.reset();
                SimBackend::MobileManipulator {
                    sim,
                    obs,
                    command: MobileManipulatorAction::default(),
                    arm_target: None,
                    arm_trajectory: std::collections::VecDeque::new(),
                    lift_arm_target: None,
                    lift_arm_trajectory: std::collections::VecDeque::new(),
                }
            }
        };

        Self {
            mode,
            backend,
            sim_ticks: 0,
            playback: SimulationState::STATE_PLAYING,
        }
    }

    /// Active backend mode.
    pub fn mode(&self) -> BridgeMode {
        self.mode
    }

    /// Latest values for smoke checks.
    pub fn snapshot(&self) -> BridgeSnapshot {
        let frame = self.frame();
        BridgeSnapshot {
            base_x_m: frame.base_x_m,
            lidar_points: frame.lidar_points,
            joint_count: frame.joint_state.names.len(),
            has_shoulder_joint: frame
                .joint_state
                .names
                .iter()
                .any(|name| name == "shoulder_joint"),
            has_lift_joint: frame
                .joint_state
                .names
                .iter()
                .any(|name| name == "lift_joint"),
            wrist_camera_pixels: frame
                .wrist_camera
                .as_ref()
                .map(|image| image.rgba8.len())
                .unwrap_or(0),
        }
    }

    /// Current simulation clock in nanosecond ticks.
    pub fn sim_ticks(&self) -> u64 {
        self.sim_ticks
    }

    /// Current playback state (`SimulationState::STATE_*`).
    pub fn playback(&self) -> u8 {
        self.playback
    }

    /// Collects the latest values to publish on ROS topics.
    pub fn frame(&self) -> BridgeFrame {
        match &self.backend {
            SimBackend::DiffDrive { sim, obs } => BridgeFrame {
                sim_ticks: self.sim_ticks,
                base_x_m: obs.base_x_m,
                base_y_m: obs.base_y_m,
                base_z_m: obs.base_z_m,
                base_yaw_rad: obs.base_yaw_rad,
                lidar_points: obs.lidar_points,
                lidar_cloud: sim.latest_lidar_cloud().unwrap_or_else(PointCloud::new),
                lidar_world: sim.primary_lidar_world_transform(),
                lidar_spec: sim.primary_lidar_spec(),
                ee_world_m: None,
                joint_state: sim.joint_state(),
                wrist_camera: None,
            },
            SimBackend::MobileManipulator { sim, obs, .. } => BridgeFrame {
                sim_ticks: self.sim_ticks,
                base_x_m: obs.base_x_m,
                base_y_m: obs.base_y_m,
                base_z_m: obs.base_z_m,
                base_yaw_rad: obs.base_yaw_rad,
                lidar_points: 0,
                lidar_cloud: PointCloud::new(),
                lidar_world: None,
                lidar_spec: None,
                ee_world_m: Some(Vec3::new(obs.ee_x_m, obs.ee_y_m, obs.ee_z_m)),
                joint_state: sim.latest_joint_state(),
                wrist_camera: sim.latest_wrist_camera(),
            },
        }
    }

    /// Applies a geometry twist command (mobile manipulator mode only).
    pub fn set_cmd_vel(&mut self, linear_x_m_s: f64, angular_z_rad_s: f64) {
        let SimBackend::MobileManipulator { command, .. } = &mut self.backend else {
            return;
        };
        let shoulder = command.shoulder_velocity_rad_s;
        let elbow = command.elbow_velocity_rad_s;
        let gripper = command.gripper_velocity_rad_s;
        *command = mobile_action_from_twist_and_arm(linear_x_m_s, angular_z_rad_s, shoulder, elbow);
        command.gripper_velocity_rad_s = gripper;
    }

    /// Applies a gripper velocity command (mobile manipulator mode only).
    ///
    /// Negative closes the gripper (and triggers a contact weld grasp); positive opens
    /// it and releases any grasped object.
    pub fn set_gripper_velocity(&mut self, gripper_velocity_rad_s: f64) {
        let SimBackend::MobileManipulator { command, .. } = &mut self.backend else {
            return;
        };
        command.gripper_velocity_rad_s = gripper_velocity_rad_s;
    }

    /// Applies a vertical lift velocity command (mobile manipulator mode only).
    ///
    /// Positive raises the lift, negative lowers it. Only the lift-equipped robot acts on
    /// this; robots without a lift joint ignore it.
    pub fn set_lift_velocity(&mut self, lift_velocity_m_s: f64) {
        let SimBackend::MobileManipulator {
            command,
            lift_arm_target,
            lift_arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        command.lift_velocity_m_s = lift_velocity_m_s;
        *lift_arm_target = None;
        lift_arm_trajectory.clear();
    }

    fn clear_revolute_arm_targets(backend: &mut SimBackend) {
        let SimBackend::MobileManipulator {
            arm_target,
            arm_trajectory,
            ..
        } = backend
        else {
            return;
        };
        *arm_target = None;
        arm_trajectory.clear();
    }

    fn clear_lift_arm_targets(backend: &mut SimBackend) {
        let SimBackend::MobileManipulator {
            lift_arm_target,
            lift_arm_trajectory,
            ..
        } = backend
        else {
            return;
        };
        *lift_arm_target = None;
        lift_arm_trajectory.clear();
    }

    /// Applies arm joint velocity targets by joint name (mobile manipulator mode only).
    ///
    /// A velocity command cancels any active position target.
    pub fn set_arm_joint_velocities(
        &mut self,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
    ) {
        let SimBackend::MobileManipulator {
            command,
            arm_target,
            arm_trajectory,
            lift_arm_target,
            lift_arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        command.shoulder_velocity_rad_s = shoulder_velocity_rad_s;
        command.elbow_velocity_rad_s = elbow_velocity_rad_s;
        *arm_target = None;
        arm_trajectory.clear();
        *lift_arm_target = None;
        lift_arm_trajectory.clear();
    }

    /// Sets (shoulder, elbow) joint position targets driven by P-control until reached
    /// (mobile manipulator mode only).
    pub fn set_arm_joint_positions(&mut self, shoulder_position_rad: f64, elbow_position_rad: f64) {
        let SimBackend::MobileManipulator {
            arm_target,
            arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        Self::clear_lift_arm_targets(&mut self.backend);
        *arm_target = Some((shoulder_position_rad, elbow_position_rad));
        arm_trajectory.clear();
    }

    /// Sets lift + shoulder + elbow joint position targets on the `mm_lift` robot.
    pub fn set_lift_arm_joint_positions(&mut self, target: MmLiftJointTarget) {
        let SimBackend::MobileManipulator {
            lift_arm_target,
            lift_arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        Self::clear_revolute_arm_targets(&mut self.backend);
        *lift_arm_target = Some(target);
        lift_arm_trajectory.clear();
    }

    /// Queues a sequence of (shoulder, elbow) waypoints, visited in order via P-control
    /// (mobile manipulator mode only).
    pub fn set_arm_trajectory(&mut self, waypoints: Vec<(f64, f64)>) {
        let SimBackend::MobileManipulator {
            arm_target,
            arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        Self::clear_lift_arm_targets(&mut self.backend);
        let mut queue: std::collections::VecDeque<(f64, f64)> = waypoints.into();
        *arm_target = queue.pop_front();
        *arm_trajectory = queue;
    }

    /// Queues lift + shoulder + elbow waypoints visited via direct position motors.
    pub fn set_lift_arm_trajectory(&mut self, waypoints: Vec<MmLiftJointTarget>) {
        let SimBackend::MobileManipulator {
            lift_arm_target,
            lift_arm_trajectory,
            ..
        } = &mut self.backend
        else {
            return;
        };
        Self::clear_revolute_arm_targets(&mut self.backend);
        let mut queue: std::collections::VecDeque<MmLiftJointTarget> = waypoints.into();
        *lift_arm_target = queue.pop_front();
        *lift_arm_trajectory = queue;
    }

    /// Resets simulation scope per `simulation_interfaces/ResetSimulation`.
    pub fn reset(&mut self, scope: u8) -> SimResult {
        let scope = normalize_reset_scope(scope);
        if scope & ResetSimulation_Request::SCOPE_TIME != 0 {
            self.sim_ticks = 0;
        }
        if scope & ResetSimulation_Request::SCOPE_STATE != 0 {
            match &mut self.backend {
                SimBackend::DiffDrive { sim, obs } => *obs = sim.reset(),
                SimBackend::MobileManipulator {
                    sim,
                    obs,
                    command,
                    arm_target,
                    arm_trajectory,
                    lift_arm_target,
                    lift_arm_trajectory,
                } => {
                    *obs = sim.reset();
                    *command = MobileManipulatorAction::default();
                    *arm_target = None;
                    arm_trajectory.clear();
                    *lift_arm_target = None;
                    lift_arm_trajectory.clear();
                }
            }
        }
        ok_result()
    }

    /// Sets playback state per `simulation_interfaces/SetSimulationState`.
    pub fn set_playback(&mut self, target: u8) -> SimResult {
        if target == self.playback {
            return result_code(
                simulation_interfaces::srv::SetSimulationState_Response::ALREADY_IN_TARGET_STATE,
                String::new(),
            );
        }

        match target {
            SimulationState::STATE_PLAYING => {
                if self.playback == SimulationState::STATE_STOPPED {
                    self.reset(ResetSimulation_Request::SCOPE_ALL);
                }
                self.playback = SimulationState::STATE_PLAYING;
                ok_result()
            }
            SimulationState::STATE_PAUSED => {
                if self.playback == SimulationState::STATE_STOPPED {
                    return result_code(
                        simulation_interfaces::srv::SetSimulationState_Response::INCORRECT_TRANSITION,
                        "cannot pause while simulation is stopped".into(),
                    );
                }
                self.playback = SimulationState::STATE_PAUSED;
                ok_result()
            }
            SimulationState::STATE_STOPPED => {
                self.playback = SimulationState::STATE_STOPPED;
                self.reset(ResetSimulation_Request::SCOPE_ALL)
            }
            _ => fail_operation("unsupported simulation state"),
        }
    }

    /// Returns the current playback state message.
    pub fn playback_state(&self) -> SimulationState {
        SimulationState {
            state: self.playback,
        }
    }

    /// Advances one tick when playback is active.
    pub fn step_if_playing(&mut self, fallback: StepFallback) -> bool {
        if self.playback != SimulationState::STATE_PLAYING {
            return false;
        }
        self.step_once(fallback);
        true
    }

    /// Steps the simulation while paused, as required by step/action interfaces.
    pub fn step_while_paused(
        &mut self,
        steps: u64,
        fallback: StepFallback,
    ) -> Result<(), SimResult> {
        if self.playback != SimulationState::STATE_PAUSED {
            return Err(incorrect_state("stepping requires paused simulation"));
        }
        for _ in 0..steps {
            self.step_once(fallback);
        }
        Ok(())
    }

    fn step_once(&mut self, fallback: StepFallback) {
        match &mut self.backend {
            SimBackend::DiffDrive { sim, obs } => {
                *obs = sim.step(fallback.wheel_velocity_rad_s, fallback.wheel_velocity_rad_s);
            }
            SimBackend::MobileManipulator {
                sim,
                obs,
                command,
                arm_target,
                arm_trajectory,
                lift_arm_target,
                lift_arm_trajectory,
            } => {
                let mut action = *command;
                if action.left_wheel_velocity_rad_s.abs() < f64::EPSILON
                    && action.right_wheel_velocity_rad_s.abs() < f64::EPSILON
                {
                    action.left_wheel_velocity_rad_s = fallback.wheel_velocity_rad_s;
                    action.right_wheel_velocity_rad_s = fallback.wheel_velocity_rad_s;
                }

                if let Some(target) = *lift_arm_target {
                    action = MobileManipulatorAction::hold_lift_joints(target);
                    action.left_wheel_velocity_rad_s = command.left_wheel_velocity_rad_s;
                    action.right_wheel_velocity_rad_s = command.right_wheel_velocity_rad_s;
                    action.gripper_velocity_rad_s = command.gripper_velocity_rad_s;
                    *obs = sim.step(action);

                    if lift_arm_waypoint_reached(*target, obs) {
                        if let Some(next) = lift_arm_trajectory.pop_front() {
                            *lift_arm_target = Some(next);
                        }
                    }
                } else {
                    if let Some((shoulder_target, elbow_target)) = *arm_target {
                        // Position control: drive the arm joints toward their targets.
                        action.shoulder_velocity_rad_s =
                            arm_velocity_toward(shoulder_target, obs.shoulder_position_rad);
                        action.elbow_velocity_rad_s =
                            arm_velocity_toward(elbow_target, obs.elbow_position_rad);
                    } else if action.shoulder_velocity_rad_s.abs() < f64::EPSILON
                        && action.elbow_velocity_rad_s.abs() < f64::EPSILON
                    {
                        action.shoulder_velocity_rad_s = fallback.shoulder_velocity_rad_s;
                        action.elbow_velocity_rad_s = fallback.elbow_velocity_rad_s;
                    }
                    *obs = sim.step(action);

                    // Advance to the next trajectory waypoint once the current one is reached.
                    if let Some((shoulder_target, elbow_target)) = *arm_target {
                        let reached = (shoulder_target - obs.shoulder_position_rad).abs()
                            < ARM_WAYPOINT_TOLERANCE_RAD
                            && (elbow_target - obs.elbow_position_rad).abs()
                                < ARM_WAYPOINT_TOLERANCE_RAD;
                        if reached {
                            if let Some(next) = arm_trajectory.pop_front() {
                                *arm_target = Some(next);
                            }
                        }
                    }
                }
            }
        }
        self.sim_ticks = self.sim_ticks.saturating_add(SIM_DT_NS);
    }
}

pub fn bridge_mode_from_env() -> BridgeMode {
    match std::env::var("RNE_ROS2_MODE")
        .ok()
        .unwrap_or_default()
        .to_ascii_lowercase()
        .as_str()
    {
        "mobile_manipulator" | "mm_mobile" | "mobile" => BridgeMode::MobileManipulator,
        "mm_lift" | "lift" | "manipulator_lift" => BridgeMode::MmLift,
        _ => BridgeMode::DiffDrive,
    }
}

fn default_mm_lift_scene_path() -> PathBuf {
    if let Ok(path) = std::env::var("RNE_ROS2_SCENE_PATH") {
        return PathBuf::from(path);
    }
    rne_ai::mm_lift_scene_path()
}

fn default_ros2_scene_path() -> PathBuf {
    if let Ok(path) = std::env::var("RNE_ROS2_SCENE_PATH") {
        return PathBuf::from(path);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../assets/scenes/mesh_diff_drive.rne.scene.toml")
}

fn default_mobile_manipulator_scene_path() -> PathBuf {
    if let Ok(path) = std::env::var("RNE_ROS2_SCENE_PATH") {
        return PathBuf::from(path);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../assets/scenes/mm_mobile.rne.scene.toml")
}

fn normalize_reset_scope(scope: u8) -> u8 {
    if scope == ResetSimulation_Request::SCOPE_DEFAULT {
        ResetSimulation_Request::SCOPE_ALL
    } else {
        scope
    }
}

fn ok_result() -> SimResult {
    result_code(SimResult::RESULT_OK, String::new())
}

fn incorrect_state(message: impl Into<String>) -> SimResult {
    result_code(SimResult::RESULT_INCORRECT_STATE, message.into())
}

fn fail_operation(message: impl Into<String>) -> SimResult {
    result_code(SimResult::RESULT_OPERATION_FAILED, message.into())
}

fn result_code(code: u8, message: String) -> SimResult {
    SimResult {
        result: code,
        error_message: message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_velocity_toward_drives_to_target_and_clamps() {
        // Drives in the correct direction.
        assert!(arm_velocity_toward(1.0, 0.0) > 0.0);
        assert!(arm_velocity_toward(-1.0, 0.0) < 0.0);
        // Near the target the command shrinks toward zero.
        assert!(arm_velocity_toward(0.01, 0.0).abs() < ARM_POSITION_MAX_VELOCITY_RAD_S);
        // Large errors clamp to the velocity limit.
        assert_eq!(
            arm_velocity_toward(100.0, 0.0),
            ARM_POSITION_MAX_VELOCITY_RAD_S
        );
        assert_eq!(
            arm_velocity_toward(-100.0, 0.0),
            -ARM_POSITION_MAX_VELOCITY_RAD_S
        );
    }

    #[test]
    fn reset_scope_all_restarts_pose_and_time() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::DiffDrive);
        bridge.step_if_playing(StepFallback {
            wheel_velocity_rad_s: 6.0,
            ..StepFallback::default()
        });
        assert!(bridge.sim_ticks() > 0);
        assert!(bridge.snapshot().base_x_m > 0.0);

        bridge.reset(ResetSimulation_Request::SCOPE_ALL);
        assert_eq!(bridge.sim_ticks(), 0);
        assert!(bridge.snapshot().base_x_m.abs() < 0.01);
    }

    #[test]
    fn mobile_manipulator_publishes_four_joints() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::MobileManipulator);
        bridge.step_if_playing(StepFallback {
            wheel_velocity_rad_s: 6.0,
            ..StepFallback::default()
        });
        let frame = bridge.frame();
        assert_eq!(frame.joint_state.names.len(), 4);
        assert!(frame
            .joint_state
            .names
            .iter()
            .any(|name| name == "shoulder_joint"));
    }

    #[test]
    fn mm_lift_publishes_lift_and_arm_joints() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::MmLift);
        bridge.step_if_playing(StepFallback::default());
        let snapshot = bridge.snapshot();
        assert!(snapshot.has_lift_joint);
        assert!(snapshot.has_shoulder_joint);
        assert!(snapshot.joint_count >= 6);
    }

    #[test]
    fn lift_arm_trajectory_drives_lift_joint() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::MmLift);
        bridge.step_if_playing(StepFallback::default());
        let start_lift = joint_position(&bridge.frame().joint_state, "lift_joint");
        bridge.set_lift_arm_trajectory(vec![MmLiftJointTarget {
            lift_m: start_lift + 0.12,
            shoulder_rad: 0.15,
            elbow_rad: 0.10,
        }]);
        for _ in 0..480 {
            bridge.step_if_playing(StepFallback::default());
        }
        let end_lift = joint_position(&bridge.frame().joint_state, "lift_joint");
        assert!(
            (end_lift - (start_lift + 0.12)).abs() < 0.06,
            "lift should track trajectory target, start={start_lift:.3}, end={end_lift:.3}"
        );
    }

    fn joint_position(joint_state: &JointState, joint_name: &str) -> f64 {
        joint_state
            .names
            .iter()
            .position(|name| name == joint_name)
            .and_then(|index| joint_state.positions_rad.get(index).copied())
            .unwrap_or(0.0)
    }

    #[test]
    fn mobile_cmd_vel_updates_base_motion() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::MobileManipulator);
        bridge.set_cmd_vel(0.5, 0.0);
        for _ in 0..120 {
            bridge.step_if_playing(StepFallback::default());
        }
        assert!(
            bridge.snapshot().base_x_m.abs() > 0.05,
            "expected base motion from /cmd_vel equivalent"
        );
    }

    #[test]
    fn paused_stepping_advances_pose() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::DiffDrive);
        bridge.set_playback(SimulationState::STATE_PAUSED);
        bridge
            .step_while_paused(
                30,
                StepFallback {
                    wheel_velocity_rad_s: 6.0,
                    ..StepFallback::default()
                },
            )
            .expect("paused stepping should succeed");
        assert!(bridge.snapshot().base_x_m > 0.05);
    }

    #[test]
    fn step_while_playing_rejects_when_not_paused() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::DiffDrive);
        let err = bridge
            .step_while_paused(1, StepFallback::default())
            .expect_err("playing sim should reject paused stepping");
        assert_eq!(err.result, SimResult::RESULT_INCORRECT_STATE);
    }

    #[test]
    fn scene_includes_lidar_hits_after_steps() {
        let mut bridge = BridgeSim::with_mode(BridgeMode::DiffDrive);
        for _ in 0..60 {
            bridge.step_if_playing(StepFallback {
                wheel_velocity_rad_s: 6.0,
                ..StepFallback::default()
            });
        }
        assert!(
            bridge.snapshot().lidar_points >= 8,
            "expected lidar hits from scene asset, got {}",
            bridge.snapshot().lidar_points
        );
    }
}
