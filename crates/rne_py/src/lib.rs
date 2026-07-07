//! Python bindings for Robot Native Engine.

mod sim;

use pyo3::prelude::*;
use rne_ai::{DiffDriveEpisodeConfig, Episode, IkClutterPickPlacePolicy, Policy};
use sim::{
    DiffDriveObservation, DiffDriveSim, MmLiftGripperTarget, MmLiftIkError, MmLiftJointTarget,
    MmLiftKinematics, MobileManipulatorAction, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, MobileManipulatorObservation, MobileManipulatorSim,
    VectorizedMobileManipulatorConfig, VectorizedMobileManipulatorEnv,
};
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Write};
use std::path::{Path, PathBuf};

const CHECKPOINT_TEMP_CREATE_ATTEMPTS: u32 = 64;

/// Resolves a task name to a mobile manipulator episode configuration.
fn mm_episode_config(task: &str) -> PyResult<MobileManipulatorEpisodeConfig> {
    match task {
        "reach" => Ok(MobileManipulatorEpisodeConfig::reach()),
        "reach_random" => Ok(MobileManipulatorEpisodeConfig::reach_randomized(0)),
        "reach_curriculum" => Ok(MobileManipulatorEpisodeConfig::reach_curriculum(0)),
        "place" => Ok(MobileManipulatorEpisodeConfig::place()),
        "lift_place" => Ok(MobileManipulatorEpisodeConfig::lift_pick_place()),
        "clutter_place" => Ok(MobileManipulatorEpisodeConfig::clutter_pick_place(0)),
        "clutter_place_center" => Ok(MobileManipulatorEpisodeConfig::clutter_pick_place_center(0)),
        "mobile_clutter_place_center" => {
            Ok(MobileManipulatorEpisodeConfig::mobile_clutter_pick_place_center(0))
        }
        "mobile_clutter_place" => Ok(MobileManipulatorEpisodeConfig::mobile_clutter_pick_place(0)),
        "transport" => Ok(MobileManipulatorEpisodeConfig::transport()),
        "inspect" => Ok(MobileManipulatorEpisodeConfig::inspect()),
        other => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "unknown task '{other}', expected 'reach', 'reach_random', 'reach_curriculum', 'place', 'lift_place', 'clutter_place', 'clutter_place_center', 'mobile_clutter_place', 'mobile_clutter_place_center', 'transport', or 'inspect'"
        ))),
    }
}

fn checkpoint_temp_path(path: &Path, attempt: u32) -> PyResult<PathBuf> {
    let file_name = path.file_name().ok_or_else(|| {
        pyo3::exceptions::PyOSError::new_err(format!(
            "checkpoint path '{}' has no file name",
            path.display()
        ))
    })?;
    let mut tmp_file_name = file_name.to_os_string();
    tmp_file_name.push(format!(".{}.{attempt}.tmp", std::process::id()));
    Ok(path.with_file_name(tmp_file_name))
}

fn create_checkpoint_temp_file(path: &Path) -> PyResult<(PathBuf, File)> {
    for attempt in 0..CHECKPOINT_TEMP_CREATE_ATTEMPTS {
        let tmp_path = checkpoint_temp_path(path, attempt)?;
        match OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&tmp_path)
        {
            Ok(file) => return Ok((tmp_path, file)),
            Err(error) if error.kind() == ErrorKind::AlreadyExists => continue,
            Err(error) => {
                return Err(pyo3::exceptions::PyOSError::new_err(format!(
                    "failed to create checkpoint temp file '{}': {error}",
                    tmp_path.display()
                )));
            }
        }
    }

    Err(pyo3::exceptions::PyOSError::new_err(format!(
        "failed to create a unique checkpoint temp file for '{}' after {} attempts",
        path.display(),
        CHECKPOINT_TEMP_CREATE_ATTEMPTS
    )))
}

fn atomic_write_checkpoint(path: &Path, content: &str) -> PyResult<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|error| {
                pyo3::exceptions::PyOSError::new_err(format!(
                    "failed to create checkpoint directory '{}': {error}",
                    parent.display()
                ))
            })?;
        }
    }

    let (tmp_path, mut file) = create_checkpoint_temp_file(path)?;
    file.write_all(content.as_bytes()).map_err(|error| {
        pyo3::exceptions::PyOSError::new_err(format!(
            "failed to write checkpoint temp file '{}': {error}",
            tmp_path.display()
        ))
    })?;
    file.write_all(b"\n").map_err(|error| {
        pyo3::exceptions::PyOSError::new_err(format!(
            "failed to finish checkpoint temp file '{}': {error}",
            tmp_path.display()
        ))
    })?;
    file.sync_all().map_err(|error| {
        pyo3::exceptions::PyOSError::new_err(format!(
            "failed to sync checkpoint temp file '{}': {error}",
            tmp_path.display()
        ))
    })?;
    drop(file);

    std::fs::rename(&tmp_path, path).map_err(|error| {
        let _ = std::fs::remove_file(&tmp_path);
        pyo3::exceptions::PyOSError::new_err(format!(
            "failed to move checkpoint temp file '{}' to '{}': {error}",
            tmp_path.display(),
            path.display()
        ))
    })
}

/// Observation returned after each simulation step.
#[pyclass(name = "Observation")]
#[derive(Clone, Copy)]
struct PyObservation {
    base_x_m: f64,
    base_y_m: f64,
    base_z_m: f64,
    base_yaw_rad: f64,
    left_wheel_velocity_rad_s: f64,
    right_wheel_velocity_rad_s: f64,
    imu_ay_m_s2: f64,
    lidar_points: usize,
    goal_delta_x_m: Option<f64>,
    peer_delta_x_m: Option<f64>,
    peer_delta_z_m: Option<f64>,
    peer_separation_m: Option<f64>,
}

#[pymethods]
impl PyObservation {
    #[getter]
    fn base_x(&self) -> f64 {
        self.base_x_m
    }

    #[getter]
    fn base_y(&self) -> f64 {
        self.base_y_m
    }

    #[getter]
    fn base_z(&self) -> f64 {
        self.base_z_m
    }

    #[getter]
    fn base_yaw(&self) -> f64 {
        self.base_yaw_rad
    }

    #[getter]
    fn left_wheel_velocity(&self) -> f64 {
        self.left_wheel_velocity_rad_s
    }

    #[getter]
    fn right_wheel_velocity(&self) -> f64 {
        self.right_wheel_velocity_rad_s
    }

    #[getter]
    fn imu_ay(&self) -> f64 {
        self.imu_ay_m_s2
    }

    #[getter]
    fn lidar_points(&self) -> usize {
        self.lidar_points
    }

    #[getter]
    fn goal_delta_x(&self) -> Option<f64> {
        self.goal_delta_x_m
    }

    #[getter]
    fn peer_delta_x(&self) -> Option<f64> {
        self.peer_delta_x_m
    }

    #[getter]
    fn peer_delta_z(&self) -> Option<f64> {
        self.peer_delta_z_m
    }

    #[getter]
    fn peer_separation(&self) -> Option<f64> {
        self.peer_separation_m
    }

    fn __repr__(&self) -> String {
        format!(
            "Observation(base_x={:.3}, base_y={:.3}, yaw={:.3}, imu_ay={:.3})",
            self.base_x_m, self.base_y_m, self.base_yaw_rad, self.imu_ay_m_s2
        )
    }
}

impl From<DiffDriveObservation> for PyObservation {
    fn from(value: DiffDriveObservation) -> Self {
        Self {
            base_x_m: value.base_x_m,
            base_y_m: value.base_y_m,
            base_z_m: value.base_z_m,
            base_yaw_rad: value.base_yaw_rad,
            left_wheel_velocity_rad_s: value.left_wheel_velocity_rad_s,
            right_wheel_velocity_rad_s: value.right_wheel_velocity_rad_s,
            imu_ay_m_s2: value.imu_ay_m_s2,
            lidar_points: value.lidar_points,
            goal_delta_x_m: value.goal_delta_x_m,
            peer_delta_x_m: value.peer_delta_x_m,
            peer_delta_z_m: value.peer_delta_z_m,
            peer_separation_m: value.peer_separation_m,
        }
    }
}

/// Result of an episode reset or step.
#[pyclass(name = "StepResult")]
#[derive(Clone, Copy)]
struct PyStepResult {
    observation: PyObservation,
    reward: f64,
    terminated: bool,
    truncated: bool,
}

#[pymethods]
impl PyStepResult {
    #[getter]
    fn observation(&self) -> PyObservation {
        self.observation
    }

    #[getter]
    fn reward(&self) -> f64 {
        self.reward
    }

    #[getter]
    fn terminated(&self) -> bool {
        self.terminated
    }

    #[getter]
    fn truncated(&self) -> bool {
        self.truncated
    }

    #[getter]
    fn done(&self) -> bool {
        self.terminated || self.truncated
    }

    fn __repr__(&self) -> String {
        format!(
            "StepResult(reward={:.3}, terminated={}, truncated={})",
            self.reward, self.terminated, self.truncated
        )
    }
}

impl From<rne_ai::EpisodeStep<DiffDriveObservation>> for PyStepResult {
    fn from(value: rne_ai::EpisodeStep<DiffDriveObservation>) -> Self {
        Self {
            observation: value.observation.into(),
            reward: value.reward,
            terminated: value.terminated,
            truncated: value.truncated,
        }
    }
}

/// Headless differential drive simulation exposed to Python.
#[pyclass(name = "DiffDriveSim")]
struct PyDiffDriveSim {
    inner: DiffDriveSim,
}

#[pymethods]
impl PyDiffDriveSim {
    #[new]
    fn new() -> Self {
        Self {
            inner: DiffDriveSim::new(),
        }
    }

    fn reset(&mut self) -> PyObservation {
        self.inner.reset().into()
    }

    fn step(&mut self, left_velocity_rad_s: f64, right_velocity_rad_s: f64) -> PyObservation {
        self.inner
            .step(left_velocity_rad_s, right_velocity_rad_s)
            .into()
    }

    #[getter]
    fn step_count(&self) -> u64 {
        self.inner.step_count()
    }
}

/// Goal-reaching differential drive episode with reward and termination.
#[pyclass(name = "DiffDriveEpisode")]
struct PyDiffDriveEpisode {
    inner: sim::DiffDriveEpisode,
}

#[pymethods]
impl PyDiffDriveEpisode {
    #[new]
    #[pyo3(signature = (goal_x_m=2.0, max_steps=300))]
    fn new(goal_x_m: f64, max_steps: u64) -> Self {
        Self {
            inner: sim::DiffDriveEpisode::new(DiffDriveEpisodeConfig {
                goal_x_m,
                max_steps,
                ..DiffDriveEpisodeConfig::default()
            }),
        }
    }

    fn reset(&mut self) -> PyStepResult {
        self.inner.reset().into()
    }

    fn step(&mut self, left_velocity_rad_s: f64, right_velocity_rad_s: f64) -> PyStepResult {
        self.inner
            .step(sim::DiffDriveAction {
                left_velocity_rad_s,
                right_velocity_rad_s,
            })
            .into()
    }

    #[getter]
    fn step_in_episode(&self) -> u64 {
        self.inner.step_in_episode()
    }

    #[getter]
    fn total_reward(&self) -> f64 {
        self.inner.total_reward()
    }
}

fn ik_error_to_py(error: MmLiftIkError) -> PyErr {
    PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("{error:?}"))
}

/// Joint-space target for the `mm_lift` lift + shoulder + elbow chain.
#[pyclass(name = "MmLiftJointTarget")]
#[derive(Clone, Copy)]
struct PyMmLiftJointTarget {
    lift_m: f64,
    shoulder_rad: f64,
    elbow_rad: f64,
}

#[pymethods]
impl PyMmLiftJointTarget {
    /// Creates a joint target in simulation motor units.
    #[new]
    fn new(lift_m: f64, shoulder_rad: f64, elbow_rad: f64) -> Self {
        Self {
            lift_m,
            shoulder_rad,
            elbow_rad,
        }
    }

    #[getter]
    fn lift_m(&self) -> f64 {
        self.lift_m
    }

    #[getter]
    fn shoulder_rad(&self) -> f64 {
        self.shoulder_rad
    }

    #[getter]
    fn elbow_rad(&self) -> f64 {
        self.elbow_rad
    }

    fn __repr__(&self) -> String {
        format!(
            "MmLiftJointTarget(lift_m={:.3}, shoulder_rad={:.3}, elbow_rad={:.3})",
            self.lift_m, self.shoulder_rad, self.elbow_rad
        )
    }
}

impl From<MmLiftJointTarget> for PyMmLiftJointTarget {
    fn from(value: MmLiftJointTarget) -> Self {
        Self {
            lift_m: value.lift_m,
            shoulder_rad: value.shoulder_rad,
            elbow_rad: value.elbow_rad,
        }
    }
}

impl From<PyMmLiftJointTarget> for MmLiftJointTarget {
    fn from(value: PyMmLiftJointTarget) -> Self {
        Self {
            lift_m: value.lift_m,
            shoulder_rad: value.shoulder_rad,
            elbow_rad: value.elbow_rad,
        }
    }
}

/// World-frame gripper-base target for `mm_lift` analytic IK.
#[pyclass(name = "MmLiftGripperTarget")]
#[derive(Clone, Copy)]
struct PyMmLiftGripperTarget {
    x_m: f64,
    y_m: f64,
    z_m: f64,
}

#[pymethods]
impl PyMmLiftGripperTarget {
    /// Creates a world-frame gripper-base target in meters.
    #[new]
    fn new(x_m: f64, y_m: f64, z_m: f64) -> Self {
        Self { x_m, y_m, z_m }
    }

    #[getter]
    fn x_m(&self) -> f64 {
        self.x_m
    }

    #[getter]
    fn y_m(&self) -> f64 {
        self.y_m
    }

    #[getter]
    fn z_m(&self) -> f64 {
        self.z_m
    }

    fn __repr__(&self) -> String {
        format!(
            "MmLiftGripperTarget(x_m={:.3}, y_m={:.3}, z_m={:.3})",
            self.x_m, self.y_m, self.z_m
        )
    }
}

impl From<MmLiftGripperTarget> for PyMmLiftGripperTarget {
    fn from(value: MmLiftGripperTarget) -> Self {
        Self {
            x_m: value.x_m,
            y_m: value.y_m,
            z_m: value.z_m,
        }
    }
}

impl From<PyMmLiftGripperTarget> for MmLiftGripperTarget {
    fn from(value: PyMmLiftGripperTarget) -> Self {
        Self {
            x_m: value.x_m,
            y_m: value.y_m,
            z_m: value.z_m,
        }
    }
}

/// Analytic forward / inverse kinematics for the `mm_lift` robot.
#[pyclass(name = "MmLiftKinematics")]
struct PyMmLiftKinematics {
    inner: MmLiftKinematics,
}

#[pymethods]
impl PyMmLiftKinematics {
    /// Returns geometry for the shipped `mm_lift` asset.
    #[staticmethod]
    fn mm_lift() -> Self {
        Self {
            inner: MmLiftKinematics::mm_lift(),
        }
    }

    /// Computes the world-frame gripper-base pose from joint targets.
    fn forward_kinematics(&self, joints: PyMmLiftJointTarget) -> PyMmLiftGripperTarget {
        self.inner.forward_kinematics(joints.into()).into()
    }

    /// Solves analytic IK for a world-frame gripper-base target.
    fn inverse_kinematics(&self, target: PyMmLiftGripperTarget) -> PyResult<PyMmLiftJointTarget> {
        self.inner
            .inverse_kinematics(target.into())
            .map(Into::into)
            .map_err(ik_error_to_py)
    }

    /// Solves shoulder / elbow IK at a fixed lift displacement.
    fn inverse_kinematics_at_lift(
        &self,
        lift_m: f64,
        gripper_x_m: f64,
        gripper_z_m: f64,
    ) -> PyResult<PyMmLiftJointTarget> {
        self.inner
            .inverse_kinematics_at_lift(lift_m, gripper_x_m, gripper_z_m)
            .map(Into::into)
            .map_err(ik_error_to_py)
    }
}

/// Observation returned by the mobile manipulator environment.
#[pyclass(name = "MobileManipulatorObservation")]
#[derive(Clone, Copy)]
struct PyMmObservation {
    inner: MobileManipulatorObservation,
}

#[pymethods]
impl PyMmObservation {
    #[getter]
    fn base_x(&self) -> f64 {
        self.inner.base_x_m
    }

    #[getter]
    fn base_y(&self) -> f64 {
        self.inner.base_y_m
    }

    #[getter]
    fn base_z(&self) -> f64 {
        self.inner.base_z_m
    }

    #[getter]
    fn base_yaw(&self) -> f64 {
        self.inner.base_yaw_rad
    }

    #[getter]
    fn ee_x(&self) -> f64 {
        self.inner.ee_x_m
    }

    #[getter]
    fn ee_y(&self) -> f64 {
        self.inner.ee_y_m
    }

    #[getter]
    fn ee_z(&self) -> f64 {
        self.inner.ee_z_m
    }

    #[getter]
    fn shoulder_position(&self) -> f64 {
        self.inner.shoulder_position_rad
    }

    #[getter]
    fn elbow_position(&self) -> f64 {
        self.inner.elbow_position_rad
    }

    #[getter]
    fn gripper_position(&self) -> f64 {
        self.inner.gripper_position_rad
    }

    #[getter]
    fn lift_position_m(&self) -> f64 {
        self.inner.lift_position_m
    }

    #[getter]
    fn wrist_camera_pixels(&self) -> usize {
        self.inner.wrist_camera_pixels
    }

    #[getter]
    fn joint_state_count(&self) -> usize {
        self.inner.joint_state_count
    }

    #[getter]
    fn target_dx(&self) -> f64 {
        self.inner.target_dx_m
    }

    #[getter]
    fn target_dy(&self) -> f64 {
        self.inner.target_dy_m
    }

    #[getter]
    fn target_dz(&self) -> f64 {
        self.inner.target_dz_m
    }

    #[getter]
    fn wrist_depth_center_m(&self) -> f64 {
        self.inner.wrist_depth_center_m
    }

    #[getter]
    fn wrist_depth_min_m(&self) -> f64 {
        self.inner.wrist_depth_min_m
    }

    #[getter]
    fn target_object_index(&self) -> u32 {
        self.inner.target_object_index
    }

    fn __repr__(&self) -> String {
        format!(
            "MobileManipulatorObservation(ee=({:.3}, {:.3}, {:.3}), shoulder={:.3}, elbow={:.3}, gripper={:.3})",
            self.inner.ee_x_m,
            self.inner.ee_y_m,
            self.inner.ee_z_m,
            self.inner.shoulder_position_rad,
            self.inner.elbow_position_rad,
            self.inner.gripper_position_rad,
        )
    }
}

impl From<MobileManipulatorObservation> for PyMmObservation {
    fn from(inner: MobileManipulatorObservation) -> Self {
        Self { inner }
    }
}

/// Result of a mobile manipulator episode reset or step.
#[pyclass(name = "MobileManipulatorStepResult")]
#[derive(Clone, Copy)]
struct PyMmStepResult {
    observation: PyMmObservation,
    reward: f64,
    terminated: bool,
    truncated: bool,
}

#[pymethods]
impl PyMmStepResult {
    #[getter]
    fn observation(&self) -> PyMmObservation {
        self.observation
    }

    #[getter]
    fn reward(&self) -> f64 {
        self.reward
    }

    #[getter]
    fn terminated(&self) -> bool {
        self.terminated
    }

    #[getter]
    fn truncated(&self) -> bool {
        self.truncated
    }

    #[getter]
    fn done(&self) -> bool {
        self.terminated || self.truncated
    }

    fn __repr__(&self) -> String {
        format!(
            "MobileManipulatorStepResult(reward={:.3}, terminated={}, truncated={})",
            self.reward, self.terminated, self.truncated
        )
    }
}

impl From<rne_ai::EpisodeStep<MobileManipulatorObservation>> for PyMmStepResult {
    fn from(value: rne_ai::EpisodeStep<MobileManipulatorObservation>) -> Self {
        Self {
            observation: value.observation.into(),
            reward: value.reward,
            terminated: value.terminated,
            truncated: value.truncated,
        }
    }
}

/// Headless mobile manipulator simulation exposed to Python.
#[pyclass(name = "MobileManipulatorSim")]
struct PyMobileManipulatorSim {
    inner: MobileManipulatorSim,
}

#[pymethods]
impl PyMobileManipulatorSim {
    /// Creates a sim for `"mm_minimal"` (default), `"mm_mobile"`, or `"mm_lift"`.
    #[new]
    #[pyo3(signature = (mode="mm_minimal"))]
    fn new(mode: &str) -> PyResult<Self> {
        let inner = match mode {
            "mm_minimal" => MobileManipulatorSim::new_mm_minimal(),
            "mm_mobile" => MobileManipulatorSim::new_mm_mobile(),
            "mm_lift" => MobileManipulatorSim::new_mm_lift(),
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "unknown mode '{other}', expected 'mm_minimal', 'mm_mobile', or 'mm_lift'"
                )))
            }
        };
        Ok(Self { inner })
    }

    fn reset(&mut self) -> PyMmObservation {
        self.inner.reset().into()
    }

    #[pyo3(signature = (
        left_wheel_velocity_rad_s=0.0,
        right_wheel_velocity_rad_s=0.0,
        shoulder_velocity_rad_s=0.0,
        elbow_velocity_rad_s=0.0,
        gripper_velocity_rad_s=0.0,
        lift_velocity_m_s=0.0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        left_wheel_velocity_rad_s: f64,
        right_wheel_velocity_rad_s: f64,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
        gripper_velocity_rad_s: f64,
        lift_velocity_m_s: f64,
    ) -> PyMmObservation {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
                lift_velocity_m_s,
                ..MobileManipulatorAction::default()
            })
            .into()
    }

    /// Steps the sim while holding absolute lift-arm joint targets.
    #[pyo3(signature = (lift_m, shoulder_rad, elbow_rad, gripper_velocity_rad_s=0.0))]
    fn step_hold_lift_joints(
        &mut self,
        lift_m: f64,
        shoulder_rad: f64,
        elbow_rad: f64,
        gripper_velocity_rad_s: f64,
    ) -> PyMmObservation {
        let mut action = MobileManipulatorAction::hold_lift_joints(MmLiftJointTarget {
            lift_m,
            shoulder_rad,
            elbow_rad,
        });
        action.gripper_velocity_rad_s = gripper_velocity_rad_s;
        self.inner.step(action).into()
    }

    #[getter]
    fn step_count(&self) -> u64 {
        self.inner.step_count()
    }

    #[getter]
    fn is_grasping(&self) -> bool {
        self.inner.is_grasping()
    }
}

/// Mobile manipulator manipulation episode with reward and termination.
#[pyclass(name = "MobileManipulatorEpisode")]
struct PyMobileManipulatorEpisode {
    inner: MobileManipulatorEpisode,
}

#[pymethods]
impl PyMobileManipulatorEpisode {
    /// Creates an episode for the `"reach"`, `"place"` (default), `"lift_place"`,
    /// `"transport"`, or `"inspect"` task. `"lift_place"` needs the `lift_velocity_m_s`
    /// step argument to drive the vertical lift.
    #[new]
    #[pyo3(signature = (task="place"))]
    fn new(task: &str) -> PyResult<Self> {
        Ok(Self {
            inner: MobileManipulatorEpisode::new(mm_episode_config(task)?),
        })
    }

    fn reset(&mut self) -> PyMmStepResult {
        self.inner.reset().into()
    }

    #[pyo3(signature = (
        left_wheel_velocity_rad_s=0.0,
        right_wheel_velocity_rad_s=0.0,
        shoulder_velocity_rad_s=0.0,
        elbow_velocity_rad_s=0.0,
        gripper_velocity_rad_s=0.0,
        lift_velocity_m_s=0.0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        left_wheel_velocity_rad_s: f64,
        right_wheel_velocity_rad_s: f64,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
        gripper_velocity_rad_s: f64,
        lift_velocity_m_s: f64,
    ) -> PyMmStepResult {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
                lift_velocity_m_s,
                ..MobileManipulatorAction::default()
            })
            .into()
    }

    /// Steps the episode while holding absolute lift-arm joint targets.
    #[pyo3(signature = (lift_m, shoulder_rad, elbow_rad, gripper_velocity_rad_s=0.0))]
    fn step_hold_lift_joints(
        &mut self,
        lift_m: f64,
        shoulder_rad: f64,
        elbow_rad: f64,
        gripper_velocity_rad_s: f64,
    ) -> PyMmStepResult {
        let mut action = MobileManipulatorAction::hold_lift_joints(MmLiftJointTarget {
            lift_m,
            shoulder_rad,
            elbow_rad,
        });
        action.gripper_velocity_rad_s = gripper_velocity_rad_s;
        self.inner.step(action).into()
    }

    #[getter]
    fn step_in_episode(&self) -> u64 {
        self.inner.step_in_episode()
    }

    #[getter]
    fn total_reward(&self) -> f64 {
        self.inner.total_reward()
    }

    #[getter]
    fn is_grasping(&self) -> bool {
        self.inner.simulation().is_grasping()
    }

    /// Active reach-curriculum stage index (None when no curriculum is configured).
    #[getter]
    fn curriculum_stage(&self) -> Option<usize> {
        self.inner.curriculum_stage()
    }
}

/// Scripted IK pick-place policy for the clutter `place` task (matches example 26).
#[pyclass(name = "IkClutterPickPlacePolicy")]
struct PyIkClutterPickPlacePolicy {
    inner: IkClutterPickPlacePolicy,
}

#[pymethods]
impl PyIkClutterPickPlacePolicy {
    #[new]
    fn new() -> Self {
        Self {
            inner: IkClutterPickPlacePolicy::new(),
        }
    }

    /// Total scripted steps (settle → approach → carry → hold → release).
    fn total_steps(&self) -> u64 {
        self.inner.total_steps()
    }

    /// Returns `(left_wheel, right_wheel, shoulder, elbow, gripper, lift)` rad/s or m/s.
    fn act(&mut self, observation: PyMmObservation) -> (f64, f64, f64, f64, f64, f64) {
        let action = self.inner.act(&observation.inner);
        (
            action.left_wheel_velocity_rad_s,
            action.right_wheel_velocity_rad_s,
            action.shoulder_velocity_rad_s,
            action.elbow_velocity_rad_s,
            action.gripper_velocity_rad_s,
            action.lift_velocity_m_s,
        )
    }
}

/// Batched mobile manipulator environment for population-based / parallel RL.
#[pyclass(name = "VectorizedMobileManipulatorEnv")]
struct PyVectorizedMobileManipulatorEnv {
    inner: VectorizedMobileManipulatorEnv,
}

#[pymethods]
impl PyVectorizedMobileManipulatorEnv {
    /// Creates `num_envs` environments for the given task (default `"reach"`).
    #[new]
    #[pyo3(signature = (task="reach", num_envs=16))]
    fn new(task: &str, num_envs: usize) -> PyResult<Self> {
        if num_envs == 0 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "num_envs must be positive",
            ));
        }
        let config = VectorizedMobileManipulatorConfig::new(mm_episode_config(task)?, num_envs);
        Ok(Self {
            inner: VectorizedMobileManipulatorEnv::new(config),
        })
    }

    #[getter]
    fn num_envs(&self) -> usize {
        self.inner.num_envs()
    }

    /// Resets every environment and returns the initial observation batch.
    fn reset(&mut self) -> Vec<PyMmObservation> {
        self.inner
            .reset()
            .observations
            .into_iter()
            .map(PyMmObservation::from)
            .collect()
    }

    /// Steps all environments; returns per-env `(observations, done)`.
    ///
    /// Each action is `(left_wheel, right_wheel, shoulder, elbow, gripper)` in rad/s.
    fn step(
        &mut self,
        actions: Vec<(f64, f64, f64, f64, f64)>,
    ) -> PyResult<(Vec<PyMmObservation>, Vec<bool>)> {
        if actions.len() != self.inner.num_envs() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "expected {} actions, got {}",
                self.inner.num_envs(),
                actions.len()
            )));
        }
        let actions: Vec<MobileManipulatorAction> = actions
            .into_iter()
            .map(
                |(left, right, shoulder, elbow, gripper)| MobileManipulatorAction {
                    left_wheel_velocity_rad_s: left,
                    right_wheel_velocity_rad_s: right,
                    shoulder_velocity_rad_s: shoulder,
                    elbow_velocity_rad_s: elbow,
                    gripper_velocity_rad_s: gripper,
                    lift_velocity_m_s: 0.0,
                    ..MobileManipulatorAction::default()
                },
            )
            .collect();
        let step = self.inner.step(&actions);
        let done = step
            .terminated
            .iter()
            .zip(&step.truncated)
            .map(|(terminated, truncated)| *terminated || *truncated)
            .collect();
        let observations = step
            .observations
            .into_iter()
            .map(PyMmObservation::from)
            .collect();
        Ok((observations, done))
    }

    /// Cumulative reward of one environment's current episode.
    fn episode_reward(&self, index: usize) -> PyResult<f64> {
        if index >= self.inner.num_envs() {
            return Err(pyo3::exceptions::PyIndexError::new_err(format!(
                "env index {index} out of range (num_envs={})",
                self.inner.num_envs()
            )));
        }
        Ok(self.inner.episode(index).total_reward())
    }

    /// Returns a JSON checkpoint for deterministic resume.
    fn checkpoint_json(&self) -> PyResult<String> {
        serde_json::to_string(&self.inner.checkpoint()).map_err(|error| {
            pyo3::exceptions::PyValueError::new_err(format!(
                "failed to serialize checkpoint: {error}"
            ))
        })
    }

    /// Restores this environment from a JSON checkpoint.
    fn restore_checkpoint_json(&mut self, checkpoint_json: &str) -> PyResult<()> {
        let checkpoint: rne_ai::VectorizedMobileManipulatorSnapshot =
            serde_json::from_str(checkpoint_json).map_err(|error| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "failed to parse checkpoint: {error}"
                ))
            })?;
        self.inner
            .restore_checkpoint(&checkpoint)
            .map_err(|error| pyo3::exceptions::PyValueError::new_err(format!("{error:?}")))
    }

    /// Writes a JSON checkpoint to `path`.
    fn save_checkpoint(&self, path: &str) -> PyResult<()> {
        let json = self.checkpoint_json()?;
        atomic_write_checkpoint(Path::new(path), &json)
    }

    /// Restores this environment from a JSON checkpoint file.
    fn load_checkpoint(&mut self, path: &str) -> PyResult<()> {
        let json = std::fs::read_to_string(Path::new(path)).map_err(|error| {
            pyo3::exceptions::PyOSError::new_err(format!(
                "failed to read checkpoint '{path}': {error}"
            ))
        })?;
        self.restore_checkpoint_json(&json)
    }
}

/// Robot Native Engine Python module.
#[pymodule]
fn rne_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyDiffDriveSim>()?;
    m.add_class::<PyDiffDriveEpisode>()?;
    m.add_class::<PyObservation>()?;
    m.add_class::<PyStepResult>()?;
    m.add_class::<PyMmLiftJointTarget>()?;
    m.add_class::<PyMmLiftGripperTarget>()?;
    m.add_class::<PyMmLiftKinematics>()?;
    m.add_class::<PyMobileManipulatorSim>()?;
    m.add_class::<PyMobileManipulatorEpisode>()?;
    m.add_class::<PyIkClutterPickPlacePolicy>()?;
    m.add_class::<PyVectorizedMobileManipulatorEnv>()?;
    m.add_class::<PyMmObservation>()?;
    m.add_class::<PyMmStepResult>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mm_lift_kinematics_roundtrip_from_python_api() {
        let kin = MmLiftKinematics::mm_lift();
        let joints = MmLiftJointTarget {
            lift_m: -0.1,
            shoulder_rad: 0.4,
            elbow_rad: 0.6,
        };
        let target = kin.forward_kinematics(joints);
        let solved = kin
            .inverse_kinematics(target)
            .expect("roundtrip target should be reachable");
        let reshot = kin.forward_kinematics(solved);
        approx::assert_relative_eq!(target.x_m, reshot.x_m, epsilon = 1e-9);
        approx::assert_relative_eq!(target.z_m, reshot.z_m, epsilon = 1e-9);
    }

    fn assert_py_error<T>(error: PyErr, expected_message: &str)
    where
        T: pyo3::type_object::PyTypeInfo,
    {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            assert!(error.is_instance_of::<T>(py));
            assert!(
                error.value(py).to_string().contains(expected_message),
                "expected error message to contain {expected_message:?}, got {:?}",
                error.value(py).to_string()
            );
        });
    }

    #[derive(Debug, PartialEq)]
    struct VectorizedCheckpointSummary {
        schema_version: u32,
        auto_reset: bool,
        episodes: Vec<EpisodeCheckpointSummary>,
    }

    #[derive(Debug, PartialEq)]
    struct EpisodeCheckpointSummary {
        schema_version: u32,
        episode_index: u32,
        step_in_episode: u64,
        total_reward: f64,
        sim_ticks: u64,
        sim_step_count: u64,
        random_sequence: u64,
        random_ticks: u64,
    }

    fn vectorized_checkpoint_summary(json: &str) -> VectorizedCheckpointSummary {
        let snapshot: rne_ai::VectorizedMobileManipulatorSnapshot =
            serde_json::from_str(json).unwrap();
        VectorizedCheckpointSummary {
            schema_version: snapshot.schema_version,
            auto_reset: snapshot.auto_reset,
            episodes: snapshot
                .episodes
                .iter()
                .map(|episode| EpisodeCheckpointSummary {
                    schema_version: episode.schema_version,
                    episode_index: episode.episode_index,
                    step_in_episode: episode.step_in_episode,
                    total_reward: episode.total_reward,
                    sim_ticks: episode.simulation.sim_ticks,
                    sim_step_count: episode.simulation.step_count,
                    random_sequence: episode.random.sequence,
                    random_ticks: episode.random.sim_ticks,
                })
                .collect(),
        }
    }

    #[test]
    fn rust_sim_moves_forward() {
        let mut sim = DiffDriveSim::new();
        let mut final_x = 0.0;
        for _ in 0..300 {
            final_x = sim.step(6.0, 6.0).base_x_m;
        }
        assert!(final_x > 0.5);
    }

    #[test]
    fn rust_episode_reaches_goal() {
        let mut env = sim::DiffDriveEpisode::new(DiffDriveEpisodeConfig {
            goal_x_m: 1.5,
            ..DiffDriveEpisodeConfig::default()
        });
        let mut step = env.reset();
        while !step.is_done() {
            step = env.step(sim::DiffDriveAction::forward(6.0));
        }
        assert!(step.terminated);
    }

    #[test]
    fn mobile_manipulator_place_episode_succeeds() {
        use rne_ai::{IkClutterPickPlacePolicy, Policy};

        let mut env = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::place());
        let mut policy = IkClutterPickPlacePolicy::new();
        let mut step = env.reset();
        for _ in 0..policy.total_steps() {
            step = env.step(policy.act(&step.observation));
            if step.terminated {
                return;
            }
        }
        panic!("expected mobile manipulator place episode to terminate");
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_json_restores_state() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach_random", 2).unwrap();
        env.reset();
        env.step(vec![(0.0, 0.0, 0.5, 0.0, 0.0), (0.0, 0.0, 0.0, -0.25, 0.0)])
            .unwrap();
        let checkpoint = env.checkpoint_json().unwrap();
        let summary = vectorized_checkpoint_summary(&checkpoint);
        let reward_0 = env.episode_reward(0).unwrap();
        let reward_1 = env.episode_reward(1).unwrap();

        env.step(vec![(0.0, 0.0, -1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0, 0.0)])
            .unwrap();
        env.restore_checkpoint_json(&checkpoint).unwrap();

        assert_eq!(
            vectorized_checkpoint_summary(&env.checkpoint_json().unwrap()),
            summary
        );
        approx::assert_relative_eq!(env.episode_reward(0).unwrap(), reward_0, epsilon = 1e-12);
        approx::assert_relative_eq!(env.episode_reward(1).unwrap(), reward_1, epsilon = 1e-12);
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_file_restores_state() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach", 2).unwrap();
        env.reset();
        env.step(vec![(0.0, 0.0, 0.25, 0.0, 0.0), (0.0, 0.0, 0.0, 0.25, 0.0)])
            .unwrap();
        let checkpoint = env.checkpoint_json().unwrap();
        let summary = vectorized_checkpoint_summary(&checkpoint);
        let reward_0 = env.episode_reward(0).unwrap();
        let reward_1 = env.episode_reward(1).unwrap();

        let file = tempfile::NamedTempFile::new().unwrap();
        let path = file.path().to_string_lossy().to_string();
        env.save_checkpoint(&path).unwrap();

        env.step(vec![(0.0, 0.0, -0.5, 0.0, 0.0), (0.0, 0.0, 0.0, -0.5, 0.0)])
            .unwrap();
        env.load_checkpoint(&path).unwrap();

        assert_eq!(
            vectorized_checkpoint_summary(&env.checkpoint_json().unwrap()),
            summary
        );
        approx::assert_relative_eq!(env.episode_reward(0).unwrap(), reward_0, epsilon = 1e-12);
        approx::assert_relative_eq!(env.episode_reward(1).unwrap(), reward_1, epsilon = 1e-12);
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_save_creates_parent_directory() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        env.reset();

        let directory = tempfile::tempdir().unwrap();
        let path = directory
            .path()
            .join("nested")
            .join("mobile_manipulator_checkpoint.json");
        env.save_checkpoint(path.to_str().unwrap()).unwrap();

        assert!(path.is_file());
        assert!(std::fs::read_to_string(&path).unwrap().ends_with('\n'));
        let mut restored = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        restored.load_checkpoint(path.to_str().unwrap()).unwrap();
        assert_eq!(
            vectorized_checkpoint_summary(&restored.checkpoint_json().unwrap()),
            vectorized_checkpoint_summary(&env.checkpoint_json().unwrap())
        );
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_save_retries_stale_temp_file() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        env.reset();

        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("mobile_manipulator_checkpoint.json");
        let stale_temp = checkpoint_temp_path(&path, 0).unwrap();
        std::fs::write(&stale_temp, "stale checkpoint temp").unwrap();

        env.save_checkpoint(path.to_str().unwrap()).unwrap();

        assert!(path.is_file());
        assert_eq!(
            std::fs::read_to_string(&stale_temp).unwrap(),
            "stale checkpoint temp"
        );
        let mut restored = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        restored.load_checkpoint(path.to_str().unwrap()).unwrap();
        assert_eq!(
            vectorized_checkpoint_summary(&restored.checkpoint_json().unwrap()),
            vectorized_checkpoint_summary(&env.checkpoint_json().unwrap())
        );
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_rejects_invalid_json() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        let error = env
            .restore_checkpoint_json("{not valid json")
            .expect_err("invalid checkpoint JSON should fail");

        assert_py_error::<pyo3::exceptions::PyValueError>(error, "failed to parse checkpoint");
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_rejects_wrong_env_count() {
        let mut source = PyVectorizedMobileManipulatorEnv::new("reach", 2).unwrap();
        source.reset();
        let checkpoint = source.checkpoint_json().unwrap();
        let mut target = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        let error = target
            .restore_checkpoint_json(&checkpoint)
            .expect_err("checkpoint env count mismatch should fail");

        assert_py_error::<pyo3::exceptions::PyValueError>(error, "EnvCountMismatch");
    }

    #[test]
    fn vectorized_mobile_manipulator_checkpoint_load_reports_missing_file() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("missing_checkpoint.json");
        let mut env = PyVectorizedMobileManipulatorEnv::new("reach", 1).unwrap();
        let error = env
            .load_checkpoint(path.to_str().unwrap())
            .expect_err("missing checkpoint file should fail");

        assert_py_error::<pyo3::exceptions::PyOSError>(error, "failed to read checkpoint");
    }
}
