//! Python bindings for Robot Native Engine.

#![deny(missing_docs)]

mod sim;

use pyo3::prelude::*;
use rne_ai::{
    unitree_go2_task_spec, DiffDriveEpisodeConfig, Episode, GraspMode, IkClutterPickPlacePolicy,
    IkMobileClutterPickPlacePolicy, IkMobileLiftPickPlacePolicy, MobileLiftFailureClass, Policy,
    PortableBatchConfig, PortableBatchRunner, PortableBatchStep, TaskSpec, UnitreeGo2Action,
    UnitreeGo2Episode, UnitreeGo2EpisodeConfig, UnitreeGo2Observation,
};
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

/// Validates a TaskSpec JSON document and returns its canonical compact JSON.
#[pyfunction]
fn canonical_task_spec_json(task_spec_json: &str) -> PyResult<String> {
    let task_spec: TaskSpec = serde_json::from_str(task_spec_json).map_err(|error| {
        pyo3::exceptions::PyValueError::new_err(format!("failed to parse TaskSpec JSON: {error}"))
    })?;
    task_spec.validate().map_err(|error| {
        pyo3::exceptions::PyValueError::new_err(format!("invalid TaskSpec: {error}"))
    })?;
    serde_json::to_string(&task_spec).map_err(|error| {
        pyo3::exceptions::PyValueError::new_err(format!("failed to serialize TaskSpec: {error}"))
    })
}

/// Returns the v1 lane-local episode seed used by the Rust batch runner.
#[pyfunction(name = "derive_episode_seed")]
fn py_derive_episode_seed(root_seed: u64, lane_id: u64, episode_index: u64) -> u64 {
    rne_ai::derive_episode_seed(root_seed, lane_id, episode_index)
}

/// Resolves a task name to a mobile manipulator episode configuration.
fn mm_episode_config(task: &str) -> PyResult<MobileManipulatorEpisodeConfig> {
    match task {
        "reach" => Ok(MobileManipulatorEpisodeConfig::reach()),
        "reach_random" => Ok(MobileManipulatorEpisodeConfig::reach_randomized(0)),
        "reach_curriculum" => Ok(MobileManipulatorEpisodeConfig::reach_curriculum(0)),
        "place" => Ok(MobileManipulatorEpisodeConfig::place()),
        "lift_place" => Ok(MobileManipulatorEpisodeConfig::lift_pick_place()),
        "mobile_lift_place" => Ok(MobileManipulatorEpisodeConfig::mobile_lift_pick_place()),
        "clutter_place" => Ok(MobileManipulatorEpisodeConfig::clutter_pick_place(0)),
        "clutter_place_center" => Ok(MobileManipulatorEpisodeConfig::clutter_pick_place_center(0)),
        "mobile_clutter_place_center" => {
            Ok(MobileManipulatorEpisodeConfig::mobile_clutter_pick_place_center(0))
        }
        "mobile_clutter_place" => Ok(MobileManipulatorEpisodeConfig::mobile_clutter_pick_place(0)),
        "transport" => Ok(MobileManipulatorEpisodeConfig::transport()),
        "inspect" => Ok(MobileManipulatorEpisodeConfig::inspect()),
        other => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "unknown task '{other}', expected 'reach', 'reach_random', 'reach_curriculum', 'place', 'lift_place', 'mobile_lift_place', 'clutter_place', 'clutter_place_center', 'mobile_clutter_place', 'mobile_clutter_place_center', 'transport', or 'inspect'"
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
#[pyclass(name = "Observation", skip_from_py_object)]
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
#[pyclass(name = "StepResult", skip_from_py_object)]
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

fn unitree_go2_observation_vector(observation: UnitreeGo2Observation) -> Vec<f64> {
    vec![
        observation.base_x_m,
        observation.base_y_m,
        observation.base_z_m,
        observation.base_yaw_rad,
        observation.base_pitch_rad,
        observation.base_roll_rad,
        observation.base_linear_velocity_m_s[0],
        observation.base_linear_velocity_m_s[1],
        observation.base_linear_velocity_m_s[2],
        observation.base_angular_velocity_rad_s[0],
        observation.base_angular_velocity_rad_s[1],
        observation.base_angular_velocity_rad_s[2],
        observation.base_relative_yaw_rad,
        observation.base_relative_pitch_rad,
        observation.base_relative_roll_rad,
        observation.fl_foot_impulse_ns,
        observation.fr_foot_impulse_ns,
        observation.rl_foot_impulse_ns,
        observation.rr_foot_impulse_ns,
        observation.gait_phase,
        observation.progress,
    ]
}

/// Result of a Unitree Go2 gait reset or step.
#[pyclass(name = "UnitreeGo2StepResult", skip_from_py_object)]
#[derive(Clone)]
struct PyUnitreeGo2StepResult {
    observation: Vec<f64>,
    reward: f64,
    terminated: bool,
    truncated: bool,
}

#[pymethods]
impl PyUnitreeGo2StepResult {
    #[getter]
    fn observation(&self) -> Vec<f64> {
        self.observation.clone()
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
}

fn unitree_go2_step_result(
    step: rne_ai::EpisodeStep<UnitreeGo2Observation>,
) -> PyUnitreeGo2StepResult {
    PyUnitreeGo2StepResult {
        observation: unitree_go2_observation_vector(step.observation),
        reward: step.reward,
        terminated: step.terminated,
        truncated: step.truncated,
    }
}

/// Headless Unitree Go2 gait episode exposed to Python RL adapters.
#[pyclass(name = "UnitreeGo2GaitEpisode")]
struct PyUnitreeGo2GaitEpisode {
    inner: UnitreeGo2Episode,
    task_spec: TaskSpec,
}

#[pymethods]
impl PyUnitreeGo2GaitEpisode {
    /// Creates a seeded Go2 gait episode with a maximum step budget.
    #[new]
    #[pyo3(signature = (max_steps=600, seed=1))]
    fn new(max_steps: u64, seed: u64) -> PyResult<Self> {
        if max_steps == 0 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "max_steps must be greater than zero",
            ));
        }
        let config = UnitreeGo2EpisodeConfig {
            max_steps,
            ..UnitreeGo2EpisodeConfig::default()
        };
        let inner = UnitreeGo2Episode::new_with_seed(config, seed).map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "failed to load Unitree Go2 gait scene: {error:?}"
            ))
        })?;
        let task_spec = unitree_go2_task_spec(max_steps);
        task_spec.validate().map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "internal Unitree Go2 TaskSpec is invalid: {error}"
            ))
        })?;
        Ok(Self { inner, task_spec })
    }

    /// Returns the canonical portable TaskSpec JSON used by this episode.
    fn task_spec_json(&self) -> PyResult<String> {
        serde_json::to_string(&self.task_spec).map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "failed to serialize Unitree Go2 TaskSpec: {error}"
            ))
        })
    }

    /// Resets the episode and returns its 21-element numeric observation.
    fn reset(&mut self) -> PyUnitreeGo2StepResult {
        self.inner.reset().into()
    }

    /// Applies the five continuous gait controls and advances one 60 Hz tick.
    #[pyo3(signature = (
        stride_rad=0.12,
        foot_lift_rad=0.16,
        roll_correction_rad=0.0,
        pitch_correction_rad=0.0,
        lateral_extension_rad=0.0,
    ))]
    fn step(
        &mut self,
        stride_rad: f64,
        foot_lift_rad: f64,
        roll_correction_rad: f64,
        pitch_correction_rad: f64,
        lateral_extension_rad: f64,
    ) -> PyUnitreeGo2StepResult {
        self.inner
            .step(UnitreeGo2Action {
                stride_rad,
                foot_lift_rad,
                roll_correction_rad,
                pitch_correction_rad,
                lateral_extension_rad,
            })
            .into()
    }

    /// Number of completed control ticks in the current episode.
    #[getter]
    fn step_in_episode(&self) -> u64 {
        self.inner.step_in_episode()
    }
}

impl From<rne_ai::EpisodeStep<UnitreeGo2Observation>> for PyUnitreeGo2StepResult {
    fn from(value: rne_ai::EpisodeStep<UnitreeGo2Observation>) -> Self {
        unitree_go2_step_result(value)
    }
}

/// Result of a portable Unitree Go2 batch reset or step.
#[pyclass(name = "PortableUnitreeGo2BatchStep", skip_from_py_object)]
#[derive(Clone)]
struct PyPortableUnitreeGo2BatchStep {
    lane_ids: Vec<u64>,
    episode_indices: Vec<u64>,
    episode_seeds: Vec<Option<u64>>,
    resets: Vec<bool>,
    observations: Vec<Vec<f64>>,
    rewards: Vec<f64>,
    terminated: Vec<bool>,
    truncated: Vec<bool>,
}

#[pymethods]
impl PyPortableUnitreeGo2BatchStep {
    /// Stable lane IDs in result order.
    #[getter]
    fn lane_ids(&self) -> Vec<u64> {
        self.lane_ids.clone()
    }

    /// Lane-local episode indices in result order.
    #[getter]
    fn episode_indices(&self) -> Vec<u64> {
        self.episode_indices.clone()
    }

    /// Derived episode seeds in result order.
    #[getter]
    fn episode_seeds(&self) -> Vec<Option<u64>> {
        self.episode_seeds.clone()
    }

    /// Reset mask in result order.
    #[getter]
    fn resets(&self) -> Vec<bool> {
        self.resets.clone()
    }

    /// Flat 21-value observations in stable TaskSpec order.
    #[getter]
    fn observations(&self) -> Vec<Vec<f64>> {
        self.observations.clone()
    }

    /// Scalar rewards in result order.
    #[getter]
    fn rewards(&self) -> Vec<f64> {
        self.rewards.clone()
    }

    /// Termination flags in result order.
    #[getter]
    fn terminated(&self) -> Vec<bool> {
        self.terminated.clone()
    }

    /// Truncation flags in result order.
    #[getter]
    fn truncated(&self) -> Vec<bool> {
        self.truncated.clone()
    }
}

impl From<PortableBatchStep<UnitreeGo2Observation>> for PyPortableUnitreeGo2BatchStep {
    fn from(step: PortableBatchStep<UnitreeGo2Observation>) -> Self {
        Self {
            lane_ids: step.lane_ids,
            episode_indices: step.episode_indices,
            episode_seeds: step.episode_seeds,
            resets: step.resets,
            observations: step
                .observations
                .into_iter()
                .map(unitree_go2_observation_vector)
                .collect(),
            rewards: step.rewards,
            terminated: step.terminated,
            truncated: step.truncated,
        }
    }
}

/// TaskSpec-bound deterministic CPU batch for Unitree Go2 gait learning.
#[pyclass(name = "PortableUnitreeGo2Batch")]
struct PyPortableUnitreeGo2Batch {
    inner: PortableBatchRunner<UnitreeGo2Episode>,
}

#[pymethods]
impl PyPortableUnitreeGo2Batch {
    /// Creates a deterministic batch with stable lane IDs and lane-local seeds.
    #[new]
    #[pyo3(signature = (num_envs=1, max_steps=600, seed=1, auto_reset=true))]
    fn new(num_envs: usize, max_steps: u64, seed: u64, auto_reset: bool) -> PyResult<Self> {
        if num_envs == 0 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "num_envs must be positive",
            ));
        }
        if max_steps == 0 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "max_steps must be greater than zero",
            ));
        }
        let episode_config = UnitreeGo2EpisodeConfig {
            max_steps,
            ..UnitreeGo2EpisodeConfig::default()
        };
        let first_seed = rne_ai::derive_episode_seed(seed, 0, 0);
        UnitreeGo2Episode::new_with_seed(episode_config.clone(), first_seed).map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "failed to load Unitree Go2 batch scene: {error:?}"
            ))
        })?;
        let factory_config = episode_config.clone();
        let inner = PortableBatchRunner::from_task_spec(
            unitree_go2_task_spec(max_steps),
            PortableBatchConfig {
                num_envs,
                seed,
                auto_reset,
            },
            move |episode_seed| {
                UnitreeGo2Episode::new_with_seed(factory_config.clone(), episode_seed)
                    .expect("preflight-validated Unitree Go2 scene must remain loadable")
            },
        )
        .map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "internal Unitree Go2 TaskSpec is invalid: {error}"
            ))
        })?;
        Ok(Self { inner })
    }

    /// Number of stable batch lanes.
    #[getter]
    fn num_envs(&self) -> usize {
        self.inner.num_envs()
    }

    /// Canonical TaskSpec JSON bound to this batch and its checkpoints.
    fn task_spec_json(&self) -> PyResult<String> {
        serde_json::to_string(
            self.inner
                .task_spec()
                .expect("portable Go2 batch always has a TaskSpec"),
        )
        .map_err(|error| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "failed to serialize Unitree Go2 TaskSpec: {error}"
            ))
        })
    }

    /// Fully resets the batch to lane-local episode zero.
    fn reset(&mut self) -> PyPortableUnitreeGo2BatchStep {
        self.inner.reset().into()
    }

    /// Partially resets canonical increasing lane IDs.
    fn reset_lanes(&mut self, lane_ids: Vec<u64>) -> PyResult<PyPortableUnitreeGo2BatchStep> {
        self.inner
            .reset_lanes(&lane_ids)
            .map(PyPortableUnitreeGo2BatchStep::from)
            .map_err(|error| pyo3::exceptions::PyValueError::new_err(error.to_string()))
    }

    /// Applies one five-value gait action per lane in stable ID order.
    fn step(
        &mut self,
        actions: Vec<(f64, f64, f64, f64, f64)>,
    ) -> PyResult<PyPortableUnitreeGo2BatchStep> {
        if actions.len() != self.inner.num_envs() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "expected {} actions, got {}",
                self.inner.num_envs(),
                actions.len()
            )));
        }
        let actions = actions
            .into_iter()
            .map(
                |(
                    stride_rad,
                    foot_lift_rad,
                    roll_correction_rad,
                    pitch_correction_rad,
                    lateral_extension_rad,
                )| UnitreeGo2Action {
                    stride_rad,
                    foot_lift_rad,
                    roll_correction_rad,
                    pitch_correction_rad,
                    lateral_extension_rad,
                },
            )
            .collect::<Vec<_>>();
        Ok(self.inner.step(&actions).into())
    }

    /// Returns one lane's batch-width-independent replay digest.
    fn lane_replay_digest(&self, lane_id: u64) -> PyResult<u64> {
        self.inner.lane_replay_digest(lane_id).ok_or_else(|| {
            pyo3::exceptions::PyIndexError::new_err(format!(
                "lane ID {lane_id} out of range for {} lanes",
                self.inner.num_envs()
            ))
        })
    }

    /// Returns the versioned deterministic batch checkpoint as JSON.
    fn checkpoint_json(&self) -> PyResult<String> {
        let checkpoint = self
            .inner
            .checkpoint()
            .map_err(|error| pyo3::exceptions::PyValueError::new_err(error.to_string()))?;
        serde_json::to_string(&checkpoint).map_err(|error| {
            pyo3::exceptions::PyValueError::new_err(format!(
                "failed to serialize portable batch checkpoint: {error}"
            ))
        })
    }

    /// Restores a deterministic batch checkpoint from JSON.
    fn restore_checkpoint_json(&mut self, checkpoint_json: &str) -> PyResult<()> {
        let checkpoint: rne_ai::PortableBatchCheckpoint<UnitreeGo2Action> =
            serde_json::from_str(checkpoint_json).map_err(|error| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "failed to parse portable batch checkpoint: {error}"
                ))
            })?;
        self.inner
            .restore_checkpoint(&checkpoint)
            .map_err(|error| pyo3::exceptions::PyValueError::new_err(error.to_string()))
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
#[pyclass(name = "MmLiftJointTarget", from_py_object)]
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

/// Full mobile-manipulator action, including optional lift joint targets.
///
/// Use this type with `step_action()` when a controller emits absolute lift-arm
/// targets, as the scripted lift pick-and-place policy does. The velocity-only
/// `step()` methods remain available for ordinary RL rollouts.
#[pyclass(name = "MobileManipulatorAction", from_py_object)]
#[derive(Clone, Copy)]
struct PyMmAction {
    inner: MobileManipulatorAction,
}

#[pymethods]
impl PyMmAction {
    /// Creates a velocity action with optional absolute lift-arm targets.
    #[new]
    #[pyo3(signature = (
        left_wheel_velocity_rad_s=0.0,
        right_wheel_velocity_rad_s=0.0,
        shoulder_velocity_rad_s=0.0,
        elbow_velocity_rad_s=0.0,
        gripper_velocity_rad_s=0.0,
        gripper_velocity_m_s=0.0,
        lift_velocity_m_s=0.0,
        lift_joint_target=None,
        wrist_yaw_target_rad=None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        left_wheel_velocity_rad_s: f64,
        right_wheel_velocity_rad_s: f64,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
        gripper_velocity_rad_s: f64,
        gripper_velocity_m_s: f64,
        lift_velocity_m_s: f64,
        lift_joint_target: Option<PyMmLiftJointTarget>,
        wrist_yaw_target_rad: Option<f64>,
    ) -> Self {
        Self {
            inner: MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
                gripper_velocity_m_s,
                lift_velocity_m_s,
                lift_joint_target: lift_joint_target.map(Into::into),
                wrist_yaw_target_rad,
            },
        }
    }

    #[getter]
    fn left_wheel_velocity_rad_s(&self) -> f64 {
        self.inner.left_wheel_velocity_rad_s
    }

    #[getter]
    fn right_wheel_velocity_rad_s(&self) -> f64 {
        self.inner.right_wheel_velocity_rad_s
    }

    #[getter]
    fn shoulder_velocity_rad_s(&self) -> f64 {
        self.inner.shoulder_velocity_rad_s
    }

    #[getter]
    fn elbow_velocity_rad_s(&self) -> f64 {
        self.inner.elbow_velocity_rad_s
    }

    #[getter]
    fn gripper_velocity_rad_s(&self) -> f64 {
        self.inner.gripper_velocity_rad_s
    }

    #[getter]
    fn gripper_velocity_m_s(&self) -> f64 {
        self.inner.gripper_velocity_m_s
    }

    #[getter]
    fn lift_velocity_m_s(&self) -> f64 {
        self.inner.lift_velocity_m_s
    }

    #[getter]
    fn lift_joint_target(&self) -> Option<PyMmLiftJointTarget> {
        self.inner.lift_joint_target.map(Into::into)
    }

    #[getter]
    fn wrist_yaw_target_rad(&self) -> Option<f64> {
        self.inner.wrist_yaw_target_rad
    }

    fn __repr__(&self) -> String {
        format!(
            "MobileManipulatorAction(wheels=({:.3}, {:.3}), arm=({:.3}, {:.3}), gripper=({:.3} rad/s, {:.3} m/s), lift={:.3} m/s)",
            self.inner.left_wheel_velocity_rad_s,
            self.inner.right_wheel_velocity_rad_s,
            self.inner.shoulder_velocity_rad_s,
            self.inner.elbow_velocity_rad_s,
            self.inner.gripper_velocity_rad_s,
            self.inner.gripper_velocity_m_s,
            self.inner.lift_velocity_m_s,
        )
    }
}

impl From<MobileManipulatorAction> for PyMmAction {
    fn from(inner: MobileManipulatorAction) -> Self {
        Self { inner }
    }
}

/// World-frame gripper-base target for `mm_lift` analytic IK.
#[pyclass(name = "MmLiftGripperTarget", from_py_object)]
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
#[pyclass(name = "MobileManipulatorObservation", from_py_object)]
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
    fn wrist_yaw_position(&self) -> f64 {
        self.inner.wrist_yaw_position_rad
    }

    #[getter]
    fn gripper_position(&self) -> f64 {
        self.inner.gripper_position_rad
    }

    #[getter]
    fn gripper_position_m(&self) -> f64 {
        self.inner.gripper_position_m
    }

    #[getter]
    fn lift_position_m(&self) -> f64 {
        self.inner.lift_position_m
    }

    #[getter]
    fn is_grasping(&self) -> bool {
        self.inner.is_grasping
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
    fn wrist_target_pixel_u_px(&self) -> u32 {
        self.inner.wrist_target_pixel_u_px
    }

    #[getter]
    fn wrist_target_pixel_v_px(&self) -> u32 {
        self.inner.wrist_target_pixel_v_px
    }

    #[getter]
    fn wrist_target_depth_m(&self) -> f64 {
        self.inner.wrist_target_depth_m
    }

    #[getter]
    fn wrist_target_offset_x_m(&self) -> f64 {
        self.inner.wrist_target_offset_x_m
    }

    #[getter]
    fn wrist_target_offset_y_m(&self) -> f64 {
        self.inner.wrist_target_offset_y_m
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
#[pyclass(name = "MobileManipulatorStepResult", skip_from_py_object)]
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
    /// Creates a sim for `"mm_minimal"` (default), `"mm_mobile"`, `"mm_lift"`,
    /// or the lift-capable `"mm_mobile_lift"` robot.
    #[new]
    #[pyo3(signature = (mode="mm_minimal"))]
    fn new(mode: &str) -> PyResult<Self> {
        let inner = match mode {
            "mm_minimal" => MobileManipulatorSim::new_mm_minimal(),
            "mm_mobile" => MobileManipulatorSim::new_mm_mobile(),
            "mm_lift" => MobileManipulatorSim::new_mm_lift(),
            "mm_mobile_lift" => MobileManipulatorSim::new_mm_mobile_lift(),
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "unknown mode '{other}', expected 'mm_minimal', 'mm_mobile', 'mm_lift', or 'mm_mobile_lift'"
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
        gripper_velocity_m_s=0.0,
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
        gripper_velocity_m_s: f64,
    ) -> PyMmObservation {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
                gripper_velocity_m_s,
                lift_velocity_m_s,
                ..MobileManipulatorAction::default()
            })
            .into()
    }

    /// Applies a full action object, preserving optional lift-arm targets.
    fn step_action(&mut self, action: PyMmAction) -> PyMmObservation {
        self.inner.step(action.inner).into()
    }

    /// Selects `"weld"` or `"friction"` grasping for the next contact-triggered grasp.
    fn set_grasp_mode(&mut self, mode: &str) -> PyResult<()> {
        let mode = match mode {
            "weld" => GraspMode::Weld,
            "friction" => GraspMode::Friction,
            _ => {
                return Err(pyo3::exceptions::PyValueError::new_err(
                    "grasp mode must be 'weld' or 'friction'",
                ));
            }
        };
        self.inner.set_grasp_mode(mode);
        Ok(())
    }

    /// Steps the sim while holding absolute lift-arm joint targets.
    #[pyo3(signature = (lift_m, shoulder_rad, elbow_rad, gripper_velocity_rad_s=0.0, gripper_velocity_m_s=0.0))]
    fn step_hold_lift_joints(
        &mut self,
        lift_m: f64,
        shoulder_rad: f64,
        elbow_rad: f64,
        gripper_velocity_rad_s: f64,
        gripper_velocity_m_s: f64,
    ) -> PyMmObservation {
        let mut action = MobileManipulatorAction::hold_lift_joints(MmLiftJointTarget {
            lift_m,
            shoulder_rad,
            elbow_rad,
        });
        action.gripper_velocity_rad_s = gripper_velocity_rad_s;
        action.gripper_velocity_m_s = gripper_velocity_m_s;
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
    /// `"mobile_lift_place"`, `"transport"`, or `"inspect"` task. Lift tasks accept
    /// the `lift_velocity_m_s` and `gripper_velocity_m_s` step arguments.
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

    /// Selects `"weld"` or `"friction"` grasping for the current episode.
    /// Call after `reset()`, which restores the default weld mode.
    fn set_grasp_mode(&mut self, mode: &str) -> PyResult<()> {
        let mode = match mode {
            "weld" => GraspMode::Weld,
            "friction" => GraspMode::Friction,
            _ => {
                return Err(pyo3::exceptions::PyValueError::new_err(
                    "grasp mode must be 'weld' or 'friction'",
                ));
            }
        };
        self.inner.set_grasp_mode(mode);
        Ok(())
    }

    #[pyo3(signature = (
        left_wheel_velocity_rad_s=0.0,
        right_wheel_velocity_rad_s=0.0,
        shoulder_velocity_rad_s=0.0,
        elbow_velocity_rad_s=0.0,
        gripper_velocity_rad_s=0.0,
        lift_velocity_m_s=0.0,
        gripper_velocity_m_s=0.0,
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
        gripper_velocity_m_s: f64,
    ) -> PyMmStepResult {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
                gripper_velocity_m_s,
                lift_velocity_m_s,
                ..MobileManipulatorAction::default()
            })
            .into()
    }

    /// Applies a full action object, preserving optional lift-arm targets.
    fn step_action(&mut self, action: PyMmAction) -> PyMmStepResult {
        self.inner.step(action.inner).into()
    }

    /// Steps the episode while holding absolute lift-arm joint targets.
    #[pyo3(signature = (lift_m, shoulder_rad, elbow_rad, gripper_velocity_rad_s=0.0, gripper_velocity_m_s=0.0))]
    fn step_hold_lift_joints(
        &mut self,
        lift_m: f64,
        shoulder_rad: f64,
        elbow_rad: f64,
        gripper_velocity_rad_s: f64,
        gripper_velocity_m_s: f64,
    ) -> PyMmStepResult {
        let mut action = MobileManipulatorAction::hold_lift_joints(MmLiftJointTarget {
            lift_m,
            shoulder_rad,
            elbow_rad,
        });
        action.gripper_velocity_rad_s = gripper_velocity_rad_s;
        action.gripper_velocity_m_s = gripper_velocity_m_s;
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

/// Scripted mobile clutter pick-place policy (matches Rust E2E tests).
#[pyclass(name = "IkMobileClutterPickPlacePolicy")]
struct PyIkMobileClutterPickPlacePolicy {
    inner: IkMobileClutterPickPlacePolicy,
}

#[pymethods]
impl PyIkMobileClutterPickPlacePolicy {
    #[new]
    fn new() -> Self {
        Self {
            inner: IkMobileClutterPickPlacePolicy::new(),
        }
    }

    /// Total scripted steps (settle → pick drive → retreat → carry → release).
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

/// Scripted weld-free mobile-lift friction pick-and-place policy.
#[pyclass(name = "IkMobileLiftPickPlacePolicy")]
struct PyIkMobileLiftPickPlacePolicy {
    inner: IkMobileLiftPickPlacePolicy,
}

fn mobile_lift_failure_name(failure: MobileLiftFailureClass) -> &'static str {
    match failure {
        MobileLiftFailureClass::None => "none",
        MobileLiftFailureClass::NavigateTimeout => "navigate_timeout",
        MobileLiftFailureClass::ApproachTimeout => "approach_timeout",
        MobileLiftFailureClass::PickupAlignmentTimeout => "pickup_alignment_timeout",
        MobileLiftFailureClass::GraspTimeout => "grasp_timeout",
        MobileLiftFailureClass::GraspSlip => "grasp_slip",
        MobileLiftFailureClass::LiftClearanceTimeout => "lift_clearance_timeout",
        MobileLiftFailureClass::TransportTimeout => "transport_timeout",
        MobileLiftFailureClass::LowerTimeout => "lower_timeout",
        MobileLiftFailureClass::ReleaseTimeout => "release_timeout",
    }
}

#[pymethods]
impl PyIkMobileLiftPickPlacePolicy {
    #[new]
    fn new() -> Self {
        Self {
            inner: IkMobileLiftPickPlacePolicy::new(),
        }
    }

    /// Maximum scripted actions through release.
    fn total_steps(&self) -> u64 {
        self.inner.total_steps()
    }

    /// Current scripted phase as a stable Rust enum debug name.
    fn phase(&self) -> String {
        format!("{:?}", self.inner.phase())
    }

    /// Deterministic failure category for the current observation.
    fn failure_class(&self, observation: PyMmObservation) -> &'static str {
        mobile_lift_failure_name(self.inner.failure_class(&observation.inner))
    }

    /// Returns a full action object, including the absolute lift-arm target.
    fn act(&mut self, observation: PyMmObservation) -> PyMmAction {
        self.inner.act(&observation.inner).into()
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
    /// Each action is `(left_wheel, right_wheel, shoulder, elbow, gripper)` in
    /// rad/s (or meters/s for a linear gripper).
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

    /// Steps all environments with full action objects, including lift targets.
    fn step_actions(
        &mut self,
        actions: Vec<PyMmAction>,
    ) -> PyResult<(Vec<PyMmObservation>, Vec<bool>)> {
        if actions.len() != self.inner.num_envs() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "expected {} actions, got {}",
                self.inner.num_envs(),
                actions.len()
            )));
        }
        let actions: Vec<MobileManipulatorAction> =
            actions.into_iter().map(|action| action.inner).collect();
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

    /// Selects the grasp mode for every environment after reset.
    fn set_grasp_mode(&mut self, mode: &str) -> PyResult<()> {
        let mode = match mode {
            "weld" => GraspMode::Weld,
            "friction" => GraspMode::Friction,
            _ => {
                return Err(pyo3::exceptions::PyValueError::new_err(
                    "grasp mode must be 'weld' or 'friction'",
                ));
            }
        };
        self.inner.set_grasp_mode(mode);
        Ok(())
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
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("TASK_SPEC_SCHEMA_VERSION", rne_ai::TASK_SPEC_SCHEMA_VERSION)?;
    m.add("TASK_SPEC_KIND", rne_ai::TASK_SPEC_KIND)?;
    m.add_function(wrap_pyfunction!(canonical_task_spec_json, m)?)?;
    m.add_function(wrap_pyfunction!(py_derive_episode_seed, m)?)?;
    m.add_class::<PyDiffDriveSim>()?;
    m.add_class::<PyDiffDriveEpisode>()?;
    m.add_class::<PyObservation>()?;
    m.add_class::<PyStepResult>()?;
    m.add_class::<PyUnitreeGo2GaitEpisode>()?;
    m.add_class::<PyUnitreeGo2StepResult>()?;
    m.add_class::<PyPortableUnitreeGo2Batch>()?;
    m.add_class::<PyPortableUnitreeGo2BatchStep>()?;
    m.add_class::<PyMmLiftJointTarget>()?;
    m.add_class::<PyMmLiftGripperTarget>()?;
    m.add_class::<PyMmAction>()?;
    m.add_class::<PyMmLiftKinematics>()?;
    m.add_class::<PyMobileManipulatorSim>()?;
    m.add_class::<PyMobileManipulatorEpisode>()?;
    m.add_class::<PyIkClutterPickPlacePolicy>()?;
    m.add_class::<PyIkMobileClutterPickPlacePolicy>()?;
    m.add_class::<PyIkMobileLiftPickPlacePolicy>()?;
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
        Python::initialize();
        Python::attach(|py| {
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
    fn python_task_spec_api_matches_rust_schema_and_seed_contract() {
        let env = PyUnitreeGo2GaitEpisode::new(123, 6602).unwrap();
        let json = env.task_spec_json().unwrap();
        let canonical = canonical_task_spec_json(&json).unwrap();
        let spec: TaskSpec = serde_json::from_str(&canonical).unwrap();
        assert_eq!(spec, unitree_go2_task_spec(123));
        assert_eq!(
            py_derive_episode_seed(42, 2, 1),
            rne_ai::derive_episode_seed(42, 2, 1)
        );
    }

    #[test]
    fn python_portable_batch_exposes_lane_metadata_and_restores_checkpoint() {
        let mut batch = PyPortableUnitreeGo2Batch::new(1, 4, 42, false).unwrap();
        let reset = batch.reset();
        assert_eq!(reset.lane_ids, vec![0]);
        assert_eq!(
            reset.episode_seeds,
            vec![Some(rne_ai::derive_episode_seed(42, 0, 0))]
        );
        let action = vec![(0.12, 0.16, 0.0, 0.0, 0.0)];
        batch.step(action.clone()).unwrap();
        let checkpoint = batch.checkpoint_json().unwrap();
        let expected = batch.step(action.clone()).unwrap();

        let mut restored = PyPortableUnitreeGo2Batch::new(1, 4, 42, false).unwrap();
        restored.restore_checkpoint_json(&checkpoint).unwrap();
        let actual = restored.step(action).unwrap();
        assert_eq!(actual.observations, expected.observations);
        assert_eq!(actual.rewards, expected.rewards);
        assert_eq!(actual.terminated, expected.terminated);
        assert_eq!(actual.truncated, expected.truncated);
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
    fn python_go2_gait_api_returns_finite_observation_and_reward() {
        let mut env = PyUnitreeGo2GaitEpisode::new(8, 6602).unwrap();
        let reset = env.reset();
        assert_eq!(reset.observation.len(), 21);
        assert!(reset.observation.iter().all(|value| value.is_finite()));
        let step = env.step(0.12, 0.16, 0.0, 0.0, 0.0);
        assert_eq!(step.observation.len(), 21);
        assert!(step.reward.is_finite());
        assert_eq!(env.step_in_episode(), 1);
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
    fn python_mobile_lift_mode_exposes_linear_gripper_observation() {
        let mut sim = PyMobileManipulatorSim::new("mm_mobile_lift").unwrap();
        let observation = sim.reset();
        assert!(observation.lift_position_m().is_finite());
        assert!(observation.gripper_position_m().is_finite());
        assert_eq!(observation.gripper_position(), 0.0);
        let stepped = sim.step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01);
        assert!(stepped.wrist_yaw_position().is_finite());
        assert!(stepped.wrist_target_depth_m().is_finite());
        assert!(!sim.is_grasping());
    }

    #[test]
    fn python_mobile_lift_policy_action_completes_friction_place() {
        let mut episode = PyMobileManipulatorEpisode::new("mobile_lift_place").unwrap();
        let mut step = episode.reset();
        episode.set_grasp_mode("friction").unwrap();
        let mut policy = PyIkMobileLiftPickPlacePolicy::new();
        let mut native_policy = IkMobileLiftPickPlacePolicy::new();
        let native_action = native_policy.act(&step.observation().inner);
        let python_action = policy.act(step.observation());
        assert_eq!(python_action.inner, native_action);
        step = episode.step_action(python_action);
        for _ in 0..policy.total_steps() {
            let action = policy.act(step.observation());
            step = episode.step_action(action);
            if step.done() {
                break;
            }
        }
        assert!(
            step.terminated(),
            "Python lift policy should place the cube: phase={} failure={} steps={} grasping={} target=({:.3},{:.3},{:.3})",
            policy.phase(),
            policy.failure_class(step.observation()),
            episode.step_in_episode(),
            episode.is_grasping(),
            step.observation().target_dx(),
            step.observation().target_dy(),
            step.observation().target_dz(),
        );
        assert_eq!(policy.failure_class(step.observation()), "none");
    }

    #[test]
    fn python_vectorized_mobile_lift_accepts_full_actions() {
        let mut env = PyVectorizedMobileManipulatorEnv::new("mobile_lift_place", 2).unwrap();
        let observations = env.reset();
        assert_eq!(observations.len(), 2);
        env.set_grasp_mode("friction").unwrap();
        let action = PyMmAction {
            inner: MobileManipulatorAction {
                lift_joint_target: Some(MmLiftJointTarget {
                    lift_m: 0.48,
                    shoulder_rad: 0.0,
                    elbow_rad: 0.0,
                }),
                gripper_velocity_m_s: -0.02,
                ..MobileManipulatorAction::default()
            },
        };
        let (next, done) = env.step_actions(vec![action, action]).unwrap();
        assert_eq!(next.len(), 2);
        assert_eq!(done.len(), 2);
        assert!(next
            .iter()
            .all(|observation| observation.lift_position_m().is_finite()));
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
