//! Python bindings for Robot Native Engine.

mod sim;

use pyo3::prelude::*;
use rne_ai::{DiffDriveEpisodeConfig, Episode};
use sim::{
    DiffDriveObservation, DiffDriveSim, MobileManipulatorAction, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, MobileManipulatorObservation, MobileManipulatorSim,
};

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
    fn wrist_camera_pixels(&self) -> usize {
        self.inner.wrist_camera_pixels
    }

    #[getter]
    fn joint_state_count(&self) -> usize {
        self.inner.joint_state_count
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
    /// Creates a sim for the `"mm_minimal"` (default) or `"mm_mobile"` robot.
    #[new]
    #[pyo3(signature = (mode="mm_minimal"))]
    fn new(mode: &str) -> PyResult<Self> {
        let inner = match mode {
            "mm_minimal" => MobileManipulatorSim::new_mm_minimal(),
            "mm_mobile" => MobileManipulatorSim::new_mm_mobile(),
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "unknown mode '{other}', expected 'mm_minimal' or 'mm_mobile'"
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
    ))]
    fn step(
        &mut self,
        left_wheel_velocity_rad_s: f64,
        right_wheel_velocity_rad_s: f64,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
        gripper_velocity_rad_s: f64,
    ) -> PyMmObservation {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
            })
            .into()
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
    /// Creates an episode for the `"place"` (default), `"transport"`, or `"inspect"` task.
    #[new]
    #[pyo3(signature = (task="place"))]
    fn new(task: &str) -> PyResult<Self> {
        let config = match task {
            "place" => MobileManipulatorEpisodeConfig::place(),
            "transport" => MobileManipulatorEpisodeConfig::transport(),
            "inspect" => MobileManipulatorEpisodeConfig::inspect(),
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "unknown task '{other}', expected 'place', 'transport', or 'inspect'"
                )))
            }
        };
        Ok(Self {
            inner: MobileManipulatorEpisode::new(config),
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
    ))]
    fn step(
        &mut self,
        left_wheel_velocity_rad_s: f64,
        right_wheel_velocity_rad_s: f64,
        shoulder_velocity_rad_s: f64,
        elbow_velocity_rad_s: f64,
        gripper_velocity_rad_s: f64,
    ) -> PyMmStepResult {
        self.inner
            .step(MobileManipulatorAction {
                left_wheel_velocity_rad_s,
                right_wheel_velocity_rad_s,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                gripper_velocity_rad_s,
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

    #[getter]
    fn is_grasping(&self) -> bool {
        self.inner.simulation().is_grasping()
    }
}

/// Robot Native Engine Python module.
#[pymodule]
fn rne_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyDiffDriveSim>()?;
    m.add_class::<PyDiffDriveEpisode>()?;
    m.add_class::<PyObservation>()?;
    m.add_class::<PyStepResult>()?;
    m.add_class::<PyMobileManipulatorSim>()?;
    m.add_class::<PyMobileManipulatorEpisode>()?;
    m.add_class::<PyMmObservation>()?;
    m.add_class::<PyMmStepResult>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let mut env = MobileManipulatorEpisode::new(MobileManipulatorEpisodeConfig::place());
        let _ = env.reset();
        let close = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.5,
            ..MobileManipulatorAction::default()
        };
        let carry = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            shoulder_velocity_rad_s: 0.6,
            ..MobileManipulatorAction::default()
        };
        let hold = MobileManipulatorAction {
            gripper_velocity_rad_s: -2.0,
            ..MobileManipulatorAction::default()
        };
        let open = MobileManipulatorAction {
            gripper_velocity_rad_s: 3.0,
            ..MobileManipulatorAction::default()
        };

        for _ in 0..30 {
            env.step(close);
            if env.simulation().is_grasping() {
                break;
            }
        }
        for _ in 0..200 {
            env.step(carry);
        }
        for _ in 0..30 {
            env.step(hold);
        }
        for _ in 0..150 {
            if env.step(open).terminated {
                return;
            }
        }
        panic!("expected mobile manipulator place episode to terminate");
    }
}
