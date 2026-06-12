//! Python bindings for Robot Native Engine.

mod sim;

use pyo3::prelude::*;
use rne_ai::{DiffDriveEpisodeConfig, Episode};
use sim::{DiffDriveObservation, DiffDriveSim};

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

/// Robot Native Engine Python module.
#[pymodule]
fn rne_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyDiffDriveSim>()?;
    m.add_class::<PyDiffDriveEpisode>()?;
    m.add_class::<PyObservation>()?;
    m.add_class::<PyStepResult>()?;
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
}
