//! Python bindings for Robot Native Engine.

mod sim;

use pyo3::prelude::*;
use sim::{DiffDriveObservation, DiffDriveSim};

/// Observation returned after each simulation step.
#[pyclass(name = "Observation")]
#[derive(Clone, Copy)]
struct PyObservation {
    base_x_m: f64,
    base_y_m: f64,
    base_z_m: f64,
    imu_ay_m_s2: f64,
    lidar_points: usize,
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
    fn imu_ay(&self) -> f64 {
        self.imu_ay_m_s2
    }

    #[getter]
    fn lidar_points(&self) -> usize {
        self.lidar_points
    }

    fn __repr__(&self) -> String {
        format!(
            "Observation(base_x={:.3}, base_y={:.3}, imu_ay={:.3})",
            self.base_x_m, self.base_y_m, self.imu_ay_m_s2
        )
    }
}

impl From<DiffDriveObservation> for PyObservation {
    fn from(value: DiffDriveObservation) -> Self {
        Self {
            base_x_m: value.base_x_m,
            base_y_m: value.base_y_m,
            base_z_m: value.base_z_m,
            imu_ay_m_s2: value.imu_ay_m_s2,
            lidar_points: value.lidar_points,
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

/// Robot Native Engine Python module.
#[pymodule]
fn rne_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyDiffDriveSim>()?;
    m.add_class::<PyObservation>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_sim_moves_forward() {
        let mut sim = DiffDriveSim::new();
        let mut final_x = 0.0;
        for _ in 0..180 {
            final_x = sim.step(6.0, 6.0).base_x_m;
        }
        assert!(final_x > 1.5);
    }
}
