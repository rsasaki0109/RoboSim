//! Discrete ZMP preview control for the Linear Inverted Pendulum Model.
//!
//! The controller follows the standard Kajita formulation. The LIPM is written
//! as a jerk-input discrete system
//!
//! ```text
//! x_{k+1} = A x_k + B u_k,   p_k = C x_k
//! ```
//!
//! with state `x = [com, com_vel, com_acc]`, input `u = com_jerk`, and output
//! `p = com - (h/g) com_acc`, the Zero Moment Point. The optimal input is
//!
//! ```text
//! u_k = -K x_k + sum_i f_i p_ref(k + i)
//! ```
//!
//! where `K` is the steady-state LQ feedback from the discrete algebraic
//! Riccati equation and `f_i` are the preview gains. Both are computed once per
//! controller with a deterministic fixed iteration.

use crate::error::LeggedError;
use crate::lipm::LimpParams;

type State = [f64; 3];
type Mat3 = [[f64; 3]; 3];

/// Feedback and preview gains of a [`ZmpPreviewController`].
#[derive(Clone, Debug, PartialEq)]
pub struct PreviewGains {
    /// State feedback `K` multiplying `[com, com_vel, com_acc]`.
    pub feedback: State,
    /// Preview gains `f_1 .. f_preview_steps` multiplying future ZMP references.
    pub preview: Vec<f64>,
}

/// One-dimensional trajectory produced by preview control.
#[derive(Clone, Debug, PartialEq)]
pub struct PreviewTrajectory {
    /// Center-of-mass positions in meters.
    pub com_m: Vec<f64>,
    /// Zero Moment Point realized by the trajectory, in meters.
    pub zmp_m: Vec<f64>,
    /// Full states `[com, com_vel, com_acc]` per sample.
    pub states: Vec<State>,
}

/// Discrete ZMP preview controller for a constant-height LIPM.
#[derive(Clone, Debug)]
pub struct ZmpPreviewController {
    params: LimpParams,
    sample_time_s: f64,
    preview_steps: usize,
    gains: PreviewGains,
    a: Mat3,
    b: State,
    c: State,
}

impl ZmpPreviewController {
    /// Builds a controller and solves its Riccati and preview gains.
    ///
    /// `state_weight` weights the ZMP tracking error and `control_weight`
    /// weights the jerk input.
    pub fn new(
        params: LimpParams,
        sample_time_s: f64,
        state_weight: f64,
        control_weight: f64,
        preview_steps: usize,
    ) -> Result<Self, LeggedError> {
        if !params.is_valid() {
            return Err(LeggedError::InvalidLimp {
                com_height_m: params.com_height_m,
                gravity_m_s2: params.gravity_m_s2,
            });
        }
        if !sample_time_s.is_finite() || sample_time_s <= 0.0 {
            return Err(LeggedError::InvalidRequest(
                "sample time must be positive and finite",
            ));
        }
        if !state_weight.is_finite() || state_weight <= 0.0 {
            return Err(LeggedError::InvalidRequest(
                "state weight must be positive and finite",
            ));
        }
        if !control_weight.is_finite() || control_weight <= 0.0 {
            return Err(LeggedError::InvalidRequest(
                "control weight must be positive and finite",
            ));
        }
        if preview_steps == 0 {
            return Err(LeggedError::EmptyPreview);
        }

        let dt = sample_time_s;
        let a = [[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]];
        let b = [dt * dt * dt / 6.0, 0.5 * dt * dt, dt];
        let omega_squared = params.gravity_m_s2 / params.com_height_m;
        let c = [1.0, 0.0, -1.0 / omega_squared];
        let gains = solve_gains(&a, &b, &c, state_weight, control_weight, preview_steps);

        Ok(Self {
            params,
            sample_time_s,
            preview_steps,
            gains,
            a,
            b,
            c,
        })
    }

    /// Sample time in seconds.
    pub fn sample_time_s(&self) -> f64 {
        self.sample_time_s
    }

    /// Number of future ZMP samples used by the preview.
    pub fn preview_steps(&self) -> usize {
        self.preview_steps
    }

    /// The controller gains.
    pub fn gains(&self) -> &PreviewGains {
        &self.gains
    }

    /// The LIPM parameters.
    pub fn params(&self) -> &LimpParams {
        &self.params
    }

    /// Runs the controller for one axis given an initial state and a ZMP
    /// reference sampled at [`Self::sample_time_s`].
    ///
    /// References past the end of the slice stay constant at the last value, so
    /// a plan can settle without special-casing.
    pub fn track_axis(&self, initial_state: State, reference_zmp_m: &[f64]) -> PreviewTrajectory {
        let sample_count = reference_zmp_m.len();
        let mut com_m = Vec::with_capacity(sample_count);
        let mut zmp_m = Vec::with_capacity(sample_count);
        let mut states = Vec::with_capacity(sample_count);
        if sample_count == 0 {
            return PreviewTrajectory {
                com_m,
                zmp_m,
                states,
            };
        }

        let mut state = initial_state;
        for index in 0..sample_count {
            com_m.push(state[0]);
            zmp_m.push(dot(&self.c, &state));
            states.push(state);

            let mut input = -dot(&self.gains.feedback, &state);
            for preview in 0..self.preview_steps {
                let target = (index + 1 + preview).min(sample_count - 1);
                input += self.gains.preview[preview] * reference_zmp_m[target];
            }
            state = ax_plus_bu(&self.a, &self.b, &state, input);
        }

        PreviewTrajectory {
            com_m,
            zmp_m,
            states,
        }
    }
}

fn solve_gains(
    a: &Mat3,
    b: &State,
    c: &State,
    state_weight: f64,
    control_weight: f64,
    preview_steps: usize,
) -> PreviewGains {
    let at = transpose(a);
    let cc = outer(c, c);
    let mut p = [[0.0; 3]; 3];
    const MAX_ITERATIONS: usize = 200_000;
    const TOLERANCE: f64 = 1.0e-15;
    for _ in 0..MAX_ITERATIONS {
        let pb = mul_vec(&p, b);
        let denominator = control_weight + dot(b, &pb);
        let atpa = mul(&at, &mul(&p, a));
        let atpb = mul_vec(&at, &pb);
        let correction = scale_mat(&outer(&atpb, &atpb), 1.0 / denominator);
        let next = add(&sub(&atpa, &correction), &scale_mat(&cc, state_weight));
        let delta = max_abs_diff(&next, &p);
        p = next;
        if delta < TOLERANCE {
            break;
        }
    }

    let pb = mul_vec(&p, b);
    let denominator = control_weight + dot(b, &pb);
    let pa = mul(&p, a);
    let feedback = [
        dot(b, &[pa[0][0], pa[1][0], pa[2][0]]) / denominator,
        dot(b, &[pa[0][1], pa[1][1], pa[2][1]]) / denominator,
        dot(b, &[pa[0][2], pa[1][2], pa[2][2]]) / denominator,
    ];

    // M = A^T - A^T P B * B^T / (R + B^T P B)
    let atpb = mul_vec(&at, &pb);
    let m = sub(&at, &scale_mat(&outer(&atpb, b), 1.0 / denominator));

    // f_i = (R + B^T P B)^-1 * B^T M^{i-1} C Q
    let mut preview = Vec::with_capacity(preview_steps);
    let mut vector = scale_vec(c, state_weight);
    for _ in 0..preview_steps {
        preview.push(dot(b, &vector) / denominator);
        vector = mul_vec(&m, &vector);
    }

    PreviewGains { feedback, preview }
}

fn ax_plus_bu(a: &Mat3, b: &State, state: &State, input: f64) -> State {
    let mut next = mul_vec(a, state);
    for (value, coefficient) in next.iter_mut().zip(b) {
        *value += coefficient * input;
    }
    next
}

fn dot(a: &State, b: &State) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn mul_vec(m: &Mat3, v: &State) -> State {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, cell) in out_row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

fn transpose(m: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = m[column][row];
        }
    }
    out
}

fn add(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = *a;
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] += b[row][column];
        }
    }
    out
}

fn sub(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = *a;
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] -= b[row][column];
        }
    }
    out
}

fn scale_mat(m: &Mat3, factor: f64) -> Mat3 {
    let mut out = *m;
    for row in out.iter_mut() {
        for value in row.iter_mut() {
            *value *= factor;
        }
    }
    out
}

fn scale_vec(v: &State, factor: f64) -> State {
    [v[0] * factor, v[1] * factor, v[2] * factor]
}

fn outer(a: &State, b: &State) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = a[row] * b[column];
        }
    }
    out
}

fn max_abs_diff(a: &Mat3, b: &Mat3) -> f64 {
    let mut maximum = 0.0_f64;
    for row in 0..3 {
        for column in 0..3 {
            maximum = maximum.max((a[row][column] - b[row][column]).abs());
        }
    }
    maximum
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn controller() -> ZmpPreviewController {
        ZmpPreviewController::new(LimpParams::new(0.3, 9.81), 0.005, 1.0, 1.0e-4, 160)
            .expect("controller")
    }

    #[test]
    fn rejects_invalid_parameters() {
        assert!(
            ZmpPreviewController::new(LimpParams::new(0.0, 9.81), 0.005, 1.0, 1.0e-4, 10).is_err()
        );
        assert!(
            ZmpPreviewController::new(LimpParams::new(0.3, 9.81), 0.0, 1.0, 1.0e-4, 10).is_err()
        );
        assert!(
            ZmpPreviewController::new(LimpParams::new(0.3, 9.81), 0.005, 1.0, 1.0e-4, 0).is_err()
        );
    }

    #[test]
    fn constant_reference_is_tracked_and_stabilized() {
        let controller = controller();
        let reference = vec![0.0; 4_000];
        // Start 5 cm away from the reference with zero velocity and acceleration.
        let trajectory = controller.track_axis([0.05, 0.0, 0.0], &reference);
        assert_eq!(trajectory.states.len(), reference.len());
        let last = trajectory.com_m.len() - 1;
        assert_relative_eq!(trajectory.zmp_m[last], 0.0, epsilon = 5.0e-3);
        assert!(trajectory.com_m[last].abs() < 2.0e-2);
        assert!(trajectory
            .states
            .iter()
            .all(|state| state.iter().all(|value| value.is_finite())));
    }

    #[test]
    fn tracking_is_linear_and_deterministic() {
        let controller = controller();
        let reference: Vec<f64> = (0..600)
            .map(|index| 0.05 * (index as f64 * 0.01).sin())
            .collect();
        let first = controller.track_axis([0.0, 0.0, 0.0], &reference);
        let second = controller.track_axis([0.0, 0.0, 0.0], &reference);
        assert_eq!(first, second);
    }

    #[test]
    fn empty_reference_produces_empty_trajectory() {
        let controller = controller();
        let trajectory = controller.track_axis([0.0, 0.0, 0.0], &[]);
        assert!(trajectory.states.is_empty());
    }
}
