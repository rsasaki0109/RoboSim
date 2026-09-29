//! Convex model-predictive control of the ground reactions, in the form the
//! MIT Cheetah 3 controller introduced (Di Carlo et al., IROS 2018).
//!
//! The trunk is a single rigid body. Over a short horizon its state follows
//! linear dynamics in the ground reactions at the feet that the gait schedule
//! says are planted; the reactions are chosen by a quadratic program that
//! tracks a reference state inside the friction pyramid. Only the first
//! step's reactions are applied, and the problem is solved again at the next
//! update.
//!
//! The world is y-up. Orientation is a small rotation vector relative to the
//! settled stance, which is accurate for the recovery the controller is for:
//! it keeps the trunk near level.

use rne_math::Vec3;

/// Number of state entries: rotation, position, angular velocity, velocity,
/// and a constant 1 that carries gravity.
const STATES: usize = 13;
/// Ground reactions are solved in units of this many newtons, which keeps
/// the quadratic program well scaled.
const FORCE_SCALE_N: f64 = 100.0;

/// Fixed parameters of the prediction and the cost.
pub(crate) struct MpcParams {
    pub(crate) dt_s: f64,
    pub(crate) horizon: usize,
    pub(crate) mass_kg: f64,
    /// World-frame inertia about the center of mass, diagonal (x, y, z).
    pub(crate) inertia_kg_m2: Vec3,
    pub(crate) friction: f64,
    pub(crate) max_normal_n: f64,
    /// State weights for rotation, position, angular velocity, velocity.
    pub(crate) state_weights: [f64; 12],
    pub(crate) force_weight: f64,
}

/// The measured state and what the horizon looks like.
pub(crate) struct MpcProblem {
    pub(crate) rotation: Vec3,
    pub(crate) position_m: Vec3,
    pub(crate) angular_velocity: Vec3,
    pub(crate) velocity_m_s: Vec3,
    pub(crate) reference_position_m: Vec3,
    pub(crate) reference_velocity_m_s: Vec3,
    /// For each horizon step, the world position of each foot that is
    /// planted during it, or `None` for a foot in swing.
    pub(crate) contacts: Vec<[Option<Vec3>; 4]>,
}

/// Solves the horizon and returns the ground reaction on the robot at each
/// foot for the first step (zero for a foot in swing).
pub(crate) fn solve_mpc(params: &MpcParams, problem: &MpcProblem) -> [Vec3; 4] {
    let horizon = params.horizon;
    // Variables: three force components for every (step, planted foot).
    let mut variables: Vec<(usize, usize)> = Vec::new();
    for (k, feet) in problem.contacts.iter().enumerate().take(horizon) {
        for (foot, contact) in feet.iter().enumerate() {
            if contact.is_some() {
                variables.push((k, foot));
            }
        }
    }
    if variables.is_empty() {
        return [Vec3::ZERO; 4];
    }
    let n = variables.len() * 3;

    // x_{k+1} = A x_k + B_k u_k, forward Euler of the rigid-body dynamics.
    let dt = params.dt_s;
    let mut a = Matrix::identity(STATES);
    for axis in 0..3 {
        a.set(axis, 6 + axis, dt); // rotation from angular velocity
        a.set(3 + axis, 9 + axis, dt); // position from velocity
    }
    a.set(10, 12, -9.81 * dt); // gravity on the vertical velocity
    let inverse_inertia = Vec3::new(
        1.0 / params.inertia_kg_m2.x,
        1.0 / params.inertia_kg_m2.y,
        1.0 / params.inertia_kg_m2.z,
    );
    // Column block of B_k for one planted foot and one force axis.
    let b_column = |lever: Vec3, axis: Vec3| -> [f64; STATES] {
        let mut column = [0.0; STATES];
        let moment = lever.cross(axis);
        column[6] = moment.x * inverse_inertia.x * dt * FORCE_SCALE_N;
        column[7] = moment.y * inverse_inertia.y * dt * FORCE_SCALE_N;
        column[8] = moment.z * inverse_inertia.z * dt * FORCE_SCALE_N;
        column[9] = axis.x / params.mass_kg * dt * FORCE_SCALE_N;
        column[10] = axis.y / params.mass_kg * dt * FORCE_SCALE_N;
        column[11] = axis.z / params.mass_kg * dt * FORCE_SCALE_N;
        column
    };

    let x0 = {
        let mut x = [0.0; STATES];
        for axis in 0..3 {
            x[axis] = problem.rotation[axis];
            x[3 + axis] = problem.position_m[axis];
            x[6 + axis] = problem.angular_velocity[axis];
            x[9 + axis] = problem.velocity_m_s[axis];
        }
        x[12] = 1.0;
        x
    };
    let mut reference = [0.0; STATES];
    for axis in 0..3 {
        reference[3 + axis] = problem.reference_position_m[axis];
        reference[9 + axis] = problem.reference_velocity_m_s[axis];
    }
    reference[12] = 1.0;

    // Condensed prediction: X = Aqp x0 + Bqp U, rows for steps 1..=N.
    let rows = STATES * horizon;
    let mut free = vec![0.0; rows];
    let mut powers = vec![Matrix::identity(STATES)];
    for k in 1..=horizon {
        let next = a.mul(&powers[k - 1]);
        powers.push(next);
    }
    for k in 0..horizon {
        let state = powers[k + 1].mul_vec(&x0);
        free[k * STATES..(k + 1) * STATES].copy_from_slice(&state);
    }
    let mut bqp = Matrix::zeros(rows, n);
    for (index, &(j, foot)) in variables.iter().enumerate() {
        let lever = problem.contacts[j][foot].expect("planted foot") - problem.position_m;
        for (component, axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
            let column = b_column(lever, axis);
            // The force at step j reaches every later state through A.
            for k in j..horizon {
                let propagated = powers[k - j].mul_vec(&column);
                for s in 0..STATES {
                    bqp.set(k * STATES + s, index * 3 + component, propagated[s]);
                }
            }
        }
    }

    // Cost: (X - Xref)^T Q (X - Xref) + U^T R U.
    let weight = |s: usize| if s < 12 { params.state_weights[s] } else { 0.0 };
    let mut hessian = Matrix::zeros(n, n);
    let mut gradient = vec![0.0; n];
    for p in 0..n {
        for q in p..n {
            let mut sum = 0.0;
            for r in 0..rows {
                let w = weight(r % STATES);
                if w != 0.0 {
                    sum += bqp.get(r, p) * w * bqp.get(r, q);
                }
            }
            hessian.set(p, q, sum);
            hessian.set(q, p, sum);
        }
        let mut sum = 0.0;
        for r in 0..rows {
            let w = weight(r % STATES);
            if w != 0.0 {
                sum += bqp.get(r, p) * w * (free[r] - reference[r % STATES]);
            }
        }
        gradient[p] = sum;
        let r_weight = params.force_weight * FORCE_SCALE_N * FORCE_SCALE_N;
        hessian.set(p, p, hessian.get(p, p) + r_weight);
    }

    // Constraints per planted foot: 0 <= fy <= max, and the friction pyramid
    // |fx| <= mu fy, |fz| <= mu fy.
    let mu = params.friction;
    let mut constraint = Matrix::zeros(5 * variables.len(), n);
    let mut lower = Vec::with_capacity(5 * variables.len());
    let mut upper = Vec::with_capacity(5 * variables.len());
    for index in 0..variables.len() {
        let (fx, fy, fz) = (index * 3, index * 3 + 1, index * 3 + 2);
        let row = index * 5;
        constraint.set(row, fy, 1.0);
        lower.push(0.0);
        upper.push(params.max_normal_n / FORCE_SCALE_N);
        for (offset, (column, sign)) in [(fx, 1.0), (fx, -1.0), (fz, 1.0), (fz, -1.0)]
            .into_iter()
            .enumerate()
        {
            constraint.set(row + 1 + offset, column, sign);
            constraint.set(row + 1 + offset, fy, -mu);
            lower.push(f64::NEG_INFINITY);
            upper.push(0.0);
        }
    }

    let solution = admm(&hessian, &gradient, &constraint, &lower, &upper);
    let mut reactions = [Vec3::ZERO; 4];
    for (index, &(k, foot)) in variables.iter().enumerate() {
        if k == 0 {
            reactions[foot] = Vec3::new(
                solution[index * 3],
                solution[index * 3 + 1],
                solution[index * 3 + 2],
            ) * FORCE_SCALE_N;
        }
    }
    reactions
}

/// Operator-splitting solver for `min 1/2 x^T P x + q^T x` subject to
/// `l <= C x <= u`, the iteration OSQP uses with a fixed step size.
fn admm(p: &Matrix, q: &[f64], c: &Matrix, l: &[f64], u: &[f64]) -> Vec<f64> {
    const RHO: f64 = 1.0;
    const SIGMA: f64 = 1.0e-6;
    const ALPHA: f64 = 1.6;
    const ITERATIONS: usize = 400;
    let n = q.len();
    let m = l.len();
    let mut kkt = p.clone();
    for i in 0..n {
        kkt.set(i, i, kkt.get(i, i) + SIGMA);
    }
    for r in 0..m {
        let nonzero: Vec<(usize, f64)> = (0..n)
            .filter_map(|i| {
                let value = c.get(r, i);
                (value != 0.0).then_some((i, value))
            })
            .collect();
        for &(i, vi) in &nonzero {
            for &(j, vj) in &nonzero {
                kkt.set(i, j, kkt.get(i, j) + RHO * vi * vj);
            }
        }
    }
    let factor = kkt.cholesky();
    let sparse: Vec<Vec<(usize, f64)>> = (0..m)
        .map(|r| {
            (0..n)
                .filter_map(|i| {
                    let value = c.get(r, i);
                    (value != 0.0).then_some((i, value))
                })
                .collect()
        })
        .collect();
    let mut x = vec![0.0; n];
    let mut z = vec![0.0; m];
    let mut y = vec![0.0; m];
    for _ in 0..ITERATIONS {
        let mut rhs: Vec<f64> = (0..n).map(|i| SIGMA * x[i] - q[i]).collect();
        for (r, row) in sparse.iter().enumerate() {
            let weight = RHO * z[r] - y[r];
            for &(i, entry) in row {
                rhs[i] += entry * weight;
            }
        }
        let x_tilde = cholesky_solve(&factor, &rhs);
        for i in 0..n {
            x[i] = ALPHA * x_tilde[i] + (1.0 - ALPHA) * x[i];
        }
        for (r, row) in sparse.iter().enumerate() {
            let z_tilde: f64 = row.iter().map(|&(i, entry)| entry * x_tilde[i]).sum();
            let relaxed = ALPHA * z_tilde + (1.0 - ALPHA) * z[r];
            let z_next = (relaxed + y[r] / RHO).clamp(l[r], u[r]);
            y[r] += RHO * (relaxed - z_next);
            z[r] = z_next;
        }
    }
    // Return the feasible projection's primal point: the last iterate.
    x
}

#[derive(Clone)]
struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Matrix {
    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    fn identity(size: usize) -> Self {
        let mut matrix = Self::zeros(size, size);
        for i in 0..size {
            matrix.set(i, i, 1.0);
        }
        matrix
    }

    fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.cols + col]
    }

    fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row * self.cols + col] = value;
    }

    fn mul(&self, other: &Self) -> Self {
        let mut out = Self::zeros(self.rows, other.cols);
        for i in 0..self.rows {
            for k in 0..self.cols {
                let a = self.get(i, k);
                if a != 0.0 {
                    for j in 0..other.cols {
                        out.data[i * other.cols + j] += a * other.get(k, j);
                    }
                }
            }
        }
        out
    }

    fn mul_vec(&self, v: &[f64]) -> Vec<f64> {
        (0..self.rows)
            .map(|i| (0..self.cols).map(|j| self.get(i, j) * v[j]).sum())
            .collect()
    }

    /// Lower-triangular Cholesky factor of a symmetric positive-definite
    /// matrix.
    fn cholesky(&self) -> Self {
        let n = self.rows;
        let mut factor = Self::zeros(n, n);
        for i in 0..n {
            for j in 0..=i {
                let sum: f64 = (0..j).map(|k| factor.get(i, k) * factor.get(j, k)).sum();
                if i == j {
                    factor.set(i, j, (self.get(i, i) - sum).max(1.0e-12).sqrt());
                } else {
                    factor.set(i, j, (self.get(i, j) - sum) / factor.get(j, j));
                }
            }
        }
        factor
    }
}

fn cholesky_solve(factor: &Matrix, b: &[f64]) -> Vec<f64> {
    let n = b.len();
    let mut y = vec![0.0; n];
    for i in 0..n {
        let sum: f64 = (0..i).map(|k| factor.get(i, k) * y[k]).sum();
        y[i] = (b[i] - sum) / factor.get(i, i);
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let sum: f64 = (i + 1..n).map(|k| factor.get(k, i) * x[k]).sum();
        x[i] = (y[i] - sum) / factor.get(i, i);
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> MpcParams {
        MpcParams {
            dt_s: 0.02,
            horizon: 10,
            mass_kg: 16.1,
            inertia_kg_m2: Vec3::new(0.1, 0.3, 0.25),
            friction: 0.5,
            max_normal_n: 300.0,
            state_weights: [25.0, 10.0, 25.0, 1.0, 100.0, 1.0, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0],
            force_weight: 1.0e-5,
        }
    }

    fn standing(velocity: Vec3) -> MpcProblem {
        let feet = [
            Vec3::new(0.19, 0.0, -0.14),
            Vec3::new(0.19, 0.0, 0.14),
            Vec3::new(-0.19, 0.0, -0.14),
            Vec3::new(-0.19, 0.0, 0.14),
        ];
        MpcProblem {
            rotation: Vec3::ZERO,
            position_m: Vec3::new(0.0, 0.25, 0.0),
            angular_velocity: Vec3::ZERO,
            velocity_m_s: velocity,
            reference_position_m: Vec3::new(0.0, 0.25, 0.0),
            reference_velocity_m_s: Vec3::ZERO,
            contacts: vec![feet.map(Some); 10],
        }
    }

    fn moment_about(problem: &MpcProblem, reactions: &[Vec3; 4]) -> Vec3 {
        (0..4).fold(Vec3::ZERO, |sum, i| {
            sum + (problem.contacts[0][i].expect("planted") - problem.position_m).cross(reactions[i])
        })
    }

    #[test]
    fn a_tilted_trunk_is_turned_back() {
        for tilt in [Vec3::new(0.05, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.05), Vec3::new(-0.05, 0.0, 0.0)] {
            let mut problem = standing(Vec3::ZERO);
            problem.rotation = tilt;
            let reactions = solve_mpc(&params(), &problem);
            let moment = moment_about(&problem, &reactions);
            eprintln!("tilt {tilt:?} moment {moment:?} reactions {reactions:?}");
            assert!(moment.dot(tilt) < 0.0, "tilt {tilt:?} moment {moment:?}");
        }
    }

    #[test]
    fn a_rolling_trunk_is_damped() {
        let mut problem = standing(Vec3::ZERO);
        problem.angular_velocity = Vec3::new(1.0, 0.0, 0.0);
        let reactions = solve_mpc(&params(), &problem);
        let moment = moment_about(&problem, &reactions);
        eprintln!("moment {moment:?} reactions {reactions:?}");
        assert!(moment.x < 0.0);
    }

    #[test]
    fn a_resting_trunk_is_held_up_by_its_weight() {
        let reactions = solve_mpc(&params(), &standing(Vec3::ZERO));
        let total = reactions.iter().fold(Vec3::ZERO, |sum, f| sum + *f);
        assert!((total.y - 16.1 * 9.81).abs() < 3.0, "total {total:?}");
        assert!(total.x.abs() < 1.0 && total.z.abs() < 1.0, "total {total:?}");
    }

    #[test]
    fn a_sliding_trunk_is_braked_inside_the_friction_pyramid() {
        let reactions = solve_mpc(&params(), &standing(Vec3::new(0.0, 0.0, 1.0)));
        let total = reactions.iter().fold(Vec3::ZERO, |sum, f| sum + *f);
        assert!(total.z < -10.0, "no braking: {total:?}");
        for reaction in reactions {
            let horizontal = reaction.x.hypot(reaction.z);
            assert!(horizontal <= 0.5 * reaction.y * 1.05 + 1.0, "{reaction:?}");
        }
    }
}
