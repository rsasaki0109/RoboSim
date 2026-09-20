//! Dynamics algorithms for an [`ArticulatedModel`].
//!
//! All routines are pure functions of `(model, q, qd, qdd, tau)` and are
//! deterministic: the tree is traversed in topological order and no hash order
//! or wall-clock value participates.

use crate::model::{ArticulatedModel, DynamicsError};
use crate::spatial::{
    add6, cross_force, cross_motion, dot6, inverse_transform, mat6_add, mat6_mul, mat6_mul_vec,
    mat6_transpose_mul_vec, mat6_zero, motion_transform, scale6, transform_point, Mat6, SpatialVec,
};
use rne_math::{Quat, Vec3};
use rne_world::Transform3;

/// A dense row-major matrix with deterministic arithmetic.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseMatrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl DenseMatrix {
    /// Creates a zero matrix.
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    /// Number of rows.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Number of columns.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Reads an entry.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.cols + col]
    }

    /// Writes an entry.
    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row * self.cols + col] = value;
    }

    /// Row-major data.
    pub fn data(&self) -> &[f64] {
        &self.data
    }

    /// Multiplies the matrix by a vector.
    pub fn mul_vec(&self, vector: &[f64]) -> Vec<f64> {
        (0..self.rows)
            .map(|row| {
                (0..self.cols)
                    .map(|col| self.get(row, col) * vector[col])
                    .sum()
            })
            .collect()
    }

    /// Solves `self * x = rhs` by Gaussian elimination with partial pivoting.
    ///
    /// Returns `None` when the matrix is singular. Pivoting selects the largest
    /// magnitude entry with [`f64::total_cmp`], so the result is deterministic.
    pub fn solve(&self, rhs: &[f64]) -> Option<Vec<f64>> {
        let n = self.rows;
        let mut matrix: Vec<Vec<f64>> = (0..n)
            .map(|row| (0..n).map(|col| self.get(row, col)).collect())
            .collect();
        let mut solution = rhs.to_vec();
        for col in 0..n {
            let pivot =
                (col..n).max_by(|&a, &b| matrix[a][col].abs().total_cmp(&matrix[b][col].abs()))?;
            if matrix[pivot][col].abs() < 1.0e-12 {
                return None;
            }
            matrix.swap(col, pivot);
            solution.swap(col, pivot);
            let pivot_row = matrix[col].clone();
            let diagonal = pivot_row[col];
            for row in (col + 1)..n {
                let factor = matrix[row][col] / diagonal;
                for (target, source) in matrix[row].iter_mut().zip(&pivot_row).skip(col) {
                    *target -= factor * source;
                }
                solution[row] -= factor * solution[col];
            }
        }
        let mut result = vec![0.0; n];
        for row in (0..n).rev() {
            let known: f64 = (row + 1..n).map(|col| matrix[row][col] * result[col]).sum();
            result[row] = (solution[row] - known) / matrix[row][row];
        }
        Some(result)
    }
}

fn validate(model: &ArticulatedModel, values: &[f64], name: &str) -> Result<(), DynamicsError> {
    if values.len() != model.nv() {
        return Err(DynamicsError::DimensionMismatch {
            provided: values.len(),
            expected: model.nv(),
        });
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(DynamicsError::NonFiniteInput);
    }
    let _ = name;
    Ok(())
}

fn inertia_matrices(model: &ArticulatedModel) -> Vec<Mat6> {
    model
        .links
        .iter()
        .map(|link| link.inertia.matrix())
        .collect()
}

fn xup_transforms(model: &ArticulatedModel, transforms: &[Transform3]) -> Vec<Mat6> {
    let mut xup = vec![mat6_zero(); model.link_count()];
    for index in 1..model.link_count() {
        if let Some(parent) = model.links[index].parent {
            let child_in_parent =
                inverse_transform(&transforms[parent]).mul_transform(&transforms[index]);
            xup[index] = motion_transform(&inverse_transform(&child_in_parent));
        }
    }
    xup
}

/// Computes the joint-space mass matrix `M(q)`.
pub fn mass_matrix(model: &ArticulatedModel, q: &[f64]) -> Result<DenseMatrix, DynamicsError> {
    validate(model, q, "q")?;
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let xup = xup_transforms(model, kinematics.transforms());
    let mut composite = inertia_matrices(model);

    for index in (1..model.link_count()).rev() {
        if let Some(parent) = model.links[index].parent {
            let transformed = mat6_mul(
                &mat6_transpose(&xup[index]),
                &mat6_mul(&composite[index], &xup[index]),
            );
            composite[parent] = mat6_add(&composite[parent], &transformed);
        }
    }

    let nv = model.nv();
    let mut matrix = DenseMatrix::zeros(nv, nv);
    for dof in 0..nv {
        let link = model.dofs[dof].link;
        let column = model.dofs[dof].s;
        let force = mat6_mul_vec(&composite[link], &column);
        for &other in &model.link_dofs[link] {
            let value = dot6(&model.dofs[other].s, &force);
            matrix.set(dof, other, value);
            matrix.set(other, dof, value);
        }
        let mut current = link;
        let mut force = force;
        while let Some(parent) = model.links[current].parent {
            force = mat6_transpose_mul_vec(&xup[current], &force);
            current = parent;
            for &other in &model.link_dofs[current] {
                let value = dot6(&model.dofs[other].s, &force);
                matrix.set(dof, other, value);
                matrix.set(other, dof, value);
            }
        }
    }
    Ok(matrix)
}

/// Analytical derivative of the joint-space mass matrix with respect to `q`.
///
/// Entry `[k]` is the dense matrix `dM/dq_k`. The computation mirrors
/// [`mass_matrix`] exactly, propagating the analytic `d(xup)/dq` through the
/// composite-inertia recursion and then through the per-DoF assembly, so it
/// needs no finite differencing.
#[allow(clippy::needless_range_loop)]
pub fn mass_matrix_gradient(
    model: &ArticulatedModel,
    q: &[f64],
) -> Result<Vec<DenseMatrix>, DynamicsError> {
    validate(model, q, "q")?;
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let xup = xup_transforms(model, kinematics.transforms());
    let xup_gradient = xup_derivatives(model, q)?;
    let link_count = model.link_count();
    let nv = model.nv();

    let mut composite = inertia_matrices(model);
    let mut composite_gradient: Vec<Vec<Mat6>> = vec![vec![mat6_zero(); nv]; link_count];

    for index in (1..link_count).rev() {
        let Some(parent) = model.links[index].parent else {
            continue;
        };
        let x = xup[index];
        let c = composite[index];
        let transformed = mat6_mul(&mat6_transpose(&x), &mat6_mul(&c, &x));
        composite[parent] = mat6_add(&composite[parent], &transformed);
        for k in 0..nv {
            // d(X^T C X) = dX^T C X + X^T (dC X + C dX).
            let d_x = xup_gradient[index][k];
            let d_c = composite_gradient[index][k];
            let first = mat6_mul(&mat6_transpose(&d_x), &mat6_mul(&c, &x));
            let inner = mat6_add(&mat6_mul(&d_c, &x), &mat6_mul(&c, &d_x));
            let second = mat6_mul(&mat6_transpose(&x), &inner);
            composite_gradient[parent][k] =
                mat6_add(&composite_gradient[parent][k], &mat6_add(&first, &second));
        }
    }

    let mut gradient: Vec<DenseMatrix> = (0..nv).map(|_| DenseMatrix::zeros(nv, nv)).collect();
    for dof in 0..nv {
        let link = model.dofs[dof].link;
        let column = model.dofs[dof].s;
        // `force = composite[link] * S`; its derivative uses
        // `d(composite[link])/dq_k * S` because `S` does not depend on q.
        // `force` and `d_force[k]` are propagated together: the derivative of
        // `X^T f` needs both `f` and `d f`.
        let mut force = mat6_mul_vec(&composite[link], &column);
        let mut d_force: Vec<SpatialVec> = (0..nv)
            .map(|k| mat6_mul_vec(&composite_gradient[link][k], &column))
            .collect();
        for &other in &model.link_dofs[link] {
            for k in 0..nv {
                let value = dot6(&model.dofs[other].s, &d_force[k]);
                gradient[k].set(dof, other, value);
                gradient[k].set(other, dof, value);
            }
        }

        let mut current = link;
        while let Some(parent) = model.links[current].parent {
            let x = xup[current];
            let d_x = &xup_gradient[current];
            let next_force = mat6_transpose_mul_vec(&x, &force);
            let mut next_d_force: Vec<SpatialVec> = Vec::with_capacity(nv);
            for k in 0..nv {
                next_d_force.push(add6(
                    &mat6_transpose_mul_vec(&d_x[k], &force),
                    &mat6_transpose_mul_vec(&x, &d_force[k]),
                ));
            }
            current = parent;
            force = next_force;
            d_force = next_d_force;
            for &other in &model.link_dofs[current] {
                for k in 0..nv {
                    let value = dot6(&model.dofs[other].s, &d_force[k]);
                    gradient[k].set(dof, other, value);
                    gradient[k].set(other, dof, value);
                }
            }
        }
    }

    Ok(gradient)
}

/// Recursive Newton-Euler inverse dynamics.
///
/// Returns the generalized force that realizes `qdd` at state `(q, qd)`,
/// including gravity. For a floating base the first six entries are the base
/// wrench in the base body frame.
pub fn rnea(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
) -> Result<Vec<f64>, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;
    validate(model, qdd, "qdd")?;

    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let xup = xup_transforms(model, transforms);
    let inertia = inertia_matrices(model);
    let link_count = model.link_count();

    let base_rotation = crate::spatial::rotation_matrix(transforms[0].rotation);
    let gravity_body = crate::spatial::mat3_mul_vec(
        &crate::spatial::mat3_transpose(&base_rotation),
        model.gravity_m_s2,
    );

    let mut velocity: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut acceleration: Vec<SpatialVec> = vec![[0.0; 6]; link_count];

    if model.base_dof() == 6 {
        velocity[0] = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]];
        acceleration[0] = [qdd[0], qdd[1], qdd[2], qdd[3], qdd[4], qdd[5]];
        for index in 0..3 {
            acceleration[0][index] -= gravity_body[index];
        }
    } else {
        for index in 0..3 {
            acceleration[0][index] = -gravity_body[index];
        }
    }

    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let mut link_velocity = mat6_mul_vec(&xup[index], &velocity[parent]);
        let mut link_acceleration = mat6_mul_vec(&xup[index], &acceleration[parent]);
        if let Some(joint) = &model.links[index].joint {
            if let Some(dof) = joint.dof {
                let subspace = model.dofs[dof].s;
                let joint_velocity = scale6(&subspace, qd[dof]);
                let joint_acceleration = scale6(&subspace, qdd[dof]);
                link_velocity = add6(&link_velocity, &joint_velocity);
                let bias = cross_motion(&link_velocity, &joint_velocity);
                link_acceleration = add6(&add6(&link_acceleration, &joint_acceleration), &bias);
            }
        }
        velocity[index] = link_velocity;
        acceleration[index] = link_acceleration;
    }

    let mut transmitted: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut tau = vec![0.0; model.nv()];
    for index in (0..link_count).rev() {
        let momentum = mat6_mul_vec(&inertia[index], &velocity[index]);
        let force = add6(
            &add6(
                &mat6_mul_vec(&inertia[index], &acceleration[index]),
                &cross_force(&velocity[index], &momentum),
            ),
            &transmitted[index],
        );
        for &dof in &model.link_dofs[index] {
            tau[dof] = dot6(&model.dofs[dof].s, &force);
        }
        if let Some(parent) = model.links[index].parent {
            transmitted[parent] = add6(
                &transmitted[parent],
                &mat6_transpose_mul_vec(&xup[index], &force),
            );
        }
    }
    Ok(tau)
}

/// Integrates one configuration step `q += dt * qdot + 0.5 * dt^2 * qddot`.
///
/// For a floating base the generalized velocity `qd[..6]` is the body twist,
/// while `q[..6]` is the `(translation, roll-pitch-yaw)` chart. This function
/// maps the twist through `base_velocity_map` before integrating, so the chart
/// step is consistent with the dynamics. Treating the body twist as Euler rates
/// (the naive `q += qd * dt`) mishandles large rotations such as an aerial flip.
pub fn integrate_configuration(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    qdd: &[f64],
    dt: f64,
) -> Result<Vec<f64>, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;
    validate(model, qdd, "qdd")?;
    let mut next = q.to_vec();
    let base = model.base_dof();
    if base == 6 {
        let kinematics = model.kinematic.forward_kinematics(q)?;
        let base_rotation = kinematics.transforms()[0].rotation;
        let map = base_velocity_map(q, base_rotation);
        let mut twist = [0.0; 6];
        for (index, value) in twist.iter_mut().enumerate() {
            *value = qd[index] + 0.5 * qdd[index] * dt;
        }
        let chart_velocity = mat6_mul_vec(&map, &twist);
        for index in 0..6 {
            next[index] += chart_velocity[index] * dt;
        }
    }
    for index in base..model.nv() {
        next[index] += qd[index] * dt + 0.5 * qdd[index] * dt * dt;
    }
    Ok(next)
}

/// Gravity and velocity bias `C(q, qd) qd + g(q)`.
pub fn non_linear_effects(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<Vec<f64>, DynamicsError> {
    rnea(model, q, qd, &vec![0.0; model.nv()])
}

/// Analytical gradient of the nonlinear effects `h(q, qd) = C(q, qd) qd + g(q)`.
///
/// `with_respect_to_q[k]` is `dh/dq_k` and `with_respect_to_qd[k]` is
/// `dh/dqd_k`. Both are computed by differentiating the recursive Newton-Euler
/// pass analytically, mirroring [`non_linear_effects`].
#[allow(clippy::needless_range_loop)]
pub fn non_linear_effects_gradient(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<NonLinearEffectsGradient, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;

    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let xup = xup_transforms(model, transforms);
    let xup_gradient = xup_derivatives(model, q)?;
    let inertia = inertia_matrices(model);
    let link_count = model.link_count();
    let nv = model.nv();
    let base = model.base_dof();

    let base_rotation = crate::spatial::rotation_matrix(transforms[0].rotation);
    let gravity_body = crate::spatial::mat3_mul_vec(
        &crate::spatial::mat3_transpose(&base_rotation),
        model.gravity_m_s2,
    );

    // Forward velocity and acceleration, matching `rnea` with `qdd = 0`.
    let mut velocity: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut acceleration: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    if base == 6 {
        velocity[0] = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]];
        acceleration[0] = [0.0; 6];
        for index in 0..3 {
            acceleration[0][index] = -gravity_body[index];
        }
    } else {
        for index in 0..3 {
            acceleration[0][index] = -gravity_body[index];
        }
    }

    let mut velocity_dq: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];
    let mut velocity_dqd: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];
    let mut acceleration_dq: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];
    let mut acceleration_dqd: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];

    if base == 6 {
        for k in 0..6 {
            let mut basis = [0.0; 6];
            basis[k] = 1.0;
            velocity_dqd[0][k] = basis;
        }
        // Gravity is expressed in the base frame: `acceleration[0] = -R^T g`.
        // A base orientation step rotates the frame, so
        // `d(R^T g)/dq_k = -R^T (world_axis_k x g)` with the ZYX world axes.
        // Body-frame angular velocity columns of the ZYX chart, rotated to the
        // world frame. Unlike `R * X/Y/Z` these are the actual chart axes.
        let (roll, pitch) = (q[3], q[4]);
        let body_columns = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, roll.cos(), -roll.sin()),
            Vec3::new(
                -pitch.sin(),
                pitch.cos() * roll.sin(),
                pitch.cos() * roll.cos(),
            ),
        ];
        let world_axes = body_columns.map(|column| transforms[0].rotation * column);
        for (offset, world_axis) in world_axes.iter().enumerate() {
            let k = 3 + offset;
            let body_correction = crate::spatial::mat3_mul_vec(
                &crate::spatial::mat3_transpose(&base_rotation),
                world_axis.cross(model.gravity_m_s2),
            );
            for index in 0..3 {
                acceleration_dq[0][k][index] = body_correction[index];
            }
        }
    }

    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let joint_dof = model.links[index]
            .joint
            .as_ref()
            .and_then(|joint| joint.dof);
        let joint_velocity = joint_dof
            .map(|dof| scale6(&model.dofs[dof].s, qd[dof]))
            .unwrap_or([0.0; 6]);

        let mut link_velocity = mat6_mul_vec(&xup[index], &velocity[parent]);
        let mut link_acceleration = mat6_mul_vec(&xup[index], &acceleration[parent]);
        if joint_dof.is_some() {
            link_velocity = add6(&link_velocity, &joint_velocity);
            let bias = cross_motion(&link_velocity, &joint_velocity);
            link_acceleration = add6(&link_acceleration, &bias);
        }
        velocity[index] = link_velocity;
        acceleration[index] = link_acceleration;

        for k in 0..nv {
            let conveyed_v = mat6_mul_vec(&xup[index], &velocity_dq[parent][k]);
            let d_x_v = mat6_mul_vec(&xup_gradient[index][k], &velocity[parent]);
            let dv = add6(&conveyed_v, &d_x_v);
            if joint_dof == Some(k) {
                // d(joint_velocity)/dq_k = 0 (S is constant, qd fixed).
            }
            velocity_dq[index][k] = dv;

            let conveyed_vd = mat6_mul_vec(&xup[index], &velocity_dqd[parent][k]);
            let mut dvd = conveyed_vd;
            if joint_dof == Some(k) {
                dvd = add6(&dvd, &model.dofs[k].s);
            }
            velocity_dqd[index][k] = dvd;

            // Bias derivative w.r.t. q: joint velocity is constant in q.
            let bias_dq = cross_motion(&dv, &joint_velocity);
            let conveyed_a = mat6_mul_vec(&xup[index], &acceleration_dq[parent][k]);
            let d_x_a = mat6_mul_vec(&xup_gradient[index][k], &acceleration[parent]);
            acceleration_dq[index][k] = add6(&add6(&conveyed_a, &d_x_a), &bias_dq);

            // Bias derivative w.r.t. qd: velocity and joint velocity both move.
            let mut bias_dqd = cross_motion(&dvd, &joint_velocity);
            if joint_dof == Some(k) {
                bias_dqd = add6(&bias_dqd, &cross_motion(&link_velocity, &model.dofs[k].s));
            }
            let conveyed_ad = mat6_mul_vec(&xup[index], &acceleration_dqd[parent][k]);
            acceleration_dqd[index][k] = add6(&conveyed_ad, &bias_dqd);
        }
    }

    // Backward wrench pass with derivatives.
    let mut transmitted: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut transmitted_dq: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];
    let mut transmitted_dqd: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];
    let mut gradient = NonLinearEffectsGradient {
        with_respect_to_q: vec![vec![0.0; nv]; nv],
        with_respect_to_qd: vec![vec![0.0; nv]; nv],
    };

    for index in (0..link_count).rev() {
        let momentum = mat6_mul_vec(&inertia[index], &velocity[index]);
        let force = add6(
            &add6(
                &mat6_mul_vec(&inertia[index], &acceleration[index]),
                &cross_force(&velocity[index], &momentum),
            ),
            &transmitted[index],
        );
        for &dof in &model.link_dofs[index] {
            for k in 0..nv {
                // dh/dq_k = S^T d(force)/dq_k.
                let d_momentum = mat6_mul_vec(&inertia[index], &velocity_dq[index][k]);
                let d_cross = dual_cross_derivative(
                    &velocity[index],
                    &velocity_dq[index][k],
                    &momentum,
                    &d_momentum,
                );
                let d_force = add6(
                    &add6(
                        &mat6_mul_vec(&inertia[index], &acceleration_dq[index][k]),
                        &d_cross,
                    ),
                    &transmitted_dq[index][k],
                );
                gradient.with_respect_to_q[k][dof] = dot6(&model.dofs[dof].s, &d_force);

                let d_momentum_qd = mat6_mul_vec(&inertia[index], &velocity_dqd[index][k]);
                let d_cross_qd = dual_cross_derivative(
                    &velocity[index],
                    &velocity_dqd[index][k],
                    &momentum,
                    &d_momentum_qd,
                );
                let d_force_qd = add6(
                    &add6(
                        &mat6_mul_vec(&inertia[index], &acceleration_dqd[index][k]),
                        &d_cross_qd,
                    ),
                    &transmitted_dqd[index][k],
                );
                gradient.with_respect_to_qd[k][dof] = dot6(&model.dofs[dof].s, &d_force_qd);
            }
        }
        if let Some(parent) = model.links[index].parent {
            for k in 0..nv {
                let d_momentum = mat6_mul_vec(&inertia[index], &velocity_dq[index][k]);
                let d_cross = dual_cross_derivative(
                    &velocity[index],
                    &velocity_dq[index][k],
                    &momentum,
                    &d_momentum,
                );
                let d_force = add6(
                    &add6(
                        &mat6_mul_vec(&inertia[index], &acceleration_dq[index][k]),
                        &d_cross,
                    ),
                    &transmitted_dq[index][k],
                );
                let conveyed = mat6_transpose_mul_vec(&xup[index], &d_force);
                let conveyed_xup = mat6_transpose_mul_vec(&xup_gradient[index][k], &force);
                transmitted_dq[parent][k] =
                    add6(&transmitted_dq[parent][k], &add6(&conveyed, &conveyed_xup));

                let d_momentum_qd = mat6_mul_vec(&inertia[index], &velocity_dqd[index][k]);
                let d_cross_qd = dual_cross_derivative(
                    &velocity[index],
                    &velocity_dqd[index][k],
                    &momentum,
                    &d_momentum_qd,
                );
                let d_force_qd = add6(
                    &add6(
                        &mat6_mul_vec(&inertia[index], &acceleration_dqd[index][k]),
                        &d_cross_qd,
                    ),
                    &transmitted_dqd[index][k],
                );
                let conveyed_qd = mat6_transpose_mul_vec(&xup[index], &d_force_qd);
                transmitted_dqd[parent][k] = add6(&transmitted_dqd[parent][k], &conveyed_qd);
            }
            transmitted[parent] = add6(
                &transmitted[parent],
                &mat6_transpose_mul_vec(&xup[index], &force),
            );
        }
    }

    Ok(gradient)
}

/// Gradient of the nonlinear effects, one vector per generalized coordinate.
#[derive(Clone, Debug, PartialEq)]
pub struct NonLinearEffectsGradient {
    /// `dh/dq_k` for each generalized coordinate.
    pub with_respect_to_q: Vec<Vec<f64>>,
    /// `dh/dqd_k` for each generalized coordinate.
    pub with_respect_to_qd: Vec<Vec<f64>>,
}

/// Derivative of `cross_force(v, I v)` with respect to the state that moves
/// both `v` and `I v`.
fn dual_cross_derivative(
    velocity: &SpatialVec,
    d_velocity: &SpatialVec,
    momentum: &SpatialVec,
    d_momentum: &SpatialVec,
) -> SpatialVec {
    add6(
        &cross_force(d_velocity, momentum),
        &cross_force(velocity, d_momentum),
    )
}

/// Generalized gravity force `g(q)`.
pub fn gravity_torque(model: &ArticulatedModel, q: &[f64]) -> Result<Vec<f64>, DynamicsError> {
    rnea(model, q, &vec![0.0; model.nv()], &vec![0.0; model.nv()])
}

/// Forward dynamics `qdd = M(q)^-1 (tau - C(q, qd) qd - g(q))`.
pub fn forward_dynamics(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
) -> Result<Vec<f64>, DynamicsError> {
    validate(model, tau, "tau")?;
    let matrix = mass_matrix(model, q)?;
    let bias = non_linear_effects(model, q, qd)?;
    let rhs: Vec<f64> = tau
        .iter()
        .zip(bias.iter())
        .map(|(effort, bias)| effort - bias)
        .collect();
    matrix.solve(&rhs).ok_or(DynamicsError::SingularMassMatrix)
}

/// Analytical gradient of the forward dynamics `qdd = M(q)^-1 (tau - h(q, qd))`.
///
/// The entries are built from [`mass_matrix_gradient`] and
/// [`non_linear_effects_gradient`]: differentiating `M qdd = tau - h` gives
/// `dM/dq_k qdd + M dqdd/dq_k = -dh/dq_k`, and the control columns are the mass
/// matrix inverse. No finite differencing is used.
#[allow(clippy::needless_range_loop)]
pub fn forward_dynamics_gradient(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
) -> Result<ForwardDynamicsGradient, DynamicsError> {
    validate(model, tau, "tau")?;
    let nv = model.nv();
    let matrix = mass_matrix(model, q)?;
    let acceleration = forward_dynamics(model, q, qd, tau)?;
    let mass_gradient = mass_matrix_gradient(model, q)?;
    let bias_gradient = non_linear_effects_gradient(model, q, qd)?;

    let mut with_respect_to_q = vec![vec![0.0; nv]; nv];
    let mut with_respect_to_qd = vec![vec![0.0; nv]; nv];
    let mut with_respect_to_control = vec![vec![0.0; nv]; nv];
    for k in 0..nv {
        let mut rhs = vec![0.0; nv];
        for row in 0..nv {
            let mut product = 0.0;
            for column in 0..nv {
                product += mass_gradient[k].get(row, column) * acceleration[column];
            }
            rhs[row] = -bias_gradient.with_respect_to_q[k][row] - product;
        }
        let column = matrix
            .solve(&rhs)
            .ok_or(DynamicsError::SingularMassMatrix)?;
        with_respect_to_q[k][..nv].copy_from_slice(&column[..nv]);

        let rhs_qd: Vec<f64> = bias_gradient.with_respect_to_qd[k]
            .iter()
            .map(|value| -value)
            .collect();
        let column_qd = matrix
            .solve(&rhs_qd)
            .ok_or(DynamicsError::SingularMassMatrix)?;
        with_respect_to_qd[k][..nv].copy_from_slice(&column_qd[..nv]);
    }
    // d(qdd)/dtau_j = M^-1 e_j.
    for j in 0..nv {
        let mut basis = vec![0.0; nv];
        basis[j] = 1.0;
        let column = matrix
            .solve(&basis)
            .ok_or(DynamicsError::SingularMassMatrix)?;
        with_respect_to_control[j][..nv].copy_from_slice(&column[..nv]);
    }

    Ok(ForwardDynamicsGradient {
        with_respect_to_q,
        with_respect_to_qd,
        with_respect_to_control,
    })
}

/// Jacobians of the forward dynamics with respect to state and control.
#[derive(Clone, Debug, PartialEq)]
pub struct ForwardDynamicsGradient {
    /// `d(qdd)/dq_k`, one vector per generalized coordinate.
    pub with_respect_to_q: Vec<Vec<f64>>,
    /// `d(qdd)/dqd_k`, one vector per generalized coordinate.
    pub with_respect_to_qd: Vec<Vec<f64>>,
    /// `d(qdd)/dtau_j`, one vector per generalized force.
    pub with_respect_to_control: Vec<Vec<f64>>,
}

/// Center of mass of the model at configuration `q`, in world coordinates.
pub fn center_of_mass(model: &ArticulatedModel, q: &[f64]) -> Result<Vec3, DynamicsError> {
    validate(model, q, "q")?;
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let mut total_mass = 0.0;
    let mut weighted = Vec3::ZERO;
    for (index, link) in model.links.iter().enumerate() {
        let mass = link.inertia.mass_kg;
        if mass == 0.0 {
            continue;
        }
        let transform = kinematics
            .transform_at(index)
            .copied()
            .unwrap_or(Transform3::IDENTITY);
        weighted += transform_point(&transform, link.inertia.center_of_mass_m) * mass;
        total_mass += mass;
    }
    if total_mass > 0.0 {
        Ok(weighted / total_mass)
    } else {
        Ok(Vec3::ZERO)
    }
}

/// Spatial Jacobian of a link point, `6 x nv`, in the dynamics convention.
///
/// The returned matrix maps a generalized velocity (with the floating-base
/// body-twist convention used by [`mass_matrix`] and [`rnea`]) to the world
/// `[linear; angular]` velocity of the point. It differs from
/// [`rne_robot::KinematicModel::jacobian`] in the base columns, which use
/// body-frame twist rather than roll-pitch-yaw rates. The joint columns are
/// identical.
#[allow(clippy::needless_range_loop)]
pub fn frame_jacobian(
    model: &ArticulatedModel,
    q: &[f64],
    link: rne_ecs::Entity,
    point_local_m: Vec3,
) -> Result<DenseMatrix, DynamicsError> {
    validate(model, q, "q")?;
    let rne_jacobian = model.kinematic.jacobian(q, link, point_local_m)?;
    let nv = model.nv();
    let mut jacobian = DenseMatrix::zeros(6, nv);
    for row in 0..6 {
        for column in 0..nv {
            jacobian.set(row, column, rne_jacobian.get(row, column));
        }
    }
    if model.base_dof() != 6 {
        return Ok(jacobian);
    }

    let kinematics = model.kinematic.forward_kinematics(q)?;
    let base_rotation = kinematics.transforms()[0].rotation;
    let map = base_velocity_map(q, base_rotation);
    let mut base_block = [[0.0; 6]; 6];
    for row in 0..6 {
        for column in 0..6 {
            base_block[row][column] = rne_jacobian.get(row, column);
        }
    }
    for row in 0..6 {
        for column in 0..6 {
            let mut value = 0.0;
            for k in 0..6 {
                value += base_block[row][k] * map[k][column];
            }
            jacobian.set(row, column, value);
        }
    }
    Ok(jacobian)
}

/// Maps a base generalized velocity in the dynamics convention (body linear and
/// angular velocity) to the `(translation_rate, rpy_rate)` convention used by
/// [`rne_robot::KinematicModel`].
#[allow(clippy::needless_range_loop)]
fn base_velocity_map(q: &[f64], base_rotation: rne_math::Quat) -> [[f64; 6]; 6] {
    let rotation = crate::spatial::rotation_matrix(base_rotation);
    let euler_rate = euler_rate_map(q[3], q[4], q[5]);
    let inverse = invert3(&euler_rate).unwrap_or([[0.0; 3]; 3]);
    let angular = crate::spatial::mat3_mul(&inverse, &rotation);
    let mut map = [[0.0; 6]; 6];
    for row in 0..3 {
        for column in 0..3 {
            map[row][column] = rotation[row][column];
            map[row + 3][column + 3] = angular[row][column];
        }
    }
    map
}

/// Columns are the world axes of the roll, pitch, and yaw rates.
fn euler_rate_map(_roll: f64, pitch: f64, yaw: f64) -> crate::spatial::Mat3 {
    use rne_math::Quat;
    let qy = Quat::from_rotation_z(yaw);
    let qp = Quat::from_rotation_y(pitch);
    let roll_axis = qy * (qp * Vec3::X);
    let pitch_axis = qy * Vec3::Y;
    let yaw_axis = Vec3::Z;
    [
        [roll_axis.x, pitch_axis.x, yaw_axis.x],
        [roll_axis.y, pitch_axis.y, yaw_axis.y],
        [roll_axis.z, pitch_axis.z, yaw_axis.z],
    ]
}

fn invert3(matrix: &crate::spatial::Mat3) -> Option<crate::spatial::Mat3> {
    let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    if determinant.abs() < 1.0e-12 {
        return None;
    }
    let inverse_determinant = 1.0 / determinant;
    Some([
        [
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inverse_determinant,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inverse_determinant,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inverse_determinant,
        ],
        [
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inverse_determinant,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inverse_determinant,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inverse_determinant,
        ],
        [
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inverse_determinant,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inverse_determinant,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inverse_determinant,
        ],
    ])
}

/// Jacobian of the center of mass, `6 x nv`, ordered `[linear; angular]`, in
/// the dynamics convention.
pub fn com_jacobian(model: &ArticulatedModel, q: &[f64]) -> Result<DenseMatrix, DynamicsError> {
    validate(model, q, "q")?;
    let mut jacobian = DenseMatrix::zeros(6, model.nv());
    let mut total_mass = 0.0;
    for link in &model.links {
        let mass = link.inertia.mass_kg;
        if mass == 0.0 {
            continue;
        }
        total_mass += mass;
        let link_jacobian = frame_jacobian(model, q, link.entity, link.inertia.center_of_mass_m)?;
        for row in 0..6 {
            for col in 0..model.nv() {
                let value = jacobian.get(row, col) + mass * link_jacobian.get(row, col);
                jacobian.set(row, col, value);
            }
        }
    }
    if total_mass > 0.0 {
        for row in 0..6 {
            for col in 0..model.nv() {
                jacobian.set(row, col, jacobian.get(row, col) / total_mass);
            }
        }
    }
    Ok(jacobian)
}

/// Centroidal momentum `[linear; angular]` in world coordinates.
///
/// The linear part is `M * v_com`; the angular part is taken about the whole-body
/// center of mass. During a flight phase (no external wrench) the angular part
/// is conserved, so it is the natural quantity for shaping an aerial rotation
/// such as a flip.
pub fn centroidal_momentum(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<SpatialVec, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;

    let motions = link_motions(model, q, qd)?;
    let com = center_of_mass(model, q)?;
    let mut linear = Vec3::ZERO;
    let mut angular_about_origin = Vec3::ZERO;
    for (index, motion) in motions.iter().enumerate() {
        let Some(inertia) = model.link_inertia(index) else {
            continue;
        };
        if inertia.mass_kg == 0.0 {
            continue;
        }
        let offset = motion.world_transform.rotation * inertia.center_of_mass_m;
        let com_world = motion.world_transform.translation + offset;
        let velocity =
            motion.linear_velocity_world_m_s + motion.angular_velocity_world_rad_s.cross(offset);
        let momentum = velocity * inertia.mass_kg;
        linear += momentum;
        let rotation = crate::spatial::rotation_matrix(motion.world_transform.rotation);
        let inertia_world = crate::spatial::mat3_mul(
            &crate::spatial::mat3_mul(&rotation, &inertia.inertia_about_com_kg_m2),
            &crate::spatial::mat3_transpose(&rotation),
        );
        angular_about_origin += com_world.cross(momentum)
            + crate::spatial::mat3_mul_vec(&inertia_world, motion.angular_velocity_world_rad_s);
    }
    // Shift the angular momentum from the world origin to the center of mass.
    let angular = angular_about_origin - com.cross(linear);
    Ok([
        linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
    ])
}

/// Centroidal momentum matrix `A(q)`, `6 x nv`, such that `L = A(q) qd`.
///
/// The rows are `[linear; angular]` about the whole-body center of mass, in the
/// world frame, matching [`centroidal_momentum`]. The momentum is linear in
/// `qd` with no bias, so the columns are exact unit perturbations. An aerial
/// controller uses this as the task Jacobian for a momentum-rate objective.
pub fn centroidal_momentum_matrix(
    model: &ArticulatedModel,
    q: &[f64],
) -> Result<DenseMatrix, DynamicsError> {
    validate(model, q, "q")?;
    let nv = model.nv();
    let mut matrix = DenseMatrix::zeros(6, nv);
    for column in 0..nv {
        let mut qd = vec![0.0; nv];
        qd[column] = 1.0;
        let momentum = centroidal_momentum(model, q, &qd)?;
        for (row, value) in momentum.iter().enumerate() {
            matrix.set(row, column, *value);
        }
    }
    Ok(matrix)
}

/// Centroidal momentum rate bias `c(q, qd)` with `Ldot = A(q) qdd + c`.
///
/// This is `(dL/dq) qd` for `L = A(q) qd`, computed by central-differencing
/// [`centroidal_momentum`] in `q`. An aerial controller adds it so the momentum
/// rate task is exact at the acceleration level.
#[allow(clippy::needless_range_loop)]
pub fn centroidal_momentum_bias(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<SpatialVec, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;
    let nv = model.nv();
    let epsilon = 1.0e-6;
    let mut bias = [0.0; 6];
    for index in 0..nv {
        let mut plus = q.to_vec();
        plus[index] += epsilon;
        let mut minus = q.to_vec();
        minus[index] -= epsilon;
        let plus_momentum = centroidal_momentum(model, &plus, qd)?;
        let minus_momentum = centroidal_momentum(model, &minus, qd)?;
        for row in 0..6 {
            bias[row] += (plus_momentum[row] - minus_momentum[row]) / (2.0 * epsilon) * qd[index];
        }
    }
    Ok(bias)
}

/// Transforms a spatial motion vector by the adjoint of a pose.
///
/// `adjoint` maps a body-frame motion vector into the parent frame, so it is
/// the same matrix as [`motion_transform`] applied to the pose itself.
fn adjoint(pose: &Transform3) -> Mat6 {
    motion_transform(pose)
}

/// Derivative of the child-in-parent motion transform `xup` with respect to
/// every generalized coordinate.
///
/// Entry `[link][k]` is the `6x6` matrix `d(xup[link])/dq_k`, where
/// `xup[link]` is the same transform used by [`mass_matrix`]. It uses the
/// identity `xup = Ad_{M^-1}` and `d(Ad_{M^-1}) = -ad_{xi} Ad_{M^-1}`, with
/// `xi` the body twist of `M`. The result is in topological link order and has
/// one matrix per generalized coordinate.
#[allow(clippy::needless_range_loop)]
pub fn xup_derivatives(
    model: &ArticulatedModel,
    q: &[f64],
) -> Result<Vec<Vec<Mat6>>, DynamicsError> {
    validate(model, q, "q")?;
    let kinematic = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematic.transforms();
    let twists = link_pose_body_twist_derivatives(model, q)?;
    let nv = model.nv();
    let link_count = model.link_count();

    let mut result: Vec<Vec<Mat6>> = vec![vec![mat6_zero(); nv]; link_count];
    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let child_in_parent =
            inverse_transform(&transforms[parent]).mul_transform(&transforms[index]);
        let xup = motion_transform(&inverse_transform(&child_in_parent));
        for k in 0..nv {
            // Body twist of `M = P^-1 Q` is `xi_Q - Ad_{M^-1} xi_P`, where
            // `xup = Ad_{M^-1}`. This cancels the parent motion so a base
            // translation produces no `xup` derivative.
            let conveyed = mat6_mul_vec(&xup, &twists[parent][k]);
            let twist = {
                let mut value = twists[index][k];
                for component in 0..6 {
                    value[component] -= conveyed[component];
                }
                value
            };
            let ad = motion_cross_matrix(&twist);
            result[index][k] = negate_mat6(&mat6_mul(&ad, &xup));
        }
    }

    Ok(result)
}

/// The `6x6` motion cross-product matrix `ad_v` such that `ad_v m =
/// cross_motion(v, m)`.
fn motion_cross_matrix(v: &SpatialVec) -> Mat6 {
    let mut matrix = mat6_zero();
    for column in 0..6 {
        let mut basis = [0.0; 6];
        basis[column] = 1.0;
        let image = cross_motion(v, &basis);
        for row in 0..6 {
            matrix[row][column] = image[row];
        }
    }
    matrix
}

fn negate_mat6(matrix: &Mat6) -> Mat6 {
    let mut out = *matrix;
    for row in out.iter_mut() {
        for value in row.iter_mut() {
            *value = -*value;
        }
    }
    out
}

/// Derivative of a link world pose with respect to every generalized
/// coordinate, expressed as a body-frame twist.
///
/// Entry `[link][k]` is the 6-vector `[linear; angular]` such that, to first
/// order, `dpose/dq_k = pose * twist^` in the body frame. This chart composes
/// additively along the forward-kinematics recursion, which makes it the right
/// input for differentiating the mass matrix. The result is in topological link
/// order, matching [`ArticulatedModel::link_entity`].
#[allow(clippy::needless_range_loop)]
pub fn link_pose_body_twist_derivatives(
    model: &ArticulatedModel,
    q: &[f64],
) -> Result<Vec<Vec<SpatialVec>>, DynamicsError> {
    validate(model, q, "q")?;
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let base = model.base_dof();
    let nv = model.nv();
    let link_count = model.link_count();

    let mut result: Vec<Vec<SpatialVec>> = vec![vec![[0.0; 6]; nv]; link_count];

    if base == 6 {
        // `pose[0]` translation is `q[0..3]`; the body twist of a translation
        // derivative is the world axis rotated into the body frame.
        let rotation = transforms[0].rotation;
        let inverse_rotation = rotation.conjugate();
        for k in 0..3 {
            let mut axis = [0.0; 6];
            let world_axis = match k {
                0 => Vec3::X,
                1 => Vec3::Y,
                _ => Vec3::Z,
            };
            let body_axis = inverse_rotation * world_axis;
            axis[0] = body_axis.x;
            axis[1] = body_axis.y;
            axis[2] = body_axis.z;
            result[0][k] = axis;
        }
        // Body-frame angular velocity columns of the ZYX Euler chart
        // `R = Rz(yaw) Ry(pitch) Rx(roll)`.
        let (roll, pitch) = (q[3], q[4]);
        let body_angular = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, roll.cos(), -roll.sin()),
            Vec3::new(
                -pitch.sin(),
                pitch.cos() * roll.sin(),
                pitch.cos() * roll.cos(),
            ),
        ];
        for (offset, body_axis) in body_angular.iter().enumerate() {
            let mut axis = [0.0; 6];
            axis[3] = body_axis.x;
            axis[4] = body_axis.y;
            axis[5] = body_axis.z;
            result[0][3 + offset] = axis;
        }
    }

    // `pose[i] = pose[parent] * child_in_parent`, where `child_in_parent`
    // depends on `q_k` only through link `i`'s own joint. With body twists,
    // `d(pose[i])/dq_k = pose[i] * (Ad_{child_in_parent^-1} xi_parent + xi_joint)`
    // and the joint contribution is the motion subspace at unit velocity.
    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let child_in_parent =
            inverse_transform(&transforms[parent]).mul_transform(&transforms[index]);
        let inverse_child = inverse_transform(&child_in_parent);
        let adjoint_inverse_child = adjoint(&inverse_child);
        let joint_dof = model.links[index]
            .joint
            .as_ref()
            .and_then(|joint| joint.dof);
        for k in 0..nv {
            let conveyed = mat6_mul_vec(&adjoint_inverse_child, &result[parent][k]);
            let mut twist = conveyed;
            if joint_dof == Some(k) {
                let subspaces = model.dofs[k].s;
                twist = add6(&twist, &subspaces);
            }
            result[index][k] = twist;
        }
    }

    Ok(result)
}

/// Derivative of a link world pose with respect to every generalized
/// coordinate, in a translation/rotation-vector convention.
///
/// The translation part is `d(pose.translation)/dq_k`. The rotation part is
/// `d(rotation_vector)/dq_k`, where `rotation_vector = axis * angle` is the
/// logarithm of `pose.rotation`. This is the chart whose finite difference of
/// the pose matches exactly, which is what a solver differentiates.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkPoseDerivative {
    /// `d(translation)/dq_k` for each generalized coordinate, in meters/unit.
    pub translation: Vec<Vec3>,
    /// `d(rotation_vector)/dq_k`, in radians/unit.
    pub rotation_vector: Vec<Vec3>,
}

/// Computes the derivative of every link world pose with respect to `q`.
///
/// The result is in topological link order, matching
/// [`ArticulatedModel::link_entity`], and each entry has one `Vec3` per
/// generalized coordinate. The computation is exact for the same forward
/// kinematics used by [`mass_matrix`]: it propagates the pose derivative down
/// the tree instead of finite-differencing.
pub fn link_pose_derivatives(
    model: &ArticulatedModel,
    q: &[f64],
) -> Result<Vec<LinkPoseDerivative>, DynamicsError> {
    validate(model, q, "q")?;
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let base = model.base_dof();
    let nv = model.nv();
    let link_count = model.link_count();

    let mut result: Vec<LinkPoseDerivative> = (0..link_count)
        .map(|_| LinkPoseDerivative {
            translation: vec![Vec3::ZERO; nv],
            rotation_vector: vec![Vec3::ZERO; nv],
        })
        .collect();

    // Only the floating base root carries a direct dependence on the first six
    // coordinates; every other link inherits it through the tree recursion.
    if base == 6 {
        result[0].translation[0] = Vec3::X;
        result[0].translation[1] = Vec3::Y;
        result[0].translation[2] = Vec3::Z;
        // The root orientation uses the ZYX Euler chart, so the world-frame
        // rotation derivative of coordinates 3, 4, 5 is not the identity.
        let yaw = Quat::from_rotation_z(q[5]);
        result[0].rotation_vector[3] =
            (yaw * Quat::from_rotation_y(q[4]) * Vec3::X).normalize_or_zero();
        result[0].rotation_vector[4] = (yaw * Vec3::Y).normalize_or_zero();
        result[0].rotation_vector[5] = Vec3::Z;
    }

    // A link's pose factorizes as `pose[i] = prefix_i * motion_i(q_k)`, where
    // `prefix_i` is the pose with the joint displacement removed. `motion_i`
    // only depends on the link's own DoF, so `prefix_i` is constant during the
    // differentiation and can be recovered from forward kinematics evaluated at
    // zero displacement for that DoF.
    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let joint_dof = model.links[index]
            .joint
            .as_ref()
            .and_then(|joint| joint.dof);
        let prefix = match joint_dof {
            Some(dof) => {
                let mut zeroed = q.to_vec();
                zeroed[dof] = 0.0;
                model
                    .kinematic
                    .forward_kinematics(&zeroed)?
                    .transform_at(index)
                    .copied()
                    .unwrap_or(Transform3::IDENTITY)
            }
            None => transforms[index],
        };
        let prefix_rotation = prefix.rotation;
        let prefix_translation = prefix.translation;
        for k in 0..nv {
            let parent_translation = result[parent].translation[k];
            let parent_rotation = result[parent].rotation_vector[k];
            // `pose[parent]` moves the whole chain rigidly: its translation
            // derivative propagates directly, and its rotation derivative adds
            // the cross term for the lever arm from the parent origin to the
            // child origin.
            let lever = prefix_translation - transforms[parent].translation;
            let mut translation = parent_translation + parent_rotation.cross(lever);
            let mut rotation_vector = parent_rotation;

            if let Some(dof) = joint_dof {
                if dof == k {
                    // d(motion_i)/dq_i at zero displacement is the joint motion
                    // subspace referred to the parent frame.
                    let subspaces = model.dofs[dof].s;
                    let axis_parent = Vec3::new(subspaces[3], subspaces[4], subspaces[5]);
                    let linear_parent = Vec3::new(subspaces[0], subspaces[1], subspaces[2]);
                    if axis_parent.length_squared() > 0.0 {
                        rotation_vector += prefix_rotation * axis_parent;
                    }
                    if linear_parent.length_squared() > 0.0 {
                        translation += prefix_rotation * linear_parent;
                    }
                }
            }
            result[index].translation[k] = translation;
            result[index].rotation_vector[k] = rotation_vector;
        }
    }

    Ok(result)
}

/// World-frame motion and bias acceleration of one link at state `(q, qd)`.
///
/// The linear and angular accelerations are the *bias* accelerations, i.e. the
/// link acceleration when `qdd = 0` and gravity is excluded. They are the
/// `Jdot * qd` term needed by task-space controllers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkMotion {
    /// World transform of the link frame.
    pub world_transform: Transform3,
    /// Linear velocity of the link origin in world coordinates, in m/s.
    pub linear_velocity_world_m_s: Vec3,
    /// Angular velocity in world coordinates, in rad/s.
    pub angular_velocity_world_rad_s: Vec3,
    /// Bias linear acceleration of the link origin, in m/s².
    pub linear_acceleration_world_m_s2: Vec3,
    /// Bias angular acceleration, in rad/s².
    pub angular_acceleration_world_rad_s2: Vec3,
}

impl LinkMotion {
    /// Bias acceleration of a point fixed in the link, in world coordinates.
    pub fn point_bias_acceleration_m_s2(&self, point_local_m: Vec3) -> Vec3 {
        let offset = self.world_transform.rotation * point_local_m;
        let omega = self.angular_velocity_world_rad_s;
        self.linear_acceleration_world_m_s2
            + self.angular_acceleration_world_rad_s2.cross(offset)
            + omega.cross(omega.cross(offset))
    }
}

/// Computes world-frame motion and bias acceleration for every link.
///
/// The result is in the model's topological link order, matching
/// [`ArticulatedModel::link_entity`].
pub fn link_motions(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<Vec<LinkMotion>, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;

    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let xup = xup_transforms(model, transforms);
    let link_count = model.link_count();

    let mut velocity: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut acceleration: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    if model.base_dof() == 6 {
        velocity[0] = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]];
    }
    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let mut link_velocity = mat6_mul_vec(&xup[index], &velocity[parent]);
        let mut link_acceleration = mat6_mul_vec(&xup[index], &acceleration[parent]);
        if let Some(joint) = &model.links[index].joint {
            if let Some(dof) = joint.dof {
                let subspaces = model.dofs[dof].s;
                let joint_velocity = scale6(&subspaces, qd[dof]);
                link_velocity = add6(&link_velocity, &joint_velocity);
                let bias = cross_motion(&link_velocity, &joint_velocity);
                link_acceleration = add6(&link_acceleration, &bias);
            }
        }
        velocity[index] = link_velocity;
        acceleration[index] = link_acceleration;
    }

    let mut motions = Vec::with_capacity(link_count);
    for index in 0..link_count {
        let rotation = transforms[index].rotation;
        let linear_velocity =
            rotation * Vec3::new(velocity[index][0], velocity[index][1], velocity[index][2]);
        let angular_velocity =
            rotation * Vec3::new(velocity[index][3], velocity[index][4], velocity[index][5]);
        // The spatial acceleration is the body-frame derivative of the spatial
        // velocity. The classical world acceleration of the link origin adds the
        // frame-rotation term `omega_body x v_body` before rotating to world.
        let velocity_body = Vec3::new(velocity[index][0], velocity[index][1], velocity[index][2]);
        let omega_body = Vec3::new(velocity[index][3], velocity[index][4], velocity[index][5]);
        let linear_acceleration = rotation
            * (Vec3::new(
                acceleration[index][0],
                acceleration[index][1],
                acceleration[index][2],
            ) + omega_body.cross(velocity_body));
        let angular_acceleration = rotation
            * Vec3::new(
                acceleration[index][3],
                acceleration[index][4],
                acceleration[index][5],
            );
        motions.push(LinkMotion {
            world_transform: transforms[index],
            linear_velocity_world_m_s: linear_velocity,
            angular_velocity_world_rad_s: angular_velocity,
            linear_acceleration_world_m_s2: linear_acceleration,
            angular_acceleration_world_rad_s2: angular_acceleration,
        });
    }
    Ok(motions)
}

/// A point contact active during a dynamics step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactSpec {
    /// Link that owns the contact point.
    pub link: rne_ecs::Entity,
    /// Contact position in the link frame, in meters.
    pub point_local_m: Vec3,
}

/// Contact compliance added to the constrained-dynamics KKT diagonal.
///
/// This keeps the system solvable when the active points are linearly
/// dependent (several feet that do not independently constrain the body) at the
/// cost of a small, bounded contact acceleration.
pub const CONTACT_REGULARIZATION: f64 = 1.0e-8;

/// Constrained forward dynamics with rigid point contacts.
///
/// Solves the KKT system
///
/// ```text
/// [ M  -Jᵀ ] [ qdd ]   [ tau - h ]
/// [ J   0  ] [  λ  ] = [ -Jdot qd ]
/// ```
///
/// where the contact Jacobian `J` stacks the linear Jacobians of the active
/// points and `-Jdot qd` is their bias acceleration. Returns the joint
/// acceleration and the contact force (Lagrange multiplier) at each point, so
/// the contacts neither accelerate nor separate.
#[allow(clippy::needless_range_loop)]
pub fn constrained_forward_dynamics(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactSpec],
) -> Result<(Vec<f64>, Vec<Vec3>), DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;
    validate(model, tau, "tau")?;
    let nv = model.nv();
    let nc = contacts.len();
    let size = nv + 3 * nc;

    let mass = mass_matrix(model, q)?;
    let bias = non_linear_effects(model, q, qd)?;
    let motions = link_motions(model, q, qd)?;

    let mut jacobian = vec![vec![0.0; nv]; 3 * nc];
    let mut contact_bias = vec![0.0; 3 * nc];
    for (contact, spec) in contacts.iter().enumerate() {
        let link_index = model
            .kinematic
            .link_index(spec.link)
            .ok_or(DynamicsError::MissingDofOwner(contact))?;
        let jac = frame_jacobian(model, q, spec.link, spec.point_local_m)?;
        for row in 0..3 {
            for column in 0..nv {
                jacobian[3 * contact + row][column] = jac.get(row, column);
            }
        }
        let point_bias = motions[link_index].point_bias_acceleration_m_s2(spec.point_local_m);
        contact_bias[3 * contact] = point_bias.x;
        contact_bias[3 * contact + 1] = point_bias.y;
        contact_bias[3 * contact + 2] = point_bias.z;
    }

    let mut matrix = DenseMatrix::zeros(size, size);
    let mut rhs = vec![0.0; size];
    for row in 0..nv {
        for column in 0..nv {
            matrix.set(row, column, mass.get(row, column));
        }
        rhs[row] = tau[row] - bias[row];
    }
    for contact in 0..nc {
        for component in 0..3 {
            let row = 3 * contact + component;
            for column in 0..nv {
                let value = jacobian[row][column];
                matrix.set(column, nv + row, -value);
                matrix.set(nv + row, column, value);
            }
            // Contact compliance regularizes redundant contacts (for example
            // several feet that do not independently constrain the body).
            matrix.set(nv + row, nv + row, -CONTACT_REGULARIZATION);
            rhs[nv + row] = -contact_bias[row];
        }
    }

    let solution = matrix
        .solve(&rhs)
        .ok_or(DynamicsError::SingularMassMatrix)?;
    let joint_acceleration = solution[..nv].to_vec();
    let mut forces = Vec::with_capacity(nc);
    for contact in 0..nc {
        forces.push(Vec3::new(
            solution[nv + 3 * contact],
            solution[nv + 3 * contact + 1],
            solution[nv + 3 * contact + 2],
        ));
    }
    Ok((joint_acceleration, forces))
}

/// Impulsive reset of the joint velocities when new contacts are established.
///
/// Solves the impulse KKT system
///
/// ```text
/// [ M  -Jᵀ ] [ qd⁺ ]   [ M qd⁻ ]
/// [ J   0  ] [  Λ  ] = [   0    ]
/// ```
///
/// where `J` stacks the linear Jacobians of the new contacts. Returns the
/// post-impact velocity `qd⁺` (with the new contact points instantaneously at
/// rest) and the impulse `Λ` at each point. The configuration is unchanged.
#[allow(clippy::needless_range_loop)]
pub fn impulse_velocity(
    model: &ArticulatedModel,
    q: &[f64],
    qd_minus: &[f64],
    contacts: &[ContactSpec],
) -> Result<(Vec<f64>, Vec<Vec3>), DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd_minus, "qd")?;
    let nv = model.nv();
    let nc = contacts.len();
    let size = nv + 3 * nc;

    let mass = mass_matrix(model, q)?;

    let mut jacobian = vec![vec![0.0; nv]; 3 * nc];
    for (contact, spec) in contacts.iter().enumerate() {
        let jac = frame_jacobian(model, q, spec.link, spec.point_local_m)?;
        for row in 0..3 {
            for column in 0..nv {
                jacobian[3 * contact + row][column] = jac.get(row, column);
            }
        }
    }

    let mut matrix = DenseMatrix::zeros(size, size);
    let mut rhs = vec![0.0; size];
    for row in 0..nv {
        for column in 0..nv {
            matrix.set(row, column, mass.get(row, column));
        }
        rhs[row] = (0..nv)
            .map(|column| mass.get(row, column) * qd_minus[column])
            .sum();
    }
    for contact in 0..nc {
        for component in 0..3 {
            let row = 3 * contact + component;
            for column in 0..nv {
                let value = jacobian[row][column];
                matrix.set(column, nv + row, -value);
                matrix.set(nv + row, column, value);
            }
            matrix.set(nv + row, nv + row, -CONTACT_REGULARIZATION);
        }
    }

    let solution = matrix
        .solve(&rhs)
        .ok_or(DynamicsError::SingularMassMatrix)?;
    let post_impact = solution[..nv].to_vec();
    let mut impulses = Vec::with_capacity(nc);
    for contact in 0..nc {
        impulses.push(Vec3::new(
            solution[nv + 3 * contact],
            solution[nv + 3 * contact + 1],
            solution[nv + 3 * contact + 2],
        ));
    }
    Ok((post_impact, impulses))
}

/// Jacobian of the impulsive velocity reset with respect to the pre-impact
/// velocity.
///
/// The reset `qd+ = P(q) qd-` is linear in `qd-` because the KKT matrix does
/// not depend on `qd-`, so the returned `nv x nv` matrix has column `i` equal
/// to `impulse_velocity(model, q, e_i, contacts)`. An impulse-aware DDP uses
/// this as the transition Jacobian at a contact-addition node. With no contacts
/// the reset is the identity.
pub fn impulse_velocity_gradient(
    model: &ArticulatedModel,
    q: &[f64],
    contacts: &[ContactSpec],
) -> Result<DenseMatrix, DynamicsError> {
    validate(model, q, "q")?;
    let nv = model.nv();
    let mut matrix = DenseMatrix::zeros(nv, nv);
    if contacts.is_empty() {
        for index in 0..nv {
            matrix.set(index, index, 1.0);
        }
        return Ok(matrix);
    }
    for column in 0..nv {
        let mut basis = vec![0.0; nv];
        basis[column] = 1.0;
        let (post_impact, _) = impulse_velocity(model, q, &basis, contacts)?;
        for (row, value) in post_impact.iter().enumerate().take(nv) {
            matrix.set(row, column, *value);
        }
    }
    Ok(matrix)
}

fn mat6_transpose(matrix: &Mat6) -> Mat6 {
    let mut out = mat6_zero();
    for row in 0..6 {
        for col in 0..6 {
            out[row][col] = matrix[col][row];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::{spawn_named, World};
    use rne_math::{Quat, Vec3};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};
    use rne_world::Transform3;

    fn point_mass_inertia(com: Vec3) -> RigidBodyInertia {
        RigidBodyInertia {
            center_of_mass_local_m: com,
            ixx_kg_m2: 0.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 0.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 0.0,
        }
    }

    fn joint(robot: rne_ecs::Entity, parent: rne_ecs::Entity, child: rne_ecs::Entity) -> Joint {
        Joint {
            robot,
            parent_link: parent,
            child_link: child,
            kind: JointKind::Revolute,
            limits: JointLimits::default(),
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        }
    }

    fn two_link_world(m1: f64, m2: f64, l1: f64, l2: f64) -> (World, rne_ecs::Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let link2 = spawn_named(&mut world, "link2");
        let joint1 = spawn_named(&mut world, "joint1");
        let joint2 = spawn_named(&mut world, "joint2");

        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "two_link".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: 0.0,
                ..RigidBody::default()
            },
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: m1,
                ..RigidBody::default()
            },
            point_mass_inertia(Vec3::new(l1, 0.0, 0.0)),
        ));
        world.entity_mut(link2).insert((
            Link {
                robot,
                name: "link2".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(l1, 0.0, 0.0), Quat::IDENTITY),
            RigidBody {
                mass_kg: m2,
                ..RigidBody::default()
            },
            point_mass_inertia(Vec3::new(l2, 0.0, 0.0)),
        ));
        world.entity_mut(joint1).insert(joint(robot, base, link1));
        world.entity_mut(joint2).insert(joint(robot, link1, link2));
        (world, robot)
    }

    fn floating_body_world(mass: f64, com: Vec3, tensor: [f64; 3]) -> (World, rne_ecs::Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "floating".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            FloatingBase,
            RigidBody {
                mass_kg: mass,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: com,
                ixx_kg_m2: tensor[0],
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: tensor[1],
                iyz_kg_m2: 0.0,
                izz_kg_m2: tensor[2],
            },
        ));
        (world, robot)
    }

    #[test]
    fn two_link_mass_matrix_matches_closed_form() {
        let (m1, m2, l1, l2) = (2.0, 1.5, 0.7, 0.5);
        let (world, robot) = two_link_world(m1, m2, l1, l2);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        for q2 in [0.0, 0.3, -0.8, 1.2] {
            let q = vec![0.4, q2];
            let matrix = mass_matrix(&model, &q).expect("mass matrix");
            let cosine = q2.cos();
            let expected_11 = m1 * l1 * l1 + m2 * (l1 * l1 + l2 * l2 + 2.0 * l1 * l2 * cosine);
            let expected_12 = m2 * (l2 * l2 + l1 * l2 * cosine);
            let expected_22 = m2 * l2 * l2;
            assert_relative_eq!(matrix.get(0, 0), expected_11, epsilon = 1.0e-10);
            assert_relative_eq!(matrix.get(0, 1), expected_12, epsilon = 1.0e-10);
            assert_relative_eq!(matrix.get(1, 0), expected_12, epsilon = 1.0e-10);
            assert_relative_eq!(matrix.get(1, 1), expected_22, epsilon = 1.0e-10);
        }
    }

    #[test]
    fn two_link_gravity_torque_matches_closed_form() {
        let (m1, m2, l1, l2) = (2.0, 1.5, 0.7, 0.5);
        let (world, robot) = two_link_world(m1, m2, l1, l2);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let tau = gravity_torque(&model, &[0.0, 0.0]).expect("gravity");
        let g = 9.81;
        assert_relative_eq!(tau[0], m1 * g * l1 + m2 * g * (l1 + l2), epsilon = 1.0e-9);
        assert_relative_eq!(tau[1], m2 * g * l2, epsilon = 1.0e-9);
    }

    #[test]
    fn rnea_is_linear_in_acceleration() {
        let (world, robot) = two_link_world(1.3, 0.9, 0.6, 0.4);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = [0.4, -0.7];
        let qd = [0.3, 0.5];
        let qdd = [0.9, -0.2];
        let matrix = mass_matrix(&model, &q).expect("mass matrix");
        let total = rnea(&model, &q, &qd, &qdd).expect("rnea");
        let bias = non_linear_effects(&model, &q, &qd).expect("bias");
        let expected = matrix.mul_vec(&qdd);
        for index in 0..2 {
            assert_relative_eq!(
                total[index],
                expected[index] + bias[index],
                epsilon = 1.0e-10
            );
        }
    }

    #[test]
    fn mass_matrix_is_symmetric_positive_definite() {
        let (world, robot) = two_link_world(1.3, 0.9, 0.6, 0.4);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = [0.4, -0.7];
        let matrix = mass_matrix(&model, &q).expect("mass matrix");
        for row in 0..2 {
            for col in 0..2 {
                assert_relative_eq!(
                    matrix.get(row, col),
                    matrix.get(col, row),
                    epsilon = 1.0e-12
                );
            }
        }
        let probe = [0.7, -0.3];
        let image = matrix.mul_vec(&probe);
        let quadratic: f64 = probe.iter().zip(image.iter()).map(|(a, b)| a * b).sum();
        assert!(quadratic > 0.0);
    }

    #[test]
    fn floating_body_mass_matrix_is_its_spatial_inertia() {
        let com = Vec3::new(0.1, 0.2, 0.3);
        let (world, robot) = floating_body_world(3.0, com, [0.4, 0.5, 0.6]);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        assert_eq!(model.nv(), 6);
        let matrix = mass_matrix(&model, &[0.0; 6]).expect("mass matrix");
        let expected = crate::SpatialInertia::new(
            3.0,
            com,
            [[0.4, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.6]],
        )
        .matrix();
        for row in 0..6 {
            for col in 0..6 {
                assert_relative_eq!(matrix.get(row, col), expected[row][col], epsilon = 1.0e-12);
            }
        }
    }

    #[test]
    fn floating_body_gravity_wrench_matches_hand_derivation() {
        let com = Vec3::new(0.1, 0.2, 0.3);
        let (world, robot) = floating_body_world(3.0, com, [0.4, 0.5, 0.6]);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let tau = gravity_torque(&model, &[0.0; 6]).expect("gravity");
        let mass = 3.0;
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        // Generalized gravity force is `d V / d q` with `V = -m g . p`, so the
        // linear part opposes the gravity acceleration and the angular part is
        // the negative moment of the gravity force about the frame origin.
        let expected_force = -(gravity * mass);
        let expected_torque = -com.cross(gravity * mass);
        assert_relative_eq!(tau[0], expected_force.x, epsilon = 1.0e-10);
        assert_relative_eq!(tau[1], expected_force.y, epsilon = 1.0e-10);
        assert_relative_eq!(tau[2], expected_force.z, epsilon = 1.0e-10);
        assert_relative_eq!(tau[3], expected_torque.x, epsilon = 1.0e-10);
        assert_relative_eq!(tau[4], expected_torque.y, epsilon = 1.0e-10);
        assert_relative_eq!(tau[5], expected_torque.z, epsilon = 1.0e-10);
    }

    /// A two-link chain on a floating base with a massive base link.
    fn floating_two_link_world(
        m1: f64,
        m2: f64,
        l1: f64,
        l2: f64,
        base_mass: f64,
    ) -> (World, rne_ecs::Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let link2 = spawn_named(&mut world, "link2");
        let joint1 = spawn_named(&mut world, "joint1");
        let joint2 = spawn_named(&mut world, "joint2");

        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "floating_two_link".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            FloatingBase,
            RigidBody {
                mass_kg: base_mass,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::ZERO,
                ixx_kg_m2: 0.05,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.06,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.07,
            },
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: m1,
                ..RigidBody::default()
            },
            point_mass_inertia(Vec3::new(l1, 0.0, 0.0)),
        ));
        world.entity_mut(link2).insert((
            Link {
                robot,
                name: "link2".into(),
            },
            Transform3::from_translation_rotation(Vec3::new(l1, 0.0, 0.0), Quat::IDENTITY),
            RigidBody {
                mass_kg: m2,
                ..RigidBody::default()
            },
            point_mass_inertia(Vec3::new(l2, 0.0, 0.0)),
        ));
        world.entity_mut(joint1).insert(joint(robot, base, link1));
        world.entity_mut(joint2).insert(joint(robot, link1, link2));
        (world, robot)
    }

    fn assert_vec3_close(actual: Vec3, expected: Vec3, tolerance: f64) {
        let difference = (actual - expected).length();
        let scale = 1.0 + expected.length();
        assert!(
            difference <= tolerance * scale,
            "expected {expected:?}, got {actual:?} (|diff| {difference})"
        );
    }

    fn quaternion_log(quaternion: Quat) -> Vec3 {
        let q = quaternion.normalize();
        let (vector, scalar) = (Vec3::new(q.x, q.y, q.z), q.w);
        let length = vector.length();
        if length < 1.0e-12 {
            Vec3::ZERO
        } else {
            vector * (2.0 * length.atan2(scalar) / length)
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn link_pose_body_twist_derivatives_match_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let derivatives = link_pose_body_twist_derivatives(&model, &q).expect("derivatives");
        let nv = model.nv();
        let h = 1.0e-6;

        for link in 0..model.link_count() {
            for dof in 0..nv {
                let mut plus = q.clone();
                plus[dof] += h;
                let mut minus = q.clone();
                minus[dof] -= h;
                let pose = model
                    .kinematic
                    .forward_kinematics(&q)
                    .expect("fk")
                    .transform_at(link)
                    .copied()
                    .expect("pose");
                let pose_plus = model
                    .kinematic
                    .forward_kinematics(&plus)
                    .expect("fk")
                    .transform_at(link)
                    .copied()
                    .expect("pose");
                let pose_minus = model
                    .kinematic
                    .forward_kinematics(&minus)
                    .expect("fk")
                    .transform_at(link)
                    .copied()
                    .expect("pose");

                // Body twist: pose^-1 * dpose. The translation part is
                // R^-1 * d(translation); the rotation part is the vector of the
                // skew matrix R^-1 * d(R).
                let inverse_rotation = pose.rotation.conjugate();
                let translation_fd = inverse_rotation
                    * ((pose_plus.translation - pose_minus.translation) / (2.0 * h));
                let r = crate::spatial::rotation_matrix(pose.rotation);
                let r_plus = crate::spatial::rotation_matrix(pose_plus.rotation);
                let r_minus = crate::spatial::rotation_matrix(pose_minus.rotation);
                let mut delta = [[0.0_f64; 3]; 3];
                for row in 0..3 {
                    for column in 0..3 {
                        delta[row][column] =
                            (r_plus[row][column] - r_minus[row][column]) / (2.0 * h);
                    }
                }
                let body_delta =
                    crate::spatial::mat3_mul(&crate::spatial::mat3_transpose(&r), &delta);
                // Extract the skew vector of `R^-1 dR`: for a skew matrix `S`,
                // `S[2][1] = v.x`, `S[0][2] = v.y`, `S[1][0] = v.z`.
                let rotation_fd = Vec3::new(body_delta[2][1], body_delta[0][2], body_delta[1][0]);
                let finite_difference: SpatialVec = [
                    translation_fd.x,
                    translation_fd.y,
                    translation_fd.z,
                    rotation_fd.x,
                    rotation_fd.y,
                    rotation_fd.z,
                ];
                for component in 0..6 {
                    let error =
                        (derivatives[link][dof][component] - finite_difference[component]).abs();
                    assert!(
                        error < 1.0e-5,
                        "link {link} dof {dof} component {component}: {} vs {}",
                        derivatives[link][dof][component],
                        finite_difference[component]
                    );
                }
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn centroidal_momentum_matrix_reproduces_the_momentum() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let qd = vec![0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3];
        let matrix = centroidal_momentum_matrix(&model, &q).expect("matrix");
        let predicted = matrix.mul_vec(&qd);
        let momentum = centroidal_momentum(&model, &q, &qd).expect("momentum");
        for row in 0..6 {
            assert_relative_eq!(predicted[row], momentum[row], epsilon = 1.0e-9);
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn centroidal_momentum_bias_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let qd = vec![0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3];
        let qdd = vec![0.1, 0.05, -0.2, 0.12, 0.08, -0.15, 0.2, -0.1];
        let nv = model.nv();
        let matrix = centroidal_momentum_matrix(&model, &q).expect("matrix");
        let bias = centroidal_momentum_bias(&model, &q, &qd).expect("bias");
        let predicted = matrix.mul_vec(&qdd);
        let h = 1.0e-6;
        let plus_q: Vec<f64> = (0..nv).map(|i| q[i] + h * qd[i]).collect();
        let minus_q: Vec<f64> = (0..nv).map(|i| q[i] - h * qd[i]).collect();
        let plus_qd: Vec<f64> = (0..nv).map(|i| qd[i] + h * qdd[i]).collect();
        let minus_qd: Vec<f64> = (0..nv).map(|i| qd[i] - h * qdd[i]).collect();
        let plus = centroidal_momentum(&model, &plus_q, &plus_qd).expect("momentum");
        let minus = centroidal_momentum(&model, &minus_q, &minus_qd).expect("momentum");
        for row in 0..6 {
            let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
            assert_relative_eq!(
                predicted[row] + bias[row],
                finite_difference,
                epsilon = 1.0e-4,
                max_relative = 1.0e-4
            );
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn impulse_velocity_gradient_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let link = model.link_entity(2).expect("link");
        let contacts = vec![
            ContactSpec {
                link,
                point_local_m: Vec3::new(0.2, 0.0, 0.0),
            },
            ContactSpec {
                link,
                point_local_m: Vec3::new(0.0, 0.1, 0.0),
            },
        ];
        let gradient = impulse_velocity_gradient(&model, &q, &contacts).expect("gradient");
        let nv = model.nv();
        let qd = vec![0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3];
        let h = 1.0e-6;
        for column in 0..nv {
            let mut plus = qd.clone();
            plus[column] += h;
            let mut minus = qd.clone();
            minus[column] -= h;
            let (plus_impact, _) = impulse_velocity(&model, &q, &plus, &contacts).expect("impact");
            let (minus_impact, _) =
                impulse_velocity(&model, &q, &minus, &contacts).expect("impact");
            for row in 0..nv {
                let finite_difference = (plus_impact[row] - minus_impact[row]) / (2.0 * h);
                assert_relative_eq!(
                    gradient.get(row, column),
                    finite_difference,
                    epsilon = 1.0e-5,
                    max_relative = 1.0e-6
                );
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn forward_dynamics_gradient_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let qd = vec![0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3];
        let tau = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35, -0.2];
        let gradient = forward_dynamics_gradient(&model, &q, &qd, &tau).expect("gradient");
        let nv = model.nv();
        let h = 1.0e-6;

        for k in 0..nv {
            let mut q_plus = q.clone();
            q_plus[k] += h;
            let mut q_minus = q.clone();
            q_minus[k] -= h;
            let plus = forward_dynamics(&model, &q_plus, &qd, &tau).expect("qdd");
            let minus = forward_dynamics(&model, &q_minus, &qd, &tau).expect("qdd");
            for row in 0..nv {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                let error = (gradient.with_respect_to_q[k][row] - finite_difference).abs();
                assert!(
                    error < 1.0e-5,
                    "dqdd/dq {k} row {row}: {} vs {}",
                    gradient.with_respect_to_q[k][row],
                    finite_difference
                );
            }

            let mut qd_plus = qd.clone();
            qd_plus[k] += h;
            let mut qd_minus = qd.clone();
            qd_minus[k] -= h;
            let plus = forward_dynamics(&model, &q, &qd_plus, &tau).expect("qdd");
            let minus = forward_dynamics(&model, &q, &qd_minus, &tau).expect("qdd");
            for row in 0..nv {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                let error = (gradient.with_respect_to_qd[k][row] - finite_difference).abs();
                assert!(
                    error < 1.0e-5,
                    "dqdd/dqd {k} row {row}: {} vs {}",
                    gradient.with_respect_to_qd[k][row],
                    finite_difference
                );
            }
        }

        for j in 0..nv {
            let mut tau_plus = tau.clone();
            tau_plus[j] += h;
            let mut tau_minus = tau.clone();
            tau_minus[j] -= h;
            let plus = forward_dynamics(&model, &q, &qd, &tau_plus).expect("qdd");
            let minus = forward_dynamics(&model, &q, &qd, &tau_minus).expect("qdd");
            for row in 0..nv {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                let error = (gradient.with_respect_to_control[j][row] - finite_difference).abs();
                assert!(
                    error < 1.0e-5,
                    "dqdd/dtau {j} row {row}: {} vs {}",
                    gradient.with_respect_to_control[j][row],
                    finite_difference
                );
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn non_linear_effects_gradient_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let qd = vec![0.2, -0.1, 0.3, 0.15, -0.25, 0.1, 0.4, -0.3];
        let gradient = non_linear_effects_gradient(&model, &q, &qd).expect("gradient");
        let nv = model.nv();
        let h = 1.0e-6;

        for k in 0..nv {
            let mut q_plus = q.clone();
            q_plus[k] += h;
            let mut q_minus = q.clone();
            q_minus[k] -= h;
            let plus = non_linear_effects(&model, &q_plus, &qd).expect("bias");
            let minus = non_linear_effects(&model, &q_minus, &qd).expect("bias");
            for row in 0..nv {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                let error = (gradient.with_respect_to_q[k][row] - finite_difference).abs();
                assert!(
                    error < 1.0e-5,
                    "dh/dq dof {k} row {row}: {} vs {}",
                    gradient.with_respect_to_q[k][row],
                    finite_difference
                );
            }

            let mut qd_plus = qd.clone();
            qd_plus[k] += h;
            let mut qd_minus = qd.clone();
            qd_minus[k] -= h;
            let plus = non_linear_effects(&model, &q, &qd_plus).expect("bias");
            let minus = non_linear_effects(&model, &q, &qd_minus).expect("bias");
            for row in 0..nv {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                let error = (gradient.with_respect_to_qd[k][row] - finite_difference).abs();
                assert!(
                    error < 1.0e-5,
                    "dh/dqd dof {k} row {row}: {} vs {}",
                    gradient.with_respect_to_qd[k][row],
                    finite_difference
                );
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn mass_matrix_gradient_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let gradient = mass_matrix_gradient(&model, &q).expect("gradient");
        let nv = model.nv();
        let h = 1.0e-6;
        for dof in 0..nv {
            let mut plus = q.clone();
            plus[dof] += h;
            let mut minus = q.clone();
            minus[dof] -= h;
            let m_plus = mass_matrix(&model, &plus).expect("mass");
            let m_minus = mass_matrix(&model, &minus).expect("mass");
            for row in 0..nv {
                for column in 0..nv {
                    let finite_difference =
                        (m_plus.get(row, column) - m_minus.get(row, column)) / (2.0 * h);
                    let error = (gradient[dof].get(row, column) - finite_difference).abs();
                    assert!(
                        error < 1.0e-5,
                        "dof {dof} [{row}][{column}]: {} vs {}",
                        gradient[dof].get(row, column),
                        finite_difference
                    );
                }
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn xup_derivatives_match_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let derivatives = xup_derivatives(&model, &q).expect("derivatives");
        let nv = model.nv();
        let h = 1.0e-6;

        let xup_at = |values: &[f64]| {
            let kinematics = model.kinematic.forward_kinematics(values).expect("fk");
            xup_transforms(&model, kinematics.transforms())
        };

        for link in 1..model.link_count() {
            for dof in 0..nv {
                let mut plus = q.clone();
                plus[dof] += h;
                let mut minus = q.clone();
                minus[dof] -= h;
                let plus_xup = xup_at(&plus);
                let minus_xup = xup_at(&minus);
                for row in 0..6 {
                    for column in 0..6 {
                        let finite_difference = (plus_xup[link][row][column]
                            - minus_xup[link][row][column])
                            / (2.0 * h);
                        let error = (derivatives[link][dof][row][column] - finite_difference).abs();
                        assert!(
                            error < 1.0e-5,
                            "link {link} dof {dof} [{row}][{column}]: {} vs {}",
                            derivatives[link][dof][row][column],
                            finite_difference
                        );
                    }
                }
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn link_pose_derivatives_match_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let derivatives = link_pose_derivatives(&model, &q).expect("derivatives");
        let nv = model.nv();
        let h = 1.0e-6;

        for link in 0..model.link_count() {
            for dof in 0..nv {
                let mut plus = q.clone();
                plus[dof] += h;
                let mut minus = q.clone();
                minus[dof] -= h;
                let fk_plus = model.kinematic.forward_kinematics(&plus).expect("fk");
                let fk_minus = model.kinematic.forward_kinematics(&minus).expect("fk");
                let pose_plus = fk_plus.transform_at(link).expect("pose");
                let pose_minus = fk_minus.transform_at(link).expect("pose");

                let translation_fd = (pose_plus.translation - pose_minus.translation) / (2.0 * h);
                assert_vec3_close(derivatives[link].translation[dof], translation_fd, 1.0e-6);

                // World-frame rotation chart: log(R_plus * R_minus^T) is a
                // second-order-accurate difference of the rotation itself.
                let delta_rotation = pose_plus.rotation * pose_minus.rotation.conjugate();
                let rotation_fd = quaternion_log(delta_rotation) / (2.0 * h);
                assert_vec3_close(derivatives[link].rotation_vector[dof], rotation_fd, 1.0e-6);
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn floating_base_link_motions_match_frame_jacobian() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        assert_eq!(model.base_dof(), 6);
        let q = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let qd = vec![0.1, -0.05, 0.2, 0.3, -0.1, 0.15, 0.4, 0.6];
        let motions = link_motions(&model, &q, &qd).expect("link motions");

        // The spatial Jacobian applied to the body-twist generalized velocity
        // must reproduce the world link velocity for every link.
        for index in 0..model.link_count() {
            let entity = model.link_entity(index).expect("link entity");
            let jacobian = frame_jacobian(&model, &q, entity, Vec3::ZERO).expect("jacobian");
            for row in 0..6 {
                let mapped: f64 = (0..model.nv())
                    .map(|column| jacobian.get(row, column) * qd[column])
                    .sum();
                let expected = if row < 3 {
                    motions[index].linear_velocity_world_m_s.to_array()[row]
                } else {
                    motions[index].angular_velocity_world_rad_s.to_array()[row - 3]
                };
                assert_relative_eq!(mapped, expected, epsilon = 1.0e-9, max_relative = 1.0e-9);
            }
        }

        // The CoM Jacobian applied to the same twist must equal the mass-weighted
        // average of the link COM velocities read from `link_motions`.
        let com_jacobian = com_jacobian(&model, &q).expect("com jacobian");
        let mut expected = Vec3::ZERO;
        let mut total_mass = 0.0;
        for index in 0..model.link_count() {
            let Some(inertia) = model.link_inertia(index) else {
                continue;
            };
            if inertia.mass_kg == 0.0 {
                continue;
            }
            total_mass += inertia.mass_kg;
            let motion = &motions[index];
            let com_world = motion.world_transform.rotation * inertia.center_of_mass_m;
            let velocity = motion.linear_velocity_world_m_s
                + motion.angular_velocity_world_rad_s.cross(com_world);
            expected += velocity * inertia.mass_kg;
        }
        expected /= total_mass;
        for row in 0..3 {
            let mapped: f64 = (0..model.nv())
                .map(|column| com_jacobian.get(row, column) * qd[column])
                .sum();
            assert_relative_eq!(
                mapped,
                expected.to_array()[row],
                epsilon = 1.0e-9,
                max_relative = 1.0e-9
            );
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn floating_base_point_bias_acceleration_matches_finite_difference() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model =
            ArticulatedModel::from_robot_with_gravity(&world, robot, Vec3::ZERO).expect("model");
        let q0 = vec![0.3, -0.2, 0.1, 0.4, -0.3, 0.25, 0.5, -0.7];
        let spatial_qd = vec![0.15, -0.1, 0.2, 0.2, -0.15, 0.1, 0.4, -0.3];

        // The model chart advances at `map * spatial_qd` for the base and the
        // joint rates directly for the joints; holding the spatial twist constant
        // makes the spatial acceleration zero, so a central difference of the
        // point velocity is exactly the bias (Jdot * qd) term.
        let chart_base_rate = |q: &[f64]| -> Vec<f64> {
            let kinematics = model.kinematic.forward_kinematics(q).expect("kinematics");
            let map = base_velocity_map(q, kinematics.transforms()[0].rotation);
            (0..6)
                .map(|row| (0..6).map(|k| map[row][k] * spatial_qd[k]).sum())
                .collect()
        };
        let advance = |q: &[f64], dt: f64| -> Vec<f64> {
            let mut next = q.to_vec();
            let base = chart_base_rate(q);
            for index in 0..6 {
                next[index] += dt * base[index];
            }
            for index in 6..8 {
                next[index] += dt * spatial_qd[index];
            }
            next
        };

        let h = 1.0e-6;
        let q_plus = advance(&q0, h);
        let q_minus = advance(&q0, -h);
        let motions = link_motions(&model, &q0, &spatial_qd).expect("motions");
        for index in 0..model.link_count() {
            let entity = model.link_entity(index).expect("link");
            let point = model
                .link_inertia(index)
                .map(|inertia| inertia.center_of_mass_m)
                .unwrap_or(Vec3::ZERO);
            let velocity = |q: &[f64]| -> [f64; 6] {
                let jacobian = frame_jacobian(&model, q, entity, point).expect("jacobian");
                let mut out = [0.0; 6];
                for row in 0..6 {
                    out[row] = (0..model.nv())
                        .map(|column| jacobian.get(row, column) * spatial_qd[column])
                        .sum();
                }
                out
            };
            let plus = velocity(&q_plus);
            let minus = velocity(&q_minus);
            let motion = &motions[index];
            let expected_linear = motion.point_bias_acceleration_m_s2(point).to_array();
            let expected_angular = motion.angular_acceleration_world_rad_s2.to_array();
            for row in 0..3 {
                let finite_difference = (plus[row] - minus[row]) / (2.0 * h);
                assert_relative_eq!(
                    finite_difference,
                    expected_linear[row],
                    epsilon = 1.0e-5,
                    max_relative = 1.0e-5
                );
                let finite_difference = (plus[3 + row] - minus[3 + row]) / (2.0 * h);
                assert_relative_eq!(
                    finite_difference,
                    expected_angular[row],
                    epsilon = 1.0e-5,
                    max_relative = 1.0e-5
                );
            }
        }
    }

    #[test]
    fn floating_body_centroidal_momentum_matches_hand_derivation() {
        let mass = 3.0;
        let com = Vec3::new(0.1, -0.2, 0.3);
        let tensor = [0.4, 0.5, 0.6];
        let (world, robot) = floating_body_world(mass, com, tensor);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let angular_velocity = Vec3::new(0.3, -0.2, 0.1);
        let qd = [
            0.0,
            0.0,
            0.0,
            angular_velocity.x,
            angular_velocity.y,
            angular_velocity.z,
        ];
        let momentum = centroidal_momentum(&model, &[0.0; 6], &qd).expect("momentum");

        // With the origin at rest, the CoM velocity is `omega x com`, and the
        // angular momentum about the CoM is `I_com * omega`.
        let expected_linear = angular_velocity.cross(com) * mass;
        let expected_angular = Vec3::new(
            tensor[0] * angular_velocity.x,
            tensor[1] * angular_velocity.y,
            tensor[2] * angular_velocity.z,
        );
        for row in 0..3 {
            assert_relative_eq!(
                momentum[row],
                expected_linear.to_array()[row],
                epsilon = 1.0e-10
            );
            assert_relative_eq!(
                momentum[3 + row],
                expected_angular.to_array()[row],
                epsilon = 1.0e-10
            );
        }
    }

    #[test]
    fn free_floating_body_matches_newton_euler() {
        let mass = 3.0;
        let tensor = [0.4, 0.5, 0.6];
        let (world, robot) = floating_body_world(mass, Vec3::ZERO, tensor);
        let model =
            ArticulatedModel::from_robot_with_gravity(&world, robot, Vec3::ZERO).expect("model");
        let q = vec![0.0; 6];
        let linear = Vec3::new(0.3, -0.2, 0.1);
        let angular = Vec3::new(0.2, 0.4, -0.3);
        let qd = vec![
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
        ];

        // With no external wrench, a free rigid body obeys
        //   m (dv/dt + w x v) = 0            => dv/dt = -w x v
        //   I dw/dt + w x (I w) = 0          => dw/dt = -I^-1 (w x I w)
        let qdd = forward_dynamics(&model, &q, &qd, &[0.0; 6]).expect("forward dynamics");
        let inertia = Vec3::new(tensor[0], tensor[1], tensor[2]);
        let angular_momentum = Vec3::new(
            inertia.x * angular.x,
            inertia.y * angular.y,
            inertia.z * angular.z,
        );
        let couple = angular.cross(angular_momentum);
        let expected_linear = -angular.cross(linear);
        let expected_angular = -Vec3::new(
            couple.x / inertia.x,
            couple.y / inertia.y,
            couple.z / inertia.z,
        );
        for row in 0..3 {
            assert_relative_eq!(qdd[row], expected_linear.to_array()[row], epsilon = 1.0e-10);
            assert_relative_eq!(
                qdd[3 + row],
                expected_angular.to_array()[row],
                epsilon = 1.0e-10
            );
        }
    }

    #[test]
    fn integrate_configuration_maps_body_twist_into_the_euler_chart() {
        let (world, robot) = floating_two_link_world(2.0, 1.5, 0.7, 0.5, 4.0);
        let model =
            ArticulatedModel::from_robot_with_gravity(&world, robot, Vec3::ZERO).expect("model");
        let q = vec![0.1, -0.2, 0.3, 0.2, -0.1, 0.15, 0.3, -0.4];
        let omega_body = Vec3::new(0.3, -0.2, 0.25);
        let linear_body = Vec3::new(0.05, -0.02, 0.01);
        let mut qd = vec![0.0; model.nv()];
        qd[0] = linear_body.x;
        qd[1] = linear_body.y;
        qd[2] = linear_body.z;
        qd[3] = omega_body.x;
        qd[4] = omega_body.y;
        qd[5] = omega_body.z;
        let dt = 1.0e-9;
        let next = integrate_configuration(&model, &q, &qd, &vec![0.0; model.nv()], dt)
            .expect("integrate");

        let fk = model.kinematic().forward_kinematics(&q).expect("fk");
        let base_rotation = fk.transforms()[0].rotation;
        // Translation advances by the world-frame body velocity.
        let expected_translation = base_rotation * (linear_body * dt);
        for row in 0..3 {
            assert_relative_eq!(
                next[row] - q[row],
                expected_translation.to_array()[row],
                epsilon = 1.0e-9
            );
        }
        // The orientation change is the world angular velocity `R * omega_body`.
        let next_fk = model.kinematic().forward_kinematics(&next).expect("fk");
        let delta = base_rotation.inverse() * next_fk.transforms()[0].rotation;
        let (axis, angle) = delta.to_axis_angle();
        let rotation_vector = axis * angle;
        let expected_angular = base_rotation * (omega_body * dt);
        for row in 0..3 {
            assert_relative_eq!(
                rotation_vector.to_array()[row],
                expected_angular.to_array()[row],
                epsilon = 1.0e-8,
                max_relative = 1.0e-4
            );
        }
    }

    #[test]
    fn double_pendulum_forward_dynamics_conserves_energy() {
        let (world, robot) = two_link_world(1.0, 1.0, 0.5, 0.5);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let total_mass: f64 = (0..model.link_count())
            .filter_map(|index| model.link_inertia(index))
            .map(|inertia| inertia.mass_kg)
            .sum();

        let energy = |q: &[f64], qd: &[f64]| {
            let matrix = mass_matrix(&model, q).expect("mass matrix");
            let kinetic: f64 = qd
                .iter()
                .enumerate()
                .map(|(row, value)| {
                    value
                        * (0..qd.len())
                            .map(|col| matrix.get(row, col) * qd[col])
                            .sum::<f64>()
                })
                .sum::<f64>()
                * 0.5;
            let com = center_of_mass(&model, q).expect("com");
            let potential = -model.gravity_m_s2().dot(com) * total_mass;
            kinetic + potential
        };

        let mut q = vec![0.6, -0.4];
        let mut qd = vec![0.0, 0.0];
        let initial = energy(&q, &qd);
        let dt = 1.0e-4;
        let steps = 5_000;
        for _ in 0..steps {
            let acceleration = |q: &[f64], qd: &[f64]| {
                forward_dynamics(&model, q, qd, &[0.0, 0.0]).expect("forward dynamics")
            };
            let k1_q = qd.clone();
            let k1_v = acceleration(&q, &qd);
            let q2: Vec<f64> = q.iter().zip(&k1_q).map(|(a, b)| a + 0.5 * dt * b).collect();
            let v2: Vec<f64> = qd
                .iter()
                .zip(&k1_v)
                .map(|(a, b)| a + 0.5 * dt * b)
                .collect();
            let k2_q = v2.clone();
            let k2_v = acceleration(&q2, &v2);
            let q3: Vec<f64> = q.iter().zip(&k2_q).map(|(a, b)| a + 0.5 * dt * b).collect();
            let v3: Vec<f64> = qd
                .iter()
                .zip(&k2_v)
                .map(|(a, b)| a + 0.5 * dt * b)
                .collect();
            let k3_q = v3.clone();
            let k3_v = acceleration(&q3, &v3);
            let q4: Vec<f64> = q.iter().zip(&k3_q).map(|(a, b)| a + dt * b).collect();
            let v4: Vec<f64> = qd.iter().zip(&k3_v).map(|(a, b)| a + dt * b).collect();
            let k4_q = v4.clone();
            let k4_v = acceleration(&q4, &v4);
            for index in 0..q.len() {
                q[index] +=
                    dt / 6.0 * (k1_q[index] + 2.0 * k2_q[index] + 2.0 * k3_q[index] + k4_q[index]);
                qd[index] +=
                    dt / 6.0 * (k1_v[index] + 2.0 * k2_v[index] + 2.0 * k3_v[index] + k4_v[index]);
            }
        }
        let final_energy = energy(&q, &qd);
        let relative = ((final_energy - initial) / initial.abs()).abs();
        assert!(relative < 1.0e-6, "energy drift {relative}");
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn impulse_velocity_stops_incoming_contacts() {
        let (world, robot) = floating_body_world(3.0, Vec3::ZERO, [0.1, 0.1, 0.1]);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let base = world.get::<Robot>(robot).expect("robot").base_link;
        let contacts = [
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.2, 0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.2, -0.1),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(-0.1, -0.2, 0.0),
            },
        ];
        let q = vec![0.0; 6];
        let qd_minus = vec![0.0, -2.0, 0.0, 0.0, 0.0, 0.0];
        let (qd_plus, impulses) =
            impulse_velocity(&model, &q, &qd_minus, &contacts).expect("impulse");
        assert!(qd_plus[1] > qd_minus[1], "downward velocity not arrested");
        assert!(impulses.iter().all(|impulse| impulse.is_finite()));
        for spec in &contacts {
            let jac = frame_jacobian(&model, &q, spec.link, spec.point_local_m).expect("jacobian");
            for row in 0..3 {
                let velocity: f64 = (0..6)
                    .map(|column| jac.get(row, column) * qd_plus[column])
                    .sum();
                assert!(velocity.abs() < 1.0e-6, "contact velocity {velocity}");
            }
        }
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn constrained_dynamics_keeps_contacts_stationary() {
        let (world, robot) = floating_body_world(3.0, Vec3::ZERO, [0.1, 0.1, 0.1]);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let base = world
            .get::<Robot>(robot)
            .expect("robot component")
            .base_link;
        let contacts = [
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(0.1, -0.2, 0.0),
            },
            ContactSpec {
                link: base,
                point_local_m: Vec3::new(-0.1, -0.2, 0.0),
            },
        ];
        let q = vec![0.0; 6];
        let qd = vec![0.0; 6];
        let tau = vec![0.0; 6];
        let (qdd, forces) =
            constrained_forward_dynamics(&model, &q, &qd, &tau, &contacts).expect("dynamics");
        assert!(qdd.iter().all(|value| value.abs() < 1.0e-6));
        let total_vertical: f64 = forces.iter().map(|force| force.y).sum();
        assert_relative_eq!(total_vertical, 3.0 * 9.81, epsilon = 0.5);

        let motions = link_motions(&model, &q, &qd).expect("motions");
        for spec in &contacts {
            let jac = frame_jacobian(&model, &q, spec.link, spec.point_local_m).expect("jacobian");
            let link_index = model.kinematic.link_index(spec.link).expect("index");
            let bias = motions[link_index].point_bias_acceleration_m_s2(spec.point_local_m);
            let bias = [bias.x, bias.y, bias.z];
            for row in 0..3 {
                let acceleration: f64 = (0..6)
                    .map(|column| jac.get(row, column) * qdd[column])
                    .sum::<f64>()
                    + bias[row];
                assert!(acceleration.abs() < 1.0e-6, "contact accel {acceleration}");
            }
        }
    }

    #[test]
    fn link_motion_bias_matches_centripetal_acceleration() {
        let (world, robot) = two_link_world(1.0, 1.0, 0.5, 0.5);
        let model = ArticulatedModel::from_robot(&world, robot).expect("model");
        let motions = link_motions(&model, &[0.0, 0.0], &[2.0, 0.0]).expect("motions");
        let link1 = motions[1];
        let acceleration = link1.point_bias_acceleration_m_s2(Vec3::new(0.5, 0.0, 0.0));
        // omega = 2 * z, r = 0.5 * x, so the centripetal bias is -omega^2 r.
        assert_relative_eq!(acceleration.x, -2.0, epsilon = 1.0e-10);
        assert!(acceleration.y.abs() < 1.0e-10);
        assert!(acceleration.z.abs() < 1.0e-10);
        assert_relative_eq!(link1.angular_velocity_world_rad_s.z, 2.0, epsilon = 1.0e-10);
    }
}
