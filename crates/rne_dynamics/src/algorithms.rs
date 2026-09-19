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
use rne_math::Vec3;
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

/// Gravity and velocity bias `C(q, qd) qd + g(q)`.
pub fn non_linear_effects(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
) -> Result<Vec<f64>, DynamicsError> {
    rnea(model, q, qd, &vec![0.0; model.nv()])
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
