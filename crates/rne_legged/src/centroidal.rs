//! Centroidal (single rigid body) control for dynamic maneuvers.
//!
//! This is the classical model-based layer above the walking templates: it
//! treats the robot as one rigid body and reasons about the net ground-reaction
//! wrench that realizes a desired center-of-mass and angular acceleration,
//! distributes that wrench across the active contact points subject to a
//! Coulomb friction cone, and places swing feet with the Raibert heuristic and
//! a minimal-jerk swing trajectory. It is the abstraction needed for
//! flight phases, where no contact exists and the center of mass follows a
//! ballistic arc.
//!
//! The formulation follows the open-source centroidal controllers in
//! `yxyang/cajun` (Centroidal QP controller) and `go2-convex-mpc`
//! (contact-force optimization with Raibert foot placement and quintic swing).

use rne_math::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Row-major `3x3` matrix.
pub type Mat3 = [[f64; 3]; 3];

/// A single-rigid-body centroidal model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentroidalModel {
    /// Total mass in kilograms.
    pub mass_kg: f64,
    /// Inertia tensor about the center of mass in the body frame, in kg·m².
    pub inertia_body_kg_m2: Mat3,
    /// Gravity vector in world coordinates, in m/s².
    pub gravity_m_s2: Vec3,
}

/// Centroidal state of the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentroidalState {
    /// Center-of-mass position in world coordinates, in meters.
    pub com_position_m: Vec3,
    /// Center-of-mass velocity in world coordinates, in m/s.
    pub com_velocity_m_s: Vec3,
    /// Body orientation as a world-from-body rotation.
    pub orientation: Quat,
    /// Body angular velocity in world coordinates, in rad/s.
    pub angular_velocity_rad_s: Vec3,
}

/// Desired centroidal acceleration to realize.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentroidalTarget {
    /// Desired center-of-mass acceleration in world coordinates, in m/s².
    pub com_acceleration_m_s2: Vec3,
    /// Desired angular acceleration in world coordinates, in rad/s².
    pub angular_acceleration_rad_s: Vec3,
}

/// One active ground contact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundContact {
    /// Contact position in world coordinates, in meters.
    pub position_m: Vec3,
    /// Outward contact normal in world coordinates; normalized internally.
    pub normal_world: Vec3,
    /// Coulomb friction coefficient.
    pub friction_coefficient: f64,
}

/// Weights for the contact-force distribution least-squares problem.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactAllocationConfig {
    /// Weight on matching the net linear force.
    pub linear_weight: f64,
    /// Weight on matching the net moment.
    pub moment_weight: f64,
    /// Tikhonov weight on the contact forces.
    pub force_regularization: f64,
    /// Diagonal added to the normal equations for numerical rank.
    pub solver_regularization: f64,
}

impl Default for ContactAllocationConfig {
    fn default() -> Self {
        Self {
            linear_weight: 1.0,
            moment_weight: 1.0,
            force_regularization: 1.0e-3,
            solver_regularization: 1.0e-9,
        }
    }
}

/// Result of a contact-force distribution solve.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactForceSolution {
    /// Friction-projected contact forces in world coordinates, in newtons.
    pub forces_world_n: Vec<Vec3>,
    /// Residual of `sum(f) - m (a - g)`, in newtons.
    pub linear_residual_n: Vec3,
    /// Residual of `sum(r x f) - M_des`, in newton-meters.
    pub moment_residual_nm: Vec3,
}

/// Distributed a desired centroidal wrench across the active contacts.
pub fn distribute_contact_forces(
    model: &CentroidalModel,
    state: &CentroidalState,
    contacts: &[GroundContact],
    target: &CentroidalTarget,
    config: &ContactAllocationConfig,
) -> ContactForceSolution {
    let contact_count = contacts.len();
    if contact_count == 0 {
        return ContactForceSolution {
            forces_world_n: Vec::new(),
            linear_residual_n: -(model.mass_kg
                * (target.com_acceleration_m_s2 - model.gravity_m_s2)),
            moment_residual_nm: Vec3::ZERO,
        };
    }

    let rotation = rotation_matrix(state.orientation);
    let inertia_world = rotate_inertia(&rotation, &model.inertia_body_kg_m2);
    let desired_force = model.mass_kg * (target.com_acceleration_m_s2 - model.gravity_m_s2);
    let angular_momentum = mat3_mul_vec(&inertia_world, state.angular_velocity_rad_s);
    let desired_moment = mat3_mul_vec(&inertia_world, target.angular_acceleration_rad_s)
        + state.angular_velocity_rad_s.cross(angular_momentum);

    let cols = 3 * contact_count;
    let mut rows: Vec<Vec<f64>> = Vec::new();
    let mut rhs: Vec<f64> = Vec::new();

    // Linear rows: sum(f) = m (a - g).
    let linear_scale = config.linear_weight.sqrt();
    for row in 0..3 {
        let mut coefficients = vec![0.0; cols];
        for contact in 0..contact_count {
            coefficients[3 * contact + row] = 1.0;
        }
        push_row(
            &mut rows,
            &mut rhs,
            coefficients,
            desired_force[row],
            linear_scale,
        );
    }

    // Moment rows: sum((p - c) x f) = M_des.
    let moment_scale = config.moment_weight.sqrt();
    for row in 0..3 {
        let mut coefficients = vec![0.0; cols];
        for (contact, point) in contacts.iter().enumerate() {
            let r = point.position_m - state.com_position_m;
            let lever = [[0.0, -r.z, r.y], [r.z, 0.0, -r.x], [-r.y, r.x, 0.0]];
            for component in 0..3 {
                coefficients[3 * contact + component] = lever[row][component];
            }
        }
        push_row(
            &mut rows,
            &mut rhs,
            coefficients,
            desired_moment[row],
            moment_scale,
        );
    }

    // Force regularization.
    let regularization_scale = config.force_regularization.sqrt();
    for variable in 0..cols {
        let mut coefficients = vec![0.0; cols];
        coefficients[variable] = 1.0;
        push_row(&mut rows, &mut rhs, coefficients, 0.0, regularization_scale);
    }

    let solution = solve_least_squares(&rows, &rhs, cols, config.solver_regularization)
        .unwrap_or_else(|| vec![0.0; cols]);

    let mut forces_world_n = Vec::with_capacity(contact_count);
    for (contact, point) in contacts.iter().enumerate() {
        let force = Vec3::new(
            solution[3 * contact],
            solution[3 * contact + 1],
            solution[3 * contact + 2],
        );
        forces_world_n.push(project_friction(force, point));
    }

    let mut net_force = Vec3::ZERO;
    let mut net_moment = Vec3::ZERO;
    for (contact, point) in contacts.iter().enumerate() {
        let force = forces_world_n[contact];
        net_force += force;
        net_moment += (point.position_m - state.com_position_m).cross(force);
    }

    ContactForceSolution {
        forces_world_n,
        linear_residual_n: net_force - desired_force,
        moment_residual_nm: net_moment - desired_moment,
    }
}

/// Raibert-style swing-foot placement.
///
/// `stance_center_m` is the nominal foot position under the body; the returned
/// foot position leads the body by half the stance time plus a velocity-error
/// term.
pub fn raibert_foot_placement(
    stance_center_m: Vec3,
    com_velocity_m_s: Vec3,
    desired_velocity_m_s: Vec3,
    stance_time_s: f64,
    velocity_gain_s: f64,
) -> Vec3 {
    stance_center_m
        + com_velocity_m_s * (0.5 * stance_time_s)
        + (com_velocity_m_s - desired_velocity_m_s) * velocity_gain_s
}

/// Minimal-jerk quintic swing trajectory between two footholds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwingTrajectory {
    /// Liftoff position in world coordinates, in meters.
    pub start_m: Vec3,
    /// Touchdown position in world coordinates, in meters.
    pub end_m: Vec3,
    /// Additional apex height above the straight line, in meters.
    pub apex_height_m: f64,
    /// Swing duration in seconds.
    pub duration_s: f64,
}

impl SwingTrajectory {
    /// Samples position and velocity at time `time_s`.
    pub fn sample(&self, time_s: f64) -> (Vec3, Vec3) {
        if self.duration_s <= 0.0 {
            return (self.end_m, Vec3::ZERO);
        }
        let u = (time_s / self.duration_s).clamp(0.0, 1.0);
        let u2 = u * u;
        let u3 = u2 * u;
        let u4 = u3 * u;
        let u5 = u4 * u;
        let s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
        let ds_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
        let delta = self.end_m - self.start_m;
        let position = self.start_m
            + delta * s
            + Vec3::Y * (self.apex_height_m * (std::f64::consts::PI * u).sin());
        let velocity = (delta * (ds_du / self.duration_s))
            + Vec3::Y
                * (self.apex_height_m * std::f64::consts::PI * (std::f64::consts::PI * u).cos()
                    / self.duration_s);
        (position, velocity)
    }
}

/// Apex height `v0^2 / (2 g)` of a ballistic flight, in meters.
pub fn flight_apex_height_m(takeoff_speed_m_s: f64, gravity_m_s2: f64) -> f64 {
    if gravity_m_s2 <= 0.0 {
        return 0.0;
    }
    takeoff_speed_m_s * takeoff_speed_m_s / (2.0 * gravity_m_s2)
}

/// Total ballistic flight time `2 v0 / g`, in seconds.
pub fn flight_duration_s(takeoff_speed_m_s: f64, gravity_m_s2: f64) -> f64 {
    if gravity_m_s2 <= 0.0 {
        return 0.0;
    }
    2.0 * takeoff_speed_m_s / gravity_m_s2
}

fn project_friction(force_world_n: Vec3, contact: &GroundContact) -> Vec3 {
    let normal = contact.normal_world.normalize_or_zero();
    if normal.length_squared() <= 1.0e-9 {
        return force_world_n;
    }
    let normal_force = force_world_n.dot(normal).max(0.0);
    let tangential = force_world_n - normal * force_world_n.dot(normal);
    let max_tangential = contact.friction_coefficient * normal_force;
    let tangential_norm = tangential.length();
    let tangential = if tangential_norm > max_tangential && tangential_norm > 0.0 {
        tangential * (max_tangential / tangential_norm)
    } else {
        tangential
    };
    normal * normal_force + tangential
}

fn push_row(
    rows: &mut Vec<Vec<f64>>,
    rhs: &mut Vec<f64>,
    coefficients: Vec<f64>,
    target: f64,
    scale: f64,
) {
    rows.push(
        coefficients
            .into_iter()
            .map(|value| value * scale)
            .collect(),
    );
    rhs.push(target * scale);
}

// Index feeds multiple parallel arrays/matrix slots keyed by the same position; an iterator adapter would obscure the indexing.
#[allow(clippy::needless_range_loop)]
fn solve_least_squares(
    rows: &[Vec<f64>],
    rhs: &[f64],
    cols: usize,
    regularization: f64,
) -> Option<Vec<f64>> {
    let mut normal = vec![vec![0.0; cols]; cols];
    let mut normal_rhs = vec![0.0; cols];
    for (row, target) in rows.iter().zip(rhs) {
        for i in 0..cols {
            if row[i] == 0.0 {
                continue;
            }
            normal_rhs[i] += row[i] * target;
            for j in i..cols {
                normal[i][j] += row[i] * row[j];
            }
        }
    }
    for i in 0..cols {
        for j in 0..i {
            normal[i][j] = normal[j][i];
        }
        normal[i][i] += regularization;
    }
    solve_linear(normal, normal_rhs)
}

fn solve_linear(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Option<Vec<f64>> {
    let n = rhs.len();
    for col in 0..n {
        let pivot =
            (col..n).max_by(|&a, &b| matrix[a][col].abs().total_cmp(&matrix[b][col].abs()))?;
        if matrix[pivot][col].abs() < 1.0e-12 {
            return None;
        }
        matrix.swap(col, pivot);
        rhs.swap(col, pivot);
        let pivot_row = matrix[col].clone();
        let diagonal = pivot_row[col];
        for row in (col + 1)..n {
            let factor = matrix[row][col] / diagonal;
            for (target, source) in matrix[row].iter_mut().zip(&pivot_row).skip(col) {
                *target -= factor * source;
            }
            rhs[row] -= factor * rhs[col];
        }
    }
    let mut solution = vec![0.0; n];
    for row in (0..n).rev() {
        let known: f64 = (row + 1..n)
            .map(|col| matrix[row][col] * solution[col])
            .sum();
        solution[row] = (rhs[row] - known) / matrix[row][row];
    }
    Some(solution)
}

fn rotation_matrix(rotation: Quat) -> Mat3 {
    let q = rotation.normalize();
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn transpose(matrix: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = matrix[column][row];
        }
    }
    out
}

fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = (0..3).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

fn mat3_mul_vec(matrix: &Mat3, vector: Vec3) -> Vec3 {
    Vec3::new(
        matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
    )
}

fn rotate_inertia(rotation: &Mat3, inertia_body: &Mat3) -> Mat3 {
    mat3_mul(&mat3_mul(rotation, inertia_body), &transpose(rotation))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn model() -> CentroidalModel {
        CentroidalModel {
            mass_kg: 3.0,
            inertia_body_kg_m2: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
        }
    }

    fn state() -> CentroidalState {
        CentroidalState {
            com_position_m: Vec3::new(0.0, 0.3, 0.0),
            com_velocity_m_s: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            angular_velocity_rad_s: Vec3::ZERO,
        }
    }

    #[test]
    fn four_contacts_support_the_body_weight() {
        let contacts = [
            GroundContact {
                position_m: Vec3::new(0.15, 0.0, 0.1),
                normal_world: Vec3::Y,
                friction_coefficient: 0.6,
            },
            GroundContact {
                position_m: Vec3::new(0.15, 0.0, -0.1),
                normal_world: Vec3::Y,
                friction_coefficient: 0.6,
            },
            GroundContact {
                position_m: Vec3::new(-0.15, 0.0, 0.1),
                normal_world: Vec3::Y,
                friction_coefficient: 0.6,
            },
            GroundContact {
                position_m: Vec3::new(-0.15, 0.0, -0.1),
                normal_world: Vec3::Y,
                friction_coefficient: 0.6,
            },
        ];
        let solution = distribute_contact_forces(
            &model(),
            &state(),
            &contacts,
            &CentroidalTarget {
                com_acceleration_m_s2: Vec3::ZERO,
                angular_acceleration_rad_s: Vec3::ZERO,
            },
            &ContactAllocationConfig::default(),
        );
        let total: Vec3 = solution.forces_world_n.iter().copied().sum();
        assert_relative_eq!(total.y, 3.0 * 9.81, epsilon = 0.5);
        assert!(solution.forces_world_n.iter().all(|force| force.y > 0.0));
        assert!(solution.linear_residual_n.length() < 0.5);
        assert!(solution.moment_residual_nm.length() < 0.2);
    }

    #[test]
    fn friction_limits_tangential_force() {
        let contacts = [GroundContact {
            position_m: Vec3::new(0.0, 0.0, 0.0),
            normal_world: Vec3::Y,
            friction_coefficient: 0.3,
        }];
        // Asking for a large lateral acceleration forces a tangential request
        // beyond the cone, so the projection must clamp it.
        let solution = distribute_contact_forces(
            &model(),
            &state(),
            &contacts,
            &CentroidalTarget {
                com_acceleration_m_s2: Vec3::new(8.0, 0.0, 0.0),
                angular_acceleration_rad_s: Vec3::ZERO,
            },
            &ContactAllocationConfig::default(),
        );
        let force = solution.forces_world_n[0];
        assert!(force.y >= 0.0);
        assert!(force.x.abs() <= 0.3 * force.y + 1.0e-9);
    }

    #[test]
    fn raibert_places_the_foot_ahead_when_moving_forward() {
        let foot = raibert_foot_placement(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.3,
            0.05,
        );
        assert!(foot.x > 0.0);
        assert_relative_eq!(foot.x, 1.0 * 0.15, epsilon = 1.0e-9);
    }

    #[test]
    fn swing_trajectory_hits_endpoints_and_apex() {
        let swing = SwingTrajectory {
            start_m: Vec3::new(0.0, 0.0, 0.0),
            end_m: Vec3::new(0.2, 0.0, 0.0),
            apex_height_m: 0.05,
            duration_s: 0.3,
        };
        let (start, _) = swing.sample(0.0);
        let (mid, _) = swing.sample(0.15);
        let (end, _) = swing.sample(0.3);
        assert_relative_eq!(start.x, 0.0, epsilon = 1.0e-12);
        assert_relative_eq!(end.x, 0.2, epsilon = 1.0e-12);
        assert!((mid.y - 0.05).abs() < 1.0e-9);
    }

    #[test]
    fn flight_helpers_match_ballistics() {
        let apex = flight_apex_height_m(1.5, 9.81);
        let duration = flight_duration_s(1.5, 9.81);
        assert_relative_eq!(apex, 1.5 * 1.5 / (2.0 * 9.81), epsilon = 1.0e-12);
        assert_relative_eq!(duration, 2.0 * 1.5 / 9.81, epsilon = 1.0e-12);
    }
}
