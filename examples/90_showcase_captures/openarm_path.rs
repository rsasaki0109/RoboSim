//! Cartesian arm paths for the OpenArm showcase.
//!
//! The keyposes are inverse-kinematics solutions that put each gripper where
//! it must be: over the block, on it, at the relay point, on the pad. Blending
//! straight between them in *joint* space moved the hands along whatever arc
//! the seven joints happened to trace, swung elbows up and out, and reached
//! across the torso on the way. Here only the keyposes' gripper poses are kept.
//! The gripper moves between them in a straight line (position) and a slerp
//! (orientation) on a smooth time profile, and every control step is solved by
//! damped least squares, warm-started from the step before, with the arm's
//! spare degree of freedom pulled toward a hanging, elbow-down posture.

use anyhow::{Context, Result};
use rne_ai::UrdfSceneSim;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_robot::{KinematicModel, Robot};

/// The seven arm joints solved here; the finger joints follow the keyposes.
pub(crate) const ARM_JOINTS: usize = 7;

/// One arm's kinematics, gripper link and preferred posture.
pub(crate) struct ArmSolver {
    model: KinematicModel,
    gripper: Entity,
    limits: Vec<(f64, f64)>,
    rest: [f64; ARM_JOINTS],
}

/// A gripper pose: position and orientation in the world.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GripperPose {
    pub(crate) position: Vec3,
    pub(crate) rotation: Quat,
}

impl ArmSolver {
    /// The solver for the robot named `robot_name`, driving `gripper_link`,
    /// preferring `rest` in its null space.
    pub(crate) fn new(
        sim: &UrdfSceneSim,
        robot_name: &str,
        gripper_link: &str,
        rest: [f64; ARM_JOINTS],
    ) -> Result<Self> {
        let robot = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                entity
                    .get::<Robot>()
                    .filter(|robot| robot.model_name == robot_name)
                    .map(|_| entity.id())
            })
            .with_context(|| format!("no robot {robot_name}"))?;
        let model = KinematicModel::from_robot(sim.world(), robot)
            .with_context(|| format!("kinematic model of {robot_name}"))?;
        let gripper = model
            .link_entity_by_name(gripper_link)
            .with_context(|| format!("no link {gripper_link}"))?;
        let limits = model
            .joint_limits()
            .iter()
            .take(ARM_JOINTS)
            .map(|limit| (limit.lower, limit.upper))
            .collect();
        Ok(Self {
            model,
            gripper,
            limits,
            rest,
        })
    }

    fn full(&self, q: &[f64; ARM_JOINTS]) -> Vec<f64> {
        let mut full = vec![0.0; self.model.dof()];
        full[..ARM_JOINTS].copy_from_slice(q);
        full
    }

    /// The gripper pose at joint positions `q`.
    pub(crate) fn gripper_pose(&self, q: &[f64; ARM_JOINTS]) -> Result<GripperPose> {
        let fk = self.model.forward_kinematics(&self.full(q))?;
        let transform = fk
            .link_transform(self.gripper)
            .context("gripper transform")?;
        Ok(GripperPose {
            position: transform.translation,
            rotation: transform.rotation,
        })
    }

    /// Joint positions that put the gripper at `target`, starting from `seed`.
    pub(crate) fn solve(
        &self,
        target: GripperPose,
        seed: [f64; ARM_JOINTS],
    ) -> Result<[f64; ARM_JOINTS]> {
        const DAMPING: f64 = 0.02;
        const POSTURE_GAIN: f64 = 0.08;
        const ORIENTATION_WEIGHT: f64 = 0.4;
        let mut q = seed;
        for _ in 0..40 {
            let current = self.gripper_pose(&q)?;
            let position_error = target.position - current.position;
            let rotation_error = rotation_vector(target.rotation * current.rotation.conjugate());
            let error = [
                position_error.x,
                position_error.y,
                position_error.z,
                ORIENTATION_WEIGHT * rotation_error.x,
                ORIENTATION_WEIGHT * rotation_error.y,
                ORIENTATION_WEIGHT * rotation_error.z,
            ];
            if position_error.length() < 2e-4 && rotation_error.length() < 2e-3 {
                break;
            }
            let jacobian = self
                .model
                .jacobian(&self.full(&q), self.gripper, Vec3::ZERO)?;
            // J: 6 x 7, angular rows weighted like the error.
            let j: [[f64; ARM_JOINTS]; 6] = std::array::from_fn(|row| {
                let weight = if row < 3 { 1.0 } else { ORIENTATION_WEIGHT };
                std::array::from_fn(|col| weight * jacobian.get(row, col))
            });
            // A = J J^T + damping^2 I, and J+ = J^T A^-1.
            let a: [[f64; 6]; 6] = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    let dot: f64 = (0..ARM_JOINTS).map(|k| j[r][k] * j[c][k]).sum();
                    dot + if r == c { DAMPING * DAMPING } else { 0.0 }
                })
            });
            let y = solve6(a, error).context("singular IK system")?;
            let primary: [f64; ARM_JOINTS] =
                std::array::from_fn(|k| (0..6).map(|r| j[r][k] * y[r]).sum());
            // Null-space pull toward the rest posture: (I - J+ J) (rest - q).
            let pull: [f64; ARM_JOINTS] =
                std::array::from_fn(|k| POSTURE_GAIN * (self.rest[k] - q[k]));
            let j_pull: [f64; 6] =
                std::array::from_fn(|r| (0..ARM_JOINTS).map(|k| j[r][k] * pull[k]).sum());
            let z = solve6(a, j_pull).context("singular IK system")?;
            for k in 0..ARM_JOINTS {
                let projected: f64 = (0..6).map(|r| j[r][k] * z[r]).sum();
                let (lower, upper) = self.limits[k];
                q[k] = (q[k] + primary[k] + pull[k] - projected).clamp(lower, upper);
            }
        }
        Ok(q)
    }
}

/// The rotation vector (axis times angle) of a unit quaternion, taking the
/// short way round.
fn rotation_vector(rotation: Quat) -> Vec3 {
    let rotation = if rotation.w < 0.0 {
        -rotation
    } else {
        rotation
    };
    let (axis, angle) = rotation.to_axis_angle();
    axis * angle
}

/// Solves a 6x6 linear system by Gaussian elimination with partial pivoting.
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for column in 0..6 {
        let pivot =
            (column..6).max_by(|x, y| a[*x][column].abs().total_cmp(&a[*y][column].abs()))?;
        if a[pivot][column].abs() < 1e-12 {
            return None;
        }
        a.swap(column, pivot);
        b.swap(column, pivot);
        for row in column + 1..6 {
            let factor = a[row][column] / a[column][column];
            let pivot_row = a[column];
            for (k, value) in a[row].iter_mut().enumerate().skip(column) {
                *value -= factor * pivot_row[k];
            }
            b[row] -= factor * b[column];
        }
    }
    let mut x = [0.0; 6];
    for row in (0..6).rev() {
        let rest: f64 = (row + 1..6).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - rest) / a[row][row];
    }
    Some(x)
}

/// Every step's nine joint targets for one arm: the gripper glides between
/// the keyposes' gripper poses on `ease`, solved by IK from the step before;
/// the two finger targets blend between the keyposes' finger values.
pub(crate) fn plan_arm(
    solver: &ArmSolver,
    keyframes: &[(u64, [f64; 9])],
    steps: u64,
    ease: impl Fn(f64) -> f64,
) -> Result<Vec<[f64; 9]>> {
    let arm = |pose: &[f64; 9]| -> [f64; ARM_JOINTS] { std::array::from_fn(|k| pose[k]) };
    let targets: Vec<GripperPose> = keyframes
        .iter()
        .map(|(_, pose)| solver.gripper_pose(&arm(pose)))
        .collect::<Result<_>>()?;
    let mut q = arm(&keyframes[0].1);
    let mut plan = Vec::with_capacity(steps as usize + 1);
    for step in 0..=steps {
        let segment = keyframes
            .windows(2)
            .position(|window| step <= window[1].0)
            .unwrap_or(keyframes.len() - 2);
        let (start, end) = (keyframes[segment], keyframes[segment + 1]);
        let alpha = if end.0 > start.0 {
            ease((step.saturating_sub(start.0)) as f64 / (end.0 - start.0) as f64)
        } else {
            1.0
        };
        let (from, to) = (targets[segment], targets[segment + 1]);
        let target = GripperPose {
            position: from.position + (to.position - from.position) * alpha,
            rotation: from.rotation.slerp(to.rotation, alpha).normalize(),
        };
        q = solver.solve(target, q)?;
        let mut pose = [0.0; 9];
        pose[..ARM_JOINTS].copy_from_slice(&q);
        for (finger, value) in pose.iter_mut().enumerate().skip(ARM_JOINTS) {
            *value = start.1[finger] + (end.1[finger] - start.1[finger]) * alpha;
        }
        plan.push(pose);
    }
    Ok(plan)
}
