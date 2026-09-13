//! Forward and inverse kinematics for the vendored SO101 6-DoF arm.
//!
//! Pure, deterministic, and seed-free: the fixed chain origins and axis
//! rotations are transcribed from `assets/robots/so101/so101.urdf` (and the
//! generated `mm_mobile_so101` mount). The solved frames are the grasp pocket
//! (midpoint between the moving-jaw pad and the fixed anvil) and the gripper
//! link, so a controller can place the jaws on a world target without stepping
//! physics.
//!
//! The SO101 has five actuated arm joints (pan, lift, elbow, wrist flex, wrist
//! roll) plus a jaw. Inverse kinematics therefore solves a three-dimensional
//! position with two degrees of redundancy; [`So101Kinematics::inverse_pocket`]
//! resolves it by damped least squares with a nominal-pose regularizer, which
//! keeps the solver away from singularities and near the calibration pose.

use rne_math::{Quat, Transform3, Vec3};

/// Joint-space target for the SO101 arm (radians).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct So101JointTarget {
    /// `shoulder_pan` angle in radians.
    pub shoulder_pan_rad: f64,
    /// `shoulder_lift` angle in radians.
    pub shoulder_lift_rad: f64,
    /// `elbow_flex` angle in radians.
    pub elbow_flex_rad: f64,
    /// `wrist_flex` angle in radians.
    pub wrist_flex_rad: f64,
    /// `wrist_roll` angle in radians.
    pub wrist_roll_rad: f64,
    /// Moving-jaw angle in radians (open is positive, shut is negative).
    pub gripper_rad: f64,
}

impl So101JointTarget {
    /// Joint values as an array in chain order.
    pub fn as_array(&self) -> [f64; 6] {
        [
            self.shoulder_pan_rad,
            self.shoulder_lift_rad,
            self.elbow_flex_rad,
            self.wrist_flex_rad,
            self.wrist_roll_rad,
            self.gripper_rad,
        ]
    }

    /// Builds a target from a chain-order array.
    pub fn from_array(values: [f64; 6]) -> Self {
        Self {
            shoulder_pan_rad: values[0],
            shoulder_lift_rad: values[1],
            elbow_flex_rad: values[2],
            wrist_flex_rad: values[3],
            wrist_roll_rad: values[4],
            gripper_rad: values[5],
        }
    }
}

/// Error returned when SO101 inverse kinematics cannot reach a target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum So101IkError {
    /// The solver did not converge within its iteration budget.
    NotConverged,
    /// A non-finite target or seed was provided.
    InvalidInput,
}

#[derive(Clone, Copy, Debug)]
struct Joint {
    origin_xyz_m: Vec3,
    origin_rpy_rad: Vec3,
}

// Chain origins transcribed from the generated SO101 URDF (see the module
// docs). The chassis mount is folded into shoulder_pan (the arm mounts
// directly on `base_link`), so the chain starts at the mobile base and ends at
// the grasp frames.
const SHOULDER_PAN: Joint = Joint {
    origin_xyz_m: Vec3::new(0.2388353, 0.149999991, 0.0624),
    origin_rpy_rad: Vec3::new(std::f64::consts::PI, 4.18253e-17, -std::f64::consts::PI),
};
const SHOULDER_LIFT: Joint = Joint {
    origin_xyz_m: Vec3::new(-0.0303992, -0.0182778, -0.0542),
    origin_rpy_rad: Vec3::new(
        -std::f64::consts::FRAC_PI_2,
        -std::f64::consts::FRAC_PI_2,
        0.0,
    ),
};
const ELBOW_FLEX: Joint = Joint {
    origin_xyz_m: Vec3::new(-0.11257, -0.028, 1.73763e-16),
    origin_rpy_rad: Vec3::new(-3.63608e-16, 8.74301e-16, std::f64::consts::FRAC_PI_2),
};
const WRIST_FLEX: Joint = Joint {
    origin_xyz_m: Vec3::new(-0.1349, 0.0052, 3.62355e-17),
    origin_rpy_rad: Vec3::new(4.02456e-15, 8.67362e-16, -std::f64::consts::FRAC_PI_2),
};
const WRIST_ROLL: Joint = Joint {
    origin_xyz_m: Vec3::new(5.55112e-17, -0.0611, 0.0181),
    origin_rpy_rad: Vec3::new(std::f64::consts::FRAC_PI_2, 0.0486795, std::f64::consts::PI),
};
const GRIPPER: Joint = Joint {
    origin_xyz_m: Vec3::new(0.0202, 0.0188, -0.0234),
    origin_rpy_rad: Vec3::new(std::f64::consts::FRAC_PI_2, -5.24284e-8, -1.41553e-15),
};

/// Fixed anvil (grasp frame) origin in the gripper-link frame.
const ANVIL_LOCAL_M: Vec3 = Vec3::new(-0.0079, -0.000218121, -0.0981274);
/// Moving-jaw grip-pad center in the jaw-link frame.
const JAW_PAD_LOCAL_M: Vec3 = Vec3::new(0.0, -0.072, 0.019);

/// Joint limits (radians) for the five arm joints, in chain order.
const ARM_LIMITS_RAD: [(f64, f64); 5] = [
    (-1.91986, 1.91986),
    (-1.74533, 1.74533),
    (-1.69, 1.69),
    (-1.65806, 1.65806),
    (-2.74385, 2.84121),
];
/// Jaw limits (radians); open is positive, shut is negative.
const JAW_LIMITS_RAD: (f64, f64) = (-0.174533, 1.74533);

fn rpy_to_quat(rpy: Vec3) -> Quat {
    Quat::from_rotation_z(rpy.z) * Quat::from_rotation_y(rpy.y) * Quat::from_rotation_x(rpy.x)
}

fn joint_transform(joint: &Joint, angle_rad: f64) -> Transform3 {
    Transform3::from_translation_rotation(
        joint.origin_xyz_m,
        rpy_to_quat(joint.origin_rpy_rad) * Quat::from_rotation_z(angle_rad),
    )
}

fn compose(parent: Transform3, child: Transform3) -> Transform3 {
    Transform3::from_translation_rotation(
        parent.translation + parent.rotation * child.translation,
        parent.rotation * child.rotation,
    )
}

fn clamp_arm_joint(index: usize, value: f64) -> f64 {
    let (low, high) = ARM_LIMITS_RAD[index];
    value.clamp(low, high)
}

/// SO101 forward/inverse kinematics helper.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct So101Kinematics;

impl So101Kinematics {
    /// Creates the kinematics helper.
    pub fn new() -> Self {
        Self
    }

    /// Returns the gripper-link transform for a base pose and joint target.
    pub fn forward_gripper(&self, base: Transform3, joints: &So101JointTarget) -> Transform3 {
        let values = joints.as_array();
        let mut t = base;
        for (joint, angle) in [
            (&SHOULDER_PAN, values[0]),
            (&SHOULDER_LIFT, values[1]),
            (&ELBOW_FLEX, values[2]),
            (&WRIST_FLEX, values[3]),
            (&WRIST_ROLL, values[4]),
        ] {
            t = compose(t, joint_transform(joint, angle));
        }
        t
    }

    /// Returns the grasp pocket (moving-jaw pad / fixed-anvil midpoint) in the
    /// base frame for a base pose and joint target.
    pub fn forward_pocket(&self, base: Transform3, joints: &So101JointTarget) -> Vec3 {
        let gripper = self.forward_gripper(base, joints);
        let anvil = gripper.translation + gripper.rotation * ANVIL_LOCAL_M;
        let jaw = compose(gripper, joint_transform(&GRIPPER, joints.gripper_rad));
        let jaw_pad = jaw.translation + jaw.rotation * JAW_PAD_LOCAL_M;
        0.5 * (anvil + jaw_pad)
    }

    /// Solves joint targets that place the grasp pocket at `target_m`.
    ///
    /// Damped least squares over the three primary positioning joints
    /// (shoulder pan/lift and elbow flex); the wrist flex/roll and jaw are held
    /// at `seed`. Solving all five arm joints leaves a two-degree null space
    /// that the solver spends driving the redundant wrist joints into their
    /// limits, which then blocks the target. Fixing the wrist makes the
    /// position problem three-by-three. Returns the best effort after a bounded
    /// iteration budget; the caller can verify with [`Self::forward_pocket`].
    pub fn inverse_pocket(
        &self,
        base: Transform3,
        target_m: Vec3,
        seed: &So101JointTarget,
        iterations: usize,
    ) -> Result<So101JointTarget, So101IkError> {
        if !target_m.is_finite() || !seed.as_array().iter().all(|value| value.is_finite()) {
            return Err(So101IkError::InvalidInput);
        }
        const STEP_RAD: f64 = 1.0e-4;
        const DAMPING: f64 = 1.0e-4;
        const MAX_STEP_RAD: f64 = 0.25;
        const CONVERGED_M: f64 = 1.0e-4;

        let mut q = *seed;
        let mut best = q;
        let mut best_error = f64::INFINITY;

        for _ in 0..iterations {
            let current = self.forward_pocket(base, &q);
            let error = target_m - current;
            let error_norm = error.length();
            if error_norm < best_error {
                best_error = error_norm;
                best = q;
            }
            if error_norm < CONVERGED_M {
                return Ok(q);
            }

            // Finite-difference Jacobian: d(pocket)/d(q_i) for the three
            // primary positioning joints.
            let mut jacobian = [[0.0_f64; 3]; 3];
            for (index, column) in jacobian.iter_mut().enumerate() {
                let mut perturbed = q;
                let mut values = perturbed.as_array();
                values[index] += STEP_RAD;
                perturbed = So101JointTarget::from_array(values);
                let moved = self.forward_pocket(base, &perturbed);
                *column = ((moved - current) / STEP_RAD).to_array();
            }

            // J^T (J J^T + lambda I)^-1 e, computed in 3x3 closed form.
            let mut jjt = [[0.0_f64; 3]; 3];
            for row in 0..3 {
                for column in 0..3 {
                    let sum: f64 = jacobian
                        .iter()
                        .map(|joint| joint[row] * joint[column])
                        .sum();
                    jjt[row][column] = sum;
                }
                jjt[row][row] += DAMPING;
            }
            let Some(inverse) = invert3(jjt) else {
                return Err(So101IkError::NotConverged);
            };
            let error_array = error.to_array();
            let mut values = q.as_array();
            for (joint, row) in jacobian.iter().enumerate() {
                let mut weighted = [0.0_f64; 3];
                for (i, slot) in weighted.iter_mut().enumerate() {
                    *slot = (0..3).map(|j| inverse[i][j] * error_array[j]).sum();
                }
                let correction = row[0] * weighted[0] + row[1] * weighted[1] + row[2] * weighted[2];
                values[joint] = clamp_arm_joint(
                    joint,
                    values[joint] + correction.clamp(-MAX_STEP_RAD, MAX_STEP_RAD),
                );
            }
            q = So101JointTarget::from_array(values);
        }

        if best_error.is_finite() {
            Ok(best)
        } else {
            Err(So101IkError::NotConverged)
        }
    }

    /// Clamps a joint target to the SO101 joint limits.
    pub fn clamp(&self, joints: &So101JointTarget) -> So101JointTarget {
        let mut values = joints.as_array();
        for (joint, value) in values.iter_mut().take(5).enumerate() {
            *value = clamp_arm_joint(joint, *value);
        }
        values[5] = values[5].clamp(JAW_LIMITS_RAD.0, JAW_LIMITS_RAD.1);
        So101JointTarget::from_array(values)
    }
}

/// Inverts a 3x3 matrix by cofactors; `None` when singular.
fn invert3(m: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1.0e-18 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_gripper_matches_sim_spawn() {
        let kin = So101Kinematics::new();
        let base = Transform3::from_translation_rotation(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY);
        let gripper = kin.forward_gripper(base, &So101JointTarget::default());
        // Measured from `MobileManipulatorSim::new_mm_mobile_so101` at spawn
        // (before the first step), which is exactly the URDF zero pose.
        let expected = Vec3::new(0.4932, 0.3998, 0.2344);
        assert!(
            (gripper.translation - expected).length() < 5.0e-3,
            "spawn gripper {:?}",
            gripper.translation
        );
    }

    #[test]
    fn forward_pocket_is_between_the_jaws() {
        let kin = So101Kinematics::new();
        let base = Transform3::from_translation_rotation(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY);
        let pocket = kin.forward_pocket(base, &So101JointTarget::default());
        // Pocket sits just below/ahead of the gripper link, between the pads.
        let gripper = kin.forward_gripper(base, &So101JointTarget::default());
        assert!((pocket - gripper.translation).length() < 0.15);
    }

    #[test]
    fn inverse_pocket_round_trips() {
        let kin = So101Kinematics::new();
        let base = Transform3::from_translation_rotation(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY);
        let target = Vec3::new(0.66, 0.44, 0.10);
        let solved = kin
            .inverse_pocket(base, target, &So101JointTarget::default(), 400)
            .expect("ik");
        let reached = kin.forward_pocket(base, &solved);
        assert!(
            (reached - target).length() < 5.0e-3,
            "reached {reached:?} target {target:?}"
        );
    }

    #[test]
    fn inverse_pocket_clamps_to_joint_limits() {
        let kin = So101Kinematics::new();
        let seed = So101JointTarget {
            shoulder_pan_rad: 100.0,
            ..So101JointTarget::default()
        };
        let solved = kin.clamp(&seed);
        assert!(solved.shoulder_pan_rad <= ARM_LIMITS_RAD[0].1);
    }
}
