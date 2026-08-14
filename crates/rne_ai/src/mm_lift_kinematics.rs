//! Analytic forward / inverse kinematics for the `mm_lift` column + 2R arm chain.
//!
//! Pure, deterministic, and seed-free: geometry matches `assets/robots/mm_lift/mm_lift.urdf`
//! and the robot's `[urdf].initial_translation_m`. The solved frame is the **gripper base**
//! (top-down claw mount), which is the manipulation frame for pick-and-place.

/// World-frame reach target for the gripper base.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmLiftGripperTarget {
    /// Target X in meters (world).
    pub x_m: f64,
    /// Target Y in meters (world).
    pub y_m: f64,
    /// Target Z in meters (world).
    pub z_m: f64,
}

impl MmLiftGripperTarget {
    /// Creates a world-frame gripper-base target.
    pub fn new(x_m: f64, y_m: f64, z_m: f64) -> Self {
        Self { x_m, y_m, z_m }
    }
}

/// Joint-space solution for the lift + shoulder + elbow chain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmLiftJointTarget {
    /// Prismatic lift displacement in meters (same convention as the sim motor target).
    pub lift_m: f64,
    /// Shoulder revolute angle in radians.
    pub shoulder_rad: f64,
    /// Elbow revolute angle in radians.
    pub elbow_rad: f64,
}

/// Error returned when a gripper target lies outside the analytic workspace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MmLiftIkError {
    /// Horizontal reach exceeds the sum of the upper arm and forearm links.
    ReachTooFar,
    /// Horizontal reach is below the minimum span of the two links.
    ReachTooNear,
    /// Lift displacement would fall outside the sim travel limits.
    LiftOutOfRange,
}

/// Fixed geometric parameters for the `mm_lift` URDF chain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmLiftKinematics {
    /// Robot base center height in world Y (from `.rne.robot.toml`).
    pub base_y_m: f64,
    /// Lift anchor X offset on the base link in meters.
    pub anchor_x_m: f64,
    /// Lift anchor Y offset on the base link in meters.
    pub anchor_y_m: f64,
    /// Shoulder pivot offset from the carriage along +X in meters.
    pub shoulder_offset_x_m: f64,
    /// Upper-arm link length in meters.
    pub upper_arm_m: f64,
    /// Forearm link length to the gripper base in meters.
    pub forearm_m: f64,
    /// Minimum commanded lift displacement in meters (matches the sim).
    pub lift_min_m: f64,
    /// Maximum commanded lift displacement in meters (matches the sim).
    pub lift_max_m: f64,
}

impl Default for MmLiftKinematics {
    fn default() -> Self {
        Self::mm_lift()
    }
}

impl MmLiftKinematics {
    /// Geometry for the shipped `mm_lift` asset.
    pub fn mm_lift() -> Self {
        Self {
            base_y_m: 0.75,
            anchor_x_m: 0.15,
            anchor_y_m: 0.15,
            shoulder_offset_x_m: 0.16,
            upper_arm_m: 0.5,
            forearm_m: 0.4,
            lift_min_m: -0.5,
            lift_max_m: 0.5,
        }
    }

    /// Geometry for the shipped `mm_mobile_lift` asset in its zero base pose.
    ///
    /// The arm chain matches [`Self::mm_lift`], while the mobile chassis places
    /// the lift origin 0.25 m above the ground.
    pub fn mm_mobile_lift() -> Self {
        Self {
            base_y_m: 0.25,
            anchor_y_m: 0.0,
            ..Self::mm_lift()
        }
    }

    /// Shoulder pivot in world XZ when the lift carriage is at `lift_m`.
    pub fn shoulder_xz_m(&self, _lift_m: f64) -> (f64, f64) {
        (self.anchor_x_m + self.shoulder_offset_x_m, 0.0)
    }

    /// Shoulder pivot height in world Y for a given lift displacement.
    pub fn shoulder_y_m(&self, lift_m: f64) -> f64 {
        // The prismatic joint reads as displacement from the robot base height; the URDF
        // anchor offset is already baked into the carriage rest pose in the sim.
        self.base_y_m + lift_m
    }

    /// Computes the world-frame gripper pose for a lift robot on a planar mobile base.
    ///
    /// The base uses the engine's Y-up pose convention. `base_yaw_rad` follows the
    /// same sign convention exposed by `MobileManipulatorObservation`.
    pub fn forward_kinematics_at_base(
        &self,
        base_x_m: f64,
        base_y_m: f64,
        base_z_m: f64,
        base_yaw_rad: f64,
        joints: MmLiftJointTarget,
    ) -> MmLiftGripperTarget {
        let local = Self { base_y_m, ..*self }.forward_kinematics(joints);
        let (world_x, world_z) = rotate_y_xz(local.x_m, local.z_m, -base_yaw_rad);
        MmLiftGripperTarget::new(base_x_m + world_x, local.y_m, base_z_m + world_z)
    }

    /// Solves lift and arm joints for a world-frame target at a planar mobile-base pose.
    pub fn inverse_kinematics_at_base(
        &self,
        base_x_m: f64,
        base_y_m: f64,
        base_z_m: f64,
        base_yaw_rad: f64,
        target: MmLiftGripperTarget,
    ) -> Result<MmLiftJointTarget, MmLiftIkError> {
        let (local_x, local_z) =
            rotate_y_xz(target.x_m - base_x_m, target.z_m - base_z_m, base_yaw_rad);
        Self { base_y_m, ..*self }
            .inverse_kinematics(MmLiftGripperTarget::new(local_x, target.y_m, local_z))
    }

    /// Computes the world-frame gripper-base pose from joint targets.
    ///
    /// `joints` uses the same shoulder sign convention as the simulation motors.
    pub fn forward_kinematics(&self, joints: MmLiftJointTarget) -> MmLiftGripperTarget {
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m(joints.lift_m);
        let elbow_sign = if self.is_mobile_lift_geometry() {
            -1.0
        } else {
            1.0
        };
        let (dx, dz) = planar_chain_tip(
            self.upper_arm_m,
            self.forearm_m,
            -joints.shoulder_rad,
            elbow_sign * joints.elbow_rad,
        );
        MmLiftGripperTarget {
            x_m: shoulder_x + dx,
            y_m: self.shoulder_y_m(joints.lift_m),
            z_m: shoulder_z + dz,
        }
    }

    /// Solves analytic IK for a world-frame gripper-base target.
    ///
    /// Picks the "elbow-up" branch when two solutions exist. Deterministic and seed-free.
    pub fn inverse_kinematics(
        &self,
        target: MmLiftGripperTarget,
    ) -> Result<MmLiftJointTarget, MmLiftIkError> {
        let lift_m = target.y_m - self.base_y_m;
        self.inverse_kinematics_at_lift(lift_m, target.x_m, target.z_m)
    }

    /// Solves shoulder / elbow IK at a fixed lift displacement toward a horizontal target.
    pub fn inverse_kinematics_at_lift(
        &self,
        lift_m: f64,
        gripper_x_m: f64,
        gripper_z_m: f64,
    ) -> Result<MmLiftJointTarget, MmLiftIkError> {
        if !(self.lift_min_m..=self.lift_max_m).contains(&lift_m) {
            return Err(MmLiftIkError::LiftOutOfRange);
        }

        let (shoulder_rad, elbow_rad) =
            self.planar_inverse_kinematics(lift_m, gripper_x_m, gripper_z_m)?;

        Ok(MmLiftJointTarget {
            lift_m,
            shoulder_rad: -shoulder_rad,
            elbow_rad: if self.is_mobile_lift_geometry() {
                -elbow_rad
            } else {
                elbow_rad
            },
        })
    }

    fn is_mobile_lift_geometry(&self) -> bool {
        self.base_y_m < 0.5 && self.anchor_y_m.abs() <= f64::EPSILON
    }

    fn planar_inverse_kinematics(
        &self,
        lift_m: f64,
        gripper_x_m: f64,
        gripper_z_m: f64,
    ) -> Result<(f64, f64), MmLiftIkError> {
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m(lift_m);
        let rx = gripper_x_m - shoulder_x;
        let rz = gripper_z_m - shoulder_z;
        let reach = (rx * rx + rz * rz).sqrt();
        let l1 = self.upper_arm_m;
        let l2 = self.forearm_m;

        if reach > l1 + l2 + 1e-9 {
            return Err(MmLiftIkError::ReachTooFar);
        }
        if reach + 1e-9 < (l1 - l2).abs() {
            return Err(MmLiftIkError::ReachTooNear);
        }

        let cos_elbow = ((reach * reach - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
        let elbow_rad = if self.is_mobile_lift_geometry() {
            -cos_elbow.acos()
        } else {
            cos_elbow.acos()
        };
        let shoulder_rad =
            planar_chain_shoulder(rx, rz, l1, l2, elbow_rad).ok_or(MmLiftIkError::ReachTooFar)?;
        Ok((shoulder_rad, elbow_rad))
    }
}

fn rotate_y_xz(x: f64, z: f64, angle_rad: f64) -> (f64, f64) {
    let (sin, cos) = angle_rad.sin_cos();
    (x * cos - z * sin, x * sin + z * cos)
}

fn planar_chain_tip(
    upper_arm_m: f64,
    forearm_m: f64,
    shoulder_rad: f64,
    elbow_rad: f64,
) -> (f64, f64) {
    let (x1, z1) = rotate_y_xz(upper_arm_m, 0.0, shoulder_rad);
    let (x2, z2) = rotate_y_xz(forearm_m, 0.0, shoulder_rad + elbow_rad);
    (x1 + x2, z1 + z2)
}

fn planar_chain_shoulder(
    rx: f64,
    rz: f64,
    upper_arm_m: f64,
    forearm_m: f64,
    elbow_rad: f64,
) -> Option<f64> {
    let (sin_e, cos_e) = elbow_rad.sin_cos();
    let k1 = upper_arm_m + forearm_m * cos_e;
    let k2 = forearm_m * sin_e;
    let denom = k1 * k1 + k2 * k2;
    if denom <= f64::EPSILON {
        return None;
    }
    Some(rz.atan2(rx) - k2.atan2(k1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn fk_ik_roundtrip_for_reachable_targets() {
        let kin = MmLiftKinematics::mm_lift();
        let seeds = [
            MmLiftJointTarget {
                lift_m: 0.0,
                shoulder_rad: 0.0,
                elbow_rad: 0.0,
            },
            MmLiftJointTarget {
                lift_m: -0.2,
                shoulder_rad: 0.6,
                elbow_rad: 0.8,
            },
            MmLiftJointTarget {
                lift_m: 0.15,
                shoulder_rad: -0.4,
                elbow_rad: 1.1,
            },
        ];

        for joints in seeds {
            let target = kin.forward_kinematics(joints);
            let solved = kin
                .inverse_kinematics(target)
                .expect("seed should be reachable");
            let reshot = kin.forward_kinematics(solved);
            assert_relative_eq!(target.x_m, reshot.x_m, epsilon = 1e-9);
            assert_relative_eq!(target.y_m, reshot.y_m, epsilon = 1e-9);
            assert_relative_eq!(target.z_m, reshot.z_m, epsilon = 1e-9);
            assert_relative_eq!(joints.shoulder_rad, solved.shoulder_rad, epsilon = 1e-6);
            assert_relative_eq!(joints.elbow_rad, solved.elbow_rad, epsilon = 1e-6);
            assert_relative_eq!(joints.lift_m, solved.lift_m, epsilon = 1e-9);
        }
    }

    #[test]
    fn mobile_base_fk_ik_roundtrip_with_yaw() {
        let kin = MmLiftKinematics::mm_mobile_lift();
        let joints = MmLiftJointTarget {
            lift_m: 0.2,
            shoulder_rad: 0.5,
            elbow_rad: 0.7,
        };
        let base = (1.2_f64, 0.25_f64, -0.8_f64);
        for yaw_rad in [0.0, 0.6, -1.1, 2.4] {
            let target = kin.forward_kinematics_at_base(base.0, base.1, base.2, yaw_rad, joints);
            let solved = kin
                .inverse_kinematics_at_base(base.0, base.1, base.2, yaw_rad, target)
                .expect("reachable rotated target");
            assert_relative_eq!(solved.lift_m, joints.lift_m, epsilon = 1e-9);
            assert_relative_eq!(solved.shoulder_rad, joints.shoulder_rad, epsilon = 1e-9);
            assert_relative_eq!(solved.elbow_rad, joints.elbow_rad, epsilon = 1e-9);
        }
    }

    #[test]
    fn fk_matches_sim_at_idle() {
        use crate::{MobileManipulatorAction, MobileManipulatorSim};

        let kin = MmLiftKinematics::mm_lift();
        let mut sim = MobileManipulatorSim::new_mm_lift();
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        let obs = sim.observe();
        let lift_m = sim.lift_position_m();
        let fk = kin.forward_kinematics(MmLiftJointTarget {
            lift_m,
            shoulder_rad: obs.shoulder_position_rad,
            elbow_rad: obs.elbow_position_rad,
        });
        let gripper = sim
            .link_translation_m("gripper_base_link")
            .expect("gripper link");
        assert_relative_eq!(fk.x_m, gripper.0, epsilon = 0.02);
        assert_relative_eq!(fk.y_m, gripper.1, epsilon = 0.02);
        assert_relative_eq!(fk.z_m, gripper.2, epsilon = 0.02);
    }
}
