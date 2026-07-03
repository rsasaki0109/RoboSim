//! Analytic forward / inverse kinematics for the fixed-base `mm_minimal` SCARA chain.
//!
//! Pure, deterministic, and seed-free: geometry matches `assets/robots/mm_minimal/mm_minimal.urdf`
//! and the robot's `[urdf].initial_translation_m`. The solved frame is the **gripper base**
//! link origin (parallel-jaw mount), which is the manipulation frame for pick-and-place.

/// World-frame reach target for the gripper base.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmMinimalGripperTarget {
    /// Target X in meters (world).
    pub x_m: f64,
    /// Target Y in meters (world).
    pub y_m: f64,
    /// Target Z in meters (world).
    pub z_m: f64,
}

impl MmMinimalGripperTarget {
    /// Creates a world-frame gripper-base target.
    pub fn new(x_m: f64, y_m: f64, z_m: f64) -> Self {
        Self { x_m, y_m, z_m }
    }
}

/// Joint-space solution for the shoulder + elbow chain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmMinimalJointTarget {
    /// Shoulder revolute angle in radians (sim motor convention).
    pub shoulder_rad: f64,
    /// Elbow revolute angle in radians (sim motor convention).
    pub elbow_rad: f64,
}

/// Error returned when a gripper target lies outside the analytic workspace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MmMinimalIkError {
    /// Horizontal reach exceeds the sum of the upper arm and forearm links.
    ReachTooFar,
    /// Horizontal reach is below the minimum span of the two links.
    ReachTooNear,
}

/// Fixed geometric parameters for the `mm_minimal` URDF chain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MmMinimalKinematics {
    /// Robot base center height in world Y (from `.rne.robot.toml`).
    pub base_y_m: f64,
    /// Shoulder pivot Y offset on the base link in meters.
    pub shoulder_offset_y_m: f64,
    /// Shoulder pivot X in world meters (base is fixed at the origin in XZ).
    pub shoulder_x_m: f64,
    /// Shoulder pivot Z in world meters.
    pub shoulder_z_m: f64,
    /// Upper-arm link length in meters (shoulder to elbow).
    pub upper_arm_m: f64,
    /// Forearm link length to the gripper base in meters.
    pub forearm_m: f64,
}

impl Default for MmMinimalKinematics {
    fn default() -> Self {
        Self::mm_minimal()
    }
}

impl MmMinimalKinematics {
    /// Geometry for the shipped `mm_minimal` asset.
    pub fn mm_minimal() -> Self {
        Self {
            base_y_m: 0.3,
            shoulder_offset_y_m: 0.3,
            shoulder_x_m: 0.0,
            shoulder_z_m: 0.0,
            upper_arm_m: 0.5,
            forearm_m: 0.4,
        }
    }

    /// Shoulder pivot height in world Y.
    pub fn shoulder_y_m(&self) -> f64 {
        self.base_y_m + self.shoulder_offset_y_m
    }

    /// Shoulder pivot in world XZ.
    pub fn shoulder_xz_m(&self) -> (f64, f64) {
        (self.shoulder_x_m, self.shoulder_z_m)
    }

    /// Computes the world-frame gripper-base pose from joint targets.
    ///
    /// `joints` uses the same shoulder sign convention as the simulation motors.
    pub fn forward_kinematics(&self, joints: MmMinimalJointTarget) -> MmMinimalGripperTarget {
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m();
        let (dx, dz) = planar_chain_tip(
            self.upper_arm_m,
            self.forearm_m,
            -joints.shoulder_rad,
            -joints.elbow_rad,
        );
        MmMinimalGripperTarget {
            x_m: shoulder_x + dx,
            y_m: self.shoulder_y_m(),
            z_m: shoulder_z + dz,
        }
    }

    /// Solves analytic IK for a world-frame gripper-base target.
    ///
    /// Picks the "elbow-up" branch when two solutions exist. Deterministic and seed-free.
    pub fn inverse_kinematics(
        &self,
        target: MmMinimalGripperTarget,
    ) -> Result<MmMinimalJointTarget, MmMinimalIkError> {
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m();
        let rx = target.x_m - shoulder_x;
        let rz = target.z_m - shoulder_z;
        let reach = (rx * rx + rz * rz).sqrt();
        let l1 = self.upper_arm_m;
        let l2 = self.forearm_m;

        if reach > l1 + l2 + 1e-9 {
            return Err(MmMinimalIkError::ReachTooFar);
        }
        if reach + 1e-9 < (l1 - l2).abs() {
            return Err(MmMinimalIkError::ReachTooNear);
        }

        let cos_elbow = ((reach * reach - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
        let elbow_rad = cos_elbow.acos();
        let shoulder_rad = planar_chain_shoulder(rx, rz, l1, l2, elbow_rad)
            .ok_or(MmMinimalIkError::ReachTooFar)?;

        Ok(MmMinimalJointTarget {
            shoulder_rad: -shoulder_rad,
            elbow_rad: -elbow_rad,
        })
    }

    /// Maximum horizontal reach from the shoulder pivot in meters.
    pub fn max_reach_m(&self) -> f64 {
        self.upper_arm_m + self.forearm_m
    }

    /// Returns true when the horizontal target lies inside the analytic workspace.
    pub fn is_reachable(&self, x_m: f64, z_m: f64) -> bool {
        self.inverse_kinematics(MmMinimalGripperTarget::new(x_m, self.shoulder_y_m(), z_m))
            .is_ok()
    }

    /// Nearest workspace point at maximal horizontal reach toward `(x_m, z_m)`.
    ///
    /// Used when a place target lies outside the fixed-base workspace.
    pub fn max_reach_toward(&self, x_m: f64, z_m: f64) -> MmMinimalGripperTarget {
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m();
        let dx = x_m - shoulder_x;
        let dz = z_m - shoulder_z;
        let dist = (dx * dx + dz * dz).sqrt();
        if dist <= 1e-9 {
            return MmMinimalGripperTarget::new(
                shoulder_x + self.max_reach_m(),
                self.shoulder_y_m(),
                shoulder_z,
            );
        }
        let scale = (self.max_reach_m() * 0.98) / dist;
        MmMinimalGripperTarget::new(
            shoulder_x + dx * scale,
            self.shoulder_y_m(),
            shoulder_z + dz * scale,
        )
    }
}

/// Ground place target for fixed-base clutter episodes (inside analytic reach).
pub const MM_MINIMAL_CLUTTER_PLACE_X_M: f64 = 0.82;
/// Ground place height for fixed-base clutter episodes.
pub const MM_MINIMAL_CLUTTER_PLACE_Y_M: f64 = 0.03;
/// Ground place lateral target off the table edge (table spans z ∈ [-0.35, 0.35]).
pub const MM_MINIMAL_CLUTTER_PLACE_Z_M: f64 = -0.42;

/// Returns the fixed-base clutter [`ReachTarget`](crate::ReachTarget) inside workspace reach.
pub fn mm_minimal_clutter_place_target() -> crate::reach::ReachTarget {
    crate::reach::ReachTarget::new(
        MM_MINIMAL_CLUTTER_PLACE_X_M,
        MM_MINIMAL_CLUTTER_PLACE_Y_M,
        MM_MINIMAL_CLUTTER_PLACE_Z_M,
    )
}

/// Ground place target for mobile clutter navigate-and-place episodes.
pub const MM_MOBILE_CLUTTER_PLACE_X_M: f64 = 1.23;
/// Ground place height for mobile clutter episodes.
pub const MM_MOBILE_CLUTTER_PLACE_Y_M: f64 = 0.03;
/// Ground place lateral target beside the mobile clutter table.
pub const MM_MOBILE_CLUTTER_PLACE_Z_M: f64 = -0.53;

/// Returns the mobile clutter [`ReachTarget`](crate::ReachTarget) used by
/// `mobile_clutter_pick_place` episodes.
pub fn mm_mobile_clutter_place_target() -> crate::reach::ReachTarget {
    crate::reach::ReachTarget::new(
        MM_MOBILE_CLUTTER_PLACE_X_M,
        MM_MOBILE_CLUTTER_PLACE_Y_M,
        MM_MOBILE_CLUTTER_PLACE_Z_M,
    )
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
        let kin = MmMinimalKinematics::mm_minimal();
        let seeds = [
            MmMinimalJointTarget {
                shoulder_rad: 0.0,
                elbow_rad: 0.0,
            },
            MmMinimalJointTarget {
                shoulder_rad: 0.6,
                elbow_rad: 0.8,
            },
            MmMinimalJointTarget {
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
        }
    }

    #[test]
    fn fk_matches_sim_at_idle() {
        use crate::{MobileManipulatorAction, MobileManipulatorSim};

        let kin = MmMinimalKinematics::mm_minimal();
        let mut sim = MobileManipulatorSim::new_mm_minimal();
        for _ in 0..150 {
            sim.step(MobileManipulatorAction::default());
        }
        let obs = sim.observe();
        let fk = kin.forward_kinematics(MmMinimalJointTarget {
            shoulder_rad: obs.shoulder_position_rad,
            elbow_rad: obs.elbow_position_rad,
        });
        let gripper = sim
            .link_translation_m("gripper_base_link")
            .expect("gripper link");
        assert_relative_eq!(fk.x_m, gripper.0, epsilon = 0.03);
        assert_relative_eq!(fk.z_m, gripper.2, epsilon = 0.10);
    }

    #[test]
    fn ik_solves_clutter_tabletop_targets() {
        let kin = MmMinimalKinematics::mm_minimal();
        for (name, x_m, z_m) in [
            ("table_near", 0.75, 0.0),
            ("table_mid", 0.82, 0.10),
            ("table_far", 0.88, -0.10),
        ] {
            let target = MmMinimalGripperTarget::new(x_m, kin.shoulder_y_m(), z_m);
            kin.inverse_kinematics(target)
                .unwrap_or_else(|_| panic!("{name} should be analytically reachable"));
        }
    }

    #[test]
    fn clutter_scene_center_cube_is_beyond_analytic_reach() {
        let kin = MmMinimalKinematics::mm_minimal();
        assert!(
            !kin.is_reachable(1.05, 0.0),
            "center clutter cube relies on proportional approach outside analytic reach"
        );
    }

    #[test]
    fn fixed_base_clutter_place_target_is_reachable() {
        let kin = MmMinimalKinematics::mm_minimal();
        let target = mm_minimal_clutter_place_target();
        let max_reach = kin.max_reach_toward(target.x_m, target.z_m);
        kin.inverse_kinematics(max_reach)
            .expect("max-reach toward fixed-base clutter place should be solvable");
    }

    #[test]
    fn max_reach_toward_mobile_place_target_is_solvable() {
        let kin = MmMinimalKinematics::mm_minimal();
        let target = kin.max_reach_toward(1.23, -0.53);
        kin.inverse_kinematics(target)
            .expect("max-reach pose toward clutter place should be solvable");
        let (shoulder_x, shoulder_z) = kin.shoulder_xz_m();
        let reach = ((target.x_m - shoulder_x).powi(2) + (target.z_m - shoulder_z).powi(2)).sqrt();
        assert!(reach <= kin.max_reach_m() + 1e-6);
    }
}
