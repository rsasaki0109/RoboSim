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

    /// Geometry for the diff-drive `mm_mobile` asset (shoulder pivot offset differs).
    pub fn mm_mobile() -> Self {
        Self {
            base_y_m: 0.25,
            shoulder_offset_y_m: 0.15,
            shoulder_x_m: 0.0,
            shoulder_z_m: 0.0,
            upper_arm_m: 0.5,
            forearm_m: 0.4,
        }
    }

    /// Extra shoulder-pivot Z offset in world meters for the mobile base pose.
    pub fn mobile_shoulder_z_offset_m(&self) -> f64 {
        0.0
    }

    /// Shoulder pivot height in world Y for a base at `base_y_m`.
    pub fn shoulder_y_at_base(&self, base_y_m: f64) -> f64 {
        base_y_m + self.shoulder_offset_y_m
    }

    /// Solves IK when the shoulder pivot sits at the given mobile-base pose.
    ///
    /// `base_yaw_rad` rotates the world-frame target into the base frame so the
    /// planar chain solves in the frame the joint motors are defined in.
    pub fn inverse_kinematics_at_base(
        &self,
        base_x_m: f64,
        base_y_m: f64,
        base_z_m: f64,
        base_yaw_rad: f64,
        target: MmMinimalGripperTarget,
    ) -> Result<MmMinimalJointTarget, MmMinimalIkError> {
        let shoulder_x_m = base_x_m;
        let shoulder_z_m = base_z_m + self.mobile_shoulder_z_offset_m();
        let (local_x, local_z) = rotate_y_xz(
            target.x_m - shoulder_x_m,
            target.z_m - shoulder_z_m,
            base_yaw_rad,
        );
        let local = Self {
            base_y_m,
            shoulder_x_m: 0.0,
            shoulder_z_m: 0.0,
            ..*self
        };
        local.inverse_kinematics(MmMinimalGripperTarget::new(local_x, target.y_m, local_z))
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

    /// Solves analytic IK picking the "elbow-down" branch (the mirror of
    /// [`Self::inverse_kinematics`] across the shoulder-to-target ray).
    ///
    /// For targets on the arm's -z side this branch folds the gripper tip toward
    /// the target in a tight arc instead of swinging the elbow (and the forearm's
    /// long side face) across the +z half of the workspace, which matters when the
    /// approach must brush tabletop objects with the fingertips first (the grasp
    /// weld triggers on finger contact only). Deterministic and seed-free.
    pub fn inverse_kinematics_elbow_down(
        &self,
        target: MmMinimalGripperTarget,
    ) -> Result<MmMinimalJointTarget, MmMinimalIkError> {
        let up = self.inverse_kinematics(target)?;
        let (shoulder_x, shoulder_z) = self.shoulder_xz_m();
        let bearing_rad = (target.z_m - shoulder_z).atan2(target.x_m - shoulder_x);
        // Elbow-up solves `shoulder_rad = -(bearing - beta)` with `beta` the interior
        // shoulder correction; the mirrored branch is `-(bearing + beta)` with the
        // elbow sign flipped.
        let beta_rad = up.shoulder_rad + bearing_rad;
        Ok(MmMinimalJointTarget {
            shoulder_rad: up.shoulder_rad - 2.0 * beta_rad,
            elbow_rad: -up.elbow_rad,
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

/// Ground place target for fixed-base clutter episodes. Matches where the scripted
/// IK pick-place carry (`IkClutterPickPlacePolicy`) sets the center cube down under
/// the stable arm dynamics — re-derived under two-finger weld + retarget (see
/// `env::mobile_manipulator::episode::tests::ik_clutter_policy_completes_center_place`).
/// Re-derived again after `canonical_grasp_anchor` gained a forward standoff (see
/// `GRASP_FORWARD_STANDOFF_MARGIN_M` in `env::mobile_manipulator::sim`) so the carried
/// cube no longer visually embeds in the forearm/gripper mount: the standoff shifts
/// where the welded cube rides relative to the gripper base by a few centimeters, so
/// the carry's open-loop release point moves with it.
pub const MM_MINIMAL_CLUTTER_PLACE_X_M: f64 = 0.716;
/// Ground place height for fixed-base clutter episodes.
pub const MM_MINIMAL_CLUTTER_PLACE_Y_M: f64 = 0.03;
/// Ground place lateral target off the table edge (table spans z ∈ [-0.35, 0.35]).
pub const MM_MINIMAL_CLUTTER_PLACE_Z_M: f64 = -0.388;

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

/// Rotates a world-frame XZ offset into the mobile-base frame (positive yaw about +Y).
pub(crate) fn rotate_y_xz(x: f64, z: f64, angle_rad: f64) -> (f64, f64) {
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
    fn fk_matches_sim_mm_mobile_at_idle() {
        use crate::{mm_mobile_clutter_scene_path, MobileManipulatorAction, MobileManipulatorSim};

        let kin = MmMinimalKinematics::mm_mobile();
        let mut sim =
            MobileManipulatorSim::from_scene_path(&mm_mobile_clutter_scene_path()).expect("scene");
        for _ in 0..80 {
            sim.step(MobileManipulatorAction::default());
        }
        let obs = sim.observe();
        // FK in the base frame, then rotate by the settled base yaw so the check
        // holds even when the physics settle drifts differently per platform.
        let local = MmMinimalKinematics {
            base_y_m: obs.base_y_m,
            shoulder_x_m: 0.0,
            shoulder_z_m: 0.0,
            ..kin
        };
        let fk_local = local.forward_kinematics(MmMinimalJointTarget {
            shoulder_rad: obs.shoulder_position_rad,
            elbow_rad: obs.elbow_position_rad,
        });
        let (wx, wz) = rotate_y_xz(fk_local.x_m, fk_local.z_m, -obs.base_yaw_rad);
        let fk_x = obs.base_x_m + wx;
        let fk_y = fk_local.y_m;
        let fk_z = obs.base_z_m + kin.mobile_shoulder_z_offset_m() + wz;
        let gripper = sim
            .link_translation_m("gripper_base_link")
            .expect("gripper link");
        let err =
            ((fk_x - gripper.0).powi(2) + (fk_y - gripper.1).powi(2) + (fk_z - gripper.2).powi(2))
                .sqrt();
        assert!(
            err < 0.08,
            "mm_mobile FK should match sim after settle, err={err:.3} m sim=({:.3},{:.3},{:.3}) fk=({:.3},{:.3},{:.3}) yaw={:.3}",
            gripper.0,
            gripper.1,
            gripper.2,
            fk_x,
            fk_y,
            fk_z,
            obs.base_yaw_rad
        );
    }

    #[test]
    fn ik_at_base_roundtrip_with_yaw() {
        let kin = MmMinimalKinematics::mm_mobile();
        // Elbow-up branch (IK always returns elbow_rad <= 0).
        let joints = MmMinimalJointTarget {
            shoulder_rad: 0.5,
            elbow_rad: -0.7,
        };
        let base = (1.2_f64, 0.25_f64, -0.8_f64);
        for yaw_rad in [0.0, 0.6, -1.1, 2.4] {
            // Tip in the base frame (shoulder pivot at the origin).
            let local = MmMinimalKinematics {
                base_y_m: base.1,
                shoulder_x_m: 0.0,
                shoulder_z_m: 0.0,
                ..kin
            };
            let tip_local = local.forward_kinematics(joints);
            // rotate_y_xz(x, z, -yaw) applies the URDF-convention base yaw rotation.
            let (wx, wz) = rotate_y_xz(tip_local.x_m, tip_local.z_m, -yaw_rad);
            let target = MmMinimalGripperTarget::new(base.0 + wx, tip_local.y_m, base.2 + wz);
            let solved = kin
                .inverse_kinematics_at_base(base.0, base.1, base.2, yaw_rad, target)
                .expect("reachable rotated target");
            assert_relative_eq!(solved.shoulder_rad, joints.shoulder_rad, epsilon = 1e-9);
            assert_relative_eq!(solved.elbow_rad, joints.elbow_rad, epsilon = 1e-9);
        }
    }

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
    fn elbow_down_branch_reaches_same_targets_mirrored() {
        let kin = MmMinimalKinematics::mm_minimal();
        for (x_m, z_m) in [(0.75, -0.10), (0.785, -0.402), (0.82, 0.10), (0.6, 0.0)] {
            let target = MmMinimalGripperTarget::new(x_m, kin.shoulder_y_m(), z_m);
            let down = kin
                .inverse_kinematics_elbow_down(target)
                .expect("reachable target");
            let tip = kin.forward_kinematics(down);
            assert_relative_eq!(tip.x_m, x_m, epsilon = 1e-9);
            assert_relative_eq!(tip.z_m, z_m, epsilon = 1e-9);
            assert!(
                down.elbow_rad >= 0.0,
                "elbow-down branch should bend the elbow the other way"
            );
        }
    }

    #[test]
    fn clutter_scene_cubes_are_within_analytic_reach() {
        // The clutter cubes were re-homed inside the analytic workspace when the
        // arm's settle physics were fixed: the old layout (x = 1.05..1.22) was only
        // pickable because the unstable arm's contact chaos stretched its impulse
        // joints past the kinematic reach limit, which no longer exists.
        let kin = MmMinimalKinematics::mm_minimal();
        for (name, x_m, z_m) in [
            ("clutter_cube_a", 0.79, 0.20),
            ("clutter_cube_b", 0.75, -0.10),
            ("clutter_cube_c", 0.72, 0.31),
        ] {
            assert!(
                kin.is_reachable(x_m, z_m),
                "{name} should be inside the analytic workspace"
            );
        }
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
