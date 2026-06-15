//! Transport helpers for grasp-and-move manipulation smoke tests.

use crate::grasp::finger_contacts_named;
use crate::MobileManipulatorSim;

/// Minimum object displacement for transport smoke success.
pub const TRANSPORT_SUCCESS_M: f64 = 0.02;

/// World-frame translation of a named scene body.
pub fn named_translation_m(sim: &MobileManipulatorSim, name: &str) -> Option<(f64, f64, f64)> {
    sim.named_translation_m(name)
}

/// Euclidean distance between two world-frame positions in meters.
pub fn displacement_m(initial: (f64, f64, f64), current: (f64, f64, f64)) -> f64 {
    let dx = current.0 - initial.0;
    let dy = current.1 - initial.1;
    let dz = current.2 - initial.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Returns true when the named body moved at least `min_m` from its initial pose.
pub fn body_moved_at_least_m(
    sim: &MobileManipulatorSim,
    body_name: &str,
    initial: (f64, f64, f64),
    min_m: f64,
) -> bool {
    named_translation_m(sim, body_name)
        .map(|current| displacement_m(initial, current) >= min_m)
        .unwrap_or(false)
}

/// Returns true when fingers contacted the body at least once during the rollout.
pub fn had_finger_contact(sim: &MobileManipulatorSim, body_name: &str, contacted: bool) -> bool {
    contacted || finger_contacts_named(sim, body_name)
}

/// Returns true when a named body is within a zone obstacle's horizontal footprint.
pub fn body_within_zone_m(
    sim: &MobileManipulatorSim,
    body_name: &str,
    zone_name: &str,
    half_extent_m: f64,
) -> bool {
    match (
        named_translation_m(sim, body_name),
        named_translation_m(sim, zone_name),
    ) {
        (Some(body), Some(zone)) => {
            let dx = (body.0 - zone.0).abs();
            let dz = (body.2 - zone.2).abs();
            dx <= half_extent_m && dz <= half_extent_m
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mm_minimal_transport_scene_path, MobileManipulatorAction, MobileManipulatorSim};

    #[test]
    fn dynamic_cube_moves_after_grasp_and_shoulder_sweep() {
        let scene_path = mm_minimal_transport_scene_path();

        for _ in 0..3 {
            let mut sim =
                MobileManipulatorSim::from_scene_path(&scene_path).expect("load transport scene");
            let initial = named_translation_m(&sim, "grasp_cube").expect("grasp_cube pose");

            let close = MobileManipulatorAction {
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            };
            let transport = MobileManipulatorAction {
                gripper_velocity_rad_s: -2.0,
                shoulder_velocity_rad_s: 4.0,
                ..MobileManipulatorAction::default()
            };

            let mut contacted = false;
            for _ in 0..120 {
                sim.step(close);
                contacted = had_finger_contact(&sim, "grasp_cube", contacted);
            }
            for _ in 0..600 {
                sim.step(transport);
                contacted = had_finger_contact(&sim, "grasp_cube", contacted);
            }

            if contacted && body_moved_at_least_m(&sim, "grasp_cube", initial, TRANSPORT_SUCCESS_M)
            {
                return;
            }
        }

        panic!(
            "expected finger contact and grasp_cube displacement >= {} m",
            TRANSPORT_SUCCESS_M
        );
    }
}
