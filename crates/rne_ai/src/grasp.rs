//! Grasp and contact helpers for mobile manipulator manipulation smoke tests.

use crate::MobileManipulatorSim;
use rne_ecs::Entity;

/// Link names used for parallel-jaw contact checks.
pub const FINGER_LINK_NAMES: [&str; 2] = ["left_finger_link", "right_finger_link"];

/// Returns true when any finger link contacts the named scene body from the last step.
pub fn finger_contacts_named(sim: &MobileManipulatorSim, body_name: &str) -> bool {
    FINGER_LINK_NAMES
        .iter()
        .any(|finger| sim_contacts_named(sim, finger, body_name))
}

/// Returns true when the two named entities contacted on the last physics step.
pub fn sim_contacts_named(sim: &MobileManipulatorSim, entity_a: &str, entity_b: &str) -> bool {
    match (sim.entity_named(entity_a), sim.entity_named(entity_b)) {
        (Some(a), Some(b)) => sim.contacts_between(a, b),
        _ => false,
    }
}

/// Returns the first named entity in the simulation world.
pub fn entity_named(sim: &MobileManipulatorSim, name: &str) -> Option<Entity> {
    sim.entity_named(name)
}
