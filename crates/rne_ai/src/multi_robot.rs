//! Multi-robot helpers for contact detection and peer-relative observations.

use crate::env::DiffDriveSim;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::ContactEvent;
use rne_world::Transform3;
use std::collections::HashSet;

/// Peer-relative offsets and distance to the nearest other robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PeerObservation {
    /// Nearest peer base X minus this robot base X in meters.
    pub delta_x_m: f64,
    /// Nearest peer base Z minus this robot base Z in meters.
    pub delta_z_m: f64,
    /// Euclidean separation between base links in meters.
    pub separation_m: f64,
}

/// Prebuilt head-on collision scenario with two joint-driven robots.
pub fn head_on_collision_configs() -> [rne_robot::DiffDriveConfig; 2] {
    use rne_robot::{DiffDriveConfig, DiffDriveDriveMode};
    [
        DiffDriveConfig {
            model_name: "robot_a".into(),
            initial_translation_m: Vec3::new(0.0, 0.25, 0.0),
            drive_mode: DiffDriveDriveMode::JointDriven,
            ..DiffDriveConfig::default()
        },
        DiffDriveConfig {
            model_name: "robot_b".into(),
            initial_translation_m: Vec3::new(0.48, 0.25, 0.0),
            drive_mode: DiffDriveDriveMode::JointDriven,
            ..DiffDriveConfig::default()
        },
    ]
}

/// Creates a two-robot simulation configured for a head-on interaction.
pub fn head_on_collision_sim() -> DiffDriveSim {
    let configs = head_on_collision_configs();
    DiffDriveSim::with_robot_configs(&configs)
}

/// Returns contact events from the last physics step.
pub fn last_contacts(sim: &DiffDriveSim) -> &[ContactEvent] {
    sim.last_contacts()
}

/// Returns contacts where both entities belong to different robots.
pub fn inter_robot_contacts(sim: &DiffDriveSim) -> Vec<ContactEvent> {
    let robot_entities = robot_body_entities(sim);
    last_contacts(sim)
        .iter()
        .copied()
        .filter(|contact| {
            robot_entities.contains(&contact.entity_a)
                && robot_entities.contains(&contact.entity_b)
                && robot_for_body(sim, contact.entity_a) != robot_for_body(sim, contact.entity_b)
        })
        .collect()
}

/// Returns true when two robots touched during the last physics step.
pub fn robots_in_contact(sim: &DiffDriveSim, robot_a: Entity, robot_b: Entity) -> bool {
    inter_robot_contacts(sim).iter().any(|contact| {
        let left = robot_for_body(sim, contact.entity_a);
        let right = robot_for_body(sim, contact.entity_b);
        (left == robot_a && right == robot_b) || (left == robot_b && right == robot_a)
    })
}

/// Computes peer-relative observation data for the nearest other robot.
pub fn nearest_peer_observation(sim: &DiffDriveSim, robot: Entity) -> Option<PeerObservation> {
    let self_transform = base_transform(sim, robot)?;
    let mut best: Option<(f64, PeerObservation)> = None;

    for peer in sim.robots() {
        if peer.robot == robot {
            continue;
        }
        let peer_transform = base_transform(sim, peer.robot)?;
        let delta = peer_transform.translation - self_transform.translation;
        let separation_m = delta.length();
        let peer_obs = PeerObservation {
            delta_x_m: delta.x,
            delta_z_m: delta.z,
            separation_m,
        };
        if best
            .as_ref()
            .is_none_or(|(best_sep, _)| separation_m < *best_sep)
        {
            best = Some((separation_m, peer_obs));
        }
    }

    best.map(|(_, observation)| observation)
}

/// Returns the Euclidean distance between two robot base links.
pub fn robot_separation_m(sim: &DiffDriveSim, robot_a: Entity, robot_b: Entity) -> Option<f64> {
    let transform_a = base_transform(sim, robot_a)?;
    let transform_b = base_transform(sim, robot_b)?;
    Some((transform_b.translation - transform_a.translation).length())
}

fn base_transform(sim: &DiffDriveSim, robot: Entity) -> Option<Transform3> {
    let spawned = sim.robots().iter().find(|spawned| spawned.robot == robot)?;
    sim.world().get::<Transform3>(spawned.base_link).copied()
}

fn robot_body_entities(sim: &DiffDriveSim) -> HashSet<Entity> {
    sim.robots()
        .iter()
        .flat_map(|robot| [robot.base_link, robot.left_wheel, robot.right_wheel])
        .collect()
}

fn robot_for_body(sim: &DiffDriveSim, entity: Entity) -> Entity {
    sim.robots()
        .iter()
        .find(|robot| {
            robot.base_link == entity || robot.left_wheel == entity || robot.right_wheel == entity
        })
        .map(|robot| robot.robot)
        .unwrap_or(entity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DiffDriveAction;

    #[test]
    fn head_on_robots_make_contact() {
        let mut sim = head_on_collision_sim();
        let robot_a = sim.robots()[0].robot;
        let robot_b = sim.robots()[1].robot;

        let mut contacted = false;
        for _ in 0..400 {
            sim.step_robots_actions(&[
                (robot_a, DiffDriveAction::forward(6.0)),
                (robot_b, DiffDriveAction::forward(-6.0)),
            ]);
            if robots_in_contact(&sim, robot_a, robot_b) {
                contacted = true;
                break;
            }
        }

        assert!(
            contacted,
            "expected inter-robot contact during head-on rollout"
        );
    }

    #[test]
    fn nearest_peer_observation_tracks_approach() {
        let mut sim = head_on_collision_sim();
        let robot_a = sim.robots()[0].robot;
        let robot_b = sim.robots()[1].robot;

        let initial = nearest_peer_observation(&sim, robot_a).expect("peer observation");
        assert!(initial.delta_x_m > 0.0);

        let mut min_separation = initial.separation_m;
        let mut contacted = false;
        for _ in 0..120 {
            sim.step_robots_actions(&[
                (robot_a, DiffDriveAction::forward(6.0)),
                (robot_b, DiffDriveAction::forward(-6.0)),
            ]);
            min_separation =
                min_separation.min(robot_separation_m(&sim, robot_a, robot_b).unwrap());
            contacted |= robots_in_contact(&sim, robot_a, robot_b);
        }

        assert!(
            contacted || min_separation < initial.separation_m,
            "peer tracking should see approach or contact: initial={} min={min_separation} contacted={contacted}",
            initial.separation_m
        );
    }
}
