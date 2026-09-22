//! MOBIL lane-change decision model.
//!
//! Minimizing Overall Braking Induced by Lane changes (MOBIL) scores a candidate
//! lane change by the acceleration gained by the changing vehicle plus a
//! politeness-weighted change for the new and old followers, subject to a safety
//! criterion on the new follower. Accelerations are supplied by the caller (for
//! example from [`crate::car_following::idm_acceleration`]), so the decision is a
//! pure function of the surrounding traffic and is fully deterministic.

use crate::car_following::{idm_acceleration, IdmParams};

/// One neighbouring vehicle on a lane, relative to the subject.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MobilNeighbor {
    /// Neighbour speed in meters per second.
    pub speed_m_s: f64,
    /// Bumper-to-bumper gap to the subject in meters.
    pub gap_m: f64,
}

/// MOBIL parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MobilParams {
    /// Politeness factor in `[0, 1]`; `0` is selfish.
    pub politeness: f64,
    /// Maximum deceleration the new follower may be forced to accept, m/s^2.
    pub safe_braking_m_s2: f64,
    /// Minimum incentive required to change lanes, m/s^2.
    pub minimum_incentive_m_s2: f64,
}

impl Default for MobilParams {
    fn default() -> Self {
        Self {
            politeness: 0.2,
            safe_braking_m_s2: 3.0,
            minimum_incentive_m_s2: 0.2,
        }
    }
}

impl MobilParams {
    /// Returns true when every parameter is finite and physically valid.
    pub fn is_valid(&self) -> bool {
        self.politeness.is_finite()
            && (0.0..=1.0).contains(&self.politeness)
            && self.safe_braking_m_s2.is_finite()
            && self.safe_braking_m_s2 > 0.0
            && self.minimum_incentive_m_s2.is_finite()
    }
}

/// MOBIL incentive for a candidate lane change, in meters per second squared.
///
/// The four follower arguments are accelerations (m/s^2) before and after the
/// change for the old-lane follower and the new-lane follower respectively.
pub fn mobil_incentive(
    params: &MobilParams,
    own_acceleration_before: f64,
    own_acceleration_after: f64,
    old_follower_before: f64,
    old_follower_after: f64,
    new_follower_before: f64,
    new_follower_after: f64,
) -> f64 {
    let own_gain = own_acceleration_after - own_acceleration_before;
    let new_follower_gain = new_follower_after - new_follower_before;
    let old_follower_gain = old_follower_after - old_follower_before;
    own_gain + params.politeness * (new_follower_gain + old_follower_gain)
}

/// MOBIL safety criterion: the new follower must not brake harder than allowed.
pub fn mobil_safe(params: &MobilParams, new_follower_after: f64) -> bool {
    new_follower_after >= -params.safe_braking_m_s2
}

/// Full MOBIL decision: safe and sufficiently incentivized.
pub fn mobil_should_change(
    params: &MobilParams,
    incentive_m_s2: f64,
    new_follower_after: f64,
) -> bool {
    incentive_m_s2 > params.minimum_incentive_m_s2 && mobil_safe(params, new_follower_after)
}

/// Composes IDM car-following with the MOBIL criterion into one decision.
///
/// `current_*` describe the subject's lane and `target_*` the candidate lane.
/// Accelerations are computed with IDM for the subject and for both followers
/// before and after the change, then scored with [`mobil_incentive`] and
/// [`mobil_should_change`]. The old follower's post-change gap is approximated as
/// the sum of its gap to the subject and the subject's gap to its current leader.
#[allow(clippy::too_many_arguments)]
pub fn mobil_idm_decision(
    idm: &IdmParams,
    mobil: &MobilParams,
    subject_speed_m_s: f64,
    current_leader: Option<MobilNeighbor>,
    current_follower: Option<MobilNeighbor>,
    target_leader: Option<MobilNeighbor>,
    target_follower: Option<MobilNeighbor>,
) -> bool {
    let follow = |speed: f64, leader: Option<MobilNeighbor>| match leader {
        Some(leader) => idm_acceleration(idm, speed, leader.speed_m_s, leader.gap_m),
        // No leader: effectively free flow.
        None => idm_acceleration(idm, speed, speed, 1.0e3),
    };

    let own_before = follow(subject_speed_m_s, current_leader);
    let own_after = follow(subject_speed_m_s, target_leader);

    let (old_follower_before, old_follower_after) = match current_follower {
        None => (0.0, 0.0),
        Some(follower) => {
            let before = follow(
                follower.speed_m_s,
                Some(MobilNeighbor {
                    speed_m_s: subject_speed_m_s,
                    gap_m: follower.gap_m,
                }),
            );
            let after_leader = current_leader.map(|leader| MobilNeighbor {
                speed_m_s: leader.speed_m_s,
                gap_m: follower.gap_m + leader.gap_m,
            });
            (before, follow(follower.speed_m_s, after_leader))
        }
    };

    let (new_follower_before, new_follower_after) = match target_follower {
        None => (0.0, 0.0),
        Some(follower) => {
            let before = follow(follower.speed_m_s, target_leader);
            let after = follow(
                follower.speed_m_s,
                Some(MobilNeighbor {
                    speed_m_s: subject_speed_m_s,
                    gap_m: follower.gap_m,
                }),
            );
            (before, after)
        }
    };

    let incentive = mobil_incentive(
        mobil,
        own_before,
        own_after,
        old_follower_before,
        old_follower_after,
        new_follower_before,
        new_follower_after,
    );
    mobil_should_change(mobil, incentive, new_follower_after)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn incentive_is_positive_when_the_change_helps() {
        let params = MobilParams::default();
        let incentive = mobil_incentive(&params, 0.0, 1.0, 0.0, 0.5, 0.0, 0.5);
        assert!(incentive > 0.0, "incentive {incentive}");
        assert!(mobil_should_change(&params, incentive, 0.5));
    }

    #[test]
    fn incentive_is_negative_when_the_change_hurts() {
        let params = MobilParams::default();
        let incentive = mobil_incentive(&params, 0.0, -1.0, 0.0, -0.5, 0.0, -0.5);
        assert!(incentive < 0.0, "incentive {incentive}");
        assert!(!mobil_should_change(&params, incentive, -0.5));
    }

    #[test]
    fn safety_rejects_a_hard_braking_new_follower() {
        let params = MobilParams::default();
        assert!(!mobil_safe(&params, -5.0));
        // A selfish driver still cannot force the new follower to brake hard.
        let incentive = mobil_incentive(&params, 0.0, 10.0, 0.0, 0.0, 0.0, -5.0);
        assert!(!mobil_should_change(&params, incentive, -5.0));
    }

    #[test]
    fn politeness_penalizes_harming_the_new_follower() {
        let selfish = MobilParams {
            politeness: 0.0,
            ..MobilParams::default()
        };
        let polite = MobilParams {
            politeness: 1.0,
            ..MobilParams::default()
        };
        let selfish_incentive = mobil_incentive(&selfish, 0.0, 0.5, 0.0, 0.0, 0.0, -2.0);
        let polite_incentive = mobil_incentive(&polite, 0.0, 0.5, 0.0, 0.0, 0.0, -2.0);
        assert!(polite_incentive < selfish_incentive);
    }

    #[test]
    fn idm_mobil_changes_when_the_target_lane_is_free() {
        let decision = mobil_idm_decision(
            &IdmParams::default(),
            &MobilParams::default(),
            10.0,
            Some(MobilNeighbor {
                speed_m_s: 2.0,
                gap_m: 8.0,
            }),
            None,
            None,
            None,
        );
        assert!(decision, "a free target lane should be attractive");
    }

    #[test]
    fn idm_mobil_rejects_a_hard_braking_new_follower() {
        let decision = mobil_idm_decision(
            &IdmParams::default(),
            &MobilParams::default(),
            10.0,
            Some(MobilNeighbor {
                speed_m_s: 2.0,
                gap_m: 8.0,
            }),
            None,
            None,
            Some(MobilNeighbor {
                speed_m_s: 10.0,
                gap_m: 1.0,
            }),
        );
        assert!(!decision, "safety must reject a hard-braking new follower");
    }

    #[test]
    fn idm_mobil_holds_when_both_lanes_are_equal() {
        let leader = Some(MobilNeighbor {
            speed_m_s: 5.0,
            gap_m: 20.0,
        });
        let decision = mobil_idm_decision(
            &IdmParams::default(),
            &MobilParams::default(),
            5.0,
            leader,
            None,
            leader,
            None,
        );
        assert!(!decision, "no incentive when the lanes are equivalent");
    }
}
