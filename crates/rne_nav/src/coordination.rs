//! Reservation-based multi-robot traffic coordination.
//!
//! [`TrafficCoordinator`] serializes robots through shared cells. Each robot
//! requests the cells its next plan step needs; the coordinator grants a lease
//! for `claim_horizon_s` only when no other robot holds an unexpired claim, so
//! paths that cross are ordered rather than collided. [`TrafficCoordinator::resolve_deadlock`]
//! breaks a mutual wait by giving the highest-priority (lowest-id) robot the
//! cells it needs and evicting only lower-priority holders.
//!
//! Storage is a `BTreeMap` and every scan is id-ordered, so coordination is
//! deterministic and reproducible.

use crate::grid::GridCoord;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Errors raised by the traffic coordinator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum TrafficError {
    /// The claim horizon was non-finite or non-positive.
    #[error("invalid traffic configuration")]
    InvalidConfig,
    /// A time value was non-finite.
    #[error("non-finite traffic time")]
    NonFiniteTime,
}

/// Configuration for [`TrafficCoordinator`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrafficConfig {
    /// How long a granted claim stays valid, in seconds.
    pub claim_horizon_s: f64,
}

impl Default for TrafficConfig {
    fn default() -> Self {
        Self {
            claim_horizon_s: 2.0,
        }
    }
}

impl TrafficConfig {
    /// Whether the configuration is finite and positive.
    pub fn is_valid(&self) -> bool {
        self.claim_horizon_s.is_finite() && self.claim_horizon_s > 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Claim {
    robot_id: u32,
    expires_s: f64,
}

/// A deterministic cell-reservation traffic manager.
#[derive(Clone, Debug, PartialEq)]
pub struct TrafficCoordinator {
    config: TrafficConfig,
    claims: BTreeMap<GridCoord, Claim>,
}

impl TrafficCoordinator {
    /// Creates a coordinator after validating the configuration.
    pub fn new(config: TrafficConfig) -> Result<Self, TrafficError> {
        if !config.is_valid() {
            return Err(TrafficError::InvalidConfig);
        }
        Ok(Self {
            config,
            claims: BTreeMap::new(),
        })
    }

    /// The configuration.
    pub fn config(&self) -> TrafficConfig {
        self.config
    }

    /// Number of active claims.
    pub fn claim_count(&self) -> usize {
        self.claims.len()
    }

    /// The robot currently holding a cell, if any (expire first with
    /// [`TrafficCoordinator::expire`] for current leases).
    pub fn owner(&self, coord: GridCoord) -> Option<u32> {
        self.claims.get(&coord).map(|claim| claim.robot_id)
    }

    /// Drops expired claims, returning how many were removed.
    pub fn expire(&mut self, now_s: f64) -> usize {
        if !now_s.is_finite() {
            return 0;
        }
        let before = self.claims.len();
        self.claims.retain(|_, claim| claim.expires_s > now_s);
        before - self.claims.len()
    }

    /// Releases every claim held by `robot_id`.
    pub fn release(&mut self, robot_id: u32) {
        self.claims.retain(|_, claim| claim.robot_id != robot_id);
    }

    /// Requests a lease on `coords` for `robot_id`.
    ///
    /// Returns `true` when the lease was granted. A request that overlaps an
    /// unexpired claim held by another robot is denied.
    pub fn request(
        &mut self,
        robot_id: u32,
        coords: &[GridCoord],
        now_s: f64,
    ) -> Result<bool, TrafficError> {
        if !now_s.is_finite() {
            return Err(TrafficError::NonFiniteTime);
        }
        self.expire(now_s);
        for coord in coords {
            if let Some(claim) = self.claims.get(coord) {
                if claim.robot_id != robot_id && claim.expires_s > now_s {
                    return Ok(false);
                }
            }
        }
        let expires_s = now_s + self.config.claim_horizon_s;
        for coord in coords {
            self.claims.insert(
                *coord,
                Claim {
                    robot_id,
                    expires_s,
                },
            );
        }
        Ok(true)
    }

    /// Grants the highest-priority waiting robot by evicting lower-priority
    /// blockers.
    ///
    /// `waiting` is a list of `(robot_id, requested_cells)`. Returns the id of
    /// the robot that was granted its cells, or `None` if no waiter could be
    /// granted without preempting a higher-priority robot.
    pub fn resolve_deadlock(
        &mut self,
        waiting: &[(u32, Vec<GridCoord>)],
        now_s: f64,
    ) -> Result<Option<u32>, TrafficError> {
        if !now_s.is_finite() {
            return Err(TrafficError::NonFiniteTime);
        }
        self.expire(now_s);
        let mut order: Vec<usize> = (0..waiting.len()).collect();
        order.sort_by_key(|index| waiting[*index].0);

        for index in order {
            let (robot_id, coords) = &waiting[index];
            let mut blocked_by_higher_priority = false;
            for coord in coords {
                if let Some(claim) = self.claims.get(coord) {
                    if claim.robot_id != *robot_id && claim.robot_id < *robot_id {
                        blocked_by_higher_priority = true;
                        break;
                    }
                }
            }
            if blocked_by_higher_priority {
                continue;
            }
            for coord in coords {
                if let Some(claim) = self.claims.get(coord) {
                    if claim.robot_id != *robot_id && claim.robot_id > *robot_id {
                        self.claims.remove(coord);
                    }
                }
            }
            if self.request(*robot_id, coords, now_s)? {
                return Ok(Some(*robot_id));
            }
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coord(x: isize, y: isize) -> GridCoord {
        GridCoord { x, y }
    }

    #[test]
    fn grants_free_cells_and_denies_conflicts() {
        let mut coordinator = TrafficCoordinator::new(TrafficConfig::default()).unwrap();
        let path = [coord(0, 0), coord(1, 0)];
        assert!(coordinator.request(1, &path, 0.0).unwrap());
        assert!(!coordinator.request(2, &[coord(1, 0)], 0.0).unwrap());
        assert!(coordinator.request(2, &[coord(5, 5)], 0.0).unwrap());
        coordinator.release(1);
        assert!(coordinator.request(2, &path, 0.0).unwrap());
    }

    #[test]
    fn claims_expire() {
        let config = TrafficConfig {
            claim_horizon_s: 1.0,
        };
        let mut coordinator = TrafficCoordinator::new(config).unwrap();
        assert!(coordinator.request(1, &[coord(2, 2)], 0.0).unwrap());
        assert!(!coordinator.request(2, &[coord(2, 2)], 0.5).unwrap());
        assert_eq!(coordinator.expire(1.5), 1);
        assert!(coordinator.request(2, &[coord(2, 2)], 1.5).unwrap());
        assert_eq!(coordinator.owner(coord(2, 2)), Some(2));
    }

    #[test]
    fn deadlock_grants_the_lowest_id_robot() {
        let mut coordinator = TrafficCoordinator::new(TrafficConfig::default()).unwrap();
        // Two robots each hold a cell the other needs: a mutual wait.
        assert!(coordinator.request(2, &[coord(1, 0)], 0.0).unwrap());
        assert!(coordinator.request(3, &[coord(0, 1)], 0.0).unwrap());
        let waiting = vec![(2, vec![coord(0, 1)]), (3, vec![coord(1, 0)])];
        // Robot 2 has priority and only robot 3 (higher id) blocks it.
        assert_eq!(
            coordinator.resolve_deadlock(&waiting, 0.1).unwrap(),
            Some(2)
        );
        assert_eq!(coordinator.owner(coord(0, 1)), Some(2));
    }

    #[test]
    fn lower_id_blocker_is_not_preempted() {
        let mut coordinator = TrafficCoordinator::new(TrafficConfig::default()).unwrap();
        assert!(coordinator.request(1, &[coord(4, 4)], 0.0).unwrap());
        // Robot 2 waits on a cell held by the higher-priority robot 1.
        let waiting = vec![(2, vec![coord(4, 4)])];
        assert_eq!(coordinator.resolve_deadlock(&waiting, 0.1).unwrap(), None);
    }

    #[test]
    fn rejects_invalid_config_and_time() {
        let bad = TrafficConfig {
            claim_horizon_s: 0.0,
        };
        assert_eq!(
            TrafficCoordinator::new(bad),
            Err(TrafficError::InvalidConfig)
        );
        let mut coordinator = TrafficCoordinator::new(TrafficConfig::default()).unwrap();
        assert_eq!(
            coordinator.request(1, &[coord(0, 0)], f64::NAN),
            Err(TrafficError::NonFiniteTime)
        );
    }
}
