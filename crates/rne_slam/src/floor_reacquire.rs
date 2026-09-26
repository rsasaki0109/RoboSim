//! Recovering which floor a robot is on after a lift carries it.
//!
//! A robot riding a lift is not navigating: its wheels are still, its odometry
//! reports no motion, and its scan sees the same car walls the whole way. Yet
//! its pose in the building changes by a whole storey. When the doors open, the
//! robot has to answer a question odometry cannot: *where am I now*.
//!
//! The question is bounded, which is what makes it tractable. The robot knows
//! which lift it boarded and therefore which floors that lift serves, so the
//! hypothesis set is one pose per served floor, near that lift's alighting
//! point — not the whole building.
//!
//! **The hard part is not finding a match; it is that floors look alike.** An
//! office stairwell landing on 3F and on 4F can be identical to a planar
//! scanner, and a matcher asked for its best candidate will return one with
//! high confidence. Placing a robot on the wrong floor is worse than admitting
//! ignorance: it will navigate confidently to a room that is not there. So an
//! identification here is accepted only when the best hypothesis beats the
//! runner-up by a declared margin, and the runner-up is always reported.
//!
//! That margin is the real defence. [`ReacquisitionConfig::min_score`] is only a
//! sanity floor, and a weak one: mean scan likelihood is dominated by whatever
//! the candidate floors have in common, so a map missing a full-height interior
//! partition still scores 0.83 against a scan taken on the partitioned floor.
//! A caller relying on a single candidate — a lift that serves one floor, or a
//! robot that has narrowed the set some other way — has no margin to fall back
//! on and should raise `min_score` deliberately.

use crate::likelihood::{LikelihoodConfig, LikelihoodField};
use crate::scan_match::{ScanMatchConfig, ScanMatcher};
use rne_math::Vec3;
use rne_nav::{FloorId, OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};

/// One floor a lift could have delivered the robot to.
#[derive(Clone, Debug)]
pub struct FloorCandidate<'a> {
    /// Floor this hypothesis refers to.
    pub floor: FloorId,
    /// That floor's occupancy map.
    pub map: &'a OccupancyGrid,
    /// Where the robot expects to be standing on it, in that floor's frame.
    pub expected_pose: Pose2d,
}

/// A scored floor hypothesis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloorHypothesis {
    /// Floor this hypothesis refers to.
    pub floor: FloorId,
    /// Best matched pose on that floor.
    pub pose: Pose2d,
    /// Mean scan likelihood at that pose, in `[0, 1]`.
    pub score: f64,
}

/// The outcome of a floor reacquisition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloorIdentification {
    /// Winning hypothesis.
    pub best: FloorHypothesis,
    /// Closest rival, when the lift served more than one candidate floor.
    ///
    /// Always reported, because it is the evidence for how safe the answer is.
    pub runner_up: Option<FloorHypothesis>,
    /// How far the winner beat the runner-up; `f64::INFINITY` when alone.
    pub margin: f64,
}

/// Why a reacquisition produced no answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FloorAmbiguity {
    /// No candidate matched well enough to be believed at all.
    NothingMatched,
    /// Two floors matched comparably well, so the answer would be a guess.
    ///
    /// This is the identical-floor-plan case, and the honest outcome is to stay
    /// lost rather than to navigate confidently on the wrong storey.
    Ambiguous,
}

/// Settings for floor reacquisition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReacquisitionConfig {
    /// Likelihood field built from each candidate floor's map.
    pub likelihood: LikelihoodConfig,
    /// Local search around each candidate's expected pose.
    pub matcher: ScanMatchConfig,
    /// Score a hypothesis must reach to be believed at all.
    pub min_score: f64,
    /// Score lead the winner must hold over the runner-up.
    pub min_margin: f64,
}

impl Default for ReacquisitionConfig {
    fn default() -> Self {
        Self {
            likelihood: LikelihoodConfig::default(),
            matcher: ScanMatchConfig::default(),
            min_score: 0.5,
            // Floors of one building are often near-identical; a small lead is
            // noise, not evidence.
            min_margin: 0.08,
        }
    }
}

impl ReacquisitionConfig {
    /// Returns whether the thresholds are finite and positive.
    pub fn is_valid(&self) -> bool {
        self.min_score.is_finite()
            && self.min_score > 0.0
            && self.min_margin.is_finite()
            && self.min_margin > 0.0
    }
}

/// Error raised while reacquiring a floor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ReacquisitionError {
    /// A threshold was non-finite or non-positive.
    #[error("reacquisition thresholds must be finite and positive")]
    InvalidConfig,
    /// No candidate floors were supplied.
    #[error("reacquisition requires at least one candidate floor")]
    NoCandidates,
    /// The scan was empty or contained non-finite points.
    #[error("reacquisition requires a non-empty finite scan")]
    InvalidScan,
}

/// Identifies which candidate floor a scan was taken on.
///
/// Each candidate is matched locally around its expected pose, so the search is
/// the size of a lift lobby rather than a building. Candidates are scored in the
/// order supplied and ties break toward the earlier one, so the same inputs
/// always produce the same answer.
///
/// Returns `Err` for unusable inputs, `Ok(Err(..))` when the evidence does not
/// support an answer, and `Ok(Ok(..))` only when one floor both matched well and
/// beat every rival by [`ReacquisitionConfig::min_margin`].
#[allow(clippy::result_large_err)]
pub fn reacquire_floor(
    candidates: &[FloorCandidate<'_>],
    scan_points_sensor_m: &[Vec3],
    sensor_from_base: Pose2d,
    config: &ReacquisitionConfig,
) -> Result<Result<FloorIdentification, FloorAmbiguity>, ReacquisitionError> {
    if !config.is_valid() {
        return Err(ReacquisitionError::InvalidConfig);
    }
    if candidates.is_empty() {
        return Err(ReacquisitionError::NoCandidates);
    }
    if scan_points_sensor_m.is_empty()
        || scan_points_sensor_m.iter().any(|point| !point.is_finite())
        || !sensor_from_base.is_finite()
    {
        return Err(ReacquisitionError::InvalidScan);
    }

    let matcher = ScanMatcher::new(config.matcher);
    let mut scored: Vec<FloorHypothesis> = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        if !candidate.expected_pose.is_finite() {
            return Err(ReacquisitionError::InvalidScan);
        }
        let Ok(field) = LikelihoodField::from_occupancy(candidate.map, &config.likelihood) else {
            continue;
        };
        let Some(result) = matcher.match_scan(
            &field,
            scan_points_sensor_m,
            sensor_from_base,
            candidate.expected_pose,
        ) else {
            continue;
        };
        scored.push(FloorHypothesis {
            floor: candidate.floor,
            pose: result.pose,
            score: result.score,
        });
    }

    // Stable ordering: strictly greater score wins, so an equal score leaves the
    // earlier candidate in front and the result is reproducible.
    let mut ranked = scored;
    ranked.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let Some(best) = ranked.first().copied() else {
        return Ok(Err(FloorAmbiguity::NothingMatched));
    };
    if best.score < config.min_score {
        return Ok(Err(FloorAmbiguity::NothingMatched));
    }
    let runner_up = ranked.get(1).copied();
    let margin = runner_up.map_or(f64::INFINITY, |rival| best.score - rival.score);
    if margin < config.min_margin {
        return Ok(Err(FloorAmbiguity::Ambiguous));
    }
    Ok(Ok(FloorIdentification {
        best,
        runner_up,
        margin,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::GridCoord;

    /// A 10 m x 10 m floor with walls, plus optional distinguishing features.
    ///
    /// `features` are extra wall cells that make one floor tell itself apart
    /// from another with the same outline.
    fn floor_map(features: &[(isize, isize)]) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(40, 40, 0.25, Pose2d::new(0.0, 0.0, 0.0)).expect("grid");
        for y in 0..40isize {
            for x in 0..40isize {
                for _ in 0..8 {
                    grid.mark_free(GridCoord { x, y });
                }
            }
        }
        let mut wall = |x: isize, y: isize| {
            for _ in 0..8 {
                grid.mark_occupied(GridCoord { x, y });
            }
        };
        for i in 0..40isize {
            wall(i, 0);
            wall(i, 39);
            wall(0, i);
            wall(39, i);
        }
        for (x, y) in features {
            wall(*x, *y);
        }
        grid
    }

    /// Simulates a scan of every occupied cell within range of a true pose.
    fn simulated_scan(grid: &OccupancyGrid, true_pose: Pose2d, max_range_m: f64) -> Vec<Vec3> {
        let mut points = Vec::new();
        for y in 0..grid.height() as isize {
            for x in 0..grid.width() as isize {
                let coord = GridCoord { x, y };
                if grid.probability(coord).unwrap_or(0.5) < 0.6 {
                    continue;
                }
                let world = grid.grid_to_world(coord);
                let dx = world.x - true_pose.x_m;
                let dy = world.y - true_pose.y_m;
                if (dx * dx + dy * dy).sqrt() <= max_range_m {
                    points.push(true_pose.inverse_transform_point(world));
                }
            }
        }
        points
    }

    fn identity() -> Pose2d {
        Pose2d::new(0.0, 0.0, 0.0)
    }

    #[test]
    fn a_distinctive_floor_is_identified_and_reports_its_closest_rival() {
        // Two floors sharing an outline; only the second is partitioned. A
        // handful of extra cells would not settle it, and should not: the
        // difference has to be something a scanner actually sees.
        let plain = floor_map(&[]);
        let partition: Vec<(isize, isize)> = (4..36).map(|y| (20isize, y)).collect();
        let alcove = floor_map(&partition);
        let truth = Pose2d::new(5.0, 5.0, 0.0);
        let scan = simulated_scan(&alcove, truth, 12.0);

        let candidates = [
            FloorCandidate {
                floor: FloorId(0),
                map: &plain,
                expected_pose: truth,
            },
            FloorCandidate {
                floor: FloorId(1),
                map: &alcove,
                expected_pose: truth,
            },
        ];
        let identification = reacquire_floor(
            &candidates,
            &scan,
            identity(),
            &ReacquisitionConfig::default(),
        )
        .expect("inputs are valid")
        .expect("the alcove should settle it");
        assert_eq!(identification.best.floor, FloorId(1));
        assert!(identification.best.score >= 0.5);
        // The rival is always reported: it is the evidence for the answer.
        let runner_up = identification.runner_up.expect("a rival floor existed");
        assert_eq!(runner_up.floor, FloorId(0));
        assert!(identification.margin >= ReacquisitionConfig::default().min_margin);
    }

    #[test]
    fn identical_floors_are_refused_rather_than_guessed() {
        // The failure this module exists for: an office where two storeys are
        // the same to a planar scanner. Answering would place the robot on the
        // wrong floor with full confidence.
        let first = floor_map(&[]);
        let second = floor_map(&[]);
        let truth = Pose2d::new(5.0, 5.0, 0.0);
        let scan = simulated_scan(&first, truth, 12.0);

        let candidates = [
            FloorCandidate {
                floor: FloorId(3),
                map: &first,
                expected_pose: truth,
            },
            FloorCandidate {
                floor: FloorId(4),
                map: &second,
                expected_pose: truth,
            },
        ];
        assert_eq!(
            reacquire_floor(
                &candidates,
                &scan,
                identity(),
                &ReacquisitionConfig::default()
            )
            .expect("inputs are valid"),
            Err(FloorAmbiguity::Ambiguous)
        );
    }

    #[test]
    fn a_single_served_floor_needs_no_margin_but_still_needs_a_match() {
        let map = floor_map(&[]);
        let truth = Pose2d::new(5.0, 5.0, 0.0);
        let scan = simulated_scan(&map, truth, 12.0);

        let sole = [FloorCandidate {
            floor: FloorId(2),
            map: &map,
            expected_pose: truth,
        }];
        let identification =
            reacquire_floor(&sole, &scan, identity(), &ReacquisitionConfig::default())
                .expect("inputs are valid")
                .expect("a lone candidate that matches is the answer");
        assert_eq!(identification.best.floor, FloorId(2));
        assert!(identification.runner_up.is_none());
        assert_eq!(identification.margin, f64::INFINITY);

        // With one candidate the only defence is `min_score`, and mean
        // likelihood is a weak one: a scan taken on a floor partitioned by a
        // full-height wall still scores 0.83 against this unpartitioned map,
        // because the shared outer walls carry most of the beams. A caller who
        // needs a lone candidate checked has to raise the floor deliberately.
        let partition: Vec<(isize, isize)> = (4..36).map(|y| (20isize, y)).collect();
        let elsewhere = floor_map(&partition);
        let foreign_scan = simulated_scan(&elsewhere, truth, 12.0);
        let permissive = reacquire_floor(
            &sole,
            &foreign_scan,
            identity(),
            &ReacquisitionConfig::default(),
        )
        .expect("inputs are valid")
        .expect("the default floor accepts it");
        assert!(
            permissive.best.score < identification.best.score,
            "the foreign scan must at least score worse: {} vs {}",
            permissive.best.score,
            identification.best.score
        );
        let strict = ReacquisitionConfig {
            min_score: 0.9,
            ..ReacquisitionConfig::default()
        };
        assert_eq!(
            reacquire_floor(&sole, &foreign_scan, identity(), &strict).expect("inputs are valid"),
            Err(FloorAmbiguity::NothingMatched)
        );
    }

    #[test]
    fn unusable_inputs_are_rejected_rather_than_scored() {
        let map = floor_map(&[]);
        let truth = Pose2d::new(5.0, 5.0, 0.0);
        let scan = simulated_scan(&map, truth, 12.0);
        let candidates = [FloorCandidate {
            floor: FloorId(0),
            map: &map,
            expected_pose: truth,
        }];

        assert_eq!(
            reacquire_floor(&[], &scan, identity(), &ReacquisitionConfig::default()),
            Err(ReacquisitionError::NoCandidates)
        );
        assert_eq!(
            reacquire_floor(
                &candidates,
                &[],
                identity(),
                &ReacquisitionConfig::default()
            ),
            Err(ReacquisitionError::InvalidScan)
        );
        assert_eq!(
            reacquire_floor(
                &candidates,
                &[Vec3::new(f64::NAN, 0.0, 0.0)],
                identity(),
                &ReacquisitionConfig::default()
            ),
            Err(ReacquisitionError::InvalidScan)
        );
        assert_eq!(
            reacquire_floor(
                &candidates,
                &scan,
                identity(),
                &ReacquisitionConfig {
                    min_margin: 0.0,
                    ..ReacquisitionConfig::default()
                }
            ),
            Err(ReacquisitionError::InvalidConfig)
        );
    }
}
