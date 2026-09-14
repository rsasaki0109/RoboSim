//! Goal-constraint sampling.
//!
//! This is the RNE analogue of MoveIt's `ConstraintSampler`: it draws a
//! collision-free configuration that satisfies a [`GoalConstraint`], using
//! seeded random-restart inverse kinematics for pose, position, and orientation
//! goals.

use crate::constraints::GoalConstraint;
use crate::error::PlanningError;
use crate::group::PlanningGroup;
use crate::request::PlanningOptions;
use crate::scene::PlanningScene;
use rne_math::{Pose3, Quat, Vec3};
use rne_robot::{IkRequest, DAMPED_LEAST_SQUARES_SOLVER};

/// Samples configurations that satisfy a goal constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct ConstraintSampler {
    goal: GoalConstraint,
    attempts: usize,
    seed: u64,
}

impl ConstraintSampler {
    /// Creates a sampler for `goal` with a seeded restart budget.
    pub fn new(goal: GoalConstraint, attempts: usize, seed: u64) -> Self {
        Self {
            goal,
            attempts,
            seed,
        }
    }

    /// The sampled goal constraint.
    pub fn goal(&self) -> &GoalConstraint {
        &self.goal
    }

    /// Restart budget.
    pub fn attempts(&self) -> usize {
        self.attempts
    }

    /// Sampler seed.
    pub fn seed(&self) -> u64 {
        self.seed
    }

    /// Draws a collision-free configuration satisfying the goal.
    ///
    /// Returns `Ok(None)` when the constraint could not be satisfied within the
    /// restart budget or every solution collides.
    pub fn sample(
        &self,
        scene: &PlanningScene,
        base: &[f64],
        group: Option<&PlanningGroup>,
        options: &PlanningOptions,
    ) -> Result<Option<Vec<f64>>, PlanningError> {
        let dof = scene.model().dof();
        if base.len() != dof {
            return Err(PlanningError::JointCountMismatch {
                provided: base.len(),
                expected: dof,
            });
        }

        if let GoalConstraint::Joint { positions } = &self.goal {
            let candidate = if positions.len() == dof {
                positions.clone()
            } else if let Some(group) = group {
                if positions.len() == group.dof() {
                    group.embed(base, positions)
                } else {
                    return Err(PlanningError::JointCountMismatch {
                        provided: positions.len(),
                        expected: group.dof(),
                    });
                }
            } else {
                return Err(PlanningError::JointCountMismatch {
                    provided: positions.len(),
                    expected: dof,
                });
            };
            return Ok(scene.is_state_valid(&candidate)?.then_some(candidate));
        }

        let (end_link, target, solve_position, solve_orientation) = match &self.goal {
            GoalConstraint::Pose { end_link, target } => (*end_link, *target, true, true),
            GoalConstraint::Position { end_link, target } => (
                *end_link,
                Pose3 {
                    translation: *target,
                    rotation: Quat::IDENTITY,
                },
                true,
                false,
            ),
            GoalConstraint::Orientation { end_link, target } => (
                *end_link,
                Pose3 {
                    translation: Vec3::ZERO,
                    rotation: *target,
                },
                false,
                true,
            ),
            GoalConstraint::Joint { .. } => unreachable!("joint goals handled above"),
        };

        let solver_name = options
            .solver
            .as_deref()
            .unwrap_or(DAMPED_LEAST_SQUARES_SOLVER);
        let solver = scene
            .solvers()
            .get(solver_name)
            .ok_or_else(|| PlanningError::UnknownSolver(solver_name.to_string()))?;
        let mut ik_options = options.ik_options;
        ik_options.solve_position = solve_position;
        ik_options.solve_orientation = solve_orientation;
        let mut request = IkRequest::new(end_link, target, base.to_vec(), ik_options);
        if let Some(group) = group {
            request = request.with_active_dof(group.active_mask(dof));
        }

        match solver.search_position_ik(scene.model(), &request, self.attempts, self.seed) {
            Ok(solution) if scene.is_state_valid(&solution.joint_positions)? => {
                Ok(Some(solution.joint_positions))
            }
            Ok(_) | Err(_) => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::arm_world;

    #[test]
    fn samples_a_valid_joint_goal() {
        let (world, robot, _tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let sampler = ConstraintSampler::new(GoalConstraint::joint(vec![0.3, 0.2]), 0, 0);
        let sample = sampler
            .sample(&scene, &[0.0, 0.0], None, &PlanningOptions::default())
            .unwrap();
        assert_eq!(sample, Some(vec![0.3, 0.2]));
    }

    #[test]
    fn samples_a_position_goal_with_restarts() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let target = scene
            .model()
            .forward_kinematics(&[0.8, 0.2])
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        let sampler = ConstraintSampler::new(GoalConstraint::position(tool, target), 50, 11);
        let sample = sampler
            .sample(&scene, &[0.3, 0.2], None, &PlanningOptions::default())
            .unwrap()
            .expect("sampled configuration");
        let reached = scene
            .model()
            .forward_kinematics(&sample)
            .unwrap()
            .link_transform(tool)
            .unwrap()
            .translation;
        assert!((reached - target).length() < 1.0e-3);
    }
}
