//! Planning groups: named subsets of a robot's degrees of freedom.
//!
//! A planning group is the RNE analogue of an SRDF planning group. It names a
//! chain (or an explicit joint set) so that planning, inverse kinematics, and
//! sampling can be scoped to those joints while the rest of the robot holds its
//! start configuration.

use crate::error::PlanningError;
use rne_ecs::Entity;
use rne_robot::KinematicModel;

/// A named subset of a robot's degrees of freedom.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanningGroup {
    name: String,
    dof_indices: Vec<usize>,
    base_link: Entity,
    tip_link: Entity,
}

impl PlanningGroup {
    /// Builds a group from the kinematic chain between `base_link` and
    /// `tip_link`.
    ///
    /// Fixed joints on the chain are ignored; mimic joints are ignored because
    /// they have no independent degree of freedom.
    pub fn chain(
        model: &KinematicModel,
        name: impl Into<String>,
        base_link: Entity,
        tip_link: Entity,
    ) -> Result<Self, PlanningError> {
        let joints = model.chain_joints(base_link, tip_link)?;
        let mut dof_indices = Vec::new();
        for joint in joints {
            if let Some(dof) = model.dof_index_of_joint(joint) {
                dof_indices.push(dof);
            }
        }
        Self::from_dof_indices(name.into(), dof_indices, base_link, tip_link)
    }

    /// Builds a group from an explicit set of independent joint entities.
    pub fn joints(
        model: &KinematicModel,
        name: impl Into<String>,
        joints: &[Entity],
        base_link: Entity,
        tip_link: Entity,
    ) -> Result<Self, PlanningError> {
        let mut dof_indices = Vec::new();
        for joint in joints {
            let dof = model.dof_index_of_joint(*joint).ok_or_else(|| {
                PlanningError::Invalid(format!(
                    "joint {joint:?} is not an independent degree of freedom"
                ))
            })?;
            dof_indices.push(dof);
        }
        dof_indices.sort_unstable();
        dof_indices.dedup();
        Self::from_dof_indices(name.into(), dof_indices, base_link, tip_link)
    }

    fn from_dof_indices(
        name: String,
        dof_indices: Vec<usize>,
        base_link: Entity,
        tip_link: Entity,
    ) -> Result<Self, PlanningError> {
        if dof_indices.is_empty() {
            return Err(PlanningError::Invalid(
                "planning group has no degrees of freedom".to_string(),
            ));
        }
        Ok(Self {
            name,
            dof_indices,
            base_link,
            tip_link,
        })
    }

    /// Group name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Degree-of-freedom indices in model order.
    pub fn dof_indices(&self) -> &[usize] {
        &self.dof_indices
    }

    /// Number of degrees of freedom in the group.
    pub fn dof(&self) -> usize {
        self.dof_indices.len()
    }

    /// Chain base link.
    pub fn base_link(&self) -> Entity {
        self.base_link
    }

    /// Chain tip link.
    pub fn tip_link(&self) -> Entity {
        self.tip_link
    }

    /// Extracts the group's values from a full configuration.
    pub fn project(&self, full: &[f64]) -> Vec<f64> {
        self.dof_indices.iter().map(|&index| full[index]).collect()
    }

    /// Writes group values into a full configuration, preserving other joints.
    pub fn embed(&self, full: &[f64], group: &[f64]) -> Vec<f64> {
        let mut embedded = full.to_vec();
        for (slot, &index) in self.dof_indices.iter().enumerate() {
            if let Some(value) = group.get(slot) {
                embedded[index] = *value;
            }
        }
        embedded
    }

    /// Active degree-of-freedom mask over `dof` model joints.
    pub fn active_mask(&self, dof: usize) -> Vec<bool> {
        let mut mask = vec![false; dof];
        for &index in &self.dof_indices {
            if index < dof {
                mask[index] = true;
            }
        }
        mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::PlanningScene;
    use crate::test_support::arm_world;

    #[test]
    fn chain_group_projects_and_embeds() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        let group =
            PlanningGroup::chain(scene.model(), "arm", scene.model().base_link(), tool).unwrap();
        assert_eq!(group.dof(), 2);
        assert_eq!(group.dof_indices(), &[0, 1]);
        assert_eq!(group.project(&[0.1, 0.2, 0.3]), vec![0.1, 0.2]);
        assert_eq!(
            group.embed(&[0.1, 0.2, 0.3], &[0.7, 0.8]),
            vec![0.7, 0.8, 0.3]
        );
        assert_eq!(group.active_mask(3), vec![true, true, false]);
    }

    #[test]
    fn chain_group_rejects_wrong_direction() {
        let (world, robot, tool) = arm_world();
        let scene = PlanningScene::from_world(&world, robot).unwrap();
        assert!(
            PlanningGroup::chain(scene.model(), "bad", tool, scene.model().base_link()).is_err()
        );
    }
}
