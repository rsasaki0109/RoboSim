//! Minimal SRDF import for planning groups.
//!
//! This parses the `<group>` elements of an SRDF document, the same source
//! `MoveIt` uses for planning groups, and applies them to a [`PlanningScene`].
//! Only chain and explicit-joint groups are read; other SRDF elements are
//! ignored so a full `MoveIt` SRDF can be passed unchanged.

use crate::error::PlanningError;
use crate::group::PlanningGroup;
use crate::scene::PlanningScene;
use roxmltree::Document;
use thiserror::Error;

/// A planning group parsed from SRDF.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SrdfGroup {
    /// A kinematic chain between two links.
    Chain {
        /// Group name.
        name: String,
        /// Base link name.
        base_link: String,
        /// Tip link name.
        tip_link: String,
    },
    /// An explicit set of joints.
    Joints {
        /// Group name.
        name: String,
        /// Joint names.
        joints: Vec<String>,
    },
}

impl SrdfGroup {
    /// Group name.
    pub fn name(&self) -> &str {
        match self {
            SrdfGroup::Chain { name, .. } | SrdfGroup::Joints { name, .. } => name,
        }
    }
}

/// Parsed SRDF planning groups.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SrdfDocument {
    groups: Vec<SrdfGroup>,
}

impl SrdfDocument {
    /// Parsed groups in document order.
    pub fn groups(&self) -> &[SrdfGroup] {
        &self.groups
    }

    /// Whether the document declared no groups.
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }
}

/// Error returned while parsing an SRDF document.
#[derive(Debug, Error)]
pub enum SrdfError {
    /// The XML could not be parsed.
    #[error("invalid SRDF XML: {0}")]
    Xml(String),
    /// A required attribute was missing.
    #[error("SRDF element is missing attribute {0}")]
    MissingAttribute(&'static str),
    /// A group declared neither a chain nor any joint.
    #[error("SRDF group {0:?} has no chain or joint")]
    EmptyGroup(String),
}

/// Parses the planning groups of an SRDF document.
pub fn parse_srdf(xml: &str) -> Result<SrdfDocument, SrdfError> {
    let document = Document::parse(xml).map_err(|error| SrdfError::Xml(error.to_string()))?;
    let robot = document.root_element();
    let mut groups = Vec::new();
    for group in robot
        .children()
        .filter(|node| node.is_element() && node.tag_name().name() == "group")
    {
        let name = group
            .attribute("name")
            .ok_or(SrdfError::MissingAttribute("group@name"))?
            .to_string();
        if let Some(chain) = group
            .children()
            .find(|node| node.is_element() && node.tag_name().name() == "chain")
        {
            let base_link = chain
                .attribute("base_link")
                .ok_or(SrdfError::MissingAttribute("chain@base_link"))?
                .to_string();
            let tip_link = chain
                .attribute("tip_link")
                .ok_or(SrdfError::MissingAttribute("chain@tip_link"))?
                .to_string();
            groups.push(SrdfGroup::Chain {
                name,
                base_link,
                tip_link,
            });
            continue;
        }
        let joints: Vec<String> = group
            .children()
            .filter(|node| node.is_element() && node.tag_name().name() == "joint")
            .filter_map(|node| node.attribute("name").map(str::to_string))
            .collect();
        if joints.is_empty() {
            return Err(SrdfError::EmptyGroup(name));
        }
        groups.push(SrdfGroup::Joints { name, joints });
    }
    Ok(SrdfDocument { groups })
}

impl PlanningScene {
    /// Applies SRDF planning groups to the scene.
    ///
    /// Chain groups resolve their base and tip links; joint groups resolve each
    /// named independent joint. Returns the number of groups added. Unknown
    /// links or joints return [`PlanningError::Invalid`].
    pub fn apply_srdf(&mut self, document: &SrdfDocument) -> Result<usize, PlanningError> {
        let mut groups = Vec::with_capacity(document.groups().len());
        for group in document.groups() {
            let planning_group = match group {
                SrdfGroup::Chain {
                    name,
                    base_link,
                    tip_link,
                } => {
                    let base = self.model().link_entity_by_name(base_link).ok_or_else(|| {
                        PlanningError::Invalid(format!("SRDF unknown link {base_link:?}"))
                    })?;
                    let tip = self.model().link_entity_by_name(tip_link).ok_or_else(|| {
                        PlanningError::Invalid(format!("SRDF unknown link {tip_link:?}"))
                    })?;
                    PlanningGroup::chain(self.model(), name.clone(), base, tip)?
                }
                SrdfGroup::Joints { name, joints } => {
                    let mut entities = Vec::new();
                    for joint in joints {
                        let entity = self.model().joint_entity_by_name(joint).ok_or_else(|| {
                            PlanningError::Invalid(format!("SRDF unknown joint {joint:?}"))
                        })?;
                        if self.model().dof_index_of_joint(entity).is_some() {
                            entities.push(entity);
                        }
                    }
                    if entities.is_empty() {
                        return Err(PlanningError::Invalid(format!(
                            "SRDF group {name:?} has no independent joints"
                        )));
                    }
                    let base = self
                        .model()
                        .joint_parent_link(entities[0])
                        .unwrap_or_else(|| self.model().base_link());
                    let tip = self
                        .model()
                        .joint_child_link(*entities.last().expect("non-empty"))
                        .unwrap_or_else(|| self.model().base_link());
                    PlanningGroup::joints(self.model(), name.clone(), &entities, base, tip)?
                }
            };
            groups.push(planning_group);
        }
        let count = groups.len();
        for group in groups {
            self.add_group(group);
        }
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::arm_world;

    const DOC: &str = r#"
    <robot name="arm">
      <group name="arm">
        <chain base_link="base" tip_link="tool"/>
      </group>
      <group name="shoulder">
        <joint name="joint1"/>
      </group>
    </robot>
    "#;

    #[test]
    fn parses_chain_and_joint_groups() {
        let document = parse_srdf(DOC).unwrap();
        assert_eq!(document.groups().len(), 2);
        assert_eq!(
            document.groups()[0],
            SrdfGroup::Chain {
                name: "arm".into(),
                base_link: "base".into(),
                tip_link: "tool".into(),
            }
        );
        assert_eq!(
            document.groups()[1],
            SrdfGroup::Joints {
                name: "shoulder".into(),
                joints: vec!["joint1".into()],
            }
        );
    }

    #[test]
    fn rejects_a_group_without_chain_or_joints() {
        let result =
            parse_srdf(r#"<robot name="r"><group name="g"><link name="base"/></group></robot>"#);
        assert!(matches!(result, Err(SrdfError::EmptyGroup(name)) if name == "g"));
    }

    #[test]
    fn applies_groups_to_a_scene() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        let document = parse_srdf(DOC).unwrap();
        assert_eq!(scene.apply_srdf(&document).unwrap(), 2);
        assert_eq!(scene.group("arm").unwrap().dof(), 2);
        assert_eq!(scene.group("shoulder").unwrap().dof(), 1);
    }

    #[test]
    fn reports_unknown_names() {
        let (world, robot, _tool) = arm_world();
        let mut scene = PlanningScene::from_world(&world, robot).unwrap();
        let document = parse_srdf(
            r#"<robot name="arm"><group name="bad"><chain base_link="base" tip_link="missing"/></group></robot>"#,
        )
        .unwrap();
        assert!(matches!(
            scene.apply_srdf(&document),
            Err(PlanningError::Invalid(_))
        ));
    }
}
