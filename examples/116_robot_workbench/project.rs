//! Versioned editor data independent of rendering and simulation execution.

use anyhow::{bail, ensure, Context, Result};
use rne_urdf_import::{parse_urdf_document, UrdfDocument, UrdfJointType};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const MAX_PROJECT_BYTES: usize = 4 * 1024 * 1024;
const MAX_OBJECTS: usize = 128;
const MAX_POSES: usize = 128;

/// The supported source format; MJCF retains the importer's strict subset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum RobotFormat {
    Urdf,
    Mjcf,
}

/// Source text is embedded so project files do not depend on an upload filename.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct RobotSource {
    pub format: RobotFormat,
    pub xml: String,
}

impl RobotSource {
    pub fn document(&self) -> Result<UrdfDocument> {
        ensure!(
            self.xml.len() <= MAX_PROJECT_BYTES / 2,
            "robot source too large"
        );
        let xml = match self.format {
            RobotFormat::Urdf => self.xml.clone(),
            RobotFormat::Mjcf => rne_mjcf::mjcf_to_urdf(&self.xml).context("import MJCF")?,
        };
        parse_urdf_document(&xml).context("import URDF")
    }

    pub fn digest(&self) -> String {
        let mut hash = Sha256::new();
        hash.update(match self.format {
            RobotFormat::Urdf => b"urdf".as_slice(),
            RobotFormat::Mjcf => b"mjcf".as_slice(),
        });
        hash.update(self.xml.as_bytes());
        format!("{:x}", hash.finalize())
    }
}

/// Angular and linear values cannot be confused when saving a pose.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(crate) enum JointPosition {
    Revolute { position_rad: f64 },
    Prismatic { position_m: f64 },
}

impl JointPosition {
    pub fn value(self) -> f64 {
        match self {
            Self::Revolute { position_rad } => position_rad,
            Self::Prismatic { position_m } => position_m,
        }
    }
}

/// Model-derived bounds for a controllable joint.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct JointInfo {
    pub name: String,
    pub linear: bool,
    pub lower: f64,
    pub upper: f64,
    pub max_velocity: f64,
    pub max_effort: f64,
    pub limits_from_model: bool,
}

/// An entire named posture, bound to the exact robot source.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SavedPose {
    pub robot_digest: String,
    pub joints: BTreeMap<String, JointPosition>,
}

/// Static floor or obstacle; dimensions and position are in metres.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SceneObject {
    pub name: String,
    pub position_m: [f64; 3],
    pub size_m: [f64; 3],
    pub color_rgba: [f32; 4],
}

/// Self-contained editor project. Mesh files are resolved by the host asset root.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Project {
    pub schema_version: u32,
    pub robot: RobotSource,
    #[serde(default)]
    pub use_declared_inertias: bool,
    #[serde(default)]
    pub robot_rotation_rpy_rad: [f64; 3],
    pub targets: BTreeMap<String, JointPosition>,
    pub poses: BTreeMap<String, SavedPose>,
    pub objects: Vec<SceneObject>,
}

impl Project {
    pub fn new(robot: RobotSource) -> Result<Self> {
        let mut project = Self {
            schema_version: 1,
            robot,
            use_declared_inertias: false,
            robot_rotation_rpy_rad: [0.0; 3],
            targets: BTreeMap::new(),
            poses: BTreeMap::new(),
            objects: vec![SceneObject {
                name: "floor".into(),
                position_m: [0.0, -0.15, 0.0],
                size_m: [6.0, 0.1, 6.0],
                color_rgba: [0.25, 0.3, 0.35, 1.0],
            }],
        };
        for joint in project.joint_catalog()? {
            let value = 0.0_f64.clamp(joint.lower, joint.upper);
            let position = if joint.linear {
                JointPosition::Prismatic { position_m: value }
            } else {
                JointPosition::Revolute {
                    position_rad: value,
                }
            };
            project.targets.insert(joint.name, position);
        }
        project.validate()?;
        Ok(project)
    }

    pub fn save_pose(&mut self, name: &str) -> Result<()> {
        validate_name(name)?;
        let mut candidate = self.clone();
        candidate.poses.insert(
            name.into(),
            SavedPose {
                robot_digest: self.robot.digest(),
                joints: self.targets.clone(),
            },
        );
        candidate.validate()?;
        *self = candidate;
        Ok(())
    }

    pub fn apply_pose(&mut self, pose: &SavedPose) -> Result<()> {
        ensure!(
            pose.robot_digest == self.robot.digest(),
            "pose belongs to another robot"
        );
        let mut candidate = self.clone();
        candidate.targets = pose.joints.clone();
        candidate.validate()?;
        *self = candidate;
        Ok(())
    }

    pub fn from_json(bytes: &[u8]) -> Result<Self> {
        ensure!(bytes.len() <= MAX_PROJECT_BYTES, "project exceeds 4 MiB");
        let project: Self = serde_json::from_slice(bytes).context("read project JSON")?;
        project.validate()?;
        Ok(project)
    }

    pub fn joint_catalog(&self) -> Result<Vec<JointInfo>> {
        let document = self.robot.document()?;
        let mut names = BTreeSet::new();
        let mut catalog = Vec::new();
        for joint in &document.robot.joints {
            ensure!(names.insert(joint.name.clone()), "duplicate joint name");
            if joint.joint_type == UrdfJointType::Fixed || joint.mimic.is_some() {
                continue;
            }
            let linear = joint.joint_type == UrdfJointType::Prismatic;
            let (lower, upper) = if joint.joint_type == UrdfJointType::Continuous {
                (-std::f64::consts::TAU, std::f64::consts::TAU)
            } else {
                let limits = joint.limit.context("bounded joint requires limits")?;
                (limits.lower, limits.upper)
            };
            let (mut max_velocity, mut max_effort) = joint
                .limit
                .map(|limit| (limit.max_velocity_rad_s, limit.max_effort_nm))
                .unwrap_or((1.0, 10.0));
            // The strict MJCF converter does not import actuators and emits
            // zero effort/velocity placeholders. These are explicit workbench
            // servo defaults, not claimed authored actuator specifications.
            let limits_from_model = self.robot.format == RobotFormat::Urdf && joint.limit.is_some();
            if self.robot.format == RobotFormat::Mjcf {
                max_velocity = 1.0;
                max_effort = 10.0;
            }
            ensure!(
                lower.is_finite()
                    && upper.is_finite()
                    && lower <= upper
                    && max_velocity.is_finite()
                    && max_velocity > 0.0
                    && max_effort.is_finite()
                    && max_effort > 0.0,
                "invalid limits for joint {}",
                joint.name
            );
            catalog.push(JointInfo {
                name: joint.name.clone(),
                linear,
                lower,
                upper,
                max_velocity,
                max_effort,
                limits_from_model,
            });
        }
        catalog.sort_by(|a, b| a.name.cmp(&b.name));
        ensure!(
            catalog.len() <= 256,
            "robot exceeds 256 controllable joints"
        );
        Ok(catalog)
    }

    pub fn validate(&self) -> Result<()> {
        ensure!(self.schema_version == 1, "unsupported project schema");
        ensure!(
            self.robot_rotation_rpy_rad
                .iter()
                .all(|v| v.is_finite() && v.abs() <= std::f64::consts::TAU),
            "robot rotation must be finite and within one full turn"
        );
        ensure!(self.objects.len() <= MAX_OBJECTS, "too many scene objects");
        ensure!(self.poses.len() <= MAX_POSES, "too many saved poses");
        let catalog = self.joint_catalog()?;
        validate_targets(&catalog, &self.targets)?;
        let digest = self.robot.digest();
        for (name, pose) in &self.poses {
            validate_name(name)?;
            ensure!(
                pose.robot_digest == digest,
                "pose {name} belongs to another robot"
            );
            validate_targets(&catalog, &pose.joints)?;
        }
        let mut names = BTreeSet::new();
        for object in &self.objects {
            validate_name(&object.name)?;
            ensure!(names.insert(&object.name), "duplicate scene object name");
            ensure!(
                object
                    .position_m
                    .iter()
                    .all(|v| v.is_finite() && v.abs() <= 1000.0),
                "object position must be finite and within 1000 m"
            );
            ensure!(
                object
                    .size_m
                    .iter()
                    .all(|v| v.is_finite() && *v >= 0.001 && *v <= 1000.0),
                "object sizes must be between 0.001 and 1000 m"
            );
            ensure!(
                object
                    .color_rgba
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
                "object color must be in [0, 1]"
            );
        }
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_PROJECT_BYTES,
            "serialized project exceeds 4 MiB"
        );
        Ok(())
    }
}

fn validate_name(name: &str) -> Result<()> {
    ensure!(
        !name.trim().is_empty() && name.len() <= 128 && !name.chars().any(char::is_control),
        "name must contain 1–128 bytes without control characters"
    );
    Ok(())
}

fn validate_targets(
    catalog: &[JointInfo],
    targets: &BTreeMap<String, JointPosition>,
) -> Result<()> {
    ensure!(
        catalog.len() == targets.len(),
        "pose must contain exactly the robot's controllable joints"
    );
    for joint in catalog {
        let position = targets
            .get(&joint.name)
            .with_context(|| format!("missing joint {}", joint.name))?;
        let linear = matches!(position, JointPosition::Prismatic { .. });
        ensure!(
            linear == joint.linear,
            "wrong position unit for {}",
            joint.name
        );
        let value = position.value();
        if !value.is_finite() || value < joint.lower || value > joint.upper {
            bail!(
                "joint {} target is outside [{}, {}]",
                joint.name,
                joint.lower,
                joint.upper
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slider() -> Project {
        Project::new(RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
                .into(),
        })
        .unwrap()
    }

    #[test]
    fn complete_project_roundtrips_with_saved_pose_and_scene() {
        let mut project = slider();
        project.targets.insert(
            "slider_joint".into(),
            JointPosition::Prismatic { position_m: 0.1 },
        );
        project.save_pose("extended").unwrap();
        let encoded = serde_json::to_vec(&project).unwrap();
        assert_eq!(Project::from_json(&encoded).unwrap(), project);
    }

    #[test]
    fn rejected_pose_never_partially_changes_current_targets() {
        let mut project = slider();
        project.save_pose("home").unwrap();
        let before = project.clone();
        let mut pose = project.poses["home"].clone();
        for value in [
            JointPosition::Prismatic { position_m: 0.2 },
            JointPosition::Prismatic {
                position_m: f64::NAN,
            },
            JointPosition::Revolute { position_rad: 0.1 },
        ] {
            pose.joints.insert("slider_joint".into(), value);
            assert!(project.apply_pose(&pose).is_err());
            assert_eq!(project, before);
        }
        pose.joints.clear();
        assert!(project.apply_pose(&pose).is_err());
        assert_eq!(project, before);
    }

    #[test]
    fn saved_pose_is_bound_to_robot_and_names() {
        let mut project = slider();
        project.save_pose("home").unwrap();
        let mut pose = project.poses["home"].clone();
        pose.robot_digest = "different robot".into();
        assert!(project.apply_pose(&pose).is_err());
        assert!(project.save_pose("\n").is_err());
        let mut other = project.clone();
        other.robot.xml = other
            .robot
            .xml
            .replace("prismatic_slider", "another_slider");
        assert!(other.validate().is_err());
    }

    #[test]
    fn malformed_scene_and_future_schema_are_rejected() {
        let project = slider();
        let mut bad = project.clone();
        bad.objects.push(bad.objects[0].clone());
        assert!(bad.validate().is_err());
        let mut bad = project.clone();
        bad.objects[0].size_m[1] = -0.1;
        assert!(bad.validate().is_err());
        let mut bad = project.clone();
        bad.schema_version = 2;
        assert!(Project::from_json(&serde_json::to_vec(&bad).unwrap()).is_err());
    }

    #[test]
    fn supported_mjcf_uses_same_named_joint_catalog() {
        let project = Project::new(RobotSource {
            format: RobotFormat::Mjcf,
            xml: include_str!("../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").into(),
        })
        .unwrap();
        let names = project
            .joint_catalog()
            .unwrap()
            .into_iter()
            .map(|j| j.name)
            .collect::<Vec<_>>();
        assert_eq!(names, ["elbow", "shoulder"]);
        assert!(project
            .targets
            .values()
            .all(|p| matches!(p, JointPosition::Revolute { .. })));
    }
}
