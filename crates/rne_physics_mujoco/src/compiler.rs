//! Deterministic ECS-to-MJCF compilation kept private to the MuJoCo adapter.

use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, FixedJointDesc, JointActuation, PhysicsWorldDesc, PrismaticJointDesc,
    RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_world::Transform3;
use std::fmt::Write as _;
use thiserror::Error;

const QUATERNION_TOLERANCE: f64 = 1.0e-9;

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CompiledRigidBodyModel {
    pub(crate) mjcf: String,
    pub(crate) bindings: Vec<BodyBinding>,
    pub(crate) topology: Vec<BodyTopology>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct BodyBinding {
    pub(crate) entity: Entity,
    pub(crate) body_name: String,
    pub(crate) joint: JointBinding,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum JointBinding {
    Free {
        joint_name: String,
    },
    Revolute {
        joint_name: String,
        actuator_name: String,
    },
    Prismatic {
        joint_name: String,
        actuator_name: String,
    },
    Fixed,
}

impl JointBinding {
    pub(crate) fn joint_name(&self) -> Option<&str> {
        match self {
            Self::Free { joint_name }
            | Self::Revolute { joint_name, .. }
            | Self::Prismatic { joint_name, .. } => Some(joint_name),
            Self::Fixed => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BodyTopology {
    entity: Entity,
    body_type: RigidBodyType,
    mass_kg: f64,
    collider: Collider,
    structural_transform: Option<Transform3>,
    joint: Option<JointSpec>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum JointSpec {
    Revolute(RevoluteJointDesc),
    Prismatic(PrismaticJointDesc),
    Fixed(FixedJointDesc),
}

impl JointSpec {
    const fn parent(self) -> Entity {
        match self {
            Self::Revolute(desc) => desc.parent,
            Self::Prismatic(desc) => desc.parent,
            Self::Fixed(desc) => desc.parent,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct BodyInput {
    entity: Entity,
    rigid_body: RigidBody,
    collider: Collider,
    transform: Transform3,
    joint: Option<JointSpec>,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub(crate) enum CompileError {
    #[error("MuJoCo rigid-body world is empty")]
    EmptyWorld,
    #[error("entity {entity_index} has a collider but no rigid body")]
    ColliderWithoutRigidBody { entity_index: u32 },
    #[error("entity {entity_index} has a rigid body but no collider")]
    MissingCollider { entity_index: u32 },
    #[error("entity {entity_index} has a rigid body but no Transform3")]
    MissingTransform { entity_index: u32 },
    #[error("entity {entity_index} uses unsupported kinematic motion")]
    UnsupportedKinematicBody { entity_index: u32 },
    #[error("entity {entity_index} has multiple joint descriptions")]
    MultipleJointDescriptions { entity_index: u32 },
    #[error("entity {entity_index} references missing parent {parent_index}")]
    MissingJointParent {
        entity_index: u32,
        parent_index: u32,
    },
    #[error("joint graph contains a cycle at entity {entity_index}")]
    JointCycle { entity_index: u32 },
    #[error("invalid joint actuation on entity {entity_index}: {reason}")]
    InvalidActuation {
        entity_index: u32,
        reason: &'static str,
    },
    #[error("entity {entity_index} has invalid {field}")]
    InvalidValue {
        entity_index: u32,
        field: &'static str,
    },
}

pub(crate) fn compile_rigid_body_model(
    world: &World,
    desc: PhysicsWorldDesc,
    timestep_s: f64,
) -> Result<CompiledRigidBodyModel, CompileError> {
    let mut bodies = collect_bodies(world)?;
    if bodies.is_empty() {
        return Err(CompileError::EmptyWorld);
    }
    bodies.sort_unstable_by_key(|body| body.entity.index());
    validate_joint_graph(&bodies, world)?;

    let mut mjcf =
        String::from("<mujoco model=\"rne-ecs-dynamics\">\n  <compiler angle=\"radian\"/>\n");
    write!(
        mjcf,
        "  <option timestep=\"{timestep_s:.9}\" gravity=\"{}\" integrator=\"Euler\"",
        vector(desc.gravity_m_s2)
    )
    .expect("writing to String cannot fail");
    if desc.solver_iterations > 0 {
        write!(mjcf, " iterations=\"{}\"", desc.solver_iterations)
            .expect("writing to String cannot fail");
    }
    mjcf.push_str("/>\n  <worldbody>\n");

    let mut bindings = Vec::with_capacity(bodies.len());
    let mut topology = Vec::with_capacity(bodies.len());
    let mut actuators = Vec::new();
    for body in bodies.iter().filter(|body| body.joint.is_none()) {
        write_body(
            &mut mjcf,
            &bodies,
            *body,
            None,
            2,
            &mut bindings,
            &mut topology,
            &mut actuators,
        );
    }
    mjcf.push_str("  </worldbody>\n");
    if !actuators.is_empty() {
        mjcf.push_str("  <actuator>\n");
        for actuator in actuators {
            writeln!(mjcf, "    {actuator}").expect("writing to String cannot fail");
        }
        mjcf.push_str("  </actuator>\n");
    }
    mjcf.push_str("</mujoco>\n");
    Ok(CompiledRigidBodyModel {
        mjcf,
        bindings,
        topology,
    })
}

fn collect_bodies(world: &World) -> Result<Vec<BodyInput>, CompileError> {
    let mut bodies = Vec::new();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        let rigid_body = entity_ref.get::<RigidBody>().copied();
        let collider = entity_ref.get::<Collider>().copied();
        match (rigid_body, collider) {
            (None, Some(_)) => {
                return Err(CompileError::ColliderWithoutRigidBody {
                    entity_index: entity.index(),
                });
            }
            (Some(_), None) => {
                return Err(CompileError::MissingCollider {
                    entity_index: entity.index(),
                });
            }
            (None, None) => continue,
            (Some(rigid_body), Some(collider)) => {
                let transform = entity_ref.get::<Transform3>().copied().ok_or(
                    CompileError::MissingTransform {
                        entity_index: entity.index(),
                    },
                )?;
                validate_body(entity, rigid_body, collider, transform)?;
                let joints = [
                    entity_ref
                        .get::<RevoluteJointDesc>()
                        .copied()
                        .map(JointSpec::Revolute),
                    entity_ref
                        .get::<PrismaticJointDesc>()
                        .copied()
                        .map(JointSpec::Prismatic),
                    entity_ref
                        .get::<FixedJointDesc>()
                        .copied()
                        .map(JointSpec::Fixed),
                ];
                let joint_count = joints.iter().filter(|joint| joint.is_some()).count();
                if joint_count > 1 {
                    return Err(CompileError::MultipleJointDescriptions {
                        entity_index: entity.index(),
                    });
                }
                bodies.push(BodyInput {
                    entity,
                    rigid_body,
                    collider,
                    transform,
                    joint: joints.into_iter().flatten().next(),
                });
            }
        }
    }
    Ok(bodies)
}

fn validate_joint_graph(bodies: &[BodyInput], world: &World) -> Result<(), CompileError> {
    for body in bodies {
        let Some(joint) = body.joint else {
            if world.get::<JointActuation>(body.entity).is_some() {
                return Err(CompileError::InvalidActuation {
                    entity_index: body.entity.index(),
                    reason: "actuation without joint",
                });
            }
            continue;
        };
        let parent = joint.parent();
        let Some(_) = bodies.iter().find(|candidate| candidate.entity == parent) else {
            return Err(CompileError::MissingJointParent {
                entity_index: body.entity.index(),
                parent_index: parent.index(),
            });
        };
        if body.rigid_body.body_type != RigidBodyType::Dynamic {
            return Err(invalid(body.entity.index(), "joint child body_type"));
        }
        validate_joint(body, joint)?;
        validate_actuation(world, body.entity, joint)?;

        let mut cursor = parent;
        for _ in 0..bodies.len() {
            if cursor == body.entity {
                return Err(CompileError::JointCycle {
                    entity_index: body.entity.index(),
                });
            }
            let Some(ancestor) = bodies.iter().find(|candidate| candidate.entity == cursor) else {
                break;
            };
            let Some(ancestor_joint) = ancestor.joint else {
                break;
            };
            cursor = ancestor_joint.parent();
        }
    }
    Ok(())
}

fn validate_joint(child: &BodyInput, joint: JointSpec) -> Result<(), CompileError> {
    let index = child.entity.index();
    match joint {
        JointSpec::Revolute(desc) => {
            validate_axis_and_anchors(index, desc.axis, desc.anchor_parent_m, desc.anchor_child_m)?;
            validate_limits(index, "revolute limits", desc.lower_rad, desc.upper_rad)
        }
        JointSpec::Prismatic(desc) => {
            validate_axis_and_anchors(index, desc.axis, desc.anchor_parent_m, desc.anchor_child_m)?;
            validate_limits(index, "prismatic limits", desc.lower_m, desc.upper_m)
        }
        JointSpec::Fixed(desc) => {
            validate_vec3(index, "fixed anchor_parent_m", desc.anchor_parent_m)?;
            validate_vec3(index, "fixed anchor_child_m", desc.anchor_child_m)?;
            validate_quat(index, "fixed relative_rotation", desc.relative_rotation)
        }
    }
}

fn validate_axis_and_anchors(
    index: u32,
    axis: Vec3,
    anchor_parent_m: Vec3,
    anchor_child_m: Vec3,
) -> Result<(), CompileError> {
    validate_vec3(index, "joint axis", axis)?;
    validate_vec3(index, "anchor_parent_m", anchor_parent_m)?;
    validate_vec3(index, "anchor_child_m", anchor_child_m)?;
    if axis.length_squared() <= 1.0e-18 {
        return Err(invalid(index, "joint axis"));
    }
    Ok(())
}

fn validate_limits(
    index: u32,
    field: &'static str,
    lower: Option<f64>,
    upper: Option<f64>,
) -> Result<(), CompileError> {
    match (lower, upper) {
        (None, None) => Ok(()),
        (Some(lower), Some(upper)) if lower.is_finite() && upper.is_finite() && lower <= upper => {
            Ok(())
        }
        _ => Err(invalid(index, field)),
    }
}

fn validate_actuation(world: &World, entity: Entity, joint: JointSpec) -> Result<(), CompileError> {
    let Some(command) = world.get::<JointActuation>(entity).copied() else {
        return Ok(());
    };
    let supported = match joint {
        JointSpec::Revolute(_) => command.supports_revolute(),
        JointSpec::Prismatic(_) => command.supports_prismatic(),
        JointSpec::Fixed(_) => command == JointActuation::Disabled,
    };
    if command.has_valid_values() && supported {
        Ok(())
    } else {
        Err(CompileError::InvalidActuation {
            entity_index: entity.index(),
            reason: "mode, value, gain, or limit",
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn write_body(
    output: &mut String,
    bodies: &[BodyInput],
    body: BodyInput,
    parent: Option<BodyInput>,
    depth: usize,
    bindings: &mut Vec<BodyBinding>,
    topology: &mut Vec<BodyTopology>,
    actuators: &mut Vec<String>,
) {
    let indent = "  ".repeat(depth);
    let index = body.entity.index();
    let body_name = format!("rne_body_{index}");
    let structural_transform = parent.map_or_else(
        || {
            if body.rigid_body.body_type == RigidBodyType::Fixed {
                body.transform
            } else {
                Transform3::IDENTITY
            }
        },
        |parent| relative_transform(parent.transform, body.transform),
    );
    writeln!(
        output,
        "{indent}<body name=\"{body_name}\" pos=\"{}\" quat=\"{}\">",
        vector(structural_transform.translation),
        quaternion(structural_transform.rotation)
    )
    .expect("writing to String cannot fail");

    let joint = write_joint(output, body, structural_transform, depth + 1, actuators);
    write_geom(output, index, body.rigid_body, body.collider, depth + 1);
    bindings.push(BodyBinding {
        entity: body.entity,
        body_name,
        joint,
    });
    topology.push(BodyTopology {
        entity: body.entity,
        body_type: body.rigid_body.body_type,
        mass_kg: body.rigid_body.mass_kg,
        collider: body.collider,
        structural_transform: (body.rigid_body.body_type == RigidBodyType::Fixed
            || matches!(body.joint, Some(JointSpec::Fixed(_))))
        .then_some(structural_transform),
        joint: body.joint,
    });

    for child in bodies.iter().filter(|candidate| {
        candidate
            .joint
            .is_some_and(|joint| joint.parent() == body.entity)
    }) {
        write_body(
            output,
            bodies,
            *child,
            Some(body),
            depth + 1,
            bindings,
            topology,
            actuators,
        );
    }
    writeln!(output, "{indent}</body>").expect("writing to String cannot fail");
}

fn write_joint(
    output: &mut String,
    body: BodyInput,
    relative: Transform3,
    depth: usize,
    actuators: &mut Vec<String>,
) -> JointBinding {
    let indent = "  ".repeat(depth);
    let index = body.entity.index();
    let joint_name = format!("rne_joint_{index}");
    let actuator_name = format!("rne_actuator_{index}");
    match body.joint {
        None if body.rigid_body.body_type == RigidBodyType::Dynamic => {
            writeln!(output, "{indent}<freejoint name=\"{joint_name}\"/>")
                .expect("writing to String cannot fail");
            JointBinding::Free { joint_name }
        }
        None | Some(JointSpec::Fixed(_)) => JointBinding::Fixed,
        Some(JointSpec::Revolute(desc)) => {
            let axis_child = relative.rotation.conjugate() * desc.axis.normalize();
            write!(
                output,
                "{indent}<joint name=\"{joint_name}\" type=\"hinge\" pos=\"{}\" axis=\"{}\"",
                vector(desc.anchor_child_m),
                vector(axis_child)
            )
            .expect("writing to String cannot fail");
            write_range(output, desc.lower_rad, desc.upper_rad);
            output.push_str("/>\n");
            actuators.push(format!(
                "<motor name=\"{actuator_name}\" joint=\"{joint_name}\" gear=\"1\"/>"
            ));
            JointBinding::Revolute {
                joint_name,
                actuator_name,
            }
        }
        Some(JointSpec::Prismatic(desc)) => {
            let axis_child = relative.rotation.conjugate() * desc.axis.normalize();
            write!(
                output,
                "{indent}<joint name=\"{joint_name}\" type=\"slide\" pos=\"{}\" axis=\"{}\"",
                vector(desc.anchor_child_m),
                vector(axis_child)
            )
            .expect("writing to String cannot fail");
            write_range(output, desc.lower_m, desc.upper_m);
            output.push_str("/>\n");
            actuators.push(format!(
                "<motor name=\"{actuator_name}\" joint=\"{joint_name}\" gear=\"1\"/>"
            ));
            JointBinding::Prismatic {
                joint_name,
                actuator_name,
            }
        }
    }
}

fn write_range(output: &mut String, lower: Option<f64>, upper: Option<f64>) {
    if let (Some(lower), Some(upper)) = (lower, upper) {
        write!(
            output,
            " limited=\"true\" range=\"{lower:.17} {upper:.17}\""
        )
        .expect("writing to String cannot fail");
    }
}

fn relative_transform(parent: Transform3, child: Transform3) -> Transform3 {
    let inverse_rotation = parent.rotation.conjugate();
    Transform3::from_translation_rotation(
        inverse_rotation * (child.translation - parent.translation),
        (inverse_rotation * child.rotation).normalize(),
    )
}

fn validate_body(
    entity: Entity,
    rigid_body: RigidBody,
    collider: Collider,
    transform: Transform3,
) -> Result<(), CompileError> {
    let entity_index = entity.index();
    if rigid_body.body_type == RigidBodyType::Kinematic {
        return Err(CompileError::UnsupportedKinematicBody { entity_index });
    }
    if !rigid_body.mass_kg.is_finite() || rigid_body.mass_kg <= 0.0 {
        return Err(invalid(entity_index, "mass_kg"));
    }
    validate_vec3(entity_index, "translation_m", transform.translation)?;
    validate_quat(entity_index, "rotation", transform.rotation)?;
    validate_unit_scale(entity_index, "body scale", transform.scale)?;
    validate_vec3(
        entity_index,
        "linear_velocity_m_s",
        rigid_body.linear_velocity_m_s,
    )?;
    validate_vec3(
        entity_index,
        "angular_velocity_rad_s",
        rigid_body.angular_velocity_rad_s,
    )?;
    validate_vec3(
        entity_index,
        "collider local translation_m",
        collider.local_offset.translation,
    )?;
    validate_quat(
        entity_index,
        "collider local rotation",
        collider.local_offset.rotation,
    )?;
    validate_unit_scale(
        entity_index,
        "collider local scale",
        collider.local_offset.scale,
    )?;
    if !collider.material.friction.is_finite() || collider.material.friction < 0.0 {
        return Err(invalid(entity_index, "friction"));
    }
    if !collider.material.restitution.is_finite()
        || !(0.0..=1.0).contains(&collider.material.restitution)
    {
        return Err(invalid(entity_index, "restitution"));
    }
    match collider.shape {
        ColliderShape::Sphere { radius_m } => validate_positive(entity_index, "radius_m", radius_m),
        ColliderShape::Cuboid { half_extents_m } => {
            validate_vec3(entity_index, "half_extents_m", half_extents_m)?;
            if half_extents_m.min_element() <= 0.0 {
                Err(invalid(entity_index, "half_extents_m"))
            } else {
                Ok(())
            }
        }
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => {
            validate_positive(entity_index, "half_height_m", half_height_m)?;
            validate_positive(entity_index, "radius_m", radius_m)
        }
        ColliderShape::Plane { normal } => {
            validate_vec3(entity_index, "plane normal", normal)?;
            if rigid_body.body_type != RigidBodyType::Fixed || normal.length_squared() <= 1.0e-18 {
                Err(invalid(entity_index, "fixed plane normal"))
            } else {
                Ok(())
            }
        }
    }
}

fn write_geom(
    output: &mut String,
    index: u32,
    rigid_body: RigidBody,
    collider: Collider,
    depth: usize,
) {
    let (kind, size, alignment) = match collider.shape {
        ColliderShape::Sphere { radius_m } => ("sphere", format!("{radius_m:.17}"), Quat::IDENTITY),
        ColliderShape::Cuboid { half_extents_m } => ("box", vector(half_extents_m), Quat::IDENTITY),
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => (
            "capsule",
            format!("{radius_m:.17} {half_height_m:.17}"),
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        ),
        ColliderShape::Plane { normal } => (
            "plane",
            "1 1 0.1".to_string(),
            Quat::from_rotation_arc(Vec3::Z, normal.normalize()),
        ),
    };
    let rotation = collider.local_offset.rotation * alignment;
    let indent = "  ".repeat(depth);
    let sensor_attributes = if collider.sensor {
        " contype=\"0\" conaffinity=\"0\""
    } else {
        ""
    };
    writeln!(
        output,
        "{indent}<geom name=\"rne_geom_{index}\" type=\"{kind}\" size=\"{size}\" pos=\"{}\" quat=\"{}\" mass=\"{:.17}\" friction=\"{:.9}\"{sensor_attributes}/>",
        vector(collider.local_offset.translation),
        quaternion(rotation),
        rigid_body.mass_kg,
        collider.material.friction,
    )
    .expect("writing to String cannot fail");
}

fn invalid(entity_index: u32, field: &'static str) -> CompileError {
    CompileError::InvalidValue {
        entity_index,
        field,
    }
}

fn validate_positive(
    entity_index: u32,
    field: &'static str,
    value: f64,
) -> Result<(), CompileError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(invalid(entity_index, field))
    }
}

fn validate_vec3(entity_index: u32, field: &'static str, value: Vec3) -> Result<(), CompileError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(invalid(entity_index, field))
    }
}

fn validate_quat(entity_index: u32, field: &'static str, value: Quat) -> Result<(), CompileError> {
    if value.is_finite() && (value.length_squared() - 1.0).abs() <= QUATERNION_TOLERANCE {
        Ok(())
    } else {
        Err(invalid(entity_index, field))
    }
}

fn validate_unit_scale(
    entity_index: u32,
    field: &'static str,
    value: Vec3,
) -> Result<(), CompileError> {
    if value == Vec3::ONE {
        Ok(())
    } else {
        Err(invalid(entity_index, field))
    }
}

fn vector(value: Vec3) -> String {
    format!("{:.17} {:.17} {:.17}", value.x, value.y, value.z)
}

fn quaternion(value: Quat) -> String {
    format!(
        "{:.17} {:.17} {:.17} {:.17}",
        value.w, value.x, value.y, value.z
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::spawn_named;

    fn body(world: &mut World, name: &str, body_type: RigidBodyType, position: Vec3) -> Entity {
        let entity = spawn_named(world, name);
        world.entity_mut(entity).insert((
            RigidBody {
                body_type,
                mass_kg: 2.0,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::new(0.25, 0.5, 0.75)),
            Transform3::from_translation_rotation(position, Quat::IDENTITY),
        ));
        entity
    }

    #[test]
    fn compiles_canonical_backend_private_mjcf() {
        let mut world = World::new();
        let dynamic = body(
            &mut world,
            "dynamic",
            RigidBodyType::Dynamic,
            Vec3::new(1.0, 2.0, 3.0),
        );
        let fixed = body(
            &mut world,
            "fixed",
            RigidBodyType::Fixed,
            Vec3::new(0.0, -0.5, 0.0),
        );
        let compiled = compile_rigid_body_model(
            &world,
            PhysicsWorldDesc {
                gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
                solver_iterations: 12,
            },
            0.016_666_666,
        )
        .unwrap();

        assert_eq!(compiled.bindings[0].entity, dynamic);
        assert_eq!(compiled.bindings[1].entity, fixed);
        assert_eq!(compiled.bindings[0].joint.joint_name(), Some("rne_joint_0"));
        assert_eq!(compiled.bindings[1].joint, JointBinding::Fixed);
        assert!(compiled.mjcf.contains("iterations=\"12\""));
        assert!(compiled.mjcf.contains("<freejoint name=\"rne_joint_0\"/>"));
        assert!(compiled.mjcf.contains(
            "<body name=\"rne_body_1\" pos=\"0.00000000000000000 -0.50000000000000000 0.00000000000000000\""
        ));
        assert!(!compiled.mjcf.contains(
            "<body name=\"rne_body_0\" pos=\"1.00000000000000000 2.00000000000000000 3.00000000000000000\""
        ));
    }

    #[test]
    fn dynamic_pose_is_state_while_structural_pose_is_topology() {
        let mut world = World::new();
        let dynamic = body(&mut world, "dynamic", RigidBodyType::Dynamic, Vec3::ZERO);
        let fixed = body(&mut world, "fixed", RigidBodyType::Fixed, Vec3::ZERO);
        let first =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        world.get_mut::<Transform3>(dynamic).unwrap().translation.x = 3.0;
        let dynamic_moved =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert_eq!(first.topology, dynamic_moved.topology);

        world.get_mut::<Transform3>(fixed).unwrap().translation.x = 1.0;
        let fixed_moved =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert_ne!(first.topology, fixed_moved.topology);
    }

    #[test]
    fn compiles_revolute_and_prismatic_tree_with_actuators() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let hinge = body(
            &mut world,
            "hinge",
            RigidBodyType::Dynamic,
            Vec3::new(0.0, -1.0, 0.0),
        );
        world.entity_mut(hinge).insert((
            RevoluteJointDesc {
                parent: root,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::Y,
                lower_rad: Some(-0.5),
                upper_rad: Some(0.5),
            },
            JointActuation::RevoluteVelocity {
                target_velocity_rad_s: 1.0,
                gain_nm_s_per_rad: 2.0,
                max_effort_nm: 10.0,
            },
        ));
        let slider = body(
            &mut world,
            "slider",
            RigidBodyType::Dynamic,
            Vec3::new(0.0, -2.0, 0.0),
        );
        world.entity_mut(slider).insert(PrismaticJointDesc {
            parent: hinge,
            axis: Vec3::X,
            anchor_parent_m: Vec3::new(0.0, -1.0, 0.0),
            anchor_child_m: Vec3::ZERO,
            lower_m: Some(-0.2),
            upper_m: Some(0.2),
        });

        let compiled =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert!(compiled.mjcf.contains("type=\"hinge\""));
        assert!(compiled.mjcf.contains("type=\"slide\""));
        assert!(compiled.mjcf.contains("<actuator>"));
        assert!(compiled.mjcf.contains("rne_actuator_1"));
        assert!(compiled.mjcf.contains("rne_actuator_2"));
        assert!(matches!(
            compiled.bindings[1].joint,
            JointBinding::Revolute { .. }
        ));
        assert!(matches!(
            compiled.bindings[2].joint,
            JointBinding::Prismatic { .. }
        ));
    }

    #[test]
    fn mismatched_actuation_is_rejected_before_native_model_creation() {
        let mut world = World::new();
        let parent = body(&mut world, "parent", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, -Vec3::Y);
        world.entity_mut(child).insert((
            RevoluteJointDesc {
                parent,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::Y,
                lower_rad: None,
                upper_rad: None,
            },
            JointActuation::PrismaticEffort {
                force_n: 1.0,
                max_force_n: 2.0,
            },
        ));
        assert!(matches!(
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666,),
            Err(CompileError::InvalidActuation { .. })
        ));
    }
}
