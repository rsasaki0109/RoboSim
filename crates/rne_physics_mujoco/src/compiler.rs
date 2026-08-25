//! Deterministic ECS-to-MJCF compilation kept private to the MuJoCo adapter.

use rne_ecs::{Entity, Parent, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, FixedJointDesc, JointActuation, JointMotor, JointPassiveDynamics,
    PhysicsCapability, PhysicsWorldDesc, PrismaticJointDesc, RevoluteJointDesc, RigidBody,
    RigidBodyInertia, RigidBodyType,
};
use rne_world::{world_transform_of, Transform3};
use std::fmt::Write as _;
use thiserror::Error;

const QUATERNION_TOLERANCE: f64 = 1.0e-9;
const LEGACY_REVOLUTE_STIFFNESS_MAX: f64 = 100.0;
const LEGACY_REVOLUTE_DAMPING_MAX: f64 = 20.0;
const LEGACY_PRISMATIC_STIFFNESS_MAX: f64 = 1_000.0;
const LEGACY_PRISMATIC_DAMPING_MAX: f64 = 80.0;

pub(crate) fn legacy_motor_gains(motor: JointMotor, revolute: bool) -> (f64, f64) {
    let (stiffness_max, damping_max) = if revolute {
        (LEGACY_REVOLUTE_STIFFNESS_MAX, LEGACY_REVOLUTE_DAMPING_MAX)
    } else {
        (LEGACY_PRISMATIC_STIFFNESS_MAX, LEGACY_PRISMATIC_DAMPING_MAX)
    };
    (
        motor.stiffness.min(stiffness_max),
        motor.gain.min(damping_max),
    )
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CompiledRigidBodyModel {
    pub(crate) mjcf: String,
    pub(crate) bindings: Vec<BodyBinding>,
    pub(crate) topology: Vec<BodyTopology>,
    pub(crate) joint_dynamics: Vec<JointDynamics>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct JointDynamics {
    entity: Entity,
    implicit_damping: f64,
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
    inertia: Option<RigidBodyInertia>,
    collider: Option<Collider>,
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
    inertia: Option<RigidBodyInertia>,
    collider: Option<Collider>,
    local_transform: Transform3,
    transform: Transform3,
    ecs_parent: Option<Entity>,
    joint: Option<JointSpec>,
    legacy_motor: Option<JointMotor>,
    actuation: Option<JointActuation>,
    passive_dynamics: Option<JointPassiveDynamics>,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub(crate) enum CompileError {
    #[error("entity {entity_index} requires unsupported capability {capability:?}")]
    MissingCapability {
        capability: PhysicsCapability,
        entity_index: u32,
    },
    #[error("MuJoCo rigid-body world is empty")]
    EmptyWorld,
    #[error("entity {entity_index} has a collider but no rigid body")]
    ColliderWithoutRigidBody { entity_index: u32 },
    #[error("entity {entity_index} has a rigid body but no Transform3")]
    MissingTransform { entity_index: u32 },
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
    #[error("invalid passive joint dynamics on entity {entity_index}: {reason}")]
    InvalidPassiveDynamics {
        entity_index: u32,
        reason: &'static str,
    },
    #[error("invalid rigid-body inertia on entity {entity_index}: {reason}")]
    InvalidInertia {
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
    let integrator = if bodies.iter().any(|body| {
        matches!(
            body.joint,
            Some(JointSpec::Revolute(_) | JointSpec::Prismatic(_))
        )
    }) {
        // Stiff position-controlled robot joints need MuJoCo's implicit
        // velocity treatment at 60 Hz. Collider-only rigid-body fixtures keep
        // Euler so their registered semi-implicit conformance vector remains
        // unchanged.
        "implicitfast"
    } else {
        "Euler"
    };
    write!(
        mjcf,
        "  <option timestep=\"{timestep_s:.9}\" gravity=\"{}\" integrator=\"{integrator}\"",
        vector(desc.gravity_m_s2),
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
    let joint_dynamics = bodies
        .iter()
        .filter_map(|body| {
            passive_damping(*body).map(|implicit_damping| JointDynamics {
                entity: body.entity,
                implicit_damping,
            })
        })
        .collect();
    Ok(CompiledRigidBodyModel {
        mjcf,
        bindings,
        topology,
        joint_dynamics,
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
            (None, None) => continue,
            (Some(rigid_body), collider) => {
                let local_transform = entity_ref.get::<Transform3>().copied().ok_or(
                    CompileError::MissingTransform {
                        entity_index: entity.index(),
                    },
                )?;
                // ECS Transform3 components are local to their Parent. MuJoCo's
                // compiler consumes world poses before deriving its own nested
                // body frames, so feeding the local component directly would
                // apply every parent transform twice for imported robots.
                let transform = world_transform_of(world, entity);
                let inertia = entity_ref.get::<RigidBodyInertia>().copied();
                validate_body(entity, rigid_body, inertia, collider, transform)?;
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
                let legacy_motor = entity_ref
                    .get::<JointActuation>()
                    .is_none()
                    .then(|| entity_ref.get::<JointMotor>().copied())
                    .flatten();
                bodies.push(BodyInput {
                    entity,
                    rigid_body,
                    inertia,
                    collider,
                    local_transform,
                    transform,
                    ecs_parent: entity_ref.get::<Parent>().map(|parent| parent.0),
                    joint: joints.into_iter().flatten().next(),
                    legacy_motor,
                    actuation: entity_ref.get::<JointActuation>().copied(),
                    passive_dynamics: entity_ref.get::<JointPassiveDynamics>().copied(),
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
            if body.passive_dynamics.is_some() {
                return Err(CompileError::InvalidPassiveDynamics {
                    entity_index: body.entity.index(),
                    reason: "passive dynamics without joint",
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
        validate_legacy_motor(body, joint)?;
        validate_passive_dynamics(body, joint)?;

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

fn validate_passive_dynamics(body: &BodyInput, joint: JointSpec) -> Result<(), CompileError> {
    let Some(dynamics) = body.passive_dynamics else {
        return Ok(());
    };
    if !dynamics.has_valid_values() {
        return Err(CompileError::InvalidPassiveDynamics {
            entity_index: body.entity.index(),
            reason: "non-finite or negative coefficient",
        });
    }
    let coulomb_friction = match dynamics {
        JointPassiveDynamics::Revolute {
            coulomb_friction_nm,
            ..
        } => coulomb_friction_nm,
        JointPassiveDynamics::Prismatic {
            coulomb_friction_n, ..
        } => coulomb_friction_n,
    };
    if coulomb_friction != 0.0 {
        return Err(CompileError::InvalidPassiveDynamics {
            entity_index: body.entity.index(),
            reason: "nonzero Coulomb friction is not yet portable",
        });
    }
    let compatible = matches!(
        (dynamics, joint),
        (
            JointPassiveDynamics::Revolute { .. },
            JointSpec::Revolute(_)
        ) | (
            JointPassiveDynamics::Prismatic { .. },
            JointSpec::Prismatic(_)
        )
    );
    if compatible {
        Ok(())
    } else {
        Err(CompileError::InvalidPassiveDynamics {
            entity_index: body.entity.index(),
            reason: "joint kind mismatch",
        })
    }
}

fn validate_legacy_motor(body: &BodyInput, joint: JointSpec) -> Result<(), CompileError> {
    let Some(motor) = body.legacy_motor else {
        return Ok(());
    };
    if matches!(joint, JointSpec::Fixed(_)) {
        return Err(CompileError::InvalidActuation {
            entity_index: body.entity.index(),
            reason: "legacy JointMotor cannot actuate a fixed joint",
        });
    }
    if motor.velocity_rad_s.is_finite()
        && motor.gain.is_finite()
        && motor.stiffness.is_finite()
        && motor.target_position.is_finite()
        && motor.max_force.is_finite()
        && motor.gain >= 0.0
        && motor.stiffness >= 0.0
        && motor.max_force >= 0.0
    {
        Ok(())
    } else {
        Err(CompileError::InvalidActuation {
            entity_index: body.entity.index(),
            reason: "legacy JointMotor value, gain, or limit",
        })
    }
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
        |parent| {
            if matches!(body.joint, Some(JointSpec::Fixed(_)))
                && body.ecs_parent == Some(parent.entity)
            {
                body.local_transform
            } else {
                relative_transform(parent.transform, body.transform)
            }
        },
    );
    writeln!(
        output,
        "{indent}<body name=\"{body_name}\" pos=\"{}\" quat=\"{}\">",
        vector(structural_transform.translation),
        quaternion(structural_transform.rotation)
    )
    .expect("writing to String cannot fail");

    let joint = write_joint(output, body, structural_transform, depth + 1, actuators);
    if let Some(inertia) = body.inertia {
        write_exact_inertial(output, body.rigid_body, inertia, depth + 1);
    }
    if let Some(collider) = body.collider {
        write_geom(
            output,
            index,
            body.rigid_body,
            collider,
            body.inertia.is_none(),
            depth + 1,
        );
    } else if body.inertia.is_none() {
        write_inertial(output, body.rigid_body, depth + 1);
    }
    bindings.push(BodyBinding {
        entity: body.entity,
        body_name,
        joint,
    });
    topology.push(BodyTopology {
        entity: body.entity,
        body_type: body.rigid_body.body_type,
        mass_kg: body.rigid_body.mass_kg,
        inertia: body.inertia,
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
            write_passive_damping(output, body, true);
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
            write_passive_damping(output, body, false);
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

fn passive_damping(body: BodyInput) -> Option<f64> {
    let revolute = matches!(body.joint, Some(JointSpec::Revolute(_)));
    let prismatic = matches!(body.joint, Some(JointSpec::Prismatic(_)));
    if !revolute && !prismatic {
        return None;
    }
    let actuator_damping = if let Some(actuation) = body.actuation {
        Some(match actuation {
            JointActuation::RevolutePosition {
                damping_nm_s_per_rad,
                ..
            } if revolute => damping_nm_s_per_rad,
            JointActuation::RevoluteVelocity {
                gain_nm_s_per_rad, ..
            } if revolute => gain_nm_s_per_rad,
            JointActuation::PrismaticPosition {
                damping_n_s_per_m, ..
            } if prismatic => damping_n_s_per_m,
            JointActuation::PrismaticVelocity { gain_n_s_per_m, .. } if prismatic => gain_n_s_per_m,
            JointActuation::Disabled
            | JointActuation::RevoluteEffort { .. }
            | JointActuation::PrismaticEffort { .. } => 0.0,
            _ => 0.0,
        })
    } else {
        body.legacy_motor
            .map(|motor| legacy_motor_gains(motor, revolute).1)
    };
    let plant_damping = match body.passive_dynamics {
        Some(JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad,
            ..
        }) if revolute => Some(viscous_damping_nm_s_per_rad),
        Some(JointPassiveDynamics::Prismatic {
            viscous_damping_n_s_per_m,
            ..
        }) if prismatic => Some(viscous_damping_n_s_per_m),
        _ => None,
    };
    match (actuator_damping, plant_damping) {
        (None, None) => None,
        (actuator, plant) => Some(actuator.unwrap_or(0.0) + plant.unwrap_or(0.0)),
    }
}

fn write_passive_damping(output: &mut String, body: BodyInput, revolute: bool) {
    let expected_joint = if revolute {
        matches!(body.joint, Some(JointSpec::Revolute(_)))
    } else {
        matches!(body.joint, Some(JointSpec::Prismatic(_)))
    };
    if !expected_joint {
        return;
    }
    let Some(damping) = passive_damping(body) else {
        return;
    };
    write!(output, " damping=\"{damping:.17}\"").expect("writing to String cannot fail");
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
    inertia: Option<RigidBodyInertia>,
    collider: Option<Collider>,
    transform: Transform3,
) -> Result<(), CompileError> {
    let entity_index = entity.index();
    if rigid_body.body_type == RigidBodyType::Kinematic {
        return Err(CompileError::MissingCapability {
            capability: PhysicsCapability::KinematicBody,
            entity_index,
        });
    }
    if !rigid_body.mass_kg.is_finite() || rigid_body.mass_kg <= 0.0 {
        return Err(invalid(entity_index, "mass_kg"));
    }
    if inertia.is_some_and(|properties| !properties.is_valid()) {
        return Err(CompileError::InvalidInertia {
            entity_index,
            reason: "physically invalid tensor",
        });
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
    let Some(collider) = collider else {
        return Ok(());
    };
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
    include_mass: bool,
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
    let mass_attribute = if include_mass {
        format!(" mass=\"{:.17}\"", rigid_body.mass_kg)
    } else {
        String::new()
    };
    writeln!(
        output,
        "{indent}<geom name=\"rne_geom_{index}\" type=\"{kind}\" size=\"{size}\" pos=\"{}\" quat=\"{}\"{mass_attribute} friction=\"{:.9}\"{sensor_attributes}/>",
        vector(collider.local_offset.translation),
        quaternion(rotation),
        collider.material.friction,
    )
    .expect("writing to String cannot fail");
}

fn write_exact_inertial(
    output: &mut String,
    rigid_body: RigidBody,
    inertia: RigidBodyInertia,
    depth: usize,
) {
    let indent = "  ".repeat(depth);
    writeln!(
        output,
        "{indent}<inertial pos=\"{}\" mass=\"{:.17}\" fullinertia=\"{:.17} {:.17} {:.17} {:.17} {:.17} {:.17}\"/>",
        vector(inertia.center_of_mass_local_m),
        rigid_body.mass_kg,
        inertia.ixx_kg_m2,
        inertia.iyy_kg_m2,
        inertia.izz_kg_m2,
        inertia.ixy_kg_m2,
        inertia.ixz_kg_m2,
        inertia.iyz_kg_m2,
    )
    .expect("writing to String cannot fail");
}

fn write_inertial(output: &mut String, rigid_body: RigidBody, depth: usize) {
    // Legacy bodies without an exact inertia component retain the historical
    // backend-private isotropic fallback.
    let inertia_kg_m2 = (rigid_body.mass_kg * 1.0e-2).max(1.0e-9);
    let indent = "  ".repeat(depth);
    writeln!(
        output,
        "{indent}<inertial pos=\"0 0 0\" mass=\"{:.17}\" diaginertia=\"{:.17} {:.17} {:.17}\"/>",
        rigid_body.mass_kg, inertia_kg_m2, inertia_kg_m2, inertia_kg_m2,
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
    fn exact_inertia_is_emitted_independently_of_collision_geometry() {
        let mut world = World::new();
        let dynamic = body(&mut world, "identified", RigidBodyType::Dynamic, Vec3::ZERO);
        world.entity_mut(dynamic).insert(RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(0.1, -0.2, 0.3),
            ixx_kg_m2: 0.4,
            ixy_kg_m2: 0.01,
            ixz_kg_m2: -0.02,
            iyy_kg_m2: 0.5,
            iyz_kg_m2: 0.03,
            izz_kg_m2: 0.6,
        });

        let compiled =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert!(compiled.mjcf.contains(
            "<inertial pos=\"0.10000000000000001 -0.20000000000000001 0.29999999999999999\" mass=\"2.00000000000000000\" fullinertia=\"0.40000000000000002 0.50000000000000000 0.59999999999999998 0.01000000000000000 -0.02000000000000000 0.03000000000000000\"/>"
        ));
        let geom = compiled
            .mjcf
            .lines()
            .find(|line| line.contains("rne_geom_0"))
            .expect("collision geometry");
        assert!(!geom.contains(" mass="));
    }

    #[test]
    fn invalid_exact_inertia_is_rejected() {
        let mut world = World::new();
        let dynamic = body(&mut world, "invalid", RigidBodyType::Dynamic, Vec3::ZERO);
        world.entity_mut(dynamic).insert(RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: 1.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: -1.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 1.0,
        });
        assert!(matches!(
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666),
            Err(CompileError::InvalidInertia { .. })
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
    fn legacy_motor_damping_is_native_dynamics_not_body_topology() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, Vec3::Y);
        world.entity_mut(child).insert((
            RevoluteJointDesc {
                parent: root,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                lower_rad: None,
                upper_rad: None,
            },
            JointMotor {
                gain: 60.0,
                stiffness: 400.0,
                ..JointMotor::default()
            },
        ));

        let first =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert!(first.mjcf.contains("damping=\"20.00000000000000000\""));
        assert_eq!(first.joint_dynamics.len(), 1);

        world.get_mut::<JointMotor>(child).unwrap().gain = 10.0;
        let changed =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert_eq!(first.topology, changed.topology);
        assert_ne!(first.joint_dynamics, changed.joint_dynamics);
        assert!(changed.mjcf.contains("damping=\"10.00000000000000000\""));
    }

    #[test]
    fn passive_joint_loss_is_separate_from_actuator_damping() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, Vec3::Y);
        world.entity_mut(child).insert((
            RevoluteJointDesc {
                parent: root,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                lower_rad: None,
                upper_rad: None,
            },
            JointActuation::RevolutePosition {
                target_position_rad: 0.0,
                stiffness_nm_per_rad: 120.0,
                damping_nm_s_per_rad: 20.0,
                max_effort_nm: 7.0,
            },
            JointPassiveDynamics::Revolute {
                viscous_damping_nm_s_per_rad: 2.5,
                coulomb_friction_nm: 0.0,
            },
        ));

        let compiled =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert!(compiled.mjcf.contains("damping=\"22.50000000000000000\""));
        assert!(!compiled.mjcf.contains("frictionloss="));
        assert_eq!(
            compiled.joint_dynamics,
            vec![JointDynamics {
                entity: child,
                implicit_damping: 22.5,
            }]
        );
    }

    #[test]
    fn nonzero_coulomb_friction_fails_before_native_model_creation() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, Vec3::Y);
        world.entity_mut(child).insert((
            RevoluteJointDesc {
                parent: root,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                lower_rad: None,
                upper_rad: None,
            },
            JointPassiveDynamics::Revolute {
                viscous_damping_nm_s_per_rad: 0.0,
                coulomb_friction_nm: 0.1,
            },
        ));
        assert!(matches!(
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666),
            Err(CompileError::InvalidPassiveDynamics { .. })
        ));
    }

    #[test]
    fn typed_actuation_damping_is_compiled_as_implicit_joint_dynamics() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, Vec3::Y);
        world.entity_mut(child).insert((
            RevoluteJointDesc {
                parent: root,
                axis: Vec3::Z,
                anchor_parent_m: Vec3::ZERO,
                anchor_child_m: Vec3::ZERO,
                lower_rad: None,
                upper_rad: None,
            },
            JointActuation::RevolutePosition {
                target_position_rad: 0.2,
                stiffness_nm_per_rad: 120.0,
                damping_nm_s_per_rad: 7.5,
                max_effort_nm: 20.0,
            },
        ));

        let first =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert!(first.mjcf.contains("damping=\"7.50000000000000000\""));
        assert_eq!(first.joint_dynamics.len(), 1);

        let JointActuation::RevolutePosition {
            target_position_rad,
            stiffness_nm_per_rad,
            max_effort_nm,
            ..
        } = *world.get::<JointActuation>(child).unwrap()
        else {
            unreachable!()
        };
        world
            .entity_mut(child)
            .insert(JointActuation::RevolutePosition {
                target_position_rad,
                stiffness_nm_per_rad,
                damping_nm_s_per_rad: 9.0,
                max_effort_nm,
            });
        let changed =
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666).unwrap();
        assert_eq!(first.topology, changed.topology);
        assert_ne!(first.joint_dynamics, changed.joint_dynamics);
        assert!(changed.mjcf.contains("damping=\"9.00000000000000000\""));
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
        assert!(compiled.mjcf.contains("integrator=\"implicitfast\""));
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
    fn colliderless_articulated_link_compiles_with_backend_private_inertia() {
        let mut world = World::new();
        let root = body(&mut world, "root", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(
            &mut world,
            "visual_only_wheel",
            RigidBodyType::Dynamic,
            -Vec3::Y,
        );
        world.entity_mut(child).remove::<Collider>();
        world.entity_mut(child).insert(RevoluteJointDesc {
            parent: root,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::Y,
            lower_rad: None,
            upper_rad: None,
        });

        let compiled = compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666)
            .expect("colliderless articulated link");
        assert!(compiled
            .mjcf
            .contains("<inertial pos=\"0 0 0\" mass=\"2.00000000000000000\""));
        assert!(!compiled.mjcf.contains("rne_geom_1"));
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
