//! Deterministic ECS-to-MJCF compilation kept private to the MuJoCo adapter.

use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, FixedJointDesc, JointMotor, MultibodyLink, PhysicsCapability,
    PhysicsWorldDesc, PrismaticJointDesc, RevoluteJointDesc, RigidBody, RigidBodyType,
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
    pub(crate) joint_name: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BodyTopology {
    entity: Entity,
    body_type: RigidBodyType,
    mass_kg: f64,
    collider: Collider,
    fixed_transform: Option<Transform3>,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub(crate) enum CompileError {
    #[error("MuJoCo backend lacks required capability {0:?}")]
    MissingCapability(PhysicsCapability),
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
    let mut bodies = Vec::new();
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if entity_ref.contains::<RevoluteJointDesc>()
            || entity_ref.contains::<PrismaticJointDesc>()
            || entity_ref.contains::<FixedJointDesc>()
            || entity_ref.contains::<MultibodyLink>()
            || entity_ref.contains::<JointMotor>()
        {
            return Err(CompileError::MissingCapability(
                PhysicsCapability::Articulation,
            ));
        }

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
                bodies.push((entity, rigid_body, collider, transform));
            }
        }
    }
    if bodies.is_empty() {
        return Err(CompileError::EmptyWorld);
    }
    bodies.sort_unstable_by_key(|(entity, _, _, _)| entity.index());

    let mut mjcf =
        String::from("<mujoco model=\"rne-ecs-rigid-bodies\">\n  <compiler angle=\"radian\"/>\n");
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
    for (entity, rigid_body, collider, transform) in bodies {
        let index = entity.index();
        let joint_name =
            (rigid_body.body_type == RigidBodyType::Dynamic).then(|| format!("rne_joint_{index}"));
        let compiled_transform = if rigid_body.body_type == RigidBodyType::Fixed {
            transform
        } else {
            Transform3::IDENTITY
        };
        writeln!(
            mjcf,
            "    <body name=\"rne_body_{index}\" pos=\"{}\" quat=\"{}\">",
            vector(compiled_transform.translation),
            quaternion(compiled_transform.rotation)
        )
        .expect("writing to String cannot fail");
        if let Some(name) = joint_name.as_deref() {
            writeln!(mjcf, "      <freejoint name=\"{name}\"/>")
                .expect("writing to String cannot fail");
        }
        write_geom(&mut mjcf, index, rigid_body, collider);
        mjcf.push_str("    </body>\n");

        bindings.push(BodyBinding { entity, joint_name });
        topology.push(BodyTopology {
            entity,
            body_type: rigid_body.body_type,
            mass_kg: rigid_body.mass_kg,
            collider,
            fixed_transform: (rigid_body.body_type == RigidBodyType::Fixed).then_some(transform),
        });
    }
    mjcf.push_str("  </worldbody>\n</mujoco>\n");
    Ok(CompiledRigidBodyModel {
        mjcf,
        bindings,
        topology,
    })
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
    if collider.sensor {
        return Err(CompileError::MissingCapability(
            PhysicsCapability::ContactForce,
        ));
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

fn write_geom(output: &mut String, index: u32, rigid_body: RigidBody, collider: Collider) {
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
    writeln!(
        output,
        "      <geom name=\"rne_geom_{index}\" type=\"{kind}\" size=\"{size}\" pos=\"{}\" quat=\"{}\" mass=\"{:.17}\" friction=\"{:.9}\"/>",
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
    use rne_math::Vec3;

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
        assert_eq!(
            compiled.bindings[0].joint_name.as_deref(),
            Some("rne_joint_0")
        );
        assert_eq!(compiled.bindings[1].joint_name, None);
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
    fn dynamic_pose_is_state_while_fixed_pose_is_topology() {
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
    fn articulation_is_rejected_before_native_model_creation() {
        let mut world = World::new();
        let parent = body(&mut world, "parent", RigidBodyType::Fixed, Vec3::ZERO);
        let child = body(&mut world, "child", RigidBodyType::Dynamic, Vec3::Y);
        world.entity_mut(child).insert(RevoluteJointDesc {
            parent,
            axis: Vec3::Z,
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::ZERO,
            lower_rad: None,
            upper_rad: None,
        });
        assert_eq!(
            compile_rigid_body_model(&world, PhysicsWorldDesc::default(), 0.016_666_666,),
            Err(CompileError::MissingCapability(
                PhysicsCapability::Articulation
            ))
        );
    }
}
