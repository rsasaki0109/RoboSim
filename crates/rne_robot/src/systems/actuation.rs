use super::*;

/// Applies queued actuator commands to actuators and joints.
pub fn apply_actuator_commands(world: &mut World, buffer: &mut ActuatorCommandBuffer) {
    let entries: Vec<_> = buffer.drain().collect();

    for entry in entries {
        let _ = apply_one_command(world, &entry.command);
    }
}

pub(crate) fn apply_one_command(
    world: &mut World,
    command: &ActuatorCommand,
) -> CommandApplyResult {
    match command {
        ActuatorCommand::JointPosition {
            joint,
            position_rad,
        } => apply_joint_position(world, *joint, *position_rad),
        ActuatorCommand::JointVelocity {
            joint,
            velocity_rad_s,
        } => apply_joint_velocity(world, *joint, *velocity_rad_s),
        ActuatorCommand::JointEffort { joint, effort_nm } => {
            apply_joint_effort(world, *joint, *effort_nm)
        }
        ActuatorCommand::WheelVelocity {
            wheel,
            velocity_rad_s,
        } => apply_wheel_velocity(world, *wheel, *velocity_rad_s),
        ActuatorCommand::GripperWidth { .. } | ActuatorCommand::BodyWrench { .. } => {
            CommandApplyResult::InvalidTarget
        }
        ActuatorCommand::Ackermann {
            vehicle,
            speed_m_s,
            steering_rad,
        } => match command_ackermann_drive(world, *vehicle, *speed_m_s, *steering_rad) {
            AckermannCommandResult::Applied => CommandApplyResult::Applied,
            AckermannCommandResult::InvalidTarget | AckermannCommandResult::NonFiniteCommand => {
                CommandApplyResult::InvalidTarget
            }
        },
    }
}

pub(crate) fn apply_joint_position(
    world: &mut World,
    joint_entity: Entity,
    position_rad: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let validated = match validate_joint_position(&joint, position_rad) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };
    joint_mut.position = validated;

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Position;
            actuator.target.position_rad = actuator.limits.clamp_position(validated);
        }
    }

    CommandApplyResult::Applied
}

pub(crate) fn apply_joint_velocity(
    world: &mut World,
    joint_entity: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(joint) = world.get::<Joint>(joint_entity).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    if joint.kind == JointKind::Fixed && velocity_rad_s.abs() > f64::EPSILON {
        return CommandApplyResult::JointRejected(JointValidationError::FixedJointNonZero);
    }

    let validated = match validate_joint_velocity(&joint, velocity_rad_s) {
        Ok(value) => value,
        Err(error) => return CommandApplyResult::JointRejected(error),
    };

    if let Some(mut joint_mut) = world.get_mut::<Joint>(joint_entity) {
        joint_mut.velocity = validated;
    }

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Velocity;
            actuator.target.velocity_rad_s = actuator.limits.clamp_velocity(validated);
        }
    }

    CommandApplyResult::Applied
}

pub(crate) fn apply_joint_effort(
    world: &mut World,
    joint_entity: Entity,
    effort_nm: f64,
) -> CommandApplyResult {
    let Some(_joint) = world.get::<Joint>(joint_entity) else {
        return CommandApplyResult::InvalidTarget;
    };

    if let Some(actuator_entity) = find_actuator_for_joint(world, joint_entity) {
        if let Some(mut actuator) = world.get_mut::<Actuator>(actuator_entity) {
            actuator.mode = ControlMode::Effort;
            actuator.target.effort_nm = effort_nm.clamp(
                -actuator.limits.max_effort_nm,
                actuator.limits.max_effort_nm,
            );
            return CommandApplyResult::Applied;
        }
    }

    CommandApplyResult::InvalidTarget
}

pub(crate) fn apply_wheel_velocity(
    world: &mut World,
    wheel_actuator: Entity,
    velocity_rad_s: f64,
) -> CommandApplyResult {
    let Some(actuator) = world.get::<Actuator>(wheel_actuator).cloned() else {
        return CommandApplyResult::InvalidTarget;
    };

    let clamped = actuator.limits.clamp_velocity(velocity_rad_s);
    let Some(mut actuator_mut) = world.get_mut::<Actuator>(wheel_actuator) else {
        return CommandApplyResult::InvalidTarget;
    };
    actuator_mut.mode = ControlMode::Velocity;
    actuator_mut.target.velocity_rad_s = clamped;

    if let Some(joint_entity) = actuator_mut.joint {
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.velocity = clamped;
        }
    }

    CommandApplyResult::Applied
}

pub(crate) fn find_actuator_for_joint(world: &World, joint_entity: Entity) -> Option<Entity> {
    for entity_ref in world.iter_entities() {
        let entity = entity_ref.id();
        if world
            .get::<Actuator>(entity)
            .is_some_and(|actuator| actuator.joint == Some(joint_entity))
        {
            return Some(entity);
        }
    }
    None
}

/// Integrates differential drive kinematics for one simulation step.
pub fn differential_drive_kinematics(
    world: &mut World,
    drives: &[DifferentialDrive],
    dt: SimDuration,
) {
    let dt_s = dt.as_seconds().value();

    for drive in drives {
        let Some(left) = world.get::<Actuator>(drive.left_actuator) else {
            continue;
        };
        let Some(right) = world.get::<Actuator>(drive.right_actuator) else {
            continue;
        };

        let v_left = left.target.velocity_rad_s * drive.wheel_radius_m;
        let v_right = right.target.velocity_rad_s * drive.wheel_radius_m;
        let linear_m_s = (v_left + v_right) * 0.5;
        let yaw_rad_s = (v_right - v_left) / drive.track_width_m;

        let (base_snapshot, forward) = {
            let Some(mut transform) = world.get_mut::<Transform3>(drive.base_link) else {
                continue;
            };

            let forward = transform.rotation * Vec3::X;
            transform.translation += forward * linear_m_s * dt_s;
            transform.rotation =
                (Quat::from_rotation_y(yaw_rad_s * dt_s) * transform.rotation).normalize();
            (*transform, forward)
        };

        if world
            .get::<RigidBody>(drive.base_link)
            .is_some_and(|body| body.body_type == RigidBodyType::Kinematic)
        {
            integrate_kinematic_wheel_joints(world, drive, dt_s);
            sync_wheel_transforms(world, drive, &base_snapshot);
        }

        if let Some(mut body) = world.get_mut::<RigidBody>(drive.base_link) {
            let forward_flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
            body.linear_velocity_m_s = forward_flat * linear_m_s;
            body.angular_velocity_rad_s = Vec3::new(0.0, yaw_rad_s, 0.0);
        }
    }
}

pub(crate) fn integrate_kinematic_wheel_joints(
    world: &mut World,
    drive: &DifferentialDrive,
    dt_s: f64,
) {
    for actuator_entity in [drive.left_actuator, drive.right_actuator] {
        let Some(joint_entity) = world
            .get::<Actuator>(actuator_entity)
            .and_then(|actuator| actuator.joint)
        else {
            continue;
        };
        if let Some(mut joint) = world.get_mut::<Joint>(joint_entity) {
            joint.position += joint.velocity * dt_s;
        }
    }
}

pub(crate) fn sync_wheel_transforms(
    world: &mut World,
    drive: &DifferentialDrive,
    base: &Transform3,
) {
    let half_track = drive.track_width_m * 0.5;
    let wheel_y = world
        .get::<Collider>(drive.base_link)
        .and_then(|collider| match collider.shape {
            ColliderShape::Cuboid { half_extents_m } => Some(-half_extents_m.y),
            _ => None,
        })
        .unwrap_or(0.0);

    for (wheel, z_offset) in [
        (drive.left_actuator, -half_track),
        (drive.right_actuator, half_track),
    ] {
        let Some(actuator) = world.get::<Actuator>(wheel) else {
            continue;
        };
        let Some(wheel_entity) = actuator.joint else {
            continue;
        };
        let Some(mut wheel_transform) = world.get_mut::<Transform3>(wheel_entity) else {
            continue;
        };
        let offset = base.rotation * Vec3::new(0.0, wheel_y, z_offset);
        wheel_transform.translation = base.translation + offset;
        wheel_transform.rotation = base.rotation;
    }
}

/// Copies every actuator target into unit-explicit [`JointActuation`].
///
/// The optional `drives` argument on [`sync_joint_motors_from_actuators`] is kept
/// for source compatibility with older diff-drive callers. Named URDF actuators
/// use this function directly and are resolved through their [`Joint`] child link.
/// Existing [`JointMotor`] components are updated as a compatibility path.
pub fn sync_all_joint_motors_from_actuators(world: &mut World) {
    let mut actuator_entities: Vec<_> = world
        .iter_entities()
        .map(|entity| entity.id())
        .filter(|entity| world.get::<Actuator>(*entity).is_some())
        .collect();
    actuator_entities.sort_unstable();

    for actuator_entity in actuator_entities {
        let Some((joint_entity, mode, target, limits)) =
            world.get::<Actuator>(actuator_entity).map(|actuator| {
                (
                    actuator.joint,
                    actuator.mode,
                    actuator.target,
                    actuator.limits,
                )
            })
        else {
            continue;
        };
        let Some(joint_entity) = joint_entity else {
            continue;
        };
        let Some((child_link, joint_kind)) = world
            .get::<Joint>(joint_entity)
            .map(|joint| (joint.child_link, joint.kind))
        else {
            continue;
        };
        let tuning = world
            .get::<JointMotor>(child_link)
            .copied()
            .unwrap_or_default();
        let max_output = if limits.max_effort_nm.is_finite() {
            limits.max_effort_nm.max(0.0)
        } else {
            0.0
        };
        let stiffness = if tuning.stiffness.is_finite() && tuning.stiffness > 0.0 {
            tuning.stiffness
        } else {
            40.0
        };
        let gain = if tuning.gain.is_finite() && tuning.gain >= 0.0 {
            tuning.gain
        } else {
            1.0
        };
        let actuation = match (joint_kind, mode) {
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Position) => {
                JointActuation::RevolutePosition {
                    target_position_rad: target.position_rad,
                    stiffness_nm_per_rad: stiffness,
                    damping_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Velocity) => {
                JointActuation::RevoluteVelocity {
                    target_velocity_rad_s: target.velocity_rad_s,
                    gain_nm_s_per_rad: gain,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Revolute | JointKind::Continuous, ControlMode::Effort) => {
                JointActuation::RevoluteEffort {
                    effort_nm: target.effort_nm,
                    max_effort_nm: max_output,
                }
            }
            (JointKind::Prismatic, ControlMode::Position) => JointActuation::PrismaticPosition {
                target_position_m: target.position_rad,
                stiffness_n_per_m: stiffness,
                damping_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Velocity) => JointActuation::PrismaticVelocity {
                target_velocity_m_s: target.velocity_rad_s,
                gain_n_s_per_m: gain,
                max_force_n: max_output,
            },
            (JointKind::Prismatic, ControlMode::Effort) => JointActuation::PrismaticEffort {
                force_n: target.effort_nm,
                max_force_n: max_output,
            },
            (JointKind::Fixed, _) => JointActuation::Disabled,
        };
        world.entity_mut(child_link).insert(actuation);
        if let Some(mut motor) = world.get_mut::<JointMotor>(child_link) {
            motor.velocity_rad_s = match mode {
                ControlMode::Velocity => target.velocity_rad_s,
                ControlMode::Position | ControlMode::Effort => 0.0,
            };
            if mode == ControlMode::Position {
                motor.target_position = target.position_rad;
                motor.stiffness = stiffness;
            }
        }
    }
}

/// Copies actuator velocity targets into [`JointMotor`] components for physics stepping.
pub fn sync_joint_motors_from_actuators(world: &mut World, _drives: &[DifferentialDrive]) {
    sync_all_joint_motors_from_actuators(world);
}
