//! Controller input/output boundary (the RNE analogue of Choreonoid's
//! `ControllerIO`).
//!
//! Controllers read a [`ControllerIoFrame`] snapshot of simulation time and
//! joint state and write a [`ControllerOutput`] of joint commands. The boundary
//! is deliberately small and backend-neutral: it does not depend on ROS,
//! renderers, physics engines, or the AI policy traits, so the same controller
//! can drive an ECS world, a plugin, or a test fixture.

use crate::components::{Actuator, Joint, JointKind, JointLimits};
use crate::joint::{validate_joint_position, validate_joint_velocity, JointValidationError};
use bevy_ecs::prelude::World;
use rne_ecs::Entity;
use std::collections::HashMap;
use thiserror::Error;

/// Read-only joint state exposed to a controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerJointState {
    /// Joint entity.
    pub joint: Entity,
    /// Current joint position in radians or meters.
    pub position: f64,
    /// Current joint velocity in radians per second or meters per second.
    pub velocity: f64,
    /// Joint limits.
    pub limits: JointLimits,
}

/// Input snapshot passed to a controller for one step.
#[derive(Clone, Debug, PartialEq)]
pub struct ControllerIoFrame {
    /// Owning robot entity.
    pub robot: Entity,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Fixed step duration in seconds.
    pub step_dt_s: f64,
    /// Joint state in deterministic joint order.
    pub joints: Vec<ControllerJointState>,
}

impl ControllerIoFrame {
    /// State of a joint, if present in the frame.
    pub fn joint(&self, joint: Entity) -> Option<&ControllerJointState> {
        self.joints.iter().find(|state| state.joint == joint)
    }

    /// Position of a joint, if present in the frame.
    pub fn position(&self, joint: Entity) -> Option<f64> {
        self.joint(joint).map(|state| state.position)
    }

    /// Velocity of a joint, if present in the frame.
    pub fn velocity(&self, joint: Entity) -> Option<f64> {
        self.joint(joint).map(|state| state.velocity)
    }
}

/// A single joint command produced by a controller.
///
/// Unset fields (`None`) are left untouched when the output is applied.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerCommand {
    /// Joint entity.
    pub joint: Entity,
    /// Target position in radians or meters.
    pub position: Option<f64>,
    /// Target velocity in radians per second or meters per second.
    pub velocity: Option<f64>,
    /// Target effort in newton-meters or newtons.
    pub effort: Option<f64>,
}

/// Output written by a controller for one step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ControllerOutput {
    /// Joint commands in controller order.
    pub commands: Vec<ControllerCommand>,
}

impl ControllerOutput {
    /// Creates an empty output.
    pub fn new() -> Self {
        Self::default()
    }

    /// Pushes a position command.
    pub fn set_position(&mut self, joint: Entity, position: f64) {
        self.commands.push(ControllerCommand {
            joint,
            position: Some(position),
            velocity: None,
            effort: None,
        });
    }

    /// Pushes a velocity command.
    pub fn set_velocity(&mut self, joint: Entity, velocity: f64) {
        self.commands.push(ControllerCommand {
            joint,
            position: None,
            velocity: Some(velocity),
            effort: None,
        });
    }

    /// Clears all commands.
    pub fn clear(&mut self) {
        self.commands.clear();
    }
}

/// A controller consumes an input frame and produces an output.
pub trait Controller {
    /// Advances the controller by one step.
    fn update(&mut self, io: &ControllerIoFrame, output: &mut ControllerOutput);
}

/// Summary of an applied controller output.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControllerApplyReport {
    /// Number of commands applied.
    pub applied: usize,
}

/// Error returned while building input frames or applying output.
#[derive(Clone, Copy, Debug, PartialEq, Error)]
pub enum ControllerIoError {
    /// A command referenced a joint that does not exist.
    #[error("controller command references unknown joint {0:?}")]
    UnknownJoint(Entity),
    /// A command contained a non-finite value.
    #[error("controller command for joint {0:?} contains a non-finite value")]
    NonFiniteCommand(Entity),
    /// A position or velocity command violated the joint limits.
    #[error("controller command for joint {0:?} violates limits: {1}")]
    JointLimits(Entity, JointValidationError),
    /// The step duration was not finite and positive.
    #[error("controller step duration must be finite and positive")]
    InvalidStep,
}

/// Builds an input frame for a robot from current joint state.
pub fn build_controller_io(
    world: &World,
    robot: Entity,
    time_s: f64,
    step_dt_s: f64,
) -> Result<ControllerIoFrame, ControllerIoError> {
    if !step_dt_s.is_finite() || step_dt_s <= 0.0 {
        return Err(ControllerIoError::InvalidStep);
    }
    let mut joints: Vec<(String, ControllerJointState)> = Vec::new();
    for entity_ref in world.iter_entities() {
        let Some(joint) = entity_ref.get::<Joint>() else {
            continue;
        };
        if joint.robot != robot || joint.kind == JointKind::Fixed {
            continue;
        }
        let name = entity_ref
            .get::<rne_ecs::Name>()
            .map(|name| name.0.clone())
            .unwrap_or_default();
        joints.push((
            name,
            ControllerJointState {
                joint: entity_ref.id(),
                position: joint.position,
                velocity: joint.velocity,
                limits: joint.limits,
            },
        ));
    }
    joints.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1.joint.index().cmp(&b.1.joint.index()))
    });
    Ok(ControllerIoFrame {
        robot,
        time_s,
        step_dt_s,
        joints: joints.into_iter().map(|(_, state)| state).collect(),
    })
}

/// Applies a controller output to the world.
///
/// Joint position and velocity are clamped to joint limits and written to the
/// [`Joint`] component. When an [`Actuator`] drives the same joint, provided
/// targets are additionally clamped to actuator limits and written to the
/// actuator's target.
pub fn apply_controller_output(
    world: &mut World,
    output: &ControllerOutput,
) -> Result<ControllerApplyReport, ControllerIoError> {
    let mut actuators: HashMap<Entity, Vec<(Entity, crate::actuator::ActuatorLimits)>> =
        HashMap::new();
    for entity_ref in world.iter_entities() {
        let Some(actuator) = entity_ref.get::<Actuator>() else {
            continue;
        };
        if let Some(joint) = actuator.joint {
            actuators
                .entry(joint)
                .or_default()
                .push((entity_ref.id(), actuator.limits));
        }
    }

    let mut applied = 0;
    for command in &output.commands {
        if let Some(position) = command.position {
            if !position.is_finite() {
                return Err(ControllerIoError::NonFiniteCommand(command.joint));
            }
        }
        if let Some(velocity) = command.velocity {
            if !velocity.is_finite() {
                return Err(ControllerIoError::NonFiniteCommand(command.joint));
            }
        }
        if let Some(effort) = command.effort {
            if !effort.is_finite() {
                return Err(ControllerIoError::NonFiniteCommand(command.joint));
            }
        }

        {
            let Some(mut joint) = world.get_mut::<Joint>(command.joint) else {
                return Err(ControllerIoError::UnknownJoint(command.joint));
            };
            if let Some(position) = command.position {
                let position = validate_joint_position(&joint, position)
                    .map_err(|error| ControllerIoError::JointLimits(command.joint, error))?;
                joint.position = position;
            }
            if let Some(velocity) = command.velocity {
                let velocity = validate_joint_velocity(&joint, velocity)
                    .map_err(|error| ControllerIoError::JointLimits(command.joint, error))?;
                joint.velocity = velocity;
            }
        }

        if let Some(drivers) = actuators.get(&command.joint) {
            for (entity, limits) in drivers {
                if let Some(mut actuator) = world.get_mut::<Actuator>(*entity) {
                    if let Some(position) = command.position {
                        actuator.target.position_rad = limits.clamp_position(position);
                    }
                    if let Some(velocity) = command.velocity {
                        actuator.target.velocity_rad_s = limits.clamp_velocity(velocity);
                    }
                    if let Some(effort) = command.effort {
                        actuator.target.effort_nm =
                            effort.clamp(-limits.max_effort_nm, limits.max_effort_nm);
                    }
                }
            }
        }
        applied += 1;
    }

    Ok(ControllerApplyReport { applied })
}

/// Runs a controller for one step: builds input, updates, and applies output.
pub fn step_controller<C: Controller>(
    world: &mut World,
    robot: Entity,
    time_s: f64,
    step_dt_s: f64,
    controller: &mut C,
    output: &mut ControllerOutput,
) -> Result<ControllerApplyReport, ControllerIoError> {
    let io = build_controller_io(world, robot, time_s, step_dt_s)?;
    output.clear();
    controller.update(&io, output);
    apply_controller_output(world, output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::{JointKind, Robot};
    use crate::joint::JointValidationError;
    use rne_ecs::{spawn_named, World};
    use rne_math::Vec3;

    struct HoldPosition;

    impl Controller for HoldPosition {
        fn update(&mut self, io: &ControllerIoFrame, output: &mut ControllerOutput) {
            for state in &io.joints {
                output.set_position(state.joint, 0.25);
            }
        }
    }

    fn robot_with_joint() -> (World, Entity, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let joint = spawn_named(&mut world, "joint");
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "robot".into(),
            base_link: base,
        });
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: base,
            kind: JointKind::Revolute,
            limits: JointLimits {
                lower: -1.0,
                upper: 1.0,
                max_velocity: 5.0,
                max_effort: 10.0,
            },
            axis: Vec3::Y,
            position: 0.0,
            velocity: 0.0,
        });
        (world, robot, joint)
    }

    #[test]
    fn step_controller_writes_joint_position() {
        let (mut world, robot, joint) = robot_with_joint();
        let mut controller = HoldPosition;
        let mut output = ControllerOutput::new();
        let report = step_controller(
            &mut world,
            robot,
            0.0,
            1.0 / 60.0,
            &mut controller,
            &mut output,
        )
        .unwrap();
        assert_eq!(report.applied, 1);
        assert_eq!(world.get::<Joint>(joint).unwrap().position, 0.25);
    }

    #[test]
    fn apply_rejects_out_of_limit_command() {
        let (mut world, _robot, joint) = robot_with_joint();
        let mut output = ControllerOutput::new();
        output.set_position(joint, 5.0);
        assert!(matches!(
            apply_controller_output(&mut world, &output),
            Err(ControllerIoError::JointLimits(
                _,
                JointValidationError::PositionOutOfLimits { .. }
            ))
        ));
    }

    #[test]
    fn apply_rejects_unknown_joint() {
        let (mut world, _robot, _joint) = robot_with_joint();
        let unknown = spawn_named(&mut world, "unknown");
        let mut output = ControllerOutput::new();
        output.set_velocity(unknown, 1.0);
        assert_eq!(
            apply_controller_output(&mut world, &output),
            Err(ControllerIoError::UnknownJoint(unknown))
        );
    }
}
