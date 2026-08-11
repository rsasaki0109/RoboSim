//! Deterministic multi-controller and multi-robot fixed-step scheduling.

use crate::{
    ControllerActionFrame, ControllerCapability, ControllerConfiguration, ControllerHost,
    ControllerJointVelocityCommand, ControllerLifecycleError, ControllerLifecycleState,
    ControllerObservationFrame, ControllerPlugin, ControllerResetContext, ControllerRobotAction,
    ControllerSchemaError,
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug)]
struct ScheduledController {
    robot_ids: Vec<String>,
    host: ControllerHost,
}

/// Stable scheduler that merges controller outputs by robot and joint name.
#[derive(Debug)]
pub struct ControllerScheduler {
    controllers: BTreeMap<String, ScheduledController>,
    state: ControllerLifecycleState,
}

impl Default for ControllerScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl ControllerScheduler {
    /// Creates an empty scheduler in the created state.
    pub fn new() -> Self {
        Self {
            controllers: BTreeMap::new(),
            state: ControllerLifecycleState::Created,
        }
    }

    /// Returns the scheduler lifecycle state shared by all registered hosts.
    pub fn state(&self) -> ControllerLifecycleState {
        self.state
    }

    /// Returns registered controller IDs in deterministic scheduling order.
    pub fn controller_ids(&self) -> impl Iterator<Item = &str> {
        self.controllers.keys().map(String::as_str)
    }

    /// Registers one controller and its sorted, unique robot assignment.
    pub fn register(
        &mut self,
        controller_id: impl Into<String>,
        plugin: Box<dyn ControllerPlugin>,
        robot_ids: impl IntoIterator<Item = String>,
    ) -> Result<(), ControllerScheduleError> {
        self.require_state("register", ControllerLifecycleState::Created)?;
        let controller_id = controller_id.into();
        validate_id("controller_id", &controller_id)?;
        if self.controllers.contains_key(&controller_id) {
            return Err(ControllerScheduleError::DuplicateController(controller_id));
        }
        let robot_ids = robot_ids.into_iter().collect::<BTreeSet<_>>();
        if robot_ids.is_empty() {
            return Err(ControllerScheduleError::Invalid(
                "controller robot assignment must not be empty".to_string(),
            ));
        }
        for robot_id in &robot_ids {
            validate_id("robot_id", robot_id)?;
        }
        let host =
            ControllerHost::new(plugin).map_err(|source| ControllerScheduleError::Controller {
                controller_id: controller_id.clone(),
                source,
            })?;
        self.controllers.insert(
            controller_id,
            ScheduledController {
                robot_ids: robot_ids.into_iter().collect(),
                host,
            },
        );
        Ok(())
    }

    /// Negotiates every controller in stable ID order.
    ///
    /// Joint-position input and joint-velocity output are mandatory for this
    /// scheduler. Multi-robot support is additionally required for any single
    /// controller assigned to more than one robot.
    pub fn configure(&mut self) -> Result<(), ControllerScheduleError> {
        self.require_state("configure", ControllerLifecycleState::Created)?;
        if self.controllers.is_empty() {
            return Err(ControllerScheduleError::Invalid(
                "controller scheduler must contain at least one controller".to_string(),
            ));
        }

        for (controller_id, controller) in &self.controllers {
            let mut required = vec![
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
            ];
            if controller.robot_ids.len() > 1 {
                required.push(ControllerCapability::MultiRobot);
            }
            let missing = required
                .into_iter()
                .filter(|capability| !controller.host.descriptor().supports(*capability))
                .collect::<Vec<_>>();
            if !missing.is_empty() {
                return Err(ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source: ControllerLifecycleError::MissingCapabilities(missing),
                });
            }
        }

        for (controller_id, controller) in &mut self.controllers {
            let mut required = vec![
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
            ];
            if controller.robot_ids.len() > 1 {
                required.push(ControllerCapability::MultiRobot);
            }
            controller
                .host
                .configure(ControllerConfiguration::new(required))
                .map_err(|source| ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source,
                })?;
        }
        self.state = ControllerLifecycleState::Configured;
        Ok(())
    }

    /// Activates the first episode for every controller in stable ID order.
    pub fn activate(
        &mut self,
        context: ControllerResetContext,
    ) -> Result<(), ControllerScheduleError> {
        self.require_state("activate", ControllerLifecycleState::Configured)?;
        for (controller_id, controller) in &mut self.controllers {
            controller.host.activate(context).map_err(|source| {
                ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source,
                }
            })?;
        }
        self.state = ControllerLifecycleState::Active;
        Ok(())
    }

    /// Resets every active controller in stable ID order.
    pub fn reset(
        &mut self,
        context: ControllerResetContext,
    ) -> Result<(), ControllerScheduleError> {
        self.require_state("reset", ControllerLifecycleState::Active)?;
        for (controller_id, controller) in &mut self.controllers {
            controller.host.reset(context).map_err(|source| {
                ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source,
                }
            })?;
        }
        Ok(())
    }

    /// Steps controllers in stable ID order and merges their canonical actions.
    pub fn step(
        &mut self,
        observation: &ControllerObservationFrame,
    ) -> Result<ControllerActionFrame, ControllerScheduleError> {
        self.require_state("step", ControllerLifecycleState::Active)?;
        observation.validate()?;
        let mut commands =
            BTreeMap::<(String, String), (String, ControllerJointVelocityCommand)>::new();

        for (controller_id, controller) in &mut self.controllers {
            let mut robots = Vec::with_capacity(controller.robot_ids.len());
            for robot_id in &controller.robot_ids {
                let robot = observation.robot(robot_id).ok_or_else(|| {
                    ControllerScheduleError::UnknownRobot {
                        controller_id: controller_id.clone(),
                        robot_id: robot_id.clone(),
                    }
                })?;
                robots.push(robot.clone());
            }
            let controller_observation = ControllerObservationFrame::new(
                observation.step,
                observation.sim_time_ticks,
                robots,
            )?;
            let action = controller
                .host
                .step(&controller_observation)
                .map_err(|source| ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source,
                })?;
            for robot in action.robots {
                for command in robot.joint_velocities {
                    let key = (robot.robot_id.clone(), command.name.clone());
                    if let Some((first_controller_id, _)) = commands.get(&key) {
                        return Err(ControllerScheduleError::CommandConflict {
                            robot_id: key.0,
                            joint: key.1,
                            first_controller_id: first_controller_id.clone(),
                            second_controller_id: controller_id.clone(),
                        });
                    }
                    commands.insert(key, (controller_id.clone(), command));
                }
            }
        }

        let mut robot_commands = BTreeMap::<String, Vec<ControllerJointVelocityCommand>>::new();
        for ((robot_id, _), (_, command)) in commands {
            robot_commands.entry(robot_id).or_default().push(command);
        }
        let robot_actions = robot_commands
            .into_iter()
            .map(|(robot_id, commands)| ControllerRobotAction::new(robot_id, commands))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ControllerActionFrame::new(observation.step, robot_actions)?)
    }

    /// Shuts down every controller in stable ID order.
    pub fn shutdown(&mut self) -> Result<(), ControllerScheduleError> {
        if self.state == ControllerLifecycleState::Shutdown {
            return Err(ControllerScheduleError::InvalidTransition {
                operation: "shutdown",
                state: self.state,
            });
        }
        for (controller_id, controller) in &mut self.controllers {
            controller
                .host
                .shutdown()
                .map_err(|source| ControllerScheduleError::Controller {
                    controller_id: controller_id.clone(),
                    source,
                })?;
        }
        self.state = ControllerLifecycleState::Shutdown;
        Ok(())
    }

    fn require_state(
        &self,
        operation: &'static str,
        required: ControllerLifecycleState,
    ) -> Result<(), ControllerScheduleError> {
        if self.state != required {
            return Err(ControllerScheduleError::InvalidTransition {
                operation,
                state: self.state,
            });
        }
        Ok(())
    }
}

/// Deterministic scheduler registration, lifecycle, or merge failure.
#[derive(Debug, thiserror::Error)]
pub enum ControllerScheduleError {
    /// Scheduler data violates an invariant.
    #[error("invalid controller schedule: {0}")]
    Invalid(String),
    /// A stable controller ID was registered more than once.
    #[error("duplicate scheduled controller `{0}`")]
    DuplicateController(String),
    /// A scheduled robot is absent from the current observation frame.
    #[error("controller `{controller_id}` targets unknown robot `{robot_id}`")]
    UnknownRobot {
        /// Stable controller identity.
        controller_id: String,
        /// Stable robot identity.
        robot_id: String,
    },
    /// Two controllers produced commands for the same robot joint.
    #[error(
        "controllers `{first_controller_id}` and `{second_controller_id}` both command `{robot_id}/{joint}`"
    )]
    CommandConflict {
        /// Stable robot identity.
        robot_id: String,
        /// Stable joint name.
        joint: String,
        /// First controller in scheduling order.
        first_controller_id: String,
        /// Later conflicting controller.
        second_controller_id: String,
    },
    /// The scheduler operation was requested from the wrong lifecycle state.
    #[error("controller scheduler operation `{operation}` is invalid while {state:?}")]
    InvalidTransition {
        /// Requested scheduler operation.
        operation: &'static str,
        /// Current scheduler state.
        state: ControllerLifecycleState,
    },
    /// One named controller failed negotiation, lifecycle, or stepping.
    #[error("controller `{controller_id}` failed: {source}")]
    Controller {
        /// Stable controller identity.
        controller_id: String,
        /// Underlying host failure.
        #[source]
        source: ControllerLifecycleError,
    },
    /// Observation/action schema validation failed.
    #[error(transparent)]
    Schema(#[from] ControllerSchemaError),
}

fn validate_id(field: &str, value: &str) -> Result<(), ControllerScheduleError> {
    if value.trim().is_empty() || value.contains('\0') {
        return Err(ControllerScheduleError::Invalid(format!(
            "{field} must be non-empty and NUL-free"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ControllerJointObservation, ControllerRobotObservation, VelocityServoController};

    fn robot(robot_id: &str, position_rad: f64) -> ControllerRobotObservation {
        ControllerRobotObservation::new(
            robot_id,
            vec![ControllerJointObservation::position(
                "shoulder_joint",
                position_rad,
            )],
        )
        .unwrap()
    }

    fn scheduler(robot_ids: &[&str]) -> ControllerScheduler {
        let mut scheduler = ControllerScheduler::new();
        scheduler
            .register(
                "servo",
                Box::new(
                    VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
                        .unwrap(),
                ),
                robot_ids.iter().map(|id| (*id).to_string()),
            )
            .unwrap();
        scheduler.configure().unwrap();
        scheduler
            .activate(ControllerResetContext {
                episode: 0,
                seed: 7,
                step: 0,
                sim_time_ticks: 0,
            })
            .unwrap();
        scheduler
    }

    #[test]
    fn reversed_robot_input_produces_byte_identical_actions_and_named_state() {
        let mut first = scheduler(&["robot_b", "robot_a"]);
        let mut second = scheduler(&["robot_a", "robot_b"]);
        let first_observation = ControllerObservationFrame::new(
            0,
            0,
            vec![robot("robot_b", -0.5), robot("robot_a", 0.25)],
        )
        .unwrap();
        let second_observation = ControllerObservationFrame::new(
            0,
            0,
            vec![robot("robot_a", 0.25), robot("robot_b", -0.5)],
        )
        .unwrap();

        let first_action = first.step(&first_observation).unwrap();
        let second_action = second.step(&second_observation).unwrap();
        assert_eq!(
            first_action.to_json_pretty().unwrap(),
            second_action.to_json_pretty().unwrap()
        );

        let advance_named_state =
            |observation: &ControllerObservationFrame, action: &ControllerActionFrame| {
                let mut state = BTreeMap::new();
                for robot in &observation.robots {
                    state.insert(robot.robot_id.clone(), robot.joints[0].position_rad);
                }
                for robot in &action.robots {
                    *state.get_mut(&robot.robot_id).unwrap() +=
                        robot.joint_velocities[0].velocity_rad_s * 0.01;
                }
                state
            };
        assert_eq!(
            advance_named_state(&first_observation, &first_action),
            advance_named_state(&second_observation, &second_action)
        );
    }

    #[test]
    fn multi_robot_assignment_requires_declared_capability() {
        #[derive(Debug)]
        struct LegacyController;

        impl ControllerPlugin for LegacyController {
            fn name(&self) -> &str {
                "legacy"
            }

            fn joint_velocity_commands(
                &self,
                _joint_names: &[&str],
                _positions_rad: &[f64],
            ) -> Vec<(String, f64)> {
                Vec::new()
            }
        }

        let mut scheduler = ControllerScheduler::new();
        scheduler
            .register(
                "legacy",
                Box::new(LegacyController),
                ["robot_a".to_string(), "robot_b".to_string()],
            )
            .unwrap();
        assert!(matches!(
            scheduler.configure(),
            Err(ControllerScheduleError::Controller {
                source: ControllerLifecycleError::MissingCapabilities(_),
                ..
            })
        ));
    }

    #[test]
    fn conflicting_commands_are_rejected_in_controller_id_order() {
        let mut scheduler = ControllerScheduler::new();
        for controller_id in ["second", "first"] {
            scheduler
                .register(
                    controller_id,
                    Box::new(
                        VelocityServoController::new(
                            controller_id,
                            "shoulder_joint",
                            1.0,
                            1.0,
                            5.0,
                        )
                        .unwrap(),
                    ),
                    ["robot".to_string()],
                )
                .unwrap();
        }
        scheduler.configure().unwrap();
        scheduler
            .activate(ControllerResetContext {
                episode: 0,
                seed: 0,
                step: 0,
                sim_time_ticks: 0,
            })
            .unwrap();
        let observation = ControllerObservationFrame::new(0, 0, vec![robot("robot", 0.0)]).unwrap();
        let error = scheduler.step(&observation).expect_err("command conflict");
        assert!(matches!(
            error,
            ControllerScheduleError::CommandConflict {
                first_controller_id,
                second_controller_id,
                ..
            } if first_controller_id == "first" && second_controller_id == "second"
        ));
    }
}
