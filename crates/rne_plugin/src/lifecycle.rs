//! Controller capability negotiation and host-owned lifecycle state.

use crate::{
    ControllerActionFrame, ControllerObservationFrame, ControllerPlugin, ControllerSchemaError,
    CONTROLLER_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// One capability supported or required by a robot-native controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControllerCapability {
    /// Named joint positions are accepted in observation frames.
    JointPositionObservation,
    /// Named joint velocities are accepted when present in observation frames.
    JointVelocityObservation,
    /// Named joint-velocity commands can be produced.
    JointVelocityCommand,
    /// Several stable robot IDs can be processed in one fixed-step frame.
    MultiRobot,
}

/// Versioned identity and canonical capability set for one controller.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerDescriptor {
    /// Observation/action schema version consumed by this controller.
    pub schema_version: u32,
    /// Stable controller name.
    pub name: String,
    /// Sorted, unique supported capabilities.
    pub capabilities: Vec<ControllerCapability>,
}

impl ControllerDescriptor {
    /// Creates and validates a descriptor, canonicalizing its capabilities.
    pub fn new(
        name: impl Into<String>,
        capabilities: impl IntoIterator<Item = ControllerCapability>,
    ) -> Result<Self, ControllerLifecycleError> {
        let descriptor = Self {
            schema_version: CONTROLLER_SCHEMA_VERSION,
            name: name.into(),
            capabilities: capabilities
                .into_iter()
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect(),
        };
        descriptor.validate()?;
        Ok(descriptor)
    }

    /// Validates schema compatibility, identity, and canonical ordering.
    pub fn validate(&self) -> Result<(), ControllerLifecycleError> {
        if self.schema_version != CONTROLLER_SCHEMA_VERSION {
            return Err(ControllerLifecycleError::UnsupportedSchema {
                expected: CONTROLLER_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        validate_name("controller name", &self.name)?;
        if self.capabilities.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err(ControllerLifecycleError::Invalid(
                "controller capabilities must be sorted and unique".to_string(),
            ));
        }
        Ok(())
    }

    /// Returns true when this controller supports a capability.
    pub fn supports(&self, capability: ControllerCapability) -> bool {
        self.capabilities.binary_search(&capability).is_ok()
    }
}

/// Versioned configuration requested by the simulation host.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerConfiguration {
    /// Observation/action schema version required by the host.
    pub schema_version: u32,
    /// Sorted, unique capabilities required by the scenario.
    pub required_capabilities: Vec<ControllerCapability>,
}

impl ControllerConfiguration {
    /// Creates a configuration with a canonical required-capability set.
    pub fn new(required_capabilities: impl IntoIterator<Item = ControllerCapability>) -> Self {
        Self {
            schema_version: CONTROLLER_SCHEMA_VERSION,
            required_capabilities: required_capabilities
                .into_iter()
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect(),
        }
    }

    /// Validates schema compatibility and canonical capability ordering.
    pub fn validate(&self) -> Result<(), ControllerLifecycleError> {
        if self.schema_version != CONTROLLER_SCHEMA_VERSION {
            return Err(ControllerLifecycleError::UnsupportedSchema {
                expected: CONTROLLER_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self
            .required_capabilities
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(ControllerLifecycleError::Invalid(
                "required controller capabilities must be sorted and unique".to_string(),
            ));
        }
        Ok(())
    }
}

/// Successful capability negotiation passed to the controller configure hook.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerNegotiation {
    /// Descriptor accepted by the host.
    pub controller: ControllerDescriptor,
    /// Configuration whose requirements were satisfied.
    pub configuration: ControllerConfiguration,
}

/// Deterministic episode reset metadata passed to a controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerResetContext {
    /// Monotonic episode index owned by the runner.
    pub episode: u64,
    /// Explicit deterministic world seed.
    pub seed: u64,
    /// Fixed simulation step at which reset becomes visible.
    pub step: u64,
    /// Simulation timestamp represented as stable integer ticks.
    pub sim_time_ticks: u64,
}

/// Host-owned lifecycle state for one controller instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControllerLifecycleState {
    /// Plugin instance exists but has not negotiated capabilities.
    Created,
    /// Capabilities are accepted but no episode is active.
    Configured,
    /// An episode is active and fixed-step calls are allowed.
    Active,
    /// The terminal shutdown hook has been invoked.
    Shutdown,
}

/// Error reported by controller-owned hooks or typed step execution.
#[derive(Debug, thiserror::Error)]
pub enum ControllerPluginError {
    /// The plugin rejected a lifecycle operation.
    #[error("controller rejected operation: {0}")]
    Rejected(String),
    /// The plugin produced or consumed an invalid schema frame.
    #[error(transparent)]
    Schema(#[from] ControllerSchemaError),
}

/// Capability negotiation, lifecycle, or controller-step failure.
#[derive(Debug, thiserror::Error)]
pub enum ControllerLifecycleError {
    /// The descriptor or configuration uses an unsupported control schema.
    #[error("unsupported controller schema: expected {expected}, got {actual}")]
    UnsupportedSchema {
        /// Schema version supported by this runtime.
        expected: u32,
        /// Schema version requested by the controller or host.
        actual: u32,
    },
    /// A lifecycle value violates an invariant.
    #[error("invalid controller lifecycle data: {0}")]
    Invalid(String),
    /// The controller does not support every scenario requirement.
    #[error("controller is missing required capabilities: {0:?}")]
    MissingCapabilities(Vec<ControllerCapability>),
    /// An operation was requested from the wrong lifecycle state.
    #[error("controller operation `{operation}` is invalid while {state:?}")]
    InvalidTransition {
        /// Requested lifecycle operation.
        operation: &'static str,
        /// Current host-owned lifecycle state.
        state: ControllerLifecycleState,
    },
    /// A plugin hook failed.
    #[error(transparent)]
    Plugin(#[from] ControllerPluginError),
    /// Observation/action validation failed at the host boundary.
    #[error(transparent)]
    Schema(#[from] ControllerSchemaError),
}

/// Host wrapper that enforces negotiation, lifecycle, and frame validation.
#[derive(Debug)]
pub struct ControllerHost {
    plugin: Box<dyn ControllerPlugin>,
    descriptor: ControllerDescriptor,
    negotiation: Option<ControllerNegotiation>,
    state: ControllerLifecycleState,
}

impl ControllerHost {
    /// Creates a host in the [`ControllerLifecycleState::Created`] state.
    pub fn new(plugin: Box<dyn ControllerPlugin>) -> Result<Self, ControllerLifecycleError> {
        let descriptor = ControllerDescriptor::new(plugin.name(), plugin.capabilities())?;
        Ok(Self {
            plugin,
            descriptor,
            negotiation: None,
            state: ControllerLifecycleState::Created,
        })
    }

    /// Returns the validated controller descriptor.
    pub fn descriptor(&self) -> &ControllerDescriptor {
        &self.descriptor
    }

    /// Returns the current host-owned lifecycle state.
    pub fn state(&self) -> ControllerLifecycleState {
        self.state
    }

    /// Returns the accepted negotiation after successful configuration.
    pub fn negotiation(&self) -> Option<&ControllerNegotiation> {
        self.negotiation.as_ref()
    }

    /// Negotiates capabilities and invokes the plugin configure hook.
    pub fn configure(
        &mut self,
        configuration: ControllerConfiguration,
    ) -> Result<&ControllerNegotiation, ControllerLifecycleError> {
        self.require_state("configure", ControllerLifecycleState::Created)?;
        configuration.validate()?;
        let missing = configuration
            .required_capabilities
            .iter()
            .copied()
            .filter(|capability| !self.descriptor.supports(*capability))
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            return Err(ControllerLifecycleError::MissingCapabilities(missing));
        }
        let negotiation = ControllerNegotiation {
            controller: self.descriptor.clone(),
            configuration,
        };
        self.plugin.on_configure(&negotiation)?;
        self.negotiation = Some(negotiation);
        self.state = ControllerLifecycleState::Configured;
        Ok(self.negotiation.as_ref().expect("just configured"))
    }

    /// Activates the first deterministic episode through the reset hook.
    pub fn activate(
        &mut self,
        context: ControllerResetContext,
    ) -> Result<(), ControllerLifecycleError> {
        self.require_state("activate", ControllerLifecycleState::Configured)?;
        self.plugin.on_reset(context)?;
        self.state = ControllerLifecycleState::Active;
        Ok(())
    }

    /// Resets an active controller for another deterministic episode.
    pub fn reset(
        &mut self,
        context: ControllerResetContext,
    ) -> Result<(), ControllerLifecycleError> {
        self.require_state("reset", ControllerLifecycleState::Active)?;
        self.plugin.on_reset(context)?;
        Ok(())
    }

    /// Executes one fixed-step callback and validates the returned action.
    pub fn step(
        &mut self,
        observation: &ControllerObservationFrame,
    ) -> Result<ControllerActionFrame, ControllerLifecycleError> {
        self.require_state("step", ControllerLifecycleState::Active)?;
        observation.validate()?;
        let action = self.plugin.step_frame(observation)?;
        action.validate_against(observation)?;
        Ok(action)
    }

    /// Invokes the terminal shutdown hook exactly once.
    pub fn shutdown(&mut self) -> Result<(), ControllerLifecycleError> {
        if self.state == ControllerLifecycleState::Shutdown {
            return Err(ControllerLifecycleError::InvalidTransition {
                operation: "shutdown",
                state: self.state,
            });
        }
        self.plugin.on_shutdown()?;
        self.state = ControllerLifecycleState::Shutdown;
        Ok(())
    }

    fn require_state(
        &self,
        operation: &'static str,
        required: ControllerLifecycleState,
    ) -> Result<(), ControllerLifecycleError> {
        if self.state != required {
            return Err(ControllerLifecycleError::InvalidTransition {
                operation,
                state: self.state,
            });
        }
        Ok(())
    }
}

fn validate_name(field: &str, value: &str) -> Result<(), ControllerLifecycleError> {
    if value.trim().is_empty() || value.contains('\0') {
        return Err(ControllerLifecycleError::Invalid(format!(
            "{field} must be non-empty and NUL-free"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ControllerJointObservation, ControllerJointVelocityCommand, ControllerRobotAction,
        ControllerRobotObservation,
    };
    use std::sync::{Arc, Mutex};

    #[derive(Debug, Default)]
    struct HookCounts {
        configure: u32,
        reset: u32,
        shutdown: u32,
    }

    #[derive(Debug)]
    struct TestController {
        hooks: Arc<Mutex<HookCounts>>,
        multi_robot: bool,
    }

    impl ControllerPlugin for TestController {
        fn name(&self) -> &str {
            "test_controller"
        }

        fn capabilities(&self) -> Vec<ControllerCapability> {
            let mut capabilities = vec![
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
            ];
            if self.multi_robot {
                capabilities.push(ControllerCapability::MultiRobot);
            }
            capabilities
        }

        fn joint_velocity_commands(
            &self,
            joint_names: &[&str],
            _positions_rad: &[f64],
        ) -> Vec<(String, f64)> {
            joint_names
                .first()
                .map(|name| vec![(name.to_string(), 0.5)])
                .unwrap_or_default()
        }

        fn on_configure(
            &mut self,
            _negotiation: &ControllerNegotiation,
        ) -> Result<(), ControllerPluginError> {
            self.hooks.lock().unwrap().configure += 1;
            Ok(())
        }

        fn on_reset(
            &mut self,
            _context: ControllerResetContext,
        ) -> Result<(), ControllerPluginError> {
            self.hooks.lock().unwrap().reset += 1;
            Ok(())
        }

        fn on_shutdown(&mut self) -> Result<(), ControllerPluginError> {
            self.hooks.lock().unwrap().shutdown += 1;
            Ok(())
        }
    }

    fn observation() -> ControllerObservationFrame {
        ControllerObservationFrame::new(
            3,
            30,
            vec![ControllerRobotObservation::new(
                "robot",
                vec![ControllerJointObservation::position("joint", 0.0)],
            )
            .unwrap()],
        )
        .unwrap()
    }

    fn reset_context() -> ControllerResetContext {
        ControllerResetContext {
            episode: 0,
            seed: 42,
            step: 0,
            sim_time_ticks: 0,
        }
    }

    #[test]
    fn host_enforces_lifecycle_and_invokes_hooks_once() {
        let hooks = Arc::new(Mutex::new(HookCounts::default()));
        let controller = TestController {
            hooks: hooks.clone(),
            multi_robot: false,
        };
        let mut host = ControllerHost::new(Box::new(controller)).unwrap();
        assert_eq!(host.state(), ControllerLifecycleState::Created);
        assert!(matches!(
            host.step(&observation()),
            Err(ControllerLifecycleError::InvalidTransition { .. })
        ));

        host.configure(ControllerConfiguration::new([
            ControllerCapability::JointPositionObservation,
            ControllerCapability::JointVelocityCommand,
        ]))
        .unwrap();
        host.activate(reset_context()).unwrap();
        let action = host.step(&observation()).unwrap();
        assert_eq!(action.robots[0].joint_velocities[0].velocity_rad_s, 0.5);
        host.reset(ControllerResetContext {
            episode: 1,
            ..reset_context()
        })
        .unwrap();
        host.shutdown().unwrap();
        assert_eq!(host.state(), ControllerLifecycleState::Shutdown);
        assert!(host.shutdown().is_err());

        let hooks = hooks.lock().unwrap();
        assert_eq!(hooks.configure, 1);
        assert_eq!(hooks.reset, 2);
        assert_eq!(hooks.shutdown, 1);
    }

    #[test]
    fn missing_capability_fails_before_configure_hook() {
        let hooks = Arc::new(Mutex::new(HookCounts::default()));
        let controller = TestController {
            hooks: hooks.clone(),
            multi_robot: false,
        };
        let mut host = ControllerHost::new(Box::new(controller)).unwrap();
        let error = host
            .configure(ControllerConfiguration::new([
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
                ControllerCapability::MultiRobot,
            ]))
            .expect_err("missing multi-robot support");
        assert!(matches!(
            error,
            ControllerLifecycleError::MissingCapabilities(_)
        ));
        assert_eq!(host.state(), ControllerLifecycleState::Created);
        assert_eq!(hooks.lock().unwrap().configure, 0);
    }

    #[test]
    fn host_rejects_plugin_action_that_does_not_match_observation() {
        #[derive(Debug)]
        struct BadController;

        impl ControllerPlugin for BadController {
            fn name(&self) -> &str {
                "bad"
            }

            fn joint_velocity_commands(
                &self,
                _joint_names: &[&str],
                _positions_rad: &[f64],
            ) -> Vec<(String, f64)> {
                Vec::new()
            }

            fn step_frame(
                &mut self,
                observation: &ControllerObservationFrame,
            ) -> Result<ControllerActionFrame, ControllerPluginError> {
                Ok(ControllerActionFrame::new(
                    observation.step,
                    vec![ControllerRobotAction::new(
                        "robot",
                        vec![ControllerJointVelocityCommand::new("unknown", 1.0)],
                    )?],
                )?)
            }
        }

        let mut host = ControllerHost::new(Box::new(BadController)).unwrap();
        host.configure(ControllerConfiguration::new([
            ControllerCapability::JointPositionObservation,
            ControllerCapability::JointVelocityCommand,
        ]))
        .unwrap();
        host.activate(reset_context()).unwrap();
        assert!(matches!(
            host.step(&observation()),
            Err(ControllerLifecycleError::Schema(
                ControllerSchemaError::UnknownJoint { .. }
            ))
        ));
    }
}
