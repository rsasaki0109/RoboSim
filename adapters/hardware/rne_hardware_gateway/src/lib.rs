//! Bounded, fail-closed contracts for hardware, shadow, and HIL adapters.
//!
//! This crate is deliberately outside RNE core. The gateway state machine uses an
//! injected monotonic host tick and never reads a wall clock itself. A process or
//! vendor adapter owns I/O and supplies those ticks, while [`TaskSpec`] remains the
//! shared observation/action contract.

#![deny(missing_docs)]

pub mod conformance;
pub mod mock;
pub mod shadow;
pub mod wire;

use rne_ai::{TaskSpec, TaskSpecValidationError, TensorDType, TensorSpec};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Schema version for snapshots and events produced by this gateway contract.
pub const HARDWARE_GATEWAY_SCHEMA_VERSION: u32 = 1;

/// Stable artifact discriminator for [`GatewayEvidence`].
pub const HARDWARE_GATEWAY_EVIDENCE_KIND: &str = "rne_hardware_gateway_evidence";

/// Execution authority granted to a hardware gateway session.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HardwareMode {
    /// Consume a recorded stream without a live connection or actuator output.
    Playback,
    /// Observe a live device and evaluate actions without sending them.
    Shadow,
    /// Exchange observations and bounded actions with a hardware-in-the-loop rig.
    Hil,
    /// Exchange observations and bounded actions with a physical robot.
    Live,
}

impl HardwareMode {
    fn can_actuate(self) -> bool {
        matches!(self, Self::Hil | Self::Live)
    }
}

/// Host-side timing and memory bounds for one gateway session.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GatewayConfig {
    /// Session authority mode.
    pub mode: HardwareMode,
    /// Maximum accepted observation age in monotonic host milliseconds.
    pub max_observation_age_ms: u64,
    /// Maximum delay from an observation arriving to its action being submitted.
    pub command_deadline_ms: u64,
    /// Maximum time an accepted action may remain undelivered or unrefreshed.
    pub max_command_age_ms: u64,
    /// Maximum retained observations.
    pub observation_capacity: usize,
    /// Maximum pending actuator writes, including a fail-closed stop.
    pub actuation_capacity: usize,
    /// Maximum retained audit events.
    pub event_capacity: usize,
}

impl GatewayConfig {
    /// Validates non-zero timing and queue bounds.
    pub fn validate(self) -> Result<(), GatewayBuildError> {
        for (field, value) in [
            ("max_observation_age_ms", self.max_observation_age_ms),
            ("command_deadline_ms", self.command_deadline_ms),
            ("max_command_age_ms", self.max_command_age_ms),
        ] {
            if value == 0 {
                return Err(GatewayBuildError::InvalidConfig {
                    field,
                    reason: "must be greater than zero",
                });
            }
        }
        for (field, value) in [
            ("observation_capacity", self.observation_capacity),
            ("actuation_capacity", self.actuation_capacity),
            ("event_capacity", self.event_capacity),
        ] {
            if value == 0 {
                return Err(GatewayBuildError::InvalidConfig {
                    field,
                    reason: "must be greater than zero",
                });
            }
        }
        Ok(())
    }
}

impl Default for GatewayConfig {
    fn default() -> Self {
        Self {
            mode: HardwareMode::Shadow,
            max_observation_age_ms: 100,
            command_deadline_ms: 20,
            max_command_age_ms: 100,
            observation_capacity: 4,
            actuation_capacity: 2,
            event_capacity: 128,
        }
    }
}

/// One flattened actuator limit inherited from the bound [`TaskSpec`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActionElementLimit {
    /// Action tensor name.
    pub tensor_name: String,
    /// Row-major element index within that tensor.
    pub tensor_element: usize,
    /// Unit declared by the task.
    pub unit: String,
    /// Scalar dtype declared by the task.
    pub dtype: TensorDType,
    /// Inclusive lower limit.
    pub lower: f64,
    /// Inclusive upper limit.
    pub upper: f64,
}

/// One observation received from a hardware or playback adapter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareObservation {
    /// Connection-local, strictly increasing sequence.
    pub sequence: u64,
    /// Monotonic host tick assigned by the gateway process.
    pub received_at_ms: u64,
    /// Flattened values in TaskSpec tensor and row-major order.
    pub values: Vec<f64>,
}

/// One validated controller action awaiting delivery.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareAction {
    /// Connection-local, strictly increasing action sequence.
    pub sequence: u64,
    /// Observation sequence used to compute this action.
    pub observation_sequence: u64,
    /// Monotonic host tick when the gateway accepted the action.
    pub accepted_at_ms: u64,
    /// Flattened values in TaskSpec tensor and row-major order.
    pub values: Vec<f64>,
}

/// Reason the gateway suppressed or replaced actuator output.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafetyReason {
    /// The hardware transport disconnected.
    Disconnected,
    /// The newest observation exceeded its configured age.
    ObservationStale,
    /// No action arrived before the observation-to-command deadline.
    CommandDeadlineMissed,
    /// A queued or active command exceeded its configured age.
    CommandStale,
    /// At least one action value exceeded its TaskSpec limit.
    ActuatorLimit,
    /// The pending actuator queue could not accept another command.
    QueueOverrun,
    /// The adapter supplied a decreasing monotonic host tick.
    ClockRegression,
    /// An operator or hardware input asserted emergency stop.
    EmergencyStop,
    /// The host controller failed to produce a valid action contract.
    ControllerFault,
    /// An operator deliberately disarmed the session.
    ManualDisarm,
}

/// One actuator write returned to the transport owner.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuationFrame {
    /// Source action sequence, or `None` for a gateway-generated safe stop.
    pub action_sequence: Option<u64>,
    /// Monotonic host tick when this frame entered the bounded queue.
    pub queued_at_ms: u64,
    /// Flattened actuator values.
    pub values: Vec<f64>,
    /// True when values were generated by fail-closed behavior.
    pub safety_stop: bool,
    /// Stop reason when `safety_stop` is true.
    pub reason: Option<SafetyReason>,
}

/// Result of accepting a controller action.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandDisposition {
    /// The bounded transport queue owns an actuator write.
    Queued,
    /// Playback or shadow mode validated but suppressed the actuator write.
    Suppressed,
}

/// Externally visible gateway connection state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GatewayConnectionState {
    /// No hardware transport is connected.
    Disconnected,
    /// A transport is connected without actuator authority.
    Connected,
    /// A connected HIL or live transport has actuator authority.
    Armed,
}

/// Bounded audit event emitted by the gateway state machine.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum GatewayEvent {
    /// A transport connected.
    Connected {
        /// Monotonic host tick.
        host_time_ms: u64,
    },
    /// A transport disconnected.
    Disconnected {
        /// Monotonic host tick.
        host_time_ms: u64,
    },
    /// The session gained actuator authority.
    Armed {
        /// Monotonic host tick.
        host_time_ms: u64,
    },
    /// The session deliberately relinquished actuator authority.
    Disarmed {
        /// Monotonic host tick.
        host_time_ms: u64,
    },
    /// A safety latch was explicitly cleared.
    SafetyCleared {
        /// Monotonic host tick.
        host_time_ms: u64,
    },
    /// A safety condition removed authority and queued a stop when applicable.
    SafetyTripped {
        /// Monotonic host tick.
        host_time_ms: u64,
        /// Safety condition.
        reason: SafetyReason,
    },
    /// One observation entered the bounded queue.
    ObservationAccepted {
        /// Observation sequence.
        sequence: u64,
    },
    /// The oldest observation was discarded to preserve the configured bound.
    ObservationDropped {
        /// Discarded observation sequence.
        sequence: u64,
    },
    /// One action entered the bounded actuator queue.
    ActionQueued {
        /// Action sequence.
        sequence: u64,
    },
    /// A non-actuating mode suppressed one otherwise valid action.
    ActionSuppressed {
        /// Action sequence.
        sequence: u64,
        /// Session mode.
        mode: HardwareMode,
    },
    /// The transport owner received one actuator frame.
    ActuationDelivered {
        /// Source action sequence, absent for a safe stop.
        action_sequence: Option<u64>,
        /// Whether the delivered frame is a safe stop.
        safety_stop: bool,
    },
}

/// Serializable, bounded status for diagnostics and evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GatewaySnapshot {
    /// Snapshot schema version.
    pub schema_version: u32,
    /// Bound TaskSpec identity.
    pub task_id: String,
    /// Session mode.
    pub mode: HardwareMode,
    /// Current connection and authority state.
    pub connection_state: GatewayConnectionState,
    /// Latched safety condition, if any.
    pub safety_latch: Option<SafetyReason>,
    /// Retained observation count.
    pub queued_observations: usize,
    /// Pending actuator write count.
    pub queued_actuations: usize,
    /// Audit events dropped from the bounded ring.
    pub dropped_events: u64,
    /// Observations dropped from the bounded ring.
    pub dropped_observations: u64,
    /// Most recent accepted observation sequence.
    pub last_observation_sequence: Option<u64>,
    /// Most recent accepted or suppressed action sequence.
    pub last_action_sequence: Option<u64>,
}

/// Versioned audit artifact for one bounded gateway session.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GatewayEvidence {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Evidence schema version.
    pub schema_version: u32,
    /// Bound TaskSpec identity.
    pub task_id: String,
    /// Session authority mode.
    pub mode: HardwareMode,
    /// Retained audit events in occurrence order.
    pub events: Vec<GatewayEvent>,
    /// Final bounded gateway status.
    pub final_snapshot: GatewaySnapshot,
}

/// Failure constructing a gateway from a portable task contract.
#[derive(Debug, thiserror::Error)]
pub enum GatewayBuildError {
    /// The TaskSpec is invalid or unsupported.
    #[error(transparent)]
    Task(#[from] TaskSpecValidationError),
    /// A timing or queue bound is zero.
    #[error("invalid gateway config {field}: {reason}")]
    InvalidConfig {
        /// Config field.
        field: &'static str,
        /// Failed invariant.
        reason: &'static str,
    },
    /// This v1 gateway represents hardware values as floating-point arrays only.
    #[error("unsupported {space} tensor dtype for {tensor:?}")]
    UnsupportedTensorDType {
        /// Observation or action space.
        space: &'static str,
        /// Tensor name.
        tensor: String,
    },
    /// A tensor shape overflowed the host index type.
    #[error("{space} tensor element count overflowed for {tensor:?}")]
    TensorElementCountOverflow {
        /// Observation or action space.
        space: &'static str,
        /// Tensor name.
        tensor: String,
    },
    /// Hardware actions must carry explicit finite TaskSpec limits.
    #[error("action tensor {tensor:?} must declare bounds for hardware execution")]
    MissingActionBounds {
        /// Tensor name.
        tensor: String,
    },
}

/// Runtime rejection from the bounded gateway state machine.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum GatewayError {
    /// A connection operation conflicts with current state.
    #[error("hardware transport is already connected")]
    AlreadyConnected,
    /// The operation requires a connected transport.
    #[error("hardware transport is disconnected")]
    Disconnected,
    /// The operation requires actuator authority.
    #[error("hardware gateway is not armed")]
    NotArmed,
    /// Playback and shadow sessions cannot be armed.
    #[error("mode {0:?} cannot acquire actuator authority")]
    NonActuatingMode(HardwareMode),
    /// A latched safety condition must be cleared explicitly.
    #[error("hardware safety latch is active: {0:?}")]
    SafetyLatched(SafetyReason),
    /// A fresh observation is required before arming or commanding.
    #[error("no fresh hardware observation is available")]
    NoFreshObservation,
    /// The injected host clock moved backward.
    #[error("monotonic host tick regressed from {previous_ms} to {current_ms}")]
    ClockRegression {
        /// Previously accepted tick.
        previous_ms: u64,
        /// Rejected tick.
        current_ms: u64,
    },
    /// An observation or action sequence did not increase.
    #[error("{stream} sequence {actual} must be greater than {previous}")]
    NonMonotonicSequence {
        /// Observation or action stream.
        stream: &'static str,
        /// Previous accepted sequence.
        previous: u64,
        /// Rejected sequence.
        actual: u64,
    },
    /// A flat payload does not match the TaskSpec width.
    #[error("{space} value count must be {expected}, got {actual}")]
    ValueCount {
        /// Observation or action space.
        space: &'static str,
        /// Required count.
        expected: usize,
        /// Received count.
        actual: usize,
    },
    /// A hardware value was NaN or infinite.
    #[error("{space} value {index} must be finite")]
    NonFiniteValue {
        /// Observation or action space.
        space: &'static str,
        /// Flattened value index.
        index: usize,
    },
    /// A normalized observation value does not represent its declared TaskSpec dtype.
    #[error("observation value {index} does not represent {dtype:?}")]
    ObservationDType {
        /// Flattened value index.
        index: usize,
        /// Required TaskSpec dtype.
        dtype: TensorDType,
    },
    /// A normalized action value cannot be represented by its TaskSpec dtype.
    #[error("action value {index} does not represent {dtype:?}")]
    ActionDType {
        /// Flattened value index.
        index: usize,
        /// Required TaskSpec dtype.
        dtype: TensorDType,
    },
    /// The action did not target the newest observation.
    #[error("action targets observation {actual}, newest observation is {expected}")]
    ObservationSequenceMismatch {
        /// Newest sequence.
        expected: u64,
        /// Action's sequence reference.
        actual: u64,
    },
    /// The newest observation exceeded the age policy.
    #[error("observation age {age_ms} ms exceeds {limit_ms} ms")]
    ObservationStale {
        /// Observed age.
        age_ms: u64,
        /// Configured maximum.
        limit_ms: u64,
    },
    /// The observation-to-command deadline expired.
    #[error("command delay {age_ms} ms exceeds deadline {limit_ms} ms")]
    CommandDeadlineMissed {
        /// Observed delay.
        age_ms: u64,
        /// Configured deadline.
        limit_ms: u64,
    },
    /// A command exceeded a TaskSpec actuator limit.
    #[error("action value {index}={value} is outside [{lower}, {upper}]")]
    ActuatorLimit {
        /// Flattened action index.
        index: usize,
        /// Rejected value.
        value: f64,
        /// Inclusive lower limit.
        lower: f64,
        /// Inclusive upper limit.
        upper: f64,
    },
    /// The bounded actuator queue is full.
    #[error("actuator queue is full")]
    ActuationQueueFull,
    /// A clean close requires authority to be relinquished first.
    #[error("hardware gateway must be disarmed before a clean close")]
    ArmedOnCleanClose,
    /// A clean close requires every queued actuator frame to be delivered.
    #[error("hardware gateway has pending actuation during clean close")]
    PendingActuationOnCleanClose,
}

/// Task-bound, fail-closed gateway state machine.
///
/// The owner supplies monotonic host milliseconds to every mutating operation.
/// Regressing time, stale data, missed deadlines, over-limit commands, queue
/// overruns, disconnects, and emergency stop remove actuator authority.
#[derive(Debug)]
pub struct HardwareGateway {
    task: TaskSpec,
    config: GatewayConfig,
    observation_width: usize,
    observation_dtypes: Vec<TensorDType>,
    action_limits: Vec<ActionElementLimit>,
    connected: bool,
    armed: bool,
    armed_at_ms: Option<u64>,
    safety_latch: Option<SafetyReason>,
    last_host_time_ms: Option<u64>,
    last_observation_sequence: Option<u64>,
    last_action_sequence: Option<u64>,
    last_action_observation_sequence: Option<u64>,
    last_action_time_ms: Option<u64>,
    observations: VecDeque<HardwareObservation>,
    actuations: VecDeque<ActuationFrame>,
    events: VecDeque<GatewayEvent>,
    dropped_events: u64,
    dropped_observations: u64,
}

impl HardwareGateway {
    /// Binds a validated TaskSpec and derives strict actuator limits from it.
    pub fn new(task: TaskSpec, config: GatewayConfig) -> Result<Self, GatewayBuildError> {
        task.validate()?;
        config.validate()?;
        let observation_dtypes = flatten_observation_dtypes(&task.observation.tensors)?;
        let observation_width = observation_dtypes.len();
        let action_limits = flatten_action_limits(&task.action.tensors)?;
        Ok(Self {
            task,
            config,
            observation_width,
            observation_dtypes,
            action_limits,
            connected: false,
            armed: false,
            armed_at_ms: None,
            safety_latch: None,
            last_host_time_ms: None,
            last_observation_sequence: None,
            last_action_sequence: None,
            last_action_observation_sequence: None,
            last_action_time_ms: None,
            observations: VecDeque::with_capacity(config.observation_capacity),
            actuations: VecDeque::with_capacity(config.actuation_capacity),
            events: VecDeque::with_capacity(config.event_capacity),
            dropped_events: 0,
            dropped_observations: 0,
        })
    }

    /// Returns the immutable portable task contract.
    pub fn task_spec(&self) -> &TaskSpec {
        &self.task
    }

    /// Returns the active gateway configuration.
    pub fn config(&self) -> GatewayConfig {
        self.config
    }

    /// Returns the expected flattened observation width.
    pub fn observation_width(&self) -> usize {
        self.observation_width
    }

    /// Returns the expected flattened action width.
    pub fn action_width(&self) -> usize {
        self.action_limits.len()
    }

    /// Returns TaskSpec-derived actuator limits in flattened action order.
    pub fn action_limits(&self) -> &[ActionElementLimit] {
        &self.action_limits
    }

    /// Returns a serializable bounded status snapshot.
    pub fn snapshot(&self) -> GatewaySnapshot {
        GatewaySnapshot {
            schema_version: HARDWARE_GATEWAY_SCHEMA_VERSION,
            task_id: self.task.task_id.clone(),
            mode: self.config.mode,
            connection_state: self.connection_state(),
            safety_latch: self.safety_latch,
            queued_observations: self.observations.len(),
            queued_actuations: self.actuations.len(),
            dropped_events: self.dropped_events,
            dropped_observations: self.dropped_observations,
            last_observation_sequence: self.last_observation_sequence,
            last_action_sequence: self.last_action_sequence,
        }
    }

    /// Returns the current connection and authority state.
    pub fn connection_state(&self) -> GatewayConnectionState {
        if !self.connected {
            GatewayConnectionState::Disconnected
        } else if self.armed {
            GatewayConnectionState::Armed
        } else {
            GatewayConnectionState::Connected
        }
    }

    /// Opens a new connection epoch.
    pub fn connect(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        if self.connected {
            return Err(GatewayError::AlreadyConnected);
        }
        self.connected = true;
        self.record_event(GatewayEvent::Connected {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Disconnects, removes authority, forgets connection-local sequences, and stops.
    pub fn disconnect(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        if !self.connected {
            return Err(GatewayError::Disconnected);
        }
        if self.config.mode.can_actuate() {
            self.trip_safety(SafetyReason::Disconnected, now_ms);
        }
        self.connected = false;
        self.armed = false;
        self.armed_at_ms = None;
        self.observations.clear();
        self.last_observation_sequence = None;
        self.last_action_sequence = None;
        self.last_action_observation_sequence = None;
        self.last_action_time_ms = None;
        self.record_event(GatewayEvent::Disconnected {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Completes an orderly transport close without latching a disconnect fault.
    ///
    /// Actuating sessions must first be disarmed and deliver the resulting safe
    /// stop. Any armed authority or pending actuator frame rejects the close.
    pub fn close_cleanly(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        if self.armed {
            return Err(GatewayError::ArmedOnCleanClose);
        }
        if !self.actuations.is_empty() {
            return Err(GatewayError::PendingActuationOnCleanClose);
        }
        if let Some(reason) = self.safety_latch {
            return Err(GatewayError::SafetyLatched(reason));
        }
        self.connected = false;
        self.armed_at_ms = None;
        self.observations.clear();
        self.last_observation_sequence = None;
        self.last_action_sequence = None;
        self.last_action_observation_sequence = None;
        self.last_action_time_ms = None;
        self.record_event(GatewayEvent::Disconnected {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Clears a safety latch without re-arming the session.
    pub fn clear_safety_latch(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        self.safety_latch = None;
        self.armed = false;
        self.armed_at_ms = None;
        self.actuations.clear();
        self.last_action_time_ms = None;
        self.last_action_observation_sequence = None;
        self.record_event(GatewayEvent::SafetyCleared {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Arms a live or HIL session after a fresh observation and cleared latch.
    pub fn arm(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        if !self.config.mode.can_actuate() {
            return Err(GatewayError::NonActuatingMode(self.config.mode));
        }
        if let Some(reason) = self.safety_latch {
            return Err(GatewayError::SafetyLatched(reason));
        }
        self.require_fresh_observation(now_ms)?;
        self.armed = true;
        self.armed_at_ms = Some(now_ms);
        self.record_event(GatewayEvent::Armed {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Deliberately removes authority and queues a zero action without latching a fault.
    pub fn disarm(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        self.armed = false;
        self.armed_at_ms = None;
        self.actuations.clear();
        if self.config.mode.can_actuate() {
            self.queue_safe_stop(SafetyReason::ManualDisarm, now_ms);
        }
        self.record_event(GatewayEvent::Disarmed {
            host_time_ms: now_ms,
        });
        Ok(())
    }

    /// Latches emergency stop and queues a zero action in an actuating mode.
    pub fn emergency_stop(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        self.trip_safety(SafetyReason::EmergencyStop, now_ms);
        Ok(())
    }

    /// Latches a host-controller failure and queues a zero action.
    pub fn controller_fault(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        self.trip_safety(SafetyReason::ControllerFault, now_ms);
        Ok(())
    }

    /// Mirrors a device-side safety signal into the gateway safety latch.
    ///
    /// Device adapters call this only after the peer has independently
    /// confirmed its stop. The gateway still queues its own zero stop in an
    /// actuating mode so the final snapshot records both safety layers.
    pub fn device_safety_signal(
        &mut self,
        reason: SafetyReason,
        now_ms: u64,
    ) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        self.trip_safety(reason, now_ms);
        Ok(())
    }

    /// Accepts one connection-local observation into the bounded ring.
    pub fn ingest_observation(
        &mut self,
        now_ms: u64,
        sequence: u64,
        mut values: Vec<f64>,
    ) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        normalize_observation_values(&self.observation_dtypes, &mut values)?;
        if let Some(previous) = self.last_observation_sequence {
            if sequence <= previous {
                return Err(GatewayError::NonMonotonicSequence {
                    stream: "observation",
                    previous,
                    actual: sequence,
                });
            }
        }
        if self.observations.len() == self.config.observation_capacity {
            if let Some(dropped) = self.observations.pop_front() {
                self.dropped_observations = self.dropped_observations.saturating_add(1);
                self.record_event(GatewayEvent::ObservationDropped {
                    sequence: dropped.sequence,
                });
            }
        }
        self.observations.push_back(HardwareObservation {
            sequence,
            received_at_ms: now_ms,
            values,
        });
        self.last_observation_sequence = Some(sequence);
        self.record_event(GatewayEvent::ObservationAccepted { sequence });
        Ok(())
    }

    /// Validates and either queues or suppresses one controller action.
    pub fn submit_action(
        &mut self,
        now_ms: u64,
        sequence: u64,
        observation_sequence: u64,
        mut values: Vec<f64>,
    ) -> Result<CommandDisposition, GatewayError> {
        self.advance_clock(now_ms)?;
        self.require_connected()?;
        normalize_action_values(&self.action_limits, &mut values)?;
        if let Some(previous) = self.last_action_sequence {
            if sequence <= previous {
                return Err(GatewayError::NonMonotonicSequence {
                    stream: "action",
                    previous,
                    actual: sequence,
                });
            }
        }
        let observation = self
            .observations
            .back()
            .ok_or(GatewayError::NoFreshObservation)?;
        if observation.sequence != observation_sequence {
            return Err(GatewayError::ObservationSequenceMismatch {
                expected: observation.sequence,
                actual: observation_sequence,
            });
        }
        let observation_age_ms = now_ms - observation.received_at_ms;
        if observation_age_ms > self.config.max_observation_age_ms {
            self.trip_safety(SafetyReason::ObservationStale, now_ms);
            return Err(GatewayError::ObservationStale {
                age_ms: observation_age_ms,
                limit_ms: self.config.max_observation_age_ms,
            });
        }
        if observation_age_ms > self.config.command_deadline_ms {
            self.trip_safety(SafetyReason::CommandDeadlineMissed, now_ms);
            return Err(GatewayError::CommandDeadlineMissed {
                age_ms: observation_age_ms,
                limit_ms: self.config.command_deadline_ms,
            });
        }
        if let Some((index, limit, value)) = self.first_limit_violation(&values) {
            let error = GatewayError::ActuatorLimit {
                index,
                value,
                lower: limit.lower,
                upper: limit.upper,
            };
            self.trip_safety(SafetyReason::ActuatorLimit, now_ms);
            return Err(error);
        }
        if !self.config.mode.can_actuate() {
            self.last_action_sequence = Some(sequence);
            self.last_action_observation_sequence = Some(observation_sequence);
            self.last_action_time_ms = Some(now_ms);
            self.record_event(GatewayEvent::ActionSuppressed {
                sequence,
                mode: self.config.mode,
            });
            return Ok(CommandDisposition::Suppressed);
        }
        if let Some(reason) = self.safety_latch {
            return Err(GatewayError::SafetyLatched(reason));
        }
        if !self.armed {
            return Err(GatewayError::NotArmed);
        }
        if self.actuations.len() == self.config.actuation_capacity {
            self.trip_safety(SafetyReason::QueueOverrun, now_ms);
            return Err(GatewayError::ActuationQueueFull);
        }
        self.last_action_sequence = Some(sequence);
        self.last_action_observation_sequence = Some(observation_sequence);
        self.last_action_time_ms = Some(now_ms);
        self.actuations.push_back(ActuationFrame {
            action_sequence: Some(sequence),
            queued_at_ms: now_ms,
            values,
            safety_stop: false,
            reason: None,
        });
        self.record_event(GatewayEvent::ActionQueued { sequence });
        Ok(CommandDisposition::Queued)
    }

    /// Applies stale-data policies at the supplied monotonic host tick.
    pub fn tick(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        self.advance_clock(now_ms)?;
        if !self.config.mode.can_actuate() || !self.armed {
            return Ok(());
        }
        let Some(observation) = self.observations.back() else {
            self.trip_safety(SafetyReason::ObservationStale, now_ms);
            return Ok(());
        };
        let observation_age_ms = now_ms - observation.received_at_ms;
        if observation_age_ms > self.config.max_observation_age_ms {
            self.trip_safety(SafetyReason::ObservationStale, now_ms);
            return Ok(());
        }
        if self.last_action_observation_sequence != Some(observation.sequence)
            && observation_age_ms > self.config.command_deadline_ms
        {
            self.trip_safety(SafetyReason::CommandDeadlineMissed, now_ms);
            return Ok(());
        }
        let command_reference_ms = self.last_action_time_ms.or(self.armed_at_ms);
        if command_reference_ms
            .is_some_and(|command_ms| now_ms - command_ms > self.config.max_command_age_ms)
        {
            self.trip_safety(SafetyReason::CommandStale, now_ms);
        }
        Ok(())
    }

    /// Returns the next actuator write, replacing an expired command with a safe stop.
    pub fn poll_actuation(&mut self, now_ms: u64) -> Result<Option<ActuationFrame>, GatewayError> {
        self.advance_clock(now_ms)?;
        let command_is_stale = self.actuations.front().is_some_and(|frame| {
            !frame.safety_stop && now_ms - frame.queued_at_ms > self.config.max_command_age_ms
        });
        if command_is_stale {
            self.trip_safety(SafetyReason::CommandStale, now_ms);
        }
        let frame = self.actuations.pop_front();
        if let Some(frame) = &frame {
            self.record_event(GatewayEvent::ActuationDelivered {
                action_sequence: frame.action_sequence,
                safety_stop: frame.safety_stop,
            });
        }
        Ok(frame)
    }

    /// Drains retained audit events in occurrence order.
    pub fn drain_events(&mut self) -> Vec<GatewayEvent> {
        self.events.drain(..).collect()
    }

    /// Drains retained audit events into one versioned session artifact.
    pub fn take_evidence(&mut self) -> GatewayEvidence {
        GatewayEvidence {
            kind: HARDWARE_GATEWAY_EVIDENCE_KIND.into(),
            schema_version: HARDWARE_GATEWAY_SCHEMA_VERSION,
            task_id: self.task.task_id.clone(),
            mode: self.config.mode,
            final_snapshot: self.snapshot(),
            events: self.drain_events(),
        }
    }

    fn require_connected(&self) -> Result<(), GatewayError> {
        if self.connected {
            Ok(())
        } else {
            Err(GatewayError::Disconnected)
        }
    }

    fn require_fresh_observation(&self, now_ms: u64) -> Result<(), GatewayError> {
        let observation = self
            .observations
            .back()
            .ok_or(GatewayError::NoFreshObservation)?;
        let age_ms = now_ms - observation.received_at_ms;
        if age_ms > self.config.max_observation_age_ms {
            return Err(GatewayError::ObservationStale {
                age_ms,
                limit_ms: self.config.max_observation_age_ms,
            });
        }
        Ok(())
    }

    fn first_limit_violation<'a>(
        &'a self,
        values: &[f64],
    ) -> Option<(usize, &'a ActionElementLimit, f64)> {
        self.action_limits
            .iter()
            .zip(values)
            .enumerate()
            .find_map(|(index, (limit, value))| {
                (*value < limit.lower || *value > limit.upper).then_some((index, limit, *value))
            })
    }

    fn advance_clock(&mut self, now_ms: u64) -> Result<(), GatewayError> {
        if let Some(previous_ms) = self.last_host_time_ms {
            if now_ms < previous_ms {
                self.trip_safety(SafetyReason::ClockRegression, previous_ms);
                return Err(GatewayError::ClockRegression {
                    previous_ms,
                    current_ms: now_ms,
                });
            }
        }
        self.last_host_time_ms = Some(now_ms);
        Ok(())
    }

    fn trip_safety(&mut self, reason: SafetyReason, now_ms: u64) {
        self.safety_latch = Some(reason);
        self.armed = false;
        self.armed_at_ms = None;
        self.actuations.clear();
        if self.config.mode.can_actuate() {
            self.queue_safe_stop(reason, now_ms);
        }
        self.record_event(GatewayEvent::SafetyTripped {
            host_time_ms: now_ms,
            reason,
        });
    }

    fn queue_safe_stop(&mut self, reason: SafetyReason, now_ms: u64) {
        if self.config.actuation_capacity == 0 {
            return;
        }
        if self.actuations.len() == self.config.actuation_capacity {
            self.actuations.pop_front();
        }
        self.actuations.push_back(ActuationFrame {
            action_sequence: None,
            queued_at_ms: now_ms,
            values: vec![0.0; self.action_width()],
            safety_stop: true,
            reason: Some(reason),
        });
    }

    fn record_event(&mut self, event: GatewayEvent) {
        if self.events.len() == self.config.event_capacity {
            self.events.pop_front();
            self.dropped_events = self.dropped_events.saturating_add(1);
        }
        self.events.push_back(event);
    }
}

fn flatten_observation_dtypes(
    tensors: &[TensorSpec],
) -> Result<Vec<TensorDType>, GatewayBuildError> {
    let mut dtypes = Vec::new();
    for tensor in tensors {
        let elements = tensor_elements("observation", tensor)?;
        match tensor.dtype {
            TensorDType::F32
            | TensorDType::F64
            | TensorDType::I32
            | TensorDType::I64
            | TensorDType::U8
            | TensorDType::Bool => dtypes.extend(std::iter::repeat_n(tensor.dtype, elements)),
            _ => {
                return Err(GatewayBuildError::UnsupportedTensorDType {
                    space: "observation",
                    tensor: tensor.name.clone(),
                });
            }
        }
    }
    Ok(dtypes)
}

fn float_space_width(
    space: &'static str,
    tensors: &[TensorSpec],
) -> Result<usize, GatewayBuildError> {
    tensors.iter().try_fold(0_usize, |total, tensor| {
        if !matches!(tensor.dtype, TensorDType::F32 | TensorDType::F64) {
            return Err(GatewayBuildError::UnsupportedTensorDType {
                space,
                tensor: tensor.name.clone(),
            });
        }
        let elements = tensor_elements(space, tensor)?;
        total
            .checked_add(elements)
            .ok_or_else(|| GatewayBuildError::TensorElementCountOverflow {
                space,
                tensor: tensor.name.clone(),
            })
    })
}

fn flatten_action_limits(
    tensors: &[TensorSpec],
) -> Result<Vec<ActionElementLimit>, GatewayBuildError> {
    let width = float_space_width("action", tensors)?;
    let mut limits = Vec::with_capacity(width);
    for tensor in tensors {
        let elements = tensor_elements("action", tensor)?;
        let bounds =
            tensor
                .bounds
                .as_ref()
                .ok_or_else(|| GatewayBuildError::MissingActionBounds {
                    tensor: tensor.name.clone(),
                })?;
        for element in 0..elements {
            limits.push(ActionElementLimit {
                tensor_name: tensor.name.clone(),
                tensor_element: element,
                unit: tensor.unit.clone(),
                dtype: tensor.dtype,
                lower: bounds.lower[if bounds.lower.len() == 1 { 0 } else { element }],
                upper: bounds.upper[if bounds.upper.len() == 1 { 0 } else { element }],
            });
        }
    }
    Ok(limits)
}

fn tensor_elements(space: &'static str, tensor: &TensorSpec) -> Result<usize, GatewayBuildError> {
    tensor
        .shape
        .iter()
        .try_fold(1_usize, |total, dimension| total.checked_mul(*dimension))
        .ok_or_else(|| GatewayBuildError::TensorElementCountOverflow {
            space,
            tensor: tensor.name.clone(),
        })
}

fn validate_values(
    space: &'static str,
    expected: usize,
    values: &[f64],
) -> Result<(), GatewayError> {
    if values.len() != expected {
        return Err(GatewayError::ValueCount {
            space,
            expected,
            actual: values.len(),
        });
    }
    if let Some(index) = values.iter().position(|value| !value.is_finite()) {
        return Err(GatewayError::NonFiniteValue { space, index });
    }
    Ok(())
}

fn normalize_observation_values(
    dtypes: &[TensorDType],
    values: &mut [f64],
) -> Result<(), GatewayError> {
    validate_values("observation", dtypes.len(), values)?;
    const EXACT_I64_MAX: f64 = 9_007_199_254_740_992.0;
    for (index, (dtype, value)) in dtypes.iter().zip(values).enumerate() {
        let valid = match dtype {
            TensorDType::F32 => *value >= f64::from(f32::MIN) && *value <= f64::from(f32::MAX),
            TensorDType::F64 => true,
            TensorDType::I32 => {
                value.fract() == 0.0
                    && *value >= f64::from(i32::MIN)
                    && *value <= f64::from(i32::MAX)
            }
            TensorDType::I64 => value.fract() == 0.0 && value.abs() <= EXACT_I64_MAX,
            TensorDType::U8 => {
                value.fract() == 0.0 && *value >= 0.0 && *value <= f64::from(u8::MAX)
            }
            TensorDType::Bool => *value == 0.0 || *value == 1.0,
            _ => false,
        };
        if !valid {
            return Err(GatewayError::ObservationDType {
                index,
                dtype: *dtype,
            });
        }
        if *dtype == TensorDType::F32 {
            *value = f64::from(*value as f32);
        }
    }
    Ok(())
}

fn normalize_action_values(
    limits: &[ActionElementLimit],
    values: &mut [f64],
) -> Result<(), GatewayError> {
    validate_values("action", limits.len(), values)?;
    for (index, (limit, value)) in limits.iter().zip(values).enumerate() {
        if limit.dtype == TensorDType::F32 {
            if *value < f64::from(f32::MIN) || *value > f64::from(f32::MAX) {
                return Err(GatewayError::ActionDType {
                    index,
                    dtype: limit.dtype,
                });
            }
            *value = f64::from(*value as f32);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ai::{diff_drive_goal_task_spec, DiffDriveRewardConfig};

    fn config(mode: HardwareMode) -> GatewayConfig {
        GatewayConfig {
            mode,
            max_observation_age_ms: 50,
            command_deadline_ms: 10,
            max_command_age_ms: 20,
            observation_capacity: 2,
            actuation_capacity: 2,
            event_capacity: 32,
        }
    }

    fn gateway(mode: HardwareMode) -> HardwareGateway {
        HardwareGateway::new(
            diff_drive_goal_task_spec(180, DiffDriveRewardConfig::default()),
            config(mode),
        )
        .unwrap()
    }

    fn observation(gateway: &HardwareGateway) -> Vec<f64> {
        vec![0.0; gateway.observation_width()]
    }

    #[test]
    fn shadow_validates_but_never_queues_actuation() {
        let mut gateway = gateway(HardwareMode::Shadow);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        assert_eq!(
            gateway.submit_action(1, 0, 0, vec![1.0, 1.0]),
            Ok(CommandDisposition::Suppressed)
        );
        assert_eq!(gateway.poll_actuation(1).unwrap(), None);
        assert_eq!(
            gateway.arm(1),
            Err(GatewayError::NonActuatingMode(HardwareMode::Shadow))
        );
    }

    #[test]
    fn live_mode_delivers_only_armed_bounded_actions() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        assert_eq!(
            gateway.submit_action(1, 0, 0, vec![1.0, 1.0]),
            Err(GatewayError::NotArmed)
        );
        gateway.arm(1).unwrap();
        assert_eq!(
            gateway.submit_action(2, 0, 0, vec![1.0, 1.0]),
            Ok(CommandDisposition::Queued)
        );
        let frame = gateway.poll_actuation(2).unwrap().unwrap();
        assert_eq!(frame.action_sequence, Some(0));
        assert!(!frame.safety_stop);
        assert_eq!(frame.values, vec![1.0, 1.0]);
    }

    #[test]
    fn actuator_limit_trips_and_replaces_output_with_zero_stop() {
        let mut gateway = gateway(HardwareMode::Hil);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        assert!(matches!(
            gateway.submit_action(1, 0, 0, vec![100.0, 0.0]),
            Err(GatewayError::ActuatorLimit { index: 0, .. })
        ));
        assert_eq!(
            gateway.snapshot().safety_latch,
            Some(SafetyReason::ActuatorLimit)
        );
        let stop = gateway.poll_actuation(1).unwrap().unwrap();
        assert!(stop.safety_stop);
        assert_eq!(stop.reason, Some(SafetyReason::ActuatorLimit));
        assert_eq!(stop.values, vec![0.0, 0.0]);
    }

    #[test]
    fn missed_deadline_trips_without_polling_a_command() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        gateway.tick(11).unwrap();
        assert_eq!(
            gateway.snapshot().safety_latch,
            Some(SafetyReason::CommandDeadlineMissed)
        );
        assert!(gateway.poll_actuation(11).unwrap().unwrap().safety_stop);
    }

    #[test]
    fn queued_command_expires_to_a_safe_stop() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        gateway.submit_action(1, 0, 0, vec![1.0, 1.0]).unwrap();
        let stop = gateway.poll_actuation(22).unwrap().unwrap();
        assert!(stop.safety_stop);
        assert_eq!(stop.reason, Some(SafetyReason::CommandStale));
    }

    #[test]
    fn disconnect_reconnect_requires_clear_fresh_observation_and_rearm() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        gateway.disconnect(1).unwrap();
        assert_eq!(
            gateway.snapshot().safety_latch,
            Some(SafetyReason::Disconnected)
        );
        assert!(gateway.poll_actuation(1).unwrap().unwrap().safety_stop);
        gateway.connect(2).unwrap();
        gateway.clear_safety_latch(2).unwrap();
        assert_eq!(gateway.arm(2), Err(GatewayError::NoFreshObservation));
        gateway
            .ingest_observation(3, 0, observation(&gateway))
            .unwrap();
        gateway.arm(3).unwrap();
        assert_eq!(gateway.connection_state(), GatewayConnectionState::Armed);
    }

    #[test]
    fn clean_close_requires_disarm_and_delivery_of_the_zero_stop() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        assert_eq!(
            gateway.close_cleanly(1),
            Err(GatewayError::ArmedOnCleanClose)
        );

        gateway.disarm(1).unwrap();
        assert_eq!(
            gateway.close_cleanly(1),
            Err(GatewayError::PendingActuationOnCleanClose)
        );
        let stop = gateway.poll_actuation(1).unwrap().unwrap();
        assert!(stop.safety_stop);
        assert_eq!(stop.reason, Some(SafetyReason::ManualDisarm));

        gateway.close_cleanly(2).unwrap();
        let evidence = gateway.take_evidence();
        assert_eq!(
            evidence.final_snapshot.connection_state,
            GatewayConnectionState::Disconnected
        );
        assert_eq!(evidence.final_snapshot.safety_latch, None);
        assert_eq!(evidence.final_snapshot.queued_actuations, 0);
        assert!(evidence
            .events
            .contains(&GatewayEvent::Disarmed { host_time_ms: 1 }));
        assert!(evidence
            .events
            .contains(&GatewayEvent::Disconnected { host_time_ms: 2 }));
    }

    #[test]
    fn device_safety_signal_latches_and_queues_an_independent_stop() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();

        gateway
            .device_safety_signal(SafetyReason::CommandStale, 1)
            .unwrap();
        assert_eq!(
            gateway.snapshot().safety_latch,
            Some(SafetyReason::CommandStale)
        );
        assert_eq!(
            gateway.connection_state(),
            GatewayConnectionState::Connected
        );
        let stop = gateway.poll_actuation(1).unwrap().unwrap();
        assert!(stop.safety_stop);
        assert_eq!(stop.reason, Some(SafetyReason::CommandStale));
    }

    #[test]
    fn emergency_stop_is_latched_until_explicit_clear() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(0).unwrap();
        gateway
            .ingest_observation(0, 0, observation(&gateway))
            .unwrap();
        gateway.arm(0).unwrap();
        gateway.emergency_stop(1).unwrap();
        assert_eq!(
            gateway.arm(1),
            Err(GatewayError::SafetyLatched(SafetyReason::EmergencyStop))
        );
        gateway.clear_safety_latch(2).unwrap();
        gateway.arm(2).unwrap();
    }

    #[test]
    fn observation_ring_is_bounded_and_audited() {
        let mut gateway = gateway(HardwareMode::Shadow);
        gateway.connect(0).unwrap();
        for sequence in 0..3 {
            gateway
                .ingest_observation(sequence, sequence, observation(&gateway))
                .unwrap();
        }
        let snapshot = gateway.snapshot();
        assert_eq!(snapshot.queued_observations, 2);
        assert_eq!(snapshot.dropped_observations, 1);
        assert!(gateway
            .drain_events()
            .contains(&GatewayEvent::ObservationDropped { sequence: 0 }));
    }

    #[test]
    fn observation_integer_dtype_is_enforced() {
        let mut gateway = gateway(HardwareMode::Shadow);
        gateway.connect(0).unwrap();
        let mut values = observation(&gateway);
        values[7] = 0.5;
        assert_eq!(
            gateway.ingest_observation(0, 0, values),
            Err(GatewayError::ObservationDType {
                index: 7,
                dtype: TensorDType::I64,
            })
        );
    }

    #[test]
    fn regressing_host_tick_fails_closed() {
        let mut gateway = gateway(HardwareMode::Live);
        gateway.connect(10).unwrap();
        assert!(matches!(
            gateway.ingest_observation(9, 0, observation(&gateway)),
            Err(GatewayError::ClockRegression {
                previous_ms: 10,
                current_ms: 9
            })
        ));
        assert_eq!(
            gateway.snapshot().safety_latch,
            Some(SafetyReason::ClockRegression)
        );
    }

    #[test]
    fn action_bounds_are_required_at_construction() {
        let mut task = diff_drive_goal_task_spec(180, DiffDriveRewardConfig::default());
        task.action.tensors[0].bounds = None;
        assert!(matches!(
            HardwareGateway::new(task, config(HardwareMode::Shadow)),
            Err(GatewayBuildError::MissingActionBounds { .. })
        ));
    }

    #[test]
    fn snapshot_json_is_versioned_and_bounded() {
        let gateway = gateway(HardwareMode::Playback);
        let value = serde_json::to_value(gateway.snapshot()).unwrap();
        assert_eq!(value["schema_version"], HARDWARE_GATEWAY_SCHEMA_VERSION);
        assert_eq!(value["queued_observations"], 0);
        assert_eq!(value["queued_actuations"], 0);
    }
}
