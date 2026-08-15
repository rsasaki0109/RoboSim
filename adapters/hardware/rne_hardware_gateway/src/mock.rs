//! Deterministic device process used to prove the public hardware wire contract.
//!
//! The mock never reads a clock or sleeps. Every poll emits a zero-valued
//! observation with a connection-local sequence, and configured faults trigger
//! after an exact exchange count. It is a conformance device, not a physical
//! robot model.

use crate::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareWireError, HostWireFrame, HostWirePayload,
    WireDisconnectReason, WireRejectionCode,
};
use crate::{HardwareMode, SafetyReason};
use serde::{Deserialize, Serialize};

/// Schema version for deterministic process-mock conformance reports.
pub const MOCK_CONFORMANCE_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`MockConformanceReport`].
pub const MOCK_CONFORMANCE_REPORT_KIND: &str = "rne_hardware_mock_conformance";

/// Required process-level hardware mock case in canonical report order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MockConformanceCase {
    /// Observation-to-command deadline expires before an action is submitted.
    CommandDeadline,
    /// The device transport disconnects during actuation.
    Disconnect,
    /// A disconnected gateway explicitly clears, observes, and rearms.
    Reconnect,
    /// A queued command expires before device delivery.
    CommandStale,
    /// A controller action exceeds a TaskSpec actuator bound.
    ActuatorLimit,
    /// The device process independently asserts emergency stop.
    EmergencyStop,
}

impl MockConformanceCase {
    const REQUIRED: [Self; 6] = [
        Self::CommandDeadline,
        Self::Disconnect,
        Self::Reconnect,
        Self::CommandStale,
        Self::ActuatorLimit,
        Self::EmergencyStop,
    ];

    fn expected_reason(self) -> Option<SafetyReason> {
        match self {
            Self::CommandDeadline => Some(SafetyReason::CommandDeadlineMissed),
            Self::Disconnect => Some(SafetyReason::Disconnected),
            Self::Reconnect => None,
            Self::CommandStale => Some(SafetyReason::CommandStale),
            Self::ActuatorLimit => Some(SafetyReason::ActuatorLimit),
            Self::EmergencyStop => Some(SafetyReason::EmergencyStop),
        }
    }
}

/// Evidence verdict for one real child-process conformance case.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MockConformanceCaseResult {
    /// Canonical case identity.
    pub case: MockConformanceCase,
    /// Gateway latch expected at the case proof point, absent after rearm.
    pub gateway_reason: Option<SafetyReason>,
    /// Whether a device response confirmed zero output.
    pub device_stop_confirmed: bool,
    /// Whether the gateway delivered its locally queued zero frame.
    pub gateway_stop_delivered: bool,
    /// True only when the reconnect case regained explicit authority.
    pub reconnect_rearmed: bool,
    /// Complete case verdict.
    pub passed: bool,
}

/// Canonical six-case report produced by the process-isolated mock suite.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MockConformanceReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Results in [`MockConformanceCase`] canonical order.
    pub cases: Vec<MockConformanceCaseResult>,
    /// True only when every required case passed.
    pub all_passed: bool,
}

impl MockConformanceReport {
    /// Creates and validates a complete canonical report.
    pub fn new(cases: Vec<MockConformanceCaseResult>) -> Result<Self, MockConformanceReportError> {
        let report = Self {
            kind: MOCK_CONFORMANCE_REPORT_KIND.to_string(),
            schema_version: MOCK_CONFORMANCE_SCHEMA_VERSION,
            all_passed: cases.iter().all(|case| case.passed),
            cases,
        };
        report.validate()?;
        Ok(report)
    }

    /// Validates exact case coverage and fail-closed proof fields.
    pub fn validate(&self) -> Result<(), MockConformanceReportError> {
        if self.kind != MOCK_CONFORMANCE_REPORT_KIND {
            return Err(MockConformanceReportError::InvalidKind);
        }
        if self.schema_version != MOCK_CONFORMANCE_SCHEMA_VERSION {
            return Err(MockConformanceReportError::UnsupportedSchemaVersion {
                actual: self.schema_version,
            });
        }
        if self.cases.len() != MockConformanceCase::REQUIRED.len() {
            return Err(MockConformanceReportError::CaseCount {
                actual: self.cases.len(),
            });
        }
        for (index, (result, expected)) in self
            .cases
            .iter()
            .zip(MockConformanceCase::REQUIRED)
            .enumerate()
        {
            let reconnect_rearmed = expected == MockConformanceCase::Reconnect;
            if result.case != expected {
                return Err(MockConformanceReportError::CaseOrder {
                    index,
                    expected,
                    actual: result.case,
                });
            }
            if result.gateway_reason != expected.expected_reason()
                || !result.device_stop_confirmed
                || !result.gateway_stop_delivered
                || result.reconnect_rearmed != reconnect_rearmed
                || !result.passed
            {
                return Err(MockConformanceReportError::CaseFailed { case: expected });
            }
        }
        if !self.all_passed || self.all_passed != self.cases.iter().all(|case| case.passed) {
            return Err(MockConformanceReportError::AggregateMismatch);
        }
        Ok(())
    }
}

/// Failure validating process-mock conformance evidence.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum MockConformanceReportError {
    /// The report kind is unsupported.
    #[error("invalid hardware mock conformance kind")]
    InvalidKind,
    /// The report schema is unsupported.
    #[error("unsupported hardware mock conformance schema {actual}")]
    UnsupportedSchemaVersion {
        /// Received schema version.
        actual: u32,
    },
    /// The report does not contain exactly six required cases.
    #[error("hardware mock conformance requires 6 cases, got {actual}")]
    CaseCount {
        /// Received result count.
        actual: usize,
    },
    /// A result is not in canonical case order.
    #[error("hardware mock case {index} must be {expected:?}, got {actual:?}")]
    CaseOrder {
        /// Case index.
        index: usize,
        /// Required case.
        expected: MockConformanceCase,
        /// Received case.
        actual: MockConformanceCase,
    },
    /// A required stop, latch, or rearm proof is absent.
    #[error("hardware mock conformance case {case:?} did not prove its invariants")]
    CaseFailed {
        /// Failed case.
        case: MockConformanceCase,
    },
    /// The aggregate verdict disagrees with case verdicts.
    #[error("hardware mock conformance aggregate verdict mismatch")]
    AggregateMismatch,
}

/// One deterministic terminal fault injected by the mock device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MockDeviceFault {
    /// Disconnect instead of acknowledging the selected actuation count.
    DisconnectAfterActuations {
        /// One-based actuation count that triggers the disconnect.
        count: u64,
    },
    /// Assert emergency stop instead of returning the selected observation.
    EmergencyStopAfterObservations {
        /// One-based observation poll count that triggers emergency stop.
        count: u64,
    },
}

impl MockDeviceFault {
    fn count(self) -> u64 {
        match self {
            Self::DisconnectAfterActuations { count }
            | Self::EmergencyStopAfterObservations { count } => count,
        }
    }
}

/// Configuration for one deterministic mock device process.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MockDeviceConfig {
    /// Stable device identity returned by the open handshake.
    pub device_id: String,
    /// Optional exact-count terminal fault.
    pub fault: Option<MockDeviceFault>,
    /// Optional fixed TaskSpec binding enforced during open.
    pub binding: Option<MockDeviceBinding>,
}

/// Fixed TaskSpec identity and flattened widths enforced by a mock process.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MockDeviceBinding {
    /// Portable task identity accepted by the mock.
    pub task_id: String,
    /// Required flattened observation width.
    pub observation_width: usize,
    /// Required flattened action width.
    pub action_width: usize,
}

impl MockDeviceConfig {
    /// Validates a non-empty identity and non-zero fault count.
    pub fn validate(&self) -> Result<(), MockDeviceError> {
        if self.device_id.trim().is_empty() {
            return Err(MockDeviceError::EmptyDeviceId);
        }
        if self.fault.is_some_and(|fault| fault.count() == 0) {
            return Err(MockDeviceError::ZeroFaultCount);
        }
        if self.binding.as_ref().is_some_and(|binding| {
            binding.task_id.trim().is_empty()
                || binding.observation_width == 0
                || binding.action_width == 0
        }) {
            return Err(MockDeviceError::InvalidBinding);
        }
        Ok(())
    }
}

impl Default for MockDeviceConfig {
    fn default() -> Self {
        Self {
            device_id: "rne-mock-device-v1".to_string(),
            fault: None,
            binding: None,
        }
    }
}

#[derive(Debug)]
struct MockSession {
    session_id: String,
    mode: HardwareMode,
    observation_width: usize,
    action_width: usize,
}

/// Stateful deterministic implementation of the device side of protocol v1.
#[derive(Debug)]
pub struct MockHardwareDevice {
    config: MockDeviceConfig,
    session: Option<MockSession>,
    last_host_sequence: Option<u64>,
    observation_count: u64,
    actuation_count: u64,
    terminal: bool,
}

impl MockHardwareDevice {
    /// Creates a mock with validated deterministic fault configuration.
    pub fn new(config: MockDeviceConfig) -> Result<Self, MockDeviceError> {
        config.validate()?;
        Ok(Self {
            config,
            session: None,
            last_host_sequence: None,
            observation_count: 0,
            actuation_count: 0,
            terminal: false,
        })
    }

    /// Handles one validated host request and always returns a correlated response.
    pub fn handle(&mut self, frame: HostWireFrame) -> Result<DeviceWireFrame, MockDeviceError> {
        frame.validate()?;
        if self.terminal {
            return Ok(rejection(&frame, WireRejectionCode::TerminalState));
        }
        if let Some(previous) = self.last_host_sequence {
            if frame.sequence <= previous {
                return Ok(rejection(&frame, WireRejectionCode::NonMonotonicSequence));
            }
        }
        if self
            .session
            .as_ref()
            .is_some_and(|session| session.session_id != frame.session_id)
        {
            return Ok(rejection(&frame, WireRejectionCode::SessionMismatch));
        }
        self.last_host_sequence = Some(frame.sequence);

        match &frame.payload {
            HostWirePayload::Open {
                task_id,
                mode,
                observation_width,
                action_width,
            } => {
                if self.session.is_some() {
                    return Ok(rejection(&frame, WireRejectionCode::AlreadyOpen));
                }
                if self.config.binding.as_ref().is_some_and(|binding| {
                    binding.task_id != *task_id
                        || binding.observation_width != *observation_width
                        || binding.action_width != *action_width
                }) {
                    return Ok(rejection(&frame, WireRejectionCode::WidthMismatch));
                }
                self.session = Some(MockSession {
                    session_id: frame.session_id.clone(),
                    mode: *mode,
                    observation_width: *observation_width,
                    action_width: *action_width,
                });
                Ok(DeviceWireFrame::new(
                    &frame.session_id,
                    frame.sequence,
                    DeviceWirePayload::Ready {
                        device_id: self.config.device_id.clone(),
                        task_id: task_id.clone(),
                        observation_width: *observation_width,
                        action_width: *action_width,
                    },
                ))
            }
            HostWirePayload::PollObservation => self.poll_observation(&frame),
            HostWirePayload::Actuate { frame: actuation } => {
                let Some(session) = &self.session else {
                    return Ok(rejection(&frame, WireRejectionCode::NotOpen));
                };
                if actuation.values.len() != session.action_width {
                    return Ok(rejection(&frame, WireRejectionCode::WidthMismatch));
                }
                if actuation.values.iter().any(|value| !value.is_finite()) {
                    return Ok(rejection(&frame, WireRejectionCode::NonFiniteValue));
                }
                if !actuation.safety_stop
                    && matches!(session.mode, HardwareMode::Playback | HardwareMode::Shadow)
                {
                    return Ok(rejection(&frame, WireRejectionCode::AuthorityDenied));
                }
                self.actuation_count = self.actuation_count.saturating_add(1);
                if self.config.fault
                    == Some(MockDeviceFault::DisconnectAfterActuations {
                        count: self.actuation_count,
                    })
                {
                    self.terminal = true;
                    return Ok(DeviceWireFrame::new(
                        &frame.session_id,
                        frame.sequence,
                        DeviceWirePayload::Disconnected {
                            reason: WireDisconnectReason::InjectedFault,
                            safe_stop_applied: true,
                        },
                    ));
                }
                Ok(DeviceWireFrame::new(
                    &frame.session_id,
                    frame.sequence,
                    DeviceWirePayload::ActuationAccepted {
                        action_sequence: actuation.action_sequence,
                        safety_stop: actuation.safety_stop,
                    },
                ))
            }
            HostWirePayload::Close => {
                if self.session.is_none() {
                    return Ok(rejection(&frame, WireRejectionCode::NotOpen));
                }
                self.terminal = true;
                Ok(DeviceWireFrame::new(
                    &frame.session_id,
                    frame.sequence,
                    DeviceWirePayload::Closed,
                ))
            }
        }
    }

    fn poll_observation(
        &mut self,
        frame: &HostWireFrame,
    ) -> Result<DeviceWireFrame, MockDeviceError> {
        let Some(session) = &self.session else {
            return Ok(rejection(frame, WireRejectionCode::NotOpen));
        };
        self.observation_count = self.observation_count.saturating_add(1);
        if self.config.fault
            == Some(MockDeviceFault::EmergencyStopAfterObservations {
                count: self.observation_count,
            })
        {
            self.terminal = true;
            return Ok(DeviceWireFrame::new(
                &frame.session_id,
                frame.sequence,
                DeviceWirePayload::SafetySignal {
                    reason: SafetyReason::EmergencyStop,
                    safe_stop_applied: true,
                },
            ));
        }
        Ok(DeviceWireFrame::new(
            &frame.session_id,
            frame.sequence,
            DeviceWirePayload::Observation {
                sequence: self.observation_count,
                values: vec![0.0; session.observation_width],
            },
        ))
    }
}

/// Failure configuring or executing the deterministic mock device.
#[derive(Debug, thiserror::Error)]
pub enum MockDeviceError {
    /// The configured device identity is empty.
    #[error("mock device_id must not be empty")]
    EmptyDeviceId,
    /// A one-based injected fault count is zero.
    #[error("mock fault count must be greater than zero")]
    ZeroFaultCount,
    /// A fixed TaskSpec binding is empty or zero-width.
    #[error("mock TaskSpec binding must have a task id and non-zero widths")]
    InvalidBinding,
    /// The host frame is not valid protocol v1.
    #[error(transparent)]
    InvalidFrame(#[from] HardwareWireError),
}

fn rejection(frame: &HostWireFrame, code: WireRejectionCode) -> DeviceWireFrame {
    DeviceWireFrame::new(
        &frame.session_id,
        frame.sequence,
        DeviceWirePayload::Rejected { code },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::HostWirePayload;
    use crate::{ActuationFrame, HardwareMode};

    fn open(sequence: u64) -> HostWireFrame {
        HostWireFrame::new(
            "session-1",
            sequence,
            HostWirePayload::Open {
                task_id: "rne.test.task.v1".into(),
                mode: HardwareMode::Hil,
                observation_width: 3,
                action_width: 2,
            },
        )
    }

    #[test]
    fn mock_session_is_deterministic_and_width_bound() {
        let mut device = MockHardwareDevice::new(MockDeviceConfig::default()).unwrap();
        assert!(matches!(
            device.handle(open(1)).unwrap().payload,
            DeviceWirePayload::Ready { .. }
        ));
        let observed = device
            .handle(HostWireFrame::new(
                "session-1",
                2,
                HostWirePayload::PollObservation,
            ))
            .unwrap();
        assert_eq!(
            observed.payload,
            DeviceWirePayload::Observation {
                sequence: 1,
                values: vec![0.0; 3],
            }
        );
        let rejected = device
            .handle(HostWireFrame::new(
                "session-1",
                3,
                HostWirePayload::Actuate {
                    frame: ActuationFrame {
                        action_sequence: Some(1),
                        queued_at_ms: 0,
                        values: vec![0.0],
                        safety_stop: false,
                        reason: None,
                    },
                },
            ))
            .unwrap();
        assert_eq!(
            rejected.payload,
            DeviceWirePayload::Rejected {
                code: WireRejectionCode::WidthMismatch,
            }
        );
    }

    #[test]
    fn disconnect_fault_confirms_device_side_stop() {
        let mut device = MockHardwareDevice::new(MockDeviceConfig {
            device_id: "mock-fault".into(),
            fault: Some(MockDeviceFault::DisconnectAfterActuations { count: 1 }),
            binding: None,
        })
        .unwrap();
        device.handle(open(1)).unwrap();
        let response = device
            .handle(HostWireFrame::new(
                "session-1",
                2,
                HostWirePayload::Actuate {
                    frame: ActuationFrame {
                        action_sequence: Some(1),
                        queued_at_ms: 0,
                        values: vec![0.1, -0.1],
                        safety_stop: false,
                        reason: None,
                    },
                },
            ))
            .unwrap();
        assert_eq!(
            response.payload,
            DeviceWirePayload::Disconnected {
                reason: WireDisconnectReason::InjectedFault,
                safe_stop_applied: true,
            }
        );
    }

    #[test]
    fn invalid_fault_counts_are_rejected() {
        assert!(matches!(
            MockHardwareDevice::new(MockDeviceConfig {
                device_id: "mock".into(),
                fault: Some(MockDeviceFault::EmergencyStopAfterObservations { count: 0 }),
                binding: None,
            }),
            Err(MockDeviceError::ZeroFaultCount)
        ));
    }

    #[test]
    fn fixed_binding_and_shadow_authority_fail_closed() {
        let mut device = MockHardwareDevice::new(MockDeviceConfig {
            device_id: "bound-mock".into(),
            fault: None,
            binding: Some(MockDeviceBinding {
                task_id: "rne.test.task.v1".into(),
                observation_width: 3,
                action_width: 2,
            }),
        })
        .unwrap();
        let wrong = device
            .handle(HostWireFrame::new(
                "session-1",
                1,
                HostWirePayload::Open {
                    task_id: "rne.test.task.v1".into(),
                    mode: HardwareMode::Shadow,
                    observation_width: 3,
                    action_width: 3,
                },
            ))
            .unwrap();
        assert_eq!(
            wrong.payload,
            DeviceWirePayload::Rejected {
                code: WireRejectionCode::WidthMismatch,
            }
        );
        let mut shadow = open(2);
        let HostWirePayload::Open { mode, .. } = &mut shadow.payload else {
            unreachable!();
        };
        *mode = HardwareMode::Shadow;
        assert!(matches!(
            device.handle(shadow).unwrap().payload,
            DeviceWirePayload::Ready { .. }
        ));
        let denied = device
            .handle(HostWireFrame::new(
                "session-1",
                3,
                HostWirePayload::Actuate {
                    frame: ActuationFrame {
                        action_sequence: Some(0),
                        queued_at_ms: 0,
                        values: vec![0.0; 2],
                        safety_stop: false,
                        reason: None,
                    },
                },
            ))
            .unwrap();
        assert_eq!(
            denied.payload,
            DeviceWirePayload::Rejected {
                code: WireRejectionCode::AuthorityDenied,
            }
        );
    }
}
