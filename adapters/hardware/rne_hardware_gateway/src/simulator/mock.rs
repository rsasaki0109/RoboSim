//! Deterministic reference implementation of the external simulator protocol.
//!
//! The mock proves framing and conformance behavior only. It is an
//! in-repository test double and cannot qualify as independent simulator
//! evidence.

use super::wire::{
    SimulatorAdapterFrame, SimulatorAdapterPayload, SimulatorHostFrame, SimulatorHostPayload,
    SimulatorRejectionCode,
};
use super::{is_sha256_hex, valid_identifier};
use sha2::{Digest, Sha256};

/// Fixed TaskSpec and simulator identity accepted by a mock adapter process.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MockSimulatorBinding {
    /// Stable simulator family.
    pub simulator_id: String,
    /// Exact simulator version.
    pub simulator_version: String,
    /// Stable adapter identity.
    pub adapter_id: String,
    /// Accepted TaskSpec identity.
    pub task_id: String,
    /// SHA-256 of the accepted TaskSpec bytes.
    pub task_sha256: String,
    /// Flattened TaskSpec observation width.
    pub observation_width: usize,
    /// Flattened TaskSpec action width.
    pub action_width: usize,
    /// Simulation-time ticks per accepted action.
    pub fixed_delta_ticks: u64,
}

impl MockSimulatorBinding {
    /// Validates all identities, widths, digest, and timing fields.
    pub fn validate(&self) -> Result<(), MockSimulatorError> {
        for (field, value) in [
            ("simulator_id", self.simulator_id.as_str()),
            ("simulator_version", self.simulator_version.as_str()),
            ("adapter_id", self.adapter_id.as_str()),
            ("task_id", self.task_id.as_str()),
        ] {
            if !valid_identifier(value) {
                return Err(MockSimulatorError::InvalidBinding(field));
            }
        }
        if !is_sha256_hex(&self.task_sha256) {
            return Err(MockSimulatorError::InvalidBinding("task_sha256"));
        }
        if self.observation_width == 0 {
            return Err(MockSimulatorError::InvalidBinding("observation_width"));
        }
        if self.action_width == 0 {
            return Err(MockSimulatorError::InvalidBinding("action_width"));
        }
        if self.fixed_delta_ticks == 0 {
            return Err(MockSimulatorError::InvalidBinding("fixed_delta_ticks"));
        }
        Ok(())
    }
}

/// Stateful deterministic simulator protocol test double.
#[derive(Clone, Debug)]
pub struct MockSimulatorAdapter {
    binding: MockSimulatorBinding,
    session_id: Option<String>,
    last_request_sequence: Option<u64>,
    seed: Option<u64>,
    next_action_sequence: u64,
    step: u64,
    action_state: Vec<f64>,
    closed: bool,
}

impl MockSimulatorAdapter {
    /// Creates a closed mock with one immutable TaskSpec binding.
    pub fn new(binding: MockSimulatorBinding) -> Result<Self, MockSimulatorError> {
        binding.validate()?;
        let action_width = binding.action_width;
        Ok(Self {
            binding,
            session_id: None,
            last_request_sequence: None,
            seed: None,
            next_action_sequence: 0,
            step: 0,
            action_state: vec![0.0; action_width],
            closed: false,
        })
    }

    /// Handles one validated host request without reading a wall clock.
    pub fn handle(
        &mut self,
        request: SimulatorHostFrame,
    ) -> Result<SimulatorAdapterFrame, MockSimulatorError> {
        request.validate()?;
        let session = request.session_id.clone();
        let sequence = request.sequence;
        if self.closed {
            return Ok(self.rejected(&session, sequence, SimulatorRejectionCode::TerminalState));
        }
        if self
            .last_request_sequence
            .is_some_and(|previous| sequence <= previous)
        {
            return Ok(self.rejected(
                &session,
                sequence,
                SimulatorRejectionCode::NonMonotonicSequence,
            ));
        }
        self.last_request_sequence = Some(sequence);
        if self
            .session_id
            .as_deref()
            .is_some_and(|expected| expected != session)
        {
            return Ok(self.rejected(&session, sequence, SimulatorRejectionCode::SessionMismatch));
        }

        let payload = match request.payload {
            SimulatorHostPayload::Open {
                task_id,
                task_sha256,
                observation_width,
                action_width,
                fixed_delta_ticks,
            } => {
                if self.session_id.is_some() {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::AlreadyOpen,
                    }
                } else if task_id != self.binding.task_id || task_sha256 != self.binding.task_sha256
                {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::TaskMismatch,
                    }
                } else if observation_width != self.binding.observation_width
                    || action_width != self.binding.action_width
                {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::WidthMismatch,
                    }
                } else if fixed_delta_ticks != self.binding.fixed_delta_ticks {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::FixedDeltaMismatch,
                    }
                } else {
                    self.session_id = Some(session.clone());
                    SimulatorAdapterPayload::Ready {
                        simulator_id: self.binding.simulator_id.clone(),
                        simulator_version: self.binding.simulator_version.clone(),
                        adapter_id: self.binding.adapter_id.clone(),
                        task_id,
                        task_sha256,
                        observation_width,
                        action_width,
                        fixed_delta_ticks,
                    }
                }
            }
            SimulatorHostPayload::Reset { seed } => {
                if self.session_id.is_none() {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::NotOpen,
                    }
                } else {
                    self.seed = Some(seed);
                    self.next_action_sequence = 0;
                    self.step = 0;
                    self.action_state.fill(0.0);
                    let values = self.observation();
                    SimulatorAdapterPayload::ResetComplete {
                        seed,
                        state_digest: state_digest(seed, self.step, &values),
                        values,
                    }
                }
            }
            SimulatorHostPayload::Step {
                action_sequence,
                values,
            } => {
                if self.session_id.is_none() {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::NotOpen,
                    }
                } else if self.seed.is_none() {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::ResetRequired,
                    }
                } else if values.len() != self.binding.action_width {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::WidthMismatch,
                    }
                } else if values.iter().any(|value| !value.is_finite()) {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::NonFiniteValue,
                    }
                } else if action_sequence != self.next_action_sequence {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::ActionSequenceMismatch,
                    }
                } else {
                    let next_action_sequence = self
                        .next_action_sequence
                        .checked_add(1)
                        .ok_or(MockSimulatorError::CounterOverflow)?;
                    let next_step = self
                        .step
                        .checked_add(1)
                        .ok_or(MockSimulatorError::CounterOverflow)?;
                    let sim_time_ticks = self
                        .binding
                        .fixed_delta_ticks
                        .checked_mul(next_step)
                        .ok_or(MockSimulatorError::CounterOverflow)?;
                    self.action_state.clone_from(&values);
                    self.next_action_sequence = next_action_sequence;
                    self.step = next_step;
                    let values = self.observation();
                    let seed = self.seed.expect("step requires reset seed");
                    SimulatorAdapterPayload::Stepped {
                        action_sequence,
                        step: self.step,
                        sim_time_ticks,
                        state_digest: state_digest(seed, self.step, &values),
                        values,
                        terminated: false,
                        truncated: false,
                    }
                }
            }
            SimulatorHostPayload::Close => {
                if self.session_id.is_none() {
                    SimulatorAdapterPayload::Rejected {
                        code: SimulatorRejectionCode::NotOpen,
                    }
                } else {
                    self.closed = true;
                    SimulatorAdapterPayload::Closed
                }
            }
        };
        Ok(SimulatorAdapterFrame::new(session, sequence, payload))
    }

    fn rejected(
        &self,
        session: &str,
        sequence: u64,
        code: SimulatorRejectionCode,
    ) -> SimulatorAdapterFrame {
        SimulatorAdapterFrame::new(
            session,
            sequence,
            SimulatorAdapterPayload::Rejected { code },
        )
    }

    fn observation(&self) -> Vec<f64> {
        let seed = self.seed.unwrap_or(0);
        (0..self.binding.observation_width)
            .map(|index| {
                let action = self.action_state[index % self.action_state.len()];
                let seed_component = ((seed >> (index % 32)) & 0xff) as f64 / 255.0;
                seed_component + self.step as f64 * 0.01 + action * 0.125
            })
            .collect()
    }
}

/// Invalid mock configuration or host frame.
#[derive(Debug, thiserror::Error)]
pub enum MockSimulatorError {
    /// One fixed binding field is malformed.
    #[error("invalid mock external simulator binding field {0}")]
    InvalidBinding(&'static str),
    /// A protocol counter or exact simulation timestamp cannot be represented.
    #[error("mock external simulator counter overflow")]
    CounterOverflow,
    /// Host frame violates the simulator wire contract.
    #[error(transparent)]
    Wire(#[from] super::wire::SimulatorWireError),
}

fn state_digest(seed: u64, step: u64, values: &[f64]) -> u64 {
    let mut hasher = Sha256::new();
    hasher.update(b"rne_external_simulator_mock_state_v1");
    hasher.update(seed.to_le_bytes());
    hasher.update(step.to_le_bytes());
    for value in values {
        hasher.update(value.to_bits().to_le_bytes());
    }
    let digest = hasher.finalize();
    u64::from_le_bytes(digest[..8].try_into().expect("SHA-256 has eight bytes"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binding() -> MockSimulatorBinding {
        MockSimulatorBinding {
            simulator_id: "gazebo_sim_fixture".to_string(),
            simulator_version: "8.9.0".to_string(),
            adapter_id: "rne_gazebo_fixture".to_string(),
            task_id: "task-v1".to_string(),
            task_sha256: "0".repeat(64),
            observation_width: 3,
            action_width: 2,
            fixed_delta_ticks: 10,
        }
    }

    fn open(sequence: u64) -> SimulatorHostFrame {
        let binding = binding();
        SimulatorHostFrame::new(
            "session-v1",
            sequence,
            SimulatorHostPayload::Open {
                task_id: binding.task_id,
                task_sha256: binding.task_sha256,
                observation_width: binding.observation_width,
                action_width: binding.action_width,
                fixed_delta_ticks: binding.fixed_delta_ticks,
            },
        )
    }

    #[test]
    fn reset_and_steps_are_exactly_repeatable() {
        let mut adapter = MockSimulatorAdapter::new(binding()).unwrap();
        assert!(matches!(
            adapter.handle(open(1)).unwrap().payload,
            SimulatorAdapterPayload::Ready { .. }
        ));
        let reset =
            SimulatorHostFrame::new("session-v1", 2, SimulatorHostPayload::Reset { seed: 7 });
        let first = adapter.handle(reset).unwrap();
        let step = SimulatorHostFrame::new(
            "session-v1",
            3,
            SimulatorHostPayload::Step {
                action_sequence: 0,
                values: vec![0.25, -0.5],
            },
        );
        let stepped = adapter.handle(step).unwrap();
        assert!(matches!(
            stepped.payload,
            SimulatorAdapterPayload::Stepped {
                step: 1,
                sim_time_ticks: 10,
                ..
            }
        ));
        let repeated = adapter
            .handle(SimulatorHostFrame::new(
                "session-v1",
                4,
                SimulatorHostPayload::Reset { seed: 7 },
            ))
            .unwrap();
        assert_eq!(first.payload, repeated.payload);
    }

    #[test]
    fn simulation_time_overflow_fails_without_advancing_state() {
        let mut adapter = MockSimulatorAdapter::new(binding()).unwrap();
        adapter.handle(open(1)).unwrap();
        adapter
            .handle(SimulatorHostFrame::new(
                "session-v1",
                2,
                SimulatorHostPayload::Reset { seed: 7 },
            ))
            .unwrap();
        adapter.step = u64::MAX;

        let error = adapter
            .handle(SimulatorHostFrame::new(
                "session-v1",
                3,
                SimulatorHostPayload::Step {
                    action_sequence: 0,
                    values: vec![0.25, -0.5],
                },
            ))
            .unwrap_err();

        assert!(matches!(error, MockSimulatorError::CounterOverflow));
        assert_eq!(adapter.step, u64::MAX);
        assert_eq!(adapter.next_action_sequence, 0);
        assert_eq!(adapter.action_state, vec![0.0, 0.0]);
    }
}
