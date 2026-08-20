//! TaskSpec-ordered comparison of hardware observations and simulation rollout.
//!
//! Shadow comparison never owns actuator authority. It consumes already
//! timestamped hardware observations plus deterministic simulation steps and
//! emits a bounded, tolerance-declared report with the first divergent field.

use crate::{
    flatten_observation_dtypes, normalize_observation_values, GatewayBuildError, GatewayError,
    HardwareObservation,
};
use rne_ai::{TaskSpec, TaskSpecValidationError, TensorDType};
use serde::{Deserialize, Serialize};

/// Schema version for shadow comparison reports.
pub const SHADOW_COMPARISON_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`ShadowComparisonReport`].
pub const SHADOW_COMPARISON_REPORT_KIND: &str = "rne_hardware_shadow_comparison";

/// Absolute tolerance for one TaskSpec observation tensor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowTensorTolerance {
    /// Tensor name in TaskSpec observation order.
    pub tensor_name: String,
    /// Inclusive absolute error tolerance in the tensor's declared unit.
    pub absolute_tolerance: f64,
}

/// Memory bound and complete ordered tolerance contract for shadow evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowComparisonConfig {
    /// Maximum retained comparison samples.
    pub sample_capacity: usize,
    /// One tolerance per observation tensor in exact TaskSpec order.
    pub tensors: Vec<ShadowTensorTolerance>,
}

/// First flattened element that exceeded its declared tolerance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowViolation {
    /// Tensor name from the TaskSpec.
    pub tensor_name: String,
    /// Row-major element index within the tensor.
    pub tensor_element: usize,
    /// Tensor unit from the TaskSpec.
    pub unit: String,
    /// Hardware observation value after dtype normalization.
    pub hardware_value: f64,
    /// Simulation value after dtype normalization.
    pub simulation_value: f64,
    /// Absolute error in the declared unit.
    pub absolute_error: f64,
    /// Inclusive absolute tolerance.
    pub absolute_tolerance: f64,
}

/// Deterministic metrics for one hardware/simulation observation pair.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowComparisonSample {
    /// Connection-local hardware observation sequence.
    pub hardware_sequence: u64,
    /// Injected monotonic host receipt tick.
    pub hardware_received_at_ms: u64,
    /// Deterministic simulation step paired with the observation.
    pub simulation_step: u64,
    /// Simulation timestamp in SimClock nanosecond ticks.
    pub simulation_time_ticks: u64,
    /// Normalized hardware values in TaskSpec flattened order.
    pub hardware_values: Vec<f64>,
    /// Normalized simulation values in TaskSpec flattened order.
    pub simulation_values: Vec<f64>,
    /// Maximum elementwise absolute error.
    pub max_absolute_error: f64,
    /// Sum of elementwise absolute error in stable flattened order.
    pub sum_absolute_error: f64,
    /// Mean elementwise absolute error in stable flattened order.
    pub mean_absolute_error: f64,
    /// Number of elements outside tolerance.
    pub violating_elements: usize,
    /// First violating field in TaskSpec and row-major order.
    pub first_violation: Option<ShadowViolation>,
}

/// Aggregate deterministic metrics for a completed shadow comparison.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowComparisonSummary {
    /// Number of retained hardware/simulation pairs.
    pub compared_samples: usize,
    /// Total number of compared flattened elements.
    pub compared_elements: usize,
    /// Number of samples containing at least one violation.
    pub violating_samples: usize,
    /// Total number of elements outside tolerance.
    pub violating_elements: usize,
    /// Maximum absolute error across the report.
    pub max_absolute_error: f64,
    /// Mean absolute error across all elements in stable order.
    pub mean_absolute_error: f64,
    /// True only when no element exceeded tolerance.
    pub passed: bool,
}

/// Bounded, versioned shadow comparison evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShadowComparisonReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Bound TaskSpec identity.
    pub task_id: String,
    /// Ordered tensor tolerance contract.
    pub tolerances: Vec<ShadowTensorTolerance>,
    /// Comparisons in hardware sequence order.
    pub samples: Vec<ShadowComparisonSample>,
    /// Aggregate report verdict and metrics.
    pub summary: ShadowComparisonSummary,
}

impl ShadowComparisonReport {
    /// Rebinds an untrusted report to its TaskSpec and validates all structural
    /// metrics, ordering, first-violation, aggregate, and verdict invariants.
    pub fn validate_against(&self, task: &TaskSpec) -> Result<(), ShadowComparisonError> {
        task.validate()?;
        if self.kind != SHADOW_COMPARISON_REPORT_KIND
            || self.schema_version != SHADOW_COMPARISON_SCHEMA_VERSION
        {
            return Err(ShadowComparisonError::InvalidReport {
                reason: "unsupported kind or schema version",
            });
        }
        if self.task_id != task.task_id {
            return Err(ShadowComparisonError::ReportTaskMismatch {
                expected: task.task_id.clone(),
                actual: self.task_id.clone(),
            });
        }
        if self.samples.is_empty() {
            return Err(ShadowComparisonError::EmptyReport);
        }
        let config = ShadowComparisonConfig {
            sample_capacity: self.samples.len(),
            tensors: self.tolerances.clone(),
        };
        let mut comparator = ShadowComparator::new(task.clone(), config)?;
        for sample in &self.samples {
            let recomputed = comparator.compare(
                HardwareObservation {
                    sequence: sample.hardware_sequence,
                    received_at_ms: sample.hardware_received_at_ms,
                    values: sample.hardware_values.clone(),
                },
                sample.simulation_step,
                sample.simulation_time_ticks,
                sample.simulation_values.clone(),
            )?;
            if recomputed != sample {
                return Err(ShadowComparisonError::InvalidReport {
                    reason: "sample does not match replayed observation vectors",
                });
            }
        }
        if comparator.finish()? != *self {
            return Err(ShadowComparisonError::InvalidReport {
                reason: "summary does not match replayed samples",
            });
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct ShadowElementTolerance {
    tensor_name: String,
    tensor_element: usize,
    unit: String,
    absolute_tolerance: f64,
}

/// Bounded deterministic shadow observation comparator.
#[derive(Debug)]
pub struct ShadowComparator {
    task_id: String,
    config: ShadowComparisonConfig,
    observation_dtypes: Vec<TensorDType>,
    elements: Vec<ShadowElementTolerance>,
    samples: Vec<ShadowComparisonSample>,
    last_hardware_sequence: Option<u64>,
    last_hardware_received_at_ms: Option<u64>,
    last_simulation_step: Option<u64>,
    last_simulation_time_ticks: Option<u64>,
    compared_elements: usize,
    violating_samples: usize,
    violating_elements: usize,
    max_absolute_error: f64,
    sum_absolute_error: f64,
}

impl ShadowComparator {
    /// Binds a validated TaskSpec to one complete ordered tolerance contract.
    pub fn new(
        task: TaskSpec,
        config: ShadowComparisonConfig,
    ) -> Result<Self, ShadowComparisonError> {
        task.validate()?;
        if config.sample_capacity == 0 {
            return Err(ShadowComparisonError::ZeroCapacity);
        }
        if config.tensors.len() != task.observation.tensors.len() {
            return Err(ShadowComparisonError::ToleranceCount {
                expected: task.observation.tensors.len(),
                actual: config.tensors.len(),
            });
        }
        let observation_dtypes = flatten_observation_dtypes(&task.observation.tensors)?;
        let mut elements = Vec::with_capacity(observation_dtypes.len());
        for (tensor_index, (tensor, tolerance)) in task
            .observation
            .tensors
            .iter()
            .zip(&config.tensors)
            .enumerate()
        {
            if tolerance.tensor_name != tensor.name {
                return Err(ShadowComparisonError::ToleranceName {
                    index: tensor_index,
                    expected: tensor.name.clone(),
                    actual: tolerance.tensor_name.clone(),
                });
            }
            if !tolerance.absolute_tolerance.is_finite() || tolerance.absolute_tolerance < 0.0 {
                return Err(ShadowComparisonError::InvalidTolerance {
                    tensor: tensor.name.clone(),
                });
            }
            if !matches!(tensor.dtype, TensorDType::F32 | TensorDType::F64)
                && tolerance.absolute_tolerance != 0.0
            {
                return Err(ShadowComparisonError::NonFloatTolerance {
                    tensor: tensor.name.clone(),
                    dtype: tensor.dtype,
                });
            }
            let tensor_elements = tensor
                .shape
                .iter()
                .try_fold(1_usize, |total, dimension| total.checked_mul(*dimension));
            let Some(tensor_elements) = tensor_elements else {
                return Err(ShadowComparisonError::ElementCountOverflow {
                    tensor: tensor.name.clone(),
                });
            };
            for tensor_element in 0..tensor_elements {
                elements.push(ShadowElementTolerance {
                    tensor_name: tensor.name.clone(),
                    tensor_element,
                    unit: tensor.unit.clone(),
                    absolute_tolerance: tolerance.absolute_tolerance,
                });
            }
        }
        Ok(Self {
            task_id: task.task_id,
            samples: Vec::with_capacity(config.sample_capacity),
            config,
            observation_dtypes,
            elements,
            last_hardware_sequence: None,
            last_hardware_received_at_ms: None,
            last_simulation_step: None,
            last_simulation_time_ticks: None,
            compared_elements: 0,
            violating_samples: 0,
            violating_elements: 0,
            max_absolute_error: 0.0,
            sum_absolute_error: 0.0,
        })
    }

    /// Compares one hardware observation with one deterministic simulation step.
    pub fn compare(
        &mut self,
        mut hardware: HardwareObservation,
        simulation_step: u64,
        simulation_time_ticks: u64,
        mut simulation_values: Vec<f64>,
    ) -> Result<&ShadowComparisonSample, ShadowComparisonError> {
        if self.samples.len() == self.config.sample_capacity {
            return Err(ShadowComparisonError::CapacityExceeded {
                capacity: self.config.sample_capacity,
            });
        }
        if let Some(previous) = self.last_hardware_sequence {
            if hardware.sequence <= previous {
                return Err(ShadowComparisonError::NonMonotonicHardwareSequence {
                    previous,
                    actual: hardware.sequence,
                });
            }
        }
        if let Some(previous) = self.last_hardware_received_at_ms {
            if hardware.received_at_ms < previous {
                return Err(ShadowComparisonError::HostTimeRegression {
                    previous_ms: previous,
                    actual_ms: hardware.received_at_ms,
                });
            }
        }
        if let Some(previous) = self.last_simulation_step {
            if simulation_step <= previous {
                return Err(ShadowComparisonError::NonMonotonicSimulationStep {
                    previous,
                    actual: simulation_step,
                });
            }
        }
        if let Some(previous) = self.last_simulation_time_ticks {
            if simulation_time_ticks <= previous {
                return Err(ShadowComparisonError::NonMonotonicSimulationTime {
                    previous,
                    actual: simulation_time_ticks,
                });
            }
        }
        normalize_observation_values(&self.observation_dtypes, &mut hardware.values)?;
        normalize_observation_values(&self.observation_dtypes, &mut simulation_values)?;

        let mut max_absolute_error = 0.0_f64;
        let mut sum_absolute_error = 0.0_f64;
        let mut violating_elements = 0_usize;
        let mut first_violation = None;
        for (index, ((hardware_value, simulation_value), element)) in hardware
            .values
            .iter()
            .zip(&simulation_values)
            .zip(&self.elements)
            .enumerate()
        {
            let absolute_error = (*hardware_value - *simulation_value).abs();
            if !absolute_error.is_finite() {
                return Err(ShadowComparisonError::MetricOverflow { index });
            }
            max_absolute_error = max_absolute_error.max(absolute_error);
            sum_absolute_error += absolute_error;
            if !sum_absolute_error.is_finite() {
                return Err(ShadowComparisonError::MetricOverflow { index });
            }
            if absolute_error > element.absolute_tolerance {
                violating_elements += 1;
                if first_violation.is_none() {
                    first_violation = Some(ShadowViolation {
                        tensor_name: element.tensor_name.clone(),
                        tensor_element: element.tensor_element,
                        unit: element.unit.clone(),
                        hardware_value: *hardware_value,
                        simulation_value: *simulation_value,
                        absolute_error,
                        absolute_tolerance: element.absolute_tolerance,
                    });
                }
            }
        }
        let compared = self.elements.len();
        let mean_absolute_error = sum_absolute_error / compared as f64;
        let compared_elements = self
            .compared_elements
            .checked_add(compared)
            .ok_or(ShadowComparisonError::CountOverflow)?;
        let total_violating_elements = self
            .violating_elements
            .checked_add(violating_elements)
            .ok_or(ShadowComparisonError::CountOverflow)?;
        let violating_samples = if violating_elements > 0 {
            self.violating_samples
                .checked_add(1)
                .ok_or(ShadowComparisonError::CountOverflow)?
        } else {
            self.violating_samples
        };
        let total_sum_absolute_error = self.sum_absolute_error + sum_absolute_error;
        if !total_sum_absolute_error.is_finite() {
            return Err(ShadowComparisonError::MetricOverflow {
                index: compared.saturating_sub(1),
            });
        }
        self.compared_elements = compared_elements;
        self.violating_elements = total_violating_elements;
        self.violating_samples = violating_samples;
        self.max_absolute_error = self.max_absolute_error.max(max_absolute_error);
        self.sum_absolute_error = total_sum_absolute_error;
        self.last_hardware_sequence = Some(hardware.sequence);
        self.last_hardware_received_at_ms = Some(hardware.received_at_ms);
        self.last_simulation_step = Some(simulation_step);
        self.last_simulation_time_ticks = Some(simulation_time_ticks);
        self.samples.push(ShadowComparisonSample {
            hardware_sequence: hardware.sequence,
            hardware_received_at_ms: hardware.received_at_ms,
            simulation_step,
            simulation_time_ticks,
            hardware_values: hardware.values,
            simulation_values,
            max_absolute_error,
            sum_absolute_error,
            mean_absolute_error,
            violating_elements,
            first_violation,
        });
        Ok(self.samples.last().expect("sample was just appended"))
    }

    /// Finishes a non-empty comparison and computes the deterministic verdict.
    pub fn finish(self) -> Result<ShadowComparisonReport, ShadowComparisonError> {
        if self.samples.is_empty() {
            return Err(ShadowComparisonError::EmptyReport);
        }
        Ok(ShadowComparisonReport {
            kind: SHADOW_COMPARISON_REPORT_KIND.to_string(),
            schema_version: SHADOW_COMPARISON_SCHEMA_VERSION,
            task_id: self.task_id,
            tolerances: self.config.tensors,
            summary: ShadowComparisonSummary {
                compared_samples: self.samples.len(),
                compared_elements: self.compared_elements,
                violating_samples: self.violating_samples,
                violating_elements: self.violating_elements,
                max_absolute_error: self.max_absolute_error,
                mean_absolute_error: self.sum_absolute_error / self.compared_elements as f64,
                passed: self.violating_elements == 0,
            },
            samples: self.samples,
        })
    }
}

/// Failure constructing or executing a bounded shadow comparison.
#[derive(Debug, thiserror::Error)]
pub enum ShadowComparisonError {
    /// The portable task contract is invalid.
    #[error(transparent)]
    Task(#[from] TaskSpecValidationError),
    /// The observation space cannot be flattened by the hardware boundary.
    #[error(transparent)]
    ObservationContract(#[from] GatewayBuildError),
    /// The report cannot retain any samples.
    #[error("shadow sample_capacity must be greater than zero")]
    ZeroCapacity,
    /// The tolerance count differs from the TaskSpec tensor count.
    #[error("shadow tolerance count must be {expected}, got {actual}")]
    ToleranceCount {
        /// TaskSpec tensor count.
        expected: usize,
        /// Supplied tolerance count.
        actual: usize,
    },
    /// A tolerance entry is not in exact TaskSpec order.
    #[error("shadow tolerance {index} must name {expected:?}, got {actual:?}")]
    ToleranceName {
        /// Tensor index.
        index: usize,
        /// Required tensor name.
        expected: String,
        /// Supplied tensor name.
        actual: String,
    },
    /// An absolute tolerance is negative, NaN, or infinite.
    #[error("shadow tolerance for {tensor:?} must be finite and non-negative")]
    InvalidTolerance {
        /// Invalid tensor name.
        tensor: String,
    },
    /// Discrete observation tensors require exact comparison.
    #[error("shadow tolerance for discrete tensor {tensor:?} ({dtype:?}) must be zero")]
    NonFloatTolerance {
        /// Discrete tensor name.
        tensor: String,
        /// Discrete tensor dtype.
        dtype: TensorDType,
    },
    /// A tensor shape overflowed the host index type.
    #[error("shadow tensor element count overflowed for {tensor:?}")]
    ElementCountOverflow {
        /// Tensor name.
        tensor: String,
    },
    /// The retained sample bound was reached.
    #[error("shadow sample capacity {capacity} exceeded")]
    CapacityExceeded {
        /// Configured sample bound.
        capacity: usize,
    },
    /// Hardware observation sequences must increase.
    #[error("shadow hardware sequence {actual} must be greater than {previous}")]
    NonMonotonicHardwareSequence {
        /// Previous sequence.
        previous: u64,
        /// Rejected sequence.
        actual: u64,
    },
    /// Injected host receipt ticks must not regress.
    #[error("shadow host tick regressed from {previous_ms} to {actual_ms}")]
    HostTimeRegression {
        /// Previous host tick.
        previous_ms: u64,
        /// Rejected host tick.
        actual_ms: u64,
    },
    /// Paired simulation steps must increase.
    #[error("shadow simulation step {actual} must be greater than {previous}")]
    NonMonotonicSimulationStep {
        /// Previous simulation step.
        previous: u64,
        /// Rejected simulation step.
        actual: u64,
    },
    /// Paired SimClock timestamps must increase with simulation steps.
    #[error("shadow simulation time {actual} must be greater than {previous}")]
    NonMonotonicSimulationTime {
        /// Previous simulation time in ticks.
        previous: u64,
        /// Rejected simulation time in ticks.
        actual: u64,
    },
    /// A hardware or simulation observation violates TaskSpec shape or dtype.
    #[error(transparent)]
    Observation(#[from] GatewayError),
    /// Absolute-error accumulation overflowed.
    #[error("shadow metric overflowed at flattened element {index}")]
    MetricOverflow {
        /// Flattened element index.
        index: usize,
    },
    /// A deterministic report counter overflowed.
    #[error("shadow comparison count overflowed")]
    CountOverflow,
    /// A report cannot be produced without a comparison.
    #[error("shadow comparison report must contain at least one sample")]
    EmptyReport,
    /// A report discriminator or retained metric is internally inconsistent.
    #[error("invalid shadow comparison report: {reason}")]
    InvalidReport {
        /// First failed report invariant.
        reason: &'static str,
    },
    /// An untrusted report names a different TaskSpec.
    #[error("shadow report task mismatch: expected {expected:?}, got {actual:?}")]
    ReportTaskMismatch {
        /// TaskSpec identity.
        expected: String,
        /// Report identity.
        actual: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ai::{
        ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TensorBounds,
        TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
    };

    fn task() -> TaskSpec {
        TaskSpec::new(
            "rne.shadow.test.v1",
            0.01,
            ObservationSpec::new(vec![
                TensorSpec::new("position_m", TensorDType::F64, vec![2], "m"),
                TensorSpec::new("contact", TensorDType::Bool, vec![], "1"),
            ]),
            ActionSpec::new(vec![TensorSpec::new(
                "command",
                TensorDType::F64,
                vec![1],
                "1",
            )
            .with_bounds(TensorBounds::broadcast(-1.0, 1.0))]),
            RewardSpec::weighted_sum(vec![RewardTermSpec::new("step", -0.01, "1")]),
            TerminationSpec::new(
                vec![TerminationConditionSpec::new(
                    "done",
                    TerminationKind::Success,
                )],
                Some(10),
            ),
            ResetSpec::splitmix64(true),
        )
    }

    fn config(capacity: usize) -> ShadowComparisonConfig {
        ShadowComparisonConfig {
            sample_capacity: capacity,
            tensors: vec![
                ShadowTensorTolerance {
                    tensor_name: "position_m".into(),
                    absolute_tolerance: 0.1,
                },
                ShadowTensorTolerance {
                    tensor_name: "contact".into(),
                    absolute_tolerance: 0.0,
                },
            ],
        }
    }

    #[test]
    fn report_preserves_first_task_order_violation() {
        let mut comparator = ShadowComparator::new(task(), config(2)).unwrap();
        comparator
            .compare(
                HardwareObservation {
                    sequence: 1,
                    received_at_ms: 10,
                    values: vec![1.0, 2.0, 1.0],
                },
                4,
                40,
                vec![1.05, 2.2, 0.0],
            )
            .unwrap();
        let report = comparator.finish().unwrap();
        assert!(!report.summary.passed);
        assert_eq!(report.summary.violating_elements, 2);
        assert_eq!(
            report.samples[0]
                .first_violation
                .as_ref()
                .map(|violation| (&*violation.tensor_name, violation.tensor_element)),
            Some(("position_m", 1))
        );
    }

    #[test]
    fn capacity_and_discrete_tolerance_fail_closed() {
        let mut invalid = config(1);
        invalid.tensors[1].absolute_tolerance = 1.0;
        assert!(matches!(
            ShadowComparator::new(task(), invalid),
            Err(ShadowComparisonError::NonFloatTolerance { .. })
        ));

        let mut comparator = ShadowComparator::new(task(), config(1)).unwrap();
        comparator
            .compare(
                HardwareObservation {
                    sequence: 1,
                    received_at_ms: 1,
                    values: vec![0.0, 0.0, 0.0],
                },
                1,
                1,
                vec![0.0, 0.0, 0.0],
            )
            .unwrap();
        assert!(matches!(
            comparator.compare(
                HardwareObservation {
                    sequence: 2,
                    received_at_ms: 2,
                    values: vec![0.0, 0.0, 0.0],
                },
                2,
                2,
                vec![0.0, 0.0, 0.0],
            ),
            Err(ShadowComparisonError::CapacityExceeded { capacity: 1 })
        ));
    }
}
