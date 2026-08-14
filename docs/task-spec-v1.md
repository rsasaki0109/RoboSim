# Portable TaskSpec v1

`rne_ai::TaskSpec` is the portable boundary between an RNE task and Rust,
Python, Gymnasium, accelerator, dataset, or hardware consumers. It describes
data and episode semantics; it does not own a learning algorithm or a physics
backend.

```rust
use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec,
    TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind,
    TerminationSpec,
};

let spec = TaskSpec::new(
    "example.velocity.v1",
    0.02,
    ObservationSpec::new(vec![TensorSpec::new(
        "velocity_m_s",
        TensorDType::F32,
        vec![3],
        "m/s",
    )]),
    ActionSpec::new(vec![TensorSpec::new(
        "target_velocity_m_s",
        TensorDType::F32,
        vec![3],
        "m/s",
    )
    .with_bounds(TensorBounds::broadcast(-2.0, 2.0))]),
    RewardSpec::weighted_sum(vec![RewardTermSpec::new("tracking", 1.0, "1")]),
    TerminationSpec::new(
        vec![TerminationConditionSpec::new(
            "target_reached",
            TerminationKind::Success,
        )],
        Some(500),
    ),
    ResetSpec::splitmix64(true),
);
spec.validate()?;
# Ok::<(), rne_ai::TaskSpecValidationError>(())
```

The serialized order of every tensor, reward term, terminal condition,
curriculum stage, and randomization parameter is consumer-visible and must be
preserved. Use `"1"` for dimensionless values and explicit unit symbols for all
other fields. Scalar tensors use an empty shape; all tensors use row-major
logical ordering.

A single environment is lane zero. Its episode seed is
`derive_episode_seed(root_seed, 0, episode_index)`. A batch uses the same
function with each stable lane ID, so increasing batch width or partially
resetting another lane cannot perturb an existing lane.

The canonical schema example is
[`tests/golden/tasks/task-spec-v1.json`](../tests/golden/tasks/task-spec-v1.json).
Compatibility rationale and versioning rules are recorded in
[`ADR 013`](adr/013-portable-task-contract.md).

## Deterministic CPU batch runner

Use `PortableBatchRunner::from_task_spec` to bind the validated contract to an
episode factory. The factory receives the exact seed for a lane-local episode.
`reset_lanes(&[...])` accepts canonical increasing lane IDs and returns only
those lanes in the same order. Every full reset or batch step reports lane IDs,
episode indices, episode seeds, and a reset mask alongside observations,
rewards, termination, and truncation.

Auto-reset preserves the terminal transition. If lane 3 ends during one call,
that call still returns its terminal observation and reward; on the next call,
lane 3 returns a reset observation with `resets[3] == true` and its supplied
action is ignored. Other lanes step normally.

Checkpoint schema v2 embeds the TaskSpec and chronological steps/partial
resets. Restoring replays from the implicit full reset and verifies each lane's
episode identity, seed, pending auto-reset state, and digest. The canonical
shape is
[`tests/golden/tasks/vectorized-checkpoint-v2.json`](../tests/golden/tasks/vectorized-checkpoint-v2.json).

The outer-harness CPU scaling command and its measurement rules are documented
in [`TASK_SCALE_BENCHMARK.md`](TASK_SCALE_BENCHMARK.md). It covers 1, 16, 256,
and 4096 lanes while keeping wall-clock values outside correctness digests.
