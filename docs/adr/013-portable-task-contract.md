# ADR 013: Portable task contract v1

## Status

Accepted for the v0.4 portable-tasks milestone. This milestone label does not
change the public package or release version by itself.

## Context

RNE already has typed Rust episodes and several task-specific vectorized
wrappers. Their Rust structs communicate intent inside one crate, but they do
not give Python, Gymnasium, an accelerator adapter, a dataset reader, or a
hardware gateway one versioned description of shapes, dtypes, units, ordering,
reward terms, termination, reset, curriculum, and randomization.

The existing generic vectorized wrapper also constructs lane seeds as
`root_seed + lane_index`. That identifies an initial lane but does not specify a
lane-local episode stream or establish that partial reset and batch-width
changes preserve a lane's sequence.

## Decision

`rne_ai` owns `TaskSpec` schema v1 and its subordinate `ObservationSpec`,
`ActionSpec`, `RewardSpec`, `TerminationSpec`, `ResetSpec`, `CurriculumSpec`, and
`RandomizationSpec` values. The contract contains no NumPy, Gymnasium, PyTorch,
JAX, ROS 2, renderer, or physics-backend type.

Observation and action spaces are ordered lists of named, fixed-shape tensors.
Each tensor declares a scalar dtype, row-major layout, unit, and optional
flattened numeric bounds. An empty shape is one scalar. Bounds contain either
one broadcast value or one value per row-major element. List order is semantic:
it is the array/view order used by every binding and evidence artifact.

Reward terms are ordered and form one scalar through a declared aggregation.
Terminal success and terminal failure are distinct from step-budget
truncation. Curriculum stages advance by lane-local episode index, never a
global batch reset counter. Randomization parameters are sampled in declared
order once per lane-local reset and must record their sampled values in future
replay and dataset evidence.

Reset schema v1 names `split_mix64_lane_episode_v1`. Its exact implementation is
the public `derive_episode_seed(root_seed, lane_id, episode_index)` function.
Lane IDs are stable non-negative integers; a single environment is lane zero.
The derivation has separate lane and episode domains, so batch width, execution
order, and resets of other lanes do not enter the result.

`PortableBatchRunner::from_task_spec` is the deterministic CPU reference runner.
It retains the validated TaskSpec, reconstructs every reset from the declared
lane/episode seed, emits stable lane metadata, and rejects partial reset unless
the task declares it. Explicit partial resets require strictly increasing,
unique lane IDs and leave every unselected lane untouched. Auto-reset is
deferred until the ended lane's next batch step, so a terminal observation,
reward, and flags are never overwritten by a reset observation.

Vectorized checkpoint schema v2 embeds the TaskSpec and stores chronological
step/partial-reset operations plus lane episode indices, seeds, pending-reset
state, and lane digests. Full reset is the implicit replay origin. Lane digests
exclude batch width and unrelated lanes; the aggregate digest combines lane
digests in ID order. Digests are same-build diagnostics over stable serde JSON,
while the operation log, typed values, and declared exact/tolerance contracts
remain the authoritative portable evidence.

JSON artifacts carry `kind = "rne_task_spec"` and `schema_version = 1`.
`release/contracts.toml` pins the compiled version and a committed golden JSON
pins the v1 field shape. Within v1, unknown JSON fields and invalid values are
rejected. A field addition, removal, rename, meaning change, dtype change, shape
change, unit change, or ordered-list change requires a new task artifact or a
schema-version compatibility decision. Rust schema types are non-exhaustive so
downstream matches and construction do not accidentally freeze internal
implementation details beyond the serialized contract.

## Consequences

Task authors can describe one task independently of its single, batch, Python,
accelerator, dataset, or hardware execution path. Consumers must call
`TaskSpec::validate` after decoding untrusted JSON. Validation rejects malformed
identifiers, units, shapes, bounds, duplicate names, non-finite numbers,
unordered curriculum stages, invalid distributions, and unsupported versions.

Schema v1 deliberately covers fixed-shape dense tensors and scalar weighted
rewards. Variable-length/ragged data and new aggregation forms require a later
schema version instead of framework-specific escape hatches. The next slice
must expose the same contract and runner buffers to Python/Gymnasium and measure
the CPU reference path before selecting an accelerator adapter.
