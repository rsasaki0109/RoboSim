# Robot Behavior CI Plan

Status: Proposed

## Goal

Differentiate RNE as a deterministic robot-behavior test engine rather than
competing primarily on photorealism, asset-library size, or maximum GPU training
throughput.

The first complete workflow should run the existing G1 + Dex3 task across many
seeded workpiece placements, evaluate behavioral contracts headlessly, and
produce a deterministic failure artifact that can be inspected in a browser.

The intended command-line experience is:

```bash
rne test scenarios/g1_dex3_pick.rne.test.toml --seeds 0..99
```

A failure should identify the violated contract, first failing simulation step,
relevant entities, and replay artifact:

```text
FAIL seed=37 step=83
contract=no_tray_interference
contact=right_hand_palm_link <-> dex3_inspection_tray
replay=artifacts/g1-dex3-seed-37.rne-replay
```

## Product thesis

RNE already has most of the required foundation:

- deterministic simulation clocks and seeded random streams;
- headless episodes with typed observations and actions;
- world snapshots, replay metadata, and stable hashes;
- robot, sensor, actuator, agent, and episode entities;
- a typed DataBus independent of ROS2;
- real URDF articulations and contact observations;
- native wgpu rendering and a browser viewer.

Robot Behavior CI combines these pieces into one product surface: robot tasks
become executable regression tests that can run locally and on every pull
request.

## Design principles

1. Deterministic by construction. Every failure must carry the seed, timestep,
   scenario digest, engine compatibility metadata, and action history required
   to reproduce it.
2. Headless first. Rendering must never be required to evaluate a contract.
3. Typed core before text DSL. Prove the contract model through Rust APIs and
   the G1 task before stabilizing the TOML schema.
4. Outcome contracts over exact trajectories. Exact hashes are useful for
   replay diagnostics, while public behavior tests should normally express
   tolerances and task outcomes.
5. Backend neutral. No Rapier handles or Rapier-specific types may enter the
   public contract API.
6. ROS2 remains an adapter. Contract evaluation belongs in native RNE crates;
   ROS2 and real-hardware execution can be added later through adapters.
7. Failure artifacts are first-class. A failed run is not complete until it
   explains what failed and provides a reproducible artifact.

## Non-goals

- Replacing physics-engine validation with behavior contracts.
- Building a general-purpose temporal-logic language in the first milestone.
- Requiring a renderer, GPU, ROS2 installation, or external service in tests.
- Promising bit-identical physics across different operating systems or physics
  backends.
- Competing immediately with GPU-native simulators on millions of parallel
  environments.

## Initial G1 + Dex3 contracts

The first scenario will cover contracts already grounded in current episode
observations and contact data:

- `dual_contact_before_grasp`: attachment cannot begin from one-sided contact;
- `stable_contact_for_3_steps`: the grasp gate remains valid consecutively;
- `no_inactive_hand_contact`: the unused hand never contacts workcell objects;
- `no_tray_contact_before_place`: the working arm and payload avoid the tray
  until the placement phase;
- `grasp_within_5_seconds`: randomized acquisition completes within a deadline;
- `payload_never_teleports`: per-step payload displacement stays below a bound;
- `place_within_zone`: the fixed task releases inside the named marker;
- `settled_place`: released payload speed is below the configured limit;
- `deterministic_replay`: identical seed and actions produce identical results.

## Contract model

Phase 1 needs only three temporal forms:

- `Always`: a predicate must remain true for every evaluated step;
- `Eventually`: a predicate must become true before a deadline;
- `Consecutive`: a predicate must remain true for a required number of steps.

The initial Rust API should keep predicates typed and task-owned. A later stable
TOML representation may look like:

```toml
[scenario]
scene = "../assets/scenes/unitree_g1_dex3_pick_place.rne.scene.toml"
episode = "unitree_g1_dex3_pick_place"

[[contracts]]
name = "acquire_part"
eventually = { within_s = 5.0, condition = "part.grasped" }

[[contracts]]
name = "no_tray_interference"
always = "inactive_hand.contacts(tray) == false"

[[contracts]]
name = "stable_grasp"
consecutive = { steps = 3, condition = "thumb.contact && index.contact" }
```

This syntax is illustrative until the typed API and the first scenario establish
the necessary semantics.

## Milestones

### Phase 1: Typed Behavior Contract MVP

Deliverables:

- backend-neutral contract result and violation data types;
- `Always`, `Eventually`, and `Consecutive` evaluators using `SimClock` time;
- a deterministic multi-seed runner;
- G1 + Dex3 contract predicates;
- JSON summary and JUnit XML output;
- unit tests for each evaluator and a headless G1 integration test.

Likely ownership:

- contract types and evaluation: `crates/rne_ai`;
- stable report structures: `crates/rne_data` only if they are useful outside AI;
- CLI entry point: a new `crates/rne_cli` subcommand or `xtask` prototype;
- G1 scenario: `tests/integration` plus a runnable example.

Definition of done:

- ten G1 seeds run from one command;
- an intentionally invalid tray layout is detected;
- repeated runs report the same seed, step, and contract;
- output is valid JSON and JUnit XML;
- no rendering or ROS2 dependency is introduced.

### Phase 2: Deterministic Failure Replay

Deliverables:

- record actions, selected observations, contact events, world hashes, and
  compatibility metadata;
- identify and preserve the first failing step;
- serialize a versioned `.rne-replay` bundle;
- replay the bundle headlessly and verify the same violation;
- load the bundle in the existing browser viewer;
- optionally render a short failure GIF as a CI artifact, never as a test
  prerequisite.

Definition of done:

- a replay produced on failure reproduces the same contract violation;
- incompatible replay or engine versions fail with a clear error;
- the browser viewer can inspect the failing interval without rerunning policy
  code.

### Phase 3: Failure Minimization

Deliverables:

- represent randomization parameters as stable named dimensions;
- disable unrelated dimensions while preserving the failure;
- shrink continuous ranges using deterministic bisection;
- shrink discrete choices using deterministic ordered elimination;
- write the minimized case as a standalone scenario override;
- verify the minimized scenario before reporting success.

Definition of done:

- a multi-parameter G1 failure is reduced to the smallest known reproducing
  parameter set;
- minimization itself is deterministic;
- the original and minimized replay artifacts name the same violated contract.

### Phase 4: GitHub Actions Integration

Deliverables:

- a reusable Behavior CI workflow;
- JUnit test annotations and a Markdown summary;
- JSON, replay, and optional GIF artifacts;
- comparison against a checked-in or base-branch success-rate baseline;
- configurable pass thresholds and regression budgets.

Definition of done:

- a pull request displays seeds passed, success rate, first failure, and change
  from the baseline;
- artifacts are available for failed seeds;
- the workflow runs without a display server or GPU.

### Phase 5: Simulation Diff

Deliverables:

- execute the same scenario, seed, and action sequence on base and candidate;
- find the first divergent stable world hash;
- report bounded pose, velocity, joint target, contact, observation, reward, and
  termination differences;
- separate expected tolerance-level floating-point differences from semantic
  changes.

Definition of done:

- a controller or scene change reports the first meaningful behavior divergence;
- unchanged scenarios remain clean;
- reports point to named entities and fields rather than raw backend handles.

### Phase 6: Cross-backend and hardware contracts

Long-term targets:

- execute the same outcome contracts with Rapier and a future MuJoCo backend;
- ingest ROS2 or hardware action/observation traces through adapters;
- compare completion, safety, timing, and tolerance-bounded measurements;
- produce one conformance matrix across simulation backends and real runs.

Exact state hashes will remain backend-specific. Cross-backend checks evaluate
behavioral outcomes and documented tolerances.

## Testing strategy

Every phase must add at least one unit or integration test and remain runnable
headlessly.

Required coverage includes:

- evaluator boundary conditions at step zero and at deadlines;
- deterministic ordering across contracts and seeds;
- non-finite observation and tolerance validation;
- replay format version and compatibility rejection;
- first-failure preservation when later failures occur;
- deterministic minimizer ordering;
- G1 success and intentional-failure scenarios;
- stable JSON and JUnit golden serialization.

The normal repository quality gates remain:

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo run -p xtask -- ci
```

## Risks and mitigations

### Over-generalizing the DSL

Risk: implementing a full expression language delays the first useful result.

Mitigation: start with typed G1 predicates and three temporal evaluators. Publish
the TOML grammar only after at least two robot tasks use the same model.

### False regressions from contact sensitivity

Risk: exact state comparisons flag harmless floating-point divergence.

Mitigation: distinguish deterministic same-backend replay hashes from public
tolerance-based outcome contracts.

### Replay files becoming too large

Risk: recording every component every step produces impractical CI artifacts.

Mitigation: store actions, selected observations, events, hashes, periodic
checkpoints, and a bounded window around the first failure.

### Contract predicates depending on task internals

Risk: every task grows a bespoke evaluation path.

Mitigation: standardize semantic observations such as named contact, pose,
velocity, attachment, task marker distance, and episode phase while allowing
task-owned predicates at the boundary.

### Core dependency leakage

Risk: reporting, GitHub, ROS2, or physics-specific concerns enter core crates.

Mitigation: keep core contract types small and immutable; place CLI, CI, ROS2,
rendering, and backend integration at their existing boundaries.

## Release sequence

1. G1 typed contracts and ten-seed runner.
2. JSON/JUnit reports and intentional-failure integration test.
3. Versioned replay bundle and headless replay verification.
4. Browser failure viewer and GitHub Actions artifact flow.
5. One-hundred-seed G1 benchmark and README demonstration.
6. Deterministic failure minimizer.
7. Base-versus-candidate simulation diff.
8. Second robot task to validate generality.
9. Stabilized `.rne.test.toml` schema.
10. Cross-backend and real-hardware conformance experiments.

## Final definition of success

The initial initiative is complete when a contributor can change the G1 scene or
controller, push a pull request, and receive a deterministic Behavior CI report
that:

- executes 100 seeded G1 + Dex3 scenarios headlessly;
- evaluates grasp, placement, stability, and forbidden-contact contracts;
- reports any regression at a stable seed and simulation step;
- provides a replay that reproduces the failure;
- opens the relevant failure interval in a browser;
- does not require ROS2, a GPU, or physics-backend-specific public APIs.
