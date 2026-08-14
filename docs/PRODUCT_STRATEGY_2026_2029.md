# RNE product strategy, 2026-2029

Status: canonical forward plan
Last updated: 2026-08-14

This document is the source of truth for work after `v0.1.0`. The detailed
milestone records in [ROADMAP.md](ROADMAP.md) remain useful history, but when a
priority or version label conflicts with this strategy, this document wins.

## Executive decision

RNE will not try to become the physics engine with the most solvers, the GPU
trainer with the highest synthetic FPS, or the GUI with the largest model
library. Mature projects already occupy those positions.

RNE will become the **portable robot-task verification runtime**:

> Define a robot task once, execute it through interchangeable simulation or
> hardware adapters, and turn every result or failure into evidence another
> machine can verify.

The product owns five things end to end:

1. backend-neutral world, robot, sensor, actuator, agent, and episode semantics;
2. typed observation, action, reward, time, and failure contracts;
3. deterministic scheduling and explicit exact/tolerance/outcome guarantees;
4. portable reports, replays, datasets, and Failure Capsules;
5. a headless-first workflow that runs locally, in CI, and against hardware.

Physics engines, renderers, ROS 2, learning frameworks, and cloud runners remain
replaceable adapters. They are important integrations, not the identity of the
core.

## Competitive position

The plan deliberately takes the strongest idea from each neighboring project
without duplicating its entire product:

| Project family | Established strength | RNE response |
|---|---|---|
| [Gazebo Sim](https://gazebosim.org/libs/sim/) | Broad sensors, rendering, plugins, GUI, and runtime-selectable physics | Keep RNE headless and evidence-centric; interoperate instead of rebuilding a comparable GUI and asset ecosystem |
| [Choreonoid](https://choreonoid.org/en/documents/latest/simulation/simulator-items.html) | Integrated robot GUI, controller workflows, recording, and several selectable physics engines | Make backend switching testable through one machine-readable conformance and replay contract, with no GUI required |
| [MuJoCo / MJX](https://mujoco.readthedocs.io/en/latest/mjx.html) | Accurate articulated dynamics plus accelerator-scale batched simulation | Promote native MuJoCo behind RNE's physics contract, then evaluate MJX as an external batch adapter rather than putting JAX in core |
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/quickstart.html) | Modular GPU-vectorized learning environments and a large task ecosystem | Freeze a portable TaskSpec and deterministic batch semantics; integrate one accelerator path instead of recreating Isaac Sim |
| [Genesis World](https://genesis-world.readthedocs.io/en/latest/user_guide/configuration/concepts.html) | Multi-physics, portable compute backends, and thousands of parallel environments | Treat it as a possible high-throughput execution adapter after the task contract is stable |
| [CARLA](https://carla.readthedocs.io/en/latest/ts_traffic_simulation_overview/) | Rich autonomous-driving sensors, traffic, and OpenSCENARIO workflows | Use RNE traffic and PLATEAU support only where it strengthens the flagship robot task; do not pursue full AV-simulator parity |

The defensible difference is therefore not another importer or renderer. It is
the combination of task portability, declared determinism, backend conformance,
and a portable failure artifact.

## Product rules

These rules apply to every milestone:

- Own semantic contracts and evidence; integrate specialized engines.
- Keep ROS 2, wall-clock I/O, GPU frameworks, and vendor types outside core.
- A capability is not advertised until a committed case proves it.
- Exact comparison is only claimed within an explicit compatible runtime;
  unlike solvers use named SI-unit tolerances or semantic outcomes.
- Every positive flagship path has an intentionally failing case that produces
  a verifiable Failure Capsule.
- Rendering is optional for simulation and verification.
- Add an importer, robot demo, or backend only when a current milestone needs
  it and the addition exercises reusable core behavior.
- Stable correctness evidence never contains wall-clock timing. Performance
  measurements live in a separate, hardware-named report.
- One milestone is active at a time. Maintenance may proceed in parallel, but
  the project does not run two architectural migrations simultaneously.

## Dependency order

The program follows one critical path:

1. trust and evidence contracts;
2. interchangeable dynamics;
3. portable task and batch contracts;
4. perception and dataset evidence;
5. real-robot and hardware-in-the-loop adapters;
6. a flagship end-to-end validation workflow;
7. ecosystem hardening and compatibility freeze.

Later work may be researched early, but it does not become a production
dependency before the preceding contract is stable.

## Release roadmap

Dates are planning windows, not promises. A release moves only when its exit
evidence passes. Missing a gate moves the date; it does not weaken the gate.

| Milestone | Planning window | Product outcome |
|---|---|---|
| `v0.2` Trust and evidence | Aug-Oct 2026 | A clean checkout can inventory claims, reproduce benchmarks, classify determinism, and carry a failure between machines |
| `v0.3` Interchangeable dynamics | Nov 2026-Feb 2027 | The same supported task fixture runs through Analytic, Rapier, and MuJoCo with honest exact/tolerance contracts |
| `v0.4` Portable tasks and scalable learning | Mar-Jul 2027 | One typed TaskSpec runs as a single environment or a deterministic batch and can be consumed from Rust and Python |
| `v0.5` Perception and dataset evidence | Aug-Dec 2027 | Timestamped sensor runs become portable, calibrated, hash-verified datasets with offline evaluation |
| `v0.6` Sim-to-real and HIL | Jan-Jun 2028 | The same observation/action contract drives simulation, shadow mode, and a bounded real-robot adapter |
| `v0.7` Flagship validation workflow | Jul-Dec 2028 | One mobility-plus-manipulation scenario demonstrates task, perception, traffic, replay, fault injection, and browser inspection together |
| `v0.8` Ecosystem and certification | Jan-Jun 2029 | Third parties can author plugins, backends, controllers, and task bundles and run the conformance kit independently |
| `v0.9` Compatibility freeze | Jul-Dec 2029 | Public candidates for the 1.0 Rust API, C ABI, protocols, CLI, and artifact schemas survive external use without unplanned breaks |
| `v1.0` | Gate-driven, no fixed date | Long-term compatibility begins only after the 1.0 readiness gates are met |

### v0.2: trust and evidence

The implementation foundation was merged to `main` by PR #164:
`DeterminismContract`, capability and benchmark reports, Failure Capsules, the
aggregate evidence gate, and the default-off MuJoCo spike.

Implemented on `main`:

- expose a single `xtask evidence` aggregate that runs capability, benchmark,
  conformance, and capsule-fixture verification;
- publish JSON schemas or equivalent golden shapes for all four new artifacts;
- add a clean-install tutorial that reproduces a committed failure capsule;
- keep all reports timing-free and versioned in `release/contracts.toml`.

The merge gate confirmed the Linux and Windows evidence jobs, full workspace
aggregate, release rehearsal, parity, MSRV, supply-chain, ROS2 adapter, and
dedicated MuJoCo jobs from the same source commit.

Exit evidence:

- two clean machines produce byte-identical stable reports from the committed
  inputs;
- capsule verification rejects tampering, traversal, unsupported schema, and a
  successful replay presented as a failure;
- Windows and Linux default builds require no MuJoCo runtime;
- full workspace, headless, release, and dedicated MuJoCo gates pass.

### v0.3: interchangeable dynamics

This is the immediate next development milestone.

Current implementation status:

- conformance report schema v2 and backend manifest schema v2 are registered;
- the capability catalog is callable through a generic `PhysicsBackend`
  factory for Analytic, Rapier, and feature-gated MuJoCo;
- Rapier synchronizes backend-neutral completed-step `JointState` values into
  ECS, removing the conformance runner's dependency on a Rapier-only getter;
- MuJoCo compiles multiple fixed/dynamic ECS bodies into backend-private MJCF,
  synchronizes pose and velocity at an explicit fixed step, and joins the shared
  rigid-body catalog on Windows and Linux;
- MuJoCo reports canonical physical contacts and zero-impulse sensor overlaps;
  its preflight rejects unadvertised kinematic bodies with a typed capability
  error, and rejects invalid geometry or post-step-0 topology changes before
  they can be approximated;
- Rapier and MuJoCo implement unit-explicit revolute/prismatic position,
  velocity, and effort actuation; MuJoCo now advertises `articulation` and
  passes the same revolute catalog vector as Rapier.

Delivery slices:

1. **Conformance harness v2** — make the existing physics catalog callable by
   any backend, with a backend manifest, required capability cases, tolerance
   registry, and deterministic JSON report.
2. **MuJoCo rigid bodies** — compile the supported ECS sphere/box/capsule slice,
   mass, pose, velocity, gravity, and fixed-step configuration into a private
   MuJoCo model representation.
3. **Articulation and actuation** — add revolute/prismatic joints, limits,
   position/velocity/effort actuation, and named state synchronization.
4. **Contact evidence** — canonical contact pairs, normals, impulses, and a
   documented cross-backend tolerance profile.
5. **Cross-backend diagnosis** — compare the first divergent observable and
   package the relevant replay and conformance report into a Failure Capsule.

All five v0.3 implementation slices now have executable evidence. The
cross-backend diagnostic keeps the production 10 cm Rapier-vs-MuJoCo position
contract passing, injects a separate 1 cm diagnostic bound, captures both
traces through the first violation, and verifies the resulting replay/report
pair through the portable Failure Capsule reader on Windows and Linux CI.

Exit evidence:

- each advertised capability has at least one shared committed vector;
- same-runtime fresh runs are exact where declared;
- Analytic/Rapier/MuJoCo comparisons pass named, unit-bearing tolerances;
- one articulated robot fixture executes unchanged on Rapier and MuJoCo;
- no MuJoCo type appears in `rne_physics`, `rne_robot`, or `rne_world` public
  APIs;
- unsupported geometry or actuation fails before the first step with a precise
  capability error.

### v0.4: portable tasks and scalable learning

RNE should standardize the task before selecting a GPU stack.

Delivery slices:

- freeze versioned `TaskSpec`, `ObservationSpec`, `ActionSpec`, `RewardSpec`,
  termination, reset, curriculum, and randomization schemas;
- define deterministic batch lane identity, seeded reset streams, partial
  reset, checkpoint, and stable output ordering;
- provide a CPU reference batch runner and zero-copy-friendly Rust/Python array
  views where safe;
- provide Gymnasium compatibility and keep learning algorithms outside core;
- benchmark one task at 1, 16, 256, and 4096 environments where supported;
- select exactly one accelerator adapter after a measured spike comparing MJX,
  Genesis World, and Isaac Lab integration cost.

Exit evidence:

- lane zero in a batch reproduces the equivalent single-environment replay;
- changing batch width does not change a lane's seeded episode sequence;
- checkpoints restore observations, actions, rewards, termination, and random
  state;
- Python and Rust consumers agree on schema, shape, dtype, units, and ordering;
- throughput reports name hardware, backend, precision, batch size, warm-up,
  and task; no performance value participates in correctness hashes.

### v0.5: perception and dataset evidence

Delivery slices:

- freeze calibration, frame, capture-time, availability-time, latency, noise,
  and ground-truth annotation contracts;
- stream RGB, depth, LiDAR, IMU, transforms, actions, and task outcomes into a
  versioned dataset bundle without retaining the entire run in memory;
- record seeded domain-randomization decisions and asset digests;
- provide offline validation and metric runners that do not load a renderer;
- keep wgpu as the portable baseline and evaluate external high-fidelity
  rendering only through an adapter.

Exit evidence:

- a dataset verifies all payload hashes, stream ordering, calibration, units,
  and timestamps;
- the same seeded capture reproduces its declared exact or tolerance contract;
- delayed and dropped sensor frames remain distinguishable from absent data;
- an offline evaluator reproduces the committed perception metrics headlessly.

### v0.6: sim-to-real and hardware in the loop

Delivery slices:

- define a bounded hardware gateway outside core with explicit deadlines,
  stale-data policy, actuator limits, and fail-closed behavior;
- map simulated and recorded hardware observations/actions to the same TaskSpec;
- support playback, shadow, HIL, and live modes without teaching simulation
  logic about wall-clock time;
- harden the ROS 2 adapter and retain a direct C/Python controller path;
- choose one affordable reference robot before implementation; brand-specific
  types remain in an adapter.

Exit evidence:

- a process-level hardware mock proves timeout, disconnect, reconnect, stale
  command, limit, and emergency-stop behavior;
- shadow mode compares live observations with a simulation rollout without
  sending actuations;
- a recorded hardware failure can be inspected through the same evidence tools
  as a simulation failure;
- core crates remain ROS 2- and wall-clock-free.

### v0.7: flagship validation workflow

Build one memorable workflow instead of another collection of disconnected
demos. The target is a mobile manipulator completing an inspection and
pick/place task while sharing a structured environment with traffic or other
agents.

The workflow must combine:

- imported robot and environment assets;
- navigation, manipulation, perception, and typed policy evaluation;
- native and at least one interchangeable physics execution path;
- deterministic scenario events and seeded fault injection;
- a successful run, an intentionally failing run, and a minimized Failure
  Capsule;
- headless CI plus browser-based replay inspection.

Exit evidence is a single clean-checkout command that reproduces the success
and failure on Windows and Linux. Every subsystem included in the demo must be
replaceable or testable independently; no special-case core logic is allowed.

### v0.8-v0.9: ecosystem and compatibility

The final pre-1.0 program focuses on other people successfully extending RNE:

- split authoring SDKs from internal implementation crates where useful;
- ship scaffolds, manifests, examples, and a standalone conformance kit;
- sign release artifacts and include dependency/license provenance;
- certify one third-party controller plugin and one independently maintained
  backend or adapter;
- freeze candidate Rust APIs, C ABI, frontend protocol, TaskSpec, replay,
  dataset, and Failure Capsule formats;
- provide explicit migration notes and compatibility fixtures for every break.

`v0.9` lasts as long as necessary. A calendar date or GitHub star count does
not turn it into 1.0.

## 1.0 readiness gates

RNE 1.0 is allowed only when all of the following are true:

- the candidate stable surfaces complete at least six months of real use
  without an unplanned breaking change;
- at least two external projects reproduce a task and Failure Capsule without
  repository-author assistance;
- at least one third-party plugin and one external backend or hardware adapter
  pass the published conformance kit;
- the flagship workflow installs from release artifacts and passes on Windows
  and Linux;
- current and supported historical artifacts fail or load according to the
  published compatibility policy;
- there are no open P0/P1 correctness, safety, or supply-chain blockers;
- the maintainers can support the promised API, ABI, protocol, and schema
  compatibility for the documented support period.

Stars may indicate awareness, but they are not an engineering readiness gate.
If the external-use gates are not met, the project remains at 0.x.

## The next 12 weeks

The immediate execution order is intentionally narrow:

| Weeks | Work | Demonstrable result |
|---|---|---|
| 1-2 | Land and release-harden the v0.2 trust foundation | One aggregate evidence command and a clean-install capsule tutorial |
| 3-5 | Extract conformance harness v2 and backend manifest | Analytic and Rapier run through the new generic harness with unchanged results |
| 6-9 | Promote MuJoCo rigid-body compilation and synchronization | Shared free-fall and impulse fixtures pass exact/tolerance contracts |
| 10-11 | Add the first revolute joint and actuator vector | One minimal articulated fixture runs on Rapier and MuJoCo |
| 12 | Cross-backend divergence diagnostics and review | A deliberately perturbed run emits the first divergence and a verified capsule |

No new importer, renderer, physics backend, or large demo enters these twelve
weeks unless it is necessary for one of those results.

## Portfolio and maintenance policy

Default engineering allocation over a milestone:

- 45% contract, correctness, tests, and compatibility;
- 25% the current end-to-end product slice;
- 15% developer experience, installability, and documentation;
- 10% performance and external adapters;
- 5% bounded research spikes.

Research spikes are time-boxed and default-off. Promotion requires an ADR,
capability declaration, tests, documentation, CI ownership, and a removal plan
if the integration becomes unmaintained.

## Product metrics

Track outcomes instead of source-file count or demo count:

- **Reproduction:** 100% of committed failures verify from a clean checkout on
  both tier-1 platforms.
- **Portability:** the flagship TaskSpec runs on at least two production physics
  backends and one hardware or accelerator adapter before v0.9.
- **Diagnosis:** a failed CI task identifies the first violating contract and
  emits a capsule without a manual rerun.
- **Time to proof:** a release artifact gets a new user from install to verified
  capsule in at most 15 minutes on the reference machine.
- **CI health:** required pull-request gates stay below 15 minutes at p95 through
  sharding; full and native-backend rehearsals remain below 45 minutes.
- **External proof:** v0.9 requires independent plugin/adapter conformance and
  two external task users, not a star threshold.

## Explicit non-goals through v0.7

- building a new general-purpose rigid-body or GPU physics engine;
- matching Gazebo or Choreonoid's full GUI/editor surface;
- matching Isaac Lab, MJX, or Genesis on raw GPU throughput;
- matching CARLA's complete autonomous-driving sensor and map catalog;
- putting ROS 2, CUDA, JAX, PyTorch, DDS, or vendor SDK types in core crates;
- adding cloud orchestration before the local single-node evidence workflow is
  stable and measured;
- growing the example count without consolidating reusable task, sensor,
  controller, and evidence APIs.

These exclusions are strategic focus, not permanent prohibitions. They can be
reconsidered only when product evidence shows that an adapter cannot satisfy a
real user workflow.
