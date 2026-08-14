# Compatibility and migration policy

This policy applies to Robot Native Engine 0.x. The `0.1.0` release freezes
the supported surface for this milestone; later 0.x releases may tighten
validation but must not silently reinterpret accepted input.

## Supported toolchains and platforms

- MSRV: Rust `1.88.0`.
- CI/reference Rust: `1.95.0` with the committed `Cargo.lock`.
- Tier-1 release rehearsal platforms: GitHub-hosted Linux x86_64 and Windows
  x86_64.
- Python: CPython 3.9 or newer through one ABI3 wheel per platform.
- ROS2 remains an adapter and follows the ROS distribution/toolchain declared
  by the adapter workflow; it is not required by core crates.

An MSRV increase in 0.x requires a minor release and release-note entry. The
locked graph may require a newer compiler than an individual RNE crate when an
optional backend is enabled; the full default workspace is the release MSRV
authority.

## Rust API

Public items in publishable `rne_*` crates remain pre-1.0 and may evolve with
documented migration notes.

- Patch releases may fix behavior without removing, renaming, or changing the
  type of a public item.
- Minor releases may add APIs, deprecate old APIs, or make documented breaking
  changes while the engine remains below 1.0. A deprecated API remains through
  at least the next minor release.
- Once 1.0 is declared, removing or incompatibly changing a stable API
  requires a major release.
- APIs explicitly documented as experimental are exempt until promoted, but
  must remain memory-safe and must produce actionable errors.

`cargo-semver-checks` compares release-candidate changes with the frozen
baseline. Workspace rustdoc runs with warnings denied, and public libraries
deny missing documentation.

The v0.3 interchangeable-dynamics milestone extends the pre-1.0 exhaustive
`PhysicsCapability` and `PhysicsError` enums with `KinematicBody` and
`InvalidActuation`. Downstream exhaustive matches must add arms for those
variants. `KinematicBody` is appended after the frozen capability variants, so
their discriminants and derived ordering do not change. The `rne_physics`
SemVer policy reports enum additions as warnings during 0.x while continuing
to reject removals, payload changes, and variant reordering.

## C ABI and plugin compatibility

Controller ABI v3 is the current authoring ABI. The host continues to load the
frozen v2 compatibility fixture. Plugins must negotiate ABI version and
capabilities before configuration or stepping.

- Unknown ABI versions are rejected before a plugin function is invoked.
- Host-owned lifecycle order remains configure, episode activation/reset,
  robot-scoped step, and shutdown.
- Struct layout, ownership, and nullability rules are part of the ABI contract.
- A future ABI version is additive unless it is assigned a new major number;
  v2 support cannot be removed without an explicit migration note.

Core crates never expose adapter or backend-specific types to make an external
ABI fit.

## Frontend protocol

Runner/frontend protocol v1 is supported throughout 0.x. Negotiation rejects
incompatible major versions before simulation control. Unknown optional frame
kinds may be skipped only when the negotiated capability set allows it;
malformed lengths, limits, or required fields are errors. Slow or disconnected
frontends must not stall simulation or retain unbounded data.

Legacy line-oriented `--control-port` protocol v1 remains available during the
0.x transition. Command acknowledgement precedes the corresponding applied
status boundary.

## Assets, manifests, and reports

Each serialized format carries an explicit version where required. Readers
follow these rules:

- the same schema version is read exactly;
- additive optional fields use stable defaults;
- an unknown major/schema version is rejected with expected and actual values;
- input digests are verified before replay;
- migration never fabricates sensor frames, actions, random state, or physics
  hashes that were absent in the source artifact.

Current 0.1 formats include scene, robot, run, plugin, traffic, replay, Behavior
CI, physics-conformance, scenario-scale, determinism-contract, capability,
benchmark, and Failure Capsule artifacts. OpenSCENARIO, SDF, MJCF, URDF, SUMO,
and PLATEAU inputs are import formats: accepted subsets are documented, and
unsupported constructs remain explicit import errors.

TaskSpec schema v1 rejects unknown fields and validates fixed tensor shape,
dtype, row-major order, units, bounds, reward terms, termination, reset,
curriculum, and randomization before execution. Ordered arrays are semantic.
Portable batch checkpoint schema v2 embeds its TaskSpec and chronological
step/partial-reset operations. The legacy `VectorizedEpisode` API and its
checkpoint remain unchanged for patch compatibility; new portable execution
uses the separately additive `PortableBatchRunner` API.

The evidence-manifest schema inventories one verified run of the capability,
physics-conformance, benchmark, and Failure Capsule gates. It is provenance,
not a claim that compiler- and platform-bearing capsule bytes match across
hosts. Canonical schema-v1 examples live under `tests/golden/evidence/`.

Physics conformance report schema v2 embeds backend-manifest schema v2,
catalog version, tolerance-registry version, declared/runtime capabilities, and
coverage verdicts. Its canonical shape lives at
`tests/golden/physics/conformance-report-v2.json`. A backend identifier with no
registered shared vector or tolerance profile produces a failing case rather
than silently weakening coverage.
Manifest schema v2 adds `kinematic_body` as a refinement of `rigid_body`.
Analytic and Rapier prove it with the shared external-pose vector; MuJoCo
rejects it at preflight with `MissingCapabilities` before native compilation.

`JointActuation` is a tagged, backend-neutral ECS command with distinct
revolute/prismatic position, velocity, and effort variants. Field names carry
their SI units. A backend rejects unknown variants, non-finite values, negative
gains/limits, and joint-kind mismatches before a physics step.

The machine-readable values frozen by this policy live in
`release/contracts.toml`. The release gate compares them with the compiled ABI,
transport, asset, replay, physics, determinism, task, accelerator-selection,
and evidence constants; changing one side alone fails.

## Replay migration

Replay artifacts are evidence, not mutable project files. Exact replay requires
the recorded engine compatibility version, schema, source assets, fixed clock,
and input digests. When an older artifact is rejected:

1. retain the original artifact unchanged;
2. install the recorded RNE version and verify it there;
3. rerun the original source manifest with the new engine to create a new
   artifact;
4. compare named diagnostics and document any expected outcome change.

There is deliberately no byte-rewriting migration that claims an old run was
produced by a new engine. Declarative scene/robot/run assets may gain a separate
lossless migration command in a future minor release.

## Python API

The wheel uses `abi3-py39`. Python class, method, task, and keyword names follow
the same documented 0.x compatibility rules as public Rust APIs. Pickled Python objects are
not a stable artifact format; use versioned RNE JSON/replay/checkpoint formats.

## Determinism compatibility

Stable hashes prove repetition under one declared engine/version/backend
contract. They do not promise identical floating-point bits across unlike
physics backends. A release that intentionally changes a deterministic result
must update its golden evidence and explain the semantic reason in the
changelog. Timing values and filesystem paths never participate in stable
simulation digests.

## Security and support

Malformed external input is supported only to the point of safe rejection.
Security reports should use GitHub's private vulnerability reporting for the
repository. P0/P1 release blockers are tracked in `release/blockers.toml`; a
release rehearsal fails while either severity is open.
