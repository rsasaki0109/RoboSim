# ADR 011: Backend-neutral determinism contracts

## Status

Accepted for the 0.2 trust-foundation slice.

## Decision

`rne_core` owns a small declarative `DeterminismContract`. A contract records a
stable name, a finite simulation step window, backend-neutral observable names,
and one of three comparison tiers:

- `Exact` — every declared observable must compare exactly;
- `Tolerance` — numeric differences use explicit absolute and relative bounds;
- `Outcome` — a stable semantic criterion must match, without requiring an
  identical trajectory.

The contract is metadata, not an evaluator. Replay, physics, sensor, runner,
and behavior crates provide evidence and may interpret the declared observable
names. No backend handle, renderer type, wall-clock value, or ROS2 type enters
the API. Invalid names, empty scopes, overflowing step windows, and invalid
tolerance values are rejected at construction and validation boundaries.

## Consequences

Exact replay can report stable hashes without implying cross-backend bit
identity. Tolerance and outcome checks can be added for unlike solvers and
task-level behavior while sharing one versioned declaration. Later evidence
artifacts can serialize the contract and evolve independently from the core
comparison implementation.
