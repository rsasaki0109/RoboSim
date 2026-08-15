# ADR 018: Language-boundary compatibility manifests

- Status: Accepted
- Date: 2026-08-15

## Context

Rust SemVer checks and dynamic-plugin conformance covered source-level Rust API
changes and observable controller behavior, but neither proved the native C
memory layout nor the installed Python call shape. The release bundle exposed a
Rust SDK and ABI3 wheel without a C header or an exhaustive machine-readable
inventory of Python names and signatures.

## Decision

Ship a dependency-free C/C++ header from `rne_plugin_sdk` and retain one strict,
content-addressed schema-v1 layout fixture for the supported 64-bit Linux and
Windows targets. The fixture fixes capability values, structure sizes,
alignments, field offsets, required symbols, introduction versions, and
normalized C signatures. Rust layout tests and the installed compatibility
runner validate it.

Retain the public `rne_py` module surface in a separate strict schema-v1
manifest. It records every public export, constant value, class constructor,
method text signature, and property name. Source Python CI and the extracted
ABI3 wheel must reproduce it exactly and emit a deterministic report.

The manifests are candidate-freeze evidence, not permission to declare 1.0.
Rust API history remains enforced by `cargo-semver-checks`, and independent use
and the six-month stability clock remain separate gates.

## Consequences

- C and C++ authors have a canonical header that does not depend on Cargo.
- Accidental padding, calling-surface, symbol, Python keyword, or property drift
  fails before release.
- Adding a Python export is an intentional manifest update even when additive;
  removing or changing one requires the documented compatibility process.
- A future 32-bit tier needs its own explicit layout fixture rather than
  relabelling the 64-bit evidence.
