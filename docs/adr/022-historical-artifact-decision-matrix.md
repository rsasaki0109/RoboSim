# ADR 022: Historical artifact compatibility decisions

- Status: Accepted
- Date: 2026-08-16

## Context

The installed corpus already proves lossless mobile-manipulator snapshot
migration, but not every version number represents one evolvable artifact
family. Generic `VectorizedEpisodeCheckpoint` v1 and portable batch checkpoint
v2 are separate contracts: the latter adds TaskSpec, lane seeds, partial reset,
and lane state. Scenario replay v2/v3 are direct predecessors of v4, but their
serializers did not record all evidence now required for exact replay.

Treating those cases uniformly would be unsafe. Relabelling the generic
checkpoint as portable v2 would invent state. Relabelling scenario replay v2 or
v3 as v4 would invent input provenance or canonical actor, action, ownership,
and result-digest evidence. Rejecting every old artifact without proving the
still-supported generic checkpoint would discard valid compatibility.

## Decision

Add schema-v1 `rne_historical_compatibility_decision` wrappers to the installed
corpus. Each wrapper freezes the artifact family, old/current schemas, source
commit and Git tree, workspace version, canonical source digest, expected
outcome, reason code, and either an exact replay digest or exact typed error.

The first matrix contains three real historical artifacts:

- `bd4d44f5bd781fc41fd8305938001f0a858993a5` emits generic vectorized
  checkpoint v1 after reset and one two-lane action step. The current runtime
  must restore it exactly and reproduce replay digest
  `17972057113911492359`.
- `533729ddc78e53284eaa11d823afae18dcd110ab` emits a 300-step scenario replay
  v2. It must be rejected with `expected 4, got 2`; the source has no scenario
  digest, network digest, engine version, or v4 result evidence.
- `e959e3ffe8426de3a8320d2d4c95e4e1438a50ad` emits the same 300-step run as
  scenario replay v3 with real input digests and engine version. It must be
  rejected with `expected 4, got 3` because actor/action/ownership/result-digest
  evidence is still absent.

The scenario artifacts remain immutable evidence. Changing only their schema
to v4 must fail. The supported path is to retain and, when needed, verify the
old bytes with the recorded engine, then rerun the original manifest with the
current engine. No compatibility layer may fabricate fields absent from the
source run.

Source `release-check` verifies the three commits remain ancestors of `HEAD`,
their exact trees and workspace versions match, their serializer sources
declare the registered schemas, and the scenario inputs exist. Extracted
bundles use only the embedded content-addressed artifacts and compiled
provenance identities. The generic checkpoint schema constant becomes public,
and its JSON reader rejects unknown fields.

## Consequences

- The installed corpus grows from seventeen to twenty fixtures.
- Compatibility reports can prove an accepted wrapper whose embedded artifact
  is intentionally rejected; the expected decision, not unconditional source
  acceptance, is the contract.
- Portable batch v2 remains a separate additive contract rather than a false
  migration target for generic checkpoint v1.
- Scenario replay history now has executable, provenance-bound rejection
  evidence instead of documentation alone.
- Dataset, protocol, TaskSpec, and Failure Capsule histories still require
  equivalent migration or required-rerun matrices before 1.0.
