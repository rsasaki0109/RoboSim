# Compatibility fixture corpus

RNE retains a small executable corpus of public release artifacts. Its purpose
is to catch accidental reader breakage before release and to let users rerun
the same check from an extracted native bundle.

Run it from a source checkout:

```bash
cargo run --locked -p rne_compatibility_suite -- \
  --output artifacts/compatibility/report.json
```

Run the installed form from the bundle root:

```bash
./bin/rne-compatibility --root . \
  --output compatibility-fixture-report.json
```

When this report is used as 1.0 readiness evidence, the source-side promotion
gate does not trust its aggregate verdict. It verifies the registered ancestor
history, reruns the same corpus with current typed readers, and requires the
retained report to match the newly computed report exactly.

The strict registry is `release/compatibility-fixtures.toml`. Schema v1
contains twenty-five fixtures:

| Contract | Retained artifact |
|---|---|
| Task contract | TaskSpec v1 |
| Batch execution | portable batch checkpoint v2 |
| Controller ABI | ABI-v3 64-bit C layout, capabilities, and required symbols |
| Replay | generic replay v1, behavior replay v1, and scenario replay v4 |
| Historical decisions | Frontend transport v1, TaskSpec v1, Failure Capsule v1, dataset bundle v1, and vectorized checkpoint v1 retained exactly; scenario replay v2 and v3 rejected with a required-rerun decision |
| Dataset | bundle manifest v1, depth evaluation v1, and native payload v1 |
| Renderer dataset evidence | renderer capture report v1 |
| Frontend | current and first committed protocol-v1 `ClientHello` frame bytes and negotiated limits |
| Historical migration | retained zero-step snapshot v1 plus provenance-bound, sensor-bearing snapshots v1 and v2 restored as v3 |
| Failure evidence | Failure Capsule v1 |
| Hardware safety | process-mock conformance v1 |
| Physics | built-in conformance v2 and external-backend conformance v1 |

For every entry the runner:

1. resolves a canonical relative path without following it outside the corpus;
2. bounds the file at 4 MiB and requires a regular file;
3. verifies the SHA-256 of canonical compact JSON;
4. deserializes and semantically validates it with the current typed reader;
5. changes the version to a deterministic future value and requires rejection;
6. adds an unknown top-level field and requires rejection.

A historical-decision wrapper is itself the accepted typed fixture. Its
embedded artifact must then produce the recorded outcome: exact restore for a
still-supported schema, or typed rejection when safe migration would require
evidence the old serializer never recorded. A rejected source therefore remains
a passing corpus check only when rejection is the frozen, audited decision.

The binary frontend and dataset fixtures store lowercase hexadecimal wire bytes
beside their semantic values. Their readers must decode and re-encode the exact
bytes. The runner also requires frontend magic, message-kind, truncation, and
trailing-byte failures, an explicit incompatible-version negotiation rejection,
and truncation/trailing-byte failures for all five retained dataset payload
families.

The report has no timestamps, host paths, random IDs, or timing data. The same
registry and fixture content therefore produce identical report bytes on Linux
and Windows. Canonical JSON hashing deliberately ignores indentation and line
endings while preserving every parsed field and value.

## Historical source provenance

The two provenance-bound migration cases were serialized after seven fixed
simulation ticks by the source code that actually declared each schema:

| Source schema | Commit | Git tree | Required retained state |
|---|---|---|---|
| v1 | `47525b127a77cbffa9da27b1e0c127ee673aa641` | `bb408cec26d34bd2a9b423dbf8b2a4d44cdf7013` | joint-state and RGB frames; no depth or grasp-retarget field |
| v2 | `2255cbefec9d1eb5040603fbb119a290ad855191` | `373e5453c7ba94ee4efbeceb9985db4c97f5feff` | joint-state, RGB, and populated depth frames; no grasp-retarget field |

Each fixture stores the complete old snapshot, its canonical digest, source
workspace version, scene, step count, and the complete tolerance-normalized v3
digest. The installed runner restores both sources and checks every state value,
not only the newly added fields. Source `release-check` additionally requires
both commits to remain ancestors of `HEAD`, match the recorded trees, declare
the expected schema, and contain the source scene. The release-contract CI job
therefore uses a full-history checkout. Extracted bundles need no Git history:
their compiled verifier contains the frozen provenance identities and verifies
the content-addressed snapshots directly.

The decision matrix additionally binds three ancestor serializers:

| Artifact | Commit | Git tree | Current decision |
|---|---|---|---|
| Vectorized episode checkpoint v1 | `bd4d44f5bd781fc41fd8305938001f0a858993a5` | `23482add2c5d1de2978897d894d1ba745787bd06` | Restore exactly and reproduce replay digest `17972057113911492359` |
| Scenario replay v2 | `533729ddc78e53284eaa11d823afae18dcd110ab` | `b016841b2aed16bafc131f6a4698ee3b30cec34d` | Reject with `expected 4, got 2`; rerun because input provenance and v4 result evidence are absent |
| Scenario replay v3 | `e959e3ffe8426de3a8320d2d4c95e4e1438a50ad` | `17c6045624ccf2ed1271d19ea50926cb568ab337` | Reject with `expected 4, got 3`; rerun because v4 actor/action/ownership/result-digest evidence is absent |

The two scenario sources are real 300-step executions over the retained speed
scenario and corridor network. They share the historical stable result hash;
v3 additionally retains real input digests and its engine version. Neither is
byte-relabelled as v4 because that would fabricate evidence added after the
run. Source `release-check` verifies all three revisions, trees, workspace
versions, schema declarations, and ancestry. The installed runner verifies the
embedded source digest, exact decision, unsafe-relabel rejection, future-schema
rejection, and wrapper unknown-field rejection without Git.

The same matrix binds three introducing v1 serializers that have no later
schema transition yet:

| Artifact | Commit | Git tree | Installed proof |
|---|---|---|---|
| TaskSpec v1 | `70a9ff35afbf0215803dd288103bdda79fa46891` | `94459bcb0c5090921bf6edbcf6f63246ebdd6a40` | Exact typed validation and semantic JSON round trip |
| Dataset bundle v1 | `aecafb62c99f432b2a76956575f4562c6047a6bc` | `0bc9d2d48185282da31dc80eb8857d84012a5928` | Original manifest plus 736-byte shard; two streams, six records, four samples, two drops, and exact offline-evaluation digest |
| Failure Capsule v1 | `61d6c813e79d7eac6a8ab212776d620069f98905` | `5dac12166fe39da5a1207426f3e7520851e415d2` | Exact typed validation and semantic JSON round trip |
| Frontend transport v1 | `be53f16347beb7df822850748d0e01ce41d227a0` | `78a68abd73fb4564793559d8e75e021ad5090129` | Original `ClientHello` frame blob; exact frame/payload round trip and negotiation with fail-closed mutations |

Their original golden blobs are byte-identical to the retained current
goldens. Source `release-check` verifies those blobs at the recorded revisions,
the serializer declarations, ancestry, and the dataset generation recipe. The
installed verifier additionally reconstructs the dataset directory from the
embedded binary shard and rejects one-bit corruption before accepting the
historical decision.

Frontend protocol v1 was introduced earlier by commit
`1a38391362ece24cc73c0e1470a51bd7f933e6fc`, tree
`3117bd4949f19c36a1fba66524b97bd4bd1af3d4`. Source checks bind that
introduction's protocol declarations and header vector as well as the first
full golden above. The golden's Git blob is unchanged at `HEAD`, so later
payload-limit, negotiation, and queue hardening is proven wire-compatible.

## Change policy

Append a fixture when a candidate-stable artifact becomes release-facing or
when an older supported artifact needs explicit retention. Update its typed
dispatch, tests, golden report, contract registry, compatibility documentation,
and release bundle together. Do not silently replace an older fixture with a
newer shape; keep both while the older version is supported.

The current corpus is an expanded v0.9 slice, not a complete 1.0 declaration.
The renderer capture report uses the same strict public `rne_data` type in the
producer, independent dataset verifier, and installed compatibility runner; its
fixture freezes schema and aggregate RGB-D invariants without imposing a
cross-adapter pixel golden.
It retains one negotiated frontend reference frame rather than every message
kind and freezes the C layout rather than every platform's compiled library
image. Python call shape is verified by the separate installed wheel manifest.
The fixed 31-crate Rust API baseline establishes source-level history. The
snapshot v1/v2-to-v3 matrix proves multi-generation state migration, while the
checkpoint/replay decision matrix proves both exact retention and an explicit
non-migratable boundary. Dataset bundle, TaskSpec, Failure Capsule, and frontend
transport now have exact ancestor retention, but none has a second public
schema to exercise a transition. Broader frontend message-family coverage and
any future v1-to-v2
migrations or required-rerun decisions remain future work.
Independent-use and six-month stability gates remain mandatory.
