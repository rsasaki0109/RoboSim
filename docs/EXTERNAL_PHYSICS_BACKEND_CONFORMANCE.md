# External physics backend conformance

`rne_physics_conformance` is the publishable, backend-neutral authoring and
conformance kit for independently maintained RNE physics backends. It depends
on the public `PhysicsBackend` boundary, never on Rapier, MuJoCo, a renderer,
ROS 2, or a vendor SDK.

The built-in aggregate remains a separate workspace suite named
`rne_physics_conformance_suite`. That suite compares RNE's Analytic, Rapier,
and feature-gated MuJoCo implementations. External authors use the public crate
and receive the distinct
`rne_external_physics_backend_conformance_report` schema v1.

## Add the kit

Use the exact RNE release shared by the backend:

```toml
[dev-dependencies]
rne_physics = "=0.2.0"
rne_physics_conformance = "=0.2.0"
```

Then bind the exact implementation artifact or deterministic source bundle,
declare a backend manifest, and provide a fresh backend factory:

```rust
use rne_physics::{
    PhysicsBackendManifest, PhysicsBackendRepeatability, PhysicsCapability,
};
use rne_physics_conformance::{
    run_external_backend_conformance, ExternalPhysicsBackendConformanceConfig,
    ExternalPhysicsBackendSubject,
};

let implementation_bytes = std::fs::read("my-backend-source.tar.zst")?;
let subject = ExternalPhysicsBackendSubject::from_bytes(
    "my-backend-source.tar.zst",
    &implementation_bytes,
)?;
let manifest = PhysicsBackendManifest::new(
    "my_backend",
    "0.1.0",
    "my_engine",
    "2.4.1",
    [
        PhysicsCapability::RigidBody,
        PhysicsCapability::DeterministicStep,
    ],
    PhysicsBackendRepeatability::SameRuntimeExact,
)?;
let report = run_external_backend_conformance::<MyBackend, _>(
    ExternalPhysicsBackendConformanceConfig::new(subject, manifest),
    MyBackend::new,
)?;
report.write_json("my-backend.conformance.json")?;
assert!(report.passed());
# Ok::<(), Box<dyn std::error::Error>>(())
```

The complete runnable reference is
`crates/rne_physics_conformance/examples/reference_external_backend.rs`:

```bash
cargo run -p rne_physics_conformance \
  --example reference_external_backend -- \
  --output artifacts/external-physics-conformance/report.json
```

## Catalog v1

The report always carries these nine checks in this order:

| Check | Contract |
|---|---|
| `manifest_identity` | Manifest schema, canonical capability order, and exact runtime declaration agree |
| `rigid_body.free_fall` | 60 Hz SI-unit gravity produces bounded position and velocity plus a canonical snapshot |
| `articulation.revolute_limit` | A velocity-driven revolute joint preserves its anchor and bounded limit |
| `gpu_rigid_body.catalog_unsupported` | Fails closed when GPU rigid bodies are advertised because v1 has no portable vector |
| `deterministic_step.repeat_snapshot` | Two fresh executions produce identical canonical snapshots |
| `soft_body.catalog_unsupported` | Fails closed when soft bodies are advertised because v1 has no portable vector |
| `contact_force.resting_impulse` | A settled load-bearing pair reports a bounded positive impulse |
| `raycast_batch.ordered_hits` | Query shape, hit distance/order, and repeated output are stable |
| `kinematic_body.external_pose` | An externally supplied pose remains authoritative across a fixed step |

Unadvertised capabilities are recorded as `not_advertised` and do not weaken
the verdict. An advertised capability must pass its fixed catalog case. Authors
cannot supply custom tolerances: catalog v1 owns the named SI-unit bounds.
Semantic failures produce a valid failed report; malformed subjects or report
shapes are errors. Reports contain no timestamp, duration, host path, or random
identifier, so two same-runtime fresh executions are byte-identical.

## Evidence and certification

The report hashes both the implementation subject and canonical backend
manifest. A Failure Capsule accepts this report only when the exact subject
bytes are included and their SHA-256 matches. This prevents a passing report
from being relabelled onto different backend code.

The committed golden shape is
`crates/rne_physics_conformance/tests/golden/external-backend-conformance-v1.json`.
The in-repository reference proves the public author workflow; it is not the
independent third-party certification required for RNE 1.0.

The eventual readiness pack must retain the exact subject bytes named by the
report—normally the independently built implementation artifact or deterministic
source bundle—plus the external repository URL and lowercase 40-character
tested commit. Manifest v3 rehashes the subject and requires its file label to
match; a copied passing report without those bytes fails closed.
