# MuJoCo backend

`rne_physics_mujoco` is an optional, `publish = false` backend. The
default workspace does not enable its `mujoco` feature and therefore does not
require a MuJoCo runtime or native library.

The current `rigid_body` implementation compiles a deterministic MJCF model from
ECS before step 0. It supports multiple dynamic and fixed bodies with sphere,
cuboid, capsule, or fixed-plane colliders. Dynamic bodies are represented by
backend-private free joints; fixed bodies are welded into the model. Position,
orientation, linear velocity, and angular velocity synchronize in both directions
at an explicit fixed timestep. Model topology is immutable after compilation.

`MuJoCoBackend::preflight_world` validates the topology before native model
creation. Kinematic motion, invalid units, malformed joint graphs, and other
unsupported inputs fail explicitly instead of being silently approximated.
Kinematic motion is represented by the backend-neutral `kinematic_body`
capability: MuJoCo leaves it unadvertised and returns a typed missing-capability
error from both preflight and ECS sync before a native model is created.
The original caller-owned one-sphere MJCF constructor remains as a compatibility
fixture, not as the primary backend path.

With the `mujoco` feature enabled, `rne_physics_conformance_suite` runs MuJoCo through
the same Harness v2 rigid-body vector used for Analytic and Rapier. Its named
position tolerance is `mujoco_free_fall_position_m_v1`. The Windows/Linux MuJoCo
workflow runs both the backend integration tests and this shared conformance gate.
It also generates and verifies the Rapier-vs-MuJoCo diagnostic Failure Capsule,
then uploads the portable capsule directory as a short-lived CI artifact.

The same adapter now executes the v0.7 mobile-lift shared-aisle flagship without
changing its TaskSpec, policy, behavior contracts, or scene assets. Run
`cargo run --locked -p xtask -- flagship --cross-backend` after configuring the
3.9 runtime. The command writes both backend success reports, both intentional
failure reports and replays, plus
`artifacts/flagship-validation/cross-backend-report.json`, requires the exact
inspection/traffic/grasp/place outcome on both engines, and evaluates nine named
SI-unit tolerances. Schema v2 also applies the same minimized perception
blackout to the identical controller on each backend and requires the expected
contract, first violation step, and simulation timestamp to match exactly.
Final state digests are retained for backend-local diagnosis but are
deliberately not compared across unlike solvers.

The publishable `rne_ai` crate accepts a typed backend factory and has no direct
dependency on this `publish = false` adapter. The non-published flagship example
owns the optional MuJoCo dependency and injects its constructor and preflight
hook at the application boundary. This keeps `cargo package -p rne_ai`
resolvable from crates.io while preserving the real native execution path.

Legacy `JointMotor` controllers are translated into a MuJoCo-native passive
damping term plus motor feed-forward, with bounded backend-private gains. This
preserves the PD law while allowing `implicitfast` to solve damping on lightweight
links without explicit-force instability. If a runtime controller changes the
bounded damping coefficient, the adapter deterministically recompiles only the
native model dynamics and restores current ECS state; body topology remains under
the existing post-step-0 immutability check.

The backend also advertises `articulation`. Revolute and prismatic descriptions
compile into nested hinge/slide joints, scalar state synchronizes through
`JointState`, and the unit-explicit `JointActuation` contract supports position,
velocity, and direct effort/force modes. Invalid values or a command whose units
do not match the joint kind return `PhysicsError::InvalidActuation` before the
step. Rapier implements the same command contract, and the shared revolute vector
uses it directly.

The backend also advertises `contact_force`. Native contact-point forces are
integrated over the fixed step, summed across every point for the same canonical
entity pair, and emitted with a stable A-to-B normal. Disabled-response sensor
geometries are checked explicitly and emit zero-normal, zero-impulse overlaps.
The shared resting-load vector uses the unit-bearing
`mujoco_resting_impulse_n_s_v1` tolerance and records a canonical snapshot hash.
Raycasts advertise `raycast_batch`: the adapter walks farther hits by excluding
each previously intersected body from native `mj_ray`, matching the Rapier
multi-hit distance ordering contract.

## Runtime and provenance

The crate uses `mujoco-rs` `5.0.0+mj-3.9.0` with default features disabled,
which is bound to MuJoCo 3.9.0.  Cargo cannot match semver build metadata in a
dependency requirement, so the manifest uses `=5.0.0` and `Cargo.lock` pins the
resolved package to `5.0.0+mj-3.9.0`; lockfile updates must preserve that exact
package.  The runtime version is checked before loading a model and must begin
with `3.9.`.  Do not use a 3.11 runtime with these bindings.

The official MuJoCo release provides prebuilt shared libraries for Windows and
Linux.  Feature builds require `MUJOCO_DYNAMIC_LINK_DIR` to point at the
runtime's `lib/` directory and the platform loader path to contain the shared
library directory.  The official release page is the source of release assets
and checksums:

- <https://github.com/google-deepmind/mujoco/releases/tag/3.9.0>
- <https://github.com/google-deepmind/mujoco>
- <https://mujoco.readthedocs.io/en/stable/programming/index.html>

Verified release assets (SHA256):

- Windows x86_64 zip: `544f44a8a7df3e94648a7eaf41500f4456eb59f9f01df3ec2cfb03bdbf5c2bb9`
- Linux x86_64 tarball: `d11f281540d0d1844e2923bf43b6fff5ad186ec55927a8dae0eb26b9e579eed2`

MuJoCo is Apache-2.0.  `mujoco-rs` is MIT OR Apache-2.0:

- <https://github.com/davidhozic/mujoco-rs>
- <https://docs.rs/mujoco-rs/5.0.0%2Bmj-3.9.0>

No runtime archive is vendored.  The dedicated
[MuJoCo workflow](../.github/workflows/mujoco.yml) downloads these exact
official assets, verifies their SHA256 values, sets the dynamic-link and
platform loader paths, and runs the feature-gated tests on Ubuntu and Windows.
The feature-gated tests must not be replaced with a fake runtime; when the
native library is unavailable locally, run the default workspace checks and
report the feature-build blocker.

## Determinism contract

MuJoCo repeatability is an exact-repeat check only for the same platform,
runtime binary, binding version, and input fixture.  Cross-platform results
are tolerance-level comparisons; this spike does not advertise
`deterministic_step` and does not claim cross-platform bit equivalence.
