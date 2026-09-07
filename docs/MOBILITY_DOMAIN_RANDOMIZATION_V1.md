# Mobility domain randomization reference batch v1

This subgate adds a deterministic CPU reference batch for the Mobility Physical AI
foundation. It is a replay and integration contract, not a GPU-throughput claim and not
evidence of real-vehicle fidelity.

## Research basis

- [IsaacGymEnvs domain randomization](https://github.com/isaac-sim/IsaacGymEnvs/blob/main/docs/domain_randomization.md)
  applies parameter randomization at reset boundaries. RNE likewise derives one frozen
  physical profile per lane and episode rather than changing physics silently within a
  rollout.
- [Isaac Lab actuator models](https://isaac-sim.github.io/IsaacLab/develop/source/concepts/actuators.html)
  distinguish simulator-owned implicit drives from explicitly modeled actuator dynamics.
  RNE randomizes its explicit motor/transmission plant without transferring controller
  gains into a backend.
- [MuJoCo MJX](https://mujoco.readthedocs.io/en/3.1.0/mjx.html) represents model and data
  fields with batch dimensions. RNE v1 instead defines backend-neutral lane identity,
  sampling, ordering, and replay first; an accelerator may implement the batch later
  without changing these semantics.
- [Brax](https://github.com/google/brax/) demonstrates accelerator-oriented parallel
  rigid-body simulation. RNE does not infer equivalent throughput from this ordered CPU
  reference implementation.

These are interface and architecture references, not parity claims.

## Frozen distribution

`MobilityRandomizationSpec::training_v1()` contains bounded SI-compatible ranges for:

- vehicle mass;
- DC motor resistance and a coupled torque/back-EMF constant;
- rotor inertia, transmission ratio, and drive/backdrive efficiency;
- wheel radius, wheel inertia, and rolling resistance;
- longitudinal/lateral tire stiffness and peak friction;
- road friction and grade; and
- suspension stiffness and damping.

Parameters that represent the same physical quantity remain coupled. In particular,
torque and back-EMF constants receive the same scale, normal and tire reference loads
follow vehicle mass, and both transmission directions share an efficiency scale. Every
range must be finite and ordered. Conservative all-lower and all-upper profiles must also
pass the existing plant and suspension validity checks before any lane runs.

## Seed, ordering, and replay contract

For lane `L` and episode `E`, RNE derives an episode seed from `(root_seed, L, E)` and uses
keyed, channel-specific sampling. Therefore:

- repeated runs with the same inputs are byte stable;
- lane `L` does not change when the batch width changes;
- output is serialized in ascending lane order; and
- report validation re-samples and re-runs every lane, then verifies lane and batch
  digests.

The v1 reference rollout applies 18 V for 2,000 steps at 1 ms to the analytic
longitudinal motor/transmission/wheel/tire plant. The report retains the exact sample,
the fully applied physical structs, final state, maximum friction utilization, and a
digest for every lane. Batch width is bounded to `1..=4096`.

Run it with build and evidence data on an external SSD:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = 'E:\RNE-build\tmp'
cargo run -p rne_mobility_benchmark --bin rne-mobility-benchmark -- `
  --backend mobility-randomized-batch `
  --seed 20260903 --episode-index 0 --num-envs 32 `
  --output E:\RNE-build\m3c-sensor\mobility-randomized-batch-v1.json
```

## Reproduced evidence

The 32-lane external-SSD run passed all lanes and produced:

| field | value |
| --- | ---: |
| artifact bytes | 127,186 |
| report digest | `fnv1a64:13803833f6d5aab1` |
| SHA-256 | `7B28F3EDA9475C67BC6C7A85EF3EF90900451D64D5BE1DAA42413A1A24267AA3` |
| final position range | 1.608309–2.213188 m |
| final speed range | 0.873170–1.203574 m/s |
| final current range | 0.670941–5.961011 A |
| maximum friction-utilization range | 0.787016–1.000000 |

Two independent executions must be compared byte-for-byte before accepting this
evidence record.

## Applied Rapier/MuJoCo profile gate

The follow-on gate binds the same 15 ordered ranges into the portable `TaskSpec`, derives
one width-independent episode seed, and applies the resulting physical profile to the
shared Rapier/MuJoCo contact-to-tire-to-wrench fixture. This is an actual backend run: the
sample changes rigid-body mass plus the backend-neutral motor, transmission, wheel, tire,
and road structs before either solver advances. Both traces retain the exact same sampled
profile and task contract. Each backend result is self-verifying, and the comparison uses
the existing SI-unit tolerances instead of requiring bit equality between solvers.

Road grade rotates gravity into the road-aligned fixture frame, affecting both downhill
acceleration and support load. A regression test independently varies only grade and
checks that uphill speed is below level-road speed and downhill speed is above it.
Suspension stiffness and damping are retained parameters, not active dynamics here.

The backend fixture has one driven support path, so its nominal normal load and tire
reference load differ from the two-driven-wheel analytic reference batch. Sampling is
therefore split into deterministic scale generation and application to an explicit base
plant. This prevents a profile generated for one fixture from silently importing the
other fixture's load assumptions. Suspension scales remain retained but are not applied
to the rigid-support backend fixture yet.

Run the cross-backend evidence entirely on an external SSD:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = 'E:\RNE-build\tmp'
$env:MUJOCO_DYNAMIC_LINK_DIR = 'E:\RoboSim-mujoco\lib'
$env:PATH = 'E:\RoboSim-mujoco\bin;' + $env:PATH
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend mobility-randomized-backend-compare `
  --seed 20260903 --lane-id 0 --episode-index 0 `
  --output E:\RNE-build\m3c-sensor\mobility-randomized-backend-compare-v1.json
```

The reproduced lane-0 evidence used episode seed `15606677303828933863`, sampled a
115.531173 kg carrier, road-friction scale 0.617472, and road grade 0.050031 rad. Two
independent 81,725-byte outputs were byte-identical after the gravity/grade correction:

| field | value |
| --- | ---: |
| comparison digest | `fnv1a64:7c797c03999f633b` |
| SHA-256 | `A2E3DE8FAA2B8EA62710689CF53095510DAC72C3BA86113F326DE9EA24631F90` |
| forward-position gap | 0.002715 m / 0.05 m |
| forward-velocity gap | 0.001416 m/s / 0.05 m/s |
| maximum tire-utilization gap | 0.033945 / 0.1 |
| maximum tilt gap | 0.000025 rad / 0.005 rad |

The artifact also contains contact participation, lateral drift, motor-current, and
vertical-displacement gaps; all declared tolerances passed.

## Deliberate limits and next gate

The suspension parameters are sampled and retained but are not excited by either the v1
analytic longitudinal rollout or the rigid-support backend fixture. Physical profiles now
execute under the shared Rapier/MuJoCo TaskSpec, and that task separates actor,
privileged-truth, and diagnostic tensors. A separate
[sensor reset experiment](MOBILITY_SENSOR_OBSERVED_CLOSED_LOOP_V1.md#seeded-sensor-reset-experiment)
now applies seeded gyro/current offsets, common transport latency/jitter, and an encoder
dropout through the sensor-only controller. Its joint-reset API now combines those sensor
resets with a randomized longitudinal chassis while retaining nominal estimator calibration.
Neither runner executes lanes concurrently or reports hardware throughput.
A separate [CPU-parallel sensor episode runner](MOBILITY_SENSOR_EPISODE_BATCH_V1.md)
now executes independent joint physical/sensor longitudinal episodes concurrently;
it is not a lockstep vectorized policy interface or a throughput claim.

The next gate extends joint resets to the per-wheel skid and Ackermann fixtures,
independent transport jitter, LiDAR, and camera faults, then proves single-lane versus ordered-batch
replay equivalence. Only after that should an MJX, GPU, or other accelerator adapter
advertise vectorized throughput.
