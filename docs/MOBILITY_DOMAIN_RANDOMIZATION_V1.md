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

## Deliberate limits and next gate

The suspension parameters are sampled and retained but are not excited by the v1
analytic longitudinal rollout. The batch also does not yet execute the shared Rapier and
MuJoCo rigid-road TaskSpec, randomize sensor calibration/timing/faults, expose separate
actor and privileged tensors, run lanes concurrently, or report hardware throughput.

The next gate applies each profile unchanged to the Rapier/MuJoCo mobility fixtures and
adds deterministic encoder, current, IMU, LiDAR, and camera randomization at reset. Only
after single-lane versus batch replay equivalence is proven should an MJX, GPU, or other
accelerator adapter advertise vectorized throughput.
