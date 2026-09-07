# CPU-parallel mobility sensor episodes v1

`rne_mobility_benchmark::observed_batch` runs independent randomized longitudinal
physical/sensor episodes using Rapier or MuJoCo on bounded scoped CPU threads.
Each lane owns a fresh backend, world, sensor bus and controller. This is **episode
parallelism**, not a lockstep vectorized policy API or a GPU throughput result.

## Contract

- Explicit root seed, episode index and stable lane ID derive the reset seed.
  Worker identity and completion order never participate in seed derivation.
- Joint plant and sensor resets use the existing sensor-observed contract. The
  actor uses measured/estimated signals; sampled plant parameters remain privileged.
- The underlying WorldRandom/noise seed remains zero to isolate reset-profile
  variation. This does not demonstrate independent stochastic noise realizations.
- Reports retain every requested episode, including task failures. `passed` means
  every task passed; successful artifact generation does not imply task success.
- Bounds: 1–64 lanes and 1–16 workers. Each worker processes its assigned lanes
  sequentially. Factory/solver errors and worker panics return errors, not partial
  successful reports. All workers are joined before errors propagate.
- Results are sorted by lane ID. Worker count is deliberately absent from the
  canonical artifact, permitting byte-identical serial/parallel evidence.
- Validation checks seeds, ordering, nested trace contracts/digests and failure
  accounting. It does not replay physics; use the voltage replay API for that.

## Run on the external SSD

Use the repository's external build/TEMP/MuJoCo environment configuration first.

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- --backend sensor-episode-batch-rapier --seed 42 --episode-index 0 --num-envs 3 --workers 2 --output E:\RNE-build\m3c-sensor\episode-parallel-rapier.json
cargo run -p rne_mobility_benchmark --features mujoco -- --backend sensor-episode-batch-mujoco --seed 42 --episode-index 0 --num-envs 3 --workers 2 --output E:\RNE-build\m3c-sensor\episode-parallel-mujoco.json
```

Repeat with `--workers 1` and a distinct output filename to compare file hashes.
Default worker count is one; both seed and lane count must be explicit.

## Verification and remaining scope

Unit tests compare complete serial/parallel reports for both backends, verify
Rapier lane identity across batch widths, reject reordered lanes/invalid bounds,
and exercise factory failures and multiple panicking workers.

Local CLI evidence (root seed 42, episode 0, three lanes, workers 1 versus 2)
was byte-identical within each backend; all three tasks passed in each run:

- Rapier file SHA-256: `3b0d6b957796d2efe77e13a9e14b1d32a62b250b6273fd034e9c7825c13e8c0e`.
- MuJoCo file SHA-256: `b5e3faf9ad87ae42ddbc4a0ca52957fb3431a920c5990fc011d11652242bbf90`.

Files are stored externally as `E:\RNE-build\m3c-sensor\episode-{backend}-workers-{1,2}.json`.
These results establish scheduling independence for the tested cases, not every
possible seed or a cross-backend equality guarantee.

Validation for this slice: all 70 Mobility library tests with `--features mujoco`
passed, along with feature-enabled Clippy. The full `cargo run -p xtask -- ci`
completed with exit code 0, covering workspace formatting/Clippy/tests, examples,
Python/RL smokes, headless integration, parity, fuzz smoke and 10/10 behavior seeds.
The full log is retained at `E:\RNE-build\m3c-sensor\episode-policy-ci.log`.

This slice covers the existing longitudinal fixture, not broad per-wheel/skid or
Ackermann reset coverage. Lockstep reset/step/terminal-mask interfaces, independent
noise-stream variation, policy batching, measured throughput and real-log
calibration remain separate development gates. Cross-backend acceptance tolerances
are not replaced by within-backend exact determinism.

## External policy boundary

`observed::run_mobility_sensor_policy` now accepts a caller-owned voltage policy
for one fresh joint-reset episode. Its `SensorPolicyObservation` contains only
sensor-derived odometry/uncertainty/health, available motor measurements, decision
simulation time and the task target. Neither the world nor privileged scoring
fields nor hidden reset parameters are passed to the callback.

The callback runs at accepted estimator decisions, so missing sensor updates can
extend the action-hold interval. Initial voltage is zero. A returned command feeds
the next drive evaluation, retaining existing one-step wrench staging. NaN,
infinity, voltages outside [-24, 24] V and callback errors abort before applying
that action; no silent clamping or successful partial trace is produced.

The completed trace is evaluator-only output and remains compatible with exact
voltage replay. Its PI gain fields specify the reference controller, not proof
that the supplied callback ran PI. Policy identity/version/weights are caller-owned
and not attested by this trace. A callback can of course capture caller-provided
data; this typed boundary is not a process security sandbox.

This is the control injection point for a future stateful reset/step wrapper,
not itself that wrapper: the call still runs a whole episode. A separate
[`SensorFixedEnvironment` primitive](MOBILITY_SYNCHRONOUS_ENV_DESIGN.md) now provides
persistent 10 ms stepping; its training/batch contract is still incomplete. The episode batch above
still uses the reference PI controller. Regression tests compare a supplied PI
callback with the reference complete trace, demonstrate changed physical state
under a zero-voltage policy, retain its failed verdict, verify voltage replay,
and reject invalid actions at the first callback.
