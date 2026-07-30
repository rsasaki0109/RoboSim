# Sensor latency in the loop

Every sensor frame in RNE carries two timestamps: `capture_time`, when the measurement
was made, and `available_time`, when transport and processing latency have elapsed and
a consumer could actually hold it. `Sensor::latency_ticks` has always populated the
second — but nothing forced a controller to respect it, and a loop that reads simulator
state directly is using data no real system could have.

## The reading contract

`DataBus::latest` returns the newest published frame regardless of availability. It is
the right call for logging and offline analysis, and the wrong call for control. The
new `DataBus::latest_available(stream, now)` returns the newest frame whose
`available_time` is at or before `now` — the read a real system performs. Until the
first frame arrives it returns `None`, which is a cold start, not an error.

```rust
if let Some(pose) = bus.latest_available::<PoseSample>(stream, now) {
    // pose is from the past; the vehicle has moved since.
}
```

`PoseSample { position_m, yaw_rad }` is a new payload for localization outputs, so
pose can travel through the bus like any other measurement instead of being read out
of the ECS.

## Example 51: the phase-margin threshold

A dynamic-bicycle vehicle follows the sweeper course from examples 49–50 under pure
pursuit. The controller never touches the simulator state: a localization source
publishes the true pose at 60 Hz with a configured transport latency, and the
controller steers from whatever `latest_available` returns — a pose of the past.

Three runs differ only in that latency, and the result has the shape feedback delay
actually has. It is a threshold phenomenon, not a linear tax:

| latency | RMS error | max error | settles |
| --- | --- | --- | --- |
| 0 ms | 0.468 m | 1.50 m | yes |
| 120 ms | 0.460 m | 1.50 m | yes |
| 240 ms | 2.05 m | 3.50 m | **never** |

At 12 m/s with a 5 m lookahead, 120 ms of delay sits inside the loop's phase margin
and costs nothing measurable. 240 ms exceeds it: the vehicle covers 2.9 m — most of
the lookahead — before its feedback arrives, and the loop oscillates across the course
for the rest of the run. The lookahead is chosen deliberately tight; a longer one is a
stronger phase lead and would hide the moderate delay entirely.

```bash
cargo run --release -p latency_in_the_loop --example 51_latency_in_the_loop
RNE_SKIP_GPU=1 cargo run -p latency_in_the_loop --example 51_latency_in_the_loop
```

The acceptance tests pin the threshold shape: zero latency tracks and settles,
moderate latency costs at most a small factor, heavy latency never settles and more
than doubles both RMS and maximum error, and every run is bit-identical on repeat.

## Why this matters

A controller evaluated against direct state reads carries an implicit zero-latency
assumption that hardware will not honor. Putting `available_time` in the read path
makes latency a first-class experimental variable: the same harness that swept
friction and actuator lag in example 50 can now sweep perception delay, and a policy
that only works with instantaneous feedback fails in simulation instead of in the
field.
