# Sensor sampling jitter

## Goal

Real feedback devices do not sample exactly on their nominal grid. Clock
jitter, trigger jitter, and internal scheduling move each capture instant by a
small, unpredictable amount. This slice adds that impairment to the typed
sensor frontends without changing their nominal schedule, their serialized
payloads, or any existing evidence chain.

## Scope

`sensor::components::SensorSamplingJitter` is an opt-in ECS component for the
typed feedback frontends:

- `ImuFeedbackSensor`
- `IncrementalEncoderSensor`
- `MotorElectricalFeedbackSensor`
- `JointFeedbackSensor`

Attach it to the sensor entity. It is independent of the compatibility-stable
`SensorKind` path and of the frontend `fault` enum.

## Model

Each attempt draws a bounded delay

```text
delay_ticks ∈ [0, maximum_delay_ticks]
```

from a `KeyedRandom` stream derived from the world seed, `SensorSamplingJitter::seed`,
the stream id, and the sample sequence. The capture instant becomes

```text
capture_ticks = scheduled_capture_ticks + delay_ticks
```

and the frontend samples at the first simulation tick at or after `capture_ticks`.

Properties:

- **Non-negative.** A triggered acquisition cannot complete before its nominal
  instant, so the delay is one-sided. A two-sided model would require sampling
  before the simulated instant, which a fixed-step replay cannot represent.
- **Bounded and monotonic.** The draw is saturated to `period - 1` ticks, so the
  nominal schedule stays strictly increasing. A period of one tick disables
  jitter.
- **Deterministic.** Replaying the same run with the same seeds reproduces the
  same jittered capture times; each sensor, stream and sequence has its own
  stream.
- **Value-faithful.** Because the sample is taken at the delayed tick, the
  measurement reflects the plant state at that instant, not at the nominal one.

The payload keeps its existing meaning: `scheduled_capture_ticks` is still the
nominal schedule and `sample_phase_error_ticks` is the non-negative offset from
it, now including the jitter delay. No `rne_data` schema changes are needed.

## Backward compatibility

An absent component, or `maximum_delay_ticks == 0`, draws a zero delay and
reproduces the nominal schedule bit-for-bit. Existing golden files, serialized
frames and content-digest-bound evidence stay valid unchanged.

## Evidence

`crates/rne_sensor/src/systems.rs` covers:

- an absent or disabled component drawing zero delay;
- bounded, reproducible, seed-sensitive, period-saturated draws;
- each of the four frontends advancing its first capture to
  `nominal + delay` and reporting `sample_phase_error_ticks == delay`.
