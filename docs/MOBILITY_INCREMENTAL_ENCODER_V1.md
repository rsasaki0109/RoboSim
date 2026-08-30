# Mobility incremental encoder v1

RNE's compatibility-stable `WheelEncoderSample` reports a completed wheel
coordinate. It remains available for existing scenes. The additive
`IncrementalEncoderSensor` models the digital frontend that a controller sees:
integer edges, a finite signed counter, calibration, capture timing, and a
velocity estimate reconstructed from count and time differences.

## Contract

The sensor reads completed revolute position only. It never reads completed
joint velocity or actuator targets. Its externally visible processing order is:

1. scheduled capture and phase-error measurement;
2. position-to-edge quantization using decoded counts per revolution (CPR),
   direction, and mechanical zero offset;
3. signed finite-counter wrapping or saturation;
4. modular count-difference reconstruction and an N-sample rolling velocity
   window using actual capture timestamps;
5. optional once-per-revolution index-phase crossing;
6. stuck-value substitution, frame dropout, then DataBus availability latency.

`IncrementalEncoderFeedback` contains the raw finite counter, modular count
delta, counter-derived position and velocity, wrap/index flags, the scheduled
capture tick, phase error, and an explicit initialization/nominal/saturated/
stuck status. A dropped capture advances the frontend history but emits no
frame, producing a visible sequence gap like real acquisition software.

The first capture reports zero velocity with `initializing`. At low speed the
frequency estimate remains exactly zero until an integer edge is observed.
Increasing `velocity_window_samples` reduces count quantization noise at the
cost of bandwidth and delay. Wrap reconstruction assumes fewer than half of
the configured counter range is traversed between adjacent captures; exceeding
that bound is physically ambiguous from the two counter values alone.

## Research and OSS correspondence

- Anuchin, Dianov, and Briz, *Synchronous Constant Elapsed Time Speed
  Estimation using Incremental Encoders*, describes quadrature edge counting,
  finite sampling-time frequency estimation, its low-speed resolution limit,
  and period/CET alternatives. It also establishes that quadrature decoding
  counts all A/B transitions, yielding four counts per pulse period:
  https://digibuo.uniovi.es/dspace/bitstream/handle/10651/53622/53622.pdf?sequence=1
- SimpleFOC's official encoder documentation distinguishes PPR from CPR,
  documents `CPR = 4 * PPR` in quadrature mode, supports an index input, and
  rate-limits velocity recomputation with a configurable minimum elapsed time:
  https://docs.simplefoc.com/encoder
- ODrive's public firmware interface is retained as an OSS integration
  reference for exposing encoder estimates and calibration through a motor
  controller boundary rather than substituting commanded motion:
  https://github.com/odriverobotics/ODrive/blob/master/Firmware/odrive-interface.yaml

RNE v1 deliberately implements the fixed-capture frequency method because it
is deterministic, identifiable, and matches synchronous control-loop sampling.
It does **not** claim CET/M-T fidelity. A future additive estimator may retain
individual edge timestamps and implement period or synchronous-CET modes; such
modes must remain explicit because their delay and low-speed behavior differ.

## Known omissions

- no A/B analog waveform, comparator duty-cycle, phase-placement, bounce, or
  electrical line model;
- no stochastic missed/extra edge process yet;
- no per-edge timestamp payload or period/CET/M-T velocity mode;
- no absolute encoder protocol, gearbox-side mounting, or compliance model;
- index phase is quantized to the nearest decoded edge.

These omissions prevent claiming hardware-fidelity from simulation alone. Real
logs must identify CPR, direction, zero/index calibration, sampling phase,
latency, missed-edge rate, and the velocity estimator/window before a benchmark
may label an encoder profile as matched hardware.
