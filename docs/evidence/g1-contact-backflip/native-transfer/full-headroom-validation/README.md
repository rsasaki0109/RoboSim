# Full-contact command-headroom validation

Producer `0c84bfdec8bdbdd3a4d9724e5bf13469fc895243`, +0.02 rad landing-knee
candidate, 0.001 Nm direct-effort command headroom. Physical joint ratings and
all audit gates remain unchanged. Full body contacts and structural masks
are active; no root trajectory or external root wrench is applied.

At 500 µs the 15-second rollout passes every recorded gate in
`g1_native_audit.py`: peak speed 1.03187251x, standing error 0.00465579,
minimum final-second height 0.78060549 m, maximum base speed 0.00205620 m/s,
continuous positive foot support, no nonfoot ground contact and no negative
nonadjacent self-contact solver gaps. Peak realized knee effort is
119.99901581 Nm, below the unchanged 120 Nm ceiling. All 23 joints have
32,000 finite effort samples.

125 µs and 62.5 µs runs of the exact same candidate and immutable producer
are in progress. Passing a coarse recorded-metrics audit alone does not prove
timestep refinement or final qualification. A successful native GIF remains
pending finer validation and review of producer/model provenance.

## Finer-step failure

The 62.5 µs rollout collapsed at 2.98925 s after landing. This rejects the
+0.02 rad landing-knee candidate despite all coarse recorded gates passing.
The 125 µs run remains live for comparison. Three 62.5 µs five-second probes
now test smaller knee increments (+0.002, +0.005, +0.010 rad relative to
full-launch-open candidate 14), each with the same 0.001 Nm command headroom,
full-contact model, control gains, and producer. No limits or contacts are
relaxed and no final native GIF is claimed.
