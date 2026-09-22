# Corrected-plant launch/opening search (failed landings)

Two deterministic coordinate rounds at 500 µs vary crouch knee, crouch hip
bias, push duration and opening angle. All 17 evaluations use corrected root
COM integration, normalized rotations, direct torque, armature 0.01 kg·m² and
contact-phase gains 500 Nm/rad and 40 Nm·s/rad.

Baseline loss is 249.631746; candidate 9 reaches
218.066015. Collapse time increases from
1.859500 s to 2.743000 s,
but its peak measured speed/rating is 1.190973.
**All candidates fail standing recovery.** Longer survival and a better
objective do not constitute backflip qualification or actuator compliance.

The best candidate has crouch knee 2.450359 rad
and opening 6.050000 rad. This is a coarse-step
screening result; fine-step validation remains necessary for useful candidates.
No native-success GIF is produced.

`search.json` stores the four bounded axes, fixed-order results and per-rollout
hashes. `sources.json` records binary/source/artifact hashes. The baseline is
byte-identical to the corresponding baseline in `landing-search`.

Reproduction uses `scripts.g1_native_search.search` with `rounds=2`, `workers=4`,
`dt_us=500`, `motor_mode="effort"` and these axes (index, lower, upper, step):

```python
[(0, 2.0, 2.7, .15), (12, -.6, 0.0, .1),
 (2, .03, .12, .02), (11, 4.5, 6.1, .25)]
```

Use `candidate-0000.json` as the seed and a freshly generated passive-loss
scene. The helper requires a fresh output path and at least 30 GiB free.

## Follow-up crouch grid

Three additional 500 µs probes keep the best candidate's other parameters and
vary crouch knee only. All fail; none improves on candidate 9. They are separate
from the 17 evaluations in `search.json` and have not been promoted to a
fine-step success candidate.

| Crouch knee (rad) | Peak speed/rating | Collapse (s) |
| ---: | ---: | ---: |
| 2.30 | 1.122679 | 1.916000 |
| 2.35 | 1.065855 | 2.079000 |
| 2.40 | 1.075354 | 2.428500 |
