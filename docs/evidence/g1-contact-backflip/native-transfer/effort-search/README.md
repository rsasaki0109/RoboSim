# Fine-step direct-torque native search (failed landings)

The deterministic two-round search varied launch hip, tucked knee and opening
angle at 125 µs with reflected motor inertia 0.01 kg·m². These 13 evaluations
use the binary **before** the f64 quaternion conversion fix; do not mix them
with subsequent native recordings.

Baseline loss is 290.980980. Candidate 12 reaches 280.848683 (3.48% lower),
with tuck knee 2.016356 rad and opening 5.125 rad. Its peak measured joint
speed/rating is 1.030165. Every trial collapses and stops at 1.200125 s.
The score is diagnostic, not a qualification gate. No native-success GIF
is produced.

`search.json` pins each result hash and binary hash. `sources.json` pins
native/search source commits and all JSON artifacts. The baseline rollout
is byte-identical to the earlier `armature/open-5.json` recording.

```sh
python3 scripts/g1_native_search.py \
  --binary /path/to/release/examples/114_g1_backflip_gif \
  --scene /path/to/native-passive-loss/scene.rne.scene.toml \
  --candidate docs/evidence/g1-contact-backflip/native-transfer/armature/open-5-candidate.json \
  --output /fresh/output --rounds 2 --workers 3 \
  --dt-us 125 --motor-mode effort --axes launch-tuck-open
```
