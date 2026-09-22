# Contact-phase gain and landing-pose probes (failed)

The corrected native model still rebounds or tips forward after landing.
These probes vary post-touchdown stiffness/damping independently of the flight
servo, then vary landing hip bias. Opening is 5.8 rad throughout. All use
armature 0.01 kg·m² and the passive-loss scene; every trial collapses.

The baseline at 500 µs stops at 1.8595 s versus 1.87325 s at 125 µs, reproducing
the same failure mode. Coarse-step candidates remain screening trials and
require fine-step revalidation. Neither search scores nor these recordings
qualify a native backflip. Full-body/self-collision remains disabled.

| Trial | Step (µs) | kp (Nm/rad) | kd (Nm·s/rad) | Hip bias (rad) | Peak speed/rating | Stop (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| gain-300-30 | 125 | 300 | 30 | 0.092024 | 1.032550 | 1.865000 |
| gain-500-40 | 125 | 500 | 40 | 0.092024 | 1.032550 | 1.873250 |
| gain-1000-40 | 125 | 1000 | 40 | 0.092024 | 1.032550 | 1.881500 |
| hip--0p1 | 500 | 500 | 40 | -0.100000 | 1.032730 | 1.830000 |
| hip--0p25 | 500 | 500 | 40 | -0.250000 | 1.032521 | 1.794000 |
| hip-0p09202427761939469 | 500 | 500 | 40 | 0.092024 | 1.032988 | 1.859500 |
| hip-0p3 | 500 | 500 | 40 | 0.300000 | 1.032988 | 1.829500 |
| hip-0p5 | 500 | 500 | 40 | 0.500000 | 1.032988 | 1.810000 |
| hip-0p7 | 500 | 500 | 40 | 0.700000 | 1.032988 | 1.791500 |

Candidate files record all parameters. `sources.json` pins the binary, source and every artifact. No native-success GIF is produced.
