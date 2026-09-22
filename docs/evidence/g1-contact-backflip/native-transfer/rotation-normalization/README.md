# Quaternion conversion correction: native comparison

Promoted f32 rotations are normalized in f64 before hierarchical transforms.
The torque-excited passive-loss scene closes authored joint translations to
6.713e-7 m, has maximum quaternion squared-norm error 4.641e-14 and normalized
orientation dot deficit 8.438e-15. This is a kinematic consistency check.

The two 125 µs direct-torque flips below use armature 0.01 kg·m². Both still
collapse at 1.200125 s. The later free-root COM fix is **not** in this binary.

| Candidate | Final rotation (rad) | Peak speed/rating | Touchdown (s) |
| --- | ---: | ---: | ---: |
| baseline | -5.340833 | 1.029489 | 1.158000 |
| search-best | -5.669286 | 1.030117 | 1.128000 |

`kinematic-audit.json` and `sources.json` retain metrics and source/artifact hashes. No native backflip qualification or success GIF is claimed.
