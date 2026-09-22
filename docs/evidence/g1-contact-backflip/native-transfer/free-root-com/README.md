# Native free-root COM correction and landing probes

The welded-pair regression exposed spurious COM motion during free rotation.
The corrected root coordinates keep the body origin and physical COM distinct;
see [ADR 031](../../../../adr/031-multibody-free-root-com.md).

An independent source-URDF forward-kinematics audit reconstructs robot COM
positions from recorded native root poses and joint angles. It never advances
MuJoCo time. For the same `open-5` candidate, the maximum airborne ballistic
position residual falls from **0.152132 m to 0.0001713 m**. The audit uses each
run's first retained airborne COM velocity and gravity; sample intervals are
recorded, and residuals include integration/rounding differences. This checks
kinematics/COM integration, not contact equivalence or backflip qualification.
The source model mass is 34.133858 kg versus native 34.13385728 kg.

All seven 125 µs direct-torque trials below retain armature 0.01 kg·m². The
original source candidate completes one rotation and contacts the ground, then
rebounds and falls. Earlier opening reduces overspeed but does not maintain
standing; slower recovery also fails. Native full-body/self-contact remains
disabled. **No native-success GIF or hardware qualification is claimed.**

| Candidate | Opening (rad) | Recovery (s) | Peak speed/rating | Stop time (s) |
| --- | ---: | ---: | ---: | ---: |
| source | 6.035704 | 0.5 | 1.265716 | 1.991125 |
| open-5 | 5.000000 | 0.5 | 1.032819 | 1.390750 |
| search-best | 5.125000 | 0.5 | 1.033869 | 2.594125 |
| open-5p5 | 5.500000 | 0.5 | 1.032476 | 2.819250 |
| open-5p75 | 5.750000 | 0.5 | 1.032528 | 1.867875 |
| open-5p875 | 5.875000 | 0.5 | 1.085083 | 1.925375 |
| recovery-1p2 | 6.035704 | 1.2 | 1.265716 | 2.161125 |

Every result contains its explicit collapse failure. `sources.json` pins the
binary, source commit, all artifacts and targeted validation. Historical
pre-correction recordings remain in separate evidence directories.

To reproduce the offline audit with the existing optional research environment
(MuJoCo 3.13.0, NumPy 2.4.2, SciPy 1.17.0), run from the repository root:

```sh
python -I docs/evidence/g1-contact-backflip/native-transfer/free-root-com/flight_audit.py \
  . docs/evidence/g1-contact-backflip/native-transfer/free-root-com/open-5.json
```

The command prints JSON. `flight-audit.json` also contains the pre-fix and
other initial post-fix comparisons; all referenced rollout hashes are checked
against their archived bytes. Unit tests exercise the native COM invariant
without this optional environment.
