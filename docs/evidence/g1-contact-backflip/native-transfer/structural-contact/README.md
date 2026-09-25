# Topology-derived structural filtering and collision-free tuck probes

`--native-structural-filter` groups fixed-connected links into rigid clusters
and excludes within-cluster and directly joint-connected cluster pairs. This
single-G1 probe uses 24 rigid clusters and records 38 excluded collider-link
pairs. The rule depends only on the authored joint graph, not contact outcomes
or candidate parameters. Ground and nonadjacent self-contact remain enabled.
Tests check these retained pairs, fixed-cluster adjacency, ordering independence
and the bounded 31-cluster mask capacity.

The previous logo/torso and joint-neighbor impulses disappear. Standing now
has minimum final-second upright cosine 0.999741, but still fails: maximum root
speed is 0.284782 m/s and 287 of 2000 final-second steps lack positive foot
impulse (longest gap 47.5 ms). Stable-looking posture is not standing qualification.

The baseline backflip contacts knee/torso in flight at about 0.773 s, reaches
1.153899x rated speed and falls at 2.5165 s. Knee-ground contact is also recorded
on the eventual fall. Reducing tuck hip target (parameter 6) eliminates all
positive nonadjacent self-contact impulses in three follow-ups, but each falls:

| Tuck hip (rad) | Stop (s) | Peak speed/rating |
| ---: | ---: | ---: |
| 2.10 | 2.0010 | 1.343726 |
| 2.20 | 1.9665 | 1.208562 |
| 2.25 | 2.2830 | 1.189493 |

These are coarse-step diagnostics, not converged or qualified backflips.
Next optimize launch and opening around a self-contact-free tuck, while retaining
all measured limit/contact gates. Full raw recordings are gzip-compressed;
`search.json` records the bounded hip grid and decompressed rollout hashes.
The baseline candidate is `../held-recovery/candidate.json`; use the generated
full-contact scene from `../convex-full-contact/README.md`, example 114 at
500 µs, `--native-stop-on-fall` and `--native-structural-filter`.

The current source sole profile still uses four 2 mm ground/support spheres per
foot; no source/native solver equivalence is claimed. Tests: example 15 tests
and targeted release Clippy pass; full workspace CI was not rerun to preserve
the 30 GiB reserve. Native standing/backflip qualification remains false.
