# Full-contact launch/opening optimization

Two deterministic coordinate rounds (17 evaluations) vary crouch knee, push
hip bias, landing knee and opening angle with full convex body geometry and
topology-derived structural filtering. Tuck hip is fixed at 2.25 rad. The
objective includes measured speed/position limits and positive non-support
contact impulses. It remains a search score, not qualification.

Candidate 14 holds the maneuver through 5 s at 500 µs with peak speed/rating
1.049782, within the unchanged 1.05 threshold. Its maximum joint-position
excess is 1.069e-6 rad. All 2000 final-second steps carry positive foot impulse;
minimum upright cosine is 0.998162. No positive nonadjacent self-contact impulse
is recorded. However, final-second root speed reaches 0.161819 m/s while its
10 s recovery is still in progress, so it does not meet the 0.1 m/s standing
gate at 5 s. Full qualification is not established.

The best landing knee target is 1.657802647 rad. Search loss decreases from
about 742 to 3.988075. Candidate 16 also survives 5 s but exceeds the speed
threshold (1.115073x). Remaining candidates fall. Raw compressed recordings,
exact candidate vectors, fixed-order evaluations and hashes are retained.

These recordings predate signed-distance auditing. Zero impulse is not proof
of nonpenetration. The selected candidate is being revalidated through 15 s
at 500 and 125 µs with signed solver-manifold separations. Further 62.5 µs
validation is supported with exact integer-nanosecond timing, but has not yet
been completed. No success GIF or full-body qualification is claimed here.

Reproduce with `scripts.g1_native_search.search`, seed `candidate-0000.json`,
`rounds=2`, `workers=4`, `dt_us=500`, `motor_mode="effort"`,
`structural_filter=True`, and axes (index, lower, upper, step):

```python
[(0, 2.0, 2.7, .05), (3, .5, 2.0, .08),
 (8, 1.2, 2.0, .08), (11, 5.7, 6.1, .05)]
```

Use the full-contact scene from `../convex-full-contact/README.md`. Native
producer and search-driver revisions are separately pinned in `sources.json`.
The search driver rejects ignored structural-policy requests and missing
geometry/contact evidence. The new signed-distance API also checks zero-
impulse pairs and respects filters after they change. Targeted backend tests
(33 with armature), example timing/contact tests (17), Python tests (14) and
Clippy pass. Full workspace CI remains pending to retain the 30 GiB reserve.
