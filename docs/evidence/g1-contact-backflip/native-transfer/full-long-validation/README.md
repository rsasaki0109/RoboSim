# Full-contact long validation: finer steps pending

The selected candidate completes 15 s at 500 µs. Final-second upright cosine
is at least 0.999989349; maximum root speed is 0.000756922 m/s, with positive
foot impulse throughout that second. Peak joint speed/rating is 1.049782467
and position excess is 1.069e-6 rad. The rotation is about -6.279 rad.

Signed solver-manifold auditing records no negative nonadjacent self-contact
separation. Hip-roll/pelvis predictive pairs have minimum positive gaps above
1.40 mm; normal impulse is zero. No non-foot environment contact is recorded.
All 600 frames shared with the earlier 5 s recording are exactly identical,
confirming the read-only separation query and nanosecond scheduling retain the
500 µs motion. Raw records and producer hashes are preserved.

**Full qualification is still pending:** 125 µs and 62.5 µs, 15 s evaluations
are running separately. The native `qualified_backflip` field remains false.
No new success GIF is generated from this coarse-step result alone.

Two coarse-step landing-knee margin probes use the same controller except
parameter 8. Adding 0.02 rad holds through 5 s with peak speed/rating 1.031872,
but recovery is incomplete (tail speed 0.161963 m/s). Adding 0.04 rad falls at
2.81 s despite 1.031731x speed. Both have no negative nonadjacent self-contact
separation. They have not undergone long or fine-step validation.

Native producer is commit `81179a5`; the normal binary hash is recorded in
`step-500us-search.json`. The scene is the full convex model from
`../convex-full-contact/README.md`, with `--native-structural-filter` and direct
joint effort. The additional signed-distance test verifies filtering after
masks change; it does not step or modify the actual optimization recordings.

## Finer-step rejection

The 125 µs run completed 15 seconds without falling, with 0.9999995 minimum
upright and 0.0010942 m/s maximum base speed during the final second. All
8,000 final-second physics steps have positive foot support. Nonadjacent
self-contact manifold gaps remain positive, and no nonfoot ground pairs occur.
However, peak left-knee speed is **1.0502659798 times its rating**, above the
strict `< 1.05` gate. This candidate is therefore rejected. The 62.5 µs run
was deliberately cancelled after this failure; its partial log and reason are
retained and are not a completed validation. Next candidate: +0.02 rad landing
knee, which previously held at 500 µs with peak speed 1.031872x.

These historical records lack per-step measured-effort aggregates, minimum
tail height, and the source standing-error scalar. `g1_native_audit.py` rejects
missing telemetry rather than substituting assumed values. New producer
recordings must provide these fields before final qualification.
