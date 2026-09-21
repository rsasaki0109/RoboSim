# Explicit native friction comparison

All four candidates fail landing at 0.5 ms; all stop on the collapse guard.
Changing sliding friction alone to the source model's 0.7 does not transfer
the backflip. This remains the 38.13 kg primitive-contact native model with
self-collision off, not the complete source plant.

| Candidate | Sliding coefficient | Peak speed / rating | Final signed pitch (rad) |
|---|---:|---:|---:|
| friction07-canonical | 0.7 | 2.31775 | -5.59714 |
| friction07-open5 | 0.7 | 1.09322 | -4.93275 |
| friction07-bias27 | 0.7 | 2.97391 | -5.32194 |
| friction05-open5 | 0.5 | 0.91256 | -4.92810 |

The 0.5 early-opening run reproduces every recorded physical frame and the
peak speed of the preceding default-friction run exactly. The bounded command
avoids overspeed in that candidate but does not produce a successful landing.
Other candidates exceed the measured-speed screening gate despite bounded
velocity commands. Source and recording hashes are in `sources.json`.

Reproduce using the parent README command, choosing a candidate JSON here
and a fresh output filename. No success GIF is produced for these failures.
