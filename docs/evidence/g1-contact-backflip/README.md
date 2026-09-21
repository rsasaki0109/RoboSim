# G1 non-RL contact backflip evidence

See [the benchmark contract and reproduction commands](../../G1_CONTACT_BACKFLIP.md).

- `candidate.json`: the 13 optimized motion parameters, landing gains, and model settings.
- `step-125us/`: passing five-second simulation, sampled every 10 ms for rendering.
- `step-62us/`: independent replay with half the integration step; also passes.
- `rejected-refinement.json`: an earlier controller that fell when the step was refined.
  Replay it with `--parameters` to reproduce the failed gate; it is not the final candidate.

Each positive summary records the URDF and recording SHA-256. The GIF uses
`step-125us/rollout.json`; no root positions or orientations were imposed during
simulation. Only the renderer loads recorded poses after simulation has finished.
The GIF labels its 139 N m knee ceiling and disabled self-collision.
The regression test independently simulates the pinned controller and compares
the complete recording hash. Regeneration on a different numerical platform may
require re-evaluating numerical reproducibility rather than replacing a checksum.

Search used SciPy differential evolution (seed 20260921), followed by local
sweeps of flight pose, landing feedback, and explicitly recorded constraint
settings. No policy network, reward training, pretrained action data, or external
base wrench was used. Simulator settings are part of this benchmark's contract,
not inferred hardware parameters. Self-collision is disabled.

## Stricter screening

`screening/` contains dynamic replays of the same candidate with independently
changed constraints and combined G1/G1 EDU partial-specification profiles.
The full settings are embedded in each summary. Reproduce any summary with
`--parameters <summary.json> --generations 0 --output <directory>`.
The stricter combined profiles fail; the original GIF is not hardware evidence.
Only `control500hz.json` passes, with every other original benchmark assumption
retained. See the main document for the public specification sources and unknowns.

### Bounded EDU search results

- `optimized-launch.json`: two DE generations over seven free launch parameters
  (13-value controller, seed 20260921) found an upward takeoff COM velocity of
  about 1.98 m/s and pitch angular momentum about -19.78 N m s. This is a
  launch-only diagnostic; its complete-backflip `passed` flag is false.
- `optimized-launch-full.json`: continuing that candidate fails at about
  0.711 s on knee–torso self-contact. It does not complete a backflip.
- `flight-search-seed.json`: the earlier launch candidate used to initialize
  the separate flight search.
- `arm-spread-full.json`: a persisted 14-parameter candidate, including arm
  spread, avoids self-contact until about 0.680 s but then hits the floor with
  its hands after about 143 degrees of backward rotation. It fails.

The arm-spread search requested two generations but terminated with exit 143
before writing its final summary; it is not reported as a completed search.
The saved best candidate was then replayed separately to completion of the
failure gate. These limited searches do not establish infeasibility. All four
summaries can be replayed with `--parameters`; `--stage flip` continues a
launch-only summary through the full maneuver. Recording hashes in these
summaries identify reproducible output; large failed-rollout recordings are
kept out of the repository.
