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
