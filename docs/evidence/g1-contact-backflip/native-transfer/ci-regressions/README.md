# CI regression investigation

Native validation does not replace the repository CI requirements. Run
[35685000529](https://github.com/rsasaki0109/RoboSim/actions/runs/35685000529)
checks producer commit `eed9b4152b12326b52831dba175fff1ea8f1d43e`; it is not a
final-head green result.

- The assets SemVer job flags `UrdfRobotAsset.weld_fixed_children`. The same
  field failed on PR base `407acbad5079aa93c22752612b985f689c033ae2` in
  [run 35480027356](https://github.com/rsasaki0109/RoboSim/actions/runs/35480027356),
  job 105996051648. That base also reported the articulation-config field and
  longitudinal-mobility fields against the immutable baseline. These require
  resolution; they are not waived by this investigation.
- The base's smoke stages passed. Current smoke 31 fails to carry its cube
  (`0.02 m`), and smoke 60 produces only `+0.098 rad` turn. Both failures are
  reproducible locally and remain regressions until the affected existing
  assertions pass. No smoke thresholds are lowered.
- A large changed-file list exposes another CI issue: `echo "$changed" |
  grep -q ...` under `pipefail` can return a writer SIGPIPE after an early grep
  match. The package is then incorrectly treated as unchanged. Using a
  here-string removes this false skip. Reproduced with an initial matching
  crate path followed by 10,000 source paths: the old filter skips, the fixed
  filter checks, and a genuinely unchanged crate still skips. Earlier fast
  SemVer successes must not be treated as proof that every changed crate ran.

Local builds reuse the release target and keep at least 30 GiB free. Full CI
runs remotely to avoid exhausting the local disk reserve. Native physics jobs
use immutable, hash-pinned producer executables and are independent of these
regression builds.

## Targeted repairs

- Lift fingers: retain velocity gain 30 and bound torque at 0.3 Nm. The
  production example 31 now carries 1.13 m and releases; all five `mm_lift`
  filtered tests and six `pick_place` tests pass, including bitwise checkpoint
  replay. Friction-mode finger limits remain configured by their own path.
- Go2 overlay: scale the historical pinned coefficients by 0.85. The unchanged
  sustained-turn/3e-9-perturbation/repeatability regression passes; production
  example 60 measures +0.288 rad and 2.86 m displacement, with both robots
  upright. No heading, displacement or height gate is lowered.
- Full crate and final-head CI results remain to be collected. The earlier
  failed workflow is retained as regression evidence, not represented as green.
