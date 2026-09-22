# Native landing feedback and recovery probes

30 deterministic, non-RL, direct-torque probes at 500 µs retain raw recordings
and per-case source/binary hashes in `sources.json`. All use the corrected
free-root COM plant and the same 34.13 kg foot-contact model.

Optional capture feedback alone did not hold landing: positive gains shortened
survival, and reversing the correction extended it only slightly. Pairwise
saturation preserves the equal/opposite hip and ankle correction within both
joint ranges. Default gain is zero. Lower early ankle pitch-rate gains also
failed. Increasing this gain to 1 s and extending recovery to 10 s produced a
held 5 s rollout; extending the same candidate to 15 s reached standing.

Earlier opening (5.9, 5.95, 6.0 rad) reduced peak joint speed but lost landing.
Deeper tuck with 6.0 rad opening also failed. These comparisons are retained
rather than treating lower overspeed or longer survival as qualification.

`com-audit.json` independently compares 375 native-frame whole-robot COM
positions with source-URDF forward kinematics. Maximum disagreement is
2.466e-7 m; external physics time remains zero.

The long recording and rendering manifest are in [held-recovery](../held-recovery/README.md).
The native full-body qualification flag remains false. The source revisions
are intentionally per-case: feedback support evolved during these probes.

Raw comparison rollouts are stored as deterministic `.json.gz` files to reduce
repository and disk usage. Search hashes refer to the decompressed JSON bytes;
`sources.json` additionally hashes the stored compressed artifacts.
