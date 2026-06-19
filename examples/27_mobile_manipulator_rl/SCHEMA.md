# Mobile Manipulator RL Artifact Schemas

This directory uses small, versioned artifacts so training runs can be resumed,
compared, and used as regression gates without depending on external packages.
The schemas below are example-level contracts for
`examples/27_mobile_manipulator_rl`; they are not yet global RNE benchmark
formats. The policy artifact schema descriptors and hashes are defined in
`policy_schema.py`; the rollout CSV column order is defined in
`rollout_schema.py`. Both are consumed by `train.py`, `compare_reports.py`, and
the regression tests.

All JSON files are UTF-8, pretty-printed, newline-terminated, and written through
a temporary file followed by atomic replace where the current tool owns the
writer. Relative artifact paths are resolved from the JSON file that contains
them.

## Versioning

Each machine-readable artifact carries an explicit `schema_version`. Readers
reject unknown schema or algorithm IDs when the artifact is consumed as
executable state, such as policy loading or CEM checkpoint resume. The policy
artifact and report manifest are currently `schema_version: 2`; the other
artifacts in this directory are currently `schema_version: 1`.

Derived artifacts such as `rollout.svg`, `rollout.html`, `rollout_house.gif`,
`rollout_house.json`, and `index.html` are not canonical. They can be regenerated
from the JSON/CSV artifacts.

## CEM Training Checkpoint

Written by:

```bash
python train.py --checkpoint cem_checkpoint.json
```

Purpose: resume an interrupted CEM training run exactly at the next unfinished
iteration.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `algorithm` | string | `rne_mobile_manipulator_cem_reach_v1` |
| `next_iteration` | integer | next CEM iteration to execute |
| `target_iterations` | integer | total requested iterations for the saved run |
| `population` | integer | CEM population size |
| `elite` | integer | selected elite count |
| `seed` | integer | Python CEM sampler seed |
| `param_dim` | integer | policy parameter count, currently `4` |
| `targets_per_candidate` | integer | randomized targets per candidate |
| `episode_steps` | integer | max rollout steps per target |
| `mean` | array[number] | current CEM mean |
| `std` | array[number] | current CEM standard deviation |
| `best_params` | array[number] | best policy parameters seen so far |
| `best_reward` | number | best mean reward seen so far |
| `history` | array[array] | per-iteration `(iteration, mean_reward, best_reward)` rows |
| `rng_state` | array | JSON form of Python `random.Random` state |

Compatibility notes:

- `--resume` rejects mismatched `population`, `elite`, `param_dim`,
  `targets_per_candidate`, and `episode_steps`.
- `--iterations` on resume is a total target count, not an additional count.
- The checkpoint is trainer state, not a simulation replay. It does not replace
  `rne_py.VectorizedMobileManipulatorEnv` checkpoints.

## Policy Artifact

Written by:

```bash
python train.py --policy-out best_policy.json
python train.py --report-dir reports/reach
python compare_reports.py reports --best-policy-out reports/best_policy.json
python compare_reports.py reports --best-house-gif-out reports/best_rollout_house.gif --best-house-gif-metadata-out reports/best_rollout_house.json
```

Purpose: portable learned linear policy for evaluation or report generation.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `2` |
| `algorithm` | string | `rne_mobile_manipulator_linear_reach_policy_v1` |
| `observation_schema_hash` | string | SHA-256 hash of `observation_schema` |
| `action_schema_hash` | string | SHA-256 hash of `action_schema` |
| `observation_schema` | object | full observation descriptor expected by the policy |
| `action_schema` | object | full action descriptor expected by the policy |
| `policy_features` | array[string] | observation fields read by the linear policy |
| `policy_outputs` | array[string] | action fields written by the linear policy |
| `normalization` | object | input normalization contract, currently identity |
| `action_scaling` | object | output scaling/clipping contract |
| `task_compatibility` | object | compatible RNE task family and task names |
| `engine_compatibility` | object | Python/RNE API expected by the artifact |
| `param_dim` | integer | policy parameter count, currently `4` |
| `action_limit_rad_s` | number | shoulder/elbow action clipping limit |
| `params` | array[number] | linear policy parameters |
| `best_reward` | number | best training reward associated with the policy |
| `training_iterations` | integer | number of CEM iterations behind the artifact |

Compatibility notes:

- `train.py --policy-in` rejects unknown schema, algorithm, `param_dim`, or
  `action_limit_rad_s`; it also rejects observation/action schema hash,
  feature/output, normalization, scaling, task, or engine compatibility
  mismatches.
- `compare_reports.py` validates that a report's `policy.json` exists, contains
  the required fields, matches the manifest's policy schema, algorithm, and
  schema hashes, has embedded observation/action descriptors matching those
  hashes, has `params` length equal to `param_dim`, and agrees with the
  manifest's `best_reward` and `training_iterations`.
- The policy format is intentionally narrow. It is not a generic neural-network
  weight container.

## Rollout CSV

Written by:

```bash
python train.py --rollout-csv rollout.csv
python train.py --rollout-csv rollout.csv --rollout-house-gif rollout_house.gif --rollout-house-gif-metadata rollout_house.json
python train.py --report-dir reports/reach
```

Purpose: canonical per-step trajectory data for plotting and HTML replay.
`render_house_gif.py` also consumes the same CSV to create a derived animated
GIF of the mobile manipulator moving through a small house scene. `train.py`
can write that derived GIF directly with `--rollout-house-gif`, but the CSV
remains the canonical trajectory artifact.

`render_house_gif.py --metadata-out rollout_house.json` and
`train.py --rollout-house-gif-metadata rollout_house.json` write a derived
metadata JSON containing:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `artifact` | string | `rne_mobile_manipulator_house_gif` |
| `gif_path` | string | GIF path passed to `--out` or the default output path |
| `source` | object | `{kind: "demo"}`, `{kind: "demo", rollout_csv_path: ...}`, or `{kind: "rollout_csv", path: ...}` |
| `sample_count` | integer | samples read before frame selection |
| `frame_count` | integer | encoded image frames |
| `max_frames` | integer | requested maximum encoded frames |
| `width` | integer | GIF logical screen width |
| `height` | integer | GIF logical screen height |
| `fps` | number | requested playback frames per second |
| `byte_size` | integer | GIF file size in bytes |
| `sha256` | string | GIF content digest as `sha256:<hex>` |

`render_house_gif.py --verify-metadata rollout_house.json` validates the metadata
schema, resolves `gif_path` relative to the metadata JSON when needed, checks that
the GIF is structurally well-formed, and verifies `width`, `height`,
`frame_count`, `byte_size`, and `sha256` against the GIF bytes.

Header fields:

```text
step,base_x,base_y,base_yaw,ee_x,ee_y,ee_z,target_dx,target_dy,target_dz,shoulder_action,elbow_action,reward,total_reward,done
```

Semantics:

- `compare_reports.py` requires this exact header and rejects missing, extra, or
  reordered columns.
- `render_house_gif.py --demo --demo-rollout-csv path.csv` writes the same
  canonical CSV header for its built-in synthetic trajectory.
- `step` is zero-based within the recorded rollout.
- `step` values must be contiguous zero-based integers.
- `base_*` and `ee_*` are the observation before applying that row's action.
- `target_d*` is the target offset in the observation used by the policy.
- `reward` is the reward delta produced by the action.
- `total_reward` is the episode return after the action.
- `total_reward` must equal the cumulative sum of `reward` through that row.
- `done` is the post-step termination flag and must be `true` or `false`.
- No rows may appear after the first `done=true` row.

## Report Manifest

Written by:

```bash
python train.py --report-dir reports/reach
```

Purpose: index one report bundle and provide the fields needed for leaderboard
ranking.

Required fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `2` |
| `policy_algorithm` | string | policy algorithm ID |
| `policy_schema_version` | integer | policy schema version used by `policy.json` |
| `observation_schema_hash` | string | hash of the policy observation schema |
| `action_schema_hash` | string | hash of the policy action schema |
| `rollout_schema_version` | integer | rollout CSV schema version |
| `rollout_schema_hash` | string | hash of the rollout CSV schema |
| `best_reward` | number | best training reward |
| `training_iterations` | integer | CEM iterations behind the policy |
| `rollout_rows` | integer | number of rows in `rollout.csv` |
| `final_total_reward` | number | final total reward from the recorded rollout |
| `final_target_error` | number | final target error from the recorded rollout |
| `artifacts` | object | relative artifact paths |
| `rollout_house_gif` | object | optional derived GIF metadata when `artifacts.rollout_house_gif` is present |

Current `artifacts` keys:

| key | default path | meaning |
|---|---|---|
| `index` | `index.html` | report landing page |
| `policy` | `policy.json` | policy artifact |
| `rollout_csv` | `rollout.csv` | canonical rollout data |
| `rollout_svg` | `rollout.svg` | derived static visualization |
| `rollout_html` | `rollout.html` | derived interactive replay |
| `rollout_house_gif` | `rollout_house.gif` | optional derived house-scene GIF |
| `rollout_house_gif_metadata` | `rollout_house.json` | optional checksum/provenance JSON for `rollout_house_gif` |

`compare_reports.py` requires `index`, `policy`, and `rollout_csv` to be
present and to point at existing files. All artifact paths must be non-empty
relative paths that stay inside the report directory; absolute paths and `../`
escapes are rejected. It also validates the manifest's rollout schema
version/hash, reloads `rollout_csv`, and verifies `rollout_rows`,
`final_total_reward`, and `final_target_error` against the manifest. Optional
derived artifacts such as `rollout_house_gif` are path-validated when present.
House GIF artifacts are also checked for a valid GIF header, non-empty logical
screen, well-formed block structure, trailer, and at least one image frame. They
are not required for leaderboard ranking. When `rollout_house_gif` metadata is
present, `compare_reports.py` also verifies `width`, `height`, and
`frame_count` against the GIF file and checks that `frame_count <= max_frames`
and `fps > 0`. It also verifies `byte_size` and `sha256` against the GIF bytes.
When `artifacts.rollout_house_gif_metadata` is present, `compare_reports.py`
loads `rollout_house.json`, verifies that it points at the report's
`rollout_house.gif` and `rollout.csv`, and checks its dimensions, frame count,
sample count, FPS, byte size, and SHA-256 against the manifest metadata.

Current `rollout_house_gif` metadata fields:

| field | type | meaning |
|---|---|---|
| `width` | integer | GIF logical screen width |
| `height` | integer | GIF logical screen height |
| `max_frames` | integer | requested maximum encoded frames |
| `frame_count` | integer | actual encoded image frames |
| `sample_count` | integer | source rollout samples before frame selection |
| `fps` | number | requested playback frames per second |
| `byte_size` | integer | GIF file size in bytes |
| `sha256` | string | GIF content digest as `sha256:<hex>` |

## Leaderboard JSON

Written by:

```bash
python compare_reports.py reports --json reports/leaderboard.json
python sweep.py --out reports/sweep
```

Purpose: machine-readable ranked results across report bundles.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `reports_considered` | integer | number of report manifests ranked |
| `ranking` | array[object] | ordered tie-breaking criteria |
| `reports` | array[object] | ranked report rows |

Ranking criteria:

1. `final_target_error` ascending
2. `final_total_reward` descending
3. `best_reward` descending
4. `report` ascending

Each `reports` row contains:

| field | type | meaning |
|---|---|---|
| `rank` | integer | one-based rank |
| `report` | string | report directory name |
| `policy_algorithm` | string | policy algorithm ID |
| `policy_schema_version` | integer | policy artifact schema version |
| `observation_schema_hash` | string | hash of the policy observation schema |
| `action_schema_hash` | string | hash of the policy action schema |
| `rollout_schema_version` | integer | rollout CSV schema version |
| `rollout_schema_hash` | string | hash of the rollout CSV schema |
| `final_target_error` | number | ranking primary metric |
| `final_total_reward` | number | ranking secondary metric |
| `best_reward` | number | training best reward |
| `training_iterations` | integer | CEM iterations behind the policy |
| `rollout_rows` | integer | rollout CSV rows |
| `index_path` | string | path relative to the leaderboard JSON directory |
| `policy_path` | string | path relative to the leaderboard JSON directory |
| `rollout_house_gif_path` | string or null | optional path relative to the leaderboard JSON directory |
| `rollout_house_gif_metadata_path` | string or null | optional metadata JSON path relative to the leaderboard JSON directory |
| `rollout_house_gif_width` | integer or null | optional GIF logical screen width |
| `rollout_house_gif_height` | integer or null | optional GIF logical screen height |
| `rollout_house_gif_frames` | integer or null | optional GIF image frame count |
| `rollout_house_gif_sample_count` | integer or null | optional source rollout samples before frame selection |
| `rollout_house_gif_fps` | number or null | optional GIF playback FPS from report metadata |
| `rollout_house_gif_byte_size` | integer or null | optional GIF file size in bytes |
| `rollout_house_gif_sha256` | string or null | optional GIF content digest |
| `manifest_path` | string | path relative to the leaderboard JSON directory |

## Best Report JSON

Written by:

```bash
python compare_reports.py reports --best-report-out reports/best_report.json
python sweep.py --out reports/sweep
```

Purpose: compact machine-readable pointer to the selected winning report.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `reports_considered` | integer | number of report manifests ranked |
| `ranking` | array[object] | same criteria as `leaderboard.json` |
| `best` | object | same row shape as one `leaderboard.json.reports` entry |

`best` is intentionally the same payload as `leaderboard.json.reports[0]` when
both files are written to the same directory.

`compare_reports.py --best-house-gif-out` copies the top-ranked report's
`rollout_house_gif` artifact to a stable GIF path. Add
`--best-house-gif-metadata-out` to write a metadata JSON with the same
`rne_mobile_manipulator_house_gif` artifact shape used by
`render_house_gif.py --metadata-out`, except `source.kind` is `best_report` and
the source object points at the winning manifest and original report GIF. The
command fails if the winning report has no house GIF artifact, or if the copied
GIF checksum differs from the manifest metadata.

## Sweep Outputs

`sweep.py --out reports/sweep` writes:

| path | canonical | meaning |
|---|---:|---|
| `seed_XXXX/checkpoint.json` | yes | CEM resume state for that seed |
| `seed_XXXX/manifest.json` | yes | report manifest for that seed |
| `seed_XXXX/policy.json` | yes | policy artifact for that seed |
| `seed_XXXX/rollout.csv` | yes | rollout data for that seed |
| `leaderboard.csv` | yes | tabular leaderboard |
| `leaderboard.json` | yes | machine-readable full leaderboard |
| `best_policy.json` | yes | copied policy from the winning report |
| `best_report.json` | yes | metrics and paths for the winning report |
| `best_rollout_house.gif` | no | copied GIF from the winning report when GIF reports are enabled |
| `best_rollout_house.json` | yes | checksum/provenance JSON for `best_rollout_house.gif` when GIF reports are enabled |
| `sweep_manifest.json` | yes | sweep configuration, expected inputs, outputs, and gates |
| `leaderboard.html` | no | derived leaderboard page |
| `seed_XXXX/index.html` | no | derived report landing page |
| `seed_XXXX/rollout.svg` | no | derived static rollout visualization |
| `seed_XXXX/rollout.html` | no | derived interactive rollout replay |
| `seed_XXXX/rollout_house.gif` | no | optional derived house-scene GIF |
| `seed_XXXX/rollout_house.json` | yes | optional checksum/provenance JSON for `seed_XXXX/rollout_house.gif` |

The sweep compares only the expected `seed_XXXX/manifest.json` files for the
requested `--runs` and `--seed-start`. Stale reports elsewhere under `--out` do
not affect ranking or quality gates.

## Sweep Manifest JSON

Written by:

```bash
python sweep.py --out reports/sweep
```

Purpose: record the sweep configuration and provenance after a successful
training and compare pass.

Top-level fields:

| field | type | meaning |
|---|---|---|
| `schema_version` | integer | `1` |
| `runner` | string | runner path, currently `examples/27_mobile_manipulator_rl/sweep.py` |
| `config` | object | requested sweep parameters |
| `seeds` | array[integer] | expected seed values for this sweep |
| `quality_gates` | object | gate thresholds passed to `compare_reports.py` |
| `expected_manifests` | array[string] | manifest paths relative to `sweep_manifest.json` |
| `outputs` | object | sweep output paths relative to `sweep_manifest.json` |
| `commands` | object | train and compare commands used by the run |

`sweep_manifest.json` is written only after `compare_reports.py` succeeds. In
`--dry-run` mode the command lines are printed, but the manifest is not written.
When `config.report_house_gif` is true, `config` also records
`report_house_gif_width`, `report_house_gif_height`,
`report_house_gif_max_frames`, and `report_house_gif_fps`,
`outputs.best_house_gif` points at the copied winning GIF, and
`outputs.best_house_gif_metadata` points at the copied GIF's checksum/provenance
JSON.

## Quality Gates

`compare_reports.py` and `sweep.py` can fail with a non-zero exit status when:

- fewer than `--require-reports-at-least` manifests are ranked;
- the winning `final_target_error` is greater than
  `--require-final-error-at-most`;
- the winning `final_total_reward` is less than
  `--require-final-reward-at-least`.

These gates are intended for short regression checks. Longer multi-seed learning
quality gates should use stable seed sets and aggregate metrics before being
promoted to CI.
